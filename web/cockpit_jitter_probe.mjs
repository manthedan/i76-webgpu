#!/usr/bin/env node
/*
 * H-UAT-059 deterministic cockpit-jitter diagnosis.
 *
 * Runs one fixed-input P09 drive on a straight road and on adjacent rough
 * terrain.  A controlled 60 Hz requestAnimationFrame clock samples the exact
 * camera consumed by the production page while its simulation remains fixed
 * at 20 Hz.  Cockpit/chase and software/WebGPU cases reload the same mission
 * and replay the same initial pose/input. H-UAT-069 additionally alternates
 * full-left/full-right steer every simulation tick and records the authoritative
 * car pose, rendered car basis, camera transform, and projected car centre.
 * The GPU arm additionally observes
 * each real queue.submit and compares a stable visible cockpit/car vertex
 * through the exact uploaded f32 VP/model/MVP stream against the SW-equivalent
 * interpolated double pose. The JSON receipt retains every render frame and
 * every distinct simulation tick; the summary reports mean magnitudes of
 * second differences for pose and submitted-matrix projection error.
 *
 * Usage: node web/cockpit_jitter_probe.mjs <new-json-output> <new-log-output>
 * Env: NITRO_APP (required), CHROME and I76_BUILD_ID (optional).
 * Build first with web/build.sh.
 */
import puppeteer from 'puppeteer';
import { appendFileSync, createReadStream, readdirSync, writeFileSync } from 'node:fs';
import { externalPath, newOutputFile } from './tests/external_paths.mjs';
import { createServer } from 'node:http';
import { extname, resolve, sep } from 'node:path';
import { fileURLToPath } from 'node:url';

const WEB_DIR = fileURLToPath(new URL('./', import.meta.url));
const APP_INPUT = process.env.NITRO_APP;
const JSON_OUT = process.argv[2];
const LOG_OUT = process.argv[3];
if (!APP_INPUT || !JSON_OUT || !LOG_OUT) {
  console.error('usage: NITRO_APP=/owned/app node cockpit_jitter_probe.mjs <new-json-output> <new-log-output>');
  process.exit(1);
}
const APP = externalPath(APP_INPUT, 'NITRO_APP', { existing: true }) + sep;
for (const output of [JSON_OUT, LOG_OUT]) newOutputFile(output);
const BUILD_SHA = process.env.I76_BUILD_ID || 'unversioned-source';
const CAMLERP = process.env.CAMLERP || 'extrap';
if (!['interp', 'extrap'].includes(CAMLERP))
  throw new Error(`CAMLERP must be interp or extrap, got ${CAMLERP}`);
const GPURES = process.env.GPURES || 'fidelity';
if (!['fidelity', 'native'].includes(GPURES))
  throw new Error(`GPURES must be fidelity or native, got ${GPURES}`);
const DT_RENDER_MS = 1000 / 60;
const FRAMES = 180;
const WARMUP_FRAMES = 12; // discard synthetic instant-20m/s placement edge
const FIXTURES = {
  straight: { x: 3350, z: 48100, yaw: Math.PI, speed: 20,
    note: 'P09 authored road ribbon north of the aaramp7 launch' },
  rough: { x: 3300, z: 48100, yaw: Math.PI, speed: 20,
    note: 'P09 terrain 39.8 m west of the same road ribbon' },
};
const VIEWS = { cockpit: 1, chase: 2 };
const FLAGS = [
  '--no-sandbox', '--disable-dev-shm-usage', '--enable-unsafe-webgpu',
  '--enable-webgpu-developer-features', '--use-angle=vulkan',
  '--disable-background-timer-throttling', '--disable-renderer-backgrounding',
  '--disable-backgrounding-occluded-windows',
];

const log = (...parts) => {
  const line = parts.map((p) => typeof p === 'string' ? p : JSON.stringify(p)).join(' ');
  console.log(line);
  appendFileSync(LOG_OUT, line + '\n');
};

function serveWeb() {
  const mime = {
    '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
    '.mjs': 'text/javascript; charset=utf-8', '.css': 'text/css; charset=utf-8',
    '.json': 'application/json; charset=utf-8', '.wasm': 'application/wasm',
    '.png': 'image/png', '.ico': 'image/x-icon', '.ttf': 'font/ttf',
  };
  const server = createServer((req, res) => {
    let pathname;
    try { pathname = decodeURIComponent(new URL(req.url, 'http://localhost').pathname); }
    catch { res.writeHead(400).end('bad request'); return; }
    if (pathname === '/') pathname = '/index.html';
    const file = resolve(WEB_DIR, '.' + pathname);
    if (file !== WEB_DIR && !file.startsWith(WEB_DIR.endsWith(sep) ? WEB_DIR : WEB_DIR + sep)) {
      res.writeHead(403).end('forbidden'); return;
    }
    const stream = createReadStream(file);
    stream.on('open', () => {
      res.writeHead(200, { 'Content-Type': mime[extname(file).toLowerCase()] ||
                                          'application/octet-stream',
                           'Cache-Control': 'no-store' });
      stream.pipe(res);
    });
    stream.on('error', () => res.writeHead(404).end('not found'));
  });
  return new Promise((ok, reject) => {
    server.once('error', reject);
    server.listen(0, '127.0.0.1', () => {
      const { port } = server.address();
      ok({ server, url: `http://127.0.0.1:${port}/index.html` });
    });
  });
}

async function installFixedFrameClock(page) {
  await page.evaluateOnNewDocument(() => {
    const nativeRaf = window.requestAnimationFrame.bind(window);
    const nativeCancel = window.cancelAnimationFrame.bind(window);
    let enabled = false, now = 0, nextId = 1;
    const queued = [], cancelled = new Set(), nativePending = new Map();
    window.requestAnimationFrame = (callback) => {
      const id = nextId++;
      if (enabled) queued.push({ id, callback });
      else {
        const nativeId = nativeRaf((t) => {
          nativePending.delete(id);
          if (!cancelled.delete(id)) callback(t);
        });
        nativePending.set(id, { nativeId, callback });
      }
      return id;
    };
    window.cancelAnimationFrame = (id) => {
      const pending = nativePending.get(id);
      if (pending) { nativeCancel(pending.nativeId); nativePending.delete(id); }
      else cancelled.add(id);
    };
    window.__jitterClock = {
      enable() {
        enabled = true; now = performance.now();
        for (const [id, pending] of nativePending) {
          nativeCancel(pending.nativeId);
          queued.push({ id, callback: pending.callback });
        }
        nativePending.clear();
      },
      pending() { return queued.length; },
      pump(dt) {
        now += dt;
        const batch = queued.splice(0);
        for (const item of batch)
          if (!cancelled.delete(item.id)) item.callback(now);
        return { callbacks: batch.length, now };
      },
    };
  });
}

function cameraAngles(camera) {
  const f = camera.forward, r = camera.right;
  const pitch = Math.asin(Math.max(-1, Math.min(1, f[1])));
  const yaw = Math.atan2(-f[0], f[2]);
  const sy = Math.sin(yaw), cy = Math.cos(yaw);
  const sp = Math.sin(pitch), cp = Math.cos(pitch);
  const baseR = [cy, 0, sy];
  const baseU = [sy * sp, cp, -cy * sp];
  const dot = (a, b) => a.reduce((n, v, i) => n + v * b[i], 0);
  const roll = Math.atan2(dot(r, baseU), dot(r, baseR));
  return { yaw, pitch, roll };
}

const unwrap = (values) => {
  if (!values.length) return [];
  const out = [values[0]];
  for (let i = 1; i < values.length; i++) {
    let v = values[i], d = v - out[i - 1];
    while (d > Math.PI) { v -= 2 * Math.PI; d -= 2 * Math.PI; }
    while (d < -Math.PI) { v += 2 * Math.PI; d += 2 * Math.PI; }
    out.push(v);
  }
  return out;
};
const secondVector = (rows, keys) => {
  const cols = keys.map((k) => unwrap(rows.map((r) => r[k])));
  const out = [];
  for (let i = 2; i < rows.length; i++)
    out.push(Math.hypot(...cols.map((c) => c[i] - 2 * c[i - 1] + c[i - 2])));
  return out;
};
const mean = (xs) => xs.length ? xs.reduce((a, v) => a + v, 0) / xs.length : 0;
const pearson = (a, b) => {
  const n = Math.min(a.length, b.length);
  if (n < 3) return null;
  const aa = a.slice(-n), bb = b.slice(-n), ma = mean(aa), mb = mean(bb);
  let num = 0, da = 0, db = 0;
  for (let i = 0; i < n; i++) {
    const x = aa[i] - ma, y = bb[i] - mb;
    num += x * y; da += x * x; db += y * y;
  }
  return da > 0 && db > 0 ? num / Math.sqrt(da * db) : null;
};

function summarize(frames, ticks) {
  const cameraRows = frames.map((f) => ({
    x: f.camera.eye[0], y: f.camera.eye[1], z: f.camera.eye[2],
    ...f.cameraAngles,
  }));
  const tickCameraRows = ticks.map((f) => ({
    x: f.camera.eye[0], y: f.camera.eye[1], z: f.camera.eye[2],
    ...f.cameraAngles,
  }));
  const framePos2 = secondVector(cameraRows, ['x', 'y', 'z']);
  const framePosAxes = Object.fromEntries(['x', 'y', 'z'].map((axis) =>
    [axis, mean(secondVector(cameraRows, [axis]))]));
  const frameAng2 = secondVector(cameraRows, ['yaw', 'pitch', 'roll']);
  const frameAngAxes = Object.fromEntries(['yaw', 'pitch', 'roll'].map((axis) =>
    [axis, mean(secondVector(cameraRows, [axis]))]));
  const tickPos2 = secondVector(tickCameraRows, ['x', 'y', 'z']);
  const tickAng2 = secondVector(tickCameraRows, ['yaw', 'pitch', 'roll']);
  let interRepeat = 0, interCount = 0, tickStep = 0;
  for (let i = 1; i < frames.length; i++) {
    const a = cameraRows[i - 1], b = cameraRows[i];
    const d = Math.hypot(b.x - a.x, b.y - a.y, b.z - a.z,
                         b.yaw - a.yaw, b.pitch - a.pitch, b.roll - a.roll);
    if (frames[i].tick === frames[i - 1].tick) {
      interCount++;
      if (d < 1e-10) interRepeat++;
    } else if (d >= 1e-10) tickStep++;
  }
  const attitudeRows = ticks.map((t) => ({
    pitch: t.pose.pitch, roll: t.pose.roll,
  }));
  /* car.c applies current = previous + 0.25*(terrainTarget-previous) while
   * grounded. Reconstruct that terrain-normal input without adding a mutable
   * instrumentation hook to simulation. */
  const terrainTargets = attitudeRows.map((v, i) => {
    if (!i || !ticks[i].diag.follow) return v;
    const p = attitudeRows[i - 1];
    return { pitch: p.pitch + 4 * (v.pitch - p.pitch),
             roll: p.roll + 4 * (v.roll - p.roll) };
  });
  const attitude2 = secondVector(attitudeRows, ['pitch', 'roll']);
  const terrainTarget2 = secondVector(terrainTargets, ['pitch', 'roll']);
  const ground2 = secondVector(ticks.map((t) => ({ y: t.diag.groundY })), ['y']);
  const yaw2 = secondVector(ticks.map((t) => ({ yaw: t.pose.yaw })), ['yaw']);
  const gpuRows = frames.filter((f) => f.gpuMatrix).map((f) => ({
    x: f.gpuMatrix.submittedPx[0], y: f.gpuMatrix.submittedPx[1],
    ex: f.gpuMatrix.errorPx[0], ey: f.gpuMatrix.errorPx[1],
  }));
  const gpuError2 = secondVector(gpuRows, ['ex', 'ey']);
  const gpuMotion2 = secondVector(gpuRows, ['x', 'y']);
  const gpuErrors = frames.filter((f) => f.gpuMatrix)
    .map((f) => f.gpuMatrix.errorMagnitudePx);
  const carScreenRows = frames.map((f) => ({ x: f.renderCarScreen[0] }));
  const carScreenSteps = frames.slice(1).map((f, i) => ({
    frame: f.frame, tick: f.tick, steer: f.commandSteer,
    dx: f.renderCarScreen[0] - frames[i].renderCarScreen[0],
  }));
  const carScreenSecond = secondVector(carScreenRows, ['x']);
  const peakCarStep = carScreenSteps.reduce((best, row) =>
    !best || Math.abs(row.dx) > Math.abs(best.dx) ? row : best, null);
  const peakCarSecond = carScreenSecond.reduce((best, value, i) =>
    !best || Math.abs(value) > Math.abs(best.value)
      ? { frame: frames[i + 2].frame, tick: frames[i + 2].tick,
          steer: frames[i + 2].commandSteer, value }
      : best, null);
  const authorityRows = ticks.map((f) => ({ x: f.pose.x, z: f.pose.z }));
  const authoritySteps = authorityRows.slice(1).map((f, i) =>
    Math.hypot(f.x - authorityRows[i].x, f.z - authorityRows[i].z));
  const authorityLateralSteps = authorityRows.slice(1).map((f, i) =>
    Math.abs(f.x - authorityRows[i].x));
  const authoritySecond = secondVector(authorityRows, ['x', 'z']);
  return {
    renderFrames: frames.length, simTicks: ticks.length,
    repeatedBetweenTicks: interRepeat, betweenTickIntervals: interCount,
    repeatFraction: interCount ? interRepeat / interCount : 0,
    nonzeroTickSteps: tickStep,
    jitter: {
      perFrame: { position: mean(framePos2), orientation: mean(frameAng2),
                  positionAxes: framePosAxes, orientationAxes: frameAngAxes },
      perTick: { position: mean(tickPos2), orientation: mean(tickAng2) },
    },
    gpuSubmittedMatrix: gpuRows.length ? {
      samples: gpuRows.length,
      meanProjectionErrorPx: mean(gpuErrors),
      maxProjectionErrorPx: Math.max(...gpuErrors),
      jitterErrorPxPerFrame2: mean(gpuError2),
      motionPxPerFrame2: mean(gpuMotion2),
    } : null,
    horizontalCar: {
      startPx: frames[0]?.renderCarScreen[0],
      endPx: frames.at(-1)?.renderCarScreen[0],
      meanAbsSecondPxPerFrame2: mean(carScreenSecond.map(Math.abs)),
      peakStep: peakCarStep,
      peakSecond: peakCarSecond,
      authorityPeakStepMPerTick: Math.max(...authoritySteps),
      authorityPeakLateralStepMPerTick: Math.max(...authorityLateralSteps),
      authorityPeakSecondMPerTick2: Math.max(...authoritySecond),
    },
    presentation: {
      modes: [...new Set(frames.map((f) => f.presentation?.mode).filter(Boolean))],
      clampFrames: frames.filter((f) => f.presentation?.clamp).length,
      clampReasons: Object.fromEntries([...new Set(frames.map(
        (f) => f.presentation?.clamp || 0))].map((reason) => [reason,
          frames.filter((f) => (f.presentation?.clamp || 0) === reason).length])),
      colliderFrames: frames.filter((f) => f.diag?.collider >= 0).length,
      colliders: [...new Set(frames.map((f) => f.diag?.collider)
        .filter((v) => v >= 0))],
      velocityConstrainedFrames: frames.filter(
        (f) => f.presentation?.velocityConstrained).length,
    },
    correlation: {
      cameraOrientationVsAttitude: pearson(tickAng2, attitude2),
      cameraOrientationVsTerrainTarget: pearson(tickAng2, terrainTarget2),
      cameraPositionVsGround: pearson(tickPos2, ground2),
      cameraOrientationVsYaw: pearson(tickAng2, yaw2),
    },
    roadClearanceStart: frames[0]?.roadClearance,
    roadClearanceEnd: frames.at(-1)?.roadClearance,
  };
}

let browser, server;
const pageErrors = [];
try {
  const served = await serveWeb(); server = served.server;
  log(`[probe] build=${BUILD_SHA} browser-clock=60Hz sim=20Hz frames=${FRAMES} gpures=${GPURES}`);
  browser = await puppeteer.launch({
    headless: 'new', protocolTimeout: 900000,
    ...(process.env.CHROME ? { executablePath: process.env.CHROME } : {}),
    args: FLAGS,
  });
  log(`[probe] browser=${await browser.version()} flags=${FLAGS.join(' ')}`);
  const page = await browser.newPage();
  await installFixedFrameClock(page);
  await page.setViewport({ width: 1000, height: 1000,
                           deviceScaleFactor: GPURES === 'native' ? 2 : 1 });
  page.on('console', (m) => { if (m.type() === 'error') log(`[browser:error] ${m.text()}`); });
  page.on('pageerror', (e) => {
    pageErrors.push(String(e.stack || e.message));
    log(`[PAGEERROR] ${e.stack || e.message}`);
  });
  await page.goto(served.url + `?dev=1&renderer=sw&camlerp=${CAMLERP}&gpures=${GPURES}`,
                  { waitUntil: 'load', timeout: 120000 });
  await page.waitForFunction("!document.getElementById('zixFile').disabled", { timeout: 120000 });
  const auto = await page.waitForFunction(
    () => document.documentElement.dataset.i76Restore !== undefined,
    { timeout: 240000 }).then(() =>
      page.evaluate(() => document.documentElement.dataset.i76Restore));
  log(`[probe] asset restore mode=${auto}`);
  if (auto === 'none') {
    const stageOne = async (selector, files, source) => {
      await (await page.$(selector)).uploadFile(...files);
      await page.waitForFunction((s) => new RegExp(s).test(
        document.getElementById('stagedInfo').textContent),
        { timeout: 400000 }, source);
    };
    await page.$eval('#assetManual', (d) => { d.open = true; });
    const miss = readdirSync(APP + 'miss8').map((f) => APP + 'miss8/' + f);
    const fnts = readdirSync(APP).filter((f) => /\.fnt$/i.test(f)).map((f) => APP + f);
    await stageOne('#zixFile', [APP + 'nitro.zix'], 'need nitro\\.zfs');
    await stageOne('#zfsFile', [APP + 'nitro.zfs'], 'Nitro Pack archive ready');
    await stageOne('#fntFile', fnts, '[1-9]\\d* fonts');
    await stageOne('#campFile', [APP + 'addon/SCENARIO.DAT'], '· campaign');
    await stageOne('#dirInput', miss, '[1-9]\\d* mission files');
    await page.click('#loadBtn');
  }
  await page.waitForFunction("!document.getElementById('shellPanel').hidden", { timeout: 180000 });
  await page.waitForFunction(() => window.__i76?.M, { timeout: 30000 });
  await page.waitForFunction(() => window.__i76.frames > 0,
                             { polling: 10, timeout: 10000 });
  await page.evaluate(() => window.__jitterClock.enable());
  await page.waitForFunction(() => window.__jitterClock.pending() === 1, { timeout: 10000 });

  /* Enter drive mode through the shipped shell once. The unlocked first row
   * is P01; every measured case below immediately reloads P09 through the
   * same Wasm drive owner before placing the deterministic fixture. */
  await page.click('#missionList .missionRow:nth-child(1)');
  await page.waitForFunction("!document.getElementById('shellBrief').hidden", { timeout: 30000 });
  const title = await page.$eval('#briefTitle', (e) => e.textContent);
  if (!/New Hope/i.test(title)) throw new Error(`first mission is not New Hope: ${title}`);
  await page.click('#shellStartBtn');
  await page.evaluate((dt) => window.__jitterClock.pump(dt), DT_RENDER_MS);
  await page.waitForFunction("!document.getElementById('gamePanel').hidden", { timeout: 10000 });

  const runCase = async (backend, surface, view, steering = 'straight') => page.evaluate(
    async ({ backend, fixture, preset, frames, dt, warmupFrames, steering }) => {
      const M = window.__i76.M;
      const pose = () => JSON.parse(M.ccall('web_drive_pose', 'string', [], []));
      const camera = () => JSON.parse(M.ccall('web_drive_camera_pose', 'string', [], []));
      const tickCamera = () => JSON.parse(M.ccall('web_drive_camera_tick_pose', 'string', [], []));
      const diag = () => JSON.parse(M.ccall('web_drive_step_diag', 'string', [], []));
      const presentation = () => JSON.parse(M.ccall(
        'web_drive_presentation_state', 'string', [], []));
      const f64 = (ptr, n) => Array.from(new Float64Array(
        M.wasmMemory.buffer, ptr, n));
      const xformToMat4 = (t) => [
        t[0], t[1], t[2], 0, t[3], t[4], t[5], 0,
        t[6], t[7], t[8], 0, t[9], t[10], t[11], 1,
      ];
      const mat4mul = (a, b) => {
        const out = new Array(16).fill(0);
        for (let col = 0; col < 4; col++)
          for (let row = 0; row < 4; row++)
            for (let k = 0; k < 4; k++)
              out[col * 4 + row] += a[k * 4 + row] * b[col * 4 + k];
        return out;
      };
      const cameraVp = (c, origin = [0, 0, 0]) => {
        const e = c.eye.map((v, i) => v - origin[i]);
        const r = c.right, u = c.up, f = c.forward;
        const near = 0.25, far = 3000, alpha = far / (far - near);
        const V = [r[0], u[0], f[0], 0, r[1], u[1], f[1], 0,
                   r[2], u[2], f[2], 0,
                   -(r[0]*e[0]+r[1]*e[1]+r[2]*e[2]),
                   -(u[0]*e[0]+u[1]*e[1]+u[2]*e[2]),
                   -(f[0]*e[0]+f[1]*e[1]+f[2]*e[2]), 1];
        const P = [1/c.fov_tan_half, 0, 0, 0, 0,
                   (640/480)/c.fov_tan_half, 0, 0, 0, 0, alpha, 1,
                   0, 0, -far*near/(far-near), 0];
        return mat4mul(P, V);
      };
      const project = (vp, model, point, f32) => {
        const mul = (m, v) => {
          const out = [];
          for (let row = 0; row < 4; row++) {
            let s = f32 ? Math.fround(0) : 0;
            for (let k = 0; k < 4; k++) {
              const p = f32 ? Math.fround(m[k*4+row] * v[k])
                            : m[k*4+row] * v[k];
              s = f32 ? Math.fround(s + p) : s + p;
            }
            out.push(s);
          }
          return out;
        };
        const world = mul(model, [point[0], point[1], point[2], 1]);
        const clip = mul(vp, world);
        return [(clip[0] / clip[3] + 1) * 320,
                (1 - clip[1] / clip[3]) * 240, clip[3]];
      };
      let gpuAnchor = null;
      const gpuMatrixSample = (c, submission) => {
        if (!submission) return null;
        const n = M._web_gpu_drawlist_build();
        const candidates = submission.models.filter((m) =>
          m.kind === (backend === 'gpu' && preset === 1 ? 'interior' : 'car'));
        let chosen = null;
        const inspect = (sm, vertexIndex = null) => {
          if (sm.drawIndex < 0 || sm.drawIndex >= n) return null;
          const xp = M._web_gpu_drawlist_model(sm.drawIndex);
          const mp = M._web_gpu_drawlist_mesh(sm.drawIndex);
          if (!xp || !mp) return null;
          const canonicalModel = xformToMat4(f64(xp, 12));
          const nv = M._web_gpu_mesh_num_verts(mp);
          const vp = M._web_gpu_mesh_verts(mp);
          if (!(nv > 0) || !vp) return null;
          const verts = new Float32Array(M.wasmMemory.buffer, vp, nv * 3);
          const first = vertexIndex === null ? 0 : vertexIndex;
          const last = vertexIndex === null ? nv : vertexIndex + 1;
          let best = null;
          for (let vi = first; vi < last; vi++) {
            const point = [verts[vi*3], verts[vi*3+1], verts[vi*3+2]];
            const expectedPx = project(cameraVp(c), canonicalModel, point, false);
            if (vertexIndex === null &&
                (!(expectedPx[2] > 0.25) || expectedPx[0] < 20 || expectedPx[0] > 620 ||
                 expectedPx[1] < 20 || expectedPx[1] > 460)) continue;
            const score = Math.hypot(expectedPx[0] - 320, expectedPx[1] - 240);
            if (!best || score < best.score)
              best = { sm, vi, point, canonicalModel, expectedPx, score };
          }
          return best;
        };
        if (gpuAnchor) {
          const sm = submission.models.find((m) => m.drawIndex === gpuAnchor.drawIndex);
          if (sm) chosen = inspect(sm, gpuAnchor.vertexIndex);
        } else {
          for (const sm of candidates) {
            const sample = inspect(sm);
            if (sample && (!chosen || sample.score < chosen.score)) chosen = sample;
          }
          if (chosen)
            gpuAnchor = { drawIndex: chosen.sm.drawIndex, vertexIndex: chosen.vi };
        }
        if (!chosen) return null;
        const submittedPx = chosen.sm.mvp
          ? project(chosen.sm.mvp,
                    [1, 0, 0, 0, 0, 1, 0, 0,
                     0, 0, 1, 0, 0, 0, 0, 1], chosen.point, true)
          : project(submission.vp, chosen.sm.model, chosen.point, true);
        const errorPx = [submittedPx[0] - chosen.expectedPx[0],
                         submittedPx[1] - chosen.expectedPx[1]];
        return {
          serial: submission.serial, submitTimeMs: submission.submitTimeMs,
          worldOrigin: submission.worldOrigin, kind: chosen.sm.kind,
          drawIndex: chosen.sm.drawIndex, vertexIndex: chosen.vi,
          vp: submission.vp, model: chosen.sm.model,
          mvp: chosen.sm.mvp || null, point: chosen.point,
          expectedPx: chosen.expectedPx, submittedPx, errorPx,
          errorMagnitudePx: Math.hypot(...errorPx),
        };
      };
      const angles = (c) => {
        const f = c.forward, r = c.right;
        const pitch = Math.asin(Math.max(-1, Math.min(1, f[1])));
        const yaw = Math.atan2(-f[0], f[2]);
        const sy = Math.sin(yaw), cy = Math.cos(yaw), sp = Math.sin(pitch), cp = Math.cos(pitch);
        const br = [cy, 0, sy], bu = [sy * sp, cp, -cy * sp];
        const dot = (a, b) => a.reduce((n, v, i) => n + v * b[i], 0);
        return { yaw, pitch, roll: Math.atan2(dot(r, bu), dot(r, br)) };
      };
      if (M.ccall('web_drive_load', 'number', ['string', 'string'], ['miss8/P09.MSN', '']) !== 0)
        throw new Error('P09 reload failed');
      let pulsed = false;
      for (let i = 0; i < 4000; i++) {
        const p = pose();
        if (!pulsed && p.tick >= 500) {
          M._web_drive_analog(0, 0, 0, 1); M._web_drive_step();
          M._web_drive_analog(0, 0, 0, 0); pulsed = true;
        } else M._web_drive_step();
        const life = JSON.parse(M.ccall('web_drive_lifecycle_state', 'string', [], []));
        if (pulsed && !life.cam && !life.scripted) break;
        if (i === 3999) throw new Error('P09 opening did not release');
      }
      if (backend === 'gpu') {
        /* Direct diagnostic mission reloads invalidate C-owned mesh pointers
         * without crossing the page's normal shell dirty hook. Recreate the
         * GPU scene after each reload so no case can sample stale resources. */
        document.getElementById('gpuBtn').click(); // gpu -> sw, teardown
        document.getElementById('gpuBtn').click(); // sw -> gpu, retry
        let active = false;
        for (let i = 0; i < 120; i++) {
          window.__jitterClock.pump(dt);
          await new Promise((done) => setTimeout(done, 0));
          const screen = document.getElementById('screen');
          const gpu = screen?.nextElementSibling;
          if (gpu && gpu.tagName === 'CANVAS' &&
              getComputedStyle(gpu).display !== 'none') {
            active = true;
            break;
          }
        }
        if (!active) throw new Error('WebGPU renderer did not reactivate after mission reload');
      }
      M._web_drive_preset(preset);
      /* Normalize the page accumulator phase after reload/GPU startup. The
       * first tick edge leaves the 60 Hz / 20 Hz accumulator at its common
       * boundary; re-place after that unmeasured edge so every case has the
       * same two repeated render frames followed by one sim step. */
      M._web_drive_probe_place(fixture.x, fixture.z, fixture.yaw, fixture.speed);
      M._web_drive_analog(0, 0, 0, 0);
      const alignTick = pose().tick;
      let aligned = false;
      for (let i = 0; i < 4; i++) {
        window.__jitterClock.pump(dt);
        if (pose().tick !== alignTick) { aligned = true; break; }
      }
      if (!aligned) throw new Error('could not align render/sim accumulator phase');
      M._web_drive_probe_place(fixture.x, fixture.z, fixture.yaw, fixture.speed);
      M._web_drive_analog(1, 0, 0, 0);
      /* Probe placement changes 0 -> fixture.speed instantaneously, which is
       * intentionally classified as a discontinuity by extrapolation. It is
       * not steady driving: warm through four sim ticks before measuring the
       * owner-approved straight/rough smoothness fixtures. */
      for (let frame = 0; frame < warmupFrames; frame++)
        window.__jitterClock.pump(dt);
      if (backend === 'gpu') window.__i76.gpuSubmissionTrace = [];
      const rows = [];
      const alternatingTick0 = pose().tick;
      for (let frame = 0; frame < frames; frame++) {
        const beforePose = pose();
        const commandSteer = steering === 'alternate'
          ? (((beforePose.tick - alternatingTick0) & 1) ? -1 : 1)
          : 0;
        M._web_drive_analog(1, commandSteer, 0, 0);
        const before = beforePose.tick;
        const clock = window.__jitterClock.pump(dt);
        const p = pose(), c = camera(), tc = tickCamera(), d = diag();
        const renderBasis = f64(M._web_gpu_car_xform(), 12);
        const renderCarScreen = project(cameraVp(c), xformToMat4(renderBasis),
                                        [0, 0, 0], false);
        const submissions = backend === 'gpu'
          ? window.__i76.gpuSubmissionTrace.splice(0) : [];
        if (backend === 'gpu' && submissions.length !== 1)
          throw new Error(`GPU frame ${frame} submitted ${submissions.length} command buffers`);
        rows.push({
          frame, renderTimeMs: clock.now, callbacks: clock.callbacks,
          tickBefore: before, tick: p.tick, commandSteer, steering,
          pose: p, diag: d, camera: c,
          cameraAngles: angles(c), tickCamera: tc, tickCameraAngles: angles(tc),
          renderBasis, renderCarScreen, presentation: presentation(),
          gpuMatrix: gpuMatrixSample(c, submissions[0]),
          roadClearance: M._web_road_clearance(p.x, p.z),
        });
      }
      M._web_drive_analog(0, 0, 0, 0);
      const canvases = [...document.querySelectorAll('#gamePanel canvas')].map((c) => ({
        id: c.id, display: getComputedStyle(c).display, width: c.width, height: c.height,
      }));
      return { backend, rows, canvases, gpuTarget: window.__i76.gpuTarget || null,
               blitErr: window.__i76.blitErr || '' };
    }, { backend, fixture: FIXTURES[surface], preset: VIEWS[view],
         frames: FRAMES, dt: DT_RENDER_MS, warmupFrames: WARMUP_FRAMES,
         steering });

  const cases = [];
  for (const backend of ['sw', 'gpu']) {
    if (backend === 'gpu') {
      /* Opt in before GpuScene construction so ordinary ?dev=1 sessions do
       * not pay submission-trace allocations on every frame. */
      await page.evaluate(() => { window.__i76.gpuSubmissionTrace = []; });
      await page.click('#gpuBtn');
      for (let i = 0; i < 120; i++) {
        await page.evaluate((dt) => window.__jitterClock.pump(dt), DT_RENDER_MS);
        const active = await page.evaluate(() => {
          const screen = document.getElementById('screen');
          const gpu = screen?.nextElementSibling;
          return !!gpu && gpu.tagName === 'CANVAS' && getComputedStyle(gpu).display !== 'none';
        });
        if (active) break;
        if (i === 119) throw new Error('WebGPU renderer did not become active');
      }
    }
    const specs = [];
    for (const surface of Object.keys(FIXTURES))
      for (const view of Object.keys(VIEWS))
        specs.push({ surface, view, steering: 'straight' });
    specs.push({ surface: 'straight', view: 'chase', steering: 'alternate' });
    for (const { surface, view, steering } of specs) {
        const raw = await runCase(backend, surface, view, steering);
        if (raw.rows.some((r) => r.callbacks !== 1))
          throw new Error(`${backend}/${surface}/${view}: fixed clock callback count != 1`);
        const ticks = raw.rows
          .filter((r, i, a) => i === 0 || r.tick !== a[i - 1].tick)
          .map((r) => ({ ...r, camera: r.tickCamera,
                         cameraAngles: r.tickCameraAngles }));
        const summary = summarize(raw.rows, ticks);
        if (summary.repeatFraction > 0.02 ||
            (steering === 'straight' && summary.jitter.perFrame.position > 0.01) ||
            (steering === 'straight' && view === 'cockpit' &&
             summary.jitter.perFrame.orientation > 0.002))
          throw new Error(`${backend}/${surface}/${view}: render camera still steps ` +
            JSON.stringify(summary));
        if (backend === 'gpu' && !raw.canvases.some((c) =>
              c.id !== 'screen' && c.display !== 'none'))
          throw new Error(`${surface}/${view}: WebGPU canvas not active`);
        if (backend === 'gpu' && raw.blitErr)
          throw new Error(`${surface}/${view}: WebGPU fallback/error ${raw.blitErr}`);
        if (backend === 'gpu' &&
            (!raw.gpuTarget || raw.gpuTarget.mode !== GPURES ||
             (GPURES === 'fidelity'
               ? raw.gpuTarget.w !== 640 || raw.gpuTarget.h !== 480
               : raw.gpuTarget.w <= 640 || raw.gpuTarget.h <= 480)))
          throw new Error(`${surface}/${view}: wrong ${GPURES} GPU target ` +
                          JSON.stringify(raw.gpuTarget));
        /* H-UAT-059 GPU residual: gate the matrices written at queue.submit,
         * not the shared double C camera. The tracked cockpit/car vertex is
         * projected once in SW-equivalent double precision and once through
         * the exact f32 VP/model stream uploaded to WebGPU. */
        const gm = summary.gpuSubmittedMatrix;
        if (backend === 'gpu' &&
            (!gm || gm.samples !== FRAMES ||
             !Object.values(gm).every(Number.isFinite) ||
             gm.jitterErrorPxPerFrame2 > 0.01 ||
             gm.maxProjectionErrorPx > 0.15))
          throw new Error(`${surface}/${view}: GPU-submitted matrix stream jitters ` +
            JSON.stringify(summary.gpuSubmittedMatrix));
        cases.push({ backend, surface, view, steering,
                     fixture: FIXTURES[surface], summary, frames: raw.rows,
                     ticks, canvases: raw.canvases,
                     gpuTarget: raw.gpuTarget, blitErr: raw.blitErr });
        log('[case]', backend, surface, view, steering, summary);
      }
  }

  /* Identical-drive guard: car tick traces cannot vary with view/backend. */
  const traceKey = (c) => JSON.stringify(c.ticks.map((r) => [
    r.pose.x, r.pose.y, r.pose.z, r.pose.yaw, r.pose.pitch, r.pose.roll,
    r.diag.groundY, r.diag.terrainRate,
  ]));
  for (const surface of Object.keys(FIXTURES)) {
    const peers = cases.filter((c) => c.surface === surface &&
      c.steering === 'straight');
    if (!peers.every((c) => traceKey(c) === traceKey(peers[0])))
      throw new Error(`${surface}: view/backend changed the deterministic sim trace`);
    for (const view of Object.keys(VIEWS)) {
      const [sw, gpu] = cases.filter((c) => c.surface === surface &&
        c.view === view && c.steering === 'straight');
      const metricDelta = Math.max(
        Math.abs(sw.summary.jitter.perFrame.position - gpu.summary.jitter.perFrame.position),
        Math.abs(sw.summary.jitter.perFrame.orientation - gpu.summary.jitter.perFrame.orientation),
        Math.abs(sw.summary.jitter.perTick.position - gpu.summary.jitter.perTick.position),
        Math.abs(sw.summary.jitter.perTick.orientation - gpu.summary.jitter.perTick.orientation));
      if (metricDelta > 1e-9)
        throw new Error(`${surface}/${view}: SW/WebGPU camera metric delta ${metricDelta}`);
    }
  }
  const alternate = cases.filter((c) => c.steering === 'alternate');
  if (alternate.length !== 2 || traceKey(alternate[0]) !== traceKey(alternate[1]))
    throw new Error('alternating steer changed the deterministic sim trace by renderer');
  const result = {
    schema: 'i76-cockpit-jitter-trace-v3', generated: new Date().toISOString(),
    provenance: { buildSha: BUILD_SHA, browser: await browser.version(),
      assets: 'purchaser-owned Nitro Pack (not embedded)', renderHz: 60, simHz: 20,
      framesPerCase: FRAMES, warmupFrames: WARMUP_FRAMES,
      camlerp: CAMLERP, gpuResolution: GPURES, flags: FLAGS },
    mission: { file: 'miss8/P09.MSN', title: 'Mojo Rising' }, fixtures: FIXTURES,
    cases,
  };
  writeFileSync(JSON_OUT, JSON.stringify(result, null, 2) + '\n');
  log(`[result] wrote ${JSON_OUT}`);
  for (const c of alternate) {
    const h = c.summary.horizontalCar;
    /* H-UAT-069: a rigid chase camera/car pair keeps the car centre fixed
     * during a one-tick left/right reversal. This is measured in canonical
     * 640x480 source pixels; native DPR presentation scales it afterward. */
    if (!h.peakStep || Math.abs(h.peakStep.dx) > 0.05 ||
        h.meanAbsSecondPxPerFrame2 > 0.01)
      throw new Error(`${c.backend}: alternating steer jumps horizontally ` +
                      JSON.stringify(h));
  }
  if (pageErrors.length) throw new Error(`page errors: ${pageErrors.join('; ')}`);
  log('RESULT: PASS deterministic cockpit/chase SW/WebGPU jitter trace');
} catch (e) {
  log(`[FATAL] ${e.stack || e}`);
  process.exitCode = 1;
} finally {
  if (browser) await browser.close().catch((e) => log(`[close] browser: ${e}`));
  if (server) await new Promise((done) => server.close(done));
}
