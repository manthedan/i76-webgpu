#!/usr/bin/env node
/*
 * M6 full-frame parity rung 1 browser driver.
 *
 * This launches a real headless WebGPU device and drives gpu_scene_probe.html
 * at fixed world and 2D-overlay frames for P01 and N01. Software is always
 * the golden. Asset requests are fulfilled from NITRO_APP by Puppeteer
 * interception so the
 * purchaser-owned tree never needs to be copied under web/ or served by the
 * gate's static HTTP server.
 *
 * Exit 0 PASS, 1 divergence/harness failure, 2 WebGPU unavailable (SKIP).
 */
import puppeteer from 'puppeteer';
import { readFile, realpath, stat, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { externalPath, loopbackPage, newOutputDir } from './tests/external_paths.mjs';

const PAGE = process.argv[2];
const LOG_DIR = process.argv[3];
const APP_INPUT = process.env.NITRO_APP;
if (!PAGE || !LOG_DIR || !APP_INPUT) {
  console.error('usage: NITRO_APP=/owned/app node full_frame_parity.mjs <page-url> <new-output-dir>');
  process.exit(1);
}
loopbackPage(PAGE);
newOutputDir(LOG_DIR);
const CASES = [
  { tag: 'p01-world', mission: 'miss8/P01.MSN', view: 'chase', tick: 60 },
  { tag: 'n01-world', mission: 'miss8/N01.CBT', view: 'chase', tick: 60 },
  { tag: 'n01-fx', mission: 'miss8/N01.CBT', view: 'chase', tick: 400,
    fxCanary: 'xbulc1.xdf' },
  { tag: 'p01-title', mission: 'miss8/P01.MSN', view: 'chase', tick: 20,
    paper: 'title' },
  { tag: 'p01-title-damage', mission: 'miss8/P01.MSN', view: 'chase', tick: 20,
    paper: 'title', damage: true },
  { tag: 'p01-map', mission: 'miss8/P01.MSN', view: 'chase', tick: 60,
    paper: 'map' },
  { tag: 'n01-view-cycle', mission: 'miss8/N01.CBT', view: 'chase', tick: 60,
    viewCycle: true },
];
/* Exact canonical SwiftShader RGBA hashes. P01 retains the pre-native-
 * resolution baseline (main 9453551); N01 was deliberately re-pinned on the
 * post-combat merged tree because three fielded opponents, spawn clamping and
 * live combat HUD state change the canonical tick-60 frame. Later class-4
 * dropper and native turret-convergence slices re-pin only N01: its fielded
 * loadout/HUD, live dropper FX, and now-honest turret hits are intentional
 * authority pixels; P01 and paper baselines remain unchanged. Chrome remains
 * pinned at 151.0.7922.71. Other adapters retain the portable SW/GPU tolerance
 * gate because rasterization bytes are device-specific.
 * 2026-08-17 steward repin: p01-world + p01-map only, justified by the
 * H-UAT-076a convoy road-adherence merge (5cdbb63) — tanker1/tanker2/Jade
 * now hold the authored road at the pinned ticks, moving their authority
 * pixels and map blips. Isolated by bisect: f88ee22 PASS, 5491fa4 FAIL,
 * hashes stable through 4bdc5ba (draw-pose interpolation is identity at
 * exact tick boundaries; no caltrops exist at tick 60). SW-vs-GPU parity
 * itself never regressed (p01-map diff 0.000%). N01/title pins untouched.
 * 2026-09-02 N01 repin: a fresh per-commit browser bisect found 0df3042
 * PASS and 9369363 FAIL. The latter intentionally replaced the marked 90 m
 * combat hold with decoded FUN_0040aa20 octant pursuit (H-UAT-077f), moving
 * N01's deterministic opponent/combat authority pixels. The new tick-60
 * hash is 0x039937e9 (world + view cycle). Tick 210 no longer has visible FX,
 * so the both-path FX canary moves to tick 250 and hash 0xe0272cac rather
 * than weakening its presence assertion. Portable SW/GPU comparison remains
 * bounded and unchanged. At tick 60 the
 * current comparison is full 0.335%, terrain 0.541%, road 0.450%, sky
 * 0.014%; P01 and paper hashes remain unchanged.
 *
 * 2026-09-02 H-UAT-079d repin: initial selection no longer arms a source-0
 * dropper. N01's fresh Valepre now highlights its final non-dropper 30cal,
 * changing only intentional weapon-HUD authority pixels. Two fresh pre-repin
 * runs reproduced 0x84337ed9 at tick 60/view-cycle and 0x4812d548 at the FX
 * tick with the exact comparison metrics above; every P01/paper hash stayed
 * byte-identical.
 *
 * 2026-09-03 Stage-3 repin: physical car.c authority for the deterministic
 * melee opponent moves only that opponent, its radar/ammo feedback and damage
 * edge at ticks 60/250. The old/new tick-250 software-frame diff is 11
 * connected components: the body/effect cluster (2,409 px), right-edge hit
 * cue (1,440 px), and small radar/ammo contacts; no unrelated world component
 * changes. Tick 250's transitional SwiftShader hash is 0xa6dc4cf4, but the
 * physical opponent has no on-screen FX there (`fxEvents=2`, `swFx=gpuFx=0`),
 * so retaining it would weaken the both-path canary. The first scanned stable
 * active frame is tick 400 (`fxEvents=11`, `swFx=gpuFx=1`), pinned below at
 * 0x52850219; tick 60 moves to 0xeb221979. The measured tick-60 comparison
 * moves marginally to full 0.336%, terrain 0.544%, road 0.450%, sky 0.014%;
 * the unchanged 1% threshold is not a pin and was not relaxed.
 *
 * 2026-09-03 H-UAT-079b repin: tick 400 remains the FX canary. Replacing
 * Fire-Dropper's gameplay-radius disc with its authored horizontal OGEO
 * correctly removes 462 old pixels; the eleven live MFIRESPL objects are
 * outside this camera's visible/depth-owned footprint. Rather than accept a
 * zero-visible-FX frame, the gate injects one presentation-only xbulc1 XDF at
 * a deterministic visible point and requires authored resolution, no missing
 * fallback, and SW/GPU presence. The three frame-0 pixels plus the disc removal
 * move only n01-fx to 0x64fa4c09; HUD exactness and parity limits are unchanged. */
const SWIFTSHADER_FIDELITY_HASHES = Object.freeze({
  'p01-world': '0xd3b9e84c',
  /* 2026-09-03 steward repin (H-UAT-079d weapon HUD on the Stage-3 tree):
   * measured on the merged tree; each N01 software frame differs from the
   * Stage-3 golden by exactly 2,819 px, all inside the weapon-panel rows
   * (x 225..355, y 8..93: dropper labels/ammo + 30mm cannon armed at load);
   * world/opponent pixels are byte-identical. P01/paper hashes unchanged. */
  'n01-world': '0x01bd8909',
  'n01-fx': '0x64fa4c09',
  'p01-title': '0x3b7a1a04',
  /* H-UAT-013: episode card plus a real queued tanker hit; damage safety
   * feedback is the final overlay in both paths. Re-pinned from the first
   * clean SwiftShader run of this exact new case (SW/GPU diff 0.000%). */
  'p01-title-damage': '0xcd412ad8',
  'p01-map': '0xaed44f01',
  'n01-view-cycle': '0x01bd8909',
});
const FLAGS = [
  '--no-sandbox',
  '--disable-dev-shm-usage',
  '--enable-unsafe-webgpu',
  '--enable-webgpu-developer-features',
  '--use-angle=vulkan',
  '--disable-background-timer-throttling',
  '--disable-renderer-backgrounding',
  '--disable-backgrounding-occluded-windows',
];
const ASSET_PREFIX = '/__i76_assets/';

let app;
try {
  app = externalPath(APP_INPUT, 'NITRO_APP', { existing: true });
  if (!(await stat(path.join(app, 'nitro.zfs'))).isFile())
    throw new Error('nitro.zfs is not a file');
} catch (e) {
  console.error(`FAIL asset prerequisite ${APP_INPUT}: ${e.message}`);
  process.exit(1);
}
const appPrefix = app.endsWith(path.sep) ? app : app + path.sep;

function safeAssetPath(urlText) {
  const url = new URL(urlText);
  const pathname = url.pathname;
  if (url.origin !== new URL(PAGE).origin || !pathname.startsWith(ASSET_PREFIX)) return null;
  let rel;
  try { rel = decodeURIComponent(pathname.slice(ASSET_PREFIX.length)); }
  catch { return false; }
  if (!rel || rel.includes('\0') || rel.includes('\\')) return false;
  const full = path.resolve(app, rel);
  if (!full.startsWith(appPrefix)) return false;
  return full;
}

async function installAssetRoute(page) {
  await page.setRequestInterception(true);
  page.on('request', async (request) => {
    const full = safeAssetPath(request.url());
    if (full === null) { await request.continue(); return; }
    if (full === false) { await request.respond({ status: 400 }); return; }
    try {
      const actual = await realpath(full);
      if (!actual.startsWith(appPrefix)) {
        await request.respond({ status: 403 });
        return;
      }
      const body = await readFile(actual);
      const headers = {
        'Access-Control-Allow-Origin': '*',
        'Content-Type': 'application/octet-stream',
        'Content-Length': String(body.length),
      };
      await request.respond({ status: 200, headers,
        body: request.method() === 'HEAD' ? undefined : body });
    } catch (e) {
      if (e.code !== 'ENOENT') console.error(`asset route ${full}: ${e.message}`);
      await request.respond({ status: e.code === 'ENOENT' ? 404 : 500 });
    }
  });
}

function regionSummary(frame) {
  const ff = frame.patterns?.find((p) => p.name === 'full-frame-color');
  if (!ff) return 'missing full-frame-color metrics';
  return ff.regions.map((r) =>
    `${r.name}=${(r.outlierFraction * 100).toFixed(3)}% ` +
    `(p95=${r.p95MaxChannelDiff}, max=${r.maxChannelDiff})`).join('; ');
}

async function saveFailureShots(page, tag) {
  const paths = {
    sideBySide: path.join(LOG_DIR, `${tag}-side-by-side.png`),
    software: path.join(LOG_DIR, `${tag}-software.png`),
    webgpu: path.join(LOG_DIR, `${tag}-webgpu.png`),
  };
  const comparison = await page.$('#comparison');
  const sw = await page.$('#ref');
  const gpu = await page.$('#gpu');
  if (!comparison || !sw || !gpu)
    throw new Error('comparison canvases absent while saving failure evidence');
  await comparison.screenshot({ path: paths.sideBySide });
  await sw.screenshot({ path: paths.software });
  await gpu.screenshot({ path: paths.webgpu });
  return paths;
}

let browser;
let failed = false;
let skipped = null;
try {
  browser = await puppeteer.launch({
    headless: 'new', protocolTimeout: 900000,
    ...(process.env.CHROME ? { executablePath: process.env.CHROME } : {}),
    args: FLAGS,
  });
  const version = await browser.version();
  console.log(`webgpu-full-frame: browser=${version}`);
  console.log(`webgpu-full-frame: flags=${FLAGS.join(' ')}`);
  console.log(`webgpu-full-frame: log_dir=${LOG_DIR}`);

  for (const test of CASES) {
    const page = await browser.newPage();
    const consoleLines = [];
    const pageErrors = [];
    page.on('console', (msg) => consoleLines.push(`${msg.type()}: ${msg.text()}`));
    page.on('pageerror', (e) => {
      const line = `pageerror: ${e.message}`;
      pageErrors.push(line);
      consoleLines.push(line);
    });
    await installAssetRoute(page);
    await page.setViewport({ width: 1400, height: 1100, deviceScaleFactor: 1 });
    const pageUrl = new URL(PAGE);
    pageUrl.searchParams.set('app', pageUrl.origin + ASSET_PREFIX);
    pageUrl.searchParams.set('present', '0');
    pageUrl.searchParams.set('gpures', 'fidelity');
    pageUrl.searchParams.set('mission', test.mission);
    pageUrl.searchParams.set('view', test.view);
    pageUrl.searchParams.set('tick', String(test.tick));
    if (test.paper) pageUrl.searchParams.set('paper', test.paper);
    if (test.damage) pageUrl.searchParams.set('damage', '1');
    if (test.viewCycle) pageUrl.searchParams.set('cycle', 'views');
    if (test.tag === 'n01-fx') pageUrl.searchParams.set('requirefx', '1');
    if (test.fxCanary) pageUrl.searchParams.set('fxcanary', test.fxCanary);

    let probe;
    try {
      await page.goto(pageUrl.href, { waitUntil: 'domcontentloaded', timeout: 120000 });
      await page.waitForFunction(
        () => /RESULT:\s*(PASS|FAIL|SKIP)/.test(document.title),
        { timeout: test.viewCycle ? 600000 : 300000 });
      probe = await page.evaluate(() => window.__gpuSceneProbe || null);
      if (!probe || !['PASS', 'FAIL', 'SKIP'].includes(probe.status))
        throw new Error('probe returned no terminal structured result');
    } catch (e) {
      probe = { status: 'FAIL', reason: `browser harness: ${e.message}` };
    }

    const record = { test, browser: version, flags: FLAGS, pageErrors, probe };
    await writeFile(path.join(LOG_DIR, `${test.tag}.json`),
                    JSON.stringify(record, null, 2) + '\n');
    await writeFile(path.join(LOG_DIR, `${test.tag}-console.log`),
                    consoleLines.join('\n') + '\n');

    if (probe.status === 'SKIP') {
      skipped = `${test.tag}: ${probe.reason || 'WebGPU unavailable'}`;
      console.log(`SKIPPED — headless WebGPU unavailable: ${skipped}`);
      await page.close();
      break;
    }

    const frame = probe.frame;
    const fixed = frame && frame.mission === test.mission &&
      frame.tick === test.tick && frame.requestedView === test.view &&
      frame.requestedPaper === (test.paper || 'none') &&
      frame.requestedDamage === Boolean(test.damage);
    const ff = probe.patterns?.find((p) => p.name === 'full-frame-color');
    const hudBand = probe.patterns?.find((p) => p.name === 'hud-band-overlay');
    const cycle = probe.patterns?.find((p) => p.name === 'view-cycle-live');
    const canonicalSwiftShader = /swiftshader/i.test(probe.adapter || '');
    const expectedHash = SWIFTSHADER_FIDELITY_HASHES[test.tag];
    const fidelityExact = !canonicalSwiftShader ||
      frame?.hashes?.webgpuRgba === expectedHash;
    const fxOverlay = probe.patterns?.find((p) => p.name === 'combat-fx-overlay');
    const valid = probe.webgpu === true && typeof probe.adapter === 'string' &&
      probe.adapter.length > 0 && fixed && ff && ff.regions?.length === 4 &&
      (test.tag !== 'n01-fx' || (fxOverlay && fxOverlay.match &&
       fxOverlay.fxEvents > 0 && fxOverlay.swFx === 1 && fxOverlay.gpuFx === 1 &&
       fxOverlay.fxCanary === test.fxCanary && fxOverlay.fxCanaryArmed &&
       fxOverlay.impactAuthored > 0 && fxOverlay.impactMissing === 0)) &&
      frame?.gpuResolution === 'fidelity' &&
      frame?.renderTarget?.w === 640 && frame?.renderTarget?.h === 480 &&
      (!test.tag.startsWith('n01-') || (hudBand && hudBand.match &&
       hudBand.covered > 1000 && hudBand.mismatches === 0)) &&
      fidelityExact && pageErrors.length === 0 &&
      (!test.viewCycle || (cycle && cycle.match));
    let shots = null;
    if (test.tag === 'n01-fx') {
      try { shots = await saveFailureShots(page, test.tag); }
      catch (e) { console.error(`FAIL ${test.tag} artifact capture: ${e.message}`); }
    }
    if (probe.status !== 'PASS' || !valid) {
      failed = true;
      if (!shots) {
        try { shots = await saveFailureShots(page, test.tag); }
        catch (e) { console.error(`FAIL ${test.tag} artifact capture: ${e.message}`); }
      }
      console.error(`FAIL ${test.tag} ${test.mission}: ` +
        `${probe.reason || regionSummary(probe)}`);
      if (cycle)
        console.error(`FAIL view cycle: ${cycle.results.map((item) =>
          `${item.key}/${item.name} gap=${item.visualGapMs ?? 'n/a'}ms ` +
          `budget=${item.stallBudgetMs ?? 'n/a'}ms ` +
          `${item.stalled ? 'STALLED' : item.error ? 'ERROR' : 'ok'}`).join('; ')}`);
      if (!fidelityExact)
        console.error(`FAIL fidelity bytes: expected ${expectedHash}, got ` +
                      `${frame?.hashes?.webgpuRgba}`);
      if (pageErrors.length)
        console.error(`FAIL page errors: ${pageErrors.join(' | ')}`);
      if (consoleLines.length)
        console.error(`FAIL console: ${consoleLines.slice(-20).join(' | ')}`);
      console.error(`FAIL evidence: ${shots ? JSON.stringify(shots) : LOG_DIR}`);
    } else {
      console.log(`ok   ${test.tag} ${test.mission} tick=${frame.tick} ` +
        `adapter=${probe.adapter}`);
      console.log(`info ${test.tag} ${regionSummary(probe)}`);
      console.log(`info ${test.tag} hashes sw=${frame.hashes?.softwareRgba} ` +
                  `gpu=${frame.hashes?.webgpuRgba}` +
                  (canonicalSwiftShader ? ` fidelity-golden=${expectedHash} BIT-IDENTICAL` :
                    ' fidelity-golden=adapter-specific (portable parity only)'));
    }
    await page.close();
  }
} catch (e) {
  console.error(`FAIL WebGPU browser launch: ${e.stack || e}`);
  failed = true;
} finally {
  if (browser) await browser.close().catch(() => {});
}

if (skipped) process.exit(2);
if (failed) process.exit(1);
console.log(`RESULT: PASS (${CASES.length} cases checked; SW golden + live view cycle)`);
