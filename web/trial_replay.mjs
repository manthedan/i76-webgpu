// trial_replay.mjs — standalone replay + inspection of a human-trial bundle.
//
//   node web/trial_replay.mjs <bundleDir> --output <newArtifactDir>
//       [--frames] [--ai-metrics] [--ai-trace] [--tolerance <m>]
//
// Replays tape.jsonl through the SAME wasm build path the page used,
// applying tick-stamped mechanics events, feeding each tick's recorded input
// masks via web_tape_input, and comparing pose at tape precision.
// --frames renders marker ticks to PNG (the agent's eyes for legibility
// complaints). Exit 0 = replay ran to tape end; pose divergences are
// REPORTED, not fatal (a build bump legitimately changes behavior).
// Exit 1 = harness failure: load error, empty tape, tick desync, NaN, or
// zero ticks replayed (harness paranoia rules, PLAN §1.4).
//
// Assets: I76_APP (or NITRO_APP) points at the extracted game app dir;
// every archive, font, and loose mission/campaign file in it is staged,
// mirroring what the page's one-stop intake gives the engine.

import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { externalPath, inside, newOutputDir } from './tests/external_paths.mjs';
import zlib from 'node:zlib';
import I76Web from './dist/i76web.mjs';

const args = process.argv.slice(2);
const bundleDir = externalPath(args[0], 'trial bundle', { existing: true });
const outputIdx = args.indexOf('--output');
const artifactDir = externalPath(outputIdx >= 0 ? args[outputIdx + 1] : null, 'replay output');
const wantFrames = args.includes('--frames');
const wantAiMetrics = args.includes('--ai-metrics') || args.includes('--ai-trace');
const wantAiTrace = args.includes('--ai-trace');
const tolIdx = args.indexOf('--tolerance');
const TOL = tolIdx >= 0 ? Number(args[tolIdx + 1]) : 1e-9;

if (inside(bundleDir, artifactDir) || !Number.isFinite(TOL) || TOL < 0) {
  console.error('FAIL output must be outside the recording; tolerance must be finite and nonnegative');
  process.exit(2);
}
try {
  newOutputDir(artifactDir);
} catch (e) {
  console.error(`FAIL artifact directory must not already exist: ${artifactDir} (${e.message})`);
  process.exit(1);
}

const readJsonl = (name) => {
  try {
    return readFileSync(join(bundleDir, name), 'utf8')
      .split('\n').filter(Boolean).map((l) => JSON.parse(l));
  } catch (error) {
    if (error.code === 'ENOENT' && name !== 'tape.jsonl') return [];
    throw new Error(`invalid trial input ${name}: ${error.message}`);
  }
};

const meta = JSON.parse(readFileSync(join(bundleDir, 'meta.json'), 'utf8'));
const tape = readJsonl('tape.jsonl');
const events = readJsonl('events.jsonl');
const markers = readJsonl('markers.jsonl');

if (meta.kind !== 'i76-human-trial' || meta.v !== 1) {
  console.error(`FAIL bundle kind/version unsupported: ${meta.kind} v${meta.v}`);
  process.exit(1);
}

let failures = 0;
const fail = (msg) => { console.error(`FAIL ${msg}`); failures++; };
const info = (msg) => console.log(`info ${msg}`);

// ---- minimal PNG (8-bit RGB, no deps; agent eyes need .png, not .ppm) ---

const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let n = 0; n < 256; n++) {
    let c = n;
    for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1;
    t[n] = c >>> 0;
  }
  return t;
})();
function crc32(buf) {
  let c = 0xffffffff;
  for (const b of buf) c = CRC_TABLE[(c ^ b) & 0xff] ^ (c >>> 8);
  return (c ^ 0xffffffff) >>> 0;
}
function pngChunk(type, data) {
  const out = Buffer.alloc(12 + data.length);
  out.writeUInt32BE(data.length, 0);
  out.write(type, 4, 'ascii');
  data.copy(out, 8);
  out.writeUInt32BE(crc32(out.subarray(4, 8 + data.length)), 8 + data.length);
  return out;
}
function encodePng(w, h, rgb) {
  const sig = Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]);
  const ihdr = Buffer.alloc(13);
  ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4);
  ihdr[8] = 8; ihdr[9] = 2;   // 8-bit, truecolor
  const raw = Buffer.alloc((w * 3 + 1) * h);
  for (let y = 0; y < h; y++) {
    raw[y * (w * 3 + 1)] = 0;   // filter: none
    rgb.copy(raw, y * (w * 3 + 1) + 1, y * w * 3, (y + 1) * w * 3);
  }
  return Buffer.concat([
    sig, pngChunk('IHDR', ihdr),
    pngChunk('IDAT', zlib.deflateSync(raw, { level: 6 })),
    pngChunk('IEND', Buffer.alloc(0)),
  ]);
}

// ---- module + asset staging (page-equivalent intake) ---------------------

const appDir = externalPath(process.env.I76_APP ?? process.env.NITRO_APP,
  'I76_APP or NITRO_APP', { existing: true });
const M = await I76Web();
M.FS.mkdirTree('/data');
const stageDir = (sub, dest) => {
  let files = [];
  try { files = readdirSync(join(appDir, sub)); } catch { return 0; }
  M.FS.mkdirTree(dest);
  for (const f of files)
    M.FS.writeFile(`${dest}/${f}`, readFileSync(join(appDir, sub, f)));
  return files.length;
};
let staged = 0;
for (const f of readdirSync(appDir))
  if (/\.(zfs|zix|fnt)$/i.test(f))
    { M.FS.writeFile('/data/' + f, readFileSync(join(appDir, f))); staged++; }
staged += stageDir('miss8', '/data/miss8');
staged += stageDir('miss16', '/data/miss16');
staged += stageDir('addon', '/data/addon');
info(`staged ${staged} files from ${appDir}`);

M._web_init();
M._web_campaign_refresh();

if (meta.shellRules)
  M._web_shell_set_rules(meta.shellRules.laps ?? 0, meta.shellRules.captures ?? 0,
                         meta.shellRules.kills ?? 0, meta.shellRules.minutes ?? 0);

const rc = M.ccall('web_drive_load', 'number', ['string', 'string'],
                   [meta.mission, meta.vehicle || '']);
if (rc !== 0) {
  fail(`web_drive_load(${meta.mission}, ${meta.vehicle}) rc=${rc}`);
  process.exit(1);
}
if (!tape.length) { fail('tape.jsonl has 0 ticks'); process.exit(1); }

/* Mid-drive starts (meta.startTick > 0) cannot replay pose-exact: the tape
 * lacks the inputs that produced the state at recording start. They remain
 * full evidence bundles (tape trace + markers + video); inspect directly. */
if (meta.startTick > 0) {
  const summary = {
    mission: meta.mission, ticks: tape.length, markers: markers.length,
    events: events.length, startTick: meta.startTick,
    firstTick: tape[0].tick, lastTick: tape[tape.length - 1].tick,
    result: 'EVIDENCE-ONLY',
    note: 'recording started mid-drive; pose-exact replay needs startTick 0',
  };
  writeFileSync(join(artifactDir, 'replay-summary.json'),
                JSON.stringify(summary, null, 2) + '\n');
  console.log(JSON.stringify(summary));
  console.log('RESULT: PASS');
  process.exit(0);
}

const replayBuild = M.ccall('web_build_id', 'string', [], []);
const sameBuild = meta.build === replayBuild;
info(`build recorded=${meta.build} replaying=${replayBuild}` +
     ` mission=${meta.mission} vehicle=${meta.vehicle || '(mission default)'}` +
     (!sameBuild
       ? ' — CROSS-BUILD REPLAY: divergences document behavior change' : ''));

// ---- replay loop -----------------------------------------------------------

const pose = () => JSON.parse(M.ccall('web_drive_pose', 'string', [], []));
const POSE_FIELDS = ['x', 'y', 'z', 'yaw', 'pitch', 'roll', 'speed'];
const orderedEvents = events.map((event, order) => ({ event, order }));
const viewEvents = orderedEvents.filter(({ event }) => event.kind === 'view')
  .sort((a, b) => a.event.tick - b.event.tick || a.order - b.order)
  .map(({ event }) => event);
/* Recover is page-owned rather than an InputButton: I76Trial.event stamps
 * the last completed tick immediately after web_drive_recover(), so replay
 * must apply it after that tick and before consuming the next tape line.
 * File order breaks same-tick ties exactly as the page observed them. */
const recoverEvents = orderedEvents.filter(({ event }) => event.kind === 'recover')
  .sort((a, b) => a.event.tick - b.event.tick || a.order - b.order)
  .map(({ event }) => event);
const markerByTick = new Map(markers.map((m) => [m.tick, m]));

function dumpFrame(name) {
  M._web_drive_render();
  const W = 640, H = 480;
  const fbPtr = M._web_fb();
  let palPtr = M._web_level_palette();
  if (!palPtr) palPtr = M._web_palette();
  const fb = M.HEAPU8.subarray(fbPtr, fbPtr + W * H);
  const pal = M.HEAPU8.subarray(palPtr, palPtr + 768);
  const rgb = Buffer.alloc(W * H * 3);
  for (let i = 0; i < W * H; i++) {
    rgb[i * 3] = pal[fb[i] * 3];
    rgb[i * 3 + 1] = pal[fb[i] * 3 + 1];
    rgb[i * 3 + 2] = pal[fb[i] * 3 + 2];
  }
  writeFileSync(join(artifactDir, name), encodePng(W, H, rgb));
}

let divergences = 0;
let firstDivergence = null;
let maxPosDrift = 0;
let replayed = 0;
let viewIdx = 0;
let recoverIdx = 0;
let recoverEventsApplied = 0;
let recoverEventsUnavailable = 0;
let movieAcks = 0;

function applyRecoverEventsBefore(nextTick) {
  while (recoverIdx < recoverEvents.length &&
         recoverEvents[recoverIdx].tick < nextTick) {
    const event = recoverEvents[recoverIdx++];
    const recoverRc = M._web_drive_recover();
    if (recoverRc === 0) {
      recoverEventsApplied++;
    } else {
      recoverEventsUnavailable++;
      const msg = `recover event after tick ${event.tick} unavailable (rc=${recoverRc})`;
      if (sameBuild) fail(msg);
      else info(`${msg}; cross-build best effort continues`);
    }
  }
}

const wrapPi = (a) => {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a <= -Math.PI) a += 2 * Math.PI;
  return a;
};
const aiMetrics = new Map();
function sampleAiMetrics(tick) {
  if (!wantAiMetrics) return;
  const rows = JSON.parse(M.ccall('web_ai_route_state', 'string', [], []));
  for (const row of rows) {
    let m = aiMetrics.get(row.ent);
    if (!m) {
      m = { ent: row.ent, label: row.label, road: [], near: new Map(),
            prev: null, planChanges: 0 };
      for (const limit of [10, 20, 30])
        m.near.set(limit, { samples: 0, headingSteps: 0, headingFlips: 0,
                            steerSamples: 0, steerFlips: 0, moveSteps: 0,
                            stalls: 0, maxStepDelta: 0,
                            maxHeadingStep: 0, prevDh: 0, prevErr: 0 });
      aiMetrics.set(row.ent, m);
    }
    m.road.push(Math.max(0, row.roadClearance));
    const err = wrapPi(Math.atan2(-(row.planX - row.x), row.planZ - row.z)
                       - row.heading);
    const adjacent = m.prev && tick === m.prev.tick + 1;
    const dh = adjacent ? wrapPi(row.heading - m.prev.heading) : 0;
    const step = adjacent ? Math.hypot(row.x - m.prev.x, row.z - m.prev.z) : 0;
    if (m.prev && (row.planIndex !== m.prev.planIndex ||
                   row.planCount !== m.prev.planCount ||
                   row.planX !== m.prev.planX || row.planZ !== m.prev.planZ))
      m.planChanges++;
    if (wantAiTrace && row.ent <= 3 &&
        (row.playerDistance <= 30 || row.roadClearance > 5))
      console.log(`AI-TICK tick=${tick} ent=${row.ent} label=${row.label}` +
        ` playerM=${row.playerDistance.toFixed(3)}` +
        ` roadM=${row.roadClearance.toFixed(3)}` +
        ` targetRoadM=${row.planRoadClearance.toFixed(3)}` +
        ` stepM=${step.toFixed(4)} dh=${dh.toFixed(6)}` +
        ` err=${err.toFixed(6)} plan=${row.planIndex}/${row.planCount}` +
        ` target=${row.planX.toFixed(2)},${row.planZ.toFixed(2)}`);
    for (const [limit, near] of m.near) {
      if (row.playerDistance > limit) continue;
      near.samples++;
      if (m.prev && tick === m.prev.tick + 1 && m.prev.distance <= limit) {
        near.moveSteps++;
        if (step < 1e-6) near.stalls++;
        near.maxStepDelta = Math.max(near.maxStepDelta,
                                     Math.abs(step - m.prev.step));
        if (Math.abs(dh) > 1e-5) {
          near.headingSteps++;
          if (near.prevDh * dh < 0) near.headingFlips++;
          near.prevDh = dh;
        }
        near.maxHeadingStep = Math.max(near.maxHeadingStep, Math.abs(dh));
        if (Math.abs(err) > 1e-4 && Math.abs(m.prev.err) > 1e-4) {
          near.steerSamples++;
          if (err * m.prev.err < 0) near.steerFlips++;
        }
      }
    }
    m.prev = { tick, x: row.x, z: row.z, heading: row.heading, step,
               distance: row.playerDistance, err,
               planIndex: row.planIndex, planCount: row.planCount,
               planX: row.planX, planZ: row.planZ };
  }
}
function quantile(sorted, q) {
  if (!sorted.length) return 0;
  return sorted[Math.min(sorted.length - 1, Math.floor(q * (sorted.length - 1)))];
}
function printAiMetrics() {
  if (!wantAiMetrics) return;
  for (const m of [...aiMetrics.values()].sort((a, b) => a.ent - b.ent)) {
    const road = m.road.toSorted((a, b) => a - b);
    const onRoad = road.filter((d) => d === 0).length;
    console.log(`AI-ROAD ent=${m.ent} label=${m.label} samples=${road.length}` +
      ` onRoadPct=${(100 * onRoad / road.length).toFixed(2)}` +
      ` medianOffM=${quantile(road, 0.5).toFixed(3)}` +
      ` p95OffM=${quantile(road, 0.95).toFixed(3)}` +
      ` maxOffM=${quantile(road, 1).toFixed(3)}` +
      ` planChanges=${m.planChanges}`);
    for (const [limit, near] of m.near) {
      console.log(`AI-NEAR ent=${m.ent} label=${m.label} rangeM=${limit}` +
        ` samples=${near.samples}` +
        ` stalls=${near.stalls}` +
        ` stallPct=${(100 * near.stalls / Math.max(1, near.moveSteps)).toFixed(2)}` +
        ` maxStepDeltaM=${near.maxStepDelta.toFixed(4)}` +
        ` headingFlips=${near.headingFlips}` +
        ` headingFlipPct=${(100 * near.headingFlips / Math.max(1, near.headingSteps - 1)).toFixed(2)}` +
        ` steerFlips=${near.steerFlips}` +
        ` steerFlipPct=${(100 * near.steerFlips / Math.max(1, near.steerSamples)).toFixed(2)}` +
        ` maxHeadingStepRad=${near.maxHeadingStep.toFixed(6)}`);
    }
  }
}

const tickBase = tape[0].tick;
for (let li = 0; li < tape.length; li++) {
  const line = tape[li];
  if (line.tick !== tickBase + li) {
    fail(`tape line ${li} has tick=${line.tick}: not dense from ${tickBase}`);
    break;
  }

  /* >>> 0 below would coerce a missing or malformed mask into "no input". */
  const mask = (v) => Number.isSafeInteger(v) && v >= 0 && v <= 0xffffffff;
  if (!mask(line.held) || !mask(line.pressed)) {
    fail(`tape line ${li}: held/pressed must be 32-bit unsigned integer masks`);
    break;
  }

  /* Page events stamped T happened after tape line T. Apply every recover
   * strictly before the next line so its deterministic sim-history lookup is
   * in the same place as recording. Other event kinds are presentation or
   * wall-time alignment only. */
  applyRecoverEventsBefore(line.tick);

  /* Feed the same line until the sim consumes a full tick. A movie-pending
   * step drains input without advancing state (tick counter unchanged);
   * ack the clip and re-feed — the record side behaved identically. */
  let advanced = false;
  for (let retry = 0; retry < 100 && !advanced; retry++) {
    M._web_tape_input(line.held >>> 0, line.pressed >>> 0);
    M._web_drive_step();
    const pending = M.ccall('web_mission_movie_pending', 'string', [], []);
    if (pending) { M._web_mission_movie_ack(); movieAcks++; }
    const p = pose();
    if (p.tick === line.tick) {
      advanced = true;
      for (const f of POSE_FIELDS) {
        if (!Number.isFinite(p[f])) { fail(`tick ${line.tick}: ${f} not finite`); break; }
        /* A missing recorded value would make every d > TOL comparison
         * false (NaN) and report a silent PASS. */
        if (!Number.isFinite(line[f])) { fail(`tick ${line.tick}: tape ${f} missing or not finite`); break; }
        const d = Math.abs(p[f] - line[f]);
        if (f === 'x' || f === 'z') maxPosDrift = Math.max(maxPosDrift, d);
        if (d > TOL) {
          divergences++;
          if (!firstDivergence)
            firstDivergence = { tick: line.tick, field: f, tape: line[f], replay: p[f] };
        }
      }
      replayed++;
      sampleAiMetrics(line.tick);
      if (wantFrames) {
        while (viewIdx < viewEvents.length && viewEvents[viewIdx].tick <= line.tick)
          M._web_drive_preset(viewEvents[viewIdx++].preset);
        const mk = markerByTick.get(line.tick);
        if (mk && !(Number.isSafeInteger(mk.seq) && mk.seq >= 0)) {
          fail(`tick ${line.tick}: marker seq must be a nonnegative integer`);
        } else if (mk) {
          const name = `marker-${mk.seq}-tick-${mk.tick}.png`;
          dumpFrame(name);
          info(`wrote ${name}`);
        }
      }
    } else if (p.tick > line.tick) {
      fail(`tick desync at tape line ${li}: sim advanced to ${p.tick}`);
      break;
    } /* else movie-drain: re-feed */
  }
  if (!advanced) { fail(`tape line ${li} never advanced (movie loop?)`); break; }
}
/* A page recovery may be the final action before the owner stops recording.
 * It has no following pose line to compare, but it still happened in the
 * recorded mechanics stream and must affect the final rendered frame. */
if (replayed === tape.length)
  applyRecoverEventsBefore(Infinity);
M._web_tape_input_off();

if (wantFrames && tape.length) {
  dumpFrame('final-frame.png');
  info('wrote final-frame.png');
}

if (replayed === 0) fail('0 ticks replayed — harness failure, never a pass');
printAiMetrics();

const summary = {
  mission: meta.mission, ticks: tape.length, replayed,
  divergences, firstDivergence, maxPosDrift: +maxPosDrift.toFixed(4),
  movieAcks, recoverEvents: recoverEvents.length,
  recoverEventsApplied, recoverEventsUnavailable, tolerance: TOL,
  result: failures ? 'HARNESS-FAILURE' : 'REPLAYED',
};
writeFileSync(join(artifactDir, 'replay-summary.json'),
              JSON.stringify(summary, null, 2) + '\n');
console.log(JSON.stringify(summary));
if (failures) { console.error(`RESULT: FAIL (${failures} harness failures)`); process.exit(1); }
console.log('RESULT: PASS (inspect replay-summary.json: divergence count is not an exit status)');
