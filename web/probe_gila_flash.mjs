/*
 * probe_gila_flash.mjs — temporal stability gate for the P01 Gila Gas
 * forecourt slab in the SOFTWARE renderer (the page's default drive path).
 *
 * Background: bflgila1's FL1_LOT1 lot slab is a zero-thickness face that
 * used to sit coplanar with the heightfield; reciprocal-depth quantization
 * made slab and terrain trade pixels as the camera moved (ground flash).
 * The scene classifies the slab as a drive surface and lifts it 0.1 m for
 * rendering (the ROAD_LIFT_M precedent), so the slab now strictly wins.
 * The existing test_node.mjs P01 block proves the physics side (the lot is
 * crossable, the building still blocks); NOTHING watched rendered frames.
 * This probe closes that: it renders through the exact exports the page
 * uses (web_drive_load/step/render, web_fb_pre_hud, web_gpu_project,
 * web_fb_fnv1a) and asserts temporal stability at the forecourt.
 *
 * What it proves:
 *   1. Fixed-pose determinism: repeated web_drive_render() of an unchanged
 *      simulation state hashes identically (web_fb_fnv1a), at three
 *      pose/view combinations; distinct poses hash distinctly (the hash is
 *      not degenerate).
 *   2. Camera sweep: parked on the slab, a cockpit glance sweep
 *      (center -> left -> center -> right -> center) never exposes the
 *      P01 terrain index at any admitted flat slab point.
 *   3. Drive sweep: a real throttled drive the length of the west lane
 *      (chase view) keeps those points scene-slab-owned too. Adjacent
 *      uniform lot texels may legitimately sample as 249 or 250 as the
 *      projected pixel moves; the pre-fix ownership failure is terrain 183.
 *   4. Sensitivity: the same sampling pipeline, pointed at the car's drive
 *      line WITHOUT the occlusion exclusion, MUST flag the car driving
 *      over a point (slab -> bodywork -> slab) — proof the gate observes
 *      ownership alternation in real frames and cannot pass vacuously.
 *
 * Sampling discipline (why "admitted flat points"): the lot is a TEXTURED
 * face. Points near texture features (pump pad, stains, the antenna) can
 * legitimately resample, so a point is admitted only when the 5x5 patch
 * around its first on-screen projection is one index. Mixed later patches
 * are skipped. Uniform 249<->250 changes are neighboring texels on the same
 * lot surface after the canonical camera correction; terrain index 183 is
 * the ownership violation. The lift-disabled mutation produces 249<->183,
 * so testing the owner signature still detects the original regression.
 *
 * Run (candidate root):
 *   NITRO_APP=/owned/game/app node web/probe_gila_flash.mjs
 * The purchaser-owned asset directory is always explicit.
 */
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import I76Web from './dist/i76web.mjs';

const appDir = process.env.NITRO_APP;
if (!appDir) {
  console.error('probe_gila_flash.mjs: NITRO_APP is required');
  process.exit(2);
}
const FB_W = 640, FB_H = 480;

// PlatformKey values used here (platform.h enum order; test_node.mjs docs):
const PK_SPACE = 3;   // popCam out of the scripted intro
const PK_UP = 4;      // throttle
const PK_KP_LEFT = 26, PK_KP_RIGHT = 27;

let failures = 0;
const check = (name, cond, detail = '') => {
  if (cond) console.log(`ok   ${name}${detail ? ' — ' + detail : ''}`);
  else { console.error(`FAIL ${name}${detail ? ' — ' + detail : ''}`); failures++; }
};

/* ------------------------------------------------------------------ */
/* Sequence analyzer used by the detector self-test and car-body sensitivity */
/* arm. seq: samples, -1 = not visible. It deliberately flags ANY change;   */
/* the lot arms below use the P01-specific ownership classifier instead.    */
/* ------------------------------------------------------------------ */
function analyze(seq) {
  const visible = seq.filter((v) => v >= 0);
  const distinct = [...new Set(visible)];
  let alternations = 0;
  for (let i = 0; i + 3 < seq.length; i++) {
    const [a, b, c, d] = [seq[i], seq[i + 1], seq[i + 2], seq[i + 3]];
    if (a >= 0 && b >= 0 && c >= 0 && d >= 0 &&
        a === c && b === d && a !== b) alternations++;
  }
  return { visible: visible.length, distinct, changes: distinct.length - 1,
           alternations, stable: distinct.length <= 1 };
}

// P01-specific ownership oracle measured by the lift-disabled negative
// control: FL1_LOT1 uses the 249/250 ramp here; the terrain underneath is 183.
// Reject terrain even when it wins every visible sample (stronger than merely
// looking for alternation), while allowing legitimate lot-texture resampling.
const P01_TERRAIN_INDEX = 183;
function analyzeLotOwnership(seq) {
  const visible = seq.filter((v) => v >= 0);
  const terrain = visible.filter((v) => v === P01_TERRAIN_INDEX).length;
  return { visible: visible.length, distinct: [...new Set(visible)],
           terrain, stable: terrain === 0 };
}

// 1. Detector self-test: retain the generic change detector for the car-body
// sensitivity arm, and prove the P01 owner oracle rejects the measured
// slab<->terrain signature without rejecting adjacent slab texels.
{
  const abab = analyze([249, 183, 249, 183, 249, 183]);
  const step = analyze([249, 249, 183, 183, 183]);
  const flat = analyze([249, 249, 249, 249, 249]);
  const sparse = analyze([-1, 249, -1, 249, -1, 249]);
  const sparseFlip = analyze([-1, 249, -1, 183, -1, 249]);
  const lotTextureStep = analyzeLotOwnership([249, 249, 250, 250]);
  const terrainWin = analyzeLotOwnership([183, 183, 183]);
  check('analyzers flag ownership changes but allow adjacent lot texels',
        !abab.stable && abab.alternations > 0 &&
        !step.stable && flat.stable && sparse.stable && !sparseFlip.stable &&
        lotTextureStep.stable && !terrainWin.stable,
        `abab=${JSON.stringify(abab)} terrain=${terrainWin.terrain}`);
}

/* ------------------------------------------------------------------ */
/* Engine boot — same staging as test_node.mjs, purchaser assets.     */
/* ------------------------------------------------------------------ */
const M = await I76Web();
M.FS.mkdir('/data');
M.FS.writeFile('/data/nitro.zfs', readFileSync(join(appDir, 'nitro.zfs')));
M.FS.writeFile('/data/nitro.zix', readFileSync(join(appDir, 'nitro.zix')));
M.FS.mkdirTree('/data/miss8');
for (const f of ['P01.MSN', 'P01.TER'])
  M.FS.writeFile('/data/miss8/' + f, readFileSync(join(appDir, 'miss8', f)));

check('engine init opens the purchaser archive', M._web_init() > 0);
const loadRc = M.ccall('web_drive_load', 'number', ['string', 'string'],
                       ['miss8/P01.MSN', 'vdrampg2']);
check('P01.MSN drive loads', loadRc === 0, `rc=${loadRc}`);
if (loadRc !== 0) {
  console.error('RESULT: FAIL (cannot load P01)');
  process.exit(1);
}

const heap = () => new Uint8Array(M.wasmMemory.buffer);
const f64 = () => new Float64Array(M.wasmMemory.buffer);
const pose = () => JSON.parse(M.ccall('web_drive_pose', 'string', [], []));
const preHud = () => {
  const p = M._web_fb_pre_hud();
  return heap().slice(p, p + FB_W * FB_H);
};
const project = (x, y, z) => {
  const p = M._web_gpu_project(x, y, z);
  const d = f64();
  return [d[p >> 3], d[(p >> 3) + 1], d[(p >> 3) + 2]];
};

// Enter like the page: cockpit request, let the intro play, popCam out.
M._web_drive_set_view(0);
for (let i = 0; i < 200; i++) M._web_drive_step();
M._web_key_event(PK_SPACE, 1);
M._web_drive_step();
M._web_key_event(PK_SPACE, 0);
M._web_drive_step();
let waited = 0;
while (M._web_drive_view() !== 0 && waited < 500) {
  M._web_drive_step();
  waited++;
}
check('manual cockpit control after popCam', M._web_drive_view() === 0,
      `waited=${waited}/500 ticks`);

const teleport = (x, z) => {
  M.ccall('web_drive_teleport', null, ['number', 'number'], [x, z]);
  for (let i = 0; i < 8; i++) M._web_drive_step();
};

/* ------------------------------------------------------------------ */
/* 2. Fixed-pose determinism: identical poses hash identically.       */
/* ------------------------------------------------------------------ */
const fnv = () => M._web_fb_fnv1a() >>> 0;
const nonBg = (fb) => fb.reduce((n, v) => n + (v !== 0 ? 1 : 0), 0);

const GILA_PARK = [1582.0, 47500.0];      // north apron, facing north
const REPRO_POINT = [1650.0, 47390.0];    // user-reported approach, terrain
const hex = (n) => '0x' + n.toString(16).padStart(8, '0');

teleport(...GILA_PARK);
M._web_drive_set_view(0);
const parkCockpit = [];
for (let i = 0; i < 3; i++) { M._web_drive_render(); parkCockpit.push(fnv()); }
const parkFb = preHud();

M._web_drive_set_view(1);
M._web_drive_render();
const parkChase = [fnv()];
M._web_drive_render();
parkChase.push(fnv());

teleport(...REPRO_POINT);
M._web_drive_set_view(0);
M._web_drive_render();
const reproHash = [fnv()];
M._web_drive_render();
reproHash.push(fnv());

check('identical fixed poses hash identically (3 pose/view combos)',
      parkCockpit[0] === parkCockpit[1] && parkCockpit[1] === parkCockpit[2] &&
      parkChase[0] === parkChase[1] && reproHash[0] === reproHash[1],
      `cockpit=${parkCockpit.map(hex).join(',')} ` +
      `chase=${parkChase.map(hex).join(',')} repro=${reproHash.map(hex).join(',')}`);
check('the frame hash is not degenerate across poses/views',
      new Set([parkCockpit[0], parkChase[0], reproHash[0]]).size === 3);
check('fixed frames are non-empty (world drew through the software path)',
      nonBg(parkFb) > 20000, `nonbg=${nonBg(parkFb)}`);

/* ------------------------------------------------------------------ */
/* Sampling machinery for the motion arms.                            */
/* A candidate becomes ADMITTED at its first on-screen frame when the */
/* 5x5 patch around the projected pixel is one index (flat texel      */
/* region). After admission a frame only COUNTS when the current 5x5  */
/* patch is still uniform: texture-feature edges (pump pad, stains,   */
/* posts) drift through the projected pixel as perspective expands    */
/* them. A uniform neighboring lot texel may still change 249<->250,  */
/* so ownership is judged by the lift-disabled terrain signature 183, */
/* not by palette equality. This also rejects a stable terrain win.   */
/*                                                                    */
/* Frames MUST be sampled with the camera that rendered them: project */
/* against the live wasm camera immediately after each render.        */
/* ------------------------------------------------------------------ */
function makeSampler(points) {
  return points.map(([x, z]) => ({ x, z, admitted: false, rejected: false,
                                   seq: [] }));
}

function sampleFrame(samplers, fb, slabY, opts = {}) {
  const { occluder = null, minZs = 6, raw = false } = opts;
  for (const s of samplers) {
    if (s.rejected) { s.seq.push(-1); continue; }
    const [sx, sy, zs] = project(s.x, slabY, s.z);
    const ix = Math.round(sx), iy = Math.round(sy);
    const on = zs >= minZs && zs <= 400 &&
               ix >= 3 && ix < FB_W - 3 && iy >= 3 && iy < FB_H - 3;
    if (!on || (occluder && occluder(s))) { s.seq.push(-1); continue; }
    const center = fb[iy * FB_W + ix];
    if (!s.admitted) {
      let uniform = true;
      for (let dy = -2; dy <= 2 && uniform; dy++)
        for (let dx = -2; dx <= 2; dx++)
          if (fb[(iy + dy) * FB_W + ix + dx] !== center) { uniform = false; break; }
      if (!uniform) { s.rejected = true; s.seq.push(-1); continue; }
      s.admitted = true;
    }
    if (!raw) {
      // Post-admission: a mixed patch is a texture edge crossing the
      // pixel — skip the frame; a uniform patch is judged.
      let uniform = true;
      for (let dy = -2; dy <= 2 && uniform; dy++)
        for (let dx = -2; dx <= 2; dx++)
          if (fb[(iy + dy) * FB_W + ix + dx] !== center) { uniform = false; break; }
      if (!uniform) { s.seq.push(-1); continue; }
    }
    s.seq.push(center);
  }
}

function reportArm(name, samplers, minAdmitted, minVisible) {
  const admitted = samplers.filter((s) => s.admitted);
  const foreign = admitted.filter((s) => !analyzeLotOwnership(s.seq).stable);
  const covered = admitted.filter(
    (s) => analyzeLotOwnership(s.seq).visible >= minVisible);
  for (const s of foreign) {
    const a = analyzeLotOwnership(s.seq);
    console.error(`  terrain-owned (${s.x},${s.z}) values=` +
      JSON.stringify(a.distinct) + ` terrain=${a.terrain} ` +
      `seq=${s.seq.map((v) => (v < 0 ? '.' : v)).join(' ')}`);
  }
  check(`${name}: admitted flat lot points remain scene-slab-owned`,
        foreign.length === 0,
        `${foreign.length}/${admitted.length} terrain-owned`);
  check(`${name}: coverage is meaningful`,
        covered.length >= minAdmitted,
        `covered=${covered.length}/${samplers.length} ` +
        `admitted=${admitted.length} (need >=${minAdmitted} with ` +
        `>=${minVisible} frames each)`);
  return { admitted, foreign };
}

/* ------------------------------------------------------------------ */
/* 3. Cockpit glance sweep, parked on the forecourt.                  */
/* ------------------------------------------------------------------ */
teleport(...GILA_PARK);
M._web_drive_set_view(0);
const slabY = pose().y;   // car rests on the slab: slab surface proxy
const glancePts = [];
// North apron (visible centered) — avoids the pump pad via admission.
for (const z of [47515, 47520, 47525, 47530, 47535, 47540, 47545])
  for (const x of [1550, 1555, 1560, 1565, 1570, 1575, 1580, 1585, 1590, 1595])
    glancePts.push([x, z]);
// West flank (visible during the left glance) and east flank (right glance).
for (const z of [47480, 47485, 47490, 47495, 47500, 47505, 47510, 47515, 47520])
  for (const x of [1548, 1551, 1554, 1557, 1560, 1565, 1570])
    glancePts.push([x, z]);
for (const z of [47485, 47490, 47495, 47500, 47505, 47510, 47515])
  for (const x of [1590, 1595, 1600, 1605, 1610, 1615])
    glancePts.push([x, z]);
const glanceSamplers = makeSampler(glancePts);

// Sample every frame against the LIVE camera right after its render.
const glanceSample = () => sampleFrame(glanceSamplers, preHud(), slabY);
M._web_drive_render();
glanceSample();
M._web_key_event(PK_KP_LEFT, 1);
for (let i = 0; i < 3; i++) { M._web_drive_step(); M._web_drive_render(); glanceSample(); }
M._web_key_event(PK_KP_LEFT, 0);
for (let i = 0; i < 4; i++) M._web_drive_step();
M._web_drive_render();
glanceSample();
M._web_key_event(PK_KP_RIGHT, 1);
for (let i = 0; i < 3; i++) { M._web_drive_step(); M._web_drive_render(); glanceSample(); }
M._web_key_event(PK_KP_RIGHT, 0);
for (let i = 0; i < 4; i++) M._web_drive_step();
M._web_drive_render();
glanceSample();
reportArm('glance sweep (cockpit, parked)', glanceSamplers, 15, 3);

/* ------------------------------------------------------------------ */
/* 4. Throttled drive down the west lane, chase view.                 */
/*    The car-window exclusion (|dx|<3.5, -6<dz<12) skips frames      */
/*    where the car body legitimately occludes the point — measured   */
/*    in probe round 4; arm 5 below proves those frames really do     */
/*    carry a different owner.                                        */
/* ------------------------------------------------------------------ */
M._web_drive_set_view(1);
teleport(1560.0, 47440.0);
const drivePts = [];
for (let z = 47448; z <= 47548; z += 4)
  for (const x of [1548, 1550, 1552, 1555, 1558, 1561, 1564, 1567, 1570, 1573, 1576])
    drivePts.push([x, z]);
const driveSamplers = makeSampler(drivePts);

M._web_key_event(PK_UP, 1);
for (let t = 0; t < 100; t++) {
  M._web_drive_step();
  if (t % 2 !== 0) continue;
  M._web_drive_render();
  const fb = preHud();
  const cp = pose();
  sampleFrame(driveSamplers, fb, slabY, { minZs: 15, occluder: (s) =>
    Math.abs(cp.x - s.x) < 3.5 && (s.z - cp.z) > -6 && (s.z - cp.z) < 12 });
}
M._web_key_event(PK_UP, 0);
const driveEnd = pose();
check('drive arm actually drove the lane (and stayed on the slab)',
      driveEnd.z > 47500 && driveEnd.z < 47550 && driveEnd.speed > 15,
      `end=(${driveEnd.x.toFixed(1)},${driveEnd.z.toFixed(1)}) ` +
      `v=${driveEnd.speed.toFixed(1)} m/s`);
reportArm('drive sweep (chase, throttled)', driveSamplers, 40, 4);

/* ------------------------------------------------------------------ */
/* 5. Sensitivity: the pipeline MUST catch real ownership change.     */
/*    Same drive, points on the drive line, NO car-window exclusion:  */
/*    the car body covering a point (slab -> bodywork -> slab) is an  */
/*    ownership alternation in real frames. If nothing flags here,    */
/*    arms 3-4 could be passing vacuously.                            */
/* ------------------------------------------------------------------ */
teleport(1560.0, 47440.0);
const sensePts = [];
for (const z of [47460, 47470, 47480, 47490, 47500, 47510])
  for (const x of [1553, 1554, 1555, 1556, 1557, 1558, 1559, 1560])
    sensePts.push([x, z]);
const senseSamplers = makeSampler(sensePts);

M._web_key_event(PK_UP, 1);
for (let t = 0; t < 100; t++) {
  M._web_drive_step();
  if (t % 2 !== 0) continue;
  M._web_drive_render();
  // raw: judge the center pixel even under the (non-uniform) car body.
  sampleFrame(senseSamplers, preHud(), slabY, { raw: true });
}
M._web_key_event(PK_UP, 0);
const flagged = senseSamplers.filter((s) => s.admitted && !analyze(s.seq).stable);
check('sensitivity: the analyzer flags real ownership change (car drive-over)',
      flagged.length >= 1,
      `${flagged.length} point(s) changed owner under the car`);

if (failures) {
  console.error(`RESULT: FAIL (${failures})`);
  process.exit(1);
}
console.log('RESULT: PASS');
