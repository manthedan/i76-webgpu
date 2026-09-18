/*
 * frame_probe.mjs — wasm twin of tools/frame_probe.c.
 *
 * Calls web_render_fixed() with the SAME mission and camera the native
 * probe uses, and prints the FNV-1a of the raw 640x480 index buffer and of
 * the 768-byte level palette in the same format. tools/frame_gate.sh
 * compares the two lines.
 *
 * The point (docs/specs/m8/software-raster.md §8): the filled rasterizer is
 * about to be written, and the same C compiles to both wasm and x86-64. An
 * empirical cross-target equality check is worth more than any argument
 * about float precision — notably because the wasm spec permits
 * nondeterministic NaN payloads, so one 0/0 in a degenerate face breaks
 * bit-identity by specification. This is how we find out.
 *
 * Usage: node frame_probe.mjs <mission> [ex ey ez tx ty tz] [--ppm out.ppm]
 */
import { readFileSync, writeFileSync, readdirSync, existsSync } from 'node:fs';
import { join } from 'node:path';
import I76Web from './dist/i76web.mjs';

const appDir = process.env.NITRO_APP;
if (!appDir) {
  console.error('frame_probe.mjs: NITRO_APP is required');
  process.exit(2);
}

const args = process.argv.slice(2);
const mission = args[0];
if (!mission) {
  console.error('usage: frame_probe.mjs <mission> [ex ey ez tx ty tz] ' +
                '[--ppm out.ppm] [--filled]');
  process.exit(2);
}
// Defaults must match tools/frame_probe.c exactly.
let eye = [2997.5, 60.0, 48600.0];
let tgt = [2997.5, 30.0, 48900.0];
if (args.length >= 7 && args[1] !== '--ppm') {
  eye = [Number(args[1]), Number(args[2]), Number(args[3])];
  tgt = [Number(args[4]), Number(args[5]), Number(args[6])];
}
const filled = args.includes('--filled');
const ppmIdx = args.indexOf('--ppm');
const ppm = ppmIdx >= 0 ? args[ppmIdx + 1] : null;
const benchIdx = args.indexOf('--bench');
const bench = benchIdx >= 0 ? Number(args[benchIdx + 1]) : 0;

const FB_W = 640, FB_H = 480;
const fnv1a = (buf) => {
  let h = 2166136261 >>> 0;
  for (let i = 0; i < buf.length; i++) {
    h = (h ^ buf[i]) >>> 0;
    h = Math.imul(h, 16777619) >>> 0;
  }
  return h >>> 0;
};

const M = await I76Web();
M.FS.mkdir('/data');
M.FS.writeFile('/data/nitro.zfs', readFileSync(join(appDir, 'nitro.zfs')));
M.FS.writeFile('/data/nitro.zix', readFileSync(join(appDir, 'nitro.zix')));
M._web_init();

// Stage the mission directory the mission path names (miss8/ or miss16/).
const dir = mission.includes('/') ? mission.slice(0, mission.indexOf('/')) : '';
if (dir && existsSync(join(appDir, dir))) {
  M.FS.mkdirTree('/data/' + dir);
  for (const f of readdirSync(join(appDir, dir)))
    M.FS.writeFile('/data/' + dir + '/' + f, readFileSync(join(appDir, dir, f)));
}

const rc = M.ccall('web_mission_load', 'number', ['string'], [mission]);
if (rc !== 0) {
  console.error(`frame_probe.mjs: web_mission_load(${mission}) rc=${rc}`);
  process.exit(1);
}

/* Set the backend EXPLICITLY in both directions, not just on --filled: the
 * engine default is now FILLED, so leaving the wire arm implicit would turn
 * the gate's two arms into the same arm and stop testing the wire path. */
M._web_set_backend(filled ? 1 : 0);
M._web_render_fixed(eye[0], eye[1], eye[2], tgt[0], tgt[1], tgt[2]);

/* --bench N: the wasm half of the §9 frame-cost measurement. Same median/min
 * reporting as the native probe, for the same reason -- the first frames carry
 * one-time per-face palette resolution that a mean would smear into the
 * steady-state cost. */
if (bench > 0) {
  const ms = [];
  for (let i = 0; i < bench; i++) {
    const t0 = performance.now();
    M._web_render_fixed(eye[0], eye[1], eye[2], tgt[0], tgt[1], tgt[2]);
    ms.push(performance.now() - t0);
  }
  const first = ms[0];
  ms.sort((a, b) => a - b);
  const med = ms[bench >> 1];
  console.log(`BENCH backend=${M.UTF8ToString(M._web_backend_name())} ` +
    `frames=${bench} first=${first.toFixed(2)}ms min=${ms[0].toFixed(2)}ms ` +
    `median=${med.toFixed(2)}ms p95=${ms[Math.floor((bench - 1) * 0.95)].toFixed(2)}ms ` +
    `fps_median=${(1000 / med).toFixed(1)}`);
  process.exit(0);
}

const heap = new Uint8Array(M.wasmMemory.buffer);
const fb = heap.subarray(M._web_fb(), M._web_fb() + FB_W * FB_H);
let nonbg = 0;
for (let i = 0; i < fb.length; i++) if (fb[i] !== 0) nonbg++;
/* Filled frames gate on GEOMETRY coverage: the sky fills every pixel, so a
 * colour-based floor would pass even with the whole world missing. */
const geo = filled ? M._web_geometry_pixels() : nonbg;

const palPtr = M._web_level_palette();
const pal = palPtr ? heap.subarray(palPtr, palPtr + 768) : null;

const hex = (n) => '0x' + (n >>> 0).toString(16).padStart(8, '0');
console.log(`FRAME backend=${M.UTF8ToString(M._web_backend_name())} mission=${mission} fb_fnv1a=${hex(fnv1a(fb))} ` +
  `pal_fnv1a=${hex(pal ? fnv1a(pal) : 0)} nonbg=${nonbg} geo=${geo} ` +
  `eye=${eye.map((v) => v.toFixed(1)).join(',')} ` +
  `tgt=${tgt.map((v) => v.toFixed(1)).join(',')}`);

if (ppm) {
  const px = Buffer.alloc(FB_W * FB_H * 3);
  for (let i = 0; i < FB_W * FB_H; i++) {
    if (pal) { const c = fb[i] * 3; px[i * 3] = pal[c]; px[i * 3 + 1] = pal[c + 1]; px[i * 3 + 2] = pal[c + 2]; }
  }
  writeFileSync(ppm, Buffer.concat([Buffer.from(`P6\n${FB_W} ${FB_H}\n255\n`), px]));
  console.error(`wrote ${ppm}`);
}

// A frame that drew nothing must not be pinnable as "matching".
process.exit(geo > 0 ? 0 : 1);
