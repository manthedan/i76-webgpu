#!/usr/bin/env node
/*
 * gpu_mirror_probe.mjs — focused proof for the GPU rearview mirror pass
 * (_drawMirror in gpu_scene.mjs).
 *
 * What it proves (without claiming an adapter-backed result): the proof is
 * behavioral recording + pure composition. The browser full-frame gate is
 * the separate authority for acquisition, pipeline execution and readback.
 *
 *   1. ORIENTATION (pure math): the VP flip applied to the mirror pass
 *      (flipVpX) projects every point to the horizontally mirrored pixel —
 *      sx' = w - sx — the exact GPU twin of the software composite's
 *      dst[x] = mirror_fb[mw-1-x] (webmain.c). flip∘flip = identity.
 *
 *   2. PASS BEHAVIOR (recording mock, no GPU device): the REAL
 *      GpuScene.prototype._drawMirror + _pipesFor + _frameUniforms run
 *      against a recording mock device/encoder. Asserts:
 *      - the uniform block written to the mirror UBO carries the x-flipped
 *        VP (the old code wrote mirror.vp unflipped — fails);
 *      - the pass uses the fs_*_mirror pipelines whose frontFace is
 *        inverted relative to the primary variants (the old code used the
 *        primary pipelines — fails);
 *      - the mirror fragment entry points exist in the exported WGSL and
 *        clip through the HUD overlay (static consistency — WGSL cannot be
 *        validated headless here);
 *      - viewport+scissor are exactly the wasm-owned mirror rect, and
 *        interior/hidden instances stay out of the mirror.
 *
 *   3. COMPOSITION (deterministic, real engine data): a reference
 *      compositor implementing the GPU mirror semantics — scissor to the
 *      rect, horizontal flip, write only where the HUD layer is
 *      transparent — over the REAL N01 HUD layer / pre-HUD frame / mirror
 *      rect from wasm. Asserts the HUD layer carries the authored mirror
 *      housing INSIDE the rect (nonzero coverage — webmain.c copies the
 *      zmiri mask's non-0xFF housing texels into the layer's transparent
 *      pixels when the rearview presents, so HUD opacity is exactly the
 *      authored glass cutout), that those housing pixels are never
 *      overwritten (old rectangle overwrite — fails), that glass pixels
 *      carry the FLIPPED content (old unflipped — fails), that pixels
 *      outside the rect are untouched, and cross-checks the HUD-wins rule
 *      against the software reference frame. A discrimination block proves
 *      the OLD semantics (unflipped rectangle) violate these checks.
 *
 * MASK PATH: hud_mirror_mask (zmiri101.map) stays C-internal — no JS
 * export — but it no longer needs one: its housing texels arrive through
 * web_gpu_hud_layer itself, so the GPU u_hud clip is the authored glass
 * cutout and the HUD pass paints the housing ring, matching the software
 * composite bezel-for-bezel.
 *
 * Run:  node gpu_mirror_probe.mjs
 *       GPU_SCENE_SRC=./other.mjs node gpu_mirror_probe.mjs   (variant)
 */
const SRC = process.env.GPU_SCENE_SRC ?? './gpu_scene.mjs';
const {
  GpuScene, SCENE_WGSL, flipVpX, flipFront, mirrorBasis, cameraViewProj,
  projectPoint, readRearview, bootEngine, heap8, FB_W, FB_H,
} = await import(SRC);

let failures = 0;
const check = (name, cond, detail = '') => {
  if (cond) console.log(`ok   ${name}${detail ? ' — ' + detail : ''}`);
  else { console.error(`FAIL ${name}${detail ? ' — ' + detail : ''}`); failures++; }
};
const info = (msg) => console.log(`info ${msg}`);

/* ------------------------------------------------------------------ */
/* 1. Pure orientation math                                            */
/* ------------------------------------------------------------------ */

{
  // A rearward-looking camera with a non-trivial basis (rolled slightly).
  const cam = { eye: [12.5, 3.0, -40.0], right: [-0.99, 0.05, 0.12],
    up: [0.05, 0.99, -0.06], forward: [-0.12, 0.02, -0.99] };
  const vp = cameraViewProj(cam, 256, 64);
  const fvp = flipVpX(vp);

  let mirrored = true, maxErr = 0;
  for (const p of [[0, 0, 60], [30, 4, 80], [-45, -2, 120], [7, 15, 200],
                   [120, -8, 90]]) {
    const [sx, sy, zs] = projectPoint(vp, p, 256, 64);
    const [fsx, fsy, fzs] = projectPoint(fvp, p, 256, 64);
    maxErr = Math.max(maxErr, Math.abs(fsx - (256 - sx)));
    // 1e-4 px: flipVpX stores float32 (the UBO's precision), so allow the
    // float64->float32 rounding of the matrix itself.
    if (Math.abs(fsx - (256 - sx)) > 1e-4 || Math.abs(fsy - sy) > 1e-4 ||
        Math.abs(fzs - zs) > 1e-4) mirrored = false;
  }
  check('flipped VP projects to the horizontally mirrored pixel (sx=w-sx)',
        mirrored, `maxErr=${maxErr.toExponential(2)}`);

  const twice = flipVpX(fvp);
  check('flip is an involution (flip∘flip = identity)',
        [...vp].every((v, i) => Math.abs(v - twice[i]) < 1e-6));

  // The flip must not be a no-op on a real VP (guards a degenerate helper).
  check('flip actually changes the VP', [...vp].some((v, i) => v !== fvp[i]));

  // Concrete reflection: a point right of the optical axis in the unflipped
  // view lands the same distance LEFT of center in the mirror.
  const [sx0] = projectPoint(vp, [30, 4, 80], 256, 64);
  const [fsx0] = projectPoint(fvp, [30, 4, 80], 256, 64);
  check('right-of-axis content reflects to left-of-axis',
        (sx0 - 128) * (fsx0 - 128) < 0 &&
        Math.abs(Math.abs(sx0 - 128) - Math.abs(fsx0 - 128)) < 1e-6,
        `sx=${sx0.toFixed(2)} -> ${fsx0.toFixed(2)} (center 128)`);

  check('flipFront inverts winding', flipFront('ccw') === 'cw' &&
        flipFront('cw') === 'ccw');
}

/* ------------------------------------------------------------------ */
/* 2. Behavioral recording of the real _drawMirror                     */
/* ------------------------------------------------------------------ */

{
  const rec = { writes: [], pipelines: [], pass: [] };
  const stub = Object.create(GpuScene.prototype);
  stub.device = {
    queue: {
      writeBuffer: (buf, off, data) =>
        rec.writes.push({ buf, off, data: Float32Array.from(data) }),
    },
  };
  stub.canvasFormat = 'rgba8unorm';
  stub._pipes = new Map();
  stub._mk = (layout, vs, fs, format, buffers, cull = 'back', depth = null,
              front = 'ccw') => {
    const p = { layout, vs, fs, format, cull, front };
    rec.pipelines.push(p);
    return p;
  };
  stub._layFull = 'layFull'; stub._layFlat = 'layFlat';
  stub._terrBufs = 'terrBufs'; stub._meshBufs2 = 'meshBufs';
  stub._depthStencil = 'depthStencil';
  stub.terrFront = 'cw'; stub.meshFront = 'ccw';
  stub.tileM = 100; stub.fogStart = 900; stub.fogEnd = 2600;
  stub.lightAmb = 0.45; stub.sky = [0.5, 0.4, 0.3];
  stub.lightDir = [0.48, 0.78, 0.40]; stub.zenith = [0.2, 0.2, 0.35];
  stub.mirrorUbo = 'mirrorUbo'; stub.mirrorBind0 = 'mirrorBind0';
  stub.camBasis = {
    right: [1, 0, 0], up: [0, 1, 0], forward: [0, 0, 1],
    focal: 320, cx: 320, cy: 240,
  };
  stub.skyOn = true;
  stub.depthView = 'depthView';
  stub.terrain = { vbo: 'tv', ibo: 'ti', nIdx: 12 };
  stub.terrainBind1 = 'tb1'; stub.terrainBind2 = 'tb2';
  const mkInst = (kind, hidden) => ({
    draw: { hidden, kind }, vbo: `v-${kind}`,
    groups: [{ bind1: `b-${kind}-${hidden}-0`, tex: { bind: 'gtex' },
               count: 3, first: 0 },
             { bind1: `b-${kind}-${hidden}-1`, tex: null, count: 6, first: 3 }],
  });
  stub.instances = [mkInst('scene', false), mkInst('scene', true),
                    mkInst('interior', false)];

  const pass = new Proxy({}, {
    get: (t, name) => (...args) => rec.pass.push([name, ...args]),
  });
  const enc = { beginRenderPass: (desc) => { rec.passDesc = desc; return pass; } };

  const cam = { eye: [0, 2, 0], right: [-1, 0, 0], up: [0, 1, 0],
                forward: [0, 0, -1] };
  const mirror = { vp: Float32Array.from(cameraViewProj(cam, 256, 64)),
                   cam, x: 360, y: 64, w: 256, h: 64 };

  GpuScene.prototype._drawMirror.call(stub, enc, 'colorView', mirror);

  // -- the uniform block carries the FLIPPED vp -------------------------
  const w0 = rec.writes[0];
  const expectFlip = flipVpX(mirror.vp);
  const isFlip = w0 && w0.buf === 'mirrorUbo' &&
    [...expectFlip].every((v, i) => Math.abs(v - w0.data[i]) < 1e-6);
  const isRaw = w0 &&
    [...mirror.vp].every((v, i) => Math.abs(v - w0.data[i]) < 1e-6);
  check('mirror UBO carries the x-flipped VP (not the raw rearview VP)',
        isFlip && !isRaw,
        isFlip ? 'vp row0 negated' :
          isRaw ? 'OLD BEHAVIOR: raw rearview VP written (unflipped image)'
                : 'unexpected uniform payload');

  const expectBasis = mirrorBasis(cam, mirror);
  const basisOk = w0 &&
    [...expectBasis.right].every((v, i) => Math.abs(v - w0.data[32 + i]) < 1e-6) &&
    [...expectBasis.up].every((v, i) => Math.abs(v - w0.data[36 + i]) < 1e-6) &&
    [...expectBasis.forward].every((v, i) => Math.abs(v - w0.data[40 + i]) < 1e-6) &&
    Math.abs(expectBasis.focal - w0.data[44]) < 1e-6 &&
    Math.abs(expectBasis.cx - w0.data[45]) < 1e-6 &&
    Math.abs(expectBasis.cy - w0.data[46]) < 1e-6;
  check('mirror UBO carries the reflected rear-camera sky basis', basisOk);

  // -- the pass uses the mirror pipelines -------------------------------
  const pipeCalls = rec.pass.filter(([n]) => n === 'setPipeline')
    .map(([, p]) => p);
  check('pass sets sky/terrain/mesh MIRROR pipelines in order',
        pipeCalls.length === 3 &&
        pipeCalls[0].fs === 'fs_sky_mirror' &&
        pipeCalls[1].fs === 'fs_terrain_mirror' &&
        pipeCalls[2].fs === 'fs_mesh_mirror',
        pipeCalls.map((p) => p.fs).join(',') ||
          'OLD BEHAVIOR: primary pipelines (unclipped overwrite)');

  // -- winding compensation ----------------------------------------------
  const prim = Object.fromEntries(rec.pipelines.map((p) => [p.fs, p]));
  check('mirror pipelines invert frontFace relative to the primary passes',
        prim.fs_terrain_mirror.front === flipFront(prim.fs_terrain.front) &&
        prim.fs_mesh_mirror.front === flipFront(prim.fs_mesh.front),
        `terr ${prim.fs_terrain?.front}->${prim.fs_terrain_mirror?.front}, ` +
        `mesh ${prim.fs_mesh?.front}->${prim.fs_mesh_mirror?.front}`);

  const materialBinds = rec.pass
    .filter(([name, slot]) => name === 'setBindGroup' && slot === 1)
    .map(([, , bind]) => bind);
  check('mirror binds each visible material group independently',
        materialBinds.includes('b-scene-false-0') &&
        materialBinds.includes('b-scene-false-1'));

  // -- WGSL consistency (static; browser gate owns live validation) -------
  const mirrorFns = ['fs_sky_mirror', 'fs_terrain_mirror', 'fs_mesh_mirror'];
  check('mirror fragment entries exist in SCENE_WGSL',
        mirrorFns.every((f) => new RegExp(`fn\\s+${f}\\b`).test(SCENE_WGSL)));
  const clipCalls = (SCENE_WGSL.match(/hud_covers\(fpos\)/g) || []).length;
  check('every mirror fragment clips through the HUD overlay',
        clipCalls === 6, `${clipCalls} hud_covers call sites`);
  /* H-UAT-063 GPU divergence: a textured opaque group always takes sampled
   * RGB; only material mode 2 consults alpha and discards. The SW defect was
   * its unrelated face-flat fallback for source 0xFF — this shader has no
   * equivalent fallback and therefore already renders decoded 0xFF RGB dark. */
  check('opaque GPU textures use sampled RGB; only cutout mode discards alpha',
        /base\s*=\s*t\.rgb\s*;/.test(SCENE_WGSL) &&
        /u_model\.color\.a\s*>\s*1\.5\s*&&\s*t\.a\s*<\s*0\.5\)\s*\{\s*discard/.test(SCENE_WGSL));

  // -- confinement + exclusions ------------------------------------------
  const vpCall = rec.pass.find(([n]) => n === 'setViewport');
  const scCall = rec.pass.find(([n]) => n === 'setScissorRect');
  check('viewport and scissor are exactly the wasm-owned mirror rect',
        vpCall && scCall &&
        vpCall.slice(1, 5).join() === '360,64,256,64' &&
        scCall.slice(1).join() === '360,64,256,64');
  const draws = rec.pass.filter(([n]) => n === 'draw').length;
  const indexed = rec.pass.filter(([n]) => n === 'drawIndexed').length;
  check('hidden and interior instances stay out of the mirror',
        draws === 1 + 2 && indexed === 1,
        `${draws} draw + ${indexed} drawIndexed (sky + 1 visible scene inst)`);
}

/* ------------------------------------------------------------------ */
/* 3. Deterministic composition coverage on real engine data           */
/* ------------------------------------------------------------------ */

/*
 * Reference compositor for the GPU mirror semantics, in the palette-index
 * domain: primary = HUD-over-world composite; inside the rect the mirror
 * writes the FLIPPED rearview content exactly where the HUD layer is
 * transparent. This is the contract the fs_*_mirror fragments implement.
 */
function compositeGpuMirror(preHud, hud, rect, content) {
  const [rx, ry, mw, mh] = rect;
  const out = Uint8Array.from(preHud);
  for (let y = 0; y < FB_H; y++)
    for (let x = 0; x < FB_W; x++) {
      const i = y * FB_W + x;
      if (hud[i] !== 0) { out[i] = hud[i]; continue; }   // HUD wins
      if (x >= rx && x < rx + mw && y >= ry && y < ry + mh)
        out[i] = content[(y - ry) * mw + (mw - 1 - (x - rx))];  // reflected
    }
  return out;
}
/* The old behavior: the mirror pass ran AFTER the HUD pass and stamped an
 * unflipped rectangle over the whole rect — cockpit art included. */
function compositeOldMirror(preHud, hud, rect, content) {
  const [rx, ry, mw, mh] = rect;
  const out = Uint8Array.from(preHud);
  for (let i = 0; i < FB_W * FB_H; i++) if (hud[i] !== 0) out[i] = hud[i];
  for (let y = 0; y < mh; y++)
    for (let x = 0; x < mw; x++)
      out[(ry + y) * FB_W + rx + x] = content[y * mw + x];
  return out;
}

const { readFileSync, existsSync } = await import('node:fs');
const { join } = await import('node:path');
const appDir = process.env.NITRO_APP;
if (!appDir || !existsSync(join(appDir, 'nitro.zfs'))) {
  console.error('FAIL engine composition prerequisite — set NITRO_APP explicitly');
  check('game data present', false, appDir || '(unset)');
} else {
  const io = { fetchAsset: async (rel) => {
    try { return new Uint8Array(readFileSync(join(appDir, rel))); }
    catch { return null; }
  } };
  const { M, rc } = await bootEngine(io);
  check('engine boot + drive load (N01/vdrampg2)', rc === 0, `rc=${rc}`);

  if (rc === 0) {
    // Mask path fact: no direct JS mask export — the housing art arrives
    // through the HUD layer instead (webmain.c bezel-to-HUD composition).
    const maskExports = Object.keys(M).filter((k) =>
      /mask|bezel/i.test(k) && k.startsWith('_web'));
    info(`mask exports to JS: ${maskExports.length ? maskExports.join(',') :
      'NONE — the authored zmiri housing reaches the GPU through ' +
      'web_gpu_hud_layer (see header)'}`);

    M._web_drive_set_view(0);                     // cockpit owns the frame
    for (let i = 0; i < 5; i++) M._web_drive_step();
    M._web_drive_render();
    check('rearview active in the cockpit', M._web_rearview_active() === 1);

    const rv = readRearview(M);
    check('rearview camera + rect exported', !!rv,
          rv ? `rect=${rv.rect}` : 'null');
    if (rv) {
      const [rx, ry, mw, mh] = rv.rect;
      check('mirror rect inside the framebuffer',
            rx >= 0 && ry >= 0 && rx + mw <= FB_W && ry + mh <= FB_H &&
            mw > 0 && mh > 0, `${rx},${ry} ${mw}x${mh}`);
      info(`rect ${mw}x${mh}: 256x64 means the authored zmiri mask IS loaded ` +
           'C-side (its size feeds web_rearview_rect) while staying ' +
           'unexported to JS');

      const hudP = M._web_gpu_hud_layer();
      const preP = M._web_fb_pre_hud();
      check('HUD layer + pre-HUD frame available', !!hudP && !!preP);
      if (hudP && preP) {
        const hud = heap8(M).slice(hudP, hudP + FB_W * FB_H);
        const preHud = heap8(M).slice(preP, preP + FB_W * FB_H);

        // Synthetic asymmetric mirror content: column-varying, nonzero.
        const content = new Uint8Array(mw * mh);
        for (let y = 0; y < mh; y++)
          for (let x = 0; x < mw; x++)
            content[y * mw + x] = ((x * 7 + y * 13) % 254) + 1;

        const out = compositeGpuMirror(preHud, hud, rv.rect, content);
        const outOld = compositeOldMirror(preHud, hud, rv.rect, content);

        // Buckets inside the rect: housing (HUD-opaque, thanks to the
        // webmain.c bezel-to-HUD composition) vs glass (HUD-transparent).
        let hudPx = 0, glassPx = 0, orientOk = 0, clipOk = 0;
        let oldOrientBad = 0;
        for (let y = ry; y < ry + mh; y++)
          for (let x = rx; x < rx + mw; x++) {
            const i = y * FB_W + x;
            const cx = x - rx, cy = y - ry;
            if (hud[i] !== 0) {
              hudPx++;
              if (out[i] === hud[i]) clipOk++;
            } else {
              glassPx++;
              if (out[i] === content[cy * mw + (mw - 1 - cx)]) orientOk++;
              if (outOld[i] !== content[cy * mw + (mw - 1 - cx)])
                oldOrientBad++;
            }
          }
        check('mirror content is horizontally reflected (flip, not copy)',
              glassPx > 0 && orientOk === glassPx,
              `${orientOk}/${glassPx} glass (HUD-transparent) rect px`);
        check('HUD layer carries the authored housing inside the rect',
              hudPx > 0,
              `${hudPx} opaque px (zmiri non-0xFF texels via webmain.c)`);
        check('real housing pixels inside the rect are never overwritten',
              clipOk === hudPx, `${clipOk}/${hudPx}`);

        // Outside the rect the mirror must not touch anything.
        let outsideOk = true;
        for (let y = 0; y < FB_H && outsideOk; y += 3)
          for (let x = 0; x < FB_W; x += 3) {
            if (x >= rx && x < rx + mw && y >= ry && y < ry + mh) continue;
            const i = y * FB_W + x;
            const prim = hud[i] !== 0 ? hud[i] : preHud[i];
            if (out[i] !== prim) { outsideOk = false; break; }
          }
        check('pixels outside the rect are untouched', outsideOk);

        // Discrimination: the OLD semantics violate these same checks.
        check('the old unflipped rectangle FAILS the orientation check',
              oldOrientBad > 0, `${oldOrientBad} wrongly-oriented px`);

        // HUD-clip rule, deterministic: a synthetic HUD overlay that DOES
        // reach into the rect (left band + bottom row) proves HUD-opaque
        // pixels are never overwritten while the rest stays flipped.
        {
          const hud2 = new Uint8Array(FB_W * FB_H);
          // 255: outside the content ramp's range, so a clobber can never
          // accidentally reproduce the HUD value.
          for (let y = ry; y < ry + mh; y++)
            for (let x = rx; x < rx + 24; x++) hud2[y * FB_W + x] = 255;
          for (let x = rx; x < rx + mw; x++)
            hud2[(ry + mh - 1) * FB_W + x] = 255;
          let covered = 0;
          for (const v of hud2) if (v !== 0) covered++;
          const out2 = compositeGpuMirror(preHud, hud2, rv.rect, content);
          const out2Old = compositeOldMirror(preHud, hud2, rv.rect, content);
          let keepOk = 0, keepBad = 0, flipOk = 0, flipN = 0;
          for (let y = ry; y < ry + mh; y++)
            for (let x = rx; x < rx + mw; x++) {
              const i = y * FB_W + x;
              if (hud2[i] !== 0) {
                if (out2[i] === hud2[i]) keepOk++; else keepBad++;
              } else {
                flipN++;
                if (out2[i] === content[(y - ry) * mw + (mw - 1 - (x - rx))])
                  flipOk++;
              }
            }
          check('HUD-covered rect pixels keep the cockpit art (clip rule)',
                keepBad === 0 && keepOk === covered,
                `${keepOk}/${covered} protected`);
          check('uncovered rect pixels still carry the reflected content',
                flipOk === flipN && flipN > 0, `${flipOk}/${flipN}`);
          let oldClipBad = 0;
          for (let i = 0; i < FB_W * FB_H; i++)
            if (hud2[i] !== 0 && out2Old[i] !== hud2[i]) oldClipBad++;
          check('the old rectangle FAILS the HUD-clip check',
                oldClipBad === covered, `${oldClipBad} overwritten HUD px`);
        }

        // Cross-check against the software reference: where the HUD layer
        // is opaque (dash art + the authored mirror housing), the final
        // software frame carries exactly that index — the same HUD-wins
        // rule the GPU mirror fragments implement.
        const fbP = M._web_fb();
        check('final software frame available for the cross-check', !!fbP);
        if (fbP) {
          const fin = heap8(M).slice(fbP, fbP + FB_W * FB_H);
          let refOk = 0;
          for (let y = ry; y < ry + mh; y++)
            for (let x = rx; x < rx + mw; x++) {
              const i = y * FB_W + x;
              if (hud[i] !== 0 && fin[i] === hud[i]) refOk++;
            }
          check('software reference agrees: housing/HUD wins inside the ' +
                'mirror rect', refOk === hudPx, `${refOk}/${hudPx}`);
        }
        info(`rect composition: ${glassPx} glass px show the reflected ` +
             `world, ${hudPx} housing px carry the authored ring art`);
      }
    }
  }
}

if (failures) { console.error(`RESULT: FAIL (${failures})`); process.exit(1); }
console.log('RESULT: PASS');
