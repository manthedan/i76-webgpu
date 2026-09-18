/*
 * Browser-only runner for the portable GPU verification page.
 *
 * The production renderer and extraction helpers stay in ../gpu_scene.mjs.
 * This module owns test-only asset fetching, fixed-frame orchestration and
 * evidence-canvas presentation. It requires an explicit ?app= URL.
 */
import {
  FB_W, FB_H, IOU_DILATE_R, RAW_IOU_THRESHOLD,
  SILHOUETTE_IOU_THRESHOLD, CORNER_TOL_PX, GpuScene, applySoftwareLight,
  bootEngine, cameraViewProj, collectDraws, columnFill, cornerCheck, dilate,
  downsampleRgba, fnv1aHex, fullFrameDiff, heap8, indexedFrameToRgba,
  maskFromRgba, maskIou, readCamera, readOverlay, readPalette, readRoads,
  readSkyColor, readSkyTex, readSkyZenith, readTerrainGrid, readTerrainLod,
  readTerrainTexture, readVisibilityCounts, refreshDraws, renderTargetSize,
  silhouetteDelta, terrainCorners, worldDepthMask,
} from '../gpu_scene.mjs';

function paintRgbaCanvas(canvas, rgba, w = FB_W, h = FB_H,
                         forceOpaque = false) {
  if (!canvas || !rgba) return;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  const image = ctx.createImageData(w, h);
  image.data.set(rgba);
  /* WebGPU alpha is retained as a geometry-coverage channel. Force opacity
   * only in this diagnostic canvas so valid RGB is visible; comparisons and
   * coverage metrics continue to use the untouched readback. */
  if (forceOpaque)
    for (let i = 3; i < image.data.length; i += 4) image.data[i] = 255;
  ctx.putImageData(image, 0, 0);
}

export async function run(doc = document) {
  const banner = doc.getElementById('banner');
  const resultEl = doc.getElementById('result');
  const patterns = [];
  const finish = (res, status) => {
    res.status = status;
    doc.title = `gpu scene probe — RESULT: ${status}`;
    const gated = patterns.filter((p) => !p.skipped);
    banner.textContent = status === 'SKIP'
      ? `RESULT: SKIP — ${res.reason || 'WebGPU unavailable'}`
      : `RESULT: ${status} — ${gated.filter((p) => p.match).length}/${gated.length} gated patterns match`;
    banner.dataset.status = status;
    banner.className = status.toLowerCase();
    resultEl.textContent = JSON.stringify(res, null, 2);
    window.__gpuSceneProbe = res;   // structured browser-driver result
  };

  if (typeof navigator === 'undefined' || !navigator.gpu) {
    finish({ webgpu: false, reason: 'navigator.gpu absent', patterns }, 'SKIP');
    return;
  }

  try {
    const adapter = await navigator.gpu.requestAdapter();
    if (!adapter) {
      finish({ webgpu: false, reason: 'requestAdapter() null', patterns }, 'SKIP');
      return;
    }
    let device;
    try {
      device = await adapter.requestDevice();
    } catch (e) {
      finish({ webgpu: false,
        reason: `requestDevice() failed: ${String(e)}`, patterns }, 'SKIP');
      return;
    }
    const info = adapter.info
      ? [adapter.info.vendor, adapter.info.architecture,
         adapter.info.description, adapter.info.backend,
         adapter.info.type].filter(Boolean).join(' ')
      : 'unknown';
    const gpuErrors = [];
    device.addEventListener?.('uncapturederror', (event) => {
      const message = String(event.error?.message || event.error || 'unknown WebGPU error');
      gpuErrors.push(message);
      console.error(`WebGPU uncaptured error: ${message}`);
    });

    // Test data is available only through the caller-provided asset route.
    const params = new URLSearchParams(location.search);
    const base = params.get('app');
    if (!base) {
      patterns.push({ name: 'assets', skipped: false, match: false,
        note: 'missing required ?app= asset route' });
      finish({ webgpu: true, adapter: info, match: false, patterns }, 'FAIL');
      return;
    }
    try {
      const probe = await fetch(base + 'nitro.zix', { method: 'HEAD' });
      if (!probe.ok) throw new Error(`HTTP ${probe.status}`);
    } catch (e) {
      patterns.push({ name: 'assets', skipped: false, match: false,
        note: `explicit ?app= route unavailable: ${String(e)}` });
      finish({ webgpu: true, adapter: info, match: false, patterns }, 'FAIL');
      return;
    }
    const io = {
      fetchAsset: async (rel) => {
        const r = await fetch(base + rel);
        return r.ok ? new Uint8Array(await r.arrayBuffer()) : null;
      },
    };
    const mission = params.get('mission') || 'miss8/N01.CBT';
    const viewWant = params.get('view') || 'chase';
    const gpuRes = params.get('gpures') || 'fidelity';
    if (!['fidelity', 'native'].includes(gpuRes))
      throw new Error(`gpures must be fidelity or native, got ${gpuRes}`);
    const { M, rc } = await bootEngine(io, { mission });
    if (rc !== 0) {
      patterns.push({ name: 'engine-boot', skipped: true, match: false,
        note: `init/drive rc=${rc} mission=${mission}` });
      finish({ webgpu: true, adapter: info, match: false, patterns }, 'FAIL');
      return;
    }
    window.__i76 = { M, frames: 0, mission };

    // A — .m16 decode gates (m16.c through the web_gpu_m16_* exports):
    // a vehicle tile via its .pix manifest, and a loose zone tile.
    for (const spec of [['a4tank16.pak#0', 'pak tile'], ['zhr45101.m16', 'loose tile']]) {
      const okRc = M.ccall('web_gpu_m16_load', 'number', ['string'], [spec[0]]) === 0;
      let nonUniform = false, w = 0, h = 0;
      if (okRc) {
        w = M._web_gpu_m16_w(); h = M._web_gpu_m16_h();
        const p = M._web_gpu_m16_rgba();
        const rgba = heap8(M).subarray(p, p + w * h * 4);
        const first = [rgba[0], rgba[1], rgba[2]];
        for (let i = 4; i < rgba.length; i += 4)
          if (rgba[i] !== first[0] || rgba[i + 1] !== first[1] ||
              rgba[i + 2] !== first[2]) { nonUniform = true; break; }
      }
      patterns.push({ name: `m16-${spec[0]}`, kind: spec[1], w, h,
        match: okRc && w > 0 && h > 0 && nonUniform });
    }

    // Fixed promotion-ladder frame at the engine-owned camera. Overlay cases
    // queue the ordinary stock input on the final tick so both renderers read
    // one simulation/render snapshot with the authored paper state active.
    const fixedTick = Math.max(1, Math.min(1000,
      Number.parseInt(params.get('tick') || '60', 10) || 60));
    const paperWant = params.get('paper') || 'none';
    const damageWant = params.get('damage') === '1';
    const fxCanary = params.get('fxcanary') || '';
    if (!['none', 'title', 'map'].includes(paperWant))
      throw new Error(`unsupported paper case: ${paperWant}`);
    let damageArmed = false;
    for (let i = 0; i < fixedTick; i++) {
      if (paperWant === 'map' && i === fixedTick - 1) {
        const PK_M = 36; // platform.h stock show_map binding
        M._web_key_event(PK_M, 1);
        M._web_key_event(PK_M, 0);
      }
      if (damageWant && i === fixedTick - 1) {
        damageArmed = M.ccall('web_damage_probe_hit', 'number',
                              ['number', 'number'], [1, 8]) === 0;
      }
      M._web_drive_step();
    }
    if (damageWant) {
      const combat = JSON.parse(M.ccall('web_combat_state', 'string', [], []));
      patterns.push({ name: 'damage-hit', armed: damageArmed,
        hp: combat.hp, hpMax: combat.hpMax,
        match: damageArmed && combat.hp < combat.hpMax });
    }
    let fxCanaryArmed = false;
    if (fxCanary) {
      const pose = JSON.parse(M.ccall('web_drive_pose', 'string', [], []));
      const distance = 18.0;
      fxCanaryArmed = M.ccall('web_audit_fx_impact', 'number',
        ['string', 'number', 'number', 'number'],
        [fxCanary, pose.x - Math.sin(pose.yaw) * distance,
          pose.y + 1.0, pose.z + Math.cos(pose.yaw) * distance]) === 0;
    }
    const paperActive = M._web_paper_active ? M._web_paper_active() : 0;
    const titleActive = M._web_paper_title_active
      ? M._web_paper_title_active() : 0;
    if (paperWant !== 'none') patterns.push({
      name: 'paper-state', requested: paperWant, paperActive, titleActive,
      match: paperWant === 'title' ? titleActive === 1 : paperActive === 1,
    });
    if (M._web_drive_set_view) {
      // 0=cockpit 1=chase (webmain DRIVE_VIEW_*)
      M._web_drive_set_view(viewWant === 'cockpit' ? 0 : 1);
    }
    M._web_drive_render();

    /* Freeze every software-owned input immediately after the one authoritative
     * render. The old probe sampled the final framebuffer early but deferred
     * other mutable wasm-owned inputs until after GPU material setup. Copy the
     * complete SW frame, overlay, palette, and depth at one boundary, then
     * assert that no simulation tick can enter before GPUTexture readback. */
    const capturePose = JSON.parse(M.ccall(
      'web_drive_pose', 'string', [], []));
    const captureTick = capturePose.tick;
    const swFinalPtr = M._web_fb();
    const swFinal = heap8(M).slice(swFinalPtr, swFinalPtr + FB_W * FB_H);
    const palette = readPalette(M);
    const swRgba = indexedFrameToRgba(swFinal, palette);
    const swMask = worldDepthMask(M);
    if (!swMask) throw new Error('software world depth unavailable');
    const overlay = readOverlay(M);
    if (!overlay) throw new Error('software overlay unavailable');
    const fxAudit = M._web_audit_render_state
      ? JSON.parse(M.ccall('web_audit_render_state', 'string', [], [])) : null;
    const requireFx = params.get('requirefx') === '1';
    patterns.push({
      name: 'combat-fx-overlay', required: requireFx, ...(fxAudit || {}),
      fxCanary, fxCanaryArmed,
      match: !requireFx || (!!fxAudit && fxAudit.fxEvents > 0 &&
        fxAudit.swFx === 1 && fxAudit.gpuFx === 1 &&
        (!fxCanary || (fxCanaryArmed && fxAudit.impactAuthored > 0 &&
          fxAudit.impactMissing === 0))),
      note: 'WebGPU consumes the exact software post-world FX overlay; SW remains authority',
    });

    // B — corner projections: JS matrix vs wasm software camera, ±2 px.
    const grid = readTerrainGrid(M);
    const lod = readTerrainLod(M);
    const cam = readCamera(M);
    /* Fidelity probes stay exactly 640x480. Native probes use the physical
     * display size of the evidence canvas (DPR-aware, bounded at 2x). */
    const gpuCanvas = doc.getElementById('gpu');
    const target = gpuRes === 'native'
      ? renderTargetSize(gpuRes, gpuCanvas.clientWidth || FB_W,
                         gpuCanvas.clientHeight || FB_H,
                         globalThis.devicePixelRatio || 1)
      : renderTargetSize('fidelity');
    gpuCanvas.width = target.w;
    gpuCanvas.height = target.h;
    const responsiveTarget = renderTargetSize('native', 533, 400, 1.25);
    const cappedTarget = renderTargetSize('native', 640, 480, 3);
    patterns.push({
      name: 'render-target-contract', responsiveTarget, cappedTarget,
      match: responsiveTarget.w % 4 === 0 && responsiveTarget.h % 3 === 0 &&
        responsiveTarget.w / responsiveTarget.h === FB_W / FB_H &&
        responsiveTarget.w <= 533 * 1.25 && responsiveTarget.h <= 400 * 1.25 &&
        cappedTarget.w === 1280 && cappedTarget.h === 960 && cappedTarget.capped,
    });
    const rw = target.w, rh = target.h;
    const vp = cameraViewProj(cam, rw, rh);
    const corners = terrainCorners(grid);
    const checks = cornerCheck(M, vp, corners ?? [], rw, rh);
    const live = checks.filter((c) => !c.clipped);
    patterns.push({
      name: 'terrain-corner-projection',
      checks: checks.map((c) => ({ js: c.js.map((v) => +v.toFixed(2)),
        c: c.c.map((v) => +v.toFixed(2)), dx: +c.dx.toFixed(3),
        dy: +c.dy.toFixed(3), zs: +c.zs.toFixed(1), clipped: c.clipped })),
      checked: live.length,
      match: live.length > 0 &&
        live.every((c) => c.dx <= CORNER_TOL_PX && c.dy <= CORNER_TOL_PX),
    });

    // B2 — visibility membership (shared draw counts).
    const vis = readVisibilityCounts(M);
    patterns.push({
      name: 'visibility-membership',
      ...vis,
      lodBands: lod && lod.bands,
      match: !!vis && vis.carParts > 0 && (lod ? lod.nverts > 0 : true),
    });

    // C — coverage IoU: software pre-HUD fb vs GPU readback.
    const scene = collectDraws(M);
    if (!scene)
      throw new Error('web_gpu_drawlist_build missing or failed — the required GPU scene bridge is unavailable');
    const tex = readTerrainTexture(M);

    // ?present=0 renders offscreen only. Headless Chrome destroys the device
    // when a WebGPU canvas swapchain is presented to with no compositor
    // surface ("A valid external Instance reference no longer exists"), which
    // would abort the geometric gate for a display-only reason. Offscreen
    // readback is exactly what the gate measures, so CI can opt out.
    const present = params.get('present') !== '0';
    /* The historical ?scale supersample remains fidelity-only. It is not a
     * product resolution selector and cannot perturb gate 32's default. */
    const supersample = gpuRes === 'fidelity'
      ? Math.max(1, Math.min(2, +(params.get('scale') || 1) || 1)) : 1;
    if (supersample > 1) {
      target.w = FB_W * supersample;
      target.h = FB_H * supersample;
    }
    const renderW = target.w, renderH = target.h;
    gpuCanvas.width = renderW;
    gpuCanvas.height = renderH;
    const gs = new GpuScene(device, present ? gpuCanvas : null,
                            readSkyColor(M), renderW, renderH, readSkyZenith(M),
                            false, { pointSampled: gpuRes === 'native' });
    applySoftwareLight(M, gs);
    const skyTex = readSkyTex(M);
    gs.setSkyTex(skyTex);
    patterns.push({
      name: 'sky-tex',
      w: skyTex ? skyTex.w : 0, h: skyTex ? skyTex.h : 0,
      match: !!skyTex && skyTex.w > 0,
    });
    if (lod && lod.nverts > 0) gs.setTerrainLod(lod.verts, lod.nverts, tex);
    else gs.setTerrain(grid, tex);
    const roads = readRoads(M);
    if (roads) gs.setRoads(roads);
    patterns.push({
      name: 'roads-stream',
      nverts: roads ? roads.nverts : 0,
      byType: roads ? roads.byType : null,
      match: !!roads,  // zero verts ok (mission with no roads still builds)
    });
    gs.setInstances(scene, M);
    patterns.push({
      name: 'drawlist-stream',
      stream: scene.stream,
      draws: scene.draws.length,
      match: scene.stream === 'drawlist' && scene.draws.length > 0,
    });
    // Complete software-owned 2-D composite: HUD/cockpit, combat feedback,
    // damage flash and paper surfaces, with an explicit index-0-safe mask.
    gs.setHud(overlay.indices, palette, overlay.mask);
    patterns.push({ name: 'overlay-layer', match: true });
    gs.setCameraBasis(cam);
    const vp32 = Float32Array.from(cameraViewProj(cam, renderW, renderH));
    const setupTick = JSON.parse(M.ccall(
      'web_drive_pose', 'string', [], [])).tick;
    const nativeRgba = await gs.frame(vp32, scene, true);
    const readbackTick = JSON.parse(M.ccall(
      'web_drive_pose', 'string', [], [])).tick;
    patterns.push({
      name: 'same-simulation-tick', requested: fixedTick,
      captureTick, setupTick, readbackTick,
      match: captureTick === fixedTick && setupTick === captureTick &&
        readbackTick === captureTick,
    });
    const perfMs = [];
    const perfFrames = Math.max(0, Math.min(20,
      Number.parseInt(params.get('perf') || '0', 10) || 0));
    for (let i = 0; i < perfFrames; i++) {
      const started = performance.now();
      await gs.frame(vp32, scene, false);
      await device.queue.onSubmittedWorkDone();
      perfMs.push(performance.now() - started);
    }
    const sortedPerf = [...perfMs].sort((a, b) => a - b);
    const perf = {
      samples: perfMs.length,
      medianMs: sortedPerf.length ? sortedPerf[Math.floor(sortedPerf.length / 2)] : null,
      p95Ms: sortedPerf.length ? sortedPerf[Math.min(sortedPerf.length - 1,
        Math.ceil(sortedPerf.length * 0.95) - 1)] : null,
      valuesMs: perfMs,
    };
    const rgba = (renderW === FB_W && renderH === FB_H)
      ? nativeRgba : downsampleRgba(nativeRgba, renderW, renderH, FB_W, FB_H);
    const gpuMask = maskFromRgba(rgba);
    const fullFrame = fullFrameDiff(swRgba, rgba);
    patterns.push({ name: 'full-frame-color', ...fullFrame,
      skipped: gpuRes !== 'fidelity',
      note: gpuRes === 'fidelity'
        ? 'SW is golden; tolerance is a PORT DECISION, not fidelity proof'
        : 'report only in native mode; gate 32 compares fidelity mode' });

    /* Exact upper-HUD contract. The broad 1%-outlier frame alarm is allowed
     * to tolerate world-raster drift and could therefore hide a narrow panel
     * regression. Every software-owned covered pixel in rows 0..127 must be
     * the exact palette RGB on WebGPU; transparent world pixels are excluded. */
    let hudBandCovered = 0, hudBandExact = 0;
    for (let y = 0; y < 128; y++)
      for (let x = 0; x < FB_W; x++) {
        const source = y * FB_W + x;
        if (overlay.mask[source] < 128) continue;
        const out = source * 4, pal = overlay.indices[source] * 3;
        hudBandCovered++;
        if (rgba[out] === palette[pal] &&
            rgba[out + 1] === palette[pal + 1] &&
            rgba[out + 2] === palette[pal + 2]) hudBandExact++;
      }
    patterns.push({
      name: 'hud-band-overlay', rows: [0, 127], covered: hudBandCovered,
      exact: hudBandExact, mismatches: hudBandCovered - hudBandExact,
      skipped: hudBandCovered <= 1000,
      match: hudBandCovered > 1000 && hudBandExact === hudBandCovered,
      note: hudBandCovered > 1000
        ? 'exact software-owned covered pixels; world/transparent pixels excluded'
        : 'no material upper HUD in this authored camera frame',
    });

    const overlayCovered = overlay
      ? overlay.mask.reduce((n, value) => n + (value >= 128 ? 1 : 0), 0) : 0;
    let overlayExact = 0, overlayChecked = 0;
    if (overlay) {
      const maxChecks = 12000;
      const stride = Math.max(1, Math.floor((FB_W * FB_H) / maxChecks));
      for (let source = 0; source < FB_W * FB_H; source += stride) {
        if (overlay.mask[source] < 128) continue;
        const sx = source % FB_W, sy = Math.floor(source / FB_W);
        const tx = Math.min(renderW - 1, Math.floor((sx + 0.5) * renderW / FB_W));
        const ty = Math.min(renderH - 1, Math.floor((sy + 0.5) * renderH / FB_H));
        const out = (ty * renderW + tx) * 4;
        const pal = overlay.indices[source] * 3;
        overlayChecked++;
        if (nativeRgba[out] === palette[pal] &&
            nativeRgba[out + 1] === palette[pal + 1] &&
            nativeRgba[out + 2] === palette[pal + 2]) overlayExact++;
      }
    }
    let nonBlack = 0, geometry = 0;
    for (let i = 0; i < renderW * renderH; i++) {
      const o = i * 4;
      if (nativeRgba[o] || nativeRgba[o + 1] || nativeRgba[o + 2]) nonBlack++;
      if (nativeRgba[o + 3]) geometry++;
    }
    patterns.push({
      name: 'native-presentation', skipped: gpuRes !== 'native',
      mode: gpuRes, target: { ...target, w: renderW, h: renderH },
      aspect: renderW / renderH, nonBlack, geometry,
      overlayCovered, overlayChecked, overlayExact,
      overlayExactFraction: overlayChecked ? overlayExact / overlayChecked : 0,
      pointSampled: gs.pointSampled,
      perf,
      match: gpuRes === 'native' && renderW > FB_W && renderH > FB_H &&
        Math.abs(renderW / renderH - FB_W / FB_H) < 1e-6 &&
        nonBlack > renderW * renderH * 0.5 && geometry > 1000 &&
        (overlayCovered === 0 || (overlayCovered > 1000 &&
          overlayChecked > 100 && overlayExact / overlayChecked > 0.995)) &&
        gs.pointSampled,
    });

    const raw = maskIou(swMask, gpuMask);
    const dSw = dilate(swMask, FB_W, FB_H, IOU_DILATE_R);
    const dGpu = dilate(gpuMask, FB_W, FB_H, IOU_DILATE_R);
    const dil = maskIou(dSw, dGpu);
    const sil = silhouetteDelta(swMask, gpuMask, FB_W, FB_H);
    // Paint the exact compared arrays. With ?present=0 both canvases remain
    // 2-D evidence surfaces, while `rgba` still came from GPUTexture readback.
    paintRgbaCanvas(doc.getElementById('ref'), swRgba);
    /* Evidence canvases stay CSS-sized. Native readback is downsampled only
     * for this diagnostic paint; nativeRgba above is the gated GPU result. */
    paintRgbaCanvas(doc.getElementById('gpu'), rgba, FB_W, FB_H, true);
    const silRegion = maskIou(columnFill(swMask, FB_W, FB_H),
                              columnFill(gpuMask, FB_W, FB_H));
    patterns.push({
      name: 'coverage-iou',
      silhouetteIou: +silRegion.iou.toFixed(4),
      rawIou: +raw.iou.toFixed(4), dilatedRawIou: +dil.iou.toFixed(4),
      dilateR: IOU_DILATE_R,
      thresholds: {
        raw: RAW_IOU_THRESHOLD,
        rawDilated: RAW_IOU_THRESHOLD,
        silhouetteReportOnly: SILHOUETTE_IOU_THRESHOLD,
      },
      swPixels: raw.a, gpuPixels: raw.b,
      silhouette: { meanDy: +sil.meanDy.toFixed(2), bothCols: sil.bothCols,
        onlyCols: sil.onlyCols, onlyFrac: +sil.colFrac.toFixed(3) },
      terrainTexFallback: tex.fallback,
      draws: scene.draws.length,
      // Filled-vs-filled: raw/dilated coverage is the hard bar. Silhouette
      // (columnFill) stays in the report for horizon-LOD diagnosis.
      skipped: gpuRes !== 'fidelity',
      match: raw.iou >= RAW_IOU_THRESHOLD && dil.iou >= RAW_IOU_THRESHOLD,
    });

    /*
     * H-UAT-044: keep one device/scene alive while the ordinary preset family
     * changes underneath it. Each preset gets three advancing simulation
     * frames through GPUTexture readback. A last-good canvas or one successful
     * switch cannot pass: every preset must produce at least two distinct
     * hashes, differ from the preceding preset, and emit no WebGPU error.
     *
     * This follows the production view-refresh lifecycle: persistent mission
     * textures/meshes stay resident while camera-dependent terrain/road
     * geometry and draw visibility refresh. Regressing to a whole-mission
     * reset makes the first-frame gap exceed the adapter-relative stall bar.
     */
    if (params.get('cycle') === 'views') {
      const cycleFrames = 3;
      const presets = [
        { preset: 2, key: 'F2', name: 'chase', view: 1 },
        { preset: 1, key: 'F1', name: 'cockpit', view: 0 },
        { preset: 3, key: 'F3', name: 'fixed-180', view: 2 },
        { preset: 12, key: 'F4', name: 'track', view: 4 },
        { preset: 11, key: 'F5', name: 'track-swapped', view: 4 },
        { preset: 4, key: 'F6', name: 'free-eye', view: 5 },
        { preset: 6, key: 'F7', name: 'fixed-0', view: 2 },
        { preset: 7, key: 'F8', name: 'fixed-rear', view: 2 },
        { preset: 8, key: 'F9', name: 'fixed-150', view: 2 },
        { preset: 10, key: 'F10', name: 'overhead', view: 3 },
        { preset: 5, key: 'F11', name: 'track-mode8-fallback', view: 4 },
        { preset: 9, key: 'unbound', name: 'fixed-30', view: 2 },
      ];
      const fnv1a = (bytes) => {
        let h = 0x811c9dc5;
        for (let i = 0; i < bytes.length; i++) {
          h ^= bytes[i];
          h = Math.imul(h, 0x01000193) >>> 0;
        }
        return `0x${h.toString(16).padStart(8, '0')}`;
      };
      const results = [];
      let previousHash = null;
      let liveScene = scene;
      const rebuild = () => {
        const refreshed = refreshDraws(M, liveScene);
        if (refreshed !== liveScene.count)
          throw new Error(`view-cycle drawlist changed ${liveScene.count}->${refreshed}`);
        const liveLod = readTerrainLod(M);
        if (liveLod && liveLod.nverts > 0)
          gs.setTerrainLod(liveLod.verts, liveLod.nverts);
        else
          gs.setTerrain(readTerrainGrid(M));
        const liveRoads = readRoads(M);
        if (liveRoads) gs.setRoads({ ...liveRoads, tex: null });
      };

      const PK_UP = 4; // platform.h; keep the world moving within each preset
      M._web_key_event(PK_UP, 1);
      try {
        for (const preset of presets) {
          const item = { ...preset, hashes: [], cameras: [] };
          try {
            const switchAt = performance.now();
            M._web_drive_preset(preset.preset);
            rebuild();
            item.rebuildMs = +(performance.now() - switchAt).toFixed(2);
            item.viewState = JSON.parse(M.ccall(
              'web_drive_camera_state', 'string', [], []));
            item.visibleCar = liveScene.draws.filter(
              (draw) => draw.kind === 'car' && !draw.hidden).length;
            item.visibleInterior = liveScene.draws.filter(
              (draw) => draw.kind === 'interior' && !draw.hidden).length;
            item.visibilityMatch = item.viewState.view === preset.view &&
              (preset.view === 0
                ? item.visibleCar === 0 && item.visibleInterior > 0
                : item.visibleCar > 0 && item.visibleInterior === 0);
            item.frameMs = [];
            for (let i = 0; i < cycleFrames; i++) {
              const frameAt = performance.now();
              M._web_drive_step();
              M._web_drive_render();
              const refreshed = refreshDraws(M, liveScene);
              if (refreshed !== liveScene.count)
                throw new Error(`drawlist changed ${liveScene.count}->${refreshed}`);
              const liveOverlay = readOverlay(M);
              if (!liveOverlay) throw new Error('overlay unavailable');
              gs.setHud(liveOverlay.indices, readPalette(M), liveOverlay.mask);
              const liveCam = readCamera(M);
              gs.setCameraBasis(liveCam);
              const pixels = await gs.frame(
                Float32Array.from(cameraViewProj(liveCam, rw, rh)),
                liveScene, true);
              await device.queue.onSubmittedWorkDone();
              item.hashes.push(fnv1a(pixels));
              item.cameras.push({ eye: liveCam.eye, forward: liveCam.forward });
              item.frameMs.push(+(performance.now() - frameAt).toFixed(2));
            }
            item.changing = new Set(item.hashes).size > 1;
            item.changedFromPrevious = previousHash === null ||
              item.hashes.some((hash) => hash !== previousHash);
            const sortedFrameMs = [...item.frameMs].sort((a, b) => a - b);
            item.steadyMedianMs = sortedFrameMs[Math.floor(sortedFrameMs.length / 2)];
            item.visualGapMs = +(item.rebuildMs + item.frameMs[0]).toFixed(2);
            item.stallBudgetMs = +Math.max(100, item.steadyMedianMs * 4).toFixed(2);
            item.stalled = item.visualGapMs > item.stallBudgetMs;
            item.match = item.changing && item.changedFromPrevious &&
              item.visibilityMatch && !item.stalled;
            previousHash = item.hashes[item.hashes.length - 1];
          } catch (e) {
            item.error = String((e && e.stack) || e);
            item.match = false;
          }
          results.push(item);
          if (item.error) break;
        }
      } finally {
        M._web_key_event(PK_UP, 0);
      }
      patterns.push({
        name: 'view-cycle-live', framesPerPreset: cycleFrames,
        presetsExpected: presets.length, presetsChecked: results.length,
        results, gpuErrors,
        match: results.length === presets.length &&
          results.every((item) => item.match) && gpuErrors.length === 0,
      });
    }

    const gated = patterns.filter((p) => !p.skipped);
    const match = gated.length > 0 && gated.every((p) => p.match);
    finish({ webgpu: true, adapter: info, match,
      frame: { mission, tick: fixedTick, requestedView: viewWant,
        requestedPaper: paperWant, requestedDamage: damageWant,
        paperActive, titleActive,
        gpuResolution: gpuRes,
        renderTarget: { ...target, w: renderW, h: renderH }, perf,
        hashes: { softwareRgba: fnv1aHex(swRgba),
          webgpuRgba: fnv1aHex(nativeRgba), comparedRgba: fnv1aHex(rgba) },
        camera: { eye: cam.eye, right: cam.right, up: cam.up,
          forward: cam.forward } },
      patterns }, match ? 'PASS' : 'FAIL');
  } catch (e) {
    patterns.push({ name: 'exception', match: false,
      error: String((e && e.stack) || e) });
    finish({ webgpu: true, match: false, patterns }, 'FAIL');
  }
}
