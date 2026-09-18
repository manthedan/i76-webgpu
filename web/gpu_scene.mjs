/*
 * gpu_scene.mjs — M6 Tier-2: WebGPU hardware scene path (the Glide look).
 *
 * Spec: docs/specs/m6/webgpu-plan.md ("Tier 2 — hardware scene path").
 * Renders an experimental C export of terrain, placed scene meshes, and
 * vehicle state through WebGPU using the live exported camera. Software
 * remains authoritative and builds its own draw queue; this bridge is not a
 * unified renderer-neutral stream and can still diverge from it.
 *
 * Geometry/texture sources (webmain.c M6 Tier-2 exports):
 *   web_gpu_terrain_data    heightfield grid over the used extent
 *   web_gpu_terrain_tex     mission WRLD +108 .map tile -> level-pal RGBA
 *   web_gpu_scene_*         placed-object parts: GeoMesh ptr + world xform
 *   web_gpu_car_xform/mesh  player car (software draws part 0 at car_basis)
 *   web_gpu_interior_*      first-person interior parts (VGEO set 16),
 *                             drawn only while the cockpit view owns the frame
 *   web_gpu_drawlist_*      required GPU scene-membership bridge; independent
 *                           of the software renderer's draw queue
 *                           (collectDraws fails closed without it)
 *   web_gpu_m16_*           .m16 / *6.pak tile decode (m16.c)
 *   web_gpu_project         software-camera projection of a world point
 *   web_fb_pre_hud          pre-HUD software frame (geometry diagnostic)
 *   web_gpu_overlay_*       complete software-owned post-world 2-D layer
 *
 * DECISION (mesh shading): OEG face RGB + optional .m16 per texture-name
 * bucket (readMesh material groups). The fragment stage samples a bound
 * tile when u_model.color.a > 0.5; otherwise it uses the face's authored
 * RGB. Fidelity mode retains its established bilinear+mip sampling byte-for-
 * byte; native presentation point-samples the same indexed-source tiles.
 * The browser's M6 promotion rung now compares the complete shaded colour
 * frame against the software golden with a documented bounded tolerance;
 * projection/coverage remain its geometric diagnostics.
 *
 * DECISION (terrain texture): the whole terrain is tiled with the ONE
 * WRLD surface texture (docs/specs/m2/terrain.md §4 — CONFIRMED), uv
 * period = texels/10 meters (terrain.md §4 — INFERRED Open76 mapping).
 * Fidelity mode retains the promotion gate's established bilinear+mip path;
 * native mode uses level-0 nearest sampling so higher target resolution never
 * smooths palette art. When the
 * mission's tile is unresolvable, a generated 2-tone checker stands in
 * (geometry/parity unaffected).
 *
 * Browser entry: index.html drives GpuScene directly for drive mode.
 * Renderer and read-only extraction helpers are retained here; this runtime
 * module does not fetch game files. Portable browser-test orchestration lives
 * separately in web/tests/gpu_scene_runner.mjs and requires an explicit asset
 * route. A CPU adapter result is still a real WebGPU result, but it is not a
 * consumer hardware/browser support matrix.
 */

export const FB_W = 640;
export const FB_H = 480;
/* Native presentation follows the canvas's physical display size, bounded at
 * 2x fidelity resolution so high-DPR/zoomed displays cannot silently turn the
 * 1997 draw list into an unbounded fill-rate workload. At the product page's
 * 640x480 CSS size this is exact through DPR 2 (1280x960). */
export const NATIVE_MAX_SCALE = 2;

export function renderTargetSize(mode, cssW = FB_W, cssH = FB_H,
                                 dpr = 1) {
  if (mode === 'fidelity')
    return { mode, w: FB_W, h: FB_H, scale: 1, capped: false };
  if (mode !== 'native') throw new Error(`unknown GPU resolution mode: ${mode}`);
  const displayW = Math.max(1, Number(cssW) * Math.max(1, Number(dpr) || 1));
  const displayH = Math.max(1, Number(cssH) * Math.max(1, Number(dpr) || 1));
  const displayScale = Math.min(displayW / FB_W, displayH / FB_H);
  /* Keep an exact integer 4:3 target. Independent width/height rounding can
   * create a one-pixel aspect skew at responsive widths; one 4x3 pixel block
   * of unused display is preferable to changing projection geometry. */
  const block = Math.max(1, Math.floor(Math.min(
    displayW / 4, displayH / 3, (FB_W * NATIVE_MAX_SCALE) / 4)));
  const w = block * 4, h = block * 3;
  const scale = w / FB_W;
  return { mode, w, h, scale, capped: displayScale > NATIVE_MAX_SCALE,
    displayW: Math.round(displayW), displayH: Math.round(displayH) };
}

/* WebGPU usage enums — always resolve from globalThis so module scope is safe
 * under every Chrome secure-context path (mac-mini headed, etc.). */
const _g = globalThis;
const GPUTextureUsage = _g.GPUTextureUsage || {
  COPY_SRC: 0x01, COPY_DST: 0x02, TEXTURE_BINDING: 0x04,
  STORAGE_BINDING: 0x08, RENDER_ATTACHMENT: 0x10,
};
const GPUBufferUsage = _g.GPUBufferUsage || {
  MAP_READ: 0x0001, MAP_WRITE: 0x0002, COPY_SRC: 0x0004, COPY_DST: 0x0008,
  INDEX: 0x0010, VERTEX: 0x0020, UNIFORM: 0x0040, STORAGE: 0x0080,
  INDIRECT: 0x0100, QUERY_RESOLVE: 0x0200,
};
const GPUShaderStage = _g.GPUShaderStage || { VERTEX: 0x1, FRAGMENT: 0x2, COMPUTE: 0x4 };
const GPUMapMode = _g.GPUMapMode || { READ: 0x0001, WRITE: 0x0002 };

/* Exported so gpu_road_policy_probe.mjs pins the actual WebGPU mirror state,
 * not a duplicated expectation. Native SW paints roads after terrain without
 * a z-buffer; this is the hardware equivalent that preserves hill occlusion. */
export const ROAD_TERRAIN_TOLERANCE_M = 2.25;
export const ROAD_DEPTH_PULL_M = 0.0;
export const ROAD_DEPTH_POLICY = Object.freeze({
  format: 'depth24plus', depthWriteEnabled: true, depthCompare: 'less',
  /* PORT DECISION: fs_road mirrors the software bridge's continuous bounded
   * view-Z comparison. A road may sit at most ROAD_TERRAIN_TOLERANCE_M behind
   * visible terrain, absorbing the port's camera-centred LOD drift without an
   * absolute integer-metre threshold. Terrain acceptance is shader-only;
   * unbiased strict depth in the dedicated road attachment resolves road
   * intersections without allowing a later quantized tie to overwrite the
   * first owner. A colour-masked,
   * unbiased owner pass on the main attachment means visible roads retain real
   * depth for later meshes, while road-only depth cannot become a false world
   * occluder. */
  depthBias: 0, depthBiasSlopeScale: 0, depthBiasClamp: 0,
});
export const ROAD_OWNER_DEPTH_POLICY = Object.freeze({
  format: 'depth24plus', depthWriteEnabled: true, depthCompare: 'always',
  depthBias: 0, depthBiasSlopeScale: 0, depthBiasClamp: 0,
});

export function roadDepthForViewZ(zs) {
  const near = 0.25, far = 3000.0;
  const alpha = far / (far - near), beta = -far * near / (far - near);
  /* Terrain acceptance is the separate linear view-Z comparison. This is
   * unbiased depth for nearest-road ordering in the isolated attachment. */
  return Math.max(0, Math.min(1,
    alpha + beta / Math.max(zs, near)));
}

/* ------------------------------------------------------------------ */
/* Camera — exact JS replica of terrain.c terrain_render_cam/camspace */
/* (FACT: src/engine/terrain.c + raster.h): right = norm(fz,0,-fx), up =  */
/* fwd x right, focal = (w*0.5)/tan(45 deg) = w*0.5 [90 deg HORIZONTAL,   */
/* binary-derived from nitro.exe FUN_00469dd0/FUN_00469fb0], near 5,      */
/* far 3000; screen x = w/2 + f*xs/zs, y = h/2 - f*ys/zs.             */
/* ------------------------------------------------------------------ */

/* Column-major 4x4 multiply: out = a * b. */
export function mat4mul(a, b) {
  const o = new Float64Array(16);
  for (let c = 0; c < 4; c++)
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      o[c * 4 + r] = s;
    }
  return o;
}

/*
 * View-projection matrix from the canonical camera basis exported by wasm.
 * Returns Float64Array(16), column-major, with WebGPU clip conventions
 * (z in [0,1], w = view zs). No world-up reconstruction: scripted roll is
 * part of the view.
 */
export function cameraViewProj(camera, w = FB_W, h = FB_H,
                               worldOrigin = [0, 0, 0]) {
  const { right: r, up: u, forward: fwd } = camera;
  const eye = camera.eye.map((v, i) => v - worldOrigin[i]);
  const tanHalf = Number.isFinite(camera.fovTanHalf) && camera.fovTanHalf > 0
    ? camera.fovTanHalf : 1;
  // Horizontal half-FOV tangent comes from CameraView. Normal views use 1
  // (90 degrees); PRESET_VIEW_4 uses tan(60 degrees) for its verified 120.
  const focal = (w * 0.5) / tanHalf;
  const A = focal / (w * 0.5), B = focal / (h * 0.5);
  // DEVIATION (near plane): the software wireframe default is 5 m, which
  // clips the cockpit foreground. The hardware path has a real depth buffer
  // and does not need that software rasteriser margin.
  const near = 0.25, far = 3000.0;
  const alpha = far / (far - near), beta = -far * near / (far - near);
  const tx = -(r[0] * eye[0] + r[1] * eye[1] + r[2] * eye[2]);
  const ty = -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]);
  const tz = -(fwd[0] * eye[0] + fwd[1] * eye[1] + fwd[2] * eye[2]);
  const V = [r[0], u[0], fwd[0], 0,
             r[1], u[1], fwd[1], 0,
             r[2], u[2], fwd[2], 0,
             tx, ty, tz, 1];
  const P = [A, 0, 0, 0,
             0, B, 0, 0,
             0, 0, alpha, 1,
             0, 0, beta, 0];
  return mat4mul(P, V);
}

/*
 * Project a world point to framebuffer pixels through a matrix from
 * cameraViewProj. Returns [sx, sy, zs]; zs < 5 is inside the software
 * wireframe near plane.
 */
export function projectPoint(vp, p, w = FB_W, h = FB_H) {
  const cx = vp[0] * p[0] + vp[4] * p[1] + vp[8] * p[2] + vp[12];
  const cy = vp[1] * p[0] + vp[5] * p[1] + vp[9] * p[2] + vp[13];
  const cw = vp[3] * p[0] + vp[7] * p[1] + vp[11] * p[2] + vp[15];
  return [(cx / cw + 1) * w * 0.5, (1 - cy / cw) * h * 0.5, cw];
}

/* xform12 (r/u/f/pos basis columns — scene.c D4) -> column-major mat4. */
export function xformToMat4(t) {
  return [t[0], t[1], t[2], 0,
          t[3], t[4], t[5], 0,
          t[6], t[7], t[8], 0,
          t[9], t[10], t[11], 1];
}

/*
 * Horizontal clip-space reflection for the rearview mirror pass: negate the
 * clip-x row of a column-major VP (indices 0/4/8/12) so NDC x -> -x. That
 * mirrors the image left-right — the GPU twin of the software path's
 * dst[x] = mirror_fb[mw-1-x] (webmain.c rearview composite). It also
 * inverts framebuffer-space triangle winding, so the mirror pipelines use
 * the opposite frontFace (see _pipesFor).
 */
export function flipVpX(vp) {
  const o = Float32Array.from(vp);
  o[0] = -o[0]; o[4] = -o[4]; o[8] = -o[8]; o[12] = -o[12];
  return o;
}

/* flipVpX inverts winding; mirror pipelines flip frontFace to compensate. */
export const flipFront = (f) => (f === 'ccw' ? 'cw' : 'ccw');

/*
 * Sky-dome basis for the rearview mirror pass, from the derived rear
 * CameraView (readRearview). Horizontally REFLECTED to stay consistent with
 * the flipVpX'd VP written alongside it: with clip x negated, a fragment px
 * pixels right of the rect centre corresponds to view -px, so the dome ray
 * uses the negated rear right vector. rect = { x, y, w, h } in framebuffer
 * pixels; cx/cy are the rect centre because the mirror pass's viewport
 * offset shifts fragment positions into the rect.
 */
export function mirrorBasis(cam, rect) {
  const fth = (cam.fovTanHalf > 0) ? cam.fovTanHalf : 1;
  return {
    right: [-cam.right[0], -cam.right[1], -cam.right[2]],
    up: cam.up, forward: cam.forward,
    focal: (rect.w * 0.5) / fth,
    cx: rect.x + rect.w * 0.5,
    cy: rect.y + rect.h * 0.5,
  };
}

/* ------------------------------------------------------------------ */
/* Wasm export readers (fresh heap views every call — MEMFS writes can */
/* grow wasm memory and detach earlier views; index.html convention).  */
/* ------------------------------------------------------------------ */

export const heap8 = (M) => new Uint8Array(M.wasmMemory.buffer);
export const heapF32 = (M) => new Float32Array(M.wasmMemory.buffer);
export const heapI32 = (M) => new Int32Array(M.wasmMemory.buffer);
export const heapF64 = (M) => new Float64Array(M.wasmMemory.buffer);

const readF64 = (M, ptr, n) =>
  Array.from(heapF64(M).subarray(ptr >> 3, (ptr >> 3) + n));

/* web_gpu_camera -> canonical basis plus horizontal half-FOV tangent. */
export function readCamera(M) {
  const p = M._web_gpu_camera();
  const v = readF64(M, p, 13);
  return {
    eye: v.slice(0, 3), right: v.slice(3, 6),
    up: v.slice(6, 9), forward: v.slice(9, 12), fovTanHalf: v[12],
  };
}

/*
 * Rearview mirror seam: the C side owns the derived second CameraView
 * (web_gpu_rearview_camera) and the destination rect (web_rearview_rect),
 * exactly like the primary camera — the GPU path never reconstructs its
 * own. Null while the mirror is not presenting (toggle off, chase view,
 * or a live mission camera).
 */
export function readRearview(M) {
  if (!M._web_rearview_active || M._web_rearview_active() !== 1) return null;
  const p = M._web_gpu_rearview_camera();
  if (!p) return null;
  const v = readF64(M, p, 13);
  const r = heapI32(M).subarray(M._web_rearview_rect() >> 2,
                                (M._web_rearview_rect() >> 2) + 4);
  return {
    cam: { eye: v.slice(0, 3), right: v.slice(3, 6),
           up: v.slice(6, 9), forward: v.slice(9, 12),
           fovTanHalf: v[12] },
    rect: [r[0], r[1], r[2], r[3]],
  };
}

/* web_gpu_terrain_data -> { cols, rows, wx0, wz0, step, heights } | null */
export function readTerrainGrid(M) {
  const p = M._web_gpu_terrain_data();
  if (!p) return null;
  const f = heapF32(M);
  const b = p >> 2;
  const cols = f[b], rows = f[b + 1];
  const wx0 = f[b + 2], wz0 = f[b + 3], step = f[b + 4];
  const heights = f.slice(b + 5, b + 5 + cols * rows);
  return { cols, rows, wx0, wz0, step, heights };
}

/*
 * Software-derived LOD ring mesh export. Rebuilds from the current drive
 * camera. Returns { verts: Float32Array, nverts, bands:[n,m,f] } or null.
 */
export function readTerrainLod(M) {
  if (!M._web_gpu_terrain_lod_build || M._web_gpu_terrain_lod_build() !== 0)
    return null;
  const n = M._web_gpu_terrain_lod_n();
  const p = M._web_gpu_terrain_lod_ptr();
  if (!n || !p) return { verts: new Float32Array(0), nverts: 0, bands: [0, 0, 0] };
  const f = heapF32(M);
  const verts = f.slice(p >> 2, (p >> 2) + n * LOD_VERT_FLOATS);
  let bands = [0, 0, 0];
  if (M._web_gpu_terrain_lod_bands) {
    const bp = M._web_gpu_terrain_lod_bands();
    const i32 = heapI32(M);
    const o = bp >> 2;
    bands = [i32[o], i32[o + 1], i32[o + 2]];
  }
  return { verts, nverts: n, bands };
}

/* Road ribbon mesh + per-type .map textures (palette-expanded RGBA). */
export function readRoads(M) {
  if (!M._web_gpu_roads_build || M._web_gpu_roads_build() !== 0) return null;
  const n = M._web_gpu_roads_n();
  const p = M._web_gpu_roads_ptr();
  if (!n || !p) return { verts: new Float32Array(0), nverts: 0, byType: [0, 0, 0], tex: [] };
  const f = heapF32(M);
  const verts = f.slice(p >> 2, (p >> 2) + n * LOD_VERT_FLOATS);
  const bp = M._web_gpu_roads_by_type();
  const i32 = heapI32(M);
  const o = bp >> 2;
  const byType = [i32[o], i32[o + 1], i32[o + 2]];
  const tex = [];
  for (let t = 0; t < 3; t++) {
    const w = M._web_gpu_road_tex_w(t), h = M._web_gpu_road_tex_h(t);
    const rp = M._web_gpu_road_tex_rgba(t);
    if (w > 0 && h > 0 && rp)
      tex.push({ w, h, rgba: heap8(M).slice(rp, rp + w * h * 4) });
    else tex.push(null);
  }
  return { verts, nverts: n, byType, tex };
}

/* Apply software sun/ambient into a GpuScene (if export present). */
export function applySoftwareLight(M, gs) {
  if (!M._web_gpu_light_params) return;
  const p = M._web_gpu_light_params();
  if (!p) return;
  const f = heapF32(M);
  const o = p >> 2;
  gs.lightDir = [f[o], f[o + 1], f[o + 2]];
  gs.lightAmb = f[o + 3];
}

export function readVisibilityCounts(M) {
  if (!M._web_gpu_visibility_counts) return null;
  const p = M._web_gpu_visibility_counts();
  if (!p) return null;
  const i32 = heapI32(M);
  const o = p >> 2;
  return {
    sceneVisible: i32[o], sceneHidden: i32[o + 1],
    carParts: i32[o + 2], interiorParts: i32[o + 3],
    interiorHidden: i32[o + 4], lodQuads: i32[o + 5],
  };
}

/* Terrain texture via web_gpu_terrain_tex; generated checker fallback. */
export function readTerrainTexture(M) {
  if (M._web_gpu_terrain_tex() === 0) {
    const w = M._web_gpu_terrain_tex_w(), h = M._web_gpu_terrain_tex_h();
    const p = M._web_gpu_terrain_tex_rgba();
    return { w, h, rgba: heap8(M).slice(p, p + w * h * 4), fallback: false };
  }
  // DECISION (fallback): 64x64 two-tone earth checker, 8px cells —
  // deterministic, only used when the mission names no resolvable tile.
  const w = 64, h = 64;
  const rgba = new Uint8Array(w * h * 4);
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      const on = ((x >> 3) ^ (y >> 3)) & 1;
      const i = (y * w + x) * 4;
      rgba[i] = on ? 0x6a : 0x4a;
      rgba[i + 1] = on ? 0x54 : 0x38;
      rgba[i + 2] = on ? 0x38 : 0x24;
      rgba[i + 3] = 255;
    }
  return { w, h, rgba, fallback: true };
}

/*
 * Unique-mesh cache keyed by the wasm GeoMesh pointer (scene.c owns the
 * meshes; repeated parts share one decode). Fan-triangulates the OEG
 * polygon faces (geomesh.h: face_first/face_count into indices[]).
 */
export function readMesh(M, cache, ptr) {
  if (cache.has(ptr)) return cache.get(ptr);
  const nv = M._web_gpu_mesh_num_verts(ptr);
  const ni = M._web_gpu_mesh_num_indices(ptr);
  const nf = M._web_gpu_mesh_num_faces(ptr);
  if (!nv || !nf || !ni) { cache.set(ptr, null); return null; }
  const vp = M._web_gpu_mesh_verts(ptr);
  const verts = heapF32(M).slice(vp >> 2, (vp >> 2) + nv * 3);
  const ip = M._web_gpu_mesh_indices(ptr);
  const fp = M._web_gpu_mesh_face_first(ptr);
  const cp = M._web_gpu_mesh_face_count(ptr);
  const i32 = heapI32(M);
  const indices = i32.subarray(ip >> 2, (ip >> 2) + ni);
  const first = i32.subarray(fp >> 2, (fp >> 2) + nf);
  const count = i32.subarray(cp >> 2, (cp >> 2) + nf);
  const tris = [];
  for (let f = 0; f < nf; f++) {
    const v0 = indices[first[f]];
    for (let k = 1; k + 1 < count[f]; k++) {
      const a = indices[first[f] + k], b = indices[first[f] + k + 1];
      if (v0 >= 0 && a >= 0 && b >= 0 && v0 < nv && a < nv && b < nv)
        tris.push(v0, a, b);
    }
  }

  /*
   * Material geometry (docs/specs/m6/oeg-face-format.md). The arrays above
   * stay exactly as they were — the node rasterizer and the parity gate read
   * them — and the GPU gets a second, non-indexed, material-grouped build:
   * every triangle carries its face's authored RGB and its own UV, so faces
   * sharing a position vertex but differing in colour/UV stay distinct.
   * Layout per vertex: pos3, nrm3, uv2, rgb3 = 11 floats.
   */
  const nrmP = M._web_gpu_mesh_normals(ptr);
  const rgbP = M._web_gpu_mesh_face_rgb(ptr);
  const flgP = M._web_gpu_mesh_face_flags(ptr);
  const texP = M._web_gpu_mesh_face_tex(ptr);
  const uvP = M._web_gpu_mesh_uvs(ptr);
  const nixP = M._web_gpu_mesh_normal_indices(ptr);
  let vbo = null, groups = [];
  if (nrmP && rgbP && flgP && texP && uvP && nixP) {
    const stride = M._web_gpu_mesh_tex_stride();
    const f32 = heapF32(M), u8 = heap8(M);
    const nrm = f32.subarray(nrmP >> 2, (nrmP >> 2) + nv * 3);
    const uvs = f32.subarray(uvP >> 2, (uvP >> 2) + ni * 2);
    const nix = i32.subarray(nixP >> 2, (nixP >> 2) + ni);
    const rgb = u8.subarray(rgbP, rgbP + nf * 3);
    const flg = u8.subarray(flgP, flgP + nf * 3);
    const texName = (f) => {
      let s = '';
      for (let k = 0; k < stride; k++) {
        const c = u8[texP + f * stride + k];
        if (!c) break;
        s += String.fromCharCode(c);
      }
      return s;
    };
    // Bucket faces by texture name so each bucket becomes one draw.
    const buckets = new Map();
    for (let f = 0; f < nf; f++) {
      const key = texName(f);
      let g = buckets.get(key);
      if (!g) { buckets.set(key, g = { tex: key, cutout: false, faces: [] }); }
      if (GEO_FLAG2_CUTOUT(flg[f * 3 + 1])) g.cutout = true;
      g.faces.push(f);
    }
    const out = [];
    for (const g of buckets.values()) {
      const startVert = out.length / 11;
      for (const f of g.faces) {
        const base = first[f], n = count[f];
        const r = rgb[f * 3] / 255, gg = rgb[f * 3 + 1] / 255,
              bb = rgb[f * 3 + 2] / 255;
        const push = (slot) => {
          const vi = indices[slot], nI = nix[slot];
          out.push(verts[vi * 3], verts[vi * 3 + 1], verts[vi * 3 + 2],
                   nrm[nI * 3], nrm[nI * 3 + 1], nrm[nI * 3 + 2],
                   uvs[slot * 2], uvs[slot * 2 + 1], r, gg, bb);
        };
        // Fan from corner 0; winding is CCW from the +normal side (CONFIRMED
        // corpus-wide), which is WebGPU's default frontFace, so emit as-is.
        for (let k = 1; k + 1 < n; k++) {
          const a = indices[base], b = indices[base + k], c = indices[base + k + 1];
          if (a < 0 || b < 0 || c < 0 || a >= nv || b >= nv || c >= nv) continue;
          push(base); push(base + k); push(base + k + 1);
        }
      }
      const nVert = out.length / 11 - startVert;
      if (nVert > 0)
        groups.push({ tex: g.tex, cutout: g.cutout, first: startVert, count: nVert });
    }
    vbo = Float32Array.from(out);
  }

  const mesh = { verts, tris: Uint32Array.from(tris), vbo, groups,
                 name: M._web_gpu_mesh_name ? readCStr(M, M._web_gpu_mesh_name(ptr)) : '' };
  cache.set(ptr, mesh);
  return mesh;
}

/* flags2 5/7 mark cut-out surfaces (headlights, fences, rails) — spec §flags. */
const GEO_FLAG2_CUTOUT = (f2) => f2 === 5 || f2 === 7;

export function readCStr(M, ptr, max = 64) {
  if (!ptr) return '';
  const u8 = heap8(M);
  let s = '';
  for (let i = 0; i < max; i++) {
    const c = u8[ptr + i];
    if (!c) break;
    s += String.fromCharCode(c);
  }
  return s;
}

/*
 * Collect draws from the C GPU drawlist bridge (web_gpu_drawlist_*). This is
 * the only scene-membership input accepted by this JS backend, but it is
 * independent of the software renderer's draw queue and therefore does not
 * make drift impossible. There is no JS-side scene/car/interior traversal
 * fallback: when the export is missing or the rebuild reports failure
 * (<= 0 draws), collectDraws returns null and the caller MUST fall back to
 * the software renderer.
 * Returns { meshes: Map, draws, stream: 'drawlist', count } | null.
 */
function readPaintVtf(M, kind, objIdx) {
  if (kind === 'scene') {
    if (M._web_gpu_scene_obj_vtf)
      return readCStr(M, M._web_gpu_scene_obj_vtf(objIdx), 16);
    return '';
  }
  /* Player car body + interior panels share the driven paint scheme. */
  if (M._web_gpu_car_vtf) return readCStr(M, M._web_gpu_car_vtf(), 16);
  return '';
}

export function collectDraws(M) {
  if (!M._web_gpu_drawlist_build) return null;
  const n = M._web_gpu_drawlist_build();
  if (!(n > 0)) return null;
  const cache = new Map();
  const draws = [];
  const kinds = ['scene', 'car', 'interior'];
  for (let i = 0; i < n; i++) {
    const kind = kinds[M._web_gpu_drawlist_kind(i)] || 'scene';
    const mp = M._web_gpu_drawlist_mesh(i);
    const xp = M._web_gpu_drawlist_model(i);
    if (!mp || !xp) return null;
    const mesh = readMesh(M, cache, mp);
    if (!mesh) return null;
    const d = {
      ptr: mp, mesh, model: xformToMat4(readF64(M, xp, 12)),
      kind, hidden: !!M._web_gpu_drawlist_hidden(i),
      drawIndex: i, vtf: '',
    };
    if (kind === 'scene') {
      d.obj = M._web_gpu_drawlist_a(i); d.part = M._web_gpu_drawlist_b(i);
      d.vtf = readPaintVtf(M, kind, d.obj);
    } else if (kind === 'car') {
      d.carPart = M._web_gpu_drawlist_a(i);
      d.vtf = readPaintVtf(M, kind, -1);
    } else {
      d.intPart = M._web_gpu_drawlist_a(i);
      d.vtf = readPaintVtf(M, kind, -1);
    }
    draws.push(d);
  }
  return { meshes: cache, draws, stream: 'drawlist', count: n };
}

/*
 * Re-read every draw's live transform and visibility from wasm. Without this
 * the GPU view freezes all mission objects at spawn (convoys, enemies) and
 * keeps drawing objects the sim has hidden. Drawlist-only: returns the
 * rebuilt draw count, or -1 when the stream is unavailable or any record's
 * identity changed (including a same-count membership replacement).
 */
export function refreshDraws(M, scene) {
  if (!scene || scene.stream !== 'drawlist' || !M._web_gpu_drawlist_build)
    return -1;
  const n = M._web_gpu_drawlist_build();
  if (n < 0) return -1;
  for (const d of scene.draws) {
    const i = d.drawIndex;
    if (i === undefined || i >= n) return -1;
    const kind = ['scene', 'car', 'interior'][M._web_gpu_drawlist_kind(i)];
    if (kind !== d.kind || M._web_gpu_drawlist_mesh(i) !== d.ptr) return -1;
    if (kind === 'scene' &&
        (M._web_gpu_drawlist_a(i) !== d.obj ||
         M._web_gpu_drawlist_b(i) !== d.part)) return -1;
    if (kind === 'car' && M._web_gpu_drawlist_a(i) !== d.carPart) return -1;
    if (kind === 'interior' && M._web_gpu_drawlist_a(i) !== d.intPart) return -1;
    const xp = M._web_gpu_drawlist_model(i);
    if (!xp) return -1;
    d.hidden = !!M._web_gpu_drawlist_hidden(i);
    d.model = xformToMat4(readF64(M, xp, 12));
  }
  return n;
}

/* Per-mesh flat color, hashed from the GeoMesh pointer (DECISION —
 * deterministic, no UVs available; see file header). Muted palette. */
function meshColor(ptr) {
  let h = (ptr * 2654435761) >>> 0;
  h ^= h >>> 13; h = (h * 1274126177) >>> 0;
  const r = 0.35 + 0.35 * ((h & 255) / 255);
  const g = 0.35 + 0.35 * (((h >>> 8) & 255) / 255);
  const b = 0.35 + 0.35 * (((h >>> 16) & 255) / 255);
  return [r, g, b, 1];
}

/* ------------------------------------------------------------------ */
/* WGSL                                                                */
/* ------------------------------------------------------------------ */

/*
 * Shading model (docs/specs/m6/oeg-face-format.md + the Glide target):
 *  - meshes carry the authored per-face RGB and per-vertex normals from the
 *    GEO; a single directional light plus a generous ambient reproduces the
 *    era's Gouraud look without inventing a light rig;
 *  - distance fog toward the horizon colour hides the far clip, exactly what
 *    grFogMode/grFogColorValue did on the Voodoo;
 *  - fidelity keeps the gated bilinear+mip texture path; native presentation
 *    point-samples level 0 for authentic indexed-source art;
 *  - the software-owned 2-D composite is drawn last through the level palette
 *    so HUD/cockpit, combat feedback and paper states share one authority.
 * u_frame.fog = (start, end, unused, unused); u_frame.sky = horizon colour.
 */
export const SCENE_WGSL = `
struct UFrame {
  vp: mat4x4<f32>,
  tile_m: f32,
  fog_start: f32,
  fog_end: f32,
  light_amb: f32,
  sky: vec4<f32>,          // horizon / fog convergence colour
  light_dir: vec4<f32>,
  zenith: vec4<f32>,       // top of the level's own sky ramp
  cam_right: vec4<f32>,
  cam_up: vec4<f32>,
  cam_forward: vec4<f32>,
  // x=focal_px, y=cx, z=cy, w=1 when authored sky texture is bound
  cam_proj: vec4<f32>,
  // xy = render-target pixels, z = point-sample indexed world textures.
  render_target: vec4<f32>,
};
struct UModel {
  model: mat4x4<f32>,
  // Primary-view projection is composed in JS double precision before the
  // final f32 upload. This avoids cancelling separate ~48 km VP/model terms
  // in the cockpit vertex shader; mirror passes retain frame.vp * model.
  mvp: mat4x4<f32>,
  color: vec4<f32>,
};
// group 0: per-frame + final software 2-D overlay + authored sky dome
// group 1: per-draw model uniform
// group 2: per-material texture (terrain surface, or a face's .m16 tile)
@group(0) @binding(0) var<uniform> u_frame: UFrame;
@group(0) @binding(1) var u_hud: texture_2d<f32>;      // r8unorm index
@group(0) @binding(2) var u_pal: texture_2d<f32>;      // 256x1 rgba palette
@group(0) @binding(3) var u_hud_samp: sampler;         // nearest
@group(0) @binding(4) var u_sky_tex: texture_2d<f32>;
@group(0) @binding(5) var u_sky_samp: sampler;
@group(0) @binding(6) var u_hud_mask: texture_2d<f32>; // explicit overlay coverage
@group(0) @binding(7) var u_road_mask: texture_2d<u32>;
@group(0) @binding(8) var u_road_terrain_z: texture_2d<f32>;
@group(1) @binding(0) var<uniform> u_model: UModel;
@group(2) @binding(0) var u_tex: texture_2d<f32>;
@group(2) @binding(1) var u_samp: sampler;

fn apply_fog(c: vec3<f32>, zs: f32) -> vec3<f32> {
  let t = clamp((zs - u_frame.fog_start) /
                max(u_frame.fog_end - u_frame.fog_start, 1.0), 0.0, 1.0);
  return mix(c, u_frame.sky.rgb, t);
}

/* ---- terrain: tiled surface texture, lit by slope, fogged -------------
 * Vertex: pos3 + nrm3 + uv2. UV is software-baked for LOD rings
 * (wx*10/w_tex); legacy grid path writes p.xz / tile_m into the same slots.
 */
struct TerrOut {
  @builtin(position) pos: vec4<f32>,
  @location(0) uv: vec2<f32>,
  @location(1) zs: f32,
  @location(2) nrm: vec3<f32>,
};
@vertex
fn vs_terrain(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>,
              @location(2) uv: vec2<f32>) -> TerrOut {
  var o: TerrOut;
  let clip = u_frame.vp * vec4<f32>(p, 1.0);
  o.pos = clip;
  o.uv = uv;
  o.zs = clip.w;                       // view-space depth (w = zs)
  o.nrm = n;
  return o;
}
fn indexed_world_sample(uv: vec2<f32>) -> vec4<f32> {
  if (u_frame.render_target.z > 0.5) {
    let dim = vec2<i32>(textureDimensions(u_tex));
    let wrapped = fract(uv);
    let texel = min(vec2<i32>(floor(wrapped * vec2<f32>(dim))), dim - vec2i(1));
    return textureLoad(u_tex, texel, 0);
  }
  return textureSample(u_tex, u_samp, uv);
}
fn terrain_color(uv: vec2<f32>, zs: f32, nrm: vec3<f32>) -> vec4<f32> {
  let c = indexed_world_sample(uv);
  let n = normalize(nrm);
  let lam = max(dot(n, normalize(u_frame.light_dir.xyz)), 0.0);
  let lit = c.rgb * (u_frame.light_amb + (1.0 - u_frame.light_amb) * lam);
  return vec4<f32>(apply_fog(lit, zs), 1.0);
}
@fragment
fn fs_terrain(i: TerrOut) -> @location(0) vec4<f32> {
  return terrain_color(i.uv, i.zs, i.nrm);
}

struct RoadFrag {
  @location(0) color: vec4<f32>,
  @location(1) winner: u32,
  @builtin(frag_depth) depth: f32,
};
fn road_frag_depth(zs: f32) -> f32 {
  let near = 0.25;
  let far = 3000.0;
  let alpha = far / (far - near);
  let beta = -far * near / (far - near);
  return clamp(alpha + beta / max(zs, near), 0.0, 1.0);
}
fn road_painter_accept(road_z: f32, terrain_z: f32) -> bool {
  if (terrain_z <= 0.0) { return true; }
  return road_z <= terrain_z + ${ROAD_TERRAIN_TOLERANCE_M.toFixed(2)};
}
@fragment
fn fs_road_seed(i: TerrOut) -> @location(0) f32 { return i.zs; }
/* Gate-only projected-road footprint, before the ownership predicate. */
@fragment
fn fs_road_footprint(i: TerrOut) -> @location(0) u32 { return bitcast<u32>(i.zs); }
@fragment
fn fs_road(i: TerrOut) -> RoadFrag {
  let terrain_z = textureLoad(u_road_terrain_z, vec2<i32>(i.pos.xy), 0).x;
  if (!road_painter_accept(i.zs, terrain_z)) { discard; }
  let depth = road_frag_depth(i.zs);
  return RoadFrag(terrain_color(i.uv, i.zs, i.nrm), bitcast<u32>(i.zs), depth);
}
@fragment
fn fs_road_owner(i: TerrOut) -> @location(0) vec4<f32> {
  let won = textureLoad(u_road_mask, vec2<i32>(i.pos.xy), 0).x;
  if (won != bitcast<u32>(i.zs)) { discard; }
  return terrain_color(i.uv, i.zs, i.nrm);
}

/* ---- scene / vehicle meshes: authored RGB, optional texture ----------- */
struct MeshOut {
  @builtin(position) pos: vec4<f32>,
  @location(0) nrm: vec3<f32>,
  @location(1) uv: vec2<f32>,
  @location(2) rgb: vec3<f32>,
  @location(3) zs: f32,
};
@vertex
fn vs_mesh(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>,
           @location(2) uv: vec2<f32>, @location(3) rgb: vec3<f32>) -> MeshOut {
  var o: MeshOut;
  let world = u_model.model * vec4<f32>(p, 1.0);
  let clip = u_model.mvp * vec4<f32>(p, 1.0);
  o.pos = clip;
  // Normals are transformed by the model's rotation only; the scene xforms
  // are orthonormal bases (scene.c D4) so the basis itself is the correct
  // normal matrix.
  o.nrm = (u_model.model * vec4<f32>(n, 0.0)).xyz;
  o.uv = uv;
  o.rgb = rgb;
  o.zs = clip.w;
  return o;
}
/* Rear-view uses its own reflected frame VP, so it deliberately keeps the
 * ordinary local-origin VP*model path rather than the primary MVP. */
@vertex
fn vs_mesh_mirror(@location(0) p: vec3<f32>, @location(1) n: vec3<f32>,
                  @location(2) uv: vec2<f32>,
                  @location(3) rgb: vec3<f32>) -> MeshOut {
  var o: MeshOut;
  let world = u_model.model * vec4<f32>(p, 1.0);
  let clip = u_frame.vp * world;
  o.pos = clip;
  o.nrm = (u_model.model * vec4<f32>(n, 0.0)).xyz;
  o.uv = uv;
  o.rgb = rgb;
  o.zs = clip.w;
  return o;
}
@fragment
fn fs_mesh(i: MeshOut) -> @location(0) vec4<f32> {
  // u_model.color.a encodes the material mode: 0 = authored face RGB only,
  // 1 = sample the bound tile (opaque), 2 = sample + alpha cut-out
  // (flags2 5/7: headlights, fences, rails).
  var base = i.rgb;
  if (u_model.color.a > 0.5) {
    let t = indexed_world_sample(i.uv);
    base = t.rgb;
    if (u_model.color.a > 1.5 && t.a < 0.5) { discard; }
  }
  // posB normals point INWARD (measured: dot(posB, p - centroid) < 0 on every
  // mesh sampled), so negate to get the outward shading normal.
  let n = -normalize(i.nrm);
  let lam = max(dot(n, normalize(u_frame.light_dir.xyz)), 0.0);
  let lit = base * (u_frame.light_amb + (1.0 - u_frame.light_amb) * lam);
  return vec4<f32>(apply_fog(lit, i.zs), 1.0);
}

/* ---- sky: fullscreen triangle; authored dome or gradient --------------- */
struct SkyOut {
  @builtin(position) pos: vec4<f32>,
  @location(0) t: f32,
};
@vertex
fn vs_sky(@builtin(vertex_index) vi: u32) -> SkyOut {
  var xy = array<vec2<f32>, 3>(vec2f(-1.0, -3.0), vec2f(-1.0, 1.0), vec2f(3.0, 1.0));
  var o: SkyOut;
  let p = xy[vi];
  o.pos = vec4<f32>(p, 1.0, 1.0);       // z = 1: behind everything
  o.t = (1.0 - p.y) * 0.5;              // 0 at top of screen, 1 at bottom
  return o;
}
@fragment
fn fs_sky(i: SkyOut) -> @location(0) vec4<f32> {
  // alpha 0: sky is not geometry (coverage mask).
  if (u_frame.cam_proj.w > 0.5) {
    // Equirectangular dome matching worldrender draw_sky_tex (SKY_AZ_WRAPS=4,
    // SKY_ROWS_PER_RAD=256, SKY_HORIZON_ROW=64). Use fragment position
    // from the vertex-built clip pos (framebuffer coords after raster).
    //
    // Do NOT fract() before sampling: wrapping address mode + level-0
    // sample matches the software path's umask/vmask wrap. Pre-fract
    // with bilinear/mips drew visible tile "links" (derivative blow-up
    // at every azimuth wrap and at the tile edge).
    let px = i.pos.x - u_frame.cam_proj.y;
    let py = u_frame.cam_proj.z - i.pos.y;
    let f = u_frame.cam_proj.x;
    var d = f * u_frame.cam_forward.xyz + px * u_frame.cam_right.xyz
            + py * u_frame.cam_up.xyz;
    d = normalize(d);
    let dim = vec2<f32>(textureDimensions(u_sky_tex));
    let uu = 0.5 + atan2(d.x, d.z) / (2.0 * 3.14159265) * 4.0;
    let vv = (64.0 - asin(clamp(d.y, -1.0, 1.0)) * 256.0) / max(dim.y, 1.0);
    // Nearest + lod 0: software sky is point-sampled; mips at the
    // atan2 cut and at each of the 4 azimuth wraps left bright seams.
    let c = textureSampleLevel(u_sky_tex, u_sky_samp, vec2<f32>(uu, vv), 0.0);
    return vec4<f32>(c.rgb, 0.0);
  }
  return vec4<f32>(mix(u_frame.zenith.rgb, u_frame.sky.rgb, pow(i.t, 1.6)), 0.0);
}

/* ---- HUD overlay: 8-bit indexed layer through the level palette ------- */
struct HudOut {
  @builtin(position) pos: vec4<f32>,
  @location(0) uv: vec2<f32>,
};
@vertex
fn vs_hud(@builtin(vertex_index) vi: u32) -> HudOut {
  var xy = array<vec2<f32>, 3>(vec2f(-1.0, -3.0), vec2f(-1.0, 1.0), vec2f(3.0, 1.0));
  var o: HudOut;
  let p = xy[vi];
  o.pos = vec4<f32>(p, 0.0, 1.0);
  o.uv = vec2<f32>((p.x + 1.0) * 0.5, (1.0 - p.y) * 0.5);
  return o;
}
@fragment
fn fs_hud(i: HudOut) -> @location(0) vec4<f32> {
  let covered = textureSample(u_hud_mask, u_hud_samp, i.uv).r;
  if (covered < 0.5) { discard; }
  let idx = textureSample(u_hud, u_hud_samp, i.uv).r;
  let c = textureSample(u_pal, u_hud_samp, vec2<f32>(idx * (255.0 / 256.0) +
                                                     0.5 / 256.0, 0.5));
  return vec4<f32>(c.rgb, 1.0);
}

/* ---- rearview mirror pass --------------------------------------------- *
 * The mirror reuses the world vertex shaders above with two twists:
 *  1. the VP written to the mirror UBO is clip-x-flipped (flipVpX) — the
 *     image is a reflection, the GPU twin of the software composite's
 *     dst[x] = mirror_fb[mw-1-x];
 *  2. every fragment first tests the HUD overlay: where the cockpit art is
 *     opaque the mirror must NOT overwrite it — the software path draws
 *     the mirror BEFORE the HUD composite, so the dash always wins.
 * The HUD layer IS the authored mirror mask: when the rearview presents,
 * webmain.c copies the zmiri mask's non-0xFF housing texels into the
 * HUD-layer pixels the art leaves transparent (the mask has no 0x00
 * texels, so every housing texel lands opaque; the 0xFF glass stays
 * transparent). HUD opacity inside the rect is therefore exactly the
 * authored glass cutout, and the HUD pass paints the housing ring — the
 * GPU mirror matches the software composite, bezel included.
 */
fn hud_covers(fpos: vec4<f32>) -> bool {
  let dim = vec2<i32>(textureDimensions(u_hud_mask));
  let source = min(vec2<i32>(floor(fpos.xy * vec2<f32>(dim) /
                                     u_frame.render_target.xy)), dim - vec2i(1));
  return textureLoad(u_hud_mask, source, 0).r >= 0.5;
}

struct SkyMirrorIn { @location(0) t: f32, };
@fragment
fn fs_sky_mirror(@builtin(position) fpos: vec4<f32>, i: SkyMirrorIn) -> @location(0) vec4<f32> {
  if (hud_covers(fpos)) { discard; }
  if (u_frame.cam_proj.w > 0.5) {
    let px = fpos.x - u_frame.cam_proj.y;
    let py = u_frame.cam_proj.z - fpos.y;
    let f = u_frame.cam_proj.x;
    var d = f * u_frame.cam_forward.xyz + px * u_frame.cam_right.xyz
            + py * u_frame.cam_up.xyz;
    d = normalize(d);
    let dim = vec2<f32>(textureDimensions(u_sky_tex));
    let uu = 0.5 + atan2(d.x, d.z) / (2.0 * 3.14159265) * 4.0;
    let vv = (64.0 - asin(clamp(d.y, -1.0, 1.0)) * 256.0) / max(dim.y, 1.0);
    let c = textureSampleLevel(u_sky_tex, u_sky_samp, vec2<f32>(uu, vv), 0.0);
    return vec4<f32>(c.rgb, 0.0);
  }
  return vec4<f32>(mix(u_frame.zenith.rgb, u_frame.sky.rgb, pow(i.t, 1.6)), 0.0);
}

struct TerrMirrorIn {
  @location(0) uv: vec2<f32>,
  @location(1) zs: f32,
  @location(2) nrm: vec3<f32>,
};
fn terrain_mirror_color(uv: vec2<f32>, zs: f32, nrm: vec3<f32>) -> vec4<f32> {
  return terrain_color(uv, zs, nrm);
}
@fragment
fn fs_terrain_mirror(@builtin(position) fpos: vec4<f32>, i: TerrMirrorIn) -> @location(0) vec4<f32> {
  if (hud_covers(fpos)) { discard; }
  return terrain_mirror_color(i.uv, i.zs, i.nrm);
}
@fragment
fn fs_road_seed_mirror(@builtin(position) fpos: vec4<f32>, i: TerrMirrorIn) -> @location(0) f32 {
  if (hud_covers(fpos)) { discard; }
  return i.zs;
}
@fragment
fn fs_road_mirror(@builtin(position) fpos: vec4<f32>, i: TerrMirrorIn) -> RoadFrag {
  if (hud_covers(fpos)) { discard; }
  let terrain_z = textureLoad(u_road_terrain_z, vec2<i32>(fpos.xy), 0).x;
  if (!road_painter_accept(i.zs, terrain_z)) { discard; }
  let depth = road_frag_depth(i.zs);
  return RoadFrag(terrain_mirror_color(i.uv, i.zs, i.nrm), bitcast<u32>(i.zs), depth);
}
@fragment
fn fs_road_owner_mirror(@builtin(position) fpos: vec4<f32>, i: TerrMirrorIn) -> @location(0) vec4<f32> {
  if (hud_covers(fpos)) { discard; }
  let won = textureLoad(u_road_mask, vec2<i32>(fpos.xy), 0).x;
  if (won != bitcast<u32>(i.zs)) { discard; }
  return terrain_mirror_color(i.uv, i.zs, i.nrm);
}

struct MeshMirrorIn {
  @location(0) nrm: vec3<f32>,
  @location(1) uv: vec2<f32>,
  @location(2) rgb: vec3<f32>,
  @location(3) zs: f32,
};
@fragment
fn fs_mesh_mirror(@builtin(position) fpos: vec4<f32>, i: MeshMirrorIn) -> @location(0) vec4<f32> {
  if (hud_covers(fpos)) { discard; }
  // Same material modes as fs_mesh: 0 = face RGB, 1 = opaque tile,
  // 2 = tile + alpha cut-out.
  var base = i.rgb;
  if (u_model.color.a > 0.5) {
    let t = indexed_world_sample(i.uv);
    base = t.rgb;
    if (u_model.color.a > 1.5 && t.a < 0.5) { discard; }
  }
  // posB normals point INWARD — negate, as in fs_mesh.
  let n = -normalize(i.nrm);
  let lam = max(dot(n, normalize(u_frame.light_dir.xyz)), 0.0);
  let lit = base * (u_frame.light_amb + (1.0 - u_frame.light_amb) * lam);
  return vec4<f32>(apply_fog(lit, i.zs), 1.0);
}
`;

/* Box-downsample used to build mip chains (WebGPU has no generateMipmap). */
const MIP_WGSL = `
@group(0) @binding(0) var src: texture_2d<f32>;
@group(0) @binding(1) var smp: sampler;
struct O { @builtin(position) pos: vec4<f32>, @location(0) uv: vec2<f32> };
@vertex
fn vs(@builtin(vertex_index) vi: u32) -> O {
  var xy = array<vec2<f32>, 3>(vec2f(-1.0, -3.0), vec2f(-1.0, 1.0), vec2f(3.0, 1.0));
  var o: O;
  let p = xy[vi];
  o.pos = vec4<f32>(p, 0.0, 1.0);
  o.uv = vec2<f32>((p.x + 1.0) * 0.5, (1.0 - p.y) * 0.5);
  return o;
}
@fragment
fn fs(i: O) -> @location(0) vec4<f32> { return textureSample(src, smp, i.uv); }
`;

/* UFrame: vp(64) + scalars(16) + sky/light/zenith(48) + cam basis(48) + proj(16). */
export const UFRAME_BYTES = 256;
/* UModel: mat4 model (64) + primary MVP (64) + color vec4 (16). */
export const UMODEL_BYTES = 144;
/* Mesh vertex: pos3 + nrm3 + uv2 + rgb3 = 11 floats. */
export const MESH_VERT_BYTES = 44;
/*
 * Legacy uniform-grid UV period (8 samples × 5 m). LOD mesh path embeds
 * software UVs (wx * 10/w_tex) per vertex — tileM is then unused for UV.
 */
export const TERRAIN_UV_PERIOD_M = 40;
/* Mesh vertex stride for LOD terrain: pos3 + nrm3 + uv2 = 8 floats. */
export const LOD_VERT_FLOATS = 8;

/* ------------------------------------------------------------------ */
/* GpuScene — the Tier-2 renderer                                     */
/* ------------------------------------------------------------------ */

export class GpuScene {
  /*
   * device: GPUDevice. canvas: optional visible canvas (presented to when
   * given; the offscreen target is rendered only when a readback is asked
   * for, or when there is no canvas). skyRGB: horizon/fog colour, 0-255.
   * Geometry writes alpha 1 and the clear writes alpha 0, so readback alpha
   * remains the coverage mask the Tier-2 parity gate measures.
   */
  constructor(device, canvas, skyRGB = [0, 0, 0], w = FB_W, h = FB_H,
              zenithRGB = null, roadDiagnostics = false, options = {}) {
    this.device = device;
    this.w = w;
    this.h = h;
    this.paddedBpr = Math.ceil((w * 4) / 256) * 256;
    this.pointSampled = !!options.pointSampled;
    this.hudW = FB_W;
    this.hudH = FB_H;
    const d = device;

    this.rt = d.createTexture({
      size: [w, h], format: 'rgba8unorm',
      usage: GPUTextureUsage.RENDER_ATTACHMENT | GPUTextureUsage.COPY_SRC,
    });
    this.depth = d.createTexture({
      size: [w, h], format: 'depth24plus',
      usage: GPUTextureUsage.RENDER_ATTACHMENT,
    });
    this.depthView = this.depth.createView();
    /* GpuScene dimensions are immutable; the page recreates the whole scene
     * on target-size changes, so roadDepth shares depth/rt lifecycle. */
    this.roadDepth = d.createTexture({
      size: [w, h], format: 'depth24plus',
      usage: GPUTextureUsage.RENDER_ATTACHMENT,
    });
    this.roadDepthView = this.roadDepth.createView();
    this.roadMask = d.createTexture({
      size: [w, h], format: 'r32uint',
      usage: GPUTextureUsage.RENDER_ATTACHMENT | GPUTextureUsage.TEXTURE_BINDING |
             GPUTextureUsage.COPY_SRC,
    });
    this.roadMaskView = this.roadMask.createView();
    this.roadDiagnostics = roadDiagnostics;
    this.roadFootprint = roadDiagnostics ? d.createTexture({
      size: [w, h], format: 'r32uint',
      usage: GPUTextureUsage.RENDER_ATTACHMENT | GPUTextureUsage.COPY_SRC,
    }) : null;
    this.roadFootprintView = this.roadFootprint?.createView() ?? null;
    this.roadTerrainZ = d.createTexture({
      size: [w, h], format: 'r32float',
      usage: GPUTextureUsage.RENDER_ATTACHMENT | GPUTextureUsage.TEXTURE_BINDING,
    });
    this.roadTerrainZView = this.roadTerrainZ.createView();
    this.rtView = this.rt.createView();
    this.readBuf = d.createBuffer({
      size: this.paddedBpr * h,
      usage: GPUBufferUsage.COPY_DST | GPUBufferUsage.MAP_READ,
    });
    this.roadReadBuf = d.createBuffer({
      size: this.paddedBpr * h,
      usage: GPUBufferUsage.COPY_DST | GPUBufferUsage.MAP_READ,
    });
    this.frameUbo = d.createBuffer({
      size: UFRAME_BYTES,
      usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
    });
    /* Same layout as frameUbo, written only when the rearview mirror pass
     * presents — the mirror gets its own VP without disturbing the primary
     * frame's uniforms. */
    this.mirrorUbo = d.createBuffer({
      size: UFRAME_BYTES,
      usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST,
    });

    this.sampler = d.createSampler({
      magFilter: 'linear', minFilter: 'linear', mipmapFilter: 'linear',
      addressModeU: 'repeat', addressModeV: 'repeat',   // bilinear + mips
    });
    /* Sky: point-sample + wrap, matching software draw_sky_tex. The
     * sky dome uses this sampler (bind0 slot 5), not the mipmapped one. */
    this.skySampler = d.createSampler({
      magFilter: 'nearest', minFilter: 'nearest',
      addressModeU: 'repeat', addressModeV: 'repeat',
    });
    this.hudSampler = d.createSampler({
      magFilter: 'nearest', minFilter: 'nearest',
      addressModeU: 'clamp-to-edge', addressModeV: 'clamp-to-edge',
    });

    // Exact software-owned 2-D overlay: index plane + explicit coverage mask
    // (paper may write palette index 0 opaquely) + the 256x1 level palette.
    this.hudTex = d.createTexture({
      size: [this.hudW, this.hudH], format: 'r8unorm',
      usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST,
    });
    this.hudMaskTex = d.createTexture({
      size: [this.hudW, this.hudH], format: 'r8unorm',
      usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST,
    });
    this.palTex = d.createTexture({
      size: [256, 1], format: 'rgba8unorm',
      usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST,
    });
    this.hudValid = false;

    this.white = this._solidTexture([255, 255, 255, 255]);
    /* Placeholder 1x1 sky until setSkyTex; gradient used when skyOn=0. */
    this.skyTex = this._solidTexture([80, 120, 180, 255]);
    this.skyOn = false;
    this.camBasis = {
      right: [1, 0, 0], up: [0, 1, 0], forward: [0, 0, 1],
      focal: this.w * 0.5, cx: this.w * 0.5, cy: this.h * 0.5,
    };

    const bgl0 = d.createBindGroupLayout({
      entries: [
        { binding: 0, visibility: GPUShaderStage.VERTEX | GPUShaderStage.FRAGMENT,
          buffer: { type: 'uniform' } },
        { binding: 1, visibility: GPUShaderStage.FRAGMENT, texture: { sampleType: 'float' } },
        { binding: 2, visibility: GPUShaderStage.FRAGMENT, texture: { sampleType: 'float' } },
        { binding: 3, visibility: GPUShaderStage.FRAGMENT, sampler: { type: 'non-filtering' } },
        { binding: 4, visibility: GPUShaderStage.FRAGMENT, texture: { sampleType: 'float' } },
        { binding: 5, visibility: GPUShaderStage.FRAGMENT, sampler: { type: 'filtering' } },
        { binding: 6, visibility: GPUShaderStage.FRAGMENT, texture: { sampleType: 'float' } },
        { binding: 7, visibility: GPUShaderStage.FRAGMENT, texture: { sampleType: 'uint' } },
        { binding: 8, visibility: GPUShaderStage.FRAGMENT, texture: { sampleType: 'unfilterable-float' } },
      ],
    });
    /* The seed and compatibility passes sample the HUD coverage mask for the
     * rear-view cutout. The compatibility pass also samples roadTerrainZ,
     * which is never attached during that pass. Keep these layouts minimal:
     * the full layout's six textures remain below WebGPU's guaranteed limit. */
    const bgl0Seed = d.createBindGroupLayout({
      entries: [
        { binding: 0, visibility: GPUShaderStage.VERTEX | GPUShaderStage.FRAGMENT,
          buffer: { type: 'uniform' } },
        { binding: 6, visibility: GPUShaderStage.FRAGMENT,
          texture: { sampleType: 'float' } },
      ],
    });
    const bgl0Road = d.createBindGroupLayout({
      entries: [
        { binding: 0, visibility: GPUShaderStage.VERTEX | GPUShaderStage.FRAGMENT,
          buffer: { type: 'uniform' } },
        { binding: 6, visibility: GPUShaderStage.FRAGMENT,
          texture: { sampleType: 'float' } },
        { binding: 8, visibility: GPUShaderStage.FRAGMENT,
          texture: { sampleType: 'unfilterable-float' } },
      ],
    });
    const bgl1 = d.createBindGroupLayout({
      entries: [
        { binding: 0, visibility: GPUShaderStage.VERTEX | GPUShaderStage.FRAGMENT,
          buffer: { type: 'uniform' } },
      ],
    });
    const bgl2 = d.createBindGroupLayout({
      entries: [
        { binding: 0, visibility: GPUShaderStage.FRAGMENT, texture: { sampleType: 'float' } },
        { binding: 1, visibility: GPUShaderStage.FRAGMENT, sampler: { type: 'filtering' } },
      ],
    });
    this.bgl0Seed = bgl0Seed;
    this.bgl0Road = bgl0Road;
    this.bgl1 = bgl1;
    this.bgl2 = bgl2;

    const mod = d.createShaderModule({ code: SCENE_WGSL });
    this.mod = mod;
    const layFull = d.createPipelineLayout({ bindGroupLayouts: [bgl0, bgl1, bgl2] });
    const laySeed = d.createPipelineLayout({ bindGroupLayouts: [bgl0Seed, bgl1, bgl2] });
    const layRoad = d.createPipelineLayout({ bindGroupLayouts: [bgl0Road, bgl1, bgl2] });
    const layFlat = d.createPipelineLayout({ bindGroupLayouts: [bgl0] });

    const depthStencil = {
      format: 'depth24plus', depthWriteEnabled: true, depthCompare: 'less',
    };
    const depthStencilMesh = {
      format: 'depth24plus', depthWriteEnabled: true, depthCompare: 'less-equal',
    };
    /* nitro.exe submits RDEF roads after terrain into its software painter
     * queue (FUN_004959d0 -> FUN_00494420 -> FUN_00498a70): no z-buffer,
     * no +0.1 m lift, no road-only bias. WebGPU mirrors that decal policy
     * with a depth-tested pass. The shader's bounded linear view-Z test
     * handles road-vs-LOD-terrain drift; unbiased strict depth resolves road
     * intersections. A nearer hill remains in front. */
    const depthStencilRoad = ROAD_DEPTH_POLICY;
    this._depthStencilRoad = depthStencilRoad;
    // Terrain: pos3 + nrm3 + uv2 (LOD software UVs or grid-baked). Meshes: +rgb3.
    const TERR_BUFS = [{
      arrayStride: 32,
      attributes: [
        { shaderLocation: 0, offset: 0, format: 'float32x3' },
        { shaderLocation: 1, offset: 12, format: 'float32x3' },
        { shaderLocation: 2, offset: 24, format: 'float32x2' },
      ],
    }];
    const MESH_BUFS = [{
      arrayStride: MESH_VERT_BYTES,
      attributes: [
        { shaderLocation: 0, offset: 0, format: 'float32x3' },
        { shaderLocation: 1, offset: 12, format: 'float32x3' },
        { shaderLocation: 2, offset: 24, format: 'float32x2' },
        { shaderLocation: 3, offset: 32, format: 'float32x3' },
      ],
    }];
    /*
     * Backface culling: cull 'back'; see this.frontFace for why front faces
     * are clockwise in framebuffer space.
     */
    const mk = (layout, vs, fs, format, buffers, cull = 'back',
                depth = depthStencil, front = 'ccw', writeMask = 0xf,
                extraTargets = []) =>
      d.createRenderPipeline({
        layout,
        vertex: { module: mod, entryPoint: vs, buffers },
        fragment: { module: mod, entryPoint: fs,
                    targets: [{ format, writeMask }, ...extraTargets] },
        primitive: { topology: 'triangle-list', cullMode: cull, frontFace: front },
        ...(depth ? { depthStencil: depth } : {}),
      });
    this._mk = mk;
    this._layFull = layFull;
    this._laySeed = laySeed;
    this._layRoad = layRoad;
    this._layFlat = layFlat;
    this._terrBufs = TERR_BUFS;
    this._meshBufs2 = MESH_BUFS;
    this._depthStencil = depthStencil;
    this._depthStencilMesh = depthStencilMesh;
    /*
     * Winding, measured on this repo's own decoder over real N01 meshes
     * (signed volume of the emitted triangle set, and dot(posB, p-centroid)):
     *
     *   OEG meshes  — listed corner order is CW seen from OUTSIDE, and the
     *                 stored plane/posB normals point INWARD. ("CCW from the
     *                 +normal side" is true and consistent with this: the
     *                 +normal side is the inside.)
     *   terrain     — setTerrain emits (i,j),(i,j+1),(i+1,j), i.e. CCW seen
     *                 from ABOVE, the opposite sense.
     *
     * WebGPU decides facing from the signed area in FRAMEBUFFER space, and
     * the NDC->framebuffer y-flip inverts it once more. Net: the two layers
     * need OPPOSITE frontFace. Sharing one (as this did) silently culled a
     * whole layer — the player car rendered as a featureless black box
     * because only its interior faces survived.
     * Terrain 'cw' is verified empirically: with 'ccw' the ground in front of
     * the camera vanishes and the sky shows through the bottom of the frame.
     */
    this.terrFront = 'cw';
    this.meshFront = 'ccw';
    this._pipes = new Map();            // format -> { terr, mesh, sky, hud }

    this.ctx = canvas ? canvas.getContext('webgpu') : null;
    this.canvas = canvas ?? null;
    if (this.ctx) {
      const format = navigator.gpu.getPreferredCanvasFormat();
      this.ctx.configure({ device: d, format, alphaMode: 'opaque' });
      this.canvasFormat = format;
    }

    this.setSky(skyRGB, zenithRGB);
    this.tileM = TERRAIN_UV_PERIOD_M;
    this.fogStart = 900;
    this.fogEnd = 2600;
    /* Match software terrain/scene: TERRAIN_AMBIENT / SCENE_AMBIENT 0.35 and
     * the shared unit sun vector (placeholder, but the same placeholder). */
    this.lightAmb = 0.35;
    this.lightDir = [-0.45267873, 0.81482171, -0.36214298];
    this.terrain = null;
    this.terrainLod = false;   /* true when verts carry software UVs */
    this.roads = null;         /* [{ vbo, nVerts, bind2 } x types] */
    this.instances = [];
    this._meshBufs = new Map();         // ptr -> { vbo, groups }
    /* Optional dev/probe observer at the exact queue.submit seam. It receives
     * the f32 VP/model matrices written for that submission, rather than a
     * shared C pose that may never reach the device unchanged. */
    this.submissionObserver = null;
    this.submissionSerial = 0;
    this.worldOrigin = [0, 0, 0];
    this._texCache = new Map();         // (vtf\0name) -> { bind } | null
    this.bind0 = null;
    this._rebuildBind0(bgl0);
    this.bgl0 = bgl0;
    /* Mission-owned skins (destroyed on replacement / resetMission). */
    this.terrainTex = null;
    this._roadTex = null;              // [GPUTexture|null x3] once roads load
    this.terrainBind2 = this._bind2(this.white);
  }

  /* --- small helpers ------------------------------------------------- */

  _solidTexture(rgba) {
    const t = this.device.createTexture({
      size: [1, 1], format: 'rgba8unorm',
      usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST,
    });
    this.device.queue.writeTexture({ texture: t }, new Uint8Array(rgba),
      { bytesPerRow: 4, rowsPerImage: 1 },
      { width: 1, height: 1, depthArrayLayers: 1, depthOrArrayLayers: 1 });
    return t;
  }

  _rebuildBind0(bgl0 = this.bgl0) {
    const skyView = this.skyTex.createView();
    this.bind0 = this.device.createBindGroup({
      layout: bgl0,
      entries: [
        { binding: 0, resource: { buffer: this.frameUbo } },
        { binding: 1, resource: this.hudTex.createView() },
        { binding: 2, resource: this.palTex.createView() },
        { binding: 3, resource: this.hudSampler },
        { binding: 4, resource: skyView },
        { binding: 5, resource: this.skySampler },
        { binding: 6, resource: this.hudMaskTex.createView() },
        { binding: 7, resource: this.roadMaskView },
        { binding: 8, resource: this.roadTerrainZView },
      ],
    });
    this.mirrorBind0 = this.device.createBindGroup({
      layout: bgl0,
      entries: [
        { binding: 0, resource: { buffer: this.mirrorUbo } },
        { binding: 1, resource: this.hudTex.createView() },
        { binding: 2, resource: this.palTex.createView() },
        { binding: 3, resource: this.hudSampler },
        { binding: 4, resource: skyView },
        { binding: 5, resource: this.skySampler },
        { binding: 6, resource: this.hudMaskTex.createView() },
        { binding: 7, resource: this.roadMaskView },
        { binding: 8, resource: this.roadTerrainZView },
      ],
    });
    const roadEntries = ubo => [
      { binding: 0, resource: { buffer: ubo } },
      { binding: 6, resource: this.hudMaskTex.createView() },
      { binding: 8, resource: this.roadTerrainZView },
    ];
    const seedEntries = ubo => [
      { binding: 0, resource: { buffer: ubo } },
      { binding: 6, resource: this.hudMaskTex.createView() },
    ];
    this.seedBind0 = this.device.createBindGroup({
      layout: this.bgl0Seed, entries: seedEntries(this.frameUbo),
    });
    this.mirrorSeedBind0 = this.device.createBindGroup({
      layout: this.bgl0Seed, entries: seedEntries(this.mirrorUbo),
    });
    this.roadBind0 = this.device.createBindGroup({
      layout: this.bgl0Road, entries: roadEntries(this.frameUbo),
    });
    this.mirrorRoadBind0 = this.device.createBindGroup({
      layout: this.bgl0Road, entries: roadEntries(this.mirrorUbo),
    });
  }

  /* The per-frame uniform block (VP + fog/light/sky + cam for sky dome).
   * basis defaults to the primary camera; the mirror pass passes its
   * reflected rear basis (mirrorBasis). */
  _frameUniforms(vpF32, basis = this.camBasis) {
    const u = new Float32Array(UFRAME_BYTES / 4);
    u.set(vpF32, 0);
    u[16] = this.tileM;
    u[17] = this.fogStart;
    u[18] = this.fogEnd;
    u[19] = this.lightAmb;
    u.set([this.sky[0], this.sky[1], this.sky[2], 1], 20);
    u.set([this.lightDir[0], this.lightDir[1], this.lightDir[2], 0], 24);
    u.set([this.zenith[0], this.zenith[1], this.zenith[2], 1], 28);
    const b = basis;
    u.set([b.right[0], b.right[1], b.right[2], 0], 32);
    u.set([b.up[0], b.up[1], b.up[2], 0], 36);
    u.set([b.forward[0], b.forward[1], b.forward[2], 0], 40);
    u.set([b.focal, b.cx, b.cy, this.skyOn ? 1 : 0], 44);
    u.set([this.w, this.h, this.pointSampled ? 1 : 0, 0], 48);
    return u;
  }

  /** Authored cloud .map as RGBA. Null clears to gradient sky. The previous
   * sky texture is released — sky skins are mission-owned, never per-frame. */
  setSkyTex(tex) {
    if (this.skyTex) this.skyTex.destroy();
    if (!tex || !tex.w || !tex.rgba) {
      this.skyOn = false;
      this.skyTex = this._solidTexture([80, 120, 180, 255]);
      this._rebuildBind0();
      return;
    }
    this.skyTex = this._uploadTexture(tex.w, tex.h, tex.rgba);
    this.skyOn = true;
    this._rebuildBind0();
  }

  /** Live camera basis for equirectangular sky (from readCamera). */
  setCameraBasis(cam) {
    if (!cam) return;
    const fth = (cam.fovTanHalf > 0) ? cam.fovTanHalf : 1;
    this.camBasis = {
      right: cam.right, up: cam.up, forward: cam.forward,
      focal: (this.w * 0.5) / fth,
      cx: this.w * 0.5, cy: this.h * 0.5,
    };
  }

  _bind2(tex) {
    return this.device.createBindGroup({
      layout: this.bgl2,
      entries: [
        { binding: 0, resource: tex.createView() },
        { binding: 1, resource: this.sampler },
      ],
    });
  }

  /* Pipelines are per colour-target format (offscreen rgba8unorm vs the
   * canvas's preferred format); built lazily and cached. */
  _pipesFor(format) {
    let p = this._pipes.get(format);
    if (!p) {
      p = {
        terr: this._mk(this._layFull, 'vs_terrain', 'fs_terrain', format,
                       this._terrBufs, 'back', this._depthStencil, this.terrFront),
        roadSeed: this._mk(this._laySeed, 'vs_terrain', 'fs_road_seed', 'r32float',
                       this._terrBufs, 'back', this._depthStencil, this.terrFront),
        roadFootprint: this._mk(this._laySeed, 'vs_terrain', 'fs_road_footprint',
                       'r32uint', this._terrBufs, 'back', null, this.terrFront),
        roadOwner: this._mk(this._layFull, 'vs_terrain', 'fs_road_owner', format,
                       this._terrBufs, 'back', ROAD_OWNER_DEPTH_POLICY,
                       this.terrFront, 0),
        /* Bounded shader acceptance plus unbiased road/owner depth. */
        road: this._mk(this._layRoad, 'vs_terrain', 'fs_road', format,
                       this._terrBufs, 'back', this._depthStencilRoad,
                       this.terrFront, 0xf, [{ format: 'r32uint' }]),
        mesh: this._mk(this._layFull, 'vs_mesh', 'fs_mesh', format,
                       this._meshBufs2, 'back', this._depthStencilMesh, this.meshFront),
        // Sky writes at z=1 with depthCompare 'less-equal' and no depth write
        // so it fills only untouched pixels; drawn first, it costs one pass.
        sky: this._mk(this._layFlat, 'vs_sky', 'fs_sky', format, [], 'none',
          { format: 'depth24plus', depthWriteEnabled: false, depthCompare: 'less-equal' }),
        // HUD is an overlay: no depth interaction at all.
        // RGB only: preserve alpha as the world-geometry coverage diagnostic
        // even when an opaque title/map covers the whole colour frame.
        hud: this._mk(this._layFlat, 'vs_hud', 'fs_hud', format, [], 'none',
          { format: 'depth24plus', depthWriteEnabled: false, depthCompare: 'always' },
          'ccw', 0x7),
        /*
         * Mirror variants of the world passes (see fs_*_mirror): same
         * geometry, but each fragment discards where the HUD overlay is
         * opaque — the dash always wins over the mirror — and frontFace is
         * INVERTED because flipVpX flips clip-space x and with it the
         * framebuffer-space winding. The sky keeps cull 'none'; its
         * gradient is vertical, so the x-flip leaves it unchanged.
         */
        skyMirror: this._mk(this._layFlat, 'vs_sky', 'fs_sky_mirror', format,
          [], 'none',
          { format: 'depth24plus', depthWriteEnabled: false,
            depthCompare: 'less-equal' }),
        terrMirror: this._mk(this._layFull, 'vs_terrain', 'fs_terrain_mirror',
          format, this._terrBufs, 'back', this._depthStencil,
          flipFront(this.terrFront)),
        roadSeedMirror: this._mk(this._laySeed, 'vs_terrain', 'fs_road_seed_mirror',
          'r32float', this._terrBufs, 'back', this._depthStencil,
          flipFront(this.terrFront)),
        roadOwnerMirror: this._mk(this._layFull, 'vs_terrain', 'fs_road_owner_mirror',
          format, this._terrBufs, 'back', ROAD_OWNER_DEPTH_POLICY,
          flipFront(this.terrFront), 0),
        roadMirror: this._mk(this._layRoad, 'vs_terrain', 'fs_road_mirror',
          format, this._terrBufs, 'back', this._depthStencilRoad,
          flipFront(this.terrFront), 0xf, [{ format: 'r32uint' }]),
        meshMirror: this._mk(this._layFull, 'vs_mesh_mirror', 'fs_mesh_mirror',
          format, this._meshBufs2, 'back', this._depthStencilMesh,
          flipFront(this.meshFront)),
      };
      this._pipes.set(format, p);
    }
    return p;
  }

  /* Horizon / fog colour (0-255). Fogged geometry melts into the sky band. */
  setSky(rgb, zenithRGB = null) {
    this.sky = [rgb[0] / 255, rgb[1] / 255, rgb[2] / 255];
    const z = zenithRGB ?? [rgb[0] * 0.35, rgb[1] * 0.42, rgb[2] * 0.62];
    this.zenith = [z[0] / 255, z[1] / 255, z[2] / 255];
    this.clear = { r: this.sky[0], g: this.sky[1], b: this.sky[2], a: 0 };
  }

  /*
   * Upload the complete software-owned post-world 2-D layer and palette.
   * `mask` is explicit because map/title art can write index 0 opaquely.
   * Legacy HUD-only callers may omit it; their established nonzero key is
   * expanded here so focused mirror/HUD probes retain their old seam.
   */
  setHud(indices, palRGB, mask = null) {
    if (!indices || !palRGB) { this.hudValid = false; return; }
    const d = this.device;
    const bpr = Math.ceil(this.hudW / 256) * 256;
    const bytes = this.hudW * this.hudH;
    if (indices.length < bytes || palRGB.length < 256 * 3 ||
        (mask && mask.length < bytes))
      throw new Error('GpuScene.setHud: short 640x480 overlay or palette');
    const packRows = (plane) => {
      if (bpr === this.hudW) return plane;
      const out = new Uint8Array(bpr * this.hudH);
      for (let y = 0; y < this.hudH; y++)
        out.set(plane.subarray(y * this.hudW, (y + 1) * this.hudW), y * bpr);
      return out;
    };
    let coverage = mask;
    if (!coverage) {
      coverage = new Uint8Array(bytes);
      for (let i = 0; i < coverage.length; i++)
        coverage[i] = indices[i] === 0 ? 0 : 0xff;
    }
    d.queue.writeTexture({ texture: this.hudTex }, packRows(indices),
      { bytesPerRow: bpr, rowsPerImage: this.hudH },
      { width: this.hudW, height: this.hudH, depthOrArrayLayers: 1 });
    d.queue.writeTexture({ texture: this.hudMaskTex }, packRows(coverage),
      { bytesPerRow: bpr, rowsPerImage: this.hudH },
      { width: this.hudW, height: this.hudH, depthOrArrayLayers: 1 });
    const pal = new Uint8Array(256 * 4);
    for (let i = 0; i < 256; i++) {
      pal[i * 4] = palRGB[i * 3];
      pal[i * 4 + 1] = palRGB[i * 3 + 1];
      pal[i * 4 + 2] = palRGB[i * 3 + 2];
      pal[i * 4 + 3] = 255;
    }
    d.queue.writeTexture({ texture: this.palTex }, pal,
      { bytesPerRow: 256 * 4, rowsPerImage: 1 },
      { width: 256, height: 1, depthOrArrayLayers: 1 });
    this.hudValid = true;
  }

  /* RGBA8 upload with a full mip chain (period-correct trilinear minification;
   * without mips the tiled terrain shimmers badly at distance). */
  _uploadTexture(w, h, rgba) {
    const d = this.device;
    const levels = 1 + Math.floor(Math.log2(Math.max(w, h)));
    const tex = d.createTexture({
      size: [w, h], format: 'rgba8unorm', mipLevelCount: levels,
      usage: GPUTextureUsage.TEXTURE_BINDING | GPUTextureUsage.COPY_DST |
             GPUTextureUsage.RENDER_ATTACHMENT,
    });
    d.queue.writeTexture({ texture: tex }, rgba,
      { bytesPerRow: w * 4, rowsPerImage: h },
      { width: w, height: h, depthOrArrayLayers: 1 });
    if (levels > 1) this._genMips(tex, w, h, levels);
    return tex;
  }

  _genMips(tex, w, h, levels) {
    const d = this.device;
    if (!this._mipPipe) {
      this._mipMod = d.createShaderModule({ code: MIP_WGSL });
      this._mipPipe = d.createRenderPipeline({
        layout: 'auto',
        vertex: { module: this._mipMod, entryPoint: 'vs' },
        fragment: { module: this._mipMod, entryPoint: 'fs',
                    targets: [{ format: 'rgba8unorm' }] },
        primitive: { topology: 'triangle-list' },
      });
      this._mipSamp = d.createSampler({ magFilter: 'linear', minFilter: 'linear' });
    }
    const enc = d.createCommandEncoder();
    for (let l = 1; l < levels; l++) {
      const bind = d.createBindGroup({
        layout: this._mipPipe.getBindGroupLayout(0),
        entries: [
          { binding: 0, resource: tex.createView({ baseMipLevel: l - 1, mipLevelCount: 1 }) },
          { binding: 1, resource: this._mipSamp },
        ],
      });
      const pass = enc.beginRenderPass({
        colorAttachments: [{
          view: tex.createView({ baseMipLevel: l, mipLevelCount: 1 }),
          loadOp: 'clear', storeOp: 'store',
          clearValue: { r: 0, g: 0, b: 0, a: 0 },
        }],
      });
      pass.setPipeline(this._mipPipe);
      pass.setBindGroup(0, bind);
      pass.draw(3);
      pass.end();
    }
    d.queue.submit([enc.finish()]);
  }

  /*
   * Resolve an OEG face texture name to a bind group, via wasm
   * web_gpu_face_tex_load(name, vtf). Cached per (name, vtf) for the
   * mission (resetMission drops the cache and destroys the textures); an
   * unresolvable name caches null so the face falls back to its authored
   * RGB instead of retrying every rebuild. vtf is the owning vehicle's
   * paint scheme — required so two cars sharing "V1 FT LF.MAP" slots do
   * not inherit the first-resolved (player) panels.
   */
  _faceTexture(M, name, vtf = '') {
    if (!name) return null;
    const key = (vtf || '') + '\0' + name;
    if (this._texCache.has(key)) return this._texCache.get(key);
    let entry = null;
    if (M && M._web_gpu_face_tex_load &&
        M.ccall('web_gpu_face_tex_load', 'number',
                ['string', 'string'], [name, vtf || '']) === 0) {
      const w = M._web_gpu_m16_w(), h = M._web_gpu_m16_h();
      const p = M._web_gpu_m16_rgba();
      if (w > 0 && h > 0 && p) {
        const rgba = heap8(M).slice(p, p + w * h * 4);
        const tex = this._uploadTexture(w, h, rgba);
        entry = { tex, bind: this._bind2(tex) };
      }
    }
    this._texCache.set(key, entry);
    return entry;
  }

  /* --- geometry ------------------------------------------------------ */

  /*
   * Keep GPU coordinates local to the camera's stable 80 m LOD cell. The
   * software rasterizer subtracts its double-precision eye before projection;
   * feeding ~48 km world translations through separate f32 VP/model uniforms
   * instead lost sub-pixel cockpit motion when those values cancelled in the
   * vertex shader. The caller changes this only when it also rebuilds the
   * cell-owned terrain/road VBOs; instance models are rebased on every upload.
   */
  setWorldOrigin(origin) {
    if (!origin || origin.length < 3 || origin.some((v) => !Number.isFinite(v)))
      throw new Error('GpuScene.setWorldOrigin: invalid origin');
    const next = [Number(origin[0]), Number(origin[1]), Number(origin[2])];
    const changed = next.some((v, i) => v !== this.worldOrigin[i]);
    this.worldOrigin = next;
    return changed;
  }

  _rebaseVertexStream(src) {
    /* Preserve setTerrainLod's ArrayBuffer input contract; Float32Array.from
     * treats a bare ArrayBuffer as length zero rather than viewing its bytes. */
    const values = src instanceof ArrayBuffer ? new Float32Array(src) : src;
    const out = Float32Array.from(values);
    for (let i = 0; i + 2 < out.length; i += LOD_VERT_FLOATS) {
      out[i] -= this.worldOrigin[0];
      out[i + 1] -= this.worldOrigin[1];
      out[i + 2] -= this.worldOrigin[2];
    }
    return out;
  }

  _ensureTerrainUbo() {
    const d = this.device;
    if (this.terrainUbo) return;
    this.terrainUbo = d.createBuffer({
      size: UMODEL_BYTES,
      usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
    this.terrainBind1 = d.createBindGroup({
      layout: this.bgl1,
      entries: [{ binding: 0, resource: { buffer: this.terrainUbo } }],
    });
    const u = new Float32Array(UMODEL_BYTES / 4);
    const identity = [1, 0, 0, 0, 0, 1, 0, 0,
                      0, 0, 1, 0, 0, 0, 0, 1];
    u.set(identity, 0);
    u.set(identity, 16);
    u.set([1, 1, 1, 1], 32);
    d.queue.writeBuffer(this.terrainUbo, 0, u);
  }

  /*
   * Preferred path: software LOD ring mesh (pos3 nrm3 uv2 triangle list).
   * Same bands as terrain_render_filled — geometric parity with SW depth.
   */
  setTerrainLod(lodVerts, nverts, tex) {
    const d = this.device;
    if (this.terrain) {
      this.terrain.vbo.destroy();
      if (this.terrain.ibo) this.terrain.ibo.destroy();
      this.terrain = null;
    }
    this.terrainLod = false;
    if (lodVerts && nverts > 0) {
      const verts = this._rebaseVertexStream(lodVerts);
      const vbo = d.createBuffer({
        size: verts.byteLength,
        usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST });
      d.queue.writeBuffer(vbo, 0, verts);
      this.terrain = { vbo, ibo: null, nIdx: 0, nVerts: nverts, lod: true };
      this.terrainLod = true;
      this._ensureTerrainUbo();
    }
    if (tex) this.setTerrainTexture(tex);
  }

  /*
   * Upload/replace the mission surface skin (WRLD .map tile). Persistent
   * for the mission: LOD ring rebuilds reuse it, resetMission/destroy
   * release it. Software UV period = w_tex/10 m (TR_TEX_M_PER_10); LOD
   * verts already bake that scale into uv, so tileM only feeds the legacy
   * grid path.
   */
  setTerrainTexture(tex) {
    if (!tex || !tex.w || !tex.rgba) return;
    if (this.terrainTex) this.terrainTex.destroy();
    this.tileM = tex.w > 0 ? tex.w / 10 : TERRAIN_UV_PERIOD_M;
    this.terrainTex = this._uploadTexture(tex.w, tex.h, tex.rgba);
    this.terrainBind2 = this._bind2(this.terrainTex);
  }

  /*
   * Road ribbons (fill_roads twin). packs = { verts, nverts, byType:[n0,n1,n2],
   * tex:[{w,h,rgba}|null x3] }. Drawn after terrain with the terrain pipeline.
   * Geometry (verts) tracks the camera's snapped LOD cell; the per-type
   * tiles are mission-persistent — replaced only when packs.tex supplies
   * them, so a cell rebuild uploads no textures.
   */
  setRoads(packs) {
    const d = this.device;
    if (this.roads) {
      for (const r of this.roads) if (r && r.vbo) r.vbo.destroy();
      this.roads = null;
    }
    if (!packs || !packs.verts || !packs.nverts) return;
    this._ensureTerrainUbo();
    if (!this._roadTex) this._roadTex = [null, null, null];
    if (packs.tex)
      for (let t = 0; t < 3; t++) {
        const tex = packs.tex[t];
        if (tex && tex.w > 0 && tex.rgba) {
          if (this._roadTex[t]) this._roadTex[t].destroy();
          this._roadTex[t] = this._uploadTexture(tex.w, tex.h, tex.rgba);
        }
      }
    const out = [];
    let off = 0;
    for (let t = 0; t < 3; t++) {
      const nv = packs.byType[t] | 0;
      if (nv <= 0) { out.push(null); continue; }
      const slice = packs.verts.subarray(off * LOD_VERT_FLOATS,
                                         (off + nv) * LOD_VERT_FLOATS);
      off += nv;
      const verts = this._rebaseVertexStream(slice);
      const vbo = d.createBuffer({
        size: verts.byteLength,
        usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST });
      d.queue.writeBuffer(vbo, 0, verts);
      const bind2 = this._roadTex[t]
        ? this._bind2(this._roadTex[t]) : this.terrainBind2;
      out.push({ vbo, nVerts: nv, bind2 });
    }
    this.roads = out;
  }

  /* Upload the legacy uniform grid + surface texture (fallback). */
  setTerrain(grid, tex) {
    const d = this.device;
    if (this.terrain) {
      this.terrain.vbo.destroy();
      if (this.terrain.ibo) this.terrain.ibo.destroy();
      this.terrain = null;
    }
    this.terrainLod = false;
    if (grid) {
      // pos3 + nrm3 + uv2; UV matches software period (wx * 10/w_tex).
      const { cols, rows, wx0, wz0, step, heights } = grid;
      const period = (tex && tex.w > 0) ? tex.w / 10 : TERRAIN_UV_PERIOD_M;
      const verts = new Float32Array(cols * rows * 8);
      const at = (i, j) => heights[Math.min(rows - 1, Math.max(0, j)) * cols +
                                   Math.min(cols - 1, Math.max(0, i))];
      let k = 0;
      for (let j = 0; j < rows; j++)
        for (let i = 0; i < cols; i++) {
          const hL = at(i - 1, j), hR = at(i + 1, j);
          const hD = at(i, j - 1), hU = at(i, j + 1);
          let nx = hL - hR, ny = 2 * step, nz = hD - hU;
          const il = 1 / Math.hypot(nx, ny, nz);
          const wx = wx0 + i * step, wz = wz0 + j * step;
          verts[k++] = wx - this.worldOrigin[0];
          verts[k++] = heights[j * cols + i] - this.worldOrigin[1];
          verts[k++] = wz - this.worldOrigin[2];
          verts[k++] = nx * il; verts[k++] = ny * il; verts[k++] = nz * il;
          verts[k++] = wx / period; verts[k++] = wz / period;
        }
      // CCW when seen from above (+normal side), matching cullMode 'back'.
      const idx = new Uint32Array((cols - 1) * (rows - 1) * 6);
      k = 0;
      for (let j = 0; j + 1 < rows; j++)
        for (let i = 0; i + 1 < cols; i++) {
          const a = j * cols + i, b = a + 1;
          const c = a + cols, e = c + 1;
          idx[k++] = a; idx[k++] = c; idx[k++] = b;
          idx[k++] = b; idx[k++] = c; idx[k++] = e;
        }
      const vbo = d.createBuffer({
        size: verts.byteLength,
        usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST });
      d.queue.writeBuffer(vbo, 0, verts);
      const ibo = d.createBuffer({
        size: idx.byteLength,
        usage: GPUBufferUsage.INDEX | GPUBufferUsage.COPY_DST });
      d.queue.writeBuffer(ibo, 0, idx);
      this.terrain = { vbo, ibo, nIdx: idx.length, lod: false };
      this._ensureTerrainUbo();
    }
    if (tex) this.setTerrainTexture(tex);
  }

  /*
   * Upload instance draws (collectDraws output). `M` is the wasm module, used
   * to resolve face texture names; omit it to render meshes on their authored
   * colours only.
   *
   * Each material group of each instance gets its OWN model UBO + bind
   * group: queue.writeBuffer lands before the submit that reads it, so a
   * shared per-instance UBO rewritten per group would make every queued
   * draw observe the FINAL group's state. Per-group buffers let each
   * encoded draw see its own material mode (color.a: 0 = face RGB,
   * 1 = opaque tile, 2 = cut-out tile — the readMesh cutout flag).
   */
  setInstances(scene, M = null) {
    const d = this.device;
    this._clearInstances();
    for (const draw of scene.draws) {
      /* Geometry is shared per mesh pointer; paint tiles are NOT — two
       * chassis instances with different .vtf schemes must resolve their
       * "V1 …" slots independently. Key the uploaded VBO by ptr, and
       * resolve face textures per instance from draw.vtf. */
      let g = this._meshBufs.get(draw.ptr);
      if (g === undefined) {
        const data = draw.mesh.vbo;
        if (!data || !data.length) { this._meshBufs.set(draw.ptr, null); continue; }
        const vbo = d.createBuffer({
          size: data.byteLength,
          usage: GPUBufferUsage.VERTEX | GPUBufferUsage.COPY_DST });
        d.queue.writeBuffer(vbo, 0, data);
        g = { vbo, groups: draw.mesh.groups.map((grp) => ({
          first: grp.first, count: grp.count, cutout: !!grp.cutout,
          texName: grp.tex || '',
        })) };
        this._meshBufs.set(draw.ptr, g);
      }
      if (!g) continue;
      const inst = { vbo: g.vbo, kind: draw.kind, draw, groups: [] };
      const vtf = draw.vtf || '';
      for (const grp of g.groups) {
        const ubo = d.createBuffer({
          size: UMODEL_BYTES,
          usage: GPUBufferUsage.UNIFORM | GPUBufferUsage.COPY_DST });
        const bind1 = d.createBindGroup({
          layout: this.bgl1,
          entries: [{ binding: 0, resource: { buffer: ubo } }],
        });
        const tex = this._faceTexture(M, grp.texName, vtf);
        // Material mode for fs_mesh: a textured bucket samples its tile;
        // the cut-out flag (flags2 5/7) adds the alpha discard.
        const mode = tex ? (grp.cutout ? 2 : 1) : 0;
        inst.groups.push({ first: grp.first, count: grp.count,
                           tex, mode, ubo, bind1 });
      }
      this.instances.push(inst);
      this._writeModels(inst);
    }
  }

  /* Release instance model UBOs and shared mesh vertex buffers. */
  _clearInstances() {
    for (const inst of this.instances)
      for (const g of inst.groups) g.ubo.destroy();
    for (const g of this._meshBufs.values()) if (g) g.vbo.destroy();
    this._meshBufs = new Map();
    this.instances = [];
  }

  /*
   * Push each group UBO to the instance's current model matrix and the
   * group's material mode (color.a). Called once per visible instance per
   * frame, before encoding — every buffer is written exactly once per
   * frame, so no queued draw can observe another group's state. Reuses a
   * scratch staging array (no per-frame allocation).
   */
  _writeModels(inst, vp = null, traceModels = null) {
    const u = this._modelScratch ??
      (this._modelScratch = new Float32Array(UMODEL_BYTES / 4));
    const model = inst.draw.model;
    u.set(model, 0);
    u[12] = model[12] - this.worldOrigin[0];
    u[13] = model[13] - this.worldOrigin[1];
    u[14] = model[14] - this.worldOrigin[2];
    if (vp) {
      /* Compose directly into the persistent f32 UBO scratch. The operands
       * remain JS doubles until assignment, with no per-draw render-loop
       * arrays (trace copies below exist only when a probe opts in). */
      for (let c = 0; c < 4; c++)
        for (let r = 0; r < 4; r++) {
          let sum = 0;
          for (let k = 0; k < 4; k++) {
            let b = model[c * 4 + k];
            if (c === 3 && k < 3) b -= this.worldOrigin[k];
            sum += vp[k * 4 + r] * b;
          }
          u[16 + c * 4 + r] = sum;
        }
    } else {
      u.copyWithin(16, 0, 16);
    }
    if (traceModels && inst.draw.kind !== 'scene') {
      traceModels.push({
        kind: inst.draw.kind, drawIndex: inst.draw.drawIndex,
        model: Array.from(u.subarray(0, 16)),
        mvp: Array.from(u.subarray(16, 32)),
      });
    }
    for (const g of inst.groups) {
      u[32] = 1; u[33] = 1; u[34] = 1; u[35] = g.mode;
      this.device.queue.writeBuffer(g.ubo, 0, u);
    }
  }

  /* --- draw ---------------------------------------------------------- */

  /*
   * Draw one frame. vpF32: Float32Array(16) from cameraViewProj.
   * scene: collectDraws result (draw transforms/visibility come from the
   * inst.draw references refreshDraws keeps live). readback: when true,
   * also render the offscreen target and return its unpadded RGBA
   * (row 0 = top). mirror: optional { vp, cam, x, y, w, h } from
   * readRearview — a second world pass (sky/terrain/meshes, NO HUD)
   * confined to the cockpit mirror rect; cam is the rear CameraView the
   * mirror sky dome needs.
   */
  async frame(vpF32, scene, readback = false, mirror = null) {
    const d = this.device;
    if (this._mapping) throw new Error('GpuScene.frame: readback already in flight');

    const frameUniforms = this._frameUniforms(vpF32);
    d.queue.writeBuffer(this.frameUbo, 0, frameUniforms);
    // One writeBuffer per group UBO per frame, BEFORE the encoder submits:
    // every encoded draw observes its own group's material state.
    const tracedModels = this.submissionObserver ? [] : null;
    for (const inst of this.instances)
      if (!inst.draw.hidden) this._writeModels(inst, vpF32, tracedModels);

    const enc = d.createCommandEncoder();
    /* drawHud: presentation and offscreen-only readback get instruments.
     * When a canvas exists, the second offscreen target remains the geometry
     * diagnostic used by the older coverage probe. Rung 1 uses ?present=0. */
    const drawInto = (format, view, drawHud) => {
      const P = this._pipesFor(format);
      const drawTerrain = (pass, pipeline) => {
        if (!this.terrain) return;
        pass.setPipeline(pipeline);
        pass.setBindGroup(1, this.terrainBind1);
        pass.setBindGroup(2, this.terrainBind2);
        pass.setVertexBuffer(0, this.terrain.vbo);
        if (this.terrain.lod) pass.draw(this.terrain.nVerts);
        else {
          pass.setIndexBuffer(this.terrain.ibo, 'uint32');
          pass.drawIndexed(this.terrain.nIdx);
        }
      };
      let pass = enc.beginRenderPass({
        colorAttachments: [{
          view, loadOp: 'clear', storeOp: 'store', clearValue: this.clear,
        }],
        depthStencilAttachment: {
          view: this.depthView,
          depthClearValue: 1, depthLoadOp: 'clear', depthStoreOp: 'store',
        },
      });
      pass.setBindGroup(0, this.bind0);

      pass.setPipeline(P.sky);            // horizon first, depth-test only
      pass.draw(3);

      drawTerrain(pass, P.terr);
      if (this.roads) {
        pass.end();
        if (this.roadDiagnostics) {
          /* Exact projected-road footprint before ownership/depth policy.
           * This extra pass exists only for the opted-in temporal gate. */
          pass = enc.beginRenderPass({
            colorAttachments: [{
              view: this.roadFootprintView, loadOp: 'clear', storeOp: 'store',
              clearValue: { r: 0, g: 0, b: 0, a: 0 },
            }],
          });
          pass.setBindGroup(0, this.seedBind0);
          pass.setPipeline(P.roadFootprint);
          pass.setBindGroup(1, this.terrainBind1);
          for (const r of this.roads) {
            if (!r) continue;
            pass.setBindGroup(2, r.bind2);
            pass.setVertexBuffer(0, r.vbo);
            pass.draw(r.nVerts);
          }
          pass.end();
        }
        /* Seed visible linear terrain view-Z, then clear the temporary depth
         * before rendering roads: the shader owns terrain acceptance and the
         * isolated depth attachment owns nearest-road ordering only. */
        pass = enc.beginRenderPass({
          colorAttachments: [{
            view: this.roadTerrainZView, loadOp: 'clear', storeOp: 'store',
            clearValue: { r: 0, g: 0, b: 0, a: 0 },
          }],
          depthStencilAttachment: {
            view: this.roadDepthView,
            depthClearValue: 1, depthLoadOp: 'clear', depthStoreOp: 'store',
          },
        });
        pass.setBindGroup(0, this.seedBind0);
        drawTerrain(pass, P.roadSeed);
        pass.end();

        pass = enc.beginRenderPass({
          colorAttachments: [
            { view, loadOp: 'load', storeOp: 'store' },
            { view: this.roadMaskView, loadOp: 'clear', storeOp: 'store',
              clearValue: { r: 0, g: 0, b: 0, a: 0 } },
          ],
          depthStencilAttachment: {
            /* road_painter_accept is the sole terrain-ownership authority.
             * Reset the seed depth so depth24plus cannot independently reject
             * a fragment inside the continuous view-Z envelope; this depth is
             * now only the nearest-road resolver. */
            view: this.roadDepthView, depthClearValue: 1,
            depthLoadOp: 'clear', depthStoreOp: 'store',
          },
        });
        pass.setBindGroup(0, this.roadBind0);
        pass.setPipeline(P.road);
        pass.setBindGroup(1, this.terrainBind1);
        for (const r of this.roads) {
          if (!r) continue;
          pass.setBindGroup(2, r.bind2);
          pass.setVertexBuffer(0, r.vbo);
          pass.draw(r.nVerts);
        }
        pass.end();

        /* Return to real terrain depth. P.roadOwner uses depthCompare=always:
         * the exact accepted-fragment mask, not stale terrain depth, gates the
         * draw. It therefore replaces terrain with unbiased accepted-road
         * depth for later meshes, matching the software owner write. */
        pass = enc.beginRenderPass({
          colorAttachments: [{ view, loadOp: 'load', storeOp: 'store' }],
          depthStencilAttachment: {
            view: this.depthView, depthLoadOp: 'load', depthStoreOp: 'store',
          },
        });
        pass.setBindGroup(0, this.bind0);
        pass.setPipeline(P.roadOwner);
        pass.setBindGroup(1, this.terrainBind1);
        for (const r of this.roads) {
          if (!r) continue;
          pass.setBindGroup(2, r.bind2);
          pass.setVertexBuffer(0, r.vbo);
          pass.draw(r.nVerts);
        }
      }

      pass.setPipeline(P.mesh);
      for (const inst of this.instances) {
        if (inst.draw.hidden) continue;   // sim-hidden objects must not draw
        pass.setVertexBuffer(0, inst.vbo);
        for (const g of inst.groups) {
          // Each group binds its OWN model/material UBO (written once per
          // frame in frame()), so queued draws never share mutable state.
          pass.setBindGroup(1, g.bind1);
          pass.setBindGroup(2, g.tex ? g.tex.bind : this.terrainBind2);
          pass.draw(g.count, 1, g.first, 0);
        }
      }

      if (drawHud && this.hudValid) {
        pass.setPipeline(P.hud);
        pass.draw(3);
      }
      pass.end();
    };

    if (this.ctx) {
      const view = this.ctx.getCurrentTexture().createView();
      drawInto(this.canvasFormat, view, true);
      if (mirror) this._drawMirror(enc, view, mirror);
    }
    // Offscreen: world only (parity vs software depth). Canvas: +HUD.
    if (readback || !this.ctx)
      drawInto('rgba8unorm', this.rtView, !this.ctx && this.hudValid);

    if (readback)
      enc.copyTextureToBuffer(
        { texture: this.rt },
        { buffer: this.readBuf, bytesPerRow: this.paddedBpr, rowsPerImage: this.h },
        { width: this.w, height: this.h, depthOrArrayLayers: 1 });
    d.queue.submit([enc.finish()]);
    if (this.submissionObserver) {
      this.submissionObserver({
        serial: ++this.submissionSerial,
        submitTimeMs: performance.now(),
        vp: Array.from(frameUniforms.subarray(0, 16)),
        worldOrigin: Array.from(this.worldOrigin),
        models: tracedModels,
      });
    }

    if (!readback) return null;
    this._mapping = true;
    try {
      await this.readBuf.mapAsync(GPUMapMode.READ);
      const src = new Uint8Array(this.readBuf.getMappedRange());
      const out = new Uint8Array(this.w * this.h * 4);
      for (let y = 0; y < this.h; y++)
        out.set(src.subarray(y * this.paddedBpr, y * this.paddedBpr + this.w * 4),
                y * this.w * 4);
      this.readBuf.unmap();
      return out;
    } finally {
      this._mapping = false;
    }
  }

  async _readRoadTexture(texture, owner) {
    if (!texture) throw new Error(`GpuScene.${owner}: road diagnostics disabled`);
    if (this._mapping) throw new Error(`GpuScene.${owner}: readback in flight`);
    const enc = this.device.createCommandEncoder();
    enc.copyTextureToBuffer(
      { texture },
      { buffer: this.roadReadBuf, bytesPerRow: this.paddedBpr,
        rowsPerImage: this.h },
      { width: this.w, height: this.h, depthOrArrayLayers: 1 });
    this.device.queue.submit([enc.finish()]);
    this._mapping = true;
    try {
      await this.roadReadBuf.mapAsync(GPUMapMode.READ);
      const src = new Uint8Array(this.roadReadBuf.getMappedRange());
      const out = new Uint32Array(this.w * this.h);
      for (let y = 0; y < this.h; y++) {
        const row = new Uint32Array(src.buffer, src.byteOffset + y * this.paddedBpr,
                                    this.w);
        out.set(row, y * this.w);
      }
      this.roadReadBuf.unmap();
      return out;
    } finally {
      this._mapping = false;
    }
  }

  /* Read the accepted primary-road ownership mask from the most recent
   * frame. Gate-only diagnostics use this to distinguish "GPU rendered" from
   * "GPU rejected every distant road"; production never calls it. */
  async readRoadMask() {
    return this._readRoadTexture(this.roadMask, 'readRoadMask');
  }

  /* Exact projected-road footprint/view-Z from the opted-in diagnostic pass. */
  async readRoadFootprint() {
    return this._readRoadTexture(this.roadFootprint, 'readRoadFootprint');
  }

  /*
   * The rearview mirror pass: the world from the derived rearview CameraView
   * (readRearview's vp), viewport+scissor-confined to the mirror rect. Runs
   * after the primary pass with its own depth clear — the primary colour is
   * already committed, and the viewport keeps mirror pixels out of it. The
   * HUD is deliberately absent: a mirror shows the world, not the dash.
   *
   * Two presentation rules mirror the software path (webmain.c):
   *  - the image is a REFLECTION: the VP is clip-x-flipped (flipVpX), the
   *    GPU twin of the software dst[x] = mirror_fb[mw-1-x]; the inverted
   *    winding is absorbed by the mirror pipelines' flipped frontFace;
   *  - cockpit art always wins: the fs_*_mirror fragments discard wherever
   *    the HUD layer is opaque at that pixel (the software path composites
   *    the HUD over the mirror). When the rearview presents, webmain.c
   *    copies the authored zmiri housing texels into the HUD layer's
   *    transparent pixels, so HUD opacity inside the rect is exactly the
   *    authored glass cutout and the HUD pass paints the housing ring —
   *    see gpu_mirror_probe.mjs for the composition contract.
   */
  _drawMirror(enc, view, mirror) {
    const d = this.device;
    // The mirror's sky dome must sample the authored sky through the REAR
    // CameraView, horizontally reflected to match the flipVpX'd VP — not
    // through the primary camera basis.
    const basis = mirror.cam ? mirrorBasis(mirror.cam, mirror) : this.camBasis;
    d.queue.writeBuffer(this.mirrorUbo, 0,
                        this._frameUniforms(flipVpX(mirror.vp), basis));
    const P = this._pipesFor(this.canvasFormat);
    let pass = enc.beginRenderPass({
      colorAttachments: [{ view, loadOp: 'load', storeOp: 'store' }],
      depthStencilAttachment: {
        view: this.depthView,
        depthClearValue: 1, depthLoadOp: 'clear', depthStoreOp: 'store',
      },
    });
    pass.setViewport(mirror.x, mirror.y, mirror.w, mirror.h, 0, 1);
    pass.setScissorRect(mirror.x, mirror.y, mirror.w, mirror.h);
    pass.setBindGroup(0, this.mirrorBind0);

    pass.setPipeline(P.skyMirror);
    pass.draw(3);

    if (this.terrain) {
      pass.setPipeline(P.terrMirror);
      pass.setBindGroup(1, this.terrainBind1);
      pass.setBindGroup(2, this.terrainBind2);
      pass.setVertexBuffer(0, this.terrain.vbo);
      if (this.terrain.lod) {
        pass.draw(this.terrain.nVerts);
      } else {
        pass.setIndexBuffer(this.terrain.ibo, 'uint32');
        pass.drawIndexed(this.terrain.nIdx);
      }
    }
    if (this.roads) {
      /* At this boundary the mirror depth contains only sky/terrain: meshes
       * are intentionally submitted below, after roads, and cockpit housing
       * is rejected in fs_road_mirror via hud_covers(). Seeding roadDepth with
       * mirrored terrain therefore preserves every pre-road occluder. */
      pass.end();
      pass = enc.beginRenderPass({
        colorAttachments: [{
          view: this.roadTerrainZView, loadOp: 'clear', storeOp: 'store',
          clearValue: { r: 0, g: 0, b: 0, a: 0 },
        }],
        depthStencilAttachment: {
          view: this.roadDepthView,
          depthClearValue: 1, depthLoadOp: 'clear', depthStoreOp: 'store',
        },
      });
      pass.setViewport(mirror.x, mirror.y, mirror.w, mirror.h, 0, 1);
      pass.setScissorRect(mirror.x, mirror.y, mirror.w, mirror.h);
      pass.setBindGroup(0, this.mirrorSeedBind0);
      if (this.terrain) {
        pass.setPipeline(P.roadSeedMirror);
        pass.setBindGroup(1, this.terrainBind1);
        pass.setBindGroup(2, this.terrainBind2);
        pass.setVertexBuffer(0, this.terrain.vbo);
        if (this.terrain.lod) pass.draw(this.terrain.nVerts);
        else {
          pass.setIndexBuffer(this.terrain.ibo, 'uint32');
          pass.drawIndexed(this.terrain.nIdx);
        }
      }
      pass.end();

      pass = enc.beginRenderPass({
        colorAttachments: [
          { view, loadOp: 'load', storeOp: 'store' },
          { view: this.roadMaskView, loadOp: 'clear', storeOp: 'store',
            clearValue: { r: 0, g: 0, b: 0, a: 0 } },
        ],
        depthStencilAttachment: {
          /* The shader owns road-vs-terrain acceptance; isolated hardware
           * depth resolves road-vs-road intersections only. */
          view: this.roadDepthView, depthClearValue: 1,
          depthLoadOp: 'clear', depthStoreOp: 'store',
        },
      });
      pass.setViewport(mirror.x, mirror.y, mirror.w, mirror.h, 0, 1);
      pass.setScissorRect(mirror.x, mirror.y, mirror.w, mirror.h);
      pass.setBindGroup(0, this.mirrorRoadBind0);
      pass.setPipeline(P.roadMirror);
      pass.setBindGroup(1, this.terrainBind1);
      for (const r of this.roads) {
        if (!r) continue;
        pass.setBindGroup(2, r.bind2);
        pass.setVertexBuffer(0, r.vbo);
        pass.draw(r.nVerts);
      }
      pass.end();

      pass = enc.beginRenderPass({
        colorAttachments: [{ view, loadOp: 'load', storeOp: 'store' }],
        depthStencilAttachment: {
          view: this.depthView, depthLoadOp: 'load', depthStoreOp: 'store',
        },
      });
      pass.setViewport(mirror.x, mirror.y, mirror.w, mirror.h, 0, 1);
      pass.setScissorRect(mirror.x, mirror.y, mirror.w, mirror.h);
      pass.setBindGroup(0, this.mirrorBind0);
      pass.setPipeline(P.roadOwnerMirror);
      pass.setBindGroup(1, this.terrainBind1);
      for (const r of this.roads) {
        if (!r) continue;
        pass.setBindGroup(2, r.bind2);
        pass.setVertexBuffer(0, r.vbo);
        pass.draw(r.nVerts);
      }
    }

    pass.setPipeline(P.meshMirror);
    for (const inst of this.instances) {
      if (inst.draw.hidden) continue;
      // A mirror shows the world behind, never the cockpit interior.
      if (inst.draw.kind === 'interior') continue;
      pass.setVertexBuffer(0, inst.vbo);
      for (const g of inst.groups) {
        pass.setBindGroup(1, g.bind1);
        pass.setBindGroup(2, g.tex ? g.tex.bind : this.terrainBind2);
        pass.draw(g.count, 1, g.first, 0);
      }
    }
    pass.end();
  }

  /*
   * Drop every mission-owned resource: instance material UBOs, mesh vertex
   * buffers, cached .m16 face tiles, terrain/road geometry and their skins,
   * and the authored sky texture (back to the gradient placeholder). Called
   * on mission (re)load before the fresh uploads. Frame-level resources
   * (frame/mirror UBOs, HUD planes, pipelines, samplers) survive.
   */
  resetMission() {
    this._clearInstances();
    for (const e of this._texCache.values()) if (e && e.tex) e.tex.destroy();
    this._texCache = new Map();
    if (this.terrain) {
      this.terrain.vbo.destroy();
      if (this.terrain.ibo) this.terrain.ibo.destroy();
      this.terrain = null;
    }
    if (this.terrainTex) { this.terrainTex.destroy(); this.terrainTex = null; }
    this.terrainBind2 = this._bind2(this.white);
    if (this.roads) {
      for (const r of this.roads) if (r && r.vbo) r.vbo.destroy();
      this.roads = null;
    }
    if (this._roadTex) {
      for (const t of this._roadTex) if (t) t.destroy();
      this._roadTex = null;
    }
    this.setSkyTex(null);
  }

  /* Release every GPU resource this scene owns. */
  destroy() {
    this._clearInstances();
    for (const e of this._texCache.values()) if (e && e.tex) e.tex.destroy();
    this._texCache = new Map();
    if (this.terrain) {
      this.terrain.vbo.destroy();
      if (this.terrain.ibo) this.terrain.ibo.destroy();
      this.terrain = null;
    }
    if (this.terrainTex) { this.terrainTex.destroy(); this.terrainTex = null; }
    if (this.roads) {
      for (const r of this.roads) if (r && r.vbo) r.vbo.destroy();
      this.roads = null;
    }
    if (this._roadTex) {
      for (const t of this._roadTex) if (t) t.destroy();
      this._roadTex = null;
    }
    if (this.terrainUbo) { this.terrainUbo.destroy(); this.terrainUbo = null; }
    if (this.skyTex) { this.skyTex.destroy(); this.skyTex = null; }
    this.white.destroy();
    this.rt.destroy();
    this.depth.destroy();
    this.roadDepth.destroy();
    this.roadMask.destroy();
    this.roadFootprint?.destroy();
    this.roadReadBuf.destroy();
    this.roadTerrainZ.destroy();
    this.readBuf.destroy();
    this.frameUbo.destroy();
    this.mirrorUbo.destroy();
    this.hudTex.destroy();
    this.hudMaskTex.destroy();
    this.palTex.destroy();
    this.hudValid = false;
    if (this.ctx) {
      try { this.ctx.unconfigure(); } catch { /* device may be lost */ }
      this.ctx = null;
    }
  }
}

/* ------------------------------------------------------------------ */
/* Coverage masks + IoU (the Tier-2 geometric parity metric)           */
/* ------------------------------------------------------------------ */

/* Software geometry coverage: reciprocal depth is nonzero; sky stays zero. */
export function maskFromDepth(depth, w = FB_W, h = FB_H) {
  const m = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) if (depth[i] !== 0) m[i] = 1;
  return m;
}

export function worldDepthMask(M, w = FB_W, h = FB_H) {
  const ptr = M._web_gpu_world_depth();
  if (!ptr) return null;
  return maskFromDepth(heapI32(M).subarray(ptr >> 2, (ptr >> 2) + w * h), w, h);
}

/* GPU mask: readback alpha > 0 (geometry writes alpha 1, clear is 0). */
export function maskFromRgba(rgba, w = FB_W, h = FB_H) {
  const m = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) if (rgba[i * 4 + 3] > 0) m[i] = 1;
  return m;
}

/* Box-filter RGBA downsample (Phase D supersample present path). */
export function downsampleRgba(src, sw, sh, dw, dh) {
  if (sw === dw && sh === dh) return src;
  const out = new Uint8Array(dw * dh * 4);
  const sx = sw / dw, sy = sh / dh;
  for (let y = 0; y < dh; y++) {
    for (let x = 0; x < dw; x++) {
      let r = 0, g = 0, b = 0, a = 0, n = 0;
      const x0 = Math.floor(x * sx), x1 = Math.floor((x + 1) * sx);
      const y0 = Math.floor(y * sy), y1 = Math.floor((y + 1) * sy);
      for (let yy = y0; yy < y1; yy++)
        for (let xx = x0; xx < x1; xx++) {
          const i = (yy * sw + xx) * 4;
          r += src[i]; g += src[i + 1]; b += src[i + 2]; a += src[i + 3];
          n++;
        }
      const o = (y * dw + x) * 4;
      out[o] = r / n; out[o + 1] = g / n; out[o + 2] = b / n; out[o + 3] = a / n;
    }
  }
  return out;
}

/*
 * M6 promotion ladder, rung 1 (PORT DECISION): compare complete 640x480
 * colour frames, not only geometry masks.  The hardware renderer deliberately
 * filters and shades differently from the indexed software authority, so this
 * is a bounded drift alarm rather than pixel-fidelity proof.  A pixel is an
 * outlier when any RGB channel differs by more than the declared tolerance.
 * Fixed screen bands localise failures; they are diagnostic presentation
 * regions, not semantic segmentation of every road/terrain polygon.
 */
export const FULL_FRAME_CHANNEL_TOLERANCE = 32;
export const FULL_FRAME_MAX_OUTLIER_FRACTION = 0.01;
export const FULL_FRAME_BANDS = Object.freeze([
  { name: 'sky', y0: 0, y1: 160 },
  { name: 'terrain', y0: 160, y1: 320 },
  { name: 'road', y0: 320, y1: 480 },
]);

export function indexedFrameToRgba(indices, palette, w = FB_W, h = FB_H) {
  if (!indices || indices.length < w * h || !palette || palette.length < 768)
    throw new Error('indexedFrameToRgba: short frame or palette');
  const out = new Uint8Array(w * h * 4);
  for (let i = 0; i < w * h; i++) {
    const p = indices[i] * 3, o = i * 4;
    out[o] = palette[p]; out[o + 1] = palette[p + 1];
    out[o + 2] = palette[p + 2]; out[o + 3] = 255;
  }
  return out;
}

function diffRegion(sw, gpu, w, x0, y0, x1, y1, tolerance) {
  const hist = new Uint32Array(256);
  let pixels = 0, outliers = 0, sum = 0, sumSq = 0, max = 0;
  for (let y = y0; y < y1; y++)
    for (let x = x0; x < x1; x++) {
      const o = (y * w + x) * 4;
      const d = Math.max(Math.abs(sw[o] - gpu[o]),
                         Math.abs(sw[o + 1] - gpu[o + 1]),
                         Math.abs(sw[o + 2] - gpu[o + 2]));
      hist[d]++; pixels++; sum += d; sumSq += d * d;
      if (d > tolerance) outliers++;
      if (d > max) max = d;
    }
  const p95At = Math.max(1, Math.ceil(pixels * 0.95));
  let seen = 0, p95 = 0;
  for (; p95 < hist.length; p95++) {
    seen += hist[p95];
    if (seen >= p95At) break;
  }
  return {
    pixels, tolerance, outliers,
    outlierFraction: pixels ? outliers / pixels : 1,
    meanMaxChannelDiff: pixels ? sum / pixels : 255,
    rmsMaxChannelDiff: pixels ? Math.sqrt(sumSq / pixels) : 255,
    p95MaxChannelDiff: p95,
    maxChannelDiff: max,
  };
}

export function fullFrameDiff(sw, gpu, w = FB_W, h = FB_H,
                              tolerance = FULL_FRAME_CHANNEL_TOLERANCE,
                              maxOutlierFraction = FULL_FRAME_MAX_OUTLIER_FRACTION) {
  if (!sw || !gpu || sw.length < w * h * 4 || gpu.length < w * h * 4)
    throw new Error('fullFrameDiff: short RGBA frame');
  const regions = [{ name: 'full', y0: 0, y1: h }, ...FULL_FRAME_BANDS]
    .map(({ name, y0, y1 }) => {
      const m = diffRegion(sw, gpu, w, 0, y0, w, Math.min(y1, h), tolerance);
      return { name, ...m,
        match: m.pixels > 0 && m.outlierFraction <= maxOutlierFraction };
    });
  const worst = regions.slice(1).sort((a, b) =>
    b.outlierFraction - a.outlierFraction)[0];
  return {
    tolerance: { metric: 'max(|ΔR|,|ΔG|,|ΔB|)', channel: tolerance,
      maxOutlierFraction },
    bands: 'fixed screen rows: sky 0-159, terrain 160-319, road 320-479',
    regions,
    worstRegion: worst?.name ?? 'none',
    match: regions.length > 1 && regions.every((r) => r.match),
  };
}

export function fnv1aHex(bytes) {
  let hash = 0x811c9dc5;
  for (let i = 0; i < bytes.length; i++) {
    hash ^= bytes[i];
    hash = Math.imul(hash, 0x01000193) >>> 0;
  }
  return `0x${hash.toString(16).padStart(8, '0')}`;
}

function paintRgbaCanvas(canvas, rgba, w = FB_W, h = FB_H,
                         forceOpaque = false) {
  if (!canvas || !rgba) return;
  const ctx = canvas.getContext('2d');
  if (!ctx) return;
  const image = ctx.createImageData(w, h);
  image.data.set(rgba);
  /* The offscreen WebGPU target deliberately keeps alpha as world-geometry
   * coverage: sky is 0 and the RGB-only HUD pass preserves that alpha. A 2-D
   * evidence canvas interprets those valid RGB pixels as transparency and
   * displays its black backing store, creating a false terrain-shaped HUD
   * occluder. Product WebGPU presents with alphaMode=opaque. Match that
   * presentation only while painting evidence; retain raw alpha for IoU. */
  if (forceOpaque)
    for (let i = 3; i < image.data.length; i += 4) image.data[i] = 255;
  ctx.putImageData(image, 0, 0);
}

/* Box dilation, radius r (separable two-pass max). */
export function dilate(mask, w, h, r) {
  const tmp = new Uint8Array(w * h);
  const out = new Uint8Array(w * h);
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      let v = 0;
      for (let k = -r; k <= r; k++) {
        const xx = x + k;
        if (xx >= 0 && xx < w && mask[y * w + xx]) { v = 1; break; }
      }
      tmp[y * w + x] = v;
    }
  for (let y = 0; y < h; y++)
    for (let x = 0; x < w; x++) {
      let v = 0;
      for (let k = -r; k <= r; k++) {
        const yy = y + k;
        if (yy >= 0 && yy < h && tmp[yy * w + x]) { v = 1; break; }
      }
      out[y * w + x] = v;
    }
  return out;
}

export function maskIou(a, b) {
  let inter = 0, union = 0, an = 0, bn = 0;
  for (let i = 0; i < a.length; i++) {
    const x = a[i] ? 1 : 0, y = b[i] ? 1 : 0;
    an += x; bn += y;
    if (x && y) inter++;
    if (x || y) union++;
  }
  return { iou: union ? inter / union : 1, inter, union, a: an, b: bn };
}

/*
 * Column-fill (silhouette region) of a coverage mask: every pixel at or
 * below the column's topmost covered pixel. Turns a wireframe and a
 * filled render of the same surface into comparable sky/ground regions.
 */
export function columnFill(mask, w, h) {
  const out = new Uint8Array(w * h);
  for (let x = 0; x < w; x++) {
    let y = 0;
    while (y < h && !mask[y * w + x]) y++;
    for (; y < h; y++) out[y * w + x] = 1;
  }
  return out;
}

/*
 * Per-column silhouette delta: for each column the topmost covered
 * pixel in both masks; reports the mean |dy| over columns covered in
 * BOTH, plus the fraction of columns covered in exactly one.
 */
export function silhouetteDelta(a, b, w, h) {
  let sum = 0, both = 0, only = 0;
  for (let x = 0; x < w; x++) {
    let ta = -1, tb = -1;
    for (let y = 0; y < h; y++) {
      if (ta < 0 && a[y * w + x]) ta = y;
      if (tb < 0 && b[y * w + x]) tb = y;
      if (ta >= 0 && tb >= 0) break;
    }
    if (ta >= 0 && tb >= 0) { sum += Math.abs(ta - tb); both++; }
    else if (ta >= 0 || tb >= 0) only++;
  }
  return { meanDy: both ? sum / both : 0, bothCols: both, onlyCols: only,
           colFrac: (both + only) ? only / (both + only) : 0 };
}

/* ------------------------------------------------------------------ */
/* Pure-JS fill rasterizer — node-side stand-in for GPU coverage.      */
/* Same geometry + same camera as the GPU path; filled-triangle union. */
/* DECISION (near clip): triangles with ANY vertex at zs < near are    */
/* dropped (no proper clip). Once software became filled + far LOD,    */
/* that under-cover is large vs worldDepthMask — so node silhouette    */
/* vs this proxy is INFORMATIONAL, not a WebGPU pass/fail. Real GPU    */
/* hardware clips and is gated in the browser probe.                   */
/* ------------------------------------------------------------------ */

function fillTri(mask, w, h, x0, y0, x1, y1, x2, y2) {
  const minX = Math.max(0, Math.floor(Math.min(x0, x1, x2)));
  const maxX = Math.min(w - 1, Math.ceil(Math.max(x0, x1, x2)));
  const minY = Math.max(0, Math.floor(Math.min(y0, y1, y2)));
  const maxY = Math.min(h - 1, Math.ceil(Math.max(y0, y1, y2)));
  if (minX > maxX || minY > maxY) return;
  const d = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
  if (d === 0) return;
  for (let y = minY; y <= maxY; y++)
    for (let x = minX; x <= maxX; x++) {
      const px = x + 0.5, py = y + 0.5;
      const w0 = ((x1 - px) * (y2 - py) - (x2 - px) * (y1 - py)) / d;
      const w1 = ((x2 - px) * (y0 - py) - (x0 - px) * (y2 - py)) / d;
      const w2 = 1 - w0 - w1;
      if (w0 >= 0 && w1 >= 0 && w2 >= 0) mask[y * w + x] = 1;
    }
}

/* Rasterize the exported scene (terrain grid or LOD mesh + draws) to a mask. */
export function rasterizeMask(grid, scene, vp, w = FB_W, h = FB_H, lod = null) {
  const mask = new Uint8Array(w * h);
  const projVert = (x, y, z) => projectPoint(vp, [x, y, z], w, h);
  if (lod && lod.nverts > 0 && lod.verts) {
    const v = lod.verts;
    for (let i = 0; i + 2 < lod.nverts; i += 3) {
      const o0 = i * LOD_VERT_FLOATS, o1 = (i + 1) * LOD_VERT_FLOATS,
            o2 = (i + 2) * LOD_VERT_FLOATS;
      const p = projVert(v[o0], v[o0 + 1], v[o0 + 2]);
      const q = projVert(v[o1], v[o1 + 1], v[o1 + 2]);
      const r = projVert(v[o2], v[o2 + 1], v[o2 + 2]);
      if (p[2] < 5 || q[2] < 5 || r[2] < 5) continue;
      fillTri(mask, w, h, p[0], p[1], q[0], q[1], r[0], r[1]);
    }
  } else if (grid) {
    const { cols, rows, wx0, wz0, step, heights } = grid;
    const scr = new Array(cols * rows);
    for (let j = 0; j < rows; j++)
      for (let i = 0; i < cols; i++)
        scr[j * cols + i] =
          projVert(wx0 + i * step, heights[j * cols + i], wz0 + j * step);
    for (let j = 0; j + 1 < rows; j++)
      for (let i = 0; i + 1 < cols; i++) {
        const a = scr[j * cols + i], b = scr[j * cols + i + 1];
        const c = scr[(j + 1) * cols + i], e = scr[(j + 1) * cols + i + 1];
        for (const [p, q, r] of [[a, c, b], [b, c, e]]) {
          if (p[2] < 5 || q[2] < 5 || r[2] < 5) continue;
          fillTri(mask, w, h, p[0], p[1], q[0], q[1], r[0], r[1]);
        }
      }
  }
  for (const draw of scene.draws) {
    if (draw.hidden) continue;   // match GpuScene.frame / software visibility
    if (!draw.mesh || !draw.mesh.verts || !draw.mesh.tris) continue;
    const m = draw.model;
    const v = draw.mesh.verts;
    const world = new Array(v.length / 3);
    for (let i = 0; i < v.length; i += 3) {
      const x = v[i], y = v[i + 1], z = v[i + 2];
      world[i / 3] = projVert(
        m[0] * x + m[4] * y + m[8] * z + m[12],
        m[1] * x + m[5] * y + m[9] * z + m[13],
        m[2] * x + m[6] * y + m[10] * z + m[14]);
    }
    const t = draw.mesh.tris;
    for (let i = 0; i < t.length; i += 3) {
      const p = world[t[i]], q = world[t[i + 1]], r = world[t[i + 2]];
      if (p[2] < 5 || q[2] < 5 || r[2] < 5) continue;
      fillTri(mask, w, h, p[0], p[1], q[0], q[1], r[0], r[1]);
    }
  }
  return mask;
}

/* ------------------------------------------------------------------ */
/* Shared driver: engine boot + drive settle + export reads. Used by   */
/* the browser probe and the node smoke. `io` supplies asset bytes:    */
/*   io.fetchAsset(rel) -> Uint8Array | null                           */
/* ------------------------------------------------------------------ */

/*
 * bootEngine(io, { mission, vehicle })
 *   mission: VFS path like 'miss8/N01.CBT' or 'miss8/P01.MSN' (default N01)
 *   vehicle: VCF basename default vdrampg2
 * Stages nitro.zfs/zix + the mission file + sibling .TER when present.
 */
export async function bootEngine(io, opts = {}) {
  const mission = opts.mission || 'miss8/N01.CBT';
  const vehicle = opts.vehicle || 'vdrampg2';
  const { default: I76Web } =
    await import(io.moduleUrl ?? new URL('./dist/i76web.mjs', import.meta.url));
  const M = await I76Web();
  M.FS.mkdir('/data');
  const put = (path, buf) => buf && M.FS.writeFile(path, buf);
  put('/data/nitro.zfs', await io.fetchAsset('nitro.zfs'));
  put('/data/nitro.zix', await io.fetchAsset('nitro.zix'));
  const slash = mission.lastIndexOf('/');
  const dir = slash >= 0 ? mission.slice(0, slash) : '';
  const base = slash >= 0 ? mission.slice(slash + 1) : mission;
  if (dir) M.FS.mkdirTree('/data/' + dir);
  put('/data/' + mission, await io.fetchAsset(mission));
  const stem = base.replace(/\.[^.]+$/, '');
  const terRel = (dir ? dir + '/' : '') + stem + '.TER';
  const ter = await io.fetchAsset(terRel);
  if (ter) put('/data/' + terRel, ter);
  // Also try lowercase .ter
  if (!ter) {
    const ter2 = await io.fetchAsset((dir ? dir + '/' : '') + stem + '.ter');
    if (ter2) put('/data/' + (dir ? dir + '/' : '') + stem + '.ter', ter2);
  }
  const n = M._web_init();
  if (n <= 0) return { M, rc: n };
  const rc = M.ccall('web_drive_load', 'number', ['string', 'string'],
                     [mission, vehicle]);
  return { M, rc, mission, vehicle };
}

/* Read the current level palette as [r,g,b] of index 0 (the GPU clear). */
export function readClearColor(M) {
  const lv = M._web_level_palette();
  const base = lv !== 0 ? lv : M._web_palette();
  const pb = heap8(M).subarray(base, base + 3);
  return [pb[0], pb[1], pb[2]];
}

/* The whole 256-entry level palette as raw RGB triples (for the HUD LUT). */
export function readPalette(M) {
  const lv = M._web_level_palette();
  const base = lv !== 0 ? lv : M._web_palette();
  return heap8(M).slice(base, base + 768);
}

/* Complete post-world 2-D layer authored by the software path. The explicit
 * mask preserves opaque palette index 0 in maps/title cards. Null means no
 * completed drive render exists; callers must not reuse a stale upload. */
export function readOverlay(M, w = FB_W, h = FB_H) {
  if (!M._web_gpu_overlay_layer || !M._web_gpu_overlay_mask) return null;
  const indices = M._web_gpu_overlay_layer();
  const mask = M._web_gpu_overlay_mask();
  if (!indices || !mask) return null;
  const bytes = w * h;
  /* Snapshot, do not retain wasm-heap views. Subsequent GPU material loads may
   * grow memory and detach an earlier subarray; a capture-both probe must keep
   * the exact overlay bytes produced beside its software frame. */
  return {
    indices: heap8(M).slice(indices, indices + bytes),
    mask: heap8(M).slice(mask, mask + bytes),
  };
}

/*
 * Sky/haze ramp. Palette indices SKY_RAMP_LO..SKY_RAMP_HI hold the level's own
 * vertical sky gradient — the one systematic difference between every
 * mission's 8-bit and 16-bit palette variants. Measured on N01 it runs
 * 224 = (190,118,48) warm tan at the HORIZON monotonically down to
 * 239 = (31,24,47) dark blue at the ZENITH: a dusk sky.
 *
 * readSkyColor returns the horizon end — the colour distance fog converges on,
 * so fogged geometry melts into the horizon band with no seam.
 * Index 0 (the software background) is near-black here and would turn a dusk
 * desert into midnight, so it is NOT used; a neutral haze stands in when no
 * level palette is loaded.
 */
export const SKY_RAMP_LO = 224;
export const SKY_RAMP_HI = 239;
const SKY_FALLBACK_HORIZON = [150, 143, 125];
const SKY_FALLBACK_ZENITH = [58, 62, 88];

function skyRampEntry(M, index, fallback) {
  const lv = M._web_level_palette();
  if (!lv) return fallback;
  const pb = heap8(M).subarray(lv + index * 3, lv + index * 3 + 3);
  if (!pb[0] && !pb[1] && !pb[2]) return fallback;
  return [pb[0], pb[1], pb[2]];
}

export function readSkyTex(M) {
  if (!M._web_gpu_sky_tex || M._web_gpu_sky_tex() !== 0) return null;
  const w = M._web_gpu_sky_tex_w(), h = M._web_gpu_sky_tex_h();
  const p = M._web_gpu_sky_tex_rgba();
  if (!w || !h || !p) return null;
  return { w, h, rgba: heap8(M).slice(p, p + w * h * 4) };
}

export function readSkyColor(M) {
  return skyRampEntry(M, SKY_RAMP_LO, SKY_FALLBACK_HORIZON);
}

export function readSkyZenith(M) {
  return skyRampEntry(M, SKY_RAMP_HI, SKY_FALLBACK_ZENITH);
}

/*
 * The four terrain used-extent corner world points (y = the grid's own
 * corner heights). Returns [{x,y,z}, ...] or null without terrain.
 */
export function terrainCorners(grid) {
  if (!grid) return null;
  const { cols, rows, wx0, wz0, step, heights } = grid;
  const wx1 = wx0 + (cols - 1) * step, wz1 = wz0 + (rows - 1) * step;
  return [
    { x: wx0, y: heights[0], z: wz0 },
    { x: wx1, y: heights[cols - 1], z: wz0 },
    { x: wx0, y: heights[(rows - 1) * cols], z: wz1 },
    { x: wx1, y: heights[rows * cols - 1], z: wz1 },
  ];
}

/*
 * Corner spot-check: project `corners` through the JS matrix AND the
 * wasm software camera (web_gpu_project), compare pixel positions.
 * Corners inside the near plane are reported clipped, not gated.
 */
export function cornerCheck(M, vp, corners, w = FB_W, h = FB_H) {
  const out = [];
  for (const c of corners) {
    const js = projectPoint(vp, [c.x, c.y, c.z], w, h);
    const raw = readF64(M, M._web_gpu_project(c.x, c.y, c.z), 3);
    const wp = [raw[0] * w / FB_W, raw[1] * h / FB_H, raw[2]];
    const clipped = wp[2] < 5;
    out.push({
      js: [js[0], js[1]], c: [wp[0], wp[1]], zs: wp[2], clipped,
      dx: Math.abs(js[0] - wp[0]), dy: Math.abs(js[1] - wp[1]),
    });
  }
  return out;
}

/* ------------------------------------------------------------------ */
/* Browser driver — gpu_scene_probe.html (self-driving on load)        */
/* ------------------------------------------------------------------ */

/* DECISION (parity metric, 2026-08-10 rebaseline on mac-mini M4):
 * Software is filled (M8). Browser probe compares software worldDepthMask
 * to REAL GPU readback (alpha>0; sky must write alpha 0).
 *
 * Wireframe-era gate used silhouette-region IoU (columnFill) because raw
 * IoU vs sparse wire was useless. Filled-vs-filled raw IoU is meaningful
 * again: mac-mini N01 measured raw~0.92 / dilated~0.92 while silhouette
 * stayed ~0.59 (far LOD / mid-ring tops disagree with the GPU heightfield
 * export even when per-pixel coverage largely matches). Hard bar is now:
 *   dilated raw IoU >= RAW_IOU_THRESHOLD (0.80)
 *   raw IoU         >= RAW_IOU_THRESHOLD (same floor; reported)
 * Silhouette is reported, not hard-gated.
 *
 * Node's pure-JS fill is a PROXY only (near-straddle drops; no WebGPU).
 * Node hard-fails only on PROXY_* collapse floors. */
export const IOU_DILATE_R = 2;
export const RAW_IOU_THRESHOLD = 0.80;          /* real WebGPU filled-vs-filled */
export const SILHOUETTE_IOU_THRESHOLD = 0.85;   /* legacy report only */
export const PROXY_SILHOUETTE_FLOOR = 0.40;     /* node JS stand-in only */
export const PROXY_RAW_FLOOR = 0.15;            /* node JS stand-in only */
export const CORNER_TOL_PX = 2;
