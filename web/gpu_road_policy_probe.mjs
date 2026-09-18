#!/usr/bin/env node
/* Static/recording pin for the WebGPU mirror of the RDEF painter-order
 * contract. This is not adapter-backed evidence: gpu_scene_probe.html owns
 * real pipeline/render/readback validation and may honestly SKIP when no
 * adapter is available. */
const { GpuScene, ROAD_DEPTH_POLICY, ROAD_OWNER_DEPTH_POLICY,
        ROAD_TERRAIN_TOLERANCE_M, ROAD_DEPTH_PULL_M, roadDepthForViewZ,
        SCENE_WGSL } = await import('./gpu_scene.mjs');
let failures = 0;
const check = (name, cond, detail = '') => {
  if (cond) console.log(`ok   ${name}${detail ? ' — ' + detail : ''}`);
  else { console.error(`FAIL ${name}${detail ? ' — ' + detail : ''}`); failures++; }
};

check('road-only depth writes only to dedicated road attachment',
      ROAD_DEPTH_POLICY.depthWriteEnabled === true);
check('unbiased owner pass writes real accepted road depth',
      ROAD_OWNER_DEPTH_POLICY.depthWriteEnabled === true &&
      ROAD_OWNER_DEPTH_POLICY.depthCompare === 'always');
check('road-only depth keeps the first owner on quantized ties',
      ROAD_DEPTH_POLICY.depthCompare === 'less');
check('road policy avoids adapter-dependent raster bias',
      ROAD_DEPTH_POLICY.depthBias === 0 &&
      ROAD_DEPTH_POLICY.depthBiasSlopeScale === 0);
check('continuous shader tolerance uses an unbiased road-only depth resolver',
      ROAD_TERRAIN_TOLERANCE_M === 2.25 && ROAD_DEPTH_PULL_M === 0.0 &&
      SCENE_WGSL.includes('road_z <= terrain_z + 2.25') &&
      SCENE_WGSL.includes('beta / max(zs, near)'));

const built = [];
const stub = Object.create(GpuScene.prototype);
stub._pipes = new Map();
stub._layFull = 'full'; stub._laySeed = 'seed'; stub._layRoad = 'road'; stub._layFlat = 'flat';
stub._terrBufs = 'terrain'; stub._meshBufs2 = 'mesh';
stub._depthStencil = { depthWriteEnabled: true, depthCompare: 'less' };
stub._depthStencilMesh = { depthWriteEnabled: true, depthCompare: 'less-equal' };
stub._depthStencilRoad = ROAD_DEPTH_POLICY;
stub.terrFront = 'cw'; stub.meshFront = 'ccw';
stub._mk = (layout, vs, fs, format, buffers, cull, depth, front,
            writeMask = 0xf, extraTargets = []) => {
  const p = { layout, vs, fs, format, buffers, cull, depth, front,
              writeMask, extraTargets };
  built.push(p); return p;
};
const pipes = GpuScene.prototype._pipesFor.call(stub, 'rgba8unorm');
check('primary road pipeline consumes exported road policy',
      pipes.road.depth === ROAD_DEPTH_POLICY && pipes.road.layout === 'road');
check('mirror road pipeline consumes identical road policy',
      pipes.roadMirror.depth === ROAD_DEPTH_POLICY);
check('primary and mirror roads use bounded-depth fragment shaders',
      pipes.road.fs === 'fs_road' && pipes.roadMirror.fs === 'fs_road_mirror');
check('terrain still writes strict ordinary depth',
      pipes.terr.depth.depthWriteEnabled === true &&
      pipes.terr.depth.depthCompare === 'less');
check('later world meshes preserve exact road painter ties',
      pipes.mesh.depth.depthWriteEnabled === true &&
      pipes.mesh.depth.depthCompare === 'less-equal' &&
      pipes.meshMirror.depth.depthCompare === 'less-equal');
check('dedicated road acceptance texture is seeded with linear terrain view-Z',
      pipes.roadSeed.format === 'r32float' && pipes.roadSeed.layout === 'seed' &&
      pipes.roadSeedMirror.format === 'r32float');
check('road colour pass records an explicit accepted-fragment mask',
      pipes.road.extraTargets[0]?.format === 'r32uint' &&
      pipes.roadMirror.extraTargets[0]?.format === 'r32uint');
check('road owner passes write depth without touching colour',
      pipes.roadOwner.writeMask === 0 &&
      pipes.roadOwner.depth === ROAD_OWNER_DEPTH_POLICY &&
      pipes.roadOwnerMirror.writeMask === 0 &&
      pipes.roadOwner.fs === 'fs_road_owner' &&
      pipes.roadOwnerMirror.fs === 'fs_road_owner_mirror');

/* Truth table for WebGPU's compare direction: smaller is nearer. The port's
 * continuous 2.25 m view-Z tolerance absorbs coarse-LOD drift; a road behind
 * meaningfully nearer relief still fails. Unbiased depth lives in a dedicated
 * road-only attachment and preserves nearest-road ordering after the shader
 * has made the terrain decision. */
const passes = (roadZ, terrainZ) =>
  roadZ <= terrainZ + ROAD_TERRAIN_TOLERANCE_M;
check('equal-depth road paints over terrain', passes(100, 100));
check('sub-tolerance LOD drift paints', passes(102.2, 100));
check('continuous tolerance includes its exact bound', passes(102.25, 100));
check('beyond-tolerance relief occludes road', !passes(102.26, 100));
check('meaningfully nearer hill occludes road', !passes(110, 100));
check('representable-distance exact ties remain visible', passes(800, 800));
check('isolated unbiased depth preserves nearest-road ordering',
      roadDepthForViewZ(2.2) < roadDepthForViewZ(2.3) &&
      roadDepthForViewZ(100) < roadDepthForViewZ(101));
check('continuous policy is translation-invariant at distance',
      passes(2000, 2000) && passes(2002.25, 2000) && !passes(2002.35, 2000));

if (failures) { console.error(`RESULT: FAIL (${failures})`); process.exit(1); }
console.log('RESULT: PASS (static WebGPU road-state contract; adapter-backed gpu_scene probe remains required)');
