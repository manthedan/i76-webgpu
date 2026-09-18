// Source-candidate closure regression; not browser rendering qualification.
import assert from 'node:assert/strict';
import { readFileSync, existsSync } from 'node:fs';
import * as GPU from '../gpu_scene.mjs';
assert.equal(typeof GPU.GpuScene, 'function');
assert.equal(typeof GPU.readPalette, 'function');
assert.equal(typeof GPU.readOverlay, 'function');
assert(!('run' in GPU), 'private HTTP-asset browser runner must not be exported');
const gpuText = readFileSync(new URL('../gpu_scene.mjs', import.meta.url), 'utf8');
assert(!/\bfetch\s*\(|XMLHttpRequest|\?app=|nodeSmoke/.test(gpuText));
const html = readFileSync(new URL('../index.html', import.meta.url), 'utf8');
for (const [, value] of html.matchAll(/(?:src|href)="([^"]+)"/g)) {
  if (/^(?:https?:|data:|#)/.test(value)) continue;
  assert(existsSync(new URL(value.split('?')[0], new URL('../index.html', import.meta.url))), `missing static reference ${value}`);
}
for (const f of ['assets.js','save.js','shell-art.js','gpu_scene.mjs','dist/stamp.json','dist/i76web.mjs','dist/i76web.wasm'])
  assert(existsSync(new URL('../'+f, import.meta.url)), `missing runtime closure ${f}`);
console.log('Public surface PASS: local static references resolve; private HTTP runner absent.');
