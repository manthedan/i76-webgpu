// Asset-free preference policy; browser presentation is qualified separately.
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import * as Save from '../save.js';

assert.equal(Save.profile().prefs.renderer, 'gpu', 'fresh profiles prefer WebGPU');
for (const requested of [null, undefined, '', 'bogus', 'GPU', 0, {}]) {
  for (const saved of [null, undefined, '', 'bogus', 'GPU', 0, {}])
    assert.equal(Save.selectRenderer(requested, saved), 'gpu');
  assert.equal(Save.selectRenderer(requested, 'sw'), 'sw');
  assert.equal(Save.selectRenderer(requested, 'gpu'), 'gpu');
}
for (const saved of [null, 'sw', 'gpu', 'bogus']) {
  assert.equal(Save.selectRenderer('sw', saved), 'sw');
  assert.equal(Save.selectRenderer('gpu', saved), 'gpu');
}

// Storage-disabled startup still gets a usable default. Legacy explicit
// preferences migrate without being overwritten just because GPU is default.
assert.equal((await Save.load()).prefs.renderer, 'gpu');
globalThis.localStorage = { getItem: (key) => key === 'i76.renderer' ? 'sw' : null };
assert.equal((await Save.load()).prefs.renderer, 'sw');
assert.equal(Save.selectRenderer(null, Save.profile().prefs.renderer), 'sw');
assert.equal(Save.selectRenderer('gpu', Save.profile().prefs.renderer), 'gpu');
delete globalThis.localStorage;

const html = readFileSync(new URL('../index.html', import.meta.url), 'utf8');
assert.match(html, /let renderer = Save\.selectRenderer\(query\.get\('renderer'\), progress\.prefs\.renderer\);/,
             'ordinary page selection must not be gated on developer mode');
assert.match(html, /const gpuResParam = DEV \? query\.get\('gpures'\) : null;/);
assert.match(html, /const gpuResolution = gpuResParam === 'fidelity' \? 'fidelity' : 'native';/);
console.log('Renderer choice PASS: fresh GPU, saved choices, explicit overrides, invalid values and disabled storage.');
