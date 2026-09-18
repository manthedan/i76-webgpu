// Asset-free initialization/failure smoke. Not decoder or mission acceptance.
import assert from 'node:assert/strict';
import I76Web from '../dist/i76web.mjs';
const M = await I76Web();
assert(M.HEAPU8 instanceof Uint8Array);
assert(M.HEAPU8.byteLength > 0);
assert.equal(typeof M._web_drive_step, 'function');
assert.equal(typeof M._web_movie_open, 'function');
assert.equal(typeof M._web_key_event, 'function');
const id = M.ccall('web_build_id', 'string', [], []);
assert.equal(typeof id, 'string');
assert(id.length > 0);
M.FS.mkdirTree('/data');
assert.equal(M._web_init(), -1, 'missing archives must not initialize successfully');
assert.equal(M._web_mesh_count(), 0);
assert.equal(M.ccall('web_movie_open', 'number', ['string'], ['']), -1);
assert(M.ccall('web_drive_load', 'number', ['string', 'string'], ['miss8/missing.msn', 'missing.vcf']) < 0);
console.log(`Asset-free Wasm smoke PASS (build ${id}); no playable mission or valid movie tested.`);
