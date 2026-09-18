// shell_art_test.mjs — synthetic format checks for database.tvf shell art.
// Purchaser assets never enter the repository; this pins the parser's bounds,
// SHP token semantics, palette handoff, and vehicle-table mapping with a tiny
// generated database carrying the same Nitro entry/count contract.
import { open } from '../shell-art.js';

let failures = 0;
const check = (name, cond, detail = '') => {
  console.log(`${cond ? 'ok  ' : 'FAIL'} ${name}${detail ? ' — ' + detail : ''}`);
  if (!cond) failures++;
};
const wr16 = (b, o, n) => { b[o] = n & 255; b[o + 1] = (n >>> 8) & 255; };
const wr32 = (b, o, n) => {
  b[o] = n & 255; b[o + 1] = (n >>> 8) & 255;
  b[o + 2] = (n >>> 16) & 255; b[o + 3] = (n >>> 24) & 255;
};

function shp(count) {
  // Every shape is a 2x1 image: colour 7, one transparent skip, EOL.
  const header = 8 + count * 8, stride = 30;
  const b = new Uint8Array(header + count * stride);
  b.set([49, 46, 49, 48]); wr32(b, 4, count);
  for (let i = 0; i < count; i++) {
    const o = header + i * stride;
    wr32(b, 8 + i * 8, o);
    wr16(b, o, 0); wr16(b, o + 2, 1);
    b.set([2, 7, 1, 1, 0, 0], o + 24);
  }
  return b;
}

function literalEfa(payload) {
  // EFA control 0xff copies eight literal bytes; pad to an 8-byte boundary.
  const padded = new Uint8Array(Math.ceil(payload.length / 8) * 8);
  padded.set(payload);
  const out = new Uint8Array(4 + padded.length + padded.length / 8);
  wr32(out, 0, padded.length);
  let s = 0, d = 4;
  while (s < padded.length) { out[d++] = 0xff; out.set(padded.subarray(s, s + 8), d); d += 8; s += 8; }
  return out;
}

function nitcar() {
  const count = 37, stride = 28;
  const b = new Uint8Array(4 + count * stride); wr32(b, 0, count);
  const models = Array.from({ length: count }, (_, i) => `model${i}`);
  models[10] = 'vcvan';
  models[12] = 'vdrampg';
  models[30] = 'vsvan';
  models[31] = 'vavikea';
  models[34] = 'vxamblc';
  models[35] = 'vxbus';
  const portraits = Array.from({ length: count }, (_, i) => i);
  /* The authored key is not the record position: Service Van deliberately
   * reuses the van art, and the late models therefore do not equal their
   * table rows. This is the exact defect shape from H-UAT-040. */
  portraits[30] = 10;
  portraits[31] = 30;
  portraits[34] = 33;
  portraits[35] = 34;
  for (let i = 0; i < count; i++) {
    const at = 4 + i * stride;
    wr32(b, at, portraits[i]); wr32(b, at + 4, 1); wr32(b, at + 8, 1);
    b.set(Buffer.from(models[i], 'ascii'), at + 12);
  }
  return b;
}

function database() {
  const pcx = new Uint8Array(776);
  pcx[7] = 0x0c; // decoder checks the marker at output[-769]
  pcx[8 + 7 * 3] = 12; pcx[8 + 7 * 3 + 1] = 34; pcx[8 + 7 * 3 + 2] = 56;
  const entries = Array.from({ length: 76 }, () => new Uint8Array([0]));
  entries[1] = literalEfa(pcx);
  entries[53] = shp(74);
  entries[54] = shp(37);
  const head = 4 + entries.length * 4;
  const total = head + entries.reduce((n, e) => n + e.length, 0);
  const db = new Uint8Array(total); wr32(db, 0, entries.length);
  let at = head;
  entries.forEach((e, i) => { wr32(db, 4 + i * 4, at); db.set(e, at); at += e.length; });
  return db;
}

const art = open(database(), nitcar());
check('authored portrait key, not nitcar.def record position, selects art',
      art?.vehicleIndex('vsvan2') === 10 &&
      art?.vehicleIndex('vavikea1.vcf') === 30 &&
      art?.vehicleIndex('vxamblc1') === 33 && art?.vehicleIndex('vxbus2') === 34);
check('VCF variants share their authored model portrait',
      art?.vehicleIndex('vdrampg2') === 12 && art?.vehicleIndex('vdrampg1') === 12);
check('unknown/custom vehicles do not borrow another portrait',
      art?.vehicleIndex('custom1') === -1);
check('synthetic Nitro database and car table open', !!art);
const shape = art?.side.shape(24);
check('SHP run + skip decode preserves size and transparency',
      shape?.width === 2 && shape?.height === 1 &&
      shape.rgba[0] === 12 && shape.rgba[1] === 34 && shape.rgba[2] === 56 &&
      shape.rgba[3] === 255 && shape.rgba[7] === 0,
      shape ? Array.from(shape.rgba).join(',') : 'no shape');
const malformed = database(); wr32(malformed, 4 + 54 * 4, malformed.length + 1);
check('out-of-range/non-monotonic database offsets fail closed',
      open(malformed, nitcar()) === null);
const badKey = nitcar(); wr32(badKey, 4, 37);
check('out-of-range authored portrait keys fail closed',
      open(database(), badKey) === null);
const truncatedTable = nitcar().subarray(0, 1039);
check('truncated authored vehicle tables fail closed',
      open(database(), truncatedTable) === null);

process.exit(failures ? 1 : 0);
