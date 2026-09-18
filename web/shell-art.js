// shell-art.js — authored vehicle sketches from Nitro's loose database.tvf.
//
// The original Scenario/Driver and Chassis Configuration forms do not source
// their vehicle portraits from nitro.zfs. database.tvf is an offset table:
// entry 53 is the 37 front/side SHP pairs and entry 54 the 37 square chassis
// drawings. Loose nitcar.def is the authored model-to-picture table: each
// 28-byte record starts with the SHP index and ends with its 16-byte VCF model
// stem. SHP pixels use the palette embedded in the shell's EFA/PCX form
// backgrounds. This decoder is deliberately browser-side: the art remains
// purchaser-supplied loose data and is copied only into the form canvases.

const NITRO_ENTRY_COUNT = 76;
const NITRO_PALETTE_EFA = 1; // Chassis Configuration form; shared shell palette
const NITRO_CARS_SIDE = 53;
const NITRO_CARS_3D = 54;
const NITRO_CAR_RECORDS = 37;
const NITRO_CAR_RECORD_SIZE = 28;

const u16 = (b, o) => b[o] | (b[o + 1] << 8);
const i16 = (b, o) => (u16(b, o) << 16) >> 16;
const u32 = (b, o) => (b[o] | (b[o + 1] << 8) |
                       (b[o + 2] << 16) | (b[o + 3] << 24)) >>> 0;

function sliceEntry(db, offsets, i) {
  if (i < 0 || i >= offsets.length) return null;
  const begin = offsets[i];
  const end = i + 1 < offsets.length ? offsets[i + 1] : db.length;
  return begin <= end && end <= db.length ? db.subarray(begin, end) : null;
}

// EFA is the shell's small LZ wrapper around a complete 8-bit PCX.
function decodeEfa(src) {
  if (!src || src.length < 8) return null;
  const size = u32(src, 0);
  if (!size || size > 4 * 1024 * 1024) return null;
  const out = new Uint8Array(size);
  let input = 4, output = 0, failed = false;

  const copy = (count) => {
    if (input + count > src.length || output + count > out.length) {
      failed = true; return;
    }
    out.set(src.subarray(input, input + count), output);
    input += count; output += count;
  };
  const codeword = () => {
    if (input + 2 > src.length) { failed = true; return; }
    const code = u16(src, input); input += 2;
    const count = (code >>> 12) + 3;
    const distance = code & 0x0fff;
    for (let i = 0; i < count; i++) {
      if (output >= out.length) { failed = true; return; }
      let page = 0;
      if (output > 4096) {
        page = output & 0x7f000;
        if (output - page < distance) page -= 4096;
      }
      const from = page + distance + i;
      out[output++] = from >= 0 && from < output ? out[from] : 0;
    }
  };
  const controls = [
    'dddd', 'cddd', 'dcdd', 'ccdd', 'ddcd', 'cdcd', 'dccd', 'cccd',
    'dddc', 'cddc', 'dcdc', 'ccdc', 'ddcc', 'cdcc', 'dccc', 'cccc',
  ];
  while (!failed && input < src.length && output < out.length) {
    const control = src[input++];
    for (const nibble of [control & 15, control >>> 4]) {
      for (const op of controls[nibble]) {
        if (failed) break;
        if (op === 'c') copy(1); else codeword();
      }
    }
  }
  /* The original stream may end in the middle of the final eight-operation
   * control group once the declared output size is satisfied. Bytes after
   * that boundary are irrelevant; require the complete declared output, not
   * another input token that has nowhere to write. */
  return output === out.length ? out : null;
}

function pcxPalette(pcx) {
  if (!pcx || pcx.length < 769 || pcx[pcx.length - 769] !== 0x0c) return null;
  return pcx.subarray(pcx.length - 768);
}

function openShp(bytes, palette) {
  if (!bytes || bytes.length < 8 || String.fromCharCode(...bytes.subarray(0, 4)) !== '1.10')
    return null;
  const count = u32(bytes, 4);
  if (!count || count > 1000 || 8 + count * 8 > bytes.length) return null;

  return {
    count,
    shape(index) {
      if (index < 0 || index >= count) return null;
      const start = u32(bytes, 8 + index * 8);
      if (start + 24 > bytes.length) return null;
      const height = i16(bytes, start) + 1;
      const width = i16(bytes, start + 2) + 1;
      if (width <= 0 || height <= 0 || width * height > 1024 * 1024) return null;

      const rgba = new Uint8ClampedArray(width * height * 4);
      let input = start + 24, line = height, pixel = 0, failed = false;
      const paint = (colour) => {
        if (pixel < 0 || pixel >= width * height) { failed = true; return; }
        const d = pixel++ * 4, s = colour * 3;
        rgba[d] = palette[s]; rgba[d + 1] = palette[s + 1];
        rgba[d + 2] = palette[s + 2]; rgba[d + 3] = 255;
      };
      while (!failed && line > 0) {
        if (input >= bytes.length) { failed = true; break; }
        const token = bytes[input++];
        if (token === 0) {
          line--;
          pixel = (height - line) * width;
        } else if (token === 1) {
          if (input >= bytes.length) { failed = true; break; }
          pixel += bytes[input++];
          if (pixel > width * height) failed = true;
        } else if ((token & 1) === 0) {
          if (input >= bytes.length) { failed = true; break; }
          const count = token >>> 1, colour = bytes[input++];
          for (let i = 0; i < count && !failed; i++) paint(colour);
        } else {
          const count = (token - 1) >>> 1;
          if (input + count > bytes.length) { failed = true; break; }
          for (let i = 0; i < count && !failed; i++) paint(bytes[input++]);
        }
      }
      return failed || line !== 0 ? null : { width, height, rgba };
    },
  };
}

function openVehicleTable(bytes, portraitCount) {
  const table = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes || 0);
  if (table.length !== 4 + NITRO_CAR_RECORDS * NITRO_CAR_RECORD_SIZE ||
      u32(table, 0) !== NITRO_CAR_RECORDS) return null;

  const portraits = new Map();
  for (let i = 0; i < NITRO_CAR_RECORDS; i++) {
    const at = 4 + i * NITRO_CAR_RECORD_SIZE;
    const portrait = u32(table, at);
    let end = at + 12;
    while (end < at + NITRO_CAR_RECORD_SIZE && table[end]) end++;
    const modelBytes = table.subarray(at + 12, end);
    if (!modelBytes.length || portrait >= portraitCount) return null;
    let model = '';
    for (const c of modelBytes) {
      if (!((c >= 97 && c <= 122) || (c >= 48 && c <= 57) || c === 95))
        return null;
      model += String.fromCharCode(c);
    }
    if (portraits.has(model)) return null;
    portraits.set(model, portrait);
  }

  return (vcf) => {
    const model = String(vcf || '').toLowerCase()
      .replace(/\.vcf$/, '').replace(/\d+$/, '');
    return portraits.get(model) ?? -1;
  };
}

export function open(bytes, vehicleBytes) {
  const db = bytes instanceof Uint8Array ? bytes : new Uint8Array(bytes || 0);
  if (db.length < 8) return null;
  const count = u32(db, 0);
  if (count !== NITRO_ENTRY_COUNT || 4 + count * 4 > db.length) return null;
  const offsets = [];
  for (let i = 0; i < count; i++) offsets.push(u32(db, 4 + i * 4));
  if (offsets.some((x, i) => x > db.length || (i && x < offsets[i - 1]))) return null;

  const palette = pcxPalette(decodeEfa(sliceEntry(db, offsets, NITRO_PALETTE_EFA)));
  if (!palette) return null;
  const side = openShp(sliceEntry(db, offsets, NITRO_CARS_SIDE), palette);
  const chassis = openShp(sliceEntry(db, offsets, NITRO_CARS_3D), palette);
  if (!side || side.count !== 74 || !chassis || chassis.count !== 37) return null;
  const vehicleIndex = openVehicleTable(vehicleBytes, chassis.count);
  if (!vehicleIndex) return null;
  return { side, chassis, vehicleIndex };
}
