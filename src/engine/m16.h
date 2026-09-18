#ifndef M16_H
#define M16_H

/*
 * m16.h — .M16 hardware-texture decode (M6 Tier-2)
 *
 * The .m16 format is the hardware-renderer (-glide) texture: every *6.pak
 * set (e.g. a4tank16.pak, tp18m6.pak) plus loose zone tiles (zhr*, zsr* sets).
 * Layout (cracked 2026-07-09, docs in reference/i76-everywhere; vehicle
 * tiles verified lossless round-trip):
 *
 *   u32   width
 *   u32   height | flags << 24         (flags 0x80 observed; meaning open)
 *   u8    indices[width*height]        (row-major, top-down)
 *   u32   paletteCount                 (max 255 — 0xFF is reserved)
 *   u16   palette[paletteCount]        (RGB565 little-endian, per-tile local)
 *
 * Decode semantics mirror tools/i76img.py decode_m16 byte-exactly (the
 * reference implementation, cross-verified by tools/m16_probe.c):
 *   index == 0xFF (and >= paletteCount)      -> transparent (0,0,0,0)
 *   index >= paletteCount (and != 0xFF)      -> opaque black (palette[0]=0
 *                                               fallback in the python)
 *   else -> RGB565 expanded per channel v*255/31 (v*255/63 for green),
 *           alpha 255.
 *
 * Pure C11, no platform deps. Buffers returned are heap-owned by the
 * caller.
 */
#include <stddef.h>
#include <stdint.h>

typedef struct {
    uint32_t w, h;          /* pixel dimensions                          */
    uint32_t flags;         /* header flags byte (h_raw >> 24)           */
    uint32_t pal_count;     /* palette entries (<= 255 in real data)     */
    uint8_t *indices;       /* w*h palette indices (heap)                */
    uint16_t *pal565;       /* pal_count RGB565 LE values (heap)         */
} M16Image;

/*
 * m16_decode(data, size)
 *   Parse and validate an .m16 image. Returns a heap M16Image (free with
 *   m16_image_free) or NULL on malformed/truncated input. Zero-dimension
 *   headers are rejected (no real data has them).
 */
M16Image *m16_decode(const void *data, size_t size);

void m16_image_free(M16Image *img);

/*
 * m16_to_rgba(img)
 *   Expand to RGBA8 (w*h*4 bytes, row-major, top-down) with the exact
 *   i76img.py decode_m16 semantics above. Returns heap buffer (free()
 *   it) or NULL on bad arguments.
 */
uint8_t *m16_to_rgba(const M16Image *img);

/*
 * m16_decode_rgba(data, size, w, h, flags)
 *   One-shot convenience: decode + expand. Returns heap RGBA8 (free())
 *   or NULL on malformed input. Out pointers may be NULL.
 */
uint8_t *m16_decode_rgba(const void *data, size_t size,
                         uint32_t *w, uint32_t *h, uint32_t *flags);

/*
 * m16_pix_tile_range(pix_text, index, name_out, off, len)
 *   Parse a .pix manifest (ASCII: count, then "NAME off len" triples) and
 *   return the byte range of tile `index` inside the sibling .pak.
 *   Shared by the native probe and the web terrain-texture resolver.
 *   Returns 0 on success, -1 when the manifest is short or `index` is out
 *   of range.
 */
int m16_pix_tile_range(const char *pix_text, int index, char name_out[64],
                       uint32_t *off, uint32_t *len);

#endif /* M16_H */
