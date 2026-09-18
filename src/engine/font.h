#ifndef FONT_H
#define FONT_H

/*
 * font.h — Bitmap font loader and renderer
 *
 * Loads the game's .fnt format (base6x7.fnt, base6x74.fnt, base6x76.fnt).
 *
 * File format:
 *   [0x00]  uint32  magic       = 0x00002e31
 *   [0x04]  uint32  glyph_count = 128
 *   [0x08]  uint32  height      — glyph height in pixels
 *   [0x0c]  uint32  transparent — palette index meaning "skip this pixel"
 *   [0x10]  uint32  offsets[glyph_count] — file offsets to each glyph
 *
 *   Each glyph:
 *     uint32  width
 *     uint8   pixels[height × width]  — transparent or foreground index
 *
 * font_draw writes palette indices into an 8-bit indexed pixel buffer.
 * The caller decides what palette index to use for the foreground colour.
 */

#include <stdint.h>

typedef struct {
    uint32_t  count;
    uint32_t  height;
    uint8_t   transparent;  /* pixel value to skip */
    uint32_t *widths;        /* [count] */
    uint8_t **pixels;        /* [count][height * width] */
} Font;

/*
 * Load a .fnt file via the VFS. Returns NULL on failure.
 * Caller must call font_free() when done.
 */
Font *font_load(const char *path);
void  font_free(Font *f);

/*
 * Measure the pixel width of a string in this font.
 */
int font_text_width(const Font *f, const char *text);

/*
 * Draw text into an 8-bit indexed pixel buffer.
 *   pixels  — destination buffer (indexed, one byte per pixel)
 *   stride  — row stride in bytes (usually buffer width)
 *   x, y    — top-left origin
 *   color   — palette index to use for foreground pixels
 *
 * Clips at the buffer boundaries defined by (stride, buf_h).
 */
void font_draw(const Font *f, const char *text,
               uint8_t *pixels, int stride, int buf_h,
               int x, int y, uint8_t color);

#endif /* FONT_H */
