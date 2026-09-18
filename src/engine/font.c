/*
 * font.c — Bitmap font loader
 *
 * Loads .fnt files from the VFS. See font.h for format docs.
 */

#include "font.h"
#include "vfs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FNT_MAGIC 0x00002e31u

Font *font_load(const char *path)
{
    size_t   size;
    /* probe first: fonts are optional (HUD text degrades gracefully) and
     * vfs_read_file logs every miss — callers report absence themselves */
    uint8_t *data = vfs_exists(path) ? vfs_read_file(path, &size) : NULL;
    if (!data) return NULL;

    if (size < 16) {
        fprintf(stderr, "[font] Too small: %s\n", path);
        vfs_free(data); return NULL;
    }

    uint32_t magic, count, height, transparent;
    memcpy(&magic,       data +  0, 4);
    memcpy(&count,       data +  4, 4);
    memcpy(&height,      data +  8, 4);
    memcpy(&transparent, data + 12, 4);

    if (magic != FNT_MAGIC) {
        fprintf(stderr, "[font] Bad magic 0x%08x: %s\n", magic, path);
        vfs_free(data); return NULL;
    }
    if (count == 0 || height == 0) {
        fprintf(stderr, "[font] Invalid count/height: %s\n", path);
        vfs_free(data); return NULL;
    }
    if (size < 16 + count * 4) {
        fprintf(stderr, "[font] Offset table truncated: %s\n", path);
        vfs_free(data); return NULL;
    }

    Font *f = malloc(sizeof(Font));
    if (!f) { vfs_free(data); return NULL; }

    f->count       = count;
    f->height      = height;
    f->transparent = (uint8_t)(transparent & 0xff);
    f->widths      = malloc(count * sizeof(uint32_t));
    f->pixels      = malloc(count * sizeof(uint8_t *));

    if (!f->widths || !f->pixels) {
        free(f->widths); free(f->pixels); free(f);
        vfs_free(data); return NULL;
    }

    for (uint32_t i = 0; i < count; i++) {
        uint32_t off;
        memcpy(&off, data + 16 + i * 4, 4);

        if (off + 4 > size) {
            f->widths[i]  = 0;
            f->pixels[i]  = NULL;
            continue;
        }

        uint32_t w;
        memcpy(&w, data + off, 4);
        f->widths[i] = w;

        size_t nbytes = (size_t)w * height;
        if (nbytes == 0 || off + 4 + nbytes > size) {
            f->pixels[i] = NULL;
            continue;
        }

        f->pixels[i] = malloc(nbytes);
        if (f->pixels[i])
            memcpy(f->pixels[i], data + off + 4, nbytes);
    }

    vfs_free(data);
    return f;
}

void font_free(Font *f)
{
    if (!f) return;
    for (uint32_t i = 0; i < f->count; i++)
        free(f->pixels[i]);
    free(f->pixels);
    free(f->widths);
    free(f);
}

int font_text_width(const Font *f, const char *text)
{
    int w = 0;
    for (const char *p = text; *p; p++) {
        unsigned idx = (unsigned char)*p;
        if (idx < f->count)
            w += (int)f->widths[idx];
    }
    return w;
}

void font_draw(const Font *f, const char *text,
               uint8_t *pixels, int stride, int buf_h,
               int x, int y, uint8_t color)
{
    int cx = x;
    for (const char *p = text; *p; p++) {
        unsigned idx = (unsigned char)*p;
        if (idx >= f->count) continue;

        uint32_t gw = f->widths[idx];
        const uint8_t *src = f->pixels[idx];

        if (src) {
            for (uint32_t row = 0; row < f->height; row++) {
                int py = y + (int)row;
                if (py < 0 || py >= buf_h) continue;
                for (uint32_t col = 0; col < gw; col++) {
                    int px = cx + (int)col;
                    if (px < 0 || px >= stride) continue;
                    uint8_t v = src[row * gw + col];
                    if (v != f->transparent)
                        pixels[py * stride + px] = color;
                }
            }
        }

        cx += (int)gw;
    }
}
