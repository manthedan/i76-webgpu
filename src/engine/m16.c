/*
 * m16.c — .M16 hardware-texture decode (see m16.h)
 *
 * Byte-exact port of tools/i76img.py decode_m16. The python is the
 * reference: tools/m16_probe.c decodes real tiles through this module and
 * the output is compared byte-for-byte against the python's RGBA on the
 * same bytes.
 */
#include "m16.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

M16Image *m16_decode(const void *data, size_t size)
{
    const uint8_t *d = data;
    if (!d || size < 12) return NULL;

    uint32_t w     = rd_u32(d);
    uint32_t h_raw = rd_u32(d + 4);
    uint32_t h     = h_raw & 0xFFFFFFu;
    uint32_t flags = h_raw >> 24;
    if (w == 0 || h == 0) return NULL;

    uint64_t npix = (uint64_t)w * (uint64_t)h;
    /* header + indices + count field must fit before the palette read */
    if (npix > (64u << 20)) return NULL;             /* sanity: 16 Mpx   */
    if (size < 12 + (size_t)npix) return NULL;

    uint32_t count = rd_u32(d + 8 + (size_t)npix);
    if (size < 12 + (size_t)npix + (uint64_t)count * 2) return NULL;

    M16Image *img = calloc(1, sizeof *img);
    if (!img) return NULL;
    img->indices = malloc((size_t)npix);
    img->pal565  = malloc(count ? (size_t)count * 2 : 1);
    if (!img->indices || !img->pal565) {
        m16_image_free(img);
        return NULL;
    }
    img->w = w;
    img->h = h;
    img->flags = flags;
    img->pal_count = count;
    memcpy(img->indices, d + 8, (size_t)npix);
    const uint8_t *p16 = d + 12 + (size_t)npix;
    for (uint32_t i = 0; i < count; i++)
        img->pal565[i] = (uint16_t)(p16[i * 2] | ((uint16_t)p16[i * 2 + 1] << 8));
    return img;
}

void m16_image_free(M16Image *img)
{
    if (!img) return;
    free(img->indices);
    free(img->pal565);
    free(img);
}

uint8_t *m16_to_rgba(const M16Image *img)
{
    if (!img || !img->indices) return NULL;
    uint64_t npix = (uint64_t)img->w * (uint64_t)img->h;
    uint8_t *rgba = malloc((size_t)npix * 4);
    if (!rgba) return NULL;

    for (uint64_t p = 0; p < npix; p++) {
        uint32_t i = img->indices[p];
        uint8_t *o = rgba + (size_t)p * 4;
        if (i == 0xFF && i >= img->pal_count) {
            /* transparent (the python's `i == 0xFF and i >= count`) */
            o[0] = o[1] = o[2] = o[3] = 0;
            continue;
        }
        uint32_t c = i < img->pal_count ? img->pal565[i] : 0;
        o[0] = (uint8_t)(((c >> 11) & 31) * 255 / 31);
        o[1] = (uint8_t)(((c >> 5) & 63) * 255 / 63);
        o[2] = (uint8_t)((c & 31) * 255 / 31);
        o[3] = 255;
    }
    return rgba;
}

uint8_t *m16_decode_rgba(const void *data, size_t size,
                         uint32_t *w, uint32_t *h, uint32_t *flags)
{
    M16Image *img = m16_decode(data, size);
    if (!img) return NULL;
    uint8_t *rgba = m16_to_rgba(img);
    if (rgba) {
        if (w) *w = img->w;
        if (h) *h = img->h;
        if (flags) *flags = img->flags;
    }
    m16_image_free(img);
    return rgba;
}

static const char *pix_tok(const char *p, char *out, size_t cap)
{
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (!*p) return NULL;
    size_t n = 0;
    while (*p && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
        if (n + 1 < cap) out[n++] = *p;
        p++;
    }
    out[n] = '\0';
    return p;
}

int m16_pix_tile_range(const char *pix_text, int index, char name_out[64],
                       uint32_t *off, uint32_t *len)
{
    if (!pix_text || index < 0 || !off || !len) return -1;
    char tok[64];
    const char *p = pix_tok(pix_text, tok, sizeof tok);   /* count (unused) */
    for (int i = 0; p && i <= index; i++) {
        char name[64], so[32], sl[32];
        p = pix_tok(p, name, sizeof name);
        if (!p) break;
        p = pix_tok(p, so, sizeof so);
        if (!p) break;
        p = pix_tok(p, sl, sizeof sl);
        if (i == index) {
            if (name_out)
                snprintf(name_out, 64, "%s", name);
            *off = (uint32_t)strtoul(so, NULL, 10);
            *len = (uint32_t)strtoul(sl, NULL, 10);
            return 0;
        }
    }
    return -1;
}
