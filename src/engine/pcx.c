/*
 * pcx.c — PCX image loader
 *
 * ZSoft PCX format, version 5, 8-bit palettized (1 plane).
 * All I'76 PCX files use this variant.
 *
 * On-disk layout:
 *   [0..127]    128-byte header
 *   [128..]     RLE-encoded scanlines (bytes_per_line * nplanes per row)
 *   [EOF-769]   0x0C palette marker
 *   [EOF-768]   256 × 3 RGB bytes
 *
 * RLE rule: if high two bits of a byte are both set (byte >= 0xC0),
 *           count = low 6 bits, next byte = pixel value to repeat.
 *           Otherwise, the byte itself is the pixel value (count = 1).
 */

#include "pcx.h"
#include "vfs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ZSoft PCX file header — 128 bytes */
typedef struct __attribute__((packed)) {
    uint8_t  manufacturer;   /* must be 0x0A */
    uint8_t  version;
    uint8_t  encoding;       /* 1 = RLE */
    uint8_t  bits_per_plane;
    uint16_t x_min, y_min;
    uint16_t x_max, y_max;
    uint16_t hdpi, vdpi;
    uint8_t  ega_palette[48];
    uint8_t  reserved;
    uint8_t  nplanes;
    uint16_t bytes_per_line;
    uint16_t palette_info;
    uint16_t h_screen;
    uint16_t v_screen;
    uint8_t  filler[54];
} PcxHeader;

typedef struct { const uint8_t *data; size_t size; size_t pos; } Buf;

static inline int buf_getc(Buf *b)
{
    return (b->pos < b->size) ? (int)b->data[b->pos++] : -1;
}

PcxImage *pcx_load(const char *path)
{
    size_t size;
    uint8_t *data = vfs_read_file(path, &size);
    if (!data)
        return NULL; /* vfs already logged the error */

    if (size < sizeof(PcxHeader) + 769) {
        fprintf(stderr, "[pcx] Too small: %s\n", path);
        vfs_free(data);
        return NULL;
    }

    const PcxHeader *hdr = (const PcxHeader *)data;

    if (hdr->manufacturer != 0x0A || hdr->encoding != 1 ||
        hdr->bits_per_plane != 8  || hdr->nplanes != 1) {
        fprintf(stderr, "[pcx] Unsupported format in %s "
                "(manufacturer=%02x encoding=%d bpp=%d planes=%d)\n",
                path, hdr->manufacturer, hdr->encoding,
                hdr->bits_per_plane, hdr->nplanes);
        vfs_free(data);
        return NULL;
    }

    uint32_t w   = (uint32_t)(hdr->x_max - hdr->x_min + 1);
    uint32_t h   = (uint32_t)(hdr->y_max - hdr->y_min + 1);
    uint32_t bpl = hdr->bytes_per_line; /* may be >= w for alignment */

    PcxImage *img = malloc(sizeof(PcxImage));
    if (!img) { vfs_free(data); return NULL; }

    img->width  = w;
    img->height = h;
    img->pixels = malloc((size_t)w * h);
    if (!img->pixels) { free(img); vfs_free(data); return NULL; }

    Buf buf = { data, size, sizeof(PcxHeader) };

    /* RLE decode — one scanline at a time */
    for (uint32_t y = 0; y < h; y++) {
        uint32_t x = 0;
        while (x < bpl) {
            int byte = buf_getc(&buf);
            if (byte < 0) goto truncated;

            if ((byte & 0xC0) == 0xC0) {
                /* Run */
                int count = byte & 0x3F;
                int val   = buf_getc(&buf);
                if (val < 0) goto truncated;
                while (count-- > 0 && x < bpl) {
                    if (x < w) img->pixels[y * w + x] = (uint8_t)val;
                    x++;
                }
            } else {
                /* Literal */
                if (x < w) img->pixels[y * w + x] = (uint8_t)byte;
                x++;
            }
        }
    }

    /* Palette: last 769 bytes */
    {
        const uint8_t *pal_ptr = data + size - 769;
        if (*pal_ptr != 0x0C) {
            fprintf(stderr, "[pcx] Missing palette marker in %s — zeroing\n", path);
            memset(img->palette, 0, 768);
        } else {
            memcpy(img->palette, pal_ptr + 1, 768);
        }
    }

    vfs_free(data);
    return img;

truncated:
    fprintf(stderr, "[pcx] Truncated pixel data: %s\n", path);
    pcx_free(img);
    vfs_free(data);
    return NULL;
}

void pcx_free(PcxImage *img)
{
    if (!img) return;
    free(img->pixels);
    free(img);
}
