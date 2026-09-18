#ifndef PCX_H
#define PCX_H

/*
 * pcx.h — PCX image loader
 *
 * The original game used 8-bit palettized PCX files throughout:
 *   addon\loadgame.pcx   — initial boot loading screen
 *   addon\loadscr.pcx    — mission/session load screen
 *   <scenario>.pcx       — scenario-specific loading screen
 *   (others throughout menus and UI)
 *
 * Confirmed from REVERSING.md:
 *   FUN_00470b90  pcx_open()    — loads header, returns ptr or NULL
 *   FUN_00471000  pcx_decode()  — decodes pixels into a dest surface
 *   FUN_00499b20  pcx_free()    — frees the header ptr
 *
 * Format: ZSoft PCX, version 5, 8-bit indexed (1 plane), RLE encoded.
 * Palette: 256 × RGB, last 769 bytes of file (0x0C marker + 768 data bytes).
 */

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t  width;
    uint32_t  height;
    uint8_t  *pixels;       /* width × height bytes, 8-bit palette indices */
    uint8_t   palette[768]; /* 256 × RGB triples (matches on-disk layout)  */
} PcxImage;

/*
 * pcx_load()
 *   Load and decode a PCX file. Returns NULL on failure (file not found,
 *   bad header, OOM). Caller owns the returned pointer.
 */
PcxImage *pcx_load(const char *path);

/*
 * pcx_free()
 *   Release all memory for a PcxImage returned by pcx_load().
 */
void pcx_free(PcxImage *img);

#endif /* PCX_H */
