/*
 * paint.c — vehicle paint-scheme texture resolution (see paint.h).
 */

#include "paint.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vfs.h"

/* Type-1 vehicle TMT record: 64-byte header, u32 name count at +0x18,
 * then count x char[8] base names (one per damage state). The general TMT
 * loader also admits the type-2 (+0x18 x +0x1c) layout; paint VTFs use type 1.
 * Verified on blade101.tmt. */
#define TMT_COUNT_OFF  0x18
#define TMT_NAMES_OFF  0x40
#define TMT_NAME_LEN   8

/* VTFC payload: char[13] vdf, char[16] scheme, then the TMT table. */
#define VTFC_TMT_OFF   29
#define VTFC_TMT_COUNT 78
#define VTFC_NAME_LEN  13

static int str_ieq(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca = (char)(ca - ('a' - 'A'));
        if (cb >= 'a' && cb <= 'z') cb = (char)(cb - ('a' - 'A'));
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == *b;
}

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int paint_face_key(const char *face_name, char *out, size_t out_sz)
{
    if (!face_name || !out || out_sz < 16) return -1;
    if (face_name[0] != 'V' && face_name[0] != 'v') return -1;
    char buf[32];
    size_t k = 0;
    for (const char *p = face_name + 1; *p && *p != '.'; p++) {
        if (*p == ' ') continue;
        if (k + 1 >= sizeof buf) return -1;
        char c = *p;
        if (c >= 'a' && c <= 'z') c = (char)(c - ('a' - 'A'));
        buf[k++] = c;
    }
    if (k == 0) return -1;
    buf[k] = '\0';
    /* LF -> LT: the GEO spells the panel "left front", the TMT "LT". */
    if (k >= 2 && buf[k - 2] == 'L' && buf[k - 1] == 'F') buf[k - 1] = 'T';
    snprintf(out, out_sz, "%s.TMT", buf);
    return 0;
}

/* Scan every .pix manifest for `tile`; on hit fill pak/off/len. */
struct scan_ctx {
    const char *tile;
    char pak[192];
    uint32_t off, len;
    int found;
};

static void scan_cb(const char *name, int src_type, void *ud)
{
    (void)src_type;
    struct scan_ctx *c = ud;
    if (c->found || !name) return;
    size_t n = strlen(name);
    /* The ".pix"->".pak" rewrite below writes at name[n-4]; a name longer
     * than the buffer would put that write out of bounds. */
    if (n < 4 || n >= sizeof c->pak) return;
    if (!str_ieq(name + n - 4, ".pix")) return;
    size_t psz = 0;
    char *text = vfs_read_file(name, &psz);
    if (!text) return;
    /* vfs_read_file returns raw file bytes with NO terminator; strtok_r
     * would walk off the end of the allocation (ASan SEGV, and the wasm
     * "memory access out of bounds" this chain hit). Copy into a
     * NUL-terminated scratch before tokenising. */
    char *body = malloc(psz + 1);
    if (!body) { vfs_free(text); return; }
    memcpy(body, text, psz);
    body[psz] = '\0';
    vfs_free(text);
    char *save = NULL;
    for (char *line = strtok_r(body, "\n", &save); line;
         line = strtok_r(NULL, "\n", &save)) {
        char tname[64];
        uint32_t off = 0, len = 0;
        if (sscanf(line, "%63s %u %u", tname, &off, &len) != 3) continue;
        if (!str_ieq(tname, c->tile)) continue;
        memcpy(c->pak, name, n);
        c->pak[n] = '\0';
        memcpy(c->pak + n - 4, ".pak", 5);   /* incl. terminator */
        c->off = off;
        c->len = len;
        c->found = 1;
        break;
    }
    free(body);
}

int paint_resolve_face(const char *face_name, const char *vtf_file,
                       char *base_out, size_t base_sz)
{
    char key[32];
    if (!base_out || base_sz < 16) return -1;
    if (paint_face_key(face_name, key, sizeof key) != 0) return -1;
    if (!vtf_file || !*vtf_file) return -1;

    size_t vsz = 0;
    uint8_t *v = vfs_read_file(vtf_file, &vsz);
    if (!v) return -1;

    /* BWD2 chunk walk to VTFC. */
    char tmt[VTFC_NAME_LEN + 1] = {0};
    for (size_t off = 8; off + 8 <= vsz; ) {
        uint32_t total = rd_u32(v + off + 4);
        if (total < 8 || (uint64_t)off + total > (uint64_t)vsz) break;
        if (memcmp(v + off, "VTFC", 4) == 0) {
            size_t p = off + 8 + VTFC_TMT_OFF;
            for (int i = 0; i < VTFC_TMT_COUNT; i++) {
                uint64_t e = (uint64_t)p + (uint64_t)i * VTFC_NAME_LEN;
                if (e + VTFC_NAME_LEN > (uint64_t)vsz) break;
                /* Copy out first: the field need not be NUL-terminated. */
                char ent[VTFC_NAME_LEN + 1];
                memcpy(ent, v + (size_t)e, VTFC_NAME_LEN);
                ent[VTFC_NAME_LEN] = '\0';
                size_t n = strlen(ent);
                if (n <= 3 || str_ieq(ent, "NULL")) continue;
                if (str_ieq(ent + 3, key)) {
                    memcpy(tmt, ent, n + 1);
                    break;
                }
            }
            break;
        }
        off += total;
    }
    vfs_free(v);
    if (!tmt[0]) return -1;

    /* The .TMT lives in a pak indexed by a sibling .pix — scan for it
     * rather than deriving the pak name (rampage2.vtf -> rampag2t.pak is
     * not a derivable rule). It may also be a loose file. A loose miss
     * is the normal case (H-UAT-024: 2dr1MDLT.TMT is authored-but-absent). */
    size_t tsz = 0;
    uint8_t *owned = vfs_try_read(tmt, &tsz);
    const uint8_t *rec = NULL;
    uint32_t reclen = 0;
    struct scan_ctx ctx;
    uint8_t *pak = NULL;
    if (owned && tsz >= TMT_NAMES_OFF + TMT_NAME_LEN) {
        rec = owned;
        reclen = (uint32_t)tsz;
    } else {
        vfs_free(owned);
        owned = NULL;
        memset(&ctx, 0, sizeof ctx);
        ctx.tile = tmt;
        vfs_foreach(scan_cb, &ctx);
        if (!ctx.found || ctx.len < TMT_NAMES_OFF + TMT_NAME_LEN) return -1;
        size_t psz = 0;
        pak = vfs_read_file(ctx.pak, &psz);
        /* size_t is 32-bit under wasm — range-check in 64-bit. */
        if (!pak || (uint64_t)ctx.off + (uint64_t)ctx.len > (uint64_t)psz) {
            vfs_free(pak);
            return -1;
        }
        rec = pak + ctx.off;
        reclen = ctx.len;
    }

    uint32_t cnt = rd_u32(rec + TMT_COUNT_OFF);
    int ok = 0;
    if (cnt >= 1 &&
        (uint64_t)TMT_NAMES_OFF + (uint64_t)cnt * TMT_NAME_LEN <= (uint64_t)reclen) {
        snprintf(base_out, base_sz, "%.*s", TMT_NAME_LEN,
                 (const char *)rec + TMT_NAMES_OFF);
        ok = base_out[0] != '\0';
    }
    vfs_free(owned);
    vfs_free(pak);
    return ok ? 0 : -1;
}
