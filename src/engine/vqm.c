/*
 * vqm.c — .vqm tile decode. See vqm.h.
 *
 * STEP 2: moved verbatim from hud.c, where it served only the dashboard. The
 * algorithm is unchanged; the ONE interface change is that hud.c's read of its
 * own `s.orient` static becomes an explicit `transpose` parameter, so this
 * module has no dependency on the HUD. That makes this a behaviour-preserving
 * move rather than a strictly "pure" one -- labelled honestly, because "pure
 * move" invites rubber-stamping.
 *
 * Gated on: all 4824 pak-embedded tiles decoding byte-identically, both
 * tools/hud_probe.c PGMs unmoved, and all 12 native<->wasm frame pairs unmoved.
 *
 * STEP 3 added, as behaviour separate from the move: lazy codebook load, an
 * all-retaining codebook cache, and vfs_free() on VFS buffers (the original
 * used free(), which works only because vfs_free IS free today -- a latent
 * contract violation).
 */
#include "engine/vqm.h"
#include "engine/vfs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0]         | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* -----------------------------------------------------------------------
 * Codebook cache (step 3)
 *
 * 201 distinct codebooks serve 4824 tiles, so without a cache a full corpus
 * decode costs 4824 LZO decompressions of ~64 KB each to read the same 201
 * buffers over and over.
 *
 * CACHING CANNOT CHANGE OUTPUT, and that is the property that makes sizing a
 * performance decision rather than a rendering one: decode is a pure function
 * of (tile bytes, codebook bytes), so a miss costs a deterministic reload of
 * byte-identical data. Nothing here is keyed by pointer -- native ASLR varies
 * addresses run to run, and an address-ordered structure would make iteration
 * order vary with it.
 *
 * ALL codebooks are retained rather than running an N-slot LRU. The design
 * proposed 8 slots, sized against an assumed 45 codebooks; the real number is
 * 201 (tools/tex_verify.c), and 8 thrashes under any cyclic reuse of 9+. The
 * whole set is ~13 MB at the observed sizes, and eviction can be added if
 * measurement ever justifies it -- but a cache that cannot evict also cannot
 * be blamed for a frame difference.
 * ----------------------------------------------------------------------- */

#define VQM_MAX_CBK 256

typedef struct {
    char     name[16];
    uint8_t *data;
    size_t   n;
    int      missing;      /* looked up once, not present: do not retry */
} CbkSlot;

static CbkSlot s_cbk[VQM_MAX_CBK];
static int     s_ncbk;
static long    s_hits, s_misses, s_absent, s_solid_only;

static const CbkSlot *cbk_get(const char *name)
{
    for (int i = 0; i < s_ncbk; i++)
        if (strcmp(s_cbk[i].name, name) == 0) { s_hits++; return &s_cbk[i]; }

    if (s_ncbk >= VQM_MAX_CBK) return NULL;      /* counted by the caller */
    CbkSlot *c = &s_cbk[s_ncbk];
    snprintf(c->name, sizeof c->name, "%s", name);
    c->data = vfs_read_file(name, &c->n);
    c->missing = (c->data == NULL);
    s_ncbk++;
    s_misses++;
    if (c->missing) s_absent++;
    return c;
}

uint8_t *vqm_decode(const uint8_t *buf, size_t n, int transpose,
                    int *ow, int *oh)
{
    /* Set on ENTRY, not only on success: hud.c's original left these
     * untouched on all five failure returns, which is a trap for any caller
     * that does not pre-initialise. The existing caller does, so this is
     * behaviour-neutral. */
    if (ow) *ow = 0;
    if (oh) *oh = 0;

    if (n < 24) return NULL;
    uint32_t w = rd_u32(buf + 0), h = rd_u32(buf + 4);
    if (w == 0 || h == 0 || w > 4096 || h > 4096) return NULL;

    char cbk_name[13];
    memcpy(cbk_name, buf + 8, 12);
    cbk_name[12] = '\0';
    for (int i = 0; cbk_name[i]; i++)
        if (cbk_name[i] >= 'A' && cbk_name[i] <= 'Z')
            cbk_name[i] = (char)(cbk_name[i] + ('a' - 'A'));

    uint32_t bw = (w + 3u) / 4u, bh = (h + 3u) / 4u;
    size_t nblocks = (size_t)bw * bh;
    if (n - 24 < nblocks * 2u) {
        fprintf(stderr, "[vqm] stream truncated (%zu < %zu)\n",
                n - 24, nblocks * 2u);
        return NULL;
    }
    const uint8_t *stream = buf + 24;

    /*
     * LAZY CODEBOOK. A block with bit 15 set is a solid fill and needs no
     * codebook; 265 of the 4824 tiles are 100% solid and referenced a .cbk
     * they never read. Loading eagerly made those tiles fail outright when
     * their codebook was absent -- losing real art for no reason. Scan first,
     * and only fetch when a block actually indexes the codebook.
     */
    int needs_cbk = 0;
    for (size_t bi = 0; bi < nblocks; bi++)
        if (!(rd_u16(stream + bi * 2) & 0x8000u)) { needs_cbk = 1; break; }

    const uint8_t *cbk = NULL;
    size_t cbk_n = 0;
    if (needs_cbk) {
        const CbkSlot *c = cbk_get(cbk_name);
        if (!c || c->missing) {
            fprintf(stderr, "[vqm] codebook %s not found\n", cbk_name);
            return NULL;
        }
        cbk = c->data;
        cbk_n = c->n;
    } else {
        s_solid_only++;
    }

    uint8_t *out = malloc((size_t)w * h);
    if (!out) return NULL;

    for (size_t bi = 0; bi < nblocks; bi++) {
        /* block grid position of stored block bi */
        uint32_t bx = transpose ? (uint32_t)(bi / bh) : (uint32_t)(bi % bw);
        uint32_t by = transpose ? (uint32_t)(bi % bh) : (uint32_t)(bi / bw);
        uint16_t v = rd_u16(stream + bi * 2);

        uint8_t blk[16];
        if (v & 0x8000u) {
            memset(blk, v & 0xFFu, sizeof blk);
        } else {
            size_t off = 4u + (size_t)v * 16u;
            if (!cbk || off + 16 > cbk_n) {
                memset(blk, 0, sizeof blk);
            } else {
                memcpy(blk, cbk + off, 16);
            }
        }
        for (uint32_t py = 0; py < 4; py++) {
            for (uint32_t px = 0; px < 4; px++) {
                uint32_t x = bx * 4 + px, y = by * 4 + py;
                if (x >= w || y >= h) continue;
                out[(size_t)y * w + x] =
                    transpose ? blk[px * 4 + py] : blk[py * 4 + px];
            }
        }
    }

    if (ow) *ow = (int)w;
    if (oh) *oh = (int)h;
    return out;
}

void vqm_cache_reset(void)
{
    for (int i = 0; i < s_ncbk; i++)
        if (s_cbk[i].data) vfs_free(s_cbk[i].data);
    memset(s_cbk, 0, sizeof s_cbk);
    s_ncbk = 0;
    s_hits = s_misses = s_absent = s_solid_only = 0;
}

int vqm_stats(char *buf, size_t n)
{
    return snprintf(buf, n,
        "vqm: cbk_loaded=%d hits=%ld misses=%ld absent=%ld solid_only=%ld",
        s_ncbk, s_hits, s_misses, s_absent, s_solid_only);
}
