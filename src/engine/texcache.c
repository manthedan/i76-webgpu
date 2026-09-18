/*
 * texcache.c — face texture name -> decoded tile. See texcache.h.
 *
 * STEP 6 of docs/specs/m8/texture-pipeline.md: the resolution and caching
 * plumbing lands here and in scene.c while the rasterizer is still handed
 * tex=NULL, so this step must leave all 12 native<->wasm frame pairs unmoved.
 * Step 7 flips the last argument and the hashes move deliberately.
 *
 * The three policies the spec listed as "accepted omissions to close before
 * implementing" are decided here, in one place, and each is written down where
 * the code implements it:
 *
 *   1. NAME NORMALISATION AND ARCHIVE PRECEDENCE  -> tile_key() / tile_load()
 *   2. THE has_key RULE                           -> tile_build()
 *   3. ONE OWNER FOR FLAT FALLBACK                 -> this file, always by
 *      returning TEX_ID_NONE. There is no other way for a face to lose its
 *      texture: missing name, failed paint chain, absent tile, decode failure,
 *      non-power-of-two dimensions and allocation failure all converge on the
 *      same return value, and every one of them is counted separately so the
 *      cause is never guesswork.
 */
#include "engine/texcache.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "engine/paint.h"
#include "engine/pixidx.h"
#include "engine/raster.h"
#include "engine/vfs.h"
#include "engine/vqm.h"

/* ids are uint16_t; 0 is TEX_ID_NONE and 0xFFFF is the caller-side PENDING
 * sentinel, so 0xFFFE is the largest assignable id. The measured m-tier corpus
 * is 4824 tiles, and no mission references all of them. */
#define TEX_MAX_TILES 0xFFFEu

#define TEX_HASH_BITS 12
#define TEX_HASH_SIZE (1u << TEX_HASH_BITS)
#define TEX_HASH_MASK (TEX_HASH_SIZE - 1u)

/* Decompressed .pak buffers, MRU. A PURE READ CACHE of identical bytes: a miss
 * costs a deterministic reload, so eviction cannot change a framebuffer and the
 * slot count is free to be a performance number. The m tier is 483 paks /
 * 7.0 MB total with the largest at 256 KB, so four slots is ~1 MB worst case
 * and holds the working set of a single mesh, whose faces overwhelmingly share
 * one container. */
#define TEX_PAK_SLOTS 4

typedef struct {
    char     key[16];        /* "a2_1sg_1.vqm", lowercase                */
    char     source[16];     /* exact sibling PIX, or "" for normal path */
    TexTile  tile;
    uint8_t *texels;         /* the allocation tile.texels points into   */
    int      next;           /* hash chain, -1 = end                     */
} TileRec;

typedef struct {
    char     face[16];       /* raw GeoMesh face_tex entry (13 + NUL)    */
    char     vtf[16];        /* owning vehicle's paint file, "" static   */
    uint16_t id;
    int      next;
} NameRec;

typedef struct {
    char     name[16];
    uint8_t *data;
    size_t   n;
    unsigned stamp;          /* MRU age; 0 = empty slot                  */
} PakSlot;

typedef struct {
    char       key[16];      /* canonical descriptor filename            */
    char       manifest[16]; /* sibling package index, e.g. xos1_101.pix */
    TexTmtInfo info;
    char      *frames;       /* info.name_count x 9, NUL-terminated       */
    PixIndex  *pix;          /* exact sibling manifest; may be empty      */
    int        valid;        /* negative entries are cached too           */
} TmtRec;

static PixIndex *s_mpix;
static int       s_mpix_tried;

/*
 * The cockpit's DR51DASH OEG names ZDASH101/102 directly, but those VQMs live
 * in zdash101.pak rather than the ordinary m-tier. Keep this one authored
 * non-m manifest lazy: gameplay that never renders the dashboard pays nothing,
 * and we do not scan/decompress all 1,758 PIX/PAK families on an m-tier miss.
 */
static PixIndex *s_dashpix;
static int       s_dashpix_tried;

/* D-C27 smoke/dust sprite families (xwp1/xsg1/xbp1/xcp1): VQM frames in a
 * sibling PIX/PAK package with no TMT descriptor. A tiny manifest cache —
 * the native registers exactly four families (FUN_0042d040). */
#define TEX_FAMILY_SLOTS 4
static char       s_fam_name[TEX_FAMILY_SLOTS][16];
static PixIndex  *s_fam_pix[TEX_FAMILY_SLOTS];
static int        s_fam_n;
static TileRec  *s_tile;
static int       s_ntile, s_tilecap;
static int       s_tile_head[TEX_HASH_SIZE];

static NameRec  *s_name;
static int       s_nname, s_namecap;
static int       s_name_head[TEX_HASH_SIZE];

static TmtRec   *s_tmt;
static int       s_ntmt, s_tmtcap;

static PakSlot   s_pak[TEX_PAK_SLOTS];
static unsigned  s_pak_clock;

static unsigned  s_gen = 1;

/* Counters. Every one of them is a reason a face ended up flat, or the proof
 * that it did not. */
static long s_c_direct, s_c_paint, s_c_no_name, s_c_paint_fail;
static long s_c_tmt, s_c_tmt_fail;
static long s_c_absent, s_c_decode_fail, s_c_npot, s_c_alloc_fail;
static long s_c_lookups, s_c_hits;
static long s_bytes, s_keyed_tiles;

/*
 * The hash heads hold -1 for "empty", which static zero-initialisation cannot
 * express — slot 0 is a real tile index. Rather than require every caller to
 * remember texcache_init(), the tables initialise themselves on first use;
 * texcache_init() stays as the explicit entry point and does the same thing.
 */
static int s_inited;

static void ensure_init(void)
{
    if (s_inited) return;
    for (unsigned i = 0; i < TEX_HASH_SIZE; i++) {
        s_tile_head[i] = -1;
        s_name_head[i] = -1;
    }
    s_inited = 1;
}

static unsigned fnv1a(const char *s)
{
    unsigned h = 2166136261u;
    for (; *s; s++) h = (h ^ (unsigned char)*s) * 16777619u;
    return h;
}

static char lc(char c) { return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : c; }

/* Alignment-safe little-endian scalar read. */
static uint32_t rd32l(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }

/* ----------------------------------------------------------------------- */
/* 1. Name normalisation and archive precedence                             */
/* ----------------------------------------------------------------------- */

/*
 * stem -> lookup key.
 *
 * THE RULE, stated once so nothing downstream has to guess:
 *   - take characters up to the first '.' (a face names "A2_1SG_1.MAP"; the
 *     stored asset is "A2_1SG_1.VQM", so the extension is replaced, never kept)
 *   - drop trailing spaces (the GEO field is space-padded in places)
 *   - fold A-Z to a-z, ASCII only — the archives are 1997 DOS names and a
 *     locale-sensitive tolower() would be a portability hazard for no gain
 *   - append ".vqm"
 *   - truncate to 15 characters + NUL
 *
 * The truncation is deliberate and matched to pixidx.c, which stores its keys
 * through the same 16-byte snprintf. Truncating identically on both sides is
 * what makes an over-long name resolve consistently instead of resolving in the
 * index and missing at lookup.
 *
 * Returns 0 on success, -1 when the stem is empty.
 */
static int tile_key(const char *stem, char *out, size_t out_sz)
{
    char buf[32];
    size_t k = 0;
    for (const char *p = stem; *p && *p != '.'; p++) {
        if (k + 1 >= sizeof buf) break;
        buf[k++] = lc(*p);
    }
    while (k > 0 && buf[k - 1] == ' ') k--;
    if (k == 0) return -1;
    buf[k] = '\0';
    snprintf(out, out_sz, "%s.vqm", buf);
    return 0;
}

static int tmt_key(const char *name, char out[16])
{
    if (!name) return -1;
    size_t n = strlen(name);
    if (n < 5 || n >= 16 || name[n - 4] != '.' ||
        lc(name[n - 3]) != 't' || lc(name[n - 2]) != 'm' ||
        lc(name[n - 1]) != 't')
        return -1;
    for (size_t i = 0; i < n - 4; i++) {
        unsigned char c = (unsigned char)name[i];
        if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
              (c >= '0' && c <= '9') || c == '_'))
            return -1;
    }
    for (size_t i = 0; i <= n; i++) out[i] = lc(name[i]);
    return 0;
}

/*
 * TMT is a descriptor, not compressed image bytes. Shipped descriptors have a
 * 64-byte header followed by fixed char[8] VQM base names. The two measured
 * layouts account for every Nitro TMT exactly:
 *   kind 1: +0x18 names
 *   kind 2: +0x18 groups x +0x1c names
 * The byte length must agree with that header arithmetic; neither a corrupt
 * count nor trailing partial name is accepted. The raw +0x28/+0x2c fields are
 * exposed for consumers that have evidence for their animation semantics.
 */
static TmtRec *tmt_get(const char *name)
{
    char key[16];
    if (tmt_key(name, key) != 0) return NULL;
    for (int i = 0; i < s_ntmt; i++)
        if (strcmp(s_tmt[i].key, key) == 0) return &s_tmt[i];

    if (s_ntmt == s_tmtcap) {
        int cap = s_tmtcap ? s_tmtcap * 2 : 16;
        TmtRec *p = realloc(s_tmt, (size_t)cap * sizeof *p);
        if (!p) { s_c_alloc_fail++; return NULL; }
        s_tmt = p;
        s_tmtcap = cap;
    }
    TmtRec *r = &s_tmt[s_ntmt++];
    memset(r, 0, sizeof *r);
    snprintf(r->key, sizeof r->key, "%s", key);

    size_t n = 0;
    uint8_t *buf = vfs_try_read(key, &n);
    if (!buf || n < 64 || (n - 64) % 8 != 0 || rd32l(buf) != 1u) {
        vfs_free(buf);
        return r;                       /* cached invalid/missing descriptor */
    }

    r->info.kind = rd32l(buf + 0x14);
    r->info.count = rd32l(buf + 0x18);
    r->info.stride = rd32l(buf + 0x1c);
    memcpy(&r->info.rate, buf + 0x28, sizeof r->info.rate);
    r->info.mode = rd32l(buf + 0x2c);
    uint64_t names = (uint64_t)(n - 64) / 8u;
    uint64_t expected = r->info.kind == 1u ? r->info.count :
                        r->info.kind == 2u ?
                        (uint64_t)r->info.count * r->info.stride : 0u;
    if (names == 0 || names > UINT32_MAX || names != expected ||
        names > SIZE_MAX / 9u) {
        vfs_free(buf);
        return r;
    }

    r->frames = malloc((size_t)names * 9u);
    if (!r->frames) {
        s_c_alloc_fail++;
        vfs_free(buf);
        return r;
    }
    for (uint32_t i = 0; i < (uint32_t)names; i++) {
        const uint8_t *src = buf + 64 + (size_t)i * 8u;
        size_t len = 0;
        while (len < 8 && src[len]) {
            uint8_t c = src[len];
            if (!((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                  (c >= '0' && c <= '9') || c == '_')) {
                free(r->frames);
                r->frames = NULL;
                vfs_free(buf);
                return r;
            }
            r->frames[(size_t)i * 9u + len] = lc((char)c);
            len++;
        }
        if (len == 0) {
            free(r->frames);
            r->frames = NULL;
            vfs_free(buf);
            return r;
        }
        r->frames[(size_t)i * 9u + len] = '\0';
    }
    vfs_free(buf);

    r->info.name_count = (uint32_t)names;
    size_t stem = strlen(key) - 4u;
    snprintf(r->manifest, sizeof r->manifest, "%.*s.pix", (int)stem, key);
    r->pix = pixidx_build_exact(r->manifest);
    r->valid = 1;
    return r;
}

/*
 * Read the tile bytes for `key`.
 *
 * PRECEDENCE: the m-tier .pix/.pak index, the authored ZDASH cockpit package,
 * then a loose VFS file of the same name. The narrow second source is needed
 * because DR51DASH's OEG faces name ZDASH101/102 while those VQMs live in
 * zdash101.pak, not the m tier. Keeping it explicit avoids building the full
 * 23,243-entry all-PIX index during first-person rendering.
 *
 * Returns a borrowed pointer into the pak cache (valid until the next
 * tile_bytes call) or into *owned, which the caller frees.
 */
static const uint8_t *tile_bytes(const char *key, const PixIndex *exact,
                                 size_t *len, uint8_t **owned)
{
    *owned = NULL;
    *len = 0;

    if (!s_mpix_tried) {
        s_mpix = pixidx_build("m.pix");
        s_mpix_tried = 1;
    }

    /* A direct TMT owns its sibling package before the broad m-tier. XOS is
     * the shipped canary: xos1_101.vqm exists only in xos1_101.pix/pak. */
    const PixEnt *e = exact ? pixidx_find(exact, key) : NULL;
    if (!e) e = s_mpix ? pixidx_find(s_mpix, key) : NULL;
    if (!e &&
        (strcmp(key, "zdash101.vqm") == 0 ||
         strcmp(key, "zdash102.vqm") == 0)) {
        if (!s_dashpix_tried) {
            s_dashpix = pixidx_build("zdash101.pix");
            s_dashpix_tried = 1;
        }
        e = s_dashpix ? pixidx_find(s_dashpix, key) : NULL;
    }
    if (e) {
        PakSlot *slot = NULL;
        for (int i = 0; i < TEX_PAK_SLOTS; i++)
            if (s_pak[i].stamp && strcmp(s_pak[i].name, e->pak) == 0) {
                slot = &s_pak[i];
                break;
            }
        if (!slot) {
            PakSlot *victim = &s_pak[0];
            for (int i = 1; i < TEX_PAK_SLOTS; i++)
                if (s_pak[i].stamp < victim->stamp) victim = &s_pak[i];
            if (victim->data) vfs_free(victim->data);
            memset(victim, 0, sizeof *victim);
            size_t sz = 0;
            uint8_t *b = vfs_read_file(e->pak, &sz);
            if (!b) return NULL;
            snprintf(victim->name, sizeof victim->name, "%s", e->pak);
            victim->data = b;
            victim->n = sz;
            slot = victim;
        }
        slot->stamp = ++s_pak_clock;
        /* size_t is 32-bit under wasm: range-check in 64-bit. */
        if ((uint64_t)e->off + (uint64_t)e->len > (uint64_t)slot->n) return NULL;
        *len = e->len;
        return slot->data + e->off;
    }

    /* Loose VFS is the last fallback after the pix/pak index. A miss
     * here is a dangling authored name (H-UAT-024: fg_1pmt1.vqm) and
     * the caller already falls through to TEX_ID_NONE / flat. */
    size_t sz = 0;
    uint8_t *b = vfs_try_read(key, &sz);
    if (!b) return NULL;
    *owned = b;
    *len = sz;
    return b;
}

/* ----------------------------------------------------------------------- */
/* 2. Tile construction                                                     */
/* ----------------------------------------------------------------------- */

static int is_pow2(unsigned v) { return v && (v & (v - 1)) == 0; }

static uint8_t log2u(unsigned v)
{
    uint8_t s = 0;
    while ((1u << s) < v) s++;
    return s;
}

/*
 * Decode one tile into a TileRec, or fail.
 *
 * THE has_key RULE: a tile carries a colour key iff at least one of its texels
 * equals RASTER_TEXEL_TRANSPARENT (0xFF). It is a measurement of the decoded
 * tile, not a flag read out of the container, and nothing about a face or its
 * flags participates.
 *
 * The index itself is not invented: docs/REVERSING.md records that the .fnt
 * header carries an explicit "transparent" field whose value is ALWAYS 0xff,
 * so 255 is demonstrably this engine's key value in the one place the format
 * states it outright. Phase-D §3.1 later confirmed the world-fragment split:
 * keyed drawers skip it; unkeyed drawers consume it as an ordinary palette/LUT
 * index. raster.h therefore keys the behaviors off `cutout` and counts
 * texels_keyed_opaque.
 *
 * The POT/<=256 check is a rejection, not a clamp. raster.c wraps with
 * `u & umask`, which is only correct for a power of two; a non-conforming tile
 * would silently sample the wrong texels forever. Measured 0/4824 in the
 * shipped corpus, so this fires only for a user or Nitro-only asset — exactly
 * the case the spec asked to be validated rather than assumed.
 */
static int tile_build(const uint8_t *buf, size_t n, int raw_map,
                      TileRec *rec)
{
    int w = 0, h = 0;
    uint8_t *texels = NULL;
    if (raw_map) {
        if (n < 8) { s_c_decode_fail++; return -1; }
        uint32_t rw = rd32l(buf), rh = rd32l(buf + 4);
        if (rw == 0 || rh == 0 || rw > 256 || rh > 256 ||
            !is_pow2(rw) || !is_pow2(rh)) {
            s_c_npot++;
            return -1;
        }
        if ((uint64_t)8 + (uint64_t)rw * rh > (uint64_t)n) {
            s_c_decode_fail++;
            return -1;
        }
        w = (int)rw;
        h = (int)rh;
        texels = malloc((size_t)rw * rh);
        if (texels) memcpy(texels, buf + 8, (size_t)rw * rh);
        if (!texels) { s_c_alloc_fail++; return -1; }
    } else {
        texels = vqm_decode(buf, n, 0, &w, &h);
        if (!texels) { s_c_decode_fail++; return -1; }
    }

    if (w <= 0 || h <= 0 || w > 256 || h > 256 ||
        !is_pow2((unsigned)w) || !is_pow2((unsigned)h)) {
        free(texels);
        s_c_npot++;
        return -1;
    }

    size_t npx = (size_t)w * (size_t)h;
    uint8_t key = 0;
    for (size_t i = 0; i < npx; i++)
        if (texels[i] == RASTER_TEXEL_TRANSPARENT) { key = 1; break; }

    rec->texels      = texels;
    rec->tile.texels = texels;
    rec->tile.w      = (uint16_t)w;
    rec->tile.h      = (uint16_t)h;
    rec->tile.umask  = (uint16_t)(w - 1);
    rec->tile.vmask  = (uint16_t)(h - 1);
    rec->tile.vshift = log2u((unsigned)w);
    rec->tile.has_key = key;

    s_bytes += (long)npx;
    if (key) s_keyed_tiles++;
    return 0;
}

/*
 * Raw .map entry loader for WDEF-named world assets (the mission sky at
 * WRLD +82; scene.md §2). This is NOT the GEO-face rule above: a face named
 * "FOO.MAP" resolves to FOO.VQM, but a WRLD field names the stored asset
 * itself, a raw indexed bitmap (pipeline.md §3.1: u32 w, u32 h, then w*h
 * LEVEL-palette indices). terrain.c keeps its own copy of this loader for
 * the surface tile (it predates this function); the two decode the same
 * format with the same rejection rules.
 *
 * Resolution precedence mirrors tile_bytes: m-tier .pix/.pak index first,
 * then a loose VFS file — and the loose arm is not theoretical: every miss8
 * sky (nk_1cld*.map) is a loose record, while the miss16 nk_6cld*.map set
 * sits in ncloud*m.pak. texcache owns the m-tier index, which is why this
 * lives here rather than beside its caller.
 *
 * POT 8..256 is REJECTED, not clamped: callers wrap with `u & umask`, the
 * same contract tile_build enforces for the rasterizer. Returns 0 on
 * success; *pixels is then caller-owned (and identical to out->texels).
 */
int texcache_load_map(const char *name, RTex *out, uint8_t **pixels)
{
    *pixels = NULL;
    if (!name || !*name) return -1;

    char key[16];
    size_t k = 0;
    for (const char *p = name; *p && k + 1 < sizeof key; p++)
        key[k++] = lc(*p);
    key[k] = '\0';
    if (k == 0) return -1;

    size_t len = 0;
    uint8_t *owned = NULL;
    const uint8_t *bytes = tile_bytes(key, NULL, &len, &owned);
    if (!bytes) { vfs_free(owned); return -1; }

    int rc = -1;
    if (len >= 8 + 64) {
        uint32_t w = rd32l(bytes), h = rd32l(bytes + 4);
        if (w >= 8 && h >= 8 && w <= 256 && h <= 256 &&
            is_pow2(w) && is_pow2(h) &&
            (uint64_t)8 + (uint64_t)w * h <= (uint64_t)len) {
            uint8_t *px = malloc((size_t)w * h);
            if (px) {
                memcpy(px, bytes + 8, (size_t)w * h);
                uint8_t key_hit = 0;
                for (uint32_t i = 0; i < w * h; i++)
                    if (px[i] == RASTER_TEXEL_TRANSPARENT) { key_hit = 1; break; }
                out->texels  = px;
                out->w       = (uint16_t)w;
                out->h       = (uint16_t)h;
                out->umask   = (uint16_t)(w - 1);
                out->vmask   = (uint16_t)(h - 1);
                out->vshift  = log2u(w);
                out->has_key = key_hit;
                *pixels = px;
                rc = 0;
            }
        }
    }
    vfs_free(owned);
    return rc;
}

/* Tile memo: one decoded copy per (key, exact package), however many faces
 * want it. The source axis prevents a user package that repeats an m-tier VQM
 * name from inheriting whichever copy happened to load first. */
static uint16_t tile_get(const char *key, const char *source,
                         const PixIndex *exact, int raw_map)
{
    if (!source) source = "";
    char hash_key[32];
    snprintf(hash_key, sizeof hash_key, "%s|%s", key, source);
    unsigned h = fnv1a(hash_key) & TEX_HASH_MASK;
    for (int i = s_tile_head[h]; i >= 0; i = s_tile[i].next)
        if (strcmp(s_tile[i].key, key) == 0 &&
            strcmp(s_tile[i].source, source) == 0)
            return (uint16_t)(i + 1);            /* id 0 is TEX_ID_NONE */

    if ((unsigned)s_ntile >= TEX_MAX_TILES) { s_c_alloc_fail++; return TEX_ID_NONE; }
    if (s_ntile == s_tilecap) {
        int cap = s_tilecap ? s_tilecap * 2 : 256;
        TileRec *p = realloc(s_tile, (size_t)cap * sizeof *p);
        if (!p) { s_c_alloc_fail++; return TEX_ID_NONE; }
        s_tile = p;
        s_tilecap = cap;
    }

    size_t len = 0;
    uint8_t *owned = NULL;
    const uint8_t *bytes = tile_bytes(key, exact, &len, &owned);
    if (!bytes) {
        vfs_free(owned);
        s_c_absent++;
        return TEX_ID_NONE;
    }

    TileRec rec;
    memset(&rec, 0, sizeof rec);
    int ok = tile_build(bytes, len, raw_map, &rec);
    vfs_free(owned);
    if (ok != 0) return TEX_ID_NONE;             /* counted inside tile_build */

    snprintf(rec.key, sizeof rec.key, "%s", key);
    snprintf(rec.source, sizeof rec.source, "%s", source);
    rec.next = s_tile_head[h];
    s_tile[s_ntile] = rec;
    s_tile_head[h] = s_ntile;
    s_ntile++;
    return (uint16_t)s_ntile;                    /* index + 1 */
}

/* ----------------------------------------------------------------------- */
/* 3. Public resolution                                                     */
/* ----------------------------------------------------------------------- */

int texcache_tmt_info(const char *tmt_name, TexTmtInfo *out)
{
    ensure_init();
    if (!out) return -1;
    memset(out, 0, sizeof *out);
    TmtRec *r = tmt_get(tmt_name);
    if (!r || !r->valid) return -1;
    *out = r->info;
    return 0;
}

uint16_t texcache_resolve_tmt_frame(const char *tmt_name, uint32_t frame)
{
    ensure_init();
    TmtRec *r = tmt_get(tmt_name);
    if (!r || !r->valid || frame >= r->info.name_count) {
        s_c_tmt_fail++;
        return TEX_ID_NONE;
    }

    char key[16];
    const char *base = r->frames + (size_t)frame * 9u;
    if (tile_key(base, key, sizeof key) != 0) {
        s_c_tmt_fail++;
        return TEX_ID_NONE;
    }
    const PixIndex *exact = r->pix && pixidx_find(r->pix, key) ? r->pix : NULL;
    const char *source = exact ? r->manifest : "";
    uint16_t id = tile_get(key, source, exact, 0);

    /* TMT software frames ship in both encodings. XOS uses VQM+codebook;
     * MCALT, with the same descriptor shape, stores raw indexed MAP records. */
    if (id == TEX_ID_NONE) {
        size_t stem = strlen(key) - 4u;
        snprintf(key + stem, sizeof key - stem, ".map");
        exact = r->pix && pixidx_find(r->pix, key) ? r->pix : NULL;
        source = exact ? r->manifest : "";
        id = tile_get(key, source, exact, 1);
    }
    if (id == TEX_ID_NONE) s_c_tmt_fail++;
    else s_c_tmt++;
    return id;
}

/* Resolve one frame from an exact sibling PIX/PAK package that has no TMT
 * descriptor (the D-C27 smoke families: "xbp1_101.pix" + "xbp1_201.vqm").
 * Same fail-closed law as the TMT path: a missing package or frame is
 * TEX_ID_NONE, never a substituted tile. */
uint16_t texcache_resolve_family_frame(const char *pix_manifest,
                                       const char *frame_name)
{
    ensure_init();
    if (!pix_manifest || !frame_name)
        return TEX_ID_NONE;

    PixIndex *ix = NULL;
    for (int i = 0; i < s_fam_n; i++)
        if (strcmp(s_fam_name[i], pix_manifest) == 0) {
            ix = s_fam_pix[i];
            break;
        }
    if (!ix && s_fam_n < TEX_FAMILY_SLOTS) {
        snprintf(s_fam_name[s_fam_n], sizeof s_fam_name[0], "%s",
                 pix_manifest);
        s_fam_pix[s_fam_n] = pixidx_build_exact(pix_manifest);
        ix = s_fam_pix[s_fam_n];
        s_fam_n++;
    }

    char key[16];
    if (tile_key(frame_name, key, sizeof key) != 0)
        return TEX_ID_NONE;
    const PixIndex *exact = ix && pixidx_find(ix, key) ? ix : NULL;
    uint16_t id = tile_get(key, exact ? pix_manifest : "", exact, 0);
    if (id == TEX_ID_NONE) {
        /* Same dual-encoding law as TMT frames: raw indexed MAP storage. */
        size_t stem = strlen(key) - 4u;
        snprintf(key + stem, sizeof key - stem, ".map");
        exact = ix && pixidx_find(ix, key) ? ix : NULL;
        id = tile_get(key, exact ? pix_manifest : "", exact, 1);
    }
    return id;
}

static uint16_t resolve_uncached(const char *face_name, const char *vtf_file)
{    char key[16];

    /*
     * Vehicle panels first. "V1 FT LF.MAP" is not a file name at all — it is a
     * paint-scheme slot, and paint.c walks .vtf -> .tmt -> base for it. Only a
     * vehicle has a .vtf, so static geometry never enters this branch.
     */
    if (vtf_file && vtf_file[0]) {
        char base[16];
        if (paint_resolve_face(face_name, vtf_file, base, sizeof base) == 0) {
            if (tile_key(base, key, sizeof key) == 0) {
                s_c_paint++;
                return tile_get(key, "", NULL, 0);
            }
        } else if (face_name[0] == 'V' || face_name[0] == 'v') {
            /*
             * A placeholder whose chain broke: the panel is "NULL" in this
             * car's scheme, or the .tmt is missing. Falling through to the
             * direct path would look up a name with spaces in it and miss
             * anyway; counting it separately is what makes "the car is flat"
             * distinguishable from "the tile is absent".
             */
            s_c_paint_fail++;
            return TEX_ID_NONE;
        }
    }

    if (tmt_key(face_name, key) == 0)
        return texcache_resolve_tmt_frame(face_name, 0);
    if (tile_key(face_name, key, sizeof key) != 0) return TEX_ID_NONE;
    s_c_direct++;
    return tile_get(key, "", NULL, 0);
}

uint16_t texcache_resolve(const char *face_name, const char *vtf_file)
{
    ensure_init();
    if (!face_name || !face_name[0]) { s_c_no_name++; return TEX_ID_NONE; }
    if (!vtf_file) vtf_file = "";

    s_c_lookups++;

    char fkey[32];
    snprintf(fkey, sizeof fkey, "%s|%s", face_name, vtf_file);
    unsigned h = fnv1a(fkey) & TEX_HASH_MASK;
    for (int i = s_name_head[h]; i >= 0; i = s_name[i].next)
        if (strcmp(s_name[i].face, face_name) == 0 &&
            strcmp(s_name[i].vtf, vtf_file) == 0) {
            s_c_hits++;
            return s_name[i].id;
        }

    uint16_t id = resolve_uncached(face_name, vtf_file);

    /*
     * Memoize NEGATIVES too. Resolution is a pure function of (VFS contents,
     * face_name, vtf_file) and the VFS does not change within a mission, so a
     * miss is permanent — and re-running the paint chain per frame for every
     * unresolvable face would be the single most expensive thing in this file.
     */
    if (s_nname == s_namecap) {
        int cap = s_namecap ? s_namecap * 2 : 512;
        NameRec *p = realloc(s_name, (size_t)cap * sizeof *p);
        if (!p) { s_c_alloc_fail++; return id; }
        s_name = p;
        s_namecap = cap;
    }
    NameRec *r = &s_name[s_nname];
    snprintf(r->face, sizeof r->face, "%s", face_name);
    snprintf(r->vtf, sizeof r->vtf, "%s", vtf_file);
    r->id = id;
    r->next = s_name_head[h];
    s_name_head[h] = s_nname;
    s_nname++;
    return id;
}

const TexTile *texcache_tile(uint16_t id)
{
    ensure_init();
    if (id == TEX_ID_NONE || id == TEX_ID_PENDING) return NULL;
    int i = (int)id - 1;
    if (i < 0 || i >= s_ntile) return NULL;
    return &s_tile[i].tile;
}

const char *texcache_tile_name(uint16_t id)
{
    ensure_init();
    if (id == TEX_ID_NONE || id == TEX_ID_PENDING) return "";
    int i = (int)id - 1;
    return i >= 0 && i < s_ntile ? s_tile[i].key : "";
}

void texcache_init(void) { ensure_init(); }

void texcache_reset(void)
{
    for (int i = 0; i < s_ntile; i++) free(s_tile[i].texels);
    free(s_tile);
    s_tile = NULL;
    s_ntile = s_tilecap = 0;

    free(s_name);
    s_name = NULL;
    s_nname = s_namecap = 0;

    for (int i = 0; i < s_ntmt; i++) {
        free(s_tmt[i].frames);
        pixidx_free(s_tmt[i].pix);
    }
    free(s_tmt);
    s_tmt = NULL;
    s_ntmt = s_tmtcap = 0;

    for (int i = 0; i < TEX_PAK_SLOTS; i++)
        if (s_pak[i].data) vfs_free(s_pak[i].data);
    memset(s_pak, 0, sizeof s_pak);
    s_pak_clock = 0;

    pixidx_free(s_mpix);
    s_mpix = NULL;
    s_mpix_tried = 0;
    pixidx_free(s_dashpix);
    s_dashpix = NULL;
    s_dashpix_tried = 0;

    for (int i = 0; i < s_fam_n; i++)
        pixidx_free(s_fam_pix[i]);
    memset(s_fam_pix, 0, sizeof s_fam_pix);
    memset(s_fam_name, 0, sizeof s_fam_name);
    s_fam_n = 0;

    vqm_cache_reset();

    s_c_direct = s_c_paint = s_c_no_name = s_c_paint_fail = 0;
    s_c_tmt = s_c_tmt_fail = 0;
    s_c_absent = s_c_decode_fail = s_c_npot = s_c_alloc_fail = 0;
    s_c_lookups = s_c_hits = 0;
    s_bytes = s_keyed_tiles = 0;

    s_inited = 0;
    ensure_init();
    s_gen++;
}

unsigned texcache_generation(void) { return s_gen; }

int texcache_stats(char *buf, size_t n)
{
    return snprintf(buf, n,
        "texcache: tiles=%d bytes=%ld keyed=%ld names=%d lookups=%ld hits=%ld "
        "direct=%ld paint=%ld paint_fail=%ld tmt=%ld tmt_fail=%ld "
        "no_name=%ld absent=%ld decode_fail=%ld npot=%ld alloc_fail=%ld",
        s_ntile, s_bytes, s_keyed_tiles, s_nname, s_c_lookups, s_c_hits,
        s_c_direct, s_c_paint, s_c_paint_fail, s_c_tmt, s_c_tmt_fail,
        s_c_no_name, s_c_absent, s_c_decode_fail, s_c_npot, s_c_alloc_fail);
}
