/*
 * scene.c — M2 scene-object module (see scene.h)
 *
 * Mission (.msn) BWD2 chunk walk, ODEF/OBJ placement parse, placement-chain
 * geometry resolution (SDF statics, VCF->VDF/WDF cars) and a wireframe scene
 * renderer into the 8-bit indexed framebuffer.
 *
 * Sources: docs/specs/m2/scene.md (byte tables, confidence tags), verified
 * against the Nitro Pack data (miss8/P01.MSN). Empirical divergences from the
 * spec are marked DECISION and summarized in the M2 report:
 *
 *  D1  SGEO holds partCount + 6*partCount 120-byte records (observed: 3 damage
 *      states x (normal + wrecked) sets), not "partCount + 2 wrecked" as the
 *      spec's Open76-derived table says. We read the first partCount (intact).
 *  D2  VGEO holds partCount + 28*partCount 100-byte records (4 damage-state
 *      sets + 24 further sets; the spec's "skip 100*partCount*12 then
 *      first-person" does not match Nitro's vfypony.vdf). We read the first
 *      partCount (damage state 0).
 *  D3  Mesh resolution: the original merges every *g.pix index into a virtual
 *      file table mapping "<part>.geo" -> (pak, offset, length); our
 *      meshcache loader reads whole files only (its documented unimplemented
 *      vfs_lod_pak boundary). scene.c therefore builds the merged pix index
 *      itself and decodes pak sub-records with geomesh_decode; whole-file
 *      ".geo" and "<part>g.pak" still go through geo_cache_acquire (the
 *      meshview convention).
 *  D4  OBJ basis vectors are orthonormal with right x up = forward (verified
 *      on all 33 P01 records) => right-handed, used as COLUMNS of the object
 *      rotation (world = R*local + pos). Part frames use the same convention.
 *  D5  Wheels: WGEO has 16 records for partCount=1 (8 right + 8 left
 *      variants); we take record 0 as the intact right wheel and the first
 *      record with a differing name as the intact left wheel. Wheel world
 *      transform = WLOC frame x WGEO part frame (verified numerically: wheel
 *      bottom lands at y=0, correct side and axle).
 *  D6  VCF WEPN weapon mounts (GDF chains) are skipped — scene.md §6.5 defers
 *      weapon rendering to M3.
 *  D7  classId 82 (not in the spec's enum table; itd27_1/ijd27t1 in P01) is
 *      treated like the other static classes: resolve <label>.sdf.
 *  D8  RDEF road ribbons are left to the terrain module (visual-only per
 *      scene.md §5, conformed to terrain); ADEF/FSM entity binding is M3.
 *  D9  Camera: fixed elevation 0.35 rad and the binary-derived 90-degree
 *      horizontal FOV (raster.h RASTER_FOV_TAN_HALF; the render signature
 *      carries only yaw/dist); dist <= 0 auto-fits the scene.
 *  D10 LDEF string objects: parsed per scene.md §4 (label, classId, position
 *      list, yaw-to-next instancing). No Nitro mission ships LDEF entries, so
 *      this path is compile-checked only.
 *  D11 Drive surfaces: parts the decoded geometry proves to be ground rather
 *      than structure (large, near-horizontal, vehicle-thin, at the object's
 *      base — the gas-station lot slab in P01's bflgila1 is the model case)
 *      are classified once at commit. Collision registers per-part OBBs and
 *      skips these (the car rides the terrain across them); the filled
 *      renderer lifts exactly these parts 0.1 m (terrain.c ROAD_LIFT_M's
 *      precedent) so a terrain-conformed slab stops z-fighting the
 *      heightfield. Walls, pumps, posts, signs and roofs keep colliders.
 *  D12 Native drivable structures (not a port decision): at load, SDFC/ODEF
 *      classes 11/12/13 register exact upward OEG face polygons with terrain's
 *      chassis probe. Class 12 admits only SGEO pclass-13 deck parts. This is
 *      separate from D11: no render lift and no terrain-height substitution.
 */

#include "scene.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "engine/fs.h"
#include "engine/vfs.h"
#include "engine/meshcache.h"
#include "engine/geomesh.h"
#include "engine/raster.h"
#include "engine/texcache.h"
#include "engine/terrain.h"

/* ----------------------------------------------------------------------- */
/* Little-endian scalar reads (alignment-safe)                              */
/* ----------------------------------------------------------------------- */

static uint32_t rd_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static uint16_t rd_u16(const uint8_t *p) { uint16_t v; memcpy(&v, p, 2); return v; }
static float    rd_f32(const uint8_t *p) { float    v; memcpy(&v, p, 4); return v; }

/* Bounded string copy, always NUL-terminates (no truncation warnings). */
static void copy_str(char *dst, size_t dstsz, const char *src)
{
    size_t i = 0;
    if (dstsz == 0) return;
    while (i + 1 < dstsz && src[i]) { dst[i] = src[i]; i++; }
    dst[i] = '\0';
}

/* ----------------------------------------------------------------------- */
/* Transforms — 3x3 basis (columns = right/up/forward) + translation (D4)   */
/* ----------------------------------------------------------------------- */

typedef struct {
    double m[3][3];     /* column j = local axis j expressed in parent space */
    double t[3];
} Xform;

static Xform xform_identity(void)
{
    Xform x;
    memset(&x, 0, sizeof x);
    x.m[0][0] = x.m[1][1] = x.m[2][2] = 1.0;
    return x;
}

/* a after b: world = a * (b * v) */
static Xform xform_compose(const Xform *a, const Xform *b)
{
    Xform r;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = a->m[i][0] * b->m[0][j]
                      + a->m[i][1] * b->m[1][j]
                      + a->m[i][2] * b->m[2][j];
        }
        r.t[i] = a->m[i][0] * b->t[0]
               + a->m[i][1] * b->t[1]
               + a->m[i][2] * b->t[2]
               + a->t[i];
    }
    return r;
}

static void xform_apply(const Xform *x, const float *v, double *out)
{
    for (int i = 0; i < 3; i++)
        out[i] = x->m[i][0] * v[0] + x->m[i][1] * v[1] + x->m[i][2] * v[2] + x->t[i];
}

/* Build from the stored right/up/forward/position quadruple (D4). */
static Xform xform_from_frame(const uint8_t *p)
{
    Xform x;
    for (int i = 0; i < 3; i++) {
        x.m[i][0] = rd_f32(p +  0 + (size_t)i * 4);     /* right   */
        x.m[i][1] = rd_f32(p + 12 + (size_t)i * 4);     /* up      */
        x.m[i][2] = rd_f32(p + 24 + (size_t)i * 4);     /* forward */
        x.t[i]    = rd_f32(p + 36 + (size_t)i * 4);     /* position */
    }
    return x;
}

static void xform_to12(const Xform *x, double out12[12])
{
    for (int i = 0; i < 3; i++) {
        out12[i]     = x->m[i][0];
        out12[3 + i] = x->m[i][1];
        out12[6 + i] = x->m[i][2];
        out12[9 + i] = x->t[i];
    }
}

static void xform_from12(Xform *x, const double in12[12])
{
    for (int i = 0; i < 3; i++) {
        x->m[i][0] = in12[i];
        x->m[i][1] = in12[3 + i];
        x->m[i][2] = in12[6 + i];
        x->t[i]    = in12[9 + i];
    }
}

static double xform_dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void xform_cross3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static int xform_normalize3(double v[3])
{
    double n = sqrt(xform_dot3(v, v));
    if (!isfinite(n) || n < 1e-12)
        return 0;
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
    return 1;
}

/* Nlerp object bases the same way webmain interpolates the player car:
 * right/forward columns + translation, reconstruct up, renormalize. */
static void xform_nlerp(Xform *out, const Xform *a, const Xform *b,
                        double alpha)
{
    double ra[12], rb[12], ro[12], r[3], f[3], u[3];
    xform_to12(a, ra);
    xform_to12(b, rb);
    for (int i = 0; i < 3; i++) {
        r[i] = ra[i] + (rb[i] - ra[i]) * alpha;
        f[i] = ra[6 + i] + (rb[6 + i] - ra[6 + i]) * alpha;
        ro[9 + i] = ra[9 + i] + (rb[9 + i] - ra[9 + i]) * alpha;
    }
    if (!xform_normalize3(f))
        memcpy(f, rb + 6, sizeof f);
    double rf = xform_dot3(r, f);
    for (int i = 0; i < 3; i++)
        r[i] -= f[i] * rf;
    if (!xform_normalize3(r))
        memcpy(r, rb, sizeof r);
    xform_cross3(f, r, u);
    if (!xform_normalize3(u))
        memcpy(u, rb + 3, sizeof u);
    memcpy(ro, r, sizeof r);
    memcpy(ro + 3, u, sizeof u);
    memcpy(ro + 6, f, sizeof f);
    xform_from12(out, ro);
}

/* ----------------------------------------------------------------------- */
/* BWD2 chunk walking                                                        */
/* ----------------------------------------------------------------------- */

typedef struct {
    char     tag[5];
    uint32_t total;     /* total length incl. 8-byte header */
    size_t   payload;   /* offset of payload in buffer     */
    size_t   next;      /* offset of the following chunk   */
} Chunk;

static int chunk_at(const uint8_t *b, size_t len, size_t off, Chunk *c)
{
    if (off + 8 > len) return 0;
    uint32_t total = rd_u32(b + off + 4);
    if (total < 8 || (size_t)total > len - off) return 0;
    memcpy(c->tag, b + off, 4);
    c->tag[4] = '\0';
    c->total   = total;
    c->payload = off + 8;
    c->next    = off + total;
    return 1;
}

static int tag_is(const Chunk *c, const char four[4])
{
    return memcmp(c->tag, four, 4) == 0;
}

/* ----------------------------------------------------------------------- */
/* Merged g.pix index — the original VFS's virtual mesh file table (D3)      */
/* ----------------------------------------------------------------------- */

typedef struct {
    char     key[13];   /* "<part>.geo", lowercase */
    char     pak[13];   /* backing g-tier pak, lowercase */
    uint32_t off;
    uint32_t len;
} PixEnt;

static PixEnt *s_pix;
static int     s_npix, s_pixcap;
static int     s_pix_ready;

static int pix_cmp(const void *a, const void *b)
{
    return strcmp(((const PixEnt *)a)->key, ((const PixEnt *)b)->key);
}

static void pix_add(const char *key, const char *pak, uint32_t off, uint32_t len)
{
    if (s_npix == s_pixcap) {
        int ncap = s_pixcap ? s_pixcap * 2 : 1024;
        PixEnt *np = realloc(s_pix, (size_t)ncap * sizeof *np);
        if (!np) return;
        s_pix = np;
        s_pixcap = ncap;
    }
    PixEnt *e = &s_pix[s_npix++];
    memset(e, 0, sizeof *e);
    copy_str(e->key, sizeof e->key, key);
    copy_str(e->pak, sizeof e->pak, pak);
    e->off = off;
    e->len = len;
}

/* Parse one text .pix buffer: line 1 = count, then "NAME <off> <len>". */
static void pix_parse(const char *pak, const char *text, size_t len)
{
    size_t i = 0;
    int line_no = 0;
    while (i < len) {
        size_t j = i;
        while (j < len && text[j] != '\n') j++;
        size_t llen = j - i;
        if (llen > 0 && text[i + llen - 1] == '\r') llen--;
        char line[96];
        if (llen >= sizeof line) llen = sizeof line - 1;
        memcpy(line, text + i, llen);
        line[llen] = '\0';
        i = j + 1;
        if (line_no++ == 0) continue;   /* count line — informational */
        char name[32];
        unsigned off, plen;
        if (sscanf(line, "%31s %u %u", name, &off, &plen) != 3) continue;
        for (char *c = name; *c; c++) *c = (char)tolower((unsigned char)*c);
        pix_add(name, pak, off, plen);
    }
}

static void pix_collect(const char *name, int src_type, void *ud)
{
    (void)src_type; (void)ud;
    size_t n = strlen(name);
    if (n < 5 || strcasecmp(name + n - 5, "g.pix") != 0) return;

    size_t sz = 0;
    char *buf = vfs_read_file(name, &sz);
    if (!buf) return;
    char pak[16];
    snprintf(pak, sizeof pak, "%s", name);
    memcpy(pak + n - 3, "pak", 4);      /* <base>g.pix -> <base>g.pak */
    pix_parse(pak, buf, sz);
    vfs_free(buf);
}

static void pix_index_build(void)
{
    if (s_pix_ready) return;
    vfs_foreach(pix_collect, NULL);
    qsort(s_pix, (size_t)s_npix, sizeof *s_pix, pix_cmp);
    s_pix_ready = 1;
    fprintf(stdout, "[scene] pix index: %d mesh records\n", s_npix);
}

static const PixEnt *pix_lookup(const char *key)    /* key = "<part>.geo" lc */
{
    if (!s_pix_ready) pix_index_build();
    PixEnt probe;
    memset(&probe, 0, sizeof probe);
    copy_str(probe.key, sizeof probe.key, key);
    return bsearch(&probe, s_pix, (size_t)s_npix, sizeof *s_pix, pix_cmp);
}

/* ----------------------------------------------------------------------- */
/* Mesh table — per-scene, keyed by part name (D3)                           */
/* ----------------------------------------------------------------------- */

#define SCENE_MAX_MESHES 768

/*
 * M8 V2: per-face tile ids for ONE paint scheme.
 *
 * The list exists because a mesh is shared and a paint scheme is not: two
 * copies of the same chassis with different .vtf files resolve the SAME face
 * name to DIFFERENT tiles, so the array cannot hang off the mesh alone. Static
 * geometry has vtf "" and therefore exactly one node, which is the common case.
 *
 * Keyed by the vtf string rather than by an object index so that N cars sharing
 * a scheme share one array -- and so the key is a value, never a pointer whose
 * ordering could vary with ASLR.
 */
typedef struct TileSet {
    struct TileSet *next;
    char      vtf[16];
    unsigned  gen;          /* texcache_generation() it was built for */
    uint16_t *ids;          /* [num_faces], TEX_ID_NONE = draw flat    */
} TileSet;

typedef struct {
    char     key[9];        /* part name, lowercase */
    GeoMesh *mesh;          /* NULL => known-unresolvable */
    int      from_cache;    /* owned by meshcache (release on unload) */

    /* M8: face_rgb resolved to palette indices, one entry per face. Resolved
     * once per mesh per palette rather than per face per frame -- the search
     * is an exact 256-entry scan (see raster_rgb_to_index) and the level
     * palette only changes when a mission loads. */
    uint8_t *face_idx;
    unsigned face_idx_gen;  /* raster_palette_generation() it was built for */

    TileSet *tiles;         /* M8 V2: one node per distinct paint scheme */
} MeshSlot;

static MeshSlot s_mesh[SCENE_MAX_MESHES];
static int      s_nmesh;
static int      s_mesh_ok, s_mesh_fail;

static GeoMesh *mesh_load_pak_record(const PixEnt *e)
{
    size_t sz = 0;
    uint8_t *pak = vfs_read_file(e->pak, &sz);
    if (!pak) return NULL;
    GeoMesh *m = NULL;
    if ((size_t)e->off + e->len <= sz)
        m = geomesh_decode(pak + e->off, e->len);
    vfs_free(pak);
    return m;
}

static int mesh_get(const char *part)
{
    char key[9];
    int i;
    for (i = 0; part[i] && i < 8; i++)
        key[i] = (char)tolower((unsigned char)part[i]);
    key[i] = '\0';

    for (i = 0; i < s_nmesh; i++)
        if (strcmp(s_mesh[i].key, key) == 0)
            return i;

    if (s_nmesh == SCENE_MAX_MESHES) return -1;
    MeshSlot *s = &s_mesh[s_nmesh];
    memset(s, 0, sizeof *s);
    snprintf(s->key, sizeof s->key, "%s", key);

    char name[24];

    /* (a) whole-file loose/archived .geo through the mesh cache */
    snprintf(name, sizeof name, "%s.geo", key);
    if (vfs_exists(name) > 0) {
        s->mesh = (GeoMesh *)geo_cache_acquire(name);
        s->from_cache = s->mesh != NULL;
    }
    /* (b) OEG sub-record of a g-tier pak via the merged pix index */
    if (!s->mesh) {
        const PixEnt *e = pix_lookup(name);
        if (e) s->mesh = mesh_load_pak_record(e);
    }
    /* (c) whole "<part>g.pak" (first-record semantics, meshview-style) */
    if (!s->mesh) {
        snprintf(name, sizeof name, "%sg.pak", key);
        if (vfs_exists(name) > 0) {
            s->mesh = (GeoMesh *)geo_cache_acquire(name);
            s->from_cache = s->mesh != NULL;
        }
    }

    if (s->mesh) s_mesh_ok++;
    else         s_mesh_fail++;
    s_nmesh++;
    return (int)(s - s_mesh);
}

/* ----------------------------------------------------------------------- */
/* Placed objects                                                            */
/* ----------------------------------------------------------------------- */

typedef struct {
    int   mesh;         /* index into s_mesh, -1 = none */
    Xform local;        /* model-space transform (parents chained) */
    unsigned char drive_surf;   /* D11 terrain-owned slab classifier  */
    unsigned char pclass;       /* SGEO record +0x5c class word (0 = n/a) */
    unsigned char gate_state;   /* class-7 gate: 1 = closed, 0 = open    */
} PartInst;

/* SGEO part-record class word value that marks a gate child (the native's
 * class-7 linked entity — scene.h's gate API block). */
#define SCENE_PART_CLASS_GATE 7
static int part_active(const PartInst *p)
{
    return p->pclass != SCENE_PART_CLASS_GATE || p->gate_state != 0;
}


typedef struct {
    char     label[9];
    int      label_id;
    uint32_t class_id;
    uint32_t sdf_class; /* SDFC +16; native drivable registrar class word */
    uint16_t flags;
    uint16_t team;
    int      hidden;    /* M7 combat/story visibility (additive; 0=shown) */
    int      consumed;  /* live player owns this ODEF body's rendering    */
    Xform    world;
    PartInst *parts;
    int      nparts;
    int      drivable_object; /* terrain registry id + 1; zero = none */
    /* M8 V2: this object's own paint scheme, from its VCFC +29. "" for
     * everything that is not a car. Per-object from the start rather than
     * borrowing the player's, so AI cars never wear the player's livery. */
    char     vtf[16];
    /* H-UAT-013 distant-vehicle impostor: the object's dominant authored
     * face colour as a palette index, resolved lazily per palette
     * generation (0 = not yet resolved / none resolvable). */
    uint8_t  imp_idx;
    unsigned imp_gen;
    /* Presentation history (H-UAT-077b). tick_prev/curr are last two sim
     * worlds; present is the prepared draw pose. Collision keeps `world`. */
    Xform    tick_prev;
    Xform    tick_curr;
    Xform    present;
    int      present_ready;
    int      present_armed;
} SceneObj;

static SceneObj *s_objs;
static int      s_nobj, s_objcap;
static int      s_present_draw;

static const Xform *obj_draw_world(const SceneObj *o)
{
    return (s_present_draw && o->present_armed) ? &o->present : &o->world;
}

static int s_n_cars, s_n_statics, s_n_strings, s_n_markers, s_n_skipped;
static double s_center[3], s_min[3], s_max[3];
static char   s_mission[32];
static int    s_loaded;
static unsigned s_gate_generation = 1;


/* Mission sky (WDEF/WRLD +82; scene.md §2): the authored cloud .map the
 * filled renderer samples instead of its placeholder gradient. Loaded on
 * scene_load, released on scene_unload; s_sky_pixels is owned here and is
 * the storage s_sky.texels points into. */
static char     s_sky_name[16];
static RTex     s_sky;
static uint8_t *s_sky_pixels;
static int      s_sky_ok;

/* Mission shade/translucency tables (WDEF/WRLD +43/+56; scene.md §2,
 * CONFIRMED layout — the same fields nitro.exe's FUN_004b45f0 reads at
 * blob+0x33/+0x40, docs/specs/re/phase-d-renderer.md §1). Every shipped
 * .lum/.tbl decompresses to exactly 65536 bytes — the loader's 0x4000-dword
 * copy — so a fixed-size payload needs no heap: static storage owned here,
 * valid from scene_load to scene_unload, never allocated per pixel or per
 * frame. s_table_gen lets the renderer's per-frame sync notice a reload
 * even when the storage address is unchanged. */
static char     s_lum_name[16];
static char     s_tbl_name[16];
static uint8_t  s_lum[256 * 256];
static uint8_t  s_tbl[256 * 256];
static int      s_lum_ok, s_tbl_ok;
static unsigned s_table_gen;

/* Copy a 13-byte null-padded WRLD string field, lowercased. */
static void wrld_field(const uint8_t *f, char *out)
{
    size_t l = 0;
    while (l < 13 && f[l]) l++;
    for (size_t i = 0; i < l; i++) {
        char ch = (char)f[i];
        if (ch >= 'A' && ch <= 'Z') ch = (char)(ch - 'A' + 'a');
        out[i] = ch;
    }
    out[l] = '\0';
}

/* Load one WRLD-named 65536-byte table through the VFS. The size check IS
 * the format check: every shipped .lum/.tbl is exactly 65536 bytes
 * decompressed (measured 35/35 across both game profiles), and anything
 * else is not the asset this loader understands. */
static void table_load(const char *name, uint8_t out[256 * 256], int *ok)
{
    size_t n = 0;
    uint8_t *buf = vfs_read_file(name, &n);
    if (buf && n == (256 * 256)) {
        memcpy(out, buf, 256 * 256);
        *ok = 1;
    }
    vfs_free(buf);
}

/*
 * WDEF payload -> WRLD sub-chunk -> the world-asset name fields (scene.md
 * §2, CONFIRMED layout): luma table +43, translucency table +56, sky .map
 * +82 — all 13-byte null-padded fields. The tables resolve through the
 * plain VFS (they live in nitro.zfs next to the level .act); the sky
 * decodes through the m-tier index / loose VFS. Any failure leaves the
 * matching ok flag at 0, which the renderer reads as "use the placeholder
 * fallback". Measured over the corpus: all 149 miss8/miss16 missions name a
 * resolvable sky (nk_1cld*.map loose, nk_6cld*.map in ncloud*m.pak), and
 * every miss8 mission names a resolvable .lum/.tbl pair (27/27 distinct
 * pairs present in nitro.zfs), so the fallbacks are for user/corrupt data.
 */
static void wrld_assets_load(const uint8_t *wdef, size_t len)
{
    for (size_t off = 0; off < len; ) {
        Chunk c;
        if (!chunk_at(wdef, len, off, &c)) break;
        if (tag_is(&c, "EXIT")) break;
        if (tag_is(&c, "WRLD")) {
            const uint8_t *p = wdef + c.payload;
            size_t plen = c.total - 8;
            if (plen >= 43 + 13) {
                wrld_field(p + 43, s_lum_name);
                if (s_lum_name[0]) table_load(s_lum_name, s_lum, &s_lum_ok);
            }
            if (plen >= 56 + 13) {
                wrld_field(p + 56, s_tbl_name);
                if (s_tbl_name[0]) table_load(s_tbl_name, s_tbl, &s_tbl_ok);
            }
            if (plen >= 82 + 13) {
                wrld_field(p + 82, s_sky_name);
                if (s_sky_name[0])
                    s_sky_ok = texcache_load_map(s_sky_name, &s_sky,
                                                 &s_sky_pixels) == 0;
            }
            return;
        }
        off = c.next;
    }
}

const char *scene_sky_name(void) { return s_sky_name; }
const char *scene_lum_name(void) { return s_lum_name; }
const char *scene_tbl_name(void) { return s_tbl_name; }

int scene_sky_tex(const RTex **out)
{
    if (!s_sky_ok) return 0;
    if (out) *out = &s_sky;
    return 1;
}

const uint8_t *scene_shade_table(void)
{
    return s_lum_ok ? s_lum : NULL;
}

const uint8_t *scene_translucency_table(void)
{
    return s_tbl_ok ? s_tbl : NULL;
}

unsigned scene_table_generation(void) { return s_table_gen; }

/* Driver-queued dynamic meshes (M3 chase view): drawn after the placed
 * objects, queue order = draw order. Owned by the caller; scene_unload()
 * drops the references. The queue is fixed storage so the drive render
 * can rebuild it every frame without allocating (scene.h SCENE_DYN_MAX). */
static GeoMesh *s_dyn_mesh[SCENE_DYN_MAX];
static const RTex *s_dyn_tex[SCENE_DYN_MAX];
static Xform    s_dyn_xf[SCENE_DYN_MAX];
static int      s_dyn_count;
/* The driven car's paint scheme (car_vtf_file()). Set once per mission by the
 * driver, not per frame with the transform: it identifies the car, not its
 * pose, and re-deriving it 20 times a second would be the paint chain in the
 * render loop. */
static char     s_dyn_vtf[16];

/* Raw part record: name[8] @0, frame (right/up/forward/pos) @8, parent[8] @56.
 * Shared by SGEO (120-byte records), VGEO and WGEO (100-byte records). */
static void part_record_read(const uint8_t *r, char *name, char *parent, Xform *frame)
{
    memcpy(name, r, 8);
    name[8] = '\0';
    *frame = xform_from_frame(r + 8);
    memcpy(parent, r + 56, 8);
    parent[8] = '\0';
}

static int name_is_null(const char *n)
{
    return n[0] == '\0' || strcasecmp(n, "NULL") == 0;
}

/*
 * Append model parts (first `count` stride-byte records at `p`) to object `o`,
 * resolving parent chains in record order (parents precede children in every
 * file inspected; a missing parent falls back to WORLD and is counted).
 * `class_off` is the record offset of the part class word (SGEO: +0x5c;
 * -1 for record families without one — VGEO wheels/bodies).
 */
static void object_add_parts(SceneObj *o, const uint8_t *p, size_t avail,
                             size_t stride, int count, int class_off)
{
    int max_rec = (int)(avail / stride);
    if (count > max_rec) count = max_rec;

    Xform *model = malloc((size_t)count * sizeof *model);
    if (!model) return;

    int parent_miss = 0;
    for (int i = 0; i < count; i++) {
        char name[9], parent[9];
        Xform frame;
        part_record_read(p + (size_t)i * stride, name, parent, &frame);

        Xform pm = xform_identity();
        if (!name_is_null(parent) && strcasecmp(parent, "WORLD") != 0) {
            /* find the already-resolved parent by re-reading earlier names */
            int found = 0;
            for (int j = 0; j < i; j++) {
                char pn[9], pp[9];
                Xform pf;
                part_record_read(p + (size_t)j * stride, pn, pp, &pf);
                if (strcasecmp(pn, parent) == 0) { pm = model[j]; found = 1; break; }
            }
            if (!found) parent_miss++;
        }
        model[i] = xform_compose(&pm, &frame);

        if (name_is_null(name)) continue;
        int slot = mesh_get(name);
        if (slot < 0) continue;
        PartInst *np = realloc(o->parts, (size_t)(o->nparts + 1) * sizeof *np);
        if (!np) continue;
        o->parts = np;
        unsigned pclass = class_off >= 0
                        ? rd_u32(p + (size_t)i * stride + (size_t)class_off)
                        : 0;
        o->parts[o->nparts].mesh  = slot;
        o->parts[o->nparts].local = model[i];
        o->parts[o->nparts].drive_surf = 0;
        o->parts[o->nparts].pclass = (unsigned char)pclass;
        /* class-7 parts load with the gate state the native triggerGate
         * zeroes (scene.h): 1 = closed until triggered. */
        o->parts[o->nparts].gate_state =
            pclass == SCENE_PART_CLASS_GATE ? 1 : 0;
        o->nparts++;
    }
    if (parent_miss)
        fprintf(stderr, "[scene] %s: %d part(s) with unresolved parent\n",
                o->label, parent_miss);
    free(model);
}

/* ----------------------------------------------------------------------- */
/* Asset parsers (flat BWD2 files; EXIT chunks appear mid-stream as list    */
/* terminators, so we scan the whole file and dispatch by tag)              */
/* ----------------------------------------------------------------------- */

/* <label>.sdf -> SDFC class word + SGEO first partCount records (D1).
 * Native's registrar reads the attached static's SDFC class (11/12/13),
 * while SGEO +0x5c distinguishes class-12 deck parts (13) from rails. */
static void build_static(SceneObj *o)
{
    char name[24];
    snprintf(name, sizeof name, "%s.sdf", o->label);
    size_t sz = 0;
    uint8_t *buf = vfs_read_file(name, &sz);
    if (!buf) return;

    int built = 0;
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        if (tag_is(&c, "SDFC") && c.total - 8 >= 20) {
            o->sdf_class = rd_u32(buf + c.payload + 16);
        } else if (tag_is(&c, "SGEO") && !built) {
            const uint8_t *p = buf + c.payload;
            size_t avail = c.total - 8;
            if (avail >= 4) {
                int count = (int)rd_u32(p);
                /* SGEO record +0x5c is the part class word (7 = gate). */
                object_add_parts(o, p + 4, avail - 4, 120, count, 0x5c);
                built = 1;
            }
        }
        off = c.next;
    }
    vfs_free(buf);
}

/* VCFC: +16 vdf name, +29 vtf (paint scheme), +54/+67/+80 front/mid/rear wdf
 * names. The +29 field is the same one car.c reads for the player (vcfc_read);
 * it is the input to the paint-slot texture chain, so the renderer needs it for
 * every placed car, not only the driven one. */
static void vcf_read(const uint8_t *p, char *vdf, char *vtf, char wdf[3][13])
{
    memcpy(vdf, p + 16, 13);
    vdf[12] = '\0';
    memcpy(vtf, p + 29, 13);
    vtf[12] = '\0';
    for (int i = 0; i < 3; i++) {
        memcpy(wdf[i], p + 54 + (size_t)i * 13, 13);
        wdf[i][12] = '\0';
    }
}

/* WLOC: 6 records of u32 present + 4xvec3 frame + f32. */
static int wloc_read(const uint8_t *p, size_t avail, Xform frames[6], int present[6])
{
    if (avail < 6 * 56) return 0;
    for (int i = 0; i < 6; i++) {
        const uint8_t *r = p + (size_t)i * 56;
        present[i] = rd_u32(r) != 0;
        frames[i]  = xform_from_frame(r + 4);
    }
    return 1;
}

/*
 * Wheels from <wdf>.wdf WGEO (D5): right = record 0, left = first record with
 * a differing name. Places the pair on the two WLOC frames of axle `axle`.
 */
static void car_add_wheels(SceneObj *o, const char *wdf_name,
                           const Xform frames[6], const int present[6], int axle)
{
    if (name_is_null(wdf_name)) return;
    size_t sz = 0;
    uint8_t *buf = vfs_read_file(wdf_name, &sz);
    if (!buf) return;

    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        if (tag_is(&c, "WGEO")) {
            const uint8_t *p = buf + c.payload;
            size_t avail = c.total - 8;
            if (avail < 4) break;
            int nrec = (int)((avail - 4) / 100);

            char rname[9], lname[9], par[9];
            Xform rframe, lframe;
            part_record_read(p + 4, rname, par, &rframe);
            int have_l = 0;
            for (int i = 1; i < nrec; i++) {
                part_record_read(p + 4 + (size_t)i * 100, lname, par, &lframe);
                if (strcmp(lname, rname) != 0) { have_l = 1; break; }
            }
            if (!have_l) { memcpy(lname, rname, 9); lframe = rframe; }

            for (int wi = axle * 2; wi < axle * 2 + 2 && wi < 6; wi++) {
                if (!present[wi]) continue;
                int right_side = frames[wi].t[0] > 0.0;
                const char *wn = right_side ? rname : lname;
                Xform wf = right_side ? rframe : lframe;
                if (name_is_null(wn)) continue;
                int slot = mesh_get(wn);
                if (slot < 0) continue;
                PartInst *np = realloc(o->parts, (size_t)(o->nparts + 1) * sizeof *np);
                if (!np) break;
                o->parts = np;
                o->parts[o->nparts].mesh  = slot;
                o->parts[o->nparts].local = xform_compose(&frames[wi], &wf);
                o->parts[o->nparts].drive_surf = 0;
                o->parts[o->nparts].pclass = 0;   /* WGEO: no class word */
                o->parts[o->nparts].gate_state = 0;
                o->nparts++;
            }
            break;
        }
        off = c.next;
    }
    vfs_free(buf);
}

/* <vcf_base>.vcf -> <vdf>.vdf (VGEO state-0 parts + WLOC) + wheel WDFs
 * (D2,D5). `vcf_base` is normally the object's own ODEF label; it is a
 * parameter so scene_obj_make_car() can dress an object whose label is not
 * an asset name (a melee `spawn` marker) in a shipped vehicle. */
static void build_car_named(SceneObj *o, const char *vcf_base)
{
    char name[24];
    snprintf(name, sizeof name, "%s.vcf", vcf_base);
    size_t sz = 0;
    /* probe first: mission data references cars no shipped asset defines
     * (e.g. N21's "check4") — a miss is routine, not log-worthy */
    uint8_t *vcf = vfs_exists(name) ? vfs_read_file(name, &sz) : NULL;
    if (!vcf) return;

    char vdf[13] = {0}, vtf[13] = {0}, wdf[3][13] = {{0}};
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(vcf, sz, off, &c)) break;
        if (tag_is(&c, "VCFC") && c.total - 8 >= 93) {
            vcf_read(vcf + c.payload, vdf, vtf, wdf);
            break;
        }
        off = c.next;
    }
    vfs_free(vcf);
    if (vdf[0] == '\0') return;
    if (!name_is_null(vtf)) copy_str(o->vtf, sizeof o->vtf, vtf);

    /* the VCFC vdf field already carries the ".vdf" extension */
    uint8_t *vbuf = vfs_read_file(vdf, &sz);
    if (!vbuf) return;

    Xform wframes[6];
    int wpresent[6] = {0};
    int have_wloc = 0;

    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(vbuf, sz, off, &c)) break;
        if (tag_is(&c, "VGEO")) {
            const uint8_t *p = vbuf + c.payload;
            size_t avail = c.total - 8;
            if (avail >= 4) {
                int count = (int)rd_u32(p);
                /* VGEO records have no part class word. */
                object_add_parts(o, p + 4, avail - 4, 100, count, -1);
            }
        } else if (tag_is(&c, "WLOC")) {
            have_wloc = wloc_read(vbuf + c.payload, c.total - 8, wframes, wpresent);
        }
        off = c.next;
    }
    vfs_free(vbuf);

    if (have_wloc)
        for (int axle = 0; axle < 3; axle++)
            car_add_wheels(o, wdf[axle], wframes, wpresent, axle);
}

static void build_car(SceneObj *o)
{
    build_car_named(o, o->label);
}

/* ----------------------------------------------------------------------- */
/* Mission (.msn) parse                                                      */
/* ----------------------------------------------------------------------- */

/* scene.md §3.2 packed label: 7 ASCII bits per byte, high bits form the id. */
static void label_unpack(const uint8_t *p, char *name, int *id_out)
{
    int id = 0, n = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t b = p[i];
        if (b & 0x80) id = (id << 1) | 1;
        else          id = (id << 1) & 0xFE;
        char ch = (char)(b & 0x7F);
        if (ch && n < 8) name[n++] = ch;
    }
    name[n] = '\0';
    *id_out = id;
}

static int label_is_marker(const char *l)
{
    return strcasecmp(l, "SPAWN") == 0 || strcasecmp(l, "REGEN") == 0 ||
           strcasecmp(l, "CHECK") == 0;
}

/* ----------------------------------------------------------------------- */
/* D11 drive-surface classification                                         */
/*                                                                          */
/* One aggregate OBB per object cannot represent a building standing on its */
/* own drive-on apron: P01's bflgila1 unions 21 parts into a single 60x37 m */
/* box that walled off the whole gas-station lot, while the part the car    */
/* must cross (FL1_LOT1, a 75x120 m zero-thickness slab at the object base) */
/* is geometrically a patch of GROUND, not a piece of structure. Collision  */
/* (web build_colliders) and the filled renderer share this one classifier: */
/* collision skips these parts (the car rides the terrain across them) and  */
/* the renderer lifts them SCENE_SURFACE_LIFT_M so a slab conformed to the  */
/* heightfield stops trading raster depth ties with it — the terrain.c      */
/* ROAD_LIFT_M precedent, applied per part rather than to global depth      */
/* semantics. Walls, pumps, posts, signs and roofs fail at least one test   */
/* and keep their own colliders. Pure decoded-geometry policy: no labels,   */
/* no mission coordinates.                                                  */
/* ----------------------------------------------------------------------- */

/* Both horizontal half-extents at least a car width — smaller flat parts  */
/* (curbs, pump islands) are obstacles to drive into, not surfaces.        */
#define SCENE_SURFACE_MIN_HALF   2.0
/* Vertical span far under the car body band (car.h CAR_COLLIDE_H = 2 m):  */
/* a slab, not a wall or a machine. Pumps (1.54 m) fail this.              */
#define SCENE_SURFACE_MAX_THICK  0.5
/* Bottom within this of the object's lowest part bottom — a ground        */
/* surface, not a roof deck (bflgila1's roof flats sit ~4.4 m up).         */
#define SCENE_SURFACE_BASE_BAND  1.0
/* World up axis within ~26 degrees of vertical.                           */
#define SCENE_SURFACE_UP_MIN     0.9
/* The render lift itself is SCENE_SURFACE_LIFT_M in scene.h (shared with
 * the GPU export path). */

/* World-space AABB of one placed part (mesh bbox corners through
 * world∘local). Returns 0 on success, -1 when the part has no mesh. */
static int part_world_aabb(const SceneObj *o, int pi, double lo[3],
                           double hi[3])
{
    int slot = o->parts[pi].mesh;
    GeoMesh *m = (slot >= 0 && slot < s_nmesh) ? s_mesh[slot].mesh : NULL;
    if (!m || m->num_verts <= 0) return -1;
    Xform xf = xform_compose(&o->world, &o->parts[pi].local);
    for (int i = 0; i < 3; i++) { lo[i] = 1e30; hi[i] = -1e30; }
    for (int c = 0; c < 8; c++) {
        float v[3] = {
            (c & 1) ? m->bb_max[0] : m->bb_min[0],
            (c & 2) ? m->bb_max[1] : m->bb_min[1],
            (c & 4) ? m->bb_max[2] : m->bb_min[2],
        };
        double w[3];
        xform_apply(&xf, v, w);
        for (int k = 0; k < 3; k++) {
            if (w[k] < lo[k]) lo[k] = w[k];
            if (w[k] > hi[k]) hi[k] = w[k];
        }
    }
    return 0;
}

static int part_classify_drive_surface(const SceneObj *o, int pi)
{
    double lo[3], hi[3];
    if (part_world_aabb(o, pi, lo, hi) != 0)
        return 0;
    Xform xf = xform_compose(&o->world, &o->parts[pi].local);
    if (fabs(xf.m[1][1]) < SCENE_SURFACE_UP_MIN)
        return 0;                                   /* not near-horizontal */
    if (hi[1] - lo[1] > SCENE_SURFACE_MAX_THICK)
        return 0;                                   /* wall/machine height  */
    if ((hi[0] - lo[0]) * 0.5 < SCENE_SURFACE_MIN_HALF ||
        (hi[2] - lo[2]) * 0.5 < SCENE_SURFACE_MIN_HALF)
        return 0;                                   /* strip, not a surface */
    double base = 1e30;
    for (int j = 0; j < o->nparts; j++) {
        double plo[3], phi[3];
        if (part_world_aabb(o, j, plo, phi) == 0 && plo[1] < base)
            base = plo[1];
    }
    if (lo[1] - base > SCENE_SURFACE_BASE_BAND)
        return 0;                                   /* raised deck/roof     */
    return 1;
}

static void classify_parts(SceneObj *o)
{
    for (int pi = 0; pi < o->nparts; pi++)
        o->parts[pi].drive_surf =
            (unsigned char)part_classify_drive_surface(o, pi);
}

/* Horizontal circle used by native FUN_00423690's first-stage reject.
 * Centre is the placed object's own XZ and radius encloses every transformed
 * part vertex (bbox corners suffice for a rigid transform). */
static int object_horizontal_radius(const SceneObj *o, double *out)
{
    double best2 = 0.0;
    int any = 0;
    for (int pi = 0; pi < o->nparts; pi++) {
        int slot = o->parts[pi].mesh;
        GeoMesh *m = slot >= 0 && slot < s_nmesh ? s_mesh[slot].mesh : NULL;
        if (!m || m->num_verts <= 0) continue;
        Xform xf = xform_compose(&o->world, &o->parts[pi].local);
        for (int c = 0; c < 8; c++) {
            float v[3] = { (c & 1) ? m->bb_max[0] : m->bb_min[0],
                           (c & 2) ? m->bb_max[1] : m->bb_min[1],
                           (c & 4) ? m->bb_max[2] : m->bb_min[2] };
            double w[3];
            xform_apply(&xf, v, w);
            double dx = w[0] - o->world.t[0];
            double dz = w[2] - o->world.t[2];
            double d2 = dx * dx + dz * dz;
            if (d2 > best2) best2 = d2;
            any = 1;
        }
    }
    if (!any) return -1;
    *out = sqrt(best2);
    return 0;
}

/* Register one placed static exactly at the decoded ownership boundary:
 * class 11/13 admits every SGEO part; class 12 admits only pclass-13 deck
 * parts. World-space face normal.y must exceed 0.4. The registry copies the
 * polygon, so it safely outlives scene mesh-cache teardown ordering. */
static int register_drivable_object(SceneObj *o)
{
    uint32_t cls = o->sdf_class >= 11 && o->sdf_class <= 13
                 ? o->sdf_class : o->class_id;
    if (cls < 11 || cls > 13) return 0;

    double radius;
    if (object_horizontal_radius(o, &radius) != 0) return 0;
    int object = -1;
    for (int pi = 0; pi < o->nparts; pi++) {
        PartInst *part = &o->parts[pi];
        if (cls == 12 && part->pclass != 13) continue;
        int slot = part->mesh;
        GeoMesh *m = slot >= 0 && slot < s_nmesh ? s_mesh[slot].mesh : NULL;
        if (!m || m->num_faces <= 0 || !m->face_plane) continue;
        Xform xf = xform_compose(&o->world, &part->local);

        for (int fi = 0; fi < m->num_faces; fi++) {
            int nv = m->face_count[fi];
            int first = m->face_first[fi];
            if (nv < 3 || first < 0 || first > m->num_indices - nv) continue;

            const float *lp = &m->face_plane[fi * 4];
            double normal[3];
            for (int k = 0; k < 3; k++)
                normal[k] = xf.m[k][0] * lp[0] +
                            xf.m[k][1] * lp[1] +
                            xf.m[k][2] * lp[2];
            double nl = sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                             normal[2] * normal[2]);
            if (!(nl > 1e-12)) continue;
            for (int k = 0; k < 3; k++) normal[k] /= nl;
            /* SGEO class 13 is the native class-12 object's explicit DECK
             * child. Some shipped bridge deck OEGs store their planar face
             * with the renderer's measured inward normal (all qualifying
             * EB*_ZRD* faces are -Y); orient that same plane to the deck's
             * upward side before applying FUN_00423550's ny > 0.4 gate.
             * Class-11 ramps are not sign-corrected: their underside remains
             * rejected and their authored +Y top is the only admitted face. */
            if (cls == 12 && part->pclass == 13 && normal[1] < -0.4)
                for (int k = 0; k < 3; k++) normal[k] = -normal[k];
            if (!(normal[1] > 0.4)) continue;

            double *xz = malloc((size_t)nv * 2 * sizeof *xz);
            if (!xz) return -1;
            double first_world[3] = {0.0, 0.0, 0.0};
            int valid = 1;
            for (int vi = 0; vi < nv; vi++) {
                int idx = m->indices[first + vi];
                if (idx < 0 || idx >= m->num_verts) { valid = 0; break; }
                double w[3];
                xform_apply(&xf, &m->verts[idx * 3], w);
                if (vi == 0) memcpy(first_world, w, sizeof w);
                xz[vi * 2] = w[0];
                xz[vi * 2 + 1] = w[2];
            }
            if (!valid) { free(xz); continue; }
            double plane[4] = { normal[0], normal[1], normal[2],
                -(normal[0] * first_world[0] + normal[1] * first_world[1] +
                  normal[2] * first_world[2]) };
            if (object < 0) {
                object = terrain_drivable_object_add(o->world.t[0],
                                                      o->world.t[2], radius);
                if (object < 0) { free(xz); return -1; }
            }
            if (terrain_drivable_face_add(object, plane, xz, nv) != 0) {
                free(xz);
                return -1;
            }
            free(xz);
        }
    }
    if (object >= 0) o->drivable_object = object + 1;
    return 0;
}

static int register_drivable_faces(void)
{
    terrain_drivable_clear();
    for (int i = 0; i < s_nobj; i++)
        if (register_drivable_object(&s_objs[i]) != 0) {
            terrain_drivable_clear();
            return -1;
        }
    return 0;
}

static void object_track_extent(const Xform *w)
{
    for (int i = 0; i < 3; i++) {
        if (w->t[i] < s_min[i]) s_min[i] = w->t[i];
        if (w->t[i] > s_max[i]) s_max[i] = w->t[i];
    }
}

static void object_commit(SceneObj *o)
{
    if (s_nobj == s_objcap) {
        int ncap = s_objcap ? s_objcap * 2 : 64;
        SceneObj *no = realloc(s_objs, (size_t)ncap * sizeof *no);
        if (!no) return;
        s_objs = no;
        s_objcap = ncap;
    }
    object_track_extent(&o->world);
    classify_parts(o);          /* D11: flags ride the PartInst, computed
                                 * once here (and in scene_obj_make_car) so
                                 * the per-frame render loop just reads them */
    s_objs[s_nobj++] = *o;
}

/* ODEF OBJ record, 100-byte payload (scene.md §3.1). */
static void odef_obj(const uint8_t *p, size_t len)
{
    if (len < 100) return;
    SceneObj o;
    memset(&o, 0, sizeof o);
    label_unpack(p, o.label, &o.label_id);
    o.world    = xform_from_frame(p + 0x08);
    o.class_id = rd_u32(p + 0x5C);
    o.flags    = rd_u16(p + 0x60);
    o.team     = rd_u16(p + 0x62);

    if (o.class_id == 1 && label_is_marker(o.label)) {
        s_n_markers++;
    } else if (o.class_id == 9) {
        s_n_skipped++;                          /* script-only (scene.md §3.3) */
    } else if (o.class_id == 1) {
        build_car(&o);
        s_n_cars++;
    } else {
        build_static(&o);                       /* classes 2,3,4,11,80,82 (D7) */
        s_n_statics++;
    }

    /* SPAWN rows are placement slots, not scene/render objects. Melee host
     * setup consumes them later; printing six part-less placeholders on every
     * load is misleading noise in browser diagnostics. Keep REGEN/CHECK and
     * all materialized objects visible in the scene log. */
    if (!(o.class_id == 1 && strcasecmp(o.label, "SPAWN") == 0))
        fprintf(stdout,
                "[scene] obj '%s'#%d cls=%u flags=0x%02x team=%u "
                "pos=(%.1f, %.1f, %.1f) parts=%d\n",
                o.label, o.label_id, o.class_id, o.flags, o.team,
                o.world.t[0], o.world.t[1], o.world.t[2], o.nparts);
    object_commit(&o);
}

/* LDEF OBJ record (scene.md §4): label[8], skip 84, classId, u32, count,
 * count x vec3 positions. One SDF model instanced along the polyline,
 * yaw-rotated to face the next point (D10). */
static void ldef_obj(const uint8_t *p, size_t len)
{
    if (len < 108) return;
    char label[9];
    memcpy(label, p, 8);
    label[8] = '\0';
    uint32_t count = rd_u32(p + 100);
    if (len < 104 + (size_t)count * 12) return;

    uint32_t class_id = rd_u32(p + 92);
    for (uint32_t i = 0; i < count; i++) {
        const uint8_t *q = p + 104 + (size_t)i * 12;
        double px = rd_f32(q), py = rd_f32(q + 4), pz = rd_f32(q + 8);
        /* yaw toward the next point (last reuses the previous segment) */
        uint32_t a = i, b = i + 1 < count ? i + 1 : i;
        if (i == count - 1 && i > 0) a = i - 1;
        const uint8_t *qa = p + 104 + (size_t)a * 12;
        const uint8_t *qb = p + 104 + (size_t)b * 12;
        double dx = (double)rd_f32(qb) - rd_f32(qa);
        double dz = (double)rd_f32(qb + 8) - rd_f32(qa + 8);
        double yaw = (dx || dz) ? atan2(dx, dz) : 0.0;

        SceneObj inst;
        memset(&inst, 0, sizeof inst);
        snprintf(inst.label, sizeof inst.label, "%s", label);
        inst.class_id = class_id;
        inst.world = xform_identity();
        double cy = cos(yaw), sy = sin(yaw);
        inst.world.m[0][0] = cy;  inst.world.m[0][2] = sy;
        inst.world.m[2][0] = -sy; inst.world.m[2][2] = cy;
        inst.world.t[0] = px; inst.world.t[1] = py; inst.world.t[2] = pz;
        build_static(&inst);        /* mesh_get caches; parts array is fresh */
        s_n_strings++;
        object_commit(&inst);
    }
}

/* Walk one *DEF payload's sub-chunk stream (OREV/LREV, OBJ..., EXIT). */
static void def_walk(const uint8_t *b, size_t len, int is_odef)
{
    for (size_t off = 0; off < len; ) {
        Chunk c;
        if (!chunk_at(b, len, off, &c)) break;
        if (tag_is(&c, "OBJ\0")) {
            if (is_odef) odef_obj(b + c.payload, c.total - 8);
            else         ldef_obj(b + c.payload, c.total - 8);
        } else if (tag_is(&c, "EXIT")) {
            break;
        }
        off = c.next;
    }
}

int scene_load(const char *name)
{
    scene_unload();

    /* Missions are loose files; the original resolves .msn via miss8/
     * (miss16 for the Glide tier). Try the plain name first. */
    char path[64];
    size_t sz = 0;
    uint8_t *buf = NULL;
    const char *dirs[] = { "", "miss8/", "miss16/", "missions/" };
    for (size_t i = 0; i < sizeof dirs / sizeof dirs[0] && !buf; i++) {
        snprintf(path, sizeof path, "%s%s", dirs[i], name);
        buf = vfs_read_file(path, &sz);
    }
    if (!buf) {
        fprintf(stderr, "[scene] mission not found: %s\n", name);
        return -1;
    }
    copy_str(s_mission, sizeof s_mission, path);

    s_min[0] = s_min[1] = s_min[2] =  1e30;
    s_max[0] = s_max[1] = s_max[2] = -1e30;

    /* Top-level chunk stream: BWD2/REV lead chunks, then
     * WDEF TDEF RDEF ODEF LDEF ADEF EXIT (scene.md §1.1). */
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        if (tag_is(&c, "ODEF"))
            def_walk(buf + c.payload, c.total - 8, 1);
        else if (tag_is(&c, "LDEF"))
            def_walk(buf + c.payload, c.total - 8, 0);
        else if (tag_is(&c, "WDEF"))
            wrld_assets_load(buf + c.payload, c.total - 8);
        else if (tag_is(&c, "EXIT"))
            break;
        /* TDEF/RDEF/ADEF: not this module (D8) */
        off = c.next;
    }
    vfs_free(buf);

    if (s_nobj == 0) {
        fprintf(stderr, "[scene] %s: no objects placed\n", s_mission);
        return -1;
    }

    for (int i = 0; i < 3; i++)
        s_center[i] = 0.5 * (s_min[i] + s_max[i]);

    if (register_drivable_faces() != 0) {
        fprintf(stderr, "[scene] %s: drivable-face registration failed\n",
                s_mission);
        scene_unload();
        return -1;
    }

    s_loaded = 1;
    fprintf(stdout,
            "[scene] %s: %d objects (%d cars, %d statics, %d string, "
            "%d markers, %d skipped), meshes %d ok %d missing\n",
            s_mission, s_nobj, s_n_cars, s_n_statics, s_n_strings,
            s_n_markers, s_n_skipped, s_mesh_ok, s_mesh_fail);
    return 0;
}

/* ----------------------------------------------------------------------- */
/* Renderer (own small Bresenham — meshview's is static to its file)         */
/* ----------------------------------------------------------------------- */

#define IDX_BG   0
#define IDX_WIRE 1
#define IDX_VERT 2

#define CAM_ELEV 0.35                           /* rad, fixed (D9) */
#define CAM_NEAR 0.5

static void draw_line(uint8_t *fb, int w, int h,
                      int x0, int y0, int x1, int y1, uint8_t col)
{
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if ((unsigned)x0 < (unsigned)w && (unsigned)y0 < (unsigned)h)
            fb[(size_t)y0 * (size_t)w + x0] = col;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void put_dot(uint8_t *fb, int w, int h, int x, int y, uint8_t col)
{
    for (int yy = y - 1; yy <= y + 1; yy++)
        for (int xx = x - 1; xx <= x + 1; xx++)
            if ((unsigned)xx < (unsigned)w && (unsigned)yy < (unsigned)h)
                fb[(size_t)yy * (size_t)w + xx] = col;
}

static void vec_sub(const double a[3], const double b[3], double *o)
{
    o[0] = a[0] - b[0]; o[1] = a[1] - b[1]; o[2] = a[2] - b[2];
}

static double vec_dot(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void vec_cross(const double a[3], const double b[3], double *o)
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

static void vec_norm(double *v)
{
    double l = sqrt(vec_dot(v, v));
    if (l > 1e-12) { v[0] /= l; v[1] /= l; v[2] /= l; }
}

static void render_objects(uint8_t *fb, int w, int h, const double eye[3],
                           const double fwd[3], const double rgt[3],
                           const double up[3], double focal);

void scene_render(uint8_t *fb, int w, int h, double yaw, double dist)
{
    memset(fb, IDX_BG, (size_t)w * (size_t)h);
    if (!s_loaded) return;

    /* auto-fit: distance so the object-position bounding disc fits the FOV */
    if (dist <= 0.0) {
        double r = 0.0;
        for (int i = 0; i < s_nobj; i++) {
            double dx = s_objs[i].world.t[0] - s_center[0];
            double dz = s_objs[i].world.t[2] - s_center[2];
            double d = sqrt(dx * dx + dz * dz);
            if (d > r) r = d;
        }
        dist = 1.1 * (r + 40.0) / RASTER_FOV_TAN_HALF;
    }

    double eye[3] = {
        s_center[0] + dist * cos(CAM_ELEV) * sin(yaw),
        s_center[1] + dist * sin(CAM_ELEV),
        s_center[2] + dist * cos(CAM_ELEV) * cos(yaw),
    };
    double fwd[3], rgt[3], up[3], wup[3] = {0.0, 1.0, 0.0};
    vec_sub(s_center, eye, fwd); vec_norm(fwd);
    /* right = cross(world_up, fwd) — matches terrain.c render_perspective_body
     * and web_gpu_project. cross(fwd, world_up) is its NEGATION and yields a
     * LEFT-handed basis (right x up = -fwd), which mirrored every scene object
     * in X against the terrain drawn into the same framebuffer. See the D4
     * note above: object bases are right-handed, and so is the camera. */
    vec_cross(wup, fwd, rgt);    vec_norm(rgt);
    vec_cross(fwd, rgt, up);

    double focal = (w * 0.5) / RASTER_FOV_TAN_HALF;
    render_objects(fb, w, h, eye, fwd, rgt, up, focal);
}

/*
 * Draw one mesh wireframe with world transform `xf` through the camera
 * (eye, fwd/rgt/up basis, focal length). The projection scratch buffers are
 * function-static and grow to the largest mesh seen (meshview-style).
 */
static void draw_one_mesh(uint8_t *fb, int w, int h, const GeoMesh *m,
                          const Xform *xf, const double eye[3],
                          const double fwd[3], const double rgt[3],
                          const double up[3], double focal)
{
    static float  *px, *py;
    static int    *ok;
    static int     cap;

    if (!m || m->num_verts == 0) return;
    if (m->num_verts > cap) {
        int ncap = m->num_verts;
        float *npx = realloc(px, (size_t)ncap * sizeof *npx);
        float *npy = realloc(py, (size_t)ncap * sizeof *npy);
        int   *nok = realloc(ok, (size_t)ncap * sizeof *nok);
        if (!npx || !npy || !nok) { free(npx); free(npy); free(nok); return; }
        px = npx; py = npy; ok = nok; cap = ncap;
    }

    {
            for (int v = 0; v < m->num_verts; v++) {
                double wp[3], rel[3];
                xform_apply(xf, &m->verts[v * 3], wp);
                vec_sub(wp, eye, rel);
                double cz = vec_dot(rel, fwd);
                if (cz > CAM_NEAR) {
                    double cx = vec_dot(rel, rgt);
                    double cy = vec_dot(rel, up);
                    px[v] = (float)(w * 0.5 + focal * cx / cz);
                    py[v] = (float)(h * 0.5 - focal * cy / cz);
                    ok[v] = 1;
                } else {
                    ok[v] = 0;
                }
            }
            /* vertex dots only when the part reads large enough on screen;
             * at scene scale 3x3 dots would drown the wireframe */
            float pminx = 1e30f, pmaxx = -1e30f, pminy = 1e30f, pmaxy = -1e30f;
            int nvis = 0;
            for (int v = 0; v < m->num_verts; v++) {
                if (!ok[v]) continue;
                nvis++;
                if (px[v] < pminx) pminx = px[v];
                if (px[v] > pmaxx) pmaxx = px[v];
                if (py[v] < pminy) pminy = py[v];
                if (py[v] > pmaxy) pmaxy = py[v];
            }
            int draw_dots = nvis > 0 &&
                (pmaxx - pminx > 10.0f || pmaxy - pminy > 10.0f);

            for (int f = 0; f < m->num_faces; f++) {
                int first = m->face_first[f];
                int n     = m->face_count[f];
                for (int k = 0; k < n; k++) {
                    int a = m->indices[first + k];
                    int b = m->indices[first + (k + 1) % n];
                    if (!ok[a] || !ok[b]) continue;
                    draw_line(fb, w, h,
                              (int)lroundf(px[a]), (int)lroundf(py[a]),
                              (int)lroundf(px[b]), (int)lroundf(py[b]),
                              IDX_WIRE);
                }
            }
            if (draw_dots)
                for (int v = 0; v < m->num_verts; v++)
                    if (ok[v])
                        put_dot(fb, w, h, (int)lroundf(px[v]),
                                (int)lroundf(py[v]), IDX_VERT);
    }
}

/* -----------------------------------------------------------------------
 * M8 filled path (docs/specs/m8/software-raster.md)
 * ----------------------------------------------------------------------- */

/*
 * Sun direction (world space) and ambient floor.
 *
 * BOTH ARE PLACEHOLDERS. The spec's §1 position is "invent the loop, reverse
 * the decisions", and the light vector is explicitly on the reverse-me list
 * along with the ambient term and how face_rgb is modulated. This is a
 * plausible high desert sun, not a measurement, and it is isolated here so
 * that replacing it is a two-line change. Unit length by construction:
 * (-0.5, 0.9, -0.4) / |(-0.5, 0.9, -0.4)|.
 *
 * H-UAT-013 recalibration (still a placeholder, now calibrated against the
 * admitted native captures rather than guessed): at ambient 0.35 every face
 * whose normal points away from the invented sun rendered at light 0.35,
 * which the authored .lum ramps remap to their near-black rows — vehicles
 * read as "2-3 dark pixels" at range and shadow-side mesa statics as solid
 * black. docs/evidence/m4-oracle-gameplay-p01.png shows shadow-side faces
 * warm and textured, never black, so the ambient floor is raised to match
 * that observation. The directional term keeps the same sun vector.
 */
static const double SUN_DIR[3] = { -0.45267873, 0.81482171, -0.36214298 };
#define SCENE_AMBIENT 0.60

/* Faces skipped because they are cut-out/see-through and V1 has no textures
 * to key against. Counted, never silently dropped (spec §"open questions"). */
static long s_filled_cutout_skipped;
static long s_filled_faces, s_filled_backfaced;
/* M8 V2 step 6: faces whose texture name RESOLVED to a tile. Nothing samples
 * yet -- this is what makes "the plumbing works" observable in the same frame
 * whose hash must not move. */
static long s_filled_textured;

/*
 * Resolve this mesh's per-face palette indices, rebuilding if the level
 * palette changed since they were built.
 */
static const uint8_t *mesh_face_indices(MeshSlot *s)
{
    const GeoMesh *m = s->mesh;
    if (!m || m->num_faces <= 0 || !m->face_rgb) return NULL;

    unsigned gen = raster_palette_generation();
    if (s->face_idx && s->face_idx_gen == gen) return s->face_idx;
    if (!s->face_idx) {
        s->face_idx = malloc((size_t)m->num_faces);
        if (!s->face_idx) return NULL;
    }
    for (int f = 0; f < m->num_faces; f++)
        s->face_idx[f] = raster_rgb_to_index(m->face_rgb[f * 3 + 0],
                                             m->face_rgb[f * 3 + 1],
                                             m->face_rgb[f * 3 + 2]);
    s->face_idx_gen = gen;
    return s->face_idx;
}

/*
 * Resolve this mesh's per-face tile ids under paint scheme `vtf`, building the
 * node on first use and rebuilding it if the texture cache was reset.
 *
 * Resolution happens ONCE PER MESH PER SCHEME, not per face per frame: the
 * paint chain reads the .vtf and a .tmt out of the VFS, which is far too
 * expensive to sit in a render loop. texcache memoizes on top of this anyway;
 * this array exists so the common case is an indexed load rather than a hash
 * lookup.
 */
static const uint16_t *mesh_face_tiles(MeshSlot *s, const char *vtf)
{
    const GeoMesh *m = s->mesh;
    if (!m || m->num_faces <= 0 || !m->face_tex) return NULL;
    if (!vtf) vtf = "";

    unsigned gen = texcache_generation();
    TileSet *ts = NULL;
    for (ts = s->tiles; ts; ts = ts->next)
        if (strcmp(ts->vtf, vtf) == 0) break;

    if (ts && ts->gen == gen) return ts->ids;

    if (!ts) {
        ts = calloc(1, sizeof *ts);
        if (!ts) return NULL;
        copy_str(ts->vtf, sizeof ts->vtf, vtf);
        ts->next = s->tiles;
        s->tiles = ts;
    }
    if (!ts->ids) {
        ts->ids = malloc((size_t)m->num_faces * sizeof *ts->ids);
        if (!ts->ids) return NULL;
    }
    for (int f = 0; f < m->num_faces; f++)
        ts->ids[f] = texcache_resolve(&m->face_tex[(size_t)f * GEO_TEX_NAME_LEN],
                                      vtf);
    ts->gen = gen;
    return ts->ids;
}

/*
 * Tile ids for the driver-supplied dynamic meshes, which have no MeshSlot.
 * One cache per queue slot: the drive render pushes the same parts in the
 * same order every frame, so a slot's resolve runs once per mission (or
 * texcache generation), never per frame.
 *
 * The identity check uses the mesh POINTER, which is safe here for a reason
 * worth stating: it is an equality test for invalidation, never an ordering or
 * a hash. Nothing iterates it, so ASLR cannot reorder anything, and a recycled
 * allocation at the same address would carry the same face table by
 * construction -- the driver hands over a mesh owned by meshcache, keyed by
 * name.
 */
typedef struct {
    const GeoMesh *mesh;
    char           vtf[16];
    unsigned       gen;
    uint16_t      *ids;
    int            faces;
} DynTileCache;

static DynTileCache s_dyn_tiles[SCENE_DYN_MAX];

static const uint16_t *dyn_face_tiles(int slot)
{
    const GeoMesh *m = s_dyn_mesh[slot];
    if (!m || m->num_faces <= 0 || !m->face_tex) return NULL;

    DynTileCache *c = &s_dyn_tiles[slot];
    unsigned gen = texcache_generation();
    if (c->ids && c->mesh == m && c->gen == gen && c->faces == m->num_faces &&
        strcmp(c->vtf, s_dyn_vtf) == 0)
        return c->ids;

    if (c->faces != m->num_faces || !c->ids) {
        uint16_t *p = realloc(c->ids, (size_t)m->num_faces * sizeof *p);
        if (!p) return NULL;
        c->ids = p;
        c->faces = m->num_faces;
    }
    for (int f = 0; f < m->num_faces; f++)
        c->ids[f] = texcache_resolve(
            &m->face_tex[(size_t)f * GEO_TEX_NAME_LEN], s_dyn_vtf);

    c->mesh = m;
    c->gen = gen;
    copy_str(c->vtf, sizeof c->vtf, s_dyn_vtf);
    return c->ids;
}

/*
 * Draw one mesh filled.
 *
 * The face normal is derived from the TRANSFORMED world-space vertices rather
 * than by transforming the stored face_plane. That sidesteps the spec's open
 * question "are object transforms always rigid (else normals need
 * inverse-transpose)?" entirely: a normal computed from transformed positions
 * is correct under any affine transform, rigid or not. tools/geo_survey.c
 * confirmed the stored plane normal agrees with this geometric normal on all
 * 33035 faces in the game, so nothing is lost by recomputing it.
 */
static void draw_one_mesh_filled(RTarget *t, const GeoMesh *m, const Xform *xf,
                                 const double eye[3], const double fwd[3],
                                 const double rgt[3], const double up[3],
                                 const uint8_t *face_idx,
                                 const uint16_t *face_tile,
                                 const RTex *face_override,
                                 const char *history_layer)
{
    static double *wp;      /* world positions, grown to the largest mesh */
    static int     cap;

    if (!m || m->num_verts == 0 || m->num_faces == 0) return;
    if (m->num_verts > cap) {
        double *n = realloc(wp, (size_t)m->num_verts * 3 * sizeof *n);
        if (!n) return;
        wp = n; cap = m->num_verts;
    }
    for (int v = 0; v < m->num_verts; v++)
        xform_apply(xf, &m->verts[v * 3], &wp[v * 3]);

    for (int f = 0; f < m->num_faces; f++) {
        int first = m->face_first[f];
        int n = m->face_count[f];
        if (n < 3 || n > RASTER_CLIP_MAX - 6) continue;
        s_filled_faces++;

        const RTex *tex = face_override;
        if (!tex && face_tile && face_tile[f] != TEX_ID_NONE)
            tex = texcache_tile(face_tile[f]);

        /*
         * Cut-out faces (fences, rails, headlight glass) draw as of step 9 --
         * but ONLY with a tile to key against. All 2603 of them carry a texture
         * name, so one without a resolved tile is an asset miss, and drawing it
         * opaque would put a solid wall where a chain-link fence belongs: the
         * worse of the two errors in a driving game. The skip therefore
         * survives as the no-texture fallback rather than as the rule.
         */
        int cutout = m->face_flags &&
                     GEO_FLAG2_CUTOUT(m->face_flags[f * 3 + 1]);
        if (cutout && !tex) {
            s_filled_cutout_skipped++;
            continue;
        }

        const double *a = &wp[m->indices[first + 0] * 3];
        const double *b = &wp[m->indices[first + 1] * 3];
        const double *c = &wp[m->indices[first + 2] * 3];
        double e1[3], e2[3], nrm[3];
        vec_sub(b, a, e1);
        vec_sub(c, a, e2);
        vec_cross(e1, e2, nrm);
        double nl = sqrt(vec_dot(nrm, nrm));
        if (nl < 1e-12) continue;              /* degenerate after transform */
        nrm[0] /= nl; nrm[1] /= nl; nrm[2] /= nl;

        /* OEG corners are clockwise as seen from outside, and their stored
         * plane normals point inward (gpu_scene.mjs uses the same measured
         * convention).  An outside-facing polygon therefore has its normal
         * pointing away from the eye. */
        double look[3];
        vec_sub(eye, a, look);
        if (vec_dot(nrm, look) >= 0.0) {
            s_filled_backfaced++;
            continue;
        }

        nrm[0] = -nrm[0]; nrm[1] = -nrm[1]; nrm[2] = -nrm[2];
        double lit = vec_dot(nrm, SUN_DIR);
        if (lit < 0.0) lit = 0.0;
        double light = SCENE_AMBIENT + (1.0 - SCENE_AMBIENT) * lit;

        RVert poly[RASTER_CLIP_MAX];
        for (int k = 0; k < n; k++) {
            int vi = m->indices[first + k];
            double rel[3];
            vec_sub(&wp[vi * 3], eye, rel);
            poly[k].x = vec_dot(rel, rgt);
            poly[k].y = vec_dot(rel, up);
            poly[k].z = vec_dot(rel, fwd);
            /* Carried, not yet consumed at the fragment stage (spec §10). */
            poly[k].u = m->uvs ? m->uvs[(first + k) * 2 + 0] : 0.0f;
            poly[k].v = m->uvs ? m->uvs[(first + k) * 2 + 1] : 0.0f;
            poly[k].light = (float)light;
            poly[k].fog = 0.0f;
        }
        /* No cached array (the dynamic mesh isn't in the slot table): resolve
         * this face directly rather than falling back to a fixed index, which
         * would paint the player's car one flat colour. Bounded by one mesh
         * per frame. */
        uint8_t base = 1;
        if (face_idx) base = face_idx[f];
        else if (m->face_rgb) base = raster_rgb_to_index(m->face_rgb[f * 3 + 0],
                                                         m->face_rgb[f * 3 + 1],
                                                         m->face_rgb[f * 3 + 2]);

        /*
         * Index 0xFF means two different things one argument apart: on a
         * cut-out face the keyed drawer discards it with NO DEPTH WRITE; on an
         * opaque face the unkeyed drawer consumes ordinary palette index 255.
         * Nitro's split is confirmed in phase-d-renderer.md §3.1; substituting
         * the face-flat colour here was H-UAT-063's grey->yellow defect.
         */
        if (tex) s_filled_textured++;
        raster_pixel_history_draw(t, f, m->name, history_layer, "world");
        if (tex) {
            const char *texture_asset = "";
            if (!face_override && face_tile && face_tile[f] != TEX_ID_NONE)
                texture_asset = texcache_tile_name(face_tile[f]);
            if (!face_override && !texture_asset[0] && m->face_tex)
                texture_asset = &m->face_tex[(size_t)f * GEO_TEX_NAME_LEN];
            /* A dynamic paint override has no asset name in RTex.  Unknown is
             * truthful; the mesh face name identifies a different texture. */
            raster_pixel_history_texture(t, texture_asset);
        }
        raster_polygon_tex(t, poly, n, base, tex, cutout);
    }
}

/* -----------------------------------------------------------------------
 * H-UAT-013 distant-vehicle legibility — minimum-screen-size impostor.
 *
 * PORT CONVENTION (INVENTED thresholds), not fidelity: at 640x480 with the
 * binary-derived 90-degree FOV a car (~1.5 m tall, ~5 m long) projects
 * roughly 3x1 px at 500 m, and faces that small mostly fail the kernel's
 * pixel-centre coverage — P01's scripted battle beyond ~300 m rendered as
 * 2-3 dark pixels. No LOD/impostor system is documented for nitro.exe
 * (docs/specs/re/phase-d-renderer.md has no such path), so this is a marked
 * port convention: a placed CAR-CLASS object whose projected height falls
 * under SCENE_IMP_MIN_H_PX draws a depth-tested block at a minimum screen
 * size instead of its subpixel mesh. Everything ELSE about the block is
 * authored data, not invention: its colour is the object's own dominant
 * face colour (face_rgb through the level palette, unshaded), its width
 * tracks the object's real world bounds (a tanker truck reads wider than a
 * coupe), and it depth-tests against the frame like any geometry, so mesas
 * still occlude it. Statics/markers never get impostors, and the ODEF
 * player-flagged record (flags & 0x10, mission.c D5) is skipped — the
 * player's own car is the sim-driven dynamic mesh.
 * ----------------------------------------------------------------------- */
#define SCENE_IMP_MIN_H_PX 3.0  /* impostor below this projected height   */
#define SCENE_IMP_MAX_W_PX 5    /* clamps: legible, never a smeared blob  */
#define SCENE_IMP_MAX_H_PX 3
#define SCENE_IMP_MIN_Z_M  60.0 /* subpixel projection is impossible this
                                 * close; guards the divide as a bonus     */

/* Dominant authored face colour of the object, resolved lazily once per
 * palette generation. 0 (the colour-key index) doubles as "none". */
static uint8_t object_impostor_color(SceneObj *o)
{
    unsigned gen = raster_palette_generation();
    if (o->imp_gen == gen)
        return o->imp_idx;
    int cnt[256] = {0};
    for (int pi = 0; pi < o->nparts; pi++) {
        MeshSlot *s = &s_mesh[o->parts[pi].mesh];
        const uint8_t *idx = mesh_face_indices(s);
        if (!idx) continue;
        for (int f = 0; f < s->mesh->num_faces; f++)
            if (idx[f] != 0) cnt[idx[f]]++;
    }
    /* cnt[0] is never incremented, so a plain strict argmax both finds the
     * mode and leaves best == 0 ("none") when no face resolved; ties break
     * to the lowest index, deterministically. */
    int best = 0;
    for (int i = 1; i < 256; i++)
        if (cnt[i] > cnt[best]) best = i;
    o->imp_idx = (uint8_t)best;
    o->imp_gen = gen;
    return o->imp_idx;
}

static void draw_vehicle_impostors(RTarget *t, const CameraView *camera)
{
    const double *eye = camera->eye;
    for (int i = 0; i < s_nobj; i++) {
        SceneObj *o = &s_objs[i];
        if (o->class_id != 1 || o->hidden || o->consumed || o->nparts == 0)
            continue;
        if (o->flags & 0x10)
            continue;                   /* the player's ODEF record (D5) */

        double lo[3] = { 1e30, 1e30, 1e30 };
        double hi[3] = { -1e30, -1e30, -1e30 };
        int have = 0;
        for (int pi = 0; pi < o->nparts; pi++) {
            double plo[3], phi[3];
            if (part_world_aabb(o, pi, plo, phi) != 0)
                continue;
            for (int k = 0; k < 3; k++) {
                if (plo[k] < lo[k]) lo[k] = plo[k];
                if (phi[k] > hi[k]) hi[k] = phi[k];
            }
            have = 1;
        }
        if (!have)
            continue;

        double ctr[3] = { 0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]),
                          0.5 * (lo[2] + hi[2]) };
        double rel[3];
        vec_sub(ctr, eye, rel);
        double z = vec_dot(rel, camera->forward);
        if (z < SCENE_IMP_MIN_Z_M || z > t->zfar)
            continue;
        double h_px = t->f * (hi[1] - lo[1]) / z;
        if (h_px >= SCENE_IMP_MIN_H_PX)
            continue;                   /* the mesh itself is legible */

        uint8_t color = object_impostor_color(o);
        if (color == 0)
            continue;                   /* no authored colour resolved */
        uint32_t d = raster_depth_for_z(t, z);
        if (d == 0u)
            continue;

        double spanx = hi[0] - lo[0], spanz = hi[2] - lo[2];
        double w_m = spanx > spanz ? spanx : spanz;
        int wi = (int)lround(t->f * w_m / z);
        int hgt = (int)lround(h_px);
        if (wi < 3) wi = 3;     /* min 3x2: a 2x2 dot vanishes into the
                                 * terrain speckle at 800 m (measured) */
        if (wi > SCENE_IMP_MAX_W_PX) wi = SCENE_IMP_MAX_W_PX;
        if (hgt < 2) hgt = 2;
        if (hgt > SCENE_IMP_MAX_H_PX) hgt = SCENE_IMP_MAX_H_PX;

        int sx = (int)lround(t->cx + t->f * vec_dot(rel, camera->right) / z);
        /* Anchor the block's BOTTOM row at the projected top of the bbox
         * and extend upward. The block is already a minimum-size
         * convention (the real subtended height is under a pixel), and at
         * the grazing/overlook sightlines it exists for, every row below
         * the silhouette top projects onto NEARER ground, which correctly
         * wins the depth test and would eat the block; rows above it land
         * on farther terrain or sky, where the same depth test keeps the
         * vehicle visible — and still hides it behind a true foreground
         * occluder like a mesa. */
        double top[3] = { ctr[0], hi[1], ctr[2] };
        vec_sub(top, eye, rel);
        int sy = (int)lround(t->cy - t->f * vec_dot(rel, camera->up) / z);
        for (int y = sy - hgt + 1; y <= sy; y++) {
            if ((unsigned)y >= (unsigned)t->h) continue;
            for (int x = sx - wi / 2; x < sx - wi / 2 + wi; x++) {
                if ((unsigned)x >= (unsigned)t->w) continue;
                size_t px = (size_t)y * t->w + x;
                uint32_t prior = t->depth[px];
                int wins = d > prior;       /* the kernel's strict rule */
                raster_pixel_history_direct(t, i, o->label,
                                            "vehicle-impostor", "world",
                                            x, y, z, d, prior, wins,
                                            "depth_test");
                if (wins) {
                    t->depth[px] = d;
                    t->color[px] = color;
                }
            }
        }
    }
}

void scene_render_filled(RTarget *t, const CameraView *camera)
{
    if (!t || !t->color || !camera || !camera_view_valid(camera))
        return;
    /* Dyn-only is valid without a loaded mission (garage preview). Placed
     * objects still require s_loaded. */
    if (!s_loaded && s_dyn_count == 0)
        return;

    /* Per-FRAME counters, not lifetime totals: a stat that only ever grows
     * cannot answer "did this frame draw the objects?". */
    s_filled_faces = s_filled_backfaced = s_filled_cutout_skipped = 0;
    s_filled_textured = 0;

    const double *eye = camera->eye;
    const double *fwd = camera->forward;
    const double *rgt = camera->right;
    const double *up = camera->up;

    if (s_loaded) {
        for (int i = 0; i < s_nobj; i++) {
            SceneObj *o = &s_objs[i];
            if (o->hidden || o->consumed) continue;
            for (int pi = 0; pi < o->nparts; pi++) {
                if (!part_active(&o->parts[pi])) continue;
                MeshSlot *slot = &s_mesh[o->parts[pi].mesh];
                Xform xf = xform_compose(obj_draw_world(o),
                                         &o->parts[pi].local);
                /* D11 render-only lift for classified ground surfaces: keeps a
                 * terrain-conformed slab (P01's gas-station lot) from trading
                 * reciprocal-depth ties with the heightfield underneath it.
                 * Same 0.1 m as terrain.c ROAD_LIFT_M, applied per part — the
                 * raster's global strict-depth rule is untouched, and the
                 * collision build skips exactly these parts via the same
                 * classification, so physics never sees the lift. */
                if (o->parts[pi].drive_surf)
                    xf.t[1] += SCENE_SURFACE_LIFT_M;
                draw_one_mesh_filled(t, slot->mesh, &xf, eye, fwd, rgt, up,
                                     mesh_face_indices(slot),
                                     mesh_face_tiles(slot, o->vtf), NULL,
                                     "scene-object");
            }
        }
    }
    /* The dynamic meshes (the player's car) are not in the slot table, so
     * they have no cached index arrays and resolve their faces inline. Their
     * tiles come from the paint scheme the driver supplied with the meshes. */
    for (int i = 0; i < s_dyn_count; i++)
        draw_one_mesh_filled(t, s_dyn_mesh[i], &s_dyn_xf[i], eye, fwd, rgt,
                             up, NULL, dyn_face_tiles(i), s_dyn_tex[i],
                             "dynamic-object");

    /* H-UAT-013: after every real mesh, the distant-vehicle impostors —
     * depth-tested, so anything already drawn nearer still wins. */
    if (s_loaded)
        draw_vehicle_impostors(t, camera);
}

int scene_filled_stats(char *buf, size_t n)
{
    return snprintf(buf, n,
                    "faces=%ld backfaced=%ld cutout_skipped=%ld textured=%ld",
                    s_filled_faces, s_filled_backfaced, s_filled_cutout_skipped,
                    s_filled_textured);
}

/* Draw every placed object, then the queued dynamic mesh (if any). */
static void render_objects(uint8_t *fb, int w, int h, const double eye[3],
                           const double fwd[3], const double rgt[3],
                           const double up[3], double focal)
{
    for (int i = 0; i < s_nobj; i++) {
        SceneObj *o = &s_objs[i];
        if (o->hidden || o->consumed)
            continue;   /* M7/H-UAT-060: hidden/consumed objects don't draw */
        for (int pi = 0; pi < o->nparts; pi++) {
            if (!part_active(&o->parts[pi])) continue;
            Xform xf = xform_compose(obj_draw_world(o), &o->parts[pi].local);
            draw_one_mesh(fb, w, h, s_mesh[o->parts[pi].mesh].mesh, &xf,
                          eye, fwd, rgt, up, focal);
        }
    }
    for (int i = 0; i < s_dyn_count; i++)
        draw_one_mesh(fb, w, h, s_dyn_mesh[i], &s_dyn_xf[i],
                      eye, fwd, rgt, up, focal);
}

void scene_render_camera(uint8_t *fb, int w, int h,
                         const CameraView *camera)
{
    if (!fb || w <= 0 || h <= 0 || !camera || !camera_view_valid(camera))
        return;
    memset(fb, IDX_BG, (size_t)w * (size_t)h);
    /* Menu chassis forms queue only dynamic car parts; no mission scene is
     * loaded there. Match scene_render_filled's dyn-only contract. */
    if (!s_loaded && s_dyn_count == 0)
        return;

    double focal = (w * 0.5) / camera_view_fov_tan_half(camera);
    render_objects(fb, w, h, camera->eye, camera->forward,
                   camera->right, camera->up, focal);
}

/* -----------------------------------------------------------------------
 * H-UAT-007 combat presentation (scene.h contract). Pure 2-D primitives
 * over the caller's frame: the fire streak, muzzle flash, contact burst,
 * kill burst, debris and opaque smoke phases all key off the snapshot's
 * tick-aged fields, so the decay schedule lives in combat.c and this
 * code only draws it. Palette indices resolve through the level palette
 * (raster_rgb_to_index); with no palette loaded the effects simply do
 * not draw (index 0), matching every other filled-path consumer.
 * ----------------------------------------------------------------------- */

typedef struct {
    uint8_t *fb;
    int      w, h;
    double   eye[3], rgt[3], up[3], fwd[3];
    double   focal;
} FxCanvas;

/* Camera-space z (<= near = behind); out gets framebuffer coords. */
static double fx_project(const FxCanvas *c, const double p[3],
                         double out[2])
{
    double d[3] = { p[0] - c->eye[0], p[1] - c->eye[1], p[2] - c->eye[2] };
    double x = d[0] * c->rgt[0] + d[1] * c->rgt[1] + d[2] * c->rgt[2];
    double y = d[0] * c->up[0]  + d[1] * c->up[1]  + d[2] * c->up[2];
    double z = d[0] * c->fwd[0] + d[1] * c->fwd[1] + d[2] * c->fwd[2];
    if (z > 1e-9) {
        out[0] = c->w * 0.5 + c->focal * x / z;
        out[1] = c->h * 0.5 - c->focal * y / z;
    }
    return z;
}

static void fx_put(FxCanvas *c, int x, int y, uint8_t color)
{
    if ((unsigned)x < (unsigned)c->w && (unsigned)y < (unsigned)c->h)
        c->fb[(size_t)y * c->w + x] = color;
}

static int fx_outcode(double x, double y, int w, int h)
{
    return (x < 0.0 ? 1 : x > w - 1 ? 2 : 0) |
           (y < 0.0 ? 4 : y > h - 1 ? 8 : 0);
}

static int fx_clip_line(double *x0, double *y0, double *x1, double *y1,
                        int w, int h)
{
    for (;;) {
        int a = fx_outcode(*x0, *y0, w, h);
        int b = fx_outcode(*x1, *y1, w, h);
        if (!(a | b)) return 1;
        if (a & b) return 0;
        int cc = a ? a : b;
        double x, y;
        if (cc & 4) {
            if (fabs(*y1 - *y0) < 1e-9) return 0;
            x = *x0 + (*x1 - *x0) * (0.0 - *y0) / (*y1 - *y0);
            y = 0.0;
        } else if (cc & 8) {
            if (fabs(*y1 - *y0) < 1e-9) return 0;
            x = *x0 + (*x1 - *x0) * ((h - 1.0) - *y0) / (*y1 - *y0);
            y = h - 1.0;
        } else if (cc & 2) {
            if (fabs(*x1 - *x0) < 1e-9) return 0;
            y = *y0 + (*y1 - *y0) * ((w - 1.0) - *x0) / (*x1 - *x0);
            x = w - 1.0;
        } else {
            if (fabs(*x1 - *x0) < 1e-9) return 0;
            y = *y0 + (*y1 - *y0) * (0.0 - *x0) / (*x1 - *x0);
            x = 0.0;
        }
        if (cc == a) { *x0 = x; *y0 = y; }
        else         { *x1 = x; *y1 = y; }
    }
}

static void fx_line(FxCanvas *c, double x0, double y0,
                    double x1, double y1, uint8_t color)
{
    if (!fx_clip_line(&x0, &y0, &x1, &y1, c->w, c->h)) return;
    int ax = (int)lround(x0), ay = (int)lround(y0);
    int bx = (int)lround(x1), by = (int)lround(y1);
    int dx = abs(bx - ax), sx = ax < bx ? 1 : -1;
    int dy = -abs(by - ay), sy = ay < by ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        fx_put(c, ax, ay, color);
        if (ax == bx && ay == by) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; ax += sx; }
        if (e2 <= dx) { err += dx; ay += sy; }
    }
}

static void fx_cross(FxCanvas *c, double x, double y,
                     int radius, uint8_t color)
{
    fx_line(c, x - radius, y, x + radius, y, color);
    fx_line(c, x, y - radius, x, y + radius, color);
}

/*
 * Draw the world-space segment a->b clipped against the near plane in 3-D
 * (H-UAT-013). The old streak path required BOTH endpoints in front of the
 * camera, which silently erased every incoming tracer: enemy fire ends at
 * the player's own car, so its end point sits at the camera and the whole
 * streak was dropped. Clipping the segment at z = FX_NEAR keeps the visible
 * part, which is exactly what makes incoming fire read as a line growing
 * toward the viewer.
 */
#define FX_NEAR 0.5
static void fx_line3d_width(FxCanvas *c, const double a[3], const double b[3],
                            uint8_t color, int width)
{
    double ra[3] = { a[0] - c->eye[0], a[1] - c->eye[1], a[2] - c->eye[2] };
    double rb[3] = { b[0] - c->eye[0], b[1] - c->eye[1], b[2] - c->eye[2] };
    double za = ra[0] * c->fwd[0] + ra[1] * c->fwd[1] + ra[2] * c->fwd[2];
    double zb = rb[0] * c->fwd[0] + rb[1] * c->fwd[1] + rb[2] * c->fwd[2];
    if (za < FX_NEAR && zb < FX_NEAR)
        return;                         /* wholly behind the near plane */
    double pa[3] = { a[0], a[1], a[2] }, pb[3] = { b[0], b[1], b[2] };
    if (za < FX_NEAR || zb < FX_NEAR) {
        double s = (FX_NEAR - za) / (zb - za);  /* za != zb: one is < FX_NEAR */
        double q[3] = { a[0] + (b[0] - a[0]) * s,
                        a[1] + (b[1] - a[1]) * s,
                        a[2] + (b[2] - a[2]) * s };
        if (za < FX_NEAR) memcpy(pa, q, sizeof pa);
        else              memcpy(pb, q, sizeof pb);
    }
    double sa[2], sb[2];
    if (fx_project(c, pa, sa) >= FX_NEAR - 1e-6 &&
        fx_project(c, pb, sb) >= FX_NEAR - 1e-6) {
        double dx = sb[0] - sa[0], dy = sb[1] - sa[1];
        double len = sqrt(dx * dx + dy * dy);
        double nx = len > 1e-9 ? -dy / len : 0.0;
        double ny = len > 1e-9 ?  dx / len : 1.0;
        if (width < 1) width = 1;
        for (int o = -(width - 1) / 2; o <= width / 2; o++)
            fx_line(c, sa[0] + nx * o, sa[1] + ny * o,
                    sb[0] + nx * o, sb[1] + ny * o, color);
    }
}

static void fx_circle(FxCanvas *c, double x, double y,
                      double radius, uint8_t color)
{
    double px = x + radius, py = y;
    for (int i = 1; i <= 16; i++) {
        double a = (6.28318530717958647692 * i) / 16.0;
        double nx = x + cos(a) * radius, ny = y + sin(a) * radius;
        fx_line(c, px, py, nx, ny, color);
        px = nx; py = ny;
    }
}

/* Filled disc — the smoke contract is an OPAQUE plume, not an outline. */
static void fx_disc(FxCanvas *c, double x, double y,
                    double radius, uint8_t color)
{
    int r = (int)(radius + 0.5);
    for (int dy = -r; dy <= r; dy++) {
        int half = (int)sqrt((double)(r * r - dy * dy));
        for (int dx = -half; dx <= half; dx++)
            fx_put(c, (int)x + dx, (int)y + dy, color);
    }
}

/* H-UAT-079b: every class-4 GDF carries a real OGEO part. Render that exact
 * purchaser-authored deployer into the software-owned indexed overlay;
 * WebGPU consumes the resulting pixels and never reauthors the object. The
 * shipped assets are tiny, so a bounded face-id array is sufficient. Gameplay
 * proximity radius never crosses this presentation seam. */
#define FX_AUTHORED_FACE_MAX 64
static int s_fx_authored_deployer_draws;
static int s_fx_oil_fallback_draws;
static char s_fx_authored_deployer_model[9];
static unsigned s_fx_authored_deployer_models;

int scene_fx_authored_deployer_draws(void)
{
    return s_fx_authored_deployer_draws;
}

int scene_fx_oil_fallback_draws(void)
{
    return s_fx_oil_fallback_draws;
}

const char *scene_fx_authored_deployer_model(void)
{
    return s_fx_authored_deployer_model;
}

unsigned scene_fx_authored_deployer_models(void)
{
    return s_fx_authored_deployer_models;
}

static int fx_authored_deployer(RTarget *t, const FxCanvas *c,
                                const CombatFx *e)
{
    const char *name = e->ordnance_model;
    GeoMesh *m = name[0] ? scene_part_mesh(name) : NULL;
    if (!m || m->num_faces <= 0 || m->num_faces > FX_AUTHORED_FACE_MAX)
        return 0;

    uint16_t tiles[FX_AUTHORED_FACE_MAX];
    int drawable = 0;
    for (int f = 0; f < m->num_faces; f++) {
        const char *face_tex = m->face_tex
                             ? &m->face_tex[(size_t)f * GEO_TEX_NAME_LEN] : "";
        tiles[f] = TEX_ID_NONE;
        if (face_tex[0]) {
            TexTmtInfo info;
            if (texcache_tmt_info(face_tex, &info) == 0 && info.name_count) {
                /* FACT weapon-impact-presentation.md §4: deployed oil,
                 * caltrops, and fire use eight authored frames at 1/6 s.
                 * Oil/caltrops settle on their final frame; fire's distinct
                 * FUN_0043b600 flags ping-pong the sequence. */
                uint32_t step = (uint32_t)((uint64_t)e->age * 3u / 10u);
                uint32_t frame;
                if (e->deploy_kind == CAR_DEPLOY_FIRE && info.name_count > 1) {
                    uint32_t period = (info.name_count - 1u) * 2u;
                    frame = step % period;
                    if (frame >= info.name_count)
                        frame = period - frame;
                } else {
                    frame = step < info.name_count ? step
                                                   : info.name_count - 1u;
                }
                tiles[f] = texcache_resolve_tmt_frame(face_tex, frame);
            }
        }
        if (face_tex[0] && tiles[f] == TEX_ID_NONE)
            tiles[f] = texcache_resolve(face_tex, "");
        int cutout = m->face_flags &&
                     GEO_FLAG2_CUTOUT(m->face_flags[f * 3 + 1]);
        if (!cutout || tiles[f] != TEX_ID_NONE)
            drawable++;
    }
    /* A keyed face without a resolved tile intentionally writes nothing.
     * Treat a missing/invalid TMT->frame->PIX/PAK chain as unavailable authored
     * appearance and fail closed; gameplay-radius fallback art is forbidden. */
    if (drawable == 0)
        return 0;
    Xform xf = xform_identity();
    xf.t[0] = e->end[0];
    xf.t[1] = e->end[1];
    xf.t[2] = e->end[2];
    draw_one_mesh_filled(t, m, &xf, c->eye, c->fwd, c->rgt, c->up,
                         NULL, tiles, NULL, "authored-deployed-ordnance");
    /* Resolution/submission owns fallback choice. An authored mesh that is
     * currently off-screen or fully occluded must not turn into fallback art
     * merely because this camera wrote no pixels. */
    return 1;
}

/* ----------------------------------------------------------------------- */
/* D-C27 native death presentation (vehicle-death-presentation.md)          */
/* ----------------------------------------------------------------------- */

/* Authored-vs-fallback submission counters for the kill/wreck sequence.
 * Same law as the deployer pair: resolution/submission owns the choice —
 * an authored asset that is off-camera or occluded is NOT a fallback. */
static int s_fx_kill_authored_draws;
static int s_fx_kill_fallback_draws;

int scene_fx_kill_authored_draws(void)
{
    return s_fx_kill_authored_draws;
}

int scene_fx_kill_fallback_draws(void)
{
    return s_fx_kill_fallback_draws;
}

/* Effect XDFs name one authored quad in a same-stem PAK. XDFC supplies the
 * native frame count and lifetime; the quad's face names its TMT flipbook.
 * Cache the small shipped family so impacts do no VFS work after first use. */
#define FX_EFFECT_ASSET_MAX 32
typedef struct {
    char name[14];
    char tmt[14];
    GeoMesh *mesh;
    uint32_t frames;
    int life_ticks;
    int valid;
} FxEffectAsset;
static FxEffectAsset s_fx_effect_asset[FX_EFFECT_ASSET_MAX];
static int s_fx_effect_asset_n;
static int s_fx_impact_authored_draws;
static int s_fx_impact_missing_draws;

int scene_fx_impact_authored_draws(void) { return s_fx_impact_authored_draws; }
int scene_fx_impact_missing_draws(void) { return s_fx_impact_missing_draws; }

static FxEffectAsset *fx_effect_asset(const char *xdf)
{
    if (!xdf || !xdf[0] || strcasecmp(xdf, "null") == 0)
        return NULL;
    for (int i = 0; i < s_fx_effect_asset_n; i++)
        if (strcasecmp(s_fx_effect_asset[i].name, xdf) == 0)
            return s_fx_effect_asset[i].valid ? &s_fx_effect_asset[i] : NULL;
    if (s_fx_effect_asset_n >= FX_EFFECT_ASSET_MAX)
        return NULL;

    FxEffectAsset *a = &s_fx_effect_asset[s_fx_effect_asset_n++];
    memset(a, 0, sizeof *a);
    copy_str(a->name, sizeof a->name, xdf);

    size_t len = 0;
    uint8_t *b = vfs_read_file(xdf, &len);
    if (!b) return NULL;
    for (size_t off = 0; off + 8 <= len; ) {
        uint32_t total = rd_u32(b + off + 4);
        if (total < 8 || (size_t)total > len - off) break;
        if (memcmp(b + off, "XDFC", 4) == 0 && total >= 48) {
            const uint8_t *p = b + off + 8;
            a->frames = rd_u32(p);
            double seconds = (double)rd_f32(p + 24);
            /* Purchaser/replacement binary data: reject non-finite and
             * unreasonable values before float-to-int conversion. Shipped
             * impact XDFs are 1-2 s; 60 s is a defensive format ceiling. */
            if (isfinite(seconds) && seconds > 0.0 && seconds <= 60.0)
                a->life_ticks = (int)ceil(seconds * 20.0);
            break;
        }
        off += total;
    }
    vfs_free(b);

    char pak[20];
    size_t stem = 0;
    while (xdf[stem] && xdf[stem] != '.' && stem + 5 < sizeof pak) {
        pak[stem] = xdf[stem];
        stem++;
    }
    memcpy(pak + stem, ".pak", 5);
    a->mesh = (GeoMesh *)geo_cache_acquire(pak);
    if (!a->mesh || a->mesh->num_verts != 4 || a->mesh->num_faces < 1 ||
        !a->mesh->face_tex || a->frames == 0 || a->life_ticks <= 0)
        return NULL;
    copy_str(a->tmt, sizeof a->tmt, a->mesh->face_tex);
    TexTmtInfo info;
    if (texcache_tmt_info(a->tmt, &info) != 0 || info.name_count == 0)
        return NULL;
    if (a->frames > info.name_count)
        return NULL;
    /* Native FUN_004a91e0 submits every explsn at fixed 0.1 s/frame;
     * TMT metadata does not own impact playback cadence. */
    a->valid = 1;
    return a;
}

/* Draw an authored flipbook quad as an upright camera-facing billboard
 * (horizontal axis = camera right, vertical = world up) with its base at
 * `base` — the native positions the object at the entity origin with no
 * centre offset, and the authored quad verts span y 0..height. Submission
 * counts as authored even when every pixel clips (the deployer law). */
static int fx_flipbook_quad(RTarget *t, const FxCanvas *c, const GeoMesh *m,
                            const double base[3], uint16_t tile)
{
    const RTex *tex = texcache_tile(tile);
    if (!m || m->num_verts != 4 || !m->uvs || !tex)
        return 0;
    RVert q[4];
    for (int i = 0; i < 4; i++) {
        double w[3] = { base[0] + c->rgt[0] * m->verts[i * 3],
                        base[1] + m->verts[i * 3 + 1],
                        base[2] + c->rgt[2] * m->verts[i * 3] };
        double rel[3] = { w[0] - c->eye[0], w[1] - c->eye[1],
                          w[2] - c->eye[2] };
        q[i].x = rel[0] * c->rgt[0] + rel[1] * c->rgt[1] + rel[2] * c->rgt[2];
        q[i].y = rel[0] * c->up[0]  + rel[1] * c->up[1]  + rel[2] * c->up[2];
        q[i].z = rel[0] * c->fwd[0] + rel[1] * c->fwd[1] + rel[2] * c->fwd[2];
        q[i].u = m->uvs[i * 2];
        q[i].v = m->uvs[i * 2 + 1];
        q[i].light = 1.0f;
        q[i].fog = 0.0f;
    }
    /* The frames are keyed 64/128px VQMs: cutout so the colour key (and
     * only it) stays transparent. */
    raster_polygon_tex(t, q, 4, 1, tex, 1);
    return 1;
}

static int fx_authored_effect(RTarget *t, const FxCanvas *c,
                              const char *xdf, const double pos[3], int age)
{
    FxEffectAsset *a = fx_effect_asset(xdf);
    if (!a || age < 0)
        return 0;
    if (age >= a->life_ticks)
        return -1; /* authored expiry, not a missing-asset fallback */
    uint32_t frame = (uint32_t)age / 2u; /* native fixed 10 fps at 20 Hz */
    if (frame >= a->frames)
        frame = a->frames - 1;
    uint16_t tile = texcache_resolve_tmt_frame(a->tmt, frame);
    return fx_flipbook_quad(t, c, a->mesh, pos, tile);
}

/* One camera-facing smoke sprite from an authored family frame. */
static int fx_smoke_sprite(RTarget *t, const FxCanvas *c,
                           const double pos[3], double half, uint16_t tile)
{
    const RTex *tex = texcache_tile(tile);
    if (!tex || !(half > 0.0))
        return 0;
    static const float uvs[4][2] = { {0, 1}, {1, 1}, {1, 0}, {0, 0} };
    static const double sx[4] = {-1, 1, 1, -1};
    static const double sy[4] = {-1, -1, 1, 1};
    RVert q[4];
    for (int i = 0; i < 4; i++) {
        double w[3] = {
            pos[0] + c->rgt[0] * sx[i] * half + c->up[0] * sy[i] * half,
            pos[1] + c->rgt[1] * sx[i] * half + c->up[1] * sy[i] * half,
            pos[2] + c->rgt[2] * sx[i] * half + c->up[2] * sy[i] * half
        };
        double rel[3] = { w[0] - c->eye[0], w[1] - c->eye[1],
                          w[2] - c->eye[2] };
        q[i].x = rel[0] * c->rgt[0] + rel[1] * c->rgt[1] + rel[2] * c->rgt[2];
        q[i].y = rel[0] * c->up[0]  + rel[1] * c->up[1]  + rel[2] * c->up[2];
        q[i].z = rel[0] * c->fwd[0] + rel[1] * c->fwd[1] + rel[2] * c->fwd[2];
        q[i].u = uvs[i][0];
        q[i].v = uvs[i][1];
        q[i].light = 1.0f;
        q[i].fog = 0.0f;
    }
    raster_polygon_tex(t, q, 4, 1, tex, 1);
    return 1;
}

/* Submit an authored debris mesh (CHUNK1/CHUNK2) at a world position,
 * faces textured by name through the ordinary resolve path. */
static int fx_mesh_at(RTarget *t, const FxCanvas *c, GeoMesh *m,
                      const double pos[3])
{
    if (!m || m->num_faces <= 0 || m->num_faces > FX_AUTHORED_FACE_MAX)
        return 0;
    uint16_t tiles[FX_AUTHORED_FACE_MAX];
    int drawable = 0;
    for (int f = 0; f < m->num_faces; f++) {
        const char *face_tex = m->face_tex
                             ? &m->face_tex[(size_t)f * GEO_TEX_NAME_LEN] : "";
        tiles[f] = face_tex[0] ? texcache_resolve(face_tex, "") : TEX_ID_NONE;
        int cutout = m->face_flags &&
                     GEO_FLAG2_CUTOUT(m->face_flags[f * 3 + 1]);
        if (!cutout || tiles[f] != TEX_ID_NONE)
            drawable++;
    }
    if (drawable == 0)
        return 0;
    Xform xf = xform_identity();
    xf.t[0] = pos[0];
    xf.t[1] = pos[1];
    xf.t[2] = pos[2];
    draw_one_mesh_filled(t, m, &xf, c->eye, c->fwd, c->rgt, c->up,
                         NULL, tiles, NULL, "wreck-chunk-debris");
    return 1;
}


void scene_render_combat_fx(uint8_t *fb, int w, int h,
                            const CameraView *camera,
                            const CombatFx *fx, int nfx,
                            const double *target_xyz)
{
    s_fx_authored_deployer_draws = 0;
    s_fx_oil_fallback_draws = 0;
    s_fx_authored_deployer_model[0] = '\0';
    s_fx_authored_deployer_models = 0;
    s_fx_kill_authored_draws = 0;
    s_fx_kill_fallback_draws = 0;
    s_fx_impact_authored_draws = 0;
    s_fx_impact_missing_draws = 0;
    if (!fb || w <= 0 || h <= 0 || !camera || !camera_view_valid(camera))
        return;
    if (!raster_palette_ready())
        return;             /* no level palette: no indices to draw with */

    FxCanvas c;
    c.fb = fb;
    c.w = w;
    c.h = h;
    memcpy(c.eye, camera->eye, sizeof c.eye);
    memcpy(c.rgt, camera->right, sizeof c.rgt);
    memcpy(c.up, camera->up, sizeof c.up);
    memcpy(c.fwd, camera->forward, sizeof c.fwd);
    c.focal = (w * 0.5) / camera_view_fov_tan_half(camera);

    uint8_t white  = raster_rgb_to_index(255, 248, 190);
    uint8_t yellow = raster_rgb_to_index(255, 220, 48);
    uint8_t orange = raster_rgb_to_index(255, 96, 24);
    uint8_t red    = raster_rgb_to_index(255, 24, 24);
    uint8_t smoke  = raster_rgb_to_index(145, 145, 145);
    uint8_t darksm = raster_rgb_to_index(82, 82, 78);
    uint8_t gas    = raster_rgb_to_index(116, 162, 88);
    uint8_t debris = raster_rgb_to_index(60, 48, 40);

    /* Authored deployed meshes share one local depth plane so overlapping
     * mines/oil resolve deterministically before the indexed layer is handed
     * unchanged to WebGPU. This depth is presentation-only and never feeds
     * collision or the world renderer. */
    static uint32_t *deployer_depth;
    static size_t deployer_depth_cap;
    size_t deployer_pixels = (size_t)w * (size_t)h;
    RTarget deployer_target;
    memset(&deployer_target, 0, sizeof deployer_target);
    if (deployer_pixels <= SIZE_MAX / sizeof *deployer_depth) {
        if (deployer_pixels > deployer_depth_cap) {
            uint32_t *next = realloc(deployer_depth,
                                     deployer_pixels * sizeof *next);
            if (next) {
                deployer_depth = next;
                deployer_depth_cap = deployer_pixels;
            }
        }
        if (deployer_depth && deployer_depth_cap >= deployer_pixels) {
            memset(deployer_depth, 0, deployer_pixels * sizeof *deployer_depth);
            raster_begin(&deployer_target, fb, deployer_depth, w, h, c.focal,
                         FX_NEAR, RASTER_ZFAR);
        }
    }

    /* The target marker rides the entity the fire cone would strike. */
    if (target_xyz) {
        double p[2];
        if (fx_project(&c, target_xyz, p) >= 0.5) {
            int r = 9;
            /* four corner brackets, open toward the target */
            fx_line(&c, p[0] - r, p[1] - r, p[0] - r / 3, p[1] - r, yellow);
            fx_line(&c, p[0] - r, p[1] - r, p[0] - r, p[1] - r / 3, yellow);
            fx_line(&c, p[0] + r, p[1] - r, p[0] + r / 3, p[1] - r, yellow);
            fx_line(&c, p[0] + r, p[1] - r, p[0] + r, p[1] - r / 3, yellow);
            fx_line(&c, p[0] - r, p[1] + r, p[0] - r / 3, p[1] + r, yellow);
            fx_line(&c, p[0] - r, p[1] + r, p[0] - r, p[1] + r / 3, yellow);
            fx_line(&c, p[0] + r, p[1] + r, p[0] + r / 3, p[1] + r, yellow);
            fx_line(&c, p[0] + r, p[1] + r, p[0] + r, p[1] + r / 3, yellow);
        }
    }

    for (int i = 0; i < nfx; i++) {
        const CombatFx *e = &fx[i];
        if (!e->active)
            continue;

        if (e->type == COMBAT_FX_DEPLOYED) {
            /* The GDF's OGEO is presentation authority for all six families.
             * Missing purchaser art fails closed: no gameplay-radius disc,
             * square, cross, or other synthetic hit-volume visualization. */
            if (deployer_target.color &&
                fx_authored_deployer(&deployer_target, &c, e)) {
                s_fx_authored_deployer_draws++;
                copy_str(s_fx_authored_deployer_model,
                         sizeof s_fx_authored_deployer_model,
                         e->ordnance_model);
                static const char *const model[] = {
                    "MOILSPIL", "MFIRESPL", "MLNDMINE", "MCALTROP", "mblox"
                };
                for (unsigned m = 0; m < sizeof model / sizeof model[0]; m++)
                    if (strcasecmp(e->ordnance_model, model[m]) == 0)
                        s_fx_authored_deployer_models |= 1u << m;
            }
            continue;
        }

        if (e->type == COMBAT_FX_MUZZLE) {
            double p[2];
            uint8_t color = e->weapon_class == COMBAT_FX_GAS ? gas : yellow;
            if (fx_project(&c, e->end, p) >= FX_NEAR)
                fx_cross(&c, p[0], p[1], 4 - (e->age > 2 ? 2 : e->age),
                         color);
            continue;
        }

        if (e->type == COMBAT_FX_PROJECTILE || e->type == 0) {
            const double *first = e->trail_count > 0 ? e->trail[0] : e->start;
            const double *last = e->trail_count > 0
                               ? e->trail[e->trail_count - 1] : e->end;
            if (e->weapon_class == COMBAT_FX_MISSILE) {
                /* The plume is attached to chronological positions sampled
                 * from CombatProjectile, including guided turns. */
                for (int k = 0; k < e->trail_count - 1; k++) {
                    double p[2];
                    if (fx_project(&c, e->trail[k], p) >= FX_NEAR)
                        fx_disc(&c, p[0], p[1], 2 + (k & 1),
                                k & 1 ? smoke : darksm);
                }
                fx_line3d_width(&c, first, last, orange, 2);
                double p[2];
                if (fx_project(&c, e->end, p) >= FX_NEAR) {
                    fx_disc(&c, p[0], p[1], 3, red);
                    fx_cross(&c, p[0], p[1], 2, yellow);
                }
            } else if (e->weapon_class == COMBAT_FX_FLAME ||
                       e->weapon_class == COMBAT_FX_GAS) {
                uint8_t outer = e->weapon_class == COMBAT_FX_GAS ? gas : orange;
                /* ORDF 9/10/11 exposes the current sim-authority stream as a
                 * bounded sampled trail. Width styling is PORT PRESENTATION;
                 * collision uses combat.c's separately marked swept radius. */
                if (e->trail_count > 1)
                    for (int k = 1; k < e->trail_count; k++)
                        fx_line3d_width(&c, e->trail[k - 1], e->trail[k],
                                        outer, 2 + k / 3);
                for (int k = 0; k < e->trail_count; k++) {
                    double p[2];
                    if (fx_project(&c, e->trail[k], p) >= FX_NEAR)
                        fx_disc(&c, p[0], p[1], 2 + k / 2, outer);
                }
                double p[2];
                if (fx_project(&c, e->end, p) >= FX_NEAR)
                    fx_disc(&c, p[0], p[1], 2 + e->trail_count / 2,
                            e->weapon_class == COMBAT_FX_GAS ? gas : yellow);
            } else if (e->weapon_class == COMBAT_FX_EXPLOSIVE) {
                fx_line3d_width(&c, first, last, orange,
                                e->damage >= 100 ? 3 : 2);
                double p[2];
                if (fx_project(&c, e->end, p) >= FX_NEAR)
                    fx_disc(&c, p[0], p[1], e->damage >= 100 ? 3 : 2,
                            yellow);
            } else if (e->weapon_class == COMBAT_FX_TRACER_LIGHT) {
                /* H-UAT-067d: an MG tracer is the newest bright flight
                 * segment, not the complete chronological pose trail plus a
                 * glowing projectile body. Missiles alone retain smoke along
                 * their full trail above. */
                const double *a = e->trail_count > 1
                                ? e->trail[e->trail_count - 2] : first;
                fx_line3d_width(&c, a, last, white, 1);
            } else {
                /* H-UAT-079c: ORDF-6 cannons are non-tracking shells, not
                 * missiles. Show only a bounded newest streak and no glowing
                 * head/full chronological trail. The software-owned overlay
                 * is consumed unchanged by WebGPU. */
                const double *a = e->trail_count > 1
                                ? e->trail[e->trail_count - 2] : first;
                double tail[3] = { a[0], a[1], a[2] };
                double dx = last[0] - a[0], dy = last[1] - a[1],
                       dz = last[2] - a[2];
                double len = sqrt(dx * dx + dy * dy + dz * dz);
                const double streak = 4.0;
                if (len > streak) {
                    double scale = streak / len;
                    tail[0] = last[0] - dx * scale;
                    tail[1] = last[1] - dy * scale;
                    tail[2] = last[2] - dz * scale;
                }
                fx_line3d_width(&c, tail, last, orange,
                                e->damage >= 200 ? 2 : 1);
            }
            continue;
        }

        int phase = e->age;
        double p[2];
        int visible = fx_project(&c, e->end, p) >= FX_NEAR;

        if (e->type == COMBAT_FX_SECONDARY) {
            /* D-C27: X1_CARS1 secondary pop beside the burning hull — the
             * 12-frame XCS1_101.TMT flipbook at 10 fps on the authored
             * 2.3x2.0 m quad, under the same 2.0 s explsn lifetime. */
            if (phase < COMBAT_EXPLOSION_TICKS) {
                if (deployer_target.color &&
                    fx_authored_effect(&deployer_target, &c, "xcars1.xdf",
                                       e->end, phase) > 0)
                    s_fx_kill_authored_draws++;
                else
                    s_fx_kill_fallback_draws++;
            }
            continue;
        }

        if (e->type == COMBAT_FX_DEBRIS) {
            /* D-C27: CHUNK1/CHUNK2 debris flung straight up at the native
             * 15 m/s. The ballistic drop is MARKED 9.8 m/s^2 — the chunk
             * manager's gravity constant is undecoded. */
            double tsec = phase * 0.05;
            double q[3] = { e->end[0],
                            e->end[1] + e->speed * tsec - 4.9 * tsec * tsec,
                            e->end[2] };
            if (deployer_target.color &&
                fx_mesh_at(&deployer_target, &c,
                           scene_part_mesh(e->ordnance_model), q)) {
                s_fx_kill_authored_draws++;
            } else {
                double sp[2];
                if (fx_project(&c, q, sp) >= FX_NEAR) {
                    fx_cross(&c, sp[0], sp[1], 2, debris);
                    s_fx_kill_fallback_draws++;
                }
            }
            continue;
        }

        if (e->type != COMBAT_FX_IMPACT && e->type != COMBAT_FX_KILL)
            continue;

        if (!e->killed) {
            /* ORDF names the native target-class XDF. Empty/"null" is an
             * authored request for no effect; a named but unavailable asset
             * fails closed instead of reviving the spherical disc fallback. */
            if (e->effect_name[0] && deployer_target.color) {
                /* Submission succeeds even when camera-clipped; a missing
                 * local depth target is not an asset failure. */
                int drawn = fx_authored_effect(&deployer_target, &c,
                                                e->effect_name, e->end, phase);
                if (drawn > 0)
                    s_fx_impact_authored_draws++;
                else if (drawn == 0)
                    s_fx_impact_missing_draws++;
            }
            continue;
        }

        /* Vehicle kill (D-C27, vehicle-death-presentation.md §1): the
         * native authored sequence supersedes the invented fireball /
         * debris / 20 s smoke column — the 2.0 s X1_CARX1 explosion
         * (24-frame XCX1_101.TMT flipbook at 10 fps on the authored
         * 5.6x4.3 m quad at the entity position, no centre offset),
         * while the still-visible hull burns with heavy xbp1 smoke for
         * its whole 10.0 s. The 20 m dynamic-light flash waits on
         * renderer light support (spec §4.1 port decision: skip
         * silently). Secondary X1_CARS1 pops and CHUNK debris arrive as
         * their own records above. H-UAT-079b removes the old flat-disc
         * fallback: unavailable purchaser art is counted and omitted. */
        if (visible && phase < COMBAT_EXPLOSION_TICKS) {
            if (deployer_target.color &&
                fx_authored_effect(&deployer_target, &c, "xcarx1.xdf",
                                   e->end, phase) > 0)
                s_fx_kill_authored_draws++;
            else
                s_fx_kill_fallback_draws++;
        }
        /* The native engine-smoke emitter survives death at level 2
         * (xbp1, rate 1.0), so the wreck smokes for its full 10.0 s.
         * Frame cadence is MARKED — the frame table DAT_004f2590 is
         * decoded (level 2, rate 1.0 -> type 1 -> the 201-204 set) but
         * its per-tick advance is not. */
        for (int s = 0; s < 9; s++) {
            int puff_age = (phase + s * 9) % 72;
            double q[3], sp[2];
            double drift = puff_age * 0.025;
            q[0] = e->end[0] +
                   sin((double)(e->seed * 7 + s * 11)) * drift;
            q[1] = e->end[1] + 0.7 + puff_age * 0.13;
            q[2] = e->end[2] +
                   cos((double)(e->seed * 5 + s * 13)) * drift;
            if (fx_project(&c, q, sp) < FX_NEAR)
                continue;
            char frame_name[16];
            snprintf(frame_name, sizeof frame_name, "xbp1_%d.vqm",
                     201 + (puff_age / 2) % 4);
            uint16_t tile = texcache_resolve_family_frame("xbp1_101.pix",
                                                          frame_name);
            if (deployer_target.color &&
                fx_smoke_sprite(&deployer_target, &c, q,
                                0.8 + puff_age * 0.03, tile)) {
                s_fx_kill_authored_draws++;
            } else {
                fx_disc(&c, sp[0], sp[1], 3 + puff_age / 18,
                        puff_age < 28 ? darksm : smoke);
                s_fx_kill_fallback_draws++;
            }
        }
    }
}

void scene_set_dynamic_paint(const char *vtf)
{
    copy_str(s_dyn_vtf, sizeof s_dyn_vtf, vtf ? vtf : "");
}

void scene_dyn_clear(void)
{
    s_dyn_count = 0;
}

int scene_dyn_add_textured(GeoMesh *m, const double r[3], const double u[3],
                           const double f[3], const double pos[3],
                           const RTex *tex)
{
    if (!m || s_dyn_count >= SCENE_DYN_MAX) return -1;
    int slot = s_dyn_count++;
    s_dyn_mesh[slot] = m;
    s_dyn_tex[slot] = tex;
    s_dyn_xf[slot] = xform_identity();
    for (int i = 0; i < 3; i++) {
        s_dyn_xf[slot].m[i][0] = r[i];  /* columns = basis vectors (D4) */
        s_dyn_xf[slot].m[i][1] = u[i];
        s_dyn_xf[slot].m[i][2] = f[i];
        s_dyn_xf[slot].t[i]    = pos[i];
    }
    return slot;
}

int scene_dyn_add(GeoMesh *m, const double r[3], const double u[3],
                  const double f[3], const double pos[3])
{
    return scene_dyn_add_textured(m, r, u, f, pos, NULL);
}

void scene_set_dynamic(GeoMesh *m, const double r[3], const double u[3],
                       const double f[3], const double pos[3])
{
    scene_dyn_clear();
    if (m)
        (void)scene_dyn_add(m, r, u, f, pos);
}

/* ----------------------------------------------------------------------- */
/* Read-only enumeration (M6 Tier-2 geometry export; additive — no effect  */
/* on any existing call path)                                               */
/* ----------------------------------------------------------------------- */

int scene_obj_count(void)
{
    return s_loaded ? s_nobj : 0;
}

int scene_obj_part_count(int obj)
{
    if (obj < 0 || obj >= s_nobj) return 0;
    return s_objs[obj].nparts;
}

int scene_obj_part_active(int obj, int part)
{
    if (obj < 0 || obj >= s_nobj) return 0;
    const SceneObj *o = &s_objs[obj];
    if (part < 0 || part >= o->nparts) return 0;
    return part_active(&o->parts[part]);
}
int scene_obj_part_gate(int obj, int part)
{
    if (obj < 0 || obj >= s_nobj) return 0;
    const SceneObj *o = &s_objs[obj];
    if (part < 0 || part >= o->nparts) return 0;
    return o->parts[part].pclass == SCENE_PART_CLASS_GATE;
}


GeoMesh *scene_obj_part_mesh(int obj, int part)
{
    if (!scene_obj_part_active(obj, part)) return NULL;
    const SceneObj *o = &s_objs[obj];
    return s_mesh[o->parts[part].mesh].mesh;
}

const char *scene_obj_vtf(int obj)
{
    if (!s_loaded || obj < 0 || obj >= s_nobj) return "";
    return s_objs[obj].vtf;
}

/* Shared tail of the two OBB exports: place an own-frame AABB (lo/hi) by
 * the object transform and derive the world box the sim collides against
 * (centre, half-extents, XZ axis, true world y span). */
static void obb_finish(const Xform *world, const double lo[3],
                       const double hi[3], double centre[3], double half[2],
                       double axis[2], double yspan[2])
{
    double cl[3] = { (lo[0]+hi[0])*0.5, (lo[1]+hi[1])*0.5, (lo[2]+hi[2])*0.5 };
    float clf[3] = { (float)cl[0], (float)cl[1], (float)cl[2] };
    xform_apply(world, clf, centre);
    half[0] = (hi[0] - lo[0]) * 0.5;
    half[1] = (hi[2] - lo[2]) * 0.5;

    /* Box +x axis = the object's local +x projected into world XZ. A object
     * standing on end (degenerate XZ projection) falls back to world +x. */
    double ax = world->m[0][0], az = world->m[2][0];
    double al = sqrt(ax * ax + az * az);
    if (al < 1e-9) { axis[0] = 1.0; axis[1] = 0.0; }
    else           { axis[0] = ax / al; axis[1] = az / al; }

    /* World vertical span over the same corners — the height test needs
     * true world y, which a rotated placement changes. */
    double y0 = 1e30, y1 = -1e30;
    for (int c = 0; c < 8; c++) {
        float v[3] = { (float)((c & 1) ? hi[0] : lo[0]),
                       (float)((c & 2) ? hi[1] : lo[1]),
                       (float)((c & 4) ? hi[2] : lo[2]) };
        double w[3];
        xform_apply(world, v, w);
        if (w[1] < y0) y0 = w[1];
        if (w[1] > y1) y1 = w[1];
    }
    yspan[0] = y0;
    yspan[1] = y1;
}

int scene_obj_obb(int obj, double centre[3], double half[2], double axis[2],
                  double yspan[2])
{
    if (!centre || !half || !axis || !yspan) return -1;
    if (obj < 0 || obj >= s_nobj) return -1;
    const SceneObj *o = &s_objs[obj];

    /* Pass 1: the object's own-frame AABB over every part's mesh bbox,
     * composed through the PART transform only — the object placement is
     * applied afterwards, which is what makes the box oriented rather than
     * an inflated world-axis-aligned one (car.h D19). */
    double lo[3] = { 1e30, 1e30, 1e30 }, hi[3] = { -1e30, -1e30, -1e30 };
    int any = 0;
    for (int pi = 0; pi < o->nparts; pi++) {
        if (!part_active(&o->parts[pi])) continue;
        int slot = o->parts[pi].mesh;
        GeoMesh *m = (slot >= 0 && slot < s_nmesh) ? s_mesh[slot].mesh : NULL;
        if (!m || m->num_verts <= 0) continue;
        for (int c = 0; c < 8; c++) {
            float v[3] = {
                (c & 1) ? m->bb_max[0] : m->bb_min[0],
                (c & 2) ? m->bb_max[1] : m->bb_min[1],
                (c & 4) ? m->bb_max[2] : m->bb_min[2],
            };
            double w[3];
            xform_apply(&o->parts[pi].local, v, w);
            for (int k = 0; k < 3; k++) {
                if (w[k] < lo[k]) lo[k] = w[k];
                if (w[k] > hi[k]) hi[k] = w[k];
            }
            any = 1;
        }
    }
    if (!any) return -1;

    obb_finish(&o->world, lo, hi, centre, half, axis, yspan);
    return 0;
}

int scene_obj_ground_gaps(int obj, double (*height_at)(double, double),
                          double *min_gap, double *max_gap)
{
    if (!height_at || !min_gap || !max_gap || obj < 0 || obj >= s_nobj)
        return -1;
    const SceneObj *o = &s_objs[obj];
    double lo = 1e30, hi = -1e30;
    int any = 0;
    for (int pi = 0; pi < o->nparts; pi++) {
        if (!part_active(&o->parts[pi])) continue;
        int slot = o->parts[pi].mesh;
        GeoMesh *m = (slot >= 0 && slot < s_nmesh) ? s_mesh[slot].mesh : NULL;
        if (!m || m->num_verts <= 0) continue;
        Xform world = xform_compose(&o->world, &o->parts[pi].local);
        for (int v = 0; v < m->num_verts; v++) {
            double p[3];
            xform_apply(&world, &m->verts[v * 3], p);
            double gap = p[1] - height_at(p[0], p[2]);
            if (gap < lo) lo = gap;
            if (gap > hi) hi = gap;
            any = 1;
        }
    }
    if (!any) return -1;
    *min_gap = lo;
    *max_gap = hi;
    return 0;
}

int scene_obj_part_obb(int obj, int part, double centre[3], double half[2],
                       double axis[2], double yspan[2])
{
    if (!centre || !half || !axis || !yspan) return -1;
    if (obj < 0 || obj >= s_nobj) return -1;
    const SceneObj *o = &s_objs[obj];
    if (part < 0 || part >= o->nparts) return -1;
    if (!part_active(&o->parts[part])) return -1;
    int slot = o->parts[part].mesh;
    GeoMesh *m = (slot >= 0 && slot < s_nmesh) ? s_mesh[slot].mesh : NULL;
    if (!m || m->num_verts <= 0) return -1;

    /* Same construction as scene_obj_obb with the union pass narrowed to
     * this one part: own-frame AABB through the PART transform, placed by
     * the object transform. */
    double lo[3] = { 1e30, 1e30, 1e30 }, hi[3] = { -1e30, -1e30, -1e30 };
    for (int c = 0; c < 8; c++) {
        float v[3] = {
            (c & 1) ? m->bb_max[0] : m->bb_min[0],
            (c & 2) ? m->bb_max[1] : m->bb_min[1],
            (c & 4) ? m->bb_max[2] : m->bb_min[2],
        };
        double w[3];
        xform_apply(&o->parts[part].local, v, w);
        for (int k = 0; k < 3; k++) {
            if (w[k] < lo[k]) lo[k] = w[k];
            if (w[k] > hi[k]) hi[k] = w[k];
        }
    }
    obb_finish(&o->world, lo, hi, centre, half, axis, yspan);
    return 0;
}

int scene_part_drive_surface(int obj, int part)
{
    if (!s_loaded || obj < 0 || obj >= s_nobj) return 0;
    const SceneObj *o = &s_objs[obj];
    if (part < 0 || part >= o->nparts) return 0;
    return o->parts[part].drive_surf;
}

int scene_part_drivable_object(int obj, int part)
{
    if (!s_loaded || obj < 0 || obj >= s_nobj) return 0;
    const SceneObj *o = &s_objs[obj];
    if (part < 0 || part >= o->nparts || o->drivable_object <= 0) return 0;
    uint32_t cls = o->sdf_class >= 11 && o->sdf_class <= 13
                 ? o->sdf_class : o->class_id;
    if (cls < 11 || cls > 13) return 0;
    if (cls == 12 && o->parts[part].pclass != 13) return 0;
    return o->drivable_object;
}

int scene_part_drivable_parent(int obj, int part)
{
    if (!s_loaded || obj < 0 || obj >= s_nobj) return 0;
    const SceneObj *o = &s_objs[obj];
    if (part < 0 || part >= o->nparts || o->drivable_object <= 0)
        return 0;
    uint32_t cls = o->sdf_class >= 11 && o->sdf_class <= 13
                 ? o->sdf_class : o->class_id;
    if (cls != 12 || o->parts[part].pclass == 13) return 0;

    double c[3], half[2], axis[2], yspan[2];
    if (scene_obj_part_obb(obj, part, c, half, axis, yspan) != 0)
        return 0;
    for (int deck = 0; deck < o->nparts; deck++) {
        if (o->parts[deck].pclass != 13) continue;
        double dc[3], dh[2], da[2], dy[2];
        if (scene_obj_part_obb(obj, deck, dc, dh, da, dy) != 0)
            continue;
        /* PORT ADAPTATION: only the near-identical under-deck bbox that
         * creates the measured false wall may borrow scoped face probes.
         * Narrower/taller rail and pylon boxes remain ordinary solids. */
        if (hypot(c[0] - dc[0], c[2] - dc[2]) <= 0.1 &&
            fabs(half[0] - dh[0]) <= 0.1 &&
            fabs(half[1] - dh[1]) <= 0.1 &&
            fabs(axis[0] * da[0] + axis[1] * da[1]) >= 0.9999 &&
            yspan[1] <= dy[1] + 0.05)
            return o->drivable_object;
    }
    return 0;
}

int scene_obj_bounds(int obj, double centre[3], double *radius)
{
    if (!centre || !radius || obj < 0 || obj >= s_nobj) return -1;
    const SceneObj *o = &s_objs[obj];
    for (int i = 0; i < 3; i++) centre[i] = o->world.t[i];
    double best = 0.0;
    int any = 0;
    for (int pi = 0; pi < o->nparts; pi++) {
        if (!part_active(&o->parts[pi])) continue;
        int slot = o->parts[pi].mesh;      /* index into s_mesh, -1 = none */
        GeoMesh *m = (slot >= 0 && slot < s_nmesh) ? s_mesh[slot].mesh : NULL;
        if (!m || m->num_verts <= 0) continue;
        Xform xf = xform_compose(&o->world, &o->parts[pi].local);
        /* Every bbox corner through the part transform; keep the largest
         * horizontal distance from the object's own centre. */
        for (int c = 0; c < 8; c++) {
            float v[3] = {
                (c & 1) ? m->bb_max[0] : m->bb_min[0],
                (c & 2) ? m->bb_max[1] : m->bb_min[1],
                (c & 4) ? m->bb_max[2] : m->bb_min[2],
            };
            double w[3];
            xform_apply(&xf, v, w);
            double dx = w[0] - centre[0], dz = w[2] - centre[2];
            double d = sqrt(dx * dx + dz * dz);
            if (d > best) best = d;
            any = 1;
        }
    }
    if (!any) return -1;
    *radius = best;
    return 0;
}

int scene_obj_part_xform(int obj, int part, double out12[12])
{
    if (!out12 || obj < 0 || obj >= s_nobj) return -1;
    const SceneObj *o = &s_objs[obj];
    if (part < 0 || part >= o->nparts) return -1;
    if (!part_active(&o->parts[part])) return -1;
    Xform xf = xform_compose(obj_draw_world(o), &o->parts[part].local);
    for (int i = 0; i < 3; i++) {
        out12[i]     = xf.m[i][0];      /* right   column */
        out12[3 + i] = xf.m[i][1];      /* up      column */
        out12[6 + i] = xf.m[i][2];      /* forward column */
        out12[9 + i] = xf.t[i];         /* position       */
    }
    return 0;
}

const char *scene_obj_label(int obj)
{
    return (obj >= 0 && obj < s_nobj) ? s_objs[obj].label : "";
}

int scene_obj_class_id(int obj)
{
    return obj >= 0 && obj < s_nobj ? (int)s_objs[obj].class_id : -1;
}

int scene_obj_is_vehicle_mesh(int obj)
{
    return obj >= 0 && obj < s_nobj && s_objs[obj].class_id == 1 &&
           s_objs[obj].nparts > 0;
}

int scene_obj_find(const char *label, int label_id)
{
    int first = -1;
    if (!s_loaded || !label)
        return -1;
    for (int i = 0; i < s_nobj; i++) {
        if (strcasecmp(s_objs[i].label, label) != 0)
            continue;
        if (s_objs[i].label_id == label_id)
            return i;
        if (first < 0)
            first = i;
    }
    return first;
}
/*
 * The nth (0-based) placed object carrying `label`, in ODEF file order.
 *
 * scene_obj_find keys on the packed label id, which is NOT unique in the
 * shipped data — N02.CBT has eight `spawn` markers sharing five ids and
 * N52.CBT eight sharing six — so it maps several distinct grid slots onto
 * one object. The melee spawner needs one body per slot, and ordinal
 * position is the only thing that separates them. Both this module and
 * mission.c walk the same ODEF chunk in file order, so the nth `spawn` here
 * is the nth `spawn` there; -1 when there are fewer than nth+1.
 */
int scene_obj_find_nth(const char *label, int nth)
{
    int k = 0;
    if (!s_loaded || !label || nth < 0)
        return -1;
    for (int i = 0; i < s_nobj; i++) {
        if (strcasecmp(s_objs[i].label, label) != 0)
            continue;
        if (k == nth)
            return i;
        k++;
    }
    return -1;
}

/* World translation of a placed object (the counterpart of
 * scene_obj_set_pos). 0 on success, -1 on a bad index/NULL. */
int scene_obj_pos(int obj, double out[3])
{
    if (!out || obj < 0 || obj >= s_nobj)
        return -1;
    out[0] = s_objs[obj].world.t[0];
    out[1] = s_objs[obj].world.t[1];
    out[2] = s_objs[obj].world.t[2];
    return 0;
}

int scene_obj_set_pos(int obj, const double pos[3])
{
    if (!pos || obj < 0 || obj >= s_nobj)
        return -1;
    s_objs[obj].world.t[0] = pos[0];
    s_objs[obj].world.t[1] = pos[1];
    s_objs[obj].world.t[2] = pos[2];
    return 0;
}

int scene_obj_world_xform(int obj, double out12[12])
{
    if (!out12 || obj < 0 || obj >= s_nobj)
        return -1;
    xform_to12(&s_objs[obj].world, out12);
    return 0;
}

int scene_obj_draw_xform(int obj, double out12[12])
{
    if (!out12 || obj < 0 || obj >= s_nobj)
        return -1;
    xform_to12(obj_draw_world(&s_objs[obj]), out12);
    return 0;
}

#define SCENE_PRESENT_TELEPORT_M 20.0

void scene_present_reset(void)
{
    s_present_draw = 0;
    for (int i = 0; i < s_nobj; i++) {
        s_objs[i].present_ready = 0;
        s_objs[i].present_armed = 0;
    }
}

void scene_present_capture_tick(void)
{
    s_present_draw = 0;
    for (int i = 0; i < s_nobj; i++) {
        SceneObj *o = &s_objs[i];
        if (!o->present_ready) {
            o->tick_prev = o->world;
            o->tick_curr = o->world;
            o->present_ready = 1;
        } else {
            o->tick_prev = o->tick_curr;
            o->tick_curr = o->world;
        }
    }
}

void scene_present_prepare(double alpha, int clamp)
{
    if (alpha < 0.0) alpha = 0.0;
    if (alpha > 1.0) alpha = 1.0;
    s_present_draw = 1;
    for (int i = 0; i < s_nobj; i++) {
        SceneObj *o = &s_objs[i];
        if (!o->present_ready) {
            o->present = o->world;
            o->present_armed = 1;
            continue;
        }
        double dx = o->tick_curr.t[0] - o->tick_prev.t[0];
        double dz = o->tick_curr.t[2] - o->tick_prev.t[2];
        int teleported = !isfinite(dx) || !isfinite(dz) ||
                         hypot(dx, dz) > SCENE_PRESENT_TELEPORT_M;
        if (clamp || teleported)
            o->present = o->tick_curr;
        else
            xform_nlerp(&o->present, &o->tick_prev, &o->tick_curr, alpha);
        o->present_armed = 1;
    }
}

/*
 * Give a placed object a car body (the melee opponent spawner).
 *
 * The 24 `.CBT` melee maps carry 226 class-1 `spawn` markers and no vehicle
 * of any kind, so their slots load with zero parts and draw nothing. This
 * runs the ordinary VCF->VDF->WDF placement chain against a DIFFERENT asset
 * name, keeping the marker's own world frame, so a slot becomes a real car
 * with real geometry through exactly the path every scripted mission's cars
 * already take. Nothing about the object's identity changes: the label, id,
 * class and team stay the marker's, which is what scene_obj_find, the AI
 * write-back and the hide-on-death path all key on.
 *
 * Returns the part count placed (0 = the asset did not resolve, and the
 * object is left with no parts), or -1 on a bad index/argument.
 */
int scene_obj_make_car(int obj, const char *vcf_base)
{
    if (!s_loaded || obj < 0 || obj >= s_nobj || !vcf_base || !*vcf_base)
        return -1;
    SceneObj *o = &s_objs[obj];
    free(o->parts);
    o->parts  = NULL;
    o->nparts = 0;
    o->vtf[0] = '\0';
    build_car_named(o, vcf_base);
    classify_parts(o);          /* fresh parts need fresh D11 flags */
    return o->nparts;
}

/*
 * Point a placed object along a horizontal direction, keeping its position.
 *
 * scene_obj_set_pos moves an object without turning it, which was invisible
 * while the only movers were P01's convoy driving a near-straight route. A
 * melee opponent circles the player, so a fixed basis reads as a car
 * sliding sideways.
 *
 * Takes a DIRECTION rather than an angle deliberately: the callers all have
 * a motion delta already, and sin/cos here would put a rendered pose at the
 * mercy of two libm implementations across the native/wasm frame gates. sqrt
 * is correctly rounded by IEEE-754.
 *
 * Returns 0, or -1 on a bad index or a degenerate direction (the object is
 * then left facing whichever way it already faced).
 */
int scene_obj_set_facing(int obj, double fx, double fz)
{
    return scene_obj_set_ground_facing(obj, fx, fz, 0.0, 1.0, 0.0);
}

int scene_obj_set_car_facing(int obj, double yaw, double pitch, double roll)
{
    if (obj < 0 || obj >= s_nobj || !isfinite(yaw) ||
        !isfinite(pitch) || !isfinite(roll))
        return -1;
    double r[3] = { cos(yaw), 0.0, sin(yaw) };
    double u[3] = { 0.0, 1.0, 0.0 };
    double f[3] = { -sin(yaw), 0.0, cos(yaw) };
    double cp = cos(pitch), sp = sin(pitch);
    double f1[3], u1[3];
    for (int i = 0; i < 3; i++) {
        f1[i] = f[i] * cp + u[i] * sp;
        u1[i] = u[i] * cp - f[i] * sp;
    }
    double cr = cos(roll), sr = sin(roll);
    Xform *w = &s_objs[obj].world;
    for (int i = 0; i < 3; i++) {
        w->m[i][0] = r[i] * cr + u1[i] * sr;
        w->m[i][1] = u1[i] * cr - r[i] * sr;
        w->m[i][2] = f1[i];
    }
    return 0;
}

int scene_obj_set_ground_facing(int obj, double fx, double fz,
                                double nx, double ny, double nz)
{
    if (obj < 0 || obj >= s_nobj)
        return -1;
    double fl = hypot(fx, fz);
    double nl = sqrt(nx * nx + ny * ny + nz * nz);
    if (!(fl > 1e-9) || !(nl > 1e-9))
        return -1;
    fx /= fl;
    fz /= fl;
    nx /= nl;
    ny /= nl;
    nz /= nl;
    if (ny < 0.0) { nx = -nx; ny = -ny; nz = -nz; }

    /* Project the horizontal heading onto the local tangent plane. Clamp the
     * normal to an upward 45-degree cone: terrain discontinuities must not
     * stand a kinematic car on end. The convention is the same right/up/
     * forward column frame used by every scene object (D4). */
    Xform *w = &s_objs[obj].world;
    if (ny < 0.7071067811865476) {
        double hl = hypot(nx, nz);
        if (hl > 1e-9) {
            nx = nx / hl * 0.7071067811865476;
            nz = nz / hl * 0.7071067811865476;
            ny = 0.7071067811865476;
        } else {
            nx = nz = 0.0;
            ny = 1.0;
        }
    }
    double dot = fx * nx + fz * nz;
    double fwd[3] = { fx - dot * nx, -dot * ny, fz - dot * nz };
    fl = sqrt(fwd[0] * fwd[0] + fwd[1] * fwd[1] + fwd[2] * fwd[2]);
    if (!(fl > 1e-9))
        return -1;
    for (int k = 0; k < 3; k++) fwd[k] /= fl;
    double right[3] = { ny * fwd[2] - nz * fwd[1],
                        nz * fwd[0] - nx * fwd[2],
                        nx * fwd[1] - ny * fwd[0] };
    double rl = sqrt(right[0] * right[0] + right[1] * right[1] +
                     right[2] * right[2]);
    if (!(rl > 1e-9))
        return -1;
    for (int k = 0; k < 3; k++) right[k] /= rl;

    double up[3] = { nx, ny, nz };
    for (int k = 0; k < 3; k++) {
        w->m[k][0] = right[k];
        w->m[k][1] = up[k];
        w->m[k][2] = fwd[k];
    }
    return 0;
}

/* M7 combat/story visibility (additive; no effect on existing paths —
 * objects default to hidden=0 and every pre-M7 call path leaves it so). */
int scene_obj_set_hidden(int obj, int hidden)
{
    if (obj < 0 || obj >= s_nobj)
        return -1;
    s_objs[obj].hidden = hidden ? 1 : 0;
    return 0;
}

int scene_obj_consume(int obj)
{
    if (obj < 0 || obj >= s_nobj)
        return -1;
    s_objs[obj].consumed = 1;
    return 0;
}

int scene_obj_hidden(int obj)
{
    if (obj < 0 || obj >= s_nobj)
        return 0;
    return s_objs[obj].hidden || s_objs[obj].consumed;
}

int scene_obj_consumed(int obj)
{
    return obj >= 0 && obj < s_nobj && s_objs[obj].consumed;
}

/* First class-7 gate part of an object, or -1 — the native FUN_0045a550
 * link walk returns the first class-7 child, and shipped models carry at
 * most one gate part each. */
static int gate_part_of(const SceneObj *o)
{
    for (int pi = 0; pi < o->nparts; pi++)
        if (o->parts[pi].pclass == SCENE_PART_CLASS_GATE)
            return pi;
    return -1;
}

int scene_obj_gate_state(int obj)
{
    if (obj < 0 || obj >= s_nobj)
        return -1;
    int pi = gate_part_of(&s_objs[obj]);
    return pi < 0 ? -1 : s_objs[obj].parts[pi].gate_state;
}

int scene_obj_trigger_gate(int obj)
{
    if (obj < 0 || obj >= s_nobj)
        return -1;
    int pi = gate_part_of(&s_objs[obj]);
    if (pi < 0)
        return -1;
    /* FUN_00456270: the native open/triggered transition zeroes the gate
     * component's state field; re-triggering rewrites the same zero. */
    if (s_objs[obj].parts[pi].gate_state != 0) {
        s_objs[obj].parts[pi].gate_state = 0;
        s_gate_generation++;
    }
    return 0;
}

void scene_gates_reset(void)
{
    int changed = 0;
    for (int oi = 0; oi < s_nobj; oi++)
        for (int pi = 0; pi < s_objs[oi].nparts; pi++)
            if (s_objs[oi].parts[pi].pclass == SCENE_PART_CLASS_GATE &&
                s_objs[oi].parts[pi].gate_state == 0) {
                s_objs[oi].parts[pi].gate_state = 1;
                changed = 1;
            }
    if (changed)
        s_gate_generation++;
}

unsigned scene_gate_generation(void)
{
    return s_gate_generation;
}
GeoMesh *scene_part_mesh(const char *part_name)
{
    if (!part_name) return NULL;
    char bare[16];
    copy_str(bare, sizeof bare, part_name);
    size_t n = strlen(bare);
    if (n > 4 && strcasecmp(bare + n - 4, ".geo") == 0)
        bare[n - 4] = '\0';
    if (!bare[0]) return NULL;
    int slot = mesh_get(bare);
    return slot < 0 ? NULL : s_mesh[slot].mesh;
}

/* ----------------------------------------------------------------------- */
/* Stats / unload                                                            */
/* ----------------------------------------------------------------------- */

int scene_first_marker_pos(double out[3])
{
    if (!out) return -1;
    for (int i = 0; i < s_nobj; i++) {
        if (s_objs[i].class_id == 1 && label_is_marker(s_objs[i].label)) {
            out[0] = s_objs[i].world.t[0];
            out[1] = s_objs[i].world.t[1];
            out[2] = s_objs[i].world.t[2];
            return 0;
        }
    }
    return -1;
}

int scene_stats(char *buf, size_t n)
{
    if (!buf || n == 0) return -1;
    if (!s_loaded)
        return snprintf(buf, n, "scene: nothing loaded");
    return snprintf(buf, n,
                    "%s: %d objects (%d cars, %d statics, %d string-inst, "
                    "%d markers, %d skipped), meshes %d ok %d missing, "
                    "extents x[%.0f..%.0f] y[%.0f..%.0f] z[%.0f..%.0f]",
                    s_mission, s_nobj, s_n_cars, s_n_statics, s_n_strings,
                    s_n_markers, s_n_skipped, s_mesh_ok, s_mesh_fail,
                    s_min[0], s_max[0], s_min[1], s_max[1], s_min[2], s_max[2]);
}

void scene_unload(void)
{
    s_present_draw = 0;
    terrain_drivable_clear();
    for (int i = 0; i < s_nobj; i++)
        free(s_objs[i].parts);
    free(s_objs);
    s_objs = NULL;
    s_nobj = s_objcap = 0;

    for (int i = 0; i < s_nmesh; i++) {
        if (s_mesh[i].from_cache) geo_cache_release(s_mesh[i].mesh);
        else                      geomesh_free(s_mesh[i].mesh);
        free(s_mesh[i].face_idx);
        s_mesh[i].face_idx = NULL;
        s_mesh[i].face_idx_gen = 0;
        for (TileSet *ts = s_mesh[i].tiles; ts; ) {
            TileSet *next = ts->next;
            free(ts->ids);
            free(ts);
            ts = next;
        }
        s_mesh[i].tiles = NULL;
    }
    s_nmesh = 0;
    s_mesh_ok = s_mesh_fail = 0;

    free(s_pix);
    s_pix = NULL;
    s_npix = s_pixcap = 0;
    s_pix_ready = 0;

    s_n_cars = s_n_statics = s_n_strings = s_n_markers = s_n_skipped = 0;
    s_mission[0] = '\0';
    s_dyn_count = 0;

    free(s_sky_pixels);
    s_sky_pixels = NULL;
    memset(&s_sky, 0, sizeof s_sky);
    s_sky_name[0] = '\0';
    s_sky_ok = 0;

    /* The .lum/.tbl buffers themselves are static and simply stop being
     * handed out; the generation bump is what tells the renderer's sync
     * that any previously pushed table is stale. */
    s_lum_name[0] = '\0';
    s_tbl_name[0] = '\0';
    s_lum_ok = s_tbl_ok = 0;
    s_table_gen++;

    for (int i = 0; i < SCENE_DYN_MAX; i++) {
        s_dyn_mesh[i] = NULL;
        free(s_dyn_tiles[i].ids);
        s_dyn_tiles[i].ids = NULL;
        s_dyn_tiles[i].faces = 0;
        s_dyn_tiles[i].mesh = NULL;
        s_dyn_tiles[i].vtf[0] = '\0';
        s_dyn_tiles[i].gen = 0;
    }
    s_dyn_vtf[0] = '\0';

    for (int i = 0; i < s_fx_effect_asset_n; i++)
        if (s_fx_effect_asset[i].mesh)
            geo_cache_release(s_fx_effect_asset[i].mesh);
    memset(s_fx_effect_asset, 0, sizeof s_fx_effect_asset);
    s_fx_effect_asset_n = 0;

    /* The tile cache holds decoded texels keyed by names from THIS mission's
     * archives and indices into THIS mission's level palette. Both change with
     * the mission, so it is released here rather than carried across. */
    texcache_reset();

    s_loaded = 0;
}
