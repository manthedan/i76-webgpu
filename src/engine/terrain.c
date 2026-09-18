/*
 * terrain.c — M2 terrain loader + spike renderer (see terrain.h)
 *
 * Loader path: mission BWD2 -> TDEF(ZMAP, ZONE) -> .ter blocks, all via the
 * engine VFS. Renderer: two spike-stage views into the 8-bit indexed fb:
 *   - top-down: point-sampled hillshade + height ramp over palette indices
 *     0/4/1/2/3 with 4x4 Bayer dithering, dim coast outlines and contour
 *     lines — always available;
 *   - perspective: wireframe heightfield from an orbiting pinhole camera —
 *     best-effort stand-in for the original's unknown terrain algorithm.
 *
 * Fidelity limits (docs/specs/m2/terrain.md §5, T1-T7):
 *   - filled path LOD is a 3-band ring scheme of our own (the original's
 *     algorithm is UNKNOWN); no distance fog (curve UNKNOWN);
 *   - adjacent present patches share the same sample-127 -> sample-0 seam
 *     cell used by both renderers; empty neighbours remain unclothed edges;
 *   - perspective view: binary-derived 90 deg horizontal FOV (raster.h),
 *     fixed elevation (original camera elevation UNKNOWN — pipeline P6);
 *     depth cue is a hard green->dim switch;
 *   - terrain flag nibble is parsed and counted in stats but not rendered
 *     (bit semantics T2 UNKNOWN).
 */
#include "terrain.h"
#include "engine/raster.h"
#include "engine/pixidx.h"
#include "vfs.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

/* Palette indices (meshview convention). */
#define IDX_BG     0
#define IDX_WIRE   1
#define IDX_VERT   2
#define IDX_TEXT   3
#define IDX_DIM    4

#define SAMPLES_PER_BLOCK (TERRAIN_PATCH_DIM * TERRAIN_PATCH_DIM)
#define GRID_SAMPLES      (TERRAIN_GRID_DIM * TERRAIN_PATCH_DIM)

/* Native class-11/12/13 drivable mesh-face registry. Scene owns the source
 * meshes; this module copies only the world-space collision data needed by
 * FUN_00423690's chassis query, so scene mesh-cache lifetime is irrelevant. */
typedef struct {
    double cx, cz, radius2;
} DrivableObject;

typedef struct {
    int object;
    int nverts;
    double plane[4];             /* unit world normal + d, n.p+d=0 */
    double *xz;                  /* [nverts*2], world polygon       */
} DrivableFace;

static struct {
    DrivableObject *objects;
    int nobjects, object_cap;
    DrivableFace *faces;
    int nfaces, face_cap;
} D;

/* -----------------------------------------------------------------------
 * State
 * ----------------------------------------------------------------------- */
static struct {
    char     mission[128];      /* mission path as passed to terrain_load  */
    char     ter_name[160];     /* resolved .ter path                      */
    uint8_t *blocks;            /* raw .ter bytes (nblocks * 32 KiB)       */
    int      nblocks;           /* .ter size / 32 KiB                      */
    int      zmap_count;        /* ZMAP count byte (blocks referenced)     */
    uint8_t  zone[TERRAIN_GRID_DIM * TERRAIN_GRID_DIM];
    int      used_cells;        /* non-0xFF, in-range zone cells           */
    int      cell_x0, cell_x1;  /* used bbox, patch units                  */
    int      cell_z0, cell_z1;
    double   hmin, hmax;        /* height range (m) over used cells        */
    long     flagged_samples;   /* samples with nonzero flag nibble        */
    int      view;              /* TERRAIN_VIEW_*                          */

    /* Mission surface texture (WDEF/WRLD +108; see load_named_tile). */
    char     surf_name[16];     /* entry name as named, e.g. "tt021sd_.map"*/
    RTex     surf;              /* decoded tile; valid iff surf_ok         */
    int      surf_ok;
    uint8_t *surf_pixels;       /* owned backing store for surf.texels     */

    /* Mission surface property table (ghidra-physics.md §Q15): 8 records
     * x 0x14 copied verbatim from the WRLD chunk at tag+0x9f. f08/f0c are
     * stored raw for fidelity but have no confirmed consumer. */
    struct {
        float   grip;           /* +0x00 wheel-force grip scale          */
        float   rr;             /* +0x04 rolling resistance              */
        float   f08;            /* +0x08 raw (no confirmed consumer)     */
        float   f0c;            /* +0x0c raw (no accessor exists)        */
        int32_t impact;         /* +0x10 landing impact/damage scale     */
    } surf_tab[8];
    int      surf_tab_ok;       /* complete table parsed this load       */

    /* RDEF/RSEG road ribbons (docs/specs/m2/scene.md §5). */
    struct RoadSeg *roads;
    int      nroads;
    long     road_pieces;       /* total pieces across accepted segments   */
    long     road_pieces_skip;  /* pieces dropped (bad length/type)        */
    unsigned road_generation;   /* changes on every road-world load        */
    RTex     road_tex[3];       /* paved / dirt / riverbed                 */
    int      road_tex_ok[3];
    uint8_t *road_pixels[3];    /* owned backing stores                    */
    long road_px_written;       /* last filled road pass                   */
    long road_px_rescued;       /* painter order beat a z rejection        */
    long road_px_occluded;      /* nearer terrain kept the pixel           */
} T;
static unsigned s_road_generation;

/* Gate-only road ownership diagnostics. Disabled in production: the browser
 * regression gate opts in before rendering its consecutive far-field pairs. */
static int      s_road_diag_enabled;
static float   *s_road_diag_footprint_z;
static uint8_t *s_road_diag_owner;
static size_t   s_road_diag_footprint_cap;
static size_t   s_road_diag_owner_cap;

static void road_diag_prepare(RTarget *t)
{
    if (!t) return;
    /* A target may be reused after diagnostics are disabled or an allocation
     * fails. Never leave prior-frame buffer ownership attached to it. */
    t->diag_road_footprint_z = NULL;
    t->diag_road_owner = NULL;
    if (!s_road_diag_enabled || t->w <= 0 || t->h <= 0) return;
    size_t need = (size_t)t->w * (size_t)t->h;
    if (s_road_diag_footprint_cap < need) {
        float *p = realloc(s_road_diag_footprint_z, need * sizeof *p);
        if (p) {
            s_road_diag_footprint_z = p;
            s_road_diag_footprint_cap = need;
        }
    }
    if (s_road_diag_owner_cap < need) {
        uint8_t *p = realloc(s_road_diag_owner, need);
        if (p) {
            s_road_diag_owner = p;
            s_road_diag_owner_cap = need;
        }
    }
    if (s_road_diag_footprint_cap < need || s_road_diag_owner_cap < need)
        return;
    memset(s_road_diag_footprint_z, 0, need * sizeof *s_road_diag_footprint_z);
    memset(s_road_diag_owner, 0, need);
    t->diag_road_footprint_z = s_road_diag_footprint_z;
    t->diag_road_owner = s_road_diag_owner;
}

/* One parsed RSEG: type + count + count x 6 f32 (left xyz, right xyz).
 * Y is CONFORMED to terrain at load (see conform_roads); the stored Y is
 * dropped -- Tony observed garbage/large values at some junction ends
 * (UNKNOWN), and Open76 recomputes Y at load too. */
struct RoadSeg {
    uint32_t type;              /* 0 paved, 1 dirt, 2 riverbed             */
    uint32_t count;
    float   *lr;                /* count x 6 floats                        */
};

/* -----------------------------------------------------------------------
 * Little-endian / string helpers
 * ----------------------------------------------------------------------- */
static uint16_t rd16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float rdf32(const uint8_t *p)
{
    uint32_t u = rd32(p);
    float f;
    memcpy(&f, &u, sizeof f);
    return f;
}
static void copy_str(char *dst, size_t cap, const char *src)
{
    size_t l = strlen(src);
    if (l >= cap) l = cap - 1;
    memcpy(dst, src, l);
    dst[l] = '\0';
}

/* -----------------------------------------------------------------------
 * BWD2 chunk walking (chunk = tag[4] | u32 total length | payload)
 * ----------------------------------------------------------------------- */

/* Find a sub-chunk by tag inside a chunk payload d[0..n). Returns the
 * payload offset (>0), or 0 when absent. *plen receives the payload size. */
static size_t chunk_find(const uint8_t *d, size_t n, const char tag[4],
                         size_t *plen)
{
    for (size_t off = 0; off + 8 <= n;) {
        uint32_t len = rd32(d + off + 4);
        if (len < 8 || (size_t)len > n - off) break;
        if (memcmp(d + off, tag, 4) == 0) {
            *plen = (size_t)len - 8;
            return off + 8;
        }
        off += len;
    }
    return 0;
}

/* Find a top-level chunk in a BWD2 stream. The observed Nitro layout puts
 * the chunk stream right after the 8-byte "BWD2" header chunk; if that walk
 * desyncs (e.g. the 28-byte header variant of scene.md §1), fall back to a
 * raw tag scan. */
static size_t bwd2_find(const uint8_t *d, size_t n, const char tag[4],
                        size_t *plen)
{
    size_t r = chunk_find(d + 8, n - 8, tag, plen);
    if (r) return 8 + r;
    for (size_t off = 8; off + 8 <= n; off++) {
        if (memcmp(d + off, tag, 4) != 0) continue;
        uint32_t len = rd32(d + off + 4);
        if (len >= 8 && (size_t)len <= n - off) {
            *plen = (size_t)len - 8;
            return off + 8;
        }
    }
    return 0;
}

/* -----------------------------------------------------------------------
 * Sample access
 * ----------------------------------------------------------------------- */

/* Raw sample at global sample coords (0..80*128 across the whole zone
 * grid). Out-of-grid and empty cells read as 0 (flat default ground). */
static uint16_t sample_global(int gx, int gz)
{
    if (gx < 0 || gz < 0 || gx >= GRID_SAMPLES || gz >= GRID_SAMPLES)
        return 0;
    int cx = gx / TERRAIN_PATCH_DIM, cz = gz / TERRAIN_PATCH_DIM;
    uint8_t idx = T.zone[cz * TERRAIN_GRID_DIM + cx];
    if (idx == TERRAIN_EMPTY_CELL || idx >= T.nblocks) return 0;
    const uint8_t *p = T.blocks +
        ((size_t)idx * SAMPLES_PER_BLOCK +
         (size_t)(gz % TERRAIN_PATCH_DIM) * TERRAIN_PATCH_DIM +
         (size_t)(gx % TERRAIN_PATCH_DIM)) * 2;
    return rd16(p);
}

static double height_global(int gx, int gz)
{
    return (double)(sample_global(gx, gz) & 0xFFF) * TERRAIN_HEIGHT_SCALE;
}

static int sample_present(int gx, int gz);

double terrain_height_at(double wx, double wz)
{
    if (!T.blocks) return 0.0;
    int cx = (int)floor(wx / TERRAIN_PATCH_SIZE_M);
    int cz = (int)floor(wz / TERRAIN_PATCH_SIZE_M);
    if (cx < 0 || cz < 0 || cx >= TERRAIN_GRID_DIM || cz >= TERRAIN_GRID_DIM)
        return 0.0;
    uint8_t idx = T.zone[cz * TERRAIN_GRID_DIM + cx];
    if (idx == TERRAIN_EMPTY_CELL || idx >= T.nblocks) return 0.0;

    double lx = (wx - cx * TERRAIN_PATCH_SIZE_M) / TERRAIN_SAMPLE_STEP_M;
    double lz = (wz - cz * TERRAIN_PATCH_SIZE_M) / TERRAIN_SAMPLE_STEP_M;
    if (lx < 0) lx = 0;
    else if (lx >= TERRAIN_PATCH_DIM)
        lx = nextafter((double)TERRAIN_PATCH_DIM, 0.0);
    if (lz < 0) lz = 0;
    else if (lz >= TERRAIN_PATCH_DIM)
        lz = nextafter((double)TERRAIN_PATCH_DIM, 0.0);
    int x0 = (int)floor(lx), z0 = (int)floor(lz);
    double fx = lx - x0, fz = lz - z0;

    int gx = cx * TERRAIN_PATCH_DIM + x0;
    int gz = cz * TERRAIN_PATCH_DIM + z0;
    int gx1 = gx + 1, gz1 = gz + 1;
    /* The last sample of one 640 m patch is at +635 m; the next patch's
     * sample zero is the +640 m corner of the intervening rendered cell.
     * The old x0/z0 clamp silently flattened that final five-metre strip,
     * then jumped at the patch boundary.  Weld only to PRESENT neighbours:
     * an authored empty ZMAP cell remains the historical flat/clamped edge. */
    if (!sample_present(gx1, gz)) gx1 = gx;
    if (!sample_present(gx, gz1)) gz1 = gz;
    /* Do not invent a diagonal surface at an L-shaped four-patch corner:
     * the renderer rejects that quad because its fourth sample is absent. */
    if (gx1 != gx && gz1 != gz && !sample_present(gx1, gz1)) {
        gx1 = gx;
        gz1 = gz;
    }
    double h00 = height_global(gx, gz);
    double h10 = height_global(gx1, gz);
    double h01 = height_global(gx, gz1);
    double h11 = height_global(gx1, gz1);
    double h0 = h00 + (h10 - h00) * fx;
    double h1 = h01 + (h11 - h01) * fx;
    return h0 + (h1 - h0) * fz;
}

static int segment_triangle_hit(const double a[3], const double b[3],
                                const double v0[3], const double v1[3],
                                const double v2[3], double *hit_t)
{
    double d[3] = {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
    double e1[3] = {v1[0]-v0[0], v1[1]-v0[1], v1[2]-v0[2]};
    double e2[3] = {v2[0]-v0[0], v2[1]-v0[1], v2[2]-v0[2]};
    double p[3] = {d[1]*e2[2]-d[2]*e2[1],
                   d[2]*e2[0]-d[0]*e2[2],
                   d[0]*e2[1]-d[1]*e2[0]};
    double det=e1[0]*p[0]+e1[1]*p[1]+e1[2]*p[2];
    if (fabs(det) < 1e-12) return 0;
    double inv=1.0/det;
    double s[3] = {a[0]-v0[0], a[1]-v0[1], a[2]-v0[2]};
    double u=(s[0]*p[0]+s[1]*p[1]+s[2]*p[2])*inv;
    if (u < 0.0 || u > 1.0) return 0;
    double q[3] = {s[1]*e1[2]-s[2]*e1[1],
                   s[2]*e1[0]-s[0]*e1[2],
                   s[0]*e1[1]-s[1]*e1[0]};
    double v=(d[0]*q[0]+d[1]*q[1]+d[2]*q[2])*inv;
    if (v < 0.0 || u+v > 1.0) return 0;
    double t=(e2[0]*q[0]+e2[1]*q[1]+e2[2]*q[2])*inv;
    if (t < 0.0 || t > 1.0) return 0;
    if (hit_t) *hit_t=t;
    return 1;
}

int terrain_segment_hit(const double a[3], const double b[3], double *hit_t)
{
    if (!T.blocks || !a || !b) return 0;
    double minx=fmin(a[0],b[0]), maxx=fmax(a[0],b[0]);
    double minz=fmin(a[2],b[2]), maxz=fmax(a[2],b[2]);
    int gx0=(int)floor(minx/TERRAIN_SAMPLE_STEP_M);
    int gx1=(int)floor(maxx/TERRAIN_SAMPLE_STEP_M);
    int gz0=(int)floor(minz/TERRAIN_SAMPLE_STEP_M);
    int gz1=(int)floor(maxz/TERRAIN_SAMPLE_STEP_M);
    if (gx0 < 0) gx0=0; if (gz0 < 0) gz0=0;
    if (gx1 >= GRID_SAMPLES-1) gx1=GRID_SAMPLES-2;
    if (gz1 >= GRID_SAMPLES-1) gz1=GRID_SAMPLES-2;
    if (gx0 > gx1 || gz0 > gz1) return 0;
    double best=2.0;
    for (int gz=gz0; gz<=gz1; gz++)
        for (int gx=gx0; gx<=gx1; gx++) {
            if (!sample_present(gx,gz) || !sample_present(gx+1,gz) ||
                !sample_present(gx,gz+1) || !sample_present(gx+1,gz+1))
                continue;
            double x=(double)gx*TERRAIN_SAMPLE_STEP_M;
            double z=(double)gz*TERRAIN_SAMPLE_STEP_M;
            double s=TERRAIN_SAMPLE_STEP_M;
            double v0[3]={x,height_global(gx,gz),z};
            double v1[3]={x,height_global(gx,gz+1),z+s};
            double v2[3]={x+s,height_global(gx+1,gz+1),z+s};
            double v3[3]={x+s,height_global(gx+1,gz),z};
            double t;
            if (segment_triangle_hit(a,b,v0,v1,v2,&t) && t<best) best=t;
            if (segment_triangle_hit(a,b,v0,v2,v3,&t) && t<best) best=t;
        }
    if (best > 1.0) return 0;
    if (hit_t) *hit_t=best;
    return 1;
}

/* Native FUN_004adf90 advances in one-metre increments and compares Y with
 * the bilinear FUN_004953e0 height field, then bisects the first crossing up
 * to nine times with a 0.02 m acceptance band.  This is deliberately distinct
 * from terrain_segment_hit's rendered source triangles: Nitro ordnance/LOS
 * uses the height query, not renderer tessellation.  The port bounds the last
 * increment to b rather than testing up to one metre beyond the requested
 * segment, preserving this API's documented t in [0,1]. */
int terrain_ordnance_segment_hit(const double a[3], const double b[3],
                                 double *hit_t)
{
    if (!T.blocks || !a || !b) return 0;
    double d[3]={b[0]-a[0],b[1]-a[1],b[2]-a[2]};
    double length=sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
    if(length<1e-12){
        int cx=(int)floor(a[0]/TERRAIN_PATCH_SIZE_M);
        int cz=(int)floor(a[2]/TERRAIN_PATCH_SIZE_M);
        if(cx>=0&&cz>=0&&cx<TERRAIN_GRID_DIM&&cz<TERRAIN_GRID_DIM&&
           T.zone[cz*TERRAIN_GRID_DIM+cx]!=TERRAIN_EMPTY_CELL&&
           T.zone[cz*TERRAIN_GRID_DIM+cx]<T.nblocks&&
           a[1]<=terrain_height_at(a[0],a[2])){
            if(hit_t)*hit_t=0.0;
            return 1;
        }
        return 0;
    }
    int steps=(int)ceil(length);
    if (steps < 1) steps=1;
    double prev_t=0.0;
    for(int k=0;k<=steps;k++){
        double t=k==steps?1.0:fmin((double)k/length,1.0);
        double x=a[0]+d[0]*t,z=a[2]+d[2]*t;
        int cx=(int)floor(x/TERRAIN_PATCH_SIZE_M);
        int cz=(int)floor(z/TERRAIN_PATCH_SIZE_M);
        /* Preserve the port's authored-empty-patch boundary: there is no
         * surface to hit where ZMAP has no terrain block. */
        if(cx<0||cz<0||cx>=TERRAIN_GRID_DIM||cz>=TERRAIN_GRID_DIM||
           T.zone[cz*TERRAIN_GRID_DIM+cx]==TERRAIN_EMPTY_CELL||
           T.zone[cz*TERRAIN_GRID_DIM+cx]>=T.nblocks){
            prev_t=t;
            continue;
        }
        double y=a[1]+d[1]*t;
        if(y<=terrain_height_at(x,z)){
            if(k==0){if(hit_t)*hit_t=0.0;return 1;}
            double lo=prev_t,hi=t,mid=hi;
            for(int n=0;n<9;n++){
                mid=0.5*(lo+hi);
                double mx=a[0]+d[0]*mid;
                double my=a[1]+d[1]*mid;
                double mz=a[2]+d[2]*mid;
                double clearance=my-terrain_height_at(mx,mz);
                if(clearance<0.02){
                    if(hit_t)*hit_t=mid;
                    return 1;
                }
                lo=mid;
            }
            if(hit_t)*hit_t=mid;
            return 1;
        }
        prev_t=t;
        if(t>=1.0)break;
    }
    return 0;
}

int terrain_drivable_object_add(double cx, double cz, double radius)
{
    if (!isfinite(cx) || !isfinite(cz) || !isfinite(radius) || radius < 0.0)
        return -1;
    if (D.nobjects == D.object_cap) {
        int cap = D.object_cap ? D.object_cap * 2 : 16;
        DrivableObject *p = realloc(D.objects, (size_t)cap * sizeof *p);
        if (!p) return -1;
        D.objects = p;
        D.object_cap = cap;
    }
    DrivableObject *o = &D.objects[D.nobjects];
    o->cx = cx;
    o->cz = cz;
    o->radius2 = radius * radius;
    return D.nobjects++;
}

int terrain_drivable_face_add(int object, const double plane[4],
                              const double *xz, int nverts)
{
    if (object < 0 || object >= D.nobjects || !plane || !xz ||
        nverts < 3 || nverts > 65535 || !isfinite(plane[0]) ||
        !isfinite(plane[1]) || !isfinite(plane[2]) ||
        !isfinite(plane[3]) || !(plane[1] > 0.4))
        return -1;
    double *poly = malloc((size_t)nverts * 2 * sizeof *poly);
    if (!poly) return -1;
    for (int i = 0; i < nverts * 2; i++) {
        if (!isfinite(xz[i])) {
            free(poly);
            return -1;
        }
        poly[i] = xz[i];
    }
    if (D.nfaces == D.face_cap) {
        int cap = D.face_cap ? D.face_cap * 2 : 32;
        DrivableFace *p = realloc(D.faces, (size_t)cap * sizeof *p);
        if (!p) {
            free(poly);
            return -1;
        }
        D.faces = p;
        D.face_cap = cap;
    }
    DrivableFace *f = &D.faces[D.nfaces++];
    f->object = object;
    f->nverts = nverts;
    memcpy(f->plane, plane, sizeof f->plane);
    f->xz = poly;
    return 0;
}

void terrain_drivable_clear(void)
{
    for (int i = 0; i < D.nfaces; i++) free(D.faces[i].xz);
    free(D.faces);
    free(D.objects);
    memset(&D, 0, sizeof D);
}

void terrain_drivable_stats(int *objects, int *faces)
{
    if (objects) *objects = D.nobjects;
    if (faces) *faces = D.nfaces;
}

static int point_on_xz_edge(double x, double z, double ax, double az,
                            double bx, double bz)
{
    double dx = bx - ax, dz = bz - az;
    double cross = (x - ax) * dz - (z - az) * dx;
    double scale = fabs(dx) + fabs(dz) + 1.0;
    if (fabs(cross) > 1e-9 * scale) return 0;
    return x >= fmin(ax, bx) - 1e-9 && x <= fmax(ax, bx) + 1e-9 &&
           z >= fmin(az, bz) - 1e-9 && z <= fmax(az, bz) + 1e-9;
}

/* XZ even/odd crossing with boundary inclusion, matching the native face
 * query's outcode/edge-crossing ownership rather than an OBB approximation. */
static int point_in_xz_face(const DrivableFace *f, double x, double z)
{
    int inside = 0;
    for (int i = 0, j = f->nverts - 1; i < f->nverts; j = i++) {
        double xi = f->xz[i * 2], zi = f->xz[i * 2 + 1];
        double xj = f->xz[j * 2], zj = f->xz[j * 2 + 1];
        if (point_on_xz_edge(x, z, xi, zi, xj, zj)) return 1;
        if ((zi > z) != (zj > z)) {
            double at_x = xi + (z - zi) * (xj - xi) / (zj - zi);
            if (x < at_x) inside = !inside;
        }
    }
    return inside;
}

static void closest_on_xz_edge(double x, double z, double ax, double az,
                               double bx, double bz, double *qx, double *qz)
{
    double dx = bx - ax, dz = bz - az;
    double d2 = dx * dx + dz * dz;
    double t = d2 > 0.0 ? ((x - ax) * dx + (z - az) * dz) / d2 : 0.0;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;
    *qx = ax + t * dx;
    *qz = az + t * dz;
}

int terrain_drivable_nearest_height(int object_token, double wx, double wz,
                                     double *height)
{
    if (!height || !isfinite(wx) || !isfinite(wz) || object_token <= 0)
        return -1;
    int object = object_token - 1;
    if (object < 0 || object >= D.nobjects) return 0;

    double best_d2 = 1e300, best_h = 0.0;
    int found = 0;
    for (int fi = 0; fi < D.nfaces; fi++) {
        const DrivableFace *f = &D.faces[fi];
        if (f->object != object) continue;
        double qx = wx, qz = wz, face_d2 = 0.0;
        if (!point_in_xz_face(f, wx, wz)) {
            face_d2 = 1e300;
            for (int i = 0, j = f->nverts - 1; i < f->nverts; j = i++) {
                double ex, ez;
                closest_on_xz_edge(wx, wz,
                                   f->xz[j * 2], f->xz[j * 2 + 1],
                                   f->xz[i * 2], f->xz[i * 2 + 1],
                                   &ex, &ez);
                double dx = wx - ex, dz = wz - ez;
                double d2 = dx * dx + dz * dz;
                if (d2 < face_d2) {
                    face_d2 = d2;
                    qx = ex;
                    qz = ez;
                }
            }
        }
        double h = -(f->plane[0] * qx + f->plane[2] * qz + f->plane[3]) /
                   f->plane[1];
        if (!isfinite(h)) continue;
        if (!found || face_d2 < best_d2 ||
            (face_d2 == best_d2 && h > best_h)) {
            best_d2 = face_d2;
            best_h = h;
            found = 1;
        }
    }
    if (!found) return 0;
    *height = best_h;
    return 1;
}

int terrain_drivable_object_probe(int object_token, double wx,
                                   double current_y, double wz,
                                   double *height, double normal[3])
{
    if (!height || !normal || !isfinite(wx) || !isfinite(wz) ||
        !isfinite(current_y) || object_token <= 0)
        return -1;
    int object = object_token - 1;
    if (object < 0 || object >= D.nobjects) return 0;
    const DrivableObject *o = &D.objects[object];
    double dx = wx - o->cx, dz = wz - o->cz;
    if (dx * dx + dz * dz > o->radius2) return 0;

    double band_h = -1e30;
    int band_face = -1;
    for (int fi = 0; fi < D.nfaces; fi++) {
        const DrivableFace *f = &D.faces[fi];
        if (f->object != object || !point_in_xz_face(f, wx, wz)) continue;
        double h = -(f->plane[0] * wx + f->plane[2] * wz + f->plane[3]) /
                   f->plane[1];
        if (!isfinite(h)) continue;
        if (current_y > h - 3.0 && current_y < h + 3.0 && h > band_h) {
            band_h = h;
            band_face = fi;
        }
    }
    if (band_face < 0) return 0;
    const DrivableFace *f = &D.faces[band_face];
    *height = band_h;
    normal[0] = f->plane[0];
    normal[1] = f->plane[1];
    normal[2] = f->plane[2];
    return 1;
}

static void terrain_normal_at(double x, double z, double normal[3])
{
    const double span = 0.5;
    double gx = (terrain_height_at(x + span, z) -
                 terrain_height_at(x - span, z)) / (2.0 * span);
    double gz = (terrain_height_at(x, z + span) -
                 terrain_height_at(x, z - span)) / (2.0 * span);
    double len = sqrt(gx * gx + 1.0 + gz * gz);
    normal[0] = -gx / len;
    normal[1] = 1.0 / len;
    normal[2] = -gz / len;
}

int terrain_drivable_probe(double wx, double current_y, double wz,
                           double *height, double normal[3])
{
    if (!height || !normal) return -1;
    double band_h = -1e30, below_h = -1e30;
    int band_face = -1, below_face = -1;

    for (int oi = 0; oi < D.nobjects; oi++) {
        const DrivableObject *o = &D.objects[oi];
        double dx = wx - o->cx, dz = wz - o->cz;
        if (dx * dx + dz * dz > o->radius2) continue;
        for (int fi = 0; fi < D.nfaces; fi++) {
            const DrivableFace *f = &D.faces[fi];
            if (f->object != oi || !point_in_xz_face(f, wx, wz)) continue;
            double h = -(f->plane[0] * wx + f->plane[2] * wz +
                         f->plane[3]) / f->plane[1];
            if (!isfinite(h)) continue;
            if (current_y > h - 3.0 && current_y < h + 3.0) {
                if (h > band_h) { band_h = h; band_face = fi; }
            } else if (h < current_y && h > below_h) {
                below_h = h;
                below_face = fi;
            }
        }
    }

    int chosen = band_face >= 0 ? band_face : below_face;
    if (chosen >= 0) {
        const DrivableFace *f = &D.faces[chosen];
        *height = band_face >= 0 ? band_h : below_h;
        normal[0] = f->plane[0];
        normal[1] = f->plane[1];
        normal[2] = f->plane[2];
        return 1;
    }
    *height = terrain_height_at(wx, wz);
    terrain_normal_at(wx, wz, normal);
    return 0;
}

int terrain_nav_sample(double wx, double wz,
                       unsigned *surface_class, int *blocked)
{
    if (!surface_class || !blocked || !T.blocks)
        return -1;

    /* Nitro FUN_00497790/FUN_004976d0 address the 5 m tile nearest the
     * requested world coordinate. Keep this discrete lookup separate from
     * terrain_height_at's bilinear presentation/grounding query. */
    int gx = (int)floor(wx / TERRAIN_SAMPLE_STEP_M + 0.5);
    int gz = (int)floor(wz / TERRAIN_SAMPLE_STEP_M + 0.5);
    if (gx < 0 || gz < 0 || gx >= GRID_SAMPLES || gz >= GRID_SAMPLES)
        return -1;
    int cx = gx / TERRAIN_PATCH_DIM, cz = gz / TERRAIN_PATCH_DIM;
    uint8_t idx = T.zone[cz * TERRAIN_GRID_DIM + cx];
    if (idx == TERRAIN_EMPTY_CELL || idx >= T.nblocks)
        return -1;

    uint16_t sample = sample_global(gx, gz);
    *surface_class = sample >> 13;
    *blocked = (sample >> 12) & 1;
    return 0;
}

int terrain_surface_props(unsigned surface_class, TerrainSurfaceProps *out)
{
    if (!out || surface_class > 7 || !T.blocks || !T.surf_tab_ok)
        return -1;
    out->grip   = T.surf_tab[surface_class].grip;
    out->rr     = T.surf_tab[surface_class].rr;
    out->impact = T.surf_tab[surface_class].impact;
    return 0;
}

/* -----------------------------------------------------------------------
 * Indexed .map tiles (mission surface + road ribbons)
 *
 * `.map` is the raw indexed bitmap of pipeline.md §3.1: u32 width, u32
 * height, then w*h LEVEL-palette indices. The mission names its terrain
 * surface texture in WDEF/WRLD +108 (scene.md §2, CONFIRMED layout); the
 * stored asset lives inside an m-tier .pak under the SAME .map name
 * (measured: tt021sd_.map in tt02m.pak, r2ayr_51.map in r2ayr1m.pak), so
 * resolution goes through pixidx, NOT through texcache -- texcache
 * implements the GEO-face rule "FOO.MAP -> FOO.VQM", which does not apply
 * to these files.
 *
 * ORIENTATION IS INFERRED, not verified: decoded row-major, row 0 = top
 * (i76img.py convention, matching vqm_decode and RTex.texels). Open76 reads
 * .map column-major and the P3/T4 experiment that decides which is right
 * has not been run; if it lands on transposed, this is the one place to
 * flip. Texels are level-palette indices, so no colour conversion exists.
 * ----------------------------------------------------------------------- */

/* Decode one .map into an RTex. POT 8..256 is REJECTED, not clamped: the
 * rasterizer wraps with `u & umask`, which is only defined for powers of
 * two (same contract texcache enforces). Returns 0 on success. */
static int map_tile_decode(const uint8_t *buf, size_t n, RTex *out,
                           uint8_t **pixels)
{
    *pixels = NULL;
    if (!buf || n < 8 + 64) return -1;
    uint32_t w = rd32(buf), h = rd32(buf + 4);
    if (w < 8 || h < 8 || w > 256 || h > 256) return -1;
    if ((w & (w - 1)) || (h & (h - 1))) return -1;
    if ((uint64_t)8 + (uint64_t)w * h > (uint64_t)n) return -1;
    uint8_t *px = malloc((size_t)w * h);
    if (!px) return -1;
    memcpy(px, buf + 8, (size_t)w * h);

    uint8_t vshift = 0;
    while ((1u << vshift) < w) vshift++;
    int has_key = 0;
    for (uint32_t i = 0; i < w * h; i++)
        if (px[i] == RASTER_TEXEL_TRANSPARENT) { has_key = 1; break; }

    out->texels  = px;
    out->w       = (uint16_t)w;
    out->h       = (uint16_t)h;
    out->umask   = (uint16_t)(w - 1);
    out->vmask   = (uint16_t)(h - 1);
    out->vshift  = vshift;
    out->has_key = (uint8_t)has_key;
    *pixels = px;
    return 0;
}

/* Shared m-tier index, built lazily on first texture request. texcache
 * keeps its OWN copy; duplicating the one-time build keeps terrain and the
 * face cache independent (either can load without the other). */
static PixIndex *terrain_mpix(void)
{
    static PixIndex *ix;
    static int tried;
    if (!tried) { ix = pixidx_build("m.pix"); tried = 1; }
    return ix;
}

/* Resolve entry `name` (e.g. "tt021sd_.map") to bytes and decode it.
 * PRECEDENCE mirrors texcache: m-tier .pix/.pak index first, then a loose
 * VFS file -- fixed rather than "first found" so a user asset dropped
 * beside the archives cannot silently change indexed resolution. */
static int load_named_tile(const char *name, RTex *out, uint8_t **pixels)
{
    char key[16];
    size_t k = 0;
    for (const char *p = name; *p && k + 1 < sizeof key; p++) {
        char c = *p;
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        key[k++] = c;
    }
    key[k] = '\0';
    if (k == 0) return -1;

    PixIndex *ix = terrain_mpix();
    const PixEnt *e = ix ? pixidx_find(ix, key) : NULL;
    if (e) {
        size_t psz = 0;
        uint8_t *pak = vfs_read_file(e->pak, &psz);
        if (!pak) return -1;
        int rc = -1;
        /* size_t is 32-bit under wasm: range-check in 64-bit. */
        if ((uint64_t)e->off + (uint64_t)e->len <= (uint64_t)psz)
            rc = map_tile_decode(pak + e->off, e->len, out, pixels);
        vfs_free(pak);
        return rc;
    }

    size_t sz = 0;
    uint8_t *loose = vfs_read_file(key, &sz);
    if (!loose) return -1;
    int rc = map_tile_decode(loose, sz, out, pixels);
    vfs_free(loose);
    return rc;
}

/* Per-RSEG-type road texture entry names. These are the names Open76's
 * RoadManager maps segmentType 0/1/2 to (INFERRED there, scene.md §5), and
 * each is CONFIRMED present in the shipped Nitro data (r2ayr1m.pak,
 * r2dnr7m.pak, r2wnr9m.pak; all 64x32). Whether nitro.exe binds the same
 * three is UNREVERSED -- treat as the best-grounded choice available. */
static const char *const ROAD_TEX_NAME[3] = {
    "r2ayr_51.map",     /* 0 paved highway   */
    "r2dnr_37.map",     /* 1 dirt track      */
    "r2wnr_39.map",     /* 2 river bed       */
};

static void load_terrain_textures(void)
{
    if (T.surf_name[0])
        T.surf_ok = load_named_tile(T.surf_name, &T.surf, &T.surf_pixels) == 0;
    for (int i = 0; i < 3; i++)
        T.road_tex_ok[i] =
            load_named_tile(ROAD_TEX_NAME[i], &T.road_tex[i],
                            &T.road_pixels[i]) == 0;
}

/* -----------------------------------------------------------------------
 * RDEF / RSEG road ribbons (docs/specs/m2/scene.md §5)
 * ----------------------------------------------------------------------- */

/* nitro.exe FUN_00493cc0 replaces both RSEG endpoint Ys with
 * FUN_004953e0(x,z). That sampler bilinearly interpolates the raw 12-bit
 * height and multiplies by 0.1; it does NOT add another 0.1 m. Open76's
 * `GetInterpolatedHeight(...) + 0.1f` is therefore not native policy. */
#define ROAD_LIFT_M 0.0

static int road_seg_parse(const uint8_t *p, size_t n, struct RoadSeg *out)
{
    if (n < 8) return -1;
    out->type  = rd32(p);
    out->count = rd32(p + 4);
    /* Total chunk payload = 24*pieceCount + 8 (CONFIRMED, scene.md §5). */
    if ((uint64_t)out->count * 24 + 8 != (uint64_t)n) return -1;
    if (out->count > (1u << 20)) return -1;     /* corrupt-input bound */
    out->lr = malloc((size_t)out->count * 6 * sizeof(float));
    if (!out->lr) return -1;
    memcpy(out->lr, p + 8, (size_t)out->count * 24);
    return 0;
}

/* Conform every road point to terrain_height_at + ROAD_LIFT_M. The stored
 * Y is discarded: Tony observed unexplained large values at some junction
 * ends (UNKNOWN), Open76 recomputes Y at load, and scene.md §5 tells M2 to
 * conform all road Y to terrain anyway. */
static void conform_roads(void)
{
    for (int s = 0; s < T.nroads; s++) {
        struct RoadSeg *r = &T.roads[s];
        for (uint32_t i = 0; i < r->count; i++) {
            float *pt = r->lr + (size_t)i * 6;
            pt[1] = (float)(terrain_height_at(pt[0], pt[2]) + ROAD_LIFT_M);
            pt[4] = (float)(terrain_height_at(pt[3], pt[5]) + ROAD_LIFT_M);
        }
    }
}

static void parse_wdef_rdef(const uint8_t *m, size_t msize)
{
    /* WDEF/WRLD: terrain surface texture name at payload +108, 13-byte
     * null-padded field (scene.md §2, CONFIRMED). */
    size_t wdef_len = 0;
    size_t wdef = bwd2_find(m, msize, "WDEF", &wdef_len);
    if (wdef) {
        size_t wrld_len = 0;
        size_t wrld = chunk_find(m + wdef, wdef_len, "WRLD", &wrld_len);
        if (wrld && wrld_len >= 121) {
            size_t l = 0;
            while (l < 13 && m[wdef + wrld + 108 + l]) l++;
            if (l >= sizeof(T.surf_name)) l = sizeof(T.surf_name) - 1;
            for (size_t i = 0; i < l; i++) {
                char c = (char)m[wdef + wrld + 108 + i];
                if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
                T.surf_name[i] = c;
            }
            T.surf_name[l] = '\0';
        }

        /* §Q15 surface property table: 8 records x 0x14 at WRLD payload
         * +0x97 (= chunk tag +0x9f, the original's verbatim copy source
         * for its runtime table @0x5fb500). All-or-nothing: an absent or
         * short WRLD leaves surf_tab_ok = 0 and consumers keep their
         * fallback profile; a complete table is never partially read. */
        if (wrld && wrld_len >= 0x97 + 8 * 0x14) {
            const uint8_t *rec = m + wdef + wrld + 0x97;
            for (int i = 0; i < 8; i++) {
                T.surf_tab[i].grip   = rdf32(rec + (size_t)i * 0x14 + 0x00);
                T.surf_tab[i].rr     = rdf32(rec + (size_t)i * 0x14 + 0x04);
                T.surf_tab[i].f08    = rdf32(rec + (size_t)i * 0x14 + 0x08);
                T.surf_tab[i].f0c    = rdf32(rec + (size_t)i * 0x14 + 0x0c);
                T.surf_tab[i].impact = (int32_t)rd32(rec + (size_t)i * 0x14 + 0x10);
            }
            T.surf_tab_ok = 1;
        }
    }

    /* RDEF: RREV, then RSEG chunks until EXIT (scene.md §5). */
    size_t rd_len = 0;
    size_t rd = bwd2_find(m, msize, "RDEF", &rd_len);
    if (!rd) return;
    const uint8_t *p = m + rd;
    int cap = 0;
    for (size_t off = 0; off + 8 <= rd_len;) {
        uint32_t len = rd32(p + off + 4);
        if (len < 8 || (size_t)len > rd_len - off) break;
        if (memcmp(p + off, "EXIT", 4) == 0) break;
        if (memcmp(p + off, "RSEG", 4) == 0) {
            struct RoadSeg seg;
            memset(&seg, 0, sizeof seg);
            if (road_seg_parse(p + off + 8, (size_t)len - 8, &seg) == 0) {
                /* segmentType 3072 ("four-lane") exists once in the corpus
                 * (T05) with ZERO pieces; types outside 0..2 carry no
                 * grounded texture and are skipped + counted, not guessed. */
                if (seg.type <= 2) {
                    if (T.nroads == cap) {
                        cap = cap ? cap * 2 : 8;
                        struct RoadSeg *nr = realloc(T.roads,
                            (size_t)cap * sizeof *nr);
                        if (!nr) { free(seg.lr); T.road_pieces_skip += seg.count; break; }
                        T.roads = nr;
                    }
                    T.roads[T.nroads++] = seg;
                    T.road_pieces += seg.count;
                } else {
                    T.road_pieces_skip += seg.count;
                    free(seg.lr);
                }
            } else {
                T.road_pieces_skip++;
            }
        }
        off += len;
    }
}

/* -----------------------------------------------------------------------
 * Loader
 * ----------------------------------------------------------------------- */
static void compute_stats(void)
{
    T.used_cells = 0;
    T.cell_x0 = T.cell_z0 = TERRAIN_GRID_DIM;
    T.cell_x1 = T.cell_z1 = -1;
    T.hmin = 1e9;
    T.hmax = 0.0;
    T.flagged_samples = 0;
    for (int cz = 0; cz < TERRAIN_GRID_DIM; cz++) {
        for (int cx = 0; cx < TERRAIN_GRID_DIM; cx++) {
            uint8_t idx = T.zone[cz * TERRAIN_GRID_DIM + cx];
            if (idx == TERRAIN_EMPTY_CELL || idx >= T.nblocks) continue;
            T.used_cells++;
            if (cx < T.cell_x0) T.cell_x0 = cx;
            if (cx > T.cell_x1) T.cell_x1 = cx;
            if (cz < T.cell_z0) T.cell_z0 = cz;
            if (cz > T.cell_z1) T.cell_z1 = cz;
            const uint8_t *b = T.blocks + (size_t)idx * SAMPLES_PER_BLOCK * 2;
            for (int i = 0; i < SAMPLES_PER_BLOCK; i++) {
                uint16_t v = rd16(b + (size_t)i * 2);
                double hm = (double)(v & 0xFFF) * TERRAIN_HEIGHT_SCALE;
                if (hm < T.hmin) T.hmin = hm;
                if (hm > T.hmax) T.hmax = hm;
                if (v >> 12) T.flagged_samples++;
            }
        }
    }
    if (T.used_cells == 0) {
        T.hmin = T.hmax = 0.0;
        T.cell_x0 = T.cell_x1 = T.cell_z0 = T.cell_z1 = 0;
    }
}

int terrain_load(const char *name)
{
    terrain_unload();

    size_t msize = 0;
    uint8_t *m = vfs_read_file(name, &msize);
    if (!m) return -1;
    if (msize < 8 || memcmp(m, "BWD2", 4) != 0) { vfs_free(m); return -1; }

    size_t tdef_len = 0;
    size_t tdef = bwd2_find(m, msize, "TDEF", &tdef_len);
    if (!tdef) { vfs_free(m); return -1; }

    size_t zm_len = 0, zn_len = 0;
    const uint8_t *td = m + tdef;   /* TDEF payload: TREV ZMAP ZONE EXIT */
    size_t zm = chunk_find(td, tdef_len, "ZMAP", &zm_len);
    size_t zn = chunk_find(td, tdef_len, "ZONE", &zn_len);
    if (!zm || zm_len < 1 + (size_t)(TERRAIN_GRID_DIM * TERRAIN_GRID_DIM)) {
        vfs_free(m);
        return -1;
    }
    T.zmap_count = td[zm];
    memcpy(T.zone, td + zm + 1, sizeof(T.zone));

    /* WDEF (surface texture name) and RDEF (road ribbons) ride the same
     * mission buffer; parse them before it is freed below. */
    parse_wdef_rdef(m, msize);
    T.road_generation = ++s_road_generation;

    char ter_name[16] = {0};
    if (zn && zn_len >= 14) {
        memcpy(ter_name, td + zn + 1, 13);  /* +0 = unknown byte (T5) */
        ter_name[13] = '\0';
    } else {
        /* No ZONE chunk: derive <mission-basename>.ter (spec §3.2). */
        const char *base = strrchr(name, '/');
        base = base ? base + 1 : name;
        copy_str(ter_name, sizeof(ter_name), base);
        char *dot = strrchr(ter_name, '.');
        if (dot) *dot = '\0';
        size_t l = strlen(ter_name);
        if (l + 4 < sizeof(ter_name)) memcpy(ter_name + l, ".ter", 5);
    }

    /* Resolve the .ter relative to the mission's directory first, then
     * bare (the original mounts its mission dirs into the data path; the
     * VFS only has the asset root, so we prefix explicitly). */
    size_t tsize = 0;
    uint8_t *ter = NULL;
    char ter_path[256];
    const char *slash = strrchr(name, '/');
    if (slash) {
        size_t dirlen = (size_t)(slash - name);
        size_t bl = strlen(ter_name);
        if (dirlen + 1 + bl + 1 > sizeof(ter_path))
            bl = sizeof(ter_path) - dirlen - 2;
        memcpy(ter_path, name, dirlen);
        ter_path[dirlen] = '/';
        memcpy(ter_path + dirlen + 1, ter_name, bl);
        ter_path[dirlen + 1 + bl] = '\0';
        ter = vfs_read_file(ter_path, &tsize);
    }
    if (!ter) {
        copy_str(ter_path, sizeof(ter_path), ter_name);
        ter = vfs_read_file(ter_path, &tsize);
    }
    vfs_free(m);
    if (!ter) return -1;
    if (tsize < TERRAIN_BLOCK_BYTES || tsize % TERRAIN_BLOCK_BYTES != 0) {
        vfs_free(ter);
        return -1;
    }

    T.blocks = ter;
    T.nblocks = (int)(tsize / TERRAIN_BLOCK_BYTES);
    copy_str(T.mission, sizeof(T.mission), name);
    copy_str(T.ter_name, sizeof(T.ter_name), ter_path);
    compute_stats();
    conform_roads();          /* needs T.blocks/T.zone in place */
    load_terrain_textures();  /* soft-fail: missing tiles -> flat fallback */
    return 0;
}

void terrain_unload(void)
{
    terrain_drivable_clear();
    if (T.blocks) vfs_free(T.blocks);
    free(T.surf_pixels);
    for (int i = 0; i < 3; i++) free(T.road_pixels[i]);
    for (int i = 0; i < T.nroads; i++) free(T.roads[i].lr);
    free(T.roads);
    memset(&T, 0, sizeof(T));
    T.view = TERRAIN_VIEW_TOPDOWN;
}

int terrain_is_loaded(void)
{
    return T.blocks != NULL;
}

int terrain_used_bounds(double *wx0, double *wz0,
                        double *wx1, double *wz1)
{
    if (!T.blocks || T.used_cells == 0 ||
        !wx0 || !wz0 || !wx1 || !wz1)
        return -1;
    *wx0 = T.cell_x0 * TERRAIN_PATCH_SIZE_M;
    *wz0 = T.cell_z0 * TERRAIN_PATCH_SIZE_M;
    *wx1 = (T.cell_x1 + 1) * TERRAIN_PATCH_SIZE_M;
    *wz1 = (T.cell_z1 + 1) * TERRAIN_PATCH_SIZE_M;
    return 0;
}

long terrain_road_pieces(void)
{
    return T.road_pieces;
}

int terrain_road_point(int seg, unsigned piece, double lr[6])
{
    if (seg < 0 || seg >= T.nroads || piece >= T.roads[seg].count || !lr)
        return -1;
    const float *p = T.roads[seg].lr + (size_t)piece * 6;
    for (int i = 0; i < 6; i++) lr[i] = p[i];
    return 0;
}

double terrain_road_nearest(double wx, double wz, int *segment,
                            double nearest_xz[2], double tangent_xz[2])
{
    /* Fixed-size exact-coordinate cache. Planner/smoother queries revisit
     * grid destinations heavily; rescanning every RSEG piece made broad
     * coverage watchdog-bound on road-dense missions. Deterministic direct
     * map, no allocation and no simulation-visible approximation. */
    typedef struct { double x,z,clear,nx,nz,tx,tz; int seg,valid; } RoadNear;
    static RoadNear cache[1024];
    static unsigned cache_generation;
    if (cache_generation != T.road_generation) {
        memset(cache, 0, sizeof cache);
        cache_generation = T.road_generation;
    }
    unsigned slot = ((unsigned)(long long)llround(wx * 2.0) * 0x9e3779b1u ^
                     (unsigned)(long long)llround(wz * 2.0) * 0x85ebca6bu) & 1023u;
    RoadNear *hit=&cache[slot];
    if (hit->valid && hit->x==wx && hit->z==wz) {
        if(segment)*segment=hit->seg;
        if(nearest_xz){nearest_xz[0]=hit->nx;nearest_xz[1]=hit->nz;}
        if(tangent_xz){tangent_xz[0]=hit->tx;tangent_xz[1]=hit->tz;}
        return hit->clear;
    }
    double best = 1e30;
    int best_seg = -1;
    double best_x = 0.0, best_z = 0.0, best_tx = 0.0, best_tz = 0.0;
    for (int s = 0; s < T.nroads; s++) {
        const struct RoadSeg *r = &T.roads[s];
        for (unsigned k = 0; k + 1 < r->count; k++) {
            const float *a = r->lr + (size_t)k * 6;
            const float *b = a + 6;
            double ax = ((double)a[0] + a[3]) * 0.5;
            double az = ((double)a[2] + a[5]) * 0.5;
            double bx = ((double)b[0] + b[3]) * 0.5;
            double bz = ((double)b[2] + b[5]) * 0.5;
            double dx = bx - ax, dz = bz - az;
            double l2 = dx * dx + dz * dz;
            double u = l2 > 1e-12
                     ? ((wx - ax) * dx + (wz - az) * dz) / l2 : 0.0;
            if (u < 0.0) u = 0.0;
            if (u > 1.0) u = 1.0;
            double px = ax + u * dx, pz = az + u * dz;
            /* Exact signed distance to this finite authored trapezoid:
             * a-left, b-left, b-right, a-right. This includes both caps and
             * does not assume parallel/equal-width side edges. */
            double qx[4] = { a[0], b[0], b[3], a[3] };
            double qz[4] = { a[2], b[2], b[5], a[5] };
            double edge_min = 1e30;
            int pos = 0, neg = 0;
            for (int e = 0; e < 4; e++) {
                int j = (e + 1) & 3;
                double ex = qx[j] - qx[e], ez = qz[j] - qz[e];
                double el2 = ex * ex + ez * ez;
                double eu = el2 > 1e-12
                          ? ((wx - qx[e]) * ex + (wz - qz[e]) * ez) / el2
                          : 0.0;
                if (eu < 0.0) eu = 0.0;
                if (eu > 1.0) eu = 1.0;
                double dist = hypot(wx - (qx[e] + eu * ex),
                                    wz - (qz[e] + eu * ez));
                int external = e == 0 || e == 2 ||
                               (e == 3 && k == 0) ||
                               (e == 1 && k + 2 == r->count);
                if (external && dist < edge_min) edge_min = dist;
                double cross = ex * (wz - qz[e]) - ez * (wx - qx[e]);
                if (cross > 1e-9) pos = 1;
                if (cross < -1e-9) neg = 1;
            }
            double clear = !(pos && neg) ? -edge_min : edge_min;
            if (clear < best) {
                best = clear;
                best_seg = s;
                best_x = px;
                best_z = pz;
                double dl = sqrt(l2);
                best_tx = dl > 1e-9 ? dx / dl : 0.0;
                best_tz = dl > 1e-9 ? dz / dl : 0.0;
            }
        }
    }
    *hit=(RoadNear){wx,wz,best,best_x,best_z,best_tx,best_tz,best_seg,1};
    if (segment) *segment = best_seg;
    if (nearest_xz) { nearest_xz[0] = best_x; nearest_xz[1] = best_z; }
    if (tangent_xz) { tangent_xz[0] = best_tx; tangent_xz[1] = best_tz; }
    return best;
}

void terrain_set_view(int mode)
{
    T.view = mode;
}

int terrain_view(void)
{
    return T.view;
}

int terrain_stats(char *buf, size_t n)
{
    if (!T.blocks)
        return snprintf(buf, n, "terrain: not loaded");
    double ex = T.used_cells ? (T.cell_x1 - T.cell_x0 + 1) * TERRAIN_PATCH_SIZE_M : 0;
    double ez = T.used_cells ? (T.cell_z1 - T.cell_z0 + 1) * TERRAIN_PATCH_SIZE_M : 0;
    int r = snprintf(buf, n,
        "terrain: mission=%s ter=%s blocks=%d zmap_count=%d used=%d "
        "bbox=[%d..%d]x[%d..%d] extent=%.0fx%.0fm h=%.1f..%.1fm "
        "flagged=%ld view=%s",
        T.mission, T.ter_name, T.nblocks, T.zmap_count, T.used_cells,
        T.cell_x0, T.cell_x1, T.cell_z0, T.cell_z1, ex, ez,
        T.hmin, T.hmax, T.flagged_samples,
        T.view == TERRAIN_VIEW_PERSPECTIVE ? "perspective" : "topdown");
    int used = (r > 0 && (size_t)r < n) ? r : 0;
    return used + snprintf(buf + used, n - (size_t)used,
        " surf=%s%s surftab=%s roads=%dsegs/%ldpieces(skip=%ld) roadtex=%d%d%d",
        T.surf_name[0] ? T.surf_name : "none",
        T.surf_ok ? "" : "(missing)",
        T.surf_tab_ok ? "8rec(Q15)" : "none",
        T.nroads, T.road_pieces, T.road_pieces_skip,
        T.road_tex_ok[0], T.road_tex_ok[1], T.road_tex_ok[2]);
}

/* -----------------------------------------------------------------------
 * Framebuffer primitives (spike-local; meshview's are static to it)
 * ----------------------------------------------------------------------- */
static void draw_line(uint8_t *fb, int w, int h,
                      int x0, int y0, int x1, int y1, uint8_t col)
{
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if ((unsigned)x0 < (unsigned)w && (unsigned)y0 < (unsigned)h)
            fb[(size_t)y0 * (size_t)w + (size_t)x0] = col;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/* -----------------------------------------------------------------------
 * Top-down shaded heightmap
 * ----------------------------------------------------------------------- */

/* Height ramp over the meshview indices: dark bg -> dim -> green -> yellow
 * -> white, ordered-dithered for intermediate shades. */
static const uint8_t RAMP[] = { IDX_BG, IDX_DIM, IDX_WIRE, IDX_VERT, IDX_TEXT };
#define RAMP_LAST 4   /* RAMP count - 1 */
static const uint8_t BAYER4[16] = {
     0,  8,  2, 10,
    12,  4, 14,  6,
     3, 11,  1,  9,
    15,  7, 13,  5
};

static uint8_t shade_pixel(double hgt, double inten, int px, int py)
{
    double range = T.hmax - T.hmin;
    double t = range > 0 ? (hgt - T.hmin) / range : 0.5;
    double v = (0.30 + 0.70 * t) * (0.40 + 0.60 * inten);
    if (v < 0) v = 0; else if (v > 1) v = 1;
    double level = v * RAMP_LAST;
    int base = (int)level;
    if (base > RAMP_LAST - 1) base = RAMP_LAST - 1;
    double frac = level - base;
    int th = BAYER4[(py & 3) * 4 + (px & 3)];
    return RAMP[base + (frac * 16.0 > (double)th + 0.5 ? 1 : 0)];
}

/* Light from above-west-northwest, normalized at use site. */
#define LIGHT_X (-0.6)
#define LIGHT_Y   0.9
#define LIGHT_Z (-0.4)

static void render_topdown(uint8_t *fb, int w, int h)
{
    if (!T.blocks || T.used_cells == 0) return;

    int pw = (T.cell_x1 - T.cell_x0 + 1) * TERRAIN_PATCH_DIM;
    int ph = (T.cell_z1 - T.cell_z0 + 1) * TERRAIN_PATCH_DIM;
    double scale = fmin((double)(w - 8) / pw, (double)(h - 8) / ph);
    if (scale <= 0) return;
    int rw = (int)(pw * scale), rh = (int)(ph * scale);
    if (rw < 1 || rh < 1) return;
    int ox = (w - rw) / 2, oy = (h - rh) / 2;

    for (int y = 0; y < rh; y++) {
        for (int x = 0; x < rw; x++) {
            /* North-up: screen row 0 = highest sample z. */
            int gx = T.cell_x0 * TERRAIN_PATCH_DIM + (int)(x / scale);
            int gz = T.cell_z0 * TERRAIN_PATCH_DIM + (int)((rh - 1 - y) / scale);
            double hc  = height_global(gx, gz);
            double hxe = height_global(gx + 1, gz);
            double hxw = height_global(gx - 1, gz);
            double hzn = height_global(gx, gz + 1);
            double hzs = height_global(gx, gz - 1);
            double dhdx = (hxe - hxw) / (2.0 * TERRAIN_SAMPLE_STEP_M);
            double dhdz = (hzn - hzs) / (2.0 * TERRAIN_SAMPLE_STEP_M);
            double nl = sqrt(dhdx * dhdx + 1.0 + dhdz * dhdz);
            double inten = (-dhdx * LIGHT_X + LIGHT_Y - dhdz * LIGHT_Z) / nl;
            if (inten < 0) inten = 0; else if (inten > 1) inten = 1;
            uint8_t col = shade_pixel(hc, inten, ox + x, oy + y);
            /* Contour every 8 m, dim — topo-map legibility aid. */
            if ((int)(hc / 8.0) != (int)(hxe / 8.0) ||
                (int)(hc / 8.0) != (int)(hzn / 8.0))
                col = IDX_DIM;
            fb[(size_t)(oy + y) * (size_t)w + (size_t)(ox + x)] = col;
        }
    }

    /* Coast outline: borders of used cells facing empty/out-of-grid cells. */
    for (int cz = T.cell_z0; cz <= T.cell_z1; cz++) {
        for (int cx = T.cell_x0; cx <= T.cell_x1; cx++) {
            uint8_t idx = T.zone[cz * TERRAIN_GRID_DIM + cx];
            if (idx == TERRAIN_EMPTY_CELL || idx >= T.nblocks) continue;
            int sx0 = ox + (int)((cx - T.cell_x0) * TERRAIN_PATCH_DIM * scale);
            int sx1 = ox + (int)((cx - T.cell_x0 + 1) * TERRAIN_PATCH_DIM * scale) - 1;
            int sy0 = oy + (int)((T.cell_z1 - cz) * TERRAIN_PATCH_DIM * scale);
            int sy1 = oy + (int)((T.cell_z1 - cz + 1) * TERRAIN_PATCH_DIM * scale) - 1;
            int west_empty  = cx == 0 ||
                T.zone[cz * TERRAIN_GRID_DIM + cx - 1] == TERRAIN_EMPTY_CELL;
            int east_empty  = cx == TERRAIN_GRID_DIM - 1 ||
                T.zone[cz * TERRAIN_GRID_DIM + cx + 1] == TERRAIN_EMPTY_CELL;
            int south_empty = cz == 0 ||
                T.zone[(cz - 1) * TERRAIN_GRID_DIM + cx] == TERRAIN_EMPTY_CELL;
            int north_empty = cz == TERRAIN_GRID_DIM - 1 ||
                T.zone[(cz + 1) * TERRAIN_GRID_DIM + cx] == TERRAIN_EMPTY_CELL;
            if (west_empty)  draw_line(fb, w, h, sx0, sy0, sx0, sy1, IDX_DIM);
            if (east_empty)  draw_line(fb, w, h, sx1, sy0, sx1, sy1, IDX_DIM);
            if (north_empty) draw_line(fb, w, h, sx0, sy0, sx1, sy0, IDX_DIM);
            if (south_empty) draw_line(fb, w, h, sx0, sy1, sx1, sy1, IDX_DIM);
        }
    }
}

/* -----------------------------------------------------------------------
 * Perspective wireframe heightfield
 * ----------------------------------------------------------------------- */
typedef struct {
    double px, py, pz;      /* camera position (world, meters) */
    double fx, fy, fz;      /* forward unit vector             */
    double rx, ry, rz;      /* right unit vector               */
    double ux, uy, uz;      /* up unit vector                  */
    double f;               /* focal length (pixels)           */
    double cx, cy;          /* screen center                   */
    double near, far;       /* near clip / draw distance       */
} Cam;

static void camspace(const Cam *c, double wx, double wy, double wz,
                     double *xs, double *ys, double *zs)
{
    double dx = wx - c->px, dy = wy - c->py, dz = wz - c->pz;
    *xs = dx * c->rx + dy * c->ry + dz * c->rz;
    *ys = dx * c->ux + dy * c->uy + dz * c->uz;
    *zs = dx * c->fx + dy * c->fy + dz * c->fz;
}

/* Draw one camera-space segment, clipped to the near plane, culled past
 * `far`; near half = green, far half = dim (fog curve UNKNOWN, pipeline
 * §6, so depth cue is a hard switch). */
static void draw_seg(uint8_t *fb, int w, int h, const Cam *c,
                     double ax, double ay, double az,
                     double bx, double by, double bz)
{
    if (az < c->near && bz < c->near) return;
    if (az < c->near) {
        double t = (c->near - az) / (bz - az);
        ax += (bx - ax) * t; ay += (by - ay) * t; az = c->near;
    } else if (bz < c->near) {
        double t = (c->near - bz) / (az - bz);
        bx += (ax - bx) * t; by += (ay - by) * t; bz = c->near;
    }
    double zm = 0.5 * (az + bz);
    if (zm > c->far) return;
    uint8_t col = zm < c->far * 0.55 ? IDX_WIRE : IDX_DIM;
    int x0 = (int)(c->cx + c->f * ax / az);
    int y0 = (int)(c->cy - c->f * ay / az);
    int x1 = (int)(c->cx + c->f * bx / bz);
    int y1 = (int)(c->cy - c->f * by / bz);
    if ((x0 < 0 && x1 < 0) || (x0 >= w && x1 >= w) ||
        (y0 < 0 && y1 < 0) || (y0 >= h && y1 >= h)) return;
    draw_line(fb, w, h, x0, y0, x1, y1, col);
}

/* Polyline strip across one patch row or column of the heightfield. */
static void draw_strip(uint8_t *fb, int w, int h, const Cam *c,
                       int cellx, int cellz, int along_x, int fixed)
{
    double px = 0, py = 0, pz = 0;
    int have_prev = 0;
    for (int i = 0; i < TERRAIN_PATCH_DIM; i += 2) {
        int lx = along_x ? i : fixed;
        int lz = along_x ? fixed : i;
        double wx = cellx * TERRAIN_PATCH_SIZE_M + lx * TERRAIN_SAMPLE_STEP_M;
        double wz = cellz * TERRAIN_PATCH_SIZE_M + lz * TERRAIN_SAMPLE_STEP_M;
        double wy = height_global(cellx * TERRAIN_PATCH_DIM + lx,
                                  cellz * TERRAIN_PATCH_DIM + lz);
        double xs, ys, zs;
        camspace(c, wx, wy, wz, &xs, &ys, &zs);
        if (have_prev) draw_seg(fb, w, h, c, px, py, pz, xs, ys, zs);
        px = xs; py = ys; pz = zs;
        have_prev = 1;
    }
}

/* Shared perspective body: heightfield strips + ground outline from an
 * already-built camera. Used by the orbit view and terrain_render_cam. */
static void render_perspective_body(uint8_t *fb, int w, int h, const Cam *c)
{
    double wx0 = T.cell_x0 * TERRAIN_PATCH_SIZE_M;
    double wx1 = (T.cell_x1 + 1) * TERRAIN_PATCH_SIZE_M;
    double wz0 = T.cell_z0 * TERRAIN_PATCH_SIZE_M;
    double wz1 = (T.cell_z1 + 1) * TERRAIN_PATCH_SIZE_M;

    /* Grid line stride: keep the total line count bounded on big maps. */
    int kstep = 8;
    while (kstep < TERRAIN_PATCH_DIM &&
           T.used_cells * 2 * (TERRAIN_PATCH_DIM / kstep + 1) > 2600)
        kstep <<= 1;

    for (int cz = 0; cz < TERRAIN_GRID_DIM; cz++) {
        for (int cx = 0; cx < TERRAIN_GRID_DIM; cx++) {
            uint8_t idx = T.zone[cz * TERRAIN_GRID_DIM + cx];
            if (idx == TERRAIN_EMPTY_CELL || idx >= T.nblocks) continue;
            for (int lz = 0; lz < TERRAIN_PATCH_DIM; lz += kstep)
                draw_strip(fb, w, h, c, cx, cz, 1, lz);
            for (int lx = 0; lx < TERRAIN_PATCH_DIM; lx += kstep)
                draw_strip(fb, w, h, c, cx, cz, 0, lx);
        }
    }

    /* Ground-plane outline of the used extent (reference), dim. */
    const double gpx[4] = { wx0, wx1, wx1, wx0 };
    const double gpz[4] = { wz0, wz0, wz1, wz1 };
    for (int i = 0; i < 4; i++) {
        int j = (i + 1) & 3;
        double ax, ay, az, bx, by, bz;
        camspace(c, gpx[i], 0, gpz[i], &ax, &ay, &az);
        camspace(c, gpx[j], 0, gpz[j], &bx, &by, &bz);
        draw_seg(fb, w, h, c, ax, ay, az, bx, by, bz);
    }
}

static void render_perspective(uint8_t *fb, int w, int h,
                               double yaw, double dist)
{
    if (!T.blocks || T.used_cells == 0) return;

    double wx0 = T.cell_x0 * TERRAIN_PATCH_SIZE_M;
    double wx1 = (T.cell_x1 + 1) * TERRAIN_PATCH_SIZE_M;
    double wz0 = T.cell_z0 * TERRAIN_PATCH_SIZE_M;
    double wz1 = (T.cell_z1 + 1) * TERRAIN_PATCH_SIZE_M;
    double cx = (wx0 + wx1) / 2, cz = (wz0 + wz1) / 2;
    double extent = fmax(wx1 - wx0, wz1 - wz0);
    if (dist <= 0) dist = extent;

    Cam c;
    c.px = cx + dist * sin(yaw);
    c.pz = cz + dist * cos(yaw);
    c.py = T.hmax + dist * 0.45;

    /* Look-at: terrain center slightly above the height midrange. */
    double fx = cx - c.px, fy = T.hmax * 0.35 - c.py, fz = cz - c.pz;
    double fl = sqrt(fx * fx + fy * fy + fz * fz);
    fx /= fl; fy /= fl; fz /= fl;
    c.fx = fx; c.fy = fy; c.fz = fz;
    /* right = norm(cross(world_up, fwd)); up = cross(fwd, right) */
    double rx = fz, rz = -fx;
    double rl = sqrt(rx * rx + rz * rz);
    if (rl < 1e-9) { rx = 1; rz = 0; rl = 1; }
    c.rx = rx / rl; c.ry = 0; c.rz = rz / rl;
    c.ux = c.fy * c.rz - c.fz * c.ry;
    c.uy = c.fz * c.rx - c.fx * c.rz;
    c.uz = c.fx * c.ry - c.fy * c.rx;
    c.f  = (w * 0.5) / RASTER_FOV_TAN_HALF;   /* 90 deg horizontal FOV: tan(45 deg) = 1 */
    c.cx = w * 0.5;
    c.cy = h * 0.5;
    c.near = 5.0;
    c.far  = dist * 2.2 + extent;

    render_perspective_body(fb, w, h, &c);
}

/* -----------------------------------------------------------------------
 * Public render entry
 * ----------------------------------------------------------------------- */
void terrain_render(uint8_t *fb, int w, int h, double yaw, double dist)
{
    if (!fb || w <= 0 || h <= 0) return;
    memset(fb, IDX_BG, (size_t)w * (size_t)h);
    if (T.view == TERRAIN_VIEW_PERSPECTIVE)
        render_perspective(fb, w, h, yaw, dist);
    else
        render_topdown(fb, w, h);
}

/* Sun and ambient shared in spirit with scene.c's SUN_DIR: both are
 * PLACEHOLDERS pending the reversed light vector (spec §1). Kept numerically
 * identical so terrain and objects in the same frame are lit consistently --
 * two different suns would read as a rendering bug.
 *
 * H-UAT-013 recalibration (matches scene.c's, same evidence): ambient 0.35
 * rendered every cliff face whose normal points away from the invented sun
 * at light 0.35, which the authored .lum ramps remap near black — P01's
 * canyon walls read as a solid-black wedge filling a third of the cockpit
 * view. docs/evidence/m4-oracle-gameplay-p01.png shows mesa faces warm and
 * textured on every side, so the ambient floor rises until shadow faces
 * keep their texture. Still a placeholder, now capture-calibrated. */
static const double TERRAIN_SUN[3] = { -0.45267873, 0.81482171, -0.36214298 };
#define TERRAIN_AMBIENT 0.60

/* -----------------------------------------------------------------------
 * M8 filled path (docs/specs/m8/software-raster.md)
 * ----------------------------------------------------------------------- */

/*
 * Heightfield LOD, V2: three concentric square rings with stitched seams.
 *
 * V1 drew one uniform 10 m grid out to 700 m, because concentric rings crack
 * where the step changes, and a crack in the ground is a hole you can see the
 * sky through. V2 gets the horizon past that cutoff without multiplying the
 * per-frame cost, and keeps the seams watertight:
 *
 *   band   step     half-width   candidate quads (vs V1's ~19.9k)
 *   near   10 m      640 m        128^2         = 16384
 *   mid    40 m     1600 m         80^2 - 32^2 =  5376
 *   far    80 m     3200 m         80^2 - 40^2 =  4800
 *
 * P01's H-UAT-005 candidate extends the mid ring to 1920 m
 * (96^2 - 32^2 = 8192) so its measured mesa band stays on the 40 m LOD.
 * This presentation tuning remains scenario-local until representative
 * authored missions establish that the same range/contrast is appropriate.
 *
 * All three bands share ONE center: the eye snapped to the far grid (80 m).
 * Band boundaries lie on the next-coarser grid; T-junctions on finer edges
 * are snapped onto the coarse chord (band_vertex_height).
 *
 * The original's terrain LOD algorithm is UNKNOWN; this scheme is ours. The
 * 90 deg horizontal FOV, the shared depth buffer, and the quad winding/diag
 * split are unchanged from V1.
 */
#define TR_STEP_NEAR   2        /* samples per quad edge: 10 m              */
#define TR_STEP_MID    8        /* 40 m                                     */
#define TR_STEP_FAR    16       /* 80 m                                     */
#define TR_QUADS_NEAR  64       /* half-width 64 quads  = 640 m             */
#define TR_QUADS_MID   40       /* half-width 40 quads  = 1600 m             */
#define TR_QUADS_P01   48       /* P01 H-UAT-005 candidate = 1920 m          */
#define TR_QUADS_FAR   40       /* half-width 40 quads  = 3200 m             */
#define TR_INNER_MID   (TR_QUADS_NEAR * TR_STEP_NEAR / TR_STEP_MID)  /* 16 */
#define TR_INNER_FAR(q) ((q) * TR_STEP_MID / TR_STEP_FAR)
#define TR_FAR_EDGE_M  (TR_QUADS_FAR * TR_STEP_FAR * TERRAIN_SAMPLE_STEP_M)

/* Is there a loaded block under this sample? Empty cells read as height 0
 * through sample_global, which is indistinguishable from real ground at sea
 * level -- drawing them would carpet the world in a flat plane. */
static int sample_present(int gx, int gz)
{
    if (gx < 0 || gz < 0 || gx >= GRID_SAMPLES || gz >= GRID_SAMPLES) return 0;
    int cx = gx / TERRAIN_PATCH_DIM, cz = gz / TERRAIN_PATCH_DIM;
    uint8_t idx = T.zone[cz * TERRAIN_GRID_DIM + cx];
    return !(idx == TERRAIN_EMPTY_CELL || idx >= T.nblocks);
}

/* Corner height for a band quad. On the band's OUTER boundary, vertices
 * that do not coincide with the next-coarser band's grid (off % costep !=
 * 0) are snapped onto the coarse edge's chord so the seam is watertight
 * (see the band comment above). costep == 0 selects the far band, whose
 * outer edge borders sky-haze and needs no stitch. Chord endpoints are
 * real sampled heights at coarse grid points, so a boundary facing an
 * empty cell sags toward 0 -- a coastline cliff, accepted and bounded to
 * the band edge. */
static double band_vertex_height(int vx, int vz,
                                 int xlo, int xhi, int zlo, int zhi,
                                 int costep)
{
    if (costep > 0) {
        if (vx == xlo || vx == xhi) {
            int r = (vz - zlo) % costep;
            if (r) {
                double h0 = height_global(vx, vz - r);
                double h1 = height_global(vx, vz - r + costep);
                return h0 + (h1 - h0) * ((double)r / (double)costep);
            }
        } else if (vz == zlo || vz == zhi) {
            int r = (vx - xlo) % costep;
            if (r) {
                double h0 = height_global(vx - r, vz);
                double h1 = height_global(vx - r + costep, vz);
                return h0 + (h1 - h0) * ((double)r / (double)costep);
            }
        }
    }
    return height_global(vx, vz);
}

/* Camera basis shared by the terrain bands and the road pass. */
typedef struct {
    double fx, fy, fz;
    double rx, ry, rz;
    double ux, uy, uz;
} TBasis;

static void basis_from_camera(const CameraView *camera, TBasis *b)
{
    b->fx = camera->forward[0];
    b->fy = camera->forward[1];
    b->fz = camera->forward[2];
    b->rx = camera->right[0];
    b->ry = camera->right[1];
    b->rz = camera->right[2];
    b->ux = camera->up[0];
    b->uy = camera->up[1];
    b->uz = camera->up[2];
}

/* Surface-texture repeat: one tile spans (w/10) x (h/10) metres, Open76's
 * LevelLoader tiling (terrain.md §4/T7 -- INFERRED for the original; the
 * starting guess pending screenshot comparison). For the measured 128x128
 * P01 tile that is 12.8 m. */
#define TR_TEX_M_PER_10 10.0

static void fill_band(RTarget *t, const double eye[3], const TBasis *b,
                      int csx, int csz, int step, int quads, int inner,
                      int costep, uint8_t base, int skyline_tune,
                      RPainterPixel *painter_order)
{
    const RTex *tex = T.surf_ok ? &T.surf : NULL;
    const float su = tex ? (float)(TR_TEX_M_PER_10 / (double)tex->w) : 0.0f;
    const float sv = tex ? (float)(TR_TEX_M_PER_10 / (double)tex->h) : 0.0f;
    const int half = quads * step;      /* samples */
    const int xlo = csx - half, xhi = csx + half;
    const int zlo = csz - half, zhi = csz + half;

    for (int qj = -quads; qj < quads; qj++) {
        int qj_in = inner && qj >= -inner && qj < inner;
        for (int qi = -quads; qi < quads; qi++) {
            if (qj_in && qi >= -inner && qi < inner) continue;
            int gx = csx + qi * step;
            int gz = csz + qj * step;
            if (!sample_present(gx, gz) || !sample_present(gx + step, gz) ||
                !sample_present(gx, gz + step) || !sample_present(gx + step, gz + step))
                continue;

            /* Wound so the normal points UP: (x,z) (x,z+s) (x+s,z+s) (x+s,z).
             * raster_polygon fans from vertex 0, which pins the quad diagonal
             * to v0-v2 (spec §2) -- it must match the GPU path's split. */
            const int ox[4] = { 0, 0, step, step };
            const int oz[4] = { 0, step, step, 0 };
            double wx[4], wy[4], wz[4];
            for (int k = 0; k < 4; k++) {
                int vx = gx + ox[k], vz = gz + oz[k];
                wx[k] = (double)vx * TERRAIN_SAMPLE_STEP_M;
                wz[k] = (double)vz * TERRAIN_SAMPLE_STEP_M;
                wy[k] = band_vertex_height(vx, vz, xlo, xhi, zlo, zhi, costep);
            }

            /* Normal from the two diagonals: for a non-planar heightfield quad
             * that is the average of the two triangle normals, which shades
             * better than picking one of them. */
            double d0[3] = { wx[2] - wx[0], wy[2] - wy[0], wz[2] - wz[0] };
            double d1[3] = { wx[3] - wx[1], wy[3] - wy[1], wz[3] - wz[1] };
            double nx = d0[1] * d1[2] - d0[2] * d1[1];
            double ny = d0[2] * d1[0] - d0[0] * d1[2];
            double nz = d0[0] * d1[1] - d0[1] * d1[0];
            double nl = sqrt(nx * nx + ny * ny + nz * nz);
            if (nl < 1e-12) continue;
            nx /= nl; ny /= nl; nz /= nl;

            double lit = nx * TERRAIN_SUN[0] + ny * TERRAIN_SUN[1] + nz * TERRAIN_SUN[2];
            if (lit < 0.0) lit = 0.0;
            double light = TERRAIN_AMBIENT + (1.0 - TERRAIN_AMBIENT) * lit;

            if (skyline_tune) {
                /*
                 * H-UAT-005 P01 skyline candidate (authored TER, no synthetic
                 * backdrop). Distant relief is present in depth but washed
                 * against horizon haze. Two port-tuned DECISION shades:
                 * range-darken flats beyond 250 m and emphasize steep
                 * faces/height jumps. These are not reversed light constants.
                 */
                double z_sum = 0.0;
                for (int k = 0; k < 4; k++) {
                    double dx = wx[k] - eye[0], dy = wy[k] - eye[1],
                           dz = wz[k] - eye[2];
                    z_sum += dx * b->fx + dy * b->fy + dz * b->fz;
                }
                double z_avg = 0.25 * z_sum;
                double dist_k = 0.0;
                if (z_avg > 250.0) {
                    dist_k = (z_avg - 250.0) / 1800.0;
                    if (dist_k > 1.0) dist_k = 1.0;
                }
                double cliff = 1.0 - ny;
                if (cliff < 0.0) cliff = 0.0;
                double edge = step * TERRAIN_SAMPLE_STEP_M;
                double dh = fabs(wy[0] - wy[1]) + fabs(wy[1] - wy[2]) +
                            fabs(wy[2] - wy[3]) + fabs(wy[3] - wy[0]);
                double jump = edge > 1e-6 ? 0.25 * dh / edge : 0.0;
                if (jump > 1.0) jump = 1.0;
                if (jump > cliff) cliff = jump;
                if (cliff > 1.0) cliff = 1.0;
                light *= 1.0 - 0.40 * dist_k;
                /* H-UAT-013 rebalance against the raised TERRAIN_AMBIENT:
                 * the cliff coefficient rises 0.55 -> 0.65 so the DISTANT
                 * skyline band keeps sky_probe's terrain-to-sky contrast
                 * floor (measured: mean gap 16.9 luma), and the floor rises
                 * 0.10 -> 0.27 so mid-range canyon walls stop saturating
                 * into the authored .lum's blackest rows — they were the
                 * solid-black wedge filling a third of the P01 cockpit
                 * view. Texture stays readable on the rock, matching
                 * m4-oracle-gameplay-p01.png's warm mesa faces. */
                light *= 1.0 - 0.65 * cliff * (0.40 + 0.60 * dist_k);
                if (light < 0.27) light = 0.27;
            }

            RVert poly[4];
            int behind = 0;
            for (int k = 0; k < 4; k++) {
                double dx = wx[k] - eye[0], dy = wy[k] - eye[1], dz = wz[k] - eye[2];
                poly[k].x = dx * b->rx + dy * b->ry + dz * b->rz;
                poly[k].y = dx * b->ux + dy * b->uy + dz * b->uz;
                poly[k].z = dx * b->fx + dy * b->fy + dz * b->fz;
                poly[k].u = (float)(wx[k] * su);
                poly[k].v = (float)(wz[k] * sv);
                poly[k].light = (float)light;
                poly[k].fog = 0.0f;
                if (poly[k].z < t->znear) behind++;
            }
            if (behind == 4) continue;      /* wholly behind the eye */
            double sort_z = poly[0].z;
            for (int k = 1; k < 4; k++)
                if (poly[k].z < sort_z) sort_z = poly[k].z;
            if (painter_order)
                raster_painter_terrain(t, painter_order, sort_z);
            raster_pixel_history_draw(t, gz * GRID_SAMPLES + gx,
                                      T.ter_name[0] ? T.ter_name : T.mission,
                                      "terrain", "world");
            if (tex) raster_pixel_history_texture(t, T.surf_name);
            raster_polygon_tex(t, poly, 4, base, tex, 0);
        }
    }
}

/* Road ribbons, drawn after the heightfield so they resolve against it in
 * the shared depth buffer. Each quad spans pieces k..k+1; U spans 0->1
 * across the road, V increments once per piece (Open76's construction,
 * scene.md §5 -- CONFIRMED as Open76 behaviour). Ribbons are conformed to
 * the terrain at load, so no per-frame height queries happen here. */
static void fill_roads(RTarget *t, const double eye[3], const TBasis *b,
                       double far_m, RPainterPixel *painter_order)
{
    /* Fallback flat tones, used only when the road tile failed to load.
     * INVENTED fallbacks, nearest-matched into the level palette -- they
     * exist so a missing asset still reads as a road, not as ground. */
    static const uint8_t FALLBACK_RGB[3][3] = {
        { 96, 96, 100 },    /* paved asphalt-grey   */
        { 122, 96, 70 },    /* dirt brown           */
        { 150, 138, 112 },  /* riverbed pale sand   */
    };
    const double far2 = far_m * far_m;

    for (int s = 0; s < T.nroads; s++) {
        const struct RoadSeg *r = &T.roads[s];
        if (r->count < 2) continue;
        const RTex *tex = T.road_tex_ok[r->type] ? &T.road_tex[r->type] : NULL;
        uint8_t base = raster_rgb_to_index(FALLBACK_RGB[r->type][0],
                                           FALLBACK_RGB[r->type][1],
                                           FALLBACK_RGB[r->type][2]);
        for (uint32_t k = 0; k + 1 < r->count; k++) {
            const float *p0 = r->lr + (size_t)k * 6;
            const float *p1 = p0 + 6;

            /* Cheap radial cull on the quad midpoint; also keeps ribbons
             * from floating past the terrain's far edge. */
            double mx = 0.25 * ((double)p0[0] + p0[3] + p1[0] + p1[3]) - eye[0];
            double mz = 0.25 * ((double)p0[2] + p0[5] + p1[2] + p1[5]) - eye[2];
            if (mx * mx + mz * mz > far2) continue;

            /* Quad: left_k, right_k, right_k+1, left_k+1 (wound UP). */
            double wx[4] = { p0[0], p0[3], p1[3], p1[0] };
            double wy[4] = { p0[1], p0[4], p1[4], p1[1] };
            double wz[4] = { p0[2], p0[5], p1[5], p1[2] };
            const float uu[4] = { 0.0f, 1.0f, 1.0f, 0.0f };
            const float vv[4] = { (float)k, (float)k, (float)(k + 1), (float)(k + 1) };

            /* Normal from the diagonals, forced UP (the ribbon is
             * double-sided in practice: raster does not cull, but lighting
             * must not flip on a reversed span). */
            double d0[3] = { wx[2] - wx[0], wy[2] - wy[0], wz[2] - wz[0] };
            double d1[3] = { wx[3] - wx[1], wy[3] - wy[1], wz[3] - wz[1] };
            double nx = d0[1] * d1[2] - d0[2] * d1[1];
            double ny = d0[2] * d1[0] - d0[0] * d1[2];
            double nz = d0[0] * d1[1] - d0[1] * d1[0];
            double nl = sqrt(nx * nx + ny * ny + nz * nz);
            double light = 1.0;
            if (nl > 1e-12) {
                if (ny < 0.0) { nx = -nx; ny = -ny; nz = -nz; }
                double lit = (nx * TERRAIN_SUN[0] + ny * TERRAIN_SUN[1] +
                              nz * TERRAIN_SUN[2]) / nl;
                if (lit < 0.0) lit = 0.0;
                light = TERRAIN_AMBIENT + (1.0 - TERRAIN_AMBIENT) * lit;
            }

            RVert poly[4];
            int behind = 0;
            for (int c = 0; c < 4; c++) {
                double dx = wx[c] - eye[0], dy = wy[c] - eye[1], dz = wz[c] - eye[2];
                poly[c].x = dx * b->rx + dy * b->ry + dz * b->rz;
                poly[c].y = dx * b->ux + dy * b->uy + dz * b->uz;
                poly[c].z = dx * b->fx + dy * b->fy + dz * b->fz;
                poly[c].u = uu[c];
                poly[c].v = vv[c];
                poly[c].light = (float)light;
                poly[c].fog = 0.0f;
                if (poly[c].z < t->znear) behind++;
            }
            if (behind == 4) continue;
            double sort_z = poly[0].z;
            for (int c = 1; c < 4; c++)
                if (poly[c].z < sort_z) sort_z = poly[c].z;
            if (painter_order)
                raster_painter_overlay(t, painter_order, sort_z);
            int primitive = (int)(((uint32_t)s * 100000u + k) & 0x7fffffffu);
            raster_pixel_history_draw(t, primitive,
                                      ROAD_TEX_NAME[r->type],
                                      "road", "world");
            if (tex) raster_pixel_history_texture(t, ROAD_TEX_NAME[r->type]);
            raster_polygon_tex(t, poly, 4, base, tex, 0);
        }
    }
}

void terrain_render_filled_order(RTarget *t, const CameraView *camera,
                                 RPainterPixel *painter_order)
{
    if (!t || !t->color || !camera || !camera_view_valid(camera) ||
        !T.blocks || T.used_cells == 0)
        return;

    TBasis b;
    basis_from_camera(camera, &b);

    /*
     * Ground colour fallback. Used when the mission surface tile is absent
     * or rejected (non-POT etc.), and for any keyed texel on a non-cut-out
     * face. The low 12 bits of a sample are height; what the top 4 encode
     * is not reversed, so this still does NOT invent a surface-type palette.
     */
    uint8_t base = raster_rgb_to_index(150, 124, 86);

    /* One center for all bands, snapped to the far grid so the rings nest
     * exactly. Bands past the target's draw distance are skipped. */
    const double *eye = camera->eye;
    int csx = (int)floor(eye[0] / (TR_STEP_FAR * TERRAIN_SAMPLE_STEP_M)) * TR_STEP_FAR;
    int csz = (int)floor(eye[2] / (TR_STEP_FAR * TERRAIN_SAMPLE_STEP_M)) * TR_STEP_FAR;
    const char *base_name = strrchr(T.mission, '/');
    base_name = base_name ? base_name + 1 : T.mission;
    int skyline_tune = strcasecmp(base_name, "P01.MSN") == 0;
    int mid_quads = skyline_tune ? TR_QUADS_P01 : TR_QUADS_MID;

    fill_band(t, eye, &b, csx, csz, TR_STEP_NEAR, TR_QUADS_NEAR, 0,
              TR_STEP_MID, base, 0, painter_order);
    if (t->zfar > TR_QUADS_NEAR * TR_STEP_NEAR * TERRAIN_SAMPLE_STEP_M)
        fill_band(t, eye, &b, csx, csz, TR_STEP_MID, mid_quads,
                  TR_INNER_MID, TR_STEP_FAR, base, skyline_tune,
                  painter_order);
    if (t->zfar > mid_quads * TR_STEP_MID * TERRAIN_SAMPLE_STEP_M)
        fill_band(t, eye, &b, csx, csz, TR_STEP_FAR, TR_QUADS_FAR,
                  TR_INNER_FAR(mid_quads), 0, base, skyline_tune,
                  painter_order);

    road_diag_prepare(t);
    double road_far = t->zfar < TR_FAR_EDGE_M ? t->zfar : TR_FAR_EDGE_M;
    long wrote0 = t->pixels_written;
    long rescued0 = t->painter_pixels_rescued;
    long occluded0 = t->painter_pixels_occluded;
    fill_roads(t, eye, &b, road_far, painter_order);
    T.road_px_written = t->pixels_written - wrote0;
    T.road_px_rescued = t->painter_pixels_rescued - rescued0;
    T.road_px_occluded = t->painter_pixels_occluded - occluded0;
    raster_painter_disable(t);
}

void terrain_render_filled(RTarget *t, const CameraView *camera)
{
    /* Direct/probe callers still need the same road policy. Keep bounded
     * scratch here; the canonical world compositor supplies its own reusable
     * buffer through terrain_render_filled_order(). */
    static RPainterPixel *order;
    static size_t cap;
    if (!t || t->w <= 0 || t->h <= 0) {
        terrain_render_filled_order(t, camera, NULL);
        return;
    }
    size_t need = (size_t)t->w * (size_t)t->h; /* painter/view-Z owners */
    if (cap < need) {
        RPainterPixel *p = realloc(order, need * sizeof *p);
        if (!p) {
            terrain_render_filled_order(t, camera, NULL);
            return;
        }
        order = p;
        cap = need;
    }
    memset(order, 0, need * sizeof *order);
    terrain_render_filled_order(t, camera, order);
}

/* ---- LOD mesh export for WebGPU (same bands as fill_band) ------------- */

typedef struct {
    float *v;
    int    n;       /* floats written */
    int    cap;     /* floats capacity */
    int    nquads;  /* accepted quads this band */
    int    fail;    /* sticky: a reserve failed; buffer is incomplete */
} LodMeshBuf;

static int lod_reserve(LodMeshBuf *b, int need_floats)
{
    if (b->fail) return -1;
    if (b->n + need_floats <= b->cap) return 0;
    int nc = b->cap ? b->cap * 2 : 1 << 16;
    while (nc < b->n + need_floats) nc *= 2;
    float *nv = realloc(b->v, (size_t)nc * sizeof(float));
    if (!nv) { b->fail = 1; return -1; }
    b->v = nv;
    b->cap = nc;
    return 0;
}

static void lod_push_vert(LodMeshBuf *b, double wx, double wy, double wz,
                          double nx, double ny, double nz, float u, float v)
{
    if (lod_reserve(b, 8) != 0) return;
    float *p = b->v + b->n;
    p[0] = (float)wx; p[1] = (float)wy; p[2] = (float)wz;
    p[3] = (float)nx; p[4] = (float)ny; p[5] = (float)nz;
    p[6] = u; p[7] = v;
    b->n += 8;
}

/* World-space twin of fill_band: two tris per accepted quad, same diagonal. */
static void fill_band_mesh(LodMeshBuf *mb, int csx, int csz,
                           int step, int quads, int inner, int costep,
                           float su, float sv)
{
    const int half = quads * step;
    const int xlo = csx - half, xhi = csx + half;
    const int zlo = csz - half, zhi = csz + half;
    mb->nquads = 0;

    for (int qj = -quads; qj < quads; qj++) {
        int qj_in = inner && qj >= -inner && qj < inner;
        for (int qi = -quads; qi < quads; qi++) {
            if (qj_in && qi >= -inner && qi < inner) continue;
            int gx = csx + qi * step;
            int gz = csz + qj * step;
            if (!sample_present(gx, gz) || !sample_present(gx + step, gz) ||
                !sample_present(gx, gz + step) ||
                !sample_present(gx + step, gz + step))
                continue;

            const int ox[4] = { 0, 0, step, step };
            const int oz[4] = { 0, step, step, 0 };
            double wx[4], wy[4], wz[4];
            float uu[4], vv[4];
            for (int k = 0; k < 4; k++) {
                int vx = gx + ox[k], vz = gz + oz[k];
                wx[k] = (double)vx * TERRAIN_SAMPLE_STEP_M;
                wz[k] = (double)vz * TERRAIN_SAMPLE_STEP_M;
                wy[k] = band_vertex_height(vx, vz, xlo, xhi, zlo, zhi, costep);
                uu[k] = (float)(wx[k] * (double)su);
                vv[k] = (float)(wz[k] * (double)sv);
            }

            double d0[3] = { wx[2] - wx[0], wy[2] - wy[0], wz[2] - wz[0] };
            double d1[3] = { wx[3] - wx[1], wy[3] - wy[1], wz[3] - wz[1] };
            double nx = d0[1] * d1[2] - d0[2] * d1[1];
            double ny = d0[2] * d1[0] - d0[0] * d1[2];
            double nz = d0[0] * d1[1] - d0[1] * d1[0];
            double nl = sqrt(nx * nx + ny * ny + nz * nz);
            if (nl < 1e-12) continue;
            nx /= nl; ny /= nl; nz /= nl;

            /* Fan v0-v1-v2 and v0-v2-v3 — same diagonal as raster_polygon. */
            const int tris[6] = { 0, 1, 2, 0, 2, 3 };
            for (int t = 0; t < 6; t++) {
                int i = tris[t];
                lod_push_vert(mb, wx[i], wy[i], wz[i], nx, ny, nz, uu[i], vv[i]);
            }
            mb->nquads++;
        }
    }
}

int terrain_lod_mesh_export(const CameraView *camera, double zfar_m,
                            float **out_verts, int *out_nverts,
                            int out_bands[3])
{
    if (out_verts) *out_verts = NULL;
    if (out_nverts) *out_nverts = 0;
    if (out_bands) out_bands[0] = out_bands[1] = out_bands[2] = 0;
    if (!out_verts || !out_nverts || !camera || !camera_view_valid(camera))
        return -1;
    if (!T.blocks || T.used_cells == 0)
        return 0;

    double zfar = zfar_m > 0.0 ? zfar_m : 3000.0;
    const double *eye = camera->eye;
    int csx = (int)floor(eye[0] / (TR_STEP_FAR * TERRAIN_SAMPLE_STEP_M)) * TR_STEP_FAR;
    int csz = (int)floor(eye[2] / (TR_STEP_FAR * TERRAIN_SAMPLE_STEP_M)) * TR_STEP_FAR;
    const char *base_name = strrchr(T.mission, '/');
    base_name = base_name ? base_name + 1 : T.mission;
    int skyline_tune = strcasecmp(base_name, "P01.MSN") == 0;
    int mid_quads = skyline_tune ? TR_QUADS_P01 : TR_QUADS_MID;

    float su = 0.0f, sv = 0.0f;
    if (T.surf_ok && T.surf.w > 0 && T.surf.h > 0) {
        su = (float)(TR_TEX_M_PER_10 / (double)T.surf.w);
        sv = (float)(TR_TEX_M_PER_10 / (double)T.surf.h);
    }

    LodMeshBuf mb = { 0 };
    int bands[3] = { 0, 0, 0 };

    fill_band_mesh(&mb, csx, csz, TR_STEP_NEAR, TR_QUADS_NEAR, 0,
                   TR_STEP_MID, su, sv);
    bands[0] = mb.nquads;

    if (zfar > TR_QUADS_NEAR * TR_STEP_NEAR * TERRAIN_SAMPLE_STEP_M) {
        fill_band_mesh(&mb, csx, csz, TR_STEP_MID, mid_quads,
                       TR_INNER_MID, TR_STEP_FAR, su, sv);
        bands[1] = mb.nquads;
    }
    if (zfar > mid_quads * TR_STEP_MID * TERRAIN_SAMPLE_STEP_M) {
        fill_band_mesh(&mb, csx, csz, TR_STEP_FAR, TR_QUADS_FAR,
                       TR_INNER_FAR(mid_quads), 0, su, sv);
        bands[2] = mb.nquads;
    }

    /* OOM anywhere above is sticky: publish nothing rather than a
     * partial mesh. Outputs stay zeroed from entry. */
    if (mb.fail) {
        free(mb.v);
        return -1;
    }

    if (out_bands) {
        out_bands[0] = bands[0];
        out_bands[1] = bands[1];
        out_bands[2] = bands[2];
    }
    *out_verts = mb.v;
    *out_nverts = mb.n / 8;
    return 0;
}

int terrain_surf_tex_size(int *w, int *h)
{
    if (!T.surf_ok || T.surf.w <= 0 || T.surf.h <= 0) {
        if (w) *w = 0;
        if (h) *h = 0;
        return 0;
    }
    if (w) *w = T.surf.w;
    if (h) *h = T.surf.h;
    return 1;
}

void terrain_light_params(float sun_dir[3], float *ambient)
{
    if (sun_dir) {
        sun_dir[0] = (float)TERRAIN_SUN[0];
        sun_dir[1] = (float)TERRAIN_SUN[1];
        sun_dir[2] = (float)TERRAIN_SUN[2];
    }
    if (ambient) *ambient = (float)TERRAIN_AMBIENT;
}

int terrain_road_seg_count(void) { return T.nroads; }

void terrain_road_render_stats(long *written, long *rescued, long *occluded)
{
    if (written) *written = T.road_px_written;
    if (rescued) *rescued = T.road_px_rescued;
    if (occluded) *occluded = T.road_px_occluded;
}

/* Browser traversal gate seam. These expose the same per-frame counters as
 * terrain_road_render_stats() without requiring JS-owned pointer scratch.
 * They are diagnostics only: no value feeds back into rendering or sim. */
EMSCRIPTEN_KEEPALIVE int terrain_road_pixels_written(void)
{
    return (int)T.road_px_written;
}

EMSCRIPTEN_KEEPALIVE int terrain_road_pixels_rescued(void)
{
    return (int)T.road_px_rescued;
}

EMSCRIPTEN_KEEPALIVE int terrain_road_pixels_occluded(void)
{
    return (int)T.road_px_occluded;
}

EMSCRIPTEN_KEEPALIVE void terrain_road_diagnostics_enable(int enabled)
{
    s_road_diag_enabled = enabled != 0;
    if (!s_road_diag_enabled) {
        free(s_road_diag_footprint_z);
        free(s_road_diag_owner);
        s_road_diag_footprint_z = NULL;
        s_road_diag_owner = NULL;
        s_road_diag_footprint_cap = 0;
        s_road_diag_owner_cap = 0;
    }
}

EMSCRIPTEN_KEEPALIVE const float *terrain_road_footprint_z_ptr(void)
{
    return s_road_diag_enabled ? s_road_diag_footprint_z : NULL;
}

EMSCRIPTEN_KEEPALIVE const uint8_t *terrain_road_owner_ptr(void)
{
    return s_road_diag_enabled ? s_road_diag_owner : NULL;
}

int terrain_road_tex(int type, const uint8_t **texels, int *w, int *h)
{
    if (type < 0 || type > 2 || !T.road_tex_ok[type]) {
        if (texels) *texels = NULL;
        if (w) *w = 0;
        if (h) *h = 0;
        return 0;
    }
    if (texels) *texels = T.road_tex[type].texels;
    if (w) *w = T.road_tex[type].w;
    if (h) *h = T.road_tex[type].h;
    return 1;
}

int terrain_roads_mesh_export(const double eye[3], double far_m,
                              float **out_verts, int *out_nverts,
                              int out_by_type[3])
{
    if (out_verts) *out_verts = NULL;
    if (out_nverts) *out_nverts = 0;
    if (out_by_type) out_by_type[0] = out_by_type[1] = out_by_type[2] = 0;
    if (!out_verts || !out_nverts || !eye) return -1;
    if (!T.blocks || T.nroads == 0) return 0;

    double far = far_m > 0.0 ? far_m : TR_FAR_EDGE_M;
    if (far > TR_FAR_EDGE_M) far = TR_FAR_EDGE_M;
    double far2 = far * far;

    LodMeshBuf buckets[3] = { { 0 }, { 0 }, { 0 } };

    for (int s = 0; s < T.nroads; s++) {
        const struct RoadSeg *r = &T.roads[s];
        if (r->count < 2) continue;
        int ty = (int)r->type;
        if (ty < 0 || ty > 2) ty = 0;
        LodMeshBuf *mb = &buckets[ty];

        for (uint32_t k = 0; k + 1 < r->count; k++) {
            const float *p0 = r->lr + (size_t)k * 6;
            const float *p1 = p0 + 6;
            double mx = 0.25 * ((double)p0[0] + p0[3] + p1[0] + p1[3]) - eye[0];
            double mz = 0.25 * ((double)p0[2] + p0[5] + p1[2] + p1[5]) - eye[2];
            if (mx * mx + mz * mz > far2) continue;

            double wx[4] = { p0[0], p0[3], p1[3], p1[0] };
            double wy[4] = { p0[1], p0[4], p1[4], p1[1] };
            double wz[4] = { p0[2], p0[5], p1[5], p1[2] };
            const float uu[4] = { 0.0f, 1.0f, 1.0f, 0.0f };
            const float vv[4] = { (float)k, (float)k, (float)(k + 1), (float)(k + 1) };

            double d0[3] = { wx[2] - wx[0], wy[2] - wy[0], wz[2] - wz[0] };
            double d1[3] = { wx[3] - wx[1], wy[3] - wy[1], wz[3] - wz[1] };
            double nx = d0[1] * d1[2] - d0[2] * d1[1];
            double ny = d0[2] * d1[0] - d0[0] * d1[2];
            double nz = d0[0] * d1[1] - d0[1] * d1[0];
            double nl = sqrt(nx * nx + ny * ny + nz * nz);
            if (nl < 1e-12) { nx = 0; ny = 1; nz = 0; }
            else {
                if (ny < 0.0) { nx = -nx; ny = -ny; nz = -nz; }
                nx /= nl; ny /= nl; nz /= nl;
            }

            /*
             * Same diagonal as fill_roads / raster_polygon (v0-v2), but the
             * corner order for road quads is L0,R0,R1,L1 — opposite the
             * heightfield's SW,NW,NE,SE. Emitting 0-1-2 / 0-2-3 therefore
             * winds the ribbon the other way from fill_band_mesh. Software
             * does not cull, so fill_roads never noticed; the GPU terrain
             * pipeline culls back faces with the same frontFace as the LOD
             * heightfield, so the ribbon was fully discarded. Reverse the
             * fan corners so a +Y normal (after the force-up above) matches
             * terrain winding.
             */
            const int tris[6] = { 0, 2, 1, 0, 3, 2 };
            for (int t = 0; t < 6; t++) {
                int i = tris[t];
                lod_push_vert(mb, wx[i], wy[i], wz[i], nx, ny, nz, uu[i], vv[i]);
            }
        }
    }

    /* Sticky OOM in any bucket: free temporaries, keep every output
     * zeroed (entry already cleared them) — never publish partial
     * geometry. */
    for (int i = 0; i < 3; i++) {
        if (buckets[i].fail) {
            for (int j = 0; j < 3; j++) free(buckets[j].v);
            return -1;
        }
    }

    int total_floats = buckets[0].n + buckets[1].n + buckets[2].n;
    if (total_floats == 0) return 0;
    float *all = malloc((size_t)total_floats * sizeof(float));
    if (!all) {
        for (int i = 0; i < 3; i++) free(buckets[i].v);
        return -1;
    }
    int off = 0;
    for (int i = 0; i < 3; i++) {
        if (out_by_type) out_by_type[i] = buckets[i].n / 8;
        if (buckets[i].n) {
            memcpy(all + off, buckets[i].v, (size_t)buckets[i].n * sizeof(float));
            off += buckets[i].n;
        }
        free(buckets[i].v);
    }
    *out_verts = all;
    *out_nverts = total_floats / 8;
    return 0;
}

void terrain_render_camera(uint8_t *fb, int w, int h,
                           const CameraView *camera, double far)
{
    if (!fb || w <= 0 || h <= 0 || !camera || !camera_view_valid(camera))
        return;
    if (!T.blocks || T.used_cells == 0)
        return;

    Cam c;
    c.px = camera->eye[0];
    c.py = camera->eye[1];
    c.pz = camera->eye[2];
    c.fx = camera->forward[0];
    c.fy = camera->forward[1];
    c.fz = camera->forward[2];
    c.rx = camera->right[0];
    c.ry = camera->right[1];
    c.rz = camera->right[2];
    c.ux = camera->up[0];
    c.uy = camera->up[1];
    c.uz = camera->up[2];
    c.f  = (w * 0.5) / camera_view_fov_tan_half(camera);
    c.cx = w * 0.5;
    c.cy = h * 0.5;
    c.near = 5.0;
    c.far  = far > 0 ? far : 3000.0;

    render_perspective_body(fb, w, h, &c);
}
