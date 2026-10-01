/*
 * raster_test.c — kernel goldens for the M8 software rasterizer.
 *
 * Spec §8.4 asks for: shared edges, every clip plane, degenerate triangles,
 * winding, overlapping and equal depth, masked pixels, perspective.
 * Phase-D adds: native .lum shade-table population (identity row, dark
 * saturation, fallback), and the destination-indexed translucency LUT
 * (table[(texel<<8)|dest], orientation, opaque untouched, cutout preserved).
 *
 * These deliberately need NO GAME ASSETS. A kernel golden that depends on
 * nitro.zfs cannot run in CI on a machine without the game, and the whole
 * point of a golden is that it is always runnable. The palette below is
 * synthetic and chosen so shading is predictable: entry i is grey (i,i,i), so
 * at full light shade[i][MAX] == i and a base index reads straight back out of
 * the framebuffer. The .lum/.tbl fixtures are synthetic too, built to the
 * MEASURED layout of the shipped files (identity row 7, saturation row 31).
 *
 * Build (from engine/):
 *   OUT=/external/path tools/build_probe.sh raster_test
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "engine/raster.h"

#define W 64
#define H 64

static uint8_t  g_color[W * H];
static uint32_t g_depth[W * H];

static int g_fail;
static int g_checks;

static void check(int cond, const char *what)
{
    g_checks++;
    if (!cond) { printf("  FAIL: %s\n", what); g_fail++; }
}

static void synth_palette(void)
{
    uint8_t pal[768];
    for (int i = 0; i < 256; i++) {
        pal[i * 3 + 0] = (uint8_t)i;
        pal[i * 3 + 1] = (uint8_t)i;
        pal[i * 3 + 2] = (uint8_t)i;
    }
    raster_set_palette(pal);
}

static RTarget fresh(void)
{
    RTarget t;
    raster_begin(&t, g_color, g_depth, W, H, (W * 0.5) / RASTER_FOV_TAN_HALF,
                 1.0, 1000.0);
    raster_clear(&t, 0);
    return t;
}

static RVert vert(double x, double y, double z)
{
    RVert v;
    v.x = x; v.y = y; v.z = z;
    v.u = 0.0f; v.v = 0.0f; v.light = 1.0f; v.fog = 0.0f;
    return v;
}

/* Coverage is "the depth buffer was written", which is independent of what
 * colour the fragment happened to be. */
static int covered(const uint32_t *d, int i) { return d[i] != 0; }

static int count_covered(const uint32_t *d)
{
    int n = 0;
    for (int i = 0; i < W * H; i++) if (covered(d, i)) n++;
    return n;
}

/*
 * Projection contract (binary-derived: nitro.exe FUN_00469dd0 clamps the
 * full aperture to 90 degrees HORIZONTAL and FUN_00469fb0 derives focal
 * from the viewport WIDTH). The half-FOV tangent must be exactly
 * tan(45 deg) == 1 -- anything else, in particular the previously authored
 * 60-degree VERTICAL formula tan(30 deg) ~= 0.5773503, is a regression.
 */
static void test_projection_contract(void)
{
    check(RASTER_FOV_TAN_HALF == 1.0,
          "half-FOV tangent is exactly tan(45 deg) = 1 (90 deg horizontal)");
    check(RASTER_FOV_TAN_HALF != 0.57735026918962576,
          "not the old 60 deg vertical tan(30 deg)");
}

/* ---------------------------------------------------------------------- */

/*
 * Watertightness. Two triangles that share an edge must cover every pixel of
 * the quad exactly once: no seam (a pixel neither claims) and no double-hit
 * (a pixel both claim). This is the property the top-left rule exists for,
 * and the reason spec §3 puts watertightness ahead of determinism — meshes
 * here are fan-triangulated n-gons, so shared edges are everywhere.
 */
static void test_shared_edge(void)
{
    printf("shared edge (watertight)\n");
    RVert q[4] = {
        vert(-6.0,  5.0, 12.0),
        vert( 7.0,  6.0, 12.0),
        vert( 8.0, -4.0, 12.0),
        vert(-5.0, -6.0, 12.0),
    };

    static uint32_t da[W * H], db[W * H], dq[W * H];

    RTarget t = fresh();
    raster_triangle(&t, &q[0], &q[1], &q[2], 100);
    memcpy(da, g_depth, sizeof da);

    t = fresh();
    raster_triangle(&t, &q[0], &q[2], &q[3], 100);
    memcpy(db, g_depth, sizeof db);

    t = fresh();
    raster_polygon(&t, q, 4, 100);
    memcpy(dq, g_depth, sizeof dq);

    int overlap = 0, seam = 0, extra = 0;
    for (int i = 0; i < W * H; i++) {
        int a = covered(da, i), b = covered(db, i), whole = covered(dq, i);
        if (a && b) overlap++;
        if (whole && !(a || b)) seam++;
        if (!whole && (a || b)) extra++;
    }
    printf("  quad=%d triA=%d triB=%d overlap=%d seam=%d extra=%d\n",
           count_covered(dq), count_covered(da), count_covered(db),
           overlap, seam, extra);
    check(count_covered(dq) > 200, "quad covers a meaningful area");
    check(overlap == 0, "no pixel is covered by both halves");
    check(seam == 0, "no pixel of the quad is missed by both halves");
    check(extra == 0, "halves cover nothing outside the quad");
}

/*
 * Winding. The kernel accepts either winding because double-sided faces exist
 * and culling is the caller's job. Reversing the winding must not move a
 * single pixel.
 */
static void test_winding(void)
{
    printf("winding independence\n");
    /* Fully inside the frustum: at z=10 with the contract focal (w/2) the
     * screen bound is |x|,|y| < 10, so nothing here clips and the triangle
     * must reach the kernel as exactly one triangle. */
    RVert a = vert(-5.0, 4.0, 10.0), b = vert(5.0, 4.0, 10.0), c = vert(0.0, -5.0, 10.0);
    static uint32_t d1[W * H];
    static uint8_t  c1[W * H];

    RTarget t = fresh();
    raster_triangle(&t, &a, &b, &c, 120);
    memcpy(d1, g_depth, sizeof d1);
    memcpy(c1, g_color, sizeof c1);
    long drawn = t.tris_drawn;

    t = fresh();
    raster_triangle(&t, &c, &b, &a, 120);

    check(drawn == 1 && t.tris_drawn == 1, "unclipped triangle is exactly one triangle");
    check(memcmp(d1, g_depth, sizeof d1) == 0, "depth identical under reversal");
    check(memcmp(c1, g_color, sizeof c1) == 0, "colour identical under reversal");

    /* Reversal must also survive the clipper. Sutherland-Hodgman emits in
     * polygon order, so a reversed input is fanned from a different starting
     * vertex — if that changed coverage, shared edges between adjacent faces
     * would seam wherever one of them happened to be clipped. */
    RVert wa = vert(-40.0, 30.0, 10.0), wb = vert(40.0, 30.0, 10.0), wc = vert(0.0, -40.0, 10.0);
    t = fresh();
    raster_triangle(&t, &wa, &wb, &wc, 120);
    memcpy(d1, g_depth, sizeof d1);
    check(t.tris_clipped == 1, "oversized triangle really is clipped");

    t = fresh();
    raster_triangle(&t, &wc, &wb, &wa, 120);
    check(memcmp(d1, g_depth, sizeof d1) == 0, "clipped coverage identical under reversal");
}

/*
 * Degenerate input. Zero-area and collinear triangles must be counted and
 * dropped, never divided by. Spec §3: one 0/0 breaks bit-identity across
 * targets by specification, because wasm NaN payloads are nondeterministic.
 */
static void test_degenerate(void)
{
    printf("degenerate triangles\n");
    RVert a = vert(-4.0, 3.0, 10.0);

    RTarget t = fresh();
    raster_triangle(&t, &a, &a, &a, 100);
    check(t.pixels_written == 0, "point triangle writes nothing");

    RVert p = vert(-4.0, -4.0, 10.0), q = vert(0.0, 0.0, 10.0), r = vert(4.0, 4.0, 10.0);
    t = fresh();
    raster_triangle(&t, &p, &q, &r, 100);
    check(t.pixels_written == 0, "collinear triangle writes nothing");
    check(t.tris_degenerate >= 1, "degenerate triangles are counted");

    /* A NaN must not reach the kernel as a coordinate. */
    RVert n = vert(0.0 / 1.0, 0.0, 10.0);
    n.x = NAN;
    t = fresh();
    raster_triangle(&t, &n, &q, &r, 100);
    check(t.pixels_written == 0, "NaN vertex writes nothing");
}

/*
 * Depth. Nearer wins; farther loses; an exact tie goes to whichever triangle
 * was submitted FIRST, because the compare is strict `>`. Spec §3 requires
 * that operator to be pinned, so this test is what pins it.
 */
static void test_depth(void)
{
    printf("depth compare and ties\n");
    RVert far0 = vert(-5.0, 5.0, 20.0), far1 = vert(5.0, 5.0, 20.0), far2 = vert(0.0, -5.0, 20.0);
    RVert nr0  = vert(-5.0, 5.0, 10.0), nr1  = vert(5.0, 5.0, 10.0), nr2  = vert(0.0, -5.0, 10.0);

    /* near drawn second must win */
    RTarget t = fresh();
    raster_triangle(&t, &far0, &far1, &far2, 100);
    raster_triangle(&t, &nr0, &nr1, &nr2, 150);
    check(g_color[(H / 2) * W + W / 2] == 150, "nearer triangle overwrites farther");

    /* near drawn first must survive */
    t = fresh();
    raster_triangle(&t, &nr0, &nr1, &nr2, 150);
    raster_triangle(&t, &far0, &far1, &far2, 100);
    check(g_color[(H / 2) * W + W / 2] == 150, "farther triangle does not overwrite nearer");

    /* exact tie: first submission wins */
    t = fresh();
    raster_triangle(&t, &nr0, &nr1, &nr2, 150);
    long wrote = t.pixels_written;
    raster_triangle(&t, &nr0, &nr1, &nr2, 100);
    check(g_color[(H / 2) * W + W / 2] == 150, "equal depth: first submission wins");
    check(t.pixels_written == wrote, "equal depth: second submission writes no pixels");
}

/*
 * Masking. Nothing outside the triangle may be touched, and nothing outside
 * the viewport may be written at all — the latter is a memory-safety check as
 * much as a correctness one.
 */
static void test_mask(void)
{
    printf("masking and viewport containment\n");
    RVert a = vert(-2.0, 2.0, 10.0), b = vert(2.0, 2.0, 10.0), c = vert(0.0, -2.0, 10.0);
    RTarget t = fresh();
    raster_triangle(&t, &a, &b, &c, 100);

    int n = count_covered(g_depth);
    check(n > 0, "small triangle covers something");
    check(n < W * H, "small triangle does not cover the screen");
    check(t.pixels_written == n, "every written pixel is distinct");

    /* Corners are far outside a triangle centred on screen. */
    check(!covered(g_depth, 0), "top-left corner untouched");
    check(!covered(g_depth, W - 1), "top-right corner untouched");
    check(!covered(g_depth, (H - 1) * W), "bottom-left corner untouched");
    check(!covered(g_depth, W * H - 1), "bottom-right corner untouched");
}

/*
 * Clipping. Every plane gets a triangle that straddles it. The plane
 * equations are RE-DERIVED here rather than shared with raster.c: a golden
 * that calls the same helper it is testing only proves self-consistency.
 */
static void test_clip_planes(void)
{
    printf("frustum clipping, all six planes\n");
    RTarget t = fresh();

    struct { const char *name; RVert v[3]; } cases[] = {
        { "near",   { vert(-3.0,  3.0, -5.0), vert( 3.0,  3.0, 20.0), vert( 0.0, -3.0, 20.0) } },
        { "far",    { vert(-3.0,  3.0, 900.0), vert( 3.0,  3.0, 2000.0), vert( 0.0, -3.0, 2000.0) } },
        { "left",   { vert(-900.0, 3.0, 20.0), vert( 3.0,  3.0, 20.0), vert( 0.0, -3.0, 20.0) } },
        { "right",  { vert( 900.0, 3.0, 20.0), vert(-3.0,  3.0, 20.0), vert( 0.0, -3.0, 20.0) } },
        { "top",    { vert(-3.0, 900.0, 20.0), vert( 3.0, -3.0, 20.0), vert( 0.0, -3.0, 20.0) } },
        { "bottom", { vert(-3.0, -900.0, 20.0), vert( 3.0,  3.0, 20.0), vert( 0.0,  3.0, 20.0) } },
    };

    for (unsigned k = 0; k < sizeof cases / sizeof cases[0]; k++) {
        RVert out[RASTER_CLIP_MAX];
        int m = raster_clip_poly(&t, cases[k].v, 3, out);
        char msg[128];

        snprintf(msg, sizeof msg, "%s: clip produces a polygon", cases[k].name);
        check(m >= 3, msg);

        int inside = 1;
        for (int i = 0; i < m; i++) {
            /* Independently re-derived bounds, with a subpixel of slack for
             * the interpolation rounding at the plane itself. */
            const double eps = 1e-6;
            if (out[i].z < t.znear - eps || out[i].z > t.zfar + eps) inside = 0;
            double sx = t.cx + t.f * out[i].x / out[i].z;
            double sy = t.cy - t.f * out[i].y / out[i].z;
            if (sx < -eps || sx > W + eps || sy < -eps || sy > H + eps) inside = 0;
        }
        snprintf(msg, sizeof msg, "%s: every clipped vertex is inside the frustum",
                 cases[k].name);
        check(inside, msg);
    }

    /* Wholly behind the eye: nothing survives. */
    RVert behind[3] = { vert(-3.0, 3.0, -20.0), vert(3.0, 3.0, -20.0), vert(0.0, -3.0, -20.0) };
    RVert out[RASTER_CLIP_MAX];
    check(raster_clip_poly(&t, behind, 3, out) == 0, "fully-behind triangle is rejected");

    /* A triangle straddling the near plane must still fill pixels. */
    RTarget t2 = fresh();
    raster_triangle(&t2, &cases[0].v[0], &cases[0].v[1], &cases[0].v[2], 100);
    check(t2.pixels_written > 0, "near-straddling triangle still rasterizes");
}

/*
 * Perspective. A ground quad receding from the camera must produce depth that
 * decreases monotonically with distance. This is what would catch someone
 * "fixing" the depth buffer to interpolate z instead of 1/z.
 */
static void test_perspective(void)
{
    printf("perspective depth ordering\n");
    RVert g[4] = {
        vert(-40.0, -4.0,  10.0),
        vert( 40.0, -4.0,  10.0),
        vert( 40.0, -4.0, 400.0),
        vert(-40.0, -4.0, 400.0),
    };
    RTarget t = fresh();
    raster_polygon(&t, g, 4, 100);
    check(t.pixels_written > 0, "ground quad rasterizes");

    /* Walk up the screen (toward the horizon); depth must not increase. */
    int bad = 0, rows = 0, first = -1, last = -1;
    uint32_t prev = 0xffffffffu;
    for (int y = H - 1; y >= 0; y--) {
        uint32_t d = g_depth[y * W + W / 2];
        if (!d) continue;
        if (first < 0) first = y;
        last = y;
        if (rows && d > prev) bad++;
        prev = d; rows++;
    }
    printf("  covered=%d rows[%d..%d] n=%d clipped=%ld drawn=%ld\n",
           count_covered(g_depth), last, first, rows, t.tris_clipped, t.tris_drawn);
    check(rows > 4, "ground quad spans several rows at screen centre");
    check(bad == 0, "reciprocal depth decreases toward the horizon");
}

/* Palette tables: the reserved indices must be unreachable, or shading
 * punches transparent holes through the world (spec §4). */
static void test_palette(void)
{
    printf("palette tables and reserved indices\n");
    int leaked = 0;
    for (int i = 0; i < 256; i++)
        for (int l = 0; l < RASTER_SHADE_LEVELS; l++) {
            uint8_t s = raster_shade((uint8_t)i, l);
            if (s == RASTER_KEY_INDEX)
                leaked++;
        }
    check(leaked == 0, "no shade level lands on the colour key");

    leaked = 0;
    for (int i = 0; i < 256; i++)
        for (int l = 0; l < RASTER_FOG_LEVELS; l++) {
            uint8_t s = raster_fog((uint8_t)i, l);
            if (s == RASTER_KEY_INDEX)
                leaked++;
        }
    check(leaked == 0, "no fog level lands on the colour key");

    /* With a grey ramp palette, full light returns the base colour itself. */
    check(raster_shade(100, RASTER_SHADE_LEVELS - 1) == 100, "full light is the base colour");
    check(raster_shade(100, 0) < 8, "zero light is near black");
    /* Exact, not approximate — the search is not quantized. See the comment
     * on raster_rgb_to_index for why the 5:5:5 cube was dropped. */
    check(raster_rgb_to_index(100, 100, 100) == 100, "rgb lookup is exact");
    check(raster_rgb_to_index(37, 37, 37) == 37, "rgb lookup is exact off the 8-grid");
}

/* ---------------------------------------------------------------------- */
/* M8 V2 — the textured fragment stage                                      */
/*                                                                          */
/* The texture path shipped with ZERO kernel coverage, which is the gap the  */
/* spec's own review named: native<->wasm equality proves reproducibility,   */
/* not correctness, and it would happily pin a swapped u/v forever. These    */
/* are the properties that can be checked without an original-binary oracle. */
/* ---------------------------------------------------------------------- */

/* A tile whose texels encode their own coordinates, so a sampled framebuffer
 * pixel names the texel it came from. Under the synthetic grey palette a base
 * index reads straight back out, so at full light shade[i][MAX] == i. */
#define TILE_W 8
#define TILE_H 8
static uint8_t g_tile_px[TILE_W * TILE_H];

static RTex tile_uv(void)
{
    for (int y = 0; y < TILE_H; y++)
        for (int x = 0; x < TILE_W; x++)
            g_tile_px[y * TILE_W + x] = (uint8_t)(16 + y * TILE_W + x);
    RTex tx;
    tx.texels = g_tile_px;
    tx.w = TILE_W; tx.h = TILE_H;
    tx.umask = TILE_W - 1; tx.vmask = TILE_H - 1;
    tx.vshift = 3;               /* log2(8) */
    tx.has_key = 0;
    return tx;
}

/* A screen-facing quad at constant depth, so sampling is exact and any error
 * is a mapping error rather than a perspective one. `u0..v1` are NORMALISED;
 * raster.c scales them by the tile's dimensions at projection. */
static void quad(RVert v[4], double z, float u0, float v0, float u1, float v1)
{
    v[0] = vert(-2.0,  2.0, z); v[0].u = u0; v[0].v = v0;
    v[1] = vert( 2.0,  2.0, z); v[1].u = u1; v[1].v = v0;
    v[2] = vert( 2.0, -2.0, z); v[2].u = u1; v[2].v = v1;
    v[3] = vert(-2.0, -2.0, z); v[3].u = u0; v[3].v = v1;
}

static void test_texture_sampling(void)
{
    printf("textured fragment stage\n");
    RTex tx = tile_uv();
    RVert v[4];

    /* 1. NULL tex is bit-identical to the flat entry point. This is the
     *    contract raster.h states and the reason 19 existing goldens did not
     *    have to change; if it ever breaks, every one of them is lying. */
    quad(v, 10.0, 0.0f, 0.0f, 1.0f, 1.0f);
    RTarget t = fresh();
    raster_polygon(&t, v, 4, 100);
    uint8_t flat_fb[W * H];
    uint32_t flat_d[W * H];
    memcpy(flat_fb, g_color, sizeof flat_fb);
    memcpy(flat_d, g_depth, sizeof flat_d);
    long flat_written = t.pixels_written;

    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, NULL, 0);
    check(memcmp(flat_fb, g_color, sizeof flat_fb) == 0,
          "tex=NULL colour is byte-identical to raster_polygon");
    check(memcmp(flat_d, g_depth, sizeof flat_d) == 0,
          "tex=NULL depth is byte-identical to raster_polygon");
    check(t.pixels_textured == 0, "tex=NULL samples nothing");

    /* 2. A textured quad samples, and it samples MORE THAN ONE texel. A single
     *    value everywhere is what a broken interpolation looks like, and it
     *    would still pass a "did it draw?" check. */
    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 0);
    check(t.pixels_textured == flat_written,
          "every covered fragment sampled the tile");
    int distinct = 0;
    unsigned char seen[256] = {0};
    for (int i = 0; i < W * H; i++)
        if (covered(g_depth, i) && !seen[g_color[i]]) { seen[g_color[i]] = 1; distinct++; }
    check(distinct > 8, "sampling varies across the face (not one flat texel)");

    /* 3. ORIENTATION. u runs +x on screen, v runs +y DOWNWARD -- row 0 of the
     *    tile is the top row (texcache.h), and screen y is also down, so the
     *    two agree without a flip. Read the actual corners rather than
     *    asserting a whole-image hash, because a hash cannot tell a rotation
     *    from a mirror. Texel value is 16 + v*8 + u under tile_uv(). */
    int left = -1, right = -1, top = -1, bottom = -1;
    for (int x = 0; x < W; x++)
        if (covered(g_depth, (H / 2) * W + x)) { left = g_color[(H / 2) * W + x]; break; }
    for (int x = W - 1; x >= 0; x--)
        if (covered(g_depth, (H / 2) * W + x)) { right = g_color[(H / 2) * W + x]; break; }
    for (int y = 0; y < H; y++)
        if (covered(g_depth, y * W + W / 2)) { top = g_color[y * W + W / 2]; break; }
    for (int y = H - 1; y >= 0; y--)
        if (covered(g_depth, y * W + W / 2)) { bottom = g_color[y * W + W / 2]; break; }
    check(left >= 0 && right >= 0 && top >= 0 && bottom >= 0, "quad edges found");
    check(((right - 16) & 7) > ((left - 16) & 7),
          "u increases to the RIGHT on screen");
    check(((bottom - 16) >> 3) > ((top - 16) >> 3),
          "v increases DOWNWARD on screen (tile row 0 is the top)");

    /* 4. WRAP, not clamp. tools/tex_verify.c measured 1195 UV components
     *    greater than 1 in the shipped geometry, so a quad spanning u in
     *    [0,4] must repeat the tile four times, and the far edge must NOT be
     *    pinned to the last texel the way a clamp would pin it. */
    quad(v, 10.0, 0.0f, 0.0f, 4.0f, 1.0f);
    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 0);
    int cols = 0, prev = -1, wraps = 0;
    for (int x = 0; x < W; x++) {
        int i = (H / 2) * W + x;
        if (!covered(g_depth, i)) continue;
        int u = (g_color[i] - 16) & 7;
        if (prev >= 0 && u < prev) wraps++;
        prev = u; cols++;
    }
    check(cols > 8, "wrapped quad covers a span");
    check(wraps >= 3, "u wraps repeatedly across a 4x quad (not clamped)");

    /* 5. NEGATIVE UVs floor rather than truncate. 2777 components in the
     *    shipped geometry are negative, and C truncation toward zero would put
     *    a one-texel seam at every integer boundary. */
    quad(v, 10.0, -1.0f, 0.0f, 0.0f, 1.0f);
    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 0);
    check(t.pixels_textured > 0, "negative UVs still sample");
    check(t.faces_uv_clamped == 0, "ordinary negative UVs are not clamped");
}

static void test_texture_cutout(void)
{
    printf("cut-out keying\n");
    RTex tx = tile_uv();
    /* Punch the key value into the left half of the tile. */
    for (int y = 0; y < TILE_H; y++)
        for (int x = 0; x < TILE_W / 2; x++)
            g_tile_px[y * TILE_W + x] = RASTER_TEXEL_TRANSPARENT;
    tx.has_key = 1;

    RVert v[4];
    quad(v, 10.0, 0.0f, 0.0f, 1.0f, 1.0f);

    /* THE property, checked directly rather than via a general
     * order-independence claim: a keyed fragment on a cut-out face writes
     * NEITHER colour NOR depth. Writing depth is the failure that makes an
     * invisible texel occlude the world behind a fence. */
    RTarget t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 1);
    check(t.pixels_keyed > 0, "cut-out face discarded some fragments");
    check(t.pixels_written > 0, "cut-out face still drew its opaque half");
    check(t.pixels_written + t.pixels_keyed == t.pixels_textured,
          "every sampled fragment either wrote or was discarded");

    int keyed_depth = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (g_depth[y * W + x] && g_color[y * W + x] == 0) keyed_depth++;
    check(keyed_depth == 0, "no discarded fragment left a depth value behind");

    /* The same 0xFF sample on an OPAQUE material is an ordinary palette
     * index, not a request to substitute the face's unrelated flat colour.
     * Nitro's unkeyed blitter mode maps every source byte; only keyed modes
     * skip 0xFF (phase-d-renderer.md §3.1). With this synthetic identity
     * palette every counted opaque-key sample must therefore write index 255.
     * The former flat fallback wrote base 100 — H-UAT-063's grey->yellow
     * mechanism under real level palettes. */
    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 0);
    int opaque_255 = 0;
    for (int i = 0; i < W * H; i++)
        if (g_color[i] == RASTER_TEXEL_TRANSPARENT) opaque_255++;
    check(t.texels_keyed_opaque > 0, "opaque material samples index 255");
    check(opaque_255 == t.texels_keyed_opaque,
          "opaque index 255 shades through its palette row, never flat fallback");

    /* A cut-out quad in FRONT of a solid one must let the solid one show
     * through its holes -- and must do so whichever order they are submitted,
     * because a discarded fragment writes no depth to occlude with. */
    RVert back[4];
    quad(back, 20.0, 0.0f, 0.0f, 1.0f, 1.0f);
    /* The backdrop needs its own texels: tile_uv() rewrites g_tile_px,
     * which still backs the keyed cut-out texture tx. */
    static uint8_t back_px[TILE_W * TILE_H];
    for (int i = 0; i < TILE_W * TILE_H; i++) back_px[i] = (uint8_t)(16 + i);
    RTex opaque = tx;
    opaque.texels = back_px;
    opaque.has_key = 0;
    check(tx.texels[0] == RASTER_TEXEL_TRANSPARENT,
          "the cut-out keeps its transparent texels for this regression");

    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 1);          /* cut-out alone */
    uint8_t near_only[W * H];
    memcpy(near_only, g_color, sizeof near_only);

    t = fresh();
    raster_polygon_tex(&t, back, 4, 100, &opaque, 0);   /* far first */
    raster_polygon_tex(&t, v, 4, 100, &tx, 1);          /* near cut-out second */
    uint8_t far_first[W * H];
    memcpy(far_first, g_color, sizeof far_first);

    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 1);          /* near cut-out first */
    raster_polygon_tex(&t, back, 4, 100, &opaque, 0);   /* far second */
    check(memcmp(far_first, g_color, sizeof far_first) == 0,
          "a cut-out over a solid surface is submission-order independent");

    /* The farther quad projects inside the near quad's footprint, so a
     * pixel that is empty with the cut-out alone but drawn with the
     * backdrop is the backdrop seen through a hole. */
    int through = 0;
    for (int i = 0; i < W * H; i++)
        if (!near_only[i] && far_first[i]) through++;
    check(through > 0, "the surface behind the cut-out is visible through it");
}

/*
 * The malformed-UV policy, which is LOAD-BEARING rather than defensive:
 * tools/tex_verify.c measured UV components at -1.69e38 in the shipped
 * geometry. Converting that to int is undefined behaviour in C and a hard
 * i32.trunc_f64_s trap in wasm, so this test is the difference between a
 * rendering artefact and the browser tab dying on a real asset.
 */
static void test_texture_malformed_uv(void)
{
    printf("malformed UVs do not trap\n");
    RTex tx = tile_uv();
    RVert v[4];

    quad(v, 10.0, -1.69e38f, 0.0f, 1.0f, 1.0f);
    RTarget t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 0);
    check(t.pixels_textured > 0, "a face with garbage UVs still rasterizes");
    check(t.faces_uv_clamped >= 0, "clamped-face counter is readable");

    /* NaN and infinity reach the same path through the !(x > y) comparisons. */
    quad(v, 10.0, 0.0f, 0.0f, 1.0f, 1.0f);
    v[0].u = (float)(0.0 / 0.0);
    v[1].v = (float)(1.0 / 0.0);
    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &tx, 0);
    check(1, "NaN and Inf UVs did not trap");
}

/* ---------------------------------------------------------------------- */
/* Native mission tables (docs/specs/re/phase-d-renderer.md)                 */
/*                                                                          */
/* Like everything else in this file these use SYNTHETIC tables, not the    */
/* shipped .lum/.tbl: the kernel golden must run without game assets. The   */
/* synthetic layout mirrors the measured properties of the real files: an   */
/* authored identity row (7, the common case), darkening rows above it,     */
/* saturation at row 31, rows 32+ repeating.                                */
/* ---------------------------------------------------------------------- */

static uint8_t g_lum[256 * 256];

static void synth_lum(void)
{
    for (int i = 0; i < 256; i++) {
        for (int r = 0; r < 7; r++)      /* overbright rows (unselectable) */
            g_lum[r * 256 + i] = (uint8_t)(i + 40 > 255 ? 255 : i + 40);
        g_lum[7 * 256 + i] = (uint8_t)i;                 /* identity row  */
        for (int r = 8; r < 32; r++)     /* darkening rows */
            g_lum[r * 256 + i] = (uint8_t)(i * (32 - r) / 25);
        for (int r = 32; r < 256; r++)   /* saturation repeats row 31 */
            g_lum[r * 256 + i] = g_lum[31 * 256 + i];
    }
}

static void test_shade_table(void)
{
    printf("native shade table (.lum)\n");
    synth_lum();

    check(raster_set_shade_table(g_lum) == 0, "valid .lum installs");
    check(raster_shade_table_native(), "native flag reports installed");

    /* Level L-1 is "unshaded" (raster.h's contract) and must land exactly on
     * the detected identity row. */
    check(raster_shade(100, RASTER_SHADE_LEVELS - 1) == 100,
          "full light is the identity row");
    check(raster_shade(37, RASTER_SHADE_LEVELS - 1) == 37,
          "full light is the identity row off the grid");

    /* Level 0 lands on the saturation row: lum[31][i] = i/25. */
    check(raster_shade(200, 0) == 8, "zero light is the dark saturation row");
    /* The mapping walks the native rows: level 16 -> row 7 + (15*24)/31 = 18,
     * and lum[18][i] = i*14/25. */
    check(raster_shade(200, 16) == 200 * 14 / 25, "mid levels pick native rows");

    /* Every level's value comes from a native row in [identity, dark], and
     * brightening never happens as light falls. */
    int prev = -1, monotone = 1, in_band = 1;
    for (int l = RASTER_SHADE_LEVELS - 1; l >= 0; l--) {
        int s = raster_shade(200, l);
        if (prev >= 0 && s > prev) monotone = 0;
        prev = s;
        int found = 0;
        for (int r = 7; r <= 31; r++)
            if (g_lum[r * 256 + 200] == s) found = 1;
        if (!found) in_band = 0;
    }
    check(monotone, "darkening is monotonic with falling light");
    check(in_band, "every level value comes from a native dark-range row");

    /* A table with no identity row is REJECTED and changes nothing. */
    uint8_t bogus[256 * 256];
    memset(bogus, 0, sizeof bogus);
    check(raster_set_shade_table(bogus) == -1, "identity-less table rejected");
    check(raster_shade(100, RASTER_SHADE_LEVELS - 1) == 100,
          "rejected table left the ramps untouched");

    /* NULL restores the bounded placeholder fallback. */
    raster_set_shade_table(NULL);
    check(!raster_shade_table_native(), "NULL uninstalls the native table");
    check(raster_shade(100, RASTER_SHADE_LEVELS - 1) == 100,
          "placeholder still identity at full light (grey palette)");
    check(raster_shade(100, 0) < 8, "placeholder still near black at zero light");
}

/* A tile whose every texel is the same value, so a translucent quad's output
 * names the table cell it came from rather than a sampling position. */
static uint8_t g_const_px[TILE_W * TILE_H];

static RTex tile_const(uint8_t texel)
{
    memset(g_const_px, texel, sizeof g_const_px);
    RTex tx;
    tx.texels = g_const_px;
    tx.w = TILE_W; tx.h = TILE_H;
    tx.umask = TILE_W - 1; tx.vmask = TILE_H - 1;
    tx.vshift = 3;
    tx.has_key = 0;
    return tx;
}

static void test_translucency(void)
{
    printf("translucency LUT (destination-indexed)\n");

    /* Asymmetric blend map: T[(a<<8)|b] = 2a + b (mod 256). If the kernel
     * ever transposes the index, the expected values below all shift. */
    static uint8_t tbl[256 * 256];
    for (int a = 0; a < 256; a++)
        for (int b = 0; b < 256; b++)
            tbl[(a << 8) | b] = (uint8_t)(2 * a + b);
    raster_set_translucency(tbl);
    check(raster_translucency_active(), "table reports installed");

    RVert near_q[4], far_q[4];
    quad(near_q, 10.0, 0.0f, 0.0f, 1.0f, 1.0f);
    quad(far_q,  20.0, 0.0f, 0.0f, 1.0f, 1.0f);
    RTex tx = tile_const(50);       /* every sampled texel is 50 */

    /* Flat backdrop (dest = 90 at full light on the grey palette), then one
     * translucent quad over it. Expected outputs, with shade identity at
     * full light:
     *   over the backdrop:  T[(50<<8)|90]  = 2*50 + 90  = 190
     *   over background 0:  T[(50<<8)|0]   = 2*50 + 0   = 100
     * A transposed index would give 2*90+50 = 230 / 2*0+50 = 50 instead. */
    RTarget t = fresh();
    raster_triangle(&t, &far_q[0], &far_q[1], &far_q[2], 90);
    raster_triangle(&t, &far_q[0], &far_q[2], &far_q[3], 90);
    long opaque_written = t.pixels_written;
    raster_polygon_translucent(&t, near_q, 4, 100, &tx, 0);

    int n190 = 0, n100 = 0, nother = 0;
    for (int i = 0; i < W * H; i++) {
        if (!covered(g_depth, i)) continue;
        if (g_color[i] == 190) n190++;
        else if (g_color[i] == 100) n100++;
        else nother++;
    }
    printf("  over-dest=%d over-bg=%d other=%d translucent=%ld\n",
           n190, n100, nother, t.pixels_translucent);
    check(n190 > 0, "blend over a lit backdrop hits table[(texel<<8)|dest]");
    check(n100 > 0, "blend over background hits a DIFFERENT table cell");
    check(nother == 0, "no pixel took the transposed-index or unblended value");
    check(t.pixels_translucent > opaque_written,
          "the translucent quad composed through the LUT");

    /* DESTINATION DEPENDENCE, isolated: same texel, two dest values, two
     * different outputs — proved by the 190/100 split above; here assert the
     * exact formula on a pixel whose dest is known. */
    check(g_color[(H / 2) * W + W / 2] == 190,
          "screen centre: out == table[(texel<<8)|dest] exactly");

    /* OPAQUE OUTPUT UNCHANGED by the table's presence. */
    RVert v[4];
    quad(v, 10.0, 0.0f, 0.0f, 1.0f, 1.0f);
    RTex varying = tile_uv();
    static uint8_t with_tbl[W * H], without_tbl[W * H];

    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &varying, 0);
    memcpy(with_tbl, g_color, sizeof with_tbl);
    check(t.pixels_translucent == 0, "opaque path never touches the LUT");

    raster_set_translucency(NULL);
    check(!raster_translucency_active(), "NULL uninstalls the table");
    t = fresh();
    raster_polygon_tex(&t, v, 4, 100, &varying, 0);
    memcpy(without_tbl, g_color, sizeof without_tbl);
    check(memcmp(with_tbl, without_tbl, sizeof with_tbl) == 0,
          "opaque output byte-identical with and without the table");

    /* FALLBACK: translucent with no table == opaque, byte for byte. */
    static uint8_t transl_fallback[W * H];
    t = fresh();
    raster_polygon_translucent(&t, v, 4, 100, &varying, 0);
    memcpy(transl_fallback, g_color, sizeof transl_fallback);
    check(memcmp(transl_fallback, without_tbl, sizeof transl_fallback) == 0,
          "no-table translucent falls back to opaque shading");
    check(t.pixels_translucent == 0, "fallback does not count LUT blends");

    /* CUTOUT PRESERVED in the translucent material: reinstall the table,
     * key the left half of the tile, and a cut-out face must still discard —
     * no colour, no depth — because the shipped tables' 0xFF row is not a
     * passthrough (measured). */
    raster_set_translucency(tbl);
    RTex keyed = tile_uv();
    for (int y = 0; y < TILE_H; y++)
        for (int x = 0; x < TILE_W / 2; x++)
            g_tile_px[y * TILE_W + x] = RASTER_TEXEL_TRANSPARENT;
    keyed.has_key = 1;
    t = fresh();
    raster_polygon_translucent(&t, v, 4, 100, &keyed, 1);
    check(t.pixels_keyed > 0, "translucent cut-out discarded fragments");
    check(t.pixels_written + t.pixels_keyed == t.pixels_textured,
          "every translucent fragment either wrote or was discarded");
    int keyed_depth = 0;
    for (int i = 0; i < W * H; i++)
        if (g_depth[i] && g_color[i] == 0) keyed_depth++;
    check(keyed_depth == 0, "no translucent cut-out left depth behind");

    raster_set_translucency(NULL);   /* leave the module in fallback state */
}

int main(void)
{
    synth_palette();
    check(raster_palette_ready(), "palette tables built");

    test_projection_contract();
    test_palette();
    test_shared_edge();
    test_winding();
    test_degenerate();
    test_depth();
    test_mask();
    test_clip_planes();
    test_perspective();
    test_texture_sampling();
    test_texture_cutout();
    test_texture_malformed_uv();
    test_shade_table();
    test_translucency();

    printf("\nraster_test: %d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
