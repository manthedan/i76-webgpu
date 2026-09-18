/*
 * sky_probe.c — focused gate for the purchaser-authored mission sky.
 *
 * The mission's WDEF/WRLD +82 field names a raw indexed .map (scene.md §2);
 * scene.c parses it, texcache_load_map() decodes it, and worldrender.c's
 * filled path samples it as a wrapping equirectangular dome instead of the
 * placeholder gradient. This probe proves, end to end on P01:
 *
 *  1. SELECTION: P01's sky field resolves to nk_1cld3.map and decodes
 *     (128x128 POT tile).
 *  2. OUTPUT: the rendered sky carries that texture's texels — checked by
 *     REIMPLEMENTING the dome mapping here with libm (a gate that imports
 *     the thing it tests cannot catch it being wrong) and comparing texels
 *     on a sky-pixel grid. The engine's deterministic polynomial atan2/asin
 *     (no libm, so the native<->wasm frame gate cannot diverge) must agree
 *     with libm except at a vanishing share of texel-boundary pixels.
 *  3. NOT THE GRADIENT: the gradient writes constant-colour rows for an
 *     unrolled camera; the textured sky must show within-row variation.
 *  4. DETERMINISM: the same camera rendered twice hashes identically, in
 *     colour AND depth.
 *  5. ORIENTATION: yaw and pitch change the sampled texels.
 *  6. FALLBACK + DEPTH ISOLATION: a synthetic copy of P01 with the 13-byte
 *     sky field zeroed (built in a sandbox vfs root; purchaser data is
 *     never modified) renders the gradient signature, and its depth buffer
 *     hashes EXACTLY equal to the textured run's — the sky path cannot
 *     disturb foreground depth.
 *  7. LOADER REJECTION: a nonexistent .map name fails cleanly.
 *
 * Build (from the candidate root):
 *   OUT=/external/path tools/build_probe.sh sky_probe
 *
 * Usage: sky_probe [--ppm prefix]
 *        (asset root from required $NITRO_APP; scratch from required
 *         $I76_VERIFY_TMP)
 */
#include <dirent.h>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

#include "engine/fs.h"
#include "engine/vfs.h"
#include "engine/worldrender.h"
#include "engine/raster.h"
#include "engine/terrain.h"
#include "engine/scene.h"
#include "engine/hud.h"
#include "engine/texcache.h"

#define FB_W 640
#define FB_H 480

static uint8_t s_fb[FB_W * FB_H];
static uint8_t fb3[FB_W * FB_H];    /* skyline-phase pose renders */

static uint8_t fb_vista[FB_W * FB_H];
static uint32_t depth_vista[FB_W * FB_H];
static int g_fail;
static void check(int cond, const char *what)
{
    printf("%s %s\n", cond ? "ok  " : "FAIL", what);
    if (!cond) g_fail++;
}

static uint32_t fnv1a(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

/* Fixed camera over P01's spawn cluster, near level, looking north. Pinned
 * constants: the fallback phase must reproduce this pose bit for bit. */
static const double EYE[3] = { 2407.5, 40.0, 49392.5 };
static const double TGT[3] = { 2407.5, 30.0, 49492.5 };

static void render(uint8_t *fb, const CameraView *cam,
                   int have_terrain, int have_scene)
{
    worldrender_set_backend(WORLD_BACKEND_FILLED);
    worldrender_camera(fb, FB_W, FB_H, cam, 0.0, 0.0, have_terrain, have_scene);
}

/* Fraction of rows in [y0,y1) whose SKY pixels (depth == 0) carry more than
 * one distinct palette index. The gradient scores 0 for an unrolled camera;
 * the textured dome scores high. */
static double sky_row_variation(const uint8_t *fb, const uint32_t *depth,
                                int y0, int y1)
{
    int varied = 0, rows = 0;
    for (int y = y0; y < y1; y++) {
        int seen[256];
        memset(seen, 0, sizeof seen);
        int distinct = 0, sky_px = 0;
        for (int x = 0; x < FB_W; x++) {
            size_t i = (size_t)y * FB_W + x;
            if (depth[i]) continue;
            sky_px++;
            if (!seen[fb[i]]) { seen[fb[i]] = 1; distinct++; }
        }
        if (sky_px < FB_W / 2) continue;    /* mostly-terrain rows tell nothing */
        rows++;
        if (distinct > 1) varied++;
    }
    return rows ? (double)varied / rows : 0.0;
}

/*
 * Independent re-implementation of the engine's sky mapping using libm:
 * equirectangular dome, u = azimuth (4 wraps/turn), v = 64 - el * 256.
 * Returns the expected palette index for screen (x, y), or -1 when the
 * pixel is not sky (depth != 0).
 */
static int expected_sky_texel(const CameraView *cam, const RTex *sky,
                              const uint32_t *depth, double f, int x, int y)
{
    if (depth[(size_t)y * FB_W + x]) return -1;
    double px = (double)x - FB_W * 0.5;
    double py = FB_H * 0.5 - (double)y;
    double dx = f * cam->forward[0] + px * cam->right[0] + py * cam->up[0];
    double dy = f * cam->forward[1] + px * cam->right[1] + py * cam->up[1];
    double dz = f * cam->forward[2] + px * cam->right[2] + py * cam->up[2];
    double inv = 1.0 / sqrt(dx * dx + dy * dy + dz * dz);
    double az = atan2(dx, dz);
    double el = asin(dy * inv < -1.0 ? -1.0 : (dy * inv > 1.0 ? 1.0 : dy * inv));
    int ti = (int)floor((0.5 + az / (2.0 * M_PI) * 4.0) * sky->w) & sky->umask;
    int tj = (int)floor(64.0 - el * 256.0) & sky->vmask;
    return sky->texels[(tj << sky->vshift) | ti];
}

static void write_ppm(const char *path, const uint8_t *fb, const uint8_t *pal)
{
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", FB_W, FB_H);
    for (int i = 0; i < FB_W * FB_H; i++)
        fwrite(pal + fb[i] * 3, 1, 3, f);
    fclose(f);
}

/* --- distant-skyline measurement (phase 1.5) -------------------------- */

/* Inverse of raster.c's depth_quantize() at the clip planes this probe
 * renders with (render() passes near/far 0 -> RASTER_ZNEAR/RASTER_ZFAR):
 * the stored value is dn * (2^32-1), dn interpolating 1/z linearly from
 * 1/znear (dn=1) to 1/zfar (dn=0). */
static double depth_to_z(uint32_t d)
{
    double dn = (double)d / 4294967295.0;
    double invz = dn * (1.0 / RASTER_ZNEAR - 1.0 / RASTER_ZFAR) +
                1.0 / RASTER_ZFAR;
    return 1.0 / invz;
}

/* Row of the zero-elevation (horizon) ray in column x, from the same view
 * ray model the renderer samples: d = f*F + (x-cx)*R + (cy-y)*U, horizon
 * where d.y == 0. Per-column so a pitched/rolled camera is handled. */
static double horizon_row(const CameraView *cam, double f, int x)
{
    double ry = f * cam->forward[1] +
                ((double)x - FB_W * 0.5) * cam->right[1];
    return FB_H * 0.5 + ry / cam->up[1];
}

/* First row from the top carrying geometry (depth != 0); FB_H when the
 * column is sky all the way down. */
static int skyline_top_row(const uint32_t *depth, int x)
{
    for (int y = 0; y < FB_H; y++)
        if (depth[(size_t)y * FB_W + x]) return y;
    return FB_H;
}

struct skyline_stat {
    int     columns;        /* silhouette rising above the horizon row   */
    double  z_min, z_med;   /* z of silhouette-top pixels (metres)       */
    int     top[FB_W];      /* per-column silhouette top row (FB_H none) */
};

static void skyline_measure(const CameraView *cam, const uint32_t *depth,
                            struct skyline_stat *st)
{
    double f = (FB_W * 0.5) / camera_view_fov_tan_half(cam);
    double zs[FB_W];
    int n = 0;
    st->z_min = 1e30;
    st->z_med = 0.0;
    for (int x = 0; x < FB_W; x++) {
        int top = skyline_top_row(depth, x);
        double hr = horizon_row(cam, f, x);
        if (top < hr - 1.5) {
            double z = depth_to_z(depth[(size_t)top * FB_W + x]);
            st->top[x] = top;
            zs[n++] = z;
            if (z < st->z_min) st->z_min = z;
        } else {
            st->top[x] = FB_H;
        }
    }
    st->columns = n;
    if (n) {
        /* insertion sort: n <= 640, once per pose */
        for (int i = 1; i < n; i++) {
            double v = zs[i];
            int j = i - 1;
            while (j >= 0 && zs[j] > v) { zs[j + 1] = zs[j]; j--; }
            zs[j + 1] = v;
        }
        st->z_med = zs[n / 2];
    }
}

struct skyline_contrast {
    int columns;
    int clearly_darker;
    double mean_gap;
};

static int palette_luma(const uint8_t *pal, uint8_t idx)
{
    const uint8_t *rgb = pal + (size_t)idx * 3;
    return (77 * rgb[0] + 150 * rgb[1] + 29 * rgb[2]) >> 8;
}

/*
 * Measure the actual colour discontinuity at distant silhouette tops.
 * Depth chooses authored terrain and the adjacent sky pixel independently of
 * palette index; the assertion therefore observes the range/cliff shading,
 * unlike the geometric silhouette-rise gate.
 */
static void skyline_contrast_measure(const struct skyline_stat *st,
                                     const uint8_t *fb,
                                     const uint32_t *depth,
                                     const uint8_t *pal,
                                     struct skyline_contrast *out)
{
    memset(out, 0, sizeof *out);
    double sum = 0.0;
    for (int x = 0; x < FB_W; x++) {
        int ground_y = st->top[x];
        if (ground_y <= 0 || ground_y >= FB_H) continue;
        size_t ground_i = (size_t)ground_y * FB_W + x;
        if (!depth[ground_i] || depth_to_z(depth[ground_i]) < 500.0) continue;

        int sky_y = ground_y - 1;
        while (sky_y >= 0 && depth[(size_t)sky_y * FB_W + x])
            sky_y--;
        if (sky_y < 0) continue;

        int gap = palette_luma(pal, fb[(size_t)sky_y * FB_W + x]) -
                  palette_luma(pal, fb[ground_i]);
        sum += gap;
        out->columns++;
        if (gap >= 8) out->clearly_darker++;
    }
    if (out->columns)
        out->mean_gap = sum / out->columns;
}

/* Silhouette-height profile: pixels the skyline rises above the horizon,
 * 0 where the column has no skyline. */
static void skyline_heights(const CameraView *cam, const int top[FB_W],
                            double h[FB_W])
{
    double f = (FB_W * 0.5) / camera_view_fov_tan_half(cam);
    for (int x = 0; x < FB_W; x++) {
        double hr = horizon_row(cam, f, x);
        h[x] = (top[x] < FB_H) ? hr - (double)top[x] : 0.0;
    }
}

/* Lateral shift s minimising mean |mov[x] - ref[x+s]| over the overlap.
 * A pure camera yaw translates the whole silhouette profile by the same
 * screen amount at every depth (no translation => no parallax), so the
 * best shift pins how the skyline moves under yaw. */
static int skyline_shift(const double ref[FB_W], const double mov[FB_W])
{
    int best = 0;
    double bestd = 1e30;
    for (int s = -FB_W + 1; s < FB_W; s++) {
        double d = 0.0;
        int n = 0;
        for (int x = 0; x < FB_W; x++) {
            int xr = x + s;
            if (xr < 0 || xr >= FB_W) continue;
            d += fabs(mov[x] - ref[xr]);
            n++;
        }
        if (n > FB_W / 2) {
            d /= n;
            if (d < bestd) { bestd = d; best = s; }
        }
    }
    return best;
}

/* Zero the 13-byte WDEF/WRLD +82 sky field in a mission image.
 * Returns 0 on success (field found and cleared). */
static int clear_sky_field(uint8_t *b, size_t n)
{
    for (size_t off = 0; off + 8 <= n; ) {
        uint32_t total;
        memcpy(&total, b + off + 4, 4);
        if (total < 8 || (size_t)total > n - off) break;
        if (!memcmp(b + off, "WDEF", 4)) {
            size_t end = off + total;
            for (size_t p = off + 8; p + 8 <= end; ) {
                uint32_t sub;
                memcpy(&sub, b + p + 4, 4);
                if (sub < 8 || (size_t)sub > end - p) break;
                if (!memcmp(b + p, "WRLD", 4) && sub >= 8 + 82 + 13) {
                    memset(b + p + 8 + 82, 0, 13);
                    return 0;
                }
                p += sub;
            }
            return -1;
        }
        off += total;
    }
    return -1;
}

int main(int argc, char **argv)
{
    const char *ppm_prefix = NULL;
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--ppm") && i + 1 < argc) ppm_prefix = argv[++i];

    const char *root = getenv("NITRO_APP");
    const char *tmp_root = getenv("I76_VERIFY_TMP");
    if (!root || !*root || !tmp_root || !*tmp_root) {
        fprintf(stderr, "sky_probe: NITRO_APP and I76_VERIFY_TMP are required\n");
        return 2;
    }
    fs_set_root(root);
    if (!vfs_init()) { fprintf(stderr, "sky_probe: vfs_init failed\n"); return 1; }

    /* --- phase 1: textured sky --------------------------------------- */
    const char *mission = "miss8/p01.msn";
    int have_terrain = terrain_load(mission) == 0;
    int have_scene = scene_load(mission) == 0;
    hud_load_mission(mission, 0);
    if (!have_terrain || !have_scene) {
        fprintf(stderr, "sky_probe: P01 terrain/scene load failed\n");
        return 1;
    }

    check(strcmp(scene_sky_name(), "nk_1cld3.map") == 0,
          "P01 WDEF/WRLD +82 names nk_1cld3.map");

    const RTex *sky = NULL;
    check(scene_sky_tex(&sky) == 1 && sky && sky->texels,
          "authored sky tile decoded and exposed");
    if (sky)
        check(sky->w == 128 && sky->h == 128,
              "nk_1cld3.map is the measured 128x128 tile");

    uint8_t *rej_px = (uint8_t *)1;
    RTex rej;
    check(texcache_load_map("zz_no_such_sky.map", &rej, &rej_px) == -1 &&
          rej_px == NULL,
          "missing .map name is rejected cleanly (gradient fallback arm)");

    CameraView cam;
    if (camera_view_look_at(&cam, EYE, TGT) != 0) {
        fprintf(stderr, "sky_probe: camera invalid\n");
        return 1;
    }

    render(s_fb, &cam, have_terrain, have_scene);
    const uint32_t *depth = worldrender_depth();
    check(depth != NULL, "filled backend produced a depth buffer");
    if (!depth) return 1;

    uint32_t fb_tex = fnv1a(s_fb, sizeof s_fb);
    uint32_t depth_tex = fnv1a((const uint8_t *)depth,
                               (size_t)FB_W * FB_H * sizeof(uint32_t));
    long geo = worldrender_geometry_pixels();
    check(geo > 0, "foreground geometry present in depth");
    printf("info textured fb=0x%08x depth=0x%08x geo=%ld\n",
           fb_tex, depth_tex, geo);

    /* 2. output carries the authored texels (libm re-implementation). */
    if (sky) {
        double f = (FB_W * 0.5) / 1.0;   /* RASTER_FOV_TAN_HALF is 1 (90 deg) */
        int cmp_n = 0, mismatch = 0, light = 0;
        for (int y = 0; y < FB_H; y += 7)
            for (int x = 0; x < FB_W; x += 7) {
                int e = expected_sky_texel(&cam, sky, depth, f, x, y);
                if (e < 0) continue;
                cmp_n++;
                if (s_fb[(size_t)y * FB_W + x] != e) mismatch++;
                if (e <= 233) light++;
            }
        printf("info grid=%d mismatch=%d light_wisp=%d\n",
               cmp_n, mismatch, light);
        check(cmp_n > 100, "sky pixel grid is nonempty");
        check(cmp_n > 0 && mismatch * 1000 <= cmp_n,
              "fb matches the dome mapping of nk_1cld3.map texels "
              "(<0.1% texel-boundary slack)");
        check(light > 0, "cloud wisp texels (idx <= 233) present in sky");
    }

    /* 3. not the gradient: textured sky varies within rows. */
    double var_tex = sky_row_variation(s_fb, depth, 0, 120);
    printf("info row_variation_textured=%.2f\n", var_tex);
    check(var_tex > 0.5, "sky rows vary within the row (not the 32-band "
                         "gradient, which is constant per row)");

    /* 4. determinism. */
    static uint8_t fb2[FB_W * FB_H];
    render(fb2, &cam, have_terrain, have_scene);
    const uint32_t *depth2 = worldrender_depth();
    check(memcmp(s_fb, fb2, sizeof s_fb) == 0 && depth2 &&
          fnv1a((const uint8_t *)depth2,
                (size_t)FB_W * FB_H * sizeof(uint32_t)) == depth_tex,
          "same camera renders identical colour AND depth twice");

    /* 5. orientation changes sampling. */
    double tgt[3];
    /* yaw +90 deg: target east of eye at the same pitch */
    tgt[0] = EYE[0] + 100.0; tgt[1] = TGT[1]; tgt[2] = EYE[2];
    CameraView cam_yaw;
    camera_view_look_at(&cam_yaw, EYE, tgt);
    render(fb2, &cam_yaw, have_terrain, have_scene);
    uint32_t fb_yaw = fnv1a(fb2, sizeof fb2);
    check(fb_yaw != fb_tex, "yaw +90 changes the sampled sky");
    /* pitch up: target above the eye */
    tgt[0] = EYE[0]; tgt[1] = EYE[1] + 60.0; tgt[2] = TGT[2];
    CameraView cam_pitch;
    camera_view_look_at(&cam_pitch, EYE, tgt);
    render(fb2, &cam_pitch, have_terrain, have_scene);
    uint32_t fb_pitch = fnv1a(fb2, sizeof fb2);
    check(fb_pitch != fb_tex && fb_pitch != fb_yaw,
          "pitch change samples different sky texels");
    printf("info fb_yaw90=0x%08x fb_pitchup=0x%08x\n", fb_yaw, fb_pitch);

    if (ppm_prefix) {
        const uint8_t *pal = hud_palette();
        char path[256];
        render(s_fb, &cam, have_terrain, have_scene);
        snprintf(path, sizeof path, "%s-textured.ppm", ppm_prefix);
        write_ppm(path, s_fb, pal);
        render(fb2, &cam_yaw, have_terrain, have_scene);
        snprintf(path, sizeof path, "%s-yaw90.ppm", ppm_prefix);
        write_ppm(path, fb2, pal);
        render(fb2, &cam_pitch, have_terrain, have_scene);
        snprintf(path, sizeof path, "%s-pitchup.ppm", ppm_prefix);
        write_ppm(path, fb2, pal);
    }

    /* --- phase 1.5: authored distant skyline (H-UAT-005) ---------------
     *
     * The decoded purchaser heightfield (miss8/P01.TER via the mission's
     * TDEF/ZMAP) carries P01's mesa relief -- validated against the
     * native recorder's car tape to 0.1 m along the drive line -- and the
     * reference views show that relief as a distant flat-topped mesa band
     * standing against the sky. This phase
     * proves the production filled world renderer draws that authored
     * skyline: it appears from representative cockpit/chase/vista
     * headings, it is DISTANT terrain (z recovered from the shared depth
     * buffer), it translates with camera yaw, and nearer terrain/scene
     * geometry occludes it (depth order, not paint order).
     *
     * Poses: A is the pinned representative P01 cockpit basis; B a
     * chase-style orbit pose over the same drive line; C the western
     * plain the reference opening vista looks across. The drive line
     * itself supplies the scene occluder: the authored p01pp01/nomilk3
     * cars sit on the road dead ahead of A. */
    struct {
        const char *name;
        double eye[3], tgt[3];
    } poses[] = {
        { "cockpit",  { 2810.04,  6.79, 47601.51 }, { 2809.64, 13.58, 47701.28 } },
        { "chase",    { 2810.04, 12.50, 47589.00 }, { 2810.04,  7.30, 47601.50 } },
        { "vista-n",  { 1100.00,  4.00, 48200.00 }, { 1100.00,  4.00, 48300.00 } },
        { "vista-e",  { 1100.00,  4.00, 48200.00 }, { 1200.00,  4.00, 48200.00 } },
        { "vista-y+", { 1100.00,  4.00, 48200.00 }, { 1150.00,  4.00, 48286.60 } },
        { "vista-y-", { 1100.00,  4.00, 48200.00 }, { 1050.00,  4.00, 48286.60 } },
        { "baseline", { 2710.00, 11.40, 47884.00 }, { 2710.00, 21.40, 47984.00 } },
    };
    enum { P_COCKPIT, P_CHASE, P_VISTA_N, P_VISTA_E, P_VISTA_YP, P_VISTA_YM,
           P_BASELINE };
    struct skyline_stat st[7];
    CameraView pcam[7];
    for (int p = 0; p < 7; p++) {
        if (camera_view_look_at(&pcam[p], poses[p].eye, poses[p].tgt) != 0) {
            fprintf(stderr, "sky_probe: pose %s camera invalid\n", poses[p].name);
            return 1;
        }
        render(fb3, &pcam[p], have_terrain, have_scene);
        const uint32_t *pdepth = worldrender_depth();
        if (!pdepth) {
            fprintf(stderr, "sky_probe: pose %s produced no depth\n", poses[p].name);
            return 1;
        }
        skyline_measure(&pcam[p], pdepth, &st[p]);
        printf("info skyline %-8s columns=%d z_min=%.0f z_med=%.0f geo=%ld\n",
               poses[p].name, st[p].columns, st[p].z_min, st[p].z_med,
               worldrender_geometry_pixels());
        if (p == P_VISTA_N) {
            memcpy(fb_vista, fb3, sizeof fb_vista);
            memcpy(depth_vista, pdepth, sizeof depth_vista);
        }
    }

    check(st[P_COCKPIT].columns >= 40,
          "cockpit heading (native tape pose): skyline above the horizon");
    check(st[P_CHASE].columns >= 20,
          "chase heading: skyline above the horizon");
    check(st[P_VISTA_N].columns >= 100 && st[P_VISTA_N].z_med >= 700.0,
          "vista heading: mesa band is DISTANT terrain (median z >= 700 m)");
    check(st[P_VISTA_E].columns >= 40 && st[P_VISTA_E].z_min <= 450.0,
          "vista east: near canyon wall occludes the far mesas (depth order)");

    double h_n[FB_W], h_p[FB_W], h_m[FB_W];
    skyline_heights(&pcam[P_VISTA_N], st[P_VISTA_N].top, h_n);
    skyline_heights(&pcam[P_VISTA_YP], st[P_VISTA_YP].top, h_p);
    skyline_heights(&pcam[P_VISTA_YM], st[P_VISTA_YM].top, h_m);
    /* Human-legible silhouette: not just "some column has depth above the
     * horizon" but a measurable rise in pixels (YouTube refs show a stepped
     * mesa band, not a 1 px hairline). Median over columns with any rise. */
    {
        double sum = 0.0;
        int n_pos = 0;
        double mx = 0.0;
        for (int x = 0; x < FB_W; x++) {
            if (h_n[x] <= 0.0) continue;
            sum += h_n[x];
            n_pos++;
            if (h_n[x] > mx) mx = h_n[x];
        }
        double med = 0.0;
        if (n_pos > 0) {
            /* crude median via partial sort of positives only */
            double *tmp = malloc((size_t)n_pos * sizeof *tmp);
            int k = 0;
            for (int x = 0; x < FB_W; x++)
                if (h_n[x] > 0.0) tmp[k++] = h_n[x];
            for (int i = 0; i < n_pos; i++)
                for (int j = i + 1; j < n_pos; j++)
                    if (tmp[j] < tmp[i]) {
                        double s = tmp[i]; tmp[i] = tmp[j]; tmp[j] = s;
                    }
            med = tmp[n_pos / 2];
            free(tmp);
        }
        printf("info skyline vista-n silhouette rise: cols=%d med=%.1fpx "
               "max=%.1fpx mean=%.1fpx\n",
               n_pos, med, mx, n_pos ? sum / n_pos : 0.0);
        check(n_pos >= 80 && med >= 6.0,
              "vista mesa silhouette is human-legible "
              "(>=80 cols, median rise >= 6 px above horizon)");
    }
    {
        const uint8_t *pal = hud_palette();
        struct skyline_contrast contrast;
        check(pal != NULL, "P01 palette available for skyline contrast");
        if (pal) {
            skyline_contrast_measure(&st[P_VISTA_N], fb_vista, depth_vista,
                                     pal, &contrast);
            printf("info skyline vista-n contrast: cols=%d darker=%d "
                   "mean-gap=%.1f luma\n",
                   contrast.columns, contrast.clearly_darker,
                   contrast.mean_gap);
            /* Native .lum remapping intentionally changes the old linear-RGB
             * contrast. Keep this as a visibility floor, not a pin to that
             * removed placeholder: at least 300 columns must be clearly
             * darker and the average luma gap must remain substantial. */
            check(contrast.columns >= 400 &&
                  contrast.clearly_darker >= 300 &&
                  contrast.mean_gap >= 16.0,
                  "distant P01 terrain is measurably darker than adjacent sky");
        }
    }
    int sh_p = skyline_shift(h_n, h_p);
    int sh_m = skyline_shift(h_n, h_m);
    printf("info skyline yaw shift +30deg=%dpx -30deg=%dpx\n", sh_p, sh_m);
    check(sh_p > 100 && sh_p < 270 && sh_m < -100 && sh_m > -270,
          "skyline translates with camera yaw (+/-30 deg yaw -> ~+/-185 px)");

    /* Scene occlusion: the authored drive line puts p01pp01 (a placed
     * cls-1 car 81 m ahead) between the cockpit eye and the far terrain.
     * Where its silhouette rises above the horizon, the silhouette-top
     * depth must be the CAR's z, not the terrain's behind it. Re-render
     * the cockpit pose (the pose loop left the baseline pose's buffer);
     * the phase-1 determinism check makes the reused top[] profile valid
     * against this identical re-render. */
    render(fb3, &pcam[P_COCKPIT], have_terrain, have_scene);
    const uint32_t *cdepth = worldrender_depth();
    int near_obj = 0;
    for (int x = 0; x < FB_W; x++) {
        if (st[P_COCKPIT].top[x] >= FB_H) continue;
        double z = depth_to_z(cdepth[(size_t)st[P_COCKPIT].top[x] * FB_W + x]);
        if (z < 120.0) near_obj++;
    }
    printf("info skyline cockpit near-object cols=%d\n", near_obj);
    check(near_obj >= 4,
          "near scene geometry silhouettes against the distant terrain "
          "(scene occludes skyline)");

    double var_drive = sky_row_variation(fb3, cdepth, 0, 120);
    printf("info row_variation_cockpit=%.2f\n", var_drive);
    check(var_drive > 0.5,
          "textured sky preserved at the gameplay poses");

    if (ppm_prefix) {
        const uint8_t *pal = hud_palette();
        char path[256];
        static const int dump[] = { P_COCKPIT, P_CHASE, P_VISTA_N };
        for (unsigned i = 0; i < sizeof dump / sizeof dump[0]; i++) {
            render(fb3, &pcam[dump[i]], have_terrain, have_scene);
            snprintf(path, sizeof path, "%s-skyline-%s.ppm", ppm_prefix,
                     poses[dump[i]].name);
            write_ppm(path, fb3, pal);
        }
    }

    /* --- phase 2: fallback + depth isolation ------------------------- */
    /* Read the real mission bytes now (this vfs is about to go away). */
    size_t msz = 0;
    uint8_t *mimg = vfs_read_file(mission, &msz);
    check(mimg != NULL && clear_sky_field(mimg, msz) == 0,
          "synthetic no-sky P01 image prepared (WRLD +82 zeroed)");

    char sandbox[1024];
    if (snprintf(sandbox, sizeof sandbox, "%s/sky-sandbox-XXXXXX", tmp_root)
        >= (int)sizeof sandbox) {
        fprintf(stderr, "sky_probe: I76_VERIFY_TMP path is too long\n");
        return 2;
    }
    if (!mkdtemp(sandbox)) { perror("mkdtemp"); return 1; }

    /* Mirror the asset root by symlink, EXCEPT miss8, which becomes a real
     * directory of per-file symlinks so the synthetic p01.msn can occupy
     * the exact path the engine resolves (terrain_load derives
     * miss8/p01.ter from the mission path). Purchaser data is never
     * modified — the sandbox only points at it. */
    DIR *d = opendir(root);
    if (d) {
        struct dirent *de;
        while ((de = readdir(d))) {
            if (de->d_name[0] == '.') continue;
            if (!strcasecmp(de->d_name, "miss8")) continue;
            char src[512], dst[512];
            snprintf(src, sizeof src, "%s/%s", root, de->d_name);
            snprintf(dst, sizeof dst, "%s/%s", sandbox, de->d_name);
            symlink(src, dst);
        }
        closedir(d);
    }
    char m8[512], real_m8[512];
    snprintf(m8, sizeof m8, "%s/miss8", sandbox);
    snprintf(real_m8, sizeof real_m8, "%s/miss8", root);
    mkdir(m8, 0755);
    d = opendir(real_m8);
    if (d) {
        struct dirent *de;
        while ((de = readdir(d))) {
            if (de->d_name[0] == '.') continue;
            if (!strcasecmp(de->d_name, "p01.msn")) continue;
            char src[512], dst[512];
            snprintf(src, sizeof src, "%s/%s", real_m8, de->d_name);
            snprintf(dst, sizeof dst, "%s/%s", m8, de->d_name);
            symlink(src, dst);
        }
        closedir(d);
    }
    char syn[512];
    snprintf(syn, sizeof syn, "%s/p01.msn", m8);
    FILE *sf = fopen(syn, "wb");
    if (sf && mimg) { fwrite(mimg, 1, msz, sf); fclose(sf); }
    if (mimg) vfs_free(mimg);

    vfs_shutdown();
    fs_set_root(sandbox);
    if (!vfs_init()) { fprintf(stderr, "sky_probe: sandbox vfs failed\n"); return 1; }

    int t2 = terrain_load(mission) == 0;
    int s2 = scene_load(mission) == 0;
    hud_load_mission(mission, 0);
    check(t2 && s2, "synthetic no-sky mission loads (terrain + scene)");

    check(scene_sky_name()[0] == '\0' && scene_sky_tex(NULL) == 0,
          "empty WRLD sky field exposes no sky tile");

    render(fb2, &cam, t2, s2);
    const uint32_t *depth_fb = worldrender_depth();
    check(depth_fb != NULL, "fallback render still produces depth");
    if (depth_fb) {
        uint32_t depth_grad = fnv1a((const uint8_t *)depth_fb,
                                    (size_t)FB_W * FB_H * sizeof(uint32_t));
        printf("info fallback fb=0x%08x depth=0x%08x\n",
               fnv1a(fb2, sizeof fb2), depth_grad);
        check(depth_grad == depth_tex,
              "missing-sky fallback leaves foreground depth BIT-IDENTICAL");
        double var_grad = sky_row_variation(fb2, depth_fb, 0, 120);
        printf("info row_variation_fallback=%.2f\n", var_grad);
        check(var_grad == 0.0, "missing-sky fallback draws the gradient "
                               "(constant rows)");
        check(fnv1a(fb2, sizeof fb2) != fb_tex,
              "fallback frame differs from textured frame in colour");
    }

    if (ppm_prefix) {
        const uint8_t *pal = hud_palette();
        char path[256];
        snprintf(path, sizeof path, "%s-fallback.ppm", ppm_prefix);
        write_ppm(path, fb2, pal);
    }

    printf("SKY_PROBE %s\n", g_fail ? "FAIL" : "PASS");
    return g_fail ? 1 : 0;
}
