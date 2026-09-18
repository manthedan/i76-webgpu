/*
 * frame_probe.c — native twin of webmain.c's web_render_fixed().
 *
 * Renders terrain + static scene from an EXPLICIT camera and prints the
 * FNV-1a of the raw 640x480 index buffer and, separately, of the 768-byte
 * level palette. tools/frame_gate.sh runs this and the wasm build over the
 * same missions and cameras and requires the hashes to match.
 *
 * Why this exists (docs/specs/m8/software-raster.md §8): the filled
 * rasterizer is about to be written, and the same C will compile to wasm
 * and x86-64. Two independent reviews both put this gate BEFORE the
 * rasterizer, on the grounds that an empirical cross-target equality check
 * is worth more than any argument about float precision. The wasm spec
 * permits nondeterministic NaN payloads, so a single 0/0 in a degenerate
 * face breaks bit-identity by specification rather than by luck — this is
 * how we would find out.
 *
 * Hash the RAW index buffer and the palette separately, never an encoded
 * image: a PNG hash also covers the encoder, and an encoder change would
 * read as a renderer regression.
 *
 * COVERAGE IS REPORTED AND GATED, not just the hash. A hash pin with no
 * coverage assertion happily locks in an all-background frame.
 *
 * This and web_render_fixed() call the SAME canonical-camera compositor,
 * engine/worldrender.c's worldrender_camera().
 *
 * Build (from the candidate root):
 *   OUT=/external/path tools/build_probe.sh frame_probe
 *
 * Usage: frame_probe <mission> [ex ey ez tx ty tz] [--ppm out.ppm]
 *        (asset root is required in $NITRO_APP)
 */
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "engine/fs.h"
#include "engine/vfs.h"
#include "engine/worldrender.h"
#include "engine/terrain.h"
#include "engine/scene.h"
#include "engine/hud.h"
#include "engine/raster.h"
#include "engine/texcache.h"

/* Must match webmain.c's MESHVIEW_FB_W/H. */
#define FB_W 640
#define FB_H 480

static uint8_t s_fb[FB_W * FB_H];

/*
 * --bench N: render the same frame N times and report per-frame cost, for
 * docs/specs/m8/software-raster.md §9. The spec's position is that a scalar
 * span writer is fast enough at 640x480 and that SIMD is not needed; that is
 * a claim about wall-clock, so it gets measured rather than asserted.
 *
 * Reports MEDIAN and MIN, not mean. The first frames resolve per-face palette
 * indices and fault in the mesh cache, and a mean folds that one-time cost
 * into the steady-state number. Min is the clean-cache floor; median is what
 * the frame actually costs.
 */
static int cmp_double(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static void bench_frames(int n, const CameraView *camera,
                         int have_terrain, int have_scene)
{
    double *ms = malloc((size_t)n * sizeof *ms);
    if (!ms) return;
    for (int i = 0; i < n; i++) {
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        worldrender_camera(s_fb, FB_W, FB_H, camera, 0.0, 0.0, have_terrain, have_scene);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        ms[i] = (t1.tv_sec - t0.tv_sec) * 1e3 +
                (t1.tv_nsec - t0.tv_nsec) / 1e6;
    }
    double first = ms[0];
    qsort(ms, (size_t)n, sizeof *ms, cmp_double);
    printf("BENCH backend=%s frames=%d first=%.2fms min=%.2fms median=%.2fms "
           "p95=%.2fms fps_median=%.1f\n",
           worldrender_backend_name(), n, first, ms[0], ms[n / 2],
           ms[(int)((n - 1) * 0.95)], 1000.0 / (ms[n / 2] > 0 ? ms[n / 2] : 1e-9));
    free(ms);
}

static uint32_t fnv1a(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr,
                "usage: frame_probe <mission> [ex ey ez tx ty tz] "
                "[--ppm out.ppm] [--filled] [--bench N]\n");
        return 2;
    }
    const char *mission = argv[1];
    const char *ppm = NULL;

    /* Default camera: a fixed pose over N01's spawn, looking north. Chosen
     * once and pinned — the gate compares builds, so the only requirement
     * is that both sides use the SAME camera. */
    double eye[3] = { 2997.5, 60.0, 48600.0 };
    double tgt[3] = { 2997.5, 30.0, 48900.0 };

    int a = 2;
    if (argc >= 8 && strcmp(argv[2], "--ppm") != 0) {
        for (int i = 0; i < 3; i++) eye[i] = atof(argv[2 + i]);
        for (int i = 0; i < 3; i++) tgt[i] = atof(argv[5 + i]);
        a = 8;
    }
    CameraView camera;
    if (camera_view_look_at(&camera, eye, tgt) != 0) {
        fprintf(stderr, "frame_probe: invalid camera\n");
        return 2;
    }
    int filled = 0, bench = 0;
    for (; a < argc; a++) {
        if (!strcmp(argv[a], "--ppm") && a + 1 < argc) ppm = argv[++a];
        else if (!strcmp(argv[a], "--filled")) filled = 1;
        else if (!strcmp(argv[a], "--bench") && a + 1 < argc) bench = atoi(argv[++a]);
    }

    const char *root = getenv("NITRO_APP");
    if (!root || !*root) {
        fprintf(stderr, "frame_probe: NITRO_APP is required\n");
        return 2;
    }
    fs_set_root(root);
    if (!vfs_init()) {
        fprintf(stderr, "frame_probe: vfs_init failed\n");
        return 1;
    }

    int have_terrain = terrain_load(mission) == 0;
    int have_scene = scene_load(mission) == 0;
    if (!have_terrain && !have_scene) {
        fprintf(stderr, "frame_probe: %s loaded neither terrain nor scene\n",
                mission);
        return 1;
    }
    /* webmain.c resolves the mission's level palette through hud_load_mission
     * on the same path that loads the scene. Without this hud_palette() is
     * NULL here and the palette hash would read 0 against the wasm build's
     * real hash — a mismatch that is the probe's fault, not the renderer's. */
    hud_load_mission(mission, 0);

    /*
     * Set the backend EXPLICITLY in both directions rather than only on
     * --filled. The engine default is now FILLED, so leaving the wire arm to
     * "whatever the default is" would quietly turn this gate's two arms into
     * the same arm and stop testing the wire path at all.
     */
    worldrender_set_backend(filled ? WORLD_BACKEND_FILLED : WORLD_BACKEND_WIRE);

    /* Same call web_render_fixed() makes. */
    worldrender_camera(s_fb, FB_W, FB_H, &camera, 0.0, 0.0, have_terrain, have_scene);

    if (bench > 0) {
        bench_frames(bench, &camera, have_terrain, have_scene);
        scene_unload();
        return 0;
    }

    int nonbg = 0;
    for (int i = 0; i < FB_W * FB_H; i++)
        if (s_fb[i] != 0) nonbg++;

    const uint8_t *pal = hud_palette();
    uint32_t pal_hash = pal ? fnv1a(pal, 768) : 0u;

    printf("FRAME backend=%s mission=%s fb_fnv1a=0x%08x pal_fnv1a=0x%08x nonbg=%d "
           "geo=%ld eye=%.1f,%.1f,%.1f tgt=%.1f,%.1f,%.1f\n",
           worldrender_backend_name(), mission, fnv1a(s_fb, sizeof s_fb), pal_hash, nonbg,
           filled ? worldrender_geometry_pixels() : (long)nonbg,
           eye[0], eye[1], eye[2], tgt[0], tgt[1], tgt[2]);

    long geo = filled ? worldrender_geometry_pixels() : (long)nonbg;
    if (filled) {
        /* Object counts, so "the frame is not empty" can be distinguished
         * from "the terrain is not empty" — objects are a small fraction of
         * coverage and a silent regression there would hide in the hash. */
        char s[160];
        scene_filled_stats(s, sizeof s);
        printf("SCENE %s geo_pixels=%ld\n", s, geo);
        /* M8 V2: every way a face can lose its texture, itemised. "textured=N"
         * above says how many kept one; this says why the rest did not, which
         * is the difference between an asset that is absent and a resolver
         * that is broken. */
        char tc[256];
        texcache_stats(tc, sizeof tc);
        printf("TEX %s\n", tc);

        /* Phase-D native mission tables (WDEF/WRLD +43/+56): the resolved
         * names, content hashes of the loaded 64 KiB payloads (proof the
         * native asset loaded, not a synthesized substitute), and whether
         * worldrender's sync installed them into the rasterizer. */
        const uint8_t *lum = scene_shade_table();
        const uint8_t *tbl = scene_translucency_table();
        printf("TABLES lum=%s tbl=%s lum_fnv1a=0x%08x tbl_fnv1a=0x%08x "
               "shade_native=%d transl_active=%d\n",
               scene_lum_name()[0] ? scene_lum_name() : "-",
               scene_tbl_name()[0] ? scene_tbl_name() : "-",
               lum ? fnv1a(lum, 256 * 256) : 0u,
               tbl ? fnv1a(tbl, 256 * 256) : 0u,
               raster_shade_table_native(),
               raster_translucency_active());
    }

    if (ppm) {
        FILE *f = fopen(ppm, "wb");
        if (!f) { perror(ppm); return 1; }
        fprintf(f, "P6\n%d %d\n255\n", FB_W, FB_H);
        for (int i = 0; i < FB_W * FB_H; i++) {
            uint8_t rgb[3] = { 0, 0, 0 };
            if (pal) memcpy(rgb, pal + s_fb[i] * 3, 3);
            fwrite(rgb, 1, 3, f);
        }
        fclose(f);
        fprintf(stderr, "wrote %s\n", ppm);
    }

    scene_unload();
    /* Mission asset lifetime: the native tables are owned by mission state
     * and must stop being handed out once the mission is gone. */
    printf("TABLES released_on_unload=%d\n",
           scene_shade_table() == NULL && scene_translucency_table() == NULL);
    /*
     * A frame that drew nothing must not be pinnable as "matching". For the
     * filled backend that means GEOMETRY pixels, not non-background colour:
     * the sky fills every pixel, so a colour-based floor would pass even if
     * the entire world failed to draw.
     */
    return geo > 0 ? 0 : 1;
}
