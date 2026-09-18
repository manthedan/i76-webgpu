/*
 * road_depth_probe.c — H-UAT-025 road-vs-terrain painter/depth regression.
 *
 * Real P02 reproduction: two consecutive 20 Hz route poses (ticks 200/201 of
 * clown_probe --skip-intro) are rendered from the shipped chase camera. Before
 * the fix, strict shared depth makes terrain and the conformed RDEF ribbon
 * alternate ownership as the camera moves. The native software renderer does
 * not z-test those coplanar polygons: FUN_004959d0 queues terrain,
 * FUN_00494420 queues roads, and FUN_00498a70 paints them far-to-near.
 *
 * The fixed gate asserts both consecutive P02 frames exercise that painter
 * rescue and leave no nearer-terrain rejection on the visible race road. A
 * synthetic mesa case then puts a road behind nearer terrain and requires the
 * overlay policy to keep the mesa — preventing a tempting depth-test-disable
 * implementation from painting roads through hills.
 */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "engine/camera.h"
#include "engine/fs.h"
#include "engine/hud.h"
#include "engine/raster.h"
#include "engine/scene.h"
#include "engine/terrain.h"
#include "engine/vfs.h"
#include "engine/worldrender.h"

#define W 640
#define H 480

static uint8_t fb[W * H];
static uint8_t color[64 * 64];
static uint32_t depth[64 * 64];
static RPainterPixel order[64 * 64]; /* continuous terrain/road view-Z owners */
static int checks, failures;

#define CHECK(name, cond) do { checks++; if (cond) printf("ok   %s\n", name); \
    else { printf("FAIL %s\n", name); failures++; } } while (0)

static uint32_t fnv1a(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

static void chase_camera(CameraView *camera, double x, double y, double z,
                         double yaw)
{
    const double radius = 2.8 * 2.787733; /* shipped P02 Rampage COLP radius */
    const double el = 0.52359877559829887308;
    double eye[3] = {
        x - sin(yaw + 3.14159265358979323846) * radius * cos(el),
        y + radius * sin(el),
        z + cos(yaw + 3.14159265358979323846) * radius * cos(el)
    };
    double target[3] = { x, y + 2.0, z };
    double ground = terrain_height_at(eye[0], eye[2]);
    if (eye[1] < ground + 2.0) eye[1] = ground + 2.0;
    if (camera_view_look_at(camera, eye, target) != 0) {
        fprintf(stderr, "road_depth_probe: invalid chase camera\n");
        exit(2);
    }
}

static void p02_pair(void)
{
    /* Exact consecutive route poses captured from clown_probe --skip-intro. */
    static const double pose[2][4] = {
        { 3635.434924619, 2.400000095, 47714.524846092, 0.494651275928 },
        { 3634.604156296, 2.400000095, 47716.065081137, 0.494651275928 },
    };
    static const uint32_t expected_hash[2] = { 0xb775af20u, 0x35ece0c6u };
    for (int i = 0; i < 2; i++) {
        CameraView camera;
        chase_camera(&camera, pose[i][0], pose[i][1], pose[i][2], pose[i][3]);
        worldrender_camera(fb, W, H, &camera, 0, 0, 1, 1);
        long written = 0, rescued = 0, occluded = 0;
        terrain_road_render_stats(&written, &rescued, &occluded);
        uint32_t hash = fnv1a(fb, sizeof fb);
        printf("P02_FRAME tick=%d hash=0x%08x road=%ld rescued=%ld occluded=%ld\n",
               200 + i, hash, written, rescued, occluded);
        CHECK(i ? "P02 tick 201 pins spatial road ownership"
                : "P02 tick 200 pins spatial road ownership",
              hash == expected_hash[i]);
        CHECK(i ? "P02 tick 201 road pixels are nonzero"
                : "P02 tick 200 road pixels are nonzero", written > 1000);
        CHECK(i ? "P02 tick 201 resolves coplanar road by native order"
                : "P02 tick 200 resolves coplanar road by native order",
              rescued > 100);
        /* Every coplanar/equal-or-later painter rejection is counted as a
         * rescue, never left for quantized depth to alternate. Remaining
         * rejections are only nearer terrain owners (the occlusion contract)
         * and stay a small minority of the visible route. */
        CHECK(i ? "P02 tick 201 has no unresolved road/terrain ties"
                : "P02 tick 200 has no unresolved road/terrain ties",
              rescued > 100 && occluded < written);
    }
}

static RVert rv(double x, double y, double z)
{
    RVert v = { x, y, z, 0, 0, 1, 0 };
    return v;
}

static void near_relief_occludes_road(void)
{
    memset(color, 0, sizeof color);
    memset(depth, 0, sizeof depth);
    memset(order, 0, sizeof order);
    RTarget t;
    raster_begin(&t, color, depth, 64, 64, 32.0, 0.5, 100.0);
    raster_clear(&t, 0);

    /* The continuous 2.25 m view-Z tolerance is the bounded coarse-LOD
     * bridge; meaningfully farther relief remains an occluder. */
    RVert relief[4] = { rv(-6,-3,10.51), rv(6,-3,10.51),
                        rv(6,3,10.51), rv(-6,3,10.51) };
    RVert accepted[4] = { rv(-5,-2,11.81), rv(-0.3,-2,11.81),
                          rv(-0.3,2,11.81), rv(-5,2,11.81) };
    RVert rejected[4] = { rv(0.3,-2,12.81), rv(5,-2,12.81),
                          rv(5,2,12.81), rv(0.3,2,12.81) };
    RVert chained[4] = { rv(-5,-2,14.11), rv(-0.3,-2,14.11),
                         rv(-0.3,2,14.11), rv(-5,2,14.11) };
    raster_painter_terrain(&t, order, 10.51);
    raster_polygon(&t, relief, 4, 60);
    raster_painter_overlay(&t, order, 10.81);
    raster_polygon(&t, accepted, 4, 70);
    raster_painter_overlay(&t, order, 11.21);
    raster_polygon(&t, rejected, 4, 80);
    raster_painter_overlay(&t, order, 11.11);
    raster_polygon(&t, chained, 4, 90);
    raster_painter_disable(&t);
    int accepted_px = 0, rejected_px = 0, chained_px = 0;
    for (size_t i = 0; i < sizeof color; i++) {
        if (color[i] == 70) accepted_px++;
        if (color[i] == 80) rejected_px++;
        if (color[i] == 90) chained_px++;
    }
    CHECK("within-tolerance LOD drift is accepted", accepted_px > 0);
    CHECK("beyond-tolerance relief occludes road",
          rejected_px == 0 && t.painter_pixels_occluded > 0);
    CHECK("overlapping roads cannot jump beyond the terrain tolerance",
          chained_px == 0 && accepted_px > 0);
}

static void mesa_occludes_road(void)
{
    memset(color, 0, sizeof color);
    memset(depth, 0, sizeof depth);
    memset(order, 0, sizeof order);
    RTarget t;
    raster_begin(&t, color, depth, 64, 64, 32.0, 0.5, 100.0);
    raster_clear(&t, 0);

    RVert terrain[4] = { rv(-8,-8,20), rv(8,-8,20), rv(8,8,20), rv(-8,8,20) };
    RVert mesa[4] = { rv(-2,-4,10), rv(2,-4,10), rv(2,4,10), rv(-2,4,10) };
    RVert road[4] = { rv(-6,-2,20), rv(6,-2,20), rv(6,2,20), rv(-6,2,20) };

    raster_painter_terrain(&t, order, 20.0);
    raster_polygon(&t, terrain, 4, 10);
    raster_painter_terrain(&t, order, 10.0);
    raster_polygon(&t, mesa, 4, 20);
    raster_painter_overlay(&t, order, 20.0);
    raster_polygon(&t, road, 4, 30);
    raster_painter_disable(&t);

    int road_px = 0, mesa_px = 0;
    for (size_t i = 0; i < sizeof color; i++) {
        if (color[i] == 30) road_px++;
        if (color[i] == 20) mesa_px++;
    }
    printf("MESA road=%d mesa=%d rescued=%ld occluded=%ld\n",
           road_px, mesa_px, t.painter_pixels_rescued,
           t.painter_pixels_occluded);
    CHECK("coplanar road wins terrain painter tie", road_px > 0 &&
          t.painter_pixels_rescued > 0);
    CHECK("nearer mesa still occludes road", mesa_px > 0 &&
          t.painter_pixels_occluded > 0 && color[32 * 64 + 32] == 20);

    /* A later farther overlay must not overwrite an already accepted nearer
     * road at a ribbon intersection. This specifically pins painter_order's
     * overlay-owner update; retaining only the terrain key regresses it. */
    RVert farther[4] = { rv(-6,-2,30), rv(6,-2,30), rv(6,2,30), rv(-6,2,30) };
    long occluded0 = t.painter_pixels_occluded;
    raster_painter_overlay(&t, order, 30.0);
    raster_polygon(&t, farther, 4, 40);
    raster_painter_disable(&t);
    int road_after = 0, farther_px = 0;
    for (size_t i = 0; i < sizeof color; i++) {
        if (color[i] == 30) road_after++;
        if (color[i] == 40) farther_px++;
    }
    CHECK("nearer accepted road owns later farther overlap",
          road_after == road_px && farther_px == 0 &&
          t.painter_pixels_occluded > occluded0);

    RVert object_tie[4] = { rv(-5,-1,20), rv(-3,-1,20),
                            rv(-3,1,20), rv(-5,1,20) };
    raster_painter_objects(&t, order);
    raster_polygon(&t, object_tie, 4, 45);
    raster_painter_disable(&t);
    CHECK("later world polygon wins exact road depth tie",
          color[32 * 64 + 25] == 45);

    /* Polygon sort keys alone must not let a slanted road's near endpoint
     * pull its far fragments through nearer terrain. */
    RVert slanted[4] = { rv(-6,-2,9), rv(6,-2,30), rv(6,2,30), rv(-6,2,9) };
    occluded0 = t.painter_pixels_occluded;
    raster_painter_overlay(&t, order, 9.0);
    raster_polygon(&t, slanted, 4, 50);
    raster_painter_disable(&t);
    CHECK("slanted road cannot paint far pixels through terrain",
          color[32 * 64 + 42] != 50 &&
          t.painter_pixels_occluded > occluded0);
}

int main(int argc, char **argv)
{
    const char *root = argc > 1 ? argv[1] : getenv("NITRO_APP");
    if (!root || !*root) {
        fprintf(stderr, "usage: road_depth_probe <asset-root> (or set NITRO_APP)\n");
        return 2;
    }
    fs_set_root(root);
    CHECK("VFS initializes", vfs_init());
    CHECK("P02 terrain loads", terrain_load("miss8/P02.MSN") == 0);
    CHECK("P02 scene loads", scene_load("miss8/P02.MSN") == 0);
    CHECK("P02 palette loads", hud_load_mission("miss8/P02.MSN", 0) == 0);
    worldrender_set_backend(WORLD_BACKEND_FILLED);
    p02_pair();
    near_relief_occludes_road();
    mesa_occludes_road();
    scene_unload();
    terrain_unload();
    worldrender_shutdown();
    printf("road_depth_probe: %d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
