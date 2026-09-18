/*
 * pixel_history_probe.c — render.explain_pixel acceptance for H-UAT-053.
 *
 * Replays the diagnosis' exact synthetic ownership geometry at one pixel:
 * visible terrain at view-Z 10.51 m, then a conformed road at 11.81 m. Strict
 * reciprocal depth rejects the road, but the continuous 2.25 m road/terrain
 * tolerance admits it. The history must name both inputs and the road winner.
 *
 * Usage: OUT=/external/path tools/build_probe.sh pixel_history_probe &&
 *        /external/path/pixel_history_probe
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "engine/raster.h"

#define W 64
#define H 64
#define PX 20
#define PY 32

static uint8_t color[W * H];
static uint8_t road_texel[1] = { 70 };
static uint8_t key_texel[1] = { RASTER_TEXEL_TRANSPARENT };
static uint32_t depth[W * H];
static RPainterPixel owner[W * H];
static int checks, failures;

#define CHECK(name, cond) do { checks++; if (cond) printf("ok   %s\n", name); \
    else { printf("FAIL %s\n", name); failures++; } } while (0)

static uint32_t fnv1a(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) { h ^= p[i]; h *= 16777619u; }
    return h;
}

static RVert rv(double x, double y, double z)
{
    RVert v = { x, y, z, 0, 0, 1, 0 };
    return v;
}

static uint32_t render_case(int history, int top_k)
{
    RTarget t;
    memset(color, 0, sizeof color);
    memset(depth, 0, sizeof depth);
    memset(owner, 0, sizeof owner);
    if (history) {
        raster_pixel_history_enable(PX, PY, top_k, "probe-build",
                                    "H-UAT-053 synthetic continuous tolerance");
        raster_pixel_history_frame_begin(color, W, H, 1, "software");
    }
    raster_begin(&t, color, depth, W, H, 32.0, 0.5, 100.0);
    raster_clear(&t, 0);
    raster_pixel_history_note_overlay(color, W, H, 0,
                                      "software-sky-gradient", "sky", "world",
                                      "background_pass");

    RVert terrain[4] = { rv(-8,-8,10.51), rv(8,-8,10.51),
                         rv(8,8,10.51), rv(-8,8,10.51) };
    RVert road[4] = { rv(-8,-4,11.81), rv(0,-4,11.81),
                      rv(0,4,11.81), rv(-8,4,11.81) };
    raster_painter_terrain(&t, owner, 10.51);
    raster_pixel_history_draw(&t, 41042, "p02.ter", "terrain", "world");
    raster_polygon(&t, terrain, 4, 60);
    raster_painter_overlay(&t, owner, 11.81);
    raster_pixel_history_draw(&t, 70003, "r2ayr_51.map", "road", "world");
    raster_pixel_history_texture(&t, "r2ayr_51.vqm");
    RTex road_tex = { road_texel, 1, 1, 0, 0, 0, 0 };
    raster_polygon_tex(&t, road, 4, 70, &road_tex, 0);
    raster_painter_disable(&t);
    if (history) raster_pixel_history_frame_end();
    return fnv1a(color, sizeof color);
}

static void render_cutout_history(void)
{
    RTarget t;
    memset(color, 0, sizeof color);
    memset(depth, 0, sizeof depth);
    raster_pixel_history_enable(PX, PY, 64, "probe-build",
                                "H-UAT-063 discarded cutout truth");
    raster_pixel_history_frame_begin(color, W, H, 2, "software");
    raster_begin(&t, color, depth, W, H, 32.0, 0.5, 100.0);
    raster_clear(&t, 0);
    raster_pixel_history_note_overlay(color, W, H, 0,
                                      "software-sky-gradient", "sky", "world",
                                      "background_pass");
    RVert poly[4] = { rv(-8,-8,10), rv(8,-8,10),
                      rv(8,8,10), rv(-8,8,10) };
    RTex tex = { key_texel, 1, 1, 0, 0, 0, 1 };
    raster_pixel_history_draw(&t, 99, "fence.geo", "scene-object", "world");
    raster_pixel_history_texture(&t, "fence.vqm");
    raster_polygon_tex(&t, poly, 4, 10, &tex, 1);
    raster_pixel_history_frame_end();
}

int main(void)
{
    uint8_t palette[768];
    for (int i = 0; i < 256; i++)
        palette[i * 3 + 0] = palette[i * 3 + 1] = palette[i * 3 + 2] =
            (uint8_t)i;
    CHECK("palette initializes", raster_set_palette(palette) == 0);

    raster_pixel_history_disable();
    uint32_t off_before = render_case(0, 64);
    uint32_t on_hash = render_case(1, 64);
    const char *json = raster_pixel_history_json();
    CHECK("history is valid JSON-shaped output",
          json[0] == '{' && strstr(json, "\"verb\":\"render.explain_pixel\""));
    CHECK("terrain provenance is present",
          strstr(json, "\"asset\":\"p02.ter\"") != NULL);
    CHECK("road provenance is present",
          strstr(json, "\"asset\":\"r2ayr_51.map\"") != NULL);
    CHECK("continuous ownership inputs are present",
          strstr(json, "\"road_z\":11.8100004") != NULL &&
          strstr(json, "\"terrain_z\":10.5100002") != NULL &&
          strstr(json, "\"tolerance_m\":2.25") != NULL);
    CHECK("road wins by continuous tolerance",
          strstr(json, "\"layer\":\"road\",\"pass\":\"world\"}") != NULL &&
          strstr(json, "\"reason\":\"road_terrain_tolerance\","
                       "\"final_owner\":true") != NULL);
    CHECK("texture and palette-index chain are present",
          strstr(json, "\"texture_asset\":\"r2ayr_51.vqm\"") != NULL &&
          strstr(json, "\"source_kind\":\"texture_texel\","
                       "\"source_index\":70") != NULL &&
          strstr(json, "\"shade_applied\":true,\"shade_level\":31,"
                       "\"lum_row\":null,\"shade_index\":70") != NULL &&
          strstr(json, "\"final_index\":70,\"final_rgb\":[70,70,70]") != NULL);
    uint8_t replacement_palette[768];
    memset(replacement_palette, 1, sizeof replacement_palette);
    CHECK("replacement palette initializes",
          raster_set_palette(replacement_palette) == 0);
    RasterPixelPalettePath path;
    CHECK("programmatic winner preserves the captured palette chain",
          raster_pixel_history_winner_palette(&path) == 1 &&
          path.source_index == 70 && path.source_rgb[0] == 70 &&
          path.shade_valid && path.shade_index == 70 &&
          path.shade_rgb[0] == 70 && path.final_index == 70 &&
          path.final_rgb[0] == 70 && path.lum_row == -1 &&
          strcmp(path.texture_asset, "r2ayr_51.vqm") == 0);
    CHECK("original palette restores", raster_set_palette(palette) == 0);
    CHECK("enabled recorder is observer-only", on_hash == off_before);

    raster_pixel_history_disable();
    uint32_t off_after = render_case(0, 64);
    CHECK("off-mode framebuffer hash is stable",
          off_before == off_after && off_before == on_hash);

    render_cutout_history();
    const char *cutout = raster_pixel_history_json();
    CHECK("discarded cutout reports source but no fabricated shade stage",
          strstr(cutout, "\"reason\":\"transparent_cutout\"") != NULL &&
          strstr(cutout, "\"source_index\":255") != NULL &&
          strstr(cutout, "\"shade_applied\":false,\"shade_level\":null,"
                         "\"lum_row\":null,\"shade_index\":null,"
                         "\"shade_rgb\":null") != NULL);
    RasterPixelPalettePath cutout_path;
    CHECK("discarded cutout is not the final palette winner",
          raster_pixel_history_winner_palette(&cutout_path) == 0);

    (void)render_case(1, 2);
    const char *bounded = raster_pixel_history_json();
    CHECK("bounded top-k reports truncation",
          strstr(bounded, "\"top_k\":2") != NULL &&
          strstr(bounded, "\"total_fragments\":3") != NULL &&
          strstr(bounded, "\"returned\":2,\"truncated\":true") != NULL);
    raster_pixel_history_disable();

    printf("PIXEL_HISTORY_HASH off_before=0x%08x on=0x%08x off_after=0x%08x\n",
           off_before, on_hash, off_after);
    /* Emit the complete non-truncated acceptance artifact last. */
    (void)render_case(1, 64);
    printf("PIXEL_HISTORY_JSON %s\n", raster_pixel_history_json());
    raster_pixel_history_disable();
    printf("RESULT: %s pixel_history_probe (%d checks, %d failed)\n",
           failures ? "FAIL" : "PASS", checks, failures);
    return failures ? 1 : 0;
}
