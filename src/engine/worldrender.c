/*
 * worldrender.c — world frame composition. See worldrender.h for why.
 *
 * Two backends live here. The wireframe path is the pre-M8 debug view, moved
 * across unchanged. The filled path (M8) draws sky, terrain and objects
 * through engine/raster.c into ONE colour buffer and ONE shared depth buffer,
 * because a filled world resolves occlusion by depth rather than by the
 * colour-key composite the wireframe path uses. If the filled path cannot run
 * (no palette yet, allocation failure) it falls back to the wireframe rather
 * than silently drawing nothing.
 */

#include "engine/worldrender.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "engine/hud.h"
#include "engine/raster.h"
#include "engine/scene.h"
#include "engine/terrain.h"

/*
 * FILLED is the default: this is what the game looks like. The wireframe is
 * the pre-M8 debug view and is now reachable only by asking for it
 * (worldrender_set_backend / web_set_backend / frame_probe without --filled).
 *
 * render_filled() still falls back to the wire path if it cannot run at all --
 * no palette resolved yet, or the depth allocation failed -- so a mission that
 * somehow arrives without a level palette draws a debug view rather than a
 * black screen.
 */
static WorldBackend s_backend = WORLD_BACKEND_FILLED;
static uint8_t      *s_scene_fb;
static size_t        s_scene_cap;
static uint32_t     *s_depth;
static size_t        s_depth_cap;
static RPainterPixel *s_painter_order;
static size_t        s_painter_order_cap;
static uint8_t       s_pal_seen[768];
static int           s_pal_seen_valid;
static long          s_geo_pixels;
static long          s_grey_yellow_pixels;
static int           s_depth_valid;

void worldrender_set_backend(WorldBackend b) { s_backend = b; }
WorldBackend worldrender_backend(void) { return s_backend; }

const char *worldrender_backend_name(void)
{
    switch (s_backend) {
    case WORLD_BACKEND_FILLED:      return "software-filled";
    case WORLD_BACKEND_FILLED_WIRE: return "software-filled+wire";
    default:                        return "wire-only";
    }
}

static uint8_t *scene_scratch(int w, int h)
{
    size_t need = (size_t)w * (size_t)h;
    if (s_scene_cap < need) {
        uint8_t *p = realloc(s_scene_fb, need);
        if (!p) return NULL;
        s_scene_fb = p;
        s_scene_cap = need;
    }
    return s_scene_fb;
}

/*
 * Both terrain and scene clear-on-render (standalone-tool semantics), so the
 * scene draws into scratch and is overlaid by colour key: palette index 0 is
 * the shared background and doubles as transparent.
 */
static void composite(uint8_t *color, const uint8_t *scene, int w, int h)
{
    int n = w * h;
    for (int i = 0; i < n; i++)
        if (scene[i] != 0) color[i] = scene[i];
}

static uint32_t *depth_buffer(int w, int h)
{
    size_t need = (size_t)w * (size_t)h;
    if (s_depth_cap < need) {
        uint32_t *p = realloc(s_depth, need * sizeof *p);
        if (!p) return NULL;
        s_depth = p;
        s_depth_cap = need;
    }
    return s_depth;
}

static RPainterPixel *painter_order_buffer(int w, int h)
{
    size_t need = (size_t)w * (size_t)h; /* painter/view-Z owners */
    if (s_painter_order_cap < need) {
        RPainterPixel *p = realloc(s_painter_order, need * sizeof *p);
        if (!p) return NULL;
        s_painter_order = p;
        s_painter_order_cap = need;
    }
    memset(s_painter_order, 0, need * sizeof *s_painter_order);
    return s_painter_order;
}

/*
 * Push the mission's level palette into the rasterizer when it changes.
 *
 * Compared by content rather than by pointer: hud_palette() can hand back the
 * same buffer refilled for a new mission, and a pointer check would then keep
 * shading against the previous mission's ramps. 768 bytes per frame is
 * nothing next to being subtly wrong.
 */
static void sync_palette(void)
{
    const uint8_t *pal = hud_palette();
    if (!pal) return;
    if (s_pal_seen_valid && memcmp(pal, s_pal_seen, sizeof s_pal_seen) == 0)
        return;
    memcpy(s_pal_seen, pal, sizeof s_pal_seen);
    s_pal_seen_valid = 1;
    raster_set_palette(pal);
}

/*
 * Push the mission's native shade/translucency tables into the rasterizer
 * when they change.
 *
 * Two generations have to agree before the push is skipped: scene's table
 * generation (a new mission may refill the same static buffers) and the
 * raster palette generation (raster_set_palette rebuilds the placeholder
 * ramps, so a palette swap MUST be followed by re-installing the native
 * .lum — the ordering raster.h documents). Pointers would not be enough:
 * scene.c's table storage is static, so its address never changes.
 *
 * NULL tables (mission names no asset, or the asset failed validation) are
 * pushed too: that is what restores the bounded placeholder fallback when a
 * table-carrying mission is followed by one without.
 */
static unsigned s_tab_seen_gen, s_tab_seen_palgen;

static void sync_tables(void)
{
    unsigned gen = scene_table_generation();
    unsigned pg  = raster_palette_generation();
    if (gen == s_tab_seen_gen && pg == s_tab_seen_palgen) return;
    s_tab_seen_gen = gen;
    s_tab_seen_palgen = pg;
    raster_set_shade_table(scene_shade_table());
    raster_set_translucency(scene_translucency_table());
}

/*
 * Sky: a vertical gradient across the reserved sky band, written with NO
 * depth (spec §5 puts it first in the composition order, below everything).
 *
 * The horizon is placed by projecting the camera's own pitch rather than
 * pinned to the screen centre, so the sky moves correctly as the camera looks
 * up and down instead of sliding as a fixed band.
 *
 * It fills the WHOLE frame, not just above the horizon. The terrain's draw
 * distance is far shorter than the real horizon, so stopping at the horizon
 * row leaves a black band between the last terrain quad and the sky. Painting
 * horizon haze below the horizon turns that band into distance haze, which is
 * what it should have looked like anyway.
 */
static void draw_sky_gradient(RTarget *t, const CameraView *camera)
{
    /*
     * The world horizon is the ray whose world-space Y component is zero.
     * Expressing world +Y in view coordinates makes the horizon respond to
     * pitch AND roll; a row-only gradient would silently erase scripted roll.
     */
    double yr = camera->right[1];
    double yu = camera->up[1];
    double yf = camera->forward[1];
    for (int y = 0; y < t->h; y++) {
        uint8_t *row = t->color + (size_t)y * t->w;
        for (int x = 0; x < t->w; x++) {
            double top_above = t->f * yf + ((double)x - t->cx) * yr +
                               t->cy * yu;
            double above = top_above - (double)y * yu;
            int level = RASTER_SKY_LEVELS - 1;
            if (above > 0.0 && top_above > 0.0) {
                /* Keep the original top-to-horizon ramp exactly when roll is
                 * zero; top_above simply varies by column for a rolled view. */
                level = (int)((top_above - above) *
                              (RASTER_SKY_LEVELS - 1) / top_above);
                if (level < 0) level = 0;
                if (level >= RASTER_SKY_LEVELS) level = RASTER_SKY_LEVELS - 1;
            }
            row[x] = raster_sky_index(level);
        }
    }
}

/*
 * Authored sky: the mission's WDEF/WRLD +82 .map, sampled by view ray
 * direction as an equirectangular dome: u from ray azimuth (wrapping),
 * v from ray elevation.
 *
 * Evidence state (native mapping UNREVERSED — no nitro.exe sky-code xref
 * exists as of 2026-08-09):
 *   - Native P01 captures (docs/evidence/m4-oracle-gameplay-p01.png) show
 *     THIS cloud texture across the sky, with wisps keeping their azimuth
 *     extent all the way down to the horizon.
 *   - The Open76 community reimplementation (Sky.cs/Sky.prefab — CONFIRMED
 *     as Open76 behaviour only) pins a textured plane above the camera. A
 *     ray-plane was evaluated here first: its 1/tan(el) singularity either
 *     moirés at the horizon (capture-calibrated scale) or inflates clouds
 *     past anything the captures show (horizon-calm scale). The dome keeps
 *     azimuth resolution at the horizon, which is what the captures show,
 *     so the dome is what ships. Open76's time-scrolling UV offset is NOT
 *     reproduced: nothing evidences it for nitro.exe and it would make
 *     frames time-dependent.
 *   - SKY_AZ_WRAPS / SKY_ROWS_PER_RAD / SKY_HORIZON_ROW are calibrated
 *     against the native captures (cloud angular size and horizon
 *     placement), not reversed. The texture is authored to tile — its bottom
 *     cloud band (rows ~116-127) flows into its top one (rows ~0-24) with a
 *     clear blue band between — so BOTH axes wrap; integer azimuth wraps
 *     keep the azimuth seam invisible.
 *
 * DETERMINISM: the native<->wasm framebuffer gate pins bit-identity, and
 * two libms may disagree by an ulp on atan2/asin, so NO libm
 * transcendentals are called here — sky_atan2/sky_asin below are fixed
 * evaluation-order polynomials over IEEE-exact ops (add, mul, div, sqrt)
 * only, identical on every target this compiles to.
 */

/* atan(t) for t in [-1, 1]: two half-angle reductions (each exact-op only)
 * shrink t into [-0.2, 0.2], then the Taylor series to t^13 in fixed Horner
 * order. Max error ~1e-11 rad — measured, not assumed, by sky_probe. */
static double sky_atan_reduced(double t)
{
    t = t / (1.0 + sqrt(1.0 + t * t));
    t = t / (1.0 + sqrt(1.0 + t * t));
    double t2 = t * t;
    double p = 1.0 / 13.0;
    p = p * t2 - 1.0 / 11.0;
    p = p * t2 + 1.0 / 9.0;
    p = p * t2 - 1.0 / 7.0;
    p = p * t2 + 1.0 / 5.0;
    p = p * t2 - 1.0 / 3.0;
    p = p * t2 + 1.0;
    return 4.0 * (t * p);
}

/* Pi to the last bit, as hex floats so every compiler rounds identically. */
#define SKY_PI      0x1.921fb54442d18p+1
#define SKY_HALF_PI 0x1.921fb54442d18p+0

/* atan2(y, x) over the full circle, deterministic. */
static double sky_atan2(double y, double x)
{
    if (x == 0.0)
        return y > 0.0 ? SKY_HALF_PI : (y < 0.0 ? -SKY_HALF_PI : 0.0);
    double r = y / x;
    if (r >= -1.0 && r <= 1.0) {
        double a = sky_atan_reduced(r);
        if (x < 0.0) a = y >= 0.0 ? a + SKY_PI : a - SKY_PI;
        return a;
    }
    /* |r| > 1: atan2(y,x) = sign(y)*pi/2 - atan(x/y), with |x/y| < 1. */
    double a = sky_atan_reduced(x / y);
    return (y > 0.0 ? SKY_HALF_PI : -SKY_HALF_PI) - a;
}

/* asin(z) = atan2(z, sqrt(1 - z^2)); z clamped first so a 1-ulp overshoot
 * of |dy|/|d| past 1.0 cannot reach a NaN. */
static double sky_asin(double z)
{
    if (z >= 1.0)  return SKY_HALF_PI;
    if (z <= -1.0) return -SKY_HALF_PI;
    return sky_atan2(z, sqrt(1.0 - z * z));
}

#define SKY_AZ_WRAPS     4.0   /* texture repeats per full turn (calibrated) */
#define SKY_ROWS_PER_RAD 256.0 /* texture rows per radian of elevation       */
#define SKY_HORIZON_ROW  64.0  /* texture row at zero elevation              */

static void draw_sky_tex(RTarget *t, const CameraView *camera, const RTex *sky)
{
    const double r[3] = { camera->right[0], camera->right[1], camera->right[2] };
    const double u[3] = { camera->up[0], camera->up[1], camera->up[2] };
    const double f[3] = { camera->forward[0], camera->forward[1],
                          camera->forward[2] };
    for (int y = 0; y < t->h; y++) {
        uint8_t *row = t->color + (size_t)y * t->w;
        double py = t->cy - (double)y;
        for (int x = 0; x < t->w; x++) {
            double px = (double)x - t->cx;
            double dx = t->f * f[0] + px * r[0] + py * u[0];
            double dy = t->f * f[1] + px * r[1] + py * u[1];
            double dz = t->f * f[2] + px * r[2] + py * u[2];
            double inv = 1.0 / sqrt(dx * dx + dy * dy + dz * dz);

            /* Both axes wrap (the tile is seamless by design); el is bounded
             * by +/-pi/2 so both casts sit a few hundred texels from 0. */
            double uu = (0.5 + sky_atan2(dx, dz) / (2.0 * SKY_PI) *
                         SKY_AZ_WRAPS) * sky->w;
            double vv = SKY_HORIZON_ROW - sky_asin(dy * inv) * SKY_ROWS_PER_RAD;
            row[x] = sky->texels[((int)floor(vv) & sky->vmask) << sky->vshift |
                                 ((int)floor(uu) & sky->umask)];
        }
    }
}

static void draw_sky(RTarget *t, const CameraView *camera)
{
    const RTex *sky = NULL;
    if (scene_sky_tex(&sky)) {
        draw_sky_tex(t, camera, sky);
        return;
    }
    /* Missing/corrupt/unresolvable sky asset: the placeholder gradient. */
    draw_sky_gradient(t, camera);
}

/*
 * The filled path. Terrain and objects share ONE depth buffer and one colour
 * buffer -- there is no colour-key composite here, because a filled world has
 * to resolve occlusion by depth. That sharing is the whole reason worldrender
 * exists.
 */
static int render_filled(uint8_t *color, int w, int h,
                         const CameraView *camera, double near, double far,
                         int have_terrain, int have_scene)
{
    uint32_t *depth = depth_buffer(w, h);
    RPainterPixel *painter_order = painter_order_buffer(w, h);
    if (!depth || !painter_order) return 0;

    sync_palette();
    if (!raster_palette_ready()) return 0;   /* no palette yet: caller falls back */
    sync_tables();

    RTarget t;
    double tan_half = camera_view_fov_tan_half(camera);
    raster_begin(&t, color, depth, w, h, (w * 0.5) / tan_half,
                 near > 0.0 ? near : RASTER_ZNEAR,
                 far > 0.0 ? far : RASTER_ZFAR);
    if (!t.color) return 0;
    raster_clear(&t, 0);

    draw_sky(&t, camera);
    raster_pixel_history_note_overlay(
        color, w, h, 0,
        scene_sky_name()[0] ? scene_sky_name() : "software-sky-gradient",
        "sky", "world", "background_pass");
    if (have_terrain) terrain_render_filled_order(&t, camera, painter_order);
    if (have_scene) {
        if (painter_order) raster_painter_objects(&t, painter_order);
        scene_render_filled(&t, camera);
        raster_painter_disable(&t);
    }

    /*
     * Pixels actually covered by GEOMETRY, counted from the depth buffer.
     *
     * The colour buffer can no longer answer this: the sky fills every pixel,
     * so "how many pixels are non-background" is now always w*h and a coverage
     * floor built on it can never fail. Depth is written only by the
     * rasterizer, so a frame that drew no world still reports zero here.
     */
    s_geo_pixels = 0;
    for (size_t i = 0, n = (size_t)w * (size_t)h; i < n; i++)
        if (depth[i]) s_geo_pixels++;
    s_grey_yellow_pixels = t.pixels_grey_to_yellow;
    return 1;
}

void worldrender_camera(uint8_t *color, int w, int h,
                        const CameraView *camera, double near, double far,
                        int have_terrain, int have_scene)
{
    s_geo_pixels = 0;
    s_grey_yellow_pixels = 0;
    s_depth_valid = 0;
    if (!color || w <= 0 || h <= 0 || !camera || !camera_view_valid(camera))
        return;

    if (s_backend != WORLD_BACKEND_WIRE &&
        render_filled(color, w, h, camera, near, far,
                      have_terrain, have_scene)) {
        s_depth_valid = 1;
        if (s_backend != WORLD_BACKEND_FILLED_WIRE) return;
        /* Filled+wire: overlay the wireframe so geometry and coverage can be
         * compared in one frame. The wire pass still writes through scratch. */
        uint8_t *scratch = scene_scratch(w, h);
        if (scratch && have_scene) {
            scene_render_camera(scratch, w, h, camera);
            composite(color, scratch, w, h);
        }
        return;
    }

    memset(color, 0, (size_t)w * (size_t)h);
    if (have_terrain)
        terrain_render_camera(color, w, h, camera, far);
    if (have_scene) {
        uint8_t *scratch = scene_scratch(w, h);
        if (scratch) {
            scene_render_camera(scratch, w, h, camera);
            composite(color, scratch, w, h);
        }
    }
}

/*
 * The orbit view stays WIREFRAME on purpose, even though the filled backend is
 * now the default everywhere else.
 *
 * This is the dev mission-inspection camera, and its distance defaults to the
 * whole map extent -- kilometres. The filled terrain draws to TERRAIN_FILL_FAR
 * (700 m), so a filled orbit frame would be sky plus a speck of ground in the
 * middle: strictly less useful than the wireframe, which shows the entire used
 * extent and its cell grid. Unifying the backends here would be consistency at
 * the cost of the only thing this view is for.
 *
 * If terrain LOD ever reaches map scale, revisit -- terrain.c already has the
 * camera derivation this would need (render_perspective).
 */
void worldrender_orbit(uint8_t *color, int w, int h, double yaw, double dist,
                        int have_terrain, int have_scene)
{
    s_geo_pixels = 0;
    s_grey_yellow_pixels = 0;
    s_depth_valid = 0;
    if (!color || w <= 0 || h <= 0) return;

    memset(color, 0, (size_t)w * (size_t)h);
    if (have_terrain)
        terrain_render(color, w, h, yaw, dist);
    if (have_scene) {
        uint8_t *scratch = scene_scratch(w, h);
        if (scratch) {
            scene_render(scratch, w, h, yaw, dist);
            composite(color, scratch, w, h);
        }
    }
}

long worldrender_geometry_pixels(void) { return s_geo_pixels; }
long worldrender_grey_to_yellow_pixels(void) { return s_grey_yellow_pixels; }
const uint32_t *worldrender_depth(void)
{
    return s_depth_valid ? s_depth : NULL;
}

void worldrender_shutdown(void)
{
    free(s_scene_fb);
    s_scene_fb = NULL;
    s_scene_cap = 0;
    free(s_depth);
    s_depth = NULL;
    s_depth_cap = 0;
    free(s_painter_order);
    s_painter_order = NULL;
    s_painter_order_cap = 0;
    s_pal_seen_valid = 0;
    s_tab_seen_gen = 0;
    s_tab_seen_palgen = 0;
    s_geo_pixels = 0;
    s_grey_yellow_pixels = 0;
    s_depth_valid = 0;
}
