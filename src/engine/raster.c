/*
 * raster.c — filled software rasterizer (M8 V1). See raster.h for the
 * decisions baked into the interface, and docs/specs/m8/software-raster.md
 * for why they were made.
 *
 * Layout of this file:
 *   1. palette ramp tables      (spec §4)
 *   2. frustum clipping         (spec §2)
 *   3. projection + depth       (spec §3)
 *   4. the integer edge kernel  (spec §3)
 */

#include "engine/raster.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* =======================================================================
 * 1. Palette ramp tables
 *
 * Everything here is INTEGER arithmetic on purpose. The tables feed the
 * framebuffer directly, the framebuffer is hashed by tools/frame_gate.sh, and
 * that gate compares a wasm build against an x86-64 build. Building the tables
 * without touching a float removes the whole question from the gate.
 * ======================================================================= */

/* Sky endpoints. PLACEHOLDERS -- see raster.h. */
#define SKY_ZENITH_R   58
#define SKY_ZENITH_G  104
#define SKY_ZENITH_B  173
#define SKY_HORIZON_R 198
#define SKY_HORIZON_G 214
#define SKY_HORIZON_B 228

static uint8_t s_pal[768];
static uint8_t s_pal_low_sat[256], s_pal_yellow[256];
static int     s_pal_ready;
static unsigned s_pal_gen;
/*
 * TRANSPOSED to [level][base] (was [base][level]).
 *
 * The values are identical -- raster_shade() below still answers
 * shade(base,level) -- but a whole shading LEVEL is now contiguous, so the
 * textured fragment stage can hoist one row pointer per polygon and do a
 * single indexed load per pixel instead of a two-dimensional index.
 */
static uint8_t s_shade[RASTER_SHADE_LEVELS][256];
/* Source .lum row for each compressed port shade level; -1 marks the
 * synthetic RGB fallback, which has no authored row to report. */
static int16_t s_shade_source_row[RASTER_SHADE_LEVELS];
static uint8_t s_fogt[256][RASTER_FOG_LEVELS];
static uint8_t s_sky[RASTER_SKY_LEVELS];
static int     s_fog_on;               /* off by default; see raster.h    */
static int     s_grey_yellow_detector_on; /* explicit gate observer only   */

/*
 * Native mission tables (docs/specs/re/phase-d-renderer.md):
 *  - s_shade is REPOPULATED from the mission .lum by raster_set_shade_table;
 *    s_shade_native records which population is live so the fallback state
 *    is observable rather than implied.
 *  - s_transl is the 65536-entry translucency LUT from the mission .tbl,
 *    indexed (texel << 8) | dest. Both are per-mission data pushed in after
 *    every palette change; there is no per-pixel allocation anywhere below.
 */
static int     s_shade_native;
static uint8_t s_transl[256 * 256];
static int     s_transl_ready;

/* -----------------------------------------------------------------------
 * render.explain_pixel — one-pixel, bounded fragment provenance.
 *
 * The fixed storage exists with the rasterizer, but no framebuffer-sized
 * allocation and no fragment bookkeeping happens until explicitly armed.
 * The hot kernel only enters the recorder for the configured x/y. Keeping
 * this beside the ownership predicate is load-bearing: a post-frame depth
 * readback cannot explain whether a road won by strict depth, continuous
 * terrain tolerance, or a later native painter-key tie.
 * ----------------------------------------------------------------------- */
#define PIXEL_HISTORY_MAX       64
#define PIXEL_HISTORY_JSON_CAP  65536
#define PIXEL_HISTORY_NAME_CAP  64
#define PIXEL_HISTORY_REASON_CAP 40

typedef struct {
    uint32_t draw_id;
    int32_t primitive_id;
    char asset[PIXEL_HISTORY_NAME_CAP];
    char texture_asset[PIXEL_HISTORY_NAME_CAP];
    char layer[32];
    char pass[32];
    double view_z;
    uint32_t depth, prior_depth;
    uint32_t painter_key, prior_painter_key;
    float road_z, terrain_z, tolerance;
    uint8_t has_view_z, has_road_z, has_terrain_z, has_tolerance;
    uint8_t passed, final_owner;
    /* Palette-index chain for a sampled textured fragment. A rejected depth
     * fragment is deliberately chain-less because the hot path never samples
     * its texture; a transparent cutout has a source but no final index. */
    uint8_t palette_valid, palette_textured, palette_shade_valid;
    uint8_t palette_final_valid;
    uint8_t source_index, shade_level, shade_index, final_index, dest_index;
    uint8_t source_rgb[3], shade_rgb[3], final_rgb[3];
    uint8_t palette_translucent, palette_fogged;
    int16_t lum_row;
    char reason[PIXEL_HISTORY_REASON_CAP];
    char final_reason[PIXEL_HISTORY_REASON_CAP];
    uint32_t overwritten_by_draw_id;
} PixelHistoryEvent;

static struct {
    int enabled, frame_open, ready;
    int x, y, w, h, top_k;
    uint8_t *framebuffer;
    uint64_t frame_id;
    uint32_t draw_seq;
    uint32_t total_fragments;
    int count, winner_stored;
    char build[48], run[160], renderer[32];
    PixelHistoryEvent events[PIXEL_HISTORY_MAX];
    PixelHistoryEvent winner;
    int have_winner;
    char json[PIXEL_HISTORY_JSON_CAP];
} s_pixel_history;

static void ph_copy(char *dst, size_t cap, const char *src)
{
    if (!dst || cap == 0) return;
    if (!src) src = "";
    snprintf(dst, cap, "%s", src);
}

static int ph_target_matches(const uint8_t *fb, int w, int h)
{
    return s_pixel_history.enabled && s_pixel_history.frame_open &&
           fb == s_pixel_history.framebuffer && w == s_pixel_history.w &&
           h == s_pixel_history.h &&
           (unsigned)s_pixel_history.x < (unsigned)w &&
           (unsigned)s_pixel_history.y < (unsigned)h;
}

static void ph_record(const PixelHistoryEvent *src)
{
    PixelHistoryEvent e = *src;
    int slot = -1;
    e.final_owner = e.passed ? 1 : 0;
    ph_copy(e.final_reason, sizeof e.final_reason,
            e.passed ? "visible" : e.reason);

    if (e.passed && s_pixel_history.have_winner) {
        if (s_pixel_history.winner_stored >= 0) {
            PixelHistoryEvent *old =
                &s_pixel_history.events[s_pixel_history.winner_stored];
            old->final_owner = 0;
            ph_copy(old->final_reason, sizeof old->final_reason,
                    "overwrite_by_later_pass");
            old->overwritten_by_draw_id = e.draw_id;
        }
    }

    if (s_pixel_history.count < s_pixel_history.top_k) {
        slot = s_pixel_history.count++;
        s_pixel_history.events[slot] = e;
    }
    s_pixel_history.total_fragments++;

    if (e.passed) {
        s_pixel_history.winner = e;
        s_pixel_history.have_winner = 1;
        s_pixel_history.winner_stored = slot;
    }
}

static void ph_event_base(PixelHistoryEvent *e, uint32_t draw_id,
                          int primitive_id, const char *asset,
                          const char *layer, const char *pass)
{
    memset(e, 0, sizeof *e);
    e->draw_id = draw_id;
    e->primitive_id = primitive_id;
    ph_copy(e->asset, sizeof e->asset, asset);
    ph_copy(e->layer, sizeof e->layer, layer);
    ph_copy(e->pass, sizeof e->pass, pass);
}

int raster_pixel_history_enable(int x, int y, int top_k,
                                const char *build_stamp,
                                const char *run_provenance)
{
    if (x < 0 || y < 0) return -1;
    if (top_k < 1) top_k = 1;
    if (top_k > PIXEL_HISTORY_MAX) top_k = PIXEL_HISTORY_MAX;
    memset(&s_pixel_history, 0, sizeof s_pixel_history);
    s_pixel_history.enabled = 1;
    s_pixel_history.x = x;
    s_pixel_history.y = y;
    s_pixel_history.top_k = top_k;
    s_pixel_history.winner_stored = -1;
    ph_copy(s_pixel_history.build, sizeof s_pixel_history.build, build_stamp);
    ph_copy(s_pixel_history.run, sizeof s_pixel_history.run, run_provenance);
    return 0;
}

void raster_pixel_history_disable(void)
{
    s_pixel_history.enabled = 0;
    s_pixel_history.frame_open = 0;
}

int raster_pixel_history_active(void) { return s_pixel_history.enabled; }

int raster_pixel_history_target_index(const uint8_t *framebuffer, int w, int h)
{
    if (!ph_target_matches(framebuffer, w, h)) return -1;
    return s_pixel_history.y * w + s_pixel_history.x;
}

void raster_pixel_history_frame_begin(uint8_t *framebuffer, int w, int h,
                                      uint64_t frame_id,
                                      const char *renderer)
{
    if (!s_pixel_history.enabled || !framebuffer || w <= 0 || h <= 0 ||
        (unsigned)s_pixel_history.x >= (unsigned)w ||
        (unsigned)s_pixel_history.y >= (unsigned)h)
        return;
    s_pixel_history.framebuffer = framebuffer;
    s_pixel_history.w = w;
    s_pixel_history.h = h;
    s_pixel_history.frame_id = frame_id;
    s_pixel_history.draw_seq = 0;
    s_pixel_history.total_fragments = 0;
    s_pixel_history.count = 0;
    s_pixel_history.winner_stored = -1;
    s_pixel_history.have_winner = 0;
    s_pixel_history.ready = 0;
    s_pixel_history.json[0] = '\0';
    ph_copy(s_pixel_history.renderer, sizeof s_pixel_history.renderer, renderer);
    s_pixel_history.frame_open = 1;
}

void raster_pixel_history_draw(RTarget *t, int primitive_id,
                               const char *asset, const char *layer,
                               const char *pass)
{
    if (!t || !ph_target_matches(t->color, t->w, t->h)) return;
    t->history_draw_id = ++s_pixel_history.draw_seq;
    t->history_primitive_id = primitive_id;
    t->history_asset = asset;
    t->history_texture_asset = NULL;
    t->history_layer = layer;
    t->history_pass = pass;
}

void raster_pixel_history_texture(RTarget *t, const char *texture_asset)
{
    if (!t || !ph_target_matches(t->color, t->w, t->h)) return;
    t->history_texture_asset = texture_asset;
}

void raster_pixel_history_note_overlay(uint8_t *framebuffer, int w, int h,
                                       int primitive_id, const char *asset,
                                       const char *layer, const char *pass,
                                       const char *reason)
{
    if (!ph_target_matches(framebuffer, w, h)) return;
    PixelHistoryEvent e;
    ph_event_base(&e, ++s_pixel_history.draw_seq, primitive_id,
                  asset, layer, pass);
    e.passed = 1;
    ph_copy(e.reason, sizeof e.reason, reason ? reason : "later_pass");
    ph_record(&e);
}

void raster_pixel_history_direct(RTarget *t, int primitive_id,
                                 const char *asset, const char *layer,
                                 const char *pass, int x, int y,
                                 double view_z, uint32_t depth,
                                 uint32_t prior_depth, int passed,
                                 const char *reason)
{
    if (!t || !ph_target_matches(t->color, t->w, t->h) ||
        x != s_pixel_history.x || y != s_pixel_history.y) return;
    PixelHistoryEvent e;
    uint32_t draw_id = ++s_pixel_history.draw_seq;
    ph_event_base(&e, draw_id, primitive_id, asset, layer, pass);
    e.view_z = view_z;
    e.has_view_z = isfinite(view_z) ? 1 : 0;
    e.depth = depth;
    e.prior_depth = prior_depth;
    e.passed = passed ? 1 : 0;
    ph_copy(e.reason, sizeof e.reason, reason ? reason : "depth_test");
    ph_record(&e);
}

static void ph_fragment(RTarget *t, double view_z, uint32_t depth,
                        uint32_t prior_depth, uint32_t painter_key,
                        uint32_t prior_painter_key, float road_z,
                        float terrain_z, int has_tolerance, int passed,
                        const char *reason, int palette_valid,
                        int palette_textured, uint8_t source_index,
                        int palette_shade_valid, int shade_level,
                        uint8_t shade_index, int palette_final_valid,
                        uint8_t final_index,
                        uint8_t dest_index, int palette_translucent,
                        int palette_fogged)
{
    PixelHistoryEvent e;
    ph_event_base(&e, t->history_draw_id, t->history_primitive_id,
                  t->history_asset, t->history_layer, t->history_pass);
    ph_copy(e.texture_asset, sizeof e.texture_asset,
            t->history_texture_asset);
    e.view_z = view_z;
    e.has_view_z = isfinite(view_z) ? 1 : 0;
    e.depth = depth;
    e.prior_depth = prior_depth;
    e.painter_key = painter_key;
    e.prior_painter_key = prior_painter_key;
    e.road_z = road_z;
    e.terrain_z = terrain_z;
    e.has_road_z = road_z > 0.0f;
    e.has_terrain_z = terrain_z > 0.0f;
    e.has_tolerance = has_tolerance ? 1 : 0;
    e.tolerance = has_tolerance ? RASTER_ROAD_TERRAIN_TOLERANCE_M : 0.0f;
    e.passed = passed ? 1 : 0;
    e.palette_valid = palette_valid ? 1 : 0;
    e.palette_textured = palette_textured ? 1 : 0;
    e.palette_shade_valid = palette_shade_valid ? 1 : 0;
    e.palette_final_valid = palette_final_valid ? 1 : 0;
    e.source_index = source_index;
    e.shade_level = (uint8_t)shade_level;
    e.shade_index = shade_index;
    e.final_index = final_index;
    e.dest_index = dest_index;
    if (palette_valid)
        memcpy(e.source_rgb, s_pal + source_index * 3, 3);
    if (palette_valid && palette_shade_valid)
        memcpy(e.shade_rgb, s_pal + shade_index * 3, 3);
    if (palette_valid && palette_final_valid)
        memcpy(e.final_rgb, s_pal + final_index * 3, 3);
    e.palette_translucent = palette_translucent ? 1 : 0;
    e.palette_fogged = palette_fogged ? 1 : 0;
    e.lum_row = palette_valid && palette_shade_valid && shade_level >= 0 &&
                shade_level < RASTER_SHADE_LEVELS
              ? s_shade_source_row[shade_level] : -1;
    ph_copy(e.reason, sizeof e.reason, reason);
    ph_record(&e);
}

static void ph_json_append(size_t *used, const char *fmt, ...)
{
    if (*used >= sizeof s_pixel_history.json) return;
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_pixel_history.json + *used,
                      sizeof s_pixel_history.json - *used, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    size_t wrote = (size_t)n;
    if (wrote >= sizeof s_pixel_history.json - *used)
        *used = sizeof s_pixel_history.json - 1;
    else
        *used += wrote;
}

static void ph_json_string(size_t *used, const char *s)
{
    ph_json_append(used, "\"");
    for (const unsigned char *p = (const unsigned char *)(s ? s : "");
         *p && *used + 7 < sizeof s_pixel_history.json; p++) {
        switch (*p) {
        case '\\': ph_json_append(used, "\\\\"); break;
        case '"':  ph_json_append(used, "\\\""); break;
        case '\n': ph_json_append(used, "\\n"); break;
        case '\r': ph_json_append(used, "\\r"); break;
        case '\t': ph_json_append(used, "\\t"); break;
        default:
            if (*p < 0x20) ph_json_append(used, "\\u%04x", (unsigned)*p);
            else ph_json_append(used, "%c", *p);
            break;
        }
    }
    ph_json_append(used, "\"");
}

static void ph_json_nullable_float(size_t *used, int have, double value)
{
    if (have) ph_json_append(used, "%.9g", value);
    else ph_json_append(used, "null");
}

void raster_pixel_history_frame_end(void)
{
    if (!s_pixel_history.frame_open) return;
    s_pixel_history.frame_open = 0;
    size_t u = 0;
    ph_json_append(&u, "{\"verb\":\"render.explain_pixel\","
                   "\"schema\":\"i76-software-pixel-history-v1\","
                   "\"build\":{\"stamp\":");
    ph_json_string(&u, s_pixel_history.build);
    ph_json_append(&u, "},\"run\":{\"provenance\":");
    ph_json_string(&u, s_pixel_history.run);
    ph_json_append(&u, ",\"renderer\":");
    ph_json_string(&u, s_pixel_history.renderer);
    ph_json_append(&u, ",\"frame\":%llu},"
                   "\"pixel\":{\"x\":%d,\"y\":%d,\"width\":%d,\"height\":%d},"
                   "\"bounded\":{\"top_k\":%d,\"total_fragments\":%u,"
                   "\"returned\":%d,\"truncated\":%s,\"artifact\":null},"
                   "\"winner\":",
                   (unsigned long long)s_pixel_history.frame_id,
                   s_pixel_history.x, s_pixel_history.y,
                   s_pixel_history.w, s_pixel_history.h,
                   s_pixel_history.top_k, s_pixel_history.total_fragments,
                   s_pixel_history.count,
                   s_pixel_history.total_fragments > (uint32_t)s_pixel_history.count
                       ? "true" : "false");
    if (s_pixel_history.have_winner) {
        PixelHistoryEvent *e = &s_pixel_history.winner;
        ph_json_append(&u, "{\"draw_id\":%u,\"primitive_id\":%d,\"asset\":",
                       e->draw_id, e->primitive_id);
        ph_json_string(&u, e->asset);
        ph_json_append(&u, ",\"texture_asset\":");
        if (e->texture_asset[0]) ph_json_string(&u, e->texture_asset);
        else ph_json_append(&u, "null");
        ph_json_append(&u, ",\"layer\":"); ph_json_string(&u, e->layer);
        ph_json_append(&u, ",\"pass\":"); ph_json_string(&u, e->pass);
        ph_json_append(&u, "}");
    } else {
        ph_json_append(&u, "null");
    }
    ph_json_append(&u, ",\"fragments\":[");
    for (int i = 0; i < s_pixel_history.count; i++) {
        PixelHistoryEvent *e = &s_pixel_history.events[i];
        if (i) ph_json_append(&u, ",");
        ph_json_append(&u, "{\"order\":%d,\"draw_id\":%u,"
                       "\"primitive_id\":%d,\"asset\":",
                       i, e->draw_id, e->primitive_id);
        ph_json_string(&u, e->asset);
        ph_json_append(&u, ",\"texture_asset\":");
        if (e->texture_asset[0]) ph_json_string(&u, e->texture_asset);
        else ph_json_append(&u, "null");
        ph_json_append(&u, ",\"layer\":"); ph_json_string(&u, e->layer);
        ph_json_append(&u, ",\"pass\":"); ph_json_string(&u, e->pass);
        ph_json_append(&u, ",\"view_z\":");
        ph_json_nullable_float(&u, e->has_view_z, e->view_z);
        ph_json_append(&u, ",\"depth\":%u,\"prior_depth\":%u,"
                       "\"ownership\":{\"painter_key\":%u,"
                       "\"prior_painter_key\":%u,\"road_z\":",
                       e->depth, e->prior_depth,
                       e->painter_key, e->prior_painter_key);
        ph_json_nullable_float(&u, e->has_road_z, e->road_z);
        ph_json_append(&u, ",\"terrain_z\":");
        ph_json_nullable_float(&u, e->has_terrain_z, e->terrain_z);
        ph_json_append(&u, ",\"tolerance_m\":");
        ph_json_nullable_float(&u, e->has_tolerance, e->tolerance);
        ph_json_append(&u, "},\"palette\":");
        if (e->palette_valid) {
            ph_json_append(&u, "{\"source_kind\":");
            ph_json_string(&u, e->palette_textured ? "texture_texel" : "flat");
            ph_json_append(&u, ",\"source_index\":%u,\"source_rgb\":[%u,%u,%u],"
                           "\"shade_applied\":%s,\"shade_level\":",
                           e->source_index,
                           e->source_rgb[0], e->source_rgb[1],
                           e->source_rgb[2],
                           e->palette_shade_valid ? "true" : "false");
            if (e->palette_shade_valid)
                ph_json_append(&u, "%u", e->shade_level);
            else ph_json_append(&u, "null");
            ph_json_append(&u, ",\"lum_row\":");
            if (e->lum_row >= 0) ph_json_append(&u, "%d", e->lum_row);
            else ph_json_append(&u, "null");
            ph_json_append(&u, ",\"shade_index\":");
            if (e->palette_shade_valid) {
                ph_json_append(&u, "%u,\"shade_rgb\":[%u,%u,%u]",
                               e->shade_index,
                               e->shade_rgb[0], e->shade_rgb[1],
                               e->shade_rgb[2]);
            } else {
                ph_json_append(&u, "null,\"shade_rgb\":null");
            }
            ph_json_append(&u, ",\"dest_index\":%u,\"translucent\":%s,"
                           "\"fogged\":%s,\"final_index\":",
                           e->dest_index,
                           e->palette_translucent ? "true" : "false",
                           e->palette_fogged ? "true" : "false");
            if (e->palette_final_valid) {
                ph_json_append(&u, "%u,\"final_rgb\":[%u,%u,%u]",
                               e->final_index,
                               e->final_rgb[0], e->final_rgb[1],
                               e->final_rgb[2]);
            } else {
                ph_json_append(&u, "null,\"final_rgb\":null");
            }
            ph_json_append(&u, "}");
        } else {
            ph_json_append(&u, "null");
        }
        ph_json_append(&u, ",\"passed\":%s,\"reason\":",
                       e->passed ? "true" : "false");
        ph_json_string(&u, e->reason);
        ph_json_append(&u, ",\"final_owner\":%s,\"final_reason\":",
                       e->final_owner ? "true" : "false");
        ph_json_string(&u, e->final_reason);
        ph_json_append(&u, ",\"overwritten_by_draw_id\":");
        if (e->overwritten_by_draw_id)
            ph_json_append(&u, "%u", e->overwritten_by_draw_id);
        else
            ph_json_append(&u, "null");
        ph_json_append(&u, "}");
    }
    ph_json_append(&u, "]}");
    s_pixel_history.json[sizeof s_pixel_history.json - 1] = '\0';
    s_pixel_history.ready = 1;
}

const char *raster_pixel_history_json(void)
{
    return s_pixel_history.ready ? s_pixel_history.json : "";
}

int raster_pixel_history_winner_palette(RasterPixelPalettePath *out)
{
    if (!out || !s_pixel_history.ready || !s_pixel_history.have_winner ||
        !s_pixel_history.winner.palette_valid ||
        !s_pixel_history.winner.palette_textured)
        return 0;
    const PixelHistoryEvent *e = &s_pixel_history.winner;
    memset(out, 0, sizeof *out);
    out->source_index = e->source_index;
    out->shade_level = e->shade_level;
    out->shade_index = e->shade_index;
    out->final_index = e->final_index;
    out->dest_index = e->dest_index;
    memcpy(out->source_rgb, e->source_rgb, 3);
    if (e->palette_shade_valid)
        memcpy(out->shade_rgb, e->shade_rgb, 3);
    if (e->palette_final_valid)
        memcpy(out->final_rgb, e->final_rgb, 3);
    out->lum_row = e->lum_row;
    out->shade_valid = e->palette_shade_valid;
    out->final_valid = e->palette_final_valid;
    out->translucent = e->palette_translucent;
    out->fogged = e->palette_fogged;
    out->primitive_id = e->primitive_id;
    ph_copy(out->asset, sizeof out->asset, e->asset);
    ph_copy(out->texture_asset, sizeof out->texture_asset, e->texture_asset);
    ph_copy(out->layer, sizeof out->layer, e->layer);
    ph_copy(out->pass, sizeof out->pass, e->pass);
    return 1;
}

/* The one index a nearest-colour search must never return. See raster.h:
 * landing on the colour key punches a transparent hole through the world. */
static int reserved_index(int i)
{
    return i == RASTER_KEY_INDEX;
}

/*
 * Nearest non-reserved palette entry, plain squared-RGB distance.
 *
 * Ties resolve to the LOWEST index (the compare is strict `<`). That is
 * arbitrary but it must be *fixed*: several i76 palettes contain duplicate
 * entries, and an unstable tiebreak would make the framebuffer hash depend on
 * iteration order.
 */
static uint8_t nearest_index(int r, int g, int b)
{
    long best = -1;
    int  bi = 1;
    for (int i = 0; i < 256; i++) {
        if (reserved_index(i)) continue;
        long dr = r - s_pal[i * 3 + 0];
        long dg = g - s_pal[i * 3 + 1];
        long db = b - s_pal[i * 3 + 2];
        long d  = dr * dr + dg * dg + db * db;
        if (best < 0 || d < best) { best = d; bi = i; }
    }
    return (uint8_t)bi;
}

/*
 * Placeholder shade ramps from s_pal: shade[i][L-1] is the unshaded colour
 * and shade[i][0] is black; levels in between scale the base colour linearly.
 *
 * This is the FALLBACK ramp builder, used when a mission names no .lum (or
 * the named asset fails validation). Linear RGB scaling reproduces the shape
 * of a shading ramp without claiming to reproduce i76's; the native contents
 * come from raster_set_shade_table below. Replacing this loop does not touch
 * the kernel — and neither does the native path: both only write s_shade.
 */
static void build_placeholder_shade(void)
{
    for (int l = 0; l < RASTER_SHADE_LEVELS; l++)
        s_shade_source_row[l] = -1;
    for (int i = 0; i < 256; i++) {
        int r = s_pal[i * 3 + 0], g = s_pal[i * 3 + 1], b = s_pal[i * 3 + 2];
        for (int l = 0; l < RASTER_SHADE_LEVELS; l++) {
            int d = RASTER_SHADE_LEVELS - 1;
            s_shade[l][i] = nearest_index(r * l / d, g * l / d, b * l / d);
        }
    }
    s_shade_native = 0;
}

int raster_set_palette(const uint8_t pal[768])
{
    if (!pal) return -1;
    memcpy(s_pal, pal, 768);

    /* H-UAT-063 detector classes are precomputed once per mission palette so
     * the hot fragment path pays only two indexed byte tests. Hue uses the
     * ordinary HSV definition; black is low-saturation but never yellow. */
    for (int i = 0; i < 256; i++) {
        int r = s_pal[i * 3 + 0], g = s_pal[i * 3 + 1], b = s_pal[i * 3 + 2];
        int mx = r > g ? (r > b ? r : b) : (g > b ? g : b);
        int mn = r < g ? (r < b ? r : b) : (g < b ? g : b);
        int chroma = mx - mn;
        double hue = -1.0;
        if (chroma) {
            if (mx == r) hue = 60.0 * fmod((double)(g - b) / chroma, 6.0);
            else if (mx == g) hue = 60.0 * ((double)(b - r) / chroma + 2.0);
            else hue = 60.0 * ((double)(r - g) / chroma + 4.0);
            if (hue < 0.0) hue += 360.0;
        }
        s_pal_low_sat[i] = mx == 0 || chroma * 100 <= mx * 15;
        s_pal_yellow[i] = mx >= 40 && chroma * 100 >= mx * 25 &&
                          hue >= 40.0 && hue <= 70.0;
    }

    /* Palette change invalidates any installed .lum: back to the placeholder
     * until the caller pushes the new mission's table (raster.h ORDERING). */
    build_placeholder_shade();

    /*
     * Sky gradient, zenith -> horizon haze. PLACEHOLDER endpoint colours (a
     * plausible high-desert sky), nearest-matched into this mission's palette.
     * The spec's suggested fixed band 224-239 turned out to be ordinary
     * shading colours -- grey in N02, orange-to-dark in N01 -- so picking
     * indices by colour is both more honest and more portable across missions.
     */
    for (int l = 0; l < RASTER_SKY_LEVELS; l++) {
        int d = RASTER_SKY_LEVELS - 1;
        int ir = (SKY_ZENITH_R * (d - l) + SKY_HORIZON_R * l) / d;
        int ig = (SKY_ZENITH_G * (d - l) + SKY_HORIZON_G * l) / d;
        int ib = (SKY_ZENITH_B * (d - l) + SKY_HORIZON_B * l) / d;
        s_sky[l] = nearest_index(ir, ig, ib);
    }

    /*
     * Fog converges on the horizon haze, because fog in this game is where the
     * world meets the sky. Both the target colour and the curve are
     * UNREVERSED — which is exactly why fog ships disabled (§5/§6).
     */
    int fr = SKY_HORIZON_R, fg = SKY_HORIZON_G, fb = SKY_HORIZON_B;

    for (int i = 0; i < 256; i++) {
        int r = s_pal[i * 3 + 0], g = s_pal[i * 3 + 1], b = s_pal[i * 3 + 2];
        for (int l = 0; l < RASTER_FOG_LEVELS; l++) {
            int d = RASTER_FOG_LEVELS - 1;
            s_fogt[i][l] = nearest_index((r * (d - l) + fr * l) / d,
                                         (g * (d - l) + fg * l) / d,
                                         (b * (d - l) + fb * l) / d);
        }
    }

    s_pal_ready = 1;
    s_pal_gen++;
    return 0;
}

uint8_t raster_sky_index(int level)
{
    if (level < 0) level = 0;
    if (level > RASTER_SKY_LEVELS - 1) level = RASTER_SKY_LEVELS - 1;
    return s_sky[level];
}

/*
 * No dithered variant here on purpose. §6 of the spec decided dithering is OFF
 * for V1, to be revisited only with evidence from original captures. The sky
 * bands visibly (see the palette measurement in the spec) and dithering would
 * fix it, but "it looks better to me" is not the evidence that decision asked
 * for, and sky polish is not what the release path needs next.
 */

int raster_palette_ready(void) { return s_pal_ready; }
unsigned raster_palette_generation(void) { return s_pal_gen; }

/*
 * Exact nearest search, NOT a quantized cube lookup.
 *
 * The first draft used a 5:5:5 cube, which is the usual trick — but it is the
 * wrong trade here. face_rgb resolves to a palette index ONCE PER FACE at mesh
 * load (the same principle as triangulating at load, spec §2), never per
 * pixel, so there is no inner loop to accelerate. A cube would only buy the
 * right to be wrong: 5:5:5 rounds 100 to 98 before the search even starts, and
 * with 256 palette entries that is occasionally enough to pick a different
 * colour. Callers must resolve at load time and cache the result.
 */
uint8_t raster_rgb_to_index(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_pal_ready) return 0;
    return nearest_index(r, g, b);
}

uint8_t raster_shade(uint8_t base, int level)
{
    if (level < 0) level = 0;
    if (level > RASTER_SHADE_LEVELS - 1) level = RASTER_SHADE_LEVELS - 1;
    return s_shade[level][base];
}

uint8_t raster_fog(uint8_t index, int level)
{
    if (level < 0) level = 0;
    if (level > RASTER_FOG_LEVELS - 1) level = RASTER_FOG_LEVELS - 1;
    return s_fogt[index][level];
}

void raster_set_fog_enabled(int on) { s_fog_on = on ? 1 : 0; }
int  raster_fog_enabled(void)       { return s_fog_on; }
void raster_grey_yellow_detector_enable(int on)
{
    s_grey_yellow_detector_on = on ? 1 : 0;
}

/* Identity-remap rows in the shipped .lum files carry 254-255 of 256 exact
 * entries (duplicate palette entries account for the rest); 250 admits those
 * and nothing else plausible. */
#define LUM_IDENTITY_MIN 250

int raster_set_shade_table(const uint8_t *lum)
{
    if (!lum) {
        /* Back to the placeholder: the bounded no-asset fallback. */
        if (s_pal_ready) build_placeholder_shade();
        return 0;
    }

    /* The identity row (unshaded) is AUTHORED per level — row 7 in 29 of the
     * 35 shipped files, rows 4/10/16/22 in the exceptions — so it is found,
     * not assumed. Lowest row wins ties (adjacent near-identity rows happen
     * where the remap lands on duplicate palette entries). */
    int ident = -1, ident_row = -1;
    for (int r = 0; r < 256; r++) {
        int n = 0;
        for (int i = 0; i < 256; i++)
            if (lum[r * 256 + i] == i) n++;
        if (n > ident) { ident = n; ident_row = r; }
    }
    if (ident < LUM_IDENTITY_MIN) return -1;

    /* The dark end: the first row above the identity row that simply repeats
     * its successor is the saturation floor — row 31 in all 35 shipped files
     * (rows 32..255 are copies). No repetition found means the ramp runs the
     * full 256 rows. */
    int dark_row = 255;
    for (int r = ident_row + 1; r < 255; r++) {
        if (memcmp(lum + r * 256, lum + (r + 1) * 256, 256) == 0) {
            dark_row = r;
            break;
        }
    }
    if (dark_row <= ident_row) return -1;

    /* Map the port's 32 levels onto [ident_row, dark_row]: level L-1 is
     * "unshaded" (raster.h's contract) and lands exactly on the identity
     * row, level 0 on the saturation row. Integer floor keeps the mapping
     * monotonic and identical on every target. */
    for (int l = 0; l < RASTER_SHADE_LEVELS; l++) {
        int d = RASTER_SHADE_LEVELS - 1;
        int row = ident_row + ((d - l) * (dark_row - ident_row)) / d;
        memcpy(s_shade[l], lum + row * 256, 256);
        s_shade_source_row[l] = (int16_t)row;
    }
    s_shade_native = 1;
    return 0;
}

int raster_shade_table_native(void) { return s_shade_native; }

int raster_set_translucency(const uint8_t *tbl)
{
    if (!tbl) { s_transl_ready = 0; return 0; }
    memcpy(s_transl, tbl, sizeof s_transl);
    s_transl_ready = 1;
    return 0;
}

int raster_translucency_active(void) { return s_transl_ready; }

/* =======================================================================
 * 2. Frustum clipping — camera space, before the divide (spec §2)
 * ======================================================================= */

void raster_begin(RTarget *t, uint8_t *color, uint32_t *depth,
                  int w, int h, double focal, double znear, double zfar)
{
    memset(t, 0, sizeof *t);
    if (!(znear > 0.0) || !(zfar > znear)) return;   /* leaves color NULL */
    t->color = color;
    t->depth = depth;
    t->w = w;
    t->h = h;
    t->f = focal;
    t->cx = w * 0.5;
    t->cy = h * 0.5;
    t->znear = znear;
    t->zfar = zfar;
    t->inv_znear = 1.0 / znear;
    t->inv_zfar = 1.0 / zfar;
    t->depth_scale = 1.0 / (t->inv_znear - t->inv_zfar);
}

void raster_clear(RTarget *t, uint8_t bg)
{
    if (!t || !t->color || !t->depth) return;
    size_t n = (size_t)t->w * (size_t)t->h;
    memset(t->color, bg, n);
    memset(t->depth, 0, n * sizeof *t->depth);
}

/*
 * Signed distance to plane `p`, positive inside. The side planes are derived
 * from the SCREEN bounds rather than from a symmetric FOV, so a clipped
 * polygon projects exactly into [0,w] x [0,h]. That is what bounds the 28.4
 * fixed-point coordinates in the kernel.
 *
 *   sx = cx + f*x/z  in [0,w]  ->  f*x + cx*z >= 0  and  (w-cx)*z - f*x >= 0
 *   sy = cy - f*y/z  in [0,h]  ->  cy*z - f*y >= 0  and  (h-cy)*z + f*y >= 0
 *
 * (Both multiplied through by z, which is positive once past the near plane.)
 */
static double plane_dist(const RTarget *t, const RVert *v, int p)
{
    switch (p) {
    case 0:  return v->z - t->znear;
    case 1:  return t->zfar - v->z;
    case 2:  return t->f * v->x + t->cx * v->z;
    case 3:  return (t->w - t->cx) * v->z - t->f * v->x;
    case 4:  return t->cy * v->z - t->f * v->y;
    default: return (t->h - t->cy) * v->z + t->f * v->y;
    }
}

/* Interpolate EVERY attribute, including the ones V1's fragment stage
 * ignores. This is the "keeps V2 from being a rewrite" clause of spec §2. */
static void vlerp(RVert *o, const RVert *a, const RVert *b, double s)
{
    o->x = a->x + (b->x - a->x) * s;
    o->y = a->y + (b->y - a->y) * s;
    o->z = a->z + (b->z - a->z) * s;
    o->u = (float)(a->u + (b->u - a->u) * s);
    o->v = (float)(a->v + (b->v - a->v) * s);
    o->light = (float)(a->light + (b->light - a->light) * s);
    o->fog = (float)(a->fog + (b->fog - a->fog) * s);
}

int raster_clip_poly(const RTarget *t, const RVert *in, int n, RVert *out)
{
    RVert bufa[RASTER_CLIP_MAX], bufb[RASTER_CLIP_MAX];
    RVert *src = bufa, *dst = bufb;

    if (!t || !in || !out || n < 3 || n > RASTER_CLIP_MAX) return 0;
    memcpy(src, in, (size_t)n * sizeof *in);

    for (int p = 0; p < 6; p++) {
        int m = 0;
        for (int i = 0; i < n; i++) {
            const RVert *a = &src[i];
            const RVert *b = &src[(i + 1) % n];
            double da = plane_dist(t, a, p);
            double db = plane_dist(t, b, p);
            int ina = da >= 0.0, inb = db >= 0.0;

            if (ina && m < RASTER_CLIP_MAX) dst[m++] = *a;
            if (ina != inb) {
                double den = da - db;
                /* den cannot be zero when the two ends straddle the plane,
                 * but a NaN coordinate would reach here and produce a NaN
                 * parameter. The wasm spec leaves NaN payloads
                 * nondeterministic, so refusing the vertex is the only way to
                 * keep the cross-target gate meaningful. */
                if (den != 0.0 && m < RASTER_CLIP_MAX)
                    vlerp(&dst[m++], a, b, da / den);
            }
        }
        n = m;
        if (n < 3) return 0;
        { RVert *sw = src; src = dst; dst = sw; }
    }
    memcpy(out, src, (size_t)n * sizeof *out);
    return n;
}

/* =======================================================================
 * 3. Projection and depth
 * ======================================================================= */

typedef struct {
    int32_t x, y;   /* 28.4 screen coordinates */
    double  dn;     /* normalized reciprocal depth, 1 at near, 0 at far */
    /* u/z and v/z in TEXEL units. Screen-linear (which u,v alone are not), so
     * the existing barycentric lerp interpolates them exactly with no extra
     * setup. Zero when the polygon is untextured. */
    double  uoz, voz;
} RScreen;

static void project(const RTarget *t, const RVert *v, const RTex *tex,
                    RScreen *s)
{
    double invz = 1.0 / v->z;           /* z >= znear > 0, guaranteed by clip */
    double sx = t->cx + t->f * v->x * invz;
    double sy = t->cy - t->f * v->y * invz;
    s->x = (int32_t)lround(sx * RASTER_SUBPIXEL);
    s->y = (int32_t)lround(sy * RASTER_SUBPIXEL);
    double dn = (invz - t->inv_zfar) * t->depth_scale;
    s->dn = dn < 0.0 ? 0.0 : (dn > 1.0 ? 1.0 : dn);

    /* Scale normalised u,v to THIS tile's texels here rather than at the
     * vertex: the clipper has already run, so clip-generated vertices are
     * handled for free, and the two multiplies happen per vertex instead of
     * per fragment. */
    if (tex) {
        s->uoz = (double)v->u * (double)tex->w * invz;
        s->voz = (double)v->v * (double)tex->h * invz;
    } else {
        s->uoz = 0.0;
        s->voz = 0.0;
    }
}

/* The `!(dn > 0)` form is deliberate: it also catches NaN, which must map to
 * "infinitely far" rather than to an out-of-range conversion (UB in C, and a
 * nondeterministic payload in wasm). */
static uint32_t depth_quantize(double dn)
{
    if (!(dn > 0.0)) return 0u;
    if (dn >= 1.0) return 0xffffffffu;
    return (uint32_t)(dn * 4294967295.0);
}

uint32_t raster_depth_for_z(const RTarget *t, double z)
{
    if (!t || !(z > 0.0)) return 0u;    /* !(z>0) also catches NaN */
    double dn = (1.0 / z - t->inv_zfar) * t->depth_scale;
    return depth_quantize(dn < 0.0 ? 0.0 : (dn > 1.0 ? 1.0 : dn));
}

/* Nitro FUN_00498dc0 buckets `(sort_z - near)` by integer metres, then
 * FUN_00498a70 flushes bucket 4096 down to 0. Preserve that resolution: full
 * reciprocal-z keys would discard terrain-then-road submission order for two
 * polygons that native puts in the same bucket. Larger remains nearer. */
static uint32_t painter_key_for_z(const RTarget *t, double z)
{
    if (!t || isnan(z)) return 0;
    if (z <= t->znear) return 4097u;
    double rel = floor(z - t->znear);
    if (rel < 0.0) rel = 0.0;
    if (rel > 4095.0) rel = 4095.0;
    return 4096u - (uint32_t)rel; /* 0 remains the no-owner sentinel */
}

void raster_painter_terrain(RTarget *t, RPainterPixel *order, double sort_z)
{
    if (!t) return;
    t->painter_order = order;
    t->painter_key = painter_key_for_z(t, sort_z);
    t->painter_overlay = 0;
}

void raster_painter_overlay(RTarget *t, RPainterPixel *order, double sort_z)
{
    if (!t) return;
    t->painter_order = order;
    t->painter_key = painter_key_for_z(t, sort_z);
    t->painter_overlay = 1;
}

void raster_painter_objects(RTarget *t, RPainterPixel *order)
{
    if (!t) return;
    t->painter_order = order;
    t->painter_key = 0; /* each polygon derives its native minimum-z bucket */
    t->painter_overlay = 2;
}

void raster_painter_disable(RTarget *t)
{
    if (!t) return;
    t->painter_order = NULL;
    t->painter_key = 0;
    t->painter_overlay = 0;
    t->diag_road_footprint_z = NULL;
    t->diag_road_owner = NULL;
}

/* =======================================================================
 * 4. The kernel — integer edge functions, top-left fill rule (spec §3)
 *
 * Edge function, for the directed edge v_i -> v_{i+1}:
 *
 *   E_i(p) = (x_{i+1}-x_i)*(p.y - y_i) - (y_{i+1}-y_i)*(p.x - x_i)
 *
 * With the winding normalized so that area2 = E_0(v_2) > 0, a point is inside
 * iff every E_i >= 0. Screen y runs DOWNWARD, which is what fixes the sense of
 * the top-left rule below.
 *
 * Edge values are stepped incrementally by pure integer addition, so they are
 * bit-identical to direct evaluation — this is the canonical form, not an
 * optimization that trades away exactness.
 * ======================================================================= */

/*
 * With y down and area2 > 0: a horizontal edge running +x is the TOP edge of
 * the triangle, and any edge running -y is on its LEFT. Those are included;
 * every other boundary edge is excluded, so two triangles sharing an edge
 * cover each shared pixel exactly once.
 */
static int is_top_left(int64_t dx, int64_t dy)
{
    return (dy < 0) || (dy == 0 && dx > 0);
}

static int64_t imin3(int64_t a, int64_t b, int64_t c)
{
    int64_t m = a < b ? a : b;
    return m < c ? m : c;
}

static int64_t imax3(int64_t a, int64_t b, int64_t c)
{
    int64_t m = a > b ? a : b;
    return m > c ? m : c;
}

/*
 * ONE kernel body, three instantiations. `textured` and `translucent` are
 * compile-time constants in each, so the flat and opaque paths keep exactly
 * the instruction sequence they had -- the translucent branch folds away
 * entirely -- and there is only one copy of the coverage, fill-rule and
 * depth logic to keep watertight. `translucent` implies `textured`.
 */
static inline void fill_tri_body(RTarget *t, RScreen a, RScreen b, RScreen c,
                                 uint8_t flat, const RTex *tex, int cutout,
                                 const uint8_t *shade_row, int shade_level,
                                 int textured, int translucent)
{
    int64_t x0 = a.x, y0 = a.y, x1 = b.x, y1 = b.y, x2 = c.x, y2 = c.y;
    int64_t area2 = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);

    if (area2 == 0) { t->tris_degenerate++; return; }
    if (area2 < 0) {
        /* Accept either winding — backface culling is the caller's job, since
         * only it can read face_flags, and double-sided faces exist. */
        RScreen sw = b; b = c; c = sw;
        x1 = b.x; y1 = b.y; x2 = c.x; y2 = c.y;
        area2 = -area2;
    }

    /* Clipping bounded these to the viewport, but lround() at the edges can
     * land one subpixel outside, so clamp rather than trust. */
    int64_t lo_x = imin3(x0, x1, x2), hi_x = imax3(x0, x1, x2);
    int64_t lo_y = imin3(y0, y1, y2), hi_y = imax3(y0, y1, y2);
    if (lo_x < 0) lo_x = 0;
    if (lo_y < 0) lo_y = 0;

    int px0 = (int)(lo_x >> RASTER_SUBPIXEL_BITS);
    int py0 = (int)(lo_y >> RASTER_SUBPIXEL_BITS);
    int px1 = (int)(hi_x >> RASTER_SUBPIXEL_BITS);
    int py1 = (int)(hi_y >> RASTER_SUBPIXEL_BITS);
    if (px1 > t->w - 1) px1 = t->w - 1;
    if (py1 > t->h - 1) py1 = t->h - 1;
    if (px0 > px1 || py0 > py1) { t->tris_offscreen++; return; }

    int64_t bias0 = is_top_left(x1 - x0, y1 - y0) ? 0 : -1;
    int64_t bias1 = is_top_left(x2 - x1, y2 - y1) ? 0 : -1;
    int64_t bias2 = is_top_left(x0 - x2, y0 - y2) ? 0 : -1;

    /* Sample at pixel centres: (px + 0.5, py + 0.5) in 28.4. */
    int64_t sx0 = ((int64_t)px0 << RASTER_SUBPIXEL_BITS) + RASTER_SUBPIXEL / 2;
    int64_t sy0 = ((int64_t)py0 << RASTER_SUBPIXEL_BITS) + RASTER_SUBPIXEL / 2;

    int64_t e0row = (x1 - x0) * (sy0 - y0) - (y1 - y0) * (sx0 - x0);
    int64_t e1row = (x2 - x1) * (sy0 - y1) - (y2 - y1) * (sx0 - x1);
    int64_t e2row = (x0 - x2) * (sy0 - y2) - (y0 - y2) * (sx0 - x2);

    int64_t e0dx = -(y1 - y0) * RASTER_SUBPIXEL, e0dy = (x1 - x0) * RASTER_SUBPIXEL;
    int64_t e1dx = -(y2 - y1) * RASTER_SUBPIXEL, e1dy = (x2 - x1) * RASTER_SUBPIXEL;
    int64_t e2dx = -(y0 - y2) * RASTER_SUBPIXEL, e2dy = (x0 - x2) * RASTER_SUBPIXEL;

    double inv_area = 1.0 / (double)area2;
    t->tris_drawn++;

    for (int py = py0; py <= py1; py++) {
        int64_t e0 = e0row, e1 = e1row, e2 = e2row;
        uint8_t  *crow = t->color + (size_t)py * t->w;
        uint32_t *drow = t->depth + (size_t)py * t->w;

        for (int px = px0; px <= px1; px++) {
            t->pixels_tested++;
            if (((e0 + bias0) | (e1 + bias1) | (e2 + bias2)) >= 0) {
                /* Barycentrics from the UNBIASED edge values — the fill-rule
                 * bias decides coverage only, never interpolation.
                 * E_1 is zero on the edge opposite v0, so it weights v0. */
                double l0 = (double)e1 * inv_area;
                double l1 = (double)e2 * inv_area;
                double l2 = (double)e0 * inv_area;
                double dn = l0 * a.dn + l1 * b.dn + l2 * c.dn;

                uint32_t d = depth_quantize(dn);
                double frag_invz = dn / t->depth_scale + t->inv_zfar;
                double frag_z = frag_invz > 0.0 ? 1.0 / frag_invz : t->zfar;
                uint32_t frag_key = painter_key_for_z(t, frag_z);
                size_t pi = (size_t)py * (size_t)t->w + (size_t)px;
                int z_wins = d > drow[px];    /* strict: first submitted wins ties */
                RPainterPixel *owner = t->painter_order
                    ? &t->painter_order[pi] : NULL;
                int history_hit = t->history_draw_id != 0 &&
                    px == s_pixel_history.x && py == s_pixel_history.y &&
                    ph_target_matches(t->color, t->w, t->h);
                uint32_t prior_painter_key = owner ? owner->key : 0;
                float prior_terrain_z = owner ? owner->terrain_z : 0.0f;
                float event_road_z = t->painter_overlay == 1
                                   ? (float)frag_z
                                   : owner ? owner->road_z : 0.0f;
                uint32_t event_painter_key = t->painter_overlay == 2
                                           ? t->painter_key : frag_key;
                /* H-UAT-053: compare continuous f32 view-Z, exactly like the
                 * WebGPU shader. The old floor(z-near) buckets changed owner
                 * at every absolute metre boundary under tiny camera motion.
                 * A road may sit at most 2.25 m behind visible terrain; accepted
                 * roads then order strictly near-to-far among themselves. */
                int painter_wins = 0;
                if (!z_wins && t->painter_overlay && owner && owner->key != 0) {
                    if (t->painter_overlay == 1) {
                        /* Continuous tolerance is only road-vs-terrain.
                         * Once a road owns the pixel, ordinary strict depth
                         * resolves intersections; a later quantized tie cannot
                         * replace the first road with a farther fragment. */
                        if (!(owner->road_z > 0.0f)) {
                            float road_z = (float)frag_z;
                            painter_wins = !(owner->terrain_z > 0.0f) ||
                                road_z <= owner->terrain_z +
                                    RASTER_ROAD_TERRAIN_TOLERANCE_M;
                        }
                    } else if (t->painter_key >= owner->key) {
                        /* GPU meshes use less-equal too: carry only exact
                         * quantized native submission ties into world objects. */
                        painter_wins = d == drow[px];
                    }
                }
                if (t->painter_overlay == 1 && t->diag_road_footprint_z)
                    t->diag_road_footprint_z[pi] = (float)frag_z;
                if (z_wins || painter_wins) {
                    uint8_t frag = flat;
                    uint8_t palette_source = 0, palette_shade = flat;
                    uint8_t palette_dest = crow[px];
                    int palette_valid = 0, palette_translucent = 0;
                    int palette_fogged = 0;

                    if (textured) {
                        /* Perspective-correct, per pixel. The reciprocal is
                         * recovered with the fog path's EXACT expression below
                         * rather than a second algebraically-equal form, so a
                         * fogged textured fragment costs one divide, not two,
                         * and there is only one way to recover z in this file. */
                        double invz_t = dn / t->depth_scale + t->inv_zfar;
                        if (invz_t > 0.0) {
                            double zz = 1.0 / invz_t;
                            double uu = (l0 * a.uoz + l1 * b.uoz + l2 * c.uoz) * zz;
                            double vv = (l0 * a.voz + l1 * b.voz + l2 * c.voz) * zz;

                            /* Clamp FIRST: the shipped geometry contains UVs of
                             * -1.69e38, which is UB to convert in C and a trap
                             * in wasm. The !(x > y) form is NaN-safe. */
                            if (!(uu > -RASTER_UV_LIMIT))     uu = -RASTER_UV_LIMIT;
                            else if (uu > RASTER_UV_LIMIT)    uu =  RASTER_UV_LIMIT;
                            if (!(vv > -RASTER_UV_LIMIT))     vv = -RASTER_UV_LIMIT;
                            else if (vv > RASTER_UV_LIMIT)    vv =  RASTER_UV_LIMIT;

                            /* Exact floor: truncate, then correct. NO bias is
                             * added -- adding 2^20 would round away ~20 bits of
                             * mantissa and snap coordinates within ~6e-11 of an
                             * integer onto it, sampling the neighbouring texel.
                             * This does no arithmetic on the value at all. */
                            int32_t iu = (int32_t)uu; if ((double)iu > uu) --iu;
                            int32_t iv = (int32_t)vv; if ((double)iv > vv) --iv;

                            /* Unsigned conversion is defined modulo 2^32, so the
                             * wrap does not depend on signed-negative bits. */
                            uint32_t tu = (uint32_t)iu & tex->umask;
                            uint32_t tv = (uint32_t)iv & tex->vmask;
                            uint8_t texel = tex->texels[(tv << tex->vshift) | tu];
                            palette_valid = 1;
                            palette_source = texel;

                            /* Counted HERE, before the key test, because the
                             * name says "fragments that sampled a tile" and a
                             * discarded cut-out fragment did sample one. That
                             * also makes the invariant testable:
                             * pixels_textured == pixels_written + pixels_keyed
                             * for a fully textured, depth-passing draw. */
                            t->pixels_textured++;

                            if (tex->has_key &&
                                texel == RASTER_TEXEL_TRANSPARENT && cutout) {
                                /* Native keyed modes skip 0xFF; no colour AND
                                 * no depth means the world remains visible. */
                                t->pixels_keyed++;
                                if (history_hit)
                                    ph_fragment(t, frag_z, d, drow[px],
                                                event_painter_key,
                                                prior_painter_key,
                                                event_road_z,
                                                prior_terrain_z,
                                                t->painter_overlay == 1,
                                                0, "transparent_cutout",
                                                1, 1, texel, 0, shade_level,
                                                0, 0, 0, palette_dest, 0, 0);
                                e0 += e0dx; e1 += e1dx; e2 += e2dx;
                                continue;
                            }
                            /* Native unkeyed mode maps EVERY source byte,
                             * including 0xFF, through the shade/palette table.
                             * The old opaque special case kept `flat` here;
                             * on P12/N55 that replaced a black/grey index 255
                             * texel with the face's yellow flat colour. */
                            if (tex->has_key &&
                                texel == RASTER_TEXEL_TRANSPARENT)
                                t->texels_keyed_opaque++;
                            frag = shade_row[texel];
                            palette_shade = frag;
                            if (translucent && s_transl_ready) {
                                /* Shade THEN blend (phase-d §2/§3): the
                                 * CONFIRMED index is (texel << 8) | dest
                                 * with dest the framebuffer index already
                                 * here, so this read is what makes the
                                 * path destination-dependent. No z-fog
                                 * term, no dither — both NOT-FOUND on the
                                 * native software span path. */
                                frag = s_transl[((unsigned)frag << 8) |
                                                crow[px]];
                                palette_translucent = 1;
                                t->pixels_translucent++;
                            }
                        }
                    }

                    if (s_fog_on) {
                        /* Recover z from the interpolated reciprocal rather
                         * than interpolating a fog attribute: 1/z is linear in
                         * screen space and z is not, so this is exact where an
                         * affine fog attribute would only approximate.
                         * The CURVE is provisional — see raster_set_palette. */
                        double invz = dn / t->depth_scale + t->inv_zfar;
                        if (invz > 0.0) {
                            double z = 1.0 / invz;
                            double fl = z / t->zfar * (RASTER_FOG_LEVELS - 1);
                            frag = raster_fog(frag, (int)(fl + 0.5));
                            palette_fogged = 1;
                        }
                    }
                    if (s_grey_yellow_detector_on && palette_valid &&
                        s_pal_low_sat[palette_source] && s_pal_yellow[frag])
                        t->pixels_grey_to_yellow++;
                    if (history_hit) {
                        const char *reason = z_wins ? "depth_test"
                            : t->painter_overlay == 1
                                ? "road_terrain_tolerance"
                                : "painter_key";
                        ph_fragment(t, frag_z, d, drow[px],
                                    event_painter_key, prior_painter_key,
                                    event_road_z, prior_terrain_z,
                                    t->painter_overlay == 1, 1, reason,
                                    palette_valid, palette_valid,
                                    palette_source, palette_valid, shade_level,
                                    palette_shade, palette_valid, frag,
                                    palette_dest, palette_translucent,
                                    palette_fogged);
                    }
                    /* Accepted roads become the visible depth owner in the
                     * port's z-buffer. Native SW has no z-buffer, but its
                     * global far-to-near queue includes world objects too;
                     * retaining stale terrain depth here would let a later
                     * object behind the road overwrite it. Writing the road's
                     * real (unbiased) depth preserves that native ordering in
                     * the port while a nearer hill still rejects the road. */
                    if (z_wins || t->painter_overlay) drow[px] = d;
                    /* Accepted overlays become the painter owner too. RSEG
                     * quads arrive in source order rather than a pre-sorted
                     * road list; retaining only the old terrain key would let
                     * a later farther road overwrite an already-painted
                     * nearer road at intersections. */
                    if (owner) {
                        if (t->painter_overlay == 0) {
                            owner->key = frag_key;
                            owner->terrain_z = (float)frag_z;
                            owner->road_z = 0.0f;
                        } else if (t->painter_overlay == 1) {
                            owner->key = frag_key;
                            owner->road_z = (float)frag_z;
                            /* Retain the fixed terrain seed for later roads.
                             * Both owner-buffer allocators memset every frame,
                             * so terrain_z is zero here when no terrain won. */
                        } else {
                            owner->key = t->painter_key;
                        }
                    }
                    if (t->painter_overlay == 1 && t->diag_road_owner)
                        t->diag_road_owner[pi] = 1;
                    crow[px] = frag;
                    t->pixels_written++;
                    if (!z_wins) t->painter_pixels_rescued++;
                } else {
                    if (history_hit) {
                        const char *reason = "depth_test";
                        if (t->painter_overlay == 1 && owner) {
                            if (owner->road_z > 0.0f)
                                reason = "road_depth_test";
                            else if (owner->terrain_z > 0.0f &&
                                     (float)frag_z > owner->terrain_z +
                                         RASTER_ROAD_TERRAIN_TOLERANCE_M)
                                reason = "road_terrain_tolerance";
                        } else if (t->painter_overlay == 2 && owner &&
                                   t->painter_key < owner->key) {
                            reason = "painter_key";
                        }
                        ph_fragment(t, frag_z, d, drow[px],
                                    event_painter_key, prior_painter_key,
                                    event_road_z, prior_terrain_z,
                                    t->painter_overlay == 1, 0, reason,
                                    0, 0, 0, 0, shade_level, 0, 0, 0,
                                    0, 0, 0);
                    }
                    if (t->painter_overlay && drow[px] != 0)
                        t->painter_pixels_occluded++;
                }
            }
            e0 += e0dx; e1 += e1dx; e2 += e2dx;
        }
        e0row += e0dy; e1row += e1dy; e2row += e2dy;
    }
}

static void fill_tri(RTarget *t, RScreen a, RScreen b, RScreen c,
                     uint8_t flat, int shade_level)
{
    fill_tri_body(t, a, b, c, flat, NULL, 0, NULL, shade_level, 0, 0);
}

static void fill_tri_tex(RTarget *t, RScreen a, RScreen b, RScreen c,
                         uint8_t flat, const RTex *tex, int cutout,
                         const uint8_t *shade_row, int shade_level)
{
    fill_tri_body(t, a, b, c, flat, tex, cutout, shade_row, shade_level,
                  1, 0);
}

static void fill_tri_transl(RTarget *t, RScreen a, RScreen b, RScreen c,
                            uint8_t flat, const RTex *tex, int cutout,
                            const uint8_t *shade_row, int shade_level)
{
    fill_tri_body(t, a, b, c, flat, tex, cutout, shade_row, shade_level,
                  1, 1);
}

static int light_to_level(double light)
{
    int l = (int)(light * (RASTER_SHADE_LEVELS - 1) + 0.5);
    if (l < 0) return 0;
    if (l > RASTER_SHADE_LEVELS - 1) return RASTER_SHADE_LEVELS - 1;
    return l;
}

void raster_polygon(RTarget *t, const RVert *v, int n, uint8_t base_index)
{
    raster_polygon_tex(t, v, n, base_index, NULL, 0);
}

/* Shared body of raster_polygon_tex / raster_polygon_translucent. The only
 * difference is which textured kernel instantiation fans the polygon. */
static void poly_tex_impl(RTarget *t, const RVert *v, int n,
                          uint8_t base_index, const RTex *tex, int cutout,
                          int translucent)
{
    RVert   clipped[RASTER_CLIP_MAX];
    RScreen s[RASTER_CLIP_MAX];

    if (!t || !t->color || !t->depth || !v) return;
    /* Each of the 6 planes can add at most one vertex. Anything wider than
     * this should have been triangulated at LOAD time (spec §2). */
    if (n < 3 || n > RASTER_CLIP_MAX - 6) return;

    t->tris_in += n - 2;

    if (t->painter_overlay == 2) {
        double sort_z = v[0].z;
        for (int i = 1; i < n; i++)
            if (v[i].z < sort_z) sort_z = v[i].z;
        t->painter_key = painter_key_for_z(t, sort_z);
    }

    int m = raster_clip_poly(t, v, n, clipped);
    if (m < 3) { t->tris_offscreen += n - 2; return; }
    if (m != n) t->tris_clipped++;

    for (int i = 0; i < m; i++) project(t, &clipped[i], tex, &s[i]);

    /* V1 is flat AT THE FRAGMENT STAGE only (spec §10): one ramp lookup per
     * polygon. The ramp index itself is the final design; interpolating the
     * light level across the face is the V2 change, and it lands here. */
    int level = light_to_level(clipped[0].light);
    uint8_t flat = raster_shade(base_index, level);

    /* Fan AFTER clipping, so shared interior edges are never re-clipped and
     * the top-left rule can keep them watertight. */
    if (tex) {
        const uint8_t *shade_row = s_shade[level];   /* one row, one load/pixel */
        if (translucent) {
            for (int i = 1; i + 1 < m; i++)
                fill_tri_transl(t, s[0], s[i], s[i + 1], flat, tex, cutout,
                                shade_row, level);
        } else {
            for (int i = 1; i + 1 < m; i++)
                fill_tri_tex(t, s[0], s[i], s[i + 1], flat, tex, cutout,
                             shade_row, level);
        }
    } else {
        /* tex == NULL draws exactly like raster_polygon in BOTH materials:
         * there is no texel to key a blend on (raster.h). */
        for (int i = 1; i + 1 < m; i++)
            fill_tri(t, s[0], s[i], s[i + 1], flat, level);
    }
}

void raster_polygon_tex(RTarget *t, const RVert *v, int n, uint8_t base_index,
                        const RTex *tex, int cutout)
{
    poly_tex_impl(t, v, n, base_index, tex, cutout, 0);
}

void raster_polygon_translucent(RTarget *t, const RVert *v, int n,
                                uint8_t base_index, const RTex *tex,
                                int cutout)
{
    poly_tex_impl(t, v, n, base_index, tex, cutout, 1);
}

void raster_triangle(RTarget *t, const RVert *a, const RVert *b,
                     const RVert *c, uint8_t base_index)
{
    RVert in[3];
    in[0] = *a; in[1] = *b; in[2] = *c;
    raster_polygon(t, in, 3, base_index);
}
