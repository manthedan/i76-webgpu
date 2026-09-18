#ifndef RASTER_H
#define RASTER_H

/*
 * raster.h — filled software rasterizer (M8 V1).
 *
 * See docs/specs/m8/software-raster.md. The short version of the decisions
 * that are baked into this interface, because changing them later invalidates
 * every hash pin downstream:
 *
 *   - Camera space is +x right, +y up, +z forward, matching terrain.c's
 *     camspace() and scene.c's draw_one_mesh(). Screen mapping is
 *     sx = cx + f*x/z, sy = cy - f*y/z.
 *   - Triangles are clipped against the FULL frustum in camera space, before
 *     the perspective divide (spec §2). Not near-plane only. That is what
 *     bounds projected coordinates and makes fixed-point edge setup safe.
 *   - Coverage is decided by INTEGER edge functions with a top-left fill rule
 *     at 28.4 subpixel (spec §3). The motivation is watertightness first and
 *     determinism second: fan-triangulated n-gons share edges everywhere, and
 *     float edge evaluation eventually opens pinholes along shared edges.
 *   - Depth interpolates 1/z, NEVER z (spec §3). Stored as normalized
 *     reciprocal depth in a uint32_t: larger = nearer, cleared to zero
 *     (zero == infinitely far). Compare is strict `>`, so for a genuine tie
 *     the first triangle submitted wins and submission order must stay stable.
 *   - RVert carries u, v, light and fog FROM DAY ONE even though V1's fragment
 *     stage ignores u and v (spec §10). V1 is flat only at the fragment stage;
 *     the vertex pipeline, clipper and attribute interpolation are the final
 *     ones. This is what keeps V2 (textures) from being a rewrite.
 *
 * Shading is ramp-indexed — shade[base][level] — not an RGB round trip
 * (spec §4). The ramp index is the architecture; the ramp CONTENTS are now
 * reversed: nitro.exe loads the mission's .lum (WDEF/WRLD +43, scene.md §2)
 * as a 256x256 [light][texel] palette remap (docs/specs/re/phase-d-renderer.md
 * §3), and raster_set_shade_table() populates these ramps from it. The linear
 * RGB scaling remains as the bounded fallback for a mission whose WRLD names
 * no .lum. Translucency is likewise a native 256x256 byte LUT indexed
 * (texel << 8) | dest (phase-d §2, CONFIRMED disassembly), installed by
 * raster_set_translucency() and consumed by raster_polygon_translucent().
 */

#include <stdint.h>

/* Ramp lengths. Powers of two so level math is exact in fixed point. */
#define RASTER_SHADE_LEVELS 32
#define RASTER_FOG_LEVELS   32

/*
 * Projection constants, shared so terrain and objects cannot disagree.
 *
 * Binary-derived (nitro.exe FUN_00469dd0 / FUN_00469fb0): the full aperture
 * clamps to 90 degrees HORIZONTAL and the focal length comes from the
 * viewport WIDTH -- focal = (w * 0.5) / tan(45 deg) = w * 0.5. At the 4:3
 * framebuffer the vertical FOV is then 2*atan(3/4) ~= 73.74 degrees, not
 * the authored 60-degree vertical this port used before the oracle
 * measurement.
 *
 * The half-FOV tangent stays a LITERAL even though tan(45 deg) == 1 needs
 * no libm: it scales every projected coordinate, and a compile-time
 * constant keeps the native<->wasm equality gate independent of any
 * runtime evaluation.
 */
#define RASTER_FOV_TAN_HALF 1.0   /* tan(45 deg) => 90 deg HORIZONTAL FOV */
#define RASTER_ZNEAR        0.5
#define RASTER_ZFAR         3000.0

/* 28.4 subpixel. At 640x480 a screen coordinate is at most 640*16 = 10240,
 * so edge-function products stay far inside int64. */
#define RASTER_SUBPIXEL_BITS 4
#define RASTER_SUBPIXEL      (1 << RASTER_SUBPIXEL_BITS)

/*
 * Clipping a convex n-gon against 6 planes adds at most one vertex per plane,
 * so the bound is n+6. tools/geo_survey.c measured the widest face in the game
 * at 10 vertices (33035 faces across 2851 OEG records), which makes the real
 * worst case 16. 24 leaves deliberate headroom rather than sitting exactly on
 * a measured maximum that a Nitro-only asset could exceed.
 */
#define RASTER_CLIP_MAX 24

/*
 * The colour key: composite_scene_over_terrain() treats index 0 as
 * transparent, so a shaded surface that lands on 0 punches a hole straight
 * through the world (spec §4). It is excluded from every nearest-colour
 * search. This one is REAL, not assumed — the composite is in the tree.
 *
 * There is deliberately no reserved "sky band" constant here. The spec
 * suggested reserving 224–239; dumping the actual level palettes showed that
 * range is a grey ramp in N02 and an orange-to-dark ramp in N01, i.e. ordinary
 * shading colours. Reserving it excluded real colours from the world's palette
 * and bought nothing. The sky instead nearest-matches its own gradient like
 * any other colour (raster_sky_index).
 */
#define RASTER_KEY_INDEX   0

/*
 * THE OTHER TRANSPARENCY. Keep these two adjacent, because conflating them
 * punches holes through the world:
 *
 *   RASTER_KEY_INDEX         0     a transparent PIXEL in the framebuffer,
 *                                  tested by the HUD/cockpit composite.
 *   RASTER_TEXEL_TRANSPARENT 0xFF  the key TEXEL inside a keyed material;
 *                                  an ordinary palette index in unkeyed mode.
 *
 * They are different values in different spaces and neither implies the other.
 */
#define RASTER_TEXEL_TRANSPARENT 0xFFu

/*
 * UV range limit, in texels. Applied as a clamp before the float->int
 * conversion, and it is LOAD-BEARING rather than defensive: tools/tex_verify.c
 * measured u and v bottoming out at -1.69e38 in the shipped geometry (garbage
 * or uninitialised float data). Without the clamp that value is undefined
 * behaviour in C and a hard i32.trunc_f64_s trap in wasm.
 *
 * Note this is a CLAMP, not a wrap: 2^20 + 1.25 should wrap to texel 1 but
 * pins to texel 0. That is a deliberate malformed-input policy and must never
 * be described as wrapping.
 */
#define RASTER_UV_LIMIT 1048576.0   /* 2^20 */

/* Rows of sky gradient resolved at palette-build time. */
#define RASTER_SKY_LEVELS  32

typedef struct {
    double x, y, z;   /* camera space */
    float  u, v;      /* NORMALISED; scaled to texels at projection time */
    float  light;     /* 0..1 diffuse term */
    float  fog;       /* 0..1, 1 = fully fogged */
} RVert;

/*
 * A tile ready for sampling. Everything is precomputed so the inner loop
 * derives no invariants per polygon.
 *
 * w/h are powers of two, 8..256 — VERIFIED 4824/4824 by tools/tex_verify.c.
 * That invariant is what makes the mask wrap correct, so a tile that ever
 * fails it must be REJECTED at load rather than sampled.
 *
 * `texels` are LEVEL-palette indices, which is why no colour conversion exists
 * anywhere in this pipeline: the fragment stage is shade[level][texel].
 *
 * Note where u/v are scaled: at PROJECTION, not at the vertex. raster_clip_poly
 * runs first and interpolates the normalised u/v; project() then multiplies by
 * this tile's dimensions. So clip-generated vertices need no special handling,
 * and a vertex shared by two faces with different tiles is not a problem —
 * the clipped array is per-polygon scratch, not a shared mesh buffer.
 */
typedef struct {
    const uint8_t *texels;   /* w*h palette indices, row-major, row 0 = top */
    uint16_t w, h;
    uint16_t umask, vmask;   /* w-1, h-1 */
    uint8_t  vshift;         /* log2(w): index = (v << vshift) | u */
    uint8_t  has_key;        /* tile contains RASTER_TEXEL_TRANSPARENT */
} RTex;

/* Per-pixel bridge between native painter order and the port's z-buffer.
 * `key` retains native integer-metre ordering for later world-object ties.
 * Road-vs-terrain ownership itself uses continuous f32 view-Z: terrain_z is
 * the visible terrain fragment and road_z the nearest accepted road. */
typedef struct {
    uint32_t key;
    float terrain_z;
    float road_z;
} RPainterPixel;

#define RASTER_ROAD_TERRAIN_TOLERANCE_M 2.25f

typedef struct {
    uint8_t  *color;      /* w*h palette indices                       */
    uint32_t *depth;      /* w*h normalized reciprocal depth, 0 = far  */
    int       w, h;
    double    f, cx, cy;  /* focal length in pixels, screen centre     */
    double    znear, zfar;

    /* Precomputed from znear/zfar so the inner loop does no divides. */
    double    inv_znear, inv_zfar, depth_scale;

    /* Counters. Cheap, and they are what turns "it drew something" into a
     * statement about WHAT it drew — the coverage assertions in spec §8.3
     * need more than a hash. */
    long tris_in, tris_drawn, tris_clipped, tris_degenerate, tris_offscreen;
    long pixels_tested, pixels_written;

    /* V2 counters, APPENDED so no existing field's offset moves.
     * pixels_written keeps its asserted meaning: a discarded cut-out fragment
     * must NOT increment it (tools/raster_test.c depends on that). */
    long pixels_keyed;         /* cut-out fragments discarded: no colour, no depth */
    long texels_keyed_opaque;  /* 0xFF hit on a NON-cut-out face (see below)      */
    long faces_uv_clamped;     /* faces whose UVs hit RASTER_UV_LIMIT             */
    /* Fragments that SAMPLED a tile, discarded ones included -- a keyed cut-out
     * fragment did read a texel. So for a fully textured, depth-passing draw
     * pixels_textured == pixels_written + pixels_keyed, which tools/raster_test.c
     * asserts. Counting it after the key test instead would make the two
     * counters incomparable. */
    long pixels_textured;
    /* Fragments composed through the translucency LUT (source AND
     * destination dependent); a subset of pixels_textured. Lets a probe
     * prove the translucent path ran without hashing the framebuffer. */
    long pixels_translucent;
    /* Gate diagnostic: accepted textured fragments whose source palette entry
     * is low-saturation (<=15%) and final palette entry is yellow (HSV hue
     * 40..70, saturation >=25%, value >=40/255). It is framebuffer-observer
     * accounting only and never changes the selected index. */
    long pixels_grey_to_yellow;

    /* Native software painter-order bridge for terrain decals. The original
     * has no z-buffer: terrain and RDEF roads share a far-to-near polygon
     * queue, with roads submitted second. The port keeps its z-buffer for
     * general geometry and records continuous terrain/road fragment view-Z so
     * coarse LOD polygons cannot make roads camera-proximity dependent. Fields
     * are appended to preserve every existing offset above. */
    RPainterPixel *painter_order;  /* w*h painter/view-Z owners              */
    uint32_t  painter_key;         /* current polygon key; larger = nearer   */
    uint8_t   painter_overlay;     /* 1=road compatibility, 2=object ties   */
    long painter_pixels_rescued;   /* z rejected, native order accepted      */
    long painter_pixels_occluded;  /* z rejected, nearer native owner kept   */

    /* Optional gate-only road ownership diagnostics. `footprint_z` records
     * the last projected road fragment at each pixel before the ownership
     * test; `owner` records whether any road won that pixel. Production
     * targets leave both NULL, so no diagnostic buffers are touched. */
    float   *diag_road_footprint_z;
    uint8_t *diag_road_owner;

    /* Optional one-pixel provenance. These are current-draw labels only;
     * the bounded recorder lives in raster.c and is untouched while disabled.
     * Callers set them once per submitted primitive with
     * raster_pixel_history_draw(). */
    uint32_t history_draw_id;
    int32_t  history_primitive_id;
    const char *history_asset;
    const char *history_texture_asset;
    const char *history_layer;
    const char *history_pass;
} RTarget;

/* --- palette tables ---------------------------------------------------- */

/*
 * Build the shade/fog ramps and the nearest-colour cube from a 768-byte RGB
 * palette. Call once per mission (the level palette changes with the mission);
 * cheap enough to be unconditional, ~10 ms.
 *
 * Returns 0 on success, -1 if pal is NULL.
 */
int  raster_set_palette(const uint8_t pal[768]);
int  raster_palette_ready(void);

/*
 * Bumped by every successful raster_set_palette. Callers that cache anything
 * derived from the palette -- notably per-face base indices resolved once per
 * mesh -- compare against this to know when the cache went stale, which
 * happens whenever a mission with a different level palette loads.
 */
unsigned raster_palette_generation(void);

/* Nearest non-reserved palette index for a 24-bit authoring colour. */
uint8_t raster_rgb_to_index(uint8_t r, uint8_t g, uint8_t b);

/* shade[base][level]; level is clamped to [0, RASTER_SHADE_LEVELS-1].
 * level 0 is darkest, RASTER_SHADE_LEVELS-1 is the unshaded base colour. */
uint8_t raster_shade(uint8_t base, int level);

/* fog[index][level]; level 0 is unfogged. */
uint8_t raster_fog(uint8_t index, int level);

/*
 * Sky gradient: level 0 is the zenith, RASTER_SKY_LEVELS-1 the horizon haze.
 * The two endpoint colours are PLACEHOLDERS (a plausible high-desert sky), not
 * reversed values, and they are nearest-matched into whatever palette the
 * mission carries — so this adapts per mission instead of hardcoding indices
 * that meant something else.
 */
uint8_t raster_sky_index(int level);

/*
 * Fog is DESIGNED FOR but disabled by default (spec §5/§6): the fog curve and
 * the colour it converges on are unreversed, so shipping it on would be
 * inventing a look rather than reproducing one. The span write routes through
 * the step either way, so enabling it is not a pipeline change.
 */
void raster_set_fog_enabled(int on);
int  raster_fog_enabled(void);

/* Gate-only H-UAT-063 observer. Disabled by default; while enabled, filled
 * targets count accepted low-saturation-source -> yellow-final fragments in
 * pixels_grey_to_yellow. It never changes colour/depth output. */
void raster_grey_yellow_detector_enable(int on);

/* --- native mission tables (phase-d-renderer.md) ------------------------- */

/*
 * Install the mission's native luminance/shade table: the .lum named by the
 * mission WDEF/WRLD payload +43 (scene.md §2, CONFIRMED), loaded by
 * nitro.exe's FUN_004b45f0 into its shade LUT (blob+0x33 → DAT_00639b40;
 * phase-d §1/§3).
 *
 * The produced format (measured across all 35 shipped .lum files, both game
 * profiles): exactly 65536 bytes = 256 rows x 256 columns of palette
 * indices; row r is the full-palette remap for light level r. One row is the
 * IDENTITY remap (unshaded); it is AUTHORED per level (row 7 in 29 of 35
 * files; rows 4, 10, 16 and 22 in the exceptions), so it is detected, not
 * hardcoded. Rows below the identity row brighten (overbright), rows above
 * darken, and the dark end saturates at row 31 in every shipped file (rows
 * 32..255 repeat it). The 32-level ramp maps level RASTER_SHADE_LEVELS-1
 * ("unshaded", raster_shade's contract) to the identity row and level 0 to
 * the saturation row; the overbright rows are loaded by the original but are
 * not selectable here because RVert.light is clamped to [0,1] — widening
 * that is native-light-model work, not table work.
 *
 * lum == NULL restores the placeholder linear RGB ramps (the bounded
 * fallback for missions whose WRLD names no .lum, and for the synthetic
 * palettes of tools/raster_test.c, which ship no assets).
 *
 * Returns 0 on success, -1 when the table fails validation (no row with
 * >= 250/256 identity entries, or no dark range above it) — the current
 * ramps are then left untouched, so a corrupt asset can never half-apply.
 *
 * ORDERING: raster_set_palette() rebuilds the placeholder ramps, so the
 * shade table must be (re)installed AFTER any palette change. worldrender's
 * per-frame sync owns this ordering for the game paths.
 */
int  raster_set_shade_table(const uint8_t *lum /* [65536] */);
int  raster_shade_table_native(void);   /* 1 while a native .lum is installed */

/*
 * Install the mission's native translucency LUT: the .tbl named by WDEF/WRLD
 * payload +56, copied by FUN_004b45f0 into DAT_00619a20 (phase-d §1/§2).
 * 65536 byte entries; the CONFIRMED index (disassembly at 0x0047920d) is
 *
 *     out = table[(texel << 8) | dest]
 *
 * with texel the source palette index and dest the existing framebuffer
 * index. Measured properties of the shipped tables, which the translucent
 * drawer relies on: the diagonal is identity (blend(a,a)==a), the map is
 * NOT symmetric, and the 0xFF row is not a passthrough — so a 0xFF texel
 * must be discarded by the span (the cutout rule), never fed to the LUT.
 *
 * tbl == NULL disables translucency; translucent polygons then shade exactly
 * like opaque ones (the bounded fallback, since every miss8 mission names a
 * resolvable .tbl — measured 27/27 distinct pairs present in nitro.zfs).
 * The table is copied; the caller's buffer may be released immediately.
 */
int  raster_set_translucency(const uint8_t *tbl /* [65536] */);
int  raster_translucency_active(void);  /* 1 while a native .tbl is installed */

/* --- frame ------------------------------------------------------------- */

void raster_begin(RTarget *t, uint8_t *color, uint32_t *depth,
                  int w, int h, double focal, double znear, double zfar);
void raster_clear(RTarget *t, uint8_t bg);

/*
 * Clip a camera-space polygon against the full frustum. `in`/`out` may not
 * overlap. Returns the output vertex count (0 if fully clipped, never >
 * RASTER_CLIP_MAX).
 */
int  raster_clip_poly(const RTarget *t, const RVert *in, int n, RVert *out);

/*
 * Depth-buffer value for camera-space depth z on target t — the exact
 * quantization the triangle kernel writes (normalized reciprocal depth, 1 at
 * the near plane, 0 at/beyond far, NaN-safe). For callers that draw
 * point/impostor primitives directly into the target's color+depth buffers
 * and must depth-test against rasterized geometry with the kernel's own
 * "strict greater wins" rule. z at or behind the eye maps to 0 (never wins).
 */
uint32_t raster_depth_for_z(const RTarget *t, double z);

/*
 * Native painter-order compatibility for coplanar terrain overlays.
 *
 * nitro.exe's software path queues terrain first and RDEF roads second, bins
 * both by polygon camera depth, then paints far-to-near without a z-buffer
 * (FUN_00430890 -> FUN_004959d0/FUN_00494420 -> FUN_00498a70). The filled
 * port cannot disable depth for roads outright because a nearer mesa must
 * still hide a road. Instead, the terrain pass records each visible
 * fragment's continuous f32 view-Z in `order`; a road wins when its view-Z
 * is no more than RASTER_ROAD_TERRAIN_TOLERANCE_M behind that terrain. This
 * bounded PORT adaptation absorbs the port's coarse camera-centred LOD drift
 * without the absolute integer-metre threshold that caused H-UAT-053 temporal
 * flips. Accepted overlays write real, unbiased depth so later world geometry
 * tests against the road rather than stale terrain. Integer `key` and `sort_z`
 * remain only for exact later world-object ties, matching FUN_00494420. Pass
 * NULL to disable/reset.
 */
void raster_painter_terrain(RTarget *t, RPainterPixel *order, double sort_z);
void raster_painter_overlay(RTarget *t, RPainterPixel *order, double sort_z);
void raster_painter_objects(RTarget *t, RPainterPixel *order);
void raster_painter_disable(RTarget *t);

/*
 * render.explain_pixel — bounded software fragment provenance.
 *
 * This recorder is explicitly armed for one framebuffer pixel and otherwise
 * performs no allocation or per-pixel bookkeeping. A host calls enable once,
 * begins one frame around the exact software framebuffer it wants observed,
 * labels each submitted draw, and ends the frame before reading JSON. The JSON
 * is stable until the next completed frame. `top_k` is clamped to 1..64;
 * total_fragments and truncated report contests beyond the retained prefix.
 *
 * raster_pixel_history_note_overlay() covers non-raster software passes such
 * as sky/HUD/paper. raster_pixel_history_direct() covers depth-tested custom
 * primitives (vehicle impostors). Neither function writes the framebuffer.
 */
int  raster_pixel_history_enable(int x, int y, int top_k,
                                 const char *build_stamp,
                                 const char *run_provenance);
void raster_pixel_history_disable(void);
int  raster_pixel_history_active(void);
int  raster_pixel_history_target_index(const uint8_t *framebuffer,
                                       int w, int h);
void raster_pixel_history_frame_begin(uint8_t *framebuffer, int w, int h,
                                      uint64_t frame_id,
                                      const char *renderer);
void raster_pixel_history_frame_end(void);
const char *raster_pixel_history_json(void);

/* Programmatic view of the final fragment's palette chain. This is the same
 * winner serialized in JSON, not a second detector path. Returns 1 when the
 * winner sampled a palette-indexed texture and `out` was filled, else 0. */
typedef struct {
    uint8_t source_index, shade_level, shade_index, final_index, dest_index;
    uint8_t source_rgb[3], shade_rgb[3], final_rgb[3];
    int16_t lum_row;              /* -1 for the synthetic fallback */
    uint8_t shade_valid, final_valid, translucent, fogged;
    int32_t primitive_id;
    char asset[64], texture_asset[64], layer[32], pass[32];
} RasterPixelPalettePath;
int raster_pixel_history_winner_palette(RasterPixelPalettePath *out);

void raster_pixel_history_draw(RTarget *t, int primitive_id,
                               const char *asset, const char *layer,
                               const char *pass);
/* Optional sampled texture identity for the current draw; call immediately
 * after raster_pixel_history_draw(). Empty/NULL leaves texture_asset null. */
void raster_pixel_history_texture(RTarget *t, const char *texture_asset);
void raster_pixel_history_note_overlay(uint8_t *framebuffer, int w, int h,
                                       int primitive_id, const char *asset,
                                       const char *layer, const char *pass,
                                       const char *reason);
void raster_pixel_history_direct(RTarget *t, int primitive_id,
                                 const char *asset, const char *layer,
                                 const char *pass, int x, int y,
                                 double view_z, uint32_t depth,
                                 uint32_t prior_depth, int passed,
                                 const char *reason);

/*
 * Clip, project and fill one camera-space triangle with a flat ramp colour.
 * Accepts either winding — backface culling belongs to the caller, which is
 * the only place that knows face_flags (double-sided faces exist).
 */
void raster_triangle(RTarget *t, const RVert *a, const RVert *b,
                     const RVert *c, uint8_t base_index);

/*
 * Convex polygon convenience: clips once, then fans. Fanning AFTER the clip
 * (rather than clipping each fan triangle) avoids re-clipping shared interior
 * edges and is what keeps the shared edges watertight.
 */
void raster_polygon(RTarget *t, const RVert *v, int n, uint8_t base_index);

/*
 * The textured entry point. raster_polygon(t,v,n,b) is exactly
 * raster_polygon_tex(t,v,n,b,NULL,0), so the two signatures above stay
 * unchanged and every existing call site — 19 in tools/raster_test.c, plus
 * scene.c and terrain.c — compiles untouched. Changing them would have broken
 * the determinism goldens before it broke the game.
 *
 * `tex` NULL means flat: identical output to raster_polygon, byte for byte.
 *
 * `cutout` selects what a transparent texel does, and the two behaviours are
 * deliberately different:
 *   cutout != 0  discard the fragment entirely — no colour AND NO DEPTH.
 *                Writing depth would let an invisible texel occlude whatever
 *                is behind it, which is the whole failure a fence or chain-link
 *                surface would show.
 *   cutout == 0  shade 0xFF as an ordinary palette index and count it. The
 *                original's unkeyed mode maps every source byte; only keyed
 *                modes skip 0xFF (phase-d-renderer.md §3.1). Substituting the
 *                unrelated face-flat colour here caused H-UAT-063.
 */
void raster_polygon_tex(RTarget *t, const RVert *v, int n, uint8_t base_index,
                        const RTex *tex, int cutout);

/*
 * The TRANSLUCENT textured entry point: identical coverage, clipping, depth
 * and cutout semantics to raster_polygon_tex, but a sampled, non-keyed texel
 * is composed shade-then-blend (phase-d §2/§3):
 *
 *     frag = translucency[(shade_row[texel] << 8) | dest]
 *
 * where dest is the framebuffer index already at that pixel — output depends
 * on BOTH the source texel and the destination, which is the whole point of
 * the path. nitro.exe has both a pure translucent span drawer
 * (FUN_00478fd0: LUT indexed by the raw sampled texel) and a combined
 * shade+translucency one (FUN_00469680); the port ships the combined
 * composition as its one translucent drawer, per the confirmed contracts.
 *
 * Depth: tested and written exactly as the opaque path (strict `>`, first
 * submission wins ties). The native software depth treatment for translucent
 * spans is UNREVERSED; sharing the opaque rule is the least-invented option
 * in a single-depth-buffer architecture and keeps the cutout
 * submission-order guarantee intact.
 *
 * With no table installed (raster_translucency_active() == 0) output is
 * byte-identical to raster_polygon_tex — the bounded fallback above.
 *
 * tex == NULL is degenerate for a translucent material (there is no texel to
 * key the blend on) and draws exactly like raster_polygon.
 *
 * NOTE: which FACES select this path is Phase-D residual §7.4 — the native
 * material->drawer selection is UNREVERSED, so no engine caller is wired yet.
 * Open76's surfaceFlags2 5/7 mapping is Open76-only evidence and is
 * deliberately not adopted. This entry point exists for the confirmed LUT
 * contract and its probes.
 */
void raster_polygon_translucent(RTarget *t, const RVert *v, int n,
                                uint8_t base_index, const RTex *tex,
                                int cutout);

#endif /* RASTER_H */
