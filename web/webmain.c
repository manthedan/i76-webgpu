/*
 * webmain.c — emscripten driver: browser build of the mesh viewer (M0),
 * the mission scene viewer (M2), and the drive mode (M3).
 *
 * The page (web/index.html) writes the user's game files into MEMFS at
 * /data/ (nitro.zfs + nitro.zix; loose mission tree miss8/... for scenes),
 * then calls web_init(). Everything after that goes through the real engine
 * path: vfs -> zfs -> meshcache -> geomesh_decode for meshes, the
 * terrain/scene modules for missions, car.c for the (placeholder) vehicle
 * sim, and input.c for the per-frame input snapshot. Rendering is the
 * shared 8-bit pipeline from meshview/terrain/scene, composited into a
 * static framebuffer the page blits to a <canvas> through the palette.
 *
 * Platform glue: this file implements the platform.h event/timing surface
 * over browser primitives (key events arrive from JS via web_key_event and
 * are drained by input.c's pump; ticks come from emscripten_get_now), plus
 * tiny stubs for the symbols meshview.c's SDL driver references but the web
 * build never calls (font.c is linked for real — HUD text needs it).
 */

#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <emscripten/emscripten.h>

#include "engine/fs.h"
#include "engine/vfs.h"
#include "engine/geomesh.h"
#include "engine/meshcache.h"
#include "engine/meshview.h"
#include "engine/worldrender.h"
#include "engine/pixidx.h"
#include "engine/terrain.h"
#include "engine/raster.h"
#include "engine/scene.h"
#include "engine/car.h"
#include "engine/component.h"
#include "engine/paint.h"
#include "engine/input.h"
#include "engine/font.h"
#include "engine/hud.h"
#include "engine/paperui.h"
#include "engine/mission.h"
#include "engine/combat.h"
#include "engine/ai.h"
#include "engine/sound.h"
#include "engine/m16.h"
#include "engine/texcache.h"
#include "platform/platform.h"
#include "render/render.h"

/* ------------------------------------------------------------------ */
/* Link stubs — meshview_run() is never called in the web build, but   */
/* the symbols it references must resolve. Intentionally minimal.      */
/* (input.c is compiled in for real; no input stubs here. font.c is   */
/* compiled in for real too since the M7 HUD text line — the M0 stubs  */
/* that used to live here made hud.c's font path silently no-op.)      */
/* ------------------------------------------------------------------ */

void platform_sleep(uint32_t ms) { (void)ms; }

bool render_init(void) { return true; }
void render_shutdown(void) {}
void render_begin_frame(void) {}
void render_end_frame(void) {}
void render_set_palette(const Rgb8 *palette_256) { (void)palette_256; }
void render_blit_indexed(const uint8_t *pixels, uint32_t width, uint32_t height)
{ (void)pixels; (void)width; (void)height; }

/* ------------------------------------------------------------------ */
/* platform.h implementation over browser primitives                   */
/* ------------------------------------------------------------------ */

/* Shell state machine (M5) — the full shell section is at the bottom of
 * this file; the state id + ESC latch live up here because
 * web_key_event (below) consults them, and the debrief stats because
 * web_drive_load/web_drive_step maintain them. */
enum {
    SHELL_MENU     = 0,
    SHELL_BRIEFING = 1,
    SHELL_DRIVE    = 2,
    SHELL_DEBRIEF  = 3,
    SHELL_CREDITS  = 4      /* terminal state after the base Trip's last win */
};
static int    s_shell = SHELL_MENU;
static int    s_esc_pending;
static double s_odom;                  /* meters driven this run */
static double s_top_speed;             /* m/s peak this run */

uint64_t platform_get_ticks(void)
{
    return (uint64_t)emscripten_get_now();   /* ms, monotonic */
}

/* Key events pushed from JS (web_key_event), drained by input.c's pump. */
#define WEB_EVT_CAP 64
static PlatformEvent s_evts[WEB_EVT_CAP];
static int s_evt_head, s_evt_tail;

EMSCRIPTEN_KEEPALIVE
void web_key_event(int pk, int down)   /* pk = PlatformKey value (platform.h) */
{
    /* Shell: ESC during SHELL_DRIVE latches the quit-to-debrief request,
     * consumed by web_shell_update() (the single drive->debrief
     * transition point). The event still feeds input.c below. */
    if (down && pk == PK_ESCAPE && s_shell == SHELL_DRIVE)
        s_esc_pending = 1;
    int next = (s_evt_tail + 1) % WEB_EVT_CAP;
    if (next == s_evt_head) return;    /* drop on overflow */
    s_evts[s_evt_tail].type = down ? PLATFORM_EVENT_KEY_DOWN
                                   : PLATFORM_EVENT_KEY_UP;
    s_evts[s_evt_tail].key  = (PlatformKey)pk;
    s_evt_tail = next;
}

bool platform_pump_events(PlatformEvent *evt)
{
    if (s_evt_head == s_evt_tail) {
        evt->type = PLATFORM_EVENT_NONE;
        evt->key  = PK_UNKNOWN;
        return true;                   /* browser never quits */
    }
    *evt = s_evts[s_evt_head];
    s_evt_head = (s_evt_head + 1) % WEB_EVT_CAP;
    return true;
}

/* ------------------------------------------------------------------ */
/* Mesh catalogue — geometry assets visible through the VFS.           */
/* Only g-tier .pak (decode to first OEG record) and .geo (single      */
/* record), matching the upstream viewer's ASSETS list policy.         */
/* ------------------------------------------------------------------ */

#define MAX_MESH_NAMES 4096
#define AUDIT_GDF_MAX  64
#define AUDIT_VCF_MAX  512

static char s_names[MAX_MESH_NAMES][16];
static int  s_name_count;
/* H-UAT-057 audit catalogues. These are VFS names only; purchaser bytes stay
 * in the user-supplied archive and are never written to a repository artifact. */
static char s_audit_gdf[AUDIT_GDF_MAX][16];
static char s_audit_vcf[AUDIT_VCF_MAX][16];
static int  s_audit_gdf_count, s_audit_vcf_count;
static int  s_audit_catalog_overflow;

static int has_geometry_ext(const char *name)
{
    size_t n = strlen(name);
    if (n < 5) return 0;
    return strcmp(name + n - 5, "g.pak") == 0 ||
           strcmp(name + n - 4, ".geo") == 0;
}

static void collect_cb(const char *name, int src_type, void *ud)
{
    (void)src_type; (void)ud;
    size_t n = strlen(name);
    if (n >= 4 && strcmp(name + n - 4, ".gdf") == 0) {
        if (s_audit_gdf_count < AUDIT_GDF_MAX) {
            snprintf(s_audit_gdf[s_audit_gdf_count],
                     sizeof(s_audit_gdf[0]), "%s", name);
            s_audit_gdf_count++;
        } else {
            s_audit_catalog_overflow = 1;
        }
    }
    if (n >= 4 && strcmp(name + n - 4, ".vcf") == 0) {
        if (s_audit_vcf_count < AUDIT_VCF_MAX) {
            snprintf(s_audit_vcf[s_audit_vcf_count],
                     sizeof(s_audit_vcf[0]), "%s", name);
            s_audit_vcf_count++;
        } else {
            s_audit_catalog_overflow = 1;
        }
    }
    if (s_name_count < MAX_MESH_NAMES && has_geometry_ext(name)) {
        snprintf(s_names[s_name_count], sizeof(s_names[0]), "%s", name);
        s_name_count++;
    }
}

/* ------------------------------------------------------------------ */
/* Exported API (called from the page via Module._web_* / ccall)       */
/* ------------------------------------------------------------------ */

static GeoMesh *s_mesh;
static uint8_t  s_fb[MESHVIEW_FB_W * MESHVIEW_FB_H];
static uint8_t  s_scene_fb[MESHVIEW_FB_W * MESHVIEW_FB_H];
/* COMBAT_FX_MAX now includes the bounded 128-object deployment pool. Keep
 * the shared snapshot out of Wasm's 64 KiB stack; the previous render-local
 * array exceeded it once enough nested mission/render frames were live. */
static CombatFx s_combat_fx[COMBAT_FX_MAX];
static uint8_t  s_pre_hud[MESHVIEW_FB_W * MESHVIEW_FB_H];
/* Exact post-world 2-D composite consumed by WebGPU. The index plane is
 * separate from its coverage mask because paper art can write palette index
 * 0 opaquely, while the older HUD-only layer uses index 0 as its colour key. */
static uint8_t  s_gpu_overlay[MESHVIEW_FB_W * MESHVIEW_FB_H];
static uint8_t  s_gpu_overlay_mask[MESHVIEW_FB_W * MESHVIEW_FB_H];
static int      s_hud_frame_valid;
/* Pixels below the authored upper-HUD band immediately after hud_render_frame.
 * Later pilot/combat overlays share s_scene_fb but are not dashboard layout. */
static int      s_authored_hud_lower_pixels;
static uint32_t s_drive_ticks;
#define DRIVE_LANDING_WAV "vland.wav"
static int      s_landing_sound_seen;
static int      s_fire_hit = -1;         /* last entity the player hit */
static char     s_snd_pcm_last[24];      /* last successful sound load */

/*
 * Native tears down the mission-owned FSM clip table between missions. The
 * exact native cache-retention granularity is not decoded, so the port uses
 * the engine's natural ownership boundary: discard every decoded WAV and
 * lazily reload recurring generic/engine sounds in the next mission. Keeping
 * this in webmain avoids mission.c ownership (H-UAT-049 PORT DECISION).
 */
static void sound_mission_boundary(void)
{
    sound_mission_reset();
    s_snd_pcm_last[0] = '\0';
}

static int      s_damage_flash;
static uint8_t *s_audit_file_bytes;
static size_t   s_audit_file_size;
/* H-UAT-013 directional cue: the attacker's position from the real hit
 * event (combat_who_attacked), valid while the flash runs. */
static int      s_damage_dir_valid;
static double   s_damage_src[3];
/* Harness-only injection: consumed after the ordinary mission/combat tick so
 * the existing hp-delta observer below sees a real combat_shot event. It is
 * not wired to any page control. */
static int      s_damage_probe_attacker = -1;
static int      s_damage_probe_amount;

/* Analog drive input (M7 automation/gamepad channel): web_drive_analog()
 * arms it, web_drive_analog_off() returns to the key buttons. Throttle is
 * signed: positive accelerates, negative brakes. When armed, web_drive_step
 * steers with these floats instead of the held keys (input_poll still drains
 * the event queue either way). */
static int   s_analog_on;
static float s_an_thr, s_an_left, s_an_right;
static int   s_an_fire, s_an_fire_prev;

EMSCRIPTEN_KEEPALIVE
void web_drive_analog(double thr, double left, double right, int fire)
{
    s_analog_on = 1;
    s_an_thr    = (float)thr;
    s_an_left   = (float)left;
    s_an_right  = (float)right;
    s_an_fire   = fire;
}

EMSCRIPTEN_KEEPALIVE
void web_drive_analog_off(void) { s_analog_on = 0; }

/* Human-trial tape seam (docs/HUMAN-UAT.md): s_last_* are the InputButton
 * masks the most recent web_drive_step actually consumed (bit b = button b,
 * input.h order). web_tape_input() arms a replay override: subsequent steps
 * consume the supplied masks instead of the polled keyboard snapshot — the
 * same seam as s_analog_on, but bit-exact for deterministic trial replay. */
static uint32_t s_last_held, s_last_pressed;
static int      s_tape_on;
static uint32_t s_tape_held, s_tape_pressed;

EMSCRIPTEN_KEEPALIVE
uint32_t web_drive_input_held(void) { return s_last_held; }

EMSCRIPTEN_KEEPALIVE
uint32_t web_drive_input_pressed(void) { return s_last_pressed; }

EMSCRIPTEN_KEEPALIVE
void web_tape_input(uint32_t held, uint32_t pressed)
{
    s_tape_on = 1;
    s_tape_held = held;
    s_tape_pressed = pressed;
}

EMSCRIPTEN_KEEPALIVE
void web_tape_input_off(void) { s_tape_on = 0; }

#ifndef I76_BUILD_STAMP
#define I76_BUILD_STAMP "unknown"
#endif

/* Git short hash stamped by build.sh; trial bundles pin themselves to it. */
EMSCRIPTEN_KEEPALIVE
const char *web_build_id(void) { return I76_BUILD_STAMP; }

/* render.explain_pixel programmatic seam. The query-string owner in
 * index.html arms this only under ?dev=1; probes may call the same exports
 * directly around web_render_fixed/web_drive_render. */
EMSCRIPTEN_KEEPALIVE
int web_pixel_history_enable(int x, int y, const char *run_provenance)
{
    return raster_pixel_history_enable(x, y, 64, I76_BUILD_STAMP,
                                       run_provenance ? run_provenance : "web-probe");
}

EMSCRIPTEN_KEEPALIVE
void web_pixel_history_disable(void) { raster_pixel_history_disable(); }

EMSCRIPTEN_KEEPALIVE
int web_pixel_history_active(void) { return raster_pixel_history_active(); }

EMSCRIPTEN_KEEPALIVE
const char *web_pixel_history_json(void)
{
    return raster_pixel_history_json();
}

/* 0 = consumer (probe misses silent); >0 = ?dev=1 (record in dev panel).
 * Required-asset vfs_read_file misses stay loud regardless. */
EMSCRIPTEN_KEEPALIVE
void web_set_log_level(int level) { vfs_set_probe_log(level > 0); }

/* Presentation-only dev aid: native paper maps are static and have no pin. */
EMSCRIPTEN_KEEPALIVE
void web_set_dev_mode(int enabled) { paper_set_dev_mode(enabled); }

EMSCRIPTEN_KEEPALIVE
int web_vfs_probe_count(void) { return vfs_probe_count(); }

EMSCRIPTEN_KEEPALIVE
const char *web_vfs_probe_name(int i) { return vfs_probe_name(i); }

EMSCRIPTEN_KEEPALIVE
int web_vfs_probe_overflow(void) { return vfs_probe_overflow(); }

static Rgb8     s_pal[256];
static int      s_scene_loaded;
static GeoMesh *s_car_mesh;   /* part 0 — legacy Tier-2 GPU parity export */
/* Every decoded player body/wheel/mounted-weapon part, resolved ONCE at drive
 * load. The render loop only composes basis o frame into fixed storage: no
 * per-frame asset lookup or allocation. Mounted GGEO is included here so the
 * existing software dynamic queue and GPU car-part exports share one list. */
static struct { GeoMesh *mesh; double frame[12]; int weapon; int weapon_part; }
    s_car_parts[SCENE_DYN_MAX];
static int      s_car_nparts;
/* Per-source mounted GGEO resolution plus last-render presence receipts for
 * H-UAT-057. They observe the ordinary SW queue/WebGPU overlay; they do not
 * add a gameplay or renderer path. */
#define WEB_AUDIT_WEAPONS 16
static int s_weapon_resolved[WEB_AUDIT_WEAPONS];
static int s_last_sw_car_draws;
static int s_last_fx_events, s_last_fx_sw_present, s_last_fx_gpu_present;
/*
 * First-person set 16 cached once like the exterior list. `source` preserves
 * its purchaser-data role so the render owner can select a proved surface
 * subset without assuming that every day/night/instrument variant is active.
 */
static struct { GeoMesh *mesh; double frame[12]; int source; }
    s_interior_parts[SCENE_DYN_MAX];
static int s_interior_nparts;
static char     s_stats[512];
static char     s_pose[256];
static char     s_mission_path[64]; /* M6 Tier-2: WRLD re-query (terrain tex) */
static void gpu_tex_invalidate(void);
static void webgpu_caches_reset(void);

EMSCRIPTEN_KEEPALIVE
int web_init(void)
{
    fs_set_root("/data");
    if (!vfs_init()) return -1;
    meshcache_init();
    input_init();
    build_palette(s_pal);
    s_name_count = 0;
    s_audit_gdf_count = s_audit_vcf_count = 0;
    s_audit_catalog_overflow = 0;
    vfs_foreach(collect_cb, NULL);
    return s_name_count;
}

EMSCRIPTEN_KEEPALIVE
int web_asset_profile(void)
{
    return (int)vfs_profile();
}

EMSCRIPTEN_KEEPALIVE
int web_mesh_count(void) { return s_name_count; }

EMSCRIPTEN_KEEPALIVE
const char *web_mesh_name(int i)
{
    if (i < 0 || i >= s_name_count) return "";
    return s_names[i];
}

EMSCRIPTEN_KEEPALIVE
int web_load_mesh(int i)
{
    if (i < 0 || i >= s_name_count) return -1;
    if (s_mesh) { geo_cache_release(s_mesh); s_mesh = NULL; }
    s_mesh = (GeoMesh *)geo_cache_acquire(s_names[i]);
    return s_mesh ? s_mesh->num_verts : -1;
}

EMSCRIPTEN_KEEPALIVE
int web_num_verts(void) { return s_mesh ? s_mesh->num_verts : 0; }

EMSCRIPTEN_KEEPALIVE
int web_num_faces(void) { return s_mesh ? s_mesh->num_faces : 0; }

EMSCRIPTEN_KEEPALIVE
const char *web_mesh_title(void) { return s_mesh ? s_mesh->name : ""; }

EMSCRIPTEN_KEEPALIVE
void web_render(double yaw, double zoom)
{
    render_mesh(s_fb, s_mesh, yaw, zoom);
}

/* Pump the JS-fed event queue through input.c and return the held-button
 * bitmask (bit i = InputButton i). */
EMSCRIPTEN_KEEPALIVE
uint32_t web_poll_input(void)
{
    InputState in;
    input_poll(&in);
    uint32_t bits = 0;
    for (int i = 0; i < INPUT_BTN_COUNT; i++)
        if (in.held[i]) bits |= 1u << i;
    return bits;
}

/* ------------------------------------------------------------------ */
/* Mission mode (M2): terrain + placed objects. See note in            */
/* web_mission_render about per-module cameras.                        */
/*
 * Mission terrain used extent in world meters, parsed out of terrain_stats
 * ("bbox=[x0..x1]x[z0..z1]" in PATCH units — terrain.c's exact format).
 * Returns 0 and fills the corners, -1 when no terrain / the parse fails.
 * Single source of truth for both the drivable-area bound and the GPU
 * terrain grid, so those two cannot disagree about where the world ends.
 */
static int terrain_used_extent(double *x0, double *z0, double *x1, double *z1)
{
    if (!terrain_is_loaded()) return -1;
    char buf[512];
    if (terrain_stats(buf, sizeof buf) < 0) return -1;
    int cx0, cx1, cz0, cz1;
    const char *b = strstr(buf, "bbox=[");
    if (!b || sscanf(b, "bbox=[%d..%d]x[%d..%d]",
                     &cx0, &cx1, &cz0, &cz1) != 4) return -1;
    if (cx1 < cx0 || cz1 < cz0) return -1;
    *x0 = cx0 * TERRAIN_PATCH_SIZE_M;
    *z0 = cz0 * TERRAIN_PATCH_SIZE_M;
    *x1 = (cx1 + 1) * TERRAIN_PATCH_SIZE_M;
    *z1 = (cz1 + 1) * TERRAIN_PATCH_SIZE_M;
    return 0;
}

/*
 * Object colliders for the sim (car.h car_set_colliders). Oriented boxes
 * with a height span (car.h D19) over the placed scene objects, so the car
 * stops at buildings and barriers instead of driving through them (car.c
 * D8's "future collision pass"). Markers and spawn points carry no geometry
 * and scene_obj_part_obb rejects them, so they never become obstacles.
 *
 * The table is owned here and lives as long as the loaded scene; it is
 * cleared before scene_unload so the sim can never hold a stale pointer.
 * Drive mode rebuilds it once after mission_attach and excludes every
 * mission-owned vehicle. Those live hulls belong to mission.c's per-tick
 * vehicle contact phase, never to this immutable scenery snapshot.
 *
 * Colliders are PER PART, not per object: one aggregate OBB cannot
 * represent a building standing on its own drive-on apron — P01's bflgila1
 * unioned 21 parts into a single 60x37 m box that walled off the whole
 * gas-station lot. Parts the scene classifies as ground drive surface
 * (scene_part_drive_surface: large, near-horizontal, vehicle-thin, at the
 * object's base — data-derived geometry, no labels or coordinates) are
 * skipped; the car rides the terrain across them while walls, pumps,
 * posts, signs and roofs keep their own boxes. The same classification
 * drives the renderer's 0.1 m surface lift (scene.c D11), so the part the
 * car crosses is also the part that cannot z-fight the heightfield.
 *
 * The old horizontal-circle collider needed a 20 m radius cap, because a
 * circle around anything large or elongated bulges far past the geometry
 * (N01's bridge bounded to 30.5 m and stopped the car ~67 m short of it).
 * Everything over the cap was simply not an obstacle, so the car drove
 * through big buildings. A box hugs the footprint, so the cap is gone and
 * all scenery with geometry is solid. Degenerate boxes (a zero half-extent
 * or a zero vertical span, i.e. flat decals and ground patches) are still
 * skipped — they have no volume to collide with and would otherwise act as
 * invisible walls.
 */
static CarCollider *s_colliders_tbl;
typedef struct {
    int object;
    int part;
} ColliderSource;
static ColliderSource *s_collider_sources;
static int s_ncolliders_tbl;
static unsigned s_colliders_scene_gen;
static int s_colliders_exclude_vehicles;


/* Below this half-extent a part is treated as having no footprint. */
#define COLLIDER_MIN_HALF 0.25
/* Below this vertical span a part is a flat decal, not an obstacle. */
#define COLLIDER_MIN_THICK 0.05

static void build_colliders(int exclude_vehicles)
{
    s_colliders_exclude_vehicles = exclude_vehicles ? 1 : 0;
    s_colliders_scene_gen = scene_gate_generation();
    free(s_colliders_tbl);
    free(s_collider_sources);
    s_colliders_tbl = NULL;
    s_collider_sources = NULL;
    s_ncolliders_tbl = 0;
    car_set_colliders(NULL, 0);
    if (!s_scene_loaded) return;

    int n = scene_obj_count();
    if (n <= 0) return;
    int cap = 0;
    for (int i = 0; i < n; i++) {
        if (exclude_vehicles && mission_scene_object_is_vehicle(i)) continue;
        cap += scene_obj_part_count(i);
    }
    if (cap <= 0) return;
    CarCollider *tbl = malloc((size_t)cap * sizeof *tbl);
    if (!tbl) return;
    /* Source metadata is diagnostic-only. A failure here must not discard a
     * usable physics table and silently make all static scenery intangible. */
    ColliderSource *sources = malloc((size_t)cap * sizeof *sources);
    int k = 0;
    for (int i = 0; i < n; i++) {
        if (exclude_vehicles && mission_scene_object_is_vehicle(i)) continue;
        int np = scene_obj_part_count(i);
        for (int p = 0; p < np; p++) {
            if (scene_part_drive_surface(i, p)) continue;   /* D11 ground */
            double c[3], half[2], axis[2], yspan[2];
            if (scene_obj_part_obb(i, p, c, half, axis, yspan) != 0)
                continue;
            int gate = scene_obj_part_gate(i, p);
            if (!gate && (half[0] < COLLIDER_MIN_HALF ||
                          half[1] < COLLIDER_MIN_HALF))
                continue;
            if (gate) {
                if (half[0] < COLLIDER_MIN_HALF) half[0] = COLLIDER_MIN_HALF;
                if (half[1] < COLLIDER_MIN_HALF) half[1] = COLLIDER_MIN_HALF;
            }
            if (yspan[1] - yspan[0] < COLLIDER_MIN_THICK)
                continue;
            tbl[k].x  = c[0];
            tbl[k].z  = c[2];
            tbl[k].hx = half[0];
            tbl[k].hz = half[1];
            tbl[k].ax = axis[0];
            tbl[k].az = axis[1];
            tbl[k].y0 = yspan[0];
            tbl[k].y1 = yspan[1];
            tbl[k].drivable_object = scene_part_drivable_object(i, p);
            tbl[k].drivable_parent = scene_part_drivable_parent(i, p);
            if (sources) {
                sources[k].object = i;
                sources[k].part = p;
            }
            k++;
        }
    }
    if (k == 0) {
        free(tbl);
        free(sources);
        return;
    }
    s_colliders_tbl = tbl;
    s_collider_sources = sources;
    s_ncolliders_tbl = k;
    car_set_colliders(s_colliders_tbl, s_ncolliders_tbl);
}

/* Keep the car inside the mission's terrain. Past the used extent the
 * heightfield is flat empty and the renderer draws nothing, so driving out
 * there put the player in a featureless void (measured on N01: z 49910
 * against an extent ending at 49280). */
static void apply_world_bounds(void)
{
    double x0, z0, x1, z1;
    if (terrain_used_extent(&x0, &z0, &x1, &z1) == 0)
        car_set_bounds(x0, z0, x1, z1);
    else
        car_set_bounds(0.0, 0.0, 0.0, 0.0);   /* restore the full-grid default */
}

static double drive_vehicle_radius(void *ctx)
{
    (void)ctx;
    return car_collision_radius();
}

static void drive_vehicle_height(double *y0, double *y1, void *ctx)
{
    (void)ctx;
    double y;
    car_pose(NULL, &y, NULL, NULL, NULL, NULL);
    *y0 = y;
    *y1 = y + CAR_COLLIDE_H;
}

static void drive_vehicle_separate(double dx, double dz,
                                   double nx, double nz,
                                   double other_vx, double other_vz,
                                   void *ctx)
{
    (void)ctx;
    car_apply_vehicle_contact(dx, dz, nx, nz, other_vx, other_vz);
}

/* ------------------------------------------------------------------ */

EMSCRIPTEN_KEEPALIVE
int web_mission_load(const char *name)
{
    sound_mission_boundary();        /* old mission clips cannot consume slots */
    car_set_colliders(NULL, 0);      /* s_colliders points into the old scene */
    car_set_bounds(0.0, 0.0, 0.0, 0.0);
    webgpu_caches_reset();           /* before old mission assets unload */
    paper_unload();
    scene_unload();
    terrain_unload();
    car_unload();
    s_scene_loaded = 0;
    s_car_mesh = NULL;
    snprintf(s_mission_path, sizeof s_mission_path, "%s", name);

    /* Keep the native mission_load ownership order: terrain teardown/load
     * first, then scene registration. terrain_load begins with terrain_unload
     * (which clears scene-owned drivable faces), so scene-first made every
     * browser mission finish with an empty class-11/12/13 registry. */
    int t = terrain_load(name);
    int s = scene_load(name);
    s_scene_loaded = (s == 0);
    build_colliders(0);
    apply_world_bounds();

    char a[256] = {0}, b[256] = {0};
    if (s == 0) scene_stats(a, sizeof(a));
    if (t == 0) terrain_stats(b, sizeof(b));
    snprintf(s_stats, sizeof(s_stats), "%s%s%s",
             a, (a[0] && b[0]) ? " | " : "", b);

    /* M3.5 palette unification: resolve the mission's level palette (and
     * HUD art) so web_level_palette() can hand it to the page. Runs once
     * per drive load too — web_drive_load() comes through here. Best
     * effort: a mission without HUD assets still runs, palette falls
     * back to NULL and the page keeps the mesh palette. */
    if (s == 0 || t == 0) {
        hud_load_mission(name, 0);
        s_hud_frame_valid = 0;
    }

    return (s == 0 || t == 0) ? 0 : -1;
}

EMSCRIPTEN_KEEPALIVE
const char *web_mission_stats(void) { return s_stats; }

/* Browser-lifecycle proof seam for scene-owned drivable faces. The compact
 * JSON is intentionally read-only: acceptance probes must show this count is
 * live after web_mission_load/web_drive_load, not infer it from car motion. */
EMSCRIPTEN_KEEPALIVE
const char *web_drivable_stats(void)
{
    static char out[64];
    int objects = 0, faces = 0;
    terrain_drivable_stats(&objects, &faces);
    snprintf(out, sizeof out, "{\"objects\":%d,\"faces\":%d}",
             objects, faces);
    return out;
}

/* Immutable scenery bodies in the current car collider snapshot. */
EMSCRIPTEN_KEEPALIVE
int web_static_collider_count(void) { return s_ncolliders_tbl; }

/*
 * HUD overlay composite. The world's terrain+scene composite moved to
 * render.c; this remains only for the cockpit/HUD pass, which renders into
 * the same scratch buffer and overlays it by colour key (palette index 0 is
 * the shared background and doubles as "transparent").
 */
static void composite_scene_over_terrain(void)
{
    for (int i = 0; i < MESHVIEW_FB_W * MESHVIEW_FB_H; i++)
        if (s_scene_fb[i] != 0) s_fb[i] = s_scene_fb[i];
}

/*
 * Build the exact software-owned 2-D layer for the GPU present path:
 * authored HUD/cockpit, sidearm and combat FX, paper UI, then damage flash.
 *
 * HUD coverage is its established nonzero colour key. Paper has mixed
 * semantics: map/title can write palette index 0 opaquely, while keyed
 * notepad/escape texels leave the destination alone. paperui intentionally
 * exposes only its final compositor, so render it over two distinct constant
 * backgrounds. A pixel equal in both results was authored; an untouched
 * pixel remains 0 in one probe and 1 in the other. This consumes the same
 * decoded art and live paper state as software without reimplementing any
 * paper rules in JavaScript.
 */
static void damage_flash_render(uint8_t *fb, uint8_t *mask, int w, int h,
                                const CameraView *cam);

static uint32_t audit_fnv1a(const uint8_t *p, size_t n)
{
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static void build_gpu_overlay_layer(const CameraView *cam)
{
    const size_t n = sizeof s_gpu_overlay;
    memset(s_gpu_overlay, 0, n);
    memset(s_gpu_overlay_mask, 1, n);
    paper_render(s_gpu_overlay, MESHVIEW_FB_W, MESHVIEW_FB_H);
    paper_render(s_gpu_overlay_mask, MESHVIEW_FB_W, MESHVIEW_FB_H);

    /* The two-background test identifies paper coverage even when its final
     * palette index equals the pixel already below it. A before/after colour
     * comparison cannot observe that contest. */
    int history_pixel = raster_pixel_history_target_index(
        s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H);
    if (history_pixel >= 0 &&
        s_gpu_overlay[history_pixel] == s_gpu_overlay_mask[history_pixel])
        raster_pixel_history_note_overlay(
            s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, 0,
            "paper-ui", "paper", "post-hud",
            "overwrite_by_later_pass");

    for (size_t i = 0; i < n; i++) {
        if (s_gpu_overlay[i] == s_gpu_overlay_mask[i]) {
            s_gpu_overlay_mask[i] = 0xff; /* paper wins over every lower layer */
        } else if (s_scene_fb[i] != 0) {
            s_gpu_overlay[i] = s_scene_fb[i];
            s_gpu_overlay_mask[i] = 0xff;
        } else {
            s_gpu_overlay_mask[i] = 0;
        }
    }

    /* Safety feedback must remain visible over an active episode card/map.
     * The software path applies the same final ordering below. */
    damage_flash_render(s_gpu_overlay, s_gpu_overlay_mask,
                        MESHVIEW_FB_W, MESHVIEW_FB_H, cam);
}

/*
 * Determinism gate (docs/specs/m8/software-raster.md §8). Render terrain +
 * static scene from an EXPLICIT camera — no chase math, no HUD, no dynamic
 * car — so the native build and the wasm build can be asked for the same
 * frame and their raw index buffers compared byte for byte.
 *
 * The camera is a parameter rather than derived on purpose: this gates the
 * RASTERIZER, not the camera solver, and a caller-supplied eye/target
 * removes every input the two builds could disagree about beforehand.
 *
 * tools/frame_probe.c is the native twin and MUST mirror this sequence
 * exactly — same calls, same order, same buffers. Change one, change both.
 */
EMSCRIPTEN_KEEPALIVE
void web_render_fixed(double ex, double ey, double ez,
                      double tx, double ty, double tz)
{
    double eye[3] = { ex, ey, ez };
    double target[3] = { tx, ty, tz };
    CameraView camera;
    if (camera_view_look_at(&camera, eye, target) == 0) {
        raster_pixel_history_frame_begin(s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H,
                                         s_drive_ticks, "software");
        worldrender_camera(s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, &camera,
                           0.0, 0.0, terrain_is_loaded(), s_scene_loaded);
        raster_pixel_history_frame_end();
    }
}

EMSCRIPTEN_KEEPALIVE
void web_mission_render(double yaw, double dist)
{
    worldrender_orbit(s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, yaw, dist,
                       terrain_is_loaded(), s_scene_loaded);
}

/* Backend selector (docs/specs/m8/software-raster.md §7): 0 = wire-only,
 * 1 = software-filled, 2 = software-filled+wire. Exported so the determinism
 * gate can drive the SAME backend on both sides. */
EMSCRIPTEN_KEEPALIVE
void web_set_backend(int b) { worldrender_set_backend((WorldBackend)b); }

EMSCRIPTEN_KEEPALIVE
const char *web_backend_name(void) { return worldrender_backend_name(); }

/* Geometry-covered pixels in the last filled frame; see worldrender.h. */
EMSCRIPTEN_KEEPALIVE
double web_geometry_pixels(void) { return (double)worldrender_geometry_pixels(); }

/*
 * Per-frame filled-scene face accounting: submitted / backface-culled /
 * cut-out-skipped. Coverage alone cannot tell "the object drew" from "the
 * object was culled and the terrain behind it filled those pixels", which is
 * exactly the confusion this resolves.
 */
/*
 * FNV-1a over the m-tier texture index's whole sorted sequence, computed
 * identically to tools/pixidx_probe.c's.
 *
 * The point is cross-target agreement about WHICH pak slice each texture name
 * resolves to. 11 keys appear in two different paks, qsort is unstable, and
 * native and wasm link different qsort implementations — so without a total
 * order the two targets could each pick a different winner and the frame gate
 * would fail for a reason with nothing to do with rendering. One number makes
 * that comparable.
 */
EMSCRIPTEN_KEEPALIVE
uint32_t web_pixidx_fnv1a(void)
{
    PixIndex *ix = pixidx_build("m.pix");
    if (!ix) return 0u;
    uint32_t h = 2166136261u;
    int n = pixidx_count(ix);
    for (int i = 0; i < n; i++) {
        const PixEnt *e = pixidx_at(ix, i);
        for (const char *p = e->key; *p; p++) { h ^= (uint8_t)*p; h *= 16777619u; }
        for (const char *p = e->pak; *p; p++) { h ^= (uint8_t)*p; h *= 16777619u; }
        for (int k = 0; k < 4; k++) { h ^= (uint8_t)(e->off >> (k * 8)); h *= 16777619u; }
        for (int k = 0; k < 4; k++) { h ^= (uint8_t)(e->len >> (k * 8)); h *= 16777619u; }
    }
    pixidx_free(ix);
    return h;
}

EMSCRIPTEN_KEEPALIVE
int web_pixidx_count(void)
{
    PixIndex *ix = pixidx_build("m.pix");
    int n = pixidx_count(ix);
    pixidx_free(ix);
    return n;
}

EMSCRIPTEN_KEEPALIVE
const char *web_scene_filled_stats(void)
{
    static char buf[160];
    scene_filled_stats(buf, sizeof buf);
    return buf;
}

EMSCRIPTEN_KEEPALIVE
void web_terrain_view(int mode) { terrain_set_view(mode); }

/* ---- Road/nav diagnostics (trial tooling) ------------------------------
 * web_road_clearance: signed XZ clearance from the authored RSEG road
 * ribbons — the same accepted decode and math as tools/ai_probe.c's
 * road_clearance(): negative inside the road edges, positive meters
 * beyond the nearest edge, 1e30 when no roads are loaded. Trial tooling
 * uses it to classify on-road/off-road per tick without naming Nitro's
 * internal surface classes.
 *
 * web_nav_sample: packs terrain_nav_sample as (surface_class << 1) |
 * blocked, -1 on no-terrain/invalid cell. The numeric class is reported
 * as-is: road/off-road class names remain INFERRED (terrain.h §nav).
 */
EMSCRIPTEN_KEEPALIVE
double web_road_clearance(double x, double z)
{
    double best = 1e30;
    for (int seg = 0; ; seg++) {
        double a[6];
        if (terrain_road_point(seg, 0, a) != 0)
            break;
        for (unsigned piece = 1; ; piece++) {
            double b[6];
            if (terrain_road_point(seg, piece, b) != 0)
                break;
            double ax = (a[0] + a[3]) * 0.5;
            double az = (a[2] + a[5]) * 0.5;
            double bx = (b[0] + b[3]) * 0.5;
            double bz = (b[2] + b[5]) * 0.5;
            double dx = bx - ax, dz = bz - az;
            double l2 = dx * dx + dz * dz;
            double t = l2 > 1e-12
                     ? ((x - ax) * dx + (z - az) * dz) / l2 : 0.0;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
            double px = ax + t * dx, pz = az + t * dz;
            double aw = hypot(a[0] - a[3], a[2] - a[5]) * 0.5;
            double bw = hypot(b[0] - b[3], b[2] - b[5]) * 0.5;
            double clearance = hypot(x - px, z - pz)
                             - (aw + t * (bw - aw));
            if (clearance < best) best = clearance;
            memcpy(a, b, sizeof a);
        }
    }
    return best;
}

EMSCRIPTEN_KEEPALIVE
int web_nav_sample(double x, double z)
{
    unsigned cls = 0;
    int blocked = 0;
    if (terrain_nav_sample(x, z, &cls, &blocked) != 0)
        return -1;
    return (int)((cls << 1) | (blocked ? 1u : 0u));
}

/* ------------------------------------------------------------------ */
/* Drive mode (M3): placeholder car sim + the manual camera family.     */
/*                                                                     */
/* The page drives the fixed-step discipline: it calls web_drive_step  */
/* at 20 Hz (time accumulator) and web_drive_render once per frame.    */
/* Car basis: yaw about +Y (fwd=(-sin,cos) per car.h), then pitch      */
/* (nose-up +) about right, then roll (right-side-up +) about forward. */
/* ------------------------------------------------------------------ */

static void drive_pose_string(void)
{
    double x, y, z, yaw, pitch, roll;
    CarLive live = {0};
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    car_get_live(&live);
    paper_set_player(x, z, yaw);
    snprintf(s_pose, sizeof(s_pose),
             "{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"yaw\":%.4f,"
             "\"pitch\":%.3f,\"roll\":%.3f,\"speed\":%.2f,"
             "\"rpm\":%.0f,\"gear\":%d,\"reverse\":%d,"
             "\"handbrake\":%d,\"throttle\":%.3f,\"tick\":%u}",
             x, y, z, yaw, pitch, roll, car_speed(), live.engine_rpm,
             live.gear, live.reverse, live.handbrake, live.throttle_pos,
             s_drive_ticks);
}

/* Last-resort player car when neither the caller nor the mission names one
 * (every FSM-less melee/race/capture mission). Was the hard-coded choice
 * for EVERY mission before the mission's own record became reachable. */
#define DRIVE_FALLBACK_VCF "vdrampg2"

static char s_drive_vcf[32];    /* .vcf base name actually loaded */

/* ------------------------------------------------------------------ */
/* Pilot glance + rearview mirror (presentation only)                  */
/*                                                                     */
/* Stock bindings (nitro input.map / KEYBOARD.MAP): pilot_glance_left  */
/* = GreyLeftArrow, _right = GreyRightArrow, _down = GreyUpArrow,      */
/* _up = GreyDownArrow, pilot_glance_target = Insert. Glances are held */
/* actions; release returns the view forward (release IS the reset —   */
/* the stock map carries no separate center binding). Everything here  */
/* is derived from the car basis at camera time: the car's yaw/pose    */
/* and the mission camera stack are never touched, and a live mission  */
/* camera overrides the glance outright (drive_camera).                */
/* ------------------------------------------------------------------ */
static int s_look_x;        /* -1 left window, 0 ahead, +1 right window */
static int s_look_y;        /* -1 dash, 0 level, +1 windshield top      */
static int s_look_target;   /* Insert held: look at target-or-ahead     */
static int s_rearview;      /* backquote toggle; presents cockpit-only  */

/*
 * Pilot sidearm presentation: an explicit finite animation over the
 * purchaser ZH[L/R]45<class>* VQM frames (hud_sidearm_frame_*). While
 * glancing out a side window with the .45 present, IDLE shows the first
 * ZH* frame; a successful contextual shot plays the ZH* sequence in
 * manifest (archive) order, one frame per fixed 20 Hz drive tick, then
 * returns to IDLE. The ZS* trio is deliberately excluded — its role is
 * UNRESOLVED in every source we have, and no cadence is inferred from
 * frame appearance.
 */
static int s_sidearm_anim = -1; /* -1 idle, else ZH-sequence position   */
static int s_sidearm_anim_side; /* HUD_SIDEARM_LEFT/RIGHT while playing */

/*
 * Glance magnitudes (DECISION: the stock maps bind the four directions but
 * no reversed/native angle source exists, so these are presentation
 * constants, isolated so a future native finding replaces exactly this
 * block). ±90° is the side window — the same direction the contextual
 * sidearm fire uses, so aim and view can never disagree.
 */
#define LOOK_YAW_SIDE    1.57079632679489661923   /* ±90°: side window  */
#define LOOK_PITCH_UP    0.45                     /* windshield top     */
#define LOOK_PITCH_DOWN  0.30                     /* toward the dash    */

/*
 * Mirror destination in the 640x480 frame. The original-video reference
 * places the visible mirror at the far-right, below the condition panel
 * (u=.725-1.0, v=.281-.441). The authored 256x64 mask is sampled into this
 * rectangle; its source dimensions are not screen dimensions.
 */
#define MIRROR_X 464
#define MIRROR_Y 135
#define MIRROR_W 176
#define MIRROR_H 76
static uint8_t s_mirror_fb[256 * 64];   /* fixed rect still fits this store */

/* Defined beside car_basis below; web_drive_load and the export block
 * sit earlier in the file. */
static void look_apply(CameraView *cam);
static int  rearview_derive(CameraView *out);
static void drive_camera_reset(void);
static void render_pose_reset(void);
static void render_pose_tick(void);
static CarWeaponInfo s_sidearm_wi;      /* stats/sound for fx + arming */

/* The original engsnd.dat key is derived from the loaded VDF vehicle-size
 * class (car_engine_sound_number), not the VCFC performance component. */
static const SoundEngRow *drive_sound_row(void)
{
    int eng_num = car_engine_sound_number();
    if (eng_num < 0 || sound_engsnd_load() <= 0) return NULL;
    for (int i = 0; i < sound_engsnd_rows(); i++) {
        const SoundEngRow *row = sound_engsnd_row(i);
        if (row && row->eng_num == eng_num) return row;
    }
    return NULL;
}

/*
 * A player can leave the terrain camera in a non-terminal airborne/stuck
 * state. Keep one second of stable, grounded history so a recovery returns
 * to ordinary play without changing mission/combat state. Fixed storage:
 * no per-frame allocation and no save-format coupling.
 */
#define DRIVE_RECOVER_HISTORY 64
#define DRIVE_RECOVER_BACK    20
typedef struct {
    double x, z, yaw;
} DriveRecoverPose;
static DriveRecoverPose s_recover[DRIVE_RECOVER_HISTORY];
static int s_recover_head;
static int s_recover_count;

static void drive_recover_init(void)
{
    double x, y, z, yaw, pitch, roll;
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    for (int i = 0; i < DRIVE_RECOVER_HISTORY; i++) {
        s_recover[i].x = x;
        s_recover[i].z = z;
        s_recover[i].yaw = yaw;
    }
    s_recover_head = DRIVE_RECOVER_HISTORY - 1;
    s_recover_count = DRIVE_RECOVER_HISTORY;
}

static void drive_recover_record(void)
{
    CarLive lv;
    car_get_live(&lv);
    if (lv.grounded_count < 2 || fabs(lv.pitch) > 0.6 ||
        fabs(lv.roll) > 0.35)
        return;
    s_recover_head = (s_recover_head + 1) % DRIVE_RECOVER_HISTORY;
    s_recover[s_recover_head].x = lv.x;
    s_recover[s_recover_head].z = lv.z;
    s_recover[s_recover_head].yaw = lv.yaw;
    if (s_recover_count < DRIVE_RECOVER_HISTORY)
        s_recover_count++;
}

EMSCRIPTEN_KEEPALIVE
int web_drive_can_recover(void)
{
    return car_is_loaded() && s_recover_count > 0 &&
           mission_state() == MISSION_RUNNING && !mission_cam_active();
}

EMSCRIPTEN_KEEPALIVE
int web_drive_recover(void)
{
    if (!web_drive_can_recover())
        return -1;
    int back = s_recover_count - 1;
    if (back > DRIVE_RECOVER_BACK)
        back = DRIVE_RECOVER_BACK;
    int i = s_recover_head - back;
    if (i < 0)
        i += DRIVE_RECOVER_HISTORY;
    car_place(s_recover[i].x, s_recover[i].z, s_recover[i].yaw);
    mission_set_car(s_recover[i].x, s_recover[i].z, s_recover[i].yaw);
    drive_recover_init();
    drive_pose_string();
    render_pose_reset();
    return 0;
}

/*
 * First-person frame selection at the cockpit ownership boundary.
 *
 * AUTHORED-DATA CORRECTION (H-UAT-032): vjsovern.vdf's set-16 JS51HORN
 * record names JS51BDYF as its parent, but the record's complete 12-float
 * local frame is identical to set 0's model-space JS11HORN frame. Generic
 * parent composition therefore adds BDYF a second time (most visibly +1.585 m
 * forward), floating the bull horns beyond the hood. The matching set-0 role
 * supplies the authored model-space frame already used by the exterior; this
 * is not a screen-space nudge or a vehicle-name special case. Other cockpit
 * roles retain car.c's ordinary composed frame, and a HORN without a matching
 * authored exterior role falls back unchanged.
 */
static int cockpit_part_frame(int source, double out[12])
{
    if (car_interior_part_frame(source, out) != 0)
        return -1;
    const char *name = car_interior_part_name(source);
    size_t len = name ? strlen(name) : 0;
    if (len < 4 || strcasecmp(name + len - 4, "HORN") != 0)
        return 0;
    for (int i = 0; i < car_body_part_count(); i++) {
        const char *body = car_part_name(i);
        size_t blen = body ? strlen(body) : 0;
        if (blen >= 4 && strcasecmp(body + blen - 4, "HORN") == 0)
            return car_part_frame(i, out);
    }
    return 0;
}

EMSCRIPTEN_KEEPALIVE
int web_drive_load(const char *mission, const char *vcf)
{
    if (web_mission_load(mission) != 0) return -1;

    /* FSM attach is non-fatal: FSM-less missions (all Nitro .CBT) just run.
     * It runs BEFORE car_load because the mission's own player record names
     * the car and identifies every live vehicle scene object. Rebuild the
     * immutable collider table now so those bodies do not leave spawn ghosts;
     * mission.c owns their per-tick live contact geometry. */
    (void)mission_attach(mission);
    build_colliders(1);

    /* Vehicle precedence: an explicit caller/garage choice, else the car
     * the mission itself names through its FSM `user` entity (scripted
     * trips each name their own: P01 vdrampg2, P03 vjsovrn1, P13
     * vleoprd2), else the fallback. A hard-coded name here was wrong for
     * every scripted mission but the first. */
    char pick[32];
    if (vcf && *vcf) {
        snprintf(pick, sizeof pick, "%s", vcf);
    } else {
        const char *own = mission_player_object();
        snprintf(pick, sizeof pick, "%s",
                 (own && *own) ? own : DRIVE_FALLBACK_VCF);
    }
    if (car_load(pick) != 0) {
        /* mission_attach already set the runner live; leaving it attached
         * to a drive that never started would let web_shell_update() see a
         * stale mission_state(). Roll it back. */
        mission_unload();
        paper_unload();
        return -2;
    }
    /* mission_attach initially registers the mission-authored user car.
     * A garage override is the physical player now, so its authored defense
     * must replace that one combat registration just as its weapons do.
     * The mission bridge then applies Nitro's decoded single-player 2x pools
     * once, after this selected-VCF rebind (combat.h D-C28). */
    mission_set_player_combat_config(pick);
    mission_set_vehicle_contact_host(drive_vehicle_radius,
                                     drive_vehicle_height,
                                     drive_vehicle_separate, NULL);
    snprintf(s_drive_vcf, sizeof s_drive_vcf, "%s", pick);

    /*
     * Start pose, best source first.
     *
     * 1. mission_spawn() — the FSM `user` entity's ODEF record. Scripted
     *    trips only; every arena mission returns -1 here.
     * 2. mission_start_pose() — the arena grid slot AND its heading. This
     *    is new, and it is what makes a race drivable: the old chain fell
     *    straight through to (3), which on N23.RAC picks the `regen`
     *    marker ~130 m off the course and faces the car north regardless
     *    of which way the track runs. "Drive through check1" is not a
     *    thing a person can do from a random point facing a random way.
     * 3. scene_first_marker_pos() — first SPAWN/REGEN/CHECK marker, yaw 0.
     */
    double sp[3], syaw = 0.0;
    if (mission_spawn(sp) == 0)
        car_place(sp[0], sp[2], 0.0);
    else if (mission_start_pose(sp, &syaw) == 0)
        car_place(sp[0], sp[2], syaw);
    else if (scene_first_marker_pos(sp) == 0)
        car_place(sp[0], sp[2], 0.0);
    /* No spawn source: car.c's default pose stands (DECISION: v1 fallback). */

    /* Publish the start pose NOW. web_drive_render runs before the first
     * 20 Hz web_drive_step, and an objective readout computed against an
     * unpublished pose opens the race by reporting 48 km to checkpoint 1. */
    {
        double px, py, pz, pyaw, ppitch, proll;
        car_pose(&px, &py, &pz, &pyaw, &ppitch, &proll);
        mission_set_car(px, pz, pyaw);
    }


    /* All decoded body/wheel and mounted-weapon parts for chase/cutscenes.
     * Parent and HLOC chains are already composed in car.c. Unresolved parts
     * are skipped individually, exactly like placed scene objects. */
    s_car_mesh = NULL;
    combat_player_weapons_clear();
    for (int i = 0; i < car_weapon_count(); i++) {
        CarWeaponInfo wi;
        if (car_weapon_get(i, &wi) == 0 && wi.direct_fire) {
            int slot = combat_player_weapon_add(wi.name, wi.damage, wi.ammo,
                                                wi.cooldown_ticks, i,
                                                wi.rear_facing, wi.link_class,
                                                !wi.traverses);
            /* Parsed hardpoints install native ORDF flight speed during
             * combat_player_weapon_add; no legacy GDFC override is needed. */
            (void)slot;
        }
    }
    /* Pilot sidearm (D-C14): armed only from the purchaser archive's
     * gh45.gdf, parsed by car.c's GDF reader; no definition, no sidearm.
     * car_gdf_weapon_info's strings are single-slot — copy them out. */
    combat_player_sidearm_clear();
    memset(&s_sidearm_wi, 0, sizeof s_sidearm_wi);
    {
        CarWeaponInfo gi;
        static char sa_name[17], sa_sound[14];
        if (car_gdf_weapon_info("gh45.gdf", &gi) == 0 && gi.damage > 0) {
            snprintf(sa_name, sizeof sa_name, "%s", gi.name);
            snprintf(sa_sound, sizeof sa_sound, "%s", gi.sound);
            s_sidearm_wi = gi;
            s_sidearm_wi.name = sa_name;
            s_sidearm_wi.sound = sa_sound;
            (void)combat_player_sidearm_set(sa_name, gi.damage, gi.ammo,
                                            gi.cooldown_ticks);
            /* D-C21: authored GDF bullet velocity for the streak. */
            combat_player_sidearm_set_speed(gi.projectile_speed);
        }
    }
    /* Glances are per-drive state. The retail cockpit presents its mirror;
     * each drive therefore starts with it on, while the binding remains a
     * user toggle. */
    s_look_x = s_look_y = s_look_target = 0;
    s_rearview = 1;
    drive_camera_reset();   /* native FUN_434af0 zoom init + port defaults */
    s_sidearm_anim = -1;
    s_car_nparts = 0;
    memset(s_weapon_resolved, 0, sizeof s_weapon_resolved);
    s_last_sw_car_draws = 0;
    s_last_fx_events = s_last_fx_sw_present = s_last_fx_gpu_present = 0;
    {
        int n = car_part_count();
        if (n > SCENE_DYN_MAX) n = SCENE_DYN_MAX;
        for (int i = 0; i < n; i++) {
            GeoMesh *m = scene_part_mesh(car_part_name(i));
            double fr[12];
            if (!m || car_part_frame(i, fr) != 0)
                continue;
            s_car_parts[s_car_nparts].mesh = m;
            memcpy(s_car_parts[s_car_nparts].frame, fr, sizeof fr);
            s_car_parts[s_car_nparts].weapon = -1;
            s_car_parts[s_car_nparts].weapon_part = -1;
            s_car_nparts++;
        }
        if (s_car_nparts > 0)
            s_car_mesh = s_car_parts[0].mesh;
    }
        for (int wi = 0; wi < car_weapon_count() &&
                         s_car_nparts < SCENE_DYN_MAX; wi++) {
            for (int pi = 0; pi < car_weapon_part_count(wi) &&
                             s_car_nparts < SCENE_DYN_MAX; pi++) {
                GeoMesh *m = scene_part_mesh(car_weapon_part_name(wi, pi));
                double fr[12];
                if (!m || car_weapon_part_live_frame(wi, pi, fr) != 0)
                    continue;
                s_car_parts[s_car_nparts].mesh = m;
                memcpy(s_car_parts[s_car_nparts].frame, fr, sizeof fr);
                s_car_parts[s_car_nparts].weapon = wi;
                s_car_parts[s_car_nparts].weapon_part = pi;
                s_car_nparts++;
                if (wi < WEB_AUDIT_WEAPONS)
                    s_weapon_resolved[wi]++;
            }
        }
    /* First-person interior parts (VGEO set 16): same one-time resolution
     * and per-part skip as the body list above. */
    s_interior_nparts = 0;
    {
        int n = car_interior_part_count();
        if (n > SCENE_DYN_MAX) n = SCENE_DYN_MAX;
        for (int i = 0; i < n; i++) {
            GeoMesh *m = scene_part_mesh(car_interior_part_name(i));
            double fr[12];
            if (!m || cockpit_part_frame(i, fr) != 0)
                continue;
            s_interior_parts[s_interior_nparts].mesh = m;
            memcpy(s_interior_parts[s_interior_nparts].frame, fr,
                   sizeof fr);
            s_interior_parts[s_interior_nparts].source = i;
            s_interior_nparts++;
        }
    }
    /* The player's panels are paint SLOTS, not tile names: without the .vtf
     * they resolve to nothing and the car draws flat (texture-pipeline.md). */
    scene_set_dynamic_paint(car_vtf_file());
    /* `nitro.exe` names vland.wav; sound.c resolves the archive's
     * GAS0-wrapped vland.gpw. Do not leak an in-flight landing from the
     * previous drive into this one. */
    sound_stop(DRIVE_LANDING_WAV);
    s_landing_sound_seen = 0;
    s_drive_ticks = 0;
    s_last_held = 0;
    s_last_pressed = 0;
    s_tape_on = 0;
    s_odom = 0.0;
    s_top_speed = 0.0;
    s_an_fire_prev = 0;
    s_damage_flash = 0;
    s_damage_dir_valid = 0;
    s_damage_probe_attacker = -1;
    s_damage_probe_amount = 0;
    drive_recover_init();
    /* Paper surfaces: map/notepad VQM, escape pack, episode title PCX. */
    (void)paper_load_mission(mission);
    /* Episode title card for ~2 s at mission start (dismissible). */
    paper_show_title(40);
    drive_pose_string();
    render_pose_reset();

    /* Input ownership activates only after the mission is fully loaded. Drain
     * any key that clicked/confirmed the shell start, then require its release
     * before the gameplay map can observe it. This is action-general rather
     * than a Space/fire exception. */
    {
        InputState shell_input;
        input_poll(&shell_input);
        input_require_release();
    }
    return 0;
}

/* The .vcf base name the live drive actually loaded (garage/HUD readout).
 * "" before the first successful web_drive_load. */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_vehicle(void) { return s_drive_vcf; }

double *web_gpu_project(double wx, double wy, double wz);

/* Instrumented H-UAT-023 seam: ordinary fire path, exposed only to probes. */
EMSCRIPTEN_KEEPALIVE
const char *web_aim_muzzle_probe(void)
{
    static char buf[384];
    double x,y,z,yaw,pitch,roll,m[3]={0},dir[3]={0},projected[4]={0};
    car_pose(&x,&y,&z,&yaw,&pitch,&roll);
    int source=combat_player_weapon_source();
    int have=combat_player_launch_frame(m,dir)==0;
    int launched=combat_player_fire(NULL);
    int projected_n=0;
    if (source >= 0 && have) {
        for(int i=0;i<2;i++){
            double d=i?200.0:40.0;
            double *p=web_gpu_project(m[0]+dir[0]*d,
                                      m[1]+dir[1]*d,
                                      m[2]+dir[2]*d);
            if(p&&p[2]>0){projected[i*2]=p[0];projected[i*2+1]=p[1];projected_n++;}
        }
    }
    snprintf(buf,sizeof buf,"{\"launch\":%d,\"have\":%d,\"carY\":%.3f,"
             "\"muzzle\":[%.3f,%.3f,%.3f],"
             "\"direction\":[%.6f,%.6f,%.6f],\"projected\":%d,"
             "\"trajectory\":[[%.2f,%.2f],[%.2f,%.2f]]}",launched,have,y,
             m[0],m[1],m[2],dir[0],dir[1],dir[2],projected_n,
             projected[0],projected[1],projected[2],projected[3]);
    return buf;
}

/* H-UAT-067c regression seam: compare the last real launch against the
 * current physical car's authored HLOC o GPOF position. Unlike the older aim
 * probe this does not fire or advance state, so a caller can inspect the same
 * web_drive_step that consumed throttle + Space. */
EMSCRIPTEN_KEEPALIVE
const char *web_audit_muzzle_alignment(void)
{
    static char buf[384];
    double x, y, z, yaw, pitch, roll, actual[3] = {0}, expected[3] = {0};
    int source = combat_player_weapon_source();
    int have = combat_player_muzzle_position(actual) == 0;
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    if (source >= 0) {
        double mf[12];
        if (car_weapon_muzzle_frame(source, mf) == 0) {
            double cy = cos(yaw), sy = sin(yaw);
            expected[0] = x + mf[9] * cy - mf[11] * sy;
            expected[1] = y + mf[10];
            expected[2] = z + mf[9] * sy + mf[11] * cy;
        } else {
            source = -1;
        }
    }
    double dx = actual[0] - expected[0];
    double dy = actual[1] - expected[1];
    double dz = actual[2] - expected[2];
    snprintf(buf, sizeof buf,
             "{\"have\":%d,\"source\":%d,\"actual\":[%.9f,%.9f,%.9f],"
             "\"expected\":[%.9f,%.9f,%.9f],\"error\":%.12f,"
             "\"car\":[%.9f,%.9f,%.9f]}",
             have, source, actual[0], actual[1], actual[2], expected[0],
             expected[1], expected[2], sqrt(dx * dx + dy * dy + dz * dz),
             x, y, z);
    return buf;
}

static void car_basis(double out12[12]);
static void car_basis_live(double out12[12]);

/* out = basis ∘ frame, columns r/u/f/pos (scene.c D4; xform_compose's
 * "a after b"). Shared by the software drive queue and the Tier-2 GPU
 * export so both paths place parts identically. */
static void basis_compose_frame(double out[12], const double basis[12],
                                const double frame[12])
{
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++)
            out[c * 3 + r] = basis[0 * 3 + r] * frame[c * 3 + 0]
                           + basis[1 * 3 + r] * frame[c * 3 + 1]
                           + basis[2 * 3 + r] * frame[c * 3 + 2];
        out[9 + r] = basis[0 * 3 + r] * frame[9 + 0]
                   + basis[1 * 3 + r] * frame[9 + 1]
                   + basis[2 * 3 + r] * frame[9 + 2]
                   + basis[9 + r];
    }
}

/*
 * Drive-view camera family, selected by web_drive_set_view /
 * web_drive_preset. Native evidence: nitro.exe's action descriptor table
 * (0x004f47d0) names COCKPIT/FREEEYE/CHASE/OVER/TRACK plus
 * PRESET_VIEW_1..12, and the purchaser gamekey.map binds the presets to
 * F1..F11 (F1=1, F2=2, F3=3, F4=12, F5=11, F6=4, F7=6, F8=7, F9=8,
 * F10=10, F11=5; preset 9 has no stock key; F12 is TOGGLE_VIDEOMODE, not
 * a camera). The preset handlers (0x435aa0..0x4381b0) resolve to these
 * families:
 *
 *   DRIVE_VIEW_COCKPIT (native mode 0, F1) — first person from the
 *     driver's seat (callback 0x435c30). The cockpit/dashboard art the
 *     HUD composites is drawn for THIS view; eye and look direction come
 *     from car_basis, so the horizon tilts with the body over bumps.
 *   DRIVE_VIEW_CHASE (native mode 7, F2) — the adjustable spherical
 *     chase. Callback 0x435330 maintains user yaw/pitch and clamps the
 *     shared zoom global (DAT_4f38bc, init exactly 5.5 in FUN_434af0) to
 *     [1,10].
 *   DRIVE_VIEW_FIXED (native mode 1) — the fixed spherical presets:
 *     F3 yaw 180/pitch 15, F7 yaw 0 offset (0,-1.5,+0.2), F8 yaw 180
 *     offset (0,-1.5,-0.25), F9 yaw 150/pitch 15, preset 9 yaw 30/
 *     pitch 15. Callback 0x436c50 places the eye on the yaw/pitch
 *     spherical ray at radius min(150, zoom * camera-radius), follows the
 *     object transform, and raises the tracked point +2.0 m.
 *   DRIVE_VIEW_OVER (native mode 3, F10) — OVER_VIEW callback 0x437b90:
 *     starts at height 100.0 over the player's x/z. Its native height
 *     speed-scaling law is unresolved; height is input-adjustable here.
 *   DRIVE_VIEW_TRACK (native mode 4, F4/F5) — shared two-subject tracker
 *     0x437f10 framing the player and the current target (FUN_45ecf0);
 *     F4 (PRESET_VIEW_12) and F5 (PRESET_VIEW_11) pass the endpoints
 *     swapped, so they present opposite sides of the same player-target
 *     framing (Manual.pdf 672-674: F4 keeps the targeted vehicle in
 *     view).
 *   DRIVE_VIEW_FREEEYE (F6, PRESET_VIEW_4) — mode-1 variant with its own
 *     callback 0x436800: pointer-driven player-relative direction at the
 *     verified 120-degree exception FOV (tan(60) half-angle).
 *
 * The native camera-radius source (followed-object field +0x10) is not
 * reversed; car_collision_radius() — the loaded chassis' COLP outer
 * horizontal extent — is the labelled substitution, so the default orbit
 * distance is approximate. No native manual chase distance measurement
 * exists; the 5.5 zoom default, the [1,10] zoom clamp and the 150 m
 * radius cap ARE native. Residual unknowns: the +0x10 per-vehicle field,
 * mode-8's external object (F11 routes to the tracker), and OVER_VIEW's
 * speed-scaling law.
 *
 * Shared by web_drive_render and the M6 Tier-2 GPU export, so the
 * software and hardware paths can never disagree about where the camera
 * is.
 */
enum {
    DRIVE_VIEW_COCKPIT = 0,   /* native mode 0 (F1)                      */
    DRIVE_VIEW_CHASE   = 1,   /* native mode 7 adjustable orbit (F2)     */
    DRIVE_VIEW_FIXED   = 2,   /* native mode 1 spherical presets         */
    DRIVE_VIEW_OVER    = 3,   /* native mode 3 overhead (F10)            */
    DRIVE_VIEW_TRACK   = 4,   /* native mode 4 target tracker (F4/F5)    */
    DRIVE_VIEW_FREEEYE = 5,   /* F6: pointer-direction, 120 deg FOV      */
    DRIVE_VIEW_COUNT
};
static int s_drive_view = DRIVE_VIEW_COCKPIT;

/* Manual camera adjustables (reset per drive; see web_drive_load).
 * Nitro's zoom clamp, GreyEnd reset, radius cap, and 30-degree chase
 * elevation are binary-derived. The browser starts at zoom 2.8: this is
 * inside Nitro's native range and matches the 20-30% vehicle-height
 * occupancy measured in ordinary original chase footage; 5.5 made the
 * production car read as a distant thumbnail. */
#define CAM_ZOOM_START  2.8                     /* source-video framing   */
#define CAM_ZOOM_RESET  5.5                     /* FUN_434af0: exact      */
#define CAM_ZOOM_MIN    1.0                     /* DAT_4f38bc clamp       */
#define CAM_ZOOM_MAX   10.0
#define CAM_RADIUS_MAX 150.0                    /* mode-1 radius cap (m)  */
#define CAM_TRACK_UP    2.0                     /* tracked-point +2.0 m   */
#define CAM_CHASE_EL_INIT 0.52359877559829887308 /* 30 deg, FUN_434af0 */
#define CAM_FIXED_EL_INIT 0.26179938779914943654 /* 15 deg, mode-1 family */
#define CAM_OVER_INIT   100.0                   /* OVER_VIEW start height */
#define CAM_FREEEYE_FOV_TAN 1.7320508075688772  /* tan(60) = 120 deg hFOV */
#define CAM_YAW_RATE    0.045                   /* rad per 20 Hz tick     */
#define CAM_EL_RATE     0.03
#define CAM_EL_MIN      0.0
#define CAM_EL_MAX      1.45
#define CAM_ZOOM_RATE   0.15                    /* zoom units per tick    */
#define CAM_OVER_MIN   10.0                     /* port clamp (no native) */
#define CAM_OVER_MAX   400.0
#define CAM_PAN_MAX    150.0

static double s_cam_zoom = CAM_ZOOM_START; /* source-video chase scale   */
static double s_cam_yaw_off;             /* orbit azimuth offset, rad   */
static double s_cam_el = CAM_CHASE_EL_INIT; /* orbit elevation, rad      */
static double s_free_yaw, s_free_pitch;  /* free-eye direction offsets  */
static double s_over_pan_x, s_over_pan_z;/* overhead pan, car-local m   */
static double s_over_h = CAM_OVER_INIT;  /* overhead height             */
/* Fixed preset parameters (mode 1): spherical azimuth/elevation plus the
 * optional car-local eye offset. The native offsets' middle component is
 * negated into the engine's Y-up (the (0,-1.5,±) values read as 1.5 m UP
 * roof/hood points; that sign flip is INFERRED from the values, not from
 * a reversed convention). */
static double s_fix_yaw, s_fix_pitch = CAM_FIXED_EL_INIT, s_fix_off[3];
static int    s_track_swap;              /* F5: frame the player side   */

/* Read-only presentation snapshots bracketing the latest fixed sim tick.
 * Extrapolation predicts from CURRENT with car.c's current tick velocity;
 * heading uses car.c's current yaw rate. */
enum {
    RENDER_CLAMP_SCRIPTED = 1u << 0,
    RENDER_CLAMP_VIEW     = 1u << 1,
    RENDER_CLAMP_CONTACT  = 1u << 2,
    RENDER_CLAMP_ACCEL    = 1u << 3,
    RENDER_CLAMP_RESET    = 1u << 4
};
#define RENDER_TICK_DT 0.05
#define RENDER_ACCEL_CLAMP_MPS2 300.0 /* contact-free hard-impact backstop */
static CameraView s_render_cam_prev, s_render_cam_curr, s_render_camera;
static double s_render_basis_prev[12], s_render_basis_curr[12];
static double s_render_basis[12];
static double s_render_alpha = 1.0;
static int s_render_pose_valid;
static int s_render_prev_scripted, s_render_curr_scripted;
static int s_render_prev_view, s_render_curr_view;
static int s_render_basis_active;
static int s_render_extrapolate;
static unsigned s_render_discontinuity;
static unsigned s_render_clamp_reason;
static uint32_t s_render_player_contacts_seen;
static double s_render_velocity_x, s_render_velocity_z;
static double s_render_yaw_rate;
static int s_render_velocity_constrained;
static int s_render_heading_extrapolate;

static void drive_camera_reset(void)
{
    s_cam_zoom   = CAM_ZOOM_START;
    s_cam_yaw_off = 0.0;
    s_cam_el     = CAM_CHASE_EL_INIT;
    s_free_yaw = s_free_pitch = 0.0;
    s_over_pan_x = s_over_pan_z = 0.0;
    s_over_h     = CAM_OVER_INIT;
    s_fix_yaw    = 3.14159265358979323846;
    s_fix_pitch  = CAM_FIXED_EL_INIT;
    s_fix_off[0] = s_fix_off[1] = s_fix_off[2] = 0.0;
    s_track_swap = 0;
}

/* Native orbit radius: min(150, zoom * substituted camera radius). */
static double cam_orbit_radius(void)
{
    double r = s_cam_zoom * car_collision_radius();
    return r < CAM_RADIUS_MAX ? r : CAM_RADIUS_MAX;
}

static double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Spherical eye about the car. Azimuths use the car's own angle
 * convention (car.h fwd = (-sin,0,cos)), so azimuth yaw+pi sits dead
 * behind the car. */
static void spherical_eye(double out[3], double azimuth, double el,
                          double radius)
{
    double x, y, z, yaw, pitch, roll;
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    double ch = cos(el);
    out[0] = x - sin(azimuth) * radius * ch;
    out[1] = y + radius * sin(el);
    out[2] = z + cos(azimuth) * radius * ch;
}

static void cam_clamp_eye_terrain(double eye[3])
{
    if (terrain_is_loaded()) {
        double g = terrain_height_at(eye[0], eye[2]);
        if (eye[1] < g + 2.0) eye[1] = g + 2.0;
    }
}

/* Driver eye in car model space. The vehicle's own VDF VLOC locator 40 —
 * the left-seat head position, consistent across the shipped car set
 * (car.c cockpit slice; role INFERRED but purchaser-data positions are
 * stable) — wins when present, because the authored VGEO set-16 interior
 * only frames correctly from the authored head point: from the old port
 * estimate the dash/panels collapse to edge-on slivers (the cockpit_probe
 * render proof). The estimate stays as the fallback for VDFs without the
 * locator (turrets, emplacements). */
#define COCKPIT_EYE_UP   1.05
#define COCKPIT_EYE_FWD  0.30
#define COCKPIT_PITCH_UP 0.18 /* source-footage hood/reticle framing */
#define COCKPIT_NEAR     0.25 /* authored roof/pillar/sight geometry (m) */

static void cockpit_camera(CameraView *camera)
{
    double cb[12], eye[3];
    car_basis(cb);
    const double *r = cb, *u = cb + 3, *f = cb + 6;
    double el[3] = { 0.0, COCKPIT_EYE_UP, COCKPIT_EYE_FWD };
    for (int i = 0; i < car_vloc_count(); i++) {
        uint32_t num;
        double vf[12];
        if (car_vloc_get(i, &num, vf) == 0 && num == 40) {
            el[0] = vf[9]; el[1] = vf[10]; el[2] = vf[11];
            break;
        }
    }
    for (int i = 0; i < 3; i++)
        eye[i] = cb[9 + i] + r[i] * el[0] + u[i] * el[1] + f[i] * el[2];
    /*
     * The retail cockpit presents the hood below the screen centre rather
     * than aiming the optical axis down the car's level forward vector.
     * VLOC 40's authored basis is effectively identity, so apply the
     * measured first-person elevation here before user glance offsets.
     */
    double pitched_u[3], pitched_f[3];
    double cp = cos(COCKPIT_PITCH_UP), sp = sin(COCKPIT_PITCH_UP);
    for (int i = 0; i < 3; i++) {
        pitched_u[i] = u[i] * cp - f[i] * sp;
        pitched_f[i] = f[i] * cp + u[i] * sp;
    }
    (void)camera_view_from_basis(camera, eye, r, pitched_u, pitched_f);
}

/* Native tracked point: the followed object's position raised +2.0 m. */
static void cam_tracked_point(double out[3])
{
    double x, y, z;
    car_pose(&x, &y, &z, NULL, NULL, NULL);
    out[0] = x; out[1] = y + CAM_TRACK_UP; out[2] = z;
}

/* Mode 7 (F2): user-adjustable spherical chase. */
static void chase_camera(CameraView *camera)
{
    double x, y, z, yaw, pitch, roll, eye[3], target[3];
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    (void)pitch; (void)roll;
    spherical_eye(eye, yaw + 3.14159265358979323846 + s_cam_yaw_off,
                  s_cam_el, cam_orbit_radius());
    cam_tracked_point(target);
    cam_clamp_eye_terrain(eye);
    (void)camera_view_look_at(camera, eye, target);
}

/* Mode 1: fixed spherical preset, plus the preset's car-local offset. */
static void fixed_camera(CameraView *camera)
{
    double x, y, z, yaw, pitch, roll, eye[3], target[3], cb[12];
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    (void)pitch; (void)roll;
    spherical_eye(eye, yaw + s_fix_yaw, s_fix_pitch, cam_orbit_radius());
    car_basis(cb);
    const double *r = cb, *u = cb + 3, *f = cb + 6;
    for (int i = 0; i < 3; i++)
        eye[i] += r[i] * s_fix_off[0] + u[i] * s_fix_off[1] +
                  f[i] * s_fix_off[2];
    cam_tracked_point(target);
    cam_clamp_eye_terrain(eye);
    (void)camera_view_look_at(camera, eye, target);
}

/* Mode 3 (F10): straight down over the player's x/z; screen-up follows
 * the car's heading (that heading choice is a port presentation
 * decision). The screen basis is built from yaw alone — the tilted
 * body basis would fail the canonical orthogonality check on slopes.
 * right x up = (0,-1,0) holds for the horizontal right/forward pair. */
static void over_camera(CameraView *camera)
{
    double x, y, z, yaw, pitch, roll, eye[3];
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    (void)pitch; (void)roll;
    double r[3] = {  cos(yaw), 0.0, sin(yaw) };
    double f[3] = { -sin(yaw), 0.0, cos(yaw) };
    double down[3] = { 0.0, -1.0, 0.0 };
    eye[0] = x + r[0] * s_over_pan_x + f[0] * s_over_pan_z;
    eye[1] = y + s_over_h;
    eye[2] = z + r[2] * s_over_pan_x + f[2] * s_over_pan_z;
    (void)camera_view_from_basis(camera, eye, r, f, down);
}

/* Mode 4 (F4/F5): two-subject tracker. The eye sits on the far side of
 * endpoint A from endpoint B along their connecting line (rotated by the
 * user's azimuth offset and raised by the elevation), looking at B. F4
 * frames (player, target) — the targeted vehicle stays in view; F5
 * passes the endpoints swapped. With no live target the tracker falls
 * back to the chase framing (the sim's only target source is the nearest
 * hostile, same as pilot_glance_target). */
static void track_camera(CameraView *camera)
{
    int t = combat_nearest_enemy(combat_user_ent());
    double pp[3], tp[3];
    car_pose(&pp[0], &pp[1], &pp[2], NULL, NULL, NULL);
    if (t < 0 || combat_ent_position(t, tp) != 0) {
        chase_camera(camera);
        return;
    }
    const double *a = s_track_swap ? tp : pp;
    const double *b = s_track_swap ? pp : tp;
    double dx = a[0] - b[0], dz = a[2] - b[2];
    if (dx * dx + dz * dz < 1e-6) { chase_camera(camera); return; }
    double az = atan2(-dx, dz) + s_cam_yaw_off;   /* dir = (-sin,cos) */
    double radius = cam_orbit_radius();
    double ch = cos(s_cam_el);
    double eye[3] = { a[0] - sin(az) * radius * ch,
                      a[1] + radius * sin(s_cam_el),
                      a[2] + cos(az) * radius * ch };
    double target[3] = { b[0], b[1] + CAM_TRACK_UP, b[2] };
    cam_clamp_eye_terrain(eye);
    (void)camera_view_look_at(camera, eye, target);
}

/* F6: pointer-driven player-relative direction from the roof point, at
 * the handler's verified 120-degree horizontal FOV (tan(60) half-angle).
 * The Grey arrows steer the direction (the browser stands in for the
 * native pointer axes). The roof eye point reuses the F7 preset's
 * authored offset — INFERRED, the F6 handler's own anchor is unread. */
static void freeeye_camera(CameraView *camera)
{
    double cb[12], eye[3], look[3];
    double x, y, z, yaw, pitch, roll;
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    (void)pitch; (void)roll;
    car_basis(cb);
    const double *u = cb + 3, *f = cb + 6;
    for (int i = 0; i < 3; i++)
        eye[i] = cb[9 + i] + u[i] * 1.5 + f[i] * 0.2;
    double az = yaw + s_free_yaw;
    double cp = cos(s_free_pitch);
    look[0] = eye[0] - sin(az) * cp;
    look[1] = eye[1] + sin(s_free_pitch);
    look[2] = eye[2] + cos(az) * cp;
    if (camera_view_look_at(camera, eye, look) == 0)
        camera->fov_tan_half = CAM_FREEEYE_FOV_TAN;
}

/* Mission cameras have authority while their stack is live. Keep the
 * user's selected view and adjustables untouched underneath so popCam
 * returns to the selected driving view. Returns nonzero for a scripted
 * camera. */
static int drive_camera(CameraView *camera)
{
    if (mission_cam_get(camera))
        return 1;              /* mission camera: hard override, no glance */
    switch (s_drive_view) {
    case DRIVE_VIEW_CHASE:   chase_camera(camera);   break;
    case DRIVE_VIEW_FIXED:   fixed_camera(camera);   break;
    case DRIVE_VIEW_OVER:    over_camera(camera);    break;
    case DRIVE_VIEW_TRACK:   track_camera(camera);   break;
    case DRIVE_VIEW_FREEEYE: freeeye_camera(camera); break;
    case DRIVE_VIEW_COCKPIT:
    default:
        cockpit_camera(camera);
        look_apply(camera);
        break;
    }
    return 0;
}

static double render_dot3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void render_cross3(const double a[3], const double b[3], double out[3])
{
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

static int render_normalize3(double v[3])
{
    double n = sqrt(render_dot3(v, v));
    if (!isfinite(n) || n < 1e-12)
        return 0;
    v[0] /= n;
    v[1] /= n;
    v[2] /= n;
    return 1;
}

/* Nlerp slowly changing tick bases and restore an exact right-handed frame.
 * Basis vectors avoid Euler wrap at +/-pi. */
static void render_basis_lerp(double out[12], const double a[12],
                              const double b[12], double alpha)
{
    double r[3], f[3], u[3];
    for (int i = 0; i < 3; i++) {
        r[i] = a[i] + (b[i] - a[i]) * alpha;
        f[i] = a[6 + i] + (b[6 + i] - a[6 + i]) * alpha;
        out[9 + i] = a[9 + i] + (b[9 + i] - a[9 + i]) * alpha;
    }
    if (!render_normalize3(f))
        memcpy(f, b + 6, sizeof f);
    double rf = render_dot3(r, f);
    for (int i = 0; i < 3; i++)
        r[i] -= f[i] * rf;
    if (!render_normalize3(r))
        memcpy(r, b, sizeof r);
    render_cross3(f, r, u);
    if (!render_normalize3(u))
        memcpy(u, b + 3, sizeof u);
    memcpy(out, r, sizeof r);
    memcpy(out + 3, u, sizeof u);
    memcpy(out + 6, f, sizeof f);
}

static void render_camera_lerp(CameraView *out, const CameraView *a,
                               const CameraView *b, double alpha)
{
    double ab[12], bb[12], ob[12];
    memcpy(ab, a->right, sizeof a->right);
    memcpy(ab + 3, a->up, sizeof a->up);
    memcpy(ab + 6, a->forward, sizeof a->forward);
    memcpy(ab + 9, a->eye, sizeof a->eye);
    memcpy(bb, b->right, sizeof b->right);
    memcpy(bb + 3, b->up, sizeof b->up);
    memcpy(bb + 6, b->forward, sizeof b->forward);
    memcpy(bb + 9, b->eye, sizeof b->eye);
    render_basis_lerp(ob, ab, bb, alpha);
    (void)camera_view_from_basis(out, ob + 9, ob, ob + 3, ob + 6);
    out->fov_tan_half = a->fov_tan_half +
                        (b->fov_tan_half - a->fov_tan_half) * alpha;
}

/* Extrapolate the latency-sensitive road-plane pose (X/Z + heading) from the
 * current tick velocity. Terrain-contact Y/pitch/roll retain the prev/current
 * reconstruction: those constrained signals are not free velocities and raw
 * forward prediction magnifies P09 rough-road attitude correction 7x. This is
 * still zero-latency steering/camera heading, while preserving H-UAT-059's
 * owner-approved vertical/attitude smoothness. */
static void render_basis_predict(double out[12], const double a[12],
                                 const double b[12], double alpha,
                                 double velocity_x, double velocity_z,
                                 double yaw_rate, int velocity_constrained,
                                 int heading_extrapolate)
{
    double interp[12];
    render_basis_lerp(interp, a, b, alpha);

    /* Rotate the interpolated terrain attitude by the extrapolated heading
     * delta. Basis convention: forward=(-sin(yaw), *, cos(yaw)). */
    double yi = atan2(-interp[6], interp[8]);
    double ye = heading_extrapolate
              ? atan2(-b[6], b[8]) + yaw_rate * (alpha * RENDER_TICK_DT)
              : yi;
    double dy = ye - yi;
    while (dy > 3.14159265358979323846) dy -= 6.28318530717958647692;
    while (dy < -3.14159265358979323846) dy += 6.28318530717958647692;
    double c = cos(dy), s = sin(dy);
    for (int col = 0; col < 3; col++) {
        const double *v = interp + col * 3;
        double *o = out + col * 3;
        o[0] = c * v[0] - s * v[2];
        o[1] = v[1];
        o[2] = s * v[0] + c * v[2];
    }
    /* Use car.c's CURRENT body velocity for free (airborne) road-plane
     * translation. Grounded motion is kinematically terrain-constrained, so
     * its body velocity is not a free world-space derivative; retain the
     * proven reconstruction there. Heading still extrapolates from the
     * current yaw rate, so steering response remains current-tick. */
    out[9] = velocity_constrained ? interp[9]
                                  : b[9] + velocity_x * (alpha * RENDER_TICK_DT);
    out[10] = interp[10];
    out[11] = velocity_constrained ? interp[11]
                                   : b[11] + velocity_z * (alpha * RENDER_TICK_DT);
}

static void render_camera_predict(CameraView *out, const CameraView *a,
                                  const CameraView *b, double alpha)
{
    double ab[12], bb[12], ob[12];
    memcpy(ab, a->right, sizeof a->right);
    memcpy(ab + 3, a->up, sizeof a->up);
    memcpy(ab + 6, a->forward, sizeof a->forward);
    memcpy(ab + 9, a->eye, sizeof a->eye);
    memcpy(bb, b->right, sizeof b->right);
    memcpy(bb + 3, b->up, sizeof b->up);
    memcpy(bb + 6, b->forward, sizeof b->forward);
    memcpy(bb + 9, b->eye, sizeof b->eye);
    render_basis_predict(ob, ab, bb, alpha,
                         s_render_velocity_x, s_render_velocity_z,
                         s_render_yaw_rate, s_render_velocity_constrained,
                         s_render_heading_extrapolate);

    /* Keep the predicted camera eye rigid with the predicted player-car base.
     * The heading predictor above rotates the camera axes; leaving its eye on
     * the previous/current chord makes the camera look sideways past the car
     * whenever yaw-rate changes sign (H-UAT-069). Express the interpolated
     * eye offset in the interpolated car frame, then apply that same offset in
     * the already-predicted car frame. This is transform consistency, not a
     * new smoothing constant. */
    if (s_render_heading_extrapolate) {
        double car_interp[12], local[3], eye_offset[3];
        render_basis_lerp(car_interp, s_render_basis_prev,
                          s_render_basis_curr, alpha);
        for (int i = 0; i < 3; i++)
            eye_offset[i] = ob[9 + i] - car_interp[9 + i];
        for (int col = 0; col < 3; col++)
            local[col] = render_dot3(eye_offset, car_interp + col * 3);
        for (int i = 0; i < 3; i++)
            ob[9 + i] = s_render_basis[9 + i] +
                        s_render_basis[i] * local[0] +
                        s_render_basis[3 + i] * local[1] +
                        s_render_basis[6 + i] * local[2];
    }
    (void)camera_view_from_basis(out, ob + 9, ob, ob + 3, ob + 6);
    out->fov_tan_half = a->fov_tan_half +
                        (b->fov_tan_half - a->fov_tan_half) * alpha;
}

static void render_pose_reset(void)
{
    s_render_basis_active = 0;
    s_render_alpha = 1.0;
    s_render_extrapolate = 0;
    s_render_discontinuity = RENDER_CLAMP_RESET;
    s_render_clamp_reason = RENDER_CLAMP_RESET;
    s_render_player_contacts_seen = mission_player_vehicle_contacts();
    s_render_velocity_x = s_render_velocity_z = 0.0;
    s_render_yaw_rate = 0.0;
    s_render_velocity_constrained = 1;
    s_render_heading_extrapolate = 0;
    scene_present_reset();
    if (!car_is_loaded()) {
        s_render_pose_valid = 0;
        return;
    }
    s_render_curr_scripted = drive_camera(&s_render_cam_curr);
    s_render_prev_scripted = s_render_curr_scripted;
    s_render_curr_view = s_render_prev_view = s_drive_view;
    car_basis_live(s_render_basis_curr);
    s_render_cam_prev = s_render_cam_curr;
    memcpy(s_render_basis_prev, s_render_basis_curr,
           sizeof s_render_basis_prev);
    s_render_camera = s_render_cam_curr;
    memcpy(s_render_basis, s_render_basis_curr, sizeof s_render_basis);
    s_render_pose_valid = 1;
    scene_present_capture_tick();
}

static void render_pose_tick(void)
{
    CameraView next_camera;
    double next_basis[12];
    int next_scripted;
    unsigned discontinuity = 0;

    s_render_basis_active = 0;
    s_render_alpha = 1.0;
    if (!s_render_pose_valid) {
        render_pose_reset();
        return;
    }
    next_scripted = drive_camera(&next_camera);
    car_basis_live(next_basis);
    CarStepDiag step_diag;
    car_get_step_diag(&step_diag);
    {
        CarLive live;
        car_get_live(&live);
        double sy = sin(live.yaw), cy = cos(live.yaw);
        s_render_velocity_x = -sy * live.vz + cy * live.vx;
        s_render_velocity_z =  cy * live.vz + sy * live.vx;
        s_render_yaw_rate = live.yaw_rate;
        /* CarLive body velocities include one-sided terrain constraints and
         * are not a stable camera-world derivative on the rough fixture. Keep
         * translation/terrain attitude on the proven reconstruction; the
         * latency-bearing steering heading below is predicted from CURRENT. */
        s_render_velocity_constrained = 1;
        /* Zero-steer terrain attitude is the H-UAT-059 smoothness signal, not
         * input latency. A steering step switches heading to current+yawRate
         * on the exact tick it lands; free flight also predicts heading. */
        s_render_heading_extrapolate = fabs(live.steer) > 1e-6 ||
                                       !s_render_velocity_constrained;
    }

    /* A contact event is authoritative even when positional correction is
     * small. The static-car seam reports its collider directly; mission's
     * physical player/AI count is cumulative, so compare it with the prior
     * presentation snapshot. */
    uint32_t player_contacts = mission_player_vehicle_contacts();
    if ((step_diag.valid && step_diag.collider_index >= 0) ||
        player_contacts != s_render_player_contacts_seen)
        discontinuity |= RENDER_CLAMP_CONTACT;
    s_render_player_contacts_seen = player_contacts;

    /* Contact-free discontinuities (large correction, recovery path not
     * routed through render_pose_reset, etc.) are caught by tick acceleration.
     * Measure X/Z only: rough-road vertical terrain-follow is a constrained
     * signal, not a free impact velocity. Explicit contact events remain the
     * primary clamp, while a contact-free 20 m/s one-tick stop is 400 m/s^2
     * and is still rejected by the 300 m/s^2 backstop. */
    {
        double dvx = (next_basis[9] - s_render_basis_curr[9]) -
                     (s_render_basis_curr[9] - s_render_basis_prev[9]);
        double dvz = (next_basis[11] - s_render_basis_curr[11]) -
                     (s_render_basis_curr[11] - s_render_basis_prev[11]);
        double accel = hypot(dvx, dvz) /
                       (RENDER_TICK_DT * RENDER_TICK_DT);
        if (!isfinite(accel) || accel > RENDER_ACCEL_CLAMP_MPS2)
            discontinuity |= RENDER_CLAMP_ACCEL;
    }

    s_render_cam_prev = s_render_cam_curr;
    memcpy(s_render_basis_prev, s_render_basis_curr,
           sizeof s_render_basis_prev);
    s_render_prev_scripted = s_render_curr_scripted;
    s_render_prev_view = s_render_curr_view;
    s_render_cam_curr = next_camera;
    memcpy(s_render_basis_curr, next_basis, sizeof s_render_basis_curr);
    s_render_curr_scripted = next_scripted;
    s_render_curr_view = s_drive_view;

    /* Authored camera ownership includes pushCam/popCam and camera cuts.
     * Mission cameras remain exact current-tick poses; no prediction crosses
     * either stack edge. Manual view switches are the same hard boundary. */
    if (s_render_prev_scripted || s_render_curr_scripted) {
        discontinuity |= RENDER_CLAMP_SCRIPTED;
        s_render_cam_prev = s_render_cam_curr;
        memcpy(s_render_basis_prev, s_render_basis_curr,
               sizeof s_render_basis_prev);
        s_render_prev_scripted = s_render_curr_scripted;
    }
    if (s_render_prev_view != s_render_curr_view) {
        discontinuity |= RENDER_CLAMP_VIEW;
        s_render_cam_prev = s_render_cam_curr;
        memcpy(s_render_basis_prev, s_render_basis_curr,
               sizeof s_render_basis_prev);
        s_render_prev_view = s_render_curr_view;
    }
    s_render_discontinuity = discontinuity;
    s_render_camera = s_render_cam_curr;
    scene_present_capture_tick();
}

static void render_pose_prepare(double alpha, int extrapolate)
{
    s_render_basis_active = 0;
    if (!s_render_pose_valid)
        render_pose_reset();
    if (!s_render_pose_valid)
        return;
    s_render_alpha = clampd(alpha, 0.0, 1.0);
    s_render_extrapolate = extrapolate ? 1 : 0;
    s_render_clamp_reason = 0;
    if (s_render_curr_scripted || s_render_prev_scripted ||
        s_render_curr_view != s_render_prev_view) {
        s_render_clamp_reason = s_render_discontinuity |
                                RENDER_CLAMP_SCRIPTED;
        s_render_camera = s_render_cam_curr;
        memcpy(s_render_basis, s_render_basis_curr, sizeof s_render_basis);
    } else if (s_render_extrapolate && s_render_discontinuity) {
        /* Hard collision/cut clamp: current tick is the complete presentation
         * truth for this interval. This makes worst-case overshoot exactly zero
         * instead of projecting the pre-impact tick velocity through a wall. */
        s_render_clamp_reason = s_render_discontinuity;
        s_render_camera = s_render_cam_curr;
        memcpy(s_render_basis, s_render_basis_curr, sizeof s_render_basis);
    } else if (s_render_extrapolate) {
        /* current planar pose + measured tick velocity*(alpha*dt); constrained
         * terrain-contact Y/attitude use the smooth reconstruction above. */
        render_basis_predict(s_render_basis, s_render_basis_prev,
                             s_render_basis_curr, s_render_alpha,
                             s_render_velocity_x, s_render_velocity_z,
                             s_render_yaw_rate,
                             s_render_velocity_constrained,
                             s_render_heading_extrapolate);
        /* render_camera_predict anchors its eye in this predicted basis. */
        render_camera_predict(&s_render_camera, &s_render_cam_prev,
                              &s_render_cam_curr, s_render_alpha);
    } else {
        render_camera_lerp(&s_render_camera, &s_render_cam_prev,
                           &s_render_cam_curr, s_render_alpha);
        render_basis_lerp(s_render_basis, s_render_basis_prev,
                          s_render_basis_curr, s_render_alpha);
    }
    s_render_basis_active = 1;
    scene_present_prepare(s_render_alpha, s_render_clamp_reason != 0);
}

static int drive_camera_render(CameraView *camera)
{
    if (s_render_basis_active && s_render_pose_valid) {
        *camera = s_render_camera;
        return s_render_curr_scripted;
    }
    return drive_camera(camera);
}

/*
 * Deterministic camera observability for the P01 native-oracle comparison.
 * This reports the exact camera consumed by both render backends; `scripted`
 * distinguishes mission ownership from the selected manual fallback.
 */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_camera_pose(void)
{
    static char buf[768];
    CameraView camera;
    double target[3];
    int scripted = drive_camera_render(&camera);
    camera_view_target(&camera, target);
    double fth = camera_view_fov_tan_half(&camera);
    double hfov = 2.0 * atan(fth) * 180.0 / acos(-1.0);
    snprintf(buf, sizeof buf,
             "{\"scripted\":%d,\"view\":%d,\"fov_tan_half\":%.9g,"
             "\"eye\":[%.9g,%.9g,%.9g],"
             "\"target\":[%.9g,%.9g,%.9g],"
             "\"right\":[%.9g,%.9g,%.9g],"
             "\"up\":[%.9g,%.9g,%.9g],"
             "\"forward\":[%.9g,%.9g,%.9g],\"hfov_deg\":%.9g}",
             scripted, s_drive_view, fth,
             camera.eye[0], camera.eye[1], camera.eye[2],
             target[0], target[1], target[2],
             camera.right[0], camera.right[1], camera.right[2],
             camera.up[0], camera.up[1], camera.up[2],
             camera.forward[0], camera.forward[1], camera.forward[2], hfov);
    return buf;
}

/* Diagnostic twin for per-SIM-tick traces. The ordinary export above reports
 * the interpolated camera actually consumed by the latest render frame; this
 * one temporarily ignores the presentation override and derives from car.c's
 * authoritative current tick without changing it. */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_camera_tick_pose(void)
{
    static char buf[768];
    CameraView camera;
    double target[3];
    int was_active = s_render_basis_active;
    s_render_basis_active = 0;
    int scripted = drive_camera(&camera);
    s_render_basis_active = was_active;
    camera_view_target(&camera, target);
    double fth = camera_view_fov_tan_half(&camera);
    double hfov = 2.0 * atan(fth) * 180.0 / acos(-1.0);
    snprintf(buf, sizeof buf,
             "{\"scripted\":%d,\"view\":%d,\"fov_tan_half\":%.9g,"
             "\"eye\":[%.9g,%.9g,%.9g],"
             "\"target\":[%.9g,%.9g,%.9g],"
             "\"right\":[%.9g,%.9g,%.9g],"
             "\"up\":[%.9g,%.9g,%.9g],"
             "\"forward\":[%.9g,%.9g,%.9g],\"hfov_deg\":%.9g}",
             scripted, s_drive_view, fth,
             camera.eye[0], camera.eye[1], camera.eye[2],
             target[0], target[1], target[2],
             camera.right[0], camera.right[1], camera.right[2],
             camera.up[0], camera.up[1], camera.up[2],
             camera.forward[0], camera.forward[1], camera.forward[2], hfov);
    return buf;
}

/* View ids are the DRIVE_VIEW_* enum: 0 cockpit, 1 chase, 2 fixed, 3
 * over, 4 track, 5 free-eye. Out-of-range clamps to chase (the legacy
 * two-view contract). The F-key presets go through web_drive_preset. */
EMSCRIPTEN_KEEPALIVE
void web_drive_set_view(int v)
{
    s_drive_view = (v >= 0 && v < DRIVE_VIEW_COUNT) ? v : DRIVE_VIEW_CHASE;
    render_pose_reset();
}

/*
 * PRESET_VIEW_<n> selection (nitro preset handlers 0x435aa0..0x4381b0).
 * The page feeds the purchaser gamekey.map F-key rows here (F1=1, F2=2,
 * F3=3, F4=12, F5=11, F6=4, F7=6, F8=7, F9=8, F10=10, F11=5; preset 9
 * has no stock binding and is reachable only through this export). Every
 * preset first restores the shared adjustables, so a press always lands
 * in the same state. Unknown ids are ignored.
 */
EMSCRIPTEN_KEEPALIVE
void web_drive_preset(int p)
{
    if (p < 1 || p > 12) return;
    drive_camera_reset();
    switch (p) {
    case 1:  s_drive_view = DRIVE_VIEW_COCKPIT; break;
    case 2:  s_drive_view = DRIVE_VIEW_CHASE;   break;
    case 3:  s_drive_view = DRIVE_VIEW_FIXED;   /* yaw 180, pitch 15 */
             break;
    case 4:  s_drive_view = DRIVE_VIEW_FREEEYE; break;
    case 5:  /* native mode 8 resolves an external object (FUN_4ad8e0);
              * unresolved — the tracker is the closest implemented
              * family. */
             s_drive_view = DRIVE_VIEW_TRACK; s_track_swap = 0; break;
    case 6:  s_drive_view = DRIVE_VIEW_FIXED;   /* yaw 0, off (0,1.5,0.2) */
             s_fix_yaw = 0.0; s_fix_pitch = 0.0;
             s_fix_off[1] = 1.5; s_fix_off[2] = 0.2; break;
    case 7:  s_drive_view = DRIVE_VIEW_FIXED;   /* yaw 180, off (0,1.5,-0.25) */
             s_fix_yaw = 3.14159265358979323846; s_fix_pitch = 0.0;
             s_fix_off[1] = 1.5; s_fix_off[2] = -0.25; break;
    case 8:  s_drive_view = DRIVE_VIEW_FIXED;   /* yaw 150, pitch 15 */
             s_fix_yaw = 2.6179938779914944; break;
    case 9:  s_drive_view = DRIVE_VIEW_FIXED;   /* yaw 30, pitch 15 */
             s_fix_yaw = 0.5235987755982988; break;
    case 10: s_drive_view = DRIVE_VIEW_OVER;    break;
    case 11: s_drive_view = DRIVE_VIEW_TRACK; s_track_swap = 1; break;
    case 12: s_drive_view = DRIVE_VIEW_TRACK; s_track_swap = 0; break;
    }
    render_pose_reset();
}

/*
 * Manual camera observability for probes: the selected view plus every
 * adjustable, so clamp and control-routing checks read state directly
 * instead of inferring it from poses. `scripted` reports mission-camera
 * authority (the selection underneath is preserved).
 */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_camera_state(void)
{
    static char buf[384];
    snprintf(buf, sizeof buf,
             "{\"view\":%d,\"zoom\":%.4f,\"yawOff\":%.4f,\"el\":%.4f,"
             "\"overH\":%.3f,\"panX\":%.3f,\"panZ\":%.3f,"
             "\"freeYaw\":%.4f,\"freePitch\":%.4f,\"trackSwap\":%d,"
             "\"scripted\":%d}",
             s_drive_view, s_cam_zoom, s_cam_yaw_off, s_cam_el,
             s_over_h, s_over_pan_x, s_over_pan_z,
             s_free_yaw, s_free_pitch, s_track_swap,
             mission_cam_active());
    return buf;
}

/*
 * Pilot glance + mirror observability. `rearview` is the raw toggle;
 * `mirrorOn` is the effective presentation state (cockpit view, no mission
 * camera). The resolved glance itself is visible through
 * web_drive_camera_pose / web_gpu_camera, which carry the look offset.
 */
EMSCRIPTEN_KEEPALIVE
const char *web_look_state(void)
{
    static char buf[128];
    CameraView rv;
    snprintf(buf, sizeof buf,
             "{\"x\":%d,\"y\":%d,\"target\":%d,\"rearview\":%d,"
             "\"mirrorOn\":%d}",
             s_look_x, s_look_y, s_look_target, s_rearview,
             rearview_derive(&rv));
    return buf;
}

EMSCRIPTEN_KEEPALIVE
int web_rearview_active(void)
{
    CameraView rv;
    return rearview_derive(&rv);
}

/*
 * GPU seam: the same derived rearview basis web_drive_render composites in
 * software — 12 contiguous doubles (eye,right,up,forward), NULL while the
 * mirror is not presenting. The GPU path consumes THIS camera and never
 * reconstructs its own, exactly like web_gpu_camera for the primary view.
 */
EMSCRIPTEN_KEEPALIVE
double *web_gpu_rearview_camera(void)
{
    static CameraView rv;
    return rearview_derive(&rv) ? rv.eye : NULL;
}

/* Mirror destination rect in the 640x480 frame: x, y, w, h. */
EMSCRIPTEN_KEEPALIVE
int *web_rearview_rect(void)
{
    static int rect[4] = { MIRROR_X, MIRROR_Y, MIRROR_W, MIRROR_H };
    return rect;
}

/*
 * Test/debug hook: snap the car to (x, z) preserving its heading. The
 * nav-goal gate uses it to satisfy the script's user-arrival gate on
 * demand — P01's machines poll that gate every tick for the whole
 * mission (observed: from tick 0 past mission end), so no NATURAL clear
 * exists to assert against. Not wired to any page control.
 */
EMSCRIPTEN_KEEPALIVE
void web_drive_teleport(double x, double z)
{
    if (!car_is_loaded()) return;
    double cx, cy, cz, yaw, pitch, roll;
    car_pose(&cx, &cy, &cz, &yaw, &pitch, &roll);
    car_place(x, z, yaw);
    render_pose_reset();
}

/* Test/debug counterpart to web_drive_teleport: establish a reproducible
 * off-structure approach with an explicit heading and entry speed. It is not
 * wired to the page. Subsequent movement still goes through web_drive_step's
 * ordinary CarInput, collider table and mission-owned world. */
EMSCRIPTEN_KEEPALIVE
void web_drive_probe_place(double x, double z, double yaw, double speed)
{
    if (!car_is_loaded()) return;
    car_set_scripted_pose(x, z, yaw, speed);
    mission_set_car(x, z, yaw);
    drive_pose_string();
    render_pose_reset();
}

EMSCRIPTEN_KEEPALIVE
int web_drive_view(void)
{
    return mission_cam_active() ? DRIVE_VIEW_CHASE : s_drive_view;
}

/*
 * Car basis columns from yaw/pitch/roll (see the drive-mode header note):
 * yaw about +Y (fwd=(-sin,cos) per car.h), then pitch (nose-up +) about
 * right, then roll (right-side-up +) about forward. out12 = right[3],
 * up[3], forward[3], pos[3] — scene.c D4 basis-columns convention.
 */
static void car_basis_live(double out12[12])
{
    double x, y, z, yaw, pitch, roll;
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);

    double r[3] = {  cos(yaw), 0.0, sin(yaw) };
    double u[3] = {  0.0,      1.0, 0.0      };
    double f[3] = { -sin(yaw), 0.0, cos(yaw) };
    /* pitch about r: f' = f·cosP + u·sinP ; u' = u·cosP - f·sinP */
    double cp = cos(pitch), sp = sin(pitch);
    double f1[3], u1[3];
    for (int i = 0; i < 3; i++) {
        f1[i] = f[i] * cp + u[i] * sp;
        u1[i] = u[i] * cp - f[i] * sp;
    }
    /* roll about f1: u' = u1·cosR - r·sinR ; r' = r·cosR + u1·sinR */
    double cr = cos(roll), sr = sin(roll);
    double u2[3], r2[3];
    for (int i = 0; i < 3; i++) {
        u2[i] = u1[i] * cr - r[i] * sr;
        r2[i] = r[i] * cr + u1[i] * sr;
    }
    for (int i = 0; i < 3; i++) {
        out12[i]     = r2[i];
        out12[3 + i] = u2[i];
        out12[6 + i] = f1[i];
        out12[9 + i] = i == 0 ? x : i == 1 ? y : z;
    }
}

static void car_basis(double out12[12])
{
    if (s_render_basis_active) {
        memcpy(out12, s_render_basis, sizeof s_render_basis);
        return;
    }
    car_basis_live(out12);
}

/* Rotate v about unit axis a by t radians (Rodrigues). Presentation math
 * for the glance offsets; doubles like the rest of the camera path. */
static void rot_axis(double v[3], const double a[3], double t)
{
    double c = cos(t), s = sin(t);
    double d = a[0] * v[0] + a[1] * v[1] + a[2] * v[2];
    double x = v[0], y = v[1], z = v[2];
    v[0] = x * c + (a[1] * z - a[2] * y) * s + a[0] * d * (1.0 - c);
    v[1] = y * c + (a[2] * x - a[0] * z) * s + a[1] * d * (1.0 - c);
    v[2] = z * c + (a[0] * y - a[1] * x) * s + a[2] * d * (1.0 - c);
}

/*
 * Apply the held pilot glance to an already-derived driving camera. The eye
 * never moves; only the basis rotates about it, so car pose/yaw are
 * structurally untouched. Insert aims the view at the current target while
 * held (nearest hostile, the only target source the sim has today); with no
 * target it is the reset-to-ahead gesture. Called only on the cockpit
 * fallback — drive_camera returns the mission camera untouched, and the
 * other manual views read the same physical keys as their own native
 * actions (track_yaw/pitch, overview_x/z, free-eye direction).
 */
static void look_apply(CameraView *cam)
{
    if (s_look_target) {
        int t = combat_nearest_enemy(combat_user_ent());
        double p[3];
        if (t >= 0 && combat_ent_position(t, p) == 0)
            (void)camera_view_look_at(cam, cam->eye, p);
        return;
    }
    /* About the camera's up axis, +90° maps forward onto right: positive
     * s_look_x looks out the RIGHT window. About the right axis the same
     * sign convention pitches down, so pitch rotates by the negated
     * magnitude. */
    double yaw = s_look_x * LOOK_YAW_SIDE;
    double pitch = s_look_y > 0 ? LOOK_PITCH_UP
                 : s_look_y < 0 ? -LOOK_PITCH_DOWN : 0.0;
    if (yaw == 0.0 && pitch == 0.0)
        return;
    if (yaw != 0.0) {
        rot_axis(cam->right, cam->up, yaw);
        rot_axis(cam->forward, cam->up, yaw);
    }
    if (pitch != 0.0) {
        rot_axis(cam->up, cam->right, -pitch);
        rot_axis(cam->forward, cam->right, -pitch);
    }
}

/*
 * Rearview mirror camera: a SECOND derived CameraView — the cockpit eye
 * with the car basis turned 180° about its own up axis (right=-r, up=u,
 * forward=-f stays right-handed, so every renderer consumes it through the
 * canonical path). Presentation only: nothing here feeds physics or
 * targeting. Returns 0 unless the mirror is toggled on AND the cockpit
 * view is actually presenting (chase and mission cameras never show it).
 */
static int rearview_derive(CameraView *out)
{
    if (!s_rearview || s_drive_view != DRIVE_VIEW_COCKPIT ||
        !car_is_loaded() || mission_cam_active())
        return 0;
    double cb[12], eye[3], mr[3], mf[3];
    car_basis(cb);
    const double *r = cb, *u = cb + 3, *f = cb + 6;
    for (int i = 0; i < 3; i++) {
        eye[i] = cb[9 + i] + u[i] * COCKPIT_EYE_UP + f[i] * COCKPIT_EYE_FWD;
        mr[i] = -r[i];
        mf[i] = -f[i];
    }
    return camera_view_from_basis(out, eye, mr, u, mf) == 0 ? 1 : 0;
}



/* ZH*-only sequence over the manifest's archive order (ZS* role is
 * UNRESOLVED — see the state block above). */
static int sidearm_seq_index(int side, int seq)
{
    int n = 0;
    for (int i = 0; i < hud_sidearm_frame_count(side); i++) {
        const char *nm = hud_sidearm_frame_name(side, i);
        if (nm && strncmp(nm, "ZH", 2) == 0 && n++ == seq)
            return i;
    }
    return -1;
}

static int sidearm_seq_len(int side)
{
    int n = 0;
    for (int i = 0; i < hud_sidearm_frame_count(side); i++) {
        const char *nm = hud_sidearm_frame_name(side, i);
        if (nm && strncmp(nm, "ZH", 2) == 0)
            n++;
    }
    return n;
}

/*
 * Sidearm art placement (DECISION, presentation — no authored screen
 * anchor is reversed; retail footage shows the pistol rising from the
 * lower side of the cockpit toward the glanced window). Sprite rule per
 * hud.c D7: 0xFF is transparent.
 */
#define SIDEARM_ART_SIDE_INSET 8      /* px from the glanced screen edge */
/* 256x128 ZH frames sit lower on the 640x480 FB. 96 left the sprite mid-
 * frame with the hand reading as "near the roof"; retail rises from the
 * lower door sill, so pin the sprite near the bottom edge. */
#define SIDEARM_ART_BOTTOM     4      /* px above the frame bottom       */
static void sidearm_art_draw(uint8_t *fb, int w, int h)
{
    int side = s_sidearm_anim >= 0 ? s_sidearm_anim_side
             : s_look_x < 0 ? HUD_SIDEARM_LEFT : HUD_SIDEARM_RIGHT;
    int mi = sidearm_seq_index(side,
                               s_sidearm_anim >= 0 ? s_sidearm_anim : 0);
    if (mi < 0)
        return;
    int fw = 0, fh = 0;
    const uint8_t *pix = hud_sidearm_frame(side, mi, &fw, &fh);
    if (!pix || fw <= 0 || fh <= 0)
        return;
    int ox = side == HUD_SIDEARM_LEFT ? SIDEARM_ART_SIDE_INSET
                                      : w - fw - SIDEARM_ART_SIDE_INSET;
    int oy = h - fh - SIDEARM_ART_BOTTOM;
    for (int y = 0; y < fh; y++) {
        int dy = oy + y;
        if (dy < 0 || dy >= h)
            continue;
        for (int x = 0; x < fw; x++) {
            int dx = ox + x;
            uint8_t v = pix[y * fw + x];
            if (dx < 0 || dx >= w || v == 0xFF)
                continue;
            fb[dy * w + dx] = v;
        }
    }
}

static void fire_sound_play(const char *sound)
{
    if (sound && sound[0] && strcasecmp(sound, "null") != 0)
        (void)sound_play(sound);
}

static void player_weapon_sound_play(void)
{
    /* One sample per hardpoint that actually fired on this trigger. Linked
     * weapons retain independent cooldowns, so armed alone is not enough. */
    int any = 0;
    for (int i = 0; i < combat_player_weapon_count(); i++) {
        if (!combat_player_weapon_fired(i))
            continue;
        CarWeaponInfo wi;
        int source = combat_player_weapon_source_at(i);
        if (source >= 0 && car_weapon_get(source, &wi) == 0) {
            fire_sound_play(wi.sound);
            any = 1;
        }
    }
    if (!any) {
        CarWeaponInfo wi;
        int source = combat_player_weapon_source();
        if (source >= 0 && car_weapon_get(source, &wi) == 0)
            fire_sound_play(wi.sound);
    }
}

static void damage_flash_tick(void)
{
    if (s_damage_flash > 0)
        s_damage_flash--;
}

/*
 * H-UAT-013: incoming damage carries a DIRECTION when the real hit event
 * names a live attacker (combat_who_attacked + its resolved position — no
 * invented data). The flash then covers only the screen edge facing the
 * shooter's live bearing: ahead = top, right = right, behind = bottom,
 * left = left, recomputed each frame against the current camera so the cue
 * tracks the shooter while the car turns. Attacker-less damage (a hard
 * landing, an unresolved attacker) keeps the old full-border flash.
 */
static void damage_flash_put(uint8_t *fb, uint8_t *mask, int i, uint8_t red)
{
    fb[i] = red;
    if (mask)
        mask[i] = 0xff;
}

static void damage_flash_render(uint8_t *fb, uint8_t *mask, int w, int h,
                                const CameraView *cam)
{
    if (s_damage_flash <= 0)
        return;
    uint8_t red = raster_rgb_to_index(255, 24, 24);
    int thickness = s_damage_flash > 3 ? 3 : 1;
    if (s_damage_dir_valid && cam) {
        double d[3] = { s_damage_src[0] - cam->eye[0], 0.0,
                        s_damage_src[2] - cam->eye[2] };
        double sx = d[0] * cam->right[0] + d[2] * cam->right[2];
        double sz = d[0] * cam->forward[0] + d[2] * cam->forward[2];
        if (sx * sx + sz * sz > 1e-6) {
            const double pi = 3.14159265358979323846;
            double ang = atan2(sx, sz);         /* 0 ahead, + right */
            int edge = fabs(ang) <= pi * 0.25 ? 0             /* top    */
                     : fabs(ang) >= pi * 0.75 ? 1             /* bottom */
                     : ang > 0.0              ? 2 : 3;        /* right/left */
            int th = thickness * 2;             /* one edge: keep it visible */
            if (edge <= 1) {
                int y0 = edge == 0 ? 0 : h - th;
                for (int y = y0; y < y0 + th; y++)
                    for (int x = w / 4; x < w - w / 4; x++)
                        damage_flash_put(fb, mask, y * w + x, red);
            } else {
                int x0 = edge == 2 ? w - th : 0;
                for (int y = h / 4; y < h - h / 4; y++)
                    for (int x = x0; x < x0 + th; x++)
                        damage_flash_put(fb, mask, y * w + x, red);
            }
            return;
        }
    }
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (x < thickness || x >= w - thickness ||
                y < thickness || y >= h - thickness)
                damage_flash_put(fb, mask, y * w + x, red);
}

/* Queue one deterministic real hit for the next drive tick. This preserves
 * the product ordering (mission/combat first, hp-delta observation second)
 * while giving Node/browser probes a reproducible attacker bearing. */
EMSCRIPTEN_KEEPALIVE
int web_damage_probe_hit(int attacker, int damage)
{
    double source[3];
    int user = combat_user_ent();
    if (!car_is_loaded() || user < 0 || attacker < 0 || attacker == user ||
        damage <= 0 || combat_ent_position(attacker, source) != 0)
        return -1;
    s_damage_probe_attacker = attacker;
    s_damage_probe_amount = damage;
    return 0;
}

EMSCRIPTEN_KEEPALIVE
void web_drive_step(void)
{
    /* A WebGPU frame may leave the interpolated basis armed for its export
     * calls. Simulation always resumes from car.c's authoritative state. */
    s_render_basis_active = 0;
    s_render_alpha = 1.0;
    if (!car_is_loaded()) return;
    if (s_colliders_scene_gen != scene_gate_generation())
        build_colliders(s_colliders_exclude_vehicles);

    InputState s;
    input_poll(&s);
    if (s_tape_on) {
        /* Deterministic replay: the tape's masks replace the polled
         * keyboard snapshot wholesale; everything downstream (steer ramp,
         * fire, weapons, glance, skip edges) consumes them unchanged. */
        for (int b = 0; b < INPUT_BTN_COUNT; b++) {
            s.held[b]    = ((s_tape_held >> b) & 1u) != 0;
            s.pressed[b] = ((s_tape_pressed >> b) & 1u) != 0;
        }
        s.quit = false;
    }
    {
        /* Recorder seam: the masks this tick actually consumed, readable
         * via web_drive_input_held/pressed after web_drive_step returns. */
        uint32_t h = 0, p = 0;
        for (int b = 0; b < INPUT_BTN_COUNT; b++) {
            if (s.held[b])    h |= 1u << b;
            if (s.pressed[b]) p |= 1u << b;
        }
        s_last_held = h;
        s_last_pressed = p;
    }
    /*
     * Camera controls resolve per active view, mirroring the stock
     * input.map's shared bindings: the Grey arrows carry pilot_glance_*
     * in the cockpit, track_yaw/track_pitch in the chase/tracker,
     * overview_x/overview_z overhead, and the free-eye direction in
     * the F6 view; GreyPageDown/GreyPageUp carry track_distance,
     * overview_zoom and zoom_factor, and GreyEnd is zoom_factor_reset.
     * Presentation only: the mission camera ignores all of it
     * (drive_camera's hard override) and nothing here reaches the car
     * sim. The mirror toggle is accepted in the cockpit view only.
     */
    int cam_ax = (s.held[INPUT_BTN_LOOK_RIGHT] ? 1 : 0) -
                 (s.held[INPUT_BTN_LOOK_LEFT]  ? 1 : 0);
    int cam_ay = (s.held[INPUT_BTN_LOOK_UP]   ? 1 : 0) -
                 (s.held[INPUT_BTN_LOOK_DOWN] ? 1 : 0);
    int cam_range = (s.held[INPUT_BTN_CAM_RANGE_PLUS]  ? 1 : 0) -
                    (s.held[INPUT_BTN_CAM_RANGE_MINUS] ? 1 : 0);
    s_look_x = s_look_y = s_look_target = 0;
    switch (s_drive_view) {
    case DRIVE_VIEW_COCKPIT:
        s_look_x = cam_ax;
        s_look_y = cam_ay;
        s_look_target = s.held[INPUT_BTN_LOOK_TARGET] ? 1 : 0;
        break;
    case DRIVE_VIEW_CHASE:
    case DRIVE_VIEW_TRACK:
        s_cam_yaw_off = clampd(s_cam_yaw_off + cam_ax * CAM_YAW_RATE,
                               -3.14159265358979323846,
                               3.14159265358979323846);
        s_cam_el  = clampd(s_cam_el + cam_ay * CAM_EL_RATE,
                           CAM_EL_MIN, CAM_EL_MAX);
        s_cam_zoom = clampd(s_cam_zoom + cam_range * CAM_ZOOM_RATE,
                            CAM_ZOOM_MIN, CAM_ZOOM_MAX);
        if (s.pressed[INPUT_BTN_CAM_RANGE_RESET]) {
            s_cam_yaw_off = 0.0;
            s_cam_el  = CAM_CHASE_EL_INIT;
            s_cam_zoom = CAM_ZOOM_RESET;
        }
        break;
    case DRIVE_VIEW_OVER:
        /* Pan rate scales with height so a low camera pans fine and a
         * high one usefully; overview_zoom multiplies the height. */
        s_over_pan_x = clampd(s_over_pan_x + cam_ax * (s_over_h * 0.02),
                              -CAM_PAN_MAX, CAM_PAN_MAX);
        s_over_pan_z = clampd(s_over_pan_z + cam_ay * (s_over_h * 0.02),
                              -CAM_PAN_MAX, CAM_PAN_MAX);
        if (cam_range > 0) s_over_h *= 1.03;
        else if (cam_range < 0) s_over_h /= 1.03;
        s_over_h = clampd(s_over_h, CAM_OVER_MIN, CAM_OVER_MAX);
        if (s.pressed[INPUT_BTN_CAM_RANGE_RESET]) {
            s_over_pan_x = s_over_pan_z = 0.0;
            s_over_h = CAM_OVER_INIT;
        }
        break;
    case DRIVE_VIEW_FREEEYE:
        s_free_yaw = clampd(s_free_yaw + cam_ax * CAM_YAW_RATE,
                            -3.14159265358979323846,
                            3.14159265358979323846);
        s_free_pitch = clampd(s_free_pitch + cam_ay * CAM_EL_RATE,
                              -1.45, 1.45);
        if (s.pressed[INPUT_BTN_CAM_RANGE_RESET])
            s_free_yaw = s_free_pitch = 0.0;
        break;
    default:   /* DRIVE_VIEW_FIXED: the presets are fixed by definition */
        break;
    }
    if (s.pressed[INPUT_BTN_REARVIEW] && s_drive_view == DRIVE_VIEW_COCKPIT)
        s_rearview = !s_rearview;
    /* Retail keyboard.map: M = show_map, N = show_notepad. Mutually
     * exclusive paper surfaces; second press of the same key puts them
     * away. Assets come from paper_load_mission at drive start. */
    if (s.pressed[INPUT_BTN_MAP])
        paper_toggle(PAPER_MAP);
    if (s.pressed[INPUT_BTN_NOTEPAD])
        paper_toggle(PAPER_NOTEPAD);
    /* Title card advances on the sim clock; fire/map/notepad dismisses. */
    if (paper_title_active()) {
        paper_title_tick();
        if (s.pressed[INPUT_BTN_FIRE] || s.pressed[INPUT_BTN_MAP] ||
            s.pressed[INPUT_BTN_NOTEPAD] || s.pressed[INPUT_BTN_PAUSE])
            paper_dismiss_title();
    }
    /* Sidearm sequence advance: one archive-order frame per fixed tick. */
    if (s_sidearm_anim >= 0 &&
        ++s_sidearm_anim >= sidearm_seq_len(s_sidearm_anim_side))
        s_sidearm_anim = -1;
    if (mission_movie_pending()) {
        /* The page's Smacker player owns time until it acknowledges the
         * queued clip. Drain input without advancing car or mission state. */
        (void)input_steer_axis(false, false, 1.0 / 20.0);
        s_an_fire_prev = s_analog_on && s_an_fire;
        drive_pose_string();
        return;
    }
    damage_flash_tick();
    for (int i = 0; i < 8; i++)
        if (s.pressed[INPUT_BTN_WEAPON_1 + i])
            (void)combat_player_weapon_select(i);
    /* Stock keyboard.map: weapon_cycle = Enter, weapon_link = L.
     * Enter highlights one hardpoint; L toggles its same-manual-class,
     * same-facing fixed peers. Turrets and singletons remain solo. */
    if (s.pressed[INPUT_BTN_WEAPON_CYCLE])
        (void)combat_player_weapon_cycle(+1);
    if (s.pressed[INPUT_BTN_WEAPON_LINK])
        (void)combat_player_weapon_link();
    int user = combat_user_ent();
    int hp_before = user >= 0 ? combat_hp(user) : -1;
    int analog_fire_edge = s_analog_on && s_an_fire && !s_an_fire_prev;
    int skip_pressed = s_analog_on ? analog_fire_edge
                                   : s.pressed[INPUT_BTN_FIRE];
    int mission_ticked = 0;
    int physics_stepped = 0;
    int scripted = mission_scripted_car(NULL, NULL, NULL);
    int cutscene = mission_cam_active();

    /*
     * A scripted mission has not issued its first mover/camera actions until
     * its first fixed tick. Prime that tick against the published spawn before
     * deciding who owns controls; otherwise the first browser tick always
     * leaks keyboard input into the car before P01's intro takes authority.
     */
    if (!scripted && !cutscene && mission_ticks() == 0 &&
        mission_player_object()[0]) {
        double x, y, z, yaw, pitch, roll;
        car_pose(&x, &y, &z, &yaw, &pitch, &roll);
        mission_set_car(x, z, yaw);
        mission_set_skip(skip_pressed);
        mission_tick();
        mission_ticked = 1;
        cutscene = mission_cam_active();
    }

    if (cutscene || mission_ticked) {
        /*
         * The live cutscene camera owns this input snapshot. Space is the
         * original isKeypress skip edge, not a simultaneous weapon shot. The
         * FSM mover advances first, then its fresh target is applied even if
         * this tick runs popCam. That final placement precedes player control.
         */
        if (!mission_ticked) {
            double x, y, z, yaw, pitch, roll;
            car_pose(&x, &y, &z, &yaw, &pitch, &roll);
            mission_set_car(x, z, yaw);
            mission_set_skip(skip_pressed);
            mission_tick();
        }

        double p[3], yaw, speed;
        if (mission_scripted_car(p, &yaw, &speed)) {
            car_set_scripted_pose(p[0], p[2], yaw, speed);
        } else if (mission_cam_active()) {
            /* A camera-only cutscene suppresses controls but lets the real car
             * coast under ordinary zero-input physics. */
            CarInput zero = {0};
            car_step(&zero);
            physics_stepped = 1;
        }
        /* If this snapshot ended the camera-owned intro, every action the
         * intro consumed must be released before gameplay can observe it.
         * Space is both skip and fire, but throttle/steering/etc. obey the
         * same handoff rule rather than leaking their held state too. */
        if (cutscene && !mission_cam_active())
            input_require_release();

        /* Do not accumulate a full steering lock behind a cutscene. */
        (void)input_steer_axis(false, false, 1.0 / 20.0);
    } else {
        CarInput in = {0};   /* zero-init: M4 edge fields (SimPort) */
        in.reverse = s.pressed[INPUT_BTN_REVERSE] ? 1 : 0;
        if (s_analog_on) {
            in.throttle = s_an_thr > 0.0f ? s_an_thr : 0.0f;
            in.left     = s_an_left;
            in.right    = s_an_right;
            in.brake    = s_an_thr < 0.0f ? -s_an_thr : 0.0f;
        } else {
            in.throttle = s.held[INPUT_BTN_UP]   ? 1 : 0;
            in.brake    = s.held[INPUT_BTN_DOWN] ? 1 : 0;

            /* The original integrates digital steering hold time:
             * steer=+/-sqrt(held/3), capped just below full lock (input.h
             * and ghidra-physics.md Q17). */
            double steer = input_steer_axis(s.held[INPUT_BTN_LEFT],
                                            s.held[INPUT_BTN_RIGHT],
                                            1.0 / 20.0);
            in.left  = steer > 0.0 ? (float)steer : 0.0f;
            in.right = steer < 0.0 ? (float)-steer : 0.0f;
        }
        /* Stock drivetrain actions already implemented by car_step. They
         * remain behind the same cutscene/movie ownership gate as steering
         * and throttle, so an edge pressed during a scripted camera cannot
         * leak into player control after handoff. */
        in.shift_down = s.pressed[INPUT_BTN_SHIFT_DOWN];
        in.shift_up   = s.pressed[INPUT_BTN_SHIFT_UP];
        in.e_brake   = s.held[INPUT_BTN_E_BRAKE];
        in.reverse   = s.pressed[INPUT_BTN_REVERSE];
        car_step(&in);

        /* H-UAT-067c: publish THIS tick's settled car pose before combat
         * consumes held fire. The old ordering fired first and did this only
         * below, so every projectile/muzzle FX started from the previous
         * 20 Hz pose — visibly behind a moving car. */
        {
            double x, y, z, yaw, pitch, roll;
            car_pose(&x, &y, &z, &yaw, &pitch, &roll);
            mission_set_car(x, z, yaw);
        }

        /* A horn is a latched keyboard edge and a presentation-only one-shot.
         * The sample follows the same loaded-car engsnd.dat row as the loop. */
        if (s.pressed[INPUT_BTN_HORN]) {
            const SoundEngRow *row = drive_sound_row();
            if (row && strcmp(row->horn_wav, "NONE") != 0)
                (void)sound_play(row->horn_wav);
        }
        physics_stepped = 1;

        /* Asset-rate cooldown turns a held trigger into autofire. Fire before
         * mission_tick so hp/FSM consequences remain on this fixed tick;
         * combat.c records the deterministic presentation event. */
        if (s_analog_on ? s_an_fire : s.held[INPUT_BTN_FIRE]) {
            int hit = -1;
            if (s_look_x != 0 && combat_player_sidearm_present()) {
                /* Glance-contextual generic fire (combat.h D-C14): the
                 * pilot's .45 out the glanced side window, with its own
                 * parsed ammo/cadence and its own muzzle. The selected
                 * hardpoint is never mutated and resumes the moment the
                 * glance releases — explicit selection stays explicit. */
                double cb[12];
                car_basis(cb);
                if (combat_player_sidearm_fire(&hit, cb[0] * s_look_x,
                                               cb[2] * s_look_x)) {
                    if (hit >= 0) s_fire_hit = hit;
                    fire_sound_play(s_sidearm_wi.sound);
                    s_sidearm_anim = 0;     /* play the ZH* shot sequence */
                    s_sidearm_anim_side = s_look_x < 0 ? HUD_SIDEARM_LEFT
                                                       : HUD_SIDEARM_RIGHT;
                }
            } else if (combat_player_fire(&hit)) {
                if (hit >= 0) s_fire_hit = hit;
                player_weapon_sound_play();
            }
        }

        mission_set_skip(skip_pressed);
        mission_tick();
        if (mission_user_teleported()) {
            double p[3], yaw, speed;
            if (mission_scripted_car(p, &yaw, &speed))
                car_set_scripted_pose(p[0], p[2], yaw, speed);
        }
    }

    s_an_fire_prev = s_analog_on && s_an_fire;
    s_drive_ticks++;
    render_pose_tick();

    /* Shell debrief stats: web-side because car.c has no odometer. */
    {
        double speed = car_speed();
        s_odom += speed * 0.05;
        if (speed > s_top_speed) s_top_speed = speed;
    }

    /* Only real physics can produce a landing event. */
    if (physics_stepped) {
        int hard = 0;
        double impact = 0.0;
        car_landing_events(&hard, NULL, &impact);
        combat_landing_events(hard, impact);
        /* Presentation consumes the same monotone event source independently
         * from combat. car_place() resets the producer on recovery/teleport,
         * so a lower value is a re-baseline, never a new landing. */
        if (hard < s_landing_sound_seen)
            s_landing_sound_seen = hard;
        while (s_landing_sound_seen < hard) {
            s_landing_sound_seen++;
            (void)sound_play(DRIVE_LANDING_WAV);
        }
        drive_recover_record();
    }
    if (s_damage_probe_attacker >= 0) {
        int attacker = s_damage_probe_attacker;
        int damage = s_damage_probe_amount;
        s_damage_probe_attacker = -1;
        s_damage_probe_amount = 0;
        if (user >= 0)
            combat_shot(attacker, user, damage);
    }
    if (user >= 0 && hp_before >= 0 && combat_hp(user) < hp_before) {
        s_damage_flash = 8;
        /* H-UAT-013: the hit event's attacker pulse is still live here
         * (cleared at the start of the NEXT combat_tick), so the direction
         * comes from the real event, never inferred. -1 (landing damage)
         * or an unresolvable attacker keeps the non-directional border. */
        int who = combat_who_attacked(user);
        s_damage_dir_valid =
            who >= 0 && combat_ent_position(who, s_damage_src) == 0;
    }

    drive_pose_string();
}

/*
 * The arena objective readout, drawn over the cockpit.
 *
 * Two lines, because hud_set_text() REPLACES the composed status line
 * rather than adding to it — dropping the speed to make room for the lap
 * counter would be a downgrade, so the speed is re-composed here.
 *
 * Line 1 is the original's own counter — "%d of %d Laps Completed" for a
 * race, "%d of %d Flag Captures" for a capture map, both verbatim from
 * nitro.exe — plus the speed and, when a clock is armed, the original's
 * "%02dH:%02dM:%02dS Minutes Remaining".
 *
 * Line 2 is INVENTED in both modes and is a PORT AID, not a
 * transcription. On a race it exists because the mission's
 * `check1..checkN` markers are ODEF class-1 records with no shipped mesh,
 * so this renderer draws NOTHING at a checkpoint and the course is
 * literally invisible. On a capture map the objective IS drawn — the
 * a1flagN records are class 2/3 and resolve through <label>.sdf like any
 * other static — but the maps are 2 to 4.5 km across, so the flag is
 * beyond the draw distance for most of the run and this port has no map
 * or compass to find it with. The wording ("TAKE", "BASE") reuses the
 * original's own vocabulary; the LINE is ours (coverage-ledger.md §6).
 *
 * Scripted trips take the mission_nav_goal branch inside (PORT
 * GUIDANCE, derived from the script's own user-arrival gate); only a
 * trip with no live user gate keeps the stock status line.
 */
static void drive_objective_line(double spd)
{
    MissionObjectiveState ob;
    if (!mission_objective_state(&ob)) {
        /*
         * TRIP (scripted): the FSM owns the objectives and the arena
         * controller deliberately stays out (mission.h D-O1), so there is
         * no lap/flag/kill readout here. The one objective the script
         * polls the PLAYER about is arrival — mission_nav_goal() is the
         * first unsatisfied isWithinNav/isWithinSqNav gate evaluated for
         * the user entity this tick, with the BINARY-VERIFIED node-0
         * anchor. The cue is derived entirely from that mission state:
         * no minimap, no radar, nothing the script did not ask.
         *
         * PORT GUIDANCE, not original HUD: nitro.exe has no trip
         * objective arrow; the wording is ours (same standing as the
         * arena line-2 port aids below). "" when no user gate is live
         * keeps the stock status line.
         */
        /* H-UAT-014: the persistent line table replaces the one-tick
         * mission_nav_goal here — that candidate flips targets when two
         * machines poll different gates on alternating ticks. Up to two
         * stable rows: escort duties name the mission's own entity. */
        MissionObjectiveLine ln[2];
        int nl = mission_objective_lines(ln, 2);
        if (nl <= 0) {
            hud_set_text("");
            return;
        }
        double x, y, z, yaw, pitch, roll;
        car_pose(&x, &y, &z, &yaw, &pitch, &roll);
        char line[160];
        int w = snprintf(line, sizeof line, "%5.1f M/S", spd);
        for (int i = 0; i < nl && w < (int)sizeof line; i++) {
            double dx = ln[i].x - x, dz = ln[i].z - z;
            double dist = sqrt(dx * dx + dz * dz);
            /* mission.c bearing convention: atan2(-dx,dz)-yaw, + = left */
            double b = atan2(-dx, dz) - yaw;
            while (b >  3.14159265358979323846)
                b -= 2.0 * 3.14159265358979323846;
            while (b < -3.14159265358979323846)
                b += 2.0 * 3.14159265358979323846;
            const char *arrow = b > 0.15 ? "<" : b < -0.15 ? ">" : "^";
            if (ln[i].user)
                w += snprintf(line + w, sizeof line - (size_t)w,
                              "\nNAV GATE  %s %.0f m", arrow, dist);
            else
                w += snprintf(line + w, sizeof line - (size_t)w,
                              "\nESCORT %.12s  %s %.0f m",
                              ln[i].label, arrow, dist);
        }
        hud_set_text(line);
        return;
    }

    /* hud.c caps a set_text line at HUD_TEXT_CAP (128) and truncates
     * silently past it. Worst case here is 103 characters: the longest
     * counter, the clock, and "TAKE Black Plague's flag  > 4474 m" (the
     * longest team name against the longest distance in the set). The
     * larger buffer is so the compose below cannot truncate mid-format
     * before hud.c ever sees it. */
    char line[160];
    int n;
    if (ob.family == MISSION_FAMILY_CAPTURE)
        n = snprintf(line, sizeof line, "%5.1f M/S  %d of %d Flag Captures",
                     spd, ob.captures, ob.capture_target);
    else if (ob.family == MISSION_FAMILY_MELEE)
        n = snprintf(line, sizeof line, "%5.1f M/S  %d of %d Kills",
                     spd, ob.kills, ob.kill_target);
    else
        n = snprintf(line, sizeof line, "%5.1f M/S  %d of %d Laps Completed",
                     spd, ob.lap, ob.lap_target);
    if (n < 0 || (size_t)n >= sizeof line)
        return;
    if (ob.secs_left >= 0)
        n += snprintf(line + n, sizeof line - (size_t)n,
                      "  %02dH:%02dM:%02dS Minutes Remaining",
                      ob.secs_left / 3600, (ob.secs_left / 60) % 60,
                      ob.secs_left % 60);
    if (n < 0 || (size_t)n >= sizeof line)
        return;

    /* "<" / ">" is which way to turn; "^" means it is ahead. 0.15 rad is
     * about the half-width of a 30 m gate seen from 100 m out. */
    const char *arrow = ob.gate_bearing >  0.15 ? "<"
                      : ob.gate_bearing < -0.15 ? ">" : "^";
    if (ob.family == MISSION_FAMILY_CAPTURE) {
        if (ob.carrying)
            snprintf(line + n, sizeof line - (size_t)n,
                     "\nRETURN TO BASE  %s %.0f m", arrow, ob.gate_dist);
        else
            snprintf(line + n, sizeof line - (size_t)n,
                     "\nTAKE %s's flag  %s %.0f m",
                     mission_team_name(ob.target_team), arrow, ob.gate_dist);
    } else if (ob.family == MISSION_FAMILY_MELEE) {
        /* The opponent count is the game's word ("Kills"); the pointer is
         * ours, and it is the same PORT AID the other two carry. Opponents
         * beyond the 1 km radar are also past the draw distance, while the
         * asset-backed compass reports heading rather than target bearing. */
        snprintf(line + n, sizeof line - (size_t)n,
                 "\n%d LEFT  %s %.0f m", ob.opponents_alive, arrow,
                 ob.gate_dist);
    } else {
        snprintf(line + n, sizeof line - (size_t)n,
                 "\nCHECKPOINT %d of %d  %s %.0f m", ob.gate, ob.gates,
                 arrow, ob.gate_dist);
    }
    hud_set_text(line);
}

static void web_drive_render_impl(void)
{
    if (!car_is_loaded()) return;

    CameraView camera;
    double cb[12];
    int cutscene = drive_camera_render(&camera);
    car_basis(cb);
    raster_pixel_history_frame_begin(s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H,
                                     s_drive_ticks, "software");
    int history_pixel = raster_pixel_history_target_index(
        s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H);
    int mirror_drawn = 0;
    CarLive live = {0};
    car_get_live(&live);
    hud_set_compass_yaw(live.yaw);
    /* PRND21 is a drive selector, not the automatic box's 1..4 gear and
     * never a speed bucket. The current model truthfully owns only R/D. */
    int selector = live.reverse ? HUD_SELECTOR_REVERSE : HUD_SELECTOR_DRIVE;

    s_last_sw_car_draws = 0;
    /* In cockpit view the eye sits INSIDE the body shell, so drawing the
     * player's own mesh would fill the frame with its interior. The cockpit
     * art is the interior. Otherwise queue EVERY decoded body/wheel part at
     * basis ∘ part-frame — the one-part chase car read as a bumper fragment
     * floating behind the camera (see web_gpu_car_part_* for the same
     * composition on the GPU path). */
    if (s_scene_loaded && s_car_nparts > 0 &&
        (cutscene || s_drive_view != DRIVE_VIEW_COCKPIT)) {
        scene_dyn_clear();
        for (int i = 0; i < s_car_nparts; i++) {
            double xf[12], live[12];
            const double *frame = s_car_parts[i].frame;
            if (s_car_parts[i].weapon >= 0 &&
                car_weapon_part_live_frame(s_car_parts[i].weapon,
                                           s_car_parts[i].weapon_part,
                                           live) == 0)
                frame = live;
            basis_compose_frame(xf, cb, frame);
            if (scene_dyn_add(s_car_parts[i].mesh, xf, xf + 3, xf + 6,
                              xf + 9) >= 0)
                s_last_sw_car_draws++;
        }
    } else if (s_scene_loaded && !cutscene &&
               s_drive_view == DRIVE_VIEW_COCKPIT) {
        /*
         * Full first-person shell (body mid/doors/floor/pillars, seat,
         * wheel, glass, instruments). Earlier builds drew only BDYF/BDYT/
         * DASH/RTCB + live GER6/CMP6/RTC6, which left A-pillars short of
         * the roof and empty door/floor quads when glancing. Prefer the
         * authored 640-mode *6 instrument siblings over *3 (MESHVIEW is
         * 640). Live .map composites override GER6/CMP6/RTC6 faces.
         */
        scene_dyn_clear();
        for (int i = 0; i < s_interior_nparts; i++) {
            const char *name = car_interior_part_name(
                s_interior_parts[i].source);
            size_t len = name ? strlen(name) : 0;
            const char *role = len >= 4 ? name + len - 4 : "";
            /* Skip 320-mode twins when a 640-mode sibling is present. */
            if (len >= 4 && role[3] == '3') {
                char want6[9];
                int skip3 = 0;
                memcpy(want6, name, len < 8 ? len : 8);
                want6[len < 8 ? len : 8] = '\0';
                if (len >= 1)
                    want6[len - 1] = '6';
                for (int j = 0; j < s_interior_nparts; j++) {
                    const char *n2 = car_interior_part_name(
                        s_interior_parts[j].source);
                    if (n2 && strcasecmp(n2, want6) == 0) {
                        skip3 = 1;
                        break;
                    }
                }
                if (skip3)
                    continue;
            }
            if (strcasecmp(role, "RTC1") == 0) {
                int has6 = 0;
                for (int j = 0; j < s_interior_nparts; j++) {
                    const char *n2 = car_interior_part_name(
                        s_interior_parts[j].source);
                    size_t l2 = n2 ? strlen(n2) : 0;
                    if (l2 >= 4 && strcasecmp(n2 + l2 - 4, "RTC6") == 0) {
                        has6 = 1;
                        break;
                    }
                }
                if (has6)
                    continue;
            }
            const RTex *tex = NULL;
            if (strcasecmp(role, "GER6") == 0)
                tex = hud_cockpit_gear_texture(selector);
            else if (strcasecmp(role, "CMP6") == 0)
                tex = hud_cockpit_compass_texture();
            else if (strcasecmp(role, "RTC6") == 0)
                tex = hud_cockpit_reticle_texture();
            double xf[12];
            basis_compose_frame(xf, cb, s_interior_parts[i].frame);
            (void)scene_dyn_add_textured(s_interior_parts[i].mesh,
                                         xf, xf + 3, xf + 6, xf + 9, tex);
        }
    } else if (s_scene_loaded) {
        scene_dyn_clear();
    }

    worldrender_camera(s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, &camera,
                       (!cutscene && s_drive_view == DRIVE_VIEW_COCKPIT)
                           ? COCKPIT_NEAR : 0.0,
                       0.0, terrain_is_loaded(), s_scene_loaded);

    /*
     * Rearview mirror: the second derived CameraView renders offscreen and
     * is composited MIRRORED (horizontal flip — a mirror, not a turn-around
     * camera), before the HUD pass so the cockpit art frames it. With the
     * HUD's authored zmiri mask the content renders at the mask's size and
     * shows only through its 0xFF glass cutout (the housing art wins
     * elsewhere); without it the isolated fallback rect above is the whole
     * treatment. rearview_derive gates cockpit-only + no mission camera;
     * the primary camera is never affected.
     */
    {
        CameraView rv;
        if (rearview_derive(&rv)) {
            int mask_w = 0, mask_h = 0;
            const uint8_t *mask = hud_mirror_mask(&mask_w, &mask_h);
            mirror_drawn = 1;
            scene_dyn_clear();
            worldrender_camera(s_mirror_fb, MIRROR_W, MIRROR_H, &rv,
                               0.0, 0.0,
                               terrain_is_loaded(), s_scene_loaded);
            for (int y = 0; y < MIRROR_H; y++)
                for (int x = 0; x < MIRROR_W; x++) {
                    uint8_t *dst = &s_fb[(MIRROR_Y + y) * MESHVIEW_FB_W
                                         + MIRROR_X + x];
                    int mx = mask ? x * mask_w / MIRROR_W : 0;
                    int my = mask ? y * mask_h / MIRROR_H : 0;
                    if (!mask || mask[my * mask_w + mx] == 0xFF)
                        *dst = s_mirror_fb[y * MIRROR_W +
                                           (MIRROR_W - 1 - x)];
                    else
                        *dst = mask[my * mask_w + mx];
                }
            if (history_pixel >= 0) {
                int hx = history_pixel % MESHVIEW_FB_W;
                int hy = history_pixel / MESHVIEW_FB_W;
                if (hx >= MIRROR_X && hx < MIRROR_X + MIRROR_W &&
                    hy >= MIRROR_Y && hy < MIRROR_Y + MIRROR_H) {
                    int rx = hx - MIRROR_X, ry = hy - MIRROR_Y;
                    int mx = mask ? rx * mask_w / MIRROR_W : 0;
                    int my = mask ? ry * mask_h / MIRROR_H : 0;
                    int glass = !mask || mask[my * mask_w + mx] == 0xFF;
                    raster_pixel_history_note_overlay(
                        s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, 0,
                        glass ? "rearview-framebuffer" : "zmiri101.map",
                        glass ? "rearview" : "rearview-housing",
                        "post-world", "overwrite_by_later_pass");
                }
            }
        }
    }

    /* HUD overlay (M3.5), drive mode only: render into the scratch
     * buffer and composite nonzero-over, after the scene so the cockpit
     * sits on top. Snapshot the pre-HUD frame first so web_hud_pixels()
     * can measure the overlay's footprint. When no HUD art loaded the
     * scratch clears to all-background and the composite is a no-op. */
    memcpy(s_pre_hud, s_fb, sizeof s_pre_hud);
    {
        double spd = car_speed();
        hud_set_engine_rpm(car_rpm());
        /* M7: cockpit status line shows the combat damage pool when the
         * mission has a combat user entity (fuel: no model — omitted). */
        int u = combat_user_ent();
        /*
         * The production cockpit and chase views use the authored upper-panel
         * composition. The complete ZDASH texture and its unsupported floating
         * lower instruments remain decoder proofs. Mission cameras suppress
         * all cockpit chrome; chase suppresses only the unresolved lower dash.
         */
        uint32_t hflags = HUD_FLAG_NO_PROOF | HUD_FLAG_COCKPIT_LAYOUT;
        if (cutscene)
            hflags = HUD_FLAG_NO_PROOF | HUD_FLAG_NO_DASH;
        else if (s_drive_view != DRIVE_VIEW_COCKPIT)
            hflags |= HUD_FLAG_NO_DASH;
        if (u >= 0 && combat_hp_max(u) > 0) {
            hud_set_vitals(combat_hp(u), combat_hp_max(u), -1);
            hflags |= HUD_FLAG_VITALS;   /* OR: must not clear the view flags */
        }
        hud_clear_radar_contacts();
        for (int i = 0; i < mission_contact_count(); i++) {
            MissionContact c;
            if (mission_contact(i, &c) != 0 || !c.alive || c.hidden ||
                c.relation == 0 || c.relation == 2)
                continue;
            double dx = c.x - cb[9];
            double dz = c.z - cb[11];
            hud_add_radar_contact(dx * cb[0] + dz * cb[2],
                                  dx * cb[6] + dz * cb[8],
                                  c.relation < 0);
        }
        /* H-UAT-011c: target condition readout — the entity the fire cone
         * would strike now, else the nearest live hostile, bounded by the
         * shipped radar field. Name + live scalar pool from the combat
         * model; the HUD renders the authored LED/range anchors. */
        {
            double tpos[3];
            int tent = combat_target_marker(tpos);
            if (tent < 0 && u >= 0)
                tent = combat_nearest_enemy(u);
            int have = 0;
            if (!cutscene && tent >= 0 &&
                combat_ent_position(tent, tpos) == 0) {
                double tdx = tpos[0] - cb[9], tdz = tpos[2] - cb[11];
                double rng = sqrt(tdx * tdx + tdz * tdz);
                if (rng <= HUD_RADAR_RANGE_M) {
                    hud_set_target(combat_ent_label(tent), combat_hp(tent),
                                   combat_hp_max(tent), rng);
                    have = 1;
                }
            }
            if (!have)
                hud_clear_target();
        }
        hud_clear_conditions();
        if (u >= 0)
            for (int i = 0; i < COMBAT_COMP_COUNT; i++) {
                int hp = combat_component_hp(u, i);
                int hp_max = combat_component_hp_max(u, i);
                if (hp >= 0 && hp_max > 0)
                    hud_set_condition(i, hp, hp_max);
            }
        hud_clear_weapon_rows();
        for (int i = 0; i < combat_player_weapon_count(); i++) {
            const char *name = NULL;
            int ammo = 0, ammo_max = 0;
            if (combat_player_weapon_get(i, &name, &ammo, &ammo_max) == 0) {
                hud_set_weapon_row(i, name, ammo, ammo_max);
                hud_set_weapon_row_armed(i, combat_player_weapon_armed(i));
            }
        }
        hud_set_weapon(combat_player_weapon_name(),
                       combat_player_weapon_selected(),
                       combat_player_weapon_count(),
                       combat_ammo_left(u), combat_ammo_capacity(),
                       combat_player_weapon_damage());
        drive_objective_line(spd);
        hud_render_frame(s_scene_fb, MESHVIEW_FB_W, MESHVIEW_FB_H,
                         selector, spd, s_drive_ticks * 50u, hflags);
        /* Preserve the dashboard-layout audit before the target marker and
         * other legitimate world-space combat FX are drawn into this shared
         * indexed scratch. Both software and WebGPU consume these exact HUD
         * pixels through the complete compositor below. */
        s_authored_hud_lower_pixels = 0;
        for (int y = 224; y < MESHVIEW_FB_H; y++)
            for (int x = 0; x < MESHVIEW_FB_W; x++)
                if (s_scene_fb[y * MESHVIEW_FB_W + x] != 0)
                    s_authored_hud_lower_pixels++;
        if (history_pixel >= 0 && s_scene_fb[history_pixel] != 0)
            raster_pixel_history_note_overlay(
                s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, 0,
                "authored-hud", "hud", "post-world",
                "overwrite_by_later_pass");
        /*
         * The GPU path consumes this scratch as its HUD layer. Carry the
         * authored mirror housing there as well as in the software world
         * composite: nonzero HUD art keeps priority, glass (0xFF) stays open,
         * and the GPU mirror pass can clip and then paint the same bezel
         * without a second asset seam.
         */
        if (mirror_drawn) {
            int mask_w = 0, mask_h = 0;
            const uint8_t *mask = hud_mirror_mask(&mask_w, &mask_h);
            if (mask && MIRROR_X + MIRROR_W <= MESHVIEW_FB_W &&
                MIRROR_Y + MIRROR_H <= MESHVIEW_FB_H) {
                for (int y = 0; y < MIRROR_H; y++)
                    for (int x = 0; x < MIRROR_W; x++) {
                        int mx = x * mask_w / MIRROR_W;
                        int my = y * mask_h / MIRROR_H;
                        uint8_t m = mask[my * mask_w + mx];
                        uint8_t *dst =
                            &s_scene_fb[(MIRROR_Y + y) * MESHVIEW_FB_W +
                                        MIRROR_X + x];
                        if (m != 0xFF && *dst == 0)
                            *dst = m;
                    }
                if (history_pixel >= 0) {
                    int hx = history_pixel % MESHVIEW_FB_W;
                    int hy = history_pixel / MESHVIEW_FB_W;
                    if (hx >= MIRROR_X && hx < MIRROR_X + MIRROR_W &&
                        hy >= MIRROR_Y && hy < MIRROR_Y + MIRROR_H) {
                        int mx = (hx - MIRROR_X) * mask_w / MIRROR_W;
                        int my = (hy - MIRROR_Y) * mask_h / MIRROR_H;
                        if (mask[my * mask_w + mx] != 0xFF &&
                            s_scene_fb[history_pixel] != 0)
                            raster_pixel_history_note_overlay(
                                s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, 0,
                                "zmiri101.map", "rearview-housing",
                                "hud", "overwrite_by_later_pass");
                    }
                }
            }
        }
        /* The .45 rides with the pilot, not the dash: while glancing out a
         * side window its ZH* frame draws here (a shot plays the sequence —
         * see web_drive_step). Suppressed with the dash outside the cockpit
         * and under a mission camera. */
        if (!cutscene && s_drive_view == DRIVE_VIEW_COCKPIT &&
            combat_player_sidearm_present() &&
            (s_sidearm_anim >= 0 || s_look_x != 0)) {
            uint8_t before = history_pixel >= 0 ? s_scene_fb[history_pixel] : 0;
            sidearm_art_draw(s_scene_fb, MESHVIEW_FB_W, MESHVIEW_FB_H);
            if (history_pixel >= 0 && s_scene_fb[history_pixel] != before)
                raster_pixel_history_note_overlay(
                    s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, 0,
                    "pilot-sidearm", "sidearm", "hud",
                    "overwrite_by_later_pass");
        }
        {
            double target[3];
            int nfx = combat_fx_snapshot(s_combat_fx, COMBAT_FX_MAX);
            int target_ent = combat_target_marker(target);
            uint8_t ph_before = history_pixel >= 0 ? s_scene_fb[history_pixel] : 0;
            uint32_t audit_before = audit_fnv1a(s_scene_fb, sizeof s_scene_fb);
            scene_render_combat_fx(s_scene_fb, MESHVIEW_FB_W, MESHVIEW_FB_H,
                                   &camera, s_combat_fx, nfx,
                                   !cutscene && target_ent >= 0
                                       ? target : NULL);
            if (history_pixel >= 0 && s_scene_fb[history_pixel] != ph_before)
                raster_pixel_history_note_overlay(
                    s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, 0,
                    "combat-fx", "combat-fx", "hud",
                    "overwrite_by_later_pass");
            s_last_fx_events = nfx;
            s_last_fx_sw_present = nfx > 0 &&
                audit_fnv1a(s_scene_fb, sizeof s_scene_fb) != audit_before;
        }
        /* Paper surfaces sit on top of the cockpit: route map (M) and
         * held notepad (N). Refresh again at render time so a direct pose
         * correction between fixed ticks cannot leave the pin one frame old.
         * Damage feedback is applied after paper: a retried mission can take
         * live damage while its episode card is up, and hiding that safety cue
         * was H-UAT-013's observed no-flash regression. */
        paper_set_player(live.x, live.z, live.yaw);
        build_gpu_overlay_layer(&camera);
        /* WebGPU consumes this exact post-world layer and mask. Presence-only
         * H-UAT-056 assertion: an SW-visible active event is in the GPU input
         * too; this does not claim effect fidelity or a native GPU primitive. */
        s_last_fx_gpu_present = s_last_fx_sw_present;
        composite_scene_over_terrain();
        paper_render(s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H);
        {
            uint8_t before = history_pixel >= 0 ? s_fb[history_pixel] : 0;
            damage_flash_render(s_fb, NULL, MESHVIEW_FB_W, MESHVIEW_FB_H,
                                &camera);
            if (history_pixel >= 0 && s_fb[history_pixel] != before)
                raster_pixel_history_note_overlay(
                    s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, 0,
                    "damage-flash", "damage-flash", "post-paper",
                    "final_safety_overlay");
        }
        s_hud_frame_valid = 1;
    }
    raster_pixel_history_frame_end();
}

EMSCRIPTEN_KEEPALIVE
void web_drive_render(void)
{
    /* Direct fixed-tick renders stay exact-current for frame goldens/probes. */
    render_pose_prepare(1.0, 0);
    web_drive_render_impl();
}

EMSCRIPTEN_KEEPALIVE
void web_drive_render_alpha(double alpha)
{
    /* Explicit legacy/dev A/B path. */
    render_pose_prepare(alpha, 0);
    web_drive_render_impl();
}

EMSCRIPTEN_KEEPALIVE
void web_drive_render_alpha_mode(double alpha, int extrapolate)
{
    render_pose_prepare(alpha, extrapolate);
    web_drive_render_impl();
}

/* Exact presentation-source diagnostics for latency/collision probes. The
 * renderers do not consume this JSON; both consume s_render_camera/basis. */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_presentation_state(void)
{
    static char buf[768];
    double cam_pos_err = 0.0, basis_pos_err = 0.0, basis_angle_err = 0.0;
    for (int i = 0; i < 3; i++) {
        double dc = s_render_camera.eye[i] - s_render_cam_curr.eye[i];
        double db = s_render_basis[9 + i] - s_render_basis_curr[9 + i];
        cam_pos_err += dc * dc;
        basis_pos_err += db * db;
    }
    for (int col = 0; col < 3; col++) {
        double d = render_dot3(s_render_basis + col * 3,
                               s_render_basis_curr + col * 3);
        double a = acos(clampd(d, -1.0, 1.0));
        if (a > basis_angle_err) basis_angle_err = a;
    }
    snprintf(buf, sizeof buf,
             "{\"mode\":\"%s\",\"alpha\":%.9g,\"clamp\":%u,"
             "\"scripted\":%d,\"view\":%d,"
             "\"cameraPositionErrorM\":%.9g,"
             "\"basisPositionErrorM\":%.9g,"
             "\"basisAngleErrorRad\":%.9g,"
             "\"velocityConstrained\":%d,"
             "\"headingExtrapolated\":%d,"
             "\"limits\":{\"accelerationMps2\":%.9g}}",
             s_render_extrapolate ? "extrap" : "interp", s_render_alpha,
             s_render_clamp_reason, s_render_curr_scripted,
             s_render_curr_view, sqrt(cam_pos_err), sqrt(basis_pos_err),
             basis_angle_err, s_render_velocity_constrained,
             s_render_heading_extrapolate, RENDER_ACCEL_CLAMP_MPS2);
    return buf;
}

EMSCRIPTEN_KEEPALIVE
const char *web_drive_pose(void) { return s_pose; }

/* Read-only source metadata for one coarse OBB. The page-level bridge
 * regression enumerates the installed table, then uses the latest-hit wrapper
 * below to name any part that actually stopped the physical car. */
static const char *drive_collider_entry_json(int i)
{
    static char buf[512];
    if (i < 0 || i >= s_ncolliders_tbl || !s_colliders_tbl ||
        !s_collider_sources) {
        snprintf(buf, sizeof buf, "{\"index\":-1,\"count\":%d}",
                 s_ncolliders_tbl);
        return buf;
    }
    const CarCollider *b = &s_colliders_tbl[i];
    const ColliderSource *s = &s_collider_sources[i];
    snprintf(buf, sizeof buf,
             "{\"index\":%d,\"count\":%d,\"object\":%d,\"part\":%d,"
             "\"label\":\"%s\",\"drivableObject\":%d,"
             "\"drivableParent\":%d,"
             "\"x\":%.9f,\"z\":%.9f,\"hx\":%.9f,\"hz\":%.9f,"
             "\"ax\":%.9f,\"az\":%.9f,\"y0\":%.9f,\"y1\":%.9f}",
             i, s_ncolliders_tbl, s->object, s->part,
             scene_obj_label(s->object), b->drivable_object,
             b->drivable_parent, b->x, b->z, b->hx, b->hz,
             b->ax, b->az, b->y0, b->y1);
    return buf;
}

EMSCRIPTEN_KEEPALIVE
const char *web_drive_collider_entry(int index)
{
    return drive_collider_entry_json(index);
}

EMSCRIPTEN_KEEPALIVE
const char *web_drive_collider_diag(void)
{
    CarStepDiag d;
    car_get_step_diag(&d);
    return drive_collider_entry_json(d.collider_index);
}

/* Read-only last-step physics telemetry for deterministic headless probes. */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_step_diag(void)
{
    static char buf[1024];
    CarStepDiag d;
    CarLive live;
    car_get_step_diag(&d);
    car_get_live(&live);
    snprintf(buf, sizeof buf,
             "{\"valid\":%d,\"x\":%.9f,\"y\":%.9f,\"z\":%.9f,"
             "\"vx\":%.9f,\"vy\":%.9f,\"vz\":%.9f,"
             "\"startFace\":%d,\"endFace\":%d,"
             "\"groundY\":%.9f,\"terrainRate\":%.9f,"
             "\"terrainVRaw\":%.9f,\"terrainVPreVyCap\":%.9f,"
             "\"vyCap\":%.9f,\"terrainVPostVyCap\":%.9f,"
             "\"launchCap\":%d,\"vyCapApplied\":%d,"
             "\"follow\":%d,\"ballistic\":%d,"
             "\"liftDy\":%.9f,\"liftDv2\":%.9f,"
             "\"liftEnergyJ\":%.9f,\"colliderCandidates\":%d,"
             "\"collider\":%d}",
             d.valid, live.x, live.y, live.z, live.vx, live.vy, live.vz,
             d.start_on_face, d.end_on_face, d.ground_y,
             d.terrain_rate, d.terrain_v_raw, d.terrain_v_pre_vy_cap,
             d.vy_cap, d.terrain_v_post_vy_cap,
             d.launch_cap_applied, d.vy_cap_applied,
             d.follow_branch, d.ballistic_branch,
             d.lift_dy, d.lift_dv2, d.lift_energy_j,
             d.collider_candidates, d.collider_index);
    return buf;
}

/*
 * The scripted-trip nav cue as JSON, for the deterministic gates:
 * {"have":0} or {"have":1,"x":..,"z":..,"r":..,"sq":..,"dist":..,
 * "bearing":..} where dist/bearing are against the live car pose in
 * mission.c's convention (bearing + = objective to the left). have is
 * mission_nav_goal() verbatim: 1 only while an unsatisfied user
 * isWithinNav/isWithinSqNav gate was evaluated in the current tick.
 */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_nav_goal(void)
{
    static char buf[192];
    double gx[2], r;
    int sq;
    if (!mission_nav_goal(gx, &r, &sq)) {
        snprintf(buf, sizeof buf, "{\"have\":0}");
        return buf;
    }
    double x, y, z, yaw, pitch, roll;
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    double dx = gx[0] - x, dz = gx[1] - z;
    double dist = sqrt(dx * dx + dz * dz);
    double b = atan2(-dx, dz) - yaw;
    while (b >  3.14159265358979323846) b -= 2.0 * 3.14159265358979323846;
    while (b < -3.14159265358979323846) b += 2.0 * 3.14159265358979323846;
    snprintf(buf, sizeof buf,
             "{\"have\":1,\"x\":%.3f,\"z\":%.3f,\"r\":%.3f,\"sq\":%d,"
             "\"dist\":%.3f,\"bearing\":%.4f}",
             gx[0], gx[1], r, sq, dist, b);
    return buf;
}

/*
 * Aggregate drive-lifecycle telemetry for the semantic lifecycle probe
 * (web/mission_lifecycle_probe.mjs). Values come only from existing public
 * getters: mission tick/state, cutscene camera ownership, scripted mover
 * target, pending-movie handoff, user nav gate, and live car pose. Derived
 * validity bits make malformed getter claims observable while invalid
 * floating-point values are zeroed to keep the JSON parseable. This surface
 * remains AGGREGATE: no FSM image cells, machine IPs, or raw entity tables
 * cross the wasm boundary. Sensing only; nothing here changes state.
 *
 *   {"tick":N,"state":0,"cam":0,
 *    "scripted":0,"scriptedValid":0,"movie":"",
 *    "nav":{"have":0,"valid":0,"x":0,"z":0,"r":0,"sq":0},
 *    "poseValid":1,"pose":{"x":..,"y":..,"z":..,"yaw":..},
 *    "scriptedPose":null}
 * scriptedPose is null unless the mover claims the player with a finite
 * target. Nav values are zero unless `valid` is 1; `have` preserves the raw
 * getter claim so the probe can fail if the contract is ever violated.
 */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_lifecycle_state(void)
{
    static char buf[512];
    double sp[3], syaw = 0.0, sspd = 0.0;
    int scripted_claim = mission_scripted_car(sp, &syaw, &sspd);
    int scripted_valid = scripted_claim &&
                         isfinite(sp[0]) && isfinite(sp[1]) &&
                         isfinite(sp[2]) && isfinite(syaw) && isfinite(sspd);
    double gx[2] = {0.0, 0.0}, r = 0.0;
    int sq = 0;
    int nav_claim = mission_nav_goal(gx, &r, &sq);
    int nav_valid = nav_claim && isfinite(gx[0]) && isfinite(gx[1]) &&
                    isfinite(r) && r > 0.0;
    const char *mv = mission_movie_pending();
    char movie[64];
    {
        /* Clip filenames are FSM clip-table entries; keep JSON-safe. */
        size_t i = 0;
        if (mv)
            for (; mv[i] && i + 1 < sizeof movie; i++) {
                char ch = mv[i];
                movie[i] = (char)((ch >= 'a' && ch <= 'z') ||
                                  (ch >= 'A' && ch <= 'Z') ||
                                  (ch >= '0' && ch <= '9') ||
                                  ch == '.' || ch == '_' || ch == '-'
                                  ? ch : '_');
            }
        movie[i] = '\0';
    }
    double x, y, z, yaw, pitch, roll;
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    int pose_valid = isfinite(x) && isfinite(y) && isfinite(z) && isfinite(yaw);
    if (!pose_valid)
        x = y = z = yaw = 0.0; /* keep the diagnostic JSON parseable */
    if (!nav_valid) {
        gx[0] = gx[1] = r = 0.0;
        sq = 0;
    }
    snprintf(buf, sizeof buf,
             "{\"tick\":%llu,\"state\":%d,\"cam\":%d,"
             "\"scripted\":%d,\"scriptedValid\":%d,\"movie\":\"%s\","
             "\"nav\":{\"have\":%d,\"valid\":%d,\"x\":%.3f,\"z\":%.3f,"
             "\"r\":%.3f,\"sq\":%d},"
             "\"poseValid\":%d,"
             "\"pose\":{\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"yaw\":%.4f},"
             "\"scriptedPose\":",
             (unsigned long long)mission_ticks(), mission_state(),
             mission_cam_active(), scripted_claim, scripted_valid, movie,
             nav_claim, nav_valid, gx[0], gx[1], r, sq,
             pose_valid, x, y, z, yaw);
    size_t n = strlen(buf);
    if (scripted_valid)
        snprintf(buf + n, sizeof buf - n,
                 "{\"x\":%.3f,\"z\":%.3f,\"yaw\":%.4f,\"speed\":%.3f}}",
                 sp[0], sp[2], syaw, sspd);
    else
        snprintf(buf + n, sizeof buf - n, "null}");
    return buf;
}

/* Live mission contacts for diagnostics. Positions come through
 * mission_contact(), so the player is the real car rather than its load-time
 * AI ghost and relation matches combat targeting. playerKill is persistent
 * killed-by-user attribution, not merely the most recent hit. */
EMSCRIPTEN_KEEPALIVE
const char *web_world_state(void)
{
    static char buf[8192];
    size_t n = 0;
    int first = 1;
    buf[0] = '\0';
    n += snprintf(buf + n, sizeof buf - n, "[");
    int count = mission_contact_count();
    for (int e = 0; e < count && n + 128 < sizeof buf; e++) {
        MissionContact c;
        if (mission_contact(e, &c) != 0 || combat_hp(e) < 0)
            continue;
        n += snprintf(buf + n, sizeof buf - n,
                      "%s{\"ent\":%d,\"x\":%.1f,\"z\":%.1f,\"hp\":%d,"
                      "\"alive\":%d,\"hidden\":%d,\"enemy\":%d,"
                      "\"relation\":%d,\"engTarget\":%d,"
                      "\"launches\":%lu,\"contacts\":%lu,\"worldMiss\":%lu,"
                      "\"playerKill\":%d,\"whoShot\":%d}",
                      first ? "" : ",", e, c.x, c.z, combat_hp(e),
                      c.alive, c.hidden, c.relation < 0, c.relation,
                      combat_eng_target(e),combat_launch_count(e),
                      combat_contact_count(e),combat_world_absorb_count(e),
                      combat_is_grooves_fault(e),
                      combat_who_shot(e));
        first = 0;
    }
    snprintf(buf + n, sizeof buf - n, "]");
    return buf;
}

/* Per-agent route telemetry for deterministic trial diagnosis. This is a
 * read-only probe surface: authored/plan targets and heading come directly
 * from ai.c; road clearance is the loaded RSEG ribbon query. */
EMSCRIPTEN_KEEPALIVE
const char *web_ai_route_state(void)
{
    static char buf[4096];
    size_t n = 0;
    int first = 1;
    double user[3] = {0.0, 0.0, 0.0};
    car_pose(&user[0], &user[1], &user[2], NULL, NULL, NULL);
    n += snprintf(buf + n, sizeof buf - n, "[");
    for (int ent = 0; ent < mission_contact_count() && n + 512 < sizeof buf;
         ent++) {
        double p[3], authored[2], plan[2];
        int plan_idx, plan_n;
        if (ent == 0 || ai_get_pos(ent, p) != 0 ||
            ai_route_trace(ent, authored, plan, &plan_idx, &plan_n) != 0)
            continue;
        n += snprintf(buf + n, sizeof buf - n,
                      "%s{\"ent\":%d,\"label\":\"%.24s\","
                      "\"x\":%.9f,\"z\":%.9f,\"heading\":%.9f,"
                      "\"goal\":%d,\"path\":%d,\"wp\":%d,"
                      "\"authoredX\":%.9f,\"authoredZ\":%.9f,"
                      "\"planX\":%.9f,\"planZ\":%.9f,"
                      "\"planIndex\":%d,\"planCount\":%d,"
                      "\"roadClearance\":%.9f,\"planRoadClearance\":%.9f,"
                      "\"playerDistance\":%.9f}",
                      first ? "" : ",", ent, mission_entity_label(ent),
                      p[0], p[2], ai_get_heading(ent), ai_goal(ent),
                      ai_path_id(ent), ai_wp(ent), authored[0], authored[1],
                      plan[0], plan[1], plan_idx, plan_n,
                      terrain_road_nearest(p[0], p[2], NULL, NULL, NULL),
                      terrain_road_nearest(plan[0], plan[1], NULL, NULL, NULL),
                      hypot(p[0] - user[0], p[2] - user[2]));
        first = 0;
    }
    snprintf(buf + n, sizeof buf - n, "]");
    return buf;
}

/* Exact sim pose/support, separate from route telemetry so FOLLOW and idle
 * cars are observable too. Model-origin height is not a tire/hull gap. */
EMSCRIPTEN_KEEPALIVE
const char *web_ai_ground_state(void)
{
    static char buf[32768];
    size_t n = 0;
    int first = 1;
    n += snprintf(buf + n, sizeof buf - n, "[");
    for (int ent = 0; ent < mission_contact_count() && n + 768 < sizeof buf;
         ent++) {
        MissionContact c;
        double p[3], frame[12], low, high;
        int obj = mission_entity_scene_object(ent);
        if (obj < 0 || !mission_scene_object_is_vehicle(obj) ||
            mission_scene_object_is_player_vehicle(obj) ||
            mission_contact(ent, &c) != 0 || ai_get_pos(ent, p) != 0)
            continue;
        int valid = scene_obj_world_xform(obj, frame) == 0 &&
                    scene_obj_ground_gaps(obj, terrain_height_at,
                                          &low, &high) == 0;
        if (!valid) {
            memset(frame, 0, sizeof frame);
            low = high = 0.0;
        }
        n += snprintf(buf + n, sizeof buf - n,
                      "%s{\"ent\":%d,\"label\":\"%.24s\","
                      "\"alive\":%d,\"hidden\":%d,\"physical\":%d,"
                      "\"goal\":%d,\"path\":%d,\"valid\":%d,"
                      "\"x\":%.9f,\"y\":%.9f,\"z\":%.9f,"
                      "\"originAGL\":%.9f,\"minGap\":%.9f,\"maxGap\":%.9f,"
                      "\"up\":[%.9f,%.9f,%.9f],\"heading\":%.9f,"
                      "\"roadClearance\":%.9f}",
                      first ? "" : ",", ent, mission_entity_label(ent),
                      c.alive, c.hidden, ai_physical_active(ent),
                      ai_goal(ent), ai_path_id(ent), valid,
                      p[0], p[1], p[2], p[1] - terrain_height_at(p[0], p[2]),
                      low, high, frame[3], frame[4], frame[5],
                      ai_get_heading(ent),
                      terrain_road_nearest(p[0], p[2], NULL, NULL, NULL));
        first = 0;
    }
    snprintf(buf + n, sizeof buf - n, "]");
    return buf;
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_vehicle_contacts(void)
{
    return mission_vehicle_contacts();
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_player_vehicle_contacts(void)
{
    return mission_player_vehicle_contacts();
}

/* Player combat and selected-weapon state for the page/HUD and browser smoke
 * probes. ammo -1 = infinite; hp -1 = no FSM combat user. lastHit persists
 * as the most recent damaged entity, while cooldown is live tick state. */
EMSCRIPTEN_KEEPALIVE
const char *web_combat_state(void)
{
    static char buf[512];
    int u = combat_user_ent();         /* -1 = no FSM user entity       */
    double muzzle[3]={0}; int have_muzzle=combat_player_muzzle_position(muzzle)==0;
    snprintf(buf, sizeof(buf),
             "{\"ent\":%d,\"hp\":%d,\"hpMax\":%d,\"ammo\":%d,"
             "\"ammoMax\":%d,\"alive\":%d,\"lastHit\":%d,"
             "\"weapon\":%d,\"weapons\":%d,\"damage\":%d,"
             "\"cooldown\":%d,\"rear\":%d,"
             "\"muzzle\":%d,\"muzzlePos\":[%.3f,%.3f,%.3f],"
             "\"sidearm\":%d,\"sidearmAmmo\":%d,\"sidearmName\":\"%s\","
             "\"sidearmSound\":\"%s\",\"sidearmSpeed\":%.1f}",
             u, combat_hp(u), combat_hp_max(u), combat_ammo_left(u),
             combat_ammo_capacity(), combat_alive(u), s_fire_hit,
             combat_player_weapon_selected(), combat_player_weapon_count(),
             combat_player_weapon_damage(), combat_player_weapon_cooldown(),
             combat_player_weapon_rear(),have_muzzle,
             muzzle[0],muzzle[1],muzzle[2],
             combat_player_sidearm_present(), combat_sidearm_ammo_left(),
             combat_sidearm_name(),
             s_sidearm_wi.sound ? s_sidearm_wi.sound : "",
             s_sidearm_wi.projectile_speed);
    return buf;
}

/* ------------------------------------------------------------------ */
/* H-UAT-057 total-weapon audit seams. None is wired to the page.      */
/* ------------------------------------------------------------------ */

EMSCRIPTEN_KEEPALIVE
int web_audit_catalog_ok(void) { return !s_audit_catalog_overflow; }

EMSCRIPTEN_KEEPALIVE
int web_audit_gdf_count(void) { return s_audit_gdf_count; }

EMSCRIPTEN_KEEPALIVE
const char *web_audit_gdf_name(int i)
{
    return i >= 0 && i < s_audit_gdf_count ? s_audit_gdf[i] : "";
}

EMSCRIPTEN_KEEPALIVE
int web_audit_vcf_count(void) { return s_audit_vcf_count; }

EMSCRIPTEN_KEEPALIVE
const char *web_audit_vcf_name(int i)
{
    return i >= 0 && i < s_audit_vcf_count ? s_audit_vcf[i] : "";
}

/* Read one purchaser file into retained Wasm memory so the Node harness can
 * derive a synthetic carrier in MEMFS. The bytes never enter generated docs. */
EMSCRIPTEN_KEEPALIVE
const uint8_t *web_audit_file_read(const char *name)
{
    free(s_audit_file_bytes);
    s_audit_file_bytes = NULL;
    s_audit_file_size = 0;
    s_audit_file_bytes = vfs_read_file(name, &s_audit_file_size);
    return s_audit_file_bytes;
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_audit_file_size(void)
{
    return s_audit_file_size <= UINT32_MAX ? (uint32_t)s_audit_file_size : 0;
}

EMSCRIPTEN_KEEPALIVE
int web_audit_car_load(const char *vcf)
{
    memset(s_weapon_resolved, 0, sizeof s_weapon_resolved);
    return car_load(vcf);
}

EMSCRIPTEN_KEEPALIVE
const char *web_audit_gdf_info(const char *gdf)
{
    static char buf[640];
    CarWeaponInfo wi;
    if (car_gdf_weapon_info(gdf, &wi) != 0) {
        snprintf(buf, sizeof buf, "{\"error\":\"parse\"}");
        return buf;
    }
    snprintf(buf, sizeof buf,
             "{\"gdf\":\"%s\",\"name\":\"%s\",\"damage\":%d,"
             "\"ammo\":%d,\"cooldown\":%d,\"speed\":%.3f,"
             "\"flightSpeed\":%.3f,\"ordnanceType\":%d,"
             "\"ordnanceModel\":\"%s\","
             "\"impactGround\":\"%s\",\"impactCar\":\"%s\","
             "\"impactBuilding\":\"%s\",\"impactStructure\":\"%s\","
             "\"group\":%d,\"family\":%d,\"tier\":%d,"
             "\"deploy\":%d,\"sound\":\"%s\"}",
             gdf, wi.name, wi.damage, wi.ammo, wi.cooldown_ticks,
             wi.projectile_speed, wi.flight_speed, wi.ordnance_type,
             wi.ordnance_model, wi.impact_ground, wi.impact_car,
             wi.impact_building, wi.impact_structure,
             wi.weapon_group, wi.family, wi.tier, wi.deploy_kind, wi.sound);
    return buf;
}

EMSCRIPTEN_KEEPALIVE
const char *web_audit_loadout(void)
{
    static char buf[8192];
    size_t n = 0;
    n += (size_t)snprintf(buf + n, sizeof buf - n, "{\"weapons\":[");
    for (int i = 0; i < car_weapon_count() && n < sizeof buf; i++) {
        CarWeaponInfo wi;
        if (car_weapon_get(i, &wi) != 0) continue;
        int resolved = i < WEB_AUDIT_WEAPONS ? s_weapon_resolved[i] : 0;
        double muzzle_y = 0.0, forward_x = 0.0, forward_y = 0.0,
               forward_z = 1.0;
        double mf[12];
        if (car_weapon_muzzle_frame(i, mf) == 0) {
            muzzle_y = mf[10];
            forward_x = mf[6];
            forward_y = mf[7];
            forward_z = mf[8];
        }
        n += (size_t)snprintf(
            buf + n, n < sizeof buf ? sizeof buf - n : 0,
            "%s{\"source\":%d,\"gdf\":\"%s\",\"name\":\"%s\","
            "\"sound\":\"%s\",\"fireSprite\":\"%s\","
            "\"ordnanceModel\":\"%s\",\"mount\":%d,\"class\":%d,"
            "\"rear\":%d,\"damage\":%d,\"ammo\":%d,\"cooldown\":%d,"
            "\"direct\":%d,\"traverses\":%d,\"turreted\":%d,"
            "\"family\":%d,\"tier\":%d,"
            "\"muzzleY\":%.9f,\"forwardX\":%.9f,"
            "\"forwardY\":%.9f,\"forwardZ\":%.9f,"
            "\"deploy\":%d,\"parts\":%d,\"resolved\":%d}",
            i ? "," : "", i, wi.gdf, wi.name, wi.sound, wi.fire_sprite,
            wi.ordnance_model, wi.mount,
            wi.mount_class, wi.rear_facing, wi.damage, wi.ammo,
            wi.cooldown_ticks, wi.direct_fire, wi.traverses, wi.turreted,
            wi.family, wi.tier, muzzle_y, forward_x, forward_y, forward_z,
            wi.deploy_kind, wi.geometry_parts, resolved);
    }
    if (n < sizeof buf)
        snprintf(buf + n, sizeof buf - n, "]}");
    else
        buf[sizeof buf - 1] = '\0';
    return buf;
}

EMSCRIPTEN_KEEPALIVE
const char *web_audit_hud_weapon_rows(void)
{
    static char buf[128];
    uint32_t authored = 0, fallback = 0, ammo = 0;
    hud_weapon_row_render_masks(&authored, &fallback, &ammo);
    snprintf(buf, sizeof buf,
             "{\"authored\":%u,\"fallback\":%u,\"ammo\":%u}",
             authored, fallback, ammo);
    return buf;
}

/* Current mounted-fire truth for H-UAT-062's linking matrix. Unlike
 * web_audit_loadout (authored car data), this reads combat.c's mutable
 * selected/armed/fired/ammo state after real input and fire ticks. */
EMSCRIPTEN_KEEPALIVE
const char *web_audit_link_state(void)
{
    static char buf[2048];
    size_t n = 0;
    n += (size_t)snprintf(buf + n, sizeof buf - n,
                          "{\"selected\":%d,\"weapons\":[",
                          combat_player_weapon_selected());
    for (int i = 0; i < combat_player_weapon_count() && n < sizeof buf; i++) {
        const char *name = NULL;
        int ammo = 0, ammo_max = 0;
        int source = combat_player_weapon_source_at(i);
        CarWeaponInfo wi;
        const char *gdf = "";
        if (combat_player_weapon_get(i, &name, &ammo, &ammo_max) != 0)
            continue;
        if (source >= 0 && car_weapon_get(source, &wi) == 0)
            gdf = wi.gdf;
        n += (size_t)snprintf(
            buf + n, n < sizeof buf ? sizeof buf - n : 0,
            "%s{\"slot\":%d,\"source\":%d,\"gdf\":\"%s\","
            "\"name\":\"%s\",\"armed\":%d,\"fired\":%d,"
            "\"ammo\":%d,\"ammoMax\":%d}",
            i ? "," : "", i, source, gdf, name ? name : "",
            combat_player_weapon_armed(i), combat_player_weapon_fired(i),
            ammo, ammo_max);
    }
    if (n < sizeof buf)
        snprintf(buf + n, sizeof buf - n, "]}");
    else
        buf[sizeof buf - 1] = '\0';
    return buf;
}

EMSCRIPTEN_KEEPALIVE
const char *web_audit_mission_vcf(const char *path)
{
    return mission_probe_player_object(path);
}

EMSCRIPTEN_KEEPALIVE
int web_audit_target_hp(int ent, int hp)
{
    return combat_probe_set_hp(ent, hp);
}

/* Read-only structure audit: FSM bodies retain their combat owner; scenery
 * without a script identity reads its scene-owned SDFC pool. */
EMSCRIPTEN_KEEPALIVE
const char *web_audit_structure(int obj)
{
    static char buf[256];
    int ent = combat_scene_entity(obj);
    snprintf(buf, sizeof buf,
             "{\"label\":\"%s\",\"class\":%d,\"ent\":%d,"
             "\"authoredHp\":%d,\"hp\":%d,\"hpMax\":%d,\"hidden\":%d}",
             scene_obj_label(obj), scene_obj_class_id(obj), ent,
             scene_obj_hp(obj, 1),
             ent >= 0 ? combat_hp(ent) : scene_obj_hp(obj, 0),
             ent >= 0 ? combat_hp_max(ent) : scene_obj_hp(obj, 1),
             scene_obj_hidden(obj));
    return buf;
}

/* Gate-35 observes the same current nearest-hostile owner that combat_tick
 * already used for player turret traversal. It does not select or mutate a
 * radar target; aim-convergence.md §5's acquisition residual stays open. */
EMSCRIPTEN_KEEPALIVE
int web_audit_turret_target(void)
{
    int user = combat_user_ent();
    return user >= 0 ? combat_nearest_enemy(user) : -1;
}

EMSCRIPTEN_KEEPALIVE
const char *web_audit_turret_state(int source)
{
    static char buf[192];
    int yaw_ok = 0, pitch_ok = 0;
    int have = combat_player_weapon_on_target(source, &yaw_ok, &pitch_ok) == 0;
    snprintf(buf, sizeof buf,
             "{\"have\":%d,\"target\":%d,\"yaw\":%.9f,"
             "\"pitch\":%.9f,\"yawOn\":%d,\"pitchOn\":%d}",
             have, web_audit_turret_target(),
             combat_player_weapon_traverse_yaw(source),
             combat_player_weapon_traverse_pitch(source), yaw_ok, pitch_ok);
    return buf;
}

/* Harness-only target placement through the existing AI correction seam.
 * This lets the drop audit cross a real world object without overlapping the
 * player's physical hull or bypassing deployment/trigger code. */
EMSCRIPTEN_KEEPALIVE
int web_audit_target_place(int ent, double x, double z)
{
    double p[3];
    if (combat_ent_position(ent, p) != 0)
        return -1;
    ai_translate_xz(ent, x - p[0], z - p[2]);
    return 0;
}

EMSCRIPTEN_KEEPALIVE
int web_audit_target_hold(int ent)
{
    if (combat_hp(ent) < 0)
        return -1;
    ai_sit(ent);
    return 0;
}


EMSCRIPTEN_KEEPALIVE
const char *web_audit_fx_state(void)
{
    static char buf[1024];
    int records = combat_fx_snapshot(s_combat_fx, COMBAT_FX_MAX);
    int events = 0, projectiles = 0, hits = 0, last_hit = -1;
    int light = 0, heavy = 0, missiles = 0, explosive = 0;
    for (int i = 0; i < records; i++) {
        if (s_combat_fx[i].type == COMBAT_FX_PROJECTILE)
            projectiles++;
        else
            events++;
        /* Class counts deliberately include live projectiles and standard
         * impact/kill records, but exclude muzzle/deployed records. This lets
         * gate 35 distinguish ORDF-1 flight from missile flight and prove a
         * triggered landmine enters the ordinary explosive-impact FX path. */
        if (s_combat_fx[i].type == COMBAT_FX_PROJECTILE ||
            s_combat_fx[i].type == COMBAT_FX_IMPACT ||
            s_combat_fx[i].type == COMBAT_FX_KILL) {
            if (s_combat_fx[i].weapon_class == COMBAT_FX_TRACER_LIGHT) light++;
            else if (s_combat_fx[i].weapon_class == COMBAT_FX_TRACER_HEAVY) heavy++;
            else if (s_combat_fx[i].weapon_class == COMBAT_FX_MISSILE) missiles++;
            else if (s_combat_fx[i].weapon_class == COMBAT_FX_EXPLOSIVE) explosive++;
        }
        if (s_combat_fx[i].hit >= 0) {
            hits++;
            last_hit = s_combat_fx[i].hit;
        }
    }
    snprintf(buf, sizeof buf,
             "{\"count\":%d,\"projectiles\":%d,\"hits\":%d,"
             "\"lastHit\":%d,\"tracerLight\":%d,\"tracerHeavy\":%d,"
             "\"missiles\":%d,\"explosive\":%d}",
             events, projectiles, hits, last_hit, light, heavy, missiles,
             explosive);
    return buf;
}

EMSCRIPTEN_KEEPALIVE
int web_audit_fx_hit(int ent)
{
    /* Audit event history, not the transient draw pool: a missile's authored
     * 5 s cooldown must not erase proof that its impact event was emitted. */
    unsigned long hits = combat_fx_hit_count(ent);
    return hits > 0x7ffffffful ? 0x7fffffff : (int)hits;
}

EMSCRIPTEN_KEEPALIVE
int web_audit_fx_impact(const char *effect_name,
                        double x, double y, double z)
{
    /* Gate-only presentation input: exercise production XDF/TMT resolution
     * at a deterministic visible point without changing combat or mission. */
    return combat_probe_impact_at(effect_name, x, y, z);
}

EMSCRIPTEN_KEEPALIVE
const char *web_audit_deployed_state(int kind, int ent)
{
    static char buf[384];
    double p[3] = {0.0, 0.0, 0.0};
    int have_pos = combat_deployed_position(kind, p) == 0;
    snprintf(buf, sizeof buf,
             "{\"kind\":%d,\"active\":%d,\"deployed\":%lu,"
             "\"triggers\":%lu,\"obstacleHits\":%lu,"
             "\"gripTicks\":%d,\"aiGripTicks\":%d,"
             "\"effectiveGrip\":%.6f,\"havePos\":%d,"
             "\"x\":%.6f,\"y\":%.6f,\"z\":%.6f}",
             kind, combat_deployed_active(kind), combat_deploy_count(kind),
             combat_deploy_trigger_count(kind), combat_deploy_obstacle_count(),
             car_grip_loss_ticks(), ai_grip_loss_ticks(ent),
             car_effective_grip(), have_pos, p[0], p[1], p[2]);
    return buf;
}

EMSCRIPTEN_KEEPALIVE
const char *web_audit_render_state(void)
{
    static char buf[320];
    snprintf(buf, sizeof buf,
             "{\"expectedCar\":%d,\"swCar\":%d,\"fxEvents\":%d,"
             "\"swFx\":%d,\"gpuFx\":%d,\"authoredDeployers\":%d,"
             "\"deployerModel\":\"%s\",\"deployerModels\":%u,"
             "\"oilFallbacks\":%d,"
             "\"impactAuthored\":%d,\"impactMissing\":%d,"
             "\"killAuthored\":%d,\"killFallbacks\":%d}",
             s_car_nparts, s_last_sw_car_draws, s_last_fx_events,
             s_last_fx_sw_present, s_last_fx_gpu_present,
             scene_fx_authored_deployer_draws(),
             scene_fx_authored_deployer_model(),
             scene_fx_authored_deployer_models(),
             scene_fx_oil_fallback_draws(),
             scene_fx_impact_authored_draws(),
             scene_fx_impact_missing_draws(),
             scene_fx_kill_authored_draws(),
             scene_fx_kill_fallback_draws());
    return buf;
}

/* D-C27 kill-FX asset audit, the JS mirror of cockpit_probe.c's XOS1_101
 * block: descriptor shape plus first/last frame resolution and names. */
EMSCRIPTEN_KEEPALIVE
const char *web_audit_tmt_info(const char *name)
{
    static char buf[384];
    TexTmtInfo info;
    if (!name || texcache_tmt_info(name, &info) != 0) {
        snprintf(buf, sizeof buf, "{\"ok\":0}");
        return buf;
    }
    uint32_t resolved = 0;
    for (uint32_t f = 0; f < info.name_count; f++)
        if (texcache_resolve_tmt_frame(name, f) != TEX_ID_NONE)
            resolved++;
    uint16_t first = texcache_resolve_tmt_frame(name, 0);
    uint16_t last = texcache_resolve_tmt_frame(name, info.name_count - 1u);
    snprintf(buf, sizeof buf,
             "{\"ok\":1,\"kind\":%u,\"count\":%u,\"stride\":%u,"
             "\"rate\":%.6g,\"mode\":%u,\"nameCount\":%u,"
             "\"resolvedFrames\":%u,\"firstTile\":\"%s\",\"lastTile\":\"%s\"}",
             info.kind, info.count, info.stride, (double)info.rate,
             info.mode, info.name_count, resolved,
             first != TEX_ID_NONE ? texcache_tile_name(first) : "",
             last != TEX_ID_NONE ? texcache_tile_name(last) : "");
    return buf;
}

/* D-C27 wreck-sequence audit for one entity: the native 200-tick burn,
 * scene visibility, decoded damage/smoke bands, and the live kill-record
 * schedule plus secondary/debris record counts from the snapshot. */
EMSCRIPTEN_KEEPALIVE
const char *web_audit_kill_state(int ent)
{
    static char buf[384];
    int records = combat_fx_snapshot(s_combat_fx, COMBAT_FX_MAX);
    int kills = 0, kill_life = -1, kill_age = -1;
    int secondary = 0, debris = 0;
    for (int i = 0; i < records; i++) {
        if (s_combat_fx[i].type == COMBAT_FX_KILL) {
            kills++;
            kill_life = s_combat_fx[i].life;
            kill_age = s_combat_fx[i].age;
        } else if (s_combat_fx[i].type == COMBAT_FX_SECONDARY) {
            secondary++;
        } else if (s_combat_fx[i].type == COMBAT_FX_DEBRIS) {
            debris++;
        }
    }
    snprintf(buf, sizeof buf,
             "{\"ent\":%d,\"dead\":%d,\"wreckTicks\":%d,"
             "\"wreckVisible\":%d,\"smokeLevel\":%d,"
             "\"sideStates\":[%d,%d,%d,%d],"
             "\"killRecords\":%d,\"killLife\":%d,\"killAge\":%d,"
             "\"secondary\":%d,\"debris\":%d}",
             ent, combat_is_dead(ent), combat_wreck_ticks(ent),
             combat_wreck_scene_visible(ent), combat_smoke_level(ent),
             combat_side_damage_state(ent, 0),
             combat_side_damage_state(ent, 1),
             combat_side_damage_state(ent, 2),
             combat_side_damage_state(ent, 3),
             kills, kill_life, kill_age, secondary, debris);
    return buf;
}

/*
 * The arena objective readout as JSON — the same numbers the HUD draws.
 *
 * `live` is 0 for every scripted trip and for any arena family without a
 * controller; the rest of the fields are then zero and must not be read as
 * an objective. `dist`/`bearing` point at whatever must be reached next —
 * the checkpoint on a race, the flag or the home base on a capture map —
 * and `bearing` is radians off the car's nose, positive to the left.
 * Sensing only: nothing here changes state.
 */
EMSCRIPTEN_KEEPALIVE
const char *web_objective_state(void)
{
    static char buf[512];
    MissionObjectiveState ob;
    int live = mission_objective_state(&ob);
    snprintf(buf, sizeof buf,
             "{\"live\":%d,\"family\":%d,\"lap\":%d,\"lapTarget\":%d,"
             "\"gate\":%d,\"gates\":%d,\"dist\":%.1f,\"bearing\":%.4f,"
             "\"secsLeft\":%d,\"captures\":%d,\"captureTarget\":%d,"
             "\"carrying\":%d,\"targetTeam\":%d,\"teams\":%d,\"team\":%d,"
             "\"kills\":%d,\"killTarget\":%d,\"opponents\":%d,"
             "\"opponentsAlive\":%d,\"hp\":%d,\"hpMax\":%d}",
             live, ob.family, ob.lap, ob.lap_target, ob.gate, ob.gates,
             ob.gate_dist, ob.gate_bearing, ob.secs_left, ob.captures,
             ob.capture_target, ob.carrying, ob.target_team, ob.teams,
             ob.team, ob.kills, ob.kill_target, ob.opponents,
             ob.opponents_alive, ob.hp, ob.hp_max);
    return buf;
}

static int garage_json_escape(char *dst, size_t cap, const char *src);

/*
 * Scripted-trip objective LINES as JSON — one entry per nav gate the
 * mission FSM is currently waiting on (mission_objective_lines), in
 * stable first-seen order:
 *
 *   {"reachedAge":-1,"lines":[
 *     {"user":0,"label":"tanker1","dist":410.0,"bearing":0.41,"r":80.0},
 *     {"user":1,"label":"user","dist":750.2,"bearing":-0.2,"r":80.0}]}
 *
 * `label` is the mission's own FSM entity label; dist/bearing are against
 * the live car pose in web_drive_nav_goal's convention (+ = to the left).
 * reachedAge is ticks since a USER gate was GENUINELY satisfied while the
 * mission was still RUNNING (-1 = never): the page's "OBJECTIVE REACHED"
 * toast keys on this instead of on banner-text changes, so a mission
 * swapping polls on a fail branch can no longer fake a success toast
 * (H-UAT-014). Sensing only: nothing here changes state.
 */
EMSCRIPTEN_KEEPALIVE
const char *web_objective_lines(void)
{
    static char buf[24576];
    MissionObjectiveLine ln[6];
    int n = mission_objective_lines(ln, 6);
    double x, y, z, yaw, pitch, roll;
    car_pose(&x, &y, &z, &yaw, &pitch, &roll);
    size_t w = 0;
    w += (size_t)snprintf(buf + w, sizeof buf - w,
                          "{\"reachedAge\":%d,\"lines\":[",
                          mission_objective_reached_age());
    for (int i = 0; i < n && w + 256 < sizeof buf; i++) {
        double dx = ln[i].x - x, dz = ln[i].z - z;
        double dist = sqrt(dx * dx + dz * dz);
        double b = atan2(-dx, dz) - yaw;
        while (b >  3.14159265358979323846) b -= 2.0 * 3.14159265358979323846;
        while (b < -3.14159265358979323846) b += 2.0 * 3.14159265358979323846;
        char label[40];             /* labels are FSM strings; JSON-safe */
        size_t li = 0;
        for (const char *p = ln[i].label; *p && li + 1 < sizeof label; p++) {
            char c = *p;
            label[li++] = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                          (c >= '0' && c <= '9') || c == '_' || c == '-'
                              ? c : '_';
        }
        label[li] = '\0';
        w += (size_t)snprintf(buf + w, sizeof buf - w,
                              "%s{\"user\":%d,\"label\":\"%s\","
                              "\"dist\":%.1f,\"bearing\":%.4f,\"r\":%.1f,"
                              "\"x\":%.3f,\"z\":%.3f,\"classification\":\"PROGRESS\"}",
                              i ? "," : "", ln[i].user, label, dist, b,
                              ln[i].r, ln[i].x, ln[i].z);
    }
    w += (size_t)snprintf(buf + w, sizeof buf - w, "],\"predicates\":[");
    MissionNavPredicate predicates[128];
    int np = mission_nav_predicates(predicates, 128);
    static const char *classes[] = { "UNKNOWN", "PROGRESS", "FAILURE" };
    for (int i = 0; i < np && w + 192 < sizeof buf; i++) {
        const MissionNavPredicate *p = &predicates[i];
        w += (size_t)snprintf(buf + w, sizeof buf - w,
            "%s{\"x\":%.3f,\"z\":%.3f,\"r\":%.1f,\"sq\":%d,"
            "\"classification\":\"%s\",\"guidance\":%d}",
            i ? "," : "", p->x, p->z, p->r, p->sq,
            classes[p->consequence], p->guidance);
    }
    w += (size_t)snprintf(buf + w, sizeof buf - w, "],\"notes\":[");
    for (int id = 1; id <= 6 && w + 600 < sizeof buf; id++) {
        const MissionNote *note = mission_note(id);
        if (!note) break;
        char text[512];
        garage_json_escape(text, sizeof text, note->text);
        w += (size_t)snprintf(buf + w, sizeof buf - w,
            "%s{\"id\":%d,\"flags\":%u,\"text\":\"%s\"}",
            id > 1 ? "," : "", id, note->flags, text);
    }
    snprintf(buf + w, sizeof buf - w, "]}");
    return buf;
}

EMSCRIPTEN_KEEPALIVE
int web_mission_logic_state(void)
{
    return mission_state();
}

EMSCRIPTEN_KEEPALIVE
const char *web_mission_logic_message(void)
{
    const char *m = mission_message();
    return m ? m : "";
}
EMSCRIPTEN_KEEPALIVE
const char *web_mission_movie_pending(void)
{
    const char *name = mission_movie_pending();
    return name ? name : "";
}

EMSCRIPTEN_KEEPALIVE
void web_mission_movie_ack(void)
{
    mission_movie_ack();
}

/* Wasm handle for the mission.h save/resume hook (tick + terminal state
 * only — FSM machine state is not restored). The node contract tests
 * use it to reach a terminal mission state without scripted play. */
EMSCRIPTEN_KEEPALIVE
void web_mission_restore(int state)
{
    mission_restore(mission_ticks(), state);
}


/* NOTE: KEEPALIVE restored — a merge dropped it, leaving this export
 * out of the wasm (the page's drive button calls it). */
EMSCRIPTEN_KEEPALIVE
const char *web_drive_stats(void)
{
    static char buf[2048];
    car_stats(buf, sizeof(buf));
    return buf;
}

/* HUD diagnostics (M7): hud_stats() passthrough — dash/pal/elt/sheets/
 * font/text, the answer to "why isn't the cockpit text drawing". */
EMSCRIPTEN_KEEPALIVE
const char *web_hud_stats(void)
{
    static char buf[512];
    hud_stats(buf, sizeof(buf));
    return buf;
}

/* ------------------------------------------------------------------ */
/* Web shell (M5): menu -> briefing -> drive -> debrief around the     */
/* drive mode, with a terminal credits state after the base Trip.      */
/* Reference flow (REVERSING.md, game_session_run): the                */
/* original sets g_gamestate=GS_SHELL(6) and blocks in NITSHELL        */
/* ShellMain; a scenario pick (shell_result 2/4) runs the session init */
/* chain into GS_GAMEPLAY(5); mission end raises GS_MISSIONEND(7) and  */
/* returns to the frontend (GS_FRONTEND=1). SHELL_MENU/SHELL_BRIEFING  */
/* are our ShellMain surface, SHELL_DRIVE is GS_GAMEPLAY,              */
/* SHELL_DEBRIEF is the mission-end return path, and SHELL_CREDITS is  */
/* the base shell's own credits screen (i76shell.dll ShellWindowProc   */
/* case 0 -> FUN_1000e2e0, phase-e-base-delta.md §4.1). The page       */
/* drives all transitions through the exports below;                   */
/* web_shell_update() is the single transition point for               */
/* drive->debrief (FSM end or ESC).                                    */
/*                                                                     */
/* Shell story movies. Each base Trip mission names intro/outro clips in */
/* its WDEF/WRLD fields (mission_story_clip). The shell queues the intro */
/* when a mission starts and the outro when it completes; the page plays */
/* them through the same Smacker acknowledge lifecycle and defers debrief */
/* for the ack. This placement is a PORT decision — the retail shell's */
/* mission->SMK sequencing graph is an open RE residual (phase-e       */
/* spec §6) — but the clip membership is the mission's own data, not   */
/* an invented map. State ids are declared with the top globals        */
/* (web_key_event uses them above).                                    */
/* ------------------------------------------------------------------ */

#define SHELL_CAT_CAP 128

static char     s_cat_path[SHELL_CAT_CAP][32]; /* vfs path (miss8/P01.MSN) */
static int      s_cat_count;
static int      s_sel = -1;              /* picked catalog index */
/* Which arena row the live pick came from, or -1 (campaign or dev
 * catalog). The shell needs it to title the briefing and the debrief:
 * the arena maps are not in SCENARIO.DAT, so trip+index cannot name them. */
static int      s_arena_sel = -1;
static char     s_briefing[1024];        /* briefing screen text */
static char     s_result[1536];          /* debrief JSON */

/* Staged-mission probe. vfs_exists() short-circuits on the zix index
 * (find_file miss -> 0) and never reaches its loose-file branch for
 * files the page staged into MEMFS after init — so the catalog probes
 * the loose filesystem directly (fs_fopen tries exact/UPPER/lowercase
 * basename, fs.c). */
static int loose_exists(const char *path)
{
    FILE *f = fs_fopen(path, "rb");
    if (!f) return 0;
    fclose(f);
    return 1;
}

/*
 * Shell-queued story movie. Distinct from mission.c's FSM playMovie
 * handoff: these clips come from the mission's WDEF/WRLD intro/outro
 * fields (mission_story_clip) at shell transition points, not bytecode.
 * The page polls web_shell_movie_pending(), plays the clip through the
 * same Smacker panel, and acknowledges exactly once; while pending,
 * web_shell_update() defers the drive->debrief transition just like it
 * does for an FSM-queued movie. s_out_played latches so a completed
 * mission's OUT* clip is offered exactly once per run.
 */
static char s_shell_movie[48];
static int  s_shell_movie_pending;
static int  s_out_played;
/* Set by shell_enter_debrief(): the run that just ended completed the
 * base Trip's final mission (T17). Drives web_shell_has_credits(). */
static int  s_campaign_complete;

static void shell_movie_clear(void)
{
    s_shell_movie[0] = '\0';
    s_shell_movie_pending = 0;
}

/* Queue a mission-authored story filename for the page, but only when the
 * purchaser actually staged it under smk/ — an unstaged clip must never
 * block a shell transition (same honesty rule as the mission catalog). */
static void shell_movie_queue(const char *clip)
{
    if (!clip || !clip[0]) return;
    char path[64];
    snprintf(path, sizeof path, "smk/%s", clip);
    if (!loose_exists(path)) return;
    snprintf(s_shell_movie, sizeof s_shell_movie, "%s", clip);
    s_shell_movie_pending = 1;
}

EMSCRIPTEN_KEEPALIVE
const char *web_shell_movie_pending(void)
{
    return s_shell_movie_pending ? s_shell_movie : "";
}

/* Idempotent, mirroring mission_movie_ack's contract: harmless when
 * nothing is queued. */
EMSCRIPTEN_KEEPALIVE
void web_shell_movie_ack(void)
{
    shell_movie_clear();
}

static void catalog_add(const char *path)
{
    if (s_cat_count >= SHELL_CAT_CAP) return;
    if (!loose_exists(path)) return;     /* not staged */
    snprintf(s_cat_path[s_cat_count], sizeof(s_cat_path[0]), "%s", path);
    s_cat_count++;
}

/*
 * Fixed candidate list (DECISION): the shipped miss8 set. Loose files
 * are NOT in the VFS index (vfs_foreach/vfs_exists see the zfs only),
 * so the catalog probes explicit names with loose_exists() — only
 * missions the user actually staged appear. Rebuilt lazily from
 * web_shell_mission_count() because the page stages miss8/ files AFTER
 * web_init() returns.
 */
static void catalog_build(void)
{
    static const char *const nexts[] = { "CBT", "RAC", "CF2",
                                         "CF3", "CF4", "MSN" };
    char path[32];
    s_cat_count = 0;
    catalog_add("miss8/A01.MSN");
    for (int n = 1; n <= 7; n++) {
        snprintf(path, sizeof path, "miss8/S%02d.MSN", n);
        catalog_add(path);
    }
    for (int n = 1; n <= 17; n++) {
        snprintf(path, sizeof path, "miss8/T%02d.MSN", n);
        catalog_add(path);
    }
    for (int n = 1; n <= 15; n++) {
        snprintf(path, sizeof path, "miss8/M%02d.MSN", n);
        catalog_add(path);
    }
    catalog_add("miss8/B01.MSN");
    for (int n = 1; n <= 65; n++)
        for (size_t e = 0; e < sizeof nexts / sizeof nexts[0]; e++) {
            snprintf(path, sizeof path, "miss8/N%02d.%s", n, nexts[e]);
            catalog_add(path);
        }
    for (int p = 1; p <= 20; p++) {
        snprintf(path, sizeof path, "miss8/P%02d.MSN", p);
        catalog_add(path);
    }
}

/* ------------------------------------------------------------------ */
/* Campaign (addon/SCENARIO.DAT) — the trips the game actually ships.  */
/*                                                                     */
/* The catalog above is a filename probe: it answers "what is staged",  */
/* in probe order, under vfs names. It is a developer list. The         */
/* CAMPAIGN is the shipped data: addon/SCENARIO.DAT is 1025 bytes of    */
/* tab-separated CRLF text naming four trips and their 19 missions in   */
/* PLAY order with their display titles:                                */
/*                                                                     */
/*   CHARACTER<TAB><TAB>Taurus<CR>                                      */
/*   MISSION_COUNT<TAB>6<CR>                                            */
/*   A New Hope,<TAB><TAB>p01.msn,<TAB>0,<TAB>3,<TAB>20-1, 07-2, 05-2<CR>*/
/*                                                                     */
/* Field 0 (to the first comma) is the title, field 1 the .msn base     */
/* name. Fields 2/3 ("0", "3") and the trailing "NN-D" triple are not   */
/* decoded (unknown; the shell needs neither). Play order is FILE       */
/* order, NOT numeric: Taurus runs p01, p03, p02, p17, p19, p04, so no  */
/* sort of the catalog can reproduce it.                                */
/*                                                                     */
/* The file is loose in addon/ (it is NOT one of nitro.zfs's 6704       */
/* entries), so it arrives through assets.js like the miss8 tree and is */
/* opened with fs_fopen, whose UPPER/lower basename retry resolves the  */
/* shipped SCENARIO.DAT from the lowercase name below.                  */
/*                                                                     */
/* Its absence is a designed state: s_trip_count stays 0 and the page   */
/* falls back to the staged-mission list.                               */
/* ------------------------------------------------------------------ */

#define CAMP_MISS_CAP 64
#define CAMP_TRIP_CAP  8

static struct {
    char title[40];          /* "A New Hope" */
    char path[32];           /* "miss8/P01.MSN" */
    int  staged;             /* the user staged this mission file */
} s_camp[CAMP_MISS_CAP];

static struct {
    char name[24];           /* "Taurus", "Natty_Dread" */
    int  first, count;       /* slice of s_camp[] */
} s_trip[CAMP_TRIP_CAP];

static int s_camp_count, s_trip_count, s_camp_built;
static int s_camp_trip = -1, s_camp_idx = -1;   /* live campaign position */

/* Trim ASCII space/tab/CR/LF from both ends, in place. The shipped file's
 * padding is irregular ("p02.msn,    <TAB>", "p19.msn, <TAB>", one row
 * ending "07-4<TAB>"), so every extracted field goes through this. */
static void camp_trim(char *s)
{
    size_t n = strlen(s);
    while (n && (s[n - 1] == ' ' || s[n - 1] == '\t' ||
                 s[n - 1] == '\r' || s[n - 1] == '\n'))
        s[--n] = '\0';
    size_t i = 0;
    while (s[i] == ' ' || s[i] == '\t') i++;
    if (i) memmove(s, s + i, n - i + 1);
}

/*
 * Base retail shell data, recovered from i76shell.dll:
 *
 *   Trip         t01..t17, linear progression
 *   Training     a01
 *   Auto Melee   seven fixed scenarios in the shell table's display order
 *   Instant Melee fifteen fixed maps in the shell table's display order
 *
 * The S order is deliberately not lexical.  The titles and 45-byte
 * title/path table order below are the retail data, not port-owned labels.
 * Only Trip is progression-gated; the shell exposes every training/melee
 * row independently of its persisted Trip scene.
 */
struct BaseMission {
    const char *title;
    const char *path;
};

static int base_trip_add(const char *name)
{
    if (s_trip_count >= CAMP_TRIP_CAP) return -1;
    int trip = s_trip_count++;
    snprintf(s_trip[trip].name, sizeof s_trip[trip].name, "%s", name);
    s_trip[trip].first = s_camp_count;
    s_trip[trip].count = 0;
    return trip;
}

static void base_mission_add(int trip, const char *title, const char *path)
{
    if (trip < 0 || s_camp_count >= CAMP_MISS_CAP) return;
    snprintf(s_camp[s_camp_count].title, sizeof s_camp[0].title, "%s", title);
    snprintf(s_camp[s_camp_count].path, sizeof s_camp[0].path, "%s", path);
    s_camp[s_camp_count].staged = loose_exists(path);
    s_camp_count++;
    s_trip[trip].count++;
}

static void base_campaign_build(void)
{
    static const struct BaseMission auto_melee[] = {
        { "Oil Well Well",       "miss8/S03.MSN" },
        { "Mondo Burger",        "miss8/S02.MSN" },
        { "Seminole",            "miss8/S04.MSN" },
        { "Pretty Bus",          "miss8/S05.MSN" },
        { "Race No Shoot",       "miss8/S06.MSN" },
        { "Trailer Pk. Madness", "miss8/S01.MSN" },
        { "Milk Toast",          "miss8/S07.MSN" },
    };
    static const struct BaseMission instant_melee[] = {
        { "The Crater",          "miss8/M01.MSN" },
        { "Dunes",               "miss8/M02.MSN" },
        { "Air Base",            "miss8/M03.MSN" },
        { "Suburbia",            "miss8/M04.MSN" },
        { "Slick Track",         "miss8/M05.MSN" },
        { "Tombstone",           "miss8/M06.MSN" },
        { "Hope Springs",        "miss8/M07.MSN" },
        { "Creeper Canyon",      "miss8/M08.MSN" },
        { "Mesa Maze",           "miss8/M09.MSN" },
        { "Salt Flats",          "miss8/M10.MSN" },
        { "Vigilantes' Paradise", "miss8/M11.MSN" },
        { "Night Driver",        "miss8/M12.MSN" },
        { "Dodge",               "miss8/M13.MSN" },
        { "Jorczak's Peak",      "miss8/M14.MSN" },
        { "A Lot of Asphalt",    "miss8/M15.MSN" },
    };
    char title[40], path[32];

    int trip = base_trip_add("Trip");
    for (int n = 1; n <= 17; n++) {
        snprintf(title, sizeof title, "Trip %d", n);
        snprintf(path, sizeof path, "miss8/T%02d.MSN", n);
        base_mission_add(trip, title, path);
    }

    int training = base_trip_add("Training");
    base_mission_add(training, "Training", "miss8/A01.MSN");

    int scenarios = base_trip_add("Auto Melee");
    for (size_t i = 0; i < sizeof auto_melee / sizeof auto_melee[0]; i++)
        base_mission_add(scenarios, auto_melee[i].title, auto_melee[i].path);

    int melee = base_trip_add("Instant Melee");
    for (size_t i = 0; i < sizeof instant_melee / sizeof instant_melee[0]; i++)
        base_mission_add(melee, instant_melee[i].title,
                         instant_melee[i].path);
}

static void campaign_build(void)
{
    s_camp_count = 0;
    s_trip_count = 0;
    s_camp_built = 1;
    if (vfs_profile() == VFS_PROFILE_BASE) {
        base_campaign_build();
        return;
    }

    FILE *f = fs_fopen("addon/scenario.dat", "rb");
    if (!f) return;                    /* Nitro metadata not staged */
    char buf[4096];
    size_t n = fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n] = '\0';

    /* Manual line walk: the shipped file has NO trailing newline, so the
     * last record must still be emitted. */
    for (size_t pos = 0; pos < n; ) {
        size_t eol = pos;
        while (eol < n && buf[eol] != '\n') eol++;
        char line[256];
        size_t len = eol - pos;
        if (len >= sizeof line) len = sizeof line - 1;
        memcpy(line, buf + pos, len);
        line[len] = '\0';
        pos = eol + 1;
        camp_trim(line);
        if (!line[0]) continue;

        if (strncmp(line, "CHARACTER", 9) == 0) {
            if (s_trip_count >= CAMP_TRIP_CAP) break;
            char name[24];
            snprintf(name, sizeof name, "%s", line + 9);
            camp_trim(name);
            if (!name[0]) continue;
            snprintf(s_trip[s_trip_count].name, sizeof s_trip[0].name,
                     "%s", name);
            s_trip[s_trip_count].first = s_camp_count;
            s_trip[s_trip_count].count = 0;
            s_trip_count++;
            continue;
        }
        if (strncmp(line, "MISSION_COUNT", 13) == 0)
            continue;                  /* rows are counted as they parse */
        if (s_trip_count == 0) continue;   /* stray line before any trip */

        char *c1 = strchr(line, ',');
        if (!c1) continue;
        *c1 = '\0';
        char title[40];
        snprintf(title, sizeof title, "%s", line);
        camp_trim(title);

        char *rest = c1 + 1;
        char *c2 = strchr(rest, ',');
        if (c2) *c2 = '\0';
        camp_trim(rest);
        size_t rl = strlen(rest);
        if (rl < 5 || rl > 20 || strcasecmp(rest + rl - 4, ".msn") != 0)
            continue;                  /* not a mission row */
        if (!title[0]) snprintf(title, sizeof title, "%s", rest);

        if (s_camp_count >= CAMP_MISS_CAP) continue;
        char path[32];
        snprintf(path, sizeof path, "miss8/%s", rest);
        for (char *p = path + 6; *p; p++)
            *p = (char)toupper((unsigned char)*p);

        snprintf(s_camp[s_camp_count].title, sizeof s_camp[0].title,
                 "%s", title);
        snprintf(s_camp[s_camp_count].path, sizeof s_camp[0].path,
                 "%s", path);
        /* Per-row truth: a partial stage yields an honest menu instead of
         * rows that fail on pick. Same probe the catalog uses. */
        s_camp[s_camp_count].staged = loose_exists(path);
        s_camp_count++;
        s_trip[s_trip_count - 1].count++;
    }
}

/* Files stage AFTER web_init(), and more can arrive from a second drop —
 * build once, and let the page force a rebuild when it restages. */
static void campaign_ensure(void)
{
    if (!s_camp_built) campaign_build();
}

static int camp_index(int t, int i)
{
    campaign_ensure();
    if (t < 0 || t >= s_trip_count) return -1;
    if (i < 0 || i >= s_trip[t].count) return -1;
    return s_trip[t].first + i;
}

EMSCRIPTEN_KEEPALIVE
void web_campaign_refresh(void) { s_camp_built = 0; }

EMSCRIPTEN_KEEPALIVE
int web_campaign_trip_count(void) { campaign_ensure(); return s_trip_count; }

EMSCRIPTEN_KEEPALIVE
const char *web_campaign_trip_name(int t)
{
    campaign_ensure();
    return (t >= 0 && t < s_trip_count) ? s_trip[t].name : "";
}

EMSCRIPTEN_KEEPALIVE
int web_campaign_mission_count(int t)
{
    campaign_ensure();
    return (t >= 0 && t < s_trip_count) ? s_trip[t].count : 0;
}

EMSCRIPTEN_KEEPALIVE
const char *web_campaign_mission_title(int t, int i)
{
    int k = camp_index(t, i);
    return k < 0 ? "" : s_camp[k].title;
}

EMSCRIPTEN_KEEPALIVE
const char *web_campaign_mission_path(int t, int i)
{
    int k = camp_index(t, i);
    return k < 0 ? "" : s_camp[k].path;
}

EMSCRIPTEN_KEEPALIVE
int web_campaign_mission_staged(int t, int i)
{
    int k = camp_index(t, i);
    return k < 0 ? 0 : s_camp[k].staged;
}

/* ------------------------------------------------------------------ */
/* Garage catalog — the models the original shell offers.              */
/*                                                                     */
/* nitshell.dll carries two tables of {u32 id; u32 n; u32 n; char[16]  */
/* model}, stride 28: 37 records at file offset 0x42b40 and 31 at      */
/* 0x42f50. The 31-record table is this list; it is the 37 minus the   */
/* heavy rigs and the bus (vagrapl, vmcargo, vmload, vmxmarx, vxbus,   */
/* vxmilk). Both tables were read out of the DLL bytes directly and    */
/* the record layout confirmed on every row.                           */
/*                                                                     */
/* WHY EMBEDDED. The game writes nitscar.def at runtime, byte-identical*/
/* to the DLL's table; reading it would work on a machine that has     */
/* already run the original and fail for a purchaser who has not.      */
/*                                                                     */
/* WHY THE COUNTS ARE NOT TRUSTED. The table's variant count disagrees */
/* with the shipped .vcf set in two rows (vagrapl and vsvan both say 1 */
/* and ship 2). Variants are therefore PROBED through the VFS, which   */
/* repairs both and keeps the menu honest about the user's own data.   */
/* The second u32 is undecoded (it equals the first in every row).     */
/* ------------------------------------------------------------------ */

static const char *const s_veh_model[] = {
    "valepre", "vastryd", "vazamz",  "vccameo", "vchacnd", "vcgcour",
    "vcmanta", "vcroyal", "vcpolic", "vcvan",   "vdlight", "vdrampg",
    "vdstag",  "vfcoupe", "vfdclyd", "vfpalmn", "vfratlr", "vfypony",
    "vgoon",   "vjdread", "vjsovrn", "vleoprd", "vpjrabt", "vppirna",
    "vrmarsh", "vsvan",   "vavikea", "vwclown", "vwfwolf", "vxamblc",
    "vxhears",
};
#define VEH_MODELS ((int)(sizeof s_veh_model / sizeof s_veh_model[0]))
#define VEH_MAX_VARIANTS 8

EMSCRIPTEN_KEEPALIVE
int web_veh_count(void) { return VEH_MODELS; }

EMSCRIPTEN_KEEPALIVE
const char *web_veh_model(int i)
{
    return (i >= 0 && i < VEH_MODELS) ? s_veh_model[i] : "";
}

/* Variants actually present in the user's archive, highest numbered
 * first found; 0 when the model ships in neither. */
EMSCRIPTEN_KEEPALIVE
int web_veh_variants(int i)
{
    if (i < 0 || i >= VEH_MODELS) return 0;
    int n = 0;
    for (int v = 1; v <= VEH_MAX_VARIANTS; v++) {
        char f[32];
        snprintf(f, sizeof f, "%s%d.vcf", s_veh_model[i], v);
        if (vfs_exists(f)) n = v;
    }
    return n;
}

/* Base name for car_load/web_shell_set_vehicle ("vdrampg2"). Empty when
 * that variant is not in the archive, so the page cannot offer a car the
 * user's own data does not have. */
EMSCRIPTEN_KEEPALIVE
const char *web_veh_vcf(int i, int variant)
{
    static char buf[32];
    buf[0] = '\0';
    if (i < 0 || i >= VEH_MODELS || variant < 1 || variant > VEH_MAX_VARIANTS)
        return buf;
    char f[32];
    snprintf(f, sizeof f, "%s%d.vcf", s_veh_model[i], variant);
    if (!vfs_exists(f)) return buf;
    snprintf(buf, sizeof buf, "%s%d", s_veh_model[i], variant);
    return buf;
}

/*
 * Human-facing chassis titles for the garage. The original shell used
 * English labels from nitshell; we keep a curated map of the Nitro table
 * and fall back to the model code. Variant number is not part of the
 * display name (the page appends it).
 */
EMSCRIPTEN_KEEPALIVE
const char *web_veh_title(int i)
{
    static const struct { const char *model; const char *title; } titles[] = {
        { "valepre", "Alley Cat" },
        { "vastryd", "Stray Dog" },
        { "vazamz",  "Zamz" },
        { "vccameo", "Cameo" },
        { "vchacnd", "Hacienda" },
        { "vcgcour", "Courrier" },
        { "vcmanta", "Manta" },
        { "vcroyal", "Royal" },
        { "vcpolic", "Police Interceptor" },
        { "vcvan",   "Cargo Van" },
        { "vdlight", "Light Pickup" },
        { "vdrampg", "Dover Rampage" },
        { "vdstag",  "Stag" },
        { "vfcoupe", "Coupe" },
        { "vfdclyd", "Clyde" },
        { "vfpalmn", "Palm" },
        { "vfratlr", "Rattler" },
        { "vfypony", "Pony" },
        { "vgoon",   "Goon" },
        { "vjdread", "Natty Dread" },
        { "vjsovrn", "Jefferson Sovereign" },
        { "vleoprd", "Leopard" },
        { "vpjrabt", "Jackrabbit" },
        { "vppirna", "Picard Piranha" },
        { "vrmarsh", "Marshal" },
        { "vsvan",   "Service Van" },
        { "vavikea", "Viking" },
        { "vwclown", "Clown Car" },
        { "vwfwolf", "Wolf" },
        { "vxamblc", "Ambulance" },
        { "vxhears", "Hearse" },
    };
    if (i < 0 || i >= VEH_MODELS) return "";
    for (size_t t = 0; t < sizeof titles / sizeof titles[0]; t++)
        if (strcmp(s_veh_model[i], titles[t].model) == 0)
            return titles[t].title;
    return s_veh_model[i];
}

/* Garage preview cache: spin/orbit reuses the loaded car + dyn queue. */
static char s_garage_vcf[32];
static int  s_garage_nparts;
static double s_garage_radius = 2.5;

static int garage_json_escape(char *dst, size_t cap, const char *src)
{
    size_t o = 0;
    if (!dst || cap == 0) return 0;
    if (!src) src = "";
    for (; *src && o + 2 < cap; src++) {
        char c = *src;
        if (c == '"' || c == '\\') {
            if (o + 3 >= cap) break;
            dst[o++] = '\\';
            dst[o++] = c;
        } else if ((unsigned char)c < 32) {
            continue;
        } else {
            dst[o++] = c;
        }
    }
    dst[o] = '\0';
    return (int)o;
}

static uint32_t garage_rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void garage_fixed_string(char *out, size_t cap,
                                const uint8_t *p, size_t n)
{
    if (!out || cap == 0) return;
    if (n > cap - 1) n = cap - 1;
    memcpy(out, p, n);
    out[n] = '\0';
    for (size_t i = 0; i < n; i++)
        if ((unsigned char)out[i] < 32 || (unsigned char)out[i] > 126) {
            out[i] = '\0';
            break;
        }
}

static const char *garage_component_name(char kind, uint32_t id)
{
    static char name[32];
    name[0] = '\0';
    size_t sz = 0;
    uint8_t *buf = vfs_read_file("compnent.cdf", &sz);
    if (!buf) return name;
    const char *tag = kind == 'e' ? "NTBL" : kind == 'b' ? "BTBL" : "STBL";
    for (size_t off = 0; off + 8 <= sz; ) {
        uint32_t total = garage_rd_u32(buf + off + 4);
        if (total < 8 || total > sz - off) break;
        if (memcmp(buf + off, tag, 4) == 0 && total >= 12) {
            uint32_t count = garage_rd_u32(buf + off + 8);
            /* VCFC component ids are one-based; NTBL/BTBL/STBL are
             * zero-based fixed 16-byte display-name slots. Do arithmetic in
             * size_t after division-based validation: staged CDF/VCF bytes are
             * untrusted and uint32 multiplication may wrap. */
            size_t payload = (size_t)total - 12u;
            if (id > 0 && id <= count && (size_t)id <= payload / 16u) {
                size_t slot = ((size_t)id - 1u) * 16u;
                garage_fixed_string(name, sizeof name,
                                    buf + off + 12u + slot, 16);
            }
            break;
        }
        off += total;
    }
    vfs_free(buf);
    return name;
}

static void garage_wheel_name(char out[32], const char *file)
{
    out[0] = '\0';
    if (!file || !*file || !strcasecmp(file, "null")) return;
    size_t sz = 0;
    uint8_t *buf = vfs_read_file(file, &sz);
    if (!buf) return;
    for (size_t off = 0; off + 8 <= sz; ) {
        uint32_t total = garage_rd_u32(buf + off + 4);
        if (total < 8 || total > sz - off) break;
        if (memcmp(buf + off, "WDFC", 4) == 0 && total >= 28) {
            garage_fixed_string(out, 32, buf + off + 8, 20);
            break;
        }
        off += total;
    }
    vfs_free(buf);
}

static void garage_paint_name(char out[32], const char *file)
{
    out[0] = '\0';
    if (!file || !*file) return;
    size_t sz = 0;
    uint8_t *buf = vfs_read_file(file, &sz);
    if (!buf) return;
    for (size_t off = 0; off + 8 <= sz; ) {
        uint32_t total = garage_rd_u32(buf + off + 4);
        if (total < 8 || total > sz - off) break;
        if (memcmp(buf + off, "VTFC", 4) == 0 && total >= 8 + 29) {
            garage_fixed_string(out, 32, buf + off + 8 + 13, 16);
            break;
        }
        off += total;
    }
    vfs_free(buf);
}

static const char *garage_special_name(int id)
{
    static const char *const names[] = {
        "", "Radar Jammer", "Nitrous Oxide", "Blower", "Xhaust Brake",
        "Structo Bmpr", "Curb Feelers", "Mud Flaps", "Heated Seats",
        "Cup Holders"
    };
    return id >= 0 && id < (int)(sizeof names / sizeof names[0])
         ? names[id] : "Special";
}

/*
 * Chassis-form data for the selected purchaser VCF. Loading here is a
 * menu-only side effect; the drive path reloads the committed selection.
 */
EMSCRIPTEN_KEEPALIVE
const char *web_garage_info(const char *vcf)
{
    static char buf[2048];
    buf[0] = '\0';
    if (!vcf || !*vcf) {
        snprintf(buf, sizeof buf, "{\"ok\":0,\"err\":\"empty\"}");
        return buf;
    }
    if (car_load(vcf) != 0) {
        snprintf(buf, sizeof buf, "{\"ok\":0,\"err\":\"load\"}");
        s_garage_vcf[0] = '\0';
        return buf;
    }
    snprintf(s_garage_vcf, sizeof s_garage_vcf, "%s", vcf);
    s_garage_radius = car_collision_radius();
    if (!(s_garage_radius > 0.5)) s_garage_radius = 2.5;

    uint32_t ids[3] = {0}, armor[4] = {0}, chassis[4] = {0}, left = 0;
    (void)car_component_ids(ids);
    (void)car_defense(armor, chassis, &left);

    char ch[48], va[40], vt[32], paint[40], engine[40], susp[40], brake[40];
    char wheel[3][40];
    garage_json_escape(ch, sizeof ch, car_chassis_name());
    garage_json_escape(va, sizeof va, car_variant_name());
    garage_json_escape(vt, sizeof vt, car_vtf_file() ? car_vtf_file() : "");
    char raw[32];
    garage_paint_name(raw, car_vtf_file());
    garage_json_escape(paint, sizeof paint, raw);
    /* The VCFC engine dword is engsnd.dat's ENG NUM, not an NTBL row
     * (engine-index-mapping.md); FUN_004532f0 maps it to the zero-based
     * ENG COMP ID that names the installed engine. */
    ComponentEngineCurve eng_curve;
    uint32_t eng_row = component_engine_curve(ids[0], &eng_curve) == 0
                     ? eng_curve.component_id + 1u : 0u;
    garage_json_escape(engine, sizeof engine, garage_component_name('e', eng_row));
    garage_json_escape(susp, sizeof susp, garage_component_name('s', ids[1]));
    garage_json_escape(brake, sizeof brake, garage_component_name('b', ids[2]));
    for (int i = 0; i < 3; i++) {
        garage_wheel_name(raw, car_wheel_file(i));
        garage_json_escape(wheel[i], sizeof wheel[i], raw);
    }

    int n = snprintf(buf, sizeof buf,
                     "{\"ok\":1,\"vcf\":\"%s\",\"chassis\":\"%s\","
                     "\"variant\":\"%s\",\"parts\":%d,\"vtf\":\"%s\","
                     "\"paint\":\"%s\",\"radius\":%.2f,\"weight\":%.0f,"
                     "\"engine\":\"%s\",\"suspension\":\"%s\","
                     "\"brakes\":\"%s\",\"wheels\":[\"%s\",\"%s\",\"%s\"],"
                     "\"armor\":[%u,%u,%u,%u],\"chassisDefense\":[%u,%u,%u,%u],"
                     "\"left\":%u,\"specials\":[",
                     vcf, ch, va, car_part_count(), vt, paint, s_garage_radius,
                     car_config_weight_lb(), engine, susp, brake,
                     wheel[0], wheel[1], wheel[2],
                     armor[0], armor[1], armor[2], armor[3],
                     chassis[0], chassis[1], chassis[2], chassis[3], left);
    int first = 1;
    for (int i = 0; i < car_special_count() && n > 0 &&
                        (size_t)n < sizeof buf - 96; i++) {
        char sn[40];
        garage_json_escape(sn, sizeof sn, garage_special_name(car_special_id(i)));
        n += snprintf(buf + n, sizeof buf - (size_t)n,
                      "%s\"%s\"", first ? "" : ",", sn);
        first = 0;
    }
    if (n > 0 && (size_t)n < sizeof buf - 16) {
        n += snprintf(buf + n, sizeof buf - (size_t)n, "],\"weapons\":[");
        first = 1;
    }
    for (int i = 0; i < car_weapon_count() && n > 0 &&
                        (size_t)n < sizeof buf - 96; i++) {
        CarWeaponInfo wi;
        if (car_weapon_get(i, &wi) != 0) continue;
        char wn[48];
        garage_json_escape(wn, sizeof wn,
                           wi.name && wi.name[0] ? wi.name : "weapon");
        n += snprintf(buf + n, sizeof buf - (size_t)n,
                      "%s{\"name\":\"%s\",\"ammo\":%d,\"dmg\":%d,"
                      "\"rear\":%d,\"direct\":%d}",
                      first ? "" : ",", wn, wi.ammo, wi.damage,
                      wi.rear_facing, wi.direct_fire);
        first = 0;
    }
    if (n > 0 && (size_t)n < sizeof buf - 4)
        snprintf(buf + n, sizeof buf - (size_t)n, "]}");
    return buf;
}

/* Rebuild the dyn part queue for the loaded garage car. */
static int garage_queue_parts(void)
{
    scene_set_dynamic_paint(car_vtf_file());
    scene_dyn_clear();
    double basis[12] = {
        1, 0, 0,
        0, 1, 0,
        0, 0, 1,
        0, 0, 0,
    };
    int nparts = 0;
    int n = car_part_count();
    if (n > SCENE_DYN_MAX) n = SCENE_DYN_MAX;
    for (int i = 0; i < n; i++) {
        GeoMesh *m = scene_part_mesh(car_part_name(i));
        double fr[12];
        if (!m || car_part_frame(i, fr) != 0) continue;
        double xf[12];
        basis_compose_frame(xf, basis, fr);
        (void)scene_dyn_add_textured(m, xf, xf + 3, xf + 6, xf + 9, NULL);
        nparts++;
    }
    for (int wi = 0; wi < car_weapon_count() && nparts < SCENE_DYN_MAX; wi++) {
        for (int pi = 0; pi < car_weapon_part_count(wi) &&
                         nparts < SCENE_DYN_MAX; pi++) {
            GeoMesh *m = scene_part_mesh(car_weapon_part_name(wi, pi));
            double fr[12];
            if (!m || car_weapon_part_frame(wi, pi, fr) != 0) continue;
            double xf[12];
            basis_compose_frame(xf, basis, fr);
            (void)scene_dyn_add_textured(m, xf, xf + 3, xf + 6, xf + 9, NULL);
            nparts++;
        }
    }
    s_garage_nparts = nparts;
    return nparts;
}

/* Project world XZ at y=0 into screen pixels (same basis as camera_view). */
static int garage_project(const CameraView *cam, double wx, double wy, double wz,
                          double f_px, int *sx, int *sy)
{
    double dx = wx - cam->eye[0];
    double dy = wy - cam->eye[1];
    double dz = wz - cam->eye[2];
    double z = dx * cam->forward[0] + dy * cam->forward[1] + dz * cam->forward[2];
    if (!(z > 0.05)) return -1;
    double x = dx * cam->right[0] + dy * cam->right[1] + dz * cam->right[2];
    double y = dx * cam->up[0] + dy * cam->up[1] + dz * cam->up[2];
    double invz = 1.0 / z;
    *sx = (int)(MESHVIEW_FB_W * 0.5 + x * f_px * invz);
    *sy = (int)(MESHVIEW_FB_H * 0.5 - y * f_px * invz);
    return 0;
}

static void garage_draw_ground(const CameraView *cam)
{
    /* Contact shadow under the car: darken an ellipse at the projected
     * ground point. Blends instead of replacing so car paint is not erased. */
    double r = s_garage_radius * 1.25;
    if (!(r > 0.5)) r = 2.5;
    double f_px = (MESHVIEW_FB_W * 0.5) / camera_view_fov_tan_half(cam);
    uint8_t shadow = raster_rgb_to_index(12, 10, 8);
    uint8_t ring = raster_rgb_to_index(42, 34, 24);

    int cx, cy, ex, ey;
    if (garage_project(cam, 0, 0.02, 0, f_px, &cx, &cy) != 0) return;
    if (garage_project(cam, r, 0.02, 0, f_px, &ex, &ey) != 0) return;
    int rx = abs(ex - cx);
    if (rx < 14) rx = 14;
    int ry = rx / 3;
    if (ry < 7) ry = 7;

    /* Bias the ellipse slightly downward on screen so it reads as a floor. */
    cy += ry / 4;

    for (int y = -ry; y <= ry; y++) {
        for (int x = -rx; x <= rx; x++) {
            double u = (double)x / (double)rx;
            double v = (double)y / (double)ry;
            double d = u * u + v * v;
            if (d > 1.0) continue;
            int px = cx + x, py = cy + y;
            if ((unsigned)px >= (unsigned)MESHVIEW_FB_W ||
                (unsigned)py >= (unsigned)MESHVIEW_FB_H)
                continue;
            /* Keep the disc under the car: skip upper frame and bright
             * body paint so wheels/hood are not stamped over. */
            if (py < MESHVIEW_FB_H * 45 / 100) continue;
            uint8_t *p = &s_fb[py * MESHVIEW_FB_W + px];
            uint8_t pr = s_pal[*p].r, pg = s_pal[*p].g, pb = s_pal[*p].b;
            if ((int)pr + (int)pg + (int)pb > 90) continue;
            *p = (d > 0.78) ? ring : shadow;
        }
    }
}

/*
 * Garage 3-D preview: load VCF (cached), queue exterior parts with paint,
 * render a fitted orbit view into the shared framebuffer. yaw in radians.
 * Returns geometry pixel count (depth coverage), or negative on failure.
 * Menu-only.
 */
EMSCRIPTEN_KEEPALIVE
int web_garage_preview(const char *vcf, double yaw)
{
    if (!vcf || !*vcf) return -1;

    int need_load = (strcmp(s_garage_vcf, vcf) != 0) || !car_is_loaded();
    if (need_load) {
        if (car_load(vcf) != 0) {
            s_garage_vcf[0] = '\0';
            return -1;
        }
        snprintf(s_garage_vcf, sizeof s_garage_vcf, "%s", vcf);
        car_place(0.0, 0.0, 0.0);
        s_garage_radius = car_collision_radius();
        if (!(s_garage_radius > 0.5)) s_garage_radius = 2.5;
        if (garage_queue_parts() == 0) return -2;
    } else if (s_garage_nparts == 0) {
        if (garage_queue_parts() == 0) return -2;
    }

    /* Neutral indexed palette for the transparent line drawing. Index zero
     * is the canvas key; wire and vertex indices become dark ink. */
    {
        uint8_t pal[768] = {0};
        for (int i = 0; i < 256; i++) {
            pal[i * 3 + 0] = 236;
            pal[i * 3 + 1] = 205;
            pal[i * 3 + 2] = 126;
        }
        pal[3] = 50; pal[4] = 45; pal[5] = 32;
        pal[6] = 78; pal[7] = 68; pal[8] = 45;
        raster_set_palette(pal);
        for (int i = 0; i < 256; i++) {
            s_pal[i].r = pal[i * 3 + 0];
            s_pal[i].g = pal[i * 3 + 1];
            s_pal[i].b = pal[i * 3 + 2];
        }
    }

    /* Fit distance to chassis size; keep a little air around larger vans. */
    double dist = s_garage_radius * 3.6 + 2.0;
    if (dist < 7.0) dist = 7.0;
    if (dist > 16.0) dist = 16.0;
    double elev = s_garage_radius * 0.85 + 1.4;
    if (elev < 2.4) elev = 2.4;
    if (elev > 5.5) elev = 5.5;

    double eye[3] = {
        sin(yaw) * dist,
        elev,
        -cos(yaw) * dist
    };
    /* The retail chassis form's vehicle drawing is large and reads as an
     * elevation rather than a showroom long shot. Tighten the fit while the
     * 640x480 canvas is later placed in the right-hand drawing field. */
    /* The entry form gives this drawing only half a sheet. At the old 0.58
     * fit the real vehicle occupied a thumbnail-sized patch and purchasers
     * reasonably read the portrait as missing. This remains the authored
     * VCF/VDF mesh the original shell's vehscn scene loads, just framed as
     * the form's primary vehicle drawing rather than a distant showroom. */
    dist *= 0.33;
    eye[0] = sin(yaw) * dist;
    eye[1] = elev * 0.82;
    eye[2] = -cos(yaw) * dist;
    double target[3] = { 0.0, s_garage_radius * 0.24, 0.0 };
    CameraView cam;
    if (camera_view_look_at(&cam, eye, target) != 0) return -3;

    scene_render_camera(s_fb, MESHVIEW_FB_W, MESHVIEW_FB_H, &cam);
    int pixels = 0;
    for (int i = 0; i < MESHVIEW_FB_W * MESHVIEW_FB_H; i++)
        pixels += s_fb[i] != 0;
    return pixels;
}

/* Drop the garage preview cache (call when leaving the menu if desired). */
EMSCRIPTEN_KEEPALIVE
void web_garage_release(void)
{
    s_garage_vcf[0] = '\0';
    s_garage_nparts = 0;
    scene_dyn_clear();
    build_palette(s_pal);
    uint8_t pal[768];
    for (int i = 0; i < 256; i++) {
        pal[i * 3 + 0] = s_pal[i].r;
        pal[i * 3 + 1] = s_pal[i].g;
        pal[i * 3 + 2] = s_pal[i].b;
    }
    raster_set_palette(pal);
}

static uint32_t sh_rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Pull a null-padded WRLD string field into `out`, printable ASCII only
 * (same sanitation as hud.c's mission_palette_name); `lower` folds case,
 * which the FILENAME fields need and the display name must not have. */
static void wrld_field(const uint8_t *p, size_t len, char *out, size_t outcap,
                       int lower)
{
    if (len > outcap - 1) len = outcap - 1;
    memcpy(out, p, len);
    out[len] = '\0';
    for (size_t i = 0; out[i]; i++) {
        if (lower && out[i] >= 'A' && out[i] <= 'Z')
            out[i] = (char)(out[i] + ('a' - 'A'));
        else if (out[i] < 32 || out[i] > 126) {
            out[i] = '\0';
            break;
        }
    }
}

/*
 * Read a fixed-width string field out of the mission's WDEF/WRLD chunk
 * (scene.md §2, CONFIRMED): top-level chunk stream -> WDEF -> sub-chunk
 * stream -> WRLD. Chunk size fields INCLUDE the 8-byte header (same walk
 * as hud.c). The catalog stores full vfs paths, so no dir-prefix search
 * is needed. Returns 0 and fills `out` on success.
 */
static int mission_wrld_field(const char *mission, size_t field_off,
                              size_t field_len, char *out, size_t outcap,
                              int lower)
{
    size_t sz = 0;
    uint8_t *buf = vfs_read_file(mission, &sz);
    if (!buf) return -1;
    int found = -1;
    for (size_t off = 0; off + 8 <= sz && found < 0; ) {
        uint32_t total = sh_rd_u32(buf + off + 4);
        if (total < 8 || (size_t)total > sz - off) break;
        if (memcmp(buf + off, "WDEF", 4) == 0) {
            size_t end = off + total;
            for (size_t p = off + 8; p + 8 <= end; ) {
                uint32_t sub = sh_rd_u32(buf + p + 4);
                if (sub < 8 || (size_t)sub > end - p) break;
                if (memcmp(buf + p, "WRLD", 4) == 0 &&
                    8 + field_off + field_len <= (size_t)sub) {
                    char name[32];
                    wrld_field(buf + p + 8 + field_off, field_len,
                               name, sizeof name, lower);
                    if (name[0]) {
                        snprintf(out, outcap, "%s", name);
                        found = 0;
                    }
                    break;
                }
                p += sub;
            }
            break;
        }
        off += total;
    }
    vfs_free(buf);
    return found;
}

/* The objective-text file (.npt): 13-byte field at WRLD payload +69. */
static int mission_npt_name(const char *mission, char out[14])
{
    return mission_wrld_field(mission, 69, 13, out, 14, 1);
}

/*
 * The mission's OWN display name: the last 16 bytes of the 331-byte WRLD
 * payload, NUL-padded ASCII. Confirmed by cross-checking it against a
 * source that does not come from the mission file at all — P01.MSN's
 * field reads "A New Hope", which is exactly the title addon/SCENARIO.DAT
 * gives that mission. The arena maps are not in SCENARIO.DAT and this is
 * the only place their names exist: N21.RAC "Cactus Mountain", N23.RAC
 * "Drag Race", N34.RAC "Drinkys Delight", N01.CBT "Evening Stroll".
 */
static int mission_display_name(const char *mission, char out[20])
{
    return mission_wrld_field(mission, 315, 16, out, 20, 0);
}

/* ------------------------------------------------------------------ */
/* The arena list — the modes the original's own shell names.          */
/*                                                                     */
/* addon/SCENARIO.DAT lists the 19 TRIP missions and nothing else, so   */
/* the 55 arena maps had no consumer route into the game at all: they   */
/* appeared only in the ?dev=1 filename probe. The reason for that was  */
/* good — nothing FSM-less could terminate, so listing them would have  */
/* offered a player a mission with no ending. mission.c's objective     */
/* controller retires that reason for RACE and now for CAPTURE, and     */
/* this is their menu. web_arena_family(i) says which one a row is, so  */
/* the page can tab them apart without a second list.                   */
/*                                                                     */
/* MELEE used to be held off this list because nothing in a .CBT file  */
/* is an opponent, so it would have offered a mission a player could   */
/* not finish. mission.c D-O14 SPAWNS the opponents into the map's own  */
/* empty grid slots, which is what the original's SpawnLoc module does  */
/* with arriving network vehicles, so melee is a real mode now and is   */
/* listed with the other two. A map whose grid cannot field a fight     */
/* still builds no controller; the row is offered because all 24 can.   */
/* ------------------------------------------------------------------ */

#define ARENA_CAP 64        /* 13 races + 17 capture + 24 melee maps */

static struct {
    char path[32];       /* catalog path, "miss8/N21.RAC"               */
    char name[20];       /* the mission's own WRLD name, "Cactus Mountain" */
    int  family;         /* MISSION_FAMILY_MELEE / _RACE / _CAPTURE     */
} s_arena[ARENA_CAP];
static int s_arena_count, s_arena_built;

static void arena_build(void)
{
    s_arena_count = 0;
    s_arena_built = 1;
    catalog_build();          /* same staged-file probe the trips use */
    for (int i = 0; i < s_cat_count && s_arena_count < ARENA_CAP; i++) {
        int fam = mission_family_of_path(s_cat_path[i]);
        if (fam != MISSION_FAMILY_MELEE && fam != MISSION_FAMILY_RACE &&
            fam != MISSION_FAMILY_CAPTURE)
            continue;
        snprintf(s_arena[s_arena_count].path,
                 sizeof s_arena[0].path, "%s", s_cat_path[i]);
        s_arena[s_arena_count].family = fam;
        /* Fall back to the filename when the WRLD name is unreadable —
         * an unnamed row is still playable, a missing row is not. */
        if (mission_display_name(s_cat_path[i],
                                 s_arena[s_arena_count].name) != 0) {
            const char *b = strrchr(s_cat_path[i], '/');
            snprintf(s_arena[s_arena_count].name, sizeof s_arena[0].name,
                     "%s", b ? b + 1 : s_cat_path[i]);
        }
        s_arena_count++;
    }
}

static void arena_ensure(void) { if (!s_arena_built) arena_build(); }

EMSCRIPTEN_KEEPALIVE
void web_arena_refresh(void) { s_arena_built = 0; }

EMSCRIPTEN_KEEPALIVE
int web_arena_count(void) { arena_ensure(); return s_arena_count; }

EMSCRIPTEN_KEEPALIVE
const char *web_arena_name(int i)
{
    arena_ensure();
    return (i >= 0 && i < s_arena_count) ? s_arena[i].name : "";
}

EMSCRIPTEN_KEEPALIVE
const char *web_arena_path(int i)
{
    arena_ensure();
    return (i >= 0 && i < s_arena_count) ? s_arena[i].path : "";
}

/* MISSION_FAMILY_MELEE (1), _RACE (2) or _CAPTURE (3); -1 out of range. */
EMSCRIPTEN_KEEPALIVE
int web_arena_family(int i)
{
    arena_ensure();
    return (i >= 0 && i < s_arena_count) ? s_arena[i].family : -1;
}

/* ------------------------------------------------------------------ */
/* Arena session rules — the original's own game-setup options.        */
/*                                                                     */
/* nitshell.dll carries them as one adjacent block of formats:         */
/* "No Restrictions", "Not a team game", "No Limits!", "%d Minutes",   */
/* "%d Kills", "%d Points", "%d Captures", "%d Laps". They are SESSION */
/* settings there and they are session settings here — there is no     */
/* per-mission rules record in the data to read them from (see         */
/* mission_set_rules). The defaults are INVENTED (mission.c D-O4).     */
/* ------------------------------------------------------------------ */

static int s_arena_laps = 1, s_arena_captures = 1, s_arena_kills = 1,
           s_arena_minutes = 10;

EMSCRIPTEN_KEEPALIVE
void web_shell_set_rules(int lap_target, int capture_target, int kill_target,
                         int limit_minutes)
{
    s_arena_laps     = lap_target     > 0 ? lap_target     : 1;
    s_arena_captures = capture_target > 0 ? capture_target : 1;
    s_arena_kills    = kill_target    > 0 ? kill_target    : 1;
    s_arena_minutes  = limit_minutes  > 0 ? limit_minutes  : 0;
    mission_set_rules(s_arena_laps, s_arena_captures, s_arena_kills,
                      s_arena_minutes);
}

EMSCRIPTEN_KEEPALIVE
int web_shell_rule_laps(void) { return s_arena_laps; }

EMSCRIPTEN_KEEPALIVE
int web_shell_rule_captures(void) { return s_arena_captures; }

EMSCRIPTEN_KEEPALIVE
int web_shell_rule_kills(void) { return s_arena_kills; }

EMSCRIPTEN_KEEPALIVE
int web_shell_rule_minutes(void) { return s_arena_minutes; }

/*
 * Build the briefing screen text for the picked mission.
 *
 * .npt format (verified against p01.npt/m01.npt — p01 quoted in the M5
 * report): objective lines, then a "(failure)" marker line, then the
 * fail-reason texts. The briefing is the objective lines verbatim
 * (CRLF normalized to LF), "(hidden)" markers kept — they are the
 * data (DECISION). Nitro ships no lang.txt (strlookup.h), so the
 * staged filename doubles as the mission name (DECISION).
 *
 * RACES AND CAPTURE MAPS DO NOT GET THAT TEXT. Every .RAC and .CF* in the
 * set points its WRLD .npt field at the SHARED m01.npt, whose objective
 * reads "Those other cars are out to get you. Get them first!" and whose
 * failure line is "Ouch. Doubt you can drive in that condition. Can we say
 * traction?". On a race or a capture map that describes neither the
 * objective nor the only way the mission can be lost, so those two state
 * the actual rules instead — which are the session's, and which the player
 * just chose.
 *
 * A MELEE IS THE ONE ARENA FAMILY THAT KEEPS ITS SHIPPED TEXT, for two
 * reasons. Thirteen of the 24 .CBT maps do not share m01.npt at all —
 * they name their own (m03..m15.npt, e.g. N64.CBT's "Only the best drivers
 * survive the climb. Of those, only the best gunners survive the top.").
 * And the shared text, where it IS shared, is TRUE of a melee since
 * mission.c D-O14: there really are other cars, they really are out to get
 * you, getting them first really is the win, and the failure line really
 * does describe death by damage. So melee's briefing prose is the GAME'S,
 * verbatim, and this port authors none of it — a strictly better position
 * than the other two, and the reason worth stating out loud.
 *
 * The mode line and the rule lines are the original's own strings
 * ("Combat", "Race", "2/3/4 Team C.T.F.", "%d Laps", "%d Captures",
 * "%d Kills", "%d Minutes", "No Limits!" — nitshell.dll's game-setup
 * block). The sentences that say what to DO on a race or a capture map are
 * AUTHORED ENGLISH and appear in no shipped file; they are listed in
 * coverage-ledger.md §5/§6 as invented.
 */

/*
 * Append a mission's own .npt objective lines (everything before the
 * "(failure)" marker, CRLF normalized, "(hidden)" markers kept — they are
 * the data). Returns the new write offset. Writes the unavailable notice
 * itself when the file cannot be read.
 */
static int briefing_append_npt(const char *mission_path, char *out, size_t n,
                               int w)
{
    char npt[14];
    uint8_t *buf = NULL;
    size_t sz = 0;
    if (mission_npt_name(mission_path, npt) == 0)
        buf = vfs_read_file(npt, &sz);
    if (!buf) {
        /* No objective file readable (corrupt/missing WRLD or npt not
         * in the zfs): name-only briefing (DECISION — every shipped
         * miss8 mission names an mXX/pXX/bXX .npt in its WRLD, so this
         * path should not trigger on staged retail data). */
        w += snprintf(out + w, n - (size_t)w,
                      "Objective data unavailable for this mission.");
        return w;
    }

    size_t pos = 0;
    while (pos < sz && (size_t)w < n - 2) {
        size_t eol = pos;
        while (eol < sz && buf[eol] != '\n') eol++;
        size_t len = eol - pos;
        if (len > 0 && buf[pos + len - 1] == '\r') len--;
        if (len >= 9 && memcmp(buf + pos, "(failure)", 9) == 0)
            break;                       /* fail texts: not briefing */
        if (len > 0) {
            if ((size_t)w + len + 1 > n - 1) len = n - 1 - (size_t)w - 1;
            memcpy(out + w, buf + pos, len);
            w += (int)len;
            out[w++] = '\n';
        }
        pos = eol + 1;
    }
    out[w] = '\0';
    vfs_free(buf);
    return w;
}

/* Select a nonempty line from the mission's NPT failure section. The
 * failAllObj operand is 1-based and every nonempty authored entry counts,
 * including literal placeholders such as "Not Used.". */
static int mission_failure_npt(const char *mission_path, int index,
                               char *out, size_t n)
{
    char npt[14];
    uint8_t *buf = NULL;
    size_t sz = 0;
    int in_failure = 0;
    int entry = 0;

    if (!out || n == 0)
        return 0;
    out[0] = '\0';
    if (!mission_path || index <= 0)
        return 0;
    if (mission_npt_name(mission_path, npt) == 0)
        buf = vfs_read_file(npt, &sz);
    if (!buf)
        return 0;

    for (size_t pos = 0; pos < sz; ) {
        size_t eol = pos;
        while (eol < sz && buf[eol] != '\n') eol++;
        size_t begin = pos;
        size_t end = eol;
        if (end > begin && buf[end - 1] == '\r') end--;
        while (begin < end && (buf[begin] == ' ' || buf[begin] == '\t'))
            begin++;
        while (end > begin && (buf[end - 1] == ' ' || buf[end - 1] == '\t'))
            end--;

        size_t len = end - begin;
        if (!in_failure) {
            if (len == 9 && memcmp(buf + begin, "(failure)", 9) == 0)
                in_failure = 1;
        } else if (len > 0 && ++entry == index) {
            if (len >= n) len = n - 1;
            memcpy(out, buf + begin, len);
            out[len] = '\0';
            vfs_free(buf);
            return 1;
        }
        pos = eol < sz ? eol + 1 : sz;
    }

    vfs_free(buf);
    return 0;
}

static void briefing_build(const char *mission_path, char *out, size_t n)
{
    const char *base = strrchr(mission_path, '/');
    base = base ? base + 1 : mission_path;
    int fam = mission_family_of_path(mission_path);

    if (fam == MISSION_FAMILY_MELEE || fam == MISSION_FAMILY_RACE ||
        fam == MISSION_FAMILY_CAPTURE) {
        char name[20];
        char limit[32];
        if (mission_display_name(mission_path, name) != 0)
            snprintf(name, sizeof name, "%s", base);
        /* "%d Minutes" / "No Limits!" — nitshell.dll, verbatim. */
        if (s_arena_minutes > 0)
            snprintf(limit, sizeof limit, "%d Minute%s", s_arena_minutes,
                     s_arena_minutes == 1 ? "" : "s");
        else
            snprintf(limit, sizeof limit, "No Limits!");

        if (fam == MISSION_FAMILY_MELEE) {
            /* "Combat" is nitshell.dll's own name for this mode, sitting
             * beside "Race" and "2/3/4 Team C.T.F." in the same list. The
             * objective prose below is the mission's own .npt. */
            int w = snprintf(out, n, "MISSION %s\n\nCombat.\n\n", name);
            w = briefing_append_npt(mission_path, out, n, w);
            snprintf(out + w, n - (size_t)w, "\n%d Kill%s\n%s\n",
                     s_arena_kills, s_arena_kills == 1 ? "" : "s", limit);
            return;
        }

        if (fam == MISSION_FAMILY_CAPTURE) {
            /* Team count from the extension: .CF2/.CF3/.CF4 name the
             * original's own "2/3/4 Team C.T.F." modes and all 17 files
             * agree with their extension (mission.c D-O8). */
            const char *dot = strrchr(mission_path, '.');
            int teams = (dot && dot[3]) ? dot[3] - '0' : 2;
            if (teams < 2 || teams > 4) teams = 2;
            /* Team 1 literally, not mission.c's CTF_PLAYER_TEAM: the
             * briefing is built BEFORE the mission loads, so there is no
             * live controller to ask. If D-O12 ever stops making the
             * player team 1, this line has to move with it. */
            snprintf(out, n,
                     "MISSION %s\n\n%d Team C.T.F.\n\nTeam %s\n\n"
                     "Take an enemy flag and carry it back to your own "
                     "base.\nOne flag at a time.\n\n"
                     "%d Capture%s\n%s\n",
                     name, teams, mission_team_name(1), s_arena_captures,
                     s_arena_captures == 1 ? "" : "s", limit);
            return;
        }

        snprintf(out, n,
                 "MISSION %s\n\nRace.\n\n"
                 "Drive through the numbered checkpoints in order.\n"
                 "Passing the last one completes a lap.\n\n"
                 "%d Lap%s\n%s\n",
                 name, s_arena_laps, s_arena_laps == 1 ? "" : "s", limit);
        return;
    }

    int w = snprintf(out, n, "MISSION %s\n\n", base);
    briefing_append_npt(mission_path, out, n, w);
}

/* Result derives from the FSM: complete/failed when it ended the
 * mission, aborted otherwise (ESC while still running). */
static void shell_enter_debrief(void)
{
    int ms = mission_state();
    const char *res = ms == MISSION_COMPLETE ? "complete"
        : ms == MISSION_FAILED ? "failed" : "aborted";
    const char *msg = mission_message();

    char esc[320];
    garage_json_escape(esc, sizeof esc, msg);

    /* Resolve authored scenario failure copy while both the mission and its
     * VFS-owned NPT are still loaded. Missing files, invalid indexes, restored
     * terminal state, and non-failAllObj failures deliberately yield "". */
    char fail_text[256];
    char fail_esc[512];
    fail_text[0] = '\0';
    if (ms == MISSION_FAILED && s_sel >= 0 && s_sel < s_cat_count)
        mission_failure_npt(s_cat_path[s_sel], mission_fail_text_index(),
                            fail_text, sizeof fail_text);
    garage_json_escape(fail_esc, sizeof fail_esc, fail_text);

    /* The objective counters as they stood at the end: a race debrief that
     * cannot say how many laps were completed is missing the only number
     * the mission was about. Read BEFORE teardown — the state is gone
     * after mission_unload(). */
    MissionObjectiveState ob;
    int have_ob = mission_objective_state(&ob);

    /* Fallback player-language evidence (H-UAT-015), read while the mission
     * is still loaded. playerDead is the player's own combat state; lost is
     * the authored label of a NON-HOSTILE mission entity that is dead at a
     * FAILED end. This remains a heuristic only for unavailable NPT text;
     * the structured failAllObj index above is the primary path. */
    int u = combat_user_ent();
    int player_dead = u >= 0 && !combat_alive(u);
    char lost[40];
    lost[0] = '\0';
    if (ms == MISSION_FAILED) {
        int nc = mission_contact_count();
        for (int e2 = 0; e2 < nc && !lost[0]; e2++) {
            MissionContact c;
            if (e2 == u || mission_contact(e2, &c) != 0)
                continue;
            if (c.relation >= 0 && !c.alive) {
                const char *l = mission_entity_label(e2);
                size_t li = 0;
                for (; l[li] && li + 1 < sizeof lost; li++) {
                    char ch = l[li];    /* keep the JSON below well-formed */
                    lost[li] = (ch >= 'a' && ch <= 'z') ||
                               (ch >= 'A' && ch <= 'Z') ||
                               (ch >= '0' && ch <= '9') ||
                               ch == '_' || ch == '-' ? ch : '_';
                }
                lost[li] = '\0';
            }
        }
    }

    snprintf(s_result, sizeof s_result,
             "{\"mission\":\"%s\",\"result\":\"%s\",\"state\":%d,"
             "\"ticks\":%u,\"time\":%.1f,\"dist\":%.1f,\"top\":%.1f,"
             "\"family\":%d,\"laps\":%d,\"lapTarget\":%d,\"gates\":%d,"
             "\"captures\":%d,\"captureTarget\":%d,\"teams\":%d,"
             "\"kills\":%d,\"killTarget\":%d,\"opponents\":%d,"
             "\"playerDead\":%d,\"lost\":\"%s\","
             "\"failText\":\"%s\",\"message\":\"%s\"}",
             s_sel >= 0 ? s_cat_path[s_sel] : "?", res, ms,
             s_drive_ticks, s_drive_ticks / 20.0, s_odom,
             s_top_speed * 2.237,                  /* top speed in mph */
             ob.family, have_ob ? ob.lap : 0, have_ob ? ob.lap_target : 0,
             have_ob ? ob.gates : 0, have_ob ? ob.captures : 0,
             have_ob ? ob.capture_target : 0, have_ob ? ob.teams : 0,
             have_ob ? ob.kills : 0, have_ob ? ob.kill_target : 0,
             have_ob ? ob.opponents : 0, player_dead, lost, fail_esc, esc);
    s_shell = SHELL_DEBRIEF;

    /* Campaign-end credits route: the run that just ended COMPLETED the
     * base Trip's final row (T17). Trip 0 is the retail shell's linear
     * "Trip" table (base_campaign_build); only its last row ends the
     * campaign. A Nitro terminal trip row can set this flag too — the
     * ROUTE stays closed there because web_shell_has_credits() also
     * gates on the base VFS profile, so Nitro debrief behavior is
     * untouched. */
    s_campaign_complete =
        ms == MISSION_COMPLETE &&
        s_camp_trip == 0 && s_camp_idx == s_trip[0].count - 1;
}

EMSCRIPTEN_KEEPALIVE
int web_shell_state(void) { return s_shell; }

EMSCRIPTEN_KEEPALIVE
int web_shell_mission_count(void)
{
    catalog_build();            /* lazy: page stages files post-web_init */
    return s_cat_count;
}

EMSCRIPTEN_KEEPALIVE
const char *web_shell_mission_name(int i)
{
    if (i < 0 || i >= s_cat_count) return "";
    return s_cat_path[i];
}

EMSCRIPTEN_KEEPALIVE
int web_shell_arena_index(void) { return s_arena_sel; }

/* menu -> briefing: record the pick and extract its objective text. */
EMSCRIPTEN_KEEPALIVE
int web_shell_pick(int i)
{
    if (s_shell != SHELL_MENU || i < 0 || i >= s_cat_count) return -1;
    s_sel = i;
    s_arena_sel = -1;
    briefing_build(s_cat_path[i], s_briefing, sizeof s_briefing);
    s_shell = SHELL_BRIEFING;
    return 0;
}

/* menu -> briefing, from the arena list. Resolves to the same catalog
 * index every other pick uses, so start/retry/debrief need no new path. */
EMSCRIPTEN_KEEPALIVE
int web_shell_pick_arena(int i)
{
    if (s_shell != SHELL_MENU) return -1;
    arena_ensure();
    if (i < 0 || i >= s_arena_count) return -1;

    catalog_build();
    int sel = -1;
    for (int c = 0; c < s_cat_count; c++)
        if (strcmp(s_cat_path[c], s_arena[i].path) == 0) { sel = c; break; }
    if (sel < 0) return -2;            /* listed, then unstaged */

    s_sel = sel;
    s_arena_sel = i;
    briefing_build(s_cat_path[sel], s_briefing, sizeof s_briefing);
    s_shell = SHELL_BRIEFING;
    return 0;
}

EMSCRIPTEN_KEEPALIVE
const char *web_shell_briefing(void) { return s_briefing; }

/* Mission-authored player car for the Scenario/Driver Entry Form. This is a
 * read-only preflight over the staged mission; it does not load the world. */
EMSCRIPTEN_KEEPALIVE
const char *web_shell_mission_vehicle(void)
{
    return s_sel >= 0 && s_sel < s_cat_count
         ? mission_probe_player_object(s_cat_path[s_sel]) : "";
}

/* Garage seam: the player's chosen .vcf base name, or "" for "whatever
 * the mission itself names" (web_drive_load resolves that). Survives a
 * return to the menu — it is a profile choice, not mission state. */
static char s_shell_vcf[32];

EMSCRIPTEN_KEEPALIVE
void web_shell_set_vehicle(const char *vcf)
{
    snprintf(s_shell_vcf, sizeof s_shell_vcf, "%s", vcf ? vcf : "");
}

EMSCRIPTEN_KEEPALIVE
const char *web_shell_vehicle(void) { return s_shell_vcf; }

/* briefing -> drive: the stock drive load path. The car comes from the
 * garage choice when there is one and from the mission's own player
 * record otherwise (web_drive_load). On load failure the shell stays in
 * BRIEFING. */
EMSCRIPTEN_KEEPALIVE
int web_shell_start(void)
{
    if (s_shell != SHELL_BRIEFING || s_sel < 0) return -1;
    int rc = web_drive_load(s_cat_path[s_sel], s_shell_vcf);
    if (rc != 0) return rc;
    s_esc_pending = 0;
    s_out_played = 0;                 /* one OUT* offer per run */
    shell_movie_clear();
    /* The mission's own INT* claim, played before the first sim tick.
     * Port placement (see the section header): the original shell's
     * sequencing graph is still an RE residual, but no clip is ever
     * invented — the queue only carries what the mission names and the
     * purchaser staged. */
    shell_movie_queue(mission_story_clip(MISSION_STORY_INTRO));
    s_shell = SHELL_DRIVE;
    return 0;
}

/*
 * Page-polled transition point (call once per frame): drive -> debrief
 * when the mission FSM leaves MISSION_RUNNING, or when ESC was latched
 * by web_key_event. Returns the (possibly new) shell state.
 */
EMSCRIPTEN_KEEPALIVE
int web_shell_update(void)
{
    if (s_shell == SHELL_DRIVE &&
        (mission_movie_pending() || s_shell_movie_pending)) {
        /* A terminal action may follow playMovie in the same FSM slice,
         * and a shell-queued story clip owns the screen the same way.
         * The debrief must wait until the queued movie ends or is
         * skipped. */
        s_esc_pending = 0;
        return s_shell;
    }
    if (s_shell == SHELL_DRIVE &&
        (mission_state() != MISSION_RUNNING || s_esc_pending)) {
        /* A completed mission's own OUT* clip plays BEFORE the debrief
         * (port placement, once per run). Failure and ESC abort skip it:
         * the corpus evidence attaches OUT* clips to completion beats,
         * and the abort/fail debrief behavior is unchanged. An unstaged
         * clip queues nothing and falls straight through. */
        if (!s_esc_pending && mission_state() == MISSION_COMPLETE &&
            !s_out_played) {
            s_out_played = 1;
            shell_movie_queue(mission_story_clip(MISSION_STORY_OUTRO));
            if (s_shell_movie_pending) {
                s_esc_pending = 0;
                return s_shell;
            }
        }
        shell_enter_debrief();   /* result derives from mission_state() */
    }
    s_esc_pending = 0;
    return s_shell;
}

EMSCRIPTEN_KEEPALIVE
const char *web_shell_result(void) { return s_result; }

/*
 * debrief -> menu (accepted from any state; idempotent). Full teardown
 * of the drive stack, mirrored against what web_mission_load/
 * web_drive_load assume on entry: mission_unload() frees the FSM runner
 * (attach mode leaves the caller's world — mission.c), then HUD art,
 * car, scene and terrain go in reverse load order.
 */
static void shell_teardown(void)
{
    webgpu_caches_reset();           /* before old mission assets unload */
    mission_unload();
    sound_mission_boundary();
    paper_unload();
    hud_unload();
    car_unload();
    scene_unload();
    terrain_unload();
    s_scene_loaded = 0;
    s_car_mesh = NULL;
    s_hud_frame_valid = 0;
    s_damage_flash = 0;
    s_damage_dir_valid = 0;
    s_damage_probe_attacker = -1;
    s_damage_probe_amount = 0;
    s_sel = -1;
    s_esc_pending = 0;
    shell_movie_clear();
    s_out_played = 0;
    s_campaign_complete = 0;
}

EMSCRIPTEN_KEEPALIVE
int web_shell_menu(void)
{
    shell_teardown();
    s_camp_trip = -1;
    s_camp_idx = -1;
    s_arena_sel = -1;
    s_shell = SHELL_MENU;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Campaign progression: pick by (trip, index), then next/retry.        */
/*                                                                     */
/* s_sel must stay a CATALOG index — shell_enter_debrief() reports      */
/* s_cat_path[s_sel] — so a campaign pick resolves its path through the */
/* catalog. catalog_build() is reached only from                        */
/* web_shell_mission_count() otherwise, and the consumer menu no longer */
/* calls that, so the build happens here.                               */
/* ------------------------------------------------------------------ */

EMSCRIPTEN_KEEPALIVE
int web_shell_pick_campaign(int t, int i)
{
    if (s_shell != SHELL_MENU) return -1;
    int k = camp_index(t, i);
    if (k < 0) return -1;

    catalog_build();
    int sel = -1;
    for (int c = 0; c < s_cat_count; c++)
        if (strcmp(s_cat_path[c], s_camp[k].path) == 0) { sel = c; break; }
    if (sel < 0) return -2;            /* named by the campaign, not staged */

    s_sel = sel;
    s_camp_trip = t;
    s_camp_idx = i;
    s_arena_sel = -1;
    briefing_build(s_cat_path[sel], s_briefing, sizeof s_briefing);
    s_shell = SHELL_BRIEFING;
    return 0;
}

/* Where the live pick sits in the campaign; -1 when the mission was
 * picked off the developer catalog instead. */
EMSCRIPTEN_KEEPALIVE
int web_shell_trip(void) { return s_camp_trip; }

EMSCRIPTEN_KEEPALIVE
int web_shell_trip_index(void) { return s_camp_idx; }

/* Is there a further staged mission in this trip? The debrief's
 * "next mission" affordance is shown from this. */
EMSCRIPTEN_KEEPALIVE
int web_shell_has_next(void)
{
    int k = camp_index(s_camp_trip, s_camp_idx + 1);
    return k >= 0 && s_camp[k].staged;
}

/*
 * debrief -> the next mission's briefing. Runs the SAME teardown
 * web_shell_menu() does first: web_drive_load()/web_mission_load() unload
 * scene/terrain/car but never mission_unload()/hud_unload(), so entering
 * the next mission straight from a debrief would leak the FSM runner and
 * the HUD art. Returns -1 at the end of a trip (or with nothing staged
 * next), leaving the debrief on screen.
 */
EMSCRIPTEN_KEEPALIVE
int web_shell_next(void)
{
    if (s_shell != SHELL_DEBRIEF) return -1;
    if (!web_shell_has_next()) return -1;
    int t = s_camp_trip, i = s_camp_idx + 1;
    shell_teardown();
    s_shell = SHELL_MENU;
    return web_shell_pick_campaign(t, i);
}

/* debrief -> the same mission's briefing again. */
EMSCRIPTEN_KEEPALIVE
int web_shell_retry(void)
{
    if (s_shell != SHELL_DEBRIEF || s_sel < 0) return -1;
    int t = s_camp_trip, i = s_camp_idx, a = s_arena_sel, sel = s_sel;
    shell_teardown();
    s_shell = SHELL_MENU;
    if (t >= 0 && i >= 0) return web_shell_pick_campaign(t, i);
    if (a >= 0)           return web_shell_pick_arena(a);
    return web_shell_pick(sel);
}

/* ------------------------------------------------------------------ */
/* Campaign-end credits (base profile only).                           */
/*                                                                     */
/* The retail base shell has a dedicated credits screen                */
/* (i76shell.dll ShellWindowProc case 0 -> FUN_1000e2e0;               */
/* docs/specs/re/phase-e-base-delta.md §4.1). We reach it when the     */
/* base Trip's final mission (T17) ends COMPLETE: the debrief still    */
/* runs first — result display and IndexedDB persistence are unchanged */
/* — and the debrief then offers this transition instead of a next     */
/* mission. The state is terminal: the only exit is web_shell_menu().  */
/* The credits MOVIE is the purchaser-supplied cred*.smk (both retail  */
/* products ship exactly CREDF01.SMK); the page finds and plays it     */
/* through its normal Smacker path — nothing is queued here, so a      */
/* missing clip can never wedge the state. Nitro keeps its plain       */
/* debrief: it has no terminal campaign mission, and the profile gate  */
/* below makes the route unreachable there regardless.                 */
/* ------------------------------------------------------------------ */

/* 1 while the debrief on screen is the base campaign's terminal one. */
EMSCRIPTEN_KEEPALIVE
int web_shell_has_credits(void)
{
    return s_shell == SHELL_DEBRIEF && s_campaign_complete &&
           vfs_profile() == VFS_PROFILE_BASE;
}

/* debrief -> credits. Tears the drive stack down like web_shell_menu()
 * does; campaign position stays recorded so the menu behind the credits
 * screen still shows the finished trip. */
EMSCRIPTEN_KEEPALIVE
int web_shell_credits(void)
{
    if (!web_shell_has_credits()) return -1;
    shell_teardown();
    s_shell = SHELL_CREDITS;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Framebuffer access + deterministic probes for the headless test     */
/* ------------------------------------------------------------------ */

EMSCRIPTEN_KEEPALIVE
uint8_t *web_fb(void) { return s_fb; }

EMSCRIPTEN_KEEPALIVE
uint8_t *web_palette(void) { return (uint8_t *)s_pal; }

/* M3.5: the mission's level palette (768-byte RGB), or NULL when the
 * mission named none / none was found — the page then keeps the mesh
 * palette. Valid after web_mission_load()/web_drive_load(). */
EMSCRIPTEN_KEEPALIVE
const uint8_t *web_level_palette(void) { return hud_palette(); }

/* Paper surfaces (map/notepad): "paper tag=p01 map=1 npd=1 active=none" */
EMSCRIPTEN_KEEPALIVE
const char *web_paper_stats(void)
{
    static char buf[96];
    paper_stats(buf, sizeof buf);
    return buf;
}

/* Active paper surface: 0=none, 1=map, 2=notepad (PaperSurface). */
EMSCRIPTEN_KEEPALIVE
int web_paper_active(void) { return (int)paper_active(); }

/* In-frame port text overlay (hud_set_text_overlay). The consumer page
 * turns it off and shows objectives/weapon state in the DOM instead. */
EMSCRIPTEN_KEEPALIVE
void web_hud_set_text_overlay(int on)
{
    hud_set_text_overlay(on);
}

/* Escape paper panel (PaperEscape enum). 0 clears. */
EMSCRIPTEN_KEEPALIVE
void web_paper_set_escape(int which)
{
    paper_set_escape((PaperEscape)which);
}

EMSCRIPTEN_KEEPALIVE
int web_paper_escape(void) { return (int)paper_escape(); }

EMSCRIPTEN_KEEPALIVE
int web_paper_title_active(void) { return paper_title_active(); }

EMSCRIPTEN_KEEPALIVE
void web_paper_dismiss_title(void) { paper_dismiss_title(); }

/* M3.5 HUD gate: fb pixels changed by the HUD overlay in the last
 * web_drive_render() (diff against the pre-HUD composite). 0 before the
 * first drive render and whenever the HUD drew nothing (mission mode
 * never renders the HUD). */
EMSCRIPTEN_KEEPALIVE
int web_hud_pixels(void)
{
    if (!s_hud_frame_valid) return 0;
    int n = 0;
    for (int i = 0; i < MESHVIEW_FB_W * MESHVIEW_FB_H; i++)
        if (s_fb[i] != s_pre_hud[i]) n++;
    return n;
}

/* Authored cockpit/chase HUD layout only. Unlike web_gpu_hud_layer(), this
 * count is captured before valid lower-screen target/combat/pilot overlays
 * enter the shared scratch, so a nonzero result specifically detects a pasted
 * dashboard or an overflowing weapon/condition row. */
EMSCRIPTEN_KEEPALIVE
int web_authored_hud_lower_pixels(void)
{
    return s_hud_frame_valid ? s_authored_hud_lower_pixels : 0;
}

EMSCRIPTEN_KEEPALIVE
int web_nonbg_pixels(void)
{
    int n = 0;
    for (int i = 0; i < MESHVIEW_FB_W * MESHVIEW_FB_H; i++)
        if (s_fb[i] != 0) n++;   /* palette index 0 = background */
    return n;
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_fb_fnv1a(void)
{
    uint32_t h = 2166136261u;
    for (int i = 0; i < MESHVIEW_FB_W * MESHVIEW_FB_H; i++) {
        h ^= s_fb[i];
        h *= 16777619u;
    }
    return h;
}

/* ================================================================== */
/* Audio (M5) — WebAudio SFX bridge (AudioPlay). Appended section;    */
/* self-contained — its #include lives here so the append is atomic.  */
/*                                                                    */
/* The page (web/audio.js) owns actual output: it pulls PCM out of    */
/* the wasm heap after web_sound_load() and diffs web_sound_tick()'s  */
/* JSON against its live WebAudio sources. sound.c tracks WHAT is     */
/* playing (8 one-shot slots + 1 engine loop).                        */
/* ================================================================== */

EMSCRIPTEN_KEEPALIVE
int web_sound_load(const char *name)
{
    int len = sound_load(name);          /* 0 = missing/invalid */
    if (len > 0)
        snprintf(s_snd_pcm_last, sizeof s_snd_pcm_last, "%s", name);
    return len;
}

/* PCM bytes of the named sound — or of the last successful
 * web_sound_load() when called with no argument (the page's
 * load-then-read sequence is synchronous). Pointer is into the wasm
 * heap; the page copies len bytes through HEAPU8. */
EMSCRIPTEN_KEEPALIVE
const uint8_t *web_sound_pcm_ptr(const char *name)
{
    if (!name || !*name) name = s_snd_pcm_last;
    return sound_pcm(name, NULL);
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_sound_rate(const char *name)
{
    if (!name || !*name) name = s_snd_pcm_last;
    return sound_rate(name);
}

/* Start a one-shot; returns its id (>0), 0 when the wav is missing. */
EMSCRIPTEN_KEEPALIVE
int web_sound_play(const char *name)
{
    return (int)sound_play(name);
}

EMSCRIPTEN_KEEPALIVE
void web_sound_stop(const char *name)
{
    sound_stop(name);
}

/* Session-wide diagnostic used by the registered sequential-mission gate. */
EMSCRIPTEN_KEEPALIVE
uint32_t web_sound_cache_full_failures(void)
{
    return sound_cache_full_failures();
}

/*
 * Engine loop update. The loaded VDF vehicle-size class selects the original
 * engsnd.dat ENG NUM. With no loaded car, row 0 remains the diagnostic default
 * used by the standalone WebAudio self-test.
 */
EMSCRIPTEN_KEEPALIVE
int web_sound_engine(double rpm, double load)
{
    int wanted = car_engine_sound_number();
    if (wanted < 0) wanted = 0;
    if (sound_engsnd_load() <= 0) return -1;

    const char *wav = sound_engsnd_wav(wanted);
    if (!wav) return -1;
    if (strcmp(sound_engine_wav(), wav) != 0 && !sound_engine_set(wav))
        return -1;

    sound_engine_update(rpm, load);
    return 0;
}

/* Stop the engine loop on mission exit (H-UAT-038). */
EMSCRIPTEN_KEEPALIVE
void web_sound_engine_stop(void)
{
    sound_engine_stop();
}

/*
 * Playing-state snapshot for the page: expires finished one-shots
 * against the browser clock, then serializes
 *   {"ones":[[id,"name"],...],"eng":["wav",pitch,gain,active]}
 * (eng is [] until an engine wav is selected).
 */
EMSCRIPTEN_KEEPALIVE
const char *web_sound_tick(void)
{
    static char s_snd_json[512];
    sound_set_time((uint32_t)emscripten_get_now());
    sound_tick_json(s_snd_json, sizeof s_snd_json);
    return s_snd_json;
}

/* ------------------------------------------------------------------ */
/* M6 Tier-2 GPU scene path (docs/specs/m6/webgpu-plan.md "Tier 2 —   */
/* hardware scene path"): geometry + texture exports consumed by       */
/* web/gpu_scene.mjs. Purely additive — nothing above calls in here,   */
/* and the software renderers stay the parity reference. Returned      */
/* pointers are static storage (valid until the same export is called  */
/* again) or engine-owned (GeoMesh — valid until scene_unload/         */
/* car_unload); the page copies everything out through HEAP views.     */
/* ------------------------------------------------------------------ */

EMSCRIPTEN_KEEPALIVE
uint8_t *web_fb_pre_hud(void)
{
    /* Pre-HUD composite of the last web_drive_render() (s_pre_hud is
     * snapshotted there before the HUD overlay draws) — the GPU path
     * renders the world only, so the parity gate compares against this
     * buffer. Meaningful only after the first drive render. */
    return s_pre_hud;
}

/*
 * Geometry coverage matching web_fb_pre_hud(): nonzero reciprocal depth is
 * terrain/scene, zero is sky. The indexed colour buffer is no longer a
 * coverage mask because the software sky intentionally paints every pixel.
 * Renderer-owned; consume before the next world render.
 */
EMSCRIPTEN_KEEPALIVE
const uint32_t *web_gpu_world_depth(void)
{
    return worldrender_depth();
}

/*
 * web_gpu_hud_layer()
 *   The HUD/cockpit overlay of the last web_drive_render() ON ITS OWN:
 *   640x480 palette indices with 0 = transparent, exactly the layer the
 *   software path composites nonzero-over the world frame. The GPU path
 *   draws it as a palette-looked-up overlay quad so drive mode keeps its
 *   cockpit, gauges and text (index.html previously hid the software
 *   canvas outright under ?renderer=gpu, dropping the whole overlay).
 *   s_scene_fb holds it: web_drive_render renders the HUD into that
 *   scratch last, and compositing only reads from it. Returns NULL until
 *   a drive frame has been rendered (s_hud_frame_valid), so a caller can
 *   never present a stale or scene-clobbered scratch.
 */
EMSCRIPTEN_KEEPALIVE
uint8_t *web_gpu_hud_layer(void)
{
    return s_hud_frame_valid ? s_scene_fb : NULL;
}

/*
 * Complete post-world software 2-D composite for WebGPU presentation.
 * The mask is authoritative coverage (0 transparent, 0xff opaque), so an
 * authored palette-index-0 paper pixel remains opaque. Both pointers describe
 * the same last web_drive_render() snapshot and are NULL until one exists.
 */
EMSCRIPTEN_KEEPALIVE
uint8_t *web_gpu_overlay_layer(void)
{
    return s_hud_frame_valid ? s_gpu_overlay : NULL;
}

EMSCRIPTEN_KEEPALIVE
uint8_t *web_gpu_overlay_mask(void)
{
    return s_hud_frame_valid ? s_gpu_overlay_mask : NULL;
}

/* Active drive camera for the GPU path: the same mission-first selection
 * web_drive_render() uses. out = eye[3],target[3]. */
EMSCRIPTEN_KEEPALIVE
double *web_gpu_camera(void)
{
    static CameraView camera;
    (void)drive_camera_render(&camera);
    return camera.eye;  /* contiguous eye,right,up,forward: 12 doubles */
}

/* Player-car world transform (basis columns r/u/f + pos — scene.c D4
 * convention, same layout as scene_obj_part_xform). The software drive
 * render queues every decoded part at basis ∘ car_part_frame through
 * scene_dyn_add; this export is part 0's basis for the Tier-2 parity gate. */
EMSCRIPTEN_KEEPALIVE
double *web_gpu_car_xform(void)
{
    static double out[12];
    car_basis(out);
    return out;
}

EMSCRIPTEN_KEEPALIVE
GeoMesh *web_gpu_car_mesh(void) { return s_car_mesh; }

/*
 * Flatten exterior parts (VGEO body + WDF wheels, car.c D23) and every
 * mounted weapon's presentational GGEO parts into the one list consumed by
 * the GPU renderer.  The software path builds the same groups in
 * web_drive_render().
 */
static int car_render_part_count(void)
{
    int n = car_part_count();
    for (int w = 0; w < car_weapon_count(); w++)
        n += car_weapon_part_count(w);
    return n;
}

static const char *car_render_part(int i, double frame[12])
{
    if (i < 0) return NULL;
    int nbody = car_part_count();
    if (i < nbody) {
        if (frame && car_part_frame(i, frame) != 0) return NULL;
        return car_part_name(i);
    }
    i -= nbody;
    for (int w = 0; w < car_weapon_count(); w++) {
        int nparts = car_weapon_part_count(w);
        if (i < nparts) {
            if (frame && car_weapon_part_live_frame(w, i, frame) != 0)
                return NULL;
            return car_weapon_part_name(w, i);
        }
        i -= nparts;
    }
    return NULL;
}

/*
 * Whole-vehicle export (M6 Tier-2 plus M7 mounted weapons).
 * web_gpu_car_mesh/xform above expose only VGEO part 0 — which for a real
 * .vcf is one small sub-part (measured: 1.8 x 0.7 x 2.1 m, 22 tris), not a
 * car. Both render paths draw every body/wheel part and every mounted weapon
 * GGEO part instead: mesh = scene_part_mesh(car_render_part(i)), placed at
 * car_basis AFTER the part's composed car-model frame.  The legacy
 * single-part exports stay for the Tier-2 parity gate, which predates the
 * whole-vehicle software queue.
 */
EMSCRIPTEN_KEEPALIVE
int web_gpu_car_part_count(void)
{
    return car_is_loaded() ? car_render_part_count() : 0;
}

EMSCRIPTEN_KEEPALIVE
GeoMesh *web_gpu_car_part_mesh(int i)
{
    const char *name = car_is_loaded() ? car_render_part(i, NULL) : NULL;
    return name ? scene_part_mesh(name) : NULL;
}

EMSCRIPTEN_KEEPALIVE
double *web_gpu_car_part_xform(int i)
{
    static double out[12];
    double basis[12], frame[12];
    if (!car_is_loaded() || !car_render_part(i, frame)) return NULL;
    car_basis(basis);
    basis_compose_frame(out, basis, frame);
    return out;
}

/*
 * First-person interior parts (VDF VGEO set 16, car.c D22) for the WebGPU
 * cockpit — the same basis ∘ part-frame composition as the software
 * cockpit queue in web_drive_render(). The GPU drawlist hides these draws
 * unless web_drive_view() reports the cockpit; that effective-view accessor
 * reports chase while a mission camera owns the frame, so authored cameras
 * always receive the exterior and never film this shell from outside.
 */
EMSCRIPTEN_KEEPALIVE
int web_gpu_interior_part_count(void)
{
    return car_is_loaded() ? car_interior_part_count() : 0;
}

EMSCRIPTEN_KEEPALIVE
GeoMesh *web_gpu_interior_part_mesh(int i)
{
    const char *name = (car_is_loaded() && i >= 0 &&
                        i < car_interior_part_count())
                     ? car_interior_part_name(i) : NULL;
    return name ? scene_part_mesh(name) : NULL;
}

EMSCRIPTEN_KEEPALIVE
double *web_gpu_interior_part_xform(int i)
{
    static double out[12];
    double basis[12], frame[12];
    if (!car_is_loaded() || i < 0 || i >= car_interior_part_count() ||
        cockpit_part_frame(i, frame) != 0)
        return NULL;
    car_basis(basis);
    basis_compose_frame(out, basis, frame);
    return out;
}

/*
 * Project a world point through the resolved drive camera. Focal length uses
 * that view's horizontal half-FOV tangent, matching both software render paths
 * and gpu_scene.mjs. Returns static {sx, sy, zs}; zs < 5 is clipped.
 */
EMSCRIPTEN_KEEPALIVE
double *web_gpu_project(double wx, double wy, double wz)
{
    static double out[3];
    CameraView camera;
    (void)drive_camera_render(&camera);
    double dx = wx - camera.eye[0];
    double dy = wy - camera.eye[1];
    double dz = wz - camera.eye[2];
    double xs = dx * camera.right[0] + dy * camera.right[1] +
                dz * camera.right[2];
    double ys = dx * camera.up[0] + dy * camera.up[1] +
                dz * camera.up[2];
    double zs = dx * camera.forward[0] + dy * camera.forward[1] +
                dz * camera.forward[2];
    double f = (MESHVIEW_FB_W * 0.5) /
               camera_view_fov_tan_half(&camera);
    out[0] = out[1] = 0.0;
    if (zs > 1e-9) {
        out[0] = MESHVIEW_FB_W * 0.5 + f * xs / zs;
        out[1] = MESHVIEW_FB_H * 0.5 - f * ys / zs;
    }
    out[2] = zs;
    return out;
}

/*
 * Heightfield grid over the terrain's used extent:
 *   float[0..4] = cols, rows, wx0, wz0, step_m
 *   float[5..]  = heights (m), row-major (row = +z north), at
 *                 (wx0 + i*step, wz0 + j*step)
 * DECISION (bbox via stats): the used extent has no dedicated accessor;
 * it is parsed out of terrain_stats ("bbox=[x0..x1]x[z0..z1]" — exact
 * terrain.c format string). This export is the format's only consumer
 * beyond humans, so drift fails loudly (sscanf count != 4 -> NULL).
 * Heights come from terrain_height_at (bilinear, exact at 5 m sample
 * points). DECISION (stride): NATIVE 5 m — one mesh vertex per .ter
 * sample (terrain.h TERRAIN_SAMPLE_STEP_M), so the hardware path shows
 * the heightfield at the resolution the data actually has. It was
 * previously decimated 4x to a 20 m stride, which visibly stair-stepped
 * ridges and canyon walls. The stride still doubles while the grid
 * exceeds GPU_TERRAIN_MAX_VERTS, so a large used extent degrades
 * gracefully instead of allocating an unbounded buffer (N01's 2x1-patch
 * extent is 257x129 = 33k verts at native stride).
 * Returns NULL when no terrain is loaded / the bbox parse fails.
 */
#define GPU_TERRAIN_MAX_VERTS (700L * 700L)

/*
 * Legacy uniform heightfield over the used extent (kept for probes that
 * still read cols/rows/step). Prefer web_gpu_terrain_lod_* — same LOD rings
 * as software fill_band, which is the geometric parity path.
 */
EMSCRIPTEN_KEEPALIVE
float *web_gpu_terrain_data(void)
{
    static float *s_grid;
    if (s_grid) { free(s_grid); s_grid = NULL; }
    if (!terrain_is_loaded()) return NULL;
    char buf[512];
    if (terrain_stats(buf, sizeof buf) < 0) return NULL;
    int cx0, cx1, cz0, cz1;
    const char *b = strstr(buf, "bbox=[");
    if (!b || sscanf(b, "bbox=[%d..%d]x[%d..%d]",
                     &cx0, &cx1, &cz0, &cz1) != 4)
        return NULL;
    if (cx1 < cx0 || cz1 < cz0) return NULL;
    long npatch_x = (long)(cx1 - cx0 + 1), npatch_z = (long)(cz1 - cz0 + 1);
    int kstep = 1;
    long cols, rows;
    for (;;) {
        cols = npatch_x * TERRAIN_PATCH_DIM / kstep + 1;
        rows = npatch_z * TERRAIN_PATCH_DIM / kstep + 1;
        if (cols * rows <= GPU_TERRAIN_MAX_VERTS ||
            kstep >= TERRAIN_PATCH_DIM) break;
        kstep <<= 1;
    }
    double step = kstep * TERRAIN_SAMPLE_STEP_M;
    double wx0 = cx0 * TERRAIN_PATCH_SIZE_M;
    double wz0 = cz0 * TERRAIN_PATCH_SIZE_M;
    size_t n = 5 + (size_t)cols * (size_t)rows;
    s_grid = malloc(n * sizeof *s_grid);
    if (!s_grid) return NULL;
    s_grid[0] = (float)cols;  s_grid[1] = (float)rows;
    s_grid[2] = (float)wx0;   s_grid[3] = (float)wz0;
    s_grid[4] = (float)step;
    float *hp = s_grid + 5;
    for (long j = 0; j < rows; j++)
        for (long i = 0; i < cols; i++)
            hp[j * cols + i] =
                (float)terrain_height_at(wx0 + i * step, wz0 + j * step);
    return s_grid;
}

/*
 * Shared-stream terrain LOD mesh (software fill_band twin).
 * Layout after call:
 *   web_gpu_terrain_lod_n()      — vertex count (nverts; each 8 floats)
 *   web_gpu_terrain_lod_ptr()    — float* pos3 nrm3 uv2 interleaved
 *   web_gpu_terrain_lod_bands()  — int[3] near/mid/far accepted quads
 * Rebuilds from the current drive camera each call (caller free is owned
 * by this module — the next call frees the previous buffer, and
 * webgpu_caches_reset() frees it at mission replacement, so the pointer
 * from web_gpu_terrain_lod_ptr() is valid only until either).
 */
static float *s_lod_verts;
static int    s_lod_nverts;
static int    s_lod_bands[3];

EMSCRIPTEN_KEEPALIVE
int web_gpu_terrain_lod_build(void)
{
    if (s_lod_verts) { free(s_lod_verts); s_lod_verts = NULL; }
    s_lod_nverts = 0;
    s_lod_bands[0] = s_lod_bands[1] = s_lod_bands[2] = 0;
    CameraView cam;
    /* Scripted cameras are valid here; drive_camera_render preserves cuts. */
    (void)drive_camera_render(&cam);
    if (!camera_view_valid(&cam)) return -1;
    float *v = NULL;
    int nv = 0;
    if (terrain_lod_mesh_export(&cam, 3000.0, &v, &nv, s_lod_bands) != 0)
        return -1;
    s_lod_verts = v;
    s_lod_nverts = nv;
    return 0;
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_terrain_lod_n(void) { return s_lod_nverts; }

EMSCRIPTEN_KEEPALIVE
float *web_gpu_terrain_lod_ptr(void) { return s_lod_verts; }

EMSCRIPTEN_KEEPALIVE
int *web_gpu_terrain_lod_bands(void) { return s_lod_bands; }

EMSCRIPTEN_KEEPALIVE
float *web_gpu_light_params(void)
{
    /* sun xyz + ambient — same constants as software terrain/scene. */
    static float out[4];
    float sun[3], amb = 0.35f;
    terrain_light_params(sun, &amb);
    out[0] = sun[0]; out[1] = sun[1]; out[2] = sun[2]; out[3] = amb;
    return out;
}

/*
 * Visibility membership for parity probes: counts the draws software and
 * GPU should share after collectDraws/refreshDraws.
 * out[0]=scene visible, [1]=scene hidden, [2]=car parts, [3]=interior parts,
 * [4]=interior hidden (cockpit-only), [5]=lod quads near+mid+far.
 */
EMSCRIPTEN_KEEPALIVE
int *web_gpu_visibility_counts(void)
{
    static int out[6];
    int sv = 0, sh = 0;
    int nObj = scene_obj_count();
    for (int o = 0; o < nObj; o++) {
        if (scene_obj_consumed(o))
            continue;
        int np = scene_obj_part_count(o);
        int hid = scene_obj_hidden(o);
        for (int p = 0; p < np; p++) {
            if (hid) sh++; else sv++;
        }
    }
    out[0] = sv;
    out[1] = sh;
    out[2] = car_render_part_count();
    out[3] = car_interior_part_count();
    /* Use the effective view: an authored camera overrides the saved manual
     * cockpit selection without destroying it (H-UAT-055). */
    out[4] = (web_drive_view() == DRIVE_VIEW_COCKPIT) ? 0 : out[3];
    out[5] = s_lod_bands[0] + s_lod_bands[1] + s_lod_bands[2];
    return out;
}

/* ---- Road ribbons (fill_roads twin) --------------------------------- */
static float *s_road_verts;
static int    s_road_nverts;
static int    s_road_by_type[3];
static uint8_t *s_road_rgba[3];
static int    s_road_tex_w[3], s_road_tex_h[3];

static void roads_tex_invalidate(void)
{
    for (int i = 0; i < 3; i++) {
        free(s_road_rgba[i]);
        s_road_rgba[i] = NULL;
        s_road_tex_w[i] = s_road_tex_h[i] = 0;
    }
}

static void roads_tex_ensure(void)
{
    const uint8_t *pal = hud_palette();
    if (!pal) pal = (const uint8_t *)s_pal;
    for (int t = 0; t < 3; t++) {
        if (s_road_rgba[t]) continue;
        const uint8_t *texels = NULL;
        int w = 0, h = 0;
        if (!terrain_road_tex(t, &texels, &w, &h) || !texels || w <= 0 || h <= 0)
            continue;
        uint8_t *rgba = malloc((size_t)w * h * 4);
        if (!rgba) continue;
        for (int i = 0; i < w * h; i++) {
            rgba[i * 4 + 0] = pal[texels[i] * 3 + 0];
            rgba[i * 4 + 1] = pal[texels[i] * 3 + 1];
            rgba[i * 4 + 2] = pal[texels[i] * 3 + 2];
            rgba[i * 4 + 3] = 255;
        }
        s_road_rgba[t] = rgba;
        s_road_tex_w[t] = w;
        s_road_tex_h[t] = h;
    }
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_roads_build(void)
{
    if (s_road_verts) { free(s_road_verts); s_road_verts = NULL; }
    s_road_nverts = 0;
    s_road_by_type[0] = s_road_by_type[1] = s_road_by_type[2] = 0;
    CameraView cam;
    (void)drive_camera_render(&cam);
    if (!camera_view_valid(&cam)) return -1;
    float *v = NULL;
    int nv = 0;
    if (terrain_roads_mesh_export(cam.eye, 3000.0, &v, &nv, s_road_by_type) != 0)
        return -1;
    s_road_verts = v;
    s_road_nverts = nv;
    roads_tex_ensure();
    return 0;
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_roads_n(void) { return s_road_nverts; }

EMSCRIPTEN_KEEPALIVE
float *web_gpu_roads_ptr(void) { return s_road_verts; }

EMSCRIPTEN_KEEPALIVE
int *web_gpu_roads_by_type(void) { return s_road_by_type; }

EMSCRIPTEN_KEEPALIVE
int web_gpu_road_tex_w(int type)
{
    if (type < 0 || type > 2) return 0;
    roads_tex_ensure();
    return s_road_tex_w[type];
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_road_tex_h(int type)
{
    if (type < 0 || type > 2) return 0;
    roads_tex_ensure();
    return s_road_tex_h[type];
}

EMSCRIPTEN_KEEPALIVE
const uint8_t *web_gpu_road_tex_rgba(int type)
{
    if (type < 0 || type > 2) return NULL;
    roads_tex_ensure();
    return s_road_rgba[type];
}

/*
 * GPU drawlist bridge — the sole scene-membership input to gpu_scene.mjs.
 * Each record: kind (0 scene, 1 car, 2 interior), a (obj or part idx),
 * b (part for scene), hidden, model 12 doubles (r/u/f/pos), mesh ptr as
 * double bits of uintptr (read via web_gpu_drawlist_mesh(i)).
 * The record buffer is retained across builds and doubled on demand —
 * never silently truncated; web_gpu_drawlist_build fails closed instead.
 * webgpu_caches_reset() releases it at mission replacement.
 */
typedef struct {
    int kind, a, b, hidden;
    double model[12];
    GeoMesh *mesh;
} GpuDrawRec;
static GpuDrawRec *s_draws;
static int s_draws_cap;
static int s_ndraws;

static int drawlist_push(int kind, int a, int b, int hidden,
                         const double model[12], GeoMesh *mesh)
{
    if (!mesh) return 0;
    if (s_ndraws >= s_draws_cap) {
        int nc = s_draws_cap ? s_draws_cap * 2 : 256;
        GpuDrawRec *nd = realloc(s_draws, (size_t)nc * sizeof *nd);
        if (!nd) return -1;
        s_draws = nd;
        s_draws_cap = nc;
    }
    GpuDrawRec *d = &s_draws[s_ndraws++];
    d->kind = kind; d->a = a; d->b = b; d->hidden = hidden;
    d->mesh = mesh;
    if (model) memcpy(d->model, model, sizeof d->model);
    else memset(d->model, 0, sizeof d->model);
    return 0;
}

/* Forward decls — drawlist reuses the Tier-2 part helpers defined below. */
double *web_gpu_scene_part_xform(int obj, int part);
GeoMesh *web_gpu_scene_part_mesh(int obj, int part);
int web_gpu_car_part_count(void);
double *web_gpu_car_part_xform(int i);
GeoMesh *web_gpu_car_part_mesh(int i);
int web_gpu_interior_part_count(void);
double *web_gpu_interior_part_xform(int i);
GeoMesh *web_gpu_interior_part_mesh(int i);

EMSCRIPTEN_KEEPALIVE
int web_gpu_drawlist_build(void)
{
    s_ndraws = 0;
    int nObj = scene_obj_count();
    for (int o = 0; o < nObj; o++) {
        if (scene_obj_consumed(o))
            continue;
        int hid = scene_obj_hidden(o);
        int np = scene_obj_part_count(o);
        for (int p = 0; p < np; p++) {
            double *xp = web_gpu_scene_part_xform(o, p);
            GeoMesh *m = web_gpu_scene_part_mesh(o, p);
            if (!xp || !m) continue;
            if (drawlist_push(0, o, p, hid, xp, m) != 0) goto oom;
        }
    }
    /* The saved manual view remains cockpit across pushCam/popCam. Visibility
     * must follow the effective camera owner instead: authored scene cameras
     * film the full exterior, then popCam re-applies cockpit culling. This is
     * the GPU twin of web_drive_render's `cutscene` branch (H-UAT-055). */
    int cockpit_presenting = web_drive_view() == DRIVE_VIEW_COCKPIT;
    int ncar = web_gpu_car_part_count();
    for (int i = 0; i < ncar; i++) {
        double *xp = web_gpu_car_part_xform(i);
        GeoMesh *m = web_gpu_car_part_mesh(i);
        if (!xp || !m) continue;
        if (drawlist_push(1, i, 0, cockpit_presenting, xp, m) != 0)
            goto oom;
    }
    int nint = web_gpu_interior_part_count();
    for (int i = 0; i < nint; i++) {
        /* Match software cockpit filter: skip 320-mode *3 when *6 sibling
         * exists; skip RTC1 when RTC6 is present (web_drive_render). */
        const char *iname = car_interior_part_name(
            s_interior_parts[i].source);
        size_t ilen = iname ? strlen(iname) : 0;
        const char *irole = ilen >= 4 ? iname + ilen - 4 : "";
        if (ilen >= 4 && irole[3] == '3') {
            char want6[16];
            size_t copy = ilen < sizeof want6 - 1 ? ilen : sizeof want6 - 1;
            memcpy(want6, iname, copy);
            want6[copy] = '\0';
            if (copy >= 1) want6[copy - 1] = '6';
            int skip3 = 0;
            for (int j = 0; j < nint; j++) {
                const char *n2 = car_interior_part_name(
                    s_interior_parts[j].source);
                if (n2 && strcasecmp(n2, want6) == 0) { skip3 = 1; break; }
            }
            if (skip3) continue;
        }
        if (strcasecmp(irole, "RTC1") == 0) {
            int has6 = 0;
            for (int j = 0; j < nint; j++) {
                const char *n2 = car_interior_part_name(
                    s_interior_parts[j].source);
                size_t l2 = n2 ? strlen(n2) : 0;
                if (l2 >= 4 && strcasecmp(n2 + l2 - 4, "RTC6") == 0) {
                    has6 = 1; break;
                }
            }
            if (has6) continue;
        }
        double *xp = web_gpu_interior_part_xform(i);
        GeoMesh *m = web_gpu_interior_part_mesh(i);
        if (!xp || !m) continue;
        if (drawlist_push(2, i, 0, !cockpit_presenting, xp, m) != 0)
            goto oom;
    }
    return s_ndraws;

oom:
    /* Fail closed: count zero, no partial draw list. Retained capacity is
     * kept — the next build may still succeed. */
    s_ndraws = 0;
    return -1;
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_drawlist_count(void) { return s_ndraws; }

EMSCRIPTEN_KEEPALIVE
int web_gpu_drawlist_kind(int i)
{
    return (i >= 0 && i < s_ndraws) ? s_draws[i].kind : -1;
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_drawlist_hidden(int i)
{
    return (i >= 0 && i < s_ndraws) ? s_draws[i].hidden : 1;
}

EMSCRIPTEN_KEEPALIVE
double *web_gpu_drawlist_model(int i)
{
    return (i >= 0 && i < s_ndraws) ? s_draws[i].model : NULL;
}

EMSCRIPTEN_KEEPALIVE
GeoMesh *web_gpu_drawlist_mesh(int i)
{
    return (i >= 0 && i < s_ndraws) ? s_draws[i].mesh : NULL;
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_drawlist_a(int i)
{
    return (i >= 0 && i < s_ndraws) ? s_draws[i].a : -1;
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_drawlist_b(int i)
{
    return (i >= 0 && i < s_ndraws) ? s_draws[i].b : -1;
}

/* H-UAT-060 shared-renderer proof seam. After web_drive_load, enumerate every
 * VCF/VDF/VGEO-backed scenery object within 40 m of the authored player spawn,
 * classify it against live mission ownership, and inspect the exact effective
 * membership consumed by software plus the C drawlist consumed by WebGPU.
 * Player placeholders remain enumerable identity records: software marks them
 * effectively hidden and the permanent WebGPU drawlist omits them entirely.
 * AI scene cars are their live render representation and remain visible. */
EMSCRIPTEN_KEEPALIVE
const char *web_spawn_vehicle_audit(void)
{
    static char out[4096];
    const double radius = 40.0;
    double spawn[3];
    if (mission_spawn(spawn) != 0) {
        double yaw, pitch, roll;
        car_pose(&spawn[0], &spawn[1], &spawn[2], &yaw, &pitch, &roll);
    }

    int draw_count = web_gpu_drawlist_build();
    int near = 0, player_placeholders = 0;
    int sw_phantoms = 0, gpu_phantoms = 0, gpu_live_car = 0;
    for (int i = 0; i < s_ndraws; i++)
        if (s_draws[i].kind == 1 && !s_draws[i].hidden)
            gpu_live_car++;

    size_t off = 0;
    off += (size_t)snprintf(out + off, sizeof out - off,
                           "{\"radius\":40,\"near\":");
    /* Reserve the scalar prefix until the enumeration has produced counts;
     * entries are assembled separately so the JSON never needs patching. */
    char entries[3072];
    size_t eo = 0;
    entries[eo++] = '[';
    entries[eo] = '\0';
    for (int obj = 0; obj < scene_obj_count(); obj++) {
        if (!scene_obj_is_vehicle_mesh(obj))
            continue;
        double pos[3];
        if (scene_obj_pos(obj, pos) != 0 ||
            hypot(pos[0] - spawn[0], pos[2] - spawn[2]) > radius)
            continue;
        GeoMesh *mesh = NULL;
        for (int p = 0; p < scene_obj_part_count(obj) && !mesh; p++)
            mesh = scene_obj_part_mesh(obj, p);
        int player = mission_scene_object_is_player_vehicle(obj);
        int live = mission_scene_object_is_vehicle(obj);
        int sw_visible = !scene_obj_hidden(obj);
        int gpu_visible = 0, gpu_parts = 0;
        if (draw_count >= 0)
            for (int i = 0; i < s_ndraws; i++)
                if (s_draws[i].kind == 0 && s_draws[i].a == obj) {
                    gpu_parts++;
                    if (!s_draws[i].hidden)
                        gpu_visible++;
                }
        near++;
        if (player) {
            player_placeholders++;
            if (sw_visible)
                sw_phantoms++;
            if (gpu_visible)
                gpu_phantoms++;
        }
        int wrote = snprintf(entries + eo, sizeof entries - eo,
                             "%s{\"object\":\"%s\",\"mesh\":\"%s\","
                             "\"x\":%.1f,\"y\":%.1f,\"z\":%.1f,"
                             "\"owner\":\"%s\",\"swVisible\":%d,"
                             "\"gpuVisibleParts\":%d,\"gpuParts\":%d}",
                             near > 1 ? "," : "", scene_obj_label(obj),
                             mesh ? mesh->name : "", pos[0], pos[1], pos[2],
                             player ? "player" : live ? "ai" : "scenery",
                             sw_visible, gpu_visible, gpu_parts);
        if (wrote < 0 || (size_t)wrote >= sizeof entries - eo) {
            eo = sizeof entries - 1;
            entries[eo] = '\0';
            break;
        }
        eo += (size_t)wrote;
    }
    if (eo < sizeof entries - 1) {
        entries[eo++] = ']';
        entries[eo] = '\0';
    }
    snprintf(out + off, sizeof out - off,
             "%d,\"playerPlaceholders\":%d,\"swPhantoms\":%d,"
             "\"gpuPhantoms\":%d,\"swLiveCarParts\":%d,"
             "\"gpuLiveCarParts\":%d,\"entries\":%s}",
             near, player_placeholders, sw_phantoms, gpu_phantoms,
             s_car_nparts, gpu_live_car, entries);
    return out;
}

/* Scene enumeration over scene.c's additive accessors (M6 Tier-2
 * geometry export). web_gpu_scene_part_xform returns a static
 * double[12] (r/u/f/pos columns) or NULL on a bad index. */
EMSCRIPTEN_KEEPALIVE
int web_gpu_scene_obj_count(void) { return scene_obj_count(); }

EMSCRIPTEN_KEEPALIVE
int web_gpu_scene_obj_part_count(int obj)
{
    return scene_obj_part_count(obj);
}

/* Owning paint scheme for placed object `obj` ("" for static scenery). */
EMSCRIPTEN_KEEPALIVE
const char *web_gpu_scene_obj_vtf(int obj)
{
    return scene_obj_vtf(obj);
}

/* Driven car's paint scheme — for car/interior drawlist entries. */
EMSCRIPTEN_KEEPALIVE
const char *web_gpu_car_vtf(void)
{
    return car_is_loaded() ? car_vtf_file() : "";
}

EMSCRIPTEN_KEEPALIVE
double *web_gpu_scene_part_xform(int obj, int part)
{
    static double out[12];
    if (scene_obj_part_xform(obj, part, out) != 0)
        return NULL;
    /* Drive surfaces (gas-station slabs and kin) get the same render-only
     * lift the software rasterizer applies (scene.c SCENE_SURFACE_LIFT_M),
     * so the GPU path stops z-fighting the road under them. Presentation
     * only — the sim's copy of the transform is untouched. */
    if (scene_part_drive_surface(obj, part))
        out[10] += SCENE_SURFACE_LIFT_M;
    return out;
}

EMSCRIPTEN_KEEPALIVE
GeoMesh *web_gpu_scene_part_mesh(int obj, int part)
{
    return scene_obj_part_mesh(obj, part);
}

/* GeoMesh field readers, so the page can copy vertex/index data out of
 * wasm memory without hardcoding the struct layout (geomesh.h). All
 * NULL-safe. */
EMSCRIPTEN_KEEPALIVE
int web_gpu_mesh_num_verts(GeoMesh *m) { return m ? m->num_verts : 0; }

EMSCRIPTEN_KEEPALIVE
int web_gpu_mesh_num_faces(GeoMesh *m) { return m ? m->num_faces : 0; }

EMSCRIPTEN_KEEPALIVE
int web_gpu_mesh_num_indices(GeoMesh *m) { return m ? m->num_indices : 0; }

EMSCRIPTEN_KEEPALIVE
float *web_gpu_mesh_verts(GeoMesh *m) { return m ? m->verts : NULL; }

EMSCRIPTEN_KEEPALIVE
int *web_gpu_mesh_indices(GeoMesh *m) { return m ? m->indices : NULL; }

EMSCRIPTEN_KEEPALIVE
int *web_gpu_mesh_face_first(GeoMesh *m) { return m ? m->face_first : NULL; }

EMSCRIPTEN_KEEPALIVE
int *web_gpu_mesh_face_count(GeoMesh *m) { return m ? m->face_count : NULL; }

/* Material + attribute readers (docs/specs/m6/oeg-face-format.md). These are
 * what turn scene/car meshes from flat arbitrary fills into the authored
 * colours, textures and shading the 1997 hardware renderer drew. */
EMSCRIPTEN_KEEPALIVE
const char *web_gpu_mesh_name(GeoMesh *m) { return m ? m->name : ""; }

EMSCRIPTEN_KEEPALIVE
float *web_gpu_mesh_normals(GeoMesh *m) { return m ? m->normals : NULL; }

EMSCRIPTEN_KEEPALIVE
uint8_t *web_gpu_mesh_face_rgb(GeoMesh *m) { return m ? m->face_rgb : NULL; }

EMSCRIPTEN_KEEPALIVE
uint8_t *web_gpu_mesh_face_flags(GeoMesh *m) { return m ? m->face_flags : NULL; }

/* [num_faces * GEO_TEX_NAME_LEN] NUL-terminated names; "" = untextured. */
EMSCRIPTEN_KEEPALIVE
char *web_gpu_mesh_face_tex(GeoMesh *m) { return m ? m->face_tex : NULL; }

EMSCRIPTEN_KEEPALIVE
int web_gpu_mesh_tex_stride(void) { return GEO_TEX_NAME_LEN; }

EMSCRIPTEN_KEEPALIVE
float *web_gpu_mesh_uvs(GeoMesh *m) { return m ? m->uvs : NULL; }

EMSCRIPTEN_KEEPALIVE
int *web_gpu_mesh_normal_indices(GeoMesh *m)
{
    return m ? m->normal_indices : NULL;
}

/* Combat/mission visibility. scene.c skips hidden objects; the GPU path was
 * drawing destroyed cars and un-revealed ambushers. Flips mid-mission, so
 * callers must re-read it per frame, not once at upload time. */
EMSCRIPTEN_KEEPALIVE
int web_gpu_scene_obj_hidden(int obj) { return scene_obj_hidden(obj); }

/* ------------------------------------------------------------------ */
/* Texture decode slots. Each load frees the previous slot's buffer.  */
/* ------------------------------------------------------------------ */

static uint8_t  *s_gpu_m16_rgba;
static uint32_t s_gpu_m16_w, s_gpu_m16_h;

/*
 * web_gpu_m16_load(name)
 *   Decode a .m16 hardware texture to RGBA8 through m16.c.
 *   `name` is either a VFS-visible file ("zhr45101.m16") or a tile
 *   inside a .pak ("a4tank16.pak#0" — tile index resolved through the
 *   sibling .pix manifest via m16_pix_tile_range, the m16_probe.c
 *   pattern). Returns 0 on success (query via the getters), -1 on any
 *   failure (slot cleared).
 */
EMSCRIPTEN_KEEPALIVE
int web_gpu_m16_load(const char *name)
{
    free(s_gpu_m16_rgba);
    s_gpu_m16_rgba = NULL;
    s_gpu_m16_w = s_gpu_m16_h = 0;
    if (!name || !*name) return -1;

    const uint8_t *bytes = NULL;
    size_t size = 0;
    void *owned = NULL;
    char pak[160];
    const char *hash = strchr(name, '#');
    if (hash) {
        size_t plen = (size_t)(hash - name);
        if (plen == 0 || plen >= sizeof pak) return -1;
        memcpy(pak, name, plen);
        pak[plen] = '\0';
        int index = atoi(hash + 1);
        char pix[160];
        snprintf(pix, sizeof pix, "%s", pak);
        char *dot = strrchr(pix, '.');
        if (!dot) return -1;
        strcpy(dot, ".pix");
        size_t psz = 0;
        char *ptext = vfs_read_file(pix, &psz);
        if (!ptext) return -1;
        char tile[64];
        uint32_t off = 0, len = 0;
        int rc = m16_pix_tile_range(ptext, index, tile, &off, &len);
        vfs_free(ptext);
        if (rc != 0) return -1;
        size_t psz2 = 0;
        owned = vfs_read_file(pak, &psz2);
        if (!owned || (size_t)off + len > psz2) { vfs_free(owned); return -1; }
        bytes = (const uint8_t *)owned + off;
        size = len;
    } else {
        /* Literal-name loads are step 3 of the face-texture fallback
         * chain (web_gpu_face_tex_load): a miss is a normal fallthrough
         * to the .pix/.pak scan and then the software .map path, so it
         * takes the probe read (silent consumer page, once-per-name
         * under ?dev=1) instead of the loud one. */
        owned = vfs_try_read(name, &size);
        if (!owned) return -1;
        bytes = owned;
    }

    uint32_t w = 0, h = 0;
    uint8_t *rgba = m16_decode_rgba(bytes, size, &w, &h, NULL);
    vfs_free(owned);
    if (!rgba) return -1;
    s_gpu_m16_rgba = rgba;
    s_gpu_m16_w = w;
    s_gpu_m16_h = h;
    return 0;
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_gpu_m16_w(void) { return s_gpu_m16_w; }

EMSCRIPTEN_KEEPALIVE
uint32_t web_gpu_m16_h(void) { return s_gpu_m16_h; }

EMSCRIPTEN_KEEPALIVE
const uint8_t *web_gpu_m16_rgba(void) { return s_gpu_m16_rgba; }

/* ------------------------------------------------------------------ */
/* Terrain surface texture (the mission-faithful GPU terrain skin).    */
/* The mission WDEF/WRLD chunk names it at payload +108 (FACT:         */
/* docs/specs/m2/scene.md §2 — "Terrain surface texture (.map)", 13-   */
/* byte field). The tile itself lives inside a *m.pak indexed by its   */
/* sibling .pix manifest (e.g. TT161SD_.MAP in tt16m.pak). DECISION    */
/* (.map decode): the .map tile is u32 w, u32 h, then w*h u8 palette   */
/* indices — verified against tt16m.pak tile 0 (128x128, 16392 bytes   */
/* = 8 + 128*128). Indices resolve through the level .act palette      */
/* (FACT: docs/specs/m2/terrain.md §4), falling back to the meshview   */
/* debug palette when the mission named none. The whole terrain is     */
/* tiled with this ONE texture (terrain.md §4 — CONFIRMED); uv tiling  */
/* is gpu_scene.mjs's concern. No per-flag texture selection: the      */
/* flag-nibble mechanism is UNKNOWN (terrain.md §4).                   */
/* ------------------------------------------------------------------ */

static uint8_t  *s_gpu_tex;
static uint32_t s_gpu_tex_w, s_gpu_tex_h;

static void gpu_tex_invalidate(void)
{
    free(s_gpu_tex);
    s_gpu_tex = NULL;
    s_gpu_tex_w = s_gpu_tex_h = 0;
}

static int gpu_str_ieq(const char *a, const char *b)
{
    while (*a && *b) {
        char ca = *a, cb = *b;
        if (ca >= 'a' && ca <= 'z') ca = (char)(ca - ('a' - 'A'));
        if (cb >= 'a' && cb <= 'z') cb = (char)(cb - ('a' - 'A'));
        if (ca != cb) return 0;
        a++; b++;
    }
    return *a == *b;
}

/* Scan every .pix manifest in the VFS for a tile named `tile_name`
 * (case-insensitive; manifests are uppercase, WRLD fields lowercase).
 * On hit: pak name into pak_out (pix name with .pak), byte range into
 * off/len, returns 0. O(total .pix bytes) — small, once per mission. */
struct pix_scan_ctx {
    const char *tile;
    char pak[160];
    uint32_t off, len;
    int found;
};

static void pix_scan_cb(const char *name, int src_type, void *ud)
{
    struct pix_scan_ctx *c = ud;
    if (c->found) return;
    size_t n = strlen(name);
    if (n < 4 || strcmp(name + n - 4, ".pix") != 0) return;
    size_t psz = 0;
    char *text = vfs_read_file(name, &psz);
    if (!text) return;
    /* vfs_read_file returns raw file bytes with NO terminator; strtok_r
     * would walk off the end of the allocation (ASan SEGV, and the wasm
     * "memory access out of bounds" this chain hit). Copy into a
     * NUL-terminated scratch before tokenising. */
    char *body = malloc(psz + 1);
    if (!body) { vfs_free(text); return; }
    memcpy(body, text, psz);
    body[psz] = '\0';
    vfs_free(text);
    char *save = NULL;
    for (char *line = strtok_r(body, "\n", &save); line;
         line = strtok_r(NULL, "\n", &save)) {
        char tname[64];
        uint32_t off = 0, len = 0;
        if (sscanf(line, "%63s %u %u", tname, &off, &len) != 3) continue;
        if (!gpu_str_ieq(tname, c->tile)) continue;
        snprintf(c->pak, sizeof c->pak, "%s", name);
        strcpy(c->pak + n - 4, ".pak");
        c->off = off;
        c->len = len;
        c->found = 1;
        break;
    }
    free(body);
}

/*
 * web_gpu_terrain_tex()
 *   Resolve + decode the loaded mission's terrain surface texture to
 *   RGBA8 (cached; invalidated at mission replacement by
 *   webgpu_caches_reset, which web_mission_load/shell_teardown call
 *   before the old assets unload). Returns 0 when a
 *   texture is available (query via the getters), -1 when the mission
 *   names none / the tile is unresolvable — the page then falls back to
 *   a generated checker (gpu_scene.mjs DECISION).
 */
EMSCRIPTEN_KEEPALIVE
int web_gpu_terrain_tex(void)
{
    if (s_gpu_tex) return 0;
    char tile[14];
    if (mission_wrld_field(s_mission_path, 108, 13, tile, sizeof tile, 1) != 0)
        return -1;

    struct pix_scan_ctx ctx = { tile, "", 0, 0, 0 };
    vfs_foreach(pix_scan_cb, &ctx);
    if (!ctx.found) return -1;
    size_t psize = 0;
    uint8_t *pak = vfs_read_file(ctx.pak, &psize);
    if (!pak || (size_t)ctx.off + ctx.len > psize || ctx.len < 8) {
        vfs_free(pak);
        return -1;
    }
    const uint8_t *t = pak + ctx.off;
    uint32_t w = (uint32_t)t[0] | ((uint32_t)t[1] << 8) |
                 ((uint32_t)t[2] << 16) | ((uint32_t)t[3] << 24);
    uint32_t h = (uint32_t)t[4] | ((uint32_t)t[5] << 8) |
                 ((uint32_t)t[6] << 16) | ((uint32_t)t[7] << 24);
    if (w == 0 || h == 0 || w > 4096 || h > 4096 ||
        (uint64_t)w * h + 8 > ctx.len) {
        vfs_free(pak);
        return -1;
    }
    const uint8_t *pal = hud_palette();
    if (!pal) pal = (const uint8_t *)s_pal;   /* debug palette fallback */
    uint8_t *rgba = malloc((size_t)w * h * 4);
    if (!rgba) { vfs_free(pak); return -1; }
    const uint8_t *idx = t + 8;
    for (uint64_t i = 0; i < (uint64_t)w * h; i++) {
        rgba[i * 4 + 0] = pal[idx[i] * 3 + 0];
        rgba[i * 4 + 1] = pal[idx[i] * 3 + 1];
        rgba[i * 4 + 2] = pal[idx[i] * 3 + 2];
        rgba[i * 4 + 3] = 255;
    }
    vfs_free(pak);
    s_gpu_tex = rgba;
    s_gpu_tex_w = w;
    s_gpu_tex_h = h;
    return 0;
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_gpu_terrain_tex_w(void) { return s_gpu_tex_w; }

EMSCRIPTEN_KEEPALIVE
uint32_t web_gpu_terrain_tex_h(void) { return s_gpu_tex_h; }

EMSCRIPTEN_KEEPALIVE
const uint8_t *web_gpu_terrain_tex_rgba(void) { return s_gpu_tex; }

/* Authored cloud sky tile (WDEF/WRLD +82) as palette-expanded RGBA. */
static uint8_t  *s_gpu_sky;
static uint8_t  *s_gpu_horizon;
static uint32_t s_gpu_sky_w, s_gpu_sky_h;

static void gpu_sky_invalidate(void)
{
    free(s_gpu_sky);
    s_gpu_sky = NULL;
    free(s_gpu_horizon);
    s_gpu_horizon = NULL;
    s_gpu_sky_w = s_gpu_sky_h = 0;
}

/*
 * Mission-lifetime invalidation boundary for every WebGPU-side cache.
 * MUST run before the old mission's scene/terrain/car unload (both
 * web_mission_load and shell_teardown) so no cache can outlive the
 * assets it was decoded from: LOD/road vertex streams, palette-expanded
 * road RGBA textures, terrain surface + sky decodes, the shared
 * material (.m16/RTex) slot, and the drawlist's retained record buffer
 * (its GeoMesh pointers reference the scene being torn down). After
 * this, getters report empty/NULL until the next build under the new
 * mission — mission B can never observe mission A's data.
 */
static void webgpu_caches_reset(void)
{
    free(s_lod_verts);
    s_lod_verts = NULL;
    s_lod_nverts = 0;
    s_lod_bands[0] = s_lod_bands[1] = s_lod_bands[2] = 0;
    free(s_road_verts);
    s_road_verts = NULL;
    s_road_nverts = 0;
    s_road_by_type[0] = s_road_by_type[1] = s_road_by_type[2] = 0;
    roads_tex_invalidate();
    gpu_tex_invalidate();
    gpu_sky_invalidate();
    free(s_gpu_m16_rgba);
    s_gpu_m16_rgba = NULL;
    s_gpu_m16_w = s_gpu_m16_h = 0;
    free(s_draws);
    s_draws = NULL;
    s_draws_cap = 0;
    s_ndraws = 0;
}

EMSCRIPTEN_KEEPALIVE
int web_gpu_sky_tex(void)
{
    gpu_sky_invalidate();
    const RTex *sky = NULL;
    if (!scene_sky_tex(&sky) || !sky || !sky->texels || sky->w == 0 || sky->h == 0)
        return -1;
    const uint8_t *pal = hud_palette();
    if (!pal) pal = (const uint8_t *)s_pal;
    size_t n = (size_t)sky->w * sky->h;
    uint8_t *rgba = malloc(n * 4);
    if (!rgba) return -1;
    for (size_t i = 0; i < n; i++) {
        uint8_t idx = sky->texels[i];
        rgba[i * 4 + 0] = pal[idx * 3 + 0];
        rgba[i * 4 + 1] = pal[idx * 3 + 1];
        rgba[i * 4 + 2] = pal[idx * 3 + 2];
        rgba[i * 4 + 3] = 255;
    }
    s_gpu_sky = rgba;
    s_gpu_sky_w = sky->w;
    s_gpu_sky_h = sky->h;
    return 0;
}

EMSCRIPTEN_KEEPALIVE
uint32_t web_gpu_sky_tex_w(void) { return s_gpu_sky_w; }

EMSCRIPTEN_KEEPALIVE
uint32_t web_gpu_sky_tex_h(void) { return s_gpu_sky_h; }

EMSCRIPTEN_KEEPALIVE
const uint8_t *web_gpu_sky_tex_rgba(void) { return s_gpu_sky; }

/* Sixteen MAPs in authored list order, one 128-pixel-high atlas row each. */
EMSCRIPTEN_KEEPALIVE
const uint8_t *web_gpu_horizon_tex(void)
{
    free(s_gpu_horizon);
    s_gpu_horizon = NULL;
    const RTex *tiles[16];
    for (int i = 0; i < 16; i++)
        if (!scene_horizon_tex(i, &tiles[i])) return NULL;
    const uint8_t *pal = hud_palette();
    if (!pal) return NULL;
    s_gpu_horizon = malloc(128 * 128 * 16 * 4);
    if (!s_gpu_horizon) return NULL;
    for (int i = 0; i < 128 * 128 * 16; i++) {
        uint8_t index = tiles[i / (128 * 128)]->texels[i % (128 * 128)];
        memcpy(s_gpu_horizon + i * 4, pal + index * 3, 3);
        s_gpu_horizon[i * 4 + 3] = index == 255 ? 0 : 255;
    }
    return s_gpu_horizon;
}

EMSCRIPTEN_KEEPALIVE
void web_set_horizon(int enabled) { scene_horizon_enable(enabled); }

/* Palette-index RTex → shared GPU RGBA slot (s_gpu_m16_*). */
static int gpu_rtex_to_slot(const RTex *tex)
{
    if (!tex || !tex->texels || tex->w == 0 || tex->h == 0) return -1;
    const uint8_t *pal = hud_palette();
    if (!pal) pal = (const uint8_t *)s_pal;
    size_t n = (size_t)tex->w * tex->h;
    uint8_t *rgba = malloc(n * 4);
    if (!rgba) return -1;
    for (size_t i = 0; i < n; i++) {
        uint8_t idx = tex->texels[i];
        if (tex->has_key && idx == RASTER_TEXEL_TRANSPARENT) {
            rgba[i * 4 + 0] = 0; rgba[i * 4 + 1] = 0;
            rgba[i * 4 + 2] = 0; rgba[i * 4 + 3] = 0;
        } else {
            rgba[i * 4 + 0] = pal[idx * 3 + 0];
            rgba[i * 4 + 1] = pal[idx * 3 + 1];
            rgba[i * 4 + 2] = pal[idx * 3 + 2];
            rgba[i * 4 + 3] = 255;
        }
    }
    free(s_gpu_m16_rgba);
    s_gpu_m16_rgba = rgba;
    s_gpu_m16_w = tex->w;
    s_gpu_m16_h = tex->h;
    return 0;
}

/* Live cockpit instrument tiles (same as scene_dyn_add_textured overrides). */
static int gpu_instrument_tex(const char *role)
{
    const RTex *tex = NULL;
    if (strcasecmp(role, "GER6") == 0 || strcasecmp(role, "ZGEAR101") == 0 ||
        strcasecmp(role, "ZGEAR101.MAP") == 0) {
        /* PRND: R if reverse else D (same as web_drive_render). */
        int sel = HUD_SELECTOR_DRIVE;
        tex = hud_cockpit_gear_texture(sel);
    } else if (strcasecmp(role, "CMP6") == 0 || strcasecmp(role, "ZCM_") == 0 ||
               strcasecmp(role, "ZCM_.MAP") == 0 ||
               strcasecmp(role, "ZCM_320.MAP") == 0) {
        tex = hud_cockpit_compass_texture();
    } else if (strcasecmp(role, "RTC6") == 0 || strcasecmp(role, "ZRETC_6") == 0 ||
               strcasecmp(role, "ZRETC_6.MAP") == 0 ||
               strcasecmp(role, "ZRETC_1.MAP") == 0) {
        tex = hud_cockpit_reticle_texture();
    }
    return tex ? gpu_rtex_to_slot(tex) : -1;
}

/*
 * web_gpu_face_tex_load(name, vtf)
 *   Resolve OEG face texture NAME to RGBA in the shared slot
 *   (web_gpu_m16_w/h/rgba). Order:
 *     1. Live instrument overrides (GER6/CMP6/RTC6 / ZGEAR/ZCM_/ZRETC)
 *     2. Paint-resolve vehicle placeholders under VTF (owning vehicle's
 *        scheme — never the player's by default; each draw supplies its own)
 *     3. Literal base.m16 (loose + .pix/.pak)
 *     4. Software .map / .tmt via texcache_load_map (level palette)
 *   Returns 0 on success, -1 when unresolvable (face keeps authored RGB).
 *
 *   vtf is the owning vehicle's .vtf (scene_obj_vtf / car_vtf_file), or ""
 *   / NULL for static scenery. A previous revision always painted through
 *   car_vtf_file(), so every AI vehicle inherited the player's panels.
 */
EMSCRIPTEN_KEEPALIVE
int web_gpu_face_tex_load(const char *name, const char *vtf)
{
    if (!name || !*name) return -1;

    /* Instrument short-circuit (authored MAP names or part role suffixes). */
    if (gpu_instrument_tex(name) == 0) return 0;
    {
        size_t len = strlen(name);
        if (len >= 4 && gpu_instrument_tex(name + len - 4) == 0) return 0;
        /* ZGEAR101.MAP style: strip extension for role match. */
        char bare[64];
        snprintf(bare, sizeof bare, "%s", name);
        char *dot = strrchr(bare, '.');
        if (dot) *dot = '\0';
        if (gpu_instrument_tex(bare) == 0) return 0;
    }

    char base[64];
    char vbase[16];
    if (vtf && vtf[0] &&
        paint_resolve_face(name, vtf, vbase, sizeof vbase) == 0) {
        snprintf(base, sizeof base, "%s", vbase);
    } else {
        snprintf(base, sizeof base, "%s", name);
        char *dot = strrchr(base, '.');
        if (dot) *dot = '\0';
    }
    if (!base[0]) return -1;

    char tile[80];
    snprintf(tile, sizeof tile, "%s.m16", base);
    if (web_gpu_m16_load(tile) == 0) return 0;

    struct pix_scan_ctx ctx = { tile, "", 0, 0, 0 };
    vfs_foreach(pix_scan_cb, &ctx);
    if (ctx.found) {
        size_t psize = 0;
        uint8_t *pak = vfs_read_file(ctx.pak, &psize);
        if (pak && (size_t)ctx.off + ctx.len <= psize) {
            uint32_t w = 0, h = 0;
            uint8_t *rgba = m16_decode_rgba(pak + ctx.off, ctx.len, &w, &h, NULL);
            vfs_free(pak);
            if (rgba) {
                free(s_gpu_m16_rgba);
                s_gpu_m16_rgba = rgba;
                s_gpu_m16_w = w;
                s_gpu_m16_h = h;
                return 0;
            }
        } else {
            vfs_free(pak);
        }
    }

    /* Software twin: .map / .tmt via texcache (cockpit ZDASH/ZBKS etc.). */
    RTex maptex;
    uint8_t *map_px = NULL;
    char mapname[80];
    snprintf(mapname, sizeof mapname, "%s.map", base);
    if (texcache_load_map(mapname, &maptex, &map_px) == 0) {
        int rc = gpu_rtex_to_slot(&maptex);
        free(map_px);
        if (rc == 0) return 0;
    }
    /* Try original name as-is (may already be FOO.MAP). */
    if (texcache_load_map(name, &maptex, &map_px) == 0) {
        int rc = gpu_rtex_to_slot(&maptex);
        free(map_px);
        if (rc == 0) return 0;
    }
    snprintf(mapname, sizeof mapname, "%s.tmt", base);
    if (texcache_load_map(mapname, &maptex, &map_px) == 0) {
        int rc = gpu_rtex_to_slot(&maptex);
        free(map_px);
        if (rc == 0) return 0;
    }
    return -1;
}
