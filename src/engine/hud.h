#ifndef HUD_H
#define HUD_H

/*
 * hud.h — M3 HUD: cockpit dashboard, sprite sheets, text line
 *
 * Renders the cockpit dashboard of the player's car plus a text line into
 * the caller's 8-bit palette-indexed framebuffer.
 *
 * Assets (all via the engine VFS, formats per docs/specs/m2/pipeline.md):
 *   zdash<N>01.pix/.pak — day dashboard for cockpit art class N: .pix
 *      text manifest lists two 256x128 VQM tiles inside the .pak
 *      (ZDASH<N>01 = instrument cluster, ZDASH<N>02 = center console;
 *      stacked into a 256x256 dash). Night variants (zdash*06.pak) are
 *      M16 hardware textures — out of scope for the software path
 *      (pipeline.md §3.3). The class N is scenario-derived — FACT from
 *      nitro.exe (FUN @0x49f7e0 + shipped table @0x503c88): the scenario
 *      name parsed "%1s%2d" maps through the shipped lookup table to the
 *      art class digit used by the cockpit-art format names
 *      (ZSWLS%1d01.tmt, ZNPD%1d01.map, ZHR45%1d01.tmt, ZHL45%1d01.tmt).
 *      See hud_load_mission() for the table and defaults.
 *   vpit.cbk — shared VQM codebook named by the tile headers.
 *   <level>.act — 256x3 RGB level palette (p01.act for Nitro scene 1).
 *      VQM pixels are indices into the *level* palette, not vpit.act
 *      (empirical: vpit.act is a sparse mostly-magenta table — indices
 *      used by the dash art fall in its unassigned magenta range, while
 *      the level palettes resolve to coherent grays/yellows; matches the
 *      CONFIRMED "VQM is palette-indexed against the level .act" note in
 *      I76E:docs/HD-TEXTURES-RESEARCH.md).
 *   vpit_1.elt — text sprite tables (ETBL): "dst|src <sheet>.map" groups
 *      of "label <name> <x> <y> <w> <h>" sub-rects (scene.md §6.2).
 *   zgear101.map / zneedle6.map — gear-indicator plate + gauge-needle
 *      rotation frames referenced by the .elt. The needle sheet is the
 *      live speedo/tach instrument art (D8 in hud.c): 38 rotation
 *      frames, selected by the original's own speed/RPM formulas and
 *      composited over the dash's baked gauge faces.
 *   zmiri101.map / zmiro101.map — authored interior-mirror mask (256x64,
 *      0xFF cutout over the dark palette ramp) and mirror bezel art
 *      (223x72). Decoded for the rearview composite; see
 *      hud_mirror_mask()/hud_mirror_bezel(). Only the set-1/day variant
 *      ships in the Nitro data.
 *   zsy_.map / zsye.map — condition-panel base plus the authored live
 *      engine/suspension/brake/tire/armor/chassis state sprites. The port's
 *      scalar combat HP selects one shared state until component damage lands.
 *   base6x7.fnt — bitmap font via the existing font.c loader.
 *
 * Pixel orientation (pipeline.md open question P3) is resolved
 * EMPIRICALLY, see the DECISION list in hud.c:
 *   .vqm — row-major as documented (block grid L-to-R/T-to-B, row-major
 *          within each 4x4 block, top-down). Verified against i76img.py
 *          byte-for-byte; the column-major reading yields garbage.
 *   .map — row-major byte order but stored BOTTOM-UP (display row 0 is
 *          the last stored row; BMP-style). The zgear101 "PRND21" gear
 *          plate is only upright/readable decoded this way; top-down is
 *          upside-down, column-major is noise.
 */

#include <stddef.h>
#include <stdint.h>

#include "engine/raster.h"

/*
 * hud_load()
 *   Load palette, dashboard tiles, sprite sheets, elt tables and font via
 *   the VFS (fs_set_root + vfs_init must have been called by the host).
 * Returns 0 when the dashboard tiles decoded (the minimum viable HUD),
 *   -1 otherwise. Missing optional pieces (palette, sheets, elt, font)
 *   degrade gracefully and are reported in hud_stats().
 */
int hud_load(void);

/*
 * hud_scenario_art_class(mission)
 *   The cockpit art class digit of the original's ZSWLS%1d01.tmt /
 *   ZNPD%1d01.map / ZHR45%1d01.tmt / ZHL45%1d01.tmt format names — FACT
 *   (nitro.exe FUN @0x49f7e0 + shipped table @0x503c88): scenario name
 *   parsed "%1s%2d", mapped P01-P04/P17/P19 -> 2, P05-P08/P18/B01 -> 3,
 *   P09-P12 -> 4, P13-P16 -> 5; unparseable -> 2, unlisted -> 1.
 *   Stateless; also drives hud_load_mission's dash-set choice.
 *
 * hud_art_class()
 *   The class the loaded HUD derived (1 when hud_load() loaded the
 *   default set directly). 0 before any successful load.
 */
int hud_scenario_art_class(const char *mission);
int hud_art_class(void);

/*
 * hud_load_mission(mission, car_id)
 *   Like hud_load(), but resolves the level palette from the mission
 *   itself: the mission file's WDEF/WRLD world-reference chunk names the
 *   level .act at payload +30 (13-byte field; scene.md §2, CONFIRMED),
 *   e.g. miss8/N01.CBT -> t15.act, miss8/P01.MSN -> p01.act. `mission`
 *   is the same path handed to scene_load(); the same dir-prefix search
 *   (plain, miss8/, miss16/, missions/) applies. When the mission or its
 *   WRLD is unreadable the D3 candidate chain (p01/t01/vpit) is the
 *   safety net.
 *
 *   The dashboard art set is scenario-keyed — FACT (nitro.exe FUN
 *   @0x49f7e0, shipped table @0x503c88): the scenario basename parsed
 *   "%1s%2d" maps P01-P04/P17/P19 -> class 2, P05-P08/P18/B01 -> 3,
 *   P09-P12 -> 4, P13-P16 -> 5; an unparseable name yields 2 and a
 *   well-formed but unlisted name (e.g. N01 melee) yields 1. The day
 *   (01) tile set zdash<class>01 is loaded; when those files are absent
 *   (non-Nitro data) set 1 is the fallback. `car_id` remains reserved:
 *   no vehicle->dash-set mapping exists in the purchaser data or the
 *   binary (the table above is scenario-keyed, not vehicle-keyed).
 */
int hud_load_mission(const char *mission, int car_id);

/*
 * hud_mirror_mask(w, h) / hud_mirror_bezel(w, h)
 *   The authored interior-mirror mask (zmiri101.map) and mirror bezel art
 *   (zmiro101.map), decoded like every .map sheet (bottom-up rows, hud.c
 *   D1) as 8-bit palette indices. The mask is 0xFF where the mirror glass
 *   shows through — the rearview composite clips to it; the bezel is the
 *   opaque surround art. Either returns NULL when its sheet is absent
 *   (out pointers then untouched). Pointers are module-owned and valid
 *   until hud_unload(); dimensions via the out pointers. The mirror's 3-D
 *   surface part is a car-module query (car_interior_mirror_part()).
 */
const uint8_t *hud_mirror_mask(int *w, int *h);
const uint8_t *hud_mirror_bezel(int *w, int *h);

/*
 * Authored 3-D cockpit-surface textures. The 640-mode VGEO records identify
 * their own source maps and UVs:
 *   GER6 -> ZGEAR101.MAP (plus the live ZGEARE selector sprite)
 *   CMP6 -> ZCM_.MAP     (plus the live ZCME bearing crop)
 *   RTC6 -> ZRETC_6.MAP
 * These return level-palette RTex views owned by hud.c, ready for
 * scene_dyn_add_textured(). NULL means the source assets/ELT geometry did not
 * validate. Pointers remain valid until hud_unload().
 */
const RTex *hud_cockpit_gear_texture(int selector);
const RTex *hud_cockpit_compass_texture(void);
const RTex *hud_cockpit_reticle_texture(void);

/*
 * hud_sidearm_frame_count(side) / hud_sidearm_frame(side, i, w, h) /
 * hud_sidearm_frame_name(side, i)
 *   The glance/sidearm art families — FACT names from nitro.exe format
 *   strings ZHL45%1d01.tmt / ZHR45%1d01.tmt (left/right). Each side's
 *   pix manifest lists its VQM frames in archive order (the set-1 family
 *   carries ZHL45../ZSL.. entries on the left, ZHR45../ZSR.. on the
 *   right); the frames
 *   decode like the dash tiles and are exported in manifest order with
 *   their manifest names so the consumer splits backgrounds from
 *   overlays with its own evidence — no frame semantics are invented
 *   here. `side` is HUD_SIDEARM_LEFT or HUD_SIDEARM_RIGHT. The family
 *   follows the scenario art class (hud_scenario_art_class), with the
 *   set-1 fallback when the class family is absent. Night .m16 siblings
 *   are out of scope (same rule as the dash, D4).
 *   Pixels are module-owned, valid until hud_unload(); count 0 and NULL
 *   returns when the family is absent.
 */
#define HUD_SIDEARM_LEFT  0
#define HUD_SIDEARM_RIGHT 1
int             hud_sidearm_frame_count(int side);
const char     *hud_sidearm_frame_name(int side, int index);
const uint8_t  *hud_sidearm_frame(int side, int index, int *w, int *h);

/*
 * hud_render_frame(fb, w, h, selector, speed, lap_ms, flags)
 *   Draw the dashboard (centered at the bottom edge), the sprite-sheet
 *   proof strip (gear plate + needle frames cut per the .elt rects) and
 *   a text line into fb (w*h palette indices). Background is filled with
 *   palette index 0.
 *
 *   Drive parameters:
 * selector - HUD_SELECTOR_* in the shipped PRND21 order; clamped. This is
 * the drive selector, not the automatic transmission's physical gear.
 * The browser currently supplies only Drive or Reverse from the car's
 * actual drive-direction toggle.
 * speed - signed m/s, shown in the status line and — when the dash set
 * carries the two-gauge layout (D8: validated at load from the dash
 * art) — driving the live speedometer needle, converted to the face's
 * mph by the original's own clamp/quantization.
 * lap_ms - lap time in milliseconds, shown as M:SS.T.
 * flags - HUD_FLAG_VITALS swaps the status line's LAP readout for
 * the combat vitals set by hud_set_vitals() (M7: the
 * damage pool; fuel shows only when a model sets it).
 * 0 keeps the default readouts.
 *   The text line is the hud_set_text() override when set, otherwise a
 *   status line composed from selector/speed/lap_ms (font permitting).
 */
enum {
    HUD_SELECTOR_PARK = 0,
    HUD_SELECTOR_REVERSE,
    HUD_SELECTOR_NEUTRAL,
    HUD_SELECTOR_DRIVE,
    HUD_SELECTOR_SECOND,
    HUD_SELECTOR_FIRST
};

#define HUD_FLAG_VITALS 0x1u

/*
 * HUD_FLAG_NO_DASH — skip the decoded lower dashboard and its instruments.
 *   When HUD_FLAG_COCKPIT_LAYOUT is also set, the authored upper radar,
 *   weapon and condition panels remain; this is the original chase-view
 *   composition. Without COCKPIT_LAYOUT, only the text line remains.
 *
 * HUD_FLAG_NO_PROOF — skip the gear-plate/needle decoder proof strip.
 *   That strip belongs to tools/hud_probe.c, not a playing frame.
 *
 * HUD_FLAG_COCKPIT_LAYOUT — draw the 640-mode authored upper panels instead
 *   of pasting the complete 256x256 ZDASH texture into the windshield.
 *   Lower gear/compass/reticle art is mapped onto the authored 3-D surfaces
 *   through hud_cockpit_*_texture(), not duplicated in this 2-D pass.
 */
#define HUD_FLAG_NO_DASH        0x2u
#define HUD_FLAG_NO_PROOF       0x4u
#define HUD_FLAG_COCKPIT_LAYOUT 0x8u
void hud_render_frame(uint8_t *fb, int w, int h,
                      int selector, double speed, uint32_t lap_ms,
                      uint32_t flags);

/*
 * hud_set_vitals(hp, hp_max, fuel_pct)
 *   The combat vitals shown when HUD_FLAG_VITALS is set: hp/hp_max is
 *   the damage pool (-1 hp_max = no combat entity, vitals omitted even
 *   under the flag); fuel_pct 0..100, or -1 = no fuel model (omitted —
 *   the sim has no fuel consumption today). Additive setter in the
 *   hud_set_text pattern; survives hud_unload like the text line.
 */
void hud_set_vitals(int hp, int hp_max, int fuel_pct);

/*
 * hud_set_compass_yaw(yaw)
 *   Set the live vehicle heading for the asset-backed compass panel. `yaw`
 *   uses car_pose() convention: 0 faces world +Z (north), positive turns
 *   left. The HUD converts it to a clockwise compass bearing and crops the
 *   original zcme.map strip through the vpit_1.elt anchors. Presentation
 *   only; no simulation or serialized state.
 */
void hud_set_compass_yaw(double yaw);

/*
 * hud_set_engine_rpm(rpm)
 *   Set the live engine RPM for the tachometer needle (D8 in hud.c).
 *   Additive setter in the hud_set_text pattern; survives hud_unload.
 *   0 (the default) rests the needle at the face's 0 mark; the original's
 *   clamp/quantization is applied at render (nitro.exe: value clamped to
 *   8400, frame = trunc(rpm*0.003 + 1) capped at frame 25). The host
 *   feeds car_rpm(); presentation only, no simulation state.
 */
void hud_set_engine_rpm(double rpm);
/* Selected direct-fire state, rendered independently of hud_set_text so an
 * objective override cannot hide it. `slot` is zero-based; ammo < 0 is INF. */
void hud_set_weapon(const char *name, int slot, int count, int ammo,
                    int ammo_max, int damage);

/*
 * H-UAT-002: radar contacts for the authored green sweep playfield
 * (zradf000.pak tiles under the zradmask.map housing). The caller feeds
 * live contacts once per frame before hud_render_frame(); positions are
 * in the player car's horizontal frame — metres RIGHT and metres AHEAD
 * of the car — so the HUD never needs the car's yaw. Contacts beyond the
 * authored 1 km field are dropped; the rest draw as blips clipped to the
 * playfield circle measured from the housing art itself. `threat`
 * selects the blip colour (hostile red vs contact bright — the port's
 * existing ALLY/THREAT vocabulary; the original's blip colour semantics
 * are not reversed).
 */
#define HUD_RADAR_MAX_CONTACTS 32
#define HUD_RADAR_RANGE_M      1000.0
void hud_clear_radar_contacts(void);
void hud_add_radar_contact(double right_m, double fwd_m, int threat);

/*
 * H-UAT-003: per-component condition state for the authored damage panel
 * (zsy_.map anchors + zsye.map state sprites). Component indices match
 * combat.h's COMBAT_COMP_* order exactly (engine, suspension, brakes,
 * RR/RL/FR/FL tires, F/R/L/B armor, F/R/L/B chassis). Feed once per
 * frame from the combat model's component pools. Until the first
 * hud_set_condition call after a (re)load the panel keeps its legacy
 * scalar behaviour — every region follows hud_set_vitals — so a host
 * without a component model sees no change.
 */
#define HUD_CONDITION_COMPONENTS 15
void hud_clear_conditions(void);
void hud_set_condition(int comp, int hp, int hp_max);

/*
 * H-UAT-011c: target condition readout on the radar unit. vpit_1.elt's
 * "dst zrad.map" table authors two anchors — range_pos (139,4) and
 * led_pos (150,84) — and zdde.map supplies the green/yellow/red/off/drk
 * LED states. zrad.map itself is absent from nitro.zfs (hud.c D5), so
 * the PORT DECISION is to resolve those anchors in the radar housing's
 * own space (the zradmask.map origin, where the sweep/overlay already
 * composite): both land on housing chrome outside the measured playfield
 * circle, which is consistent with a bezel LED and range readout. The
 * LED follows the target's live scalar pool through the same 2/3-1/3
 * thresholds as the condition panel; the range draws with the znbe.map
 * digit strip. The target NAME line and thin condition bar under the
 * housing are PORT PRESENTATION (no authored name/bar anchor is
 * decoded). Feed once per frame; hud_clear_target hides all of it.
 */
void hud_set_target(const char *name, int hp, int hp_max, double range_m);
void hud_clear_target(void);
/* Populate the five authored weapon-panel rows before hud_render_frame().
 * Extra loadout slots remain keyboard-selectable but do not fit this panel. */
void hud_clear_weapon_rows(void);
void hud_set_weapon_row(int row, const char *name, int ammo, int ammo_max);
/* Light the row's "on" strip when the hardpoint is in the armed fire set
 * (selected hardpoint + class-linked peers). Call after row setup. */
void hud_set_weapon_row_armed(int row, int armed);
/* Read-only audit of the rows whose authored-strip, bitmap-font fallback,
 * and ammo draw paths completed in the most recent hud_render_frame(). Bit N
 * corresponds to panel row N. */
void hud_weapon_row_render_masks(uint32_t *authored, uint32_t *fallback,
                                 uint32_t *ammo);

/* One-line summary into buf (snprintf semantics/result). */
int hud_stats(char *buf, size_t n);

/* Free all loaded HUD state. */
void hud_unload(void);

/*
 * hud_palette()
 *   The loaded 256*3-byte RGB level palette, or NULL when no .act was
 *   found. In the NULL case hud_render_frame emits the meshview debug
 *   palette indices (0 bg, 1 green, 2 yellow, 3 white, 4 dim) instead of
 *   real art indices, so the web shell's fixed palette still works.
 */
const uint8_t *hud_palette(void);

/*
 * hud_set_text(text)
 *   Set the text line drawn by hud_render_frame (7-bit ASCII; the stock
 *   fonts cover 128 glyphs). Copied internally, truncated at 127 chars.
 */
void hud_set_text(const char *text);

/*
 * hud_set_orientation(mode) — P3 EXPERIMENT HOOK, call before hud_load().
 *   HUD_ORIENT_SHIPPING    (0) — resolved decodes (vqm row-major,
 *                                .map row-major bottom-up). Default.
 *   HUD_ORIENT_MAP_TOPDOWN (1) — .map without the bottom-up flip.
 *   HUD_ORIENT_COLUMN      (2) — Open76's column-major reading of both
 *                                .map and .vqm (observed: garbage).
 * Exists so tools/hud_probe.c can emit the alternate-orientation frames
 * that evidence the P3 DECISION; not a runtime switch.
 */
enum {
    HUD_ORIENT_SHIPPING = 0,
    HUD_ORIENT_MAP_TOPDOWN = 1,
    HUD_ORIENT_COLUMN = 2
};
void hud_set_orientation(int mode);
int  hud_orientation(void);

#endif /* HUD_H */
