#ifndef SCENE_H
#define SCENE_H

/*
 * scene.h — M2 scene-object module
 *
 * Loads a mission (.msn, BWD2 chunk container), parses the ODEF object
 * placement records (100-byte OBJ: packed label, right/up/forward basis +
 * absolute position, classId, flags, team), resolves each placed object to
 * its geometry through the placement chain
 *
 *     OBJ(class 1 car)   -> <label>.vcf -> <vdf>.vdf (VGEO parts, WLOC wheel
 *                          frames) -> <wdf>.wdf (WGEO wheel parts)
 *     OBJ(static classes)-> <label>.sdf -> SGEO parts
 *     part name          -> "<part>.geo" record: loose .geo, or an OEG record
 *                          inside a g-tier .pak located via the merged .pix
 *                          index (the original VFS's virtual file table)
 *
 * and renders the result as a wireframe into a caller-supplied 8-bit
 * palette-indexed framebuffer (meshview palette indices: 0 bg, 1 wire,
 * 2 verts).
 *
 * Byte layouts follow docs/specs/m2/scene.md; where the Nitro Pack data
 * diverges from the spec the choice is marked DECISION in scene.c.
 *
 * Usage:
 *   fs_set_root(<asset dir>); vfs_init();
 *   scene_load("p01.msn");                    // missions resolve via miss8/
 *   scene_render(fb, 640, 480, yaw, dist);    // dist <= 0 => auto-fit
 *   scene_stats(buf, sizeof buf);
 *   scene_unload();
 */

#include <stddef.h>
#include <stdint.h>

#include "engine/geomesh.h"
#include "engine/raster.h"
#include "engine/camera.h"
#include "engine/combat.h"       /* CombatFx (scene_render_combat_fx) */

/*
 * scene_render_combat_fx(fb, w, h, camera, fx, nfx, target_xyz)
 *   H-UAT-007: draw the combat presentation contracts over the world
 *   frame, under the HUD layer the caller composites afterwards:
 *   LIVE projectile trails from the real asynchronous CombatProjectile
 *   poses, ORDF-classed MG/cannon tracers, missile smoke, flame/gas plumes,
 *   authored ORDF target-class impact flipbooks, muzzle flash, native car
 *   explosion/debris/wreck smoke, and every deployed object's authored OGEO,
 *   plus the target marker
 *   (target_xyz, from combat_target_marker; NULL = no marker). All state
 *   comes from combat_fx_snapshot(); ages/trails advance on combat ticks,
 *   so the pass is deterministic and pure. scene.c deliberately
 *   never links combat.c (sky_probe and friends link scene without
 *   mission): the snapshot parameter is the seam.
 */
void scene_render_combat_fx(uint8_t *fb, int w, int h,
                            const CameraView *camera,
                            const CombatFx *fx, int nfx,
                            const double *target_xyz);
/* Authored deployer/impact submissions in the latest pass. Gameplay radii
 * never enter these counters or the presentation snapshot. oil-fallback is
 * retained as an audit compatibility field and remains zero. */
int scene_fx_authored_deployer_draws(void);
int scene_fx_oil_fallback_draws(void);
const char *scene_fx_authored_deployer_model(void);
unsigned scene_fx_authored_deployer_models(void); /* OGEO identity bitset */
int scene_fx_impact_authored_draws(void);
int scene_fx_impact_missing_draws(void);
/* D-C27 kill/wreck sequence: authored flipbook/smoke/debris submissions;
 * missing counters fail closed and draw no spherical fallback. */
int scene_fx_kill_authored_draws(void);
int scene_fx_kill_fallback_draws(void);

/* Load mission `name` (e.g. "p01.msn") through the engine VFS.
 * Returns 0 on success, -1 if the mission file cannot be read/parsed. */
int  scene_load(const char *name);

/* scene_load also owns native drivable-static registration: exact upward
 * OEG faces from SDFC/ODEF classes 11/12/13 are copied into terrain's
 * mesh-aware chassis registry (class 12: SGEO pclass-13 decks only).
 * scene_unload clears it. This is independent of the D11 terrain-owned slab
 * classifier below and never applies SCENE_SURFACE_LIFT_M to simulation. */

/*
 * The mission's authored sky (WDEF/WRLD +82; scene.md §2): the .map entry
 * name as written (lowercased, "" when the field is absent), and the
 * decoded tile when it resolved and decoded cleanly. scene_sky_tex()
 * returns 0 — and the renderer falls back to its placeholder gradient —
 * for a missing, unresolvable or corrupt asset. The returned pointer is
 * valid until the next scene_load/scene_unload.
 */
/* Authored WRLD+134 horizon list; sixteen 128x128 transparent MAP tiles.
 * Graphics option defaults ON and persists across mission replacement. */
const char *scene_horizon_name(void);
int         scene_horizon_tex(int slot, const RTex **out);
void        scene_horizon_enable(int enabled);

const char *scene_sky_name(void);
int         scene_sky_tex(const RTex **out);

/*
 * The mission's native shade/translucency tables (WDEF/WRLD +43/+56;
 * scene.md §2, CONFIRMED — nitro.exe FUN_004b45f0 reads the same fields at
 * blob+0x33/+0x40, docs/specs/re/phase-d-renderer.md §1). Both are exactly
 * 65536 bytes: the .lum is the [light][texel] shade remap raster.h's
 * raster_set_shade_table() consumes, the .tbl the (texel << 8) | dest
 * translucency LUT consumed by raster_set_translucency(). The names are as
 * written, lowercased ("" when the field is absent); the table pointers are
 * NULL when the named asset was missing, unresolvable, or not the evidenced
 * 65536-byte size. Pointers are scene-owned and valid until the next
 * scene_load/scene_unload.
 */
const char    *scene_lum_name(void);
const char    *scene_tbl_name(void);
const uint8_t *scene_shade_table(void);
const uint8_t *scene_translucency_table(void);

/*
 * Bumped by every scene_unload (which scene_load runs first), so a renderer
 * that pushed these tables into raster.c can tell "same buffers, new
 * mission" apart from "nothing changed" without a 128 KB compare per frame.
 */
unsigned scene_table_generation(void);

/*
 * Render every placed object's wireframe into fb (w*h palette indices).
 * Camera orbits the scene centroid at distance `dist` (meters; dist <= 0
 * auto-fits the whole scene) and azimuth `yaw` (radians); elevation and FOV
 * are fixed inside the module (DECISION — the signature carries yaw/dist
 * only). Palette indices: 1 = wire, 2 = vertex dots, background = 0.
 */
void scene_render(uint8_t *fb, int w, int h, double yaw, double dist);

/* Explicit canonical-camera variant for gameplay rendering. */
void scene_render_camera(uint8_t *fb, int w, int h,
                         const CameraView *camera);

/*
 * M8 filled path: draw the placed objects and dynamic meshes into the
 * caller's shared target using the same canonical camera basis.
 */
void scene_render_filled(RTarget *t, const CameraView *camera);

/* "faces=N backfaced=N cutout_skipped=N textured=N" — cut-out faces are
 * skipped in V1 (no textures to key against) and counted so the omission is
 * visible; `textured` is the count of faces whose name resolved to a tile. */
int  scene_filled_stats(char *buf, size_t n);

/*
 * Queue a dynamic mesh drawn after the placed objects (e.g. the player car
 * under sim control). r/u/f are the world basis COLUMNS — the same
 * convention as the ODEF OBJ records — and pos the world position. The mesh
 * is owned by the caller and must outlive its use here; the scene module
 * never frees it. m = NULL clears the slot; scene_unload also drops it.
 *
 * Equivalent to scene_dyn_clear() + scene_dyn_add(): the single-mesh entry
 * point predates the multi-part queue and stays for its callers.
 */
void scene_set_dynamic(GeoMesh *m, const double r[3], const double u[3],
                       const double f[3], const double pos[3]);

/*
 * Multi-part dynamic queue (drive presentation): the player vehicle is
 * every decoded VGEO body/wheel part plus mounted GGEO weapon parts at
 * car_basis o car-model frame, not one mesh. scene_dyn_clear() empties the
 * queue (draws nothing until the next add); scene_dyn_add() appends one part
 * and returns its slot, or -1 when the queue is full or m is NULL. Both are
 * O(1) against fixed storage -- no allocation -- so the drive render can
 * rebuild the queue every frame; per-slot face-tile caches are keyed by mesh
 * pointer and resolve once per mission, not per frame. Queue order is draw
 * order (depth buffering makes it irrelevant to the filled path). Capacity
 * covers the measured 31-part Rampage plus its mounted weapons.
 */
#define SCENE_DYN_MAX 64
void scene_dyn_clear(void);
int  scene_dyn_add(GeoMesh *m, const double r[3], const double u[3],
                   const double f[3], const double pos[3]);

/*
 * Source-texture override for one queued part. The cockpit's 640-mode GER6,
 * CMP6 and RTC6 meshes carry the exact ZGEAR101.MAP, ZCM_.MAP and
 * ZRETC_6.MAP surfaces; their live selector/bearing composites are owned by
 * hud.c and passed here instead of replacing the authored geometry with a
 * screen-space overlay. `tex` is borrowed through the next render only.
 * NULL is identical to scene_dyn_add().
 */
int  scene_dyn_add_textured(GeoMesh *m, const double r[3], const double u[3],
                            const double f[3], const double pos[3],
                            const RTex *tex);

/*
 * Paint scheme (.vtf) of the dynamic mesh, i.e. car_vtf_file() for the driven
 * car. Separate from scene_set_dynamic because it identifies the car rather
 * than its pose: the transform is pushed every frame, the paint once per
 * mission. Without it the player's own panels resolve to no texture at all —
 * a vehicle face names a paint SLOT ("V1 FT LF.MAP"), not a tile. NULL or ""
 * means untextured vehicle panels.
 */
void scene_set_dynamic_paint(const char *vtf);

/*
 * Resolve a bare part name ("FY11BDYF" or "FY11BDYF.GEO") to its decoded
 * mesh via the merged g.pix index (plus whole-file fallbacks). Returns NULL
 * if unresolvable. The mesh stays owned by the scene module — valid until
 * scene_unload; do not free or geo_cache_release it.
 */
GeoMesh *scene_part_mesh(const char *part_name);

/*
 * Read-only enumeration of the placed objects (M6 Tier-2 geometry export;
 * additive accessors, no effect on existing paths). Pointers stay owned by
 * the scene module — valid until scene_unload.
 *   scene_obj_count:       placed-object count (0 when nothing loaded).
 *   scene_obj_part_count:  part count of object i (0 on bad index).
 *   scene_obj_part_active: nonzero while part j participates in rendering,
 *                          export, and collision (triggerGate opens a class-7
 *                          part by making it inactive).
 *   scene_obj_part_gate:   nonzero when part j is the authored class-7 gate.
 *   scene_obj_part_mesh:   decoded mesh of active object i part j (NULL on
 *                          bad/inactive index).
 *   scene_obj_part_xform:  composed draw transform of active object i part j
 *                          as out12 = right[3], up[3], forward[3], pos[3]
 *                          (scene.c D4 basis-columns convention). Uses the
 *                          prepared presentation pose while it is live,
 *                          otherwise the sim world. 0 on success, -1 on
 *                          bad/inactive index or NULL out.
 */
int      scene_obj_count(void);
int      scene_obj_part_count(int obj);
int      scene_obj_part_active(int obj, int part);
int      scene_obj_part_gate(int obj, int part);
GeoMesh *scene_obj_part_mesh(int obj, int part);
int      scene_obj_part_xform(int obj, int part, double out12[12]);

/*
 * Spawn/scenery audit metadata. A vehicle-mesh object is a class-1 ODEF
 * placement whose VCF -> VDF -> VGEO chain produced at least one decoded
 * part; bare SPAWN/REGEN/CHECK markers therefore return false. The label is
 * scene-owned and remains valid until scene_unload().
 */
const char *scene_obj_label(int obj);
int         scene_obj_class_id(int obj); /* authored ODEF runtime class */
/* SDFC +40 health, -1 when absent (zero remains authored/inert). The mutable
 * pool is for scenery without an FSM combat owner; registered bodies use
 * combat_hp/max. Damage hides destroyed scenery through the normal scene API. */
int         scene_obj_hp(int obj, int maximum);
int         scene_obj_damage(int obj, int damage);
int         scene_obj_is_vehicle_mesh(int obj);

/*
 * Paint scheme (.vtf) for placed object `obj` — the VCFC +29 field set when
 * the object was built as a car, else "". Used by the GPU face-texture path
 * so AI vehicles resolve paint slots under THEIR scheme, not the player's.
 * Pointer owned by the scene module; valid until scene_unload. "" on a bad
 * index or when no scene is loaded.
 */
const char *scene_obj_vtf(int obj);

/*
 * M7 story-motion write-back (additive; no effect on existing paths).
 * mission.c's AI mover uses these to move placed objects on screen.
 *   scene_obj_find:    placed-object index matching an ODEF label +
 *                      duplicate id (the label_unpack packing the FSM
 *                      entity table uses; first name match when the id
 *                      misses, mirroring mission.c D4). -1 when no scene
 *                      is loaded or nothing matches.
 *   scene_obj_set_pos: overwrite object `obj`'s world translation; parts
 *                      follow the object transform at render time.
 *                      0 on success, -1 on bad index/NULL.
 */
int  scene_obj_find(const char *label, int label_id);
int  scene_obj_set_pos(int obj, const double pos[3]);

/*
 *   scene_obj_find_nth: the nth (0-based) placed object carrying `label`, in
 *                       ODEF file order; -1 when there are fewer. Needed
 *                       because label ids are NOT unique — N02.CBT's eight
 *                       `spawn` markers share five ids — so scene_obj_find
 *                       collapses distinct grid slots onto one object.
 *   scene_obj_pos:      world translation of a placed object (the read side
 *                       of scene_obj_set_pos). 0 / -1 on a bad index.
 */
int  scene_obj_find_nth(const char *label, int nth);
int  scene_obj_pos(int obj, double out[3]);
/* Sim world as right/up/forward/pos columns (same layout as
 * scene_obj_part_xform). Collision and AI write-back keep using this. */
int  scene_obj_world_xform(int obj, double out12[12]);
/* Draw world: the prepared presentation pose while scene_present_prepare
 * is live, otherwise the sim world. */
int  scene_obj_draw_xform(int obj, double out12[12]);

/*
 * Presentation interpolation of placed objects. Simulation poses stay on
 * the 20 Hz write-back. The drive page captures the last two tick worlds
 * and prepares a read-only draw pose between them. scene_obj_pos, OBBs,
 * and collision keep the sim world. scene_obj_part_xform and the
 * software/GPU draw paths consume the prepared pose while it is live.
 *
 *   scene_present_reset:        drop history (load/unload/cuts).
 *   scene_present_capture_tick: shift prev<-curr, curr<-world.
 *   scene_present_prepare:      lerp prev->curr, or snap to current
 *                               when clamp!=0 or the object teleported.
 */
void scene_present_reset(void);
void scene_present_capture_tick(void);
void scene_present_prepare(double alpha, int clamp);

/*
 * Melee opponent spawner (additive; nothing pre-existing calls either).
 *
 *   scene_obj_make_car:  run the ordinary <vcf_base>.vcf -> .vdf -> .wdf
 *                        placement chain onto an ALREADY PLACED object,
 *                        replacing its parts and keeping its world frame,
 *                        label, id, class and team. This is how a `.CBT`
 *                        map's bodiless class-1 `spawn` marker becomes a
 *                        drawable opponent: the 24 melee maps ship 226 of
 *                        those markers and not one vehicle. Returns the
 *                        part count placed (0 = asset did not resolve),
 *                        -1 on a bad index/argument.
 *   scene_obj_set_facing: turn a placed object to face a horizontal
 *                        direction, position unchanged. A direction, not an
 *                        angle, so no sin/cos reaches a rendered pose.
 *                        Returns 0, or -1 on a bad index or a zero-length
 *                        direction (the object keeps its old facing).
 */
int  scene_obj_make_car(int obj, const char *vcf_base);
int  scene_obj_set_facing(int obj, double fx, double fz);
/* Ground-vehicle presentation frame: forward is projected onto the supplied
 * terrain normal, then right = up x forward. This is the slope-aware sibling
 * of scene_obj_set_facing; translation is unchanged. */
int  scene_obj_set_ground_facing(int obj, double fx, double fz,
                                 double nx, double ny, double nz);
/* Exact car.h Euler frame: yaw, then nose-up pitch about right, then
 * right-side-up roll about forward. Translation is unchanged. */
int  scene_obj_set_car_facing(int obj, double yaw, double pitch, double roll);

/*
 * M7 visibility store plus drive-load consumption. Script/combat visibility
 * remains mutable through scene_obj_set_hidden. Consumption is separate and
 * monotonic for the loaded scene: the player's authored ODEF body is identity
 * and spawn data, but the physical car's dynamic draw is its only render body.
 *   scene_obj_set_hidden: hidden != 0 applies script/combat hiding.
 *   scene_obj_consume:    permanently removes this placement from shared
 *                         software/WebGPU scenery membership for this load.
 *   scene_obj_hidden:     effective render state (hidden OR consumed).
 *   scene_obj_consumed:   distinguishes permanent removal from mutable hide.
 * All return 0 on success (or 0=false for getters), -1 on bad setters.
 */
int  scene_obj_set_hidden(int obj, int hidden);
int  scene_obj_consume(int obj);
int  scene_obj_hidden(int obj);
int  scene_obj_consumed(int obj);

/*
 * M7 gate children. The SGEO part record's +0x5c class word marks gate
 * parts (class 7 — afence5's AF1_GAT5, bcaikgat's CA1_GAT1, bcaik6's
 * CA1_DR61 are the three shipped examples). In the original these become
 * class-7 child entities; triggerGate walks that link (FUN_0045a550) and
 * zeros the first gate component's state (FUN_00456270, gate+0x90).
 *
 * The port gives that state one conservative consumer: state 0 removes the
 * gate part from render/export geometry and from static collision. It does
 * not invent the original gate animation. scene_gate_generation changes
 * only on a real transition so hosts can rebuild their fixed collider table.
 * scene_gates_reset closes every gate when a caller re-attaches a runner to
 * an already-loaded scene.
 */
int      scene_obj_gate_state(int obj);
int      scene_obj_trigger_gate(int obj);
void     scene_gates_reset(void);
unsigned scene_gate_generation(void);

/*
 * scene_obj_bounds(obj, centre, radius)
 *   World-space horizontal bounding circle of placed object `obj`, over all
 *   its parts' decoded mesh bounding boxes composed through their part
 *   transforms. `centre` is the world position (y = the object's own
 *   translation); `radius` is the max horizontal extent from it.
 *   Feeds car.h car_set_colliders — the sim never sees scene.c itself.
 *   Returns 0 on success, -1 on a bad index or an object with no geometry.
 */
int  scene_obj_bounds(int obj, double centre[3], double *radius);

/*
 * scene_obj_obb(obj, centre, half, axis, yspan)
 *   World-space ORIENTED bounding box of placed object `obj` — the box the
 *   sim actually collides against (car.h D19). Built from the object's
 *   own-frame AABB over its parts' mesh bboxes, then placed by the object
 *   transform, so it hugs elongated geometry instead of the loose circle
 *   scene_obj_bounds returns.
 *     centre[3] — world centre of the box
 *     half[2]   — half-extents along the box's own x/z axes, meters
 *     axis[2]   — unit world-XZ vector of the box's local +x axis
 *     yspan[2]  — world { min y, max y }, for the collider height test
 *   Returns 0 on success, -1 on a bad index or an object with no geometry.
 */
int  scene_obj_obb(int obj, double centre[3], double half[2], double axis[2],
                   double yspan[2]);
/* Signed world-Y gaps over every transformed authored mesh vertex relative to
 * the supplied terrain height function. Used to seat pitched NPC geometry. */
int  scene_obj_ground_gaps(int obj, double (*height_at)(double, double),
                           double *min_gap, double *max_gap);

/*
 * scene_obj_part_obb(obj, part, centre, half, axis, yspan)
 *   World-space oriented bounding box of ONE part of placed object `obj`,
 *   built exactly like scene_obj_obb with the union pass narrowed to that
 *   part. The sim's static-scenery colliders are per part (D11): one
 *   aggregate box per object cannot represent a building standing on its
 *   own drive-on apron — P01's bflgila1 unioned 21 parts into a 60x37 m
 *   box that walled off the whole gas-station lot.
 *   Returns 0 on success, -1 on a bad index or a part with no geometry.
 */
int  scene_obj_part_obb(int obj, int part, double centre[3], double half[2],
                        double axis[2], double yspan[2]);

/* Render-only lift applied to drive-surface parts, in metres — the
 * terrain.c ROAD_LIFT_M value. Exposed (not scene.c-private) so the GPU
 * export path applies the SAME constant rather than a second hardcode. */
#define SCENE_SURFACE_LIFT_M 0.1

/*
 * scene_part_drive_surface(obj, part)
 *   D11 drive-surface classification, shared by the collider build and the
 *   filled renderer: nonzero when part `part` of placed object `obj` is a
 *   patch of ground rather than a structure — large (both horizontal
 *   half-extents >= 2 m), near-horizontal, thin (vertical span <= 0.5 m,
 *   far under the car body band), and sitting at the object's base (within
 *   1 m of its lowest part bottom, so roof decks fail). Pure decoded-
 *   geometry policy: no labels and no mission coordinates. Collision skips
 *   these parts (the car rides the terrain across them); the renderer
 *   lifts them 0.1 m (the terrain.c ROAD_LIFT_M precedent) so a terrain-
 *   conformed slab stops trading raster depth ties with the heightfield.
 *   Returns 0 on a bad index too — an unresolvable part is never ground.
 */
int  scene_part_drive_surface(int obj, int part);

/* Encoded terrain drivable-registry owner for a class-11/13 part or a
 * class-12 pclass-13 deck part. Zero means the part has no decoded top-face
 * owner; positive values are opaque tokens consumed by CarCollider. */
int  scene_part_drivable_object(int obj, int part);
/* Parent token only for a class-12 non-deck part whose coarse OBB is the
 * deck's near-identical underlay/overhang. Rails and pylons do not qualify.
 * This does not classify the sibling as drivable; it only scopes bounded
 * exact-face contact samples to the associated structure. */
int  scene_part_drivable_parent(int obj, int part);

/* Return the world position of the first marker object (SPAWN/REGEN/CHECK,
 * class 1), or -1 when the mission has none. Used to place the player car. */
int  scene_first_marker_pos(double out[3]);

/* One-line summary: mission, object counts by kind, mesh hit/miss, world
 * extents. Returns the snprintf length, or -1 on bad arguments. */
int  scene_stats(char *buf, size_t n);

/* Free all mission state and release cached meshes. Idempotent. */
void scene_unload(void);

#endif /* SCENE_H */
