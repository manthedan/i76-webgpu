#ifndef TERRAIN_H
#define TERRAIN_H

/*
 * terrain.h — M2 terrain: .TER heightmap blocks + mission TDEF/ZMAP/ZONE
 *
 * Format summary (docs/specs/m2/terrain.md):
 *   .ter  = N concatenated 32 KiB blocks, each 128x128 little-endian u16
 *           samples (bits 11..0 = height, bits 15..12 = terrain flags).
 *           5 m sample spacing; one block covers a 640 m x 640 m patch.
 *   The mission file (.msn/.cbt/.rac — BWD2 container) TDEF chunk holds:
 *     ZMAP: u8 count + u8 zone[80][80] grid of .ter block indices
 *           (0xFF = empty flat patch). Grid (0,0) is the bottom-left of
 *           the map; increasing x = east, increasing z = north.
 *     ZONE: u8 unknown + char[13] .ter filename (authoritative over the
 *           same-basename-as-mission convention).
 *   Height scale: 0.1 m per LSB (Open76's mapping; spec T3 — INFERRED).
 *   World units are meters, absolute mission coordinates (no recentering,
 *   spec §3.4).
 */
#include <stddef.h>
#include <stdint.h>

#include "engine/raster.h"
#include "engine/camera.h"

#define TERRAIN_GRID_DIM        80                /* zone map patches/side */
#define TERRAIN_PATCH_DIM       128               /* samples/patch side    */
#define TERRAIN_BLOCK_BYTES     (128 * 128 * 2)   /* 32 KiB                */
#define TERRAIN_EMPTY_CELL      0xFFu             /* zone grid: no block   */
#define TERRAIN_SAMPLE_STEP_M   5.0               /* meters per sample     */
#define TERRAIN_PATCH_SIZE_M    640.0             /* meters per patch      */
#define TERRAIN_HEIGHT_SCALE    0.1               /* meters per height LSB */

/* View modes for terrain_render (select with terrain_set_view). */
enum {
    TERRAIN_VIEW_TOPDOWN = 0,   /* north-up shaded heightmap (default)     */
    TERRAIN_VIEW_PERSPECTIVE    /* orbiting wireframe heightfield          */
};

/*
 * terrain_load(name)
 *   Load the terrain set for a mission. `name` is the mission file path as
 *   resolved by the engine VFS (e.g. "miss8/n01.cbt"). The .ter file name
 *   comes from the mission's ZONE chunk, resolved relative to the mission's
 *   directory first, then bare (when no ZONE chunk exists the .ter name is
 *   derived from the mission basename). Returns 0 on success, -1 on failure.
 */
int  terrain_load(const char *name);

/*
 * terrain_render(fb, w, h, yaw, dist)
 *   Render the loaded terrain into the caller's 8-bit palette-indexed fb
 *   (w*h bytes) using the meshview palette indices:
 *     0 = background, 1 = green (terrain), 2 = yellow (accents),
 *     3 = white (peaks), 4 = dim (coast/far/contour lines).
 *   TERRAIN_VIEW_TOPDOWN: north-up orthographic heightmap, auto-fit to fb;
 *     yaw and dist are ignored.
 *   TERRAIN_VIEW_PERSPECTIVE: camera orbits the used-terrain center at
 *     azimuth `yaw` radians and distance `dist` meters (dist <= 0 = auto
 *     from the used extent), fixed elevation, true vertical scale.
 */
void terrain_render(uint8_t *fb, int w, int h, double yaw, double dist);

/*
 * Canonical-camera gameplay renderers. `far` <= 0 selects the wire path's
 * 3000 m default. Neither function clears the caller-owned framebuffer;
 * terrain and scene share the filled target's depth buffer.
 */
void terrain_render_filled(RTarget *t, const CameraView *camera);

/* World-compositor form: caller supplies w*h native painter/view-Z owners.
 * The two-argument wrapper above provides bounded scratch for direct callers. */
void terrain_render_filled_order(RTarget *t, const CameraView *camera,
                                 RPainterPixel *painter_order);
void terrain_render_camera(uint8_t *fb, int w, int h,
                           const CameraView *camera, double far);

/* One-line summary into buf (counts, extents; snprintf semantics/result). */
int  terrain_stats(char *buf, size_t n);

/* Free all loaded terrain state. */
void terrain_unload(void);

/* Select/query the render view mode (see the enum above). */
void terrain_set_view(int mode);
int  terrain_view(void);

/*
 * terrain_height_at(wx, wz)
 *   Bilinear-interpolated terrain height in meters at absolute world
 *   position (wx, wz). Adjacent present patches share the rendered five-metre
 *   seam cell (sample 127 -> the neighbour's sample 0); an absent neighbour
 *   remains a flat/clamped edge. Empty and out-of-grid cells are flat 0.
 *   Returns 0 when no terrain is loaded.
 */
double terrain_height_at(double wx, double wz);

/* Earliest contact of segment a->b with the loaded terrain surface.
 * Uses the authored 5 m sample-grid triangles (the same v0-v1-v2,
 * v0-v2-v3 diagonal as rendering), not a tunable ray-march pitch.
 * Returns 1 and fills hit_t in [0,1] on contact, 0 otherwise. */
int terrain_segment_hit(const double a[3], const double b[3], double *hit_t);

/* Native ordnance/LOS terrain owner (FUN_004adf90): one-metre samples against
 * the bilinear FUN_004953e0 height field, followed by the decoded bounded
 * crossing refinement. Unlike terrain_segment_hit, renderer triangles do not
 * participate. The final sample is bounded to b; hit_t remains in [0,1]. */
int terrain_ordnance_segment_hit(const double a[3], const double b[3],
                                 double *hit_t);

/*
 * Native drivable-static registry and chassis probe
 * (docs/specs/re/drivable-structures.md, FUN_00423550/FUN_00423690).
 *
 * scene_load registers world-space upward mesh faces from static classes
 * 11/12/13. Objects are bounded first by their authored geometry circle;
 * each face then uses an XZ point-in-polygon test and its plane height.
 * Registration copies all supplied data and is cleared at either scene or
 * terrain unload. object_add returns a non-negative registry id, -1 on bad
 * input/OOM; face_add returns 0 or -1.
 *
 * terrain_drivable_probe is the chassis query. It selects the highest face
 * whose height is strictly within current_y +/- 3 m; if none is in-band it
 * selects the highest face below current_y. With no object hit it returns the
 * unchanged terrain_height_at result and a finite-difference terrain normal.
 * Return value 1 means an object face won, 0 means terrain fallback, -1 means
 * bad output arguments. This function is intentionally separate from
 * terrain_height_at: native per-wheel probes remain heightfield-only.
 */
int  terrain_drivable_object_add(double cx, double cz, double radius);
int  terrain_drivable_face_add(int object, const double plane[4],
                               const double *xz, int nverts);
void terrain_drivable_clear(void);
int  terrain_drivable_probe(double wx, double current_y, double wz,
                            double *height, double normal[3]);
/* Nearest real top-face height for one encoded registry object token
 * (object id + 1; zero means no drivable owner). Unlike the chassis probe,
 * this clamps XZ to the nearest polygon edge instead of requiring PIP. It is
 * the coarse-OBB handoff seam: callers may clear a box overhang only when the
 * chassis is inside/above the associated mesh top's native mount band.
 * Returns 1 on a face, 0 when the token/object has none, -1 on bad arguments.
 */
int  terrain_drivable_nearest_height(int object_token, double wx, double wz,
                                     double *height);
/* Exact PIP/plane query restricted to one encoded registry object and the
 * strict ±3 m contact band. Unlike the global chassis probe, it has no
 * below-car candidate and returns 0 without terrain fallback. */
int  terrain_drivable_object_probe(int object_token, double wx,
                                   double current_y, double wz,
                                   double *height, double normal[3]);
void terrain_drivable_stats(int *objects, int *faces);

/*
 * Discrete navigation sample used by Nitro's route planner. Coordinates are
 * rounded to the nearest 5 m terrain tile; surface_class receives bits 15..13
 * and blocked receives bit 12. Returns -1 for missing output pointers,
 * unloaded terrain, an out-of-grid coordinate, or an empty/invalid ZMAP cell.
 * No road/off-road names are implied by the numeric surface class.
 */
int terrain_nav_sample(double wx, double wz,
                       unsigned *surface_class, int *blocked);

/*
 * Mission surface property table (docs/specs/m4/ghidra-physics.md §Q15,
 * CONFIRMED): the original's WRLD chunk handler (FUN_004b45f0) copies 8
 * records of 0x14 bytes verbatim from the mission WRLD chunk at tag+0x9f
 * into its runtime table @0x5fb500; the terrain tile word's top 3 bits
 * (sample >> 13, see terrain_nav_sample) are the identity index 0..7.
 * The table is parsed at terrain_load and dies with terrain_unload.
 *
 * terrain_surface_props(surface_class, out) fills *out with the three
 * CONFIRMED fields of one record:
 *   grip   — +0x00 float wheel-force grip scale (FUN_00497750)
 *   rr     — +0x04 float rolling resistance    (FUN_00497760)
 *   impact — +0x10 int32 landing impact/damage scale (FUN_00497780)
 * and returns 0. Returns -1 when out is NULL, surface_class > 7, no
 * terrain is loaded, or the mission's WRLD chunk was absent or too short
 * to hold the complete table (payload +0x97 + 8*0x14 bytes) — callers
 * keep their fallback profile in that case. Record fields +0x08/+0x0c
 * have no confirmed consumer in the original (§Q15 residual UNKNOWNs)
 * and are stored raw but not exposed. No road/off-road names are implied
 * by the numeric surface class.
 */
typedef struct {
    float   grip;
    float   rr;
    int32_t impact;
} TerrainSurfaceProps;

int terrain_surface_props(unsigned surface_class, TerrainSurfaceProps *out);

/* Nonzero when a terrain set is loaded. */
int  terrain_is_loaded(void);

/*
 * terrain_used_bounds(wx0, wz0, wx1, wz1)
 *   Absolute world-metre AABB of the used ZMAP patches (cell bbox *
 *   TERRAIN_PATCH_SIZE_M). Returns 0 and fills all four outs when terrain
 *   is loaded with at least one used cell; -1 otherwise. Used by the
 *   paper map pin to place the player on the authored route art.
 */
int  terrain_used_bounds(double *wx0, double *wz0,
                         double *wx1, double *wz1);

/*
 * Road ribbon introspection (RDEF/RSEG, scene.md §5). Read-only, for
 * probes and verification: terrain_road_pieces() is the accepted piece
 * total across segments; terrain_road_point() copies one conformed piece's
 * left/right endpoints (6 doubles: lx ly lz rx ry rz, Y already conformed
 * to native bilinear terrain height with no extra lift) and returns 0, or -1
 * when out of range.
 */
long terrain_road_pieces(void);
int  terrain_road_point(int seg, unsigned piece, double lr[6]);
/* Signed XZ clearance from the nearest authored RSEG ribbon. Negative is
 * inside its interpolated left/right edges; positive is metres outside.
 * Optionally returns the nearest segment and centreline point/tangent. */
double terrain_road_nearest(double wx, double wz, int *segment,
                            double nearest_xz[2], double tangent_xz[2]);

/*
 * terrain_lod_mesh_export — shared draw-stream terrain (M6 resume Phase A/C).
 *
 * Builds the SAME three-band LOD heightfield the software filled path draws
 * (terrain_render_filled / fill_band), as a world-space triangle list for the
 * WebGPU backend. Vertex layout: pos3, nrm3, uv2 (8 floats). UV uses the
 * software surface tiling: u = wx * (10/w_tex), v = wz * (10/h_tex) when a
 * surface tile is loaded; otherwise UV is zero.
 *
 * Out-parameters:
 *   *out_verts   — malloc'd float buffer (caller frees), or NULL on empty
 *   *out_nverts  — vertex count (triangles * 3)
 *   *out_bands   — optional [3] filled with quad-candidate counts per band
 *                  (near/mid/far); may be NULL
 *
 * Returns 0 on success (including zero verts when no terrain), -1 on OOM
 * or invalid camera. On -1 every out-parameter stays NULL/0 — partial
 * bands are never published. zfar_m <= 0 uses 3000 m (raster default).
 */
int terrain_lod_mesh_export(const CameraView *camera, double zfar_m,
                            float **out_verts, int *out_nverts,
                            int out_bands[3]);

/* Surface tile size for GPU UV period (0 if no tile). */
int terrain_surf_tex_size(int *w, int *h);

/* Lighting constants shared with the software path (placeholders, same values). */
void terrain_light_params(float sun_dir[3], float *ambient);

/*
 * terrain_roads_mesh_export — road ribbons as world-space triangle lists
 * matching fill_roads (same quad winding, UV, far cull). Vertex layout
 * pos3 nrm3 uv2 (8 floats), same as LOD terrain.
 *
 * out_by_type[3] = vertex counts per type (0 paved, 1 dirt, 2 riverbed).
 * *out_verts is one buffer: type0 verts, then type1, then type2.
 * Caller frees *out_verts. eye used for radial far cull (like fill_roads).
 * Returns -1 on OOM or bad args with every out-parameter left NULL/0 —
 * partial type buckets are never published.
 */
int terrain_roads_mesh_export(const double eye[3], double far_m,
                              float **out_verts, int *out_nverts,
                              int out_by_type[3]);

/*
 * Palette-index road tile (software RTex). type 0..2. Returns 1 if ok.
 * *texels owned by terrain module (valid until unload).
 */
int terrain_road_tex(int type, const uint8_t **texels, int *w, int *h);

/* Number of road segments currently loaded. */
int terrain_road_seg_count(void);

/* Last filled road-pass pixel accounting. `rescued` proves native painter
 * order resolved a fragment that strict depth rejected; `occluded` proves a
 * nearer terrain owner still hid a road fragment. */
void terrain_road_render_stats(long *written, long *rescued, long *occluded);

/* Gate-only temporal ownership readback. Disabled by default. Once enabled,
 * the pointers address the most recent 640x480 (or target-sized) filled road
 * pass: projected road view-Z before ownership and accepted-road mask. */
void terrain_road_diagnostics_enable(int enabled);
const float *terrain_road_footprint_z_ptr(void);
const uint8_t *terrain_road_owner_ptr(void);

#endif /* TERRAIN_H */
