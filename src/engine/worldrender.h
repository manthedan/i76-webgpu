#ifndef WORLDRENDER_H
#define WORLDRENDER_H

/*
 * worldrender.h — the one place that owns a world frame.
 *
 * Before this existed, "draw terrain, draw the scene into a scratch buffer,
 * composite it over the terrain by colour key" was open-coded in four places:
 * three in web/webmain.c and once more in tools/frame_probe.c, whose header
 * comment had to warn "MUST mirror this sequence exactly -- change one, change
 * both". That warning is a bug waiting to happen, and it matters more now: the
 * filled rasterizer needs terrain and objects to share ONE depth buffer, so
 * every copy of the sequence would have to grow a depth buffer in lockstep.
 *
 * Backends are selectable and recorded in captures, per
 * docs/specs/m8/software-raster.md §7.
 */

#include <stdint.h>
#include "engine/camera.h"

typedef enum {
    WORLD_BACKEND_WIRE = 0,     /* magenta line art; the pre-M8 debug view          */
    WORLD_BACKEND_FILLED,       /* M8 software rasterizer                           */
    WORLD_BACKEND_FILLED_WIRE   /* filled, with the wireframe drawn over it         */
} WorldBackend;

void          worldrender_set_backend(WorldBackend b);
WorldBackend worldrender_backend(void);
const char   *worldrender_backend_name(void);

/*
 * Render terrain + scene from the canonical world-space camera. `near` and
 * `far` <= 0 select the raster defaults. A caller may lower the near plane
 * for authored first-person geometry close to the eye without changing the
 * ordinary world projection. Both backends consume this exact basis; no
 * renderer reconstructs a global-Y look-at and loses scripted roll.
 */
void worldrender_camera(uint8_t *color, int w, int h,
                        const CameraView *camera, double near, double far,
                        int have_terrain, int have_scene);

/* Orbit-camera variant (yaw/distance around the scene centre). */
void worldrender_orbit(uint8_t *color, int w, int h, double yaw, double dist,
                        int have_terrain, int have_scene);

/*
 * Pixels covered by GEOMETRY in the last filled frame, counted from the depth
 * buffer. The colour buffer cannot answer this once the sky fills every pixel
 * — a coverage floor built on "non-background colour" can never fail. Returns
 * 0 for the wireframe backend, which has no depth buffer.
 */
long worldrender_geometry_pixels(void);

/* Accepted textured-fragment conversions matching the bounded H-UAT-063
 * low-saturation-source -> yellow-final detector in the last filled frame. */
long worldrender_grey_to_yellow_pixels(void);

/*
 * Reciprocal-depth coverage of the last successful filled frame. Geometry
 * writes nonzero values; sky remains zero. The pointer is renderer-owned and
 * valid until the next world render or shutdown. NULL means the last frame
 * did not use the filled backend.
 */
const uint32_t *worldrender_depth(void);

void worldrender_shutdown(void);

#endif /* WORLDRENDER_H */
