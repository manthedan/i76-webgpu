#ifndef MESHVIEW_H
#define MESHVIEW_H

/*
 * meshview.h — In-engine OEG mesh viewer
 *
 * A self-contained inner loop (same shape as gameplay_run) that loads meshes
 * through the real engine path — geo_cache_acquire -> vfs_read_file ->
 * geomesh_decode — and software-renders them as a rotating wireframe into the
 * 8-bit framebuffer, blitted via the normal Vulkan palette path.
 *
 * Controls: LEFT/RIGHT cycle assets, UP/DOWN zoom, SPACE toggles auto-spin,
 *           ESC / window-close to quit.
 *
 * Entered from main.c when the binary is run with --meshview. Requires
 * render_init() + meshcache_init() to have run first.
 */

void meshview_run(void);

/* -----------------------------------------------------------------------
 * Shared pure rasterizer
 *
 * build_palette() + render_mesh() are platform-independent (8-bit indexed
 * framebuffer out, no OS calls). The SDL/Vulkan driver above and the
 * browser driver (web/webmain.c, emscripten) both use them — do not
 * duplicate this logic in the web shell.
 * ----------------------------------------------------------------------- */
#include <stdint.h>
#include "engine/geomesh.h"
#include "render/render.h"

#define MESHVIEW_FB_W 640
#define MESHVIEW_FB_H 480

void build_palette(Rgb8 pal[256]);
void render_mesh(uint8_t *fb, const GeoMesh *m, double yaw, double zoom);

#endif /* MESHVIEW_H */
