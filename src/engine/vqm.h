#ifndef VQM_H
#define VQM_H

/*
 * vqm.h — .vqm tile decode (M8 V2).
 *
 * VQM is Interstate '76's vector-quantized image format: a stream of 4x4
 * blocks, each either a solid fill or an index into a shared .cbk codebook.
 * It is what BOTH the cockpit dashboard and the world textures are stored as,
 * which is why this lives here rather than inside hud.c.
 *
 * THE IMPORTANT PROPERTY: decoded texels are indices into the mission's LEVEL
 * palette -- the same palette raster.c builds its shade ramps from. There is no
 * colour conversion anywhere downstream. A texel IS a palette index, so the
 * fragment stage is shade[level][texel] and no RGB cube or 64 KiB LUT is
 * needed (docs/specs/m8/texture-pipeline.md).
 *
 * Container: tiles live inside <base>m.pak, indexed by <base>m.pix. A face
 * names its texture "FOO.MAP"; the stored asset is "FOO.VQM". See pixidx.h.
 *
 * Measured over nitro.zfs (tools/tex_verify.c): 4824 tiles, ALL power-of-two,
 * max 256x256, 265 of them 100% solid-fill (they reference no codebook at
 * all), served by 201 distinct codebooks.
 *
 * Reads codebooks through the engine VFS: fs_set_root() + vfs_init() first.
 *
 * STATUS: declared in step 1 (build wiring). The decoder itself moves here
 * from hud.c in step 2 as a pure refactor, gated on all 4824 tiles decoding
 * byte-identically and both hud_probe PGMs being unmoved.
 */

#include <stddef.h>
#include <stdint.h>

/*
 * vqm_decode(buf, n, transpose, ow, oh)
 *   Decode one tile. Returns a malloc'd w*h buffer of LEVEL-palette indices
 *   (row-major, row 0 = top), or NULL. Caller frees with free().
 *
 *   `transpose` selects the column-major block walk. It exists ONLY as
 *   evidence for the P3 orientation decision (hud.h HUD_ORIENT_COLUMN) and is
 *   0 for every real decode; it is a parameter rather than a read of hud.c's
 *   static state so this module has no dependency on the HUD.
 *
 *   `*ow` and `*oh` are set to 0 on ENTRY, not merely on success. hud.c's
 *   left them untouched on all five failure returns, which is a trap for any
 *   new caller that does not pre-initialise.
 */
uint8_t *vqm_decode(const uint8_t *buf, size_t n, int transpose,
                    int *ow, int *oh);

/*
 * Codebook cache. Decode is a pure function of (tile bytes, codebook bytes),
 * so caching cannot change output -- a miss costs a deterministic reload of
 * identical data. That is what makes cache SIZING a performance decision and
 * never a framebuffer decision, and it is why nothing here is ever keyed by
 * pointer value.
 */
void vqm_cache_reset(void);

/* "cbk_loaded=N cbk_hits=N cbk_misses=N solid_only=N absent=N" */
int  vqm_stats(char *buf, size_t n);

#endif /* VQM_H */
