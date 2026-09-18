#ifndef TEXCACHE_H
#define TEXCACHE_H

/*
 * texcache.h — face texture name -> decoded tile (M8 V2).
 *
 * The ONLY module that knows "A2_1SG_1.MAP" and "A2_1SG_1.VQM" are the same
 * thing, and the only one that knows a face name may not be a file name at all.
 *
 * Three resolution paths converge on the same indexed RTex contract:
 *
 *   static geometry   "A2_1SG_1.MAP" ────────────────────────→ <stem>.vqm
 *   vehicle panels    "V1 FT LF.MAP" → paint_resolve_face ──→ <stem>.vqm
 *   direct table      "XOS1_101.TMT" → descriptor frame ────→ .vqm or .map
 *
 * Ordinary static/vehicle tiles use the m-tier index. A direct TMT may instead
 * own a sibling PIX/PAK family; its exact package has precedence.
 *
 * IDS, NOT POINTERS. Callers hold uint16_t tile ids. The reason is
 * determinism, not tidiness: a pointer-keyed structure would iterate in
 * address order, and native ASLR varies address order run to run. Ids also let
 * per-instance arrays stay small and let the cache grow without invalidating
 * callers.
 *
 * STATUS: declared in step 1 (build wiring); implemented in step 6, which
 * lands the plumbing while still passing tex=NULL to the rasterizer -- the
 * second and last "prove the hashes have not moved" checkpoint.
 */

#include <stddef.h>
#include <stdint.h>

#include "engine/raster.h"

#define TEX_ID_NONE     0u        /* resolved: no usable texture for this face */
#define TEX_ID_PENDING  0xFFFFu   /* caller-side lazy-fill sentinel; never returned */

/*
 * A tile ready for sampling, and it is LITERALLY the rasterizer's RTex rather
 * than a layout-identical twin of it. Two identical structs would compile, and
 * the cast at the call site would work right up until somebody added a field to
 * one of them; making this a typedef means the fragment stage and the cache
 * cannot drift apart at all. raster.h owns the definition because raster.c is
 * what dereferences it per pixel.
 *
 * The invariants that definition relies on -- power-of-two, <= 256 -- are
 * ENFORCED here, in texcache.c's tile_build: a tile that fails them is rejected
 * at load and never handed out, because raster.c wraps with `u & umask` and has
 * no way to notice.
 */
typedef RTex TexTile;

void     texcache_init(void);
void     texcache_reset(void);      /* from scene_unload; also resets vqm cache */
unsigned texcache_generation(void);

/*
 * face_name is a raw GeoMesh.face_tex entry ("A2_1SG_1.MAP" or "V1 FT LF.MAP").
 * vtf_file is the OWNING VEHICLE's paint scheme, "" for static geometry.
 *
 * Resolution is a pure function of (VFS contents, face_name, vtf_file), so
 * negatives are memoized permanently within a mission. Returns TEX_ID_NONE
 * when the face has no usable texture -- callers draw it flat.
 */
uint16_t texcache_resolve(const char *face_name, const char *vtf_file);

/* NULL for TEX_ID_NONE. Stable until texcache_reset(). */
const TexTile *texcache_tile(uint16_t id);
/* Canonical resolved VQM/raw-MAP key (borrowed cache storage), or "". */
const char *texcache_tile_name(uint16_t id);

/*
 * Direct TMT descriptor support. Some OEG faces name a .TMT rather than a
 * .MAP. The 64-byte descriptor then names one or more indexed VQM/raw-MAP
 * frames; those frames may live in the descriptor's sibling PIX/PAK rather
 * than the ordinary m-tier index. Header field names stay structural because
 * their runtime animation semantics are only partly decoded.
 */
typedef struct {
    uint32_t kind;          /* raw +0x14 */
    uint32_t count;         /* raw +0x18 */
    uint32_t stride;        /* raw +0x1c */
    uint32_t name_count;    /* validated char[8] entries after byte 0x40 */
    float    rate;          /* raw +0x28 */
    uint32_t mode;          /* raw +0x2c */
} TexTmtInfo;

/* 0 on a validated descriptor, -1 when missing or malformed. */
int texcache_tmt_info(const char *tmt_name, TexTmtInfo *out);
/* Resolve one exact descriptor name-table entry. Out-of-range frames fail. */
uint16_t texcache_resolve_tmt_frame(const char *tmt_name, uint32_t frame);
/* Resolve one frame from an exact sibling PIX/PAK package with no TMT
 * descriptor (the xwp1/xsg1/xbp1/xcp1 smoke sprite families, D-C27).
 * Fails closed (TEX_ID_NONE) on a missing package or frame. */
uint16_t texcache_resolve_family_frame(const char *pix_manifest,
                                       const char *frame_name);

/*
 * Raw indexed .map loader for WDEF-named world assets (the mission sky at
 * WRLD +82; scene.md §2) — NOT the GEO-face rule above: this loads the
 * stored bitmap itself (u32 w, u32 h, then w*h level-palette indices),
 * resolving m-tier .pix/.pak first, then a loose VFS file. Non-POT or
 * out-of-range dimensions are REJECTED (callers wrap with `u & umask`).
 * Returns 0 on success; *pixels is then caller-owned and equal to
 * out->texels. -1 on any failure, *pixels NULL.
 */
int texcache_load_map(const char *name, RTex *out, uint8_t **pixels);

/* Human-readable cache/resolution counters. */
int texcache_stats(char *buf, size_t n);

#endif /* TEXCACHE_H */
