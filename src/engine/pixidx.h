#ifndef PIXIDX_H
#define PIXIDX_H

/*
 * pixidx.h — merged .pix -> .pak virtual-file index (M8 V2).
 *
 * Interstate '76 stores small assets in <base><tier>.pak containers described
 * by a text <base><tier>.pix manifest ("<count>", then "NAME off len" lines).
 * scene.c already merges the 'g' (geometry) tier; this module generalises the
 * mechanism so the 'm' (texture) tier can be indexed the same way.
 *
 * Measured for the m tier (tools/tex_verify.c): 483 manifests, 4824 entries.
 *
 * THE SORT IS A TOTAL ORDER -- (key, pak, off), then adjacent-duplicate
 * dedupe keeping the first. This is not fastidiousness: 11 keys genuinely
 * resolve to DIFFERENT paks (e.g. 'gtutf_.vqm' in both gturretm.pak and
 * weapncmm.pak). qsort is not stable, and native and wasm link different
 * qsort implementations, so keying on the name alone would let the winner
 * depend on which libc built the binary -- and the whole point of this
 * project's frame gate is that it must not.
 *
 * STATUS: declared in step 1 (build wiring); implemented in step 5, gated on
 * the index built twice with reversed insertion order producing byte-identical
 * arrays, and on the duplicate-key winners being pinned identically on native
 * and wasm.
 */

#include <stddef.h>
#include <stdint.h>

typedef struct {
    char     key[16];      /* lowercased entry name, e.g. "a2_1sg_1.vqm" */
    char     pak[16];      /* owning container                          */
    uint32_t off, len;
} PixEnt;

typedef struct PixIndex PixIndex;

/* Build over every manifest whose name ends with `suffix` (e.g. "m.pix"). */
PixIndex     *pixidx_build(const char *suffix);
/* Build from one case-insensitively exact manifest (direct TMT sibling path). */
PixIndex     *pixidx_build_exact(const char *manifest);
void          pixidx_free(PixIndex *ix);
const PixEnt *pixidx_find(const PixIndex *ix, const char *key);
int           pixidx_count(const PixIndex *ix);

/* Entry i in sorted order. For gates and probes: the sorted sequence is the
 * thing that must be identical across targets, so it has to be inspectable. */
const PixEnt *pixidx_at(const PixIndex *ix, int i);

/* "manifests=N entries=N dups=N dropped=N" */
int           pixidx_stats(const PixIndex *ix, char *buf, size_t n);

#endif /* PIXIDX_H */
