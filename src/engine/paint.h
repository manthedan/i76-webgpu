#ifndef PAINT_H
#define PAINT_H

#include <stddef.h>

/*
 * paint.h — vehicle paint-scheme texture resolution (the GEO 'V...'
 * placeholder chain). Spec: docs/specs/m6/oeg-face-format.md
 * "Texture resolution chain".
 *
 * A vehicle GEO face names a PLACEHOLDER like "V1 FT LF.MAP" rather than a
 * real tile — which pixels it gets depends on the car's paint scheme:
 *
 *   1. "V1 FT LF.MAP" -> strip the leading V, drop spaces, LF->LT, ".TMT"
 *      => "1FTLT.TMT" (the key)
 *   2. the car's .vtf (BWD2, VTFC chunk: char[13] vdf, char[16] scheme,
 *      char[13] tmt[78], char[13] maps[13]) holds entries like
 *      "2dr1FTLT.TMT"; entry+3 is exactly the key. "NULL" = absent panel.
 *   3. that type-1 .TMT (64-byte header, u32 name count at +0x18,
 *      then count x char[8] base names, one per damage state) gives the base.
 *   4. base -> "<base>.m16" for the hardware path.
 *
 * Layouts verified against real bytes: rampage2.vtf (25 non-NULL entries,
 * scheme "blue w/yellow st") and blade101.tmt (3 states BLADE101/102/103).
 *
 * Lives in the engine rather than the web driver so the same code is
 * exercised by tools/paint_probe.c natively (including under
 * -fsanitize=address) and by the wasm build.
 *
 * Reads through the engine VFS: call fs_set_root() + vfs_init() first.
 */

/*
 * paint_face_key(face_name, out32)
 *   Placeholder name -> TMT key. Returns 0 and fills `out` (at least 32
 *   bytes), -1 when `face_name` is not a 'V' placeholder or is unusable.
 */
int paint_face_key(const char *face_name, char *out, size_t out_sz);

/*
 * paint_resolve_face(face_name, vtf_file, base_out, base_sz)
 *   Full chain. `vtf_file` is the car's paint file (car_vtf_file()).
 *   Returns 0 and fills `base_out` with the damage-state-0 base texture
 *   name, -1 when the face is not a placeholder or any link misses.
 *   base_out should be >= 16 bytes.
 */
int paint_resolve_face(const char *face_name, const char *vtf_file,
                       char *base_out, size_t base_sz);

#endif /* PAINT_H */
