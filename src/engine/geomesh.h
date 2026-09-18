#ifndef GEOMESH_H
#define GEOMESH_H

/*
 * geomesh.h — Decoded OEG mesh
 *
 * The renderable form of an "OEG." geometry record, produced by geomesh_decode
 * from the (already decompressed) raw image that geo_build_mesh (FUN_0043aa60)
 * consumes in the original. See docs/REVERSING.md "GEO/OEG mesh file format —
 * CONFIRMED & DECODED" for the byte layout. The decode here mirrors the
 * original's pointer arithmetic faithfully:
 *
 *   0x00 u32  magic "OEG." (0x2e47454f)
 *   0x04 u32  count
 *   0x08 char name[16]
 *   0x18 u32  nVerts          (= param_1[6])
 *   0x1c u32  nFaces          (= param_1[7])
 *   0x20 u32  ?
 *   0x24 vec3f posA[nVerts]   (positions — what we render)
 *        vec3f posB[nVerts]   (second vertex set; normals?)
 *        faces[nFaces]: 0x37-byte header (+0x04 = nFaceVerts) then
 *                       nFaceVerts × 0x10-byte entries (+0x00 = vertex index)
 *
 * NOTE: a .pak holds several OEG records concatenated. geomesh_decode reads the
 * FIRST record at the start of `image` (for a single .geo that's the whole
 * file; for a g-tier .pak that's the first sub-mesh — typically the hull).
 */

#include <stddef.h>
#include <stdint.h>

/*
 * Per-face material + per-face-vertex attributes are decoded too — the full
 * 55-byte face header and 16-byte vertex entry are documented in
 * docs/specs/m6/oeg-face-format.md (CONFIRMED against 9019 records /
 * 114451 faces, and against geo_build_mesh's disassembly). Without these the
 * hardware renderer has no colours, no textures, no UVs and no normals, and
 * can only draw meshes as flat arbitrary fills.
 *
 * The position/topology fields below are unchanged and remain the identity
 * the tier-2 mesh pins (verify/tier2_meshes.sh) hash.
 */
typedef struct {
    char   name[17];        /* embedded 16-byte name, null-terminated        */
    int    num_verts;
    int    num_faces;
    int    num_indices;     /* total face-vertex references                   */
    float *verts;           /* [num_verts*3]  x,y,z (positions / array A)      */
    int   *face_first;      /* [num_faces]    offset of face f into indices[]  */
    int   *face_count;      /* [num_faces]    vertex count of face f           */
    int   *indices;         /* [num_indices]  vertex indices into verts        */
    float  bb_min[3];
    float  bb_max[3];

    /* Vertex array B: per-vertex normals (the mean of the incident unit face
     * normals). NOT unit length — the original folded the magnitude into its
     * diffuse term — so normalise before shading. */
    float *normals;         /* [num_verts*3]                                  */

    /* Per-face material (face header +0x08/+0x0b/+0x1f/+0x22). */
    uint8_t *face_rgb;      /* [num_faces*3]  24-bit authoring colour          */
    float   *face_plane;    /* [num_faces*4]  unit normal + d (n.v + d == 0)   */
    uint8_t *face_flags;    /* [num_faces*3]  flags1, flags2, flags3           */
    char    *face_tex;      /* [num_faces*13] NUL-terminated name, "" = none   */

    /* Per-face-vertex attributes, parallel to indices[]. */
    float *uvs;             /* [num_indices*2]  U,V (normalised, wraps)        */
    int   *normal_indices;  /* [num_indices]    index into normals[]           */
} GeoMesh;

#define GEO_TEX_NAME_LEN 13   /* face_tex stride (face header +0x22 field)    */

/* face_flags[f*3 + 1] (flags2) == 0 iff the face carries no texture name.
 * Values 5 and 7 mark cut-out/see-through surfaces (headlights, fences,
 * rails); 1 and 3 are opaque. See the spec for the evidence. */
#define GEO_FLAG2_CUTOUT(f2)  ((f2) == 5u || (f2) == 7u)

/*
 * Decode the first OEG record in `image` (decompressed bytes, `size` long).
 * Returns a heap GeoMesh (free with geomesh_free) or NULL on a bad/short image.
 * A valid header with zero verts/faces yields a non-NULL empty mesh.
 */
GeoMesh *geomesh_decode(const void *image, size_t size);

void geomesh_free(GeoMesh *m);

#endif /* GEOMESH_H */
