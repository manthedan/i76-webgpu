/*
 * geomesh.c — OEG mesh decode (see geomesh.h)
 *
 * Faithful transcription of geo_build_mesh's (FUN_0043aa60) input parse, run on
 * the decompressed OEG image. Extracts positions (vertex array A), per-vertex
 * normals (array B), the per-face vertex-index lists, and the full per-face
 * material block — colour, plane, mode flags, texture name — plus per-face-
 * vertex UVs and normal indices.
 *
 * Field layout: docs/specs/m6/oeg-face-format.md (CONFIRMED — the whole
 * 55-byte face header and 16-byte vertex entry are accounted for, validated
 * against 9019 records / 114451 faces / 408906 face-vertices and against the
 * disassembly of geo_build_mesh and its consumers).
 */

#include "geomesh.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OEG_MAGIC 0x2e47454fu

/* Sanity caps — guard against a bad/misaligned image producing wild counts. */
#define MAX_VERTS      100000
#define MAX_FACES      100000
#define MAX_FACE_VERTS 64

static uint32_t rd_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static float    rd_f32(const uint8_t *p) { float    v; memcpy(&v, p, 4); return v; }

/* Header field offsets (relative to the OEG magic at byte 0). */
#define OFF_NAME    0x08
#define OFF_NVERTS  0x18
#define OFF_NFACES  0x1c
#define OFF_VERTS   0x24
/* Per-face record: 0x37-byte header, then nFaceVerts × 0x10-byte entries. */
#define FACE_HDR    0x37
#define FACE_VTX    0x10
/* Face header interior (offsets from the face record start). */
#define FOFF_NVERTS 0x04
#define FOFF_RGB    0x08    /* u8 R, G, B                                     */
#define FOFF_PLANE  0x0b    /* f32 nx, ny, nz, d — unaligned, hence memcpy    */
#define FOFF_FLAGS1 0x1f
#define FOFF_FLAGS2 0x20
#define FOFF_FLAGS3 0x21
#define FOFF_TEX    0x22    /* char[13], NUL-terminated                       */
/* Vertex entry interior. */
#define VOFF_POS    0x00
#define VOFF_NRM    0x04
#define VOFF_U      0x08
#define VOFF_V      0x0c

GeoMesh *geomesh_decode(const void *image, size_t size)
{
    const uint8_t *b = (const uint8_t *)image;

    if (size < OFF_VERTS)        return NULL;
    if (rd_u32(b) != OEG_MAGIC)  return NULL;

    int nV = (int)rd_u32(b + OFF_NVERTS);
    int nF = (int)rd_u32(b + OFF_NFACES);
    if (nV < 0 || nV > MAX_VERTS) return NULL;
    if (nF < 0 || nF > MAX_FACES) return NULL;

    /* Faces begin right after the two vertex arrays: (nVerts*6 + 9) dwords.
     * Array A (positions) is at OFF_VERTS, array B (normals) directly after. */
    size_t faces_off = (size_t)(nV * 6 + 9) * 4;
    if (faces_off > size) return NULL;
    size_t norms_off = (size_t)OFF_VERTS + (size_t)nV * 12;

    /* First pass: validate the face table fits and count total indices. */
    size_t p = faces_off;
    long   total_idx = 0;
    for (int f = 0; f < nF; f++) {
        if (p + FACE_HDR > size) return NULL;
        int nfv = (int)rd_u32(b + p + 4);
        if (nfv < 0 || nfv > MAX_FACE_VERTS) return NULL;
        size_t rec = (size_t)FACE_HDR + (size_t)nfv * FACE_VTX;
        if (p + rec > size) return NULL;
        for (int k = 0; k < nfv; k++) {
            const uint8_t *e = b + p + FACE_HDR + (size_t)k * FACE_VTX;
            int idx = (int)rd_u32(e + VOFF_POS);
            int nidx = (int)rd_u32(e + VOFF_NRM);
            if (idx < 0 || idx >= nV) return NULL;
            /* Normal indices address array B, which is parallel to A. They are
             * equal to the position index in every shipped asset, but the
             * format allows them to differ — validate, do not assume. */
            if (nidx < 0 || nidx >= nV) return NULL;
        }
        total_idx += nfv;
        p += rec;
    }

    GeoMesh *m = calloc(1, sizeof(*m));
    if (!m) return NULL;

    memcpy(m->name, b + OFF_NAME, 16);
    m->name[16]     = '\0';
    m->num_verts    = nV;
    m->num_faces    = nF;
    m->num_indices  = (int)total_idx;

    size_t nv1 = nV ? (size_t)nV : 1;
    size_t nf1 = nF ? (size_t)nF : 1;
    size_t ni1 = total_idx ? (size_t)total_idx : 1;

    m->verts      = malloc(sizeof(float) * 3 * nv1);
    m->face_first = malloc(sizeof(int) * nf1);
    m->face_count = malloc(sizeof(int) * nf1);
    m->indices    = malloc(sizeof(int) * ni1);
    m->normals        = malloc(sizeof(float) * 3 * nv1);
    m->face_rgb       = malloc(3 * nf1);
    m->face_plane     = malloc(sizeof(float) * 4 * nf1);
    m->face_flags     = malloc(3 * nf1);
    m->face_tex       = malloc(GEO_TEX_NAME_LEN * nf1);
    m->uvs            = malloc(sizeof(float) * 2 * ni1);
    m->normal_indices = malloc(sizeof(int) * ni1);
    if (!m->verts || !m->face_first || !m->face_count || !m->indices ||
        !m->normals || !m->face_rgb || !m->face_plane || !m->face_flags ||
        !m->face_tex || !m->uvs || !m->normal_indices) {
        geomesh_free(m);
        return NULL;
    }

    /* Positions (vertex array A) — 3 floats each, at byte 0x24. */
    for (int i = 0; i < nV; i++) {
        const uint8_t *v = b + OFF_VERTS + (size_t)i * 12;
        m->verts[i * 3 + 0] = rd_f32(v + 0);
        m->verts[i * 3 + 1] = rd_f32(v + 4);
        m->verts[i * 3 + 2] = rd_f32(v + 8);
    }

    /* Per-vertex normals (vertex array B) — mean of incident face normals,
     * NOT unit length. Stored verbatim; consumers normalise. */
    for (int i = 0; i < nV; i++) {
        const uint8_t *v = b + norms_off + (size_t)i * 12;
        m->normals[i * 3 + 0] = rd_f32(v + 0);
        m->normals[i * 3 + 1] = rd_f32(v + 4);
        m->normals[i * 3 + 2] = rd_f32(v + 8);
    }

    /* Faces — vertex-index lists plus the material block and per-corner UVs. */
    p = faces_off;
    int oi = 0;
    for (int f = 0; f < nF; f++) {
        int nfv = (int)rd_u32(b + p + FOFF_NVERTS);
        m->face_first[f] = oi;
        m->face_count[f] = nfv;

        m->face_rgb[f * 3 + 0] = b[p + FOFF_RGB + 0];
        m->face_rgb[f * 3 + 1] = b[p + FOFF_RGB + 1];
        m->face_rgb[f * 3 + 2] = b[p + FOFF_RGB + 2];
        for (int c = 0; c < 4; c++)
            m->face_plane[f * 4 + c] = rd_f32(b + p + FOFF_PLANE + (size_t)c * 4);
        m->face_flags[f * 3 + 0] = b[p + FOFF_FLAGS1];
        m->face_flags[f * 3 + 1] = b[p + FOFF_FLAGS2];
        m->face_flags[f * 3 + 2] = b[p + FOFF_FLAGS3];
        /* The name field is 13 bytes and need not be NUL-terminated when a
         * name fills it exactly; terminate defensively. */
        memcpy(m->face_tex + (size_t)f * GEO_TEX_NAME_LEN,
               b + p + FOFF_TEX, GEO_TEX_NAME_LEN);
        m->face_tex[(size_t)f * GEO_TEX_NAME_LEN + GEO_TEX_NAME_LEN - 1] = '\0';

        for (int k = 0; k < nfv; k++) {
            const uint8_t *e = b + p + FACE_HDR + (size_t)k * FACE_VTX;
            m->normal_indices[oi] = (int)rd_u32(e + VOFF_NRM);
            m->uvs[oi * 2 + 0]    = rd_f32(e + VOFF_U);
            m->uvs[oi * 2 + 1]    = rd_f32(e + VOFF_V);
            m->indices[oi++]      = (int)rd_u32(e + VOFF_POS);
        }
        p += (size_t)FACE_HDR + (size_t)nfv * FACE_VTX;
    }

    /* Bounding box. */
    if (nV > 0) {
        for (int c = 0; c < 3; c++) m->bb_min[c] = m->bb_max[c] = m->verts[c];
        for (int i = 1; i < nV; i++)
            for (int c = 0; c < 3; c++) {
                float val = m->verts[i * 3 + c];
                if (val < m->bb_min[c]) m->bb_min[c] = val;
                if (val > m->bb_max[c]) m->bb_max[c] = val;
            }
    }
    return m;
}

void geomesh_free(GeoMesh *m)
{
    if (!m) return;
    free(m->verts);
    free(m->face_first);
    free(m->face_count);
    free(m->indices);
    free(m->normals);
    free(m->face_rgb);
    free(m->face_plane);
    free(m->face_flags);
    free(m->face_tex);
    free(m->uvs);
    free(m->normal_indices);
    free(m);
}
