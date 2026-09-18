/*
 * lzodec.h — engine-local LZO1X/LZO1Y block decompressor.
 *
 * Clean-room replacement for the GPL liblzo2 dependency previously linked
 * into every shipped decompression path. No LZO implementation text was
 * consulted or copied. Sources of format knowledge:
 *
 *   - LZO1X: the public byte-level description of the stream format in the
 *     Linux kernel's staging document "lzo.txt" (a reverse-engineered
 *     format description, not code).
 *   - LZO1Y: a distinct variant, NOT the same stream as LZO1X. Its two
 *     differences were derived from the algorithm's published configuration
 *     (M2 max length 14, M2 max distance 1024) and from behavioural probing
 *     of the compressor's public API, then confirmed byte-identical against
 *     the reference decompressor over every LZO1Y entry in nitro.zfs
 *     (4,219 entries) and over generated round-trip corpora:
 *       1. t >= 64: one unified match op, len = (t>>4)-1 (3..14),
 *          dist = 1 + ((t>>2)&3) + (B<<2) (<= 1024)
 *          [LZO1X: len 3..8, dist = 1 + ((t>>2)&7) + (B<<3) (<= 2048)]
 *       2. t < 16 after a >=4 literal run: len 3,
 *          dist = 1025 + (t>>2) + (B<<2)
 *          [LZO1X: 2049 + ...]
 *
 *   - Only the classic (version 0) format is decoded. The later kernel-only
 *     RLE variant (first byte 17 used as a version tag) is not produced by
 *     any compressor relevant to the 1998 game data and is not implemented;
 *     such input fails closed like any other malformed stream.
 *
 * Provenance: format semantics as above; this code is project-original and
 * falls under the project MIT licence.
 */
#ifndef LZODEC_H
#define LZODEC_H

#include <stddef.h>

/* Return codes. Any non-OK code means the output buffer contents are
 * unspecified and MUST NOT be consumed by the caller (fail-closed). */
#define LZODEC_OK                 0
#define LZODEC_INPUT_OVERRUN     (-1)  /* read would pass end of src         */
#define LZODEC_OUTPUT_OVERRUN    (-2)  /* write would pass dst capacity      */
#define LZODEC_LOOKBEHIND        (-3)  /* match distance reaches before dst  */
#define LZODEC_INPUT_NOT_CONSUMED (-4) /* end-of-stream with trailing input  */

/* Stream variants. */
#define LZODEC_LZO1X 0
#define LZODEC_LZO1Y 1

/*
 * Decompress one LZO1X- or LZO1Y-format block.
 *
 * src/src_len : compressed block.
 * dst         : caller-provided output buffer; *dst_len on entry is its
 *               capacity, on success the produced byte count.
 * format      : LZODEC_LZO1X or LZODEC_LZO1Y.
 *
 * Returns LZODEC_OK only when the stream's end-of-stream marker is reached
 * with every input byte consumed and no bound ever exceeded. Overlapping
 * match copies (distance < length, i.e. RLE replication) follow the format's
 * forward byte-wise semantics.
 *
 * Allocates nothing.
 */
int lzodec_decompress(const unsigned char *src, size_t src_len,
                      unsigned char *dst, size_t *dst_len, int format);

#endif /* LZODEC_H */
