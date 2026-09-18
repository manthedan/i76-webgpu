/*
 * lzodec.c — clean-room LZO1X/LZO1Y block decompressor. See lzodec.h for
 * provenance, the LZO1X/LZO1Y format differences, and the fail-closed
 * contract.
 *
 * Shared instruction set (distances relative to the current output
 * position; "S" bits are the trailing-literal count 0..3 that follows
 * almost every match):
 *
 *   first byte:
 *     0..17   regular instruction (17 = M4 with len bits 1; "17 0 0" is
 *             also the canonical end-of-stream marker)
 *     18..21  literal run of (byte-17), state = byte-17
 *     22..255 literal run of (byte-17), state = 4
 *
 *   loop (state = literals just copied: 0, 1..3, or 4 for ">=4"):
 *     t < 16, state == 0  : long literal run, len = 3 + (t ?: 15+ext)
 *     t < 16, state 1..3  : len 2, dist = 1 + (t>>2) + (B<<2)
 *     t < 16, state == 4  : len 3, dist = base + (t>>2) + (B<<2)
 *                           base = 2049 (1X) / 1025 (1Y)
 *     16..31              : len = 2 + (L ?: 7+ext), L = t&7
 *                           dist = 16384 + ((t&8)<<11) + (LE16>>2)
 *                           dist == 16384 -> end of stream
 *     32..63              : len = 2 + (L ?: 31+ext), L = t&31
 *                           dist = 1 + (LE16>>2)
 *     t >= 64, 1X         : len = 3 + ((t>>5)&1) (t<128) / 5 + ((t>>5)&3)
 *                           dist = 1 + ((t>>2)&7) + (B<<3)
 *     t >= 64, 1Y         : len = (t>>4) - 1     (3..14)
 *                           dist = 1 + ((t>>2)&3) + (B<<2)   (<= 1024)
 *
 *   ext (variable length): while next byte == 0, add 255; add final byte.
 *   After every match, S literals are copied and become the new state.
 */

#include "lzodec.h"

#include <string.h>

/* Length-extension bytes: each zero byte contributes 255, terminated by a
 * non-zero byte that is added as-is. Bounded by src_len, so no overflow. */
static int read_ext(const unsigned char **ipp, const unsigned char *in_end,
                    size_t *len)
{
    for (;;) {
        unsigned b;
        if (*ipp >= in_end)
            return LZODEC_INPUT_OVERRUN;
        b = **ipp;
        (*ipp)++;
        if (b != 0) {
            *len += b;
            return LZODEC_OK;
        }
        *len += 255;
    }
}

static int copy_literals(unsigned char **opp, unsigned char *out_end,
                         const unsigned char **ipp, const unsigned char *in_end,
                         size_t n)
{
    if (n > (size_t)(in_end - *ipp))
        return LZODEC_INPUT_OVERRUN;
    if (n > (size_t)(out_end - *opp))
        return LZODEC_OUTPUT_OVERRUN;
    memcpy(*opp, *ipp, n);
    *opp += n;
    *ipp += n;
    return LZODEC_OK;
}

/* Copy `n` bytes from `dist` back in the output. Forward byte order gives
 * the required overlap (RLE) semantics when dist < n. */
static int copy_match(unsigned char **opp, unsigned char *out_start,
                      unsigned char *out_end, size_t dist, size_t n)
{
    unsigned char *op = *opp;
    const unsigned char *from;

    if (dist == 0 || dist > (size_t)(op - out_start))
        return LZODEC_LOOKBEHIND;
    if (n > (size_t)(out_end - op))
        return LZODEC_OUTPUT_OVERRUN;

    from = op - dist;
    if (dist >= n) {
        memcpy(op, from, n);
        op += n;
    } else {
        while (n--)
            *op++ = *from++;
    }
    *opp = op;
    return LZODEC_OK;
}

static int read_le16(const unsigned char **ipp, const unsigned char *in_end,
                     unsigned *v)
{
    const unsigned char *ip = *ipp;
    if ((size_t)(in_end - ip) < 2)
        return LZODEC_INPUT_OVERRUN;
    *v = (unsigned)ip[0] | ((unsigned)ip[1] << 8);
    *ipp = ip + 2;
    return LZODEC_OK;
}

int lzodec_decompress(const unsigned char *src, size_t src_len,
                      unsigned char *dst, size_t *dst_len, int format)
{
    const int base_after_run = (format == LZODEC_LZO1Y) ? 1024 : 2048;
    const unsigned char *ip = src;
    const unsigned char *const in_end = src + src_len;
    unsigned char *op = dst;
    unsigned char *const out_start = dst;
    unsigned char *const out_end = dst + *dst_len;
    unsigned state = 0;   /* 0, 1..3, or 4 (meaning ">=4 literals copied") */
    unsigned t;
    int rc;

    if (ip >= in_end)
        return LZODEC_INPUT_OVERRUN;

    t = *ip++;
    if (t > 17) {
        /* First-byte literal run; state = run length clamped to 4. */
        size_t n = t - 17;
        rc = copy_literals(&op, out_end, &ip, in_end, n);
        if (rc != LZODEC_OK)
            return rc;
        state = n < 4 ? (unsigned)n : 4;
        if (ip >= in_end)
            return LZODEC_INPUT_OVERRUN;   /* stream must end with EOS marker */
        t = *ip++;
    }
    /* t <= 17 falls through: it is the first regular instruction. */

    for (;;) {
        size_t len, dist;
        unsigned s, le16;

        if (t < 16) {
            if (state == 0) {
                /* Long literal run. */
                len = t;
                if (len == 0) {
                    len = 15;
                    rc = read_ext(&ip, in_end, &len);
                    if (rc != LZODEC_OK)
                        return rc;
                }
                len += 3;
                rc = copy_literals(&op, out_end, &ip, in_end, len);
                if (rc != LZODEC_OK)
                    return rc;
                state = 4;
                if (ip >= in_end)
                    return LZODEC_INPUT_OVERRUN;
                t = *ip++;
                continue;
            }
            /* t < 16 after literals: short match, B follows. */
            if (ip >= in_end)
                return LZODEC_INPUT_OVERRUN;
            dist = 1 + (t >> 2) + ((unsigned)*ip++ << 2);
            if (state != 4) {
                len = 2;
            } else {
                len = 3;
                dist += (size_t)base_after_run;
            }
            s = t & 3;
        } else if (t < 32) {
            /* M4: 16..48 kB lookback; also carries the EOS marker. */
            len = t & 7;
            if (len == 0) {
                len = 7;
                rc = read_ext(&ip, in_end, &len);
                if (rc != LZODEC_OK)
                    return rc;
            }
            len += 2;
            rc = read_le16(&ip, in_end, &le16);
            if (rc != LZODEC_OK)
                return rc;
            dist = ((size_t)(t & 8) << 11) + (le16 >> 2);
            if (dist == 0) {
                /* distance would be exactly 16384: end of stream. */
                if (ip != in_end)
                    return LZODEC_INPUT_NOT_CONSUMED;
                *dst_len = (size_t)(op - out_start);
                return LZODEC_OK;
            }
            dist += 16384;
            s = le16 & 3;
        } else if (t < 64) {
            /* M3: up to 16 kB lookback. */
            len = t & 31;
            if (len == 0) {
                len = 31;
                rc = read_ext(&ip, in_end, &len);
                if (rc != LZODEC_OK)
                    return rc;
            }
            len += 2;
            rc = read_le16(&ip, in_end, &le16);
            if (rc != LZODEC_OK)
                return rc;
            dist = 1 + (le16 >> 2);
            s = le16 & 3;
        } else {
            if (ip >= in_end)
                return LZODEC_INPUT_OVERRUN;
            if (format == LZODEC_LZO1Y) {
                /* 1Y: one unified op, len 3..14, dist <= 1024. */
                len = (size_t)(t >> 4) - 1;
                dist = 1 + ((t >> 2) & 3) + ((unsigned)*ip++ << 2);
            } else {
                /* 1X: M2 (64..127) / M1 (128..255), dist <= 2048. */
                dist = 1 + ((t >> 2) & 7) + ((unsigned)*ip++ << 3);
                len = t < 128 ? 3u + ((t >> 5) & 1) : 5u + ((t >> 5) & 3);
            }
            s = t & 3;
        }

        rc = copy_match(&op, out_start, out_end, dist, len);
        if (rc != LZODEC_OK)
            return rc;

        /* Trailing literals of this instruction; they set the next state. */
        state = s;
        if (s) {
            rc = copy_literals(&op, out_end, &ip, in_end, s);
            if (rc != LZODEC_OK)
                return rc;
        }

        if (ip >= in_end)
            return LZODEC_INPUT_OVERRUN;
        t = *ip++;
    }
}
