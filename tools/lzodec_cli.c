/*
 * lzodec_cli.c — minimal CLI over src/engine/lzodec.c for the focused
 * decompression test (tools/test_lzodec.py).
 *
 *   lzodec_cli <x|y> <in-file> <out-file>
 *
 * Decodes <in-file> as LZO1X (x) or LZO1Y (y) into <out-file>. The output
 * capacity is taken from the decoder itself: the file is decoded with a
 * buffer of LZODEC_CLI_CAP bytes; on success the produced byte count is
 * written. Exit status is the lzodec return code (0 == LZODEC_OK), so
 * malformed input fails closed with a nonzero exit and no output file.
 *
 * Build: see test_lzodec.py (also built with -fsanitize=address there for
 * the corruption sweep).
 */

#include <stdio.h>
#include <stdlib.h>
#include "engine/lzodec.h"

#define LZODEC_CLI_CAP (64u * 1024u * 1024u)

int main(int argc, char **argv)
{
    if (argc != 4 || (argv[1][0] != 'x' && argv[1][0] != 'y') || argv[1][1]) {
        fprintf(stderr, "usage: lzodec_cli <x|y> <in> <out>\n");
        return 2;
    }
    int format = argv[1][0] == 'y' ? LZODEC_LZO1Y : LZODEC_LZO1X;

    FILE *f = fopen(argv[2], "rb");
    if (!f) { perror(argv[2]); return 2; }
    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (n < 0) { fclose(f); return 2; }

    unsigned char *in = malloc((size_t)n ? (size_t)n : 1);
    unsigned char *out = malloc(LZODEC_CLI_CAP);
    if (!in || !out) { fclose(f); free(in); free(out); return 2; }
    if (n > 0 && fread(in, 1, (size_t)n, f) != (size_t)n) {
        fclose(f); free(in); free(out); return 2;
    }
    fclose(f);

    size_t out_len = LZODEC_CLI_CAP;
    int rc = lzodec_decompress(in, (size_t)n, out, &out_len, format);
    if (rc != LZODEC_OK) {
        free(in); free(out);
        return 1;
    }

    FILE *o = fopen(argv[3], "wb");
    if (!o) { perror(argv[3]); free(in); free(out); return 2; }
    fwrite(out, 1, out_len, o);
    fclose(o);
    free(in); free(out);
    return 0;
}
