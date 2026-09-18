#include "zfs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "lzodec.h"

#define ZFS_MAGIC         "ZFSF"
#define ZFS_ENTRIES_PER_BLOCK 100
#define ZFS_BLOCK_SIZE    3604   /* 4 (next-block offset) + 100*36 (entries) */

struct ZFSHandle {
    FILE     *file;
    char      path[256];
    int       num_blocks;
    int       active_count;   /* live (non-deleted) entries */
    uint32_t  xor_key;        /* per-archive XOR encryption key; 0 = none */
    int       total_count;    /* raw count from header */
    uint32_t *block_offsets;  /* [num_blocks] absolute file offsets */
    ZFSEntry *entries;        /* [active_count] sorted by name (case-insensitive) */
};

/* -----------------------------------------------------------------------
 * Comparator — case-insensitive on the 16-byte name field.
 * Used for both qsort and bsearch; key is a bare char[16].
 * ----------------------------------------------------------------------- */

static int entry_cmp(const void *a, const void *b)
{
    return strncasecmp((const char *)a, (const char *)b, 16);
}

/* -----------------------------------------------------------------------
 * zfs_open
 * ----------------------------------------------------------------------- */

ZFSHandle *zfs_open(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "[zfs] Cannot open: %s\n", path);
        return NULL;
    }
    setvbuf(f, NULL, _IONBF, 0);

    uint8_t hdr[28];
    if (fread(hdr, 1, 28, f) != 28 || memcmp(hdr, ZFS_MAGIC, 4) != 0) {
        fprintf(stderr, "[zfs] Bad magic: %s\n", path);
        fclose(f);
        return NULL;
    }

    uint32_t version, epb, total_count, xor_key, first_block;
    memcpy(&version,     hdr +  4, 4);
    memcpy(&epb,         hdr + 12, 4);
    memcpy(&total_count, hdr + 16, 4);
    memcpy(&xor_key,     hdr + 20, 4);
    memcpy(&first_block, hdr + 24, 4);

    if (version != 1 || epb != 100) {
        fprintf(stderr, "[zfs] Unexpected header in: %s\n", path);
        fclose(f);
        return NULL;
    }

    int num_blocks = (int)(total_count / ZFS_ENTRIES_PER_BLOCK) + 1;

    ZFSHandle *z = calloc(1, sizeof(*z));
    if (!z) { fclose(f); return NULL; }

    z->file          = f;
    z->num_blocks    = num_blocks;
    z->total_count   = (int)total_count;
    z->xor_key       = xor_key;
    z->entries       = malloc(total_count * sizeof(ZFSEntry));
    z->block_offsets = malloc((size_t)num_blocks * sizeof(uint32_t));
    snprintf(z->path, sizeof(z->path), "%s", path);

    if (!z->entries || !z->block_offsets) {
        zfs_close(z);
        return NULL;
    }

    int live = 0, scanned = 0, bidx = 0;
    long block_pos = (long)first_block;

    while (scanned < (int)total_count) {
        if (bidx < num_blocks)
            z->block_offsets[bidx++] = (uint32_t)block_pos;

        fseek(f, block_pos, SEEK_SET);

        uint32_t next_block;
        if (fread(&next_block, 4, 1, f) != 1) break;

        uint32_t n = ((uint32_t)total_count - (uint32_t)scanned < ZFS_ENTRIES_PER_BLOCK)
                   ? ((uint32_t)total_count - (uint32_t)scanned)
                   : ZFS_ENTRIES_PER_BLOCK;

        for (uint32_t i = 0; i < n; i++) {
            ZFSEntry e;
            fread(e.name,       1, 16, f);
            fread(&e.offset,    4,  1, f);
            fread(&e.index,     4,  1, f);
            fread(&e.size,      4,  1, f);
            fread(&e.timestamp, 4,  1, f);
            fread(&e.flags,     4,  1, f);

            if (e.name[0] == '\0' || (e.flags & ZFS_FLAG_DELETED))
                continue;
            z->entries[live++] = e;
        }

        scanned += (int)n;
        if (next_block == 0) break;
        block_pos = (long)next_block;
    }

    z->active_count = live;
    qsort(z->entries, (size_t)live, sizeof(ZFSEntry), entry_cmp);

    fprintf(stdout, "[zfs] %s: %d entries\n", path, live);
    return z;
}

void zfs_close(ZFSHandle *z)
{
    if (!z) return;
    if (z->file) fclose(z->file);
    free(z->entries);
    free(z->block_offsets);
    free(z);
}

/* -----------------------------------------------------------------------
 * Internal lookup
 * ----------------------------------------------------------------------- */

static ZFSEntry *lookup(ZFSHandle *z, const char *name)
{
    char key[16] = {0};
    strncpy(key, name, sizeof(key));
    return bsearch(key, z->entries, (size_t)z->active_count,
                   sizeof(ZFSEntry), entry_cmp);
}

/* -----------------------------------------------------------------------
 * zfs_get_size
 * ----------------------------------------------------------------------- */

int zfs_get_size(ZFSHandle *z, const char *name)
{
    ZFSEntry *e = lookup(z, name);
    if (!e) return -1;
    return (e->flags & ZFS_FLAG_COMPRESSED)
         ? (int)ZFS_UNCOMPRESSED_SIZE(e->flags)
         : (int)e->size;
}

/* -----------------------------------------------------------------------
 * Decompress + XOR-decrypt
 * ----------------------------------------------------------------------- */

static bool decompress_entry(const ZFSEntry *e, const void *src, void *dst)
{
    /* LZO1X and LZO1Y are distinct stream variants; lzodec covers both. */
    size_t dst_len = ZFS_UNCOMPRESSED_SIZE(e->flags);
    int format = (e->flags & ZFS_FLAG_COMP_LZO1Y) ? LZODEC_LZO1Y : LZODEC_LZO1X;
    int rc = lzodec_decompress(src, e->size, dst, &dst_len, format);

    if (rc != LZODEC_OK) {
        fprintf(stderr, "[zfs] Decompress failed (rc=%d) for %.16s\n",
                rc, e->name);
        return false;
    }
    return true;
}

static void xor_decrypt(void *buf, size_t size, uint32_t key)
{
    if (!key) return;
    uint32_t *p = (uint32_t *)buf;
    for (size_t i = 0; i < size / 4; i++)
        p[i] ^= key;
}

/* -----------------------------------------------------------------------
 * Core read — decompresses entry into caller-supplied output buffer.
 * comp_tmp must be at least e->size bytes.
 * ----------------------------------------------------------------------- */

static bool read_entry(ZFSHandle *z, const ZFSEntry *e,
                       void *out, size_t *size_out)
{
    bool compressed = (e->flags & ZFS_FLAG_COMPRESSED) != 0;
    size_t out_size = compressed
                    ? (size_t)ZFS_UNCOMPRESSED_SIZE(e->flags)
                    : (size_t)e->size;

    fseek(z->file, (long)e->offset, SEEK_SET);

    if (!compressed) {
        if (fread(out, 1, e->size, z->file) != e->size) return false;
        xor_decrypt(out, e->size, z->xor_key);
        if (size_out) *size_out = e->size;
        return true;
    }

    void *comp = malloc(e->size);
    if (!comp) return false;

    bool ok = false;
    if (fread(comp, 1, e->size, z->file) == e->size &&
        decompress_entry(e, comp, out)) {
        xor_decrypt(out, out_size, z->xor_key);
        if (size_out) *size_out = out_size;
        ok = true;
    }

    free(comp);
    return ok;
}

/* -----------------------------------------------------------------------
 * zfs_read / zfs_read_into
 * ----------------------------------------------------------------------- */

void *zfs_read(ZFSHandle *z, const char *name, size_t *size_out)
{
    ZFSEntry *e = lookup(z, name);
    if (!e) {
        fprintf(stderr, "[zfs] Not found: %.16s\n", name);
        return NULL;
    }

    bool compressed = (e->flags & ZFS_FLAG_COMPRESSED) != 0;
    size_t out_size = compressed
                    ? (size_t)ZFS_UNCOMPRESSED_SIZE(e->flags)
                    : (size_t)e->size;

    void *buf = malloc(out_size);
    if (!buf) return NULL;

    if (!read_entry(z, e, buf, size_out)) {
        free(buf);
        return NULL;
    }
    return buf;
}

bool zfs_read_into(ZFSHandle *z, const char *name,
                   void *buf, size_t cap, size_t *size_out)
{
    ZFSEntry *e = lookup(z, name);
    if (!e) return false;

    bool compressed = (e->flags & ZFS_FLAG_COMPRESSED) != 0;
    size_t out_size = compressed
                    ? (size_t)ZFS_UNCOMPRESSED_SIZE(e->flags)
                    : (size_t)e->size;

    if (cap < out_size) return false;
    return read_entry(z, e, buf, size_out);
}

/* -----------------------------------------------------------------------
 * Enumeration (tools)
 * ----------------------------------------------------------------------- */

int zfs_entry_count(ZFSHandle *z)
{
    return z ? z->active_count : 0;
}

const ZFSEntry *zfs_entry_at(ZFSHandle *z, int i)
{
    if (!z || i < 0 || i >= z->active_count) return NULL;
    return &z->entries[i];
}