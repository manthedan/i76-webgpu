#ifndef ZFS_H
#define ZFS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Entry flags (low byte) */
#define ZFS_FLAG_DELETED     0x01u
#define ZFS_FLAG_COMP_LZO1X  0x02u   /* bit 1 — LZO1X-format stream */
#define ZFS_FLAG_COMP_LZO1Y  0x04u   /* bit 2 — LZO1Y-format stream */
/* Distinct stream variants; both decode through src/engine/lzodec.c. */
#define ZFS_FLAG_COMPRESSED  (ZFS_FLAG_COMP_LZO1X | ZFS_FLAG_COMP_LZO1Y)

/* Bits 8-31 of flags encode the uncompressed size for compressed entries. */
#define ZFS_UNCOMPRESSED_SIZE(flags) ((uint32_t)((flags) >> 8))

typedef struct {
    char     name[16];     /* null-padded, DOS 8.3; stored all-caps on disk */
    uint32_t offset;       /* absolute byte offset of file data within .zfs */
    uint32_t index;        /* sequential 0-based index across all blocks */
    uint32_t size;         /* stored (compressed) size in bytes */
    uint32_t timestamp;
    uint32_t flags;
} ZFSEntry;

typedef struct ZFSHandle ZFSHandle;

/* Opens a ZFS archive for reading. Returns NULL on failure. */
ZFSHandle *zfs_open(const char *path);
void       zfs_close(ZFSHandle *z);

/*
 * Returns the uncompressed byte count for the named file, or -1 if not found.
 * For uncompressed entries this equals entry.size.
 * For compressed entries this equals entry.flags >> 8.
 */
int  zfs_get_size(ZFSHandle *z, const char *name);

/*
 * Allocates a buffer, reads and decompresses the named file into it.
 * Caller must free() the returned pointer.
 * Returns NULL if the file is not found or on I/O or decompression error.
 */
void *zfs_read(ZFSHandle *z, const char *name, size_t *size_out);

/*
 * Reads and decompresses the named file into a caller-supplied buffer.
 * Returns false if the file is not found, cap < uncompressed size, or on error.
 */
bool zfs_read_into(ZFSHandle *z, const char *name,
                   void *buf, size_t cap, size_t *size_out);

/*
 * Enumeration (for tools). Entries are the live (non-deleted) directory
 * entries, sorted by name (case-insensitive). zfs_entry_at returns NULL for
 * out-of-range indices. The returned pointer is owned by the handle.
 */
int             zfs_entry_count(ZFSHandle *z);
const ZFSEntry *zfs_entry_at(ZFSHandle *z, int i);

#endif /* ZFS_H */