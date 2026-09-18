#ifndef VFS_H
#define VFS_H

#include <stdbool.h>
#include <stddef.h>

typedef enum {
    VFS_PROFILE_NONE = 0,
    VFS_PROFILE_NITRO,
    VFS_PROFILE_BASE
} VFSProfile;

/*
 * Initialise the VFS from the archive index under fs_get_root().
 * Nitro's nitro.zix wins when both profiles are present; otherwise the base
 * game's I76.ZIX is used. Call once after fs_set_root().
 */
bool vfs_init(void);
void vfs_shutdown(void);
VFSProfile vfs_profile(void);

/*
 * Read a file by name. Searches ZFS archives and loose files.
 * Returns a heap-allocated buffer; caller must call vfs_free().
 * Returns NULL if the file is not found.
 *
 * vfs_read_file logs every miss — required-asset callers keep that.
 * vfs_try_read is the probe path: a miss is a fallthrough to another
 * working source, recorded once per stored name only when probe logging is
 * on (native default on; browser only after web_set_log_level / ?dev=1).
 * Native tools print each stored name and retain their 512-name saturation
 * marker. The browser prints one summary line and exposes its larger table to
 * the dev panel. Overflow counts every miss that cannot fit in the table.
 */
void *vfs_read_file(const char *name, size_t *size_out);
void *vfs_try_read(const char *name, size_t *size_out);
void vfs_set_probe_log(int on);
int vfs_probe_count(void);
const char *vfs_probe_name(int i);
int vfs_probe_overflow(void);

/*
 * Returns the uncompressed byte count for the named file, or 0 if not found.
 * Does not allocate or read file data.
 */
int vfs_exists(const char *name);

/* Free a buffer returned by vfs_read_file() / vfs_try_read(). */
void vfs_free(void *buf);

/* Call fn(name, src_type, ud) for every file in the VFS index.
 * src_type: 0 = loose, 1 = ZFS archive. */
void vfs_foreach(void (*fn)(const char *name, int src_type, void *ud), void *ud);

#endif /* VFS_H */