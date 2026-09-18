#ifndef FS_H
#define FS_H

/*
 * fs.h — Filesystem utilities
 *
 * The original game ran on Windows with a case-insensitive filesystem and
 * DOS 8.3 uppercase asset names (LOADGAME.PCX, ENGSND.DAT, etc.).
 * On Linux we need to handle both the backslash-to-slash conversion and
 * the case mismatch transparently, so callers can use the paths exactly
 * as they appear in the original binary without per-call workarounds.
 */

#include <stdio.h>

/*
 * fs_set_root()
 *   Set the asset root directory. All subsequent fs_fopen() calls prepend
 *   this prefix. Defaults to "." (current working directory).
 *   Call once at startup after parsing command-line arguments.
 *   The path is copied internally; caller does not need to keep it alive.
 */
void fs_set_root(const char *root);

/*
 * fs_fopen()
 *   Drop-in replacement for fopen() that:
 *     1. Prepends the asset root set by fs_set_root().
 *     2. Converts backslashes to forward slashes.
 *     3. Tries the resolved path as-is.
 *     4. If that fails, tries with the basename uppercased.
 *     5. If that fails, tries with the basename lowercased.
 *   Returns NULL only if all three attempts fail.
 */
FILE *fs_fopen(const char *path, const char *mode);

/* Returns the current asset root (pointer to internal buffer — do not free). */
const char *fs_get_root(void);

#endif /* FS_H */
