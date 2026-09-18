/*
 * fs.c — Filesystem utilities
 */

#include "fs.h"

#include <ctype.h>
#include <string.h>

static char s_root[512] = ".";

void fs_set_root(const char *root)
{
    strncpy(s_root, root, sizeof(s_root) - 1);
    s_root[sizeof(s_root) - 1] = '\0';
    /* Strip trailing slash for consistent joining */
    size_t len = strlen(s_root);
    while (len > 1 && (s_root[len - 1] == '/' || s_root[len - 1] == '\\'))
        s_root[--len] = '\0';
}

static void normalise_separators(char *buf)
{
    for (char *p = buf; *p; p++)
        if (*p == '\\') *p = '/';
}

static void set_basename_case(char *buf, int upper)
{
    char *base = buf;
    for (char *p = buf; *p; p++)
        if (*p == '/') base = p + 1;
    for (char *p = base; *p; p++)
        *p = (char)(upper ? toupper((unsigned char)*p)
                          : tolower((unsigned char)*p));
}

static void build_path(char *buf, size_t bufsz, const char *path)
{
    if (path[0] == '/' || (path[0] && path[1] == ':')) {
        /* Absolute path — use as-is */
        strncpy(buf, path, bufsz - 1);
    } else {
        snprintf(buf, bufsz, "%s/%s", s_root, path);
    }
    buf[bufsz - 1] = '\0';
    normalise_separators(buf);
}

const char *fs_get_root(void) { return s_root; }

FILE *fs_fopen(const char *path, const char *mode)
{
    char buf[1024];
    build_path(buf, sizeof(buf), path);

    FILE *f = fopen(buf, mode);
    if (f) return f;

    set_basename_case(buf, 1); /* try UPPERCASE.EXT */
    f = fopen(buf, mode);
    if (f) return f;

    set_basename_case(buf, 0); /* try lowercase.ext */
    return fopen(buf, mode);
}
