/*
 * vfs.c — Virtual File System
 *
 * Parses the selected game's .zix to build a file→source mapping, then
 * dispatches reads to ZFS archives or loose files.
 *
 * .zix format (text, CRLF):
 *   Line 1:   total file count
 *   Lines 2+: source declarations — "<type> <path>" until first "---" separator
 *             type 0 = loose directory, ignored (we use asset root)
 *   Then:     "DIR: <archive_name>" introduces a ZFS source
 *             "---" separators are skipped
 *             "<src_idx> <filename>" — one file entry per line
 */

#include "vfs.h"
#include "zfs.h"
#include "fs.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

/* -----------------------------------------------------------------------
 * Source table
 * ----------------------------------------------------------------------- */

#define VFS_MAX_SOURCES 16

typedef enum { SRC_LOOSE = 0, SRC_ZFS = 1 } VFSSrcType;

typedef struct {
    VFSSrcType  type;
    char        name[256];    /* archive filename (e.g. "nitro.zfs") or "" for loose */
    ZFSHandle  *zfs;          /* non-NULL once lazily opened */
} VFSSource;

static VFSSource  s_sources[VFS_MAX_SOURCES];
static int        s_source_count = 0;
static VFSProfile s_profile = VFS_PROFILE_NONE;

#ifdef __EMSCRIPTEN__
static int s_probe_log = 0;   /* browser: silent until ?dev=1 */
#else
static int s_probe_log = 1;   /* native tools keep probe visibility */
#endif

/* Browser probe misses feed the ?dev=1 panel instead of stderr. Keep enough
 * names for a complete diagnostic session; the final bound is calibrated by
 * the P01 + vehicle-heavy browser journey (H-UAT-031). Native tools retain
 * their existing 512-name stderr bound and marker behavior. */
#ifdef __EMSCRIPTEN__
#define VFS_PROBE_LOG_MAX  1024
#else
#define VFS_PROBE_LOG_MAX  512
#endif
#define VFS_PROBE_NAME_LEN 32
static char s_probe_seen[VFS_PROBE_LOG_MAX][VFS_PROBE_NAME_LEN];
static int  s_probe_n;
static int  s_probe_overflow;

/* -----------------------------------------------------------------------
 * File table — sorted by name for bsearch
 * ----------------------------------------------------------------------- */

typedef struct {
    char    name[16];    /* lowercase, null-padded */
    uint8_t src_idx;
} VFSFile;

static VFSFile *s_files      = NULL;
static int      s_file_count = 0;

static int file_cmp(const void *a, const void *b)
{
    return strncasecmp((const char *)a, ((const VFSFile *)b)->name, 16);
}

static int file_sort_cmp(const void *a, const void *b)
{
    return strncasecmp(((const VFSFile *)a)->name,
                       ((const VFSFile *)b)->name, 16);
}

/* -----------------------------------------------------------------------
 * .zix parser helpers
 * ----------------------------------------------------------------------- */

static char *zix_line(FILE *f, char *buf, size_t n)
{
    if (!fgets(buf, (int)n, f)) return NULL;
    size_t len = strlen(buf);
    while (len > 0 && (buf[len-1] == '\r' || buf[len-1] == '\n'))
        buf[--len] = '\0';
    return buf;
}

static bool is_separator(const char *line)
{
    for (const char *p = line; *p; p++)
        if (*p != '-') return false;
    return line[0] == '-';
}

/* -----------------------------------------------------------------------
 * vfs_init
 * ----------------------------------------------------------------------- */

bool vfs_init(void)
{
    const char *root = fs_get_root();
    static const struct {
        const char *name;
        VFSProfile profile;
    } candidates[] = {
        { "nitro.zix", VFS_PROFILE_NITRO },
        { "NITRO.ZIX", VFS_PROFILE_NITRO },
        { "i76.zix",   VFS_PROFILE_BASE },
        { "I76.ZIX",   VFS_PROFILE_BASE },
    };
    char zix_path[512];
    FILE *f = NULL;

    s_profile = VFS_PROFILE_NONE;
    for (size_t i = 0; i < sizeof candidates / sizeof candidates[0]; i++) {
        snprintf(zix_path, sizeof zix_path, "%s/%s", root, candidates[i].name);
        f = fopen(zix_path, "r");
        if (f) {
            s_profile = candidates[i].profile;
            break;
        }
    }
    if (!f) {
        fprintf(stderr, "[vfs] no supported archive index under %s\n", root);
        return false;
    }

    char line[512];

    /* Line 1: total file count */
    if (!zix_line(f, line, sizeof(line))) { fclose(f); return false; }
    int total = atoi(line);
    if (total <= 0) { fclose(f); return false; }

    s_files = malloc((size_t)total * sizeof(VFSFile));
    if (!s_files) { fclose(f); return false; }

    /* Source 0 is always the loose-file root (type=0).
     * Ignore the Windows path in the .zix; use asset root instead. */
    s_sources[0].type = SRC_LOOSE;
    s_sources[0].name[0] = '\0';
    s_sources[0].zfs  = NULL;
    s_source_count = 1;

    int loaded = 0;

    typedef enum { STATE_SOURCES, STATE_FILES } ParseState;
    ParseState state = STATE_SOURCES;

    while (zix_line(f, line, sizeof(line))) {
        if (is_separator(line)) {
            state = STATE_FILES;
            continue;
        }

        if (state == STATE_SOURCES) {
            /* "<type> <path>" — only care about type here; path is Windows-specific */
            continue;
        }

        /* STATE_FILES */
        if (strncmp(line, "DIR:", 4) == 0) {
            /* New ZFS source */
            if (s_source_count < VFS_MAX_SOURCES) {
                const char *name = line + 4;
                while (*name == ' ') name++;
                int idx = s_source_count++;
                s_sources[idx].type = SRC_ZFS;
                snprintf(s_sources[idx].name, sizeof(s_sources[idx].name),
                         "%s", name);
                s_sources[idx].zfs = NULL;
            }
            continue;
        }

        /* "<src_idx> <filename>" */
        int src_idx = 0;
        char fname[64] = {0};
        if (sscanf(line, "%d %63s", &src_idx, fname) != 2) continue;
        if (fname[0] == '\0' || src_idx >= s_source_count) continue;
        if (loaded >= total) continue;

        VFSFile *vf = &s_files[loaded++];
        memset(vf->name, 0, 16);
        strncpy(vf->name, fname, 15);
        /* lowercase for consistent lookup */
        for (int i = 0; i < 16 && vf->name[i]; i++)
            vf->name[i] = (char)tolower((unsigned char)vf->name[i]);
        vf->src_idx = (uint8_t)src_idx;
    }

    fclose(f);
    s_file_count = loaded;
    qsort(s_files, (size_t)s_file_count, sizeof(VFSFile), file_sort_cmp);

    fprintf(stdout, "[vfs] %d files across %d source(s)\n",
            s_file_count, s_source_count);
    return true;
}

/* -----------------------------------------------------------------------
 * vfs_shutdown
 * ----------------------------------------------------------------------- */

void vfs_shutdown(void)
{
    for (int i = 0; i < s_source_count; i++) {
        if (s_sources[i].zfs) {
            zfs_close(s_sources[i].zfs);
            s_sources[i].zfs = NULL;
        }
    }
    free(s_files);
    s_files       = NULL;
    s_file_count  = 0;
    s_source_count = 0;
    s_profile = VFS_PROFILE_NONE;
    s_probe_n = 0;
    s_probe_overflow = 0;
}

VFSProfile vfs_profile(void)
{
    return s_profile;
}
/* -----------------------------------------------------------------------
 * Lazy ZFS open
 * ----------------------------------------------------------------------- */

static ZFSHandle *get_zfs(int src_idx)
{
    VFSSource *src = &s_sources[src_idx];
    if (src->zfs) return src->zfs;

    char path[512];
    snprintf(path, sizeof(path), "%s/%s", fs_get_root(), src->name);

    /* Try exact case, then uppercase, then lowercase */
    src->zfs = zfs_open(path);
    if (!src->zfs) {
        /* uppercase the filename part */
        char up[512];
        snprintf(up, sizeof(up), "%s", path);
        char *slash = strrchr(up, '/');
        char *p = slash ? slash + 1 : up;
        for (; *p; p++) *p = (char)toupper((unsigned char)*p);
        src->zfs = zfs_open(up);
    }
    if (!src->zfs)
        fprintf(stderr, "[vfs] Cannot open ZFS source: %s\n", src->name);
    return src->zfs;
}

/* -----------------------------------------------------------------------
 * File lookup
 * ----------------------------------------------------------------------- */

static VFSFile *find_file(const char *name)
{
    char key[16] = {0};
    strncpy(key, name, 15);
    for (int i = 0; i < 15 && key[i]; i++)
        key[i] = (char)tolower((unsigned char)key[i]);
    return bsearch(key, s_files, (size_t)s_file_count, sizeof(VFSFile), file_cmp);
}

/* -----------------------------------------------------------------------
 * vfs_exists
 * ----------------------------------------------------------------------- */

int vfs_exists(const char *name)
{
    VFSFile *vf = find_file(name);
    if (vf) {
        VFSSource *src = &s_sources[vf->src_idx];
        if (src->type == SRC_ZFS) {
            ZFSHandle *z = get_zfs(vf->src_idx);
            if (!z) return 0;
            int sz = zfs_get_size(z, name);
            return sz > 0 ? sz : 0;
        }
    }
    /* Loose file — indexed with a loose source OR not indexed at all
       (web staging drops files straight into MEMFS): open to get size. */
    FILE *f = fs_fopen(name, "rb");
    if (!f) return 0;
    fseek(f, 0, SEEK_END);
    int sz = (int)ftell(f);
    fclose(f);
    return sz;
}

/* -----------------------------------------------------------------------
 * vfs_read_file
 * ----------------------------------------------------------------------- */

static void *read_loose(const char *name, size_t *size_out)
{
    FILE *f = fs_fopen(name, "rb");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    size_t sz = (size_t)ftell(f);
    fseek(f, 0, SEEK_SET);
    void *buf = malloc(sz);
    if (!buf) { fclose(f); return NULL; }
    if (fread(buf, 1, sz, f) != sz) { free(buf); fclose(f); return NULL; }
    fclose(f);
    if (size_out) *size_out = sz;
    return buf;
}

void vfs_set_probe_log(int on)
{
    s_probe_log = on ? 1 : 0;
}

static int probe_record(const char *name)
{
    /* Compare the stored (possibly truncated) form. snprintf writes at most
     * VFS_PROBE_NAME_LEN-1 chars; a full-length strcasecmp against `name`
     * would miss itself and re-log forever, flooding the table. */
    char key[VFS_PROBE_NAME_LEN];
    memset(key, 0, sizeof key);
    snprintf(key, sizeof key, "%s", name);

    for (int i = 0; i < s_probe_n; i++)
        if (strcasecmp(s_probe_seen[i], key) == 0) return 0;
    if (s_probe_n >= VFS_PROBE_LOG_MAX) {
        s_probe_overflow++;
#ifndef __EMSCRIPTEN__
        if (s_probe_overflow == 1) {
            fprintf(stderr,
                    "[vfs] probe table full — further misses suppressed (%d seen)\n",
                    s_probe_n);
        }
#endif
        return 0;
    }
    memcpy(s_probe_seen[s_probe_n++], key, VFS_PROBE_NAME_LEN);
    return 1;
}

int vfs_probe_count(void)
{
    return s_probe_n;
}

const char *vfs_probe_name(int i)
{
    return i >= 0 && i < s_probe_n ? s_probe_seen[i] : "";
}

int vfs_probe_overflow(void)
{
    return s_probe_overflow;
}

static void *read_file(const char *name, size_t *size_out)
{
    VFSFile *vf = find_file(name);

    if (vf && s_sources[vf->src_idx].type == SRC_ZFS) {
        ZFSHandle *z = get_zfs(vf->src_idx);
        if (!z) return NULL;
        return zfs_read(z, name, size_out);
    }

    /* SRC_LOOSE or not in index: read via fs_fopen (handles subdirs + case) */
    return read_loose(name, size_out);
}

void *vfs_read_file(const char *name, size_t *size_out)
{
    void *buf = read_file(name, size_out);
    if (!buf)
        fprintf(stderr, "[vfs] Not found: %s\n", name);
    return buf;
}

void *vfs_try_read(const char *name, size_t *size_out)
{
    void *buf = read_file(name, size_out);
    if (!buf && s_probe_log && name && *name && probe_record(name)) {
#ifdef __EMSCRIPTEN__
        if (s_probe_n == 1)
            fprintf(stderr, "[vfs] probe misses recorded — see dev panel\n");
#else
        fprintf(stderr, "[vfs] Not found: %s\n", name);
#endif
    }
    return buf;
}

/* -----------------------------------------------------------------------
 * vfs_free / vfs_foreach
 * ----------------------------------------------------------------------- */

void vfs_free(void *buf)
{
    free(buf);
}

void vfs_foreach(void (*fn)(const char *name, int src_type, void *ud), void *ud)
{
    for (int i = 0; i < s_file_count; i++) {
        VFSFile *vf = &s_files[i];
        int type = (vf->src_idx < s_source_count)
                   ? (int)s_sources[vf->src_idx].type : 0;
        fn(vf->name, type, ud);
    }
}