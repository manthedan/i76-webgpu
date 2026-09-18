/*
 * meshcache.c — GEO mesh cache (see meshcache.h)
 *
 * Faithful reimplementation of the original's mesh cache. The data-structure
 * mechanics are reproduced exactly from the reversed code; the two boundaries
 * we haven't fully reversed (file resolution + GEO byte decode) sit behind the
 * `s_loader` hook and `geo_build_mesh()`, both clearly marked TODO.
 *
 * Memory model note: the original used two Win32 heaps (g_object_heap for mesh
 * data, g_object_heap2 for nodes) and triggered eviction on HeapAlloc failure,
 * while tracking free space in g_object_heap_size. malloc() won't fail the same
 * way, so we reproduce the *behaviour* with an explicit byte budget
 * (s_heap_free): eviction kicks in when an allocation would exceed the budget.
 * That preserves both the eviction order and the free-space accounting the
 * original exposes (e.g. assets_preload's "< 200000" preload cut-offs).
 */

#include "meshcache.h"

#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "engine/vfs.h"
#include "engine/geomesh.h"

/* ----------------------------------------------------------------------- */
/* Constants (all reversed)                                                  */
/* ----------------------------------------------------------------------- */

#define MC_BUCKETS   2029        /* 0x7ed */
#define MC_HASH_MUL  1741        /* 0x6cd */
#define MC_HASH_ADD  2731        /* 0xaab */

#define GEO_MAGIC    0x2e47454fu /* GEO mesh file magic */

/*
 * Heap budget. Original obj_system_init: dwTotalPhys > 48 MB -> 2,000,000;
 * otherwise 1,300,000. Modern machines are always the high tier.
 */
#define MC_BUDGET    2000000L

/* ----------------------------------------------------------------------- */
/* Node (our analogue of the reversed 0x20-byte cache node)                  */
/* Field comments give the original byte offsets for cross-reference.        */
/* ----------------------------------------------------------------------- */

typedef struct MeshNode {
    char     name[8];               /* +0x00  uppercased 8-byte key          */
    uint32_t refcount;              /* +0x08  0 => idle, on LRU, evictable    */
    void    *mesh;                  /* +0x0c  decoded mesh buffer             */
    size_t   mesh_size;             /* (ours) for budget accounting           */
    struct MeshNode *byptr_next;    /* +0x10  chain in the by-pointer index   */
    struct MeshNode *byname_next;   /* +0x14  chain in the by-name index      */
    struct MeshNode *lru_next;      /* +0x18                                  */
    struct MeshNode *lru_prev;      /* +0x1c                                  */
} MeshNode;

/* ----------------------------------------------------------------------- */
/* State (file-static analogues of the original globals)                     */
/* ----------------------------------------------------------------------- */

static MeshNode *s_by_ptr[MC_BUCKETS];   /* g_obj_buckets  — keyed by mesh ptr */
static MeshNode *s_by_name[MC_BUCKETS];  /* g_obj_buckets2 — keyed by name      */
static MeshNode *s_lru_head;             /* g_meshcache_lru_head (oldest)       */
static MeshNode *s_lru_tail;             /* g_meshcache_lru_tail (newest)       */
static long      s_heap_free;            /* g_object_heap_size (free budget)    */
static bool      s_inited;

/*
 * Image loader hook (our analogue of vfs_lod). Returns the raw GEO file bytes
 * for `name` (caller frees via s_free), or NULL if not found. Swappable so the
 * self-test can inject synthetic images without touching the VFS.
 */
typedef void *(*MeshLoaderFn)(const char *name, size_t *out_size);
static void  *default_loader(const char *name, size_t *out_size);
static void   default_free(void *buf);

static MeshLoaderFn s_loader = default_loader;
static void       (*s_free)(void *) = default_free;

/* ----------------------------------------------------------------------- */
/* Hashing (reversed exactly)                                                */
/* ----------------------------------------------------------------------- */

static unsigned mc_hash(uint32_t k)
{
    return (unsigned)((k * (uint32_t)MC_HASH_MUL + (uint32_t)MC_HASH_ADD) % MC_BUCKETS);
}

/* Normalise a name to the 8-byte uppercased key the original hashes. */
static void make_key(const char *name, char key[8])
{
    memset(key, 0, 8);
    for (int i = 0; i < 8 && name[i]; i++)
        key[i] = (char)toupper((unsigned char)name[i]);
}

/* By-name bucket: fold the two 32-bit halves, ASCII-upcase mask, then hash. */
static unsigned hash_name(const char key[8])
{
    uint32_t lo, hi;
    memcpy(&lo, key,     4);   /* little-endian load matches the original */
    memcpy(&hi, key + 4, 4);
    return mc_hash((hi ^ lo) & 0xdfdfdfdfu);
}

/* By-pointer bucket: hash the mesh buffer's address. */
static unsigned hash_ptr(const void *mesh)
{
    return mc_hash((uint32_t)(uintptr_t)mesh);
}

/* ----------------------------------------------------------------------- */
/* LRU list (refcount==0 nodes only)                                         */
/* ----------------------------------------------------------------------- */

static void lru_append(MeshNode *n)   /* add at tail (most-recently-released) */
{
    n->lru_next = NULL;
    n->lru_prev = s_lru_tail;
    if (s_lru_tail) s_lru_tail->lru_next = n;
    else            s_lru_head = n;
    s_lru_tail = n;
}

static void lru_unlink(MeshNode *n)
{
    if (n->lru_prev) n->lru_prev->lru_next = n->lru_next;
    else             s_lru_head = n->lru_next;
    if (n->lru_next) n->lru_next->lru_prev = n->lru_prev;
    else             s_lru_tail = n->lru_prev;
    n->lru_next = n->lru_prev = NULL;
}

/* ----------------------------------------------------------------------- */
/* Index chain removal                                                       */
/* ----------------------------------------------------------------------- */

static void byname_unlink(MeshNode *n)
{
    unsigned h = hash_name(n->name);
    MeshNode **pp = &s_by_name[h];
    while (*pp && *pp != n) pp = &(*pp)->byname_next;
    if (*pp) *pp = n->byname_next;
}

static void byptr_unlink(MeshNode *n)
{
    unsigned h = hash_ptr(n->mesh);
    MeshNode **pp = &s_by_ptr[h];
    while (*pp && *pp != n) pp = &(*pp)->byptr_next;
    if (*pp) *pp = n->byptr_next;
}

/* ----------------------------------------------------------------------- */
/* Eviction — free oldest LRU entries until `need` bytes fit the budget      */
/* ----------------------------------------------------------------------- */

static void make_room(size_t need)
{
    while (s_heap_free < (long)need) {
        MeshNode *victim = s_lru_head;
        if (!victim) {
            /*
             * Original fatal_error("new geometry: couldn't make room") — every
             * cached mesh is still in use (refcount>0). We log and let the
             * allocation push us over budget rather than abort the process.
             */
            fprintf(stderr, "[meshcache] over budget: no idle entries to evict "
                            "(need %zu, free %ld)\n", need, s_heap_free);
            return;
        }
        lru_unlink(victim);
        byname_unlink(victim);
        byptr_unlink(victim);
        s_heap_free += (long)victim->mesh_size;
        geomesh_free(victim->mesh);   /* free decoded mesh (the cached buffer)  */
        free(victim);                 /* free node (heap2)                      */
    }
}

/* ----------------------------------------------------------------------- */
/* GEO load + decode (the un-reversed boundary)                              */
/* ----------------------------------------------------------------------- */

static void *default_loader(const char *name, size_t *out_size)
{
    /*
     * TODO: the original vfs_lod resolves a LOD-suffixed filename before
     * reading. We don't have that reversed, so we read `name` directly.
     */
    return vfs_read_file(name, out_size);
}

static void default_free(void *buf)
{
    vfs_free(buf);
}

/*
 * geo_build_mesh (FUN_0043aa60) — decode a raw GEO image into a renderable
 * mesh. Now uses the real OEG decoder (geomesh_decode); the cached buffer is a
 * GeoMesh*. Budget accounting uses the source image size (the original tracked
 * the decoded HeapSize; our explicit byte budget is already an approximation
 * — see file header — and source size is a stable, faithful-enough proxy).
 */
static void *geo_build_mesh(const void *image, size_t image_size, size_t *out_size)
{
    GeoMesh *m = geomesh_decode(image, image_size);
    if (!m) {
        fprintf(stderr, "[meshcache] GEO decode failed (%zu bytes)\n", image_size);
        return NULL;
    }
    *out_size = image_size;
    return m;
}

/* ----------------------------------------------------------------------- */
/* Public API                                                                */
/* ----------------------------------------------------------------------- */

void meshcache_init(void)
{
    memset(s_by_ptr,  0, sizeof(s_by_ptr));
    memset(s_by_name, 0, sizeof(s_by_name));
    s_lru_head = s_lru_tail = NULL;
    s_heap_free = MC_BUDGET;
    s_inited = true;
}

void meshcache_shutdown(void)
{
    for (int i = 0; i < MC_BUCKETS; i++) {
        MeshNode *n = s_by_name[i];
        while (n) {
            MeshNode *next = n->byname_next;
            geomesh_free(n->mesh);
            free(n);
            n = next;
        }
    }
    memset(s_by_ptr,  0, sizeof(s_by_ptr));
    memset(s_by_name, 0, sizeof(s_by_name));
    s_lru_head = s_lru_tail = NULL;
    s_heap_free = MC_BUDGET;
    s_inited = false;
}

void *geo_cache_acquire(const char *name)
{
    if (!s_inited) meshcache_init();

    char key[8];
    make_key(name, key);

    /* --- hit: find by name, bump refcount, pull off LRU if it was idle --- */
    unsigned h = hash_name(key);
    for (MeshNode *n = s_by_name[h]; n; n = n->byname_next) {
        if (memcmp(n->name, key, 8) == 0) {
            if (n->refcount++ == 0)
                lru_unlink(n);
            return n->mesh;
        }
    }

    /* --- miss: load + decode + insert --- */
    size_t img_size = 0;
    void *image = s_loader(name, &img_size);
    if (!image)
        return NULL;

    size_t mesh_size = 0;
    void *mesh = geo_build_mesh(image, img_size, &mesh_size);
    s_free(image);
    if (!mesh)
        return NULL;

    make_room(mesh_size);
    s_heap_free -= (long)mesh_size;

    MeshNode *n = calloc(1, sizeof(*n));
    if (!n) {
        free(mesh);
        s_heap_free += (long)mesh_size;
        return NULL;
    }
    memcpy(n->name, key, 8);
    n->refcount  = 1;          /* in use — not on the LRU */
    n->mesh      = mesh;
    n->mesh_size = mesh_size;

    n->byname_next = s_by_name[h];
    s_by_name[h] = n;

    unsigned hp = hash_ptr(mesh);
    n->byptr_next = s_by_ptr[hp];
    s_by_ptr[hp] = n;

    return mesh;
}

void geo_cache_release(void *mesh)
{
    if (!mesh) return;

    unsigned hp = hash_ptr(mesh);
    for (MeshNode *n = s_by_ptr[hp]; n; n = n->byptr_next) {
        if (n->mesh == mesh) {
            if (n->refcount == 0) {
                fprintf(stderr, "[meshcache] double release of %.8s\n", n->name);
                return;
            }
            if (--n->refcount == 0)
                lru_append(n);   /* idle now: cached but evictable */
            return;
        }
    }
    fprintf(stderr, "[meshcache] release of unknown mesh %p\n", mesh);
}

long meshcache_free_bytes(void) { return s_heap_free; }

int meshcache_count(void)
{
    int c = 0;
    for (int i = 0; i < MC_BUCKETS; i++)
        for (MeshNode *n = s_by_name[i]; n; n = n->byname_next)
            c++;
    return c;
}

/* ----------------------------------------------------------------------- */
/* Self-test — synthetic loader, exercises the certain mechanics             */
/* ----------------------------------------------------------------------- */

/* Synthetic GEO image: valid magic + padding to a size encoded in the name's
 * trailing digit (e.g. "MESH3" -> 3 KB), so the test can drive the budget. */
static void *synth_loader(const char *name, size_t *out_size)
{
    size_t n = strlen(name);
    int kb = (n && isdigit((unsigned char)name[n - 1])) ? name[n - 1] - '0' : 1;
    if (kb < 1) kb = 1;
    size_t sz = (size_t)kb * 1024;
    uint8_t *buf = malloc(sz);
    if (!buf) return NULL;
    memset(buf, 0, sz);
    uint32_t magic = GEO_MAGIC;
    memcpy(buf, &magic, 4);
    *out_size = sz;
    return buf;
}

static void synth_free(void *buf) { free(buf); }

#define MC_CHECK(cond) do { \
    if (!(cond)) { fprintf(stderr, "[meshcache] selftest FAIL: %s (line %d)\n", \
                           #cond, __LINE__); ok = false; goto done; } } while (0)

bool meshcache_selftest(void)
{
    bool ok = true;

    /* Save real state, install synthetic loader on a clean cache. */
    MeshLoaderFn save_loader = s_loader;
    void (*save_free)(void *) = s_free;
    meshcache_shutdown();
    s_loader = synth_loader;
    s_free   = synth_free;
    meshcache_init();

    /* 1. case-insensitive identity: same name -> same buffer, refcount shared */
    void *a1 = geo_cache_acquire("car1");   /* 1 KB */
    void *a2 = geo_cache_acquire("CAR1");   /* same key, uppercased */
    MC_CHECK(a1 != NULL);
    MC_CHECK(a1 == a2);
    MC_CHECK(meshcache_count() == 1);
    MC_CHECK(meshcache_free_bytes() == MC_BUDGET - 1024);

    /* 2. distinct names -> distinct entries */
    void *b = geo_cache_acquire("tank2");   /* 2 KB */
    MC_CHECK(b != NULL && b != a1);
    MC_CHECK(meshcache_count() == 2);
    MC_CHECK(meshcache_free_bytes() == MC_BUDGET - 1024 - 2048);

    /* 3. release to idle keeps it cached; re-acquire is a hit (no new entry) */
    geo_cache_release(a1);                  /* car1 refcount 2->1 */
    geo_cache_release(a2);                  /* car1 refcount 1->0, now on LRU */
    MC_CHECK(meshcache_count() == 2);       /* still cached */
    void *a3 = geo_cache_acquire("car1");   /* hit -> pulled off LRU */
    MC_CHECK(a3 == a1);
    MC_CHECK(meshcache_count() == 2);
    geo_cache_release(a3);                  /* back to idle */

    /* 4. eviction: shrink budget so a new mesh forces the idle entry out */
    geo_cache_release(b);                   /* tank2 idle too; LRU = [car1, tank2] */
    s_heap_free = 512;                      /* < 1 KB free: next mesh forces eviction */
    void *c = geo_cache_acquire("jeep1");   /* 1 KB -> must evict car1 (oldest) */
    MC_CHECK(c != NULL);
    /* car1 was the LRU head (oldest idle) -> evicted; tank2 + jeep1 remain */
    MC_CHECK(meshcache_count() == 2);
    void *car1_again = geo_cache_acquire("car1");  /* miss now -> reloaded, new ptr */
    MC_CHECK(car1_again != NULL);

done:
    /* Restore real loader + clean cache. */
    meshcache_shutdown();
    s_loader = save_loader;
    s_free   = save_free;
    meshcache_init();
    if (ok) fprintf(stdout, "[meshcache] selftest PASS\n");
    return ok;
}
