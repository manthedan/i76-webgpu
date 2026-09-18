#ifndef MESHCACHE_H
#define MESHCACHE_H

/*
 * meshcache.h — GEO mesh cache (faithful reimplementation)
 *
 * Reverse-engineered from the original binary; see docs/REVERSING.md
 * "Object / Geometry Mesh Cache". This is a named, refcounted, LRU-evicting
 * cache of decoded GEO meshes — NOT the live game-entity store.
 *
 * Original functions this mirrors:
 *   obj_system_init   (FUN_0043a6d0)  -> meshcache_init
 *   geo_cache_acquire (FUN_0043a770)  -> geo_cache_acquire
 *   geo_cache_release (FUN_0043a9a0)  -> geo_cache_release
 *   geo_build_mesh    (FUN_0043aa60)  -> (internal) decode + eviction
 *
 * WHAT IS CERTAIN (reversed, reproduced faithfully here):
 *   - 2029-bucket dual index: by name and by mesh-buffer pointer
 *   - hash h(k) = (k*0x6cd + 0xaab) % 0x7ed   (= (k*1741 + 2731) % 2029)
 *   - 8-byte uppercased name key; by-name hash folds the two 32-bit halves
 *     (hi ^ lo) & 0xdfdfdfdf before hashing
 *   - 0x20-byte node: name, refcount, mesh ptr, two index chains, LRU links
 *   - refcount==0 => node sits on the LRU free-list and may be evicted
 *   - acquire find-or-load (refcount++); release deref (refcount--, append LRU)
 *   - eviction frees oldest LRU entry first when the heap budget is exceeded
 *
 * WHAT IS NOT YET REVERSED (stubbed, clearly marked in meshcache.c):
 *   - vfs_lod's LOD-suffix file resolution (we read the name directly)
 *   - the GEO byte-level decode in geo_build_mesh (we wrap the raw image)
 */

#include <stdbool.h>
#include <stddef.h>

/* Set up the two indices, the LRU list, and the heap budget. Idempotent. */
void meshcache_init(void);

/* Free every cached mesh + node and reset. */
void meshcache_shutdown(void);

/*
 * Acquire a mesh by name (≤8-char key, case-insensitive). Loads + decodes on a
 * miss, bumps the refcount on a hit. Returns an opaque pointer to the decoded
 * mesh buffer, or NULL if the GEO could not be loaded/decoded.
 *
 * Every successful acquire must be balanced by a geo_cache_release.
 */
void *geo_cache_acquire(const char *name);

/*
 * Release a mesh previously returned by geo_cache_acquire. When its refcount
 * reaches zero the entry stays cached (on the LRU) and becomes evictable.
 */
void geo_cache_release(void *mesh);

/* Debug: bytes of budget remaining, and number of cached entries. */
long meshcache_free_bytes(void);
int  meshcache_count(void);

/*
 * Self-test of the cache mechanics (hash/refcount/LRU/eviction) using a
 * synthetic in-memory loader — no assets required. Returns true on pass.
 * Leaves the cache empty and the real loader restored.
 */
bool meshcache_selftest(void);

#endif /* MESHCACHE_H */
