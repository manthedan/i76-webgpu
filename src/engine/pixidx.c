/*
 * pixidx.c — merged .pix -> .pak virtual-file index. See pixidx.h.
 *
 * STEP 5. Built and tested, but nothing in the render path calls it yet.
 */
#include "engine/pixidx.h"
#include "engine/vfs.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

struct PixIndex {
    PixEnt *ent;
    int     n, cap;
    int     manifests;
    int     dups;          /* keys seen more than once, before dedupe */
    int     dropped;       /* entries discarded: bad bounds, overflow, alloc */
    int     alloc_fail;
};

static void lower(char *s) { for (; *s; s++) if (*s >= 'A' && *s <= 'Z') *s += 32; }

/*
 * TOTAL order: key, then pak, then off.
 *
 * Ordering on the key alone would be enough to FIND an entry, but not enough
 * to decide WHICH entry wins when a key appears twice -- and 11 keys in the m
 * tier genuinely resolve to different paks (gtutf_.vqm is in both gturretm.pak
 * and weapncmm.pak). qsort is not required to be stable, and native and wasm
 * link different qsort implementations, so a key-only comparator would let the
 * winner depend on which libc built the binary. The frame gate would then fail
 * for a reason that has nothing to do with rendering.
 *
 * With a total order the sorted sequence is unique regardless of input order or
 * sort stability, which is exactly the property the reversed-insertion test in
 * tools/pixidx_probe.c checks.
 */
static int ent_cmp(const void *a, const void *b)
{
    const PixEnt *x = a, *y = b;
    int c = strcmp(x->key, y->key);
    if (c) return c;
    c = strcmp(x->pak, y->pak);
    if (c) return c;
    return (x->off > y->off) - (x->off < y->off);
}

static int ent_push(PixIndex *ix, const char *key, const char *pak,
                    uint32_t off, uint32_t len)
{
    if (ix->n == ix->cap) {
        int cap = ix->cap ? ix->cap * 2 : 512;
        PixEnt *p = realloc(ix->ent, (size_t)cap * sizeof *p);
        if (!p) {
            /* COUNTED, not silently swallowed. A partial index that pretends
             * to be complete turns into "that texture just doesn't exist",
             * which is indistinguishable from a real asset problem. */
            ix->alloc_fail++;
            ix->dropped++;
            return -1;
        }
        ix->ent = p;
        ix->cap = cap;
    }
    PixEnt *e = &ix->ent[ix->n++];
    snprintf(e->key, sizeof e->key, "%s", key);
    lower(e->key);
    snprintf(e->pak, sizeof e->pak, "%s", pak);
    lower(e->pak);
    e->off = off;
    e->len = len;
    return 0;
}

/* "<count>\n" then "NAME <off> <len>" lines. */
static void pix_parse(PixIndex *ix, const char *pak, const char *text,
                      size_t len, size_t pak_size)
{
    const char *p = text, *end = text + len;
    while (p < end && *p != '\n') p++;          /* skip the count line */
    if (p < end) p++;

    while (p < end) {
        char nm[64];
        int i = 0;
        while (p < end && (*p == ' ' || *p == '\r' || *p == '\n')) p++;
        while (p < end && *p != ' ' && *p != '\r' && *p != '\n' &&
               i < (int)sizeof nm - 1)
            nm[i++] = *p++;
        nm[i] = '\0';
        if (i == 0) { while (p < end && *p != '\n') p++; if (p < end) p++; continue; }

        unsigned long long off = 0, elen = 0;
        while (p < end && *p == ' ') p++;
        while (p < end && *p >= '0' && *p <= '9') off = off * 10ull + (unsigned)(*p++ - '0');
        while (p < end && *p == ' ') p++;
        while (p < end && *p >= '0' && *p <= '9') elen = elen * 10ull + (unsigned)(*p++ - '0');

        /*
         * Bounds-check in 64-bit. size_t is 32 BITS under wasm, so computing
         * off + len in size_t can wrap and turn an out-of-range slice into an
         * apparently valid one -- a read straight past the end of the pak.
         */
        if (pak_size && (off + elen > (unsigned long long)pak_size ||
                         off > 0xffffffffull || elen > 0xffffffffull)) {
            ix->dropped++;
        } else {
            ent_push(ix, nm, pak, (uint32_t)off, (uint32_t)elen);
        }
        while (p < end && *p != '\n') p++;
        if (p < end) p++;
    }
}

typedef struct {
    PixIndex *ix;
    const char *pattern;
    size_t plen;
    int exact;
} Collect;

static void collect(const char *name, int src_type, void *ud)
{
    (void)src_type;
    Collect *c = ud;
    size_t n = strlen(name);
    if (c->exact) {
        if (strcasecmp(name, c->pattern) != 0) return;
    } else if (n < c->plen ||
               strcasecmp(name + n - c->plen, c->pattern) != 0) {
        return;
    }

    /*
     * <base><tier>.pix -> <base><tier>.pak, LENGTH-GUARDED. scene.c's
     * equivalent does an unguarded memcpy at name+n-3 into a char[16]; the
     * measured maximum .pix name is 12 characters, so it fits with exactly
     * zero margin and a longer name from a Nitro-only or user asset would
     * write past the buffer.
     */
    char pak[16];
    if (n + 1 > sizeof pak) { c->ix->dropped++; return; }
    snprintf(pak, sizeof pak, "%s", name);
    memcpy(pak + n - 3, "pak", 3);
    pak[n] = '\0';

    size_t sz = 0;
    char *buf = vfs_read_file(name, &sz);
    if (!buf) return;

    /* Size the pak so entry bounds can be validated against something real. */
    size_t paksz = 0;
    void *pb = vfs_read_file(pak, &paksz);
    if (pb) vfs_free(pb);

    pix_parse(c->ix, pak, buf, sz, paksz);
    vfs_free(buf);
    c->ix->manifests++;
}

static PixIndex *pixidx_build_match(const char *pattern, int exact)
{
    if (!pattern || !*pattern) return NULL;
    PixIndex *ix = calloc(1, sizeof *ix);
    if (!ix) return NULL;

    Collect c = { ix, pattern, strlen(pattern), exact };
    vfs_foreach(collect, &c);

    if (ix->n > 1) {
        qsort(ix->ent, (size_t)ix->n, sizeof ix->ent[0], ent_cmp);

        /* Dedupe adjacent equal KEYS, keeping the first under the total order.
         * Counted, because a duplicate key means two paks disagree about what
         * a name refers to and somebody may want to know which won. */
        int w = 1;
        for (int i = 1; i < ix->n; i++) {
            if (strcmp(ix->ent[i].key, ix->ent[w - 1].key) == 0) {
                ix->dups++;
                continue;
            }
            ix->ent[w++] = ix->ent[i];
        }
        ix->n = w;
    }
    return ix;
}

PixIndex *pixidx_build(const char *suffix)
{
    return pixidx_build_match(suffix, 0);
}

PixIndex *pixidx_build_exact(const char *manifest)
{
    return pixidx_build_match(manifest, 1);
}

void pixidx_free(PixIndex *ix)
{
    if (!ix) return;
    free(ix->ent);
    free(ix);
}

const PixEnt *pixidx_find(const PixIndex *ix, const char *key)
{
    if (!ix || !ix->n || !key) return NULL;
    char k[16];
    snprintf(k, sizeof k, "%s", key);
    lower(k);

    int lo = 0, hi = ix->n - 1;
    while (lo <= hi) {
        int mid = lo + (hi - lo) / 2;
        int c = strcmp(k, ix->ent[mid].key);
        if (c == 0) return &ix->ent[mid];
        if (c < 0) hi = mid - 1; else lo = mid + 1;
    }
    return NULL;
}

int pixidx_count(const PixIndex *ix) { return ix ? ix->n : 0; }

const PixEnt *pixidx_at(const PixIndex *ix, int i)
{
    if (!ix || i < 0 || i >= ix->n) return NULL;
    return &ix->ent[i];
}

int pixidx_stats(const PixIndex *ix, char *buf, size_t n)
{
    if (!ix) return snprintf(buf, n, "pixidx: none");
    return snprintf(buf, n,
        "pixidx: manifests=%d entries=%d dups=%d dropped=%d alloc_fail=%d",
        ix->manifests, ix->n, ix->dups, ix->dropped, ix->alloc_fail);
}
