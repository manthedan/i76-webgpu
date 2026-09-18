/*
 * sound.c — M5 SFX registry + mixing state. See sound.h for the model.
 *
 * RIFF parse per docs/specs/m5/audio-inventory.md §1: after ZFS-level
 * LZO decompression the authored sounds are RIFF/WAVE PCM mono. Most are
 * 8-bit unsigned; 01tnk01.wav is 16-bit signed. The cache normalizes both
 * to 8-bit unsigned PCM (0x80 = silence) for the existing page contract.
 * Files stored as .gpw
 * carry a 0x1c-byte GAS0 wrapper (verified: "GAS0" + u32 0x4b + five
 * 0xffffffff dwords, then RIFF at offset 0x1c) which we skip.
 *
 * Native build: see tools/sound_probe.c header.
 */

#include "sound.h"
#include "vfs.h"

#include <limits.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

/* ------------------------------------------------------------------ */
/* Wav registry                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    char     name[24];   /* registry key: requested name, lowercased */
    uint8_t *pcm;        /* owned copy of the data-chunk payload     */
    uint32_t len;
    uint32_t rate;
    int      loaded;     /* 1 = pcm valid; -1 = known-bad (negative cache) */
} SoundEntry;

static SoundEntry s_cache[SOUND_CACHE_MAX];
static uint32_t   s_cache_full_failures;

static void sound_key(const char *name, char out[24])
{
    size_t i;
    for (i = 0; i < 23 && name[i]; i++) {
        char c = name[i];
        if (c >= 'A' && c <= 'Z') c = (char)(c + ('a' - 'A'));
        out[i] = c;
    }
    out[i] = '\0';
}

static SoundEntry *cache_find(const char *key)
{
    for (int i = 0; i < SOUND_CACHE_MAX; i++)
        if (s_cache[i].loaded && strcmp(s_cache[i].name, key) == 0)
            return &s_cache[i];
    return NULL;
}

/* Little-endian readers (the file format deserves explicit decoding). */
static uint16_t rd_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}
static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/*
 * Parse buf as [GAS0] RIFF/WAVE PCM mono. On success fills the raw data
 * location, format, and rate. Chunk bounds are limited by both the supplied
 * buffer and the RIFF container; authored odd-sized files may carry their pad
 * byte immediately after the declared RIFF extent.
 */
static int wav_parse(const uint8_t *buf, size_t sz, size_t *pcm_off,
                     uint32_t *pcm_len, uint32_t *rate, uint16_t *bits_out)
{
    size_t off = 0;

    /* Optional GAS0 wrapper: magic + 6 descriptor dwords = 0x1c bytes. */
    if (sz >= 0x1c && memcmp(buf, "GAS0", 4) == 0)
        off = 0x1c;

    if (sz < off + 12 || memcmp(buf + off, "RIFF", 4) != 0 ||
        memcmp(buf + off + 8, "WAVE", 4) != 0)
        return 0;

    uint32_t riff_size = rd_u32(buf + off + 4);
    if (riff_size < 4 || (size_t)riff_size > sz - off - 8)
        return 0;
    size_t riff_end = off + 8 + (size_t)riff_size;

    int fmt_ok = 0, data_ok = 0;
    uint32_t rate_seen = 0;
    uint16_t bits_seen = 0;
    size_t pos = off + 12;
    while (pos <= riff_end && riff_end - pos >= 8 && !(fmt_ok && data_ok)) {
        uint32_t csize = rd_u32(buf + pos + 4);
        if ((size_t)csize > riff_end - pos - 8) return 0;
        if (memcmp(buf + pos, "fmt ", 4) == 0 && csize >= 16) {
            uint16_t format     = rd_u16(buf + pos + 8);
            uint16_t channels   = rd_u16(buf + pos + 10);
            uint16_t block_align = rd_u16(buf + pos + 20);
            uint32_t byte_rate  = rd_u32(buf + pos + 16);
            bits_seen = rd_u16(buf + pos + 22);
            rate_seen = rd_u32(buf + pos + 12);
            uint16_t sample_bytes = (uint16_t)(bits_seen / 8);
            fmt_ok = format == 1 && channels == 1 &&
                     (bits_seen == 8 || bits_seen == 16) &&
                     (rate_seen == 11025 || rate_seen == 22050) &&
                     block_align == sample_bytes &&
                     rate_seen <= UINT32_MAX / block_align &&
                     byte_rate == rate_seen * block_align;
        } else if (memcmp(buf + pos, "data", 4) == 0 && csize > 0) {
            *pcm_off = pos + 8;
            *pcm_len = csize;
            data_ok = 1;
        }
        size_t next = pos + 8 + (size_t)csize;
        pos = next + ((csize & 1) && next < riff_end ? 1 : 0);
    }

    if (!fmt_ok || !data_ok || (*pcm_len % (bits_seen / 8)) != 0)
        return 0;
    *rate = rate_seen;
    *bits_out = bits_seen;
    return 1;
}

/* vfs_read_file with the .wav<->.gpw extension swap (engsnd.dat says
 * ".wav" for sounds the archive stores as GAS0-wrapped ".gpw"). */
static void *read_vfs_swapped(const char *name, size_t *size_out)
{
    /* probe first: the swap makes a first-miss the normal case, and
     * vfs_read_file logs every miss — only a both-names miss is real */
    if (vfs_exists(name)) return vfs_read_file(name, size_out);

    char alt[24];
    const char *dot = strrchr(name, '.');
    if (!dot) goto miss;
    size_t stem = (size_t)(dot - name);
    if (stem + 5 >= sizeof alt) goto miss;
    memcpy(alt, name, stem);
    if (strcasecmp(dot, ".wav") == 0)
        snprintf(alt + stem, sizeof alt - stem, ".gpw");
    else if (strcasecmp(dot, ".gpw") == 0)
        snprintf(alt + stem, sizeof alt - stem, ".wav");
    else
        goto miss;
    if (vfs_exists(alt)) return vfs_read_file(alt, size_out);

miss:
    fprintf(stderr, "[sound] Not found: %s\n", name);
    return NULL;
}

int sound_load(const char *name)
{
    if (!name || !*name) return 0;

    char key[24];
    sound_key(name, key);
    SoundEntry *e = cache_find(key);
    if (e) return e->loaded > 0 ? (int)e->len : 0;

    e = NULL;
    for (int i = 0; i < SOUND_CACHE_MAX; i++)
        if (!s_cache[i].loaded) { e = &s_cache[i]; break; }
    if (!e) {
        s_cache_full_failures++;
        fprintf(stderr, "[sound] cache full (%d), cannot load %s\n",
                SOUND_CACHE_MAX, name);
        return 0;
    }

    size_t sz = 0;
    uint8_t *buf = read_vfs_swapped(name, &sz);
    if (!buf) {
        fprintf(stderr, "[sound] not found: %s\n", name);
        return 0;    /* not negatively cached: page may stage it later */
    }

    size_t pcm_off = 0;
    uint32_t pcm_len = 0, rate = 0;
    uint16_t bits = 0;
    if (!wav_parse(buf, sz, &pcm_off, &pcm_len, &rate, &bits)) {
        fprintf(stderr, "[sound] unsupported/malformed PCM mono RIFF: %s\n",
                name);
        vfs_free(buf);
        snprintf(e->name, sizeof e->name, "%s", key);
        e->pcm = NULL;
        e->len = 0;
        e->loaded = -1;
        return 0;
    }

    uint32_t out_len = pcm_len / (bits / 8);
    if (out_len > INT_MAX) {
        fprintf(stderr, "[sound] PCM payload too large: %s\n", name);
        vfs_free(buf);
        return 0;
    }
    uint8_t *pcm = malloc(out_len);
    if (!pcm) { vfs_free(buf); return 0; }
    if (bits == 8) {
        memcpy(pcm, buf + pcm_off, out_len);
    } else {
        /* Signed little-endian 16-bit -> unsigned 8-bit. Taking the high
         * byte preserves zero/silence exactly and cannot overflow. */
        for (uint32_t i = 0; i < out_len; i++)
            pcm[i] = buf[pcm_off + (size_t)i * 2 + 1] ^ 0x80;
    }
    vfs_free(buf);

    snprintf(e->name, sizeof e->name, "%s", key);
    e->pcm  = pcm;
    e->len  = out_len;
    e->rate = rate;
    e->loaded = 1;
    return (int)out_len;
}

const uint8_t *sound_pcm(const char *name, size_t *len)
{
    if (!name || !*name) return NULL;
    char key[24];
    sound_key(name, key);
    SoundEntry *e = cache_find(key);
    if (!e) {
        if (sound_load(name) <= 0) return NULL;
        e = cache_find(key);
        if (!e) return NULL;
    }
    if (e->loaded <= 0) return NULL;
    if (len) *len = e->len;
    return e->pcm;
}

uint32_t sound_rate(const char *name)
{
    char key[24];
    sound_key(name, key);
    SoundEntry *e = cache_find(key);
    return (e && e->loaded > 0) ? e->rate : 0;
}

/* ------------------------------------------------------------------ */
/* One-shot mixing state                                               */
/* ------------------------------------------------------------------ */

typedef struct {
    uint32_t id;
    char     name[24];
    uint32_t start_ms;
    uint32_t dur_ms;
    int      active;
} OneShot;

static OneShot  s_shots[SOUND_ONESHOT_MAX];
static uint32_t s_next_id = 1;
static uint32_t s_now_ms;

uint32_t sound_play(const char *name)
{
    if (sound_load(name) <= 0) return 0;

    char key[24];
    sound_key(name, key);
    SoundEntry *e = cache_find(key);
    if (!e || e->loaded <= 0) return 0;

    int slot = -1;
    for (int i = 0; i < SOUND_ONESHOT_MAX; i++)
        if (!s_shots[i].active) { slot = i; break; }
    if (slot < 0) {
        /* Steal the oldest (lowest id). */
        slot = 0;
        for (int i = 1; i < SOUND_ONESHOT_MAX; i++)
            if (s_shots[i].id < s_shots[slot].id) slot = i;
    }

    OneShot *s = &s_shots[slot];
    s->id       = s_next_id++;
    if (s_next_id == 0) s_next_id = 1;      /* 0 = invalid id */
    snprintf(s->name, sizeof s->name, "%s", key);
    s->start_ms = s_now_ms;
    s->dur_ms   = (uint32_t)(((uint64_t)e->len * 1000) / e->rate);
    s->active   = 1;
    return s->id;
}

int sound_stop(const char *name)
{
    char key[24];
    sound_key(name, key);
    int n = 0;
    for (int i = 0; i < SOUND_ONESHOT_MAX; i++)
        if (s_shots[i].active && strcmp(s_shots[i].name, key) == 0) {
            s_shots[i].active = 0;
            n++;
        }
    return n;
}

void sound_set_time(uint32_t now_ms)
{
    s_now_ms = now_ms;
    for (int i = 0; i < SOUND_ONESHOT_MAX; i++)
        if (s_shots[i].active &&
            (uint32_t)(now_ms - s_shots[i].start_ms) >= s_shots[i].dur_ms)
            s_shots[i].active = 0;
}

int sound_playing_count(void)
{
    int n = 0;
    for (int i = 0; i < SOUND_ONESHOT_MAX; i++)
        if (s_shots[i].active) n++;
    return n;
}

/* ------------------------------------------------------------------ */
/* Engine loop                                                         */
/* ------------------------------------------------------------------ */

static struct {
    int    active;
    char   wav[24];
    double pitch;
    double gain;
} s_engine;

int sound_engine_set(const char *name)
{
    if (!name || sound_load(name) <= 0) return 0;
    sound_key(name, s_engine.wav);
    return 1;
}

void sound_engine_update(double rpm, double load)
{
    /* DECISION curve (sound.h): pitch 0.8 + rpm/6000, gain 0.3+0.7*load. */
    if (rpm < 0.0) rpm = 0.0;
    if (load < 0.0) load = 0.0;
    else if (load > 1.0) load = 1.0;
    s_engine.pitch  = 0.8 + rpm / 6000.0;
    s_engine.gain   = 0.3 + 0.7 * load;
    s_engine.active = s_engine.wav[0] != '\0';
}

/* H-UAT-038: active latches on once a wav is set (rpm 0 only idles the
 * pitch), so leaving the drive shell must deactivate explicitly or the
 * loop outlives the mission into the menus. */
void sound_engine_stop(void)
{
    s_engine.active = 0;
}

const char *sound_engine_wav(void)   { return s_engine.wav; }
double sound_engine_pitch(void)      { return s_engine.pitch; }
double sound_engine_gain(void)       { return s_engine.gain; }
void   sound_engine_off(void)        { s_engine.active = 0; }

/* ------------------------------------------------------------------ */
/* Playing-state JSON                                                  */
/* ------------------------------------------------------------------ */

/* snprintf truncates safely, but its return is the would-have-written
 * length — clamp the cursor so the remaining size never underflows. */
static void tj_cat(char *buf, size_t n, size_t *w, const char *fmt, ...)
{
    if (*w >= n) return;
    va_list ap;
    va_start(ap, fmt);
    int r = vsnprintf(buf + *w, n - *w, fmt, ap);
    va_end(ap);
    if (r < 0) return;
    *w += (size_t)r;
    if (*w >= n) *w = n - 1;
}

void sound_tick_json(char *buf, size_t n)
{
    if (n == 0) return;
    size_t w = 0;
    tj_cat(buf, n, &w, "{\"ones\":[");
    int first = 1;
    for (int i = 0; i < SOUND_ONESHOT_MAX; i++) {
        if (!s_shots[i].active) continue;
        tj_cat(buf, n, &w, "%s[%u,\"%s\"]",
               first ? "" : ",", s_shots[i].id, s_shots[i].name);
        first = 0;
    }
    if (s_engine.wav[0])
        tj_cat(buf, n, &w, "],\"eng\":[\"%s\",%.3f,%.3f,%d]}",
               s_engine.wav, s_engine.pitch, s_engine.gain,
               s_engine.active);
    else
        tj_cat(buf, n, &w, "],\"eng\":[]}");
}

/* ------------------------------------------------------------------ */
/* engsnd.dat                                                          */
/* ------------------------------------------------------------------ */

/*
 * Format (verified against the shipped file, 12 rows):
 *   # comments start with '#'
 *   ENG NUM  ENG COMP ID  REVABLE FLAG  ENGINE SND  HORN SND
 *   IGN NORM SND  IGN NORM TIME  IGN DMG1 SND  IGN DMG1 TIME
 *   IGN DMG2 SND  IGN DMG2 TIME
 * e.g.:
 *    0 0 1 einp1.wav  vhorn1.wav esnp1.wav 0.4 esnp2.wav 1.2 esnp3.wav 4.0
 *   20 0 0 aheli.wav  vhorn6.wav esnp1.wav 0.4 esnp2.wav 1.2 esnp3.wav 4.0
 * "NONE" marks a slot with no sound (row 99 is all-NONE).
 */

#define ENGSND_MAX_ROWS 32

static SoundEngRow s_rows[ENGSND_MAX_ROWS];
static int         s_row_count;
static int         s_engsnd_tried;

static int is_none(const char *s) { return strcasecmp(s, "NONE") == 0; }

int sound_engsnd_load(void)
{
    if (s_engsnd_tried) return s_row_count;
    s_engsnd_tried = 1;

    size_t sz = 0;
    char *buf = vfs_read_file("engsnd.dat", &sz);
    if (!buf) {
        fprintf(stderr, "[sound] engsnd.dat not found\n");
        return 0;
    }

    size_t pos = 0;
    while (pos < sz && s_row_count < ENGSND_MAX_ROWS) {
        size_t eol = pos;
        while (eol < sz && buf[eol] != '\n') eol++;
        size_t len = eol - pos;
        if (len > 0 && buf[pos + len - 1] == '\r') len--;

        char line[192];
        if (len >= sizeof line) len = sizeof line - 1;
        memcpy(line, buf + pos, len);
        line[len] = '\0';
        pos = eol + 1;

        char *p = line;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0' || *p == '#') continue;

        SoundEngRow r;
        int got = sscanf(p, "%d %d %d %15s %15s %15s %lf %15s %lf %15s %lf",
                         &r.eng_num, &r.comp_id, &r.revable,
                         r.engine_wav, r.horn_wav,
                         r.ign_wav[0], &r.ign_time[0],
                         r.ign_wav[1], &r.ign_time[1],
                         r.ign_wav[2], &r.ign_time[2]);
        if (got != 11) {
            fprintf(stderr,
                    "[sound] engsnd.dat: skipped bad row (%d fields): %s\n",
                    got, line);
            continue;
        }
        s_rows[s_row_count++] = r;
    }
    vfs_free(buf);
    return s_row_count;
}

int sound_engsnd_rows(void) { return s_row_count; }

const SoundEngRow *sound_engsnd_row(int i)
{
    if (i < 0 || i >= s_row_count) return NULL;
    return &s_rows[i];
}

const char *sound_engsnd_wav(int eng_num)
{
    for (int i = 0; i < s_row_count; i++)
        if (s_rows[i].eng_num == eng_num)
            return is_none(s_rows[i].engine_wav) ? NULL : s_rows[i].engine_wav;
    return NULL;
}

const char *sound_engsnd_comp_wav(int comp_id)
{
    for (int i = 0; i < s_row_count; i++)
        if (s_rows[i].comp_id == comp_id)
            return is_none(s_rows[i].engine_wav) ? NULL : s_rows[i].engine_wav;
    return NULL;
}

/* ------------------------------------------------------------------ */

void sound_mission_reset(void)
{
    for (int i = 0; i < SOUND_CACHE_MAX; i++) {
        if (s_cache[i].loaded > 0) free(s_cache[i].pcm);
        memset(&s_cache[i], 0, sizeof s_cache[i]);
    }
    memset(s_shots, 0, sizeof s_shots);
    memset(&s_engine, 0, sizeof s_engine);
    s_now_ms = 0;
}

uint32_t sound_cache_full_failures(void)
{
    return s_cache_full_failures;
}

void sound_reset(void)
{
    sound_mission_reset();
    s_row_count = 0;
    s_engsnd_tried = 0;
    s_cache_full_failures = 0;
}
