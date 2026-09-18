#ifndef SOUND_H
#define SOUND_H

/*
 * sound.h — M5 SFX registry + mixing state (platform-agnostic).
 *
 * The corpus (docs/specs/m5/audio-inventory.md): 1000 wavs in nitro.zfs.
 * They are RIFF WAVE PCM mono after ZFS-level LZO decompression (already
 * handled by zfs.c): 999 are 8-bit unsigned at 11025 Hz and 01tnk01.wav is
 * 16-bit signed at 11025 Hz. 123 engine/effect sounds use .gpw: 120 are
 * 8-bit/11025 Hz and 3 are 8-bit/22050 Hz. One (`wmgr2.gpw`) has a malformed
 * trailing LIST chunk after its complete PCM data; the parser stops once the
 * required fmt/data chunks are bounded. The cache normalizes accepted 8/16-bit
 * input to unsigned 8-bit PCM. The
 * .gpw files — the engine,
 * horn and ignition sounds referenced by engsnd.dat — are stored under a
 * .gpw extension with a 0x1c-byte "GAS0" wrapper (magic + 6 descriptor
 * dwords, verified on einp1.gpw) before the RIFF header; engsnd.dat
 * still calls them ".wav". sound_load() therefore resolves the name
 * as given first, then retries with the .wav<->.gpw extension swapped.
 *
 * This module tracks WHAT is playing; actual audio output lives on the
 * page (web/audio.js, WebAudio) for the wasm build. State:
 *   - up to SOUND_ONESHOT_MAX concurrent one-shots (sound_play/stop),
 *     each stamped with a monotonically increasing id so the page can
 *     diff its live WebAudio sources against sound_tick_json();
 *   - one engine loop (sound_engine_set/update) with pitch/gain factors.
 *
 * Time is caller-supplied (sound_set_time) so the module stays free of
 * platform clock dependencies and the native probe can drive it
 * deterministically. One-shots auto-expire after pcm_len/rate ms.
 */

#include <stddef.h>
#include <stdint.h>

#define SOUND_ONESHOT_MAX 8
#define SOUND_CACHE_MAX   48

/* ------------------------------------------------------------------ */
/* Wav registry                                                        */
/* ------------------------------------------------------------------ */

/*
 * Load (or return cached) the named wav through the VFS. Parses an optional
 * GAS0 wrapper, then bounded RIFF/WAVE fmt+data chunks; PCM (format 1), mono,
 * 8-bit or 16-bit at an authored 11025/22050 Hz rate is accepted. Keeps
 * unsigned 8-bit normalized PCM. Returns
 * the normalized PCM byte count, or 0 when missing/invalid.
 */
int sound_load(const char *name);

/* Decoded PCM bytes for a previously loaded sound (loads on demand).
 * *len receives the byte count. Returns NULL when unavailable. */
const uint8_t *sound_pcm(const char *name, size_t *len);

/* Sample rate of a loaded sound (0 when missing). Output is mono/8-bit. */
uint32_t sound_rate(const char *name);

/* ------------------------------------------------------------------ */
/* One-shot mixing state                                               */
/* ------------------------------------------------------------------ */

/* Start a one-shot (loads on demand). Returns its id (>0), 0 when the
 * wav is unavailable. When all SOUND_ONESHOT_MAX slots are busy the
 * oldest one-shot is stolen. */
uint32_t sound_play(const char *name);

/* Stop every one-shot playing this name. Returns the count stopped. */
int sound_stop(const char *name);

/* Advance the module clock (ms, monotonic) and expire one-shots whose
 * duration elapsed. The web side passes emscripten_get_now(). */
void sound_set_time(uint32_t now_ms);

/* Number of currently-active one-shot slots. */
int sound_playing_count(void);

/*
 * Serialize the playing state for the page to sync against:
 *   {"ones":[[<id>,"<name>"],...],"eng":["<wav>",<pitch>,<gain>,<0|1>]]}
 * ("eng" is [] when no engine wav was selected yet.) Always fits in
 * the caller's buffer; 512 bytes is ample for 8 one-shots + engine.
 */
void sound_tick_json(char *buf, size_t n);

/* ------------------------------------------------------------------ */
/* Engine loop                                                         */
/* ------------------------------------------------------------------ */

/*
 * DECISION pitch/gain curve (v1, documented — no original curve data
 * recovered): pitch = 0.8 + rpm/6000 (0.8 at idle .. 1.8 at the 6000
 * redline), gain = 0.3 + 0.7*load (0.3 idle whisper .. 1.0 at full
 * throttle). rpm is clamped >= 0, load to [0,1]. Both are strictly
 * monotonic. The loop becomes audible once a wav was selected.
 */
void sound_engine_update(double rpm, double load);
/* Deactivate the engine loop (mission exit); the next update re-arms. */
void sound_engine_stop(void);

/* Select (and load) the engine loop wav. Returns 1 on success. */
int  sound_engine_set(const char *name);

/* Current engine wav name ("" when none), pitch and gain factors. */
const char *sound_engine_wav(void);
double sound_engine_pitch(void);
double sound_engine_gain(void);

/* Silence the engine loop (one-shots unaffected). */
void sound_engine_off(void);

/* ------------------------------------------------------------------ */
/* engsnd.dat — engine-configuration -> wav wiring table               */
/* ------------------------------------------------------------------ */

typedef struct {
    int     eng_num;        /* ENG NUM (0-5 = player engine tiers)   */
    int     comp_id;        /* ENG COMP ID (engine component id)     */
    int     revable;        /* REVABLE FLAG                          */
    char    engine_wav[16]; /* ENGINE SND FILE ("NONE" = silent)     */
    char    horn_wav[16];   /* HORN SND FILE                         */
    char    ign_wav[3][16]; /* IGN NORM/DMG1/DMG2 SND                */
    double  ign_time[3];    /* matching IGN times (seconds)          */
} SoundEngRow;

/* Parse engsnd.dat through the VFS (idempotent). Returns the row
 * count, 0 when unavailable/unparseable. */
int sound_engsnd_load(void);

/* Row count after sound_engsnd_load (0 before). */
int sound_engsnd_rows(void);

/* Row i (NULL out of range). Valid until the next sound_engsnd_load. */
const SoundEngRow *sound_engsnd_row(int i);

/* Engine wav for an ENG NUM / first row matching an ENG COMP ID.
 * NULL when no row matches or the row's file is "NONE". */
const char *sound_engsnd_wav(int eng_num);
const char *sound_engsnd_comp_wav(int comp_id);

/*
 * Mission-boundary teardown. Drops every decoded WAV plus active one-shots
 * and the selected engine loop; engsnd.dat's immutable wiring table remains
 * parsed for lazy reuse by the next mission. This is a full per-mission flush,
 * not an LRU (PORT DECISION; docs/specs/m5/audio-inventory.md §1.1).
 */
void sound_mission_reset(void);

/* Diagnostic count of loads denied specifically because all 48 cache slots
 * were occupied. Mission resets deliberately preserve it so a sequential-
 * mission gate can detect any denial across the whole session. */
uint32_t sound_cache_full_failures(void);

/* Drop all cached sounds, playback, table state, and diagnostics. */
void sound_reset(void);

#endif /* SOUND_H */
