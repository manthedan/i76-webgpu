/* movie.c — client-side playback bridge for user-supplied Smacker files. */

#include "movie.h"

#include "engine/vfs.h"
#include "smacker.h"

#include <limits.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
#define EMSCRIPTEN_KEEPALIVE
#endif

static smk s_movie;
static unsigned long s_width;
static unsigned long s_height;
static unsigned long s_frames;
static double s_frame_us;
static unsigned char s_audio_track = 0xff;
static unsigned char s_audio_channels[7];
static unsigned char s_audio_depth[7];
static unsigned long s_audio_rate[7];

EMSCRIPTEN_KEEPALIVE
void web_movie_close(void)
{
    if (s_movie) smk_close(s_movie);
    s_movie = NULL;
    s_width = s_height = s_frames = 0;
    s_frame_us = 0.0;
    s_audio_track = 0xff;
    memset(s_audio_channels, 0, sizeof s_audio_channels);
    memset(s_audio_depth, 0, sizeof s_audio_depth);
    memset(s_audio_rate, 0, sizeof s_audio_rate);
}

EMSCRIPTEN_KEEPALIVE
int web_movie_open(const char *path)
{
    web_movie_close();
    if (!path || !path[0]) return -1;

    size_t size = 0;
    uint8_t *bytes = vfs_read_file(path, &size);
    if (!bytes || size == 0 || size > ULONG_MAX) {
        vfs_free(bytes);
        return -2;
    }

    s_movie = smk_open_memory(bytes, (unsigned long)size);
    vfs_free(bytes);
    if (!s_movie) return -3;

    unsigned char track_mask = 0;
    unsigned char y_scale = 0;
    if (smk_info_all(s_movie, NULL, &s_frames, &s_frame_us) != 0 ||
        smk_info_video(s_movie, &s_width, &s_height, &y_scale) != 0 ||
        smk_info_audio(s_movie, &track_mask, s_audio_channels,
                       s_audio_depth, s_audio_rate) != 0 ||
        s_width == 0 || s_height == 0 || s_width > 4096 || s_height > 4096 ||
        s_frames == 0 || s_frame_us <= 0.0) {
        web_movie_close();
        return -4;
    }

    unsigned char enabled = SMK_VIDEO_TRACK;
    for (unsigned char i = 0; i < 7; i++) {
        if (track_mask & (1u << i)) {
            s_audio_track = i;
            enabled |= (unsigned char)(1u << i);
            break;
        }
    }
    if (smk_enable_all(s_movie, enabled) != 0 || smk_first(s_movie) < 0) {
        web_movie_close();
        return -5;
    }
    return 0;
}

EMSCRIPTEN_KEEPALIVE
int web_movie_width(void) { return (int)s_width; }

EMSCRIPTEN_KEEPALIVE
int web_movie_height(void) { return (int)s_height; }

EMSCRIPTEN_KEEPALIVE
int web_movie_frame_count(void) { return (int)s_frames; }

EMSCRIPTEN_KEEPALIVE
int web_movie_frame_index(void)
{
    unsigned long frame = 0;
    if (!s_movie || smk_info_all(s_movie, &frame, NULL, NULL) != 0) return -1;
    return (int)frame;
}

EMSCRIPTEN_KEEPALIVE
double web_movie_frame_ms(void) { return s_frame_us / 1000.0; }

EMSCRIPTEN_KEEPALIVE
uintptr_t web_movie_palette(void)
{
    return s_movie ? (uintptr_t)smk_get_palette(s_movie) : 0;
}

EMSCRIPTEN_KEEPALIVE
uintptr_t web_movie_pixels(void)
{
    return s_movie ? (uintptr_t)smk_get_video(s_movie) : 0;
}

EMSCRIPTEN_KEEPALIVE
int web_movie_advance(void) { return s_movie ? smk_next(s_movie) : -1; }

EMSCRIPTEN_KEEPALIVE
int web_movie_audio_rate(void)
{
    return s_audio_track < 7 ? (int)s_audio_rate[s_audio_track] : 0;
}

EMSCRIPTEN_KEEPALIVE
int web_movie_audio_channels(void)
{
    return s_audio_track < 7 ? s_audio_channels[s_audio_track] : 0;
}

EMSCRIPTEN_KEEPALIVE
int web_movie_audio_depth(void)
{
    return s_audio_track < 7 ? s_audio_depth[s_audio_track] : 0;
}

EMSCRIPTEN_KEEPALIVE
int web_movie_audio_size(void)
{
    return s_movie && s_audio_track < 7
         ? (int)smk_get_audio_size(s_movie, s_audio_track) : 0;
}

EMSCRIPTEN_KEEPALIVE
uintptr_t web_movie_audio(void)
{
    return s_movie && s_audio_track < 7
         ? (uintptr_t)smk_get_audio(s_movie, s_audio_track) : 0;
}
