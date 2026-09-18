#ifndef I76_WEB_MOVIE_H
#define I76_WEB_MOVIE_H

#include <stdint.h>

/* Browser-facing Smacker decoder bridge. All pointers refer to decoder-owned
 * buffers and remain valid until the next advance/close call. */
int web_movie_open(const char *path);
void web_movie_close(void);
int web_movie_width(void);
int web_movie_height(void);
int web_movie_frame_count(void);
int web_movie_frame_index(void);
double web_movie_frame_ms(void);
uintptr_t web_movie_palette(void);
uintptr_t web_movie_pixels(void);
int web_movie_advance(void);
int web_movie_audio_rate(void);
int web_movie_audio_channels(void);
int web_movie_audio_depth(void);
int web_movie_audio_size(void);
uintptr_t web_movie_audio(void);

#endif
