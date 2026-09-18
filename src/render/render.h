#ifndef RENDER_H
#define RENDER_H

/*
 * render.h — Render abstraction layer
 *
 * Replaces NITSHELL.DLL from the original binary.
 *
 * Original exported an opaque function-pointer table populated via
 * GetProcAddress after LoadLibraryA("NITSHELL.DLL"). The known slots were:
 *
 *   DAT_00653f3c  →  BeginFrame(surface*)
 *   DAT_00653f40  →  EndFrame(surface*)
 *   DAT_00653f48  →  SetPalette(surface*, palette*)
 *   DAT_00653f4c  →  Init(surface*, mode)
 *   DAT_00653f60  →  Flip(surface*)
 *
 * We replicate those roles here with a clean C interface. Backend is Vulkan
 * (render_vk.c); D3D12 would be a second implementation of this header.
 *
 * Rendering model:
 *   The game was 8-bit palettized software-rendered at 640×480.
 *   We upload the raw 8-bit index surface as a texture and do palette lookup
 *   on the GPU (see blit.frag). This preserves palette animations and fades
 *   with zero CPU overhead.
 */

#include <stdbool.h>
#include <stdint.h>

/* One RGB palette entry — matches the PCX on-disk format byte for byte. */
typedef struct { uint8_t r, g, b; } Rgb8;

/* -----------------------------------------------------------------------
 * Lifecycle
 * ----------------------------------------------------------------------- */

/*
 * render_init()
 *   Initialise Vulkan: instance, device, swapchain, pipeline.
 *   Must be called after platform_init() (needs the SDL_Window*).
 *   Returns false on failure; caller should treat as fatal.
 */
bool render_init(void);

/*
 * render_shutdown()
 *   Destroy all Vulkan objects in reverse-creation order.
 *   Mirrors the original's FUN_0046dc10 + FreeLibrary(NITSHELL) sequence.
 */
void render_shutdown(void);

/* -----------------------------------------------------------------------
 * Per-frame
 * ----------------------------------------------------------------------- */

/*
 * render_begin_frame()
 *   Acquire the next swapchain image, wait on its in-flight fence.
 *   Call once at the top of the game loop — mirrors DAT_00653f3c (BeginFrame).
 */
void render_begin_frame(void);

/*
 * render_end_frame()
 *   Record + submit the command buffer, present the image.
 *   Mirrors DAT_00653f40 (EndFrame) + DAT_00653f60 (Flip).
 */
void render_end_frame(void);

/* -----------------------------------------------------------------------
 * Palettized blit
 * ----------------------------------------------------------------------- */

/*
 * render_set_palette()
 *   Upload 256 RGB entries to the GPU palette UBO.
 *   Mirrors DAT_00653f48 (SetPalette).
 *   Safe to call every frame; the copy is a 4 KB memcpy to a mapped buffer.
 */
void render_set_palette(const Rgb8 *palette_256);

/*
 * render_blit_indexed()
 *   Upload a width×height 8-bit index surface and draw it fullscreen.
 *   The fragment shader expands indices → RGB via the current palette.
 *   width/height must match the surface the game rendered into.
 */
void render_blit_indexed(const uint8_t *pixels, uint32_t width, uint32_t height);

#endif /* RENDER_H */
