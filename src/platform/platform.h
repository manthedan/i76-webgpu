#ifndef PLATFORM_H
#define PLATFORM_H

/*
 * platform.h — Platform Hardware Abstraction Layer
 *
 * WHAT THIS IS:
 *   Everything in this file maps to Win32 calls found in Fun_Real_Entry
 *   and the surrounding game code. The goal is that no file outside of
 *   src/platform/ ever calls SDL, Win32, or any OS API directly.
 *
 * ORIGINAL BINARY NOTES:
 *   - Window class name: "Interstate '76 Nitro Pack"  (s_Interstate_'76_Nitro_Pack_004f3618)
 *   - Single-instance guard via FindWindowA() at top of Fun_Real_Entry
 *   - WM_QUIT (0x12) was intercepted manually rather than via normal
 *     PostQuitMessage flow — sets DAT_00653c18=1, DAT_00526a44=wParam
 *   - Message pump pattern repeated 5+ times in Fun_Real_Entry alone —
 *     we unify that into platform_pump_events()
 *   - Process priority was toggled around AI computation:
 *     HIGH_PRIORITY_CLASS (0x100) during AI, NORMAL (0x20) after.
 *     On modern hardware this is irrelevant, stubbed out for now.
 *   - Window creation used WNDCLASSA with:
 *       style       = CS_HREDRAW|CS_VREDRAW (3)
 *       cbWndExtra  = 4
 *       hCursor     = IDC_ARROW (0x7F00)
 *       hbrBackground = GetStockObject(BLACK_BRUSH) (4)
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Forward-declare SDL_Window so callers don't need to include SDL headers. */
struct SDL_Window;

/* ------------------------------------------------------------------ */
/* Types                                                                */
/* ------------------------------------------------------------------ */

/*
 * Platform event — what the game loop actually cares about.
 * We don't expose raw SDL/Win32 events; just what the game needs.
 */
typedef enum {
    PLATFORM_EVENT_NONE,
    PLATFORM_EVENT_QUIT,        /* WM_QUIT equivalent */
    PLATFORM_EVENT_KEY_DOWN,
    PLATFORM_EVENT_KEY_UP,
} PlatformEventType;

/*
 * Platform-neutral key codes.
 *
 * The original game read raw Win32 VK_ codes; we don't want SDL_Keycode (or
 * any OS key enum) leaking past the HAL boundary, so platform_sdl.c translates
 * its native keys into these. Add entries here as the game needs more keys —
 * everything above the HAL refers to keys only by PlatformKey.
 */
typedef enum {
    PK_UNKNOWN = 0,
    PK_ESCAPE,
    PK_RETURN,
    PK_SPACE,
    PK_UP, PK_DOWN, PK_LEFT, PK_RIGHT,
    PK_W, PK_A, PK_S, PK_D,
    PK_P, PK_X,
    PK_1, PK_2, PK_3, PK_4, PK_5, PK_6, PK_7, PK_8,
    PK_G,
    PK_COMMA, PK_PERIOD,
    PK_ALT,
    /* Keypad-cluster glance keys (the original's "Grey*" arrows, bound to
     * pilot_glance_* in the stock input.map), plus Insert (pilot_glance_
     * target) and the backquote (rearview mirror toggle). SDL2 delivers the
     * navigation-cluster arrows and the numpad arrows as distinct scancodes;
     * the original binds the GREY (dedicated/numpad-cluster) set, so the
     * main arrow block stays on driving. */
    PK_KP_LEFT, PK_KP_RIGHT, PK_KP_UP, PK_KP_DOWN,
    PK_INSERT,
    PK_BACKQUOTE,
    /* GreyPageUp / GreyPageDown / GreyEnd of the stock input.map: shared
     * by track_distance_-/+, overview_zoom_-/+ and zoom_factor_-/+ (and
     * zoom_factor_reset on End), resolved per active drive view by the
     * camera code. Same numpad-cluster reading as the glance keys above:
     * KP_9/KP_3/KP_1 (plus the dedicated PageUp/PageDown/End, which no
     * other binding claims). */
    PK_KP_PGUP, PK_KP_PGDN, PK_KP_END,
    PK_L,                         /* stock weapon_link (appended; keep
                                   * prior ordinals stable for web PK map) */
    /* show_map / show_notepad (stock keyboard.map: M / N). Appended after
     * PK_L so existing PlatformKey ordinals used by the page and probes
     * stay put. */
    PK_M, PK_N,
    PK_COUNT
} PlatformKey;

typedef struct {
    PlatformEventType type;
    PlatformKey       key;      /* PK_UNKNOWN for keys we don't translate */
} PlatformEvent;

/* ------------------------------------------------------------------ */
/* Lifecycle                                                            */
/* ------------------------------------------------------------------ */

/*
 * platform_init()
 *   Bring up the window and SDL subsystems.
 *   Mirrors RegisterClassA + CreateWindowA + ShowWindow sequence
 *   in Fun_Real_Entry around 0x004339a0.
 *
 *   title  — window title (original: "Interstate '76 Nitro Pack")
 *   width  — client area width  in pixels
 *   height — client area height in pixels
 *
 *   Returns true on success, false on failure (caller should exit).
 */
/*
 * platform_get_window()
 *   Return the SDL_Window* created by platform_init().
 *   Used by render_vk.c to create the Vulkan surface via SDL_Vulkan_*.
 *   Returns NULL before platform_init() succeeds.
 */
struct SDL_Window *platform_get_window(void);

bool platform_init(const char *title, int width, int height);

/*
 * platform_shutdown()
 *   Tears down window and SDL. Mirrors the cleanup block in Fun_Real_Entry
 *   at the GS_QUIT / local_3c==-1 exit path:
 *     FUN_00484af0(&DAT_00653b40)  — destroy window
 *     FreeLibrary(DAT_00653f78)    — drop NITSHELL.DLL
 */
void platform_shutdown(void);

/* ------------------------------------------------------------------ */
/* Event loop                                                           */
/* ------------------------------------------------------------------ */

/*
 * platform_pump_events()
 *   Non-blocking event pump. Call once per frame at the top of the
 *   game loop. Fills *evt with the first relevant event found, or
 *   PLATFORM_EVENT_NONE if the queue is empty.
 *
 *   Returns false if a quit event was received (caller should set
 *   g_gamestate = GS_QUIT).
 *
 *   Original: PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE) pattern,
 *   repeated at addresses: 0x00431xxx, 0x00432xxx, 0x00433xxx etc.
 *   The original explicitly checked message != WM_QUIT (0x12) before
 *   dispatching; we handle that here transparently.
 */
bool platform_pump_events(PlatformEvent *evt);

/* ------------------------------------------------------------------ */
/* Timing                                                               */
/* ------------------------------------------------------------------ */

/*
 * platform_get_ticks()
 *   Milliseconds since platform_init(). Used for the game timer
 *   referenced via FUN_004a2ce0() throughout Fun_Real_Entry —
 *   that function almost certainly wraps timeGetTime() or
 *   QueryPerformanceCounter().
 */
uint64_t platform_get_ticks(void);

/*
 * platform_sleep()
 *   Yield CPU for approximately ms milliseconds.
 *   Used to cap frame rate when the game is idle.
 */
void platform_sleep(uint32_t ms);

/* ------------------------------------------------------------------ */
/* Memory                                                               */
/* ------------------------------------------------------------------ */

/*
 * platform_valloc() / platform_vfree()
 *   Large virtual memory blocks. In the original:
 *     DAT_00653fa0 = FUN_004a3c00(0x1a5e0, 0x40000)  — ~107KB, 256KB reserved
 *     DAT_00653fa4 = FUN_004a3c00(0x80000, 0x80000)  — 512KB
 *   FUN_004a3c00 is almost certainly VirtualAlloc(NULL, reserve,
 *   MEM_COMMIT|MEM_RESERVE, PAGE_READWRITE).
 *
 *   On Linux we use mmap(MAP_ANONYMOUS|MAP_PRIVATE).
 *   Returns NULL on failure (caller mirrors original fatal error path).
 */
void *platform_valloc(size_t size);
void  platform_vfree(void *ptr, size_t size);

#endif /* PLATFORM_H */
