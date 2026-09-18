/*
 * input.c — Per-frame input snapshot (see input.h)
 *
 * Held state persists across frames in file-static storage; input_poll()
 * mutates it from the events drained each frame and latches every up-to-down
 * transition, including a complete tap that lands between two 20 Hz polls.
 */

#include "input.h"

#include <math.h>
#include <string.h>

#include "platform/platform.h"

/* Held state, persisted between frames. Updated by key down/up events. */
static bool s_held[INPUT_BTN_COUNT];
/* Rising edges waiting to be delivered by the next snapshot. */
static bool s_pressed[INPUT_BTN_COUNT];
/* Actions consumed by an outgoing input owner. A matching release is the
 * only event that re-arms them for the new owner. */
static bool s_release_required[INPUT_BTN_COUNT];

/*
 * Map a platform key to a game button, or -1 if the key isn't bound.
 * WASD and the arrows alias onto the same drive buttons.
 */
static int key_to_button(PlatformKey key)
{
    switch (key) {
    case PK_UP: case PK_W: return INPUT_BTN_UP;
    case PK_DOWN: case PK_S: return INPUT_BTN_DOWN;
    case PK_LEFT: case PK_A: return INPUT_BTN_LEFT;
    case PK_RIGHT: case PK_D: return INPUT_BTN_RIGHT;
    case PK_SPACE: return INPUT_BTN_FIRE;
    case PK_P: return INPUT_BTN_PAUSE;
    case PK_X: return INPUT_BTN_REVERSE;
    case PK_1: return INPUT_BTN_WEAPON_1;
    case PK_2: return INPUT_BTN_WEAPON_2;
    case PK_3: return INPUT_BTN_WEAPON_3;
    case PK_4: return INPUT_BTN_WEAPON_4;
    case PK_5: return INPUT_BTN_WEAPON_5;
    case PK_6: return INPUT_BTN_WEAPON_6;
    case PK_7: return INPUT_BTN_WEAPON_7;
    case PK_8: return INPUT_BTN_WEAPON_8;
    case PK_RETURN: return INPUT_BTN_WEAPON_CYCLE;
    case PK_L: return INPUT_BTN_WEAPON_LINK;
    case PK_COMMA: return INPUT_BTN_SHIFT_DOWN;
    case PK_PERIOD: return INPUT_BTN_SHIFT_UP;
    case PK_ALT: return INPUT_BTN_E_BRAKE;
    case PK_G: return INPUT_BTN_HORN;
    case PK_KP_LEFT:  return INPUT_BTN_LOOK_LEFT;
    case PK_KP_RIGHT: return INPUT_BTN_LOOK_RIGHT;
    case PK_KP_DOWN:  return INPUT_BTN_LOOK_UP;    /* stock GreyDownArrow */
    case PK_KP_UP:    return INPUT_BTN_LOOK_DOWN;  /* stock GreyUpArrow   */
    case PK_INSERT:   return INPUT_BTN_LOOK_TARGET;
    case PK_BACKQUOTE: return INPUT_BTN_REARVIEW;
    case PK_KP_PGUP:  return INPUT_BTN_CAM_RANGE_MINUS; /* stock GreyPageUp   */
    case PK_KP_PGDN:  return INPUT_BTN_CAM_RANGE_PLUS;  /* stock GreyPageDown */
    case PK_KP_END:   return INPUT_BTN_CAM_RANGE_RESET; /* stock GreyEnd      */
    case PK_M:        return INPUT_BTN_MAP;             /* stock show_map     */
    case PK_N:        return INPUT_BTN_NOTEPAD;         /* stock show_notepad */
    default: return -1;
    }
}

/* Q17's one signed digital-steer held-time accumulator, seconds. */
static double s_steer_t;

void input_init(void)
{
    memset(s_held, 0, sizeof(s_held));
    memset(s_pressed, 0, sizeof(s_pressed));
    memset(s_release_required, 0, sizeof(s_release_required));
    s_steer_t = 0.0;
}

void input_require_release(void)
{
    for (int i = 0; i < INPUT_BTN_COUNT; i++) {
        if (s_held[i])
            s_release_required[i] = true;
        s_held[i] = false;
        s_pressed[i] = false;
    }
    /* Steering authority belongs to the gameplay input owner too. */
    s_steer_t = 0.0;
}

/*
 * The original's digital steering ramp (see input.h for the evidence and for
 * why it is load-bearing rather than cosmetic).
 *
 * Q17 names one accumulator, DAT_005340a8, for the signed digital-steer
 * branch.  Direction changes while either key remains held therefore keep
 * the accumulated magnitude; both keys centre the signed output.  The old
 * port used independent left/right timers.  After a long handbrake turn that
 * made the continued direction retain a large ramp while the opposite
 * direction restarted near zero — H-UAT-033's second directional latch.
 */
double input_steer_axis(bool left_held, bool right_held, double dt)
{
    if (!(dt > 0.0)) dt = 0.0;

    /* Release of BOTH keys zeroes the one timer -- it does not decay. */
    s_steer_t = (left_held || right_held) ? s_steer_t + dt : 0.0;
    if (s_steer_t > INPUT_STEER_FULL_S) s_steer_t = INPUT_STEER_FULL_S;

    double axis = sqrt(s_steer_t / INPUT_STEER_FULL_S);
    if (left_held == right_held) axis = 0.0;
    else if (right_held) axis = -axis;
    if (axis >  INPUT_STEER_CLAMP) axis =  INPUT_STEER_CLAMP;
    if (axis < -INPUT_STEER_CLAMP) axis = -INPUT_STEER_CLAMP;
    return axis;
}

void input_poll(InputState *out)
{

    bool quit = false;

    for (;;) {
        PlatformEvent e;
        if (!platform_pump_events(&e)) {
            /* Hard quit (window closed). */
            quit = true;
            break;
        }
        if (e.type == PLATFORM_EVENT_NONE)
            break;                              /* queue drained */

        /* ESC quits out of the state that owns input. */
        if (e.type == PLATFORM_EVENT_KEY_DOWN && e.key == PK_ESCAPE)
            quit = true;

        int btn = key_to_button(e.key);
        if (btn < 0)
            continue;

        if (e.type == PLATFORM_EVENT_KEY_DOWN) {
            /* Auto-repeat and a physically held handoff key stay blocked.
             * Only KEY_UP below re-arms this action. */
            if (s_release_required[btn])
                continue;
            if (!s_held[btn])
                s_pressed[btn] = true;
            s_held[btn] = true;
        } else if (e.type == PLATFORM_EVENT_KEY_UP) {
            s_held[btn] = false;
            s_release_required[btn] = false;
        }
    }

    for (int i = 0; i < INPUT_BTN_COUNT; i++) {
        out->held[i]    = s_held[i];
        out->pressed[i] = s_pressed[i];
        s_pressed[i]    = false;
    }
    out->quit = quit;
}
