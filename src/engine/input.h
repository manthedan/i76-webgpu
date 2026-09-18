#ifndef INPUT_H
#define INPUT_H

/*
 * input.h — Per-frame input snapshot
 *
 * The game loop wants a coherent picture of input for one frame, not a stream
 * of raw events. input_poll() drains the platform event queue, folds key
 * up/down events into a held-key state, and hands back a snapshot:
 *
 *   held[]     — button is down right now
 *   pressed[]  — at least one up→down transition arrived since the previous
 *                poll, even when its matching key-up arrived before this poll
 *                (edge; good for toggles and menu navigation)
 *   quit       — the player asked to quit (window closed, or ESC in gameplay)
 *
 * Buttons are abstract game actions, not physical keys. The mapping from
 * PlatformKey → InputButton lives in input.c, so rebinding later touches one
 * table. WASD and the arrow keys are aliased onto the same drive buttons.
 *
 * ORIGINAL BINARY:
 *   The original polled Win32 keyboard state (and DirectInput for the joystick)
 *   inside each inner loop. We don't have those globals reversed yet; this is a
 *   clean re-implementation of the same per-frame-snapshot idea. The joystick /
 *   analog steering axes will grow into this struct when we get there.
 */

#include <stdbool.h>

typedef enum {
    INPUT_BTN_UP,             /* accelerate / forward (Up / W) */
    INPUT_BTN_DOWN,           /* service brake (Down / S) */
    INPUT_BTN_LEFT,           /* steer left (Left / A) */
    INPUT_BTN_RIGHT,          /* steer right (Right / D) */
    INPUT_BTN_FIRE,           /* fire (Space) */
    INPUT_BTN_PAUSE,          /* pause toggle (P) */
    INPUT_BTN_REVERSE,        /* reverse-direction edge (X) */
    INPUT_BTN_WEAPON_1,       /* hardpoint selection (1..8) */
    INPUT_BTN_WEAPON_2,
    INPUT_BTN_WEAPON_3,
    INPUT_BTN_WEAPON_4,
    INPUT_BTN_WEAPON_5,
    INPUT_BTN_WEAPON_6,
    INPUT_BTN_WEAPON_7,
    INPUT_BTN_WEAPON_8,
    INPUT_BTN_SHIFT_DOWN,     /* manual shift down (Comma) */
    INPUT_BTN_SHIFT_UP,       /* manual shift up (Period) */
    INPUT_BTN_E_BRAKE,        /* held handbrake (Alt) */
    INPUT_BTN_HORN,           /* horn one-shot edge (G) */
    /*
     * Pilot glance (stock input.map / KEYBOARD.MAP: pilot_glance_left =
     * GreyLeftArrow, pilot_glance_right = GreyRightArrow, pilot_glance_down
     * = GreyUpArrow, pilot_glance_up = GreyDownArrow, pilot_glance_target =
     * Insert). "Grey" is the numpad-cluster arrow set, so these bind the
     * keypad arrows; the shipped up/down inversion is preserved here, at
     * the one mapping table. Glances are HELD actions: the look lasts
     * exactly as long as the key, and release returns the view forward
     * (that release-is-reset semantic is why there is no separate center
     * binding in the stock map).
     */
    INPUT_BTN_LOOK_LEFT,      /* keypad Left  */
    INPUT_BTN_LOOK_RIGHT,     /* keypad Right */
    INPUT_BTN_LOOK_UP,        /* keypad Down  (pilot_glance_up)   */
    INPUT_BTN_LOOK_DOWN,      /* keypad Up    (pilot_glance_down) */
    INPUT_BTN_LOOK_TARGET,    /* Insert: look at the current target */
    INPUT_BTN_REARVIEW,       /* backquote edge: rearview mirror toggle */
    /*
     * Camera range controls (stock input.map: GreyPageDown is simultaneously
     * track_distance_plus, overview_zoom_plus and zoom_factor_plus;
     * GreyPageUp the matching _minus actions; GreyEnd is zoom_factor_reset).
     * Like the glance arrows above, one physical key carries several native
     * actions and the active drive view decides which one applies — the
     * camera code in webmain.c owns that resolution.
     */
    INPUT_BTN_CAM_RANGE_MINUS, /* GreyPageUp: distance/zoom minus (held)  */
    INPUT_BTN_CAM_RANGE_PLUS,  /* GreyPageDown: distance/zoom plus (held) */
    INPUT_BTN_CAM_RANGE_RESET, /* GreyEnd: reset view adjustables (edge)  */
    /* Appended: keep prior ordinals stable for trial bitmasks / tapes. */
    INPUT_BTN_WEAPON_CYCLE,   /* cycle highlighted mount (Enter) */
    INPUT_BTN_WEAPON_LINK,    /* toggle same-class fixed mounts (L) */
    /* show_map / show_notepad (stock keyboard.map M / N). Appended after
     * the existing weapon buttons so all established bit positions remain
     * stable. */
    INPUT_BTN_MAP,
    INPUT_BTN_NOTEPAD,
    INPUT_BTN_COUNT
} InputButton;

typedef struct {
    bool held[INPUT_BTN_COUNT];
    bool pressed[INPUT_BTN_COUNT];
    bool quit;
} InputState;

/*
 * Reset the persistent held-key state. Call once when entering a state that
 * owns the input (e.g. start of the gameplay loop) so stale key-downs from a
 * previous state don't leak in.
 */
void input_init(void);

/*
 * Consume the current input owner at a handoff boundary. Every action that is
 * held now is suppressed until its mapped key is released; pending press edges
 * are discarded. This is deliberately action-general: a shell/cutscene key
 * must never become throttle, steering, fire, or another gameplay action just
 * because gameplay became active while the physical key stayed down.
 *
 * Call after input_poll() has drained the outgoing owner's events. The caller
 * may use that final snapshot itself (for example Space to skip an intro), but
 * the next owner will not see any of its held actions until release/re-press.
 */
void input_require_release(void);

/*
 * Drain the platform event queue and fill *out with this frame's snapshot.
 * Call exactly once per frame, before updating the world.
 */
void input_poll(InputState *out);

/*
 * input_steer_axis(left_held, right_held, dt)
 *   Digital keys -> a steering axis in [-1, +1], with the ORIGINAL'S RAMP.
 *
 * FACT, not invention (docs/specs/m4/ghidra-physics.md §Q17, input path
 * FUN_0043ebf0): the original's input layer does NOT hand the simulation a
 * held key as full lock. It integrates how long the key has been down and
 * produces
 *
 *     steer = +-sqrt(held / 3)      held capped at 3 s, release zeroes it,
 *                                   final clamp +-0.9999
 *
 * — a fast initial rise reaching full lock only after three seconds. car.c's
 * D5 records this and says it "lives in its input layer (caller-side for us)",
 * and for a long time no caller implemented it: web_drive_step mapped a held
 * arrow straight to 1.0.
 *
 * WHY IT MATTERS, beyond fidelity. The weapon cone is +-5 degrees, and full
 * lock is 45 degrees at the wheels. With bang-bang steering the smallest
 * correction a keyboard can express overshoots the cone, which is why this
 * repository twice measured that keyboard play "cannot hold the fire cone"
 * and concluded a driver needed analog input. It did not: it needed the ramp.
 * A player tapping the key now gets a small correction, which is the whole
 * point of a sqrt curve that starts steep and flattens.
 *
 * Q17's decompile names one held-time accumulator (`DAT_005340a8`) for the
 * signed digital-steer branch. Holding BOTH keys centres the output while
 * that one timer continues; switching direction without a neutral tick keeps
 * the accumulated magnitude. Releasing both keys resets it to zero.
 *
 * Stateful across calls (the timer is the point); `dt` is seconds since the
 * previous call. input_init() clears them. Pure arithmetic apart from sqrt,
 * which IEEE-754 requires to be correctly rounded, so this stays
 * bit-identical across native and wasm.
 */
#define INPUT_STEER_FULL_S  3.0     /* seconds of hold to reach full lock  */
#define INPUT_STEER_CLAMP   0.9999  /* the original's final clamp (Q17)    */

double input_steer_axis(bool left_held, bool right_held, double dt);

#endif /* INPUT_H */
