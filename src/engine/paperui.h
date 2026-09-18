#ifndef PAPERUI_H
#define PAPERUI_H

/*
 * paperui.h — Original map / notepad / escape / title paper surfaces.
 *
 * Retail surfaces (all palette-index art, no CSS pastiche):
 *
 *   zmap{3|6}<scenario>.vqm   route map (M / show_map)
 *   znpd{3|6}<artclass>01.vqm held notepad (N / show_notepad)
 *   6escape.pak / 3escape.pak  Esc paper (MAINMN, PLYOPT, ABRTMS, …)
 *   p01load.pcx / loadscr.pcx  episode / boot title cards
 *
 * VQM texels are level-palette indices. PCX title cards carry their own
 * 256×RGB palette and are remapped into the live level palette via nearest
 * RGB match so they composite into the same 8-bit framebuffer.
 */

#include <stddef.h>
#include <stdint.h>

/* Interactive held surfaces (M/N toggles). Escape and title stack above. */
typedef enum {
    PAPER_NONE = 0,
    PAPER_MAP,
    PAPER_NOTEPAD
} PaperSurface;

/* Escape-menu paper panel (from 3escape/6escape.pak). */
typedef enum {
    PAPER_ESC_NONE = 0,
    PAPER_ESC_MAIN,     /* 6MAINMN1 — main escape menu */
    PAPER_ESC_PLYOPT,   /* 6PLYOPT1 — play options (port pause) */
    PAPER_ESC_ABRT,     /* 6ABRTMS1 — abort mission */
    PAPER_ESC_AUDCON,   /* 6AUDCON1 — audio */
    PAPER_ESC_GRXDET,   /* 6GRXDET1 — graphics detail */
    PAPER_ESC_SAVPNT,   /* 6SAVPNT1 — save point */
    PAPER_ESC_EXTGME,   /* 6EXTGME1 — exit game */
    PAPER_ESC_COUNT
} PaperEscape;

/*
 * paper_load_mission(mission_path)
 *   Decode map + notepad + escape pack + best-effort title PCX for this
 *   scenario. Returns 0 when anything loaded, -1 when nothing did.
 */
int  paper_load_mission(const char *mission_path);
void paper_unload(void);

/* Map / notepad (mutually exclusive). */
void         paper_toggle(PaperSurface which);
void         paper_hide(void);
PaperSurface paper_active(void);
int          paper_has(PaperSurface which);

/* Escape paper: shown while the port pause menu is up (or for native ESC). */
void         paper_set_escape(PaperEscape which);
PaperEscape  paper_escape(void);
int          paper_has_escape(PaperEscape which);

/*
 * Title card: episode PCX (p01load.pcx) or loadscr.pcx. paper_show_title()
 * arms a timed overlay; paper_title_tick() advances (call at 20 Hz);
 * paper_dismiss_title() ends early. paper_title_active() is nonzero while
 * the card is up.
 */
int  paper_has_title(void);
void paper_show_title(int ticks_20hz);   /* e.g. 40 = 2 s */
void paper_title_tick(void);
void paper_dismiss_title(void);
int  paper_title_active(void);

/*
 * The estimated map marker is a PORT DEV AID, never native UI. It is off by
 * default; the browser enables it only for ?dev=1. Player pose is in world
 * metres with yaw in radians.
 */
void paper_set_dev_mode(int enabled);
void paper_set_player(double x, double z, double yaw);

/*
 * paper_render(fb, w, h)
 *   Draw order: map/notepad (if active) → escape panel → title card.
 * Call AFTER the HUD composite. Escape and title sit on top of everything.
 */
void paper_render(uint8_t *fb, int w, int h);

/* "paper tag=p01 map=1 npd=1 esc=1 title=1 active=map escape=plyopt" */
int paper_stats(char *buf, size_t n);

#endif /* PAPERUI_H */
