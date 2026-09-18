/*
 * hud.c — M3 HUD module (see hud.h)
 *
 * Loads the cockpit dashboard (VQM tiles via .pix/.pak + .cbk codebook),
 * the level .act palette, the .elt sprite tables and their .map sheets,
 * and renders dashboard + sprite proof strip + text line into the 8-bit
 * indexed framebuffer.
 *
 * Sources: docs/specs/m2/pipeline.md (pix/pak/vqm/cbk/map/act byte
 * tables), scene.md §6.2 (ETBL), Open76 EltParser.cs (elt text grammar —
 * semantics only), verified against the Nitro Pack data (nitro.zfs).
 * Empirical DECISIONs (evidence in tools/hud_probe.c output + M3 report):
 *
 *  D1  .map orientation (pipeline.md P3): row-major byte order, stored
 *      BOTTOM-UP (display row 0 = last stored row). Evidence: zgear101.map
 *      decoded this way shows the gear plate "PRND21" upright and
 *      readable; top-down row-major shows it upside-down; Open76's
 *      column-major reading yields vertical-stripe noise in the diagnostic
 *      comparison. Matches Tony's "needs
 *      to be flipped and rotated" note — for square textures a flip+90°
 *      rotation reads like a transpose, which explains Open76's
 *      column-major addressing working there.
 *  D2  .vqm orientation: row-major exactly as pipeline.md §3.2 documents
 *      (blocks L-to-R/T-to-B, row-major within each 4x4 block, no flip).
 *      Verified byte-for-byte against i76img.py decode_vqm, whose dash
 *      decode is a coherent instrument cluster; the column-major reading
 *      scrambles blocks. (The two formats genuinely differ — see D1.)
 *  D3  Palette: dash VQM indices resolve against the LEVEL .act
 *      (p01.act for Nitro scene 1), not vpit.act. vpit.act is a sparse
 *      table (indices 0..~175 all #FF00FF magenta, real colors only in
 *      the upper range — likely a Glide-era leftover, role UNKNOWN);
 *      rendering the dash through it paints the cluster piping magenta.
 *      The level palettes (p01/t01) resolve to coherent grayscale gauges
 *      + yellow warning lamps. Load order: the mission's own WDEF/WRLD
 *      palette reference (scene.md §2, +30 13-byte field — CONFIRMED),
 *      then p01.act, t01.act, vpit.act, else meshview-index fallback
 *      (hud_palette() returns NULL then).
 *  D4  Dash composition: a zdashN set's two 256x128 tiles stack into a
 *      256x256 dash — tile 1 (instrument cluster) on top, tile 2
 *      (radio/console) below; content makes the order unambiguous.
 *      Which zdashN set a scenario uses is now FACT (nitro.exe FUN
 *      @0x49f7e0 + shipped table @0x503c88): the cockpit-art class
 *      digit of the ZSWLS%1d01.tmt / ZNPD%1d01.map / ZHR45%1d01.tmt /
 *      ZHL45%1d01.tmt format names is derived from the SCENARIO name
 *      parsed "%1s%2d" through a shipped lookup table (P01-P04/P17/P19
 *      -> 2, P05-P08/P18/B01 -> 3, P09-P12 -> 4, P13-P16 -> 5;
 *      unparseable -> 2, unlisted -> 1). hud_load_mission() applies
 *      that table to the zdash<cls>01 day tiles, falling back to set 1
 *      when the class set is absent from the data. It is scenario-keyed,
 *      NOT vehicle-keyed: every Nitro VDF names the same vpit_1.elt and
 *      no vehicle->dash-set field exists anywhere in the data.
 *  D5  Sprite placement: the .elt gives sub-rects and anchor labels but
 *      NOT dash-space positions for the src sheets (needle rotation
 *      centers live on dst sheets; radar anchors range_pos/led_pos have
 *      no resolved dst — zrad.map is absent from nitro.zfs). The probe
 *      therefore draws a "proof strip" (gear plate + 3 needle frames cut
 *      per .elt rects) in the frame's upper area instead of guessing
 *      gauge composites. Two composites ARE fully decoded and drawn for
 *      real: the gear indicator (plate, arrow sprite and per-gear dst
 *      anchors — drawn over the dash) and the compass (the zcm_.map
 *      panel's compass_window dst anchor plus the zcme.map strip's
 *      left/right crop labels, which give the crop size and the wrap
 *      period — drawn left of the dash; see hud_render_frame). The
 *      speedo/tach needles joined them in D8: their sweep-vs-state
 *      formulas are FACT from nitro.exe's gauge code and their dash
 *      positions are measured from the dash art itself. The damage
 *      panel's per-system state remains UNRESOLVED and undrawn.
 *  D6  Text color: brightest palette index by Rec.601 luminance
 *      (excluding 0xFF, which the sprite blits treat as the key) — legible
 *      on the dark dashboard across level palettes without hardcoding an
 *      index. Excluding it costs nothing either way: 0xFF is BLACK in the
 *      level palettes, so a brightest-entry search would never pick it.
 *  D7  0xFF is transparent for SPRITES and OPAQUE for the DASH sheet, and
 *      the difference is measured rather than chosen. Keying the dash made
 *      the cockpit 59.9% holes — 19,616 of ZDASH101's 32,768 texels are
 *      0xFF, dithered through the panel body against 254/253/252/250/221,
 *      because 249..255 is a contiguous dark ramp in the level palette and
 *      the art is blending two ramp entries. Unkeyed, the same bytes render
 *      a complete solid dashboard. See the blitter comment for the full
 *      evidence; nitro.exe's own blit is UNREAD, so this is two measured
 *      behaviours, not one reversed rule.
 *  D8  Speedo/tach needles (LIVE instruments, gameplay output — not the
 *      proof strip): the needle frame for the current speed/RPM is cut
 *      from zneedle6.map and composited over the dash's baked gauge
 *      faces at the hub dots carried by the dash art itself. Frame
 *      selection is FACT from nitro.exe's gauge code (speedo
 *      0x450a80-0x450c68: value clamped [0,150] mph, frame =
 *      9 + trunc(mph * 0.1933333) over frames 9..38; tach
 *      0x450c6b-0x450e14: value clamped 8400 rpm, frame =
 *      trunc(rpm * 0.003 + 1) clamped 25; sprite name sprintf
 *      "3needle_%d" @0x4f98cc; anchors speedneedleloc/tachneedleloc
 *      @0x4f98bc/0x4f98ac). See gauge_validate/gauge_speed_frame for the
 *      full evidence: hub-dot detection (which dash sets get needles),
 *      the zneedle3-vs-zneedle6 twinning, the .elt rows' separator-line
 *      quirk (blit h-1), and the m/s->mph input conversion (the one
 *      port-side mapping: hud_render_frame takes m/s, the face is mph).
 */

#include "hud.h"
#include "vfs.h"
#include "vqm.h"
#include "font.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* -----------------------------------------------------------------------
 * Little-endian readers (asset formats are LE; host order not assumed)
 * ----------------------------------------------------------------------- */

static uint32_t rd_u32(const uint8_t *p)
{
    return (uint32_t)p[0]         | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* rd_u16 left with the decoder in vqm.c; nothing else in hud.c reads 16-bit
 * fields. */

/* -----------------------------------------------------------------------
 * Module state
 * ----------------------------------------------------------------------- */

#define HUD_MAX_PIX_ENTRIES 8
#define HUD_MAX_TABLES      64
#define HUD_MAX_ITEMS       1024
#define HUD_TEXT_CAP        128

/* Full turn in radians (M_PI is not strict C11; mission.c does the same). */
#define HUD_COMPASS_TURN    6.28318530717958647692

typedef struct {
    char     map[16];      /* sheet filename, lowercase */
    int      is_dst;       /* "dst" marker (anchors) vs "src" (sprites) */
    int      items;        /* item count attributed to this table */
} HudEltTable;

typedef struct {
    char name[20];
    int  table;            /* index into s.tables */
    int  x, y, w, h;
} HudEltItem;

static struct {
    int      loaded;
    int      orient;             /* HUD_ORIENT_* experiment knob */

    uint8_t  pal[256][3];
    int      have_pal;
    char     pal_name[16];
    uint8_t  remap[256];         /* identity, or meshview buckets w/o pal */

    uint8_t *dash;               /* stacked dash tiles, dash_w*dash_h */
    int      dash_w, dash_h;
    int      dash_tiles;
    char     dash_base[16];      /* e.g. "zdash201" (D4 class) */
    int      art_class;          /* derived class digit, 0 pre-load   */

    uint8_t *mirror_mask;        /* zmiri101.map — 0xFF glass cutout   */
    int      mm_w, mm_h;
    uint8_t *mirror_bezel;       /* zmiro101.map — opaque surround art */
    int      mb_w, mb_h;

    uint8_t *radar_mask;         /* zradmask.map — upper-left housing */
    int      rad_w, rad_h;
    /* H-UAT-002: the AUTHORED radar unit — zradf000.pak's ZRADF000..030
     * VQM tiles (256x128, 0xFF-keyed surround) are the green phosphor
     * playfield + rotating sweep the original shows; the port used to
     * blit only the housing, leaving the playfield black. */
#define HUD_RADAR_FRAMES 31
    uint8_t *radar_sweep[HUD_RADAR_FRAMES];
    int      radar_sweep_count;
    int      radar_sweep_w, radar_sweep_h;
    uint8_t *radar_overlay;      /* housing+crosshair: mask with the
                                    index-1 surround flood-keyed to 0xFF */
    int      radar_cx, radar_cy, radar_r;   /* playfield circle, px      */
    uint8_t  radar_threat_idx, radar_ally_idx;
    struct { double right, fwd; int threat; } radar_contacts[HUD_RADAR_MAX_CONTACTS];
    int      radar_contact_count;
    uint8_t *weapon_panel;       /* zwpe.map — upper-centre rack */
    int      wep_w, wep_h;
    uint8_t *damage_panel;       /* zsy_.map — upper-right condition */
    int      dmg_w, dmg_h;
    uint8_t *damage_states;      /* zsye.map — live condition sprites */
    int      ds_w, ds_h;
    uint8_t *weapon_labels;      /* zdue.map — named on/off strips */
    int      wl_w, wl_h;
    uint8_t *weapon_digits;      /* znbe.map — vertical 0..9 glyphs */
    int      wd_w, wd_h;
    uint8_t *target_led;         /* zdde.map — target-condition LED
                                    states (H-UAT-011c)               */
    int      tl_w, tl_h;
    /* H-UAT-011c target readout (hud_set_target). */
    int      tgt_set;
    char     tgt_name[41];
    int      tgt_hp, tgt_hp_max;
    int      tgt_range_m;

#define HUD_SIDEARM_SIDES 2
#define HUD_MAX_SIDEARM_FRAMES 16
    /* ZHL45/ZHR45<class>01 glance/sidearm frames, manifest order. */
    uint8_t *sa_frame[HUD_SIDEARM_SIDES][HUD_MAX_SIDEARM_FRAMES];
    char     sa_name[HUD_SIDEARM_SIDES][HUD_MAX_SIDEARM_FRAMES][16];
    int      sa_fw[HUD_SIDEARM_SIDES][HUD_MAX_SIDEARM_FRAMES];
    int      sa_fh[2][HUD_MAX_SIDEARM_FRAMES];
    int      sa_count[HUD_SIDEARM_SIDES];

    uint8_t *gear;               /* zgear101.map, decoded */
    int      gear_w, gear_h;

    uint8_t *gear_arrow;         /* zgeare.map — the PRND21 selector */
    int      ga_w, ga_h;
    uint8_t *gear_live;          /* remapped base + live selector arrow */
    RTex     gear_live_tex;
    int      gear_live_selector;
    int      gear_live_valid;

    uint8_t *needles;            /* zneedle6.map, decoded */
    int      ndl_w, ndl_h;

    uint8_t *compass;            /* zcm_.map compass panel, decoded */
    int      comp_w, comp_h;
    uint8_t *compass_strip;      /* zcme.map bearing strip, decoded */
    int      cs_w, cs_h;
    int      comp_ok;            /* elt geometry validated; draw gate */
    int      comp_win_x, comp_win_y;   /* compass_window anchor in panel */
    int      comp_x0, comp_y0;         /* left crop origin = bearing 0 */
    int      comp_crop_w, comp_crop_h; /* crop size (left/right w,h) */
    int      comp_period;              /* right.x - left.x, px per turn */
    double   comp_yaw;                 /* hud_set_compass_yaw (car_pose
                                          convention: 0=+Z, positive-left) */
    uint8_t *compass_live;       /* remapped panel + live bearing crop */
    RTex     compass_live_tex;
    int      compass_live_cx;
    int      compass_live_valid;

    uint8_t *reticle;            /* zretc_6.map — 640-mode sight ring */
    int      reticle_w, reticle_h;
    uint8_t *reticle_live;       /* level-palette copy for 3-D sampling */
    RTex     reticle_live_tex;

#define HUD_NEEDLE_FRAMES 38           /* needle_1..needle_38 in zneedle6 */
    int      gauge_ok;                 /* hub dots validated; draw gate   */
    int      spd_hub_x, spd_hub_y;     /* speedo needle pivot, dash space */
    int      tach_hub_x, tach_hub_y;   /* tach needle pivot, dash space   */
    int      ndl_fx[HUD_NEEDLE_FRAMES];/* needle frame rects in zneedle6  */
    int      ndl_fy[HUD_NEEDLE_FRAMES];
    int      ndl_fw, ndl_fh;           /* uniform frame w, h (blit h-1)   */
    double   eng_rpm;                  /* hud_set_engine_rpm; 0 = at rest */

    HudEltTable tables[HUD_MAX_TABLES];
    int         table_count;
    int         table_overflow;
    HudEltItem  items[HUD_MAX_ITEMS];
    int         item_count;
    int         item_overflow;

    Font     *font;
    char      font_name[16];

    char      text[HUD_TEXT_CAP];
    uint8_t   text_color;

    int       vit_hp, vit_hp_max, vit_fuel;   /* hud_set_vitals (M7) */
    /* H-UAT-003: per-component condition pools (hud_set_condition); the
     * cond_set flag keeps the legacy scalar fallback until a host feeds
     * component state. */
    int       cond_hp[HUD_CONDITION_COMPONENTS];
    int       cond_max[HUD_CONDITION_COMPONENTS];
    int       cond_set;
    char      weapon[17];
    int       weapon_slot, weapon_count;
    int       weapon_ammo, weapon_ammo_max, weapon_damage;
#define HUD_WEAPON_ROWS 5
    char      weapon_row_name[HUD_WEAPON_ROWS][17];
    int       weapon_row_ammo[HUD_WEAPON_ROWS];
    int       weapon_row_max[HUD_WEAPON_ROWS];
    int       weapon_row_armed[HUD_WEAPON_ROWS]; /* multi-link "on" strips */
    uint32_t  weapon_authored_rows;    /* last render: zdue label-strip draws */
    uint32_t  weapon_fallback_rows;    /* last render: bitmap-font label draws */
    uint32_t  weapon_ammo_rows;        /* last render: actual digit draws      */
} s;

/* -----------------------------------------------------------------------
 * .act palette
 * ----------------------------------------------------------------------- */

/* Read + validate one 768-byte .act into s.pal. 0 on success. */
static int try_palette(const char *name)
{
    size_t n = 0;
    uint8_t *buf = vfs_read_file(name, &n);
    if (!buf) return -1;
    if (n != 768u) {
        fprintf(stderr, "[hud] %s: want 768 bytes, got %zu — skipped\n",
                name, n);
        vfs_free(buf);
        return -1;
    }
    for (int c = 0; c < 256; c++) {
        s.pal[c][0] = buf[c * 3 + 0];
        s.pal[c][1] = buf[c * 3 + 1];
        s.pal[c][2] = buf[c * 3 + 2];
    }
    snprintf(s.pal_name, sizeof s.pal_name, "%s", name);
    vfs_free(buf);
    return 0;
}

static int load_palette(const char *preferred)
{
    /* D3: level palettes first; vpit.act last (sparse magenta table).
     * A mission-resolved name (WRLD, below) outranks the chain. */
    static const char *const candidates[] = {
        "p01.act", "t01.act", "vpit.act"
    };
    if (preferred && try_palette(preferred) == 0)
        return 0;
    for (size_t i = 0; i < sizeof candidates / sizeof candidates[0]; i++) {
        if (preferred && strcmp(candidates[i], preferred) == 0)
            continue;               /* already tried */
        if (try_palette(candidates[i]) == 0)
            return 0;
    }
    return -1;
}

/*
 * Resolve the mission's level palette from its WDEF/WRLD world-reference
 * chunk (scene.md §2, CONFIRMED): top-level chunk stream -> WDEF ->
 * sub-chunk stream -> WRLD, whose payload holds the .act name as a
 * 13-byte null-padded field at +30. Chunk size fields INCLUDE the 8-byte
 * header (same walk as scene.c's chunk_at). Mirrors scene_load()'s
 * dir-prefix search. Returns 0 and fills `out` (lowercased) on success.
 */
static int mission_palette_name(const char *mission, char out[16])
{
    static const char *const dirs[] = { "", "miss8/", "miss16/",
                                        "missions/" };
    char path[64];
    size_t sz = 0;
    uint8_t *buf = NULL;
    for (size_t i = 0; i < sizeof dirs / sizeof dirs[0] && !buf; i++) {
        snprintf(path, sizeof path, "%s%s", dirs[i], mission);
        buf = vfs_read_file(path, &sz);
    }
    if (!buf) return -1;

    int found = -1;
    for (size_t off = 0; off + 8 <= sz && found < 0; ) {
        uint32_t total = rd_u32(buf + off + 4);
        if (total < 8 || (size_t)total > sz - off) break;
        if (memcmp(buf + off, "WDEF", 4) == 0) {
            size_t end = off + total;
            for (size_t p = off + 8; p + 8 <= end; ) {
                uint32_t sub = rd_u32(buf + p + 4);
                if (sub < 8 || (size_t)sub > end - p) break;
                if (memcmp(buf + p, "WRLD", 4) == 0 &&
                    p + 8 + 30 + 13 <= p + sub) {
                    char name[14];
                    memcpy(name, buf + p + 8 + 30, 13);
                    name[13] = '\0';
                    for (int i = 0; name[i]; i++) {
                        if (name[i] >= 'A' && name[i] <= 'Z')
                            name[i] = (char)(name[i] + ('a' - 'A'));
                        else if (name[i] < 32 || name[i] > 126) {
                            name[i] = '\0';   /* printable only */
                            break;
                        }
                    }
                    size_t ln = strlen(name);
                    if (ln > 4 && strcmp(name + ln - 4, ".act") == 0) {
                        snprintf(out, 16, "%s", name);
                        found = 0;
                    }
                    break;
                }
                p += sub;
            }
            break;
        }
        off += total;
    }
    vfs_free(buf);
    return found;
}

/* Brightest index by Rec.601 luminance, excluding 0xFF (D6). */
static uint8_t brightest_index(void)
{
    int best = 1, best_lum = -1;
    for (int i = 0; i < 255; i++) {
        int lum = 299 * s.pal[i][0] + 587 * s.pal[i][1] + 114 * s.pal[i][2];
        if (lum > best_lum) {
            best_lum = lum;
            best = i;
        }
    }
    return (uint8_t)best;
}

/* -----------------------------------------------------------------------
 * .pix manifest (text: count line, then "NAME <off> <len>" per entry)
 * ----------------------------------------------------------------------- */

/* Portable CRLF line splitter (strict C11, no POSIX extensions). Returns each
 * line NUL-terminated in place and advances *cursor past it. */
static char *next_line(char **cursor)
{
    char *start = *cursor;
    if (!start) return NULL;
    char *p = start;
    while (*p && *p != '\r' && *p != '\n') p++;
    if (*p == '\0') { *cursor = NULL; return start; }
    *p = '\0';
    p++;
    while (*p == '\r' || *p == '\n') p++;
    *cursor = p;
    return start;
}

typedef struct {
    char     name[16];
    uint32_t off, len;
} PixEntry;

static int pix_parse(char *text, PixEntry *entries, int max_entries)
{
    int count = -1, n = 0;
    char *cursor = text;
    for (char *line = next_line(&cursor);
         line;
         line = next_line(&cursor)) {
        if (count < 0) {
            count = atoi(line);
            continue;
        }
        if (n >= max_entries) break;
        char name[16] = {0};
        unsigned off = 0, len = 0;
        if (sscanf(line, "%15s %u %u", name, &off, &len) != 3) continue;
        PixEntry *e = &entries[n++];
        snprintf(e->name, sizeof e->name, "%s", name);
        e->off = off;
        e->len = len;
    }
    return n;
}

/* .vqm tile decode moved to engine/vqm.c (M8 V2): world textures are the
 * same format as the dashboard tiles, and one decoder is the point. hud.c's
 * `s.orient` read became an explicit parameter so vqm.c has no HUD dependency.
 */

/* -----------------------------------------------------------------------
 * .map sheet decode (D1: row-major, bottom-up storage)
 * ----------------------------------------------------------------------- */

static uint8_t *map_decode(const uint8_t *buf, size_t n, int *ow, int *oh)
{
    if (n < 8) return NULL;
    uint32_t w = rd_u32(buf + 0), h = rd_u32(buf + 4);
    if (w == 0 || h == 0 || w > 4096 || h > 4096) return NULL;
    if (n - 8 < (size_t)w * h) return NULL;
    const uint8_t *px = buf + 8;

    uint8_t *out = malloc((size_t)w * h);
    if (!out) return NULL;

    for (uint32_t y = 0; y < h; y++) {
        for (uint32_t x = 0; x < w; x++) {
            size_t src_i;
            switch (s.orient) {
            case HUD_ORIENT_MAP_TOPDOWN:          /* row-major, top-down */
                src_i = (size_t)y * w + x;
                break;
            case HUD_ORIENT_COLUMN:               /* Open76 column-major */
                src_i = (size_t)x * h + y;
                break;
            case HUD_ORIENT_SHIPPING:
            default:                              /* row-major, bottom-up */
                src_i = (size_t)(h - 1u - y) * w + x;
                break;
            }
            out[(size_t)y * w + x] = px[src_i];
        }
    }
    *ow = (int)w;
    *oh = (int)h;
    return out;
}

static uint8_t *map_load(const char *name, int *ow, int *oh)
{
    size_t n = 0;
    uint8_t *buf = vfs_read_file(name, &n);
    if (!buf) return NULL;
    uint8_t *img = map_decode(buf, n, ow, oh);
    vfs_free(buf);
    return img;
}

/* Nearest level-palette index to an RGB triple, excluding the 0xFF key
 * (same search standing as D6's brightest_index). */
static uint8_t nearest_pal_index(int r, int g, int b)
{
    int best = 1, best_d = -1;
    for (int i = 0; i < 255; i++) {
        int dr = s.pal[i][0] - r, dg = s.pal[i][1] - g, db = s.pal[i][2] - b;
        int d = dr * dr + dg * dg + db * db;
        if (best_d < 0 || d < best_d) {
            best_d = d;
            best = i;
        }
    }
    return (uint8_t)best;
}

/*
 * H-UAT-002: the authored radar unit. nitro.zfs (and the base I76.ZFS)
 * ship zradf000.pak/.pix/.tmt: 62 VQM tiles, ZRADF000..030 (line sweep
 * with the phosphor trail — the look the recorded original frames show)
 * and ZRADB000..030 (a wedge-beam variant, unused here). Each 256x128
 * tile carries the WHOLE unit — housing, green playfield, range rings —
 * with an 0xFF-keyed surround, so a keyed blit replaces the black
 * playfield the housing-only blit left. The B-set/F-set semantics are
 * not reversed (names only); using the F set is a presentation DECISION
 * grounded in the captures, not a fidelity claim about the runtime.
 *
 * zradmask.map then overlays the housing + crosshair: its index-1 beige
 * surround (the HUD_BLIT_KEY1 key) and its 0xFF playfield window both
 * become transparent here — the surround by a border flood fill, the
 * window by the key — so the sweep and the contact blips show through
 * while the crosshair and dial chrome keep their authored pixels. The
 * playfield circle (centre/radius for the blip clip) is MEASURED from
 * the mask's 0xFF window at load, not hardcoded.
 */
static void radar_load(void)
{
    s.radar_sweep_count = 0;
    size_t pix_n = 0, pak_n = 0;
    uint8_t *pix = vfs_read_file("zradf000.pix", &pix_n);
    uint8_t *pak = vfs_read_file("zradf000.pak", &pak_n);
    if (pix && pak) {
        char *text = malloc(pix_n + 1);
        if (text) {
            memcpy(text, pix, pix_n);
            text[pix_n] = '\0';
            PixEntry entries[64];
            int te = pix_parse(text, entries, 64);
            for (int i = 0; i < te &&
                        s.radar_sweep_count < HUD_RADAR_FRAMES; i++) {
                if (strncmp(entries[i].name, "ZRADF", 5) != 0)
                    continue;
                if ((size_t)entries[i].off + entries[i].len > pak_n)
                    continue;
                int w = 0, h = 0;
                uint8_t *t = vqm_decode(pak + entries[i].off,
                                        entries[i].len,
                                        s.orient == HUD_ORIENT_COLUMN,
                                        &w, &h);
                if (!t)
                    continue;
                if (s.radar_sweep_count == 0) {
                    s.radar_sweep_w = w;
                    s.radar_sweep_h = h;
                } else if (w != s.radar_sweep_w || h != s.radar_sweep_h) {
                    free(t);
                    continue;
                }
                s.radar_sweep[s.radar_sweep_count++] = t;
            }
            free(text);
        }
    }
    if (pix) vfs_free(pix);
    if (pak) vfs_free(pak);

    free(s.radar_overlay);
    s.radar_overlay = NULL;
    if (!s.radar_mask)
        return;
    int w = s.rad_w, h = s.rad_h, npx = w * h;

    /* Measure the playfield window: the largest 0xFF blob in the mask. */
    {
        uint8_t *seen = calloc((size_t)npx, 1);
        int *stack = malloc((size_t)npx * sizeof(int));
        int best_n = 0, bx0 = 0, bx1 = 0, by0 = 0, by1 = 0;
        if (seen && stack) {
            for (int i = 0; i < npx; i++) {
                if (s.radar_mask[i] != 0xFF || seen[i])
                    continue;
                int sp = 0, cnt = 0;
                int x0 = w, x1 = -1, y0 = h, y1 = -1;
                stack[sp++] = i;
                seen[i] = 1;
                while (sp > 0) {
                    int j = stack[--sp];
                    int x = j % w, y = j / w;
                    cnt++;
                    if (x < x0) x0 = x;
                    if (x > x1) x1 = x;
                    if (y < y0) y0 = y;
                    if (y > y1) y1 = y;
                    if (x > 0     && s.radar_mask[j-1] == 0xFF && !seen[j-1]) { seen[j-1] = 1; stack[sp++] = j-1; }
                    if (x < w - 1 && s.radar_mask[j+1] == 0xFF && !seen[j+1]) { seen[j+1] = 1; stack[sp++] = j+1; }
                    if (y > 0     && s.radar_mask[j-w] == 0xFF && !seen[j-w]) { seen[j-w] = 1; stack[sp++] = j-w; }
                    if (y < h - 1 && s.radar_mask[j+w] == 0xFF && !seen[j+w]) { seen[j+w] = 1; stack[sp++] = j+w; }
                }
                if (cnt > best_n) {
                    best_n = cnt;
                    bx0 = x0; bx1 = x1; by0 = y0; by1 = y1;
                }
            }
        }
        free(seen);
        free(stack);
        if (best_n > 100) {
            s.radar_cx = (bx0 + bx1) / 2;
            s.radar_cy = (by0 + by1) / 2;
            int rx = (bx1 - bx0) / 2, ry = (by1 - by0) / 2;
            s.radar_r = rx < ry ? rx : ry;
        } else {
            s.radar_r = 0;          /* no window: no blips */
        }
    }

    /* Overlay: the mask with its index-1 surround (border-connected)
     * rekeyed to 0xFF, so one keyed blit keeps housing + crosshair and
     * opens both the surround and the playfield window. */
    s.radar_overlay = malloc((size_t)npx);
    if (!s.radar_overlay)
        return;
    memcpy(s.radar_overlay, s.radar_mask, (size_t)npx);
    {
        int *stack = malloc((size_t)npx * sizeof(int));
        int sp = 0;
        if (stack) {
            for (int x = 0; x < w; x++) {
                if (s.radar_overlay[x] == 1) stack[sp++] = x;
                if (s.radar_overlay[(h-1)*w + x] == 1)
                    stack[sp++] = (h-1)*w + x;
            }
            for (int y = 0; y < h; y++) {
                if (s.radar_overlay[y*w] == 1) stack[sp++] = y*w;
                if (s.radar_overlay[y*w + w-1] == 1)
                    stack[sp++] = y*w + w-1;
            }
            while (sp > 0) {
                int j = stack[--sp];
                if (s.radar_overlay[j] != 1)
                    continue;
                s.radar_overlay[j] = 0xFF;
                int x = j % w, y = j / w;
                if (x > 0     && s.radar_overlay[j-1] == 1) stack[sp++] = j-1;
                if (x < w - 1 && s.radar_overlay[j+1] == 1) stack[sp++] = j+1;
                if (y > 0     && s.radar_overlay[j-w] == 1) stack[sp++] = j-w;
                if (y < h - 1 && s.radar_overlay[j+w] == 1) stack[sp++] = j+w;
            }
            free(stack);
        }
    }

    /* Blip colours: the port's ALLY/THREAT vocabulary (the original's
     * blip colour semantics are not reversed) — the legible bright index
     * for contacts, the nearest red for threats, both palette-resolved. */
    s.radar_ally_idx = brightest_index();
    s.radar_threat_idx = nearest_pal_index(220, 40, 30);
}

/* -----------------------------------------------------------------------
 * .elt sprite tables (text: "dst|src <map>" then "label <n> x y w h")
 * ----------------------------------------------------------------------- */

static void elt_parse(char *text)
{
    int cur = -1;
    char *cursor = text;
    for (char *line = next_line(&cursor);
         line;
         line = next_line(&cursor)) {
        char w0[20] = {0}, w1[20] = {0};
        int a = 0, b = 0, c = 0, d = 0;
        int fields = sscanf(line, "%19s %19s %d %d %d %d",
                            w0, w1, &a, &b, &c, &d);
        if (fields == 2 &&
            (strcmp(w0, "dst") == 0 || strcmp(w0, "src") == 0)) {
            if (s.table_count < HUD_MAX_TABLES) {
                HudEltTable *t = &s.tables[s.table_count];
                snprintf(t->map, sizeof t->map, "%.15s", w1);
                t->is_dst = (strcmp(w0, "dst") == 0);
                t->items = 0;
                cur = s.table_count++;
            } else {
                s.table_overflow++;
            }
        } else if (fields == 6 && strcmp(w0, "label") == 0 && cur >= 0) {
            if (s.item_count < HUD_MAX_ITEMS) {
                HudEltItem *it = &s.items[s.item_count++];
                snprintf(it->name, sizeof it->name, "%s", w1);
                it->table = cur;
                it->x = a; it->y = b; it->w = c; it->h = d;
                s.tables[cur].items++;
            } else {
                s.item_overflow++;
            }
        }
    }
}

/* Find a sprite rect by sheet name + label (for the proof strip). */
static const HudEltItem *elt_find(const char *map, const char *label)
{
    for (int i = 0; i < s.item_count; i++) {
        const HudEltItem *it = &s.items[i];
        if (strcmp(it->name, label) != 0) continue;
        if (strcmp(s.tables[it->table].map, map) == 0) return it;
    }
    return NULL;
}

/*
 * Compass geometry — every number comes from the original vpit_1.elt:
 *
 *   dst zcm_.map  "compass_window"  window anchor inside the panel
 *                                    (its w/h are 0 in the data: an
 *                                    anchor, not a rect; the window
 *                                    takes the crop's size)
 *   src zcme.map  "left" / "right"  the same heading one full turn
 *                                    apart, so left's rect is the crop
 *                                    and right.x - left.x is the wrap
 *                                    period in pixels
 *
 * Anything absent or inconsistent rejects the whole compass and the HUD
 * loads without it — the compass is optional chrome, and a malformed
 * optional asset must not fail the load or the frame. Validates the
 * geometry once here so the render path never re-checks source bounds:
 * both labeled crops inside the strip bounds every intermediate crop
 * x in [left.x, right.x] as well.
 */
static int compass_validate(void)
{
    if (!s.compass || !s.compass_strip) return -1;
    const HudEltItem *win   = elt_find("zcm_.map", "compass_window");
    const HudEltItem *left  = elt_find("zcme.map", "left");
    const HudEltItem *right = elt_find("zcme.map", "right");
    if (!win || !left || !right) return -1;
    if (left->w <= 0 || left->h <= 0) return -1;
    if (right->w != left->w || right->h != left->h) return -1;
    if (right->y != left->y) return -1;      /* the crop slides in x only */
    int period = right->x - left->x;
    if (period <= 0) return -1;
    if (left->x < 0 || left->y < 0 ||
        left->x + left->w > s.cs_w || left->y + left->h > s.cs_h)
        return -1;
    if (right->x + right->w > s.cs_w || right->y + right->h > s.cs_h)
        return -1;
    if (win->x < 0 || win->y < 0 ||
        win->x + left->w > s.comp_w || win->y + left->h > s.comp_h)
        return -1;
    s.comp_win_x  = win->x;
    s.comp_win_y  = win->y;
    s.comp_x0     = left->x;
    s.comp_y0     = left->y;
    s.comp_crop_w = left->w;
    s.comp_crop_h = left->h;
    s.comp_period = period;
    return 0;
}

/*
 * Gauge geometry — the speedo/tach needle pivots on the decoded dash.
 * The .elt names the anchors (dst zspeedo6.map "speedneedleloc" /
 * dst ztach6.map "tachneedleloc") but pins them at (0,0) of their own
 * 128x128 face sheets — it never says where the gauges sit on the dash
 * (D5). The dash tiles themselves answer it: both baked gauge faces
 * carry a hub dot painted in the needle-red family, measured indices
 * 207/218/219 (the needle sheet's own ink is 207/217/218/220 — 217 and
 * 220 recur through dash body art, e.g. zdash501, and 240 is the
 * sheet's separator row, so the dot set is exactly 207/218/219).
 *
 * Detection: 8-connected clusters of those indices in the dash's top
 * half (the instrument tile; the radio/console tile below carries
 * unrelated red art — zdash401's LED blocks). A hub dot is a compact
 * blob: 6..81 px in a 3..9 px bounding box. Exactly two survivors,
 * split left/right by the dash midline, roughly level and clearly
 * separated, validate as speedo (left) and tach (right). Anything else
 * rejects the whole gauge overlay and the HUD loads without it — same
 * standing as the compass. Measured outcomes on the Nitro data:
 * zdash101/201/301 validate (hubs (125,43)/(208,41) dash-space);
 * zdash401 (a different cockpit: no round gauges) and zdash501 (left
 * dot smeared into adjacent red art) reject, so those scenarios keep
 * the static baked faces rather than needles at guessed positions.
 *
 * The needle frame rects (needle_1..38, uniform 79x79) are cached here
 * as well so the render path never string-scans the .elt per frame.
 */
static int gauge_validate(void)
{
    if (!s.dash || !s.needles) return -1;

    for (int i = 0; i < HUD_NEEDLE_FRAMES; i++) {
        char label[16];
        snprintf(label, sizeof label, "needle_%d", i + 1);
        const HudEltItem *it = elt_find("zneedle6.map", label);
        if (!it || it->w <= 1 || it->h <= 1) return -1;
        if (i == 0) {
            s.ndl_fw = it->w;
            s.ndl_fh = it->h;
        } else if (it->w != s.ndl_fw || it->h != s.ndl_fh) {
            return -1;              /* non-uniform frames: not a sweep */
        }
        if (it->x < 0 || it->y < 0 ||
            it->x + it->w > s.ndl_w || it->y + it->h > s.ndl_h)
            return -1;
        s.ndl_fx[i] = it->x;
        s.ndl_fy[i] = it->y;
    }

    /* Hub-dot scan. Clusters are flood-filled in place on a copy of the
     * top tile: visited dots are cleared as they are consumed. */
    static const uint8_t DOT[3] = { 207, 218, 219 };
    int cw = s.dash_w, ch = s.dash_h / 2;
    uint8_t *work = malloc((size_t)cw * (size_t)ch);
    int (*stack)[2] = malloc((size_t)cw * (size_t)ch * sizeof *stack);
    if (!work || !stack) {
        free(work);
        free(stack);
        return -1;
    }
    memcpy(work, s.dash, (size_t)cw * (size_t)ch);

    double hub[2][2];
    int found = 0;
    for (int y = 0; y < ch && found < 3; y++) {
        for (int x = 0; x < cw && found < 3; x++) {
            uint8_t v = work[(size_t)y * cw + x];
            if (v != DOT[0] && v != DOT[1] && v != DOT[2]) continue;
            /* flood fill this cluster, tracking count/bbox/sum */
            int n = 0, x0 = x, x1 = x, y0 = y, y1 = y;
            long sx = 0, sy = 0;
            int sp = 0;
            work[(size_t)y * cw + x] = 0;
            stack[sp][0] = x; stack[sp][1] = y; sp++;
            while (sp > 0) {
                sp--;
                int px = stack[sp][0], py = stack[sp][1];
                n++; sx += px; sy += py;
                if (px < x0) x0 = px;
                if (px > x1) x1 = px;
                if (py < y0) y0 = py;
                if (py > y1) y1 = py;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        int nx = px + dx, ny = py + dy;
                        if (nx < 0 || nx >= cw || ny < 0 || ny >= ch)
                            continue;
                        uint8_t nv = work[(size_t)ny * cw + nx];
                        if (nv != DOT[0] && nv != DOT[1] && nv != DOT[2])
                            continue;
                        work[(size_t)ny * cw + nx] = 0;
                        stack[sp][0] = nx; stack[sp][1] = ny; sp++;
                    }
                }
            }
            int bw = x1 - x0 + 1, bh = y1 - y0 + 1;
            if (n < 6 || n > 81 ||
                bw < 3 || bw > 9 || bh < 3 || bh > 9)
                continue;               /* noise or non-hub red art    */
            if (found < 2) {
                hub[found][0] = (double)sx / n;
                hub[found][1] = (double)sy / n;
            }
            found++;
        }
    }
    free(work);
    free(stack);
    if (found != 2) return -1;
    /* Left = speedo, right = tach (the two-gauge family layout). */
    int l = hub[0][0] <= hub[1][0] ? 0 : 1, r = 1 - l;
    if (hub[r][0] - hub[l][0] < 32.0) return -1;   /* clear separation */
    double dy = hub[0][1] - hub[1][1];
    if (dy < -16.0 || dy > 16.0) return -1;        /* roughly level    */
    s.spd_hub_x  = (int)(hub[l][0] + 0.5);
    s.spd_hub_y  = (int)(hub[l][1] + 0.5);
    s.tach_hub_x = (int)(hub[r][0] + 0.5);
    s.tach_hub_y = (int)(hub[r][1] + 0.5);
    return 0;
}

/*
 * Needle frame selection — FACT, recovered from nitro.exe's gauge code
 * (the software-path 2D sprite draw; the hardware path rotated a 3-D
 * needle by the equivalent angle):
 *
 *   speedo (0x450a80-0x450c68): value clamped to [0, 150] (constants
 *       @0x4c46bc/0x4c46c0 — the face reads mph), then
 *       frame = 9 - ftol(value * -0.1933333) (@0x4c46d4), clamped to
 *       38 — i.e. frame = 9 + trunc(mph * 0.1933333) over frames 9..38,
 *       rest at frame 9 (needle down-left, the face's 0 mark).
 *   tach (0x450c6b-0x450e14): value clamped to 8400 (@0x4c46d8), then
 *       frame = ftol(rpm * 0.003 + 1.0) (@0x4c46e0/0x4c46e4), clamped
 *       to 25 — frames 1..25, rest at frame 1 (needle down-right, the
 *       tach face's 0 mark).
 *
 * The sprite name is sprintf("3needle_%d", frame) (@0x4f98cc) cut from
 * the zneedle3.map labels; we render the zneedle6.map twin ("needle_%d")
 * — the same 38-frame rotation sweep at the 640-mode resolution,
 * matching the zspeedo6/ztach6 faces the binary loads in that mode and
 * the sheet this port already decodes. One measured sheet quirk: the
 * .elt frame rows after the first are pitch 81/80 and each such rect's
 * LAST row lands on a full-width separator line the artists left in the
 * sheet (index 240, white in the level palettes — nothing the original
 * could have blitted). The composite therefore blits h-1 rows of every
 * needle frame: the art never reaches that row (hub pivot at frame row
 * 39, longest tip at radius ~39), so nothing real is lost.
 *
 * ftol truncation is C's (int) cast on a non-negative double; the
 * float32 constants below are the exact bit patterns from the binary.
 */
static int gauge_speed_frame(double speed_mps)
{
    double mph = fabs(speed_mps) * 2.2369363;   /* m/s -> mph (port note:
                                                 * hud_render_frame takes
                                                 * m/s; the face is mph) */
    if (!isfinite(mph)) mph = 0.0;
    if (mph < 0.0) mph = 0.0;
    if (mph > 150.0) mph = 150.0;
    int frame = 9 + (int)(mph * 0.19333334267139435);
    if (frame > HUD_NEEDLE_FRAMES) frame = HUD_NEEDLE_FRAMES;
    return frame;
}

static int gauge_tach_frame(double rpm)
{
    if (!isfinite(rpm)) rpm = 0.0;
    if (rpm < 0.0) rpm = 0.0;
    if (rpm > 8400.0) rpm = 8400.0;
    int frame = (int)(rpm * 0.003000000026077032 + 1.0);
    if (frame > 25) frame = 25;
    return frame;
}

/* -----------------------------------------------------------------------
 * Blitter
 *
 * 0xFF is transparent for SPRITES and OPAQUE for the dashboard sheet, and
 * that difference is measured, not stylistic.
 *
 * The universal-0xFF rule this file used to apply cited a "pipeline.md §2.3"
 * that does not exist in the tree, and applying it to the dash produced a
 * cockpit that was 59.9% holes: ZDASH101 is 19,616 texels of 0xFF out of
 * 32,768, scattered through the panel body as a DITHER against indices 254,
 * 253, 252, 250 and 221. A shipped 1997 cockpit is not a stipple, and the
 * pattern is the giveaway -- 249..255 is a contiguous dark ramp at the top of
 * the level palette, so the art is dithering two ramp entries to get a shade
 * between them. In p01.act index 255 is BLACK.
 *
 * Rendering the same sheet with NO key against the level palette produces a
 * complete, solid dashboard: gauge faces, radio, switch banks, rounded bezel
 * corners. That is the check that settles it, and it is reproducible with
 * tools/hud_probe.c.
 *
 * Sprites keep the key. A needle or gear frame cut out of a sheet must not
 * paint its bounding box, and those sheets use 0xFF as genuine background.
 * Nobody has read nitro.exe's blit, so this is TWO measured behaviours rather
 * than one reversed rule -- which is why the parameter is explicit at every
 * call site instead of hidden in a default.
 * ----------------------------------------------------------------------- */

#define HUD_BLIT_OPAQUE (-1)
#define HUD_BLIT_KEYED  0xFF
#define HUD_BLIT_KEY1   0x01

static void blit_rect(uint8_t *fb, int fbw, int fbh,
                      const uint8_t *src, int sw,
                      int rx, int ry, int rw, int rh,
                      int dx, int dy, int keyed)
{
    for (int y = 0; y < rh; y++) {
        int fy = dy + y;
        if (fy < 0 || fy >= fbh) continue;
        int syy = ry + y;
        for (int x = 0; x < rw; x++) {
            int fx = dx + x;
            if (fx < 0 || fx >= fbw) continue;
            uint8_t v = src[(size_t)syy * sw + (size_t)(rx + x)];
            if (keyed >= 0 && v == (uint8_t)keyed) continue;
            fb[(size_t)fy * fbw + fx] = s.remap[v];
        }
    }
}

static void blit(uint8_t *fb, int fbw, int fbh,
                 const uint8_t *src, int sw, int sh,
                 int dx, int dy, int keyed)
{
    blit_rect(fb, fbw, fbh, src, sw, 0, 0, sw, sh, dx, dy, keyed);
}

static const char *const s_selector_label[6] = {
    "park", "reverse", "neutral", "drive", "second", "first"
};

/*
 * Build a display-order, level-palette image for an authored VGEO cockpit
 * surface. Callers composite their live sprites into this image, then flip the
 * completed image to raw texture-row order with live_map_finish().
 *
 * Source .map index 0xff stays 0xff: GER6 is opaque and therefore falls back
 * to its black face colour there, while CMP6/RTC6 are authored cut-outs and
 * need the world to remain visible. Everything else follows the same cockpit
 * palette remap as the established 2-D HUD blitter.
 */
static int live_map_prepare(uint8_t **live, RTex *tex,
                            const uint8_t *base, int w, int h)
{
    if (!base || w <= 0 || h <= 0 || w > UINT16_MAX || h > UINT16_MAX ||
        (w & (w - 1)) != 0 || (h & (h - 1)) != 0)
        return -1;
    size_t pixels = (size_t)w * (size_t)h;
    if (!*live) {
        *live = malloc(pixels);
        if (!*live) return -1;
    }
    int keyed = 0;
    for (size_t i = 0; i < pixels; i++) {
        uint8_t v = base[i];
        (*live)[i] = v == 0xff ? 0xff : s.remap[v];
        if (v == 0xff) keyed = 1;
    }
    uint8_t shift = 0;
    while ((1u << shift) < (unsigned)w) shift++;
    tex->texels = *live;
    tex->w = (uint16_t)w;
    tex->h = (uint16_t)h;
    tex->umask = (uint16_t)(w - 1);
    tex->vmask = (uint16_t)(h - 1);
    tex->vshift = shift;
    tex->has_key = (uint8_t)keyed;
    return 0;
}

/*
 * HUD map_decode returns display order for 2-D blits. GEO face UVs sample
 * bottom-up .map storage, as texcache_load_map does, so flip only after all
 * display-space overlays have been composited. Flipping the base first would
 * mirror and misplace the gear arrow and compass crop added by the callers.
 */
static void live_map_finish(uint8_t *live, int w, int h)
{
    for (int y = 0; y < h / 2; y++) {
        uint8_t *a = live + (size_t)y * (size_t)w;
        uint8_t *b = live + (size_t)(h - 1 - y) * (size_t)w;
        for (int x = 0; x < w; x++) {
            uint8_t t = a[x];
            a[x] = b[x];
            b[x] = t;
        }
    }
}

const RTex *hud_cockpit_gear_texture(int selector)
{
    if (!s.loaded || !s.gear || !s.gear_arrow) return NULL;
    if (selector < HUD_SELECTOR_PARK) selector = HUD_SELECTOR_PARK;
    if (selector > HUD_SELECTOR_FIRST) selector = HUD_SELECTOR_FIRST;
    if (s.gear_live_valid && selector == s.gear_live_selector)
        return &s.gear_live_tex;

    const HudEltItem *at = elt_find("zgear101.map",
                                    s_selector_label[selector]);
    if (!at || live_map_prepare(&s.gear_live, &s.gear_live_tex,
                                s.gear, s.gear_w, s.gear_h) != 0)
        return NULL;
    blit(s.gear_live, s.gear_w, s.gear_h,
         s.gear_arrow, s.ga_w, s.ga_h, at->x, at->y, HUD_BLIT_KEYED);
    live_map_finish(s.gear_live, s.gear_w, s.gear_h);
    s.gear_live_selector = selector;
    s.gear_live_valid = 1;
    return &s.gear_live_tex;
}

const RTex *hud_cockpit_compass_texture(void)
{
    if (!s.loaded || !s.comp_ok) return NULL;
    double f = fmod(-s.comp_yaw, HUD_COMPASS_TURN) / HUD_COMPASS_TURN;
    if (f < 0.0) f += 1.0;
    if (!isfinite(f)) f = 0.0;
    int cx = s.comp_x0 + (int)(f * (double)s.comp_period + 0.5);
    if (s.compass_live_valid && cx == s.compass_live_cx)
        return &s.compass_live_tex;
    if (live_map_prepare(&s.compass_live, &s.compass_live_tex,
                         s.compass, s.comp_w, s.comp_h) != 0)
        return NULL;
    blit_rect(s.compass_live, s.comp_w, s.comp_h,
              s.compass_strip, s.cs_w,
              cx, s.comp_y0, s.comp_crop_w, s.comp_crop_h,
              s.comp_win_x, s.comp_win_y, HUD_BLIT_KEYED);
    live_map_finish(s.compass_live, s.comp_w, s.comp_h);
    s.compass_live_cx = cx;
    s.compass_live_valid = 1;
    return &s.compass_live_tex;
}

const RTex *hud_cockpit_reticle_texture(void)
{
    if (!s.reticle_live) {
        if (live_map_prepare(&s.reticle_live, &s.reticle_live_tex,
                             s.reticle, s.reticle_w, s.reticle_h) != 0)
            return NULL;
        live_map_finish(s.reticle_live, s.reticle_w, s.reticle_h);
    }
    return &s.reticle_live_tex;
}

/*
 * Cockpit art class for a scenario — FACT (nitro.exe FUN @0x49f7e0 and
 * the shipped lookup table @0x503c88, dumped from the purchaser binary;
 * numbers decimal). The original derives the class digit of its
 * ZSWLS%1d01.tmt / ZNPD%1d01.map / ZHR45%1d01.tmt / ZHL45%1d01.tmt
 * cockpit-art names by parsing the current scenario name with
 * sscanf("%1s%2d") and mapping (letter, number) through this table.
 * Its defaults: sscanf failure -> 2; parsed but unlisted pair -> 1.
 */
static int dash_class_for_mission(const char *mission)
{
    static const struct { char kind; int num; int cls; } TAB[] = {
        { 'P',  1, 2 }, { 'P',  2, 2 }, { 'P',  3, 2 }, { 'P',  4, 2 },
        { 'P', 17, 2 }, { 'P', 19, 2 },
        { 'P',  5, 3 }, { 'P',  6, 3 }, { 'P',  7, 3 }, { 'P',  8, 3 },
        { 'P', 18, 3 }, { 'B',  1, 3 },
        { 'P',  9, 4 }, { 'P', 10, 4 }, { 'P', 11, 4 }, { 'P', 12, 4 },
        { 'P', 13, 5 }, { 'P', 14, 5 }, { 'P', 15, 5 }, { 'P', 16, 5 },
    };
    const char *base = strrchr(mission, '/');
    base = base ? base + 1 : mission;
    char kind = base[0];
    if (kind >= 'a' && kind <= 'z') kind = (char)(kind - ('a' - 'A'));
    if (kind < 'A' || kind > 'Z') return 2;   /* "%1s" needs a letter  */
    int num = 0, digits = 0;
    for (int i = 1; base[i] >= '0' && base[i] <= '9' && digits < 2; i++) {
        num = num * 10 + (base[i] - '0');     /* "%2d" — stops at '.'  */
        digits++;
    }
    if (digits == 0) return 2;
    for (size_t i = 0; i < sizeof TAB / sizeof TAB[0]; i++)
        if (TAB[i].kind == kind && TAB[i].num == num)
            return TAB[i].cls;
    return 1;
}

/*
 * Glance/sidearm art family (FACT names: nitro.exe ZHL45%1d01.tmt /
 * ZHR45%1d01.tmt format strings; the class digit is the scenario art
 * class). The pix manifest lists every VQM frame in archive order;
 * frames are decoded like the dash tiles and exported with their
 * manifest names (hud_sidearm_frame*), semantics left to the consumer.
 * Set-1 fallback mirrors the dash rule (D4). Optional chrome: a missing
 * family degrades to count 0, never to a failed load.
 */
static void sidearm_load(int side, int art_class)
{
    char pix_name[24], pak_name[24];
    snprintf(pix_name, sizeof pix_name, "zh%c45%d01.pix",
             side == HUD_SIDEARM_LEFT ? 'l' : 'r', art_class);
    snprintf(pak_name, sizeof pak_name, "zh%c45%d01.pak",
             side == HUD_SIDEARM_LEFT ? 'l' : 'r', art_class);
    if (art_class != 1 &&
        (vfs_exists(pix_name) <= 0 || vfs_exists(pak_name) <= 0)) {
        snprintf(pix_name, sizeof pix_name, "zh%c45101.pix",
                 side == HUD_SIDEARM_LEFT ? 'l' : 'r');
        snprintf(pak_name, sizeof pak_name, "zh%c45101.pak",
                 side == HUD_SIDEARM_LEFT ? 'l' : 'r');
    }
    size_t pix_n = 0, pak_n = 0;
    uint8_t *pix = vfs_read_file(pix_name, &pix_n);
    uint8_t *pak = vfs_read_file(pak_name, &pak_n);
    if (!pix || !pak) {
        if (pix) vfs_free(pix);
        if (pak) vfs_free(pak);
        return;
    }
    char *text = malloc(pix_n + 1);
    if (text) {
        memcpy(text, pix, pix_n);
        text[pix_n] = '\0';
        PixEntry entries[HUD_MAX_SIDEARM_FRAMES];
        int te = pix_parse(text, entries, HUD_MAX_SIDEARM_FRAMES);
        for (int i = 0; i < te && s.sa_count[side] < HUD_MAX_SIDEARM_FRAMES;
             i++) {
            if ((size_t)entries[i].off + entries[i].len > pak_n) {
                fprintf(stderr, "[hud] %s: pix entry outside pak\n",
                        entries[i].name);
                continue;
            }
            int w = 0, h = 0;
            uint8_t *px = vqm_decode(pak + entries[i].off, entries[i].len,
                                     s.orient == HUD_ORIENT_COLUMN, &w, &h);
            if (!px) continue;
            int slot = s.sa_count[side]++;
            s.sa_frame[side][slot] = px;
            s.sa_fw[side][slot] = w;
            s.sa_fh[side][slot] = h;
            snprintf(s.sa_name[side][slot], sizeof s.sa_name[side][slot],
                     "%s", entries[i].name);
        }
        free(text);
    }
    vfs_free(pix);
    vfs_free(pak);
}

/* -----------------------------------------------------------------------
 * Public API
 * ----------------------------------------------------------------------- */

static int hud_load_impl(const char *preferred_pal, int dash_class)
{
    hud_unload();
    s.art_class = dash_class;

    /* Palette (D3 + WRLD resolution) + index remap (meshview fallback). */
    s.have_pal = (load_palette(preferred_pal) == 0);
    if (s.have_pal) {
        for (int i = 0; i < 256; i++) s.remap[i] = (uint8_t)i;
        s.text_color = brightest_index();
    } else {
        fprintf(stderr, "[hud] no .act palette found; "
                "falling back to meshview indices\n");
        for (int i = 0; i < 256; i++)
            s.remap[i] = (uint8_t)(i == 0 ? 0 : 1 + (i >> 6)); /* 0..4 */
        s.text_color = 3; /* meshview white */
    }

    /* Dashboard: pix manifest + pak slices -> stacked tiles (D4). The
     * set is scenario-derived (dash_class_for_mission, FACT); set 1 is
     * the fallback when the derived set is absent from the data. */
    char pix_name[20], pak_name[20];
    snprintf(s.dash_base, sizeof s.dash_base, "zdash%d01", dash_class);
    snprintf(pix_name, sizeof pix_name, "%s.pix", s.dash_base);
    snprintf(pak_name, sizeof pak_name, "%s.pak", s.dash_base);
    if (dash_class != 1 &&
        (vfs_exists(pix_name) <= 0 || vfs_exists(pak_name) <= 0)) {
        fprintf(stderr, "[hud] %s/%s absent — falling back to zdash101\n",
                pix_name, pak_name);
        snprintf(s.dash_base, sizeof s.dash_base, "zdash101");
        snprintf(pix_name, sizeof pix_name, "%s.pix", s.dash_base);
        snprintf(pak_name, sizeof pak_name, "%s.pak", s.dash_base);
    }
    size_t pix_n = 0, pak_n = 0;
    uint8_t *pix = vfs_read_file(pix_name, &pix_n);
    uint8_t *pak = vfs_read_file(pak_name, &pak_n);
    if (pix && pak) {
        PixEntry entries[HUD_MAX_PIX_ENTRIES];
        char *text = malloc(pix_n + 1);
        if (text) {
            memcpy(text, pix, pix_n);
            text[pix_n] = '\0';
            int te = pix_parse(text, entries, HUD_MAX_PIX_ENTRIES);
            uint8_t *tiles[HUD_MAX_PIX_ENTRIES] = {NULL};
            int      tile_h[HUD_MAX_PIX_ENTRIES] = {0};
            int tw = 0, th = 0, ok = 0;
            for (int i = 0; i < te; i++) {
                if ((size_t)entries[i].off + entries[i].len > pak_n) {
                    fprintf(stderr, "[hud] %s: pix entry outside pak\n",
                            entries[i].name);
                    continue;
                }
                int w = 0, h = 0;
                tiles[i] = vqm_decode(pak + entries[i].off, entries[i].len,
                                      s.orient == HUD_ORIENT_COLUMN, &w, &h);
                if (!tiles[i]) continue;
                if (ok == 0) tw = w;
                if (w != tw) {  /* unexpected; drop mismatched tile */
                    fprintf(stderr, "[hud] %s: tile width %d != %d\n",
                            entries[i].name, w, tw);
                    free(tiles[i]);
                    tiles[i] = NULL;
                    continue;
                }
                tile_h[i] = h;
                th += h;
                ok++;
            }
            if (ok > 0) {
                s.dash = malloc((size_t)tw * (size_t)th);
                if (s.dash) {
                    int oy = 0;
                    for (int i = 0; i < te; i++) {
                        if (!tiles[i]) continue;
                        memcpy(s.dash + (size_t)oy * (size_t)tw, tiles[i],
                               (size_t)tw * (size_t)tile_h[i]);
                        oy += tile_h[i];
                    }
                    s.dash_w = tw;
                    s.dash_h = th;
                    s.dash_tiles = ok;
                }
            }
            for (int i = 0; i < te; i++) free(tiles[i]);
            free(text);
        }
    }
    if (pix) vfs_free(pix);
    if (pak) vfs_free(pak);

    /* Sprite tables (text) + the sheets the proof strip draws (D5). */
    size_t elt_n = 0;
    uint8_t *elt = vfs_read_file("vpit_1.elt", &elt_n);
    if (elt) {
        char *text = malloc(elt_n + 1);
        if (text) {
            memcpy(text, elt, elt_n);
            text[elt_n] = '\0';
            elt_parse(text);
            free(text);
        }
        vfs_free(elt);
    }
    s.gear = map_load("zgear101.map", &s.gear_w, &s.gear_h);
    s.gear_arrow = map_load("zgeare.map", &s.ga_w, &s.ga_h);
    s.needles = map_load("zneedle6.map", &s.ndl_w, &s.ndl_h);

    s.radar_mask = map_load("zradmask.map", &s.rad_w, &s.rad_h);
    radar_load();               /* H-UAT-002: authored green sweep unit */
    s.weapon_panel = map_load("zwpe.map", &s.wep_w, &s.wep_h);
    s.damage_panel = map_load("zsy_.map", &s.dmg_w, &s.dmg_h);
    s.damage_states = map_load("zsye.map", &s.ds_w, &s.ds_h);
    s.weapon_labels = map_load("zdue.map", &s.wl_w, &s.wl_h);
    s.weapon_digits = map_load("znbe.map", &s.wd_w, &s.wd_h);
    s.target_led = map_load("zdde.map", &s.tl_w, &s.tl_h);
    /* Speedo/tach needles: hub geometry from the dash art + the .elt
     * frame rects (see gauge_validate). Optional like the compass:
     * rejected geometry degrades to the static baked faces. */
    s.gauge_ok = (gauge_validate() == 0);
    if (!s.gauge_ok)
        fprintf(stderr, "[hud] gauge hubs not found on this dash — "
                "needles disabled\n");

    /* Compass: panel + bearing strip, geometry from the .elt (see
     * compass_validate). Optional: rejected geometry degrades to no
     * compass, never to a failed HUD load. */
    s.compass = map_load("zcm_.map", &s.comp_w, &s.comp_h);
    s.compass_strip = map_load("zcme.map", &s.cs_w, &s.cs_h);
    s.reticle = map_load("zretc_6.map", &s.reticle_w, &s.reticle_h);
    s.comp_ok = (compass_validate() == 0);
    if (!s.comp_ok && (s.compass || s.compass_strip))
        fprintf(stderr, "[hud] compass assets missing/malformed — "
                "compass disabled\n");

    /* Authored interior-mirror mask + bezel (optional chrome, same
     * standing as the compass): decoded for the rearview composite —
     * see hud_mirror_mask()/hud_mirror_bezel(). Only the set-1 day
     * variant ships in the Nitro data. */
    s.mirror_mask  = map_load("zmiri101.map", &s.mm_w, &s.mm_h);
    s.mirror_bezel = map_load("zmiro101.map", &s.mb_w, &s.mb_h);

    /* Glance/sidearm families at the scenario art class (optional). */
    sidearm_load(HUD_SIDEARM_LEFT, dash_class);
    sidearm_load(HUD_SIDEARM_RIGHT, dash_class);

    /* Font (loose file in the app root, read through the VFS). */
    s.font = font_load("base6x7.fnt");
    if (s.font)
        snprintf(s.font_name, sizeof s.font_name, "%s", "base6x7.fnt");

    s.loaded = (s.dash != NULL);
    if (!s.loaded) {
        fprintf(stderr, "[hud] dashboard tiles failed to load\n");
        s.art_class = 0;
        return -1;
    }
    return 0;
}

int hud_scenario_art_class(const char *mission)
{
    if (!mission || !mission[0]) return 1;
    return dash_class_for_mission(mission);
}

int hud_art_class(void)
{
    return s.art_class;
}

int hud_load(void)
{
    return hud_load_impl(NULL, 1);
}

int hud_load_mission(const char *mission, int car_id)
{
    (void)car_id;   /* D4: dash set is scenario-keyed (FACT); no
                     * vehicle->dash-set mapping exists in the data. */
    char pal[16] = {0};
    const char *preferred = NULL;
    if (mission && mission_palette_name(mission, pal) == 0)
        preferred = pal;
    int dash_class = (mission && mission[0])
                   ? dash_class_for_mission(mission) : 1;
    return hud_load_impl(preferred, dash_class);
}

void hud_unload(void)
{
    free(s.dash);    s.dash = NULL;
    free(s.gear);    s.gear = NULL;
    free(s.gear_arrow); s.gear_arrow = NULL;
    free(s.gear_live); s.gear_live = NULL;
    free(s.needles); s.needles = NULL;
    free(s.compass); s.compass = NULL;
    free(s.compass_strip); s.compass_strip = NULL;
    free(s.compass_live); s.compass_live = NULL;
    free(s.reticle); s.reticle = NULL;
    free(s.reticle_live); s.reticle_live = NULL;
    free(s.mirror_mask);  s.mirror_mask = NULL;
    free(s.mirror_bezel); s.mirror_bezel = NULL;
    free(s.radar_mask);   s.radar_mask = NULL;
    for (int i = 0; i < HUD_RADAR_FRAMES; i++) {
        free(s.radar_sweep[i]);
        s.radar_sweep[i] = NULL;
    }
    s.radar_sweep_count = 0;
    free(s.radar_overlay); s.radar_overlay = NULL;
    s.radar_contact_count = 0;
    s.cond_set = 0;
    free(s.weapon_panel); s.weapon_panel = NULL;
    free(s.damage_panel); s.damage_panel = NULL;
    free(s.damage_states); s.damage_states = NULL;
    free(s.weapon_labels); s.weapon_labels = NULL;
    free(s.weapon_digits); s.weapon_digits = NULL;
    free(s.target_led); s.target_led = NULL;
    s.tgt_set = 0;
    for (int d = 0; d < HUD_SIDEARM_SIDES; d++)
        for (int i = 0; i < HUD_MAX_SIDEARM_FRAMES; i++) {
            free(s.sa_frame[d][i]);
            s.sa_frame[d][i] = NULL;
        }
    if (s.font) { font_free(s.font); s.font = NULL; }
    int orient = s.orient;
    char text[HUD_TEXT_CAP];
    int vhp = s.vit_hp, vmax = s.vit_hp_max, vfuel = s.vit_fuel;
    double cyaw = s.comp_yaw;
    double erpm = s.eng_rpm;
    memcpy(text, s.text, sizeof text);
    memset(&s, 0, sizeof s);
    s.orient = orient;                 /* experiment knob survives unload */
    memcpy(s.text, text, sizeof text); /* as does the host-set text line */
    s.vit_hp = vhp;                    /* and the M7 vitals */
    s.vit_hp_max = vmax;
    s.vit_fuel = vfuel;
    s.comp_yaw = cyaw;                 /* and the live heading */
    s.eng_rpm = erpm;                  /* and the live engine rpm */
}

static const char *weapon_label_base(const char *name)
{
    static const struct { const char *name, *asset; } labels[] = {
        { "20mm Cannon", "20mm_can" }, { "25mm Cannon", "25mm_can" },
        { "30mm Cannon", "30mm_can" }, { "30cal MG", "30cal_mg" },
        { "50cal MG", "50cal_mg" }, { "7.62mm MG", "762_mg" },
        { "20mm Turret", "20mm_trt" }, { "25mm Turret", "25mm_trt" },
        { "30mm Turret", "30mm_trt" }, { "30cal Turret", "30cal_trt" },
        { "50cal Turret", "50cal_trt" }, { "7.62 Turret", "762_trt" },
        { "FireRite Rkt", "fr_rocket" }, { "FireRite Trt", "fr_trt" },
        { "Aim-Nein Msl", "an_missile" }, { "Aim-Nein Trt", "aim_trt" },
        { "DrRadar Msl", "dr_missile" }, { "DrRadar Trt", "dr_trt" },
        { "Gas Launcher", "gaslaunchr" }, { "FlameThrower", "flame" },
        { "Napalm Hose", "napalm" }, { "HE Mortar", "he_mortar" },
        { "4Get-U-Not Msl", "4get_u_not" },
        { "Chemical Mortar", "chem_bomb" },
        /* Purchaser GDFC +132/+148 names carry the same stems with a
         * leading `3` for the 320 sheet. vpit_1.elt's 640 zdue table authors
         * these exact six on/off pairs (H-UAT-079d). */
        { "Oil Slick", "oilslick" }, { "Fire-Dropper", "firedroppr" },
        { "Landmines", "landmines" }, { "Caltrops", "caltrops" },
        { "BloxDropper", "bloxdroppr" }, { "Car-E-Racer", "car_eraser" }
    };
    for (size_t i = 0; i < sizeof labels / sizeof labels[0]; i++)
        if (strcmp(name, labels[i].name) == 0)
            return labels[i].asset;
    return NULL;
}

static void render_weapon_rows(uint8_t *fb, int w, int h, int panel_x)
{
    s.weapon_authored_rows = 0;
    s.weapon_fallback_rows = 0;
    s.weapon_ammo_rows = 0;
    if (!s.weapon_labels && !s.font && !s.weapon_digits) return;
    for (int row = 0; row < HUD_WEAPON_ROWS; row++) {
        if (!s.weapon_row_name[row][0])
            continue;
        const char *base = weapon_label_base(s.weapon_row_name[row]);
        /* "on" for every armed hardpoint (selected + class-linked peers),
         * not only the keyboard-focus slot — matches stock zwpe panel. */
        int lit = (row == s.weapon_slot) ||
                  (row < s.weapon_count && s.weapon_row_armed[row]);
        const HudEltItem *art = NULL;
        if (base && s.weapon_labels) {
            char label[24];
            snprintf(label, sizeof label, "%s_%s", base,
                     lit ? "on" : "off");
            art = elt_find("zdue.map", label);
        }
        if (art) {
            blit_rect(fb, w, h, s.weapon_labels, s.wl_w,
                      art->x, art->y, art->w, art->h,
                      panel_x + 32, row * 24 + 1, HUD_BLIT_OPAQUE);
            s.weapon_authored_rows |= 1u << row;
        } else if (s.font) {
            /* Unknown/future GDFs must not erase the whole row. Use the same
             * purchaser bitmap font as the rest of the HUD, clipped by
             * truncating to the authored 93-pixel label bay. */
            char fallback[sizeof s.weapon_row_name[row]];
            snprintf(fallback, sizeof fallback, "%s", s.weapon_row_name[row]);
            size_t len = strlen(fallback);
            while (len > 0 && font_text_width(s.font, fallback) > 91)
                fallback[--len] = '\0';
            if (fallback[0]) {
                font_draw(s.font, fallback, fb, w, h,
                          panel_x + 34, row * 24 + 8,
                          s.remap[s.text_color]);
                s.weapon_fallback_rows |= 1u << row;
            }
        }

        if (s.weapon_digits) {
            int ammo = s.weapon_row_ammo[row];
            if (ammo < 0) ammo = 9999;
            if (ammo > 9999) ammo = 9999;
            for (int digit = 0, place = 1000; digit < 4;
                 digit++, place /= 10) {
                int value = (ammo / place) % 10;
                blit_rect(fb, w, h, s.weapon_digits, s.wd_w,
                          0, value * 10, 8, 10,
                          panel_x + 127 + digit * 9, row * 24 + 7,
                          HUD_BLIT_OPAQUE);
            }
            s.weapon_ammo_rows |= 1u << row;
        }
    }
}

static const char *condition_state(uint32_t flags)
{
    if (!(flags & HUD_FLAG_VITALS) || s.vit_hp_max <= 0)
        return "off";
    if (s.vit_hp <= 0)
        return "drk";
    int64_t scaled = (int64_t)s.vit_hp * 3;
    if (scaled > (int64_t)s.vit_hp_max * 2)
        return "grn";
    if (scaled > s.vit_hp_max)
        return "ylw";
    return "red";
}

/*
 * The condition panel's base MAP deliberately leaves its live regions blank.
 * vpit_1.elt supplies both the destination anchors in zsy_.map and the
 * state-specific source rectangles in zsye.map. H-UAT-003: when a host feeds
 * per-component state (hud_set_condition, from the combat model's facet
 * pools — the original's independent component/facet condition), each region
 * follows its OWN component and the regions change independently. Without
 * component state the regions keep the legacy behaviour of following the one
 * scalar damage pool together. The asset geometry and palette states
 * themselves are authored data.
 */
static const char *condition_name_for(double hp, double hp_max)
{
    if (hp_max <= 0)
        return "off";
    if (hp <= 0)
        return "drk";
    if (hp * 3 > hp_max * 2)
        return "grn";
    if (hp * 3 > hp_max)
        return "ylw";
    return "red";
}

static void render_condition_states(uint8_t *fb, int w, int h, int panel_x,
                                    uint32_t flags)
{
    static const struct {
        const char *dst;
        const char *src;
        int         comp;   /* combat.h COMBAT_COMP_* order */
    } items[] = {
        { "engine", "engine", 0 }, { "suspen", "suspen", 1 },
        { "brakes", "brakes", 2 },
        { "rrtire", "tire", 3 }, { "rltire", "tire", 4 },
        { "frtire", "tire", 5 }, { "fltire", "tire", 6 },
        { "farm", "farm", 7 }, { "rarm", "rarm", 8 },
        { "larm", "larm", 9 }, { "barm", "barm", 10 },
        { "fchas", "fchas", 11 }, { "rchas", "rchas", 12 },
        { "lchas", "lchas", 13 }, { "bchas", "bchas", 14 },
    };
    if (!s.damage_states)
        return;

    const char *state = condition_state(flags);
    for (size_t i = 0; i < sizeof items / sizeof items[0]; i++) {
        const HudEltItem *dst = elt_find("zsy_.map", items[i].dst);
        char label[24];
        const char *st = state;
        if (s.cond_set && items[i].comp >= 0 &&
            items[i].comp < HUD_CONDITION_COMPONENTS)
            st = condition_name_for((double)s.cond_hp[items[i].comp],
                                    (double)s.cond_max[items[i].comp]);
        snprintf(label, sizeof label, "%s_%s", items[i].src, st);
        const HudEltItem *src = elt_find("zsye.map", label);
        if (!dst || !src || src->x < 0 || src->y < 0 ||
            src->w <= 0 || src->h <= 0 ||
            src->x + src->w > s.ds_w || src->y + src->h > s.ds_h)
            continue;
        blit_rect(fb, w, h, s.damage_states, s.ds_w,
                  src->x, src->y, src->w, src->h,
                  panel_x + dst->x, dst->y, HUD_BLIT_KEYED);
    }
}


/*
 * H-UAT-002: the authored radar unit in its housing. The sweep frame is
 * the 20 Hz tick's ZRADF tile (lap_ms is tick-derived: webmain passes
 * ticks*50); contact blips clip to the playfield circle measured from
 * the mask at load; the overlay restores the housing chrome and
 * crosshair over both. Drawn identically in cockpit and chase — the
 * original retains the enclosed radar in every drive view, which is
 * what removes the unframed over-sky markers this entry reported.
 */
static void render_radar(uint8_t *fb, int w, int h, uint32_t lap_ms)
{
    if (s.radar_sweep_count > 0) {
        int frame = (int)(lap_ms / 50u) % s.radar_sweep_count;
        blit(fb, w, h, s.radar_sweep[frame], s.radar_sweep_w,
             s.radar_sweep_h, 0, 0, HUD_BLIT_KEYED);
        for (int i = 0; i < s.radar_contact_count && s.radar_r > 4; i++) {
            double sx = s.radar_cx +
                        s.radar_contacts[i].right * s.radar_r /
                        HUD_RADAR_RANGE_M;
            double sy = s.radar_cy -
                        s.radar_contacts[i].fwd * s.radar_r /
                        HUD_RADAR_RANGE_M;
            uint8_t idx = s.radar_contacts[i].threat
                        ? s.radar_threat_idx : s.radar_ally_idx;
            int px = (int)(sx + 0.5), py = (int)(sy + 0.5);
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    int cx = px + dx - s.radar_cx;
                    int cy = py + dy - s.radar_cy;
                    if (cx * cx + cy * cy > (s.radar_r - 2) *
                                             (s.radar_r - 2))
                        continue;
                    int fx = px + dx, fy = py + dy;
                    if ((unsigned)fx < (unsigned)w &&
                        (unsigned)fy < (unsigned)h)
                        fb[(size_t)fy * w + fx] = s.remap[idx];
                }
        }
        if (s.radar_overlay)
            blit(fb, w, h, s.radar_overlay, s.rad_w, s.rad_h,
                 0, 0, HUD_BLIT_KEYED);
    } else if (s.radar_mask) {
        /* No sweep tiles in the data: the legacy housing-only blit. */
        blit(fb, w, h, s.radar_mask, s.rad_w, s.rad_h,
             0, 0, HUD_BLIT_KEY1);
    }
}

/*
 * H-UAT-011c: target condition readout (hud.h contract). The LED and the
 * range digits sit on the authored vpit_1.elt "dst zrad.map" anchors —
 * led_pos (150,84), range_pos (139,4) — resolved in radar-housing space
 * (PORT DECISION: zrad.map is absent from nitro.zfs; both anchors land
 * on housing chrome outside the measured playfield circle). LED states
 * are the authored zdde.map sprites; the LED follows the target's live
 * scalar pool through the same 2/3-1/3 thresholds as the condition
 * panel. The name line and thin bar under the housing are PORT
 * PRESENTATION — no authored target-name surface is decoded.
 */
static void render_target_readout(uint8_t *fb, int w, int h)
{
    if (!s.tgt_set)
        return;
    if (s.target_led) {
        const char *st = condition_name_for((double)s.tgt_hp,
                                            (double)s.tgt_hp_max);
        const char *label = strcmp(st, "grn") == 0 ? "green"
                          : strcmp(st, "ylw") == 0 ? "yellow"
                          : strcmp(st, "red") == 0 ? "red"
                          : strcmp(st, "drk") == 0 ? "drk"
                          : "off";
        const HudEltItem *src = elt_find("zdde.map", label);
        if (src && src->x >= 0 && src->y >= 0 && src->w > 0 && src->h > 0 &&
            src->x + src->w <= s.tl_w && src->y + src->h <= s.tl_h)
            blit_rect(fb, w, h, s.target_led, s.tl_w,
                      src->x, src->y, src->w, src->h,
                      150, 84, HUD_BLIT_KEYED);
    }
    if (s.weapon_digits) {
        int range = s.tgt_range_m;
        if (range < 0) range = 0;
        if (range > 9999) range = 9999;
        for (int digit = 0, place = 1000; digit < 4;
             digit++, place /= 10) {
            int value = (range / place) % 10;
            blit_rect(fb, w, h, s.weapon_digits, s.wd_w,
                      0, value * 10, 8, 10,
                      139 + digit * 9, 4, HUD_BLIT_OPAQUE);
        }
    }
    int ty = s.rad_h + 2;
    if (s.font) {
        char line[64];
        snprintf(line, sizeof line, "TGT %s", s.tgt_name);
        font_draw(s.font, line, fb, w, h, 8, ty,
                  s.remap[s.text_color]);
        ty += (int)s.font->height + 2;
    }
    if (s.tgt_hp_max > 0) {
        const int bx = 8, bw = 52, bh = 4;
        int hp = s.tgt_hp > 0 ? s.tgt_hp : 0;
        int fill = (int)((double)hp * (bw - 2) / s.tgt_hp_max + 0.5);
        uint8_t c = s.remap[s.text_color];
        for (int y = 0; y < bh; y++)
            for (int x = 0; x < bw; x++) {
                int edge = y == 0 || y == bh - 1 || x == 0 || x == bw - 1;
                int lit = edge || (x - 1 < fill);
                int fx2 = bx + x, fy2 = ty + y;
                if (lit && (unsigned)fx2 < (unsigned)w &&
                    (unsigned)fy2 < (unsigned)h)
                    fb[(size_t)fy2 * w + fx2] = c;
            }
    }
}

/*
 * Original-video upper layout at 640x480: radar housing top-left, weapon rack
 * top-centre, condition panel top-right. Lower instruments belong on the
 * unresolved cockpit/dashboard surface; drawing them alone makes them float
 * over the road. ZDASH remains a decoder source, not a screen-sized overlay.
 */
static void render_cockpit_layout(uint8_t *fb, int w, int h, uint32_t flags,
                                  uint32_t lap_ms)
{
    render_radar(fb, w, h, lap_ms);
    render_target_readout(fb, w, h);
    if (s.weapon_panel) {
        int x = (w * 3) / 10;
        blit(fb, w, h, s.weapon_panel, s.wep_w, s.wep_h,
             x, 0, HUD_BLIT_OPAQUE);
        render_weapon_rows(fb, w, h, x);
    }
    if (s.damage_panel) {
        int x = w - s.dmg_w;
        blit(fb, w, h, s.damage_panel, s.dmg_w, s.dmg_h,
             x, 0, HUD_BLIT_OPAQUE);
        render_condition_states(fb, w, h, x, flags);
    }
}

void hud_render_frame(uint8_t *fb, int w, int h,
                      int selector, double speed, uint32_t lap_ms,
                      uint32_t flags)
{
    if (selector < HUD_SELECTOR_PARK) selector = HUD_SELECTOR_PARK;
    if (selector > HUD_SELECTOR_FIRST) selector = HUD_SELECTOR_FIRST;
    if (!fb || w <= 0 || h <= 0) return;
    memset(fb, 0, (size_t)w * (size_t)h);
    if (!s.loaded) return;

    if (flags & HUD_FLAG_COCKPIT_LAYOUT)
        render_cockpit_layout(fb, w, h, flags, lap_ms);
    else if (!(flags & HUD_FLAG_NO_DASH))
        blit(fb, w, h, s.dash, s.dash_w, s.dash_h,
             (w - s.dash_w) / 2, h - s.dash_h, HUD_BLIT_OPAQUE);

    /*
     * Speedo/tach needles — dashboard chrome gated with the dash, drawn
     * over the baked gauge faces at the validated hub dots (D8). Each
     * frame is cut from zneedle6.map per the .elt rects, minus the last
     * row (the sheet's separator line, D8), pivot at the frame's
     * measured hub pixel (39,39), keyed like every sprite (D7). Frame
     * selection follows the original's own formulas (gauge_speed_frame/
     * gauge_tach_frame): speed comes with the frame parameters, RPM via
     * hud_set_engine_rpm (0 = needle at rest until a host feeds it).
     */
    if (!(flags & (HUD_FLAG_NO_DASH | HUD_FLAG_COCKPIT_LAYOUT)) &&
        s.gauge_ok) {
        int dx0 = (w - s.dash_w) / 2, dy0 = h - s.dash_h;
        int sf = gauge_speed_frame(speed) - 1;   /* 0-based sheet index */
        int tf = gauge_tach_frame(s.eng_rpm) - 1;
        blit_rect(fb, w, h, s.needles, s.ndl_w,
                  s.ndl_fx[sf], s.ndl_fy[sf], s.ndl_fw, s.ndl_fh - 1,
                  dx0 + s.spd_hub_x - 39, dy0 + s.spd_hub_y - 39,
                  HUD_BLIT_KEYED);
        blit_rect(fb, w, h, s.needles, s.ndl_w,
                  s.ndl_fx[tf], s.ndl_fy[tf], s.ndl_fw, s.ndl_fh - 1,
                  dx0 + s.tach_hub_x - 39, dy0 + s.tach_hub_y - 39,
                  HUD_BLIT_KEYED);
    }

    /*
     * Compass — dashboard chrome like the gear indicator, drawn only with
     * the dash: the zcm_.map panel sits immediately left of the dash with
     * its bottom edge on the dash's top edge, and the current bearing crop
     * of the zcme.map strip blits into the panel's compass_window. The
     * panel is a panel (OPAQUE, D7); the strip is a src sheet and keeps
     * the 0xFF key — moot on the shipped data, whose whole crop slide
     * range contains no 0xFF texel, but correct for a sprite sheet.
     *
     * Car yaw is 0 at north (+Z) and positive turning LEFT; the strip is
     * indexed by CLOCKWISE bearing, hence the negate. f in [0,1) turns of
     * a full circle maps linearly onto [left.x, right.x]; yaw 0 crops at
     * the left label (north) and f approaching 1 approaches the right
     * label — the same heading one full turn on, which is the wrap.
     * comp_ok (compass_validate at load) already bounded every crop x in
     * that range inside the sheet, so the blit needs no source re-check.
     * The .elt anchors are panel-space; the panel's own dash-relative
     * spot is not in the data, so left-of-dash is a port decision, same
     * standing as the gear strip's above-dash placement.
     */
    if (!(flags & (HUD_FLAG_NO_DASH | HUD_FLAG_COCKPIT_LAYOUT)) &&
        s.comp_ok) {
        int px = (w - s.dash_w) / 2 - s.comp_w;
        int py = h - s.dash_h - s.comp_h;
        blit(fb, w, h, s.compass, s.comp_w, s.comp_h,
             px, py, HUD_BLIT_OPAQUE);
        double f = fmod(-s.comp_yaw, HUD_COMPASS_TURN) / HUD_COMPASS_TURN;
        if (f < 0.0) f += 1.0;
        if (!isfinite(f)) f = 0.0;    /* NaN/Inf yaw: show north, never UB */
        int cx = s.comp_x0 + (int)(f * (double)s.comp_period + 0.5);
        blit_rect(fb, w, h, s.compass_strip, s.cs_w,
                  cx, s.comp_y0, s.comp_crop_w, s.comp_crop_h,
                  px + s.comp_win_x, py + s.comp_win_y, HUD_BLIT_KEYED);
    }

    /*
     * Gear indicator — the one gauge composite whose inputs are ALL
     * decoded: the zgear101 plate strip, the zgeare arrow sprite, the
     * arrow's per-gear anchors from the .elt's own dst table
     * (park/reverse/neutral/drive/second/first, left to right = the
     * selector param's PRND21 order), and the selector state the caller
     * passes. The strip is dash-width and sits directly above the dash,
     * the steering-column position it occupies in the original's art
     * direction; its exact dash-space offset is NOT in the .elt (D5),
     * so the above-dash placement is a port decision, not a measured
     * one. The plate blits OPAQUE like the dash (a panel, D7); the
     * arrow is a sprite and keeps the 0xFF key.
     */
    if (!(flags & (HUD_FLAG_NO_DASH | HUD_FLAG_COCKPIT_LAYOUT)) &&
        s.gear && s.gear_arrow) {
        int gx = (w - s.gear_w) / 2;
        int gy = h - s.dash_h - s.gear_h;
        blit(fb, w, h, s.gear, s.gear_w, s.gear_h, gx, gy, HUD_BLIT_OPAQUE);
        const HudEltItem *at = elt_find("zgear101.map",
                                        s_selector_label[selector]);
        if (at)
            blit(fb, w, h, s.gear_arrow, s.ga_w, s.ga_h,
                 gx + at->x, gy + at->y, HUD_BLIT_KEYED);
    }

    /* Proof strip (D5): gear plate, then needle frames cut per .elt.
     * Evidence for the sprite-decode decisions, not cockpit art — the drive
     * path passes HUD_FLAG_NO_PROOF so it does not float in the sky in play. */
    if (!(flags & HUD_FLAG_NO_PROOF)) {
        int sy = 16;
        if (s.gear) {
            blit(fb, w, h, s.gear, s.gear_w, s.gear_h,
                 (w - s.gear_w) / 2, sy, HUD_BLIT_KEYED);
            sy += s.gear_h + 8;
        }
        if (s.needles) {
            static const char *const frames[] = {
                "needle_1", "needle_2", "needle_3"
            };
            const HudEltItem *it[3] = {NULL, NULL, NULL};
            int total_w = 0;
            for (int i = 0; i < 3; i++) {
                it[i] = elt_find("zneedle6.map", frames[i]);
                if (it[i]) total_w += it[i]->w + 8;
            }
            int nx = (w - total_w) / 2;
            for (int i = 0; i < 3; i++) {
                if (!it[i]) continue;
                blit_rect(fb, w, h, s.needles, s.ndl_w,
                          it[i]->x, it[i]->y, it[i]->w, it[i]->h,
                          nx, sy, HUD_BLIT_KEYED);
                nx += it[i]->w + 8;
            }
        }
    }

    /* Text line: hud_set_text() override wins, else the drive status
     * line composed from the frame params. Font-gated either way. */
    if (s.font && !(flags & HUD_FLAG_COCKPIT_LAYOUT)) {
        char line[HUD_TEXT_CAP];
        const char *text = s.text;
        if (!text[0]) {
            static const char selectors[] = "PRND21";
            unsigned tenths = (lap_ms % 1000u) / 100u;
            unsigned lap_s  = lap_ms / 1000u;
            if ((flags & HUD_FLAG_VITALS) && s.vit_hp_max > 0) {
                /* M7 combat vitals (hud_set_vitals): the damage pool;
                 * fuel only when a model set it (none today). */
                int n = snprintf(line, sizeof line, "%c %5.1f M/S  DMG %d/%d",
                                 selectors[selector], speed, s.vit_hp,
                                 s.vit_hp_max);
                if (s.vit_fuel >= 0)
                    snprintf(line + n, sizeof line - (size_t)n,
                             "  FUEL %d%%", s.vit_fuel);
            } else {
                snprintf(line, sizeof line, "%c %5.1f M/S  LAP %u:%02u.%u",
                         selectors[selector], speed, lap_s / 60u, lap_s % 60u,
                         tenths);
            }
            text = line;
        }
        /*
         * '\n' starts a new line. The composed status line is always one
         * line; hud_set_text() callers can use two, which is what the
         * arena objective readout needs — the original's own counter
         * strings ("%d of %d Laps Completed", "%02dH:%02dM:%02dS Minutes
         * Remaining") do not fit beside the speed on one 640 px row, and
         * abbreviating them would mean inventing wording the game has.
         */
        int ty = 8;
        for (const char *p = text; *p; ) {
            const char *nl = strchr(p, '\n');
            size_t len = nl ? (size_t)(nl - p) : strlen(p);
            char row[HUD_TEXT_CAP];
            if (len >= sizeof row) len = sizeof row - 1;
            memcpy(row, p, len);
            row[len] = '\0';
            font_draw(s.font, row, fb, w, h, 8, ty, s.remap[s.text_color]);
            ty += (int)s.font->height + 2;
            if (!nl) break;
            p = nl + 1;
        }
    }

    /* Weapon state has its own row instead of competing with hud_set_text():
     * objective controllers own up to two rows at the top left. Put this
     * compact line on row three, right-aligned, so selection/ammo/damage
     * remain visible in both cockpit and chase views. */
    if (s.font && s.weapon_count > 0 &&
        !(flags & HUD_FLAG_COCKPIT_LAYOUT)) {
        char weapon[96], ammo[24];
        if (s.weapon_ammo < 0)
            snprintf(ammo, sizeof ammo, "INF");
        else
            snprintf(ammo, sizeof ammo, "%d/%d", s.weapon_ammo,
                     s.weapon_ammo_max);
        snprintf(weapon, sizeof weapon, "WPN %d/%d %s  AMMO %s  DMG %d",
                 s.weapon_slot + 1, s.weapon_count, s.weapon, ammo,
                 s.weapon_damage);
        int x = w - font_text_width(s.font, weapon) - 8;
        if (x < 8) x = 8;
        font_draw(s.font, weapon, fb, w, h, x,
                  8 + 2 * ((int)s.font->height + 2),
                  s.remap[s.text_color]);
    }
}

int hud_stats(char *buf, size_t n)
{
    if (!s.loaded)
        return snprintf(buf, n, "hud: nothing loaded");
    return snprintf(buf, n,
        "hud: dash %s %d tiles %dx%d, pal=%s, elt %d tables/%d items"
        "%s, sheets radar=%s/%dfx weapon=%s damage=%s states=%s, "
        "gear=%s arrow=%s needles=%s compass=%s strip=%s, gauges=%s, "
        "mirror mask=%s bezel=%s, sidearm L=%d R=%d, font=%s, orient=%d, "
        "text='%s'",
        s.dash_base, s.dash_tiles, s.dash_w, s.dash_h,
        s.have_pal ? s.pal_name : "meshview-fallback",
        s.table_count, s.item_count,
        (s.table_overflow || s.item_overflow) ? " (OVERFLOW)" : "",
        s.radar_mask ? "ok" : "missing",
        s.radar_sweep_count,
        s.weapon_panel ? "ok" : "missing",
        s.damage_panel ? "ok" : "missing",
        s.damage_states ? "ok" : "missing",
        s.gear ? "ok" : "missing",
        s.gear_arrow ? "ok" : "missing",
        s.needles ? "ok" : "missing",
        s.compass ? (s.comp_ok ? "ok" : "rejected") : "missing",
        s.compass_strip ? (s.comp_ok ? "ok" : "rejected") : "missing",
        s.gauge_ok ? "ok" : "off",
        s.mirror_mask ? "ok" : "missing",
        s.mirror_bezel ? "ok" : "missing",
        s.sa_count[HUD_SIDEARM_LEFT], s.sa_count[HUD_SIDEARM_RIGHT],
        s.font ? s.font_name : "missing",
        s.orient,
        s.text);
}

const uint8_t *hud_palette(void)
{
    return s.have_pal ? &s.pal[0][0] : NULL;
}

const uint8_t *hud_mirror_mask(int *w, int *h)
{
    if (!s.mirror_mask) return NULL;
    if (w) *w = s.mm_w;
    if (h) *h = s.mm_h;
    return s.mirror_mask;
}

const uint8_t *hud_mirror_bezel(int *w, int *h)
{
    if (!s.mirror_bezel) return NULL;
    if (w) *w = s.mb_w;
    if (h) *h = s.mb_h;
    return s.mirror_bezel;
}

int hud_sidearm_frame_count(int side)
{
    if (side < 0 || side >= HUD_SIDEARM_SIDES) return 0;
    return s.sa_count[side];
}

const char *hud_sidearm_frame_name(int side, int index)
{
    if (side < 0 || side >= HUD_SIDEARM_SIDES ||
        index < 0 || index >= s.sa_count[side])
        return NULL;
    return s.sa_name[side][index];
}

const uint8_t *hud_sidearm_frame(int side, int index, int *w, int *h)
{
    if (side < 0 || side >= HUD_SIDEARM_SIDES ||
        index < 0 || index >= s.sa_count[side] || !s.sa_frame[side][index])
        return NULL;
    if (w) *w = s.sa_fw[side][index];
    if (h) *h = s.sa_fh[side][index];
    return s.sa_frame[side][index];
}

void hud_set_vitals(int hp, int hp_max, int fuel_pct)
{
    s.vit_hp     = hp;
    s.vit_hp_max = hp_max;
    s.vit_fuel   = fuel_pct;
}

void hud_clear_radar_contacts(void)
{
    s.radar_contact_count = 0;
}

void hud_add_radar_contact(double right_m, double fwd_m, int threat)
{
    if (s.radar_contact_count >= HUD_RADAR_MAX_CONTACTS)
        return;
    /* Past the authored 1 km field a contact is also past the draw
     * distance — drop it rather than pinning it to the rim. */
    double d2 = right_m * right_m + fwd_m * fwd_m;
    if (d2 > HUD_RADAR_RANGE_M * HUD_RADAR_RANGE_M)
        return;
    s.radar_contacts[s.radar_contact_count].right  = right_m;
    s.radar_contacts[s.radar_contact_count].fwd    = fwd_m;
    s.radar_contacts[s.radar_contact_count].threat = threat != 0;
    s.radar_contact_count++;
}

void hud_clear_conditions(void)
{
    /* Keeps cond_set: clearing marks the pools unknown (off) rather than
     * reverting to the scalar fallback once a component model fed us. */
    for (int i = 0; i < HUD_CONDITION_COMPONENTS; i++) {
        s.cond_hp[i] = 0;
        s.cond_max[i] = 0;
    }
}

void hud_set_condition(int comp, int hp, int hp_max)
{
    if (comp < 0 || comp >= HUD_CONDITION_COMPONENTS)
        return;
    s.cond_hp[comp] = hp;
    s.cond_max[comp] = hp_max;
    s.cond_set = 1;
}

void hud_set_target(const char *name, int hp, int hp_max, double range_m)
{
    s.tgt_set = 1;
    snprintf(s.tgt_name, sizeof s.tgt_name, "%s", name ? name : "?");
    s.tgt_hp = hp;
    s.tgt_hp_max = hp_max;
    s.tgt_range_m = (int)(range_m + 0.5);
}

void hud_clear_target(void)
{
    s.tgt_set = 0;
}

void hud_set_compass_yaw(double yaw)
{
    s.comp_yaw = yaw;
}

void hud_set_engine_rpm(double rpm)
{
    s.eng_rpm = rpm;
}

void hud_set_weapon(const char *name, int slot, int count, int ammo,
                    int ammo_max, int damage)
{
    snprintf(s.weapon, sizeof s.weapon, "%s", name ? name : "");
    s.weapon_slot = slot;
    s.weapon_count = count;
    s.weapon_ammo = ammo;
    s.weapon_ammo_max = ammo_max;
    s.weapon_damage = damage;
}

void hud_clear_weapon_rows(void)
{
    memset(s.weapon_row_name, 0, sizeof s.weapon_row_name);
    memset(s.weapon_row_ammo, 0, sizeof s.weapon_row_ammo);
    memset(s.weapon_row_max, 0, sizeof s.weapon_row_max);
    memset(s.weapon_row_armed, 0, sizeof s.weapon_row_armed);
}

void hud_set_weapon_row(int row, const char *name, int ammo, int ammo_max)
{
    if (row < 0 || row >= HUD_WEAPON_ROWS) return;
    snprintf(s.weapon_row_name[row], sizeof s.weapon_row_name[row],
             "%s", name ? name : "");
    s.weapon_row_ammo[row] = ammo;
    s.weapon_row_max[row] = ammo_max;
}

void hud_set_weapon_row_armed(int row, int armed)
{
    if (row < 0 || row >= HUD_WEAPON_ROWS) return;
    s.weapon_row_armed[row] = armed ? 1 : 0;
}

void hud_weapon_row_render_masks(uint32_t *authored, uint32_t *fallback,
                                 uint32_t *ammo)
{
    if (authored) *authored = s.weapon_authored_rows;
    if (fallback) *fallback = s.weapon_fallback_rows;
    if (ammo) *ammo = s.weapon_ammo_rows;
}

void hud_set_text(const char *text)
{
    if (!text) text = "";
    snprintf(s.text, sizeof s.text, "%s", text);
}

void hud_set_orientation(int mode)
{
    if (mode < HUD_ORIENT_SHIPPING || mode > HUD_ORIENT_COLUMN) return;
    s.orient = mode;
}

int hud_orientation(void)
{
    return s.orient;
}
