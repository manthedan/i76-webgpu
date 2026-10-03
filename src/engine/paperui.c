/*
 * paperui.c — VQM map / notepad / escape + PCX title cards. See paperui.h.
 */

#include "engine/paperui.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "engine/font.h"
#include "engine/hud.h"
#include "engine/mission.h"
#include "engine/pcx.h"
#include "engine/raster.h"
#include "engine/terrain.h"
#include "engine/vfs.h"
#include "engine/vqm.h"

typedef struct {
    uint8_t *pix;
    int      w, h;
    int      ok;
} PaperTile;

static PaperTile   s_map, s_npd;
static uint8_t    *s_npd_blank;
static unsigned   s_note_flags[6];
static PaperTile   s_esc[PAPER_ESC_COUNT];
static PaperTile   s_title;          /* remapped to level palette indices */
static PaperSurface s_active = PAPER_NONE;
static PaperEscape  s_escape = PAPER_ESC_NONE;
static char        s_tag[16];
static int         s_have_player;
static int         s_dev_mode;
static double      s_px, s_pz, s_pyaw;
static int         s_title_ticks;    /* remaining 20 Hz ticks */

/* Escape panel basenames inside 6escape.pix / 3escape.pix (no prefix). */
static const char *const ESC_BASE[PAPER_ESC_COUNT] = {
    "",          /* NONE */
    "MAINMN1",
    "PLYOPT1",
    "ABRTMS1",
    "AUDCON1",
    "GRXDET1",
    "SAVPNT1",
    "EXTGME1",
};

static void tile_free(PaperTile *t)
{
    free(t->pix);
    t->pix = NULL;
    t->w = t->h = t->ok = 0;
}

static int tile_load_vqm_buf(PaperTile *t, const uint8_t *buf, size_t n)
{
    if (!buf || n < 24) return -1;
    int w = 0, h = 0;
    uint8_t *pix = vqm_decode(buf, n, 0, &w, &h);
    if (!pix || w <= 0 || h <= 0) {
        free(pix);
        return -1;
    }
    tile_free(t);
    t->pix = pix;
    t->w = w;
    t->h = h;
    t->ok = 1;
    return 0;
}

static int tile_load_vqm(PaperTile *t, const char *name)
{
    if (!vfs_exists(name)) return -1;
    size_t n = 0;
    uint8_t *buf = vfs_read_file(name, &n);
    if (!buf) return -1;
    int rc = tile_load_vqm_buf(t, buf, n);
    vfs_free(buf);
    return rc;
}

static void mission_tag(const char *path, char *out, size_t out_n)
{
    if (!out || out_n == 0) return;
    out[0] = '\0';
    if (!path || !*path) return;
    const char *base = path;
    for (const char *p = path; *p; p++)
        if (*p == '/' || *p == '\\') base = p + 1;
    size_t i = 0;
    for (const char *p = base; *p && *p != '.' && i + 1 < out_n; p++)
        out[i++] = (char)tolower((unsigned char)*p);
    out[i] = '\0';
}

static int load_map_for_tag(const char *tag)
{
    char name[32];
    snprintf(name, sizeof name, "zmap6%s.vqm", tag);
    if (tile_load_vqm(&s_map, name) == 0) return 0;
    snprintf(name, sizeof name, "zmap3%s.vqm", tag);
    if (tile_load_vqm(&s_map, name) == 0) return 0;
    if (strcmp(tag, "m00") != 0) {
        if (tile_load_vqm(&s_map, "zmap6m00.vqm") == 0) return 0;
        if (tile_load_vqm(&s_map, "zmap3m00.vqm") == 0) return 0;
    }
    return -1;
}

static int load_notepad_for_mission(const char *mission_path)
{
    int cls = hud_scenario_art_class(mission_path);
    if (cls < 1 || cls > 5) cls = 1;
    char name[32];
    snprintf(name, sizeof name, "znpd6%d01.vqm", cls);
    if (tile_load_vqm(&s_npd, name) == 0) return 0;
    snprintf(name, sizeof name, "znpd3%d01.vqm", cls);
    if (tile_load_vqm(&s_npd, name) == 0) return 0;
    if (cls != 1) {
        if (tile_load_vqm(&s_npd, "znpd6101.vqm") == 0) return 0;
        if (tile_load_vqm(&s_npd, "znpd3101.vqm") == 0) return 0;
    }
    return -1;
}

static void draw_notes(const Font *font, uint8_t *page, int w, int h,
                       uint8_t ink)
{
    int y = 0;
    int height = (int)font->height;
    for (int id = 1; id <= 6 && y + height <= h; id++) {
        const MissionNote *note = mission_note(id);
        if (!note) break;
        if (note->flags & 1) continue;
        const uint8_t *text = (const uint8_t *)note->text;
        size_t pos = 0, end = strlen(note->text);
        while (pos < end && y + height <= h) {
            char line[256];
            size_t count = 0, space = 0;
            int width = 0;
            while (pos + count < end && count + 1 < sizeof line) {
                unsigned char c = text[pos + count];
                if (c == '\t') c = ' ';
                char glyph[2] = { (char)c, 0 };
                int advance = font_text_width(font, glyph);
                if (width + advance > w && count) break;
                width += advance;
                line[count++] = (char)c;
                if (c == ' ') space = count;
            }
            if (pos + count < end && space) count = space;
            line[count] = '\0';
            font_draw(font, line, page, w, h, 0, y, ink);
            /* FUN_004591f0 -> FUN_004a2a30 strikes each wrapped success
             * line at half its text height. Failure has no strike. */
            if (note->flags & 2) {
                int width = font_text_width(font, line);
                if (width > w) width = w;
                memset(page + (y + height / 2) * w, ink, (size_t)width);
            }
            pos += count;
            while (pos < end && (text[pos] == ' ' || text[pos] == '\t')) pos++;
            y += height + (height >= 14 ? 4 : 2);
        }
        y += height;
    }
}

static void write_notepad(void)
{
    if (!s_npd.ok || !s_npd_blank) return;
    memcpy(s_npd.pix, s_npd_blank, (size_t)s_npd.w * s_npd.h);
    Font *font = font_load(s_npd.w > 320 ? "base6x76.fnt" : "base6x7.fnt");
    if (!font) font = font_load("base6x7.fnt");
    if (!font) return;

    /* Native FUN_00458ca0 uses (82,102), 448x317 in mode 6 and
     * (40,41), 228x199 in mode 3. PORT DECISION: retain the available
     * bitmap font instead of the native Windows GDI font. */
    int x = s_npd.w > 320 ? 82 : 40, y = s_npd.w > 320 ? 102 : 41;
    int w = s_npd.w > 320 ? 448 : 228, h = s_npd.w > 320 ? 317 : 199;
    if (x >= s_npd.w || y >= s_npd.h) { font_free(font); return; }
    if (w > s_npd.w - x) w = s_npd.w - x;
    if (h > s_npd.h - y) h = s_npd.h - y;
    uint8_t *page = malloc((size_t)w * h);
    if (page && w > 0 && h > 0) {
        for (int row = 0; row < h; row++)
            memcpy(page + row * w, s_npd.pix + (y + row) * s_npd.w + x, w);
        /* Zero is transparent in the held tile, so choose nonzero ink. */
        uint8_t ink = 1;
        const uint8_t *pal = hud_palette();
        if (pal) {
            int darkest = 3 * 255 * 255 + 1;
            for (int i = 1; i < 256; i++) {
                int r = pal[3*i], g = pal[3*i+1], b = pal[3*i+2];
                int d = r*r + g*g + b*b;
                if (d < darkest) { darkest = d; ink = (uint8_t)i; }
            }
        }
        draw_notes(font, page, w, h, ink);
        for (int row = 0; row < h; row++)
            memcpy(s_npd.pix + (y + row) * s_npd.w + x, page + row * w, w);
    }
    free(page);
    font_free(font);
    for (int i = 0; i < 6; i++) {
        const MissionNote *note = mission_note(i + 1);
        s_note_flags[i] = note ? note->flags : 0;
    }
}

/*
 * Parse a one-line "NAME off len" pix/pak pair and extract matching VQMs.
 * Prefer mode-6 pack; fall back to mode-3.
 */
static int load_escape_pack(void)
{
    static const char *const packs[][2] = {
        { "6escape.pix", "6escape.pak" },
        { "3escape.pix", "3escape.pak" },
    };
    for (int p = 0; p < 2; p++) {
        size_t pix_n = 0, pak_n = 0;
        if (!vfs_exists(packs[p][0]) || !vfs_exists(packs[p][1]))
            continue;
        char *pix = vfs_read_file(packs[p][0], &pix_n);
        uint8_t *pak = vfs_read_file(packs[p][1], &pak_n);
        if (!pix || !pak) {
            vfs_free(pix);
            vfs_free(pak);
            continue;
        }
        /* Ensure NUL termination for strtok-style scan. */
        char *text = malloc(pix_n + 1);
        if (!text) {
            vfs_free(pix);
            vfs_free(pak);
            continue;
        }
        memcpy(text, pix, pix_n);
        text[pix_n] = '\0';
        vfs_free(pix);

        int loaded = 0;
        char *save = NULL;
        char *line = strtok_r(text, "\r\n", &save);
        /* First line is count. */
        if (line) line = strtok_r(NULL, "\r\n", &save);
        for (; line; line = strtok_r(NULL, "\r\n", &save)) {
            char name[32];
            unsigned off = 0, len = 0;
            if (sscanf(line, "%31s %u %u", name, &off, &len) != 3)
                continue;
            if (len == 0 || (size_t)off + len > pak_n)
                continue;
            /* Match 6MAINMN1.vqm / 3MAINMN1.vqm against ESC_BASE. */
            char low[32];
            size_t nl = strlen(name);
            if (nl >= sizeof low) continue;
            for (size_t i = 0; i <= nl; i++)
                low[i] = (char)tolower((unsigned char)name[i]);
            for (int e = 1; e < PAPER_ESC_COUNT; e++) {
                char want[24];
                snprintf(want, sizeof want, "%s.vqm", ESC_BASE[e]);
                for (char *c = want; *c; c++)
                    if (*c >= 'A' && *c <= 'Z')
                        *c = (char)(*c - 'A' + 'a');
                /* strip leading 3/6 digit from low for compare */
                const char *body = low;
                if (body[0] == '3' || body[0] == '6') body++;
                if (strcmp(body, want) != 0) continue;
                if (tile_load_vqm_buf(&s_esc[e], pak + off, len) == 0)
                    loaded++;
                break;
            }
        }
        free(text);
        vfs_free(pak);
        if (loaded > 0) return 0;
    }
    return -1;
}

/*
 * Remap a PCX's own palette indices into the live level palette by nearest
 * RGB distance. Falls back to mesh/debug palette via raster_rgb_to_index
 * when no level .act is loaded.
 */
static int load_title_pcx(const char *tag)
{
    static const char *const try_fmt[] = {
        "addon/%sload.pcx",
        "%sload.pcx",
        "addon/loadscr.pcx",
        "loadscr.pcx",
        "addon/loadgame.pcx",
        "loadgame.pcx",
    };
    char path[48];
    PcxImage *img = NULL;
    for (size_t i = 0; i < sizeof try_fmt / sizeof try_fmt[0]; i++) {
        if (strstr(try_fmt[i], "%s")) {
            if (!tag[0]) continue;
            snprintf(path, sizeof path, try_fmt[i], tag);
        } else {
            snprintf(path, sizeof path, "%s", try_fmt[i]);
        }
        /* fs_fopen retries basename case; try as written then upper base. */
        img = pcx_load(path);
        if (!img) {
            char up[48];
            snprintf(up, sizeof up, "%s", path);
            char *base = strrchr(up, '/');
            base = base ? base + 1 : up;
            for (char *c = base; *c; c++)
                if (*c >= 'a' && *c <= 'z') *c = (char)(*c - 'a' + 'A');
            img = pcx_load(up);
        }
        if (img) break;
    }
    if (!img) return -1;

    const uint8_t *level = hud_palette();
    uint8_t *out = malloc((size_t)img->width * img->height);
    if (!out) {
        pcx_free(img);
        return -1;
    }
    /* Build a 256-entry remap once. */
    uint8_t remap[256];
    for (int i = 0; i < 256; i++) {
        int r = img->palette[i * 3 + 0];
        int g = img->palette[i * 3 + 1];
        int b = img->palette[i * 3 + 2];
        if (level) {
            int best = 0, bestd = 1 << 30;
            for (int j = 0; j < 256; j++) {
                int dr = (int)level[j * 3 + 0] - r;
                int dg = (int)level[j * 3 + 1] - g;
                int db = (int)level[j * 3 + 2] - b;
                int d = dr * dr + dg * dg + db * db;
                if (d < bestd) { bestd = d; best = j; }
            }
            remap[i] = (uint8_t)best;
        } else {
            remap[i] = raster_rgb_to_index((uint8_t)r, (uint8_t)g, (uint8_t)b);
        }
    }
    size_t np = (size_t)img->width * img->height;
    for (size_t i = 0; i < np; i++)
        out[i] = remap[img->pixels[i]];

    tile_free(&s_title);
    s_title.pix = out;
    s_title.w = (int)img->width;
    s_title.h = (int)img->height;
    s_title.ok = 1;
    pcx_free(img);
    return 0;
}

int paper_load_mission(const char *mission_path)
{
    paper_unload();
    mission_tag(mission_path, s_tag, sizeof s_tag);
    if (!s_tag[0]) return -1;

    int map_ok = load_map_for_tag(s_tag) == 0;
    int npd_ok = load_notepad_for_mission(mission_path) == 0;
    if (npd_ok) {
        size_t size = (size_t)s_npd.w * s_npd.h;
        s_npd_blank = malloc(size);
        if (s_npd_blank) memcpy(s_npd_blank, s_npd.pix, size);
        write_notepad();
    }
    int esc_ok = load_escape_pack() == 0;
    int ttl_ok = load_title_pcx(s_tag) == 0;
    return (map_ok || npd_ok || esc_ok || ttl_ok) ? 0 : -1;
}

void paper_unload(void)
{
    tile_free(&s_map);
    tile_free(&s_npd);
    free(s_npd_blank);
    s_npd_blank = NULL;
    memset(s_note_flags, 0, sizeof s_note_flags);
    for (int i = 0; i < PAPER_ESC_COUNT; i++)
        tile_free(&s_esc[i]);
    tile_free(&s_title);
    s_active = PAPER_NONE;
    s_escape = PAPER_ESC_NONE;
    s_tag[0] = '\0';
    s_have_player = 0;
    s_title_ticks = 0;
}

void paper_toggle(PaperSurface which)
{
    if (which != PAPER_MAP && which != PAPER_NOTEPAD) {
        s_active = PAPER_NONE;
        return;
    }
    if (!paper_has(which))
        return;
    /* Holding map/notepad dismisses escape paper and title. */
    s_escape = PAPER_ESC_NONE;
    s_title_ticks = 0;
    s_active = (s_active == which) ? PAPER_NONE : which;
}

void paper_hide(void)
{
    s_active = PAPER_NONE;
    s_escape = PAPER_ESC_NONE;
}

PaperSurface paper_active(void) { return s_active; }

int paper_has(PaperSurface which)
{
    if (which == PAPER_MAP) return s_map.ok;
    if (which == PAPER_NOTEPAD) return s_npd.ok;
    return 0;
}

void paper_set_escape(PaperEscape which)
{
    if (which < 0 || which >= PAPER_ESC_COUNT)
        which = PAPER_ESC_NONE;
    if (which != PAPER_ESC_NONE && !s_esc[which].ok)
        which = PAPER_ESC_NONE;
    s_escape = which;
    if (which != PAPER_ESC_NONE) {
        s_active = PAPER_NONE;   /* escape replaces held map/notepad */
        s_title_ticks = 0;
    }
}

PaperEscape paper_escape(void) { return s_escape; }

int paper_has_escape(PaperEscape which)
{
    if (which <= PAPER_ESC_NONE || which >= PAPER_ESC_COUNT) return 0;
    return s_esc[which].ok;
}

int paper_has_title(void) { return s_title.ok; }

void paper_show_title(int ticks_20hz)
{
    if (!s_title.ok) {
        s_title_ticks = 0;
        return;
    }
    if (ticks_20hz < 1) ticks_20hz = 1;
    s_title_ticks = ticks_20hz;
    s_active = PAPER_NONE;
    s_escape = PAPER_ESC_NONE;
}

void paper_title_tick(void)
{
    if (s_title_ticks > 0)
        s_title_ticks--;
}

void paper_dismiss_title(void) { s_title_ticks = 0; }

int paper_title_active(void) { return s_title_ticks > 0 && s_title.ok; }

void paper_set_dev_mode(int enabled)
{
    s_dev_mode = enabled != 0;
}

void paper_set_player(double x, double z, double yaw)
{
    s_px = x;
    s_pz = z;
    s_pyaw = yaw;
    s_have_player = 1;
}

static void blit_tile(uint8_t *fb, int fw, int fh, const PaperTile *t,
                      int opaque_zero)
{
    if (!t || !t->ok || !t->pix || fw <= 0 || fh <= 0) return;

    int scale = 1;
    if (t->w * 2 <= fw && t->h * 2 <= fh && t->w <= 320)
        scale = 2;

    int dw = t->w * scale;
    int dh = t->h * scale;
    int ox = (fw - dw) / 2;
    int oy = (fh - dh) / 2;

    for (int y = 0; y < dh; y++) {
        int sy = y / scale;
        int dy = oy + y;
        if (dy < 0 || dy >= fh) continue;
        for (int x = 0; x < dw; x++) {
            int sx = x / scale;
            int dx = ox + x;
            if (dx < 0 || dx >= fw) continue;
            uint8_t p = t->pix[sy * t->w + sx];
            if (opaque_zero || p != 0)
                fb[dy * fw + dx] = p;
        }
    }
}

/*
 * PORT DEV AID — native has no player pin and no world-to-map transform.
 * Keep this terrain-AABB estimate only under ?dev=1, visibly marked EST so
 * it cannot be mistaken for game UI. See docs/specs/re/paper-map.md.
 * The projection itself is intentionally unchanged: it is cheap spatial
 * debugging, not a calibration claim.
 */
static void draw_map_pin(uint8_t *fb, int fw, int fh,
                         int ox, int oy, int dw, int dh)
{
    if (!s_dev_mode || !s_have_player || dw < 8 || dh < 8) return;
    double wx0, wz0, wx1, wz1;
    if (terrain_used_bounds(&wx0, &wz0, &wx1, &wz1) != 0)
        return;
    double dx = wx1 - wx0, dz = wz1 - wz0;
    if (!(dx > 1.0) || !(dz > 1.0)) return;

    /* Authored route maps leave a painted border; keep a ~8% inset. */
    const double inset = 0.08;
    double u = (s_px - wx0) / dx;
    double v = (s_pz - wz0) / dz;
    if (u < 0.0) u = 0.0; else if (u > 1.0) u = 1.0;
    if (v < 0.0) v = 0.0; else if (v > 1.0) v = 1.0;
    u = inset + u * (1.0 - 2.0 * inset);
    v = inset + v * (1.0 - 2.0 * inset);

    /* North-up art: increasing world Z = north = up the image → invert v. */
    int cx = ox + (int)(u * (dw - 1));
    int cy = oy + (int)((1.0 - v) * (dh - 1));

    uint8_t cross_col = raster_rgb_to_index(255, 0, 255);
    uint8_t tag_col = raster_rgb_to_index(0, 255, 255);
    int arm = dw / 40;
    if (arm < 4) arm = 4;
    if (arm > 14) arm = 14;
    for (int i = -arm; i <= arm; i++) {
        int x = cx + i, y = cy;
        if ((unsigned)x < (unsigned)fw && (unsigned)y < (unsigned)fh)
            fb[y * fw + x] = cross_col;
        x = cx; y = cy + i;
        if ((unsigned)x < (unsigned)fw && (unsigned)y < (unsigned)fh)
            fb[y * fw + x] = cross_col;
    }
    /* Heading tick: game yaw 0 faces +Z (north); screen up is -Y. */
    double hx = sin(s_pyaw), hz = cos(s_pyaw);
    int tx = cx + (int)(hx * (arm + 4));
    int ty = cy - (int)(hz * (arm + 4));
    /* Bresenham short segment. */
    int x0 = cx, y0 = cy, x1 = tx, y1 = ty;
    int adx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int ady = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = adx + ady;
    for (;;) {
        if ((unsigned)x0 < (unsigned)fw && (unsigned)y0 < (unsigned)fh)
            fb[y0 * fw + x0] = tag_col;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= ady) { err += ady; x0 += sx; }
        if (e2 <= adx) { err += adx; y0 += sy; }
    }

    /* 3x5 debug lettering: E S T, deliberately unlike authored paper art. */
    static const char est[5][12] = {
        "###.###.###",
        "#...#....#.",
        "##..###..#.",
        "#.....#..#.",
        "###.###..#.",
    };
    int lx = cx + arm + 3, ly = cy - 2;
    for (int y = 0; y < 5; y++)
        for (int x = 0; x < 11; x++)
            if (est[y][x] == '#' &&
                (unsigned)(lx + x) < (unsigned)fw &&
                (unsigned)(ly + y) < (unsigned)fh)
                fb[(ly + y) * fw + lx + x] = tag_col;
}

static void blit_map_with_pin(uint8_t *fb, int fw, int fh)
{
    if (!s_map.ok) return;
    int scale = 1;
    if (s_map.w * 2 <= fw && s_map.h * 2 <= fh && s_map.w <= 320)
        scale = 2;
    int dw = s_map.w * scale;
    int dh = s_map.h * scale;
    int ox = (fw - dw) / 2;
    int oy = (fh - dh) / 2;
    blit_tile(fb, fw, fh, &s_map, 1);
    draw_map_pin(fb, fw, fh, ox, oy, dw, dh);
}

void paper_render(uint8_t *fb, int w, int h)
{
    if (!fb) return;

    if (s_active == PAPER_MAP && s_map.ok)
        blit_map_with_pin(fb, w, h);
    else if (s_active == PAPER_NOTEPAD && s_npd.ok) {
        for (int i = 0; i < 6; i++) {
            const MissionNote *note = mission_note(i + 1);
            if (note && note->flags != s_note_flags[i]) {
                write_notepad();
                break;
            }
        }
        blit_tile(fb, w, h, &s_npd, 0);
    }

    if (s_escape > PAPER_ESC_NONE && s_escape < PAPER_ESC_COUNT &&
        s_esc[s_escape].ok)
        blit_tile(fb, w, h, &s_esc[s_escape], 0);

    if (s_title_ticks > 0 && s_title.ok)
        blit_tile(fb, w, h, &s_title, 1);
}

int paper_stats(char *buf, size_t n)
{
    const char *act = "none";
    if (s_active == PAPER_MAP) act = "map";
    else if (s_active == PAPER_NOTEPAD) act = "npd";
    const char *esc = "none";
    switch (s_escape) {
    case PAPER_ESC_MAIN:   esc = "main"; break;
    case PAPER_ESC_PLYOPT: esc = "plyopt"; break;
    case PAPER_ESC_ABRT:   esc = "abrt"; break;
    case PAPER_ESC_AUDCON: esc = "aud"; break;
    case PAPER_ESC_GRXDET: esc = "grx"; break;
    case PAPER_ESC_SAVPNT: esc = "sav"; break;
    case PAPER_ESC_EXTGME: esc = "ext"; break;
    default: break;
    }
    int nesc = 0;
    for (int i = 1; i < PAPER_ESC_COUNT; i++)
        if (s_esc[i].ok) nesc++;
    return snprintf(buf, n,
        "paper tag=%s map=%d npd=%d esc=%d title=%d active=%s escape=%s ttl=%d",
        s_tag[0] ? s_tag : "-",
        s_map.ok, s_npd.ok, nesc, s_title.ok,
        act, esc, s_title_ticks);
}
