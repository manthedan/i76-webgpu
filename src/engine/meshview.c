/*
 * meshview.c — In-engine OEG mesh viewer (see meshview.h)
 *
 * Loads meshes through the real engine path and software-renders a rotating
 * wireframe into the 8-bit indexed framebuffer, which the existing Vulkan blit
 * turns into pixels. Proves the GEO decode end-to-end inside the engine.
 */

#include "meshview.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "platform/platform.h"
#include "engine/input.h"
#include "engine/font.h"
#include "engine/meshcache.h"
#include "engine/geomesh.h"
#include "render/render.h"

#define FB_W MESHVIEW_FB_W
#define FB_H MESHVIEW_FB_H
#define TARGET_FRAME_MS (1000.0 / 60.0)

/* Palette indices. */
#define IDX_BG     0
#define IDX_WIRE   1
#define IDX_VERT   2
#define IDX_TEXT   3
#define IDX_DIM    4
#define IDX_AXIS   5

/*
 * Assets to browse. A g-tier .pak decodes to its FIRST OEG record; a .geo is a
 * single record. All resolve through vfs_read_file by filename. The names are
 * the game's own; what each mesh actually represents is NOT confirmed, so no
 * role labels here — the HUD shows the embedded OEG name. LEFT/RIGHT cycle.
 */
static const char *const ASSETS[] = {
    "A4TANK1G.PAK",
    "A3PLIN1G.PAK",
    "A1FLAG1G.PAK",
    "A2FNSG1G.PAK",
    "NEEDLE.GEO",
    "CHUNK1.GEO",
    "CHUNK2.GEO",
};
#define NUM_ASSETS ((int)(sizeof(ASSETS) / sizeof(ASSETS[0])))

void build_palette(Rgb8 *pal)
{
    memset(pal, 0, sizeof(Rgb8) * 256);
    pal[IDX_BG]   = (Rgb8){ 12,  14,  24  };
    pal[IDX_WIRE] = (Rgb8){ 90,  220, 120 };
    pal[IDX_VERT] = (Rgb8){ 255, 240, 90  };
    pal[IDX_TEXT] = (Rgb8){ 235, 235, 235 };
    pal[IDX_DIM]  = (Rgb8){ 110, 110, 120 };
    pal[IDX_AXIS] = (Rgb8){ 70,  80,  110 };
}

/* Bresenham line into the 8-bit framebuffer, clipped to bounds. */
static void draw_line(uint8_t *fb, int x0, int y0, int x1, int y1, uint8_t col)
{
    int dx =  abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        if ((unsigned)x0 < FB_W && (unsigned)y0 < FB_H)
            fb[(size_t)y0 * FB_W + x0] = col;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void put_dot(uint8_t *fb, int x, int y, uint8_t col)
{
    for (int yy = y - 1; yy <= y + 1; yy++)
        for (int xx = x - 1; xx <= x + 1; xx++)
            if ((unsigned)xx < FB_W && (unsigned)yy < FB_H)
                fb[(size_t)yy * FB_W + xx] = col;
}

/*
 * Render the mesh wireframe. yaw spins it; pitch is a fixed slight top-down.
 * Vertices are rotated, then orthographically projected and auto-fit to the
 * viewport from their own 2D extent (so any mesh scale fills the screen).
 */
void render_mesh(uint8_t *fb, const GeoMesh *m, double yaw, double zoom)
{
    memset(fb, IDX_BG, (size_t)FB_W * FB_H);
    if (!m || m->num_verts == 0) return;

    const double pitch = -0.42;
    double cy = cos(yaw),   sy = sin(yaw);
    double cp = cos(pitch), sp = sin(pitch);

    /* Center on the bounding box. */
    float ctr[3];
    for (int c = 0; c < 3; c++) ctr[c] = 0.5f * (m->bb_min[c] + m->bb_max[c]);

    /* Project all vertices to 2D camera space first (to auto-fit). */
    static float px[100000], py[100000];   /* MAX_VERTS in geomesh.c */
    double minx = 1e30, maxx = -1e30, miny = 1e30, maxy = -1e30;
    for (int i = 0; i < m->num_verts; i++) {
        double x = m->verts[i * 3 + 0] - ctr[0];
        double y = m->verts[i * 3 + 1] - ctr[1];
        double z = m->verts[i * 3 + 2] - ctr[2];
        /* yaw about Y, then pitch about X */
        double x1 =  x * cy + z * sy;
        double z1 = -x * sy + z * cy;
        double y2 =  y * cp - z1 * sp;
        px[i] = (float)x1;
        py[i] = (float)y2;
        if (x1 < minx) minx = x1;
        if (x1 > maxx) maxx = x1;
        if (y2 < miny) miny = y2;
        if (y2 > maxy) maxy = y2;
    }

    double spanx = (maxx - minx) > 1e-6 ? (maxx - minx) : 1e-6;
    double spany = (maxy - miny) > 1e-6 ? (maxy - miny) : 1e-6;
    double margin = 70.0;
    double scale = zoom * fmin((FB_W - margin) / spanx, (FB_H - margin) / spany);
    double ox = FB_W * 0.5 - 0.5 * (minx + maxx) * scale;
    double oy = FB_H * 0.5 + 0.5 * (miny + maxy) * scale;  /* +: flip Y for screen */

    /* Faces as closed wireframe loops. */
    for (int f = 0; f < m->num_faces; f++) {
        int first = m->face_first[f];
        int n     = m->face_count[f];
        for (int k = 0; k < n; k++) {
            int a = m->indices[first + k];
            int b = m->indices[first + (k + 1) % n];
            int ax = (int)lround(ox + px[a] * scale), ay = (int)lround(oy - py[a] * scale);
            int bx = (int)lround(ox + px[b] * scale), by = (int)lround(oy - py[b] * scale);
            draw_line(fb, ax, ay, bx, by, IDX_WIRE);
        }
    }
    /* Vertices on top. */
    for (int i = 0; i < m->num_verts; i++)
        put_dot(fb, (int)lround(ox + px[i] * scale), (int)lround(oy - py[i] * scale), IDX_VERT);
}

void meshview_run(void)
{
    static uint8_t fb[FB_W * FB_H];
    Rgb8 pal[256];
    build_palette(pal);

    Font *font = font_load("base6x7.fnt");
    if (!font)
        fprintf(stdout, "[meshview] base6x7.fnt not found — HUD text disabled.\n");

    input_init();

    int      cur   = 0;
    GeoMesh *mesh  = (GeoMesh *)geo_cache_acquire(ASSETS[cur]);
    double   yaw   = 0.6;
    double   zoom  = 1.0;
    bool     spin  = true;

    fprintf(stdout, "[meshview] running. LEFT/RIGHT cycle, UP/DOWN zoom, "
                    "SPACE spin, ESC quit.\n");

    uint64_t last = platform_get_ticks();
    bool running = true;
    while (running) {
        uint64_t frame_start = platform_get_ticks();
        double dt = (double)(frame_start - last) / 1000.0;
        last = frame_start;
        if (dt > 0.1) dt = 0.1;

        InputState in;
        input_poll(&in);
        if (in.quit) break;

        int prev = cur;
        if (in.pressed[INPUT_BTN_RIGHT]) cur = (cur + 1) % NUM_ASSETS;
        if (in.pressed[INPUT_BTN_LEFT])  cur = (cur - 1 + NUM_ASSETS) % NUM_ASSETS;
        if (cur != prev) {
            if (mesh) geo_cache_release(mesh);
            mesh = (GeoMesh *)geo_cache_acquire(ASSETS[cur]);
            zoom = 1.0;
        }
        if (in.held[INPUT_BTN_UP])    zoom *= 1.0 + 1.5 * dt;
        if (in.held[INPUT_BTN_DOWN])  zoom *= 1.0 - 1.5 * dt;
        if (zoom < 0.1) zoom = 0.1;
        if (zoom > 8.0) zoom = 8.0;
        if (in.pressed[INPUT_BTN_FIRE]) spin = !spin;
        if (spin) yaw += dt * 0.8;

        render_mesh(fb, mesh, yaw, zoom);

        if (font) {
            char line[128];
            if (mesh)
                snprintf(line, sizeof(line), "%s   \"%s\"   %d verts  %d faces",
                         ASSETS[cur], mesh->name, mesh->num_verts, mesh->num_faces);
            else
                snprintf(line, sizeof(line), "%s   <load/decode failed>", ASSETS[cur]);
            font_draw(font, line, fb, FB_W, FB_H, 8, 8, IDX_TEXT);
            char idx[48];
            snprintf(idx, sizeof(idx), "[%d/%d]", cur + 1, NUM_ASSETS);
            font_draw(font, idx, fb, FB_W, FB_H, 8, 8 + (int)font->height + 4, IDX_DIM);
            font_draw(font, "LEFT/RIGHT MESH   UP/DOWN ZOOM   SPACE SPIN   ESC QUIT",
                      fb, FB_W, FB_H, 8, FB_H - 16, IDX_DIM);
        }

        render_begin_frame();
        render_set_palette(pal);
        render_blit_indexed(fb, FB_W, FB_H);
        render_end_frame();

        double elapsed = (double)(platform_get_ticks() - frame_start);
        if (elapsed < TARGET_FRAME_MS)
            platform_sleep((uint32_t)(TARGET_FRAME_MS - elapsed));
    }

    if (mesh) geo_cache_release(mesh);
    if (font) font_free(font);
}
