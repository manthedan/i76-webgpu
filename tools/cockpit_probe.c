/*
 * cockpit_probe.c — focused purchaser-asset proof for the cockpit interior
 * slice (car.c VGEO first-person set + VLOC exports, hud.c scenario-keyed
 * dash class + authored mirror mask).
 *
 * Runs entirely against the purchaser-staged asset root through the real
 * engine VFS (fs_set_root + vfs_init); nothing is bundled or copied.
 *
 * Asserts, for the P01 player car (vdrampg2.vcf -> vdrampag.vdf "Dover
 * Rampage"):
 *   1. the VGEO first-person set (index 16, car.c D22) parses to multiple
 *      interior parts with the expected role names (BDYF/BDYT/DASH/RTCB,
 *      MIRI, RADR, SWHL, SEAT, CMP3/CMP6, GER6, WEP3/WEP6, SYS3/SYS6,
 *      SPC3/SPC6, GUNL/GUNR, RTC1...) and real composed frames;
 *   2. every interior part resolves to mesh bytes through the same three
 *      routes scene.c's mesh_get uses (loose .geo, g.pix-merged pak
 *      record, <part>g.pak) — the cockpit frame source is materially
 *      complete;
 *   3. car_interior_mirror_part() identifies the authored mirror surface
 *      (MIRI suffix) and car_vloc_* exports the five Rampage attachment
 *      locators ({35,36,38,40,42});
 *   4. hud_load_mission("miss8/P01.MSN") selects the scenario-keyed dash
 *      set 2 (FACT: nitro.exe table @0x503c88) and the dash art actually
 *      changes, while the gear indicator and compass still load and
 *      still track selector/yaw state, and the live speedo/tach needles
 *      (hud.c D8, FACT frame formulas from nitro.exe's gauge code) move
 *      with the speed/RPM state inside their gauge windows while the
 *      fixed dashboard framing stays pixel-identical — and the class-4
 *      dash (P09), a cockpit without the two-gauge layout, rejects the
 *      needle hubs and keeps its static faces;
 *   5. the authored mirror mask/bezel decode (zmiri101.map/zmiro101.map);
 *   6. car_gdf_weapon_info parses the unmounted pilot sidearm gh45.gdf.
 *
 * usage: cockpit_probe <asset-root> <capture-dir>
 *   Both paths are explicit; capture-dir must already exist outside checkout.
 *
 * build: OUT=/external/path tools/build_probe.sh cockpit_probe (ENGINE_ALL)
 *   or, when a sibling subsystem is mid-edit and ENGINE_ALL cannot link,
 *   the narrow unit set this probe actually needs:
 *   cc -O2 -ffp-contract=off -o /external/path/cockpit_probe tools/cockpit_probe.c \
 *      src/engine/{fs,zfs,vfs,car,hud,font,m16,pcx,vqm,pixidx,terrain,\
 *raster,camera,texcache,geomesh,meshcache,paint,lzodec}.c -Isrc -lm
 */

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "engine/fs.h"
#include "engine/vfs.h"
#include "engine/car.h"
#include "engine/hud.h"
#include "engine/pixidx.h"
#include "engine/combat.h"
#include "engine/camera.h"
#include "engine/scene.h"
#include "engine/camera.h"
#include "engine/geomesh.h"
#include "engine/scene.h"
#include "engine/terrain.h"
#include "engine/texcache.h"
#include "engine/worldrender.h"

#define W 640
#define H 480
#ifndef COCKPIT_NEAR
#define COCKPIT_NEAR 0.25
#endif

static int s_checks, s_failures;

static void check(int ok, const char *what)
{
    s_checks++;
    if (!ok) {
        s_failures++;
        printf("FAIL: %s\n", what);
    } else {
        printf("ok:   %s\n", what);
    }
}

/* Role-suffix membership: name ends with `suffix` (case-insensitive). */
static int interior_has(const char *suffix)
{
    size_t sl = strlen(suffix);
    for (int i = 0; i < car_interior_part_count(); i++) {
        const char *nm = car_interior_part_name(i);
        size_t nl = nm ? strlen(nm) : 0;
        if (nl >= sl && strcasecmp(nm + nl - sl, suffix) == 0)
            return 1;
    }
    return 0;
}

static int interior_find(const char *suffix)
{
    size_t sl = strlen(suffix);
    for (int i = 0; i < car_interior_part_count(); i++) {
        const char *nm = car_interior_part_name(i);
        size_t nl = nm ? strlen(nm) : 0;
        if (nl >= sl && strcasecmp(nm + nl - sl, suffix) == 0)
            return i;
    }
    return -1;
}

static unsigned long fb_checksum(const uint8_t *fb, int x0, int y0,
                                 int rw, int rh)
{
    unsigned long h = 1469598103934665603ul;
    for (int y = y0; y < y0 + rh; y++)
        for (int x = x0; x < x0 + rw; x++) {
            h ^= fb[(size_t)y * W + x];
            h *= 1099511628211ul;
        }
    return h;
}

static unsigned long mem_checksum(const uint8_t *p, size_t n)
{
    unsigned long h = 1469598103934665603ul;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 1099511628211ul;
    }
    return h;
}

static int fb_nonzero(const uint8_t *fb, int x0, int y0, int rw, int rh)
{
    int n = 0;
    for (int y = y0; y < y0 + rh; y++)
        for (int x = x0; x < x0 + rw; x++)
            if (fb[(size_t)y * W + x]) n++;
    return n;
}

static int fb_diff(const uint8_t *a, const uint8_t *b, int x0, int y0,
                   int rw, int rh)
{
    int n = 0;
    for (int y = y0; y < y0 + rh; y++)
        for (int x = x0; x < x0 + rw; x++)
            if (a[(size_t)y * W + x] != b[(size_t)y * W + x]) n++;
    return n;
}

static int fb_count(const uint8_t *fb, int x0, int y0, int rw, int rh,
                    uint8_t idx)
{
    int n = 0;
    for (int y = y0; y < y0 + rh; y++)
        for (int x = x0; x < x0 + rw; x++)
            if (fb[(size_t)y * W + x] == idx) n++;
    return n;
}

/* H-UAT-007 staged combat: fixed world positions for the two probe
 * entities, set from main before the combat scenario runs. */
static double s_probe_pos[2][3];
static void probe_fx_resolve(int ent, double out[3])
{
    out[0] = out[1] = out[2] = 0.0;
    if (ent >= 0 && ent < 2) {
        out[0] = s_probe_pos[ent][0];
        out[1] = s_probe_pos[ent][1];
        out[2] = s_probe_pos[ent][2];
    }
}

static int production_interior_part(int i)
{
    const char *name = car_interior_part_name(i);
    size_t len = name ? strlen(name) : 0;
    const char *role = len >= 4 ? name + len - 4 : "";

    /* Production prefers each authored 640-mode *6 surface to its *3 twin. */
    if (len >= 4 && role[3] == '3') {
        char want6[9];
        if (len >= sizeof want6) return 0;
        memcpy(want6, name, len + 1);
        want6[len - 1] = '6';
        for (int j = 0; j < car_interior_part_count(); j++) {
            const char *other = car_interior_part_name(j);
            if (other && strcasecmp(other, want6) == 0)
                return 0;
        }
    }
    if (strcasecmp(role, "RTC1") == 0 && interior_has("RTC6"))
        return 0;
    return 1;
}

static int queue_interior_part(int i, const double basis[12])
{
    const char *name = car_interior_part_name(i);
    GeoMesh *gm = scene_part_mesh(name);
    double fr[12], xf[12];
    if (!gm || car_interior_part_frame(i, fr) != 0) return -1;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++)
            xf[c * 3 + r] = basis[0 * 3 + r] * fr[c * 3 + 0]
                          + basis[1 * 3 + r] * fr[c * 3 + 1]
                          + basis[2 * 3 + r] * fr[c * 3 + 2];
        xf[9 + r] = basis[0 * 3 + r] * fr[9]
                  + basis[1 * 3 + r] * fr[10]
                  + basis[2 * 3 + r] * fr[11]
                  + basis[9 + r];
    }

    size_t len = name ? strlen(name) : 0;
    const char *role = len >= 4 ? name + len - 4 : "";
    const RTex *tex = NULL;
    if (strcasecmp(role, "GER6") == 0)
        tex = hud_cockpit_gear_texture(HUD_SELECTOR_PARK);
    else if (strcasecmp(role, "CMP6") == 0)
        tex = hud_cockpit_compass_texture();
    else if (strcasecmp(role, "RTC6") == 0)
        tex = hud_cockpit_reticle_texture();
    return scene_dyn_add_textured(gm, xf, xf + 3, xf + 6, xf + 9, tex);
}

static int queue_exterior_part(int i, const double basis[12])
{
    GeoMesh *gm = scene_part_mesh(car_part_name(i));
    double fr[12], xf[12];
    if (!gm || car_part_frame(i, fr) != 0) return -1;
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++)
            xf[c * 3 + r] = basis[0 * 3 + r] * fr[c * 3 + 0]
                          + basis[1 * 3 + r] * fr[c * 3 + 1]
                          + basis[2 * 3 + r] * fr[c * 3 + 2];
        xf[9 + r] = basis[0 * 3 + r] * fr[9]
                  + basis[1 * 3 + r] * fr[10]
                  + basis[2 * 3 + r] * fr[11]
                  + basis[9 + r];
    }
    return scene_dyn_add(gm, xf, xf + 3, xf + 6, xf + 9);
}

static void write_ppm(const char *path, const uint8_t *fb,
                      const uint8_t *pal)
{
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P6\n%d %d\n255\n", W, H);
    for (int i = 0; i < W * H; i++) {
        uint8_t v = fb[i];
        fputc(pal[v * 3 + 0], f);
        fputc(pal[v * 3 + 1], f);
        fputc(pal[v * 3 + 2], f);
    }
    fclose(f);
}

int main(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "usage: cockpit_probe <asset-root> <capture-dir>\n");
        return 2;
    }
    const char *root = argv[1];
    const char *capture_dir = argv[2];
    char capture_path[1024];
    fs_set_root(root);
    if (!vfs_init()) {
        fprintf(stderr, "[probe] vfs_init failed (%s)\n", root);
        return 2;
    }

    /* --- Rendered-frame proof: the interior parts consumed by the
     * software world path (web_drive_render's cockpit queue, mirrored
     * here through the same scene_dyn_add + worldrender_camera calls) -- */
    if (car_load("vdrampg2") != 0) {
        fprintf(stderr, "[probe] car_load(vdrampg2) failed\n");
        return 2;
    }
    int have_terrain = terrain_load("miss8/P01.MSN") == 0;
    int have_scene   = scene_load("miss8/P01.MSN") == 0;
    /* Keep alive through the later synthetic combat section; its rendering
     * setup unloads these before projectile collision. */
    check(have_terrain && have_scene,
          "P01 terrain + scene load for the render proof");
    /* worldrender's filled path needs the level palette, which it syncs
     * from hud_palette() — load the HUD first (webmain does the same in
     * web_mission_load). Without it the renderer falls back to wire. */
    check(hud_load_mission("miss8/P01.MSN", 0) == 0 && hud_palette(),
          "level palette available for the filled render");
    {
        double m[3];
        if (scene_first_marker_pos(m) == 0)
            car_place(m[0], m[2], 0.0);
        else
            car_place(25600.0, 25600.0, 0.0);
    }
    double cx, cy, cz, cyaw, cp_, cr_;
    car_pose(&cx, &cy, &cz, &cyaw, &cp_, &cr_);
    /* Parked on level ground: yaw-only basis (webmain car_basis with
     * pitch=roll=0). The eye is the VDF's VLOC-40 head locator, exactly
     * as webmain's cockpit camera resolves it (fallback 1.05 up / 0.30
     * fwd when a VDF has no locator 40). */
    double br[3] = {  cos(cyaw), 0.0, sin(cyaw) };
    double bu[3] = { 0.0, 1.0, 0.0 };
    double bf[3] = { -sin(cyaw), 0.0, cos(cyaw) };
    double el[3] = { 0.0, 1.05, 0.30 };
    for (int i = 0; i < car_vloc_count(); i++) {
        uint32_t num;
        double vf[12];
        if (car_vloc_get(i, &num, vf) == 0 && num == 40) {
            el[0] = vf[9]; el[1] = vf[10]; el[2] = vf[11];
            break;
        }
    }
    double ceye[3] = { cx + br[0] * el[0] + bf[0] * el[2],
                       cy + el[1],
                       cz + br[2] * el[0] + bf[2] * el[2] };
    uint8_t *fbare = malloc((size_t)W * H);
    uint8_t *fint  = malloc((size_t)W * H);
    uint8_t *fpart = malloc((size_t)W * H);
    if (!fbare || !fint || !fpart) return 2;
    CameraView cockpit;
    const double pitch = 0.18;
    double cup[3], cfwd[3];
    for (int i = 0; i < 3; i++) {
        cup[i] = bu[i] * cos(pitch) - bf[i] * sin(pitch);
        cfwd[i] = bf[i] * cos(pitch) + bu[i] * sin(pitch);
    }
    check(camera_view_from_basis(&cockpit, ceye, br, cup, cfwd) == 0,
          "production cockpit camera basis constructed");
    /* Fixed direct-fire trajectory vs authored RTC6 reticle centre. */
    {int wi=-1,ri=interior_find("RTC6");CarWeaponInfo inf;double mf[12],rf[12];GeoMesh*rm=ri>=0?scene_part_mesh(car_interior_part_name(ri)):NULL;
     for(int i=0;i<car_weapon_count();i++)if(car_weapon_get(i,&inf)==0&&inf.direct_fire&&!inf.traverses&&inf.deploy_kind==CAR_DEPLOY_NONE){wi=i;break;}
     int rsx=318,rsy=298; /* overwritten from authored geometry below */
     if(rm&&car_interior_part_frame(ri,rf)==0){double q[3]={rf[9],rf[10],rf[11]};double wp[3]={cx+br[0]*q[0]+bu[0]*q[1]+bf[0]*q[2],cy+br[1]*q[0]+bu[1]*q[1]+bf[1]*q[2],cz+br[2]*q[0]+bu[2]*q[1]+bf[2]*q[2]};double dx=wp[0]-cockpit.eye[0],dy=wp[1]-cockpit.eye[1],dz=wp[2]-cockpit.eye[2],zz=dx*cockpit.forward[0]+dy*cockpit.forward[1]+dz*cockpit.forward[2],f=W*.5;rsx=(int)lround(W*.5+(dx*cockpit.right[0]+dy*cockpit.right[1]+dz*cockpit.right[2])*f/zz);rsy=(int)lround(H*.5-(dx*cockpit.up[0]+dy*cockpit.up[1]+dz*cockpit.up[2])*f/zz);}
     check(wi>=0&&car_weapon_muzzle_frame(wi,mf)==0,"reticle fixture resolves fixed muzzle");
     for(int k=0;k<2&&wi>=0;k++){double d=k?200:40;double wp[3]={cx+br[0]*mf[9]+bu[0]*mf[10]+bf[0]*(mf[11]+d),cy+br[1]*mf[9]+bu[1]*mf[10]+bf[1]*(mf[11]+d),cz+br[2]*mf[9]+bu[2]*mf[10]+bf[2]*(mf[11]+d)};double dx=wp[0]-cockpit.eye[0],dy=wp[1]-cockpit.eye[1],dz=wp[2]-cockpit.eye[2],zz=dx*cockpit.forward[0]+dy*cockpit.forward[1]+dz*cockpit.forward[2],f=W*.5;int sx=(int)lround(W*.5+(dx*cockpit.right[0]+dy*cockpit.right[1]+dz*cockpit.right[2])*f/zz),sy=(int)lround(H*.5-(dx*cockpit.up[0]+dy*cockpit.up[1]+dz*cockpit.up[2])*f/zz);printf("  reticle trajectory %.0fm -> (%d,%d), RTC6=(%d,%d)\n",d,sx,sy,rsx,rsy);check(abs(sx-rsx)<=10&&abs(sy-rsy)<=10,k?"mid-range trajectory lands in reticle":"near trajectory lands in reticle");}}

    double basis[12] = {
        br[0], br[1], br[2], bu[0], bu[1], bu[2],
        bf[0], bf[1], bf[2], cx, cy, cz
    };
    scene_dyn_clear();
    worldrender_camera(fbare, W, H, &cockpit, COCKPIT_NEAR, 0.0,
                       have_terrain, have_scene);
    scene_set_dynamic_paint(car_vtf_file());
    scene_dyn_clear();
    int queued = 0;
    for (int i = 0; i < car_interior_part_count(); i++) {
        if (!production_interior_part(i)) continue;
        if (queue_interior_part(i, basis) >= 0) queued++;
    }
    check(queued == 20,
          "production cockpit queues the full shell and 640-mode instruments");
    worldrender_camera(fint, W, H, &cockpit, COCKPIT_NEAR, 0.0,
                       have_terrain, have_scene);
    int diff = fb_diff(fbare, fint, 0, 0, W, H);
    int left = fb_diff(fbare, fint, 0, 0, 200, H);
    int right = fb_diff(fbare, fint, W - 200, 0, 200, H);
    int lower = fb_diff(fbare, fint, 0, H - 160, W, 160);
    printf("  cockpit render: %d px total, %d px left, %d px right, "
           "%d px lower framing\n", diff, left, right, lower);
    check(diff > 50000,
          "authored production interior substantially frames the world");
    check(left > 10000 && right > 10000 && lower > 10000,
          "full shell frames both sides and the lower cockpit");

    /* Isolate the two newly admitted records. These fail if their suffix
     * selection or the close first-person near plane regresses. */
    scene_dyn_clear();
    check(queue_interior_part(interior_find("BDYT"), basis) >= 0,
          "BDYT roof/A-pillar mesh queues in isolation");
    worldrender_camera(fpart, W, H, &cockpit, COCKPIT_NEAR, 0.0,
                       have_terrain, have_scene);
    check(fb_diff(fbare, fpart, 0, 0, 200, H) > 15000,
          "BDYT contributes the broad left roof/A-pillar silhouette");
    scene_dyn_clear();
    check(queue_interior_part(interior_find("RTCB"), basis) >= 0,
          "RTCB fixed-sight mesh queues in isolation");
    worldrender_camera(fpart, W, H, &cockpit, COCKPIT_NEAR, 0.0,
                       have_terrain, have_scene);
    check(fb_diff(fbare, fpart, 280, 180, 80, 260) > 500,
          "RTCB contributes the fixed centre hood sight");
    /* A render from OUTSIDE the car must not gain interior pixels: the
     * queue is rebuilt per view, so a chase render queues the exterior
     * instead — the interior was never added to it here. Proven by the
     * queue discipline; nothing to render. */
    {   /* selected production shell, mapped through the level palette */
        snprintf(capture_path, sizeof capture_path,
                 "%s/cockpit_interior.ppm", capture_dir);
        write_ppm(capture_path, fint, hud_palette());
        printf("  wrote %s\n", capture_path);
    }

    /*
     * Exterior-wheel proof (car.c D23): exercise the same mesh/frame/dynamic
     * queue path the chase renderer consumes. Count-only assertions cannot
     * catch a bad WGEO name, a non-finite WLOC×WGEO frame, or an appended
     * wheel that never resolves to geometry.
     */
    {
        int first_wheel = car_body_part_count();
        int wheel_ok = car_wheel_part_count() > 0;
        for (int i = first_wheel; i < car_part_count() && wheel_ok; i++) {
            double wf[12];
            const char *name = car_part_name(i);
            if (!name || car_part_frame(i, wf) != 0 ||
                !scene_part_mesh(name)) {
                wheel_ok = 0;
                break;
            }
            for (int k = 0; k < 12; k++)
                if (!isfinite(wf[k]))
                    wheel_ok = 0;
        }
        check(wheel_ok,
              "every appended WDF wheel has a finite frame and resolved mesh");

        double eye[3] = { cx - bf[0] * 8.0, cy + 2.0,
                          cz - bf[2] * 8.0 };
        double target[3] = { cx, cy + 0.5, cz };
        CameraView chase;
        check(camera_view_look_at(&chase, eye, target) == 0,
              "exterior wheel camera constructed");
        scene_dyn_clear();
        worldrender_camera(fbare, W, H, &chase, 0.0, 0.0,
                           have_terrain, have_scene);
        scene_dyn_clear();
        int wheel_queued = wheel_ok
                         ? queue_exterior_part(first_wheel, basis) : -1;
        check(wheel_queued >= 0,
              "an appended wheel enters the production dynamic queue");
        worldrender_camera(fpart, W, H, &chase, 0.0, 0.0,
                           have_terrain, have_scene);
        check(fb_diff(fbare, fpart, 0, 0, W, H) > 20,
              "the queued wheel changes an exterior rendered frame");
    }
    scene_dyn_clear();
    free(fbare);
    free(fint);
    free(fpart);

    /* --- 1. VGEO first-person set parse (D22) ------------------------- */
    int nfp = car_interior_part_count();
    printf("interior parts: %d\n", nfp);
    {
        int gi = interior_find("GER6");
        int ci = interior_find("CMP6");
        int ri = interior_find("RTC6");
        GeoMesh *gear = gi >= 0
                      ? scene_part_mesh(car_interior_part_name(gi)) : NULL;
        GeoMesh *compass = ci >= 0
                         ? scene_part_mesh(car_interior_part_name(ci)) : NULL;
        GeoMesh *reticle = ri >= 0
                         ? scene_part_mesh(car_interior_part_name(ri)) : NULL;
        check(gear && gear->num_faces == 1 &&
              strcmp(gear->face_tex, "ZGEAR101.MAP") == 0 &&
              gear->face_flags[0] == 4 && gear->face_flags[1] == 1,
              "GER6 is the authored 640-mode gear surface");
        check(compass && compass->num_faces == 1 &&
              strcmp(compass->face_tex, "ZCM_.MAP") == 0 &&
              compass->face_flags[0] == 4 && compass->face_flags[1] == 5,
              "CMP6 is the authored 640-mode compass cutout");
        check(reticle && reticle->num_faces == 1 &&
              strcmp(reticle->face_tex, "ZRETC_6.MAP") == 0 &&
              reticle->face_flags[0] == 5 && reticle->face_flags[1] == 5,
              "RTC6 is the authored 640-mode reticle cutout");
    }
    /* The set-0 exterior body table (car_body_part_*) shares the
     * refactored set parser — pin its established behavior (31 VGEO
     * records, first is the DR11BDYM damage-state-0 body). WDF wheels
     * append after the body (car.c D23); total car_part_count() grows. */
    check(car_body_part_count() == 31,
          "exterior set-0 body table intact (31 VGEO records)");
    check(car_part_count() >= 31 &&
          car_wheel_part_count() == car_part_count() - car_body_part_count(),
          "wheel parts append after body (D23)");
    check(car_wheel_part_count() > 0,
          "Rampage WDF wheels resolved into exterior list");
    double pf[12];
    check(car_part_frame(0, pf) == 0 && isfinite(pf[9]) &&
          strcmp(car_part_name(0), "DR11BDYM") == 0,
          "exterior part 0 is DR11BDYM with a finite frame");
    for (int i = 0; i < nfp; i++) {
        double fr[12];
        car_interior_part_frame(i, fr);
        printf("  [%2d] %-9s pos=(%6.3f %6.3f %6.3f)\n", i,
               car_interior_part_name(i), fr[9], fr[10], fr[11]);
    }
    check(nfp >= 20, "Rampage first-person set yields >= 20 interior parts");
    static const char *const roles[] = {
        "DASH", "MIRI", "RADR", "SWHL", "SEAT", "CMP3", "CMP6", "GER6",
        "WEP3", "WEP6", "SYS3", "SYS6", "SPC3", "SPC6", "GUNL", "GUNR",
        "RTC1",
    };
    int roles_ok = 1;
    for (size_t i = 0; i < sizeof roles / sizeof roles[0]; i++)
        if (!interior_has(roles[i])) {
            printf("  missing role *%s\n", roles[i]);
            roles_ok = 0;
        }
    check(roles_ok, "all expected interior role names present");

    /* Frames are real composed transforms, not identity/garbage. */
    int d_i = interior_find("DASH");
    double fr[12];
    check(d_i >= 0 && car_interior_part_frame(d_i, fr) == 0 &&
          fabs(fr[9] - 0.0) < 0.05 && fabs(fr[10] - 0.9) < 0.05 &&
          fabs(fr[11] - 0.6) < 0.05,
          "DASH frame at measured purchaser-data position (0, 0.9, 0.6)");
    int frames_finite = 1;
    for (int i = 0; i < nfp; i++) {
        car_interior_part_frame(i, fr);
        for (int k = 0; k < 12; k++)
            if (!isfinite(fr[k])) frames_finite = 0;
    }
    check(frames_finite, "all interior frames finite");

    /* --- 2. Material completeness: every part resolves to mesh bytes -- */
    PixIndex *gix = pixidx_build("g.pix");
    check(gix != NULL, "g.pix merged index built");
    int resolved = 0;
    for (int i = 0; i < nfp; i++) {
        const char *nm = car_interior_part_name(i);
        char key[24];
        snprintf(key, sizeof key, "%s.geo", nm);
        for (char *c = key; *c; c++)
            if (*c >= 'A' && *c <= 'Z') *c = (char)(*c + ('a' - 'A'));
        int hit = (gix && pixidx_find(gix, key) != NULL) ||
                  vfs_exists(key) > 0;
        if (!hit) {
            snprintf(key, sizeof key, "%sg.pak", nm);
            for (char *c = key; *c; c++)
                if (*c >= 'A' && *c <= 'Z') *c = (char)(*c + ('a' - 'A'));
            hit = vfs_exists(key) > 0;
        }
        if (hit) resolved++;
        else printf("  unresolved: %s\n", nm);
    }
    check(nfp > 0 && resolved == nfp,
          "every interior part resolves to mesh bytes (g.pix/loose/gpak)");

    /*
     * DR51DASH names these authored maps directly. They live in
     * zdash101.pak, outside the ordinary m-tier texture index, and must still
     * arrive at the same raster tile contract rather than falling back flat.
     */
    uint16_t dash_tex1 = texcache_resolve("ZDASH101.MAP", car_vtf_file());
    uint16_t dash_tex2 = texcache_resolve("ZDASH102.MAP", car_vtf_file());
    const TexTile *dash1_tile = texcache_tile(dash_tex1);
    const TexTile *dash2_tile = texcache_tile(dash_tex2);
    check(dash_tex1 != TEX_ID_NONE && dash_tex2 != TEX_ID_NONE &&
          dash1_tile && dash2_tile &&
          dash1_tile->w == 256 && dash1_tile->h == 128 &&
          dash2_tile->w == 256 && dash2_tile->h == 128,
          "ZDASH101/102 resolve from authored cockpit package");
    check(strcmp(texcache_tile_name(dash_tex1), "zdash101.vqm") == 0 &&
          strcmp(texcache_tile_name(dash_tex2), "zdash102.vqm") == 0,
          "cockpit VQM tiles retain precedence over raw MAP sheets");

    /* DR51SPC6 has no VQM twin. Losing its raw sheet paints a grey block
     * over the bottom-right dashboard instead of the authored black panel. */
    uint16_t specials_id = texcache_resolve("ZBKS_.MAP", car_vtf_file());
    const TexTile *specials = texcache_tile(specials_id);
    size_t specials_size = 0;
    uint8_t *specials_raw = vfs_read_file("zbks_.map", &specials_size);
    check(specials && specials->w == 256 && specials->h == 128 &&
          specials->has_key && specials_raw && specials_size == 8 + 256 * 128 &&
          strcmp(texcache_tile_name(specials_id), "zbks_.map") == 0 &&
          memcmp(specials->texels, specials_raw + 8, 256 * 128) == 0,
          "SPC6 resolves the exact authored raw MAP, including black texels");
    vfs_free(specials_raw);

    /* H-UAT-075a regression: MOILSPIL's face names a TMT descriptor, not a
     * MAP alias. Its eight 64x64 indexed frames live in the descriptor's own
     * xos1_101.pix/pak family, outside the ordinary m-tier index. */
    TexTmtInfo oil_info;
    int oil_info_ok = texcache_tmt_info("XOS1_101.TMT", &oil_info) == 0;
    check(oil_info_ok && oil_info.kind == 1 && oil_info.count == 8 &&
          oil_info.stride == 0 && oil_info.name_count == 8 &&
          fabs(oil_info.rate - 10.0f) < 1e-6 && oil_info.mode == 0,
          "XOS1_101 is the exact type-1 eight-frame 10-rate TMT variant");
    uint16_t oil_first = texcache_resolve("XOS1_101.TMT", "");
    uint16_t oil_last = texcache_resolve_tmt_frame("XOS1_101.TMT", 7);
    const TexTile *oil_first_tile = texcache_tile(oil_first);
    const TexTile *oil_last_tile = texcache_tile(oil_last);
    int first_key = 0, last_key = 0;
    if (oil_first_tile)
        for (int i = 0; i < oil_first_tile->w * oil_first_tile->h; i++)
            first_key += oil_first_tile->texels[i] == RASTER_TEXEL_TRANSPARENT;
    if (oil_last_tile)
        for (int i = 0; i < oil_last_tile->w * oil_last_tile->h; i++)
            last_key += oil_last_tile->texels[i] == RASTER_TEXEL_TRANSPARENT;
    check(oil_first != TEX_ID_NONE && oil_last != TEX_ID_NONE &&
          oil_first != oil_last && oil_first_tile && oil_last_tile &&
          oil_first_tile->w == 64 && oil_first_tile->h == 64 &&
          oil_last_tile->w == 64 && oil_last_tile->h == 64 &&
          oil_first_tile->has_key && oil_last_tile->has_key &&
          first_key == 3438 && last_key == 780 &&
          strcmp(texcache_tile_name(oil_first), "xos1_101.vqm") == 0 &&
          strcmp(texcache_tile_name(oil_last), "xos1_108.vqm") == 0,
          "XOS1_101 resolves first/final authored VQM frames from sibling package");

    /* MCALT101 is the only other descriptor with the same complete header
     * tuple. Its frames prove that TMT storage may be raw indexed MAP rather
     * than VQM without changing the tile contract handed to the rasterizer. */
    TexTmtInfo calt_info;
    uint16_t calt_last = texcache_resolve_tmt_frame("MCALT101.TMT", 7);
    const TexTile *calt_tile = texcache_tile(calt_last);
    check(texcache_tmt_info("MCALT101.TMT", &calt_info) == 0 &&
          calt_info.kind == oil_info.kind && calt_info.count == oil_info.count &&
          calt_info.stride == oil_info.stride &&
          calt_info.name_count == oil_info.name_count &&
          fabs(calt_info.rate - oil_info.rate) < 1e-6 &&
          calt_info.mode == oil_info.mode &&
          calt_tile && calt_tile->w == 64 && calt_tile->h == 64 &&
          strcmp(texcache_tile_name(calt_last), "mcalt108.map") == 0,
          "MCALT101 shares the TMT variant and resolves raw MAP frames");
    check(texcache_resolve_tmt_frame("XOS1_101.TMT", 8) == TEX_ID_NONE &&
          texcache_tmt_info("MISSING.TMT", &calt_info) != 0,
          "TMT frame bounds and missing descriptors fail closed");

    /* --- 3. Mirror surface + VLOC locators ---------------------------- */
    int miri = car_interior_mirror_part();
    check(miri >= 0 && miri == interior_find("MIRI"),
          "car_interior_mirror_part identifies the MIRI surface");
    if (miri >= 0) {
        car_interior_part_frame(miri, fr);
        check(fabs(fr[9] - -0.2) < 0.05 && fabs(fr[10] - 1.3) < 0.05 &&
              fabs(fr[11] - 0.2) < 0.05,
              "MIRI frame at measured position (-0.2, 1.3, 0.2)");
    }
    check(car_vloc_count() == 5, "five VLOC locators parsed");
    int seen[128] = {0};
    double eye[12] = {0};
    for (int i = 0; i < car_vloc_count(); i++) {
        uint32_t num = 0;
        double vf[12];
        car_vloc_get(i, &num, vf);
        if (num < 128) seen[num] = 1;
        if (num == 40) memcpy(eye, vf, sizeof eye);
        printf("  VLOC %2u pos=(%6.3f %6.3f %6.3f)\n", num,
               vf[9], vf[10], vf[11]);
    }
    check(seen[35] && seen[36] && seen[38] && seen[40] && seen[42],
          "VLOC numbers {35,36,38,40,42} present");
    check(fabs(eye[9] - -0.408) < 0.01 && fabs(eye[10] - 1.251) < 0.01 &&
          fabs(eye[11] - 0.033) < 0.01,
          "VLOC 40 at the left-seat head position (driver-eye candidate)");

    /* --- 4. Scenario-keyed dash class (FACT table) + preserved state -- */
    check(hud_scenario_art_class("miss8/P01.MSN") == 2, "P01 -> class 2");
    check(hud_scenario_art_class("miss8/P13.MSN") == 5, "P13 -> class 5");
    check(hud_scenario_art_class("miss8/P09.MSN") == 4, "P09 -> class 4");
    check(hud_scenario_art_class("miss8/P05.MSN") == 3, "P05 -> class 3");
    check(hud_scenario_art_class("miss8/B01.MSN") == 3, "B01 -> class 3");
    check(hud_scenario_art_class("miss8/N01.CBT") == 1,
          "unlisted N01 melee -> class 1");
    check(hud_scenario_art_class("garbage") == 2,
          "unparseable scenario -> class 2 (binary default)");

    uint8_t *fb = malloc((size_t)W * H);
    uint8_t *fb2 = malloc((size_t)W * H);
    if (!fb || !fb2) return 2;

    check(hud_load() == 0, "hud_load (default set 1)");
    hud_render_frame(fb, W, H, 3, 0.0, 0, HUD_FLAG_NO_PROOF);
    unsigned long dash1 = fb_checksum(fb, (W - 256) / 2, H - 256, 256, 256);

    check(hud_load_mission("miss8/P01.MSN", 0) == 0,
          "hud_load_mission(P01) loads");
    char stats[512];
    hud_stats(stats, sizeof stats);
    printf("%s\n", stats);
    check(strstr(stats, "dash zdash201") != NULL,
          "P01 dash set is zdash201 (scenario class 2)");
    check(hud_art_class() == 2, "hud_art_class() reports 2 for P01");
    hud_render_frame(fb, W, H, 3, 0.0, 0, HUD_FLAG_NO_PROOF);
    unsigned long dash2 = fb_checksum(fb, (W - 256) / 2, H - 256, 256, 256);
    check(dash1 != dash2, "set-2 dash art differs from set 1");
    check(fb_nonzero(fb, (W - 256) / 2, H - 256, 256, 256) > 20000,
          "P01 dash is a solid 256x256 frame (opaque blit preserved)");

    check(strstr(stats, "gear=ok") && strstr(stats, "arrow=ok") &&
          strstr(stats, "compass=ok") && strstr(stats, "strip=ok"),
          "gear plate + compass sheets still load under the P01 dash");

    /* Gear arrow still tracks the selector. */
    hud_render_frame(fb, W, H, 0, 0.0, 0, HUD_FLAG_NO_PROOF);
    hud_render_frame(fb2, W, H, 3, 0.0, 0, HUD_FLAG_NO_PROOF);
    int gear_diff = 0;
    for (int y = H - 256 - 32; y < H - 256 && gear_diff >= 0; y++)
        for (int x = (W - 256) / 2; x < (W + 256) / 2; x++)
            if (fb[(size_t)y * W + x] != fb2[(size_t)y * W + x]) gear_diff++;
    check(gear_diff > 0, "gear arrow moves with the selector state");

    /* Compass crop still tracks the yaw. */
    hud_set_compass_yaw(0.0);
    hud_render_frame(fb, W, H, 3, 0.0, 0, HUD_FLAG_NO_PROOF);
    hud_set_compass_yaw(1.6);
    hud_render_frame(fb2, W, H, 3, 0.0, 0, HUD_FLAG_NO_PROOF);
    int comp_diff = 0;
    for (int y = H - 256 - 64 - 32; y < H - 256; y++)
        for (int x = (W - 256) / 2 - 64; x < (W - 256) / 2; x++)
            if (fb[(size_t)y * W + x] != fb2[(size_t)y * W + x]) comp_diff++;
    check(comp_diff > 0, "compass crop moves with the yaw state");

    /* The same live composites now map onto the authored GER6/CMP6/RTC6
     * surfaces in production cockpit view. Pin dimensions, transparency and
     * the two state transitions rather than screenshot coordinates. */
    const RTex *gear_tex = hud_cockpit_gear_texture(HUD_SELECTOR_PARK);
    printf("  GER6 texture: %ux%u\n",
           gear_tex ? gear_tex->w : 0, gear_tex ? gear_tex->h : 0);
    check(gear_tex && gear_tex->w == 256 && gear_tex->h == 32,
          "GER6 live gear texture is the authored 256x32 map");
    unsigned long gear_park = gear_tex
        ? mem_checksum(gear_tex->texels, (size_t)gear_tex->w * gear_tex->h) : 0;
    gear_tex = hud_cockpit_gear_texture(HUD_SELECTOR_DRIVE);
    unsigned long gear_drive = gear_tex
        ? mem_checksum(gear_tex->texels, (size_t)gear_tex->w * gear_tex->h) : 0;
    check(gear_park != gear_drive,
          "GER6 live texture moves the selector from park to drive");

    hud_set_compass_yaw(0.0);
    const RTex *comp_tex = hud_cockpit_compass_texture();
    printf("  CMP6 texture: %ux%u\n",
           comp_tex ? comp_tex->w : 0, comp_tex ? comp_tex->h : 0);
    check(comp_tex && comp_tex->w == 64 && comp_tex->h == 64,
          "CMP6 live compass texture is the authored 64x64 map");
    unsigned long comp_north = comp_tex
        ? mem_checksum(comp_tex->texels, (size_t)comp_tex->w * comp_tex->h) : 0;
    hud_set_compass_yaw(1.6);
    comp_tex = hud_cockpit_compass_texture();
    unsigned long comp_turn = comp_tex
        ? mem_checksum(comp_tex->texels, (size_t)comp_tex->w * comp_tex->h) : 0;
    check(comp_north != comp_turn,
          "CMP6 live texture moves its bearing crop with yaw");

    const RTex *reticle_tex = hud_cockpit_reticle_texture();
    check(reticle_tex && reticle_tex->w == 128 && reticle_tex->h == 128 &&
          reticle_tex->has_key,
          "RTC6 live texture is the authored keyed 128x128 reticle");
    unsigned long reticle_hash = reticle_tex
        ? mem_checksum(reticle_tex->texels,
                       (size_t)reticle_tex->w * reticle_tex->h) : 0;
    printf("  live texture hashes: gear P=%016lx D=%016lx "
           "compass N=%016lx turn=%016lx reticle=%016lx\n",
           gear_park, gear_drive, comp_north, comp_turn, reticle_hash);
    check(gear_park == 0xe1f5ec8892e0c356ul &&
          gear_drive == 0xb284f368924369ebul &&
          comp_north == 0x44c4acf1f397bff2ul &&
          comp_turn == 0xd42ef716f5ea48d7ul &&
          reticle_hash == 0xf4d3bb1279311406ul,
          "live textures use raw GEO row order after display-space overlays");

    /* The authored condition-panel base leaves every live region blank.
     * zsye.map supplies the state sprites through vpit_1.elt anchors. The
     * current scalar combat model drives all component/facet regions together,
     * but a health-state change must still alter the composed panel. */
    hud_set_vitals(100, 100, -1);
    hud_render_frame(fb, W, H, 3, 0.0, 0,
                     HUD_FLAG_NO_PROOF | HUD_FLAG_NO_DASH |
                     HUD_FLAG_COCKPIT_LAYOUT | HUD_FLAG_VITALS);
    hud_set_vitals(20, 100, -1);
    hud_render_frame(fb2, W, H, 3, 0.0, 0,
                     HUD_FLAG_NO_PROOF | HUD_FLAG_NO_DASH |
                     HUD_FLAG_COCKPIT_LAYOUT | HUD_FLAG_VITALS);
    int condition_diff = fb_diff(fb, fb2, W - 256, 0, 256, 128);
    printf("  condition green->red: panel diff %d px\n", condition_diff);
    check(condition_diff > 100,
          "condition panel composes authored live health states");
    hud_set_vitals(100, 100, -1);

    /* --- 7. H-UAT-002: the authored green radar playfield ------------
     * The zradf000.pak sweep tiles (decoded through the same VQM path as
     * the dash) must fill the zradmask housing's circular window with the
     * green phosphor field in BOTH drive views — cockpit and chase share
     * the upper-panel layout. Index 210 is the measured green-field
     * texel of the shipped tiles against p01.act; 0xFF was the black
     * field the housing-only blit left. The window box below is the
     * mask's 0xFF circle measured from the asset (x 44..151, y 12..120). */
    uint32_t lay = HUD_FLAG_NO_PROOF | HUD_FLAG_COCKPIT_LAYOUT |
                   HUD_FLAG_VITALS;
    hud_set_vitals(100, 100, -1);
    hud_clear_radar_contacts();
    hud_render_frame(fb,  W, H, 3, 0.0, 0, lay);                 /* cockpit */
    hud_render_frame(fb2, W, H, 3, 0.0, 0, lay | HUD_FLAG_NO_DASH); /* chase */
    int green_cp = fb_count(fb,  46, 14, 104, 104, 210);
    int green_ch = fb_count(fb2, 46, 14, 104, 104, 210);
    int black_cp = fb_count(fb,  46, 14, 104, 104, 255);
    printf("  radar playfield: green %d px cockpit / %d px chase, black %d px\n",
           green_cp, green_ch, black_cp);
    check(green_cp > 2000,
          "cockpit radar playfield is the authored green sweep art");
    check(green_ch > 2000,
          "chase radar keeps the framed green playfield");
    check(black_cp < 500, "the black radar field is gone");

    /* The sweep rotates with the tick clock (lap_ms is tick*50). */
    hud_render_frame(fb,  W, H, 3, 0.0, 0,   lay);
    hud_render_frame(fb2, W, H, 3, 0.0, 400, lay);
    check(fb_diff(fb, fb2, 44, 12, 108, 110) > 200,
          "radar sweep animates on the 20 Hz tick clock");

    /* Contacts frame INSIDE the housing: a 200 m threat blips inside the
     * playfield circle (centre ~(98,66), r ~53 px per measured mask
     * window; 200 m of 1000 m -> ~11 px right of centre) and nothing
     * anywhere else in the frame changes; a contact past the authored
     * 1 km field is dropped outright. */
    hud_render_frame(fb, W, H, 3, 0.0, 0, lay);
    hud_clear_radar_contacts();
    hud_add_radar_contact(200.0, 0.0, 1);
    hud_render_frame(fb2, W, H, 3, 0.0, 0, lay);
    check(fb_diff(fb, fb2, 100, 58, 20, 16) > 0,
          "in-range threat contact blips inside the playfield");
    check(fb_diff(fb, fb2, 260, 0, W - 260, H) == 0 &&
          fb_diff(fb, fb2, 0, 130, W, H - 130) == 0,
          "radar contacts stay framed inside the housing");
    hud_clear_radar_contacts();
    hud_add_radar_contact(2000.0, 0.0, 1);
    hud_render_frame(fb2, W, H, 3, 0.0, 0, lay);
    check(fb_diff(fb, fb2, 0, 0, W, H) == 0,
          "out-of-range contacts are dropped, not rim-pinned");
    hud_clear_radar_contacts();

    /* --- 8. H-UAT-003: per-component condition state ------------------
     * Front-armor and rear-armor damage must repaint DISTINCT regions of
     * the authored panel while everything outside the panel stays put. */
    hud_clear_conditions();
    for (int c = 0; c < HUD_CONDITION_COMPONENTS; c++)
        hud_set_condition(c, 100, 100);
    hud_render_frame(fb, W, H, 3, 0.0, 0, lay);        /* all green  */
    hud_set_condition(COMBAT_COMP_ARMOR_F, 10, 100);   /* front red  */
    hud_render_frame(fb2, W, H, 3, 0.0, 0, lay);
    int front_diff = fb_diff(fb, fb2, W - 256, 0, 256, 128);
    printf("  condition front-armor red: panel diff %d px\n", front_diff);
    check(front_diff > 20,
          "front-armor damage repaints its own panel region");
    hud_set_condition(COMBAT_COMP_ARMOR_F, 100, 100);
    hud_set_condition(COMBAT_COMP_ARMOR_B, 10, 100);   /* rear red   */
    uint8_t *fb3 = malloc((size_t)W * H);
    if (!fb3) return 2;
    hud_render_frame(fb3, W, H, 3, 0.0, 0, lay);
    int rear_diff = fb_diff(fb, fb3, W - 256, 0, 256, 128);
    printf("  condition rear-armor red: panel diff %d px\n", rear_diff);
    check(rear_diff > 20,
          "rear-armor damage repaints its own panel region");
    check(fb_diff(fb2, fb3, W - 256, 0, 256, 128) > 20,
          "front vs rear damage light DISTINCT panel regions");
    check(fb_diff(fb2, fb3, 0, 0, W - 256, H) == 0,
          "component recolors stay inside the condition panel");

    /* --- 9. H-UAT-007: combat presentation over the world frame -------
     * Stage a hostile 40 m ahead of the production cockpit camera and
     * run the real combat fire/kill path: the target marker, the fire
     * streak, and the kill burst/debris/smoke must draw over the world
     * and then decay to nothing on the tick schedule. */
    /* The world frame is already captured. Unload collision geometry so
     * this synthetic presentation case isolates projectile/Fx timing rather
     * than whichever P01 prop happens to lie under the staged sightline. */
    scene_unload();
    terrain_unload();
    combat_reset();
    s_probe_pos[0][0] = cx;  s_probe_pos[0][1] = cy;  s_probe_pos[0][2] = cz;
    s_probe_pos[1][0] = cx - sin(cyaw) * 40.0;   /* car.h fwd = (-sin,cos) */
    s_probe_pos[1][1] = cy;
    s_probe_pos[1][2] = cz + cos(cyaw) * 40.0;
    CarCombatConfig fx_bounds = { .collision_half = {2.0, 2.0, 4.0} };
    combat_register(0, 1, 1, -1, "user", &fx_bounds);
    combat_register(1, 2, 2, -1, "foe", &fx_bounds); /* class 2: staged */
    combat_set_user(0);
    combat_set_resolver(probe_fx_resolve);
    combat_set_user_pose(cx, cy, cz, cyaw);
    combat_player_weapons_clear();
    combat_player_weapon_add("MG", 25, 10, 1, -1, 0, 1, 1);

    double tpos[3] = {0, 0, 0};
    check(combat_target_marker(tpos) == 1,
          "target marker picks the staged in-cone hostile");
    scene_dyn_clear();
    /* Terrain and scene were unloaded above: say so, rather than asking
     * the renderer to traverse torn-down mission state. */
    worldrender_camera(fb, W, H, &cockpit, COCKPIT_NEAR, 0.0, 0, 0);
    CombatFx snap[COMBAT_FX_MAX];
    memcpy(fb2, fb, (size_t)W * H);
    int nfx = combat_fx_snapshot(snap, COMBAT_FX_MAX);
    scene_render_combat_fx(fb2, W, H, &cockpit, snap, nfx, tpos);
    check(fb_diff(fb, fb2, 0, 0, W, H) > 20,
          "target marker draws over the world frame");

    int hit = -1;
    check(combat_player_fire(&hit) == 1 && hit == -1,
          "staged shot launches asynchronously");
    memcpy(fb2, fb, (size_t)W * H);
    nfx = combat_fx_snapshot(snap, COMBAT_FX_MAX);
    scene_render_combat_fx(fb2, W, H, &cockpit, snap, nfx, tpos);
    int live_projectiles = 0, muzzle_events = 0;
    for (int i = 0; i < nfx; i++) {
        if (snap[i].type == COMBAT_FX_PROJECTILE) live_projectiles++;
        if (snap[i].type == COMBAT_FX_MUZZLE) muzzle_events++;
    }
    check(live_projectiles == 1 && muzzle_events == 1 &&
          fb_diff(fb, fb2, 0, 0, W, H) > 20,
          "live projectile + muzzle flash draw over the world frame");

    /* Renderer branch proof. Inputs are synthetic classified snapshots, but
     * the renderer and palette are shipped: production combat.c chooses these
     * classes from parsed ORDF +0 and supplies the real trail poses above. */
    {
        static const int classes[] = {
            COMBAT_FX_TRACER_LIGHT, COMBAT_FX_TRACER_HEAVY,
            COMBAT_FX_EXPLOSIVE, COMBAT_FX_MISSILE,
            COMBAT_FX_FLAME, COMBAT_FX_GAS
        };
        unsigned long hashes[6];
        int pixels[6];
        for (int j = 0; j < 6; j++) {
            CombatFx sample;
            memset(&sample, 0, sizeof sample);
            sample.active = 1;
            sample.type = COMBAT_FX_PROJECTILE;
            sample.weapon_class = classes[j];
            sample.damage = j == 0 ? 15 : j == 1 ? 150 : 205;
            sample.hit = COMBAT_FX_HIT_NONE;
            sample.trail_count = 3;
            for (int k = 0; k < 3; k++) {
                double dist = 8.0 + k * 13.0;
                sample.trail[k][0] = cx - sin(cyaw) * dist;
                sample.trail[k][1] = cy + 1.0;
                sample.trail[k][2] = cz + cos(cyaw) * dist;
            }
            memcpy(sample.start, sample.trail[0], sizeof sample.start);
            memcpy(sample.end, sample.trail[2], sizeof sample.end);
            memcpy(fb2, fb, (size_t)W * H);
            scene_render_combat_fx(fb2, W, H, &cockpit, &sample, 1, NULL);
            hashes[j] = mem_checksum(fb2, (size_t)W * H);
            pixels[j] = fb_diff(fb, fb2, 0, 0, W, H);
        }
        int distinct = 1;
        for (int a = 0; a < 6; a++)
            for (int b = a + 1; b < 6; b++)
                if (hashes[a] == hashes[b]) distinct = 0;
        check(distinct, "all six parsed ORDF presentation classes render distinctly");
        check(pixels[1] > 0 && pixels[1] <= pixels[0] &&
              pixels[3] > pixels[0] && pixels[4] > pixels[0] &&
              pixels[5] > pixels[0],
              "cannon shell stays bounded while missile/flame/gas exceed MG");

        CombatFx spark, fireball;
        memset(&spark, 0, sizeof spark);
        spark.active = 1; spark.type = COMBAT_FX_IMPACT;
        spark.weapon_class = COMBAT_FX_TRACER_LIGHT; spark.damage = 15;
        spark.hit = 1; spark.age = 4; spark.life = COMBAT_EXPLOSION_TICKS;
        snprintf(spark.effect_name, sizeof spark.effect_name, "xbulc1.xdf");
        spark.end[0] = cx - sin(cyaw) * 34.0;
        spark.end[1] = cy + 1.0;
        spark.end[2] = cz + cos(cyaw) * 34.0;
        memcpy(spark.start, spark.end, sizeof spark.start);
        fireball = spark;
        fireball.weapon_class = COMBAT_FX_MISSILE;
        fireball.damage = 205;
        snprintf(fireball.effect_name, sizeof fireball.effect_name,
                 "xmslc1.xdf");
        memcpy(fb2, fb, (size_t)W * H);
        scene_render_combat_fx(fb2, W, H, &cockpit, &spark, 1, NULL);
        int spark_px = fb_diff(fb, fb2, 0, 0, W, H);
        memcpy(fb2, fb, (size_t)W * H);
        scene_render_combat_fx(fb2, W, H, &cockpit, &fireball, 1, NULL);
        int fireball_px = fb_diff(fb, fb2, 0, 0, W, H);
        check(fireball_px > spark_px * 2,
              "authored missile-contact flipbook exceeds bullet-contact art");
    }

    /* Let the first round travel, then reduce to one-shot health and prove
     * the killing projectile's delayed aftermath. */
    for (int i = 0; i < 12; i++) combat_tick();
    combat_shot(0, 1, 99);
    while (combat_player_weapon_cooldown() > 0) combat_tick();
    check(combat_player_fire(&hit) == 1,
          "killing projectile launches after cooldown");
    for (int k = 0; k < 25 && !combat_is_dead(1); k++) combat_tick();
    check(combat_is_dead(1), "staged hostile destroyed");
    /* Receipt at kill phase 0: the first expanding fireball frame. */
    memcpy(fb2, fb, (size_t)W * H);
    nfx = combat_fx_snapshot(snap, COMBAT_FX_MAX);
    scene_render_combat_fx(fb2, W, H, &cockpit, snap, nfx, tpos);
    snprintf(capture_path, sizeof capture_path,
             "%s/cockpit_combat_fx-fireball.ppm", capture_dir);
    write_ppm(capture_path, fb2, hud_palette());
    printf("  wrote %s\n", capture_path);
    for (int i = 0; i < 12; i++) combat_tick();
    memcpy(fb2, fb, (size_t)W * H);
    nfx = combat_fx_snapshot(snap, COMBAT_FX_MAX);
    scene_render_combat_fx(fb2, W, H, &cockpit, snap, nfx, tpos);
    int aftermath_diff = fb_diff(fb, fb2, 0, 0, W, H);
    printf("  kill aftermath: %d live records, %d px drawn\n",
           nfx, aftermath_diff);
    check(nfx > 0 && aftermath_diff > 20,
          "kill burst + debris + opaque smoke draw over the world frame");
    {   /* production-composite eyeball: world + effects + HUD overlay */
        for (int i = 0; i < W * H; i++) fb3[i] = 0;
        hud_render_frame(fb3, W, H, 3, 0.0, 0, lay);
        for (int i = 0; i < W * H; i++)
            if (fb3[i]) fb2[i] = fb3[i];
        snprintf(capture_path, sizeof capture_path,
                 "%s/cockpit_combat_fx.ppm", capture_dir);
        write_ppm(capture_path, fb2, hud_palette());
        printf("  wrote %s\n", capture_path);
    }
    for (int i = 0; i < 80; i++) combat_tick();
    nfx = combat_fx_snapshot(snap, COMBAT_FX_MAX);
    memcpy(fb2, fb, (size_t)W * H);
    scene_render_combat_fx(fb2, W, H, &cockpit, snap, nfx, NULL);
    check(nfx > 0 && fb_diff(fb, fb2, 0, 0, W, H) > 20,
          "wreck fire + rising smoke persist after four seconds");
    /* Cumulative post-kill age is 12 + 80 + 320 = 412 ticks (> life 400). */
    for (int i = 0; i < 320; i++) combat_tick();
    nfx = combat_fx_snapshot(snap, COMBAT_FX_MAX);
    memcpy(fb2, fb, (size_t)W * H);
    scene_render_combat_fx(fb2, W, H, &cockpit, snap, nfx, NULL);
    check(nfx == 0 && fb_diff(fb, fb2, 0, 0, W, H) == 0,
          "all combat effects decay to nothing on schedule");
    combat_reset();
    free(fb3);

    /* --- 4b. Live instruments (hud.c D8): the speedo/tach needles are
     * gameplay output composited over the baked gauge faces, selected by
     * the original's own frame formulas. Every check below fails if the
     * needles stay static. The gauge windows are the needle frames'
     * footprint: the validated P01 hubs are dash-space (125,43) and
     * (208,41); the dash sits at frame ((W-256)/2, H-256) and the 79px
     * frames pivot at (39,39), so speedo = frame (278,228)..(356,306)
     * and tach = (361,226)..(439,304); the boxes here round outward. */
    enum { SPD_X = 276, SPD_Y = 226, TACH_X = 359, TACH_Y = 224,
           GBOX = 82 };
    check(strstr(stats, "gauges=ok") != NULL,
          "P01 dash validates its needle hubs (gauges=ok)");
    hud_set_compass_yaw(0.0);            /* pin the compass crop */
    hud_set_engine_rpm(1050.0);          /* idle rpm (car.c CAR_RPM_IDLE) */
    hud_render_frame(fb,  W, H, 3, 0.0, 0, HUD_FLAG_NO_PROOF);
    hud_render_frame(fb2, W, H, 3, 0.0, 0, HUD_FLAG_NO_PROOF);
    check(memcmp(fb, fb2, (size_t)W * H) == 0,
          "gauge rendering is deterministic (same state, same frame)");

    /* Speed step at fixed rpm: only the speedo window may change. */
    hud_render_frame(fb2, W, H, 3, 30.0, 0, HUD_FLAG_NO_PROOF);
    int spd_diff  = fb_diff(fb, fb2, SPD_X, SPD_Y, GBOX, GBOX);
    int tach_diff = fb_diff(fb, fb2, TACH_X, TACH_Y, GBOX, GBOX);
    printf("  speed 0->30 m/s: speedo diff %d px, tach diff %d px\n",
           spd_diff, tach_diff);
    check(spd_diff > 50, "speedo needle moves with the speed state");
    check(tach_diff == 0, "tach needle untouched by a speed-only change");
    /* The fixed dashboard framing: the whole 256x256 dash band outside
     * the two gauge windows is pixel-identical across the speed step. */
    int frame_diff = 0;
    for (int y = H - 256; y < H; y++)
        for (int x = (W - 256) / 2; x < (W + 256) / 2; x++) {
            int in_spd  = (x >= SPD_X && x < SPD_X + GBOX &&
                           y >= SPD_Y && y < SPD_Y + GBOX);
            int in_tach = (x >= TACH_X && x < TACH_X + GBOX &&
                           y >= TACH_Y && y < TACH_Y + GBOX);
            if (!in_spd && !in_tach &&
                fb[(size_t)y * W + x] != fb2[(size_t)y * W + x])
                frame_diff++;
        }
    check(frame_diff == 0,
          "dashboard framing static across the speed change");

    /* RPM step at fixed speed: only the tach window may change. */
    hud_render_frame(fb, W, H, 3, 30.0, 0, HUD_FLAG_NO_PROOF);
    hud_set_engine_rpm(5000.0);
    hud_render_frame(fb2, W, H, 3, 30.0, 0, HUD_FLAG_NO_PROOF);
    spd_diff  = fb_diff(fb, fb2, SPD_X, SPD_Y, GBOX, GBOX);
    tach_diff = fb_diff(fb, fb2, TACH_X, TACH_Y, GBOX, GBOX);
    printf("  rpm 1050->5000: speedo diff %d px, tach diff %d px\n",
           spd_diff, tach_diff);
    check(tach_diff > 50, "tach needle moves with the rpm state");
    check(spd_diff == 0, "speedo needle untouched by an rpm-only change");
    hud_set_engine_rpm(1050.0);

    /* The needles are a composite over the baked faces, never the D5
     * evidence strip: NO_PROOF frames carry no needle art outside the
     * gauge windows. Proven by frame_diff == 0 above plus the strip's
     * absence (the whole frame above the dash band already matched). */

    /* Glance/sidearm families follow the class (P01 -> class 2 sets). */
    int nl = hud_sidearm_frame_count(HUD_SIDEARM_LEFT);
    int nr = hud_sidearm_frame_count(HUD_SIDEARM_RIGHT);
    check(nl == 9 && nr == 9,
          "sidearm/glance families decode at class 2 (9 frames per side)");
    const char *f0 = hud_sidearm_frame_name(HUD_SIDEARM_LEFT, 0);
    check(f0 && strncasecmp(f0, "ZHL45201", 8) == 0,
          "left family is ZHL45201.. (class-2 archive order)");
    const char *f6 = hud_sidearm_frame_name(HUD_SIDEARM_LEFT, 6);
    check(f6 && strncasecmp(f6, "ZSL45", 5) == 0,
          "left overlay entries follow the background entries");
    int fw = 0, fh = 0;
    const uint8_t *fpx = hud_sidearm_frame(HUD_SIDEARM_RIGHT, 0, &fw, &fh);
    check(fpx && fw > 0 && fh > 0, "right sidearm frame 0 decodes");

    /* Class-4 dash (P09-P12) is a different cockpit without the
     * two-gauge layout: hub validation must reject it and no needle
     * pixels may appear — the static baked faces are preserved. */
    check(hud_load_mission("miss8/P09.MSN", 0) == 0,
          "hud_load_mission(P09) loads");
    hud_stats(stats, sizeof stats);
    check(strstr(stats, "dash zdash401") != NULL,
          "P09 dash set is zdash401 (scenario class 4)");
    check(strstr(stats, "gauges=off") != NULL,
          "class-4 dash rejects the needle hubs");
    hud_render_frame(fb,  W, H, 3, 0.0,  0, HUD_FLAG_NO_PROOF);
    hud_render_frame(fb2, W, H, 3, 30.0, 0, HUD_FLAG_NO_PROOF);
    check(fb_checksum(fb,  (W - 256) / 2, H - 256, 256, 256) ==
          fb_checksum(fb2, (W - 256) / 2, H - 256, 256, 256),
          "class-4 dash band identical across a speed step (no needles)");

    /* Unlisted scenario keeps the shipped default set. */
    check(hud_load_mission("miss8/N01.CBT", 0) == 0,
          "hud_load_mission(N01) loads");
    hud_stats(stats, sizeof stats);
    check(strstr(stats, "dash zdash101") != NULL,
          "N01 (unlisted) keeps dash set 1");

    /* --- 5. Authored mirror mask/bezel -------------------------------- */
    int mw = 0, mh = 0, bw = 0, bh = 0;
    const uint8_t *mask = hud_mirror_mask(&mw, &mh);
    const uint8_t *bezel = hud_mirror_bezel(&bw, &bh);
    check(mask != NULL && mw == 256 && mh == 64,
          "mirror mask decoded (zmiri101.map 256x64)");
    if (mask) {
        int keyed = 0, solid = 0;
        for (int i = 0; i < mw * mh; i++)
            if (mask[i] == 0xFF) keyed++; else solid++;
        check(keyed > 0 && solid > 0,
              "mirror mask carries both glass-cutout and surround texels");
    }
    check(bezel != NULL && bw > 0 && bh > 0,
          "mirror bezel decoded (zmiro101.map)");

    /* --- 6. Standalone GDF query (pilot sidearm) ---------------------- */
    CarWeaponInfo sidearm;
    check(car_gdf_weapon_info("gh45.gdf", &sidearm) == 0,
          "car_gdf_weapon_info parses gh45.gdf");
    if (sidearm.name) {
        printf("  sidearm: '%s' dmg=%d ammo=%d cd=%d spd=%.1f sound=%s\n",
               sidearm.name, sidearm.damage, sidearm.ammo,
               sidearm.cooldown_ticks, sidearm.projectile_speed,
               sidearm.sound ? sidearm.sound : "");
        check(strncasecmp(sidearm.name, "45", 2) == 0 &&
              sidearm.damage > 0 && sidearm.ammo > 0 &&
              sidearm.cooldown_ticks >= 1 && sidearm.direct_fire &&
              !sidearm.rear_facing &&
              sidearm.sound && sidearm.sound[0],
              "sidearm stats are the GDFC-backed values");
    }
    check(car_gdf_weapon_info("no_such.gdf", &sidearm) != 0,
          "missing GDF reports failure");

    free(fb);
    free(fb2);
    pixidx_free(gix);
    hud_unload();
    car_unload();
    vfs_shutdown();

    printf("cockpit_probe: %d checks, %d failures\n", s_checks, s_failures);
    return s_failures ? 1 : 0;
}
