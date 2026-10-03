/* Authored horizon oracle: independent ray/edge intersection against sixteen
 * segments, plus mission replacement, yaw, translation and depth isolation.
 * NITRO_APP must name owned assets. Optional argv[1] is a PPM path prefix.
 * Build: OUT=<dir> tools/build_probe.sh horizon_probe */
#include <math.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "engine/fs.h"
#include "engine/vfs.h"
#include "engine/scene.h"
#include "engine/terrain.h"
#include "engine/worldrender.h"
#include "engine/texcache.h"
#include "engine/hud.h"

#define W 640
#define H 480
#define PI 3.14159265358979323846
static unsigned char off_fb[W*H], on_fb[W*H], translated[W*H];
static unsigned int depth[W*H];
static int fails;
static void check(int ok, const char *label)
{
    printf("%s %s\n", ok ? "PASS" : "FAIL", label);
    if (!ok) fails++;
}

static int expected(const CameraView *c, int x, int y, const RTex *tiles[16])
{
    double d[3];
    for (int k=0;k<3;k++) d[k]=320*c->forward[k]+(x-320)*c->right[k]+(240-y)*c->up[k];
    for (int i=0;i<16;i++) {
        double a=i*PI/8, b=(i+1)*PI/8;
        double ax=6000*sin(a), az=6000*cos(a);
        double ex=6000*sin(b)-ax, ez=6000*cos(b)-az;
        double det=d[0]*ez-d[2]*ex;
        if (fabs(det)<1e-9) continue;
        double t=(ax*ez-az*ex)/det, u=(ax*d[2]-az*d[0])/det;
        if (t<=0 || u<0 || u>=1) continue;
        double v=t*d[1]/(6000*sin(PI/16));
        if (v<0 || v>=1) return 255;
        int tx=(int)floor(1+126*u), ty=(int)floor(128*v);
        return tiles[i]->texels[ty*128+tx];
    }
    return 255;
}
static int write_bytes(const char *path, const void *bytes, size_t n)
{
    /* The sandbox holds symlinks to the purchaser's own files. Never write
     * through one: drop the directory entry (the link, not its target) and
     * create the fixture exclusively. */
    unlink(path);
    int fd=open(path,O_WRONLY|O_CREAT|O_EXCL,0600);
    if (fd<0) return 0;
    FILE *f=fdopen(fd,"wb");
    if (!f) { close(fd); return 0; }
    int ok=fwrite(bytes,1,n,f)==n;
    return fclose(f)==0 && ok;
}
static void loader_cases(const char *root)
{
    size_t n=0; unsigned char *mission=vfs_read_file("miss8/P01.MSN",&n);
    if (!mission) { fails++; return; }
    int patched=0;
    for (size_t i=0;i+8+147<=n;i++) if (!memcmp(mission+i,"WRLD",4)) {
        memset(mission+i+8+134,0,13);
        memcpy(mission+i+8+134,"test.hzd",8); patched=1; break;
    }
    check(patched,"synthetic WRLD horizon field prepared");
    char sandbox[1024];
    snprintf(sandbox,sizeof sandbox,"%s/horizon-XXXXXX",getenv("TMPDIR")?getenv("TMPDIR"):"/tmp");
    if (!mkdtemp(sandbox)) { free(mission); fails++; return; }
    DIR *d=opendir(root); struct dirent *de;
    if (!d) { free(mission); fails++; return; }
    while ((de=readdir(d))) {
        if (de->d_name[0]=='.') continue;
        char src[2048], dst[2048];
        snprintf(src,sizeof src,"%s/%s",root,de->d_name);
        snprintf(dst,sizeof dst,"%s/%s",sandbox,de->d_name);
        if (symlink(src,dst)!=0) fails++;
    }
    closedir(d);
    char msn[2048], hzd[2048];
    snprintf(msn,sizeof msn,"%s/horizon.msn",sandbox);
    snprintf(hzd,sizeof hzd,"%s/test.hzd",sandbox);
    check(write_bytes(msn,mission,n),"synthetic mission written");
    free(mission);
    vfs_shutdown(); fs_set_root(sandbox);
    check(vfs_init(),"sandbox VFS initialized");
    check(scene_load("horizon.msn")==0 && !scene_horizon_tex(0,NULL),"missing optional HZD disables horizon");
    char list[1024];
    size_t used=0;
    for (int i=0;i<16;i++) used+=(size_t)snprintf(list+used,sizeof list-used," ,\tNH_01_%02d.MAP\r\n",i+1);
    check(write_bytes(hzd,list,used),"delimited horizon fixture written");
    check(scene_load("horizon.msn")==0 && scene_horizon_tex(15,NULL),"native whitespace/comma delimiters accepted");
    check(write_bytes(hzd,list,used-22),"truncated horizon fixture written");
    check(scene_load("horizon.msn")==0 && !scene_horizon_tex(0,NULL),"partial list clears all slots after successful mission");
    memcpy(list,"missing.MAP     ",16);
    check(write_bytes(hzd,list,used),"missing tile fixture written");
    check(scene_load("horizon.msn")==0 && !scene_horizon_tex(0,NULL),"unresolvable tile disables whole horizon");
    memset(list,'X',sizeof list);
    check(write_bytes(hzd,list,sizeof list),"oversized token fixture written");
    check(scene_load("horizon.msn")==0 && !scene_horizon_tex(0,NULL),"oversized nonterminated token rejected");
    scene_unload(); vfs_shutdown();
    d=opendir(sandbox);
    if (d) {
        while ((de=readdir(d))) {
            if (de->d_name[0]=='.') continue;
            char path[2048]; snprintf(path,sizeof path,"%s/%s",sandbox,de->d_name); unlink(path);
        }
        closedir(d);
    }
    rmdir(sandbox);
}
static void draw(unsigned char *fb, const CameraView *c, int world)
{
    worldrender_camera(fb,W,H,c,0,0,world,world);
}
static void ppm(const char *prefix, const char *tag, const unsigned char *fb)
{
    if (!prefix) return;
    char path[1024]; snprintf(path,sizeof path,"%s-%s.ppm",prefix,tag);
    FILE *f=fopen(path,"wb");
    if (!f) { fails++; return; }
    fprintf(f,"P6\n%d %d\n255\n",W,H);
    const unsigned char *pal=hud_palette();
    for (int i=0;i<W*H;i++) fwrite(pal+3*fb[i],1,3,f);
    fclose(f); printf("EVIDENCE %s\n",path);
}
int main(int argc, char **argv)
{
    const char *root=getenv("NITRO_APP");
    if (!root) return 2;
    fs_set_root(root); if (!vfs_init()) return 2;
    const char *missions[]={"miss8/P01.MSN","miss8/P02.MSN"};
    const char *tags[]={"p01","p02"};
    worldrender_set_backend(WORLD_BACKEND_FILLED);
    for (int m=0;m<2;m++) {
        check(terrain_load(missions[m])==0 && scene_load(missions[m])==0,"mission loads");
        hud_load_mission(missions[m],0);
        const RTex *tiles[16]; int loaded=1;
        for (int i=0;i<16;i++) loaded &= scene_horizon_tex(i,&tiles[i]);
        check(loaded,"sixteen horizon tiles ready"); if (!loaded) continue;
        int same=1;
        for (int i=0;i<16;i++) {
            char tile[16]; RTex t; unsigned char *p=NULL;
            snprintf(tile,sizeof tile,"NH_%02d_%02d.MAP",m?5:1,m?(i+7)%16+1:i+1);
            if (texcache_load_map(tile,&t,&p)!=0) same=0;
            else if (t.w!=128 || t.h!=128 || memcmp(p,tiles[i]->texels,16384)) same=0;
            free(p);
        }
        check(same,"WRLD selects decoded mission artwork and rotated tile order");
        double eye[]={2407.5,40,49392.5}, target[]={2407.5,40,49492.5};
        if (m) { eye[0]=3687.5; eye[1]=8.4; eye[2]=47472.5; }
        target[1]=eye[1];
        CameraView cam; camera_view_look_at(&cam,eye,target);
        int mismatch=0, opaque=0, clear=0;
        for (int heading=0;heading<16;heading++) {
            double a=(heading+.5)*PI/8;
            target[0]=eye[0]+100*sin(a); target[2]=eye[2]+100*cos(a);
            camera_view_look_at(&cam,eye,target);
            scene_horizon_enable(0); draw(off_fb,&cam,0);
            scene_horizon_enable(1); draw(on_fb,&cam,0);
            for (int y=180;y<255;y+=3) for (int x=3;x<W;x+=7) {
                int e=expected(&cam,x,y,tiles), p=y*W+x;
                if (e==255) { clear++; mismatch+=on_fb[p]!=off_fb[p]; }
                else { opaque++; mismatch+=on_fb[p]!=e; }
            }
        }
        printf("INFO %s oracle opaque=%d clear=%d mismatch=%d\n",tags[m],opaque,clear,mismatch);
        check(opaque>1000 && clear>1000 && mismatch==0,"all headings match independent segment/UV/transparent oracle");
        CameraView moved=cam; moved.eye[0]+=1000; moved.eye[2]-=3000;
        draw(translated,&moved,0);
        check(!memcmp(on_fb,translated,sizeof on_fb),"camera XZ translation leaves panorama fixed in angle");
        ppm(argc>1?argv[1]:NULL,m?"p02-backdrop":"p01-backdrop",on_fb);
        scene_horizon_enable(0); draw(off_fb,&cam,1);
        memcpy(depth,worldrender_depth(),sizeof depth);
        scene_horizon_enable(1); draw(on_fb,&cam,1);
        check(worldrender_geometry_pixels()>10000,"evidence pose contains playable world geometry");
        int changed=0, foreground=0;
        for (int i=0;i<W*H;i++) if (on_fb[i]!=off_fb[i]) { changed++; if (depth[i]) foreground++; }
        printf("INFO %s visible_changed=%d foreground_changed=%d\n",tags[m],changed,foreground);
        check(!memcmp(depth,worldrender_depth(),sizeof depth) && foreground==0,"world depth and foreground pixels unchanged");
        check(changed>100,"mountain pixels visible behind world");
        ppm(argc>1?argv[1]:NULL,tags[m],on_fb);
        scene_unload();
        check(!scene_horizon_tex(0,NULL),"mission teardown clears horizon");
    }
    loader_cases(root);
    printf("RESULT: %s (%d failures)\n",fails?"FAIL":"PASS",fails);
    return fails?1:0;
}
