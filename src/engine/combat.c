/*
 * combat.c — M7 combat/damage model (see combat.h for the contract and
 * the D-C* DECISION list).
 *
 * Per-entity HP with shot/ram/landing damage events, death (10 s burning
 * wreck + sit + logging, D-C27), one-tick event pulses for the FSM
 * predicates, the attack engagement (chase-and-fire over ai.c's mover),
 * a kinematic ram scan, and authored-speed projectile delivery for
 * player and NPC fire. Terrain, static-world, and live-vehicle segment
 * contacts own hit delivery;
 * ghidra-physics.md Q8/Q12/Q18 anchors are cited in combat.h.
 *
 * Compiled by textual inclusion from mission.c (combat.h build note).
 */

#include "engine/combat.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "engine/ai.h"
#include "engine/scene.h"
#include "engine/terrain.h"

/* ----------------------------------------------------------------------- */
/* State                                                                   */
/* ----------------------------------------------------------------------- */

typedef struct {
    int      used;
    int      team;          /* ODEF team; -1 unresolved (inert)        */
    int      class_id;      /* ODEF class (1 = car)                      */
    int      scene_obj;     /* placed scene object for hide, -1          */
    char     label[41];     /* FSM entity label (for logs)               */
    int      hp, hp_max;    /* D-C1/D-C2                                 */
    int      alive;
    int      killed_by;     /* entity that dealt the killing blow, -1    */
    int      kills;         /* entities this one destroyed (D-C13)       */
    int      hidden;        /* D-C8                                      */
    int      has_obj;       /* resolved to an ODEF record                */
    int      has_vcf_config;/* authored car pools/loadout resolved       */
    int      player_defense_x2; /* D-C28: selected single-player VCF scaled */
    /* event pulses (D-C11: one-tick levels, cleared per combat_tick) */
    int      p_attacked;
    int      p_shot;
    int      p_rammed;
    int      who_attacked;  /* attacker entity index, -1 (D11)           */
    int      who_shot;
    int      who_rammed;
    /* attack engagement (D-C5) */
    int      eng_target;    /* -1 = none                                 */
    int      eng_auto;      /* D-C15: target came from the fallback      */
    int      eng_hold;      /* D-C15: script explicitly said attack none */
    int      eng_slot;      /* D-C26: holds a maxAttackers a998 slot     */
    int      eng_cool;      /* ticks until the next engagement shot      */
    CarCombatWeapon weapons[CAR_COMBAT_WEAPONS]; /* authored NPC loadout */
    int      weapon_count;
    int      weapon_cool[CAR_COMBAT_WEAPONS];
    int      burst_left[CAR_COMBAT_WEAPONS];
    double   collision_half[3];       /* authored VDF COLP target bounds */
    /* ram scan and native turret prediction bookkeeping. */
    double   px, py, pz;    /* entity-origin snapshot, previous tick      */
    double   vx, vy, vz;    /* per-tick velocity estimate, m/s            */
    int      have_pos;
    int      ram_cool;      /* ticks until this entity can ram again     */
    int      under_fire_ticks; /* MARKED protected-convoy weave lifetime */
    unsigned under_fire_phase; /* monotone while latched; hits extend only */
    /* H-UAT-003/067a: independent per-component/facet condition pools (the
     * original's live armor/chassis arrays; header enum + notes). Authored car
     * facet ratios drive the scalar compatibility HP; core systems remain
     * panel-only pending native component-death decode. */
    int      comp_hp[COMBAT_COMP_COUNT];
    int      comp_max[COMBAT_COMP_COUNT];
    /* D-C27: native 10.0 s burning-wreck timer (FUN_00464530's +0x454
     * countdown). >0 while the dead car stays visible and rolls secondary
     * events; the scene object hides at expiry. */
    int      wreck_ticks;
} CombatEnt;

typedef struct {
    char name[17];
    int  damage;
    int  ammo;
    int  ammo_max;
    int  cooldown;                      /* GDF cadence in ticks            */
    int  cool;                          /* ticks until this hardpoint can
                                         * fire again (per-weapon)         */
    int  source;                         /* car_weapon_get index          */
    int  rear;
    int  link_class;                     /* manual weapon taxonomy         */
    int  linkable;                       /* fixed weapon; turrets excluded */
    int  armed;                          /* 1 = Space fires this hardpoint */
    int  fired;                          /* fired on latest Space trigger   */
    double pspeed;                       /* ORDF flight speed, m/s          */
    double launch_spread;                /* GDFC +98 launch degrees         */
    double aim_speed;                    /* GDFC +86 convergence operand     */
    int    ordnance_type;                /* ORDF +0 flight dispatcher        */
    int    manager_type;                 /* ORDF +12 legacy NPC selector     */
    int    family, tier;                 /* GDFC pair; (3,3) skips pitch     */
    int    traverses;
    int    deploy_kind;                  /* CAR_DEPLOY_*; no projectile      */
    char   ordnance_model[9];            /* OGEO authored presentation mesh  */
    char   impact_ground[14];             /* ORDF authored XDF prototypes     */
    char   impact_car[14];
    char   impact_building[14];
    char   impact_structure[14];
    double turret_yaw;                   /* relative to parent frame         */
    double turret_pitch;                 /* child-joint elevation            */
    int turret_yaw_on_target;            /* abs remaining error < 2 degrees  */
    int turret_pitch_on_target;          /* abs remaining error < 2 degrees  */
} CombatWeapon;

typedef struct {
    int active;
    int attacker;
    int damage;
    int ordnance_type;
    int weapon_class;                     /* parsed ORDF presentation family */
    int tracking_target;                  /* -1 for decoded dumbfire         */
    int age;
    uint32_t seed;
    double x, y, z;
    double vx, vy, vz;
    double speed;
    double distance_left;
    double distance_flown;
    int trail_count;
    double trail[COMBAT_FX_TRAIL_MAX][3]; /* chronological real sim poses    */
    char impact_ground[14];               /* ORDF target-class XDF prototypes */
    char impact_car[14];
    char impact_building[14];
    char impact_structure[14];
} CombatProjectile;

typedef struct {
    int active;
    int kind;                 /* CAR_DEPLOY_*                         */
    int owner;
    int damage;               /* authored GDFC +44                   */
    int age;
    int lifetime;             /* MARKED per-family port convention   */
    int arm_ticks;
    int owner_clear;          /* own car must leave before re-trigger */
    uint64_t inside_mask;     /* entry-triggered patch bookkeeping    */
    uint32_t seed;
    char ordnance_model[9];    /* copied OGEO name; empty = marked fallback */
    char impact_car[14];       /* ORDF vehicle-contact effect, if authored */
    double x, y, z;
    double radius;
} CombatDeployed;

#define COMBAT_PLAYER_WEAPONS 8

static CombatEnt  s_cents[COMBAT_MAX_ENTS];
static CombatProjectile s_projectiles[COMBAT_PROJECTILE_MAX];
static CombatDeployed s_deployed[COMBAT_DEPLOY_MAX];
static unsigned long s_deploy_count[CAR_DEPLOY_COUNT];
static unsigned long s_deploy_trigger_count[CAR_DEPLOY_COUNT];
static unsigned long s_deploy_obstacle_count;
/* H-UAT-067g/076c: overlapping hazardous regions cannot multiply one
 * target's damage. Fire and proximity-charge families retain independent
 * MARKED 10-tick governors because their authored packets/cadence differ. */
static int s_fire_patch_cool[COMBAT_MAX_ENTS];
static int s_mine_cluster_cool[COMBAT_MAX_ENTS];
static unsigned long s_launch_count[COMBAT_MAX_ENTS];
static unsigned long s_contact_count[COMBAT_MAX_ENTS];
static unsigned long s_world_absorb_count[COMBAT_MAX_ENTS];
static unsigned long s_fx_hit_count[COMBAT_MAX_ENTS];
static int        s_user = -1;
static int        s_user_team = -1;
static void     (*s_resolve)(int ent, double out[3]);

/* player pose + direct-fire loadout (D-C3/D-C4) */
static double       s_ux, s_uy, s_uz, s_uyaw;
static int          s_have_upose;
static CombatWeapon s_weapons[COMBAT_PLAYER_WEAPONS];
static int          s_weapon_count;
static int          s_weapon_selected;
/* Legacy max of per-weapon cools — web presentation / probes only. */
static int          s_fire_cool;

/* Pilot sidearm pool (D-C14): separate from the hardpoint loadout, with
 * its OWN cadence governor — a mounted shot never delays the sidearm and
 * vice versa. */
static CombatWeapon s_sidearm;
static int          s_sidearm_valid;
static int          s_sidearm_cool;

/* landing bookkeeping (D-C7) */
static int        s_land_seen;

/* H-UAT-007: presentation-only event store (header contract). */
static CombatFx   s_fx[COMBAT_FX_EVENT_MAX];
static uint32_t   s_fx_seed;
static uint32_t   s_projectile_seed;
static double     s_last_user_muzzle[3];
static int        s_have_last_user_muzzle;

/* Physical-damage RNG (D-C10): landing/component routing only. */
static uint32_t   s_crng = 0xC0BA47u;

static uint32_t combat_rng(void)
{
    uint32_t x = s_crng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_crng = x;
    return x;
}

/* D-C27 wreck secondary-event rolls (the native death path's rand()&0xf).
 * A separate fixed stream, same law as T_C/T_D: burning-wreck presentation
 * cannot perturb physical-damage or shot-acceptance sequences. */
static uint32_t s_wreck_rng = 0xDEA74E5u;
static uint32_t wreck_rng(void)
{
    uint32_t x = s_wreck_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_wreck_rng = x;
    return x;
}

/* Separate fixed stream for decoded T_C shot acceptance. Keeping it apart
 * from landing/component routing means adding an NPC or changing its cadence
 * cannot perturb physical-damage rolls. */
static uint32_t s_fire_rng = 0x7C0B471u;
/* T_D consumes the native rand()%1000-500 role from a separate fixed stream;
 * no libc rand or accuracy dice enter the deterministic sim. */
static uint32_t s_aim_rng = 0x1A76D00Du;
/* PORT DECISION: isolated repeatable 15-bit draws, not Nitro CRT stream parity. */
static uint32_t s_spread_rng = 0x76A1B00Du;
static uint32_t fire_rng(void)
{
    uint32_t x = s_fire_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_fire_rng = x;
    return x;
}

static CombatEnt *ent_at(int ent)
{
    if (ent < 0 || ent >= COMBAT_MAX_ENTS)
        return NULL;
    return s_cents[ent].used ? &s_cents[ent] : NULL;
}

/* Defined in the H-UAT-003/H-UAT-007 sections below; combat_die and the
 * damage events ahead of them already spawn/route through these. */
static int  fx_weapon_class(int ordnance_type, int damage);
static void fx_kill(const double point[3]);
static int  fx_position(int ent, double out[3]);

static const char *ent_name(int ent)
{
    CombatEnt *e = ent_at(ent);
    if (!e)
        return "?";
    return e->label[0] ? e->label : "?";
}

/* D-C9: nonzero team, different from the reference's. */
static int hostile_to(const CombatEnt *e, int ref_team)
{
    return e->team > 0 && ref_team >= 0 && e->team != ref_team;
}

/* FACT fire-delivery.md §1: native canSee and ordnance flight share the
 * terrain/world collision owner. FUN_004adf90 steps the bilinear height field;
 * rendered terrain triangles are not ordnance collision. Static world remains
 * the already-decoded active scene-part OBBs. Drive-on decals are not
 * occluders; live vehicles are tested by the projectile target pass, not
 * duplicated here. */
static int segment_obb_hit(const double a[3], const double b[3],
                           const double c[3], const double half[2],
                           const double axis[2], const double yspan[2],
                           double *hit_t)
{
    const double ux = axis[0], uz = axis[1], vx = -axis[1], vz = axis[0];
    double p[3] = { (a[0]-c[0])*ux + (a[2]-c[2])*uz,
                    a[1],
                    (a[0]-c[0])*vx + (a[2]-c[2])*vz };
    double m[3] = { (b[0]-a[0])*ux + (b[2]-a[2])*uz,
                    b[1]-a[1],
                    (b[0]-a[0])*vx + (b[2]-a[2])*vz };
    double lo[3] = { -half[0], yspan[0], -half[1] };
    double hi[3] = {  half[0], yspan[1],  half[1] };
    double tn = 0.0, tf = 1.0;
    for (int k = 0; k < 3; k++) {
        if (fabs(m[k]) < 1e-12) {
            if (p[k] < lo[k] || p[k] > hi[k]) return 0;
            continue;
        }
        double t0 = (lo[k] - p[k]) / m[k], t1 = (hi[k] - p[k]) / m[k];
        if (t0 > t1) { double q = t0; t0 = t1; t1 = q; }
        if (t0 > tn) tn = t0;
        if (t1 < tf) tf = t1;
        if (tn > tf) return 0;
    }
    if (tf < 0.0 || tn > 1.0) return 0;
    if (hit_t) *hit_t = tn < 0.0 ? 0.0 : tn;
    return 1;
}

/* A registered, live non-car entity owns its scene geometry for projectile
 * damage. The scene object itself is the bounds handle: target tests query
 * the same live per-part authored mesh OBBs as static-world collision, so a
 * hidden object or triggerGate-inactive part cannot retain a stale box. */
static int scene_obj_damage_target(int scene_obj)
{
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        const CombatEnt *e = &s_cents[i];
        if (e->used && e->alive && !e->hidden && e->has_obj &&
            e->class_id != 1 && e->scene_obj == scene_obj)
            return i;
    }
    return -1;
}

int combat_scene_entity(int scene_obj)
{
    if (scene_obj < 0) return -1;
    for (int i = 0; i < COMBAT_MAX_ENTS; i++)
        if (s_cents[i].used && s_cents[i].has_obj &&
            s_cents[i].scene_obj == scene_obj)
            return i;
    return -1;
}

/* `attribute_targets` is the ordnance ordering switch. Projectile flight
 * excludes live damageable scene objects here because segment_target_hit()
 * owns their identical OBBs and damage attribution; LOS keeps them as world
 * occluders except for the queried target itself. Everything else remains an
 * anonymous absorbing world hit. */
static const char *impact_effect_for_class(int class_id,
                                           const char *ground,
                                           const char *car,
                                           const char *building,
                                           const char *structure)
{
    switch (class_id) {
    case -1: return ground;
    case 1: case 4: case 8: case 9: return car;
    case 2: case 0x0c: return building;
    case 3: case 0x0b: return structure;
    default: return NULL;
    }
}

static int segment_world_hit(const double a[3], const double b[3],
                             int ignore_scene_obj, int attribute_targets,
                             int native_heightfield, double radius,
                             double *hit_t, int *impact_kind, int *hit_obj)
{
    double best = 2.0, t;
    int kind = -1, object = -1;
    int terrain_hit = native_heightfield
        ? terrain_ordnance_segment_hit(a, b, &t)
        : terrain_segment_hit(a, b, &t);
    if (terrain_hit && t < best) best = t;
    for (int o = 0; o < scene_obj_count(); o++) {
        if (o == ignore_scene_obj || scene_obj_hidden(o) ||
            mission_scene_object_is_vehicle(o) ||
            (attribute_targets && scene_obj_damage_target(o) >= 0))
            continue;
        for (int p = 0; p < scene_obj_part_count(o); p++) {
            double c[3], half[2], axis[2], ys[2];
            if (!scene_obj_part_active(o, p) ||
                scene_part_drive_surface(o, p) ||
                scene_obj_part_obb(o, p, c, half, axis, ys) != 0)
                continue;
            half[0] += radius;
            half[1] += radius;
            ys[0] -= radius;
            ys[1] += radius;
            if (segment_obb_hit(a, b, c, half, axis, ys, &t) && t < best) {
                best = t;
                kind = scene_obj_class_id(o);
                object = o;
            }
        }
    }
    if (best > 1.0) return 0;
    if (hit_t) *hit_t = best;
    if (impact_kind) *impact_kind = kind;
    if (hit_obj) *hit_obj = object;
    return 1;
}

int combat_can_see(int attacker, int target)
{
    double a[3], b[3], t;
    CombatEnt *te = ent_at(target);
    int target_obj = te ? te->scene_obj : -1;
    if (fx_position(attacker, a) != 0 || fx_position(target, b) != 0)
        return 0;
    a[1] += 1.0;
    b[1] += 1.0;
    return !segment_world_hit(a, b, target_obj, 0, 1, 0.0, &t, NULL, NULL);
}

static double wrap_angle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

static void rotate_axis(double v[3], const double axis[3], double angle)
{
    double c = cos(angle), s = sin(angle);
    double cross[3] = {
        axis[1] * v[2] - axis[2] * v[1],
        axis[2] * v[0] - axis[0] * v[2],
        axis[0] * v[1] - axis[1] * v[0]
    };
    double dot = axis[0] * v[0] + axis[1] * v[1] + axis[2] * v[2];
    for (int i = 0; i < 3; i++)
        v[i] = v[i] * c + cross[i] * s + axis[i] * dot * (1.0 - c);
}

/* Apply FUN_004af210's yaw joint followed by its child pitch joint to an
 * already-composed HLOC/GPOF frame. Frames are r/u/f/position columns.
 * Positive pitch raises the forward axis, hence the negative right-axis
 * Rodrigues angle under this basis convention. */
static void turret_live_frame(const double neutral[12], const double mount[12],
                              double yaw, double pitch, double out[12])
{
    memcpy(out, neutral, 12 * sizeof *out);
    double yaw_axis[3] = { mount[3], mount[4], mount[5] };
    double yn = sqrt(yaw_axis[0] * yaw_axis[0] +
                     yaw_axis[1] * yaw_axis[1] +
                     yaw_axis[2] * yaw_axis[2]);
    if (yn < 1e-9) {
        yaw_axis[0] = 0.0; yaw_axis[1] = 1.0; yaw_axis[2] = 0.0;
    } else for (int i = 0; i < 3; i++) yaw_axis[i] /= yn;

    for (int col = 0; col < 3; col++)
        rotate_axis(out + col * 3, yaw_axis, yaw);
    double p[3] = { out[9] - mount[9], out[10] - mount[10],
                    out[11] - mount[11] };
    rotate_axis(p, yaw_axis, yaw);
    for (int i = 0; i < 3; i++) out[9 + i] = mount[9 + i] + p[i];

    double pitch_axis[3] = { out[0], out[1], out[2] };
    double pn = sqrt(pitch_axis[0] * pitch_axis[0] +
                     pitch_axis[1] * pitch_axis[1] +
                     pitch_axis[2] * pitch_axis[2]);
    if (pn < 1e-9) return;
    for (int i = 0; i < 3; i++) pitch_axis[i] /= pn;
    for (int col = 0; col < 3; col++)
        rotate_axis(out + col * 3, pitch_axis, -pitch);
    p[0] = out[9] - mount[9]; p[1] = out[10] - mount[10];
    p[2] = out[11] - mount[11];
    rotate_axis(p, pitch_axis, -pitch);
    for (int i = 0; i < 3; i++) out[9 + i] = mount[9 + i] + p[i];
}

static void player_frame_world(const double local[12], double out[12])
{
    double c = cos(s_uyaw), s = sin(s_uyaw);
    for (int col = 0; col < 3; col++) {
        int k = col * 3;
        out[k] = local[k] * c - local[k + 2] * s;
        out[k + 1] = local[k + 1];
        out[k + 2] = local[k] * s + local[k + 2] * c;
    }
    out[9] = s_ux + local[9] * c - local[11] * s;
    out[10] = s_uy + local[10];
    out[11] = s_uz + local[9] * s + local[11] * c;
}

static void npc_frame_world(const double local[12], const CombatEnt *owner,
                            double hull_fdx, double hull_fdz, double out[12])
{
    double rx = hull_fdz, rz = -hull_fdx;
    for (int col = 0; col < 3; col++) {
        int k = col * 3;
        out[k] = local[k] * rx + local[k + 2] * hull_fdx;
        out[k + 1] = local[k + 1];
        out[k + 2] = local[k] * rz + local[k + 2] * hull_fdz;
    }
    out[9] = owner->px + local[9] * rx + local[11] * hull_fdx;
    out[10] = owner->py + local[10];
    out[11] = owner->pz + local[9] * rz + local[11] * hull_fdz;
}

static double ammo_fraction(int ammo, int capacity)
{
    if (ammo < 0 || capacity <= 0) return 1.0;
    return (double)ammo / (double)capacity;
}

/* FACT aim-convergence.md §2: predict the selected target ENTITY ORIGIN,
 * clamp lead time to 0..5 s, then admit pitch only inside the parent-local
 * 90-degree yaw hemisphere. No COLP/roof-height term participates. */
static void turret_converge(int owner_ent, int target_ent,
                            const double muzzle_world[12],
                            const double neutral_world[12],
                            const double mount_world[12],
                            double aim_speed, double ammo_frac,
                            int family, int tier,
                            double *yaw, double *pitch,
                            int *yaw_on_target, int *pitch_on_target)
{
    CombatEnt *owner = ent_at(owner_ent), *target = ent_at(target_ent);
    double pt[3];
    if (!owner || !target || !yaw || !pitch ||
        fx_position(target_ent, pt) != 0)
        return;

    double speed = aim_speed > 0.0 ? aim_speed : COMBAT_FX_SPEED_DEFAULT;
    double vp[3] = { owner->vx + muzzle_world[6] * speed,
                     owner->vy + muzzle_world[7] * speed,
                     owner->vz + muzzle_world[8] * speed };
    double rv[3] = { vp[0] - target->vx, vp[1] - target->vy,
                     vp[2] - target->vz };
    double delta[3] = { pt[0] - muzzle_world[9],
                        pt[1] - muzzle_world[10],
                        pt[2] - muzzle_world[11] };
    double denom = rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2];
    double lead = denom < 0.01 ? 10000000.0
                : (rv[0] * delta[0] + rv[1] * delta[1] +
                   rv[2] * delta[2]) / denom;
    if (lead < 0.0) lead = 0.0;
    if (lead > 5.0) lead = 5.0;
    /* PORT DECISION: linear prediction also stands in for the undecoded
     * high-angular-rate branch of FUN_004012a0 (aim-convergence.md §2.1). */
    double q[3] = { pt[0] + (lead + 0.09) * target->vx,
                    pt[1] + (lead + 0.09) * target->vy,
                    pt[2] + (lead + 0.09) * target->vz };

    /* FUN_004af210 solves in the yaw joint's parent frame, not in world
     * heading/elevation. Preserve authored HLOC orientation by projecting
     * both the target vector and neutral child forward onto that basis. */
    double j[3] = { q[0] - mount_world[9], q[1] - mount_world[10],
                    q[2] - mount_world[11] };
    double local[3], neutral_local[3];
    for (int col = 0; col < 3; col++) {
        local[col] = j[0] * mount_world[col * 3] +
                     j[1] * mount_world[col * 3 + 1] +
                     j[2] * mount_world[col * 3 + 2];
        neutral_local[col] = neutral_world[6] * mount_world[col * 3] +
                             neutral_world[7] * mount_world[col * 3 + 1] +
                             neutral_world[8] * mount_world[col * 3 + 2];
    }
    double yaw_parent = local[0] * local[0] + local[2] * local[2] > 0.001
                      ? atan2(local[0], local[2]) : 0.0;
    double neutral_heading = atan2(neutral_local[0], neutral_local[2]);
    double yaw_joint_wanted = wrap_angle(yaw_parent - neutral_heading);
    double n = sqrt(local[0] * local[0] + local[1] * local[1] +
                    local[2] * local[2]);
    double nn = sqrt(neutral_local[0] * neutral_local[0] +
                     neutral_local[1] * neutral_local[1] +
                     neutral_local[2] * neutral_local[2]);
    double pitch_wanted = n > 1e-9
        ? asin(fmax(-1.0, fmin(1.0, local[1] / n))) : 0.0;
    double neutral_pitch = nn > 1e-9
        ? asin(fmax(-1.0, fmin(1.0, neutral_local[1] / nn))) : 0.0;
    /* turret_pitch is the child joint's delta from its authored neutral
     * GPOF frame; mortar/rocket frames may already carry elevation. */
    double pitch_joint_wanted = pitch_wanted - neutral_pitch;
    double max_step = AI_TICK_DT * (0.75 + 0.25 * ammo_frac) *
                      (owner->class_id == 9 ? 6.8 : 1.7);
    double ye = wrap_angle(yaw_joint_wanted - *yaw);
    double ys = fmax(-max_step, fmin(max_step, ye));
    *yaw += ys;

    int pitch_applicable = !(family == 3 && tier == 3) &&
                           fabs(yaw_parent) < M_PI_2;
    if (pitch_applicable) {
        double pe = pitch_joint_wanted - *pitch;
        double ps = fmax(-max_step, fmin(max_step, pe));
        if (fabs(sin(ps)) < 0.95)
            *pitch += ps;
    }
    if (yaw_on_target)
        *yaw_on_target = fabs(wrap_angle(yaw_joint_wanted - *yaw)) <
                         0.0349065844;
    if (pitch_on_target)
        *pitch_on_target = !pitch_applicable ||
            fabs(pitch_joint_wanted - *pitch) < 0.0349065844;
}

/* FACT aim-convergence.md §2.3: with no radar target, FUN_004af210
 * advances both live joints back toward the authored neutral frame using the
 * same ammo-scaled angular budget as convergence. */
static void turret_relax(int owner_class, double ammo_frac,
                         double *yaw, double *pitch,
                         int *yaw_on_target, int *pitch_on_target)
{
    if (!yaw || !pitch)
        return;
    double max_step = AI_TICK_DT * (0.75 + 0.25 * ammo_frac) *
                      (owner_class == 9 ? 6.8 : 1.7);
    double ye = wrap_angle(-*yaw);
    if (fabs(ye) <= max_step)
        *yaw = 0.0;
    else
        *yaw += ye < 0.0 ? -max_step : max_step;
    double pe = -*pitch;
    if (fabs(pe) <= max_step)
        *pitch = 0.0;
    else
        *pitch += pe < 0.0 ? -max_step : max_step;
    if (yaw_on_target)
        *yaw_on_target = fabs(*yaw) < 0.0349065844;
    if (pitch_on_target)
        *pitch_on_target = fabs(*pitch) < 0.0349065844;
}

static CombatWeapon *selected_weapon(void)
{
    if (s_weapon_selected < 0 || s_weapon_selected >= s_weapon_count)
        return NULL;
    return &s_weapons[s_weapon_selected];
}

/* A normal selection is one highlighted hardpoint. Linking may expand it
 * only to same-class, same-facing fixed mounts (manual pp.30/31). */
static void arm_slot(int slot)
{
    for (int i = 0; i < s_weapon_count; i++)
        s_weapons[i].armed = i == slot;
}

static int link_compatible(const CombatWeapon *a, const CombatWeapon *b)
{
    return a && b && a->linkable && b->linkable && a->rear == b->rear &&
           a->link_class > 0 && a->link_class == b->link_class;
}

/* Sync presentation governor to the max remaining per-weapon cool. */
static void refresh_fire_cool(void)
{
    int m = 0;
    for (int i = 0; i < s_weapon_count; i++)
        if (s_weapons[i].cool > m)
            m = s_weapons[i].cool;
    s_fire_cool = m;
}

/* ----------------------------------------------------------------------- */
/* Registration / lifecycle                                                */
/* ----------------------------------------------------------------------- */

void combat_reset(void)
{
    memset(s_cents, 0, sizeof s_cents);
    memset(s_projectiles, 0, sizeof s_projectiles);
    memset(s_deployed, 0, sizeof s_deployed);
    memset(s_deploy_count, 0, sizeof s_deploy_count);
    memset(s_deploy_trigger_count, 0, sizeof s_deploy_trigger_count);
    s_deploy_obstacle_count = 0;
    memset(s_fire_patch_cool, 0, sizeof s_fire_patch_cool);
    memset(s_mine_cluster_cool, 0, sizeof s_mine_cluster_cool);
    memset(s_launch_count,0,sizeof s_launch_count);
    memset(s_contact_count,0,sizeof s_contact_count);
    memset(s_world_absorb_count,0,sizeof s_world_absorb_count);
    memset(s_fx_hit_count,0,sizeof s_fx_hit_count);
    s_have_last_user_muzzle=0;
    s_user = -1;
    s_user_team = -1;
    s_resolve = NULL;
    s_have_upose = 0;
    memset(s_weapons, 0, sizeof s_weapons);
    snprintf(s_weapons[0].name, sizeof s_weapons[0].name, "Weapon");
    s_weapons[0].damage = COMBAT_FIRE_DMG_DFLT;
    s_weapons[0].ammo = -1;
    s_weapons[0].cooldown = COMBAT_FIRE_COOLDOWN;
    s_weapons[0].source = -1;
    s_weapons[0].link_class = 1;
    s_weapons[0].linkable = 1;
    s_weapons[0].armed = 1;
    s_weapon_count = 1;
    s_weapon_selected = 0;
    s_fire_cool = 0;
    s_sidearm_valid = 0;
    s_sidearm_cool = 0;
    memset(&s_sidearm, 0, sizeof s_sidearm);
    s_land_seen = 0;
    s_crng = 0xC0BA47u;
    s_fire_rng = 0x7C0B471u;
    s_aim_rng = 0x1A76D00Du;
    s_spread_rng = 0x76A1B00Du;
    s_wreck_rng = 0xDEA74E5u;
    memset(s_fx, 0, sizeof s_fx);
    s_fx_seed = 0;
    s_projectile_seed = 0;
}

void combat_register(int ent, int team, int class_id, int scene_obj,
                     const char *label, const CarCombatConfig *config)
{
    CombatEnt *e;
    if (ent < 0 || ent >= COMBAT_MAX_ENTS)
        return;
    e = &s_cents[ent];
    memset(e, 0, sizeof *e);
    e->used      = 1;
    e->team      = team;
    e->class_id  = class_id;
    ai_set_class(ent, class_id);
    e->scene_obj = scene_obj;
    e->has_obj   = team >= 0;
    snprintf(e->label, sizeof e->label, "%s", label ? label : "?");
    e->hp = e->hp_max = COMBAT_DEFAULT_HP;
    if (class_id != 1 && class_id != 9) {
        int hp = scene_obj_hp(scene_obj, 1);
        if (hp >= 0)
            e->hp = e->hp_max = hp;
    }
    e->alive     = 1;
    e->killed_by = -1;
    e->eng_target = -1;
    e->eng_slot = 0;
    e->who_attacked = e->who_shot = e->who_rammed = -1;
    /* Authored VCF side facets are the real combat scale. Native
     * FUN_00417ac0 aggregates by the weakest armor/chassis/component ratio,
     * never by summing unrelated sides. hp_max retains the weakest positive
     * authored side as the scalar unit scale; sync_car_facet_hp later derives
     * live HP from actual impacted-pool ratios. Core/tire maxima are not in
     * VCF and remain the bounded 100 fallback. */
    for (int c = 0; c < COMBAT_COMP_COUNT; c++)
        e->comp_hp[c] = e->comp_max[c] = COMBAT_DEFAULT_HP;
    if (config)
        memcpy(e->collision_half, config->collision_half,
               sizeof e->collision_half);
    /* Class-9 mission helicopters use the VDF COLP above for authored hit
     * geometry, but their full VCFC damage/loadout driver remains a separate
     * class-specific contract. Do not run it through the class-1 facet and
     * weapon owner merely because the shared parser can decode the bytes. */
    if (class_id == 1 && config) {
        e->has_vcf_config = 1;
        uint32_t weakest = UINT32_MAX;
        static const int aidx[4] = {
            COMBAT_COMP_ARMOR_F, COMBAT_COMP_ARMOR_L,
            COMBAT_COMP_ARMOR_R, COMBAT_COMP_ARMOR_B
        };
        static const int cidx[4] = {
            COMBAT_COMP_CHASSIS_F, COMBAT_COMP_CHASSIS_L,
            COMBAT_COMP_CHASSIS_R, COMBAT_COMP_CHASSIS_B
        };
        for (int i = 0; i < 4; i++) {
            uint32_t av = config->armor[i], cv = config->chassis[i];
            if (av > (uint32_t)INT_MAX) av = INT_MAX;
            if (cv > (uint32_t)INT_MAX) cv = INT_MAX;
            e->comp_hp[aidx[i]] = e->comp_max[aidx[i]] = (int)av;
            e->comp_hp[cidx[i]] = e->comp_max[cidx[i]] = (int)cv;
            if (av > 0 && av < weakest) weakest = av;
            if (cv > 0 && cv < weakest) weakest = cv;
        }
        if (weakest != UINT32_MAX)
            e->hp = e->hp_max = (int)weakest;
        e->weapon_count = config->weapon_count;
        if (e->weapon_count > CAR_COMBAT_WEAPONS)
            e->weapon_count = CAR_COMBAT_WEAPONS;
        /* Whole-record copy includes ORDF impact_ground/car/building/
         * structure names; npc_fire passes those exact fields onward. */
        memcpy(e->weapons, config->weapons,
               (size_t)e->weapon_count * sizeof e->weapons[0]);
    }
}

void combat_set_user(int ent)
{
    s_user = ent;
    s_user_team = -1;
    CombatEnt *e = ent_at(ent);
    if (e)
        s_user_team = e->team;
}

static int player_pool_x2(int value)
{
    /* Shipped authored pools are small positive integers. Keep malformed or
     * synthetic probe values deterministic without signed-overflow UB. */
    if (value > INT_MAX / 2)
        return INT_MAX;
    if (value < INT_MIN / 2)
        return INT_MIN;
    return value * 2;
}

void combat_apply_singleplayer_player_defense(void)
{
    CombatEnt *e = ent_at(s_user);
    if (!e || e->player_defense_x2)
        return;

    /* N-DECODED D-C28: FUN_004622a0:004627c8..0046284f applies the
     * non-multiplayer player-only 2x initialization after FUN_004b8400 /
     * FUN_004b8660 loaded the shell-selected VCF. Scale both sides of every
     * represented authored armor/chassis live/max pair and the scalar
     * compatibility view. Core/tire state is the port's 100-point fallback,
     * not a decoded VCF pool, so this rule cannot silently tune it. */
    e->hp = player_pool_x2(e->hp);
    e->hp_max = player_pool_x2(e->hp_max);
    for (int c = COMBAT_COMP_ARMOR_F; c <= COMBAT_COMP_CHASSIS_B; c++) {
        e->comp_hp[c] = player_pool_x2(e->comp_hp[c]);
        e->comp_max[c] = player_pool_x2(e->comp_max[c]);
    }
    e->player_defense_x2 = 1;
}

void combat_set_resolver(void (*resolve)(int ent, double out[3]))
{
    s_resolve = resolve;
}

void combat_set_user_pose(double x, double y, double z, double yaw)
{
    s_ux = x;
    s_uy = y;
    s_uz = z;
    s_uyaw = yaw;
    s_have_upose = 1;
}

void combat_player_weapons_clear(void)
{
    memset(s_weapons, 0, sizeof s_weapons);
    s_weapon_count = 0;
    s_weapon_selected = -1;
}

int combat_player_weapon_add(const char *name, int damage, int ammo,
                             int cooldown_ticks, int source, int rear,
                             int link_class, int linkable)
{
    CarWeaponInfo wi;
    int have_source = source >= 0 && car_weapon_get(source, &wi) == 0;
    int deploy_kind = have_source ? wi.deploy_kind : CAR_DEPLOY_NONE;
    if ((damage <= 0 && deploy_kind == CAR_DEPLOY_NONE) ||
        s_weapon_count >= COMBAT_PLAYER_WEAPONS)
        return -1;
    int slot = s_weapon_count++;
    CombatWeapon *w = &s_weapons[slot];
    snprintf(w->name, sizeof w->name, "%s",
             name && *name ? name : "Weapon");
    w->damage = damage;
    w->ammo = ammo;
    w->ammo_max = ammo > 0 ? ammo : 0;
    w->cooldown = cooldown_ticks > 0 ? cooldown_ticks
                                     : COMBAT_FIRE_COOLDOWN;
    w->cool = 0;
    w->source = source;
    w->rear = rear != 0;
    if (have_source) {
        w->pspeed = wi.flight_speed;
        w->aim_speed = wi.projectile_speed;
        w->launch_spread = wi.launch_spread;
        w->ordnance_type = wi.ordnance_type;
        w->manager_type = wi.manager_type;
        w->family = wi.family;
        w->tier = wi.tier;
        w->traverses = wi.traverses;
        w->deploy_kind = wi.deploy_kind;
        snprintf(w->ordnance_model, sizeof w->ordnance_model, "%s",
                 wi.ordnance_model);
        snprintf(w->impact_ground, sizeof w->impact_ground, "%s",
                 wi.impact_ground);
        snprintf(w->impact_car, sizeof w->impact_car, "%s", wi.impact_car);
        snprintf(w->impact_building, sizeof w->impact_building, "%s",
                 wi.impact_building);
        snprintf(w->impact_structure, sizeof w->impact_structure, "%s",
                 wi.impact_structure);
    } else {
        /* Source-less fire is the handgun/probe seam; Nitro's handgun loader
         * FUN_004569b0 registers this exact bullet impact table directly. */
        snprintf(w->impact_ground, sizeof w->impact_ground, "xbulg1.xdf");
        snprintf(w->impact_car, sizeof w->impact_car, "xbulc1.xdf");
        snprintf(w->impact_building, sizeof w->impact_building,
                 "xbulb1.xdf");
        snprintf(w->impact_structure, sizeof w->impact_structure,
                 "xbulc1.xdf");
    }
    w->link_class = link_class;
    w->linkable = linkable != 0;
    /* Loading never silently links mounts. L is the authored link action.
     * The hash-pinned native P01 drive_forward capture issues no Enter/L and
     * starts on its final WEPN record (front 20mm), not source-0 Landmines.
     * The native selection-init write remains the H-UAT-062 demand row, so
     * this is the narrow observable PORT DECISION: archive-order hosts keep
     * the last non-dropper as the initial owner. An all-dropper rack retains
     * its first valid slot rather than becoming unarmed. */
    if (s_weapon_selected < 0 || deploy_kind == CAR_DEPLOY_NONE) {
        s_weapon_selected = slot;
        arm_slot(slot);
    }
    return slot;
}

int combat_player_weapon_select(int slot)
{
    if (slot < 0 || slot >= s_weapon_count)
        return -1;
    s_weapon_selected = slot;
    arm_slot(slot);
    return 0;
}

int combat_player_weapon_cycle(int delta)
{
    if (s_weapon_count <= 0)
        return -1;
    if (s_weapon_selected < 0)
        s_weapon_selected = 0;
    int n = s_weapon_count;
    int step = delta >= 0 ? 1 : -1;
    s_weapon_selected = (s_weapon_selected + step + n) % n;
    /* Manual p.29: Space uses the highlighted weapon. Cycling therefore
     * replaces any old linked set; L can link the new selection afterward. */
    arm_slot(s_weapon_selected);
    return 0;
}

int combat_player_weapon_link(void)
{
    if (s_weapon_selected < 0 || s_weapon_selected >= s_weapon_count)
        return -1;
    CombatWeapon *selected = &s_weapons[s_weapon_selected];
    int peers = 0;
    int all_armed = 1;
    for (int i = 0; i < s_weapon_count; i++)
        if (link_compatible(selected, &s_weapons[i])) {
            peers++;
            if (!s_weapons[i].armed)
                all_armed = 0;
        }

    /* Turrets and singleton classes cannot link. A repeated L on a linked
     * same-class set returns to the highlighted hardpoint. */
    if (peers < 2 || all_armed) {
        arm_slot(s_weapon_selected);
    } else {
        for (int i = 0; i < s_weapon_count; i++)
            s_weapons[i].armed =
                link_compatible(selected, &s_weapons[i]) ? 1 : 0;
    }
    return 0;
}

int combat_player_weapon_armed(int slot)
{
    if (slot < 0 || slot >= s_weapon_count)
        return 0;
    return s_weapons[slot].armed ? 1 : 0;
}

int combat_player_weapon_fired(int slot)
{
    if (slot < 0 || slot >= s_weapon_count)
        return 0;
    return s_weapons[slot].fired ? 1 : 0;
}

/* ----------------------------------------------------------------------- */
/* Pilot sidearm (D-C14)                                                   */
/* ----------------------------------------------------------------------- */

void combat_player_sidearm_clear(void)
{
    s_sidearm_valid = 0;
    s_sidearm_cool = 0;
    memset(&s_sidearm, 0, sizeof s_sidearm);
}

int combat_player_sidearm_set(const char *name, int damage, int ammo,
                              int cooldown_ticks)
{
    if (damage <= 0)
        return -1;
    snprintf(s_sidearm.name, sizeof s_sidearm.name, "%s",
             name && *name ? name : "Sidearm");
    s_sidearm.damage = damage;
    s_sidearm.ammo = ammo;
    s_sidearm.ammo_max = ammo > 0 ? ammo : 0;
    s_sidearm.cooldown = cooldown_ticks > 0 ? cooldown_ticks
                                            : COMBAT_FIRE_COOLDOWN;
    s_sidearm.source = -1;
    s_sidearm.rear = 0;
    /* Native handgun FUN_004569b0 registers the ordinary bullet contact
     * table directly rather than through a mounted GDF. */
    s_sidearm.ordnance_type = 0x12;
    snprintf(s_sidearm.impact_ground, sizeof s_sidearm.impact_ground,
             "xbulg1.xdf");
    snprintf(s_sidearm.impact_car, sizeof s_sidearm.impact_car,
             "xbulc1.xdf");
    snprintf(s_sidearm.impact_building, sizeof s_sidearm.impact_building,
             "xbulb1.xdf");
    snprintf(s_sidearm.impact_structure, sizeof s_sidearm.impact_structure,
             "xbulc1.xdf");
    s_sidearm_valid = 1;
    s_sidearm_cool = 0;             /* a freshly armed sidearm can fire */
    return 0;
}

int combat_player_sidearm_present(void)
{
    return s_sidearm_valid;
}

const char *combat_sidearm_name(void)
{
    return s_sidearm_valid ? s_sidearm.name : "";
}

int combat_sidearm_ammo_left(void)
{
    return s_sidearm_valid ? s_sidearm.ammo : 0;
}

int combat_sidearm_cooldown(void)
{
    return s_sidearm_cool;
}

/* ----------------------------------------------------------------------- */
/* Death                                                                   */
/* ----------------------------------------------------------------------- */

static void combat_die(int ent, int killer, const char *how, int spawn_fx)
{
    CombatEnt *e = ent_at(ent);
    if (!e || !e->alive)
        return;
    e->alive = 0;
    e->hp = 0;
    e->killed_by = killer;      /* isGroovesFault's latched attribution */
    /* D-C13: credit the kill. Self-inflicted damage and the environment
     * (killer -1, e.g. a landing) score for no one — the melee controller
     * counts the player's own kills and nothing else, so an unattributed
     * death must not advance anybody's total. */
    if (killer >= 0 && killer != ent) {
        CombatEnt *k = ent_at(killer);
        if (k)
            k->kills++;
    }
    fprintf(stdout, "[combat] %s (%d) destroyed by %s (%d) — %s\n",
            ent_name(ent), ent,
            killer >= 0 ? ent_name(killer) : "environment", killer, how);
    /* D-C27 (supersedes D-C8's instant hide): a dead car stops and its
     * gameplay state latches hidden, but the hull stays VISIBLE as a
     * burning wreck for the native 10.0 s (FUN_00465370 arms +0x454 =
     * 10.0f; FUN_00464530 counts it down and hides at expiry). The
     * per-tick burn roll is NPC-only in the native (flags & 0x4000 =
     * player skips it; player-death visuals are an open decode, spec
     * §6.1), so the player keeps the old instant path. Non-car entities
     * also keep it — structure wreck swaps (FUN_004610b0) are a separate
     * unported lane. The wreck deals no damage (native has no wreck
     * damage call; H-UAT-077 is native-consistent). */
    ai_sit(ent);
    if (e->class_id == 1 && ent != s_user)
        e->wreck_ticks = COMBAT_WRECK_BURN_TICKS;
    else if (e->scene_obj >= 0)
        scene_obj_set_hidden(e->scene_obj, 1);
    e->hidden = 1;
    e->eng_target = -1;
    e->eng_slot = 0;
    /* Non-projectile deaths still anchor the same deterministic wreck
     * sequence. Projectile shots suppress this generic record and publish
     * the authored vehicle-kill sequence at their exact contact point.
     * D-C27: the record anchors at the entity position — the native
     * spawns X1_CARX1 at ent+0x40/48/50 with NO centre offset, so the
     * port's +1.0 "entity-centre approximation" is gone. */
    if (spawn_fx) {
        double p[3];
        if (fx_position(ent, p) == 0)
            fx_kill(p);
    }
    /* engagements aimed at the dead entity dissolve (the scripts
     * re-issue attack on their own polls; the D-C15 acquisition pass
     * re-acquires deterministically for fielded hostiles) */
    for (int i = 0; i < COMBAT_MAX_ENTS; i++)
        if (s_cents[i].used && s_cents[i].eng_target == ent) {
            s_cents[i].eng_target = -1;
            s_cents[i].eng_auto = 0;
            s_cents[i].eng_slot = 0;
        }
}

static void apply_damage(int target, int dmg, int attacker, const char *how,
                         int spawn_death_fx)
{
    CombatEnt *e = ent_at(target);
    if (!e || !e->alive || dmg <= 0 || e->hp_max == 0)
        return;
    int before = e->hp;
    e->hp -= dmg;
    if (e->hp < 0)
        e->hp = 0;      /* D-C6: clamp — the Q12 underflow is not a feature */
    fprintf(stdout, "[combat] %s hp %d->%d (%s by %s)\n", ent_name(target),
            before, e->hp, how,
            attacker >= 0 ? ent_name(attacker) : "environment");
    if (e->hp <= 0)
        combat_die(target, attacker, how, spawn_death_fx);
}

/* ----------------------------------------------------------------------- */
/* H-UAT-003: per-component condition routing                               */
/* ----------------------------------------------------------------------- */

/* Impact quadrant of an event at `target` coming from world (ax, az),
 * in the target's own frame: 0 = front, 1 = right, 2 = left, 3 = back.
 * The original picks the facet by impact-direction cosines (header
 * anchors); these are 45-degree quadrants. The user uses the live car pose;
 * mission cars use their persistent AI heading so sustained pursuit keeps
 * striking the same vehicle-relative facet instead of rotating with world
 * axes. Unresolved/non-car entities retain world alignment. */
static int impact_quadrant(int target, double ax, double az)
{
    double tx = 0.0, tz = 0.0, tyaw = 0.0;
    if (target == s_user && s_have_upose) {
        tx = s_ux;
        tz = s_uz;
        tyaw = s_uyaw;
    } else {
        CombatEnt *e = ent_at(target);
        if (!e || !e->have_pos)
            return 0;               /* unresolved: front, determinism first */
        tx = e->px;
        tz = e->pz;
        if (e->class_id == 1)
            tyaw = ai_get_heading(target);
    }
    /* mission.c bearing convention: atan2(-dx, dz), 0 = ahead, + = left. */
    double b = atan2(-(ax - tx), az - tz) - tyaw;
    while (b >  3.14159265358979323846) b -= 2.0 * 3.14159265358979323846;
    while (b < -3.14159265358979323846) b += 2.0 * 3.14159265358979323846;
    if (b >  2.3561944901923449 || b < -2.3561944901923449) return 3;
    if (b >  0.7853981633974483) return 2;  /* left  */
    if (b < -0.7853981633974483) return 1;  /* right */
    return 0;                               /* front */
}

static const int s_quad_armor[4] = {
    COMBAT_COMP_ARMOR_F, COMBAT_COMP_ARMOR_R,
    COMBAT_COMP_ARMOR_L, COMBAT_COMP_ARMOR_B
};
static const int s_quad_chassis[4] = {
    COMBAT_COMP_CHASSIS_F, COMBAT_COMP_CHASSIS_R,
    COMBAT_COMP_CHASSIS_L, COMBAT_COMP_CHASSIS_B
};

/* Deterministic surviving-core pick for facet underflow (the original
 * rolls a random surviving system; combat_rng is this module's seeded
 * stream, so the sequence is fixed per session). */
static int pick_surviving_core(CombatEnt *e)
{
    /* The decoded component spill may select vehicle/engine/brakes/
     * suspension; this port has no separate authored vehicle-core maximum,
     * so the three represented systems are the exact bounded candidate set. */
    static const int cores[3] = {
        COMBAT_COMP_ENGINE, COMBAT_COMP_SUSPENSION, COMBAT_COMP_BRAKES
    };
    int start = (int)(combat_rng() % 3);
    for (int k = 0; k < 3; k++) {
        int c = cores[(start + k) % 3];
        if (e->comp_hp[c] > 0)
            return c;
    }
    return -1;
}

/* Apply condition damage to one component pool; facet underflow spills
 * the remainder into a surviving core system (Open76 Car.cs damage flow,
 * deterministic here). The caller synchronizes scalar facet condition. */
static void component_spill(CombatEnt *e, int spill)
{
    if (spill <= 0)
        return;
    int core = pick_surviving_core(e);
    if (core >= 0) {
        e->comp_hp[core] -= spill;
        if (e->comp_hp[core] < 0)
            e->comp_hp[core] = 0;
    }
}

static void component_damage(int target, int comp, int dmg)
{
    CombatEnt *e = ent_at(target);
    if (!e || comp < 0 || comp >= COMBAT_COMP_COUNT || dmg <= 0)
        return;
    int spill = 0;
    e->comp_hp[comp] -= dmg;
    if (e->comp_hp[comp] < 0) {
        spill = -e->comp_hp[comp];
        e->comp_hp[comp] = 0;
    }
    component_spill(e, spill);
}

/* FACT phase-b-contact-damage.md §3.2: an ordinary primary damage packet
 * subtracts the same amount from the impacted armor AND chassis live pools;
 * overflow zeros both and spills only the residual beyond armor capacity. */
static void component_damage_primary_pair(int target, int armor_comp,
                                          int chassis_comp, int dmg)
{
    CombatEnt *e = ent_at(target);
    if (!e || dmg <= 0 || armor_comp < 0 || chassis_comp < 0 ||
        armor_comp >= COMBAT_COMP_COUNT || chassis_comp >= COMBAT_COMP_COUNT)
        return;
    int capacity = e->comp_hp[armor_comp];
    if (dmg > capacity) {
        e->comp_hp[armor_comp] = 0;
        e->comp_hp[chassis_comp] = 0;
        component_spill(e, dmg - capacity);
        return;
    }
    e->comp_hp[armor_comp] -= dmg;
    e->comp_hp[chassis_comp] -= dmg;
    if (e->comp_hp[chassis_comp] < 0)
        e->comp_hp[chassis_comp] = 0; /* deliberate Q12 anti-underflow */
}

/* Native FUN_00417ac0 aggregates vehicle condition from the minimum live/max
 * component/facet ratio. The mission bridge keeps its authored-unit scalar
 * maximum for existing hpLesser/FSM consumers, but derives the live value from
 * the real VCF pools instead of subtracting every hit from an unrelated weak
 * facet. No hull constant is introduced. */
static int sync_car_facet_hp(int target, int attacker, const char *how,
                             int spawn_death_fx)
{
    CombatEnt *e = ent_at(target);
    if (!e || e->class_id != 1 || !e->has_vcf_config || e->hp_max <= 0)
        return 0;
    static const int facets[8] = {
        COMBAT_COMP_ARMOR_F, COMBAT_COMP_ARMOR_L,
        COMBAT_COMP_ARMOR_R, COMBAT_COMP_ARMOR_B,
        COMBAT_COMP_CHASSIS_F, COMBAT_COMP_CHASSIS_L,
        COMBAT_COMP_CHASSIS_R, COMBAT_COMP_CHASSIS_B
    };
    int min_live = 1, min_max = 1;
    int authored = 0;
    for (int i = 0; i < 8; i++) {
        int c = facets[i];
        if (e->comp_max[c] <= 0)
            continue;
        int live = e->comp_hp[c] < 0 ? 0 : e->comp_hp[c];
        if (!authored || (int64_t)live * min_max <
                         (int64_t)min_live * e->comp_max[c]) {
            min_live = live;
            min_max = e->comp_max[c];
        }
        authored = 1;
    }
    if (!authored)
        return 0;
    int before = e->hp;
    int next = (int)((int64_t)e->hp_max * min_live / min_max);
    if (next > before)
        next = before; /* damage events never heal scalar mission state */
    e->hp = next;
    if (before != next)
        fprintf(stdout, "[combat] %s hp %d->%d (%s by %s)\n",
                ent_name(target), before, next, how,
                attacker >= 0 ? ent_name(attacker) : "environment");
    if (e->hp <= 0)
        combat_die(target, attacker, how, spawn_death_fx);
    return 1;
}

/* ----------------------------------------------------------------------- */
/* D-C27: decoded damage/smoke bands (vehicle-death-presentation.md §3)     */
/* ----------------------------------------------------------------------- */

/* FACT §3.1 (FUN_00465e20): a side's fraction is
 * min(armor_remaining/armor_max, chassis_remaining/chassis_max), with the
 * native zero-max guard (remaining x 0.01). */
static double side_fraction(const CombatEnt *e, int armor_comp,
                            int chassis_comp)
{
    double best = 1.0;
    for (int i = 0; i < 2; i++) {
        int c = i ? chassis_comp : armor_comp;
        int max = e->comp_max[c];
        int live = e->comp_hp[c] < 0 ? 0 : e->comp_hp[c];
        double f = max > 0 ? (double)live / (double)max : live * 0.01;
        if (f < best)
            best = f;
    }
    return best;
}

int combat_side_damage_state(int ent, int side)
{
    static const int armor[4] = {
        COMBAT_COMP_ARMOR_F, COMBAT_COMP_ARMOR_L,
        COMBAT_COMP_ARMOR_R, COMBAT_COMP_ARMOR_B
    };
    static const int chassis[4] = {
        COMBAT_COMP_CHASSIS_F, COMBAT_COMP_CHASSIS_L,
        COMBAT_COMP_CHASSIS_R, COMBAT_COMP_CHASSIS_B
    };
    CombatEnt *e = ent_at(ent);
    if (!e || side < 0 || side > 3)
        return -1;
    /* FUN_00465370 forces state 4 (geometry variant 3 + charred texture
     * state 3) on every side at death. */
    if (!e->alive)
        return 4;
    double f = side_fraction(e, armor[side], chassis[side]);
    if (f > 0.75) return 0;
    if (f > 0.5)  return 1;
    if (f > 0.25) return 2;
    return 3;
}

int combat_smoke_level(int ent)
{
    CombatEnt *e = ent_at(ent);
    if (!e)
        return -2;
    /* Aggregate condition, the FUN_00417ac0 analog: the minimum live/max
     * ratio over every component pool. The native 72r+28 pristine-car
     * offset and its subsystem cliff (spec §6.7) are an open decode
     * question and are deliberately not reproduced. */
    double c = 1.0;
    for (int i = 0; i < COMBAT_COMP_COUNT; i++) {
        if (e->comp_max[i] <= 0)
            continue;
        int live = e->comp_hp[i] < 0 ? 0 : e->comp_hp[i];
        double f = (double)live / (double)e->comp_max[i];
        if (f < c)
            c = f;
    }
    if (c >= 0.75) return -1;           /* emitter removed              */
    if (c > 0.6)   return 0;            /* xwp1, rate 0.0               */
    if (c > 0.4)   return 1;            /* xsg1, rate 0.5               */
    return 2;                           /* xbp1, rate 1.0 (burning wreck) */
}

/* ----------------------------------------------------------------------- */
/* H-UAT-007/H-UAT-056: presentation event store                           */
/* ----------------------------------------------------------------------- */

/* Presentation classification consumes ORDF +0, not names. Damage only
 * scales a class after that parsed discriminator has selected its shape. */
static int fx_weapon_class(int ordnance_type, int damage)
{
    switch (ordnance_type) {
    case 1: case 18:
        return COMBAT_FX_TRACER_LIGHT;
    case 6:
        return COMBAT_FX_TRACER_HEAVY;
    case 2: case 3: case 8: case 0x14:
        return COMBAT_FX_MISSILE;
    case 9: case 11:
        return COMBAT_FX_FLAME;
    case 10:
        return COMBAT_FX_GAS;
    case 4: case 5: case 7: case 21: case 22:
        return COMBAT_FX_EXPLOSIVE;
    default:
        /* PORT DECISION fallback for malformed/unknown ORDF records. */
        return damage <= 30 ? COMBAT_FX_TRACER_LIGHT
                            : COMBAT_FX_TRACER_HEAVY;
    }
}

static CombatFx *fx_alloc(int type, int weapon_class, int damage,
                          const double point[3])
{
    int slot = -1;
    for (int i = 0; i < COMBAT_FX_EVENT_MAX; i++)
        if (!s_fx[i].active) { slot = i; break; }
    if (slot < 0) {
        /* Keep the 10 s wreck contract under a dense MG volley: replace
         * the oldest non-kill first. Only an all-kill pool can evict a
         * kill. */
        for (int i = 0; i < COMBAT_FX_EVENT_MAX; i++)
            if (s_fx[i].type != COMBAT_FX_KILL &&
                (slot < 0 || s_fx[i].age > s_fx[slot].age))
                slot = i;
        if (slot < 0) {
            slot = 0;
            for (int i = 1; i < COMBAT_FX_EVENT_MAX; i++)
                if (s_fx[i].age > s_fx[slot].age) slot = i;
        }
    }
    CombatFx *fx = &s_fx[slot];
    memset(fx, 0, sizeof *fx);
    fx->active = 1;
    fx->type = type;
    fx->weapon_class = weapon_class;
    fx->damage = damage;
    fx->hit = COMBAT_FX_HIT_NONE;
    fx->seed = ++s_fx_seed;
    memcpy(fx->start, point, sizeof fx->start);
    memcpy(fx->end, point, sizeof fx->end);
    return fx;
}

static void fx_muzzle(const double point[3], int weapon_class, int damage,
                      double speed)
{
    CombatFx *fx = fx_alloc(COMBAT_FX_MUZZLE, weapon_class, damage, point);
    fx->speed = speed > 0.0 ? speed : COMBAT_FX_SPEED_DEFAULT;
    fx->life = 3;
}

static void fx_impact(const double point[3], int hit, int killed,
                      int weapon_class, int damage, double speed,
                      const char *effect_name)
{
    if (hit >= 0 && hit < COMBAT_MAX_ENTS)
        s_fx_hit_count[hit]++;
    if (!killed && (!effect_name || !effect_name[0] ||
                    strcasecmp(effect_name, "null") == 0))
        return; /* native null ORDF slot: audit the contact, emit no FX */
    CombatFx *fx = fx_alloc(killed ? COMBAT_FX_KILL : COMBAT_FX_IMPACT,
                            weapon_class, damage, point);
    fx->hit = hit;
    fx->killed = killed;
    fx->speed = speed > 0.0 ? speed : COMBAT_FX_SPEED_DEFAULT;
    if (!killed)
        snprintf(fx->effect_name, sizeof fx->effect_name, "%s", effect_name);
    /* D-C27: the kill record is the native authored sequence — the 2.0 s
     * X1_CARX1 explosion, then the burning wreck (heavy xbp1 smoke with
     * per-tick secondary rolls) for the 10.0 s wreck timer. */
    /* Named XDFs own their exact visible lifetime in scene.c. Keep ordinary
     * impact transport alive for the longest shipped 2.0 s prototype so this
     * renderer-neutral event does not parse assets. scene.c distinguishes
     * authored expiry from missing art, and fx_alloc always evicts the oldest
     * non-kill, so expired transport cannot displace a newer impact. */
    fx->life = killed ? COMBAT_WRECK_BURN_TICKS : COMBAT_EXPLOSION_TICKS;
}

int combat_probe_impact_at(const char *effect_name,
                           double x, double y, double z)
{
    if (!effect_name || !effect_name[0] ||
        strlen(effect_name) >= sizeof(((CombatFx *)0)->effect_name) ||
        strcasecmp(effect_name, "null") == 0 ||
        !isfinite(x) || !isfinite(y) || !isfinite(z))
        return -1;
    const double point[3] = { x, y, z };
    fx_impact(point, COMBAT_FX_HIT_WORLD, 0, COMBAT_FX_EXPLOSIVE,
              0, COMBAT_FX_SPEED_DEFAULT, effect_name);
    return 0;
}

static void fx_kill(const double point[3])
{
    fx_impact(point, COMBAT_FX_HIT_NONE, 1, COMBAT_FX_EXPLOSIVE,
              100, 0.0, NULL);
}

/* D-C27: one burning-wreck tick — FUN_00465370(ent, 0), which executes
 * only the rand() & 0xf switch; cases 8-15 fall through, so 50% of ticks
 * do nothing. Rolls consume the dedicated wreck stream (D-C10 law). */
static void wreck_burn_tick(int ent)
{
    double p[3];
    uint32_t roll = wreck_rng() & 0xfu;
    switch (roll) {
    case 0:
        /* Mounted-part pop-off: the native detaches one of 5 mount slots
         * and flings it (horizontal x8.0, up-velocity 2.0). The port has
         * no mounted-part objects to detach — the slot roll is consumed
         * for stream parity and the fling visual is a D-C27 flagged gap. */
        (void)(wreck_rng() % 5u);
        break;
    case 1:
        /* v-chnk body chunk: the native detaches one of 6 body-part
         * slots and flings it at x15.0. Same flagged gap as roll 0. */
        (void)(wreck_rng() % 6u);
        break;
    case 2: case 3:
        /* FUN_004a6650 per attached part at 50% — the helper is
         * undecoded (vehicle-death-presentation.md §6.2), so no port
         * behavior is invented here. */
        break;
    case 4: case 5:
        /* CHUNK1/CHUNK2 debris straight up at 15 m/s (FACT). */
        if (fx_position(ent, p) == 0) {
            CombatFx *fx = fx_alloc(COMBAT_FX_DEBRIS, COMBAT_FX_EXPLOSIVE,
                                    0, p);
            snprintf(fx->ordnance_model, sizeof fx->ordnance_model,
                     "%s", roll == 4 ? "CHUNK1" : "CHUNK2");
            fx->speed = 15.0;
            fx->life = COMBAT_DEBRIS_TICKS;
        }
        break;
    case 6: case 7:
        /* X1_CARS1 secondary explosion at a randomized offset beside the
         * hull (FACT). The native rand-scaled offsets
         * (_DAT_004c4b64/68) are decoded as random but their magnitudes
         * are unquoted — MARKED ±2.5 m hull-side scatter. */
        if (fx_position(ent, p) == 0) {
            p[0] += ((double)(wreck_rng() & 0xffu) / 255.0 - 0.5) * 5.0;
            p[2] += ((double)(wreck_rng() & 0xffu) / 255.0 - 0.5) * 5.0;
            CombatFx *fx = fx_alloc(COMBAT_FX_SECONDARY,
                                    COMBAT_FX_EXPLOSIVE, 0, p);
            fx->life = COMBAT_EXPLOSION_TICKS;
        }
        break;
    default:
        break;                          /* 8..15: 50% of ticks do nothing */
    }
}

/* Resolve an entity's live 3-D position for presentation (falls back to
 * the tick snapshot for entities without a resolver). */
static int fx_position(int ent, double out[3])
{
    CombatEnt *e = ent_at(ent);
    if (!e)
        return -1;
    if (s_resolve && e->has_obj) {
        s_resolve(ent, out);
        return 0;
    }
    if (ent == s_user && s_have_upose) {
        out[0] = s_ux;
        out[1] = 0.0;
        out[2] = s_uz;
        return 0;
    }
    if (e->have_pos) {
        out[0] = e->px;
        out[1] = 0.0;
        out[2] = e->pz;
        return 0;
    }
    return -1;
}

/* ----------------------------------------------------------------------- */
/* Damage events                                                           */
/* ----------------------------------------------------------------------- */

/* Shared shot application. Projectile callers provide their exact segment
 * contact and parsed ORDF class; direct event callers fall back to the live
 * target centre. Presentation inputs cannot alter hp/pulses/impulse. */
static void shot_apply(int attacker, int target, int dmg, double fx_speed,
                       int weapon_class, const double impact[3],
                       const char *effect_name)
{
    CombatEnt *e = ent_at(target);
    double ap[3], tp[3], point[3];
    int have_ap = 0, have_tp = 0, have_impact = impact != NULL;
    if (!e || !e->alive)
        return;
    /* Presentation-only flame/gas remains observable even when a future
     * damage owner supplies zero. Do not set pulses/components/HP/impulse. */
    if (dmg <= 0) {
        if (have_impact)
            fx_impact(impact, target, 0, weapon_class, 0, fx_speed,
                      effect_name);
        return;
    }
    if (attacker >= 0)
        have_ap = fx_position(attacker, ap) == 0;
    have_tp = fx_position(target, tp) == 0;
    if (have_impact)
        memcpy(point, impact, sizeof point);
    else if (have_tp) {
        memcpy(point, tp, sizeof point);
        point[1] += 1.0;
        have_impact = 1;
    }

    e->p_shot = 1;
    e->p_attacked = 1;
    e->who_shot = attacker;
    /* FACT ownership: isAttacked/whoAttacked is the authored event seam.
     * MARKED response: a friendly non-user car keeps an 8-second evasion
     * window after hostile fire; combat_tick turns that into a bounded weave. */
    if (target != s_user && e->class_id == 1 && e->team == s_user_team &&
        attacker >= 0 && hostile_to(ent_at(attacker), e->team)) {
        if (e->under_fire_ticks <= 0) e->under_fire_phase = 0;
        e->under_fire_ticks = 160;
    }
    e->who_attacked = attacker;
    /* H-UAT-067a: car mission HP is derived from the impacted authored pools,
     * not decremented ahead of them. Ordinary native packets hit armor and
     * chassis together (Phase B §3.2); decoded flame/gas bypasses armor and
     * reaches chassis only. Non-cars/unresolved geometry retain scalar HP. */
    if (have_ap && e->class_id == 1) {
        int q = impact_quadrant(target, ap[0], ap[2]);
        if (weapon_class == COMBAT_FX_FLAME ||
            weapon_class == COMBAT_FX_GAS)
            component_damage(target, s_quad_chassis[q], dmg);
        else
            component_damage_primary_pair(target, s_quad_armor[q],
                                           s_quad_chassis[q], dmg);
        /* This impact record becomes the kill record when damage is fatal, so
         * combat_die must not create a duplicate generic wreck sequence. */
        if (!sync_car_facet_hp(target, attacker, "shot", 0))
            apply_damage(target, dmg, attacker, "shot", 0);

        /* D-C20 (H-UAT-011a): one explicitly marked external velocity
         * impulse at the physical owner. Promoted hosts route through ai.c
         * to car.c and never also update contact_v*; the player uses context
         * 0. Unpromoted Stage-4 route cars retain the kinematic bridge. */
        if (have_tp && e->alive && e->class_id == 1) {
            double dx = tp[0] - ap[0], dz = tp[2] - ap[2];
            double d = sqrt(dx * dx + dz * dz);
            if (d > 1e-6) {
                double dv = COMBAT_SHOT_IMPULSE_PER_HP * (double)dmg;
                if (dv > COMBAT_SHOT_IMPULSE_MAX)
                    dv = COMBAT_SHOT_IMPULSE_MAX;
                if (target == s_user)
                    car_add_world_velocity(dx / d * dv, dz / d * dv);
                else
                    ai_add_contact_velocity(target, dx / d * dv,
                                            dz / d * dv);
            }
        }
    } else {
        apply_damage(target, dmg, attacker, "shot", 0);
    }
    if (have_impact) {
        if (!e->alive) {
            /* Native contact dispatch and vehicle death are independent:
             * retain the ORDF target-class XDF, then start X1_CARX1. The
             * impact record owns the one monotone hit count; fx_kill uses
             * HIT_NONE and cannot double-count the lethal projectile. */
            fx_impact(point, target, 0, weapon_class, dmg, fx_speed,
                      effect_name);
            /* X1_CARX1 is entity-origin anchored; only ORDF contact art
             * belongs on the struck COLP face. */
            fx_kill(have_tp ? tp : point);
        } else {
            fx_impact(point, target, 0, weapon_class, dmg, fx_speed,
                      effect_name);
        }
    }
}

void combat_shot(int attacker, int target, int dmg)
{
    shot_apply(attacker, target, dmg, 0.0,
               fx_weapon_class(0, dmg), NULL, NULL);
}

static void combat_ram_damage(int attacker, int target, int dmg)
{
    CombatEnt *e = ent_at(target);
    if (!e || !e->alive || dmg <= 0)
        return;
    e->p_rammed = 1;
    e->p_attacked = 1;
    e->who_rammed = attacker;
    if (target != s_user && e->class_id == 1 && e->team == s_user_team &&
        attacker >= 0 && hostile_to(ent_at(attacker), e->team)) {
        if (e->under_fire_ticks <= 0) e->under_fire_phase = 0;
        e->under_fire_ticks = 160;
    }
    e->who_attacked = attacker;

    /* H-UAT-003: force/collision damage strikes the CHASSIS facet facing
     * the impact. Authored car pools own death; unresolved/non-car targets
     * retain scalar fallback. */
    int routed = 0;
    if (attacker >= 0 && e->class_id == 1) {
        double ap[3];
        if (fx_position(attacker, ap) == 0) {
            component_damage(target,
                             s_quad_chassis[impact_quadrant(target, ap[0],
                                                            ap[2])],
                             dmg);
            routed = sync_car_facet_hp(target, attacker, "rammed", 1);
        }
    }
    if (!routed)
        apply_damage(target, dmg, attacker, "rammed", 1);
}

void combat_ram(int attacker, int target, double closing)
{
    if (!(closing > 0.0) || !isfinite(closing))
        return;
    int dmg = (int)(closing * 0.5);          /* MARKED D-C24 convention */
    if (dmg < 1)
        dmg = 1;
    if (dmg > COMBAT_RAM_DAMAGE_MAX)
        dmg = COMBAT_RAM_DAMAGE_MAX;
    combat_ram_damage(attacker, target, dmg);
}

static void combat_player_ram_pair(CombatEnt *u, CombatEnt *t, int target,
                                   double closing)
{
    /* MARKED PORT CONVENTION (D-C24): P17 establishes that an intentional
     * player ram must disable a larger target before its authored escape.
     * Keep the prior 1.125 recipient-percentage calibration only on the
     * player's outgoing packet, capped below half health so one contact never
     * kills. The symmetric return packet is the bounded speed-only cost. */
    int outgoing_cap = (int)((int64_t)t->hp_max *
                             COMBAT_RAM_OUTGOING_MAX_PCT / 100);
    if (outgoing_cap < 1) outgoing_cap = 1;
    double raw = closing * 0.5 * COMBAT_RAM_OUTGOING_SCALE *
                 (double)t->hp_max / COMBAT_DEFAULT_HP;
    int outgoing = raw >= (double)outgoing_cap ? outgoing_cap : (int)raw;
    if (outgoing < 1) outgoing = 1;
    combat_ram_damage(s_user, target, outgoing);
    combat_ram(target, s_user, closing);
    u->ram_cool = COMBAT_RAM_COOLDOWN;
    t->ram_cool = COMBAT_RAM_COOLDOWN;
}

void combat_player_vehicle_contact(int target, double closing)
{
    CombatEnt *u = ent_at(s_user);
    CombatEnt *t = ent_at(target);
    if (!u || !t || !u->alive || !t->alive || !isfinite(closing) ||
        closing < COMBAT_RAM_CLOSING || u->ram_cool > 0 || t->ram_cool > 0)
        return;
    combat_player_ram_pair(u, t, target, closing);
}

void combat_landing_events(int hard_landings, double last_impact)
{
    CombatEnt *u;
    if (hard_landings < s_land_seen)
        s_land_seen = hard_landings;        /* car re-placed: resync */
    u = s_user >= 0 ? ent_at(s_user) : NULL;
    while (s_land_seen < hard_landings) {
        s_land_seen++;
        /* MARKED H-UAT-070b convention: preserve the FACT 7.65 m/s hard-
         * landing event, but do not turn near-threshold authored-road hops
         * into attrition. Exact native speed/per-part conversion is a demand
         * row. Environmental damage emits no attack pulses. */
        int dmg = (int)floor((last_impact - COMBAT_LAND_HARD_SPEED) /
                             COMBAT_LAND_DAMAGE_STEP);
        if (dmg < 0) dmg = 0;
        if (dmg > COMBAT_LAND_DAMAGE_MAX) dmg = COMBAT_LAND_DAMAGE_MAX;
        fprintf(stdout, "[combat] hard landing %.1f m/s -> %d dmg\n",
                last_impact, dmg);
        if (u && u->alive && dmg > 0) {
            apply_damage(s_user, dmg, -1, "landing", 1);
            /* H-UAT-003 DECISION (port routing, not reversed semantics):
             * a hard landing loads the suspension and all four tires —
             * the only events that ever touch those pools, so the
             * panel's SUSP/WHEEL regions stay observable. The original's
             * per-part landing roll (FUN_00463370) is not decompiled at
             * that granularity. */
            component_damage(s_user, COMBAT_COMP_SUSPENSION, dmg);
            component_damage(s_user, COMBAT_COMP_TIRE_RR, dmg);
            component_damage(s_user, COMBAT_COMP_TIRE_RL, dmg);
            component_damage(s_user, COMBAT_COMP_TIRE_FR, dmg);
            component_damage(s_user, COMBAT_COMP_TIRE_FL, dmg);
        }
    }
}

/* ----------------------------------------------------------------------- */
/* Verbs                                                                   */
/* ----------------------------------------------------------------------- */

void combat_destroy(int ent)
{
    /* Nitro action id 50 is a kill, not a despawn (CONFIRMED:
     * phase-a-ai-driver.md §5, FUN_00418050 -> vehicle FUN_00464450 /
     * structure FUN_004610b0). Keep death, attribution, visibility, AI and
     * presentation effects centralized in combat_die. The original applies
     * full brake to a dying vehicle; ai_sit inside combat_die is this port's
     * mover-owner equivalent. A repeated destroy is harmless, matching the
     * original's guarded diagnostic path. */
    combat_die(ent, -1, "FSM destroy", 1);
}

void combat_attack(int ent, int target)
{
    CombatEnt *e = ent_at(ent);
    CombatEnt *t;
    if (!e || !e->alive)
        return;
    t = ent_at(target);
    if (!t || !t->alive || t->hidden || target == ent) {
        /* "attack no one" (e.g. nearestEnemy = -1): hold position (D-C5).
         * A dead or withdrawn (hidden) target is just as invalid — there
         * is nothing left to chase, so the attacker sits rather than
         * dissolving into the D-C15 fallback and hunting somebody else.
         * D-C15: an explicit hold also bars the autonomous fallback until
         * the script issues a real target — explicit FSM commands win. */
        e->eng_target = -1;
        e->eng_auto = 0;
        e->eng_hold = 1;
        e->eng_slot = 0;
        e->eng_cool = 0;
        ai_sit(ent);
        return;
    }
    if (e->eng_target != target || e->eng_auto) {
        if (e->eng_target != target)
            e->eng_slot = 0;          /* D-C26: new target re-contends */
        e->eng_target = target;
        e->eng_cool = 0;
    }
    e->eng_auto = 0;    /* explicit target: never the fallback's (D-C15) */
    e->eng_hold = 0;
    ai_clear_sat(ent);  /* an explicit attack order ends the sit park
                           (D-A17) — attack from ambush must fire */
}

void combat_hide(int ent, int hidden)
{
    CombatEnt *e = ent_at(ent);
    if (!e)
        return;
    /* death hiding is terminal (D-C8) */
    if (!e->alive)
        return;
    e->hidden = hidden ? 1 : 0;
    if (e->hidden)
        e->eng_slot = 0;
    if (e->scene_obj >= 0)
        scene_obj_set_hidden(e->scene_obj, e->hidden);
}

/* Player fire along the horizontal direction (fx, fz): cooldown and ammo
 * gate a projectile spawn. Segment delivery later owns contact. Shared by
 * selected hardpoints and the independently governed sidearm (D-C14). */
static double player_weapon_range(const CombatWeapon *weapon)
{
    if (weapon && weapon->source >= 0) {
        CarWeaponInfo wi;
        if (car_weapon_get(weapon->source, &wi) == 0 && wi.range_m > 0.0)
            return wi.range_m;
    }
    return COMBAT_FIRE_RANGE;
}

static int projectile_alloc(void)
{
    for (int i = 0; i < COMBAT_PROJECTILE_MAX; i++)
        if (!s_projectiles[i].active) return i;
    return -1; /* bounded native-style pool: a saturated trigger is a miss */
}

static void flame_stream_fire(int attacker, int damage, int ordnance_type,
                              const double start[3],
                              double fx, double fy, double fz,
                              double speed, double range,
                              const char *impact_ground,
                              const char *impact_car,
                              const char *impact_building,
                              const char *impact_structure);

static void projectile_spawn(int attacker, int damage, int ordnance_type,
                             int tracking_target, const double start[3],
                             double fx, double fy, double fz,
                             double speed, double range,
                             const char *impact_ground,
                             const char *impact_car,
                             const char *impact_building,
                             const char *impact_structure)
{
    double n = sqrt(fx*fx + fy*fy + fz*fz);
    if (!(speed > 0.0) || !(range > 0.0) || n < 1e-9)
        return;
    /* FACT: native FUN_004ac490 routes ORDF 9/10/11 to the dedicated flamer
     * manager instead of allocating an ordinary ordnance entity. */
    if (ordnance_type >= 9 && ordnance_type <= 11) {
        flame_stream_fire(attacker, damage, ordnance_type, start,
                          fx, fy, fz, speed, range, impact_ground,
                          impact_car, impact_building, impact_structure);
        return;
    }

    int slot = projectile_alloc();
    if (slot < 0)
        return;
    CombatProjectile *p = &s_projectiles[slot];
    /* Pool reuse starts a new clock/trail; this clears age and trail_count. */
    memset(p, 0, sizeof *p);
    p->active = 1;
    p->attacker = attacker;
    p->damage = damage;
    p->ordnance_type = ordnance_type;
    p->weapon_class = fx_weapon_class(ordnance_type, damage);
    p->tracking_target = tracking_target;
    p->seed = ++s_projectile_seed;
    p->x = start[0]; p->y = start[1]; p->z = start[2];
    p->speed = speed;
    p->vx = fx / n * speed;
    p->vy = fy / n * speed;
    p->vz = fz / n * speed;
    p->distance_left = range;
    p->trail_count = 1;
    memcpy(p->trail[0], start, sizeof p->trail[0]);
    snprintf(p->impact_ground, sizeof p->impact_ground, "%s",
             impact_ground ? impact_ground : "");
    snprintf(p->impact_car, sizeof p->impact_car, "%s",
             impact_car ? impact_car : "");
    snprintf(p->impact_building, sizeof p->impact_building, "%s",
             impact_building ? impact_building : "");
    snprintf(p->impact_structure, sizeof p->impact_structure, "%s",
             impact_structure ? impact_structure : "");
    if(attacker>=0&&attacker<COMBAT_MAX_ENTS)s_launch_count[attacker]++;

    /* Presentation observes the real projectile from this point forward;
     * the separate three-tick record is only the muzzle flash. */
    fx_muzzle(start, p->weapon_class, damage, speed);
}

static int segment_target_hit(const double a[3], const double b[3],
                              int attacker, double radius,
                              int *target, double *hit_t)
{
    int best = -1;
    double bt = 2.0;
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *e = &s_cents[i];
        double c[3], t;
        if (i == attacker || !e->used || !e->alive || e->hidden ||
            !e->has_obj || fx_position(i, c) != 0)
            continue;
        if (e->class_id != 1 && e->class_id != 9 && e->scene_obj >= 0) {
            /* Mission/destructible structures use the exact active authored
             * part OBBs the world path uses. No aggregate box or invented
             * non-car size can bridge empty space between scene parts. */
            for (int p = 0; p < scene_obj_part_count(e->scene_obj); p++) {
                double pc[3], half[2], axis[2], ys[2];
                if (!scene_obj_part_active(e->scene_obj, p) ||
                    scene_part_drive_surface(e->scene_obj, p) ||
                    scene_obj_part_obb(e->scene_obj, p, pc, half, axis,
                                       ys) != 0)
                    continue;
                half[0] += radius;
                half[1] += radius;
                ys[0] -= radius;
                ys[1] += radius;
                if (segment_obb_hit(a, b, pc, half, axis, ys, &t) &&
                    t < bt) {
                    bt = t;
                    best = i;
                }
            }
            continue;
        }
        double hx = e->collision_half[0], hy = e->collision_half[1],
               hz = e->collision_half[2];
        if (!(hx > 0.0 && hy > 0.0 && hz > 0.0))
            continue; /* unresolved entity has no invented hit volume */
        double half[2] = { hx + radius, hz + radius };
        double axis[2] = { 1.0, 0.0 };
        double ys[2] = { c[1] - hy - radius, c[1] + hy + radius };
        if (segment_obb_hit(a, b, c, half, axis, ys, &t) && t < bt) {
            bt = t;
            best = i;
        }
    }
    if (best < 0) return 0;
    if (target) *target = best;
    if (hit_t) *hit_t = bt;
    return 1;
}

/* FACT boundary: native FUN_004ac490 routes ORDF 9/10/11 to the flamer
 * manager; FUN_0042b590 builds a live chain of swept segments, tests contact
 * through FUN_004205e0, then calls FUN_004a7cb0 for class damage/FSM events.
 * The exact native curved segment construction and collision radius are not
 * decoded. MARKED PORT CONVENTION: one straight 1.5 m-radius stream from the
 * current muzzle to the published family-5 range is tested on each authored
 * cadence tick. It has no target lead, angular dispersion, or homing. */
static void flame_stream_fire(int attacker, int damage, int ordnance_type,
                              const double start[3],
                              double fx, double fy, double fz,
                              double speed, double range,
                              const char *impact_ground,
                              const char *impact_car,
                              const char *impact_building,
                              const char *impact_structure)
{
    double n = sqrt(fx * fx + fy * fy + fz * fz);
    double b[3] = { start[0] + fx / n * range,
                    start[1] + fy / n * range,
                    start[2] + fz / n * range };
    double wt = 2.0, tt = 2.0, end_t = 1.0;
    int target = -1, impact_kind = -1, hit_obj = -1;
    int th = segment_target_hit(start, b, attacker,
                                COMBAT_FLAME_STREAM_RADIUS, &target, &tt);
    /* The marked flamer convention retains its central rendered-triangle
     * terrain test; FUN_004adf90 is not an ORDF 9/10/11 owner. */
    int wh = segment_world_hit(start, b, -1, 1, 0,
                               COMBAT_FLAME_STREAM_RADIUS, &wt,
                               &impact_kind, &hit_obj);
    int weapon_class = fx_weapon_class(ordnance_type, damage);
    if (attacker >= 0 && attacker < COMBAT_MAX_ENTS)
        s_launch_count[attacker]++;

    if (th && (!wh || tt < wt))
        end_t = tt;
    else if (wh)
        end_t = wt;
    /* `end` is the selected contact fraction, not the full-range `b`; both
     * the visible stream and authored XDF anchor at this exact point. */
    double end[3] = { start[0] + (b[0] - start[0]) * end_t,
                      start[1] + (b[1] - start[1]) * end_t,
                      start[2] + (b[2] - start[2]) * end_t };

    /* PORT PRESENTATION: expose this authority stream through the existing
     * software-owned plume record for two sim ticks. WebGPU consumes the same
     * indexed overlay; no renderer owns separate stream geometry. */
    CombatFx *stream = fx_alloc(COMBAT_FX_PROJECTILE, weapon_class,
                                damage, start);
    stream->life = 2;
    stream->speed = speed;
    stream->trail_count = COMBAT_FX_TRAIL_MAX;
    for (int k = 0; k < stream->trail_count; k++) {
        double u = (double)k / (double)(stream->trail_count - 1);
        for (int axis = 0; axis < 3; axis++)
            stream->trail[k][axis] = start[axis] + (end[axis] - start[axis]) * u;
    }
    memcpy(stream->end, end, sizeof stream->end);
    fx_muzzle(start, weapon_class, damage, speed);

    if (th && (!wh || tt < wt)) {
        if (attacker >= 0 && attacker < COMBAT_MAX_ENTS)
            s_contact_count[attacker]++;
        CombatEnt *te = ent_at(target);
        const char *effect = te ? impact_effect_for_class(te->class_id,
            impact_ground, impact_car, impact_building, impact_structure) : NULL;
        shot_apply(attacker, target, damage, speed, weapon_class, end,
                   effect);
    } else if (wh) {
        if (attacker >= 0 && attacker < COMBAT_MAX_ENTS)
            s_world_absorb_count[attacker]++;
        if (combat_scene_entity(hit_obj) < 0)
            scene_obj_damage(hit_obj, damage);
        fx_impact(end, COMBAT_FX_HIT_WORLD, 0, weapon_class, damage, speed,
                  impact_effect_for_class(impact_kind, impact_ground,
                                          impact_car, impact_building,
                                          impact_structure));
    }
}

static void tracking_turn(CombatProjectile *p, const double target[3])
{
    double ux = p->vx / p->speed, uy = p->vy / p->speed,
           uz = p->vz / p->speed;
    double dx = target[0] - p->x, dy = target[1] - p->y,
           dz = target[2] - p->z;
    double n = sqrt(dx * dx + dy * dy + dz * dz);
    if (n <= 1e-9) return;
    dx /= n; dy /= n; dz /= n;
    double dot = ux * dx + uy * dy + uz * dz;
    if (dot > 1.0) dot = 1.0;
    if (dot < -1.0) dot = -1.0;

    /* FACT, FUN_004a9d40/FUN_004aa510 + 0x4c52b8..d4: type-8/0x14
     * guidance does nothing inside dot 0.998, otherwise turns by
     * 0.75*sin(error), capped at sin(3°), sin(15°), or sin(20°) after
     * 0/15/150 m of authored flight. This replaces the old perfect lock. */
    if (dot >= 0.9980000257492065) return;
    double angle = acos(dot);
    double turn = 0.75 * sin(angle);
    double cap = p->distance_flown < 15.0 ? 0.05233589932322502
               : p->distance_flown < 150.0 ? 0.2588190436363220
                                            : 0.3420201539993286;
    if (turn > cap) turn = cap;
    if (turn > angle) turn = angle;
    double sin_angle = sin(angle);
    if (sin_angle <= 1e-9) return;
    double a = sin(angle - turn) / sin_angle;
    double b = sin(turn) / sin_angle;
    ux = ux * a + dx * b;
    uy = uy * a + dy * b;
    uz = uz * a + dz * b;
    n = sqrt(ux * ux + uy * uy + uz * uz);
    if (n <= 1e-9) return;
    p->vx = ux / n * p->speed;
    p->vy = uy / n * p->speed;
    p->vz = uz / n * p->speed;
}

static void projectile_trail_append(CombatProjectile *p, const double q[3])
{
    if (p->trail_count < COMBAT_FX_TRAIL_MAX) {
        memcpy(p->trail[p->trail_count++], q, sizeof p->trail[0]);
        return;
    }
    memmove(p->trail, p->trail + 1,
            sizeof p->trail[0] * (COMBAT_FX_TRAIL_MAX - 1));
    memcpy(p->trail[COMBAT_FX_TRAIL_MAX - 1], q, sizeof p->trail[0]);
}

static void projectile_tick(void)
{
    const double dt = 0.05;
    for (int i = 0; i < COMBAT_PROJECTILE_MAX; i++) {
        CombatProjectile *p = &s_projectiles[i];
        if (!p->active) continue;
        p->age++;

        /* Types 8/0x14 now consume their decoded bounded guidance. Type 3's
         * separate FUN_004aac70/FUN_004ae340 state remains the marked interim
         * lock until that helper's state ownership is closed. */
        if ((p->ordnance_type == 3 || p->ordnance_type == 8 ||
             p->ordnance_type == 0x14) && p->tracking_target >= 0) {
            double t[3];
            if (fx_position(p->tracking_target, t) == 0) {
                t[1] += 1.0;
                if (p->ordnance_type == 8 || p->ordnance_type == 0x14)
                    tracking_turn(p, t);
                else {
                    double dx=t[0]-p->x, dy=t[1]-p->y, dz=t[2]-p->z;
                    double n=sqrt(dx*dx+dy*dy+dz*dz);
                    if (n > 1e-9) {
                        p->vx=dx/n*p->speed; p->vy=dy/n*p->speed;
                        p->vz=dz/n*p->speed;
                    }
                }
            }
        }

        double step = p->speed * dt;
        if (step > p->distance_left) step = p->distance_left;
        double a[3] = {p->x,p->y,p->z};
        double b[3] = {p->x+p->vx/p->speed*step,
                       p->y+p->vy/p->speed*step,
                       p->z+p->vz/p->speed*step};
        double wt = 2.0, tt = 2.0;
        int target = -1, impact_kind = -1, hit_obj = -1;
        int th = segment_target_hit(a, b, p->attacker, 0.0,
                                    &target, &tt);
        int wh = segment_world_hit(a, b, -1, 1, 1, 0.0, &wt,
                                   &impact_kind, &hit_obj);
        if (th && (!wh || tt < wt)) {
            double impact[3] = { a[0] + (b[0] - a[0]) * tt,
                                 a[1] + (b[1] - a[1]) * tt,
                                 a[2] + (b[2] - a[2]) * tt };
            if (p->attacker >= 0 && p->attacker < COMBAT_MAX_ENTS)
                s_contact_count[p->attacker]++;
            CombatEnt *te = ent_at(target);
            const char *effect = te ? impact_effect_for_class(te->class_id,
                p->impact_ground, p->impact_car, p->impact_building,
                p->impact_structure) : NULL;
            shot_apply(p->attacker, target, p->damage, p->speed,
                       p->weapon_class, impact, effect);
            p->active = 0;
            continue;
        }
        if (wh) {
            double impact[3] = { a[0] + (b[0] - a[0]) * wt,
                                 a[1] + (b[1] - a[1]) * wt,
                                 a[2] + (b[2] - a[2]) * wt };
            if (p->attacker >= 0 && p->attacker < COMBAT_MAX_ENTS)
                s_world_absorb_count[p->attacker]++;
            /* Non-FSM scenery owns its SDFC pool without consuming one of
             * the bounded mission/AI slots or changing scripted enemy sets. */
            if (combat_scene_entity(hit_obj) < 0)
                scene_obj_damage(hit_obj, p->damage);
            fx_impact(impact, COMBAT_FX_HIT_WORLD, 0, p->weapon_class,
                      p->damage, p->speed,
                      impact_effect_for_class(impact_kind, p->impact_ground,
                                              p->impact_car,
                                              p->impact_building,
                                              p->impact_structure));
            p->active = 0;
            continue;
        }
        p->x=b[0]; p->y=b[1]; p->z=b[2];
        projectile_trail_append(p, b);
        p->distance_left -= step;
        p->distance_flown += step;
        if (p->distance_left <= 1e-9)
            p->active = 0;
    }
}

/* ----------------------------------------------------------------------- */
/* Deployed-object/dropper gameplay                                        */
/* ----------------------------------------------------------------------- */

/* The binary's six ORDF records establish separate deployer families, but
 * their object state machines are not decoded. These bounded values are
 * therefore explicit PORT CONVENTIONS, chosen only to expose each authored
 * role without adding a new physics system: lifetimes 6–60 s, 0.5 s mine
 * arming, 0.5 s fire ticks, and oil scaling the existing grip term. */
static int deployed_spawn(int owner, int kind, int damage,
                          const char *ordnance_model,
                          const char *impact_car,
                          const double start[3])
{
    int slot = -1;
    for (int i = 0; i < COMBAT_DEPLOY_MAX; i++)
        if (!s_deployed[i].active) { slot = i; break; }
    if (slot < 0 || kind <= CAR_DEPLOY_NONE || kind >= CAR_DEPLOY_COUNT)
        return 0;                 /* bounded pool: saturated release is lost */

    CombatDeployed *o = &s_deployed[slot];
    memset(o, 0, sizeof *o);
    o->active = 1;
    o->kind = kind;
    o->owner = owner;
    o->damage = damage;
    o->seed = ++s_projectile_seed;
    snprintf(o->ordnance_model, sizeof o->ordnance_model, "%s",
             ordnance_model ? ordnance_model : "");
    snprintf(o->impact_car, sizeof o->impact_car, "%s",
             impact_car ? impact_car : "");
    o->x = start[0];
    o->z = start[2];
    o->y = terrain_height_at(o->x, o->z) + 0.08;
    switch (kind) {
    case CAR_DEPLOY_OIL:
        o->radius = 3.0; o->lifetime = 200; o->arm_ticks = 1; break;
    case CAR_DEPLOY_FIRE:
        o->radius = 2.5; o->lifetime = 120; o->arm_ticks = 1; break;
    case CAR_DEPLOY_MINE:
        o->radius = 3.0; o->lifetime = 1200; o->arm_ticks = 10; break;
    case CAR_DEPLOY_CALTROPS:
        o->radius = 2.5; o->lifetime = 400; o->arm_ticks = 1; break;
    case CAR_DEPLOY_BLOX:
        /* Contact radius includes the crossing vehicle's hull around the
         * visible cinder-block core; the fallback remains damage-only. */
        o->radius = 2.5; o->lifetime = 1200; o->arm_ticks = 1; break;
    case CAR_DEPLOY_ERASER:
        /* Same decoded ORDF 15 manager as Landmines, but five ammo and
         * 1200 authored damage make a high-tier proximity charge the
         * minimal credible convention; exact native behavior stays open. */
        o->radius = 4.0; o->lifetime = 1200; o->arm_ticks = 10; break;
    default:
        o->active = 0; return 0;
    }
    s_deploy_count[kind]++;
    return 1;
}

int combat_probe_deploy_at(int owner, int kind, int damage,
                           double x, double z)
{
    static const char *const model[CAR_DEPLOY_COUNT] = {
        "", "MOILSPIL", "MFIRESPL", "MLNDMINE", "MCALTROP", "mblox",
        "MLNDMINE"
    };
    double point[3] = { x, terrain_height_at(x, z), z };
    const char *name = kind > CAR_DEPLOY_NONE && kind < CAR_DEPLOY_COUNT
                     ? model[kind] : "";
    const char *impact = kind == CAR_DEPLOY_MINE || kind == CAR_DEPLOY_ERASER
                       ? "xmine1.xdf" : "";
    return deployed_spawn(owner, kind, damage, name, impact, point) ? 0 : -1;
}

static int deployed_fx_class(int kind)
{
    if (kind == CAR_DEPLOY_FIRE) return COMBAT_FX_FLAME;
    if (kind == CAR_DEPLOY_OIL) return COMBAT_FX_GAS;
    if (kind == CAR_DEPLOY_CALTROPS) return COMBAT_FX_TRACER_HEAVY;
    return COMBAT_FX_EXPLOSIVE;
}

static void deployed_effect(CombatDeployed *o, int target, int damage)
{
    CombatEnt *e = ent_at(target);
    double point[3] = { o->x, o->y, o->z };
    double target_origin[3];
    int have_target_origin = fx_position(target, target_origin) == 0;
    if (!e || !e->alive)
        return;
    if (damage > 0) {
        /* Dropper contact is not a projectile shot: it does not set p_shot
         * or pass through fire-delivery's LOS/aim gate. It still owns an
         * attacked event and kill attribution for ordinary combat/FSM use. */
        e->p_attacked = 1;
        e->who_attacked = o->owner;
        /* First-party/Tier-1 evidence says fire and Blox bypass armor into
         * chassis. Concussion mines use the ordinary decoded armor+chassis
         * primary packet on the impacted facet; each object still consumes
         * exactly once. Caltrops retain scalar routing pending decode. */
        if (e->class_id == 1 &&
            (o->kind == CAR_DEPLOY_FIRE || o->kind == CAR_DEPLOY_BLOX)) {
            component_damage(target,
                s_quad_chassis[impact_quadrant(target, o->x, o->z)], damage);
            if (!sync_car_facet_hp(target, o->owner, "deployed object", 0))
                apply_damage(target, damage, o->owner, "deployed object", 0);
        } else if (e->class_id == 1 &&
                   (o->kind == CAR_DEPLOY_MINE ||
                    o->kind == CAR_DEPLOY_ERASER)) {
            int q = impact_quadrant(target, o->x, o->z);
            component_damage_primary_pair(target, s_quad_armor[q],
                                           s_quad_chassis[q], damage);
            if (!sync_car_facet_hp(target, o->owner, "deployed object", 0))
                apply_damage(target, damage, o->owner, "deployed object", 0);
        } else {
            apply_damage(target, damage, o->owner, "deployed object", 0);
        }
    }
    if (!e->alive) {
        fx_impact(point, target, 0, deployed_fx_class(o->kind),
                  damage, 0.0, o->impact_car);
        fx_kill(have_target_origin ? target_origin : point);
    } else {
        fx_impact(point, target, 0, deployed_fx_class(o->kind),
                  damage, 0.0, o->impact_car);
    }
    s_deploy_trigger_count[o->kind]++;
}

static void deployed_tick(void)
{
    for (int target = 0; target < COMBAT_MAX_ENTS; target++) {
        if (s_fire_patch_cool[target] > 0)
            s_fire_patch_cool[target]--;
        if (s_mine_cluster_cool[target] > 0)
            s_mine_cluster_cool[target]--;
    }
    for (int i = 0; i < COMBAT_DEPLOY_MAX; i++) {
        CombatDeployed *o = &s_deployed[i];
        if (!o->active)
            continue;
        if (++o->age >= o->lifetime) {
            o->active = 0;
            continue;
        }
        uint64_t now_inside = 0;
        for (int target = 0; target < COMBAT_MAX_ENTS; target++) {
            CombatEnt *e = &s_cents[target];
            double p[3], dx, dz;
            uint64_t bit = UINT64_C(1) << target;
            if (!e->used || !e->alive || e->hidden || !e->has_obj ||
                e->class_id != 1 || fx_position(target, p) != 0)
                continue;
            dx = p[0] - o->x;
            dz = p[2] - o->z;
            int inside = dx * dx + dz * dz <= o->radius * o->radius;
            if (inside)
                now_inside |= bit;
            if (target == o->owner && !o->owner_clear) {
                if (!inside)
                    o->owner_clear = 1;
                else
                    continue;
            }
            if (!inside || o->age < o->arm_ticks)
                continue;
            int entered = (o->inside_mask & bit) == 0;
            switch (o->kind) {
            case CAR_DEPLOY_OIL:
                /* MARKED H-UAT-075b: native flag 0x400 corroborates a grip-
                 * kill path, but scale/duration are undecoded. Presence on the
                 * patch continuously refreshes a two-second tail at 5% of the
                 * existing authored player/AI traction-authority term. */
                if (target == s_user)
                    car_apply_grip_loss(COMBAT_OIL_GRIP_SCALE,
                                        COMBAT_OIL_TAIL_TICKS);
                else
                    ai_apply_grip_loss(target, COMBAT_OIL_GRIP_SCALE,
                                       COMBAT_OIL_TAIL_TICKS);
                if (entered)
                    deployed_effect(o, target, 0);
                break;
            case CAR_DEPLOY_FIRE:
                /* MARKED H-UAT-066/067g convention: a target inside one
                 * hazardous fire region takes the authored 15-point GDF hit
                 * at most once per 0.5 s, regardless of overlapping patch
                 * records. Native overlap/cadence remains a demand row. */
                if ((entered || (o->age - o->arm_ticks) % 10 == 0) &&
                    s_fire_patch_cool[target] == 0) {
                    deployed_effect(o, target, o->damage);
                    s_fire_patch_cool[target] = 10;
                }
                break;
            case CAR_DEPLOY_CALTROPS:
                if (entered)
                    deployed_effect(o, target, o->damage);
                break;
            case CAR_DEPLOY_MINE:
            case CAR_DEPLOY_ERASER: {
                /* MARKED H-UAT-076c convention: the tape proved two mines
                 * released 20 ticks apart only 0.49 m apart, then contacted
                 * the player on adjacent ticks. Like H-UAT-067g's fire-region
                 * precedent, one target may take at most one proximity-charge
                 * packet per 0.5 s. Every contacted object still detonates and
                 * is consumed while suppressed, rather than waiting beside the
                 * car to apply a deferred second lethal packet. Native cluster
                 * overlap remains owned by the H-UAT-066 demand row. */
                int packet = s_mine_cluster_cool[target] == 0
                           ? o->damage : 0;
                deployed_effect(o, target, packet);
                if (packet > 0)
                    s_mine_cluster_cool[target] =
                        COMBAT_MINE_CLUSTER_TICKS;
                o->active = 0;
                break;
            }
            case CAR_DEPLOY_BLOX:
                /* A dynamic scene-OBB registration would mutate the mission's
                 * borrowed static collider table. Use the task's bounded
                 * fallback instead: one authored-damage ram-style impact,
                 * then consume the block that was driven over. */
                deployed_effect(o, target, o->damage);
                s_deploy_obstacle_count++;
                o->active = 0;
                break;
            default:
                o->active = 0;
                break;
            }
            if (!o->active)
                break;
        }
        if (o->active)
            o->inside_mask = now_inside;
    }
}

/* FACT npc-aim.md: Nitro's ordinary launch branch is asymmetric: only
 * two positive draws admit yaw, and the subsequent pitch test is unreachable
 * for nonnegative authored spread. Do not replace it with a symmetric cone. */
static void launch_spread(int type, double degrees, double frame[12])
{
    switch (type) {
    case 1: case 4: case 5: case 6: case 7: case 0xf: case 0x10:
    case 0x12: case 0x13: case 0x15: case 0x16: break;
    default: return; /* Other launch-state branches remain PORT DECISION. */
    }
    if (!(degrees > 0.0) || !isfinite(degrees)) return;
    double u[3];
    for (int i = 0; i < 3; i++) {
        uint32_t x = s_spread_rng;
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        s_spread_rng = x;
        u[i] = (double)(x & 0x7fffu) * 0.000030518509447574615;
    }
    double a = 2.0 * degrees * u[0] - degrees;
    double b = 2.0 * degrees * u[1] - degrees;
    if (!(a > 0.0 && b > 0.0)) return;
    double theta = a * 0.01745329238474369;
    double c = cos(theta), s = sin(theta);
    for (int i = 0; i < 3; i++) {
        double right = frame[i], forward = frame[6+i];
        frame[i] = c * right - s * forward;
        frame[6+i] = s * right + c * forward;
    }
    double norm = sqrt(frame[6]*frame[6]+frame[7]*frame[7]+frame[8]*frame[8]);
    if (norm > 0.0)
        for (int i = 0; i < 3; i++)
            frame[9+i] += frame[6+i]/norm * u[2] * 0.6000000238418579;
}

/* Single source of truth for the selected weapon's real spawn frame. Fixed
 * mounts preserve their composed HLOC/GPOF Y axis; traversing mounts apply
 * the current live joints before both the projectile and reticle consume it. */
static int player_weapon_launch_frame(const CombatWeapon *weapon,
                                      double fallback_fx, double fallback_fz,
                                      double sp[3], double dir[3], int spread)
{
    if (s_user < 0 || !s_have_upose || !weapon || !sp || !dir)
        return -1;
    sp[0] = s_ux; sp[1] = s_uy + 1.0; sp[2] = s_uz;
    dir[0] = fallback_fx; dir[1] = 0.0; dir[2] = fallback_fz;
    if (weapon->source >= 0) {
        double neutral[12], mount[12], live[12], world_frame[12];
        if (car_weapon_muzzle_frame(weapon->source, neutral) == 0 &&
            car_weapon_mount_frame(weapon->source, mount) == 0) {
            if (weapon->traverses)
                turret_live_frame(neutral, mount, weapon->turret_yaw,
                                  weapon->turret_pitch, live);
            else
                memcpy(live, neutral, sizeof live);
            player_frame_world(live, world_frame);
            if (spread)
                launch_spread(weapon->ordnance_type, weapon->launch_spread,
                              world_frame);
            sp[0] = world_frame[9]; sp[1] = world_frame[10];
            sp[2] = world_frame[11];
            dir[0] = world_frame[6]; dir[1] = world_frame[7];
            dir[2] = world_frame[8];
        }
    }
    double n = sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (n < 1e-9)
        return -1;
    for (int i = 0; i < 3; i++)
        dir[i] /= n;
    return 0;
}

static int fire_projectile(CombatWeapon *weapon, int *cool, double fx,
                           double fz, int *hit_ent)
{
    double fire_range = player_weapon_range(weapon);
    if (hit_ent) *hit_ent = -1;
    if (s_user < 0 || !s_have_upose || !weapon) return 0;
    CombatEnt *u = ent_at(s_user);
    if (!u || !u->alive || *cool > 0 || weapon->ammo == 0) return 0;
    if (weapon->ammo > 0) weapon->ammo--;
    *cool = weapon->cooldown;

    double sp[3], dir[3];
    if (player_weapon_launch_frame(weapon, fx, fz, sp, dir, 1) != 0)
        return 0;
    memcpy(s_last_user_muzzle, sp, sizeof sp);
    s_have_last_user_muzzle = 1;
    if (weapon->deploy_kind != CAR_DEPLOY_NONE) {
        (void)deployed_spawn(s_user, weapon->deploy_kind, weapon->damage,
                             weapon->ordnance_model, weapon->impact_car, sp);
        return 1;
    }
    double speed = weapon->pspeed > 0.0 ? weapon->pspeed
                                        : COMBAT_FX_SPEED_DEFAULT;
    int tracking_target = -1;
    if (weapon->ordnance_type == 3 || weapon->ordnance_type == 8 ||
        weapon->ordnance_type == 0x14) {
        double marker[3];
        int candidate = combat_target_marker(marker);
        if (candidate >= 0)
            tracking_target = candidate;
    }
    projectile_spawn(s_user, weapon->damage, weapon->ordnance_type,
                     tracking_target, sp, dir[0], dir[1], dir[2], speed,
                     fire_range, weapon->impact_ground, weapon->impact_car,
                     weapon->impact_building, weapon->impact_structure);
    return 1;
}

int combat_player_fire(int *hit_ent)
{
    /* Space = fire the selected hardpoint or one L-linked class volley.
     * Manual p.31 says linked weapons fire simultaneously; GameSpot's combat
     * guide says linked front guns deliver doubled damage "with every hit".
     * Preserve that observable contract when class peers have different GDF
     * rates: wait until every non-empty armed mount is ready, then let each
     * authored shot set its own cooldown. The slowest member paces the volley. */
    int any = 0;
    int last_hit = -1;
    if (hit_ent)
        *hit_ent = -1;
    for (int i = 0; i < s_weapon_count; i++)
        s_weapons[i].fired = 0;
    int armed_count = 0;
    int volley_ready = 1;
    for (int i = 0; i < s_weapon_count; i++)
        if (s_weapons[i].armed && s_weapons[i].ammo != 0) {
            armed_count++;
            if (s_weapons[i].cool > 0)
                volley_ready = 0;
        }
    if (armed_count > 1 && !volley_ready) {
        refresh_fire_cool();
        return 0;
    }
    for (int i = 0; i < s_weapon_count; i++) {
        CombatWeapon *weapon = &s_weapons[i];
        if (!weapon->armed)
            continue;
        double dir = weapon->rear ? -1.0 : 1.0;
        int hit = -1;
        if (fire_projectile(weapon, &weapon->cool, -sin(s_uyaw) * dir,
                            cos(s_uyaw) * dir, &hit)) {
            weapon->fired = 1;
            any = 1;
            if (hit >= 0)
                last_hit = hit;
        }
    }
    refresh_fire_cool();
    if (hit_ent)
        *hit_ent = last_hit;
    return any;
}

int combat_player_sidearm_fire(int *hit_ent, double dirx, double dirz)
{
    if (!s_sidearm_valid)
        return 0;
    double len = sqrt(dirx * dirx + dirz * dirz);
    if (len < 1e-9)
        return 0;
    return fire_projectile(&s_sidearm, &s_sidearm_cool, dirx / len,
                           dirz / len, hit_ent);
}

/* ----------------------------------------------------------------------- */
/* Predicates                                                              */
/* ----------------------------------------------------------------------- */

int combat_is_dead(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? !e->alive : 0;
}

int combat_is_attacked(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->p_attacked : 0;
}

int combat_is_shot(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->p_shot : 0;
}

int combat_is_rammed(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->p_rammed : 0;
}

int combat_is_grooves_fault(int ent)
{
    CombatEnt *e = ent_at(ent);
    /* BINARY-VERIFIED (FUN_00417ed0 reads ai+0xa6e0; FUN_00418950 sets it):
     * the flag arms only when the victim is DEAD (ent+0x458 bit 0x20,
     * phase-b-contact-damage.md §+0x458) AND the fatal blow came from the
     * player (param_1 == *piVar5, the user object). It is NOT "any attack
     * from the user". P02's fight is exactly this contract: the player must
     * shoot the clown down to surrender (hp<28, cell9=1) WITHOUT killing
     * him — isGroovesFault stays false while he lives, so the mid-race
     * foul check (M7 fail 10/5) and the post-race check (fail 10/8) only
     * trip when the clown is actually destroyed, which is the fail the
     * mission wants (overkill). The port's former pulse-on-any-attack
     * reading fouled every legitimate fight. killed_by persists past the
     * one-tick who_attacked pulse, matching the latched ai+0xa6e0 flag. */
    return e && !e->alive && s_user >= 0 && e->killed_by == s_user;
}

int combat_hp_lesser(int ent, int pct)
{
    CombatEnt *e = ent_at(ent);
    if (!e)
        return 0;
    return (int64_t)e->hp * 100 < (int64_t)pct * e->hp_max;
}

int combat_ammo_lesser(int ent, int pct)
{
    CombatEnt *e = ent_at(ent);
    CombatWeapon *w = selected_weapon();
    if (!e || ent != s_user || !w || w->ammo < 0 || w->ammo_max <= 0)
        return 0;                           /* infinite/unknown ammo */
    return w->ammo * 100 < pct * w->ammo_max;
}

int combat_all_enemy_dead(void)
{
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        const CombatEnt *e = &s_cents[i];
        if (e->used && e->has_obj && e->alive && e->class_id == 1 &&
            hostile_to(e, s_user_team))
            return 0;
    }
    return 1;
}

int combat_all_blg_dead(void)
{
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        const CombatEnt *e = &s_cents[i];
        if (e->used && e->has_obj && e->alive && e->class_id != 1 &&
            hostile_to(e, s_user_team))
            return 0;
    }
    return 1;
}

int combat_who_attacked(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->who_attacked : -1;
}

int combat_who_shot(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->who_shot : -1;
}

int combat_who_rammed(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->who_rammed : -1;
}

int combat_nearest_enemy(int ent)
{
    CombatEnt *ref = ent_at(ent);
    double rp[3];
    int best = -1;
    double best_d2 = 0.0;
    if (!ref || !s_resolve)
        return -1;
    s_resolve(ent, rp);
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        const CombatEnt *e = &s_cents[i];
        double p[3], dx, dz, d2;
        if (!e->used || !e->alive || e->hidden || !e->has_obj ||
            e->class_id != 1 || i == ent)
            continue;
        if (!hostile_to(e, ref->team))
            continue;
        s_resolve(i, p);
        dx = p[0] - rp[0];
        dz = p[2] - rp[2];
        d2 = dx * dx + dz * dz;
        if (best < 0 || d2 < best_d2) {
            best = i;
            best_d2 = d2;
        }
    }
    return best;
}

/* ----------------------------------------------------------------------- */
/* NPC authored fire                                                      */
/* ----------------------------------------------------------------------- */

/* Consume recovered T_C gates and T_D lead-time perturbation before spawn.
 * Accepted fire is not a hit: the traveling projectile still owns contact. */
static int npc_lead_accept(int attacker, int target,
                           const CarCombatWeapon *w, const AiDirectorState *ds,
                           const double frame[12])
{
    CombatEnt *a = ent_at(attacker), *t = ent_at(target);
    if (!a || !t || !(w->flight_speed > 0.0)) return 0;
    /* FUN_00401610 uses the attachment origin and all three velocity axes;
     * its manager speed comes from FUN_004ac460's ordnance prototype. */
    double r[3] = {t->px-frame[9], t->py-frame[10], t->pz-frame[11]};
    double v[3] = {frame[6]*w->flight_speed+a->vx-t->vx,
                   frame[7]*w->flight_speed+a->vy-t->vy,
                   frame[8]*w->flight_speed+a->vz-t->vz};
    double vv = v[0]*v[0]+v[1]*v[1]+v[2]*v[2];
    double lead = vv < 0.01 ? 10000000.0
                  : (v[0]*r[0]+v[1]*r[1]+v[2]*r[2])/vv;
    uint32_t x=s_aim_rng; x^=x<<13; x^=x>>17; x^=x<<5; s_aim_rng=x;
    int draw=(int)(x%1000u)-500;
    lead += (1.0-(double)ds->aim_error)*0.00055*(double)draw;
    if (lead < 0.0) lead=0.0;
    if (lead >= 4.0) return 0;
    double separation2 = 0.0;
    for (int i = 0; i < 3; i++) {
        double miss = r[i]-v[i]*lead;
        separation2 += miss*miss;
    }
    double dx=t->px-a->px, dz=t->pz-a->pz, d2=dx*dx+dz*dz;
    double threshold = 20.0;
    if (t->class_id == 1) {
        threshold = d2 < 144.0 ? 3.0 : d2 < 625.0 ? 6.0 : 10.0;
        if (w->family == 4) threshold *= 0.7;
    }
    if (sqrt(separation2) >= threshold) return 0;
    /* FACT FUN_00401610: veto a predicted same-team car crossing the lane.
     * PORT DECISION: live visible registered cars represent its collision
     * iterator; native inactive-object membership remains unverified. */
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *f = &s_cents[i];
        if (i == attacker || i == target || !f->used || !f->has_obj ||
            !f->alive || f->hidden || f->class_id != 1 || f->team != a->team)
            continue;
        double fx = f->px-a->px, fz = f->pz-a->pz, fd2 = fx*fx+fz*fz;
        if (fd2 > 10000.0) continue;
        double fr[3] = {f->px-frame[9], f->py-frame[10], f->pz-frame[11]};
        double fv[3] = {frame[6]*w->flight_speed+a->vx-f->vx,
                        frame[7]*w->flight_speed+a->vy-f->vy,
                        frame[8]*w->flight_speed+a->vz-f->vz};
        double fv2 = fv[0]*fv[0]+fv[1]*fv[1]+fv[2]*fv[2];
        double ft = fv2 < 0.01 ? 10000000.0
                    : (fv[0]*fr[0]+fv[1]*fr[1]+fv[2]*fr[2])/fv2;
        if (ft < 0.0) ft = 0.0;
        if (ft >= 4.0) continue;
        double miss2 = 0.0;
        for (int j = 0; j < 3; j++) {
            double miss = fr[j]-fv[j]*ft;
            miss2 += miss*miss;
        }
        if (s_user < 0 || (miss2 < 64.0 && (fd2 >= 64.0 || miss2 < 4.0)))
            return 0;
    }
    return 1;
}

static int npc_fire_accept(int ent, int target, const CarCombatWeapon *w,
                           const double frame[12])
{
    AiDirectorState ds;
    if (ai_director_state(ent, &ds) != 0)
        return 1;                         /* synthetic/unregistered fallback */
    double p = ds.fire_probability;
    if (!(p > 0.0))
        return 0;
    /* PORT DECISION: retain the legacy ORDF +12 selector for now. Fresh
     * FUN_004aed70 decode identifies the native selector as GDFC family,
     * with tier-specific branches and a friendly-lane veto (npc-aim.md).
     * The shared gun lead calculation below is only one of those predicates. */
    int accepted;
    switch (w->manager_type) {
    case 0:                               /* legacy no-fire manager */
    case 8:                               /* legacy Blox exclusion */
        return 0;
    case 3:                               /* separate predictive branch */
        accepted = 1;
        break;
    case 4: {                             /* manager-4 squared T_C gate */
        unsigned draw = fire_rng() % 5000u;
        accepted = (double)draw <= p * p * 1000.0;
        break;
    }
    case 1:
    case 2: {                             /* gun / landmine manager */
        unsigned draw = fire_rng() % 1000u + fire_rng() % 1000u +
                        fire_rng() % 1000u;
        accepted = (double)draw <= p * 6000.0;
        break;
    }
    default:
        return 0;
    }
    if (!accepted) return 0;
    if ((w->manager_type == 1 || w->manager_type == 2 ||
         w->manager_type == 4) && !combat_can_see(ent, target))
        return 0;
    if (w->manager_type == 1 || w->manager_type == 2)
        return npc_lead_accept(ent, target, w, &ds, frame);
    return 1;
}

/* Nitro's weapon manager evaluates every mounted instance. Each NPC mount
 * therefore keeps its own authored cadence/ammo and every ready in-range
 * mount may trigger on a tick; the old one-ping engagement governor remains
 * only for entities whose VCF failed to decode. */
static int npc_fire(int attacker, int target, double distance,
                    CombatEnt *e, double fdx, double fdz)
{
    if (e->weapon_count <= 0) {
        if (distance <= COMBAT_ATTACK_RANGE_FALLBACK && e->eng_cool == 0) {
            combat_shot(attacker, target, COMBAT_ATTACK_DMG_FALLBACK);
            e->eng_cool = COMBAT_ATTACK_PERIOD_FALLBACK;
            return 1;
        }
        return 0;
    }
    int triggered = 0;
    for (int slot = 0; slot < e->weapon_count; slot++) {
        CarCombatWeapon *w = &e->weapons[slot];
        if (e->weapon_cool[slot] > 0 || w->ammo == 0)
            continue;
        if (w->deploy_kind != CAR_DEPLOY_NONE) {
            /* PORT DECISION: retain legacy ORDF +12 deployer routing until
             * native family/tier dispatch is consumed (npc-aim.md). Managers
             * 0/8 do not arm cadence; manager 2 uses the shared T_C / LOS /
             * lead gate, manager 4 the T_C² / LOS gate. No hit-latch dump.
             * This fallback uses the hitch facing; native FUN_004aeeb0
             * supplies the attachment frame to its decision predicate. */
            if (w->manager_type != 2 && w->manager_type != 4)
                continue;
            double dir = w->rear_facing ? -1.0 : 1.0;
            double sp[3];
            if (fx_position(attacker, sp) != 0)
                continue;
            double rear_heading = atan2(fdx, fdz) + M_PI;
            sp[0] += w->muzzle[0] * cos(rear_heading) +
                     w->muzzle[2] * sin(rear_heading);
            sp[1] += w->muzzle[1];
            sp[2] += -w->muzzle[0] * sin(rear_heading) +
                      w->muzzle[2] * cos(rear_heading);
            double frame[12] = {0};
            frame[6] = fdx * dir; frame[8] = fdz * dir;
            memcpy(frame + 9, sp, sizeof sp);
            if (!npc_fire_accept(attacker, target, w, frame)) {
                e->weapon_cool[slot] = w->cooldown_ticks > 0
                                     ? w->cooldown_ticks : 1;
                continue;
            }
            (void)deployed_spawn(attacker, w->deploy_kind, w->damage,
                                 w->ordnance_model, w->impact_car, sp);
            if (w->ammo > 0) w->ammo--;
            e->weapon_cool[slot] = w->cooldown_ticks > 0
                                 ? w->cooldown_ticks : 1;
            triggered = 1;
            continue;
        }
        if (!(w->range_m > 0.0) || distance > w->range_m)
            continue;
        double live[12], world_frame[12];
        if (w->traverses)
            turret_live_frame(w->muzzle_frame, w->mount_frame,
                              w->turret_yaw, w->turret_pitch, live);
        else
            memcpy(live, w->muzzle_frame, sizeof live);
        npc_frame_world(live, e, fdx, fdz, world_frame);
        double launch_fdx = world_frame[6], launch_fdy = world_frame[7],
               launch_fdz = world_frame[8];
        if (launch_fdx * launch_fdx + launch_fdy * launch_fdy +
            launch_fdz * launch_fdz < 1e-12) {
            double dir = w->rear_facing ? -1.0 : 1.0;
            launch_fdx = fdx * dir; launch_fdy = 0.0;
            launch_fdz = fdz * dir;
            world_frame[6] = launch_fdx; world_frame[7] = launch_fdy;
            world_frame[8] = launch_fdz;
        }
        int continuing = e->burst_left[slot] > 0;
        triggered = 1;
        if (!continuing &&
            !npc_fire_accept(attacker, target, w, world_frame)) {
            e->weapon_cool[slot] = w->cooldown_ticks > 0
                                 ? w->cooldown_ticks : 1;
            continue;                       /* rejected aim consumes cadence */
        }
        CombatEnt *t = ent_at(target);
        if (!t || !t->alive) break;
        launch_spread(w->ordnance_type, w->launch_spread, world_frame);
        launch_fdx = world_frame[6]; launch_fdy = world_frame[7];
        launch_fdz = world_frame[8];
        double sp[3] = { world_frame[9], world_frame[10], world_frame[11] };
        double speed = w->flight_speed > 0.0 ? w->flight_speed
                                             : w->projectile_speed;
        int track = (w->ordnance_type == 3 || w->ordnance_type == 8 ||
                     w->ordnance_type == 0x14) ? target : -1;
        projectile_spawn(attacker, w->damage, w->ordnance_type, track,
                         sp, launch_fdx, launch_fdy, launch_fdz,
                         speed, w->range_m, w->impact_ground,
                         w->impact_car, w->impact_building,
                         w->impact_structure);
        if (w->ammo > 0) w->ammo--;
        if (!continuing) {
            e->burst_left[slot] = w->fire_amount > 1
                                ? w->fire_amount - 1 : 0;
        } else {
            e->burst_left[slot]--;
        }
        /* FireAmount rounds are separate authored-cadence shots, not one
         * multiplied damage event. After the final round, GDFC +74 owns the
         * burst reload when present; otherwise ordinary +78 cadence does. */
        if (e->burst_left[slot] == 0 && w->burst_cooldown_ticks > 0)
            e->weapon_cool[slot] = w->burst_cooldown_ticks;
        else
            e->weapon_cool[slot] = w->cooldown_ticks > 0
                                 ? w->cooldown_ticks : 1;
    }
    return triggered;
}

/* ----------------------------------------------------------------------- */
/* Tick                                                                    */
/* ----------------------------------------------------------------------- */

/* D-C26 / FUN_00407fd0: count other living cars that already hold an
 * a998-analog slot on `target`. Eligibility is alive + not hidden (the
 * port's stand-in for flags+0x458 bit 0x20 clear). */
static int attack_slot_count(int target, int attacker)
{
    int n = 0;
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *e = &s_cents[i];
        if (!e->used || !e->alive || e->hidden || !e->eng_slot)
            continue;
        if (i == attacker || i == target)
            continue;
        if (e->eng_target == target)
            n++;
    }
    return n;
}

/* Take or keep a chase slot on `target`. Returns 1 when this attacker
 * may start/keep the combat follow. Fire is not gated. */
static int attack_slot_take(int attacker, int target)
{
    CombatEnt *e = ent_at(attacker);
    AiDirectorState ds;
    int cap = 1000;
    if (!e)
        return 0;
    if (e->eng_slot && e->eng_target == target)
        return 1;
    if (ai_director_state(target, &ds) == 0)
        cap = ds.max_attackers;
    if (cap <= 0)
        return 0;
    if (attack_slot_count(target, attacker) >= cap)
        return 0;
    e->eng_slot = 1;
    return 1;
}

void combat_tick(void)
{
    /* 1. event pulses are one-tick levels (D-C11) */
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        s_cents[i].p_attacked = 0;
        s_cents[i].p_shot = 0;
        s_cents[i].p_rammed = 0;
        s_cents[i].who_attacked = -1;
        s_cents[i].who_shot = -1;
        s_cents[i].who_rammed = -1;
    }

    for (int i = 0; i < s_weapon_count; i++)
        if (s_weapons[i].cool > 0)
            s_weapons[i].cool--;
    refresh_fire_cool();
    if (s_sidearm_cool > 0)
        s_sidearm_cool--;

    /* H-UAT-007: presentation events age on the sim tick, exactly like
     * the cooldowns — decay is deterministic and frame-rate independent. */
    for (int i = 0; i < COMBAT_FX_EVENT_MAX; i++)
        if (s_fx[i].active && ++s_fx[i].age >= s_fx[i].life)
            s_fx[i].active = 0;

    /* D-C27: burning wrecks roll their native per-tick secondary events
     * and hide the hull when the 10.0 s timer expires. Runs ahead of the
     * resolver early-return so an unresolved session still counts down. */
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *e = &s_cents[i];
        if (!e->used || e->wreck_ticks <= 0)
            continue;
        wreck_burn_tick(i);
        if (--e->wreck_ticks == 0 && e->scene_obj >= 0)
            scene_obj_set_hidden(e->scene_obj, 1);
    }

    if (!s_resolve)
        return;

    /* MARKED combat-driving behavior, event-grounded at isAttacked/
     * whoAttacked: protected convoy cars weave while their under-fire latch
     * is live instead of presenting a straight, constant-velocity target.
     * The triangle wave is deterministic at 20 Hz, bounded to +/-0.20 rad,
     * and feeds the existing D-A23 steering slew; it cannot replace routes,
     * authored speeds, or arrival predicates. */
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *e = &s_cents[i];
        if (e->under_fire_ticks <= 0)
            continue;
        int phase = (int)((e->under_fire_phase + i * 17u) % 80u);
        double tri = phase < 40 ? -1.0 + phase / 20.0
                                : 3.0 - phase / 20.0;
        ai_set_steer_bias(i, tri * 0.20);
        e->under_fire_ticks--;
        e->under_fire_phase++;
    }

    /* 2. position snapshot + per-tick velocity (teleport guard, D-C6).
     * Snapshot before flight so collision, homing, and T_D use the current
     * live target pose rather than the previous tick's cached motion. */
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *e = &s_cents[i];
        double p[3] = { 0.0, 0.0, 0.0 };
        if (!e->used || !e->has_obj)
            continue;
        s_resolve(i, p);
        if (e->have_pos) {
            double dx = p[0] - e->px, dy = p[1] - e->py,
                   dz = p[2] - e->pz;
            double d = sqrt(dx * dx + dz * dz);
            if (d > COMBAT_TELEPORT_GUARD ||
                fabs(dy) > COMBAT_TELEPORT_GUARD) {
                e->vx = e->vy = e->vz = 0.0; /* teleport, not speed */
            } else {
                e->vx = dx / AI_TICK_DT;
                e->vy = dy / AI_TICK_DT;
                e->vz = dz / AI_TICK_DT;
            }
        } else {
            e->vx = e->vy = e->vz = 0.0;
        }
        e->px = p[0];
        e->py = p[1];
        e->pz = p[2];
        e->have_pos = 1;
    }

    if (s_user >= 0 && s_have_upose) {
        /* Keep the port's existing nearest-hostile acquisition. Only the
         * decoded mount solution changes; radar re-acquisition remains the
         * explicit dynamic residual in aim-convergence.md §5. */
        int target = combat_nearest_enemy(s_user);
        for (int w = 0; w < s_weapon_count; w++) {
            CombatWeapon *weapon = &s_weapons[w];
            double neutral[12], mount[12], live[12];
            double neutral_world[12], mount_world[12], live_world[12];
            if (!weapon->traverses)
                continue;
            if (target < 0) {
                turret_relax(s_user < COMBAT_MAX_ENTS ?
                                 s_cents[s_user].class_id : 1,
                    ammo_fraction(weapon->ammo, weapon->ammo_max),
                    &weapon->turret_yaw, &weapon->turret_pitch,
                    &weapon->turret_yaw_on_target,
                    &weapon->turret_pitch_on_target);
                continue;
            }
            if (weapon->source < 0 ||
                car_weapon_muzzle_frame(weapon->source, neutral) != 0 ||
                car_weapon_mount_frame(weapon->source, mount) != 0)
                continue;
            turret_live_frame(neutral, mount, weapon->turret_yaw,
                              weapon->turret_pitch, live);
            player_frame_world(neutral, neutral_world);
            player_frame_world(mount, mount_world);
            player_frame_world(live, live_world);
            turret_converge(s_user, target, live_world, neutral_world,
                mount_world, weapon->aim_speed,
                ammo_fraction(weapon->ammo, weapon->ammo_max),
                weapon->family, weapon->tier,
                &weapon->turret_yaw, &weapon->turret_pitch,
                &weapon->turret_yaw_on_target,
                &weapon->turret_pitch_on_target);
        }
    }
    /* Existing projectiles advance before this tick's new trigger requests.
     * Damage therefore cannot land on the spawn tick at positive distance. */
    projectile_tick();
    deployed_tick();

    /* 3. autonomous acquisition (D-C15). A FIELDED hostile — alive,
     * visible, car-class, team-hostile to the user — with no valid
     * engagement and no explicit hold acquires its nearest enemy under the
     * same deterministic team rules the FSM's nearestEnemy predicate uses
     * (combat_nearest_enemy). This never overrides an explicit FSM attack
     * target (eng_target >= 0) or an explicit "attack no one" hold
     * (eng_hold), and never arms friendlies: the convoy's escorts stay
     * purely script-driven. Hidden entities are skipped, so scripted
     * ambush waves still field on their own gates. SAT entities
     * (D-A17: parked by the sit verb) are skipped too — the original's
     * sit behavior carries no target, so a parked car never fires there.
     * An authored race (D-A8) likewise owns steering and weapons until it
     * finishes; P02 deliberately switches from race to evade for its fight. */
    if (s_user_team >= 0) {
        for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
            CombatEnt *e = &s_cents[i];
            int t;
            if (!e->used || !e->alive || e->hidden || !e->has_obj ||
                e->class_id != 1 || i == s_user)
                continue;
            if (ai_sat(i))
                continue;
            if (!hostile_to(e, s_user_team))
                continue;
            if (e->eng_target >= 0 || e->eng_hold)
                continue;
            t = combat_nearest_enemy(i);
            if (t >= 0) {
                e->eng_target = t;
                e->eng_auto = 1;
                e->eng_cool = 0;
            }
        }
    }

    /* 4. engagements (D-C5) */
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *e = &s_cents[i];
        CombatEnt *t;
        double dx, dz, d;
        if (!e->used || !e->alive || e->hidden)
            continue;
        t = ent_at(e->eng_target);
        if (!t || !t->alive || t->hidden) {
            /* FUN_004af210 still runs its no-target branch: a parked or
             * disengaged NPC must not retain the last target's downward
             * pitch. The live joint returns to its authored mount rest. */
            for (int w = 0; w < e->weapon_count; w++) {
                CarCombatWeapon *weapon = &e->weapons[w];
                if (weapon->traverses)
                    turret_relax(e->class_id,
                        ammo_fraction(weapon->ammo, weapon->ammo_capacity),
                        &weapon->turret_yaw, &weapon->turret_pitch,
                        &weapon->turret_yaw_on_target,
                        &weapon->turret_pitch_on_target);
            }
            /* D-C15: a dead or withdrawn target dissolves the engagement;
             * the acquisition pass re-acquires deterministically instead
             * of the entity standing inert (the scripts also re-issue
             * attack on their own polls). */
            e->eng_target = -1;
            e->eng_auto = 0;
            e->eng_slot = 0;
            continue;
        }
        if (ai_sat(i))
            continue;       /* D-A17: parked — no chase, no fire */
        if (e->eng_cool > 0)
            e->eng_cool--;
        for (int w = 0; w < e->weapon_count; w++)
            if (e->weapon_cool[w] > 0)
                e->weapon_cool[w]--;
        if (!e->have_pos || !t->have_pos)
            continue;
        if (e->eng_auto && ai_race_active(i))
            continue;       /* authored race: retain target, do not fire */
        /* chase — unless the script is path-driving this entity. D-C15:
         * a fallback engagement yields to ANY explicit motion goal and to a
         * pending arrival pulse: ai_follow would clear the pulse before the
         * script's next machine slice could read it (the eaten pulse was
         * P02's clown lap counter never incrementing). An explicit FSM attack
         * is itself the script's ownership decision, so it may replace either
         * a completed route or an exact-zero-speed staging path. P01 enemy4's
         * authored teleport onto nav6 uses speed 0 before repeated attack;
         * clamping that staging command to 0.5 m/s left it path-driving for
         * minutes and suppressed pursuit. Moving GOTO/race paths still win.
         * P01 enemy5 never polls isArrived after staging; globally blocking
         * completed-route takeover stranded it more than a kilometre away. */
        int ranged_direct = 0;
        for (int w = 0; e->class_id == 1 && w < e->weapon_count; w++)
            if (e->weapons[w].range_m >= 150.0)
                ranged_direct = 1;
        if (e->eng_auto ? (!ai_arrival_pending(i) &&
                           ai_goal(i) == AI_GOAL_NONE)
                        : (ai_goal(i) != AI_GOAL_GOTO ||
                           ai_stationary_path(i))) {
            /* D-A25: FUN_004152e0 picks dest 6/8/9 from the init hub.
             * D-C26: only a slot holder starts the chase. */
            if (attack_slot_take(i, e->eng_target)) {
                int arms = 0, dest9 = 0;
                for (int w = 0; w < e->weapon_count; w++) {
                    CarCombatWeapon *cw = &e->weapons[w];
                    arms = 1;
                    if (cw->manager_type == 3 || cw->ordnance_type == 3 ||
                        cw->family == 3)
                        dest9 = 1;
                }
                ai_combat_set_arms(i, arms, dest9);
                ai_combat_chase(i, e->eng_target, COMBAT_ATTACK_SPEED);
            }
        }
        dx = t->px - e->px;
        dz = t->pz - e->pz;
        d = sqrt(dx * dx + dz * dz);
        /* The shared gun predicate owns muzzle-relative acceptance and
         * friendly-lane suppression, without a hull cone (npc-aim.md).
         * PORT DECISION: retain the old cone for other legacy managers. */
        if (!t->hidden && d > 1e-6) {
            double heading = ai_get_heading(i);
            /* ai.c heading is the car.h yaw convention: forward (-sin,+cos). */
            double fdx = -sin(heading), fdz = cos(heading);
            for (int w = 0; w < e->weapon_count; w++) {
                CarCombatWeapon *weapon = &e->weapons[w];
                if (weapon->traverses) {
                    /* FUN_004af210 runs every weapon pass, not only when
                     * cadence admits a shot. The engagement target remains
                     * the port's existing acquisition owner. */
                    double live[12], neutral_world[12], mount_world[12],
                           live_world[12];
                    turret_live_frame(weapon->muzzle_frame,
                                      weapon->mount_frame,
                                      weapon->turret_yaw,
                                      weapon->turret_pitch, live);
                    npc_frame_world(weapon->muzzle_frame, e, fdx, fdz,
                                    neutral_world);
                    npc_frame_world(weapon->mount_frame, e, fdx, fdz,
                                    mount_world);
                    npc_frame_world(live, e, fdx, fdz, live_world);
                    turret_converge(i, e->eng_target, live_world,
                        neutral_world, mount_world, weapon->projectile_speed,
                        ammo_fraction(weapon->ammo, weapon->ammo_capacity),
                        weapon->family, weapon->tier,
                        &weapon->turret_yaw, &weapon->turret_pitch,
                        &weapon->turret_yaw_on_target,
                        &weapon->turret_pitch_on_target);
                    continue;
                }
                if (weapon->manager_type == 1 || weapon->manager_type == 2)
                    continue;
                if (weapon->rear_facing ||
                    weapon->deploy_kind != CAR_DEPLOY_NONE) continue;
                double dot = (fdx * dx + fdz * dz) / d;
                if (dot < COMBAT_FIRE_CONE_COS)
                    e->weapon_cool[w] = e->weapon_cool[w] > 0
                                      ? e->weapon_cool[w] : 1;
            }
            /* npc_fire rechecks cadence; front mounts marked above defer
             * exactly this tick. Rear mount directional gating is handled by
             * temporarily mirroring the same dot test. */
            for (int w = 0; w < e->weapon_count; w++)
                if (e->weapons[w].rear_facing &&
                    e->weapons[w].manager_type != 1 &&
                    e->weapons[w].manager_type != 2 &&
                    !e->weapons[w].traverses &&
                    e->weapons[w].deploy_kind == CAR_DEPLOY_NONE) {
                    double dot = -(fdx * dx + fdz * dz) / d;
                    if (dot < COMBAT_FIRE_CONE_COS)
                        e->weapon_cool[w] = e->weapon_cool[w] > 0
                                          ? e->weapon_cool[w] : 1;
                }
            int establishing_pass = ranged_direct && !ai_has_raced(i) &&
                                    d < 147.0;
            double user_dx = e->px - s_ux, user_dz = e->pz - s_uz;
            CombatEnt *user_ent = ent_at(s_user);
            int user_pressure = s_have_upose && user_ent && user_ent->alive &&
                user_dx * user_dx + user_dz * user_dz <= 1000000.0;
            int pressured_heavy = ranged_direct && e->hp_max >= 800 &&
                                  user_pressure;
            if ((!establishing_pass && !pressured_heavy) || e->eng_cool == 0) {
                int fired = npc_fire(i, e->eng_target, d, e, fdx, fdz);
                /* MARKED behavior-table approximation: in the close approach
                 * envelope, gate accepted volleys for 0.3 s. An active user
                 * inside the shipped 1 km radar scale extends >=800-facet
                 * heavy cars to 3 s at any authored weapon range. Native's
                 * personality table cadence remains unrecovered; the old
                 * every-tick trigger emptied a full authored convoy facet in
                 * 1.7 s and defeated projectile travel. */
                if (fired && (establishing_pass || pressured_heavy))
                    e->eng_cool = pressured_heavy ? 60 : 6;
            }
        }
    }

    /* 5. ram scan (D-C6): the physics-driven player car vs car-class
     * entities. AI movers are kinematic (ai.c: no collision — cars pass
     * through each other), so AI-AI "rams" would be ghost contacts
     * (observed: the convoy bunching at waypoints false-fired the
     * detector). Only the user's sim car truly collides; both parties
     * take the hit (the original runs both vehicles' damage handlers,
     * ghidra-physics Q8). */
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *a = &s_cents[i];
        if (a->ram_cool > 0)
            a->ram_cool--;
    }
    if (s_user >= 0) {
        CombatEnt *u = ent_at(s_user);
        if (u && u->alive && u->have_pos && u->ram_cool == 0) {
            for (int j = 0; j < COMBAT_MAX_ENTS; j++) {
                CombatEnt *b = &s_cents[j];
                double dx, dz, d, cvx, cvz, closing;
                if (j == s_user)
                    continue;
                if (!b->used || !b->alive || b->hidden || !b->has_obj ||
                    b->class_id != 1 || !b->have_pos || b->ram_cool > 0)
                    continue;
                dx = b->px - u->px;
                dz = b->pz - u->pz;
                d = sqrt(dx * dx + dz * dz);
                if (d > COMBAT_RAM_DIST)
                    continue;
                cvx = u->vx - b->vx;
                cvz = u->vz - b->vz;
                closing = sqrt(cvx * cvx + cvz * cvz);
                if (closing < COMBAT_RAM_CLOSING)
                    continue;
                /* Same bounded asymmetric convention as the production
                 * physical-contact seam. */
                combat_player_ram_pair(u, b, j, closing);
                break;                      /* one ram per tick */
            }
        }
    }
}

/* ----------------------------------------------------------------------- */
/* Probe accessors                                                         */
/* ----------------------------------------------------------------------- */

int combat_hp(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->hp : -1;
}

int combat_hp_max(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->hp_max : -1;
}

int combat_alive(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->alive : 0;
}

int combat_is_hidden(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->hidden : 0;
}

int combat_wreck_ticks(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->wreck_ticks : 0;
}

int combat_wreck_scene_visible(int ent)
{
    CombatEnt *e = ent_at(ent);
    if (!e || e->scene_obj < 0)
        return 0;
    return !scene_obj_hidden(e->scene_obj);
}

int combat_ammo_left(int ent)
{
    CombatWeapon *w = selected_weapon();
    if (ent != s_user || !w)
        return -1;
    return w->ammo;
}

int combat_ammo_capacity(void)
{
    CombatWeapon *w = selected_weapon();
    return w ? w->ammo_max : 0;
}

int combat_player_weapon_count(void) { return s_weapon_count; }
int combat_player_weapon_selected(void) { return s_weapon_selected; }

int combat_player_weapon_set_speed(int slot, double mps)
{
    if (slot < 0 || slot >= s_weapon_count)
        return -1;
    /* Legacy callers pass GDFC +86. A parsed source already installed the
     * native ORDF +4 flight speed in combat_player_weapon_add; keep it. */
    if (!(s_weapons[slot].pspeed > 0.0))
        s_weapons[slot].pspeed = mps > 0.0 ? mps : 0.0;
    return 0;
}

void combat_player_sidearm_set_speed(double mps)
{
    s_sidearm.pspeed = mps > 0.0 ? mps : 0.0;         /* no HLOC source */
}

const char *combat_player_weapon_name(void)
{
    CombatWeapon *w = selected_weapon();
    return w ? w->name : "";
}

int combat_player_weapon_get(int slot, const char **name,
                             int *ammo, int *ammo_max)
{
    if (slot < 0 || slot >= s_weapon_count)
        return -1;
    if (name) *name = s_weapons[slot].name;
    if (ammo) *ammo = s_weapons[slot].ammo;
    if (ammo_max) *ammo_max = s_weapons[slot].ammo_max;
    return 0;
}

int combat_player_weapon_damage(void)
{
    CombatWeapon *w = selected_weapon();
    return w ? w->damage : 0;
}

int combat_player_weapon_cooldown(void)
{
    return s_fire_cool;
}

int combat_player_weapon_source(void)
{
    CombatWeapon *w = selected_weapon();
    return w ? w->source : -1;
}

int combat_player_weapon_source_at(int slot)
{
    if (slot < 0 || slot >= s_weapon_count)
        return -1;
    return s_weapons[slot].source;
}

int combat_player_weapon_rear(void)
{
    CombatWeapon *w = selected_weapon();
    return w ? w->rear : 0;
}

int combat_player_muzzle_position(double out[3])
{
    if (!out || !s_have_last_user_muzzle) return -1;
    memcpy(out, s_last_user_muzzle, sizeof s_last_user_muzzle);
    return 0;
}

int combat_player_launch_frame(double origin[3], double direction[3])
{
    CombatWeapon *weapon = selected_weapon();
    double facing = weapon && weapon->rear ? -1.0 : 1.0;
    return player_weapon_launch_frame(weapon, -sin(s_uyaw) * facing,
                                      cos(s_uyaw) * facing,
                                      origin, direction, 0);
}

double combat_player_weapon_traverse_yaw(int source)
{
    for (int i = 0; i < s_weapon_count; i++)
        if (s_weapons[i].source == source && s_weapons[i].traverses)
            return s_weapons[i].turret_yaw;
    return 0.0;
}

double combat_player_weapon_traverse_pitch(int source)
{
    for (int i = 0; i < s_weapon_count; i++)
        if (s_weapons[i].source == source && s_weapons[i].traverses)
            return s_weapons[i].turret_pitch;
    return 0.0;
}

int combat_player_weapon_on_target(int source, int *yaw_ok, int *pitch_ok)
{
    for (int i = 0; i < s_weapon_count; i++)
        if (s_weapons[i].source == source && s_weapons[i].traverses) {
            if (yaw_ok) *yaw_ok = s_weapons[i].turret_yaw_on_target;
            if (pitch_ok) *pitch_ok = s_weapons[i].turret_pitch_on_target;
            return 0;
        }
    return -1;
}

unsigned long combat_launch_count(int e){return e>=0&&e<COMBAT_MAX_ENTS?s_launch_count[e]:0;}
unsigned long combat_contact_count(int e){return e>=0&&e<COMBAT_MAX_ENTS?s_contact_count[e]:0;}
unsigned long combat_world_absorb_count(int e){return e>=0&&e<COMBAT_MAX_ENTS?s_world_absorb_count[e]:0;}
unsigned long combat_fx_hit_count(int e){return e>=0&&e<COMBAT_MAX_ENTS?s_fx_hit_count[e]:0;}

int combat_deployed_active(int kind)
{
    int n = 0;
    if (kind <= CAR_DEPLOY_NONE || kind >= CAR_DEPLOY_COUNT)
        return 0;
    for (int i = 0; i < COMBAT_DEPLOY_MAX; i++)
        if (s_deployed[i].active && s_deployed[i].kind == kind)
            n++;
    return n;
}

int combat_deployed_position(int kind, double out[3])
{
    int newest = -1;
    if (!out || kind <= CAR_DEPLOY_NONE || kind >= CAR_DEPLOY_COUNT)
        return -1;
    for (int i = 0; i < COMBAT_DEPLOY_MAX; i++)
        if (s_deployed[i].active && s_deployed[i].kind == kind &&
            (newest < 0 || s_deployed[i].seed > s_deployed[newest].seed))
            newest = i;
    if (newest < 0)
        return -1;
    out[0] = s_deployed[newest].x;
    out[1] = s_deployed[newest].y;
    out[2] = s_deployed[newest].z;
    return 0;
}

unsigned long combat_deploy_count(int kind)
{
    return kind > CAR_DEPLOY_NONE && kind < CAR_DEPLOY_COUNT
         ? s_deploy_count[kind] : 0;
}

unsigned long combat_deploy_trigger_count(int kind)
{
    return kind > CAR_DEPLOY_NONE && kind < CAR_DEPLOY_COUNT
         ? s_deploy_trigger_count[kind] : 0;
}

unsigned long combat_deploy_obstacle_count(void)
{
    return s_deploy_obstacle_count;
}

int combat_ent_position(int ent, double out[3])
{
    CombatEnt *e = ent_at(ent);
    if (!out || !e || !e->has_obj || !s_resolve)
        return -1;
    s_resolve(ent, out);
    return 0;
}

int combat_eng_target(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->eng_target : -1;
}

int combat_under_fire_ticks(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->under_fire_ticks : 0;
}

int combat_team(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->team : -1;
}

const char *combat_ent_label(int ent)
{
    return ent_name(ent);
}

int combat_is_enemy(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? (e->has_obj && e->class_id == 1 &&
                hostile_to(e, s_user_team)) : 0;
}

int combat_user_ent(void)
{
    return s_user;
}

int combat_kills(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e ? e->kills : 0;
}

int combat_killed_by(int ent)
{
    CombatEnt *e = ent_at(ent);
    return e && !e->alive ? e->killed_by : -1;
}

int combat_user_pose(double *x, double *z, double *yaw)
{
    if (!s_have_upose)
        return -1;
    if (x)   *x = s_ux;
    if (z)   *z = s_uz;
    if (yaw) *yaw = s_uyaw;
    return 0;
}

int combat_component_hp(int ent, int comp)
{
    CombatEnt *e = ent_at(ent);
    if (!e || comp < 0 || comp >= COMBAT_COMP_COUNT)
        return -1;
    return e->comp_hp[comp];
}

int combat_component_hp_max(int ent, int comp)
{
    CombatEnt *e = ent_at(ent);
    if (!e || comp < 0 || comp >= COMBAT_COMP_COUNT)
        return -1;
    return e->comp_max[comp];
}

int combat_probe_set_hp(int ent, int hp)
{
    CombatEnt *e = ent_at(ent);
    if (!e || !e->alive || hp <= 0)
        return -1;
    e->hp = e->hp_max = hp;
    for (int c = 0; c < COMBAT_COMP_COUNT; c++)
        e->comp_hp[c] = e->comp_max[c] = hp;
    return 0;
}

int combat_fx_snapshot(CombatFx *out, int cap)
{
    int n = 0;
    if (!out || cap <= 0)
        return 0;
    for (int i = 0; i < COMBAT_FX_EVENT_MAX && n < cap; i++)
        if (s_fx[i].active)
            out[n++] = s_fx[i];
    for (int i = 0; i < COMBAT_PROJECTILE_MAX && n < cap; i++) {
        const CombatProjectile *p = &s_projectiles[i];
        if (!p->active)
            continue;
        CombatFx *fx = &out[n++];
        memset(fx, 0, sizeof *fx);
        fx->active = 1;
        fx->type = COMBAT_FX_PROJECTILE;
        fx->weapon_class = p->weapon_class;
        fx->damage = p->damage;
        fx->age = p->age;
        fx->life = -1;
        fx->hit = COMBAT_FX_HIT_NONE;
        fx->seed = p->seed;
        fx->speed = p->speed;
        fx->trail_count = p->trail_count;
        for (int k = 0; k < p->trail_count; k++)
            memcpy(fx->trail[k], p->trail[k], sizeof fx->trail[k]);
        memcpy(fx->start, p->trail[0], sizeof fx->start);
        fx->end[0] = p->x;
        fx->end[1] = p->y;
        fx->end[2] = p->z;
    }
    for (int i = 0; i < COMBAT_DEPLOY_MAX && n < cap; i++) {
        const CombatDeployed *o = &s_deployed[i];
        if (!o->active)
            continue;
        CombatFx *fx = &out[n++];
        memset(fx, 0, sizeof *fx);
        fx->active = 1;
        fx->type = COMBAT_FX_DEPLOYED;
        fx->weapon_class = deployed_fx_class(o->kind);
        fx->damage = o->damage;
        fx->age = o->age;
        fx->life = o->lifetime;
        fx->hit = COMBAT_FX_HIT_NONE;
        fx->deploy_kind = o->kind;
        /* Gameplay proximity stays private to CombatDeployed. Presentation
         * receives only authored object identity and pose. */
        fx->seed = o->seed;
        snprintf(fx->ordnance_model, sizeof fx->ordnance_model, "%s",
                 o->ordnance_model);
        fx->start[0] = fx->end[0] = o->x;
        fx->start[1] = fx->end[1] = o->y;
        fx->start[2] = fx->end[2] = o->z;
    }
    return n;
}

int combat_target_marker(double out[3])
{
    /* The same deterministic forward-cone target candidate used by the
     * visible in-cone entity), minus the cooldown/ammo gates: this is
     * what the target marker rides, whether or not a shot is ready. */
    CombatWeapon *weapon = selected_weapon();
    if (s_user < 0 || !s_have_upose || !weapon || !s_resolve ||
        weapon->deploy_kind != CAR_DEPLOY_NONE)
        return -1;
    CombatEnt *u = ent_at(s_user);
    if (!u || !u->alive)
        return -1;
    double dir = weapon->rear ? -1.0 : 1.0;
    double fx = -sin(s_uyaw) * dir, fz = cos(s_uyaw) * dir;
    double fire_range = player_weapon_range(weapon);
    int best = -1;
    double best_d = 0.0;
    for (int i = 0; i < COMBAT_MAX_ENTS; i++) {
        CombatEnt *e = &s_cents[i];
        double p[3], dx, dz, d, ca;
        if (!e->used || !e->alive || e->hidden || !e->has_obj ||
            i == s_user || !hostile_to(e, u->team))
            continue;
        s_resolve(i, p);
        dx = p[0] - s_ux;
        dz = p[2] - s_uz;
        d = sqrt(dx * dx + dz * dz);
        if (d < 1e-6 || d > fire_range)
            continue;
        ca = (fx * dx + fz * dz) / d;
        if (ca < COMBAT_FIRE_CONE_COS)
            continue;
        if (best < 0 || d < best_d) {
            best = i;
            best_d = d;
        }
    }
    if (best >= 0 && out)
        s_resolve(best, out);
    return best;
}
