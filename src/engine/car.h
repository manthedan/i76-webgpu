#ifndef CAR_H
#define CAR_H

/*
 * car.h — M4 car module: VCF/VDF/WDF/GDF config-chain parser + 20 Hz
 * vehicle sim per docs/specs/m4/ghidra-physics.md §6 (the nitro.exe
 * Ghidra extraction; supersedes the physics-spec.md Part II placeholder
 * model, whose UNKNOWNs it answers).
 *
 * The 20 Hz fixed step is load-bearing (physics-spec §1: the original sim
 * is frame-coupled at ~20 FPS). Per-tick order: input levels (§Q13/§Q17)
 * -> 4-speed auto + quadratic torque powertrain (§Q5) -> per-wheel
 * terrain raycast + mesh-aware chassis contact (§Q10) -> drive/brake/drag
 * forces (§Q5/§Q14/§Q7) under the friction-budget steering constraint
 * (§Q17) -> kick-drift integration with the original's 1/2*a*dt^2 term
 * (§Q1) and explicit airborne branch (§Q2) -> [collision is the
 * M4-collision slice, not this module] -> terrain-aligned attitude
 * (§Q10) -> landing events (§Q2, bridge-visible). Every constant is FACT
 * (Ghidra-read, address-cited) or DECISION-tagged; see car.c.
 *
 * Config chain (entities.md §3): OBJ car label -> "<label>.vcf" (VCFC) ->
 * VDF chassis (VDFC: mass/size/COLP AABBs/WLOC wheels; VGEO body parts),
 * 3x WDF wheels (WDFC), SPEC specials, WEPN weapon mounts (each a GDF).
 * All files are BWD2 chunk streams read through the engine VFS (call
 * fs_set_root() + vfs_init() first). Field layouts used here are CONFIRMED
 * per the specs. WDF field meanings and compnent.cdf brake/suspension
 * semantics remain unnamed; the engine curve consumes the authored CDF
 * Tpeak and executable-derived k, with the old fit only as an absent/malformed
 * CDF fallback (docs/specs/re/engine-curve-consumption.md).
 *
 * Conventions (match terrain.h / scene.c D4):
 *   World units are meters, absolute mission coordinates; +x east, +z
 *   north, +y up.
 *   yaw: heading, radians. forward = (-sin yaw, 0, cos yaw), right =
 *   (cos yaw, 0, sin yaw) — right-handed (right x up = forward). yaw 0
 *   faces +z (north); positive yaw turns left on a north-up map; the
 *   "left" input increases yaw.
 *   pitch: positive = nose up. roll: positive = right side up.
 *   Pose y is the car model origin's world height. While in ground
 *   contact it sits one kinematic ride offset (CAR_RIDE_H, §Q10) above
 *   the chassis probe: registered class-11/12/13 static mesh faces first,
 *   terrain fallback otherwise. Per-wheel probes remain heightfield-only.
 *   Contact is a one-sided constraint, never a hard clamp of the whole model
 *   (physics-spec §8.3-8.4; M3's always-terrain-bound divergence D6 retired).
 *
 * The public car_* surface owns player context 0. Explicit opaque solver
 * contexts support multi-instance integration without presentation ownership;
 * Stage 3 gives scoped synthetic/melee and combat-FOLLOW hosts authority.
 * Pure C11, no platform deps: the web driver polls the player wrappers.
 */
#include <stddef.h>
#include <stdint.h>

#define CAR_SIM_HZ 20
#define CAR_DT (1.0 / (double)CAR_SIM_HZ)

/*
 * Inputs for one 20 Hz tick (physics-spec §4.1 input model).
 *
 * Held/analog channels (levels, re-supplied every tick; the web drive
 * buttons feed 0 or 1, tape scripts feed 0..1 / -1..1 floats):
 *   throttle — throttle notch position 0..1; 1.0 = full notch. The
 *              original's notched tap-to-increment semantics (step size,
 *              persistence, decay UNKNOWN — spec Q13) live with the
 *              CALLER: the sim reads the level each tick, so a tape script
 *              expresses notch sequences as stepped values (D19).
 *   brake    — brake pedal 0..1. Does NOT double as reverse (M3
 *              divergence D4 retired): reverse is the toggle below.
 *   left/right — steer buttons/axes 0..1 each; the steer target is
 *              (left - right) in [-1, 1], +1 = full left.
 *
 * Held action bit (level — set to exactly 1 while held; other values
 * are ignored):
 *   e_brake  — handbrake: while held, effective throttle forced to -1
 *              plus all-wheel velocity-opposing brake (FACT §Q14:
 *              type-1 held action, no latch — release turns it off).
 *
 * Edge-triggered action bits (set to exactly 1 to request the action
 * once; other values are ignored):
 *   shift_up / shift_down — manual gearbox one gear up/down (4-speed
 *              auto, FACT §Q5); suppresses the auto box for 1.5 s.
 *   reverse  — flips the direction sign, NO speed gate (FACT §Q13;
 *              the old |v|<1 m/s gate D21 is retired); the box holds
 *              gear 1 while engaged.
 * car_step CONSUMES the action bits (e_brake included): it clears them
 * in the caller's struct before returning, so a reused struct cannot
 * double-fire and a held e_brake MUST be re-asserted every tick.
 * car_step never sets them itself, so the struct MUST be
 * zero-initialized (`CarInput in = {0};`) — uninitialized action bits
 * are undefined input (the == 1 test makes stray values inert, but
 * that is defense, not contract).
 */
typedef struct {
    float throttle;
    float brake;
    float left;
    float right;
    int   shift_up;
    int   shift_down;
    int   e_brake;
    int   reverse;
} CarInput;

/*
 * Live state (physics-spec §7 state vector, damage/specials deferred).
 * DECISION (M5 save/resume, approved by Main): this typedef lives in the
 * header -- moved verbatim out of car.c -- so save.c can pack the full
 * state vector field-by-field (explicit LE writes, no struct memcpy on
 * the wire). car_get_live/car_set_live below are the additive snapshot
 * hooks the save module round-trips through; no existing path changes
 * behavior.
 */
typedef struct {
    /* pose */
    double x, y, z;                 /* world meters; y = model origin    */
    double yaw, pitch, roll;        /* radians, car.h conventions        */
    /* velocity, body frame: vx lat (+right), vy vert (+up), vz fwd      */
    double vx, vy, vz;
    double yaw_rate, pitch_rate, roll_rate;   /* rad/s                   */
    /* controls / powertrain (§Q5/§Q13/§Q14) */
    double throttle_pos;            /* signed effective throttle, -1..1  */
    double steer;                   /* applied steer input, +1 = left    */
    int    gear;                    /* 1..4 (FACT §Q5 4-speed)           */
    int    reverse;                 /* direction sign flipped (§Q13)     */
    int    handbrake;               /* e-brake held on this tick (§Q14)  */
    double shift_hold;              /* s of auto-box suppression left    */
    double engine_rpm;              /* FACT §Q5 model: 1050..6000        */
    int    ignition;                /* always 1 (D17)                    */
    /* tires/contact bookkeeping */
    double steer_angle;             /* applied front steer angle, rad    */
    double a_long_prev;             /* REPURPOSED (D18 retired): airborne
                                     * fall-apex tracker (§Q2 ent+0x474) */
    double comp_prev[6];            /* per-wheel contact depth, m (§Q10) */
    int    grounded[6];             /* per-wheel contact flags           */
    int    grounded_count;
    double wheel_w[6];              /* wheel angular vel, rad/s (cosm.)  */
    double t;                       /* sim time, s                       */
} CarLive;

/*
 * car_get_live(out) / car_set_live(in)
 *   Full live-state snapshot/restore (M5 save/resume; DECISION: additive
 *   for save.c). Plain struct copies of the module-static state; NULL is
 *   a no-op. car_set_live does not reload config: restoring onto a
 *   different car config than the snapshot was taken from is a caller
 *   error (the save format stores state only, not config identity).
 */
void car_get_live(CarLive *out);
void car_set_live(const CarLive *in);

/*
 * Read-only telemetry for the most recent car_step(). This is the diagnostic
 * seam used by deterministic jump/contact probes; no field feeds back into
 * simulation. `terrain_v_pre_vy_cap` is after CAR_LAUNCH_MAX and immediately
 * before D24c's true-grade cap, while `terrain_v_post_vy_cap` is the value the
 * follow/landing branch receives. `lift_energy_j` is the D24b potential-energy
 * charge m*g*dy; collider_index is -1 unless an OBB response actually ran.
 * `collider_candidates` proves the installed table was walked this step;
 * `collider_index` remains -1 unless an OBB actually resolved contact.
 * H-UAT-070a adds start/integrated/resolved poses, the marked mountain-wall
 * response, and exact per-wheel height/penetration observations so the
 * real-data P01/P02 regression can emit one complete row per 20 Hz tick.
 */
typedef struct {
    int valid;
    double start_x, start_y, start_z;
    double integrated_x, integrated_z;
    double resolved_x, resolved_y, resolved_z;
    int start_on_face;
    int end_on_face;
    double ground_y;
    double terrain_rate;
    double terrain_v_raw;
    double terrain_v_pre_vy_cap;
    double vy_cap;
    double terrain_v_post_vy_cap;
    int launch_cap_applied;
    int vy_cap_applied;
    int follow_branch;
    int ballistic_branch;
    double lift_dy;
    double lift_dv2;
    double lift_energy_j;
    int collider_candidates;
    int collider_index;
    int terrain_wall_hit;
    double terrain_wall_t;
    double terrain_wall_rise;
    double terrain_wall_nx, terrain_wall_nz;
    double terrain_wall_removed;
    int wheel_present[6];
    double wheel_x[6], wheel_z[6];
    double wheel_terrain_y[6];
    double wheel_penetration[6];
} CarStepDiag;

void car_get_step_diag(CarStepDiag *out);

/*
 * Explicit solver contexts (H-UAT-078c Stage 1).
 *
 * These own one immutable-after-load configuration, one CarLive vector, and
 * every mutable transient solver field. They intentionally expose no body,
 * cockpit, weapon-mesh, texture, or attachment cache. Existing public car_*
 * functions below are compatibility wrappers around player context 0.
 *
 * Context allocation/load is setup work; car_context_step performs no
 * allocation. Mission/AI callers keep contexts in stable entity order.
 */
typedef struct CarSimContext CarSimContext;
struct CarCollider;

CarSimContext *car_context_create(void);
void car_context_destroy(CarSimContext *ctx);
int  car_context_load(CarSimContext *ctx, const char *vcf_name);
void car_context_unload(CarSimContext *ctx);
int  car_context_is_loaded(const CarSimContext *ctx);
void car_context_place(CarSimContext *ctx, double x, double z, double yaw);
void car_context_set_scripted_pose(CarSimContext *ctx, double x, double z,
                                   double yaw, double speed);
void car_context_step(CarSimContext *ctx, CarInput *in);
void car_context_get_live(const CarSimContext *ctx, CarLive *out);
void car_context_set_live(CarSimContext *ctx, const CarLive *in);
void car_context_get_step_diag(const CarSimContext *ctx, CarStepDiag *out);
/* Byte-canonical hash of config + live + every mutable transient solver field;
 * borrowed collider addresses are excluded while their ordered values are
 * included. Presentation/cache data is intentionally outside this digest. */
uint64_t car_context_state_hash(const CarSimContext *ctx);
uint64_t car_state_hash(void); /* player context 0 wrapper */

/* Read-only fleet-cost counters. They are process-global diagnostics because
 * the fixed-order solver is single-threaded; they never feed simulation and
 * are excluded from context hashes. `terrain_height_queries` counts direct
 * height-field samples issued by car.c, and `static_candidates` counts each
 * registered CarCollider considered by the broadphase loop. Reset/read are
 * setup/measurement operations, not part of ordinary per-context state. */
typedef struct {
    uint64_t terrain_height_queries;
    uint64_t static_candidates;
} CarPerfCounters;
void car_perf_counters_reset(void);
void car_perf_counters_get(CarPerfCounters *out);

void car_context_set_colliders(CarSimContext *ctx,
                               const struct CarCollider *list, int n);
void car_context_set_bounds(CarSimContext *ctx, double x0, double z0,
                            double x1, double z1);
void car_context_apply_grip_loss(CarSimContext *ctx, double scale, int ticks);
int  car_context_grip_loss_ticks(const CarSimContext *ctx);
double car_context_effective_grip(const CarSimContext *ctx);
double car_context_collision_radius(const CarSimContext *ctx);
/* Physical-owner world-XZ velocity seam. D-C20 adds one explicitly marked
 * external velocity impulse here; D-A11 reads the same owner when resolving
 * its single separation/zero-restitution convention. */
void car_context_world_velocity(const CarSimContext *ctx,
                                double *vx, double *vz);
void car_context_add_world_velocity(CarSimContext *ctx,
                                    double dvx, double dvz);
void car_context_apply_vehicle_contact(CarSimContext *ctx,
                                       double dx, double dz,
                                       double nx, double nz,
                                       double other_vx, double other_vz);
void car_context_landing_events(const CarSimContext *ctx,
                                int *hard_landings, int *fall_resets,
                                double *last_impact);
double car_context_last_impact_scale(const CarSimContext *ctx);

/*
 * car_load(vcf_name)
 *   Load the car config chain. `vcf_name` is the VCF as the VFS knows it
 *   (e.g. "vdrampg2.vcf"; the ".vcf" extension is appended when absent).
 *   VCF and VDF are required (returns -1 when either is missing or
 *   malformed); missing/short WDF/GDF files degrade to placeholder values
 *   and the car still loads. Resets the live state; use car_place() to
 *   spawn. Returns 0 on success, -1 on failure.
 */
int car_load(const char *vcf_name);

/*
 * car_settled_y_at(x, z)
 *   The exact terrain/drivable-surface probe and kinematic ride offset used
 *   by car_place(). Vehicle spawn owners use this to reject authored poses
 *   below the surface without duplicating the player's placement rule.
 */
double car_settled_y_at(double x, double z);

/*
 * car_place(x, z, yaw)
 *   Set spawn position/heading (mission ODEF supplies these). The car
 *   starts settled on its suspension at rest ride height with zeroed
 *   velocities and neutral controls (gear 1, no reverse, no handbrake).
 *   Safe to call any time after car_load.
 */
void car_place(double x, double z, double yaw);

/*
 * car_set_scripted_pose(x, z, yaw, speed)
 *   Apply one fixed tick of an FSM-owned kinematic driving segment to the
 *   loaded physical car. Position is terrain-settled, attitude follows the
 *   terrain, and body velocity is made consistent with the supplied forward
 *   speed so normal car_step() can resume at handoff. No-op when unloaded.
 *
 *   This is deliberately narrower than car_set_live(): scripts own pose and
 *   speed, not the vehicle config or persistent damage/powertrain state.
 */
void car_set_scripted_pose(double x, double z, double yaw, double speed);

/*
 * car_step(in)
 *   Advance the sim exactly one fixed 20 Hz tick (CAR_DT), running the
 *   physics-spec §7 nine-step order. `in` may be NULL (= all zeros).
 *   The action bits — shift_up/shift_down/reverse edge triggers and
 *   the held e_brake level — are consumed: cleared in *in before
 *   return. No-op when no car is loaded.
 */
void car_step(CarInput *in);

/* Current pose. Any out-pointer may be NULL. */
void car_pose(double *x, double *y, double *z,
              double *yaw, double *pitch, double *roll);

/* Signed forward speed in m/s (negative while reversing). */
double car_speed(void);

/* Current drive direction: nonzero while the reverse toggle is engaged. */
int car_is_reverse(void);

/* Current gear (1..4) and engine RPM (FACT §Q5 model state). */
int    car_gear(void);
double car_rpm(void);

/* Loaded quadratic engine curve. `component_id` is the selected one-based
 * compnent.cdf row; `authored` is nonzero only when validated CDF bytes
 * supplied Tpeak/k (zero means the marked D2 fallback). Any out may be NULL.
 * Returns 0 while a car is loaded, -1 otherwise. */
int car_engine_curve(uint32_t *component_id, double *tpeak, double *k,
                     int *authored);

/*
 * engsnd.dat ENG NUM selected by the loaded VDF vehicle-size class:
 * 1 small -> 2, 2 medium -> 1, 3 large -> 0, 4 van -> 3,
 * 5 heavy -> 5, 6 tank -> 4. Returns -1 when no car is loaded or the
 * original size class is unknown.
 */
int car_engine_sound_number(void);

/*
 * car_landing_events(hard_landings, fall_resets, last_impact)
 *   Landing event counters since the last car_load/car_place (FACT §Q2
 *   thresholds: hard landing = vertical impact > 7.65 m/s @0x4c1d58;
 *   fall-reset = fall apex 35 m above the landing ground @0x4c1d44).
 *   The host bridge consumes hard landings for presentation and port damage;
 *   fall resets remain diagnostic. The original's per-part landing damage
 *   and reset-to-road behavior remain incomplete. Any out-pointer may be
 *   NULL. The counters are not serialized and never feed the simulation.
 */
void car_landing_events(int *hard_landings, int *fall_resets,
                        double *last_impact);

/*
 * car_last_impact_scale()
 *   Surface impact/damage scale of the tile under the last hard landing
 *   (FACT §Q15: WRLD surface-table record +0x10, int32; the original's
 *   landing path FUN_00424da0 multiplies it into the FUN_00463370 damage
 *   call). 1.0 before any hard landing or when no mission surface table
 *   covers the landing tile. Read-only: the port's landing-damage routing
 *   (combat.c D-C7) is unchanged; this exposes the factor the damage owner
 *   would apply without rerouting ownership. Canonical authored values
 *   span 0 (road classes) to 999999999.
 */
double car_last_impact_scale(void);

/* MARKED PORT CONVENTION: deployed-object behavior is selected only for the
 * six shipped dropper GDFs. The binary's ORDF types corroborate that these are
 * distinct deployers, but their complete runtime state machines are not yet
 * decoded (docs/specs/re/tooling-roadmap.md). */
enum {
    CAR_DEPLOY_NONE = 0,
    CAR_DEPLOY_OIL,
    CAR_DEPLOY_FIRE,
    CAR_DEPLOY_MINE,
    CAR_DEPLOY_CALTROPS,
    CAR_DEPLOY_BLOX,
    CAR_DEPLOY_ERASER,
    CAR_DEPLOY_COUNT
};

/*
 * Parsed VCF->VDF HLOC->GDF weapon contract. `direct_fire` retains its legacy
 * name but means a playable hardpoint: positive-damage top/side/turret/inside
 * ordnance OR one of the six decoded class-4 droppers. Droppers never enter
 * the direct-fire/projectile gate. Firing cadence is GDFC +78 converted to
 * 20 Hz ticks. String pointers are module-owned until car_unload().
 */
enum {
    CAR_WEAPON_LINK_NONE = 0,
    CAR_WEAPON_LINK_SLUG,
    CAR_WEAPON_LINK_SPP,
    CAR_WEAPON_LINK_FLAME,
    CAR_WEAPON_LINK_MORTAR,
    CAR_WEAPON_LINK_DROPPER,
};

typedef struct {
    const char *name;
    const char *sound;
    /* Audit/probe metadata from the same parsed WEPN/GDF/HLOC chain. Empty
     * strings and -1 mean there is no mounted-car context (the standalone
     * car_gdf_weapon_info query). */
    const char *gdf;
    const char *fire_sprite;
    char ordnance_model[9];      /* caller-owned OGEO projectile/deployer */
    /* ORDF's four target-class effect prototypes: ground, car, and two
     * structure-class groups.
     * Empty/"null" means the native weapon requests no impact art. */
    char impact_ground[14];
    char impact_car[14];
    char impact_building[14];
    char impact_structure[14]; /* native classes 3/0x0b, ORDF +111 */
    int         mount;
    int         mount_class;       /* HLOC mesh type: 1 top, 2 side,
                                      3 turret, 4 drop, 5 inside */
    int         geometry_parts;    /* selected GGEO class after HLOC compose */
    int         damage;
    int         ammo;
    int         cooldown_ticks;
    int         burst_cooldown_ticks;
    int         rear_facing;
    int         direct_fire;
    int         deploy_kind;       /* CAR_DEPLOY_*; class-4 only     */
    int         turreted;          /* decoded effective-class-3 link ban */
    int         link_class;        /* CAR_WEAPON_LINK_* manual taxonomy */
    double      projectile_speed; /* GDFC +86 legacy/presentation speed */
    double      flight_speed;     /* ORDF +4 native ordnance speed       */
    int         ordnance_type;    /* ORDF +0 flight/collision dispatcher */
    int         manager_type;     /* ORDF +12 FUN_00401610 branch        */
    int         traverses;        /* native turret variant discriminator */
    int         tracks;           /* ORDF 3/8/0x14 target link           */
    double      range_m;          /* published/family-gate range         */
    /* Raw GDFC +90 category id (+100 rear). Retained for decode/audit;
     * it is not the linking identity because it groups unlike tiers. */
    int         weapon_group;
    int         family;             /* GDFC +16, turret pitch exception */
    int         tier;               /* GDFC +20, effective class override */
} CarWeaponInfo;

int car_weapon_count(void);
int car_weapon_get(int i, CarWeaponInfo *out);

/*
 * Side-effect-free combat subset of a purchaser VCF. Mission combat needs
 * authored defense and NPC loadouts for every placed car, while car_load()
 * intentionally targets physical player context 0. This query walks the
 * same VCFC -> VDF HLOC -> WEPN/GDF fields without replacing that player.
 *
 * Armor/chassis order is VCF F/L/R/B. Weapons contain playable positive-
 * damage top/side/turret/inside mounts plus the six recognized class-4
 * droppers (including zero-damage Oil), in WEPN order. Strings live in `out`.
 */
#define CAR_COMBAT_WEAPONS 8
typedef struct {
    char          name[17];
    int           damage;
    int           ammo;
    int           cooldown_ticks;
    int           burst_cooldown_ticks;
    int           rear_facing;
    int           fire_amount;
    int           family;              /* GDFC +16 inventory family       */
    int           tier;                /* GDFC +20; >=100 forces class 3   */
    int           ammo_capacity;       /* immutable GDFC +94 rate input    */
    double        projectile_speed;     /* GDFC +86 fallback/presentation   */
    double        flight_speed;         /* ORDF +4 native ordnance speed    */
    int           ordnance_type;        /* ORDF +0 flight dispatcher        */
    int           manager_type;        /* ORDF +12 AI decision branch      */
    int           traverses;           /* native turret variant discriminator*/
    int           deploy_kind;         /* CAR_DEPLOY_*; never projectile fire */
    char          ordnance_model[9];   /* OGEO authored model name             */
    char          impact_ground[14];   /* ORDF +33 XDF, terrain/world contact  */
    char          impact_car[14];      /* ORDF +59 XDF, vehicle contact        */
    char          impact_building[14]; /* ORDF +85 XDF, classes 2/0x0c         */
    char          impact_structure[14];/* ORDF +111 XDF, classes 3/0x0b        */
    double        turret_yaw;          /* runtime relative horizontal yaw  */
    double        turret_pitch;        /* runtime child-joint elevation     */
    int           turret_yaw_on_target;   /* native 2-degree threshold       */
    int           turret_pitch_on_target; /* native 2-degree threshold       */
    double        mount_frame[12];     /* HLOC parent frame, car local      */
    double        muzzle_frame[12];    /* HLOC o GPOF frame, car local      */
    double        muzzle[3];           /* HLOC o GPOF position, car local   */
    double        range_m;             /* published/family-gate range       */
} CarCombatWeapon;

typedef struct {
    uint32_t       armor[4];
    uint32_t       chassis[4];
    double         collision_half[3]; /* VDF COLP authored X/Y/Z half extents */
    CarCombatWeapon weapons[CAR_COMBAT_WEAPONS];
    int            weapon_count;
} CarCombatConfig;

int car_combat_config(const char *vcf_name, CarCombatConfig *out);

/* Mounted GGEO parts and GPOF muzzle, already composed through the selected
 * HLOC into car-model space (r/u/f/position columns). */
int         car_weapon_part_count(int weapon);
const char *car_weapon_part_name(int weapon, int part);
int         car_weapon_part_frame(int weapon, int part, double out12[12]);
/* Runtime presentation frame. Effective-class-3 parts follow combat's live
 * horizontal attachment traversal; fixed mounts equal part_frame(). */
int         car_weapon_part_live_frame(int weapon, int part, double out12[12]);
int         car_weapon_mount_frame(int weapon, double out12[12]);
int         car_weapon_muzzle_frame(int weapon, double out12[12]);

/*
 * Exterior parts for the drive view: VGEO damage-state-0 body parts
 * followed by WDF WGEO wheel meshes placed at WLOC frames (car.c D23).
 * The caller maps car_part_name(i) to a mesh and draws it at
 * car_pose * car_part_frame(i).
 *
 * car_part_count:      total exterior parts (body + wheels; 0 if unloaded).
 * car_body_part_count: VGEO body-only count (stable pin for probes).
 * car_wheel_part_count: appended wheel count (0..6 typically).
 * car_part_name:       part i name, or NULL on a bad index. Module-owned
 *                      until car_unload().
 * car_part_frame:      part i's frame in car model space (parent chains
 *                      composed; scene.c D4 convention — basis columns
 *                      right/up/forward) as out12 = r[3], u[3], f[3],
 *                      pos[3]. 0 ok, -1 bad index or NULL out.
 * Body indices are [0, car_body_part_count()); wheels follow.
 */
/* Paint-scheme file (.vtf) named by the VCF, "" when none. The renderer
 * needs it to resolve a GEO face's 'V...' placeholder texture name through
 * the VTFC/TMT chain — see docs/specs/m6/oeg-face-format.md. */
const char *car_vtf_file(void);

/* VCFC/VDFC display strings (module-owned until car_unload). Empty if none. */
const char *car_chassis_name(void);   /* e.g. "Dover Rampage" */
const char *car_variant_name(void);   /* e.g. "Mark 2000" */

/* Read-only garage/configuration fields from the loaded purchaser VCF/VDF.
 * These expose authored inventory labels and values without making the shell
 * parse a second copy of the binary formats. IDs index compnent.cdf's engine,
 * suspension, and brake tables. Defense order is front/left/right/rear;
 * values are the VCF's raw tenths. Wheel names are the three axle WDF files.
 * Returned strings are module-owned until car_unload(). */
int         car_component_ids(uint32_t out3[3]);
int         car_defense(uint32_t armor4[4], uint32_t chassis4[4],
                        uint32_t *left_to_add);
double      car_config_weight_lb(void);
int         car_special_count(void);
int         car_special_id(int i);
const char *car_wheel_file(int axle);

int car_part_count(void);
int car_body_part_count(void);
int car_wheel_part_count(void);
const char *car_part_name(int i);
int car_part_frame(int i, double out12[12]);

/*
 * First-person (interior) parts from the VDF VGEO chunk — the part set at
 * index 16, i.e. after the 4 damage-state sets and the 12 LOD sets of the
 * entities.md §3.2 layout (Nitro VGEO carries 28 sets of numParts 100-byte
 * records; set 16 is the first set whose names are interior roles —
 * DASH/SWHL/SEAT/MIRI/RADR/GER6/CMP3/CMP6/SYS3+6/WEP3+6/SPC3+6/GUNL+GUNR —
 * verified across every drivable Nitro VDF; see car.c D22). These are the
 * authored cockpit surfaces the first-person view draws at
 * car_pose * car_interior_part_frame(i); the render owner resolves each
 * name to a mesh exactly like car_part_name(i).
 *
 * Naming roles are purchaser-data observations: trailing '3'/'6' pairs
 * (CMP3/CMP6, SYS3/SYS6, WEP3/WEP6, SPC3/SPC6) and RTC1/RTC6 are exported
 * as-is; their day/night-vs-resolution semantics are UNRESOLVED, so no
 * variant is selected here — the consumer picks with its own evidence.
 *
 * car_interior_part_count: number of non-NULL-named interior parts.
 * car_interior_part_name:  name i (module-owned, valid until car_unload),
 *                          or NULL on a bad index.
 * car_interior_part_frame: part i's frame in car model space, same out12
 *                          convention as car_part_frame. 0 ok, -1 bad.
 *
 * car_interior_mirror_part: index of the interior-mirror part (name suffix
 *                          "MIRI"; some cars also carry "MIRL"/"MIRR"
 *                          side mirrors in the same set), or -1 when the
 *                          loaded vehicle has none. This is the authored
 *                          mirror surface a rearview pass composites onto;
 *                          the authored 2-D mask art (zmiri*.map) is a HUD
 *                          asset — see hud_mirror_mask().
 */
int car_interior_part_count(void);
const char *car_interior_part_name(int i);
int car_interior_part_frame(int i, double out12[12]);
int car_interior_mirror_part(void);

/*
 * VDF VLOC attachment locators (entities.md §3.2: u32 number + r/u/f/pos
 * frame; record semantics INFERRED). Empirically consistent across the
 * Nitro vehicle set: number 40 sits at the left-seat head position
 * (driver-eye candidate), 42 near the cabin floor centre, 38 at the front
 * body tip, and 35/36 are a windshield-height angled pair. No meaning is
 * enforced by the sim; consumers map numbers with their own evidence.
 *
 * car_vloc_count: number of VLOC records (0 when no car is loaded).
 * car_vloc_get:   locator i — its record number (any NULL out-pointer
 *                 skipped) and car-model-space frame as out12. 0 ok,
 *                 -1 on bad index.
 */
int car_vloc_count(void);
int car_vloc_get(int i, uint32_t *number, double out12[12]);

/*
 * Standalone GDF weapon-stat query, independent of the loaded car's WEPN
 * list — same GDFC layout as the mounted-weapon parse (entities.md §3.3).
 * Exists for weapons the data defines but no stock VCF mounts (the pilot
 * sidearm gh45.gdf). `out` fields come from the GDF only: rear_facing is
 * 0 and direct_fire is damage > 0 (there is no HLOC mount context; the
 * caller owns mount/direction semantics). String pointers are module-owned
 * SINGLE-SLOT storage, valid until the next car_gdf_weapon_info call —
 * unlike car_weapon_get's pointers, which live until car_unload().
 * Returns 0 on success, -1 when the GDF is missing or has no GDFC chunk.
 */
int car_gdf_weapon_info(const char *gdf_name, CarWeaponInfo *out);

/*
 * Object collision (spec §7 step 7; D8's "future collision pass").
 *
 * The sim deliberately does NOT depend on scene.c — car.c is linked by
 * probes that never load a world, and a scene.c edge would drag the mesh
 * and mission stack into all of them. Instead the world layer REGISTERS
 * horizontal bounding circles and the sim resolves against them:
 *
 *   car_set_colliders(list, n)  — borrow a caller-owned array, valid until
 *                                 replaced or cleared. n <= 0 (or NULL)
 *                                 clears, which is the default: with no
 *                                 colliders registered the step is a no-op
 *                                 and motion is bit-identical to before,
 *                                 so the tier-3 sim tapes are unaffected.
 *
 * Resolution is a horizontal push-out to the obstacle boundary with the
 * inward component of velocity removed (the car slides along the contact
 * rather than sticking). Vertical motion is untouched — ground contact is
 * still the terrain constraint (D6).
 *
 * A centre already INSIDE the box (a fast tick that tunnels past the
 * contact band, a spawn point overlapping scenery) cannot use the
 * closest-point normal — it degenerates — and leaving the car embedded
 * lets it drive out through the far side. It separates along the NEAREST
 * FACE instead (minimum translation, deterministic +x/-x/+z/-z tie
 * order), with the same inward-velocity removal, consistent with
 * car_apply_vehicle_contact().
 *
 * D19 — colliders are ORIENTED BOXES WITH A HEIGHT SPAN, not circles.
 * The first cut used a horizontal bounding circle, which forced a 20 m
 * radius cap: a circle around anything large or elongated bulges far past
 * the geometry, and the measured case was the bewdbrg2 bridge (30.5 m
 * circle) stopping the car ~67 m short of a structure you drive toward.
 * Everything above the cap was simply not an obstacle, so the car drove
 * through buildings. A box hugs the real footprint, so the cap is gone and
 * large scenery is solid.
 *
 * The height span is what keeps the box honest vertically: an object only
 * blocks when its [y0,y1] overlaps the band the car body occupies, so the
 * car passes under a raised span instead of hitting its bounding volume.
 * Registered class-11/12/13 upward mesh faces now supersede the associated
 * coarse box's top contact at matching chassis height. They also own the
 * top-face handoff across box overhangs: a chassis inside or above the
 * native three-metre band at the nearest real top-face edge clears the
 * approximation rather than hitting a mid-air side wall. Ground-level side
 * and underside contacts below that band retain this OBB
 * backstop. See terrain_drivable_probe and H-UAT-041/050/058.
 */
typedef struct CarCollider {
    double x, z;      /* world centre of the footprint, meters           */
    double hx, hz;    /* half-extents along the box's OWN axes, meters   */
    double ax, az;    /* unit world-XZ vector: the box's local +x axis   */
    double y0, y1;    /* world vertical span (y0 <= y1)                  */
    int drivable_object; /* own exact-top token; zero = solid sibling   */
 int drivable_parent; /* parent token for scoped nearby face probes  */
} CarCollider;

/* Vertical band the car body occupies above its ground contact, meters. */
#define CAR_MODEL_ORIGIN_H 0.08
#define CAR_COLLIDE_H 2.0

void car_set_colliders(const CarCollider *list, int n);

/* Oil-slick gameplay feeds the existing authored surface-grip multiplier,
 * rather than a second tire-force model. `scale` is clamped to (0,1], the
 * longest requested bounded duration wins, and car_load clears it. MARKED
 * PORT CONVENTION pending native flag-0x400 duration/scale decode. */
void   car_apply_grip_loss(double scale, int ticks);
int    car_grip_loss_ticks(void);
double car_effective_grip(void);

/*
 * Dynamic vehicle-contact primitives. The world/mission layer owns live
 * vehicle membership and pair ordering; car.c owns only the physical player.
 *
 * car_collision_radius() is the loaded chassis' COLP outer horizontal extent
 * (the same circle used against static OBBs), with the existing 1 m fallback.
 *
 * car_apply_vehicle_contact() applies one horizontal positional correction
 * and removes only the player's closing velocity relative to the other body
 * along the supplied outward normal. It does not advance time, settle terrain,
 * reset controls, or touch static colliders. All vectors are world XZ.
 */
double car_collision_radius(void);
void car_world_velocity(double *vx, double *vz);
void car_add_world_velocity(double dvx, double dvz);
void car_apply_vehicle_contact(double dx, double dz,
                               double nx, double nz,
                               double other_vx, double other_vz);

/*
 * Drivable-area bounds, meters (D8's position clamp, narrowed).
 * Default is the full terrain grid [0, CAR_WORLD_MAX_M]^2, i.e. exactly the
 * clamp that was always there — so an unset bound changes nothing and the
 * sim tapes are unaffected. The world layer narrows it to the mission's
 * used terrain extent, past which the heightfield is flat empty and the
 * renderer draws nothing. x1/z1 <= x0/z0 restores the default.
 */
void car_set_bounds(double x0, double z0, double x1, double z1);

/* Multi-line config + live-state summary (snprintf semantics/result). */
int car_stats(char *buf, size_t n);

/* Free loaded config state (including the part table). */
void car_unload(void);

/* Nonzero when a car config is loaded. */
int car_is_loaded(void);

#endif /* CAR_H */
