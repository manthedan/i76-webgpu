/*
 * mission.c — mission runner: terrain + scene + mission FSM (see mission.h)
 *
 * The M7 foundation: hosts the FSM<->game bridge (FsmHost dispatch) that a
 * mission script talks through. Implemented SEMANTICALLY today (validated
 * against miss8/P01.MSN, the first Nitro trip mission — N01.CBT and every
 * other .CBT ship an empty FSM payload, proven by tools/mission_probe.c):
 *
 *   semantic:  null true false set inc dec rand isEqual isGreater isLesser
 *              startTimer timeGreater timeLesser (20 Hz sim-time timers)
 *              cbPrior cbFromPrior cb cbFrom stopCB killCB isCBEmpty
 *              (entity-owned bounded radio queue -> mission_message)
 *              successAll failAllObj failAll (mission state transitions)
 *              isWithin isWithinNav isWithinSqNav (car-pose / ODEF-position
 *              / FSM-path distances)
 *              isDead isAttacked isShot isRammed isGroovesFault isKeypress
 *              hpLesser allEnemyDead ... (constant-0 world predicates:
 *              no combat system exists yet — these are semantically TRUE
 *              answers for an undamageable world, not stubs: when M7
 *              combat lands they get real bodies)
 *              goto follow race teleport teleportOffset sit (AI story
 *              motion through ai.c's kinematic mover — D14)
 *              isArrived isAtFollow setSkill setAgg setAvoid toggleAvoid
 *              setMaxAttackers (ai.c mover/director state — D15/D20)
 *              isDead isAttacked isShot isRammed isGroovesFault hpLesser
 *              ammoLesser allEnemyDead allBlgDead whoAttacked whoShot
 *              whoRammed nearestEnemy attack hide startCar triggerGate
 *              (combat + scene state — combat.c D17, scene.c gate parts)
 *   logged-todo (counted, log-once, safe return): AI/car verbs (behave,
 *              guard, driveControl, ...) and everything else not
 *              listed above. (evade is SEMANTIC: route-follow half of
 *              the original's evade, combat.c/ai.c D-A17.)
 *              cutscene camera verbs (cam* pushCam popCam playMovie) are
 *              SEMANTIC via the cam stack (D18): they set a live eye/target
 *              the host can read through mission_cam_*; camIsArrived is a
 *              self-clearing pulse when a path cam finishes (Open76).
 *
 * The per-action call counts printed by mission_unload() are the M7
 * coverage ledger (docs/specs/m7/fsm-ledger.md).
 *
 * DECISION list (spec gaps + data realities resolved here):
 *  D1  Script source: a sibling "<base>.fsm" VFS file wins when present
 *      (the API contract); otherwise the ADEF/`FSM ` chunk embedded in the
 *      mission file is used. WHY: the probe's sibling survey shows ZERO
 *      sibling .fsm files in the shipped data — every scripted mission
 *      embeds its FSM. A sibling file is treated as a raw seven-table FSM
 *      image (no AREV/ADEF framing) — no such file exists to prove
 *      otherwise; CONFIRM BY: finding one in the wild.
 *  D2  The FSM chunk tag matches on the first 3 bytes ('F','S','M'):
 *      N01.CBT stores "FSM\0", the spec (fsm.md §1) says "FSM " — both
 *      observed conventions are accepted.
 *  D3  mission.c re-walks the mission file's ODEF chunk itself to resolve
 *      FSM entities to world positions (label unpack + class/flags/pos,
 *      scene.md §3.1 field offsets). scene.c already parses ODEF but keeps
 *      its table static and exposes no by-label lookup, and this slice may
 *      not modify scene.c. When M7 grows an entity store this table moves
 *      there. LDEF string objects are not matched (FSM entities reference
 *      cars/statics in all observed missions).
 *  D4  Entity matching: the FSM entity's 8-byte masked object name is
 *      unpacked exactly like scene.c's label_unpack (7-bit ASCII chars,
 *      high bits shifted into an id) and matched case-insensitively on the
 *      name; when several ODEF objects share the name the id bits pick the
 *      instance, falling back to the first name match. CONFIRMED by data:
 *      P01 entity 'tanker1' obj "nomilk3\x80" vs 'tanker2' "nomilk3\0"
 *      select the two 'nomilk3' ODEF records.
 *  D5  The player is the FSM entity labeled "user" (case-insensitive);
 *      fallback: the entity whose ODEF object has flags bit 0x10 set
 *      (P01: both pick vdrampg2, flags=0x10 team=1). The user entity's
 *      position is the live car pose from mission_set_car, NOT its static
 *      ODEF record.
 *  D6  Distances are 2D XZ in meters (y differences are terrain-scale
 *      noise against action radii). isWithinNav/isWithinSqNav measure
 *      from the entity to the path's NODE 0 ONLY — NOT the polyline
 *      (BINARY-VERIFIED, nitro.exe FUN_00417f50/FUN_00417fb0 via the
 *      action dispatch FUN_00413420 cases 0x2a/0x2b; path record +0x54
 *      is the node-array pointer, [0]/[2] = node 0 x/z — teleport
 *      FUN_00406120 reads it the same way): isWithinNav is
 *      dx^2+dz^2 < r^2 (strict), isWithinSqNav is the axis-aligned
 *      square |dx| < r && |dz| < r ("Sq" = square, not squared
 *      distance). This REPLACES the Open76-derived polyline reading
 *      (Open76's CarAI.IsWithinNav generalizes to segments — the
 *      binary does not). isWithin units: meters, matching the FSM path
 *      units (fsm.md §1.1); not centi-units (that conversion is
 *      documented for camera actions only).
 *  D7  Sim-time timers: startTimer writes the current whole second through
 *      its supplied IntRef; timeGreater/timeLesser compare that value plus
 *      authored seconds against fixed-step time with Nitro's -0.5 bias. This
 *      is machine-local whenever bytecode passes a local
 *      stack slot, exactly as Nitro's dispatcher cases 0xd/0x26/0x27 do.
 *      The former mission-global table let unrelated machines reset one
 *      another (P11's staggered opening pair was the first hard repro).
 *  D8  Radio model: a bounded queue (16) of {clip, owner-entity} records;
 *      queueFlag 1 = front (priority), anything else = append (fsm.md
 *      §4.3). cb/cbPrior use owner -1; cbFrom/cbFromPrior retain their
 *      entity operand, matching FUN_00418630/FUN_00418670. One clip plays
 *      at a time for its decoded PCM duration. Missing/invalid audio uses
 *      a 3 s fallback so an incomplete asset set cannot deadlock a mission.
 *      mission_message formats the clip whose playback started last.
 *      killCB destroys the queue (FUN_00419560); stopCB sets a persistent
 *      enqueue block that frees nothing and leaves playback running
 *      (FUN_00418690 -> "STOPCBXX" -> DAT_00523110), re-armed at mission
 *      reset (FUN_00419150). cbFromPrior's owner-death skip remains open.
 *  D18 Cutscene camera stack (fsm.md §4 cam* verbs): a bounded stack of
 *      eye/target poses. pushCam snapshots the current pose; popCam
 *      restores. Camera argument order, centi-unit scaling, path
 *      idempotence, terrain clearance, and target position follow
 *      nitro.exe FUN_00413420 / FUN_004a0750..FUN_004a0d70. Path actions
 *      advance at 20 Hz; camIsArrived returns the binary's self-clearing
 *      completion pulse. playMovie is a no-op semantic (video is the
 *      shell's job) that still marks the stack active so probes can observe
 *      the cutscene window. Without an active push, mission_cam_active() is
 *      0 and the selected driving camera is the host's.
 *  D9  failAllObj/failAll -> MISSION_FAILED and record the reason value in
 *      the message; successAll -> MISSION_COMPLETE. Single-objective
 *      success/fail are logged as events but do NOT flip mission state
 *      (no objective model yet — M7; Tony's traces show failAllObj as the
 *      terminal fail verb, successAll's counterpart role is INFERRED in
 *      fsm.md §4.2).
 *  D10 rand uses a local xorshift32 PRNG with a fixed seed (project
 *      determinism rule — fsm.md §6; the binary's mission RNG seeding is
 *      UNKNOWN). *ref = rng % *maxref when *maxref > 0, else 0.
 *  D11 whoAttacked/whoRammed/whoShot/nearestEnemy/nearestBlg return -1
 *      ("no one"). The encoding of "none" is UNKNOWN; -1 cannot collide
 *      with a valid entity index, which 0 would (entity 0 = user).
 *  D12 Mission file lookup mirrors scene_load's directory fallback order
 *      ("", "miss8/", "miss16/", "missions/") so callers may pass either
 *      "p01.msn" or "miss8/p01.msn".
 *  D13 An FSM whose machines have ALL halted leaves mission_state
 *      untouched (running): script termination is not mission completion.
 *      The ledger dump at mission_unload shows how far the script got.
 *  D14 Story motion: goto/follow/race/teleport/teleportOffset/sit route
 *      into ai.c's kinematic mover (src/engine/ai.c, unity-included
 *      below — see its header for the speed/units DECISIONS). Entities
 *      move in 2D XZ; multiple FSM names resolving to one ODEF object share
 *      the last name's single mover/combat/contact body (P15's hearse aliases
 *      otherwise repelled themselves and attacked as duplicate cars).
 *      ent_pos() resolves through that owner and prefers its AI pose except
 *      for the user (D5 keeps the user on the live car pose; the user's AI
 *      ghost still advances so isArrived(user)/isAtFollow(user) fire for
 *      P01's intro auto-drive). AI-driven owners write their position back to
 *      their scene object every tick; the user is excluded (its on-screen car
 *      is the sim's dynamic mesh).
 *  D15 isArrived/isAtFollow read ai.c mover state (self-clearing pulse /
 *      10 m level per fsm.md §4.2), replacing the constant-0 predicates.
 *  D16 setId READS an entity id into a cell (*args[0] = id of entity
 *      *args[1]) and entity ids INITIALIZE to the entity index — this
 *      REPLACES the M7-foundation's write-direction guess (which stored
 *      *args[1] into s_ents[*args[0]].id with ids defaulting to 0).
 *      Evidence from P01's bytecode: m5/m6/m7 run
 *      `setId(stackLocal, friendlyCell)` then `attack(enemy, stackLocal)`
 *      — only the read direction gives the attack a meaningful target
 *      (the id-lottery picks which friendly the wave chases); and the
 *      strike watchdogs (m20/m21/m22) check
 *      `isEqualId(whoAttacked, friendlyCell)` — with ids defaulting to
 *      the entity index, enemy attackers (ids 5..9) never compare equal
 *      to the friendly cells (values 0..3), while the user (id 0) always
 *      matches. Under the old write guess EVERY enemy hit on a friendly
 *      compared equal (all ids 0) and counted as a player strike — five
 *      enemy hits would fail the mission with zero player input.
 *      CONFIRM BY: binary id-slot initialization at entity spawn.
 *  D17 Combat: the isDead/isAttacked/isShot/isRammed/hpLesser/ammoLesser/
 *      allEnemyDead/allBlgDead/whoAttacked/whoShot/whoRammed predicates,
 *      nearestEnemy, the attack verb and the hide/startCar visibility
 *      store route into combat.c (unity-included below after ai.c — see
 *      combat.h for the model and its D-C* decisions). isGroovesFault is
 *      combat.c D-C17: dead-and-killed-by-the-user attribution (BINARY-
 *      VERIFIED FUN_00417ed0/FUN_00418950), not a pulse on any attack.
 *  D20 Director parameters: setSkill/setAgg/setAvoid/toggleAvoid/
 *      setMaxAttackers write ai.c's per-entity director state using the
 *      binary's one-based skill tables and setAgg call-order reset.
 *      D-A15 consumes steer/throttle as controller response gains rather
 *      than direct top-speed multipliers; combat.c consumes aim/fire as
 *      bounded shot-acceptance parameters.
 *  D21 Exact gate leaf consumed from Phase A-depth: triggerGate resolves
 *      the entity's scene object and clears its first class-7 gate-part state
 *      (FUN_00456270). Open gate parts leave render/export and collision;
 *      the native animation remains unmodeled. Notebook reveal/status now
 *      consumes the native six-entry NPT table (objective-cues.md).
 *
 * D-O1..D-O13 are the FSM-LESS OBJECTIVE CONTROLLER's decisions and live
 * with it, above objectives_init(). They are numbered apart because they
 * govern the missions the FSM does NOT run — the 55 arena maps that could
 * be driven but never won. D-O1..D-O7 are RACE; D-O8..D-O13 are CAPTURE.
 */

#include "engine/mission.h"
#include "engine/camera.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdint.h>
#include <limits.h>
#include <math.h>

#include "engine/vfs.h"
#include "engine/terrain.h"
#include "engine/scene.h"
#include "engine/car.h"
#include "engine/fsm.h"
#include "engine/sound.h"
#include "engine/ai.h"
/* ai.c is unity-included: the mission bridge is its only consumer this
 * milestone, and both web/build.sh and the native probe build lines list
 * mission.c only (see ai.h's build note). Do NOT list ai.c separately. */
#include "engine/ai.c"
/* combat.c is unity-included the same way (D17; combat.h's build note).
 * It sits after ai.c: the death path calls ai_sit, engagements call
 * ai_follow/ai_goal. */
#include "engine/combat.h"
#include "engine/combat.c"

/* ----------------------------------------------------------------------- */
/* Tunables (DECISION-marked where invented)                                */
/* ----------------------------------------------------------------------- */

#define MISSION_CBQ_MAX       16    /* radio queue bound (D8, invented)    */
#define MISSION_CB_FALLBACK_TICKS 60 /* 3 s only when audio is unavailable */
#define MISSION_MSG_MAX       160   /* mission_message buffer              */
#define MISSION_NAME_MAX      48    /* FSM label/clip/path name + slack    */
#define MISSION_CAM_STACK     8     /* cutscene camera push depth (D18)    */
#define MISSION_VEHICLE_SPAWN_MAX 256 /* bounded audit rows per mission    */
#define MISSION_PI            3.14159265358979323846
#define MISSION_TICK_HZ       20    /* the fixed sim step (car.h CAR_SIM_HZ)*/

/* Objective controller for the FSM-less arena families — see D-O1..D-O13. */
#define MISSION_MAX_CHECKS    16    /* most observed is 11 (N34.RAC)       */
#define MISSION_CTF_TEAMS      4    /* .CF2/.CF3/.CF4 — 2, 3 or 4 teams    */
#define MISSION_CTF_SLOTS      8    /* most observed is 4 (N11/N46/N20...) */

/*
 * D-O3 DATA — checkpoint trigger radius, metres, FULL 3-D.
 *
 * nitro.exe's race gate is FUN_00444bb0 (decompiled 2026-08-10): the
 * marker loader FUN_00444ad0 sscanf's "check%d", stores each marker's
 * three float coords as doubles in a 0x40-stride slot table, and the
 * per-frame gate measures dx²+dy²+dz² — Y INCLUDED — against the double
 * constant _DAT_004c4608 = 900.0, i.e. a 30 m SPHERE. The value this
 * port picked to be unambiguous turns out to be the original's own.
 * The Y axis costs nothing to honour: every check marker in the shipped
 * set sits within 0.9 m of its terrain height, so a ground car inside
 * the old 30 m XZ disc is inside the 30 m sphere as well.
 *
 * The unambiguity argument still holds in 3-D: the smallest gap between
 * consecutive checkpoints across all 13 .RAC courses plus N35.MSN is
 * 82 m (N34.RAC, check3->check4), and the nearest any grid slot sits to
 * check1 is 55 m (N35.RAC), so at 30 m no gate can be triggered out of
 * order and no race can start already inside its first gate. At the
 * 20 Hz step a car would have to exceed 600 m/s to tunnel through it.
 */
#define RACE_CHECK_R          30.0

/*
 * D-O4 INVENTED — the default session rules (mission_set_rules overrides).
 *
 * There is no per-mission rules record to read: all 75 miss8 missions
 * carry a 331-byte WDEF/WRLD payload and the one plausible limit field in
 * it holds the SAME value in every one of them, so it is a constant, not a
 * limit. The original keeps these in its multiplayer session setup instead
 * ("%d Laps", "%d Minutes", "No Limits!" — nitshell.dll).
 *
 * 1 lap because five of the thirteen races are point-to-point rather than
 * circuits (N25's last->first gap is 1755 m, N27's 2868 m, N29's 2515 m):
 * a second lap on those is a multi-kilometre backtrack that reads as a
 * broken mission. 10 minutes because the longest single lap in the set is
 * comfortably inside it and a race with no clock has no loss at all.
 */
#define RACE_LAPS_DEFAULT      1
#define RACE_MINUTES_DEFAULT  10

/*
 * D-O10 INVENTED — the capture touch radius, metres (XZ). One constant is
 * used for BOTH halves of a capture: picking an enemy flag up off its
 * stand, and bringing it home to a slot of your own base.
 *
 * Like RACE_CHECK_R this is chosen to be unambiguous rather than
 * authentic — the original's real radius is not reversed. What makes 30 m
 * safe is measured, not guessed: across all 17 .CF* files the closest any
 * flag sits to a slot belonging to a DIFFERENT team is 159.3 m, so two
 * 30 m spheres (60 m combined) can never both be satisfied at once, and
 * standing on your own base can never also count as standing on someone
 * else's flag. It is also comfortably larger than the 8-57 m a flag sits
 * from its OWN base, which is what makes a base reachable without
 * clipping the flag stand.
 */
#define CTF_TOUCH_R          30.0

/*
 * D-O11 INVENTED — the default capture target (mission_set_rules
 * overrides), same reasoning as D-O4: there is no per-mission rules
 * record, and the original keeps "%d Captures" in its session setup
 * (nitshell.dll) beside "%d Laps" and "%d Minutes".
 *
 * 1, because a capture is a 380 m round trip on the shortest map
 * (N43.CF2) and a 4474 m one on the longest (N46.CF2) — at 25 m/s that is
 * 15 s against 3 minutes, so a target big enough to be interesting on
 * N43 would not fit inside any sane clock on N46. The clock itself is
 * shared with the race (RACE_MINUTES_DEFAULT).
 */
#define CTF_CAPTURES_DEFAULT   1

/*
 * D-O12 INVENTED — the player is team 1.
 *
 * Nothing in a .CF* file marks a team as the human's; in the original the
 * lobby assigns it. Team 1 is chosen because #spawn1 exists in all 17
 * files (it is the one team every map has), and because it was already
 * the arbitrary choice mission_start_pose made for the start position.
 */
#define CTF_PLAYER_TEAM        1

/*
 * D-O14 DATA — the opponent vehicle a melee spawns.
 *
 * `app/nitmcar.def` (239 bytes, a loose file beside nitro.exe) is the Nitro
 * Pack's MULTIPLAYER CAR template: a headerless VCFC-shaped record whose
 * name field is "TSS", whose chassis is `vcgcourc.vdf`, whose paint is
 * `corchel3.vtf`, whose wheels are `wauto_0a.wdf`/null/`wauto_0a.wdf`,
 * whose eight armor/chassis words are all 0x258, and whose four weapons are
 * gfirdrop / gcmedium / gfmedium / gdumb on mounts 0..3.
 *
 * `vcgcour3.vcf` inside nitro.zfs carries every one of those fields with
 * the same values, in a BWD2 chunk container the engine's existing
 * VCF->VDF->WDF chain already reads. So the bots drive the vehicle the
 * multiplayer template names, through the shipped .vcf that IS it — no new
 * parser, and no new file for a purchaser to stage. If the two ever
 * disagree, nitmcar.def is the authority and this constant is wrong.
 */
#define MELEE_BOT_VCF        "vcgcour3"

/*
 * D-O15 INVENTED — how many opponents a melee spawns.
 *
 * Nothing in a .CBT says. The maps carry only unoccupied grid slots: 226
 * class-1 `spawn` markers across the 24 files and not one vehicle, because
 * the original filled them with however many humans joined the session
 * (nitro.exe: "No open slots for net vehicles", "SpawnLoc_GetLocation - no
 * spawn points for team %d").
 *
 * 3 because it is the largest count that fits EVERY map without putting two
 * cars on one slot: the smallest grid in the set is N10.CBT's four slots,
 * one of which is the player's. The per-map cap below still applies, so a
 * map with fewer slots gets fewer opponents rather than a stack.
 */
#define MELEE_BOTS_DEFAULT     3
#define MELEE_MAX_BOTS         8    /* table bound; names below match it   */

/*
 * D-O16 INVENTED — the default kill target (mission_set_rules overrides),
 * the same reasoning as D-O4/D-O11: there is no per-mission rules record,
 * and the original keeps "%d Kills" in its session setup (nitshell.dll)
 * beside "%d Laps", "%d Captures" and "%d Minutes".
 *
 * 1, matching RACE_LAPS_DEFAULT and CTF_CAPTURES_DEFAULT. The default of
 * every arena family in this port is the smallest whole objective, and the
 * reason is the same in all three: the default is what a player meets
 * first, and it should be finishable on a first attempt by someone who has
 * never driven this engine before. Three authored-loadout opponents converge
 * on the player at once; tools/melee_probe.c now measures that opening against
 * the VCF defense/loadout scale rather than the retired 100-hp/1-hp timing.
 * "Clear the arena" is a fine
 * game and it is what the menu's "3 Kills" is for, but it is not a first
 * five minutes.
 *
 * The target is CLAMPED at load to the number of opponents actually
 * spawned; asking for more kills than there are cars would be a mission
 * that can only be lost, which is the whole reason melee shipped without a
 * controller until now.
 */
#define MELEE_KILLS_DEFAULT    1

/*
 * D-O17 INVENTED — the player is team 1 and every opponent is team 2.
 *
 * combat.c's hostility rule (D-C9) is "nonzero team, different from the
 * reference's", so one bot team makes all of them the player's enemies and
 * none of them each other's. That is deliberate and it is not what a
 * free-for-all deathmatch would do:
 *
 *   - the player's kill counter can only be advanced by the player, so a
 *     melee cannot be won by sitting still while the bots thin themselves
 *     out (tools/adv_melee_probe.c attacks exactly this);
 *   - a mutual free-for-all would leave the last bot standing at full
 *     strength while the player watched, which is worse play, not more
 *     faithful play.
 *
 * The original's Combat mode is a free-for-all between HUMANS; its bot AI
 * for that mode is not decompiled, so there is no behaviour here to
 * reproduce, only one to choose.
 */
#define MELEE_PLAYER_TEAM      1
#define MELEE_BOT_TEAM         2

/*
 * D-O20 DATA, and a deliberately awkward one — the name the win line calls
 * the player.
 *
 * nitro.exe's melee win line is "*** %s has reached %d kills! %s" and the
 * %s is the player's own name, typed at nitshell.dll's "Enter user name".
 * This port ships no name entry, so it has no name to put there — and the
 * binary has a string for exactly that case: "Unknown Loser", its fallback
 * for a player with no name (it is also what mission_team_name returns for
 * an out-of-range team). It reads badly over a WIN, and it is kept anyway:
 * the alternative is authoring an English name where the game ships one,
 * which is the thing coverage-ledger.md §5/§6 exist to catch. Replace it
 * with the player's real name the day the shell asks for one.
 */
#define MELEE_PLAYER_NAME    "Unknown Loser"

/* ----------------------------------------------------------------------- */
/* Module state                                                            */
/* ----------------------------------------------------------------------- */

/* ODEF object snapshot (D3) — label + placement, no geometry. */
typedef struct {
    char     label[9];
    int      label_id;
    uint32_t class_id;
    uint16_t flags;
    uint16_t team;
    double   pos[3];
    double   yaw;               /* D-O2: heading from the ODEF frame     */
    int      has_yaw;           /* 0 when that frame carries no heading  */
} MissionObj;

/* FSM entity resolved into the world. The mover remains XZ-only; the
 * presentation fields keep its rendered body aligned to terrain and motion. */
typedef struct {
    const MissionObj *obj;       /* NULL when unresolved (logged)        */
    int32_t           id;        /* setId cell (default 0)               */
    int               is_user;   /* D5                                   */
    int               body_owner;/* canonical FSM alias for this ODEF    */
    int               scene_obj; /* scene.c object for write-back (D14) */
    double            ground_clearance;
    double            ai_y;      /* raw mover Y; changes only on teleport */
    double            prev_x, prev_z;
    double            motion_vx, motion_vz; /* authored velocity this tick */
    int               have_ground;
    int               have_prev;
    uint32_t          world_static_contacts;
    uint32_t          world_terrain_refusals;
    uint32_t          world_pushouts;
} MissionEnt;

/* Action ledger classes (the M7 coverage ledger). */
enum {
    LEDGE_SEMANTIC = 0,   /* fully implemented semantics                 */
    LEDGE_CONST,          /* semantically-correct constant (no system yet) */
    LEDGE_TODO            /* logged no-op — M7 work                      */
};

typedef struct {
    uint32_t    calls;
    uint8_t     cls;            /* LEDGE_*                              */
    uint8_t     announced;      /* log-once flag for CONST/TODO         */
} MissionActionStat;

static int           s_loaded;
static int           s_owns_world;  /* mission_load only; attach never does */
static char          s_mission[112];

static FsmImage     *s_img;
static FsmMachine  **s_machines;
static FsmMachine   *s_guidance_machine;
static int           s_n_machines;
static int           s_machines_alive;

static MissionObj   *s_objs;
static int           s_nobj, s_objcap;

static MissionEnt   *s_ents;
static int           s_nents;
static int           s_user_ent;    /* entity index of the player, -1 */

static double        s_car_x, s_car_y, s_car_z, s_car_yaw;
static double        s_car_vx, s_car_vz;
static int           s_car_set;     /* mission_set_car called at least once */
static int           s_skip_pressed; /* Space edge for isKeypress this tick */
static double        s_script_pos[3], s_script_yaw, s_script_speed;
static int           s_script_have;
/* A zero-speed authored player teleport reaches AI_GOAL_NONE in the same
 * mover tick. Keep that one tick host-visible so a camera cannot release the
 * physical car at the pre-teleport pose (B01 first-course handoff). */
static uint64_t      s_script_snap_tick = UINT64_MAX;
static int           s_user_snap_pending;
static double        s_user_snap_speed;

/* PORT GUIDANCE objective lines (mission_objective_lines): unlike the
 * one-tick nav goal, these survive the FSM's machine round-robin.
 * Every polled-and-unsatisfied isWithinNav/isWithinSqNav gate is tracked
 * by (machine, predicate site) and stays live while the script polls it; a line whose gate goes unpolled for the TTL dies silently, and a
 * USER gate that is genuinely satisfied records the reached tick
 * (H-UAT-014). Both bounds are port INVENTIONS (display capacity and
 * "still live" horizon; the original has no trip objective HUD at all). */
#define MISSION_NAVOBJ_MAX 128  /* live predicate audit capacity          */
#define MISSION_NAVOBJ_TTL 30   /* ticks unpolled before a line dies     */

typedef struct {
    int      used;
    const FsmMachine *machine;
    uint32_t pc;
    int poll_order;
    FsmNavConsequence consequence;
    int      path;              /* current FSM path index                  */
    int      ent;               /* current gated entity                    */
    double   x, z, r;
    int      sq;
    uint64_t seen;              /* last unsatisfied poll tick            */
    uint64_t born;              /* first poll tick — stable HUD order    */
} MissionNavObj;

static MissionNavObj s_navobj[MISSION_NAVOBJ_MAX];
static int s_nav_poll_order;
static uint64_t      s_navobj_reached_tick;
static int           s_navobj_reached_have;

static uint64_t      s_tick;

typedef struct {
    char clip[MISSION_MSG_MAX];
    int  owner;                    /* entity id, -1 for cb/cbPrior */
} MissionCb;

static MissionCb     s_cbq[MISSION_CBQ_MAX];
static int           s_cbq_len;
static MissionCb     s_cb_cur;
static int           s_cb_playing;
static int           s_cb_ticks_left;
static int           s_cb_blocked;   /* stopCB's persistent enqueue block */
static char          s_movie[MISSION_NAME_MAX];
static int           s_movie_pending;
static int           s_movie_prev_cam_active;
/* WDEF/WRLD +4/+17: the mission-authored intro/outro movie fields.
 * Each is a 13-byte NUL-padded filename; parsed before the mission buffer
 * is released and cleared with the runner. */
static char          s_story_clip[2][14];

static int           s_state;                       /* MISSION_*        */
/* Raw positive 1-based SECOND failAllObj operand. It is deliberately
 * separate from s_message: diagnostics are presentation, not control data. */
static int           s_fail_text_index;
static MissionNote   s_notes[6];
static int           s_note_count;
static char          s_message[MISSION_MSG_MAX];
static int           s_have_message;


static MissionActionStat *s_astats;   /* [action_count] when FSM loaded */
static uint32_t      s_trap_counts[16];           /* per-opcode         */
static uint32_t      s_trap_total;

static uint32_t      s_rng = 0x1C0FFEEu;          /* D10 fixed seed     */

/* Objective controller (D-O1). Per-mission state, cleared by
 * mission_runner_reset; s_rule_* below are session state and are NOT. */
static int           s_family;                    /* MISSION_FAMILY_*   */
static struct { double x, y, z; } s_course[MISSION_MAX_CHECKS];
static int           s_ncheck;                    /* checkpoints found  */
static int           s_next_check;                /* index into s_course*/
static int           s_has99;                     /* course has a finish*/
static double        s_finish[3];                 /* check99 (D-O5)     */
static int           s_lap;
static int           s_lap_target;
static uint64_t      s_limit_ticks;               /* 0 = no clock       */
static int           s_start_ok;
static double        s_start_pos[3];
static double        s_start_yaw;

/* CAPTURE (D-O8). One entry per team, indexed [team-1]; a base is a SET
 * of slots and is scored by the nearest one, never by a centroid (D-O9). */
typedef struct {
    int    has_flag;
    double fx, fz;                                   /* a1flag<team>      */
    int    nslots;
    double sx[MISSION_CTF_SLOTS], sz[MISSION_CTF_SLOTS];  /* #spawn<team> */
} MissionCtfTeam;

static MissionCtfTeam s_ctf[MISSION_CTF_TEAMS];
static int           s_ctf_teams;      /* teams found, 0 = no controller  */
static int           s_ctf_carry;      /* team whose flag is aboard, 0=none */
static int           s_captures;
static int           s_capture_target;

/* MELEE (D-O14..D-O17). One entry per spawned opponent. Motion-facing and
 * terrain clearance live in its MissionEnt, shared with scripted movers. */
typedef struct {
    int    ent;                  /* combat/ai entity index                */
    int    scene_obj;            /* placed scene object, -1 = none        */
    int    was_alive;            /* to announce a death exactly once      */
} MissionMeleeBot;

static MissionMeleeBot s_melee[MELEE_MAX_BOTS];
static int           s_melee_bots;     /* opponents fielded, 0 = no controller */
static int           s_kill_target;

/* Browser host hooks for the one physical player. Mission-owned AI/AI
 * contacts run even when these are NULL. Host state survives runner resets. */
static MissionVehicleRadiusFn   s_vehicle_radius;
static MissionVehicleHeightFn   s_vehicle_height;
static MissionVehicleSeparateFn s_vehicle_separate;
static void                    *s_vehicle_contact_ctx;
static uint32_t                 s_vehicle_contacts;
static uint32_t                 s_player_vehicle_contacts;
static int                      s_player_contact_ready;
static MissionVehicleSpawn      s_vehicle_spawns[MISSION_VEHICLE_SPAWN_MAX];
static int                      s_n_vehicle_spawns;
static int                      s_vehicle_spawn_overflow;

static int mission_ent_is_clown_race_seed(int ent)
{
    const char *role = s_img ? fsm_image_entity_label(s_img, ent) : NULL;
    /* P02's decoded FSM role is the only scripted race opponent in the trip
     * corpus. Bind the spawn correction to that authored role, not a mission
     * coordinate or vehicle asset; broad AI ODEF policy remains H-UAT-026. */
    return role && strcasecmp(role, "clown") == 0;
}

static double vehicle_spawn_place(int ent, int scene_obj, int source,
                                  double x, double z, double authored_y)
{
    /* H-UAT-061 keeps the generic floor: authored height may intentionally
     * lift an AI vehicle but never embed it. H-UAT-068a narrows the new rule
     * to P02's decoded `clown` race role: its ODEF Y was retained as a 1.02 m
     * hover even though it is a ground start, so settle that role exactly like
     * the physical player. Explicit teleports and every unrelated ODEF retain
     * the prior max(authored, required) contract. */
    double required_y = car_settled_y_at(x, z);
    int settle_race_start = source == MISSION_VEHICLE_SPAWN_ODEF &&
                            mission_ent_is_clown_race_seed(ent);
    double final_y = settle_race_start ? required_y :
                     (authored_y < required_y ? required_y : authored_y);

    if (s_n_vehicle_spawns >= MISSION_VEHICLE_SPAWN_MAX) {
        s_vehicle_spawn_overflow = 1;
        return final_y;
    }
    MissionVehicleSpawn *s = &s_vehicle_spawns[s_n_vehicle_spawns++];
    s->tick = s_tick;
    s->ent = ent;
    s->scene_obj = scene_obj;
    s->source = source;
    s->x = x;
    s->z = z;
    s->authored_y = authored_y;
    s->terrain_y = terrain_height_at(x, z);
    s->required_y = required_y;
    s->final_y = final_y;
    return final_y;
}

/*
 * Route-owned AI motion is kinematic and XZ-only; promoted Stage-3 hosts take
 * the early physical writeback below. For the remaining movers, leaving spawn
 * Y/basis frozen makes hills bury/float the body. Generic cars preserve their
 * composed model clearance; P02's
 * decoded race seed uses the player's settled terrain clearance to remove its
 * measured hover. Explicit raised teleports and non-cars preserve authored
 * clearance. A teleport is the only mover operation that changes raw Y.
 * Facing follows the mover's persistent bounded-steering heading (ai.h D-A15)
 * once it moves, and exact transformed geometry is re-seated after rotation.
 *
 * This is presentation conformance, not player suspension: it does not alter
 * ai.c path state, speed, arrival predicates, or the user's physical car.
 */
static int mission_ent_owner(int ent)
{
    if (ent < 0 || ent >= s_nents)
        return -1;
    int owner = s_ents[ent].body_owner;
    return owner >= 0 && owner < s_nents ? owner : ent;
}

static void mission_ent_motion_init(int ent, const double p[3])
{
    ent = mission_ent_owner(ent);
    if (ent < 0 || !p) return;
    MissionEnt *e = &s_ents[ent];
    e->ground_clearance = p[1] - terrain_height_at(p[0], p[2]);
    /* FACT (Q10): unrelated ground vehicles preserve their composed model
     * origin-to-hull-bottom offset. P02's race seed is the bounded exception:
     * vehicle_spawn_place deliberately selected the physical-car settled
     * clearance to remove its authored hover. */
    if (!mission_ent_is_clown_race_seed(ent) && e->obj &&
        e->obj->class_id == 1 && !e->is_user && e->scene_obj >= 0) {
        double c[3], half[2], axis[2], yspan[2];
        if (scene_obj_obb(e->scene_obj, c, half, axis, yspan) == 0)
            e->ground_clearance = p[1] - yspan[0];
    }
    e->ai_y = p[1];
    e->prev_x = p[0];
    e->prev_z = p[2];
    e->motion_vx = e->motion_vz = 0.0;
    e->have_ground = 1;
    e->have_prev = 1;
}

static void mission_ent_ground_pose(int ent, double p[3])
{
    ent = mission_ent_owner(ent);
    if (ent < 0 || !p) return;
    MissionEnt *e = &s_ents[ent];
    if (ai_physical_active(ent))
        return; /* car.c owns Y/airborne/landing state for promoted hosts */
    double ground = terrain_height_at(p[0], p[2]);
    if (!e->have_ground || fabs(p[1] - e->ai_y) > 1e-6) {
        e->ground_clearance = p[1] - ground;
        e->ai_y = p[1];
        e->have_ground = 1;
    }
    p[1] = ground + e->ground_clearance;
}

static void mission_ent_writeback(int ent, int scene_obj, double p[3])
{
    ent = mission_ent_owner(ent);
    if (ent < 0 || scene_obj < 0 || !p) return;
    MissionEnt *e = &s_ents[ent];
    if (ai_physical_active(ent)) {
        /* Stage 3: scene and combat consume the integrated pose directly.
         * Never terrain-seat it or derive a second velocity/attitude owner. */
        CarLive live = {0};
        int teleported = !e->have_prev ||
                         hypot(p[0] - e->prev_x, p[2] - e->prev_z) >
                             COMBAT_TELEPORT_GUARD;
        scene_obj_set_pos(scene_obj, p);
        e->motion_vx = e->motion_vz = 0.0;
        if (e->have_prev && !teleported) {
            e->motion_vx = (p[0] - e->prev_x) / AI_TICK_DT;
            e->motion_vz = (p[2] - e->prev_z) / AI_TICK_DT;
        }
        if (ai_physical_live(ent, &live) == 0)
            (void)scene_obj_set_car_facing(scene_obj, live.yaw,
                                           live.pitch, live.roll);
        e->ground_clearance = p[1] - terrain_height_at(p[0], p[2]);
        e->ai_y = p[1];
        e->prev_x = p[0];
        e->prev_z = p[2];
        e->have_ground = e->have_prev = 1;
        return;
    }
    /* A teleport is a pose jump, not motion: raw mover Y changes only
     * there (mission_ent_ground_pose's own convention), and treating the
     * jump as one tick of travel would hand the contact phase a bogus
     * authored velocity of hundreds of m/s. */
    int teleported = !e->have_ground || fabs(p[1] - e->ai_y) > 1e-6;
    int moved = 0;
    mission_ent_ground_pose(ent, p);
    scene_obj_set_pos(scene_obj, p);
    e->motion_vx = e->motion_vz = 0.0;
    if (e->have_prev && !teleported) {
        double dx = p[0] - e->prev_x, dz = p[2] - e->prev_z;
        moved = dx * dx + dz * dz > 1e-18;
        e->motion_vx = dx / AI_TICK_DT;
        e->motion_vz = dz / AI_TICK_DT;
    }
    if (ai_goal(ent) != AI_GOAL_NONE ||
        (moved && ai_path_id(ent) >= 0)) {
        if (!e->obj || e->obj->class_id != 1) {
            double h = ai_get_heading(ent);
            scene_obj_set_facing(scene_obj, -sin(h), cos(h));
            e->prev_x = p[0];
            e->prev_z = p[2];
            e->have_prev = 1;
            return;
        }
        /* D-A15: face the mover's persistent bounded-steering heading
         * (car.h convention: forward = (-sin h, 0, cos h)) instead of
         * per-tick displacement. Apply it while brake-steering and on the
         * final moved arrival tick so the rendered body follows the same
         * bounded delta without snapping. Contact shoves remain
         * translation-only; never-driven idle agents keep authored facing. */
        double h = ai_get_heading(ent);
        double fx = -sin(h), fz = cos(h);
        double span = fmax(hypot(e->motion_vx, e->motion_vz) * AI_TICK_DT,
                           2.0);
        double hl = terrain_height_at(p[0] - fx * span, p[2] - fz * span);
        double hrx = fz, hrz = -fx;
        double hleft = terrain_height_at(p[0] - hrx * span,
                                         p[2] - hrz * span);
        double hright = terrain_height_at(p[0] + hrx * span,
                                          p[2] + hrz * span);
        double hf = terrain_height_at(p[0] + fx * span, p[2] + fz * span);
        double tx = fx, ty = (hf - hl) / (2.0 * span), tz = fz;
        double rx = hrx, ry = (hright - hleft) / (2.0 * span), rz = hrz;
        double nx = ry * tz - rz * ty;
        double ny = rz * tx - rx * tz;
        double nz = rx * ty - ry * tx;
        scene_obj_set_ground_facing(scene_obj, fx, fz, nx, ny, nz);
        /* Rotation changes the lowest transformed vertex. Re-seat the exact
         * authored hull geometry on terrain after pitch/roll; no offset is
         * guessed, and the model origin remains whatever its VDF/geometry
         * requires. */
        double c[3], half[2], axis[2], yspan[2];
        /* First place the rotated object at the candidate AI origin, then
         * measure its world-space bottom there. Match that lowest corner to
         * terrain at the same XZ contact point, not terrain at the origin. */
        scene_obj_set_pos(scene_obj, p);
        double min_gap, max_gap;
        if (scene_obj_obb(scene_obj, c, half, axis, yspan) == 0 &&
            scene_obj_ground_gaps(scene_obj, terrain_height_at,
                                  &min_gap, &max_gap) == 0) {
            /* Exact authored mesh vertices, transformed through PART + ODEF,
             * own support. Lift the most penetrating one onto terrain. */
            p[1] -= min_gap;
            e->ai_y = p[1];
            ai_set_height(ent, p[1]);
            scene_obj_set_pos(scene_obj, p);
        }
    }
    e->prev_x = p[0];
    e->prev_z = p[2];
    e->have_prev = 1;
}

typedef struct {
    int ent;
    int scene_obj;
    double cx, cz;              /* live OBB centre */
    double hx, hz;
    double ax, az;              /* OBB local +x axis */
    double y0, y1;              /* live vertical span */
    double vx, vz;              /* authored mover velocity this tick */
    double dx, dz;              /* accumulated contact displacement */
} MissionVehicleBody;

static int mission_ent_is_vehicle(int ent)
{
    if (ent < 0 || ent >= s_nents)
        return 0;
    const MissionEnt *e = &s_ents[ent];
    if (e->is_user)
        return 1;
    if (e->obj && e->obj->class_id == 1)
        return 1;
    for (int i = 0; i < s_melee_bots; i++)
        if (s_melee[i].ent == ent)
            return 1;
    return 0;
}

static void vehicle_body_shift(MissionVehicleBody *b, double dx, double dz)
{
    b->cx += dx;
    b->cz += dz;
    b->dx += dx;
    b->dz += dz;
}

/* SAT for two horizontal OBBs. Axis order is stable (A x/z, then B x/z);
 * ties keep the earlier axis, so coincident/symmetric contacts are
 * deterministic without a pointer- or allocation-order tie break. */
static int vehicle_obb_contact(const MissionVehicleBody *a,
                               const MissionVehicleBody *b,
                               double *out_nx, double *out_nz,
                               double *out_depth)
{
    double axes[4][2] = {
        { a->ax, a->az }, { -a->az, a->ax },
        { b->ax, b->az }, { -b->az, b->ax }
    };
    double dcx = b->cx - a->cx, dcz = b->cz - a->cz;
    double best = 1.0e300, best_nx = 0.0, best_nz = 0.0;

    for (int k = 0; k < 4; k++) {
        double ux = axes[k][0], uz = axes[k][1];
        double u2 = ux * ux + uz * uz;
        if (!(u2 > 0.0))
            continue;
        double inv = 1.0 / sqrt(u2);
        ux *= inv;
        uz *= inv;

        double alx = -a->az, alz = a->ax;
        double blx = -b->az, blz = b->ax;
        double ra = a->hx * fabs(a->ax * ux + a->az * uz) +
                    a->hz * fabs(alx * ux + alz * uz);
        double rb = b->hx * fabs(b->ax * ux + b->az * uz) +
                    b->hz * fabs(blx * ux + blz * uz);
        double signed_dist = dcx * ux + dcz * uz;
        double overlap = ra + rb - fabs(signed_dist);
        if (!(overlap > 0.0))
            return 0;
        if (overlap < best) {
            best = overlap;
            double sign = signed_dist < 0.0 ? -1.0 : 1.0;
            best_nx = ux * sign;       /* A -> B */
            best_nz = uz * sign;
        }
    }
    if (!(best < 1.0e300))
        return 0;
    *out_nx = best_nx;
    *out_nz = best_nz;
    *out_depth = best;
    return 1;
}

/* Circle (physical player) against one live AI OBB. The returned normal
 * points box -> player. A centre inside the OBB exits through the nearest
 * face; exact ties choose local +x through the fixed <=/sign ordering. */
static int vehicle_player_contact(double px, double pz, double radius,
                                  const MissionVehicleBody *b,
                                  double *out_nx, double *out_nz,
                                  double *out_depth)
{
    double dx = px - b->cx, dz = pz - b->cz;
    double lx = dx * b->ax + dz * b->az;
    double lz = -dx * b->az + dz * b->ax;
    double qx = lx < -b->hx ? -b->hx : (lx > b->hx ? b->hx : lx);
    double qz = lz < -b->hz ? -b->hz : (lz > b->hz ? b->hz : lz);
    double ex = lx - qx, ez = lz - qz;
    double e2 = ex * ex + ez * ez;
    double nlx, nlz, depth;

    if (e2 > 0.0) {
        if (e2 >= radius * radius)
            return 0;
        double e = sqrt(e2);
        nlx = ex / e;
        nlz = ez / e;
        depth = radius - e;
    } else {
        double fx = b->hx - fabs(lx);
        double fz = b->hz - fabs(lz);
        if (fx <= fz) {
            nlx = lx < 0.0 ? -1.0 : 1.0;
            nlz = 0.0;
            depth = radius + fx;
        } else {
            nlx = 0.0;
            nlz = lz < 0.0 ? -1.0 : 1.0;
            depth = radius + fz;
        }
    }

    *out_nx = nlx * b->ax - nlz * b->az;
    *out_nz = nlx * b->az + nlz * b->ax;
    *out_depth = depth;
    return depth > 0.0;
}

/*
 * Dynamic vehicle contact phase: live hulls are rebuilt from source-owned
 * transforms every tick. The physical player is a COLP-derived circle; AI
 * bodies use their live scene OBBs. Every unordered pair is processed once,
 * in entity order. Equal movable bodies split both minimum translation and
 * the zero-restitution normal impulse symmetrically (D-A11); tangential
 * velocity is unchanged.
 */
static void vehicle_contacts_tick(int camera_owned_tick)
{
    MissionVehicleBody bodies[AI_MAX_AGENTS];
    int nb = 0;

    for (int ent = 0; ent < s_nents && nb < AI_MAX_AGENTS; ent++) {
        MissionEnt *e = &s_ents[ent];
        if (mission_ent_owner(ent) != ent || ent == s_user_ent ||
            !mission_ent_is_vehicle(ent) ||
            e->scene_obj < 0 || !combat_alive(ent) || combat_is_hidden(ent))
            continue;

        double c[3], half[2], axis[2], yspan[2];
        if (scene_obj_obb(e->scene_obj, c, half, axis, yspan) != 0 ||
            !(half[0] > 0.0) || !(half[1] > 0.0))
            continue;
        MissionVehicleBody *b = &bodies[nb++];
        memset(b, 0, sizeof *b);
        b->ent = ent;
        b->scene_obj = e->scene_obj;
        b->cx = c[0];
        b->cz = c[2];
        b->hx = half[0];
        b->hz = half[1];
        b->ax = axis[0];
        b->az = axis[1];
        b->y0 = yspan[0];
        b->y1 = yspan[1];
        b->vx = e->motion_vx;
        b->vz = e->motion_vz;
        if (ai_physical_active(ent))
            ai_world_velocity(ent, &b->vx, &b->vz);
    }

    /* The host has not applied this tick's scripted placement yet. Never
     * mix its destination XZ with the old physical car's contact callbacks,
     * including the tick that pops the camera or teleports without one. */
    int script_owns_player = s_user_snap_pending ||
                            ((camera_owned_tick || mission_cam_active()) &&
                             ai_goal(s_user_ent) != AI_GOAL_NONE);
    int physical_player = s_user_ent >= 0 && s_car_set &&
                          !script_owns_player && s_vehicle_radius &&
                          s_vehicle_height && s_vehicle_separate;
    if (physical_player) {
        double py0, py1;
        double radius = s_vehicle_radius(s_vehicle_contact_ctx);
        s_vehicle_height(&py0, &py1, s_vehicle_contact_ctx);
        if (py1 < py0) {
            double t = py0;
            py0 = py1;
            py1 = t;
        }
        if (radius > 0.0) {
            for (int i = 0; i < nb; i++) {
                if (bodies[i].y1 < py0 || bodies[i].y0 > py1)
                    continue;
                double nx, nz, depth;
                if (!vehicle_player_contact(s_car_x, s_car_z, radius,
                                            &bodies[i], &nx, &nz, &depth))
                    continue;
                /* n points AI -> player. Equal-mass inelastic contact gives
                 * each body half the closing normal speed; the car callback
                 * receives the updated AI velocity and applies the matching
                 * player-side change. */
                double closing = (bodies[i].vx - s_car_vx) * nx +
                                 (bodies[i].vz - s_car_vz) * nz;
                double rvx = bodies[i].vx - s_car_vx;
                double rvz = bodies[i].vz - s_car_vz;
                double ram_closing = hypot(rvx, rvz);
                if (closing > 0.0) {
                    /* D-C22: this is the physical hull-contact owner. Publish
                     * its pre-separation closing speed before the impulse
                     * moves centres apart; combat_tick's historical 2.5 m
                     * proximity fallback cannot see a resolved OBB contact.
                     * Ignore overlaps only on the first physical-player tick
                     * after placement/handoff: there is no prior sample to
                     * distinguish a spawn overlap from a driven contact. A
                     * global contact-count guard incorrectly discarded the
                     * first legitimate later collision when handoff was clear. */
                    if (s_player_contact_ready)
                        combat_player_vehicle_contact(bodies[i].ent,
                                                      ram_closing);
                    double impulse = 0.5 * closing;
                    bodies[i].vx -= nx * impulse;
                    bodies[i].vz -= nz * impulse;
                    /* Apply the already-computed half impulse exactly once.
                     * ai.c routes it to car.c for a promoted body or to the
                     * remaining kinematic contact channel, never both. */
                    ai_add_contact_velocity(bodies[i].ent,
                                            -nx * impulse,
                                            -nz * impulse);
                }
                double move = 0.5 * depth;
                s_car_x += nx * move;
                s_car_z += nz * move;
                vehicle_body_shift(&bodies[i], -nx * move, -nz * move);
                s_vehicle_separate(nx * move, nz * move, nx, nz,
                                   bodies[i].vx, bodies[i].vz,
                                   s_vehicle_contact_ctx);
                s_vehicle_contacts++;
                s_player_vehicle_contacts++;
            }
        }
    }
    if (physical_player)
        s_player_contact_ready = 1;
    else
        s_player_contact_ready = 0;

    for (int i = 0; i < nb; i++) {
        for (int j = i + 1; j < nb; j++) {
            if (bodies[i].y1 < bodies[j].y0 ||
                bodies[j].y1 < bodies[i].y0)
                continue;
            double nx, nz, depth;
            if (!vehicle_obb_contact(&bodies[i], &bodies[j],
                                     &nx, &nz, &depth))
                continue;
            /* n points body i -> body j. Exchange the closing normal
             * component as an equal-mass, zero-restitution impulse. */
            double closing = (bodies[i].vx - bodies[j].vx) * nx +
                             (bodies[i].vz - bodies[j].vz) * nz;
            if (closing > 0.0) {
                double impulse = 0.5 * closing;
                double ix = nx * impulse, iz = nz * impulse;
                bodies[i].vx -= ix;
                bodies[i].vz -= iz;
                bodies[j].vx += ix;
                bodies[j].vz += iz;
                ai_add_contact_velocity(bodies[i].ent, -ix, -iz);
                ai_add_contact_velocity(bodies[j].ent,  ix,  iz);
            }
            double move = 0.5 * depth;
            vehicle_body_shift(&bodies[i], -nx * move, -nz * move);
            vehicle_body_shift(&bodies[j],  nx * move,  nz * move);
            s_vehicle_contacts++;
        }
    }

    /* Commit only to the source owners. The first write-back already chose
     * authored facing; contact translation updates position and the next
     * tick's facing baseline, never turns a car sideways along the shove. */
    for (int i = 0; i < nb; i++) {
        MissionVehicleBody *b = &bodies[i];
        if (b->dx == 0.0 && b->dz == 0.0)
            continue;
        if (ai_physical_active(b->ent))
            ai_apply_vehicle_contact(b->ent, b->dx, b->dz,
                                      0.0, 0.0, 0.0, 0.0);
        else
            ai_translate_xz(b->ent, b->dx, b->dz);
        double p[3];
        if (ai_get_pos(b->ent, p) != 0)
            continue;
        mission_ent_ground_pose(b->ent, p);
        scene_obj_set_pos(b->scene_obj, p);
        if (!ai_physical_active(b->ent)) {
            /* The shove changed the terrain under the transformed hull.
             * Restore exact support without changing its chosen facing. */
            double low, high;
            if (scene_obj_ground_gaps(b->scene_obj, terrain_height_at,
                                      &low, &high) == 0) {
                p[1] -= low;
                s_ents[b->ent].ai_y = p[1];
                ai_set_height(b->ent, p[1]);
                scene_obj_set_pos(b->scene_obj, p);
            }
        }
        s_ents[b->ent].prev_x = p[0];
        s_ents[b->ent].prev_z = p[2];
        s_ents[b->ent].have_prev = 1;
    }
}

/*
 * AI world contact (ai.h D-A12) — the mission-owned stage between ai_tick
 * and the scene write-back. The mover stays kinematic and world-blind;
 * this stage constrains its PROPOSED moves against the world and commits
 * the corrections back through ai_constrain_xz — position only, so the
 * next tick starts from the corrected pose while the authored route and
 * dynamic-contact state (D-A10/D-A11) remain separately owned.
 *
 * Static scenery is a per-part oriented-box table built ONCE per
 * load/attach — never rebuilt per tick — with the same classification the
 * host's player-collider build applies (webmain.c build_colliders):
 * drive-on surfaces carry no box (the car rides the terrain across them,
 * scene.c D11), live vehicle hulls are excluded (their contact is the
 * dynamic phase above), and degenerate boxes (no footprint, no vertical
 * span) are not obstacles. The two thresholds below are the webmain.c
 * COLLIDER_MIN_HALF / COLLIDER_MIN_THICK policy twins.
 */
/* One immutable setup-time table serves both the established kinematic world
 * constraint and attached physical contexts. CarCollider's final top-surface
 * token stays zero because D-A12 deliberately excludes drive-on parts. */
typedef CarCollider MissionWorldBox;

static MissionWorldBox *s_world_boxes;
static int              s_nworld_boxes;
static signed char      s_world_detour[AI_MAX_AGENTS];

#define MISSION_WORLD_MIN_HALF  0.25    /* footprint twin: webmain.c   */
#define MISSION_WORLD_MIN_THICK 0.05    /* flat-decal twin: webmain.c  */

static void world_colliders_build(void)
{
    /* Shadows borrow this snapshot. Clear their pointer before replacing its
     * storage, then refresh it after the one setup-time rebuild completes. */
    ai_shadow_set_world(NULL, 0, 0.0, 0.0, 0.0, 0.0);
    free(s_world_boxes);
    s_world_boxes = NULL;
    s_nworld_boxes = 0;
    memset(s_world_detour, 0, sizeof s_world_detour);

    /* Terrain bounds remain authoritative even in a scene with zero static
     * OBB candidates. Publish them before every early return so contexts
     * attached after setup inherit the same drivable extent. */
    double x0 = 0.0, z0 = 0.0, x1 = 0.0, z1 = 0.0;
    if (terrain_used_bounds(&x0, &z0, &x1, &z1) == 0)
        ai_shadow_set_world(NULL, 0, x0, z0, x1, z1);

    int n = scene_obj_count();
    if (n <= 0)
        return;
    int cap = 0;
    for (int i = 0; i < n; i++) {
        if (mission_scene_object_is_vehicle(i))
            continue;
        cap += scene_obj_part_count(i);
    }
    if (cap <= 0)
        return;
    MissionWorldBox *tbl = malloc((size_t)cap * sizeof *tbl);
    if (!tbl)
        return;
    int k = 0;
    for (int i = 0; i < n; i++) {
        if (mission_scene_object_is_vehicle(i))
            continue;
        int np = scene_obj_part_count(i);
        for (int p = 0; p < np; p++) {
            if (scene_part_drive_surface(i, p))
                continue;                       /* D11 drive-on ground */
            double c[3], half[2], axis[2], yspan[2];
            if (scene_obj_part_obb(i, p, c, half, axis, yspan) != 0)
                continue;
            int gate = scene_obj_part_gate(i, p);
            if (!gate && (half[0] < MISSION_WORLD_MIN_HALF ||
                          half[1] < MISSION_WORLD_MIN_HALF))
                continue;
            if (gate) {
                if (half[0] < MISSION_WORLD_MIN_HALF)
                    half[0] = MISSION_WORLD_MIN_HALF;
                if (half[1] < MISSION_WORLD_MIN_HALF)
                    half[1] = MISSION_WORLD_MIN_HALF;
            }
            if (yspan[1] - yspan[0] < MISSION_WORLD_MIN_THICK)
                continue;
            tbl[k].x = c[0];
            tbl[k].z = c[2];
            tbl[k].hx = half[0];
            tbl[k].hz = half[1];
            tbl[k].ax = axis[0];
            tbl[k].az = axis[1];
            tbl[k].y0 = yspan[0];
            tbl[k].y1 = yspan[1];
            tbl[k].drivable_object = 0;
            tbl[k].drivable_parent = 0;
            k++;
        }
    }
    if (k == 0) {
        free(tbl);
        return;
    }
    s_world_boxes = tbl;
    s_nworld_boxes = k;
    ai_shadow_set_world(s_world_boxes, s_nworld_boxes,
                        x0, z0, x1, z1);
}

/*
 * Earliest hit parameter t in (0,1] of the segment start -> start+m
 * against one static box expanded by the agent hull, or -1 when the
 * segment misses or STARTS inside (the push-out backstop owns the
 * start-inside case). The expansion is the exact Minkowski sum for a
 * pure-translation sweep: each box axis grows by the hull's projection
 * onto it. On a hit the contacted face's outward normal (world XZ,
 * opposing the motion) goes to out_n*. Slab order is fixed (box x, then
 * box z) and ties keep the earlier axis, so the result is deterministic.
 */
static double world_sweep_box(double sx, double sz, double mx, double mz,
                              const MissionWorldBox *b,
                              double ahx, double ahz, double aax, double aaz,
                              double *out_nx, double *out_nz)
{
    const double ux = b->ax,  uz = b->az;   /* box local +x */
    const double vx = -b->az, vz = b->ax;   /* box local +z */
    const double wx = -aaz,   wz = aax;     /* agent local +z */
    const double ex = b->hx + ahx * fabs(aax * ux + aaz * uz) +
                            ahz * fabs(wx * ux + wz * uz);
    const double ez = b->hz + ahx * fabs(aax * vx + aaz * vz) +
                            ahz * fabs(wx * vx + wz * vz);
    const double lp[2] = { (sx - b->x) * ux + (sz - b->z) * uz,
                           (sx - b->x) * vx + (sz - b->z) * vz };
    const double lm[2] = { mx * ux + mz * uz, mx * vx + mz * vz };
    const double eh[2] = { ex, ez };

    double tnear = 0.0, tfar = 1.0;
    int naxis = -1;
    double nsign = 0.0;
    for (int k = 0; k < 2; k++) {
        if (fabs(lm[k]) < 1e-12) {
            if (fabs(lp[k]) > eh[k])
                return -1.0;
            continue;
        }
        double t0 = (-eh[k] - lp[k]) / lm[k];
        double t1 = ( eh[k] - lp[k]) / lm[k];
        double sign = -1.0;             /* entering through the -e face */
        if (t0 > t1) {
            double tmp = t0;
            t0 = t1;
            t1 = tmp;
            sign = 1.0;
        }
        if (t0 > tnear) {
            tnear = t0;
            naxis = k;
            nsign = sign;
        }
        if (t1 < tfar)
            tfar = t1;
        if (tnear > tfar)
            return -1.0;
    }
    if (naxis < 0 || tnear <= 0.0)
        return -1.0;                    /* parallel-inside or embedded  */
    if (naxis == 0) {
        *out_nx = ux * nsign;
        *out_nz = uz * nsign;
    } else {
        *out_nx = vx * nsign;
        *out_nz = vz * nsign;
    }
    return tnear;
}

/*
 * Planner query for the bounded native-route slice. Terrain surface/blocked
 * bits are the binary-verified A* inputs. Static occupancy is a port-owned
 * substitute for Nitro's predictive vehicle-control probe: test the complete
 * 10 m edge against the same immutable boxes and live hull used by the
 * authoritative post-move collision stage, rather than letting A* choose a
 * route through a building and repairing it after impact.
 */
static int mission_ai_nav_query(int ent,
                                double from_x, double from_z,
                                double to_x, double to_z,
                                AiNavSample *out)
{
    if (!out)
        return -1;
    memset(out, 0, sizeof *out);
    if (terrain_nav_sample(to_x, to_z,
                           &out->surface_class, &out->blocked) != 0)
        return -1;

    if (ent < 0 || ent >= s_nents || s_ents[ent].scene_obj < 0)
        return 0;

    /* Keep the planner and authoritative ground-contact stage on the same
     * contract. D-A12 is a per-tick rate (CAR_FOLLOW_MAX * AI_TICK_DT),
     * not a budget for the whole 10 m A* edge: scoring the raw edge (or
     * its midpoint) as one climb marks driveable authored slopes occupied.
     * Slice at one tick of AI_SPEED_MAX — the longest move the contact
     * stage will see — and keep start/mid/end samples on each slice so a
     * spike cannot hide between endpoints. Hover-clear vehicles still
     * overfly the discontinuity. */
    MissionEnt *e = &s_ents[ent];
    if (!ai_race_active(ent) && e->have_ground &&
        fabs(e->ground_clearance) <= AI_WORLD_MAX_STEP) {
        double dx = to_x - from_x, dz = to_z - from_z;
        double len = hypot(dx, dz);
        double piece = AI_SPEED_MAX * AI_TICK_DT;
        int n = (len > piece) ? (int)ceil(len / piece) : 1;
        double prev = terrain_height_at(from_x, from_z);
        for (int i = 1; i <= n; i++) {
            double t1 = (double)i / (double)n;
            double tm = ((double)i - 0.5) / (double)n;
            double hs[2] = {
                terrain_height_at(from_x + dx * tm, from_z + dz * tm),
                terrain_height_at(from_x + dx * t1, from_z + dz * t1)
            };
            for (int k = 0; k < 2; k++) {
                double dh = hs[k] - prev;
                if (dh > AI_WORLD_MAX_STEP || -dh > AI_WORLD_MAX_DROP) {
                    out->occupied = 1;
                    out->cliff = 1;     /* D-A21: terrain-step impassable */
                    return 0;
                }
                prev = hs[k];
            }
        }
    }

    /* H-UAT-068b PORT CORRECTION: an authored race already carries its
     * course through the native-style planner. The port's coarse per-part
     * static OBB substitute falsely closes P02's track at waypoint 8; Nitro's
     * unresolved road-object/predictive probe does not authorize that closure.
     * Keep terrain surface costs above, but do not feed those coarse boxes into
     * an active race plan. Ordinary GOTO/FOLLOW ownership is unchanged. */
    if (s_nworld_boxes <= 0 || ai_race_active(ent))
        return 0;

    double c[3], half[2], axis[2], yspan[2], p[3];
    if (scene_obj_obb(s_ents[ent].scene_obj, c, half, axis, yspan) != 0 ||
        ai_get_pos(ent, p) != 0 || half[0] <= 0.0 || half[1] <= 0.0)
        return 0;

    const double sx = from_x + (c[0] - p[0]);
    const double sz = from_z + (c[2] - p[2]);
    const double mx = to_x - from_x;
    const double mz = to_z - from_z;
    for (int k = 0; k < s_nworld_boxes; k++) {
        const MissionWorldBox *b = &s_world_boxes[k];
        double nx, nz;
        if (world_sweep_box(sx, sz, mx, mz, b, half[0], half[1],
                            axis[0], axis[1], &nx, &nz) >= 0.0) {
            out->occupied = 1;
            break;
        }

        /* world_sweep_box intentionally leaves start-inside to push-out.
         * A planner edge whose destination remains embedded is still
         * unusable, so test that endpoint against the same Minkowski sum. */
        const double ux = b->ax, uz = b->az;
        const double vx = -b->az, vz = b->ax;
        const double wx = -axis[1], wz = axis[0];
        const double ex = b->hx +
            half[0] * fabs(axis[0] * ux + axis[1] * uz) +
            half[1] * fabs(wx * ux + wz * uz);
        const double ez = b->hz +
            half[0] * fabs(axis[0] * vx + axis[1] * vz) +
            half[1] * fabs(wx * vx + wz * vz);
        const double dx = sx + mx - b->x;
        const double dz = sz + mz - b->z;
        if (fabs(dx * ux + dz * uz) <= ex &&
            fabs(dx * vx + dz * vz) <= ez) {
            out->occupied = 1;
            break;
        }
    }
    return 0;
}

/*
 * The stage itself. For every live, visible AI vehicle with a rendered
 * body (the dynamic phase's own filter). An active authored race retains its
 * course ownership through this port's coarse 10 m cliff/static substitutes;
 * per-tick terrain seating still happens in mission_ent_writeback, and dynamic
 * vehicle contact remains in the later shared phase.
 *
 *   0. A teleport is authored placement — raw mover Y changes only there
 *      (mission_ent_ground_pose's convention) — so it is adopted whole,
 *      never constrained. First-tick agents without a committed previous
 *      pose are likewise exempt.
 *   1. Vehicle-ground step limit (ai.h D-A12): when the terrain under the
 *      move — sampled at both ends and mid-segment, so a spike between
 *      the endpoints cannot be skipped — changes by more than
 *      AI_WORLD_MAX_STEP in EITHER direction, the move crosses a cliff
 *      face, not a slope. The whole move is refused; the body never
 *      snaps up or down the discontinuity.
 *   2. Static scenery sweep: the live hull (rebuilt from the scene
 *      object's committed transform, translated by the proposed move —
 *      pure translation, so the expansion in world_sweep_box is exact)
 *      stops at the earliest contact and the remainder slides along the
 *      contacted face. A head-on hit turns the blocked remainder clockwise
 *      along the face, so path-followers route around walls instead of
 *      deadlocking against them. One retry bounds the work.
 *   3. Push-out backstop: the dynamic phase runs AFTER this stage and a
 *      ram can embed a hull in a wall; the next tick ejects it along the
 *      minimum translation with the static side immovable.
 */

static void ai_world_contacts_tick(void)
{
    for (int ent = 0; ent < s_nents; ent++) {
        MissionEnt *e = &s_ents[ent];
        if (mission_ent_owner(ent) != ent || ent == s_user_ent ||
            e->scene_obj < 0 || ai_physical_active(ent) ||
            !mission_ent_is_vehicle(ent) ||
            !combat_alive(ent) || combat_is_hidden(ent))
            continue;
        double p[3];
        if (ai_get_pos(ent, p) != 0)
            continue;
        if (!e->have_ground || !e->have_prev ||
            fabs(p[1] - e->ai_y) > 1e-6)
            continue;                           /* teleport / first tick */
        double mx = p[0] - e->prev_x;
        double mz = p[2] - e->prev_z;
        if (mx == 0.0 && mz == 0.0)
            continue;

        double cx = p[0], cz = p[2];
        int authored_race = ai_race_active(ent);

        double c[3], half[2], axis[2], yspan[2];
        int have_hull = scene_obj_obb(e->scene_obj, c, half, axis,
                                      yspan) == 0 &&
                        half[0] > 0.0 && half[1] > 0.0;

        /* 1 — ground-step limits, ground-contacted movers only. A mover
         * whose hull bottom rides within one max-step of the terrain is
         * ground-contacted; an airborne mover (a helicopter's authored
         * hover clearance puts its hull metres above the terrain) is
         * exempt — it overflies the discontinuity exactly like it
         * overflies a static box through the y-span test below.
         * Sampled at both ends and mid-segment so a spike between the
         * endpoints cannot be skipped: a rise past AI_WORLD_MAX_STEP or
         * a drop past AI_WORLD_MAX_DROP between consecutive samples
         * refuses the whole move (ai.h D-A12). */
        {
            double h0 = terrain_height_at(e->prev_x, e->prev_z);
            int grounded = have_hull
                ? (yspan[0] <= h0 + AI_WORLD_MAX_STEP)
                : (fabs(e->ground_clearance) <= AI_WORLD_MAX_STEP);
            if (grounded && !authored_race) {
                double hs[3];
                hs[0] = h0;
                hs[1] = terrain_height_at(e->prev_x + mx * 0.5,
                                          e->prev_z + mz * 0.5);
                hs[2] = terrain_height_at(cx, cz);
                int refuse = 0;
                /* D-A21: a mover on a stunt-descent goto leg may
                 * DESCEND freely — the authored stunt (P01 navjump) is
                 * a fall a physical car would take, and refusing it
                 * wedges the mission's own actor at the cliff lip.
                 * Climbs are refused as always: the fallback must never
                 * become a wall-scaling licence. */
                int fall_ok = ai_goto_stunt_descent(ent);
                for (int k = 1; k < 3 && !refuse; k++) {
                    double d = hs[k] - hs[k - 1];
                    if (d > AI_WORLD_MAX_STEP ||
                        (-d > AI_WORLD_MAX_DROP && !fall_ok))
                        refuse = 1;
                }
                if (refuse) {
                    cx = e->prev_x;
                    cz = e->prev_z;
                    mx = 0.0;
                    mz = 0.0;
                    e->world_terrain_refusals++;
                }
            }
        }

        /* 2 — static sweep + one slide retry. ok accumulates the accepted
         * motion; rem is the untested remainder. A round with no contact
         * accepts the remainder whole. A contact truncates at the face and
         * projects the remainder onto its tangent. Near-head-on projection
         * would be zero forever because the path goal keeps pulling through
         * the wall, so preserve the blocked distance along the clockwise
         * tangent; repeated ticks carry the mover around the nearest corner.
         * A second contact keeps whatever slide was still pending. */
        if (!authored_race && have_hull && s_nworld_boxes > 0 &&
            (mx != 0.0 || mz != 0.0)) {
            double okx = 0.0, okz = 0.0;
            double remx = mx, remz = mz;
            int contacted = 0;
            for (int round = 0;
                 round < 2 && (remx != 0.0 || remz != 0.0); round++) {
                double bx = c[0] + okx, bz = c[2] + okz;
                double best_t = 2.0, bnx = 0.0, bnz = 0.0;
                for (int k = 0; k < s_nworld_boxes; k++) {
                    const MissionWorldBox *b = &s_world_boxes[k];
                    if (yspan[1] < b->y0 || yspan[0] > b->y1)
                        continue;               /* car.h D19 height test */
                    double nx, nz;
                    double t = world_sweep_box(bx, bz, remx, remz, b,
                                               half[0], half[1],
                                               axis[0], axis[1],
                                               &nx, &nz);
                    if (t >= 0.0 && t < best_t) {
                        best_t = t;
                        bnx = nx;
                        bnz = nz;
                    }
                }
                if (best_t > 1.0) {
                    okx += remx;
                    okz += remz;
                    break;
                }
                okx += remx * best_t;
                contacted = 1;
                okz += remz * best_t;
                double rx = remx * (1.0 - best_t);
                double rz = remz * (1.0 - best_t);
                const double tx = -bnz, tz = bnx;
                double tangent = rx * tx + rz * tz;
                double blocked = hypot(rx, rz);
                signed char *side = &s_world_detour[ent];
                if (*side == 0)
                    *side = tangent < 0.0 ? -1 : 1;
                tangent = *side * fmax(fabs(tangent), blocked);
                remx = tx * tangent;
                remz = tz * tangent;
            }
            if (!contacted)
                s_world_detour[ent] = 0;
            if (contacted)
                e->world_static_contacts++;
            cx = e->prev_x + okx;
            cz = e->prev_z + okz;
        }

        /* 3 — push-out backstop for shove-embedded hulls */
        if (!authored_race && have_hull && s_nworld_boxes > 0) {
            const double ox = c[0] - e->prev_x;
            const double oz = c[2] - e->prev_z;
            MissionVehicleBody agent, wall;
            memset(&agent, 0, sizeof agent);
            agent.cx = cx + ox;
            agent.cz = cz + oz;
            agent.hx = half[0];
            agent.hz = half[1];
            agent.ax = axis[0];
            agent.az = axis[1];
            agent.y0 = yspan[0];
            agent.y1 = yspan[1];
            int pushed_out = 0;
            for (int k = 0; k < s_nworld_boxes; k++) {
                const MissionWorldBox *b = &s_world_boxes[k];
                if (agent.y1 < b->y0 || agent.y0 > b->y1)
                    continue;
                memset(&wall, 0, sizeof wall);
                wall.cx = b->x;
                wall.cz = b->z;
                wall.hx = b->hx;
                wall.hz = b->hz;
                wall.ax = b->ax;
                wall.az = b->az;
                wall.y0 = b->y0;
                wall.y1 = b->y1;
                double nx, nz, depth;
                if (!vehicle_obb_contact(&agent, &wall, &nx, &nz, &depth))
                    continue;
                /* n points agent -> wall; only the agent moves. */
                vehicle_body_shift(&agent, -nx * depth, -nz * depth);
                pushed_out = 1;
            }
            if (pushed_out)
                e->world_pushouts++;
            cx = agent.cx - ox;
            cz = agent.cz - oz;
        }

        if (cx != p[0] || cz != p[2])
            ai_constrain_xz(ent, cx - p[0], cz - p[2]);
    }
}

/* Host session rules (mission_set_rules) — deliberately outside the
 * runner reset: the host picks them before the mission loads and they
 * outlive it, exactly like the original's game-setup screen. */
static int           s_rule_laps     = RACE_LAPS_DEFAULT;
static int           s_rule_captures = CTF_CAPTURES_DEFAULT;
static int           s_rule_kills    = MELEE_KILLS_DEFAULT;
static int           s_rule_minutes  = RACE_MINUTES_DEFAULT;

/* Cutscene camera stack (D18). */
typedef struct {
    int    active;              /* host should use cutscene cam         */
    int    depth;               /* pushCam stack depth                  */
    CameraView stack[MISSION_CAM_STACK];
    CameraView cur;             /* live native-compatible basis         */
    int    path_i;              /* >=0 while camTrans* is travelling    */
    int    path_n;
    const float *path_pts;      /* borrowed from FsmImage; x,y,z floats */
    double path_s;              /* arc-length progress along path (m)   */
    double path_speed;          /* m/s                                  */
    double path_height;         /* meters above terrain/path y          */
    int    path_watch;          /* entity to look at, or -1             */
    int    path_fixed_direction;/* fixed Euler basis rather than target */
    double path_angles[3];      /* literal direction slots, centidegrees */
    int    arrived_pulse;       /* camIsArrived self-clearing (Open76)  */
} MissionCam;

static MissionCam    s_cam;

/* ----------------------------------------------------------------------- */
/* Small helpers                                                           */
/* ----------------------------------------------------------------------- */

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8));
}

static float lef32(const uint8_t *p)
{
    uint32_t b = le32(p);
    float f;
    memcpy(&f, &b, 4);
    return f;
}

static void msg_set(const char *s)
{
    snprintf(s_message, sizeof s_message, "%s", s);
    s_have_message = 1;
}

static uint32_t rng_next(void)
{
    /* xorshift32 (D10) */
    uint32_t x = s_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_rng = x;
    return x;
}

/* scene.md §3.2 packed label: 7 ASCII bits per byte, high bits form the
 * id — identical to scene.c's label_unpack (kept in sync; D3). */
static void label_unpack(const uint8_t *p, char *name, int *id_out)
{
    int id = 0, n = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t b = p[i];
        if (b & 0x80) id = (id << 1) | 1;
        else          id = (id << 1) & 0xFE;
        char ch = (char)(b & 0x7F);
        if (ch && n < 8) name[n++] = ch;
    }
    name[n] = '\0';
    *id_out = id;
}

/* ----------------------------------------------------------------------- */
/* BWD2 chunk walking (minimal clone of scene.c's static walker; D3)       */
/* ----------------------------------------------------------------------- */

typedef struct {
    uint32_t tag;
    size_t   payload;   /* offset of payload                             */
    size_t   total;     /* header + payload bytes                        */
    size_t   next;      /* offset of the following chunk                 */
} MChunk;

static int mchunk_at(const uint8_t *b, size_t len, size_t off, MChunk *c)
{
    if (off + 8 > len)
        return 0;
    c->tag     = le32(b + off);
    c->total   = le32(b + off + 4);
    if (c->total < 8 || off + c->total > len)
        return 0;
    c->payload = off + 8;
    c->next    = off + c->total;
    return 1;
}

static int mtag_is(const MChunk *c, const char t[4])
{
    return memcmp(&c->tag, t, 4) == 0;
}

/* 'FSM' prefix match — "FSM " per spec, "FSM\0" observed in N01.CBT (D2). */
static int mtag_is_fsm(const MChunk *c)
{
    const uint8_t *t = (const uint8_t *)&c->tag;
    return t[0] == 'F' && t[1] == 'S' && t[2] == 'M';
}
/* Base story-movie membership lives in the mission's WDEF/WRLD record, not
 * in the ADEF/FSM sound table. The first dword is the WRLD revision; two
 * 13-byte NUL-padded fields follow at payload +4 and +17. */
static void story_field_copy(char out[14], const uint8_t *src)
{
    size_t n = 0;
    while (n < 13 && src[n]) {
        unsigned char c = src[n];
        out[n++] = (char)tolower(c);
    }
    out[n] = '\0';
}

/* FUN_00458890: ordered NPT lines, six objectives, case-insensitive tags.
 * Failure prose belongs to the existing debrief path, not this table. */
static void notes_load(const char *name)
{
    size_t n = 0;
    uint8_t *text = vfs_exists(name) ? vfs_read_file(name, &n) : NULL;
    if (!text) return;
    for (size_t pos = 0; pos < n && s_note_count < 6; ) {
        size_t end = pos;
        while (end < n && text[end] && text[end] != '\n' && text[end] != '\r') end++;
        if (end - pos >= 9 && !strncasecmp((const char *)text + pos, "(failure)", 9))
            break;
        size_t next = end;
        if (next < n && text[next] == '\r') next++;
        if (next < n && text[next] == '\n') next++;
        if (next == pos) break;
        /* FUN_004a2690 skips separator whitespace: blank lines take no slot. */
        size_t first = pos;
        while (first < end && isspace(text[first])) first++;
        if (first < end) {
            pos = first;
            MissionNote *note = &s_notes[s_note_count++];
            if (end - pos >= 8 && !strncasecmp((const char *)text + pos, "(hidden)", 8)) {
                note->flags = 1;
                pos += 8;
                while (pos < end && isspace(text[pos])) pos++;
            }
            size_t len = end - pos;
            if (len >= sizeof note->text) len = sizeof note->text - 1;
            memcpy(note->text, text + pos, len);
            note->text[len] = '\0';
        }
        if (end < n && !text[end]) break;
        pos = next;
    }
    vfs_free(text);
}

const MissionNote *mission_note(int objective_id)
{
    return objective_id > 0 && objective_id <= s_note_count
         ? &s_notes[objective_id - 1] : NULL;
}

static void story_fields_load(const uint8_t *wdef, size_t len)
{
    for (size_t off = 0; off < len; ) {
        MChunk c;
        if (!mchunk_at(wdef, len, off, &c))
            break;
        if (mtag_is(&c, "WRLD")) {
            if (c.total - 8 >= 30) {
                const uint8_t *p = wdef + c.payload;
                story_field_copy(s_story_clip[MISSION_STORY_INTRO], p + 4);
                story_field_copy(s_story_clip[MISSION_STORY_OUTRO], p + 17);
            }
            if (c.total - 8 >= 82) {
                char name[14];
                story_field_copy(name, wdef + c.payload + 69);
                if (name[0]) notes_load(name);
            }
            return;
        }
        if (mtag_is(&c, "EXIT"))
            return;
        off = c.next;
    }
}


/* ----------------------------------------------------------------------- */
/* ODEF object table (D3)                                                  */
/* ----------------------------------------------------------------------- */

static void obj_table_add(const char *label, int label_id, uint32_t class_id,
                          uint16_t flags, uint16_t team, const double pos[3],
                          double yaw, int has_yaw)
{
    MissionObj *o;
    if (s_nobj == s_objcap) {
        int ncap = s_objcap ? s_objcap * 2 : 64;
        MissionObj *no = realloc(s_objs, (size_t)ncap * sizeof *no);
        if (!no)
            return;
        s_objs = no;
        s_objcap = ncap;
    }
    o = &s_objs[s_nobj++];
    snprintf(o->label, sizeof o->label, "%s", label);
    o->label_id = label_id;
    o->class_id = class_id;
    o->flags    = flags;
    o->team     = team;
    o->pos[0]   = pos[0];
    o->pos[1]   = pos[1];
    o->pos[2]   = pos[2];
    o->yaw      = yaw;
    o->has_yaw  = has_yaw;
}

/* ODEF OBJ record, 100-byte payload (scene.md §3.1 — same offsets scene.c
 * uses): label[8] @0, 12×f32 frame @0x08 (right 0..2, up 3..5, forward
 * 6..8, pos 9..11 — scene.c's xform_from_frame reads the same quadruple),
 * class u32 @0x5C, flags u16 @0x60, team u16 @0x62.
 *
 * D-O2: the frame's FORWARD vector is kept as a yaw. The table used to
 * read elements 9..11 and throw the basis away, which was fine while the
 * only consumer was the FSM's XZ distance checks — but an arena mission
 * has no FSM to place the player, and dropping the basis is why the race
 * grid could only be faced north. The heading is real data, not a
 * flourish: every `spawn` marker in all 13 .RAC files carries a non-
 * identity basis, and in 12 of the 13 it points within 10 degrees of
 * check1 (the exception is N23.RAC, the drag strip).
 */
static void odef_walk(const uint8_t *b, size_t len)
{
    size_t off = 0;
    while (off < len) {
        MChunk c;
        if (!mchunk_at(b, len, off, &c))
            break;
        if (mtag_is(&c, "OBJ\0")) {
            const uint8_t *p = b + c.payload;
            size_t avail = c.total - 8;
            if (avail >= 100) {
                char label[9];
                int id;
                double pos[3], yaw = 0.0;
                label_unpack(p, label, &id);
                pos[0] = lef32(p + 0x08 + 9 * 4);
                pos[1] = lef32(p + 0x08 + 10 * 4);
                pos[2] = lef32(p + 0x08 + 11 * 4);
                /* car.h: forward = (-sin yaw, 0, cos yaw). A marker whose
                 * forward is vertical or zero has no heading in XZ — say
                 * so rather than reporting a yaw of atan2(0,0). */
                double fx = lef32(p + 0x08 + 6 * 4);
                double fz = lef32(p + 0x08 + 8 * 4);
                int has_yaw = (fx * fx + fz * fz) > 1e-6;
                if (has_yaw)
                    yaw = atan2(-fx, fz);
                obj_table_add(label, id, le32(p + 0x5C), le16(p + 0x60),
                              le16(p + 0x62), pos, yaw, has_yaw);
            }
        } else if (mtag_is(&c, "EXIT")) {
            break;
        }
        off = c.next;
    }
}

/* ----------------------------------------------------------------------- */
/* Objective controller for the FSM-less arena families (D-O1..D-O7)       */
/*                                                                          */
/* 55 of the 75 shipped Nitro missions carry an EMPTY `FSM ` payload. They  */
/* loaded and they drove, and that was all: mission_tick() returned on      */
/* !s_img, so there was no win, no loss and no debrief for any of them.     */
/* This unit is their objective controller. It reads the ODEF marker table  */
/* the loader has already built (s_objs) and drives s_state, so the win and */
/* the loss arrive through the SAME mission_state() the shell already polls.*/
/*                                                                          */
/*  D-O1  The controller runs only when there is no FSM image. A scripted   */
/*        mission's FSM owns its objectives; two authorities writing        */
/*        s_state would race, and the FSM would lose whenever the fallback  */
/*        rule fired first. mission_family() therefore reports TRIP for any */
/*        mission that HAS an FSM, whatever its extension says.             */
/*  D-O2  Marker headings come from the ODEF frame's forward vector (see    */
/*        odef_walk).                                                       */
/*  D-O3  RACE_CHECK_R is DATA (30 m, 3-D) — see its definition.         */
/*  D-O4  The lap target and the clock are INVENTED session defaults — see  */
/*        RACE_LAPS_DEFAULT / mission_set_rules.                            */
/*  D-O5  check99 (N23.RAC only, of all 75 missions) is THE FINISH LINE —
 *        DATA, no longer a guess. The loader FUN_00444ad0 remaps the
 *        marker name check99 to slot 100 and sets a has-finish flag
 *        (DAT_0053ec94); the gate FUN_00444bb0 then runs the course as
 *        check1..checkN followed by check99, and the LAP COMPLETES AT
 *        check99, not at checkN (disassembly-verified: next slot > max
 *        becomes 99 with the flag set, wraps to check1 without it;
 *        reaching the target inside the 30 m sphere is what counts the
 *        lap and re-arms check1). N23.RAC ("Drag Race") is a straight
 *        450 m strip — check1..check3 then check99 145 m past check3 —
 *        and the port used to end the race one gate early. */
/*  D-O6  Order matters: gates must be taken 1..N. The data supports it —   */
/*        the checkpoints are numbered, and the smallest gap between        */
/*        consecutive ones anywhere in the set is 82 m against a 30 m       */
/*        radius, so an in-order rule cannot be satisfied by accident.      */
/*        Whether the original enforced order is NOT established.           */
/*  D-O7  MELEE gets NO controller here, deliberately. It cannot be won:    */
/*        all 24 .CBT files together contain 226 `spawn` and 37 `regen`     */
/*        markers and ZERO opponent vehicles, so there is nothing in the    */
/*        data to kill. Arming the CLOCK alone would ship a mission that    */
/*        can only be LOST, which is worse for a player than today's        */
/*        drive-and-quit, so melee is not ticked at all. It does get its    */
/*        start pose (mission_start_pose), which is a strict improvement    */
/*        and costs no objective claim. CAPTURE used to sit here too; it    */
/*        does not any more — see D-O8.                                     */
/*                                                                          */
/*  D-O8  CAPTURE's course is read from `#spawn<team>` and `a1flag<team>`,  */
/*        and TEAM IDENTITY COMES FROM THE LABEL SUFFIX, not from the ODEF  */
/*        team field at +0x62. The suffix is the only self-consistent       */
/*        source in the shipped data: +0x62 is nonzero in just 4 of the 17  */
/*        .CF* files (N11, N19, N44, N45) and in two of those it CONTRADICTS*/
/*        the suffix — N44's `#spawn2` records carry team 3, N45's carry    */
/*        team 4 for `#spawn2` and 3 for `#spawn3`. A disagreement is       */
/*        logged, once per record, rather than silently preferred.          */
/*        The suffix reading is corroborated by the extension: every .CF2   */
/*        carries exactly teams 1-2, every .CF3 teams 1-3, every .CF4 teams */
/*        1-4, and all 46 flags across the 17 maps are present.             */
/*                                                                          */
/*  D-O9  A BASE IS SCORED BY THE NEAREST SLOT, NEVER BY A CENTROID.        */
/*        N46.CF2's `#spawn2` is two clusters ~620 m apart carrying         */
/*        DUPLICATE label ids (two `#0` and two `#1` records), and its      */
/*        `#spawn1` is similar: the centroid of either lands 282-286 m from */
/*        that team's own flag, i.e. outside any sane radius, so a centroid */
/*        would make N46 unwinnable while looking correct on the other 16.  */
/*                                                                          */
/*  D-O10 CTF_TOUCH_R is INVENTED — see its definition.                     */
/*  D-O11 The capture target is an INVENTED session default — see           */
/*        CTF_CAPTURES_DEFAULT / mission_set_rules. So is the rule that     */
/*        AT MOST ONE FLAG rides at a time: nothing in the data states it,  */
/*        it is the universal CTF convention, and without it a 4-team map   */
/*        could be swept in one run.                                        */
/*  D-O12 The player is team 1 (INVENTED — see CTF_PLAYER_TEAM). The      */
/*        team DISPLAY NAMES are nitro.exe's own pool, and WHICH pool     */
/*        entry belongs to which team slot is now DATA — see              */
/*        mission_team_name (decompiled pointer table @0x4f30b8).         */
/*  D-O13 A captured flag goes straight back to its own stand and can be    */
/*        taken again, which is what makes a target above 1 reachable on a  */
/*        two-team map. nitro.exe's "*** %s's flag has been returned to its */
/*        base!" is deliberately NOT used for that: in the original it      */
/*        announces a DROPPED flag timing out, and nothing here can drop    */
/*        one, because a capture map spawns no opponents to destroy the     */
/*        carrier — melee's spawner (D-O14) is not wired into ctf_build.    */
/*        A string that would only ever be a lie is better left unprinted.  */
/*                                                                          */
/*  D-O14 MELEE HAS OPPONENTS NOW, AND THEY ARE SPAWNED, NOT SHIPPED.       */
/*        D-O7 above is superseded. Its reading of the data was right and    */
/*        is unchanged — all 24 .CBT files together hold 226 `spawn` and 37  */
/*        `regen` class-1 markers and NOTHING ELSE of class 1 (263 = 226+37, */
/*        measured over every file), so no melee map ships a single opponent */
/*        vehicle. What changed is that the grid slots are now FILLED.       */
/*                                                                          */
/*        That is what the original does too: nitro.exe carries a SpawnLoc   */
/*        module whose strings are "spawn", "spawn%d",                       */
/*        "SpawnLoc_GetLocation - no spawn points for team %d", "Error: No   */
/*        spawn locations in mission!", "Cannot find a valid spawn           */
/*        location!" and "No open slots for net vehicles" — a pool of empty  */
/*        slots that arriving vehicles are placed into. The original filled  */
/*        it from the network session; melee_build fills it from             */
/*        MELEE_BOT_VCF (see D-O14's constant for why that vehicle).         */
/*                                                                          */
/*        A slot becomes a car through scene_obj_make_car, i.e. through the  */
/*        SAME VCF->VDF->WDF chain every scripted mission's cars take, so an */
/*        opponent is a real placed object with real geometry, real hide-on- */
/*        death and real AI write-back — not a marker with an HP counter     */
/*        bolted on.                                                        */
/*                                                                          */
/*  D-O15 THE CONTROLLER DISARMS ITSELF, CLOCK AND ALL, unless it actually   */
/*        fielded an opponent the player can destroy. A map with fewer than  */
/*        two usable slots, or a MELEE_BOT_VCF that does not resolve to      */
/*        geometry, leaves the mission RUNNING forever with no clock —       */
/*        exactly what ctf_build does for an unplayable capture map, and for */
/*        the same reason D-O7 refused to ship melee at all: a mission that  */
/*        can only be LOST is worse for a player than one that simply runs.  */
/*                                                                          */
/*  D-O16 THE PLAYER IS A COMBAT ENTITY IN AN ARENA FOR THE FIRST TIME.      */
/*        combat_register/combat_set_user used to be reachable only from     */
/*        resolve_entities(), which returns early without an FSM image, so   */
/*        in any melee/race/capture mission the player had no HP pool and    */
/*        the HUD's damage readout was blank. melee_build registers the      */
/*        player (entity 0) and the bots on that non-FSM path. Race and      */
/*        capture still do not — nothing shoots there.                       */
/*                                                                          */
/*  D-O17 Opponent COUNT, kill TARGET and team layout are INVENTED — see     */
/*        MELEE_BOTS_DEFAULT / MELEE_KILLS_DEFAULT / MELEE_BOT_TEAM.         */
/*                                                                          */
/*  D-O18 A melee is LOST by dying, and by the clock. Death is the melee-    */
/*        only half and it is the reason the clock is safe to arm here at    */
/*        last. Both print nitro.exe's own lines: "You lose, chump!" for a   */
/*        destroyed player, "*** Time limit has elapsed!" for the clock —    */
/*        the second shared with the race and the capture map.               */
/*                                                                          */
/*  D-O19 NO RESPAWN, for either side — INVENTED, and it is an ABSENCE      */
/*        rather than a constant, which is why it carries the word here     */
/*        and in coverage-ledger.md §7 instead of at a #define. The 37       */
/*        `regen` markers are almost                                        */
/*        certainly the original's respawn pool (they are class-1 markers    */
/*        like `spawn`, and the binary tracks a Deaths column), but nothing  */
/*        establishes the timing, the invulnerability window or whether a    */
/*        respawned kill re-scores. A dead opponent stays dead and a dead    */
/*        player has lost. CONFIRM BY: decompiling the SpawnLoc caller.      */
/*                                                                          */
/*  D-O20 The player has no NAME to put in the win line — see               */
/*        MELEE_PLAYER_NAME. The opponents' names are nitshell.dll's own    */
/*        driver pool; which name goes with which opponent is INVENTED —    */
/*        see melee_driver_name.                                            */
/* ----------------------------------------------------------------------- */

/* Defined below; combat's position resolver points at it (D17). */
static void ent_pos(int ent, double out[3]);

/* First table entry with this label, case-insensitively. Case matters:
 * .RAC files spell the grid "SPAWN" and "spawn" in different missions,
 * and N13.CF2 mixes both spellings inside one file. */
static const MissionObj *obj_find_ci(const char *label)
{
    for (int i = 0; i < s_nobj; i++)
        if (strcasecmp(s_objs[i].label, label) == 0)
            return &s_objs[i];
    return NULL;
}

/* The lowest-numbered instance of a repeated label — grid slot 0 of a
 * race, base slot 0 of a capture team. Keyed on label_id, NOT on file
 * order, and ties keep the first record: label ids are not unique
 * everywhere (N46.CF2 repeats `#spawn2#0`), so this must not assume they
 * are. */
static const MissionObj *obj_find_slot0(const char *label)
{
    const MissionObj *best = NULL;
    for (int i = 0; i < s_nobj; i++) {
        if (strcasecmp(s_objs[i].label, label) != 0)
            continue;
        if (!best || s_objs[i].label_id < best->label_id)
            best = &s_objs[i];
    }
    return best;
}

/*
 * D-O12: the team display names.
 *
 * nitro.exe carries a TWELVE-name team pool, verbatim in this image
 * order: "Black Plague", "Purple Reign", "Gang Green", "Red Dawn",
 * "Men In Black", "Mr. Brown's Clowns", "Mauve Mayhem", "Aqua Marines",
 * "Yellow Jackets", "Crimson Kings", "Puce Panthers", "Grey Hounds".
 * They are what fills the %s in the original's own "Team %s",
 * "*** %s's flag has been taken!" and "*** Team %s has captured %s's
 * flag!". A .CF* map has at most four teams, so only the first four are
 * needed here; the other eight are in coverage-ledger.md §6 so the pool
 * is recorded whole rather than half-quoted.
 *
 * WHICH NAME BELONGS TO TEAM 1 IS DATA (2026-08-10, decompiled). Both
 * binaries carry the pool as a POINTER TABLE, and the table runs in
 * REVERSE image order — the earlier suspicion that literal image order
 * proves nothing was right. nitro.exe @0x4f30b8 (mirrored by
 * nitshell.dll @0x100458d0) is the four-entry CTF table used by the
 * flag-state machine (FUN_00444f80/FUN_004451d0), indexed by the 3-bit
 * team field straight out of the network record:
 *
 *     [0] "Red Dawn"  [1] "Gang Green"  [2] "Purple Reign"  [3] "Black Plague"
 *
 * The .CF* markers name the teams 1..4 (#spawn1..#spawn4), so the
 * port's 1-based team N is the table's index N-1: team 1 is "Red Dawn".
 * The port previously assigned the pool in forward image order — exactly
 * backwards. (The sibling TWELVE-entry table @0x4f3098, selected for the
 * non-CTF modes, likewise runs reverse image order: [0] "Grey Hounds"
 * .. [11] "Black Plague".)
 *
 * "Unknown Loser" is not a joke of ours either; it is the binary's own
 * fallback for a player with no name.
 */
const char *mission_team_name(int team)
{
    static const char *const NAMES[] = {
        "Red Dawn", "Gang Green", "Purple Reign", "Black Plague",
    };
    if (team >= 1 && team <= (int)(sizeof NAMES / sizeof NAMES[0]))
        return NAMES[team - 1];
    return "Unknown Loser";
}

/*
 * The opponents' driver names (D-O17).
 *
 * nitshell.dll carries a pool of FIFTY-THREE driver names in one contiguous
 * block of 4-byte-aligned C strings, "Zippy" through "Torque Daisy",
 * immediately followed by "Unknown". They are the names an I'76 driver goes
 * by; the first MELEE_MAX_BOTS of them, in image order, are used here
 * verbatim. The whole 53 are listed in coverage-ledger.md §7 so the pool is
 * recorded intact rather than half-quoted, exactly as the twelve capture
 * team names are.
 *
 * The count was 52 here until the adversarial verifier counted the bytes
 * rather than the output of `strings`, whose DEFAULT MINIMUM LENGTH IS 4
 * and which therefore drops "Irv" — the pool's fourth name, three
 * characters long, sitting between "Wicked" and "Glytch" at nitshell.dll
 * +0x43ca8. This table had "Glytch" in the fourth slot as a result. Nothing
 * a player has seen was wrong (MELEE_BOTS_DEFAULT is 3, so only the first
 * three names have ever been printed), but every slot from the fourth on
 * was off by one against the image order it claims to reproduce. Count a
 * packed string table with `strings -n 1`, or by walking the bytes.
 *
 * WHICH NAME BELONGS TO WHICH OPPONENT IS INVENTED, for the same reason
 * mission_team_name says so: literal order in the image is not proof of
 * array order (nitshell.dll emits its own game-mode list in reverse of the
 * order the shell offers it). CONFIRM BY: decompiling the pool's consumer.
 *
 * "Unknown" is the pool's own trailing fallback and is used when a slot
 * index runs past the names carried here.
 */
static const char *melee_driver_name(int i)
{
    static const char *const NAMES[MELEE_MAX_BOTS] = {
        "Zippy", "Redwood", "Wicked", "Irv",
        "Glytch", "Quillery", "James Nicely", "Mad Hooper",
    };
    if (i >= 0 && i < MELEE_MAX_BOTS)
        return NAMES[i];
    return "Unknown";
}

int mission_family_of_path(const char *path)
{
    const char *dot = path ? strrchr(path, '.') : NULL;
    if (!dot)
        return MISSION_FAMILY_TRIP;
    if (strcasecmp(dot, ".rac") == 0)
        return MISSION_FAMILY_RACE;
    if (strcasecmp(dot, ".cbt") == 0)
        return MISSION_FAMILY_MELEE;
    if (strcasecmp(dot, ".cf2") == 0 || strcasecmp(dot, ".cf3") == 0 ||
        strcasecmp(dot, ".cf4") == 0)
        return MISSION_FAMILY_CAPTURE;
    return MISSION_FAMILY_TRIP;
}

/* The player's start pose for an arena mission: grid slot 0 and the
 * heading its ODEF frame carries. CAPTURE uses the player team's own
 * `#spawn<team>` base (D-O8/D-O12). */
static void objectives_start_pose(void)
{
    const MissionObj *sp = NULL;
    if (s_family == MISSION_FAMILY_CAPTURE) {
        char lab[16];
        snprintf(lab, sizeof lab, "#spawn%d", CTF_PLAYER_TEAM);
        sp = obj_find_slot0(lab);
    }
    if (!sp)
        sp = obj_find_slot0("spawn");
    if (!sp)
        return;

    s_start_pos[0] = sp->pos[0];
    s_start_pos[1] = sp->pos[1];
    s_start_pos[2] = sp->pos[2];
    s_start_yaw    = sp->yaw;
    s_start_ok     = 1;

    /* A marker with no heading in XZ still has to face somewhere useful:
     * on a race, the first gate. Nothing in the shipped data takes this
     * path today (every .RAC grid slot carries a heading) — it exists so
     * a missing basis degrades to "aimed at the course" instead of
     * "aimed north". */
    if (!sp->has_yaw && s_ncheck > 0) {
        s_start_yaw = atan2(-(s_course[0].x - sp->pos[0]),
                              s_course[0].z - sp->pos[2]);
        fprintf(stdout, "[mission] %s: spawn marker has no heading — "
                        "aiming at check1\n", s_mission);
    }
}

/*
 * Build the capture course from the ODEF marker table (D-O8).
 *
 * The probe loop mirrors the original's own: nitro.exe formats spawn
 * points with "spawn%d" and checkpoints with "check%d", and the .CF*
 * files spell the team bases "#spawn1".."#spawn4" (the '#' appears on
 * team spawn groups only — every .RAC grid slot is a bare "spawn"). All
 * 17 files agree with their extension: 8 .CF2 carry teams 1-2, 6 .CF3
 * carry 1-3, 3 .CF4 carry 1-4, and all 46 a1flag markers are present.
 */
static void ctf_build(void)
{
    memset(s_ctf, 0, sizeof s_ctf);
    s_ctf_teams = 0;

    int dropped = 0, team_field_disagrees = 0;
    for (int t = 1; t <= MISSION_CTF_TEAMS; t++) {
        MissionCtfTeam *ct = &s_ctf[t - 1];
        char lab[16];

        snprintf(lab, sizeof lab, "#spawn%d", t);
        for (int i = 0; i < s_nobj; i++) {
            if (strcasecmp(s_objs[i].label, lab) != 0)
                continue;
            /* D-O8: the suffix wins, but a populated +0x62 that disagrees
             * is evidence about the format and is not swallowed. */
            if (s_objs[i].team != 0 && (int)s_objs[i].team != t)
                team_field_disagrees++;
            if (ct->nslots < MISSION_CTF_SLOTS) {
                ct->sx[ct->nslots] = s_objs[i].pos[0];
                ct->sz[ct->nslots] = s_objs[i].pos[2];
                ct->nslots++;
            } else {
                dropped++;
            }
        }

        snprintf(lab, sizeof lab, "a1flag%d", t);
        const MissionObj *f = obj_find_ci(lab);
        if (f) {
            ct->has_flag = 1;
            ct->fx = f->pos[0];
            ct->fz = f->pos[2];
        }
        if (ct->nslots > 0 || ct->has_flag)
            s_ctf_teams = t;
    }

    if (team_field_disagrees)
        fprintf(stdout, "[mission] %s: %d #spawn record(s) carry an ODEF "
                        "team field that disagrees with the label suffix — "
                        "suffix wins (D-O8)\n",
                s_mission, team_field_disagrees);
    if (dropped)
        fprintf(stderr, "[mission] %s: %d base slot(s) past the %d-slot cap "
                        "were dropped\n", s_mission, dropped,
                MISSION_CTF_SLOTS);

    /* Two ways this map cannot be played, and both must disarm the whole
     * controller — including the CLOCK, or the mission becomes one that
     * can only be lost (the same reason melee is not ticked at all). */
    const MissionCtfTeam *me = &s_ctf[CTF_PLAYER_TEAM - 1];
    int enemy_flags = 0;
    for (int t = 1; t <= s_ctf_teams; t++)
        if (t != CTF_PLAYER_TEAM && s_ctf[t - 1].has_flag)
            enemy_flags++;

    if (me->nslots == 0 || enemy_flags == 0) {
        fprintf(stderr, "[mission] %s: CAPTURE with %d base slot(s) for the "
                        "player and %d enemy flag(s) — no objective "
                        "controller, mission cannot end\n",
                s_mission, me->nslots, enemy_flags);
        s_ctf_teams = 0;
        return;
    }

    fprintf(stdout, "[mission] %s: CAPTURE — %d teams, player is team %d "
                    "(%s) with %d base slot(s), %d enemy flag(s), "
                    "%d capture(s) to win, %d min limit\n",
            s_mission, s_ctf_teams, CTF_PLAYER_TEAM,
            mission_team_name(CTF_PLAYER_TEAM), me->nslots, enemy_flags,
            s_capture_target, s_rule_minutes);
}

/*
 * Field the melee opponents (D-O14..D-O19).
 *
 * The grid is the map's `spawn` markers in ODEF file order: slot 0 is the
 * player (the same slot objectives_start_pose puts them on), and every slot
 * after it takes an opponent until MELEE_BOTS_DEFAULT are fielded or the
 * grid runs out. Each opponent is
 *
 *   - dressed in MELEE_BOT_VCF through scene_obj_make_car, i.e. through the
 *     ordinary placement chain, so it is a real drawable object;
 *   - registered with the AI brain and a physical context at the slot pose;
 *   - registered with the combat model on team MELEE_BOT_TEAM, carrying the
 *     scene object so death hides the body;
 *   - pointed at the player with combat_attack, which is what makes it
 *     chase and shoot (combat.h D-C5).
 *
 * The PLAYER is registered too, as entity 0 with scene_obj -1 (the player's
 * on-screen body is the sim car, scene_set_dynamic, not a placed object).
 * That is the first time an arena mission has given the player an HP pool,
 * which is also why the HUD's damage readout was blank in one until now.
 *
 * Leaves s_melee_bots 0 — no controller, and objectives_init then disarms
 * the clock — whenever the map cannot field a fight (D-O15).
 */
static void melee_build(void)
{
    memset(s_melee, 0, sizeof s_melee);
    s_melee_bots = 0;

    /* Drop any previous entity table FIRST, so every exit path below —
     * including the two that give up — leaves the same consistent state.
     * mission_runner_reset has already done this on the production path;
     * a caller that re-runs objectives_init on a loaded mission has not. */
    free(s_ents);
    s_ents = NULL;
    s_nents = 0;
    s_user_ent = -1;

    /* The grid, in file order. Index 0 is the player's slot. Only the
     * first MELEE_MAX_BOTS+1 are kept — the maps carry up to 17 — but the
     * whole grid is counted so the log reports the map, not the cap. */
    int slots[MELEE_MAX_BOTS + 1];
    int nslot = 0, ngrid = 0;
    for (int i = 0; i < s_nobj; i++) {
        if (strcasecmp(s_objs[i].label, "spawn") != 0)
            continue;
        ngrid++;
        if (nslot < MELEE_MAX_BOTS + 1)
            slots[nslot++] = i;
    }

    if (nslot < 2) {
        fprintf(stderr, "[mission] %s: MELEE with %d spawn slot(s) — no "
                        "objective controller, mission cannot end\n",
                s_mission, ngrid);
        return;
    }

    int want = MELEE_BOTS_DEFAULT;
    if (want > nslot - 1)
        want = nslot - 1;
    if (want > MELEE_MAX_BOTS)
        want = MELEE_MAX_BOTS;

    /* Entity table: 0 = player, 1..want = opponents. Built here rather
     * than in resolve_entities, which needs an FSM image (D-O16). */
    s_ents = calloc((size_t)(want + 1), sizeof *s_ents);
    if (!s_ents) {
        fprintf(stderr, "[mission] %s: MELEE entity table alloc failed\n",
                s_mission);
        return;
    }
    s_nents = want + 1;
    /* calloc leaves scene_obj 0, which is a VALID object index — a slot
     * whose vehicle fails to build would then claim the scene's first
     * object as its body. -1 is the "no scene object" value everywhere
     * else in this file. */
    for (int i = 0; i <= want; i++) {
        s_ents[i].body_owner = i;
        s_ents[i].scene_obj = -1;
    }

    ai_reset();
    combat_reset();

    s_ents[0].obj       = &s_objs[slots[0]];
    s_ents[0].id        = 0;
    s_ents[0].is_user   = 1;
    s_ents[0].scene_obj = -1;
    {
        const double *p = s_objs[slots[0]].pos;
        ai_agent_init(0, p[0], p[1], p[2]);
        if (s_objs[slots[0]].has_yaw)
            ai_seed_heading(0, s_objs[slots[0]].yaw); /* D-A15 seed */
        mission_ent_motion_init(0, p);
    }
    {
        CarCombatConfig cc;
        combat_register(0, MELEE_PLAYER_TEAM, 1, -1, "user",
                        car_combat_config(MELEE_BOT_VCF, &cc) == 0 ? &cc : NULL);
    }
    s_user_ent = 0;

    for (int k = 0; k < want; k++) {
        const MissionObj *o = &s_objs[slots[k + 1]];
        int ent = k + 1;
        /* Ordinal lookup, not by label id: label ids repeat in this data
         * (N02.CBT's eight slots carry five ids), and scene_obj_find would
         * hand two opponents the same body. */
        int so = scene_obj_find_nth("spawn", k + 1);
        if (so < 0) {
            fprintf(stderr, "[mission] %s: MELEE slot %d has no scene "
                            "object — opponent skipped\n", s_mission, k + 1);
            continue;
        }
        /* The ordinal correspondence between this module's ODEF snapshot
         * and scene.c's is an ASSUMPTION — two walkers over the same chunk
         * — so check it rather than trust it. A mismatch would dress the
         * wrong object and leave an opponent's body somewhere its combat
         * record is not. */
        double sp[3];
        if (scene_obj_pos(so, sp) != 0 ||
            fabs(sp[0] - o->pos[0]) > 0.5 || fabs(sp[2] - o->pos[2]) > 0.5) {
            fprintf(stderr, "[mission] %s: MELEE slot %d scene object %d is "
                            "at (%.1f, %.1f), marker is at (%.1f, %.1f) — "
                            "opponent skipped\n", s_mission, k + 1, so,
                    sp[0], sp[2], o->pos[0], o->pos[2]);
            continue;
        }
        if (scene_obj_make_car(so, MELEE_BOT_VCF) <= 0) {
            fprintf(stderr, "[mission] %s: MELEE could not build '%s' for "
                            "slot %d — opponent skipped\n",
                    s_mission, MELEE_BOT_VCF, k + 1);
            continue;
        }
        s_ents[ent].obj       = o;
        s_ents[ent].id        = ent;
        s_ents[ent].is_user   = 0;
        s_ents[ent].scene_obj = so;

        double p[3] = { o->pos[0], o->pos[1], o->pos[2] };
        p[1] = vehicle_spawn_place(ent, so, MISSION_VEHICLE_SPAWN_MELEE,
                                   p[0], p[2], p[1]);
        (void)scene_obj_set_pos(so, p);
        ai_agent_init(ent, p[0], p[1], p[2]);
        if (o->has_yaw)
            ai_seed_heading(ent, o->yaw);            /* D-A15 seed */
        if (ai_shadow_attach(ent, 1, MELEE_BOT_VCF) != 0 ||
            ai_promote_physical(ent) != 0)
            fprintf(stderr, "[mission] melee class-1 physical load failed: %s\n",
                    MELEE_BOT_VCF);
        mission_ent_motion_init(ent, p);
        {
            CarCombatConfig cc;
            combat_register(ent, MELEE_BOT_TEAM, 1, so, melee_driver_name(k),
                            car_combat_config(MELEE_BOT_VCF, &cc) == 0
                            ? &cc : NULL);
        }
        combat_attack(ent, 0);          /* D-O17: every opponent hunts you */

        s_melee[s_melee_bots].ent       = ent;
        s_melee[s_melee_bots].scene_obj = so;
        s_melee[s_melee_bots].was_alive = 1;
        s_melee_bots++;
    }

    if (s_melee_bots == 0) {
        /* D-O15: no opponent means nothing to kill. Say so and leave the
         * mission running with no clock rather than shipping a loss-only
         * mission — the exact failure D-O7 refused to ship. */
        fprintf(stderr, "[mission] %s: MELEE fielded NO opponents — no "
                        "objective controller, mission cannot end\n",
                s_mission);
        free(s_ents);
        s_ents = NULL;
        s_nents = 0;
        s_user_ent = -1;
        combat_reset();
        return;
    }

    combat_set_user(0);
    combat_set_resolver(ent_pos);

    /* D-O16: you cannot be asked for more kills than there are cars. */
    s_kill_target = s_rule_kills > 0 ? s_rule_kills : MELEE_KILLS_DEFAULT;
    if (s_kill_target > s_melee_bots) {
        fprintf(stdout, "[mission] %s: MELEE kill target %d clamped to the "
                        "%d opponent(s) fielded\n",
                s_mission, s_kill_target, s_melee_bots);
        s_kill_target = s_melee_bots;
    }

    fprintf(stdout, "[mission] %s: MELEE — %d opponent(s) in '%s' on %d of "
                    "%d grid slot(s), %d kill(s) to win, %d min limit\n",
            s_mission, s_melee_bots, MELEE_BOT_VCF, s_melee_bots + 1, ngrid,
            s_kill_target, s_rule_minutes);
}

/* Called once per load, from both mission_fsm_load exits. */
static void objectives_init(void)
{
    s_ncheck = s_next_check = s_lap = 0;
    s_has99 = 0;
    s_lap_target = 0;
    s_limit_ticks = 0;
    s_start_ok = 0;
    s_start_yaw = 0.0;
    s_start_pos[0] = s_start_pos[1] = s_start_pos[2] = 0.0;
    memset(s_ctf, 0, sizeof s_ctf);
    s_ctf_teams = s_ctf_carry = s_captures = 0;
    s_capture_target = 0;
    memset(s_melee, 0, sizeof s_melee);
    s_melee_bots = 0;
    s_kill_target = 0;

    s_family = mission_family_of_path(s_mission);

    if (s_img) {                    /* D-O1: the script owns this mission */
        s_family = MISSION_FAMILY_TRIP;
        return;
    }

    /* miss8/N35.MSN is the one .MSN in the shipped set with an empty FSM
     * payload, and it carries check1..check5 — the same course as its
     * N35.RAC twin. Its extension lies about what it is; its markers do
     * not. Nothing else can reach this branch: every other .MSN embeds a
     * 45k-68k byte FSM image. */
    if (s_family == MISSION_FAMILY_TRIP && obj_find_ci("check1")) {
        s_family = MISSION_FAMILY_RACE;
        fprintf(stdout, "[mission] %s: .MSN with no FSM and a checkpoint "
                        "course — running it as a RACE\n", s_mission);
    }

    /* The clock is shared by both arena controllers (D-O4). It is armed
     * here and disarmed again by any branch that decides it has no
     * playable objective — see ctf_build and the RACE no-course case. */
    s_limit_ticks = s_rule_minutes > 0
        ? (uint64_t)s_rule_minutes * 60u * MISSION_TICK_HZ : 0;

    if (s_family == MISSION_FAMILY_RACE) {
        for (int n = 1; s_ncheck < MISSION_MAX_CHECKS; n++) {
            char lab[16];
            snprintf(lab, sizeof lab, "check%d", n);
            const MissionObj *o = obj_find_ci(lab);
            if (!o)
                break;              /* the course ends at the first gap  */
            s_course[s_ncheck].x = o->pos[0];
            s_course[s_ncheck].y = o->pos[1];
            s_course[s_ncheck].z = o->pos[2];
            s_ncheck++;
        }
        /* D-O5: check99 is the finish line (nitro.exe FUN_00444ad0 /
         * FUN_00444bb0). When the course carries one, the lap completes
         * there, not at the last numbered gate. */
        {
            const MissionObj *o99 = obj_find_ci("check99");
            if (o99) {
                s_has99 = 1;
                s_finish[0] = o99->pos[0];
                s_finish[1] = o99->pos[1];
                s_finish[2] = o99->pos[2];
            }
        }

        s_lap_target  = s_rule_laps > 0 ? s_rule_laps : 1;

        if (s_ncheck > 0) {
            fprintf(stdout, "[mission] %s: RACE — %d checkpoints%s, %d lap(s), "
                            "%d min limit\n",
                    s_mission, s_ncheck, s_has99 ? " + check99 finish" : "",
                    s_lap_target, s_rule_minutes);
        } else {
            /* No course means no objective. Say so loudly and leave the
             * mission RUNNING forever rather than inventing a win — and
             * disarm the clock with it, or this becomes a mission that
             * can only be lost. */
            s_limit_ticks = 0;
            fprintf(stderr, "[mission] %s: RACE with NO check1 marker — no "
                            "objective controller, mission cannot end\n",
                    s_mission);
        }
    } else if (s_family == MISSION_FAMILY_CAPTURE) {
        s_capture_target = s_rule_captures > 0 ? s_rule_captures : 1;
        ctf_build();
        if (s_ctf_teams == 0)
            s_limit_ticks = 0;          /* unplayable map: no clock either */
    } else if (s_family == MISSION_FAMILY_MELEE) {
        /* D-O14: the grid slots get filled. The start pose must be
         * resolved FIRST — melee_build parks the player on grid slot 0 and
         * registers them there, and an opponent placed before the player's
         * slot is known could otherwise take it. */
        objectives_start_pose();
        melee_build();
        if (s_melee_bots == 0)
            s_limit_ticks = 0;          /* D-O15: no fight, no clock       */
        return;
    } else {
        /* The FSM-less mission with no course of any kind. No controller
         * means NO CLOCK: a mission that can only be lost is worse for a
         * player than one that simply runs. */
        s_limit_ticks = 0;
    }

    objectives_start_pose();
}

/* The XZ distance from the car to the NEAREST slot of a team's base,
 * squared. D-O9: nearest, never a centroid — N46.CF2's bases are two
 * clusters ~620 m apart and their centroids land nowhere near either.
 * Returns -1 for a team with no slots at all. */
static double ctf_base_dist2(int team)
{
    if (team < 1 || team > MISSION_CTF_TEAMS)
        return -1.0;
    const MissionCtfTeam *ct = &s_ctf[team - 1];
    double best = -1.0;
    for (int i = 0; i < ct->nslots; i++) {
        double dx = s_car_x - ct->sx[i];
        double dz = s_car_z - ct->sz[i];
        double d2 = dx * dx + dz * dz;
        if (best < 0.0 || d2 < best)
            best = d2;
    }
    return best;
}

/* The enemy team whose flag is nearest the car, with its squared XZ
 * distance. 0 when there is none. Every enemy flag is on its stand
 * whenever this is called: D-O11 allows one flag aboard at a time, so
 * the only caller that matters runs while nothing is carried, and D-O13
 * puts a captured flag straight back. */
static int ctf_nearest_enemy_flag(double *d2_out)
{
    int best_t = 0;
    double best = 0.0;
    for (int t = 1; t <= s_ctf_teams; t++) {
        if (t == CTF_PLAYER_TEAM || !s_ctf[t - 1].has_flag)
            continue;
        double dx = s_car_x - s_ctf[t - 1].fx;
        double dz = s_car_z - s_ctf[t - 1].fz;
        double d2 = dx * dx + dz * dz;
        if (!best_t || d2 < best) { best_t = t; best = d2; }
    }
    if (d2_out)
        *d2_out = best_t ? best : 0.0;
    return best_t;
}

/* One 20 Hz step of the race (D-O3/D-O5/D-O6). Returns 1 when the mission
 * is over, so the caller can skip the shared clock.
 *
 * The gate geometry is the original's (FUN_00444bb0): a 30 m SPHERE around
 * the next gate's 3-D marker position. A course carrying check99 (D-O5)
 * does not count the lap at the last numbered gate — the finish line is
 * check99, and the lap counts there; the next target then wraps to check1. */
static int objectives_tick_race(void)
{
    int on_finish = s_has99 && s_next_check >= s_ncheck;
    double tx = on_finish ? s_finish[0] : s_course[s_next_check].x;
    double ty = on_finish ? s_finish[1] : s_course[s_next_check].y;
    double tz = on_finish ? s_finish[2] : s_course[s_next_check].z;
    double dx = s_car_x - tx;
    double dy = s_car_y - ty;
    double dz = s_car_z - tz;
    if (dx * dx + dy * dy + dz * dz > RACE_CHECK_R * RACE_CHECK_R)
        return 0;

    if (!on_finish) {
        s_next_check++;
        if (s_next_check < s_ncheck)
            return 0;
        if (s_has99)
            return 0;               /* finish line still ahead (D-O5)  */
    }

    s_next_check = 0;
    s_lap++;
    if (s_lap < s_lap_target)
        return 0;

    /* The original's own counter string (nitro.exe "%d of %d Laps
     * Completed") followed by its own win line ("You win!"). Neither is
     * invented wording; the ". " that welds them is (ledger §5). */
    char msg[MISSION_MSG_MAX];
    snprintf(msg, sizeof msg, "%d of %d Laps Completed. You win!",
             s_lap, s_lap_target);
    msg_set(msg);
    s_state = MISSION_COMPLETE;
    fprintf(stdout, "[mission] %s: RACE COMPLETE — %d lap(s) in %llu ticks\n",
            s_mission, s_lap, (unsigned long long)s_tick);
    return 1;
}

/*
 * One 20 Hz step of a capture map (D-O8..D-O13). Returns 1 when the
 * mission is over.
 *
 * Every string printed here is nitro.exe's, verbatim, including its
 * format specifiers: "*** %s's flag has been taken!",
 * "*** Team %s has captured %s's flag!" and the terminal
 * "*** %s has reached %d captures! %s" whose trailing %s is the binary's
 * own "You win!". What is ours is the RULE, not the wording.
 */
static int objectives_tick_ctf(void)
{
    const double r2 = CTF_TOUCH_R * CTF_TOUCH_R;
    char msg[MISSION_MSG_MAX];

    if (!s_ctf_carry) {
        double d2 = 0.0;
        int t = ctf_nearest_enemy_flag(&d2);
        if (!t || d2 > r2)
            return 0;
        /* D-O11: one flag at a time, so the pickup ends the search. */
        s_ctf_carry = t;
        snprintf(msg, sizeof msg, "*** %s's flag has been taken!",
                 mission_team_name(t));
        msg_set(msg);
        fprintf(stdout, "[mission] %s: CAPTURE — took team %d's flag at "
                        "tick %llu\n", s_mission, t,
                (unsigned long long)s_tick);
        return 0;
    }

    double home2 = ctf_base_dist2(CTF_PLAYER_TEAM);
    if (home2 < 0.0 || home2 > r2)
        return 0;

    int taken = s_ctf_carry;
    s_ctf_carry = 0;                    /* D-O13: the flag goes back home */
    s_captures++;

    if (s_captures >= s_capture_target) {
        snprintf(msg, sizeof msg, "*** %s has reached %d captures! %s",
                 mission_team_name(CTF_PLAYER_TEAM), s_captures, "You win!");
        msg_set(msg);
        s_state = MISSION_COMPLETE;
        fprintf(stdout, "[mission] %s: CAPTURE COMPLETE — %d of %d in "
                        "%llu ticks\n", s_mission, s_captures,
                s_capture_target, (unsigned long long)s_tick);
        return 1;
    }

    snprintf(msg, sizeof msg, "*** Team %s has captured %s's flag!",
             mission_team_name(CTF_PLAYER_TEAM), mission_team_name(taken));
    msg_set(msg);
    fprintf(stdout, "[mission] %s: CAPTURE — %d of %d at tick %llu\n",
            s_mission, s_captures, s_capture_target,
            (unsigned long long)s_tick);
    return 0;
}

/*
 * Advance the melee world one 20 Hz step: movers, bodies, combat.
 *
 * This is the FSM path's own ordering (ai_tick -> world contact -> scene
 * write-back -> combat_tick) reproduced for a mission that has no
 * machines to run. It is
 * separate from objectives_tick_melee below so the win/loss test reads
 * state that is already this tick's, exactly as the FSM bridge does.
 */
static void melee_world_tick(void)
{
    ai_tick(AI_TICK_DT, ent_pos, mission_ai_nav_query);
    ai_world_contacts_tick();       /* D-A12: constrain proposed moves */

    for (int i = 0; i < s_melee_bots; i++) {
        MissionMeleeBot *b = &s_melee[i];
        double p[3];
        if (b->scene_obj < 0 || ai_get_pos(b->ent, p) != 0)
            continue;
        if (!combat_alive(b->ent))
            continue;               /* dead bodies stay where they fell   */
        mission_ent_writeback(b->ent, b->scene_obj, p);
    }

    vehicle_contacts_tick(0);

    combat_tick();
}

/*
 * One 20 Hz step of a melee (D-O14..D-O19). Returns 1 when the mission is
 * over, so the caller can skip the shared clock.
 *
 * Every string here is nitro.exe's, verbatim, including format specifiers:
 * "*** %s has been killed", the terminal "*** %s has reached %d kills! %s"
 * whose trailing %s is the binary's own "You win!", and "You lose, chump!"
 * for a destroyed player. What is ours is the RULE, not the wording.
 */
static int objectives_tick_melee(void)
{
    char msg[MISSION_MSG_MAX];

    for (int i = 0; i < s_melee_bots; i++) {
        MissionMeleeBot *b = &s_melee[i];
        if (!b->was_alive || combat_alive(b->ent))
            continue;
        b->was_alive = 0;
        snprintf(msg, sizeof msg, "*** %s has been killed",
                 melee_driver_name(i));
        msg_set(msg);
        fprintf(stdout, "[mission] %s: MELEE — %s destroyed at tick %llu\n",
                s_mission, melee_driver_name(i),
                (unsigned long long)s_tick);
    }

    /* D-O18: dying loses the mission. Checked before the win so a player
     * who trades their last hit point for the last kill still loses —
     * combat resolves both inside one combat_tick and the loss is the
     * conservative reading. */
    if (!combat_alive(0)) {
        msg_set("You lose, chump!");            /* nitro.exe, verbatim */
        s_state = MISSION_FAILED;
        fprintf(stdout, "[mission] %s: MELEE FAILED — player destroyed at "
                        "tick %llu with %d of %d kill(s)\n",
                s_mission, (unsigned long long)s_tick, combat_kills(0),
                s_kill_target);
        return 1;
    }

    /* D-C13: only the player's OWN kills count. Nothing else can move this
     * number — the opponents are all one team and never fight each other
     * (D-O17) — so a melee cannot be won by watching. */
    int kills = combat_kills(0);
    if (kills < s_kill_target)
        return 0;

    snprintf(msg, sizeof msg, "*** %s has reached %d kills! %s",
             MELEE_PLAYER_NAME, kills, "You win!");
    msg_set(msg);
    s_state = MISSION_COMPLETE;
    fprintf(stdout, "[mission] %s: MELEE COMPLETE — %d of %d kill(s) in "
                    "%llu ticks\n", s_mission, kills, s_kill_target,
            (unsigned long long)s_tick);
    return 1;
}

/* One 20 Hz step of the arena controllers. Reached only with no FSM. */
static void objectives_tick(void)
{
    /* mission_restore() (save.c) can write a terminal state directly, and
     * a controller that overwrote it would resurrect a finished mission. */
    if (s_state != MISSION_RUNNING)
        return;
    if (!s_car_set)
        return;                         /* no pose published yet         */

    if (s_family == MISSION_FAMILY_RACE && s_ncheck > 0) {
        if (objectives_tick_race())
            return;
    } else if (s_family == MISSION_FAMILY_CAPTURE && s_ctf_teams > 0) {
        if (objectives_tick_ctf())
            return;
    } else if (s_family == MISSION_FAMILY_MELEE && s_melee_bots > 0) {
        if (objectives_tick_melee())
            return;
    } else {
        return;                         /* D-O15 / no course             */
    }

    if (s_limit_ticks > 0 && s_tick >= s_limit_ticks) {
        msg_set("*** Time limit has elapsed!");   /* nitro.exe, verbatim */
        s_state = MISSION_FAILED;
        fprintf(stdout, "[mission] %s: %s FAILED — time limit (%llu ticks)\n",
                s_mission,
                s_family == MISSION_FAMILY_RACE    ? "RACE"    :
                s_family == MISSION_FAMILY_CAPTURE ? "CAPTURE" : "MELEE",
                (unsigned long long)s_limit_ticks);
    }
}

/* ----------------------------------------------------------------------- */
/* Entity resolution (D4/D5)                                               */
/* ----------------------------------------------------------------------- */

static void resolve_entities(void)
{
    s_user_ent = -1;
    int n = fsm_image_entity_count(s_img);
    s_ents = calloc((size_t)(n > 0 ? n : 1), sizeof *s_ents);
    if (!s_ents)
        return;
    s_nents = n;

    for (int i = 0; i < n; i++) {
        MissionEnt *e = &s_ents[i];
        const char *label  = fsm_image_entity_label(s_img, i);
        const char *object = fsm_image_entity_object(s_img, i);
        char oname[9];
        int oid = 0;
        e->obj = NULL;
        e->id  = i;             /* D16: ids default to the entity index */
        e->is_user = 0;
        e->body_owner = i;

        if (object)
            label_unpack((const uint8_t *)object, oname, &oid);
        else
            oname[0] = '\0';

        /* D4: case-insensitive name match; id bits disambiguate
         * duplicate labels; first name match as fallback. */
        const MissionObj *first = NULL;
        for (int j = 0; j < s_nobj; j++) {
            if (strcasecmp(s_objs[j].label, oname) != 0)
                continue;
            if (!first)
                first = &s_objs[j];
            if (s_objs[j].label_id == oid) {
                e->obj = &s_objs[j];
                break;
            }
        }
        if (!e->obj)
            e->obj = first;

        /* D5: the player is the entity labeled "user". */
        if (label && strcasecmp(label, "user") == 0)
            e->is_user = 1;

        fprintf(stdout,
                "[mission] entity %2d '%s' -> '%s'(%d): %s\n", i,
                label ? label : "?", oname, oid,
                e->obj ? "resolved" : "UNRESOLVED (static pose 0,0,0)");
        if (e->obj)
            fprintf(stdout, "[mission]   pos=(%.1f, %.1f, %.1f) cls=%u "
                            "flags=0x%02x team=%u\n",
                    e->obj->pos[0], e->obj->pos[1], e->obj->pos[2],
                    e->obj->class_id, e->obj->flags, e->obj->team);
    }

    for (int i = 0; i < n; i++)
        if (s_ents[i].is_user) {
            s_user_ent = i;
            break;
        }

    /* D5 fallback: the flags&0x10 car is the player-flagged ODEF record. */
    if (s_user_ent < 0) {
        for (int i = 0; i < n && s_user_ent < 0; i++) {
            const MissionObj *o = s_ents[i].obj;
            if (o && o->class_id == 1 && (o->flags & 0x10)) {
                s_user_ent = i;
                s_ents[i].is_user = 1;
                fprintf(stdout, "[mission] no 'user' label; entity %d "
                                "(flags&0x10) is the player\n", i);
            }
        }
    }
    /* Multiple FSM names may resolve to one ODEF body. P15 carries three
     * copies of each hearse for cinematic/script roles; treating all copies
     * as independent AI/combat cars made the shared OBB repel itself and the
     * idle aliases autonomously attack the player. Collapse aliases onto the
     * last matching entity, preserving the pre-fix write-back winner from the
     * ascending entity loop while giving the body exactly one mover, combat
     * pool, and contact hull. Every action resolves through this owner. */
    for (int i = 0; i < n; i++) {
        if (!s_ents[i].obj)
            continue;
        for (int j = i + 1; j < n; j++)
            if (s_ents[j].obj == s_ents[i].obj)
                s_ents[i].body_owner = j;
    }
    if (s_user_ent >= 0)
        s_user_ent = mission_ent_owner(s_user_ent);
    if (s_user_ent >= 0)
        s_ents[s_user_ent].is_user = 1;
    fprintf(stdout, "[mission] player entity: %d%s\n", s_user_ent,
            s_user_ent >= 0 ? "" : " (NONE — car pose unused by script)");

    /* D14: register each physical ODEF body once with the kinematic mover and
     * cache the scene object used for render write-back. FSM aliases keep the
     * same scene-object reference but resolve to body_owner for live state. */
    ai_reset();
    for (int i = 0; i < n; i++) {
        double p[3] = { 0.0, 0.0, 0.0 };
        s_ents[i].scene_obj = -1;
        if (s_ents[i].obj) {
            p[0] = s_ents[i].obj->pos[0];
            p[1] = s_ents[i].obj->pos[1];
            p[2] = s_ents[i].obj->pos[2];
            s_ents[i].scene_obj = scene_obj_find(s_ents[i].obj->label,
                                                 s_ents[i].obj->label_id);
        }
        if (mission_ent_owner(i) != i)
            continue;
        if (!s_ents[i].is_user && mission_ent_is_vehicle(i)) {
            p[1] = vehicle_spawn_place(i, s_ents[i].scene_obj,
                                       MISSION_VEHICLE_SPAWN_ODEF,
                                       p[0], p[2], p[1]);
            if (s_ents[i].scene_obj >= 0)
                (void)scene_obj_set_pos(s_ents[i].scene_obj, p);
        }
        ai_agent_init(i, p[0], p[1], p[2]);
        if (s_ents[i].obj && s_ents[i].obj->class_id == 9)
            ai_set_flyer(i, 1);                      /* D-A23 */
        if (s_ents[i].obj && s_ents[i].obj->has_yaw)
            ai_seed_heading(i, s_ents[i].obj->yaw);  /* D-A15 seed */
        if (!s_ents[i].is_user && s_ents[i].obj &&
            s_ents[i].obj->class_id == 1 &&
            ai_shadow_attach(i, s_ents[i].obj->class_id,
                             s_ents[i].obj->label) != 0)
            fprintf(stderr, "[mission] class-1 context load failed: %s\n",
                    s_ents[i].obj->label);
        mission_ent_motion_init(i, p);
    }

    /* H-UAT-060: consume the authored player ODEF body at the same identity
     * resolution seam that instantiates its live mission entity. AI vehicles
     * keep their scene object because it IS their moving render body; the
     * physical player is rendered from car.c's dynamic VGEO queue instead.
     * Keep consumption separate from combat_hide: an FSM unhide must never
     * resurrect the static shell. This is label/role driven, so a garage VCF
     * override still consumes the mission-default body without a mission case. */
    if (s_user_ent >= 0 && s_user_ent < s_nents &&
        s_ents[s_user_ent].scene_obj >= 0)
        (void)scene_obj_consume(s_ents[s_user_ent].scene_obj);

    /* D17: register each physical ODEF body once with the combat model
     * (team/class from the ODEF snapshot) and point it at the live position
     * resolver + the canonical player entity. */
    for (int i = 0; i < n; i++) {
        if (mission_ent_owner(i) != i)
            continue;
        const MissionObj *o = s_ents[i].obj;
        CarCombatConfig cc;
        const CarCombatConfig *cp = NULL;
        if (o && (o->class_id == 1 || o->class_id == 9) &&
            car_combat_config(o->label, &cc) == 0)
            cp = &cc;
        combat_register(i,
                        o ? (int)o->team : -1,
                        o ? (int)o->class_id : 0,
                        s_ents[i].scene_obj,
                        fsm_image_entity_label(s_img, i), cp);
    }
    combat_set_user(s_user_ent);
    combat_set_resolver(ent_pos);
}

/* Entity world position. The user entity tracks the live car pose (D5). */
static void ent_pos(int ent, double out[3])
{
    out[0] = out[1] = out[2] = 0.0;
    ent = mission_ent_owner(ent);
    if (ent < 0)
        return;
    if (s_ents[ent].is_user && s_car_set) {
        out[0] = s_car_x;
        out[1] = s_car_y;
        out[2] = s_car_z;
        return;
    }
    /* D14: AI-tracked pose (every entity is registered at load; only
     * goal-driven entities ever move). The user reaches this branch only
     * when no car pose was ever published — the AI ghost is the best
     * answer then (P01's intro auto-drive moves it). */
    if (ai_get_pos(ent, out) == 0) {
        mission_ent_ground_pose(ent, out);
        return;
    }
    if (s_ents[ent].obj) {
        out[0] = s_ents[ent].obj->pos[0];
        out[1] = s_ents[ent].obj->pos[1];
        out[2] = s_ents[ent].obj->pos[2];
    }
}

/* Cache the user mover as a host-consumable car target. Speed comes from
 * this tick's displacement; heading is the mover's persistent D-A15
 * heading (seeded from the ODEF frame's real yaw at registration, so the
 * first sample already matches the old ODEF fallback). */
static void scripted_car_update(void)
{
    double p[3];
    if (s_user_ent < 0 || ai_get_pos(s_user_ent, p) != 0)
        return;

    if (s_script_have) {
        double dx = p[0] - s_script_pos[0];
        double dz = p[2] - s_script_pos[2];
        double d = sqrt(dx * dx + dz * dz);
        s_script_speed = d / AI_TICK_DT;
        if (d > 1e-9 || ai_goal(s_user_ent) != AI_GOAL_NONE)
            s_script_yaw = ai_get_heading(s_user_ent);
    } else {
        const MissionObj *o = s_ents[s_user_ent].obj;
        s_script_yaw = (o && o->has_yaw) ? o->yaw : s_car_yaw;
        s_script_speed = 0.0;
        s_script_have = 1;
    }
    if (s_user_snap_pending) {
        /* A teleport is placement, not displacement/dt. This also applies
         * to the first sample and to a zero-speed snap followed by sit. */
        s_script_speed = s_user_snap_speed;
        s_script_yaw = ai_get_heading(s_user_ent);
        s_script_snap_tick = s_tick;
        s_user_snap_pending = 0;
    }
    s_script_pos[0] = p[0];
    s_script_pos[1] = p[1];
    s_script_pos[2] = p[2];
}

/* ----------------------------------------------------------------------- */
/* Radio queue (D8)                                                        */
/* ----------------------------------------------------------------------- */

static void cb_enqueue(const char *clip, int owner, int32_t queue_flag)
{
    /* stopCB's block (binary DAT_00523110): while set, the enqueue
     * helper refuses every callback silently — the queue is NOT
     * touched, and playback of what is already queued continues. */
    if (s_cb_blocked)
        return;
    if (s_cbq_len >= MISSION_CBQ_MAX) {
        fprintf(stdout, "[mission] radio queue full, dropping '%s'\n", clip);
        return;
    }

    int slot;
    if (queue_flag == 1) {
        /* priority: insert at the FRONT (fsm.md §4.3) */
        memmove(&s_cbq[1], &s_cbq[0],
                (size_t)s_cbq_len * sizeof s_cbq[0]);
        slot = 0;
    } else {
        if (queue_flag != 3)
            fprintf(stdout, "[mission] cbPrior queueFlag=%d (not 1/3) — "
                            "appending\n", (int)queue_flag);
        slot = s_cbq_len;
    }
    snprintf(s_cbq[slot].clip, sizeof s_cbq[slot].clip, "%s", clip);
    s_cbq[slot].owner = owner;
    s_cbq_len++;
}

static void cb_stop_all(void)
{
    s_cbq_len = 0;
    s_cb_playing = 0;
    s_cb_ticks_left = 0;
    s_cb_cur.clip[0] = '\0';
    s_cb_cur.owner = -1;
}

/*
 * The radio FSM's isCBEmpty predicate observes actual speech completion in
 * the original. The page already decodes these same mono 8-bit PCM files;
 * byte count / sample rate is therefore the one authoritative duration.
 */
static int cb_play_ticks(const char *line)
{
    const char *name = strrchr(line, ':');
    name = name && name[1] == ' ' ? name + 2 : line;
    int pcm_len = sound_load(name);
    uint32_t rate = sound_rate(name);
    if (pcm_len <= 0 || rate == 0)
        return MISSION_CB_FALLBACK_TICKS;
    uint64_t scaled = (uint64_t)(uint32_t)pcm_len * MISSION_TICK_HZ;
    int ticks = (int)((scaled + rate - 1u) / rate);
    return ticks > 0 ? ticks : 1;
}

/* Advance playback one tick; start the next queued clip when idle. */
static void cb_tick(void)
{
    if (s_cb_playing) {
        if (--s_cb_ticks_left <= 0) {
            s_cb_playing = 0;
            s_cb_cur.clip[0] = '\0';
            s_cb_cur.owner = -1;
        }
    }
    if (!s_cb_playing && s_cbq_len > 0) {
        s_cb_cur = s_cbq[0];
        memmove(&s_cbq[0], &s_cbq[1],
                (size_t)(s_cbq_len - 1) * sizeof s_cbq[0]);
        s_cbq_len--;
        s_cb_playing = 1;
        s_cb_ticks_left = cb_play_ticks(s_cb_cur.clip);

        if (s_cb_cur.owner >= 0) {
            const char *label = fsm_image_entity_label(s_img, s_cb_cur.owner);
            char line[MISSION_MSG_MAX];
            snprintf(line, sizeof line, "%.39s: %.118s",
                     label ? label : "?", s_cb_cur.clip);
            msg_set(line);
        } else {
            msg_set(s_cb_cur.clip);
        }
    }
}


/* ----------------------------------------------------------------------- */
/* Objective lines (PORT GUIDANCE — see mission.h)                         */
/* ----------------------------------------------------------------------- */

static int navobj_fresh(const MissionNavObj *o)
{
    return o->used && s_tick - o->seen <= MISSION_NAVOBJ_TTL;
}

/* Record each predicate site: one anchor can have different consequences. */
static void navobj_touch(int path, int ent, const double pn[2], double r,
                         int sq, FsmNavConsequence consequence)
{
    MissionNavObj *slot = NULL, *reuse = NULL;
    int new_site = 0;
    for (int i = 0; i < MISSION_NAVOBJ_MAX; i++) {
        MissionNavObj *o = &s_navobj[i];
        if (o->used && o->machine == s_guidance_machine &&
            o->pc == fsm_machine_pc(s_guidance_machine)) {
            slot = o;
            break;
        }
        if (!reuse && !navobj_fresh(o))
            reuse = o;
    }
    if (!slot) {
        slot = reuse;
        if (!slot)
            return;             /* bounded: display slots exhausted      */
        new_site = 1;
        slot->used = 1;
        slot->machine = s_guidance_machine;
        slot->pc = fsm_machine_pc(s_guidance_machine);
        slot->born = s_tick;
    }
    if (slot->path != path || slot->ent != ent)
        slot->born = s_tick;
    slot->path = path;
    slot->ent = ent;
    slot->consequence = consequence;
    slot->x = pn[0];
    slot->z = pn[1];
    slot->r = r;
    slot->sq = sq;
    if (new_site || slot->seen != s_tick)
        slot->poll_order = s_nav_poll_order++;
    slot->seen = s_tick;
}

/* A gate the script polls as satisfied stops being a line. Returns 1
 * when the line existed and was still fresh — the "genuinely reached"
 * edge, as opposed to a first poll that was already within range. */
static int navobj_danger(double x, double z);

static int navobj_satisfy(void)
{
    for (int i = 0; i < MISSION_NAVOBJ_MAX; i++) {
        MissionNavObj *o = &s_navobj[i];
        if (o->used && o->machine == s_guidance_machine &&
            o->pc == fsm_machine_pc(s_guidance_machine)) {
            int fresh = navobj_fresh(o) && o->consequence == FSM_NAV_PROGRESS &&
                        !navobj_danger(o->x, o->z);
            o->used = 0;
            return fresh;
        }
    }
    return 0;
}

/* ----------------------------------------------------------------------- */
/* FSM host bridge — the dispatch at the heart of this module              */
/* ----------------------------------------------------------------------- */

typedef struct {
    const char *n;
    int32_t     ret;
    const char *why;
} MissionConstPred;

static const MissionConstPred *const_pred_find(const char *name);

/* Ledger bookkeeping + log-once for non-semantic actions. */
static void ledger_note(int action_index, const char *name, int cls,
                        const char *why)
{
    MissionActionStat *st;
    if (!s_astats || action_index < 0)
        return;
    st = &s_astats[action_index];
    if (st->cls == LEDGE_SEMANTIC)
        st->cls = (uint8_t)cls;
    if (!st->announced && cls != LEDGE_SEMANTIC) {
        st->announced = 1;
        fprintf(stdout, "[mission] %s action '%s' (idx %d): %s\n",
                cls == LEDGE_TODO ? "TODO" : "const", name, action_index,
                why);
    }
}

static int32_t mission_time_pred(int32_t slot, int32_t secs, int greater,
                                 const char *name);

/* Path node-0 XZ position (D6: the binary's isWithinNav/isWithinSqNav
 * anchor — path record +0x54 node array, first node only). */
static int path_node0(int path_index, double out[2])
{
    const float *pts = NULL;
    int n = fsm_image_path(s_img, path_index, &pts);
    if (n <= 0 || !pts)
        return -1;
    out[0] = pts[0];
    out[1] = pts[2];
    return 0;
}

/* Validated entity-index argument: derefs the cell, range-checks against
 * the entity table, and collapses multiple FSM aliases of one ODEF body to
 * its single live owner. Returns the owner or -1 (bad indices are logged). */
static int arg_ent(int32_t **args, int i, int nargs, const char *name)
{
    if (i >= nargs)
        return -1;
    int32_t v = *args[i];
    if (v < 0 || v >= s_nents) {
        fprintf(stdout, "[mission] %s: entity index %d out of range "
                        "(%d entities)\n", name, (int)v, s_nents);
        return -1;
    }
    return mission_ent_owner((int)v);
}

/* Snapshot-only conjunctions: before lookahead effects, other entities
 * keep their current pose and the user is at the proposed destination. */
static int navobj_query(void *ud, const char *name, int32_t **args, int nargs)
{
    const double *arrival = ud;
    if ((!strcmp(name, "isWithinNav") || !strcmp(name, "isWithinSqNav")) &&
        nargs == 3 && *args[1] >= 0 && *args[1] < s_nents) {
        double pn[2], pe[3];
        if (path_node0(*args[0], pn) != 0) return -1;
        int ent = mission_ent_owner(*args[1]);
        ent_pos(ent, pe);
        if (ent == s_user_ent) { pe[0] = arrival[0]; pe[2] = arrival[1]; }
        double dx = pe[0] - pn[0], dz = pe[2] - pn[1], r = *args[2];
        return name[8] == 'S' ? (fabs(dx) < r && fabs(dz) < r)
                             : (dx * dx + dz * dz < r * r);
    }
    return -1;
}

/* --- cutscene camera (D18) --------------------------------------------- */

static double cam_angle_rad(double centidegrees)
{
    int raw = (int)centidegrees;
    return (double)(raw % 36000) * (MISSION_PI / 18000.0);
}

static void cam_clamp_eye(double eye[3])
{
    /*
     * Every cutscene camera is a world-space eye, regardless of which FSM
     * placement verb produced it. Clamp once at this ownership boundary so
     * no camera mode can render live terrain as a ceiling.
     */
    double ground = terrain_height_at(eye[0], eye[2]) + 1.0;
    if (eye[1] < ground)
        eye[1] = ground;
}

static void cam_set_pose(double ex, double ey, double ez,
                         double tx, double ty, double tz)
{
    double eye[3] = { ex, ey, ez };
    double target[3] = { tx, ty, tz };
    CameraView view;
    cam_clamp_eye(eye);
    if (camera_view_look_at(&view, eye, target) != 0)
        return;
    s_cam.cur = view;
    s_cam.active = 1;
}

static void cam_set_direction(double ex, double ey, double ez,
                              double first_c, double second_c, double third_c)
{
    double eye[3] = { ex, ey, ez };
    cam_clamp_eye(eye);
    camera_view_native_angles(&s_cam.cur, eye,
                              cam_angle_rad(first_c),
                              cam_angle_rad(second_c),
                              cam_angle_rad(third_c));
    s_cam.active = 1;
}

static void cam_stop_path(void)
{
    s_cam.path_i = -1;
    s_cam.path_n = 0;
    s_cam.path_pts = NULL;
    s_cam.path_s = 0.0;
}

static void cam_sample_path(double s, double out[3], double tangent[3])
{
    /* Polyline walk: path points are packed x,y,z floats per node. */
    if (!s_cam.path_pts || s_cam.path_n <= 0) {
        out[0] = out[1] = out[2] = 0.0;
        if (tangent) { tangent[0] = 0; tangent[1] = 0; tangent[2] = 1; }
        return;
    }
    if (s_cam.path_n == 1) {
        out[0] = s_cam.path_pts[0];
        out[1] = s_cam.path_pts[1];
        out[2] = s_cam.path_pts[2];
        if (tangent) { tangent[0] = 0; tangent[1] = 0; tangent[2] = 1; }
        return;
    }
    double remain = s;
    for (int i = 0; i < s_cam.path_n - 1; i++) {
        double ax = s_cam.path_pts[i * 3 + 0];
        double ay = s_cam.path_pts[i * 3 + 1];
        double az = s_cam.path_pts[i * 3 + 2];
        double bx = s_cam.path_pts[(i + 1) * 3 + 0];
        double by = s_cam.path_pts[(i + 1) * 3 + 1];
        double bz = s_cam.path_pts[(i + 1) * 3 + 2];
        double dx = bx - ax, dy = by - ay, dz = bz - az;
        double len = sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-9)
            continue;
        if (remain <= len || i == s_cam.path_n - 2) {
            double t = remain / len;
            if (t > 1.0) t = 1.0;
            if (t < 0.0) t = 0.0;
            out[0] = ax + dx * t;
            out[1] = ay + dy * t;
            out[2] = az + dz * t;
            if (tangent) {
                tangent[0] = dx / len;
                tangent[1] = dy / len;
                tangent[2] = dz / len;
            }
            return;
        }
        remain -= len;
    }
}

static double cam_path_length(void)
{
    if (!s_cam.path_pts || s_cam.path_n < 2)
        return 0.0;
    double total = 0.0;
    for (int i = 0; i < s_cam.path_n - 1; i++) {
        double dx = s_cam.path_pts[(i + 1) * 3 + 0] - s_cam.path_pts[i * 3 + 0];
        double dy = s_cam.path_pts[(i + 1) * 3 + 1] - s_cam.path_pts[i * 3 + 1];
        double dz = s_cam.path_pts[(i + 1) * 3 + 2] - s_cam.path_pts[i * 3 + 2];
        total += sqrt(dx * dx + dy * dy + dz * dz);
    }
    return total;
}

static void cam_start_path(int path_i, double height_centi, double speed_centi,
                           int watch_ent, int fixed_direction,
                           double first_c, double second_c, double third_c)
{
    const float *pts = NULL;
    int n = fsm_image_path(s_img, path_i, &pts);
    if (n <= 0 || !pts) {
        fprintf(stdout, "[mission] camTrans: path %d missing\n", path_i);
        cam_stop_path();
        s_cam.arrived_pulse = 1;
        return;
    }
    /*
     * FSMs issue camTrans* again on every polling pass. nitro.exe only
     * initializes FUN_004a09f0/FUN_004a0d70 when the path owner changes;
     * restarting here would pin every travelling shot at node zero.
     */
    if (s_cam.path_i != path_i || s_cam.path_pts != pts) {
        s_cam.path_i = path_i;
        s_cam.path_n = n;
        s_cam.path_pts = pts;
        s_cam.path_s = 0.0;
        s_cam.arrived_pulse = 0;
    }
    s_cam.path_speed = speed_centi > 0.0 ? speed_centi * 0.01 : 0.0;
    s_cam.path_height = height_centi * 0.01;
    s_cam.path_watch = watch_ent;
    s_cam.path_fixed_direction = fixed_direction;
    s_cam.path_angles[0] = first_c;
    s_cam.path_angles[1] = second_c;
    s_cam.path_angles[2] = third_c;
    s_cam.active = 1;

    /* Place at the current path position before this tick's advance. */
    double p[3], tangent[3];
    cam_sample_path(s_cam.path_s, p, tangent);
    double th = terrain_height_at(p[0], p[2]);
    double ey = th + s_cam.path_height;
    if (ey < th + 0.5)
        ey = th + 0.5;
    if (watch_ent >= 0) {
        double watch[3];
        ent_pos(watch_ent, watch);
        cam_set_pose(p[0], ey, p[2], watch[0], watch[1], watch[2]);
    } else if (fixed_direction) {
        cam_set_direction(p[0], ey, p[2], first_c, second_c, third_c);
    } else {
        cam_set_pose(p[0], ey, p[2],
                     p[0] + tangent[0], ey + tangent[1], p[2] + tangent[2]);
    }
}

static void cam_tick(void)
{
    if (s_cam.path_i < 0 || !s_cam.path_pts)
        return;
    s_cam.path_s += s_cam.path_speed * 0.05;   /* 20 Hz */
    double plen = cam_path_length();
    int done = (plen <= 1e-6) || (s_cam.path_s >= plen);
    if (done)
        s_cam.path_s = plen;
    double p[3], tangent[3];
    cam_sample_path(s_cam.path_s, p, tangent);
    double th = terrain_height_at(p[0], p[2]);
    double ey = th + s_cam.path_height;
    if (ey < th + 0.5)
        ey = th + 0.5;
    if (s_cam.path_watch >= 0) {
        double watch[3];
        ent_pos(s_cam.path_watch, watch);
        cam_set_pose(p[0], ey, p[2], watch[0], watch[1], watch[2]);
    } else if (s_cam.path_fixed_direction) {
        cam_set_direction(p[0], ey, p[2],
                          s_cam.path_angles[0], s_cam.path_angles[1],
                          s_cam.path_angles[2]);
    } else {
        cam_set_pose(p[0], ey, p[2],
                     p[0] + tangent[0], ey + tangent[1], p[2] + tangent[2]);
    }
    if (done) {
        cam_stop_path();
        s_cam.arrived_pulse = 1;
    }
}

static void cam_place_obj_dir(int ent, double dx, double dy, double dz,
                              double first_c, double second_c, double third_c)
{
    double pe[3], zero[3] = { 0.0, 0.0, 0.0 };
    CameraView basis;
    ent_pos(ent, pe);
    camera_view_native_angles(&basis, zero,
                              cam_angle_rad(first_c),
                              cam_angle_rad(second_c),
                              cam_angle_rad(third_c));

    /* The binary's affine applies the same orientation to the local offset
     * and to the camera basis; do not independently reinterpret the angles. */
    double ox = dx * 0.01, oy = dy * 0.01, oz = dz * 0.01;
    double ex = pe[0] + ox * basis.right[0] + oy * basis.up[0] +
                        oz * basis.forward[0];
    double ey = pe[1] + ox * basis.right[1] + oy * basis.up[1] +
                        oz * basis.forward[1];
    double ez = pe[2] + ox * basis.right[2] + oy * basis.up[2] +
                        oz * basis.forward[2];
    if (ey < pe[1] + 1.0)
        ey = pe[1] + 1.0;
    cam_set_direction(ex, ey, ez, first_c, second_c, third_c);
    cam_stop_path();
}

static void cam_place_obj_obj(int anchor, double x_c, double y_c, double z_c,
                              int watch)
{
    double pa[3], pw[3];
    ent_pos(anchor, pa);
    ent_pos(watch, pw);
    double ex = pa[0] + x_c * 0.01;
    double ey = pa[1] + y_c * 0.01;
    double ez = pa[2] + z_c * 0.01;
    double th = terrain_height_at(ex, ez);
    if (ey < th + 0.5)
        ey = th + 0.5;
    cam_set_pose(ex, ey, ez, pw[0], pw[1], pw[2]);
    cam_stop_path();
}

static void cam_place_pos_obj(int path_i, double height_c, int watch)
{
    double pn[2];
    if (path_node0(path_i, pn) != 0)
        return;
    double th = terrain_height_at(pn[0], pn[1]);
    double ey = th + height_c * 0.01;
    if (ey < th + 0.5)
        ey = th + 0.5;
    double pw[3];
    ent_pos(watch, pw);
    cam_set_pose(pn[0], ey, pn[1], pw[0], pw[1], pw[2]);
    cam_stop_path();
}

static void cam_place_pos_dir(int path_i, double height_c,
                              double first_c, double second_c, double third_c)
{
    double pn[2];
    if (path_node0(path_i, pn) != 0)
        return;
    double th = terrain_height_at(pn[0], pn[1]);
    double ey = th + height_c * 0.01;
    if (ey < th + 0.5)
        ey = th + 0.5;
    cam_set_direction(pn[0], ey, pn[1], first_c, second_c, third_c);
    cam_stop_path();
}

static int32_t mission_dispatch(void *ud, int action_index,
                                const char *name, int32_t **args, int nargs)
{
    MissionActionStat *st = NULL;
    (void)ud;
    if (s_astats && action_index >= 0) {
        st = &s_astats[action_index];
        st->calls++;
    }

    /* --- pure cell verbs (fully semantic) ----------------------------- */
    if (!strcmp(name, "null"))  return 0;
    if (!strcmp(name, "true"))  return 1;
    if (!strcmp(name, "false")) return 0;
    if (!strcmp(name, "set") && nargs == 2)  { *args[0] = *args[1]; return 0; }
    if (!strcmp(name, "inc") && nargs == 1)  { ++*args[0]; return 0; }
    if (!strcmp(name, "dec") && nargs == 1)  { --*args[0]; return 0; }
    if (!strcmp(name, "rand") && nargs == 2) {           /* D10 */
        int32_t m = *args[1];
        *args[0] = m > 0 ? (int32_t)(rng_next() % (uint32_t)m) : 0;
        return 0;
    }
    if (!strcmp(name, "isEqual") && nargs == 2)   return *args[0] == *args[1];
    if (!strcmp(name, "isGreater") && nargs == 2) return *args[0] >  *args[1];
    if (!strcmp(name, "isLesser") && nargs == 2)  return *args[0] <  *args[1];

    /* --- timers (D7) --------------------------------------------------- */
    if (!strcmp(name, "startTimer") && nargs == 1) {
        /* Nitro case 0xd writes ftol(current_time) through the supplied
         * reference. Positive game time truncates to a whole second. */
        uint64_t seconds = s_tick / 20u;
        *args[0] = seconds > INT32_MAX ? INT32_MAX : (int32_t)seconds;
        return 0;
    }
    if (!strcmp(name, "timeGreater") && nargs == 2)
        return mission_time_pred(*args[0], *args[1], 1, name);
    if (!strcmp(name, "timeLesser") && nargs == 2)
        return mission_time_pred(*args[0], *args[1], 0, name);

    /* --- radio (D8/D21) ------------------------------------------------ */
    if ((!strcmp(name, "cbPrior") && nargs == 2) ||
        (!strcmp(name, "cbFromPrior") && nargs == 3)) {
        int is_from = (name[2] == 'F');
        int32_t clip_i = *args[0];
        const char *clip = fsm_image_clip_name(s_img, clip_i);
        int32_t flag = *args[is_from ? 2 : 1];
        if (!clip) {
            fprintf(stdout, "[mission] %s: clip index %d out of range\n",
                    name, (int)clip_i);
            return 0;
        }
        int owner = is_from ? arg_ent(args, 1, nargs, name) : -1;
        cb_enqueue(clip, owner, flag);
        return 0;
    }
    if ((!strcmp(name, "cb") && nargs == 1) ||
        (!strcmp(name, "cbFrom") && nargs == 2)) {
        int32_t clip_i = *args[0];
        const char *clip = fsm_image_clip_name(s_img, clip_i);
        if (!clip) {
            fprintf(stdout, "[mission] %s: clip index %d out of range\n",
                    name, (int)clip_i);
            return 0;
        }
        int owner = !strcmp(name, "cbFrom")
                  ? arg_ent(args, 1, nargs, name) : -1;
        /* Nitro FUN_00418630: cbFrom is the entity-owned fixed-mode-3
         * sibling of cbFromPrior, not an alias of ownerless cb. */
        cb_enqueue(clip, owner, 3);
        return 0;
    }
    if (!strcmp(name, "stopCB")) {
        /* FUN_00418690 -> FUN_00419170("STOPCBXX",0,3): set the
         * callback-stop flag and return. The queue is left allocated
         * and playing; every later enqueue is refused until the
         * mission lifecycle re-arms (FUN_00419150 — see reset). */
        s_cb_blocked = 1;
        return 0;
    }
    if (!strcmp(name, "killCB")) {
        /* FUN_00419560(0): teardown — destroy every queued callback
         * node and clear the head. The stop flag is NOT touched. */
        cb_stop_all();
        return 0;
    }
    if (!strcmp(name, "isCBEmpty"))
        return !s_cb_playing && s_cbq_len == 0;

    /* --- mission state (D9) -------------------------------------------- */
    if (!strcmp(name, "failAllObj") || !strcmp(name, "failAll")) {
        char line[MISSION_MSG_MAX];
        snprintf(line, sizeof line, "MISSION FAILED (%s reason=%d/%d)",
                 name, nargs > 0 ? (int)*args[0] : -1,
                 nargs > 1 ? (int)*args[1] : -1);
        msg_set(line);
        /* NPT failure entries use the positive, 1-based SECOND
         * failAllObj operand. Keep it structured; the first operand remains
         * uninterpreted and failAll has no established NPT contract. */
        s_fail_text_index = !strcmp(name, "failAllObj") && nargs > 1 &&
                            *args[1] > 0 ? (int)*args[1] : 0;
        s_state = MISSION_FAILED;
        return 0;
    }
    if (!strcmp(name, "successAll")) {
        msg_set("MISSION COMPLETE (successAll)");
        s_fail_text_index = 0;
        s_state = MISSION_COMPLETE;
        return 0;
    }
    if (!strcmp(name, "success") || !strcmp(name, "fail") ||
        !strcmp(name, "reveal")) {
        int id = nargs > 0 ? (int)*args[0] : -1;
        if (id > 0 && id <= s_note_count) {
            unsigned *flags = &s_notes[id - 1].flags;
            /* FUN_00459330/3a0/440: reveal preserves status; first terminal
             * status wins, and succeeding a hidden line does not reveal it. */
            if (!strcmp(name, "reveal")) *flags &= ~1u;
            else if (!(*flags & 6)) *flags |= !strcmp(name, "success") ? 2u : 4u;
        }
        return 0;
    }

    /* --- entity ids ------------------------------------------------------ */
    if (!strcmp(name, "setId") && nargs == 2) {
        /* D16: READ direction — *args[0] receives the id of entity
         * *args[1] (ids default to the entity index; see the header). */
        int e = arg_ent(args, 1, nargs, name);
        *args[0] = e >= 0 ? s_ents[e].id : -1;
        return 0;
    }
    if (!strcmp(name, "isEqualId") && nargs == 2) {
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 && s_ents[e].id == *args[1];
    }

    /* --- distances (D6) -------------------------------------------------- */
    if (!strcmp(name, "isWithin") && nargs == 3) {
        int a = arg_ent(args, 0, nargs, name);
        int b = arg_ent(args, 1, nargs, name);
        if (a < 0 || b < 0)
            return 0;
        double pa[3], pb[3];
        ent_pos(a, pa);
        ent_pos(b, pb);
        double dx = pa[0] - pb[0], dz = pa[2] - pb[2];
        double d = (double)*args[2];
        return dx * dx + dz * dz <= d * d;
    }
    if ((!strcmp(name, "isWithinNav") && nargs == 3) ||
        (!strcmp(name, "isWithinSqNav") && nargs == 3)) {
        /* D6 (BINARY-VERIFIED, FUN_00417f50/FUN_00417fb0): the anchor is
         * the path's node 0, not the polyline. isWithinNav: dx^2+dz^2
         * < r^2; isWithinSqNav: |dx| < r && |dz| < r (square, not
         * squared distance). Both strict. */
        int sq = (name[8] == 'S');
        int32_t path_i = *args[0];
        int e = arg_ent(args, 1, nargs, name);
        if (e < 0)
            return 0;
        double pe[3], pn[2];
        ent_pos(e, pe);
        if (path_node0((int)path_i, pn) != 0) {
            fprintf(stdout, "[mission] %s: path index %d out of range\n",
                    name, (int)path_i);
            return 0;
        }
        double dx = pe[0] - pn[0], dz = pe[2] - pn[1];
        double r = (double)*args[2];
        int within = sq ? (fabs(dx) < r && fabs(dz) < r)
                        : (dx * dx + dz * dz < r * r);
        /* PORT GUIDANCE: only arrival branches known to advance the
         * script are destinations. Warning/failure and unknown branches
         * remain available to diagnostics but never feed the HUD. */
        FsmNavConsequence consequence = e == s_user_ent
            ? fsm_machine_nav_consequence(s_guidance_machine, navobj_query, pn)
            : FSM_NAV_PROGRESS;
        if (!within)
            navobj_touch((int)path_i, e, pn, r, sq, consequence);
        else if (navobj_satisfy() && e == s_user_ent &&
                 consequence == FSM_NAV_PROGRESS && s_state == MISSION_RUNNING) {
            s_navobj_reached_tick = s_tick;
            s_navobj_reached_have = 1;
        }
        return within;
    }

    /* --- AI director parameters (D20; binary RE Phase A Q20) ----------- */
    if (!strcmp(name, "setSkill") && nargs == 3) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0 && ai_set_skill(e, (int)*args[1], (int)*args[2]) != 0)
            fprintf(stdout, "[mission] setSkill: indices %d/%d out of range\n",
                    (int)*args[1], (int)*args[2]);
        return 0;
    }
    if (!strcmp(name, "setAgg") && nargs == 2) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0 && ai_set_agg(e, (int)*args[1]) != 0)
            fprintf(stdout, "[mission] setAgg: value %d out of range\n",
                    (int)*args[1]);
        return 0;
    }
    if (!strcmp(name, "setAvoid") && nargs == 2) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            ai_set_avoid(e, (int)*args[1]);
        return 0;
    }
    if (!strcmp(name, "toggleAvoid") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            ai_toggle_avoid(e);
        return 0;
    }
    if (!strcmp(name, "setMaxAttackers") && nargs == 2) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            ai_set_max_attackers(e, (int)*args[1]);
        return 0;
    }

    /* --- AI story motion (D14/D15; ai.c — the M7 kinematic mover) ------ */
    if (!strcmp(name, "goto") && nargs == 3) {
        /* (ent, path, speed) — speed in m/s (ai.h D-A1) */
        int e = arg_ent(args, 0, nargs, name);
        const float *pts = NULL;
        int n = fsm_image_path(s_img, (int)*args[1], &pts);
        if (e >= 0) {
            if (n > 0)
                ai_goto(e, (int)*args[1], pts, n, (double)*args[2]);
            else
                fprintf(stdout, "[mission] goto: path index %d out of "
                                "range\n", (int)*args[1]);
        }
        return 0;
    }
    if (!strcmp(name, "follow") && nargs == 6) {
        /* (ent, target, unk, unk, xOff, speed) — fsm.md §4.2; xOff is
         * centi-units like the camera offsets (ai.h D-A7). */
        int e = arg_ent(args, 0, nargs, name);
        int t = arg_ent(args, 1, nargs, name);
        if (e >= 0 && t >= 0)
            ai_follow(e, t, (double)*args[4] * 0.01, (double)*args[5]);
        return 0;
    }
    if (!strcmp(name, "race") && nargs == 4) {
        /* (ent, path, speed, rival) — kinematically a goto (ai.h D-A8) */
        int e = arg_ent(args, 0, nargs, name);
        const float *pts = NULL;
        int n = fsm_image_path(s_img, (int)*args[1], &pts);
        if (e >= 0) {
            if (n > 0)
                ai_race(e, (int)*args[1], pts, n, (double)*args[2],
                        (int)*args[3]);
            else
                fprintf(stdout, "[mission] race: path index %d out of "
                                "range\n", (int)*args[1]);
        }
        return 0;
    }
    if (!strcmp(name, "evade") && nargs == 4) {
        /* (ent, path, speed, threat) — the original is route-follow plus
         * combat-recovery transitions (phase-a-ai-driver.md §7.2, PARTIAL);
         * the port keeps the route-follow half, which is the half P02's
         * post-race flee is authored on: both flee paths end inside the
         * exit2 escape corridor the referee watches, and the loops
         * re-issue evade every tick, which ai_goto's D-A5 idempotence
         * absorbs as speed-only updates. The threat arg is the flee
         * reference; the kinematic mover does not read it. */
        int e = arg_ent(args, 0, nargs, name);
        const float *pts = NULL;
        int n = fsm_image_path(s_img, (int)*args[1], &pts);
        if (e >= 0) {
            if (n > 0)
                ai_goto(e, (int)*args[1], pts, n, (double)*args[2]);
            else
                fprintf(stdout, "[mission] evade: path index %d out of "
                                "range\n", (int)*args[1]);
        }
        return 0;
    }
    if ((!strcmp(name, "teleport") && nargs == 4) ||
        (!strcmp(name, "teleportOffset") && nargs == 6)) {
        /* PORT DECISION: retain the legacy height and route continuation
         * pending TP-ground-origin (teleport-order.md). Operand four also
         * supplies the decoded native player heading. NPC heading/grounding
         * remains on the legacy contract; changing it also changes combat.
         * dx/dz are the native teleportOffset deltas scaled by 0.01. */
        int e = arg_ent(args, 0, nargs, name);
        const float *pts = NULL;
        int n = fsm_image_path(s_img, (int)*args[1], &pts);
        if (e >= 0) {
            if (n > 0) {
                double y = terrain_height_at((double)pts[0],
                                             (double)pts[2])
                         + (double)*args[3] * 0.01;
                double dx = nargs == 6 ? (double)*args[4] * 0.01 : 0.0;
                double dz = nargs == 6 ? (double)*args[5] * 0.01 : 0.0;
                if (!s_ents[e].is_user && mission_ent_is_vehicle(e))
                    y = vehicle_spawn_place(
                            e, s_ents[e].scene_obj,
                            MISSION_VEHICLE_SPAWN_FSM_TELEPORT,
                            (double)pts[0] + dx, (double)pts[2] + dz, y);
                if (s_ents[e].is_user) {
                    int angle = *args[3];
                    if (angle < 0 || angle > 360)
                        fprintf(stdout, "[mission] teleport angle out of bounds: %d\n",
                                angle);
                    if (angle > 179) angle -= 360;
                    /* FUN_00406120 / DAT_004c1680: float degree conversion;
                     * native forward +sin maps to the car's negative yaw. */
                    ai_seed_heading(e, -(double)((float)angle * 0.017445918172597885f));
                }
                ai_teleport(e, (int)*args[1], pts, n, (double)*args[2],
                            y, dx, dz);
                if (s_ents[e].is_user) {
                    /* Native teleport writes the same transform later FSM
                     * predicates read (FUN_00406120 / FUN_00417fb0). Publish
                     * placement now; the host still consumes the snap below. */
                    mission_set_car((double)pts[0] + dx,
                                    (double)pts[2] + dz, ai_get_heading(e));
                    s_car_vx = s_car_vz = 0.0;
                    s_user_snap_pending = 1;
                    s_user_snap_speed = *args[2] > 0 ?
                                        (double)*args[2] : 0.0;
                }
            } else {
                fprintf(stdout, "[mission] %s: path index %d out of "
                                "range\n", name, (int)*args[1]);
            }
        }
        return 0;
    }
    if (!strcmp(name, "sit") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            ai_sit(e);
        return 0;
    }
    if (!strcmp(name, "isArrived") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 ? ai_is_arrived(e) : 0;
    }
    if (!strcmp(name, "isAtFollow") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 ? ai_at_follow(e) : 0;
    }

    /* --- combat (D17; combat.c — the damage model) --------------------- */
    if (!strcmp(name, "isDead") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 ? combat_is_dead(e) : 0;
    }
    if (!strcmp(name, "isAttacked") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 ? combat_is_attacked(e) : 0;
    }
    if (!strcmp(name, "isShot") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 ? combat_is_shot(e) : 0;
    }
    if (!strcmp(name, "isRammed") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 ? combat_is_rammed(e) : 0;
    }
    if (!strcmp(name, "isGroovesFault") && nargs == 1) {
        /* attacked pulse whose attacker is the user (D17) */
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 ? combat_is_grooves_fault(e) : 0;
    }
    if (!strcmp(name, "hpLesser") && nargs == 3) {
        /* (ent, 0, pct) — the middle arg is always 0 in P01 (semantics
         * UNKNOWN, probably a facet selector; combat has one pool, D-C2) */
        int e = arg_ent(args, 0, nargs, name);
        return e >= 0 ? combat_hp_lesser(e, (int)*args[2]) : 0;
    }
    if (!strcmp(name, "ammoLesser") && nargs >= 2) {
        /* same shape as hpLesser assumed (never called in P01 —
         * DECISION): (ent, 0, pct); 2-arg form takes pct directly */
        int e = arg_ent(args, 0, nargs, name);
        int32_t pct = nargs == 3 ? *args[2] : *args[1];
        return e >= 0 ? combat_ammo_lesser(e, (int)pct) : 0;
    }
    if (!strcmp(name, "allEnemyDead"))
        return combat_all_enemy_dead();
    if (!strcmp(name, "allBlgDead"))
        return combat_all_blg_dead();
    if ((!strcmp(name, "whoAttacked") || !strcmp(name, "whoShot") ||
         !strcmp(name, "whoRammed")) && nargs == 2) {
        /* (ent, outCell) — out cell receives the attacker entity index,
         * -1 "no one" outside the pulse window (D11 + combat.c D-C11) */
        int e = arg_ent(args, 0, nargs, name);
        int32_t who = -1;
        if (e >= 0) {
            if (name[3] == 'A')      who = combat_who_attacked(e);
            else if (name[3] == 'S') who = combat_who_shot(e);
            else                     who = combat_who_rammed(e);
        }
        *args[1] = who;
        return 0;
    }
    if (!strcmp(name, "canSee") && nargs == 2) {
        return combat_can_see(*args[0], *args[1]);
    }

    if (!strcmp(name, "nearestEnemy") && nargs == 2) {
        int e = arg_ent(args, 0, nargs, name);
        *args[1] = e >= 0 ? combat_nearest_enemy(e) : -1;
        return 0;
    }
    if (!strcmp(name, "attack") && nargs >= 2) {
        /* (ent, target, unk) — chase-and-fire engagement (combat.c
         * D-C5); the third arg (always 1 in P01) is UNKNOWN, ignored. */
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            combat_attack(e, (int)*args[1]);
        return 0;
    }
    if (!strcmp(name, "hide") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            combat_hide(e, 1);
        return 0;
    }
    if (!strcmp(name, "startCar") && nargs == 1) {
        /* the ambush verb: field the car (unhide; combat.c D-C8) */
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            combat_hide(e, 0);
        return 0;
    }
    if (!strcmp(name, "destroy") && nargs == 1) {
        /* CONFIRMED action id 50 (phase-a-ai-driver.md §5): kill the live
         * vehicle/structure; vehicles then hold full brake. combat_die owns
         * both terminal damage state and this port's ai_sit brake analogue. */
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            combat_destroy(e);
        return 0;
    }
    if (!strcmp(name, "triggerGate") && nargs == 1) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0 && s_ents[e].scene_obj >= 0 &&
            scene_obj_trigger_gate(s_ents[e].scene_obj) == 0)
            world_colliders_build();
        return 0;
    }

    /* --- constant world predicates (systems that still do not exist;
     *     these are TRUE statements about a world without them) --- */
    {   const MissionConstPred *cp = const_pred_find(name);
        if (cp) {
            ledger_note(action_index, name, LEDGE_CONST, cp->why);
            return cp->ret;
        }
    }

    /* --- camera/cutscene verbs (D18) ------------------------------------ */
    if (!strcmp(name, "pushCam")) {
        if (s_cam.depth < MISSION_CAM_STACK)
            s_cam.stack[s_cam.depth++] = s_cam.cur;
        s_cam.active = 1;
        return 0;
    }
    if (!strcmp(name, "popCam")) {
        /*
         * The active transition belongs to the camera being popped, not to
         * the saved pose underneath it. Stop it before restoring the stack;
         * otherwise an early isKeypress pop leaves path_i live and the host
         * never gets control back.
         */
        cam_stop_path();
        if (s_cam.depth > 0)
            s_cam.cur = s_cam.stack[--s_cam.depth];
        s_cam.active = s_cam.depth > 0;
        return 0;
    }
    if (!strcmp(name, "camObjDir") && nargs >= 7) {
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0)
            cam_place_obj_dir(e, (double)*args[1], (double)*args[2],
                              (double)*args[3], (double)*args[4],
                              (double)*args[5], (double)*args[6]);
        return 0;
    }
    if (!strcmp(name, "camObjObj") && nargs >= 5) {
        int a = arg_ent(args, 0, nargs, name);
        int w = arg_ent(args, 4, nargs, name);
        if (a >= 0 && w >= 0)
            cam_place_obj_obj(a, (double)*args[1], (double)*args[2],
                              (double)*args[3], w);
        return 0;
    }
    if (!strcmp(name, "camPosObj") && nargs >= 3) {
        int w = arg_ent(args, 2, nargs, name);
        if (w >= 0)
            cam_place_pos_obj((int)*args[0], (double)*args[1], w);
        return 0;
    }
    if (!strcmp(name, "camPosDir") && nargs >= 5) {
        cam_place_pos_dir((int)*args[0], (double)*args[1],
                          (double)*args[2], (double)*args[3],
                          (double)*args[4]);
        return 0;
    }
    /* Native path-camera ABI starts path, height, speed for both variants. */
    if (!strcmp(name, "camTransObj") && nargs >= 4) {
        int watch = arg_ent(args, 3, nargs, name);
        if (watch >= 0)
            cam_start_path((int)*args[0], (double)*args[1],
                           (double)*args[2], watch, 0, 0.0, 0.0, 0.0);
        return 0;
    }
    if (!strcmp(name, "camTransDir") && nargs >= 6) {
        /* Direction slots are consumed literally by FUN_0049B170. */
        cam_start_path((int)*args[0], (double)*args[1],
                       (double)*args[2], -1, 1,
                       (double)*args[3], (double)*args[4],
                       (double)*args[5]);
        return 0;
    }
    if (!strcmp(name, "camIsArrived")) {
        /* Self-clearing pulse when a path cam finishes (Open76). Instant
         * place verbs leave the pulse clear so scripts that poll after a
         * camObj* do not spuriously advance; scripts that only wait on a
         * travelling cam get the real edge. When no path is running and
         * no pulse is pending, return 1 so intro machines that issued a
         * place (not a trans) can leave the gate (same practical effect
         * as the old D8 constant-1, but path cams are honest). */
        if (s_cam.arrived_pulse) {
            s_cam.arrived_pulse = 0;
            return 1;
        }
        if (s_cam.path_i >= 0)
            return 0;
        return 1;
    }
    if (!strcmp(name, "playMovie")) {
        const char *clip = nargs >= 1
                         ? fsm_image_clip_name(s_img, (int)*args[0]) : NULL;
        if (!clip) {
            fprintf(stdout, "[mission] playMovie: clip index %d out of range\n",
                    nargs >= 1 ? (int)*args[0] : -1);
            return 0;
        }
        if (!s_movie_pending) {
            snprintf(s_movie, sizeof s_movie, "%s", clip);
            s_movie_prev_cam_active = s_cam.active;
            s_movie_pending = 1;
            /* The browser owns decode and acknowledgement. Keep the
             * cutscene camera/input window active until it responds. */
            s_cam.active = 1;
        }
        return 0;
    }
    if (!strcmp(name, "isKeypress")) {
        ledger_note(action_index, name, LEDGE_SEMANTIC, NULL);
        return s_skip_pressed;
    }
    if (!strcmp(name, "setHeliHeight") && nargs >= 2) {
        /* Native FUN_004182b0: class-9 only, then set height. The FSM
         * argument is metres AGL (P19 counts 20..0 during landing). */
        int e = arg_ent(args, 0, nargs, name);
        if (e >= 0 && e < s_nents && s_ents[e].obj &&
            s_ents[e].obj->class_id == 9) {
            double pos[3];
            double h = (double)*args[1];
            if (h < 0.0)
                h = 0.0;
            if (ai_get_pos(e, pos) == 0) {
                double y = terrain_height_at(pos[0], pos[2]) + h;
                ai_set_height(e, y);
                s_ents[e].ai_y = y;
                s_ents[e].ground_clearance = h;
                s_ents[e].have_ground = 1;
            }
        }
        return 0;
    }

    /* --- everything else: logged-todo no-op ------------------------------ */
    ledger_note(action_index, name, LEDGE_TODO,
                "no semantic yet — no-op return 0 (M7 ledger)");
    return 0;
}

/* timeGreater/timeLesser shared body (D7). `started` is the timestamp value
 * written through the bytecode's own IntRef by startTimer, not a slot id. */
static int32_t mission_time_pred(int32_t started, int32_t secs, int greater,
                                 const char *name)
{
    (void)name;
    /* Nitro compares (started + secs) against current_time - (-0.5f).
     * In 20 Hz integer units the -0.5 constant is an exact +10 ticks;
     * strict comparisons intentionally leave equality false for both verbs. */
    int64_t target = ((int64_t)started + (secs > 0 ? secs : 0)) * 20;
    int64_t rounded_now = (int64_t)s_tick + 10;
    return greater ? (target < rounded_now) : (rounded_now < target);
}

/* Constant world predicates: the answer each predicate gives in a world
 * without the underlying system. ret is the predicate value. The combat
 * family left this table with D17 (real bodies in combat.c). */
static const MissionConstPred *const_pred_find(const char *name)
{
    static const MissionConstPred k[] = {
        { "isAirborne",     0,  "car is terrain-clamped"               },
        { "isLit",          0,  "no lights model"                      },
        { "isWithinEnemy",  0,  "no enemy proximity model"             },
        { "controlDone",    0,  "no control verbs implemented"         },
        { "camF12",         0,  "no camera/input"                      },
        /* D11: -1 = "no one" — cannot collide with a valid entity idx.
         * nearestBlg stays: no hostile-building picker (never called in
         * P01; the combat family that did land is in the dispatch). */
        { "nearestBlg",     -1, "no hostile-building picker"           },
    };
    for (size_t i = 0; i < sizeof k / sizeof k[0]; i++)
        if (!strcmp(name, k[i].n))
            return &k[i];
    return NULL;
}

static void mission_trap(void *ud, int machine_id, uint32_t pc,
                         uint32_t opcode)
{
    (void)ud;
    fprintf(stdout, "[mission] TRAP machine=%d pc=%u opcode=%u — machine "
                    "halted (unused ops 0/15, malformed operands)\n",
            machine_id, pc, opcode);
    if (opcode < 16)
        s_trap_counts[opcode]++;
    s_trap_total++;
}

static const FsmHost s_host = { mission_dispatch, mission_trap, NULL };

/* ----------------------------------------------------------------------- */
/* Runner reset (FSM state only — never touches terrain/scene)             */
/* ----------------------------------------------------------------------- */

static void mission_runner_reset(void)
{
    if (s_machines) {
        for (int i = 0; i < s_n_machines; i++)
            fsm_machine_destroy(s_machines[i]);
        free(s_machines);
    }
    s_machines = NULL;
    s_n_machines = 0;
    s_machines_alive = 0;

    if (s_img) {
        /* The M7 ledger: per-action call counts by class. */
        fprintf(stdout, "[mission] action ledger for %s "
                "(S=semantic C=const-true-today T=todo-noop "
                ".=never called):\n", s_mission);
        int n = fsm_image_action_count(s_img);
        for (int i = 0; i < n; i++) {
            const char *name = fsm_image_action_name(s_img, i);
            MissionActionStat *st = s_astats ? &s_astats[i] : NULL;
            uint32_t calls = st ? st->calls : 0;
            char cls = '.';
            if (calls) {
                cls = st->cls == LEDGE_SEMANTIC ? 'S' :
                      st->cls == LEDGE_CONST    ? 'C' : 'T';
            }
            fprintf(stdout, "[ledger] %3d %-16s %c %u\n", i,
                    name ? name : "?", cls, calls);
        }
        fprintf(stdout, "[mission] traps: %u total", s_trap_total);
        for (int op = 0; op < 16; op++)
            if (s_trap_counts[op])
                fprintf(stdout, " op%d=%u", op, s_trap_counts[op]);
        fprintf(stdout, "\n");

        fsm_image_free(s_img);
        s_img = NULL;
    }
    free(s_astats);
    s_astats = NULL;

    free(s_ents);
    s_ents = NULL;
    s_nents = 0;
    s_user_ent = -1;

    memset(&s_cam, 0, sizeof s_cam);
    s_cam.path_i = -1;
    s_movie[0] = '\0';
    s_movie_pending = 0;
    s_movie_prev_cam_active = 0;

    free(s_objs);
    s_objs = NULL;
    s_nobj = s_objcap = 0;

    s_state = MISSION_RUNNING;
    s_fail_text_index = 0;
    memset(s_notes, 0, sizeof s_notes);
    s_note_count = 0;
    s_have_message = 0;
    s_message[0] = '\0';
    s_tick = 0;
    s_car_set = 0;
    s_car_vx = s_car_vz = 0.0;
    s_player_contact_ready = 0;
    s_skip_pressed = 0;
    s_script_pos[0] = s_script_pos[1] = s_script_pos[2] = 0.0;
    s_script_yaw = s_script_speed = 0.0;
    s_script_have = 0;
    s_script_snap_tick = UINT64_MAX;
    s_user_snap_pending = 0;
    s_user_snap_speed = 0.0;
    memset(s_navobj, 0, sizeof s_navobj);
    s_navobj_reached_tick = 0;
    s_navobj_reached_have = 0;

    /* Objective controller (D-O1). s_rule_* are session settings and are
     * NOT cleared here — see mission_set_rules. */
    s_family = MISSION_FAMILY_TRIP;
    s_ncheck = s_next_check = s_lap = 0;
    s_has99 = 0;
    s_lap_target = 0;
    s_limit_ticks = 0;
    s_start_ok = 0;
    s_start_yaw = 0.0;
    s_start_pos[0] = s_start_pos[1] = s_start_pos[2] = 0.0;
    memset(s_ctf, 0, sizeof s_ctf);
    s_ctf_teams = s_ctf_carry = s_captures = 0;
    s_capture_target = 0;
    memset(s_melee, 0, sizeof s_melee);
    s_melee_bots = 0;
    s_kill_target = 0;
    s_n_vehicle_spawns = 0;
    s_vehicle_spawn_overflow = 0;

    cb_stop_all();
    /* stopCB's enqueue block re-arms with the mission — the port's
     * equivalent of FUN_00419150(0) clearing DAT_00523110. */
    s_cb_blocked = 0;
    s_trap_total = 0;
    memset(s_trap_counts, 0, sizeof s_trap_counts);
    s_rng = 0x1C0FFEEu;
    s_mission[0] = '\0';
    s_story_clip[MISSION_STORY_INTRO][0] = '\0';
    s_story_clip[MISSION_STORY_OUTRO][0] = '\0';
    ai_reset();             /* D14: drop all mover state with the runner */
    combat_reset();         /* D17: and all combat state with it         */
    free(s_world_boxes);    /* D-A12: and the static scenery table       */
    s_world_boxes = NULL;
    s_nworld_boxes = 0;
}

/* ----------------------------------------------------------------------- */
/* FSM payload resolve + runner start (shared by mission_load/_attach)     */
/* ----------------------------------------------------------------------- */

/* Returns 0 when an FSM image was loaded and its machines created,
 * 1 when the mission is FSM-less, -1 on a load/parse failure. */
static int mission_fsm_load(const char *cbt_path)
{
    /* Mission bytes via the VFS, scene_load's directory fallback (D12). */
    char path[96];
    size_t sz = 0;
    uint8_t *buf = NULL;
    const char *dirs[] = { "", "miss8/", "miss16/", "missions/" };
    for (size_t i = 0; i < sizeof dirs / sizeof dirs[0] && !buf; i++) {
        snprintf(path, sizeof path, "%s%s", dirs[i], cbt_path);
        buf = vfs_read_file(path, &sz);
    }
    if (!buf) {
        fprintf(stderr, "[mission] mission not found: %s\n", cbt_path);
        return -1;
    }
    snprintf(s_mission, sizeof s_mission, "%s", path);

    /* ODEF snapshot for entity resolution (D3). */
    const uint8_t *fsm_pay = NULL;
    size_t fsm_paylen = 0;
    for (size_t off = 0; off < sz; ) {
        MChunk c;
        if (!mchunk_at(buf, sz, off, &c))
            break;
        if (mtag_is(&c, "WDEF"))
            story_fields_load(buf + c.payload, c.total - 8);
        else if (mtag_is(&c, "ODEF"))
            odef_walk(buf + c.payload, c.total - 8);
        else if (mtag_is(&c, "ADEF")) {
            /* ADEF payload: AREV, `FSM `, EXIT (fsm.md §1). */
            const uint8_t *ab = buf + c.payload;
            size_t alen = c.total - 8;
            for (size_t aoff = 0; aoff < alen; ) {
                MChunk a;
                if (!mchunk_at(ab, alen, aoff, &a))
                    break;
                if (mtag_is_fsm(&a)) {
                    fsm_pay    = ab + a.payload;
                    fsm_paylen = a.total - 8;
                } else if (mtag_is(&a, "EXIT")) {
                    break;
                }
                aoff = a.next;
            }
        } else if (mtag_is(&c, "EXIT")) {
            break;
        }
        off = c.next;
    }

    /* D1: a sibling "<base>.fsm" file wins when one exists (none do in
     * the shipped data — the probe's sibling survey proves it). */
    char sibling[96];
    snprintf(sibling, sizeof sibling, "%s", path);
    char *dot = strrchr(sibling, '.');
    if (dot)
        strcpy(dot, ".fsm");
    else
        strcat(sibling, ".fsm");
    size_t sib_sz = 0;
    /* probe first: a miss is the normal case, vfs_read_file would log it */
    uint8_t *sib = vfs_exists(sibling) ? vfs_read_file(sibling, &sib_sz) : NULL;
    if (sib) {
        fprintf(stdout, "[mission] using sibling FSM file %s (%zu bytes)\n",
                sibling, sib_sz);
        fsm_pay = sib;
        fsm_paylen = sib_sz;
    }

    /* FSM-less missions are legal (melee/race/capture modes). They are not
     * objective-less any more: objectives_init() builds the course from the
     * ODEF markers odef_walk has just collected (D-O1). */
    if (fsm_paylen < 4) {
        fprintf(stdout, "[mission] %s: no FSM payload — running FSM-less\n",
                s_mission);
        free(sib);
        vfs_free(buf);
        objectives_init();
        return 1;
    }

    s_img = fsm_image_load(fsm_pay, fsm_paylen);
    free(sib);
    vfs_free(buf);
    if (!s_img) {
        fprintf(stderr, "[mission] %s: FSM image malformed\n", s_mission);
        return -1;
    }

    fprintf(stdout, "[mission] %s: FSM image — %d machines, %d actions, "
                    "%d entities, %d clips, %d paths\n",
            s_mission, fsm_image_machine_count(s_img),
            fsm_image_action_count(s_img), fsm_image_entity_count(s_img),
            fsm_image_clip_count(s_img), fsm_image_path_count(s_img));

    s_astats = calloc((size_t)fsm_image_action_count(s_img),
                      sizeof *s_astats);

    resolve_entities();

    /* Create every machine in the table; per-machine create failures are
     * logged by fsm.c and simply reduce the runnable set. */
    s_n_machines = fsm_image_machine_count(s_img);
    s_machines = calloc((size_t)(s_n_machines > 0 ? s_n_machines : 1),
                        sizeof *s_machines);
    s_machines_alive = 0;
    for (int i = 0; i < s_n_machines; i++) {
        s_machines[i] = fsm_machine_create(s_img, i, &s_host);
        if (s_machines[i])
            s_machines_alive++;
    }
    fprintf(stdout, "[mission] %d/%d machines created\n", s_machines_alive,
            s_n_machines);
    /* D-O1: with an image loaded this only classifies the mission as a
     * scripted TRIP and clears the controller's state. It must still run,
     * or mission_family() would report the PREVIOUS mission's family. */
    objectives_init();
    return 0;
}

/* ----------------------------------------------------------------------- */
/* Public API                                                              */
/* ----------------------------------------------------------------------- */

int mission_load(const char *cbt_path)
{
    mission_unload();

    if (!cbt_path || !*cbt_path) {
        fprintf(stderr, "[mission] load: bad path\n");
        return -1;
    }

    /* Terrain + scene via the existing M2 loaders (owned by us here). */
    if (terrain_load(cbt_path) != 0) {
        fprintf(stderr, "[mission] terrain_load(%s) failed\n", cbt_path);
        return -1;
    }
    if (scene_load(cbt_path) != 0) {
        fprintf(stderr, "[mission] scene_load(%s) failed\n", cbt_path);
        terrain_unload();
        return -1;
    }

    if (mission_fsm_load(cbt_path) < 0) {
        scene_unload();
        terrain_unload();
        return -1;
    }
    /* FSM-less (return 1) is not an error for mission_load. */

    s_owns_world = 1;
    s_loaded = 1;
    s_state = MISSION_RUNNING;
    world_colliders_build();    /* D-A12 static scenery table, once */
    return 0;
}

int mission_attach(const char *path)
{
    /* Drop any previous runner; terrain/scene are the caller's — never
     * touched here (neither loaded nor unloaded). */
    mission_runner_reset();
    scene_gates_reset();

    if (!path || !*path) {
        fprintf(stderr, "[mission] attach: bad path\n");
        return -1;
    }

    int r = mission_fsm_load(path);
    if (r < 0)
        return -1;

    s_owns_world = 0;
    s_loaded = 1;
    s_state = MISSION_RUNNING;
    world_colliders_build();    /* D-A12 static scenery table, once */
    return r;   /* 0 = FSM attached, 1 = FSM-less */
}

void mission_set_car(double x, double z, double yaw)
{
    if (s_car_set) {
        s_car_vx = (x - s_car_x) / AI_TICK_DT;
        s_car_vz = (z - s_car_z) / AI_TICK_DT;
    } else {
        s_car_vx = s_car_vz = 0.0;
    }
    s_car_x = x;
    /*
     * The mission API is deliberately XZ-only for predicates, but camera
     * actions also resolve the `user` entity through ent_pos(). Leaving Y
     * at its zero-initialized value put P01's opening camera below the
     * terrain and inside the convoy. The car is terrain-following whenever
     * a scripted camera owns the view, so publish its ground-relative
     * centre here without widening every 2D mission/probe caller.
     */
    s_car_y = terrain_height_at(x, z) + 1.0;
    s_car_z = z;
    s_car_yaw = yaw;
    s_car_set = 1;
    ai_sync_idle_pose(s_user_ent, x, z, yaw);
    combat_set_user_pose(x, terrain_height_at(x,z)+CAR_MODEL_ORIGIN_H,
                         z, yaw); /* rendered model origin + fire yaw */
}

void mission_set_skip(int pressed)
{
    s_skip_pressed = pressed != 0;
}

int mission_scripted_car(double pos[3], double *yaw, double *speed)
{
    if (!s_loaded || s_user_ent < 0 || !s_script_have)
        return 0;
    if (ai_goal(s_user_ent) == AI_GOAL_NONE && s_script_snap_tick != s_tick)
        return 0;
    if (pos) {
        pos[0] = s_script_pos[0];
        pos[1] = s_script_pos[1];
        pos[2] = s_script_pos[2];
    }
    if (yaw) *yaw = s_script_yaw;
    if (speed) *speed = s_script_speed;
    return 1;
}

int mission_user_teleported(void)
{
    return s_loaded && s_script_snap_tick == s_tick;
}

void mission_tick(void)
{
    if (!s_loaded)
        return;
    s_vehicle_contacts = 0;
    s_player_vehicle_contacts = 0;
    /* playMovie is an asynchronous host handoff. Keep sim-time, scripts,
     * movers, combat and camera motion frozen until the host finishes or
     * skips the queued Smacker clip and acknowledges it. */
    if (s_movie_pending)
        return;

    /* FSM-less: the objective controller is the whole tick (D-O1). The
     * tick counter advances here too — it used to stay at 0 for 55 of the
     * 75 missions, so nothing FSM-less could even measure elapsed time. */
    if (!s_img) {
        s_tick++;
        /* D-O14: a melee is the one FSM-less family with a live world —
         * opponents that move and shoot. Advance it in the FSM path's own
         * order (movers, bodies, combat) before the objective is judged.
         * A race or a capture map has no entities at all, so nothing to do. */
        if (s_family == MISSION_FAMILY_MELEE && s_melee_bots > 0 &&
            s_state == MISSION_RUNNING && s_car_set)
            melee_world_tick();
        objectives_tick();
        s_skip_pressed = 0;
        return;
    }

    int camera_owned_tick = mission_cam_active();
    s_tick++;
    s_nav_poll_order = 0;
    cb_tick();

    /* Round-robin: one bounded slice per live machine (fsm.md §2.4).
     * Machines that yielded resume mid-script next tick; RST/trapped
     * machines halt. */
    for (int i = 0; i < s_n_machines; i++) {
        FsmMachine *m = s_machines[i];
        if (!m || fsm_machine_halted(m))
            continue;
        s_guidance_machine = m;
        FsmStepResult r = fsm_step(m);
        s_guidance_machine = NULL;
        if (r == FSM_STEP_RST || r == FSM_STEP_TRAPPED) {
            s_machines_alive--;
            fprintf(stdout, "[mission] machine %d %s (tick %llu)\n", i,
                    r == FSM_STEP_RST ? "ended (RST)" : "TRAPPED",
                    (unsigned long long)s_tick);
        }
    }

    /* D14: advance the story-motion mover one fixed step, then write
     * AI-driven positions back to the scene objects so the renderer
     * shows the convoy moving. The user is excluded — its on-screen
     * representation is the sim car (scene_set_dynamic) and its bridge
     * pose is the live car (D5). */
    ai_tick(AI_TICK_DT, ent_pos, mission_ai_nav_query);
    ai_world_contacts_tick();       /* D-A12: constrain proposed moves */
    for (int i = 0; i < s_nents; i++) {
        double p[3];
        if (mission_ent_owner(i) != i || s_ents[i].is_user ||
            s_ents[i].scene_obj < 0)
            continue;
        if (ai_get_pos(i, p) == 0)
            mission_ent_writeback(i, s_ents[i].scene_obj, p);
    }
    vehicle_contacts_tick(camera_owned_tick);
    scripted_car_update();


    /* D17: combat runs last — pulses the machines just polled clear,
     * engagements fire against the fresh positions, deaths log now. */
    combat_tick();

    /* D18: advance any path-following cutscene camera after entity motion
     * so look-at targets use this tick's positions. */
    cam_tick();
    s_skip_pressed = 0;
}

int mission_nav_goal(double out_xz[2], double *radius, int *square)
{
    if (!s_loaded) return 0;
    const MissionNavObj *first = NULL;
    int first_order = INT_MAX;
    for (int i = 0; i < MISSION_NAVOBJ_MAX; i++) {
        const MissionNavObj *o = &s_navobj[i];
        if (!o->used || o->seen != s_tick || o->ent != s_user_ent ||
            o->consequence != FSM_NAV_PROGRESS || navobj_danger(o->x, o->z))
            continue;
        /* Several machines can wait on the same destination: one may
         * have an unresolved extra condition while another already proves
         * progress. Keep that destination's original polling priority. */
        int order = o->poll_order;
        for (int k = 0; k < MISSION_NAVOBJ_MAX; k++) {
            const MissionNavObj *p = &s_navobj[k];
            if (p->used && p->seen == s_tick && p->ent == s_user_ent &&
                p->x == o->x && p->z == o->z && p->poll_order < order)
                order = p->poll_order;
        }
        if (order < first_order) { first = o; first_order = order; }
    }
    if (!first) return 0;
    if (out_xz) { out_xz[0] = first->x; out_xz[1] = first->z; }
    if (radius) *radius = first->r;
    if (square) *square = first->sq;
    return 1;
}

/* A progress trigger may share an anchor with another machine's exit
 * boundary. Do not direct the user inside a known live failure region. */
static int navobj_danger(double x, double z)
{
    for (int i = 0; i < MISSION_NAVOBJ_MAX; i++) {
        const MissionNavObj *o = &s_navobj[i];
        if (!navobj_fresh(o) || o->ent != s_user_ent ||
            o->consequence != FSM_NAV_FAILURE) continue;
        double dx = x - o->x, dz = z - o->z;
        if (o->sq ? (fabs(dx) < o->r && fabs(dz) < o->r)
                  : (dx * dx + dz * dz < o->r * o->r)) return 1;
    }
    return 0;
}

int mission_nav_predicates(MissionNavPredicate *out, int max)
{
    int n = 0;
    if (!out || max <= 0 || !s_loaded) return 0;
    for (int i = 0; i < MISSION_NAVOBJ_MAX && n < max; i++) {
        const MissionNavObj *o = &s_navobj[i];
        if (!navobj_fresh(o) || o->ent != s_user_ent) continue;
        out[n].x = o->x; out[n].z = o->z; out[n].r = o->r;
        out[n].sq = o->sq;
        out[n].consequence = (int)o->consequence;
        out[n].guidance = o->consequence == FSM_NAV_PROGRESS &&
                          !navobj_danger(o->x, o->z);
        n++;
    }
    return n;
}

int mission_objective_lines(MissionObjectiveLine *out, int max)
{
    if (!out || max <= 0 || !s_loaded || !s_img)
        return 0;

    /* Stable order: first-seen first (born tick), insertion-sorted —
     * the HUD must never reorder lines between frames. */
    const MissionNavObj *live[MISSION_NAVOBJ_MAX];
    int nlive = 0;
    for (int i = 0; i < MISSION_NAVOBJ_MAX; i++) {
        const MissionNavObj *o = &s_navobj[i];
        if (!navobj_fresh(o) || o->consequence != FSM_NAV_PROGRESS ||
            (o->ent == s_user_ent && navobj_danger(o->x, o->z)))
            continue;
        int j = nlive++;
        while (j > 0 && live[j - 1]->born > o->born) {
            live[j] = live[j - 1];
            j--;
        }
        live[j] = o;
    }

    int n = 0;
    for (int i = 0; i < nlive && n < max; i++) {
        const MissionNavObj *o = live[i];
        int user = o->ent == s_user_ent;
        if (!user) {
            /* A gate on ANOTHER entity is an escort duty: the script
             * waits on that entity to arrive, so the player's cue points
             * at the entity itself. Hostile, dead or hidden entities'
             * gates are script plumbing, never a player objective. */
            if (combat_is_enemy(o->ent) || !combat_alive(o->ent) ||
                combat_is_hidden(o->ent))
                continue;
            int dup = 0;
            for (int k = 0; k < n; k++)
                if (!out[k].user && out[k].ent == o->ent) {
                    dup = 1;
                    break;
                }
            if (dup)
                continue;       /* several gates may wait on one escortee */
            double p[3];
            ent_pos(o->ent, p);
            out[n].x = p[0];
            out[n].z = p[2];
        } else {
            int dup = 0;
            for (int k = 0; k < n; k++)
                if (out[k].user && out[k].x == o->x && out[k].z == o->z)
                    dup = 1;
            if (dup) continue;
            out[n].x = o->x;
            out[n].z = o->z;
        }
        out[n].ent = o->ent;
        out[n].user = user;
        out[n].label = mission_entity_label(o->ent);
        out[n].r = o->r;
        out[n].sq = o->sq;
        n++;
    }
    return n;
}

int mission_objective_reached_age(void)
{
    /* s_tick can move backward through mission_restore — treat that as
     * "never reached" rather than letting the unsigned difference wrap. */
    if (!s_loaded || !s_navobj_reached_have || s_tick < s_navobj_reached_tick)
        return -1;
    uint64_t age = s_tick - s_navobj_reached_tick;
    return age > INT_MAX ? INT_MAX : (int)age;
}

const char *mission_entity_label(int ent)
{
    if (!s_loaded || !s_img || ent < 0 || ent >= s_nents)
        return "";
    const char *l = fsm_image_entity_label(s_img, ent);
    return l ? l : "";
}

int mission_state(void)
{
    return s_state;
}

int mission_fail_text_index(void)
{
    return s_state == MISSION_FAILED ? s_fail_text_index : 0;
}

uint64_t mission_ticks(void)
{
    return s_tick;
}

uint32_t mission_trap_count(void)
{
    return s_trap_total;
}

int32_t mission_cell(int index)
{
    if (!s_img)
        return 0;
    return fsm_image_get_cell(s_img, index);
}

void mission_restore(uint64_t tick, int state)
{
    /* M5 save/resume hook (approved by Main): tick counter + mission
     * state only. FSM machine state, shared cells, timers and the radio
     * queue are NOT restored (DECISION/M7: fsm.h has no machine snapshot
     * API). No existing path changes behavior. */
    s_tick = tick;
    /* The save hook does not persist failAllObj operands. Treat a restored
     * failure as unavailable rather than leaking a reason from an older run. */
    s_fail_text_index = 0;
    s_state = state;
}

const char *mission_message(void)
{
    return s_have_message ? s_message : NULL;
}

int mission_cb_owner(void)
{
    return s_cb_playing ? s_cb_cur.owner : -1;
}

const char *mission_movie_pending(void)
{
    return s_loaded && s_movie_pending ? s_movie : NULL;
}

void mission_movie_ack(void)
{
    if (!s_movie_pending)
        return;
    s_movie_pending = 0;
    s_movie[0] = '\0';
    s_cam.active = s_movie_prev_cam_active;
    s_movie_prev_cam_active = 0;
}

const char *mission_story_clip(int kind)
{
    if (!s_loaded || kind < MISSION_STORY_INTRO ||
        kind > MISSION_STORY_OUTRO)
        return NULL;
    const char *clip = s_story_clip[kind];
    size_t len = strlen(clip);
    return len >= 4 && strcasecmp(clip + len - 4, ".smk") == 0
         ? clip : NULL;
}


int mission_cam_active(void)
{
    return s_cam.active ? 1 : 0;
}

int mission_cam_get(CameraView *view)
{
    if (!s_cam.active)
        return 0;
    if (view)
        *view = s_cam.cur;
    return 1;
}

int mission_spawn(double out[3])
{
    /* !s_img guards the melee entity table: since D-O16 an arena mission
     * HAS a user entity, and its ODEF record is a bare `spawn` marker, not
     * the player's vehicle. Arena starts belong to mission_start_pose,
     * which is what this function's contract already said. */
    if (!s_loaded || !s_img || s_user_ent < 0 || !s_ents)
        return -1;
    const MissionObj *o = s_ents[s_user_ent].obj;
    if (!o)
        return -1;
    out[0] = o->pos[0];
    out[1] = o->pos[1];
    out[2] = o->pos[2];
    return 0;
}

const char *mission_player_object(void)
{
    /* !s_img for the same reason as mission_spawn: a melee's user entity
     * resolves to a `spawn` marker, and "spawn" is not a .vcf. Returning
     * it would make web_drive_load try car_load("spawn") and abort the
     * mission it had already attached. */
    if (!s_loaded || !s_img || s_user_ent < 0 || !s_ents)
        return "";
    const MissionObj *o = s_ents[s_user_ent].obj;
    return o ? o->label : "";
}

void mission_set_player_combat_config(const char *vcf)
{
    if (!s_loaded || !s_img || s_user_ent < 0 || !s_ents || !vcf || !*vcf)
        return;
    const MissionObj *o = s_ents[s_user_ent].obj;
    CarCombatConfig cc;
    if (car_combat_config(vcf, &cc) != 0)
        return;
    combat_register(s_user_ent,
                    o ? (int)o->team : -1,
                    o ? (int)o->class_id : 1,
                    s_ents[s_user_ent].scene_obj,
                    fsm_image_entity_label(s_img, s_user_ent), &cc);
    combat_set_user(s_user_ent);
    /* N-DECODED D-C28: Nitro applies the non-multiplayer player-only 2x pool
     * initialization after the shell-selected VCF has replaced the mission
     * placeholder. Re-registration above resets the one-shot latch. */
    combat_apply_singleplayer_player_defense();
}

const char *mission_probe_player_object(const char *path)
{
    static char out[16];
    out[0] = '\0';
    if (!path || !*path)
        return out;

    char found[96];
    size_t sz = 0;
    uint8_t *buf = NULL;
    const char *dirs[] = { "", "miss8/", "miss16/", "missions/" };
    for (size_t i = 0; i < sizeof dirs / sizeof dirs[0] && !buf; i++) {
        snprintf(found, sizeof found, "%s%s", dirs[i], path);
        buf = vfs_read_file(found, &sz);
    }
    if (!buf)
        return out;

    const uint8_t *fsm_pay = NULL;
    size_t fsm_paylen = 0;
    MissionObj *old_objs = s_objs;
    int old_nobj = s_nobj, old_cap = s_objcap;
    s_objs = NULL;
    s_nobj = s_objcap = 0;

    for (size_t off = 0; off < sz; ) {
        MChunk c;
        if (!mchunk_at(buf, sz, off, &c)) break;
        if (mtag_is(&c, "ODEF")) {
            odef_walk(buf + c.payload, c.total - 8);
        } else if (mtag_is(&c, "ADEF")) {
            const uint8_t *ab = buf + c.payload;
            size_t alen = c.total - 8;
            for (size_t aoff = 0; aoff < alen; ) {
                MChunk a;
                if (!mchunk_at(ab, alen, aoff, &a)) break;
                if (mtag_is_fsm(&a)) {
                    fsm_pay = ab + a.payload;
                    fsm_paylen = a.total - 8;
                }
                if (mtag_is(&a, "EXIT")) break;
                aoff = a.next;
            }
        }
        if (mtag_is(&c, "EXIT")) break;
        off = c.next;
    }

    FsmImage *img = fsm_paylen >= 4 ? fsm_image_load(fsm_pay, fsm_paylen) : NULL;
    if (img) {
        int n = fsm_image_entity_count(img);
        for (int i = 0; i < n && !out[0]; i++) {
            const char *label = fsm_image_entity_label(img, i);
            if (!label || strcasecmp(label, "user") != 0) continue;
            const char *object = fsm_image_entity_object(img, i);
            char oname[9];
            int oid;
            if (!object) break;
            label_unpack((const uint8_t *)object, oname, &oid);
            for (int j = 0; j < s_nobj; j++)
                if (s_objs[j].label_id == oid &&
                    strcasecmp(s_objs[j].label, oname) == 0) {
                    snprintf(out, sizeof out, "%s", s_objs[j].label);
                    break;
                }
        }
        fsm_image_free(img);
    }

    free(s_objs);
    s_objs = old_objs;
    s_nobj = old_nobj;
    s_objcap = old_cap;
    vfs_free(buf);
    return out;
}

void mission_set_vehicle_contact_host(MissionVehicleRadiusFn radius,
                                      MissionVehicleHeightFn height,
                                      MissionVehicleSeparateFn separate,
                                      void *ctx)
{
    s_vehicle_radius = radius;
    s_vehicle_height = height;
    s_vehicle_separate = separate;
    s_vehicle_contact_ctx = ctx;
}

int mission_scene_object_is_vehicle(int scene_obj)
{
    if (!s_loaded || scene_obj < 0)
        return 0;
    for (int ent = 0; ent < s_nents; ent++)
        if (s_ents[ent].scene_obj == scene_obj && mission_ent_is_vehicle(ent))
            return 1;
    return 0;
}

int mission_entity_scene_object(int ent)
{
    ent = mission_ent_owner(ent);
    return s_loaded && ent >= 0 ? s_ents[ent].scene_obj : -1;
}

int mission_scene_object_is_player_vehicle(int scene_obj)
{
    return s_loaded && scene_obj >= 0 && s_user_ent >= 0 &&
           s_user_ent < s_nents &&
           s_ents[s_user_ent].scene_obj == scene_obj &&
           mission_ent_is_vehicle(s_user_ent);
}

int mission_vehicle_spawn_count(void)
{
    return s_loaded ? s_n_vehicle_spawns : 0;
}

int mission_vehicle_spawn(int index, MissionVehicleSpawn *out)
{
    if (!s_loaded || !out || index < 0 || index >= s_n_vehicle_spawns)
        return -1;
    *out = s_vehicle_spawns[index];
    return 0;
}

int mission_vehicle_spawn_overflow(void)
{
    return s_loaded && s_vehicle_spawn_overflow;
}

uint32_t mission_vehicle_contacts(void)
{
    return s_vehicle_contacts;
}

uint32_t mission_player_vehicle_contacts(void)
{
    return s_player_vehicle_contacts;
}

int mission_world_contact_stats(int ent, MissionWorldContactStats *out)
{
    if (!out || !s_loaded)
        return -1;
    ent = mission_ent_owner(ent);
    if (ent < 0)
        return -1;
    out->static_contacts = s_ents[ent].world_static_contacts;
    out->terrain_refusals = s_ents[ent].world_terrain_refusals;
    out->pushouts = s_ents[ent].world_pushouts;
    return 0;
}

int mission_contact_count(void)
{
    return s_loaded ? s_nents : 0;
}

int mission_contact(int index, MissionContact *out)
{
    if (!out || !s_loaded || index < 0 || index >= s_nents)
        return -1;
    int owner = mission_ent_owner(index);
    double p[3];
    ent_pos(owner, p);
    out->x = p[0];
    out->z = p[2];
    out->alive = combat_alive(owner);
    /* Aliases remain addressable by their FSM index but do not become extra
     * radar contacts; the canonical body carries the visible contact. */
    out->hidden = index != owner || combat_is_hidden(owner);
    if (owner == s_user_ent) {
        out->relation = 2;
    } else if (combat_is_enemy(owner)) {
        out->relation = -1;
    } else {
        int user_team = combat_team(s_user_ent);
        int team = combat_team(owner);
        out->relation = user_team >= 0 && team == user_team ? 1 : 0;
    }
    return 0;
}

int mission_family(void)
{
    return s_loaded ? s_family : MISSION_FAMILY_TRIP;
}

void mission_set_rules(int lap_target, int capture_target, int kill_target,
                       int limit_minutes)
{
    s_rule_laps     = lap_target     > 0 ? lap_target     : 0;
    s_rule_captures = capture_target > 0 ? capture_target : 0;
    s_rule_kills    = kill_target    > 0 ? kill_target    : 0;
    s_rule_minutes  = limit_minutes  > 0 ? limit_minutes  : 0;
}

int mission_start_pose(double pos[3], double *yaw)
{
    if (!s_loaded || !s_start_ok)
        return -1;
    if (pos) {
        pos[0] = s_start_pos[0];
        pos[1] = s_start_pos[1];
        pos[2] = s_start_pos[2];
    }
    if (yaw)
        *yaw = s_start_yaw;
    return 0;
}

int mission_objective_state(MissionObjectiveState *out)
{
    if (!out)
        return 0;
    memset(out, 0, sizeof *out);
    out->family    = s_loaded ? s_family : MISSION_FAMILY_TRIP;
    out->secs_left = -1;

    if (!s_loaded || s_img)
        return 0;

    /* tx/tz: the point the player has to reach next. Both families fill
     * it, which is what lets one HUD line and one JSON blob serve both. */
    double tx, tz;

    if (s_family == MISSION_FAMILY_RACE && s_ncheck > 0) {
        int on_finish = s_has99 && s_next_check >= s_ncheck;
        out->lap        = s_lap;
        out->lap_target = s_lap_target;
        out->gate       = s_next_check + 1;
        out->gates      = s_ncheck + s_has99;   /* check99 is the last gate */
        tx = on_finish ? s_finish[0] : s_course[s_next_check].x;
        tz = on_finish ? s_finish[2] : s_course[s_next_check].z;
    } else if (s_family == MISSION_FAMILY_CAPTURE && s_ctf_teams > 0) {
        /* Resolve the target BEFORE writing anything: the contract says a
         * 0 return leaves `out` zeroed apart from `family`. */
        int target;
        if (s_ctf_carry) {
            /* Heading home: aim at the NEAREST slot of our own base, the
             * same slot the capture test scores against (D-O9). */
            const MissionCtfTeam *me = &s_ctf[CTF_PLAYER_TEAM - 1];
            double best = -1.0;
            tx = tz = 0.0;
            for (int i = 0; i < me->nslots; i++) {
                double dx = s_car_x - me->sx[i], dz = s_car_z - me->sz[i];
                double d2 = dx * dx + dz * dz;
                if (best < 0.0 || d2 < best) {
                    best = d2; tx = me->sx[i]; tz = me->sz[i];
                }
            }
            if (best < 0.0)
                return 0;               /* ctf_build guarantees nslots>0  */
            target = CTF_PLAYER_TEAM;
        } else {
            target = ctf_nearest_enemy_flag(NULL);
            if (!target)
                return 0;               /* ctf_build guarantees target!=0 */
            tx = s_ctf[target - 1].fx;
            tz = s_ctf[target - 1].fz;
        }
        out->captures       = s_captures;
        out->capture_target = s_capture_target;
        out->carrying       = s_ctf_carry;
        out->teams          = s_ctf_teams;
        out->team           = CTF_PLAYER_TEAM;
        out->target_team    = target;
    } else if (s_family == MISSION_FAMILY_MELEE && s_melee_bots > 0) {
        /* The nearest LIVE opponent is what the readout points at. Unlike
         * a checkpoint or a flag this is a moving target and the HUD line
         * is the only thing that says where the fight is — the maps run to
         * 4 km and an opponent 800 m away is past the draw distance. */
        double best = -1.0;
        int alive = 0;
        tx = s_car_x;
        tz = s_car_z;
        for (int i = 0; i < s_melee_bots; i++) {
            double p[3];
            if (!combat_alive(s_melee[i].ent))
                continue;
            alive++;
            if (ai_get_pos(s_melee[i].ent, p) != 0)
                continue;
            double dx = p[0] - s_car_x, dz = p[2] - s_car_z;
            double d2 = dx * dx + dz * dz;
            if (best < 0.0 || d2 < best) {
                best = d2; tx = p[0]; tz = p[2];
            }
        }
        out->kills        = combat_kills(0);
        out->kill_target  = s_kill_target;
        out->opponents    = s_melee_bots;
        out->opponents_alive = alive;
        out->hp           = combat_hp(0);
        out->hp_max       = combat_hp_max(0);
    } else {
        return 0;
    }

    double dx = tx - s_car_x;
    double dz = tz - s_car_z;
    out->gate_dist = sqrt(dx * dx + dz * dz);

    /* Bearing relative to the car's own heading, so the HUD can say "left"
     * or "right" rather than a compass number the player has to convert. */
    double b = atan2(-dx, dz) - s_car_yaw;
    while (b >  MISSION_PI) b -= 2.0 * MISSION_PI;
    while (b < -MISSION_PI) b += 2.0 * MISSION_PI;
    out->gate_bearing = b;

    if (s_limit_ticks > 0)
        out->secs_left = s_tick >= s_limit_ticks ? 0
            : (int)((s_limit_ticks - s_tick) / MISSION_TICK_HZ);
    return 1;
}

void mission_unload(void)
{
    int had_world = s_owns_world;

    mission_runner_reset();

    /* Only mission_load takes ownership of terrain/scene; an attached
     * runner (mission_attach) leaves the caller's world alone. */
    if (had_world) {
        scene_unload();
        terrain_unload();
    }
    s_owns_world = 0;
    s_loaded = 0;
}

int mission_is_loaded(void)
{
    return s_loaded;
}

int mission_path_nodes(int path_index, float *out, int max_nodes)
{
    const float *pts = NULL;
    if (!s_img || !out || max_nodes <= 0)
        return 0;
    int n = fsm_image_path(s_img, path_index, &pts);
    if (n <= 0 || !pts)
        return 0;
    if (n > max_nodes)
        n = max_nodes;
    memcpy(out, pts, (size_t)n * 3 * sizeof *out);
    return n;
}
