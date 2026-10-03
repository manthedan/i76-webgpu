#ifndef MISSION_H
#define MISSION_H

#include <stdint.h>
#include "engine/camera.h"

/*
 * mission.h — mission runner: terrain + scene + mission FSM (M7 foundation)
 *
 * Glues the M2 world loaders (terrain.c, scene.c) to the M3 mission-script
 * VM (fsm.c) and hosts the FSM<->game bridge: the FsmHost dispatch that
 * turns the script's action table into engine state. This is what makes a
 * mission ALIVE instead of a diorama: scripts poll the player car's pose,
 * queue radio messages, and flip the mission to complete/failed.
 *
 * Data reality (verified against the extracted Nitro Pack + base game,
 * tools/mission_probe.c's sibling survey):
 *   - Melee/race/capture missions (the miss8 .CBT/.RAC/.CF2/3/4 files) carry an
 *     ADEF chunk with an EMPTY `FSM ` payload — they run FSM-less.
 *   - Trip missions (miss8/P*.MSN, and vanilla A/S/T*.MSN in the base data)
 *     embed the full seven-table FSM image in ADEF (docs/specs/m3/fsm.md
 *     §1). NO mission ships a sibling "<base>.FSM" file; mission_load
 *     still honors one when present (see DECISION D1 in mission.c).
 *
 * Fixed-step contract: the integrator calls mission_set_car() with the
 * player car's current pose and then mission_tick() once per 20 Hz sim
 * tick (car.h CAR_SIM_HZ). The FSM itself is frame-coupled in the
 * original (fsm.md §2.4); at the fixed 20 Hz step the two coincide.
 *
 * Render-free by design: the web shell reads state through the getters
 * and draws with hud.c/scene.c; this module never touches a framebuffer.
 */

/* Mission states returned by mission_state(). */
enum {
    MISSION_RUNNING  = 0,
    MISSION_COMPLETE = 1,
    MISSION_FAILED   = 2
};

/*
 * Mission families (mission_family / mission_family_of_path).
 *
 * TRIP is every scripted mission: its FSM owns the objectives and the
 * FSM-less objective controller stays out of its way. The other three are
 * the arena modes the original's own shell names (nitshell.dll: "Race",
 * "Combat", "2/3/4 Team C.T.F."), which ship an EMPTY `FSM ` payload and
 * are driven from the ODEF marker table instead — see mission.c D-O1.
 */
enum {
    MISSION_FAMILY_TRIP    = 0,
    MISSION_FAMILY_MELEE   = 1,   /* .CBT — spawned opponents (D-O14)     */
    MISSION_FAMILY_RACE    = 2,   /* .RAC — check1..checkN course         */
    MISSION_FAMILY_CAPTURE = 3    /* .CF2/3/4 — #spawnN/a1flagN (D-O8)    */
};

/*
 * mission_family()
 *   The live mission's family, or MISSION_FAMILY_TRIP when nothing is
 *   loaded. Resolved at load time from the file extension, with two
 *   corrections mission_family_of_path() cannot make on a name alone:
 *   a mission that HAS an FSM image is always TRIP, and the one .MSN
 *   that ships an empty FSM payload (miss8/N35.MSN) is recognised as a
 *   RACE by its check1..checkN markers.
 *
 * mission_team_name(team)
 *   The display name of a 1-based capture team. The names are nitro.exe's
 *   own team-name pool, verbatim; WHICH name belongs to which team slot is
 *   INVENTED (mission.c D-O12). Out-of-range teams get the binary's own
 *   fallback, "Unknown Loser". Never NULL.
 *
 * mission_family_of_path(path)
 *   The family a path's extension implies, with no file access and no
 *   loaded mission — what a menu needs to group missions before it can
 *   afford to open them. Returns MISSION_FAMILY_TRIP for ".MSN" and for
 *   anything unrecognised, so N35.MSN reads as a TRIP here and as a RACE
 *   once loaded. Case-insensitive.
 */
int mission_family(void);
int mission_family_of_path(const char *path);
const char *mission_team_name(int team);

/*
 * mission_set_rules(lap_target, capture_target, kill_target, limit_minutes)
 *   The arena SESSION rules: how many laps win a race, how many flag
 *   captures win a capture map, how many kills win a melee, and how many
 *   minutes before the clock loses any of them (0 = the original's
 *   "No Limits!"). The kill target is CLAMPED at load to the number of
 *   opponents the map actually fielded (mission.c D-O16) — asking for more
 *   kills than there are cars would be a mission that can only be lost.
 *
 *   These are host state, not mission state, and that is not a shortcut:
 *   no per-mission rules record exists in the shipped data. Every one of
 *   the 75 miss8 missions carries a 331-byte WDEF/WRLD payload whose
 *   candidate limit field is the same value in all 75, and the original
 *   keeps the mode limits in its multiplayer session setup instead
 *   (nitshell.dll: "No Limits!", "%d Minutes", "%d Laps", "%d Kills",
 *   "%d Points", "%d Captures" in one adjacent block). So the host picks
 *   them, exactly as the original's own game-setup screen does.
 *
 *   Call BEFORE mission_load/mission_attach; the values are session
 *   settings and deliberately survive mission_unload(). Negative values
 *   are clamped to 0. The defaults are INVENTED (mission.c D-O4/D-O11).
 */
void mission_set_rules(int lap_target, int capture_target, int kill_target,
                       int limit_minutes);

/*
 * mission_objective_state(out)
 *   The live objective readout for the HUD and the debrief.
 *
 *   RACE fills lap/lap_target/gate/gates. CAPTURE fills captures/
 *   capture_target/carrying/teams/team. MELEE fills kills/kill_target/
 *   opponents/opponents_alive/hp/hp_max. ALL THREE fill gate_dist/
 *   gate_bearing, which point at whatever the player has to reach next —
 *   the next checkpoint on a race; on a capture either the nearest enemy
 *   flag or, once one is aboard, the player's own base; in a melee the
 *   nearest LIVE opponent, which unlike the other two is moving.
 *   secs_left is the clock for all three (-1 = no limit armed).
 *
 *   Returns 1 when an objective controller is actually driving this
 *   mission and `out` is meaningful, 0 otherwise (every scripted trip, and
 *   any arena mission whose course or grid failed to build).
 *   `out->family` is filled either way; the rest is zeroed on a 0 return.
 */
typedef struct {
    int    family;        /* MISSION_FAMILY_*                             */
    int    lap;           /* RACE: laps completed so far                  */
    int    lap_target;    /* RACE: laps needed to win                     */
    int    gate;          /* RACE: next checkpoint, 1-based               */
    int    gates;         /* RACE: checkpoints in the course              */
    double gate_dist;     /* metres to the next objective point (XZ)      */
    double gate_bearing;  /* radians off the car's heading, + = to the left */
    int    secs_left;     /* seconds on the clock, -1 = no limit          */
    int    captures;      /* CTF: flags brought home                      */
    int    capture_target;/* CTF: flags needed to win                     */
    int    carrying;      /* CTF: team whose flag is aboard, 0 = none     */
    int    target_team;   /* CTF: team gate_dist/gate_bearing point at    */
    int    teams;         /* CTF: teams on this map (2..4)                */
    int    team;          /* CTF: the player's team, 1-based              */
    int    kills;         /* MELEE: opponents the PLAYER destroyed        */
    int    kill_target;   /* MELEE: kills needed to win                   */
    int    opponents;     /* MELEE: opponents fielded at the start        */
    int    opponents_alive; /* MELEE: how many are still driving          */
    int    hp;            /* MELEE: the player's damage pool, -1 unknown  */
    int    hp_max;        /* MELEE: its capacity                          */
} MissionObjectiveState;

int mission_objective_state(MissionObjectiveState *out);

/*
 * mission_start_pose(pos, yaw)
 *   Where an arena mission puts the player, and which way it points them.
 *
 *   Arena missions have no FSM `user` entity, so mission_spawn() cannot
 *   answer for them and hosts fell back to "the first ODEF marker of any
 *   kind, facing north" — which on a race is frequently the `regen` point
 *   rather than a grid slot, aimed at whatever happens to be north. The
 *   grid slot and its heading are both in the mission file: this returns
 *   the lowest-numbered `spawn` marker's position and the yaw of its ODEF
 *   frame (car.h convention: forward = (-sin yaw, 0, cos yaw)).
 *
 *   Returns 0 and fills pos[3] (metres) and *yaw (radians) on success,
 *   -1 when the mission has no usable spawn marker — callers keep their
 *   existing fallback for that case. Either argument may be NULL.
 */
int mission_start_pose(double pos[3], double *yaw);

/*
 * mission_load(cbt_path)
 *   Load a mission end to end: terrain and scene via the existing loaders
 *   (terrain_load/scene_load), then the mission script: a sibling
 *   "<base>.fsm" file when the VFS has one (none do in the shipped data),
 *   else the ADEF/`FSM ` chunk embedded in the mission file itself.
 *   `cbt_path` is the mission as the VFS knows it (e.g. "miss8/n01.cbt";
 *   the miss8/ search fallback matches scene.c). A mission WITHOUT an FSM
 *   payload is not an error — it loads and runs FSM-less (state stays
 *   MISSION_RUNNING, mission_message() stays NULL).
 *   fs_set_root() + vfs_init() must have been called by the host.
 *   Returns 0 on success, -1 when the mission file/terrain/scene cannot
 *   be loaded or an embedded FSM image is malformed.
 */
int mission_load(const char *cbt_path);

/*
 * mission_attach(path)
 *   FSM-only attach for hosts that have ALREADY loaded terrain + scene
 *   themselves (the web drive path loads them for its HUD hook). Skips
 *   the terrain/scene loaders entirely and never takes ownership of them
 *   — mission_unload() after an attach leaves the caller's world loaded.
 *   Resolves and loads the mission script exactly like mission_load
 *   (sibling .fsm per D1, else the embedded ADEF/`FSM ` chunk per D2) and
 *   resets all runner state (machines, timers, radio queue, ledger,
 *   mission state/message).
 *   Returns 0 when an FSM image was attached, 1 when the mission is
 *   FSM-less (the same non-error case as in mission_load), -1 when the
 *   mission file or FSM image cannot be loaded.
 */
int mission_attach(const char *path);

/*
 * mission_set_car(x, z, yaw)
 *   Publish the player car's current pose to the mission. Called by the
 *   integrator every tick before mission_tick(). The FSM bridge resolves
 *   the script's `user` entity to this pose. (y comes from the terrain
 *   clamp on the car side; the bridge's distance checks are 2D XZ — see
 *   mission.c DECISION D6.)
 */
void mission_set_car(double x, double z, double yaw);

/*
 * mission_set_skip(pressed)
 *   Publish the cutscene-skip edge for the next mission_tick(). The original
 *   FSM action isKeypress polls Space; the value remains visible to every
 *   machine during that tick and is then cleared. Pass 0 on ordinary ticks.
 */
void mission_set_skip(int pressed);

/*
 * mission_scripted_car(pos, yaw, speed)
 *   The FSM mover's live player-car target while a script owns that entity
 *   through goto/follow/race/teleport. Returns 1 and copies the kinematic
 *   target, heading, and forward speed; returns 0 after sit/handoff or when no
 *   scripted player exists. Any output pointer may be NULL.
 *
 *   This is the host ownership boundary for intro/autopilot sequences. It
 *   does not make the AI mover physical; the car host must apply the target
 *   to its real car state before publishing mission_set_car().
 */
int mission_scripted_car(double pos[3], double *yaw, double *speed);

/* An authored user teleport occurred in the last mission_tick. The host
 * must apply mission_scripted_car even when no camera owns that tick. */
int mission_user_teleported(void);

/*
 * mission_nav_goal(out_xz, radius, square)
 *   PORT GUIDANCE hook (added for the drive HUD cue; not an original
 *   engine API). Scripted trips keep their objectives inside the FSM, but
 *   the one thing a script polls the PLAYER about is arrival: this
 *   reports an UNSATISFIED progress isWithinNav/isWithinSqNav gate
 *   evaluated for the `user` entity during the current mission_tick —
 *   the gate's path node-0 XZ (the BINARY-VERIFIED anchor, D6) and
 *   radius, plus whether the gate is a square.
 *
 *   Only records refreshed during this mission_tick are eligible, so a 1
 *   return always means "the script is waiting on the player to be here
 *   right now". Returns 0 for FSM-less missions (the arena objective
 *   controller owns those — mission_objective_state), before the first
 *   tick, and on any tick the script polls no unsatisfied user gate.
 *   Any output pointer may be NULL.
 */
int mission_nav_goal(double out_xz[2], double *radius, int *square);

/*
 * mission_objective_lines(out, max)
 *   PORT GUIDANCE objective lines (added for the drive HUD; not an
 *   original engine API). mission_nav_goal above lives exactly one tick,
 *   so a HUD reading it directly flips between targets when two FSM
 *   machines poll different gates on alternating round-robin ticks
 *   (H-UAT-014). This getter instead reports progress nav gates the script
 *   is currently waiting on, tracked across ticks by predicate site with
 *   a short poll TTL, in stable first-seen order:
 *
 *   - a progress gate on the `user` entity is a point the PLAYER can reach
 *     (user=1; x/z is the gate's node-0 anchor, D6);
 *   - a gate on another live, non-hostile entity is an escort duty —
 *     the script is waiting on THAT entity to arrive, so x/z is the
 *     entity's live position, the thing the player drives with.
 *     Hostile/dead/hidden entities' gates are script plumbing and are
 *     never reported.
 *
 *   User arrival consequences are a bounded bytecode lookahead (port UI,
 *   not native objective semantics). Failure and unknown branches, plus
 *   destinations inside another live failure region, are suppressed.
 *
 *   `label` is the mission's own authored FSM entity label (P01:
 *   "tanker1", "enemy5"), "" when unnamed; never NULL. Returns the
 *   number of lines written (0 for FSM-less missions — the arena
 *   controller owns those via mission_objective_state).
 *
 * mission_objective_reached_age()
 *   Ticks since a TRACKED user gate was last GENUINELY satisfied while
 *   the mission was still RUNNING, -1 when that never happened. This is
 *   the only honest "OBJECTIVE REACHED" cue: a gate the script merely
 *   stops polling (a fail branch, a wave change) does not count.
 */
typedef struct {
    int         ent;    /* FSM entity the gate waits on                  */
    int         user;   /* 1 = the player must reach x/z themselves      */
    const char *label;  /* authored FSM entity label, "" when unnamed    */
    double      x, z;   /* gate anchor (user) / live entity pos (escort) */
    double      r;      /* gate radius, metres                           */
    int         sq;     /* 1 = square gate (isWithinSqNav)               */
} MissionObjectiveLine;

int mission_objective_lines(MissionObjectiveLine *out, int max);
int mission_objective_reached_age(void);

/* Read-only audit of live user predicates, including suppressed boundaries.
 * consequence: 0 unknown, 1 progress, 2 failure (FsmNavConsequence).
 * A progress site inside another live failure region has guidance=0. */
typedef struct {
    double x, z, r;
    int sq, consequence, guidance;
} MissionNavPredicate;
int mission_nav_predicates(MissionNavPredicate *out, int max);

/*
 * mission_entity_label(ent)
 *   The authored FSM entity label for a live mission entity ("tanker1"),
 *   "" when out of range, unnamed, or no scripted mission is loaded.
 *   Never NULL. Debrief/player-language surfaces use it to name what
 *   the mission itself named.
 */
const char *mission_entity_label(int ent);
/* Resolved body owner's scene object for read-only pose diagnostics; -1 if absent. */
int mission_entity_scene_object(int ent);

/*
 * mission_tick(void)
 *   Advance the mission one 20 Hz tick: radio playback bookkeeping, then
 *   one bounded instruction slice per live FSM machine (round-robin,
 *   fsm.md §2.4). No-op when no mission is loaded.
 *
 *   An FSM-LESS mission still ticks: it advances the tick counter and
 *   runs the objective controller (mission.c D-O1), which is what gives
 *   the arena families a win and a loss. It used to return immediately,
 *   which is why 55 of the 75 shipped missions could be driven but never
 *   finished.
 */
void mission_tick(void);

/* 0 = running, 1 = complete, 2 = failed (see the enum above). */
int mission_state(void);

/*
 * mission_fail_text_index()
 *   The raw positive, 1-based SECOND operand of the failAllObj action that
 *   failed the current mission. This selects a nonempty entry in the loaded
 *   scenario's NPT failure section; placeholder entries still occupy an
 *   index. Returns 0 when the mission is not failed, another failure path
 *   ended it, the operand was invalid, or the reason is unavailable.
 */
int mission_fail_text_index(void);

/*
 * mission_ticks() / mission_restore(tick, state)
 *   M5 save/resume hooks (DECISION: additive for save.c, approved by
 *   Main). mission_ticks returns the runner's 20 Hz tick counter (0
 *   before the first mission_tick). mission_restore sets ONLY the tick
 *   counter and the mission state (MISSION_*); it does NOT restore FSM
 *   machine state (IP/SP/stack), shared image cells, timers, or the
 *   radio queue -- full FSM snapshot/restore is an M7 DECISION (fsm.h
 *   exposes no machine state accessors).
 */
uint64_t mission_ticks(void);
void mission_restore(uint64_t tick, int state);

/*
 * mission_trap_count()
 *   Total FSM machine traps since load (unknown opcodes / bad operands;
 *   each trapped machine halted). Additive probe/debug getter (M7 combat
 *   probe asserts 0 while the combat-reachable machines run).
 */
uint32_t mission_trap_count(void);

/*
 * mission_cell(index)
 *   Shared FSM image cell value (spec table 6); 0 for out-of-range or
 *   no mission loaded. Additive probe/debug getter (M7: the combat
 *   probe's gate evidence reads the mission's progress counters —
 *   P01's cell10/11/12 intro/enemy-kill/convoy-arrival cells).
 */
int32_t mission_cell(int index);

/*
 * mission_message()
 *   The last text/message the FSM produced: the radio clip whose
 *   playback started most recently, formatted "<owner>: <clip>" for
 *   owned messages (cbFrom/cbFromPrior) and "<clip>" otherwise, or the
 *   fail/complete reason after a terminal action. NULL when no message
 *   has been produced (or no mission is loaded). Owned by the module;
 *   valid until the next mission_tick()/mission_unload().
 */
const char *mission_message(void);

/*
 * mission_cb_owner()
 *   The entity index of the owner attached to the radio line currently
 *   playing (the FSM's cbFrom/cbFromPrior entity operand, retained on
 *   the queue node through playback — nitro.exe FUN_00419170's entity
 *   parameter), or -1 when no line is playing or the playing line is
 *   ownerless (cb/cbPrior). Follows the same line mission_message()
 *   reports while it plays.
 */
int mission_cb_owner(void);

/*
 * mission_movie_pending() / mission_movie_ack()
 *   Asynchronous host handoff for the scripted playMovie action. Pending
 *   returns the FSM clip-table filename until the host finishes, skips, or
 *   cannot decode it. While pending, mission_tick is frozen. The host must
 *   acknowledge exactly once to resume; ack is harmless when none is queued.
 */
const char *mission_movie_pending(void);
void mission_movie_ack(void);

/*
 * mission_story_clip(kind)
 *   The loaded mission's own story-movie claim from its WDEF/WRLD record:
 *   two fixed 13-byte NUL-padded fields at payload +4 (intro) and +17
 *   (outro). This is the membership the mission DATA asserts — base Trip
 *   missions name their clips there (T01: int01f01.smk + out01f01.smk …
 *   T17: out17f01.smk only); Nitro missions leave both fields empty, so
 *   the getter is intrinsically base-only and needs no profile gate.
 *
 *   What the retail SHELL did with these clips (exact play points, the
 *   fate of T14's preview.smk, failure-path placement) is NOT answered
 *   here: the mission→SMK sequencing graph remains an open RE residual
 *   (docs/specs/re/phase-e-base-delta.md §6). The port's placement of
 *   the returned clips is a port decision made by the web shell, not a
 *   claim about the original.
 *
 *   Missions whose FSM executes playMovie itself (base T05/T06 Ang*)
 *   are unaffected — those clips do not match the int/out prefixes and
 *   still flow through mission_movie_pending().
 *
 *   NULL for an invalid kind, no loaded mission, an empty field, or a field
 *   that is not an .smk name. Lowercase storage is owned by mission state
 *   and remains valid until mission_unload().
 */
enum { MISSION_STORY_INTRO = 0, MISSION_STORY_OUTRO = 1 };
const char *mission_story_clip(int kind);


/*
 * mission_cam_active() / mission_cam_get(view)
 *   Cutscene camera stack (D18). The getter copies the authoritative
 *   native-compatible eye/right/up/forward basis and returns 1, or returns 0
 *   when the driving camera should be used instead.
 */
int mission_cam_active(void);
int mission_cam_get(CameraView *view);

/*
 * mission_spawn(out)
 *   The player spawn: world position of the ODEF object the FSM's `user`
 *   entity resolves to (e.g. P01's vdrampg2 car record). Arena missions
 *   have no user entity — use mission_start_pose() there instead, which
 *   also carries the heading. Returns 0 and fills out[3] (x,y,z meters)
 *   on success, -1 when unresolved.
 */
int mission_spawn(double out[3]);

/*
 * mission_player_object(void)
 *   The ODEF label of the object the FSM's `user` entity resolves to —
 *   for every scripted Nitro trip that label IS the player's .vcf base
 *   name (P01 "vdrampg2", P03 "vjsovrn1", P13 "vleoprd2", P09 the
 *   mission-only "p09rm01"). Callers use it to load the car the mission
 *   itself names instead of a hard-coded one.
 *
 *   Returns "" (never NULL) when there is no user entity — every
 *   FSM-less melee/race/capture mission takes that path, because
 *   mission_fsm_load() returns before resolve_entities() there.
 */
const char *mission_player_object(void);

/* Rebind the scripted user entity to the garage-selected VCF after
 * mission_attach(), then apply Nitro's decoded non-multiplayer player-only
 * 2x defense initialization once to the new live/max pools (combat.h D-C28).
 * No-op for FSM-less arenas (their player is built by the melee controller)
 * or when the selected VCF cannot supply combat data. */
void mission_set_player_combat_config(const char *vcf);

/*
 * mission_probe_player_object(path)
 *   Read the player VCF label from a staged scripted mission without loading
 *   terrain/scene or mutating the live mission runner. This is the shell's
 *   preflight seam: the Scenario/Driver form can show the mission-authored car
 *   before ENTER AREA. Returns "" for bad/missing/FSM-less missions.
 *   The returned static buffer is replaced by the next call.
 */
const char *mission_probe_player_object(const char *path);

/*
 * Dynamic vehicle-contact host seam.
 *
 * mission.c owns live AI membership, geometry, pair ordering, and AI
 * displacement. The host owns the one physical player car. At the post-mover
 * contact phase mission calls radius(ctx) and height(y0,y1,ctx), then
 * separate(...) once per player contact. The normal points in the player's
 * correction direction; other_v* is the contacted mover's authored world-XZ
 * velocity for this tick.
 *
 * Any NULL callback disables physical-player contacts; AI/AI separation
 * remains mission-owned. Registration is host state and survives mission
 * load/unload, like mission_set_rules.
 */
typedef double (*MissionVehicleRadiusFn)(void *ctx);
typedef void (*MissionVehicleHeightFn)(double *y0, double *y1, void *ctx);
typedef void (*MissionVehicleSeparateFn)(double dx, double dz,
                                         double nx, double nz,
                                         double other_vx, double other_vz,
                                         void *ctx);
void mission_set_vehicle_contact_host(MissionVehicleRadiusFn radius,
                                      MissionVehicleHeightFn height,
                                      MissionVehicleSeparateFn separate,
                                      void *ctx);

/* Nonzero when a scene object is owned by a live mission vehicle. The browser
 * uses this after mission_attach to exclude vehicles from the immutable
 * scenery-collider snapshot. */
int mission_scene_object_is_vehicle(int scene_obj);

/* Nonzero only for the ODEF scene object consumed by the live physical
 * player car. AI vehicles keep their scene object as their live render body. */
int mission_scene_object_is_player_vehicle(int scene_obj);

/*
 * Read-only AI vehicle spawn audit. One row is recorded at each load-time
 * ODEF placement, FSM teleport/teleportOffset, and arena melee placement.
 * authored_y is the mission/FSM value before placement policy; terrain_y is
 * the terrain sample at final X/Z; required_y is the player's exact
 * terrain/drivable-surface + ride-height probe. final_y is the committed
 * model-origin height: generic rows retain the prior authored-height floor,
 * while P02's decoded Clown race seed commits required_y exactly. The bounded
 * stream resets on mission unload/load.
 */
enum {
    MISSION_VEHICLE_SPAWN_ODEF = 1,
    MISSION_VEHICLE_SPAWN_FSM_TELEPORT = 2,
    MISSION_VEHICLE_SPAWN_MELEE = 3
};
typedef struct {
    uint64_t tick;
    int ent;
    int scene_obj;
    int source;
    double x, z;
    double authored_y;
    double terrain_y;
    double required_y;
    double final_y;
} MissionVehicleSpawn;
int mission_vehicle_spawn_count(void);
int mission_vehicle_spawn(int index, MissionVehicleSpawn *out);
int mission_vehicle_spawn_overflow(void);

/* Vehicle pairs, and physical-player/AI pairs, separated during the most
 * recent mission tick. */
uint32_t mission_vehicle_contacts(void);
uint32_t mission_player_vehicle_contacts(void);

/* Cumulative post-mover world constraints for one mission entity. These are
 * observation only: the mission-owned collision stage remains authoritative.
 * Counts reset on mission load/unload. */
typedef struct {
    uint32_t static_contacts;
    uint32_t terrain_refusals;
    uint32_t pushouts;
} MissionWorldContactStats;

int mission_world_contact_stats(int ent, MissionWorldContactStats *out);

/*
 * Read-only contacts for a host-side radar. relation is 2 for the player,
 * 1 for a friendly team member, -1 for a hostile car, and 0 for neutral
 * or unresolved entities. Positions are current world XZ meters; alive and
 * hidden are the same combat/FSM state used by targeting.
 */
typedef struct {
    double x, z;
    int relation;
    int alive;
    int hidden;
} MissionContact;

int mission_contact_count(void);
int mission_contact(int index, MissionContact *out);

/*
 * mission_unload(void)
 *   Free all mission state (FSM machines/image, entity table, radio
 *   queue) and unload scene+terrain. Prints the per-action call ledger
 *   (semantic vs logged-todo) to stdout — that table is the M7 coverage
 *   ledger input. Idempotent.
 */
void mission_unload(void);

/* Nonzero when a mission is loaded (with or without an FSM). */
int mission_is_loaded(void);

/*
 * mission_path_nodes(path_index, out, max_nodes)
 *   Copy the FSM image's path `path_index` (the same table `goto`/`race`
 *   index) into `out` as max_nodes*3 floats, x/y/z per node. Returns the node
 *   count written, or 0 when no FSM is loaded or the index is out of range.
 *
 *   Read-only and additive. It exists so tests can measure an agent against
 *   the exact FSM route it was given. Dynamic vehicle/world contact may move
 *   the live body away from that route, but D-A10 leaves the owned waypoint
 *   copy unchanged.
 */
int mission_path_nodes(int path_index, float *out, int max_nodes);

#endif /* MISSION_H */
