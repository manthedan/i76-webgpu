#ifndef AI_H
#define AI_H

#include <stdint.h>
#include "engine/car.h"

/*
 * ai.h — M7 story-motion brain + Stage-3 scoped physical authority
 *
 * The mission FSM issues movement verbs (goto/follow/race/teleport/sit).
 * The decoded route/combat brain emits normalized vehicle controls at 20 Hz.
 * Stage 3 keeps route-owned mission cars kinematic, but promotes explicitly
 * synthetic/melee hosts and combat-owned FOLLOW hosts to their car.c context.
 * The same decoded command seam drives either actuator; only the selected owner
 * may publish pose or receive contact/shove velocity.
 *
 * Semantics sources:
 *  - fsm.md §4.2 action table (Open76 FSMActionDelegator.cs): goto
 *    (ent,path,speed), follow (ent,target,unk,unk,xOff,speed), isArrived
 *    (ent) "AR=1 once when car finished its path (flag self-clears)",
 *    isAtFollow (ent) "AR=1 iff within follow distance (10 m,
 *    CarAI.cs:13)", teleport (ent,path,speed,height) "move entity to path
 *    node 0, set speed, start path-following", sit (ent) "clears AI path".
 *  - P01.MSN bytecode (tools/ai_probe.c evidence): machines re-issue the
 *    SAME goto/race every poll tick while en route, so re-issuing an
 *    active path goal must not restart it (D-A5) — but the speed update
 *    must land (m4 flips jade's race 13<->23 m/s by convoy proximity).
 *    A re-issue after isArrived fired starts a fresh lap.
 *
 * Units DECISIONS (the binary's speed units are only partly resolved):
 *  D-A1 Speed arguments are treated as raw m/s. P01 uses 8 (parking leg),
 *      11 (user auto-drive), 13/23 (jade's race legs, a building
 *      progression), 15 (intro teleport), 18/19 (tanker cruise), 20
 *      (enemy chase), 22 (enemy nav hops), 35 (user exit dash) — all
 *      plausible m/s (8 m/s parking, 35 m/s ≈ 78 mph); the mph reading
 *      would make jade's race a 13 mph crawl. The native P01 tanker tape
 *      also exposes a vehicle-specific 12.52198 m/s target-speed field,
 *      so exact steady-state limits remain unresolved until non-player
 *      vehicle physics/config are consumed. Positive commands stay clamped
 *      to [AI_SPEED_MIN, AI_SPEED_MAX] so garbage cells cannot fling cars.
 *      Exact zero remains a stationary staging path: P01 enemy4 teleports
 *      onto nav6 at speed 0, then its explicit attack verb owns the
 *      transition from staging to pursuit.
 *  D-A2 Arrival = within AI_ARRIVE_DIST (5 m) of the final waypoint, AND
 *      only once the route was traversed: the authored target index must
 *      have advanced to the final node (every intermediate node reached
 *      within its advance radius). The bare distance bubble let a looped
 *      race path re-fire arrival every tick after the first crossing —
 *      the arrival snap leaves the agent standing on the final node, so
 *      each D-A5 fresh-lap re-issue arrived again instantly (P02's clown
 *      "winning" laps 2-4 in three ticks). Progress-gating matches the
 *      original's shape: its route tracker (FUN_00406510) reaches the
 *      destination by advancing through the route's nodes. Single-node
 *      paths keep the pure bubble.
 *  D-A6 Motion is 2D XZ; an agent's raw y stays whatever mission.c registered
 *      (bridge distances remain 2D XZ). Mission presentation follows terrain
 *      with that clearance; AI vehicle registration/teleport first enforces
 *      car.c's same settled terrain/drivable-surface + ride-height minimum.
 *  D-A7 follow's xOff is centi-units (×0.01 m), the camera/offset
 *      convention of fsm.md §4.2. follow speed <= 0 (P01 m1 passes -50,
 *      semantics UNKNOWN) falls back to AI_FOLLOW_SPEED.
 *  D-A8 race(ent, path, speed, rival) uses the goto mover, but fresh
 *      entry selects the nearest/forward node from the current pose
 *      (race-route-entry.md); active reissues retain their cursor. The
 *      active rival marks the path as an authored race. combat.c uses that
 *      distinction to suspend autonomous fallback fire until the race ends;
 *      an explicit attack verb still owns its normal combat behavior. Once a
 *      staged GOTO completes, explicit attack may replace it with FOLLOW even
 *      if the route's arrival pulse remains pending (P01 enemy5 never polls
 *      that pulse); autonomous fallback still yields so P02 can consume laps.
 *  D-A9 follow never sets the arrived pulse (isAtFollow is a level
 *      predicate); sit clears the goal AND any pending pulse.
 *  D-A10 goto/race begin on an owned copy of the authored FSM polyline.
 *      A former approximation displaced every multi-node path by an
 *      oracle-fitted 22.5–69 m envelope to make P01's nav5 gate fire.
 *      Although it matched aggregate off-path measurements, it changed every
 *      route and distributed the deviation where the original did not. It is
 *      superseded by physical vehicle/world contact in D-A11/D-A12.
 *  D-A11 Dynamic vehicle contact immediately displaces the actor and adds an
 *      equal-mass, zero-restitution normal impulse because neither the FSM nor
 *      current vehicle records expose mass. Velocity decays at 0.15/s and is
 *      capped at AI_SPEED_MAX. While combat's under-fire weave is active on a
 *      GOTO, D-A17's marked 0.5 m deadband keeps the query-validated local plan
 *      through sub-metre noise; a material correction still replans. Other
 *      correction behavior is unchanged.
 *  D-A12 World contact is a MISSION-owned stage (mission.c
 *      ai_world_contacts_tick) run after ai_tick and before the scene
 *      write-back; the mover itself stays kinematic and world-blind.
 *      Static scenery is a per-part oriented-box table built ONCE per
 *      load/attach (scene.c D11 policy: no drive-on surfaces, no live
 *      vehicle hulls, no degenerate boxes — the same classification the
 *      host's player-collider build applies). The tick's move is swept
 *      against the hull-expanded boxes so no 20 Hz step tunnels through
 *      a thin wall; contact truncates the move and slides the remainder
 *      along the contacted face, and a minimum-translation push-out
 *      ejects hulls a dynamic shove embedded after the stage ran.
 *      World and vehicle corrections never touch goal, waypoint, speed, or
 *      arrival state. D-A17 normally invalidates immediately; an under-fire
 *      GOTO retains its plan below the marked 0.5 m correction deadband.
 *      Ground traversal is limited per tick in BOTH directions, from the
 *      original's own vehicle constants. A rise larger than
 *      AI_WORLD_MAX_STEP — CAR_FOLLOW_MAX (20 m/s, the largest vertical
 *      rate the original's kinematic ground contact can follow before
 *      its wheel raycasts lose contact, ghidra-physics.md §Q10
 *      FUN_0041ecb0) times AI_TICK_DT — is a cliff face, not a slope:
 *      the move is refused. A drop larger than AI_WORLD_MAX_DROP — the
 *      fall that reaches the original's hard-landing impact threshold
 *      (7.65 m/s, FACT @0x4c1d58) under its own gravity (v²/2g = 2.99
 *      m) — is more descent than one tick can honestly land, and is
 *      likewise refused. Smaller drops traverse with exact ground
 *      contact: P01's own convoy route steps down 2.35 m in one tick on
 *      the wp8 leg and the native convoy rides it, so any rule that
 *      refuses route-grade drops measurably breaks authored routes.
 *      Both limits bind only GROUND-CONTACTED movers (hull bottom within
 *      one max-step of the terrain); an airborne mover whose authored
 *      clearance carries its hull above the terrain overflies the
 *      discontinuity, exactly as it overflies static boxes through the
 *      car.h D19 y-span test. The planner query applies those same
 *      per-tick limits to slices no longer than one AI_SPEED_MAX step
 *      (AI_SPEED_MAX * AI_TICK_DT), not to the raw 10 m A* edge: a
 *      driveable authored slope is not a cliff. True discontinuities
 *      (several metres over one terrain cell) stay occupied. The native far-mover cliff RESPONSE is
 *      unreversed (spec Q11: FUN_00429770/FUN_004263a0), so the
 *      conservative reading — refuse the move, never snap Y across the
 *      discontinuity — stands in for it. Teleports are authored
 *      placement and exempt (raw mover Y changes only there,
 *      mission_ent_ground_pose's own convention). H-UAT-068b adds one
 *      bounded race-owner exception: while an authored race is active, its
 *      course ignores the port's coarse 10 m cliff/static-OBB substitutes;
 *      per-tick terrain seating and dynamic vehicle contact remain active.
 *  D-A13 goto/race route each authored leg through the native 10 m-grid
 *      A* instead of driving the raw polyline. CONFIRMED Nitro mechanics
 *      (FUN_0040ff30 search, FUN_0040f720 edge costs; docs/specs/re/
 *      phase-a-ai-driver.md §4.2-3): nearest-10 start quantization with
 *      strict remainder>5 rounding; unquantized raw target; fixed
 *      8-neighbor order; no diagonal corner rejection; seed g=f=0 with
 *      its expansion free, then at most 100 main-loop expansions;
 *      1000-node pool; child g = parent g + destination edge cost;
 *      octile heuristic max(dx,dz)+(√2-1)·min(dx,dz); <11 m (121.0
 *      squared) goal region; open ordering f then z*100000+x; strict
 *      lower-g replacement only; destination surface costs 0.5 for
 *      classes 2/3/4/6/7 else 15.0, plus 1e6 only when avoid is armed
 *      and the destination probes blocked; √2 diagonal multiplier.
 *      The surface/blocked/occupied sample comes from the ai_tick nav
 *      query; AiNavSample.occupied is a PORT SUBSTITUTE for the native
 *      predictive vehicle probes (the unported FUN_0040a3d0 whisker
 *      family) and is always impassable except while D-A12's active authored
 *      race owns the course. Reconstructed start-to-goal
 *      grid points plus the raw target feed the D-A14 smoother. D-A17
 *      consumes index 0 of the reconstruction — the quantized A* search
 *      seed and smoother anchor — before the D-A15 bounded-steering
 *      kinematic actuator runs. The actuator's first target is therefore
 *      the first forward route point, and an off-grid or displaced start
 *      never turns back to chase its seed. The raw authored wp
 *      advances only when the planned leg reaches the <11 m goal region,
 *      and the final authored
 *      point still arrives on the 5 m D-A2 contract. Any external
 *      position/contact corrections normally invalidate the planned leg. An
 *      under-fire GOTO retains corrections below D-A17's marked 0.5 m
 *      deadband; larger corrections invalidate it. A failed search
 *      retains a prior valid plan; with none the agent stays stopped and
 *      retries — never a raw straight-line fallback. The optional ninth
 *      RSEG-derived neighbor and the marker stage remain unresolved RE
 *      and are deliberately absent from this slice; the smoother is
 *      ported in bounded form as D-A14.
 *  D-A14 Route smoothing (PORT APPROXIMATION — no native parity claim).
 *      The native smoother's SHAPE is CONFIRMED (FUN_0040f090: corner
 *      classification, an A* shortcut at sharp corners, a terrain-validated
 *      probe march; phase-a-ai-driver.md §4.1) but its exact construction
 *      is unresolved RE, so this is a conservative local string-pull:
 *      candidates span at most four adjacent grid edges, allowing a short
 *      staircase to collapse without replacing a road-following run by a
 *      whole-stage cross-country chord. Each candidate is sampled through
 *      AiNavQueryFn in <=10 m pieces, rejected on query failure, on an
 *      occupied piece, or on a blocked piece while avoid is armed, and
 *      accepted only when its sampled cost does not exceed the replaced
 *      grid edges (which keep their confirmed per-edge pricing; chord pieces
 *      price length-proportionally at the confirmed destination-surface
 *      rates — the native model has no chord price, so this rule is the
 *      port's). The raw final authored target is never removed.
 *  D-A15 Heading and steering (PORT APPROXIMATION — no native parity
 *      claim). A plain turn-rate limit on the old raw-waypoint mover is a
 *      measured NEGATIVE (docs/specs/m4/divergence-p01-nav5.md: it matched
 *      neither the native track nor the nav5 gate and regressed the P01
 *      win path), so this is NOT that experiment: each agent carries a
 *      persistent finite unit heading (car.h convention: forward =
 *      (-sin h, 0, cos h)); steering turns it toward the desired
 *      direction at omega = min(AI_LAT_ACCEL * steer_gain / v,
 *      AI_TURN_MAX * steer_gain) — the lateral-acceleration budget form
 *      the negative result isolated. The recovered skill tables feed the
 *      actuator: steer_gain scales the lateral/low-speed turn budgets.
 *      throttle_gain shapes longitudinal response toward the authored speed
 *      command (not a top-speed multiplier). D-A16 bounds the captured
 *      skill-1 path; other skill rows retain D-A15's prior response until
 *      native tapes establish their launch dynamics. Positive forward
 *      alignment scales speed by max(cos(error), 0)^2, brake-steering in place
 *      when the target is side-on or behind. Skill index 1 therefore responds
 *      differently from constructor-equivalent index 5 instead of remaining
 *      diagnostics.
 *      D-A17 correction: planner progress consumption is PLANNER-SCALE
 *      for reconstructed route points: a plan point is progress geometry,
 *      not a parking target.
 *      Points sit on the 10 m planner grid while the bounded turn at
 *      cruise speed needs a radius of tens of meters, so the pre-repair
 *      2 m/5 m capture let the mover pass a point it could not regain
 *      and circle it for a near-full revolution (the P01 wp4 tanker
 *      loop). Reconstructed grid points consume inside the confirmed <11 m
 *      planner goal region, or once a point that was previously ahead falls
 *      behind: the progress it marked is already made and a passed point
 *      never commands a turnaround. During an active authored race, a fresh
 *      behind-starting point first receives a real turn and a raw waypoint
 *      advances only through its <11 m contract; other goal families retain
 *      the established D-A17 rule. The authored FINAL point keeps the tight
 *      AI_WAYPT_DIST/AI_PASS_DIST capture under the 5 m D-A2 arrival
 *      contract, and an unreached final goal is never skipped. The
 *      heading seeds from the authored object yaw
 *      (ai_seed_heading) when the mission has one, otherwise from the
 *      first desired direction; re-issuing an active goal never resets
 *      it, and a same-target follow re-issue still lands its speed/xoff
 *      update. ai_get_heading exposes it for presentation (scene facing).
 *  D-A16 Longitudinal response (PORT APPROXIMATION, native launch
 *      calibrated). The canonical Windows 11/QEMU P01 tape
 *      `phasea-ai-cadence-v3.csv` samples both live tanker vehicles with a
 *      200 Hz request (about 50 Hz effective, measured from `wall_ms`):
 *      neither jumps to its command on the first observable update; both
 *      ramp from rest to about 4.6–4.8 m/s by game-time 1.94 s. For that
 *      captured skill-1 state, limit the command filter to AI_LONG_ACCEL per
 *      second (2.5 m/s²). Preserve D-A15's prior response for other skill rows
 *      until equivalent native tapes exist. This reproduces P01's first-two-
 *      second envelope without pretending the port's kinematic mover is native
 *      physics.
 *      `teleport` is the exception: its confirmed contract explicitly sets
 *      speed, so it seeds drive_speed immediately before path following.
 *  D-A22 Planner-assisted FOLLOW (PORT DECISION — the native predictive
 *      whisker family that steers Nitro's chasers around terrain is
 *      unported RE, so a bounded port-owned recovery stands in for it).
 *      A direct chase whose straight line to the target crosses ground the
 *      D-A12 stage refuses wedges terminally: FOLLOW had no planner, the
 *      stage refuses the whole move every tick, and neither party can ever
 *      reach the other (P01 e1/e2 chasers once the D24c climb limit stops
 *      the player from simply scaling the cliff to them). So: when a
 *      FOLLOW agent makes no progress toward its target for AI_STUNT_STALL
 *      ticks (the D-A21 no-progress interval, squared-meter epsilon and
 *      all) AND the straight chase line contains an impassable piece at
 *      the planner sample pitch (scan bounded to AI_NAV_STAGE, the
 *      planner's own staging radius), the chase routes through the same
 *      bounded D-A13 A* (and D-A14 smoother) that goto legs use, replanning
 *      when the plan is consumed, invalidated (D-A17 corrections apply
 *      exactly as in GOTO), or the live target leaves the confirmed <11 m
 *      goal region of the position the plan was searched toward. Every
 *      AI_STUNT_STALL ticks the line is re-tested and a CLEAR line resumes
 *      direct pursuit (with its D-A7 lateral offset; the assist drives at
 *      the target's raw position). Reaching AI_FOLLOW_DIST drops the
 *      assist. No new constants, no randomness; a chase whose line never
 *      refuses behaves bit-identically to the pre-D-A22 mover. FOLLOW
 *      still never drives D-A21 stunt legs — the assist is a planner, not
 *      a licence to cross refused ground.
 *  D-A23 Class-9 (helicopter) movers follow the authored polyline and
 *      never enter the 10 m ground A*. Nitro's setHeliHeight/FUN_004182b0
 *      class-gates helicopters; the ground planner is the reason P19's
 *      escape Huey left escape1, wandered west, and froze 600 m off the
 *      airfield. Ground cars, races, and D-A21 stunt legs are unchanged.
 *  D-A24 Combat-follow whisker fan (PORT APPROXIMATION of FUN_00404b50).
 *      Native combat ticks (FUN_0040aa20) call the fan when the current-
 *      control probe FUN_004046e0 returns a hit and the clearance field
 *      9d28 >= 1. The fan sweeps steer ±0.25 to the FUN_004284a0 unit
 *      clamp (step 0.25, score 100 − probe) via a one-step physics
 *      what-if. The port has no NPC physics integrator, so a combat
 *      FOLLOW (combat_seek or follow_hold_dist > AI_FOLLOW_DIST) whose 15 m look-ahead
 *      (_DAT_004c15c4) along the desired heading is occupied — or
 *      blocked while avoid is armed — keeps turning by the smallest
 *      ±0.25..±1.0 steer that clears that look-ahead (native holds e0
 *      until the next fan). Scripted 10 m follow, goto/race, and D-A22
 *      remain the owners of long obstacles. The native reaction timer
 *      9d34 only selects the fan's search pattern (phase-a-ai-driver.md
 *      §2.2 / FUN_00404b50); it is not a re-aim hold and is not
 *      consumed here. Open-ground combat (forward probe clear) is
 *      bit-identical to the pre-D-A24 mover.
 *  D-A25 FUN_004152e0 init-hub picker. attack() arms behavior 0, whose
 *      tick is null; the picker draws dests 1,3,10,6,9,8 from the dumped
 *      per-agg rows at DAT_004c8138. Dests 1/3 (FUN_00416680) and dest 10
 *      (FUN_004171d0) require the unported node-occupancy probe
 *      FUN_00416da0 — those dests fail closed. Dest 6/8 run FUN_00408ac0
 *      modes 1/2 (DAT_004c7138 byte table; intercept cases 4/5/6/14 steer
 *      at the target). Dest 9 runs FUN_00409320 (offset inside 70 m, then
 *      face; throttle only while |heading err| >= 0.1). Dest 8's only
 *      engage condition is the always-1 stub, so dest 15 (FUN_00409850)
 *      is the native fallback and is otherwise unreachable. FUN_0040aa20
 *      is dest 17 (recovery), not the attack hub. Same-target re-issue
 *      keeps the dest; sit / new follow clears it. termRock (12) is not
 *      an init dest.
 *  D-A26 Dest 6/8/9 transition overlay (FUN_00415490 after-tick walk).
 *      Native first-match order on dest 6: 0b2a0→28, 0b320→29,
 *      0ad20→20, 099f0→16, 0a480→17, 09ee0→18, 0c240→7. Consumed:
 *      FUN_0040ad20→20 when XZ range > 180 m (vehicle), > 200 m
 *      (non-vehicle), > 720 m (heli), or |dy| > 30 m; dest 20 is
 *      live-target close + D-A22 (native termTF is a snapshot
 *      route-follow). FUN_0040a480 / FUN_0040a3d0→17 when avoid is
 *      armed and the 15 m current-heading look-ahead is occupied
 *      (FUN_004046e0 unported). Slot/jam/swerve/steer-stall dests
 *      stay unconsumed. The init dest is not overwritten; the
 *      overlay is per-tick so dest 9 resumes inside 180 m.
 *  D-A27 Dest 6/8/9 timeUp pursuit (FUN_004098a0). Engage writes
 *      a82c = now + hold (dest 6/5/13: 20+rand%15; dest 8/14:
 *      10+rand%10; dest 9: 7+rand%10; dest 4: 7). When the clock
 *      passes a82c the dest re-picks from the dumped pursuit-1
 *      dest/weight rows. Dest 12 is not a member. Dests 1/3/10/11
 *      still fail closed. Dest 4/5/13/14 run FUN_00408ac0 modes
 *      0/3/1/4. Pursuit 0 (frame-counter) and pursuit 2 (close
 *      non-vehicle) stay unconsumed.
 *
 * Lifetime: ai_reset() clears all agents; mission.c re-registers entities
 * on every FSM load/attach. Paths are COPIED on goal assignment so agents
 * never alias the FSM image.
 *
 * Build note: ai.c is compiled by textual inclusion from mission.c (the
 * sole consumer this milestone). web/build.sh and the native probe build
 * lines list mission.c only and stay valid unchanged. DECISION — revisit
 * when build-file ownership frees up; do NOT add ai.c to build lists
 * while mission.c includes it (duplicate symbols).
 */

/* Tunables (DECISION-marked above). */
#define AI_MAX_AGENTS    64     /* FSM entity tables are <= 10 in P01      */
#define AI_SPEED_MIN     0.5    /* m/s clamp floor (D-A1)                  */
#define AI_SPEED_MAX     45.0   /* m/s clamp ceiling (D-A1)                */
#define AI_ARRIVE_DIST   5.0    /* m, final-waypoint arrival radius (D-A2) */
#define AI_WAYPT_DIST   2.0    /* m, capture radius on the authored     */
                                /* final point only (D-A15); planner     */
                                /* points consume at the <11 m D-A13     */
                                /* goal region or once passed            */
#define AI_FOLLOW_DIST   10.0   /* m, isAtFollow radius (FACT CarAI.cs:13) */
#define AI_FOLLOW_SPEED  12.5   /* m/s: P01 native intro mean (D-A7)       */
#define AI_TICK_DT       0.05   /* s per tick — the 20 Hz fixed step       */
#define AI_CONTACT_DRAG  0.15   /* 1/s exponential contact-velocity decay */
#define AI_WORLD_MAX_STEP 1.0   /* m/tick climb limit (D-A12):           */
                                /* CAR_FOLLOW_MAX 20 m/s * AI_TICK_DT     */
#define AI_WORLD_MAX_DROP 3.0   /* m/tick descent limit (D-A12): a fall */
                                /* reaching the 7.65 m/s hard landing    */
                                /* FACT @0x4c1d58 covers v^2/2g = 2.99 m */
#define AI_LONG_ACCEL   2.5    /* m/s^2 launch/brake response (D-A16)    */
#define AI_LAT_ACCEL     6.0    /* m/s^2 lateral budget (D-A15); the     */
                                /* negative-result sweep's best-track    */
                                /* family was 4-6, so use its upper bound */
#define AI_TURN_MAX      2.0    /* rad/s low-speed turn clamp (D-A15)    */
#define AI_PASS_DIST     5.0    /* m, pass-through capture radius on the */
                                /* authored final point (D-A15) — half   */
                                /* the nav grid; intermediate planner    */
                                /* points pass through at any range once */
                                /* behind the heading                    */



/* Goal kinds (ai_goal() return). */
enum {
    AI_GOAL_NONE = 0,   /* parked (initial state, post-sit, post-arrival) */
    AI_GOAL_GOTO,       /* path-following (also race — D-A8)              */
    AI_GOAL_FOLLOW      /* chasing another entity                         */
};

/* H-UAT-078c driver-output seam. These are normalized vehicle controls, not
 * pose deltas. `reverse` is desired direction state (the physical adapter
 * converts changes to CarInput's edge), while e_brake is a held level.
 * target_speed is signed m/s; arrival_intent is brain progress intent, never
 * permission for a physical pose snap. */
typedef struct {
    double steer;          /* -1 right .. +1 left (FUN_00405c80)        */
    double throttle;       /* 0..1 (FUN_004057c0 accelerate primitive)  */
    double brake;          /* 0..1 (FUN_004057c0/00406060 brake)        */
    int    reverse;        /* desired FUN_00466570 drive/reverse state  */
    int    e_brake;        /* held emergency-brake level                */
    double target_speed;   /* signed driver request, m/s                */
    int    arrival_intent; /* final-route/follow progress intent        */
    int    valid;
} AiDriveCommand;

struct CarCollider;

typedef struct {
    int      active;
    uint64_t steps;
    uint64_t command_trace_hash;
    uint64_t state_trace_hash;
    uint64_t last_state_hash;
} AiShadowTelemetry;

/* Nav sample filled by the ai_tick query callback. Planner calls are one
 * 10 m grid edge; smoother calls are <=10 m chord pieces. surface_class is
 * the native 3-bit terrain surface index (FUN_00497790; 2/3/4/6/7 are the
 * cheap 0.5-cost classes, everything else 15.0). blocked is the terrain tile
 * blocked bit (FUN_004976d0): it adds the native 1e6 cost only when the
 * agent's avoid flag is armed. occupied is a PORT SUBSTITUTE for Nitro's
 * not-yet-ported predictive vehicle probes — it makes the segment always
 * impassable and has no native cost analog. */
typedef struct {
    unsigned surface_class;
    int      blocked;
    int      occupied;
    int      cliff;     /* D-A21: occupied BECAUSE of the ground-step
                         * (cliff) rule, not a vehicle/static probe —
                         * only these pieces qualify a teleport-set goto
                         * leg as a driveable authored stunt */
} AiNavSample;

/* Nav query callback. Fills *out for one traversal segment from
 * (from_x,from_z) to (to_x,to_z) and returns 0 on success. Segments are
 * either a 10 m neighbor edge of the D-A13 planner or a <=10 m piece of a
 * D-A14 smoother chord. A nonzero return (query failure) marks the segment
 * impassable. */
typedef int (*AiNavQueryFn)(int ent, double from_x, double from_z,
                            double to_x, double to_z, AiNavSample *out);

/* D-A21: 1 while `ent` is actively driving a STUNT goto leg — a
 * TELEPORT-set authored path whose current leg line contains a
 * cliff-step piece AND whose planner made no progress for AI_STUNT_STALL
 * ticks (P01 navjump: a ramp and cliff drop the planner can never route,
 * in terrain with no detour).  Such legs are driven directly; the
 * mission ground stage lets the mover DESCEND (a fall a physical car
 * would take) but still refuses climbs. */
int  ai_goto_stunt_descent(int ent);

/* Mission-facing AI director state reconstructed from Nitro's per-vehicle
 * block (docs/specs/re/phase-a-ai-driver.md §2). The steering/throttle
 * fields feed the D-A15 kinematic controller as documented above. Aim error
 * and fire probability feed combat.c's bounded shot acceptance model. */
typedef struct {
    int   agg;               /* setAgg argument minus one, 0..4           */
    float throttle_gain;     /* T_B index 0..5; populated skills 1..5     */
    float steer_gain;        /* T_A index 0..5; populated skills 1..5     */
    float aim_error;         /* T_D index 0..5; populated skills 1..5     */
    float fire_probability;  /* T_C index 0..5; populated skills 1..5     */
    int   avoid;             /* setAvoid/toggleAvoid shared field          */
    int   max_attackers;     /* setAgg resets to 1; setMaxAttackers wins   */
} AiDirectorState;

/* Clear all agents. Call on every mission load/attach/unload. */
void ai_reset(void);

/* Register entity `ent` (an FSM entity-table index) at world position
 * (x,y,z). Re-registration snaps the position and clears goals — call
 * once per entity at mission load, before any goal verbs. */
void ai_agent_init(int ent, double x, double y, double z);

/* Setup-time physical context ownership. The mission caller attaches only
 * non-player class-1 entities and supplies that actor's real VCF. Loading may
 * allocate; ai_tick/car_context_step do not. class_id!=1 fails closed, which
 * keeps helicopters, structures, gates, and other movers kinematic-only.
 * Unpromoted contexts remain no-feedback shadows in native/dev builds and are
 * dormant in production Wasm. */
int ai_shadow_attach(int ent, unsigned class_id, const char *vcf_name);
/* Pin a synthetic/melee host to physical authority. Combat FOLLOW promotion is
 * automatic and unpinned so a later authored route verb can retake ownership
 * from the current integrated pose without physicalizing Stage-4 route cars. */
int ai_promote_physical(int ent);
int ai_physical_active(int ent);
int ai_physical_live(int ent, CarLive *out);
void ai_world_velocity(int ent, double *vx, double *vz);
void ai_apply_vehicle_contact(int ent, double dx, double dz,
                              double nx, double nz,
                              double other_vx, double other_vz);
/* Borrow the mission-owned static collider snapshot and apply terrain bounds
 * to every attached shadow. A rebuild must call this before freeing/replacing
 * the old snapshot. */
void ai_shadow_set_world(const struct CarCollider *list, int n,
                         double x0, double z0, double x1, double z1);
int ai_drive_command(int ent, AiDriveCommand *out);
int ai_shadow_telemetry(int ent, AiShadowTelemetry *out);

/* D-A23: class-9 (helicopter) movers follow the authored polyline and
 * never enter the ground A*. Call after ai_agent_init when the ODEF
 * class is 9. Unknown entities are ignored. */
void ai_set_flyer(int ent, int on);

/* Binary-verified director actions. Invalid/unregistered entities or skill /
 * aggression indices return -1 without mutating state; successful writes
 * return 0. Defaults match Nitro's constructor: setSkill(5,5), agg field 4,
 * avoid 1, maxAttackers 1000. */
int ai_set_skill(int ent, int steer_throttle, int aim_fire);
int ai_set_agg(int ent, int agg);
int ai_set_avoid(int ent, int avoid);
int ai_toggle_avoid(int ent);
int ai_set_max_attackers(int ent, int max_attackers);
int ai_director_state(int ent, AiDirectorState *out);

/* Goal verbs. pts is npts*3 floats (x,y,z world meters — fsm_image_path's
 * layout); the path is copied. path_id is the FSM path-table index, used
 * only for re-issue idempotence (D-A5). */
void ai_goto(int ent, int path_id, const float *pts, int npts, double speed);
void ai_race(int ent, int path_id, const float *pts, int npts, double speed,
             int rival_ent);
void ai_teleport(int ent, int path_id, const float *pts, int npts,
                 double speed, double y_snap, double dx_m, double dz_m);
void ai_follow(int ent, int target_ent, double xoff_m, double speed);
/* Combat seek: FUN_004152e0 picks an init dest, then dest 6/8/9 (or dest
 * 17 octant chase as fallback) drive the promoted physical context. No 90 m
 * standoff. Scripted follow remains route-owned. D-A24 whiskers still apply. */
void ai_combat_chase(int ent, int target_ent, double speed);
/* FUN_00409050 / FUN_004090f0 inputs for the next dest pick. Isolated
 * movers default to arms_ready=1, dest9_ready=0. */
void ai_combat_set_arms(int ent, int any_ready, int dest9_ready);
int  ai_combat_dest(int ent);
void ai_combat_set_dest(int ent, int dest);
void ai_sit(int ent);

/* Predicates. isArrived is a self-clearing pulse: 1 is returned exactly
 * once per path completion, the next call reads 0 (FACT fsm.md §4.2).
 * Unknown/unregistered entities read 0. */
int ai_is_arrived(int ent);
int ai_at_follow(int ent);

/* Agent state for the bridge and probes. ai_get_pos returns 0 and fills
 * out[3] when the entity is registered, -1 otherwise. ai_goal is
 * AI_GOAL_* (AI_GOAL_NONE for unknown entities). ai_race_active is true
 * while a race path with a valid rival owns the agent, including its pending
 * arrival pulse until the next script slice consumes it. ai_wp is the
 * current target waypoint index. ai_arrivals is a monotone count of path
 * completions (the probe's non-destructive evidence that isArrived
 * pulses fired — reading ai_is_arrived would consume the pulse). */
int  ai_get_pos(int ent, double out[3]);
/* Keep an unowned player mover at the physical car pose so a later authored
 * goto starts from the live handoff. Pending arrival is untouched. */
void ai_sync_idle_pose(int ent, double x, double z, double heading);
void ai_set_height(int ent, double y);
/* Apply immediate contact displacement without replacing goal, waypoint,
 * speed, or arrival. D-A17 normally replans immediately; an under-fire GOTO
 * retains sub-0.5 m correction noise. Contact velocity decays per D-A11. */
void ai_translate_xz(int ent, double dx, double dz);
/* Commit a static-world constraint correction (D-A12). Kept distinct from
 * dynamic displacement so ownership stays auditable; it shares
 * ai_translate_xz's D-A17 invalidation/hysteresis rule. */
void ai_constrain_xz(int ent, double dx, double dz);
/* Add contact/shove world velocity at the selected owner. Kinematic hosts use
 * D-A11's decaying channel; promoted hosts receive one external velocity
 * impulse in car.c and never also update contact_v*. */
void ai_add_contact_velocity(int ent, double dvx, double dvz);
/* Read-only selected-owner velocity magnitude for deterministic shove probes. */
double ai_contact_speed(int ent);
int  ai_goal(int ent);
/* True only for an active goto/teleport issued with exact authored speed 0.
 * Explicit combat may replace this stationary staging owner; moving routes
 * and races retain their normal script ownership. */
int  ai_stationary_path(int ent);
int  ai_race_active(int ent);
int  ai_has_raced(int ent);
int  ai_wp(int ent);
int  ai_arrivals(int ent);
/* D-A17 sit park: 1 after ai_sit until any goal verb (goto/race/
 * teleport/follow) or ai_clear_sat. combat.c's D-C15 fallback never
 * arms a sat entity, and a sat entity's engagement neither chases nor
 * fires — the original's sit behavior carries no target, so parked cars
 * are weapons-safe there (P02's clown sits through the intro cutscene). */
int  ai_sat(int ent);
void ai_clear_sat(int ent);
/* Non-consuming read of the isArrived pulse (ai_is_arrived CLEARS it).
 * combat.c's chase checks this before ai_follow, because follow would
 * otherwise eat a pulse the script's next machine slice must see — the
 * eaten pulse was P02's lap counter never incrementing while the
 * engagement fallback chased on the arrival tick. */
int  ai_arrival_pending(int ent);
/* FSM path index the agent is currently following, -1 when none. A probe
 * measuring contact deviation needs this: a tanker that has moved on from
 * 'convoy' to 'spot1' is not off-course, it is on a different route, and
 * without the filter its distance to the OLD polyline dominates the maximum. */
int  ai_path_id(int ent);
/* Read-only route telemetry for probes: current authored waypoint target,
 * current planned actuator target, and planned-point cursor/count. */
int  ai_route_trace(int ent, double authored_target[2],
                    double plan_target[2], int *plan_index, int *plan_count);

/* D-A15 heading. ai_seed_heading records an authored yaw (car.h
 * convention: forward = (-sin h, 0, cos h)); call once at registration
 * when the object carries a heading — re-registration clears it. Goal
 * verbs and their re-issues never reset the heading. ai_get_heading
 * returns the persistent finite heading in radians (0 when the entity is
 * unknown or no seed/desired direction has set one yet). */
void   ai_seed_heading(int ent, double yaw);
double ai_get_heading(int ent);
/* MARKED combat-driving seam: bounded transient lateral bias on the current
 * desired heading. While this command owns a GOTO, D-A17's correction deadband
 * prevents its small contact/shove response from forcing a new A* topology. */
void   ai_set_steer_bias(int ent, double radians);
/* Oil patches reuse the mover's existing steering-authority term. MARKED
 * PORT CONVENTION: native far-car grip-loss dynamics remain undecoded. */
void   ai_apply_grip_loss(int ent, double scale, int ticks);
int    ai_grip_loss_ticks(int ent);
void   ai_set_follow_hold_distance(int ent, double meters);

/* Advance every agent one tick (dt seconds — AI_TICK_DT at the fixed
 * step). resolve(ent, out3) must return the CURRENT world position of any
 * entity (the bridge's ent_pos) so follow targets track live motion;
 * called at most once per agent per tick, before that agent moves.
 * query (may be NULL) samples nav edges for the D-A13 goto/race planner;
 * NULL yields a default cheap/passable sample (class 2, unblocked,
 * unoccupied) so direct AI probes run the same planner. */
void ai_tick(double dt, void (*resolve)(int ent, double out[3]),
             AiNavQueryFn query);

#endif /* AI_H */
