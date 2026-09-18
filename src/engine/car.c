/*
 * car.c — M4 car module (see car.h)
 *
 * Parses the car config chain (VCF -> VDF/WDF/GDF per entities.md §3) into
 * a runtime state struct and runs the vehicle model of
 * docs/specs/m4/ghidra-physics.md §6 (the nitro.exe Ghidra extraction) at
 * a fixed 20 Hz step over the native-style chassis ground probe: input
 * shaping -> 4-speed auto + quadratic-torque powertrain (§Q5) -> per-wheel
 * terrain raycast + mesh-aware kinematic chassis contact (§Q10) -> forces
 * (§Q7/§Q14) under a friction-budget steering constraint (§Q17) ->
 * kick-drift integration (§Q1) with explicit airborne branch (§Q2) ->
 * terrain-aligned attitude (§Q10) -> landing events (§Q2, bridge-visible).
 *
 * Verified against the real Nitro Pack data (vdrampg2.vcf / vdrampag.vdf /
 * wauto_4a.wdf / goilslck.gdf): VCFC payload 129 B, VDFC 77 B (ELT variant),
 * WDFC 66 B, GDFC 164 B (rev 8), WEPN = repeated 25-B chunks of
 * { i32 mount; char[13] gdf }, VGEO = numParts + 28*numParts 100-B records.
 *
 * DECISION list (UNKNOWN-tag choices; specs tag layouts CONFIRMED but the
 * meanings below had to be invented or deferred — spec §13 question numbers
 * cited where applicable):
 *
 *  D1  WDFC is parsed as observed on disk: char[20] name, 5x f32
 *      (160/100/70/30/100000 — same 100000 sentinel as the VDF LOD list,
 *      so LOD-distance reading is plausible but UNKNOWN), u32 (100),
 *      f32 (10), f32 (1.0 — the spec's "radius" candidate, but 1.0 m is
 *      not a wheel radius; WLOC y ~= 0.33 m is the plausible radius),
 *      char[13] geometry name + 1 pad byte. The spec's "4x u8
 *      (255/255/127/127)" row does not appear in any Nitro WDF checked
 *      (wauto_1a/4a). All WDF fields are stored raw and UNUSED by physics
 *      (grip/load candidates — spec Q6).
 *  D2  REPLACED BY FACT (engine-curve-consumption.md): the quadratic
 *      curve consumes compnent.cdf's authored Tpeak and executable-derived
 *      k. Native FUN_004280c0 treats curve output as an acceleration
 *      numerator: × raw-mass reciprocal, × 1/|v| above 1 m/s, then tire
 *      clamp — no gear/final-drive/wheel-radius stage. The port's SI force
 *      accumulator represents raw-mass division by multiplying the numerator
 *      by lb->kg before the existing kg division. Normal P01 is the runtime-
 *      corroborated eng02 identity even though stock vdrampg2.vcf reports
 *      ENG NUM 3. Ordinary cars follow the authored engsnd.dat ENG NUM ->
 *      zero-based ENG COMP ID mapping used by native FUN_004532f0; that row
 *      selects compnent.cdf eng01..04. Missing/malformed mapping/CDF retains
 *      the old 340/2.5e-5 fit as a marked BYO-asset fallback. Brake/suspension
 *      floats remain unconsumed.
 *  D3  dragCoefficient consumption is now FACT (§Q7, near mover
 *      FUN_004263a0, scale @0x4c1d68): F = -coeff*v^2*0.1 along heading
 *      + surface rolling resistance, deadzone below 1 m/s.
 *      collisionMultiplier is stored, unused — and §Q8 now shows the
 *      ORIGINAL never reads it either (VDF +52 -> vehicle-logic struct
 *      +0x124, written by chunk handler FUN_004b8bf0, read only by the
 *      save serializer FUN_004b7ad0). Leaving it unused is faithful;
 *      scaling anything by it would be invented behaviour.
 *  D4  RETIRED (spec §14 L4): brake no longer doubles as reverse.
 *      Reverse is the CarInput.reverse edge toggle (spec §4.1
 *      `reverse_direction`).
 *  D5  REPLACED BY FACT (§Q17, input FUN_0043ebf0 / sim FUN_00428bd0
 *      @0x00428bd0): max wheel lock = pi/4 = 45 deg (@0x4c1d3c); yaw
 *      authority = steer * min(C*v, Amax/v) — proportional to v at low
 *      speed, to 1/v at high speed (the doc's min(v*m, Fmax/v) budget
 *      form; the m/Fmax operand provenance is FPU-obscured, doc §8, so
 *      the crossover C is a DECISION) with Amax = 7.84 m/s^2 = 0.8*g
 *      lateral budget (FACT @0x4c1d98). The sim reads the steer LEVEL
 *      directly; the original's digital sqrt(held/3) ramp lives in its
 *      input layer (caller-side for us). No steer slew, no low-speed
 *      kinematic blend (the budget curve is singular-free at v -> 0).
 *  D6  REPLACED (spec §14 L4): the always-terrain-bound clamp is gone.
 *      Ground contact is now per-wheel raycast suspension (D13); the
 *      ground snap survives ONLY as the one-sided chassis-contact constraint
 *      (registered class-11/12/13 upward mesh face, else terrain; inelastic:
 *      vy is zeroed when it engages) — spec §8.3/§8.4 and
 *      docs/specs/re/drivable-structures.md.
 *  D7  Heading convention: forward = (-sin yaw, 0, cos yaw), right =
 *      (cos yaw, 0, sin yaw) — right-handed, right x up = forward
 *      (scene.c D4). yaw 0 = +z (north); "left" input increases yaw.
 *  D8  No object collision (M4-collision slice owns spec §7 step 7):
 *      coll_radius is derived from the COLP outer AABB (max horizontal
 *      extent) and exposed via car_stats for the future collision pass.
 *      Position is clamped to the terrain grid [0, 51200]^2 meters.
 *  D9  Weapons are parsed for stats only (mount, GDF name/damage/ammo/
 *      health/mass); no firing, no ammo drain — M-later milestone.
 *  D10 Mass units are INFERRED lb (spec §3.2) and are now CONSUMED by
 *      the suspension/tire models, converted at CAR_LB_TO_KG. Only
 *      ratios matter for the normalized placeholders; the conversion is
 *      a DECISION pending spec Q4 tapes.
 *  D11 Armor/chassis facets from the VCF are stored as raw ints (the
 *      tenths encoding is save-file-only CONFIRMED; live encoding
 *      UNKNOWN, entities.md §4.1). No damage model here.
 *  D12 VGEO body parts: read the first numParts 100-B records (damage
 *      state 0) per scene.c's empirical D2 (Nitro VGEO carries 28 sets,
 *      not the spec's 4 + LODs + first-person layout). The 100-B record
 *      frame layout (char[8] name, 4x vec3 right/up/forward/pos, char[8]
 *      parent, 36 B skip) is CONFIRMED (scene.c D4/D5, verified on
 *      vdrampag.vdf), so car_part_frame returns REAL frames, not the
 *      identity fallback. Parent chains are composed into model space per
 *      scene.c's rule (parents precede children; "WORLD"/null/missing
 *      parent = root). Part parent names are parsed but not exposed —
 *      the drive view draws a flat part list.
 *  D13 REPLACED BY FACT (§Q10, FUN_0041ecb0 / FUN_00423690 /
 *      FUN_00424da0): the original has NO spring-damper suspension —
 *      the model is kinematic contact: per-wheel terrain raycast
 *      (contact flags + depths; ray origins use the yaw-rotated WLOC xz,
 *      pitch/roll do not move the rays), chassis kinematically follows
 *      the ground probe height while in contact, attitude aligns to the
 *      terrain normal (lerp 0.25/tick — FACT range 0.1..0.25
 *      @0x4c1d10/0x4c1d24; clamps pitch ±pi/2, roll ±pi/4 FACT
 *      @0x4c1d28..0x4c1d3c). The one-sided ground constraint (D6) is
 *      KEPT as a backstop. Ride height / contact band are DECISIONS
 *      (the original keeps them inside the wheel-component structs).
 *  D14 REPLACED BY FACT (§Q10/§Q14, FUN_00428710): tires are
 *      velocity-clamp constraints, not slip-angle springs — lateral
 *      slip velocity is strongly damped (grip budget 0.8*g FACT
 *      @0x4c1d98) and braking opposes the contact VELOCITY vector.
 *      Longitudinal drive force is clamped to the same 0.8*g budget.
 *  D15 REPLACED BY FACT (§Q5, FUN_00453700 @0x00453700 /
 *      FUN_00453520 / FUN_004534a0): 4-speed automatic, ratios
 *      3.0/1.67/0.96/0.67 (table @0x4f9928: g2 1.67 / g3 0.96 / g4 0.67
 *      FACT; slot-0 3.0 is tagged "reverse-ish" in the doc — DECISION:
 *      assigned to gear 1, whose own table entry is 0.0/unused because
 *      gear-1 RPM short-circuits to idle), final drive 3.0. Shift
 *      windows 25/40/62/98/106 km/h (FACT constants @0x4c4764..0x4c4798)
 *      with the quoted hyperbolic kickdown curves; the kickdown clauses
 *      are bounded below the matching upshift window (DECISION — the
 *      decompiled window test has an elided guard, doc §8, and the raw
 *      quotes are self-contradictory without it). Shifts apply same
 *      frame (FACT: no shift delay). shift_up/shift_down edges force
 *      one gear and suppress the auto box for 1.5 s (manual hold kept).
 *      RPM: idle 1050 (FACT @0x4c47c0), redline 6000 (FACT @0x4c47f0),
 *      peak-torque 3500 (FACT @0x4c47f4), gear-1 idle pin, target
 *      slew 2*dt window <= 0.95 (FACT). The soft rev-limiter band is a
 *      DECISION (limiter form not visible in the decompilation).
 *  D16 RETIRED — REPLACED BY FACT (§Q14): e-brake forces the effective
 *      throttle to -1.0 (full brake) and brakes ALL wheels with a force
 *      opposing the contact-patch velocity vector. The old rear-lock +
 *      rear-lateral-cut was an Open76 invention; the doc confirms no
 *      lateral-grip cut keyed on the e-brake exists in nitro.exe.
 *  D17 Ignition is always-on cosmetic state (spec §4.1 lists a
 *      start_engine action; no input channel is mapped yet).
 *  D18 RETIRED (no axle loads exist without a spring sim). The
 *      a_long_prev field is REPURPOSED as the airborne fall-apex
 *      tracker (the original tracks fall apex at ent+0x474, §Q2); the
 *      CarLive layout is frozen by save.c's field-by-field
 *      serialization, so semantics are repurposed, never the layout.
 *  D19 Notched throttle (spec §4.1; step/persistence UNKNOWN — spec
 *      Q13): CarInput.throttle is the notch LEVEL 0..1 (1.0 = full
 *      notch); tap-to-increment behavior belongs to callers/tape
 *      scripts, not the sim.
 *  D20 REPLACED BY FACT (§Q2): g = 9.8 exactly (@0x4c1c0c gravity
 *      vector, @0x4c1d54 grip scale). Airborne = gravity-only ballistic
 *      (no drive/tire forces; wheels free-spin; attitude rates persist
 *      with damping — damping coefficient still a DECISION). Landing
 *      events (bridge-visible): hard-landing threshold 7.65 m/s vertical
 *      impact (FACT @0x4c1d58), bounce restitution 0.2 (FACT @0x4c1d50),
 *      and fall-reset depth 35 m (FACT @0x4c1d44). The host consumes hard
 *      landings for port damage/audio; original per-part damage and
 *      reset-to-road behavior remain incomplete.
 *  D21 RETIRED — FACT (§Q13, FUN_0043ebf0): the original's
 *      reverse_direction action flips the direction sign with NO speed
 *      gate and the box forces gear 1; our |v|<1 m/s engagement gate was
 *      a divergence and is removed. Reverse drive = sign-flipped gear-1
 *      force; the -6 m/s reverse taper stays a DECISION (the original's
 *      reverse speed limiting was not located).
 *  D22 VGEO first-person set: Nitro VGEO carries 28 sets of numParts
 *      100-B records (D12). The entities.md §3.2 layout — 4 damage-state
 *      sets, then 12 LOD sets, then the first-person set — puts the
 *      first-person set at index 16, and the purchaser data confirms it:
 *      set 16 is the first set whose names are interior roles (DASH,
 *      SWHL, SEAT, MIRI, RADR, GER6, CMP3/CMP6, SYS3/SYS6, WEP3/WEP6,
 *      SPC3/SPC6, GUNL/GUNR, RTC1/RTC6, REST) in all 46 drivable Nitro
 *      VDFs (the two non-car oddballs varmdilo/vxufo carry no such set
 *      and parse to zero parts). Set 16 is parsed into the interior part
 *      table with NULL-named slots compacted out; trailing-3/6 name
 *      pairs are exported unresolved (day/night vs resolution UNKNOWN).
 *      The interior-mirror part is identified by the MIRI name suffix
 *      (MIRL/MIRR = side mirrors, same set); that naming, not any
 *      third-party layout, is the mask-surface evidence.
 *  D23 WDF WGEO wheel meshes: the drive exterior list used to be body
 *      VGEO only — wheels lived in scene.c's car_add_wheels for
 *      mission-placed cars, so chase/player cars looked wheel-less.
 *      After VGEO load we append up to 6 wheels from each axle's WDF
 *      (scene.c D5: WGEO record 0 = intact right, first differing name
 *      = intact left; model frame = WLOC × WGEO). WDFC radius stays
 *      diagnostic-only. Wheel spin/steer animation is NOT applied yet
 *      (static rest pose).
 *  D24 Incline projection (H-UAT-012 slope limit): the FACT §Q2 slope
 *      split (g*sin(pitch) opposing vz in contact) is completed with its
 *      two cosine complements — the drive-force tire budget clamps
 *      against the incline NORMAL force m*g*cos(pitch), and the
 *      in-contact horizontal advance projects the along-slope speed by
 *      cos(local terrain slope along the motion direction). The advance
 *      uses the terrain gradient rather than the lerped attitude pitch
 *      because the attitude lags a tile transition by ~0.3 s, and that
 *      lag was exactly the energy leak that let momentum crest cliffs.
 *      No new constants: the 0.8*g budget and surface grip stay FACT;
 *      the projection itself is a DECISION (the original's exact incline
 *      handling in FUN_004280c0 is FPU-obscured, doc §8). Effect: max
 *      sustained grade is tan(pitch) <= 0.8*grip (authored: ~50 deg P01
 *      dirt / ~59 deg road) and a momentum climb banks roughly v^2/2g
 *      of height plus what the budgeted drive force adds — near-vertical
 *      terrain stalls the car and it slides back, instead of being
 *      crested for free by the kinematic glue.
 *  D24b Cliff face as a wall, not a ramp (H-UAT-012 slope limit, second
 *      wave — verifier falsifications at >= 35 m/s entries): D24 kept the
 *      full |v| when the terrain kinked under the car, silently ROTATING
 *      the velocity onto the new slope.  Rotation is energy-free, so any
 *      face lower than v^2/2g (168 m at flat-out speed) was still
 *      crestable, and D24's cos-projections only slowed the climb.  Three
 *      completions, all against the same FACT budget 0.8*grip*g
 *      (@0x4c1d98 x §Q15 WRLD grip):
 *      (1) IMPACT: per tick the tires can steer the velocity through at
 *          most (0.8*grip*g)*dt/|v| radians (the budget as centripetal
 *          authority).  A local slope increase beyond that is a wall
 *          strike: the along-slope speed keeps only cos(excess) — the
 *          into-face component is destroyed, exactly the inelastic
 *          normal-velocity loss of hitting a wall.  The slope is sampled
 *          SHARPLY (CAR_SLOPE_SPAN_M) so a lattice-cell kink lands in
 *          one tick and is charged once — a wide sample smears the kink
 *          across ticks and cos(a)cos(b) > cos(a+b) under-charges it.
 *      (2) LIFT: the one-sided ground constraint (steep faces lose the
 *          kinematic follow at CAR_FOLLOW_MAX) pops the car up to the
 *          surface; that lift now costs its exact potential energy
 *          (vz^2 -= 2*g*dy), closing the "altitude without kinetic
 *          energy" leak on the bypass path.
 *      (3) HOLD: above the sustainable grade tan(slope) > 0.8*grip the
 *          tire budget cannot hold the car: the brake/handbrake decel
 *          saturates at 0.8*grip*g*cos(slope) there (below it the
 *          authored CAR_BRAKE_DECEL is untouched — flat braking is
 *          bit-identical), and the at-rest vz snap only engages below
 *          the sustainable grade, so a stalled car slides back down
 *          instead of perching.  The in-contact slope force and drive
 *          clamp also switch from the ~0.3 s lerped attitude pitch to
 *          the sharp local terrain slope, removing the flat->steep
 *          transition window where drive was still un-derated.
 *      DECISIONS: the centripetal reading of the FACT budget, and
 *      CAR_SLOPE_SPAN_M.  No other new constants.
 *  D24c Heading-invariant grade + landing strike (round-2 verifier
 *      falsifications of D24b):
 *      (1) DIAGONAL: every D24b grade test was body-forward only, while
 *          the lateral tire was an unbudgeted velocity clamp fed by the
 *          lerped, ±45deg-clamped attitude roll — so heading ~65 deg off
 *          the fall line diluted the felt grade below sustainable and
 *          the kinematic follow granted the true cliff height (a 76 deg
 *          face was climbable with arrow keys in the shipped page).
 *          Now the sharp slope is sampled along BOTH body axes: the
 *          sustainable-grade test and the tire budget use the TRUE
 *          steepest grade (slope_tan/cos_true, heading-invariant), the
 *          lateral gravity feed reads the terrain slope along the right
 *          axis, and above the sustainable grade the lateral clamp
 *          shares the 0.8*grip*g*cos budget with drive (friction
 *          circle).  Below the sustainable grade nothing changes.
 *      (2) LANDING STRIKE: s_slope_valid=0 on airborne used to SKIP the
 *          wall charge on landing — and that tick is the entire
 *          defence: a 0.4 m hop before the base kink let a 45 m/s entry
 *          crest through the honest-PE LIFT exchange.  Fresh contact now
 *          strikes the landing plane: per body axis the velocity keeps
 *          only its tangential component (v*cos(slope) + vy*sin(slope)).
 *      No new constants; both rules are the D24b budget/projection
 *      applied to the axes the round-2 exploits used.
 *  D25 Passive static hold (PORT DECISION — INVENTED generalization, not a
 *      native FACT): native FUN_00428710 applies NO force at all when every
 *      contact-velocity component is below its dead-zone threshold, and its
 *      all-axes opposition form runs only in braking mode (param_4, set on
 *      brake/e-brake — ghidra-physics.md Q14); the passive no-input path
 *      opposes only the lateral wheel axis. The port previously substituted
 *      a post-gravity CAR_STOP_EPS snap for the missing static case; on a
 *      mild grade one 20 Hz gravity increment can exceed the epsilon and
 *      start irreversible creep. DECISION: a no-input car already inside
 *      that near-rest epsilon is held at zero in BOTH horizontal axes while
 *      the true grade fits the existing 0.8*grip budget — an invented
 *      generalization of the braking-mode force form to the native dead-
 *      zone regime. Steep faces still slide (over budget) and any driver
 *      force releases the hold. Replace if the native at-rest mechanism
 *      (candidate: FUN_004263a0 rolling-resistance term) is ever decoded.
 *  D26 Handbrake release yaw reset (H-UAT-033): the original Q17 wheel-
 *      constraint path has no explicit persistent yaw-rate state, but the
 *      port's first-order response does. Clear that approximation on the
 *      held->released edge so old skid yaw cannot oppose the first new steer
 *      command. Lateral velocity is deliberately retained: the authored,
 *      exaggerated handbrake skid remains instead of snapping straight.
 */
#include "car.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "engine/vfs.h"
#include "engine/terrain.h"
#include "engine/component.h"

#define CAR_PI 3.14159265358979323846

/* ----------------------------------------------------------------------- */
/* Model constants. FACT = read from the nitro.exe static image (Ghidra,     */
/* docs/specs/m4/ghidra-physics.md — § + address cited per line). DECISION = */
/* an explicit port fallback/approximation, never silently source-labelled.  */
/* ----------------------------------------------------------------------- */
#define CAR_GRAVITY       9.8   /* FACT §Q2: @0x4c1c0c and @0x4c1d54    */
#define CAR_LB_TO_KG      0.45359237  /* mass units INFERRED lb (D10)    */

/* powertrain (FACT §Q5: FUN_00453700 @0x00453700 unless noted) */
#define CAR_RPM_IDLE      1050.0  /* FACT @0x4c47c0                      */
#define CAR_RPM_REDLINE   6000.0  /* FACT @0x4c47f0                      */
#define CAR_RPM_PEAK_TQ   3500.0  /* FACT torque peak rpm @0x4c47f4      */
#define CAR_TORQUE_FALLBACK_PEAK 340.0  /* DECISION: absent-CDF fallback */
#define CAR_TORQUE_FALLBACK_K 2.5e-5    /* old calibrated fit (D2)       */
#define CAR_WHEEL_RADIUS  0.33    /* m, wheel spin only (D1: WLOC y)     */
#define CAR_REVLIM_BAND   500.0   /* DECISION soft limiter width, rpm    */
#define CAR_REVERSE_MAX   6.0     /* DECISION reverse taper, m/s (D21)   */
#define CAR_ENG_BRAKE_COEF 0.2    /* DECISION engine-brake coef; the     */
                                  /* x8.0 multiplier is FACT @0x4c1d90   */
#define CAR_SHIFT_HOLD_S  1.5     /* manual shift auto-suppress, s (D15) */

/* drag + rolling resistance (FACT §Q7: near mover FUN_004263a0) */
#define CAR_DRAG_SCALE    0.1     /* FACT: F = -coeff*v^2*0.1 @0x4c1d68  */
#define CAR_DRAG_DEADZONE 1.0     /* FACT: no aero/RR below 1 m/s        */
#define CAR_RR_COEF       0.012   /* DECISION rolling-resistance coef    */

/* service brake (form FACT §Q14: opposes the contact velocity vector;  */
/* magnitude DECISION) + passive-tire solver epsilon (D25)              */
#define CAR_BRAKE_DECEL   12.0
#define CAR_STOP_EPS      0.05

/* steering / lateral constraint (FACT §Q17: FUN_00428bd0 @0x00428bd0)  */
#define CAR_STEER_MAX     (CAR_PI / 4.0) /* FACT 45 deg lock @0x4c1d3c   */
#define CAR_LAT_AMAX      7.84    /* FACT 0.8*g budget @0x4c1d98         */
#define CAR_LAT_C         0.1225  /* DECISION: crossover at 8 m/s        */
#define CAR_YAW_RESP      8.0     /* DECISION constraint response, 1/s   */
#define CAR_VX_CLAMP      8.0     /* DECISION velocity-clamp tire, 1/s   */

/* kinematic contact + attitude (FACT §Q10) */
#define CAR_RIDE_H        CAR_MODEL_ORIGIN_H /* shared render/fire origin */
#define CAR_CONTACT_H     0.15    /* DECISION wheel raycast band, m      */
#define CAR_TERRAIN_WALL_MIN_RISE 40.0 /* MARKED H-UAT-070a mountain face */
#define CAR_TERRAIN_WALL_LOOKAHEAD 15.0 /* MARKED bounded relief scan, m  */
#define CAR_TERRAIN_WALL_HULL_SAMPLES 64 /* MARKED swept radius support */
#define CAR_TERRAIN_WALL_BODY_SAMPLES 3 /* contact band + 1 m/2 m body */
#define CAR_FOLLOW_MAX    20.0    /* DECISION max kinematic follow rate, */
                                  /* m/s — steeper than this the contact */
                                  /* is lost (the original's wheel       */
                                  /* raycasts lose contact on steep      */
                                  /* faces, §Q10 FUN_0041ecb0); WITHOUT  */
                                  /* the cap the glue imparts unbounded  */
                                  /* vertical velocity on cliff ramps    */
                                  /* (energy creation)                   */
#define CAR_SLOPE_SPAN_M  0.5     /* DECISION (D24b): half-span of the   */
                                  /* sharp local terrain-slope sample,   */
                                  /* m.  Small enough that the bilinear  */
                                  /* 5 m-lattice kink at a cliff base    */
                                  /* resolves within ONE tick of travel  */
                                  /* at driving speeds (>= 0.6 m/tick at */
                                  /* 12 m/s), so the wall impact charges */
                                  /* the full angle once instead of      */
                                  /* under-charging a smeared ramp.      */
#define CAR_ATT_LERP      0.25    /* FACT range 0.1..0.25 (@0x4c1d10/24) */
#define CAR_PITCH_MAX     (CAR_PI / 2.0)  /* FACT clamp @0x4c1d28..      */
#define CAR_ROLL_MAX      (CAR_PI / 4.0)  /* FACT clamp ..0x4c1d3c       */
#define CAR_ATT_RATE_MAX  0.8     /* DECISION liftoff rate cap, rad/s    */
#define CAR_AIR_ANG_DAMP  1.0     /* DECISION airborne ang. damping, 1/s */

/* landing events (FACT §Q2; bridge-visible counters) */
#define CAR_LAND_HARD     7.65    /* FACT impact threshold @0x4c1d58     */
#define CAR_LAND_BOUNCE   0.2     /* FACT bounce impulse @0x4c1d50       */
/* Kinematic ground-follow is a positional constraint, not an unbounded
 * impulse. Retain ramp launches, but never carry more upward velocity out of
 * that constraint than the original already classifies as a hard landing. */
#define CAR_LAUNCH_MAX    CAR_LAND_HARD /* DECISION; prevents bump catapults */
#define CAR_FALL_RESET    35.0    /* FACT fall-reset depth @0x4c1d44     */

#define CAR_WORLD_MAX_M  (TERRAIN_GRID_DIM * TERRAIN_PATCH_SIZE_M) /* 51200 */

#define CAR_MAX_WEAPONS      8
#define CAR_MAX_SPECIALS     3
#define CAR_MAX_HLOCS       16
#define CAR_MAX_WEAPON_PARTS 8
#define CAR_MAX_VLOCS        8
/* VGEO set index of the first-person part set: 4 damage-state sets + 12
 * LOD sets precede it (entities.md §3.2; purchaser-data-verified, D22). */
#define CAR_VGEO_FP_SET     16

/* ----------------------------------------------------------------------- */
/* Little-endian scalar reads (alignment-safe) — house pattern              */
/* ----------------------------------------------------------------------- */
static uint32_t rd_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }
static int32_t  rd_i32(const uint8_t *p) { int32_t  v; memcpy(&v, p, 4); return v; }
static float    rd_f32(const uint8_t *p) { float    v; memcpy(&v, p, 4); return v; }

/* Bounded string copy, always NUL-terminates. */
static void copy_str(char *dst, size_t dstsz, const char *src)
{
    size_t i = 0;
    if (dstsz == 0) return;
    while (i + 1 < dstsz && src[i]) { dst[i] = src[i]; i++; }
    dst[i] = '\0';
}
/* Fixed-width field copy (NUL-padded on disk, may fill the field). */
static void copy_field(char *dst, size_t dstsz, const uint8_t *src, size_t n)
{
    size_t i = 0;
    if (dstsz == 0) return;
    while (i + 1 < dstsz && i < n && src[i]) { dst[i] = (char)src[i]; i++; }
    dst[i] = '\0';
}
static int name_is_null(const char *n)
{
    return n[0] == '\0' || strncasecmp(n, "null", 4) == 0;
}

/* The GDF names are the only decoded discriminator that separates
 * glandmin and gceracer (both ORDF 15 / manager 2 / group 15). Keep this
 * shipped-data map exact instead of guessing from damage or display text. */
static int dropper_kind_from_gdf(const char *gdf)
{
    if (!gdf) return CAR_DEPLOY_NONE;
    if (strcasecmp(gdf, "goilslck.gdf") == 0) return CAR_DEPLOY_OIL;
    if (strcasecmp(gdf, "gfirdrop.gdf") == 0) return CAR_DEPLOY_FIRE;
    if (strcasecmp(gdf, "glandmin.gdf") == 0) return CAR_DEPLOY_MINE;
    if (strcasecmp(gdf, "gcaltrop.gdf") == 0) return CAR_DEPLOY_CALTROPS;
    if (strcasecmp(gdf, "gblox.gdf") == 0) return CAR_DEPLOY_BLOX;
    if (strcasecmp(gdf, "gceracer.gdf") == 0) return CAR_DEPLOY_ERASER;
    return CAR_DEPLOY_NONE;
}

/* ----------------------------------------------------------------------- */
/* BWD2 chunk walking (chunk = tag[4] | u32 total length | payload)         */
/* ----------------------------------------------------------------------- */
typedef struct {
    char     tag[5];
    uint32_t total;     /* total length incl. 8-byte header */
    size_t   payload;   /* offset of payload in buffer     */
    size_t   next;      /* offset of the following chunk   */
} Chunk;
static int chunk_at(const uint8_t *b, size_t len, size_t off, Chunk *c)
{
    if (off + 8 > len) return 0;
    uint32_t total = rd_u32(b + off + 4);
    if (total < 8 || (size_t)total > len - off) return 0;
    memcpy(c->tag, b + off, 4);
    c->tag[4] = '\0';
    c->total   = total;
    c->payload = off + 8;
    c->next    = off + total;
    return 1;
}
static int tag_is(const Chunk *c, const char four[4])
{
    return memcmp(c->tag, four, 4) == 0;
}

/* ----------------------------------------------------------------------- */
/* Runtime car state                                                         */
/* ----------------------------------------------------------------------- */
/* Geometry part in car-model space: name + r/u/f/position frame. */
typedef struct {
    char   name[9];
    double frame[12];
} CarPart;

typedef struct {
    uint32_t index;
    uint32_t facing;               /* 1 front, 2 rear                  */
    uint32_t mesh_type;            /* 1 top, 2 side, 3 turret, 4 drop,
                                      5 inside                         */
    double   frame[12];
} CarMount;

typedef struct {
    int      mount;                 /* VDF HLOC ordinal (from VCF WEPN) */
    char     gdf[14];               /* GDF filename                     */
    char     name[17];              /* GDFC display name                */
    int32_t  tier;                  /* GDFC +20; >=100 forces turret art */
    char     fire_sprite[14];       /* GDFC +102                        */
    char     sound[14];             /* GDFC +115                        */
    char     ordnance_model[9];     /* OGEO projectile/deployer model  */
    char     impact_ground[14];     /* ORDF target-class XDF prototypes */
    char     impact_car[14];
    char     impact_building[14];
    char     impact_structure[14];
    int32_t  family;                /* GDFC +16 native weapon family    */
    int32_t  damage;                /* GDFC +44 (per projectile)        */
    int32_t  health;                /* GDFC +48 (weapon HP)             */
    int32_t  ammo;                  /* GDFC +94 (capacity)              */
    int32_t  weapon_group;          /* GDFC +90 fire-group id           */
    float    mass;                  /* GDFC +52                         */
    float    burst_rate;            /* GDFC +74: burst reload denominator */
    float    firing_rate;           /* GDFC +78: shots/s denominator    */
    float    projectile_speed;      /* GDFC +86, legacy/presentation    */
    float    flight_speed;          /* ORDF +4, native ordnance m/s     */
    int32_t  ordnance_type;         /* ORDF +0 flight dispatcher        */
    int32_t  manager_type;          /* ORDF +12 AI decision branch      */
    uint32_t facing;
    uint32_t mesh_type;
    double   mount_frame[12];       /* HLOC, car-model space            */
    double   muzzle_frame[12];      /* HLOC o GPOF, car-model space     */
    CarPart  parts[CAR_MAX_WEAPON_PARTS];
    int      nparts;
} CarWeapon;

typedef struct {
    /* VCFC (entities.md §3.1) */
    char     variant[17];
    char     vdf_file[14];
    char     vtf_file[14];
    uint32_t engine_type, susp_type, brake_type;   /* cdf catalog ids (D2) */
    char     wdf_file[3][14];                      /* front/mid/rear       */
    uint32_t armor[4], chassis[4];                 /* F/L/R/Bk raw (D11)   */
    uint32_t left_to_add;
    int      specials[CAR_MAX_SPECIALS];
    int      nspecials;
    CarWeapon weapons[CAR_MAX_WEAPONS];
    int      nweapons;
    CarMount mounts[CAR_MAX_HLOCS];
    int      nmounts;
    /* VDFC (entities.md §3.2) */
    char     chassis_name[21];
    uint32_t veh_type, veh_size;
    float    lod[5];
    float    mass, coll_mult, drag_coeff;          /* D3/D10               */
    char     elt_file[14];
    /* COLP: 12 f32 = z/x/y x (maxOuter,maxInner,minInner,minOuter) */
    float    colp[12];
    int      have_colp;
    /* WLOC: 6 wheel slots (present + full right/up/forward/pos frame) */
    int      wheel_present[6];
    float    wheel_pos[6][3];       /* position only — physics rays     */
    double   wheel_frame[6][12];    /* full frame — render (D23)        */
    /* derived at load */
    float    wheelbase, track;      /* from present WLOC pairs        */
    float    coll_radius;           /* COLP outer max extent (D8)     */
    float    wdf_radius_raw;        /* first WDF's "radius" field (D1)*/
    float    engine_tpeak, engine_k;/* authored CDF or D2 fallback    */
    uint32_t engine_curve_id;       /* selected one-based CDF row      */
    int      engine_curve_authored;
} CarConfig;

/* VDF VLOC attachment locator (entities.md §3.2; semantics INFERRED). */
typedef struct {
    uint32_t number;
    double   frame[12];
} CarVloc;

/* Presentation/cache ownership is deliberately outside CarSimContext. These
 * allocations and attachment tables are needed only for context 0's player
 * renderer. A solver context has no mesh/cache pointer and can therefore be
 * instantiated for an NPC without duplicating or mutating presentation data. */
typedef struct {
    CarPart *parts;                 /* exterior: VGEO body + WDF wheels */
    int nparts;
    int nbody_parts;                /* VGEO body-only count (D12)       */
    int nwheel_parts;               /* appended WDF wheels (D23)       */
    CarPart *fparts;                /* VGEO first-person set (D22)      */
    int nfparts;
    CarVloc vlocs[CAR_MAX_VLOCS];
    int nvlocs;
} CarPresentationCache;

/* Mutable state used by the solver but not serialized in CarLive. Keeping it
 * explicit is load-bearing for multi-instance physics: no landing, surface,
 * collision, grip, slope, bounds, diagnostic, or derived-dynamics field may
 * select a module singleton by owner identity. */
typedef struct {
    CarStepDiag step_diag;
    int hard_landings;
    int fall_resets;
    double last_impact;
    double surf_grip;
    double surf_rr;
    double surf_impact;
    double last_impact_scale;
    double grip_loss_scale;
    int grip_loss_ticks;
    int surf_have;
    unsigned surf_class;
    double slope_prev;
    double slope_r_prev;
    double slope_sgn;
    int slope_valid;
    double mass_kg;
    const CarCollider *colliders;    /* borrowed world snapshot */
    int ncolliders;
    double bx0, bz0, bx1, bz1;
} CarTransient;

/* Stage 1 context contract (docs/specs/ai-host-physics-design.md §5): config
 * is immutable after load, live is the serialized integrator vector, and all
 * other mutable solver state is transient. Existing public car_* APIs bind
 * context 0; scoped Stage-3 AI hosts use opaque independent contexts. */
struct CarSimContext {
    CarConfig config;
    CarLive live;
    CarTransient transient;
    int loaded;
};

static CarSimContext s_player_context = {
    .transient = {
        .surf_grip = 1.0,
        .surf_rr = CAR_RR_COEF,
        .surf_impact = 1.0,
        .last_impact_scale = 1.0,
        .grip_loss_scale = 1.0,
        .bx1 = CAR_WORLD_MAX_M,
        .bz1 = CAR_WORLD_MAX_M,
    },
};
static CarPresentationCache s_player_presentation;

/* Parser/solver helpers below are shared by every explicit context. Binding
 * selects their owner only for the duration of one API operation; it carries
 * no physics value itself. Public player wrappers always bind context 0. */
static CarSimContext *s_context = &s_player_context;
static CarPresentationCache *s_presentation = &s_player_presentation;

typedef struct {
    CarSimContext *context;
    CarPresentationCache *presentation;
} CarBinding;

static CarBinding context_bind(CarSimContext *ctx,
                               CarPresentationCache *presentation)
{
    CarBinding old = { s_context, s_presentation };
    s_context = ctx;
    s_presentation = presentation;
    return old;
}

static void context_restore(CarBinding old)
{
    s_context = old.context;
    s_presentation = old.presentation;
}

static void bind_player_context(void)
{
    s_context = &s_player_context;
    s_presentation = &s_player_presentation;
}

static void context_defaults(CarSimContext *ctx)
{
    memset(ctx, 0, sizeof *ctx);
    ctx->transient.surf_grip = 1.0;
    ctx->transient.surf_rr = CAR_RR_COEF;
    ctx->transient.surf_impact = 1.0;
    ctx->transient.last_impact_scale = 1.0;
    ctx->transient.grip_loss_scale = 1.0;
    ctx->transient.bx1 = CAR_WORLD_MAX_M;
    ctx->transient.bz1 = CAR_WORLD_MAX_M;
}

#define s_cfg                 (s_context->config)
#define s_lv                  (s_context->live)
#define s_loaded              (s_context->loaded)
#define s_step_diag           (s_context->transient.step_diag)
#define s_hard_landings       (s_context->transient.hard_landings)
#define s_fall_resets         (s_context->transient.fall_resets)
#define s_last_impact         (s_context->transient.last_impact)
#define s_surf_grip           (s_context->transient.surf_grip)
#define s_surf_rr             (s_context->transient.surf_rr)
#define s_surf_impact         (s_context->transient.surf_impact)
#define s_last_impact_scale   (s_context->transient.last_impact_scale)
#define s_grip_loss_scale     (s_context->transient.grip_loss_scale)
#define s_grip_loss_ticks     (s_context->transient.grip_loss_ticks)
#define s_surf_have           (s_context->transient.surf_have)
#define s_surf_class          (s_context->transient.surf_class)
#define s_slope_prev          (s_context->transient.slope_prev)
#define s_slope_r_prev        (s_context->transient.slope_r_prev)
#define s_slope_sgn           (s_context->transient.slope_sgn)
#define s_slope_valid         (s_context->transient.slope_valid)
#define s_mass_kg             (s_context->transient.mass_kg)
#define s_colliders           (s_context->transient.colliders)
#define s_ncolliders          (s_context->transient.ncolliders)
#define s_bx0                 (s_context->transient.bx0)
#define s_bz0                 (s_context->transient.bz0)
#define s_bx1                 (s_context->transient.bx1)
#define s_bz1                 (s_context->transient.bz1)
#define s_parts               (s_presentation->parts)
#define s_nparts              (s_presentation->nparts)
#define s_nbody_parts         (s_presentation->nbody_parts)
#define s_nwheel_parts        (s_presentation->nwheel_parts)
#define s_fparts              (s_presentation->fparts)
#define s_nfparts             (s_presentation->nfparts)
#define s_vlocs               (s_presentation->vlocs)
#define s_nvlocs              (s_presentation->nvlocs)

/* H-UAT-078c Stage 2 performance instrumentation. These counters are outside
 * every solver context, diagnostic-only, and never read by simulation. */
static CarPerfCounters s_perf_counters;
static double counted_terrain_height_at(double x, double z)
{
    s_perf_counters.terrain_height_queries++;
    return terrain_height_at(x, z);
}
#define terrain_height_at counted_terrain_height_at

static void dynamics_derive(void);  /* fwd: defined with the sim below */

/* ----------------------------------------------------------------------- */
/* Config-chain parsers (flat BWD2 files; EXIT chunks appear mid-stream as  */
/* list terminators, so we scan the whole file and dispatch by tag)         */
/* ----------------------------------------------------------------------- */
/* VCFC payload layout (entities.md §3.1 table). Needs >= 93 bytes for the
 * names/types, >= 129 for the armor/chassis block. */
static int vcfc_read(const uint8_t *p, size_t n)
{
    if (n < 93) return -1;
    copy_field(s_cfg.variant, sizeof s_cfg.variant, p, 16);
    copy_field(s_cfg.vdf_file, sizeof s_cfg.vdf_file, p + 16, 13);
    copy_field(s_cfg.vtf_file, sizeof s_cfg.vtf_file, p + 29, 13);
    s_cfg.engine_type = rd_u32(p + 42);
    s_cfg.susp_type   = rd_u32(p + 46);
    s_cfg.brake_type  = rd_u32(p + 50);
    for (int i = 0; i < 3; i++)
        copy_field(s_cfg.wdf_file[i], sizeof s_cfg.wdf_file[i],
                   p + 54 + (size_t)i * 13, 13);
    if (n >= 129) {
        for (int i = 0; i < 4; i++) {
            s_cfg.armor[i]   = rd_u32(p + 93 + (size_t)i * 4);
            s_cfg.chassis[i] = rd_u32(p + 109 + (size_t)i * 4);
        }
        s_cfg.left_to_add = rd_u32(p + 125);
    }
    return 0;
}

/* VDFC: fixed header (64 B, or 77 B when the ELT name is present). */
static int vdfc_read(const uint8_t *p, size_t n)
{
    if (n < 64) return -1;
    copy_field(s_cfg.chassis_name, sizeof s_cfg.chassis_name, p, 20);
    s_cfg.veh_type   = rd_u32(p + 20);
    s_cfg.veh_size   = rd_u32(p + 24);
    for (int i = 0; i < 5; i++) s_cfg.lod[i] = rd_f32(p + 28 + (size_t)i * 4);
    s_cfg.mass       = rd_f32(p + 48);
    s_cfg.coll_mult  = rd_f32(p + 52);
    s_cfg.drag_coeff = rd_f32(p + 56);
    /* p + 60: u32 unk (observed 4) */
    if (n >= 77)
        copy_field(s_cfg.elt_file, sizeof s_cfg.elt_file, p + 64, 13);
    return 0;
}

/* COLP: 12 f32, inner + outer collision AABBs (entities.md §3.2). */
static void colp_read(const uint8_t *p, size_t n)
{
    if (n < 48) return;
    for (int i = 0; i < 12; i++) s_cfg.colp[i] = rd_f32(p + (size_t)i * 4);
    s_cfg.have_colp = 1;
    float m = 0.0f;
    for (int i = 0; i < 8; i++) {           /* z and x rows only (D8) */
        float a = fabsf(s_cfg.colp[i]);
        if (a > m) m = a;
    }
    s_cfg.coll_radius = m;
}

/* WLOC: exactly 6 records of u32 present + 4xvec3 frame + f32 unk (56 B). */
static void wloc_read(const uint8_t *p, size_t n)
{
    if (n < 6 * 56) return;
    for (int i = 0; i < 6; i++) {
        const uint8_t *r = p + (size_t)i * 56;
        s_cfg.wheel_present[i] = rd_u32(r) != 0;
        /* frame = right/up/forward/pos; keep full frame for D23 compose */
        for (int k = 0; k < 12; k++)
            s_cfg.wheel_frame[i][k] = (double)rd_f32(r + 4 + (size_t)k * 4);
        s_cfg.wheel_pos[i][0] = (float)s_cfg.wheel_frame[i][9];
        s_cfg.wheel_pos[i][1] = (float)s_cfg.wheel_frame[i][10];
        s_cfg.wheel_pos[i][2] = (float)s_cfg.wheel_frame[i][11];
    }
}

/* HLOC: weapon hardpoint in car-model space (scene.md §6.2). WEPN.mount
 * indexes this list by ordinal; the stored index is retained as evidence but
 * is not a lookup key (Open76 CacheManager's shipping behavior). */
static void hloc_read(const uint8_t *p, size_t n)
{
    if (n < 80 || s_cfg.nmounts >= CAR_MAX_HLOCS) return;
    CarMount *m = &s_cfg.mounts[s_cfg.nmounts++];
    m->index = rd_u32(p + 16);
    m->facing = rd_u32(p + 20);
    m->mesh_type = rd_u32(p + 24);
    for (int k = 0; k < 12; k++)
        m->frame[k] = (double)rd_f32(p + 28 + (size_t)k * 4);
}

/* Derive wheelbase/track from the present WLOC slots (fallback: COLP). */
static void wheels_derive(void)
{
    float zmin = 0.0f, zmax = 0.0f, xmin = 0.0f, xmax = 0.0f;
    int n = 0;
    for (int i = 0; i < 6; i++) {
        if (!s_cfg.wheel_present[i]) continue;
        float x = s_cfg.wheel_pos[i][0], z = s_cfg.wheel_pos[i][2];
        if (n == 0) { zmin = zmax = z; xmin = xmax = x; }
        if (z < zmin) zmin = z;
        if (z > zmax) zmax = z;
        if (x < xmin) xmin = x;
        if (x > xmax) xmax = x;
        n++;
    }
    if (n >= 2 && zmax - zmin > 0.1f) {
        s_cfg.wheelbase = zmax - zmin;
        s_cfg.track     = xmax - xmin;
    } else if (s_cfg.have_colp) {
        /* COLP rows: z maxOuter/minOuter at [0]/[3], x at [4]/[7] */
        s_cfg.wheelbase = s_cfg.colp[0] - s_cfg.colp[3];
        s_cfg.track     = s_cfg.colp[4] - s_cfg.colp[7];
    }
    if (s_cfg.wheelbase < 0.1f) s_cfg.wheelbase = 2.5f;  /* safe default */
    if (s_cfg.track     < 0.1f) s_cfg.track     = 1.5f;
}

/* Model = parent after child: O = P . C on r/u/f/pos column frames. */
static void frame_compose(const double P[12], const double C[12],
                          double O[12])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++)
            O[j * 3 + i] = P[i]     * C[j * 3 + 0]
                         + P[3 + i] * C[j * 3 + 1]
                         + P[6 + i] * C[j * 3 + 2];
        O[9 + i] = P[i]     * C[9]
                 + P[3 + i] * C[10]
                 + P[6 + i] * C[11]
                 + P[9 + i];
    }
}

/*
 * One VGEO part set: count 100-B records at rec — char[8] name, 4x vec3
 * frame (right/up/forward/pos), char[8] parent, 36 B skip. Parent chains
 * are composed into model space (parents precede children — scene.c rule).
 * skip_null drops NULL-named records (the first-person set pads unused
 * slots with NULLs; the damage-state-0 set keeps them for index
 * stability with the existing car_part_* API). *out_count receives the
 * stored count; NULL is returned only on allocation failure.
 */
static CarPart *partset_parse(const uint8_t *rec, int count, int skip_null,
                              int *out_count)
{
    CarPart *parts = malloc((size_t)count * sizeof *parts);
    if (!parts) { *out_count = 0; return NULL; }
    static const double ID[12] = { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 };
    int nstored = 0;
    for (int i = 0; i < count; i++) {
        const uint8_t *r = rec + (size_t)i * 100;
        char name[9];
        copy_field(name, sizeof name, r, 8);
        if (skip_null && name_is_null(name)) continue;
        CarPart *part = &parts[nstored];
        copy_str(part->name, sizeof part->name, name);
        double local[12];
        for (int k = 0; k < 12; k++)
            local[k] = (double)rd_f32(r + 8 + (size_t)k * 4);
        char parent[9];
        copy_field(parent, sizeof parent, r + 56, 8);
        const double *pm = ID;
        if (!name_is_null(parent) && strcasecmp(parent, "WORLD") != 0)
            for (int j = 0; j < nstored; j++)
                if (strcasecmp(parts[j].name, parent) == 0) {
                    pm = parts[j].frame;
                    break;
                }
        frame_compose(pm, local, part->frame);
        nstored++;
    }
    *out_count = nstored;
    return parts;
}

/*
 * VGEO (D12/D22): u32 numParts, then sets of numParts 100-B part records.
 * Set 0 is damage state 0 (D12 — Nitro VGEO carries 28 sets, not the
 * spec's 4 + LODs + first-person exact count); the set at index
 * CAR_VGEO_FP_SET (4 damage + 12 LOD sets in, per the entities.md §3.2
 * layout) is the first-person/interior set, confirmed on the purchaser
 * data by its interior role names (D22).
 */
static void vgeo_read(const uint8_t *p, size_t n)
{
    if (n < 4) return;
    int count = (int)rd_u32(p);
    if (count <= 0) return;
    int sets = (int)((n - 4) / 100) / count;
    if (sets < 1) return;
    s_parts  = partset_parse(p + 4, count, 0, &s_nparts);
    s_nbody_parts = s_nparts;
    s_nwheel_parts = 0;
    if (sets > CAR_VGEO_FP_SET)
        s_fparts = partset_parse(
            p + 4 + (size_t)CAR_VGEO_FP_SET * (size_t)count * 100,
            count, 1, &s_nfparts);
}

/* VLOC: u32 number + 4xvec3 frame (52 B payload), entities.md §3.2. */
static void vloc_read(const uint8_t *p, size_t n)
{
    if (n < 52 || s_nvlocs >= CAR_MAX_VLOCS) return;
    CarVloc *v = &s_vlocs[s_nvlocs++];
    v->number = rd_u32(p);
    for (int k = 0; k < 12; k++)
        v->frame[k] = (double)rd_f32(p + 4 + (size_t)k * 4);
}

/* WDFC (D1): char[20] name, 5x f32, u32, f32, f32 "radius", char[13] geo. */
static void wdf_read_radius(const char *name)
{
    size_t sz = 0;
    uint8_t *buf = vfs_read_file(name, &sz);
    if (!buf) return;
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        if (tag_is(&c, "WDFC") && c.total - 8 >= 56) {
            /* all fields UNKNOWN-meaning (D1); keep only the raw radius */
            s_cfg.wdf_radius_raw = rd_f32(buf + c.payload + 48);
            break;
        }
        off = c.next;
    }
    vfs_free(buf);
}

/*
 * Append one exterior part (name + model-space frame). Used for WDF wheels.
 * Returns 0 on success, -1 on OOM (leaves the list unchanged on failure).
 */
static int exterior_part_append(const char *name, const double frame[12])
{
    if (name_is_null(name)) return -1;
    CarPart *np = realloc(s_parts, (size_t)(s_nparts + 1) * sizeof *np);
    if (!np) return -1;
    s_parts = np;
    copy_str(s_parts[s_nparts].name, sizeof s_parts[s_nparts].name, name);
    memcpy(s_parts[s_nparts].frame, frame, sizeof s_parts[s_nparts].frame);
    s_nparts++;
    return 0;
}

/*
 * WDF WGEO wheel meshes at WLOC frames (D23 / scene.c D5).
 * For each of the three axle WDF names: load WGEO, pick intact R/L part
 * records, place on the two present WLOC slots of that axle (wi = axle*2
 * and axle*2+1). Side pick is WLOC local +x (right_side when x > 0).
 */
static void wheels_add_meshes(void)
{
    for (int axle = 0; axle < 3; axle++) {
        const char *wdf_name = s_cfg.wdf_file[axle];
        if (name_is_null(wdf_name)) continue;

        size_t sz = 0;
        uint8_t *buf = vfs_read_file(wdf_name, &sz);
        if (!buf) continue;

        int added = 0;
        for (size_t off = 0; off < sz; ) {
            Chunk c;
            if (!chunk_at(buf, sz, off, &c)) break;
            if (tag_is(&c, "WGEO")) {
                const uint8_t *p = buf + c.payload;
                size_t avail = c.total - 8;
                if (avail < 4 + 100) break;
                int nrec = (int)((avail - 4) / 100);
                if (nrec < 1) break;

                char rname[9], lname[9], parent[9];
                double rlocal[12], llocal[12];
                const uint8_t *rr = p + 4;
                copy_field(rname, sizeof rname, rr, 8);
                for (int k = 0; k < 12; k++)
                    rlocal[k] = (double)rd_f32(rr + 8 + (size_t)k * 4);
                copy_field(parent, sizeof parent, rr + 56, 8);
                (void)parent;

                int have_l = 0;
                for (int i = 1; i < nrec; i++) {
                    const uint8_t *lr = p + 4 + (size_t)i * 100;
                    copy_field(lname, sizeof lname, lr, 8);
                    if (strcmp(lname, rname) != 0) {
                        for (int k = 0; k < 12; k++)
                            llocal[k] =
                                (double)rd_f32(lr + 8 + (size_t)k * 4);
                        have_l = 1;
                        break;
                    }
                }
                if (!have_l) {
                    memcpy(lname, rname, sizeof lname);
                    memcpy(llocal, rlocal, sizeof llocal);
                }

                for (int wi = axle * 2; wi < axle * 2 + 2 && wi < 6; wi++) {
                    if (!s_cfg.wheel_present[wi]) continue;
                    int right_side = s_cfg.wheel_pos[wi][0] > 0.0f;
                    const char *wn = right_side ? rname : lname;
                    const double *wlocal = right_side ? rlocal : llocal;
                    if (name_is_null(wn)) continue;
                    double model[12];
                    frame_compose(s_cfg.wheel_frame[wi], wlocal, model);
                    if (exterior_part_append(wn, model) == 0) {
                        s_nwheel_parts++;
                        added++;
                    }
                }
                break;
            }
            if (tag_is(&c, "WDFC") && c.total - 8 >= 56 &&
                s_cfg.wdf_radius_raw == 0.0f)
                s_cfg.wdf_radius_raw = rd_f32(buf + c.payload + 48);
            off = c.next;
        }
        vfs_free(buf);
        (void)added;
    }
}

/*
 * GDF weapon definition and presentation geometry (entities.md §3.3).
 *
 * GPOF's four frames are ordered top/side/turret/inside; GGEO slots carry
 * the same classification in name[3] (P/S/T/I). Native handlers
 * FUN_004ba0e0/FUN_004ba390 use the HLOC class only while GDFC tier < 100;
 * tier >= 100 forces class 3 for both muzzle and geometry. The selected HLOC
 * still owns the outer transform, so every exposed frame is already in
 * car-model space. OGEO projectile meshes are not queued: combat.c's live
 * ORDF entity owns asynchronous flight/contact, while the browser renders its
 * presentation snapshot with palette primitives.
 */
static void gdf_read(CarWeapon *w)
{
    static const double ID[12] = {
        1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0
    };
    double points[4][12];
    CarPart model_parts[CAR_MAX_WEAPON_PARTS];
    int have_points = 0;
    char part_kind = '\0';
    int point_index = -1;
    size_t sz = 0;
    uint8_t *buf = vfs_read_file(w->gdf, &sz);
    if (!buf) return;

    memcpy(w->mount_frame, ID, sizeof w->mount_frame);
    if (w->mount >= 0 && w->mount < s_cfg.nmounts) {
        const CarMount *m = &s_cfg.mounts[w->mount];
        w->facing = m->facing;
        w->mesh_type = m->mesh_type;
        memcpy(w->mount_frame, m->frame, sizeof w->mount_frame);
    }
    switch (w->mesh_type) {
        case 1: part_kind = 'P'; point_index = 0; break; /* top    */
        case 2: part_kind = 'S'; point_index = 1; break; /* side   */
        case 3: part_kind = 'T'; point_index = 2; break; /* turret */
        case 5: part_kind = 'I'; point_index = 3; break; /* inside */
        default: break;                                  /* none/dropper */
    }

    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        const uint8_t *p = buf + c.payload;
        size_t n = c.total - 8;

        if (tag_is(&c, "GDFC") && n >= 128) {
            copy_field(w->name, sizeof w->name, p, 16);
            w->family           = rd_i32(p + 16);
            w->tier             = rd_i32(p + 20);
            if (w->tier >= 100) {
                part_kind = 'T';
                point_index = 2;
            }
            w->damage           = rd_i32(p + 44);
            w->health           = rd_i32(p + 48);
            w->mass             = rd_f32(p + 52);
            w->burst_rate       = rd_f32(p + 74);
            w->firing_rate      = rd_f32(p + 78);
            w->projectile_speed = rd_f32(p + 86);
            w->weapon_group     = rd_i32(p + 90);  /* fire-group id (M3E §3.3) */
            w->ammo             = rd_i32(p + 94);
            copy_field(w->fire_sprite, sizeof w->fire_sprite, p + 102, 13);
            copy_field(w->sound, sizeof w->sound, p + 115, 13);
        } else if (tag_is(&c, "ORDF") && n >= 16) {
            /* FACT fire-delivery.md §2/§3 and weapon-impact-presentation.md:
             * the 16-byte base owns flight/dispatch. A full 137-byte Nitro
             * record additionally carries four target-class XDF/WAV pairs;
             * H-UAT-079b consumes visual XDFs only (impact audio is separate). */
            w->ordnance_type = rd_i32(p);
            w->flight_speed = rd_f32(p + 4);
            w->manager_type = rd_i32(p + 12);
            if (n >= 124) {
                copy_field(w->impact_ground, sizeof w->impact_ground,
                           p + 33, 13);
                copy_field(w->impact_car, sizeof w->impact_car, p + 59, 13);
                copy_field(w->impact_building, sizeof w->impact_building,
                           p + 85, 13);
                copy_field(w->impact_structure, sizeof w->impact_structure,
                           p + 111, 13);
            }
        } else if (tag_is(&c, "GPOF") && n >= sizeof points) {
            for (int i = 0; i < 4; i++)
                for (int k = 0; k < 12; k++)
                    points[i][k] =
                        (double)rd_f32(p + (size_t)i * 48 + (size_t)k * 4);
            have_points = 1;
        } else if (tag_is(&c, "OGEO") && n >= 12) {
            /* OGEO = i32 count followed by one 100-byte part record. The
             * record name is the authored in-flight/deployed model; class-4
             * droppers use it as presentation data, never as gameplay state. */
            copy_field(w->ordnance_model, sizeof w->ordnance_model, p + 4, 8);
        } else if (tag_is(&c, "GGEO") && n >= 4 && part_kind) {
            uint32_t per_set = rd_u32(p);
            size_t slots = (size_t)per_set * 4;
            size_t available = (n - 4) / 300;
            if (slots > available) slots = available;
            for (size_t i = 0; i < slots &&
                               w->nparts < CAR_MAX_WEAPON_PARTS; i++) {
                const uint8_t *r = p + 4 + i * 300;
                char name[9], parent[9];
                copy_field(name, sizeof name, r, 8);
                if (name_is_null(name) || strlen(name) < 4 ||
                    name[3] != part_kind)
                    continue;
                copy_field(parent, sizeof parent, r + 56, 8);
                double local[12];
                for (int k = 0; k < 12; k++)
                    local[k] = (double)rd_f32(r + 8 + (size_t)k * 4);
                const double *pm = ID;
                if (!name_is_null(parent) &&
                    strcasecmp(parent, "WORLD") != 0)
                    for (int j = 0; j < w->nparts; j++)
                        if (strcasecmp(model_parts[j].name, parent) == 0) {
                            pm = model_parts[j].frame;
                            break;
                        }
                int slot = w->nparts++;
                copy_str(model_parts[slot].name, sizeof model_parts[slot].name,
                         name);
                frame_compose(pm, local, model_parts[slot].frame);
                CarPart *part = &w->parts[slot];
                copy_str(part->name, sizeof part->name, name);
                frame_compose(w->mount_frame, model_parts[slot].frame,
                              part->frame);
            }
        }
        off = c.next;
    }

    if (have_points && point_index >= 0)
        frame_compose(w->mount_frame, points[point_index], w->muzzle_frame);
    else
        memcpy(w->muzzle_frame, w->mount_frame, sizeof w->muzzle_frame);
    vfs_free(buf);
}

/* WEPN chunk: repeated { i32 mount; char[13] gdf } records (17 B each). */
static void wepn_read(const uint8_t *p, size_t n)
{
    size_t nrec = n / 17;
    for (size_t i = 0; i < nrec && s_cfg.nweapons < CAR_MAX_WEAPONS; i++) {
        CarWeapon *w = &s_cfg.weapons[s_cfg.nweapons];
        memset(w, 0, sizeof *w);
        w->mount = (int)rd_i32(p + i * 17);
        copy_field(w->gdf, sizeof w->gdf, p + i * 17 + 4, 13);
        s_cfg.nweapons++;
    }
}

/* ----------------------------------------------------------------------- */
/* car_load                                                                 */
/* ----------------------------------------------------------------------- */
static int context_load_bound(const char *vcf_name)
{
    char path[64];
    memset(&s_cfg, 0, sizeof s_cfg);
    memset(&s_lv, 0, sizeof s_lv);
    free(s_parts);
    s_parts = NULL;
    s_nparts = 0;
    s_nbody_parts = 0;
    s_nwheel_parts = 0;
    free(s_fparts);
    s_fparts = NULL;
    s_nfparts = 0;
    s_nvlocs = 0;
    s_loaded = 0;
    s_hard_landings = 0;
    s_fall_resets = 0;
    s_last_impact = 0.0;
    s_last_impact_scale = 1.0;
    s_surf_grip   = 1.0;
    s_surf_rr     = CAR_RR_COEF;
    s_surf_impact = 1.0;
    s_grip_loss_scale = 1.0;
    s_grip_loss_ticks = 0;
    s_surf_have   = 0;

    copy_str(path, sizeof path, vcf_name);
    if (!strchr(path, '.')) {
        size_t l = strlen(path);
        if (l + 4 < sizeof path) memcpy(path + l, ".vcf", 5);
    }
    size_t sz = 0;
    uint8_t *vcf = vfs_read_file(path, &sz);
    if (!vcf) return -1;

    int have_vcfc = 0;
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(vcf, sz, off, &c)) break;
        if (tag_is(&c, "VCFC")) {
            if (vcfc_read(vcf + c.payload, c.total - 8) == 0)
                have_vcfc = 1;
        } else if (tag_is(&c, "SPEC") && c.total - 8 >= 4) {
            if (s_cfg.nspecials < CAR_MAX_SPECIALS)
                s_cfg.specials[s_cfg.nspecials++] =
                    (int)rd_i32(vcf + c.payload);
        } else if (tag_is(&c, "WEPN")) {
            wepn_read(vcf + c.payload, c.total - 8);
        }
        off = c.next;
    }
    vfs_free(vcf);
    if (!have_vcfc || name_is_null(s_cfg.vdf_file)) return -1;

    /* VDF chassis (required): VDFC + COLP + WLOC + VGEO parts. */
    uint8_t *vdf = vfs_read_file(s_cfg.vdf_file, &sz);
    if (!vdf) return -1;
    int have_vdfc = 0;
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(vdf, sz, off, &c)) break;
        if (tag_is(&c, "VDFC")) {
            if (vdfc_read(vdf + c.payload, c.total - 8) == 0)
                have_vdfc = 1;
        } else if (tag_is(&c, "COLP")) {
            colp_read(vdf + c.payload, c.total - 8);
        } else if (tag_is(&c, "WLOC")) {
            wloc_read(vdf + c.payload, c.total - 8);
        } else if (tag_is(&c, "VLOC")) {
            vloc_read(vdf + c.payload, c.total - 8);
        } else if (tag_is(&c, "HLOC")) {
            hloc_read(vdf + c.payload, c.total - 8);
        } else if (tag_is(&c, "VGEO")) {
            vgeo_read(vdf + c.payload, c.total - 8);
        }
        off = c.next;
    }
    vfs_free(vdf);
    if (!have_vdfc) return -1;

    wheels_derive();
    dynamics_derive();
    s_lv.gear = 1;
    s_lv.ignition = 1;
    s_lv.engine_rpm = CAR_RPM_IDLE;
    /* FACT (engine-index-mapping.md): engsnd.dat maps the VCF's ENG NUM to
     * a zero-based ENG COMP ID; native FUN_004532f0 performs that lookup and
     * FUN_004bbb80 selects the matching CDF ENGN row. Normal P01 is the one
     * evidence-backed overlay: its observed campaign install is eng02 even
     * though the garage/default vdrampg2 VCF carries ENG NUM 3. */
    char vcf_base[64];
    const char *slash = strrchr(path, '/');
    copy_str(vcf_base, sizeof vcf_base, slash ? slash + 1 : path);
    ComponentEngineCurve curve;
    int have_curve = strcasecmp(vcf_base, "vdrampg2.vcf") == 0
        ? component_engine_curve_by_id(1u, &curve)
        : component_engine_curve(s_cfg.engine_type, &curve);
    if (have_curve == 0) {
        s_cfg.engine_curve_id = curve.component_id + 1u;
        s_cfg.engine_tpeak = curve.tpeak;
        s_cfg.engine_k = curve.k;
        s_cfg.engine_curve_authored = 1;
    } else {
        s_cfg.engine_curve_id = 0u;
        s_cfg.engine_tpeak = CAR_TORQUE_FALLBACK_PEAK;
        s_cfg.engine_k = CAR_TORQUE_FALLBACK_K;
        s_cfg.engine_curve_authored = 0;
    }
    /* Wheel definitions (optional): WDFC radius (D1, diagnostic) from the
     * first present axle file; WGEO meshes appended to the exterior part
     * list at WLOC frames (D23 — matches scene.c car_add_wheels). */
    for (int i = 0; i < 3; i++)
        if (!name_is_null(s_cfg.wdf_file[i])) {
            wdf_read_radius(s_cfg.wdf_file[i]);
            break;
        }
    wheels_add_meshes();
    /* Weapon definitions (optional): gameplay fields, hardpoint-selected
     * mounted geometry, muzzle frame, and real fire sound. */
    for (int i = 0; i < s_cfg.nweapons; i++)
        if (!name_is_null(s_cfg.weapons[i].gdf))
            gdf_read(&s_cfg.weapons[i]);

    s_loaded = 1;
    return 0;
}

static void context_unload_bound(void)
{
    memset(&s_cfg, 0, sizeof s_cfg);
    free(s_parts);
    s_parts = NULL;
    s_nparts = 0;
    s_nbody_parts = 0;
    s_nwheel_parts = 0;
    free(s_fparts);
    s_fparts = NULL;
    s_nfparts = 0;
    s_nvlocs = 0;
    s_grip_loss_scale = 1.0;
    s_grip_loss_ticks = 0;
    s_loaded = 0;
}

static double settled_y_at_bound(double x, double z)
{
    double h, normal[3];
    double terrain_y = terrain_height_at(x, z) + CAR_RIDE_H;
    (void)terrain_drivable_probe(x, terrain_y, z, &h, normal);
    return h + CAR_RIDE_H;
}

static void context_place_bound(double x, double z, double yaw)
{
    memset(&s_lv, 0, sizeof s_lv);
    s_lv.x = x;
    s_lv.z = z;
    s_lv.yaw = yaw;
    s_lv.gear = 1;
    s_lv.ignition = 1;              /* always-on (D17) */
    s_lv.engine_rpm = CAR_RPM_IDLE;
    /* Kinematic settle (D13 FACT §Q10): the origin rests CAR_RIDE_H
     * above the chassis probe; no spring compression to precompute. */
    s_lv.y = settled_y_at_bound(x, z);
    s_lv.a_long_prev = s_lv.y;      /* fall-apex tracker (D18 repurpose) */
    s_hard_landings = 0;
    s_fall_resets = 0;
    s_last_impact = 0.0;
    s_last_impact_scale = 1.0;
    s_surf_have   = 0;
    s_slope_valid = 0;
    memset(&s_step_diag, 0, sizeof s_step_diag);
    s_step_diag.collider_index = -1;
    s_lv.t = 0.0;
}

/* ----------------------------------------------------------------------- */
/* M4 vehicle sim — fixed 20 Hz step (physics-spec §1: no exceptions).      */
/* Model: ghidra-physics.md §6 mapping. Per-tick order: 1 input levels      */
/* (§Q13/§Q17), 2 gearbox + RPM + torque (§Q5), 3 per-wheel terrain         */
/* Registered object colliders and drivable bounds live in CarTransient.
 * Their caller-owned storage and historical defaults are unchanged. */
static void context_set_bounds_bound(double x0, double z0,
                                     double x1, double z1)
{
    if (x1 <= x0 || z1 <= z0) {
        s_bx0 = s_bz0 = 0.0;
        s_bx1 = s_bz1 = CAR_WORLD_MAX_M;
        return;
    }
    s_bx0 = x0; s_bz0 = z0; s_bx1 = x1; s_bz1 = z1;
}

static void context_apply_vehicle_contact_bound(double dx, double dz,
                                                double nx, double nz,
                                                double other_vx,
                                                double other_vz)
{
    if (!s_loaded)
        return;

    s_lv.x += dx;
    s_lv.z += dz;

    /* Match the static-contact response, but in the other body's frame:
     * retain tangential motion and its normal velocity, discard only closing
     * relative velocity. No restitution/friction coefficient is invented. */
    double n2 = nx * nx + nz * nz;
    if (!(n2 > 0.0))
        return;
    double inv_n = 1.0 / sqrt(n2);
    nx *= inv_n;
    nz *= inv_n;

    double cy = cos(s_lv.yaw), sy = sin(s_lv.yaw);
    double vwx = -sy * s_lv.vz + cy * s_lv.vx;
    double vwz =  cy * s_lv.vz + sy * s_lv.vx;
    double closing = (vwx - other_vx) * nx + (vwz - other_vz) * nz;
    if (closing < 0.0) {
        vwx -= closing * nx;
        vwz -= closing * nz;
        s_lv.vz = -sy * vwx + cy * vwz;
        s_lv.vx =  cy * vwx + sy * vwz;
    }
}

/* raycast + kinematic contact (§Q10), 4 forces (drive §Q5, brake §Q14,     */
/* drag+RR §Q7, slope gravity §Q2), 5/6 kick-drift integration with the     */
/* original's 1/2*a*dt^2 position term (§Q1, FUN_00422df0) and explicit     */
/* airborne branch (§Q2), 7 world bounds (D8), 8 terrain-aligned attitude   */
/* (§Q10), 9 landing events (§Q2, bridge-visible).                          */
/* ----------------------------------------------------------------------- */
/* VDFC mass-derived kg lives in each context's transient solver state. */

/* FACT §Q5 gear ratio table @0x4f9928 (indexed by gear, +gear*4): g2 =
 * 1.67, g3 = 0.96, g4 = 0.67; slot 0 = 3.0 (doc: "reverse-ish" —
 * DECISION D15: used as gear 1, whose table entry is 0.0/unused because
 * gear-1 RPM short-circuits to idle). Index 0 unused. */
static const double CAR_GEAR_RATIO[5] = { 0.0, 3.0, 1.67, 0.96, 0.67 };

static double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Surface property sampling (FACT §Q15; state declared with the landing
 * counters above). */

/* Shared tile->record query: -1 when the position has no valid tile or no
 * loaded table. */
static int surface_props_at(double x, double z, unsigned *cls,
                            TerrainSurfaceProps *p)
{
    int blocked;
    if (terrain_nav_sample(x, z, cls, &blocked) != 0)
        return -1;
    return terrain_surface_props(*cls, p);
}

/* Surface impact/damage scale (WRLD +0x10) at a world position; 1.0 when
 * no table/tile covers it — the landing path's fail-closed default. */
static double surface_impact_scale_at(double x, double z)
{
    unsigned cls;
    TerrainSurfaceProps p;
    if (surface_props_at(x, z, &cls, &p) != 0)
        return 1.0;
    return (double)p.impact;
}

static void surface_sample(void)
{
    unsigned cls = 0;
    TerrainSurfaceProps p;
    s_surf_grip   = 1.0;
    s_surf_rr     = CAR_RR_COEF;
    s_surf_impact = 1.0;
    s_surf_have   = 0;
    if (surface_props_at(s_lv.x, s_lv.z, &cls, &p) == 0) {
        s_surf_have  = 1;
        s_surf_class = cls;
        /* Fail closed per field on non-finite/authored-zero values: a corrupt
         * table must not zero the tire budget or negate drag. The int32 impact
         * scale needs no guard — every bit pattern is a valid authored value
         * (shipped data spans 0..999999999). */
        if (isfinite(p.grip) && p.grip > 0.0f) s_surf_grip = p.grip;
        if (isfinite(p.rr)   && p.rr >= 0.0f)  s_surf_rr   = p.rr;
        s_surf_impact = (double)p.impact;
    }
    if (s_grip_loss_ticks > 0) {
        s_surf_grip *= s_grip_loss_scale;
        if (--s_grip_loss_ticks == 0)
            s_grip_loss_scale = 1.0;
    }
}

static void context_apply_grip_loss_bound(double scale, int ticks)
{
    if (!isfinite(scale) || !(scale > 0.0) || scale > 1.0 || ticks <= 0)
        return;
    if (s_grip_loss_ticks <= 0 || scale < s_grip_loss_scale)
        s_grip_loss_scale = scale;
    if (ticks > s_grip_loss_ticks)
        s_grip_loss_ticks = ticks;
}

/* Derive dynamics constants once per context load (after wheels_derive). */
static void dynamics_derive(void)
{
    s_mass_kg = (double)s_cfg.mass * CAR_LB_TO_KG;
    if (s_mass_kg < 100.0) s_mass_kg = 600.0;   /* sane floor (D10) */
}

/* Attitude follows FUN_00423690's chassis-probe normal on a registered
 * object face. Terrain fallback retains the port's established four-corner
 * target bit-for-bit; native per-wheel samples therefore remain terrain-only
 * and world-less physics tapes do not change. */
static void terrain_attitude(double *pitch_t, double *roll_t)
{
    double centre_h, normal[3];
    if (terrain_drivable_probe(s_lv.x, s_lv.y, s_lv.z,
                               &centre_h, normal) == 1) {
        double cy = cos(s_lv.yaw), sy = sin(s_lv.yaw);
        double gx = -normal[0] / normal[1];
        double gz = -normal[2] / normal[1];
        double sf = gx * -sy + gz * cy;
        double sr = gx *  cy + gz * sy;
        *pitch_t = clampd(atan(sf), -CAR_PITCH_MAX, CAR_PITCH_MAX);
        *roll_t  = clampd(atan(sr), -CAR_ROLL_MAX, CAR_ROLL_MAX);
        return;
    }

    double hb = s_cfg.wheelbase, tr = s_cfg.track;
    /* corner offsets in car space: FL, FR, RL, RR (left = -x, fwd = +z) */
    static const double sx[4] = { -0.5, 0.5, -0.5, 0.5 };
    static const double sz[4] = {  0.5, 0.5, -0.5, -0.5 };
    double cy = cos(s_lv.yaw), sy = sin(s_lv.yaw);
    double h[4];
    for (int i = 0; i < 4; i++) {
        double lx = sx[i] * tr, lz = sz[i] * hb;
        double wx = s_lv.x + cy * lx - sy * lz;
        double wz = s_lv.z + sy * lx + cy * lz;
        h[i] = terrain_height_at(wx, wz);
    }
    double hf  = 0.5 * (h[0] + h[1]);   /* front axle avg */
    double hr  = 0.5 * (h[2] + h[3]);   /* rear axle avg  */
    double hl  = 0.5 * (h[0] + h[2]);   /* left side avg  */
    double hrt = 0.5 * (h[1] + h[3]);   /* right side avg */
    *pitch_t = clampd(atan2(hf - hr, hb), -CAR_PITCH_MAX, CAR_PITCH_MAX);
    *roll_t  = clampd(atan2(hrt - hl, tr), -CAR_ROLL_MAX, CAR_ROLL_MAX);
}

static void context_set_scripted_pose_bound(double x, double z,
                                            double yaw, double speed)
{
    double pitch_t, roll_t;
    if (!s_loaded)
        return;

    s_lv.x = x;
    s_lv.z = z;
    s_lv.yaw = yaw;
    {
        double h, normal[3];
        double terrain_y = terrain_height_at(x, z) + CAR_RIDE_H;
        (void)terrain_drivable_probe(x, terrain_y, z, &h, normal);
        s_lv.y = h + CAR_RIDE_H;
    }
    terrain_attitude(&pitch_t, &roll_t);
    s_lv.pitch = pitch_t;
    s_lv.roll = roll_t;

    s_lv.vx = 0.0;
    s_lv.vy = 0.0;
    s_lv.vz = speed;
    s_lv.yaw_rate = s_lv.pitch_rate = s_lv.roll_rate = 0.0;
    s_lv.throttle_pos = 0.0;
    s_lv.steer = 0.0;
    s_lv.steer_angle = 0.0;
    s_lv.a_long_prev = s_lv.y;
    s_slope_valid = 0;
    s_lv.t += CAR_DT;
}

static void context_step_bound(CarInput *in)
{
    CarInput zero = { 0 };
    if (!in) in = &zero;
    if (!s_loaded) return;

    memset(&s_step_diag, 0, sizeof s_step_diag);
    s_step_diag.valid = 1;
    s_step_diag.start_x = s_lv.x;
    s_step_diag.start_y = s_lv.y;
    s_step_diag.start_z = s_lv.z;
    s_step_diag.vy_cap = -1.0;
    s_step_diag.collider_index = -1;

    const double dt = CAR_DT;
    const double m  = s_mass_kg;

    /* --- 0. surface under the start-of-tick position (FACT §Q15) ------ */
    surface_sample();

    /* --- 0b. sharp local terrain slope, BOTH body axes (D24b/D24c) ----- */
    /* Central differences over CAR_SLOPE_SPAN_M along the body forward  */
    /* AND right axes.  Sharp on purpose — the wall-impact rule below    */
    /* must see a lattice-cell kink in one tick, and the vehicle-span    */
    /* samplers (terrain_attitude, the lerped s_lv.pitch/roll) smear it. */
    /* theta_f positive = uphill ahead (attitude-pitch convention);      */
    /* theta_r positive = uphill to the RIGHT (attitude-roll             */
    /* convention: terrain_attitude's roll_t = atan2(hrt - hl, tr)).     */
    /* slope_tan is tan of the TRUE steepest grade — heading-invariant,  */
    /* so driving diagonally across a fall line cannot dilute the felt   */
    /* grade (D24c: the round-2 diagonal-ascent exploit climbed a        */
    /* 76 deg face at ~65 deg off the fall line because every grade      */
    /* test was body-forward only).  Flat terrain gives exact 0.0 for    */
    /* all three, so every D24b/D24c term below is an identity there.    */
    double sf, sr, theta_f, theta_r, slope_tan;
    double h_old, ground_normal[3];
    {
        double cy0 = cos(s_lv.yaw), sy0 = sin(s_lv.yaw);
        int on_face = terrain_drivable_probe(s_lv.x, s_lv.y, s_lv.z,
                                             &h_old, ground_normal);
        s_step_diag.start_on_face = on_face == 1;
        if (on_face == 1) {
            double gx = -ground_normal[0] / ground_normal[1];
            double gz = -ground_normal[2] / ground_normal[1];
            sf = gx * -sy0 + gz * cy0;
            sr = gx *  cy0 + gz * sy0;
        } else {
            double fx = -sy0 * CAR_SLOPE_SPAN_M;
            double fz =  cy0 * CAR_SLOPE_SPAN_M;
            double rx =  cy0 * CAR_SLOPE_SPAN_M;
            double rz =  sy0 * CAR_SLOPE_SPAN_M;
            sf = (terrain_height_at(s_lv.x + fx, s_lv.z + fz)
                  - terrain_height_at(s_lv.x - fx, s_lv.z - fz))
                 / (2.0 * CAR_SLOPE_SPAN_M);
            sr = (terrain_height_at(s_lv.x + rx, s_lv.z + rz)
                  - terrain_height_at(s_lv.x - rx, s_lv.z - rz))
                 / (2.0 * CAR_SLOPE_SPAN_M);
        }
        theta_f = atan(sf);
        theta_r = atan(sr);
        slope_tan = hypot(sf, sr);
    }
    /* Sustainable-grade test (D24b HOLD, D24c heading-invariant): the
     * tire budget can hold the car against slope gravity only while
     * tan(TRUE grade) <= 0.8*grip (0.8*g FACT @0x4c1d98; grip FACT
     * §Q15).  The budget itself clamps against the true incline normal
     * force N = m*g*cos(true grade). */
    double cos_true = 1.0 / sqrt(1.0 + slope_tan * slope_tan);
    double a_budget = 0.8 * s_surf_grip * CAR_GRAVITY * cos_true;
    int over_grade = slope_tan > 0.8 * s_surf_grip;
    /* D24c gravity projection: the in-plane slope-gravity feed on each
     * body axis is -g * s * adv_g, where adv_g is cos of the slope
     * along the MOTION direction (fall line when at rest).  On any
     * single-axis slope this is exactly the FACT g*sin(pitch) form
     * (s*cos(atan(s)) == sin(atan(s))), so straight faces, authored
     * grades and flat ground are bit-identical; on a corner where BOTH
     * axes are steep the old independent per-axis sin() normalizations
     * over-fed gravity by up to 2x its true in-plane magnitude, and
     * round 2's diagonal climb harvested the difference (KE drain
     * -5 J/s against +98 J/s of PE gain).  This form makes gravity's
     * KE work equal -g * dh/dt exactly — the glue can no longer grant
     * height that gravity did not charge. */
    double adv_g;
    {
        double v_h = hypot(s_lv.vz, s_lv.vx);
        if (v_h > 1e-6) {
            double tanm = (sf * s_lv.vz + sr * s_lv.vx) / v_h;
            adv_g = 1.0 / sqrt(1.0 + tanm * tanm);
        } else {
            adv_g = cos_true;
        }
    }

    /* --- 1. input shaping (spec §7 step 1; §Q13/§Q17 levels) ---------- */
    double pedal     = clampd((double)in->throttle, 0.0, 1.0);
    double brake_pos = clampd((double)in->brake,    0.0, 1.0);
    /* Action bits act only on exactly 1 (car.h contract); a hard equality
     * test also makes stray/uninitialized values inert — callers MUST
     * still zero-initialize the struct. */
    if (in->shift_up == 1) {            /* edge: gear up (D15) */
        if (s_lv.gear < 4) s_lv.gear++;
        s_lv.shift_hold = CAR_SHIFT_HOLD_S;
    }
    in->shift_up = 0;
    if (in->shift_down == 1) {          /* edge: gear down (D15) */
        if (s_lv.gear > 1) s_lv.gear--;
        s_lv.shift_hold = CAR_SHIFT_HOLD_S;
    }
    in->shift_down = 0;
    /* Held level (FACT §Q14: type-1 held action, no latch — release
     * clears): the handbrake follows this tick's exact active input,
     * so it never persists past a released/zero tick. */
    int handbrake_was_held = s_lv.handbrake;
    s_lv.handbrake = (in->e_brake == 1);
    in->e_brake = 0;
    /* D26 (H-UAT-033): the port's yaw response is a persistent first-order
     * state, unlike the original wheel-constraint model where yaw emerges
     * from this tick's contact forces (Q17: no explicit yaw damping/state was
     * found). Carrying that invented state across the held->released edge made
     * the first opposite command keep turning with the handbrake skid. Clear
     * only angular carryover on release; lateral velocity remains, preserving
     * the authored exaggerated skid instead of snapping the whole car straight. */
    if (handbrake_was_held && !s_lv.handbrake)
        s_lv.yaw_rate = 0.0;
    /* FACT §Q13 (FUN_0043ebf0): reverse_direction flips the direction
     * sign with NO speed gate — D21's |v|<1 m/s gate is retired. */
    if (in->reverse == 1)
        s_lv.reverse = !s_lv.reverse;
    in->reverse = 0;
    /* The sim reads the steer level directly; the original's digital
     * sqrt(held/3) ramp lives in its input layer (§Q17), caller-side
     * for us. No slew. */
    s_lv.steer = clampd((double)in->left - (double)in->right, -1.0, 1.0);
    /* FACT §Q17: front wheel angle = steer * pi/4 (max lock 45 deg,
     * @0x4c1d3c). */
    s_lv.steer_angle = s_lv.steer * CAR_STEER_MAX;
    /* Effective signed throttle: the brake pedal is negative throttle
     * (FACT §Q13/Q14 single-axis semantics); e-brake forces -1.0
     * (FACT §Q14, far-mover quote in FUN_00427ba0). */
    double thr_eff = pedal - brake_pos;
    if (s_lv.handbrake) thr_eff = -1.0;
    s_lv.throttle_pos = thr_eff;        /* repurposed: signed effective */
    /* Native wheel anti-slip is passive in both horizontal axes. Capture
     * near-rest BEFORE this tick's slope gravity is integrated; testing only
     * afterward made the hold depend on whether one gravity increment happened
     * to fit inside CAR_STOP_EPS. Driver force and over-budget grades cannot
     * arm it. */
    int passive_hold = pedal == 0.0 && brake_pos == 0.0 &&
                       !s_lv.handbrake && !over_grade &&
                       hypot(s_lv.vz, s_lv.vx) < CAR_STOP_EPS;

    /* --- 2. powertrain (FACT §Q5: FUN_00453700/00453520/004534a0) ----- */
    const double kmh = fabs(s_lv.vz) * 3.6;   /* 3.6 @0x4c4760 FACT */
    int gear = s_lv.gear;
    if (gear < 1) gear = 1;
    if (gear > 4) gear = 4;
    if (s_lv.reverse) {
        gear = 1;                /* selector 3 forces gear 1 (FACT §Q5) */
    } else if (s_lv.shift_hold > 0.0) {
        s_lv.shift_hold -= dt;           /* manual hold (D15) */
    } else {
        /* auto box: upshift windows from the FACT constants
         * 25/40/62/106 km/h (@0x4c4764..0x4c4798); no upshift while
         * coasting (throttle edge 0.002, same constant block). */
        double wup = 1e30;
        if (gear == 1)      wup = 25.0 + (40.0 - 25.0) * pedal;
        else if (gear == 2) wup = 62.0;
        else if (gear == 3) wup = 106.0;
        if (pedal > 0.002 && gear < 4 && kmh >= wup) {
            gear++;
        } else {
            /* Downshift windows + hyperbolic kickdown (FACT §Q5 quoted
             * rules: 4->3 window 98, kickdown *(0.625/67)*0.875, floor
             * 31; 3->2 window 60, kickdown *(0.5625/48)*0.875, floor
             * 12). DECISION: kickdown bands are bounded below the
             * matching upshift window — the decompiled window test has
             * an elided guard (doc §8) and the raw quotes contradict
             * the upshift constants without it. 2->1 floor DECISION. */
            if (gear == 4 &&
                (kmh < 98.0 || kmh < 31.0 ||
                 (kmh < 106.0 &&
                  (kmh - 98.0) * 0.625 * (1.0 / 67.0) * 0.875 < pedal)))
                gear = 3;
            else if (gear == 3 &&
                (kmh < 60.0 || kmh < 12.0 ||
                 (kmh < 62.0 &&
                  (kmh - 60.0) * 0.5625 * (1.0 / 48.0) * 0.875 < pedal)))
                gear = 2;
            else if (gear == 2 && kmh < 10.0)
                gear = 1;
        }
    }
    s_lv.gear = gear;
    /* RPM (FACT §Q5): gear 1 short-circuits to idle 1050 (@0x4c47c0);
     * engaged target = ratio*(|v|*0.59738*60)*3.538 + 850, slewed at
     * 2*dt per tick (window <= 0.95), capped at redline 6000
     * (@0x4c47f0). */
    double rpm_t;
    if (gear == 1) rpm_t = CAR_RPM_IDLE;
    else rpm_t = CAR_GEAR_RATIO[gear] * (fabs(s_lv.vz) * 0.59738 * 60.0)
                 * 3.538 + 850.0;
    if (rpm_t > CAR_RPM_REDLINE) rpm_t = CAR_RPM_REDLINE;
    {
        double k = 2.0 * dt;
        if (k > 0.95) k = 0.95;
        s_lv.engine_rpm += (rpm_t - s_lv.engine_rpm) * k;
    }
    /* Authored curve (FACT): FUN_00453700 writes this output directly.
     * FUN_004280c0 then applies raw VDF mass reciprocal and, above 1 m/s,
     * reciprocal speed — not gear/final-drive/wheel-radius. Multiplying by
     * lb->kg before the existing SI mass division is algebraically the
     * native raw-numeric-mass division (engine-curve-consumption.md). */
    double drpm = s_lv.engine_rpm - CAR_RPM_PEAK_TQ;
    double torque = (double)s_cfg.engine_tpeak
                  - (double)s_cfg.engine_k * drpm * drpm;
    double f_drive = 0.0;
    if (thr_eff > 0.0) {
        /* soft rev limiter remains a DECISION form; redline itself is FACT */
        double lim = clampd((CAR_RPM_REDLINE - s_lv.engine_rpm) /
                            CAR_REVLIM_BAND, 0.0, 1.0);
        double native_speed_scale = fabs(s_lv.vz) > 1.0
                                  ? 1.0 / fabs(s_lv.vz) : 1.0;
        f_drive = torque * CAR_LB_TO_KG * native_speed_scale * thr_eff * lim;
        if (s_lv.reverse) {             /* sign flip (FACT §Q13) */
            f_drive = -f_drive;
            if (s_lv.vz <= -CAR_REVERSE_MAX) f_drive = 0.0;
            else if (s_lv.vz < 0.0)
                f_drive *= 1.0 + s_lv.vz / CAR_REVERSE_MAX;
        }
        /* tire budget clamp: 0.8*g scaled by the surface grip (budget
         * FACT 7.84 @0x4c1d98; grip scale FACT §Q15, WRLD table +0x00),
         * against the NORMAL force on the incline: N = m*g*cos(pitch)
         * (DECISION D24 — the cos projection is the complement of the
         * FACT §Q2 g*sin(pitch) slope split below; no new constant).
         * Without it, authored grips > 1.25 (P01 dirt is 1.5) make
         * 0.8*grip > sin(pitch) for EVERY pitch, so full throttle
         * out-pulled gravity on arbitrarily steep faces and the car
         * drove up cliffs (H-UAT-012). The grip-derived climb limit is
         * tan(pitch) <= 0.8*grip: ~50 deg on P01 dirt, ~59 deg on road.
         * D24b: the projection reads the SHARP local terrain slope, not
         * the lerped attitude pitch whose ~0.3 s lag left full drive on
         * during flat->steep transitions. */
        double fmax = m * a_budget;
        f_drive = clampd(f_drive, -fmax, fmax);
    }

    /* --- 3. ground contact: per-wheel terrain raycast (FACT §Q10, ----- */
    /* FUN_0041ecb0): contact flags + depths; the chassis contact is the */
    /* kinematic ground-probe follow (FUN_00423690), decided below.      */
    double cy = cos(s_lv.yaw), sy = sin(s_lv.yaw);
    int grounded = 0;
    for (int i = 0; i < 6; i++) {
        s_lv.grounded[i] = 0;
        s_lv.comp_prev[i] = 0.0;        /* repurposed: contact depth */
        if (!s_cfg.wheel_present[i]) continue;
        double lx = s_cfg.wheel_pos[i][0], lz = s_cfg.wheel_pos[i][2];
        double wx = s_lv.x + cy * lx - sy * lz;
        double wz = s_lv.z + sy * lx + cy * lz;
        double wheel_h = terrain_height_at(wx, wz);
        double ride_h = s_lv.y - wheel_h;
        s_step_diag.wheel_present[i] = 1;
        s_step_diag.wheel_x[i] = wx;
        s_step_diag.wheel_z[i] = wz;
        s_step_diag.wheel_terrain_y[i] = wheel_h;
        s_step_diag.wheel_penetration[i] =
            ride_h < CAR_RIDE_H ? CAR_RIDE_H - ride_h : 0.0;
        if (ride_h <= CAR_CONTACT_H) {  /* DECISION contact band */
            s_lv.grounded[i] = 1;
            grounded++;
            s_lv.comp_prev[i] = CAR_CONTACT_H - ride_h;
            s_lv.wheel_w[i] = s_lv.vz / CAR_WHEEL_RADIUS;
        }
        /* airborne wheels free-spin (FACT §Q2): wheel_w left as-is */
    }
    s_lv.grounded_count = grounded;
    int contact = (s_lv.y <= h_old + CAR_RIDE_H + 0.02);

    /* --- 4. forces (contact only — FACT §Q2: the original's force ----- */
    /* blocks are guarded by ground contact; airborne is gravity-only)   */
    double vz_old = s_lv.vz;
    double vx_old = s_lv.vx;
    if (contact) {
        /* D24b IMPACT: the tires can steer the velocity through at most
         * (0.8*grip*g)*dt/|v| radians per tick (the FACT budget read as
         * centripetal authority — a DECISION, no new constant).  When
         * the local slope under the motion kinks up faster than that,
         * the surface is a WALL: the velocity keeps only cos(excess) —
         * the into-face component is destroyed, so kinetic energy alone
         * can never buy a crest (pre-D24b the velocity silently rotated
         * onto the new slope at full magnitude, and any face lower than
         * v^2/2g — 168 m at flat-out speed — was still crestable).
         * Sign-flip (drive->slide-back transition) re-anchors without a
         * charge; slope DECREASES (convex crests) are free — the car
         * loses contact there, it doesn't hit anything. */
        {
            if (!s_slope_valid) {
                /* D24c LANDING STRIKE: fresh contact (landing, teleport,
                 * restore) — round 2 proved that skipping the charge
                 * here is the entire defence: a 0.4 m hop just before
                 * the base kink used to reset the tracker and let a
                 * 45 m/s entry crest via the honest-PE LIFT exchange.
                 * On landing, strike the surface actually landed on:
                 * keep only the tangential component per body axis
                 * (v' = v*cos(slope) + vy*sin(slope) — the velocity
                 * component INTO the face is destroyed, exactly the
                 * D24b wall rule applied to the landing plane; no new
                 * constant).  Flat: cos==1, sin==0, vy untouched — an
                 * exact identity, and a normal glue landing reaches
                 * here with vy already absorbed, so gentle terrain
                 * feels nothing. */
                s_lv.vz = s_lv.vz * cos(theta_f) + s_lv.vy * sin(theta_f);
                s_lv.vx = s_lv.vx * cos(theta_r) + s_lv.vy * sin(theta_r);
            } else {
                double sgn = s_lv.vz >= 0.0 ? 1.0 : -1.0;
                double th_m = sgn * theta_f;    /* slope along motion */
                double av = fabs(s_lv.vz);
                if (sgn == s_slope_sgn && av > 1e-6) {
                    double dth = th_m - s_slope_prev;
                    double dth_free = 0.8 * s_surf_grip * CAR_GRAVITY * dt
                                    / av;
                    if (dth > dth_free)
                        s_lv.vz *= cos(dth - dth_free);
                }
            }
            double sgn2 = s_lv.vz >= 0.0 ? 1.0 : -1.0;
            s_slope_prev = sgn2 * theta_f;
            s_slope_r_prev = theta_r;
            s_slope_sgn = sgn2;
            s_slope_valid = 1;
        }
        /* longitudinal: drive + slope gravity (FACT §Q2: gravity is
         * projected onto the contact plane; D24b reads the sharp local
         * terrain slope instead of the ~0.3 s lerped attitude pitch;
         * D24c projects by adv_g — identical to sin(theta_f) on any
         * single-axis slope, energy-exact on diagonal corners) */
        double a_long = f_drive / m - CAR_GRAVITY * sf * adv_g;
        /* drag + rolling resistance (FACT §Q7): quadratic v^2*0.1 along
         * heading + surface rr (FACT §Q15: WRLD table +0x04 coefficient
         * against m*g; CAR_RR_COEF only when no table is loaded),
         * deadzone below 1 m/s */
        if (fabs(s_lv.vz) >= CAR_DRAG_DEADZONE) {
            double f_rr = s_surf_rr * m * CAR_GRAVITY;
            a_long -= ((double)s_cfg.drag_coeff * s_lv.vz *
                        fabs(s_lv.vz) * CAR_DRAG_SCALE
                       + (s_lv.vz > 0.0 ? f_rr : -f_rr)) / m;
        }
        s_lv.vz += a_long * dt;
        /* lateral constraint (FACT §Q10/§Q14 velocity-clamp tire):
         * slip velocity is strongly damped; slope gravity feeds it.
         * D24c: the gravity feed reads the sharp terrain slope along
         * the body right axis (same convention as the attitude roll it
         * replaces, without the lerp lag or the ±45 deg roll clamp that
         * understated steep faces), and ABOVE the sustainable grade the
         * damping is budgeted: the lateral tire force shares the
         * 0.8*grip*g*cos friction budget with the drive force (circle),
         * so a cliff face cannot be held — or climbed — by pointing
         * across the fall line and leaning on an unbudgeted lateral
         * clamp.  Below the sustainable grade the authored clamp is
         * untouched (normal cornering identical). */
        s_lv.vx -= CAR_GRAVITY * sr * adv_g * dt;
        {
            double vx_c = s_lv.vx * exp(-CAR_VX_CLAMP * dt);
            if (over_grade) {
                double a_df = fabs(f_drive) / m;
                double lat2 = a_budget * a_budget - a_df * a_df;
                double dvmax = (lat2 > 0.0 ? sqrt(lat2) : 0.0) * dt;
                double dvx = s_lv.vx - vx_c;
                if (fabs(dvx) > dvmax)
                    vx_c = s_lv.vx - copysign(dvmax, dvx);
            }
            s_lv.vx = vx_c;
        }
        /* yaw from the friction-budget steering curve (FACT §Q17):
         * yaw authority = steer * min(C*v, Amax/v) — proportional to v
         * at low speed, to 1/v at high speed; Amax = 7.84 = 0.8*g
         * (FACT @0x4c1d98) scaled by the surface grip (FACT §Q15). */
        double av = fabs(s_lv.vz);
        if (av > 0.5) {
            double r_t = s_lv.steer * fmin(CAR_LAT_C * av,
                                           CAR_LAT_AMAX * s_surf_grip / av);
            double k = 1.0 - exp(-CAR_YAW_RESP * dt);
            s_lv.yaw_rate += (r_t - s_lv.yaw_rate) * k;
        } else {
            s_lv.yaw_rate *= exp(-CAR_YAW_RESP * dt);
        }
        /* service brake / e-brake (FACT §Q14): force opposes the
         * contact VELOCITY vector on all wheels. The original also
         * applies the x8.0 engine-brake term whenever the accelerator is
         * released (FACT @0x4c1d90); the old `thr_eff < 0` guard made that
         * term zero at neutral throttle, so lifting off barely slowed the
         * car. Velocity-opposing decay stops but can never reverse it. */
        if (thr_eff < 0.0 || pedal <= 0.0) {
            double vmag = hypot(s_lv.vx, s_lv.vz);
            if (vmag > 1e-9) {
                double b = (-thr_eff > 0.0 ? -thr_eff : 0.0)
                         * CAR_BRAKE_DECEL;
                if (pedal <= 0.0 && vmag > 1.0)
                    b += CAR_ENG_BRAKE_COEF * 8.0;
                /* D24b HOLD: brakes act through the tires, so above the
                 * sustainable grade the decel saturates at the budget
                 * 0.8*grip*g*cos(slope) — less than the slope gravity
                 * feed by definition of over_grade, so the handbrake
                 * can no longer glue the car to a cliff face.  Below
                 * the sustainable grade the authored CAR_BRAKE_DECEL
                 * is untouched (flat braking bit-identical). */
                if (over_grade && b > a_budget)
                    b = a_budget;
                double dv = b * dt;
                if (dv > vmag) dv = vmag;
                double s = (vmag - dv) / vmag;
                s_lv.vx *= s;
                s_lv.vz *= s;
            }
        }
        /* D25 passive grounded-tire hold. Both axes are constrained so a
         * cross-grade component cannot drift while the speedometer stays 0.
         * Above the sustainable grade passive_hold is false (D24b), so a
         * coasting car on a cliff still slides rather than sticking. */
        if (passive_hold) {
            s_lv.vz = 0.0;
            s_lv.vx = 0.0;
        }
    } else {
        /* airborne (FACT §Q2): gravity-only ballistic, no drive/tire/
         * drag forces; attitude rates persist with damping (D20) */
        if (s_slope_valid) {
            /* D24c EXIT: just left the ground.  In contact vz/vx are
             * ALONG-SLOPE speeds while vy separately carries the
             * vertical follow rate; airborne they are consumed as
             * HORIZONTAL speeds.  Convert at the boundary (v *=
             * cos(slope it was on)) or the vertical component is
             * double-counted — round 2 proved contact flicker on a
             * steep face could harvest that double-count through the
             * landing strike and ladder the car up a wall.  Flat
             * ground: cos(0) == 1, exact identity.  A ramp lip now
             * launches with the true ballistic split (v*cos, v*sin)
             * instead of (v, v*sin). */
            s_lv.vz *= cos(s_slope_prev);
            s_lv.vx *= cos(s_slope_r_prev);
            s_slope_valid = 0;          /* D24b: re-anchor on landing */
        }
        double damp = exp(-CAR_AIR_ANG_DAMP * dt);
        s_lv.yaw_rate   *= damp;
        s_lv.pitch_rate *= damp;
        s_lv.roll_rate  *= damp;
        s_lv.pitch += s_lv.pitch_rate * dt;
        s_lv.roll  += s_lv.roll_rate * dt;
        s_lv.pitch = clampd(s_lv.pitch, -CAR_PITCH_MAX, CAR_PITCH_MAX);
        s_lv.roll  = clampd(s_lv.roll, -CAR_ROLL_MAX, CAR_ROLL_MAX);
    }

    /* --- 5/6. integration (FACT §Q1, FUN_00422df0): pos advances with  */
    /* (v_old + v_new)/2 — identical to the original's (v + 1/2*a*dt)*dt */
    /* for the linear force terms. The world velocity is carried across  */
    /* the yaw update (re-expressed in the rotated frame) so the         */
    /* centripetal coupling falls out of the frame rotation — that IS    */
    /* the anti-slip constraint (§Q10: "yaw comes from the constraint    */
    /* forces").                                                        */
    double vwx0 = -sy * vz_old + cy * vx_old;      /* old frame (D7) */
    double vwz0 =  cy * vz_old + sy * vx_old;
    double vwx1 = -sy * s_lv.vz + cy * s_lv.vx;
    double vwz1 =  cy * s_lv.vz + sy * s_lv.vx;
    /* In contact, vz is the ALONG-SLOPE speed — the §Q2 FACT slope split
     * above already applies g*sin(pitch) to it — so its horizontal
     * advance is speed * cos(local slope along the motion direction)
     * (DECISION D24, no new constant). The slope comes from the terrain
     * gradient, NOT the lerped attitude pitch: the attitude lags a tile
     * transition by ~0.3 s, and during that lag the old unprojected
     * advance let the kinematic ground glue mint potential energy — full
     * horizontal speed across a steep face plus a free vertical lift, so
     * momentum alone crested real cliffs (H-UAT-012). Flat terrain gives
     * dh == 0 -> adv == 1.0 exactly: flat-ground tapes are bit-identical.
     * Airborne keeps the unprojected advance. */
    double adv = 1.0;
    if (contact) {
        double avx = 0.5 * (vwx0 + vwx1), avz = 0.5 * (vwz0 + vwz1);
        double sp = hypot(avx, avz);
        if (sp > 1e-6) {
            const double ds = 2.5;   /* half-wheelbase probe span, m */
            double ahead_h, ahead_n[3];
            (void)terrain_drivable_probe(s_lv.x + avx / sp * ds, s_lv.y,
                                         s_lv.z + avz / sp * ds,
                                         &ahead_h, ahead_n);
            double dh = ahead_h - h_old;
            adv = ds / sqrt(ds * ds + dh * dh);
        }
    }
    s_lv.x += 0.5 * (vwx0 + vwx1) * dt * adv;
    s_lv.z += 0.5 * (vwz0 + vwz1) * dt * adv;
    s_step_diag.integrated_x = s_lv.x;
    s_step_diag.integrated_z = s_lv.z;
    s_lv.yaw += s_lv.yaw_rate * dt;
    if (s_lv.yaw > CAR_PI) s_lv.yaw -= 2.0 * CAR_PI;
    else if (s_lv.yaw <= -CAR_PI) s_lv.yaw += 2.0 * CAR_PI;
    {
        double cy1 = cos(s_lv.yaw), sy1 = sin(s_lv.yaw);
        s_lv.vz = -sy1 * vwx1 + cy1 * vwz1;
        s_lv.vx =  cy1 * vwx1 + sy1 * vwz1;
    }

    /* H-UAT-070a/H-UAT-078d — MARKED TERRAIN-WALL CONVENTION pending
     * native slope-response decode. Preserve the existing >=40 m / 15 m
     * mountain bracket and ordinary D24 small-cliff path, but make contact
     * geometric rather than approach-direction dependent. Sixty-four points on
     * the authored collision radius sweep the rendered triangles at the
     * existing ground-contact band and at one/two metres through the already
     * documented body band. Rendered-triangle sweeps are paired with the
     * grounding path's bilinear-height crossing, including a zero-distance
     * response that removes only further inward velocity when a facet seam
     * starts a support point overlapped (it never ejects). This closes grazing
     * and airborne entries: the former forward-only leading point could miss a
     * slanted facet, and the `contact` guard skipped a hopping car. Relief uses
     * the same bounded 15 m bracket uphill along the contacted face normal,
     * independent of car approach. The old height/bisection path remains only
     * as the empty->present seam
     * fallback, where no complete rendered quad exists. Response is still
     * contact-only and planar: truncate to first contact and remove only the
     * velocity into the face; never write y or vy. */
    {
        double mdx = s_lv.x - s_step_diag.start_x;
        double mdz = s_lv.z - s_step_diag.start_z;
        double travel = hypot(mdx, mdz);
        if (travel > 1e-9) {
            double ux = mdx / travel, uz = mdz / travel;
            double cr = s_cfg.coll_radius > 0.0f
                      ? (double)s_cfg.coll_radius : 1.0;
            double wall_t = 2.0, wall_nx = 0.0, wall_nz = 0.0;
            double wall_rise = 0.0;

            for (int i = 0; i < CAR_TERRAIN_WALL_HULL_SAMPLES; i++) {
                double a = 2.0 * CAR_PI * (double)i /
                           CAR_TERRAIN_WALL_HULL_SAMPLES;
                double ox = cos(a) * cr, oz = sin(a) * cr;
                for (int bi = 0; bi < CAR_TERRAIN_WALL_BODY_SAMPLES; bi++) {
                    double band_y = bi == 0 ? h_old + CAR_CONTACT_H
                                           : s_step_diag.start_y + (double)bi;
                    double p0[3] = {s_step_diag.start_x + ox, band_y,
                                    s_step_diag.start_z + oz};
                    double p1[3] = {s_step_diag.integrated_x + ox, band_y,
                                    s_step_diag.integrated_z + oz};
                    double t = 2.0, mesh_t;
                    if (terrain_segment_hit(p0, p1, &mesh_t))
                        t = mesh_t;
                    double d0 = terrain_height_at(p0[0], p0[2]) - band_y;
                    double d1 = terrain_height_at(p1[0], p1[2]) - band_y;
                    int height_inward = d1 > 0.0 &&
                                         (d0 <= 0.0 || d1 > d0 + 1e-9);
                    if (d1 > 0.0) {
                        double ht = 0.0;
                        if (d0 <= 0.0) {
                            double lo = 0.0, hi = 1.0;
                            for (int it = 0; it < 16; it++) {
                                double mid = 0.5 * (lo + hi);
                                double mx = p0[0] + (p1[0] - p0[0]) * mid;
                                double mz = p0[2] + (p1[2] - p0[2]) * mid;
                                if (terrain_height_at(mx, mz) <= band_y)
                                    lo = mid;
                                else
                                    hi = mid;
                            }
                            ht = lo;
                        }
                        if (ht < t) t = ht;
                    }
                    if (t > wall_t || t > 1.0) continue;
                    double qx = p0[0] + (p1[0] - p0[0]) * t;
                    double qz = p0[2] + (p1[2] - p0[2]) * t;
                    const double ns = 0.5;
                    double gx = (terrain_height_at(qx + ns, qz) -
                                 terrain_height_at(qx - ns, qz)) /
                                (2.0 * ns);
                    double gz = (terrain_height_at(qx, qz + ns) -
                                 terrain_height_at(qx, qz - ns)) /
                                (2.0 * ns);
                    double gl = hypot(gx, gz);
                    if (gl <= 1e-9) continue;
                    double nx = -gx / gl, nz = -gz / gl;
                    /* A below->above height crossing (or worsening overlap at
                     * a facet seam) is itself geometric inward motion. Trust
                     * it over a noisy central-difference normal; otherwise a
                     * genuinely tangent/outward segment is not contact. */
                    if (!height_inward && mdx * nx + mdz * nz >= -1e-9)
                        continue;

                    double mountain_rise = 0.0;
                    double upx = gx / gl, upz = gz / gl;
                    for (double look = 0.0;
                         look <= CAR_TERRAIN_WALL_LOOKAHEAD; look += 2.5) {
                        double h = terrain_height_at(qx + upx * look,
                                                     qz + upz * look) - h_old;
                        if (h > mountain_rise) mountain_rise = h;
                    }
                    if (mountain_rise < CAR_TERRAIN_WALL_MIN_RISE ||
                        mountain_rise /
                            fmax(cr + CAR_TERRAIN_WALL_LOOKAHEAD, 0.1) <=
                            0.8 * s_surf_grip)
                        continue;
                    wall_t = t;
                    wall_nx = nx;
                    wall_nz = nz;
                    wall_rise = mountain_rise;
                }
            }

            /* Empty->high-present patch boundaries have no complete rendered
             * triangle for terrain_segment_hit. Keep H-UAT-070a's exact
             * forward-height fallback there, including its start-contact
             * requirement and contact-band bisection. */
            if (wall_t > 1.0 && contact) {
                double lead_x = s_lv.x + ux * cr;
                double lead_z = s_lv.z + uz * cr;
                double rise = terrain_height_at(lead_x, lead_z) - h_old;
                double mountain_rise = rise;
                for (double look = 2.5;
                     look <= CAR_TERRAIN_WALL_LOOKAHEAD; look += 2.5) {
                    double h = terrain_height_at(lead_x + ux * look,
                                                 lead_z + uz * look) - h_old;
                    if (h > mountain_rise) mountain_rise = h;
                }
                if (rise > CAR_CONTACT_H &&
                    mountain_rise >= CAR_TERRAIN_WALL_MIN_RISE &&
                    mountain_rise /
                        fmax(cr + CAR_TERRAIN_WALL_LOOKAHEAD, 0.1) >
                        0.8 * s_surf_grip) {
                    double lo = 0.0, hi = 1.0;
                    for (int it = 0; it < 16; it++) {
                        double mid = 0.5 * (lo + hi);
                        double qx = s_step_diag.start_x + mdx * mid + ux * cr;
                        double qz = s_step_diag.start_z + mdz * mid + uz * cr;
                        if (terrain_height_at(qx, qz) - h_old <= CAR_CONTACT_H)
                            lo = mid;
                        else
                            hi = mid;
                    }
                    double qx = s_step_diag.start_x + mdx * lo + ux * cr;
                    double qz = s_step_diag.start_z + mdz * lo + uz * cr;
                    const double ns = 0.5;
                    double gx = (terrain_height_at(qx + ns, qz) -
                                 terrain_height_at(qx - ns, qz)) /
                                (2.0 * ns);
                    double gz = (terrain_height_at(qx, qz + ns) -
                                 terrain_height_at(qx, qz - ns)) /
                                (2.0 * ns);
                    double gl = hypot(gx, gz);
                    wall_t = lo;
                    wall_nx = gl > 1e-9 ? -gx / gl : -ux;
                    wall_nz = gl > 1e-9 ? -gz / gl : -uz;
                    wall_rise = mountain_rise;
                }
            }

            if (wall_t <= 1.0) {
                s_lv.x = s_step_diag.start_x + mdx * wall_t;
                s_lv.z = s_step_diag.start_z + mdz * wall_t;
                double cyw = cos(s_lv.yaw), syw = sin(s_lv.yaw);
                double vwx = -syw * s_lv.vz + cyw * s_lv.vx;
                double vwz =  cyw * s_lv.vz + syw * s_lv.vx;
                double vn = vwx * wall_nx + vwz * wall_nz;
                if (vn < 0.0) {
                    vwx -= vn * wall_nx;
                    vwz -= vn * wall_nz;
                    s_lv.vz = -syw * vwx + cyw * vwz;
                    s_lv.vx =  cyw * vwx + syw * vwz;
                }
                s_step_diag.terrain_wall_hit = 1;
                s_step_diag.terrain_wall_t = wall_t;
                s_step_diag.terrain_wall_rise = wall_rise;
                s_step_diag.terrain_wall_nx = wall_nx;
                s_step_diag.terrain_wall_nz = wall_nz;
                s_step_diag.terrain_wall_removed = travel * (1.0 - wall_t);
            }
        }
    }

    double end_ground_h;
    /* vertical: kinematic ground contact (FACT §Q10) — no spring sim.
     * The chassis follows the ground probe height while in contact. Its
     * positional correction may be arbitrarily steep, but only a bounded
     * upward component becomes inertial launch velocity: the old raw
     * height-delta transfer turned N01's 3.3 m terrain ramp into a 14 m
     * catapult. Falling terrain keeps its signed rate so real drop-offs still
     * produce a ballistic arc. */
    {
        double h_new, h_normal[3];
        int end_on_face = terrain_drivable_probe(s_lv.x, s_lv.y, s_lv.z,
                                                  &h_new, h_normal);
        end_ground_h = h_new;
        double h_f = h_new + CAR_RIDE_H;
        double terrain_rate = (h_new - h_old) / dt;
        double terrain_v = terrain_rate > CAR_LAUNCH_MAX
                         ? CAR_LAUNCH_MAX : terrain_rate;
        s_step_diag.end_on_face = end_on_face == 1;
        s_step_diag.ground_y = h_new;
        s_step_diag.terrain_rate = terrain_rate;
        s_step_diag.terrain_v_raw = terrain_rate;
        s_step_diag.launch_cap_applied = terrain_rate > CAR_LAUNCH_MAX;
        s_step_diag.terrain_v_pre_vy_cap = terrain_v;
        /* D24c: the upward follow-rate grant is also capped by the
         * vertical rate the car's ACTUAL speed can sustain on the true
         * local slope, |v| * sin(true grade) — round 2 proved the
         * unpaid grant was an energy pump: on a near-vertical face a
         * ~1 m/s crawl still received up to CAR_LAUNCH_MAX of free
         * vertical velocity every glued tick, and the landing strike
         * then honestly converted it into up-slope speed, laddering
         * the car up the wall.  Ramp launches are unaffected (there
         * |v|*sin(slope) is exactly the terrain rate).  No new
         * constant: slope_tan*cos_true is sin of the D24c true grade. */
        if (terrain_v > 0.0) {
            double vy_cap = hypot(s_lv.vz, s_lv.vx)
                          * slope_tan * cos_true;
            s_step_diag.vy_cap = vy_cap;
            if (terrain_v > vy_cap) {
                terrain_v = vy_cap;
                s_step_diag.vy_cap_applied = 1;
            }
        }
        s_step_diag.terrain_v_post_vy_cap = terrain_v;
        double vy_new = s_lv.vy - CAR_GRAVITY * dt;
        double y_ball = s_lv.y + 0.5 * (s_lv.vy + vy_new) * dt;
        if (y_ball <= h_f && fabs(terrain_rate) <= CAR_FOLLOW_MAX) {
            s_step_diag.follow_branch = 1;
            int was_air = (s_lv.y > h_old + CAR_RIDE_H + 0.02);
            if (was_air) {
                double impact = -vy_new;   /* vertical impact speed */
                if (impact > CAR_LAND_HARD) {  /* FACT 7.65 @0x4c1d58 */
                    s_hard_landings++;
                    s_last_impact = impact;
                    /* FACT §Q15: the original's landing path
                     * (FUN_00424da0) scales the damage call by the
                     * impacted tile's WRLD +0x10 record (FUN_00497780 ->
                     * FUN_00463370). This path owns impact detection, so
                     * the scale of the surface under the landing position
                     * is captured here; the damage routing itself
                     * (combat.c D-C7) is unchanged. */
                    s_last_impact_scale =
                        surface_impact_scale_at(s_lv.x, s_lv.z);
                    /* bounce impulse -0.2 (FACT @0x4c1d50) as a
                     * restitution: rebound at 0.2 * impact */
                    s_lv.vy = terrain_v + CAR_LAND_BOUNCE * impact;
                } else {
                    s_lv.vy = terrain_v;
                }
                /* fall 35 m below the tracked apex -> reset-to-road in
                 * the original (FACT 35.0 @0x4c1d44); counter-only here */
                if (s_lv.a_long_prev - h_new > CAR_FALL_RESET)
                    s_fall_resets++;
            } else {
                s_lv.vy = terrain_v;
            }
            s_lv.y = h_f;
            s_lv.a_long_prev = s_lv.y;   /* grounded: apex tracks pose */
        } else {
            s_step_diag.ballistic_branch = 1;
            s_lv.y = y_ball;             /* airborne ballistic (or steep-
                                          * face contact loss: the one-
                                          * sided constraint below still
                                          * keeps y out of the ground
                                          * without imparting velocity) */
            s_lv.vy = vy_new;
            /* fall-apex tracking (FACT §Q2 ent+0x474; a_long_prev
             * repurposed — D18 retired, layout frozen by save.c) */
            if (s_lv.a_long_prev < s_lv.y) s_lv.a_long_prev = s_lv.y;
        }
    }
    /* one-sided ground-contact constraint (D6, kept): inelastic.  D24b
     * LIFT: this pop-out is the path a steep face takes when the
     * kinematic follow disengages (|terrain_rate| > CAR_FOLLOW_MAX),
     * and it used to grant the altitude for free — the exact "glue
     * mints potential energy" leak.  The lift now costs its potential
     * energy out of the along-slope speed (vz^2 -= 2*g*dy, sign kept;
     * no new constant), so riding the constraint up a wall consumes
     * kinetic energy at exactly the honest exchange rate. */
    {
        /* Reuse this tick's end-position winner. Re-probing after the
         * ballistic y update can move a descending chassis just outside the
         * strict +/-3 m face band and incorrectly fall back to terrain,
         * tunnelling through the decoded top it crossed during this tick. */
        double h_c = end_ground_h;
        if (s_lv.y < h_c) {
            double dy = h_c - s_lv.y;
            s_step_diag.lift_dy = dy;
            s_step_diag.lift_dv2 = 2.0 * CAR_GRAVITY * dy;
            s_step_diag.lift_energy_j = m * CAR_GRAVITY * dy;
            double v2 = s_lv.vz * s_lv.vz - 2.0 * CAR_GRAVITY * dy;
            s_lv.vz = v2 > 0.0 ? copysign(sqrt(v2), s_lv.vz) : 0.0;
            s_lv.y = h_c;
            if (s_lv.vy < 0.0) s_lv.vy = 0.0;
        }
    }

    /* --- 7. collision pass (spec §7 step 7) --------------------------- */
    /* Horizontal push-out against registered object colliders (car.h
     * car_set_colliders). No colliders registered -> no-op, so the sim
     * tapes and every world-less probe are bit-identical to before. */
    if (s_colliders && s_ncolliders > 0) {
        double cr = s_cfg.coll_radius > 0.0f ? (double)s_cfg.coll_radius : 1.0;
        for (int i = 0; i < s_ncolliders; i++) {
            const CarCollider *b = &s_colliders[i];
            s_perf_counters.static_candidates++;
            s_step_diag.collider_candidates++;

            /* Height test (D19): skip anything the car body does not
             * vertically overlap — drive under a raised span. */
            if (b->y1 < s_lv.y || b->y0 > s_lv.y + CAR_COLLIDE_H)
                continue;

            /* Car centre in the box's own frame. */
            double dx = s_lv.x - b->x, dz = s_lv.z - b->z;
            double lx =  dx * b->ax + dz * b->az;
            double lz = -dx * b->az + dz * b->ax;

            /* Closest point on the box to the car centre. */
            double qx = lx < -b->hx ? -b->hx : (lx > b->hx ? b->hx : lx);
            double qz = lz < -b->hz ? -b->hz : (lz > b->hz ? b->hz : lz);
            double ex = lx - qx, ez = lz - qz;
            double e2 = ex * ex + ez * ez;

            /* A registered upward mesh face owns the TOP contact instead of
             * this part's coarse OBB. Test the exact closest footprint point:
             * once the chassis is at or above the face's three-metre mount
             * band, leave the top to the decoded PIP/plane probe. There is no
             * upper cutoff: an airborne car above that local face must clear
             * the coarse box rather than hit its side wall in mid-air. Side
             * and underside approaches below the mount band retain the OBB. */
            {
                double qwx = b->x + qx * b->ax - qz * b->az;
                double qwz = b->z + qx * b->az + qz * b->ax;
                double top_h, top_n[3];
                int top = terrain_drivable_probe(qwx, s_lv.y, qwz,
                                                  &top_h, top_n) == 1 &&
                          s_lv.y > top_h - 3.0;
                /* A coarse part OBB can begin behind the actual ramp face.
                 * Native collision owns mesh polygons; test the car hull's
                 * leading point too, so its circle can reach that face before
                 * the approximate box blocks the chassis centre. */
                if (!top) {
                    double cyc = cos(s_lv.yaw), syc = sin(s_lv.yaw);
                    double vwx = -syc * s_lv.vz + cyc * s_lv.vx;
                    double vwz =  cyc * s_lv.vz + syc * s_lv.vx;
                    double vl = hypot(vwx, vwz);
                    if (vl > 1e-9) {
                        double cr = s_cfg.coll_radius > 0.0f
                                  ? (double)s_cfg.coll_radius : 1.0;
                        double lead_x = s_lv.x + vwx / vl * cr;
                        double lead_z = s_lv.z + vwz / vl * cr;
                        top = terrain_drivable_probe(lead_x, s_lv.y, lead_z,
                                                     &top_h, top_n) == 1 &&
                              s_lv.y > top_h - 3.0;
                        /* A tangent circle point can remain just outside the
                         * parent deck after f32 transforms. For a collider
                         * whose own structure has registered top faces, probe
                         * 5 cm through and to either side—but only against
                         * that parent token. An unrelated deck can never make
                         * a genuine rail/wall collider yield here. */
                        if (!top && b->drivable_parent > 0) {
                            lead_x = s_lv.x + vwx / vl * (cr + 0.05);
                            lead_z = s_lv.z + vwz / vl * (cr + 0.05);
                            for (int side = -1; !top && side <= 1; side++) {
                                double side_x = lead_x - vwz / vl * 0.05 * side;
                                double side_z = lead_z + vwx / vl * 0.05 * side;
                                top = terrain_drivable_object_probe(
                                          b->drivable_parent, side_x, s_lv.y,
                                          side_z, &top_h, top_n) == 1 &&
                                      s_lv.y > top_h - 3.0;
                            }
                        }
                    }
                }
                /* A coarse OBB may overhang the real top polygon, leaving no
                 * PIP sample at its expanded wall (P09 aaramp7 is the corpus
                 * case). Only a collider explicitly associated with this
                 * drivable registry object may use the nearest polygon edge,
                 * and only when the chassis is inside or above the native
                 * three-metre mount band at that edge. A lower side approach
                 * still blocks. */
                int nearest_rc = 0;
                if (!top && b->drivable_object > 0)
                    nearest_rc = terrain_drivable_nearest_height(
                        b->drivable_object, qwx, qwz, &top_h);
                if (!top && nearest_rc == 1 && s_lv.y > top_h - 3.0)
                    top = 1;
                if (top) continue;
            }

            /* Resolve contacts, do NOT eject. The outside path pushes out
             * only while the car centre is still OUTSIDE the box, which is
             * enough to stop an approach (the contact fires cr metres out,
             * before the centre can enter).
             *
             * A centre already INSIDE the box cannot use the closest-point
             * normal (it degenerates to zero), and leaving the car embedded
             * lets it keep driving and exit through the far side — one fast
             * tick tunnels straight through a building. Recover instead by
             * separating along the NEAREST FACE: the minimum-translation
             * exit, consistent with car_apply_vehicle_contact's positional
             * correction + closing-velocity removal. The face scan order
             * (+x, -x, +z, -z, strict <) makes exact ties deterministic.
             *
             * Nearest-face is also why the spawn-overlap case stays cheap:
             * N01 starts the car at (2997.5,48692.5), inside the bunker's
             * box (x 2990..3010, z 48685..48710). The old circle collider
             * hid that with its 20 m cap, and an arbitrary ejection of
             * ~12 m moved the camera enough to fail the gpu_scene
             * silhouette gate; the minimal face exit moves it only as far
             * as the nearest wall. */
            double nlx, nlz;
            if (e2 <= 1e-12) {
                /* centre inside (or exactly on) the box — nearest face */
                double dpx = b->hx - lx, dnx = b->hx + lx;
                double dpz = b->hz - lz, dnz = b->hz + lz;
                double dmin = dpx; nlx = 1.0; nlz = 0.0;
                if (dnx < dmin) { dmin = dnx; nlx = -1.0; nlz = 0.0; }
                if (dpz < dmin) { dmin = dpz; nlx = 0.0;  nlz = 1.0; }
                if (dnz < dmin) { dmin = dnz; nlx = 0.0;  nlz = -1.0; }
                lx += nlx * (dmin + cr);
                lz += nlz * (dmin + cr);
            } else {
                if (e2 >= cr * cr) continue;    /* not touching */
                double e = sqrt(e2);
                nlx = ex / e; nlz = ez / e;
                lx = qx + nlx * cr;
                lz = qz + nlz * cr;
            }
            /* Telemetry promises an actual OBB response, not merely a box
             * whose height span was considered before the distance reject. */
            s_step_diag.collider_index = i;

            /* Back to world: box +x is (ax,az), box +z is (-az,ax). */
            s_lv.x = b->x + lx * b->ax - lz * b->az;
            s_lv.z = b->z + lx * b->az + lz * b->ax;
            double nx = nlx * b->ax - nlz * b->az;
            double nz = nlx * b->az + nlz * b->ax;

            /* Remove the inward velocity component; the tangential part
             * survives so the car slides along the obstacle. s_lv.vx/vz are
             * BODY-frame (vz forward, vx lateral — see the integration
             * above), so round-trip through world space (D7). */
            double cyv = cos(s_lv.yaw), syv = sin(s_lv.yaw);
            double vwx = -syv * s_lv.vz + cyv * s_lv.vx;
            double vwz =  cyv * s_lv.vz + syv * s_lv.vx;
            double vn = vwx * nx + vwz * nz;
            if (vn < 0.0) {
                vwx -= vn * nx;
                vwz -= vn * nz;
                s_lv.vz = -syv * vwx + cyv * vwz;
                s_lv.vx =  cyv * vwx + syv * vwz;
            }
        }
    }

    /* Drivable-area clamp. Kill the outward velocity component too, so the
     * car rests against the boundary instead of grinding into it at full
     * throttle with the speedo still reading. */
    if (s_lv.x < s_bx0 || s_lv.x > s_bx1 || s_lv.z < s_bz0 || s_lv.z > s_bz1) {
        double cyb = cos(s_lv.yaw), syb = sin(s_lv.yaw);
        double vwx = -syb * s_lv.vz + cyb * s_lv.vx;
        double vwz =  cyb * s_lv.vz + syb * s_lv.vx;
        if (s_lv.x < s_bx0) { s_lv.x = s_bx0; if (vwx < 0.0) vwx = 0.0; }
        if (s_lv.x > s_bx1) { s_lv.x = s_bx1; if (vwx > 0.0) vwx = 0.0; }
        if (s_lv.z < s_bz0) { s_lv.z = s_bz0; if (vwz < 0.0) vwz = 0.0; }
        if (s_lv.z > s_bz1) { s_lv.z = s_bz1; if (vwz > 0.0) vwz = 0.0; }
        s_lv.vz = -syb * vwx + cyb * vwz;
        s_lv.vx =  cyb * vwx + syb * vwz;
    }

    /* --- 8. terrain-aligned attitude (FACT §Q10): lerp 0.1–0.25 per --- */
    /* tick toward the terrain-normal attitude; pitch ±pi/2, roll ±pi/4  */
    /* clamps (@0x4c1d28..0x4c1d3c). Airborne rates were integrated above */
    if (contact) {
        double pitch_t, roll_t;
        terrain_attitude(&pitch_t, &roll_t);
        double dp = (pitch_t - s_lv.pitch) * CAR_ATT_LERP;
        double dr = (roll_t - s_lv.roll) * CAR_ATT_LERP;
        s_lv.pitch = clampd(s_lv.pitch + dp, -CAR_PITCH_MAX,
                            CAR_PITCH_MAX);
        s_lv.roll  = clampd(s_lv.roll + dr, -CAR_ROLL_MAX, CAR_ROLL_MAX);
        /* liftoff rate cap retained (DECISION; the original's airborne
         * attitude-rate sourcing is not fully resolved, doc §8) */
        s_lv.pitch_rate = clampd(dp / dt, -CAR_ATT_RATE_MAX,
                                 CAR_ATT_RATE_MAX);
        s_lv.roll_rate  = clampd(dr / dt, -CAR_ATT_RATE_MAX,
                                 CAR_ATT_RATE_MAX);
    }

    s_step_diag.resolved_x = s_lv.x;
    s_step_diag.resolved_y = s_lv.y;
    s_step_diag.resolved_z = s_lv.z;

    /* --- 9. events & tape hook ---------------------------------------- */
    /* Landing/fall counters are updated inline above (§Q2). The host bridge
     * consumes hard landings; FSM-visible ram/shot/dead events are latched by
     * the mission/FSM slice. Per-tick tape recording lives in
     * tools/car_probe.c (tape-format.md §2). */
    s_lv.t += dt;
}

/* ----------------------------------------------------------------------- */
/* Explicit contexts + player-context-0 compatibility wrappers.             */
/* ----------------------------------------------------------------------- */
static void presentation_clear(CarPresentationCache *cache)
{
    if (!cache) return;
    free(cache->parts);
    free(cache->fparts);
    memset(cache, 0, sizeof *cache);
}

CarSimContext *car_context_create(void)
{
    CarSimContext *ctx = malloc(sizeof *ctx);
    if (ctx) context_defaults(ctx);
    return ctx;
}

void car_context_destroy(CarSimContext *ctx)
{
    if (!ctx || ctx == &s_player_context) return;
    free(ctx);
}

int car_context_load(CarSimContext *ctx, const char *vcf_name)
{
    if (!ctx || !vcf_name) return -1;
    CarPresentationCache scratch = {0};
    CarBinding old = context_bind(ctx, &scratch);
    int rc = context_load_bound(vcf_name);
    context_restore(old);
    presentation_clear(&scratch);
    return rc;
}

void car_context_unload(CarSimContext *ctx)
{
    if (!ctx || ctx == &s_player_context) return;
    CarPresentationCache scratch = {0};
    CarBinding old = context_bind(ctx, &scratch);
    context_unload_bound();
    context_restore(old);
}

int car_context_is_loaded(const CarSimContext *ctx)
{
    return ctx && ctx->loaded;
}

void car_context_place(CarSimContext *ctx, double x, double z, double yaw)
{
    if (!ctx) return;
    CarPresentationCache scratch = {0};
    CarPresentationCache *cache = ctx == &s_player_context
                                ? &s_player_presentation : &scratch;
    CarBinding old = context_bind(ctx, cache);
    context_place_bound(x, z, yaw);
    context_restore(old);
}

void car_context_set_scripted_pose(CarSimContext *ctx, double x, double z,
                                   double yaw, double speed)
{
    if (!ctx) return;
    CarPresentationCache scratch = {0};
    CarPresentationCache *cache = ctx == &s_player_context
                                ? &s_player_presentation : &scratch;
    CarBinding old = context_bind(ctx, cache);
    context_set_scripted_pose_bound(x, z, yaw, speed);
    context_restore(old);
}

void car_context_step(CarSimContext *ctx, CarInput *in)
{
    if (!ctx) return;
    CarPresentationCache scratch = {0};
    CarPresentationCache *cache = ctx == &s_player_context
                                ? &s_player_presentation : &scratch;
    CarBinding old = context_bind(ctx, cache);
    context_step_bound(in);
    context_restore(old);
}

void car_context_get_live(const CarSimContext *ctx, CarLive *out)
{
    if (ctx && out) memcpy(out, &ctx->live, sizeof *out);
}

void car_context_set_live(CarSimContext *ctx, const CarLive *in)
{
    if (!ctx || !in) return;
    memcpy(&ctx->live, in, sizeof ctx->live);
    ctx->transient.slope_valid = 0;
}

void car_context_get_step_diag(const CarSimContext *ctx, CarStepDiag *out)
{
    if (ctx && out) memcpy(out, &ctx->transient.step_diag, sizeof *out);
}

void car_perf_counters_reset(void)
{
    memset(&s_perf_counters, 0, sizeof s_perf_counters);
}

void car_perf_counters_get(CarPerfCounters *out)
{
    if (out) *out = s_perf_counters;
}

static uint64_t context_hash_bytes(uint64_t h, const void *data, size_t n)
{
    const uint8_t *p = data;
    while (n--) {
        h ^= *p++;
        h *= UINT64_C(1099511628211);
    }
    return h;
}

uint64_t car_context_state_hash(const CarSimContext *ctx)
{
    if (!ctx) return 0;
    uint64_t h = UINT64_C(1469598103934665603);
#define HASH_VALUE(v) (h = context_hash_bytes(h, &(v), sizeof(v)))
    h = context_hash_bytes(h, &ctx->config, sizeof ctx->config);
    h = context_hash_bytes(h, &ctx->live, sizeof ctx->live);
    h = context_hash_bytes(h, &ctx->transient.step_diag,
                           sizeof ctx->transient.step_diag);
    HASH_VALUE(ctx->loaded);
    HASH_VALUE(ctx->transient.hard_landings);
    HASH_VALUE(ctx->transient.fall_resets);
    HASH_VALUE(ctx->transient.last_impact);
    HASH_VALUE(ctx->transient.surf_grip);
    HASH_VALUE(ctx->transient.surf_rr);
    HASH_VALUE(ctx->transient.surf_impact);
    HASH_VALUE(ctx->transient.last_impact_scale);
    HASH_VALUE(ctx->transient.grip_loss_scale);
    HASH_VALUE(ctx->transient.grip_loss_ticks);
    HASH_VALUE(ctx->transient.surf_have);
    HASH_VALUE(ctx->transient.surf_class);
    HASH_VALUE(ctx->transient.slope_prev);
    HASH_VALUE(ctx->transient.slope_r_prev);
    HASH_VALUE(ctx->transient.slope_sgn);
    HASH_VALUE(ctx->transient.slope_valid);
    HASH_VALUE(ctx->transient.mass_kg);
    HASH_VALUE(ctx->transient.ncolliders);
    HASH_VALUE(ctx->transient.bx0);
    HASH_VALUE(ctx->transient.bz0);
    HASH_VALUE(ctx->transient.bx1);
    HASH_VALUE(ctx->transient.bz1);
    if (ctx->transient.colliders)
        for (int i = 0; i < ctx->transient.ncolliders; i++) {
            const CarCollider *c = &ctx->transient.colliders[i];
            HASH_VALUE(c->x); HASH_VALUE(c->z);
            HASH_VALUE(c->hx); HASH_VALUE(c->hz);
            HASH_VALUE(c->ax); HASH_VALUE(c->az);
            HASH_VALUE(c->y0); HASH_VALUE(c->y1);
            HASH_VALUE(c->drivable_object);
            HASH_VALUE(c->drivable_parent);
        }
#undef HASH_VALUE
    return h;
}

void car_context_set_colliders(CarSimContext *ctx,
                               const CarCollider *list, int n)
{
    if (!ctx) return;
    ctx->transient.colliders = n > 0 ? list : NULL;
    ctx->transient.ncolliders = n > 0 ? n : 0;
}

void car_context_set_bounds(CarSimContext *ctx, double x0, double z0,
                            double x1, double z1)
{
    if (!ctx) return;
    CarPresentationCache scratch = {0};
    CarPresentationCache *cache = ctx == &s_player_context
                                ? &s_player_presentation : &scratch;
    CarBinding old = context_bind(ctx, cache);
    context_set_bounds_bound(x0, z0, x1, z1);
    context_restore(old);
}

void car_context_apply_grip_loss(CarSimContext *ctx, double scale, int ticks)
{
    if (!ctx) return;
    CarPresentationCache scratch = {0};
    CarPresentationCache *cache = ctx == &s_player_context
                                ? &s_player_presentation : &scratch;
    CarBinding old = context_bind(ctx, cache);
    context_apply_grip_loss_bound(scale, ticks);
    context_restore(old);
}

int car_context_grip_loss_ticks(const CarSimContext *ctx)
{
    return ctx ? ctx->transient.grip_loss_ticks : 0;
}

double car_context_effective_grip(const CarSimContext *ctx)
{
    return ctx ? ctx->transient.surf_grip : 1.0;
}

double car_context_collision_radius(const CarSimContext *ctx)
{
    if (!ctx) return 1.0;
    return ctx->config.coll_radius > 0.0f
         ? (double)ctx->config.coll_radius : 1.0;
}

void car_context_world_velocity(const CarSimContext *ctx,
                                double *vx, double *vz)
{
    double wx = 0.0, wz = 0.0;
    if (ctx && ctx->loaded) {
        double cy = cos(ctx->live.yaw), sy = sin(ctx->live.yaw);
        wx = -sy * ctx->live.vz + cy * ctx->live.vx;
        wz =  cy * ctx->live.vz + sy * ctx->live.vx;
    }
    if (vx) *vx = wx;
    if (vz) *vz = wz;
}

void car_context_add_world_velocity(CarSimContext *ctx,
                                    double dvx, double dvz)
{
    if (!ctx || !ctx->loaded || !isfinite(dvx) || !isfinite(dvz)) return;
    double cy = cos(ctx->live.yaw), sy = sin(ctx->live.yaw);
    ctx->live.vz += -sy * dvx + cy * dvz;
    ctx->live.vx +=  cy * dvx + sy * dvz;
}

void car_context_apply_vehicle_contact(CarSimContext *ctx,
                                       double dx, double dz,
                                       double nx, double nz,
                                       double other_vx, double other_vz)
{
    if (!ctx) return;
    CarPresentationCache scratch = {0};
    CarPresentationCache *cache = ctx == &s_player_context
                                ? &s_player_presentation : &scratch;
    CarBinding old = context_bind(ctx, cache);
    context_apply_vehicle_contact_bound(dx, dz, nx, nz, other_vx, other_vz);
    context_restore(old);
}

void car_context_landing_events(const CarSimContext *ctx,
                                int *hard_landings, int *fall_resets,
                                double *last_impact)
{
    if (!ctx) return;
    if (hard_landings) *hard_landings = ctx->transient.hard_landings;
    if (fall_resets) *fall_resets = ctx->transient.fall_resets;
    if (last_impact) *last_impact = ctx->transient.last_impact;
}

double car_context_last_impact_scale(const CarSimContext *ctx)
{
    return ctx ? ctx->transient.last_impact_scale : 1.0;
}

int car_load(const char *vcf_name)
{
    CarBinding old = context_bind(&s_player_context, &s_player_presentation);
    int rc = context_load_bound(vcf_name);
    context_restore(old);
    return rc;
}

void car_unload(void)
{
    CarBinding old = context_bind(&s_player_context, &s_player_presentation);
    context_unload_bound();
    context_restore(old);
}

int car_is_loaded(void) { return s_player_context.loaded; }

double car_settled_y_at(double x, double z)
{
    CarBinding old = context_bind(&s_player_context, &s_player_presentation);
    double y = settled_y_at_bound(x, z);
    context_restore(old);
    return y;
}

void car_place(double x, double z, double yaw)
{
    car_context_place(&s_player_context, x, z, yaw);
}

void car_set_scripted_pose(double x, double z, double yaw, double speed)
{
    car_context_set_scripted_pose(&s_player_context, x, z, yaw, speed);
}

void car_step(CarInput *in)
{
    car_context_step(&s_player_context, in);
}

void car_set_bounds(double x0, double z0, double x1, double z1)
{
    car_context_set_bounds(&s_player_context, x0, z0, x1, z1);
}

void car_set_colliders(const CarCollider *list, int n)
{
    car_context_set_colliders(&s_player_context, list, n);
}

double car_collision_radius(void)
{
    return car_context_collision_radius(&s_player_context);
}

void car_world_velocity(double *vx, double *vz)
{
    car_context_world_velocity(&s_player_context, vx, vz);
}

void car_add_world_velocity(double dvx, double dvz)
{
    car_context_add_world_velocity(&s_player_context, dvx, dvz);
}

void car_apply_vehicle_contact(double dx, double dz,
                               double nx, double nz,
                               double other_vx, double other_vz)
{
    car_context_apply_vehicle_contact(&s_player_context, dx, dz, nx, nz,
                                      other_vx, other_vz);
}

void car_apply_grip_loss(double scale, int ticks)
{
    car_context_apply_grip_loss(&s_player_context, scale, ticks);
}

int car_grip_loss_ticks(void)
{
    return car_context_grip_loss_ticks(&s_player_context);
}

double car_effective_grip(void)
{
    return car_context_effective_grip(&s_player_context);
}

/* ----------------------------------------------------------------------- */
/* Getters                                                                  */
/* ----------------------------------------------------------------------- */
void car_get_step_diag(CarStepDiag *out)
{
    car_context_get_step_diag(&s_player_context, out);
}

uint64_t car_state_hash(void)
{
    return car_context_state_hash(&s_player_context);
}

void car_pose(double *x, double *y, double *z,
              double *yaw, double *pitch, double *roll)
{
    bind_player_context();
    if (x) *x = s_lv.x;
    if (y) *y = s_lv.y;
    if (z) *z = s_lv.z;
    if (yaw) *yaw = s_lv.yaw;
    if (pitch) *pitch = s_lv.pitch;
    if (roll) *roll = s_lv.roll;
}

const char *car_vtf_file(void)
{
    bind_player_context();
    return s_cfg.vtf_file;
}

const char *car_chassis_name(void)
{
    bind_player_context();
    return s_loaded ? s_cfg.chassis_name : "";
}

const char *car_variant_name(void)
{
    bind_player_context();
    return s_loaded ? s_cfg.variant : "";
}

int car_component_ids(uint32_t out3[3])
{
    bind_player_context();
    if (!s_loaded || !out3) return -1;
    out3[0] = s_cfg.engine_type;
    out3[1] = s_cfg.susp_type;
    out3[2] = s_cfg.brake_type;
    return 0;
}

int car_defense(uint32_t armor4[4], uint32_t chassis4[4],
                uint32_t *left_to_add)
{
    bind_player_context();
    if (!s_loaded) return -1;
    if (armor4) memcpy(armor4, s_cfg.armor, sizeof s_cfg.armor);
    if (chassis4) memcpy(chassis4, s_cfg.chassis, sizeof s_cfg.chassis);
    if (left_to_add) *left_to_add = s_cfg.left_to_add;
    return 0;
}

double car_config_weight_lb(void)
{
    bind_player_context();
    if (!s_loaded) return 0.0;
    double total = s_cfg.mass;
    for (int i = 0; i < s_cfg.nweapons; i++) total += s_cfg.weapons[i].mass;
    return total;
}

int car_special_count(void)
{
    bind_player_context();
    return s_loaded ? s_cfg.nspecials : 0;
}

int car_special_id(int i)
{
    bind_player_context();
    return s_loaded && i >= 0 && i < s_cfg.nspecials ? s_cfg.specials[i] : 0;
}

const char *car_wheel_file(int axle)
{
    bind_player_context();
    return s_loaded && axle >= 0 && axle < 3 ? s_cfg.wdf_file[axle] : "";
}

double car_speed(void)
{
    return s_player_context.live.vz;
}

int car_is_reverse(void)
{
    return s_player_context.live.reverse != 0;
}

/* Current gear (1..4) and engine RPM — FACT §Q5 model state, exposed
 * for the probe's accel-curve/gearbox reporting. */
int car_gear(void) { return s_player_context.live.gear; }
double car_rpm(void) { return s_player_context.live.engine_rpm; }

int car_engine_curve(uint32_t *component_id, double *tpeak, double *k,
                     int *authored)
{
    bind_player_context();
    if (!s_loaded) return -1;
    if (component_id) *component_id = s_cfg.engine_curve_id;
    if (tpeak) *tpeak = (double)s_cfg.engine_tpeak;
    if (k) *k = (double)s_cfg.engine_k;
    if (authored) *authored = s_cfg.engine_curve_authored;
    return 0;
}

int car_engine_sound_number(void)
{
    bind_player_context();
    if (!s_loaded) return -1;

    switch (s_cfg.veh_size) {
    case 1: return 2;
    case 2: return 1;
    case 3: return 0;
    case 4: return 3;
    case 5: return 5;
    case 6: return 4;
    default: return -1;
    }
}

/* Weapon stat accessors (M7 combat player-fire hook; additive — the
 * parsed WEPN/GDF stats were already kept (D9); this only exposes
 * them. The sim itself never reads them. */
/* Weapon/gameplay/presentation accessors. All string pointers remain owned by
 * car.c and valid until car_unload. */
/* GDFC rates are stored as frequency denominators (Open76 parser: 1/x).
 * Convert either rate to the fixed 20 Hz tick. */
static int weapon_cooldown_ticks(float rate)
{
    int t = rate > 0.0f ? (int)ceil(20.0 / (double)rate) : 0;
    return t < 1 ? 1 : t;
}
int car_weapon_count(void)
{
    bind_player_context();
    return s_loaded ? s_cfg.nweapons : 0;
}

/* LD-WPN PUBLISHED-SOURCE ranges by GDFC family/tier. Families 2/4/5
 * reproduce the Tier-1 Local Ditch Range(m) chart; they are not GDF fields.
 * Only FUN_00401610's 1 km family gate is binary-decoded (family 3 and the
 * unresolved tier-15 fallback). */
static double weapon_range_m(int family, int tier)
{
    switch (family) {
    case 2: { /* machine guns + cannons; turret tiers carry +100 */
        if (tier >= 100) tier -= 100;
        static const double r[] = {150, 300, 500, 150, 300, 500, 600};
        if (tier >= 0 && tier < (int)(sizeof r / sizeof r[0]))
            return r[tier];
        /* Tank/Police cannon uses tier 15; no decoded range field exists.
         * Keep it eligible under the native decision's recovered 1000 m
         * outer family gate rather than silently disarming a boss gun. */
        return tier == 15 ? 1000.0 : 0.0;
    }
    case 3: return 1000.0;               /* artillery decision branch */
    case 4: { /* dumb/heat/radar/Cherub missiles */
        if (tier >= 100) tier -= 100;
        static const double r[] = {1000, 2000, 3000, 4000};
        return tier >= 0 && tier < (int)(sizeof r / sizeof r[0])
             ? r[tier] : 0.0;
    }
    case 5: { /* flame family */
        if (tier >= 100) tier -= 100;
        static const double r[] = {40, 35, 30, 30};
        return tier >= 0 && tier < (int)(sizeof r / sizeof r[0])
             ? r[tier] : 0.0;
    }
    default: return 0.0;
    }
}

/* Side-effect-free VCF combat query. It deliberately duplicates only the
 * small, documented VCFC/HLOC/WEPN/GDFC subset: calling the player car_load()
 * wrapper here would replace context 0 while registering each NPC. */
int car_combat_config(const char *vcf_name, CarCombatConfig *out)
{
    typedef struct {
        uint32_t facing, mesh_type;
        double frame[12];
    } Mount;
    typedef struct { int mount; char gdf[14]; } WeaponRef;
    Mount mounts[CAR_MAX_HLOCS] = {{0}};
    WeaponRef refs[CAR_MAX_WEAPONS] = {{0}};
    int nmounts = 0, nrefs = 0, have_vcfc = 0;
    char path[64];
    size_t sz = 0;
    uint8_t *buf;

    if (!vcf_name || !*vcf_name || !out)
        return -1;
    memset(out, 0, sizeof *out);
    copy_str(path, sizeof path, vcf_name);
    if (!strchr(path, '.')) {
        size_t l = strlen(path);
        if (l + 4 >= sizeof path)
            return -1;
        memcpy(path + l, ".vcf", 5);
    }
    buf = vfs_read_file(path, &sz);
    if (!buf)
        return -1;
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        const uint8_t *p = buf + c.payload;
        size_t n = c.total - 8;
        if (tag_is(&c, "VCFC") && n >= 129) {
            for (int i = 0; i < 4; i++) {
                out->armor[i] = rd_u32(p + 93 + (size_t)i * 4);
                out->chassis[i] = rd_u32(p + 109 + (size_t)i * 4);
            }
            have_vcfc = 1;
        } else if (tag_is(&c, "WEPN")) {
            size_t nr = n / 17;
            for (size_t i = 0; i < nr && nrefs < CAR_MAX_WEAPONS; i++) {
                refs[nrefs].mount = (int)rd_i32(p + i * 17);
                copy_field(refs[nrefs].gdf, sizeof refs[nrefs].gdf,
                           p + i * 17 + 4, 13);
                nrefs++;
            }
        }
        off = c.next;
    }
    vfs_free(buf);
    if (!have_vcfc)
        return -1;

    /* HLOC belongs to the VDF named by VCFC. Read that name once without
     * retaining any of car_load's geometry/physics state. */
    buf = vfs_read_file(path, &sz);
    if (!buf) return -1;
    char vdf[14] = {0};
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        if (tag_is(&c, "VCFC") && c.total - 8 >= 29) {
            copy_field(vdf, sizeof vdf, buf + c.payload + 16, 13);
            break;
        }
        off = c.next;
    }
    vfs_free(buf);
    if (name_is_null(vdf))
        return -1;
    buf = vfs_read_file(vdf, &sz);
    if (!buf) return -1;
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        if (tag_is(&c, "COLP") && c.total - 8 >= 48) {
            const uint8_t *p = buf + c.payload;
            /* COLP rows are Z, X, Y; each row starts maxOuter and ends
             * minOuter. Keep authored target extents for projectile hits. */
            out->collision_half[2] = fmax(fabs((double)rd_f32(p)),
                                           fabs((double)rd_f32(p + 12)));
            out->collision_half[0] = fmax(fabs((double)rd_f32(p + 16)),
                                           fabs((double)rd_f32(p + 28)));
            out->collision_half[1] = fmax(fabs((double)rd_f32(p + 32)),
                                           fabs((double)rd_f32(p + 44)));
        } else if (tag_is(&c, "HLOC") && c.total - 8 >= 80 &&
                   nmounts < CAR_MAX_HLOCS) {
            const uint8_t *p = buf + c.payload;
            mounts[nmounts].facing = rd_u32(p + 20);
            mounts[nmounts].mesh_type = rd_u32(p + 24);
            for (int k = 0; k < 12; k++)
                mounts[nmounts].frame[k] =
                    (double)rd_f32(p + 28 + (size_t)k * 4);
            nmounts++;
        }
        off = c.next;
    }
    vfs_free(buf);

    for (int i = 0; i < nrefs && out->weapon_count < CAR_COMBAT_WEAPONS; i++) {
        int m = refs[i].mount;
        if (m < 0 || m >= nmounts ||
            !(mounts[m].mesh_type == 1 || mounts[m].mesh_type == 2 ||
              mounts[m].mesh_type == 3 || mounts[m].mesh_type == 4 ||
              mounts[m].mesh_type == 5))
            continue;
        buf = vfs_read_file(refs[i].gdf, &sz);
        if (!buf) continue;
        CarCombatWeapon parsed = {0};
        int direct = 0, point_index = -1;
        double points[4][12] = {{0}};
        int have_points = 0;
        switch (mounts[m].mesh_type) {
        case 1: point_index = 0; break;
        case 2: point_index = 1; break;
        case 3: point_index = 2; break;
        case 5: point_index = 3; break;
        default: break;
        }
        for (size_t off = 0; off < sz; ) {
            Chunk c;
            if (!chunk_at(buf, sz, off, &c)) break;
            if (tag_is(&c, "GDFC") && c.total - 8 >= 128) {
                const uint8_t *p = buf + c.payload;
                parsed.damage = (int)rd_i32(p + 44);
                parsed.deploy_kind = mounts[m].mesh_type == 4
                                   ? dropper_kind_from_gdf(refs[i].gdf)
                                   : CAR_DEPLOY_NONE;
                if (parsed.damage > 0 || parsed.deploy_kind != CAR_DEPLOY_NONE) {
                    int tier = (int)rd_i32(p + 20);
                    /* FACT aim-convergence.md §2.2: traversal uses the
                     * effective mount class. The authored HLOC class wins
                     * below tier 100; tier 100+ forces class 3. */
                    parsed.traverses = mounts[m].mesh_type == 3 || tier >= 100;
                    parsed.tier = tier;
                    copy_field(parsed.name, sizeof parsed.name, p, 16);
                    parsed.ammo = (int)rd_i32(p + 94);
                    parsed.ammo_capacity = parsed.ammo;
                    parsed.cooldown_ticks =
                        weapon_cooldown_ticks(rd_f32(p + 78));
                    parsed.burst_cooldown_ticks = rd_f32(p + 74) > 0.0f
                        ? weapon_cooldown_ticks(rd_f32(p + 74)) : 0;
                    parsed.rear_facing = mounts[m].facing == 2;
                    parsed.fire_amount = (int)rd_i32(p + 82);
                    if (parsed.fire_amount < 1) parsed.fire_amount = 1;
                    parsed.family = (int)rd_i32(p + 16);
                    parsed.projectile_speed = (double)rd_f32(p + 86);
                    parsed.range_m = weapon_range_m(parsed.family, tier);
                    if (tier >= 100)
                        point_index = 2; /* FACT: GPOF turret override */
                    direct = 1;
                }
            } else if (tag_is(&c, "ORDF") && c.total - 8 >= 16) {
                const uint8_t *op = buf + c.payload;
                parsed.ordnance_type = (int)rd_i32(op);
                parsed.flight_speed = (double)rd_f32(op + 4);
                parsed.manager_type = (int)rd_i32(op + 12);
                if (c.total - 8 >= 124) {
                    copy_field(parsed.impact_ground,
                               sizeof parsed.impact_ground, op + 33, 13);
                    copy_field(parsed.impact_car,
                               sizeof parsed.impact_car, op + 59, 13);
                    copy_field(parsed.impact_building,
                               sizeof parsed.impact_building, op + 85, 13);
                    copy_field(parsed.impact_structure,
                               sizeof parsed.impact_structure, op + 111, 13);
                }
            } else if (tag_is(&c, "OGEO") && c.total - 8 >= 12) {
                copy_field(parsed.ordnance_model,
                           sizeof parsed.ordnance_model,
                           buf + c.payload + 4, 8);
            } else if (tag_is(&c, "GPOF") && c.total - 8 >= sizeof points) {
                const uint8_t *gp = buf + c.payload;
                for (int p = 0; p < 4; p++)
                    for (int k = 0; k < 12; k++)
                        points[p][k] = (double)rd_f32(
                            gp + (size_t)p * 48 + (size_t)k * 4);
                have_points = 1;
            }
            off = c.next;
        }
        if (direct) {
            double muzzle[12];
            if (have_points && point_index >= 0)
                frame_compose(mounts[m].frame, points[point_index], muzzle);
            else
                memcpy(muzzle, mounts[m].frame, sizeof muzzle);
            memcpy(parsed.mount_frame, mounts[m].frame,
                   sizeof parsed.mount_frame);
            memcpy(parsed.muzzle_frame, muzzle, sizeof parsed.muzzle_frame);
            parsed.muzzle[0] = muzzle[9];
            parsed.muzzle[1] = muzzle[10];
            parsed.muzzle[2] = muzzle[11];
            out->weapons[out->weapon_count++] = parsed;
        }
        vfs_free(buf);
    }
    return 0;
}

/* Manual p.31 is the authored link taxonomy: Slug Throwers, SPPs,
 * Flamethrowers, Mortars and Droppers. GDFC +90 is not that taxonomy by
 * itself (it splits cannons from MGs, flame tiers and dropper kinds), so map
 * the complete shipped category inventory explicitly. Unknown future groups
 * fail closed as unlinkable rather than joining an invented class. */
static int weapon_link_class(int group)
{
    switch (group) {
    case 1: case 6:
        return CAR_WEAPON_LINK_SLUG;
    case 2: case 3: case 8: case 20: case 21:
        return CAR_WEAPON_LINK_SPP;
    case 9: case 10: case 11:
        return CAR_WEAPON_LINK_FLAME;
    case 4: case 5: case 7: case 22:
        return CAR_WEAPON_LINK_MORTAR;
    case 12: case 14: case 15: case 16: case 17:
        return CAR_WEAPON_LINK_DROPPER;
    default:
        return CAR_WEAPON_LINK_NONE;
    }
}

int car_weapon_get(int i, CarWeaponInfo *out)
{
    bind_player_context();
    const CarWeapon *w;
    if (!s_loaded || i < 0 || i >= s_cfg.nweapons || !out)
        return -1;
    w = &s_cfg.weapons[i];
    out->name = w->name;
    out->sound = w->sound;
    out->gdf = w->gdf;
    out->fire_sprite = w->fire_sprite;
    copy_str(out->ordnance_model, sizeof out->ordnance_model,
             w->ordnance_model);
    copy_str(out->impact_ground, sizeof out->impact_ground, w->impact_ground);
    copy_str(out->impact_car, sizeof out->impact_car, w->impact_car);
    copy_str(out->impact_building, sizeof out->impact_building,
             w->impact_building);
    copy_str(out->impact_structure, sizeof out->impact_structure,
             w->impact_structure);
    out->mount = w->mount;
    out->mount_class = (int)w->mesh_type;
    out->geometry_parts = w->nparts;
    out->damage = (int)w->damage;
    out->ammo = (int)w->ammo;
    out->cooldown_ticks = weapon_cooldown_ticks(w->firing_rate);
    out->burst_cooldown_ticks = w->burst_rate > 0.0f
                              ? weapon_cooldown_ticks(w->burst_rate) : 0;
    out->rear_facing = w->facing == 2;
    out->deploy_kind = w->mesh_type == 4
                     ? dropper_kind_from_gdf(w->gdf) : CAR_DEPLOY_NONE;
    out->direct_fire = (w->damage > 0 && (w->mesh_type == 1 ||
                       w->mesh_type == 2 || w->mesh_type == 3 ||
                       w->mesh_type == 5)) || out->deploy_kind != CAR_DEPLOY_NONE;
    out->link_class = weapon_link_class((int)w->weapon_group);
    out->projectile_speed = (double)w->projectile_speed;
    out->flight_speed = w->flight_speed > 0.0f
                      ? (double)w->flight_speed
                      : (double)w->projectile_speed;
    out->ordnance_type = (int)w->ordnance_type;
    out->manager_type = (int)w->manager_type;
    /* Native FUN_004af210's effective class-3 rule is the mount datum used
     * by both live traversal and the manual p.31 link exclusion. Filename
     * spelling is not gameplay state (H-UAT-079e). */
    out->traverses = w->mesh_type == 3 || w->tier >= 100;
    out->turreted = out->traverses;
    out->tracks = w->ordnance_type == 3 || w->ordnance_type == 8 ||
                  w->ordnance_type == 0x14;
    out->range_m = weapon_range_m((int)w->family, (int)w->tier);
    /* GDFC +90; Open76 WeaponsController also offsets rear mounts by
     * +100 so front/rear of the same type never share a fire group. */
    out->weapon_group = (int)w->weapon_group +
                        (out->rear_facing ? 100 : 0);
    out->family = (int)w->family;
    out->tier = (int)w->tier;
    return 0;
}

int car_weapon_part_count(int weapon)
{
    bind_player_context();
    return s_loaded && weapon >= 0 && weapon < s_cfg.nweapons
         ? s_cfg.weapons[weapon].nparts : 0;
}

const char *car_weapon_part_name(int weapon, int part)
{
    bind_player_context();
    if (!s_loaded || weapon < 0 || weapon >= s_cfg.nweapons ||
        part < 0 || part >= s_cfg.weapons[weapon].nparts)
        return NULL;
    return s_cfg.weapons[weapon].parts[part].name;
}

int car_weapon_part_frame(int weapon, int part, double out12[12])
{
    bind_player_context();
    if (!out12 || !car_weapon_part_name(weapon, part))
        return -1;
    memcpy(out12, s_cfg.weapons[weapon].parts[part].frame,
           sizeof s_cfg.weapons[weapon].parts[part].frame);
    return 0;
}

int car_weapon_part_live_frame(int weapon, int part, double out12[12])
{
    bind_player_context();
    if (car_weapon_part_frame(weapon, part, out12) != 0)
        return -1;
    if (!(s_cfg.weapons[weapon].mesh_type == 3 ||
          s_cfg.weapons[weapon].tier >= 100))
        return 0;

    /* FUN_004af210 rotates the effective-class-3 yaw joint before the spawn
     * pass consumes it. Combat delivery owns the decoded child pitch too;
     * exterior child-joint presentation remains outside this delivery slice. */
    extern double combat_player_weapon_traverse_yaw(int source);
    double yaw = combat_player_weapon_traverse_yaw(weapon);
    if (fabs(yaw) < 1e-12)
        return 0;
    double c = cos(yaw), s = sin(yaw);
    double px = s_cfg.weapons[weapon].mount_frame[9];
    double pz = s_cfg.weapons[weapon].mount_frame[11];
    for (int col = 0; col < 3; col++) {
        int k = col * 3;
        double x = out12[k], z = out12[k + 2];
        out12[k] = c * x + s * z;
        out12[k + 2] = -s * x + c * z;
    }
    double x = out12[9] - px, z = out12[11] - pz;
    out12[9] = px + c * x + s * z;
    out12[11] = pz - s * x + c * z;
    return 0;
}

int car_weapon_mount_frame(int weapon, double out12[12])
{
    bind_player_context();
    if (!s_loaded || weapon < 0 || weapon >= s_cfg.nweapons || !out12)
        return -1;
    memcpy(out12, s_cfg.weapons[weapon].mount_frame,
           sizeof s_cfg.weapons[weapon].mount_frame);
    return 0;
}

int car_weapon_muzzle_frame(int weapon, double out12[12])
{
    bind_player_context();
    if (!s_loaded || weapon < 0 || weapon >= s_cfg.nweapons || !out12)
        return -1;
    memcpy(out12, s_cfg.weapons[weapon].muzzle_frame,
           sizeof s_cfg.weapons[weapon].muzzle_frame);
    return 0;
}

/* Landing event counters (FACT §Q2 thresholds; bridge-visible — see car.c):
 * hard landings (>7.65 m/s vertical impact, @0x4c1d58) and 35 m
 * fall-resets (@0x4c1d44) since the last car_load/car_place. */
void car_landing_events(int *hard_landings, int *fall_resets,
                        double *last_impact)
{
    car_context_landing_events(&s_player_context, hard_landings, fall_resets,
                               last_impact);
}

double car_last_impact_scale(void)
{
    return car_context_last_impact_scale(&s_player_context);
}

/* M5 save/resume snapshot hooks remain context-0 wrappers. */
void car_get_live(CarLive *out)
{
    car_context_get_live(&s_player_context, out);
}

void car_set_live(const CarLive *in)
{
    car_context_set_live(&s_player_context, in);
}

int car_part_count(void) { return s_player_presentation.nparts; }

int car_body_part_count(void) { return s_player_presentation.nbody_parts; }

int car_wheel_part_count(void) { return s_player_presentation.nwheel_parts; }

const char *car_part_name(int i)
{
    bind_player_context();
    if (i < 0 || i >= s_nparts) return NULL;
    return s_parts[i].name;
}

int car_part_frame(int i, double out12[12])
{
    bind_player_context();
    if (!out12 || i < 0 || i >= s_nparts) return -1;
    memcpy(out12, s_parts[i].frame, sizeof s_parts[i].frame);
    return 0;
}

/* Interior (first-person) part accessors — VGEO set CAR_VGEO_FP_SET (D22).
 * Same ownership/lifetime rules as car_part_*: module-owned, valid until
 * car_unload(). */
int car_interior_part_count(void) { return s_player_presentation.nfparts; }

const char *car_interior_part_name(int i)
{
    bind_player_context();
    if (i < 0 || i >= s_nfparts) return NULL;
    return s_fparts[i].name;
}

int car_interior_part_frame(int i, double out12[12])
{
    bind_player_context();
    if (!out12 || i < 0 || i >= s_nfparts) return -1;
    memcpy(out12, s_fparts[i].frame, sizeof s_fparts[i].frame);
    return 0;
}

int car_interior_mirror_part(void)
{
    bind_player_context();
    for (int i = 0; i < s_nfparts; i++) {
        const char *nm = s_fparts[i].name;
        size_t l = strlen(nm);
        if (l >= 4 && strcasecmp(nm + l - 4, "MIRI") == 0)
            return i;
    }
    return -1;
}

int car_vloc_count(void) { return s_player_presentation.nvlocs; }

int car_vloc_get(int i, uint32_t *number, double out12[12])
{
    bind_player_context();
    if (i < 0 || i >= s_nvlocs) return -1;
    if (number) *number = s_vlocs[i].number;
    if (out12) memcpy(out12, s_vlocs[i].frame, sizeof s_vlocs[i].frame);
    return 0;
}

int car_gdf_weapon_info(const char *gdf_name, CarWeaponInfo *out)
{
    /* Single-slot storage (car.h lifetime contract): overwritten by the
     * next call, never freed, safe across car_load/car_unload. */
    static struct {
        char    name[17];
        char    sound[14];
        int32_t family;
        int32_t tier;
        int32_t damage;
        int32_t ammo;
        int32_t weapon_group;
        float   burst_rate;
        float   firing_rate;
        float   projectile_speed;
        float   flight_speed;
        int32_t ordnance_type;
        int32_t manager_type;
        char    ordnance_model[9];
        char    impact_ground[14];
        char    impact_car[14];
        char    impact_building[14];
        char    impact_structure[14];
    } q;
    if (!gdf_name || !gdf_name[0] || !out) return -1;
    size_t sz = 0;
    uint8_t *buf = vfs_read_file(gdf_name, &sz);
    if (!buf) return -1;
    int found = 0;
    /* Clear optional extended ORDF fields too: 16-byte legacy records retain
     * flight/manager data and publish empty impact names, never stale bytes. */
    memset(&q, 0, sizeof q);
    q.impact_ground[0] = q.impact_car[0] = '\0';
    q.impact_building[0] = q.impact_structure[0] = '\0';
    for (size_t off = 0; off < sz; ) {
        Chunk c;
        if (!chunk_at(buf, sz, off, &c)) break;
        if (tag_is(&c, "GDFC") && c.total - 8 >= 128) {
            const uint8_t *p = buf + c.payload;
            copy_field(q.name, sizeof q.name, p, 16);
            q.family           = rd_i32(p + 16);
            q.tier             = rd_i32(p + 20);
            q.damage           = rd_i32(p + 44);
            q.burst_rate       = rd_f32(p + 74);
            q.firing_rate      = rd_f32(p + 78);
            q.projectile_speed = rd_f32(p + 86);
            q.weapon_group     = rd_i32(p + 90);
            q.ammo             = rd_i32(p + 94);
            copy_field(q.sound, sizeof q.sound, p + 115, 13);
            found = 1;
        } else if (tag_is(&c, "ORDF") && c.total - 8 >= 16) {
            const uint8_t *p = buf + c.payload;
            q.ordnance_type = rd_i32(p);
            q.flight_speed = rd_f32(p + 4);
            q.manager_type = rd_i32(p + 12);
            if (c.total - 8 >= 124) {
                copy_field(q.impact_ground, sizeof q.impact_ground,
                           p + 33, 13);
                copy_field(q.impact_car, sizeof q.impact_car, p + 59, 13);
                copy_field(q.impact_building, sizeof q.impact_building,
                           p + 85, 13);
                copy_field(q.impact_structure, sizeof q.impact_structure,
                           p + 111, 13);
            }
        } else if (tag_is(&c, "OGEO") && c.total - 8 >= 12) {
            copy_field(q.ordnance_model, sizeof q.ordnance_model,
                       buf + c.payload + 4, 8);
        }
        off = c.next;
    }
    vfs_free(buf);
    if (!found) return -1;
    out->name = q.name;
    out->sound = q.sound;
    out->gdf = gdf_name;
    out->fire_sprite = "";
    copy_str(out->ordnance_model, sizeof out->ordnance_model,
             q.ordnance_model);
    copy_str(out->impact_ground, sizeof out->impact_ground, q.impact_ground);
    copy_str(out->impact_car, sizeof out->impact_car, q.impact_car);
    copy_str(out->impact_building, sizeof out->impact_building,
             q.impact_building);
    copy_str(out->impact_structure, sizeof out->impact_structure,
             q.impact_structure);
    out->mount = -1;
    out->mount_class = -1;
    out->geometry_parts = 0;
    out->damage = (int)q.damage;
    out->ammo = (int)q.ammo;
    out->cooldown_ticks = weapon_cooldown_ticks(q.firing_rate);
    out->burst_cooldown_ticks = q.burst_rate > 0.0f
                              ? weapon_cooldown_ticks(q.burst_rate) : 0;
    out->rear_facing = 0;              /* no HLOC mount context (car.h) */
    out->deploy_kind = dropper_kind_from_gdf(gdf_name);
    out->direct_fire = q.damage > 0 || out->deploy_kind != CAR_DEPLOY_NONE;
    /* No HLOC exists in this standalone query; decoded GDFC tier still says
     * whether a future mount would force native effective class 3. */
    out->turreted = q.tier >= 100;
    out->link_class = weapon_link_class((int)q.weapon_group);
    out->projectile_speed = (double)q.projectile_speed;
    out->flight_speed = q.flight_speed > 0.0f
                      ? (double)q.flight_speed
                      : (double)q.projectile_speed;
    out->ordnance_type = (int)q.ordnance_type;
    out->manager_type = (int)q.manager_type;
    out->traverses = 0; /* standalone GDF has no mounted joint context */
    out->tracks = q.ordnance_type == 3 || q.ordnance_type == 8 ||
                  q.ordnance_type == 0x14;
    out->range_m = weapon_range_m((int)q.family, (int)q.tier);
    out->weapon_group = (int)q.weapon_group;
    out->family = (int)q.family;
    out->tier = (int)q.tier;
    return 0;
}

/* Append with snprintf semantics: w tracks the would-be total length. */
#define STAT_APPEND(...) do { \
    if ((size_t)w < n) w += snprintf(buf + w, n - (size_t)w, __VA_ARGS__); \
    else               w += snprintf(NULL, 0, __VA_ARGS__); \
} while (0)

int car_stats(char *buf, size_t n)
{
    bind_player_context();
    if (!s_loaded)
        return snprintf(buf, n, "car: none loaded");
    int w = 0;
    STAT_APPEND(
        "car: '%s' chassis='%s' (%s) vtf=%s type=%u size=%u elt=%s\n"
        "  mass=%.1f lb (%.1f kg, D10) collMult=%.3f drag=%.6f\n"
        "  wheels: wheelbase=%.3f track=%.3f wdf-raw-radius=%.3f (D1)\n"
        "  collision: radius=%.3f (COLP outer, no object collision yet, D8)\n"
        "  parts: %d total (%d body VGEO D12 + %d wheels D23)\n"
        "  interior: %d fp parts (VGEO set %d, D22) mirror=%d, %d VLOCs\n"
        "  armor F/L/R/Bk=%u/%u/%u/%u chassis F/L/R/Bk=%u/%u/%u/%u (+%u)\n"
        "  powertrain: engine=%u (curve eng%02u Tpeak=%.0f k=%.9g %s) "
        "susp=%u brakes=%u\n"
        "  drivetrain: 4-spd auto ratios 3.0/1.67/0.96/0.67 affect RPM; "
        "drive uses native raw-mass/reciprocal-speed law\n"
        "  surface: cls=%u grip=%.2f rr=%.3f impact-scale=%.0f%s "
        "(§Q15 WRLD table)\n"
        "  events: hard-landings=%d (last %.1f m/s, scale %.0f) "
        "fall-resets=%d (bridge-visible, §Q2)\n"
        "  specials: %d",
        s_cfg.variant, s_cfg.chassis_name, s_cfg.vdf_file,
        s_cfg.vtf_file, s_cfg.veh_type, s_cfg.veh_size, s_cfg.elt_file,
        (double)s_cfg.mass, (double)s_cfg.mass * CAR_LB_TO_KG,
        (double)s_cfg.coll_mult, (double)s_cfg.drag_coeff,
        (double)s_cfg.wheelbase, (double)s_cfg.track,
        (double)s_cfg.wdf_radius_raw,
        (double)s_cfg.coll_radius,
        s_nparts, s_nbody_parts, s_nwheel_parts,
        s_nfparts, CAR_VGEO_FP_SET,
        s_nfparts ? car_interior_mirror_part() : -1, s_nvlocs,
        s_cfg.armor[0], s_cfg.armor[1], s_cfg.armor[2], s_cfg.armor[3],
        s_cfg.chassis[0], s_cfg.chassis[1], s_cfg.chassis[2],
        s_cfg.chassis[3], s_cfg.left_to_add,
        s_cfg.engine_type, s_cfg.engine_curve_id,
        (double)s_cfg.engine_tpeak, (double)s_cfg.engine_k,
        s_cfg.engine_curve_authored ? "CDF" : "FALLBACK-D2",
        s_cfg.susp_type, s_cfg.brake_type,
        s_surf_class, s_surf_grip, s_surf_rr, s_surf_impact,
        s_surf_have ? "" : " FALLBACK(no table/tile)",
        s_hard_landings, s_last_impact, s_last_impact_scale,
        s_fall_resets,
        s_cfg.nspecials);
    for (int i = 0; i < s_cfg.nspecials; i++)
        STAT_APPEND(" %d", s_cfg.specials[i]);
    STAT_APPEND("\n  weapons: %d", s_cfg.nweapons);
    for (int i = 0; i < s_cfg.nweapons; i++) {
        const CarWeapon *wp = &s_cfg.weapons[i];
        STAT_APPEND("\n    [%d] %s '%s' dmg=%d ammo=%d hp=%d mass=%.1f "
                    "mesh=%u facing=%u parts=%d",
                    wp->mount, wp->gdf, wp->name, (int)wp->damage,
                    (int)wp->ammo, (int)wp->health, (double)wp->mass,
                    wp->mesh_type, wp->facing, wp->nparts);
    }
    STAT_APPEND(
        "\n  pose: x=%.2f y=%.2f z=%.2f yaw=%.3f pitch=%.3f roll=%.3f\n"
        "  vel: fwd=%.2f lat=%.2f vert=%.2f yawrate=%.3f t=%.2f\n"
        "  ctl: thr=%.2f gear=%d%s rpm=%.0f%s%s wheels=%d/6 (§6 model)",
        s_lv.x, s_lv.y, s_lv.z, s_lv.yaw, s_lv.pitch, s_lv.roll,
        s_lv.vz, s_lv.vx, s_lv.vy, s_lv.yaw_rate, s_lv.t,
        s_lv.throttle_pos, s_lv.gear, s_lv.reverse ? " R" : "",
        s_lv.engine_rpm, s_lv.handbrake ? " ebrake" : "",
        s_lv.grounded_count ? "" : " AIRBORNE",
        s_lv.grounded_count);
    return w;
}
