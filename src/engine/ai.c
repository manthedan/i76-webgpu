/*
 * ai.c — M7 story-motion mover (see ai.h for the contract + DECISIONs)
 *
 * Stage 3: the route/combat brain emits normalized drive controls at the
 * fixed 20 Hz step. Route-owned mission cars keep the established kinematic
 * actuator, while synthetic/melee and combat-owned FOLLOW hosts publish the
 * integrated pose from their car.c context. Contact and shove always enter the
 * selected pose/velocity owner exactly once.
 * goto/race route each authored leg through the confirmed Nitro 10 m-grid
 * A* (D-A13), string-pull the reconstructed route through nav-validated
 * chords (D-A14), and drive the result with a bounded-steering kinematic
 * actuator on a persistent per-agent heading (D-A15); follow steers the
 * same heading at its offset target.
 *
 * Compiled by textual inclusion from mission.c (see ai.h's build note).
 */

#include "engine/ai.h"
#include "engine/car.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* aismooth_probe unity-includes ai.c without the engine link set. It exercises
 * only brain/kinematic authority, so keep its historical asset-free build by
 * compiling the physical-context owner to inert setup/step calls there. */
#ifdef I76_AI_SHADOW_STUB
#define car_context_create() ((CarSimContext *)0)
#define car_context_destroy(ctx) ((void)(ctx))
#define car_context_load(ctx, name) ((void)(ctx), (void)(name), -1)
#define car_context_place(ctx, x, z, yaw) \
    ((void)(ctx), (void)(x), (void)(z), (void)(yaw))
#define car_context_set_scripted_pose(ctx, x, z, yaw, speed) \
    ((void)(ctx), (void)(x), (void)(z), (void)(yaw), (void)(speed))
#define car_context_step(ctx, in) ((void)(ctx), (void)(in))
#define car_context_get_live(ctx, out) ((void)(ctx), memset((out), 0, sizeof *(out)))
#define car_context_set_live(ctx, in) ((void)(ctx), (void)(in))
#define car_context_world_velocity(ctx, vx, vz) \
    ((void)(ctx), *(vx) = 0.0, *(vz) = 0.0)
#define car_context_add_world_velocity(ctx, vx, vz) \
    ((void)(ctx), (void)(vx), (void)(vz))
#define car_context_apply_vehicle_contact(ctx, dx, dz, nx, nz, ovx, ovz) \
    ((void)(ctx), (void)(dx), (void)(dz), (void)(nx), (void)(nz), \
     (void)(ovx), (void)(ovz))
#define car_context_state_hash(ctx) ((void)(ctx), UINT64_C(0))
#define car_context_set_colliders(ctx, list, n) \
    ((void)(ctx), (void)(list), (void)(n))
#define car_context_set_bounds(ctx, x0, z0, x1, z1) \
    ((void)(ctx), (void)(x0), (void)(z0), (void)(x1), (void)(z1))
#define car_context_apply_grip_loss(ctx, scale, ticks) \
    ((void)(ctx), (void)(scale), (void)(ticks))
#endif

/* ---- D-A13 grid planner constants -----------------------------------
 * CONFIRMED Nitro mechanics (FUN_0040ff30 search loop, FUN_0040f720 edge
 * costs; docs/specs/re/phase-a-ai-driver.md §4.2-3). The only non-native
 * input is AiNavSample.occupied, the port substitute for the unported
 * predictive vehicle probes. */
#define AI_NAV_GRID       10.0    /* CONFIRMED: 10 m grid               */
#define AI_NAV_GOAL_D2    121.0   /* CONFIRMED: <11 m goal region, sq.  */
#define AI_NAV_MAX_EXPAND 100     /* CONFIRMED: 100 main-loop expansions
                                   * after the (free) seed expansion    */
#define AI_NAV_MAX_NODES  1000    /* CONFIRMED: node pool cap           */
#define AI_NAV_COST_CHEAP 0.5     /* CONFIRMED: classes 2/3/4/6/7       */
#define AI_NAV_COST_OTHER 15.0    /* CONFIRMED: all other classes       */
#define AI_NAV_COST_BLOCK 1e6     /* CONFIRMED: avoid+blocked addend    */
#define AI_NAV_SQRT2      1.4142135623730951 /* CONFIRMED diagonal mult */
#define AI_NAV_STAGE      (AI_NAV_GRID * (AI_NAV_MAX_EXPAND / 2))
                                  /* PORT DECISION: native route preprocessing
                                   * supplies local targets; that complete
                                   * stage is not ported. Bound each search
                                   * to half its expansion radius so long
                                   * authored legs leave room for detours. */

#define AI_PLAN_MAX       128     /* planned-leg point cap: > the 101-node
                                   * chain a 100-expansion search can
                                   * reconstruct, plus the raw target   */
#define AI_PLAN_REPLAN_DRIFT 0.5
                                  /* PORT CONVENTION (H-UAT-067e): sub-metre
                                   * correction noise retains an under-fire
                                   * GOTO's query-validated local plan. */

#define AI_STUNT_STALL    40      /* D-A21 (DECISION): ticks (2 s) of no
                                   * progress toward the authored wp
                                   * before a teleport-set leg with a
                                   * cliff-blocked line is driven
                                   * directly as an authored stunt      */

/* FUN_00404b50 combat whisker fan (D-A24). Native steer is the [-1,1]
 * control at ent+0xe0; the sweep is ±0.25 step to the unit clamp. The
 * native what-if length is one FUN_00426330 integration; that integrator
 * is unported, so the look-ahead is the decoded 15 m approach gate. */
#define AI_WHISKER_STEP   0.25    /* FACT _DAT_004c1518 / _DAT_004c15e4 */
#define AI_WHISKER_LIMIT  1.0     /* FACT unit scale / FUN_004284a0 cap */
#define AI_WHISKER_RANGE  15.0    /* FACT _DAT_004c15c4 approach test   */

/* FUN_004152e0 init-hub dests (record 0 +0xe4). Weights are the first
 * six dwords of DAT_004c8138 + agg*0x40. */
static const int s_init_dests[6] = { 1, 3, 10, 6, 9, 8 };
static const int s_init_w[5][6] = {
    { 10, 50, 0,  25,  50, 1 },
    {  4, 60, 1,  25,  70, 1 },
    {  3, 70, 2,  15,  80, 1 },
    {  2, 80, 3,  10,  90, 1 },
    {  1, 90, 5,  50, 100, 1 }
};

/* Dest 6/8/9 pursuit-1 dest lists (FUN_004098a0 timeUp). Dest 12 is
 * not a member — termRock is not reached from these dests. */
static const int s_p1_d68[10] = { 1, 3, 10, 4, 5, 6, 8, 14, 13, 9 };
static const int s_p1_w6[5][10] = {
    { 2,  2,  0, 2, 12,  4,  1,  0, 8,  50 },
    { 2,  4,  9, 2, 10,  6,  1,  1, 7,  50 },
    { 2,  8, 10, 2,  8,  8,  1,  2, 5, 150 },
    { 2, 16, 30, 5, 12,  5,  5, 20, 3, 200 },
    { 2, 42, 80, 5,  8,  5, 20, 30, 1, 300 }
};
static const int s_p1_w8[5][10] = {
    { 2,  2,  0, 2, 15,  4,  1,  0, 4,  50 },
    { 2,  4,  9, 2, 10,  6,  1,  1, 7,  50 },
    { 2,  8, 20, 2,  8,  8,  1,  1, 5, 150 },
    { 2, 16, 80, 2, 20, 12,  1, 20, 3, 200 },
    { 2, 42, 50, 2,  8, 16,  1, 30, 1, 300 }
};
static const int s_p1_d9[7] = { 1, 3, 4, 5, 6, 8, 13 };
static const int s_p1_w9[5][7] = {
    { 2,  2, 2, 15,  4,  1, 4 },
    { 2,  4, 2, 12,  6,  1, 4 },
    { 2,  8, 2,  8,  8,  1, 1 },
    { 2, 16, 2, 12, 12, 20, 1 },
    { 2, 42, 2,  8, 16, 30, 1 }
};

/* FUN_00408ac0 DAT_004c7138, byte table, modes 0..4
 * (dest 4/6/8/5/14). Index (self_oct + (band + mode*5)*8)*8 + tgt_oct. */
static const unsigned char s_clsn_tbl[5][5][8][8] = {
    { /* mode 0 */
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  4,  5,  5,  5,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  3,  2,  2,  2,  2,  0,  0,  0 },
            {  7,  3,  2,  2,  4,  0,  0,  1 },
            {  1,  2,  2,  2,  0,  0,  0,  0 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  5,  5,  5,  4,  5 }
        },
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  4,  5,  5,  5,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  2,  2,  2,  0,  0,  0 },
            {  7,  2,  2,  2,  4,  0,  0,  0 },
            {  0,  2,  2,  2,  0,  0,  0,  0 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  5,  5,  5,  4,  5 }
        },
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  4,  4,  5,  5,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  4,  4,  4,  0,  0,  0 },
            {  7,  2,  2,  2,  4,  0,  0,  0 },
            {  0,  2,  2,  2,  4,  4,  4,  0 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  5,  5,  4,  4,  5 }
        },
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  4,  4,  5,  5,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  3,  3,  2,  2,  4,  2,  0,  1 },
            {  7,  2,  2,  2,  4,  0,  0,  0 },
            {  1,  3,  0,  0,  0,  0,  0,  1 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  5,  5,  4,  4,  5 }
        },
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  4,  4,  5,  5,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  2,  2,  2,  2,  2,  2 },
            {  3,  2,  2,  2,  4,  0,  0,  0 },
            {  0,  0,  0,  0,  0,  0,  0,  0 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  5,  5,  4,  4,  5 }
        }
    },
    { /* mode 1 */
        {
            {  7,  1,  8,  6,  6,  6,  8,  3 },
            { 10, 10,  4,  8,  8,  6,  6,  3 },
            {  3,  3,  8,  8,  8,  8,  8,  8 },
            {  2,  2,  8,  8,  8,  0,  0,  0 },
            {  7,  3,  2,  2,  8,  0,  0,  1 },
            {  0,  2,  2,  2,  8,  8,  8,  0 },
            {  1,  8,  8,  8,  8,  8,  8,  1 },
            { 10,  1,  6,  6,  6,  6,  8, 10 }
        },
        {
            {  7,  1,  8,  6,  6,  6,  8,  0 },
            { 10, 10,  8,  6,  6,  6,  6,  3 },
            {  3,  3,  8,  8,  8,  8,  8,  8 },
            {  2,  2,  8,  8,  8,  8,  0,  0 },
            {  7,  3,  2,  2,  8,  0,  0,  1 },
            {  0,  2,  2,  8,  8,  8,  8,  0 },
            {  1,  8,  8,  8,  6,  6,  8,  1 },
            { 10,  1,  6,  6,  6,  6,  8, 10 }
        },
        {
            {  7,  1,  6,  6,  6,  6,  6,  3 },
            { 10, 10,  8,  6,  6,  6,  6,  3 },
            {  3,  3,  8,  8,  8,  8,  8,  8 },
            {  2,  2,  4,  4,  4,  4,  0,  0 },
            {  7,  3,  2,  4,  4,  4,  0,  1 },
            {  0,  2,  2,  4,  4,  4,  4,  0 },
            {  1,  4,  4,  4,  6,  6,  4,  1 },
            { 10,  1,  6,  6,  6,  6,  4, 10 }
        },
        {
            {  7,  1,  5,  6,  6,  6,  5,  3 },
            { 10, 10,  5,  5,  5,  6,  6,  3 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  4,  4,  4,  4,  0,  0 },
            {  7,  3,  2,  4,  4,  4,  0,  1 },
            {  0,  2,  2,  4,  4,  4,  4,  0 },
            {  1,  4,  4,  4,  6,  6,  4,  1 },
            { 10,  1,  6,  6,  6,  6,  5, 10 }
        },
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  4,  4,  4,  4,  0,  0 },
            {  7,  3,  2,  4,  4,  4,  0,  1 },
            {  0,  2,  2,  4,  4,  4,  4,  0 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  5,  5,  5,  5,  5 }
        }
    },
    { /* mode 2 */
        {
            {  7,  2,  2,  2,  0,  0,  0,  0 },
            { 10, 10, 10, 10, 10, 10, 10, 10 },
            {  3,  3,  2,  2,  2,  2,  3,  3 },
            {  3,  2,  2,  2,  2,  2,  2,  3 },
            { 11,  0,  0,  0,  4,  2,  2,  2 },
            {  1,  1,  0,  0,  0,  0,  0,  0 },
            {  1,  1,  1,  1,  0,  0,  0,  1 },
            { 10, 10, 10, 10, 10, 10, 10, 10 }
        },
        {
            {  7,  3,  3,  3,  0,  1,  1,  1 },
            { 10, 10, 10, 10, 10, 10, 10, 10 },
            {  3,  2,  2,  2,  2,  2,  3,  3 },
            {  3,  3,  2,  2,  2,  2,  2,  3 },
            { 11,  0,  0,  0,  4,  2,  2,  2 },
            {  1,  1,  0,  0,  0,  0,  0,  0 },
            {  1,  1,  1,  1,  0,  0,  0,  1 },
            { 10, 10, 10, 10, 10, 10, 10, 10 }
        },
        {
            { 10,  3,  3,  5,  5,  5,  1,  1 },
            {  3,  3,  3,  3,  3, 10,  3,  3 },
            {  3,  3,  2,  2,  2,  2,  3,  3 },
            {  3,  2,  2,  2,  2,  2,  2,  3 },
            { 11,  0,  0,  0,  4,  2,  2,  2 },
            {  1,  1,  0,  0,  0,  0,  0,  0 },
            {  1,  1,  1,  1,  0,  0,  0,  1 },
            {  1,  1,  1, 10,  1,  1,  1,  1 }
        },
        {
            { 10,  3,  5,  5,  5,  5,  5,  1 },
            {  3,  3,  3,  3,  3,  3,  3,  3 },
            {  3,  3,  2,  2,  2,  2,  3,  3 },
            {  3,  2,  2,  2,  2,  2,  2,  3 },
            { 11,  0,  0,  0,  4,  2,  2,  2 },
            {  1,  1,  0,  0,  0,  0,  0,  0 },
            {  1,  1,  1,  1,  0,  0,  0,  1 },
            {  1,  1,  1,  1,  1,  1,  1,  1 }
        },
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  4,  4,  6,  5,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  2,  2,  2,  2,  2,  2 },
            {  3,  2,  2,  2,  4,  0,  0,  0 },
            {  0,  0,  0,  0,  0,  0,  0,  0 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  5,  6,  4,  4,  5 }
        }
    },
    { /* mode 3 */
        {
            {  8,  8,  6,  6,  6,  6,  8,  8 },
            {  8, 10,  4,  8,  8,  6,  6,  8 },
            {  3,  3,  8,  8,  8,  8,  8,  8 },
            {  2,  2,  8,  8,  8,  0,  0,  0 },
            {  7,  3,  2,  2,  8,  0,  0,  1 },
            {  0,  2,  2,  2,  8,  8,  8,  0 },
            {  1,  8,  8,  8,  8,  8,  8,  1 },
            { 10,  1,  6,  6,  8,  8,  8, 10 }
        },
        {
            {  8,  8,  8,  6,  6,  6,  8,  8 },
            {  8,  8,  8,  6,  6,  6,  6,  8 },
            {  3,  3,  8,  8,  8,  8,  8,  8 },
            {  2,  2,  8,  8,  8,  8,  0,  0 },
            {  7,  2,  2,  2,  8,  0,  0,  0 },
            {  0,  2,  2,  8,  8,  8,  8,  0 },
            {  1,  8,  8,  8,  6,  6,  8,  1 },
            {  8,  8,  6,  6,  6,  6,  8,  8 }
        },
        {
            {  6,  6,  6,  6,  6,  6,  6,  6 },
            {  6,  6,  8,  6,  6,  6,  6,  6 },
            {  2,  2,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  4,  4,  4,  4,  0,  0 },
            {  7,  2,  2,  4,  4,  4,  0,  0 },
            {  0,  2,  2,  4,  4,  4,  4,  0 },
            {  0,  4,  4,  4,  6,  6,  4,  0 },
            {  6,  6,  6,  6,  6,  6,  4,  6 }
        },
        {
            {  6,  6,  5,  6,  6,  6,  5,  6 },
            {  6,  6,  5,  6,  5,  6,  6,  6 },
            {  2,  2,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  4,  4,  4,  4,  0,  0 },
            {  7,  2,  2,  4,  4,  4,  0,  1 },
            {  0,  2,  2,  4,  4,  4,  4,  0 },
            {  0,  4,  4,  4,  6,  6,  4,  0 },
            {  6,  6,  6,  6,  5,  6,  5,  6 }
        },
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  2,  2,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  4,  4,  4,  4,  0,  0 },
            {  7,  2,  2,  4,  4,  4,  0,  0 },
            {  0,  2,  2,  4,  4,  4,  4,  0 },
            {  0,  4,  4,  4,  4,  4,  4,  0 },
            {  5,  5,  5,  5,  5,  5,  5,  5 }
        }
    },
    { /* mode 4 */
        {
            {  7,  0,  0,  0,  7,  2,  2,  2 },
            {  0,  0,  0,  0,  0,  0,  0,  0 },
            {  1,  1,  1,  1,  1,  1,  1,  1 },
            {  1,  1,  1,  1,  1,  1,  1,  1 },
            {  7,  3,  3,  3,  7,  1,  1,  1 },
            {  3,  3,  3,  3,  3,  3,  3,  3 },
            {  3,  3,  3,  3,  3,  3,  3,  3 },
            {  3,  3,  3,  3,  3,  3,  3,  3 }
        },
        {
            {  7,  0,  0,  0,  7,  2,  2,  2 },
            {  0,  0,  0,  0,  0,  0,  0,  0 },
            {  1,  1,  1,  1,  1,  1,  1,  1 },
            {  1,  1,  1,  1,  1,  1,  1,  1 },
            {  7,  3,  3,  3,  7,  1,  1,  1 },
            {  3,  3,  3,  3,  3,  3,  3,  3 },
            {  3,  3,  3,  3,  3,  3,  3,  3 },
            {  3,  3,  3,  3,  3,  3,  3,  3 }
        },
        {
            {  7,  0,  0,  0,  7,  2,  2,  2 },
            {  0,  0,  0,  0,  0,  0,  0,  0 },
            {  1,  1,  1,  1,  1,  1,  1,  1 },
            {  1,  1,  1,  1,  1,  1,  1,  1 },
            {  7,  3,  3,  3,  7,  1,  1,  1 },
            {  3,  3,  3,  3,  3,  3,  3,  3 },
            {  3,  3,  3,  3,  3,  3,  3,  3 },
            {  3,  3,  3,  3,  3,  3,  3,  3 }
        },
        {
            {  5,  5,  6,  6,  6,  6,  6,  5 },
            {  5,  5,  4,  4,  6,  6,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  3,  3,  2,  2,  4,  2,  0,  1 },
            {  7,  2,  2,  2,  4,  0,  0,  0 },
            {  1,  3,  0,  0,  0,  0,  0,  1 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  6,  6,  4,  4,  5 }
        },
        {
            {  5,  5,  5,  5,  5,  5,  5,  5 },
            {  5,  5,  4,  4,  6,  5,  5,  5 },
            {  3,  3,  4,  4,  4,  4,  4,  4 },
            {  2,  2,  2,  2,  2,  2,  2,  2 },
            {  3,  2,  2,  2,  4,  0,  0,  0 },
            {  0,  0,  0,  0,  0,  0,  0,  0 },
            {  1,  4,  4,  4,  4,  4,  4,  1 },
            {  5,  5,  5,  5,  6,  4,  4,  5 }
        }
    }
};

static uint32_t s_ai_rng = 0xA152E001u;
static double   s_ai_clock;             /* FUN_004a2be0 stand-in, seconds */


typedef struct {
    int      used;
    int      goal;        /* AI_GOAL_*                              */
    double   x, y, z;     /* world position (y frozen per D-A6)     */
    double   contact_vx, contact_vz; /* decaying world contact velocity */
    float   *path;        /* owned copy, path_n*3 floats            */
    int      path_n;
    int      path_id;     /* FSM path index; -1 = none (D-A5)       */
    int      wp;          /* current target waypoint                */
    /* D-A13 planned-leg state. Reusable per-agent buffers — the planner
     * never allocates at tick time. Index 0 of a plan is the A* seed: a
     * search origin and the D-A14 smoothing anchor, NEVER a drive target
     * — the actuator's first target is the first forward route point. */
    double   plan_x[AI_PLAN_MAX]; /* reconstructed start-to-goal grid */
    double   plan_z[AI_PLAN_MAX]; /* points, ending at the raw target */
    int      plan_n;
    int      plan_idx;    /* planned point the actuator is driving to */
    int      plan_point_ahead; /* target has been forward of heading */
    int      plan_leg_wp; /* authored wp the plan targets; -1 = none  */
    int      plan_dirty;  /* edge costs changed (avoid flip): replan  */
    int      path_stunt_ok; /* D-A21: path was set by TELEPORT — its
                             * unplannable legs are authored stunts and
                             * may be driven directly */
    int      leg_stunt_on;  /* D-A21: direct drive engaged on this leg */
    int      leg_stall;     /* D-A21: ticks without progress to the wp */
    double   leg_best_d2;   /* D-A21: best squared distance to the wp  */
    int      leg_stunt_wp;  /* D-A21: wp the three fields above track  */
    int      f_plan_on;     /* D-A22: planner-assisted chase engaged   */
    int      f_stall;       /* D-A22: ticks in the current no-progress
                             * (direct) / line-retest (assist) interval */
    double   f_best_d2;     /* D-A22: best squared distance to target  */
    double   f_gx, f_gz;    /* D-A22: target position the active assist
                             * plan was searched toward                */
    double   heading;     /* rad, car.h convention (D-A15): forward =  */
                          /* (-sin h, 0, cos h); finite, wrapped       */
    double   combat_steer_bias;    /* MARKED one-tick combat command     */
    int      combat_bias_pending;  /* command issued since last AI tick  */
    int      route_bias_contact;   /* weave/shove hysteresis latch       */
    int      route_bias_hold;      /* active for corrections this tick   */
    double   grip_scale;        /* MARKED oil-slick steering multiplier */
    int      grip_ticks;        /* bounded sim ticks remaining           */
    int      heading_set; /* seeded by ai_seed_heading or the first    */
                          /* desired direction; goal re-issues keep it */
    double   speed;       /* m/s, positive values clamped (D-A1)    */
    double   drive_speed; /* filtered command; skill is response, not vmax */
    int      stationary_path; /* active goto/teleport was exact speed 0 */
    int      arrived;     /* pending isArrived pulse                */
    int      arrivals;    /* monotone completion count (probe)      */
    int      target;      /* follow target entity, -1               */
    double   xoff;        /* follow lateral offset, meters (D-A7)   */
    double   follow_hold_dist; /* MARKED combat range, 0=FACT 10 m */
    int      at_follow;   /* within AI_FOLLOW_DIST of target        */
    int      sat;         /* parked by ai_sit: weapons-safe (D-A17)  */
    int      rival;       /* race rival entity (D-A8, unused)       */
    int      has_raced;   /* durable provenance from the race verb    */
    int      agg;         /* mission setAgg value minus one (0..4)       */
    float    throttle_gain;
    float    steer_gain;
    float    aim_error;
    float    fire_probability;
    int      avoid;
    int      max_attackers;
    int      flyer;       /* D-A23: class-9 helicopter, polyline only */
    int      combat_seek; /* FACT combat tick: no range hold, heading octant */
    int      combat_dest; /* FUN_004152e0 dest; 0 = unset                */
    int      turn_state;  /* dest 9 a97c: 1 = close-range offset branch  */
    int      arms_ready;  /* dest 6 FUN_00409050: any ready mount        */
    int      dest9_ready; /* dest 9 FUN_004090f0: rocket / type-4 ready  */
    double   dest_until;  /* a82c: dest 6/8/9 engage timer, sim seconds  */
    /* H-UAT-078c: one decoded command drives either route kinematics or the
     * attached physical context. Unpromoted native/dev contexts remain
     * observable no-feedback shadows. */
    AiDriveCommand drive_command;
    uint64_t command_trace_hash;
    CarSimContext *shadow;
    uint64_t shadow_steps;
    uint64_t shadow_state_trace_hash;
    uint64_t shadow_last_state_hash;
    int      shadow_reverse;
    int      shadow_diagnostic; /* native/dev no-feedback double-run */
    int      physical_authority;
    int      physical_pinned;   /* synthetic/melee family ownership */
#ifdef I76_AI_PHYSICS_PROBE
    /* Probe-only observability for H-UAT-078c Stage 0. These fields are
     * absent from production builds and never feed mover decisions. */
    double   probe_desired_heading;
    double   probe_steer_error;
    double   probe_actuator_speed;
    int      probe_throttle_intent;
    int      probe_control_valid;
    int      probe_direct_xz_write;
#endif
} AiAgent;

static AiAgent s_agents[AI_MAX_AGENTS];
static const CarCollider *s_context_world;
static int s_context_world_n;
static int s_context_world_bounds;
static double s_context_world_x0, s_context_world_z0;
static double s_context_world_x1, s_context_world_z1;
static const float s_skill_throttle[6] = { 0.0f, 1.0f, 0.8f, 0.6f, 0.5f, 0.4f };
static const float s_skill_steer[6]    = { 0.0f, 1.0f, 0.8f, 0.6f, 0.5f, 0.4f };
static const float s_skill_aim[6]      = { 0.0f, 1.0f, 0.8f, 0.6f, 0.4f, 0.2f };
static const float s_skill_fire[6]     = { 0.0f, 1.0f, 0.9f, 0.8f, 0.7f, 0.6f };

static AiAgent *agent_at(int ent)
{
    if (ent < 0 || ent >= AI_MAX_AGENTS)
        return NULL;
    return s_agents[ent].used ? &s_agents[ent] : NULL;
}

static double clamp_speed(double s)
{
    if (s < AI_SPEED_MIN)
        s = AI_SPEED_MIN;
    if (s > AI_SPEED_MAX)
        s = AI_SPEED_MAX;
    return s;
}

#define AI_PI 3.14159265358979323846

/* Shortest-form wrap to (-pi, pi]; keeps the D-A15 heading finite and
 * bounded no matter how long an agent steers. */
static double ang_wrap(double a)
{
    while (a > AI_PI)
        a -= 2.0 * AI_PI;
    while (a <= -AI_PI)
        a += 2.0 * AI_PI;
    return a;
}

/* Yaw of a desired XZ direction in the car.h convention (forward =
 * (-sin h, 0, cos h); matches mission.c's atan2(-dx, dz)). */
static double dir_yaw(double dx, double dz)
{
    return atan2(-dx, dz);
}

static void physical_sync(AiAgent *a)
{
    CarLive live = {0};
    if (!a || !a->physical_authority || !a->shadow)
        return;
    car_context_get_live(a->shadow, &live);
    a->x = live.x;
    a->y = live.y;
    a->z = live.z;
    a->heading = ang_wrap(live.yaw);
    a->heading_set = 1;
    a->drive_speed = live.vz;
    a->shadow_reverse = live.reverse != 0;
}

static int physical_activate(AiAgent *a, int pinned)
{
    if (!a || !a->shadow)
        return -1;
    if (!a->physical_authority) {
        CarLive live = {0};
        car_context_set_scripted_pose(a->shadow, a->x, a->z, a->heading,
                                      a->drive_speed);
        /* Preserve the source owner's complete handoff pose. The context
         * setter intentionally terrain-seats player camera scripts, but a
         * promoted NPC may carry authored hover/teleport height and must begin
         * airborne there rather than being silently dropped to terrain. */
        car_context_get_live(a->shadow, &live);
        live.y = a->y;
        live.vy = 0.0;
        car_context_set_live(a->shadow, &live);
        /* Move any pre-handoff D-A11/D-C20 velocity to the new owner once.
         * Leaving it latent would replay the old kinematic shove if an
         * authored route later reclaimed this actor. */
        car_context_add_world_velocity(a->shadow,
                                       a->contact_vx, a->contact_vz);
        a->contact_vx = a->contact_vz = 0.0;
    }
    a->physical_authority = 1;
    if (pinned)
        a->physical_pinned = 1;
    physical_sync(a);
    return 0;
}

static void plan_corrected(AiAgent *a, double dx, double dz);

static void physical_release_route(AiAgent *a)
{
    if (!a || !a->physical_authority || a->physical_pinned)
        return;
    physical_sync(a); /* route ownership starts at the integrated pose */
    {
        /* Kinematics can represent longitudinal speed directly. Preserve the
         * remaining physical world velocity (lateral/contact shove) in its
         * one decaying external channel instead of dropping it at handoff. */
        double wx = 0.0, wz = 0.0;
        double fx = -sin(a->heading), fz = cos(a->heading);
        car_context_world_velocity(a->shadow, &wx, &wz);
        a->contact_vx = wx - fx * a->drive_speed;
        a->contact_vz = wz - fz * a->drive_speed;
    }
    a->physical_authority = 0;
}

static uint64_t trace_hash_bytes(uint64_t h, const void *data, size_t n)
{
    const unsigned char *p = data;
    while (n--) {
        h ^= *p++;
        h *= UINT64_C(1099511628211);
    }
    return h;
}

static uint64_t command_trace_hash(uint64_t h, const AiDriveCommand *c)
{
#define HASH_COMMAND_FIELD(field) \
    (h = trace_hash_bytes(h, &c->field, sizeof c->field))
    HASH_COMMAND_FIELD(steer);
    HASH_COMMAND_FIELD(throttle);
    HASH_COMMAND_FIELD(brake);
    HASH_COMMAND_FIELD(reverse);
    HASH_COMMAND_FIELD(e_brake);
    HASH_COMMAND_FIELD(target_speed);
    HASH_COMMAND_FIELD(arrival_intent);
    HASH_COMMAND_FIELD(valid);
#undef HASH_COMMAND_FIELD
    return h;
}

/* FUN_00405c80 / FUN_004057c0 / FUN_00406060 / FUN_00466570 output seam.
 * D-A25–27 select `drive` versus brake and the desired heading; this helper
 * translates that primitive to normalized controls, never to pose. The
 * controller reads the selected owner's synchronized body state. An
 * unpromoted diagnostic shadow can never influence this command.
 *
 * Nitro's complete gain/rolling-resistance formula remains partly open, so
 * proportional speed-error scaling is a bounded PORT translation. The
 * physical car still supplies authored force, grip, drag, gears and surface
 * response after receiving these controls. */
static void brain_emit_controls(AiAgent *a, double desired,
                                double target_speed, int drive, double dt)
{
    AiDriveCommand c = {0};
    double signed_target = isfinite(target_speed) ? target_speed : 0.0;
    double mag = fabs(signed_target);
    double err = ang_wrap(ang_wrap(desired + a->combat_steer_bias) -
                          a->heading);
    double turn_scale = AI_TURN_MAX * dt;
    double steer = turn_scale > 0.0 ? err / turn_scale : 0.0;
    double steer_gain = fmax(0.0, fmin(1.0, (double)a->steer_gain));
    if (steer > 1.0) steer = 1.0;
    if (steer < -1.0) steer = -1.0;
    c.steer = steer * steer_gain;
    c.target_speed = signed_target;
    c.reverse = signed_target < 0.0;
    c.arrival_intent = a->arrived || a->at_follow ||
                       (a->goal == AI_GOAL_GOTO && a->path_n > 0 &&
                        a->wp >= a->path_n - 1);
    c.valid = 1;
    if (!drive || mag <= 0.0) {
        c.brake = 1.0;
    } else {
        double current = fabs(a->drive_speed);
        double speed_error = mag - current;
        double gain = fmax(0.0, fmin(1.0, (double)a->throttle_gain));
        double denom = mag > 1.0 ? mag : 1.0;
        if (speed_error > 0.0) {
            c.throttle = fmin(1.0, speed_error / denom) * gain;
        } else {
            c.brake = fmin(1.0, -speed_error / denom);
        }
    }
    a->drive_command = c;
}

/* D-A15/D-A16 KINEMATIC AUTHORITY actuator: turn the persistent heading
 * toward `desired` under the lateral-acceleration budget
 * (omega = AI_LAT_ACCEL*steer_gain / v, clamped to
 * AI_TURN_MAX*steer_gain) and return this tick's forward speed.
 * For the captured P01 skill-1 state, D-A16 limits that response to the
 * measured physical acceleration envelope. Other skill rows retain D-A15's
 * prior response until native tapes establish their launch dynamics. Neither
 * path changes the command's steady-state value.
 * Positive forward alignment then scales speed by cos(error)^2; at 90
 * degrees or worse the mover brake-steers in place. */
static double kinematic_actuator_tick(AiAgent *a, double desired, double dt)
{
    if (!(dt > 0.0) || !isfinite(dt))
        return 0.0;
    desired = ang_wrap(desired + a->combat_steer_bias);
    a->combat_steer_bias = 0.0;   /* command is one tick, never latent */
    double err = ang_wrap(desired - a->heading);
    double scale = cos(err);
    double steer = fmax((double)a->steer_gain, 0.0);
    if (a->grip_ticks > 0)
        steer *= a->grip_scale;
    double throttle = fmin(fmax((double)a->throttle_gain, 0.0), 1.0);
    double response = (a->speed - a->drive_speed) * throttle;
#ifdef I76_AI_PHYSICS_PROBE
    a->probe_desired_heading = desired;
    a->probe_steer_error = err;
    a->probe_throttle_intent = response > 0.0 ? 1 : response < 0.0 ? -1 : 0;
    a->probe_control_valid = 1;
#endif
    double max_dv = throttle >= 1.0 ? AI_LONG_ACCEL * dt : fabs(response);
    double turnmax = AI_TURN_MAX * steer;
    double v, wmax, dh;
    if (scale <= 0.0)
        scale = 0.0;
    else
        scale *= scale;
    if (response > max_dv)
        response = max_dv;
    else if (response < -max_dv)
        response = -max_dv;
    a->drive_speed += response;
    v = a->drive_speed * scale;
#ifdef I76_AI_PHYSICS_PROBE
    a->probe_actuator_speed = v;
#endif
    wmax = v > 1e-9 ? AI_LAT_ACCEL * steer / v : turnmax;
    if (wmax > turnmax)
        wmax = turnmax;
    dh = err;
    if (dh > wmax * dt) dh = wmax * dt;
    if (dh < -wmax * dt) dh = -wmax * dt;
    a->heading = ang_wrap(a->heading + dh);
    return v;
}

/* One brain decision, two explicitly separated actuator owners. Context
 * stepping is deferred to a fixed-order pass after every brain has finished,
 * preventing entity-order state from entering another actor's decision. */
static double drive_actuator_tick(AiAgent *a, double desired, int drive,
                                  double dt)
{
    brain_emit_controls(a, desired, drive ? a->speed : 0.0, drive, dt);
    if (a->physical_authority) {
        /* The command reaches car.c in the fixed-order actuator pass. The
         * brain may change controls here, never the integrated pose/heading. */
        a->combat_steer_bias = 0.0;
#ifdef I76_AI_PHYSICS_PROBE
        a->probe_desired_heading = ang_wrap(desired);
        a->probe_steer_error = ang_wrap(desired - a->heading);
        a->probe_throttle_intent = a->drive_command.throttle > 0.0 ? 1 :
                                   a->drive_command.brake > 0.0 ? -1 : 0;
        a->probe_control_valid = 1;
        a->probe_actuator_speed = a->drive_speed;
#endif
        return fabs(a->drive_speed);
    }
    return kinematic_actuator_tick(a, desired, dt);
}

/* FUN_0045a040: wrap (to-from), clamp to [-π/8, 2π-π/8], floor onto 8
 * octants. Combat tick accepts only 0, 1, and 7 (target ahead). */
static int heading_octant(double from, double to)
{
    double delta = to - from;
    if (to < from)
        delta += 6.28318530717958647692;
    if (delta > 5.8904862254808623)
        delta = 5.8904862254808623;
    if (delta < -0.39269908169872415)
        delta = -0.39269908169872415;
    int oct = (int)floor((delta + 0.39269908169872415) *
                         1.2732395447351628);
    if (oct < 0) oct = 0;
    if (oct > 7) oct = 7;
    return oct;
}

static int combat_target_ahead(const AiAgent *a, double dx, double dz)
{
    double d = sqrt(dx * dx + dz * dz);
    int oct;
    if (d < 1e-6)
        return 1;
    oct = heading_octant(a->heading, dir_yaw(dx, dz));
    return oct == 0 || oct == 1 || oct == 7;
}

static uint32_t ai_rng(void)
{
    uint32_t x = s_ai_rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s_ai_rng = x ? x : 0xA152E001u;
    return s_ai_rng;
}

/* Engage conditions we can evaluate without the unported node graph.
 * dests 1/3/10/11 require FUN_00416da0 occupancy — fail closed. */
static int dest_accept(const AiAgent *a, int dest)
{
    switch (dest) {
    case 4:
    case 5:
    case 6:
    case 13:
        return a->arms_ready;
    case 8:
    case 14:
        return 1;
    case 9:
        return a->dest9_ready;
    default:
        return 0;
    }
}

static int dest_clsn_mode(int dest)
{
    switch (dest) {
    case 4:  return 0;
    case 6:
    case 13: return 1;
    case 8:  return 2;
    case 5:  return 3;
    case 14: return 4;
    default: return -1;
    }
}

static int pick_weighted(const AiAgent *a, const int *dests,
                         const int *weights, int n)
{
    int w[16];
    int sum = 0, tries, i;
    if (n <= 0 || n > 16)
        return 15;
    for (i = 0; i < n; i++) {
        w[i] = weights[i];
        sum += w[i];
    }
    for (tries = 0; tries < 0x12 && sum > 0; tries++) {
        int draw = (int)(ai_rng() % (uint32_t)sum);
        int idx = 0;
        if (w[0] <= draw) {
            int cur = w[0];
            do {
                draw -= cur;
                idx++;
                cur = w[idx];
            } while (idx < n - 1 && cur <= draw);
        }
        if (dest_accept(a, dests[idx]))
            return dests[idx];
        sum -= w[idx];
        w[idx] = 0;
    }
    return 15;
}

/* FUN_004098a0 timeUp: dest 6/8/9 pursuit-1 re-pick. */
static int pick_timeup_dest(const AiAgent *a, int dest)
{
    int agg = a->agg;
    if (agg < 0 || agg > 4)
        agg = 4;
    if (dest == 6 || dest == 4 || dest == 5)
        return pick_weighted(a, s_p1_d68, s_p1_w6[agg], 10);
    if (dest == 8)
        return pick_weighted(a, s_p1_d68, s_p1_w8[agg], 10);
    if (dest == 9)
        return pick_weighted(a, s_p1_d9, s_p1_w9[agg], 7);
    return dest;
}

static int pick_init_dest(const AiAgent *a)
{
    int agg = a->agg;
    int w[6];
    int sum = 0, tries, i;
    if (agg < 0 || agg > 4)
        agg = 4;
    for (i = 0; i < 6; i++) {
        w[i] = s_init_w[agg][i];
        sum += w[i];
    }
    for (tries = 0; tries < 0x12 && sum > 0; tries++) {
        int draw = (int)(ai_rng() % (uint32_t)sum);
        int idx = 0;
        if (w[0] <= draw) {
            int cur = w[0];
            do {
                draw -= cur;
                idx++;
                cur = w[idx];
            } while (idx < 5 && cur <= draw);
        }
        if (dest_accept(a, s_init_dests[idx]))
            return s_init_dests[idx];
        sum -= w[idx];
        w[idx] = 0;
    }
    return 15;
}

static int range_band(double d)
{
    if (d < 9.0)
        return 0;
    if (d < 24.0)
        return 1;
    if (d < 60.0)
        return 2;
    if (d < 200.0)
        return 3;
    return 4;
}

/* FUN_00408ac0: dest 4/6/8/5/14 modes 0..4. Cases 4/5/6/14 call
 * unported intercept helpers and steer at the target. */
static void dest68_plan(const AiAgent *a, int dest, double dx, double dz,
                        double d, double tgt_heading, int have_tgt_hdg,
                        double *desired, int *throttle)
{
    int mode = dest_clsn_mode(dest);
    int soct = heading_octant(a->heading, dir_yaw(dx, dz));
    int toct = have_tgt_hdg
             ? heading_octant(tgt_heading, dir_yaw(-dx, -dz)) : 4;
    int act;
    if (mode < 0)
        mode = 1;
    act = s_clsn_tbl[mode][range_band(d)][soct][toct];
    double toward = dir_yaw(dx, dz);
    const double qturn = 0.7853981633974483;   /* _DAT_004c1784 = π/4 */
    *throttle = 1;
    switch (act) {
    case 0:
    case 1:
        *desired = ang_wrap(a->heading - qturn);
        break;
    case 2:
    case 3:
        *desired = ang_wrap(a->heading + qturn);
        break;
    case 7:
        *desired = ang_wrap(a->heading +
            (fabs(ang_wrap(toward - a->heading)) >= 1.5707963267948966
                 ? qturn : -qturn));
        break;
    case 8:
        *desired = toward;
        if (d < 24.0 && fabs(ang_wrap(toward - a->heading)) < 0.19)
            *throttle = 0;
        break;
    case 10:
    case 11:
        *desired = a->heading;
        break;
    case 12:
    case 13:
        *desired = a->heading;
        *throttle = 0;
        break;
    default:
        *desired = toward;
        break;
    }
}

/* FUN_00409320 / dest 9. Engage writes a97c=1: offset-steer inside 70 m,
 * then face and apply speed only while the heading error is >= 0.1 rad. */
static void dest9_plan(AiAgent *a, double dx, double dz, double d,
                       double *desired, int *throttle)
{
    double toward = dir_yaw(dx, dz);
    if (a->turn_state && d < 70.0) {
        *desired = ang_wrap(toward + AI_PI);
        *throttle = 0;
        return;
    }
    a->turn_state = 0;
    *desired = toward;
    *throttle = fabs(ang_wrap(toward - a->heading)) >= 0.1;
}

static void apply_combat_dest(AiAgent *a, int dest)
{
    double hold;
    a->combat_dest = dest;
    a->turn_state = dest == 9 ? 1 : 0;
    /* FUN_00408fa0 / 095a0 / 09260 / 08ec0: a82c = now + hold. */
    if (dest == 6 || dest == 5 || dest == 13)
        hold = 20.0 + (double)(ai_rng() % 15u);
    else if (dest == 8 || dest == 14)
        hold = 10.0 + (double)(ai_rng() % 10u);
    else if (dest == 9 || dest == 4)
        hold = dest == 4 ? 7.0 : 7.0 + (double)(ai_rng() % 10u);
    else
        hold = 0.0;
    a->dest_until = hold > 0.0 ? s_ai_clock + hold : 0.0;
}

/* FUN_0040ad20: dest 6/8/9 yield to dest 20 (termTF) when the target is
 * far or on a different height. Dest 20's native tick is route-follow to
 * a snapshot; the port uses live-target close + D-A22. */
static int dest_far_handoff(double d, double dy, int flyer, int tgt_vehicle)
{
    if (flyer)
        return d > 720.0;           /* _DAT_004c1860 */
    if (!tgt_vehicle)
        return d > 200.0;           /* _DAT_004c1864 */
    if (fabs(dy) > 30.0)            /* _DAT_004c1868 */
        return 1;
    return d > 180.0;               /* _DAT_004c1870 */
}

/* Replace the agent's path with an owned copy. */
static int set_path(AiAgent *a, const float *pts, int npts)
{
    float *copy = malloc((size_t)npts * 3 * sizeof *copy);
    if (!copy)
        return -1;
    memcpy(copy, pts, (size_t)npts * 3 * sizeof *copy);
    free(a->path);
    a->path   = copy;
    a->path_n = npts;
    return 0;
}

void ai_reset(void)
{
    for (int i = 0; i < AI_MAX_AGENTS; i++) {
        free(s_agents[i].path);
        car_context_destroy(s_agents[i].shadow);
    }
    memset(s_agents, 0, sizeof s_agents);
    s_context_world = NULL;
    s_context_world_n = 0;
    s_context_world_bounds = 0;
    s_context_world_x0 = s_context_world_z0 = 0.0;
    s_context_world_x1 = s_context_world_z1 = 0.0;
    s_ai_rng = 0xA152E001u;
    s_ai_clock = 0.0;
}

void ai_agent_init(int ent, double x, double y, double z)
{
    AiAgent *a;
    if (ent < 0 || ent >= AI_MAX_AGENTS)
        return;
    a = &s_agents[ent];
    free(a->path);
    car_context_destroy(a->shadow);
    memset(a, 0, sizeof *a);
    a->used    = 1;
    a->x       = x;
    a->y       = y;
    a->z       = z;
    a->path_id = -1;
    a->target  = -1;
    a->rival   = -1;
    a->plan_leg_wp = -1;    /* D-A13: no planned leg after re-register */
    a->plan_dirty  = 1;
    /* Nitro FUN_004187f0/FUN_00407420 constructor defaults. In the
     * mission-facing one-based API, these equal setSkill(ent,5,5). */
    a->agg = 4;
    a->throttle_gain = s_skill_throttle[5];
    a->steer_gain = s_skill_steer[5];
    a->aim_error = s_skill_aim[5];
    a->fire_probability = s_skill_fire[5];
    a->grip_scale = 1.0;
    a->avoid = 1;
    a->max_attackers = 1000;
    a->arms_ready = 1;      /* dest 6 accepts until combat says otherwise */
    a->dest9_ready = 0;     /* dest 9 needs FUN_004090f0 rockets/type-4 */
    a->command_trace_hash = UINT64_C(1469598103934665603);
    a->shadow_state_trace_hash = UINT64_C(1469598103934665603);
    a->drive_command.brake = 1.0;
    a->drive_command.valid = 1;
}

int ai_shadow_attach(int ent, unsigned class_id, const char *vcf_name)
{
    AiAgent *a = agent_at(ent);
    if (!a || class_id != 1 || !vcf_name || !*vcf_name)
        return -1;
    CarSimContext *ctx = car_context_create();
    if (!ctx)
        return -1;
    if (car_context_load(ctx, vcf_name) != 0) {
        car_context_destroy(ctx);
        return -1;
    }
    car_context_place(ctx, a->x, a->z, a->heading);
    car_context_destroy(a->shadow);
    a->shadow = ctx;
    if (s_context_world_n > 0)
        car_context_set_colliders(ctx, s_context_world, s_context_world_n);
    if (s_context_world_bounds)
        car_context_set_bounds(ctx, s_context_world_x0, s_context_world_z0,
                               s_context_world_x1, s_context_world_z1);
    a->shadow_steps = 0;
    a->shadow_reverse = 0;
#if !defined(__EMSCRIPTEN__) || defined(I76_AI_SHADOW_DEV)
    a->shadow_diagnostic = 1;
#else
    /* Production Wasm keeps route-owned contexts dormant. A combat FOLLOW
     * promotion reuses the already-loaded context without double-running it. */
    a->shadow_diagnostic = 0;
#endif
    a->shadow_last_state_hash = car_context_state_hash(ctx);
    a->shadow_state_trace_hash = UINT64_C(1469598103934665603);
    return 0;
}

int ai_promote_physical(int ent)
{
    return physical_activate(agent_at(ent), 1);
}

int ai_physical_active(int ent)
{
    const AiAgent *a = agent_at(ent);
    return a && a->physical_authority;
}

int ai_physical_live(int ent, CarLive *out)
{
    const AiAgent *a = agent_at(ent);
    if (!a || !a->physical_authority || !a->shadow || !out)
        return -1;
    car_context_get_live(a->shadow, out);
    return 0;
}

void ai_world_velocity(int ent, double *vx, double *vz)
{
    const AiAgent *a = agent_at(ent);
    if (a && a->physical_authority && a->shadow) {
        car_context_world_velocity(a->shadow, vx, vz);
        return;
    }
    if (vx) *vx = a ? a->drive_speed * -sin(a->heading) + a->contact_vx : 0.0;
    if (vz) *vz = a ? a->drive_speed *  cos(a->heading) + a->contact_vz : 0.0;
}

void ai_apply_vehicle_contact(int ent, double dx, double dz,
                              double nx, double nz,
                              double other_vx, double other_vz)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return;
    if (a->physical_authority && a->shadow) {
        car_context_apply_vehicle_contact(a->shadow, dx, dz, nx, nz,
                                          other_vx, other_vz);
        physical_sync(a);
        if (dx != 0.0 || dz != 0.0)
            plan_corrected(a, dx, dz);
        return;
    }
    ai_translate_xz(ent, dx, dz);
}

void ai_shadow_set_world(const CarCollider *list, int n,
                         double x0, double z0, double x1, double z1)
{
    s_context_world = n > 0 ? list : NULL;
    s_context_world_n = n > 0 ? n : 0;
    s_context_world_bounds = x1 > x0 && z1 > z0;
    s_context_world_x0 = x0; s_context_world_z0 = z0;
    s_context_world_x1 = x1; s_context_world_z1 = z1;
    /* Always visit existing contexts, including n==0: world rebuild calls
     * this before freeing borrowed storage, so zero candidates must clear the
     * old pointer rather than leave a dangling snapshot. */
    for (int i = 0; i < AI_MAX_AGENTS; i++) {
        AiAgent *a = &s_agents[i];
        if (!a->used || !a->shadow)
            continue;
        if (n > 0)
            car_context_set_colliders(a->shadow, list, n);
        else
            car_context_set_colliders(a->shadow, NULL, 0);
        car_context_set_bounds(a->shadow, x0, z0, x1, z1);
    }
}

int ai_drive_command(int ent, AiDriveCommand *out)
{
    const AiAgent *a = agent_at(ent);
    if (!a || !out)
        return -1;
    *out = a->drive_command;
    return 0;
}

int ai_shadow_telemetry(int ent, AiShadowTelemetry *out)
{
    const AiAgent *a = agent_at(ent);
    if (!a || !out)
        return -1;
    memset(out, 0, sizeof *out);
    out->active = a->shadow != NULL;
    out->steps = a->shadow_steps;
    out->command_trace_hash = a->command_trace_hash;
    out->state_trace_hash = a->shadow_state_trace_hash;
    out->last_state_hash = a->shadow_last_state_hash;
    return 0;
}

void ai_set_flyer(int ent, int on)
{
    AiAgent *a = agent_at(ent);
    if (a)
        a->flyer = on ? 1 : 0;
}

int ai_set_skill(int ent, int steer_throttle, int aim_fire)
{
    AiAgent *a = agent_at(ent);
    if (!a || steer_throttle < 0 || steer_throttle > 5 ||
        aim_fire < 0 || aim_fire > 5)
        return -1;
    a->throttle_gain = s_skill_throttle[steer_throttle];
    a->steer_gain = s_skill_steer[steer_throttle];
    a->aim_error = s_skill_aim[aim_fire];
    a->fire_probability = s_skill_fire[aim_fire];
    return 0;
}

int ai_set_agg(int ent, int agg)
{
    AiAgent *a = agent_at(ent);
    if (!a || agg < 1 || agg > 5)
        return -1;
    a->agg = agg - 1;
    /* FUN_00415290 resets this field after every setAgg call. A later
     * setMaxAttackers call wins, preserving the original call ordering. */
    a->max_attackers = 1;
    return 0;
}

int ai_set_avoid(int ent, int avoid)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return -1;
    a->avoid = avoid;
    a->plan_dirty = 1;  /* D-A13: avoid arms the 1e6 blocked-edge term */
    return 0;
}

int ai_toggle_avoid(int ent)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return -1;
    a->avoid = a->avoid == 0;
    a->plan_dirty = 1;  /* D-A13: avoid arms the 1e6 blocked-edge term */
    return 0;
}

int ai_set_max_attackers(int ent, int max_attackers)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return -1;
    a->max_attackers = max_attackers;
    return 0;
}

int ai_director_state(int ent, AiDirectorState *out)
{
    const AiAgent *a = agent_at(ent);
    if (!a || !out)
        return -1;
    out->agg = a->agg;
    out->throttle_gain = a->throttle_gain;
    out->steer_gain = a->steer_gain;
    out->aim_error = a->aim_error;
    out->fire_probability = a->fire_probability;
    out->avoid = a->avoid;
    out->max_attackers = a->max_attackers;
    return 0;
}

void ai_combat_set_arms(int ent, int any_ready, int dest9_ready)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return;
    a->arms_ready = any_ready ? 1 : 0;
    a->dest9_ready = dest9_ready ? 1 : 0;
}

int ai_combat_dest(int ent)
{
    AiAgent *a = agent_at(ent);
    return a ? a->combat_dest : 0;
}

void ai_combat_set_dest(int ent, int dest)
{
    AiAgent *a = agent_at(ent);
    if (a)
        apply_combat_dest(a, dest);
}

void ai_goto(int ent, int path_id, const float *pts, int npts, double speed)
{
    AiAgent *a = agent_at(ent);
    if (!a || !pts || npts < 1)
        return;
    physical_release_route(a);
    /* D-A5: re-issuing the path the agent is CURRENTLY on is a no-op
     * except for a speed update — P01's machines re-issue goto/race every
     * poll tick while en route (m4 flips jade's race speed 13<->23 by
     * convoy proximity), and a restart would yank the car back to
     * waypoint 0. A re-issue AFTER arrival (goal NONE) starts a fresh
     * lap from waypoint 0 — that is the script-visible meaning of
     * issuing goto again once isArrived reported the last run done. */
    if (a->path_id == path_id && a->goal == AI_GOAL_GOTO) {
        a->stationary_path = speed == 0.0;
        a->speed = a->stationary_path ? 0.0 : clamp_speed(speed);
        a->rival = -1;       /* a direct goto ends race ownership */
        return;
    }
    if (set_path(a, pts, npts) != 0)
        return;
    a->goal    = AI_GOAL_GOTO;
    a->path_id = path_id;
    a->wp      = 0;
    a->sat     = 0;         /* D-A17: a goal verb ends the sit park */
    a->plan_n  = 0;         /* D-A13: drop any prior leg's plan */
    a->plan_idx = 0;
    a->plan_leg_wp = -1;
    a->plan_dirty  = 1;
    a->path_stunt_ok = 0;   /* D-A21: only teleport-set paths may drive
                             * unplannable legs directly */
    a->stationary_path = speed == 0.0;
    a->speed   = a->stationary_path ? 0.0 : clamp_speed(speed);
    a->arrived = 0;
    a->target  = -1;
    a->rival   = -1;
    a->at_follow = 0;
}

/* Race entry is pose-local, unlike the port's explicit goto restart.
 * FUN_0040c590 selects the first nearest XZ node, then advances cyclically
 * while the outgoing segment faces the car (normalized dot > -0.1f).
 * FUN_0040c760 calls it on fresh route entry. See race-route-entry.md.
 * PORT NUMERICS: sqrt evaluates normalization instead of the executable's
 * reciprocal-sqrt approximation; this is not a bitwise trajectory claim. */
static int race_entry_wp(const AiAgent *a)
{
    const float x = (float)a->x, z = (float)a->z;
    float best = INFINITY;
    int start = 0;
    for (int i = 0; i < a->path_n; i++) {
        float dx = x - a->path[i * 3];
        float dz = z - a->path[i * 3 + 2];
        float d2 = dx * dx + dz * dz;
        if (d2 < best) { best = d2; start = i; }
    }
    int wp = start;
    do {
        int next = (wp + 1) % a->path_n;
        float dx = a->path[next * 3] - a->path[wp * 3];
        float dz = a->path[next * 3 + 2] - a->path[wp * 3 + 2];
        float rx = x - a->path[wp * 3];
        float rz = z - a->path[wp * 3 + 2];
        float segment2 = dx * dx + dz * dz;
        float radius2 = rx * rx + rz * rz;
        /* FUN_00406330 returns +1 for its near-zero vector guards. */
        double facing = 1.0;
        if (segment2 >= 0.0001f && radius2 >= 0.0001f)
            facing = ((double)dx * rx + (double)dz * rz) /
                     sqrt((double)segment2 * radius2);
        if (facing <= (double)-0.1f)
            break;
        wp = next;
    } while (wp != start);
    return wp;
}

void ai_race(int ent, int path_id, const float *pts, int npts, double speed,
             int rival_ent)
{
    AiAgent *a = agent_at(ent);
    if (!a || !pts || npts < 1)
        return;
    int entering = a->goal != AI_GOAL_GOTO || a->path_id != path_id;
    physical_release_route(a); /* explicit RACE ownership boundary */
    ai_goto(ent, path_id, pts, npts, speed);
    if (a->goal == AI_GOAL_GOTO && a->path_id == path_id) {
        if (entering)
            a->wp = race_entry_wp(a);
        a->rival = rival_ent;       /* D-A8: marks an active race */
        a->has_raced = 1;
    }
}

void ai_teleport(int ent, int path_id, const float *pts, int npts,
                 double speed, double y_snap, double dx_m, double dz_m)
{
    AiAgent *a = agent_at(ent);
    if (!a || !pts || npts < 1)
        return;
    physical_release_route(a);
    /* FACT (fsm.md §4.2, Open76 :275): move entity to path node 0, set
     * speed, start path-following. dx/dz are the teleportOffset deltas
     * (0 for plain teleport). */
    if (set_path(a, pts, npts) != 0)
        return;
    a->contact_vx = 0.0;
    a->contact_vz = 0.0;
    a->x       = pts[0] + dx_m;
    a->y       = y_snap;
    a->z       = pts[2] + dz_m;
    a->goal    = AI_GOAL_GOTO;
    a->path_id = path_id;
    a->wp      = npts > 1 ? 1 : 0;
    a->sat     = 0;         /* D-A17 */
    a->plan_n  = 0;         /* D-A13: position snap voids any plan */
    a->plan_idx = 0;
    a->plan_leg_wp = -1;
    a->plan_dirty  = 1;
    a->stationary_path = speed == 0.0;
    a->speed = a->stationary_path ? 0.0 : clamp_speed(speed);
    a->drive_speed = a->speed;  /* teleport's confirmed set-speed contract */
    a->arrived = 0;
    a->rival   = -1;
    a->target  = -1;
    a->at_follow = 0;
    a->path_stunt_ok = 1;       /* D-A21: teleport marks an authored
                                 * stunt/staging path — its unplannable
                                 * legs are driven directly */
    /* Explicit authored teleports are the only route-pose discontinuity a
     * context adopts. Normal promoted motion remains integrated. */
    if (a->shadow) {
        CarLive live = {0};
        car_context_set_scripted_pose(a->shadow, a->x, a->z,
                                      a->heading, a->speed);
        car_context_get_live(a->shadow, &live);
        live.y = a->y;
        live.vy = 0.0;
        car_context_set_live(a->shadow, &live);
        a->shadow_reverse = 0;
        physical_sync(a);
    }
}

static void follow_set(int ent, int target_ent, double xoff_m, double speed,
                       int combat_owned)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return;
    if (!combat_owned)
        physical_release_route(a);
    if (a->goal == AI_GOAL_FOLLOW && a->target == target_ent) {
        /* Same-target re-issue: land the new speed/xoff but keep every
         * bit of mover state (position, heading, at_follow) — D-A5's
         * goto/race rule applied to follow (D-A15). */
        a->xoff  = xoff_m;
        a->speed = speed > 0.0 ? clamp_speed(speed) : AI_FOLLOW_SPEED;
        a->rival = -1;
        if (!combat_owned)
            a->combat_seek = 0;
        return;
    }
    free(a->path);
    a->path    = NULL;
    a->path_n  = 0;
    a->path_id = -1;
    a->goal    = AI_GOAL_FOLLOW;
    a->stationary_path = 0;
    a->sat     = 0;         /* D-A17 */
    a->plan_n  = 0;         /* D-A13: a fresh follow drops any prior plan */
    a->plan_idx = 0;
    a->plan_leg_wp = -1;
    a->plan_dirty  = 1;
    a->f_plan_on = 0;       /* D-A22: a fresh chase starts direct */
    a->f_stall   = 0;
    a->f_best_d2 = 1e30;
    a->target  = target_ent;
    a->rival  = -1;
    a->xoff    = xoff_m;
    a->follow_hold_dist = 0.0;
    a->combat_seek = combat_owned ? 1 : 0;
    a->combat_dest = 0;
    a->turn_state = 0;
    a->speed   = speed > 0.0 ? clamp_speed(speed) : AI_FOLLOW_SPEED;
    a->arrived = 0;
    a->at_follow = 0;
}

void ai_follow(int ent, int target_ent, double xoff_m, double speed)
{
    follow_set(ent, target_ent, xoff_m, speed, 0);
}

void ai_combat_chase(int ent, int target_ent, double speed)
{
    /* attack() arms behavior 0; FUN_004152e0 picks dest 1/3/10/6/9/8.
     * Open-field dests 6/8/9 run here; dest 17 (FUN_0040aa20) remains the
     * recovery tick and the dest-15 fallback. A probe-forced dest set
     * before the first chase (target still -1) is kept; a new target
     * re-picks. */
    AiAgent *a = agent_at(ent);
    int keep = 0;
    if (a && a->combat_dest != 0 &&
        (a->target == target_ent || a->target < 0))
        keep = a->combat_dest;
    follow_set(ent, target_ent, 0.0, speed, 1);
    a = agent_at(ent);
    if (a) {
        /* Stage 3 authority handoff: combat owns this FOLLOW. Seed car.c
         * exactly once from the current route pose; later reissues retain the
         * integrated context. A missing VCF context fails closed to kinematic. */
        (void)physical_activate(a, 0);
        a->combat_seek = 1;
        a->follow_hold_dist = 0.0;
        if (keep)
            apply_combat_dest(a, keep);
        else if (a->combat_dest == 0)
            apply_combat_dest(a, pick_init_dest(a));
    }
}

void ai_sit(int ent)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return;
    physical_release_route(a);
    /* FACT (fsm.md §4.2): sit brakes to a stop and clears the AI path.
     * D-A17: the park is also the weapons-safe state — the original's sit
     * is behavior 31 (brake-and-hold, FUN_00417570), which carries no
     * target, so a sitting entity never fires there. combat.c reads this
     * flag to keep D-C15's fallback from arming a parked car (P02's clown
     * sits through the whole intro cutscene; any goal verb clears it). */
    a->sat     = 1;
    free(a->path);
    a->path    = NULL;
    a->path_n  = 0;
    a->path_id = -1;
    a->goal    = AI_GOAL_NONE;
    a->stationary_path = 0;
    a->plan_n  = 0;         /* D-A13: sit drops any planned leg */
    a->plan_idx = 0;
    a->plan_leg_wp = -1;
    a->plan_dirty  = 1;
    a->target  = -1;
    a->rival   = -1;
    a->arrived = 0;
    a->at_follow = 0;
    a->contact_vx = 0.0;
    a->contact_vz = 0.0;
    a->drive_speed = 0.0;
    a->combat_steer_bias = 0.0;
    a->combat_bias_pending = 0;
    a->route_bias_contact = 0;
    a->route_bias_hold = 0;
    a->combat_dest = 0;
    a->turn_state = 0;
}

int ai_is_arrived(int ent)
{
    AiAgent *a = agent_at(ent);
    int r;
    if (!a)
        return 0;
    r = a->arrived;
    a->arrived = 0;     /* self-clearing pulse (FACT fsm.md §4.2 :348) */
    return r;
}

int ai_at_follow(int ent)
{
    AiAgent *a = agent_at(ent);
    return a ? a->at_follow : 0;
}

int ai_get_pos(int ent, double out[3])
{
    AiAgent *a = agent_at(ent);
    if (!a || !out)
        return -1;
    out[0] = a->x;
    out[1] = a->y;
    out[2] = a->z;
    return 0;
}

void ai_sync_idle_pose(int ent, double x, double z, double heading)
{
    AiAgent *a = agent_at(ent);
    if (!a || a->goal != AI_GOAL_NONE || a->arrived ||
        !isfinite(x) || !isfinite(z) || !isfinite(heading))
        return;
    a->x = x;
    a->z = z;
    a->heading = ang_wrap(heading);
    a->heading_set = 1;
}

void ai_set_height(int ent, double y)
{
    AiAgent *a = agent_at(ent);
    if (!a || !isfinite(y))
        return;
    if (a->physical_authority && a->shadow) {
        CarLive live = {0};
        car_context_get_live(a->shadow, &live);
        live.y = y;
        car_context_set_live(a->shadow, &live);
        physical_sync(a);
    } else {
        a->y = y;
    }
}

/* D-A17 correction recovery with H-UAT-067e hysteresis. Ordinary corrections
 * retain D-A17's immediate replan. Only a GOTO carrying combat's under-fire
 * weave keeps its query-validated plan through sub-metre noise; a material
 * correction still replans. Goal/waypoint/speed/arrival remain untouched. */
static void plan_corrected(AiAgent *a, double dx, double dz)
{
    if (a->goal != AI_GOAL_GOTO &&
        !(a->goal == AI_GOAL_FOLLOW && a->f_plan_on))
        return;
    if (a->route_bias_hold && hypot(dx, dz) < AI_PLAN_REPLAN_DRIFT)
        return;
    a->plan_leg_wp = -1;
    a->plan_dirty = 1;
}

static void translate_selected_owner(AiAgent *a, double dx, double dz)
{
    if (a->physical_authority && a->shadow) {
        CarLive live = {0};
        car_context_get_live(a->shadow, &live);
        live.x += dx;
        live.z += dz;
        car_context_set_live(a->shadow, &live);
        physical_sync(a);
    } else {
        a->x += dx;
        a->z += dz;
    }
    if (dx != 0.0 || dz != 0.0)
        plan_corrected(a, dx, dz);
}

void ai_translate_xz(int ent, double dx, double dz)
{
    AiAgent *a = agent_at(ent);
    if (a)
        translate_selected_owner(a, dx, dz);
}

void ai_constrain_xz(int ent, double dx, double dz)
{
    AiAgent *a = agent_at(ent);
    if (a)
        translate_selected_owner(a, dx, dz);
}

void ai_add_contact_velocity(int ent, double dvx, double dvz)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return;
    if (a->physical_authority && a->shadow) {
        /* D-A11's computed half impulse and D-C20's marked external impulse
         * both enter car.c's velocity owner exactly once. Never mirror either
         * into contact_v*. */
        car_context_add_world_velocity(a->shadow, dvx, dvz);
        physical_sync(a);
        return;
    }
    a->contact_vx += dvx;
    a->contact_vz += dvz;
    double speed = sqrt(a->contact_vx * a->contact_vx +
                        a->contact_vz * a->contact_vz);
    if (speed > AI_SPEED_MAX) {
        double scale = AI_SPEED_MAX / speed;
        a->contact_vx *= scale;
        a->contact_vz *= scale;
    }
}

double ai_contact_speed(int ent)
{
    AiAgent *a = agent_at(ent);
    if (a && a->physical_authority && a->shadow) {
        double vx = 0.0, vz = 0.0;
        car_context_world_velocity(a->shadow, &vx, &vz);
        return hypot(vx, vz);
    }
    return a ? hypot(a->contact_vx, a->contact_vz) : 0.0;
}

int ai_goal(int ent)
{
    AiAgent *a = agent_at(ent);
    return a ? a->goal : AI_GOAL_NONE;
}

int ai_stationary_path(int ent)
{
    const AiAgent *a = agent_at(ent);
    return a && a->goal == AI_GOAL_GOTO && a->stationary_path;
}

int ai_has_raced(int ent)
{
    AiAgent *a = agent_at(ent);
    return a ? a->has_raced : 0;
}

int ai_race_active(int ent)
{
    const AiAgent *a = agent_at(ent);
    /* Arrival is produced after this tick's FSM slice, but combat runs later
     * in the same tick. Keep race ownership through that pending pulse so an
     * autonomous engagement cannot fire in the one-tick gap before the script
     * consumes isArrived and reissues/changes the authored behavior. */
    return a && a->rival >= 0 &&
           (a->goal == AI_GOAL_GOTO || a->arrived);
}

int ai_wp(int ent)
{
    AiAgent *a = agent_at(ent);
    return a ? a->wp : 0;
}

int ai_path_id(int ent)
{
    const AiAgent *a = agent_at(ent);
    return a ? a->path_id : -1;
}

int ai_route_trace(int ent, double authored_target[2],
                   double plan_target[2], int *plan_index, int *plan_count)
{
    const AiAgent *a = agent_at(ent);
    if (!a || !authored_target || !plan_target || !plan_index || !plan_count ||
        !a->path || a->wp < 0 || a->wp >= a->path_n)
        return -1;
    authored_target[0] = a->path[a->wp * 3];
    authored_target[1] = a->path[a->wp * 3 + 2];
    *plan_index = a->plan_idx;
    *plan_count = a->plan_n;
    if (a->plan_idx >= 0 && a->plan_idx < a->plan_n) {
        plan_target[0] = a->plan_x[a->plan_idx];
        plan_target[1] = a->plan_z[a->plan_idx];
    } else {
        plan_target[0] = authored_target[0];
        plan_target[1] = authored_target[1];
    }
    return 0;
}

void ai_seed_heading(int ent, double yaw)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return;
    a->heading = ang_wrap(yaw);
    a->heading_set = 1;
}

double ai_get_heading(int ent)
{
    const AiAgent *a = agent_at(ent);
    return a ? a->heading : 0.0;
}

void ai_set_follow_hold_distance(int ent, double meters)
{
    AiAgent *a = agent_at(ent);
    if (!a)
        return;
    a->follow_hold_dist = meters > AI_FOLLOW_DIST ? meters : 0.0;
}

void ai_set_steer_bias(int ent, double radians)
{
    AiAgent *a = agent_at(ent);
    if (!a || !isfinite(radians))
        return;
    if (radians > 0.60) radians = 0.60;
    if (radians < -0.60) radians = -0.60;
    a->combat_steer_bias = radians;
    a->combat_bias_pending = 1;
}

void ai_apply_grip_loss(int ent, double scale, int ticks)
{
    AiAgent *a = agent_at(ent);
    if (!a || !isfinite(scale) || !(scale > 0.0) || scale > 1.0 || ticks <= 0)
        return;
    if (a->grip_ticks <= 0 || scale < a->grip_scale)
        a->grip_scale = scale;
    if (ticks > a->grip_ticks)
        a->grip_ticks = ticks;
    if (a->shadow)
        car_context_apply_grip_loss(a->shadow, scale, ticks);
}

int ai_grip_loss_ticks(int ent)
{
    AiAgent *a = agent_at(ent);
    return a ? a->grip_ticks : 0;
}

int ai_arrivals(int ent)
{
    AiAgent *a = agent_at(ent);
    return a ? a->arrivals : 0;
}

int ai_sat(int ent)
{
    const AiAgent *a = agent_at(ent);
    return a ? a->sat : 0;
}

void ai_clear_sat(int ent)
{
    AiAgent *a = agent_at(ent);
    if (a)
        a->sat = 0;
}

int ai_arrival_pending(int ent)
{
    const AiAgent *a = agent_at(ent);
    /* Non-consuming read (unlike ai_is_arrived): combat's chase must be
     * able to see a pending pulse WITHOUT eating it — the pulse belongs
     * to the script's next machine slice. */
    return a ? a->arrived : 0;
}

/* ---- D-A13 grid planner ---------------------------------------------
 * Everything in this block is CONFIRMED Nitro mechanics (FUN_0040ff30
 * search, FUN_0040f720 edge costs) EXCEPT the clearly-marked port items:
 * the AiNavSample.occupied substitute for the unported predictive vehicle
 * probes, the query-failure impassable rule, and the D-A14 string-pull
 * smoother applied to the reconstruction. The optional ninth RSEG-derived
 * neighbor and the marker stage remain unresolved RE and are deliberately
 * absent.
 *
 * Search state is one reusable static workspace — searches run
 * sequentially inside ai_tick, so no allocation ever happens at tick
 * time. Planned routes live in the agent's own reusable buffers. */

typedef struct {
    int   x, z;     /* quantized grid coordinates (m)      */
    double g, f;    /* g = seed-to-node cost; f = g + h    */
    int   parent;   /* workspace index, -1 at the seed     */
    unsigned char open; /* 1 while on the open list        */
} AiNavNode;

static AiNavNode s_nav_nodes[AI_NAV_MAX_NODES];
static int       s_nav_n;
static int       s_nav_route[AI_NAV_MAX_NODES]; /* reconstruction scratch */
#define AI_NAV_HASH 2048                        /* > 2x node pool         */
static unsigned short s_nav_hash[AI_NAV_HASH];  /* node index+1, 0 = empty */

/* CONFIRMED fixed 8-neighbor order (dx,dz); no diagonal corner
 * rejection. The unresolved optional ninth RSEG neighbor is NOT added. */
static const int s_nav_nb[8][2] = {
    { -10, -10 }, { 0, -10 }, { 10, -10 }, { -10, 0 },
    { 10, 0 }, { -10, 10 }, { 0, 10 }, { 10, 10 }
};

/* CONFIRMED: nearest-10 quantization, strict remainder > 5 rounds up
 * (_DAT_004c19a8 = 5.0). Native worlds are positive; the negative branch
 * is the symmetric extension for out-of-bounds positions. */
static int nav_q10(double v)
{
    double r = fmod(v, AI_NAV_GRID);
    int q = (int)(v - r);
    if (r > 5.0)
        q += 10;
    else if (r < -5.0)
        q -= 10;
    return q;
}

/* CONFIRMED octile heuristic: max(dx,dz) + (sqrt(2)-1)*min(dx,dz)
 * against the raw (unquantized) target. */
static double nav_h(int x, int z, double tx, double tz)
{
    double dx = fabs((double)x - tx);
    double dz = fabs((double)z - tz);
    double mn = dx < dz ? dx : dz;
    double mx = dx > dz ? dx : dz;
    return mx + (AI_NAV_SQRT2 - 1.0) * mn;
}

/* CONFIRMED edge cost (FUN_0040f720): the DESTINATION surface class
 * prices the edge — 0.5 for classes 2/3/4/6/7, else 15.0 plus 1e6 only
 * when the agent's avoid flag is armed AND the destination probes
 * blocked; the sum scales by sqrt(2) on diagonal edges. */
static double nav_edge_cost(const AiNavSample *s, int avoid, int diagonal)
{
    double c;
    unsigned cls = s->surface_class;
    if (cls == 2 || cls == 3 || cls == 4 || cls == 6 || cls == 7) {
        c = AI_NAV_COST_CHEAP;
    } else {
        c = AI_NAV_COST_OTHER;
        if (avoid && s->blocked)
            c += AI_NAV_COST_BLOCK;
    }
    return diagonal ? c * AI_NAV_SQRT2 : c;
}

static unsigned nav_hash(int x, int z)
{
    unsigned h = (unsigned)(x / 10) * 0x9E3779B1u ^
                 (unsigned)(z / 10) * 0x85EBCA6Bu;
    h ^= h >> 13;
    return h & (AI_NAV_HASH - 1);
}

static int nav_find(int x, int z)
{
    unsigned i = nav_hash(x, z);
    while (s_nav_hash[i]) {
        const AiNavNode *n = &s_nav_nodes[s_nav_hash[i] - 1];
        if (n->x == x && n->z == z)
            return s_nav_hash[i] - 1;
        i = (i + 1) & (AI_NAV_HASH - 1);
    }
    return -1;
}

static int nav_add(int x, int z)
{
    unsigned i;
    if (s_nav_n >= AI_NAV_MAX_NODES)
        return -1;                  /* CONFIRMED: 1000-node pool cap */
    i = nav_hash(x, z);
    while (s_nav_hash[i])
        i = (i + 1) & (AI_NAV_HASH - 1);
    s_nav_hash[i] = (unsigned short)(s_nav_n + 1);
    s_nav_nodes[s_nav_n].x = x;
    s_nav_nodes[s_nav_n].z = z;
    return s_nav_n++;
}

/* ---- D-A14 string-pull smoothing -------------------------------------
 * PORT APPROXIMATION (no native parity claim): the native smoother's
 * shape is CONFIRMED (FUN_0040f090 — corner classification, an A*
 * shortcut at sharp corners, a terrain-validated probe march) but its
 * exact construction is unresolved RE. This conservative local string-pull
 * limits each shortcut to four adjacent grid edges: enough to collapse a
 * short staircase without replacing a road-following run by a whole-stage
 * cross-country chord. Each candidate is sampled through the nav query in
 * <= AI_NAV_GRID pieces and accepted only when its cost does not exceed the
 * original edges. */

/* Price one <= AI_NAV_GRID-sampled chord. Sets *ok = 0 — the chord must
 * be rejected — on query failure, on an occupied piece, or on a blocked
 * piece while avoid is armed. Chord pieces price length-proportionally at
 * the confirmed destination-surface rates (PORT RULE: the native model
 * prices grid edges only, so the chord price is the port's construction;
 * the confirmed blocked addend is subsumed by outright rejection while
 * avoid is armed). */
static double nav_chord_cost(const AiAgent *a, int ent, AiNavQueryFn query,
                             double x0, double z0, double x1, double z1,
                             int *ok)
{
    double dx = x1 - x0, dz = z1 - z0;
    double len = sqrt(dx * dx + dz * dz);
    int pieces = (int)ceil(len / AI_NAV_GRID);
    double cost = 0.0;
    *ok = 1;
    if (pieces < 1)
        pieces = 1;
    for (int k = 0; k < pieces; k++) {
        double t0 = (double)k / pieces, t1 = (double)(k + 1) / pieces;
        AiNavSample s;
        if (query) {
            if (query(ent, x0 + dx * t0, z0 + dz * t0,
                      x0 + dx * t1, z0 + dz * t1, &s) != 0) {
                *ok = 0;        /* query failure rejects the chord */
                return 0.0;
            }
        } else {
            s.surface_class = 2; /* default cheap/passable sample */
            s.blocked = 0;
            s.occupied = 0;
        }
        if (s.occupied || (a->avoid && s.blocked)) {
            *ok = 0;
            return 0.0;
        }
        cost += nav_edge_cost(&s, 0, 0) * ((t1 - t0) * len / AI_NAV_GRID);
    }
    return cost;
}

/* A reconstructed edge is a true grid edge (CONFIRMED per-edge pricing)
 * only when both endpoints sit on the 10 m lattice one pitch apart. Grid
 * coordinates are exact integer doubles, so the fmod tests are exact. */
static int nav_is_grid_edge(double x0, double z0, double x1, double z1)
{
    double dx = fabs(x1 - x0), dz = fabs(z1 - z0);
    return fmod(x0, AI_NAV_GRID) == 0.0 && fmod(z0, AI_NAV_GRID) == 0.0 &&
           fmod(x1, AI_NAV_GRID) == 0.0 && fmod(z1, AI_NAV_GRID) == 0.0 &&
           (dx == 0.0 || dx == AI_NAV_GRID) &&
           (dz == 0.0 || dz == AI_NAV_GRID) && dx + dz > 0.0;
}

/* Sampled cost of one reconstructed subpath edge: the confirmed per-edge
 * price for a grid edge, chord pricing for the raw-target append. Sets
 * *ok = 0 when the edge cannot be priced (query failure) — a candidate
 * replacing it then keeps the original subpath. */
static double nav_replaced_edge_cost(const AiAgent *a, int ent,
                                     AiNavQueryFn query,
                                     double x0, double z0,
                                     double x1, double z1, int *ok)
{
    if (nav_is_grid_edge(x0, z0, x1, z1)) {
        AiNavSample s;
        int diag = x1 != x0 && z1 != z0;
        *ok = 1;
        if (query) {
            if (query(ent, x0, z0, x1, z1, &s) != 0) {
                *ok = 0;
                return 0.0;
            }
        } else {
            s.surface_class = 2;
            s.blocked = 0;
            s.occupied = 0;
        }
        return nav_edge_cost(&s, a->avoid, diag);
    }
    return nav_chord_cost(a, ent, query, x0, z0, x1, z1, ok);
}

/* Greedy farthest-visible string pull over the reconstructed route. The
 * reconstructed start point and the raw final target are never removed. */
static void nav_smooth(AiAgent *a, int ent, AiNavQueryFn query)
{
    static double sm_x[AI_PLAN_MAX], sm_z[AI_PLAN_MAX]; /* tick scratch */
    double edge_cost[AI_PLAN_MAX];
    int    edge_ok[AI_PLAN_MAX];
    int n = a->plan_n, m, i;
    if (n < 3)
        return;
    for (i = 0; i + 1 < n; i++)
        edge_cost[i] = nav_replaced_edge_cost(a, ent, query,
                                              a->plan_x[i], a->plan_z[i],
                                              a->plan_x[i + 1],
                                              a->plan_z[i + 1],
                                              &edge_ok[i]);
    m = 0;
    i = 0;
    sm_x[m] = a->plan_x[0];
    sm_z[m] = a->plan_z[0];
    m++;
    while (i < n - 1) {
        int best = i + 1;
        for (int j = i + 4 < n ? i + 4 : n - 1; j > i + 1; j--) {
            double cc, rc = 0.0;
            int ok = 1, k;
            for (k = i; k < j; k++) {
                if (!edge_ok[k])
                    break;
                rc += edge_cost[k];
            }
            if (k < j)
                continue;    /* unpriceable subpath: keep the original */
            cc = nav_chord_cost(a, ent, query,
                                a->plan_x[i], a->plan_z[i],
                                a->plan_x[j], a->plan_z[j], &ok);
            if (ok && cc <= rc + 1e-9) {
                best = j;
                break;
            }
        }
        i = best;
        sm_x[m] = a->plan_x[i];
        sm_z[m] = a->plan_z[i];
        m++;
    }
    memcpy(a->plan_x, sm_x, (size_t)m * sizeof *sm_x);
    memcpy(a->plan_z, sm_z, (size_t)m * sizeof *sm_z);
    a->plan_n = m;
}


/* A usable plan drives the agent: it targets the CURRENT authored wp and
 * still has an unconsumed point. */
static int plan_usable(const AiAgent *a)
{
    return a->plan_leg_wp == a->wp && a->plan_idx < a->plan_n;
}

/* H-UAT-076a — PORT CONVENTION pending Nitro's unresolved RSEG-derived
 * planner neighbor/road-object preprocessing. The confirmed 10 m A* may put
 * a coarse local point outside a rendered authored road ribbon even though
 * the ribbon is inside that point's own confirmed <11 m goal region.
 * In the mission unity build, move such non-final progress points to the
 * nearest authored ribbon boundary plus the existing bounded half-grid inset
 * (not all the way to its centre, which can erase forward progress near an
 * intersection), and only when both adjoining replacement chords pass the
 * ordinary nav query. The one-grid-neighborhood bound prevents a genuinely
 * off-road authored route being captured by a distant road. The plan's last
 * point is never moved, preserving bounded-stage, D-A2, and exact-zero GOTO
 * semantics. Standalone synthetic ai.c probes have no terrain/RSEG owner and
 * deliberately retain the unmodified planner. */
static int route_has_authored_race(const AiAgent *a)
{
    if (a->path_id < 0)
        return 0;
    for (int i = 0; i < AI_MAX_AGENTS; i++) {
        const AiAgent *r = &s_agents[i];
        if (r->used && r->goal == AI_GOAL_GOTO && r->rival >= 0 &&
            r->path_id == a->path_id)
            return 1;
    }
    return 0;
}

static void nav_seat_nearby_road(AiAgent *a, int ent, AiNavQueryFn query)
{
#ifdef TERRAIN_H
    /* The plan's last point owns bounded-stage/authored-waypoint progress.
     * Never move it: seating it can leave the mover inside the seated point's
     * <11 m capture region but still outside the authored target's region,
     * repeatedly consuming/replanning without forward motion. */
    for (int i = 1; i + 1 < a->plan_n; i++) {
        double near[2], tangent[2], seated[2];
        double clear;
        int before_ok = 1, after_ok = 1;
        clear = terrain_road_nearest(a->plan_x[i], a->plan_z[i], NULL,
                                     near, tangent);
        if (!(clear > 0.0) || clear * clear >= AI_NAV_GOAL_D2)
            continue;
        seated[0] = a->plan_x[i];
        seated[1] = a->plan_z[i];
        /* terrain_road_nearest supplies centreline + exact signed ribbon
         * clearance. Advance toward that known-inside centre by exactly the
         * remaining outside clearance; repeat a bounded four times for
         * non-radial trapezoid edges. Accept only a measured inside result. */
        for (int k = 0; k < 4 && clear > 0.0; k++) {
            double vx = near[0] - seated[0], vz = near[1] - seated[1];
            double d = hypot(vx, vz);
            if (!(d > clear))
                break;
            seated[0] += vx * (clear / d);
            seated[1] += vz * (clear / d);
            clear = terrain_road_nearest(seated[0], seated[1], NULL,
                                         near, tangent);
        }
        if (clear > 1e-6)
            continue;
        /* Put the vehicle centre, not merely a point-mass wheel, inside the
         * rendered ribbon. Reuse D-A15's existing half-grid progress scale as
         * the bounded inset; clamp at the centre so narrow ribbons cannot
         * overshoot onto the opposite shoulder. */
        {
            double vx = near[0] - seated[0], vz = near[1] - seated[1];
            double d = hypot(vx, vz);
            double inset = d < AI_PASS_DIST ? d : AI_PASS_DIST;
            if (d > 1e-9) {
                seated[0] += vx * (inset / d);
                seated[1] += vz * (inset / d);
            }
        }
        (void)nav_chord_cost(a, ent, query,
                             a->plan_x[i - 1], a->plan_z[i - 1],
                             seated[0], seated[1], &before_ok);
        if (i + 1 < a->plan_n)
            (void)nav_chord_cost(a, ent, query, seated[0], seated[1],
                                 a->plan_x[i + 1], a->plan_z[i + 1],
                                 &after_ok);
        if (before_ok && after_ok) {
            a->plan_x[i] = seated[0];
            a->plan_z[i] = seated[1];
        }
    }
#else
    (void)a; (void)ent; (void)query;
#endif
}

/* D-A21 stunt legs.  P01's navjump sends enemies up a ~12 deg ramp and
 * off a cliff lip: the D-A13 query's fixed 5 m sample pitch calls both
 * the ramp (1.1 m rise per 5 m) and the lip (4.8 m drop) impassable, so
 * every route to the waypoint is unroutable and the budgeted A* chases
 * partial frontiers AWAY from it forever — round 2 proved the mission's
 * own actor (enemy3) wedges in the sealed northern town and P01 becomes
 * unwinnable once D24c stops the player from simply climbing the cliff
 * to it.  The D-A12 ground stage itself is a RATE cap (per-tick move
 * samples), which admits the same line at mover speeds — the original's
 * NPCs are physical cars that just drive the authored line.  So: a goto
 * leg whose straight authored line contains ANY over-gradient piece at
 * the query pitch is a stunt leg — unplannable BY CONSTRUCTION — and is
 * driven directly, still gated per tick by the D-A12 stage (with drops
 * admitted as authored falls; climbs keep the step rule).  Scripted
 * GOTO legs only: chases (FOLLOW) never use this. */
static int leg_stunt(const AiAgent *a, int ent, AiNavQueryFn query,
                     double tx, double tz)
{
    double dx = tx - a->x, dz = tz - a->z;
    double d = sqrt(dx * dx + dz * dz);
    int n, k;
    if (d < 1e-6 || !query)
        return 0;
    n = (int)(d / AI_NAV_GRID) + 1;     /* planner-pitch pieces (<=10 m) */
    for (k = 1; k <= n; k++) {
        double t0 = (double)(k - 1) / n, t1 = (double)k / n;
        AiNavSample s;
        if (query(ent, a->x + dx * t0, a->z + dz * t0,
                  a->x + dx * t1, a->z + dz * t1, &s) != 0)
            return 0;                   /* unknown ground: not a stunt  */
        if (s.cliff)
            return 1;                   /* terrain-step impassable line */
        if (s.occupied)
            return 0;                   /* vehicle/static probe: the
                                         * planner should route around  */
    }
    return 0;
}

/* D-A22: 1 when the straight chase line from the agent toward (tx,tz)
 * contains an impassable piece at the planner sample pitch — the line the
 * D-A12 stage will refuse (cliff step) or the static/vehicle probe rejects
 * (occupied).  Scan bounded to AI_NAV_STAGE, the planner's own staging
 * radius: a wedge is always local, and a refusal beyond the stage cannot
 * be the piece pinning the chaser in place.  Query failure (unknown
 * ground) is NOT a refusal — the planner could not route there either, so
 * the chase stays direct.  Run at most once per AI_STUNT_STALL interval
 * per agent (see ai_tick), never every tick. */
static int follow_line_refused(const AiAgent *a, int ent, AiNavQueryFn query,
                               double tx, double tz)
{
    double dx = tx - a->x, dz = tz - a->z;
    double d = sqrt(dx * dx + dz * dz);
    int n, k;
    if (d < 1e-6 || !query)
        return 0;
    if (d > AI_NAV_STAGE) {
        dx *= AI_NAV_STAGE / d;
        dz *= AI_NAV_STAGE / d;
        d = AI_NAV_STAGE;
    }
    n = (int)(d / AI_NAV_GRID) + 1;     /* planner-pitch pieces (<=10 m) */
    for (k = 1; k <= n; k++) {
        double t0 = (double)(k - 1) / n, t1 = (double)k / n;
        AiNavSample s;
        if (query(ent, a->x + dx * t0, a->z + dz * t0,
                  a->x + dx * t1, a->z + dz * t1, &s) != 0)
            return 0;                   /* unknown ground: stay direct */
        if (s.occupied)
            return 1;                   /* cliff step or static probe */
    }
    return 0;
}

/* D-A24: 1 when the one-tick look-ahead is a native-style refuse —
 * occupied, or blocked while this agent's avoid flag is armed. Query
 * failure is unknown ground: do not invent a dodge. */
static int whisker_blocked(const AiAgent *a, int ent, AiNavQueryFn query,
                           double x0, double z0, double x1, double z1)
{
    AiNavSample s;
    if (!query || query(ent, x0, z0, x1, z1, &s) != 0)
        return 0;
    return s.occupied || (a->avoid && s.blocked);
}

/* D-A24: combat-follow only. Forward-clear desired headings are returned
 * unchanged. A blocked look-ahead keeps turning by the smallest
 * ±0.25..±1.0 steer whose look-ahead is clear; if none are, one
 * unit-left step so a long slab is not a terminal wedge. */
static double whisker_desired(AiAgent *a, int ent, AiNavQueryFn query,
                              double desired)
{
    double look, hx, hz, best, best_mag, mag, turn;
    int best_sign;
    if ((!a->combat_seek && a->follow_hold_dist <= AI_FOLLOW_DIST) ||
        !a->avoid || !query)
        return desired;
    look = AI_WHISKER_RANGE;
    hx = -sin(desired);
    hz = cos(desired);
    if (!whisker_blocked(a, ent, query, a->x, a->z,
                         a->x + hx * look, a->z + hz * look))
        return desired;
    /* Target line is blocked: keep turning (native holds e0) by the
     * smallest steer whose look-ahead from the live heading is clear.
     * Re-aiming at desired+epsilon every tick cannot accumulate a dodge. */
    best = desired;
    best_mag = 1e9;
    best_sign = 0;
    turn = AI_TURN_MAX * fmax((double)a->steer_gain, 0.0) * AI_TICK_DT;
    if (turn < 1e-9)
        return desired;
    for (mag = AI_WHISKER_STEP; mag <= AI_WHISKER_LIMIT + 1e-9;
         mag += AI_WHISKER_STEP) {
        int sign;
        for (sign = -1; sign <= 1; sign += 2) {
            double cand = ang_wrap(a->heading + (double)sign * mag * turn);
            double cx = -sin(cand), cz = cos(cand);
            if (mag < best_mag &&
                !whisker_blocked(a, ent, query, a->x, a->z,
                                 a->x + cx * look, a->z + cz * look)) {
                best_mag = mag;
                best_sign = sign;
                best = cand;
            }
        }
    }
    if (best_sign == 0) {
        /* No clear candidate: still turn one unit-left step so a long
         * slab is not a terminal wedge. Sign is stable (always −). */
        best = ang_wrap(a->heading - turn);
    }
    return best;
}

/* D-A21 accessor for the mission ground stage: 1 while `ent` is actively
 * driving a stunt goto leg (the ai_tick engagement logic owns the
 * decision — see there). */
int ai_goto_stunt_descent(int ent)
{
    AiAgent *a = agent_at(ent);
    return a != NULL && a->goal == AI_GOAL_GOTO && a->path_stunt_ok &&
           a->leg_stunt_on;
}

/* Plan toward one bounded target on the current authored leg. Nitro's route
 * preprocessing supplies local route targets before its fixed 100-expansion
 * A*; that complete stage is not ported. AI_NAV_STAGE is the generic port
 * substitute: half the search radius is progress, half remains available for
 * surface/static detours. A complete search reconstructs through its bounded
 * target. A budget-limited search reconstructs only through the next node
 * selected by the confirmed open-list ordering, then replans after consuming
 * that safe frontier. Index 0 remains the quantized search origin (the A*
 * seed), which D-A17 consumes before actuation so it is never a physical
 * drive target. Open-list exhaustion returns -1 and leaves any prior plan
 * untouched; no path falls back to a raw straight line. */
static int nav_plan(AiAgent *a, int ent, AiNavQueryFn query,
                    double tx, double tz)
{
    int expansions = 0;             /* main-loop expansions after seed */
    int seed_done = 0;
    int goal = -1;
    int partial = 0;
    int frontier = -1;
    double gx = tx, gz = tz;
    double leg_dx = tx - a->x, leg_dz = tz - a->z;
    double leg_d = sqrt(leg_dx * leg_dx + leg_dz * leg_dz);
    if (leg_d > AI_NAV_STAGE) {
        gx = a->x + leg_dx / leg_d * AI_NAV_STAGE;
        gz = a->z + leg_dz / leg_d * AI_NAV_STAGE;
    }

    s_nav_n = 0;
    memset(s_nav_hash, 0, sizeof s_nav_hash);
    {
        /* CONFIRMED: the start quantizes to the nearest 10 (strict
         * remainder > 5). The target stays raw/unquantized; gx/gz is the
         * current local route target supplied by the port staging rule.
         * Seed g = f = 0. */
        int sx = nav_q10(a->x);
        int sz = nav_q10(a->z);
        int seed = nav_add(sx, sz);
        s_nav_nodes[seed].g = 0.0;
        s_nav_nodes[seed].f = 0.0;
        s_nav_nodes[seed].parent = -1;
        s_nav_nodes[seed].open = 1;
    }
    for (;;) {
        int best = -1;
        double bf = 0.0, bk = 0.0;
        AiNavNode *n;
        double dx, dz;
        int k;
        /* CONFIRMED open ordering: lowest f, ties on z*100000+x. */
        for (int j = 0; j < s_nav_n; j++) {
            double key;
            if (!s_nav_nodes[j].open)
                continue;
            key = (double)s_nav_nodes[j].z * 100000.0 +
                  (double)s_nav_nodes[j].x;
            if (best < 0 || s_nav_nodes[j].f < bf ||
                (s_nav_nodes[j].f == bf && key < bk)) {
                best = j;
                bf = s_nav_nodes[j].f;
                bk = key;
            }
        }
        if (best < 0)
            break;                  /* open list empty: failure */
        n = &s_nav_nodes[best];
        n->open = 0;
        dx = (double)n->x - gx;
        dz = (double)n->z - gz;
        /* CONFIRMED goal test: squared distance to the unquantized local
         * target below 121.0 (the <11 m region). */
        if (dx * dx + dz * dz < AI_NAV_GOAL_D2) {
            goal = best;
            break;
        }
        /* CONFIRMED budget: the seed expansion is free, then at most
         * 100 main-loop expansions. */
        if (seed_done) {
            if (expansions >= AI_NAV_MAX_EXPAND) {
                frontier = best;
                break;
            }
            expansions++;
        } else {
            seed_done = 1;
        }
        for (k = 0; k < 8; k++) {
            int nx = n->x + s_nav_nb[k][0];
            int nz = n->z + s_nav_nb[k][1];
            int diagonal = s_nav_nb[k][0] != 0 && s_nav_nb[k][1] != 0;
            AiNavSample s;
            double ng;
            int j2;
            if (query) {
                if (query(ent, (double)n->x, (double)n->z,
                          (double)nx, (double)nz, &s) != 0)
                    continue;       /* PORT RULE: query failure is
                                     * impassable */
            } else {
                s.surface_class = 2; /* default cheap/passable sample */
                s.blocked = 0;
                s.occupied = 0;
            }
            if (s.occupied)
                continue;           /* PORT SUBSTITUTE for the native
                                     * predictive vehicle probes:
                                     * always impassable, no cost */
            /* CONFIRMED: child g = parent g + destination edge cost. */
            ng = n->g + nav_edge_cost(&s, a->avoid, diagonal);
            j2 = nav_find(nx, nz);
            if (j2 < 0) {
                j2 = nav_add(nx, nz);
                if (j2 < 0)
                    continue;       /* node pool full */
                s_nav_nodes[j2].g = ng;
                s_nav_nodes[j2].parent = best;
                s_nav_nodes[j2].open = 1;
            } else if (ng < s_nav_nodes[j2].g) {
                /* CONFIRMED: strict lower-g replacement only. */
                s_nav_nodes[j2].g = ng;
                s_nav_nodes[j2].parent = best;
                s_nav_nodes[j2].open = 1;
            } else {
                continue;
            }
            s_nav_nodes[j2].f = s_nav_nodes[j2].g + nav_h(nx, nz, gx, gz);
        }
    }
    if (goal < 0) {
        /* PORT SUBSTITUTE for the unported native route-preprocessing
         * stage: when the fixed search budget expires, keep the next node
         * selected by the confirmed open-list ordering. Its parent chain is
         * fully query-validated, and consuming it lets the next bounded
         * search continue toward the same authored waypoint. Open-list
         * exhaustion still means genuinely no route. */
        if (frontier < 0)
            return -1;
        goal = frontier;
        partial = 1;
    }
    {
        /* Reconstruct start-to-goal. A complete search appends its raw local
         * target; a partial bounded search ends at its reachable frontier so
         * the mover never gets an unvalidated straight-line segment. */
        int len = 0, j, k = 0;
        for (j = goal; j >= 0; j = s_nav_nodes[j].parent)
            s_nav_route[len++] = j;
        if (len + (partial ? 0 : 1) > AI_PLAN_MAX)
            len = AI_PLAN_MAX - (partial ? 0 : 1);
        for (j = len - 1; j >= 0; j--) {
            a->plan_x[k] = (double)s_nav_nodes[s_nav_route[j]].x;
            a->plan_z[k] = (double)s_nav_nodes[s_nav_route[j]].z;
            k++;
        }
        if (!partial) {
            a->plan_x[k] = gx;
            a->plan_z[k] = gz;
            k++;
        }
        a->plan_n = k;
        a->plan_idx = 0;
        a->plan_leg_wp = a->wp;
        a->plan_dirty = 0;
        /* D-A14: string-pull the reconstructed route. Keeps the start and
         * final reconstructed point; validates every chord through the nav
         * query. */
        nav_smooth(a, ent, query);
        /* PORT CONVENTION: an explicit authored race marks its immutable path
         * as road/course-owned; convoy GOTO actors sharing that exact path
         * inherit the same local road target. This is path provenance, not a
         * mission/entity/coordinate special case, and avoids pulling unrelated
         * combat/stunt GOTOs onto any merely nearby road. */
        if (a->goal == AI_GOAL_GOTO && route_has_authored_race(a))
            nav_seat_nearby_road(a, ent, query);
        /* Consume the reconstructed search origin before the actuator
         * runs: index 0 is the quantized A* seed — a search origin and
         * the D-A14 smoothing anchor, never a drive target. An off-grid
         * or displaced start can quantize to a seed behind or side-on;
         * driving back to it is exactly the turnaround loop this forbids.
         * A degenerate one-point plan keeps index 0 — its single point is
         * the drive target, so the mover still drives instead of stalling
         * on an unconsumable plan. */
        if (a->plan_n > 1)
            a->plan_idx = 1;
        a->plan_point_ahead = 0;
    }
    return 0;
}

static void shadow_actuator_tick(AiAgent *a)
{
    CarInput in = {0};
    const AiDriveCommand *c = &a->drive_command;
    if (!a->shadow || !c->valid ||
        (!a->physical_authority && !a->shadow_diagnostic))
        return;
    if (c->steer > 0.0)
        in.left = (float)fmin(c->steer, 1.0);
    else if (c->steer < 0.0)
        in.right = (float)fmin(-c->steer, 1.0);
    in.throttle = (float)fmax(0.0, fmin(c->throttle, 1.0));
    in.brake = (float)fmax(0.0, fmin(c->brake, 1.0));
    in.e_brake = c->e_brake ? 1 : 0;
    if (c->reverse != a->shadow_reverse) {
        in.reverse = 1;                 /* CarInput direction is an edge */
        a->shadow_reverse = c->reverse;
    }
    car_context_step(a->shadow, &in);
    a->shadow_steps++;
    a->shadow_last_state_hash = car_context_state_hash(a->shadow);
    a->shadow_state_trace_hash = trace_hash_bytes(
        a->shadow_state_trace_hash, &a->shadow_last_state_hash,
        sizeof a->shadow_last_state_hash);
    physical_sync(a);
}

void ai_tick(double dt, void (*resolve)(int ent, double out[3]),
             AiNavQueryFn query)
{
    const double contact_decay = exp(-AI_CONTACT_DRAG * dt);
#ifdef I76_AI_PHYSICS_PROBE
    double probe_before_x[AI_MAX_AGENTS];
    double probe_before_z[AI_MAX_AGENTS];
    for (int i = 0; i < AI_MAX_AGENTS; i++) {
        probe_before_x[i] = s_agents[i].x;
        probe_before_z[i] = s_agents[i].z;
        s_agents[i].probe_control_valid = 0;
        s_agents[i].probe_throttle_intent = 0;
        s_agents[i].probe_direct_xz_write = 0;
    }
#endif
    if (dt > 0.0 && isfinite(dt))
        s_ai_clock += dt;
    for (int i = 0; i < AI_MAX_AGENTS; i++) {
        AiAgent *a = &s_agents[i];
        if (!a->used)
            continue;
        /* Physical state from the prior fixed step is the brain's current
         * body state. No mission/scene reconstruction feeds it back. */
        physical_sync(a);
        /* Native sit is steer=0/full brake. It is also the safe default for
         * an idle/unresolved brain; a movement primitive replaces it below. */
        memset(&a->drive_command, 0, sizeof a->drive_command);
        a->drive_command.brake = 1.0;
        a->drive_command.arrival_intent = a->arrived || a->at_follow;
        a->drive_command.valid = 1;
        if (a->grip_ticks > 0 && --a->grip_ticks == 0)
            a->grip_scale = 1.0;
        /* H-UAT-067e: combat weave remains an additive command. Only a GOTO
         * currently carrying that under-fire command retains its plan through
         * sub-metre correction noise; unrelated route/contact behavior stays
         * unchanged. The latch lasts until the associated shove settles. */
        int bias_issued = a->combat_bias_pending;
        int contact_active = a->contact_vx != 0.0 || a->contact_vz != 0.0;
        if (a->goal != AI_GOAL_GOTO)
            a->route_bias_contact = 0;
        else if (bias_issued)
            a->route_bias_contact = 1;
        else if (!contact_active)
            a->route_bias_contact = 0;
        a->route_bias_hold = a->goal == AI_GOAL_GOTO &&
                             (bias_issued || a->route_bias_contact);
        double combat_bias = a->combat_steer_bias;
        a->combat_steer_bias = 0.0;
        a->combat_bias_pending = 0;
        /* D-A17 integrates collision response in world space. Ordinary
         * corrections replan immediately; a route currently rejecting combat
         * weave uses the marked half-cell hysteresis above. */
        if (!a->physical_authority &&
            (a->contact_vx != 0.0 || a->contact_vz != 0.0)) {
            double dx = a->contact_vx * dt;
            double dz = a->contact_vz * dt;
            a->x += dx;
            a->z += dz;
            if (dx != 0.0 || dz != 0.0)
                plan_corrected(a, dx, dz);
            a->contact_vx *= contact_decay;
            a->contact_vz *= contact_decay;
            if (a->contact_vx * a->contact_vx +
                a->contact_vz * a->contact_vz < 1e-6) {
                a->contact_vx = a->contact_vz = 0.0;
                a->route_bias_contact = 0;
            }
        }

        if (a->goal == AI_GOAL_GOTO) {
            double tx, tz, dx, dz;
            if (!a->path || a->path_n < 1) {
                a->goal = AI_GOAL_NONE;
                continue;
            }
            if (a->wp >= a->path_n)
                a->wp = a->path_n - 1;
            tx = (double)a->path[a->wp * 3];
            tz = (double)a->path[a->wp * 3 + 2];
            dx = tx - a->x;
            dz = tz - a->z;
            /* D-A13: the raw authored wp stays the current target and
             * advances only when the planned leg reaches the native
             * <11 m goal region. The FINAL authored point is exempt —
             * it drives to the 5 m D-A2 arrival below. */
            if (a->wp + 1 < a->path_n &&
                dx * dx + dz * dz < AI_NAV_GOAL_D2) {
                a->wp++;
                tx = (double)a->path[a->wp * 3];
                tz = (double)a->path[a->wp * 3 + 2];
            }
            /* D-A13: plan this authored leg on the 10 m grid. Budget-limited
             * searches yield a query-validated frontier; genuine no-route
             * failures leave any prior plan untouched. With neither, the
             * agent stays stopped and retries — never a raw straight line. */
            /* D-A21: engage the direct authored-line drive only when
             * ALL of it holds — the path was TELEPORT-set (an authored
             * stunt/staging release), the leg's straight line contains
             * a cliff-step piece (so the planner can never route the
             * line itself), and the planner has made NO progress toward
             * the waypoint for AI_STUNT_STALL ticks (so no detour is
             * being found either — P03's escort legs cross cliff pieces
             * that have perfectly good planner detours and must keep
             * them).  Sticky per leg; a wp change re-arms the planner. */
            if (a->path_stunt_ok) {
                if (a->leg_stunt_wp != a->wp) {
                    a->leg_stunt_wp = a->wp;
                    a->leg_stunt_on = 0;
                    a->leg_stall = 0;
                    a->leg_best_d2 = 1e30;
                }
                if (!a->leg_stunt_on) {
                    double d2 = dx * dx + dz * dz;
                    if (d2 < a->leg_best_d2 - 1.0) {
                        a->leg_best_d2 = d2;
                        a->leg_stall = 0;
                    } else if (++a->leg_stall >= AI_STUNT_STALL &&
                               leg_stunt(a, i, query, tx, tz)) {
                        a->leg_stunt_on = 1;
                    }
                }
            }
            /* D-A23: helicopters fly the authored polyline. The 10 m
             * ground A* is the reason P19's escape Huey wandered west
             * and froze 600 m off escape1. */
            int stunt = a->flyer || (a->path_stunt_ok && a->leg_stunt_on);
            if (!stunt && (a->plan_dirty || !plan_usable(a)))
                nav_plan(a, i, query, tx, tz);
            if (stunt) {
                double fdx = tx - a->x, fdz = tz - a->z;
                double fd = sqrt(fdx * fdx + fdz * fdz);
                if (fd > 1e-6) {
                    double desired = dir_yaw(fdx, fdz);
                    double v, step;
                    if (!a->heading_set) {
                        a->heading = desired;
                        a->heading_set = 1;
                    }
                    a->combat_steer_bias = combat_bias;
                    v = drive_actuator_tick(a, desired, 1, dt);
                    step = v * dt;
                    if (!a->physical_authority) {
                        if (step > fd)
                            step = fd;
                        a->x += -sin(a->heading) * step;
                        a->z +=  cos(a->heading) * step;
                    }
                }
            } else if (plan_usable(a)) {
                /* D-A15 bounded-steering actuator on the smoothed route:
                 * turn the persistent heading toward the planned point
                 * under the lateral-accel budget and advance along the
                 * heading at the corner-reduced speed. A reconstructed
                 * plan point is PROGRESS GEOMETRY, not a parking target:
                 * points sit on the 10 m planner grid while a bounded
                 * turn at cruise speed needs a radius of tens of meters,
                 * so a capture radius below the grid pitch strands the
                 * mover circling a point it can no longer reach (the P01
                 * wp4 tanker loop). Intermediate points therefore consume
                 * at planner scale — inside the confirmed <11 m goal
                 * region, or once a point that was previously ahead falls
                 * behind the heading (the progress it marked is already made,
                 * so the mover never commands a turnaround for a passed point).
                 * An active race's freshly replanned point that starts behind
                 * must first be turned toward. Other goal families retain the
                 * established D-A17 consumption. The authored FINAL point is
                 * exempt: it keeps
                 * the tight proximity/pass-through capture under the 5 m
                 * D-A2 arrival contract below, and an unreached final
                 * goal is never skipped. */
                double px = a->plan_x[a->plan_idx];
                double pz = a->plan_z[a->plan_idx];
                double d, desired, v, step, hx, hz, dot;
                int race_target, raw_goal, final_goal;
                dx = px - a->x;
                dz = pz - a->z;
                d = sqrt(dx * dx + dz * dz);
                desired = d > 1e-6 ? dir_yaw(dx, dz) : a->heading;
                if (!a->heading_set && d > 1e-6) {
                    a->heading = desired;   /* first real desired direction */
                    a->heading_set = 1;
                }
                a->combat_steer_bias = combat_bias;
                v = drive_actuator_tick(a, desired, 1, dt);
                step = v * dt;
                hx = -sin(a->heading);
                hz = cos(a->heading);
                dot = dx * hx + dz * hz;
                race_target = a->rival >= 0;
                if (race_target && dot >= 0.0)
                    a->plan_point_ahead = 1;
                /* The plan's last point is the D-A2 parking target only
                 * on the final authored leg when the plan reached the raw
                 * authored point itself — a staged AI_NAV_STAGE point on
                 * a long final leg stays progress geometry. Both sides
                 * derive from the same authored float, so the equality
                 * test is exact. */
                raw_goal = a->plan_idx == a->plan_n - 1 &&
                           px == tx && pz == tz;
                final_goal = raw_goal && a->wp + 1 >= a->path_n;
                if (final_goal) {
                    if ((!a->physical_authority && d <= step) ||
                        d <= AI_WAYPT_DIST) {
                        /* The authored final point snaps in the arrival
                         * contract below; advancing without an
                         * intermediate snap keeps rendered displacement
                         * aligned with the persistent heading. */
                        a->plan_idx++;
                        a->plan_point_ahead = 0;
                    } else if (d <= AI_PASS_DIST && dot < 0.0 &&
                               (!race_target || a->plan_point_ahead)) {
                        a->plan_idx++;
                        a->plan_point_ahead = 0;
                    } else if (!a->physical_authority) {
                        a->x += hx * step;
                        a->z += hz * step;
                    }
                } else if (d * d < AI_NAV_GOAL_D2 ||
                           (dot < 0.0 &&
                            (!race_target ||
                             (!raw_goal && a->plan_point_ahead)))) {
                    /* Planner-grid points are progress geometry and may be
                     * discarded once passed. For an active race, a fresh
                     * target must first be observed ahead; otherwise P02's
                     * corrected heading consumes a behind-starting plan every
                     * tick without moving (H-UAT-068b, path 3 waypoint 8).
                     * Other goal families retain D-A17's established rule. */
                    a->plan_idx++;
                    a->plan_point_ahead = 0;
                    /* H-UAT-076b: reaching a road/course-owned point's
                     * confirmed <11 m progress region must not emit a
                     * deterministic zero-motion frame. Preserve the exact old
                     * steering update toward this point, then carry its normal
                     * bounded translation through the transition. Unrelated
                     * routes and passed-point recovery retain D-A17's old
                     * stop/reacquire tick; moving those changes overshoot
                     * recovery on long escort routes. */
                    if (!a->physical_authority &&
                        d * d < AI_NAV_GOAL_D2 &&
                        route_has_authored_race(a)) {
                        a->x += hx * step;
                        a->z += hz * step;
                    }
                } else if (!a->physical_authority) {
                    a->x += hx * step;
                    a->z += hz * step;
                }
            }
            /* arrival = within AI_ARRIVE_DIST of the FINAL waypoint, AND
             * only after the route there was actually traversed: the
             * authored target must have advanced to the final node
             * (every intermediate node reached within its advance
             * radius). The bare bubble was P02's phantom-lap bug: on a
             * looped race path the snap put the agent ON the final node,
             * so each post-arrival re-issue (D-A5 fresh lap) re-fired
             * arrival on the very next tick — laps 2-4 completed in 3
             * ticks after one physical lap. Progress-gating is also the
             * original's shape: its route tracker (FUN_00406510) reaches
             * the destination only by advancing through the route's
             * nodes. Single-node paths (wp 0 == path_n-1) keep the pure
             * bubble. */
            if (a->wp >= a->path_n - 1) {
                const float *L = &a->path[(a->path_n - 1) * 3];
                double lx = (double)L[0] - a->x;
                double lz = (double)L[2] - a->z;
                if (lx * lx + lz * lz <= AI_ARRIVE_DIST * AI_ARRIVE_DIST) {
                    /* Route-owned kinematics retains its historical final
                     * snap. A pinned physical synthetic host only latches the
                     * arrival and brakes from its integrated pose. */
                    if (!a->physical_authority) {
                        a->x = L[0];
                        a->z = L[2];
                    }
                    a->goal = AI_GOAL_NONE;
                    a->arrived = 1;
                    a->arrivals++;
                    a->drive_speed = 0.0;
                }
            }
        } else if (a->goal == AI_GOAL_FOLLOW) {
            double tp[3] = { 0.0, 0.0, 0.0 };
            double dx, dz, d;
            resolve(a->target, tp);
            dx = tp[0] - a->x;
            dz = tp[2] - a->z;
            d = sqrt(dx * dx + dz * dz);
            /* FACT predicate remains 10 m for zero-offset authored follow.
             * Combat seek (FUN_0040aa20) has no range hold: native throttle
             * keeps closing. The marked 90 m firing-pass standoff is gone. */
            double hold_dist = a->follow_hold_dist > AI_FOLLOW_DIST
                             ? a->follow_hold_dist : AI_FOLLOW_DIST;
            int retreat = !a->combat_seek &&
                          a->follow_hold_dist > AI_FOLLOW_DIST &&
                          d < hold_dist - 3.0;
            a->at_follow = !retreat && d <= AI_FOLLOW_DIST;
            double seek_yaw = d > 1e-6 ? dir_yaw(dx, dz) : a->heading;
            if (a->combat_seek && d > 1e-6) {
                int dest = a->combat_dest;
                int motion = dest;
                int throttle = 1;
                double desired = seek_yaw;
                AiAgent *tgt = agent_at(a->target);
                /* FUN_004098a0: dest 6/8/9 timeUp re-picks from pursuit-1
                 * when a82c expires. */
                if (a->dest_until > 0.0 && s_ai_clock >= a->dest_until &&
                    (dest == 6 || dest == 8 || dest == 9 ||
                     dest == 4 || dest == 5)) {
                    int nd = pick_timeup_dest(a, dest);
                    if (nd != dest)
                        apply_combat_dest(a, nd);
                    dest = a->combat_dest;
                    motion = dest;
                }
                /* Dest 6/8/9 transitions, first match wins (native order:
                 * 0ad20→20 before 0a480/0a3d0→17). Slot/jam/swerve
                 * dests 28/29/7 stay unconsumed. */
                if (dest == 6 || dest == 8 || dest == 9 ||
                    dest == 4 || dest == 5 || dest == 13 || dest == 14) {
                    if (dest_far_handoff(d, tp[1] - a->y, a->flyer,
                                         tgt != NULL))
                        motion = 20;
                    else if (a->avoid && query) {
                        double hx = -sin(a->heading);
                        double hz = cos(a->heading);
                        if (whisker_blocked(a, i, query, a->x, a->z,
                                            a->x + hx * AI_WHISKER_RANGE,
                                            a->z + hz * AI_WHISKER_RANGE))
                            motion = 17;
                    }
                }
                if (motion == 20) {
                    desired = seek_yaw;
                    throttle = 1;
                } else if (motion == 9) {
                    dest9_plan(a, dx, dz, d, &desired, &throttle);
                } else if (dest_clsn_mode(motion) >= 0) {
                    dest68_plan(a, motion, dx, dz, d,
                                tgt ? tgt->heading : 0.0, tgt != NULL,
                                &desired, &throttle);
                } else if (motion == 15) {
                    desired = ang_wrap(a->heading + 1.0);
                    throttle = 1;
                } else if (!combat_target_ahead(a, dx, dz)) {
                    /* dest 17 FUN_0040aa20: turn only outside octants 0/1/7 */
                    throttle = 0;
                }
                desired = whisker_desired(a, i, query, desired);
                if (!throttle) {
                    a->combat_steer_bias = combat_bias;
                    (void)drive_actuator_tick(a, desired, 0, dt);
                    a->f_plan_on = 0;
                    continue;
                }
                seek_yaw = desired;
            }
            if (retreat && d > 1e-6) {
                /* Maintain a real standoff band: an attacker acquired too
                 * close backs out rather than treating every smaller range as
                 * a valid hold point. Retreat uses the same bounded steering. */
                double desired = whisker_desired(a, i, query,
                                                 dir_yaw(-dx, -dz));
                a->combat_steer_bias = combat_bias;
                double step = drive_actuator_tick(a, desired, 1, dt) * dt;
                double nx = a->x - sin(a->heading) * step;
                double nz = a->z + cos(a->heading) * step;
                AiNavSample sample = {0};
                if (a->physical_authority) {
                    /* car.c owns the integrated move/static response; the
                     * whisker-selected command turns or brakes around it. */
                    a->f_stall = 0;
                } else if (!query ||
                           query(i, a->x, a->z, nx, nz, &sample) != 0 ||
                           !sample.blocked) {
                    a->x = nx;
                    a->z = nz;
                    a->f_stall = 0;
                } else if (a->f_stall < AI_STUNT_STALL) {
                    a->f_stall++;
                }
                a->f_plan_on = 0;
                continue;
            }
            /* D-A22 assist arbitration, one bounded decision per
             * AI_STUNT_STALL interval (2 s): a DIRECT chase that has made
             * no progress for a whole interval AND whose straight line to
             * the target is refused engages the D-A13 planner; an
             * ASSISTED chase re-tests the line each interval and a clear
             * line resumes direct pursuit.  A stalled chase with a CLEAR
             * line (e.g. blocked by a live vehicle the dynamic contact
             * phase will resolve) just restarts its interval — behavior
             * is unchanged from the pre-D-A22 mover. */
            if (a->at_follow && !a->combat_seek) {
                if (a->follow_hold_dist > AI_FOLLOW_DIST && d > 1e-6) {
                    /* A retreat reaches range facing away; turn back in place
                     * before the fixed-forward fire-cone gate can accept it. */
                    a->combat_steer_bias = combat_bias;
                    (void)drive_actuator_tick(a, dir_yaw(dx, dz), 0, dt);
                    combat_bias = 0.0;
                } else if (combat_bias != 0.0) {
                    a->combat_steer_bias = combat_bias;
                    (void)drive_actuator_tick(a, a->heading, 0, dt);
                    combat_bias = 0.0;
                }
                a->f_plan_on = 0;
                a->f_stall = 0;
                a->f_best_d2 = d * d;
            } else if (!a->f_plan_on) {
                double d2 = d * d;
                if (d2 < a->f_best_d2 - 1.0) {  /* D-A21 epsilon, m^2 */
                    a->f_best_d2 = d2;
                    a->f_stall = 0;
                } else if (++a->f_stall >= AI_STUNT_STALL) {
                    a->f_stall = 0;
                    if (follow_line_refused(a, i, query, tp[0], tp[2])) {
                        a->f_plan_on = 1;
                        a->plan_leg_wp = -1; /* never adopt a stale plan */
                        a->plan_dirty  = 1;
                    }
                }
            } else if (++a->f_stall >= AI_STUNT_STALL) {
                a->f_stall = 0;
                if (!follow_line_refused(a, i, query, tp[0], tp[2])) {
                    a->f_plan_on = 0;
                    a->f_best_d2 = d * d;   /* fresh direct interval */
                    a->plan_leg_wp = -1;    /* drop the assist plan */
                }
            }
            if (a->at_follow && !a->combat_seek) {
                /* within follow distance: hold (unchanged) */
            } else if (a->f_plan_on) {
                /* D-A22: route toward the target's position through the
                 * same bounded planner goto legs use.  Replan when the
                 * plan is consumed or invalidated (D-A17), or when the
                 * live target has left the confirmed <11 m goal region
                 * of the position the plan was searched toward.  A
                 * failed search retains a prior valid plan; with none
                 * the agent stays stopped this tick and the interval
                 * re-test above keeps the retry bounded. */
                double gdx = tp[0] - a->f_gx, gdz = tp[2] - a->f_gz;
                if (a->plan_dirty || !plan_usable(a) ||
                    gdx * gdx + gdz * gdz >= AI_NAV_GOAL_D2) {
                    if (nav_plan(a, i, query, tp[0], tp[2]) == 0) {
                        a->f_gx = tp[0];
                        a->f_gz = tp[2];
                    }
                }
                if (plan_usable(a)) {
                    /* D-A15 actuator on the planned route; every point is
                     * progress geometry (the live target is the goal, so
                     * there is no authored parking point): consume inside
                     * the <11 m planner goal region or once passed. */
                    double px = a->plan_x[a->plan_idx];
                    double pz = a->plan_z[a->plan_idx];
                    double ex = px - a->x, ez = pz - a->z;
                    double ed = sqrt(ex * ex + ez * ez);
                    double desired = whisker_desired(a, i, query,
                        ed > 1e-6 ? dir_yaw(ex, ez) : a->heading);
                    double v, step, hx, hz, dot;
                    if (!a->heading_set && ed > 1e-6) {
                        a->heading = desired;
                        a->heading_set = 1;
                    }
                    a->combat_steer_bias = combat_bias;
                    v = drive_actuator_tick(a, desired, 1, dt);
                    step = v * dt;
                    hx = -sin(a->heading);
                    hz = cos(a->heading);
                    dot = ex * hx + ez * hz;
                    if (ed * ed < AI_NAV_GOAL_D2 || dot < 0.0) {
                        a->plan_idx++;
                    } else if (!a->physical_authority) {
                        a->x += hx * step;
                        a->z += hz * step;
                    }
                }
            } else if (d > 1e-6) {
                /* chase the laterally-offset target point with the same
                 * D-A15 steering as goto/race */
                double ux = dx / d, uz = dz / d;
                double tx = tp[0] + (-uz) * a->xoff;
                double tz = tp[2] + ( ux) * a->xoff;
                double ex = tx - a->x, ez = tz - a->z;
                double ed = sqrt(ex * ex + ez * ez);
                if (ed > 1e-6) {
                    double desired = whisker_desired(a, i, query,
                        a->combat_seek ? seek_yaw : dir_yaw(ex, ez));
                    double v, step;
                    if (!a->heading_set) {
                        a->heading = desired; /* first desired direction */
                        a->heading_set = 1;
                    }
                    a->combat_steer_bias = combat_bias;
                    v = drive_actuator_tick(a, desired, 1, dt);
                    step = v * dt;
                    if (!a->physical_authority) {
                        if (step > ed)
                            step = ed;
                        a->x += -sin(a->heading) * step;
                        a->z +=  cos(a->heading) * step;
                    }
                }
            }
        }
    }

    /* Fixed-order actuator pass. Every brain decision is complete before any
     * context advances. Promoted contexts publish only after their own step;
     * diagnostic shadows still publish nothing. */
    for (int i = 0; i < AI_MAX_AGENTS; i++) {
        AiAgent *a = &s_agents[i];
        if (!a->used)
            continue;
        a->command_trace_hash = command_trace_hash(a->command_trace_hash,
                                                    &a->drive_command);
#ifdef I76_AI_PHYSICS_PROBE
        if (a->physical_authority && a->shadow) {
            CarLive owner = {0};
            car_context_get_live(a->shadow, &owner);
            /* Before the one legitimate context step/publication, the brain's
             * pose mirror must still equal the prior physical owner. This
             * catches any direct promoted-path X/Z write without confusing
             * the post-step physical movement with a kinematic writer. */
            if (a->x != owner.x || a->z != owner.z)
                a->probe_direct_xz_write = 1;
        }
#endif
        shadow_actuator_tick(a);
    }
#ifdef I76_AI_PHYSICS_PROBE
    for (int i = 0; i < AI_MAX_AGENTS; i++) {
        AiAgent *a = &s_agents[i];
        if (a->used && !a->physical_authority &&
            (a->x != probe_before_x[i] || a->z != probe_before_z[i]))
            a->probe_direct_xz_write = 1;
    }
#endif
}
