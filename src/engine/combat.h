#ifndef COMBAT_H
#define COMBAT_H

#include "engine/car.h"

/*
 * combat.h — M7 combat/damage model: per-entity HP, shot/ram/landing
 * damage events, death, and the FSM combat predicates (isDead isAttacked
 * isShot isRammed isGroovesFault hpLesser ammoLesser allEnemyDead
 * allBlgDead whoAttacked/whoShot/whoRammed nearestEnemy) plus the attack
 * engagement behind the `attack` verb and the hide/startCar visibility
 * store.
 *
 * This is what lets a scripted mission be WON and LOST: P01's win gate
 * (5 enemy deaths -> cell11, both tankers arrived -> cell12, intro done
 * -> cell10, then successAll) and its fail gates (user/tanker/gas/jade
 * isDead -> failAllObj) all read state owned here.
 *
 * Anchors (docs/specs/m4/ghidra-physics.md): live armor/chassis are int
 * arrays ent+0x148/+0x18c displayed x0.01 (Q18), the damage pipeline is
 * FUN_004a76f0 -> FUN_0044fd80 -> vehicle damage FUN_00462c00 (facet pick
 * by impact direction) -> facet apply FUN_004647f0; landing damage when
 * impact > 7.65 m/s -> FUN_00463370 (random per-part damage, rand()%6,
 * Q2); ram events feed the FSM isRammed via the merge/damage event queue
 * (Q8 pipeline). The collisionMultiplier consumer (Q8) and the ram-heals
 * underflow (Q12) are PARTIAL in the spec — we deliberately do NOT
 * reproduce the underflow bug (our armor math clamps at 0), and
 * collisionMultiplier is not applied (its consumer was never located).
 *
 * DECISIONS (spec gaps resolved here; each also flagged at the code):
 *  D-C1  Car pools come from each entity's purchaser VCF: the four armor
 *        and four chassis maxima are loaded by the side-effect-free
 *        car_combat_config chain. Native FUN_00417ac0 aggregates car health
 *        from the minimum armor/chassis/component live/max ratio, not a sum
 *        of sides. The scalar mission bridge keeps the weakest positive
 *        authored maximum as its unit scale, but derives LIVE HP from that
 *        minimum ratio after damage reaches the impacted VCF pools. A strong
 *        impacted facet therefore cannot be bypassed by an unrelated weak
 *        raw scalar. No hull constant is invented. Non-cars and unresolved
 *        VCFs retain the 100 fallback only without authored SDFC health.
 *        Structures use SDFC +40; zero stays inert (PORT DECISION pending
 *        the H-UAT-030 native durability demand), positive pools take the
 *        existing scalar packets. Scenery without FSM identity owns its
 *        pool in scene.c and takes the same projectile/stream world contacts.
 *  D-C2  The eight authored side facets are car-damage authority; one derived
 *        scalar remains the compatibility view for hpLesser/FSM/HUD callers.
 *        FACT phase-b-contact-damage.md §3.2 primary packets subtract the same
 *        damage from impacted armor and chassis; decoded flame/gas and Blox
 *        hit chassis only. Facet overflow spills only once into the three
 *        represented core systems. Exact native component-death flags remain
 *        a documented gap rather than a guessed second death model.
 *  D-C3  The host loads every parsed positive-damage top/side/turret/inside
 *        HLOC as a selectable direct-fire weapon. Utility droppers are not
 *        mislabeled as guns: deployed-object gameplay is not implemented.
 *        Name, damage, ammo, cadence and front/rear direction come from the
 *        purchased VCF/VDF/GDF chain. A host that does not supply a loadout
 *        retains the old 10-damage/infinite-ammo probe fallback.
 *  D-C4  Fire model: ordinary ORDF types use authored-speed projectiles,
 *        each decoded GDF family/tier's recovered range (150–4000 m; 150 m
 *        fallback), and hardpoint-facing direction. Native ORDF 9/10/11
 *        instead use the separate flamer stream manager. The port preserves
 *        that delivery split with a MARKED straight 1.5 m-radius swept stream
 *        per authored cadence tick, bounded by the published 30–40 m range;
 *        native curved-segment state/radius remain undecoded. The flame
 *        stream receives no target lead, angular dispersion, or homing. HP/FSM
 *        consequences occur only when its geometry contacts authored car
 *        bounds or a live non-car entity's active authored scene-part OBBs
 *        before terrain/anonymous static world (fire-delivery.md). Attributed
 *        structure parts are excluded from the world pass so one geometry
 *        cannot absorb and damage; hidden entities/inactive parts own neither
 *        path, while non-target structures still absorb.
 *  D-C5  attack(ent, target, unk) engagement: the attacker chases the
 *        target with ai_combat_chase at 20 m/s (P01's enemy chase speed,
 *        ai.h D-A1; D-A25 FUN_004152e0 dest 6/8/9, dest 17 octants as
 *        recovery/fallback, no 90 m hold). Every positive-damage direct-fire VCF mount keeps its
 *        authored GDF damage, ammo, cadence, FireAmount salvo, projectile
 *        speed, and recovered family/tier range. All ready in-range mounts
 *        are evaluated in stable WEPN order. Only an unresolved loadout
 *        keeps the old 1-hp/1.5-s/100-m fallback, as a parser fallback rather
 *        than shipped balance. Native manager branches use the decoded LOS
 *        and T_D trigger gates; accepted fire still must arrive
 *        geometrically. A script-issued goto WINS movement over the
 *        engagement follow (the script knows better; fire continues in
 *        range). attack with an invalid target — none (-1), unregistered,
 *        dead, hidden/withdrawn, or the attacker itself — clears the
 *        engagement and sits the attacker (ai_sit; jade's
 *        "attack(nearestEnemy=-1)" poll when every enemy is dead/hidden).
 *        Without the sit, an entity already chasing/following keeps its
 *        motion goal and drives on after being told to attack no one.
 *  D-C6  Rams: auto-detected between the player's physics-driven car
 *        and car-class entities within 2.5 m while closing faster than
 *        4 m/s (per-tick position deltas at the fixed step; a >15 m
 *        single-tick displacement is a teleport, not a speed); each
 *        party takes the D-C24 bounded speed packet, at most one ram per
 *        pair per 20 ticks. AI movers are kinematic and pass through each other
 *        (ai.c has no collision), so AI-AI contacts are NOT auto-rams —
 *        combat_ram() remains the explicit event for those.
 *        collisionMultiplier is NOT applied (Q8 PARTIAL). Damage clamps
 *        at 0 — no ram-heals underflow (Q12's bug is not a feature).
 *  D-C7  Landing damage (player only, Ghidra Q2): the host feeds
 *        car_landing_events() counters to combat_landing_events() each
 *        tick. >7.65 m/s vertical remains the FACT hard-landing/audio gate.
 *        MARKED H-UAT-070b convention pending the native conversion demand
 *        row: damage is floor((impact-7.65)/6), clamped 0..8. This makes
 *        ordinary road hops presentation-only while genuinely hard landings
 *        cost a bounded, speed-scaled amount. The same amount loads the
 *        suspension/four tire pools; exact native per-part count is PARTIAL.
 *        mission.c cannot link car.c (the native probe build lines list
 *        mission.c but not car.c), so the host wires the counters in.
 *  D-C8  Visibility: hidden entities (hide verb, or pre-startCar ambush
 *        waiters) cannot be shot or rammed — you cannot hit what is not
 *        fielded. Death hides the entity's scene object
 *        (scene_obj_set_hidden) and sits its AI mover (ai_sit) — dead
 *        cars stop and disappear (no wreck model yet; the render-side
 *        damage states are the M6+ lane). The death-presentation half is
 *        SUPERSEDED by D-C27 (10 s visible burning wreck); the gameplay
 *        latch (hidden entities leave every scan) stands.
 *  D-C9  Hostility: an entity is hostile to a reference when its team
 *        is nonzero and differs from the reference's team. Team 0 is
 *        neutral scenery (P01's gas station) — never an enemy.
 *        allEnemyDead counts car-class FSM entities hostile to the
 *        user; allBlgDead the non-car ones (vacuous set -> 1).
 *  D-C10 Determinism: landing/component spill, T_C acceptance, and T_D lead
 *        perturbation use separate local xorshift32 streams with fixed seeds. No libc rand
 *        or wall time enters simulation; NPC roster/cadence changes cannot
 *        perturb the physical-damage stream.
 *  D-C11 Event pulses (attacked/shot/rammed + the who* ids) are
 *        one-tick levels: set by damage events, cleared at the start of
 *        the next combat_tick (which runs at the end of mission_tick),
 *        so every polling machine observes an event exactly once — the
 *        strike-counter machines (P01 m20-22) count one strike per hit,
 *        not one per poll. whoAttacked/whoShot/whoRammed read -1 ("no
 *        one", mission.c D11) outside the pulse window.
 *  D-C13 Kill attribution: every death is credited to the entity that
 *        dealt the killing damage, and to nobody when that is -1 (a hard
 *        landing) or the victim itself. This is the melee scoreboard's
 *        only input — nitro.exe ships a "Player Name / Score / Kills /
 *        Deaths" table and "*** %1:s was killed by %2:s", so the original
 *        tracks the killer too. It is a monotone counter, not a pulse:
 *        the controller reads a total, and a one-tick pulse read at 20 Hz
 *        beside a fire path that can kill twice in one tick would drop
 *        kills.
 *  D-C12 Team comes from the mission bridge's own ODEF snapshot
 *        (mission.c's MissionObj.team — the same bytes scene.c parses;
 *        scene.h exposes no team accessor today, so adding one is
 *        avoided).
 *  D-C14 The pilot sidearm (.45 Handgun) is a separate pool, never a
 *        selectable hardpoint: no purchaser VCF mounts gh45.gdf (probed
 *        across every offered Nitro variant), so it is pilot-side
 *        equipment rather than a mounted weapon. Its runtime definition
 *        is the purchaser archive's gh45.gdf itself (GDFC display name
 *        "45 Caliber Handgun", h45ch.wav report); availability is gated
 *        on that parse succeeding — no definition, no sidearm, and no
 *        invented stats. It fires only as the glance context of the
 *        generic trigger, along the glanced side direction (the D-C4
 *        cone otherwise), drawing its own parsed ammo/cadence. Explicit
 *        hardpoint selection is never mutated by it. The cadence is its
 *        own governor: sidearm and selected hardpoint cool down
 *        independently — a hardpoint shot never delays the sidearm and
 *        vice versa — while switching hardpoints keeps sharing the one
 *        mounted governor (selection does not reset it). NPC engagement
 *        cadence (D-C5's per-entity eng_cool) is unaffected.
 *  D-C15 Autonomous acquisition: a FIELDED hostile (alive, visible,
 *        car-class, team-hostile to the user) with NO valid engagement
 *        and no explicit hold acquires its nearest enemy in combat_tick,
 *        under the same deterministic team rules as the nearestEnemy
 *        predicate. Explicit FSM commands always win: the fallback never
 *        fires while an attack target or an explicit "attack no one"
 *        hold stands, never arms friendlies (scripted convoy/escort
 *        targets are untouched), and a fallback engagement chases only
 *        when the entity has no explicit motion goal at all (an explicit
 *        engagement's chase still yields only to a live goto). A dead or
 *        hidden target dissolves the engagement and the next tick
 *        re-acquires deterministically rather than standing inert.
 *        SAT entities (ai.h D-A17 — parked by the sit verb) are never
 *        acquired, and a sat engagement neither chases nor fires: the
 *        original's sit behavior carries no target, so parked cars are
 *        weapons-safe there. An authored race (ai_race_active, D-A8)
 *        suspends an autonomous engagement's fire until another behavior
 *        owns the car; explicit attack verbs are unaffected. Together these
 *        keep P02's clown silent through the opening and four-lap race, then
 *        let its authored evade phase become the fight. The chase also never
 *        eats a pending isArrived pulse (ai_arrival_pending) — the pulse
 *        belongs to the script's next machine slice.
 *        Evidence: P01's machines DO issue attack (fsm dump, machines
 *        5/6/8/9), so this is a floor under scripted behaviour, not a
 *        replacement for it — tools/combat_probe.c's fallback scenario
 *        proves acquisition, chase, fire, explicit precedence, the goto
 *        yield and deterministic re-acquisition without assets.
 *  D-C20 (H-UAT-011a; D-C18/D-C19 are reserved by concurrent in-flight
 *        work) Weapon hits impart a physical shove: a shot that damages
 *        a LIVING non-player car-class entity adds decaying contact
 *        velocity (ai.h D-A11 — the same channel vehicle contact uses,
 *        with its drag, cap, and D-A17 replan-on-displacement semantics)
 *        along the horizontal attacker->target direction. Magnitude is
 *        COMBAT_SHOT_IMPULSE_PER_HP per damage point, capped at
 *        COMBAT_SHOT_IMPULSE_MAX per hit. Both constants are INVENTED
 *        (port approximation): the original's impulse consumer is the
 *        unresolved collisionMultiplier lane (Q8 PARTIAL) and no
 *        authored per-weapon impulse field is decoded. At authored scale,
 *        a 60-damage 25mm hit shoves 1.2 m/s and a 150-damage 30mm hit
 *        3.0 m/s; a 3200-damage Cherub hit reaches the existing 6.0 m/s
 *        per-hit cap instead of launching the car. D-A11 independently
 *        caps accumulated velocity at AI_SPEED_MAX and decays it with
 *        AI_CONTACT_DRAG. The player's
 *        own car is NOT shoved (its pose is owned by car.c physics, not
 *        the AI mover); that remains open work.
 *  D-C22 Physical player/AI contact publishes its pre-separation closing
 *        speed to combat before mission.c resolves the overlapping hulls.
 *        The older combat_tick proximity scan remains the render-free probe
 *        fallback, but cannot observe production OBB contacts after their
 *        centres have been separated beyond its 2.5 m radius. Both paths use
 *        the same combat_ram pulses and pair cooldown. D-C24 consumes measured
 *        relative speed directly for the RETURN packet; the diagnosed defect
 *        was applying the outgoing P17 percentage calibration symmetrically to
 *        the player, producing H-UAT-067f's ~100-point touch.
 *  D-C21 Presentation snapshots consume the same authored ORDF +4 flight
 *        speed as ordinary delivery, without the retired streak-lifetime
 *        floor. ORDF 9/10/11 presentation follows D-C4's current authority
 *        stream instead of pretending its authored 5 m/s state value is a
 *        point projectile. Only an unresolved source uses
 *        COMBAT_FX_SPEED_DEFAULT. NPC and player launches share the source.
 *  D-C23 Class-4 HLOCs release persistent deployed objects into a fixed
 *        128-slot pool at the tick's rear HLOC pose. GDF damage/ammo/cadence,
 *        sound and ORDF identity are authored; object radius, lifetime,
 *        arming, oil grip scale/duration, fire tick cadence, Caltrops scalar
 *        routing, Blox ram-damage fallback and Car-E-Racer's high-tier mine
 *        interpretation are MARKED PORT CONVENTIONS pending the H-UAT-066
 *        native demand row. H-UAT-067g bounds all overlapping Fire-Dropper
 *        records to one authored 15-point hit per target per marked 0.5 s
 *        region cadence. H-UAT-076c applies the same MARKED per-target
 *        10-tick window to mine/Car-E-Racer proximity-charge clusters: each
 *        contacted object still detonates and is consumed, but only the first
 *        packet inside the window damages. The native demand row owns both
 *        overlap rules. Droppers never call the D-C4 direct-fire gate. NPCs
 *        may release only on a real p_shot tick, so passive pursuit and
 *        gate-22 no-input parity
 *        cannot begin laying hazards.
 *  D-C25 Native turret convergence (aim-convergence.md): an effective
 *        class-3 mount predicts the unchanged acquisition owner's ENTITY
 *        ORIGIN with GDFC aim speed and a 0..5 s lead clamp, then advances
 *        persistent yaw plus child pitch by dt*(0.75+0.25*ammoFraction)*
 *        (1.7 ground, 6.8 helicopter). Pitch is admitted only inside strict
 *        90-degree parent-local yaw and is skipped for GDFC (3,3); the two
 *        independent on-target flags use the decoded 2-degree threshold but
 *        do not gate fire. Player and NPC delivery both launch from that live
 *        frame. Fixed mounts retain their authored HLOC/GPOF frame with no
 *        convergence. Radar acquisition/re-acquisition remains unchanged and
 *        explicitly unresolved; no COLP/roof offset or spread was invented.
 *  D-C26 maxAttackers (FUN_00407fd0) is enforced at chase start, not on
 *        fire. The target's a948 (ai.max_attackers) is the cap. Eligible
 *        others already holding an a998-analog slot on that target count;
 *        a new combat FOLLOW may start only while the count is below the
 *        cap. Per-tick fire still runs for any live eng_target — native
 *        "surplus keep moving" is motion, not a mute. Constructor default
 *        1000; setAgg resets to 1; a later setMaxAttackers wins. The slot
 *        is released on death, hide, hold, or target change. P01 machines
 *        16–19 write setMaxAttackers(user, 1), so only one car closes.
 *  D-C24 MARKED PORT CONVENTION (H-UAT-067f): native ram conversion remains
 *        undecoded. The rammed player's return packet is
 *        floor(closing_speed/2), clamped to [1,25] authored points—speed-
 *        scaled and survivable, never multiplied by player HP. An intentional
 *        player's outgoing packet retains P17's old 1.125 recipient-percentage
 *        calibration but is capped at 40% of target max, so one contact cannot
 *        kill; it is no longer reflected symmetrically. The fixed-speed native
 *        demand row owns replacement of all three marked values/curves.
 *  D-C17 isGroovesFault is the "Groove wrecked this car" flag: it is
 *        true only when the entity is DEAD and the killing blow came
 *        from the player (latched killed_by attribution). BINARY-
 *        VERIFIED: FUN_00417ed0 reads ai+0xa6e0, which FUN_00418950
 *        sets only when the incoming damage kills the victim
 *        (ent+0x458 bit 0x20 = dead, phase-b-contact-damage.md
 *        §+0x458) and the attacker is the user object. It is NOT a
 *        pulse-on-any-attack; the port's former reading fouled P02's
 *        legitimate post-race fight on the first hit.
 *  D-C28 Single-player player defense initialization is N-DECODED, not a
 *        tune. Mission load substitutes the shell-selected VCF before its
 *        armor/chassis live/max pools are written (FUN_004b8400 ->
 *        FUN_004b8660); the class-1 auxiliary initializer FUN_004622a0 then
 *        doubles the non-multiplayer player's pools. The port applies that
 *        rule once after mission_set_player_combat_config re-registers the
 *        selected VCF, to every represented authored armor/chassis live/max
 *        pair and the scalar compatibility view. NPCs and raw VCF bytes remain
 *        unchanged.
 *  D-C27 Vehicle-death presentation (vehicle-death-presentation.md,
 *        FUN_00465370/FUN_00464530) SUPERSEDES D-C8's instant hide for
 *        car-class non-player entities: the dead car stays VISIBLE as a
 *        burning wreck for 10.0 s (200 ticks), then its scene object
 *        hides. combat_die arms the wreck timer; combat_tick counts it
 *        down, rolls the native rand()&0xf secondary-event die each
 *        burning tick (50% nothing; mounted-part pop-off / v-chnk body
 *        chunk / undecoded helper / CHUNK1-CHUNK2 vertical debris /
 *        X1_CARS1 secondary explosion), and hides the scene object at
 *        expiry. Rolls consume a separate fixed xorshift32 stream
 *        (s_wreck_rng) so presentation cannot perturb the D-C10
 *        physical-damage or shot-acceptance streams. The kill FX record
 *        is the class-correct authored sequence: X1_CARX1 (24-frame
 *        XCX1_101.TMT flipbook, 2.0 s) at the entity position — D-C8's
 *        +1.0 "entity-centre approximation" and the invented 400-tick /
 *        20 s wreck-fire record are gone; record life is 200 ticks.
 *        The wreck deals NO damage (native has no wreck-damage call;
 *        H-UAT-077's 0-damage measurement is native-consistent).
 *        e->hidden keeps its D-C8 gameplay latch at death (dead cars
 *        leave every scan); only the scene-object hide is deferred.
 *        Presentation of the burn lives in scene.c's killed-record
 *        branch. Gaps flagged, not invented: damage-geometry variants
 *        (VGEO state 3) have no mesh-layer hook (scene.c reads state 0
 *        only), mounted-part/v-chnk flings have no part system, and the
 *        living-car continuous smoke emitter has no entity-anchored
 *        snapshot channel — the decoded bands are exposed through
 *        combat_side_damage_state/combat_smoke_level instead.
 *
 * Lifetime: combat_reset() clears all state; mission.c re-registers the
 * FSM entities on every load/attach (with their team/class/scene object)
 * and publishes the user entity and the position resolver.
 *
 * Build note: combat.c is compiled by textual inclusion from mission.c
 * (exactly like ai.c — the mission bridge is its only consumer this
 * milestone). web/build.sh and the native probe build lines list
 * mission.c only and stay valid unchanged. DECISION — do NOT add
 * combat.c to build lists while mission.c includes it (duplicate
 * symbols).
 */

/* Tunables (DECISION-marked above). */
#define COMBAT_MAX_ENTS        64     /* matches AI_MAX_AGENTS           */
#define COMBAT_DEFAULT_HP      100    /* unresolved/non-car fallback    */
#define COMBAT_FIRE_RANGE      150.0  /* m, D-C4                         */
#define COMBAT_FIRE_CONE_COS   0.996194698  /* cos(5 deg), D-C4          */
#define COMBAT_FIRE_COOLDOWN   10     /* ticks (0.5 s), D-C4             */
#define COMBAT_FIRE_DMG_DFLT   10     /* D-C3 fallback damage            */
#define COMBAT_ATTACK_RANGE_FALLBACK 100.0 /* unresolved-loadout fallback */
#define COMBAT_ATTACK_PERIOD_FALLBACK 30   /* old 1.5 s probe fallback   */
#define COMBAT_ATTACK_DMG_FALLBACK     1   /* old unresolved-loadout ping */
#define COMBAT_ATTACK_SPEED    20.0   /* m/s chase (ai.h D-A1), D-C5     */
#define COMBAT_RAM_DIST        2.5    /* m, D-C6                         */
#define COMBAT_RAM_CLOSING     4.0    /* m/s, D-C6                       */
#define COMBAT_RAM_COOLDOWN    20     /* ticks (1 s) per pair, D-C6      */
#define COMBAT_RAM_DAMAGE_MAX  25     /* player return, MARKED D-C24     */
#define COMBAT_LAND_HARD_SPEED 7.65   /* m/s FACT hard-landing threshold */
#define COMBAT_LAND_DAMAGE_STEP 6.0   /* m/s per point, MARKED H-UAT-070b */
#define COMBAT_LAND_DAMAGE_MAX 8      /* per event cap, MARKED H-UAT-070b */
#define COMBAT_OIL_GRIP_SCALE  0.05   /* MARKED H-UAT-075b; existing term */
#define COMBAT_OIL_TAIL_TICKS  40     /* 2 s refreshed tail, MARKED       */
#define COMBAT_RAM_OUTGOING_SCALE 1.125 /* P17 calibration, MARKED D-C24 */
#define COMBAT_RAM_OUTGOING_MAX_PCT 40 /* nonlethal cap, MARKED D-C24    */
#define COMBAT_TELEPORT_GUARD  15.0   /* m/tick — teleport, not speed    */
#define COMBAT_SHOT_IMPULSE_PER_HP 0.02 /* m/s per damage point (D-C20,  */
                                        /* INVENTED port approximation)  */
#define COMBAT_SHOT_IMPULSE_MAX    6.0  /* m/s cap per hit (D-C20,       */
                                        /* INVENTED port approximation)  */
#define COMBAT_FX_SPEED_DEFAULT  150.0  /* m/s when no authored speed    */
#define COMBAT_FLAME_STREAM_RADIUS 1.5 /* m, MARKED D-C4 convention      */
#define COMBAT_DEPLOY_MAX       128    /* bounded persistent world pool */
#define COMBAT_MINE_CLUSTER_TICKS 10   /* 0.5 s, MARKED H-UAT-076c       */
#define COMBAT_WRECK_BURN_TICKS 200    /* FACT 10.0 s wreck timer (D-C27) */
#define COMBAT_EXPLOSION_TICKS  40     /* FACT 2.0 s explsn lifetime      */
#define COMBAT_DEBRIS_TICKS     40     /* MARKED chunk-visual bound       */

/* Clear all combat state. Call on every mission load/attach/unload. */
void combat_reset(void);

/* Register FSM entity `ent` at load time. team/class_id come from the
 * ODEF record (-1/0 when unresolved — unresolved entities are inert:
 * never hostile, skipped by the fire/ram scans). scene_obj is the placed
 * scene object used for hide-on-death (-1 = none). label is the FSM
 * entity label (copied) for combat logs. config always supplies authored
 * VDF COLP hit geometry; class-1 entities additionally consume its VCFC
 * facets/loadout. Class-9 VCFC combat semantics remain a separate owner. */
void combat_register(int ent, int team, int class_id, int scene_obj,
                     const char *label, const CarCombatConfig *config);

/* The player entity index (drives hostility tests + landing damage). */
void combat_set_user(int ent);

/* D-C28: apply Nitro's decoded non-multiplayer player-only 2x defense rule
 * once to the currently registered player. The mission bridge calls this
 * only after replacing the placeholder registration with the shell-selected
 * VCF, so represented authored armor/chassis live and max pools begin at the
 * same doubled value. */
void combat_apply_singleplayer_player_defense(void);

/* The bridge's live position resolver (mission.c's ent_pos): fills
 * out[3] with the entity's current world position. Called at most once
 * per entity per combat_tick. */
void combat_set_resolver(void (*resolve)(int ent, double out[3]));

/* The player car's live pose (mission_set_car forwards it). Needed for
 * the fire cone (yaw) — the resolver only carries positions. */
void combat_set_user_pose(double x, double y, double z, double yaw);
/* Player direct-fire loadout (D-C3). `source` is the car weapon index used
 * only by presentation; ammo < 0 means infinite. Add preserves order and
 * returns the selectable slot. During load the last added non-dropper becomes
 * the initial single fire owner (H-UAT-079d observable PORT DECISION); explicit
 * selection does not reset the active cooldown.
 *
 * `link_class` follows the manual taxonomy (Slug Thrower, SPP, Flame,
 * Mortar, Dropper); `linkable` excludes decoded effective-class-3 traversing
 * mounts. A class-4
 * source may carry zero damage: the six CAR_DEPLOY_* families spawn their
 * persistent object here and never enter the direct-fire projectile gate.
 * Space fires the selected hardpoint until stock L/weapon_link toggles all same-class,
 * same-facing fixed mounts. GDFC +90 is retained only as an input to car.c's
 * explicit taxonomy map because its raw categories split some manual types. */
void combat_player_weapons_clear(void);
int  combat_player_weapon_add(const char *name, int damage, int ammo,
                              int cooldown_ticks, int source, int rear,
                              int link_class, int linkable);
int  combat_player_weapon_select(int slot);
/* Authored GDF bullet velocity (m/s) for hardpoint `slot`'s presentation
 * streak (D-C21). <= 0 keeps the default. Returns 0 on success. */
int  combat_player_weapon_set_speed(int slot, double mps);
/* Same for the pilot sidearm's own pool. */
void combat_player_sidearm_set_speed(double mps);
/* Cycle the highlighted hardpoint and make it the sole armed weapon
 * (stock weapon_cycle = Enter). delta is a signed step. */
int  combat_player_weapon_cycle(int delta);
/* Toggle the highlighted weapon between solo fire and all same-manual-class,
 * same-facing fixed mounts (stock weapon_link = L). Turrets/singletons are
 * unchanged. A mixed-rate linked set volley-syncs at its slowest cadence. */
int  combat_player_weapon_link(void);
/* 1 when hardpoint `slot` is armed for Space-fire (HUD "on" strip). */
int  combat_player_weapon_armed(int slot);
/* 1 when hardpoint `slot` fired on the latest Space trigger. Cleared for
 * every hardpoint at the start of each combat_player_fire call. */
int  combat_player_weapon_fired(int slot);

/*
 * Pilot sidearm (the .45 Handgun) — D-C14. No purchaser VCF mounts
 * gh45.gdf (verified across every offered Nitro variant): it is pilot-side
 * equipment, never part of the selectable hardpoint loadout, and number-key
 * selection never reaches it. It is the generic weapon_fire's glance
 * context: fired only while the pilot glances out a side window, along the
 * GLANCED direction rather than the selected weapon's hardpoint cone. Its
 * stats come from the purchaser archive's gh45.gdf at runtime (the host
 * calls _set only when that parse succeeds — no definition, no sidearm).
 * Explicit hardpoint selection/fire is untouched: the sidearm pool is
 * separate and selection state is never mutated by a contextual shot.
 */
void combat_player_sidearm_clear(void);
int  combat_player_sidearm_set(const char *name, int damage, int ammo,
                               int cooldown_ticks);
int  combat_player_sidearm_present(void);
const char *combat_sidearm_name(void);
int  combat_sidearm_ammo_left(void);     /* -1 infinite, 0 when absent */
int  combat_sidearm_cooldown(void);      /* ticks left on the sidearm's
 *                                            own governor (D-C14)       */
/* Glance-contextual trigger pull: D-C4 projectile along (dirx, dirz), the
 * sidearm's own ammo/cadence. Returns 1 when a shot went off. */
int  combat_player_sidearm_fire(int *hit_ent, double dirx, double dirz);

/* --- damage events ---------------------------------------------------- */

/* A projectile hit: `dmg` hp from attacker to target. Sets the target's
 * shot+attacked pulses and attacker ids. dmg <= 0 is a no-op (utility
 * droppers are not attacks). */
void combat_shot(int attacker, int target, int dmg);

/* A ram: `attacker` ran into `target` at `closing` m/s. Damage is
 * closing/2 hp (D-C6); sets the target's rammed+attacked pulses. */
void combat_ram(int attacker, int target, double closing);
/* Production player/AI OBB contact bridge (mission.c D-C22). */
void combat_player_vehicle_contact(int target, double closing);

/* Host-fed landing counters (D-C7): every increment of `hard_landings`
 * since the last call applies the bounded speed convention above. */
void combat_landing_events(int hard_landings, double last_impact);

/* --- verbs ------------------------------------------------------------ */

/* attack(ent, target, unk): chase-and-fire engagement (D-C5). */
void combat_attack(int ent, int target);

/* hide(ent, hidden) / the startCar verb (unhide). Hidden entities are
 * skipped by the fire/ram scans and stop drawing (D-C8: latched until
 * the opposite verb — death hiding is terminal). */
void combat_hide(int ent, int hidden);

/* Player trigger pull (D-C3/D-C4). Returns 1 when a projectile spawned.
 * Delivery is asynchronous, so hit_ent is set to -1 when supplied. Returns
 * 0 when the trigger did nothing (cooldown, empty, no user, user dead). */
int combat_player_fire(int *hit_ent);

/* Native-style terrain/static-world line of sight shared by the FSM and
 * decoded AI manager branches. Returns 0 for invalid/unresolved entities. */
int combat_can_see(int attacker, int target);

/* --- predicates (the FSM bridge reads these) --------------------------- */

int combat_is_dead(int ent);
int combat_is_attacked(int ent);      /* pulse (D-C11) */
int combat_is_shot(int ent);          /* pulse */
int combat_is_rammed(int ent);        /* pulse */
int combat_is_grooves_fault(int ent); /* killed by the user (D-C17): the
                                         flag arms only when the damage
                                         kills the victim — binary-
                                         verified FUN_00417ed0/00418950 */
int combat_hp_lesser(int ent, int pct);        /* hp < pct% of max      */
int combat_ammo_lesser(int ent, int pct);      /* ammo < pct% of capacity;
                                                  infinite ammo -> 0    */
int combat_all_enemy_dead(void);      /* car-class hostiles (D-C9)       */
int combat_all_blg_dead(void);        /* non-car hostiles (vacuous -> 1) */
int combat_who_attacked(int ent);     /* attacker entity, -1 (D-C11/12)  */
int combat_who_shot(int ent);
int combat_who_rammed(int ent);
int combat_nearest_enemy(int ent);    /* nearest live visible hostile
                                         (car-class) entity, -1 none    */

/* Advance the combat model one tick: clear event pulses, snapshot
 * positions, run engagements (chase + fire) and the ram scan. Called by
 * mission_tick after the FSM machines and the AI mover have run. */
void combat_tick(void);

/* --- read-only state (probes + browser presentation) ------------------- */

int combat_hp(int ent);               /* -1 for unknown entities         */
int combat_hp_max(int ent);
int combat_scene_entity(int scene_obj); /* -1 for scenery without FSM owner */
int combat_alive(int ent);
int combat_is_hidden(int ent);
int combat_ammo_left(int ent);        /* selected weapon; -1 infinite    */
int combat_ammo_capacity(void);
int combat_player_weapon_count(void);
int combat_player_weapon_selected(void);
const char *combat_player_weapon_name(void);
int combat_player_weapon_get(int slot, const char **name,
                             int *ammo, int *ammo_max);
int combat_player_weapon_damage(void);
int combat_player_weapon_cooldown(void);
int combat_player_weapon_source(void); /* car weapon index, -1 fallback   */
/* car_weapon_get index for hardpoint `slot`, or -1. */
int combat_player_weapon_source_at(int slot);
int combat_player_weapon_rear(void);
int combat_player_muzzle_position(double out[3]);
/* Current selected weapon's exact normalized spawn frame. This is the same
 * composed fixed/live-turret direction consumed by combat_player_fire. */
int combat_player_launch_frame(double origin[3], double direction[3]);
/* Deployed-object audit/probe state. Counters are monotone per mission;
 * kind is CAR_DEPLOY_*. */
int combat_deployed_active(int kind);
/* Newest active object of `kind`; 0 and fills out, -1 when absent. */
int combat_deployed_position(int kind, double out[3]);
unsigned long combat_deploy_count(int kind);
unsigned long combat_deploy_trigger_count(int kind);
unsigned long combat_deploy_obstacle_count(void);
/* Live effective-class-3 joint state. Fixed/uninitialised mounts report zero.
 * The two flags are FUN_004af210's independent 2-degree thresholds; they do
 * not invent a fire-lock consumer (aim-convergence.md §2.2/§5). */
double combat_player_weapon_traverse_yaw(int source);
double combat_player_weapon_traverse_pitch(int source);
int combat_player_weapon_on_target(int source, int *yaw_ok, int *pitch_ok);
unsigned long combat_launch_count(int ent);
unsigned long combat_contact_count(int ent);
unsigned long combat_world_absorb_count(int ent);
/* Monotone presentation audit: target-impact events emitted this mission.
 * Unlike the transient draw records, long-cooldown weapons cannot age it out. */
unsigned long combat_fx_hit_count(int ent);
int combat_ent_position(int ent, double out[3]);
int combat_eng_target(int ent);       /* engagement target, -1 none      */
int combat_under_fire_ticks(int ent); /* MARKED protected-weave latch    */
int combat_team(int ent);
/* FSM entity label ("enemy2", "tanker1"), "?" for unknown entities. The
 * pointer is module-owned and valid until the next combat_reset. */
const char *combat_ent_label(int ent);
int combat_is_enemy(int ent);         /* hostile car-class vs the user   */
int combat_user_ent(void);            /* player entity index, -1 unset   */
int combat_kills(int ent);            /* entities `ent` destroyed        */
int combat_killed_by(int ent);         /* latched killing entity, -1      */

/* --- death presentation + damage bands (D-C27) -------------------------- *
 * vehicle-death-presentation.md §1.5/§3: a dead car burns visibly for
 * COMBAT_WRECK_BURN_TICKS, and per-side armor/chassis fractions select the
 * native damage/smoke bands while alive. */
int combat_wreck_ticks(int ent);      /* burn ticks remaining, 0 = none  */
/* 1 while the dead entity's scene object is still drawn (the visible
 * burning wreck), 0 otherwise. */
int combat_wreck_scene_visible(int ent);
/* Native damage state for side 0..3 (native numbering: 0 front, 1 left,
 * 2 right, 3 back) from the side's min(armor,chassis) fraction:
 * f>0.75 -> 0, f>0.5 -> 1, f>0.25 -> 2, else 3; a dead entity reports the
 * forced death state 4 (variant 3 + charred texture state 3 on every side).
 * -1 for unknown entities or an unsupported side (the port tracks no top
 * pool). */
int combat_side_damage_state(int ent, int side);
/* Engine-smoke band from the aggregate condition (minimum live/max
 * component ratio, the FUN_00417ac0 analog): -1 = none (c >= 0.75),
 * 0 = xwp1 (0.6 < c < 0.75), 1 = xsg1 (0.4 < c <= 0.6), 2 = xbp1
 * (c <= 0.4 — always the burning wreck's level). -2 for unknown entities. */
int combat_smoke_level(int ent);

/* Live player pose read-back (combat_set_user_pose). Returns 0 and fills
 * the outs (any may be NULL) when a pose was set this session, else -1. */
int combat_user_pose(double *x, double *z, double *yaw);

/* --- per-component condition (H-UAT-003) -------------------------------- *
 * The original tracks independent condition pools per component/facet
 * (nitro.exe live armor/chassis int arrays ent+0x148/+0x18c, facet pick by
 * impact direction — header anchors above; Open76 SystemsPanel.cs: engine,
 * brakes, suspension, 4 armor facets, 4 chassis facets, 4 tires, matching
 * the authored zsy_.map/zsye.map panel regions one-to-one). FSM predicates
 * still consume one scalar compatibility view, but authored cars derive that
 * view from the minimum live/max VCF facet ratio after direction/type routing.
 * Core systems remain parallel panel state until native component-death flags
 * are decoded; non-cars/unresolved VCFs retain scalar fallback.
 *
 * Component indices (order matches the authored panel's dst anchors): */
enum {
    COMBAT_COMP_ENGINE = 0,
    COMBAT_COMP_SUSPENSION,
    COMBAT_COMP_BRAKES,
    COMBAT_COMP_TIRE_RR,
    COMBAT_COMP_TIRE_RL,
    COMBAT_COMP_TIRE_FR,
    COMBAT_COMP_TIRE_FL,
    COMBAT_COMP_ARMOR_F,
    COMBAT_COMP_ARMOR_R,
    COMBAT_COMP_ARMOR_L,
    COMBAT_COMP_ARMOR_B,
    COMBAT_COMP_CHASSIS_F,
    COMBAT_COMP_CHASSIS_R,
    COMBAT_COMP_CHASSIS_L,
    COMBAT_COMP_CHASSIS_B,
    COMBAT_COMP_COUNT
};

/* Component condition pool for `ent`, or -1 for an unknown entity/bad
 * component. Pools initialise full at registration and decrement through
 * routed combat events; authored car facet ratios feed the scalar compatibility
 * HP view, while non-facet systems remain panel state. */
int combat_component_hp(int ent, int comp);
int combat_component_hp_max(int ent, int comp);

/* Harness-only state preparation: give an existing live target a large,
 * deterministic pool so a total-weapon probe can observe several damaging
 * shots without changing gameplay constants or respawning mission objects.
 * Not wired to any page control. Returns 0 on success, -1 for bad input. */
int combat_probe_set_hp(int ent, int hp);
/* Harness-only deployed-object preparation at an exact world point. This
 * enters the real persistent pool/deployed_tick path; it is not exported by
 * the consumer page. Returns 0 on success, -1 on bad input/pool saturation. */
int combat_probe_deploy_at(int owner, int kind, int damage,
                           double x, double z);
/* Harness-only authored-impact injection for renderer parity. It creates one
 * presentation event and changes no HP, FSM, projectile, or world state. */
int combat_probe_impact_at(const char *effect_name,
                           double x, double y, double z);

/* --- combat presentation events (H-UAT-007/H-UAT-056) ------------------ *
 * Presentation observes deterministic sim-owned sources: transient
 * muzzle/impact/death events, every live asynchronous projectile, and the
 * current ORDF 9/10/11 stream segment. Nothing here feeds hp, FSM predicates,
 * collision, or AI. Event ages and trails advance only on combat_tick; no wall
 * clock or render RNG participates.
 */
enum {
    COMBAT_FX_MUZZLE = 1,
    COMBAT_FX_IMPACT,
    COMBAT_FX_KILL,
    COMBAT_FX_PROJECTILE,
    COMBAT_FX_DEPLOYED,
    COMBAT_FX_SECONDARY,    /* D-C27: X1_CARS1 wreck secondary pop      */
    COMBAT_FX_DEBRIS,       /* D-C27: CHUNK1/CHUNK2 vertical debris     */
};

enum {
    COMBAT_FX_TRACER_LIGHT = 1, /* ORDF 1/18: MG/sidearm                */
    COMBAT_FX_TRACER_HEAVY,     /* ORDF 6: cannon/tank shell            */
    COMBAT_FX_EXPLOSIVE,        /* mortar/cluster/fireball ordnance     */
    COMBAT_FX_MISSILE,          /* ORDF 2/3/8/0x14 rocket/missile       */
    COMBAT_FX_FLAME,            /* ORDF 9/11 flame                      */
    COMBAT_FX_GAS,              /* ORDF 10 gas                          */
};

#define COMBAT_FX_EVENT_MAX     32
#define COMBAT_PROJECTILE_MAX  128
#define COMBAT_FX_TRAIL_MAX      8
/* One snapshot may contain every event, projectile and deployed object. */
#define COMBAT_FX_MAX (COMBAT_FX_EVENT_MAX + COMBAT_PROJECTILE_MAX + \
                       COMBAT_DEPLOY_MAX)
#define COMBAT_FX_HIT_NONE      -1
#define COMBAT_FX_HIT_WORLD     -2

typedef struct {
    int      active;
    int      type;          /* COMBAT_FX_* event/projectile kind         */
    int      weapon_class;  /* parsed ORDF presentation family above     */
    int      damage;        /* authored damage; impact scale input only  */
    int      age;           /* ticks since event/projectile spawn         */
    int      travel_ticks;  /* retained direct-event travel schedule      */
    int      life;          /* total ticks; live projectiles use -1       */
    int      hit;           /* entity, HIT_NONE, or HIT_WORLD             */
    int      killed;        /* this impact/death destroyed the entity     */
    int      deploy_kind;   /* CAR_DEPLOY_* for COMBAT_FX_DEPLOYED       */
    uint32_t seed;          /* deterministic per-event/projectile seed    */
    char     effect_name[14]; /* authored impact XDF; empty = no effect    */
    char     ordnance_model[9]; /* OGEO authored deployed/projectile mesh */
    double   start[3];      /* muzzle/impact or oldest live trail point   */
    double   end[3];        /* impact/death/current projectile position   */
    double   speed;         /* authored ORDF flight speed, m/s            */
    int      trail_count;   /* chronological live projectile points       */
    double   trail[COMBAT_FX_TRAIL_MAX][3];
} CombatFx;

/* Snapshot active events followed by live projectiles. Returns the count
 * copied (<= COMBAT_FX_MAX, <= cap). */
int combat_fx_snapshot(CombatFx *out, int cap);

/* The entity in the player's forward fire cone RIGHT NOW (a marker hint,
 * not a guaranteed projectile contact), minus cooldown/ammo gates,
 * or -1. Fills out[3] with its live position when out != NULL. This is
 * the target the original's target-attached marker rides. */
int combat_target_marker(double out[3]);

#endif /* COMBAT_H */
