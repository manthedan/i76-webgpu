// p19_sim.mjs — headless tuning bench for the P19 keyboard journey.
//
// Same production wasm, same PlatformKey seam, same sensing surfaces as
// accept_dom_p19.mjs, but driven by web_drive_step() at CPU speed. This rig
// tunes the strategy; the shipped-page contract is proven only by
// accept_dom_p19.mjs in a real browser.
//
// Ground truth wired into the strategy (all measured on this rig):
//  - guard1/guard2 field inside 350 m of their highway seats or on being
//    shot (fsm_dump machines 4/5); hidden entities have no collision box,
//    so nothing can be engaged before it fields.
//  - the TSS fixed 7.62 lands hits only when the hull closes on the van's
//    box column; a dead-straight meridian approach 8-11 m off the column
//    misses every round (muzzle parallax). The committed duel that works:
//    steer straight at the van, plant at 130 m, hold fire.
//  - the fixed rocket pod only connects point-blank (30 launches, zero
//    hits planted at 230-340 m).
//  - duelled vans scatter 240 dmg mines where they die — arc around kills.
//  - after the limo kill, the successful route turns away from the compound,
//    stages east of the Huey's final point, crosses the <=28 m rendezvous
//    westbound with fire released, and lays the Stag's stock mines while
//    escaping southwest through the radio sequence.
//
// Usage: node p19_sim.mjs [win|lose|both]
import { readFileSync, readdirSync } from 'node:fs';
import { resolve } from 'node:path';
import I76Web from './dist/i76web.mjs';
import { p19EngagementTarget, p19EngagementSpeed, p19SteeringBand,
         P19RoadRoute } from './p19_engagement.mjs';

const APP = process.env.NITRO_APP;
if (!APP?.endsWith('/')) throw new Error('Set NITRO_APP explicitly, with trailing /');
const MODE = process.argv[2] || 'both';

const M = await I76Web();
M.FS.mkdir('/data');
for (const f of ['nitro.zfs', 'nitro.zix'])
  M.FS.writeFile('/data/' + f, readFileSync(APP + f));
for (const f of readdirSync(APP))
  if (f.toLowerCase().endsWith('.fnt'))
    M.FS.writeFile('/data/' + f, readFileSync(APP + f));
M.FS.mkdirTree('/data/addon');
M.FS.writeFile('/data/addon/SCENARIO.DAT',
               readFileSync(APP + 'addon/SCENARIO.DAT'));
M.FS.mkdirTree('/data/miss8');
for (const f of readdirSync(APP + 'miss8'))
  M.FS.writeFile('/data/miss8/' + f, readFileSync(APP + 'miss8/' + f));
if (M._web_init() <= 0) throw new Error('web_init found no meshes');

/* ---- shared constants with accept_dom_p19.mjs ---- */
const ENT_USER = 0, ENT_TARGET = 6, ENT_ESCAPE = 10;
const RADAR_RANGE = 1000;
const TTURRET = { slot: 1, range: 500 }; // Stag-v3 30mm, dmg 150
const MG = { slot: 1, range: 500 };
const TRIGGER = { x: 3380, z: 47650 };
const RENDEZVOUS_M = 25;
const CRUISE = 45;

/* ---- wasm seams ---- */
const cstr = (n, t, a, v) => M.ccall(n, t, a, v);
const j = (n) => JSON.parse(cstr(n, 'string', [], []));
const KEY = { up: 4, brake: 5, left: 6, right: 7, fire: 3, x: 13,
              w2: 15, w3: 16 };
const wrapPi = (a) => {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
};

let held = {};
function setKey(k, want) {
  if (held[k] === want) return;
  held[k] = want;
  M._web_key_event(KEY[k], want ? 1 : 0);
}
function allUp() {
  for (const k of Object.keys(KEY)) setKey(k, false);
}
function tapKey(k) {
  M._web_key_event(k, 1);
  M._web_drive_step();
  M._web_key_event(k, 0);
  M._web_drive_step();
}

function bootMission() {
  held = {};
  const rc = M.ccall('web_drive_load', 'number', ['string', 'string'],
                     ['miss8/P19.MSN', 'vdstag3.vcf']);
  if (rc !== 0) throw new Error('drive_load failed');
  for (let t = 0; t < 4000; t++) {
    M._web_drive_step();
    const lc = j('web_drive_lifecycle_state');
    if (lc.tick >= 20 && lc.tick % 25 === 0) tapKey(KEY.space);
    if (!lc.cam && lc.tick > 40) return lc.tick;
  }
  throw new Error('intro never released');
}

function journey(lose) {
  const started = Date.now();
  const t0 = bootMission();
  console.log(`[sim] intro released at tick ${t0}`);
  let phase = 'road', aimTicks = 0, slotSwitchTick = -1, slotWant = -1;
  const vel = new Map();          // ent -> current pos + per-tick delta
  const deadSpots = [];           // where duelled vans died (mine belts)
  const prevAlive = {};
  const shown = {};
  let prevMe = -1, prevF2 = -1;
  let limoAlive = true, limoMinHp = Infinity, limoMaxHp = 0, limoKill = false;
  let sawLimoRadar = false, maxLimoRange = 0;
  let closestHeli = Infinity, heliAlive = true, userAlive = true;
  let crossed = false, orbitAng = 0, heliStillTicks = 0;
  let nearHuey = false, nearHueyTick = -1, eastStaged = false;
  let retreatTop = false, retreatSouth = false, retreatWest = false;
  const stall = { tick: -1, x: 0, z: 0, wanted: false, reverseUntil: -1,
                  attempts: 0 };
  const BUDGET_TICKS = lose ? 26000 : 32000;
  const roads = new P19RoadRoute();
  let previousDecision = -1, maxDecisionSpan = 1;

  function finish(r) {
    allUp();
    const postWorld = j('web_world_state');
    for (const e of postWorld) {
      if (e.ent === ENT_USER) userAlive = !!e.alive;
      if (e.ent === ENT_TARGET) {
        limoAlive = !!e.alive;
        limoKill ||= !e.alive && !!e.playerKill;
      }
      if (e.ent === ENT_ESCAPE) heliAlive = !!e.alive;
    }
    const msg = cstr('web_mission_logic_message', 'string', [], []);
    console.log(`[sim] mode=${lose ? 'lose' : 'win'} result=${r.state} ` +
                `msg="${msg}" limoAlive=${limoAlive} kill=${limoKill} ` +
                `limoHp=${limoMinHp}/${limoMaxHp} heliClosest=` +
                `${closestHeli === Infinity ? '-' : closestHeli.toFixed(1)} ` +
                `aimTicks=${aimTicks} userAlive=${userAlive} ` +
                `wallclock=${((Date.now() - started) / 1000).toFixed(1)}s`);
    return { msg, limoAlive, limoKill, limoMinHp, limoMaxHp, closestHeli,
             heliAlive, sawLimoRadar, maxLimoRange, aimTicks, userAlive };
  }

  for (let tick = 1; tick <= BUDGET_TICKS; tick++) {
    const pose = j('web_drive_pose');
    const world = j('web_world_state');
    const cbst = j('web_combat_state');
    const lc = j('web_drive_lifecycle_state');
    if (previousDecision >= 0)
      maxDecisionSpan = Math.max(maxDecisionSpan, pose.tick - previousDecision);
    previousDecision = pose.tick;
    if ((lc.state !== 0 || !lc.poseValid) && lc.tick > 100) {
      const deadMe = world.find((e) => e.ent === ENT_USER);
      console.log(`[sim] mission ended tick=${tick} state=${lc.state} ` +
                  `me=${deadMe ? deadMe.x.toFixed(0) + ',' + deadMe.z.toFixed(0) : '?'} ` +
                  `pose=${pose.x.toFixed(0)},${pose.z.toFixed(0)} hp=${cbst.hp}`);
      return finish({ state: lc.state });
    }

    const user = world.find((e) => e.ent === ENT_USER);
    const limo = world.find((e) => e.ent === ENT_TARGET);
    const heli = world.find((e) => e.ent === ENT_ESCAPE);
    if (user) userAlive = !!user.alive;
    if (limo) {
      limoAlive = !!limo.alive;
      if (limo.alive) {
        limoMinHp = Math.min(limoMinHp, limo.hp);
        limoMaxHp = Math.max(limoMaxHp, limo.hp);
      }
      limoKill ||= !limo.alive && !!limo.playerKill;
    }
    if (heli) heliAlive = !!heli.alive;
    const heliVisible = heli && heli.alive && !heli.hidden;
    if (heliVisible) {
      closestHeli = Math.min(closestHeli,
        Math.hypot(heli.x - pose.x, heli.z - pose.z));
      const hv = vel.get(ENT_ESCAPE);
      const heliSpeed = hv ? Math.hypot(hv.dx, hv.dz) / 0.05 : Infinity;
      heliStillTicks = heliSpeed < 0.5 ? heliStillTicks + 1 : 0;
      const heliDist = Math.hypot(heli.x - pose.x, heli.z - pose.z);
      if (!nearHuey && heliStillTicks >= 20 && heliDist <= 25) {
        nearHuey = true;
        nearHueyTick = pose.tick;
        console.log(`[sim] HUEY PROXIMITY tick=${pose.tick} hp=${cbst.hp}`);
      }
    } else {
      heliStillTicks = 0;
    }
    if (cbst.hp !== prevMe) {
      console.log(`[sim] MEHP t=${pose.tick} hp=${cbst.hp} ` +
                  `userWhoShot=${user ? user.whoShot : -1}`);
      prevMe = cbst.hp;
    }
    const f2 = world.find((e) => e.ent === 2);
    if (f2 && f2.hp !== prevF2) {
      console.log(`[sim] F2HP t=${pose.tick} hp=${f2.hp}`);
      prevF2 = f2.hp;
    }
    /* Remember where duelled vans died: they scatter 240 dmg mines. */
    for (const ent of [2, 3]) {
      const e = world.find((w) => w.ent === ent);
      if (!e) continue;
      if (prevAlive[ent] && !e.alive) deadSpots.push({ x: e.x, z: e.z });
      if (e.alive && !e.hidden && !shown[ent]) {
        shown[ent] = 1;
        console.log(`[sim] FIELD tick=${pose.tick} ent=${ent} ` +
                    `me=${pose.x.toFixed(0)},${pose.z.toFixed(0)} ` +
                    `hp=${cbst.hp} whoShot=${e.whoShot}`);
      }
      prevAlive[ent] = !!e.alive;
    }
    /* Normalize entity velocity by observed simulation ticks: weapon taps
     * step the bench more than once.  Huey arrival detection consumes it. */
    for (const e of world) {
      const v = vel.get(e.ent);
      const dt = v ? Math.max(1, pose.tick - v.tick) : 1;
      const sampleDx = v ? (e.x - v.x) / dt : 0;
      const sampleDz = v ? (e.z - v.z) / dt : 0;
      const dx = v ? v.dx * 0.75 + sampleDx * 0.25 : sampleDx;
      const dz = v ? v.dz * 0.75 + sampleDz * 0.25 : sampleDz;
      vel.set(e.ent, { x: e.x, z: e.z, tick: pose.tick, dx, dz });
    }
    const nearestFoe = (maxD) =>
      world.filter((e) => e.alive && !e.hidden && e.enemy &&
                           e.ent !== ENT_TARGET)
           .map((e) => ({ e,
             d: Math.hypot(e.x - pose.x, e.z - pose.z) }))
           .filter((r) => r.d <= maxD)
           .sort((a, b) => a.d - b.d)[0];

    let target = TRIGGER, targetSpeed = CRUISE, stopAt = 0, fireEnt = null;
    let fireSlot = MG.slot, deployMines = false, prearmMines = false;

    if (!userAlive) {
      console.log(`[sim] PLAYER DEAD tick=${pose.tick} ` +
                  `pos=${pose.x.toFixed(0)},${pose.z.toFixed(0)} ` +
                  `phase=${phase}`);
      return finish({ state: 'dead' });
    }

    const limoVisible = limo && limo.alive && !limo.hidden;
    const dLimo = limoVisible
      ? Math.hypot(limo.x - pose.x, limo.z - pose.z) : Infinity;

    if (!lose) {
      if (!limoAlive && pose.x > 2820 && pose.z < 48150)
        eastStaged = true;
      if (limoAlive) {
        const highwayAlive = [2, 3].some((ent) => {
          const e = world.find((w) => w.ent === ent);
          return e && e.alive;
        });
        if (limoVisible && !highwayAlive) {
          if (dLimo <= RADAR_RANGE) {
            sawLimoRadar = true;
            maxLimoRange = Math.max(maxLimoRange, dLimo);
          }
          if (dLimo <= TTURRET.range) {
            phase = 'hunt';
            fireEnt = p19EngagementTarget(world, pose,
              M._web_audit_turret_target(), limo, TTURRET.range);
            target = fireEnt;
            fireSlot = TTURRET.slot;
            targetSpeed = 60;
          } else {
            /* Follow the authored right-fork roads rather than cutting the
             * mesa while the limo is outside weapon range.  These are route
             * waypoints; off-radar entity coordinates are not consumed. */
            if (pose.z < 47990) {
              phase = 'right-fork-entry';
              target = { x: 3505, z: 48010 };
            } else if (pose.x < 3800) {
              phase = 'right-fork-east';
              target = { x: 3820, z: 48010 };
            } else if (pose.z < 49090) {
              phase = 'intercept-north';
              target = { x: 3820, z: 49110 };
            } else {
              phase = 'intercept-east-road';
              target = { x: 5200, z: 49100 };
            }
            targetSpeed = 60;
          }
        } else {
          phase = 'road';
          /* Pre-arm the rocket turret well before his fielding radius:
           * its convergence is automatic, so the trigger can be held from
           * the first visible tick. */
          if (pose.z > 46000 && pose.z < 47400)
            slotWant = TTURRET.slot;
          const foe = nearestFoe(RADAR_RANGE);
          if (foe) {
            const fd = Math.atan2(-(foe.e.x - pose.x), foe.e.z - pose.z);
            /* Committed duel: steer straight at the van, plant at 130 m.
             * This exact configuration provably lands MG hits. */
            fireEnt = foe.e;
            /* Nothing on these cars actually traverses (mesh_type 1, tier
             * < 100), so every round flies along the hull. Orbit him: the
             * circling hull sweeps the fixed ray across his box every
             * cycle, and neither of us settles into a stale geometry. */
            orbitAng += 0.05;
            target = { x: foe.e.x + Math.sin(orbitAng) * 90,
                       z: foe.e.z + Math.cos(orbitAng) * 90 };
            targetSpeed = 32;
            stopAt = 0;
            fireSlot = TTURRET.slot;
          }
          /* Preserve the measured duel line until both van kills are
           * confirmed; then cross convoytrig on the native-proven meridian. */
          if (highwayAlive) {
            if (pose.z > 46200 && pose.z < 46950)
              target = { x: 3330, z: pose.z + 120 };
            else if (pose.z >= 46950 && pose.z < 47650)
              target = { x: 3515, z: pose.z + 150 };
            else {
              phase = 'overwatch';
              target = { x: 3400, z: 47840 };
              targetSpeed = Math.hypot(3400 - pose.x, 47840 - pose.z) > 45 ? 30 : 0;
            }
          } else {
            phase = 'trigger';
            target = TRIGGER;
            targetSpeed = 45;
          }
        }
      } else if (!retreatTop) {
        phase = 'retreat-to-speed-road';
        target = { x: 3820, z: 49110 };
        const cornerDist = Math.hypot(target.x - pose.x, target.z - pose.z);
        targetSpeed = cornerDist < 80 ? 0 : 60;
        if (cornerDist < 60 && Math.abs(pose.speed) < 3) retreatTop = true;
      } else if (!retreatSouth) {
        phase = 'retreat-south-road';
        target = { x: 3820, z: 48010 };
        const cornerDist = Math.hypot(target.x - pose.x, target.z - pose.z);
        targetSpeed = cornerDist < 80 ? 0 : 60;
        if (cornerDist < 60 && Math.abs(pose.speed) < 3) retreatSouth = true;
      } else if (!retreatWest) {
        phase = 'retreat-west-road';
        target = { x: 2850, z: 48010 };
        const cornerDist = Math.hypot(target.x - pose.x, target.z - pose.z);
        targetSpeed = cornerDist < 80 ? 0 : 60;
        if (cornerDist < 60 && Math.abs(pose.speed) < 3) retreatWest = true;
      } else if (nearHuey && pose.tick - nearHueyTick < 40) {
        /* This read-only proximity sample is not boarding ownership: machine
         * 15's authored isWithin(user, escape, 30) is authoritative. Hold the
         * crossing briefly with fire released; aiming the fixed hull mount at
         * an airport backup here puts the allied Huey directly in its ray. */
        phase = 'rendezvous-hold';
        target = { x: heli.x + 17, z: heli.z - 17 };
        targetSpeed = 8;
      } else if (nearHuey) {
        /* Cross the rendezvous westbound at speed and keep that momentum
         * away from the newly fielded airport backups. */
        phase = 'escape-airfield';
        target = { x: 2200, z: 47400 };
        targetSpeed = 60;
        deployMines = true;
      } else if (!eastStaged && pose.z > 47900 && pose.x < 2700) {
        phase = 'stage-south';
        target = { x: 2480, z: 47850 };
        targetSpeed = 60;
      } else if (!eastStaged && pose.x < 2820) {
        phase = 'stage-east';
        target = { x: 2850, z: 47850 };
        targetSpeed = 60;
      } else if (heliVisible && heliStillTicks >= 20) {
        phase = 'board-westbound';
        target = { x: heli.x + 17, z: heli.z - 17 };
        targetSpeed = 60;
        stopAt = 2;
        const airportFoe = world
          .filter((e) => e.ent >= 7 && e.ent <= 9 && e.alive && !e.hidden)
          .map((e) => ({ e,
            d: Math.hypot(e.x - pose.x, e.z - pose.z) }))
          .sort((a, b) => a.d - b.d)[0];
        if (airportFoe && airportFoe.d <= TTURRET.range)
          deployMines = true;
      } else {
        phase = heliVisible ? 'wait-heli-east' : 'airport';
        target = { x: 2850, z: 48095 };
        prearmMines = heliVisible;
        targetSpeed = Math.hypot(target.x - pose.x,
                                 target.z - pose.z) > 25 ? 30 : 0;
      }
    } else {
      /* Kill both vans with the committed duel (the 800-pool Stag
       * survives both opener missiles), cross convoytrig to field the
       * limo, then retreat south out of every engagement envelope and
       * wait for the authored 10/4 arrival. */
      if (pose.z > 47650 && !crossed) crossed = pose.tick;
      if (limo && !limo.hidden) {
        phase = crossed ? 'hold' : 'clip';
        target = crossed ? { x: 3420, z: 46980 } : TRIGGER;
        targetSpeed = crossed ? 42 : CRUISE;
      } else if (!crossed) {
        phase = 'road';
        target = TRIGGER;
        /* Post-duel corridor: west of both van columns until clear. */
        if (pose.z > 46200 && pose.z < 47570)
          target = { x: 3330, z: pose.z + 120 };
      } else {
        phase = 'clip';
        target = TRIGGER;
      }
      const foe = nearestFoe(RADAR_RANGE);
      const foeAhead = foe && Math.cos(wrapPi(
        Math.atan2(-(foe.e.x - pose.x), foe.e.z - pose.z) -
        pose.yaw)) > 0.0;
      if (foe && foeAhead && !(crossed && foe.d > 120)) {
        fireEnt = foe.e;
        target = foe.e;
        fireSlot = MG.slot;
        if (!crossed) { targetSpeed = 45; stopAt = 110; }
        else { targetSpeed = 42; stopAt = 0; }
      } else if (foe && foe.d <= MG.range) {
        const fd = Math.atan2(-(foe.e.x - pose.x), foe.e.z - pose.z);
        if (Math.cos(wrapPi(fd - pose.yaw)) > 0.2) {
          fireEnt = foe.e;
          fireSlot = MG.slot;
        }
      }
    }

    const road = lose ? null : roads.next(pose, world);
    if (road) {
      phase = road.phase;
      target = road.target;
      targetSpeed = road.speed;
      if (road.ceaseFire) fireEnt = null;
    }

    /* Mount selection with dry-rack fallback. */
    const dry = cbst.weapon === fireSlot && cbst.ammo <= 0;
    slotWant = deployMines || prearmMines ? 0
             : fireEnt ? (!dry ? fireSlot : MG.slot) : MG.slot;
    if (cbst.weapon !== slotWant &&
        (slotSwitchTick < 0 || tick - slotSwitchTick >= 4)) {
      slotSwitchTick = tick;
      tapKey(14 + slotWant);   // PK_1..PK_8 select slots 0..7
    }
    const range = cbst.weapon === TTURRET.slot ? TTURRET.range : MG.range;

    if (fireEnt && fireEnt.ent === ENT_TARGET)
      target = limo;
    const dx = target.x - pose.x, dz = target.z - pose.z;
    const dist = Math.hypot(dx, dz);
    const diff = wrapPi(Math.atan2(-dx, dz) - pose.yaw);
    const aim = deployMines || (!!fireEnt && dist <= range &&
                !(fireEnt === limo && limo?.hidden));
    if (aim) aimTicks++;
    if (fireEnt && fireEnt.ent !== ENT_TARGET && tick % 20 === 10)
      console.log(`[sim] wpn t=${pose.tick} sel=${cbst.weapon} ` +
                  `ammo=${cbst.ammo} dmg=${cbst.damage} d=${dist.toFixed(0)} ` +
                  `aim=${aim}`);
    setKey('fire', aim);

    if (pose.tick % 1000 === 0) {
      const nf = nearestFoe(RADAR_RANGE);
      console.log(`[sim] t=${pose.tick} ${phase} ` +
                  `pos=${pose.x.toFixed(0)},${pose.z.toFixed(0)} ` +
                  `v=${pose.speed.toFixed(1)} hp=${cbst.hp} ` +
                  `limo=${limo ? limo.hp : '-'} ` +
                  `foe=${nf ? `${nf.e.ent}@${nf.d.toFixed(0)}` : '-'} ` +
                  `wpn=${cbst.weapon}/${cbst.ammo}`);
    }

    if (phase === 'hunt')
      targetSpeed = Math.min(targetSpeed, p19EngagementSpeed(diff));
    if (Math.abs(diff) > 1.25) targetSpeed = Math.min(targetSpeed, 9);
    if (stopAt > 0 && dist < stopAt) targetSpeed = 0;

    if (stall.tick < 0) {
      stall.tick = pose.tick; stall.x = pose.x; stall.z = pose.z;
    }
    if (stall.reverseUntil >= 0) {
      const reversing = pose.tick < stall.reverseUntil;
      if (pose.reverse !== (reversing ? 1 : 0)) {
        tapKey(KEY.x);
        continue;
      }
      if (reversing) {
        setKey('fire', false);
        setKey('left', stall.attempts % 2 === 0);
        setKey('right', stall.attempts % 2 === 1);
        setKey('brake', false);
        setKey('up', true);
        M._web_drive_step();
        continue;
      }
      stall.reverseUntil = -1;
      stall.tick = pose.tick; stall.x = pose.x; stall.z = pose.z;
      stall.wanted = false;
    }
    if (targetSpeed > 2) stall.wanted = true;
    if (pose.tick - stall.tick >= 120) {
      const moved = Math.hypot(pose.x - stall.x, pose.z - stall.z);
      if (stall.wanted && moved < 4) {
        stall.attempts++;
        stall.reverseUntil = pose.tick + 70;
        console.log(`[sim] recover #${stall.attempts} tick=${pose.tick} ` +
                    `pos=${pose.x.toFixed(0)},${pose.z.toFixed(0)}`);
      }
      stall.tick = pose.tick; stall.x = pose.x; stall.z = pose.z;
      stall.wanted = false;
    }

    setKey('up', pose.speed < targetSpeed - 1);
    setKey('brake', pose.speed > targetSpeed + 1);
    setKey('left', diff > (lose ? 0.012 : p19SteeringBand(maxDecisionSpan)));
    setKey('right', diff < -(lose ? 0.012 : p19SteeringBand(maxDecisionSpan)));
    M._web_drive_step();
  }
  console.log('[sim] budget exhausted');
  return finish({ state: 'timeout' });
}

let fails = 0;
const check = (name, pass, detail = '') => {
  console.log(`${pass ? 'ok  ' : 'FAIL'} ${name}${detail ? ' — ' + detail : ''}`);
  if (!pass) fails++;
};

if (MODE === 'win' || MODE === 'both') {
  const r = journey(false);
  check('WIN: COMPLETE + successAll',
        /COMPLETE/.test(r.msg) && /successAll/.test(r.msg), r.msg);
  check('WIN: limo killed by player on radar',
        r.sawLimoRadar && !r.limoAlive && r.limoKill);
  check('WIN: heli reached inside boarding distance',
        r.closestHeli !== Infinity && r.closestHeli <= RENDEZVOUS_M,
        `${r.closestHeli === Infinity ? '-' : r.closestHeli.toFixed(1)}m`);
  check('WIN: allied Huey survived', r.heliAlive);
  check('WIN: player survived', r.userAlive);
}
if (MODE === 'lose' || MODE === 'both') {
  const r = journey(true);
  check('LOSE: FAILED with reason 10/4',
        /FAILED/.test(r.msg) && /reason=10\/4/.test(r.msg), r.msg);
  check('LOSE: limo survived', r.limoAlive);
}
console.log(fails ? `RESULT: FAIL (${fails})` : 'RESULT: PASS');
process.exit(fails ? 1 : 0);
