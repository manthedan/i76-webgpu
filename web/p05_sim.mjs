// p05_sim.mjs — deterministic production-Wasm bench for P05 "Stop 'N' Go".
//
// This runs the shipped engine module and acts only through its PlatformKey
// input seam.  It is the CPU-speed tuning/proof layer for
// accept_dom_p05.mjs; only that browser script proves the shipped page.
//
// Authored contract (fsm_dump + tools/p05_full_probe.c): machine 2 advances
// the protected Toupee truck over path 16 `flee` and writes cell11=4 at its
// final node; machine 0 drains the outro radio and writes cell11=5; machine
// 13 then fires successAll.  The authored loss is machine 13's failAllObj(1/4)
// after Toupee is destroyed and the CB queue empties.  Enemy machines field
// from the truck's approach to path 4 `ambush`; target discovery below is
// bounded to the shipped 1 km radar and never reads hidden positions.
//
// Chassis selection is the same purchaser choice used by the page journey:
// Stag v3 (vdstag3), selected through the page's garage in accept_dom_p05.
// Its slot-2 30 mm turret has enough authored ammo/damage to defend the truck.
//
// Usage: node p05_sim.mjs [win|lose|both]
import { readFileSync, readdirSync } from 'node:fs';
import { resolve } from 'node:path';
import I76Web from './dist/i76web.mjs';

const APP = process.env.NITRO_APP;
if (!APP?.endsWith('/')) throw new Error('Set NITRO_APP explicitly, with trailing /');
const MODE = process.argv[2] || 'both';
if (!['win', 'lose', 'both'].includes(MODE)) {
  console.error(`usage: mode must be win, lose or both (got ${JSON.stringify(MODE)})`);
  process.exit(2);
}
const M = await I76Web();

M.FS.mkdir('/data');
for (const f of ['nitro.zfs', 'nitro.zix'])
  M.FS.writeFile('/data/' + f, readFileSync(APP + f));
for (const f of readdirSync(APP))
  if (/\.fnt$/i.test(f)) M.FS.writeFile('/data/' + f, readFileSync(APP + f));
M.FS.mkdirTree('/data/addon');
M.FS.writeFile('/data/addon/SCENARIO.DAT', readFileSync(APP + 'addon/SCENARIO.DAT'));
M.FS.mkdirTree('/data/miss8');
for (const f of readdirSync(APP + 'miss8'))
  M.FS.writeFile('/data/miss8/' + f, readFileSync(APP + 'miss8/' + f));
if (M._web_init() <= 0) throw new Error('web_init found no meshes');

const ENT_USER = 1, ENT_TOUPEE = 2;
const RADAR_RANGE = 1000;
const TURRET_RANGE = 340, MG_RANGE = 200;
const LOOKAHEAD = 45;
/* P05 path 16 `flee`, decoded by fsm_dump.  This is the protected truck's
 * briefed road/riverbed course, not a hidden-entity route.  Following its
 * geometry avoids cutting the physical player across the basin relief. */
const FLEE = [
  { x: 6540, z: 44990 }, { x: 6620, z: 45090 },
  { x: 6835, z: 45070 }, { x: 6995, z: 45125 },
  { x: 7070, z: 45200 }, { x: 7160, z: 45300 },
  { x: 7705, z: 45190 }, { x: 7705, z: 45570 },
  { x: 7485, z: 45700 }, { x: 7370, z: 45800 },
  { x: 7505, z: 46075 }, { x: 7165, z: 46360 },
  { x: 7160, z: 46540 }, { x: 7135, z: 46685 },
  { x: 6945, z: 46705 }, { x: 6735, z: 46680 },
  { x: 6595, z: 46820 }, { x: 6510, z: 46820 },
  { x: 6465, z: 46820 }, { x: 6225, z: 46745 },
  { x: 6095, z: 46705 }, { x: 6040, z: 46690 },
  { x: 6025, z: 46665 }, { x: 6040, z: 46560 },
];
const KEY = { fire: 3, up: 4, brake: 5, left: 6, right: 7, x: 13,
              w2: 15 };
const cstr = (n, t, a, v) => M.ccall(n, t, a, v);
const json = (n) => JSON.parse(cstr(n, 'string', [], []));
const wrapPi = (a) => {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
};

let held = {};
function setKey(key, want) {
  if (held[key] === want) return;
  held[key] = want;
  M._web_key_event(KEY[key], want ? 1 : 0);
}
function allUp() {
  for (const key of Object.keys(KEY)) setKey(key, false);
}
function tapCode(code) {
  M._web_key_event(code, 1);
  M._web_drive_step();
  M._web_key_event(code, 0);
  M._web_drive_step();
}

function pursuit(pose) {
  let best = Infinity, bestSeg = 0, bestT = 0;
  for (let i = 0; i < FLEE.length - 1; i++) {
    const a = FLEE[i], b = FLEE[i + 1];
    const dx = b.x - a.x, dz = b.z - a.z;
    const len2 = dx * dx + dz * dz;
    const t = Math.max(0, Math.min(1,
      ((pose.x - a.x) * dx + (pose.z - a.z) * dz) / len2));
    const d = Math.hypot(pose.x - (a.x + dx * t),
                         pose.z - (a.z + dz * t));
    if (d < best) { best = d; bestSeg = i; bestT = t; }
  }
  let need = LOOKAHEAD, seg = bestSeg, t0 = bestT;
  while (seg < FLEE.length - 1) {
    const a = FLEE[seg], b = FLEE[seg + 1];
    const len = Math.hypot(b.x - a.x, b.z - a.z);
    const available = (1 - t0) * len;
    if (need <= available) {
      const t = t0 + need / len;
      return { x: a.x + (b.x - a.x) * t,
               z: a.z + (b.z - a.z) * t,
               remaining: true };
    }
    need -= available;
    seg++;
    t0 = 0;
  }
  return { ...FLEE[FLEE.length - 1], remaining: false };
}

function boot() {
  held = {};
  const rc = M.ccall('web_drive_load', 'number', ['string', 'string'],
                     ['miss8/P05.MSN', 'vdstag3.vcf']);
  if (rc !== 0) throw new Error('drive_load failed');
  for (let i = 0; i < 4000; i++) {
    M._web_drive_step();
    const life = json('web_drive_lifecycle_state');
    if (life.tick >= 500 && life.cam) tapCode(KEY.fire);
    if (!life.cam && life.tick >= 500) {
      /* Stag v3 slot 2 is its authored 30 mm turret. */
      tapCode(KEY.w2);
      return life.tick;
    }
  }
  throw new Error('intro never released');
}

function journey(lose) {
  const started = Date.now();
  const released = boot();
  console.log(`[p05-sim] ${lose ? 'LOSS' : 'WIN'} release tick=${released}`);

  let lastTick = -1, terminalTick = -1, aimTicks = 0;
  let userAlive = true, userHp = -1, userMax = -1;
  let toupeeAlive = true, toupeeHp = -1;
  let toupeeMax = -1, toupeeMin = Infinity, maxTargetRange = 0;
  let kills = 0, lastKills = -1, maxTruckGap = 0, closestTruck = Infinity;
  let weaponMode = 'turret', lockedEnt = -1;
  let lockHp = Infinity, lockProgressTick = -1, lockEverProgress = false;
  const deferredUntil = new Map();
  let lastThreatBearing = 0, evadeUntil = -1;
  const stall = { tick: -1, x: 0, z: 0, wanted: false, reverseUntil: -1,
                  attempts: 0 };

  for (let loop = 0; loop < 30000; loop++) {
    const pose = json('web_drive_pose');
    const world = json('web_world_state');
    const life = json('web_drive_lifecycle_state');
    if ((life.state !== 0 || !life.poseValid) && life.tick > 100) {
      terminalTick = life.tick;
      break;
    }
    if (pose.tick === lastTick) continue;
    lastTick = pose.tick;

    const map = new Map(world.map((e) => [e.ent, e]));
    const combat = json('web_combat_state');
    const user = map.get(ENT_USER);
    const truck = map.get(ENT_TOUPEE);
    if (user) {
      userAlive = !!user.alive;
      userHp = user.hp;
      userMax = user.hpMax ?? user.hp;
    }
    if (truck) {
      toupeeAlive = !!truck.alive;
      toupeeHp = truck.hp;
      toupeeMax = Math.max(toupeeMax, truck.hpMax ?? truck.hp);
      toupeeMin = Math.min(toupeeMin, truck.hp);
      const gap = Math.hypot(truck.x - pose.x, truck.z - pose.z);
      maxTruckGap = Math.max(maxTruckGap, gap);
      closestTruck = Math.min(closestTruck, gap);
    }
    kills = world.filter((e) => e.ent >= 3 && e.ent <= 10 &&
                                   !e.alive && e.playerKill).length;
    if (!lose && weaponMode === 'turret' && combat.ammo <= 0) {
      allUp();
      tapCode(16); // Stag slot 3, forward 7.62 mm MG
      weaponMode = 'mg';
      console.log(`[p05-sim] turret empty; MG selected tick=${pose.tick}`);
      continue;
    }

    if (kills !== lastKills) {
      console.log(`[p05-sim] tick=${pose.tick} playerKills=${kills}/8 ` +
                  `truck=${toupeeHp}/${toupeeMax}`);
      lastKills = kills;
    }
    if (!lose && kills === 8 && evadeUntil < 0) {
      evadeUntil = pose.tick + 120;
      console.log(`[p05-sim] post-volley lateral break tick=${pose.tick}`);
    }

    let threat = null, finalVisible = false;
    if (truck) {
      const visible = world.filter((e) => e.ent >= 3 && e.ent <= 10 &&
        e.alive && !e.hidden && e.relation === -1 &&
        Math.hypot(e.x - pose.x, e.z - pose.z) <= RADAR_RANGE);
      finalVisible = visible.some((e) => e.ent === 10);
      visible.sort((a, b) => {
        const ap = a.ent === 10 ? 0 : a.engTarget === ENT_TOUPEE ? 1 : 2;
        const bp = b.ent === 10 ? 0 : b.engTarget === ENT_TOUPEE ? 1 : 2;
        if (ap !== bp) return ap - bp;
        return Math.hypot(a.x - pose.x, a.z - pose.z) -
               Math.hypot(b.x - pose.x, b.z - pose.z);
      });
      const fireRange = weaponMode === 'turret' ? TURRET_RANGE : MG_RANGE;
      const inRange = visible.filter((e) =>
        Math.hypot(e.x - pose.x, e.z - pose.z) <= fireRange);
      if (!lose) {
        const prior = visible.find((e) => e.ent === lockedEnt);
        if (prior) {
          if (prior.hp < lockHp) {
            lockHp = prior.hp;
            lockProgressTick = pose.tick;
            lockEverProgress = true;
          } else {
            const priorRange = Math.hypot(prior.x - pose.x, prior.z - pose.z);
            const progressRange = prior.ent === 10 || kills < 6
              ? TURRET_RANGE : 150;
            const noContactLimit = lockEverProgress ? 400 : 600;
            if (priorRange <= progressRange &&
                pose.tick - lockProgressTick >= noContactLimit) {
              deferredUntil.set(prior.ent, pose.tick + 400);
              console.log(`[p05-sim] defer no-contact ent=${prior.ent} tick=${pose.tick}`);
              lockedEnt = -1;
            }
          }
        }
        const eligible = visible.filter((e) => e.ent === 10 ||
          pose.tick >= (deferredUntil.get(e.ent) ?? -1));
        const finalFielder = eligible.find((e) => e.ent === 10);
        /* Enemy7 crosses enemy5's lingering deployed hazards on this authored
         * lane. Take that visible body first so the strict player-attribution
         * proof cannot be stolen by hostile friendly fire. */
        const hazardVictim = eligible.find((e) => e.ent === 9);
        /* Preserve the old 650/800 defensive threshold as a pool ratio now
         * that D-C28 doubles the selected player's live/max pools. Once the
         * late wave is active, a truck attacker outranks self-defense: leaving
         * it on the warehouse line lets lingering hostile deployables own the
         * kill before the old schedule's next lock. */
        const truckDefense = kills >= 5
          ? eligible.find((e) => e.engTarget === ENT_TOUPEE)
          : null;
        const selfDefense = userMax > 0 && userHp * 16 < userMax * 13
          ? eligible.find((e) => e.engTarget === ENT_USER)
          : null;
        const locked = eligible.find((e) => e.ent === lockedEnt);
        threat = finalFielder ?? hazardVictim ?? truckDefense ?? selfDefense ??
          locked ?? eligible[0] ?? null;
        if (threat && threat.ent !== lockedEnt) {
          lockedEnt = threat.ent;
          lockHp = threat.hp;
          lockProgressTick = pose.tick;
          lockEverProgress = false;
        }
      } else {
        threat = inRange.find((e) => e.engTarget === ENT_USER) ?? null;
      }
    }
    /* When Toupee turns west onto the warehouse approach, regroup instead of
     * chasing a remote straggler. Never suppress a visible actor currently
     * targeting Toupee; the corrected pool timing can leave one active here. */
    if (!lose && truck && truck.x < 7100 && truck.z > 46600 &&
        !finalVisible && kills < 8 &&
        (!threat || threat.engTarget !== ENT_TOUPEE)) {
      threat = null;
    }

    let target = { x: pose.x, z: pose.z };
    let targetSpeed = 0, wantFire = false;
    if (threat) {
      const range = Math.hypot(threat.x - pose.x, threat.z - pose.z);
      lastThreatBearing = Math.atan2(-(threat.x - pose.x),
                                     threat.z - pose.z);
      maxTargetRange = Math.max(maxTargetRange, range);
      target = threat;
      const fireRange = weaponMode === 'mg' ? MG_RANGE :
        (threat.ent === 10 || kills < 6 ? TURRET_RANGE : 150);
      wantFire = range <= fireRange;
      targetSpeed = 32;
    } else if (!lose && truck && truck.alive) {
      /* The flee polyline folds back near the warehouse. On that final leg,
       * escort Toupee's live body instead of resolving the player's nearest
       * (potentially earlier, parallel) route segment. */
      const warehouseLeg = truck.x < 7100 && truck.z > 46600;
      target = warehouseLeg ? { x: truck.x, z: truck.z, remaining: true }
                            : pursuit(pose);
      targetSpeed = target.remaining ? 42 : 0;
      const gap = Math.hypot(truck.x - pose.x, truck.z - pose.z);
      if (gap < 70) targetSpeed = 0;
      else if (gap < 120) targetSpeed = Math.min(targetSpeed, 15);
    }
    if (!lose && pose.tick < evadeUntil) {
      const bearing = lastThreatBearing + Math.PI / 2;
      target = { x: pose.x - Math.sin(bearing) * 300,
                 z: pose.z + Math.cos(bearing) * 300 };
      targetSpeed = 45;
      wantFire = false;
    }

    const dx = target.x - pose.x, dz = target.z - pose.z;
    const dist = Math.hypot(dx, dz);
    const diff = wrapPi(Math.atan2(-dx, dz) - pose.yaw);
    const aim = wantFire && Math.cos(diff) >=
      (weaponMode === 'turret' ? 0.25 : 0.99619);
    if (aim) aimTicks++;
    if (Math.abs(diff) > 1.25) targetSpeed = Math.min(targetSpeed, 9);

    if (stall.tick < 0) {
      stall.tick = pose.tick; stall.x = pose.x; stall.z = pose.z;
    }
    if (stall.reverseUntil >= 0) {
      const reversing = pose.tick < stall.reverseUntil;
      if (pose.reverse !== (reversing ? 1 : 0)) {
        allUp(); tapCode(KEY.x); continue;
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
    if (pose.tick - stall.tick >= 160) {
      const moved = Math.hypot(pose.x - stall.x, pose.z - stall.z);
      if (stall.wanted && moved < 4) {
        stall.attempts++;
        stall.reverseUntil = pose.tick + 70;
        console.log(`[p05-sim] reverse #${stall.attempts} tick=${pose.tick}`);
      }
      stall.tick = pose.tick; stall.x = pose.x; stall.z = pose.z;
      stall.wanted = false;
    }

    if (pose.tick % 500 === 0) {
      const visible = world.filter((e) => e.ent >= 3 && e.ent <= 10 &&
                                            e.alive && !e.hidden)
        .map((e) => `${e.ent}:${e.hp}@${Math.hypot(e.x - pose.x, e.z - pose.z).toFixed(0)}` +
                    `->${e.engTarget}`).join(' ');
      console.log(`[p05-sim] tick=${pose.tick} pos=${pose.x.toFixed(0)},${pose.z.toFixed(0)} ` +
                  `truck=${truck ? `${truck.hp}@${truck.x.toFixed(0)},${truck.z.toFixed(0)}` : '-'} ` +
                  `threat=${threat ? `${threat.ent}:${threat.hp}@${dist.toFixed(0)}` : '-'} ` +
                  `user=${userHp} kills=${kills} gun=${weaponMode} ` +
                  `ammo=${combat.ammo} visible=[${visible}]`);
    }

    setKey('up', pose.speed < targetSpeed - 1);
    setKey('brake', pose.speed > targetSpeed + 1 && !aim);
    setKey('left', !aim && diff > 0.012);
    setKey('right', !aim && diff < -0.012);
    setKey('fire', aim);
    M._web_drive_step();
  }
  allUp();

  const msg = cstr('web_mission_logic_message', 'string', [], []);
  const state = /COMPLETE/.test(msg) ? 'COMPLETE' : /FAILED/.test(msg) ? 'FAILED' : 'RUNNING';
  const pass = lose
    ? state === 'FAILED' && /failAllObj reason=1\/4/.test(msg) && !toupeeAlive && userAlive
    : state === 'COMPLETE' && /successAll/.test(msg) && toupeeAlive && userAlive && kills === 8;
  console.log(`[p05-sim] FINAL mode=${lose ? 'lose' : 'win'} result=${pass ? 'PASS' : 'FAIL'} ` +
              `state=${state} verb="${msg}" tick=${terminalTick} ` +
              `user=${userHp} alive=${userAlive} truck=${toupeeHp}/${toupeeMax} ` +
              `truckMin=${toupeeMin} truckAlive=${toupeeAlive} kills=${kills}/8 ` +
              `aimTicks=${aimTicks} maxRadarTarget=${maxTargetRange.toFixed(1)}m ` +
              `truckGap=${closestTruck.toFixed(1)}..${maxTruckGap.toFixed(1)}m ` +
              `wall=${((Date.now() - started) / 1000).toFixed(1)}s`);
  return pass;
}

let failed = false;
if (MODE === 'win' || MODE === 'both') {
  const pass = journey(false);
  failed = !pass || failed;
}
if (MODE === 'lose' || MODE === 'both') {
  const pass = journey(true);
  failed = !pass || failed;
}
console.log(failed ? 'RESULT: FAIL' : 'RESULT: PASS');
process.exit(failed ? 1 : 0);
