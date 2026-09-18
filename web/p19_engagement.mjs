// Automation policy only; not game targeting, vehicle tuning or native AI.
// The caller has already acquired the limo inside weapon range. Sensing the
// current turret owner never selects or changes a game target.
export function p19EngagementTarget(world, pose, owner, limo, range) {
  return world.find(e => e.ent === owner && e.enemy && e.alive && !e.hidden &&
    Math.hypot(e.x - pose.x, e.z - pose.z) <= range) ?? limo;
}

export function p19EngagementSpeed(diff) {
  return Math.abs(diff) > 0.4 ? 18 : 32;
}

// Do not immediately shrink the neutral cone after an occasional delayed
// decision: that restores opposite-turn chatter. Caller keeps the largest
// observed gap, and leaves the LOSS pilot's steering unchanged.
export function p19SteeringBand(maxDecisionSpan) {
  return Math.min(0.08, 0.012 * maxDecisionSpan);
}

// Measured RSEG ribbon centres, not a gameplay road/terrain modification.
// The old diagonal to the second van crosses a >40 m mesa; this route follows
// its eastern road. The later road climbs around the north bank that can hide
// an evading limo. See docs/evidence/missions/p19.md and the private map audit.
const OPENER = [
  { x: 3510, z: 46800 }, { x: 3525, z: 46900 },
  { x: 3578, z: 47000 }, { x: 3575, z: 47075 },
  { x: 3537, z: 47150 }, { x: 3495, z: 47200 },
];
const RIDGE = [
  { x: 4560, z: 49159 }, { x: 4620, z: 49155 },
  { x: 4680, z: 49137 }, { x: 4730, z: 49108 },
];
const distance = (a, b) => Math.hypot(a.x - b.x, a.z - b.z);

// One instance per WIN. Returns navigation intent; the caller still applies
// ordinary keys and the existing recovery policy. No engine mutator is used.
export class P19RoadRoute {
  constructor() {
    this.opener = -1;
    this.openerDone = false;
    this.flank = -1;
    this.flankDone = false;
    this.window = null;
  }

  next(pose, world) {
    let route = null;
    if (!this.openerDone && !world.find(e => e.ent === 2)?.alive &&
        (pose.z > 46700 || this.opener >= 0)) {
      if (this.opener < 0) this.opener = 0;
      if (distance(OPENER[this.opener], pose) < 15) this.opener++;
      if (this.opener === OPENER.length) this.openerDone = true;
      else route = { phase: 'opener-road', target: OPENER[this.opener],
                     speed: 22, ceaseFire: false };
    }
    const user = world.find(e => e.ent === 0);
    const limo = world.find(e => e.ent === 6);
    if (!limo?.alive || !user) return route;
    const snapshot = () => ({ tick: pose.tick, launches: user.launches,
                             contacts: user.contacts, miss: user.worldMiss });
    if (!this.window) this.window = snapshot();
    if (pose.tick - this.window.tick >= 40) {
      const blocked = user.launches - this.window.launches >= 3 &&
        user.contacts === this.window.contacts &&
        user.worldMiss - this.window.miss >= 3;
      const guardsDead = [4, 5].every(id => !world.find(e => e.ent === id)?.alive);
      if (blocked && guardsDead && !this.flankDone && this.flank < 0 &&
          !limo.hidden && distance(limo, pose) < 250 &&
          pose.x > 4500 && pose.x < 4740 && limo.z > 49140) {
        this.flank = RIDGE.findIndex(p => p.x >= pose.x + 20);
        if (this.flank < 0) this.flank = RIDGE.length - 1;
      }
      this.window = snapshot();
    }
    if (this.flank >= 0) {
      if (distance(RIDGE[this.flank], pose) < 12) this.flank++;
      if (this.flank === RIDGE.length) {
        this.flank = -1;
        this.flankDone = true;
      } else route = { phase: 'ridge-flank', target: RIDGE[this.flank],
                       speed: 22, ceaseFire: true };
    }
    return route;
  }
}
