// Synthetic pilot decisions, not vehicle dynamics or browser acceptance.
import assert from 'node:assert/strict';
import { p19EngagementTarget, p19EngagementSpeed, p19SteeringBand,
         P19RoadRoute } from './p19_engagement.mjs';

const pose = Object.freeze({ x: 0, z: 0 });
const limo = Object.freeze({ ent: 6, x: 200, z: 0, enemy: 1, alive: 1, hidden: 0 });
const guard = Object.freeze({ ent: 5, x: 100, z: 0, enemy: 1, alive: 1, hidden: 0 });
const world = Object.freeze([limo, guard]);
assert.equal(p19EngagementTarget(world, pose, 5, limo, 500), guard,
  'steer toward the actual turret owner, not past a shooting guard');
assert.equal(p19EngagementTarget(world, pose, 6, limo, 500), limo,
  'acquisition remains game-owned: do not impose a private target lock');
for (const variant of [{ alive: 0 }, { hidden: 1 }, { enemy: 0 }, { x: 501 }]) {
  const unavailable = Object.freeze({ ...guard, ...variant });
  assert.equal(p19EngagementTarget([limo, unavailable], pose, 5, limo, 500), limo);
}
assert.equal(p19EngagementTarget(world, pose, -1, limo, 500), limo);
const boundary = Object.freeze({ ...guard, x: 500 });
assert.equal(p19EngagementTarget([limo, boundary], pose, 5, limo, 500), boundary);
assert.equal(p19EngagementSpeed(0), 32);
assert.equal(p19EngagementSpeed(0.4), 32);
assert.equal(p19EngagementSpeed(0.401), 18);
assert.equal(p19EngagementSpeed(-0.401), 18);
assert.equal(Math.min(9, p19EngagementSpeed(Math.PI)), 9,
  'the caller retains its tighter existing large-turn cap');
assert.equal(p19SteeringBand(1), 0.012);
assert.equal(p19SteeringBand(4), 0.048);
assert.equal(p19SteeringBand(8), 0.08);
assert.equal(p19SteeringBand(100), 0.08);

const opener = [[3510,46800],[3525,46900],[3578,47000],
                [3575,47075],[3537,47150],[3495,47200]];
const entities = (launches = 0, contacts = 0, miss = 0) => [
  { ent: 0, launches, contacts, worldMiss: miss },
  { ent: 2, alive: 0 }, { ent: 4, alive: 0 }, { ent: 5, alive: 0 },
  { ent: 6, x: 4622, z: 49234, alive: 1, hidden: 0 },
];
function ready() {
  const roads = new P19RoadRoute();
  const w = entities();
  assert.equal(roads.next({x:3330,z:46690,tick:0},w), null);
  let r = roads.next({x:3330,z:46701,tick:1},w);
  assert.deepEqual(r.target, {x:3510,z:46800});
  assert.equal(r.ceaseFire, false);
  for (const [i,[x,z]] of opener.entries()) {
    r = roads.next({x,z,tick:2+i},w);
    if (i < opener.length-1)
      assert.deepEqual(r.target, {x:opener[i+1][0],z:opener[i+1][1]});
  }
  assert.equal(r, null, 'the opener does not loop after its last waypoint');
  roads.next({x:4580,z:49175,tick:200},w);
  return roads;
}
const bank = {x:4580,z:49175,tick:240};
for (const change of ['contacts','tooFewMisses','tooFewLaunches','hidden',
                      'guardAlive','distant','outsideBank']) {
  const roads = ready(), w = entities(3,0,3), p = {...bank};
  if (change === 'contacts') w[0].contacts = 1;
  if (change === 'tooFewMisses') w[0].worldMiss = 2;
  if (change === 'tooFewLaunches') w[0].launches = 2;
  if (change === 'hidden') w[4].hidden = 1;
  if (change === 'guardAlive') w[2].alive = 1;
  if (change === 'distant') w[4].z = 49500;
  if (change === 'outsideBank') p.x = 4499;
  assert.equal(roads.next(p,w), null, `do not enter the flank on ${change}`);
}
const roads = ready(), blocked = entities(3,0,3);
let flank = roads.next(bank, blocked);
assert.equal(flank.phase, 'ridge-flank');
assert.equal(flank.ceaseFire, true, 'conserve the remaining rack while relocating');
for (const [i,[x,z]] of [[4620,49155],[4680,49137],[4730,49108]].entries()) {
  assert.deepEqual(flank.target, {x,z});
  flank = roads.next({x,z,tick:241+i},blocked);
}
assert.equal(flank, null);
assert.equal(roads.next({...bank,tick:300},entities(6,0,6)), null,
  'one measured flank cannot become a repeating destination loop');
console.log('[engagement] PASS synthetic target ownership, pacing, feedback bounds and road traversal');
