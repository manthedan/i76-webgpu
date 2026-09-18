// Component regression, NOT campaign evidence. Inject only the initial
// water-tower approach observed in the failed page run; subsequent motion
// uses the production key queue, physics and recovery seam. No FSM writes.
import assert from 'node:assert/strict';
import { readFileSync, readdirSync } from 'node:fs';
import { join } from 'node:path';
import I76Web from './dist/i76web.mjs';
import { P19Recovery } from './p19_recovery.mjs';

// Delayed CDP acknowledgement must not turn one X press into two toggles.
{
  const p = new P19Recovery();
  const pose = { x: 0, z: 0, speed: 0, reverse: 0, tick: 0 };
  for (let tick = 1; tick <= 20; tick++)
    assert.equal(p.next({ ...pose, tick }, 0), null, 'intentional hold stays still');
  let action;
  for (let tick = 21; tick <= 40; tick++) action = p.next({ ...pose, tick }, 9);
  assert.equal(action.tap, 'r');
  for (let tick = 41; tick <= 60; tick++) action = p.next({ ...pose, tick }, 9);
  assert.equal(action.tap, 'x');
  for (let tick = 61; tick <= 64; tick++)
    assert.equal(p.next({ ...pose, tick }, 9).tap, undefined);
  action = p.next({ ...pose, reverse: 1, tick: 65 }, 9);
  assert.equal(action.keys.up, true);
  assert.equal(action.keys.fire, undefined, 'back-out cannot fire');
  assert.equal(p.next({ ...pose, reverse: 1, tick: 135 }, 9).tap, 'x');
  assert.equal(p.next({ ...pose, reverse: 1, tick: 136 }, 9).tap, undefined);
  for (let tick = 137; tick <= 256; tick++)
    assert.equal(p.next({ ...pose, tick }, 9), null, 'give the forward turn time');
  assert.equal(p.attempts, 1);
  assert.equal(p.backouts, 1);
}

const app = process.env.NITRO_APP;
if (!app?.endsWith('/')) throw new Error('Set NITRO_APP explicitly, with trailing /');
const M = await I76Web();
M.FS.mkdirTree('/data/miss8');
for (const f of readdirSync(app))
  if (/\.(zfs|zix|fnt)$/i.test(f))
    M.FS.writeFile('/data/' + f, readFileSync(join(app, f)));
for (const f of ['P19.MSN', 'P19.TER'])
  M.FS.writeFile('/data/miss8/' + f, readFileSync(join(app, 'miss8', f)));
assert.ok(M._web_init() > 0);
assert.equal(M.ccall('web_drive_load', 'number', ['string', 'string'],
                    ['miss8/P19.MSN', 'vdstag3.vcf']), 0);
const read = name => JSON.parse(M.ccall(name, 'string', [], []));
for (let i = 0; i < 4000; i++) {
  M._web_drive_step();
  const lc = read('web_drive_lifecycle_state');
  if (!lc.cam && lc.tick > 40) break;
}
assert.equal(read('web_drive_lifecycle_state').cam, 0);
const keys = { up: 4, brake: 5, left: 6, right: 7, fire: 3 };
const held = {};
const apply = wanted => {
  for (const [key, code] of Object.entries(keys)) {
    const want = !!wanted[key];
    if (held[key] !== want) M._web_key_event(code, +want);
    held[key] = want;
  }
};
// Match the real page's retreat-to-speed-road steering, not a constant
// right turn (which can eventually circle around the tower on its own).
const normal = pose => {
  let diff = Math.atan2( -(3820 - pose.x), 49110 - pose.z) - pose.yaw;
  while (diff > Math.PI) diff -= 2 * Math.PI;
  while (diff < -Math.PI) diff += 2 * Math.PI;
  const targetSpeed = Math.abs(diff) > 1.25 ? 9 : 60;
  return { targetSpeed, keys: {
    up: pose.speed < targetSpeed - 1, brake: pose.speed > targetSpeed + 1,
    left: diff > 0.012, right: diff < -0.012,
  } };
};
M._web_drive_probe_place(4482.97, 49085.77, 1.804, 0.09);
// Seed grounded history at the observed blocked checkpoint. Without this,
// R can return to the mission spawn, which is a different bug pattern.
apply({});
for (let i = 0; i < 140; i++) M._web_drive_step();
const start = read('web_drive_pose');
let hit = null;
const pilot = new P19Recovery();
let firstRecover = null, afterRecover = null;
for (let i = 0; i < 1200; i++) {
  const pose = read('web_drive_pose');
  const contact = read('web_drive_collider_diag');
  if (contact.label === 'awatert1') hit ??= contact;
  const drive = normal(pose);
  const action = pilot.next(pose, drive.targetSpeed);
  apply(action ? action.keys ?? {} : drive.keys);
  if (action?.tap === 'r') {
    firstRecover ??= pose;
    assert.equal(M._web_drive_recover(), 0);
    afterRecover ??= read('web_drive_pose');
  } else if (action?.tap === 'x') {
    M._web_key_event(13, 1);
    M._web_key_event(13, 0);
  }
  M._web_drive_step();
}
apply({});
const end = read('web_drive_pose');
const distance = Math.hypot(end.x - start.x, end.z - start.z);
console.log('[recovery] ' + JSON.stringify({ start, hit, end, distance,
  recovers: pilot.attempts, backouts: pilot.backouts ?? 0 }));
assert.equal(hit?.label, 'awatert1', 'fixture actually contacts the reported tower');
assert.equal(hit.object, 25);
assert.ok(firstRecover && afterRecover, 'exercise the failed R checkpoint first');
assert.ok(Math.hypot(afterRecover.x - firstRecover.x,
                    afterRecover.z - firstRecover.z) < 4, 'R is not an escape');
assert.ok(pilot.backouts > 0, 'the shared pilot backs out instead of repeating R');
assert.ok(distance > 8, 'ordinary reverse clears the obstruction materially');
assert.ok(end.x < 4460, 'the pilot resumes its westbound route past the tower');
assert.equal(end.reverse, 0, 'forward gear restored after the bounded escape');
console.log('RESULT: PASS (P19 recovery component; injected approach, not a journey)');
