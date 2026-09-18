// Automation policy only; never imported by the shipped page.
// Observe pose, return ordinary key requests. The caller owns dispatch.
export class P19Recovery {
  tick = -1;
  x = 0;
  z = 0;
  wanted = false;
  slowTicks = 0;
  attempts = 0;
  backouts = 0;
  lastRecover = null;
  phase = null;
  reverseUntil = -1;
  settleUntil = -1;

  reset(pose) {
    this.tick = pose.tick;
    this.x = pose.x;
    this.z = pose.z;
    this.wanted = false;
    this.slowTicks = 0;
  }

  next(pose, targetSpeed, advance = 1) {
    // X is a toggle. Send it only on phase entry, then wait for a fresh pose
    // to acknowledge it: CDP may return a sample preceding key consumption.
    if (this.phase === 'reverse') {
      if (!pose.reverse) return { kind: 'reverse-wait' };
      if (this.reverseUntil < 0) this.reverseUntil = pose.tick + 70;
      if (pose.tick < this.reverseUntil) return {
        kind: 'back-out',
        keys: { up: true, left: this.backouts % 2 === 0,
                right: this.backouts % 2 === 1 },
      };
      this.phase = 'forward';
      return { tap: 'x', kind: 'restore-forward' };
    }
    if (this.phase === 'forward') {
      if (pose.reverse) return { kind: 'forward-wait' };
      this.phase = null;
      this.reset(pose);
      // Allow the reverse velocity to brake through zero and the car to
      // turn forward. The one-second slow trigger must not rewind that turn.
      this.settleUntil = pose.tick + 120;
    }
    if (pose.tick < this.settleUntil) return null;
    if (this.tick < 0) this.reset(pose);
    this.wanted ||= targetSpeed > 2;
    this.slowTicks = targetSpeed > 2 && Math.abs(pose.speed) < 0.8
      ? this.slowTicks + advance : 0;
    let recover = this.slowTicks >= 20;
    if (pose.tick - this.tick >= 120) {
      recover ||= this.wanted && Math.hypot(pose.x - this.x, pose.z - this.z) < 4;
      this.tick = pose.tick;
      this.x = pose.x;
      this.z = pose.z;
      this.wanted = false;
    }
    if (!recover) return null;
    this.reset(pose);
    // A second stall at the same location means R's grounded history is
    // blocked too. Back out with ordinary controls, as the Wasm bench does;
    // do not change the game's checkpoint or obstacle collision contracts.
    if (this.lastRecover && Math.hypot(pose.x - this.lastRecover.x,
                                      pose.z - this.lastRecover.z) < 4) {
      this.backouts++;
      this.phase = 'reverse';
      this.reverseUntil = -1;
      return { tap: pose.reverse ? null : 'x', kind: 'start-back-out' };
    }
    this.attempts++;
    this.lastRecover = { x: pose.x, z: pose.z };
    return { tap: 'r', kind: 'recover' };
  }
}
