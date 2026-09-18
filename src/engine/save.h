#ifndef SAVE_H
#define SAVE_H

/*
 * save.h — M5 save/load: byte-deterministic snapshot of the live sim.
 *
 * OUR format (DECISION; research: reference/i76-everywhere
 * docs/SAVE-FORMAT-GAPS.md + i76-save-editor.py header). The original
 * save###.cmp is the BETWEEN-mission garage/metagame save (car name[20],
 * variant[20], id[16] "doarmel"; Mr. Damage registry @64; 14x30 equipped
 * names @1024; 8xu32 armor/chassis tenths @2044; 116-byte part records
 * with +30 type/+46 class/+59 def/+76 durability/+80 f32 weight/+96
 * condition/+100 location; repair order as a count-prefixed trailing
 * section) plus savegame.dir bookmarks (u32 count, then 60-byte entries:
 * u32 last-completed Trip scene @+0, char[32] display name @+4, char[16]
 * save basename @+36, and shell-state fields @+52/+56). The original has
 * NO in-mission sim-state save at all — saves happen at the shack — so
 * there is nothing to reuse for resume: this module snapshots the live
 * 20 Hz sim instead.
 *
 * Buffer-based by design (DECISION): no FILE*, no VFS — the caller owns
 * I/O so the web page can localStorage/IndexedDB the bytes later.
 *
 * Byte-deterministic (DECISION): explicit little-endian writes of every
 * field, no struct memcpy, no padding, no host-order dependence. Doubles
 * are written as their IEEE-754 binary64 bit pattern (wasm32 + all
 * native targets are binary64; compile-time asserted in save.c).
 *
 * Layout (v1, fixed SAVE_SERIALIZED_SIZE bytes):
 *   +0   char[4]  magic "I76S"
 *   +4   u32le    version (= SAVE_VERSION)
 *   +8   u32le    total size in bytes (= SAVE_SERIALIZED_SIZE)
 *   +12  u64le    mission runner tick (20 Hz counter)
 *   +20  u32le    mission state (MISSION_*: 0 running, 1 complete, 2 failed)
 *   +24  CarLive, field-by-field in declaration order (car.h):
 *        31 f64le (x,y,z,yaw,pitch,roll, vx,vy,vz, yaw/pitch/roll_rate,
 *        throttle_pos, steer, shift_hold, engine_rpm, steer_angle,
 *        a_long_prev, comp_prev[6], wheel_w[6], t) then
 *        11 i32le (gear, reverse, handbrake, ignition, grounded[6],
 *        grounded_count)
 *
 * Scope DECISIONS:
 *  - Mission runner state is tick + mission_state ONLY. FSM machine
 *    state (IP/SP/stack, shared image cells), timers and the radio queue
 *    are NOT saved/restored: mission.h/fsm.h expose no machine snapshot
 *    API. Full FSM restore is the M7 DECISION (mission.c mission_restore
 *    documents the same split).
 *  - Config identity (VCF/mission names) is NOT stored: the host reloads
 *    the same mission + car first, then applies the save. State only.
 *  - A save taken mid-air/mid-slide round-trips bit-exactly: every field
 *    the next car_step reads is in the CarLive vector.
 *
 * Pure C11, no platform deps, matches car.c/mission.c conventions.
 */

#include <stddef.h>
#include <stdint.h>

#define SAVE_VERSION 1u

/* Exact v1 size: 24-byte header + 31*8 + 11*4 = 316 bytes. */
#define SAVE_SERIALIZED_SIZE ((size_t)316)

/*
 * save_serialize(buf, cap)
 *   Snapshot the live sim (full CarLive via car_get_live + mission tick/
 *   state) into buf, exactly SAVE_SERIALIZED_SIZE bytes. Returns the
 *   byte count written, or 0 on error (NULL buf, cap too small, or no
 *   car loaded — a snapshot without a live car is meaningless).
 *   The mission tick/state are recorded even when no mission is loaded
 *   (mission_state() is then the zero-initialized MISSION_RUNNING).
 */
size_t save_serialize(void *buf, size_t cap);

/*
 * save_deserialize(buf, n)
 *   Validate and apply a snapshot: restores the full CarLive via
 *   car_set_live and, when a mission is loaded, the runner tick + state
 *   via mission_restore (FSM state untouched — M7 DECISION above).
 *   Requires a loaded car (same config chain as the snapshot's — the
 *   format stores no config identity). Returns 0 on success, -1 on any
 *   validation failure (NULL, short buffer, bad magic, unsupported
 *   version, size-field mismatch, out-of-range mission state); a failed
 *   call never mutates live state.
 */
int save_deserialize(const void *buf, size_t n);

#endif /* SAVE_H */
