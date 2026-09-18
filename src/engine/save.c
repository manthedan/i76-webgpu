/*
 * save.c — M5 save/load: byte-deterministic snapshot of the live sim.
 *
 * Implements the buffer format documented in save.h (layout table,
 * DECISIONS D-OUR-format / D-buffer-IO / D-explicit-LE live there).
 *
 * Determinism rules (the probe's B==C byte-identity proof leans on all
 * of them):
 *  - every field is written explicitly, little-endian, no padding;
 *  - doubles cross as their binary64 bit pattern (memcpy into u64, then
 *    8 LE bytes) — identical on wasm32, x86-64 and arm64;
 *  - deserialize validates EVERYTHING before mutating live state, so a
 *    rejected buffer can never half-restore the sim;
 *  - serialize is a pure function of live module state: same state in,
 *    same bytes out (the round-trip fuzz in tools/save_probe.c proves
 *    serialize->deserialize->serialize is byte-identical).
 */

#include <string.h>

#include "engine/save.h"
#include "engine/car.h"
#include "engine/mission.h"

/* binary64 assumption (save.h DECISION): wasm32 + all native targets. */
typedef char save_double_must_be_8_bytes[(sizeof(double) == 8) ? 1 : -1];

/* ----------------------------------------------------------------------- */
/* Little-endian write cursor (bounds-safe: cap was pre-checked by the     */
/* caller against SAVE_SERIALIZED_SIZE, and the field walk is fixed-size)   */
/* ----------------------------------------------------------------------- */

typedef struct {
    uint8_t *p;
    size_t   off;
} SaveW;

static void w_u32(SaveW *w, uint32_t v)
{
    w->p[w->off++] = (uint8_t)(v & 0xFFu);
    w->p[w->off++] = (uint8_t)((v >> 8) & 0xFFu);
    w->p[w->off++] = (uint8_t)((v >> 16) & 0xFFu);
    w->p[w->off++] = (uint8_t)((v >> 24) & 0xFFu);
}

static void w_i32(SaveW *w, int32_t v)
{
    w_u32(w, (uint32_t)v);
}

static void w_u64(SaveW *w, uint64_t v)
{
    for (int i = 0; i < 8; i++)
        w->p[w->off++] = (uint8_t)((v >> (8 * i)) & 0xFFu);
}

static void w_f64(SaveW *w, double d)
{
    uint64_t bits;
    memcpy(&bits, &d, sizeof bits);     /* binary64 bit pattern */
    w_u64(w, bits);
}

/* ----------------------------------------------------------------------- */
/* Little-endian read cursor (every read bounds-checked; r->ok latches)     */
/* ----------------------------------------------------------------------- */

typedef struct {
    const uint8_t *p;
    size_t         n;
    size_t         off;
    int            ok;
} SaveR;

static uint32_t r_u32(SaveR *r)
{
    if (!r->ok || r->off + 4 > r->n) { r->ok = 0; return 0; }
    uint32_t v = (uint32_t)r->p[r->off]
               | ((uint32_t)r->p[r->off + 1] << 8)
               | ((uint32_t)r->p[r->off + 2] << 16)
               | ((uint32_t)r->p[r->off + 3] << 24);
    r->off += 4;
    return v;
}

static int32_t r_i32(SaveR *r)
{
    return (int32_t)r_u32(r);
}

static uint64_t r_u64(SaveR *r)
{
    if (!r->ok || r->off + 8 > r->n) { r->ok = 0; return 0; }
    uint64_t v = 0;
    for (int i = 0; i < 8; i++)
        v |= (uint64_t)r->p[r->off + i] << (8 * i);
    r->off += 8;
    return v;
}

static double r_f64(SaveR *r)
{
    uint64_t bits = r_u64(r);
    double d = 0.0;
    memcpy(&d, &bits, sizeof d);
    return d;
}

/* ----------------------------------------------------------------------- */
/* CarLive <-> cursor (field order = car.h declaration order = save.h's    */
/* layout table; the probe's fuzz would catch any drift between the two)   */
/* ----------------------------------------------------------------------- */

static void w_car(SaveW *w, const CarLive *lv)
{
    w_f64(w, lv->x);     w_f64(w, lv->y);     w_f64(w, lv->z);
    w_f64(w, lv->yaw);   w_f64(w, lv->pitch); w_f64(w, lv->roll);
    w_f64(w, lv->vx);    w_f64(w, lv->vy);    w_f64(w, lv->vz);
    w_f64(w, lv->yaw_rate);
    w_f64(w, lv->pitch_rate);
    w_f64(w, lv->roll_rate);
    w_f64(w, lv->throttle_pos);
    w_f64(w, lv->steer);
    w_i32(w, lv->gear);
    w_i32(w, lv->reverse);
    w_i32(w, lv->handbrake);
    w_f64(w, lv->shift_hold);
    w_f64(w, lv->engine_rpm);
    w_i32(w, lv->ignition);
    w_f64(w, lv->steer_angle);
    w_f64(w, lv->a_long_prev);
    for (int i = 0; i < 6; i++)
        w_f64(w, lv->comp_prev[i]);
    for (int i = 0; i < 6; i++)
        w_i32(w, lv->grounded[i]);
    w_i32(w, lv->grounded_count);
    for (int i = 0; i < 6; i++)
        w_f64(w, lv->wheel_w[i]);
    w_f64(w, lv->t);
}

static void r_car(SaveR *r, CarLive *lv)
{
    lv->x = r_f64(r);     lv->y = r_f64(r);     lv->z = r_f64(r);
    lv->yaw = r_f64(r);   lv->pitch = r_f64(r); lv->roll = r_f64(r);
    lv->vx = r_f64(r);    lv->vy = r_f64(r);    lv->vz = r_f64(r);
    lv->yaw_rate   = r_f64(r);
    lv->pitch_rate = r_f64(r);
    lv->roll_rate  = r_f64(r);
    lv->throttle_pos = r_f64(r);
    lv->steer        = r_f64(r);
    lv->gear      = r_i32(r);
    lv->reverse   = r_i32(r);
    lv->handbrake = r_i32(r);
    lv->shift_hold = r_f64(r);
    lv->engine_rpm = r_f64(r);
    lv->ignition   = r_i32(r);
    lv->steer_angle = r_f64(r);
    lv->a_long_prev = r_f64(r);
    for (int i = 0; i < 6; i++)
        lv->comp_prev[i] = r_f64(r);
    for (int i = 0; i < 6; i++)
        lv->grounded[i] = r_i32(r);
    lv->grounded_count = r_i32(r);
    for (int i = 0; i < 6; i++)
        lv->wheel_w[i] = r_f64(r);
    lv->t = r_f64(r);
}

/* ----------------------------------------------------------------------- */
/* Public API                                                               */
/* ----------------------------------------------------------------------- */

size_t save_serialize(void *buf, size_t cap)
{
    if (!buf || cap < SAVE_SERIALIZED_SIZE)
        return 0;
    if (!car_is_loaded())
        return 0;

    CarLive lv;
    memset(&lv, 0, sizeof lv);   /* padding hygiene: see below */
    car_get_live(&lv);

    SaveW w = { (uint8_t *)buf, 0 };
    w.p[w.off++] = 'I';
    w.p[w.off++] = '7';
    w.p[w.off++] = '6';
    w.p[w.off++] = 'S';
    w_u32(&w, SAVE_VERSION);
    w_u32(&w, (uint32_t)SAVE_SERIALIZED_SIZE);
    w_u64(&w, mission_ticks());
    w_u32(&w, (uint32_t)mission_state());
    w_car(&w, &lv);

    return w.off;   /* == SAVE_SERIALIZED_SIZE (probe asserts) */
}

int save_deserialize(const void *buf, size_t n)
{
    if (!buf || n < SAVE_SERIALIZED_SIZE)
        return -1;
    if (!car_is_loaded())
        return -1;

    SaveR r = { (const uint8_t *)buf, n, 0, 1 };

    /* header: validate everything before touching live state */
    uint8_t magic[4];
    magic[0] = r.p[r.off++];
    magic[1] = r.p[r.off++];
    magic[2] = r.p[r.off++];
    magic[3] = r.p[r.off++];
    if (magic[0] != 'I' || magic[1] != '7' ||
        magic[2] != '6' || magic[3] != 'S')
        return -1;
    uint32_t version = r_u32(&r);
    uint32_t total   = r_u32(&r);
    uint64_t tick    = r_u64(&r);
    uint32_t state   = r_u32(&r);
    if (!r.ok)
        return -1;
    if (version != SAVE_VERSION)
        return -1;                      /* no forward-compat guesses */
    if (total != SAVE_SERIALIZED_SIZE)
        return -1;
    if (state > 2u)
        return -1;                      /* MISSION_RUNNING..FAILED */

    /* Padding hygiene (DECISION): CarLive has tail/interior alignment
     * padding (ints among doubles); the wire format never carries it,
     * but car_set_live memcpy's the whole struct into s_lv. Zeroing the
     * local first keeps s_lv's padding zero across restores, so a raw
     * struct memcmp of two live states (the probe's B==C check) is
     * deterministic, matching car_place's memset. */
    CarLive lv;
    memset(&lv, 0, sizeof lv);
    r_car(&r, &lv);
    if (!r.ok)
        return -1;

    /* all validated: apply */
    car_set_live(&lv);
    if (mission_is_loaded())
        mission_restore(tick, (int)state);   /* tick+state only; FSM = M7 */
    return 0;
}
