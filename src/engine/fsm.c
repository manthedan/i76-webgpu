/*
 * fsm.c — Mission FSM (stack-machine scripting) interpreter (see fsm.h)
 *
 * Faithful to the vanilla I'76 semantics in docs/specs/m3/fsm.md:
 * binary-faithful stack layout (slot 0 holds the machine's start address),
 * pass-by-reference IntRef variables, pointer-based argument marshalling for
 * actions, cooperative scheduling with the 800/1000 watchdog, and the
 * decoded Nitro opcodes 2/3/11 (PUSH_REF/PUSH_VAL/GOSUB — stepper
 * FUN_00414db0, node init FUN_004132d0, phase-a §1.2). Only the unused
 * opcodes 0/15 and malformed operands trap now; a trap halts the offending
 * machine cleanly instead of crashing or silently no-oping.
 *
 * Judgment calls (spec gaps resolved here; each is also flagged at the
 * call site):
 *  - RST is the Nitro return opcode (case 0xc): IP = *FP, SP drops n+1
 *    slots, FP restores; the machine ends exactly when SP unwinds to the
 *    node base — i.e. operand == argc at the top frame (B01/P01–P19:
 *    arg == stream argc at every site; A01's few off-argc sites sit in
 *    shared blocks that tools/ghidra/fsm_cfg.py proves are reachable
 *    only from machines with a matching argc). A top-frame RST with any
 *    other operand is corrupt in the binary (FP becomes the zero gap
 *    dword) and traps here.
 *  - COPY_S/PUSH_REF/PUSH_VAL/COPY_B index slots relative to FP, the
 *    binary's frame pointer (spec §3) — at the top frame FP = 0, so
 *    top-level indexing is unchanged; Open76's n-1 is the same layout
 *    minus the slot-0 marker.
 *  - STACK_MOD n<1, POP below the slot-0 marker, out-of-range jump targets,
 *    COPY_B outside the frame or of a non-reference slot, and stack/AS
 *    overflow all take the trap path (Open76 throws; the binary would
 *    just corrupt state).
 *  - The 168-byte machine record's tail after the arg list is opaque
 *    (spec §1.2): skipped, never read, never assumed zero.
 */

#include "engine/fsm.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ----------------------------------------------------------------------- */
/* Constants                                                               */
/* ----------------------------------------------------------------------- */

/* Sanity cap on every u32 table count — real missions hold KBs, not this. */
#define FSM_TABLE_CAP (1u << 20)

/* Machine-table record size and the arg slots that fit in it (spec §1.2). */
#define FSM_MACHINE_RECORD 168
#define FSM_MACHINE_MAX_ARGS ((FSM_MACHINE_RECORD - 8) / 4)   /* 40 */

/* ----------------------------------------------------------------------- */
/* Parsed image                                                            */
/* ----------------------------------------------------------------------- */

typedef struct { uint32_t opcode; int32_t arg; } FsmInstr;

typedef struct {
    char label[41];     /* 40-byte field, NUL-terminated by us */
    char object[9];     /* 8-byte masked object name, NUL-terminated by us */
} FsmEntity;

typedef struct {
    char     name[41];
    uint32_t npoints;
    float   *pts;       /* [npoints*3] x,y,z — world meters (spec §1.1) */
} FsmPath;

typedef struct {
    uint32_t start;                         /* absolute bytecode index */
    uint32_t arg_count;
    int32_t  args[FSM_MACHINE_MAX_ARGS];    /* variable-table indices */
    /* the rest of the 168-byte record is opaque (spec §1.2) — never read */
} FsmMachineDef;

struct FsmImage {
    int            action_count;
    char         (*action_names)[41];
    int            entity_count;
    FsmEntity     *entities;
    int            clip_count;
    char         (*clip_names)[41];
    int            path_count;
    FsmPath       *paths;
    int            machine_count;
    FsmMachineDef *machines;
    int            cell_count;
    int32_t       *cells;       /* shared variable cells (spec table 6) */
    uint32_t       code_count;
    FsmInstr      *code;
};

/* ----------------------------------------------------------------------- */
/* Machine                                                                 */
/* ----------------------------------------------------------------------- */

/* Machine stack slot (spec §2.2). The binary's dword stack holds both
 * plain values and pointers; `ref` is non-NULL exactly when the slot
 * carries a cell reference (pushed by PUSH_REF, or copied by PUSH_VAL
 * from another reference slot or a below-base arg cell). Actions only
 * ever see `v` through COPY_S pointers. */
typedef struct {
    int32_t  v;
    int32_t *ref;
} FsmSlot;

struct FsmMachine {
    FsmImage      *img;
    const FsmHost *host;        /* borrowed — must outlive the machine */
    int            id;          /* machine-table index (for trap logs) */
    uint32_t       start_ip;
    uint32_t       ip;
    int32_t        ar;          /* action result register */
    int            sp;          /* next free stack slot; slot 0 = start */
    int            fp;          /* frame pointer (stack slot index) —    */
                                /* 0 at the top frame (the marker slot); */
                                /* GOSUB/RST move it (Nitro semantics)   */
    FsmSlot        stk[FSM_STACK_MAX];
    int32_t      **frame;       /* [frame_len] aliases into img->cells */
    int            frame_len;
    int32_t       *as[FSM_ARGS_MAX];    /* action argument pointer FIFO */
    int            as_count;
    bool           halted;
    bool           watchdog_tripped;    /* latched once a slice exceeds WARN */
    int            slice_count;         /* instructions in the last fsm_step */
};

/* ----------------------------------------------------------------------- */
/* Trap path (unknown opcode / bad operand — spec §7)                      */
/* ----------------------------------------------------------------------- */

static FsmStepResult fsm_trap(FsmMachine *m, uint32_t pc, uint32_t opcode,
                              const char *why)
{
    fprintf(stderr, "[fsm] trap: machine=%d pc=%u opcode=%u: %s\n",
            m->id, pc, opcode, why);
    if (m->host && m->host->trap)
        m->host->trap(m->host->ud, m->id, pc, opcode);
    m->halted = true;
    return FSM_STEP_TRAPPED;
}

/* ----------------------------------------------------------------------- */
/* Little-endian cursor (the on-disk format is x86 little-endian)          */
/* ----------------------------------------------------------------------- */

typedef struct {
    const uint8_t *cur;
    const uint8_t *end;
} Cursor;

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool take(Cursor *c, size_t n, const uint8_t **out)
{
    if ((size_t)(c->end - c->cur) < n)
        return false;
    *out = c->cur;
    c->cur += n;
    return true;
}

static bool rd_u32(Cursor *c, uint32_t *out)
{
    const uint8_t *p;
    if (!take(c, 4, &p))
        return false;
    *out = le32(p);
    return true;
}

/* 40-byte fixed name field -> NUL-terminated char[41] */
static bool rd_name(Cursor *c, char out[41])
{
    const uint8_t *p;
    if (!take(c, 40, &p))
        return false;
    memcpy(out, p, 40);
    out[40] = '\0';
    return true;
}

/* Count sanity: capped, and count fixed-size entries must fit the rest. */
static bool table_size_ok(const Cursor *c, uint32_t count, size_t entry_size)
{
    if (count > FSM_TABLE_CAP)
        return false;
    return (size_t)(c->end - c->cur) / entry_size >= count;
}

/* ----------------------------------------------------------------------- */
/* Image load / free                                                       */
/* ----------------------------------------------------------------------- */

static FsmImage *parse_fail(FsmImage *img, const char *what)
{
    fprintf(stderr, "[fsm] image load failed: %s\n", what);
    fsm_image_free(img);
    return NULL;
}

#define PARSE_CHECK(cond, what) do { \
    if (!(cond)) return parse_fail(img, what); } while (0)

FsmImage *fsm_image_load(const void *buf, size_t size)
{
    if (!buf) {
        fprintf(stderr, "[fsm] image load failed: NULL buffer\n");
        return NULL;
    }

    FsmImage *img = calloc(1, sizeof(*img));
    if (!img)
        return NULL;

    Cursor c = { buf, (const uint8_t *)buf + size };
    uint32_t count;

    /* 1. action table: count × char[40] */
    PARSE_CHECK(rd_u32(&c, &count) && table_size_ok(&c, count, 40),
                "action table truncated");
    img->action_count = (int)count;
    if (count) {
        img->action_names = calloc(count, sizeof(*img->action_names));
        if (!img->action_names) return parse_fail(img, "out of memory");
        for (uint32_t i = 0; i < count; i++)
            PARSE_CHECK(rd_name(&c, img->action_names[i]),
                        "action name truncated");
    }

    /* 2. entity table: count × { char[40] label; byte[8] masked name } */
    PARSE_CHECK(rd_u32(&c, &count) && table_size_ok(&c, count, 48),
                "entity table truncated");
    img->entity_count = (int)count;
    if (count) {
        img->entities = calloc(count, sizeof(*img->entities));
        if (!img->entities) return parse_fail(img, "out of memory");
        for (uint32_t i = 0; i < count; i++) {
            const uint8_t *obj;
            PARSE_CHECK(rd_name(&c, img->entities[i].label) &&
                        take(&c, 8, &obj), "entity entry truncated");
            memcpy(img->entities[i].object, obj, 8);
            img->entities[i].object[8] = '\0';
        }
    }

    /* 3. sound-clip table: count × char[40] */
    PARSE_CHECK(rd_u32(&c, &count) && table_size_ok(&c, count, 40),
                "sound-clip table truncated");
    img->clip_count = (int)count;
    if (count) {
        img->clip_names = calloc(count, sizeof(*img->clip_names));
        if (!img->clip_names) return parse_fail(img, "out of memory");
        for (uint32_t i = 0; i < count; i++)
            PARSE_CHECK(rd_name(&c, img->clip_names[i]),
                        "sound-clip name truncated");
    }

    /* 4. path table: count × { char[40] name; u32 npoints; npoints×3 f32 } */
    PARSE_CHECK(rd_u32(&c, &count) && table_size_ok(&c, count, 44),
                "path table truncated");
    img->path_count = (int)count;
    if (count) {
        img->paths = calloc(count, sizeof(*img->paths));
        if (!img->paths) return parse_fail(img, "out of memory");
        for (uint32_t i = 0; i < count; i++) {
            FsmPath *p = &img->paths[i];
            uint32_t npoints;
            PARSE_CHECK(rd_name(&c, p->name), "path name truncated");
            PARSE_CHECK(rd_u32(&c, &npoints) &&
                        table_size_ok(&c, npoints, 12),
                        "path points truncated");
            p->npoints = npoints;
            if (npoints) {
                p->pts = malloc((size_t)npoints * 3 * sizeof(float));
                if (!p->pts) return parse_fail(img, "out of memory");
                for (uint32_t j = 0; j < npoints * 3; j++) {
                    uint32_t bits;
                    PARSE_CHECK(rd_u32(&c, &bits), "path points truncated");
                    memcpy(&p->pts[j], &bits, 4);
                }
            }
        }
    }

    /* 5. machine table: count × 168-byte records (spec §1.2) */
    PARSE_CHECK(rd_u32(&c, &count) && table_size_ok(&c, count, FSM_MACHINE_RECORD),
                "machine table truncated");
    img->machine_count = (int)count;
    if (count) {
        img->machines = calloc(count, sizeof(*img->machines));
        if (!img->machines) return parse_fail(img, "out of memory");
        for (uint32_t i = 0; i < count; i++) {
            const uint8_t *rec;
            FsmMachineDef *def;
            PARSE_CHECK(take(&c, FSM_MACHINE_RECORD, &rec),
                        "machine record truncated");
            def = &img->machines[i];
            def->start     = le32(rec);
            def->arg_count = le32(rec + 4);
            PARSE_CHECK(def->arg_count <= FSM_MACHINE_MAX_ARGS,
                        "machine arg count exceeds record");
            for (uint32_t j = 0; j < def->arg_count; j++)
                def->args[j] = (int32_t)le32(rec + 8 + 4 * j);
            /* bytes 8 + 4*arg_count .. 168 are opaque — never read */
        }
    }

    /* 6. variable table ("Block 2"): count × i32 — the shared cells */
    PARSE_CHECK(rd_u32(&c, &count) && table_size_ok(&c, count, 4),
                "variable table truncated");
    img->cell_count = (int)count;
    if (count) {
        img->cells = calloc(count, sizeof(*img->cells));
        if (!img->cells) return parse_fail(img, "out of memory");
        for (uint32_t i = 0; i < count; i++) {
            uint32_t v;
            PARSE_CHECK(rd_u32(&c, &v), "variable table truncated");
            img->cells[i] = (int32_t)v;
        }
    }

    /* 7. bytecode ("Block 3"): count × { u32 opcode; i32 arg } */
    PARSE_CHECK(rd_u32(&c, &count) && table_size_ok(&c, count, 8),
                "bytecode truncated");
    img->code_count = count;
    if (count) {
        img->code = calloc(count, sizeof(*img->code));
        if (!img->code) return parse_fail(img, "out of memory");
        for (uint32_t i = 0; i < count; i++) {
            uint32_t op, arg;
            PARSE_CHECK(rd_u32(&c, &op) && rd_u32(&c, &arg),
                        "bytecode truncated");
            img->code[i].opcode = op;
            img->code[i].arg    = (int32_t)arg;
        }
    }

    return img;
}

void fsm_image_free(FsmImage *img)
{
    if (!img)
        return;
    for (int i = 0; i < img->path_count; i++)
        free(img->paths[i].pts);
    free(img->paths);
    free(img->action_names);
    free(img->entities);
    free(img->clip_names);
    free(img->machines);
    free(img->cells);
    free(img->code);
    free(img);
}

int fsm_image_machine_count(const FsmImage *img)
{
    return img ? img->machine_count : 0;
}

int fsm_image_action_count(const FsmImage *img)
{
    return img ? img->action_count : 0;
}

const char *fsm_image_action_name(const FsmImage *img, int index)
{
    if (!img || index < 0 || index >= img->action_count)
        return NULL;
    return img->action_names[index];
}

int32_t fsm_image_get_cell(const FsmImage *img, int index)
{
    if (!img || index < 0 || index >= img->cell_count)
        return 0;
    return img->cells[index];
}

void fsm_image_set_cell(FsmImage *img, int index, int32_t value)
{
    if (!img || index < 0 || index >= img->cell_count)
        return;
    img->cells[index] = value;
}

/* ----------------------------------------------------------------------- */
/* Read-only table accessors (host bridges)                                */
/* ----------------------------------------------------------------------- */

int fsm_image_entity_count(const FsmImage *img)
{
    return img ? img->entity_count : 0;
}

const char *fsm_image_entity_label(const FsmImage *img, int index)
{
    if (!img || index < 0 || index >= img->entity_count)
        return NULL;
    return img->entities[index].label;
}

const char *fsm_image_entity_object(const FsmImage *img, int index)
{
    if (!img || index < 0 || index >= img->entity_count)
        return NULL;
    return img->entities[index].object;
}

int fsm_image_clip_count(const FsmImage *img)
{
    return img ? img->clip_count : 0;
}

const char *fsm_image_clip_name(const FsmImage *img, int index)
{
    if (!img || index < 0 || index >= img->clip_count)
        return NULL;
    return img->clip_names[index];
}

int fsm_image_path_count(const FsmImage *img)
{
    return img ? img->path_count : 0;
}

int fsm_image_path(const FsmImage *img, int index, const float **pts)
{
    if (pts)
        *pts = NULL;
    if (!img || index < 0 || index >= img->path_count)
        return 0;
    if (pts)
        *pts = img->paths[index].pts;
    return (int)img->paths[index].npoints;
}

/* ----------------------------------------------------------------------- */
/* Machine create / destroy                                                */
/* ----------------------------------------------------------------------- */

FsmMachine *fsm_machine_create(FsmImage *img, int machine_index,
                               const FsmHost *host)
{
    const FsmMachineDef *def;
    FsmMachine *m;

    if (!img || machine_index < 0 || machine_index >= img->machine_count) {
        fprintf(stderr, "[fsm] create: bad machine index %d\n", machine_index);
        return NULL;
    }
    def = &img->machines[machine_index];

    if (def->start >= img->code_count) {
        fprintf(stderr, "[fsm] create: machine %d start %u out of range "
                        "(%u instructions)\n",
                machine_index, def->start, img->code_count);
        return NULL;
    }
    for (uint32_t i = 0; i < def->arg_count; i++) {
        if (def->args[i] < 0 || def->args[i] >= img->cell_count) {
            fprintf(stderr, "[fsm] create: machine %d arg %u = cell %d "
                            "out of range (%d cells)\n",
                    machine_index, i, def->args[i], img->cell_count);
            return NULL;
        }
    }

    m = calloc(1, sizeof(*m));
    if (!m)
        return NULL;
    m->img       = img;
    m->host      = host;
    m->id        = machine_index;
    m->start_ip  = def->start;
    m->ip        = def->start;
    m->frame_len = (int)def->arg_count;

    /* Frame: pointers into the shared cells — aliases, not copies (§2.3). */
    if (m->frame_len > 0) {
        m->frame = malloc((size_t)m->frame_len * sizeof(*m->frame));
        if (!m->frame) {
            free(m);
            return NULL;
        }
        for (int i = 0; i < m->frame_len; i++)
            m->frame[i] = &img->cells[def->args[i]];
    }

    m->stk[0].v = (int32_t)def->start;  /* slot 0 = start address (§2.2) */
    m->sp       = 1;
    m->fp       = 0;    /* top frame: FP points at the marker slot (the
                         * binary's FUN_004132d0 init lays args + a zero
                         * gap + the start-IP marker below SP; the frame
                         * cells above are normalized into m->frame)     */
    return m;
}

void fsm_machine_destroy(FsmMachine *m)
{
    if (!m)
        return;
    free(m->frame);
    free(m);
}

bool fsm_machine_halted(const FsmMachine *m)
{
    return !m || m->halted;
}

bool fsm_machine_watchdog_tripped(const FsmMachine *m)
{
    return m && m->watchdog_tripped;
}

int32_t fsm_machine_ar(const FsmMachine *m)
{
    return m ? m->ar : 0;
}

int fsm_machine_slice_count(const FsmMachine *m)
{
    return m ? m->slice_count : 0;
}

/* ----------------------------------------------------------------------- */
/* Interpreter                                                             */
/* ----------------------------------------------------------------------- */

/* Jump targets are absolute bytecode indices (spec §1.1). */
static bool fsm_jump(FsmMachine *m, int32_t target)
{
    if (target < 0 || (uint32_t)target >= m->img->code_count)
        return false;
    m->ip = (uint32_t)target;
    return true;
}

FsmStepResult fsm_step(FsmMachine *m)
{
    FsmImage *img;

    if (!m || m->halted)
        return FSM_STEP_HALTED;
    img = m->img;
    m->slice_count = 0;

    for (;;) {
        uint32_t pc;
        FsmInstr in;

        if (m->ip >= img->code_count)
            return fsm_trap(m, m->ip, 0, "instruction pointer out of range");

        pc = m->ip;
        in = img->code[pc];
        m->ip = pc + 1;         /* default: fall through */
        m->slice_count++;

        switch (in.opcode) {
        case FSM_OP_PUSH:                       /* *SP++ = arg */
            if (m->sp >= FSM_STACK_MAX)
                return fsm_trap(m, pc, in.opcode, "stack overflow");
            m->stk[m->sp].v   = in.arg;
            m->stk[m->sp].ref = NULL;
            m->sp++;
            break;

        case FSM_OP_PUSH_REF: {                 /* *SP++ = &slot(FP+n) */
            /* Binary case 2: pushes the slot's ADDRESS. Slots below FP
             * are the initial-arg pointer cells — their addresses are
             * pointer-to-pointer, useless to actions; trap like the
             * other below-base address forms. */
            const int idx = m->fp + in.arg;
            if (idx < 0)
                return fsm_trap(m, pc, in.opcode,
                                "PUSH_REF of a below-base frame slot");
            if (idx >= FSM_STACK_MAX)
                return fsm_trap(m, pc, in.opcode, "PUSH_REF slot out of range");
            if (m->sp >= FSM_STACK_MAX)
                return fsm_trap(m, pc, in.opcode, "stack overflow");
            m->stk[m->sp].v   = 0;
            m->stk[m->sp].ref = &m->stk[idx].v;
            m->sp++;
            break;
        }

        case FSM_OP_PUSH_VAL: {                 /* *SP++ = slot(FP+n) */
            /* Binary case 3: pushes the slot's VALUE. A below-base slot
             * holds an initial-arg cell pointer, so the pushed value is
             * a reference — this is how callers pass their arg cells to
             * a GOSUB subroutine by reference. */
            const int idx = m->fp + in.arg;
            int32_t  v   = 0;
            int32_t *ref = NULL;
            if (idx >= 0) {
                if (idx >= FSM_STACK_MAX)
                    return fsm_trap(m, pc, in.opcode,
                                    "PUSH_VAL slot out of range");
                v   = m->stk[idx].v;
                ref = m->stk[idx].ref;
            } else {
                const int f = m->frame_len + idx + 1;
                if (f < 0 || f >= m->frame_len)
                    return fsm_trap(m, pc, in.opcode,
                                    "PUSH_VAL frame slot out of range");
                ref = m->frame[f];
            }
            if (m->sp >= FSM_STACK_MAX)
                return fsm_trap(m, pc, in.opcode, "stack overflow");
            m->stk[m->sp].v   = v;
            m->stk[m->sp].ref = ref;
            m->sp++;
            break;
        }

        case FSM_OP_COPY_S: {                   /* enqueue &slot(FP+n) */
            /* FP-relative (binary case 4): at the top frame FP = 0, so
             * top-level COPY_S n still addresses stack slot n directly.
             * Stale slots above SP stay readable: PUSH/POP are
             * non-destructive, only SP moves (§2.2). */
            const int idx = m->fp + in.arg;
            if (idx < 0 || idx >= FSM_STACK_MAX)
                return fsm_trap(m, pc, in.opcode, "COPY_S slot out of range");
            if (m->as_count >= FSM_ARGS_MAX)
                return fsm_trap(m, pc, in.opcode, "argument stack overflow");
            m->as[m->as_count++] = &m->stk[idx].v;
            break;
        }

        case FSM_OP_COPY_B: {                   /* enqueue slot(FP+n) value */
            /* Binary case 5: enqueues the slot's VALUE, which is only a
             * usable action argument when it is a cell reference —
             * below-base slots hold the machine's initial-arg cell
             * pointers (wired at create, spec §2.2/§2.3), and stack
             * slots carry references only when PUSH_REF/PUSH_VAL put one
             * there (the GOSUB argument-passing idiom: the caller pushes
             * refs, the callee COPY_Bs them into AS). Enqueueing a plain
             * dword would hand actions a bogus pointer (the binary just
             * crashes); that takes the trap path instead.
             *
             * Reachability note: with opcodes 2/3/11 decoded, P01's
             * m26–30 COPY_B(-3)/(-4) sites at 2738+ remain statically
             * dead — the whole subroutine region has no inbound jumps in
             * any of the 20 scripted Nitro missions (B01 + P01–P19; the
             * only GOSUB site sits inside the dead region itself), and
             * the region was compiled for argc=3 machines (OP3 -4/-2/-3
             * + RST 3) so it would trap out-of-frame under the argc=1
             * stream anyway. */
            const int idx = m->fp + in.arg;
            int32_t *ref;
            if (idx >= 0) {
                if (idx >= FSM_STACK_MAX)
                    return fsm_trap(m, pc, in.opcode,
                                    "COPY_B slot out of range");
                ref = m->stk[idx].ref;
                if (!ref)
                    return fsm_trap(m, pc, in.opcode,
                                    "COPY_B of a non-reference stack slot");
            } else {
                const int f = m->frame_len + idx + 1;
                if (f < 0 || f >= m->frame_len)
                    return fsm_trap(m, pc, in.opcode,
                                    "COPY_B frame slot out of range");
                ref = m->frame[f];
            }
            if (m->as_count >= FSM_ARGS_MAX)
                return fsm_trap(m, pc, in.opcode, "argument stack overflow");
            m->as[m->as_count++] = ref;
            break;
        }

        case FSM_OP_STACK_MOD: {                /* SP += n, zero-filled */
            if (in.arg < 1)
                return fsm_trap(m, pc, in.opcode, "STACK_MOD count < 1");
            if (m->sp + in.arg > FSM_STACK_MAX)
                return fsm_trap(m, pc, in.opcode, "stack overflow");
            for (int32_t i = 0; i < in.arg; i++) {
                m->stk[m->sp + i].v   = 0;
                m->stk[m->sp + i].ref = NULL;
            }
            m->sp += in.arg;
            break;
        }

        case FSM_OP_POP:                        /* SP -= n (non-destructive) */
            /* The slot-0 start marker is never popped. */
            if (in.arg < 0 || m->sp - in.arg < 1)
                return fsm_trap(m, pc, in.opcode, "POP underflow");
            m->sp -= in.arg;
            break;

        case FSM_OP_JMP:
            if (!fsm_jump(m, in.arg))
                return fsm_trap(m, pc, in.opcode, "jump target out of range");
            break;

        case FSM_OP_JZ:
            if (m->ar == 0 && !fsm_jump(m, in.arg))
                return fsm_trap(m, pc, in.opcode, "jump target out of range");
            break;

        case FSM_OP_JMP_I:                      /* jump, then de-schedule */
            if (!fsm_jump(m, in.arg))
                return fsm_trap(m, pc, in.opcode, "jump target out of range");
            return FSM_STEP_YIELD;

        case FSM_OP_GOSUB: {                    /* Nitro call (case 0xb) */
            /* Binary: push FP, then FP = the next slot, *FP = return
             * address, SP advances past both, jump. The callee's frame
             * slots above FP are fresh locals; slots below FP are the
             * saved FP and the caller's pushed arguments. */
            if (in.arg < 0 || (uint32_t)in.arg >= img->code_count)
                return fsm_trap(m, pc, in.opcode, "jump target out of range");
            if (m->sp + 2 > FSM_STACK_MAX)
                return fsm_trap(m, pc, in.opcode, "stack overflow");
            m->stk[m->sp].v       = m->fp;      /* saved frame pointer */
            m->stk[m->sp].ref     = NULL;
            m->fp                 = m->sp + 1;
            m->stk[m->sp + 1].v   = (int32_t)m->ip;  /* return address */
            m->stk[m->sp + 1].ref = NULL;
            m->sp                += 2;
            m->ip                 = (uint32_t)in.arg;
            break;
        }

        case FSM_OP_RST: {                      /* return (case 0xc) */
            /* Binary: IP = *FP, SP = FP − (n+1) slots, FP = saved FP;
             * the machine is done exactly when SP unwinds to the node
             * base. In this normalized stack the node base sits
             * frame_len+1 slots below slot 0 (the arg-pointer cells plus
             * the zero gap the binary's init lays out), so the terminal
             * unwind is sp == -(frame_len+1) — reached by a top-frame
             * RST whose operand equals argc, which is how scripted
             * missions end (see the file header). Anything else at the
             * top frame leaves the binary with FP = 0 and a corrupted
             * stack: trap. */
            const int t     = m->fp;
            const int newsp = t - in.arg - 1;
            if (newsp == -(m->frame_len + 1)) {
                /* terminal unwind: reset IP/SP/FP, then halt for good */
                m->ip       = m->start_ip;
                m->sp       = 1;
                m->fp       = 0;
                m->as_count = 0;
                m->halted   = true;
                return FSM_STEP_RST;
            }
            if (newsp < 0 || t < 1)
                return fsm_trap(m, pc, in.opcode, "return stack underflow");
            /* The saved FP must sit below the current frame — frames
             * only grow upward. */
            if (m->stk[t - 1].v < 0 || m->stk[t - 1].v >= t)
                return fsm_trap(m, pc, in.opcode, "return frame corrupt");
            m->ip = (uint32_t)m->stk[t].v;  /* fetch bounds-checks it */
            m->fp = m->stk[t - 1].v;
            m->sp = newsp;
            break;
        }

        case FSM_OP_ACTION: {                   /* dispatch via host */
            if (in.arg < 0 || in.arg >= img->action_count)
                return fsm_trap(m, pc, in.opcode, "action index out of range");
            if (!m->host || !m->host->dispatch)
                return fsm_trap(m, pc, in.opcode,
                                "ACTION with no host dispatch bound");
            m->ar = m->host->dispatch(m->host->ud, in.arg,
                                      img->action_names[in.arg],
                                      m->as, m->as_count);
            m->as_count = 0;    /* AS is reset after every ACTION (§2.2) */
            break;
        }

        case FSM_OP_NEG:
            m->ar = (m->ar == 1) ? 0 : 1;
            break;

        default:
            /* 0/15 unused (spec §7). */
            return fsm_trap(m, pc, in.opcode, "unknown opcode");
        }

        /* Watchdog (§2.4): flag latches past 800, force de-schedule at 1000. */
        if (m->slice_count > FSM_WATCHDOG_WARN)
            m->watchdog_tripped = true;
        if (m->slice_count >= FSM_WATCHDOG_HARD)
            return FSM_STEP_WATCHDOG;
    }
}
