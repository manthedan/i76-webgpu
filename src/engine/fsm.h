#ifndef FSM_H
#define FSM_H

/*
 * fsm.h — Mission FSM (stack-machine scripting) interpreter
 *
 * Pure-C reimplementation of the ADEF/`FSM ` virtual machine that drives
 * Interstate '76 missions, per docs/specs/m3/fsm.md. An FsmImage is the
 * parsed content of a mission's FSM chunk (all seven sub-tables); an
 * FsmMachine is one running script bound to that image.
 *
 * Shared-variable model (spec §2.3, the "IntRef" model): the image owns the
 * global variable cells; a machine's frame holds *pointers* into those cells
 * (its initial arguments), so two machines created from the same image
 * observe each other's writes — pass by reference, never by value.
 *
 * Cooperative scheduling (spec §2.4): fsm_step() runs one bounded
 * instruction slice — at most FSM_WATCHDOG_HARD instructions — returning
 * when the machine yields (JMP_I), ends (RST), trips the watchdog, traps,
 * or was already halted. The caller round-robins machines.
 *
 * Nitro opcodes 2/3/11 are decoded and implemented (Nitro stepper
 * FUN_00414db0 + node init FUN_004132d0, phase-a §1.2): PUSH_REF pushes a
 * frame slot's address, PUSH_VAL its value, GOSUB calls a subroutine
 * through the machine stack, and RST is the matching return (its operand
 * pops caller slots; argc at the top frame ends the machine). Only the
 * unused opcodes 0 and 15 trap now: the machine id, pc and opcode are
 * logged to stderr, the optional host trap observer fires, and the
 * machine halts cleanly. Nothing crashes and nothing silently no-ops.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ----------------------------------------------------------------------- */
/* Opcodes (spec §3; values are the vanilla binary's)                        */
/* ----------------------------------------------------------------------- */

enum {
    FSM_OP_PUSH      = 1,   /* *SP++ = arg (immediate)                     */
    FSM_OP_PUSH_REF  = 2,   /* *SP++ = &frame_slot(n) — Nitro (spec §3)    */
    FSM_OP_PUSH_VAL  = 3,   /* *SP++ = frame_slot(n) value — Nitro         */
    FSM_OP_COPY_S    = 4,   /* enqueue pointer to stack slot n onto AS     */
    FSM_OP_COPY_B    = 5,   /* enqueue slot-n value (a cell ref) onto AS   */
    FSM_OP_STACK_MOD = 6,   /* SP += n, zero-filling the new slots         */
    FSM_OP_POP       = 7,   /* SP -= n (non-destructive)                   */
    FSM_OP_JMP       = 8,   /* IP = arg (absolute bytecode index)          */
    FSM_OP_JZ        = 9,   /* if (AR == 0) IP = arg                       */
    FSM_OP_JMP_I     = 10,  /* IP = arg, then yield to the next machine    */
    FSM_OP_GOSUB     = 11,  /* push FP + return addr, jump — Nitro call    */
    FSM_OP_RST       = 12,  /* return: pop n slots; n == argc ends machine */
    FSM_OP_ACTION    = 13,  /* dispatch action-table entry; AR = result    */
    FSM_OP_NEG       = 14   /* AR = (AR == 1) ? 0 : 1                      */
    /* 0 and 15 are unused — they trap. */
};

/* ----------------------------------------------------------------------- */
/* Limits                                                                    */
/* ----------------------------------------------------------------------- */

/* Watchdog (spec §2.4): the flag latches once a slice exceeds WARN
 * instructions without yielding; the slice is force-ended at HARD. */
#define FSM_WATCHDOG_WARN 800
#define FSM_WATCHDOG_HARD 1000

/* Per-machine capacities. Implementation choice, not reversed: the binary
 * uses fixed engine buffers whose sizes we have not recovered. Exceeding
 * either takes the trap path. */
#define FSM_STACK_MAX 256   /* dword stack cells; slot 0 = start marker    */
#define FSM_ARGS_MAX  16    /* argument-stack (AS) pointers per ACTION     */

typedef struct FsmImage   FsmImage;
typedef struct FsmMachine FsmMachine;

typedef enum {
    FSM_STEP_YIELD,     /* JMP_I: machine yielded; resumes at target       */
    FSM_STEP_RST,       /* RST: script reset and ended; machine halted     */
    FSM_STEP_WATCHDOG,  /* slice budget exhausted; force de-scheduled      */
    FSM_STEP_TRAPPED,   /* unknown opcode / bad operand; machine halted    */
    FSM_STEP_HALTED     /* machine was already halted                      */
} FsmStepResult;

/*
 * Host callback table — game systems plug in here. The table and `ud` are
 * borrowed: they must outlive every machine created with them.
 *
 * dispatch: run action `action_index` (name from the image's action table).
 *   args[0..nargs) are pointers to live cells queued by COPY_S/COPY_B —
 *   read and write through them (this is how set/inc/rand work at all).
 *   The return value becomes AR. May be NULL only if no program in the
 *   image executes ACTION; an ACTION with no dispatch bound traps.
 * trap: optional observer, fired after the stderr log just before the
 *   offending machine halts. May be NULL.
 */
typedef struct {
    int32_t (*dispatch)(void *ud, int action_index, const char *action_name,
                        int32_t **args, int nargs);
    void (*trap)(void *ud, int machine_id, uint32_t pc, uint32_t opcode);
    void *ud;
} FsmHost;

/* ----------------------------------------------------------------------- */
/* Image                                                                     */
/* ----------------------------------------------------------------------- */

/*
 * Parse a raw `FSM ` chunk payload (all seven sub-tables, spec §1.1) into
 * an image. Returns NULL on a truncated or malformed buffer (logged to
 * stderr). The byte buffer may be freed after this call; the image owns no
 * references into it.
 */
FsmImage *fsm_image_load(const void *buf, size_t size);
void      fsm_image_free(FsmImage *img);   /* all machines must be destroyed first */

int         fsm_image_machine_count(const FsmImage *img);
int         fsm_image_action_count(const FsmImage *img);
const char *fsm_image_action_name(const FsmImage *img, int index); /* NULL if out of range */

/* Shared variable cells (spec table 6). Out-of-range get returns 0;
 * out-of-range set is ignored. */
int32_t fsm_image_get_cell(const FsmImage *img, int index);
void    fsm_image_set_cell(FsmImage *img, int index, int32_t value);

/* ----------------------------------------------------------------------- */
/* Machine                                                                   */
/* ----------------------------------------------------------------------- */

/*
 * Create machine `machine_index` from the image's machine table. Its frame
 * aliases the image's shared cells (IntRef model — spec §2.3). Returns NULL
 * (logged) if the index is bad or the record is corrupt (start address past
 * the bytecode, or an initial-argument index outside the variable table).
 */
FsmMachine *fsm_machine_create(FsmImage *img, int machine_index,
                               const FsmHost *host);
void        fsm_machine_destroy(FsmMachine *m);

/* Run one bounded instruction slice (see header comment). */
FsmStepResult fsm_step(FsmMachine *m);

bool    fsm_machine_halted(const FsmMachine *m);
bool    fsm_machine_watchdog_tripped(const FsmMachine *m); /* latched at WARN */
int32_t fsm_machine_ar(const FsmMachine *m);               /* action result    */
int     fsm_machine_slice_count(const FsmMachine *m);      /* instrs last slice */

/* ----------------------------------------------------------------------- */
/* Read-only table accessors (for host bridges, e.g. the mission runner)    */
/* ----------------------------------------------------------------------- */

/* Entity table (spec table 2). The label is the script-facing name; the
 * object is the 8-byte masked ODEF label (7-bit ASCII; high bits form the
 * duplicate id — same packing as scene.c's label_unpack). NULL when the
 * index is out of range. */
int         fsm_image_entity_count(const FsmImage *img);
const char *fsm_image_entity_label(const FsmImage *img, int index);
const char *fsm_image_entity_object(const FsmImage *img, int index);

/* Sound-clip table (spec table 3): NUL-terminated filenames. NULL oob. */
int         fsm_image_clip_count(const FsmImage *img);
const char *fsm_image_clip_name(const FsmImage *img, int index);

/* Path table (spec table 4). fsm_image_path returns the point count and
 * sets *pts to npoints*3 floats (x,y,z, world meters), owned by the image.
 * Returns 0 and sets *pts to NULL when the index is out of range. */
int         fsm_image_path_count(const FsmImage *img);
int         fsm_image_path(const FsmImage *img, int index, const float **pts);

#endif /* FSM_H */
