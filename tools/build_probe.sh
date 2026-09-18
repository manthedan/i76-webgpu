#!/bin/bash
# build_probe.sh — build a native probe with the CORRECT engine source list.
#
# Why this exists: every probe carried its own hand-maintained `cc` line in a
# header comment, and eight of them had gone stale. Each of the following
# undefined symbols was reachable from a documented build line that no longer
# linked:
#
#   raster_polygon  raster_rgb_to_index  raster_palette_generation
#   texcache_resolve  texcache_tile  texcache_reset  texcache_generation
#
# A probe that cannot be built from its own instructions is a gate nobody can
# run, so the source list now lives in ONE place that a link error updates.
#
# Two things it deliberately does NOT do:
#   - glob src/engine/*.c. ai.c and combat.c are UNITY-INCLUDED by mission.c,
#     so a glob produces "multiple definition" errors. That trap is what the
#     hand-written lists were avoiding, and the fix is a curated list, not a
#     wildcard.
#   - drop -ffp-contract=off. wasm has no f64 FMA, so a contracted native build
#     cannot match the wasm one; the flag is a determinism contract shared with
#     web/build.sh and tools/frame_gate.sh.
#
# Usage:  tools/build_probe.sh <probe-name> [extra cc args...]
#         tools/build_probe.sh --list
# Output: $OUT/<probe-name>. OUT is required and must name a directory
# outside the checkout so binaries and diagnostics cannot enter a source tree.
#
# Example:
#   OUT="$I76_VERIFY_OUT/bin" tools/build_probe.sh raster_test
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd -P)"
OUT="${OUT:?set OUT to an explicit directory outside the checkout}"
[[ "$OUT" = /* ]] || { echo 'build_probe: OUT must be absolute' >&2; exit 2; }
OUT="$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve())' "$OUT")" || exit 2
case "$OUT" in
  /*) ;;
  *) echo "build_probe: OUT must be an absolute path" >&2; exit 2 ;;
esac
case "$OUT/" in
  "$ROOT/"*) echo "build_probe: OUT must be outside $ROOT" >&2; exit 2 ;;
esac

# Engine translation units, curated.
#
# Two exclusions that are load-bearing rather than tidy:
#   - ai.c and combat.c are UNITY-INCLUDED by mission.c. Adding them here gives
#     "multiple definition" at link, which is exactly the trap the old
#     hand-written lists existed to avoid.
#   - loop.c, gamestate.c, input.c, cmdline.c and meshview.c are the APPLICATION
#     layer. They reference platform_*/render_* and would drag SDL and Vulkan
#     into a headless probe. A probe drives the engine directly; it never runs
#     the game loop.
ENGINE_ALL=(
  fs zfs vfs geomesh meshcache camera terrain scene raster worldrender
  texcache vqm pixidx paint fsm mission component car hud font paperui m16 pcx sound
  strlookup save
)

# Per-probe source sets. A probe not listed here gets ENGINE_ALL, which links
# everything and is always correct if occasionally larger than necessary.
srcs_for() {
  case "$1" in
    raster_test|pixel_history_probe) echo "raster" ;;
    vfs_probe_log_test) echo "fs zfs vfs" ;;
    hud_probe)   echo "fs zfs vfs hud font m16 pcx vqm" ;;
    # paperui pulls terrain bounds (map pin) + raster_rgb_to_index + pcx.
    paper_probe) echo "fs zfs vfs paperui hud font m16 pcx vqm raster \
            geomesh meshcache camera terrain scene pixidx paint texcache" ;;
    paint_probe) echo "fs zfs vfs paint" ;;
    tex_verify|tex_survey) echo "fs zfs vfs geomesh vqm pixidx" ;;
    pixidx_probe) echo "fs zfs vfs pixidx" ;;
    component_probe) echo "fs zfs vfs component" ;;
    # The deep handbrake probes drive input.c's real steering accumulator;
    # each probe supplies a headless platform_pump_events stub.
    handbrake_probe|handbrake_input_probe) echo "${ENGINE_ALL[@]} input" ;;
    # Unity-includes sound.c behind a synthetic VFS to pin binary parsing.
    sound_wav_test) echo "" ;;
    # aismooth_probe unity-includes ai.c ALONE and drives it against a
    # synthetic nav world: no assets, no other engine units. The default
    # ENGINE_ALL would link mission.c, which unity-includes ai.c again —
    # a multiple definition, not a duplicate-but-harmless object.
    aismooth_probe) echo "" ;;
    # sky_probe stays off mission.c on purpose: mission.c unity-includes
    # ai.c/combat.c, and the sky gate has no business depending on the AI
    # slice's compile state.
    sky_probe) echo "fs zfs vfs geomesh meshcache camera terrain scene raster \
            worldrender texcache vqm pixidx paint hud font m16 pcx" ;;
    # fsm_dump, fsm_matrix, and focused FSM action probes unity-include
    # fsm.c AND mission.c (which in turn brings ai.c and combat.c) so they
    # can inspect static runner state. Linking those again would be a
    # multiple definition, not a duplicate-but-harmless object.
    fsm_dump|fsm_matrix|movie_probe|radio_probe|gate_probe|timer_probe)
      echo "fs zfs vfs geomesh meshcache camera terrain scene raster worldrender \
            texcache vqm pixidx paint component car hud font m16 pcx sound strlookup save" ;;
    # adv_ctf_probe and adv_melee_probe unity-include mission.c ONLY -- they
    # read the built objective state (s_ctf, s_melee), which no header
    # exposes -- so mission.c is excluded here but fsm.c is still needed at
    # link. aiworld_probe does the same for the static world-contact stage
    # (ai_world_contacts_tick, s_world_boxes, ent_pos, mission_ent_writeback).
    adv_ctf_probe|adv_melee_probe|av_melee_probe|aiworld_probe|ai_physics_parity|ally_proximity_probe|h077_diag_probe)
      echo "fs zfs vfs geomesh meshcache camera terrain scene raster worldrender \
            texcache vqm pixidx paint fsm component car hud font m16 pcx sound \
            strlookup save" ;;
    *) echo "${ENGINE_ALL[@]}" ;;
  esac
}

if [[ "${1:-}" == "--list" ]]; then
  for f in "$ROOT"/tools/*.c; do
    b="$(basename "$f" .c)"
    printf '%-18s %s\n' "$b" "$(srcs_for "$b")"
  done
  exit 0
fi

probe="${1:?usage: build_probe.sh <probe-name> [extra cc args...]}"
shift || true
[[ "$probe" =~ ^[A-Za-z0-9_]+$ ]] || { echo 'build_probe: probe must be a literal basename' >&2; exit 2; }

# asset_browser is an SDL viewer, not a headless probe, and it owns its own
# build. Refuse it here rather than emit a confusing missing-SDL.h error.
if [[ "$probe" == "asset_browser" ]]; then
  echo "build_probe: asset_browser is an SDL tool, not a headless probe" >&2
  exit 2
fi

src="$ROOT/tools/$probe.c"
[[ -f "$src" ]] || { echo "build_probe: no such probe: $src" >&2; exit 2; }

files=("$src")
for u in $(srcs_for "$probe"); do
  [[ -f "$ROOT/src/engine/$u.c" ]] || { echo "build_probe: missing src/engine/$u.c" >&2; exit 2; }
  files+=("$ROOT/src/engine/$u.c")
done
files+=("$ROOT/src/engine/lzodec.c")

mkdir -p "$OUT"
"${CC:-cc}" -O2 -ffp-contract=off -o "$OUT/$probe" "${files[@]}" \
   -I"$ROOT/src" "$@" -lm \
  || { echo "build_probe: $probe FAILED" >&2; exit 1; }
echo "$OUT/$probe"
