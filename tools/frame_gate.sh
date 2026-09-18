#!/bin/bash
# frame_gate.sh — native<->wasm framebuffer equality gate.
#
# Renders the same missions from the same fixed cameras through the NATIVE
# build (tools/frame_probe.c) and the WASM build (web/frame_probe.mjs) and
# requires byte-identical index buffers and palettes.
#
# Why this exists, and why it exists BEFORE the filled rasterizer
# was introduced: the same C compiles to wasm and
# x86-64, and two independent reviews both put an empirical cross-target
# equality check ahead of any argument about float precision. The wasm spec
# permits nondeterministic NaN payloads, so a single 0/0 from a degenerate
# face ends bit-identity by specification rather than by luck. A rasterizer
# is exactly the kind of code that produces degenerate faces.
#
# It gates three things, not one:
#   1. the two builds agree byte for byte;
#   2. neither drew an empty frame (coverage floor) — a hash gate alone will
#      happily pin an all-background frame and call it a pass;
#   3. the two builds agree on coverage, which localises a mismatch to the
#      renderer rather than to asset loading.
#
# Usage: tools/frame_gate.sh <scratch-dir>
# Env:   NITRO_APP  explicit purchaser-owned app directory
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd -P)"
SCRATCH="${1:?usage: frame_gate.sh <scratch-dir>}"
: "${NITRO_APP:?set NITRO_APP to the purchaser-owned app directory}"
case "$SCRATCH" in
  /*) ;;
  *) echo "frame_gate: scratch-dir must be an absolute path" >&2; exit 2 ;;
esac
SCRATCH="$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve())' "$SCRATCH")" || exit 2
NITRO_APP="$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve(strict=True))' "$NITRO_APP")" || exit 2
export NITRO_APP
for p in "$SCRATCH" "$NITRO_APP"; do
  case "$p/" in
    "$ROOT/"*) echo "frame_gate: scratch and assets must be outside $ROOT" >&2; exit 2 ;;
  esac
done
[[ -e "$NITRO_APP/nitro.zfs" && -e "$NITRO_APP/nitro.zix" ]] || {
  echo "frame_gate: NITRO_APP lacks nitro.zfs/nitro.zix" >&2; exit 2;
}
mkdir -p "$SCRATCH" || exit 2
PROBE="$SCRATCH/frame_probe"

# Missions chosen for different terrain/scene shapes, not at random: N01 is
# the reference scene used elsewhere in the gates, the others bring
# different object counts and terrain extents.
MISSIONS=(miss8/N01.CBT miss8/N02.CBT miss8/P01.MSN)

# Two cameras per mission: one low and level, one high and pitched down, so
# a mismatch that only appears under one projection cannot hide.
CAMERAS=(
  "2997.5 60.0 48600.0 2997.5 30.0 48900.0"
  "3200.0 220.0 48400.0 3000.0 20.0 49000.0"
)

# Rebuild whenever ANY engine source or the probe is newer than the binary.
#
# This used to be `if [[ ! -x "$PROBE" ]]`, which cached the probe forever: edit
# raster.c, run the gate, and it would compare a STALE native binary against a
# freshly built wasm one. That is a gate that reports PASS for a build nobody
# made -- the exact failure class this gate exists to catch, sitting in the
# gate itself. (It also meant every real invocation needed a manual `rm`.)
#
# The source list itself lives in tools/build_probe.sh, not here. It used to be
# duplicated in this file, and the copy drifted: scene.c grew calls into
# texcache.c and the probes' own documented build lines stopped linking, while
# this gate kept passing because ITS copy happened to be the maintained one. One
# list that a link error updates beats two lists that agree until they do not.
#
# Staleness is decided over the whole engine tree rather than an enumerated set,
# for the same reason: a new translation unit must not be able to go unnoticed.
probe_stale() {
  [[ ! -x "$PROBE" ]] && return 0
  local s
  for s in "$ROOT"/tools/frame_probe.c "$ROOT"/tools/build_probe.sh \
           "$ROOT"/src/engine/*.c "$ROOT"/src/engine/*.h; do
    [[ "$s" -nt "$PROBE" ]] && return 0
  done
  return 1
}

if probe_stale; then
  echo "== building native frame_probe =="
  # build_probe.sh pins -ffp-contract=off, which is a DETERMINISM CONTRACT
  # rather than an optimisation choice: wasm has no f64 FMA, so a contracted
  # native build cannot match it. The web build uses the same flag.
  OUT="$SCRATCH" "$ROOT/tools/build_probe.sh" frame_probe >/dev/null \
    || { echo "frame_gate: native build FAILED"; exit 1; }
fi

if [[ ! -f "$ROOT/web/dist/i76web.mjs" ]]; then
  echo "frame_gate: web/dist/i76web.mjs missing — run web/build.sh first"
  exit 1
fi

# Pull just the FRAME line; both probes print engine chatter on stderr and
# other diagnostics on stdout.
frame_line() { grep -m1 '^FRAME ' <<<"$1"; }
field() { sed -n "s/.*$2=\([^ ]*\).*/\1/p" <<<"$1"; }
table_line() { grep -m1 '^TABLES lum=' <<<"$1"; }

# Both backends are gated. The wireframe is the legacy debug view and the
# filled path is the real renderer; each has its own float behaviour, so
# agreeing on one says nothing about the other.
BACKENDS=("" "--filled")

pass=0; fail=0
for bk in "${BACKENDS[@]}"; do
for m in "${MISSIONS[@]}"; do
  for cam in "${CAMERAS[@]}"; do
    nat_out="$("$PROBE" "$m" $cam $bk 2>/dev/null)"; nat_rc=$?
    wasm_out="$(cd "$ROOT/web" && node frame_probe.mjs "$m" $cam $bk 2>/dev/null)"; wasm_rc=$?
    nat="$(frame_line "$nat_out")"
    wasm="$(frame_line "$wasm_out")"

    tag="${bk:---wire} $m [$cam]"
    if [[ -z "$nat" || -z "$wasm" ]]; then
      echo "FAIL $tag — a probe produced no FRAME line (native rc=$nat_rc wasm rc=$wasm_rc)"
      fail=$((fail + 1)); continue
    fi
    # Coverage floor: rc!=0 from either probe means it drew an empty frame.
    if (( nat_rc != 0 || wasm_rc != 0 )); then
      echo "FAIL $tag — empty frame (native rc=$nat_rc wasm rc=$wasm_rc)"
      echo "  native: $nat"
      echo "  wasm  : $wasm"
      fail=$((fail + 1)); continue
    fi
    # Every pinned filled mission ships both WRLD-named native tables. Assert
    # the real 64 KiB assets loaded and were installed, rather than allowing
    # native/wasm parity to agree on the placeholder fallback.
    if [[ "$bk" == "--filled" ]]; then
      ntab="$(table_line "$nat_out")"
      nlum="$(field "$ntab" lum)"; ntbl="$(field "$ntab" tbl)"
      nlum_hash="$(field "$ntab" lum_fnv1a)"
      ntbl_hash="$(field "$ntab" tbl_fnv1a)"
      nshade="$(field "$ntab" shade_native)"
      ntransl="$(field "$ntab" transl_active)"
      if [[ -z "$ntab" || "$nlum" == "-" || "$ntbl" == "-" ||
            "$nlum_hash" == "0x00000000" || "$ntbl_hash" == "0x00000000" ||
            "$nshade" != 1 || "$ntransl" != 1 ]]; then
        echo "FAIL $tag — native mission shade/translucency tables inactive"
        echo "  native: ${ntab:-no TABLES line}"
        fail=$((fail + 1)); continue
      fi
    fi


    nfb=$(field "$nat" fb_fnv1a);  wfb=$(field "$wasm" fb_fnv1a)
    npal=$(field "$nat" pal_fnv1a); wpal=$(field "$wasm" pal_fnv1a)
    ncov=$(field "$nat" nonbg);     wcov=$(field "$wasm" nonbg)
    # Geometry coverage, not colour coverage: the filled backend's sky fills
    # every pixel, so nonbg alone can no longer detect a world that failed to
    # draw. Both builds must agree on this too.
    ngeo=$(field "$nat" geo);       wgeo=$(field "$wasm" geo)

    if [[ "$nfb" == "$wfb" && "$npal" == "$wpal" && "$ncov" == "$wcov" &&
          "$ngeo" == "$wgeo" ]]; then
      echo "ok   $tag fb=$nfb pal=$npal nonbg=$ncov geo=$ngeo"
      pass=$((pass + 1))
    else
      echo "FAIL $tag native/wasm differ"
      echo "  native: fb=$nfb pal=$npal nonbg=$ncov geo=$ngeo"
      echo "  wasm  : fb=$wfb pal=$wpal nonbg=$wcov geo=$wgeo"
      fail=$((fail + 1))
    fi
  done
done
done

echo "frame_gate: $pass matched, $fail failed"
[[ $fail -eq 0 && $pass -gt 0 ]] && { echo "frame_gate: PASS"; exit 0; }
echo "frame_gate: FAIL"; exit 1
