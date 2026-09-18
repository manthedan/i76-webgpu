#!/usr/bin/env bash
# Portable public verification entry point. Browser arms are always opt-in.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MODE="${1:-}"
: "${I76_VERIFY_OUT:?set I76_VERIFY_OUT to a new absolute directory outside the checkout}"

usage() {
  cat >&2 <<'EOF'
usage: I76_VERIFY_OUT=/external/new-dir tools/public_verify.sh --units
   or: NITRO_APP=/owned/app I76_VERIFY_OUT=/external/new-dir \
       tools/public_verify.sh --gate9|--browser|--all

--units    asset-free raster/pixel-history/race-entry checks
--gate9    native/Wasm frame parity plus cockpit/sky/road/GPU static contracts
--browser  opt-in BYO-asset equivalents of private gates 32 and 33
--all      units, gate 9, then opt-in browser gates
EOF
  exit 2
}
[[ "$MODE" =~ ^--(units|gate9|browser|all)$ ]] || usage
case "$I76_VERIFY_OUT" in /*) ;; *) usage ;; esac
command -v python3 >/dev/null || { echo "public_verify: python3 missing" >&2; exit 2; }
[[ ! -e "$I76_VERIFY_OUT" ]] || {
  echo "public_verify: refusing to overwrite $I76_VERIFY_OUT" >&2; exit 2;
}
OUT_REAL="$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve())' "$I76_VERIFY_OUT")" || exit 2
case "$OUT_REAL/" in "$ROOT/"*)
  echo "public_verify: I76_VERIFY_OUT must be outside $ROOT" >&2; exit 2;; esac
if [[ "$MODE" != "--units" ]]; then
  : "${NITRO_APP:?set NITRO_APP to an explicit purchaser-owned app directory}"
  APP_REAL="$(python3 -c 'import os,sys; print(os.path.realpath(sys.argv[1]))' "$NITRO_APP")" || exit 2
  case "$APP_REAL/" in "$ROOT/"*)
    echo "public_verify: NITRO_APP must be outside $ROOT" >&2; exit 2;; esac
  [[ -f "$APP_REAL/nitro.zfs" && -f "$APP_REAL/nitro.zix" ]] || {
    echo "public_verify: NITRO_APP lacks nitro.zfs/nitro.zix" >&2; exit 2;
  }
  export NITRO_APP="$APP_REAL"
fi
mkdir -p "$(dirname "$OUT_REAL")" || exit 2
# Atomic creation prevents two concurrent runs from sharing/overwriting evidence.
mkdir -m 700 "$OUT_REAL" || exit 2
mkdir "$OUT_REAL/logs" "$OUT_REAL/bin" "$OUT_REAL/tmp" || exit 2
export I76_VERIFY_TMP="$OUT_REAL/tmp"

run_log() {
  local name="$1"; shift
  echo "== $name =="
  "$@" 2>&1 | tee "$OUT_REAL/logs/$name.log"
}

run_units() {
  # Explicit propagation also works when this function is the left side of
  # --all's && list (where Bash would suppress errexit inside the function).
  run_log build-raster env OUT="$OUT_REAL/bin" "$ROOT/tools/build_probe.sh" raster_test || return $?
  run_log raster-test "$OUT_REAL/bin/raster_test" || return $?
  run_log build-pixel-history env OUT="$OUT_REAL/bin" "$ROOT/tools/build_probe.sh" pixel_history_probe || return $?
  run_log pixel-history "$OUT_REAL/bin/pixel_history_probe" || return $?
  run_log build-ai-race-entry env OUT="$OUT_REAL/bin" "$ROOT/tools/build_probe.sh" ai_race_entry_probe || return $?
  run_log ai-race-entry "$OUT_REAL/bin/ai_race_entry_probe" || return $?
}

require_web_build() {
  [[ -f "$ROOT/web/dist/i76web.mjs" && -f "$ROOT/web/dist/i76web.wasm" ]] || {
    echo "public_verify: web/dist missing; run web/build.sh first" >&2
    return 1
  }
}

run_gate9() {
  require_web_build || return 1
  mkdir -p "$OUT_REAL/gate9/cockpit" "$OUT_REAL/gate9/sky" || return $?
  run_log gate9-frame "$ROOT/tools/frame_gate.sh" "$OUT_REAL/gate9/frame" || return $?
  run_log build-road-depth env OUT="$OUT_REAL/bin" "$ROOT/tools/build_probe.sh" road_depth_probe || return $?
  run_log gate9-road-depth "$OUT_REAL/bin/road_depth_probe" "$NITRO_APP" || return $?
  run_log build-cockpit env OUT="$OUT_REAL/bin" "$ROOT/tools/build_probe.sh" cockpit_probe || return $?
  run_log gate9-cockpit "$OUT_REAL/bin/cockpit_probe" "$NITRO_APP" "$OUT_REAL/gate9/cockpit" || return $?
  run_log build-sky env OUT="$OUT_REAL/bin" "$ROOT/tools/build_probe.sh" sky_probe || return $?
  run_log gate9-sky "$OUT_REAL/bin/sky_probe" --ppm "$OUT_REAL/gate9/sky/sky" || return $?
  run_log gate9-gpu-mirror env NITRO_APP="$NITRO_APP" node "$ROOT/web/gpu_mirror_probe.mjs" || return $?
  run_log gate9-gpu-road-policy node "$ROOT/web/gpu_road_policy_probe.mjs" || return $?
  run_log gate9-gila-flash env NITRO_APP="$NITRO_APP" node "$ROOT/web/probe_gila_flash.mjs" || return $?
}

run_browser() {
  require_web_build || return 1
  local ready="$OUT_REAL/tmp/server-ready.json"
  python3 "$ROOT/tools/serve_static.py" "$ROOT/web" "$ready" \
    >"$OUT_REAL/logs/static-server.log" 2>&1 &
  local server_pid=$!
  stop_server() {
    kill "$server_pid" 2>/dev/null || true
    wait "$server_pid" 2>/dev/null || true
  }
  trap stop_server EXIT
  trap 'stop_server; exit 130' INT
  trap 'stop_server; exit 143' TERM
  for _ in $(seq 1 100); do [[ -s "$ready" ]] && break; sleep 0.05; done
  if [[ ! -s "$ready" ]]; then
    echo "public_verify: static server did not start" >&2
    stop_server; trap - EXIT INT TERM
    return 1
  fi
  local page_base
  page_base="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["url"])' "$ready")"

  local overall=0 rc=0
  mkdir -p "$OUT_REAL/gate32"
  run_log gate32-full-frame env NITRO_APP="$NITRO_APP" node \
    "$ROOT/web/full_frame_parity.mjs" \
    "$page_base/gpu_scene_probe.html" "$OUT_REAL/gate32/full-frame" || rc=$?
  if (( rc == 2 )); then
    echo "gate32: SKIP — no WebGPU adapter/device available" | tee -a "$OUT_REAL/logs/gate32-full-frame.log"
    overall=3
  elif (( rc != 0 )); then
    echo "gate32: FAIL — an acquired path or harness diverged" >&2
    overall=1
  else
    run_log gate32-cockpit-jitter env NITRO_APP="$NITRO_APP" GPURES=fidelity \
      I76_BUILD_ID="${I76_BUILD_ID:-unversioned-source}" node \
      "$ROOT/web/cockpit_jitter_probe.mjs" \
      "$OUT_REAL/gate32/cockpit-jitter.json" \
      "$OUT_REAL/gate32/cockpit-jitter.log" || overall=1
  fi

  rc=0
  run_log gate33-road-traversal env NITRO_APP="$NITRO_APP" node \
    "$ROOT/web/road_traversal_gate.mjs" \
    "$page_base/road_traversal_probe.html" "$OUT_REAL/gate33" || rc=$?
  if (( rc == 2 )); then
    echo "gate33: SKIP — no WebGPU adapter/device available" | tee -a "$OUT_REAL/logs/gate33-road-traversal.log"
    (( overall == 0 )) && overall=3
  elif (( rc != 0 )); then
    echo "gate33: FAIL — an acquired path or harness diverged" >&2
    overall=1
  fi
  stop_server
  trap - EXIT INT TERM
  return "$overall"
}

case "$MODE" in
  --units) run_units ;;
  --gate9) run_gate9 ;;
  --browser) run_browser ;;
  --all) run_units && run_gate9 && run_browser ;;
esac
