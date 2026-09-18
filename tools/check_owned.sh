#!/usr/bin/env bash
# Optional purchaser-asset component regressions, NOT browser acceptance.
set -euo pipefail
cd "$(dirname "$0")/.."
case "${NITRO_APP:-}" in
  */) ;;
  *) echo 'Set NITRO_APP to your Nitro app/ directory, with trailing slash.' >&2; exit 2;;
esac
app=$(realpath "$NITRO_APP")
case "$app/" in
  "$(pwd -P)/"*) echo 'Keep purchased files outside the source checkout.' >&2; exit 2;;
esac
node web/p05_sim.mjs both
node web/p19_sim.mjs both
node web/p19_recovery_probe.mjs
