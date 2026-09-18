#!/usr/bin/env bash
# Asset-free public checks; intentionally not campaign acceptance.
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONDONTWRITEBYTECODE=1
python3 tools/check_source.py --working-tree
python3 tools/test_source_inventory.py
python3 tools/test_public_verify.py
python3 tools/tests/test_portable_ghidra.py
node web/tests/external_paths_test.mjs
node web/tests/assets_test.mjs
node web/tests/music_test.mjs
node web/tests/shell_art_test.mjs
node web/p19_engagement_probe.mjs
node web/tests/public_surface.mjs
node web/tests/wasm_smoke.mjs
node web/tests/replay_validation_test.mjs
