#!/usr/bin/env bash
# build.sh — emscripten build of the web viewer (M0 meshes, M2 missions,
# M3 input + drive mode). Produces dist/i76web.mjs + dist/i76web.wasm.
set -euo pipefail
cd "$(dirname "$0")"

source "${EMSDK:-$HOME/emsdk}/emsdk_env.sh" >/dev/null

mkdir -p dist

BUILD_STAMP="$(git rev-parse --short HEAD 2>/dev/null || echo unknown)"

emcc -O2 -ffp-contract=off \
  -DI76_BUILD_STAMP="\"$BUILD_STAMP\"" \
  -I../src -I../third_party/libsmacker \
  ../src/engine/fs.c \
  ../src/engine/zfs.c \
  ../src/engine/lzodec.c \
  ../src/engine/vfs.c \
  ../src/engine/geomesh.c \
  ../src/engine/meshcache.c \
  ../src/engine/meshview.c \
  ../src/engine/terrain.c \
  ../src/engine/scene.c \
  ../src/engine/raster.c \
  ../src/engine/vqm.c \
  ../src/engine/pixidx.c \
  ../src/engine/texcache.c \
  ../src/engine/camera.c \
  ../src/engine/worldrender.c \
  ../src/engine/component.c \
  ../src/engine/car.c \
  ../src/engine/input.c \
  ../src/engine/hud.c \
  ../src/engine/font.c \
  ../src/engine/paperui.c \
  ../src/engine/pcx.c \
  ../src/engine/mission.c \
  ../src/engine/m16.c \
  ../src/engine/paint.c \
  ../src/engine/sound.c \
  ../src/engine/fsm.c \
  ../third_party/libsmacker/smacker.c \
  movie.c \
  webmain.c \
  --no-entry \
  -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=I76Web \
  -sEXPORTED_RUNTIME_METHODS=ccall,cwrap,FS,HEAPU8,wasmMemory,UTF8ToString \
  -sEXPORTED_FUNCTIONS=_web_paper_set_escape,_web_set_log_level \
  -sALLOW_MEMORY_GROWTH=1 \
  -sENVIRONMENT=web,node \
  -o dist/i76web.mjs

# Cache-busting stamp consumed by index.html's dynamic module import; the
# old static ?v=20260813-buildid literal never changed between builds, so
# returning browsers could keep a stale i76web.mjs against a new wasm.
printf '{"stamp":"%s"}\n' "$BUILD_STAMP" > dist/stamp.json

echo "built dist/i76web.mjs + dist/i76web.wasm (stamp $BUILD_STAMP)"
