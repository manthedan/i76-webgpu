#!/usr/bin/env bash
# build_site.sh — assemble the static site for the hosted browser build.
#
#   tools/build_site.sh <new-output-dir>
#
# Builds the Wasm from a CLEAN checkout and copies an explicit allowlist: the
# page's runtime closure, its licence/notice documents, and the complete
# source of the exact commit as a tarball (the LGPL "corresponding source",
# served from the same place as the binary). Nothing else is published: no
# probes, tests, tools or build scratch. No game data exists in the checkout,
# and the page fetches none; visitors load their own files locally.
#
# Requires Emscripten 6.0.5 (see docs/BUILDING.md). Deploy the resulting
# directory as-is, e.g. `netlify deploy --dir <new-output-dir> --prod`.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd -P)"
OUT="${1:?usage: tools/build_site.sh <new-output-dir>}"
case "$OUT" in /*) ;; *) OUT="$PWD/$OUT" ;; esac
case "$OUT/" in "$ROOT/"*) echo "build_site: output must be outside $ROOT" >&2; exit 2 ;; esac
[[ ! -e "$OUT" ]] || { echo "build_site: $OUT already exists" >&2; exit 2; }

cd "$ROOT"
# Only ever run from a standalone public checkout. Anywhere else (for example
# the private repository's export overlay) `git archive HEAD` would package
# the whole enclosing repository into the published source tarball.
if [[ "$(git rev-parse --show-toplevel)" != "$ROOT" ]]; then
  echo "build_site: $ROOT is not the top of its git checkout; use a public checkout" >&2; exit 2
fi
# A hosted build must correspond exactly to a published commit.
if [[ -n "$(git status --porcelain --untracked-files=normal)" ]]; then
  echo "build_site: checkout has uncommitted or untracked changes" >&2; exit 2
fi
# The commit's tree must be exactly the public inventory: the source tarball
# below publishes the whole commit, so nothing else may be in it.
if ! diff -u <(git show HEAD:PUBLIC_FILES.txt) <(git ls-tree -r --name-only HEAD | LC_ALL=C sort) >&2; then
  echo "build_site: commit tree differs from PUBLIC_FILES.txt; refusing to publish it" >&2; exit 1
fi
python3 tools/check_source.py --working-tree
COMMIT="$(git rev-parse HEAD)"
SHORT="$(git rev-parse --short HEAD)"

./web/build.sh
[[ "$(python3 -c 'import json; print(json.load(open("web/dist/stamp.json"))["stamp"])')" == "$SHORT" ]] \
  || { echo "build_site: Wasm stamp does not match HEAD $SHORT" >&2; exit 1; }

mkdir -p "$OUT/web/dist" "$OUT/source"
# index.html's whole static closure: stylesheets, classic scripts, module
# imports, the GPU renderer's dynamic import and the engine build.
for f in index.html shell.css authentic-shell.css audio.js trial.js assets.js \
         save.js shell-art.js gpu_scene.mjs; do
  cp "web/$f" "$OUT/web/$f"
done
cp web/dist/i76web.mjs web/dist/i76web.wasm web/dist/stamp.json "$OUT/web/dist/"

# Notices the page links to (paths relative to the repository root).
for f in README.md LICENSE NOTICE.md THIRD-PARTY.md PUBLISHING.md \
         docs/BUILDING.md docs/PROVENANCE.md docs/legal/roanish-grant.md \
         third_party/libsmacker/COPYING; do
  mkdir -p "$OUT/$(dirname "$f")"
  cp "$f" "$OUT/$f"
done

# Complete corresponding source for this exact build.
git archive --format=tar --prefix="i76-webgpu-$SHORT/" HEAD | gzip -n -9 \
  > "$OUT/source/i76-webgpu-source.tar.gz"
printf '%s\n' "$COMMIT" > "$OUT/source/COMMIT"
cat > "$OUT/source/README.txt" <<EOF
Complete source for the build served by this site.

Commit:     $COMMIT
Archive:    i76-webgpu-source.tar.gz (git archive of that commit)
Repository: https://github.com/manthedan/i76-webgpu/tree/$COMMIT

Rebuild and relink instructions, including for the LGPL-2.1-or-later
libsmacker decoder: docs/BUILDING.md inside the archive.
EOF

cat > "$OUT/_redirects" <<'EOF'
/  /web/  302
EOF
cat > "$OUT/_headers" <<'EOF'
/*.md
  Content-Type: text/plain; charset=utf-8
/third_party/libsmacker/COPYING
  Content-Type: text/plain; charset=utf-8
/LICENSE
  Content-Type: text/plain; charset=utf-8
/source/*.txt
  Content-Type: text/plain; charset=utf-8
/source/COMMIT
  Content-Type: text/plain; charset=utf-8
/*.mjs
  Content-Type: text/javascript; charset=utf-8
/*
  X-Content-Type-Options: nosniff
EOF

echo "site for $SHORT assembled in $OUT ($(find "$OUT" -type f | wc -l) files)"
