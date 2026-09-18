#!/usr/bin/env python3
"""Exact source-export inventory; --working-tree skips local build/dependency outputs.

Neither mode is a secret scan or copyright clearance. Audit the final git archive
with the default strict mode, not --working-tree. Git administration is never
part of the source inventory.
"""
import argparse
import os
from pathlib import Path, PurePosixPath
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--working-tree', action='store_true',
                    help='allow ignored node_modules, Python caches and the three Wasm build outputs; NOT an export audit')
args = parser.parse_args()
root = Path(__file__).resolve().parent.parent
listed = (root / 'PUBLIC_FILES.txt').read_text().splitlines()
generated = {'web/dist/i76web.mjs', 'web/dist/i76web.wasm', 'web/dist/stamp.json'}


def non_source(name):
    parts = PurePosixPath(name).parts
    return (name in generated or parts[0] in {'.git', 'node_modules'}
            or '__pycache__' in parts or name.endswith('.pyc'))


if listed != sorted(set(listed)):
    raise SystemExit('PUBLIC_FILES.txt must be sorted and unique')
for name in listed:
    p = PurePosixPath(name)
    if not name or not p.parts or p.is_absolute() or '..' in p.parts or str(p) != name:
        raise SystemExit(f'Invalid inventory path: {name!r}')
    if non_source(name):
        raise SystemExit(f'Generated/administrative file cannot enter source inventory: {name}')
expected = set(listed)
actual = set()
# Prune dependencies rather than walking thousands of unaudited package files.
for directory, dirs, files in os.walk(root, followlinks=False):
    for name in list(dirs) + files:
        p = Path(directory) / name
        rel = p.relative_to(root).as_posix()
        ignored = rel == '.git' or rel.startswith('.git/') or (args.working_tree and non_source(rel))
        if ignored:
            if name in dirs:
                dirs.remove(name)
            continue
        if p.is_symlink():
            raise SystemExit(f'Symlink is not part of the source export: {rel}')
        if p.is_dir():
            continue
        if not p.is_file():
            raise SystemExit(f'Non-regular source entry: {rel}')
        actual.add(rel)
if actual != expected:
    print('Missing:', sorted(expected - actual), file=sys.stderr)
    print('Unexpected:', sorted(actual - expected), file=sys.stderr)
    raise SystemExit(1)
for name in listed:
    raw = (root / name).read_bytes()
    if len(raw) > 2_000_000 or b'\x00' in raw:
        raise SystemExit(f'Unexpected binary/large source file: {name}')
    raw.decode('utf-8')
mode = 'Working-tree' if args.working_tree else 'Strict source-export'
print(f'{mode} inventory PASS: {len(listed)} UTF-8 source files.')
if args.working_tree:
    print('Local dependencies, Python caches and generated Wasm are NOT audited by this mode.')
