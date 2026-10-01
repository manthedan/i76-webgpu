# Publishing

This repository, `https://github.com/manthedan/i76-webgpu`, is the public source home of the browser port. It publishes **source only**. There are no Pages deployments, release binaries or hosted builds.

## Decoded numeric tables

Two functional lookup tables were decoded from the original executable for compatibility:

- `src/engine/ai.c`: `s_clsn_tbl[5][5][8][8]`, 1,600 byte values, attributed in the source to the original executable's `DAT_004c7138`.
- `src/engine/hud.c`: the 20-row scenario-to-cockpit class map.

These are behavior and mapping data, not game archives or media. On 2026-10-01 the project owner reviewed them and decided to keep them unchanged in the public source. That is a risk decision, not a legal opinion. If a rights holder raises a concern, re-derive or remove the affected data rather than arguing over it. See [Provenance](docs/PROVENANCE.md) for how each table is used.

## Rules for every published change

- Never commit game archives, executables, missions, extracted/converted media, purchaser screenshots or captures, decompiled listings, analysis databases, credentials or private paths/history.
- Preserve Roanish's [recorded grant](docs/legal/roanish-grant.md) and credit, the libsmacker LGPL notices, and [THIRD-PARTY.md](THIRD-PARTY.md).
- Keep `PUBLIC_FILES.txt` in sync and run `./tools/check.sh` before pushing.
- Keep support claims narrow: source publication is not campaign completion or cross-browser qualification.
- The separate private `manthedan/i76-web` development repository stays private. Never push its refs here.

## Binaries and hosting

Distributing a compiled build (Wasm, a hosted page or Pages) is a separate decision. It requires the LGPL obligations for libsmacker to be met, including matching source and rebuild/relink materials (see [BUILDING.md](docs/BUILDING.md)), plus the applicable toolchain runtime notices.
