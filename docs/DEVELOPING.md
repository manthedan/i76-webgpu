# Developing the browser port

> This repository is an unfinished source-only developer preview. Build and test results are bounded engineering evidence, not claims of legal clearance, complete campaigns, original-game parity, or support for every browser and asset revision.

Read `README.md`, `CONTRIBUTING.md`, `PUBLISHING.md`, and `docs/BUILDING.md` before preparing a change or an export.

## Supported development path

The browser build is the product path. Shared platform/render interfaces remain, but the unsupported legacy SDL/Vulkan debug application is deliberately omitted. Native headless probes exercise port components, not a native desktop game.

Reference requirements:

- Linux;
- Bash and Python 3;
- Emscripten SDK 6.0.5;
- Node 22.12 or newer for JavaScript checks (the Emscripten SDK supplies a suitable Node);
- a local HTTP server for browser use.

No game files are needed to compile or run the asset-free checks.

## Build

With the SDK installed under `$HOME/emsdk`:

```sh
export EMSDK="$HOME/emsdk"
"$EMSDK/emsdk" install 6.0.5
"$EMSDK/emsdk" activate 6.0.5
source "$EMSDK/emsdk_env.sh"
./web/build.sh
```

Outputs are local, generated files:

- `web/dist/i76web.mjs`;
- `web/dist/i76web.wasm`;
- `web/dist/stamp.json`.

`web/build.sh` compiles an explicit source list with `-ffp-contract=off`, memory growth enabled, and both web and Node environments. Keep the explicit source list in sync when adding a module. Do not add `-ffast-math`, associative/unsafe-math options, target-specific contraction, or other flags that can make native and Wasm arithmetic disagree.

The stamp is the current short Git commit when available and `unknown` in a source archive without Git metadata. It identifies a build; it does not imply byte-reproducibility across machines or toolchain revisions.

A pinned container recipe is documented in `docs/BUILDING.md`. Use it when comparing results across development machines.

## Run locally

Serve the repository root, not `web/` alone:

```sh
python3 -m http.server 8076 --bind 127.0.0.1
```

Open `http://localhost:8076/web/`. Do not use `file://`; module imports, Wasm loading, and links assume an HTTP origin.

Use only your own purchased installation. Keep it outside the source checkout. Open or drop the extracted game `app/` folder rather than an installer executable. The page stages recognized files locally into MEMFS and may retain them in browser IndexedDB. Clear staged files/site data before changing to a different asset set.

A successful archive open is not a completeness check. Missing missions, fonts, shell records, music, or movies may surface later.

## Public checks

After building, run:

```sh
./tools/check.sh
```

The public check script performs the source-inventory check and a bounded set of asset-free Node/Wasm checks. Its current scope includes synthetic asset routing, music and shell-art behavior, engagement-policy units, and Wasm initialization/failure behavior without game archives.

For compression changes, also run:

```sh
CC=clang python3 tools/test_lzodec.py
```

On Ubuntu 24.04 this requires `clang`, `libclang-rt-dev`, and `liblzo2-2`. The system LZO library is a test reference; it is not linked into the browser engine.

If you own Nitro Pack data, selected optional component checks can run against an explicit external path:

```sh
NITRO_APP=/outside-this-checkout/nitro/app/ ./tools/check_owned.sh
```

The trailing slash is required. The script rejects an asset root inside the checkout.

These checks are not equivalent:

| Evidence | What it can establish | What it cannot establish |
|---|---|---|
| Strict source-export inventory | Export contains exactly the listed UTF-8 source files, without dependency/build outputs; `--working-tree` is a separate, weaker development mode | Absence of every secret, legal clearance, provenance of every value |
| Synthetic unit test | A bounded parser/policy branch behaves on generated inputs | Behavior on all purchaser files or all malformed inputs |
| Wasm smoke | The produced module initializes or rejects a selected state as expected | Playability, renderer fidelity, mission completion |
| Owned-file component test | One engine path works with one purchaser revision | Browser/DOM behavior, human experience, campaign coverage |
| Browser journey | One scripted page path worked in one browser configuration | Human acceptance, native parity, reliability across missions/platforms |
| Original-executable comparison | A bounded behavior or output differed/matched under a recorded setup | Unmeasured states, causal certainty, general parity, permission to redistribute inputs or captures |
| Human play report | What a person observed on a named build/setup | Unobserved missions, exact cause, legal/provenance conclusions |

[TESTING.md](TESTING.md) documents the portable subset: native/generated checks, optional renderer comparisons and tape replay. Historical private full-suite receipts, VM/oracle infrastructure and purchaser evidence are not included. Do not cite those private results as public CI successes. Maintainers must qualify release-sensitive renderer and mission changes before publication.

## Change map

### Browser intake or storage

Relevant files: `web/assets.js`, `web/save.js`, and `web/index.html`.

- Keep asset routing pure and unit-testable.
- Preserve relative mission directories; a bare mission file has no reliable destination.
- Do not upload or fetch game assets.
- Treat IndexedDB failure as recoverable and leave the page usable.
- Do not overwrite a record from a newer schema.
- Test duplicate routes, case variation, persistence failure, and profile switching.

### Wasm boundary

Relevant file: `web/webmain.c`.

- Keep `web_*` exports small and explicit.
- Document whether returned memory is static, cache-owned, or valid only until the next call/reload.
- JavaScript should copy a returned heap region if it must survive memory growth or another export.
- String arguments must pass through `ccall`/`cwrap` with a `string` argument type unless a numeric pointer is intentionally supplied.
- Add new exported symbols to the Emscripten build contract when direct export retention is required.
- Do not put browser-only policy into low-level format readers without a clear ownership reason.

### Simulation, input, AI, combat, or mission logic

Relevant files: `src/engine/car.c`, `input.c`, `ai.c`, `combat.c`, `mission.c`, and `fsm.c`.

- Preserve the 20 Hz fixed-step contract.
- Keep simulation independent of requestAnimationFrame cadence, audio clocks, and rendering interpolation.
- Maintain deterministic iteration order; avoid pointer-order or unstable-sort decisions.
- Preserve `-ffp-contract=off` on every comparable build path.
- Distinguish physical state, mission/script state, and presentation state.
- Record any fallback or invented rule as a port decision. Do not label it as original behavior.
- Never weaken a mission outcome, tape comparator, fixture, timeout, or assertion solely to obtain green automation.
- A headless call to `web_drive_step()` is component evidence, not a production-page journey.

### Mission lifecycle

Mission reload code crosses many ownership boundaries. Before editing it, read `web_mission_load()`, `web_drive_load()`, `shell_teardown()`, `mission_attach()`/`mission_unload()`, and the unload functions of every affected subsystem.

Required invariants include:

- detach collider tables before scene storage is freed;
- reset GPU caches before their engine-owned mesh/texture sources disappear;
- complete terrain teardown/load before scene-owned drivable faces register;
- release every cache-acquired mesh exactly once;
- reset mission/FSM/HUD/paper/audio state on same-page re-entry, not only on full page reload;
- make failed loads leave no borrowed pointer to partial state.

Test at least one same-page mission transition when changing lifecycle code. Reloading the browser between every test can hide stale-state defects.

### Formats and decompression

Relevant files are described in `docs/FORMATS.md`.

- Use generated fixtures; never commit purchaser bytes.
- Read unaligned values with byte helpers or `memcpy`, not packed-struct casts.
- Validate parent bounds, count caps, and overflow before allocation.
- Parse first, then publish state. A rejected input must not half-install a world, cache entry, or live save.
- Preserve opaque fields and unknown tags.
- Run sanitizers for C parser changes where practical.
- Decoder success on the known corpus does not prove arbitrary-input safety.

### Software rendering

Relevant files: `src/engine/raster.c`, `terrain.c`, `scene.c`, `worldrender.c`, texture decoders, and HUD/paper modules.

- The indexed software framebuffer is the reference output in this repository.
- Preserve palette-index semantics until final display conversion.
- Keep camera, depth/painter ownership, culling, material, and overlay changes separately testable.
- Frame hashes are sensitive: update an expected frame only after explaining the changed pixels, not merely because a new hash is stable.
- Avoid drive-by changes to numerical expressions; a harmless-looking reorder can change raster results.

### WebGPU rendering

Relevant files: `web/gpu_scene.mjs`, GPU exports in `web/webmain.c`, and shared scene/terrain code.

- Treat the C export stream as an experimental bridge, not a fully renderer-neutral API.
- Keep mission-resource, camera/LOD, and per-frame invalidation separate.
- Copy exported heap data before the owner can reload or Wasm memory can grow.
- Test device loss and errors; never leave a dead renderer presenting a stale frame.
- Verify software and GPU paths independently for changes to terrain, roads, scene geometry, camera, textures, HUD, or effects.
- Do not use adaptive GPU terrain as a physics/navigation oracle.

A WebGPU device existing is not renderer qualification. Adapter, browser, driver, output comparison, and affected scenes all matter. The release owner controls acceptance and any golden repin.

### Audio and movies

Relevant files: `src/engine/sound.c`, `web/audio.js`, `web/movie.c`, and `third_party/libsmacker/`.

- C playback state must remain deterministic and platform-independent.
- WebAudio is a reconciliation/presentation layer and may start only after a user gesture.
- Pause/resume must preserve source offsets without altering simulation.
- Mission boundaries must not leak old one-shots, loops, or decoded clip ownership.
- Keep authored local bytes in MEMFS; do not add network media fallback.
- Changes to libsmacker must preserve its notices and use the relink procedure in `docs/BUILDING.md`.

## Adding tests

Prefer the narrowest public test that can fail for the defect:

1. pure JavaScript unit for routing or browser policy;
2. generated C/Node fixture for a parser or engine rule;
3. asset-free Wasm smoke for the C/JS boundary;
4. optional external-asset component check;
5. real browser journey when DOM, event timing, Canvas, storage, audio, or GPU behavior is part of the contract.

A good regression records:

- the bug mechanism, not only an output snapshot;
- generated fixture provenance;
- exact toolchain/browser version where relevant;
- the expected failure before the fix;
- the narrow claim established after the fix;
- known branches and environments not exercised.

For deterministic tests, include a negative/control arm where practical. A comparator that also accepts a deliberately mutated tape, frame, or decode is broken and must not be counted as a pass.

Do not make purchaser assets a public CI requirement. Do not attach them, extracted derivatives, or purchaser screenshots to issues or pull requests.

## Extending public APIs

Most engine headers are internal contracts even when C exposes them broadly. Before adding a new shared interface:

- identify its owner and lifetime;
- specify units, coordinate system, cadence, and failure values;
- decide whether it is simulation authority, presentation only, or diagnostics only;
- keep diagnostics read-only unless a test-only mutation seam is explicitly isolated;
- update all consumers and both renderers when relevant;
- avoid publishing internal addresses or executable-derived artifacts;
- add documentation based on retained source behavior, not on unavailable private notes.

The separately prepared portable research toolkit is described in [`tools/ghidra/README.md`](../tools/ghidra/README.md). It is not part of the normal build/test path, and this developer guide does not claim that its external projects, binaries, or services are present.

## Before submitting a change

- [ ] No game archive, executable, mission, extracted asset, media, screenshot, credential, private host, or raw reverse-engineering dump is included.
- [ ] Source/dependency provenance is stated.
- [ ] The diff is bounded; unrelated cleanup is omitted.
- [ ] Generated fixtures and assertions were not weakened to make automation pass.
- [ ] `./web/build.sh` and the relevant public checks were run, with actual outcomes reported.
- [ ] Decoder changes received sanitizer coverage where applicable.
- [ ] Same-page teardown/re-entry was exercised for lifecycle changes.
- [ ] Both renderer paths were considered for shared world/render changes.
- [ ] Known failures and untested environments are listed.
- [ ] `PUBLIC_FILES.txt` includes deliberate new public files and excludes generated outputs.
- [ ] Documentation does not overstate campaign status, browser support, parity, provenance, or legal conclusions.

Passing this checklist makes a change reviewable. It does not authorize publication or deployment.
