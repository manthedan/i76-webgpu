# Architecture

> **Developer-preview boundary.** This is a source-only browser port that reads files supplied by a lawful owner of the game. It includes no game archives or media and makes no claim of complete campaign support, native parity, clean-room status, formal specification, or legal clearance.

This document describes the code that is present in this repository. It deliberately does not depend on private research notes, purchaser-file dumps, or decompiled listings.

## System at a glance

The product path is a browser shell around a C engine compiled to WebAssembly:

```text
purchaser-owned files
        |
        v
web/assets.js --stage--> Emscripten MEMFS (/data)
        |                         |
        |                         v
        |                  fs.c -> vfs.c -> zfs.c/lzodec.c
        |                                  |
        |                                  v
        |                mission / terrain / scene / vehicle / FSM / combat
        |                                  |
        +-------------------------- web/webmain.c
                                           |
                         +-----------------+------------------+
                         |                                    |
                  indexed software frame             exported scene stream
                         |                                    |
                  Canvas 2D presentation              web/gpu_scene.mjs
                                                           WebGPU
```

The software renderer is the reference implementation in this tree. The WebGPU renderer is an experimental second consumer of exported scene data; it is not interchangeable proof that both paths behave identically.

## Repository map

| Area | Responsibility |
|---|---|
| `web/index.html` | Browser shell, menus, input binding, fixed-step scheduling, Canvas presentation, and renderer selection. |
| `web/assets.js` | Local asset discovery, route classification, MEMFS staging, and optional IndexedDB persistence. |
| `web/webmain.c` | Browser/engine boundary: exported API, lifecycle coordination, input queue, mission driving, rendering, and diagnostics. |
| `web/audio.js` | WebAudio playback for engine-owned sound state plus local music and movie audio. |
| `web/movie.c` | Bounded bridge from VFS bytes to the vendored Smacker decoder. |
| `web/save.js` | Browser-owned campaign progress, garage selection, and preferences. |
| `web/trial.js` | Developer-only per-tick input/pose recording and evidence-bundle assembly. |
| `web/gpu_scene.mjs` | Experimental WebGPU renderer and conversion of C exports into GPU resources. |
| `src/engine/fs.c`, `vfs.c`, `zfs.c`, `lzodec.c` | Case-tolerant file access, archive indexing, entry lookup, decryption, and decompression. |
| `src/engine/geomesh.c`, `meshcache.c` | OEG geometry decode and reference-counted cache with idle-entry LRU eviction. |
| `src/engine/terrain.c`, `scene.c`, `worldrender.c`, `raster.c` | Mission world loading and indexed software rendering. |
| `src/engine/car.c`, `input.c`, `ai.c`, `combat.c` | Fixed-step vehicle, controls, AI, contacts, weapons, and damage. |
| `src/engine/mission.c`, `fsm.c` | Mission container parsing, scripted FSM hosting, and arena-family objective controllers. |
| `src/render/render.h`, `src/platform/platform.h` | Shared interfaces used by retained code; the unsupported legacy SDL/Vulkan application and `src/main.c` are omitted. |

## Asset and trust boundary

`web/assets.js` accepts a selected folder, a drag/drop tree, or legacy file inputs. Its `classify()` function maps only known inputs into `/data`, including archive pairs, mission directories, fonts, music, movies, and selected shell files. Unknown files are counted and ignored.

The page does not need to upload or fetch game assets. Staged bytes live in the Wasm process's MEMFS. With browser storage available, the intake layer may also retain a directory handle and staged bytes in IndexedDB so they can be restored later. Campaign progress and preferences use a separate IndexedDB database through `web/save.js`.

This is a convenience boundary, not a hostile-file sandbox. The readers perform many size, count, and range checks, but the test suite does not establish safety for every malformed archive or nested format. Treat purchaser files as trusted inputs and generated fuzz fixtures as untrusted test inputs.

## Startup and runtime lifecycle

### 1. Module creation

`web/index.html` reads `web/dist/stamp.json`, dynamically imports the matching `i76web.mjs`, and creates one Emscripten module. `web/build.sh` produces the loader, Wasm module, and stamp together. An archive built without Git metadata reports build ID `unknown`.

### 2. Asset staging

`Assets.attach()` creates `/data`. Intake writes the selected archive and loose files below that root. `Assets.ready()` becomes true once either a Nitro or base archive pair is staged.

The archive pair chooses the VFS profile. It does not prove that all optional loose mission, media, font, or shell files are present.

### 3. Engine initialization

`web_init()` in `web/webmain.c`:

1. sets the engine filesystem root to `/data`;
2. initializes the `.zix`-driven VFS;
3. initializes the geometry cache and input subsystem;
4. builds the indexed palette; and
5. enumerates geometry names and selected diagnostic catalogues.

Archive handles are opened lazily by `vfs.c`. Loose files may resolve even when they do not appear in the archive index, which is how files staged directly into MEMFS remain visible.

### 4. Shell state

The C boundary maintains menu, briefing, drive, debrief, and terminal-credit states. JavaScript owns DOM presentation and persistent preferences; C owns mission/simulation state. Mission catalogues are built lazily because additional loose files can be staged after `web_init()` returns.

### 5. Mission load

`web_drive_load()` enters through `web_mission_load()`. The load boundary first detaches old borrowed pointers and tears down old resources, then loads the new world in ownership order:

1. invalidate/reset WebGPU-side caches before their source objects disappear;
2. unload paper UI and old scene/terrain/car state;
3. load terrain;
4. load scene, allowing scene-owned drivable faces to register against the new terrain;
5. attach mission/FSM state;
6. load and place the selected vehicle;
7. rebuild static colliders and dynamic ownership;
8. initialize HUD, sound, combat, camera, and render state.

The order is contractual. Moving terrain teardown after scene registration can erase scene-owned drivable-surface state. Likewise, static collider storage must be detached before scene objects are freed.

Vehicle selection is host-owned: the browser garage choice takes precedence, with mission-derived and fallback choices behind it. This host policy should not be described as an exact clone of the original shell.

### 6. Fixed-step simulation and presentation

The browser frame callback accumulates wall-clock time, caps the backlog, and calls `web_drive_step()` in 0.05-second increments. Simulation therefore advances at 20 Hz even when rendering runs at another cadence.

A drive step drains the input queue into one snapshot, advances the vehicle and mission systems in their fixed order, publishes settled state to dependent systems, updates combat/AI/sound observations, and records the masks actually consumed by the engine. Browser audio and display timing do not feed back into simulation.

Rendering is separate. The normal page calls `web_drive_render_alpha_mode()` with the remaining accumulator fraction. Presentation may smooth or extrapolate between authoritative steps; direct `web_drive_render()` renders exact current fixed-step state for probes and frame comparisons.

The browser caps accumulated lag rather than attempting an unbounded catch-up. A heavily stalled tab can therefore lose wall-clock correspondence while retaining bounded simulation work.

### 7. Mission end and re-entry

Mission state changes to running, complete, or failed. The shell polls that state and transitions to debrief. Continuing to another mission performs a full teardown, including mission/FSM, HUD, sound, paper UI, vehicle, scene, terrain, and renderer caches. This is distinct from merely replacing scene files: a partial teardown risks stale entity tables, cached pointers, or mission-owned audio.

## Engine ownership and data flow

### VFS buffers

`vfs_read_file()` returns allocated bytes. Consumers release them with `vfs_free()`. A decoder that retains data must copy it or document that it takes ownership.

### Geometry cache

`geo_cache_acquire()` returns a shared decoded `GeoMesh` and increments its reference count. `geo_cache_release()` decrements it. A zero-reference entry remains cached on an LRU list and may be evicted to satisfy the explicit cache budget. Scene and car teardown must release every acquired mesh; callers must not free cache-owned meshes directly.

### Mission world

Terrain and scene own decoded world storage. Mission attach mode borrows the already loaded world. Static-collider and GPU exports therefore cannot outlive the owning scene or terrain generation. The browser resets dependent caches before unloading those owners.

### Exported Wasm memory

Many `web_*` accessors return pointers into static or engine-owned Wasm memory. JavaScript consumers should copy heap views when data must survive the next export, memory growth, cache reset, or mission reload. Do not retain a `HEAPU8`/`HEAPF32` view across operations that can grow Wasm memory.

## Rendering

### Indexed software path

The engine composes terrain, roads, static objects, vehicles, effects, HUD, and paper surfaces into a 640×480 8-bit index buffer. Palette lookup converts the final indices for Canvas display. Shared raster ownership and painter/depth conventions live in `raster.c`, `terrain.c`, `scene.c`, and `worldrender.c`.

This path is the authority for the current repository's frame comparisons. That authority is internal to the port; it is not proof of pixel parity with the original executable.

### Experimental WebGPU path

`web/gpu_scene.mjs` reads camera, terrain, road, mesh, material, texture, and overlay exports from C and builds its own GPU resources. Camera-dependent terrain/road meshes and mission-dependent resources have different invalidation lifetimes. Device loss or a frame exception falls back rather than leaving a stale frame presented indefinitely.

The path contains explicit approximation and policy choices, including adaptive terrain export and road-depth handling. A planner or test that needs the engine's full-resolution 5 m terrain samples must use the terrain query, not infer missing samples from the adaptive GPU mesh.

Any rendering change that touches terrain, roads, scene composition, camera matrices, material resolution, or overlays must be checked on both paths. A software-only pass does not qualify WebGPU, and a WebGPU availability check does not qualify rendering fidelity.

## Audio, movies, and persistence

- C owns which one-shots and engine loop are active. `web/audio.js` polls that state at a 50 ms cadence and reconciles WebAudio nodes.
- Browser autoplay rules require a user gesture before sound can start or resume.
- Music is read from locally staged files. Track-selection behavior contains explicit interim port policy and is not claimed as complete original behavior.
- `web/movie.c` reads a staged Smacker file through the VFS, opens it from memory, validates basic dimensions/timing, and exposes decoded frame/audio buffers. The vendored decoder's separate licence applies.
- Mission-boundary sound reset intentionally flushes decoded mission audio and reloads recurring samples lazily. Exact native cache retention is unresolved.
- `web/save.js` stores names, outcomes, garage selection, and preferences. It does **not** snapshot an active mission.
- The C `save.c` live-state snapshot is a deterministic developer format with partial mission state only; it is not the retail save format and is not wired as browser in-mission resume.

## Diagnostics and trial recording

Developer mode exposes additional state and controls. `web/trial.js` can record one row for each fixed simulation tick, using the held/pressed masks that `web_drive_step()` actually consumed, plus pose, page events, markers, and optional canvas video.

The tape is stronger evidence than the video for deterministic replay, but even an exact replay proves only the recorded build, asset set, initial state, and path. See [TESTING.md](TESTING.md) for the optional standalone replay and renderer checks included with the developer tools. Replay exit status and pose-drift results are separate contracts; release acceptance remains a maintainer decision, not an automatic consequence of a replay reaching its end.

## Deliberate limits and unresolved behavior

The retained code itself labels a number of rules as port decisions, inferred behavior, placeholders, or unresolved boundaries. Important examples include:

- exact original handling for several AI, collision, damage, camera, audio-mixing, arena, respawn, and shell policies;
- host-selected arena defaults and other fallback rules not carried by mission files;
- partial mission-action coverage and clean trapping of malformed/unsupported FSM operations;
- adaptive/decimated terrain in the GPU export versus the full 5 m terrain data used by native queries;
- approximate mesh-cache memory accounting based on source size;
- incomplete live-snapshot scope for FSM stacks, shared cells, timers, and radio state;
- browser and operating-system support beyond the specifically tested development setup;
- WebGPU adapter/driver behavior and equivalence to software output;
- broad campaign, human-play, malformed-input, and original-executable fidelity coverage.

When code comments distinguish decoded fields from `PORT DECISION`, `INFERRED`, `UNKNOWN`, or fallback behavior, preserve that distinction in changes and documentation. Do not turn a test pass into a broader historical or compatibility claim.
