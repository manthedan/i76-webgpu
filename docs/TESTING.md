# Portable verification and developer tools

These tools are included in the source tree. Game data, generated WebAssembly,
recordings, screenshots and browser binaries are not included.

All asset-backed commands require a purchaser-owned app directory explicitly.
Keep binaries, logs, screenshots, traces and replay artifacts outside the
checkout. The verification entry point and browser/replay tools require fresh
outputs; the underlying native build helpers can rebuild existing binaries.

## Prerequisites

- Linux with Bash, Python 3, a C compiler, and Node.js 22.12+ (other hosts are unqualified)
- the candidate WebAssembly build at `web/dist/i76web.{mjs,wasm}`; build it
  with the candidate's documented `web/build.sh` flow
- for opt-in browser checks: Puppeteer and a compatible Chromium installation
- for asset-backed checks: `NITRO_APP` pointing to the user's extracted app
  directory containing `nitro.zfs`, `nitro.zix`, `miss8/`, and related files

`package.json` and the lockfile pin `puppeteer@25.5.0`. Run `npm ci` for browser
checks; this may download Puppeteer's managed browser. Alternatively use
`npm ci --ignore-scripts` with an explicit `CHROME` path. The exact canonical
SwiftShader hashes are pinned to Chrome **151.0.7922.71**; a newer managed
browser is not permission to update those hashes. The probes do not require
`pngjs`. Asset-free checks do not require npm dependencies.

## One public entry point

Choose a **new** output path for each invocation:

```sh
# Asset-free C unit contracts only; NITRO_APP is not needed.
I76_VERIFY_OUT=/absolute/path/outside/the/checkout/i76-units-001 \
  tools/public_verify.sh --units

export NITRO_APP=/absolute/path/to/your/extracted/nitro/app

# Portable equivalent of private gate 9. Requires the Wasm build and assets.
I76_VERIFY_OUT=/absolute/path/outside/the/checkout/i76-gate9-001 \
  tools/public_verify.sh --gate9

# Opt-in BYO-asset browser equivalents of private gates 32 and 33.
# This launches Chromium. It is intentionally not an asset-free CI command.
I76_VERIFY_OUT=/absolute/path/outside/the/checkout/i76-browser-001 \
  tools/public_verify.sh --browser

# Units + gate 9 + browser gates.
I76_VERIFY_OUT=/absolute/path/outside/the/checkout/i76-all-001 \
  tools/public_verify.sh --all
```

The script refuses an existing output directory, an output inside the checkout,
or an asset root inside the checkout. It starts its static page server on an
ephemeral loopback port. Puppeteer intercepts only the synthetic
`/__i76_assets/` URLs and serves them from the explicit `NITRO_APP`; no game
asset is copied under `web/` or sent to a public host.

Browser result rules are deliberate:

- no WebGPU adapter/device: **SKIP**, overall exit 3;
- an acquired adapter that produces a mismatch, invalid frame, page error, or
  failed assertion: **FAIL**, exit 1;
- static Node probes are diagnostics/contracts, not an actual-GPU result.

`CHROME=/absolute/path/to/chrome` may select a browser. Otherwise Puppeteer's
managed browser is used. `I76_BUILD_ID=<source-id>` may label jitter evidence;
when absent it is recorded as `unversioned-source` and is not treated as a Git
claim.

## What each command runs

### `--units`

- `raster_test`: asset-free software-raster kernel goldens
- `pixel_history_probe`: ordered pixel-history and winner diagnostics
- `ai_race_entry_probe`: synthetic race-entry regression

The race probe is a component test of the current port source. It is not a
mission journey and not original-executable evidence.

### `--gate9`

This retains the gate-9 assertions, baselines, and pass/fail distinctions:

1. `frame_gate.sh`: native/Wasm equality for wire and filled renderers across
   N01, N02, and P01 at two fixed cameras; nonempty geometry and authored
   shade/translucency tables remain mandatory.
2. `road_depth_probe`: pinned P02 road ownership and nearer-relief rejection.
3. `cockpit_probe`: purchaser-backed cockpit, instruments, mirror, weapon, and
   combat-presentation assertions. PPM diagnostics go only to the external
   output directory.
4. `sky_probe`: authored sky/skyline, determinism, fallback, and depth
   isolation. Its synthetic mission sandbox and optional PPMs are external.
5. `gpu_mirror_probe` and `gpu_road_policy_probe`: static/recording WebGPU
   contracts. These do **not** acquire an adapter.
6. `probe_gila_flash`: Wasm software-render temporal ownership check.

The candidate WebAssembly build is a prerequisite rather than an implicit
side effect. This avoids rebuilding or changing the candidate unexpectedly.

### `--browser` (gates 32 and 33)

Gate 32 runs `full_frame_parity.mjs` against `gpu_scene_probe.html`, then runs
`cockpit_jitter_probe.mjs` only after the full-frame adapter path passes. The
software frame remains the authority. All seven original cases, exact canonical
SwiftShader hashes, the 32-channel/1%-outlier rule, fixed screen bands, exact
upper-HUD requirement, active-FX requirement, and view-cycle checks are
unchanged.

Gate 33 runs `road_traversal_gate.mjs` against
`road_traversal_probe.html`. Both P01 and P02 physical-drive cases, six samples,
20 one-tick far-field pairs, coverage floors, 0.25% software and 0.50% WebGPU
mid-band ownership limits, and 5% road-band drift alarm are unchanged.

`web/gpu_scene.mjs` is the candidate runtime renderer/helper module. It does
not fetch assets and has no test `run()` or Node tail. The browser-only test
orchestration is in `web/tests/gpu_scene_runner.mjs`; the page imports that
module and requires explicit `?app=` data routing.

### Normal-page WebGPU default and software fallback

```sh
NITRO_APP=/absolute/owned/app CHROME=/absolute/path/to/chrome \
  node web/tests/renderer_default_browser.mjs /absolute/external/new-renderer-smoke
```

This separate opt-in check enters stock P01 through ordinary file intake and
shell controls, without `dev=1` or engine-state injection. Fresh browser contexts
cover bare-page WebGPU, explicit `?renderer=sw`, a saved software preference,
a deliberately missing `navigator.gpu`, and a rejected GPU submission-completion
promise. Ordinary Space skips the opening sequence before checking driving frames. It checks nonempty rendered canvases, native-resolution GPU at
DPR 2, software fallback, visible renderer labels and page errors. It records
browser mode, flags, adapter information and compositor capabilities. The test
stages archives, campaign and mission files, not a complete installation; it is
not a HUD/media-completeness or mission acceptance test. Screenshots and logs
stay in the fresh external output directory. No adapter is SKIP/3; failure after
an acquired adapter is FAIL/1. SwiftShader does not prove consumer GPU support.

The default mode is headless. An offscreen WebGPU PASS does **not** establish
canvas presentation: some Linux headless configurations acquire a device but
cannot create the browser's shared presentation image. Those failures stay FAIL,
not SKIP. For a headed Vulkan presentation check, use `I76_BROWSER_MODE=vulkan-headed`
with a display (or `xvfb-run -a node ...`). This development preset enables Vulkan
and bypasses Chromium's GPU blocklist; it is not stock-browser qualification.
If multiple Vulkan drivers are installed, an explicit `VK_ICD_FILENAMES` may
select the intended system ICD. Record that environment with the run. One tested
configuration is Chrome 151 / NVIDIA RTX 3090 / driver 580.173.02 with a selected
NVIDIA ICD and Xvfb; it is not a consumer hardware/browser support matrix.

`tools/check.sh` also runs the asset-free `renderer_choice_test.mjs`: fresh GPU
preferences, saved software choices, URL precedence, invalid values and storage
failure. Its static page checks are not browser qualification.

## Individual developer commands

The public entry point is preferred because it enforces fresh external output.
The underlying commands remain available for focused diagnosis:

```sh
# Build one native probe; --list prints each curated source set.
OUT=/absolute/external/bin tools/build_probe.sh raster_test
OUT=/absolute/external/bin tools/build_probe.sh --list

# Native/Wasm frame parity only (requires NITRO_APP and web/dist).
NITRO_APP=/absolute/owned/app \
  tools/frame_gate.sh /absolute/external/frame-scratch

# Asset-free native units after building them.
/absolute/external/bin/raster_test
/absolute/external/bin/pixel_history_probe
/absolute/external/bin/ai_race_entry_probe

# Asset-backed native probe examples.
NITRO_APP=/absolute/owned/app /absolute/external/bin/frame_probe miss8/P01.MSN --filled
/absolute/external/bin/road_depth_probe /absolute/owned/app
/absolute/external/bin/cockpit_probe /absolute/owned/app /absolute/external/cockpit-captures
NITRO_APP=/absolute/owned/app I76_VERIFY_TMP=/absolute/external/tmp \
  /absolute/external/bin/sky_probe --ppm /absolute/external/sky/sky

# Wasm/static contract probes (requires web/dist; asset-backed commands require NITRO_APP).
cd web
NITRO_APP=/absolute/owned/app node frame_probe.mjs miss8/P01.MSN --filled
NITRO_APP=/absolute/owned/app node gpu_mirror_probe.mjs
node gpu_road_policy_probe.mjs
NITRO_APP=/absolute/owned/app node probe_gila_flash.mjs
```

`full_frame_parity.mjs`, `cockpit_jitter_probe.mjs`, and
`road_traversal_gate.mjs` are supported through `public_verify.sh --browser`;
direct use must still supply explicit page/output arguments and a new external
output. `serve_static.py`, `gpu_scene_probe.html`,
`road_traversal_probe.html`, and `web/tests/gpu_scene_runner.mjs` are harness
components, not independent pass claims. Opening a page without the driver's
explicit `?app=` route fails data setup. `gpu_scene.mjs` itself is runtime code,
not a Node GPU test.

## Standalone trial replay

Trial bundles and replay output must both be outside the checkout. Replay never
writes into the source bundle:

```sh
export I76_APP=/absolute/path/to/your/extracted/nitro/app
node web/trial_replay.mjs \
  /absolute/external/trials/my-recording \
  --output /absolute/external/replays/my-recording-candidate-001 \
  --frames --ai-metrics
```

Use `NITRO_APP` instead of `I76_APP` if desired. Optional flags are
`--ai-trace` and `--tolerance <metres>`.

The output directory must not already exist and cannot be nested inside the
source bundle. Marker PNGs, the final frame, and `replay-summary.json` are
written there. The original recording remains untouched.

**Exit 0 means the replay harness reached the end without a harness failure. It
does not mean zero pose divergence.** Read `divergences`, `firstDivergence`,
`maxPosDrift`, and recovery-event counts in `replay-summary.json`. Empty tapes,
tick desynchronization, non-finite state, unavailable same-build recovery, and
zero replayed ticks remain failures. Mid-drive recordings remain explicitly
`EVIDENCE-ONLY` because they lack the preceding state.

## Privacy and proof boundaries

- Keep `NITRO_APP`/`I76_APP`, trial bundles, logs, screenshots, PPMs, JSON
  traces, and browser profiles outside the checkout. Review captures before
  sharing; they can reveal purchaser assets and local paths.
- The C and Node component probes establish current-port contracts only.
- Gate 9 native/Wasm equality compares two builds of the **current port**. The
  “native” side means a host C build from current source, not Activision's
  original executable.
- Gates 32/33 compare the port's software and WebGPU renderers. They are not
  original-game pixel fidelity, broad device support, or human usability
  evidence. SwiftShader is a real WebGPU adapter when acquired, but it is not a
  consumer-hardware support matrix.
- Page automation establishes a reproducible page path, not a human verdict.
- Nothing here runs or validates the original executable, a VM, or Ghidra.
  Native-table provenance and publication approval remain separate holds.
