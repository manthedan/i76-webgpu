# i76-web — developer preview

**Local development-source candidate; not approved for publication.** See the unresolved provenance and repository-transition review in [PUBLISHING.md](PUBLISHING.md).

An experimental browser engine compatible with purchaser-owned **Interstate '76 and Nitro Pack** data. The browser is the product path; the unsupported legacy SDL/Vulkan debug frontend is deliberately omitted. This is not a native game build.

**Unfinished, source-only preview.** Publishing this code does not mean that either campaign is complete. No game archives, executables, missions, textures, screenshots, audio or movies are included. There is no hosted demo or downloadable game binary in this source release.

## Status and limitations

- The engine implements archive/mesh/terrain readers, driving, combat, mission logic, a software renderer, local media decoding and a browser shell.
- Development has exercised selected missions with automated browser input. That is not human acceptance or campaign completion. P05/P06 browser qualification remains open; P06's experimental controls still fail all-five combat completion. Known arena and P01 active-WIN test failures remain unresolved.
- The build and asset-free tests below are the public regression baseline. They do not prove a playable mission, faithful original-game behavior or safety of every malformed input.
- The tested development browser configuration is desktop Chromium on Linux. Other desktop browsers/OSes are unverified; Firefox, Safari and mobile are not supported claims. WebGPU is experimental, not a claim of default hardware-accelerated portability.

## Build and run

Requirements: Linux, Bash, Python 3, and [Emscripten SDK](https://emscripten.org/docs/getting_started/downloads.html) **6.0.5**. The SDK supplies Node; Node 22.12 or newer is suitable for the JavaScript tests and optional browser-tool dependency. No game files are needed to build.

```sh
# Assuming emsdk is installed at $HOME/emsdk:
export EMSDK="$HOME/emsdk"
"$EMSDK/emsdk" install 6.0.5
"$EMSDK/emsdk" activate 6.0.5
source "$EMSDK/emsdk_env.sh"
./web/build.sh
./tools/check.sh

# Serve the repository root, NOT just web/, so legal links also resolve.
python3 -m http.server 8076 --bind 127.0.0.1
```

Open **http://localhost:8076/web/**. Do not open the HTML with `file://`.
Build outputs are local and ignored by Git. See [BUILDING.md](docs/BUILDING.md) for the pinned container recipe and decoder rebuild/relink instructions.

## Bring your own purchased files

Buy the game from [GOG](https://www.gog.com/game/interstate76). This project is not affiliated with or endorsed by Activision or Microsoft. A purchase does not grant permission to redistribute the game data.

Use your own installed files or GOG offline backup installers. Development used installer version **2.1.0.17**; other revisions are not guaranteed. Keep the base game and Nitro installations/extractions separate and **outside this checkout**.

On Windows, locate the installed Nitro folder containing `nitro.zix` and `nitro.zfs`. On Linux, install `innoextract` and extract the Nitro offline installer:

```sh
mkdir -p "$HOME/purchased-games/nitro"
innoextract -d "$HOME/purchased-games/nitro" /path/to/nitro-offline-installer.exe
```

Select the resulting **`app/` folder** using **Open game folder…**, or drag that whole folder onto the page. Supply the archive pair, `miss8/`, `addon/SCENARIO.DAT`, loose fonts/shell art, and the supplied `music/` and `smk/` directories where available. Do not select the installer executable or combine the base and Nitro folders.

Files are read locally; the page does not upload them or fetch game assets. Browser storage may retain staged files and progress. Use **Clear staged files** before switching asset sets; clearing browser site data removes stored data. The shell's completeness/conflict diagnostics are still being improved, so a successful archive load does not guarantee a complete installation.

The source supports early base-game work too, but the first playable-product target remains a **Nitro Pack preview**, not either complete campaign.

## Tests and contributing

- `./tools/check.sh`: source inventory, synthetic asset routing/music/shell-art tests, engagement-policy units, and asset-free Wasm initialization/failure smoke.
- `python3 tools/test_lzodec.py`: generated compression/decoder and sanitizer tests. On Ubuntu 24.04 install `clang libclang-rt-dev liblzo2-2` and run with `CC=clang`; the system LZO library is a test reference only, not linked into the engine.
- `NITRO_APP=/outside-checkout/nitro/app/ ./tools/check_owned.sh`: optional P05/P19 component regressions using your purchased files. The trailing slash is required. These are not browser or human acceptance tests.

Public CI never requires game data. It builds and runs asset-free checks; it does not deploy anything. Private research notebooks, deployment/VM infrastructure and purchaser evidence are intentionally absent. Portable reverse-engineering tools and curated developer documentation are included; generated decompilations and analysis databases are not. Source comments may retain historical research identifiers; they are not promises that those notebooks are part of this export.

Start with [Architecture](docs/ARCHITECTURE.md), [Formats](docs/FORMATS.md), [Developing](docs/DEVELOPING.md), [Testing](docs/TESTING.md), and the [Ghidra toolkit](tools/ghidra/README.md). Read [Provenance](docs/PROVENANCE.md) before treating decoded constants or a passing test as publication clearance.

Read [CONTRIBUTING.md](CONTRIBUTING.md) before submitting changes or files.

## Credit and licensing

This project began as a fork of **[Roanish/i76](https://github.com/Roanish/i76)**. Its archive, mesh, font and platform foundations owe a great deal to Roanish's work and explicit permission. Preserve that attribution and the [recorded upstream grant](docs/legal/roanish-grant.md).

Our own contributions are MIT. Upstream portions rest on their recorded permission, not a licence we can apply on someone else's behalf. The vendored Smacker decoder is **LGPL-2.1-or-later**, not MIT. See [LICENSE](LICENSE), [THIRD-PARTY.md](THIRD-PARTY.md), and the decoder's [COPYING](third_party/libsmacker/COPYING).
