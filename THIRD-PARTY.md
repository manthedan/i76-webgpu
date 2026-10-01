# Code provenance and third-party terms

## Project contributions and upstream

[LICENSE](LICENSE) applies the MIT licence to this project's own contributions. It does not relicense other authors' work.

Substantial archive/VFS/mesh/font and platform foundations descend from [Roanish/i76](https://github.com/Roanish/i76), also called “Vigalante '76”. Roanish did not select a standard licence, but explicitly permitted reuse in [issue #1](https://github.com/Roanish/i76/issues/1). That permission, not an inherited MIT licence, is the basis for the upstream portions. Preserve the [verbatim grant and its interpretation](docs/legal/roanish-grant.md), copyright notices and attribution. The author declined a formal licence; this is not a request still pending with them. An informal grant is not the same as legal clearance for every downstream use.

Thank you, Roanish, for making this work possible.

## Vendored and linked: libsmacker

- Path: `third_party/libsmacker/`.
- Upstream: [JonnyH/libsmacker](https://github.com/JonnyH/libsmacker), revision `ae8d4c9ec07b24d43ccff184d6e512bae793dfd1`.
- Authors: Greg Kennedy and contributors; preserve the notices in the source.
- Licence: **LGPL-2.1-or-later**, with the complete [licence text](third_party/libsmacker/COPYING) retained.
- The four imported library files are unmodified in this preview.

The browser build links this decoder into WebAssembly. It decodes movies/audio supplied locally by the purchaser; no movie data accompanies the source.

If you distribute a compiled build, retain prominent notices, the LGPL text and complete corresponding library source, including your changes. Provide the matching application source/build materials needed to rebuild and relink a modified decoder, as required by the applicable LGPL terms. A lone Wasm file or hosted page is not a substitute. See [rebuild/relink instructions](docs/BUILDING.md). This candidate distributes source only; it has no binary download or hosted source-offer URL.

## Compression

The engine uses its project-local LZO1X/1Y decoder, `src/engine/lzodec.c`. The unlinked LZO reference source tree from development is **not included**. No LZO library is linked by `web/build.sh`.

The optional generated-data test `tools/test_lzodec.py` loads the system `liblzo2.so.2` as an independent compression/decompression reference. LZO 2 is GPL-2.0-or-later; this system test dependency is not redistributed here or linked into the engine. Install it separately if running that test.

## Community references and credit

- **That Tony**, [Hacking on Space](https://hackingonspace.blogspot.com/): foundational binary-format research.
- **David Hopkinson (“Hopper”)**, *Hopper's Guide*: model and texture-mapping research.
- **[Open76](https://github.com/r1sc/Open76)**: format/behavior reference only, no code imported. Its GPL-3 code is not part of this source release.
- **[i76-everywhere](https://github.com/therealjkvalentine/i76-everywhere)** (MIT): compatibility/tooling and behavior reference.
- **Shane Peelar**, UCyborg's AiO patch, immi101's i76fix and **MechVM**: community interoperability research and prior art.
- **[netherite](https://github.com/Infatoshi/netherite)**: verification methodology only, no code imported.

Please report missing credit or provenance concerns. Do not submit copied implementations from incompatibly licensed or unlicensed projects.

## Game data and toolchain

Interstate '76/Nitro Pack and the purchaser's game data are not licensed by this project. No purchaser archives, game executables or converted media are included. Two retained executable-derived numeric tables are documented in [Provenance](docs/PROVENANCE.md). The original titles are used to describe compatibility; there is no affiliation with or endorsement by Activision or Microsoft.

Emscripten/LLVM, Node, Python and native compilers are separately installed build/test tools, not vendored dependencies. The toolchain also supplies runtime/system-library code when you build a binary; preserve its applicable notices when distributing that binary. The supported application build is the browser; the native diagnostic probes are not a separate native game frontend.

Puppeteer (Apache-2.0) is a pinned development dependency downloaded by npm, not vendored source. Its managed Chromium download has separate notices and licences. Ghidra and a compatible JDK are separately installed tools; neither is included. The project-authored Ghidra launchers/scripts are covered by the [tooling notice](NOTICE.md). Generated analysis output is not automatically covered by the tools' licence.
