# Public release: BLOCKED — private source staging only

This tree is approved only for reviewed source staging in the separate **private** repository `git@github.com:manthedan/i76-webgpu.git`. That destination was verified private and empty before the initial import. Keep it private: this is not approval to change visibility, expose source publicly, enable Pages, publish releases or distribute compiled artifacts. Public provenance questions remain unresolved.

## Unresolved provenance review

The code inspection identified numeric tables with explicit purchaser-executable provenance:

- `src/engine/ai.c`: `s_clsn_tbl[5][5][8][8]`, 1,600 byte values, attributed in the source to the original executable's `DAT_004c7138`.
- `src/engine/hud.c`: the 20-row scenario/class map described as dumped from the purchaser binary.

These are functional lookup/mapping data, not bundled game archives or media. That distinction does not by itself settle whether their distribution is permitted. Obtain a deliberate provenance/legal assessment, or replace them through an independently documented derivation with regression evidence. Do not remove or stub gameplay merely to make a publication checklist green. They remain unchanged in this **private source** candidate so preparation does not silently alter engine behavior.

See [Provenance](docs/PROVENANCE.md) for the data's actual use, the corrected table size, and scope limits. These are known examples, not an exhaustive inventory of all decoded constants.

A source inventory or secret-scan PASS is not copyright clearance. Roanish's recorded permission covers the upstream work; it is not a licence from the original game's rights holder for copied material. This is a risk flag, not a legal conclusion about copyrightability or infringement.

## Other publication conditions

- Repository identity: **i76-webgpu**, at `https://github.com/manthedan/i76-webgpu`. Use only the reviewed source-only history here. The separate private `manthedan/i76-web` development repository must remain untouched; never push its refs, change its visibility or assume that a clean branch erases its old history.
- Review the final source-file manifest, licences, notices and provenance. Keep all purchaser evidence and private development history excluded.
- Build and run the documented public checks from that exact export; review the source/CI/curation changes before committing or publishing.
- Complete applicable renderer/integration qualification for export-only harness changes. Asset-free CI and empty-page smoke checks do not replace those gates.
- Preserve known failures and narrow support statements; source publication is not campaign or consumer-release acceptance.
- Source-only approval does not enable Pages or authorize distributing binaries. Binary distribution additionally needs matching source/build/relink materials and applicable runtime notices.

Initialize staging history from the reviewed export only; private staging does not waive any public-release condition. Never push private development refs or recursively copy a private working directory. Any eventual source commit message must exclude private paths and session links.
