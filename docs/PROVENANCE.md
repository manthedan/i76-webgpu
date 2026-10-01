# Source and data provenance

This is an engineering inventory, **not legal advice, a clean-room claim, or clearance to publish**. The project reconstructs behavior and formats for interoperability with purchaser-owned files. Functional observations, algorithms, code expression, copied tables and game media must not be treated as interchangeable categories.

## Included source

- The project began as a fork of Roanish/i76. Preserve upstream attribution and the [recorded permission](legal/roanish-grant.md). Our MIT licence does not relicense upstream contributions. That informal grant is recorded; it must not be described as a standard MIT licence from Roanish.
- Project-authored implementation and tooling are covered by the scope described in [LICENSE](../LICENSE) and [THIRD-PARTY.md](../THIRD-PARTY.md). Source identifiers such as `FUN_...`/`DAT_...` document research origins, not a claim that decompiled source is included or that every interpretation is exact.
- Vendored libsmacker has its own LGPL-2.1-or-later source, copyright and licence. Building it locally is distinct from distributing linked binaries; see [BUILDING.md](BUILDING.md).
- Reusable Ghidra scripts are tooling, not the original executable or the scripts' output. Ghidra/JDK and purchased executables are separately obtained prerequisites. Generated programs, decompilations, memory/coverage dumps, logs and screenshots stay outside this repository.

## Known decoded-data questions

### AI action-selection table

`src/engine/ai.c` declares `s_clsn_tbl` as `unsigned char[5][5][8][8]`: **1,600 elements / 1,600 C bytes**, not 3,200. The earlier preparation report doubled the size incorrectly. Counting the explicit initializers and compiling a `sizeof` check agree; the source data were not changed.

The source attributes this table to `DAT_004c7138` and `FUN_00408ac0`. `dest68_plan()` indexes it by destination mode, range band and the two heading octants, then interprets the result as an action controlling desired heading and throttle. It is behavior-selection data, not artwork, audio, a mesh or a complete mission. No equivalent independent geometrical derivation has been established here: do not substitute a guessed rule merely to remove a literal table from an inventory.

### Scenario-to-cockpit mapping

`src/engine/hud.c::dash_class_for_mission()` contains 20 `(kind, number, class)` rows. Its comment explicitly attributes the mapping to a table dumped from the purchaser executable. The lookup selects the family of cockpit artwork loaded from the user's assets; the artwork itself is not included. Replacing the table with equivalent conditionals would change its representation, not its provenance.

These are **known examples, not an exhaustive copyright/provenance audit**. Nearby AI destination/weight arrays and other annotated constants also need to be considered in any assessment of decoded behavior data. Small size or functional purpose does not by itself settle the legal question, and the presence of a native address does not by itself establish infringement.

## Owner decision (2026-10-01)

The project owner reviewed the two tables above and chose to keep them unchanged in the public source. This records a deliberate risk acceptance, not a legal conclusion. If replacement ever becomes necessary, document an independently justified derivation and validate its behavior. Do not merely re-encode copied numbers or silently remove gameplay.

The public technical notes describe the current implementation and its limits. They are not transcripts of manuals, guides, native program decompilations or a complete formal specification. No legal conclusion should be inferred from a compiler, source inventory, scanner, code review or passing mission test.

See [PUBLISHING.md](../PUBLISHING.md) for the rules that apply to published changes.
