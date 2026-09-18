# Contributing

This is an unfinished, code-only developer preview. Small reproducible changes are more useful than claims that a subsystem or campaign is complete.

## Keep purchased and private material out

Do not submit game archives, executables, missions, extracted/converted assets, movies, audio, purchaser screenshots, raw executable/disassembly dumps, credentials, private hostnames or local evidence bundles. Use generated test fixtures or hashes of locally inspected data instead. Describe reproduction steps without attaching the game files. Keep your purchased installation outside the checkout.

Do not copy another project's implementation without checking its licence and provenance. In particular, Open76 is a specification/reference source here, not a code source. Preserve upstream attribution and the recorded Roanish grant; our MIT licence cannot relicense other authors' code. See THIRD-PARTY.md.

## Make a bounded change

1. Build and run `./tools/check.sh` as described in README.md.
2. For decoder changes, also run `python3 tools/test_lzodec.py` with the documented native dependencies.
3. If you own the game, use the optional component tests and ordinary page controls to reproduce relevant behavior. Label automation, developer sensing and synthetic initial conditions honestly.
4. Include a regression and explain what it proves. Do not change mission success conditions, fixtures or assertions merely to turn a failed run green.
5. Keep numerical simulation/build changes isolated. Record toolchain/browser versions and distinguish a build, a component result, a browser journey and a human verdict.

The public `PUBLIC_FILES.txt` is an explicit source-file inventory. Add deliberate new source files to it; do not add generated artifacts or assets. `tools/check_source.py` checks inventory/text-file boundaries, not exhaustive secret detection or legal clearance. Maintainers should separately scan and review the final export before publication.

Pull requests should state intent, actual tests run, known failures and source/dependency provenance. No hosted deployment, browser support expansion or campaign-completion claim follows automatically from merging a patch.
