# Portable Ghidra tooling for i76-web development

This directory is an **asset-free developer toolkit** extracted from the
pinned i76-web development source (`979f7798d26f87c9351018f799e6b074f153af99`).
It contains Ghidra API scripts and host launchers, not a replacement disassembler.
It ships no game executable, Ghidra project/database, decompiled output, coverage
capture, purchaser asset, JDK, or Ghidra installation.

## Tested toolchain and evidence boundary

The package was prepared against:

- **Ghidra 11.4.2 PUBLIC** (`application.java.min=21`)
- **Eclipse Temurin/OpenJDK 21.0.12+8 LTS**, 64-bit
- Python 3 host launchers; Ghidra's bundled Jython environment for scripts

The focused unit tests use fake executables and a fake Unix-socket service. They
prove argument construction, path/output guards, protocol checks, namespace
binding, timeout ownership, and the absence of listed mutation APIs. The optional
`tests/run_real_smoke.py` runner separately compiles the included self-authored C
fixture and verifies import, decompile, xrefs, string search, function-map export,
negative program/address handling, timeout cleanup, and persistent lifecycle
against an installed Ghidra. It writes every generated file outside the checkout.
This runner was exercised with the pinned Ghidra/JDK versions above; it is focused
tool validation, not validation against a game binary or existing project.

Ghidra and a compatible JDK must be installed separately. Ghidra 11.4.2 requires
JDK 21. Point `--analyze-headless` at
`<GHIDRA_HOME>/support/analyzeHeadless`. If another `java` appears first on
`PATH`, pass `--java-home <JDK21_HOME>`; the launcher prepends its `bin` directory.

## Safety and data boundaries

1. **Projects, state, caches, and results must be outside this source checkout.**
   The launchers reject project directories, persistent state, and function-map
   output inside the package tree.
2. **A project and program are always explicit.** There is no private project,
   executable, or program-name default.
3. **Import/analysis is a separate mutating operation.** `import-analyze` refuses
   either an existing `<project-name>.gpr` or a residual `<project-name>.rep`, so
   a partial project is preserved rather than reused. Ordinary queries always use
   `-process PROGRAM -noanalysis -readOnly`.
4. The included read-only scripts do not force disassembly, create functions,
   rename symbols, set comments, or open transactions. Scripts from the source
   archive that did those things were deliberately excluded.
5. The persistent service listens only on an owner-mode **local Unix socket**.
   It has fixed verbs (decompile, call graph, xrefs, symbol search), no TCP
   listener, eval, arbitrary-script, import, or mutation verb.
6. Results can reveal executable structure, strings, symbols, and pseudocode.
   They are written only to paths you choose. Do not commit or publish purchaser
   binaries, Ghidra databases, decompiled output, coverage captures, or other
   derived artifacts without a separate provenance/legal review.

Decoded native tables retained in the source are covered by the owner decision in [Provenance](../../docs/PROVENANCE.md).
This toolkit is not legal clearance.

## Create your own analysis project

Use only an executable you lawfully own. Keep both the executable and generated
project outside the checkout. The parent project directory must already exist.
The explicit import operation creates a new project and performs initial analysis:

```sh
python3 tools/ghidra/headless.py import-analyze \
  --analyze-headless /opt/ghidra_11.4.2_PUBLIC/support/analyzeHeadless \
  --java-home /opt/jdk-21.0.12+8 \
  --project-dir "$HOME/ghidra-projects" \
  --project-name MyPurchasedCopy \
  --binary "$HOME/private-game-binaries/my-program.exe" \
  --analysis-timeout 1200 \
  --timeout 1800
```

The launcher does not pass `-overwrite` and refuses an existing project marker or
repository directory. Subsequent analysis changes, re-imports, and symbol edits
are intentionally out of scope; use a separate disposable project and an explicit
reviewed workflow.

## One-shot read-only queries

Every one-shot query names the installation, project, and exact program:

```sh
COMMON=(
  --analyze-headless /opt/ghidra_11.4.2_PUBLIC/support/analyzeHeadless
  --java-home /opt/jdk-21.0.12+8
  --project-dir "$HOME/ghidra-projects"
  --project-name MyPurchasedCopy
  --program my-program.exe
)

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  decomp_at.py -- 0x00401000

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  decomp_many.py -- 0x00401000 0x00401100

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  disasm_at.py -- 0x00401020 20

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  disasm_fn.py -- 0x00401000 10000

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  xref_fn.py -- 0x00401000 500

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  xref_addr.py -- 0x00410000 500

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  find_str.py -- 'diagnostic substring' 100

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  find_imm.py -- 0x56444643 1000

python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  function_lookup.py -- update_vehicle 50
```

Arguments are passed directly as an argv list; no shell command is assembled.
Project names must be literal basenames (not `.`, `..`, an option, or a wildcard),
and program selectors must be exact literals rather than options or wildcards.
Forwarded one-shot script arguments beginning `-` are rejected because Ghidra may
parse them as headless options. For a literal dash-prefixed symbol-search string,
use the persistent JSON command, for example `ghidra.search -- -literal-name`.

The launcher has a 300-second default query timeout (maximum 1,800 seconds).
Each bundled script emits a completion marker; an exit-0 Ghidra run without that
marker is a wrapper failure, while Ghidra's diagnostics remain visible. Timeouts
terminate only the new process group created for that invocation, never a host-wide
Java process match. Individual scripts also bound addresses, hits, references,
output, or instruction counts. Missing functions and out-of-memory-map addresses
fail clearly rather than creating code or function definitions.

### Function/call export for coverage mapping

`export_function_map.py` is the defensible coverage-related subset. It exports
existing function bounds and direct call edges, plus executable SHA-256, image
base, and Ghidra version. It **does not collect runtime coverage** or interpret a
coverage hit as program semantics.

```sh
mkdir -p "$HOME/private-analysis-results"
python3 tools/ghidra/headless.py query "${COMMON[@]}" \
  export_function_map.py -- \
  "$HOME/private-analysis-results/my-program-functions.json"
```

The output parent must exist; an existing output is never overwritten. Runtime
capture, VM automation, oracle jobs, and game-specific differential ranking are
not included.

## Persistent read-only service

Repeated decompilation can share one Ghidra JVM. The retained service/client is
portable after removing private path and program defaults. It uses Linux/Unix
facilities (`AF_UNIX`, `fcntl`, and `/proc` process validation); the audited
host lifecycle is currently **Linux-only**. The one-shot launcher remains the
simpler cross-host path.

Set a short external state base. The client creates an owner-mode namespace keyed
by the canonical project directory, literal project name, and literal program.
That namespace contains the socket, PID, log, binding metadata, and any
truncated-result artifacts, all potentially sensitive. Reusing the same state
base for a different project creates a different namespace; there is no socket
override that can bypass ownership. Existing general user directories are never
chmodded, while the generated namespace is required to be owner-private:

```sh
Q=(
  --analyze-headless /opt/ghidra_11.4.2_PUBLIC/support/analyzeHeadless
  --java-home /opt/jdk-21.0.12+8
  --project-dir "$HOME/ghidra-projects"
  --project-name MyPurchasedCopy
  --program my-program.exe
  --state-dir "${XDG_RUNTIME_DIR:-$HOME/.cache}/ghidra-my-program"
)

python3 tools/ghidra_query.py "${Q[@]}" service.start
python3 tools/ghidra_query.py "${Q[@]}" \
  --provenance local-investigation ghidra.decompile 0x00401000
python3 tools/ghidra_query.py "${Q[@]}" ghidra.call_graph 0x00401000 --depth 2 --top 50
python3 tools/ghidra_query.py "${Q[@]}" ghidra.xrefs 0x00401000 --top 50
python3 tools/ghidra_query.py "${Q[@]}" ghidra.search update_vehicle --top 50
python3 tools/ghidra_query.py "${Q[@]}" service.status
python3 tools/ghidra_query.py "${Q[@]}" service.stop
```

Cold start is bounded to 180 seconds by default (maximum 900). Requests are
limited to 64 KiB, responses to 4 MiB, result count to 100, graph depth to 8,
and decompilation to 120 seconds/256 KiB inline. Larger complete results, where
supported, are owner-mode artifacts in `--state-dir`. The service serializes
requests. Lifecycle operations fail explicitly off Linux or without `/proc`;
they also refuse a ping, busy PID, query response, or stop response whose binding
does not match the requested project/program. Stop it before moving or deleting
its project.

`-readOnly` prevents project saves; `-noanalysis` prevents an analysis pass.
This is genuinely read-only because service verbs only inspect existing program
state. The excluded force-disassembly/create-function scripts would mutate
in-memory program state even when saves were disabled, so they are not described
or packaged as read-only tools.

## Included scripts and limitations

| File | Purpose | Important limit |
|---|---|---|
| `decomp_at.py` | Decompile containing existing function | 120 s; 1 MiB text |
| `decomp_many.py` | Batch decompile in one JVM | 64 addresses; 256 KiB each |
| `disasm_at.py` | Existing instructions around address | radius <= 200 |
| `disasm_fn.py` | Existing instructions in function | <= 50,000 instructions |
| `xref_addr.py` | References to any mapped address | <= 10,000 |
| `xref_fn.py` | References to containing function entry | <= 10,000 |
| `find_str.py` | Defined-string substring search | <= 10,000 hits |
| `find_imm.py` | Scalar-immediate instruction search | <= 10,000 hits |
| `function_lookup.py` | Name substring or address neighbors | <= 1,000 name results |
| `export_function_map.py` | Function bounds/direct call-edge JSON | 100k functions/500k edges |
| `query_service.py` | Fixed-verb local persistent service | no mutation/eval/script verb |

Decompiler output is an analysis aid, not source code or ground truth. Function
boundaries, symbols, xrefs, and call edges are only as good as the imported
program's analysis. An absent result is not proof that behavior is absent.

## Tests

From the package root:

```sh
python3 -m unittest discover -s tools/tests -p 'test_*.py' -v
```

The unit tests are asset-free and do not open Ghidra. To run the real, still
asset-free synthetic smoke test, choose a **new short** external directory:

```sh
python3 tools/ghidra/tests/run_real_smoke.py \
  --work-dir /tmp/ghidra-synthetic-smoke-001 \
  --analyze-headless /opt/ghidra_11.4.2_PUBLIC/support/analyzeHeadless \
  --java-home /opt/jdk-21.0.12+8
```

The runner refuses an existing work directory and preserves all logs, the generated
ELF, disposable Ghidra project, JSON responses, and failed attempts there. Do not
commit those outputs. See `decomp-stage-evidence/` for the preparation run's older
retained logs.
