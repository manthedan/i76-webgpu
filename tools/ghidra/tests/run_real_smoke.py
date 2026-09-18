#!/usr/bin/env python3
"""Run the portable wrappers against a new self-authored ELF and real Ghidra.

All generated binaries, projects, maps, query results, and logs stay beneath a
new caller-selected external work directory. The directory is never deleted.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
HEADLESS = ROOT / "tools/ghidra/headless.py"
QUERY = ROOT / "tools/ghidra_query.py"
SOURCE = Path(__file__).resolve().with_name("synthetic_fixture.c")
SLEEPER_SOURCE = Path(__file__).resolve().with_name("SyntheticSleeper.java")


def command_text(command: list[str]) -> str:
    return " ".join(json.dumps(value) for value in command)


def run_step(work: Path, number: int, name: str, command: list[str],
             expected: int | tuple[int, ...]) -> str:
    log = work / ("%02d-%s.log" % (number, name))
    started = time.monotonic()
    completed = subprocess.run(command, stdin=subprocess.DEVNULL,
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, errors="replace", check=False)
    elapsed = time.monotonic() - started
    allowed = (expected,) if isinstance(expected, int) else expected
    expected_text = "/".join(str(value) for value in allowed)
    text = (
        "$ " + command_text(command) + "\n" +
        completed.stdout +
        "\n[runner] exit=%d expected=%s elapsed=%.3fs\n" %
        (completed.returncode, expected_text, elapsed)
    )
    log.write_text(text, encoding="utf-8")
    print("%-34s exit=%3d expected=%-5s log=%s" %
          (name, completed.returncode, expected_text, log))
    if completed.returncode not in allowed:
        raise RuntimeError("step %s returned %d, expected %s (see %s)" %
                           (name, completed.returncode, expected_text, log))
    return completed.stdout


def matching_processes(fragment: str) -> list[dict[str, object]]:
    matches = []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", "replace"
            )
        except OSError:
            continue
        if fragment in command:
            matches.append({"pid": int(entry.name), "cmdline": command})
    return matches


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("--work-dir", required=True,
                       help="new short external directory, preferably directly under /tmp")
    value.add_argument("--analyze-headless", required=True)
    value.add_argument("--java-home", required=True)
    value.add_argument("--cc", default="gcc")
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    work = Path(args.work_dir).expanduser().resolve()
    if work.exists():
        raise SystemExit("refusing existing --work-dir: %s" % work)
    if str(work).startswith(str(ROOT) + os.sep):
        raise SystemExit("--work-dir must be outside the source checkout")
    work.mkdir(mode=0o700, parents=True)
    projects = work / "projects"
    output = work / "output"
    state = work / "state"
    projects.mkdir()
    output.mkdir()
    binary = work / "synthetic_fixture"
    project_name = "SyntheticSmoke"
    program = binary.name

    step = 0

    def run(name: str, command: list[str], expected=0) -> str:
        nonlocal step
        step += 1
        return run_step(work, step, name, command, expected)

    try:
        run("compile", [args.cc, "-g", "-O0", "-fno-pie", "-no-pie",
                        "-Wall", "-Wextra", "-o", str(binary), str(SOURCE)])
        java_home = Path(args.java_home).resolve()
        run("compile-unrelated-jvm", [str(java_home / "bin/javac"), "-d", str(work),
                                      str(SLEEPER_SOURCE)])
        install = ["--analyze-headless", str(Path(args.analyze_headless).resolve()),
                   "--java-home", str(Path(args.java_home).resolve()),
                   "--project-dir", str(projects), "--project-name", project_name]
        run("import", [sys.executable, str(HEADLESS), "import-analyze", *install,
                       "--binary", str(binary), "--analysis-timeout", "300",
                       "--timeout", "600"])
        common = [sys.executable, str(HEADLESS), "query", *install,
                  "--program", program]
        function_map = output / "function-map.json"
        run("function-map", [*common, "export_function_map.py", "--",
                             str(function_map)])
        payload = json.loads(function_map.read_text(encoding="utf-8"))
        addresses = {item["name"]: item["entry"] for item in payload["functions"]}
        for required in ("synthetic_add", "synthetic_twice", "main"):
            if required not in addresses:
                raise RuntimeError("function map lacks %s" % required)

        run("decompile", [*common, "decomp_at.py", "--", addresses["synthetic_twice"]])
        run("xrefs", [*common, "xref_fn.py", "--", addresses["synthetic_add"], "20"])
        run("search", [*common, "find_str.py", "--", "synthetic-fixture-message", "20"])
        run("missing-address", [*common, "decomp_at.py", "--", "0x1"], expected=2)
        missing_program = list(common)
        missing_program[missing_program.index("--program") + 1] = "does-not-exist"
        run("missing-program", [*missing_program, "decomp_at.py", "--",
                                addresses["synthetic_twice"]], expected=(1, 2))
        unrelated_jvm = subprocess.Popen(
            [str(java_home / "bin/java"), "-cp", str(work), "SyntheticSleeper"],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, start_new_session=True,
        )
        try:
            time.sleep(0.25)
            if unrelated_jvm.poll() is not None:
                raise RuntimeError("unrelated synthetic JVM exited before timeout test")
            run("timeout", [*common, "--timeout", "1", "decomp_at.py", "--",
                            addresses["synthetic_twice"]], expected=124)
            survived = unrelated_jvm.poll() is None
            (work / "timeout-unrelated-jvm.json").write_text(json.dumps({
                "pid": unrelated_jvm.pid, "survived_timeout": survived,
            }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            if not survived:
                raise RuntimeError("timeout cleanup killed the unrelated synthetic JVM")
        finally:
            if unrelated_jvm.poll() is None:
                unrelated_jvm.terminate()
            unrelated_jvm.wait()
        time.sleep(1)
        survivors = matching_processes(str(projects))
        (work / "timeout-survivors.json").write_text(
            json.dumps(survivors, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        if survivors:
            raise RuntimeError("timed-out invocation left project-bound processes")

        persistent = [sys.executable, str(QUERY),
                      "--analyze-headless", str(Path(args.analyze_headless).resolve()),
                      "--java-home", str(java_home),
                      "--project-dir", str(projects), "--project-name", project_name,
                      "--program", program, "--state-dir", str(state),
                      "--startup-timeout", "180"]
        run("persistent-cold-decompile",
            [*persistent, "--provenance", "synthetic-real-smoke",
             "ghidra.decompile", "synthetic_twice"])
        run("persistent-xrefs", [*persistent, "ghidra.xrefs", "synthetic_add", "--top", "20"])
        run("persistent-search", [*persistent, "ghidra.search", "synthetic", "--top", "20"])
        run("persistent-dash-search",
            [*persistent, "ghidra.search", "--", "-literal-dash-search"])
        run("persistent-missing-address",
            [*persistent, "ghidra.decompile", "0x1"], expected=2)
        run("persistent-status", [*persistent, "service.status"])
        run("persistent-stop", [*persistent, "service.stop"])
    except Exception as exc:
        (work / "RESULT.json").write_text(json.dumps({
            "schema": "portable-ghidra-real-smoke-v1", "ok": False,
            "error": str(exc), "completed_steps": step,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print("FAIL: %s" % exc, file=sys.stderr)
        return 1

    result = {
        "schema": "portable-ghidra-real-smoke-v1",
        "ok": True,
        "completed_steps": step,
        "fixture_source": str(SOURCE),
        "generated_files_external": True,
        "work_dir": str(work),
    }
    (work / "RESULT.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
