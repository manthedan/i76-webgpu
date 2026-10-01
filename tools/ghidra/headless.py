#!/usr/bin/env python3
"""Portable, bounded launcher for this package's Ghidra headless scripts.

Read-only queries and the explicit import/analyze operation are intentionally
separate subcommands. Commands are passed as argv arrays; no shell is involved.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

READ_ONLY_SCRIPTS = {
    "decomp_at.py",
    "decomp_many.py",
    "disasm_at.py",
    "disasm_fn.py",
    "find_imm.py",
    "find_str.py",
    "function_lookup.py",
    "xref_addr.py",
    "xref_fn.py",
    "export_function_map.py",
}
DEFAULT_QUERY_TIMEOUT = 300
MAX_QUERY_TIMEOUT = 1800
DEFAULT_IMPORT_TIMEOUT = 1800
MAX_IMPORT_TIMEOUT = 7200
SUCCESS_MARKER_PREFIX = "I76_GHIDRA_SCRIPT_OK:"
PROGRAM_METACHARACTERS = "*?[]"


class LaunchError(Exception):
    pass


def source_root() -> Path:
    return Path(__file__).resolve().parents[2]


def resolved(path: str) -> Path:
    return Path(path).expanduser().resolve()


def is_within(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
        return True
    except ValueError:
        return False


def require_outside_source(path: Path, label: str) -> None:
    if is_within(path, source_root()):
        raise LaunchError(f"{label} must be outside the source checkout: {path}")


def validate_executable(path: str) -> Path:
    value = resolved(path)
    if not value.is_file() or not os.access(value, os.X_OK):
        raise LaunchError(f"analyzeHeadless is not executable: {value}")
    return value


def java_environment(java_home: str | None) -> dict[str, str]:
    environment = os.environ.copy()
    if java_home:
        home = resolved(java_home)
        java = home / "bin" / "java"
        if not java.is_file() or not os.access(java, os.X_OK):
            raise LaunchError(f"JDK java is not executable: {java}")
        environment["JAVA_HOME"] = str(home)
        environment["PATH"] = str(java.parent) + os.pathsep + environment.get("PATH", "")
    return environment


def validate_project_name(value: str) -> str:
    if (not value or value in (".", "..") or value.startswith("-") or
            any(ch in value for ch in "/\\\0" + PROGRAM_METACHARACTERS)):
        raise LaunchError(
            "project name must be a literal basename, not an option, wildcard, '.', or '..'"
        )
    return value


def validate_program_name(value: str) -> str:
    if not value:
        raise LaunchError("program must not be empty")
    if value.startswith("-") or any(ch in value for ch in PROGRAM_METACHARACTERS):
        raise LaunchError("program must be an exact literal selector (no options or wildcards)")
    if "\0" in value:
        raise LaunchError("program contains a NUL byte")
    return value


def validate_project(args: argparse.Namespace, must_exist: bool) -> tuple[Path, Path]:
    project_dir = resolved(args.project_dir)
    require_outside_source(project_dir, "project directory")
    if not project_dir.is_dir():
        raise LaunchError(f"project directory does not exist: {project_dir}")
    project_name = validate_project_name(args.project_name)
    marker = project_dir / f"{project_name}.gpr"
    if must_exist and not marker.is_file():
        raise LaunchError(f"Ghidra project marker does not exist: {marker}")
    return project_dir, marker


def query_command(args: argparse.Namespace) -> list[str]:
    analyze = validate_executable(args.analyze_headless)
    project_dir, _ = validate_project(args, must_exist=True)
    program = validate_program_name(args.program)
    script_dir = Path(__file__).resolve().parent
    if args.script not in READ_ONLY_SCRIPTS:
        raise LaunchError(f"script is not in the read-only allowlist: {args.script}")
    script_file = script_dir / args.script
    if not script_file.is_file():
        raise LaunchError(f"bundled script is missing: {script_file}")
    script_args = list(args.script_args)
    if script_args[:1] == ["--"]:
        script_args = script_args[1:]
    if any(value.startswith("-") for value in script_args):
        raise LaunchError(
            "forwarded script arguments must not begin '-'; use the persistent JSON "
            "search for a literal dash-prefixed search string"
        )
    if args.script == "export_function_map.py":
        if len(script_args) != 1:
            raise LaunchError("export_function_map.py requires exactly one output path")
        output = resolved(script_args[0])
        require_outside_source(output, "export output")
    return [
        str(analyze), str(project_dir), args.project_name,
        "-process", program, "-noanalysis", "-readOnly",
        "-scriptPath", str(script_dir), "-postScript", args.script,
        *script_args,
    ]


def import_command(args: argparse.Namespace) -> list[str]:
    analyze = validate_executable(args.analyze_headless)
    project_dir, marker = validate_project(args, must_exist=False)
    repository = project_dir / f"{args.project_name}.rep"
    existing = [path for path in (marker, repository) if path.exists() or path.is_symlink()]
    if existing:
        raise LaunchError(
            "refusing to import into an existing or partial project: %s; "
            "create a new project name for this explicit operation" %
            ", ".join(str(path) for path in existing)
        )
    binary = resolved(args.binary)
    require_outside_source(binary, "input binary")
    if not binary.is_file():
        raise LaunchError(f"input binary does not exist: {binary}")
    return [
        str(analyze), str(project_dir), args.project_name,
        "-import", str(binary),
        "-analysisTimeoutPerFile", str(args.analysis_timeout),
        "-scriptPath", str(Path(__file__).resolve().parent),
        "-postScript", "launch_success.py", "import-analyze",
    ]


def add_install_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--analyze-headless", required=True,
                        help="path to Ghidra support/analyzeHeadless")
    parser.add_argument("--java-home", help="optional JDK home; its bin is prepended to PATH")
    parser.add_argument("--project-dir", required=True,
                        help="existing project-location directory outside this checkout")
    parser.add_argument("--project-name", required=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    query = sub.add_parser("query", help="run one bundled script read-only")
    add_install_options(query)
    query.add_argument("--program", required=True,
                       help="exact Ghidra program name")
    query.add_argument("--timeout", type=int, default=DEFAULT_QUERY_TIMEOUT)
    query.add_argument("script", choices=sorted(READ_ONLY_SCRIPTS))
    query.add_argument("script_args", nargs=argparse.REMAINDER)

    initial = sub.add_parser(
        "import-analyze",
        help="explicitly create and analyze a new project from your own binary",
    )
    add_install_options(initial)
    initial.add_argument("--binary", required=True)
    initial.add_argument("--timeout", type=int, default=DEFAULT_IMPORT_TIMEOUT)
    initial.add_argument("--analysis-timeout", type=int, default=1200,
                         help="Ghidra per-file analysis timeout in seconds")
    return parser


def _terminate_invocation(process: subprocess.Popen) -> None:
    """Terminate only the process group created for this invocation."""
    if os.name == "posix":
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
    else:
        process.terminate()
    deadline = time.monotonic() + 5
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        pass
    if os.name == "posix":
        # The launcher may exit before its descendants (Java, helpers), so
        # the grace period applies to the whole group, not only its leader.
        while time.monotonic() < deadline:
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.1)
    elif process.poll() is not None:
        return
    if os.name == "posix":
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
    else:
        process.kill()
    process.wait()


def _write_diagnostics(output: bytes) -> None:
    stream = getattr(sys.stdout, "buffer", None)
    if stream is not None:
        stream.write(output)
        stream.flush()
    else:
        sys.stdout.write(output.decode("utf-8", "replace"))
        sys.stdout.flush()


def run_command(command: list[str], environment: dict[str, str], timeout: int,
                success_name: str) -> int:
    expected = (SUCCESS_MARKER_PREFIX + " " + success_name).encode("utf-8")
    with tempfile.TemporaryFile() as captured:
        process = subprocess.Popen(
            command,
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=captured,
            stderr=subprocess.STDOUT,
            start_new_session=(os.name == "posix"),
            close_fds=True,
        )
        try:
            returncode = process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            _terminate_invocation(process)
            captured.seek(0)
            _write_diagnostics(captured.read())
            print(f"error: Ghidra invocation exceeded {timeout}s", file=sys.stderr)
            return 124
        captured.seek(0)
        output = captured.read()
    _write_diagnostics(output)
    if returncode != 0:
        return returncode
    if expected not in output.splitlines():
        print(
            "error: Ghidra exited 0 without a script success marker; "
            "the program may be missing or the post-script may have failed",
            file=sys.stderr,
        )
        return 2
    return 0


def run(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "query":
            if not 1 <= args.timeout <= MAX_QUERY_TIMEOUT:
                raise LaunchError(f"query timeout must be in [1, {MAX_QUERY_TIMEOUT}]")
            command = query_command(args)
            success_name = args.script
        else:
            if not 1 <= args.timeout <= MAX_IMPORT_TIMEOUT:
                raise LaunchError(f"import timeout must be in [1, {MAX_IMPORT_TIMEOUT}]")
            if not 1 <= args.analysis_timeout <= MAX_IMPORT_TIMEOUT:
                raise LaunchError(
                    f"analysis timeout must be in [1, {MAX_IMPORT_TIMEOUT}]"
                )
            command = import_command(args)
            success_name = "import-analyze"
        return run_command(command, java_environment(args.java_home), args.timeout,
                           success_name)
    except (LaunchError, OSError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


def main() -> None:
    raise SystemExit(run())


if __name__ == "__main__":
    main()
