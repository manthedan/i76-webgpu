#!/usr/bin/env python3
"""CLI for the persistent, local, read-only Ghidra query service.

Every invocation requires an explicit Ghidra executable, project location,
project name, program, and state directory. The service listens only on an
owner-only Unix-domain socket and exposes no arbitrary script/eval operation.
Derived from i76-web's MIT-licensed query client (manthedan, 2026).
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import stat
import subprocess
import sys
import tempfile
import time
import uuid

from ghidra import headless

try:
    import fcntl
except ImportError:  # Explicit platform error is emitted by ensure_supported_platform().
    fcntl = None

REQUEST_SCHEMA = "i76-ghidra-query-request-v1"
RESPONSE_SCHEMA = "i76-ghidra-query-v1"
SERVICE_SCHEMA = "i76-ghidra-query-service-v1"
CLIENT_VERSION = "2"
MAX_WIRE_BYTES = 4 * 1024 * 1024
DEFAULT_TOP = 50
MAX_TOP = 100
DEFAULT_DEPTH = 1
MAX_DEPTH = 8


class QueryError(Exception):
    pass


def ensure_supported_platform():
    if (not sys.platform.startswith("linux") or fcntl is None or
            not Path("/proc/self/status").is_file()):
        raise QueryError(
            "persistent Ghidra service lifecycle is supported only on Linux with /proc"
        )


def _binding(project_dir, project_name, program):
    try:
        name = headless.validate_project_name(project_name)
        program_name = headless.validate_program_name(program)
        project_path = headless.resolved(project_dir)
        headless.require_outside_source(project_path, "project directory")
    except headless.LaunchError as exc:
        raise QueryError(str(exc))
    value = {
        "project_dir": str(project_path),
        "project_name": name,
        "program": program_name,
    }
    encoded = json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")
    value["id"] = hashlib.sha256(encoded).hexdigest()
    return value


def _runtime_root(configured, binding):
    base = Path(configured).expanduser().resolve()
    if headless.is_within(base, headless.source_root()):
        raise QueryError("state directory must be outside the source checkout: %s" % base)
    existed = base.exists()
    base.mkdir(mode=0o700, parents=True, exist_ok=True)
    base_stat = base.lstat()
    if base.is_symlink() or not base.is_dir() or base_stat.st_uid != os.getuid():
        raise QueryError("unsafe service state base directory: %s" % base)
    if base_stat.st_mode & 0o022:
        raise QueryError("service state base must not be group/other writable: %s" % base)
    if not existed and base_stat.st_mode & 0o077:
        raise QueryError("new service state base is not owner-private: %s" % base)

    root = base / ("ghq-" + binding["id"][:20])
    root.mkdir(mode=0o700, exist_ok=True)
    root_stat = root.lstat()
    if (root.is_symlink() or not root.is_dir() or root_stat.st_uid != os.getuid() or
            root_stat.st_mode & 0o077):
        raise QueryError("service namespace must be owner-private: %s" % root)

    identity_path = root / "binding.json"
    expected = json.dumps(binding, sort_keys=True, indent=2) + "\n"
    # Publish the binding complete or not at all: write a private temporary
    # file, then hard-link it into place (EEXIST if another client won), so a
    # concurrent first request never reads an empty or partial binding.
    descriptor, temporary = tempfile.mkstemp(dir=str(root), prefix=".binding-")
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(expected)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, str(identity_path))
        except FileExistsError:
            try:
                actual = identity_path.read_text(encoding="utf-8")
            except OSError as exc:
                raise QueryError("cannot read service namespace binding: %s" % exc)
            if actual != expected:
                raise QueryError("service namespace binding mismatch: %s" % identity_path)
    finally:
        os.unlink(temporary)
    return base, root


def default_paths(state_dir, project_dir, project_name, program):
    ensure_supported_platform()
    binding = _binding(project_dir, project_name, program)
    base, root = _runtime_root(state_dir, binding)
    socket_path = root / "s"
    if len(os.fsencode(str(socket_path))) > 107:
        raise QueryError(
            "namespaced Unix socket path is too long; choose a shorter --state-dir: %s" %
            socket_path
        )
    return {
        "base": base,
        "root": root,
        "binding": binding,
        "socket": socket_path,
        "pid": root / "service.pid",
        "log": root / "service.log",
        "lock": root / "start.lock",
        "artifacts": root / "artifacts",
    }


def _strict_json(text):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise QueryError("duplicate JSON key: %s" % key)
            result[key] = value
        return result
    try:
        return json.loads(text, object_pairs_hook=pairs)
    except (ValueError, TypeError) as exc:
        if isinstance(exc, QueryError):
            raise
        raise QueryError("invalid service JSON: %s" % exc)


def _recv_line(sock):
    chunks = []
    size = 0
    while True:
        chunk = sock.recv(min(65536, MAX_WIRE_BYTES + 1 - size))
        if not chunk:
            break
        chunks.append(chunk)
        size += len(chunk)
        if size > MAX_WIRE_BYTES:
            raise QueryError("service response exceeds %d bytes" % MAX_WIRE_BYTES)
        if b"\n" in chunk:
            break
    raw = b"".join(chunks)
    line, separator, trailing = raw.partition(b"\n")
    if not separator:
        raise QueryError("service closed without a complete response")
    if trailing.strip():
        raise QueryError("service sent data after its JSON response")
    try:
        return line.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise QueryError("service response is not UTF-8: %s" % exc)


def exchange(socket_path, payload, timeout=180.0):
    encoded = (json.dumps(payload, sort_keys=True, separators=(",", ":")) + "\n").encode("utf-8")
    if len(encoded) > 64 * 1024:
        raise QueryError("request exceeds 65536 bytes")
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout)
    try:
        client.connect(str(socket_path))
        client.sendall(encoded)
        response = _strict_json(_recv_line(client))
    except (OSError, socket.timeout) as exc:
        raise QueryError("service exchange failed: %s" % exc)
    finally:
        client.close()
    if not isinstance(response, dict):
        raise QueryError("service response must be an object")
    return response


def ping(socket_path, timeout=1.0):
    request_id = str(uuid.uuid4())
    try:
        response = exchange(socket_path, {
            "schema": REQUEST_SCHEMA,
            "verb": "service.ping",
            "request_id": request_id,
            "query": {},
            "run": {"client": "ghidra_query.py", "client_version": CLIENT_VERSION,
                    "client_pid": os.getpid(), "provenance": "lifecycle"},
        }, timeout)
    except QueryError:
        return None
    if (response.get("schema") != SERVICE_SCHEMA or
            response.get("request_id") != request_id or
            response.get("status") != "ready"):
        return None
    return response


def _tracked_process(paths):
    ensure_supported_platform()
    try:
        text = paths["pid"].read_text(encoding="ascii").strip()
        pid = int(text)
        if pid < 2:
            return None
        os.kill(pid, 0)
        cmdline = Path("/proc/%d/cmdline" % pid).read_bytes().replace(b"\0", b" ")
    except (OSError, ValueError):
        return None
    required = (b"analyzeHeadless", b"query_service.py",
                str(paths["socket"]).encode("utf-8"))
    if not all(item in cmdline for item in required):
        return None
    binding_id = paths["binding"]["id"].encode("ascii")
    if binding_id not in cmdline:
        raise QueryError("tracked service process binding mismatch; refusing lifecycle action")
    return pid


def _validate_service_binding(response, paths):
    actual = response.get("binding", {}).get("id")
    expected = paths["binding"]["id"]
    if actual != expected:
        raise QueryError("running service binding mismatch: expected %r, got %r" %
                         (expected, actual))
    actual_program = response.get("program", {}).get("name")
    if actual_program != paths["binding"]["program"]:
        raise QueryError("running service program mismatch: expected %r, got %r" %
                         (paths["binding"]["program"], actual_program))


def service_state(paths):
    ready = ping(paths["socket"])
    if ready is not None:
        _validate_service_binding(ready, paths)
        return "ready", ready
    pid = _tracked_process(paths)
    if pid is not None:
        return "busy", {"status": "busy", "service_pid": pid,
                        "project_mode": "read-only",
                        "binding": paths["binding"]}
    return "stopped", None


def _write_pid(path, pid):
    temporary = path.with_name(path.name + ".tmp-%d" % os.getpid())
    temporary.write_text("%d\n" % pid, encoding="ascii")
    os.replace(str(temporary), str(path))


def _validated_start(args):
    try:
        analyze = headless.validate_executable(args.analyze_headless)
        project_dir, _ = headless.validate_project(args, must_exist=True)
        environment = headless.java_environment(args.java_home)
    except headless.LaunchError as exc:
        raise QueryError(str(exc))
    try:
        headless.validate_program_name(args.program)
    except headless.LaunchError as exc:
        raise QueryError(str(exc))
    return analyze, project_dir, environment


def start_service(args, paths):
    analyze, project_dir, environment = _validated_start(args)
    paths["artifacts"].mkdir(mode=0o700, parents=True, exist_ok=True)
    paths["lock"].touch(mode=0o600, exist_ok=True)
    with paths["lock"].open("r+") as lock:
        fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
        state, existing = service_state(paths)
        if state == "ready":
            return existing, False
        if state == "busy":
            # The daemon is serial. A one-second ping can queue behind a
            # decompile for up to 120 seconds; unlinking its live socket here
            # would strand that JVM and race a second writer for the project.
            return existing, False
        if paths["socket"].exists() or paths["socket"].is_symlink():
            socket_stat = paths["socket"].lstat()
            if paths["socket"].is_symlink() or not stat.S_ISSOCK(socket_stat.st_mode):
                raise QueryError("refusing to unlink non-socket path: %s" % paths["socket"])
            if socket_stat.st_uid != os.getuid():
                raise QueryError("refusing foreign-owned stale socket: %s" % paths["socket"])
            paths["socket"].unlink()

        script_dir = Path(__file__).resolve().parent / "ghidra"
        command = [str(analyze), str(project_dir), args.project_name,
                   "-process", args.program, "-noanalysis", "-readOnly",
                   "-scriptPath", str(script_dir), "-postScript", "query_service.py",
                   str(paths["socket"]), str(paths["artifacts"]),
                   paths["binding"]["id"]]

        log = paths["log"].open("ab", buffering=0)
        try:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log,
                                       stderr=subprocess.STDOUT, env=environment,
                                       start_new_session=True, close_fds=True)
        finally:
            log.close()
        _write_pid(paths["pid"], process.pid)

        deadline = time.monotonic() + args.startup_timeout
        while time.monotonic() < deadline:
            status = process.poll()
            if status is not None:
                tail = ""
                try:
                    tail = paths["log"].read_text(encoding="utf-8", errors="replace")[-4000:]
                except OSError:
                    pass
                raise QueryError("Ghidra service exited %d during startup; log=%s\n%s" %
                                 (status, paths["log"], tail))
            ready = ping(paths["socket"])
            if ready is not None:
                _validate_service_binding(ready, paths)
                return ready, True
            time.sleep(0.25)
        headless._terminate_invocation(process)
        raise QueryError("Ghidra service did not become ready within %.1fs; log=%s" %
                         (args.startup_timeout, paths["log"]))


def _ensure_service(args, paths):
    """Return (status, cold_started) for a service ready to take a request."""
    state, status = service_state(paths)
    if state == "stopped" and args.no_start:
        raise QueryError("Ghidra service is not running (--no-start)")
    # A tracked process without a socket is still cold-starting, and its
    # starter holds the startup lock until the socket is ready: wait there
    # rather than connecting to a socket that does not exist yet.
    if state == "stopped" or (state == "busy" and not paths["socket"].exists()):
        return start_service(args, paths)
    return status, False


def _request(verb, args, query, transport):
    return {
        "schema": REQUEST_SCHEMA,
        "verb": verb,
        "request_id": str(uuid.uuid4()),
        "query": query,
        "run": {
            "client": "ghidra_query.py",
            "client_version": CLIENT_VERSION,
            "client_pid": os.getpid(),
            "provenance": args.provenance,
            "transport": transport,
        },
    }


def _validate_query_response(response, request, paths):
    if response.get("schema") != RESPONSE_SCHEMA:
        raise QueryError("unexpected response schema: %r" % response.get("schema"))
    if response.get("request_id") != request["request_id"]:
        raise QueryError("response request_id mismatch")
    if response.get("verb") != request["verb"]:
        raise QueryError("response verb mismatch")
    for field in ("program", "tool", "run", "limits", "result"):
        if not isinstance(response.get(field), dict):
            raise QueryError("response.%s must be an object" % field)
    sha = response["program"].get("sha256")
    if not isinstance(sha, str) or len(sha) != 64:
        raise QueryError("response has no valid program SHA-256")
    result = response["result"]
    for field in ("total_count", "returned_count", "top_k"):
        if not isinstance(result.get(field), int) or isinstance(result.get(field), bool):
            raise QueryError("response.result.%s must be an integer" % field)
    if response["run"].get("service_binding_id") != paths["binding"]["id"]:
        raise QueryError("query response service binding mismatch")
    if not isinstance(result.get("truncated"), bool):
        raise QueryError("response.result.truncated must be boolean")
    if result["returned_count"] > result["top_k"]:
        raise QueryError("response returned_count exceeds top_k")
    if result["truncated"] != (result["total_count"] > result["returned_count"] or
                               bool(result.get("content_truncated", False))):
        raise QueryError("response truncated flag is inconsistent")
    return response


def _service_payload(status, paths, extra=None):
    value = {
        "schema": SERVICE_SCHEMA,
        "status": status,
        "socket": str(paths["socket"]),
        "pid_file": str(paths["pid"]),
        "log": str(paths["log"]),
        "binding": paths["binding"],
        "tool": {"name": "ghidra_query.py", "version": CLIENT_VERSION},
    }
    if extra:
        value.update(extra)
    return value


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analyze-headless", required=True,
                        help="path to Ghidra support/analyzeHeadless")
    parser.add_argument("--java-home", help="optional JDK home")
    parser.add_argument("--project-dir", required=True,
                        help="project-location directory outside this checkout")
    parser.add_argument("--project-name", required=True)
    parser.add_argument("--program", required=True,
                        help="exact Ghidra program name")
    parser.add_argument("--state-dir", required=True,
                        help="base for namespaced socket/log/artifact state outside this checkout")
    parser.add_argument("--startup-timeout", type=float, default=180.0)
    parser.add_argument("--provenance", default="interactive-cli")
    parser.add_argument("--no-start", action="store_true",
                        help="fail rather than cold-starting Ghidra")
    sub = parser.add_subparsers(dest="verb", required=True)

    decompile = sub.add_parser("ghidra.decompile")
    decompile.add_argument("target")

    graph = sub.add_parser("ghidra.call_graph")
    graph.add_argument("target")
    graph.add_argument("--depth", type=int, default=DEFAULT_DEPTH)
    graph.add_argument("--top", type=int, default=DEFAULT_TOP)

    xrefs = sub.add_parser("ghidra.xrefs")
    xrefs.add_argument("target")
    xrefs.add_argument("--top", type=int, default=DEFAULT_TOP)

    search = sub.add_parser("ghidra.search")
    search.add_argument("pattern")
    search.add_argument("--top", type=int, default=DEFAULT_TOP)

    sub.add_parser("service.start")
    sub.add_parser("service.status")
    sub.add_parser("service.stop")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        ensure_supported_platform()
        if args.startup_timeout <= 0 or args.startup_timeout > 900:
            raise QueryError("--startup-timeout must be in (0, 900]")
        paths = default_paths(args.state_dir, args.project_dir, args.project_name,
                              args.program)
        if args.verb == "service.status":
            state, status = service_state(paths)
            value = _service_payload(state, paths,
                                     {"service": status} if status else None)
            print(json.dumps(value, indent=2, sort_keys=True))
            return 0 if state != "stopped" else 1
        if args.verb == "service.start":
            status, started = start_service(args, paths)
            state = status.get("status", "ready")
            print(json.dumps(_service_payload(state, paths, {
                "cold_started": started, "service": status}), indent=2, sort_keys=True))
            return 0
        if args.verb == "service.stop":
            state, status = service_state(paths)
            if state == "stopped":
                print(json.dumps(_service_payload("stopped", paths), indent=2, sort_keys=True))
                return 0
            request = _request("service.stop", args, {}, "persistent-service")
            response = exchange(paths["socket"], request, 130.0)
            if (response.get("schema") != SERVICE_SCHEMA or
                    response.get("request_id") != request["request_id"] or
                    response.get("status") != "stopping"):
                raise QueryError("invalid service.stop response")
            _validate_service_binding(response, paths)
            pid = _tracked_process(paths)
            if pid is not None:
                deadline = time.monotonic() + 15.0
                while time.monotonic() < deadline and _tracked_process(paths) is not None:
                    time.sleep(0.1)
            remaining = _tracked_process(paths)
            if remaining is not None:
                raise QueryError("Ghidra service acknowledged stop but pid %d is still running" %
                                 remaining)
            try:
                paths["pid"].unlink()
            except FileNotFoundError:
                pass
            print(json.dumps(response, indent=2, sort_keys=True))
            return 0

        top = getattr(args, "top", 1)
        if top < 1 or top > MAX_TOP:
            raise QueryError("--top must be in [1, %d]" % MAX_TOP)
        depth = getattr(args, "depth", None)
        if depth is not None and (depth < 1 or depth > MAX_DEPTH):
            raise QueryError("--depth must be in [1, %d]" % MAX_DEPTH)

        status, started = _ensure_service(args, paths)
        # A busy serial daemon is healthy: send the real request and let it
        # wait in the Unix-socket backlog instead of racing a cold start.
        query = {"top_k": top}
        if args.verb == "ghidra.search":
            query["pattern"] = args.pattern
        else:
            query["target"] = args.target
        if depth is not None:
            query["depth"] = depth
        transport = "cold-start" if started else "persistent-service"
        request = _request(args.verb, args, query, transport)
        response = exchange(paths["socket"], request)
        _validate_query_response(response, request, paths)
        print(json.dumps(response, indent=2, sort_keys=True))
        return 0 if response.get("ok") else 2
    except QueryError as exc:
        error = {
            "schema": "i76-ghidra-query-cli-error-v1",
            "ok": False,
            "error": {"type": exc.__class__.__name__, "message": str(exc)},
            "tool": {"name": "ghidra_query.py", "version": CLIENT_VERSION},
        }
        print(json.dumps(error, indent=2, sort_keys=True))
        return 2


if __name__ == "__main__":
    sys.exit(main())
