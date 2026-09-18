#!/usr/bin/env python3
"""Asset-free portability, lifecycle, and failure-contract tests."""

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import socket
import stat
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


headless = load_module("portable_headless", ROOT / "tools/ghidra/headless.py")
query = load_module("portable_query", ROOT / "tools/ghidra_query.py")
PROGRAM_SHA = "a" * 64


class MockUnixService:
    def __init__(self, path: Path, handler, connections: int):
        self.path = path
        self.handler = handler
        self.connections = connections
        self.requests = []
        self.error = None
        self.ready = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            server.bind(str(self.path))
            server.listen(4)
            self.ready.set()
            for _ in range(self.connections):
                client, _ = server.accept()
                with client:
                    raw = b""
                    while not raw.endswith(b"\n"):
                        chunk = client.recv(65536)
                        if not chunk:
                            break
                        raw += chunk
                    request = json.loads(raw)
                    self.requests.append(request)
                    response = self.handler(request)
                    client.sendall((json.dumps(response) + "\n").encode())
        except Exception as exc:  # surfaced in __exit__
            self.error = exc
            self.ready.set()
        finally:
            server.close()
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass

    def __enter__(self):
        self.thread.start()
        if not self.ready.wait(2):
            raise AssertionError("mock service did not start")
        if self.error:
            raise self.error
        return self

    def __exit__(self, *_args):
        self.thread.join(2)
        if self.error:
            raise self.error
        if self.thread.is_alive():
            raise AssertionError("mock service did not stop")


def service_ping(request, binding_id, program="sample.bin", status="ready"):
    return {
        "schema": query.SERVICE_SCHEMA,
        "status": status,
        "request_id": request["request_id"],
        "program": {"name": program, "sha256": PROGRAM_SHA},
        "binding": {"id": binding_id},
        "tool": {"name": "Ghidra", "version": "11.4.2", "service_version": "1"},
        "run": {"service_pid": 123, "project_mode": "read-only"},
    }


def query_response(request, binding_id, inconsistent=False):
    total = 2 if inconsistent else 1
    return {
        "schema": query.RESPONSE_SCHEMA,
        "ok": True,
        "verb": request["verb"],
        "request_id": request["request_id"],
        "claim_boundary": "locator only",
        "program": {"name": "sample.bin", "sha256": PROGRAM_SHA,
                    "image_base": "0x00400000"},
        "tool": {"name": "Ghidra", "version": "11.4.2"},
        "run": {"project_mode": "read-only", "client": request["run"],
                "service_binding_id": binding_id},
        "limits": {"max_top_k": 100},
        "result": {"total_count": total, "returned_count": 1, "top_k": 1,
                   "truncated": False, "content_truncated": False,
                   "artifact": None,
                   "items": [{"name": "entry", "entry": "0x00401000",
                              "pseudocode": "return 0;"}]},
    }


class PortableLauncherTests(unittest.TestCase):
    def make_fake_install(self, root: Path, marker=True):
        fake = root / "Ghidra install" / "support" / "analyzeHeadless"
        fake.parent.mkdir(parents=True)
        marker_code = (
            "script = sys.argv[sys.argv.index('-postScript') + 1]\n"
            "name = 'import-analyze' if script == 'launch_success.py' else script\n"
            "print('I76_GHIDRA_SCRIPT_OK: ' + name)\n"
            if marker else "print('synthetic Ghidra diagnostic: post-script failed')\n"
        )
        fake.write_text(
            "#!/usr/bin/env python3\n"
            "import json, os, sys\n"
            "with open(os.environ['CAPTURE'], 'w') as f:\n"
            " json.dump({'argv': sys.argv[1:], 'path': os.environ.get('PATH')}, f)\n" +
            marker_code
        )
        fake.chmod(0o755)
        return fake

    def query_argv(self, fake: Path, project: Path, program="sample program.bin"):
        return [
            "query", "--analyze-headless", str(fake),
            "--project-dir", str(project), "--project-name", "Own Project",
            "--program=" + program, "decomp_at.py", "--", "0x401000",
        ]

    def test_query_uses_argv_forces_read_only_and_requires_success_marker(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake = self.make_fake_install(root)
            project = root / "project with spaces"
            project.mkdir()
            (project / "Own Project.gpr").write_text("")
            capture = root / "capture.json"
            hostile = "0x401000;touch SHOULD_NOT_EXIST"
            argv = self.query_argv(fake, project)[:-1] + [hostile]
            with mock.patch.dict(os.environ, {"CAPTURE": str(capture)}, clear=False):
                rc = headless.run(argv)
            self.assertEqual(rc, 0)
            captured_argv = json.loads(capture.read_text())["argv"]
            self.assertEqual(captured_argv[:2], [str(project.resolve()), "Own Project"])
            self.assertIn("-readOnly", captured_argv)
            self.assertIn("-noanalysis", captured_argv)
            self.assertEqual(captured_argv[captured_argv.index("-process") + 1],
                             "sample program.bin")
            self.assertEqual(captured_argv[-1], hostile)
            self.assertFalse((root / "SHOULD_NOT_EXIST").exists())

    def test_exit_zero_without_success_marker_is_failure_and_keeps_diagnostics(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake = self.make_fake_install(root, marker=False)
            project = root / "project"
            project.mkdir()
            (project / "Own Project.gpr").write_text("")
            capture = root / "capture.json"
            stdout, stderr = io.StringIO(), io.StringIO()
            with mock.patch.dict(os.environ, {"CAPTURE": str(capture)}, clear=False), \
                    contextlib.redirect_stdout(stdout), contextlib.redirect_stderr(stderr):
                rc = headless.run(self.query_argv(fake, project))
            self.assertEqual(rc, 2)
            self.assertIn("synthetic Ghidra diagnostic", stdout.getvalue())
            self.assertIn("exited 0 without a script success marker", stderr.getvalue())

    def test_import_refuses_gpr_or_partial_rep(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake = self.make_fake_install(root)
            project = root / "new-project-parent"
            project.mkdir()
            binary = root / "self-authored tiny.elf"
            binary.write_bytes(b"not executed")
            capture = root / "capture.json"
            with mock.patch.dict(os.environ, {"CAPTURE": str(capture)}, clear=False):
                rc = headless.run([
                    "import-analyze", "--analyze-headless", str(fake),
                    "--project-dir", str(project), "--project-name", "Synthetic",
                    "--binary", str(binary), "--analysis-timeout", "30",
                ])
            self.assertEqual(rc, 0)
            captured_argv = json.loads(capture.read_text())["argv"]
            self.assertIn("-import", captured_argv)
            self.assertNotIn("-readOnly", captured_argv)
            self.assertNotIn("-process", captured_argv)
            self.assertEqual(captured_argv[captured_argv.index("-import") + 1],
                             str(binary.resolve()))
            self.assertEqual(captured_argv[-3:],
                             ["-postScript", "launch_success.py", "import-analyze"])

            for project_name, residue in (("Existing", ".gpr"), ("Partial", ".rep")):
                path = project / (project_name + residue)
                path.mkdir() if residue == ".rep" else path.write_text("")
                stderr = io.StringIO()
                with contextlib.redirect_stderr(stderr):
                    rc = headless.run([
                        "import-analyze", "--analyze-headless", str(fake),
                        "--project-dir", str(project), "--project-name", project_name,
                        "--binary", str(binary),
                    ])
                self.assertEqual(rc, 2)
                self.assertIn("existing or partial project", stderr.getvalue())

    def test_rejects_ghidra_option_and_wildcard_tokens(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake = self.make_fake_install(root)
            project = root / "project"
            project.mkdir()
            (project / "Own Project.gpr").write_text("")
            for program in ("*", "sample?", "[ab]", "-deleteProject"):
                stderr = io.StringIO()
                with contextlib.redirect_stderr(stderr):
                    rc = headless.run(self.query_argv(fake, project, program))
                self.assertEqual(rc, 2, program)
                self.assertIn("exact literal selector", stderr.getvalue())
            for script_arg in ("-deleteProject", "-postScript"):
                stderr = io.StringIO()
                argv = self.query_argv(fake, project)[:-1] + [script_arg]
                with contextlib.redirect_stderr(stderr):
                    rc = headless.run(argv)
                self.assertEqual(rc, 2, script_arg)
                self.assertIn("must not begin '-'", stderr.getvalue())

    def test_literal_project_names(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake = self.make_fake_install(root)
            project = root / "project"
            project.mkdir()
            for name in (".", "..", "-deleteProject", "wild*card", "a/b"):
                stderr = io.StringIO()
                with contextlib.redirect_stderr(stderr):
                    rc = headless.run([
                        "query", "--analyze-headless", str(fake),
                        "--project-dir", str(project), "--project-name=" + name,
                        "--program", "sample.bin", "decomp_at.py", "0x1",
                    ])
                self.assertEqual(rc, 2, name)
                self.assertIn("literal basename", stderr.getvalue())

    def test_project_inside_package_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            fake = self.make_fake_install(Path(temporary))
            stderr = io.StringIO()
            with contextlib.redirect_stderr(stderr):
                rc = headless.run([
                    "query", "--analyze-headless", str(fake),
                    "--project-dir", str(ROOT), "--project-name", "Nope",
                    "--program", "sample.bin", "decomp_at.py", "0x1",
                ])
            self.assertEqual(rc, 2)
            self.assertIn("outside the source checkout", stderr.getvalue())

    def test_timeout_kills_only_its_invocation_group(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake = root / "analyzeHeadless"
            fake.write_text(
                "#!/usr/bin/env python3\n"
                "import time\n"
                "print('timeout diagnostic', flush=True)\n"
                "time.sleep(30)\n"
            )
            fake.chmod(0o755)
            project = root / "project"
            project.mkdir()
            (project / "Own Project.gpr").write_text("")
            unrelated = subprocess.Popen(["sleep", "30"], start_new_session=True)
            try:
                stderr = io.StringIO()
                started = time.monotonic()
                with contextlib.redirect_stderr(stderr):
                    rc = headless.run([
                        "query", "--analyze-headless", str(fake),
                        "--project-dir", str(project), "--project-name", "Own Project",
                        "--program", "sample.bin", "--timeout", "1",
                        "decomp_at.py", "0x1",
                    ])
                self.assertEqual(rc, 124)
                self.assertLess(time.monotonic() - started, 8)
                self.assertIsNone(unrelated.poll())
                self.assertIn("exceeded 1s", stderr.getvalue())
            finally:
                unrelated.terminate()
                unrelated.wait()


class PersistentClientTests(unittest.TestCase):
    def common(self, root: Path, directory="project", project_name="Synthetic",
               program="sample.bin"):
        project = root / directory
        project.mkdir()
        (project / (project_name + ".gpr")).write_text("")
        fake = root / (directory + "-analyzeHeadless")
        fake.write_text("#!/bin/sh\nexit 99\n")
        fake.chmod(0o755)
        state = root / "state"
        argv = [
            "--analyze-headless", str(fake), "--project-dir", str(project),
            "--project-name", project_name, "--program", program,
            "--state-dir", str(state), "--no-start",
        ]
        paths = query.default_paths(state, project, project_name, program)
        return argv, paths

    def test_decompile_contract_over_namespaced_unix_socket(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            argv, paths = self.common(root)
            binding_id = paths["binding"]["id"]

            def handler(request):
                if request["verb"] == "service.ping":
                    return service_ping(request, binding_id)
                return query_response(request, binding_id)

            stdout = io.StringIO()
            with MockUnixService(paths["socket"], handler, 2) as service:
                with contextlib.redirect_stdout(stdout):
                    rc = query.main(argv + [
                        "--provenance", "unit-test", "ghidra.decompile", "entry",
                    ])
            self.assertEqual(rc, 0)
            payload = json.loads(stdout.getvalue())
            self.assertEqual(payload["program"]["sha256"], PROGRAM_SHA)
            self.assertEqual(payload["run"]["project_mode"], "read-only")
            self.assertEqual([item["verb"] for item in service.requests],
                             ["service.ping", "ghidra.decompile"])

    def test_bad_truncation_flag(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            argv, paths = self.common(root)
            binding_id = paths["binding"]["id"]

            def handler(request):
                if request["verb"] == "service.ping":
                    return service_ping(request, binding_id)
                return query_response(request, binding_id, True)

            stdout = io.StringIO()
            with MockUnixService(paths["socket"], handler, 2):
                with contextlib.redirect_stdout(stdout):
                    rc = query.main(argv + ["ghidra.decompile", "entry"])
            self.assertEqual(rc, 2)
            self.assertIn("truncated flag is inconsistent", stdout.getvalue())

    def test_same_program_name_in_different_projects_has_distinct_namespace(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _, first = self.common(root, "project-a")
            _, second = self.common(root, "project-b")
            self.assertNotEqual(first["binding"]["id"], second["binding"]["id"])
            self.assertNotEqual(first["root"], second["root"])
            self.assertNotEqual(first["socket"], second["socket"])

    def test_same_name_different_project_stop_cannot_contact_first_service(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _, first = self.common(root, "project-a")
            second_argv, second = self.common(root, "project-b")
            first_binding = first["binding"]["id"]

            def handler(request):
                return service_ping(request, first_binding)

            stdout = io.StringIO()
            with MockUnixService(first["socket"], handler, 1) as service:
                with contextlib.redirect_stdout(stdout):
                    rc = query.main(second_argv + ["service.stop"])
                self.assertEqual(rc, 0)
                self.assertEqual(json.loads(stdout.getvalue())["status"], "stopped")
                self.assertEqual(service.requests, [])
                self.assertIsNotNone(query.ping(first["socket"]))
            self.assertNotEqual(first["socket"], second["socket"])

    def test_existing_general_state_base_is_not_chmodded(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project = root / "project"
            project.mkdir()
            state = root / "general-cache"
            state.mkdir(mode=0o755)
            state.chmod(0o755)
            paths = query.default_paths(state, project, "Synthetic", "sample.bin")
            self.assertEqual(stat.S_IMODE(state.stat().st_mode), 0o755)
            self.assertEqual(stat.S_IMODE(paths["root"].stat().st_mode), 0o700)
            self.assertEqual(stat.S_IMODE((paths["root"] / "binding.json").stat().st_mode),
                             0o600)

    def test_group_writable_state_base_is_rejected_without_chmod(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project = root / "project"
            project.mkdir()
            state = root / "shared-cache"
            state.mkdir()
            state.chmod(0o770)
            with self.assertRaisesRegex(query.QueryError, "must not be group/other writable"):
                query.default_paths(state, project, "Synthetic", "sample.bin")
            self.assertEqual(stat.S_IMODE(state.stat().st_mode), 0o770)

    def test_busy_service_is_reused_only_in_exact_namespace(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            argv, paths = self.common(root)
            argv.remove("--no-start")
            argv.append("service.start")
            args = query.build_parser().parse_args(argv)
            busy = {"status": "busy", "service_pid": 42,
                    "binding": paths["binding"]}
            with mock.patch.object(query, "service_state", return_value=("busy", busy)), \
                    mock.patch.object(query.subprocess, "Popen") as popen:
                status, started = query.start_service(args, paths)
            self.assertFalse(started)
            self.assertEqual(status, busy)
            popen.assert_not_called()

    def test_tracked_busy_process_binding_mismatch_fails_closed(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            _, paths = self.common(root)
            paths["pid"].write_text(str(os.getpid()))
            command = (b"analyzeHeadless query_service.py " +
                       str(paths["socket"]).encode() + b" wrong-binding")
            with mock.patch.object(query, "ping", return_value=None), \
                    mock.patch.object(Path, "read_bytes", return_value=command):
                with self.assertRaisesRegex(query.QueryError, "binding mismatch"):
                    query.service_state(paths)

    def test_ready_mismatch_blocks_query_and_stop(self):
        for verb in (("ghidra.decompile", "entry"), ("service.stop",)):
            with self.subTest(verb=verb), tempfile.TemporaryDirectory() as temporary:
                root = Path(temporary)
                argv, paths = self.common(root)

                def handler(request):
                    return service_ping(request, "b" * 64)

                stdout = io.StringIO()
                with MockUnixService(paths["socket"], handler, 1) as service:
                    with contextlib.redirect_stdout(stdout):
                        rc = query.main(argv + list(verb))
                self.assertEqual(rc, 2)
                self.assertIn("binding mismatch", stdout.getvalue())
                self.assertEqual([item["verb"] for item in service.requests],
                                 ["service.ping"])

    def test_stop_validates_binding_before_lifecycle_cleanup(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            argv, paths = self.common(root)
            binding_id = paths["binding"]["id"]

            def handler(request):
                if request["verb"] == "service.ping":
                    return service_ping(request, binding_id)
                return service_ping(request, binding_id, status="stopping")

            stdout = io.StringIO()
            with MockUnixService(paths["socket"], handler, 2) as service:
                with contextlib.redirect_stdout(stdout):
                    rc = query.main(argv + ["service.stop"])
            self.assertEqual(rc, 0)
            self.assertEqual([item["verb"] for item in service.requests],
                             ["service.ping", "service.stop"])

    def test_stop_fails_if_acknowledged_daemon_remains_alive(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            argv, paths = self.common(root)
            binding_id = paths["binding"]["id"]

            def handler(request):
                if request["verb"] == "service.ping":
                    return service_ping(request, binding_id)
                return service_ping(request, binding_id, status="stopping")

            stdout = io.StringIO()
            with MockUnixService(paths["socket"], handler, 2), \
                    mock.patch.object(query, "_tracked_process", return_value=42), \
                    mock.patch.object(query.time, "monotonic", side_effect=[0.0, 16.0]), \
                    contextlib.redirect_stdout(stdout):
                rc = query.main(argv + ["service.stop"])
            self.assertEqual(rc, 2)
            self.assertIn("acknowledged stop but pid 42 is still running", stdout.getvalue())

    def test_cold_start_command_is_explicit_bound_and_read_only(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            argv, paths = self.common(root)
            argv.remove("--no-start")
            argv.append("service.start")
            args = query.build_parser().parse_args(argv)
            captured = {}

            class FakeProcess:
                pid = 4242

                @staticmethod
                def poll():
                    return None

            def fake_popen(command, **kwargs):
                captured["command"] = command
                captured["kwargs"] = kwargs
                return FakeProcess()

            ready = service_ping({"request_id": "unused"}, paths["binding"]["id"])
            with mock.patch.object(query, "service_state", return_value=("stopped", None)), \
                    mock.patch.object(query, "ping", return_value=ready), \
                    mock.patch.object(query.subprocess, "Popen", side_effect=fake_popen):
                status, started = query.start_service(args, paths)
            self.assertTrue(started)
            self.assertEqual(status["status"], "ready")
            command = captured["command"]
            self.assertEqual(command[:3], [str(Path(args.analyze_headless).resolve()),
                                           str(Path(args.project_dir).resolve()),
                                           "Synthetic"])
            self.assertIn("-readOnly", command)
            self.assertIn("-noanalysis", command)
            self.assertEqual(command[command.index("-process") + 1], "sample.bin")
            self.assertEqual(command[-1], paths["binding"]["id"])
            self.assertNotIn("shell", captured["kwargs"])

    def test_persistent_program_validation_and_linux_boundary(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            project = root / "project"
            project.mkdir()
            (project / "Synthetic.gpr").write_text("")
            fake = root / "analyzeHeadless"
            fake.write_text("#!/bin/sh\nexit 1\n")
            fake.chmod(0o755)
            base = [
                "--analyze-headless", str(fake), "--project-dir", str(project),
                "--project-name", "Synthetic", "--state-dir", str(root / "state"),
                "--no-start",
            ]
            for program in ("*", "-postScript"):
                stdout = io.StringIO()
                args = base + ["--program=" + program, "service.status"]
                with contextlib.redirect_stdout(stdout):
                    rc = query.main(args)
                self.assertEqual(rc, 2)
                self.assertIn("exact literal selector", stdout.getvalue())
            stdout = io.StringIO()
            with mock.patch.object(query.sys, "platform", "darwin"), \
                    contextlib.redirect_stdout(stdout):
                rc = query.main(base + ["--program", "sample.bin", "service.status"])
            self.assertEqual(rc, 2)
            self.assertIn("only on Linux with /proc", stdout.getvalue())

    def test_service_source_has_no_mutation_surface_or_socket_override(self):
        source = (ROOT / "tools/ghidra/query_service.py").read_text()
        for token in ("startTransaction", "setName(", "createLabel(",
                      "setComment(", "DisassembleCommand", "CreateFunctionCmd"):
            self.assertNotIn(token, source)
        self.assertNotIn("ServerSocket(", source)
        client = (ROOT / "tools/ghidra_query.py").read_text()
        self.assertIn('"-readOnly"', client)
        self.assertIn('"-noanalysis"', client)
        self.assertNotIn('add_argument("--socket"', client)
        self.assertNotIn("~/projects/" + "interstate76", client)
        self.assertNotIn('default="' + 'nitro.exe"', client)
        lookup = (ROOT / "tools/ghidra/function_lookup.py").read_text()
        self.assertNotIn("getFunctionBefore", lookup)
        self.assertNotIn("getFunctionAfter", lookup)


if __name__ == "__main__":
    unittest.main(verbosity=2)
