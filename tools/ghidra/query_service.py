# query_service.py -- persistent read-only Ghidra JSON query service.
#
# Started only through tools/ghidra_query.py, which invokes analyzeHeadless with
# an explicit project/program plus -noanalysis -readOnly. The service never
# opens a project transaction and exposes no script/eval/mutation verb.
# Derived from i76-web's MIT-licensed query service (manthedan, 2026).

import hashlib
import json
import os
import re
import time
import traceback
from datetime import datetime
from java.io import BufferedReader, BufferedWriter
from java.net import StandardProtocolFamily, UnixDomainSocketAddress
from java.nio.channels import Channels, ServerSocketChannel
from ghidra.app.decompiler import DecompInterface
from ghidra.framework import Application

REQUEST_SCHEMA = "i76-ghidra-query-request-v1"
RESPONSE_SCHEMA = "i76-ghidra-query-v1"
SERVICE_SCHEMA = "i76-ghidra-query-service-v1"
SERVICE_VERSION = "2"
MAX_REQUEST_CHARS = 65536
MAX_TOP = 100
MAX_DEPTH = 8
MAX_DECOMPILE_CHARS = 256 * 1024
MAX_FULL_ITEMS = 100000
REQUEST_ID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")
BINDING_ID_RE = re.compile(r"^[0-9a-f]{64}$")
NAME_RE = re.compile(r"^[A-Za-z_.$?@][A-Za-z0-9_.$?@:-]*$")
CLAIM_BOUNDARY = ("Locator output only. Address-backed manual review is required "
                  "before drawing a semantic conclusion.")

prog = currentProgram
listing = prog.getListing()
function_manager = prog.getFunctionManager()
symbol_table = prog.getSymbolTable()
reference_manager = prog.getReferenceManager()
space = prog.getAddressFactory().getDefaultAddressSpace()
program_sha256 = prog.getExecutableSHA256()
if program_sha256 is None or not re.match(r"^[0-9A-Fa-f]{64}$", str(program_sha256)):
    raise RuntimeError("query_service.py: current program has no executable SHA-256")
program_sha256 = str(program_sha256).lower()
tool_version = str(Application.getApplicationVersion())
service_started = datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")


def utc_now():
    return datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def address_text(address):
    return "0x%08x" % address.getOffset()


def function_item(fn):
    body = fn.getBody()
    maximum = body.getMaxAddress()
    return {
        "name": fn.getName(),
        "entry": address_text(fn.getEntryPoint()),
        "end_exclusive": "0x%08x" % (maximum.getOffset() + 1),
        "external": bool(fn.isExternal()),
    }


def strict_json(text):
    def pairs(values):
        result = {}
        for key, value in values:
            if key in result:
                raise ValueError("duplicate JSON key: %s" % key)
            result[key] = value
        return result
    return json.loads(text, object_pairs_hook=pairs)


def read_request_line(reader):
    # BufferedReader.readLine() has no size bound and could allocate without
    # limit before the caller checks length. Read at most the wire limit.
    chars = []
    while len(chars) <= MAX_REQUEST_CHARS:
        value = reader.read()
        if value == -1:
            raise ValueError("request ended before newline")
        if value == 10:
            return u"".join(chars)
        chars.append(unichr(value))
    raise ValueError("request exceeds %d characters" % MAX_REQUEST_CHARS)


def all_functions():
    values = []
    iterator = function_manager.getFunctions(True)
    while iterator.hasNext():
        if len(values) >= MAX_FULL_ITEMS:
            raise RuntimeError("function count exceeds service safety bound")
        values.append(iterator.next())
    return values


functions = all_functions()
functions_by_name = {}
for indexed_fn in functions:
    key = indexed_fn.getName().lower()
    functions_by_name.setdefault(key, []).append(indexed_fn)


def resolve_function(target):
    if not isinstance(target, basestring) or not target or len(target) > 256:
        raise ValueError("target must be a non-empty string of at most 256 characters")
    if re.match(r"^(?:0x)?[0-9A-Fa-f]+$", target):
        try:
            address = space.getAddress(target)
        except Exception:
            raise ValueError("invalid address: %s" % target)
        if not prog.getMemory().contains(address):
            raise ValueError("address is outside program memory: %s" % target)
        fn = listing.getFunctionContaining(address)
        if fn is None:
            fn = listing.getFunctionAt(address)
        if fn is None:
            raise ValueError("no function contains %s" % target)
        return fn, address
    if not NAME_RE.match(target):
        raise ValueError("invalid function name: %s" % target)
    candidates = functions_by_name.get(target.lower(), [])
    if not candidates:
        raise ValueError("unknown function name: %s" % target)
    if len(candidates) != 1:
        raise ValueError("ambiguous function name: %s (%d matches)" %
                         (target, len(candidates)))
    return candidates[0], candidates[0].getEntryPoint()


def validate_request(value):
    if not isinstance(value, dict):
        raise ValueError("request must be an object")
    allowed = set(["schema", "verb", "request_id", "query", "run"])
    unknown = set(value.keys()) - allowed
    if unknown:
        raise ValueError("unknown request fields: %s" % ", ".join(sorted(unknown)))
    if value.get("schema") != REQUEST_SCHEMA:
        raise ValueError("unexpected request schema")
    request_id = value.get("request_id")
    if not isinstance(request_id, basestring) or not REQUEST_ID_RE.match(request_id):
        raise ValueError("request_id must be a lowercase UUID")
    if not isinstance(value.get("verb"), basestring):
        raise ValueError("verb must be a string")
    if not isinstance(value.get("query"), dict):
        raise ValueError("query must be an object")
    run = value.get("run")
    if not isinstance(run, dict):
        raise ValueError("run must be an object")
    if not isinstance(run.get("provenance"), basestring) or not run.get("provenance"):
        raise ValueError("run.provenance must be a non-empty string")
    return value


def query_top(query, default_value):
    value = query.get("top_k", default_value)
    if isinstance(value, bool) or not isinstance(value, (int, long)) or value < 1 or value > MAX_TOP:
        raise ValueError("query.top_k must be in [1, %d]" % MAX_TOP)
    return int(value)


def artifact_path(request_id, verb, suffix):
    safe_verb = verb.replace(".", "-")
    return os.path.join(artifact_dir, "%s-%s.%s" % (request_id, safe_verb, suffix))


def write_artifact(request_id, verb, value, suffix="json"):
    path = artifact_path(request_id, verb, suffix)
    temporary = path + ".tmp-%d" % os.getpid()
    if os.path.exists(path) or os.path.exists(temporary):
        raise RuntimeError("refusing to overwrite query artifact")
    if suffix == "json":
        data = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode("utf-8")
        media_type = "application/json"
    else:
        data = value.encode("utf-8")
        media_type = "text/plain; charset=utf-8"
    stream = open(temporary, "wb")
    try:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    finally:
        stream.close()
    os.rename(temporary, path)
    os.chmod(path, 0600)
    return {
        "path": path,
        "sha256": hashlib.sha256(data).hexdigest(),
        "bytes": len(data),
        "media_type": media_type,
    }


def result_envelope(request, top_k, items, total_count=None, artifact=None,
                    content_truncated=False, extra=None):
    if total_count is None:
        total_count = len(items)
    returned = min(len(items), top_k)
    result = {
        "total_count": int(total_count),
        "returned_count": returned,
        "top_k": top_k,
        "truncated": bool(total_count > returned or content_truncated),
        "content_truncated": bool(content_truncated),
        "artifact": artifact,
        "items": items[:top_k],
    }
    if extra:
        result.update(extra)
    return result


def base_response(request, ok, result, started_ns, error=None):
    response = {
        "schema": RESPONSE_SCHEMA,
        "ok": bool(ok),
        "verb": request.get("verb"),
        "request_id": request.get("request_id"),
        "claim_boundary": CLAIM_BOUNDARY,
        "program": {
            "name": prog.getName(),
            "sha256": program_sha256,
            "image_base": address_text(prog.getImageBase()),
        },
        "tool": {
            "name": "Ghidra",
            "version": tool_version,
            "service": "i76-ghidra-query",
            "service_version": SERVICE_VERSION,
        },
        "run": {
            "service_pid": os.getpid(),
            "service_started_utc": service_started,
            "service_binding_id": binding_id,
            "request_started_utc": request.get("_started_utc", utc_now()),
            "elapsed_ms": int((time.time() * 1000000000L - started_ns) / 1000000L),
            "client": request.get("run", {}),
            "project_mode": "read-only",
        },
        "limits": {
            "max_top_k": MAX_TOP,
            "max_depth": MAX_DEPTH,
            "max_decompile_chars": MAX_DECOMPILE_CHARS,
        },
        "result": result,
    }
    if error is not None:
        response["error"] = error
    return response


def decompile_query(request):
    query = request["query"]
    if set(query.keys()) != set(["target", "top_k"]):
        raise ValueError("ghidra.decompile query requires only target and top_k")
    if query_top(query, 1) != 1:
        raise ValueError("ghidra.decompile top_k must be 1")
    fn, requested_address = resolve_function(query.get("target"))
    decompiler = DecompInterface()
    try:
        if not decompiler.openProgram(prog):
            raise RuntimeError("decompiler could not open current program")
        completed = decompiler.decompileFunction(fn, 120, monitor)
        if not completed.decompileCompleted():
            raise RuntimeError("decompile failed: %s" % completed.getErrorMessage())
        pseudocode = completed.getDecompiledFunction().getC()
    finally:
        decompiler.dispose()
    item = function_item(fn)
    item["requested_address"] = address_text(requested_address)
    item["pseudocode"] = pseudocode
    artifact = None
    content_truncated = len(pseudocode) > MAX_DECOMPILE_CHARS
    if content_truncated:
        artifact = write_artifact(request["request_id"], request["verb"], {
            "schema": "i76-ghidra-decompile-artifact-v1",
            "program_sha256": program_sha256,
            "function": function_item(fn),
            "pseudocode": pseudocode,
        })
        item["pseudocode"] = pseudocode[:MAX_DECOMPILE_CHARS]
    return result_envelope(request, 1, [item], 1, artifact, content_truncated,
                           {"resolved": function_item(fn)})


def references_for(fn):
    values = []
    iterator = reference_manager.getReferencesTo(fn.getEntryPoint())
    while iterator.hasNext():
        ref = iterator.next()
        source = ref.getFromAddress()
        caller = listing.getFunctionContaining(source)
        values.append({
            "from": address_text(source),
            "to": address_text(ref.getToAddress()),
            "type": str(ref.getReferenceType()),
            "source_type": str(ref.getSource()),
            "caller": function_item(caller) if caller is not None else None,
        })
        if len(values) > MAX_FULL_ITEMS:
            raise RuntimeError("xref count exceeds service safety bound")
    values.sort(key=lambda item: (long(item["from"], 16), item["type"]))
    return values


def xrefs_query(request):
    query = request["query"]
    if set(query.keys()) != set(["target", "top_k"]):
        raise ValueError("ghidra.xrefs query requires only target and top_k")
    top_k = query_top(query, 50)
    fn, requested_address = resolve_function(query.get("target"))
    items = references_for(fn)
    artifact = None
    if len(items) > top_k:
        artifact = write_artifact(request["request_id"], request["verb"], {
            "schema": "i76-ghidra-list-artifact-v1",
            "program_sha256": program_sha256,
            "verb": request["verb"],
            "items": items,
        })
    return result_envelope(request, top_k, items, len(items), artifact, False, {
        "resolved": function_item(fn),
        "requested_address": address_text(requested_address),
    })


def java_functions(value):
    iterator = value.iterator()
    result = []
    while iterator.hasNext():
        result.append(iterator.next())
    result.sort(key=lambda fn: fn.getEntryPoint().getOffset())
    return result


def call_graph_direction(root, depth, direction):
    seen = set([root.getEntryPoint().getOffset()])
    frontier = [root]
    items = []
    for level in range(1, depth + 1):
        next_frontier = []
        for owner in frontier:
            if direction == "callee":
                neighbours = java_functions(owner.getCalledFunctions(monitor))
            else:
                neighbours = java_functions(owner.getCallingFunctions(monitor))
            for neighbour in neighbours:
                entry = neighbour.getEntryPoint().getOffset()
                edge = {
                    "direction": direction,
                    "depth": level,
                    "from": function_item(owner if direction == "callee" else neighbour),
                    "to": function_item(neighbour if direction == "callee" else owner),
                }
                items.append(edge)
                if len(items) > MAX_FULL_ITEMS:
                    raise RuntimeError("call graph exceeds service safety bound")
                if entry not in seen:
                    seen.add(entry)
                    next_frontier.append(neighbour)
        frontier = next_frontier
        if not frontier:
            break
    return items


def call_graph_query(request):
    query = request["query"]
    if set(query.keys()) != set(["target", "top_k", "depth"]):
        raise ValueError("ghidra.call_graph query requires only target, top_k, and depth")
    top_k = query_top(query, 50)
    depth = query.get("depth")
    if isinstance(depth, bool) or not isinstance(depth, (int, long)) or depth < 1 or depth > MAX_DEPTH:
        raise ValueError("query.depth must be in [1, %d]" % MAX_DEPTH)
    fn, requested_address = resolve_function(query.get("target"))
    items = call_graph_direction(fn, int(depth), "caller")
    items.extend(call_graph_direction(fn, int(depth), "callee"))
    items.sort(key=lambda item: (item["depth"], item["direction"],
                                 long(item["from"]["entry"], 16),
                                 long(item["to"]["entry"], 16)))
    artifact = None
    if len(items) > top_k:
        artifact = write_artifact(request["request_id"], request["verb"], {
            "schema": "i76-ghidra-list-artifact-v1",
            "program_sha256": program_sha256,
            "verb": request["verb"],
            "items": items,
        })
    return result_envelope(request, top_k, items, len(items), artifact, False, {
        "resolved": function_item(fn),
        "requested_address": address_text(requested_address),
        "depth": int(depth),
    })


def search_query(request):
    query = request["query"]
    if set(query.keys()) != set(["pattern", "top_k"]):
        raise ValueError("ghidra.search query requires only pattern and top_k")
    top_k = query_top(query, 50)
    pattern = query.get("pattern")
    if not isinstance(pattern, basestring) or not pattern or len(pattern) > 256:
        raise ValueError("pattern must be a non-empty string of at most 256 characters")
    needle = pattern.lower()
    found = {}
    iterator = symbol_table.getAllSymbols(True)
    while iterator.hasNext():
        symbol = iterator.next()
        name = symbol.getName()
        if needle not in name.lower():
            continue
        address = symbol.getAddress()
        key = (name, address.getOffset(), str(symbol.getSymbolType()))
        found[key] = {
            "name": name,
            "address": address_text(address),
            "kind": str(symbol.getSymbolType()),
            "primary": bool(symbol.isPrimary()),
            "namespace": str(symbol.getParentNamespace()),
        }
        if len(found) > MAX_FULL_ITEMS:
            raise RuntimeError("search results exceed service safety bound")
    items = found.values()
    items.sort(key=lambda item: (item["name"].lower(), long(item["address"], 16), item["kind"]))
    artifact = None
    if len(items) > top_k:
        artifact = write_artifact(request["request_id"], request["verb"], {
            "schema": "i76-ghidra-list-artifact-v1",
            "program_sha256": program_sha256,
            "verb": request["verb"],
            "pattern": pattern,
            "items": items,
        })
    return result_envelope(request, top_k, items, len(items), artifact, False,
                           {"pattern": pattern, "matching": "case-insensitive substring"})


def empty_error_result(request):
    top_k = 1
    query = request.get("query")
    if isinstance(query, dict):
        candidate = query.get("top_k")
        if isinstance(candidate, (int, long)) and not isinstance(candidate, bool):
            top_k = max(1, min(int(candidate), MAX_TOP))
    return result_envelope(request, top_k, [], 0)


def handle_request(raw):
    started_ns = time.time() * 1000000000L
    request = {"verb": None, "request_id": None, "query": {}, "run": {}}
    try:
        request = validate_request(strict_json(raw))
        request["_started_utc"] = utc_now()
        verb = request["verb"]
        if verb == "ghidra.decompile":
            result = decompile_query(request)
        elif verb == "ghidra.call_graph":
            result = call_graph_query(request)
        elif verb == "ghidra.xrefs":
            result = xrefs_query(request)
        elif verb == "ghidra.search":
            result = search_query(request)
        else:
            raise ValueError("unsupported query verb: %s" % verb)
        return base_response(request, True, result, started_ns), False
    except Exception as exc:
        error = {"type": exc.__class__.__name__, "message": str(exc)}
        return base_response(request, False, empty_error_result(request), started_ns, error), False


def service_response(request, status):
    return {
        "schema": SERVICE_SCHEMA,
        "status": status,
        "request_id": request.get("request_id"),
        "program": {"name": prog.getName(), "sha256": program_sha256},
        "binding": {"id": binding_id},
        "tool": {"name": "Ghidra", "version": tool_version,
                 "service_version": SERVICE_VERSION},
        "run": {"service_pid": os.getpid(), "service_started_utc": service_started,
                "project_mode": "read-only"},
    }


def dispatch(raw):
    try:
        request = validate_request(strict_json(raw))
        if request["verb"] == "service.ping":
            if request["query"]:
                raise ValueError("service.ping query must be empty")
            return service_response(request, "ready"), False
        if request["verb"] == "service.stop":
            if request["query"]:
                raise ValueError("service.stop query must be empty")
            return service_response(request, "stopping"), True
    except Exception:
        pass
    return handle_request(raw)


args = getScriptArgs()
if len(args) != 3:
    raise RuntimeError("usage: query_service.py SOCKET_PATH ARTIFACT_DIR BINDING_ID")
socket_path = os.path.abspath(args[0])
artifact_dir = os.path.abspath(args[1])
binding_id = args[2]
if not BINDING_ID_RE.match(binding_id):
    raise RuntimeError("query_service.py: invalid binding ID")
if os.path.exists(socket_path):
    raise RuntimeError("query_service.py: refusing existing socket path %s" % socket_path)
if not os.path.isdir(os.path.dirname(socket_path)):
    raise RuntimeError("query_service.py: socket parent does not exist")
if not os.path.isdir(artifact_dir):
    raise RuntimeError("query_service.py: artifact directory does not exist")

server = ServerSocketChannel.open(StandardProtocolFamily.UNIX)
try:
    server.bind(UnixDomainSocketAddress.of(socket_path))
    os.chmod(socket_path, 0600)
    print("GHIDRA QUERY SERVICE READY: %s pid=%d program=%s sha256=%s" %
          (socket_path, os.getpid(), prog.getName(), program_sha256))
    stopping = False
    while not stopping:
        channel = server.accept()
        reader = None
        writer = None
        try:
            reader = BufferedReader(Channels.newReader(channel, "UTF-8"))
            raw = read_request_line(reader)
            response, stopping = dispatch(raw)
            writer = BufferedWriter(Channels.newWriter(channel, "UTF-8"))
            writer.write(json.dumps(response, sort_keys=True, separators=(",", ":")))
            writer.write("\n")
            writer.flush()
        except Exception as exc:
            print("GHIDRA QUERY SERVICE REQUEST ERROR: %s" % exc)
            traceback.print_exc()
        finally:
            try:
                channel.close()
            except Exception:
                pass
finally:
    try:
        server.close()
    finally:
        if os.path.exists(socket_path):
            os.unlink(socket_path)
print("GHIDRA QUERY SERVICE STOPPED")
