# export_function_map.py -- export function boundaries and direct call edges.
#
# Derived from i76-web's import_coverage.py (manthedan, 2026; MIT). Despite the
# historical source name, this only reads Ghidra program state and writes a new
# JSON result. It does not import coverage or mutate the project.

import json
from java.io import File, OutputStreamWriter
from java.nio.channels import Channels, FileChannel
from java.nio.file import Files, LinkOption, StandardOpenOption
from ghidra.framework import Application

SCHEMA = "ghidra-function-map-v1"
MAX_FUNCTIONS = 100000
MAX_CALL_EDGES = 500000


def fail(message):
    raise RuntimeError("export_function_map.py: " + message)


def address_text(address):
    return "0x%x" % address.getOffset()


def function_bounds(fn):
    body = fn.getBody()
    minimum = body.getMinAddress()
    maximum = body.getMaxAddress()
    if minimum is None or maximum is None:
        fail("function has an empty body: %s" % fn.getName())
    entry = fn.getEntryPoint().getOffset()
    end = maximum.getOffset() + 1
    if minimum.getOffset() > entry or end <= entry:
        fail("invalid function body bounds: %s" % fn.getName())
    return entry, end


args = getScriptArgs()
if len(args) != 1 or not args[0]:
    fail("usage: export_function_map.py OUTPUT.json")

output = File(args[0]).getCanonicalFile()
if output.exists():
    fail("refusing to overwrite %s" % output)
parent = output.getParentFile()
if parent is None or not parent.isDirectory():
    fail("output parent directory does not exist")


program_sha256 = currentProgram.getExecutableSHA256()
if program_sha256 is None or len(str(program_sha256)) != 64:
    fail("current program has no executable SHA-256")

manager = currentProgram.getFunctionManager()
function_objects = []
iterator = manager.getFunctions(True)
while iterator.hasNext():
    if len(function_objects) >= MAX_FUNCTIONS:
        fail("function count exceeds %d" % MAX_FUNCTIONS)
    function_objects.append(iterator.next())

functions = []
entries = set()
for fn in function_objects:
    entry, end = function_bounds(fn)
    if entry in entries:
        fail("duplicate function entry 0x%x" % entry)
    entries.add(entry)
    functions.append({
        "name": fn.getName(),
        "entry": "0x%x" % entry,
        "end_exclusive": "0x%x" % end,
    })
functions.sort(key=lambda item: int(item["entry"], 16))

calls = set()
for fn in function_objects:
    caller = fn.getEntryPoint().getOffset()
    called = fn.getCalledFunctions(monitor)
    called_iterator = called.iterator()
    while called_iterator.hasNext():
        callee = called_iterator.next().getEntryPoint().getOffset()
        if callee not in entries:
            continue
        edge = (caller, callee)
        calls.add(edge)
        if len(calls) > MAX_CALL_EDGES:
            fail("direct call-edge count exceeds %d" % MAX_CALL_EDGES)

payload = {
    "schema": SCHEMA,
    "program": {
        "name": currentProgram.getName(),
        "executable_sha256": str(program_sha256).lower(),
        "image_base": address_text(currentProgram.getImageBase()),
    },
    "tool": {
        "name": "Ghidra",
        "version": str(Application.getApplicationVersion()),
    },
    "limits": {
        "max_functions": MAX_FUNCTIONS,
        "max_call_edges": MAX_CALL_EDGES,
    },
    "functions": functions,
    "calls": [{"caller": "0x%x" % caller, "callee": "0x%x" % callee}
              for caller, callee in sorted(calls)],
}

# Exclusive, unpredictable temporary file (createTempFile uses O_EXCL), then
# reopened without following links, so a shared output directory cannot
# redirect this write through a planted symlink.
temp_path = Files.createTempFile(parent.toPath(), output.getName() + ".", ".tmp")
temp = temp_path.toFile()
writer = None
try:
    channel = FileChannel.open(temp_path, StandardOpenOption.WRITE,
                               LinkOption.NOFOLLOW_LINKS)
    writer = OutputStreamWriter(Channels.newOutputStream(channel), "UTF-8")
    writer.write(json.dumps(payload, sort_keys=True, indent=2))
    writer.write("\n")
    writer.flush()
    channel.force(True)
    writer.close()
    writer = None
    # renameTo can replace a result created by another invocation after our
    # initial exists check. Same-directory hard-link publication fails on EEXIST.
    Files.createLink(output.toPath(), temp.toPath())
    temp.delete()
except:
    if writer is not None:
        writer.close()
    temp.delete()
    raise

print("FUNCTION MAP: %d functions, %d direct call edges -> %s" %
      (len(functions), len(calls), output))
print("I76_GHIDRA_SCRIPT_OK: export_function_map.py")
