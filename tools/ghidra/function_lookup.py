# function_lookup.py -- look up existing functions by address or name substring.
# Derived from i76-web fn_before.py/list_fns.py (manthedan, 2026; MIT).
import re

MAX_RESULTS = 1000
args = getScriptArgs()
if len(args) not in (1, 2):
    raise RuntimeError("usage: function_lookup.py NAME_OR_ADDRESS [MAX_RESULTS]")
target = args[0]
limit = int(args[1]) if len(args) == 2 else 50
if limit < 1 or limit > MAX_RESULTS:
    raise RuntimeError("max results must be in [1, %d]" % MAX_RESULTS)

prog = currentProgram
manager = prog.getFunctionManager()
space = prog.getAddressFactory().getDefaultAddressSpace()

address = None
if re.match(r"^(?:0x)?[0-9a-fA-F]+$", target):
    try:
        address = space.getAddress(target)
    except Exception:
        address = None

if address is not None:
    if not prog.getMemory().contains(address):
        raise RuntimeError("address is outside program memory: %s" % target)
    containing = manager.getFunctionContaining(address)
    before = None
    after = None
    iterator = manager.getFunctions(True)
    while iterator.hasNext():
        candidate = iterator.next()
        comparison = candidate.getEntryPoint().compareTo(address)
        if comparison < 0:
            before = candidate
        elif comparison > 0:
            after = candidate
            break
    if containing is not None:
        print("containing: %s @ %s .. %s size=%d" %
              (containing.getName(), containing.getEntryPoint(),
               containing.getBody().getMaxAddress(),
               containing.getBody().getNumAddresses()))
    else:
        print("containing: <none>")
    if before is not None:
        print("before: %s @ %s .. %s size=%d" %
              (before.getName(), before.getEntryPoint(),
               before.getBody().getMaxAddress(), before.getBody().getNumAddresses()))
    if after is not None:
        print("after: %s @ %s .. %s size=%d" %
              (after.getName(), after.getEntryPoint(),
               after.getBody().getMaxAddress(), after.getBody().getNumAddresses()))
else:
    needle = target.lower()
    if not needle:
        raise RuntimeError("function name substring must not be empty")
    returned = 0
    total = 0
    iterator = manager.getFunctions(True)
    while iterator.hasNext():
        fn = iterator.next()
        if needle not in fn.getName().lower():
            continue
        total += 1
        if returned < limit:
            print("%s  %s  size=%d" %
                  (fn.getEntryPoint(), fn.getName(), fn.getBody().getNumAddresses()))
            returned += 1
    print("matching functions: total=%d returned=%d truncated=%s" %
          (total, returned, str(total > returned).lower()))
print("I76_GHIDRA_SCRIPT_OK: function_lookup.py")
