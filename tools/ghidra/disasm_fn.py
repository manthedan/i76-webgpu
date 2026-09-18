# disasm_fn.py -- bounded disassembly of an existing function.
# It does not force disassembly or create functions.
# Derived from i76-web's MIT-licensed Ghidra toolkit (manthedan, 2026).
MAX_INSTRUCTIONS = 50000
args = getScriptArgs()
if len(args) not in (1, 2):
    raise RuntimeError("usage: disasm_fn.py ADDRESS [MAX_INSTRUCTIONS]")
limit = int(args[1]) if len(args) == 2 else 10000
if limit < 1 or limit > MAX_INSTRUCTIONS:
    raise RuntimeError("max instructions must be in [1, %d]" % MAX_INSTRUCTIONS)

prog = currentProgram
listing = prog.getListing()
space = prog.getAddressFactory().getDefaultAddressSpace()
addr = space.getAddress(args[0])
if addr is None or not prog.getMemory().contains(addr):
    raise RuntimeError("address is outside program memory: %s" % args[0])
fn = listing.getFunctionContaining(addr)
if fn is None:
    raise RuntimeError("no existing function contains %s" % args[0])
print("function: %s @ %s .. %s" %
      (fn.getName(), fn.getEntryPoint(), fn.getBody().getMaxAddress()))
iterator = listing.getInstructions(fn.getBody(), True)
count = 0
while iterator.hasNext() and count < limit:
    instruction = iterator.next()
    print("%s  %s" % (instruction.getAddress(), instruction))
    count += 1
if iterator.hasNext():
    print("[TRUNCATED after %d instructions]" % count)
print("I76_GHIDRA_SCRIPT_OK: disasm_fn.py")
