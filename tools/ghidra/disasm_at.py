# disasm_at.py -- print existing instructions around an address.
# It does not force disassembly or create functions.
# Derived from i76-web's MIT-licensed Ghidra toolkit (manthedan, 2026).
args = getScriptArgs()
if len(args) not in (1, 2):
    raise RuntimeError("usage: disasm_at.py ADDRESS [RADIUS_INSTRUCTIONS]")
radius = int(args[1]) if len(args) == 2 else 12
if radius < 1 or radius > 200:
    raise RuntimeError("radius must be in [1, 200]")

prog = currentProgram
listing = prog.getListing()
space = prog.getAddressFactory().getDefaultAddressSpace()
addr = space.getAddress(args[0])
if addr is None or not prog.getMemory().contains(addr):
    raise RuntimeError("address is outside program memory: %s" % args[0])

fn = listing.getFunctionContaining(addr)
print("function: %s" % ("%s @ %s" % (fn.getName(), fn.getEntryPoint())
                         if fn is not None else "<none>"))
current = listing.getInstructionContaining(addr)
if current is None:
    current = listing.getInstructionAfter(addr)
if current is None:
    raise RuntimeError("no existing instruction at or after %s" % args[0])
start = current
for _ in range(radius):
    previous = listing.getInstructionBefore(start.getAddress())
    if previous is None:
        break
    start = previous

shown = 0
instruction = start
while instruction is not None and shown < radius * 2 + 1:
    mark = " <-- TARGET" if instruction.contains(addr) else ""
    print("%s  %s%s" % (instruction.getAddress(), instruction, mark))
    instruction = listing.getInstructionAfter(instruction.getAddress())
    shown += 1
print("I76_GHIDRA_SCRIPT_OK: disasm_at.py")
