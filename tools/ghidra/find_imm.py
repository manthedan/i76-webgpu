# find_imm.py -- find existing instructions using a scalar immediate.
# Derived from i76-web's MIT-licensed Ghidra toolkit (manthedan, 2026).
MAX_HITS = 10000
args = getScriptArgs()
if len(args) not in (1, 2):
    raise RuntimeError("usage: find_imm.py INTEGER [MAX_HITS]")
try:
    value = long(args[0], 0)
except ValueError:
    value = long(args[0], 16)
limit = int(args[1]) if len(args) == 2 else 1000
if limit < 1 or limit > MAX_HITS:
    raise RuntimeError("max hits must be in [1, %d]" % MAX_HITS)

prog = currentProgram
listing = prog.getListing()
hits = 0
truncated = False
iterator = listing.getInstructions(True)
while iterator.hasNext():
    instruction = iterator.next()
    matched = False
    for operand in range(instruction.getNumOperands()):
        for obj in instruction.getOpObjects(operand):
            try:
                if obj.getValue() == value:
                    matched = True
                    break
            except AttributeError:
                pass
        if matched:
            break
    if not matched:
        continue
    if hits >= limit:
        truncated = True
        break
    fn = listing.getFunctionContaining(instruction.getAddress())
    name = fn.getName() if fn is not None else "<none>"
    print("HIT %s  %s  (in %s)" % (instruction.getAddress(), instruction, name))
    hits += 1
print("returned immediate hits: %d%s" %
      (hits, " (truncated)" if truncated else ""))
print("I76_GHIDRA_SCRIPT_OK: find_imm.py")
