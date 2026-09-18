# xref_addr.py -- bounded xrefs to an existing address with function context.
# Derived from i76-web's MIT-licensed Ghidra toolkit (manthedan, 2026).
MAX_REFS = 10000
args = getScriptArgs()
if len(args) not in (1, 2):
    raise RuntimeError("usage: xref_addr.py ADDRESS [MAX_REFS]")
limit = int(args[1]) if len(args) == 2 else 1000
if limit < 1 or limit > MAX_REFS:
    raise RuntimeError("max refs must be in [1, %d]" % MAX_REFS)

prog = currentProgram
listing = prog.getListing()
space = prog.getAddressFactory().getDefaultAddressSpace()
address = space.getAddress(args[0])
if address is None or not prog.getMemory().contains(address):
    raise RuntimeError("address is outside program memory: %s" % args[0])
iterator = prog.getReferenceManager().getReferencesTo(address)
count = 0
while iterator.hasNext() and count < limit:
    ref = iterator.next()
    source = ref.getFromAddress()
    fn = listing.getFunctionContaining(source)
    instruction = listing.getInstructionContaining(source)
    print("%s  in %s: %s  type=%s" %
          (source, fn.getName() if fn is not None else "<none>",
           instruction if instruction is not None else "<no instruction>",
           ref.getReferenceType()))
    count += 1
print("returned xrefs: %d%s" %
      (count, " (possibly truncated)" if count == limit and iterator.hasNext() else ""))
print("I76_GHIDRA_SCRIPT_OK: xref_addr.py")
