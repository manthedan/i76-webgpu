# xref_fn.py -- bounded references to an existing function entry.
# Derived from i76-web's MIT-licensed Ghidra toolkit (manthedan, 2026).
MAX_REFS = 10000
args = getScriptArgs()
if len(args) not in (1, 2):
    raise RuntimeError("usage: xref_fn.py ADDRESS [MAX_REFS]")
limit = int(args[1]) if len(args) == 2 else 1000
if limit < 1 or limit > MAX_REFS:
    raise RuntimeError("max refs must be in [1, %d]" % MAX_REFS)

prog = currentProgram
listing = prog.getListing()
space = prog.getAddressFactory().getDefaultAddressSpace()
address = space.getAddress(args[0])
if address is None or not prog.getMemory().contains(address):
    raise RuntimeError("address is outside program memory: %s" % args[0])
fn = listing.getFunctionContaining(address)
if fn is None:
    raise RuntimeError("no existing function contains %s" % args[0])
target = fn.getEntryPoint()
print("xrefs to %s @ %s:" % (fn.getName(), target))
iterator = prog.getReferenceManager().getReferencesTo(target)
count = 0
while iterator.hasNext() and count < limit:
    ref = iterator.next()
    source = ref.getFromAddress()
    caller = listing.getFunctionContaining(source)
    print("%s  caller=%s  type=%s" %
          (source, caller.getName() if caller is not None else "<none>",
           ref.getReferenceType()))
    count += 1
print("returned xrefs: %d%s" %
      (count, " (possibly truncated)" if count == limit and iterator.hasNext() else ""))
print("I76_GHIDRA_SCRIPT_OK: xref_fn.py")
