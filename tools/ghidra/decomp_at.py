# decomp_at.py -- decompile the existing function containing an address.
# Derived from i76-web's MIT-licensed Ghidra toolkit (manthedan, 2026).
from ghidra.app.decompiler import DecompInterface

MAX_CHARS = 1024 * 1024
args = getScriptArgs()
if len(args) != 1:
    raise RuntimeError("usage: decomp_at.py ADDRESS")

prog = currentProgram
listing = prog.getListing()
space = prog.getAddressFactory().getDefaultAddressSpace()
addr = space.getAddress(args[0])
if addr is None or not prog.getMemory().contains(addr):
    raise RuntimeError("address is outside program memory: %s" % args[0])
fn = listing.getFunctionContaining(addr)
if fn is None:
    raise RuntimeError("no existing function contains %s" % args[0])

print("FUNCTION: %s @ %s" % (fn.getName(), fn.getEntryPoint()))
decomp = DecompInterface()
try:
    if not decomp.openProgram(prog):
        raise RuntimeError("decompiler could not open current program")
    res = decomp.decompileFunction(fn, 120, monitor)
    if not res.decompileCompleted():
        raise RuntimeError("decompile failed: %s" % res.getErrorMessage())
    text = res.getDecompiledFunction().getC()
    print(text[:MAX_CHARS])
    if len(text) > MAX_CHARS:
        print("[TRUNCATED: %d of %d characters returned]" % (MAX_CHARS, len(text)))
finally:
    decomp.dispose()
print("I76_GHIDRA_SCRIPT_OK: decomp_at.py")
