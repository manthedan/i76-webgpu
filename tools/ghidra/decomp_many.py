# decomp_many.py -- decompile several existing functions in one launch.
# Derived from i76-web's MIT-licensed Ghidra toolkit (manthedan, 2026).
from ghidra.app.decompiler import DecompInterface

MAX_ADDRESSES = 64
MAX_CHARS_PER_FUNCTION = 256 * 1024
args = getScriptArgs()
if not args or len(args) > MAX_ADDRESSES:
    raise RuntimeError("usage: decomp_many.py ADDRESS... (1..%d)" % MAX_ADDRESSES)

prog = currentProgram
listing = prog.getListing()
space = prog.getAddressFactory().getDefaultAddressSpace()
decomp = DecompInterface()
try:
    if not decomp.openProgram(prog):
        raise RuntimeError("decompiler could not open current program")
    for addr_s in args:
        addr = space.getAddress(addr_s)
        if addr is None or not prog.getMemory().contains(addr):
            print("===== FUNCTION <none> @ %s =====" % addr_s)
            print("ADDRESS OUTSIDE PROGRAM MEMORY")
            print("===== END =====")
            continue
        fn = listing.getFunctionContaining(addr)
        if fn is None:
            print("===== FUNCTION <none> @ %s =====" % addr_s)
            print("NO EXISTING FUNCTION CONTAINING %s" % addr_s)
            print("===== END =====")
            continue
        print("===== FUNCTION %s @ %s =====" % (fn.getName(), fn.getEntryPoint()))
        res = decomp.decompileFunction(fn, 120, monitor)
        if res.decompileCompleted():
            text = res.getDecompiledFunction().getC()
            print(text[:MAX_CHARS_PER_FUNCTION])
            if len(text) > MAX_CHARS_PER_FUNCTION:
                print("[TRUNCATED: %d of %d characters returned]" %
                      (MAX_CHARS_PER_FUNCTION, len(text)))
        else:
            print("DECOMPILE FAILED: %s" % res.getErrorMessage())
        print("===== END =====")
finally:
    decomp.dispose()
print("I76_GHIDRA_SCRIPT_OK: decomp_many.py")
