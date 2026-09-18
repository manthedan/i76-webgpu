# launch_success.py -- prove a successful explicit import/analyze post-script run.
# This marker lets the host wrapper distinguish Ghidra's exit-0 script failures.
args = getScriptArgs()
if len(args) != 1 or args[0] != "import-analyze":
    raise RuntimeError("usage: launch_success.py import-analyze")
if currentProgram is None:
    raise RuntimeError("import completed without a current program")
sha256 = currentProgram.getExecutableSHA256()
if sha256 is None or len(str(sha256)) != 64:
    raise RuntimeError("imported program has no executable SHA-256")
print("I76_GHIDRA_SCRIPT_OK: import-analyze")
