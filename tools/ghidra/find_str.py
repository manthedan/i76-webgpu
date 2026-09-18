# find_str.py -- bounded substring lookup over existing defined strings.
# Derived from i76-web's MIT-licensed Ghidra toolkit (manthedan, 2026).
MAX_HITS = 10000
MAX_REFS_PER_STRING = 1000
args = getScriptArgs()
if len(args) not in (1, 2):
    raise RuntimeError("usage: find_str.py SUBSTRING [MAX_HITS]")
needle = unicode(args[0]).lower()
if not needle:
    raise RuntimeError("substring must not be empty")
limit = int(args[1]) if len(args) == 2 else 100
if limit < 1 or limit > MAX_HITS:
    raise RuntimeError("max hits must be in [1, %d]" % MAX_HITS)

prog = currentProgram
listing = prog.getListing()
refs = prog.getReferenceManager()
hits = 0
truncated = False
for data in listing.getDefinedData(True):
    if not data.hasStringValue():
        continue
    value = data.getValue()
    if value is None or needle not in unicode(value).lower():
        continue
    if hits >= limit:
        truncated = True
        break
    address = data.getAddress()
    users = []
    references_seen = 0
    iterator = refs.getReferencesTo(address)
    while iterator.hasNext() and references_seen < MAX_REFS_PER_STRING:
        ref = iterator.next()
        references_seen += 1
        fn = listing.getFunctionContaining(ref.getFromAddress())
        if fn is not None:
            label = "%s@%s" % (fn.getName(), fn.getEntryPoint())
            if label not in users:
                users.append(label)
    text = unicode(value).encode("utf-8", "replace")[:256]
    print("[%s] %s <- %s" %
          (address, text, ", ".join(users) if users else "no function refs"))
    hits += 1
print("returned string hits: %d%s" %
      (hits, " (truncated)" if truncated else ""))
print("I76_GHIDRA_SCRIPT_OK: find_str.py")
