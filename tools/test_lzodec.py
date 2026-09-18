#!/usr/bin/env python3
"""test_lzodec.py — focused test for the engine-local LZO1X/1Y decompressor.

Three layers, none needing game assets:

  1. ROUND-TRIP: buffers of several data classes are compressed with the
     system liblzo2 (ctypes) through all four relevant compressors
     (lzo1x_1, lzo1x_999, lzo1y_1, lzo1y_999) and decoded by
     tools/lzodec_cli; output must be byte-identical to the source.

  2. REFERENCE PARITY: for each compressed stream the reference decompressor
     (lzo1x_decompress_safe / lzo1y_decompress) and lzodec_cli must agree.

  3. FAIL-CLOSED: an ASan build of lzodec_cli is fed truncations at every
     length and single-byte corruptions of valid streams plus random junk;
     it must never crash or trigger a sanitizer. Mutations may remain valid;
     rejected streams must not leave an output file.

The system liblzo2 is used here ONLY as a test oracle; nothing shipped
links it. Exit 0 on success, 1 on failure.
"""

import ctypes
import os
import random
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CLI_SRC = ROOT / "tools" / "lzodec_cli.c"
LZODEC_SRC = ROOT / "src" / "engine" / "lzodec.c"


def build(work: Path) -> tuple[Path, Path]:
    plain = work / "lzodec_cli"
    asan = work / "lzodec_cli_asan"
    for out, extra in ((plain, ["-O2"]), (asan, ["-O1", "-g", "-fsanitize=address,undefined"])):
        subprocess.check_call([
            os.environ.get("CC", "cc"), *extra, "-ffp-contract=off",
            f"-I{ROOT}/src", "-o", str(out), str(CLI_SRC), str(LZODEC_SRC),
        ])
    return plain, asan


def lzo_lib():
    lib = ctypes.CDLL("liblzo2.so.2")
    p = ctypes.POINTER(ctypes.c_ulong)
    for fn in ("lzo1x_1_compress", "lzo1x_999_compress",
               "lzo1y_1_compress", "lzo1y_999_compress",
               "lzo1x_decompress_safe", "lzo1y_decompress"):
        f = getattr(lib, fn)
        f.argtypes = [ctypes.c_char_p, ctypes.c_ulong, ctypes.c_char_p, p, ctypes.c_void_p]
        f.restype = ctypes.c_int
    return lib


def compress(lib, fn, data: bytes, wrkmem) -> bytes:
    out = ctypes.create_string_buffer(len(data) + len(data) // 16 + 256)
    n = ctypes.c_ulong(len(out))
    rc = fn(data, len(data), out, ctypes.byref(n), wrkmem)
    assert rc == 0, f"{fn} rc={rc}"
    return out.raw[: n.value]


def ref_decompress(lib, y: bool, blob: bytes, cap: int) -> bytes:
    out = ctypes.create_string_buffer(cap + 64)
    n = ctypes.c_ulong(cap + 64)
    fn = lib.lzo1y_decompress if y else lib.lzo1x_decompress_safe
    rc = fn(blob, len(blob), out, ctypes.byref(n), None)
    assert rc == 0, f"reference decompressor rc={rc}"
    return out.raw[: n.value]


def data_classes(rng: random.Random) -> list[tuple[str, bytes]]:
    def rand(n):
        return bytes(rng.randrange(256) for _ in range(n))

    def runs(n):
        b = bytearray()
        while len(b) < n:
            b += bytes([rng.randrange(256)]) * rng.randrange(1, 40)
        return bytes(b[:n])

    def textish(n):
        words = [b"MAP", b"TT02", b"16392", b"SD_", b" ", b"\r\n", b"0"]
        b = bytearray()
        while len(b) < n:
            b += rng.choice(words)
        return bytes(b[:n])

    cls = [("empty", b""), ("one", b"\x00"), ("zeros-small", bytes(64)),
           ("zeros-64k", bytes(65536)), ("rand-1B", rand(1))]
    for n in (3, 17, 100, 1024, 4096, 65536, 300000):
        cls.append((f"rand-{n}", rand(n)))
        cls.append((f"runs-{n}", runs(n)))
        cls.append((f"textish-{n}", textish(n)))
        cls.append((f"zeros-{n}", bytes(n)))
    return cls


def main() -> int:
    rng = random.Random(20260807)
    lib = lzo_lib()
    wrkmem = ctypes.create_string_buffer(16 << 20)
    failures = 0

    with tempfile.TemporaryDirectory(prefix="i76-lzodec.") as td:
        work = Path(td)
        cli, asan = build(work)

        compressors = (
            ("x", lib.lzo1x_1_compress),
            ("x", lib.lzo1x_999_compress),
            ("y", lib.lzo1y_1_compress),
            ("y", lib.lzo1y_999_compress),
        )

        # layers 1+2: round-trip and reference parity
        cases = 0
        for name, data in data_classes(rng):
            for variant, cfn in compressors:
                blob = compress(lib, cfn, data, wrkmem)
                ref = ref_decompress(lib, variant == "y", blob, len(data))
                assert ref == data
                inp = work / "in.bin"
                outp = work / "out.bin"
                inp.write_bytes(blob)
                r = subprocess.run([str(cli), variant, str(inp), str(outp)],
                                   capture_output=True)
                cases += 1
                if r.returncode != 0:
                    print(f"FAIL decode {name} via lzo1{variant}: rc={r.returncode}")
                    failures += 1
                    continue
                if outp.read_bytes() != data:
                    print(f"FAIL mismatch {name} via lzo1{variant}")
                    failures += 1

        # layer 3: fail-closed under ASan
        probes = 0
        sample_blobs = []
        for name, data in data_classes(rng)[:14]:
            for variant, cfn in compressors:
                sample_blobs.append((variant, compress(lib, cfn, data, wrkmem)))
        rng2 = random.Random(99)
        for variant, blob in sample_blobs:
            inputs = [blob[:k] for k in range(len(blob))]          # every truncation
            inputs += [blob[:i] + bytes([blob[i] ^ 0xFF]) + blob[i+1:]  # byte flips
                       for i in rng2.sample(range(len(blob)), min(24, len(blob)))]
            inputs += [bytes(rng2.randrange(256) for _ in range(rng2.randrange(0, 96)))
                       for _ in range(24)]                          # junk
            inp = work / "fuzz.bin"
            outp = work / "fuzz.out"
            for j, bad in enumerate(inputs):
                inp.write_bytes(bad)
                if outp.exists():
                    outp.unlink()
                r = subprocess.run([str(asan), variant, str(inp), str(outp)],
                                   capture_output=True)
                probes += 1
                if r.returncode < 0 or b"AddressSanitizer" in r.stderr or b"runtime error" in r.stderr:
                    print(f"FAIL crash/sanitizer on input {j} of blob (len {len(blob)}, 1{variant})")
                    print(r.stderr.decode(errors="replace")[:800])
                    failures += 1
                elif (r.returncode == 0) != outp.exists():
                    print(f"FAIL exit/output mismatch on input {j}: rc={r.returncode}")
                    failures += 1

        print(f"round-trip cases: {cases}, fail-closed probes: {probes}")

    if failures:
        print(f"RESULT: FAIL ({failures})")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
