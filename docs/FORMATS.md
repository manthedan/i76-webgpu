# Formats implemented by the port

> This is an implementation guide, not a formal game-format specification. It describes what the retained readers consume and emit. It does not promise compatibility with every game revision, every malformed file, or every field the original software understood.

All game bytes must come from the user's own installation. Do not add sample archives, extracted missions, media, texture dumps, or executable-derived blobs to tests or documentation.

## Conventions

- Multibyte game fields consumed by these readers are little-endian.
- Names are commonly fixed-width and may not be NUL-terminated; readers copy into bounded local storage and add a terminator.
- Archive and VFS lookup is generally case-insensitive and constrained by the original short-name fields.
- Container walkers require each chunk's declared total length to include its header, be at least the header size, and remain inside the parent buffer.
- Counts and offsets must be validated before multiplication, allocation, or pointer movement. Existing checks are useful, not a blanket hostile-input guarantee.
- “Observed,” “inferred,” and “port decision” are different levels of confidence. Do not collapse them into “specified.”

## Source routing: `.zix` and loose files

Implementation: `src/engine/vfs.c`, with filesystem handling in `src/engine/fs.c`.

The selected profile is determined by the first supported archive index found under the configured root:

- `nitro.zix` / `NITRO.ZIX` selects the Nitro profile;
- `i76.zix` / `I76.ZIX` selects the base profile.

A `.zix` file is parsed as CRLF-tolerant text:

1. first line: total file count;
2. source declarations until a line made only of hyphens;
3. `DIR: <archive-name>` declarations for archive sources;
4. `<source-index> <filename>` entries.

The parser creates a lowercase, fixed-width filename table and sorts it for binary search. Source zero represents the loose asset root; Windows paths embedded in the index are not trusted as host paths. Archive handles open lazily.

If an indexed lookup does not resolve, the VFS also tries a loose file under the configured root. This is required for browser-staged mission trees and loose fonts/media that are not represented by the chosen index.

Limits visible in the implementation include 16 VFS sources and 15 meaningful bytes in the indexed name key. Longer or unusual filenames should not be assumed to work merely because the browser can stage them.

## ZFS archives

Implementation: `src/engine/zfs.c`, `src/engine/zfs.h`, and `src/engine/lzodec.c`.

The reader expects:

- magic `ZFSF`;
- version 1;
- 100 directory entries per block;
- a 28-byte header;
- linked directory blocks beginning with a 32-bit next-block offset;
- 100 fixed 36-byte entry slots per full block.

Each directory entry contains:

```text
char name[16]
u32  data_offset
u32  index
u32  stored_size
u32  timestamp
u32  flags
```

Empty and deleted entries are omitted. Live entries are sorted case-insensitively for lookup.

Entry flags distinguish raw from compressed data and carry the expected expanded size. Compressed entries use either the LZO1X or LZO1Y stream variant implemented by `lzodec.c`. If the archive header carries an XOR key, the reader applies that key to complete 32-bit words after reading/decompression.

The archive reader is intended for known purchaser archives and generated decoder tests. It checks magic, version, directory mode, allocation results, and decompressor output, but not every seek/offset relationship is hardened as a hostile-archive parser. New validation should fail closed without changing accepted known-good bytes.

## BWD2 chunk containers

Implementations: bounded walkers in `src/engine/scene.c`, `terrain.c`, `mission.c`, and related modules.

Mission and configuration data use nested tagged chunks. The common pattern is:

```text
u32 tag       // four bytes, often shown as ASCII
u32 total     // header + payload bytes
byte payload[total - 8]
```

A walker advances by `total`, rejects values below 8 or beyond its parent buffer, and typically stops at `EXIT`. Tags used by current readers include world, terrain, object, action/FSM, road, paint, and component records.

Do not implement a generic “scan for four ASCII bytes” fallback. Context and parent bounds matter, and the same bytes may occur inside payloads.

## Mission world and terrain

Implementation: `src/engine/terrain.c` / `terrain.h`.

### `TDEF`, `ZMAP`, and `ZONE`

A mission container's `TDEF` chunk supplies:

- `ZMAP`: one count byte followed by an 80×80 byte zone grid;
- `ZONE`: one unknown byte followed by a 13-byte terrain filename.

A zone value is an index into the terrain block file; `0xFF` marks an empty patch. If no usable `ZONE` is present, the loader derives a `.ter` name from the mission basename. Resolution first respects the mission directory and then VFS fallback behavior.

### `.ter`

A terrain file is a concatenation of 32 KiB blocks. Each block is a 128×128 array of little-endian `u16` samples:

- low 12 bits: height;
- high nibble: terrain flags, with some consumers interpreting the top three bits as a surface class and the lowest bit of that nibble as blocked state.

The reader uses 5 m sample spacing. One patch therefore spans 640 m in the zone grid. The implemented height scale is 0.1 m per low-bit unit; source comments identify the confidence boundary for that mapping and it should not be promoted into a formal universal claim.

Adjacent present patches share the final 5 m cell by sampling the neighbor's first row/column. An absent neighbor remains a flat/clamped edge; the engine does not invent a diagonal surface through an empty patch.

### Roads and drivable faces

Road records are read from mission road chunks as left/right point ribbons. Stored Y is replaced by the terrain height query before rendering. Static scene classes can also register upward OEG polygons as drivable surfaces. Those polygons remain a separate chassis query; per-wheel terrain sampling continues to use the heightfield.

The software renderer consumes the engine's terrain queries directly. `terrain_lod_mesh_export()` emits an adaptive three-band triangle stream for WebGPU. That export can omit source 5 m samples and must not be treated as an exact height-grid serialization.

## OEG geometry

Implementation: `src/engine/geomesh.c` / `geomesh.h`.

The decoder recognizes little-endian magic `0x2e47454f` (`OEG.` in byte order). Relevant header fields are:

| Offset | Field |
|---:|---|
| `0x08` | fixed-width mesh name |
| `0x18` | vertex count |
| `0x1c` | face count |
| `0x24` | position array, three `f32` values per vertex |

A parallel three-float normal array immediately follows positions. Face records begin at `(vertex_count * 6 + 9) * 4` bytes.

Each face has a 55-byte header followed by `face_vertex_count` 16-byte corner records. The retained decoder reads:

- corner count;
- RGB bytes;
- four unaligned plane floats;
- three mode/flag bytes;
- a 13-byte texture name;
- per-corner position index, normal index, and UV floats.

Normal and position indices are separately range-checked; code must not assume they are always equal. The decoder caps vertices and faces at 100,000 each and corners per face at 64. It performs a validation/counting pass before allocating the output arrays.

`.geo` files contain one OEG image. Some geometry `.pak` entries are resolved to an OEG image by the surrounding index/container code. `geomesh_decode()` itself expects the OEG image, not an arbitrary whole asset tree.

## Geometry cache

Implementation: `src/engine/meshcache.c`.

This is not an on-disk format, but its keying rules affect format extensions:

- cache keys are uppercased fixed 8-byte geometry names;
- entries have both name and pointer indexes;
- acquired entries are reference-counted;
- zero-reference entries enter an oldest-first LRU;
- the explicit 2,000,000-byte budget accounts by source image size, not exact decoded heap size.

A new geometry decoder must preserve acquire/release ownership and deterministic naming. Bypassing the cache for scene-owned meshes can create double ownership or stale pointers at mission reload.

## Mission object and action chunks

Implementation: `src/engine/mission.c` and `src/engine/scene.c`.

The mission walker reads object definitions from ODEF `OBJ\0` records. The retained mission-side parser requires at least a 100-byte payload and consumes:

- an 8-byte packed label/instance identifier;
- a 12-float frame beginning at `0x08` (right, up, forward, position);
- class at `0x5c`;
- flags at `0x60`;
- team at `0x62`.

The same transform must be interpreted consistently by scene rendering, collision registration, spawn placement, and mission logic. Arena missions may use object markers without a non-empty FSM; scripted trip missions use both object and action data.

## FSM image

Implementation: `src/engine/fsm.c` / `fsm.h`; host binding in `src/engine/mission.c`.

An embedded FSM payload is a sequence of seven little-endian tables:

1. actions: count, then 40-byte names;
2. entities: count, then 40-byte label plus 8-byte packed object name;
3. sound clips: count, then 40-byte names;
4. paths: count, then a 40-byte name, point count, and XYZ `f32` triples;
5. machines: count, then fixed 168-byte records;
6. shared variables: count, then `i32` cells;
7. bytecode: count, then `{u32 opcode, i32 argument}` pairs.

A machine record begins with bytecode start and argument count, followed by argument cell indices. The remainder of the 168-byte record is opaque and must remain unread. The current maximum argument count follows from the record size.

Machines execute cooperatively on bounded stacks. Reference-carrying slots point into shared variable cells, so writes can become visible across machines. Bad indices, stack over/underflow, invalid control flow, unsupported opcodes, and watchdog exhaustion use explicit trap/halt behavior instead of intentionally corrupting memory.

The parser applies table count and remaining-buffer checks. This protects the paths implemented here but is not proof that arbitrary hostile FSM images are safe.

Mission files may spell the FSM chunk tag with the first three bytes `FSM` and a varying fourth byte. Scripted missions carry the seven-table image in the action definition area. Arena files can carry an empty payload and use host-side race/capture/melee controllers instead.

## Images and textures

### PCX

Implementation: `src/engine/pcx.c` / `pcx.h`.

The loader supports ZSoft PCX version 5, 8-bit indexed, one-plane RLE. It reads a 256×RGB palette from the final 769 bytes (`0x0c` marker plus 768 palette bytes) and emits a row-major index buffer.

### `.pix` / `.pak`

Implementation: `src/engine/pixidx.c` / `pixidx.h` and format-specific callers.

A text `.pix` manifest starts with a count and then records `NAME offset length` ranges into a sibling `.pak`. The merged index sorts by `(key, pak, offset)` before adjacent duplicate removal so the selected duplicate does not depend on the platform's `qsort` stability.

Always validate `offset + length` against the owning `.pak` using overflow-safe arithmetic before exposing a slice.

### VQM and codebooks

Implementation: `src/engine/vqm.c` / `vqm.h`.

VQM tiles decode as 4×4 blocks. A block is either a solid palette index or a reference into a shared codebook. Output bytes remain indices into the mission level palette; conversion to RGB belongs at final presentation, not in the decoder. Codebooks are cached by content identity/name rather than pointer address.

### M16

Implementation: `src/engine/m16.c` / `m16.h`.

The reader consumes:

```text
u32 width
u32 height_with_flags   // high byte is flags
u8  indices[width * height]
u32 palette_count
u16 palette_rgb565[palette_count]
```

The expanded result is RGBA8. Index `0xff` is transparent; other out-of-range indices use the decoder's documented black fallback. Zero dimensions, truncation, and malformed palette bounds are rejected.

### Vehicle paint chain

Implementation: `src/engine/paint.c` / `paint.h`.

Vehicle OEG faces may carry placeholder names rather than final textures. The resolver transforms the placeholder into a TMT key, finds the matching entry in a vehicle paint file, reads the TMT damage-state names, and resolves the selected texture base. The software and GPU paths must use the same owner paint file; caching only by placeholder name can incorrectly share one vehicle's paint with another.

## Sound and movies

### RIFF/WAVE and GAS0-wrapped audio

Implementation: `src/engine/sound.c` / `sound.h`; playback in `web/audio.js`.

The sound reader accepts bounded RIFF/WAVE PCM chunks, mono 8-bit unsigned or 16-bit signed data at supported authored rates. It can skip a fixed GAS0 wrapper used by some `.gpw` files and retries `.wav`/`.gpw` extension alternatives. Decoded samples are normalized to unsigned 8-bit PCM for transfer to WebAudio.

The parser stops after bounded required chunks rather than requiring trailing ancillary chunks to be perfect. Adding codec support must not make malformed chunk lengths escape their enclosing buffer.

Smacker `.smk` decoding is provided by the separately licensed vendored library. `web/movie.c` passes it a complete VFS-owned memory image, validates basic metadata, selects the first available audio track, and exposes decoded buffers. Do not describe this bridge as a new or independently specified Smacker implementation.

## Saves and browser records

### Port live-state snapshot (`I76S`)

Implementation: `src/engine/save.c` / `save.h`.

This is a port-specific fixed 316-byte developer format, not a retail save:

| Offset | Value |
|---:|---|
| 0 | magic `I76S` |
| 4 | little-endian version (`1`) |
| 8 | total byte size (`316`) |
| 12 | 64-bit mission tick |
| 20 | 32-bit mission state |
| 24 | selected live car fields, written explicitly |

Floating-point fields are serialized as their IEEE-754 binary64 bit patterns, little-endian. No C struct padding is written. Deserialization validates the complete header and state range before mutating live state.

The snapshot omits configuration identity and most script state, including FSM instruction/stack state, shared cells, timers, and queued radio. A host must load the same mission and vehicle configuration before applying it. It is unsuitable for browser-facing in-mission resume without a new, explicitly versioned design.

### Browser progress profile

Implementation: `web/save.js`.

The browser profile is structured IndexedDB data, currently schema 1. It stores completed mission paths, terminal results, garage selection, and renderer/view preferences. It preserves unknown keys, refuses to overwrite records from a newer schema, and quarantines malformed records when possible. It contains no game asset bytes and no active simulation snapshot.

### Trial bundle

Implementation: `web/trial.js`.

A developer trial bundle contains `meta.json`, JSONL tape/events/markers, a notes template, and optional WebM video. The tape records one row per completed fixed tick and the input masks actually consumed. This is a port evidence format, not a game format and not by itself a release qualification.

## Extending a reader safely

For a new field or format:

1. Identify the owning container and lifetime before writing a decoder.
2. Add a generated, redistributable fixture that reaches the exact branch.
3. Validate the smallest enclosing header first, then counts, then offset/length arithmetic.
4. Use explicit little-endian reads; do not cast unaligned bytes to host structs.
5. Cap counts before allocation and check multiplication/addition overflow.
6. Make failure leave output pointers empty and live state unchanged.
7. Preserve unknown bytes rather than assigning invented semantics.
8. Keep archive bytes, extracted assets, screenshots, and executable-derived dumps out of the repository.
9. Test accepted data and rejection paths; mutation testing does not require every mutated stream to be invalid.
10. State what the test proves and what remains unverified.
