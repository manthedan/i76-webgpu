// assets.js — one-stop asset intake for the web shell (BYO-assets).
//
// The page's old flow was three separate pickers (nitro.zix, nitro.zfs,
// miss8 multi-select). Here the SAME intake serves four sources, and the
// engine side never changes — everything is staged into MEMFS /data/,
// exactly like the old pickers did:
//
//   1. "Open game folder" — File System Access API showDirectoryPicker();
//      walks the chosen tree (any depth) for the files below.
//   2. Drop ANYWHERE on the page — files or folders, any depth, routed by
//      classify(); the whole document is the drop surface.
//   3. The legacy pickers (#zixFile/#zfsFile/#dirInput) — still wired,
//      for browsers without folder support and existing automation.
//   4. A remembered folder handle in IndexedDB — "continue" on return
//      visits. Persisted only after a successful load.
//   5. Staged BYTES persisted in IndexedDB — restoreStaged() rehydrates
//      the whole staged set automatically on the next visit, whatever
//      intake originally supplied it (H-UAT-016). All local, never
//      uploaded.
//
// Routing rules (classify — pure, node-tested in tests/assets_test.mjs):
//   nitro.zix / nitro.zfs      -> /data/<name>        (Nitro VFS core)
//   i76.zix / i76.zfs          -> /data/<name>        (base-game VFS core)
//   under a miss8/ or miss16/  -> /data/<dir>/<name>  (loose missions)
//   music/<file>               -> /data/music/<name>  (user's CD rips)
//   smk/<file>                 -> /data/smk/<name>    (user's movies)
//   *.fnt                      -> /data/<name>        (HUD fonts — the
//                                 base6x7*.fnt files ship LOOSE in the
//                                 game dir, never in the zfs; the HUD
//                                 status line needs them, font.c)
//   lee_____.ttf / Lee.fot     -> /data/<name>        (handwritten shell font)
//   zmap*/znpd*.vqm,           -> /data/<name>        (loose paper-art fallback;
//   3/6escape.{pix,pak}                              archives provide these in
//                                                    the normal GOG profile)
//   addon/SCENARIO.DAT         -> /data/addon/<name>  (Nitro campaign:
//                                 four trips, 19 missions, play order and
//                                 titles — webmain.c campaign_build)
//   database.tvf / nitcar.def  -> /data/<name>        (Nitro shell's EFA/SHP
//                                 form art + authored portrait-key records)
//   anything else              -> null (counted as ignored, reported)
// Missions keep their directory name because engine paths are
// "miss8/N01.CBT" (webmain.c catalog). Loose files outside a known dir
// have no home and are ignored — dropping a bare .cbt does nothing.

// ---- pure routing (no DOM; imported by the node unit test) -------------

export function classify(file) {
  // file: { name, path? } — path is the full walk path when known
  // ("app/miss8/N01.CBT"), or just the filename for flat file drops.
  const segs = String(file.path || file.name).split('/').filter(Boolean);
  const last = segs[segs.length - 1].toLowerCase();
  if (last === 'nitro.zix' || last === 'nitro.zfs' ||
      last === 'i76.zix' || last === 'i76.zfs') return last;
  if (last.endsWith('.fnt')) return last;   // loose HUD fonts (font.c)
  if (last === 'database.tvf' || last === 'nitcar.def') return last;
  // Retail handwritten shell font (Lee Regular) + Windows .fot companion.
  // Used by the page shell via @font-face; not inside the ZFS.
  if (last === 'lee_____.ttf' || last === 'lee.fot') return last;
  // Paper art normally resolves from nitro.zix/zfs. Route loose/extracted
  // copies too so a complete GOG-folder stage keeps the same VFS basenames.
  if ((last.endsWith('.vqm') &&
       (/^zmap[36][a-z0-9_]+\.vqm$/.test(last) ||
        /^znpd[36][a-z0-9_]+\.vqm$/.test(last))) ||
      /^[36]escape\.(pix|pak)$/.test(last))
    return segs[segs.length - 1];
  // Mission / boot PCX title cards (loose in app/ or addon/).
  if (last.endsWith('.pcx') &&
      (last === 'loadgame.pcx' || last === 'loadscr.pcx' ||
       /^(p|b)\d{2}load\.pcx$/.test(last))) {
    if (segs.length >= 2 && segs[segs.length - 2].toLowerCase() === 'addon')
      return `addon/${segs[segs.length - 1]}`;
    return last;
  }
  for (let i = segs.length - 2; i >= 0; i--) {
    const d = segs[i].toLowerCase();
    if (d === 'miss8' || d === 'miss16')
      return `${d}/${segs[segs.length - 1]}`;
  }
  if (segs.length >= 2 && segs[segs.length - 2].toLowerCase() === 'music')
    return `music/${segs[segs.length - 1]}`;
  if (last.endsWith('.smk'))
    return `smk/${segs[segs.length - 1]}`;
  // The campaign file. addon/ is 27 files and ~600 KB of it is loading
  // screens nothing can decode yet, so this routes the ONE file the shell
  // consumes rather than the directory. The directory segment is
  // lowercased (like miss8/music above) because fs_fopen only retries the
  // BASENAME's case — "addon/scenario.dat" has to find the shipped
  // SCENARIO.DAT through that retry (src/engine/fs.c).
  if (segs.length >= 2 && segs[segs.length - 2].toLowerCase() === 'addon' &&
      last === 'scenario.dat')
    return `addon/${segs[segs.length - 1]}`;
  return null;
}

// Group files by destination route. First occurrence wins on duplicates.
export function plan(files) {
  const routes = new Map();   // dest -> file entry
  let ignored = 0;
  for (const f of files) {
    const dest = classify(f);
    if (!dest) { ignored++; continue; }
    if (!routes.has(dest)) routes.set(dest, f);
  }
  return { routes, ignored };
}

// ---- MEMFS staging ------------------------------------------------------
// FS is set by attach(); everything stages under /data (the VFS root,
// webmain.c web_init). attach() creates /data up front so collectors can
// stage files before web_init runs. Re-staging an already-staged path
// is skipped so repeated drops/folder-opens don't re-read megabytes.

let FS = null;
const staged = new Map();   // dest -> { name, size }

export function attach(emModule) { FS = emModule.FS; FS.mkdirTree('/data'); }

async function put(dest, file) {
  const idx = dest.lastIndexOf('/');
  if (idx > 0) FS.mkdirTree('/data/' + dest.slice(0, idx));   /* absolute! */
  FS.writeFile('/data/' + dest, new Uint8Array(await file.arrayBuffer()));
  staged.set(dest, { name: file.name, size: file.size });
}

// Stage collected entries ({name, path, file}). Returns counts for the
// page's status line. `files` may come from any collector below.
// onProgress(totalStaged, plannedInBatch) is called periodically with the
// RUNNING total across all batches, so a long folder stage counts up
// visibly instead of sitting silent (H-UAT playtest polish).
// Newly staged files are also queued for byte persistence (below) so the
// staged set survives a reload — locally, in this browser's IndexedDB.
export async function stageFiles(files, onProgress = null) {
  const { routes, ignored } = plan(files);
  let done = 0, skipped = 0;
  const fresh = [];
  for (const [dest, f] of routes) {
    if (staged.has(dest)) { skipped++; continue; }
    await put(dest, f.file);
    fresh.push(dest);
    done++;
    if (onProgress && done % 20 === 0) onProgress(staged.size, routes.size);
  }
  persistStaged(fresh);
  return { staged: done, skipped, ignored };
}

export function has(path) { return staged.has(path); }
export function stagedPaths() { return [...staged.keys()]; }
export function stagedCount(dir = null) {
  let n = 0;
  for (const d of staged.keys())
    if (!dir || d.startsWith(dir + '/')) n++;
  return n;
}
export function archiveProfile() {
  if (has('nitro.zix') && has('nitro.zfs')) return 'nitro';
  if (has('i76.zix') && has('i76.zfs')) return 'base';
  return null;
}
export function ready() { return archiveProfile() !== null; }

// ---- music (browser-owned playback, web/audio.js) -----------------------
// Keep staged basenames in deterministic CD-number order before audio.js
// parses strict <integer>.mp3 names into its number->name map. GOG ships
// music/{0,2,3,...,11}.mp3; track 1 was the data track, and 0.mp3 is its
// duplicate of 2.mp3. Selection is NOT positional: audio.js uses native
// shell track 10 and an explicitly interim drive rotation over 2..11 until
// the authored WRLD track is available (docs/specs/re/music-system.md).
// Other names sort afterward but are not music candidates.
export function musicOrder(names) {
  const key = (n) => {
    const s = String(n);
    const stem = s.replace(/\.[^.]*$/, '');
    return /^\d+$/.test(stem) ? [0, Number(stem), ''] : [1, 0, s.toLowerCase()];
  };
  return [...names].sort((a, b) => {
    const ka = key(a), kb = key(b);
    if (ka[0] !== kb[0]) return ka[0] - kb[0];
    if (ka[0] === 0) return ka[1] - kb[1];
    return ka[2] < kb[2] ? -1 : ka[2] > kb[2] ? 1 : 0;
  });
}

// Staged music basenames in playlist order (no directory prefix).
export function musicTracks() {
  return musicOrder([...staged.keys()]
    .filter((d) => d.startsWith('music/'))
    .map((d) => d.slice('music/'.length)));
}

// Bytes of a staged file back out of MEMFS (music decode feeds on this;
// nothing is ever fetched or uploaded). Returns null when unstaged or
// unreadable.
export function readStaged(path) {
  if (!FS || !staged.has(path)) return null;
  try { return FS.readFile('/data/' + path); } catch { return null; }
}

export function movieTracks() {
  return [...staged.keys()]
    .filter((d) => d.startsWith('smk/'))
    .map((d) => d.slice('smk/'.length))
    .sort((a, b) => a.localeCompare(b, undefined, { sensitivity: 'base' }));
}

// ---- collectors ---------------------------------------------------------
// All collectors yield {name, path, file} (File objects) so stageFiles is
// the single staging path.

// DirectoryHandle walk (File System Access API), depth-unlimited.
export async function collectFromHandle(dirHandle) {
  const out = [];
  const walk = async (handle, prefix) => {
    for await (const entry of handle.values()) {
      if (entry.kind === 'file') {
        out.push({ name: entry.name, path: prefix + entry.name,
                   file: await entry.getFile() });
      } else if (entry.kind === 'directory') {
        await walk(entry, prefix + entry.name + '/');
      }
    }
  };
  await walk(dirHandle, '');
  return out;
}

// DataTransfer walk: uses FileSystemEntry recursion when the browser
// exposes it (folders + files, any depth), else flat File objects.
export async function collectFromDrop(dataTransfer) {
  const out = [];
  const walkEntry = async (entry, prefix) => {
    if (entry.isFile) {
      const f = await new Promise((res, rej) => entry.file(res, rej));
      out.push({ name: f.name, path: prefix + f.name, file: f });
    } else if (entry.isDirectory) {
      const rd = entry.createReader();
      for (;;) {   // readEntries pages in batches of 100
        const batch = await new Promise((res, rej) =>
          rd.readEntries(res, rej));
        if (!batch.length) break;
        for (const e of batch)
          await walkEntry(e, prefix + entry.name + '/');
      }
    }
  };
  let walked = false;
  for (const item of dataTransfer.items || []) {
    const entry = item.webkitGetAsEntry ? item.webkitGetAsEntry() : null;
    if (entry) { await walkEntry(entry, ''); walked = true; }
  }
  if (!walked)
    for (const f of dataTransfer.files || [])
      out.push({ name: f.name, path: f.name, file: f });
  return out;
}

// ---- folder-handle persistence (IndexedDB) ------------------------------
// The handle survives reloads in the same browser profile. It is
// persisted only after a successful load (page calls remember());
// permission may lapse across sessions, so recall() re-queries and may
// prompt once (user gesture — the "continue" click).

const DB_NAME = 'i76-assets', STORE = 'kv', KEY = 'gameDir';
const FILE_STORE = 'files';   // staged BYTES, keyed by dest path (db v2)

// ONE cached connection. The byte-persistence path below writes >100 MB;
// opening a fresh connection per operation (the old per-call pattern) made
// the write queue crawl behind a running mission's 20 Hz main-thread load.
let dbPromise = null;
function openDb() {
  if (dbPromise) return dbPromise;
  dbPromise = new Promise((res, rej) => {
    const rq = indexedDB.open(DB_NAME, 2);
    rq.onupgradeneeded = () => {
      const db = rq.result;
      if (!db.objectStoreNames.contains(STORE)) db.createObjectStore(STORE);
      if (!db.objectStoreNames.contains(FILE_STORE))
        db.createObjectStore(FILE_STORE);
    };
    rq.onsuccess = () => res(rq.result);
    rq.onerror = () => { dbPromise = null; rej(rq.error); };
    // A still-open v1 connection (an old tab) blocks the upgrade forever;
    // fail the caller instead of hanging every staged-file feature.
    rq.onblocked = () => { dbPromise = null;
                           rej(new Error('i76-assets upgrade blocked')); };
  });
  return dbPromise;
}
function idbReq(store, mode, fn) {
  return openDb().then(db => new Promise((res) => {
    const tx = db.transaction(store, mode);
    const rq = fn(tx.objectStore(store));
    tx.oncomplete = () => res(rq && 'result' in rq ? rq.result : true);
    tx.onerror = () => res(null);
    tx.onabort = () => res(null);   // quota rejection lands here, not onerror
  }));
}
export async function remember(handle) {
  try { return await idbReq(STORE, 'readwrite', s => s.put(handle, KEY)); }
  catch { return null; }
}
export async function recall(interactive = true) {
  try {
    const h = await idbReq(STORE, 'readonly', s => s.get(KEY));
    if (!h || typeof h.queryPermission !== 'function') return null;
    let p = await h.queryPermission({ mode: 'read' });
    // requestPermission needs a user gesture; startup probes pass
    // interactive=false and only surface already-granted handles.
    if (p === 'prompt' && interactive)
      p = await h.requestPermission({ mode: 'read' });
    return p === 'granted' ? h : null;
  } catch { return null; }
}

/* Raw stored handle without permission checks — the page shows its
 * "continue" button from this; permission is asked on the click. */
export async function stored() {
  try {
    const h = await idbReq(STORE, 'readonly', s => s.get(KEY));
    return h && typeof h.queryPermission === 'function' ? h : null;
  } catch { return null; }
}
export async function forget() {
  try { await idbReq(STORE, 'readwrite', s => s.delete(KEY)); }
  catch { /* nothing to forget */ }
}

// ---- staged-byte persistence (H-UAT-016) --------------------------------
// The folder handle above only exists on the showDirectoryPicker path, and
// its read permission usually lapses to 'prompt' across browser restarts —
// so a drag-drop or manual-pick purchaser re-supplied the whole game folder
// EVERY visit. Every freshly staged file's bytes are therefore also written
// to the 'files' store, and restoreStaged() puts them straight back into
// MEMFS on the next load: no click, no permission prompt, any intake path.
// Legal posture unchanged: the bytes stay in this browser's own IndexedDB
// on the user's machine; nothing is fetched or uploaded.

const MANIFEST_KEY = 'stagedManifest';   // kv: dests known durably written

let persistNote = '';                    // one-line failure notice, or ''
let persistChain = Promise.resolve();    // serialized background batches
const persistedSet = new Set();          // dests confirmed in the store

export function persistNotice() { return persistNote; }
/* Gates and the page await this before claiming the staged set is durable
 * (a reload can otherwise race the tail of a 100+ MB write queue). */
export function persistIdle() { return persistChain; }

/* One readwrite transaction per CHUNK of files, on the shared connection.
 * Per-file transactions drained far too slowly behind a running mission
 * (observed: a reload minutes after staging still caught a torn tail).
 * Bytes are read back from MEMFS at write time, so the queue holds only
 * path strings — never a second in-memory copy of a 60 MB archive. */
function chunkPut(db, dests) {
  return new Promise((res) => {
    let tx;
    try { tx = db.transaction(FILE_STORE, 'readwrite'); }
    catch { return res(false); }
    const s = tx.objectStore(FILE_STORE);
    for (const dest of dests) {
      const meta = staged.get(dest);
      try {
        s.put({ name: meta ? meta.name : dest,
                bytes: FS.readFile('/data/' + dest) }, dest);
      } catch { /* one unreadable file must not sink the chunk */ }
    }
    tx.oncomplete = () => res(true);
    tx.onerror = () => res(false);
    tx.onabort = () => res(false);      // quota rejection lands here
  });
}

// Greedy size-bounded chunking: big archives go alone, small mission files
// share a transaction. ~32 MB keeps the per-commit clone bounded.
const CHUNK_BYTES = 32 * 1024 * 1024;
function chunked(dests) {
  const out = [];
  let cur = [], bytes = 0;
  for (const d of dests) {
    const size = (staged.get(d) || {}).size || 0;
    if (cur.length && bytes + size > CHUNK_BYTES) { out.push(cur); cur = []; bytes = 0; }
    cur.push(d); bytes += size;
  }
  if (cur.length) out.push(cur);
  return out;
}

function persistStaged(dests) {
  if (typeof indexedDB === 'undefined' || !dests.length) return;
  persistChain = persistChain.then(async () => {
    if (persistNote) return;             // storage already failed: stop
    let db;
    try { db = await openDb(); } catch { return; }
    for (const chunk of chunked(dests)) {
      if (!(await chunkPut(db, chunk))) {
        persistNote = 'staged files could not be saved for your next ' +
                      'visit (storage quota or private browsing) — ' +
                      'you will need to supply the folder again.';
        return;
      }
      for (const d of chunk) persistedSet.add(d);
    }
    /*
     * The manifest commits AFTER its files, on the same connection, so
     * transaction order guarantees every listed dest is durable. A reload
     * that kills the queue mid-batch leaves unlisted stragglers in the
     * store (harmless) — never a manifest naming missing files.
     */
    await idbReq(STORE, 'readwrite',
                 (s) => s.put([...persistedSet], MANIFEST_KEY));
  }).catch(() => { /* persistence must never break staging */ });
}

/*
 * Restore a previous visit's staged set from the 'files' store into MEMFS.
 * FAIL-CLOSED: only the manifest's file list restores, and only when it
 * carries a complete core pair (.zix/.zfs) — a torn or partial write never
 * half-boots a VFS; the page just asks for the folder as before.
 * Returns the number of files restored.
 */
export async function restoreStaged() {
  if (!FS || typeof indexedDB === 'undefined') return 0;
  let db;
  try { db = await openDb(); } catch { return 0; }
  let manifest = null;
  try { manifest = await idbReq(STORE, 'readonly', (s) => s.get(MANIFEST_KEY)); }
  catch { return 0; }
  if (!Array.isArray(manifest) || !manifest.length) return 0;
  const want = new Set(manifest.map(String));
  const complete = (want.has('nitro.zix') && want.has('nitro.zfs')) ||
                   (want.has('i76.zix') && want.has('i76.zfs'));
  if (!complete) return 0;
  let n = 0;
  // Cursor, not getAll: the set is >100 MB and getAll would hold every
  // record in memory at once on top of the MEMFS copy being written.
  await new Promise((res) => {
    try {
      const rq = db.transaction(FILE_STORE, 'readonly')
                   .objectStore(FILE_STORE).openCursor();
      rq.onsuccess = () => {
        const c = rq.result;
        if (!c) return res(true);
        const dest = String(c.key), rec = c.value;
        if (want.has(dest) && !staged.has(dest) && rec && rec.bytes) {
          try {
            const idx = dest.lastIndexOf('/');
            if (idx > 0) FS.mkdirTree('/data/' + dest.slice(0, idx));
            FS.writeFile('/data/' + dest, rec.bytes);
            staged.set(dest, { name: rec.name || dest,
                               size: rec.bytes.length });
            persistedSet.add(dest);
            n++;
          } catch { /* one unreadable record must not kill the restore */ }
        }
        c.continue();
      };
      rq.onerror = () => res(true);
    } catch { res(true); }
  });
  return n;
}

/* Drop the persisted bytes (the "clear staged files" control). Files
 * already staged into MEMFS this session stay usable; the next load simply
 * starts from a clean intake again. */
export async function clearStaged() {
  try {
    await idbReq(STORE, 'readwrite', (s) => s.delete(MANIFEST_KEY));
    await idbReq(FILE_STORE, 'readwrite', (s) => s.clear());
    persistedSet.clear();
  } catch { /* nothing stored */ }
}

export function canPickFolder() {
  return typeof window !== 'undefined' &&
         typeof window.showDirectoryPicker === 'function';
}

/*
 * Retail lee_____.ttf (the shell's handwriting face) fails Chrome's font
 * sanitizer ("Invalid font data"), so the page silently fell back to a generic
 * face. Two defects must both be repaired: a version-0 OS/2 table (upgraded to
 * version 1: version field + the two code-page words, Latin-1) and a format-4
 * cmap whose segments overlap (0xB6-0xB7 then 0xB7; re-encoded as ordered,
 * non-overlapping delta segments, first segment winning as Windows reads it).
 * The sfnt search fields are also wrong and are derived afresh. Fonts without
 * these defects are returned unchanged.
 */
function cmapFormat4Map(u8) {
  /* Decode a format-4 subtable to code -> glyph; null if malformed. */
  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  if (u8.byteLength < 14 || dv.getUint16(0) !== 4) return null;
  const segX2 = dv.getUint16(6), seg = segX2 / 2;
  const ends = 14, starts = 16 + segX2, deltas = starts + segX2, ranges = deltas + segX2;
  if (ranges + segX2 > u8.byteLength) return null;
  const map = new Map();
  let overlap = false, prevEnd = -1;
  for (let i = 0; i < seg; i++) {
    const end = dv.getUint16(ends + 2 * i), start = dv.getUint16(starts + 2 * i);
    const delta = dv.getUint16(deltas + 2 * i), ro = dv.getUint16(ranges + 2 * i);
    if (start <= prevEnd) overlap = true;
    prevEnd = Math.max(prevEnd, end);
    for (let c = start; c <= end && c !== 0xffff; c++) {
      let g;
      if (ro === 0) g = (c + delta) & 0xffff;
      else {
        const at = ranges + 2 * i + ro + 2 * (c - start);
        if (at + 2 > u8.byteLength) return null;
        g = dv.getUint16(at);
        if (g) g = (g + delta) & 0xffff;
      }
      if (!map.has(c)) map.set(c, g);
    }
  }
  return { map, overlap };
}

function cmapFormat4Encode(map) {
  /* Ordered delta-only segments plus the mandatory 0xFFFF terminator. */
  const codes = [...map.keys()].filter((c) => map.get(c)).sort((a, b) => a - b);
  const segs = [];
  for (const c of codes) {
    const last = segs[segs.length - 1];
    if (last && c === last.end + 1 && map.get(c) - c === last.g - last.start)
      last.end = c;
    else segs.push({ start: c, end: c, g: map.get(c) });
  }
  segs.push({ start: 0xffff, end: 0xffff, g: 0 });
  const n = segs.length, len = 16 + 8 * n;
  const out = new Uint8Array(len), dv = new DataView(out.buffer);
  let p2 = 1, l2 = 0;
  while (p2 * 2 <= n) { p2 *= 2; l2++; }
  dv.setUint16(0, 4); dv.setUint16(2, len); dv.setUint16(4, 0);
  dv.setUint16(6, 2 * n); dv.setUint16(8, 2 * p2); dv.setUint16(10, l2);
  dv.setUint16(12, 2 * n - 2 * p2);
  segs.forEach((sg, i) => {
    dv.setUint16(14 + 2 * i, sg.end);
    dv.setUint16(16 + 2 * n + 2 * i, sg.start);
    const delta = sg.start === 0xffff ? 1 : (sg.g - sg.start) & 0xffff;
    dv.setUint16(16 + 4 * n + 2 * i, delta);
    dv.setUint16(16 + 6 * n + 2 * i, 0);
  });
  return out;
}

function repairCmap(u8) {
  /* Re-encode every overlapping format-4 subtable; null when nothing to do. */
  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  if (u8.byteLength < 4) return null;
  const n = dv.getUint16(2);
  if (4 + 8 * n > u8.byteLength) return null;
  const recs = [];
  let changed = false;
  for (let i = 0; i < n; i++) {
    const at = 4 + 8 * i, off = dv.getUint32(at + 4);
    if (off + 4 > u8.byteLength) return null;
    const fmt = dv.getUint16(off);
    /* Subtable length lives in a format-specific field; carry every format
     * through unchanged except an overlapping format 4. */
    let len = 0;
    if (fmt <= 6) len = dv.getUint16(off + 2);
    else if (fmt === 14) len = off + 6 <= u8.byteLength ? dv.getUint32(off + 2) : 0;
    else if (off + 8 <= u8.byteLength) len = dv.getUint32(off + 4);
    if (!len || off + len > u8.byteLength) return null;
    let data = u8.subarray(off, off + len);
    if (fmt === 4) {
      const decoded = cmapFormat4Map(data);
      if (!decoded) return null;
      if (decoded.overlap) { data = cmapFormat4Encode(decoded.map); changed = true; }
    }
    recs.push({ pid: dv.getUint16(at), eid: dv.getUint16(at + 2), data });
  }
  if (!changed) return null;
  let size = 4 + 8 * n;
  for (const r of recs) size += r.data.byteLength;
  const out = new Uint8Array(size), ov = new DataView(out.buffer);
  ov.setUint16(0, 0); ov.setUint16(2, n);
  let cursor = 4 + 8 * n;
  recs.forEach((r, i) => {
    ov.setUint16(4 + 8 * i, r.pid); ov.setUint16(6 + 8 * i, r.eid);
    ov.setUint32(8 + 8 * i, cursor);
    out.set(r.data, cursor);
    cursor += r.data.byteLength;
  });
  return out;
}

export function repairLegacyTrueType(bytes) {
  const src = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  if (bytes.byteLength < 12) return bytes;
  const count = src.getUint16(4);
  if (12 + count * 16 > bytes.byteLength) return bytes;
  const tables = [];
  for (let i = 0; i < count; i++) {
    const at = 12 + i * 16;
    const off = src.getUint32(at + 8), len = src.getUint32(at + 12);
    if (off + len > bytes.byteLength) return bytes;
    tables.push({ tag: src.getUint32(at), data: bytes.subarray(off, off + len) });
  }
  const OS2 = 0x4f532f32, HEAD = 0x68656164, CMAP = 0x636d6170;
  let repaired = false;
  const os2 = tables.find((t) => t.tag === OS2);
  if (os2 && os2.data.byteLength === 78 &&
      new DataView(os2.data.buffer, os2.data.byteOffset, 2).getUint16(0) === 0) {
    const upgraded = new Uint8Array(86);
    upgraded.set(os2.data);
    const v1 = new DataView(upgraded.buffer);
    v1.setUint16(0, 1);            // version 1
    v1.setUint32(78, 1);           // ulCodePageRange1: Latin 1
    v1.setUint32(82, 0);           // ulCodePageRange2
    os2.data = upgraded;
    repaired = true;
  }
  const cmap = tables.find((t) => t.tag === CMAP);
  const fixedCmap = cmap && repairCmap(cmap.data);
  if (fixedCmap) { cmap.data = fixedCmap; repaired = true; }
  if (!repaired) return bytes;

  const pad4 = (n) => (n + 3) & ~3;
  let size = 12 + count * 16;
  for (const t of tables) size += pad4(t.data.byteLength);
  const out = new Uint8Array(size);
  const dv = new DataView(out.buffer);
  out.set(bytes.subarray(0, 4));             // sfnt version
  /* The retail header's binary-search fields are also wrong; derive them. */
  let pow2 = 1, log2 = 0;
  while (pow2 * 2 <= count) { pow2 *= 2; log2++; }
  dv.setUint16(4, count);
  dv.setUint16(6, pow2 * 16);
  dv.setUint16(8, log2);
  dv.setUint16(10, count * 16 - pow2 * 16);
  const checksum = (u8) => {
    const view = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
    let sum = 0;
    for (let i = 0; i < pad4(u8.byteLength); i += 4) {
      let word = 0;
      for (let b = 0; b < 4; b++)
        word = (word << 8) | (i + b < u8.byteLength ? view.getUint8(i + b) : 0);
      sum = (sum + (word >>> 0)) >>> 0;
    }
    return sum;
  };
  tables.sort((a, b) => a.tag - b.tag);     // directory must be tag-ordered
  let cursor = 12 + count * 16;
  tables.forEach((t, i) => {
    if (t.tag === HEAD && t.data.byteLength >= 12) {
      t.data = t.data.slice();
      new DataView(t.data.buffer).setUint32(8, 0);   // checkSumAdjustment
    }
    const at = 12 + i * 16;
    dv.setUint32(at, t.tag);
    dv.setUint32(at + 4, checksum(t.data));
    dv.setUint32(at + 8, cursor);
    dv.setUint32(at + 12, t.data.byteLength);
    out.set(t.data, cursor);
    t.offset = cursor;
    cursor += pad4(t.data.byteLength);
  });
  const head = tables.find((t) => t.tag === HEAD);
  if (head && head.data.byteLength >= 12)
    dv.setUint32(head.offset + 8, (0xB1B0AFBA - checksum(out)) >>> 0);
  return out;
}
