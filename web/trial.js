// trial.js — human-trial recorder (docs/HUMAN-UAT.md). Developer-panel
// only: the page arms it, drive mode records one tape line per sim tick,
// F12 drops tick-stamped markers, and stop assembles a bundle directory via
// the File System Access API (single-ZIP download fallback without it).
// Classic script exposing the I76Trial global, same pattern as audio.js.
//
// Bundle authority boundary: the TAPE is the precise artifact (sim ticks,
// input masks, pose); the VIDEO is eyes only and is allowed to be absent
// (headless browsers, no MediaRecorder) without failing the bundle.
const I76Trial = (() => {
  let rec = null;

  // ---- video (secondary evidence, fail-soft) ----------------------------

  function startVideo(canvas) {
    try {
      if (!canvas || !canvas.captureStream || typeof MediaRecorder === 'undefined')
        return null;
      const mime = ['video/webm;codecs=vp9', 'video/webm;codecs=vp8', 'video/webm']
        .find((t) => MediaRecorder.isTypeSupported(t)) || '';
      const stream = canvas.captureStream(30);
      const mr = new MediaRecorder(stream, mime ? { mimeType: mime } : undefined);
      const chunks = [];
      mr.ondataavailable = (e) => { if (e.data && e.data.size) chunks.push(e.data); };
      mr.start(1000);
      return { mr, chunks };
    } catch { return null; }
  }

  function stopVideo(v) {
    return new Promise((resolve) => {
      try {
        v.mr.onstop = () => {
          for (const track of v.mr.stream.getTracks()) track.stop();
          const blob = new Blob(v.chunks, { type: v.mr.mimeType || 'video/webm' });
          resolve(blob.size ? blob : null);
        };
        v.mr.stop();
      } catch { resolve(null); }
    });
  }

  // ---- recording lifecycle ----------------------------------------------

  // opts: {build, mission, vehicle, shellRules, renderer, viewPreset,
  //        canvas, startTick} — startTick is the engine tick counter at
  //        recording start (0 when armed at drive load): tapes starting
  //        at 0 replay pose-exact; later starts are evidence-only
  //        (replay cannot reconstruct pre-recording inputs).
  function start(opts) {
    if (rec) return false;
    rec = {
      meta: {
        kind: 'i76-human-trial', v: 1,
        date: new Date().toISOString(),
        build: opts.build || 'unknown',
        mission: opts.mission || '',
        vehicle: opts.vehicle || '',
        shellRules: opts.shellRules || null,
        renderer: opts.renderer || 'sw',
        viewPresetStart: opts.viewPreset || 0,
        startTick: opts.startTick ?? 0,
        ticks: 0,
        result: null,
      },
      tape: [], events: [], markers: [],
      lastTick: opts.startTick ?? 0,
      video: startVideo(opts.canvas),
    };
    return true;
  }

  function recording() { return !!rec; }

  // Called by the page loop after each web_drive_step. The engine tick
  // counter increments only on a full step — movie-pending drains and
  // duplicate reads (same tick) are dropped, so tape tick N is sim tick N,
  // record and replay side alike.
  function tick(pose, held, pressed) {
    if (!rec) return;
    if (pose.tick <= rec.lastTick) return;
    rec.tape.push({
      tick: pose.tick, held: held >>> 0, pressed: pressed >>> 0,
      x: pose.x, y: pose.y, z: pose.z,
      yaw: pose.yaw, pitch: pose.pitch, roll: pose.roll, speed: pose.speed,
    });
    rec.lastTick = pose.tick;
  }

  // Page-level events the tape cannot see (view presets, pause, recover,
  // shell transitions, movie acks). Tick-stamped for video/tape alignment.
  function event(kind, detail) {
    if (!rec) return;
    rec.events.push({ tick: rec.lastTick, kind, ...(detail || {}) });
  }

  // F12 hotkey / marker button: points at the last completed tick — the
  // frame the human just saw.
  function marker() {
    if (!rec) return 0;
    const m = {
      seq: rec.markers.length + 1,
      tick: rec.lastTick,
      wallMs: Math.round(performance.now()),
    };
    rec.markers.push(m);
    return m.seq;
  }

  async function stop(result) {
    if (!rec) return null;
    const r = rec;
    rec = null;
    r.meta.ticks = r.tape.length;
    r.meta.result = result || null;
    const video = r.video ? await stopVideo(r.video) : null;
    return { meta: r.meta, tape: r.tape, events: r.events,
             markers: r.markers, video };
  }

  // ---- bundle serialization + save --------------------------------------

  function slug(s) {
    return String(s || 'drive').toLowerCase()
      .replace(/\.[a-z0-9]+$/, '').replace(/[^a-z0-9]+/g, '-')
      .replace(/^-+|-+$/g, '') || 'drive';
  }

  function bundleName(meta) {
    const d = meta.date.replace(/[-:]/g, '').replace('T', '-').slice(0, 15);
    return `${d}-${slug(meta.mission)}`;
  }

  const toJsonl = (rows) => rows.map((r) => JSON.stringify(r)).join('\n') +
    (rows.length ? '\n' : '');

  function bundleFiles(bundle) {
    const files = [
      ['meta.json', JSON.stringify(bundle.meta, null, 2) + '\n'],
      ['tape.jsonl', toJsonl(bundle.tape)],
      ['events.jsonl', toJsonl(bundle.events)],
      ['markers.jsonl', toJsonl(bundle.markers)],
      ['notes.md', notesTemplate(bundle)],
    ];
    if (bundle.video) files.push(['video.webm', bundle.video]);
    return files;
  }

  function notesTemplate(bundle) {
    const m = bundle.meta;
    const lines = [
      `# Trial notes — ${bundleName(m)}`, '',
      `build ${m.build} · mission ${m.mission} · vehicle ${m.vehicle} · ` +
        `${m.ticks} ticks · result ${m.result ?? '(stopped)'}`, '',
      'One short paragraph per marker: what you saw, what you expected.',
      'Free text; the agent aligns it by marker number.', '',
      '## Markers', '',
    ];
    for (const k of bundle.markers)
      lines.push(`- marker ${k.seq} · tick ${k.tick} · `);
    if (!bundle.markers.length)
      lines.push('(no markers dropped — describe problems below)');
    lines.push('', '## General', '');
    return lines.join('\n');
  }

  async function writeFile(dir, name, content) {
    const fh = await dir.getFileHandle(name, { create: true });
    const w = await fh.createWritable();
    await w.write(content);
    await w.close();
  }

  async function saveToDirectory(bundle) {
    const root = await window.showDirectoryPicker({ mode: 'readwrite' });
    const dir = await root.getDirectoryHandle(bundleName(bundle.meta),
                                              { create: true });
    for (const [name, content] of bundleFiles(bundle))
      await writeFile(dir, name, content);
    return dir.name;
  }

  const CRC32_TABLE = (() => {
    const table = new Uint32Array(256);
    for (let n = 0; n < table.length; n++) {
      let c = n;
      for (let k = 0; k < 8; k++)
        c = (c & 1) ? (0xedb88320 ^ (c >>> 1)) : (c >>> 1);
      table[n] = c >>> 0;
    }
    return table;
  })();

  async function crc32(blob) {
    let crc = 0xffffffff;
    const reader = blob.stream().getReader();
    for (;;) {
      const { value, done } = await reader.read();
      if (done) break;
      for (let i = 0; i < value.length; i++)
        crc = CRC32_TABLE[(crc ^ value[i]) & 0xff] ^ (crc >>> 8);
    }
    return (crc ^ 0xffffffff) >>> 0;
  }

  function zipTimestamp(iso) {
    let d = new Date(iso);
    if (!Number.isFinite(d.getTime())) d = new Date();
    const year = Math.max(1980, Math.min(2107, d.getUTCFullYear()));
    return {
      time: (d.getUTCHours() << 11) | (d.getUTCMinutes() << 5) |
            (d.getUTCSeconds() >>> 1),
      date: ((year - 1980) << 9) | ((d.getUTCMonth() + 1) << 5) |
            d.getUTCDate(),
    };
  }

  function zipLocalHeader(entry) {
    const out = new Uint8Array(30 + entry.name.length);
    const v = new DataView(out.buffer);
    v.setUint32(0, 0x04034b50, true);
    v.setUint16(4, 20, true);
    v.setUint16(6, 0x0800, true);
    v.setUint16(8, 0, true);
    v.setUint16(10, entry.time, true);
    v.setUint16(12, entry.date, true);
    v.setUint32(14, entry.crc, true);
    v.setUint32(18, entry.size, true);
    v.setUint32(22, entry.size, true);
    v.setUint16(26, entry.name.length, true);
    out.set(entry.name, 30);
    return out;
  }

  function zipCentralHeader(entry) {
    const out = new Uint8Array(46 + entry.name.length);
    const v = new DataView(out.buffer);
    v.setUint32(0, 0x02014b50, true);
    v.setUint16(4, 20, true);
    v.setUint16(6, 20, true);
    v.setUint16(8, 0x0800, true);
    v.setUint16(10, 0, true);
    v.setUint16(12, entry.time, true);
    v.setUint16(14, entry.date, true);
    v.setUint32(16, entry.crc, true);
    v.setUint32(20, entry.size, true);
    v.setUint32(24, entry.size, true);
    v.setUint16(28, entry.name.length, true);
    v.setUint32(42, entry.offset, true);
    out.set(entry.name, 46);
    return out;
  }

  async function archiveBundle(bundle) {
    const base = bundleName(bundle.meta);
    const stamp = zipTimestamp(bundle.meta.date);
    const encoder = new TextEncoder();
    const entries = [];
    const parts = [];
    let offset = 0;

    for (const [leaf, content] of bundleFiles(bundle)) {
      const data = content instanceof Blob ? content : new Blob([content]);
      if (data.size > 0xffffffff)
        throw new Error(`trial file too large for ZIP: ${leaf}`);
      const entry = {
        name: encoder.encode(`${base}/${leaf}`),
        size: data.size,
        crc: await crc32(data),
        offset,
        ...stamp,
      };
      const header = zipLocalHeader(entry);
      entries.push(entry);
      parts.push(header, data);
      offset += header.length + data.size;
      if (offset > 0xffffffff)
        throw new Error('trial bundle too large for ZIP');
    }

    const centralOffset = offset;
    for (const entry of entries) {
      const header = zipCentralHeader(entry);
      parts.push(header);
      offset += header.length;
    }
    const centralSize = offset - centralOffset;
    if (offset > 0xffffffff)
      throw new Error('trial bundle too large for ZIP');

    const end = new Uint8Array(22);
    const v = new DataView(end.buffer);
    v.setUint32(0, 0x06054b50, true);
    v.setUint16(8, entries.length, true);
    v.setUint16(10, entries.length, true);
    v.setUint32(12, centralSize, true);
    v.setUint32(16, centralOffset, true);
    parts.push(end);
    return new Blob(parts, { type: 'application/zip' });
  }

  async function saveViaDownload(bundle) {
    const name = `${bundleName(bundle.meta)}.zip`;
    const blob = await archiveBundle(bundle);
    const a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = name;
    a.click();
    setTimeout(() => URL.revokeObjectURL(a.href), 10000);
    return name;
  }

  async function save(bundle) {
    if (window.showDirectoryPicker) {
      try { return await saveToDirectory(bundle); }
      catch { /* picker blocked (no user activation on auto-stop) or
                 write failed — downloads need no gesture */ }
    }
    return saveViaDownload(bundle);
  }

  return { start, recording, tick, event, marker, stop, save,
           bundleName, notesTemplate };
})();
/* audio.js attaches its global explicitly; do the same so automation and
 * the console can reach window.I76Trial, not just script scope. */
window.I76Trial = I76Trial;
