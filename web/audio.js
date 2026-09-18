/*
 * audio.js — M5 WebAudio SFX output for the wasm engine (AudioPlay).
 *
 * NOT wired into index.html yet — Main adds:
 *   <script src="audio.js"></script>          (before the module script)
 *   I76Audio.attach(ModuleInstance);          (after I76Web() resolves)
 *
 * Model: the wasm side (src/engine/sound.c via webmain.c exports) tracks
 * WHAT is playing; this file owns actual output. Once per tick we pull
 * web_sound_tick()'s JSON and diff it against our live WebAudio nodes:
 *
 *   {"ones":[[id,"name"],...],"eng":["wav",pitch,gain,active]}
 *
 *   - a one-shot id we don't have  -> start a buffer source for it
 *   - an id of ours missing there  -> stop it (sound_stop / slot steal /
 *                                     expiry all surface this way)
 *   - eng.active                   -> one looping source, playbackRate =
 *                                     pitch, gain = gain (DECISION curve
 *                                     0.8+rpm/6000 / 0.3+0.7*load)
 *
 * PCM format: sound.c normalizes authored 8/16-bit PCM mono to 8-bit
 * UNSIGNED PCM at its authored sample rate (0x80 = silence). WebAudio wants
 * float samples in [-1,1), so the conversion is
 *
 *   f[i] = (bytes[i] - 128) / 128
 *
 * (/128, not /127: maps 0x00 exactly to -1.0 and keeps 0x80 at 0; the
 * 1-LSB asymmetry at full scale is inaudible — DECISION.)
 *
 * String-taking wasm exports MUST go through ccall/cwrap with a
 * ["string"] arg type — direct Module._web_sound_load("x") passes the
 * JS string object as a numeric pointer (0) and silently fails.
 *
 * Console self-test (after attach):  await I76Audio.selfTest()
 */
(function (global) {
'use strict';

var mod = null;              /* the emscripten module (I76Web instance) */
var ctx = null;              /* AudioContext, created on attach/unlock  */
var master = null;           /* master GainNode -> destination          */
var timer = null;            /* sync interval handle                    */

var buffers = {};            /* lc name -> AudioBuffer | Promise        */
var shots = {};              /* id -> {src, name, dead}                 */
var engine = null;           /* {src, gain, wav, buf, offset, rate}      */
var engGen = 0;              /* generation counter vs engine restarts   */
var audioPaused = false;     /* Play Options pause; movies do not use it */

var TICK_MS = 50;            /* sync cadence                            */
var movieSources = [];       /* scheduled Smacker PCM chunks             */
var movieAudioAt = 0;        /* AudioContext time after the last chunk    */

/* ------------------------------------------------------------------ */
/* AudioContext + autoplay-policy unlock                               */
/* ------------------------------------------------------------------ */

function ensureCtx() {
    if (ctx) return ctx;
    var Ctor = global.AudioContext || global.webkitAudioContext;
    if (!Ctor) return null;
    ctx = new Ctor();
    master = ctx.createGain();
    master.gain.value = 1.0;
    /* PORT DECISION (H-UAT-034): a soft limiter between master and the
     * destination. Linked fire sums one full-scale muzzle sample per
     * gun in phase (+10-12 dB for 3-4 guns) and the browser hard-clips
     * the overflow. Native mixing behavior (one sound per volley vs per
     * gun, per-instance volumes) is an open RE question; until decoded,
     * this only prevents digital clipping and does not reshape levels
     * below the threshold. */
    if (typeof ctx.createDynamicsCompressor === 'function') {
        var limiter = ctx.createDynamicsCompressor();
        limiter.threshold.value = -8;
        limiter.knee.value = 6;
        limiter.ratio.value = 20;
        /* H-UAT-034 update: 2 ms attack let the percussive muzzle
         * transient through before gain reduction engaged — owner heard
         * residual clipping on good headphones. Instant attack plus a
         * soft-knee saturator behind it: the shaper rounds whatever
         * per-sample peak the compressor's envelope still misses, and
         * is unity-slope (inaudible) below ~-6 dB. */
        limiter.attack.value = 0;
        limiter.release.value = 0.1;
        var shaper = ctx.createWaveShaper();
        var curve = new Float32Array(1024);
        for (var ci = 0; ci < 1024; ci++) {
            var x = (ci / 511.5) - 1;
            curve[ci] = Math.tanh(x * 1.3) / Math.tanh(1.3);
        }
        shaper.curve = curve;
        shaper.oversample = '2x';
        master.connect(limiter);
        limiter.connect(shaper);
        shaper.connect(ctx.destination);
    } else {
        /* Headless/mock contexts (node tests) skip the limiter. */
        master.connect(ctx.destination);
    }
    return ctx;
}

/* Browsers gate audio on a user gesture: resume (and create, on the
 * first gesture) from pointer/key/touch handlers. Handlers stay
 * registered — a suspended context can reappear (tab switch). */
function onGesture() {
    if (!ensureCtx()) return;
    if (ctx.state === 'suspended')
        Promise.resolve(ctx.resume()).then(resumeMusic);
    else
        resumeMusic();
}

/* ------------------------------------------------------------------ */
/* Wav fetching: wasm heap -> Float32 AudioBuffer                      */
/* ------------------------------------------------------------------ */

function loadBuffer(name) {
    var key = String(name).toLowerCase();
    var have = buffers[key];
    if (have) return Promise.resolve(have);

    var p = Promise.resolve().then(function () {
        var len = mod.ccall('web_sound_load', 'number', ['string'], [key]);
        if (!len) return null;               /* missing/invalid wav */
        /* Synchronous load-then-read: the no-arg form returns the PCM
         * of the load that just happened. */
        var ptr = mod._web_sound_pcm_ptr();
        var rate = mod._web_sound_rate();
        if (!ptr || !(rate > 0)) return null;
        var bytes = mod.HEAPU8.slice(ptr, ptr + len);   /* copy out */
        var f = new Float32Array(len);
        for (var i = 0; i < len; i++) f[i] = (bytes[i] - 128) / 128;
        var buf = ctx.createBuffer(1, len, rate);
        if (buf.copyToChannel) buf.copyToChannel(f, 0);
        else buf.getChannelData(0).set(f);
        buffers[key] = buf;
        return buf;
    });
    buffers[key] = p;
    return p;
}

/* ------------------------------------------------------------------ */
/* One-shot sync                                                       */
/* ------------------------------------------------------------------ */

function wrappedOffset(buf, offset, loop) {
    var duration = buf && buf.duration;
    if (!(duration > 0)) return Math.max(0, offset || 0);
    if (!loop) return Math.max(0, offset || 0);
    offset %= duration;
    return offset < 0 ? offset + duration : offset;
}

function startShotSource(id, entry) {
    if (entry.dead || audioPaused || !entry.buf ||
        !ctx || ctx.state !== 'running') return;
    if (entry.buf.duration > 0 && entry.offset >= entry.buf.duration) {
        if (shots[id] === entry) delete shots[id];
        return;
    }
    var src = ctx.createBufferSource();
    src.buffer = entry.buf;
    src.connect(master);
    src.onended = function () {
        if (shots[id] === entry) delete shots[id];
    };
    entry.src = src;
    entry.startedAt = ctx.currentTime;
    src.start(0, entry.offset);
}

function startShot(id, name) {
    var entry = { src: null, name: name, dead: false, buf: null,
                  offset: 0, startedAt: 0 };
    shots[id] = entry;
    loadBuffer(name).then(function (buf) {
        if (entry.dead) return;
        if (!buf) { delete shots[id]; return; }
        entry.buf = buf;
        if (!audioPaused && ctx.state !== 'running') {
            delete shots[id];
            return;
        }
        startShotSource(id, entry);
    });
}

function pauseShot(entry) {
    if (!entry.src) return;
    entry.offset += Math.max(0, ctx.currentTime - entry.startedAt);
    entry.src.onended = null;
    try { entry.src.stop(); } catch (e) { /* already ended */ }
    entry.src = null;
}

function stopShot(id) {
    var entry = shots[id];
    if (!entry) return;
    entry.dead = true;
    if (entry.src) {
        entry.src.onended = null;
        try { entry.src.stop(); } catch (e) { /* already ended */ }
    }
    delete shots[id];
}

/* ------------------------------------------------------------------ */
/* Engine loop                                                         */
/* ------------------------------------------------------------------ */

function captureEngineOffset() {
    if (!engine || !engine.src) return;
    engine.offset += Math.max(0, ctx.currentTime - engine.startedAt) * engine.rate;
    engine.offset = wrappedOffset(engine.buf, engine.offset, true);
    engine.startedAt = ctx.currentTime;
}

function startEngineSource() {
    if (!engine || engine.src || audioPaused ||
        !ctx || ctx.state !== 'running') return;
    var src = ctx.createBufferSource();
    src.buffer = engine.buf;
    src.loop = true;
    src.connect(engine.gain);
    src.playbackRate.value = engine.rate;
    engine.src = src;
    engine.offset = wrappedOffset(engine.buf, engine.offset, true);
    engine.startedAt = ctx.currentTime;
    src.start(0, engine.offset);
}

function pauseEngine() {
    if (!engine || !engine.src) return;
    captureEngineOffset();
    try { engine.src.stop(); } catch (e) { /* already ended */ }
    engine.src = null;
}

function stopEngine() {
    engGen++;
    if (!engine) return;
    if (engine.src) {
        try { engine.src.stop(); } catch (e) { /* already ended */ }
    }
    engine = null;
}

function syncEngine(eng) {
    if (!eng || eng.length === 0 || !eng[3]) { stopEngine(); return; }
    var wav = eng[0], pitch = eng[1], gain = eng[2];

    if (engine && engine.wav !== wav) stopEngine();
    if (!engine) {
        var gen = ++engGen;
        loadBuffer(wav).then(function (buf) {
            if (gen !== engGen || engine) return;   /* restarted meanwhile */
            if (!buf) return;
            var g = ctx.createGain();
            g.connect(master);
            engine = { src: null, gain: g, wav: wav, buf: buf,
                       offset: 0, startedAt: 0, rate: pitch };
            g.gain.value = gain;
            startEngineSource();
        });
        return;
    }
    /* Account for time at the old pitch before changing playbackRate. */
    captureEngineOffset();
    engine.rate = pitch;
    if (engine.src) engine.src.playbackRate.value = pitch;
    engine.gain.gain.value = gain;
}

/* ------------------------------------------------------------------ */
/* Tick: pull the wasm playing-state and reconcile                     */
/* ------------------------------------------------------------------ */

function tick() {
    /* The wasm owns SFX state. While Play Options is open, leave our
     * reconciliation snapshot untouched: no one-shot expiry/removal or
     * engine restart is inferred while native has the SFX mixer paused. */
    if (!mod || !ctx || audioPaused) return;
    var st;
    try {
        st = JSON.parse(mod.ccall('web_sound_tick', 'string', [], []));
    } catch (e) {
        return;              /* module mid-teardown; try next tick */
    }

    var seen = {};
    for (var i = 0; i < st.ones.length; i++) {
        var id = st.ones[i][0], name = st.ones[i][1];
        seen[id] = 1;
        if (!shots[id]) startShot(id, name);
    }
    for (var idKey in shots)
        if (!seen[idKey]) stopShot(idKey);

    syncEngine(st.eng);
}

/* ------------------------------------------------------------------ */
/* Music: user-supplied redbook rips staged under /data/music           */
/* ------------------------------------------------------------------ */
/*
 * Selection is by the staged rip's CD TRACK NUMBER, never by playlist
 * position. A strict <integer>.mp3 basename builds musicByTrack:
 *
 *   FACT (docs/specs/re/music-system.md): Nitro's shell starts track 10
 *   and loops it. Tracks 0 and 1 are never native authored starts; GOG's
 *   0.mp3 is byte-identical to 2.mp3.
 *
 *   PORT INTERIM (H-UAT-037): drive rotates staged tracks 2..11 until the
 *   mission's authored WRLD payload-u32[0] track reaches this player.
 *   TODO: replace rotation with that one authored track when the scene.c
 *   WRLD decode lands; see docs/specs/re/music-system.md sections 1 and 6.
 *
 * Shell falls back from an unstaged/undecodable 10 to the lowest usable
 * track >=2. Only when no such track exists may the GOG-only track 0 alias
 * be used. Drive never admits 0, so 0.mp3/2.mp3 cannot duplicate a song in
 * one rotation. ONE buffer source runs through a dedicated gain under the
 * SFX master (bounded at MUSIC_GAIN), gesture-gated like the rest of the
 * context, and stopped by detach(). Bytes come from MEMFS, never network;
 * decode failures are reported through onMusicStatus and skipped.
 */

var MUSIC_GAIN = 0.7;      /* bounded under the SFX master (1.0)        */

var musicRead = null;      /* fn(name) -> Promise<Uint8Array|null>      */
var musicReport = null;    /* fn({state, track, error})                 */
var musicList = [];        /* mapped numeric .mp3 basenames, track order */
var musicByTrack = {};     /* integer track number -> staged basename   */
var musicNumbers = [];     /* available track numbers, ascending        */
var musicModeNow = null;   /* 'shell' | 'drive' | null (null = stopped) */
var musicNow = null;       /* {src, name, track, gen, loop}             */
var musicGen = 0;          /* generation vs async read/decode races     */
var musicLoading = -1;     /* track number in async read/decode          */
var musicPending = -1;     /* track number waiting on a running ctx     */
var musicGainNode = null;  /* GainNode -> master                        */
var musicBad = {};         /* name -> true: undecodable, reported       */
var shellTrackOverride = -1; /* dev-panel session override; reset by mode */

function musicLoop(mode) { return mode !== 'drive'; }

function musicTrackNumber(name) {
    var m = /^(\d+)\.mp3$/i.exec(String(name));
    if (!m) return -1;
    var n = Number(m[1]);
    return Number.isSafeInteger(n) ? n : -1;
}

/* Mode preference in track numbers. Shell's track 10 is native FACT;
 * its remaining order is the explicit missing-track fallback above. */
function musicCandidates(mode) {
    var out = [];
    if (mode === 'drive') {
        for (var i = 0; i < musicNumbers.length; i++) {
            var driveTrack = musicNumbers[i];
            if (driveTrack >= 2 && driveTrack <= 11) out.push(driveTrack);
        }
        return out;
    }
    if (musicByTrack[10] !== undefined) out.push(10);
    for (var j = 0; j < musicNumbers.length; j++) {
        var shellTrack = musicNumbers[j];
        if (shellTrack >= 2 && shellTrack !== 10) out.push(shellTrack);
    }
    /* Never choose the duplicate merely because it sorted first. This is
     * the documented last resort for a stage containing only 0.mp3. */
    if (musicByTrack[0] !== undefined) out.push(0);
    return out;
}

function musicTarget(mode) {
    if (mode === 'shell' && shellTrackOverride >= 0) {
        var overrideName = musicByTrack[shellTrackOverride];
        if (overrideName !== undefined && !musicBad[overrideName])
            return shellTrackOverride;
        shellTrackOverride = -1;
    }
    /* Drive rotation is stateful after genuine mode entry. Reconciliation
     * must retain a usable current/in-flight track instead of snapping to
     * the first candidate (2). setMusicMode() stops the prior mode before
     * calling here, so these states can only belong to this drive entry. */
    if (mode === 'drive') {
        var retained = musicNow ? musicNow.track
                     : musicLoading >= 0 ? musicLoading : musicPending;
        var retainedName = musicByTrack[retained];
        if (retainedName !== undefined && !musicBad[retainedName])
            return retained;
    }
    var candidates = musicCandidates(mode);
    for (var i = 0; i < candidates.length; i++) {
        var track = candidates[i];
        if (!musicBad[musicByTrack[track]]) return track;
    }
    return -1;
}

function reportMusic(st) {
    if (musicReport) {
        try { musicReport(st); } catch (e) { /* page handler is advisory */ }
    }
}

/* Stop the current source (if any) and invalidate pending decodes.
 * The one-source guarantee lives here: every start goes through this. */
function musicStop() {
    musicGen++;
    musicLoading = -1;
    musicPending = -1;
    if (musicNow && musicNow.src) {
        musicNow.src.onended = null;
        try { musicNow.src.stop(); } catch (e) { /* already ended */ }
    }
    musicNow = null;
}

function musicCurrentOffset(entry) {
    var offset = entry.offset;
    if (entry.src)
        offset += Math.max(0, ctx.currentTime - entry.startedAt);
    return wrappedOffset(entry.buf, offset, entry.loop);
}

function musicStartSource(entry) {
    if (!entry || entry.src || audioPaused ||
        !ctx || ctx.state !== 'running') {
        if (entry && (!ctx || ctx.state !== 'running'))
            musicPending = entry.track;
        return;
    }
    var offset = musicCurrentOffset(entry);
    if (!entry.loop && entry.buf.duration > 0 && offset >= entry.buf.duration) {
        musicAdvanceFrom(entry.track);
        return;
    }
    if (!musicGainNode) {
        musicGainNode = ctx.createGain();
        musicGainNode.gain.value = MUSIC_GAIN;
        musicGainNode.connect(master);
    }
    var src = ctx.createBufferSource();
    src.buffer = entry.buf;
    src.loop = entry.loop;
    src.connect(musicGainNode);
    src.onended = function () {
        if (musicNow === entry && !entry.loop) musicAdvance();
    };
    entry.src = src;
    entry.offset = offset;
    entry.startedAt = ctx.currentTime;
    entry.paused = false;
    musicPending = -1;
    src.start(0, offset);
    reportMusic({ state: 'playing', track: entry.name });
}

function musicPause() {
    if (!musicNow) return;
    if (musicNow.src) {
        musicNow.offset = musicCurrentOffset(musicNow);
        musicNow.src.onended = null;
        try { musicNow.src.stop(); } catch (e) { /* already ended */ }
        musicNow.src = null;
    }
    musicNow.paused = true;
    reportMusic({ state: 'paused', track: musicNow.name });
}

function musicStart(track) {
    musicStop();
    var gen = musicGen;
    var name = musicByTrack[track];
    if (name === undefined || !musicRead) return;
    /* Autoplay policy: never start a source on a suspended context.
     * onGesture() re-runs syncMusic() once resume() resolves. */
    if (!ctx || ctx.state !== 'running') { musicPending = track; return; }
    musicLoading = track;
    reportMusic({ state: 'loading', track: name });
    Promise.resolve()
        .then(function () { return musicRead(name); })
        .then(function (bytes) {
            if (gen !== musicGen) return null;
            if (!bytes || !bytes.byteLength)
                throw new Error('no data for ' + name);
            /* decodeAudioData detaches its input: hand it a private copy. */
            var ab = bytes.buffer.slice(bytes.byteOffset,
                                        bytes.byteOffset + bytes.byteLength);
            return new Promise(function (res, rej) {
                /* The callback form ALSO returns a promise in modern
                 * browsers; left alone its rejection surfaces as an
                 * unhandled pageerror on top of our handled one. */
                var req = ctx.decodeAudioData(ab, res, rej);
                if (req && req.catch) req.catch(function () {});
            });
        })
        .then(function (buf) {
            if (gen !== musicGen || !buf) return;   /* superseded/stopped */
            musicLoading = -1;
            if (musicModeNow === null) return;
            if (ctx.state !== 'running') { musicPending = track; return; }
            var loop = musicLoop(musicModeNow);
            musicNow = { src: null, name: name, track: track, gen: gen,
                         loop: loop, buf: buf, offset: 0, startedAt: 0,
                         paused: audioPaused };
            if (audioPaused)
                reportMusic({ state: 'paused', track: name });
            else
                musicStartSource(musicNow);
        })
        .catch(function (err) {
            if (gen !== musicGen) return;
            musicLoading = -1;
            /* Unsupported/undecodable: say so (the page surfaces it),
             * remember it so it is never retried, and move on. */
            if (!musicBad[name]) {
                musicBad[name] = true;
                reportMusic({ state: 'unsupported', track: name,
                              error: String((err && err.message) || err) });
            }
            musicAdvanceFrom(track);
        });
}

/* Next usable numbered shell track after `from`, wrapping while excluding
 * the GOG 0 alias and data track 1. Unlike shell's authored-start preference,
 * this is numeric order because it backs an explicit dev-panel skip. */
function musicNextNumbered(from) {
    var candidates = [];
    for (var i = 0; i < musicNumbers.length; i++)
        if (musicNumbers[i] >= 2) candidates.push(musicNumbers[i]);
    var len = candidates.length;
    if (!len) return -1;
    var pos = candidates.indexOf(from);
    for (var hop = 1; hop <= len; hop++) {
        var track = candidates[(pos + hop) % len];
        if (!musicBad[musicByTrack[track]]) return track;
    }
    return -1;
}

/* Next usable drive track after `from`, wrapping within 2..11. Authored
 * shell failures re-run its preference list; dev shell overrides continue
 * in numeric order. */
function musicNext(from) {
    if (musicModeNow !== 'drive') {
        if (shellTrackOverride >= 0) {
            var shellNext = musicNextNumbered(from);
            shellTrackOverride = shellNext;
            return shellNext;
        }
        return musicTarget(musicModeNow);
    }
    var candidates = musicCandidates('drive');
    var len = candidates.length;
    if (!len) return -1;
    var pos = candidates.indexOf(from);
    for (var hop = 1; hop <= len; hop++) {
        var track = candidates[(pos + hop) % len];
        if (!musicBad[musicByTrack[track]]) return track;
    }
    return -1;
}

function musicAdvanceFrom(from) {
    var next = musicNext(from);
    if (next < 0) { musicStop(); return; }
    musicStart(next);
}

function musicAdvance() {
    musicAdvanceFrom(musicNow ? musicNow.track : -1);
}

/* DEV-PANEL ONLY — not a native Interstate '76 control. The native decode
 * exposes Music Level as the only user music control and no track-skip action
 * (docs/specs/re/music-system.md section 3). Drive deliberately reuses the
 * interim rotation's existing advance path. Shell gets a session-only numeric
 * override that loops like authored shell music; leaving the mode or reloading
 * restores shell=10. musicStart() first calls musicStop(), so a paused skip
 * cannot carry the old track's saved offset into the replacement. */
function nextTrack() {
    if (!musicModeNow) return false;
    if (musicModeNow === 'drive') {
        if (!musicCandidates('drive').length) return false;
        musicAdvance();
        return true;
    }
    var from = musicNow ? musicNow.track : musicPending;
    var next = musicNextNumbered(from);
    if (next < 0) return false;
    shellTrackOverride = next;
    musicStart(next);
    return true;
}

/* Reconcile the running source with (mode, list, bad-set). Called from
 * every setter and from onGesture once the context is running. */
function syncMusic() {
    if (!musicModeNow || !musicList.length) {
        if (musicNow || musicPending >= 0) musicStop();
        return;
    }
    var target = musicTarget(musicModeNow);
    if (target < 0) { musicStop(); return; }
    var loop = musicLoop(musicModeNow);
    /* Already on the right numbered track with the right loop behaviour:
     * same-mode re-entry must not restart or duplicate the source. */
    if (musicNow && musicNow.track === target &&
        musicNow.name === musicByTrack[target] && musicNow.loop === loop) {
        if (!musicNow.src && musicNow.paused && !audioPaused)
            musicStartSource(musicNow);
        return;
    }
    if (musicLoading === target)
        return;                                   /* async decode owns it */
    if (musicPending === target && (!ctx || ctx.state !== 'running'))
        return;                                   /* gesture will start it */
    musicStart(target);
}

/* Page supplies the byte source (MEMFS read-back; never the network). */
function setMusicSource(reader) {
    musicRead = reader;
    syncMusic();
}

/* Parse staged basenames into the number->name map. First occurrence wins
 * if a stage contains aliases such as 02.mp3 and 2.mp3; Assets.musicOrder
 * makes that choice stable for an unchanged staged set. Non-MP3/non-numeric
 * names and track 1 remain staged but are not playable music candidates. */
function setMusicTracks(names) {
    var supplied = (names || []).slice();
    var byTrack = {};
    for (var i = 0; i < supplied.length; i++) {
        var track = musicTrackNumber(supplied[i]);
        if (track >= 0 && byTrack[track] === undefined)
            byTrack[track] = supplied[i];
    }
    var numbers = Object.keys(byTrack).map(Number).sort(function (a, b) {
        return a - b;
    });
    var list = numbers.map(function (n) { return byTrack[n]; });
    var same = numbers.length === musicNumbers.length;
    if (same) {
        for (var j = 0; j < numbers.length; j++) {
            var oldTrack = musicNumbers[j];
            if (numbers[j] !== oldTrack || byTrack[numbers[j]] !== musicByTrack[oldTrack]) {
                same = false;
                break;
            }
        }
    }
    if (same) return;
    var still = {};
    for (var k = 0; k < list.length; k++) still[list[k]] = 1;
    for (var bad in musicBad)
        if (!still[bad]) delete musicBad[bad];
    musicByTrack = byTrack;
    musicNumbers = numbers;
    musicList = list;
    syncMusic();
}

/* 'shell' | 'drive' | null. Same-mode re-entry is a no-op (no restart,
 * no duplicate source). null stops output; list/mode survive detach(). */
function setMusicMode(mode) {
    if (mode !== 'shell' && mode !== 'drive') mode = null;
    if (mode === musicModeNow) return;
    /* A genuine mode entry owns a fresh authored start. Do not let the
     * current/loading track retained for same-drive sync cross modes. */
    musicStop();
    shellTrackOverride = -1;
    musicModeNow = mode;
    syncMusic();
}

function onMusicStatus(fn) { musicReport = fn; }

/* Resume a paused current track directly rather than reselecting the first
 * drive candidate. This is what preserves an advanced drive track across
 * ESC; ordinary mode/list changes still route through syncMusic(). */
function resumeMusic() {
    if (audioPaused) return;
    if (musicNow && musicNow.paused) {
        if (!musicModeNow || musicNow.name !== musicByTrack[musicNow.track] ||
            musicNow.loop !== musicLoop(musicModeNow)) {
            syncMusic();
            return;
        }
        musicStartSource(musicNow);
        return;
    }
    syncMusic();
}

/* Native Play Options pauses both MCI music and the SFX mixer. BufferSource
 * nodes cannot pause independently, so retain elapsed offsets/state, stop the
 * live nodes, and create replacements on resume. Smacker never calls this
 * seam and keeps its existing page-owned audio lifecycle. */
function setPaused(next) {
    next = !!next;
    if (next === audioPaused) return;
    audioPaused = next;
    if (audioPaused) {
        musicPause();
        for (var id in shots) pauseShot(shots[id]);
        pauseEngine();
        return;
    }
    for (var shotId in shots) startShotSource(shotId, shots[shotId]);
    startEngineSource();
    resumeMusic();
    tick();
}


/* Smacker audio arrives as decoded interleaved PCM, one chunk per video
 * frame. Keep it on the existing context/master and schedule chunks
 * back-to-back so setTimeout jitter cannot put gaps in the soundtrack. */
function playPcm(bytes, rate, channels, depth) {
    if (!bytes || !bytes.byteLength || !ensureCtx() ||
        ctx.state !== 'running' || !(rate > 0) ||
        (channels !== 1 && channels !== 2) ||
        (depth !== 8 && depth !== 16))
        return false;
    var stride = channels * (depth / 8);
    var frames = Math.floor(bytes.byteLength / stride);
    if (!frames) return false;
    var buf = ctx.createBuffer(channels, frames, rate);
    for (var ch = 0; ch < channels; ch++) {
        var dst = buf.getChannelData(ch);
        for (var i = 0; i < frames; i++) {
            var off = i * stride + ch * (depth / 8);
            if (depth === 8) {
                dst[i] = (bytes[off] - 128) / 128;
            } else {
                var v = bytes[off] | (bytes[off + 1] << 8);
                if (v & 0x8000) v -= 0x10000;
                dst[i] = v / 32768;
            }
        }
    }
    var src = ctx.createBufferSource();
    src.buffer = buf;
    src.connect(master);
    var start = Math.max(ctx.currentTime, movieAudioAt);
    movieAudioAt = start + buf.duration;
    movieSources.push(src);
    src.onended = function () {
        var i = movieSources.indexOf(src);
        if (i >= 0) movieSources.splice(i, 1);
    };
    src.start(start);
    return true;
}

function stopMovieAudio() {
    for (var i = 0; i < movieSources.length; i++) {
        movieSources[i].onended = null;
        try { movieSources[i].stop(); } catch (e) { /* already ended */ }
    }
    movieSources = [];
    movieAudioAt = 0;
}

/* Wire the module + gesture unlock + the sync interval. Idempotent. */
function attach(M, opts) {
    mod = M;
    ensureCtx();
    var evts = ['pointerdown', 'keydown', 'touchstart'];
    for (var i = 0; i < evts.length; i++)
        global.addEventListener(evts[i], onGesture);
    if (!timer && !(opts && opts.manualTick))
        timer = global.setInterval(tick, (opts && opts.tickMs) || TICK_MS);
    return api;
}

/* Stop all output and the sync timer (module state untouched). */
function detach() {
    if (timer) { global.clearInterval(timer); timer = null; }
    audioPaused = false;
    for (var id in shots) stopShot(id);
    stopEngine();
    musicStop();
    stopMovieAudio();
}

/* Feed the engine loop (page: call per frame with car rpm/load). */
function setEngine(rpm, load) {
    return mod.ccall('web_sound_engine', 'number', ['number', 'number'],
                     [rpm, load]);
}

/* Kill the engine loop now (mission exit, H-UAT-038): clear the wasm
 * active flag so the 50 ms sync cannot revive it, and stop the local
 * source immediately. Guarded: older wasm builds lack the export. */
function engineStop() {
    if (mod && typeof mod._web_sound_engine_stop === 'function')
        mod._web_sound_engine_stop();
    stopEngine();
}

/* Fire a one-shot from the page (returns the wasm one-shot id). */
function play(name) {
    return mod.ccall('web_sound_play', 'number', ['string'], [name]);
}

/* ------------------------------------------------------------------ */
/* Console self-test: await I76Audio.selfTest()                        */
/* ------------------------------------------------------------------ */

async function selfTest(M) {
    if (M) mod = M;
    var failures = 0;
    var log = function (ok, label, detail) {
        console.log((ok ? 'ok   ' : 'FAIL ') + label +
                    (detail ? ' — ' + detail : ''));
        if (!ok) failures++;
    };

    log(!!mod, 'module bound (attach() or selfTest(M) first)');
    if (!mod) { console.log('AUDIO SELFTEST: FAIL'); return failures; }

    ['web_sound_load', 'web_sound_pcm_ptr', 'web_sound_rate',
     'web_sound_play', 'web_sound_stop', 'web_sound_engine', 'web_sound_tick']
        .forEach(function (x) {
            log(typeof mod['_' + x] === 'function', 'export ' + x);
        });

    log(!!ensureCtx(), 'AudioContext available');
    if (!ctx) { console.log('AUDIO SELFTEST: FAIL'); return failures; }
    console.log('info AudioContext.state=' + ctx.state +
                ' (suspended is fine headless; a gesture resumes it)');

    /* Plain-RIFF speech wav through the exports. */
    var len = mod.ccall('web_sound_load', 'number', ['string'],
                        ['01cor01.wav']);
    log(len > 0, 'speech wav 01cor01.wav loads', 'len=' + len);

    /* engsnd.dat name resolved to the GAS0-wrapped .gpw entry. */
    var elen = mod.ccall('web_sound_load', 'number', ['string'],
                         ['einp1.wav']);
    log(elen > 0, 'engine wav einp1.wav loads via .gpw swap',
        'len=' + elen);

    /* 8-bit -> Float32 conversion sanity on the engine buffer. */
    var buf = await loadBuffer('einp1.wav');
    var f0 = buf && buf.getChannelData(0)[0];
    log(!!buf && buf.length === elen && buf.sampleRate === 11025 &&
        Math.abs(f0 - (0x80 - 128) / 128) < 1e-7,
        'PCM converts to Float32 (8-bit unsigned, 11025 Hz mono)',
        buf ? 'len=' + buf.length + ' f0=' + f0.toFixed(4) : 'no buffer');

    /* Engine curve + tick JSON contract. */
    var rc = mod.ccall('web_sound_engine', 'number', ['number', 'number'],
                       [1200, 0.5]);
    var st = JSON.parse(mod.ccall('web_sound_tick', 'string', [], []));
    log(rc === 0 && st.eng[0] === 'einp1.wav' &&
        Math.abs(st.eng[1] - 1.0) < 1e-3 &&    /* 0.8 + 1200/6000 */
        Math.abs(st.eng[2] - 0.65) < 1e-3 &&   /* 0.3 + 0.7*0.5   */
        st.eng[3] === 1,
        'engine state: einp1.wav, pitch=1.000, gain=0.650, active',
        JSON.stringify(st.eng));

    var id = mod.ccall('web_sound_play', 'number', ['string'],
                       ['vhorn1.wav']);
    st = JSON.parse(mod.ccall('web_sound_tick', 'string', [], []));
    log(id > 0 && st.ones.some(function (o) { return o[0] === id; }),
        'one-shot appears in tick JSON', 'id=' + id);

    /* Live sync smoke (only meaningful with a running context). */
    tick();
    if (ctx.state === 'running') {
        await new Promise(function (r) { global.setTimeout(r, 120); });
        log(!!shots[id] || true,
            'sync ran without exception (source may already have ended)');
        log(!!engine && engine.wav === 'einp1.wav',
            'engine loop source running');
    } else {
        console.log('info ctx not running — output-node checks skipped ' +
                    '(click the page, then re-run selfTest)');
    }

    console.log('AUDIO SELFTEST: ' + (failures ? 'FAIL' : 'PASS'));
    return failures;
}

var api = {
    attach: attach,
    detach: detach,
    tick: tick,
    play: play,
    setEngine: setEngine,
    engineStop: engineStop,
    selfTest: selfTest,
    /* music: user-supplied tracks from /data/music (see above) */
    setMusicSource: setMusicSource,
    setMusicTracks: setMusicTracks,
    setMusicMode: setMusicMode,
    nextTrack: nextTrack,
    setPaused: setPaused,
    onMusicStatus: onMusicStatus,
    playPcm: playPcm,
    stopMovieAudio: stopMovieAudio,
    /* introspection for debugging */
    _state: function () {
        return { ctx: ctx && ctx.state, paused: audioPaused,
                 shots: Object.keys(shots), engine: engine && engine.wav,
                 buffers: Object.keys(buffers),
                 movieSources: movieSources.length };
    },
    _musicState: function () {
        var byTrack = {};
        for (var i = 0; i < musicNumbers.length; i++)
            byTrack[musicNumbers[i]] = musicByTrack[musicNumbers[i]];
        return { mode: musicModeNow, tracks: musicList.slice(), byTrack: byTrack,
                 playing: musicNow && musicNow.name,
                 track: musicNow ? musicNow.track : -1,
                 loop: musicNow ? musicNow.loop : false,
                 paused: audioPaused,
                 offset: musicNow ? musicCurrentOffset(musicNow) : 0,
                 loading: musicLoading, pending: musicPending,
                 bad: Object.keys(musicBad) };
    }
};

global.I76Audio = api;

})(typeof window !== 'undefined' ? window : globalThis);
