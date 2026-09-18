// music_test.mjs — unit gate for the browser music playlist.
// Run from web/: node tests/music_test.mjs
//
// Two halves:
//   1. assets.js musicOrder — deterministic staged-basename ordering.
//   2. audio.js numbered-track state machine against a mock AudioContext:
//      strict basename parsing, native shell track 10, interim drive
//      rotation over 2..11 without the GOG 0/2 duplicate, missing-10
//      fallback, decode-fail skipping, position-preserving pause/resume,
//      dev-only next-track semantics, paused SFX deferral, and clean teardown.
//
// Shell track 10 is FACT. Drive rotation is a PORT INTERIM until scene.c
// exposes WRLD's authored per-mission track (docs/specs/re/music-system.md).

import { musicOrder } from '../assets.js';

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log(`ok   ${name}${detail ? ' — ' + detail : ''}`);
  else { console.error(`FAIL ${name}${detail ? ' — ' + detail : ''}`); failures++; }
}

// ---- 1. playlist ordering (pure) ----------------------------------------

check('numeric stems sort as CD track numbers',
  JSON.stringify(musicOrder(['2.mp3', '10.mp3', '0.mp3', '11.mp3', '3.mp3'])) ===
  JSON.stringify(['0.mp3', '2.mp3', '3.mp3', '10.mp3', '11.mp3']));
check('non-numeric names follow, case-insensitive',
  JSON.stringify(musicOrder(['bonus.mp3', '7.mp3', 'Anthem.wav'])) ===
  JSON.stringify(['7.mp3', 'Anthem.wav', 'bonus.mp3']));
check('input array is not mutated',
  (() => { const a = ['9.mp3', '1.mp3']; musicOrder(a); return a[0] === '9.mp3'; })());

// ---- 2. audio.js state machine (mock WebAudio) ---------------------------

const listeners = {};
globalThis.addEventListener = (ev, fn) => { (listeners[ev] ??= []).push(fn); };

const sources = [];   // every started buffer source, in start order
let mockCtx = null;
class MockParam { constructor(v) { this.value = v; } }
class MockGain {
  constructor() { this.gain = new MockParam(1); }
  connect(node) { this.output = node; }
}
class MockSource {
  constructor() {
    this.loop = false;
    this.onended = null;
    this.stopped = false;
    this.playbackRate = new MockParam(1);
  }
  connect(node) { this.output = node; }
  start(when = 0, offset = 0) {
    this.startWhen = when;
    this.startOffset = offset;
    sources.push(this);
  }
  stop() { this.stopped = true; }
}
class MockCtx {
  constructor() {
    this.state = 'suspended';
    this.destination = {};
    this.currentTime = 0;
    mockCtx = this;
  }
  createGain() { return new MockGain(); }
  createBufferSource() { return new MockSource(); }
  createBuffer(ch, len, sr) {
    return { length: len, sampleRate: sr, duration: len / sr,
             copyToChannel() {},
             getChannelData() { return new Float32Array(len); } };
  }
  decodeAudioData(ab, res, rej) {
    // first byte 0 = a format this "browser" cannot decode
    if (new Uint8Array(ab)[0] === 0) rej(new Error('unsupported format'));
    else res({ duration: 20 });
  }
  resume() { this.state = 'running'; return Promise.resolve(); }
}
globalThis.AudioContext = MockCtx;

await import('../audio.js');   // classic script: sets globalThis.I76Audio
const A = globalThis.I76Audio;
check('I76Audio exposed', !!A && typeof A.setMusicMode === 'function' &&
  typeof A.nextTrack === 'function');

const flush = async () => {
  for (let i = 0; i < 6; i++) await new Promise((r) => setTimeout(r, 0));
};
const gesture = async () => {
  for (const f of listeners.pointerdown || []) f();
  await flush();
};
const live = () => sources.filter((s) => !s.stopped).length;

const events = [];
const decodeBad = new Set(['3.mp3']);
const readGone = new Set(['4.mp3']);
A.onMusicStatus((s) => events.push(s));
A.attach({}, { manualTick: true });
const musicReader = (name) => Promise.resolve(
  readGone.has(name) ? null
  : decodeBad.has(name) ? new Uint8Array([0, 0, 0])
  : new Uint8Array([1, 2, 3]));
A.setMusicSource(musicReader);
A.setMusicTracks([
  '11.mp3', 'bonus.mp3', '0.mp3', '10.mp3', '2.mp3', '3.mp3',
  '4.mp3', '1.mp3', '12.mp3'
]);

let state = A._musicState();
check('numeric MP3 basenames build a number-to-name map',
  state.byTrack[0] === '0.mp3' && state.byTrack[10] === '10.mp3' &&
  state.byTrack[12] === '12.mp3' && !Object.values(state.byTrack).includes('bonus.mp3'),
  JSON.stringify(state.byTrack));

// gesture-gated: mode set on a suspended context starts nothing
A.setMusicMode('shell');
await flush();
check('no source before the gesture', sources.length === 0);
check('shell track 10 is pending on the gesture', A._musicState().pending === 10,
  JSON.stringify(A._musicState()));

await gesture();
check('shell plays native track 10 by number, not playlist position',
  A._musicState().playing === '10.mp3' && A._musicState().track === 10,
  JSON.stringify(A._musicState()));
check('shell track loops', A._musicState().loop === true);
check('exactly one live source', live() === 1, `live=${live()}`);
check('music gain is bounded under the master',
  sources[0].output.gain.value > 0 && sources[0].output.gain.value <= 0.7,
  `gain=${sources[0].output.gain.value}`);

// Play Options pauses the live source and resumes the same decoded buffer at
// its accumulated offset. It must not route through mode=null and restart 0.
mockCtx.currentTime = 7.25;
A.setPaused(true);
state = A._musicState();
check('pause stores elapsed music offset',
  sources[0].stopped && Math.abs(state.offset - 7.25) < 1e-9,
  JSON.stringify(state));
const beforeResume = sources.length;
A.setPaused(false);
await flush();
check('resume creates one source on the same track at the saved offset',
  sources.length === beforeResume + 1 && A._musicState().track === 10 &&
  Math.abs(sources.at(-1).startOffset - 7.25) < 1e-9,
  `sources=${sources.length} offset=${sources.at(-1).startOffset}`);
check('ESC resume never restarts the track from zero',
  sources.at(-1).startOffset !== 0);

// Shell music loops. Accumulated time crossing the buffer duration wraps
// before it is supplied to AudioBufferSourceNode.start().
mockCtx.currentTime = 27.25;
A.setPaused(true);
state = A._musicState();
check('shell-loop pause offset wraps by buffer duration',
  Math.abs(state.offset - 7.25) < 1e-9, `offset=${state.offset}`);
A.setPaused(false);
await flush();
check('shell-loop resume starts at the wrapped offset',
  Math.abs(sources.at(-1).startOffset - 7.25) < 1e-9,
  `offset=${sources.at(-1).startOffset}`);

// same-mode re-entry must not restart or duplicate
const shellSourcesAfterResume = sources.length;
A.setMusicMode('shell');
await flush();
check('mode re-entry does not duplicate sources',
  live() === 1 && sources.length === shellSourcesAfterResume);

// shell -> drive: starts numbered track 2, excluding tracks 0/1 and >11
A.setMusicMode('drive');
await flush();
check('drive starts at numbered track 2',
  A._musicState().playing === '2.mp3' && A._musicState().track === 2,
  JSON.stringify(A._musicState()));
check('drive track does not loop', A._musicState().loop === false);
check('shell source stopped, one live source',
  sources[0].stopped && live() === 1, `live=${live()}`);

// The dev skip reuses drive's ordinary advance path: it must move beyond
// the current track rather than restarting that track at offset zero. Decode
// failures on the path are still skipped WITH reports.
const driveTrack2Source = sources.at(-1);
check('drive nextTrack accepts the current mode', A.nextTrack() === true);
await flush();
state = A._musicState();
check('drive nextTrack advances instead of restarting track 2 from zero',
  driveTrack2Source.stopped && state.playing === '10.mp3' && state.track === 10,
  JSON.stringify(state));
const bad = events.filter((e) => e.state === 'unsupported').map((e) => e.track);
check('undecodable tracks are reported, not silent',
  bad.includes('3.mp3') && bad.includes('4.mp3'), bad.join(','));
check('drive never selects duplicate track 0', state.playing !== '0.mp3');
check('still exactly one live source after skipping', live() === 1);

// Any page reconciliation after a drive advance must be idempotent. Before
// H-UAT-051, setMusicSource() called stateless musicTarget('drive'), selected
// the first candidate (2), and superseded this advanced track.
const driveSourcesBeforeSync = sources.length;
A.setMusicSource(musicReader);
await flush();
check('explicit sync retains the advanced drive track',
  A._musicState().track === 10 && A._musicState().playing === '10.mp3',
  JSON.stringify(A._musicState()));
check('explicit sync does not restart the retained source',
  sources.length === driveSourcesBeforeSync && live() === 1,
  `sources=${sources.length} live=${live()}`);

// The current drive track may have advanced past the mode's first candidate.
// ESC must resume that exact track/position, not reselect track 2 at offset 0.
mockCtx.currentTime = 31.75;
const driveBeforePause = sources.at(-1);
A.setPaused(true);
state = A._musicState();
check('drive pause stores the advanced track offset',
  driveBeforePause.stopped && state.track === 10 &&
  Math.abs(state.offset - 4.5) < 1e-9, JSON.stringify(state));
A.setPaused(false);
await flush();
check('drive resume preserves advanced track and position',
  A._musicState().track === 10 && Math.abs(sources.at(-1).startOffset - 4.5) < 1e-9,
  JSON.stringify(A._musicState()));
check('drive ESC does not restart first candidate from zero',
  A._musicState().track !== 2 && sources.at(-1).startOffset !== 0);

// Reconciliation while the replacement is still decoding must retain that
// in-flight target and must not launch a second read/decode. This pins the
// null-musicNow interval where gesture/page sync used to supersede an advance.
let releaseTrack11;
let track11Reads = 0;
const heldReader = (name) => {
  if (name === '11.mp3') {
    track11Reads++;
    return new Promise((resolve) => { releaseTrack11 = resolve; });
  }
  return musicReader(name);
};
A.setMusicSource(heldReader);
sources[sources.length - 1].onended();
await new Promise((resolve) => setTimeout(resolve, 0));
check('natural advance retains track 11 while its decode is pending',
  A._musicState().loading === 11 && A._musicState().playing === null,
  JSON.stringify(A._musicState()));
A.setMusicSource(heldReader);
await new Promise((resolve) => setTimeout(resolve, 0));
check('sync during decode does not restart or revert the in-flight track',
  track11Reads === 1 && A._musicState().loading === 11,
  `reads=${track11Reads} state=${JSON.stringify(A._musicState())}`);
releaseTrack11(new Uint8Array([1, 2, 3]));
await flush();
check('drive continues from 10 to 11 after the retained decode',
  A._musicState().track === 11 && A._musicState().playing === '11.mp3',
  JSON.stringify(A._musicState()));
A.setMusicSource(musicReader);
sources[sources.length - 1].onended();
await flush();
check('drive wraps from 11 to 2 without playing 0',
  A._musicState().track === 2 && A._musicState().playing === '2.mp3',
  JSON.stringify(A._musicState()));

// Returning to shell restores native track 10 and looping.
A.setMusicMode('shell');
await flush();
check('shell re-entry restores track 10 looping',
  A._musicState().track === 10 && A._musicState().loop === true,
  JSON.stringify(A._musicState()));
check('one live source across the mode switch', live() === 1);

// Skipping while paused must discard the old track's accumulated offset.
// The replacement remains paused at zero and starts there on resume.
mockCtx.currentTime = 37.75;
const shellTrack10Source = sources.at(-1);
A.setPaused(true);
check('paused shell track has an old saved offset before skip',
  Math.abs(A._musicState().offset - 6) < 1e-9,
  JSON.stringify(A._musicState()));
check('paused shell nextTrack accepts the current mode', A.nextTrack() === true);
await flush();
state = A._musicState();
check('paused skip clears the stale offset for the old track',
  shellTrack10Source.stopped && state.track === 11 && state.paused &&
  Math.abs(state.offset) < 1e-9,
  JSON.stringify(state));
check('paused skip reports the replacement track for the status line',
  events.at(-1)?.state === 'paused' && events.at(-1)?.track === '11.mp3',
  JSON.stringify(events.at(-1)));
A.setPaused(false);
await flush();
check('paused skip resumes the replacement from zero',
  A._musicState().track === 11 && sources.at(-1).startOffset === 0,
  JSON.stringify(A._musicState()));
check('resumed replacement reports now-playing name',
  events.at(-1)?.state === 'playing' && events.at(-1)?.track === '11.mp3',
  JSON.stringify(events.at(-1)));
check('shell nextTrack override loops the replacement track',
  A._musicState().loop === true && sources.at(-1).loop === true);

// Numeric shell skips wrap and never admit GOG's duplicate track 0.
A.nextTrack();
await flush();
check('shell nextTrack advances numerically to 12',
  A._musicState().track === 12, JSON.stringify(A._musicState()));
A.nextTrack();
await flush();
check('shell nextTrack wraps to 2 and excludes track 0',
  A._musicState().track === 2 && A._musicState().playing === '2.mp3',
  JSON.stringify(A._musicState()));

// The shell override is mode-local: leaving and re-entering restores the
// authored shell=10 selection rather than retaining the dev choice.
A.setMusicMode('drive');
await flush();
A.setMusicMode('shell');
await flush();
check('mode re-entry clears the session shell override',
  A._musicState().track === 10 && A._musicState().loop === true,
  JSON.stringify(A._musicState()));

// Unstaged 10: choose the lowest >=2, never the sorted-first 0 alias.
A.setMusicTracks(['7.mp3', '0.mp3', '2.mp3']);
await flush();
check('unstaged-10 shell fallback chooses lowest track >=2',
  A._musicState().track === 2 && A._musicState().playing === '2.mp3',
  JSON.stringify(A._musicState()));

// The documented last resort is an install containing only GOG's 0 alias.
A.setMusicTracks(['0.mp3']);
await flush();
check('shell uses 0 only when no canonical audio track is staged',
  A._musicState().track === 0 && A._musicState().playing === '0.mp3',
  JSON.stringify(A._musicState()));
A.setMusicMode('drive');
await flush();
check('drive excludes 0 even when it is the only staged file',
  A._musicState().playing === null && live() === 0,
  JSON.stringify(A._musicState()));

// Track 1 and nonnumeric names are never authored plays.
A.setMusicMode('shell');
A.setMusicTracks(['1.mp3', 'song.mp3']);
await flush();
check('track 1 and nonnumeric basenames produce no shell candidate',
  A._musicState().playing === null && live() === 0,
  JSON.stringify(A._musicState()));

// A mapped playlist of nothing but unplayable files stops, without spinning.
decodeBad.add('5.mp3');
readGone.add('6.mp3');
A.setMusicTracks(['5.mp3', '6.mp3']);
await flush();
check('all-unplayable numbered tracks stop playback',
  A._musicState().playing === null && live() === 0,
  JSON.stringify(A._musicState()));
check('all unplayable numbered tracks are reported',
  A._musicState().bad.includes('5.mp3') && A._musicState().bad.includes('6.mp3'));

// Fresh list recovers in the current shell mode.
A.setMusicTracks(['9.mp3']);
await flush();
check('new numbered track resumes playback',
  A._musicState().playing === '9.mp3' && A._musicState().track === 9);

// teardown: detach stops the music source along with everything else
A.detach();
check('detach stops the music source', live() === 0);
check('detach clears playing state', A._musicState().playing === null);

// re-attach after detach: list/mode survive, gesture starts it again
A.attach({}, { manualTick: true });
await gesture();
check('re-attach resumes the surviving numbered track',
  A._musicState().playing === '9.mp3', JSON.stringify(A._musicState()));

// ---- 3. SFX bridge honors pause and the authored sample rate ------------
A.setMusicMode(null);
A.detach();
const sfxBytes = new Uint8Array([0, 128, 255]);
let sfxTicks = 0;
const sfxMod = {
  HEAPU8: sfxBytes,
  ccall(name) {
    if (name === 'web_sound_load') return sfxBytes.length;
    if (name === 'web_sound_play') return 1;
    if (name === 'web_sound_tick') {
      sfxTicks++;
      return '{"ones":[[1,"rate.wav"]],"eng":["rate.wav",1,0.5,1]}';
    }
    return 0;
  },
  _web_sound_pcm_ptr() { return 0; },
  _web_sound_rate() { return 22050; },
};
// Pointer 0 is the wasm null sentinel, so place bytes at a nonzero offset.
sfxMod.HEAPU8 = new Uint8Array([0, ...sfxBytes]);
sfxMod._web_sound_pcm_ptr = () => 1;
A.attach(sfxMod, { manualTick: true });
A.setPaused(true);
const madeBefore = sources.length;
const ticksBefore = sfxTicks;
A.play('rate.wav');
A.tick();
await flush();
check('SFX sync is deferred while paused',
  sources.length === madeBefore && sfxTicks === ticksBefore,
  `sources=${sources.length - madeBefore} ticks=${sfxTicks - ticksBefore}`);

A.setPaused(false);
A.tick();
await flush();
const resumedSfx = sources.slice(madeBefore);
const rateSource = resumedSfx.find((s) => !s.loop);
const engineSource = resumedSfx.find((s) => s.loop);
check('deferred one-shot resumes from retained wasm state', !!rateSource);
check('engine loop resumes from retained wasm state', !!engineSource);
check('SFX buffer uses the engine-authored sample rate',
  rateSource && rateSource.buffer.sampleRate === 22050,
  rateSource ? `rate=${rateSource.buffer.sampleRate}` : 'no source');

const beforeSfxPause = sources.length;
A.setPaused(true);
check('pause stops live one-shot and engine sources without dropping state',
  rateSource.stopped && engineSource.stopped);
A.tick();
await flush();
check('paused ticks do not recreate SFX', sources.length === beforeSfxPause);
A.setPaused(false);
await flush();
check('unpause recreates one-shot and engine sources cleanly',
  sources.length === beforeSfxPause + 2 &&
  sources.slice(-2).some((s) => s.loop) && sources.slice(-2).some((s) => !s.loop));

process.exit(failures ? 1 : 0);
