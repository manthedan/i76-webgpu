// save.js — campaign progress and player preferences that survive a reload.
//
// WHAT THE ORIGINAL SAVED. save.h records it: the retail shell wrote
// save###.cmp + savegame.dir BETWEEN missions — campaign bookmark, garage,
// selected vehicle. It never snapshotted a running mission, and neither do
// we: there is no in-mission save/resume here and none is planned.
//
// WHAT THIS STORES. Names and outcomes only, never asset bytes: every
// mission path, vehicle name and preference re-resolves through the user's
// own game files on the next visit. A profile is a few hundred bytes.
//
// WHERE. Its own IndexedDB database. assets.js owns 'i76-assets' v1 (the
// remembered folder handle); a separate database means neither has to bump
// the other's version to evolve. localStorage is kept in step for the two
// preferences that already lived there, so a downgrade still finds them.
//
// THE RULE THAT SHAPES THE ERROR HANDLING: persistence must never block
// play. Every failure path here ends with a playable page and a one-line
// notice — a missing database, a denied quota, a private-mode browser, a
// record written by a newer build, or a corrupt record.

export const SCHEMA = 1;

const DB_NAME = 'i76-save', STORE = 'profiles', ID = 'default';

function fresh() {
  return {
    id: ID,
    schema: SCHEMA,
    app: 'i76-web',
    updated: new Date().toISOString(),
    // Mission PATHS, not indices: the campaign file decides the order and
    // an index would silently point at a different mission if it changed.
    completed: [],
    // path -> last terminal outcome. Every outcome is recorded, including
    // the aborted/failed ones; only a completion enters `completed`.
    results: {},
    // The player's current car, ALWAYS defined — native Nitro persists a
    // shell-selected chassis in nituser.mel (row+variant, fresh default
    // row 0 variant 1 = valepre1) and missions spawn that car, never a
    // per-mission rack (docs/specs/re/p01-loadout-selection.md). The
    // catalog is vcf-keyed here, so the seed is the resolved vcf stem.
    garage: { vcf: 'valepre1' },
    prefs: { renderer: 'sw', view: 'cockpit' },
  };
}

// Shape check, not a schema library: reject anything a later read would
// throw on, accept anything harmless. Unknown keys are preserved.
function valid(p) {
  return !!p && typeof p === 'object' &&
         Number.isInteger(p.schema) &&
         Array.isArray(p.completed) &&
         p.completed.every((x) => typeof x === 'string') &&
         !!p.results && typeof p.results === 'object' &&
         !!p.garage && typeof p.garage.vcf === 'string' &&
         !!p.prefs && typeof p.prefs === 'object';
}

function openDb() {
  return new Promise((res, rej) => {
    const rq = indexedDB.open(DB_NAME, 1);
    rq.onupgradeneeded = () =>
      rq.result.createObjectStore(STORE, { keyPath: 'id' });
    rq.onsuccess = () => res(rq.result);
    rq.onerror = () => rej(rq.error);
  });
}

function tx(mode, fn) {
  return openDb().then((db) => new Promise((res) => {
    const t = db.transaction(STORE, mode);
    const rq = fn(t.objectStore(STORE));
    t.oncomplete = () => res(rq && 'result' in rq ? rq.result : true);
    t.onerror = () => res(null);
    t.onabort = () => res(null);
  }));
}

let cur = fresh();
let note = '';
let readOnly = false;      // a newer build's record: never overwrite it
let timer = null;

export function profile() { return cur; }
export function notice() { return note; }

export async function load() {
  note = '';
  readOnly = false;
  let rec = null;
  try {
    rec = await tx('readonly', (s) => s.get(ID));
  } catch {
    cur = fresh();
    readOnly = true;                       // no store to write to
    note = 'progress cannot be saved in this browser session.';
    migratePrefs();
    return cur;
  }

  if (!rec) { cur = fresh(); migratePrefs(); return cur; }

  if (Number.isInteger(rec.schema) && rec.schema > SCHEMA) {
    // A newer build wrote this. Overwriting would destroy progress the
    // user can still reach by going back to that build.
    cur = fresh();
    readOnly = true;
    note = `saved progress was written by a newer build (format ` +
           `${rec.schema} > ${SCHEMA}) — left untouched; this session ` +
           `will not be saved.`;
    migratePrefs();
    return cur;
  }

  if (!valid(rec)) {
    // Keep the evidence, start clean. Quarantine failing is not fatal.
    try {
      await tx('readwrite', (s) => {
        s.put({ ...rec, id: `${ID}.corrupt.${Date.now()}` });
        s.delete(ID);
      });
    } catch { /* the fresh profile below still plays */ }
    cur = fresh();
    note = 'saved progress could not be read and was set aside; ' +
           'starting a fresh profile.';
    migratePrefs();
    return cur;
  }

  cur = { ...fresh(), ...rec, id: ID, schema: SCHEMA };
  // Migration: profiles written before shell-selection modeling used
  // garage.vcf === '' to mean "the mission decides". Native Nitro has no
  // such state — the shell always owns a car (nituser.mel, fresh default
  // row 0 variant 1 = valepre1). Seed the native default; a later garage
  // visit re-seeds the real catalog pick if this stem is absent.
  if (!cur.garage.vcf) cur.garage.vcf = 'valepre1';
  return cur;
}

/* The renderer and camera choices predate this module and lived in
 * localStorage. Adopt them into a first profile so an existing visitor
 * does not lose the view they had. */
function migratePrefs() {
  try {
    const r = localStorage.getItem('i76.renderer');
    const v = localStorage.getItem('i76.view');
    if (r) cur.prefs.renderer = r;
    if (v) cur.prefs.view = v;
  } catch { /* storage disabled — defaults stand */ }
}

/* Debounced write. Called from exactly two places in the page: entering a
 * debrief and committing a garage/preference change. Never per frame. */
export function touch(delay = 400) {
  if (readOnly) return;
  if (timer) clearTimeout(timer);
  timer = setTimeout(() => { timer = null; flush(); }, delay);
}

export async function flush() {
  if (readOnly) return false;
  if (timer) { clearTimeout(timer); timer = null; }
  cur.updated = new Date().toISOString();
  try {
    return (await tx('readwrite', (s) => s.put(cur))) !== null;
  } catch {
    readOnly = true;
    note = 'progress could not be written; this session will not be saved.';
    return false;
  }
}

/*
 * Start the campaign over: clear progress, keep preferences and the
 * garage (they are not progress). Cleared IN PLACE — the page holds a
 * reference to this same object, and swapping it for a new one would
 * leave the page writing to a profile nothing persists.
 */
export async function reset() {
  cur.completed.length = 0;
  for (const k of Object.keys(cur.results)) delete cur.results[k];
  await flush();
}
