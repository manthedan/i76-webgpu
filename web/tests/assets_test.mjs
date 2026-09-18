// assets_test.mjs — unit gate for web/assets.js routing (pure part).
// Run from web/: node tests/assets_test.mjs
//
// classify()/plan() decide where every dropped/picked file lands in
// MEMFS; a wrong route silently breaks the engine (missions live at
// "miss8/<name>" in the VFS). These checks pin the routing contract.
import { classify, plan, attach, stageFiles, has, restoreStaged,
         clearStaged, persistNotice, persistIdle } from '../assets.js';

let failures = 0;
function check(name, cond, detail = '') {
  if (cond) console.log(`ok   ${name}${detail ? ' — ' + detail : ''}`);
  else { console.error(`FAIL ${name}${detail ? ' — ' + detail : ''}`); failures++; }
}

// core archives land at the VFS root regardless of walk depth
check('nitro.zfs at root', classify({ name: 'nitro.zfs', path: 'nitro.zfs' }) === 'nitro.zfs');
check('nitro.zix nested', classify({ name: 'nitro.zix', path: 'app/nitro/app/nitro.zix' }) === 'nitro.zix');
check('case-insensitive archive', classify({ name: 'NITRO.ZFS', path: 'app/NITRO.ZFS' }) === 'nitro.zfs');
check('base I76.ZIX nested',
      classify({ name: 'I76.ZIX', path: 'base/app/I76.ZIX' }) === 'i76.zix');
check('base I76.ZFS nested',
      classify({ name: 'I76.ZFS', path: 'base/app/I76.ZFS' }) === 'i76.zfs');

// missions keep their directory name (engine paths are "miss8/N01.CBT")
check('miss8 file', classify({ name: 'N01.CBT', path: 'app/miss8/N01.CBT' }) === 'miss8/N01.CBT');
check('miss16 file', classify({ name: 'X01.RAC', path: 'miss16/X01.RAC' }) === 'miss16/X01.RAC');
check('deeply nested miss8', classify({ name: 'p01.msn', path: 'a/b/miss8/p01.msn' }) === 'miss8/p01.msn');

// music rips
check('music track', classify({ name: 'track01.mp3', path: 'app/music/track01.mp3' }) === 'music/track01.mp3');
check('Smacker cutscene',
      classify({ name: 'INTROF01.SMK', path: 'app/smk/INTROF01.SMK' })
        === 'smk/INTROF01.SMK');
check('bare manual Smacker file',
      classify({ name: 'OUT01.SMK', path: 'OUT01.SMK' }) === 'smk/OUT01.SMK');

// loose HUD fonts (ship outside the zfs; the HUD status line needs them)
check('font at app root', classify({ name: 'base6x7.fnt', path: 'app/base6x7.fnt' }) === 'base6x7.fnt');
check('font case-insensitive', classify({ name: 'BASE6X74.FNT', path: 'app/BASE6X74.FNT' }) === 'base6x74.fnt');
check('Nitro shell art database routes from the app root',
      classify({ name: 'database.tvf', path: 'app/database.tvf' }) === 'database.tvf');
check('Nitro shell art database is case-insensitive',
      classify({ name: 'DATABASE.TVF', path: 'app/DATABASE.TVF' }) === 'database.tvf');
check('Nitro authored portrait-key table routes from the app root',
      classify({ name: 'nitcar.def', path: 'app/nitcar.def' }) === 'nitcar.def');
check('Nitro authored portrait-key table is case-insensitive',
      classify({ name: 'NITCAR.DEF', path: 'app/NITCAR.DEF' }) === 'nitcar.def');

// the campaign file (loose in addon/, not inside nitro.zfs). The engine
// opens it as "addon/scenario.dat" and fs_fopen retries the BASENAME's
// case only, so the directory segment must be lowercase and the basename
// must keep the case it had on disk.
check('campaign file routes to addon/',
      classify({ name: 'SCENARIO.DAT', path: 'app/addon/SCENARIO.DAT' })
        === 'addon/SCENARIO.DAT');
check('campaign file with a lowercase dir',
      classify({ name: 'SCENARIO.DAT', path: 'nitro/ADDON/SCENARIO.DAT' })
        === 'addon/SCENARIO.DAT');
// Episode / boot PCX title cards (paper UI path) route; other addon junk stays out.
check('addon episode load screen routes',
      classify({ name: 'P01LOAD.PCX', path: 'app/addon/P01LOAD.PCX' })
        === 'addon/P01LOAD.PCX');
check('loose loadgame.pcx routes',
      classify({ name: 'loadgame.pcx', path: 'app/loadgame.pcx' })
        === 'loadgame.pcx');
check('Lee handwritten font routes',
      classify({ name: 'lee_____.ttf', path: 'app/lee_____.ttf' })
        === 'lee_____.ttf');
// Paper surfaces normally come from nitro.zfs, but an extracted/loose GOG
// folder must retain the VFS basenames paperui.c opens.
check('scenario route map routes at VFS root',
      classify({ name: 'ZMAP6P01.VQM', path: 'app/ZMAP6P01.VQM' })
        === 'ZMAP6P01.VQM');
check('notepad art routes at VFS root',
      classify({ name: 'znpd6301.vqm', path: 'app/znpd6301.vqm' })
        === 'znpd6301.vqm');
check('escape pix index routes at VFS root',
      classify({ name: '6escape.pix', path: 'app/6escape.pix' })
        === '6escape.pix');
check('escape pak routes at VFS root',
      classify({ name: '3ESCAPE.PAK', path: 'app/3ESCAPE.PAK' })
        === '3ESCAPE.PAK');
check('unrelated VQM remains ignored',
      classify({ name: 'zradf000.vqm', path: 'app/zradf000.vqm' }) === null);
check('addon vcf ignored',
      classify({ name: 'vehscn.vcf', path: 'app/addon/vehscn.vcf' }) === null);
// a bare SCENARIO.DAT drop has no directory and so no home
check('bare campaign file has no route',
      classify({ name: 'SCENARIO.DAT', path: 'SCENARIO.DAT' }) === null);

// everything else is ignored (exe, dll, docs, bare loose files)
for (const p of ['nitro.exe', 'app/glide2x.dll', 'app/miss8.TXT', 'README.md',
                 'app/goggame-1207661023.dll'])
  check(`ignored: ${p}`, classify({ name: p.split('/').pop(), path: p }) === null);

// flat file drops carry only the filename: a bare .cbt has no home
check('bare dropped file has no route',
      classify({ name: 'N01.CBT', path: 'N01.CBT' }) === null);

// plan(): first-wins on duplicate routes, ignored counted
const files = [
  { name: 'nitro.zfs', path: 'a/nitro.zfs', file: 'first' },
  { name: 'nitro.zfs', path: 'b/nitro.zfs', file: 'dupe' },
  { name: 'junk.exe', path: 'a/junk.exe', file: 'junk' },
];
const { routes, ignored } = plan(files);
check('plan dedupes routes', routes.get('nitro.zfs').file === 'first' && routes.size === 1);
check('plan counts ignored', ignored === 1);

// ---- staging + persistence guards (node: no browser, no indexedDB) ------
// The byte-persistence layer (H-UAT-016 rehydration) must degrade to a
// no-op where indexedDB does not exist — staging still works, restore
// answers 0, clear never throws, and no failure note is raised.
const written = new Map();
attach({ FS: { mkdirTree: () => {},
               writeFile: (p, b) => written.set(p, b),
               readFile: (p) => written.get(p) } });
const mkFile = (bytes, name) => ({
  name, size: bytes.length,
  arrayBuffer: async () => Uint8Array.from(bytes).buffer,
});
const res = await stageFiles([
  { name: 'nitro.zfs', path: 'app/nitro.zfs', file: mkFile([1, 2], 'nitro.zfs') },
  { name: 'N01.CBT', path: 'app/miss8/N01.CBT', file: mkFile([3], 'N01.CBT') },
  { name: 'junk.exe', path: 'app/junk.exe', file: mkFile([4], 'junk.exe') },
]);
check('stageFiles stages routed files without indexedDB',
      res.staged === 2 && res.ignored === 1 &&
      has('nitro.zfs') && has('miss8/N01.CBT'),
      JSON.stringify(res));
const again = await stageFiles([
  { name: 'nitro.zfs', path: 'b/nitro.zfs', file: mkFile([9], 'nitro.zfs') }]);
check('re-staging an already-staged dest is skipped', again.skipped === 1,
      JSON.stringify(again));
check('restoreStaged answers 0 without indexedDB',
      (await restoreStaged()) === 0);
await clearStaged();                       // must not throw
await persistIdle();                       // must resolve, not hang
check('persistence degrades silently (no failure note)',
      persistNotice() === '');

process.exit(failures ? 1 : 0);
