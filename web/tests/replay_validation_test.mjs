// CLI rejection controls only. Requires the asset-free Wasm build, no game files.
import assert from 'node:assert/strict';
import { mkdtempSync, mkdirSync, writeFileSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { spawnSync } from 'node:child_process';

const root = mkdtempSync(path.join(tmpdir(), 'i76-replay-controls-'));
const script = fileURLToPath(new URL('../trial_replay.mjs', import.meta.url));
try {
  for (const [tag, name, extra, expected] of [
    ['events', 'events.jsonl', [], /invalid trial input events.jsonl/],
    ['markers', 'markers.jsonl', [], /invalid trial input markers.jsonl/],
    ['tape', 'tape.jsonl', [], /invalid trial input tape.jsonl/],
    ['tolerance', null, ['--tolerance', 'NaN'], /tolerance must be finite/],
  ]) {
    const bundle = path.join(root, tag);
    mkdirSync(bundle);
    const files = {
      'meta.json': JSON.stringify({ kind: 'i76-human-trial', v: 1 }),
      'tape.jsonl': '{"tick":0}\n',
    };
    if (name) files[name] = 'not JSON\n';
    for (const [file, text] of Object.entries(files)) writeFileSync(path.join(bundle, file), text);
    const run = spawnSync(process.execPath,
      [script, bundle, '--output', path.join(root, tag + '-output'), ...extra],
      { encoding: 'utf8', timeout: 15000 });
    assert.ifError(run.error);
    assert.notEqual(run.status, 0, run.stdout + run.stderr);
    assert.match(run.stderr, expected);
    for (const [file, text] of Object.entries(files))
      assert.equal(readFileSync(path.join(bundle, file), 'utf8'), text);
  }
  console.log('replay malformed-input, tolerance and recording-immutability controls: PASS');
} finally {
  rmSync(root, { recursive: true, force: true });
}
