import assert from 'node:assert/strict';
import { mkdtempSync, symlinkSync, readFileSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { externalPath, newOutputDir, newOutputFile, loopbackPage } from './external_paths.mjs';

const root = fileURLToPath(new URL('../..', import.meta.url));
const tmp = mkdtempSync(path.join(tmpdir(), 'i76-path-controls-'));
try {
  assert.throws(() => externalPath(root, 'fixture'), /outside/);
  assert.throws(() => externalPath(path.join(root, 'not-created'), 'fixture'), /outside/);
  assert.throws(() => externalPath('relative', 'fixture'), /absolute/);
  symlinkSync(root, path.join(tmp, 'source-link'));
  assert.throws(() => externalPath(path.join(tmp, 'source-link/web'), 'fixture'), /outside/);
  assert.throws(() => newOutputDir(path.join(tmp, 'source-link/not-created')), /outside/);
  const output = path.join(tmp, 'new-output');
  newOutputDir(output);
  assert.throws(() => newOutputDir(output), /EEXIST/);
  const file = path.join(output, 'result.json');
  newOutputFile(file);
  assert.throws(() => newOutputFile(file), /EEXIST/);
  assert.equal(readFileSync(file).length, 0);
  assert.equal(loopbackPage('http://127.0.0.1:8123/probe'), 'http://127.0.0.1:8123/probe');
  for (const url of ['https://example.com/probe', 'http://127.0.0.1.example.com/', 'file:///tmp/probe'])
    assert.throws(() => loopbackPage(url), /loopback/);
  // Dummy userinfo tests the same boundary without a credential-shaped literal.
  const credentialed = new URL('http://localhost/');
  credentialed.username = 'user';
  credentialed.password = 'pass';
  assert.throws(() => loopbackPage(credentialed.href), /loopback/);
  console.log('external path, fresh-output and loopback-page controls: PASS');
} finally {
  // Only disposable synthetic test fixtures, never purchaser inputs/evidence.
  rmSync(tmp, { recursive: true, force: true });
}
