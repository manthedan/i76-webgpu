// Shared CLI boundaries for purchaser-data diagnostics, not runtime code.
import { closeSync, mkdirSync, openSync, realpathSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const ROOT = realpathSync(fileURLToPath(new URL('../..', import.meta.url)));
export const inside = (parent, child) => child === parent || child.startsWith(parent + path.sep);

// Resolve existing ancestors even when the output itself does not yet exist.
function canonical(name) {
  try { return realpathSync(name); }
  catch (error) {
    if (error.code !== 'ENOENT') throw error;
    const parent = path.dirname(name);
    if (parent === name) throw error;
    return path.join(canonical(parent), path.basename(name));
  }
}

export function externalPath(name, label, { existing = false } = {}) {
  if (!name || !path.isAbsolute(name)) throw new Error(`${label} must be absolute`);
  const actual = existing ? realpathSync(name) : canonical(path.resolve(name));
  if (inside(ROOT, actual)) throw new Error(`${label} must be outside the checkout`);
  return actual;
}

export function newOutputDir(name) {
  const actual = externalPath(name, 'output');
  mkdirSync(path.dirname(actual), { recursive: true });
  mkdirSync(actual, { mode: 0o700 }); // Deliberately not recursive: EEXIST fails.
  return actual;
}

export function newOutputFile(name) {
  const actual = externalPath(name, 'output');
  mkdirSync(path.dirname(actual), { recursive: true });
  closeSync(openSync(actual, 'wx', 0o600));
  return actual;
}

export function loopbackPage(name) {
  const url = new URL(name);
  if (url.protocol !== 'http:' || !['localhost', '127.0.0.1', '[::1]'].includes(url.hostname)
      || url.username || url.password) throw new Error('probe page must be loopback HTTP');
  return url.href;
}
