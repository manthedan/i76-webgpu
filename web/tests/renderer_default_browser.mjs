#!/usr/bin/env node
/*
 * Opt-in real-browser smoke for the normal-page renderer default/fallback.
 *
 * Usage:
 *   NITRO_APP=/absolute/owned/app \
 *     node web/tests/renderer_default_browser.mjs /absolute/new/output-dir
 *
 * CHROME may name an explicit Chromium executable. I76_BROWSER_MODE may be
 * headless (default) or vulkan-headed (requires DISPLAY, e.g. xvfb-run).
 * Exit 0 is PASS, exit 1 is
 * FAIL, and exit 3 is SKIP when case 1 cannot acquire a WebGPU adapter.
 */
import puppeteer from 'puppeteer';
import { spawn } from 'node:child_process';
import { once } from 'node:events';
import {
  appendFileSync, closeSync, openSync, readFileSync, readdirSync, statSync,
  writeFileSync,
} from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { externalPath, loopbackPage, newOutputDir } from './external_paths.mjs';

process.umask(0o077);
const OUT_ARG = process.argv[2];
const APP_ARG = process.env.NITRO_APP;
if (!OUT_ARG || !APP_ARG) {
  console.error('usage: NITRO_APP=/owned/app node web/tests/renderer_default_browser.mjs <new-output-dir>');
  process.exit(1);
}

const OUT = newOutputDir(OUT_ARG);
const APP = externalPath(APP_ARG, 'NITRO_APP', { existing: true });
const CHROME = process.env.CHROME
  ? externalPath(process.env.CHROME, 'CHROME', { existing: true }) : null;
const WEB_DIR = fileURLToPath(new URL('../', import.meta.url));
const SERVER_TOOL = fileURLToPath(new URL('../../tools/serve_static.py', import.meta.url));
const READY_FILE = path.join(OUT, 'server-ready.json');
const RUN_LOG = path.join(OUT, 'run.log');
const BROWSER_MODE = process.env.I76_BROWSER_MODE || 'headless';
if (!['headless', 'vulkan-headed'].includes(BROWSER_MODE))
  throw new Error('I76_BROWSER_MODE must be headless or vulkan-headed');
const FLAGS = [
  '--no-sandbox', '--disable-dev-shm-usage', '--enable-unsafe-webgpu',
  '--enable-webgpu-developer-features', '--use-angle=vulkan',
  '--disable-background-timer-throttling', '--disable-renderer-backgrounding',
  '--disable-backgrounding-occluded-windows',
  ...(BROWSER_MODE === 'vulkan-headed' ? [
    '--enable-features=Vulkan', '--disable-vulkan-surface',
    '--ignore-gpu-blocklist', '--enable-gpu',
  ] : []),
];
const sleep = (ms) => new Promise((resolve) => setTimeout(resolve, ms));
const log = (...parts) => {
  const line = parts.map((part) => typeof part === 'string' ? part : JSON.stringify(part)).join(' ');
  console.log(line);
  appendFileSync(RUN_LOG, line + '\n');
};
const insist = (condition, message) => {
  if (!condition) throw new Error(message);
};

function namedFile(dir, wanted) {
  const entry = readdirSync(dir, { withFileTypes: true }).find((item) =>
    item.isFile() && item.name.toLowerCase() === wanted.toLowerCase());
  if (!entry) throw new Error(`asset prerequisite missing: ${wanted}`);
  return path.join(dir, entry.name);
}

insist(statSync(APP).isDirectory(), 'NITRO_APP must be a directory');
const missDir = path.join(APP, 'miss8');
insist(statSync(missDir).isDirectory(), 'asset prerequisite missing: miss8');
const assets = {
  zix: namedFile(APP, 'nitro.zix'),
  zfs: namedFile(APP, 'nitro.zfs'),
  campaign: namedFile(path.join(APP, 'addon'), 'SCENARIO.DAT'),
  missions: readdirSync(missDir, { withFileTypes: true })
    /* The whole folder, as the README instructs: missions also need their
     * loose .TER/.PCF files. */
    .filter((entry) => entry.isFile())
    .map((entry) => path.join(missDir, entry.name))
    .sort((a, b) => a.localeCompare(b)),
};
insist(assets.missions.some((name) => path.basename(name).toLowerCase() === 'p01.msn'),
       'asset prerequisite missing: miss8/P01.MSN');

async function startServer() {
  const serverLog = openSync(path.join(OUT, 'server.log'), 'wx', 0o600);
  const child = spawn('python3', [SERVER_TOOL, WEB_DIR, READY_FILE], {
    stdio: ['ignore', serverLog, serverLog],
  });
  closeSync(serverLog);
  let spawnError = null;
  child.on('error', (error) => { spawnError = error; });
  try {
    for (let i = 0; i < 100; i++) {
      if (spawnError) throw spawnError;
      if (child.exitCode !== null) throw new Error(`static server exited ${child.exitCode}`);
      try {
        const ready = JSON.parse(readFileSync(READY_FILE, 'utf8'));
        return { child, page: loopbackPage(new URL('/', ready.url).href) };
      } catch (error) {
        if (error.code !== 'ENOENT' && !(error instanceof SyntaxError)) throw error;
      }
      await sleep(100);
    }
    throw new Error('static server readiness timed out');
  } catch (error) {
    await stopServer(child);
    throw error;
  }
}

/* A signalled exit leaves exitCode null and sets signalCode instead. */
const hasExited = (child) => child.exitCode !== null || child.signalCode !== null;

async function stopServer(child) {
  /* A child that failed to spawn has no pid and may never emit 'exit'. */
  if (!child || child.pid === undefined || hasExited(child)) return;
  const exited = once(child, 'exit');
  child.kill('SIGTERM');
  await Promise.race([exited, sleep(3000)]);
  if (!hasExited(child)) {
    child.kill('SIGKILL');
    await Promise.race([exited, sleep(3000)]);
  }
}

async function uploadAndWait(page, selector, files, pattern) {
  const input = await page.$(selector);
  insist(input, `missing intake control ${selector}`);
  await input.uploadFile(...files);
  await page.waitForFunction((source) =>
    new RegExp(source, 'i').test(document.getElementById('stagedInfo')?.textContent || ''),
  { timeout: 400000 }, pattern);
}

async function stageAndEnterStockMission(page) {
  await page.waitForFunction(() => !document.getElementById('zixFile')?.disabled,
                             { timeout: 120000 });
  await page.waitForFunction(() =>
    document.documentElement.dataset.i76Restore !== undefined,
  { timeout: 240000 });
  const restore = await page.evaluate(() => document.documentElement.dataset.i76Restore);
  insist(restore === 'none', `browser context was not clean (restore=${restore})`);

  if (!await page.$eval('#assetManual', (details) => details.open))
    await page.click('#assetManual > summary');
  await uploadAndWait(page, '#zixFile', [assets.zix], 'need nitro\\.zfs');
  await uploadAndWait(page, '#zfsFile', [assets.zfs], 'Nitro Pack archive ready');
  await uploadAndWait(page, '#campFile', [assets.campaign], 'campaign');
  await uploadAndWait(page, '#dirInput', assets.missions, '[1-9]\\d* mission files');
  insist(await page.$eval('#loadBtn', (button) => !button.disabled),
         'Load staged assets remained disabled');
  await page.click('#loadBtn');
  await page.waitForFunction(() => !document.getElementById('shellPanel')?.hidden,
                             { timeout: 180000 });

  const first = await page.$('#missionList .missionRow:not([disabled])');
  insist(first, 'no stock mission is enabled');
  await first.click();
  await page.waitForFunction(() => !document.getElementById('shellBrief')?.hidden,
                             { timeout: 30000 });
  const title = await page.$eval('#briefTitle', (element) => element.textContent.trim());
  insist(/New Hope/i.test(title), `first stock mission is not New Hope: ${title}`);
  await page.click('#shellStartBtn');
  await page.waitForFunction(() => !document.getElementById('gamePanel')?.hidden,
                             { timeout: 120000 });
  await page.waitForFunction(() => (window.__i76?.frames || 0) >= 8,
                             { timeout: 30000 });
  await page.click('.gameViewport');
  await page.keyboard.press('Space');  // ordinary opening-sequence skip
  await page.waitForFunction(() => !document.getElementById('recoverBtn')?.disabled,
                             { timeout: 120000 });
  return { restore, mission: title, controlsReady: true };
}

async function pageState(page) {
  return page.evaluate(() => {
    const visible = (element) => {
      if (!element || element.hidden) return false;
      const style = getComputedStyle(element);
      const rect = element.getBoundingClientRect();
      return style.display !== 'none' && style.visibility !== 'hidden' &&
             Number(style.opacity || 1) !== 0 && rect.width > 0 && rect.height > 0;
    };
    const infoNodes = [
      document.getElementById('status'), document.getElementById('driveHint'),
      document.getElementById('missionInfo'), ...document.querySelectorAll('[role="status"], [aria-live]'),
    ];
    const fallback = infoNodes
      .filter((element, index, all) => element && all.indexOf(element) === index)
      .map((element) => (element.innerText || element.textContent || '').trim())
      .find((text) => /webgpu[\s\S]{0,200}(unavailable|failed)[\s\S]{0,200}software|software renderer[\s\S]{0,100}(fallback|in use)/i.test(text)) || '';
    const describe = (canvas) => {
      const rect = canvas.getBoundingClientRect();
      return { width: canvas.width, height: canvas.height,
        cssWidth: rect.width, cssHeight: rect.height, visible: visible(canvas) };
    };
    const gpu = [...document.querySelectorAll('canvas.gpuGameCanvas')].map(describe);
    return {
      frames: window.__i76?.frames || 0,
      blitErr: String(window.__i76?.blitErr || ''),
      footer: document.getElementById('buildId')?.textContent.trim() || '',
      footerVisible: visible(document.getElementById('buildId')),
      devHidden: document.getElementById('devPanel')?.hidden === true,
      software: describe(document.getElementById('screen')),
      gpu,
      visibleGpu: gpu.filter((canvas) => canvas.visible).length,
      fallback,
      navigatorGpu: !!navigator.gpu,
    };
  });
}

async function canvasStats(page, selector) {
  return page.evaluate(async (canvasSelector) => {
    // WebGPU's current texture expires at presentation. Sample within rAF,
    // after the page's already-queued frame callback, not an arbitrary timer.
    await new Promise(requestAnimationFrame);
    const canvas = document.querySelector(canvasSelector);
    if (!canvas) return null;
    const rect = canvas.getBoundingClientRect();
    const probe = document.createElement('canvas');
    probe.width = 160;
    probe.height = 120;
    const context = probe.getContext('2d', { willReadFrequently: true });
    let pixels;
    try {
      context.drawImage(canvas, 0, 0, probe.width, probe.height);
      pixels = context.getImageData(0, 0, probe.width, probe.height).data;
    } catch (error) {
      return { width: canvas.width, height: canvas.height, cssWidth: rect.width,
        cssHeight: rect.height, error: error.message };
    }
    const colors = new Set();
    let opaque = 0, min = Infinity, max = -Infinity;
    for (let i = 0; i < pixels.length; i += 4) {
      if (pixels[i + 3]) opaque++;
      const lum = pixels[i] + pixels[i + 1] + pixels[i + 2];
      min = Math.min(min, lum);
      max = Math.max(max, lum);
      if (colors.size < 512)
        colors.add((pixels[i] << 16) | (pixels[i + 1] << 8) | pixels[i + 2]);
    }
    return { width: canvas.width, height: canvas.height, cssWidth: rect.width,
      cssHeight: rect.height, opaque, uniqueColors: colors.size,
      channelSumRange: max - min };
  }, selector);
}

async function waitForRenderedCanvas(page, selector, timeout = 60000) {
  const deadline = Date.now() + timeout;
  let last = null;
  while (Date.now() < deadline) {
    if (selector === 'canvas.gpuGameCanvas') {
      const state = await pageState(page);
      insist(!state.blitErr, `GPU presentation failed: ${state.blitErr}`);
    }
    last = await canvasStats(page, selector);
    if (last && !last.error && last.width >= 320 && last.height >= 240 &&
        last.cssWidth >= 300 && last.cssHeight >= 220 && last.opaque >= 10000 &&
        last.uniqueColors >= 4 && last.channelSumRange >= 12)
      return last;
    await sleep(250);
  }
  throw new Error(`canvas did not produce a nontrivial frame: ${JSON.stringify(last)}`);
}

async function adapterDiagnostic(page) {
  return page.evaluate(async () => {
    if (!navigator.gpu) return { api: false, adapter: false, device: false };
    let adapter;
    try {
      adapter = await navigator.gpu.requestAdapter();
    } catch (error) {
      return { api: true, adapter: false, device: false,
        error: `${error.name || 'Error'}: ${error.message || error}`.slice(0, 300) };
    }
    if (!adapter) return { api: true, adapter: false, device: false };
    try {
      const device = await adapter.requestDevice();
      device.destroy();
      return { api: true, adapter: true, device: true,
        info: { vendor: adapter.info?.vendor, architecture: adapter.info?.architecture,
          description: adapter.info?.description } };
    } catch (error) {
      return { api: true, adapter: true, device: false,
        error: `${error.name || 'Error'}: ${error.message || error}`.slice(0, 300) };
    }
  });
}

class SkipCase extends Error {}

async function runCase(browser, baseUrl, spec) {
  const summary = {
    schema: 'i76-renderer-default-browser-v1', case: spec.tag,
    requested: spec.query || 'bare', mockNavigatorGpu: Boolean(spec.mockGpuMissing),
    mockSubmissionFailure: Boolean(spec.mockSubmissionFailure), savedRenderer: spec.saved || null,
    status: 'FAIL', browser: await browser.version(), mode: BROWSER_MODE, flags: FLAGS,
  };
  const consoleLines = [];
  const pageErrors = [];
  let context, page;
  try {
    context = await browser.createBrowserContext();
    page = await context.newPage();
    page.on('console', (message) => consoleLines.push(`${message.type()}: ${message.text()}`));
    page.on('pageerror', (error) => pageErrors.push(String(error.stack || error.message || error)));
    if (spec.mockGpuMissing) {
      await page.evaluateOnNewDocument(() => {
        Object.defineProperty(window.navigator, 'gpu', {
          configurable: true, value: undefined,
        });
      });
    }
    if (spec.mockSubmissionFailure) {
      await page.evaluateOnNewDocument(() => {
        if (globalThis.GPUQueue)
          GPUQueue.prototype.onSubmittedWorkDone = () =>
            Promise.reject(new Error('synthetic submission failure'));
      });
    }
    if (spec.saved)
      await page.evaluateOnNewDocument((saved) => localStorage.setItem('i76.renderer', saved), spec.saved);
    await page.setViewport({ width: 1100, height: 900, deviceScaleFactor: 2 });
    const url = new URL(baseUrl);
    if (spec.query) url.search = spec.query;
    await page.goto(url.href, { waitUntil: 'load', timeout: 120000 });
    summary.page = { path: url.pathname, search: url.search };
    // Retain acquired-device evidence even if a later initialization failure
    // makes a second adapter request unavailable. Such a run must not SKIP.
    if (spec.renderer === 'gpu' || spec.mockSubmissionFailure) {
      summary.adapterBefore = await adapterDiagnostic(page);
      if (spec.mockSubmissionFailure && !summary.adapterBefore.adapter)
        throw new SkipCase('no WebGPU adapter available for submission-failure control');
      if (summary.adapterBefore.adapter && !summary.adapterBefore.device)
        throw new Error(`adapter acquired but requestDevice failed: ${summary.adapterBefore.error}`);
    }
    Object.assign(summary, await stageAndEnterStockMission(page));
    insist((await pageState(page)).devHidden, 'normal page unexpectedly exposed developer controls');

    if (spec.renderer === 'gpu') {
      let state;
      const deadline = Date.now() + 120000;
      do {
        state = await pageState(page);
        if (state.visibleGpu || state.blitErr || state.fallback) break;
        await sleep(250);
      } while (Date.now() < deadline);
      if (!state.visibleGpu) {
        summary.rendererState = state;
        summary.adapterDiagnostic = await adapterDiagnostic(page);
        if (!pageErrors.length && !summary.adapterBefore.adapter && !summary.adapterDiagnostic.adapter)
          throw new SkipCase('no WebGPU adapter available for the bare-page default case');
        if (summary.adapterDiagnostic.adapter && !summary.adapterDiagnostic.device)
          throw new Error(`WebGPU adapter acquired but requestDevice failed: ${summary.adapterDiagnostic.error || 'unknown error'}`);
        throw new Error(`WebGPU device was acquirable but the page failed to activate it: ${state.blitErr || state.fallback || 'no renderer outcome'}`);
      }
      summary.canvas = await waitForRenderedCanvas(page, 'canvas.gpuGameCanvas');
      state = await pageState(page);
      summary.rendererState = state;
      insist(state.visibleGpu === 1, `expected one visible GPU canvas, got ${state.visibleGpu}`);
      insist(!state.software.visible, 'software canvas remained visible under active WebGPU');
      insist(summary.canvas.width > 640 && summary.canvas.height > 480,
             `GPU default lost native-resolution target: ${summary.canvas.width}x${summary.canvas.height}`);
      insist(Math.abs(summary.canvas.width / summary.canvas.height - 4 / 3) < 0.01,
             'GPU target is not 4:3');
      insist(state.footerVisible && / · gpu$/.test(state.footer),
             `footer does not report active gpu renderer: ${state.footer}`);
      insist(!state.blitErr, `WebGPU renderer reported an error: ${state.blitErr}`);
    } else {
      if (spec.mockGpuMissing)
        insist(!(await pageState(page)).navigatorGpu, 'navigator.gpu missing mock did not apply');
      summary.canvas = await waitForRenderedCanvas(page, '#screen');
      await sleep(500);
      const state = await pageState(page);
      summary.rendererState = state;
      insist(state.software.visible, 'software canvas is not visible');
      insist(state.visibleGpu === 0, 'a GPU canvas is visible in a software case');
      insist(state.footerVisible && / · sw$/.test(state.footer),
             `footer does not report active sw renderer: ${state.footer}`);
      if (spec.mockGpuMissing || spec.mockSubmissionFailure) {
        insist(Boolean(state.fallback), 'failed WebGPU did not retain a software-fallback diagnostic');
        if (spec.mockSubmissionFailure)
          insist(/synthetic submission failure/.test(state.blitErr), 'submission-failure control was not exercised');
      } else
        insist(!state.fallback, `explicit software selection showed a fallback error: ${state.fallback}`);
    }
    await sleep(250);
    insist(pageErrors.length === 0, `page errors: ${pageErrors.join(' | ')}`);
    summary.status = 'PASS';
  } catch (error) {
    summary.status = error instanceof SkipCase ? 'SKIP' : 'FAIL';
    summary.reason = error.message || String(error);
    if (page) summary.rendererState = await pageState(page).catch(() => null);
  } finally {
    summary.pageErrors = pageErrors;
    summary.consoleLines = consoleLines.length;
    if (page) {
      try {
        await page.screenshot({ path: path.join(OUT, `${spec.tag}.png`), fullPage: true });
        summary.screenshot = `${spec.tag}.png`;
      } catch (error) {
        summary.status = 'FAIL';
        summary.reason = `screenshot failed: ${error.message}`;
      }
    }
    writeFileSync(path.join(OUT, `${spec.tag}-console.log`), consoleLines.join('\n') + '\n');
    writeFileSync(path.join(OUT, `${spec.tag}.json`), JSON.stringify(summary, null, 2) + '\n');
    if (context) await context.close().catch(() => {});
  }
  log(`[${summary.status}] ${spec.tag}${summary.reason ? ` — ${summary.reason}` : ''}`);
  return summary;
}

const CASES = [
  { tag: 'bare-default-gpu', renderer: 'gpu' },
  { tag: 'query-software', renderer: 'sw', query: '?renderer=sw' },
  { tag: 'missing-webgpu-fallback', renderer: 'sw', mockGpuMissing: true },
  { tag: 'saved-software', renderer: 'sw', saved: 'sw' },
  { tag: 'submission-error-fallback', renderer: 'sw', mockSubmissionFailure: true },
];

let server = null;
let browser = null;
const results = [];
let fatal = null;
try {
  const served = await startServer();
  server = served.child;
  browser = await puppeteer.launch({
    headless: BROWSER_MODE === 'headless', protocolTimeout: 900000,
    ...(CHROME ? { executablePath: CHROME } : {}),
    // Puppeteer normalizes feature switches in-place; keep evidence immutable.
    args: [...FLAGS],
  });
  log(`[browser] ${await browser.version()} mode=${BROWSER_MODE}`);
  const diagnostics = await browser.target().createCDPSession();
  writeFileSync(path.join(OUT, 'browser-system.json'),
    JSON.stringify(await diagnostics.send('SystemInfo.getInfo'), null, 2) + '\n');
  await diagnostics.detach();
  for (const spec of CASES)
    results.push(await runCase(browser, served.page, spec));
} catch (error) {
  fatal = String(error.stack || error);
  log(`[FATAL] ${fatal}`);
} finally {
  if (browser) await browser.close().catch((error) => log(`[close] browser: ${error.message}`));
  if (server) await stopServer(server).catch((error) => log(`[close] server: ${error.message}`));
}

const overall = {
  schema: 'i76-renderer-default-browser-v1',
  status: fatal || results.some((result) => result.status === 'FAIL') ? 'FAIL'
    : results.some((result) => result.status === 'SKIP') ? 'SKIP' : 'PASS',
  fatal, cases: results.map(({ case: tag, status, reason = null }) => ({ tag, status, reason })),
};
writeFileSync(path.join(OUT, 'summary.json'), JSON.stringify(overall, null, 2) + '\n');
if (overall.status === 'FAIL') process.exitCode = 1;
else if (overall.status === 'SKIP') process.exitCode = 3;
else log('RESULT: PASS renderer default/query/fallback browser smoke');
