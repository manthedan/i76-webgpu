#!/usr/bin/env node
/* Headless-browser driver for H-UAT-052 traversal + H-UAT-053 far-field pairs. */
import puppeteer from 'puppeteer';
import {readFile,realpath,stat,writeFile} from 'node:fs/promises';
import path from 'node:path';
import {externalPath,loopbackPage,newOutputDir} from './tests/external_paths.mjs';
const PAGE=process.argv[2],OUT_ARG=process.argv[3],APP_INPUT=process.env.NITRO_APP;
if(!PAGE||!OUT_ARG||!APP_INPUT){console.error('usage: NITRO_APP=/owned/app node road_traversal_gate.mjs <page-url> <new-output-dir>');process.exit(1)}
loopbackPage(PAGE);
// Write only beneath the canonical directory validation reserved.
const OUT=newOutputDir(OUT_ARG);
const CASES=['miss8/P01.MSN','miss8/P02.MSN'];
const ASSET_PREFIX='/__i76_assets/';
const FLAGS=['--no-sandbox','--disable-dev-shm-usage','--enable-unsafe-webgpu','--enable-webgpu-developer-features','--use-angle=vulkan','--disable-background-timer-throttling','--disable-renderer-backgrounding','--disable-backgrounding-occluded-windows'];
let app;
try{app=externalPath(APP_INPUT,'NITRO_APP',{existing:true});if(!(await stat(path.join(app,'nitro.zfs'))).isFile())throw new Error('nitro.zfs is not a file')}catch(e){console.error(`FAIL asset prerequisite ${APP_INPUT}: ${e.message}`);process.exit(1)}
const prefix=app.endsWith(path.sep)?app:app+path.sep;
const safeAsset=urlText=>{const url=new URL(urlText),pn=url.pathname;if(url.origin!==new URL(PAGE).origin||!pn.startsWith(ASSET_PREFIX))return null;let rel;try{rel=decodeURIComponent(pn.slice(ASSET_PREFIX.length))}catch{return false}if(!rel||rel.includes('\0')||rel.includes('\\'))return false;const full=path.resolve(app,rel);return full.startsWith(prefix)?full:false};
const route=async page=>{await page.setRequestInterception(true);page.on('request',async req=>{const full=safeAsset(req.url());if(full===null){await req.continue();return}if(full===false){await req.respond({status:400});return}try{const actual=await realpath(full);if(!actual.startsWith(prefix)){await req.respond({status:403});return}const body=await readFile(actual);await req.respond({status:200,headers:{'Access-Control-Allow-Origin':'*','Content-Type':'application/octet-stream','Content-Length':String(body.length)},body:req.method()==='HEAD'?undefined:body})}catch(e){await req.respond({status:e.code==='ENOENT'?404:500})}})};
let browser,failed=false,skipped=null;
try{
 browser=await puppeteer.launch({headless:'new',protocolTimeout:900000,...(process.env.CHROME?{executablePath:process.env.CHROME}:{}),args:FLAGS});
 console.log(`road-traversal: browser=${await browser.version()}`);console.log(`road-traversal: artifacts=${OUT}`);
 for(const mission of CASES){
  const tag=mission.match(/P\d+/i)[0].toLowerCase(),page=await browser.newPage(),logs=[];
  page.on('console',m=>logs.push(`${m.type()}: ${m.text()}`));page.on('pageerror',e=>logs.push(`pageerror: ${e.message}`));
  await route(page);await page.setViewport({width:1050,height:900,deviceScaleFactor:1});
  const u=new URL(PAGE);u.searchParams.set('app',u.origin+ASSET_PREFIX);u.searchParams.set('mission',mission);
  let probe;
  try{await page.goto(u.href,{waitUntil:'domcontentloaded',timeout:120000});await page.waitForFunction(()=>/road traversal — (PASS|FAIL|SKIP)/.test(document.title),{timeout:600000});probe=await page.evaluate(()=>window.__roadTraversal||null);if(!probe)throw new Error('missing structured result')}catch(e){probe={status:'FAIL',reason:`browser harness: ${e.message}`}}
  const shot=path.join(OUT,`${tag}-traversal.png`);await page.screenshot({path:shot,fullPage:true});
  await writeFile(path.join(OUT,`${tag}.json`),JSON.stringify(probe,null,2)+'\n');await writeFile(path.join(OUT,`${tag}-console.log`),logs.join('\n')+'\n');
  if(probe.status==='SKIP'){skipped=`${tag}: ${probe.reason}`;console.log(`SKIPPED — ${skipped}`);await page.close();break}
  const valid=probe.status==='PASS'&&probe.match===true&&probe.lanePrepared===true&&probe.stable===true&&probe.gpuLive===true&&probe.gpuStable===true&&probe.beforeCoverage===true&&probe.farField?.pass===true&&probe.farField?.pairs===20&&probe.samples?.length===6;
  if(!valid){failed=true;console.error(`FAIL ${tag} ${probe.reason||JSON.stringify({lanePrepared:probe.lanePrepared,lanePrep:probe.lanePrep,stable:probe.stable,gpuLive:probe.gpuLive,gpuStable:probe.gpuStable,beforeCoverage:probe.beforeCoverage,moved:probe.moved,farField:probe.farField})}`);console.error(`FAIL evidence: ${shot}`)}else{
   const worst=Math.max(...probe.samples.map(s=>s.road.occluded/s.road.written));const gpuMin=Math.min(...probe.samples.map(s=>s.gpuRoadPixels));const gpuWorst=Math.max(...probe.samples.map(s=>s.gpuRoadBandOutlierFraction));
   const ff=probe.farField;
   console.log(`ok   ${tag} moved=${probe.moved.toFixed(2)}m samples=6 pre-traversal=${probe.samples.slice(0,-1).map(s=>s.distanceBeforeEnd.toFixed(1)).join(',')}m`+(probe.lanePrep?.required?` lane-prep=${probe.lanePrep.ticks} ticks contacts=${probe.lanePrep.playerContactsStart}->${probe.lanePrep.playerContactsEnd} x=${probe.lanePrep.x.toFixed(2)} yaw=${probe.lanePrep.yaw.toFixed(4)}`:''));
   console.log(`ok   ${tag} far-field pairs=${ff.pairs} moved=${ff.moved.toFixed(2)}m control_zero=${ff.controlZero} accepted_every_pair=${ff.acceptedEveryPair} mid_sw=${ff.sw.mid.flips}/${ff.sw.mid.footprint}=${ff.sw.mid.flipPct.toFixed(3)}% accepted=${ff.sw.mid.acceptedBoth} mid_gpu=${ff.gpu.mid.flips}/${ff.gpu.mid.footprint}=${ff.gpu.mid.flipPct.toFixed(3)}% accepted=${ff.gpu.mid.acceptedBoth}`);
   console.log(`info ${tag} far-bands sw=${['near','mid','far'].map(k=>`${k}:${ff.sw[k].flipPct.toFixed(3)}%`).join(',')} gpu=${['near','mid','far'].map(k=>`${k}:${ff.gpu[k].flipPct.toFixed(3)}%`).join(',')}`);
   console.log(`info ${tag} worst_sw_reject=${(worst*100).toFixed(3)}% gpu_road_outlier=${(gpuWorst*100).toFixed(3)}% min_gpu_road_px=${gpuMin} evidence=${shot}`);
  }
  await page.close();
 }
}catch(e){failed=true;console.error(`FAIL road traversal launch: ${e.stack||e}`)}finally{if(browser)await browser.close().catch(()=>{})}
if(failed)process.exit(1);if(skipped)process.exit(2);console.log(`RESULT: PASS (${CASES.length} physical-drive cases; anti-heal traversal + 20 consecutive far-field pairs, SW/GPU)`);
