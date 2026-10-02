// ============================================================================
//  capture-screen.mjs —— 抓真机屏幕拼成 PNG（"实拍图"，直接读显存，不涉及相机）
//    node tools/capture-screen.mjs [ip] [--out shot.png] [--dur 7000] [--no-toast]
//  默认抓：平时 → √ → × → 热点 →（失败 10 秒后自动接上的热点页）
// ============================================================================
import { device, png, savePng, arg, hasFlag, sleep, canvas } from './lib.mjs';

const IP = process.argv[2] || '192.168.31.99';
const OUT = arg('out', 'screen-shot.png');
const DUR = +arg('dur', 7000);
const d = device(IP);

const shots = [];
shots.push(await d.dump());
if (!hasFlag('no-toast')) {
  for (const k of ['ok', 'fail', 'ap']) {
    await d.get(`/toast?k=${k}&d=${DUR}`); await sleep(1300);
    shots.push(await d.dump());
    await sleep(DUR + 500);
  }
  await d.get(`/toast?k=fail&d=${DUR}&q=1`);
  await sleep(DUR + 1500);
  shots.push(await d.dump());
}
savePng(OUT, png(shots, { scale: +arg('scale', 3) }));
console.log(`已抓 ${shots.length} 块：平时${hasFlag('no-toast') ? '' : ' / √ / × / 热点 / 排队后的热点'}  → ${OUT}`);
