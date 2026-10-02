// ============================================================================
//  toast-verify.mjs —— 提示横幅（Toast）全套自检
//    node tools/toast-verify.mjs [ip] [--dur 7000]
//  ① 三种版面（√/×/热点）与离线渲染逐像素比对
//  ② 失败 → 排队 → 10 秒后自动接上热点页（模拟开机连不上的真实时序）
//  ③ 到期还原（中文标签 1261 点 + 数值区）
//  ④ 超长 SSID 截断逻辑（离线复算 buildSsidLine）
//  ★ 断言依据是 tools/lib.mjs 里的版式/点阵，改了固件必须同步改 lib.mjs
// ============================================================================
import { device, loadFont, canvas, drawToast, diff, arg, pass, sleep, LOG_W, LOG_H, ROOT } from './lib.mjs';
import fs from 'fs';

const IP = process.argv[2] || '192.168.31.99';
const DUR = +arg('dur', 7000);
const d = device(IP);
const font = loadFont();
const EXP = (kind, l1, l2, l3) => drawToast(canvas(), font, kind, l1, l2, l3);

// 热点名/IP 直接从固件里读（AP_SSID / AP_IP_STR），免得两边各写一份
const ino = fs.readFileSync(ROOT + 'PHT_2_0.ino', 'utf8');
const AP_NAME = (ino.match(/AP_SSID\s*=\s*"([^"]+)"/) || [, 'Weather_Station_Pro'])[1];
const AP_IP = (ino.match(/AP_IP_STR\s*=\s*"([^"]+)"/) || [, '192.168.5.1'])[1];

let ok = 0, n = 0;
const T = async (tag, url, exp, waitMs, dumpAfterMs) => {
  await d.get(url); await sleep(dumpAfterMs);
  const dd = diff(await d.dump(), exp);
  n++; ok += pass(tag, dd.n === 0, `差异 ${dd.n} px${dd.n ? ' → ' + dd.samples.join(' | ') : ''}`);
  await sleep(waitMs);
};

const probe = (await d.get(`/toast?k=ok&d=5000`)).trim();
const mm = probe.match(/SSID "(.+?)" \/ ([0-9.]+)/);
const ssid = mm ? mm[1] : 'unknown', ip = mm ? mm[2] : '0.0.0.0';
console.log(`设备 ${IP}  SSID="${ssid}"  IP=${ip}（横幅停留 ${DUR}ms）\n`);

console.log('【① 三种自动横幅】');
await T('√ WiFi OK ', `/toast?k=ok&d=${DUR}`, EXP('ok', 'WiFi OK', `SSID "${ssid}"`, ip), DUR + 1500, 1500);
await T('× WiFi FAIL', `/toast?k=fail&d=${DUR}`, EXP('fail', 'WiFi FAIL', `SSID "${ssid}"`, 'not connected'), DUR + 1500, 1500);
await T('热点 AP MODE', `/toast?k=ap&d=${DUR}`, EXP('ap', 'AP MODE', `SSID "${ssid}"`, ip), DUR + 1500, 1500);

console.log('\n【② 失败 → 排队 → 自动接上热点页】');
await d.get(`/toast?k=fail&d=${DUR}&q=1`); await sleep(1500);
let d1 = diff(await d.dump(), EXP('fail', 'WiFi FAIL', `SSID "${ssid}"`, 'not connected'));
n++; ok += pass('第 1 条 = 失败页（不给 IP）', d1.n === 0, `差异 ${d1.n} px`);
await sleep(DUR + 500);
let d2 = diff(await d.dump(), EXP('ap', 'AP MODE', `SSID "${AP_NAME}"`, AP_IP));
n++; ok += pass('第 2 条 = 自动接上的热点页（带热点名 + IP）', d2.n === 0, `差异 ${d2.n} px`);
await sleep(DUR + 1500);

console.log('\n【③ 到期还原】');
const c3 = await d.dump();
n++; ok += pass('中文标签还原 = 1261 点', c3.ink(6, 25, 54, 112) === 1261, `实测 ${c3.ink(6, 25, 54, 112)}`);
const vals = c3.ink(60, 26, 249, 110);
n++; ok += pass('数值区已重绘', vals > 500, `ink=${vals}`);

console.log('\n【④ 超长 SSID 截断（离线复算 buildSsidLine）】');
const long = 'A_very_long_hotspot_name_1234567890';
const wTop = s => font.width(s, true);
const sim = s0 => { let s = `SSID "${s0}"`; if (wTop(s) <= 240) return s; let len = s0.length; while (len > 3) { len--; s = `SSID "${s0.slice(0, len)}.."`; if (wTop(s) <= 240) return s; } return s; };
const out = sim(long);
n++; ok += pass(`35 字符 SSID 截断后 ${wTop(out)} px（屏宽 ${LOG_W}）`, wTop(out) <= 240, JSON.stringify(out));

console.log(`\n=== 通过 ${ok}/${n} ===   ${ok === n ? '✅ 全过' : '❌ 有失败'}`);
process.exit(ok === n ? 0 : 1);
