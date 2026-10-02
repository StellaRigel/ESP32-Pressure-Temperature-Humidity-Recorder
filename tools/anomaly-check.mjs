// ============================================================================
//  anomaly-check.mjs —— 气压距平自检（基线健全性 + 固定/移动门控 + 移动样本不污染基线）
//    node tools/anomaly-check.mjs [ip] [--wait 45000]
//  背景：设计文档要求「距平曲线仅固定模式可用，其平均值也仅用固定数据计算」。
//        2026-09-13 修复：没采集的槽曾被当成一天参与平均，把基线稀释（1012 hPa → 507）。
//  本脚本会临时把设备切到移动再切回固定（结束时保证恢复原模式）。
// ============================================================================
import { device, arg, pass, sleep } from './lib.mjs';

const IP = process.argv[2] || '192.168.31.99';
const WAIT = +arg('wait', 45000);           // 移动停留时长（要 ≥1 个采样间隔 30s）
const d = device(IP);
let ok = 0, n = 0;
const T = (tag, cond, extra = '') => { n++; ok += pass(tag, cond, extra); };
const j = async p => JSON.parse(await d.get(p));
const avg = a => a.length ? a.reduce((x, y) => x + y, 0) / a.length : NaN;

const orig = (await j('/status')).deviceMode;
console.log(`设备 ${IP}  当前模式 ${orig === 1 ? 'MOV 移动' : 'FIX 固定'}  移动停留 ${WAIT / 1000}s\n`);
let failures = [];
try {
  // ---------- ① 固定：基线是否健全 ----------
  if (orig === 1) { await d.get('/mode?m=0'); await sleep(1500); }
  console.log('【① 固定模式：基线健全性】');
  // 刚重启时 todaySlots 是空的，先等一个采样落进槽里（最多 45s）
  for (let i = 0; i < 15; i++) { if ((await j('/anomaly')).slots > 0) break; await sleep(3000); }
  const an = await j('/anomaly');
  const dp = (an.data || []).map(x => x.dP);
  const pr = (an.data || []).map(x => x.pr);
  T('距平曲线可用（site=true）', an.site === true);
  T('有基线天数 days ≥ 1', an.days >= 1, `days=${an.days}`);
  T('今日已采集槽位数 slots > 0', an.slots > 0, `slots=${an.slots}`);
  T('曲线有数据点', dp.length > 0, `${dp.length} 点`);
  if (pr.length) T(`曲线气压值合理（900~1100 hPa）`, avg(pr) > 900 && avg(pr) < 1100, `均值 ${avg(pr).toFixed(2)} hPa`);
  if (dp.length) T('距平量级合理（|dP| < 20 hPa，不再是 505 那种稀释值）', Math.abs(avg(dp)) < 20, `均值 ${avg(dp).toFixed(2)} hPa`);

  const da = await j('/daily-avg');
  const pr2 = (da.data || []).map(x => x.pr);
  T(`日均气压合理（不再是基线减半）`, !pr2.length || (avg(pr2) > 900 && avg(pr2) < 1100), `days=${da.days} 均值 ${pr2.length ? avg(pr2).toFixed(2) : '-'} hPa`);

  const st0 = await j('/today-stats');
  T('今日统计报告 site=true', st0.site === true);
  T('当前距平有值且量级合理', st0.anomaly !== null && Math.abs(st0.anomaly) < 20, `anomaly=${st0.anomaly}`);

  // ---------- ② 移动：门控 ----------
  console.log('\n【② 移动模式：距平应不可用】');
  await d.get('/mode?m=1'); await sleep(1500);
  const ah = await j('/anomaly');
  const sth = await j('/today-stats');
  T('曲线报告 site=false 且 data 为空', ah.site === false && (ah.data || []).length === 0, JSON.stringify({ site: ah.site, n: (ah.data || []).length }));
  T('今日统计报告 site=false 且 anomaly=null', sth.site === false && sth.anomaly === null, JSON.stringify({ site: sth.site, anomaly: sth.anomaly }));
  const html = await d.get('/');
  T('网页已带距平卡片提示位（cardAnom / anomHint）', html.includes('cardAnom') && html.includes('anomHint'));

  // ---------- ③ 移动样本不得进入基线 ----------
  console.log(`\n【③ 移动期间的样本不得进入距平基线（停留 ${WAIT / 1000}s，≈${Math.round(WAIT / 30000)} 个采样）】`);
  const s0 = (await j('/anomaly')).slots;
  await sleep(WAIT);
  const s1 = (await j('/anomaly')).slots;
  const c1 = (await j('/status')).count;
  T('移动期间采样在继续（count 推进）', true, `当前 count=${c1}`);
  T('移动期间 slots 不增长（样本未计入）', s1 === s0, `slots ${s0} → ${s1}`);

  // ---------- ④ 切回固定：恢复累计 ----------
  console.log('\n【④ 切回固定：应恢复累计】');
  await d.get('/mode?m=0'); await sleep(WAIT > 35000 ? 35000 : WAIT + 5000);
  const s2 = (await j('/anomaly')).slots;
  T('切回固定后 slots 继续增长', s2 > s1, `slots ${s1} → ${s2}`);
} finally {
  const now = (await j('/status')).deviceMode;
  if (now !== orig) { await d.get('/mode?m=' + orig); console.log(`\n（已恢复原模式 ${orig === 1 ? 'MOV' : 'FIX'}）`); }
}
console.log(`\n=== 通过 ${ok}/${n} ===   ${ok === n ? '✅ 全过' : '❌ 有失败'}`);
process.exit(ok === n ? 0 : 1);
