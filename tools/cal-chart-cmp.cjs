// tools/cal-chart-cmp.cjs —— 两跑标定对比：终端数字 + 四合一对比 SVG
//   用法: node tools/cal-chart-cmp.cjs <run1.csv> <run2.csv> [out.svg]
//   取数据: curl.exe -s "http://192.168.31.99/download?f=/battcal.csv" -o runN.csv
//   转 PNG: msedge --headless --disable-gpu --screenshot=out.png --window-size=1140,1180 file:///.../out.svg
//   ⚠️ 内置 T1/T2 是两个 21 点表（第一跑 / 第二跑）；换对比对象时从 GET /battcal/report 的 pct 数组更新

// 两跑标定对比：分析数字 + 四合一对比 SVG
//   用法: node cmp_runs.cjs <run1.csv> <run2.csv> [out.svg]
const fs = require('fs');
function load(f) {
  const rows = [];
  for (const l of fs.readFileSync(f, 'utf8').split(/\r?\n/)) {
    if (!l.trim() || l[0] === '#' || l.startsWith('t_loaded')) continue;
    const c = l.split(',').map(Number);
    if (c.length < 9 || isNaN(c[0])) continue;
    rows.push({ t: c[0], dt: c[1], vl: c[2], vr: c[3], il: Math.abs(c[4]), ir: Math.abs(c[5]), tc: c[6] });
  }
  return rows;
}
const REST = 60000;
const T1 = [2.945, 3.635, 3.710, 3.748, 3.792, 3.822, 3.842, 3.861, 3.874, 3.893, 3.919, 3.948, 3.975, 4.001, 4.015, 4.022, 4.029, 4.038, 4.054, 4.074, 4.148];
const T2 = [2.969, 3.622, 3.701, 3.735, 3.780, 3.810, 3.834, 3.851, 3.865, 3.880, 3.903, 3.931, 3.958, 3.986, 4.005, 4.014, 4.020, 4.029, 4.043, 4.064, 4.134];
const R1 = load(process.argv[2]), R2 = load(process.argv[3]);
const avg = a => a.reduce((x, y) => x + y, 0) / a.length;
const med = a => [...a].sort((x, y) => x - y)[Math.floor(a.length / 2)];
function stat(r) {
  let mAh = 0;
  for (const x of r) mAh += (x.il * x.dt + x.ir * REST) / 3600000;
  return {
    n: r.length, tEnd: r[r.length - 1].t,
    loadH: r[r.length - 1].t / 3600000,
    low: r.filter(x => x.dt === REST).length,
    mAh, ilMed: med(r.map(x => x.il)), ilAvg: avg(r.map(x => x.il)),
    irMed: med(r.map(x => x.ir)), irAvg: avg(r.map(x => x.ir)),
    v0: r[r.length - 1].vr, tc: r[r.length - 1].tc,
  };
}
const s1 = stat(R1), s2 = stat(R2);
const pctOf = (r, i) => r[i].t / r[r.length - 1].t * 100;

console.log('=== 两跑对比 ===');
console.log('指标              第一跑(09-15)   第二跑(09-16)   差异');
const row = (k, a, b, u = '', d = 1) => console.log(`${k.padEnd(16)}  ${String(a.toFixed(d)).padStart(8)}${u}  ${String(b.toFixed(d)).padStart(8)}${u}  ${((b - a) / Math.abs(a) * 100).toFixed(1)}%`);
console.log(`样本数           ${String(s1.n).padStart(9)}     ${String(s2.n).padStart(9)}     ${s2.n - s1.n}`);
row('累计加载时长(h)', s1.loadH, s2.loadH, 'h', 2);
row('低压加密轮数', s1.low, s2.low, '', 0);
row('实测容量(mAh)', s1.mAh, s2.mAh, 'mAh', 0);
row('i_load 中位(mA)', s1.ilMed, s2.ilMed, 'mA', 1);
row('i_load 平均(mA)', s1.ilAvg, s2.ilAvg, 'mA', 1);
row('i_rest 中位(mA)', s1.irMed, s2.irMed, 'mA', 1);
row('i_rest 平均(mA)', s1.irAvg, s2.irAvg, 'mA', 1);
row('末点 V(mV)', s1.v0 * 1000, s2.v0 * 1000, 'mV', 0);
console.log(`温度             ${String(s1.tc).padStart(9)}      ${String(s2.tc).padStart(9)}      ${(s2.tc - s1.tc).toFixed(1)}C`);
let dmax = 0, dsum = 0;
const deltas = T2.map((v, k) => (v - T1[k]) * 1000);
deltas.slice(1).forEach(d => { dsum += d; if (Math.abs(d) > Math.abs(dmax)) dmax = d; });
console.log(`\n21 点表差异（5%~100%）：平均 ${(dsum / 20).toFixed(1)} mV，最大 ${dmax.toFixed(1)} mV`);
console.log('逐点(mV)：' + deltas.map((d, k) => `${k * 5}%:${d > 0 ? '+' : ''}${d.toFixed(0)}`).join('  '));

// ---------------- SVG ----------------
const W = 1140, H = 1180, F = 'Segoe UI,Microsoft YaHei,sans-serif';
const o = [];
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');
const txt = (x, y, s, sz = 13, an = 'start', fl = '#222', w = 'normal') =>
  o.push(`<text x="${x.toFixed(1)}" y="${y.toFixed(1)}" font-family="${F}" font-size="${sz}" text-anchor="${an}" fill="${fl}" font-weight="${w}">${esc(s)}</text>`);
const ln = (x1, y1, x2, y2, st, w = 1, da = '') =>
  o.push(`<line x1="${x1.toFixed(1)}" y1="${y1.toFixed(1)}" x2="${x2.toFixed(1)}" y2="${y2.toFixed(1)}" stroke="${st}" stroke-width="${w}"${da ? ` stroke-dasharray="${da}"` : ''}/>`);
const rc = (x, y, w, h, f) => o.push(`<rect x="${x.toFixed(1)}" y="${y.toFixed(1)}" width="${w.toFixed(1)}" height="${h.toFixed(1)}" fill="${f}"/>`);
const dot = (x, y, r, f) => o.push(`<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="${r}" fill="${f}"/>`);
o.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">`);
rc(0, 0, W, H, '#fff');
txt(24, 34, '标定两跑对比（第一跑 09-15 / 第二跑 09-16）', 19, 'start', '#111', 'bold');
txt(24, 55, `第一跑：${s1.n} 样本 ${s1.loadH.toFixed(2)}h 加载 / ${s1.mAh.toFixed(0)}mAh / i_load ${s1.ilMed.toFixed(0)}mA · i_rest ${s1.irMed.toFixed(0)}mA      第二跑：${s2.n} 样本 ${s2.loadH.toFixed(2)}h / ${s2.mAh.toFixed(0)}mAh / i_load ${s2.ilMed.toFixed(0)}mA · i_rest ${s2.irMed.toFixed(0)}mA`, 12, 'start', '#555');

// ① 21 点表 V vs %
const A = { x0: 80, x1: 1090, y0: 85, y1: 400, vmin: 2.90, vmax: 4.20 };
const AX = p => A.x0 + p / 100 * (A.x1 - A.x0);
const AY = v => A.y1 - (v - A.vmin) / (A.vmax - A.vmin) * (A.y1 - A.y0);
rc(A.x0, A.y0, A.x1 - A.x0, A.y1 - A.y0, '#fbfcfe');
txt(A.x0 + 8, A.y0 + 19, '① 两跑 21 点表叠合（电压 ↔ 电量%）—— 差异全程 8~17mV，即高负载那几十分 mV 的 IR 压降', 13.5, 'start', '#1a3a5c', 'bold');
for (const v of [3.0, 3.2, 3.4, 3.6, 3.8, 4.0, 4.2]) { ln(A.x0, AY(v), A.x1, AY(v), '#e6ebf0'); txt(A.x0 - 8, AY(v) + 4, v.toFixed(1), 11, 'end', '#888'); }
for (let p = 0; p <= 100; p += 10) { ln(AX(p), A.y0, AX(p), A.y1, '#eef2f6'); txt(AX(p), A.y1 + 16, p + '%', 11, 'middle', '#888'); }
o.push(`<polyline fill="none" stroke="#1a6db5" stroke-width="2.5" points="${T1.map((v, k) => `${AX(k * 5)},${AY(v)}`).join(' ')}"/>`);
o.push(`<polyline fill="none" stroke="#e07b1a" stroke-width="2.5" stroke-dasharray="6 3" points="${T2.map((v, k) => `${AX(k * 5)},${AY(v)}`).join(' ')}"/>`);
T1.forEach((v, k) => dot(AX(k * 5), AY(v), 3, '#1a6db5'));
T2.forEach((v, k) => dot(AX(k * 5), AY(v), 2.2, '#e07b1a'));
txt(AX(62), AY(4.16), '第一跑', 12, 'start', '#1a6db5', 'bold');
txt(AX(62), AY(4.10), '第二跑（虚）', 12, 'start', '#e07b1a', 'bold');
ln(A.x0, A.y1, A.x1, A.y1, '#9aa5b1');

// ② 原始 V_rest vs 累计加载时长
const B = { x0: 80, x1: 1090, y0: 445, y1: 700, vmin: 2.90, vmax: 4.20, tmax: Math.max(s1.loadH, s2.loadH) };
const BX = h => B.x0 + h / B.tmax * (B.x1 - B.x0);
const BY = v => B.y1 - (v - B.vmin) / (B.vmax - B.vmin) * (B.y1 - B.y0);
rc(B.x0, B.y0, B.x1 - B.x0, B.y1 - B.y0, '#fbfcfe');
txt(B.x0 + 8, B.y0 + 19, '② 原始曲线：负载更大 → 同样电量走得更快（第二跑 11.02h vs 第一跑 13.57h）', 13.5, 'start', '#1a3a5c', 'bold');
for (const v of [3.0, 3.2, 3.4, 3.6, 3.8, 4.0, 4.2]) { ln(B.x0, BY(v), B.x1, BY(v), '#e6ebf0'); txt(B.x0 - 8, BY(v) + 4, v.toFixed(1), 11, 'end', '#888'); }
for (let h = 0; h <= 14; h += 2) { const x = BX(h); if (x > B.x1) break; ln(x, B.y0, x, B.y1, '#eef2f6'); txt(x, B.y1 + 16, h + 'h', 11, 'middle', '#888'); }
o.push(`<polyline fill="none" stroke="#1a6db5" stroke-width="2" points="${R1.map(r => `${BX(r.t / 3600000)},${BY(r.vr)}`).join(' ')}"/>`);
o.push(`<polyline fill="none" stroke="#e07b1a" stroke-width="2" points="${R2.map(r => `${BX(r.t / 3600000)},${BY(r.vr)}`).join(' ')}"/>`);
ln(B.x0, BY(3.50), B.x1, BY(3.50), '#d9534f', 1, '4 3');
txt(B.x0 + 10, BY(3.50) - 6, '3.50V 深睡', 10, 'start', '#d9534f');
ln(B.x0, B.y1, B.x1, B.y1, '#9aa5b1');

// ③ 尾部放大：0~12%
const C = { x0: 80, x1: 1090, y0: 745, y1: 940, vmin: 2.90, vmax: 3.80, pmax: 12 };
const CX = p => C.x0 + p / C.pmax * (C.x1 - C.x0);
const CY = v => C.y1 - (v - C.vmin) / (C.vmax - C.vmin) * (C.y1 - C.y0);
rc(C.x0, C.y0, C.x1 - C.x0, C.y1 - C.y0, '#fbfcfe');
txt(C.x0 + 8, C.y0 + 19, '③ 尾部放大（电量 0~12%，即跑完前那段）：两跑都撞在 2.95~2.97V（实机底线），不是表算出来的', 13.5, 'start', '#1a3a5c', 'bold');
for (const v of [2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8]) { ln(C.x0, CY(v), C.x1, CY(v), '#e6ebf0'); txt(C.x0 - 8, CY(v) + 4, v.toFixed(1), 11, 'end', '#888'); }
for (let p = 0; p <= 12; p += 2) { ln(CX(p), C.y0, CX(p), C.y1, '#eef2f6'); txt(CX(p), C.y1 + 16, p + '%', 11, 'middle', '#888'); }
for (const [R, st] of [[R1, '#1a6db5'], [R2, '#e07b1a']]) {
  const te = R[R.length - 1].t;
  const win = R.filter(r => (100 - r.t / te * 100) <= 12)
    .map(r => ({ p: 100 - r.t / te * 100, v: r.vr })).sort((a, b) => a.p - b.p);
  o.push(`<polyline fill="none" stroke="${st}" stroke-width="2" points="${win.map(q => `${CX(q.p).toFixed(1)},${CY(q.v).toFixed(1)}`).join(' ')}"/>`);
}
for (const [T, st] of [[T1, '#1a6db5'], [T2, '#e07b1a']]) for (let k = 0; k <= 2; k++) dot(CX(k * 5), CY(T[k]), 4.5, st);
txt(CX(6.5), CY(3.62), '5% 点：3.635 / 3.622V', 11, 'start', '#444');
txt(C.x1 - 8, CY(2.95) - 6, '0% 点：2.945 / 2.969V ← 两次都落在 2.95V 附近', 11, 'end', '#d9534f');
ln(C.x0, C.y1, C.x1, C.y1, '#9aa5b1');

// ④ 电流对比条
const D = { x0: 80, x1: 1090, y0: 985, y1: 1150 };
rc(D.x0, D.y0, D.x1 - D.x0, D.y1 - D.y0, '#fbfcfe');
txt(D.x0 + 8, D.y0 + 19, '④ 电流对比：负载 +15%（射频环境），静置 +53%（+25mA 恒定额外负载，待查）', 13.5, 'start', '#1a3a5c', 'bold');
const bars = [
  ['i_load', s1.ilMed, s2.ilMed], ['i_load 平均', s1.ilAvg, s2.ilAvg],
  ['i_rest', s1.irMed, s2.irMed], ['i_rest 平均', s1.irAvg, s2.irAvg],
];
const scale = 900 / 100;
bars.forEach(([k, a, b], i) => {
  const y = D.y0 + 40 + i * 26;
  txt(D.x0 + 8, y + 12, k, 12, 'start', '#444');
  rc(D.x0 + 110, y, a * scale, 16, '#1a6db5');
  rc(D.x0 + 110, y + 17, b * scale, 16, '#e07b1a');
  txt(D.x0 + 115 + a * scale + 6, y + 12, a.toFixed(1) + ' mA', 11, 'start', '#1a6db5');
  txt(D.x0 + 115 + b * scale + 6, y + 29, b.toFixed(1) + ' mA', 11, 'start', '#e07b1a');
  txt(D.x1 - 8, y + 12, `Δ ${(b - a).toFixed(1)} mA`, 11, 'end', '#d9534f');
});
txt(D.x0 + 110, D.y1 - 6, '0', 10, 'middle', '#888');
txt(D.x0 + 110 + 20 * scale, D.y1 - 6, '20mA', 10, 'middle', '#888');
txt(D.x0 + 110 + 100 * scale, D.y1 - 6, '100mA  →', 10, 'middle', '#888');
o.push('</svg>');
const out = process.argv[4] || 'cmp.svg';
fs.writeFileSync(out, o.join('\n'));
console.log('\n写入 ' + out);
