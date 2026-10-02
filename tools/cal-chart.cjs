// tools/cal-chart.cjs —— 由标定原始 CSV 生成三合一 SVG 曲线图（放电曲线 / 标定曲线 / 尾部放大）
//   用法: node tools/cal-chart.cjs <battcal.csv> [输出.svg]
//   取数据: curl.exe -s "http://192.168.31.99/download?f=/battcal.csv" -o battcal.csv
//   转 PNG（可选）: msedge --headless --disable-gpu --screenshot=out.png --window-size=1140,1000 file:///.../out.svg
//   ⚠️ 内置 TBL 是 2026-09-16 首跑的 21 点表；换一次标定要从 GET /battcal/report 的 pct 数组更新它

// 由 /battcal.csv 生成 SVG：① 放电曲线 ② 标定曲线 ③ 尾部放大
const fs = require('fs');
const csv = fs.readFileSync(process.argv[2], 'utf8');
const rows = [];
for (const l of csv.split(/\r?\n/)) {
  if (!l.trim() || l[0] === '#') continue;
  if (l.startsWith('t_loaded')) continue;
  const c = l.split(',').map(Number);
  if (c.length < 9 || isNaN(c[0])) continue;
  rows.push({ t: c[0], dt: c[1], vl: c[2], vr: c[3], il: Math.abs(c[4]), ir: Math.abs(c[5]) });
}
const TBL = [2.945, 3.635, 3.710, 3.748, 3.792, 3.822, 3.842, 3.861, 3.874, 3.893, 3.919, 3.948, 3.975, 4.001, 4.015, 4.022, 4.029, 4.038, 4.054, 4.074, 4.148];
const n = rows.length, tEnd = rows[n - 1].t;
const W = 1140, H = 1000;
const F = 'Segoe UI,Microsoft YaHei,sans-serif';
const out = [];
const esc = s => String(s).replace(/&/g, '&amp;').replace(/</g, '&lt;');
const txt = (x, y, s, size = 13, anchor = 'start', fill = '#222', weight = 'normal') =>
  out.push(`<text x="${x.toFixed(1)}" y="${y.toFixed(1)}" font-family="${F}" font-size="${size}" text-anchor="${anchor}" fill="${fill}" font-weight="${weight}">${esc(s)}</text>`);
const line = (x1, y1, x2, y2, stroke, w = 1, dash = '') =>
  out.push(`<line x1="${x1.toFixed(1)}" y1="${y1.toFixed(1)}" x2="${x2.toFixed(1)}" y2="${y2.toFixed(1)}" stroke="${stroke}" stroke-width="${w}"${dash ? ` stroke-dasharray="${dash}"` : ''}/>`);
const rect = (x, y, w, h, fill) => out.push(`<rect x="${x.toFixed(1)}" y="${y.toFixed(1)}" width="${w.toFixed(1)}" height="${h.toFixed(1)}" fill="${fill}"/>`);

out.push(`<svg xmlns="http://www.w3.org/2000/svg" width="${W}" height="${H}" viewBox="0 0 ${W} ${H}">`);
rect(0, 0, W, H, '#ffffff');
txt(24, 32, 'PHT_2_0 电量标定首跑（2026-09-15 23:22 → 09-16 17:45）', 19, 'start', '#111', 'bold');
txt(24, 52, `110 样本 · 实测容量 1791mAh / 6.95Wh · 100%=4.148V  0%=2.945V · 负载 125mA / 静置 54mA`, 13, 'start', '#555');

// ---------- Panel A: 放电曲线 ----------
const A = { x0: 80, x1: 1090, y0: 80, y1: 380, vmin: 2.85, vmax: 4.22, tmin: 0, tmax: tEnd };
const AX = v => A.x0 + (v - A.tmin) / (A.tmax - A.tmin) * (A.x1 - A.x0);
const AY = v => A.y1 - (v - A.vmin) / (A.vmax - A.vmin) * (A.y1 - A.y0);
rect(A.x0, A.y0, A.x1 - A.x0, A.y1 - A.y0, '#fbfcfe');
// 低压区底色
const x35 = AX(rows.filter(r => r.vr >= 3.50).slice(-1)[0].t);
rect(x35, A.y0, A.x1 - x35, A.y1 - A.y0, '#fff4e8');
txt(A.x0 + 8, A.y0 + 18, '① 放电曲线：V_rest（静置电压）vs 累计加载时长', 14, 'start', '#1a3a5c', 'bold');
for (const v of [3.0, 3.2, 3.4, 3.6, 3.8, 4.0, 4.2]) {
  line(A.x0, AY(v), A.x1, AY(v), '#e6ebf0');
  txt(A.x0 - 8, AY(v) + 4, v.toFixed(1), 11, 'end', '#888');
}
for (const h of [0, 2, 4, 6, 8, 10, 12]) {
  const x = AX(h * 3600000);
  if (x > A.x1) continue;
  line(x, A.y0, x, A.y1, '#eef2f6');
  txt(x, A.y1 + 16, h + 'h', 11, 'middle', '#888');
}
for (const th of [3.60, 3.55, 3.50]) line(A.x0, AY(th), A.x1, AY(th), '#d9534f', 1, '4 3');
txt(A.x0 + 10, AY(3.60) - 6, '3.60V 存档SD', 10, 'start', '#d9534f');
txt(A.x0 + 135, AY(3.55) - 6, '3.55V 停采集', 10, 'start', '#d9534f');
txt(A.x0 + 262, AY(3.50) - 6, '3.50V 深睡', 10, 'start', '#d9534f');
out.push(`<polyline fill="none" stroke="#9fb6c9" stroke-width="1" points="${rows.map(r => `${AX(r.t).toFixed(1)},${AY(r.vl).toFixed(1)}`).join(' ')}"/>`);
out.push(`<polyline fill="none" stroke="#1a6db5" stroke-width="2" points="${rows.map(r => `${AX(r.t).toFixed(1)},${AY(r.vr).toFixed(1)}`).join(' ')}"/>`);
txt(AX(tEnd * 0.30), AY(4.05), 'V_load（带载）', 11, 'start', '#9fb6c9');
txt(AX(tEnd * 0.30), AY(3.93), 'V_rest（静置）', 11, 'start', '#1a6db5');
txt(A.x1 - 8, A.y0 + 34, '低压加密区（每轮 1min）', 11, 'end', '#c47a1a');
line(A.x0, A.y1, A.x1, A.y1, '#9aa5b1', 1);

// ---------- Panel B: 标定曲线（21 点表） ----------
const B = { x0: 80, x1: 1090, y0: 440, y1: 660, vmin: 2.85, vmax: 4.22 };
const BX = p => B.x0 + p / 100 * (B.x1 - B.x0);
const BY = v => B.y1 - (v - B.vmin) / (B.vmax - B.vmin) * (B.y1 - B.y0);
rect(B.x0, B.y0, B.x1 - B.x0, B.y1 - B.y0, '#fbfcfe');
txt(B.x0 + 8, B.y0 + 18, '② 标定曲线（固件里的 21 点表，电量% ← 电压）', 14, 'start', '#1a3a5c', 'bold');
for (const v of [3.0, 3.2, 3.4, 3.6, 3.8, 4.0, 4.2]) { line(B.x0, BY(v), B.x1, BY(v), '#e6ebf0'); txt(B.x0 - 8, BY(v) + 4, v.toFixed(1), 11, 'end', '#888'); }
for (let p = 0; p <= 100; p += 20) { const x = BX(p); line(x, B.y0, x, B.y1, '#eef2f6'); txt(x, B.y1 + 16, p + '%', 11, 'middle', '#888'); }
out.push(`<polyline fill="none" stroke="#1a6db5" stroke-width="2" points="${TBL.map((v, k) => `${BX(k * 5).toFixed(1)},${BY(v).toFixed(1)}`).join(' ')}"/>`);
TBL.forEach((v, k) => out.push(`<circle cx="${BX(k * 5).toFixed(1)}" cy="${BY(v).toFixed(1)}" r="${k % 4 === 0 ? 3.5 : 2}" fill="#d9534f"/>`));
line(B.x0, BY(3.50), B.x1, BY(3.50), '#d9534f', 1, '4 3');
txt(BX(3), BY(2.945) - 6, '0% = 2.945V（设备实际停在这里）', 11, 'start', '#d9534f');
txt(BX(97), BY(4.148) - 8, '100% = 4.148V', 11, 'end', '#d9534f');
txt(BX(6), BY(3.70) + 4, '5% 起一路平坦（3.635→4.148V）', 11, 'start', '#555');
line(B.x0, B.y1, B.x1, B.y1, '#9aa5b1', 1);

// ---------- Panel C: 尾部放大 ----------
const tail = rows.slice(-30);
const C = { x0: 80, x1: 1090, y0: 720, y1: 920, vmin: 2.90, vmax: 3.70, tmin: tail[0].t, tmax: tEnd };
const CX = t => C.x0 + (t - C.tmin) / (C.tmax - C.tmin) * (C.x1 - C.x0);
const CY = v => C.y1 - (v - C.vmin) / (C.vmax - C.vmin) * (C.y1 - C.y0);
rect(C.x0, C.y0, C.x1 - C.x0, C.y1 - C.y0, '#fbfcfe');
const cl = tail.filter(r => r.dt === 60000)[0];
if (cl) rect(CX(cl.t), C.y0, C.x1 - CX(cl.t), C.y1 - C.y0, '#fff4e8');
txt(C.x0 + 8, C.y0 + 18, '③ 尾部放大：最后 30 轮（0%~5% 那个 0.69V 台阶 = 电芯拐点，不是坏读数）', 14, 'start', '#1a3a5c', 'bold');
for (const v of [2.9, 3.0, 3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7]) { line(C.x0, CY(v), C.x1, CY(v), '#e6ebf0'); txt(C.x0 - 8, CY(v) + 4, v.toFixed(1), 11, 'end', '#888'); }
out.push(`<polyline fill="none" stroke="#1a6db5" stroke-width="2" points="${tail.map(r => `${CX(r.t).toFixed(1)},${CY(r.vr).toFixed(1)}`).join(' ')}"/>`);
tail.forEach(r => out.push(`<circle cx="${CX(r.t).toFixed(1)}" cy="${CY(r.vr).toFixed(1)}" r="2.4" fill="#d9534f"/>`));
line(C.x0, CY(3.50), C.x1, CY(3.50), '#d9534f', 1, '4 3');
txt(C.x0 + 30, C.y0 + 40, '← 9min/轮（正常档）', 11, 'start', '#555');
if (cl) {
  txt(C.x1 - 8, C.y0 + 20, '1min/轮（低压加密）', 11, 'end', '#c47a1a');
  txt(C.x1 - 8, C.y0 + 36, '（每轮只计 1min 加载）', 11, 'end', '#c47a1a');
}
txt(C.x1 - 6, CY(2.945) - 6, `末点 ${tail[tail.length - 1].vr.toFixed(3)}V（触发 3.0V 硬底线收工）`, 11, 'end', '#d9534f');
txt(CX(tail[0].t) + 4, CY(tail[0].vr) - 8, `${tail[0].vr.toFixed(3)}V`, 11, 'start', '#1a6db5');
line(C.x0, C.y1, C.x1, C.y1, '#9aa5b1', 1);
txt(C.x1, C.y1 + 16, '累计加载时长（越往右越晚）→', 11, 'end', '#888');
txt(C.x0, C.y1 + 16, `${(C.tmin / 3600000).toFixed(2)}h`, 11, 'start', '#888');

out.push('</svg>');
fs.writeFileSync(process.argv[3] || 'battcal.svg', out.join('\n'));
console.log('写入 ' + (process.argv[3] || 'battcal.svg') + '  (' + rows.length + ' 点)');
