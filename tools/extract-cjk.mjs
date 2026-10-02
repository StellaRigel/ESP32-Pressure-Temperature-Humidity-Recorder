// ============================================================================
//  extract-cjk.mjs —— 中文字模反向提取：底图 → ui_cjk.h
//    node tools/extract-cjk.mjs
//  当初 ui_cjk.h 就是这么来的：把原底图 backups/ui_landscape.h 里三个静态标签
//  「温度 / 湿度 / 气压」的像素逐字抠出来，再做往返比对（0 差异）后生成字模头文件。
//  底图已从工程里删掉（只留 backups/），本脚本用于：① 复现/验证字模 ② 以后要从底图
//  再抠别的字时照这个流程走。★ 会覆盖 ui_cjk.h，跑之前先确认底图没变。
// ============================================================================
import fs from 'fs';
import { ROOT } from './lib.mjs';

const SRC = ROOT + 'backups/ui_landscape.h';
const OUT = ROOT + 'ui_cjk.h';
const W = 250, H = 122;

const txt = fs.readFileSync(SRC, 'utf8');
const bytes = txt.match(/const unsigned char ui_landscape\[\]\s*=\s*\{([\s\S]*?)\};/)[1]
  .match(/0x[0-9a-fA-F]{2}/g).map(h => parseInt(h, 16));
if (bytes.length !== 4125) throw new Error('字节数不对: ' + bytes.length);

// 逻辑(lx,ly) → 底图位。物理面板 122x250：phys_x = 121-ly (+10 COL_OFFSET)，phys_y = lx
const px = (lx, ly) => {
  const X = 131 - ly;
  if (X < 0 || X > 131) return 0;
  const idx = ((lx >> 1) * 33) + (X >> 2);
  return (bytes[idx] >> (7 - (((X % 4) * 2) + (lx % 2)))) & 1;
};

// ---- 1. 按"带"切出三行标签，再按空列切成两个字 ----
const inkRows = [];
for (let ly = 0; ly < H; ly++) { let any = 0; for (let lx = 0; lx < W && !any; lx++) any = px(lx, ly); if (any) inkRows.push(ly); }
const bands = []; let cur = [inkRows[0]];
for (let i = 1; i < inkRows.length; i++) { if (inkRows[i] - inkRows[i - 1] >= 3) { bands.push(cur); cur = []; } cur.push(inkRows[i]); }
bands.push(cur);

const WORDS = ['温度', '湿度', '气压'];
const PEN_X = 6, ADV = 26;      // 笔位：首字 x=6，字距 26（第二字 x=32），与底图一致
const gl = {}, order = [];

bands.forEach((rows, bi) => {
  const y0 = rows[0], y1 = rows[rows.length - 1];
  let mnx = 1e9, mxx = -1;
  for (const y of rows) for (let x = 0; x < W; x++) if (px(x, y)) { if (x < mnx) mnx = x; if (x > mxx) mxx = x; }
  const gaps = []; let s = -1;                     // 找两个字中间的空列带
  for (let x = mnx; x <= mxx + 1; x++) {
    const blank = x <= mxx ? (() => { for (const y of rows) if (px(x, y)) return false; return true; })() : true;
    if (blank && s < 0) s = x;
    if (!blank && s >= 0) { gaps.push([s, x - 1]); s = -1; }
  }
  const mid = (mnx + mxx) / 2;
  gaps.sort((a, b) => Math.abs((a[0] + a[1]) / 2 - mid) - Math.abs((b[0] + b[1]) / 2 - mid));
  const split = gaps[0] ? gaps[0][0] : Math.round(mid);
  console.log(`带${bi + 1} ${WORDS[bi]}  y${y0}..${y1}  x${mnx}..${mxx}  分割列=${split}`);

  [0, 1].forEach(ci => {
    const xa = ci === 0 ? mnx : split, xb = ci === 0 ? split - 1 : mxx;
    let gx0 = 1e9, gx1 = -1, gy0 = 1e9, gy1 = -1;
    for (let y = y0; y <= y1; y++) for (let x = xa; x <= xb; x++) if (px(x, y)) { if (x < gx0) gx0 = x; if (x > gx1) gx1 = x; if (y < gy0) gy0 = y; if (y > gy1) gy1 = y; }
    const w = gx1 - gx0 + 1, h = gy1 - gy0 + 1, rowsBits = [];
    for (let y = gy0; y <= gy1; y++) { const r = []; for (let x = gx0; x <= gx1; x++) r.push(px(x, y)); rowsBits.push(r); }
    const cp = WORDS[bi].codePointAt(ci);
    const xoff = gx0 - (PEN_X + ci * ADV), yoff = gy0 - y0;
    const prev = gl[cp];
    if (prev) {
      const same = prev.w === w && prev.h === h && prev.xoff === xoff && prev.yoff === yoff &&
        JSON.stringify(prev.rowsBits) === JSON.stringify(rowsBits);
      console.log(`  字${ci + 1} U+${cp.toString(16).toUpperCase()} ${String.fromCodePoint(cp)}  ${w}x${h} xoff=${xoff} yoff=${yoff}  ${same ? '与已有字模一致 ✅' : '⚠️ 与已有不同，需独立存'}`);
      if (!same) { const key = cp + '_' + ci; order.push(key); gl[key] = { cp, w, h, xoff, yoff, rowsBits, ch: String.fromCodePoint(cp), dup: true }; }
    } else {
      gl[cp] = { cp, w, h, xoff, yoff, rowsBits, ch: String.fromCodePoint(cp) };
      order.push(cp);
      console.log(`  字${ci + 1} U+${cp.toString(16).toUpperCase()} ${String.fromCodePoint(cp)}  ${w}x${h} xoff=${xoff} yoff=${yoff}`);
    }
  });
});

// ---- 2. 往返验证 ----
const rt = Array.from({ length: H }, () => new Array(W).fill(0));
const drawCp = (cp, x0, y0, ci) => {
  const g = gl[cp]; if (!g) return;
  for (let r = 0; r < g.h; r++) for (let ccc = 0; ccc < g.w; ccc++)
    if (g.rowsBits[r][ccc]) rt[y0 + g.yoff + r][x0 + g.xoff + ccc] = 1;
};
bands.forEach((rows, bi) => { for (let ci = 0; ci < 2; ci++) drawCp(WORDS[bi].codePointAt(ci), PEN_X + ci * ADV, rows[0], ci); });
let diff = 0, inkOrig = 0;
for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) { const o = px(x, y); if (o) inkOrig++; if (o !== rt[y][x]) diff++; }
console.log(`往返比对：原墨迹 ${inkOrig} 像素，重绘差异 ${diff} 像素  ${diff === 0 ? '✅ 完全一致' : '❌ 不一致（不写出）'}`);
if (diff !== 0) process.exit(1);

// ---- 3. 生成 ui_cjk.h ----
const L = [];
L.push('#pragma once');
L.push('// ============================================================================');
L.push('//  ui_cjk.h —— 中文点阵（23px），从原底图 ui_landscape.h 里【原样抠出】，勿手改');
L.push('// ----------------------------------------------------------------------------');
L.push('//  「温 度 湿 气 压」五个字形 = 底图里三个静态标签的全部像素，');
L.push('//  底图已删除，改为由 ui_display.h 动态绘制（drawCjkText）。');
L.push('//  字形 = 墨迹包围盒(rowsBits, 按行 MSB 在左) + 相对笔位的 (xoff,yoff)');
L.push('//  笔位约定：首字 x=6，字距 ADV=26（即第二字 x=32）；带顶 y=25/57/90');
L.push('//  生成：tools/extract-cjk.mjs（反解底图 + 往返比对 0 差异）');
L.push('// ============================================================================');
L.push('#include <Arduino.h>');
L.push('');
L.push('#define UI_CJK_ADV   26     // 中文字距（与底图原样一致）');
L.push('');
L.push('typedef struct {');
L.push('  uint32_t cp;            // Unicode 码点');
L.push('  uint8_t  w, h;          // 包围盒宽高');
L.push('  int8_t   xoff, yoff;    // 相对笔位的偏移');
L.push('  const uint8_t *bits;    // 逐行打包，MSB=最左列');
L.push('} UiCjkGlyph;');
L.push('');
order.forEach(k => {
  const g = gl[k];
  const stride = Math.ceil(g.w / 8), arr = [];
  for (let r = 0; r < g.h; r++) for (let b = 0; b < stride; b++) {
    let v = 0;
    for (let i = 0; i < 8; i++) { const c = b * 8 + i; if (c < g.w && g.rowsBits[r][c]) v |= (1 << (7 - i)); }
    arr.push(v);
  }
  const nm = 'cjk_' + g.cp.toString(16);
  L.push(`// ${g.ch}  ${g.w}x${g.h}  xoff=${g.xoff} yoff=${g.yoff}`);
  L.push(`static const uint8_t ${nm}[${arr.length}] = {`);
  for (let i = 0; i < arr.length; i += 16) L.push('  ' + arr.slice(i, i + 16).map(v => '0x' + v.toString(16).padStart(2, '0')).join(', ') + ',');
  L.push('};');
  L.push('');
  g.arrName = nm;
});
L.push('static const UiCjkGlyph uiCjkTable[] = {');
order.forEach(k => { const g = gl[k]; L.push(`  { 0x${g.cp.toString(16).toUpperCase()}, ${g.w}, ${g.h}, ${g.xoff}, ${g.yoff}, ${g.arrName} },  // ${g.ch}`); });
L.push('};');
L.push('static const int uiCjkCount = sizeof(uiCjkTable) / sizeof(uiCjkTable[0]);');
L.push('');
fs.writeFileSync(OUT, L.join('\n'), 'utf8');
console.log(`\n已写出: ${OUT}  (${fs.statSync(OUT).size} 字节, ${order.length} 个字形)`);
