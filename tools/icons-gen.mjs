// ============================================================================
//  icons-gen.mjs —— 自绘图标点阵生成器（生成 C++ 数组 + ASCII 预览）
//    node tools/icons-gen.mjs [--kind side|above] [--radii 5,9] [--span 45]
//  说明：屏幕上的 √ / × / 热点图标都是"固定点阵"（不用三角函数，避免浮点取整飘移；
//        也便于 tools 逐像素比对）。本脚本按同样算法生成点阵并打印可直接粘贴的 C++ 代码。
//    --kind side  : 中心点 + 双侧对称 <span>° 同心弧（热点图标就是这个，radii 给多个半径）
//    --kind above : 中心点 + 上方 <span>° 同心弧（WiFi 信号那种，顶栏在用）
//  生成的位序约定：每行一个整数，最左列 = 最高位（bit 位宽-1）；粘进 ui_display.h 后
//  用 px(cx - 偏左 + c, cy - 偏上 + y) 落笔。★ 改完记得同步 tools/lib.mjs 的 ICON。
// ============================================================================
import { arg, argList, hasFlag } from './lib.mjs';

const kind = arg('kind', 'side');
const radii = argList('radii').join(',').split(',').filter(s => s !== '').map(Number);
const span = +arg('span', 45);                     // 单侧张角（90° = span 45 即 ±45）
const dot = hasFlag('nodot') ? 0 : 1;

const pts = new Set();
const add = (x, y) => pts.add(x + ',' + y);
if (dot) for (let dx = -1; dx <= 1; dx++) for (let dy = -1; dy <= 1; dy++) add(dx, dy);
for (const r of radii) {
  for (let a = -span; a <= span; a++) {
    const rad = a * Math.PI / 180;
    const x = Math.round(r * Math.cos(rad)), y = -Math.round(r * Math.sin(rad));
    if (kind === 'above') add(x, y);                       // 上方（y 向上为负）
    else { add(x, y); add(-x, y); }                        // 双侧对称
  }
}
const P = [...pts].map(s => s.split(',').map(Number));
const x0 = Math.min(...P.map(p => p[0])), x1 = Math.max(...P.map(p => p[0]));
const y0 = Math.min(...P.map(p => p[1])), y1 = Math.max(...P.map(p => p[1]));
const W = x1 - x0 + 1, H = y1 - y0 + 1;

console.log(`${kind}  半径 [${radii}]  张角 ±${span}°  →  ${W}x${H}  相对锚点 x${x0}..${x1} y${y0}..${y1}\n`);
const rows = [];
for (let y = y0; y <= y1; y++) {
  let v = 0;
  for (let x = x0; x <= x1; x++) if (pts.has(x + ',' + y)) v |= 1 << (x1 - x);
  rows.push(v);
  let s = ''; for (let x = x0; x <= x1; x++) s += pts.has(x + ',' + y) ? '#' : '.';
  console.log('  ' + s);
}
console.log('\n--- 粘进 ui_display.h 的代码 ---');
console.log(`    void drawIcon(int cx, int cy) {`);
console.log(`        static const uint32_t r[${H}] = {`);
for (let i = 0; i < rows.length; i += 3)
  console.log('            ' + rows.slice(i, i + 3).map(v => '0x' + v.toString(16).toUpperCase().padStart(8, '0')).join(', ') + ',');
console.log(`        };`);
console.log(`        for (int y = 0; y < ${H}; y++)`);
console.log(`            for (int c = 0; c < ${W}; c++)`);
console.log(`                if (r[y] & (1u << (${x1 - x0} - c))) px(cx + ${x0} + c, cy + ${y0} + y, true);`);
console.log(`    }`);
