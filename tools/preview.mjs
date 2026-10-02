// ============================================================================
//  preview.mjs —— 离线版面预览（不接硬件，用字模把版面画出来存 PNG）
//    node tools/preview.mjs --out p.png [--scale 3] [--panel "kind|l1|l2|l3"] ...
//    kind = ok | fail | ap | plain（plain = 只画数据区的三个中文标签位）
//  例：node tools/preview.mjs --panel "ap|AP MODE|SSID \"X\"|192.168.5.1"
//  不加 --panel 时默认渲染三种标准 Toast 版面
// ============================================================================
import { loadFont, canvas, drawToast, png, savePng, arg, LOG_W, LOG_H } from './lib.mjs';

const OUT = arg('out', 'preview.png');
const SCALE = +arg('scale', 3);
const panels = [];
const raw = process.argv.slice(2);
for (let i = 0; i < raw.length; i++) if (raw[i] === '--panel') panels.push(raw[i + 1]);

const font = loadFont();
const draw = spec => {
  const [kind, l1 = '', l2 = '', l3 = ''] = spec.split('|');
  const c = canvas();
  if (kind === 'plain') {                            // 平时版面：只示意数据区
    font.draw(c, 6, 25, 'TEMP', true); font.draw(c, 6, 57, 'HUM', true); font.draw(c, 6, 90, 'PRES', true);
  } else drawToast(c, font, kind, l1, l2, l3);
  return c;
};

const specs = panels.length ? panels : [
  'ok|WiFi OK|SSID "mywifi"|192.168.1.9',
  'fail|WiFi FAIL|SSID "mywifi"|not connected',
  'ap|AP MODE|SSID "Weather_Station_Pro"|192.168.5.1',
];
savePng(OUT, png(specs.map(draw), { scale: SCALE }));
console.log(`渲染 ${specs.length} 块（${LOG_W}x${LOG_H}，x${SCALE}）→ ${OUT}`);
specs.forEach((s, i) => console.log(`  ${i + 1}. ${s}`));
