// ============================================================================
//  screen-check.mjs —— 平时版面自检（抓真机显存，不用人看屏）
//    node tools/screen-check.mjs [ip]
//  退出码 0 = 全过，1 = 有失败（可挂进脚本）
// ============================================================================
import { device, pass } from './lib.mjs';

const IP = process.argv[2] || '192.168.31.99';
const d = device(IP);
const c = await d.dump();
const rows = c.rows;

let ok = 0, n = 0;
const T = (tag, cond, extra) => { n++; ok += pass(tag, cond, extra); };

console.log(`设备 ${IP}  平时版面自检\n`);
T(`dump 完整（${rows}/122 行）`, rows === 122);
const leftBlank = c.ink(0, 23, 4, 121);
T('最左留白 x0..4 为空', leftBlank === 0, `ink=${leftBlank}`);
const lab = c.ink(6, 25, 54, 112);
T('三个中文标签 = 1261 点（与删掉的底图逐像素一致）', lab === 1261, `实测 ${lab}`);
const data = c.ink(60, 23, 249, 121);
T('数值区有内容 x60..249', data > 500, `ink=${data}`);
const top = c.ink(0, 0, 249, 22);
T('顶栏有内容', top > 200, `ink=${top}`);
const b = c.bbox(60, 23, 249, 121);
T('数值区包围盒不贴边', !!b && b.x1 < 249 && b.y1 < 121, b ? `x${b.x0}..${b.x1} y${b.y0}..${b.y1}` : '空');

console.log('\n--- 顶栏 y0..22 ---\n' + c.ascii(0, 0, 249, 22));
console.log('\n--- 数据区 y23..121 ---\n' + c.ascii(0, 23, 249, 121));
console.log(`\n=== ${ok}/${n} ===   ${ok === n ? '✅ 正常' : '❌ 有问题'}`);
if (ok !== n) console.log('提示：若屏幕此刻正显示 Toast，标签区会被盖住 —— 等横幅过期再测。');
process.exit(ok === n ? 0 : 1);
