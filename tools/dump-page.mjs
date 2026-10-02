// ============================================================
// dump-page.mjs —— 把设备上的完整网页存成本地文件（手机用）
//   蓝牙只读页的原理：同一份 HTML，WiFi 下走 fetch、蓝牙下走 BLE。
//   手机要用蓝牙时，得先把这份 HTML 存成手机本地文件（file:// 才算安全上下文），
//   在 Android Chrome 里打开后点「🔗 蓝牙连接」即可。
//
// 用法：node tools/dump-page.mjs [IP] [输出文件名]
//   node tools/dump-page.mjs 192.168.31.99 bt-page.html
// ============================================================
import fs from 'fs';

const ip  = process.argv[2] || '192.168.31.99';
const out = process.argv[3] || 'bt-page.html';

const r = await fetch(`http://${ip}/`);
if (!r.ok) { console.error(`❌ 取页面失败：HTTP ${r.status}`); process.exit(1); }
const html = await r.text();

const checks = [
  ['含传输层 apiFetch', html.includes('function apiFetch(')],
  ['含蓝牙连接按钮',   html.includes('bleConnect()')],
  ['含技能开关 applyCaps', html.includes('function applyCaps(')],
];
fs.writeFileSync(out, html, 'utf8');

console.log(`✅ 已从 ${ip} 保存页面 → ${out}（${(html.length / 1024).toFixed(1)} KB）`);
for (const [name, ok] of checks) console.log(`   ${ok ? '✅' : '❌'} ${name}`);
if (checks.some(c => !c[1])) {
  console.log('   ⚠️ 有自检项没通过：设备固件可能不是最新（先 OTA 再取页面）');
}
console.log('\n手机使用步骤：');
console.log('  1) 把这个 html 传到手机（微信「文件传输助手」/QQ/数据线都行）');
console.log('  2) 手机上用 Chrome 打开它（本地文件）');
console.log('  3) 让设备进蓝牙模式（移动+电池，或浏览器打开 /wireless?force=ble&sec=900）');
console.log('  4) 页面顶部点「🔗 蓝牙连接」→ 选 PHT_2_0');
