import http from 'http';
const IP = process.argv[2] || '192.168.31.99';

const get = p => new Promise((res, rej) => {
  const rq = http.get({ host: IP, path: p, timeout: 25000 }, r => { let d = ''; r.on('data', c => d += c); r.on('end', () => res(d)); });
  rq.on('error', rej); rq.on('timeout', () => { rq.destroy(); rej(new Error('timeout')); });
});

const dump = await get('/ui-dump');
const grid = Array.from({ length: 122 }, () => new Array(250).fill(0));
let rows = 0;
for (const line of dump.split('\n')) {
  const m = line.match(/^\s*(\d+)\|(.*)$/);
  if (!m) continue;
  const y = +m[1]; rows++;
  for (let i = 0; i < m[2].length && i < 250; i++) grid[y][i] = m[2][i] === '#' ? 1 : 0;
}
console.log(`解析到 ${rows}/122 行`);

const region = (x0, y0, x1, y1) => { let n = 0; for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) if (grid[y][x]) n++; return n; };
const bbox = (x0, y0, x1, y1) => { let mnx = 999, mxx = -1, mny = 999, mxy = -1; for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) if (grid[y][x]) { if (x < mnx) mnx = x; if (x > mxx) mxx = x; if (y < mny) mny = y; if (y > mxy) mxy = y; } return mnx > mxx ? '空' : `x${mnx}..${mxx} y${mny}..${mxy}`; };

console.log('\n区域墨迹统计（逻辑坐标）:');
const checks = [
  ['顶栏 y0..22', 0, 0, 249, 22],
  ['标签列 x0..58 y23..121（应只有三个中文标签）', 0, 23, 58, 121],
  ['最左空白 x0..4 y23..121（应为 0）', 0, 23, 4, 121],
  ['数据区 x60..249 y23..121（应有数字/箭头）', 60, 23, 249, 121],
  ['底部空白 y114..121 x60..249', 60, 114, 249, 121],
];
for (const [name, a, b, c, d] of checks) console.log(`  ${name}: ${region(a, b, c, d)} 像素  包围盒 ${bbox(a, b, c, d)}`);

console.log('\n顶栏预览 (y0..22)');
for (let y = 0; y <= 22; y++) console.log(String(y).padStart(3) + '|' + grid[y].map(v => v ? '#' : '.').join('').replace(/\.+$/, ''));
