#!/usr/bin/env node
/**
 * check-md.mjs —— Markdown 文档体检（踩坑沉淀，2026-09-15）
 * ============================================================
 * 查两类"整段渲染不出来"的经典事故：
 *
 *  ① **表格里的裸竖线**：GFM 里哪怕竖线写在反引号代码里，不转义也会被当成列分隔符，
 *     那一行多出几格 → 列数对不上表头 → 表格连同后续内容一起错乱。
 *     → 表格单元格里的竖线一律写成 `\|`（渲染出来仍是 `|`）。
 *
 *  ② **未配对的代码围栏 ``` **：多留一个会开启一个永不关闭的代码块，
 *     **把后面整篇内容吞进去**（表现为"前面正常、后面全坏"）。
 *     实测事故：设计文档在取消蓝牙提示时删了内容但漏删围栏，465 行留了个孤儿 ```，
 *     把后面 200 多行全吞了。
 *
 * 用法：
 *   node tools/check-md.mjs                 # 默认扫本工程（tools 的上一级）所有 *.md
 *   node tools/check-md.mjs a.md b.md …      # 指定文件/目录
 *   退出码 0 = 无问题，1 = 有异常
 */
import fs from 'fs';
import path from 'path';

const isRow = s => /^\s*\|/.test(s);
// 剥掉首尾竖线后，按【未转义】竖线切分（\| 不算分列符）
const cells = s => s.trim().replace(/^\||\|$/g, '').split(/(?<!\\)\|/).length;
const isFence = s => /^\s{0,3}```/.test(s);
const isHeading = s => /^\s{0,3}#{1,6}\s/.test(s);

function collect(target, out) {
  const st = fs.statSync(target);
  if (st.isFile()) { if (/\.md$/i.test(target)) out.push(target); return; }
  for (const e of fs.readdirSync(target, { withFileTypes: true })) {
    if (e.name === 'node_modules' || e.name === '.git') continue;
    collect(path.join(target, e.name), out);
  }
}

const args = process.argv.slice(2);
const root = path.resolve(process.argv[1], '..', '..');   // tools/ 的上一级 = 工程根
const files = [];
for (const t of (args.length ? args : [root])) if (fs.existsSync(t)) collect(path.resolve(t), files);

let bad = 0;
for (const f of files) {
  const lines = fs.readFileSync(f, 'utf8').split(/\r?\n/);
  const rel = path.relative(root, f).replace(/\\/g, '/') || f;
  const say = m => { console.log(m); bad++; };

  // ---------- ① 表格 ----------
  let i = 0;
  while (i < lines.length) {
    if (isRow(lines[i])) {
      const a = i; while (i < lines.length && isRow(lines[i])) i++;
      const n0 = cells(lines[a]);
      for (let k = a + 1; k < i; k++) {
        const n = cells(lines[k]);
        if (n !== n0) {
          say(`❌ ${rel}:${k + 1}  表格 ${n} 格（表头 ${n0} 格）→ 把裸竖线改成 \\|`);
          console.log(`   ${lines[k].slice(0, 150)}`);
        }
      }
    } else i++;
  }

  // ---------- ② 代码围栏 ----------
  const fences = [];
  for (let k = 0; k < lines.length; k++) if (isFence(lines[k])) fences.push(k);
  const tag = l => (l.match(/^\s{0,3}```(\S*)/) || [, ''])[1];      // 围栏后的语言标记
  if (fences.length % 2) {
    say(`❌ ${rel}  围栏数 ${fences.length}（奇数）→ 有未配对的 \`\`\`，后面内容会被吞进代码块`);
    console.log(`   围栏行号: ${fences.map(x => x + 1).join(', ')}`);
  }
  // 逐对检查：**裸围栏**（无语言标记）的块里出现标题 → 起点疑为孤儿围栏
  //   （带标记的块如 ```powershell 里的 `# 注释` 是合法的，不算；）
  for (let n = 0; n + 1 < fences.length; n += 2) {
    const [a, b] = [fences[n], fences[n + 1]];
    if (tag(lines[a])) continue;
    for (let k = a + 1; k < b; k++) {
      if (isHeading(lines[k]) || /^\s*\|?[-:| ]{5,}\|?\s*$/.test(lines[k])) {
        say(`❌ ${rel}:${a + 1}  该裸代码块（${a + 1}~${b + 1} 行）里出现了标题/表格线 → 起点疑为孤儿围栏`);
        console.log(`   块内首个可疑行在 ${k + 1} 行: ${lines[k].slice(0, 100)}`);
        break;
      }
    }
  }
}

console.log(`\n扫描 ${files.length} 个 md 文件，异常 ${bad} 处。${bad ? '' : '✅ 表格与代码围栏都正常'}`);
process.exit(bad ? 1 : 0);
