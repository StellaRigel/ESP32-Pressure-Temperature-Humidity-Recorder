// ============================================================================
//  PHT_2_0/tools/lib.mjs —— 屏幕自检工具库
// ----------------------------------------------------------------------------
//  纯 Node（≥18）、无第三方依赖。给工程里所有"改屏幕/改版面"的脚本共用：
//    · 离线字模渲染（ui_font_oldsans.h / ui_cjk.h）—— 不接硬件也能预览版面
//    · 画布 + 图标点阵（与 ui_display.h 逐像素同参，改了固件要同步改这里）
//    · 抓真机显存 GET /ui-dump → 与离线渲染逐像素比对（本工程的验钞机）
//    · PNG 写盘（无依赖，自己拼 zlib+IHDR/IDAT/IEND）
//    · 设备 HTTP 小封装（/status /weblog /ui-dump /toast ...）
//  用法见 tools/README.md
// ============================================================================
import fs from 'fs';
import zlib from 'zlib';
import http from 'http';
import { fileURLToPath } from 'url';

export const ROOT = fileURLToPath(new URL('../', import.meta.url));   // → PHT_2_0/
export const LOG_W = 250, LOG_H = 122;                                // 逻辑屏（横屏）
export const sleep = ms => new Promise(r => setTimeout(r, ms));

// ---------------------------------------------------------------- 字模
// ui_font_oldsans.h 的结构：adv 表 + 字模表（按 '{ // [i]' 分段，每字形定长）
const FM = { top: { COLS: 20, BYTES: 3, ROWS: 19, PAD: 2 }, bot: { COLS: 24, BYTES: 4, ROWS: 27, PAD: 2 } };
const EXT_CP = [0xB0, 0xB5, 0xB1, 0xB7, 0xD7, 0xF7];   // 字模表里 ASCII 之后接着的 6 个符号 °µ±·×÷

export function loadFont(file = ROOT + 'ui_font_oldsans.h') {
  const txt = fs.readFileSync(file, 'utf8');
  const adv = n => txt.match(new RegExp(n + '\\[UIFONT_CHAR_COUNT\\]\\s*=\\s*\\{([^}]*)\\}', 's'))[1].match(/\d+/g).map(Number);
  const glyphs = n => {
    const s = txt.indexOf('static const uint8_t ' + n + '['), e = txt.indexOf('\n};', s);
    const parts = txt.slice(s, e).split(/\{\s*\/\/\s*\[(\d+)\]/).slice(1);
    const g = [];
    for (let i = 0; i < parts.length; i += 2) g[+parts[i]] = (parts[i + 1].match(/0x[0-9a-fA-F]{2}/g) || []).map(h => parseInt(h, 16));
    return g;
  };
  const advTop = adv('uiFontTopAdv'), advBot = adv('uiFontBotAdv');
  const glTop = glyphs('uiFontTop'), glBot = glyphs('uiFontBot');
  const idx = cp => (cp >= 0x20 && cp <= 0x7e) ? cp - 0x20 : 95 + Math.max(0, EXT_CP.indexOf(cp));

  const api = {
    advTop, advBot, glTop, glBot, idx,
    width: (s, top = false) => [...s].reduce((a, ch) => a + (top ? advTop : advBot)[idx(ch.codePointAt(0))], 0),
    // 按 ui_display.h::drawText 的笔位约定绘制（返回落笔后的 x）
    draw(c, lx, ly, s, top = false) {
      const m = top ? FM.top : FM.bot;
      for (const ch of s) {
        const fi = idx(ch.codePointAt(0)), ox = lx - m.PAD, arr = (top ? glTop : glBot)[fi];
        for (let col = 0; col < m.COLS; col++) {
          let bits = 0;
          for (let b = 0; b < m.BYTES; b++) bits = (bits | (arr[col * m.BYTES + b] << (8 * b))) >>> 0;
          for (let row = 0; row < m.ROWS; row++) if ((bits >>> row) & 1) c.px(ox + col, ly + row);
        }
        lx += (top ? advTop : advBot)[fi];
      }
      return lx;
    },
    // 水平居中（按 250 宽逻辑屏）
    drawC(c, ly, s, top = false) { api.draw(c, Math.floor((LOG_W - api.width(s, top)) / 2), ly, s, top); },
  };
  return api;
}

// ---------------------------------------------------------------- 画布
export function canvas(w = LOG_W, h = LOG_H) {
  const g = Array.from({ length: h }, () => new Array(w).fill(0));
  const c = {
    w, h, g,
    px(x, y, v = 1) { if (x >= 0 && x < c.w && y >= 0 && y < c.h) c.g[y][x] = v; },
    fill(x0, y0, x1, y1, v = 0) { for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) c.px(x, y, v); },
    art(x0, y0, rows) { rows.forEach((row, y) => [...row].forEach((ch, x) => { if (ch === '#') c.px(x0 + x, y0 + y); })); },
    ink(x0, y0, x1, y1) { let n = 0; for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) if (c.g[y][x]) n++; return n; },
    bbox(x0, y0, x1, y1) {
      let a = 1e9, b = -1, p = 1e9, q = -1;
      for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) if (c.g[y][x]) { if (x < a) a = x; if (x > b) b = x; if (y < p) p = y; if (y > q) q = y; }
      return b <= -1 ? null : { x0: a, x1: b, y0: p, y1: q };
    },
    ascii(x0 = 0, y0 = 0, x1 = c.w - 1, y1 = c.h - 1) {
      const L = [];
      for (let y = y0; y <= y1; y++) {
        let s = ''; for (let x = x0; x <= x1; x++) s += c.g[y][x] ? '#' : '.';
        s = s.replace(/\.+$/, ''); if (s) L.push(String(y).padStart(3) + '|' + s);
      }
      return L.join('\n');
    },
  };
  return c;
}

// ---------------------------------------------------------------- 图标点阵
// ★ 与 ui_display.h 里的自绘点阵必须逐位一致（改了固件同步改这里）
export const ICON = {
  tick:  [0x0006, 0x000C, 0x0018, 0x0030, 0x0860, 0x0CC0, 0x0780, 0x0300],                     // 13x8  左上位(12-c)
  cross: [0x0603, 0x0306, 0x018C, 0x00D8, 0x0070, 0x00D8, 0x018C, 0x0306, 0x0603],             // 11x9  左上位(10-c)
  bcast: [0x0001800C, 0x00030006, 0x00022022, 0x00062023, 0x00046031, 0x00044711, 0x00044711,  // 19x13 中心点 位(18-c)
          0x00044711, 0x00046031, 0x00062023, 0x00022022, 0x00030006, 0x0001800C],
};
export const drawTick  = (c, lx, ly) => ICON.tick.forEach((b, y)  => { for (let i = 0; i < 13; i++) if (b & (1 << (12 - i))) c.px(lx + i, ly + y); });
export const drawCross = (c, lx, ly) => ICON.cross.forEach((b, y) => { for (let i = 0; i < 11; i++) if (b & (1 << (10 - i))) c.px(lx + i, ly + y); });
export const drawBcast = (c, cx, cy) => ICON.bcast.forEach((b, y) => { for (let i = 0; i < 19; i++) if (b & (1 << (18 - i))) c.px(cx - 9 + i, cy - 6 + y); });

// ---------------------------------------------------------------- Toast 版式
// ★ 与 ui_display.h::toastLine1 / showToast 同参：kind = ok|fail|ap
export const TOAST_Y = { L1: 30, L2: 63, L3: 87 };
const KIND = { ok: { iw: 13, gap: 8 }, fail: { iw: 11, gap: 8 }, ap: { iw: 21, gap: 7 } };
export function drawToast(c, font, kind, l1, l2, l3) {
  c.fill(0, 22, LOG_W - 1, LOG_H - 1, 0);                 // 数据区刷白（含中文标签）
  const k = KIND[kind] || KIND.ok;
  const x = Math.floor((LOG_W - (k.iw + k.gap + font.width(l1, false))) / 2);
  if (kind === 'fail') drawCross(c, x, 35);
  else if (kind === 'ap') drawBcast(c, x + 10, 43);
  else drawTick(c, x, 35);
  font.draw(c, x + k.iw + k.gap, TOAST_Y.L1, l1, false);
  font.drawC(c, TOAST_Y.L2, l2, true);
  font.drawC(c, TOAST_Y.L3, l3, false);
  return c;
}

// ---------------------------------------------------------------- 设备交互
export function device(ip) {
  const get = (path, timeout = 25000) => new Promise((res, rej) => {
    const rq = http.get({ host: ip, path, timeout }, r => { let d = ''; r.setEncoding('utf8'); r.on('data', c => d += c); r.on('end', () => res(d)); });
    rq.on('error', rej); rq.on('timeout', () => { rq.destroy(); rej(new Error('timeout ' + path)); });
  });
  // 表单 POST（application/x-www-form-urlencoded）
  const post = (path, body = {}, timeout = 25000) => new Promise((res, rej) => {
    const data = new URLSearchParams(body).toString();
    const rq = http.request({ host: ip, path, method: 'POST', timeout,
      headers: { 'Content-Type': 'application/x-www-form-urlencoded', 'Content-Length': Buffer.byteLength(data) } },
      r => { let d = ''; r.setEncoding('utf8'); r.on('data', c => d += c); r.on('end', () => res(d)); });
    rq.on('error', rej); rq.on('timeout', () => { rq.destroy(); rej(new Error('timeout ' + path)); });
    rq.write(data); rq.end();
  });
  const api = {
    ip, get, post,
    json: async path => JSON.parse(await get(path)),
    // 抓显存 → 直接返回画布（额外挂 rows/width 供自检用）
    dump: async (x0, y0, x1, y1) => {
      const r = parseDump(await get(x0 === undefined ? '/ui-dump' : `/ui-dump?x0=${x0}&y0=${y0}&x1=${x1}&y1=${y1}`));
      r.c.rows = r.rows; r.c.width = r.width;
      return r.c;
    },
  };
  api.status = () => api.json('/status');
  return api;
}
export async function waitOnline(ip, timeoutMs = 60000) {
  const t0 = Date.now();
  while (Date.now() - t0 < timeoutMs) {
    try { const d = device(ip); const s = JSON.parse(await d.get('/status', 4000)); return s; } catch { }
    await sleep(2000);
  }
  return null;
}

// ---------------------------------------------------------------- dump 解析 / 比对
export function parseDump(text) {
  const c = canvas();
  let rows = 0, xs = null;
  for (const line of text.split('\n')) {
    const m = line.match(/^\s*(\d+)\|(.*)$/);
    if (!m) continue;
    const y = +m[1]; rows++;
    const body = m[2];
    for (let i = 0; i < Math.min(body.length, LOG_W); i++) if (body[i] === '#') c.g[y][i] = 1;
    if (xs === null) xs = body.length;
  }
  return { c, rows, width: xs };
}
export function diff(a, b, x0 = 0, y0 = 22, x1 = LOG_W - 1, y1 = LOG_H - 1) {
  let n = 0; const samples = [];
  for (let y = y0; y <= y1; y++) for (let x = x0; x <= x1; x++) {
    if ((a.g[y][x] ? 1 : 0) !== (b.g[y][x] ? 1 : 0)) { n++; if (samples.length < 12) samples.push(`x${x},y${y} 设备=${a.g[y][x] ? 1 : 0} 期望=${b.g[y][x] ? 1 : 0}`); }
  }
  return { n, samples };
}

// ---------------------------------------------------------------- PNG（无依赖）
export function png(canvases, { scale = 3, gap = 8, sepLevel = 150 } = {}) {
  const list = Array.isArray(canvases) ? canvases : [canvases];
  const W = Math.max(...list.map(c => c.w)), H = Math.max(...list.map(c => c.h));
  const N = list.length, TOTH = H * N + gap * (N - 1);
  const OW = W * scale, OH = TOTH * scale;
  const gray = new Uint8Array(OW * OH).fill(255);
  list.forEach((c, i) => {
    const oy = i * (H + gap);
    for (let y = 0; y < c.h; y++) for (let x = 0; x < c.w; x++) {
      if (!c.g[y][x]) continue;
      for (let dy = 0; dy < scale; dy++) for (let dx = 0; dx < scale; dx++) gray[((oy + y) * scale + dy) * OW + x * scale + dx] = 0;
    }
  });
  for (let i = 1; i < N; i++) {                          // 面板之间画虚线
    const y = (i * (H + gap) - Math.ceil(gap / 2)) * scale;
    for (let x = 0; x < OW; x += 2 * scale) for (let d = 0; d < scale; d++) gray[(y + d) * OW + x] = sepLevel;
  }
  const crc = b => { let c = ~0; for (const v of b) { c ^= v; for (let k = 0; k < 8; k++) c = (c >>> 1) ^ (0xEDB88320 & -(c & 1)); } return (~c) >>> 0; };
  const chunk = (t, d) => { const l = Buffer.alloc(4); l.writeUInt32BE(d.length); const T = Buffer.from(t, 'ascii'); const z = Buffer.alloc(4); z.writeUInt32BE(crc(Buffer.concat([T, d]))); return Buffer.concat([l, T, d, z]); };
  const raw = Buffer.alloc((OW + 1) * OH);
  for (let y = 0; y < OH; y++) { raw[y * (OW + 1)] = 0; Buffer.from(gray.buffer, y * OW, OW).copy(raw, y * (OW + 1) + 1); }
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(OW, 0); ihdr.writeUInt32BE(OH, 4); ihdr[8] = 8; ihdr[9] = 0;
  return Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]);
}
export function savePng(path, data) { fs.writeFileSync(path, data); return path; }

// ---------------------------------------------------------------- 小工具
export const arg = (name, dflt) => { const i = process.argv.indexOf('--' + name); return i >= 0 ? process.argv[i + 1] : dflt; };
// 收多个值：兼容 `--radii 6 10`（PowerShell 会把 6,10 当数组拆开）与 `--radii "6,10"`
export const argList = name => { const v = []; for (let i = 0; i < process.argv.length; i++) if (process.argv[i] === '--' + name) for (let j = i + 1; j < process.argv.length && !process.argv[j].startsWith('--'); j++) v.push(process.argv[j]); return v; };
export const hasFlag = name => process.argv.includes('--' + name);
export const pass = (tag, ok, extra = '') => { console.log(`   ${ok ? '✅' : '❌'} ${tag}${extra ? '  ' + extra : ''}`); return ok ? 1 : 0; };
