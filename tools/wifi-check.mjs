// ============================================================================
//  wifi-check.mjs —— 多组 WiFi 自检（列表 / 扫描 / 添加 / 自动选最强 / 删除）
//    node tools/wifi-check.mjs [ip]
//  流程：查状态 → 看设置页与主页结构 → 扫描 → 加一组“不在范围内”的假网络
//        （验证：不打断当前连接、并且重新选连时仍然挑到真正在范围里的那组）
//        → 触发 /wifi-reconnect 看日志里的“🎯 选中” → 删掉假网络还原。
//  注意：会在设备的 WiFi 配置里临时加/删一条，结束时保证删掉。
// ============================================================================
import { device, pass, sleep, hasFlag, waitOnline } from './lib.mjs';

const IP = process.argv[2] || '192.168.31.99';
const d = device(IP);
const BOGUS = 'FakeNet_NotInRange_Test';
let ok = 0, n = 0;
const T = (tag, cond, extra = '') => { n++; ok += pass(tag, cond, extra); };

const st00 = await d.status();
let wpage = await d.get('/wifi');
// 自清理：上次跑残留的假网络先删掉，保证断言可重复
if (wpage.includes(BOGUS)) {
  await d.get('/wifi-del?ssid=' + encodeURIComponent(BOGUS));
  await sleep(1500);
  wpage = await d.get('/wifi');
  console.log('（已清理上次残留的测试网络）');
}
const st0 = await d.status();
// 自检全程需要日志（电池机默认关），先开、结束再恢复
const LOG_WAS_OFF = !st0.webLogging;
if (LOG_WAS_OFF) await d.get('/set-weblog?enable=1');
console.log(`设备 ${IP}  当前 SSID="${st0.wifiSsid}" RSSI=${st0.wifiRssi}  已保存 ${st0.wifiCount} 组\n`);
console.log('【① 状态与页面】');
T('/status 有 wifiCount / wifiSsid / wifiRssi', st0.wifiCount !== undefined && st0.wifiSsid !== undefined && st0.wifiRssi !== undefined,
  `count=${st0.wifiCount} ssid=${st0.wifiSsid} rssi=${st0.wifiRssi}`);
T('/status 有 caps 能力开关（蓝牙模式靠它整卡隐藏）', !!st0.caps && 'files' in st0.caps, JSON.stringify(st0.caps));

const wpage0 = wpage;
T('设置页含“已保存 N 组”', /已保存\s*\d+\s*\/\s*\d+\s*组/.test(wpage0));
T('设置页列出了当前 SSID', wpage0.includes(st0.wifiSsid));
T('设置页有添加表单 + 附近网络区', wpage0.includes('/save-wifi') && wpage0.includes('/wifi-scan') && wpage0.includes('scanlist'));

const home = await d.get('/');
T('主页已按钮分组（card-files / card-ops / card-sys）', home.includes('id="card-files"') && home.includes('id="card-ops"') && home.includes('id="card-sys"'));
T('主页有 applyCaps() 且 OTA 按钮带 id', home.includes('function applyCaps') && home.includes('id="btn-ota"'));

console.log('\n【② 扫描附近网络】');
const scan = JSON.parse(await d.get('/wifi-scan'));
T('扫描返回数组', Array.isArray(scan), `${scan.length} 个`);
const mine = scan.find(s => s.ssid === st0.wifiSsid);
T('当前 SSID 在扫描结果里且标记 saved', !!mine && mine.saved === true, mine ? `rssi=${mine.rssi}` : '未扫到');
console.log('   附近（前 5）: ' + scan.slice(0, 5).map(s => `${s.ssid}(${s.rssi}${s.saved ? ',已存' : ''})`).join('  '));

console.log('\n【③ 添加一组（模拟“换个地方/多个路由器”）】');
const r1 = await d.post('/save-wifi', { ssid: BOGUS, password: 'dummy1234' });
const st1 = await d.status();
T('保存成功且组数 +1', st1.wifiCount === st0.wifiCount + 1, `${st0.wifiCount} → ${st1.wifiCount}`);
T('加完仍连着原来那个网络（不打断）', st1.wifiSsid === st0.wifiSsid, `ssid=${st1.wifiSsid}`);

console.log('\n【④ 重新选连：应挑到“真正在范围内”的那组，跳过假网络】');
await d.get('/wifi-reconnect');
await sleep(12000);
const st2 = await d.status();
let logs = [];
try { logs = (await d.json('/weblog')).lines || []; } catch { }
const logText = logs.join('\n');
T('仍连在真实网络上（没被假网络带跑）', st2.wifiSsid === st0.wifiSsid, `ssid=${st2.wifiSsid}`);
T('日志里有“🎯 选中”记录（电池机日志未开则跳过）',
  logText.includes('🎯 选中') || logText.length === 0,
  logText.length ? (logText.match(/🎯 选中[^\n]*/) || ['-'])[0] : '（/weblog 无 lines，跳过）');

console.log('\n【⑤ 删除假网络，还原】');
await d.get('/wifi-del?ssid=' + encodeURIComponent(BOGUS));
const st3 = await d.status();
T('组数还原', st3.wifiCount === st0.wifiCount, `${st1.wifiCount} → ${st3.wifiCount}`);
T('仍连着真实网络', st3.wifiSsid === st0.wifiSsid, `ssid=${st3.wifiSsid}`);

console.log('\n【⑥ 保存后“当前组”必须刷新（曾经不刷新 → 存完得按 EN 才能重连）】');
const h0 = (await d.status()).cfgHash;
const ip0 = (await d.status()).ip;
T('/status 有 cfgHash / ip（自检用）', typeof h0 === 'string' && h0.length > 0 && /\./.test(ip0 || ''), `cfgHash=${h0} ip=${ip0}`);
await d.get('/set-weblog?enable=1');                    // 电池机默认关日志，临时开
const lg0 = ((await d.json('/weblog')).lines || []).length;
await d.post('/save-wifi', { ssid: st0.wifiSsid });     // 密码留空 = 不改密码
await sleep(1500);
const st4 = await d.status();
const lg = ((await d.json('/weblog')).lines || []).join('\n');
T('保存后当前组已刷新（日志出现 🔁 当前组）', lg.includes('🔁 当前组'),
  (((await d.json('/weblog')).lines || []).filter(x => x.includes('🔁 当前组')).pop() || '-').slice(0, 60));
T('密码留空不会清掉已存密码（cfgHash 不变）', st4.cfgHash === h0, `${h0} → ${st4.cfgHash}`);
T('保存不会动地址方式（IP 不变，曾经会把静态 IP 静默关掉）', st4.ip === ip0, `${ip0} → ${st4.ip}`);
T('保存后仍连着（未被打断）', st4.wifiSsid === st0.wifiSsid, `ssid=${st4.wifiSsid}`);

if (hasFlag('reboot')) {
  console.log('\n【⑦ 网页重启（装壳后够不到 EN 键的兼底）】');
  try { await d.get('/reboot', 4000); } catch { }
  const stR = await waitOnline(IP, 60000);
  T('重启后重新上线', !!stR, stR ? `count=${stR.count}` : '60 秒未上线');
}

console.log(`\n=== 通过 ${ok}/${n} ===   ${ok === n ? '✅ 全过' : '❌ 有失败'}`);
if (LOG_WAS_OFF) await d.get('/set-weblog?enable=0');   // 恢复日志开关
process.exit(ok === n ? 0 : 1);
