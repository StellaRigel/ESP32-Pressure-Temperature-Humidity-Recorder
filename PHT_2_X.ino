/**
 * PHT_2_X —— ESP32-S3 气压温湿度记录器
 *   = Pressure1_3 主逻辑  +  横屏 UI（ST7305_Landscape_UI）合并版
 *
 *   - BMP580 + SHT30 + DS3231 + SD卡
 *   - 屏幕：ST7305 2.13" 横屏 UI（ui_display.h + 中文点阵 ui_cjk.h，无底图）
 *       顶栏： 时间 · MOV/FIX模式 · 充电⚡CC/CV 或 电量% · 电池 · WiFi
 *       数据： 温度°C / 湿度% / 气压hPa
 *   - 设备模式 固定(0)/移动(1)：存 NVS，顶栏显 FIX/MOV，网页 /mode 可切
 *   - Flash 只存 26 小时，每天 02:00 自动归档
 *   - 手动归档加 _manual 后缀，不混淆
 *   - Canvas 平滑曲线
 *   - 网页日志控制台（带开关）
 *   - 采样对齐 :00/:30；插电每次采样刷屏，电池只在整分钟刷屏（省电）
 *   - 日平均引擎：30天滑动窗口（PSRAM）
 *   - 今日气压距平曲线 + 24h趋势；今日极值栏：温/湿/压 最高最低
 *   - AP 按需开启：连上WiFi自动关AP省电降热
 */

#include <Wire.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_SHT31.h>
#include <RTClib.h>
#include <LittleFS.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiUdp.h>      // → V2.1.1-b 标定负载：向网关连发 UDP（纯发送就有射频功耗）
#include "apps/ping/ping_sock.h"   // ← 内置 esp_ping（ICMP）：静态IP 查重用（core 3.3.11 实测可编译链接）
#include <WebServer.h>
#include <HTTPClient.h>   // v2.2 推送：向服务器 POST 样本（只用在这个功能的独立任务里）
#include <time.h>
#include <sys/time.h>
#include <sys/time.h>
#include <SPI.h>
#include <stdarg.h>
#include <ArduinoJson.h>   // ← 安装：搜索 "ArduinoJson by Benoit Blanchon"
#include <Update.h>        // ← 内置，OTA 用的
#include "ui_display.h"   // ← ST7305 横屏 UI 模块（底图已删，中文标签字模 ui_cjk.h）
#include "esp_sleep.h"    // ← deep-sleep / ext0 唤醒
#include <Preferences.h>   // ← NVS，存设备模式(固定/移动)
// ===== BLE 整体搁置（V2.1，设计文档附录 C）=====
//   ENABLE_BLE=1 才编入蓝牙组件，默认 0 不编译（省 flash/RAM/功耗，且消除 NimBLE 生命周期风险）。
//   代码保留不删——远期做 APP 时把 ENABLE_BLE 改为 1 即复活（届时先复核设计文档附录 B.3 的 deinit 铁律）。
#define ENABLE_BLE 0
#if ENABLE_BLE
#include <BLEDevice.h>   // ← 内置 BLE（Bluedroid，零外部依赖）
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#endif
#include "ina230.h"        // ← INA230 电池监测（v2.2：取代 INA226）
#include "pht_crypto.h"     // ← 自带 SHA-256/HMAC-SHA256（设备一机一密签名用）

// ===== USB 枚举检测（v2.2 · BQ24074 EN1/EN2 三档切换）=====
//   机制：USB 枚举成功 → EN1/EN2=(0,1) USB500 档；失败 → (1,0) ISET 档。
//   ⚠️ **靠 USB 供电刷机的那台测试机必须置 0**：它依赖 USB 供电，一旦按枚举结果切档
//      可能掉电/影响刷机；其余设备正常识别并切换。
//   用法：默认 0（安全）。启用请改 1 后编译。
//   依据：硬件核对清单 第五节清单 3（待实装）。
// ===== USB 枚举检测（**已实装** 2026-10-03）=====
//   目的：区分"插在 USB 主机（电脑）"还是"插在充电器"，据此选 BQ24074 输入限流档位。
//   原理：USB 主机每 1ms 发一个 **SOF 帧**；ESP32-S3 的 USB-Serial-JTAG 控制器
//         维护一个 **SOF 帧号计数器**（自增，11 位，绕回 2048）→ 计数在变 = 对端是活的主机。
//   ⛔ **不用 `(bool)Serial`** —— 实测它反映的是主机的 DTR，不是数据流，充电器上可能是 false。
//   寄存器（esp32s3-libs 3.3.9-cn 头文件核对）：
//     DR_REG_USB_SERIAL_JTAG_BASE = 0x60038000
//     +0x24 USB_SERIAL_JTAG_FRAM_NUM_REG（低 11 位 = SOF_FRAME_INDEX）
#define USB_JTAG_BASE        0x60038000UL
#define USB_JTAG_FRAM_NUM    (USB_JTAG_BASE + 0x24UL)
#define USB_JTAG_INT_CLR     (USB_JTAG_BASE + 0x14UL)
#define USB_SOF_FRAME_MASK   0x7FFUL          // 帧号 11 位

//   判定窗口：上电后这段时间内观察帧号变化；取 3 次、每次间隔 220ms
#define ENUM_WINDOW_MS  3000UL

#define ENUM_UNKNOWN 0
#define ENUM_IS_HOST 1
#define ENUM_IS_CHARGER 2

// ===== 【阶段1·安全启动】2026-10-03 =====
//   故障：插着 USB 拔掉电池 → 设备反复 BROWNOUT 起不来。
//   实测证据（用户示波器/万用表）：
//     · 3.3V 轨**一直有电** → 排除"3.3V 掉到 0"
//     · EN1/EN2 启动后**仍无一拉高** → 输入限流恒为 (0,0)=USB100=100mA
//     · 拔电池瞬间 PGOOD **无高脉冲** → 排除 PR1/PGOOD 误动作
//   结论：**100mA 输入限流撑不住系统启动峰值**（WiFi 初始化瞬时 300~500mA）。
//         之前"能启动"是靠电池补峰值；一旦拔掉电池就塌。
//   本阶段措施（最小改动，先定论根因）：
//     ① CE 拉高 = **禁止充电**（不跟系统抢输入电流）
//     ② EN1 抬高 → (EN2,EN1)=(0,1) = **USB500 = 500mA**（USB 规范安全值）
//     ③ PR1 保持低（LDO）
//   ⚠️ 设为 0 可整体回退到"老逻辑"（允许充电 + 不驱动 EN1/EN2）
#define PWR_SAFE_START 1
#include "web_icon.h"      // ← iOS 主屏图标 PNG 字节数组（apple-touch-icon 必须 PNG）

// 电源状态结构体（放在文件顶部：Arduino 会给所有函数生成原型到顶部，
// 若自定义类型定义太靠后会报 “does not name a type” 🍄）
typedef struct {
  bool  powered;     // 是否外部供电 (PGOOD 低)
  bool  charging;    // 是否充电中 (CHG 低)
  bool  onBattery;   // 是否电池供电
  float battVolt;    // 电池电压 (V)；INA230 离线时为 NAN（未知）
  uint8_t battPct;   // 估算电量 (0-100)；INA230 离线时无效(0)
  // ---- INA230 电池监测（V2.1，唯一电压来源；原 IO9 分压方案已删除）----
  bool  inaOK;             // INA230 是否在线（false = 电压/电量均不可用）
  float battCurrent_mA;    // 充放电电流（充电为正、放电为负）
  float battPower_mW;      // 电池端功率
  bool  cvZone;            // 是否进入 CV 区（接近充满）
} PowerState;

// 最近一次读到的电源状态（屏幕与逻辑共用，每次 readAndLog() 里更新）
PowerState g_power = {};


SPIClass sdSPI(HSPI);          // v2.2：SD 走 HSPI（IO5/4/6/7 正是 HSPI 的原生引脚）；屏走默认 SPI(FSPI)

// ==================== Web 日志系统 ====================
#define WEB_LOG_MAX 200
#define WEB_LOG_LEN 256
// 日志环形缓冲 = 200×256 = 51200 字节！放内部 DRAM 会把 dram0_0_seg 撑爆，
//   故搬到 PSRAM（ps_malloc）。类型为「指向 WEB_LOG_LEN 字节数组的指针」，
//   索引写法 webLogBuffer[i] / webLogBuffer[i][j] 与原来完全一致。
char (*webLogBuffer)[WEB_LOG_LEN] = nullptr;
volatile int webLogHead = 0;
volatile int webLogCount = 0;
bool webLoggingEnabled = true;   // ← 开关

// ===== 配网调试日志（写 Flash）=====
//   动机（2026-09-13）：移动+电池走蓝牙时设备没有 HTTP，配网出问题看不到内部状态。
//   做法：一旦触发配网（蓝牙保存 / 点已保存网络 / 长按 IO9）→ 此后所有 webLog 追加到 /cfgdbg.log，
//         按 16KB 滚动；10 分钟后自动停写（省 Flash）。**重启/断电都不丢**；
//         插上电后用浏览器打开 http://<设备IP>/cfgdbg 就能看到全过程（也可电脑远程直接读）。
const char* CFG_DBG_PATH = "/cfgdbg.log";
const size_t CFG_DBG_MAX = 16384;
bool   cfgDbgOn    = false;      // 本次是否在写
bool   cfgDbgBoot  = false;      // 本次开机是否已做过「是否续写」判断
unsigned long cfgDbgUntil = 0;
bool   fsReadyForDbg = false;    // LittleFS 挂载完成（懒判：totalBytes()==0 即未挂载）
UiDisplay uiDisplay;   // 提前定义：updateWireless() 里状态一变就要刷顶栏图标


// 复位原因（写进 Flash 日志：能区分「崩了」和「电压跌落」——这一步很关键）
const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "上电/复位键 (POWERON)";
    case ESP_RST_EXT:       return "外部引脚复位 (EXT)";
    case ESP_RST_SW:        return "软件重启 (SW，含 OTA/ESP.restart)";
    case ESP_RST_PANIC:     return "❌ 崩溃 PANIC（看串口 backtrace）";
    case ESP_RST_INT_WDT:   return "❌ 中断看门狗 INT_WDT";
    case ESP_RST_TASK_WDT:  return "❌ 任务看门狗 TASK_WDT（长时间不喂狗）";
    case ESP_RST_WDT:       return "❌ 其它看门狗";
    case ESP_RST_DEEPSLEEP: return "深睡唤醒 (DEEPSLEEP)";
    case ESP_RST_BROWNOUT:  return "❌ 电压跌落 BROWN-OUT（供电不足！）";
    case ESP_RST_SDIO:      return "SDIO 复位";
    default:                return "未知";
  }
}

void cfgDbgNote(const char* s) {
  File f = LittleFS.open(CFG_DBG_PATH, "a");
  if (!f) return;
  if (f.size() > CFG_DBG_MAX) { f.close(); LittleFS.remove(CFG_DBG_PATH);
    f = LittleFS.open(CFG_DBG_PATH, "w"); if (!f) return; f.print("…（超 16KB，已截断）\n"); }
  f.print(s);
  f.close();
}

// 由 webLog() 调用：会话期间把每条日志落盘
void cfgDbgWrite(const char* s) {
  if (!fsReadyForDbg) return;   // 挂载成功才置位，见 initFS()
  if (!cfgDbgOn) {
    if (cfgDbgBoot) return;                 // 本次开机已判断过（清过就不再续写）
    cfgDbgBoot = true;
    if (!LittleFS.exists(CFG_DBG_PATH)) return;
    cfgDbgOn = true;                        // 文件还在 → 上次调试没读完，续写（含重启后）
    cfgDbgUntil = millis() + 10UL * 60UL * 1000UL;
    cfgDbgNote("\n===== 设备重启，继续记录 =====\n");
    { time_t rt = time(nullptr); char rts[24] = ""; if (rt > 1000000000) { struct tm rtm; localtime_r(&rt, &rtm); strftime(rts, sizeof(rts), "%H:%M:%S", &rtm); }
      cfgDbgNote((String("复位原因: ") + resetReasonStr() + (rts[0] ? (String(" @ ") + rts) : String("")) + "\n").c_str()); }
  }
  if ((int32_t)(millis() - cfgDbgUntil) >= 0) { cfgDbgOn = false; return; }
  cfgDbgNote(s);
}

// 开启一次配网调试日志（每次配网重开一份，便于按时间点对号入座）
void cfgDbgBegin(const char* why) {
  if (!fsReadyForDbg) return;   // 挂载成功才置位，见 initFS()
  LittleFS.remove(CFG_DBG_PATH);
  File f = LittleFS.open(CFG_DBG_PATH, "w");
  if (f) { f.printf("===== 配网调试日志 @ %s =====\n", why); f.close(); }
  cfgDbgOn = true; cfgDbgBoot = true;
  cfgDbgUntil = millis() + 10UL * 60UL * 1000UL;
  webLogln("📝 [调试] 配网日志已写 Flash：%s（浏览器看 http://设备IP/cfgdbg；10 分钟后自动停写）", CFG_DBG_PATH);
}

static void webLog(const char* fmt, ...) {
  char buf[WEB_LOG_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  Serial.print(buf);
  cfgDbgWrite(buf);                 // 配网调试期间把每条日志落盘（未开启时几乎零开销）
  if (!webLoggingEnabled) return;
  // 首次调用时把环形缓冲分配到 PSRAM（51KB，经不起占内部 DRAM）
  if (!webLogBuffer) {
    webLogBuffer = (char (*)[WEB_LOG_LEN])ps_malloc((size_t)WEB_LOG_MAX * WEB_LOG_LEN);
    if (!webLogBuffer) return;   // PSRAM 也不够就只能丢日志，绝不再吃 DRAM
  }
  strncpy(webLogBuffer[webLogHead], buf, WEB_LOG_LEN - 1);
  webLogBuffer[webLogHead][WEB_LOG_LEN - 1] = '\0';
  webLogHead = (webLogHead + 1) % WEB_LOG_MAX;
  if (webLogCount < WEB_LOG_MAX) webLogCount++;
}

static void webLogln(const char* fmt, ...) {
  char buf[WEB_LOG_LEN];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  String s = String(buf) + "\n";
  webLog("%s", s.c_str());
}
// =====================================================

// ==================== WiFi 配置 ====================
const char* WIFI_SSID     = "";            // 出厂默认空：运行时由用户配网后写入（勿硬编码个人 SSID）
// WiFi 密码不硬编码（凭据只存 NVS / 网页配置）
const char* AP_SSID       = "Weather_Station_Pro";
const char* AP_PASSWORD   = "12345678";    // ⚠️ 公开默认值，首次使用请务必修改
const char* AP_IP_STR     = "192.168.5.1";   // 必须与 startAP() 里的 local_IP 保持一致

const IPAddress STA_IP(192, 168, 1, 97);    // 仅作 /wifi 页占位示例；实际地址由「末段 + 学习到的网关/掩码」算出
const IPAddress STA_GW(192, 168, 1, 1);    // 同上
const IPAddress STA_MASK(255, 255, 255, 0); // 学习失败时的兑底掩码
// ===================================================

// ==================== WiFi 配置管理（网页可改，支持多组自动选最强）====================
//  存储：/wifi.json = {"list":[{"ssid","pass","useIP","ipLast","gw","mask","dns"}, ...]}
//  旧格式（"ip"/"gw"/"mask" 整段）开机自动升级：取旧 ip 的末段当 ipLast，其余保留，老配置不丢。
//  静态 IP 策略（V2.1）：用户只填「末段」（默认 200）；网关/掩码/DNS 由设备 **DHCP 连一次后自己学习**
//    并落盘（与手动填写同构）。开机流程：先 DHCP 连上 → 学 gw/mask/dns 落盘 → ping 候选末段查重
//    （被占则 +1，最多 5 次）→ 配静态重连；5 次都被占或静态连不上 → **退回 DHCP** 并标出来。
//  连接策略：启动/重连时扫一遍，**只在已保存的网络里挑 RSSI 最大的那组**。
#define WIFI_MAX_SAVED 5
struct WiFiConfig {
  char ssid[32] = "";
  char password[64] = "";
  bool useIP = false;          // true = 静态 IP（只填末段）；false = 自动 (DHCP)
  uint8_t ipLast = 0;          // 静态 IP 末段；0 = 未填（连接时按 200 处理）
  IPAddress gw, mask, dns;     // 网关 / 掩码 / DNS（DHCP 学习或手动填；全 0 = 未知）
  // 由「网关 & 掩码」的网段基址 + 末段拼出静态地址（未学习到网关时返回 0.0.0.0）
  //   ⚠ 必须按字节(octet)构造：ESP32 的 IPAddress 内部是小端，`base | ipLast` 会把末段错写到第一字节！
  IPAddress staticIp() const {
    if ((uint32_t)gw == 0 || (uint32_t)mask == 0) return IPAddress((uint32_t)0);
    IPAddress base((uint32_t)gw & (uint32_t)mask);
    return IPAddress(base[0], base[1], base[2], (uint8_t)ipLast);
  }
};
WiFiConfig wifiCfg;                        // 当前生效（正在连）的那组
WiFiConfig wifiList[WIFI_MAX_SAVED];       // 已保存的多组
int  wifiCount = 0;                        // 有效组数
int  wifiRssi  = 0;                        // 选中那组的信号强度(dBm)，0 = 未知

// 小文件复制（当前只给配置备份用）
void copyFile(const char* src, const char* dst) {
  File a = LittleFS.open(src, "r");
  if (!a) return;
  File b = LittleFS.open(dst, "w");
  if (!b) { a.close(); return; }
  uint8_t buf[128]; size_t n;
  while ((n = a.read(buf, sizeof(buf))) > 0) b.write(buf, n);
  a.close(); b.close();
}

//  注：JSON 解析用 lambda 写在 loadWiFiConfig() 里——.ino 的自动函数原型会把带自定义类型的
//      自由函数原型插到结构体定义之前，报 “WiFiConfig was not declared”。

void saveWiFiConfig() {
  JsonDocument doc;                             // ArduinoJson 7：动态文档，无需容量
  JsonArray list = doc["list"].to<JsonArray>();
  for (int i = 0; i < wifiCount; i++) {
    JsonObject o = list.add<JsonObject>();
    o["ssid"] = wifiList[i].ssid;
    o["pass"] = wifiList[i].password;
    o["useIP"] = wifiList[i].useIP;
    o["ipLast"] = wifiList[i].ipLast;
    o["gw"] = wifiList[i].gw.toString();
    o["mask"] = wifiList[i].mask.toString();
    o["dns"] = wifiList[i].dns.toString();
  }
  File f = LittleFS.open("/wifi.json", "w");
  if (!f) { webLogln("⚠️ WiFi 配置写入失败"); return; }
  serializeJson(doc, f);
  f.close();
}

void loadWiFiConfig() {
  wifiCount = 0;
  if (!LittleFS.exists("/wifi.json")) return;
  File f = LittleFS.open("/wifi.json", "r");
  if (!f) return;
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  if (err) { webLogln("⚠️ WiFi 配置解析失败，按未配置处理"); return; }
  // 一条记录的解析（lambda：避开 .ino 自动原型对自定义类型的限制）
  auto entryFrom = [](WiFiConfig& c, JsonObject o) {
    strlcpy(c.ssid, o["ssid"] | "", sizeof(c.ssid));
    // 密码键：新格式 "pass"，旧格式 "password"——两个都认（当初升级时只读 pass，把密码丢了）
    const char* pw = o["pass"] | "";
    if (!*pw) pw = o["password"] | "";
    strlcpy(c.password, pw, sizeof(c.password));
    c.useIP = o["useIP"] | false;
    // 新格式：ipLast（末段）。旧格式：整段 "ip" → 取末段当 ipLast（平滑迁移）
    c.ipLast = (uint8_t)(o["ipLast"] | 0);
    if (!c.ipLast) {
      const char* ipOld = o["ip"] | "";
      if (*ipOld) { IPAddress t; if (t.fromString(ipOld)) c.ipLast = t[3]; }
    }
    const char* gw = o["gw"] | "";      if (*gw) c.gw.fromString(gw);
    const char* mk = o["mask"] | "";    if (*mk) c.mask.fromString(mk);
    const char* dn = o["dns"] | "";     if (*dn) c.dns.fromString(dn);
  };
  JsonArray list = doc["list"].as<JsonArray>();
  bool legacy = false;
  if (!list.isNull()) {
    for (JsonObject o : list) {
      if (wifiCount >= WIFI_MAX_SAVED) break;
      if (!o["ssid"]) continue;
      entryFrom(wifiList[wifiCount], o);
      if (strlen(wifiList[wifiCount].ssid)) wifiCount++;
    }
  } else if (!doc["ssid"].isNull()) {      // 旧格式 → 升级成一组
    entryFrom(wifiList[0], doc.as<JsonObject>());
    if (strlen(wifiList[0].ssid)) { wifiCount = 1; legacy = true; }
  }
  if (wifiCount > 0) wifiCfg = wifiList[0];
  if (legacy) {
    copyFile("/wifi.json", "/wifi.old.json");   // 保险：升级前留一份原格式备份
    saveWiFiConfig();
    webLogln("⬆️ WiFi 配置已升级为多组格式（1 组，原文件备份到 /wifi.old.json）");
  }
}

// 增/改一组（同 SSID 视为覆盖）；返回下标，已满返回 -1
//  useIPMode: -1 = 不改动地址方式；0 = 自动(DHCP)；1 = 静态IP（只填末段）
//   之前用 bool，表单没带 useIP 就当成 0 → 静默把静态 IP 关掉，设备一重启就跳 DHCP（实测事故）
//  ipLast: 静态末段；0/越界 = 留空 → 用默认 200（“默认只针对留空”，已填的原样保留）
int wifiUpsert(const char* ssid, const char* pass, int useIPMode, int ipLast) {
  int idx = -1;
  for (int i = 0; i < wifiCount; i++) if (strcmp(wifiList[i].ssid, ssid) == 0) { idx = i; break; }
  bool isNew = (idx < 0);
  if (isNew) {
    if (wifiCount >= WIFI_MAX_SAVED) return -1;
    idx = wifiCount++;
    wifiList[idx].useIP = true;          // 新增组：默认静态（末段留空 → 200）
  }
  WiFiConfig& c = wifiList[idx];
  strlcpy(c.ssid, ssid, sizeof(c.ssid));
  // 密码留空 = “不改密码”（页面不回显已存密码，避免手滑把密码清掉）；新条目留空则当作开放网络
  if (pass && *pass) strlcpy(c.password, pass, sizeof(c.password));
  else if (isNew)    c.password[0] = '\0';
  if (useIPMode >= 0) {                  // 表单/工具显式提交了地址方式 → 才动它（-1 时保留原样）
    c.useIP = (useIPMode == 1);
    if (c.useIP) c.ipLast = (ipLast >= 2 && ipLast <= 254) ? (uint8_t)ipLast : 200;   // 留空 → 默认 200
  }
  saveWiFiConfig();
  wifiSyncCurrent(idx);      // ★ 刷新“当前组”wifiCfg，否则重连还在用旧凭据（曾导致存完必须按 EN 才重连）
  return idx;
}

// 配置变动后刷新“当前组”wifiCfg：优先保持已连上的那个 > 指定下标 > 第一个
// 每处都打日志——自检靠它确认“保存后当前组真的刷新了”（曾经不刷新 → 存完必须按 EN 才重连）
void wifiSyncCurrent(int preferIdx) {
  if (WiFi.status() == WL_CONNECTED) {
    String cur = WiFi.SSID();
    for (int i = 0; i < wifiCount; i++)
      if (cur == wifiList[i].ssid) { wifiCfg = wifiList[i]; webLogln("🔁 当前组已刷新 '%s'（保持已连上的）", wifiCfg.ssid); return; }
  }
  if (preferIdx >= 0 && preferIdx < wifiCount) { wifiCfg = wifiList[preferIdx]; webLogln("🔁 当前组已切到 '%s'", wifiCfg.ssid); return; }
  if (wifiCount > 0) { wifiCfg = wifiList[0]; webLogln("🔁 当前组已刷新为 '%s'", wifiCfg.ssid); return; }
  memset(&wifiCfg, 0, sizeof(wifiCfg));
  webLogln("🔁 已无保存的 WiFi，当前组清空");
}

// 已保存的 WiFi 列表 → JSON（配网页展示/删除；**不回显密码**）
String buildWifiListJson() {
  String s = "{\"count\":" + String(wifiCount) + ",\"max\":" + String(WIFI_MAX_SAVED) +
             ",\"current\":\"" + String(wifiCfg.ssid) + "\",\"list\":[";
  for (int i = 0; i < wifiCount; i++) {
    if (i) s += ",";
    s += "{\"ssid\":\"" + String(wifiList[i].ssid) + "\",\"useIP\":" +
         String(wifiList[i].useIP ? 1 : 0) + ",\"ipLast\":" + String(wifiList[i].ipLast) +
         ",\"gw\":\"" + wifiList[i].gw.toString() + "\",\"mask\":\"" + wifiList[i].mask.toString() + "\"}";
  }
  return s + "]}";
}

// 扫描附近 WiFi → JSON 数组（HTTP /wifi-scan 与蓝牙共用；saved=是否已保存；同名只留最强的）
String buildWifiScanJson() {
  int n = WiFi.scanNetworks();
  String s = "[";
  int added = 0;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;
    int r = WiFi.RSSI(i);
    bool dup = false;
    for (int k = 0; k < i; k++) if (WiFi.SSID(k) == ssid && WiFi.RSSI(k) >= r) { dup = true; break; }
    if (dup) continue;
    bool saved = false;
    for (int k = 0; k < wifiCount; k++) if (ssid == wifiList[k].ssid) saved = true;
    String esc = ssid; esc.replace("\\", "\\\\"); esc.replace("\"", "\\\"");
    if (added++) s += ",";
    s += "{\"ssid\":\"" + esc + "\",\"rssi\":" + String(r) + ",\"saved\":" + (saved ? "true" : "false") + "}";
  }
  WiFi.scanDelete();
  return s + "]";
}

bool wifiRemove(const char* ssid) {
  for (int i = 0; i < wifiCount; i++) {
    if (strcmp(wifiList[i].ssid, ssid) == 0) {
      for (int k = i; k < wifiCount - 1; k++) wifiList[k] = wifiList[k + 1];
      wifiCount--;
      saveWiFiConfig();
      wifiSyncCurrent(-1);
      return true;
    }
  }
  return false;
}

// 扫描 → 在已保存的组里挑信号最强的写进 wifiCfg；返回是否挑到
//   requireInRange=true：**扫不到就返回 false，绝不硬连**（用于启动期，避免白等 10 秒）
bool wifiPickBest(bool requireInRange) {
  if (wifiCount == 0) return false;
  int n = WiFi.scanNetworks();
  int best = -1, bestR = -127;
  for (int i = 0; i < n; i++) {
    int r = WiFi.RSSI(i);
    String s = WiFi.SSID(i);
    for (int k = 0; k < wifiCount; k++)
      if (strlen(wifiList[k].ssid) && s == wifiList[k].ssid && r > bestR) { bestR = r; best = k; }
  }
  WiFi.scanDelete();
  if (best < 0) {
    if (wifiCount == 1) { wifiCfg = wifiList[0]; wifiRssi = 0; return true; }
    // ⚠️ 扫不到任何已保存热点：
    //   requireInRange=true（启动期）→ 返回 false，调用方**直接开 AP**，不再硬连（白等 10s）
    //   否则（运行期重连）→ 沿用上次的 wifiCfg 再试一次
    webLogln("⚠️ 已保存的 %d 组 WiFi 都不在扫描范围内", wifiCount);
    return false;
  }
  wifiCfg = wifiList[best];
  wifiRssi = bestR;
  webLogln("🎯 选中 '%s'（RSSI %d dBm / 共存 %d 组）", wifiCfg.ssid, wifiRssi, wifiCount);
  return true;
}

// 当前 wifiCfg（SSID+密码）的 djb2 哈希：/status 用来自检“保存后当前组是否已刷新”
uint32_t wifiCfgHash() {
  uint32_t h = 5381;
  for (const char* p = wifiCfg.ssid; *p; p++)     h = ((h << 5) + h) + (uint8_t)*p;
  h = ((h << 5) + h) + (uint8_t)'|';
  for (const char* p = wifiCfg.password; *p; p++) h = ((h << 5) + h) + (uint8_t)*p;
  return h;
}
// ===================================================

// ==================== 无线互斥状态机（V2.1：BLE 搁置） ====================
//  规格：设计文档 附录 C（V2.1 无线策略）
//    | 设备状态        | 无线        | 说明                                        |
//    | 固定(0)/插电     | WiFi STA    | 自动连已存网络；连不上 → AP 兜底               |
//    | 移动(1) + 电池   | 全关(WL_OFF) | 默认不开（史级省电）；按 IO9 按需开，超时自动关   |
//    | 无已存/连不上     | AP 热点     | IO9 按需开热点，手机连上 → 真网页配网          |
//    另：不插电绝不自动开 WiFi（否则实体 WiFi 开关无意义）；实体按钮 IO9 → 开无线。
//  注：BLE 整体搁置（ENABLE_BLE 默认 0，代码保留），仅保留 WiFi STA/AP 两条路径。
//      下面 BLE 相关代码待 #if ENABLE_BLE 包裹（待办：README 十一.5）。
#define WL_OFF  0
#define WL_WIFI 1
#define WL_BLE  2

#define PIN_WIFI_BTN  21                       // 实体按钮（v2.2：IO9 -> IO21；交互改短按）
#define BLE_DEV_NAME  "PHT_2_X"
// 前向声明：.ino 的自动函数原型在遇到 class 定义后可能失效，本段用到的后置定义全部显式声明
//   （与文件顶部 “extern uint8_t deviceMode;” 同理）
//   ⚠️ V2.1.1-b 实测：自动原型确实会漏（raw string 字面量多时会错乱）→ 标定相关函数全部显式声明
extern uint8_t deviceMode;
extern bool apEnabled;
extern unsigned long apClosedMs;
extern bool chargeForceSlow;   // V2.1.1-a：移动+插电时的「临时慢充」（定义在下方充电策略段）
// ---- V2.1.1-b 电量校准（实现见文件后半部）----
bool     calAbortPressed();
void     calScreen(const char* l1, const char* l2);
void     calScreenProgress();
void     calUdpBurst();
void     calCpuBurn(uint32_t ms);
bool     calInaBegin(bool calMode);
void     calReadIna(float& v, float& i);
void     setCalFlag(bool on);
uint8_t  getCalFlag();
void     calNowStr(char* out, size_t n);
void     calCsvHeader();
void     calCsvRow(uint32_t tLoaded, uint32_t dt);
void     calSaveTable();
void     calLoadTable();
uint8_t  calPctFromVolt(float v);
bool     calBuildTable(bool partial);
uint32_t calFullHeldMs();
// 直连工具链构建（_build/）没有 ctags 自动原型，这两条必须自己写：
//   PowerState 的定义在后面，但 applyChargeStrategy() 的调用点更靠前。
void     applyChargeStrategy(const PowerState& ps);
void     serviceBattAnchor();
// V2.1.1 低电三档跟本机表走（2026-09-29）：开机载表后刷新 battArchiveV/battStopV/battSleepV
void     updateBattThresholds();
// V2.1.1 Bug② 修复：今日统计抽成函数（实时采样与开机回灌共用）
void     feedTodaySample(uint32_t t, float tC, float h, float p);
int      backfillTodayFromLog();
bool     calStart(bool dry, String& msg);
void     calBeginDischarge();
void     calAbort(const char* why, bool tryBuild = false);
void     serviceCal();
void     serviceTopBar();   // V2.1.1-c 顶栏快通道
String   calStatusJson();
String   calReportJson();
String   calPageHtml();
// 密钥 / 推送地址配置页（2026-10-05）
String   keyPageHtml();
void     calBootCheck();
void startAP();
void stopAP();
PowerState readPowerState();
void _fallbackToRtcTime();
bool syncTimeFromNTPBlocking(unsigned long maxMs);
void ntpStart();
// 归档/缓冲相关（定义在文件后半部；enterDeepSleepIfNeeded 等早期函数会先用到）
void     archivePump();
void     checkAndArchive();
void     loadBufferFromFlash();
void serviceNtpSync();
void serviceNtpDailyCheck();
bool wifiPickBest(bool requireInRange);
// 只读数据的 JSON builder（HTTP 与 BLE 共用，定义在文件后半部）
String buildStatusJson(bool overBLE);
String buildHistoryJson(int page, int count, bool applyFilter);
String buildDailyAvgJson();
String buildAnomalyJson();
String buildTodayStatsJson();
#if ENABLE_BLE
static const char* BLE_SVC_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const char* BLE_RX_UUID  = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";   // 手机 → 设备（请求）
static const char* BLE_TX_UUID  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";   // 设备 → 手机（响应）
#endif

uint8_t  curWireless = WL_OFF;      // 当前在用的无线
bool     bleOn = false;             // BLE 协议栈已 init 并在广播（ENABLE_BLE=0 时恒为 false，/status 仍回报）
bool     bleLink = false;           // 有手机连着（同上）
bool     wifiForced = false;        // 实体按钮强制开 WiFi
unsigned long wifiRecoverUntil = 0;
time_t   nextSampleTime   = 0;      // 下一个采样点的墙上时间（采样与浅睡共用锚点；置 0 = 待重算）
// ===== NTP 重试（2026-10-02）=====
//   背景：syncTimeFromNTP() 原先**只在 setup() 调一次**、且窗口只有 10 秒；
//        若开机那 10 秒恰好没外网，整个会话内再也不会重试（只能靠 DS3231 兜底）。
//   策略：**只在「插电 + WiFi 已连 + 距上次 NTP 成功 ≥ 3 天」时**才试一次，且每次开机最多试一次。
//   ⛔ 绝不出现"电池供电还去连 WiFi 同步 NTP"的逻辑（用户明确要求）。
time_t   ntpLastOkAt      = 0;      // 上次 NTP 成功的时间（0 = 从未成功过）
// NTP 定期重同步（2026-10-03 改）：**每日 02:00 检查一次**，超过 NTP_RESYNC_DAYS 天没成功就同步。
//   理由：RTC(DS3231) 与片上时钟都会漂；本设备理想环境下**一年都可能不重启**，
//         只靠"开机时同步一次"会让时间越走越歪。
//   原 ntpTriedThisBoot（每次开机只重试一次）已由每日定时取代。
int      ntpDailyLastDay  = -1;      // 上次做每日检查的"日"（防止同日重复）
bool     ntpDailyPending  = false;   // RTC 掉电且当前无网 → 挂起，等有网再补一次
const long NTP_RESYNC_DAYS = 3L;     // 超过这么多天没成功 → 重新同步
// 【兜底】长按 IO9 强开热点模式：到此刻为止拒绝任何 STA 重连（0 = 正常）
int      wlForce = 0;               // 联调/调试：0=自动 1=强制WiFi 2=强制蓝牙（/wireless?force=）
unsigned long wlForceUntil = 0;     // 强制到期时刻（自愈：3 分钟后自动回到场景判定，避免把设备锁在蓝牙）
unsigned long lastWirelessCheck = 0;
#if ENABLE_BLE
String   bleRxBuf;
BLECharacteristic* bleTx = nullptr;
bool bleStopping = false;       // 正在主动关闭蓝牙：期间 onDisconnect 不要再重启广播
#endif

// ===== 「连网会话」：配网/按钮触发的临时 WiFi（设计文档 · 无线切换与配网流程）=====
//   移动+电池平时走蓝牙（省电）；用户配网保存 / 点已保存网络 / 长按按钮 → 临时切 WiFi 用网页。
//   会话期间只要有网页请求就续期（网页每 3 秒轮询 /status）；断流 WIFI_SESSION_MS 自动回蓝牙。
//   防误触：① 插电/固定时按钮完全无效（那时本来就该 WiFi/AP）
//          ② 会话到期自动回蓝牙 —— 丢包里蹭到也不会傻等把电耗光
//   ⚠️ v2.2（2026-10-02）：交互由**长按改短按**（依据 设计文档 V2.2 · 软件 · 交互和可视化 1）。
//      短按上限设 1200ms：既覆盖正常点按，又把 1.2~5s 的"犹豫按压"排除在外，
//      避免"想长按结果按了一半就松手"被误判成常规动作。
const unsigned long WIFI_SESSION_MS   = 60000;
const unsigned long WIFI_BTN_SHORT_MS = 1200;    // ≤ 此值 = 短按（下限见 WIFI_BTN_DEBOUNCE_MS）
const unsigned long WIFI_BTN_DEBOUNCE_MS = 30;   // < 此值视为抖动，忽略
// 【兜底】长按到这个时长（松手才触发）→ 强制开热点 + 断开 WiFi，用于设备失联时救援
//   失控场景：设备连上了某个网、却因「静态 IP/网关学错网段」而彻底够不着（实测踩到，见 开发日志 §十二）
const unsigned long WIFI_BTN_RECOVER_MS = 5000;   // 长按（松手触发）= 救援模式，**保持长按以防误触**
const unsigned long AP_SESSION_MS     = 600000;  // AP 配网热点：无人访问满 10 分钟自动关（省电，移动+电池尤其重要）
unsigned long wifiSessionUntil = 0;    // 会话到期时刻（0 = 无会话）
unsigned long apSessionUntil   = 0;    // AP 配网热点到期时刻（0 = 无超时/未启用；仅按需开启的 AP 用）
unsigned long bleHoldUntil     = 0;    // 切 WiFi 时「缓关蓝牙」到此刻（先把 SSID/IP 推送给网页）
bool     wifiBtnArmed    = false;      // 已识别按钮长按，等 loop 执行
String   pendingSSID;                  // 待连接的网络
unsigned long pendingAt   = 0;         // 何时开始连（留 400ms 把 BLE 回包发出去）
bool     webSeenSinceConnect = false;  // 本次连网后是否收到过网页请求
bool     toastDismissReq = false;      // 收到网页请求 → 请求收横幅（真正的重绘放到主循环做）

// ---- 诊断环：最近 6 条关键日志（只读，供蓝牙网页拉取）----
//   为什么需要：移动+电池走蓝牙时设备没有 HTTP，出问题时看不到内部状态；
//   页面发完请求若 12 秒没结果，就拉 `/diag` 把这几行显示出来。
String diagLines[6];
int    diagHead = 0;
void diag(const String& s) {
  diagLines[diagHead] = s;
  diagHead = (diagHead + 1) % 6;
  webLogln("🧪 %s", s.c_str());
}
String buildDiagJson() {
  String s = "{\"lines\":[";
  for (int i = 0; i < 6; i++) {
    int k = (diagHead + i) % 6;
    if (!diagLines[k].length()) continue;
    if (s.length() > 12) s += ",";
    s += "\"" + diagLines[k] + "\"";
  }
  return s + "]}";
}

// 网页被访问：给会话续期；并作为「用户真的进来了」的确认（立刻收横幅，别傻等）
//   ⚠ 定义放在 uiDisplay 对象之后（serviceWifiSession 前）—— 这里先声明，因为路由在 setup() 里就要用它
void noteWebActivity();

// 开始一次连网会话（配网保存 / 点已保存网络 / 长按按钮 都走这里）
//   ⚠ 真正的连接在 loop 末尾做（会阻塞几秒）：这里只登记，先把 BLE 回包发出去
void beginWifiSession(const String& ssid) {
  wlForce = 0; wlForceUntil = 0;      // 用户的配网意图优先：清掉 /wireless?force= 联调口留下的强制标志
  pendingSSID = ssid;
  pendingAt   = millis() + 400;
  cfgDbgBegin("配网：保存或点已保存网络");
  diag("登记连接 " + ssid + "，模式=" + String(curWireless) + " (1=WiFi 2=蓝牙)");
  webLogln("📶 准备连接 WiFi: %s …", ssid.c_str());
}

// 按设计文档表格判定「现在该用哪种无线」
uint8_t wantWireless() {
  if (wlForce && wlForceUntil && millis() > wlForceUntil) {   // 强制到期 → 自动回场景判定
    webLogln("🔧 无线强制模式到期，恢复自动");
    wlForce = 0; wlForceUntil = 0;
  }
  if (wlForce == 1) return WL_WIFI;
#if ENABLE_BLE
  if (wlForce == 2) return WL_BLE;
#endif
  // 【兜底·最高优先】长按 IO9 进入的「热点救援模式」：期间一律 WiFi 且不重连 STA
  //   ⚠ 必须排在 wlForce 之后、其余判定之前：它是逃生门，不能被任何场景判定顶掉
  if (wifiRecoverUntil && (int32_t)(millis() - wifiRecoverUntil) < 0) return WL_WIFI;
  // 连网会话（配网/按钮触发）：临时 WiFi，到期自动回蓝牙
  //   ⚠ 必须排在 wlForce / wifiForced 之前：否则被“强制蓝牙”锁住时会默默忽略配网请求
  if (wifiSessionUntil && (int32_t)(millis() - wifiSessionUntil) < 0) return WL_WIFI;
  if (wifiForced)   return WL_WIFI;          // 实体按钮：强制 WiFi + 顶掉蓝牙
  // AP 配网热点（IO9 按需开启，带超时）：热点期间必须保持 WiFi 射频（否则会被下面「移动+电池→全关」误关）
  if (apEnabled && apSessionUntil && (int32_t)(millis() - apSessionUntil) < 0) return WL_WIFI;
  if (deviceMode == 1) {                     // 移动模式
    PowerState ps = readPowerState();
    if (ps.onBattery) return WL_OFF;         // V2.1：移动+电池 → 无线全关（省电），按 IO9 临时开启
  }
  return WL_WIFI;                            // 固定 / 插电：一律 WiFi（连不上再 AP 兜底）
}

// ---- 关 WiFi（含 AP）----
void wifiDown() {
  if (apEnabled) { WiFi.softAPdisconnect(true); apEnabled = false; apClosedMs = 0; }
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  webLogln("📴 WiFi 已关闭（让位给蓝牙）");
}

// ---- ICMP 探测：ping 一下某地址；返回 true = 有回应（说明该地址**已被占用**）----
//   用途：静态 IP 查重（候选末段被占 → +1 再试）。ESP32 查不到路由器的客户端列表，只能自己发包探。
static volatile bool pingGotReply = false;
static void pingOnSuccess(esp_ping_handle_t hdl, void* args) { pingGotReply = true; }

bool pingHost(const IPAddress& ip, uint32_t timeoutMs) {
  pingGotReply = false;
  esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
  cfg.count       = 1;
  cfg.interval_ms = 100;
  cfg.timeout_ms  = timeoutMs;
  cfg.data_size   = 1;
  cfg.target_addr.type = ESP_IPADDR_TYPE_V4;
  cfg.target_addr.u_addr.ip4.addr = (uint32_t)ip;
  esp_ping_callbacks_t cbs = {};
  cbs.on_ping_success = pingOnSuccess;
  esp_ping_handle_t hdl = nullptr;
  if (esp_ping_new_session(&cfg, &cbs, &hdl) != ESP_OK || !hdl) { webLogln("⚠️ ping 会话创建失败"); return false; }
  esp_ping_start(hdl);
  unsigned long t0 = millis();
  while (!pingGotReply && (millis() - t0) < (timeoutMs + 400)) delay(10);
  esp_ping_stop(hdl);
  esp_ping_delete_session(hdl);
  return pingGotReply;
}

// ---- 连接已保存的网络（静态 IP 策略，设计文档 附录 C.3）----
//   allowAP=false：失败只报错、**绝不开热点**；true：失败 → 开 AP 兜底
//   useIP=true 时：① 先 DHCP 连一次（学 gw/mask/dns 并落盘）② ping 候选末段查重（被占 +1，最多 5 个）
//                  ③ 配静态重连 ④ 都不行 → 退回 DHCP，并把「实际用的地址」报出来
//   wifiStaticFallback = true 表示「本该静态但退回了 DHCP」——调用方据此在屏幕上标明 WiFi DHCP
bool wifiStaticFallback = false;
bool wifiRelearnProtect = false;   // true = 正在"重学后重试"，防止网关自检再次清参数互相打架
// 连上 WiFi 后的横幅标题：正常「WiFi OK」；本该静态却退回 DHCP 时标「WiFi DHCP」
//   （用户要求：退回 DHCP 必须在屏幕上标出来，不然不知道连在哪）
const char* wifiOkTitle() { return wifiStaticFallback ? "WiFi DHCP" : "WiFi OK"; }

bool wifiConnectSaved(bool allowAP) {
  wifiStaticFallback = false;
  if (strlen(wifiCfg.ssid) == 0) { if (allowAP) { webLogln("📶 未配网 → 直接开 AP"); startAP(); } return false; }
  diag(String("begin ‘") + wifiCfg.ssid + "’ 密码长" + String(strlen(wifiCfg.password)) +
       " 静态" + String(wifiCfg.useIP ? "Y" : "N") + " 末段" + String(wifiCfg.ipLast) + " 模式" + String(curWireless));
  WiFi.mode(WIFI_STA);

  auto waitConnected = [](int steps) -> bool {
    for (int i = 0; i < steps && WiFi.status() != WL_CONNECTED; i++) delay(500);
    return WiFi.status() == WL_CONNECTED;
  };
  auto bail = [&](const char* why) -> bool {
    webLogln("❌ WiFi 连接失败: %s（%s）", wifiCfg.ssid, why);
    if (allowAP) { webLogln("⚠️ → 开 AP 兜底"); diag(String("连不上 ‘") + wifiCfg.ssid + "’(" + why + ") → 开 AP"); startAP(); }
    else diag(String("连不上 ‘") + wifiCfg.ssid + "’(" + why + ")，不开热点");
    return false;
  };

  if (!wifiCfg.useIP) {
    // ---------- 纯 DHCP ----------
    WiFi.config(INADDR_NONE, INADDR_NONE);          // 清掉可能残留的静态配置，确保走 DHCP
    WiFi.begin(wifiCfg.ssid, wifiCfg.password);
    if (!waitConnected(20)) return bail("DHCP 超时");
    webLogln("✅ WiFi 已连接(DHCP): %s (%s)", wifiCfg.ssid, WiFi.localIP().toString().c_str());
    diag(String("连上了 ‘") + wifiCfg.ssid + "’ @ " + WiFi.localIP().toString() + "（DHCP）");
    return checkGatewayReachable(WiFi.localIP().toString(), "");
  }

  // ---------- 静态：① 需要时才 DHCP 学一次网络参数（2026-10-03 改）----------
  //   ⚠️ 用户指出：网关/掩码/DNS **学过就该缓存**（谁家网关天天变？），
  //      原来每次启动都无条件重学 → 白等一个 DHCP 往返（1.7s，网关慢时最多 10s）+ 每次写盘。
  //   现在：已学过 → 直接用缓存；若之后配静态/网关自检失败 → 再学一次兜底（自愈换路由器场景）。
  bool gwKnown = ((uint32_t)wifiCfg.gw != 0) && ((uint32_t)wifiCfg.mask != 0);
  String dhcpIp = "-";
  auto doLearn = [&]() -> bool {
    WiFi.config(INADDR_NONE, INADDR_NONE);
    WiFi.begin(wifiCfg.ssid, wifiCfg.password);
    if (!waitConnected(20)) return false;
    dhcpIp = WiFi.localIP().toString();
    wifiCfg.gw   = WiFi.gatewayIP();
    wifiCfg.mask = WiFi.subnetMask();
    wifiCfg.dns  = WiFi.dnsIP();
    if ((uint32_t)wifiCfg.mask == 0) wifiCfg.mask = STA_MASK;
    if ((uint32_t)wifiCfg.dns  == 0) wifiCfg.dns  = wifiCfg.gw;
    // ★ 2026-10-03 关键修复：**必须回写 wifiList 再存盘**。
    //   wifiCfg 是独立对象（由 wifiList 拷贝而来），只写它的话落盘仍是空值
    //   → 下次开机又判"没学过" → 无限重复 DHCP 学习（用户实测：每次启动都白等一个往返）。
    bool wrote = false;
    for (int i = 0; i < wifiCount; i++) {
      if (strcmp(wifiList[i].ssid, wifiCfg.ssid) == 0) {
        wifiList[i].gw = wifiCfg.gw; wifiList[i].mask = wifiCfg.mask; wifiList[i].dns = wifiCfg.dns;
        wrote = true; break;
      }
    }
    if (!wrote) webLogln("⚠️ 学习成果回写失败：wifiList 里找不到 '%s'", wifiCfg.ssid);
    saveWiFiConfig();
    return true;
  };
  if (!gwKnown) {
    webLogln("📥 首次配网 → DHCP 学习网络参数");
    if (!doLearn()) return bail("DHCP 学习阶段超时");
  } else {
    webLog("📥 使用已学习的网络参数（跳过 DHCP 学习）: 网关 %s 掩码 %s DNS %s\n",
           wifiCfg.gw.toString().c_str(), wifiCfg.mask.toString().c_str(), wifiCfg.dns.toString().c_str());
  }
  // ⚠ 按字节构造基址：IPAddress 内部小端，搞错字节序会得到 227.168.31.0 这种鬼地址（实测踩到）
  IPAddress base((uint32_t)wifiCfg.gw & (uint32_t)wifiCfg.mask);
  if (!gwKnown) {   // 只有**本次真的学了**才打印这两句（否则与"跳过"日志自相矛盾）
    webLogln("📥 已学习网络参数: 网关 %s 掩码 %s DNS %s", wifiCfg.gw.toString().c_str(),
             wifiCfg.mask.toString().c_str(), wifiCfg.dns.toString().c_str());
    diag("DHCP 学习: gw=" + wifiCfg.gw.toString() + " mask=" + wifiCfg.mask.toString());
  }

  // ---------- ② 查重：从末段(默认200)起，最多 5 个，ping 有回应 = 被占 ----------
  int start = wifiCfg.ipLast ? wifiCfg.ipLast : 200;
  int gwLast = wifiCfg.gw[3];                    // 网关末段（用字节取，别用位运算）
  int chosen = -1;
  for (int k = 0; k < 5; k++) {
    int cand = start + k;
    if (cand < 2 || cand > 254) break;
    if (cand == gwLast) { webLogln("⏭️ .%d 与网关冲突 → 跳过", cand); continue; }
    IPAddress cip(base[0], base[1], base[2], (uint8_t)cand);
    if (!pingHost(cip, 600)) { chosen = cand; webLogln("✅ .%d (%s) ping 不通 → 用作静态地址", cand, cip.toString().c_str()); break; }
    webLogln("⚠️ .%d (%s) 已被占用（ping 有回应）→ 试下一个", cand, cip.toString().c_str());
  }
  if (chosen < 0) {
    webLogln("⚠️ 候选末段均不可用（被占/越界）→ 退回 DHCP（实际地址 %s）", dhcpIp.c_str());
    wifiStaticFallback = true;
    return true;                                        // 已以 DHCP 连着，直接用
  }

  // ---------- ③ 配静态并重连（失败时**重学一次参数再试**）----------
  //   ★ 2026-10-03 用户要求：网关/掩码/DNS 用缓存配静态失败 → 重学一次再配静态，
  //     而不是像旧逻辑那样直接放弃静态、永久退回 DHCP。
  const bool useIPSaved = wifiCfg.useIP;       // 失败时用来恢复静态偏好

  auto applyStatic = [&](IPAddress useGw, IPAddress useMask, IPAddress useDns) -> bool {
    IPAddress sip(base[0], base[1], base[2], (uint8_t)chosen);
    WiFi.config(sip, useGw, useMask, useDns);
    WiFi.disconnect(false, false);
    delay(200);
    WiFi.begin(wifiCfg.ssid, wifiCfg.password);
    if (!waitConnected(20)) return false;
    wifiCfg.ipLast = (uint8_t)chosen; saveWiFiConfig();
    webLogln("✅ 静态 IP 生效: %s（DHCP 曾拿 %s）", sip.toString().c_str(), dhcpIp.c_str());
    diag(String("静态生效 ‘") + wifiCfg.ssid + "’ @ " + sip.toString());
    return checkGatewayReachable(sip.toString(), dhcpIp);
  };

  if (applyStatic(wifiCfg.gw, wifiCfg.mask, wifiCfg.dns)) return true;

  // ---------- ③b 重学兜底（仅当本来用的是"缓存的"参数时才有意义）----------
  if (gwKnown) {
    webLogln("🩹 用已学习的参数配静态失败 → 重学网络参数后再试一次（保留静态偏好）");
    // 先把静态偏好恢复回来：checkGatewayReachable 的旧自愈分支可能把它改成 DHCP
    bool wrote = false;
    for (int i = 0; i < wifiCount; i++) {
      if (strcmp(wifiList[i].ssid, wifiCfg.ssid) == 0) {
        wifiList[i].useIP = true;
        wifiList[i].gw = IPAddress(0, 0, 0, 0);
        wifiList[i].mask = IPAddress(0, 0, 0, 0);
        wifiList[i].dns = IPAddress(0, 0, 0, 0);
        wrote = true; break;
      }
    }
    if (wrote) saveWiFiConfig();
    wifiCfg.useIP = true;
    wifiCfg.gw = wifiCfg.mask = wifiCfg.dns = IPAddress(0, 0, 0, 0);

    if (doLearn()) {
      wifiRelearnProtect = true;               // 自检失败时不要再清参数（交给外层兜底）
      IPAddress base2((uint32_t)wifiCfg.gw & (uint32_t)wifiCfg.mask);
      for (int k = 0; k < 5; k++) {
        int cand2 = start + k;
        if (cand2 < 2 || cand2 > 254) break;
        if (cand2 == gwLast) continue;
        IPAddress cip2(base2[0], base2[1], base2[2], (uint8_t)cand2);
        if (!pingHost(cip2, 600)) { chosen = cand2; break; }
      }
      if (chosen > 0) {
        IPAddress base3((uint32_t)wifiCfg.gw & (uint32_t)wifiCfg.mask);
        base = base3;
        bool ok2 = applyStatic(wifiCfg.gw, wifiCfg.mask, wifiCfg.dns);
        wifiRelearnProtect = false;
        if (ok2) return true;
      } else {
        wifiRelearnProtect = false;
      }
    }
    // 重学也没救回来 → 恢复用户原本的静态偏好，交给下面的 DHCP 兜底（本次会话）
    wifiCfg.useIP = true;
    for (int i = 0; i < wifiCount; i++)
      if (strcmp(wifiList[i].ssid, wifiCfg.ssid) == 0) { wifiList[i].useIP = useIPSaved; break; }
    saveWiFiConfig();
  }

  // ---------- ④ 静态连不上 → 退回 DHCP ----------
  webLogln("⚠️ 静态地址（末段 .%d）连不上 → 退回 DHCP", (int)wifiCfg.ipLast);
  WiFi.config(INADDR_NONE, INADDR_NONE);
  WiFi.disconnect();
  delay(200);
  WiFi.begin(wifiCfg.ssid, wifiCfg.password);
  if (waitConnected(20)) {
    wifiStaticFallback = true;
    webLogln("✅ 已退回 DHCP：实际地址 %s", WiFi.localIP().toString().c_str());
    diag(String("静态失败 → 退回 DHCP @ ") + WiFi.localIP().toString());
    return true;
  }
  return bail("静态与 DHCP 均失败");
}

// ---- 【兜底】网关自检：连上后 ping 一次网关，确认「地址/网关真在同一个网段」----
//   病根（实测踩到）：网络参数（gw/mask/dns）是「DHCP 学一次就落盘」的。
//     若某次连网时网络处于异常状态（例：CPE 故障期自己发了另一个网段的 DHCP），
//     学到的那套参数就被存下来；等网络恢复正常，设备仍按旧网段配静态 IP + 指旧网关
//     → WiFi 显示"已连接"，但它上不了网、PC 也够不着它 → **彻底失联，只能拔电**（见 开发日志 §十二）
//   对策：连上后验一次网关；不通就把学来的 gw/mask/dns 清掉并改用 DHCP 重连（自愈）
//   代价：每次连网多一次 ping（600ms 超时），只影响开机那几秒
bool checkGatewayReachable(const String& myIp, const String& dhcpPrevIp) {
  IPAddress gw = WiFi.gatewayIP();
  if ((uint32_t)gw == 0 || (uint32_t)gw == 0xFFFFFFFF) {
    webLogln("⚠️ 网关未知（%s）→ 跳过网关自检", myIp.c_str());
    return true;                                     // 拿不到网关就没法判，按成功处理
  }
  bool inSameSubnet = true;
  IPAddress mask = WiFi.subnetMask();
  if ((uint32_t)mask != 0) {
    // 按字节比：我的地址与网关是否同网段（IPAddress 内部小端，别用位运算）
    for (int i = 0; i < 4; i++) {
      if (((uint8_t)WiFi.localIP()[i] & (uint8_t)mask[i]) != ((uint8_t)gw[i] & (uint8_t)mask[i])) { inSameSubnet = false; break; }
    }
  }
  if (!inSameSubnet) {
    webLogln("⚠️ 本机 %s 与网关 %s 不在同一网段（掩码 %s）→ 判定参数错，走自愈",
             myIp.c_str(), gw.toString().c_str(), mask.toString().c_str());
  } else if (pingHost(gw, 600)) {
    webLogln("✅ 网关自检通过：%s 可达（本机 %s）", gw.toString().c_str(), myIp.c_str());
    return true;                                     // 一切正常，保持原配置
  } else {
    webLogln("⚠️ 网关 %s ping 不通（本机 %s）→ 判定参数错，走自愈", gw.toString().c_str(), myIp.c_str());
  }

  // ---- 自愈分支 ----
  //   ★ 2026-10-03：**重学保护期内直接失败返回**，由调用方（wifiConnectSaved）去重学后再试。
  //      否则这里会清掉参数 + 把 useIP 改成 DHCP，与新逻辑互相打架。
  if (wifiRelearnProtect) return false;

  // ---- 自愈：清掉学来的网络参数 + 退回 DHCP 重连（保留旧逻辑作为最终兜底）----
  webLogln("🩹 自愈：清除学到的网关/掩码/DNS，改用 DHCP 重新获取（旧值 gw=%s mask=%s dns=%s）",
           wifiCfg.gw.toString().c_str(), wifiCfg.mask.toString().c_str(), wifiCfg.dns.toString().c_str());
  diag("网关自检不过 → 清网络参数 + 退 DHCP 自愈");
  wifiCfg.gw = IPAddress(0, 0, 0, 0);
  wifiCfg.mask = IPAddress(0, 0, 0, 0);
  wifiCfg.dns = IPAddress(0, 0, 0, 0);
  if (wifiCfg.useIP) { wifiCfg.useIP = false; webLogln("🩹 地址方式已由「静态」改为「自动(DHCP)」（可重配静态）"); }
  saveWiFiConfig();

  wifiStaticFallback = true;
  WiFi.config(INADDR_NONE, INADDR_NONE);
  WiFi.disconnect();
  delay(200);
  WiFi.begin(wifiCfg.ssid, wifiCfg.password);
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) delay(500);
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress gw2 = WiFi.gatewayIP();
    bool ok2 = ((uint32_t)gw2 != 0) && pingHost(gw2, 600);
    webLogln("🩹 自愈结果：DHCP 地址 %s 网关 %s → %s",
             WiFi.localIP().toString().c_str(), gw2.toString().c_str(), ok2 ? "可用 ✅" : "仍不可用 ⚠️（可长按 IO9 ≥5s 进 AP 救援）");
    uiDisplay.showToast(ok2 ? UI_TOAST_OK : UI_TOAST_FAIL, wifiOkTitle(),
                        wifiCfg.ssid, WiFi.localIP().toString().c_str(), 20000);
    return ok2;
  }
  webLogln("❌ 自愈失败：退回 DHCP 也没连上 → 建议长按 IO9 ≥5s 进 AP 救援模式重新配网");
  uiDisplay.showToast(UI_TOAST_FAIL, "WiFi FAIL", wifiCfg.ssid, "长按 IO9 ≥5s 救援", 15000);
  return false;
}

// ---- 开 WiFi（开机 / 固定用）：多组先挑最强，连不上 AP 兜底 ----
void wifiUp() {
  WiFi.mode(WIFI_STA);
  if (wifiCount > 1) wifiPickBest(false);
  wifiConnectSaved(true);
}

#if ENABLE_BLE
// ===================== BLE 收发：全部在 loop() 里做 =====================
// 教训（2026-09-13）：蓝牙写回调跑在协议栈任务上（栈约 3KB），绝不能在回调里跑大栈函数
// （/history 的解析数组就 4.8KB）——否则栈溢出 panic、设备重启。回调只拷贝，处理与发送都交给 loop()。
static char blePendBuf[512];        // 待处理请求（回调里只拷这点东西；配网 URL 可达 300+ 字节）
volatile bool blePend = false;
static String bleTxBuf;             // 待发送内容（loop 里分片发出）
static size_t bleTxPos = 0;
static bool bleTxActive = false;

// 待发送队列：避免「后来的响应把还没发完的推送顶掉」
//   实测踩到：配网成功后推 {event:wifi-connected}，被紧随其后的 /status 响应覆盖 → 手机收不到。
static const int BLE_TXQ = 4;
static String bleTxQ[BLE_TXQ];
static int bleTxQHead = 0, bleTxQTail = 0, bleTxQN = 0;

void bleSendLine(const String& s) {
  if (!bleLink || !bleTx) return;
  if (bleTxQN >= BLE_TXQ) {                 // 队列满：丢最旧的（保证最新的状态能出去）
    bleTxQHead = (bleTxQHead + 1) % BLE_TXQ; bleTxQN--;
  }
  bleTxQ[bleTxQTail] = s;
  bleTxQTail = (bleTxQTail + 1) % BLE_TXQ; bleTxQN++;
  if (!bleTxActive) {                       // 空闲则立即开始发
    bleTxBuf = bleTxQ[bleTxQHead];
    bleTxQHead = (bleTxQHead + 1) % BLE_TXQ; bleTxQN--;
    bleTxPos = 0; bleTxActive = true;
  }
}

// 在 loop() 里推进发送：每轮最多 4 片 × 180 字节，片间 4ms（几千字节约几百毫秒）
void bleTxPump() {
  if (!bleTxActive || !bleLink || !bleTx) return;
  for (int k = 0; k < 4; k++) {
    if (bleTxPos >= bleTxBuf.length()) {        // 收尾：单独一个 '\n' 作为帧结束标记
      bleTx->setValue((uint8_t*)"\n", 1);
      bleTx->notify();
      bleTxActive = false; bleTxBuf = ""; bleTxPos = 0;
      if (bleTxQN == 0) return;
      bleTxBuf = bleTxQ[bleTxQHead];            // 队列里还有 → 接着发（不丢帧）
      bleTxQHead = (bleTxQHead + 1) % BLE_TXQ; bleTxQN--;
      bleTxPos = 0; bleTxActive = true;
    }
    size_t n = bleTxBuf.length() - bleTxPos;
    if (n > 180) n = 180;
    bleTx->setValue((uint8_t*)bleTxBuf.c_str() + bleTxPos, n);
    bleTx->notify();
    bleTxPos += n;
    delay(4);
  }
}

// ---- BLE 写回调（协议栈任务，栈小）：只把请求拷走，处理交给 loop() ----
void bleHandleRequest(String req) {
  req.trim();
  if (req.startsWith("GET ")) req = req.substring(4);
  req.toCharArray(blePendBuf, sizeof(blePendBuf));
  blePend = true;
}

// 从 "?a=1&b=2" 里取整型参数
int bleQInt(const String& qs, const char* key, int def) {
  String k = String(key) + "=";
  int p = qs.indexOf(k);
  if (p < 0) return def;
  int e = qs.indexOf('&', p);
  String v = (e < 0) ? qs.substring(p + k.length()) : qs.substring(p + k.length(), e);
  return v.toInt();
}

// 配网：保存/点连接后立刻切 WiFi 去连（旧的「20 秒后重启」方案已作废）

// JSON 字符串转义（配网日志尾部回传给网页用）
String jsonEsc(const String& in) {
  String o; o.reserve(in.length() + 16);
  for (unsigned int i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if (c == '\r') { }
    else if ((uint8_t)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

// URL 解码（%XX 与 '+'）：配网的 SSID/密码经 encodeURIComponent 过来
String urlDecode(const String& in) {
  String out; out.reserve(in.length());
  for (unsigned int i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '+') { out += ' '; continue; }
    if (c == '%' && i + 2 < in.length()) {
      auto hex = [](char h)->int { if (h >= '0' && h <= '9') return h - '0';
                                   if (h >= 'a' && h <= 'f') return h - 'a' + 10;
                                   if (h >= 'A' && h <= 'F') return h - 'A' + 10; return -1; };
      int hi = hex(in[i + 1]), lo = hex(in[i + 2]);
      if (hi >= 0 && lo >= 0) { out += (char)(hi * 16 + lo); i += 2; continue; }
    }
    out += c;
  }
  return out;
}
// 取字符串参数（自动 URL 解码）
String bleQStr(const String& qs, const char* key) {
  String k = String(key) + "=";
  int p = qs.indexOf(k);
  if (p < 0) return "";
  int e = qs.indexOf('&', p);
  String v = (e < 0) ? qs.substring(p + k.length()) : qs.substring(p + k.length(), e);
  return urlDecode(v);
}
bool bleHas(const String& qs, const char* key) { return qs.indexOf(String(key) + "=") >= 0; }

// ---- BLE 请求分发（在 loop() 里跑：主任务栈 8KB，足够；与 HTTP 同一套 builder，只开放只读数据 + 配网）----
void bleProcessPending() {
  if (!blePend) return;
  if (bleTxActive) return;              // 上一帧还没发完：保留请求，下一轮 loop 再处理（不丢）
  blePend = false;
  String req(blePendBuf);
  String path = req, qs = "";
  int q = req.indexOf('?');
  if (q >= 0) { path = req.substring(0, q); qs = req.substring(q + 1); }
  // 配网请求的密码在 URL 里：日志只记路径，别把密码写进 weblog
  webLogln("🔵 BLE 请求: %s", (path == "/wifi-save") ? path.c_str() : req.c_str());

  if (path == "/status" || path == "/") { bleSendLine(buildStatusJson(true)); return; }
  if (path == "/today-stats")             { bleSendLine(buildTodayStatsJson()); return; }
  if (path == "/anomaly")                 { bleSendLine(buildAnomalyJson()); return; }
  if (path == "/daily-avg")               { bleSendLine(buildDailyAvgJson()); return; }
  if (path == "/history") {
    // 蓝牙带宽有限：默认只取 100 条（HTTP 默认 200）
    int count = bleQInt(qs, "count", 100);
    if (count <= 0 || count > 100) count = 100;
    bleSendLine(buildHistoryJson(bleQInt(qs, "page", 0), count, bleQInt(qs, "filter", 0) == 1));
    return;
  }
  // ---- 配网（蓝牙版唯一可写的地方；设计文档：只写 Flash、不立即连）----
  if (path == "/wifi-list") { bleSendLine(buildWifiListJson()); return; }
  if (path == "/diag")      { bleSendLine(buildDiagJson()); return; }   // 只读诊断：最近 6 条关键日志
  if (path == "/cfgdbg") {                                              // 只读：配网调试日志（写 Flash）尾部
    String s;
    File f = LittleFS.open(CFG_DBG_PATH, "r");
    if (f) { size_t sz = f.size(); if (sz > 1800) f.seek(sz - 1800); s = f.readString(); f.close(); }
    if (!s.length()) s = "(暂无配网调试日志：还没触发过配网/IO9 按钮)";
    bleSendLine("{\"log\":\"" + jsonEsc(s) + "\"}");
    return;
  }
  if (path == "/wifi-scan") { bleSendLine(buildWifiScanJson()); return; }
  if (path == "/wifi-save") {
    String ssid = bleQStr(qs, "ssid");
    String pass = bleQStr(qs, "pass");
    if (pass.length() == 0) pass = bleQStr(qs, "password");   // 兼容网页表单的字段名
    int useIPMode = bleHas(qs, "useIP") ? (bleQStr(qs, "useIP") == "1" ? 1 : 0) : -1;
    if (ssid.length() == 0) { bleSendLine("{\"ok\":false,\"err\":\"ssid_empty\"}"); return; }
    int idx = wifiUpsert(ssid.c_str(), pass.c_str(), useIPMode, bleQInt(qs, "ipLast", 0));
    bool powered = isExternallyPowered();
    if (idx >= 0) {
      webLogln("📶 [配网] 已保存: %s（共 %d 组）→ 立即切 WiFi 连接", ssid.c_str(), wifiCount);
      beginWifiSession(ssid);                      // 设计文档：保存后立刻连（不再等重启）
    }
    bleSendLine("{\"ok\":" + String(idx >= 0 ? "true" : "false") +
                ",\"count\":" + String(wifiCount) + ",\"max\":" + String(WIFI_MAX_SAVED) +
                ",\"ssid\":\"" + ssid + "\",\"stage\":\"" + String(idx >= 0 ? "connecting" : "failed") + "\"}");
    return;
  }
  if (path == "/wifi-connect") {
    String ssid = bleQStr(qs, "ssid");
    int idx = -1;
    for (int i = 0; i < wifiCount; i++) if (ssid == String(wifiList[i].ssid)) { idx = i; break; }
    if (idx < 0) { bleSendLine("{\"ok\":false,\"err\":\"not_saved\"}"); return; }
    wifiSyncCurrent(idx);                          // 把「当前组」切到这组（用它的静态IP等配置）
    webLogln("📶 [配网] 连接已保存网络: %s", ssid.c_str());
    beginWifiSession(ssid);
    bleSendLine("{\"ok\":true,\"ssid\":\"" + ssid + "\",\"stage\":\"connecting\"}");
    return;
  }
  if (path == "/wifi-del") {
    String ssid = bleQStr(qs, "ssid");
    bool ok = wifiRemove(ssid.c_str());
    webLogln("🗑️ [配网] 删除: %s %s（剩 %d 组）", ssid.c_str(), ok ? "OK" : "未找到", wifiCount);
    bleSendLine("{\"ok\":" + String(ok ? "true" : "false") + ",\"count\":" + String(wifiCount) + "}");
    return;
  }
  bleSendLine("{\"error\":\"not_found\",\"note\":\"蓝牙/局域网只提供只读数据与配网\"}");
}

class BleServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s)    { bleLink = true;  webLogln("🔵 蓝牙已连接"); }
  void onDisconnect(BLEServer* s) { bleLink = false; webLogln("⚪ 蓝牙断开 → 重新广播"); if (!bleStopping) BLEDevice::startAdvertising(); }
};
class BleRxCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    String v = c->getValue().c_str();
    for (size_t i = 0; i < v.length(); i++) {
      char ch = v[i];
      if (ch == '\n' || ch == '\r') { if (bleRxBuf.length()) { bleHandleRequest(bleRxBuf); bleRxBuf = ""; } }
      else if (bleRxBuf.length() < 200) bleRxBuf += ch;
      else bleRxBuf = "";                      // 防跑飞
    }
  }
};

void startBLE() {
  if (bleOn) return;
  BLEDevice::init(BLE_DEV_NAME);
  BLEServer* srv = BLEDevice::createServer();
  srv->setCallbacks(new BleServerCB());
  BLEService* svc = srv->createService(BLE_SVC_UUID);
  bleTx = svc->createCharacteristic(BLE_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());
  BLECharacteristic* rx = svc->createCharacteristic(BLE_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  rx->setCallbacks(new BleRxCB());
  svc->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SVC_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  bleOn = true;
  webLogln("🔵 蓝牙已开启并广播：%s（服务 %s）", BLE_DEV_NAME, BLE_SVC_UUID);
}

void stopBLE() {
  if (!bleOn) return;
  // ① 先停掉我自己的发送，之后不再碰任何蓝牙对象
  bleStopping = true;
  bleLink = false; bleTxActive = false; bleTxBuf = ""; bleTxQN = 0; bleTxQHead = bleTxQTail = 0;
  blePend = false;
  // ② 【关键】必须先主动断开所有客户端，并等 NimBLE 把「断开完成」事件处理完。
  //    原因：core 的 BLEDevice::deinit() **无条件 delete m_pServer**（release_memory 参数
  //    根本没被使用），删完以后晚到的 disconnect 事件还会去 erase 它的
  //    m_connectedClientsMap（毒化 0xbaad5678）→ LoadProhibited。
  //    实测 backtrace: BLEServer::removePeerDevice(BLEServer.cpp:361) ← handleGATTServerEvent(:838)
  //    ← ble_gap_rx_disconn_complete ← BLEDevice::host_task
  //    所以：手机还连着就 deinit，必炸。先断开、等干净、再 deinit。
  BLEServer* s = BLEDevice::getServer();
  if (s) {
    for (int round = 0; round < 20 && s->getConnectedCount() > 0; round++) {
      std::map<uint16_t, conn_status_t> peers = s->getPeerDevices(false);
      if (peers.empty()) { delay(20); continue; }
      for (std::map<uint16_t, conn_status_t>::iterator it = peers.begin(); it != peers.end(); ++it) {
        s->disconnect(it->first);
      }
      delay(25);
    }
    delay(60);                    // 再留一点收尾时间
  }
  // ③ 现在释放才安全
  BLEDevice::deinit(true);
  bleOn = false; bleStopping = false; bleTx = nullptr; bleRxBuf = "";
  webLogln("⚪ 蓝牙已关闭");
}
#else   // ==== ENABLE_BLE == 0：BLE 组件不编译，仅留空实现（状态机/loop 调用点零改动）====
//   函数签名与启用版一致，只是什么都不做；bleOn/bleLink 恒为 false（/status 会如实回报）。
void startBLE() {}                                    // 占位：BLE 已停用
void stopBLE()  { bleOn = false; bleLink = false; }   // 占位：确保 /status 恒报 false
void bleProcessPending() {}                           // 占位：无 BLE 请求
void bleTxPump() {}                                   // 占位：无 BLE 发送
#endif

// ---- 状态机：需要就切，切换时先停对面的射频 ----
void updateWireless() {
  uint8_t want = wantWireless();
  if (want == curWireless) return;
  // 蓝牙「缓关」：切 WiFi 时先留着蓝牙几秒，把最后的推送（SSID/IP）发给网页再关
  if (curWireless == WL_BLE && want == WL_WIFI && bleHoldUntil &&
      (int32_t)(millis() - bleHoldUntil) < 0) return;
  PowerState ps = readPowerState();
  webLogln("📡 无线切换 %s → %s（%s/%s%s）",
           curWireless == WL_BLE ? "蓝牙" : (curWireless == WL_WIFI ? "WiFi" : "无"),
           want == WL_BLE ? "蓝牙" : (want == WL_WIFI ? "WiFi" : "无"),
           deviceMode == 1 ? "移动" : "固定",
           ps.onBattery ? "电池" : (ps.powered ? "外接" : "未知"),
           wlForce ? "·强制" : (wifiForced ? "·按钮" : ""));
  diag(String("无线切换 ") + (curWireless == WL_BLE ? "蓝牙" : curWireless == WL_WIFI ? "WiFi" : "无") +
       " → " + (want == WL_BLE ? "蓝牙" : want == WL_WIFI ? "WiFi" : "无") +
       (ps.onBattery ? "（电池）" : "（外接）"));
  if (want == WL_BLE) { stopAP(); wifiDown(); }
  else if (want == WL_OFF) { stopBLE(); wifiDown(); }   // V2.1 移动+电池：射频全关
  else { stopBLE(); }
  curWireless = want;
  // 状态一变立刻刷顶栏无线图标（不等周期刷新）：此刻 CPU 正好在这儿，几乎零成本
  uiDisplay.setWifi(curWireless == WL_BLE ? UI_WIFI_BLE : wifiUIState());   // 蓝牙态显示 BT
  uiDisplay.showTop(time(nullptr));
  delay(150);                                  // 让射频/协议栈稳一下再起另一个
  if (want == WL_BLE) startBLE();
  else if (want == WL_OFF) webLogln("📴 移动+电池：无线全关（按 IO9 临时开启）");
  else if (!apEnabled && WiFi.status() != WL_CONNECTED) wifiUp();   // 已连上/正开 AP 就别重连（连网会话里 loop 已先连好）
  else webLogln("🔀 WiFi 已在线，沿用现有连接");
}

// ---- 实体 WiFi 按钮（IO9）----
//   V2.1 语义：开无线 → STA 优先（连已存最强，失败再扫）；失败/无组 → 转 AP 配网热点
//   固定：按钮【无效】（状态机本来就是 WiFi/AP，动了只会让用户疑惑「网页怎么打不开」）
//   移动 + 电池：长按 ≥1.5s 才生效 → 先选「已保存且扫到的最强」连；连不上/无组 → 开 AP 配网
//     热点带超时（AP_SESSION_MS），到期或配网成功自动关，避免把电耗光
//   V2.1.1-a 新增：移动 + 插电 → 长按切「快充 / 慢充」（临时，仅本次充电有效，拔插即回快充）
//   【兜底】**长按 ≥5s 松手 → 强制开热点 + 断开 WiFi**（任何模式、任何供电都生效）
//     用途：设备连上了网但**够不着**（如静态 IP 学错网段）时自救 —— 走 AP 192.168.5.1 重新配网
//     顺带把该网络学到的「网关/掩码/DNS」清掉，否则重连还会用旧网段（详见下面的 wifiRecover()）
void wifiRecover(const char* why);          // 前向声明（定义在 startAPWithTimeout 附近）

void checkWifiButton() {
  static bool last = HIGH;
  static unsigned long downAt = 0;
  static bool recoverHinted = false;        // 本次按住是否已提示过「可进救援模式」
  bool now = digitalRead(PIN_WIFI_BTN);
  if (last == HIGH && now == LOW) { downAt = millis(); recoverHinted = false; }
  // 按住期间：满 WIFI_BTN_RECOVER_MS 提示一次（让用户知道松手会进救援模式）—— 救援仍是长按，防误触
  if (now == LOW && !recoverHinted && downAt && (millis() - downAt) >= WIFI_BTN_RECOVER_MS) {
    recoverHinted = true;
    uiDisplay.showToast(UI_TOAST_AP, "HOLD...", "松开 = 进 AP 救援", AP_IP_STR, 4000);
    webLogln("🔘 已按住 %.1fs：再松手将进入「AP 救援模式」（强制开热点、断 WiFi）",
             (millis() - downAt) / 1000.0);
  }
  if (last == LOW && now == HIGH) {
    unsigned long held = millis() - downAt;
    if (held >= WIFI_BTN_RECOVER_MS) {
      // ①【兜底】救援模式：优先级最高，任何模式/供电都生效（**保持长按**，防误触）
      wifiRecover("长按按钮");
    } else if (held >= WIFI_BTN_DEBOUNCE_MS && held <= WIFI_BTN_SHORT_MS) {
      // ② 常规动作：**短按**（v2.2 由长按改为短按）
      const char* how = "短按";
      if (deviceMode == 1 && isExternallyPowered()) {
        // 移动 + 插电 → 切快/慢充（临时、不持久化；拔插即回快充）
        chargeForceSlow = !chargeForceSlow;
        applyChargeStrategy(readPowerState());
        uiDisplay.showToast(UI_TOAST_OK, chargeForceSlow ? "SLOW CHG" : "FAST CHG", "", "", 3000);
        webLogln("🔘 按钮%s %lums：移动+插电 → 临时%s（仅本次充电，拔插回快充）",
                 how, held, chargeForceSlow ? "慢充~297mA" : "快充~890mA");
      } else if (deviceMode == 1) {                 // 移动 + 电池 → 开无线
        wifiBtnArmed = true;
        webLogln("🔘 按钮%s %lums：扫描并连接已保存的最强网络", how, held);
      } else {                                      // 固定 → 无效
        webLogln("🔘 按钮%s：固定模式本就是 WiFi/AP → 忽略（按住 ≥5s 可进 AP 救援模式）", how);
      }
    } else if (held < WIFI_BTN_DEBOUNCE_MS) {
      // 抖动，静默忽略
    } else {
      // 1.2~5s：犹豫按压，不触发任何动作（避免"长按没按够"被误判）
      webLogln("🔘 按钮按住 %.1fs：介于短按与救援之间，未触发动作（短按≤%.1fs / 救援≥%.1fs）",
               held / 1000.0, WIFI_BTN_SHORT_MS / 1000.0, WIFI_BTN_RECOVER_MS / 1000.0);
    }
  }
  last = now;
}
// ===================================================
const int INTERVAL_SEC = 30;
const int LOG_KEEP_DAYS = 14;      // 未插卡时 /log.csv 的保留天数（兜底裁头，见 trimLogOlderThan）
const int KEEP_IN_MEMORY = 600;
const int SD_CS = 7;          // v2.2：SD CS -> IO7

// ===== 电源/电池检测引脚（v2.2 板：全部换脚）=====
// BQ24074 的 PGOOD / CHG 都是【开漏输出、低有效】：
//   PGOOD: 低 = 有效外部电源已接入；高阻 = 无有效输入
//   CHG  : 低 = 充电中；高阻 = 充电完成/未接电源
// 故 ESP32 需用【内部上拉】，读取时为 !digitalRead()
const int PIN_PGOOD = 10;    // 开漏，低=外部电源正常（v2.2：IO2 -> IO10）
const int PIN_CHG   = 9;     // 开漏，低=充电中（v2.2：IO21 -> IO9）
// 注：原 PIN_BAT(IO9) 电池分压 ADC 方案已于 V2.1 删除（板上已无 1M 分压电阻），
//     电池电压唯一来源为 INA230；IO9 释放，留作实体 WiFi 按钮（见设计文档附录 A）。

// ===== 充电控制引脚（V2.1：由 ESP32 软件控制充电）=====
//   PIN_CE   → BQ24074 CE#（pin4）：低=允许充电；高=停止充电（CE 高有效禁用充电）
//   PIN_ISET → ISET 切换 MOS(AO3400A)栅极：高=快充(短路R7，等效 1k → ≈890mA)
//                                         低=慢充(1k+2k 串联 3k → ≈297mA)
// 注：BQ24074 的 EN1/EN2 **由 GPIO 软件控制**（IO11/IO12）；v2.1 的 3P 硬件跳线方案已作废
const int PIN_CE   = 13;      // v2.2：IO17 -> IO13（BQ24074 CE#，低=允许充电）
const int PIN_ISET = 17;      // v2.2：IO18 -> IO17（IO18 让给 TPS2117 PR1）
// ===== TPS2117 电源多路复用 PR1（v2.2 新增）=====
//   PR1 ≥ VREF(≈1 V) → 选 VIN1（电池直供）；PR1 低 → 选 VIN2（LDO 输出）
//   板上已加下拉 → 上电默认低 = LDO 供电（安全默认）；固件只在低压时"单向收紧"抬为高
const int PIN_PR1  = 18;      // v2.2：TPS2117 PR1（切换 LDO ⇄ 电池直供）
// ⚠️ BQ24074 的 EN1/EN2 在 v2.2 板接到 IO11/IO12；**本版固件暂不驱动**（保持引脚默认电平），
//    待 USB 枚举检测实装后再接管（见上方 USB_ENUM_DETECT 与 硬件核对清单 第五节清单 3）
const int PIN_EN1  = 11;
const int PIN_EN2  = 12;

// ===== 电池监测 INA230（v2.2：替代 IO9 分压；INA226 已弃用）=====
//   I2C 与传感器共用总线（IO47=SDA / IO48=SCL），地址 0x40（A0=A1=GND）
//   高边 10mΩ 采样电阻跨在 BQ24074@BAT 与电池正极之间，VIN+ 靠 BQ 侧
//     → 充电时电流为正、放电为负；VBUS 接 VIN-（电池侧）→ 读电池真实电压
//   驱动实现见同目录 ina230.h（放头文件可绕开 .ino 原型注入的自定义类型问题）
const int   INA230_ADDR      = 0x40;     // A0=A1=GND
const float INA230_R_SHUNT   = 0.010f;   // 采样电阻 10 mΩ
const float INA230_MAX_A     = 1.0f;     // 预期最大电流(A)，用于自动选 Current_LSB
const float CHARGE_QUICK_mA  = 890.0f;   // 移动模式快充设定
const float CHARGE_SLOW_mA   = 297.0f;   // 固定模式慢充设定
const float CV_ZONE_RATIO    = 0.60f;    // 电流 < 设定值×60% → 判为 CV 区
const float CV_VOLT_GATE     = 4.05f;    // 电压门限：低于此值不判 CV（防误报）
INA230 ina230;                           // INA230 驱动对象（ina230.h）
// deviceMode 定义在本文件靠后（设备模式，CV 判定要用），此处先声明
extern uint8_t deviceMode;

// ===== 低电保护阈值（设计文档三档；仅电池供电且 INA230 在线时判定）=====
// 插电(PGOOD低)   : 无条件记录，外部电源绝对充足
// INA230 离线      : 电压未知 → 低电门控整体作废（照常记录、不深睡），仅告警
// 电池 + INA230在线:
//   电压 ≥ 存档档  → 正常记录
//   停采档 ~ 存档档 → 存档数据至 SD
//   深睡档 ~ 停采档 → 停止采集
//   < 深睡档       → 深睡保护（保命：强制用户充电）
// ⚠️ V2.1.1-b 整体上移（原 3.3/3.25/3.2）：实测电池 <~3.4V 时 LDO 低压差输出能力暴跌，
//    设备会在旧阈值触发【之前】就硬掉电 → 旧三档实际从未生效过（详见设计文档 V2.1.1 · 2.7）。
// ===== V2.1.1「低电特性更新」：三档改成**跟着本机校准表走**（2026-09-29）=====
//   规格：有校准文件 → 三档取本机曲线的低电端（深睡 5% / 停采 10% / 存档 15%，各留 ≥50mV 余量）；
//         无表（或只有干跑表）→ 用下面的**回落值**。
//   动机：每台设备标的是自己那块电池 → 换 LDO / 换电容 / 板间差异**自动被本机曲线吸收**，
//         不必再按"最差板"定阈值（见 实现细节.md D20-5 ④、D9 ⑦③）。
//   ⚠️ 回落值保持 3.6/3.55/3.5 不动：与标定首跑实测（按表 15/10/5% −100mV ≈ 3.65/3.61/3.54）互相印证，
//      且与最早那批 Y5V 板的 LDO 崩溃点 ~3.4V 留了 100mV 余量。
const float BATT_ARCHIVE_SD_DEF = 3.60f;   // 回落值：存档数据至 SD 卡
const float BATT_STOP_RECORD_DEF = 3.55f;  // 回落值：停止采集
const float BATT_DEEP_SLEEP_DEF  = 3.50f;  // 回落值：进入深睡保护
const float BATT_SLEEP_MARGIN_V  = 0.100f; // 深睡档余量（表 5% 点再提前 100mV，避开 LDO 崩溃点）
// 生效值：开机由 updateBattThresholds() 按本机表刷新（初始 = 回落值，表未就绪时也用它）
float battArchiveV = BATT_ARCHIVE_SD_DEF;  // 低于此电压：存档数据至 SD 卡
float battStopV    = BATT_STOP_RECORD_DEF; // 低于此电压：停止采集
float battSleepV   = BATT_DEEP_SLEEP_DEF;  // 低于此电压：进入深睡保护

// ===================================================
// ===== V2.1.1-b 电量校准（放电曲线标定）=====
//   规格见 设计文档.md · V2.1.1 · 二、电量校准
//   流程：移动+插电+充满 → 记 V_100 → 拔电 → 占空比放电（满载 T_LOAD / 静置 T_REST 采样）
//         → 掉电 → 插电开机 → 建 21 点表 /battcal.json（旧表备份 /battcal.prev.json）
//   标定期间：停 HTTP 服务、停记录、停传感器、蓝牙不跑，屏幕为唯一界面；长按 IO9 中止
// ===================================================
#define CAL_OFF  0      // 未标定
#define CAL_ARM  1      // 已确认开始，等拔掉充电器（待放电）
#define CAL_RUN  2      // 放电循环中
#define CAL_STOP 3      // 已中止

// ---- 参数 ----
const uint32_t CAL_LOAD_MS      = 9UL * 60UL * 1000UL;   // 加载段（满载耗电）
const uint32_t CAL_LOAD_LOW_MS  = 60UL * 1000UL;         // 低压段加载（加密采样）
const uint32_t CAL_REST_MS      = 60UL * 1000UL;         // 静置段（静置采样）
const float    CAL_LOW_FROM_V   = 3.60f;                 // 低于此电压 → 用低压段时长（3.50→3.60：更早转短加载段，减少下探）
// ⚠️ 2026-10-03 用户裁定：3.00V → **3.20V**。
//   原因：3.00V 太贴地 —— 放电负载下电压还会下探，容易把电池压到**保护板切断**，
//         下次开机体验极差（实测：校准把电池放到 0.474V，设备"像坏了"）。
//   留 0.2V 余量后，校准结束时电池仍有可观的剩余电量。
const float    CAL_FLOOR_V      = 3.20f;                 // 硬底线：低于此电压立即收工
const uint32_t CAL_FULL_HOLD_MS = 5UL * 60UL * 1000UL;   // 充满判定：条件需持续
const float    CAL_FULL_MIN_V   = 4.10f;                 // 充满判定：电压下限
const uint8_t  CAL_DRY_CYCLES   = 6;                     // 干跑：跑够几轮就模拟掉电重启
const uint32_t CAL_DRY_LOAD_MS  = 20UL * 1000UL;         // 干跑加载段
const uint32_t CAL_DRY_REST_MS  = 10UL * 1000UL;         // 干跑静置段
const int      CAL_MAX_ROWS     = 256;                   // CSV 最多解析行数
const char*    CAL_CSV_PATH     = "/battcal.csv";
const char*    CAL_JSON_PATH    = "/battcal.json";
const char*    CAL_PREV_PATH    = "/battcal.prev.json";
// INA230 标定期平均窗口：512 × 2 × 8.244ms ≈ 8.4 s（覆盖静置窗口，稀释自身开销）
const Ina230Avg      CAL_AVG = INA230_AVG_512;
const Ina230ConvTime CAL_CT  = INA230_CT_8244US;

// ---- 运行态（全 RAM；标定全程在一个上电周期内完成，掉电即结束）----
uint8_t  calState       = CAL_OFF;
bool     calDry         = false;      // 干跑：缩短时长 + 保留网页 + 不要求充满
uint32_t calStartMs     = 0;          // 放电起始（wall，用于「已历时」）
uint32_t calLoadedMs    = 0;          // 累计「加载时长」= 时间轴 x（静置不计）
uint32_t calCycleLoadMs = 0;          // 本轮加载时长
uint8_t  calPhase       = 0;          // 0=加载 1=静置
uint32_t calCycles      = 0;
bool     calLoadBad     = false;      // 本加载段 WiFi 是否掉过线（第9列 load_ok 的来源）
uint32_t calUiNextMs    = 0;          // 下次刷屏时刻
float    calV100        = NAN;        // 100% 锚点（开始时电池侧无电流 = 电芯静置电压）
float    calVLoad = NAN, calVRest = NAN, calILoad = NAN, calIRest = NAN;
float    calLastVolt = NAN, calLastI = NAN;
float    calAmbTemp  = NAN;           // 标定开始时的一次性环境温度（标定期间不读传感器）
int      calLastPct     = -1;
uint32_t calFullStartMs = 0;          // 满电条件起始（0=当前不满足）

// ---- 曲线表（21 点：calPctV[0]=0% … calPctV[20]=100%）----
bool  calTableOk  = false;
bool  calTableDry = false;            // 干跑建的表：只用于查看，**不生效**
int   calTableN   = 0;
float calPctV[21];
float calTableV100 = NAN, calTableVEnd = NAN;
char  calTableDate[24] = "-";

WiFiUDP calUdp;                       // 标定负载用的 UDP（只在标定时发）

// 读取电源状态
// 读取电源状态
//   电池电压/电流唯一来源 = INA230（原 IO9 分压 ADC 方案已随 V2.1 删除）。
//   INA230 离线时：battVolt = NAN、battPct = 0、inaOK = false，
//   上层据此【作废】低电保护与电量显示，改为告警 —— 不得按 0V 误判。
PowerState readPowerState() {
  PowerState ps = {};
  // 开漏输出：接内部上拉，正常态应读到低
  pinMode(PIN_PGOOD, INPUT_PULLUP);
  pinMode(PIN_CHG,   INPUT_PULLUP);
  ps.powered   = (digitalRead(PIN_PGOOD) == LOW);   // 低=有外部电源
  ps.charging  = (digitalRead(PIN_CHG)   == LOW);   // 低=充电中
  ps.onBattery = !ps.powered;

  ps.inaOK          = false;
  ps.battVolt       = NAN;      // 未知（INA230 在线时下面覆盖）
  ps.battPct        = 0;
  ps.battCurrent_mA = 0;
  ps.battPower_mW   = 0;
  ps.cvZone         = false;

  // 电池电压/电流：仅 INA230（不再有分压回退）
  if (ina230.ok()) {
    Ina230Reading ir;
    if (ina230.read(ir)) {
      ps.inaOK          = true;
      ps.battVolt       = ir.busVolt_V;      // 电池真实对地电压（VBUS 接 VIN-）
      ps.battCurrent_mA = ir.current_mA;     // 充电为正 / 放电为负
      ps.battPower_mW   = ir.power_mW;
      // CV 区判定：正在充电 + 电压已高 + 电流跌到设定值的 60% 以下
      float set_mA = (deviceMode == 1) ? CHARGE_QUICK_mA : CHARGE_SLOW_mA;
      if (ps.charging && ps.battVolt >= CV_VOLT_GATE && set_mA > 0 &&
          fabsf(ps.battCurrent_mA) < set_mA * CV_ZONE_RATIO) {
        ps.cvZone = true;
      }
    }
  }
  if (ps.inaOK) {
    // 电量：优先用标定曲线（V2.1.1-b，21 点插值）；无表或干跑表 → 退回线性估算
    //   线性估算下限定为 3.4V（LDO 崩溃点）：3.4V 以下的电量对设备无意义
    if (calTableOk && !calTableDry) {
      ps.battPct = calPctFromVolt(ps.battVolt);
    } else {
      float vmin = 3.4f, vmax = 4.2f;
      if (ps.battVolt >= vmax) ps.battPct = 100;
      else if (ps.battVolt <= vmin) ps.battPct = 0;
      else ps.battPct = (uint8_t)((ps.battVolt - vmin) / (vmax - vmin) * 100);
    }
  }
  return ps;
}

// 是否插着外部电源（PGOOD 低有效）。包装成 bool 函数：供前面的蓝牙配网代码调用，
// 避开 “早于 PIN_PGOOD 定义” 与 “.ino 原型注入自定义类型” 两个坑。
bool isExternallyPowered() { return digitalRead(PIN_PGOOD) == LOW; }

// ==================== 充电控制底层原语（BQ24074 CE# + ISET 切换 MOS）====================
// 硬件（V2.1）：
//   IO17 → BQ24074 CE#     ：低=允许充电；高=停止充电
//   IO18 → ISET 切换 MOS 栅极：高=快充(短路R7 → 等效 1k → ≈890mA)
//                              低=慢充(1k+2k 串联 → 3k → ≈297mA)
// 说明：本层只提供「底层原语」。固定/移动模式切换、80% 停充策略后续在此之上实现，
//       调用方只需 chargeSetCurrent() / chargeSetEnabled()，不直接碰 IO 电平。
// 充电使能与「充电中」状态区别：CE 是“允许/禁止充电”，CHG 是“当前是否在充电”。
// ===== 冷启动判据（2026-10-03）=====
//   设备封装后电池永不拔除 → **冷启动只可能是"电池没电"**；
//   另一种是用户拨 EN 开关（此时电池通常有电）。
//   故按 VBAT 分流：< 3.6V 视为冷启动（优先起系统），≥ 3.6V 走常规快启动。
//   用户定 3.6V：取得太保守会让"拨 EN 开机"的体验变差。
#define PWR_COLD_START_V  3.5f   // 用户裁定 3.6→3.5（读数时机改到空载后，实测空载 3.8V 与带载 3.49V 差异已消除）

// ===== 实测结论：USB500 下能否充电？（2026-10-03 实验，已定论）=====
//   【结论：**可以充，而且速度可观**】
//   实验条件：插电脑 USB 主机（判定 host）→ USB500(500mA) + 允许充电 + 快充档
//   实测数据：充电电流 **+425 mA 稳定**，电池电压 3.59V → 3.64V（持续爬升），
//             18 个采样点无复位、无失联。
//
//   ⚠️ 早先曾测得 -1.0mA 并据此写死 false —— **那个数据是错的**：
//      当时 TS 脚还是 2 kΩ（芯片判"过热"停充），测到的是 TS 故障，不是"USB500 不够"。
//      教训：**测量条件有已知故障时，数据不能当结论。**
//
//   正确模型（BQ24074 电源路径管理）：
//     系统**优先从输入(USB)取电**，剩余电流才进电池。
//     500 mA 上限下：系统约用 75 mA，剩 ≈425 mA 给电池充电。
//     ——而非"系统把 500 mA 吃满、电池拿不到"。
//
//   ⇒ 故：**USB 主机下允许充电**（插电脑就能充，体验更好）。
//      充电器（ISET 档，约 1.3 A）会更快。
const bool usbChargingViable = true;

// ===== USB 枚举状态（由 serviceUsbEnum 判定）=====
uint8_t usbEnumState   = ENUM_UNKNOWN;   // ENUM_UNKNOWN / ENUM_IS_HOST / ENUM_IS_CHARGER
bool    usbAllowCharge = false;          // 是否允许充电（枚举成功才允许）

// 读 USB-Serial-JTAG 的 SOF 帧号（低 11 位）
static inline uint32_t usbReadSofFrame() {
  return (*(volatile uint32_t*)USB_JTAG_FRAM_NUM) & USB_SOF_FRAME_MASK;
}

// ---- USB 枚举检测：观察 SOF 帧号是否在变化 ----
//   返回 true = 对端是 USB 主机（帧号在动）；false = 充电器/无主机（帧号静止）
bool usbDetectHost(uint32_t windowMs = ENUM_WINDOW_MS) {
  volatile uint32_t* clr = (volatile uint32_t*)USB_JTAG_INT_CLR;
  *clr = 0xFFFFFFFFUL;                       // 清掉历史 SOF 中断，从干净状态开始
  delay(5);

  uint32_t f0 = usbReadSofFrame();
  uint32_t start = millis();
  while (millis() - start < windowMs) {
    delay(200);
    if (usbReadSofFrame() != f0) return true;   // 只要动过一次就判定为主机
  }
  return false;
}

// ---- 启动后调用一次：判定电源类型并据此设定 BQ24074 档位 ----
void serviceUsbEnum() {
  if (usbEnumState != ENUM_UNKNOWN) return;      // 只判定一次

  bool ext = isExternallyPowered();
  uint32_t a = usbReadSofFrame();
  delay(250);
  uint32_t b = usbReadSofFrame();
  bool sofMoving = (a != b);

  if (ext && sofMoving) {
    // ===== USB 主机（电脑）=====
    usbEnumState   = ENUM_IS_HOST;
    usbAllowCharge = usbChargingViable;           // 由实测决定（见 usbChargingViable）
    //   ⚠️ 冷启动（电池没电）不否决枚举结果：若日后实测 USB500 下可充，
    //      这里会自然生效；冷启动只影响"启动期先不充、把电流让给系统"。
    // 保持 EN1=高 / EN2=低 = USB500（已在 chargeInit 设好）
    webLog("🔎 USB 枚举：**主机**（SOF 帧号 %lu→%lu 在动）→ 保持 USB500(500mA)，%s\n",
           (unsigned long)a, (unsigned long)b, usbAllowCharge ? "允许充电" : "**禁止充电**");
  } else if (ext) {
    // ===== 充电器（无 SOF）=====
    usbEnumState   = ENUM_IS_CHARGER;
    usbAllowCharge = true;
    pinMode(PIN_EN1, OUTPUT); pinMode(PIN_EN2, OUTPUT);
    digitalWrite(PIN_EN1, LOW);
    digitalWrite(PIN_EN2, HIGH);                  // (EN1,EN2)=(0,1) = ISET 档（ILIM 电阻设定，约 1.3A）
    digitalWrite(PIN_EN1, LOW); digitalWrite(PIN_EN2, HIGH);
    webLog("🔎 USB 枚举：**充电器**（SOF 帧号 %lu 静止）→ 切 ISET 档(ILIM)，允许充电\n",
           (unsigned long)b);
  } else {
    // ===== 纯电池（无外接电源）=====
    usbEnumState   = ENUM_IS_CHARGER;             // 视作"非主机"，不涉及 EN1/EN2
    usbAllowCharge = false;
    webLogln("🔎 USB 枚举：无外接电源（纯电池）→ 不涉及 EN1/EN2");
  }

  if (usbAllowCharge) digitalWrite(PIN_CE, LOW);   // 放行充电
  else                digitalWrite(PIN_CE, HIGH);  // 锁死禁充
}
// 读一次电池电压（INA230）；无效返回 NAN。
//   ⚠️ 必须在 ina230.begin() 之后调用。
static float readBatteryVoltage() {
  if (!ina230.ok()) return NAN;
  Ina230Reading r;
  if (!ina230.read(r)) return NAN;
  return r.busVolt_V;
}

// ===== 冷启动 / 热启动 分流（2026-10-03 实装 · 设计文档 V2.2 电源管理模块）=====
//   设备封装后电池永不拔除 ⇒ **冷启动只可能是"电池没电"**；
//   另一种上电是用户拨 EN 开关（此时电池通常有电）。
//   故按 VBAT 分流（阈值 PWR_COLD_START_V = 3.6V，用户定 —— 取得太保守会让拨 EN 开机体验变差）：
//     VBAT >= 3.6V → 【热启动】沿用老逻辑：允许充电，档位按 deviceMode 走
//     VBAT <  3.6V → 【冷启动】保持禁充（系统优先起），充电交由 serviceUsbEnum() 定
//   ⚠️ 调用时机：**必须在 INA230 初始化之后**（chargeInit 太早，那时读不到电压）。
bool pwrColdStart = false;      // true = 本次为冷启动
bool pwrStartupDone = false;    // 是否已分流
bool pwrStartupChargeOk = false;// 热启动时置 true → 启动期即可充电

void serviceBatteryStartup() {
  if (pwrStartupDone) return;
  pwrStartupDone = true;

  float v = readBatteryVoltage();
  if (isnan(v)) {
    // 读不到电压 → 无法判断 → 按**冷启动**保守处理（宁慢不错）
    pwrColdStart = true;
    webLogln("🔋 启动分流：电池电压不可读 → 按【冷启动】保守处理（保持禁充，待枚举）");
    return;
  }

  if (v >= PWR_COLD_START_V) {
    pwrColdStart = false;
    pwrStartupChargeOk = true;
    usbAllowCharge = true;      // 热启动：电池有电 → 提前放行充电（用户无感）
                                //   之后 serviceUsbEnum() 若判定为 USB 主机，会按实测结论收回
    webLog("🔋 启动分流：VBAT=%.3fV ≥ %.1fV → 【热启动】（拨 EN 开机）→ 沿用老逻辑，允许充电\n",
           v, PWR_COLD_START_V);
  } else {
    pwrColdStart = true;
    pwrStartupChargeOk = false;
    webLog("🔋 启动分流：VBAT=%.3fV < %.1fV → 【冷启动】（电池没电）→ 保持禁充，系统优先启动\n",
           v, PWR_COLD_START_V);
  }
}
void chargeInit() {
  // 上电安全默认。
  // ⚠️ 顺序很关键：必须先 pinMode(OUTPUT) 再 digitalWrite。
  //    实测（ESP32 core 3.3.11 / S3）：软复位（ESP.restart）后，
  //    「先 digitalWrite 再 pinMode」不会真正拉低引脚，引脚会保留上一次会话的电平——
  //    曾观测到预置快充态重启后仍读到 IO18=1，对固定"必须慢充"是隐患。
  //    改为先设方向再写电平，并末尾复写一次兜底。
  pinMode(PIN_ISET, OUTPUT);
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_ISET, LOW);   // MOS 关断 → 慢充档

  // ---- BQ24074 输入限流：**必须先抬到 USB500，早于 WiFi 初始化** ----
  //   手册真值表：(EN1,EN2)=(0,0)100mA / (1,0)USB500(500mA) / (0,1)ISET档(ILIM 电阻设定)
  //   为什么必须早：ROM/固件早期与 WiFi 初始化都有电流峰值，
  //   若等到枚举之后才抬，系统可能已经在枚举阶段掉压复位（实测教训）。
  //   500mA 是 USB 规范安全值：即便插在仅能出 500mA 的电脑口也不会过载。
  pinMode(PIN_EN1, OUTPUT);
  pinMode(PIN_EN2, OUTPUT);
  digitalWrite(PIN_EN1, HIGH);
  digitalWrite(PIN_EN2, LOW);     // → USB500
  digitalWrite(PIN_EN1, HIGH);    // 兜底复写
  digitalWrite(PIN_EN2, LOW);

  // ---- 充电使能：启动期一律**先禁充**（系统优先），由 serviceUsbEnum 事后放行 ----
  //   理由：供电受限时充电会和系统抢同一份输入电流；系统起不来就没有后续可言。
#if PWR_SAFE_START
  // 【安全启动】CE 先保持**高（禁充）**，把输入电流全留给系统。
  //   之后由两处按运行期条件放行：
  //     · serviceBatteryStartup()：热启动（VBAT>=3.6V）→ 直接放行（沿用老逻辑，用户无感）
  //     · serviceUsbEnum()      ：判定为充电器 → 放行；判定为 USB 主机 → 保持禁充
  digitalWrite(PIN_CE, HIGH);
  webLogln("🔌 【安全启动】输入限流=USB500(500mA) | CE=高(禁充，待分流/枚举后再定)");
#else
  digitalWrite(PIN_CE, LOW);      // 老逻辑：允许充电（且 EN1/EN2 不驱动 → 保持上电默认 USB100）
  webLogln("🔌 充电控制初始态：允许充电 | 输入限流 = 上电默认 (0,0)=USB100");
#endif
  digitalWrite(PIN_ISET, LOW);    // 兜底复写

  // ===== TPS2117 PR1 上电安全默认（v2.2 新增）=====
  //   ⛔ **必须最先拉低**：PR1 低 → 选 VIN2(LDO 3.3V)；PR1 高 → 选 VIN1(电池直供)。
  //      模组 VDD33 绝对最大 3.6 V，而电池满电 4.2 V → 若上电瞬间选中 VIN1，
  //      **超出绝对最大值，永久损坏芯片**（依据：硬件核对清单 第三节 ②）。
  //   ⚠️ 同样遵守"先 pinMode 再 digitalWrite"的顺序（见上面软复位实测教训）。
  //   板上已有下拉给出安全默认，这里再显式拉低一次做双保险。
  pinMode(PIN_PR1, OUTPUT);
  digitalWrite(PIN_PR1, LOW);
  digitalWrite(PIN_PR1, LOW);    // 兜底复写
}

// 电流档：fast=true → 快充(~890mA)；false → 慢充(~297mA)
void chargeSetCurrent(bool fast) { digitalWrite(PIN_ISET, fast ? HIGH : LOW); }
// 当前是否为快充档
bool chargeIsFast()              { return digitalRead(PIN_ISET) == HIGH; }

// 充电使能：en=true → 允许充电(CE低)；false → 停止充电(CE高)
void chargeSetEnabled(bool en)   { digitalWrite(PIN_CE, en ? LOW : HIGH); }
// 当前是否允许充电
bool chargeIsEnabled()           { return digitalRead(PIN_CE) == LOW; }

// 当前档位对应的设定电流(mA)，供 INA230 闭环校验 / CV 判定使用
float chargeSetpoint_mA()        { return chargeIsFast() ? CHARGE_QUICK_mA : CHARGE_SLOW_mA; }

// ===== 设备模式 → 充电策略（设计文档 V2.1 · 充电策略）=====
//   固定(0)：慢充 + 80% 停充（CE 拉高）；滞回「≥4.05V 停 / ≤4.00V 恢复」→ 约 80% 浮充
//   移动(1)：快充 + 充满（CE 保持低，由 BQ24074 硬件终止到 4.2V）
// 说明：CE 拉高只停「充电」，不切断电池补充(UPS)通路，停充期间照常供电/记录。
//       INA230 离线时无法判电压 → 一律允许充电（fail-safe），仅告警一次。
const float FIX_STOP_V   = 4.05f;   // 固定停充点（≈80%）
const float FIX_RESUME_V = 4.00f;   // 固定恢复点（滞回下沿）
bool fixChargeBlocked = false;      // true = 固定已停充（CE 高）
// --- V2.1.1-a：移动模式下的「临时慢充」（IO9 在【移动 + 插电】时切换）---
//   不持久化（RAM 标志，不写 NVS）；一断电即复位（拔插回来 = 默认快充），见 serviceChargeTemp()。
bool chargeForceSlow = false;        // true = 本次充电临时用慢充（~297mA）

// ===== TPS2117 PR1 电池直供互锁（v2.2 新增）=====
//   目的：电池电压过低时绕过 LDO 让电池直供，把可用下限从 ~3.5V 压到 ~3.0V。
//
//   ⛔⛔ **安全红线（违反 = 永久损坏硬件）** ⛔⛔
//     模组 VDD33 绝对最大 **3.6 V**（S3 手册表 9）；电池满电 **4.2 V**。
//     PR1 高 = 选 VIN1（电池直供）→ 满电时会把 4.2 V 直接灌进模组 → **烧毁**。
//     故：**VBAT > 3.6 V 时，任何代码路径都不得把 PR1 拉高。**
//
//   阈值与滞回（依据：硬件核对清单 第四节 + 第五节清单 4）：
//     VBAT < 3.4 V          → 允许 PR1 高（切电池直供）
//     VBAT ≥ 3.5 V          → 回 PR1 低（回 LDO）
//     VBAT > 3.6 V          → ⛔ 一律禁止拉高（硬约束，优先于其他一切条件）
//     INA230 离线/读数无效   → 保持 PR1 低（fail-safe：宁可早关机，也不灌 4.2V 进模组）
//
//   ⚠️ 默认态 PR1 低 = LDO 安全；板上另有下拉作硬件默认。
const float PR1_ON_V      = 3.40f;   // 低于此值才允许切电池直供
const float PR1_OFF_V     = 3.50f;   // 回到此值以上切回 LDO
const float PR1_HARD_MAX  = 3.60f;   // ⛔ 超过此值一律禁止拉高（绝对最大 3.6V）
const float PR1_VALID_MIN  = 2.00f;   // 低于此值视为读数无效（未接电池/传感器故障）
//   低电深睡用的"读数合理性"下限：0V ≈ 电池被拔/未接，绝不能据此深睡（会醒不过来）
const float LOWBAT_SANE_MIN_V = 0.50f;
const float PR1_VALID_MAX = 4.50f;   // 高于此值视为读数无效

bool pr1BatteryDirect = false;       // true = 当前选了 VIN1（电池直供）

//   上电宽限期：这段时间内一律保持 LDO，让电源轨稳定、读数可信
const unsigned long PR1_GRACE_MS = 60000UL;   // 60 秒

void pr1SetDirect(bool direct, float vbat) {
  // ⛔ 最后一道保险：拉高之前再验一次电压。任何情况下都不许在高压时切直供。
  if (direct && vbat > PR1_HARD_MAX) {
    webLog("⛔ PR1 拒绝拉高：VBAT=%.3fV > %.2fV（绝对最大 3.6V，拉了会烧模组）\n",
           vbat, PR1_HARD_MAX);
    return;                                   // 保持现状（低 = LDO）
  }
  digitalWrite(PIN_PR1, direct ? HIGH : LOW);
  if (direct != pr1BatteryDirect) {
    pr1BatteryDirect = direct;
    webLog("🔀 PR1 → %s（VBAT=%.3fV）\n",
           direct ? "VIN1 电池直供" : "VIN2 LDO", vbat);
  }
}

void servicePr1Interlock(const PowerState& p) {
  // ★ 守卫 0：上电宽限期 —— 电源未稳、读数不可信，一律保持 LDO
  if (millis() < PR1_GRACE_MS) { if (pr1BatteryDirect) pr1SetDirect(false, p.battVolt);
                                 else digitalWrite(PIN_PR1, LOW); return; }

  // ★ 守卫 1（2026-10-03 加固）：**外接电源在位时绝不切电池直供**
  //   用户指出：插着电去旁路 LDO 毫无意义 —— 旁路的意义只在电池供电时榨续航。
  //   且拔插电池时 PGOOD 会瞬变，若此刻结合一个低电压读数就会误切 → 设备瞬间断电。
  //   双重判据（PGOOD + 内部状态），且读数与状态自相矛盾时按"有外接电源"处理（保守）。
  bool extPower = p.powered || isExternallyPowered()
                  || (p.charging && !isnan(p.battVolt) && p.battVolt > PR1_OFF_V);
  if (extPower) {
    if (pr1BatteryDirect) {
      pr1SetDirect(false, p.battVolt);
      webLogln("🛡️ PR1 回 LDO：检测到外部电源（插电时不旁路 LDO）");
    } else {
      digitalWrite(PIN_PR1, LOW);
    }
    return;
  }

  // 1) 读数有效性：无效 → 保持/回到 LDO（fail-safe）
  bool vValid = p.inaOK && p.battVolt > PR1_VALID_MIN && p.battVolt < PR1_VALID_MAX;
  if (!vValid) {
    static unsigned long lastWarn = 0;
    if (pr1BatteryDirect) pr1SetDirect(false, p.battVolt);
    else digitalWrite(PIN_PR1, LOW);           // 确保低（即使状态变量没变也复写一次）
    if (millis() - lastWarn > 300000UL) {      // 5 分钟限流，避免刷屏
      lastWarn = millis();
      webLogln("🛡️ PR1 保持 LDO：电池电压读数无效（INA 离线或超出 2.0~4.5V）");
    }
    return;
  }

  // 2) 电压过高 → 强制回 LDO（**这条就是防烧板子的关键分支**）
  if (p.battVolt > PR1_HARD_MAX) {
    if (pr1BatteryDirect) {
      pr1SetDirect(false, p.battVolt);
      webLog("🛡️ PR1 强制回 LDO：VBAT=%.3fV 高于硬上限 %.2fV\n", p.battVolt, PR1_HARD_MAX);
    } else {
      digitalWrite(PIN_PR1, LOW);              // 常态：复写低，零风险
    }
    return;
  }

  // 3) 正常滞回区间（此时必然 VBAT ≤ 3.60V，切直供是安全的）
  if (!pr1BatteryDirect && p.battVolt < PR1_ON_V) {
    pr1SetDirect(true, p.battVolt);            // 低压 → 切电池直供
  } else if (pr1BatteryDirect && p.battVolt >= PR1_OFF_V) {
    pr1SetDirect(false, p.battVolt);           // 电压回升 → 回 LDO
  }
}

void applyChargeStrategy(const PowerState& ps) {
#if PWR_SAFE_START
  // ★ 安全启动模式下，**充电使能由 usbAllowCharge 决定**（运行期变量）。
  //   ⛔ 这里**必须用运行期 if，不能用 #if** ——
  //      #if 是编译期求值，会把运行期变量当成 0，导致条件恒真、
  //      永远走"禁充 + return"，把 serviceUsbEnum() 拉低的 CE 又拉回去。
  //      （2026-10-03 实测踩到：插充电器时 allowCharge=true 但 chgEnabled=false）
  // ★ 充电使能**由枚举结果决定**（usbAllowCharge）。
  //   冷/热启动只影响"启动期是否提前充电"，不影响运行期充电权：
  //     热启动 → serviceBatteryStartup() 提前把 usbAllowCharge 置 true（用户无感）
  //     冷启动 → 保持禁充，等 serviceUsbEnum() 判完再说（充电器仍会允许充电）
  if (!usbAllowCharge) {
    digitalWrite(PIN_CE, HIGH);      // 未获准充电 → 锁死禁充
    return;
  }
#endif
  // 1) 电流档位随模式切换（仅在变化时打印，避免日志刷屏）
  //    V2.1.1-a：移动默认快充；移动+插电时按 IO9 可临时切慢充（chargeForceSlow，拔插即失效）
  bool wantFast = (deviceMode == 1) && !chargeForceSlow;
  if (chargeIsFast() != wantFast) {
    chargeSetCurrent(wantFast);
    webLog("⚡ 充电档位 -> %s（%s）\n", wantFast ? "快充~890mA" : "慢充~297mA",
           deviceMode != 1 ? "固定FIX" : (chargeForceSlow ? "移动·临时慢充" : "移动MOV"));
  }

  // 2) 停充点
  if (deviceMode == 1) {                       // 移动：允许充电，充满由 BQ 硬件终止
    if (fixChargeBlocked || !chargeIsEnabled()) {
      fixChargeBlocked = false;
      chargeSetEnabled(true);
      webLogln("🔌 移动模式：允许充电（硬件终止于 4.2V 满）");
    }
    return;
  }
  if (!ps.inaOK) {                             // 固定 + INA 离线：无法判定 → 允许充电
    static bool warnedFixNoIna = false;
    if (!warnedFixNoIna) {
      webLogln("⚠️ INA230 离线：无法判定固定 80% 停充点，暂按「允许充电」处理");
      warnedFixNoIna = true;
    }
    if (fixChargeBlocked || !chargeIsEnabled()) { fixChargeBlocked = false; chargeSetEnabled(true); }
    return;
  }
  if (!fixChargeBlocked && ps.battVolt >= FIX_STOP_V) {
    fixChargeBlocked = true;
    chargeSetEnabled(false);                   // CE 高 → 停充
    webLog("🛡️ 固定：电池 %.2fV ≥ %.2fV（≈80%%）→ 停充，保护电池寿命\n", ps.battVolt, FIX_STOP_V);
  } else if (fixChargeBlocked && ps.battVolt <= FIX_RESUME_V) {
    fixChargeBlocked = false;
    chargeSetEnabled(true);                    // CE 低 → 恢复充电
    webLog("🔋 固定：电池 %.2fV ≤ %.2fV → 恢复充电\n", ps.battVolt, FIX_RESUME_V);
  }
}

// V2.1.1-a：临时慢充只活在「插电期间」—— 一断电（拔线/掉电）立即复位，再插回来就是默认快充。
//   loop() 每圈调用（一次 digitalRead，极廉价）；未切慢充时直接返回。
void serviceChargeTemp() {
  if (!chargeForceSlow) return;                       // 常态：零开销
  if (!isExternallyPowered()) {                       // 外部电源已断开 → 复位
    chargeForceSlow = false;
    applyChargeStrategy(readPowerState());
    webLogln("🔌 外部电源断开 → 临时慢充复位（再插回默认快充）");
  }
}

// ===== 低电深睡保护 + 插电唤醒 =====
// 场景：电池供电且电压 < 3.5V → 深睡（保命，防止电压不足时写SD/Flash损坏数据）。
// 唤醒源：用 PGOOD(IO2) 做 ext0 唤醒 —— 插电时 PGOOD 被拉低(低电平) → 唤醒设备。
//        因此“插电即唤醒并恢复记录”，无需按键。
// 注意：进深睡前先把内存数据同步到 SD(避免丢数据)，再关闭 WiFi。
void enterDeepSleepIfNeeded() {
  // 1. 最后一次同步内存数据到 SD（防止深睡丢数据；写失败也无所谓）
  webLogln("😴 电量过低，进入深睡保护，请插上电源唤醒...");
  // flush 需要的数据：把当前 buffer 落盘（logData 已实时写 /log.csv，这里再归档一次保底）
  // 若已到归档时机则正常归档
  archivePump();       // 归档分片泵：每圈最多 40 行，**不阻塞采样**（2026-10-02）
  checkAndArchive();
  delay(100);

  // 2. 关闭 WiFi / AP（深睡下射频无需保活）
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(50);

  // 3. 配置唤醒源：PGOOD 低电平唤醒（插电）
  //    ⚠️ 2026-10-03 修复：原写死 `GPIO_NUM_2`（PGOOD 的**旧脚**）。
  //       v2.2 引脚迁移后 PGOOD = IO10，而这一行没跟着改 →
  //       **设备一旦深睡就监听一个空脚，永远醒不过来**（实测：拔电池黑屏后插 USB 无反应）。
  //       现在改用 `PIN_PGOOD` 宏，引脚迁移时不会再漏。
  //    ext0 唤醒要求 GPIO 属于 RTC 域且支持深睡唤醒；IO10 可作 RTC GPIO。
  //    注意：此内核版本(3.3.9-cn)里第二个参数用数字电平(0=低电平触发)，不是枚举名。
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_PGOOD, 0);   // 0 = 低电平触发唤醒

  // 4. 进入深睡
  esp_deep_sleep_start();   // 不会返回，直到被唤醒（调用后重启，从 setup() 重新开始）
}

// ===== 日平均引擎 =====
#define AVG_SLOTS 2880            // 30秒一格 × 24h = 2880 槽
#define AVG_WINDOW_DAYS 30        // 30天滑动窗口（含季节特征）
#define AVG_DIR "/avg"           // 每日均值 bin 存放目录
// ======================

// ===== 绝对湿度换算（V2.1.1 · 全项目唯一口径）=====
//   为什么：相对湿度(RH)会随温度摆动 —— 温度升高时 RH 自动下降,哪怕水汽一点没变。
//   所以"湿度变率"用 RH 会误导;绝对湿度(AH = 水汽质量浓度)才是水汽的真实增减。
//   ⚠️ 本函数是**唯一**口径：网页曲线 / 网页 24h 变率 / 屏幕 24h 变率 三处都调它。
//      三处各自实现必然分叉 —— 同一天同一个数会在三个地方不一样。
//   公式（Magnus/Tetens 饱和水汽压 + 理想气体换算）：
//     es(T) = 6.112 · exp(17.67·T / (T + 243.5))          [hPa]
//     AH    = 216.7 · (RH/100) · es(T) / (T + 273.15)     [g/m³]
//   自检：T=25°C / RH=50% → 11.5 g/m³
//   任一入参 NaN → 返回 NAN（下游一律按"无数据"处理,不造虚拟值）
float absHumidity(float tC, float rh) {
  if (isnan(tC) || isnan(rh)) return NAN;
  float es = 6.112f * expf(17.67f * tC / (tC + 243.5f));
  return 216.7f * (rh / 100.0f) * es / (tC + 273.15f);
}
// ==================================================

// ===== 整点变率环（24h，三要素，仅 RAM，不落盘）=====
//   下标 = 小时(0..23)，槽内存放「上一个同整点」的值
//   变率 = 当前整点值 − 槽内旧值（即 24h 前同一整点）
//   湿度那一档用**绝对湿度**：由槽内原始 T/RH 现算（absHumidity），
//   环里仍存原始 T/RH → **环的结构与落盘格式不变**
//   掉电仅因电池耗尽的场合才会丢失；重启后需重新积累 24h 才有变率
struct HourSlot {
  float t, h, p;
  bool  valid;
};
HourSlot hourRing[24] = {};
float trendT24 = NAN, trendH24 = NAN, trendP24 = NAN;   // 最近整点算出的 24h 变率
// ==================================================

// ==================== 传感器对象 ====================
Adafruit_BMP5xx bmp;
Adafruit_SHT31 sht;
RTC_DS3231 rtc;

// ==================================================

WebServer server(80);
unsigned long recordCount = 0;
bool bmpOK = false, shtOK = false;
bool sdOK = false, rtcOK = false;
bool rtcLostPower = false;   // DS3231 是否掉过电（掉电则时间不可信，需 NTP）
bool timeSynced = false;
bool hasSyncedOnce = false;

struct DataPoint {
  uint32_t time;
  float temp;
  float humidity;
  float pressure;
};

// ===== 日平均引擎数据结构（全部放 PSRAM） =====
struct AvgSlot {
  float sumT, sumH, sumP;
  uint16_t cnt;            // 参与天数
};
struct ExtremeValue {
  float v; uint32_t t; bool set;
};
struct DayExtremes {
  ExtremeValue tempMax, tempMin, humMax, humMin, presMax, presMin;
  char date[11];
};

AvgSlot* avgSlots = NULL;      // 30天窗口历史累计 (34.5KB)
AvgSlot* todaySlots = NULL;    // 今日实时累计   (34.5KB)
DayExtremes todayExt;
bool apEnabled = false;        // AP 当前是否开启
unsigned long apClosedMs = 0;  // AP 关闭时间（断线计时用）
// ================================================

DataPoint* buffer = NULL;
int bufferHead = 0, bufferSize = 0, bufferCapacity = 0;
int dayMarkIndex = -1;
bool fileBusy = false;
time_t lastCollectTime = 0;
unsigned long fallbackLastMs = 0;

// ========== 归档追踪 ==========
int archivedMonth = -1, archivedDay = -1;

// ========== 卡尔曼滤波 ==========
struct KalmanFilter {
  float Q, R, P, K, X;
  bool initialized;
  KalmanFilter(float q = 0.01, float r = 0.1) {
    Q = q; R = r; P = 1.0; K = 0.0; X = 0.0; initialized = false;
  }
  float update(float measurement) {
    if (!initialized) { X = measurement; P = 1.0; initialized = true; return X; }
    P = P + Q; K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P; return X;
  }
  void reset() { initialized = false; P = 1.0; }
};

void applyFilterToData(DataPoint* data, int count, bool resetFilter = true) {
  if (!data || count <= 0) return;
  static KalmanFilter tempFilter(0.05, 0.15);
  static KalmanFilter humFilter(0.1, 0.2);
  static KalmanFilter presFilter(0.02, 0.1);
  if (resetFilter) {
    tempFilter = KalmanFilter(0.05, 0.15);
    humFilter = KalmanFilter(0.1, 0.2);
    presFilter = KalmanFilter(0.02, 0.1);
  }
  for (int i = 0; i < count; i++) {
    if (!isnan(data[i].temp))     data[i].temp     = tempFilter.update(data[i].temp);
    if (!isnan(data[i].humidity)) data[i].humidity = humFilter.update(data[i].humidity);
    if (!isnan(data[i].pressure)) data[i].pressure = presFilter.update(data[i].pressure);
  }
}

// ========== 文件操作 ==========
// 写盘失败的可见化：以前是静默 return false，出问题时网页/下载看起来一切正常（实测踩到）
void warnAppendFail(bool openFail) {
  static bool warned = false;
  if (warned) return;
  warned = true;
  webLogln("⚠️ Flash 写日志失败（%s）→ /log.csv 会停在旧数据！可能是文件系统满/损坏，建议 /clear 或重启",
           openFail ? "打开失败" : "写入失败");
}
bool safeAppendFile(const char* path, const String& line) {
  if (fileBusy) return false;
  fileBusy = true;
  File file = LittleFS.open(path, "a");
  if (!file) { fileBusy = false; warnAppendFail(true); return false; }
  bool ok = file.println(line);
  file.close();
  fileBusy = false;
  if (!ok) warnAppendFail(false);
  return ok;
}

bool hasValidDate(uint32_t timestamp) {
  if (timestamp < 1000000000) return false;
  struct tm tm; time_t tt = timestamp;
  localtime_r(&tt, &tm);
  return (tm.tm_year >= 100 && tm.tm_year <= 200 &&
          tm.tm_mon >= 0 && tm.tm_mon <= 11 &&
          tm.tm_mday >= 1 && tm.tm_mday <= 31);
}

// ---- NaN 存取辅助：Flash 里的 "N/A" ↔ 内存里的 NAN ----
static String fmtVal(float v) { return isnan(v) ? String("N/A") : String(v, 2); }
static float  parseVal(const String& s) {
  if (s.length() == 0) return NAN;
  char c = s.charAt(0);
  if (c == 'N' || c == 'n') return NAN;      // "N/A" / "nan"
  return s.toFloat();
}
static String jsonNum(float v) { return isnan(v) ? String("null") : String(v, 2); }

String urlEncode(const String& str) {
  String encoded = "";
  for (size_t i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      encoded += c;
    } else {
      char hex[4];
      snprintf(hex, sizeof(hex), "%%%02X", (unsigned char)c);
      encoded += hex;
    }
  }
  return encoded;
}

bool writeLineToSDByDate(const String& line, const String& header,
                         int year, int mon, int day,
                         bool isManual, const String& manualSuffix) {
  if (!sdOK) return false;
  char basePath[32];
  snprintf(basePath, sizeof(basePath), "/%04d-%02d-%02d", year, mon, day);
  String fullPath = String(basePath);
  if (isManual) fullPath += manualSuffix;
  fullPath += ".csv";
  if (!SD.exists(fullPath.c_str())) {
    File f = SD.open(fullPath.c_str(), "w");
    if (!f) return false;
    f.println(header);
    f.close();
  }
  File f = SD.open(fullPath.c_str(), "a");
  if (!f) return false;
  bool ok = f.println(line);
  f.close();
  return ok;
}

// ===== 按需挂载 SD（2026-09-30）=====
// 背景：sdOK 以前只在 setup() 里由 initSD() 判定一次（initSD 全文件只调那一处），
//   所以**运行中插卡不会被发现** —— 必须重启才认，连网页上手动点「归档」也会失败。
// 做法：把"一次性判定"改成"用之前先试一次"，只在**真正要碰卡的三个入口**调用：
//   ① archiveToSD()（每日 02:00 自动归档 / 低电归档 / 网页手动归档）
//   ② /sdlist（网页列 SD 目录）  ③ SD 文件下载
//   插卡后第一次用到就自动认出来，不必重启。
// ⚠️ 只在 !sdOK 时探测；已就绪直接返回，不重复挂载。
// ⚠️ 总线是分开的：SD 走 sdSPI(HSPI)（引脚 5/4/6/7 = HSPI 原生），屏走默认 SPI(FSPI)（引脚 1/2 经 GPIO 矩阵）→
//   运行中重新挂载 SD 不会打扰屏幕。
// ⚠️ SD.begin() 是**阻塞**的（正常约 100~300ms，坏卡可能到秒级）→ 绝不能放进 30s 采样路径，
//   只放在上面那三个低频/人工入口。
// 前置声明：initSD() 的实现要到加载缓冲那段（约 2700 行）才出现。
//   Arduino IDE 会自动生成原型，但本工程的直连工具链构建（_build/gen-protos.js）
//   只在【最后一个类型定义之后】插原型 → 这里必须自己声明。
void initSD();

bool ensureSD() {
  if (sdOK) return true;
  initSD();                    // 内部会打「✅ SD卡已就绪」或「⚠️ SD卡初始化失败」
  return sdOK;
}

// ========== 归档函数 ==========
// ===== 归档（异步分片版，2026-10-02）=====
//   背景：原 archiveToSD() 是**一次性大循环**（读整个 /log.csv + 逐行写 SD + 重载缓冲）→
//        实测 02:00 归档把主循环**阻塞了约 109 秒**，丢掉 4 个采样点（含整点 02:00:00）。
//   改法：拆成"每圈只处理若干行"的泵（与 /abtest 自卸同一模式）→ 采样/浅睡照常进行。
//   注意：**不加 delay()**，靠"每圈有限行数"自然让出 CPU。
// 归档状态机（分片泵用；定义必须在使用它的 archiveStart 之前）
struct ArchiveState {
  bool           active     = false;
  bool           isManual   = false;
  File           in, tmp;
  String         header;
  String         manualSuffix;
  int            nowYear=0, nowMon=0, nowDay=0;
  unsigned long  todayCount = 0, archivedCount = 0;
  size_t         snapshotBytes = 0;    // 归档启动时 /log.csv 的字节数（用于收尾补读新增行）
} g_arch;
// 启动异步归档：准备文件与状态。返回 true = 已启动（之后由 archivePump 分片推进）
bool archiveStart(bool isManual = false) {
  if (g_arch.active) { webLogln("📦 归档进行中，忽略重复请求"); return false; }
  if (!ensureSD()) {                 // 先试着重挂一次：可能是运行中才插的卡（合并自 feat/wide-layout）
    webLogln("⚠️ SD卡未就绪（已尝试重新挂载），跳过归档");
    return false;
  }
  time_t now = time(nullptr);
  if (now < 1000000000) { webLogln("⚠️ 时间未同步，跳过归档"); return false; }
  struct tm tm_now; localtime_r(&now, &tm_now);
  g_arch.nowYear = tm_now.tm_year + 1900;
  g_arch.nowMon  = tm_now.tm_mon + 1;
  g_arch.nowDay  = tm_now.tm_mday;
  if (!isManual && archivedMonth == g_arch.nowMon && archivedDay == g_arch.nowDay) {
    webLogln("📦 今日已自动归档，跳过");
    return false;
  }
  LittleFS.remove("/log.tmp");                  // 清掉上次残留，保证本次从空文件开始
  g_arch.in = LittleFS.open("/log.csv", "r");
  if (!g_arch.in) { webLogln("❌ 无法打开 log.csv"); return false; }
  g_arch.header = g_arch.in.readStringUntil('\n');
  g_arch.snapshotBytes = g_arch.in.position();   // 快照位置：表头之后 = 已有数据的起点
  g_arch.tmp = LittleFS.open("/log.tmp", "a");   // a：补读新增行时追加不截断（首次由 remove 保证干净）
  if (!g_arch.tmp) { g_arch.in.close(); webLogln("❌ 无法创建临时文件"); return false; }
  g_arch.tmp.println(g_arch.header);
  g_arch.manualSuffix = "";
  if (isManual) {
    char suffix[64];
    snprintf(suffix, sizeof(suffix), "_manual_%04d%02d%02d_%02d%02d%02d",
             g_arch.nowYear, g_arch.nowMon, g_arch.nowDay, tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);
    g_arch.manualSuffix = String(suffix);
  }
  g_arch.isManual    = isManual;
  g_arch.todayCount  = 0;
  g_arch.archivedCount = 0;
  g_arch.active      = true;
  webLog("📦 开始%s归档（分片，不阻塞采样）...\n", isManual ? "手动" : "自动");
  return true;
}


// 处理一行：返回 true = 已写入 SD（归档成功），false = 留在 Flash
bool archProcessLine(const String& line) {
  if (line.length() == 0) return false;
  int idx1 = line.indexOf(',');
  int idx2 = line.indexOf(',', idx1 + 1);
  if (idx1 < 0 || idx2 < 0) { g_arch.tmp.println(line); g_arch.todayCount++; return false; }
  String timeStr = line.substring(idx1 + 1, idx2);
  struct tm tm_line; memset(&tm_line, 0, sizeof(tm_line));
  if (!strptime(timeStr.c_str(), "%Y-%m-%d %H:%M:%S", &tm_line)) {
    g_arch.tmp.println(line); g_arch.todayCount++; return false;
  }
  int lineYear = tm_line.tm_year + 1900;
  int lineMon  = tm_line.tm_mon + 1;
  int lineDay  = tm_line.tm_mday;
  bool isToday = (lineYear == g_arch.nowYear && lineMon == g_arch.nowMon && lineDay == g_arch.nowDay);

  if (isToday && !g_arch.isManual) {                  // 自动归档：今日数据留 Flash
    g_arch.tmp.println(line); g_arch.todayCount++; return false;
  }
  bool ok = writeLineToSDByDate(line, g_arch.header, lineYear, lineMon, lineDay,
                                isToday && g_arch.isManual, g_arch.manualSuffix);
  if (ok) { g_arch.archivedCount++; return true; }
  g_arch.tmp.println(line); g_arch.todayCount++;
  return false;
}

// 收尾：切换文件、重载缓冲
void archFinish() {
  // ★ 先补读"归档期间新追加的行"：
  //   异步归档期间主循环仍在往 /log.csv 追加采样；若直接 rename，这些新行会随旧文件被删。
  //   做法：重新打开原文件，从快照位置读到**当前末尾**（读到 EOF 即停），逐行按同一规则处理。
  int tail = 0;
  {
    File f2 = LittleFS.open("/log.csv", "r");
    if (f2) {
      f2.seek(g_arch.snapshotBytes);
      while (f2.available() && tail < 200) {            // 上限保护
        String line = f2.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) continue;
        archProcessLine(line);                          // /tmp 以 "a" 打开，追加不截断
        tail++;
      }
      f2.close();
    }
  }
  g_arch.in.close();
  g_arch.tmp.close();
  if (tail) webLog("📥 归档期间新增 %d 行已补入归档\n", tail);
  webLog("📊 归档统计: 归档 %lu 条, Flash保留 %lu 条\n", g_arch.archivedCount, g_arch.todayCount);
  if (g_arch.archivedCount == 0) {
    webLogln("⚠️ 没有数据需要归档，Flash不动");
    LittleFS.remove("/log.tmp");
    if (!g_arch.isManual) { archivedMonth = g_arch.nowMon; archivedDay = g_arch.nowDay; }
  } else {
    LittleFS.remove("/log.csv");
    LittleFS.rename("/log.tmp", "/log.csv");
    recordCount = g_arch.todayCount;
    if (!g_arch.isManual) { archivedMonth = g_arch.nowMon; archivedDay = g_arch.nowDay; }
    bufferSize = 0; bufferHead = 0;
    loadBufferFromFlash();
    webLog("✅ 归档完成！Flash 剩余 %lu 条\n", g_arch.todayCount);
  }
  g_arch.active = false;
}

// 每圈调用；每圈最多处理 ARCH_LINES_PER_TICK 行 → 让出 CPU（采样/浅睡不受阻）
void archivePump() {
  if (!g_arch.active) return;
  const int ARCH_LINES_PER_TICK = 40;
  int n = 0;
  while (n < ARCH_LINES_PER_TICK && g_arch.in.available()) {
    String line = g_arch.in.readStringUntil('\n');
    if (line.length() == 0) continue;
    archProcessLine(line);
    n++;
  }
  if (!g_arch.in.available()) archFinish();          // 读完了 → 收尾
}

// 同步入口（保留给"手动归档"等需要立即完成的场合）——内部循环推进分片泵
bool archiveToSDSync(bool isManual = false) {
  if (!archiveStart(isManual)) return false;
  unsigned long t0 = millis();
  while (g_arch.active && (millis() - t0) < 120000UL) archivePump();   // 上限 120s 防死循环
  return !g_arch.active;
}
// ⚠️ **会阻塞主循环**（内部同步跑完分片泵）——仅在"确实必须立即完成"时使用。
//    常规路径请用 archiveStart()（非阻塞，由 loop 的 archivePump 推进）。
//    目前代码里已无调用者，保留仅为兼容与应急。
bool archiveToSD(bool isManual = false) {
  if (g_arch.active) { webLogln("📦 归档进行中，忽略"); return false; }
  return archiveToSDSync(isManual);
}

// ========== 日平均引擎 ==========
void updateExtreme(ExtremeValue& ev, float v, uint32_t t, bool isMin = false) {
  if (!ev.set) { ev.set = true; ev.v = v; ev.t = t; return; }
  if (isMin ? (v < ev.v) : (v > ev.v)) { ev.v = v; ev.t = t; }
}

void addDaysToDate(const char* date, int days, char* out) {
  struct tm tm; memset(&tm, 0, sizeof(tm));
  sscanf(date, "%d-%d-%d", &tm.tm_year, &tm.tm_mon, &tm.tm_mday);
  tm.tm_year -= 1900; tm.tm_mon -= 1;
  time_t t = mktime(&tm) + (time_t)days * 86400L;
  localtime_r(&t, &tm);
  strftime(out, 11, "%Y-%m-%d", &tm);
}

// 判定一条“日平均”是否有效：本次修复后空槽写 NaN；修复前空槽写 0（真实读数不可能是 0），
// 两者都当“当天该槽没采集”——否则会把没数据的槽也算作一天参与平均，把基线稀释（曾把 1012 hPa 算成 507）
static inline bool avgDateValid(const float* v) {
  return !isnan(v[0]) && !isnan(v[1]) && !isnan(v[2]) && v[0] != 0.0f && v[1] != 0.0f && v[2] != 0.0f;
}

// 把 todaySlots 的均值写成 YYYY-MM-DD.bin（2880×3 float = 34.5KB）
void saveAvgBin(const char* date) {
  if (!avgSlots || !todaySlots) return;
  if (!LittleFS.exists(AVG_DIR)) LittleFS.mkdir(AVG_DIR);
  String path = String(AVG_DIR) + "/" + date + ".bin";
  File f = LittleFS.open(path, "w");
  if (!f) { webLogln("⚠️ 无法写入日平均文件"); return; }
  for (int i = 0; i < AVG_SLOTS; i++) {
    float v[3] = {NAN, NAN, NAN};      // 该槽当天无采集 → 写 NaN（读取端据此跳过，不参与平均）
    if (todaySlots[i].cnt > 0) {
      v[0] = todaySlots[i].sumT / todaySlots[i].cnt;
      v[1] = todaySlots[i].sumH / todaySlots[i].cnt;
      v[2] = todaySlots[i].sumP / todaySlots[i].cnt;
    }
    f.write((const uint8_t*)v, sizeof(v));
  }
  f.close();
  webLogln("💾 日平均已存档: %s", date);
}

// 滑动窗口：把昨天的均值加进 avgSlots，踢出 30 天前的
void updateAvgWindow(const char* date) {
  if (!avgSlots) return;
  String path = String(AVG_DIR) + "/" + date + ".bin";
  File f = LittleFS.open(path, "r");
  if (f) {
    float v[3];
    for (int i = 0; i < AVG_SLOTS; i++) {
      if (f.read((uint8_t*)v, sizeof(v)) == sizeof(v) && avgDateValid(v)) {
        avgSlots[i].sumT += v[0]; avgSlots[i].sumH += v[1]; avgSlots[i].sumP += v[2];
        avgSlots[i].cnt++;
      }
    }
    f.close();
  }
  char oldDate[11];
  addDaysToDate(date, -AVG_WINDOW_DAYS, oldDate);
  String oldPath = String(AVG_DIR) + "/" + oldDate + ".bin";
  if (LittleFS.exists(oldPath)) {
    File f2 = LittleFS.open(oldPath, "r");
    if (f2) {
      float v[3];
      for (int i = 0; i < AVG_SLOTS; i++) {
        if (f2.read((uint8_t*)v, sizeof(v)) == sizeof(v) && avgDateValid(v)) {
          avgSlots[i].sumT -= v[0]; avgSlots[i].sumH -= v[1]; avgSlots[i].sumP -= v[2];
          if (avgSlots[i].cnt > 0) avgSlots[i].cnt--;
        }
      }
      f2.close();
    }
    LittleFS.remove(oldPath);
    webLogln("🗑️ 已踢出窗口: %s", oldDate);
  }
}

// 跨天：存档昨天 → 更新滑动窗口 → 重置今日
void rolloverDay() {
  String oldDate = String(todayExt.date);
  if (oldDate.length() == 10) {
    saveAvgBin(oldDate.c_str());
    updateAvgWindow(oldDate.c_str());
  }
  memset(todaySlots, 0, AVG_SLOTS * sizeof(AvgSlot));
  memset(&todayExt, 0, sizeof(todayExt));
  struct tm tm; time_t now = time(nullptr); localtime_r(&now, &tm);
  snprintf(todayExt.date, sizeof(todayExt.date), "%04d-%02d-%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
  webLogln("🌅 跨天重置：%s 已计入日平均", oldDate.c_str());
}

// 开机时从已有 bin 重建滑动窗口（最多30天）
void loadAvgWindow() {
  if (!avgSlots) return;
  if (!LittleFS.exists(AVG_DIR)) return;
  File dir = LittleFS.open(AVG_DIR);
  if (!dir || !dir.isDirectory()) return;
  char names[40][11]; int n = 0;
  File f = dir.openNextFile();
  while (f && n < 40) {
    String fn = String(f.name());
    int slash = fn.lastIndexOf('/');
    String base = fn.substring(slash + 1);
    if (base.endsWith(".bin") && base.length() == 14) {
      strncpy(names[n], base.c_str(), 10); names[n][10] = '\0'; n++;
    }
    f = dir.openNextFile();
  }
  dir.close();
  if (n == 0) return;
  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (strcmp(names[j], names[i]) < 0) { char tmp[11]; strcpy(tmp, names[i]); strcpy(names[i], names[j]); strcpy(names[j], tmp); }
  int start = (n > AVG_WINDOW_DAYS) ? (n - AVG_WINDOW_DAYS) : 0;
  for (int k = start; k < n; k++) {
    String path = String(AVG_DIR) + "/" + names[k] + ".bin";
    File bf = LittleFS.open(path, "r");
    if (!bf) continue;
    float v[3];
    for (int i = 0; i < AVG_SLOTS; i++) {
      if (bf.read((uint8_t*)v, sizeof(v)) == sizeof(v) && avgDateValid(v)) {
        avgSlots[i].sumT += v[0]; avgSlots[i].sumH += v[1]; avgSlots[i].sumP += v[2];
        avgSlots[i].cnt++;
      }
    }
    bf.close();
  }
  webLogln("📚 日平均窗口加载: %d 天", n - start);
}

// PSRAM 分配（失败则禁用引擎，不影响主功能）
void initAvgEngine() {
  avgSlots = (AvgSlot*)ps_malloc(AVG_SLOTS * sizeof(AvgSlot));
  todaySlots = (AvgSlot*)ps_malloc(AVG_SLOTS * sizeof(AvgSlot));
  if (!avgSlots || !todaySlots) {
    webLogln("⚠️ PSRAM 分配失败，日平均引擎禁用（不影响基础功能）");
    return;
  }
  memset(avgSlots, 0, AVG_SLOTS * sizeof(AvgSlot));
  memset(todaySlots, 0, AVG_SLOTS * sizeof(AvgSlot));
  loadAvgWindow();
  webLogln("✅ 日平均引擎就绪 (PSRAM %d KB)", (AVG_SLOTS * (sizeof(AvgSlot) * 2)) / 1024);
}

// 凌晨 02:00–02:05 为自动归档窗口。
// ⚠️ loop() 每轮都会调用本函数：不加守卫时，窗口内每一轮都会打一条日志
//    （SD 正常打「今日已自动归档，跳过」，SD 异常打「SD卡未就绪，跳过归档」），
//    5 分钟足以刷爆 200 条网页日志环形缓冲，把有用日志挤掉。
//    策略：当天归档成功即停；失败（如 SD 未就绪）按 30s 间隔重试。
// ===== 未插卡的兜底：裁掉 /log.csv 里超过保留期的头部（2026-09-30）=====
// 为什么需要：不插 SD 时 archiveToSD() 永远失败 → /log.csv 只涨不清。littlefs 分区
//   3.38 MB，按 2880 条/天（≈138 KB/天）约 26 天填满；满后 safeAppendFile() 失败，
//   而那时唯一能腾空间的手段是 /clear（全清）。这里给一个「只保留最近 N 天」的兜底。
//
// ⚠️ 为什么按**时间戳**裁，而不是「砍掉最老的 2880 条」：低电压时 readAndLog() 会跳过
//   记录（allowRecord=false）、重启也会丢采样，一天的实际条数经常不到 2880。
//   按固定条数裁会净减（产出 2600 / 裁掉 2880 → 每天少 280 条）→ 越裁越少，
//   最后可能只剩不到一天。按时间戳裁则保留期恒等于 keepDays，且能自我收敛。
//
// ⚠️ 为什么只能整文件重写：littlefs 不能在文件中间打洞，所以只能是
//   读 → 过滤 → 写 /log.tmp → rename（与 archiveToSD() 同一套写法）。
//   代价：每次把剩余部分重写一遍（14 天 ≈ 2 MB）。按天裁 = 每天多写约 2 MB，
//   相对记账本身的 138 KB/天 是多 15 倍，但绝对值仍可忽略（约 365 次擦写/块·年）。
//
// ✅ 读取端**完全不用改**：还是同一个 /log.csv、同样的行格式，/history 仍按行数分页。
//
// 返回：true = 已处理（裁过了，或本来就没什么可裁）；false = 出错（交给调用方重试）
bool trimLogOlderThan(int keepDays) {
  time_t now = time(nullptr);
  if (now < 1000000000) return false;          // 时间没同步：宁可不裁，否则会把整份数据删光
  time_t cutoff = now - (time_t)keepDays * 86400L;

  if (fileBusy) return false;
  fileBusy = true;

  File in = LittleFS.open("/log.csv", "r");
  if (!in) { fileBusy = false; return false; }
  in.readStringUntil('\n');                     // 丢掉表头

  // 先只找第一条数据行的时间：还没到期就什么都不做（省掉一次整文件重写）
  time_t firstT = 0;
  while (in.available() && firstT == 0) {
    String l = in.readStringUntil('\n');
    if (!l.length()) continue;
    int i1 = l.indexOf(','), i2 = l.indexOf(',', i1 + 1);
    if (i1 < 0 || i2 <= i1) continue;
    struct tm tml;
    if (strptime(l.substring(i1 + 1, i2).c_str(), "%Y-%m-%d %H:%M:%S", &tml)) firstT = mktime(&tml);
  }
  if (firstT == 0 || firstT >= cutoff) {        // 解析不出时间 / 还没到保留期
    in.close(); fileBusy = false; return true;
  }

  // 确认要裁 → 从头重读，过滤进 /log.tmp
  in.seek(0);
  String header = in.readStringUntil('\n');
  File out = LittleFS.open("/log.tmp", "w");
  if (!out) { in.close(); fileBusy = false; return false; }
  out.println(header);
  unsigned long dropped = 0, kept = 0;
  while (in.available()) {
    String l = in.readStringUntil('\n');
    if (!l.length()) continue;
    int i1 = l.indexOf(','), i2 = l.indexOf(',', i1 + 1);
    time_t lt = 0;
    if (i1 >= 0 && i2 > i1) {
      struct tm tml;
      if (strptime(l.substring(i1 + 1, i2).c_str(), "%Y-%m-%d %H:%M:%S", &tml)) lt = mktime(&tml);
    }
    if (lt != 0 && lt < cutoff) { dropped++; continue; }
    out.println(l); kept++;                      // 时间戳解析不出的行一律保留（宁可留着也不丢）
  }
  in.close(); out.close();

  if (dropped == 0) { LittleFS.remove("/log.tmp"); fileBusy = false; return true; }

  LittleFS.remove("/log.csv");
  LittleFS.rename("/log.tmp", "/log.csv");
  recordCount = kept;                            // 与 archiveToSD() 同口径：= flash 里的条数
  bufferSize = 0; bufferHead = 0;
  loadBufferFromFlash();                         // 文件变了 → 刷新 RAM 环，别让 /history 与环对不上
  fileBusy = false;
  webLogln("🧹 未插卡兜底：裁掉 %lu 条（早于 %d 天前），Flash 保留 %lu 条", dropped, keepDays, kept);
  return true;
}

void checkAndArchive() {
  time_t now = time(nullptr);
  if (now < 1000000000) return;
  struct tm tm;
  localtime_r(&now, &tm);
  if (tm.tm_hour != 2 || tm.tm_min >= 5) return;

  static int           doneYday  = -1;
  static unsigned long lastTryMs = 0;

  if (tm.tm_yday == doneYday) return;                        // 当天已完成，不再判定
  if (g_arch.active) return;                                 // 正在分片归档中，别重复发起
  unsigned long nowMs = millis();
  if (lastTryMs != 0 && (nowMs - lastTryMs) < 30000UL) return;  // 失败重试间隔 30s
  lastTryMs = nowMs;
  // ⚠️ 2026-10-02 融合两处改动：
  //   ①（feat/light-sleep）改**非阻塞启动**：archiveStart 只准备文件，实际处理交给 loop 的 archivePump；
  //      原来调 archiveToSD() 会一次跑完整个大循环 → 阻塞主循环约 109 秒 → 丢 4 个采样点。
  //   ②（feat/wide-layout）先试着重挂载 SD：可能是运行中才插的卡；确实没卡 → 兜底裁掉超期表头。
  //      ⚠️ 这里必须自己收工（doneYday = …）：本函数在窗口内每 30s 重试一次，
  //         不设守卫的话裁头会连做多遍，等于把整个文件重写多遍。
  if (!ensureSD()) {
    if (trimLogOlderThan(LOG_KEEP_DAYS)) doneYday = tm.tm_yday;
    return;
  }
  if (archiveStart(false)) doneYday = tm.tm_yday;            // 非阻塞启动；当天只发起一次
}

// ========== 初始化各模块 ==========
// ---- 旧版 CSV 表头迁移（V2.1 新增第 6 列「模式」）----
//   老 /log.csv 是 5 列，直接追加新数据会出现「新旧列数不一致」。
//   这里一次性重写：表头换成 6 列，旧数据行的模式列补占位符 "-"（历史模式未知）。
//   表头已含「模式」则原样跳过。仅开机执行一次。
static void migrateLogHeaderIfNeeded() {
  File in = LittleFS.open("/log.csv", "r");
  if (!in) return;
  String header = in.readStringUntil('\n');
  if (header.indexOf("模式") >= 0) { in.close(); return; }   // 已是新格式
  webLogln("🔄 旧版 CSV 无「模式」列，开始迁移…");
  File out = LittleFS.open("/log.tmp", "w");
  if (!out) { in.close(); webLogln("⚠️ CSV 迁移失败：无法创建临时文件"); return; }
  out.println("序号,时间,温度(°C),湿度(%RH),气压(hPa),模式");
  unsigned long n = 0;
  while (in.available()) {
    String line = in.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int commas = 0;
    for (int i = 0; i < (int)line.length(); i++) if (line.charAt(i) == ',') commas++;
    out.print(line);
    if (commas < 5) out.print(",-");      // 只有 5 列 → 补占位符（模式未知）
    out.println();
    n++;
  }
  in.close();
  out.close();
  // 先备份再替换，避免中途掉电丢数据
  LittleFS.remove("/log.bak");
  LittleFS.rename("/log.csv", "/log.bak");
  if (!LittleFS.rename("/log.tmp", "/log.csv")) {
    LittleFS.rename("/log.bak", "/log.csv");   // 回滚
    webLogln("⚠️ CSV 迁移失败：替换出错，已回滚");
    return;
  }
  LittleFS.remove("/log.bak");
  webLogln("✅ CSV 迁移完成：%lu 行已补齐「模式」列（旧行填 \"-\"）", n);
}

void initFS() {
  if (!LittleFS.begin(true, "/littlefs", 10, "littlefs")) {  // ← true = 挂载失败自动格式化
    webLogln("❌ Flash 挂载失败");
    return;
  }
  fsReadyForDbg = true;   // ✅ 挂载成功后才允许落盘

  if (!LittleFS.exists("/log.csv")) {
    File file = LittleFS.open("/log.csv", "w");
    if (file) { file.println("序号,时间,温度(°C),湿度(%RH),气压(hPa),模式"); file.close(); }
  } else {
    migrateLogHeaderIfNeeded();     // 旧 5 列表头 → 6 列（旧行补 "-"）
    File file = LittleFS.open("/log.csv", "r");
    if (file) {
      recordCount = 0;
      while (file.available()) { if (file.read() == '\n') recordCount++; }
      file.close();
      if (recordCount > 0) recordCount--;
    }
  }
  bufferCapacity = KEEP_IN_MEMORY;
  buffer = new DataPoint[bufferCapacity];
  if (!buffer) { bufferCapacity = 300; buffer = new DataPoint[bufferCapacity]; }
  if (!buffer) { bufferCapacity = 100; buffer = new DataPoint[bufferCapacity]; }
  if (buffer) loadBufferFromFlash();
}

void loadBufferFromFlash() {
  if (!buffer) return;
  File file = LittleFS.open("/log.csv", "r");
  if (!file) return;
  file.readStringUntil('\n');
  unsigned long totalLines = 0;
  while (file.available()) { if (file.read() == '\n') totalLines++; }
  file.close();
  if (totalLines == 0) return;
  file = LittleFS.open("/log.csv", "r");
  file.readStringUntil('\n');
  unsigned long skipLines = (totalLines > KEEP_IN_MEMORY) ? (totalLines - KEEP_IN_MEMORY) : 0;
  for (unsigned long i = 0; i < skipLines; i++) file.readStringUntil('\n');
  bufferSize = 0; bufferHead = 0;
  int loadedCount = 0;
  while (file.available() && loadedCount < KEEP_IN_MEMORY) {
    String line = file.readStringUntil('\n');
    if (line.length() == 0) continue;
    int idx1 = line.indexOf(','), idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1), idx4 = line.indexOf(',', idx3 + 1);
    int idx5 = line.indexOf(',', idx4 + 1);   // 第6列=设备模式（旧数据可能没有）
    if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx4 < 0) continue;
    String timeStr = line.substring(idx1 + 1, idx2);
    String tempStr = line.substring(idx2 + 1, idx3);
    String humStr  = line.substring(idx3 + 1, idx4);
    String presStr = (idx5 < 0) ? line.substring(idx4 + 1) : line.substring(idx4 + 1, idx5);
    uint32_t timestamp = 0; struct tm tm;
    if (strptime(timeStr.c_str(), "%Y-%m-%d %H:%M:%S", &tm)) { timestamp = mktime(&tm); }
    else { int h,m,s; if (sscanf(timeStr.c_str(),"%d:%d:%d",&h,&m,&s)==3) timestamp = h*3600+m*60+s; else timestamp = loadedCount * INTERVAL_SEC; }
    buffer[bufferHead].time = timestamp;
    buffer[bufferHead].temp = parseVal(tempStr);
    buffer[bufferHead].humidity = parseVal(humStr);
    buffer[bufferHead].pressure = parseVal(presStr);
    bufferHead = (bufferHead + 1) % bufferCapacity;
    bufferSize++; loadedCount++;
  }
  file.close();
  if (bufferSize > 0) {
    int lastIdx = (bufferHead - 1 + bufferCapacity) % bufferCapacity;
    lastCollectTime = buffer[lastIdx].time;
  }
}

// ===== V2.1.1 Bug②：开机把「今天」已采的数据回灌进今日统计（极值 + 距平槽）=====
//   为什么：`todayExt`/`todaySlots` 只在实时采样路径里更新，OTA / 掉电 / 复位后当天已过去的数据全丢。
//   ⚠ 必须扫**整个 /log.csv**（不能只扫 RAM 那 600 条：600×30s ≈ 5h，盖不住一整天）。
//   ⚠ 必须在「系统时间已就绪」且**日平均引擎与跨天逻辑都已就位**之后调用（见 setup 里的调用点）。
//   成本：一次线性扫描（几千行 ≈ 几十 ms），只在开机做一次，不碰采样节奏、不写 SD。
//   返回：成功喂入的样本条数（-1 表示条件不满足没跑）
int backfillTodayFromLog() {
  if (!avgSlots || !todaySlots) return -1;                 // 引擎没起来（PSRAM 分配失败）
  if (!timeSynced) return -1;                              // 时间没同步，判不了"今天"
  time_t now = time(nullptr);
  if (now < 1000000000) return -1;
  char today[11];
  struct tm tmn; localtime_r(&now, &tmn);
  snprintf(today, sizeof(today), "%04d-%02d-%02d", tmn.tm_year + 1900, tmn.tm_mon + 1, tmn.tm_mday);
  // 先给 todayExt.date 打上"今天"（回灌期间不会有跨天，也**绝不能**触发 rolloverDay）：
  //   todayExt.date 为空 → feedTodaySample() 会直接返回，所以这里必须先设好
  if (strlen(todayExt.date) != 10) snprintf(todayExt.date, sizeof(todayExt.date), "%s", today);

  File file = LittleFS.open("/log.csv", "r");
  if (!file) { webLogln("ℹ️ 今日回灌：无 /log.csv，跳过"); return 0; }
  file.readStringUntil('\n');                              // 表头
  int fed = 0, skips = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) continue;
    int idx1 = line.indexOf(','), idx2 = line.indexOf(',', idx1 + 1);
    int idx3 = line.indexOf(',', idx2 + 1), idx4 = line.indexOf(',', idx3 + 1);
    if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx4 < 0) continue;
    String timeStr = line.substring(idx1 + 1, idx2);
    if (timeStr.length() < 10) continue;
    if (timeStr.substring(0, 10) != String(today)) continue;   // 只喂"今天"的（CSV 已按时间递增）
    uint32_t ts = 0; struct tm tmv;
    memset(&tmv, 0, sizeof(tmv));
    if (strptime(timeStr.c_str(), "%Y-%m-%d %H:%M:%S", &tmv)) ts = (uint32_t)mktime(&tmv);
    if (ts == 0) { skips++; continue; }
    float tC = parseVal(line.substring(idx2 + 1, idx3));
    float h  = parseVal(line.substring(idx3 + 1, idx4));
    float p  = parseVal(line.substring(idx4 + 1));             // 第 5 列；后面可能还有「模式」列，parseVal 会处理脏尾巴
    feedTodaySample(ts, tC, h, p);
    fed++;
  }
  file.close();
  webLogln("📥 今日回灌：/log.csv 命中 %s 共 %d 条（跳过时间戳异常 %d 条）→ 极值与距平槽已补齐",
           today, fed, skips);
  diag(String("今日回灌 ") + today + "：" + String(fed) + " 条");
  return fed;
}

void initSD() {
  sdSPI.begin(5, 4, 6, 7);      // v2.2：SCK=IO5 MISO=IO4 MOSI=IO6 CS=IO7
  if (!SD.begin(SD_CS, sdSPI)) {
    webLogln("⚠️ SD卡初始化失败");
    sdOK = false;
  } else {
    webLogln("✅ SD卡已就绪");
    sdOK = true;
  }
}

void initRTC() {
  if (!rtc.begin()) {
    webLogln("⚠️ DS3231 未检测到");
    rtcOK = false;
  } else {
    webLogln("✅ DS3231 RTC 已就绪");
    rtcOK = true;
    rtcLostPower = rtc.lostPower();
    if (rtcLostPower) webLogln("⚠️ RTC 掉电，需通过 NTP 设置时间");
  }
}

void cleanupUntimestampedData() {
  if (!timeSynced || !hasSyncedOnce) return;
  File file = LittleFS.open("/log.csv", "r");
  if (!file) return;
  File tmp = LittleFS.open("/log.tmp", "w");
  if (!tmp) { file.close(); return; }
  String header = file.readStringUntil('\n');
  tmp.println(header);
  int fileCleaned = 0, fileKept = 0;
  unsigned long newRecordCount = 0;
  while (file.available()) {
    String line = file.readStringUntil('\n');
    if (line.length() == 0) continue;
    int idx1 = line.indexOf(','), idx2 = line.indexOf(',', idx1 + 1);
    if (idx1 < 0 || idx2 < 0) continue;
    String timeStr = line.substring(idx1 + 1, idx2);
    struct tm tm;
    if (strptime(timeStr.c_str(), "%Y-%m-%d %H:%M:%S", &tm)) {
      tmp.println(line); fileKept++; newRecordCount++;
    } else { fileCleaned++; }
  }
  file.close(); tmp.close();
  if (fileCleaned > 0) {
    LittleFS.remove("/log.csv");
    LittleFS.rename("/log.tmp", "/log.csv");
    recordCount = newRecordCount;
  } else { LittleFS.remove("/log.tmp"); }
  hasSyncedOnce = false;
}

// ===== NTP：非阻塞启动 + 轮询完成（2026-10-03 重构）=====
//   背景：原 syncTimeFromNTP() 用 delay(250) 轮询最多 40 次 = **最多阻塞 10 秒**，
//        在启动路径上非常拖时间。
//   拆分：
//     · ntpStart()      —— 只调 configTime() 启动 SNTP，**立即返回**
//     · ntpIsDone()     —— 查系统时间是否已被 SNTP 拉正
//     · syncTimeFromNTPBlocking(ms) —— 需要"立刻拿到时间"时才用（如 RTC 掉电）
//   ⚠️ 完成后的落地动作（写 RTC / 记 ntpLastOkAt）统一在 serviceNtpSync() 里做。
bool ntpPending = false;          // SNTP 已启动、等结果
unsigned long ntpStartedMs = 0;

void ntpStart() {
  configTime(8 * 3600, 0, "pool.ntp.org", "ntp.aliyun.com");
  ntpPending   = true;
  ntpStartedMs = millis();
  webLogln("🕐 NTP 同步已启动（非阻塞）");
}

bool ntpIsDone() { return time(nullptr) >= 1000000000; }

// 需要立刻拿到时间时用（会阻塞）；返回是否成功
bool syncTimeFromNTPBlocking(unsigned long maxMs = 10000UL) {
  time_t now = 0;
  unsigned long t0 = millis();
  while ((now = time(nullptr)) < 1000000000) {
    if (millis() - t0 > maxMs) break;
    delay(100);
  }
  if (now >= 1000000000) {
    timeSynced = true; hasSyncedOnce = true;
    ntpLastOkAt = now;
    struct timeval tv = { now, 0 };
    settimeofday(&tv, nullptr);
    char buf[30]; strftime(buf, 30, "%Y-%m-%d %H:%M:%S", localtime(&now));
    webLog("✅ NTP: %s\n", buf);
    if (rtcOK) { rtc.adjust(DateTime(now)); webLogln("✅ RTC 已同步 NTP 时间"); }
    cleanupUntimestampedData();
    ntpPending = false;
    return true;
  }
  webLogln("⚠️ NTP 超时（改用 RTC 时间）");
  _fallbackToRtcTime();
  return false;
}

// 时间回退到 RTC（NTP 不可用时的保底）
void _fallbackToRtcTime() {
  if (!rtcOK) return;
  DateTime dt = rtc.now();
  time_t rtcTime = dt.unixtime();
  if (rtcTime >= 1000000000) {
    struct timeval tv = { rtcTime, 0 };
    settimeofday(&tv, nullptr);
    timeSynced = true; hasSyncedOnce = true;
    char buf[30]; strftime(buf, 30, "%Y-%m-%d %H:%M:%S", localtime(&rtcTime));
    webLog("✅ 从 RTC 获取时间: %s\n", buf);
    cleanupUntimestampedData();
  }
}

// ===== NTP 每日 02:00 定期检查（loop 每圈调，几乎零开销）=====
//   条件：插电 + WiFi 已连 + 当前时间已有效（否则没法判断"到 02:00 了吗"）。
//   动作：跨过 02:00 当天做一次判定；距上次成功 >= NTP_RESYNC_DAYS 天 → 非阻塞同步。
//   ⚠️ RTC 掉电且当时无网络 → 挂起（ntpDailyPending），一旦有网立刻补一次。
void serviceNtpDailyCheck() {
  // 时间有效性：若时间还没被设过（< 2021），先不动
  time_t nowT = time(nullptr);
  if (nowT < 1000000000) return;

  struct tm tmv;
  localtime_r(&nowT, &tmv);
  int  today    = tmv.tm_yday;
  bool atOrAfter2 = (tmv.tm_hour >= 2);

  // 每日只在跨过 02:00 后检查一次
  bool dueToday = atOrAfter2 && (ntpDailyLastDay != today);

  // RTC 掉电且没网 → 挂起
  if (rtcLostPower && WiFi.status() != WL_CONNECTED) { ntpDailyPending = true; return; }

  if (!dueToday && !(ntpDailyPending && WiFi.status() == WL_CONNECTED)) return;

  if (!isExternallyPowered()) return;      // 电池供电时绝不去连网校时（省电）
  if (WiFi.status() != WL_CONNECTED) return;

  bool stale = (ntpLastOkAt == 0) || ((nowT - ntpLastOkAt) >= NTP_RESYNC_DAYS * 24L * 3600L);
  if (!stale && !rtcLostPower && !ntpDailyPending) { ntpDailyLastDay = today; return; }

  ntpDailyLastDay = today;
  ntpDailyPending = false;
  webLogln("🕐 每日 NTP 检查：距上次成功已超 %ld 天 → 发起同步（非阻塞）", NTP_RESYNC_DAYS);
  ntpStart();
}
// loop 每圈调用：SNTP 有结果就落地（写 RTC、记时刻），超时就回退 RTC
void serviceNtpSync() {
  if (!ntpPending) return;
  if (ntpIsDone()) {
    time_t now = time(nullptr);
    timeSynced = true; hasSyncedOnce = true;
    ntpLastOkAt = now;
    if (rtcOK) { rtc.adjust(DateTime(now)); webLogln("✅ NTP 已同步（并写入 RTC）"); }
    else       { webLogln("✅ NTP 已同步"); }
    cleanupUntimestampedData();
    ntpPending = false;
  } else if (millis() - ntpStartedMs > 20000UL) {   // 20s 无结果 → 回退
    webLogln("⚠️ NTP 20 秒无结果 → 回退 RTC 时间");
    _fallbackToRtcTime();
    ntpPending = false;
  }
}
// 兼容旧调用：**会阻塞**（最多 10s）。新代码请用 ntpStart() / serviceNtpSync()。
void syncTimeFromNTPLegacy() {
  webLog("🕐 NTP 同步...");
  configTime(8 * 3600, 0, "pool.ntp.org", "ntp.aliyun.com");
  time_t now = 0;
  int ntpRetry = 0;
  while (now < 1000000000 && ntpRetry < 40) {
    delay(250); now = time(nullptr); ntpRetry++; webLog(".");
  }
  if (now >= 1000000000) {
    timeSynced = true; hasSyncedOnce = true;
  ntpLastOkAt = time(nullptr);                     // 记录 NTP 成功时刻（供 3 天重试判据用）
    struct timeval tv = { now, 0 };
    settimeofday(&tv, nullptr);
    char buf[30]; strftime(buf, 30, "%Y-%m-%d %H:%M:%S", localtime(&now));
    webLog("\n✅ NTP: %s\n", buf);
    if (rtcOK) { rtc.adjust(DateTime(now)); webLogln("✅ RTC 已同步 NTP 时间"); }
    cleanupUntimestampedData();
  } else {
    webLogln("\n⚠️ NTP 超时");
    if (rtcOK) {
      DateTime dt = rtc.now();
      time_t rtcTime = dt.unixtime();
      if (rtcTime >= 1000000000) {
        struct timeval tv = { rtcTime, 0 };
        settimeofday(&tv, nullptr);
        timeSynced = true; hasSyncedOnce = true;
        char buf[30]; strftime(buf, 30, "%Y-%m-%d %H:%M:%S", localtime(&rtcTime));
        webLog("✅ 从 RTC 获取时间: %s\n", buf);
        cleanupUntimestampedData();
      }
    }
  }
}

// ========== JSON 构建 ==========
String buildLatestJSON(int count, bool applyFilter = true) {
  if (!buffer || bufferSize == 0) return "[]";
  int getCount = (count < bufferSize) ? count : bufferSize;
  int idx = bufferHead - getCount;
  if (idx < 0) idx += bufferCapacity;
  DataPoint* displayData = new DataPoint[getCount];
  if (!displayData) return "[]";
  for (int i = 0; i < getCount; i++) displayData[i] = buffer[(idx + i) % bufferCapacity];
  if (applyFilter && getCount > 1) applyFilterToData(displayData, getCount);
  String json = "[";
  for (int i = 0; i < getCount; i++) {
    if (i > 0) json += ",";
    bool validDate = hasValidDate(displayData[i].time);
    json += "{\"t\":" + String(displayData[i].time) + ",\"tp\":" + jsonNum(displayData[i].temp) +
            ",\"hm\":" + jsonNum(displayData[i].humidity) + ",\"pr\":" + jsonNum(displayData[i].pressure) +
            ",\"ah\":" + jsonNum(absHumidity(displayData[i].temp, displayData[i].humidity)) +
            ",\"vd\":" + String(validDate ? 1 : 0);
    if (dayMarkIndex >= 0) {
      int realIdx = (idx + i) % bufferCapacity;
      if (realIdx == dayMarkIndex) json += ",\"dm\":1";
    }
    json += "}";
  }
  json += "]";
  delete[] displayData;
  return json;
}

// ========== ★★★ Web 服务器 ★★★ ==========
String fmtHM(uint32_t t) {
  if (!hasValidDate(t)) return "-";
  struct tm tm; time_t tt = t; localtime_r(&tt, &tm);
  char b[16]; snprintf(b, sizeof(b), "%02d:%02d", tm.tm_hour, tm.tm_min);
  return String(b);
}

// ==================== 设备模式（固定 / 移动）====================
//  0 = 固定(FIX)   1 = 移动(MOV)     存 NVS(Flash)，断电不丢
//  说明：模式最终还应驱动 BQ24074 的 CE 停充点 / IO18 的 ISET 电流切换（V2.1 硬件），
//        本版先把它作为「统一状态源 + 屏幕显示」，网页 /mode 可切换。
Preferences prefs;
uint8_t deviceMode = 0;          // 默认固定

// ==================== 设备一机一密（2026-10-05）====================
//   密钥**只存 NVS，绝不进固件** —— 这样 OTA 的 .bin 是"无秘密"的，
//   固件被谁下载走都不构成泄漏；而 Preferences(NVS) 分区独立于 app 分区，
//   **OTA 升级不会清掉密钥**，所以"内网绑一次、之后随便 OTA"成立。
//
//   签名式必须与服务器 PHT_SERVER/devicekey.py 完全一致：
//     待签串 = "v1|" + deviceId + "|" + ts + "|" + body
//     头     = X-PHT-Auth: v1|<deviceId>|<ts>|<hex HMAC-SHA256>
char     devKeySecret[65] = "";   // 64 hex + NUL；空 = 未绑定 → 回退旧静态 token
char     devKeyId[33]     = "";   // 32 hex 公开标识（非凭据）
uint32_t devKeySetAt      = 0;
bool     devKeySelfTestOk = false;   // pht_crypto 自检结果（开机跑一次）

// 是否已绑定密钥
bool devKeyBound() { return devKeySecret[0] != '\0'; }

void loadDeviceMode() {
  prefs.begin("pht", true);
  deviceMode = prefs.getUChar("mode", 0);
  prefs.end();
  if (deviceMode > 1) deviceMode = 0;
}
void saveDeviceMode(uint8_t m) {
  deviceMode = (m == 1) ? 1 : 0;
  prefs.begin("pht", false);
  prefs.putUChar("mode", deviceMode);
  prefs.end();
}

// ==================== 推送（v2.2，局域网先行）====================
//   设计依据：docs/推送功能设计.md；实现清单 docs/ESP32推送实现清单.md
//   本段只做【配置与状态】；采集/发送逻辑在后面。
//
//   ⚠️ 用户已确认的决策（勿擅自改）：
//     · 设备 ID 网页可配，但【AP 模式禁止改名】= 未连 WiFi 一律拒绝
//     · 改名时必须查服务器占用，【服务器不可达也拒绝】—— 必须确保不重名
//       （重名两台设备数据会混在一起，且服务器幂等去重 → 数据上根本看不出异常）
//     · 推送约 15 分钟一次，多设备按 MAC 抖动错开
//     · 电池供电不推（恰好与 wantWireless() 的"移动+电池全关无线"一致）
//     · 插电时立刻检查并尝试同步
//     · 丢 2~3 个点可接受，不追求绝对完整
#define PUSH_INTERVAL_SEC     900       // 15 分钟
#define PUSH_JITTER_MAX_SEC   120       // MAC 派生抖动上限，多设备天然错开
#define PUSH_BATCH_MAX        200       // 每批最多条数（服务器上限 20000，留足余量）
#define PUSH_CHUNK_GAP_MS     2000      // 批间让步，压低 WiFi 占空比（关键：不影响采样）
#define PUSH_HTTP_TIMEOUT_MS  8000
#define PUSH_SRV_HOST_DEF     "192.168.1.100"
#define PUSH_SRV_PORT_DEF     8080
#define PUSH_API_PATH         "/api/v1/samples"
#define PUSH_RETRY_MIN_SEC    30        // 失败退避起点（30 → 60 → … → 上限）
#define PUSH_RETRY_MAX_SEC    600       // 失败退避上限 10 分钟
#define PUSH_TASK_STACK       8192      // HTTPClient 在这个任务的栈上干活（实测够）
#define PUSH_TASK_PRIO        1         // 与 loop 任务同优先级：靠 tick 时间片轮转，既不抢占也不饿死
#define PUSH_FW_VER           "2.2.0-dev"   // 随推送上报，便于服务器分辨固件代次

// ⚠️ token 绝不写进源码（仓库公开）。构建时以宏注入：
//    构建脚本写进 build_opt.h：-DPHT_API_TOKEN_RAW=<64 位十六进制>
//    未注入时为空 → 不发该请求头（仅在服务器 token 也留空时可用）
// 注意用【两层字符串化】：token 是 0e27... 这种以数字开头的十六进制串，
//   直接写 -DPHT_API_TOKEN="0e27..." 有时会被 gcc 的 @file 参数文件吞掉引号，
//   变成"数字字面量后缀"报错。用 RAW + 字符串化强制按标识符解析，只有纯字母
//   数字的 token 才能这样用（我们的 token 正好满足）。
#ifndef PHT_API_TOKEN_RAW
#define PHT_API_TOKEN_RAW
#endif
#define PHT_STR2(x) #x
#define PHT_STR(x)  PHT_STR2(x)
#define PHT_API_TOKEN PHT_STR(PHT_API_TOKEN_RAW)

bool     pushEnabled    = true;
// ⚠️ **身份与显示名是两回事**（2026-10-05 拆分）：
//   pushDevId   = 身份。eFuse MAC 派生，**终身不变**。进签名串、注册 URL、
//                 推送 body 的 device 字段、服务器样本归属。**永远不要改它。**
//   pushDevName = 显示名。给人看的，可改、可中文、可与别人重名。
//                 只进 NVS、改名请求、和本地网页标题；**不进签名、不进推送 body**。
char     pushDevId[33]    = {0};
char     pushDevName[104] = {0};         // UTF-8，≤24 码点（最多 4 字节/码点）
// 最近一次从服务器响应头 X-PHT-Now 读到的时间（0 = 从未读到）。
//   ⚠️ 暴露它是为了**可观测**：否则"头没读到"这种静默失效根本看不出来。
uint32_t pushSrvClock = 0;
bool     pushNoNowHdrWarned = false;
char     pushSrvHost[64] = PUSH_SRV_HOST_DEF;
uint16_t pushSrvPort    = PUSH_SRV_PORT_DEF;
uint32_t lastSyncedTs   = 0;             // 已成功上传的最大 ts（NVS）
// ⚠️ 游标"是否还有效"的标记（也存 NVS）。
//   为什么需要它：lastSyncedTs 是**绝对时间戳**，而数据源只有 RAM 环（约 5 小时）。
//   重启后如果环已经被新数据覆盖，这个旧游标会**大于环里最新点** → 待传数算出来是 0，
//   于是整批积压永远不会被推上去（2026-10-05 实测踩到：OTA 重启后 pending 卡在 0/1，passes 不涨）。
//   规则：只有【环里被完全推干净】时才置 true —— 此时游标确实是"下一个要推的点"；
//         一旦被夹紧（说明中间有空档）就置 false，重启后必须重新夹一次。
bool     pushCursorValid = false;
volatile bool pushLastOk = false;        // 最近一次发送是否成功
uint32_t pushLastTryMs = 0, pushLastOkMs = 0;
uint16_t pushFailStreak = 0;
bool     pushConfigLoaded = false;

// ---- 第二阶段（发送）运行时状态 ----
TaskHandle_t pushTaskHandle   = nullptr;   // 独立推送任务（HTTP 只在这里阻塞）
volatile bool pushTickForce   = false;     // 主循环 → 任务：立刻试一轮（手动/插电/首次联网）
uint32_t pushNextAtMs         = 0;         // 下一轮同步时刻（millis 基准；0 = 尚未排程）
uint32_t pushLastUploadTs     = 0;         // 最近一批成功上传的最大 ts
uint16_t pushLastAccepted     = 0;         // 最近一批服务器新入库条数
uint32_t pushLastUploadMs     = 0;
bool     pushNeedBackfill     = false;     // 服务器提示有更早的缺口（第二阶段 CSV/SD 补传用）
uint32_t pushBackfillBeforeTs = 0;
uint8_t  pushRunState         = 0;         // 0=空闲 1=发送中 2=退避等待
volatile bool pushCancel      = false;     // ⚠️ 预留位：目前【无人置位】。实际收手靠 WiFi.status()!=WL_CONNECTED 与 pushEnabled 两道检查（实测够用）；要在标定/断无线时立刻收手，在这里置位即可
// ---- 第三阶段：历史补传（SD 归档 / log.csv）。见 pushBackfillTick() ----
//   为什么不需要"RAM 下限"：数据按【牌组】组织，一个牌组内部天然按 ts 递增，
//   而且必须走完旧牌组才轮到新牌组 —— 顺序保证了，就不会跳过空洞。
uint32_t pushFlashCursor = 0;      // flash 补传游标（NVS 键 flashSync）
bool     pushFlashDone   = false;  // flash 已补到尽头（NVS 键 flashDone）
uint32_t pushFlashSent   = 0;      // 本次开机补传已发条数
// ---- SD 归档补传（第三阶段下半段：/2026-10-XX.csv，每天一个，冻结）----
//   时间上与 /log.csv **不重叠**（归档只写过去的整天，log.csv 是最近几天）。
//   两个牌组各用各的游标，按"从旧到新"顺序补，就不会留下空洞。
bool     pushSdDone   = false;     // SD 归档是否已补完（NVS 键 sdDone）
char     pushSdCur[24] = "";       // 正在补的文件名（NVS 键 sdFile）
uint32_t pushSdCursor = 0;         // 该文件里已推到的 ts（NVS 键 sdSync）
uint32_t pushSdSent   = 0;         // 本次开机 SD 补传已发条数
volatile bool pushPowerWasExt = false;     // 插电触发用：上一轮是否外接电源
uint32_t pushLastBootPushMs   = 0;         // 本会话首次「有无线」的时刻（首推基准）
uint8_t  pushPassCount        = 0;         // 本会话已完成的同步轮次（/status 观察用）

// 设备 **ID**：PHT-<MAC后6位>，来自芯片出厂熔丝（eFuse），**终身不变**。
//   ⚠️ 用 ESP.getEfuseMac() 而不是 WiFi.macAddress()：后者能被
//      esp_wifi_set_mac() 改掉，前者是熔丝值，跨刷机/OTA 恒定。
void pushDeriveDevId(char* out, size_t n) {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(out, n, "PHT-%02X%02X%02X",
           (unsigned)((mac >> 16) & 0xFF), (unsigned)((mac >> 8) & 0xFF), (unsigned)(mac & 0xFF));
}

// 网页标题用哪个名字：优先**显示名**，空则回退出厂默认。
//   withPrefix=true 给 <title>（默认"微型气象站 v2.1"），
//   false 给页面 <h2>（默认"气象站 v2.1"）—— 保持原有观感不变。
//   \u26a0\ufe0f 只影响**显示**；身份永远是 pushDevId。
String pushTitle(bool withPrefix) {
  if (pushDevName[0]) return String(pushDevName);
  return withPrefix ? String("\u5fae\u578b\u6c14\u8c61\u7ad9 v2.1") : String("\u6c14\u8c61\u7ad9 v2.1");
}

// 显示名合法性。⚠️ **必须与服务器的 NAME_MAX_CHARS / _name_error 保持一致**，
//   否则会出现"设备说存下了、服务器却拒绝"这种两边不一致的状态。
//   规则：1~24 个 **Unicode 码点**（中文一个字算一个），不允许控制字符，
//        其余一律允许（中文/emoji/空格/标点都行）。
//   注意这里放宽了：老版本只允许 ASCII 字母数字 . _ - —— 那是**身份名**的规则。
bool pushDevNameValid(const char* nm) {
  if (!nm) return false;
  size_t L = strlen(nm);
  if (L < 1 || L > 96) return false;          // 24 码点 × 最多 4 字节
  int cp = 0;
  for (size_t i = 0; i < L; ) {
    unsigned char c = (unsigned char)nm[i];
    int len;
    if (c < 0x80) { len = 1; if (c < 0x20 || c == 0x7F) return false; }
    else if ((c & 0xE0) == 0xC0) len = 2;
    else if ((c & 0xF0) == 0xE0) len = 3;
    else if ((c & 0xF8) == 0xF0) len = 4;
    else return false;                        // 非法前导字节
    if (i + len > L) return false;            // 多字节序列被截断
    for (int k = 1; k < len; k++) {
      if (((unsigned char)nm[i + k] & 0xC0) != 0x80) return false;   // 续字节校验
    }
    i += len; cp++;
  }
  return (cp >= 1 && cp <= 24);
}

// MAC 派生抖动（同设备固定 → 多设备天然错开，避免整点挤服务器）
uint32_t pushJitterSec() {
  uint64_t mac = ESP.getEfuseMac();
  return (uint32_t)((mac >> 3) % PUSH_JITTER_MAX_SEC);
}

// ==================== 一机一密：生成 / 清除 / 签名 / 注册 ====================

// 用硬件 TRNG 生成 64 位 hex 密钥（32 字节）
//   为什么在设备上生成而不是服务器下发：服务器就不必持有一份"别人的凭据副本"。
//   代价：绑定时要把密钥交给服务器一次（内网 HTTP）。这在你的场景（自己内网、
//   绑一次）是可接受的；换来的是"服务器被拖库 ≠ 能冒充设备以外的东西"
//   以及**内网被监听也拿不到密钥**（只看到一个公开 id）。
bool devKeyGenerate() {
  uint8_t raw[32];
  esp_fill_random(raw, sizeof(raw));
  pht_hex(raw, sizeof(raw), devKeySecret);
  memset(raw, 0, sizeof(raw));           // 别把密钥留在栈上

  uint8_t kid[16];
  esp_fill_random(kid, sizeof(kid));
  pht_hex(kid, sizeof(kid), devKeyId);
  memset(kid, 0, sizeof(kid));

  devKeySetAt = (uint32_t)time(nullptr);
  savePushConfig();
  webLogln("🔑 已生成设备密钥（keyId=%s…）—— 还需向服务器注册", devKeyId);
  return true;
}

void devKeyClear() {
  devKeySecret[0] = '\0';
  devKeyId[0]     = '\0';
  devKeySetAt     = 0;
  savePushConfig();
  webLogln("🔓 设备密钥已清除 —— 回退到旧静态 token 方式");
}

// 生成 X-PHT-Auth 头。ts 用当前 unix 秒（服务器只接受 ±90s）
bool devKeySignHeader(uint32_t ts, const String& body, String& out) {
  if (!devKeyBound()) return false;
  uint8_t key[32];
  if (!pht_unhex(devKeySecret, key, sizeof(key))) return false;

  // 待签串 —— ⚠️ 必须与服务器 devicekey._canonical 逐字节一致
  String msg = String("v1|") + pushDevId + "|" + String((unsigned long)ts) + "|" + body;

  uint8_t mac[32];
  pht_hmac_sha256(key, sizeof(key), msg.c_str(), msg.length(), mac);
  memset(key, 0, sizeof(key));

  char hex[65];
  pht_hex(mac, sizeof(mac), hex);
  memset(mac, 0, sizeof(mac));

  out = String("v1|") + pushDevId + "|" + String((unsigned long)ts) + "|" + hex;
  return true;
}

// 向服务器注册密钥（内网一次性动作）
//   服务器侧三道门：绑定口令 + **仅内网来源** + 限速。
//   ⚠️ 本函数**不能**用 pushSendBatch 那套（它自己发签名头），
//      注册本身用绑定口令认证，与数据签名无关。
bool devKeyEnroll(const char* enrollPw) {
  if (!devKeyBound()) { webLogln("❌ 注册失败：还没生成密钥"); return false; }
  if (!enrollPw || !enrollPw[0]) { webLogln("❌ 注册失败：未提供绑定口令"); return false; }
  if (WiFi.status() != WL_CONNECTED) { webLogln("❌ 注册失败：未连 WiFi"); return false; }

  String body = String("{\"secret\":\"") + devKeySecret +
                "\",\"key_id\":\"" + devKeyId + "\"}";
  String url = String("http://") + pushSrvHost + ":" + String(pushSrvPort) +
               "/api/v1/devices/" + pushDevId + "/key";

  WiFiClient c;
  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(8000);
  http.setReuse(false);
  if (!http.begin(c, url)) { webLogln("❌ 注册失败：URL 解析失败"); return false; }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-PHT-Enroll", enrollPw);

  int code = http.POST((uint8_t*)body.c_str(), body.length());
  String resp = (code > 0) ? http.getString() : String();
  http.end();

  if (code == 200) {
    webLogln("✅ 密钥已注册到服务器（%s）", pushDevId);
    return true;
  }
  if (code == 403)      webLogln("❌ 注册被拒 403：服务器认为来源不是内网。请从内网访问设备网页再点注册");
  else if (code == 401) webLogln("❌ 注册被拒 401：绑定口令不对（服务器 PHT_ENROLL_PASSWORD）");
  else if (code == 503) webLogln("❌ 注册被拒 503：服务器没配 PHT_ENROLL_PASSWORD，注册接口已禁用");
  else                  webLogln("❌ 注册失败：HTTP %d %s", code, resp.substring(0, 100).c_str());
  return false;
}

// 查询服务器上本设备的密钥状态（需管理员登录，故只用于调试）
//   —— 实际使用中"注册成功"就以 enroll 的返回码为准，这里不额外接。

void loadPushConfig() {
  prefs.begin("pht", true);
  pushEnabled = prefs.getBool("pushOn", true);
  String dn = prefs.getString("devName", "");   // 老固件的"名字"（其实是身份）
  String nm = prefs.getString("devShow", "");   // 新固件的显示名
  String sh = prefs.getString("srvHost", PUSH_SRV_HOST_DEF);
  pushSrvPort  = prefs.getUShort("srvPort", PUSH_SRV_PORT_DEF);
  lastSyncedTs = prefs.getULong("lastSync", 0);
  pushCursorValid = prefs.getBool("syncOk", false);
  pushSdDone      = prefs.getBool("sdDone", false);
  pushSdCursor    = prefs.getULong("sdSync", 0);
  { String s = prefs.getString("sdFile", ""); strlcpy(pushSdCur, s.c_str(), sizeof(pushSdCur)); }
  pushFlashCursor = prefs.getULong("flashSync", 0);
  pushFlashDone   = prefs.getBool("flashDone", false);
  // 一机一密：密钥只存 NVS，**绝不进固件**（见 deviceKeyLoad 的说明）
  {
    String sk = prefs.getString("dkSec", "");
    String ki = prefs.getString("dkId", "");
    strlcpy(devKeySecret, sk.c_str(), sizeof(devKeySecret));
    strlcpy(devKeyId,     ki.c_str(), sizeof(devKeyId));
    devKeySetAt = prefs.getULong("dkAt", 0);
  }
  prefs.end();

  // ---- 身份：永远是 eFuse 派生值（除非下面的老库迁移另有交代）----
  pushDeriveDevId(pushDevId, sizeof(pushDevId));
  // ⚠️ **老库迁移**：老固件只有一个 `devName`，它既当身份又当显示名。
  //    如果老库里存着一个**不等于**派生值的名字，说明这台设备在服务器上的
  //    身份就是那个名字 —— 必须继续沿用它当 ID，否则历史数据会对不上账。
  if (dn.length() > 0 && dn != String(pushDevId)) {
    webLogln("\u2139\ufe0f 沿用旧身份名作为设备 ID：%s（eFuse 派生值 %s）",
             dn.c_str(), pushDevId);
    strlcpy(pushDevId, dn.c_str(), sizeof(pushDevId));
  }
  // ---- 显示名：独立字段；空 = 未设置（网页与服务器都回退显示 ID）----
  if (pushDevNameValid(nm.c_str())) strlcpy(pushDevName, nm.c_str(), sizeof(pushDevName));
  else                              pushDevName[0] = '\0';
  if (sh.length() > 0) strlcpy(pushSrvHost, sh.c_str(), sizeof(pushSrvHost));
  if (pushSrvPort == 0) pushSrvPort = PUSH_SRV_PORT_DEF;
  pushConfigLoaded = true;
}

void savePushConfig() {
  prefs.begin("pht", false);
  prefs.putBool("pushOn", pushEnabled);
  prefs.putString("devName", pushDevId);      // 老固件读这个当身份 → 保持兼容
  prefs.putString("devShow", pushDevName);    // 新固件读这个当显示名
  prefs.putString("srvHost", pushSrvHost);
  prefs.putUShort("srvPort", pushSrvPort);
  prefs.putULong("lastSync", lastSyncedTs);
  prefs.putBool("syncOk", pushCursorValid);
  prefs.putULong("flashSync", pushFlashCursor);
  prefs.putBool("flashDone", pushFlashDone);
  prefs.putBool("sdDone", pushSdDone);
  prefs.putULong("sdSync", pushSdCursor);
  prefs.putString("sdFile", String(pushSdCur));
  prefs.putString("dkSec", String(devKeySecret));
  prefs.putString("dkId",  String(devKeyId));
  prefs.putULong("dkAt",   devKeySetAt);
  prefs.end();
}

// ==================== 推送：发送引擎（第二阶段，2026-10-05）====================
//   设计：docs/推送功能设计.md 三/四节 —— 主循环只做微秒级判断，HTTP 全在独立任务里。
//
//   ⚠️ 三条硬约束（踩过的坑，别改）：
//     ① HTTP 绝不能进主循环（项目已因「阻塞式串口输出」卡过主循环，HTTP 慢得多）；
//     ② 采样节拍不能被推送拖慢 —— 批间 sleep + 同优先级时间片轮转；
//     ③ 时间不可信（未同步 NTP/RTC）时**绝不推送** —— 时间戳是服务器主键，发脏了就永久错位。

// ==================== 以服务器为准的对时（2026-10-05）====================
//   为什么把服务器当时间源：签名窗是**拿服务器的钟判的**（devicekey.verify 的
//   ±HMAC_SKEW_SEC）。设备钟漂了超过 90 秒 → 每次推送必然 401；而 NTP 为了省电
//   只在"插电 + 距上次成功 ≥3 天"才跑，电池供电时根本不会跑 →
//   **数据会一直堆着传不上去，直到你插上电**。
//
//   服务器在**所有**响应上带 X-PHT-Now（含 401/429）——这是关键：
//   时钟坏掉时推送必然 401，只有 401 也带着时间，它才能在下一次推送自愈。
//
//   对电池策略零冲突：推送本来就在发生，对时是**顺带**的，不额外开射频。
#define SERVER_TIME_MIN_DRIFT 30            // 差 30 秒以内不动它（别为几秒折腾系统钟）

// 应用服务器给的时间。hdrVal 是 X-PHT-Now 的值（Unix 秒字符串）。
void applyServerTime(const String& hdrVal) {
  long long srv = atoll(hdrVal.c_str());
  if (srv < 1700000000LL) return;           // 没给 / 给了垃圾
  long long nowT = (long long)time(nullptr);
  long long drift = srv - nowT;
  long long ad = drift < 0 ? -drift : drift;
  if (ad < SERVER_TIME_MIN_DRIFT) return;

  timeval tv; tv.tv_sec = (time_t)srv; tv.tv_usec = 0;
  settimeofday(&tv, nullptr);

  // ⚠️ 时钟**向后跳**必须一并回退同步游标，否则会**静默丢数据**：
  //    推送只发 ts > lastSyncedTs 的点；往回拨钟后新采的点时间戳变小，
  //    可能落到游标之下 → 那些点永远发不出去（RAM 环和 SD 归档两条路都卡住）。
  //    回退游标会让旧点被**重发**一遍 —— 服务器按 ts 幂等去重，重发无害，漏发有害。
  if (drift < 0 && pushCursorValid && lastSyncedTs > (uint32_t)srv) {
    lastSyncedTs = (uint32_t)srv - 60;      // 多回退一点留余量
    pushCursorValid = false;
    savePushConfig();
    webLogln("\u26a0\ufe0f 时钟向后校正 %lld 秒 → 同步游标一并回退（旧点会重发，服务器去重）", drift);
  }
  if (rtcOK) rtc.adjust(DateTime((uint32_t)srv));   // 顺手校 DS3231（离线基准）
  webLogln("\U0001f550 已按服务器校时（偏差 %+lld 秒）", drift);
}

// ==================== 改显示名（2026-10-05）====================
//   **必须服务器验签通过才落 NVS**。为什么不能只在本地改：
//     * 网络断了你改了、服务器没改 → 两边长期不一致；
//     * 服务器是"改名通知"的源头，本地擅自改会被它下次下发覆盖回来。
//   鉴权用**设备自己的密钥签名**（X-PHT-Auth），**不是绑定口令** ——
//   绑定口令能换任意设备的密钥（劫持数据流），绝不能交给朋友。
bool devRenameTo(const char* want) {
  if (!want) return false;
  // 空名字是**合法语义**（清除显示名 → 网页回退显示身份 ID）。
  //   \u26a0\ufe0f 校验只针对非空 —— 否则"清除"这条路会被自己挡住。
  if (want[0] && !pushDevNameValid(want)) {
    webLogln("\u274c 改名失败：名字不合法（1~24 个字符，不能有控制字符）");
    return false;
  }
  if (!devKeyBound()) {
    webLogln("\u274c 改名失败：这台设备还没绑定密钥（改名要靠密钥签名）");
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    webLogln("\u274c 改名失败：未连 WiFi");
    return false;
  }
  if (time(nullptr) < 1700000000) {
    webLogln("\u274c 改名失败：设备时钟不可信（服务器只收 \u00b190 秒内的签名）");
    return false;
  }

  String body = String("{\"name\":\"") + pushJsonEsc(want) + "\"}";
  uint32_t ts = (uint32_t)time(nullptr);
  String hdr;
  if (!devKeySignHeader(ts, body, hdr)) {
    webLogln("\u274c 改名失败：签名失败（密钥格式非法？）");
    return false;
  }

  String url = String("http://") + pushSrvHost + ":" + String(pushSrvPort) +
               "/api/v1/devices/" + pushDevId + "/name";
  WiFiClient c;
  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(8000);
  http.setReuse(false);
  if (!http.begin(c, url)) { webLogln("\u274c 改名失败：URL 解析失败"); return false; }
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-PHT-Auth", hdr);

  int code = http.POST((uint8_t*)body.c_str(), body.length());
  String resp = (code > 0) ? http.getString() : String();
  http.end();

  if (code != 200) {
    if (code == 401)      webLogln("\u274c 改名被拒 401：签名无效（时钟偏差过大？或服务器上密钥不一致）");
    else if (code == 403) webLogln("\u274c 改名被拒 403：签名里的设备与请求路径不一致");
    else if (code == 400) webLogln("\u274c 改名被拒 400：服务器认为名字不合法 %s", resp.substring(0, 90).c_str());
    else if (code < 0)    webLogln("\u274c 改名失败：服务器不可达（%s:%u）", pushSrvHost, (unsigned)pushSrvPort);
    else                  webLogln("\u274c 改名失败：HTTP %d %s", code, resp.substring(0, 90).c_str());
    return false;
  }

  strlcpy(pushDevName, want, sizeof(pushDevName));
  savePushConfig();
  webLog("\U0001f3f7\ufe0f 设备名已改 -> %s（服务器已记下）\n", pushDevName);
  if (resp.indexOf("\"duplicate_of\":\"") >= 0) {
    webLogln("\u2139\ufe0f 提示：已有另一台设备用这个名字 —— 服务器列表里会用 ID 后缀区分，不影响使用");
  }
  return true;
}

// JSON 转义（设备名可以含中文/emoji，所以必须真的转义，不能只靠白名单）
static String pushJsonEsc(const char* s) {
  String o; o.reserve(strlen(s) + 4);
  for (const char* p = s; *p; ++p) {
    char c = *p;
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((unsigned char)c < 0x20) { o += '?'; }
    else o += c;
  }
  return o;
}

// ==================== flash(/log.csv) 第三阶段补传 ====================
//   依据：docs/推送功能设计.md 四、数据源分层（RAM → flash(/log.csv) → SD 归档）
//
//   为什么必须做：RAM 环只有 600 条（约 5 小时），开机前的历史在 /log.csv 里。
//   只推 RAM 的话，服务器永远缺"最早内存点之前"那一段（实测 PHT-6B4580 缺约 1100 条）。
//
//   三条约束：
//     ① 流式读文件，不一次性读进内存（RAM 紧张，log.csv 可能上万行）；
//     ② 只推【时间戳有效】的行 —— 早期没校时的行存成 "HH:MM:SS"（<1e9），
//        推上去会永久污染服务器时间轴，必须跳过；
//     ③ 每轮有上限（行数），推不完下轮接着推，绝不把任务占死。

// 在 /log.csv 里找第一行 ts > after，并把文件指针留在该行行首
static bool pushLogSeekAfter(File& f, uint32_t after) {
  f.seek(0);
  if (!f.available()) return false;
  f.readStringUntil('\n');                       // 跳过表头
  while (f.available()) {
    int start = f.position();
    String line = f.readStringUntil('\n');
    if (line.length() < 8) continue;
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);
    if (i1 < 0 || i2 < 0) continue;
    struct tm tm;
    if (strptime(line.substring(i1 + 1, i2).c_str(), "%Y-%m-%d %H:%M:%S", &tm)) {
      uint32_t t = (uint32_t)mktime(&tm);
      if (t < 1000000000UL) continue;            // 时间不可信
      if (t > after) { f.seek(start); return true; }
    }
  }
  return false;                                  // 没有更新的行 → 已补完
}

// ---- SD 归档牌组 ----
// 列出 SD 归档文件并排序（"2026-10-02.csv" 这种日期名，字典序 == 时间序）
static int pushSdCollect(String* out, int maxN) {
  int n = 0;
  File d = SD.open("/");
  if (!d) return 0;
  while (n < maxN) {
    File e = d.openNextFile();
    if (!e) break;
    if (!e.isDirectory()) {
      String nm = e.name();
      int sl = nm.lastIndexOf('/');
      if (sl >= 0) nm = nm.substring(sl + 1);
      if (nm.length() == 14 && nm.endsWith(".csv") && nm[4] == '-' && nm[7] == '-') {
        out[n++] = nm;
      }
    }
    e.close();
  }
  d.close();
  for (int i = 1; i < n; i++) {                    // 插入排序（n 很小）
    String key = out[i];
    int j = i - 1;
    while (j >= 0 && out[j] > key) { out[j + 1] = out[j]; j--; }
    out[j + 1] = key;
  }
  return n;
}

// 找 afterFile 之后（不含）第一个还有可信数据的 SD 文件；exhausted=true 表示到最后一个了
static bool pushSdFindNext(const String& afterFile, String& out, bool* exhausted) {
  String list[32];
  int n = pushSdCollect(list, 32);
  int from = 0;
  if (afterFile.length()) {
    from = n;                                      // 找不到就当"已经是最后一个"
    for (int i = 0; i < n; i++) { if (list[i] == afterFile) { from = i + 1; break; } }
  }
  for (int i = from; i < n; i++) {
    File f = SD.open("/" + list[i], "r");
    if (!f) continue;
    bool has = pushLogSeekAfter(f, 0);             // 有可信时间的行吗
    f.close();
    if (has) { out = list[i]; if (exhausted) *exhausted = false; return true; }
  }
  if (exhausted) *exhausted = true;
  return false;
}

// 选定本轮要补的牌组：1=SD 2=flash 0=都补完了（先旧后新，避免留下空洞）
static int pushBackfillPick(String& sdOut) {
  if (!pushSdDone) {
    // ⚠️ 卡没挂上 ≠ 补完了：initSD() 失败只置 sdOK=false，运行中还能靠 ensureSD() 重挂。
    //    这里若把"读不到文件"当成"已补完"，就会永久跳过整个 SD 归档。
    if (!sdOK) return 2;                           // 卡还没就绪 → 先补 flash，等下次
    bool ex = false;
    if (pushSdFindNext(pushSdCur, sdOut, &ex)) return 1;
    if (!ex) return 1;
    pushSdDone = true;                             // 卡已挂载且没有更新的文件了 → SD 牌组补完
    savePushConfig();
    webLogln("📚 SD 归档：已补到最后一个文件");
    return 2;
  }
  if (!pushFlashDone) return 2;
  return 0;
}

// 走一遍某个牌组。返回已发送条数；skippedRows 回填"跳过多少不可信行"；
// lastNonEmpty 回填"文件的最后一个有效时间戳"（收尾对齐游标用）。
//   cur 是引用：每成功一批就前进并落盘（换页/断电都能续）。
static uint32_t pushBackfillDeck(File& f, uint32_t& cur, const char* tag,
                                 int maxRows, int* skippedRows, uint32_t* lastNonEmpty) {
  int skipped = 0;
  uint32_t sent = 0;
  uint32_t lastT = 0;
  static uint32_t bTs[PUSH_BATCH_MAX];
  static float    bT[PUSH_BATCH_MAX], bH[PUSH_BATCH_MAX], bP[PUSH_BATCH_MAX];
  int n = 0, rows = 0;

  while (f.available() && rows < maxRows) {
    String line = f.readStringUntil('\n');
    if (line.length() < 8) continue;
    int i1 = line.indexOf(',');
    int i2 = line.indexOf(',', i1 + 1);
    int i3 = line.indexOf(',', i2 + 1);
    int i4 = line.indexOf(',', i3 + 1);
    if (i1 < 0 || i2 < 0 || i3 < 0 || i4 < 0) continue;
    struct tm tm;
    uint32_t t = 0;
    if (strptime(line.substring(i1 + 1, i2).c_str(), "%Y-%m-%d %H:%M:%S", &tm)) t = (uint32_t)mktime(&tm);
    if (t < 1000000000UL) { skipped++; continue; }     // 未校时的行 → 跳过（会污染服务器）
    if (t <= cur) continue;
    lastT = t;
    rows++;
    bTs[n] = t;
    bT[n] = parseVal(line.substring(i2 + 1, i3));
    bH[n] = parseVal(line.substring(i3 + 1, i4));
    int i5 = line.indexOf(',', i4 + 1);
    bP[n] = parseVal((i5 < 0) ? line.substring(i4 + 1) : line.substring(i4 + 1, i5));
    n++;
    if (n >= PUSH_BATCH_MAX) {                          // 攒满一批就发
      pushLastTryMs = millis();
      pushRunState = 1;
      if (!pushSendBatch(n, bTs, bT, bH, bP)) {
        pushLastOk = false; pushFailStreak++; pushRunState = 2;
        if (skippedRows) *skippedRows = skipped;
        if (lastNonEmpty) *lastNonEmpty = lastT;
        return sent;
      }
      pushLastOk = true; pushFailStreak = 0;
      pushLastOkMs = millis(); pushLastUploadMs = pushLastOkMs;
      cur = bTs[n - 1];
      if (cur > lastSyncedTs) lastSyncedTs = cur;       // RAM 游标只许前进
      pushLastUploadTs = cur;
      savePushConfig();
      sent += n;
      webLog("📚 %s 补传 %d 条 → 新入库 %u（游标 %lu）\n",
             tag, n, (unsigned)pushLastAccepted, (unsigned long)cur);
      n = 0;
      vTaskDelay(pdMS_TO_TICKS(PUSH_CHUNK_GAP_MS));     // 批间让步
    }
  }
  f.close();

  if (n > 0) {                                          // 尾批
    pushLastTryMs = millis();
    pushRunState = 1;
    if (!pushSendBatch(n, bTs, bT, bH, bP)) {
      pushLastOk = false; pushFailStreak++; pushRunState = 2;
      if (skippedRows) *skippedRows = skipped;
      if (lastNonEmpty) *lastNonEmpty = lastT;
      return sent;
    }
    pushLastOk = true; pushFailStreak = 0;
    pushLastOkMs = millis(); pushLastUploadMs = pushLastOkMs;
    cur = bTs[n - 1];
    if (cur > lastSyncedTs) lastSyncedTs = cur;
    pushLastUploadTs = cur;
    savePushConfig();
    sent += n;
    webLog("📚 %s 补传 %d 条 → 新入库 %u（游标 %lu）\n",
           tag, n, (unsigned)pushLastAccepted, (unsigned long)cur);
  }
  if (skippedRows) *skippedRows = skipped;
  if (lastNonEmpty) *lastNonEmpty = lastT;
  return sent;
}

// 补传节拍：返回 true 表示"本轮已处理，调用方应 continue"。
//   一次只走一个牌组、单轮上限 1200 行（≈6 批），推不完 2 秒后接着来。
static bool pushBackfillTick(void) {
  String deck;
  int pick = pushBackfillPick(deck);
  if (pick == 0) return false;

  int skipped = 0;
  uint32_t lastT = 0;
  File bf;
  const char* tag = "flash";
  uint32_t* cur = &pushFlashCursor;
  if (pick == 1) {
    bf = SD.open("/" + deck, "r");
    tag = deck.c_str();
    cur = &pushSdCursor;
  } else {
    bf = LittleFS.open("/log.csv", "r");
  }

  if (!bf) {                                        // 打不开（卡被拔了 / 文件不见了）
    webLog("⚠️ 补传：打不开 %s，2 分钟后重试\n", tag);
    pushRunState = 0;
    pushReschedule(120);                            // 别死循环，缓一缓再来
    return true;
  }

  uint32_t sent = 0;
  sent = pushBackfillDeck(bf, *cur, tag, 1200, &skipped, &lastT);
  if (pick == 1) pushSdSent += sent; else pushFlashSent += sent;

  if (pushFailStreak > 0) {                         // 失败 → 退避后重试
    uint32_t back = PUSH_RETRY_MIN_SEC;
    for (uint16_t i = 1; i < pushFailStreak && back < PUSH_RETRY_MAX_SEC; i++) back *= 2;
    if (back > PUSH_RETRY_MAX_SEC) back = PUSH_RETRY_MAX_SEC;
    webLog("⏳ %s 补传失败（第 %u 次）→ %lus 后重试\n", tag, (unsigned)pushFailStreak, (unsigned long)back);
    pushReschedule(back);
    return true;
  }
  if (skipped) webLog("ℹ️ %s 补传跳过 %d 行（早期未校时，时间戳不可信）\n", tag, skipped);

  if (sent > 0) {                                   // 这份还没走完 → 2 秒后接着补
    pushRunState = 0;
    pushReschedule(2);
    return true;
  }
  // sent == 0 → 这份走完了
  if (pick == 1) {                                  // SD：记下文件名，下一轮自动前进到下一份
    if (lastT > pushSdCursor) pushSdCursor = lastT;
    strlcpy(pushSdCur, deck.c_str(), sizeof(pushSdCur));
    savePushConfig();
  } else if (!pushFlashDone) {
    pushFlashDone = true;
    savePushConfig();
    webLogln("📚 flash 补传：已到文件尾，切回内存环推送");
  }
  pushRunState = 0;
  pushReschedule(2);                                // 接着看下一个牌组
  return true;
}


// 环形缓冲里 ts > after 且时间有效的点数
int pushCountNewer(uint32_t after) {
  if (!buffer || bufferSize <= 0) return 0;
  int n = 0;
  for (int i = 0; i < bufferSize; i++) {
    const DataPoint& d = buffer[(bufferHead - bufferSize + i + bufferCapacity) % bufferCapacity];
    if (d.time > after && hasValidDate(d.time)) n++;
  }
  return n;
}

// 取一批待传点（ts > lastSyncedTs 且时间有效）。
//   OUT : outSeq[i] = 在环形缓冲里的**序号**（0 = 最老），成功提交时用它推游标。
//   返回：实际条数（0 = 没有待传）
int pushCollectBatch(int maxN, uint32_t* outTs, float* outT, float* outH, float* outP, int* outSeq) {
  if (!buffer || bufferSize <= 0 || maxN <= 0) return 0;
  int n = 0;
  for (int i = 0; i < bufferSize && n < maxN; i++) {
    const DataPoint& d = buffer[(bufferHead - bufferSize + i + bufferCapacity) % bufferCapacity];
    if (d.time <= lastSyncedTs) continue;        // 已上传
    if (!hasValidDate(d.time))  continue;        // 时间不可信 → 不传（会永久污染服务器）
    outTs[n] = d.time; outT[n] = d.temp; outH[n] = d.humidity; outP[n] = d.pressure;
    outSeq[n] = i;
    n++;
  }
  return n;
}

// 组 JSON 并 POST。成功返回 true，并把响应里的 accepted / needBackfillBefore 落进全局。
//   只在推送任务里调用（这里会阻塞到超时）。
bool pushSendBatch(int n, const uint32_t* ts, const float* t, const float* h, const float* p) {
  if (n <= 0) return false;

  String body;
  body.reserve((size_t)n * 90 + 128);
  body  = "{\"device\":\"" + pushJsonEsc(pushDevId) + "\",\"fw\":\"" + PUSH_FW_VER + "\",\"samples\":[";
  char item[128];
  for (int i = 0; i < n; i++) {
    int k = snprintf(item, sizeof(item), "%s{\"ts\":%lu", (i ? "," : ""), (unsigned long)ts[i]);
    if (!isnan(t[i])) k += snprintf(item + k, sizeof(item) - k, ",\"t\":%.2f", t[i]);
    if (!isnan(h[i])) k += snprintf(item + k, sizeof(item) - k, ",\"h\":%.2f", h[i]);
    if (!isnan(p[i])) k += snprintf(item + k, sizeof(item) - k, ",\"p\":%.2f", p[i]);
    snprintf(item + k, sizeof(item) - k, ",\"mode\":%u}", (unsigned)deviceMode);
    body += item;
  }
  body += "]}";

  WiFiClient c;
  HTTPClient http;
  http.setConnectTimeout(4000);            // 连接（WiFi 正常时 <100ms）
  http.setTimeout(PUSH_HTTP_TIMEOUT_MS);   // 读写
  http.setReuse(false);                    // 每批独立连接：不用长连接，省得服务器/中间设备踢我们

  String url = String("http://") + pushSrvHost + ":" + String(pushSrvPort) + PUSH_API_PATH;
  if (!http.begin(c, url)) {
    webLogln("⚠️ 推送：URL 解析失败（%s）", url.c_str());
    return false;
  }
  http.addHeader("Content-Type", "application/json");

  // ── 认证：优先一机一密，未绑定才回退旧静态 token ──
  //   ⚠️ 签名必须覆盖 body **原始字节** —— 服务器先 await request.body() 再验签，
  //      这里送出去的也是 body 的原始字节，两边一致。
  //   ⚠️ 用 time(nullptr) 当 ts：服务器只接受 ±90s。设备时间靠 RTC/NTP，
  //      没校时时 ts 不可信 → 那种情况下服务器会 401，日志里能看到原因。
  // 收 X-PHT-Now：把服务器当时间源（见 applyServerTime 的说明）
  static const char* kNowHdr[] = { "X-PHT-Now" };
  http.collectHeaders(kNowHdr, 1);

  if (devKeyBound()) {
    uint32_t nowTs = (uint32_t)time(nullptr);
    String authHdr;
    if (devKeySignHeader(nowTs, body, authHdr)) {
      http.addHeader("X-PHT-Auth", authHdr);
    } else {
      webLogln("⚠️ 推送：密钥签名失败（密钥格式非法？）—— 本批不发送");
      http.end();
      return false;
    }
  } else {
    // token 为空 = 不发该头（只有服务器也没设 token 时才成立）
    const char* tok = PHT_API_TOKEN;
    if (tok && tok[0]) http.addHeader("X-PHT-Token", tok);
  }

  int code = http.POST((uint8_t*)body.c_str(), body.length());
  String resp = (code > 0) ? http.getString() : String();
  String srvNow = http.header("X-PHT-Now");   // \u26a0\ufe0f 必须在 end() 之前读
  http.end();

  // 无论成功还是 401/429 都校时 —— 这正是"时钟坏掉能自愈"的关键
  if (srvNow.length()) {
    pushSrvClock = (uint32_t)atoll(srvNow.c_str());
    applyServerTime(srvNow);
  } else if (!pushNoNowHdrWarned) {
    // 只报一次，免得刷屏。**这条很重要**：没有它，"头压根没读到"这种静默
    // 失效（比如 collectHeaders 没生效）会完全看不出来，功能悄悄变成空转。
    pushNoNowHdrWarned = true;
    webLogln("\u26a0\ufe0f 推送响应里没有 X-PHT-Now 头 —— 服务器版本可能过旧，"
             "无法按服务器校时（不影响推送本身）");
  }

  if (code != 200) {
    if (code == 401)      webLogln(devKeyBound()
                              ? "❌ 推送被拒：401（签名无效 —— 设备时钟偏差过大？或服务器上该设备密钥不一致；可重新注册）"
                              : "❌ 推送被拒：401（token 不对 —— 固件注入的 PHT_API_TOKEN 与服务器 /etc/pht-server.env 不一致）");
    else if (code == 429) webLogln("❌ 推送被限速：429（%s）", resp.substring(0, 80).c_str());
    else if (code < 0)    webLogln("❌ 推送失败：%s（%s:%u 不可达？）", http.errorToString(code).c_str(),
                                   pushSrvHost, (unsigned)pushSrvPort);
    else                  webLogln("❌ 推送失败：HTTP %d %s", code, resp.substring(0, 80).c_str());
    return false;
  }

  // 解析响应（结构稳定，手扫字段，不引 JSON 库）
  long accepted = -1;
  int ka = resp.indexOf("\"accepted\"");
  if (ka >= 0) { int cc = resp.indexOf(':', ka); if (cc > 0) accepted = resp.substring(cc + 1).toInt(); }
  pushLastAccepted = (accepted > 0) ? (uint16_t)accepted : 0;
  int kb = resp.indexOf("\"needBackfillBefore\"");
  if (kb >= 0) {
    int cc = resp.indexOf(':', kb);
    if (cc > 0) {
      long bf = resp.substring(cc + 1).toInt();
      if (bf > 1000000000) { pushNeedBackfill = true; pushBackfillBeforeTs = (uint32_t)bf; }
      else                 { pushNeedBackfill = false; pushBackfillBeforeTs = 0; }
    }
  }

  // 管理员在服务器上改过名 → 借这次**成功**响应回传一次（服务器取完即清，不空转）。
  //   没改名时响应里根本没有 name 字段，所以这里几乎不会命中。
  int kn = resp.indexOf("\"name\"");
  if (kn >= 0) {
    int c1 = resp.indexOf(':', kn);
    int q1 = (c1 > 0) ? resp.indexOf('"', c1 + 1) : -1;
    int q2 = (q1 > 0) ? resp.indexOf('"', q1 + 1) : -1;
    if (q1 > 0 && q2 > q1) {
      String nm = resp.substring(q1 + 1, q2);
      // 空字符串是**合法载荷**：表示"服务器清除了显示名"，本地也要清掉，
      //   否则服务器那边已经回退显示 ID、设备上还挂着旧名字。
      bool okName = (nm.length() == 0) || pushDevNameValid(nm.c_str());
      if (okName && nm != String(pushDevName)) {
        strlcpy(pushDevName, nm.c_str(), sizeof(pushDevName));
        savePushConfig();
        if (nm.length()) webLogln("\U0001f3f7\ufe0f 服务器下发了新设备名：%s", pushDevName);
        else             webLogln("\U0001f3f7\ufe0f 服务器清除了设备名（本地回退显示 ID）");
      }
    }
  }
  return true;
}

// 游标夹紧：把 lastSyncedTs 拉回"RAM 环里还有的"范围，返回是否动过。
//   为什么必须有：lastSyncedTs 是绝对时间戳，而数据源只有 RAM 环（约 5 小时）。
//   重启后环可能已被覆盖（旧游标 > 环里最新点）→ 待传数算出来是 0 → 积压永远推不上去。
//   这里只在【真的落后】时才动，并把 pushCursorValid 置 false（说明中间有空档，
//   重启后必须重新夹一次，不能直接信这个游标）。
bool pushClampCursor() {
  if (bufferSize <= 0) return false;
  uint32_t oldestTs = 0;
  for (int i = 0; i < bufferSize; i++) {
    const DataPoint& d = buffer[(bufferHead - bufferSize + i + bufferCapacity) % bufferCapacity];
    if (hasValidDate(d.time) && (oldestTs == 0 || d.time < oldestTs)) oldestTs = d.time;
  }
  if (!oldestTs || oldestTs <= lastSyncedTs) return false;      // 环里的点全都推过了
  if (lastSyncedTs > 0 && (oldestTs - lastSyncedTs) > (uint32_t)(INTERVAL_SEC * 3)) {
    webLog("⚠️ 推送游标落后内存窗口：跳过 %lu 秒空档（%.1f 小时）—— 只推 RAM 里现有的点\n",
           (unsigned long)(oldestTs - lastSyncedTs), (oldestTs - lastSyncedTs) / 3600.0f);
  }
  lastSyncedTs = oldestTs - 1;                                 // -1 保证最老点本身也被收集到
  pushCursorValid = false;                                     // 中间有空档 → 游标不再可信
  return true;
}
void pushReschedule(uint32_t delaySec) {
  if (delaySec == 0) delaySec = pushJitterSec();       // 首推也带抖动，多设备不同时打服务器
  pushNextAtMs = millis() + delaySec * 1000UL;
  if (pushNextAtMs == 0) pushNextAtMs = 1;
}

// 主循环勾子：**只做微秒级判断**（readPowerState 本身很便宜，项目里每圈都在调）。
//   触发：① 插电瞬间  ② 本会话首次联网  ③ 每 15 分钟（由任务排程）
void servicePushTick() {
  if (!pushEnabled) return;

  bool ext = readPowerState().powered;
  if (ext && !pushPowerWasExt) {          // 决策 6：插电立刻同步一次
    pushTickForce = true;
    webLogln("🔌 检测到插电 → 触发一次推送检查");
  }
  pushPowerWasExt = ext;

  bool wifi = (WiFi.status() == WL_CONNECTED);
  if (wifi && pushLastBootPushMs == 0) {  // 本会话首次联网 → 先同步一次积压
    pushLastBootPushMs = millis() ? millis() : 1;
    pushTickForce = true;
  }
  if (pushTickForce) return;              // 已经要求任务去干，不必再判断
  if (!wifi) pushNextAtMs = 0;            // 断线 → 清排程；下次联网由上面的首推分支接手
}

// 一轮同步：循环取批 + 发送，直到没有待传或失败。
//   返回 1 = 跑完，0 = 无待传，-1 = 失败（退避由调用者处理）
int pushRunPass() {
  if (!pushEnabled) return 0;
  if (WiFi.status() != WL_CONNECTED) return -1;

  static uint32_t bTs[PUSH_BATCH_MAX];
  static float    bT[PUSH_BATCH_MAX], bH[PUSH_BATCH_MAX], bP[PUSH_BATCH_MAX];
  static int      bSeq[PUSH_BATCH_MAX];

  int batches = 0;
  while (true) {
    if (!pushEnabled || pushCancel || WiFi.status() != WL_CONNECTED) return -1;
    if (batches >= 40) return 1;                   // 单轮上限 8000 条，防一次跑太久
    int n = pushCollectBatch(PUSH_BATCH_MAX, bTs, bT, bH, bP, bSeq);
    if (n <= 0) return 1;                          // 没有待传 → 本轮完成

    pushLastTryMs = millis();
    pushRunState = 1;
    uint32_t t0 = millis();
    bool ok = pushSendBatch(n, bTs, bT, bH, bP);
    uint32_t dt = millis() - t0;

    if (!ok) {
      pushLastOk = false;
      pushFailStreak++;
      pushRunState = 2;
      return -1;
    }

    pushLastOk = true;
    pushFailStreak = 0;
    pushLastOkMs = millis();
    pushLastUploadMs = pushLastOkMs;
    // 只有【整批成功】才推游标 + 落 NVS → 中途失败可安全重发（服务器按 ts 幂等）
    lastSyncedTs = bTs[n - 1];
    pushLastUploadTs = lastSyncedTs;
    savePushConfig();
    batches++;

    webLog("📤 推送 %d 条 → 新入库 %u，耗时 %lums（游标 %lu）\n",
           n, (unsigned)pushLastAccepted, (unsigned long)dt, (unsigned long)lastSyncedTs);

    vTaskDelay(pdMS_TO_TICKS(PUSH_CHUNK_GAP_MS));  // 批间让步：压低 WiFi 占空比
  }
}

// 独立推送任务：只在这里阻塞（连接 + 上传）。优先级与 loop 相同 → tick 轮转，不抢占采样。
void pushTask(void* arg) {
  (void)arg;
  vTaskDelay(pdMS_TO_TICKS(8000));                 // 等系统起稳（WiFi/RTC/NTP）
  webLogln("📮 推送任务已启动（%d 分钟一次 + MAC 抖动，失败退避 %d~%ds）",
           PUSH_INTERVAL_SEC / 60, PUSH_RETRY_MIN_SEC, PUSH_RETRY_MAX_SEC);
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(500));                // 廉价轮询；light sleep 期间会让路，醒来立刻恢复
    if (!pushEnabled || pushCancel) { pushRunState = 0; continue; }

    bool due = pushTickForce;
    if (!due && pushNextAtMs != 0) due = ((int32_t)(millis() - pushNextAtMs) >= 0);
    if (!due) continue;
    if (WiFi.status() != WL_CONNECTED) { pushRunState = 0; continue; }   // 没网先不动，等 WiFi

    if (!timeSynced || !hasValidDate((uint32_t)time(nullptr))) {         // 时间不可信 → 绝不推
      pushTickForce = false; pushRunState = 0;
      pushNextAtMs = millis() + 30000UL;                                 // 30s 后再看
      continue;
    }
    pushTickForce = false;

    // ---- 第三阶段：补历史（SD 归档 → flash → RAM），先旧后新，一次只走一个牌组 ----
    if (pushBackfillTick()) continue;

    // 游标夹紧：落后于内存窗口（重启 / 长时间断网 / 环已被覆盖）→ 退到 RAM 最老的有效点
    pushClampCursor();

    if (pushCountNewer(lastSyncedTs) <= 0) {       // 无待传：不算失败，直接排下一轮
      pushLastOk = true;
      pushRunState = 0;
      pushPassCount++;
      pushReschedule(PUSH_INTERVAL_SEC + pushJitterSec());
      continue;
    }

    int r = pushRunPass();

    if (r == 1) {                                  // ✅ 本轮推完
      pushCursorValid = true;
      savePushConfig();
      pushRunState = 0;
      pushPassCount++;
      if (pushNeedBackfill) {
        webLog("ℹ️ 服务器提示还有更早的缺口（早于 %lu）—— 属第二阶段（CSV/SD 补传）\n",
               (unsigned long)pushBackfillBeforeTs);
      }
      pushReschedule(PUSH_INTERVAL_SEC + pushJitterSec());
    } else {                                       // ❌ 失败 → 指数退避
      uint32_t back = PUSH_RETRY_MIN_SEC;
      for (uint16_t i = 1; i < pushFailStreak && back < PUSH_RETRY_MAX_SEC; i++) back *= 2;
      if (back > PUSH_RETRY_MAX_SEC) back = PUSH_RETRY_MAX_SEC;
      webLog("⏳ 推送失败（第 %u 次）→ %lus 后重试\n", (unsigned)pushFailStreak, (unsigned long)back);
      pushReschedule(back);
    }
  }
}

// 当前无线状态 -> UI 图标
// ===== V2.1.1-c 顶栏快通道：状态量（声明得早于 /status 组装）=====
uint8_t  topSeenPw   = 2,  topSeenChg = 2;      // 2 = 未知 → 首轮强制刷一次
uint8_t  topSeenWifi = 255;
int      topSeenPct  = -1, topSeenMin = -1;
bool     topSeenCv   = false, topSeenIna = true;
uint32_t topPollAt   = 0;                      // 下次 5s 轮询时刻
uint32_t topEdgeAt   = 0;                      // !=0 → 预约的新鲜电源复查
uint32_t topRefreshCount = 0;                  // 顶栏重画计数（/status 暴露，便于验证/自检）

uint8_t wifiUIState() {
  if (WiFi.status() == WL_CONNECTED) return UI_WIFI_STA;
  if (apEnabled)                     return UI_WIFI_AP;
  return UI_WIFI_OFF;
}

// 组装 SSID 行：带引号（名字含空格也能看出边界），超宽自动截断加 ".."
void buildSsidLine(char* out, size_t n, const char* ssid) {
  snprintf(out, n, "SSID \"%s\"", ssid);
  if (uiDisplay.textWidthLogical(out, true) <= 240) return;
  int len = (int)strlen(ssid);
  while (len > 3) {
    len--;
    snprintf(out, n, "SSID \"%.*s..\"", len, ssid);
    if (uiDisplay.textWidthLogical(out, true) <= 240) return;
  }
}

// 把最新状态同步进屏幕对象（同步完记得重绘）
void uiSyncStatus() {
  uiDisplay.setPower(g_power.charging, g_power.cvZone, g_power.inaOK, g_power.battPct);
  uiDisplay.setPr1(pr1BatteryDirect);        // v2.2：电池直供时顶栏显示标识
  uiDisplay.setWifi(curWireless == WL_BLE ? UI_WIFI_BLE : wifiUIState());   // 蓝牙态显示 BT
  uiDisplay.setMode(deviceMode);
}

String buildStatusJson(bool overBLE) {
  PowerState ps = readPowerState();
  String s = "{\"count\":" + String(recordCount) +
             ",\"bufferSize\":" + String(bufferSize) +
             ",\"interval\":" + String(INTERVAL_SEC) +
             ",\"mode\":" + String(deviceMode) +
             ",\"bmpOK\":" + (bmpOK?"true":"false") +
             ",\"shtOK\":" + (shtOK?"true":"false") +
             ",\"sdOK\":" + (sdOK?"true":"false") +
             ",\"rtcOK\":" + (rtcOK?"true":"false") +
             ",\"timeSynced\":" + (timeSynced?"true":"false") +
             ",\"webLogging\":" + (webLoggingEnabled?"true":"false") +
             ",\"apEnabled\":" + (apEnabled?"true":"false") +
             ",\"powered\":" + (ps.powered?"true":"false") +
             ",\"charging\":" + (ps.charging?"true":"false") +
             ",\"onBattery\":" + (ps.onBattery?"true":"false") +
             ",\"battVolt\":" + (ps.inaOK ? String(ps.battVolt, 2) : String("null")) +
             ",\"battPct\":" + String(ps.battPct) +
             // V2.1.1：低电三档的**当前生效值**（有表时按本机表算；便于网页/调试核对）
             ",\"lowThr\":{\"archive\":" + String(battArchiveV, 3) +
                            ",\"stop\":" + String(battStopV, 3) +
                            ",\"sleep\":" + String(battSleepV, 3) + "}" +
             ",\"inaOK\":" + (ps.inaOK?"true":"false") +
             ",\"battCurrent\":" + String(ps.battCurrent_mA, 1) +
             ",\"battPower\":" + String(ps.battPower_mW, 1) +
             ",\"cvZone\":" + (ps.cvZone?"true":"false") +
             ",\"chgFast\":" + (chargeIsFast()?"true":"false") +
             ",\"chgEnabled\":" + (chargeIsEnabled()?"true":"false") +
             ",\"chgSlow\":" + (chargeForceSlow?"true":"false") +      // V2.1.1-a：移动+插电时临时慢充
             // ---- TPS2117 PR1 电源通路（v2.2）----
             //   pr1Raw  = 引脚**实际回读电平**（比状态变量可信：能发现"写了但没生效"）
             //   pr1Safe = 是否处于安全态（仅在电压 ≤ 硬上限时才允许电池直供）
             ",\"pr1\":\"" + (pr1BatteryDirect ? "VIN1_BATTERY_DIRECT" : "VIN2_LDO") + "\"" +
             // ---- USB 枚举结果（v2.2）----
             ",\"usb\":" + String(usbEnumState == ENUM_IS_HOST ? "\"host\"" :
                                  (usbEnumState == ENUM_IS_CHARGER ? "\"charger\"" : "\"unknown\"")) +
             ",\"usbSof\":" + String(usbReadSofFrame()) +
             ",\"allowCharge\":" + (usbAllowCharge ? "true" : "false") +
             ",\"chargingViable\":" + (usbChargingViable ? "true" : "false") +
             // ---- 推送（v2.2，局域网先行）----
             ",\"push\":{\"enabled\":" + (pushEnabled ? "true" : "false") +
                          ",\"dev\":\"" + String(pushDevId) + "\"" +            // dev = 身份（脚本/对账用）
                          ",\"name\":\"" + pushJsonEsc(pushDevName) + "\"" +     // name = 显示名（可空）
                          ",\"srvClock\":" + String((unsigned long)pushSrvClock) +    // 最近读到的服务器时间（0=没读到）
                          ",\"srv\":\"" + String(pushSrvHost) + ":" + String(pushSrvPort) + "\"" +
                          ",\"lastSyncedTs\":" + String(lastSyncedTs) +
                           ",\"cursorOk\":" + (pushCursorValid ? "true" : "false") +   // 环已推干净（游标可信）；false=中间有空档
                          ",\"lastOk\":" + (pushLastOk ? "true" : "false") +
                          ",\"failStreak\":" + String(pushFailStreak) +
                          ",\"jitter\":" + String(pushJitterSec()) +
                          ",\"lastTrySec\":" + String(pushLastTryMs ? (millis() - pushLastTryMs) / 1000 : 0) +
                          ",\"lastOkSec\":" + String(pushLastOkMs ? (millis() - pushLastOkMs) / 1000 : 0) +
                          ",\"state\":" + String(pushRunState) +                           ",\"pending\":" + String(pushCountNewer(lastSyncedTs)) +                           ",\"passes\":" + String(pushPassCount) +                           ",\"lastUploadTs\":" + String(pushLastUploadTs) +                           ",\"lastAccepted\":" + String(pushLastAccepted) +                           ",\"lastUploadSec\":" + String(pushLastUploadMs ? (millis() - pushLastUploadMs) / 1000 : 0) +                           ",\"nextInSec\":" + String(pushNextAtMs ? (int32_t)(pushNextAtMs - millis()) / 1000 : -1) +                           ",\"needBackfill\":" + (pushNeedBackfill ? "true" : "false") +                           ",\"backfillBefore\":" + String(pushBackfillBeforeTs) +                           ",\"bufN\":" + String(bufferSize) +                           ",\"token\":" + (PHT_API_TOKEN[0] ? "true" : "false") + "}" +
             ",\"pr1Raw\":" + String(digitalRead(PIN_PR1)) +
             ",\"pr1Safe\":" + ((!pr1BatteryDirect || ps.battVolt <= PR1_HARD_MAX) ? "true" : "false") +
             ",\"pr1Thr\":{\"on\":" + String(PR1_ON_V, 2) +
                            ",\"off\":" + String(PR1_OFF_V, 2) +
                            ",\"hardMax\":" + String(PR1_HARD_MAX, 2) + "}" +
             ",\"topRefresh\":" + String(topRefreshCount) +                // V2.1.1-c：顶栏即时重画计数（自检/验证用）
             ",\"wifi\":\"" + String(WiFi.status()==WL_CONNECTED?"已连接":(apEnabled?"AP":"未连接")) + "\"" +
             ",\"wifiSsid\":\"" + String(WiFi.status()==WL_CONNECTED ? WiFi.SSID() : String(wifiCfg.ssid)) + "\"" +
             ",\"wifiCount\":" + String(wifiCount) +
             ",\"wifiRssi\":" + String(WiFi.status()==WL_CONNECTED ? WiFi.RSSI() : wifiRssi) +
             // cfgHash：当前 wifiCfg(SSID+密码) 的 djb2 哈希——自检用，确认“保存后当前组已刷新”
             ",\"cfgHash\":\"" + String(wifiCfgHash(), HEX) + "\"" +
             ",\"ip\":\"" + (WiFi.status()==WL_CONNECTED ? WiFi.localIP().toString() : String("-")) + "\"" +
             // ---- 地址策略（V2.1 静态 IP）----
             ",\"useIP\":" + (wifiCfg.useIP ? "true" : "false") +
             ",\"ipLast\":" + String(wifiCfg.ipLast) +
             ",\"gw\":\"" + wifiCfg.gw.toString() + "\"" +
             ",\"mask\":\"" + wifiCfg.mask.toString() + "\"" +
             ",\"ipFallback\":" + (wifiStaticFallback ? "true" : "false") +
             // ---- 无线互斥状态（WiFi/蓝牙不同时开）----
             ",\"wireless\":" + String(curWireless) +                       // 0=无 1=WiFi 2=蓝牙
             ",\"ble\":" + (bleOn ? "true" : "false") +
             ",\"bleLink\":" + (bleLink ? "true" : "false") +
             ",\"wlForce\":" + String(wlForce) +
             // caps：BLE 已停用（ENABLE_BLE=0），当前只有 WiFi 一条路径 → 全开；
             //   保留 caps 机制，远期复活 BLE 时前端零改动。
             //   wifi = 系统卡里通往 /wifi 完整配置页的链接（蓝牙模式无 HTTP 服务，链接是死的 → 关掉）
             ",\"caps\":" + String(overBLE
               ? "{\"files\":false,\"ops\":false,\"sys\":true,\"ota\":false,\"wifi\":false}"
               : "{\"files\":true,\"ops\":true,\"sys\":true,\"ota\":true,\"wifi\":true}") +
             "}";
  return s;
}

String buildHistoryJson(int page, int count, bool applyFilter) {
    if (count <= 0 || count > 300) count = 200;
    if (!LittleFS.exists("/log.csv")) { return "{\"data\":[],\"total\":0,\"page\":0}";}
    File file = LittleFS.open("/log.csv", "r");
    if (!file) { return "{\"data\":[],\"total\":0,\"page\":0}";}
    file.readStringUntil('\n');
    unsigned long totalLines = 0;
    while (file.available()) { if (file.read() == '\n') totalLines++; }
    file.close();
    if (totalLines == 0) { return "{\"data\":[],\"total\":0,\"page\":0}";}
    unsigned long startLine = 0;
    if (page == 0) startLine = (totalLines > count) ? (totalLines - count) : 0;
    else {
      unsigned long totalPages = (totalLines + count - 1) / count;
      if (page >= totalPages) { return "{\"data\":[],\"total\":" + String(totalLines) + ",\"page\":" + String(page) + "}";}
      startLine = totalLines - (page + 1) * count;
      if (startLine > totalLines) startLine = 0;
    }
    file = LittleFS.open("/log.csv", "r");
    file.readStringUntil('\n');
    for (unsigned long i = 0; i < startLine; i++) file.readStringUntil('\n');
    const int MAX_READ = 300;
    // 放堆上：本 builder 会被 HTTP(async_tcp 任务) 与 BLE(loop 任务) 两条路径调用，
    // 4.8KB 数组放栈上曾把蓝牙协议栈任务压崩（栈溢出 → panic → 重启）
    DataPoint* tempData = (DataPoint*)malloc(sizeof(DataPoint) * MAX_READ);
    if (!tempData) return "{\"data\":[],\"total\":0,\"page\":0}";
    int readCount = 0;
    while (file.available() && readCount < count && readCount < MAX_READ) {
      String line = file.readStringUntil('\n');
      if (line.length() == 0) continue;
      int idx1 = line.indexOf(','), idx2 = line.indexOf(',', idx1 + 1);
      int idx3 = line.indexOf(',', idx2 + 1), idx4 = line.indexOf(',', idx3 + 1);
      int idx5 = line.indexOf(',', idx4 + 1);   // 第6列=设备模式（旧数据可能没有）
      if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx4 < 0) continue;
      String timeStr = line.substring(idx1 + 1, idx2);
      String tempStr = line.substring(idx2 + 1, idx3);
      String humStr  = line.substring(idx3 + 1, idx4);
      String presStr = (idx5 < 0) ? line.substring(idx4 + 1) : line.substring(idx4 + 1, idx5);
      uint32_t timestamp = 0; struct tm tm;
      if (strptime(timeStr.c_str(), "%Y-%m-%d %H:%M:%S", &tm)) timestamp = mktime(&tm);
      else { int h,m,s; if (sscanf(timeStr.c_str(),"%d:%d:%d",&h,&m,&s)==3) timestamp = h*3600+m*60+s; }
      tempData[readCount].time = timestamp;
      tempData[readCount].temp = parseVal(tempStr);
      tempData[readCount].humidity = parseVal(humStr);
      tempData[readCount].pressure = parseVal(presStr);
      readCount++;
    }
    file.close();
    if (applyFilter && readCount > 1) applyFilterToData(tempData, readCount);
    String json = "[";
    for (int i = 0; i < readCount; i++) {
      if (i > 0) json += ",";
      bool validDate = hasValidDate(tempData[i].time);
      json += "{\"t\":" + String(tempData[i].time) + ",\"tp\":" + jsonNum(tempData[i].temp) +
              ",\"hm\":" + jsonNum(tempData[i].humidity) + ",\"pr\":" + jsonNum(tempData[i].pressure) +
              ",\"ah\":" + jsonNum(absHumidity(tempData[i].temp, tempData[i].humidity)) +
              ",\"vd\":" + String(validDate ? 1 : 0) + "}";
    }
    json += "]";
    free(tempData);
    return "{\"data\":" + json + ",\"total\":" + String(totalLines) + ",\"page\":" + String(page) + "}";
}

String buildDailyAvgJson() {
    if (!avgSlots) { return "{\"days\":0,\"data\":[]}";}
    int step = 6;  // 3分钟一格
    int maxDays = 0;
    for (int i = 0; i < AVG_SLOTS; i++) if (avgSlots[i].cnt > maxDays) maxDays = avgSlots[i].cnt;
    String s = "{\"days\":" + String(maxDays) + ",\"grid\":" + String(step * INTERVAL_SEC) + ",\"data\":[";
    bool first = true;
    for (int i = 0; i < AVG_SLOTS; i += step) {
      float st = 0, sh = 0, sp = 0; int c = 0;
      for (int j = i; j < i + step && j < AVG_SLOTS; j++)
        if (avgSlots[j].cnt > 0) { st += avgSlots[j].sumT / avgSlots[j].cnt; sh += avgSlots[j].sumH / avgSlots[j].cnt; sp += avgSlots[j].sumP / avgSlots[j].cnt; c++; }
      if (c > 0) {
        if (!first) s += ","; first = false;
        s += "{\"s\":" + String(i * INTERVAL_SEC) + ",\"tp\":" + String(st / c, 2) + ",\"hm\":" + String(sh / c, 2) + ",\"pr\":" + String(sp / c, 2) + "}";
      }
    }
    s += "]}";
    return s;
}

String buildAnomalyJson() {
    // 设计文档：气压距平曲线【仅固定模式可用】——移动模式告知前端不可用（仍回报真实的槽数/天数，便于自检）
    {
      int slots0 = 0, days0 = 0;
      if (todaySlots) for (int i = 0; i < AVG_SLOTS; i++) if (todaySlots[i].cnt > 0) slots0++;
      if (avgSlots)   for (int i = 0; i < AVG_SLOTS; i++) if (avgSlots[i].cnt > days0) days0 = avgSlots[i].cnt;
      if (deviceMode != 0) {
        return "{\"date\":\"\",\"fix\":false,\"slots\":" + String(slots0) +
                    ",\"days\":" + String(days0) + ",\"data\":[]}";}
    }
    if (!todaySlots || !avgSlots) { return "{\"date\":\"\",\"fix\":true,\"slots\":0,\"days\":0,\"data\":[]}";}
    int step = 2;  // 1分钟一格
    String s = "{\"date\":\"" + String(todayExt.date) + "\",\"fix\":true";
    int slots = 0, days = 0;                    // 今日已采槽数 / 基线天数（自检 + 前端提示用）
    for (int i = 0; i < AVG_SLOTS; i++) {
      if (todaySlots[i].cnt > 0) slots++;
      if (avgSlots[i].cnt > days) days = avgSlots[i].cnt;
    }
    s += ",\"slots\":" + String(slots) + ",\"days\":" + String(days);
    s += ",\"data\":[";
    bool first = true;
    for (int i = 0; i < AVG_SLOTS; i += step) {
      float curP = 0, avgP = 0; int curC = 0, avgC = 0;
      for (int j = i; j < i + step && j < AVG_SLOTS; j++) {
        if (todaySlots[j].cnt > 0) { curP += todaySlots[j].sumP / todaySlots[j].cnt; curC++; }
        if (avgSlots[j].cnt > 0) { avgP += avgSlots[j].sumP / avgSlots[j].cnt; avgC++; }
      }
      if (curC > 0 && avgC > 0) {
        curP /= curC; avgP /= avgC;
        if (!first) s += ","; first = false;
        s += "{\"s\":" + String(i * INTERVAL_SEC) + ",\"dP\":" + String(curP - avgP, 2) + ",\"pr\":" + String(curP, 2) + "}";
      }
    }
    s += "]}";
    return s;
}

String buildTodayStatsJson() {
    float anomaly = 0; bool hasAnom = false;
    if (deviceMode == 0 && todaySlots && avgSlots) {      // 距平仅固定模式（设计文档）
      time_t now = time(nullptr);
      if (timeSynced && now >= 1000000000) {
        struct tm tm; localtime_r(&now, &tm);
        int slot = tm.tm_hour * 120 + tm.tm_min * 2 + (tm.tm_sec >= 30 ? 1 : 0);
        for (int k = 0; k < 20; k++) {
          int s = slot - k; if (s < 0) break;
          if (todaySlots[s].cnt > 0 && avgSlots[s].cnt > 0) {
            anomaly = (todaySlots[s].sumP / todaySlots[s].cnt) - (avgSlots[s].sumP / avgSlots[s].cnt);
            hasAnom = true; break;
          }
        }
      }
    }
    String s = "{\"date\":\"" + String(todayExt.date) + "\"";
    s += ",\"fix\":" + String(deviceMode == 0 ? "true" : "false");
    s += ",\"anomaly\":" + (hasAnom ? String(anomaly, 2) : String("null"));
    s += ",\"trend24\":"  + jsonNum(trendP24);
    s += ",\"trend24T\":" + jsonNum(trendT24);
    s += ",\"trend24H\":" + jsonNum(trendH24);
    auto addExt = [&](const char* key, ExtremeValue& ev) {
      s += ",\"" + String(key) + "\":{\"v\":" + (ev.set ? String(ev.v, 2) : String("null")) + ",\"t\":\"" + (ev.set ? fmtHM(ev.t) : String("-")) + "\"}";
    };
    addExt("tempMax", todayExt.tempMax); addExt("tempMin", todayExt.tempMin);
    addExt("humMax", todayExt.humMax); addExt("humMin", todayExt.humMin);
    addExt("presMax", todayExt.presMax); addExt("presMin", todayExt.presMin);
    s += "}";
    return s;
}

// ===== 网页 logo / favicon（V2.1.1 · 交互和可视化 第2条「新增网页logo」）=====
//   母题 = **雷暴：云 + 闪电 + 气压尖峰** —— 不是随手挑的天气符号：
//     本项目的初衷就是**捕获雷暴过境时的超压**，所以画的是「雷电劈下来、顶起那一下气压尖峰」，
//     这正是本项目区别于任何天气 App 的地方。
//     （2026-09-29 用户反馈：上一版「上升曲线 + 端点圆点」看着像炒股软件。）
//   三色各有语义：云 = 深板岩 #34495e ／ 闪电 = 琥珀 #f39c12 ／ 气压尖峰 = 曲线绿 #27ae60。
//   **同一母题、两个复杂度**（响应式 logo 的常规做法）：
//     页头 logo = C 全要素（云 + 闪电 + 尖峰）—— 讲故事；30px（三元素比单一图形更吃像素）
//     favicon   = F 精简版（云 + 闪电，去掉尖峰）—— 16px 下三个元素会互相挤成一团
//   favicon 的云为什么用**深色实心**：#bdc3c7 那种浅灰云在白色标签栏上 16px 会被冲淡成一团雾
//     —— 小尺寸的可读性本质是**对比度**问题，不是形状问题。代价是深色标签栏下会糊成一块黑，
//     所以 SVG 内用 `@media(prefers-color-scheme:dark)` 把云翻成浅色（不支持该查询的浏览器
//     就退回深色云，即我们本来就要的那个，**没有下行风险**）。
//   favicon 为什么用 **SVG data URI**：零额外请求（ESP32 少一次连接）、无需新路由、AP 配网模式下同样有效；
//     而页面 HTML 是在设备上拼成 String 再发的，塞进 HTML 的每 KB 都要每次请求重拼 → 矢量优先。
//   ⚠️ `#` 等字符必须百分号编码；本串由脚本 encodeURIComponent 生成 —— **改图形要重新编码，别手改**。
//   ⚠️ 未做 apple-touch-icon：iOS 不支持 SVG 当 apple-touch-icon，必须 180×180 PNG
//      → 想让「手机添加到主屏幕」也好看，得再加一个 /logo.png 路由（见 实现细节 D26）。
// PWA manifest（Android「添加到主屏幕」用；iOS 走 apple-touch-icon，不依赖它）
//   · display:standalone → 从主屏打开时全屏、没有浏览器工具栏（看图表更舒服）
//   · purpose:"any maskable" → 一枚文件两种渲染；因图形只占 55%，被裁成圆形也不缺角
//   · theme_color / background_color 用同一深板岩，安卓工具栏与启动屏不会突然变白
static const char* WEB_MANIFEST = R"manifest({"name":"微型气象站 PHT_2_X","short_name":"气象站","lang":"zh-CN","start_url":"/","display":"standalone","background_color":"#2c3e50","theme_color":"#2c3e50","icons":[{"src":"/icon-192.png","sizes":"192x192","type":"image/png","purpose":"any maskable"},{"src":"/icon-512.png","sizes":"512x512","type":"image/png","purpose":"any maskable"}]})manifest";
static const char* WEB_ICONS =
  "<link rel=\"icon\" type=\"image/svg+xml\" href=\"data:image/svg+xml,"
  "%3Csvg%20xmlns%3D'http%3A%2F%2Fwww.w3.org%2F2000%2Fsvg'%20viewBox%3D'0%200%2048%2048'%3E%3Cstyle"
  "%3E%40media(prefers-color-scheme%3Adark)%7B%23c%7Bfill%3A%23ecf0f1%7D%7D%3C%2Fstyle%3E%3Cg%20id%"
  "3D'c'%20fill%3D'%2334495e'%3E%3Ccircle%20cx%3D'17'%20cy%3D'15'%20r%3D'7.5'%2F%3E%3Ccircle%20cx%3"
  "D'28.5'%20cy%3D'17.5'%20r%3D'8.5'%2F%3E%3Crect%20x%3D'10.5'%20y%3D'16'%20width%3D'27'%20height%3"
  "D'8.5'%20rx%3D'4.25'%2F%3E%3C%2Fg%3E%3Cpath%20d%3D'M26%2020%20L19.5%2032%20h4.5%20L21.5%2042%20L"
  "30.5%2030%20h-4.5%20L30%2020%20Z'%20fill%3D'%23f39c12'%2F%3E%3C%2Fsvg%3E"
  "\">"
  // iOS 主屏图标：**必须 PNG** —— iOS Safari 不接受 SVG 作为 apple-touch-icon（独立机制）。
  //   由 /apple-touch-icon.png 路由从 flash 直发（3 KB，见 web_icon.h）。
  "<link rel=\"apple-touch-icon\" sizes=\"180x180\" href=\"/apple-touch-icon.png\">"
  // Android「添加到主屏幕」走 PWA manifest（192/512 两档 PNG，见 web_icon.h）
  "<link rel=\"manifest\" href=\"/manifest.webmanifest\">"
  "<meta name=\"theme-color\" content=\"#2c3e50\">"
  // iOS 主屏标签：不给的话各页会各用各的 <title>（日志控制台/WiFi 设置…）
  "<meta name=\"apple-mobile-web-app-title\" content=\"气象站\">";
// 页头 logo（母题 C 全要素；深色云在白卡片上对比度好，故不需要深色模式适配）
static const char* WEB_LOGO_SVG =
  "<svg viewBox=\"0 0 48 48\" width=\"30\" height=\"30\" aria-hidden=\"true\" style=\"flex:0 0 auto\"><g fill=\"#34495e\"><circle cx=\"16\" cy=\"11\" r=\"6.5\"/><circle cx=\"26.5\" cy=\"13\" r=\"7.5\"/><rect x=\"10\" y=\"11.5\" width=\"24\" height=\"7.5\" rx=\"3.75\"/></g><path d=\"M25 17 L19 27 h4 L20.5 34 L28.5 25 h-4 L28 17 Z\" fill=\"#f39c12\"/><path d=\"M4 41 H15 L20.5 32.5 L26 41 H44\" fill=\"none\" stroke=\"#27ae60\" stroke-width=\"3.1\" stroke-linecap=\"round\" stroke-linejoin=\"round\"/></svg>";

void startServer() {
  // ----- iOS 主屏图标（apple-touch-icon）-----
  //   ⚠️ iOS Safari **不接受 SVG** 作为 apple-touch-icon（它是独立于现代 favicon 的一套老机制），
  //      所以单独用一张 180×180 PNG（见 web_icon.h，3042 字节，存 flash）。
  //   从 flash 直发（send_P）→ 不占 RAM；给长缓存，浏览器只在首次拉取。
  //   iOS 也会自动探测根路径 /apple-touch-icon.png，所以即使某页漏了标签也能取到；
  //   /apple-touch-icon-precomposed.png 是 iOS<7 的旧名，一并放行。
  auto serveTouchIcon = []() {
    server.sendHeader("Cache-Control", "public, max-age=86400");
    server.send_P(200, "image/png", (PGM_P)WEB_ICON_180_PNG, sizeof(WEB_ICON_180_PNG));
  };
  server.on("/apple-touch-icon.png", serveTouchIcon);
  server.on("/apple-touch-icon-precomposed.png", serveTouchIcon);

  // ---- Android 主屏图标 + PWA manifest ----
  //   与 apple-touch-icon 同样从 flash 直发（send_P），不占 RAM。
  auto serveIcon192 = []() {
    server.sendHeader("Cache-Control", "public, max-age=86400");
    server.send_P(200, "image/png", (PGM_P)WEB_ICON_192_PNG, sizeof(WEB_ICON_192_PNG));
  };
  server.on("/icon-192.png", serveIcon192);
  auto serveIcon512 = []() {
    server.sendHeader("Cache-Control", "public, max-age=86400");
    server.send_P(200, "image/png", (PGM_P)WEB_ICON_512_PNG, sizeof(WEB_ICON_512_PNG));
  };
  server.on("/icon-512.png", serveIcon512);
  server.on("/manifest.webmanifest", []() {
    server.sendHeader("Cache-Control", "public, max-age=86400");
    server.send(200, "application/manifest+json; charset=utf-8", WEB_MANIFEST);
  });

  // ----- 主页 -----
  server.on("/", []() {
    noteWebActivity();          // 会话续期 + 「网页已打开」确认（立刻收横幅，别傻等）
    String page = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>)rawliteral" + pushTitle(true) + R"rawliteral(</title>)rawliteral" + String(WEB_ICONS) + R"rawliteral(
<style>
body{font-family:sans-serif;margin:10px;background:#f5f5f5}
.card{background:white;padding:15px;border-radius:10px;box-shadow:0 2px 6px rgba(0,0,0,.1);margin-bottom:15px}
h2{font-size:18px;margin:0 0 10px 0}
canvas{height:200px;background:white;border-radius:6px;box-sizing:border-box;touch-action:none;margin-bottom:5px}
.info{font-size:13px;color:#666;margin:8px 0}
.btn{display:inline-block;padding:8px 16px;margin:4px;text-decoration:none;border-radius:6px;font-size:13px;color:white;border:none;cursor:pointer}
.btn-download{background:#28a745}
.btn-clear{background:#dc3545}
.btn-sync{background:#ff9800}
.btn-filter{background:#17a2b8}
.btn-nav{background:#6c757d}
.btn-manual{background:#e67e22}
.btn-log{background:#8e44ad}
#status{font-size:12px;color:#888;margin:5px 0}
.controls{display:flex;gap:10px;align-items:center;margin:10px 0;flex-wrap:wrap}
.controls select{padding:5px;border-radius:4px;border:1px solid #ddd}
/* 数值行：用 grid+auto-fit 而不是 flex —— flex 下"换行后落单的那一个"会被 flex:1 撑满整行
   （2026-09-30 宽屏改成多列后暴露：供电状态/今日极值 的 6 个值在 630px 列里换行，最后一个变成巨大蓝盒子）。
   grid 的 auto-fit 会**折叠空轨道**，所以 3 值的卡（页头）照样铺满，6 值的卡则自动 6 列/3 列，
   落单的那个只占一格。手机档（约 350px）算下来仍是 3 列 2 行，与改动前一致。 */
.latest-values{display:grid;grid-template-columns:repeat(auto-fit,minmax(90px,1fr));gap:15px;margin:10px 0}
.latest-values .val{background:#f0f8ff;padding:8px 15px;border-radius:8px;text-align:center;min-width:0}
/* 值是"文字"而不是数字的卡（供电状态）需要更宽的格子：内容是 emoji + 文字（如 "⚡外部供电"），
   实测 ≈120px，再加 30px 内边距 → 格子得 ≥160px 才不断行（132px 试过，仍会在"供/电"之间断）。
   只给这张卡加 .wide，别的卡不受影响。 */
.latest-values.wide{grid-template-columns:repeat(auto-fit,minmax(160px,1fr))}
.latest-values .val .num{font-size:24px;font-weight:bold}
.latest-values .val .label{font-size:12px;color:#666}
.temp-color{color:#e74c3c}
.hum-color{color:#3498db}
.pres-color{color:#27ae60}

/* ===== 宽屏布局（2026-09-30）：≥1000px 时改成多列仪表盘 =====
   动机：画布高度恒为 200px，宽度却跟着卡片无限涨 —— 4K 上宽高比到过 10:1，曲线被拉平，
         小回落/台阶这些细节全被压掉。手机档（约 380px → 1.9:1）本来是对的，所以本改动的
         原则是 **只加宽屏规则，窄屏逐像素不动**。
   ⚠️ auto-fit 的 **列数由 minmax 的第二个值(max)决定**，不是第一个（CSS Grid 规范：max 确定时
      按 max 算重复次数）。所以这里 max=660 → 2038px 下正好 3 列；2760px → 4 列；3420px → 5 列，
      **每列永远不超过 660px**，图再也不会被拉平。
   为什么只改布局、不动 resizeCanvas：它读的是 card.clientWidth，卡片变窄它自动就算对了。
      唯一不能做的是"卡片仍很宽、只给 canvas 加 max-width" —— 那会把画布横向压扁
      （行内 style.width 与绘图缓冲 cnvs.width 会不一致）。 */
@media (min-width:1000px){
  body{display:grid;
       grid-template-columns:repeat(auto-fit,minmax(440px,660px));
       gap:15px;justify-content:center;align-items:start}
  .card{margin-bottom:0}          /* 间距交给 grid gap，避免叠加成双倍 */
}
</style>
</head>
<body>

<div class="card">
  <div style="display:flex;align-items:center;gap:9px">)rawliteral" + String(WEB_LOGO_SVG) + R"rawliteral(
    <h2 style="margin:0">)rawliteral" + pushTitle(false) + R"rawliteral( <span style="font-size:12px;color:#888;" id="filterLabel">滤波: 卡尔曼</span></h2>
  </div>
  <div class="info">间隔: )rawliteral" + String(INTERVAL_SEC) + R"rawliteral(秒 | 显示: <span id="cnt">-</span>条 | SD: <span id="sdStatus">-</span></div>
  <div id="status">加载中...</div>
  <div class="latest-values">
    <div class="val"><div class="num temp-color" id="curTemp">--.-</div><div class="label">🌡️ 温度 (°C)</div></div>
    <div class="val"><div class="num hum-color" id="curHum">--.-</div><div class="label">💧 湿度 (%RH)</div></div>
    <div class="val"><div class="num pres-color" id="curPres">----.--</div><div class="label">📊 气压 (hPa)</div></div>
  </div>
</div>
<div class="card"><h2>🔋 供电状态</h2>
  <div class="latest-values wide">
    <div class="val"><div class="num" style="color:#8e44ad" id="pwrMode">--</div><div class="label">供电方式</div></div>
    <div class="val"><div class="num" style="color:#f39c12" id="battPct">--</div><div class="label">电量 (%)</div></div>
    <div class="val"><div class="num" style="color:#27ae60" id="battVolt">--.--</div><div class="label">电池电压 (V)</div></div>
    <div class="val"><div class="num" style="color:#e67e22" id="battCur">--</div><div class="label">电流 (mA)</div></div>
    <div class="val"><div class="num" style="color:#16a085" id="chgState">--</div><div class="label">充电阶段</div></div>
    <div class="val"><div class="num" style="color:#3498db" id="wifiSt">--</div><div class="label">WiFi</div></div>
  </div>
</div>
<!-- ===== 设备模式（固定/移动）：切换后立即联动充电策略，写入 NVS ===== -->
<div class="card">
  <h2>🔧 设备模式 <span style="font-size:12px;color:#888">切换后立即生效</span></h2>
  <div class="info">当前：<b id="modeLbl">--</b></div>
  <button class="btn" style="background:#2c7be5" onclick="setMode(0)">🏠 固定 FIX（慢充 · 80% 停充）</button>
  <button class="btn" style="background:#e67e22" onclick="setMode(1)">🎒 移动 MOV（快充 · 充满）</button>
  <span id="modeStatus" class="info"></span>
  <div class="info">固定：慢充、充到约 80% 停充（保护电池寿命）｜移动：快充、充满为止</div>
</div>
<div class="card"><h2>🌡️ 温度趋势</h2><canvas id="cTemp"></canvas></div>
<div class="card"><h2>💧 湿度趋势</h2><canvas id="cHum"></canvas><div class="info"><span style="color:#3498db">━</span> 相对湿度 <b>%RH</b>（左轴）｜<span style="color:#16a085">━</span> 绝对湿度 <b>g/m³</b>（右轴）</div></div>
<div class="card"><h2>📊 气压趋势</h2><canvas id="cPres"></canvas></div>
<div class="card">
  <h2>🏆 今日极值 <span style="font-size:12px;color:#888;" id="extDate"></span></h2>
  <div class="latest-values">
    <div class="val"><div class="num temp-color" id="exTMax">--.-</div><div class="label">🌡️ 最高 @<span id="exTMaxT">-</span></div></div>
    <div class="val"><div class="num temp-color" id="exTMin">--.-</div><div class="label">🌡️ 最低 @<span id="exTMinT">-</span></div></div>
    <div class="val"><div class="num hum-color" id="exHMax">--.-</div><div class="label">💧 最高 @<span id="exHMaxT">-</span></div></div>
    <div class="val"><div class="num hum-color" id="exHMin">--.-</div><div class="label">💧 最低 @<span id="exHMinT">-</span></div></div>
    <div class="val"><div class="num pres-color" id="exPMax">----.--</div><div class="label">📊 最高 @<span id="exPMaxT">-</span></div></div>
    <div class="val"><div class="num pres-color" id="exPMin">----.--</div><div class="label">📊 最低 @<span id="exPMinT">-</span></div></div>
  </div>
  <div class="info">📈 当前气压距平: <b id="curAnom">--</b> hPa</div>
  <div class="info">🕐 24h变率: 🌡️ <span id="trend24T">--</span> °C | 💧 <span id="trend24H">--</span> g/m³ | 📊 <span id="trend24">--</span> hPa</div>
</div>
<div class="card" id="cardAnom"><h2>📈 今日气压距平曲线 <span id="anomHint" style="font-size:12px;color:#999"></span></h2><canvas id="cAnom"></canvas></div>
<div class="controls">
  <button class="btn btn-nav" onclick="loadPrevPage()">◀ 更早</button>
  <span id="pageInfo">第 1 页 / 共 1 页</span>
  <button class="btn btn-nav" onclick="loadNextPage()">更新 ▶</button>
  <button class="btn btn-filter" onclick="loadLatest()">📌 最新</button>
  <select id="pageSizeSelect" onchange="changePageSize()">
    <option value="100">100条</option><option value="200" selected>200条</option><option value="300">300条</option>
  </select>
  <button class="btn btn-filter" onclick="toggleFilter()">🔀 切换滤波</button>
</div>
<!-- ==== 能力分组：后端 /status 里的 caps 可整卡隐藏（蓝牙模式：files:false, ops:false, ota:false）==== -->
<div class="card" id="card-files"><h2>📁 数据与文件</h2>
  <a class="btn btn-download" href="/download">📥 下载CSV</a>
  <a class="btn btn-download" href="/sdlist" style="background:#6c42c1">📂 SD文件</a>
  <a class="btn btn-clear" href="/clear" onclick="return confirm('清空所有数据?')">🗑️ 清空数据</a>
</div>
<div class="card" id="card-ops"><h2>⚙️ 采集与操作</h2>
  <button class="btn btn-sync" onclick="syncTime()">🕐 同步时间</button>
  <button class="btn btn-sync" onclick="doShowToast(this)" style="background:#2e7d32">📺 屏幕显示</button>
  <a class="btn btn-log" href="/web-log">📟 日志</a>
</div>
<!-- 原「📶 配网（WiFi）」蓝牙配网卡已于 V2.1 删除（BLE 搁置，ENABLE_BLE=0）；
     WiFi 配网统一走系统卡里的「📶 WiFi设置」链接（/wifi 完整配置页）。 -->
<div class="card" id="card-sys"><h2>🛠 系统</h2>
  <span id="card-wifi"><a class="btn btn-sync" href="/wifi">📶 WiFi设置</a></span>
<span id="card-battcal"><a class="btn btn-nav" href="/battcal">🔋 电量校准</a></span>
  <a class="btn btn-download" id="btn-ota" href="/update" style="background:#e74c3c">🔄 OTA升级</a>
  <button class="btn btn-clear" onclick="doReboot()">🔌 重启设备</button>
  <!-- 设备名：**无害的东西放明面**（朋友要能自己改），危险的才低调放 /keycfg。
       保存必须服务器验签通过才落盘 —— 连不上服务器会直接拒绝并提示。 -->
  <div style="margin-top:10px;font-size:12.5px">
    <span style="font-size:12px;color:#666;font-weight:600">设备名</span>
    <input id="dnIn" maxlength="24" placeholder="可中文，最多 24 字" style="padding:6px 8px">
    <button class="btn btn-sync" onclick="setDevName()">保存名称</button>
    <span id="dnTip" style="font-size:11.5px;color:#888"></span>
  </div>
  <!-- 密钥/推送配置入口：**故意低调**（系统卡末尾一行灰色小字，不是按钮）——
       那个页面的操作能让设备立刻与服务器失联，不适合做成显眼按钮。 -->
  <div style="margin-top:10px;font-size:11px;color:#a0aec0">
    <a href="/keycfg" style="color:#a0aec0;text-decoration:none">🔑 密钥 / 推送地址</a>
  </div>
</div>
<script>
/* ================= 传输层：统一走 HTTP fetch =================
   V2.1：BLE 已整体停用（固件 ENABLE_BLE=0），页面不再有 Web Bluetooth 路径。
   保留 apiFetch() 作为薄封装，各调用点零改动（历史：一份 HTML 双传输 WiFi/BLE）。 */
function apiFetch(url, opt){ return fetch(url, opt); }

// 保存设备名。**必须服务器验签通过才算成功**（见 devRenameTo 的说明）：
//   连不上服务器 / 时钟不可信 / 名字非法 → 直接拒绝，本地不落盘。
//   所以"保存成功"就意味着服务器已经记下了，两边不会长期不一致。
async function setDevName() {
  var t = document.getElementById('dnTip');
  var nm = document.getElementById('dnIn').value.trim();
  // 留空 = 清除显示名（网页标题回退出厂默认）。这是合法操作，但要确认一下，
  //   免得手滑清掉。服务端同样把空名字当"清除"，不是校验失败。
  if (!nm && !confirm('名字留空 = 清除显示名（网页标题回退为出厂默认）。\n继续？')) return;
  t.style.color = '#888'; t.textContent = '保存中…';
  try {
    var r = await fetch('/devname', {
      method: 'POST',
      headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
      body: 'dev=' + encodeURIComponent(nm)
    });
    var j = await r.json();
    if (j.ok) {
      t.style.color = '#16794a';
      t.textContent = '已保存：' + j.name + '（刷新页面后标题会变）';
    } else {
      t.style.color = '#c0392b';
      t.textContent = j.err || '保存失败';
    }
  } catch (e) {
    t.style.color = '#c0392b';
    t.textContent = '保存失败：' + e;
  }
}
</script>

<script>
var dpr = window.devicePixelRatio || 1;
var filterEnabled = true;
var currentPage = 0, totalPages = 1, pageSize = 200, isLoading = false;
var allData = [];

// ===== 网页能力开关 =====
//   WiFi 模式全开；蓝牙模式（只读数据 + 配 WiFi）由 /status 的 caps 关掉整张卡片：
//     caps = {"files":false,"ops":false,"ota":false}  →  ”数据与文件“”采集与操作“两张卡 + OTA 按钮直接隐藏
//   后端改 caps，前端零改动。card-XXX 是整卡，btn-XXX 是单个按钮。
var CAPS = {files:true, ops:true, ota:true};
// 网页重启：装壳后够不到 PCB 上的 EN 键时的兼底
function doReboot(){
  if(!confirm('重启设备？约 10 秒后回来（IP 不变）'))return;
  apiFetch('/reboot').then(function(){setTimeout(function(){location.reload();},12000);}).catch(function(){});
}
function applyCaps(caps){
  if(!caps)return;
  for(var k in caps){
    CAPS[k]=!!caps[k];
    var el=document.getElementById('card-'+k)||document.getElementById('btn-'+k);
    if(el)el.style.display=CAPS[k]?'':'none';
  }
}

function resizeCanvas(cnvs) {
  var card = cnvs.parentElement, s = window.getComputedStyle(card);
  var pl = parseInt(s.paddingLeft)||0, pr = parseInt(s.paddingRight)||0;
  var w = card.clientWidth - pl - pr;
  cnvs.width = w * dpr; cnvs.height = 200 * dpr;
  cnvs.style.width = w+'px'; cnvs.style.height = '200px';
  cnvs.getContext('2d').setTransform(dpr,0,0,dpr,0,0); cnvs._w = w; cnvs._h = 200;
}

// series: [{values:[…], color:'#rrggbb', axis:'l'|'r'}]
//   单序列时与原 draw() 完全等价；双序列时左轴给主量纲、右轴给量纲不同的叠加量
//   （例：湿度图的左轴 %RH + 右轴 g/m³ —— 量级差一个数量级，共用一根轴会把右轴压平）
function draw(c, labels, series, dayMarks) {
  var ctx = c.getContext('2d'), w = c._w, h = c._h;
  ctx.clearRect(0,0,w,h);
  var n = labels.length;
  if(!series||!series.length||n<2){ctx.fillStyle='#888';ctx.font='16px sans-serif';ctx.textAlign='center';ctx.fillText('数据不足',w/2,h/2);return;}
  var hasR=false;
  for(var k=0;k<series.length;k++) if((series[k].axis||'l')==='r') hasR=true;
  var pl=45, pr=hasR?46:15, pt=15, pb=35, pw=w-pl-pr, ph=h-pt-pb;
  function rangeOf(ax){
    var mn=Infinity,mx=-Infinity;
    for(var k=0;k<series.length;k++){
      if((series[k].axis||'l')!==ax) continue;
      var v=series[k].values;
      for(var i=0;i<v.length;i++){var x=v[i];if(x==null||isNaN(x))continue;if(x<mn)mn=x;if(x>mx)mx=x;}
    }
    if(mn===Infinity) return null;
    var r=(mx-mn)||1; mn-=r*0.05; mx+=r*0.05;
    return {mn:mn, r:mx-mn};
  }
  var RL=rangeOf('l'), RR=hasR?rangeOf('r'):null;
  ctx.strokeStyle='#eee'; ctx.lineWidth=1;
  for(var i=0;i<=4;i++){
    var y=pt+(ph/4)*i;
    ctx.beginPath();ctx.moveTo(pl,y);ctx.lineTo(w-pr,y);ctx.stroke();
    ctx.fillStyle='#888'; ctx.font='11px sans-serif';
    if(RL){ctx.textAlign='right';ctx.fillText((RL.mn+(RL.r/4)*(4-i)).toFixed(2),pl-5,y+4);}
    if(RR){ctx.textAlign='left'; ctx.fillText((RR.mn+(RR.r/4)*(4-i)).toFixed(2),w-pr+5,y+4);}
  }
  var sx=pw/(n-1);
  if(dayMarks&&dayMarks.length>0){
    ctx.strokeStyle='#ff9800';ctx.lineWidth=1;ctx.setLineDash([5,5]);
    for(var m=0;m<dayMarks.length;m++){var xi=dayMarks[m];if(xi>=0&&xi<n){var x=pl+sx*xi;ctx.beginPath();ctx.moveTo(x,pt);ctx.lineTo(x,pt+ph);ctx.stroke();}}
    ctx.setLineDash([]);
  }
  for(var k=0;k<series.length;k++){
    var s=series[k], R=((s.axis||'l')==='r')?RR:RL;
    if(!R) continue;
    var v=s.values, first=-1;
    for(var i=0;i<n;i++) if(v[i]!=null&&!isNaN(v[i])){first=i;break;}
    if(first<0) continue;
    ctx.strokeStyle=s.color; ctx.lineWidth=2.5; ctx.lineCap='round'; ctx.lineJoin='round';
    ctx.beginPath();
    var x0=pl+sx*first, y0=pt+ph-((v[first]-R.mn)/R.r)*ph;
    ctx.moveTo(x0, y0);
    for(var i=first+1; i<n; i++){
      if(v[i]==null||isNaN(v[i])){                       // 断点：抬笔，下一个有效点重新起笔
        var j=i; while(j<n&&(v[j]==null||isNaN(v[j]))) j++;
        if(j>=n) break;
        x0=pl+sx*j; y0=pt+ph-((v[j]-R.mn)/R.r)*ph;
        ctx.moveTo(x0, y0); i=j; continue;
      }
      var x1=pl+sx*i, y1=pt+ph-((v[i]-R.mn)/R.r)*ph;
      var xm=(x0+x1)/2, ym=(y0+y1)/2;
      ctx.quadraticCurveTo(x0, y0, xm, ym);
      x0=x1; y0=y1;
    }
    ctx.lineTo(x0, y0);
    ctx.stroke();
  }
  ctx.fillStyle='#888'; ctx.font='10px sans-serif'; ctx.textAlign='center';
  var step=Math.floor(n/8)||1;
  for(var i=0;i<n;i+=step) ctx.fillText(labels[i],pl+sx*i,h-pb+15);
}
function renderData(data){
  if(!data||!data.length){['cTemp','cHum','cPres'].forEach(function(id){var c=document.getElementById(id),ctx=c.getContext('2d');ctx.clearRect(0,0,c._w,c._h);ctx.fillStyle='#888';ctx.font='16px sans-serif';ctx.textAlign='center';ctx.fillText('无数据',c._w/2,c._h/2);});document.getElementById('cnt').textContent='0';document.getElementById('curTemp').textContent='--.-';document.getElementById('curHum').textContent='--.-';document.getElementById('curPres').textContent='----.--';return;}
  document.getElementById('cnt').textContent=data.length;
  var lst=data[data.length-1];
  function fv(v,d){return (v==null)?'--':v.toFixed(d);}
  if(lst){document.getElementById('curTemp').textContent=fv(lst.tp,2);document.getElementById('curHum').textContent=fv(lst.hm,2);document.getElementById('curPres').textContent=fv(lst.pr,2);}
  var lb=data.map(function(p){if(p.vd===0){var h=Math.floor(p.t/3600),m=Math.floor((p.t%3600)/60);return h+':'+(m<10?'0':'')+m;}var d=new Date(p.t*1000);return d.getHours()+':'+String(d.getMinutes()).padStart(2,'0');});
  var dm=[];for(var i=0;i<data.length;i++)if(data[i].dm===1)dm.push(i);
  function holdLast(arr){
    var out=[],last=null,first=null;
    for(var i=0;i<arr.length;i++){var v=arr[i];if(v==null||isNaN(v)){out.push(null);}else{out.push(v);last=v;if(first==null)first=v;}}
    if(first==null){for(var i=0;i<out.length;i++)out[i]=0;return out;}
    var prev=first;
    for(var i=0;i<out.length;i++){if(out[i]==null){out[i]=prev;}else{prev=out[i];}}
    return out;
  }
  // 湿度图两条线：左轴 %RH（相对湿度）、右轴 g/m³（绝对湿度）
  //   绝对湿度由设备端按统一口径 absHumidity() 算好、随 /history 与 /data 下发 —— 本页不重写公式
  var humSeries=[{values:holdLast(data.map(function(p){return p.hm;})),color:'#3498db'}];
  var ahv=data.map(function(p){return p.ah;});
  if(ahv.some(function(v){return v!=null&&!isNaN(v);}))humSeries.push({values:holdLast(ahv),color:'#16a085',axis:'r'});
  draw(document.getElementById('cTemp'),lb,[{values:holdLast(data.map(function(p){return p.tp;})),color:'#e74c3c'}],dm);
  draw(document.getElementById('cHum'),lb,humSeries,dm);
  draw(document.getElementById('cPres'),lb,[{values:holdLast(data.map(function(p){return p.pr;})),color:'#27ae60'}],dm);
}

function drawAnomaly(c, data){
  var ctx=c.getContext('2d'),w=c._w,h=c._h;
  ctx.clearRect(0,0,w,h);
  if(!data||data.length<2){ctx.fillStyle='#888';ctx.font='16px sans-serif';ctx.textAlign='center';ctx.fillText('数据不足（需积累数天历史）',w/2,h/2);return;}
  var mx=0.3;
  for(var i=0;i<data.length;i++){var a=Math.abs(data[i].dP);if(a>mx)mx=a;}
  mx*=1.2;
  var pl=45,pr=15,pt=15,pb=35,pw=w-pl-pr,ph=h-pt-pb;
  var mid=pt+ph/2;
  ctx.strokeStyle='#eee';ctx.lineWidth=1;
  for(var i=0;i<=4;i++){var y=pt+(ph/4)*i;ctx.beginPath();ctx.moveTo(pl,y);ctx.lineTo(w-pr,y);ctx.stroke();}
  ctx.strokeStyle='#999';ctx.lineWidth=1;
  ctx.beginPath();ctx.moveTo(pl,mid);ctx.lineTo(w-pr,mid);ctx.stroke();
  ctx.fillStyle='#888';ctx.font='11px sans-serif';ctx.textAlign='right';
  ctx.fillText('+'+mx.toFixed(1),pl-5,pt+4);
  ctx.fillText('0',pl-5,mid+4);
  ctx.fillText(mx.toFixed(1),pl-5,pt+ph+4);
  function yOf(v){return mid-(v/mx)*(ph/2-10);}
  var sx=pw/(data.length-1);
  ctx.beginPath();
  for(var i=0;i<data.length;i++){var x=pl+sx*i,y=yOf(data[i].dP);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y);}
  ctx.strokeStyle='#8e44ad';ctx.lineWidth=2;ctx.lineCap='round';ctx.lineJoin='round';
  ctx.stroke();
  var dotStep=Math.max(1,Math.floor(data.length/60));
  for(var i=0;i<data.length;i+=dotStep){
    var x=pl+sx*i,y=yOf(data[i].dP);
    ctx.fillStyle=data[i].dP>=0?'#e74c3c':'#3498db';
    ctx.beginPath();ctx.arc(x,y,2.5,0,Math.PI*2);ctx.fill();
  }
  ctx.fillStyle='#888';ctx.font='10px sans-serif';ctx.textAlign='center';
  var step=Math.floor(data.length/8)||1;
  for(var i=0;i<data.length;i+=step){
    var sec=data[i].s,h=Math.floor(sec/3600),m=Math.floor((sec%3600)/60);
    ctx.fillText(h+':'+(m<10?'0':'')+m,pl+sx*i,h-pb+15);
  }
}

function loadAnomStats(){
  apiFetch('/today-stats').then(function(r){return r.json();}).then(function(st){
    function set(id,v){var e=document.getElementById(id);if(e)e.textContent=v;}
    set('extDate','('+st.date+')');
    set('exTMax',st.tempMax.v==null?'--.-':st.tempMax.v.toFixed(2));
    set('exTMaxT',st.tempMax.t);
    set('exTMin',st.tempMin.v==null?'--.-':st.tempMin.v.toFixed(2));
    set('exTMinT',st.tempMin.t);
    set('exHMax',st.humMax.v==null?'--.-':st.humMax.v.toFixed(2));
    set('exHMaxT',st.humMax.t);
    set('exHMin',st.humMin.v==null?'--.-':st.humMin.v.toFixed(2));
    set('exHMinT',st.humMin.t);
    set('exPMax',st.presMax.v==null?'----.--':st.presMax.v.toFixed(2));
    set('exPMaxT',st.presMax.t);
    set('exPMin',st.presMin.v==null?'----.--':st.presMin.v.toFixed(2));
    set('exPMinT',st.presMin.t);
    var a=document.getElementById('curAnom');
    var fixOK=(st.fix!==false);
    if(a)a.textContent=fixOK?((st.anomaly==null)?'--':((st.anomaly>0?'+':'')+st.anomaly.toFixed(2))):'仅固定模式可用';
    function trend(id,v,fmt,th){
      var e=document.getElementById(id); if(!e)return;
      if(v==null){e.textContent='--';e.style.color='#888';return;}
      if(th==null)th=0.1;
      var arrow=v>th?'⬆️':(v<-th?'⬇️':'➡️');
      e.textContent=arrow+' '+(v>0?'+':'')+v.toFixed(fmt);
      e.style.color=v>th?'#e74c3c':(v<-th?'#3498db':'#888');
    }
    trend('trend24',  st.trend24,  2);        // 气压 hPa
    trend('trend24T', st.trend24T, 2);        // 温度 °C
    trend('trend24H', st.trend24H, 2, 0.5);   // 湿度：绝对湿度 g/m³，阈值单独定（见 ui_display.h 注释）
  }).catch(function(){});
  apiFetch('/anomaly').then(function(r){return r.json();}).then(function(a){
    var h=document.getElementById('anomHint');
    if(a.fix===false){
      if(h)h.textContent='（仅固定模式可用，切回固定后自动恢复）';
      var cv=document.getElementById('cAnom');
      if(cv){var cx=cv.getContext('2d'),W2=cv._w||300,H2=cv._h||200;cx.clearRect(0,0,W2,H2);cx.fillStyle='#aaa';cx.font='15px sans-serif';cx.textAlign='center';cx.fillText('仅固定模式可用',W2/2,H2/2);}
      return;
    }
    if(h)h.textContent=(a.days>0)?('（基线 '+a.days+' 天）'):'（还没有历史基线，先跑一天）';
    drawAnomaly(document.getElementById('cAnom'),a.data);
  }).catch(function(){});
}

function loadHistoryPage(page){
  if(isLoading)return;isLoading=true;
  document.getElementById('status').textContent='加载中...';
  var u='/history?page='+page+'&count='+pageSize+(filterEnabled?'&filter=1':'');
  apiFetch(u).then(function(r){return r.json();}).then(function(resp){
    if(!resp||!resp.data){document.getElementById('status').textContent='无更多数据';isLoading=false;return;}
    allData=resp.data;totalPages=Math.ceil(resp.total/pageSize);currentPage=resp.page;
    document.getElementById('pageInfo').textContent='第 '+(currentPage+1)+' 页 / 共 '+totalPages+' 页 (共 '+resp.total+' 条)';
    renderData(allData);document.getElementById('status').textContent='✅ 已加载';isLoading=false;
  }).catch(function(e){document.getElementById('status').textContent='❌ '+e.message;isLoading=false;});
}
function loadPrevPage(){if(currentPage<totalPages-1)loadHistoryPage(currentPage+1);else document.getElementById('status').textContent='已是最早';}
function loadNextPage(){if(currentPage>0)loadHistoryPage(currentPage-1);else loadHistoryPage(0);}
function loadLatest(){loadHistoryPage(0);}
function changePageSize(){pageSize=parseInt(document.getElementById('pageSizeSelect').value);loadHistoryPage(0);}
function toggleFilter(){filterEnabled=!filterEnabled;document.getElementById('filterLabel').textContent='滤波: '+(filterEnabled?'卡尔曼':'关闭');loadHistoryPage(currentPage);}
function syncTime(){apiFetch('/settime',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ts='+Math.floor(Date.now()/1000)}).then(function(){document.getElementById('status').textContent='🕐 已同步';setTimeout(function(){loadHistoryPage(0);},1000);});}
function doShowToast(b){var t=b.innerHTML;b.innerHTML='📺 已发送';apiFetch('/toast').then(function(){setTimeout(function(){b.innerHTML=t;},1500);});}
// 设备模式切换（0=固定 / 1=移动）：设备收到后立刻联动充电策略
function setMode(m){
  var st=document.getElementById('modeStatus');if(st)st.textContent='⏳ 切换中...';
  apiFetch('/mode?m='+m).then(function(r){return r.json();}).then(function(j){
    if(st)st.textContent='✅ 已切换到 '+(j.mode===0?'固定 FIX':'移动 MOV')+'，充电策略已生效';
    loadPowerStatus();
  }).catch(function(e){if(st)st.textContent='❌ '+e.message;});
}
document.addEventListener('keydown',function(e){if(e.key==='ArrowLeft')loadPrevPage();else if(e.key==='ArrowRight')loadNextPage();});
window.addEventListener('resize',function(){resizeCanvas(document.getElementById('cTemp'));resizeCanvas(document.getElementById('cHum'));resizeCanvas(document.getElementById('cPres'));resizeCanvas(document.getElementById('cAnom'));if(allData.length>0)renderData(allData);});
resizeCanvas(document.getElementById('cTemp'));resizeCanvas(document.getElementById('cHum'));resizeCanvas(document.getElementById('cPres'));resizeCanvas(document.getElementById('cAnom'));
loadHistoryPage(0);loadAnomStats();setInterval(function(){loadHistoryPage(currentPage);loadAnomStats();},30000);
function loadPowerStatus(){apiFetch('/status').then(function(r){return r.json();}).then(function(s){
  applyCaps(s.caps);   // 能力开关（蓝牙模式会整卡隐藏“数据与文件”/“采集与操作”与 OTA）
  document.getElementById('sdStatus').textContent=s.sdOK?'✅ 就绪':'❌ 未就绪';
  var pm=document.getElementById('pwrMode'),bp=document.getElementById('battPct'),bv=document.getElementById('battVolt'),ws=document.getElementById('wifiSt');
  if(ws&&s.wifiSsid){ws.title='已保存 '+s.wifiCount+' 组；当前 '+s.wifiSsid+(s.wifiRssi?' ('+s.wifiRssi+' dBm)':'');}
  if(pm){pm.textContent=s.charging?'🔌 充电中':(s.powered?'⚡ 外部供电':(s.onBattery?'🔋 电池':'-'));}
  // INA230 离线 → 电量/电压/电流/充电阶段整块作废，改为告警（不得按 0V 误判）
  var noIna=!s.inaOK;
  if(bp){bp.textContent=noIna?'⚠':(s.onBattery?(s.battPct+'%'):'-');}
  if(bv){bv.textContent=noIna?'--':(s.battVolt===null?'-':s.battVolt.toFixed(2));}
  var bc=document.getElementById('battCur');if(bc){bc.textContent=noIna?'--':((s.battCurrent>0?'+':'')+s.battCurrent.toFixed(0));}
  var cs=document.getElementById('chgState');if(cs){cs.textContent=noIna?'⚠ INA230 失联':(!s.charging?'未充电':(s.cvZone?'CV(将满)':'CC(充电中)'));}
  var ml=document.getElementById('modeLbl');if(ml){ml.textContent=noIna?(s.mode===1?'🎒 移动 MOV':'🏠 固定 FIX'):((s.mode===1?'🎒 移动 MOV':'🏠 固定 FIX')+' · '+(s.chgFast?'⚡快充':'🐢慢充')+(s.chgSlow?'(临时)':'')+(s.chgEnabled?'':' 🛡️已停充'));}
  if(ws){ws.textContent=s.wifi; }
});}
function loadStatus(){loadPowerStatus();}
setInterval(loadStatus,3000);
loadStatus();
</script></body></html>)rawliteral";
    server.send(200, "text/html; charset=utf-8", page);
  });

  // ----- 日志控制台页面 -----
  server.on("/web-log", []() {
    String onOff = webLoggingEnabled ? "🟢 已开启" : "🔴 已关闭";
    String checked = webLoggingEnabled ? "checked" : "";
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>📟 网页日志控制台</title>)rawliteral" + String(WEB_ICONS) + R"rawliteral(
<style>
*{box-sizing:border-box}
body{font-family:'Consolas','Courier New',monospace;margin:10px;background:#1e1e1e;color:#d4d4d4}
h2{font-size:16px;margin:0 0 10px;color:#569cd6}
#logBox{background:#252526;border:1px solid #3c3c3c;border-radius:6px;padding:10px;height:70vh;overflow-y:auto;font-size:12px;line-height:1.5;white-space:pre-wrap;word-break:break-all}
#logBox .info{color:#6a9955}
#logBox .warn{color:#dcdcaa}
#logBox .err{color:#f44747}
#logBox .ok{color:#4ec9b0}
.controls{display:flex;gap:10px;align-items:center;margin:10px 0;flex-wrap:wrap}
button{background:#0e639c;color:white;border:none;padding:6px 14px;border-radius:4px;cursor:pointer;font-size:13px}
button:hover{background:#1177bb}
button.danger{background:#a1260d}
.toggle-wrap{display:flex;align-items:center;gap:8px;color:#ccc;font-size:13px}
.switch{position:relative;display:inline-block;width:44px;height:22px}
.switch input{opacity:0;width:0;height:0}
.slider{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#555;transition:.3s;border-radius:22px}
.slider:before{position:absolute;content:"";height:16px;width:16px;left:3px;bottom:3px;background:white;transition:.3s;border-radius:50%}
input:checked+.slider{background:#4ec9b0}
input:checked+.slider:before{transform:translateX(22px)}
#status{color:#888;font-size:12px;margin-left:10px}
</style>
</head>
<body>
<h2>📟 网页日志控制台</h2>
<div class="controls">
  <label class="toggle-wrap">
    <span>日志记录</span>
    <label class="switch">
      <input type="checkbox" id="logToggle" )rawliteral" + checked + R"rawliteral( onchange="toggleLogging()">
      <span class="slider"></span>
    </label>
    <span id="toggleStatus">)rawliteral" + onOff + R"rawliteral(</span>
  </label>
  <button onclick="refreshLog()">🔄 刷新</button>
  <button class="danger" onclick="clearLog()">🗑️ 清空</button>
  <button onclick="autoScroll=!autoScroll;this.textContent=autoScroll?'📌 自动滚动：开':'📌 自动滚动：关'">📌 自动滚动：开</button>
  <span id="status">就绪</span>
</div>
<div id="logBox">⏳ 加载中...</div>
<script>
var autoScroll = true;
var lastCount = 0;

function refreshLog() {
  fetch('/weblog?count=' + lastCount).then(function(r){return r.json();}).then(function(data){
    var box = document.getElementById('logBox');
    if (data.count === 0) { box.innerHTML = '（暂无日志）'; return; }
    var html = '';
    for (var i = 0; i < data.lines.length; i++) {
      var line = data.lines[i];
      // 按内容着色
      var cls = '';
      if (line.indexOf('⚠️') >= 0 || line.indexOf('失败') >= 0 || line.indexOf('异常') >= 0) cls = 'warn';
      if (line.indexOf('❌') >= 0) cls = 'err';
      if (line.indexOf('✅') >= 0 || line.indexOf('就绪') >= 0) cls = 'ok';
      if (line.indexOf('📦') >= 0 || line.indexOf('🕐') >= 0 || line.indexOf('📶') >= 0) cls = 'info';
      html += '<div class="' + cls + '">' + escapeHtml(line) + '</div>';
    }
    box.innerHTML = html;
    lastCount = data.count;
    if (autoScroll) box.scrollTop = box.scrollHeight;
    document.getElementById('status').textContent = data.count + ' 条日志';
  }).catch(function(e){
    document.getElementById('status').textContent = '❌ ' + e.message;
  });
}

function escapeHtml(t){return t.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}

function toggleLogging() {
  var on = document.getElementById('logToggle').checked;
  fetch('/set-weblog?enable=' + (on ? '1' : '0')).then(function(r){return r.text();}).then(function(msg){
    document.getElementById('toggleStatus').textContent = on ? '🟢 已开启' : '🔴 已关闭';
    document.getElementById('status').textContent = msg;
  });
}

function clearLog() {
  if (confirm('清空缓冲区中的日志？')) {
    fetch('/clear-weblog').then(function(r){return r.text();}).then(function(msg){
      document.getElementById('logBox').innerHTML = '（已清空）';
      lastCount = 0;
      document.getElementById('status').textContent = msg;
    });
  }
}

refreshLog();
setInterval(refreshLog, 2000);
</script>
</body></html>)rawliteral";
    server.send(200, "text/html; charset=utf-8", html);
  });

  // ----- 日志数据 API -----
  server.on("/weblog", []() {
    int fromCount = server.arg("count").toInt();
    String json = "{\"lines\":[";
    int startIdx;
    if (fromCount >= webLogCount || fromCount == 0) {
      // 返回全部
      int total = webLogCount;
      if (total > 100) total = 100; // 最多返回100条
      if (total == 0) { server.send(200, "application/json", "{\"lines\":[],\"count\":0}"); return; }
      startIdx = (webLogHead - total + WEB_LOG_MAX) % WEB_LOG_MAX;
      for (int i = 0; i < total; i++) {
        if (i > 0) json += ",";
        int idx = (startIdx + i) % WEB_LOG_MAX;
        String escaped = String(webLogBuffer[idx]);
        escaped.replace("\\", "\\\\");
        escaped.replace("\"", "\\\"");
        escaped.replace("\n", "\\n");
        escaped.replace("\r", "\\r");
        json += "\"" + escaped + "\"";
      }
      json += "],\"count\":" + String(total) + "}";
    } else {
      // 返回从 fromCount 到当前的新数据
      int newCount = webLogCount - fromCount;
      if (newCount > 50) newCount = 50; // 每次最多增量50
      startIdx = (webLogHead - newCount + WEB_LOG_MAX) % WEB_LOG_MAX;
      for (int i = 0; i < newCount; i++) {
        if (i > 0) json += ",";
        int idx = (startIdx + i) % WEB_LOG_MAX;
        String escaped = String(webLogBuffer[idx]);
        escaped.replace("\\", "\\\\");
        escaped.replace("\"", "\\\"");
        escaped.replace("\n", "\\n");
        escaped.replace("\r", "\\r");
        json += "\"" + escaped + "\"";
      }
      json += "],\"count\":" + String(webLogCount) + "}";
    }
    server.send(200, "application/json; charset=utf-8", json);
  });

  // ----- 日志开关 API -----
  server.on("/set-weblog", []() {
    if (server.hasArg("enable")) {
      webLoggingEnabled = server.arg("enable").toInt() == 1;
      webLogln("📟 网页日志记录已%s", webLoggingEnabled ? "开启" : "关闭");
      server.send(200, "text/plain; charset=utf-8", webLoggingEnabled ? "✅ 日志已开启" : "⏹️ 日志已关闭");
    } else {
      server.send(400, "text/plain", "Missing enable param");
    }
  });

  // ----- 清空日志 API -----
  server.on("/clear-weblog", []() {
    webLogHead = 0;
    webLogCount = 0;
    server.send(200, "text/plain; charset=utf-8", "✅ 日志已清空");
  });

  // ----- 历史数据 API -----
  server.on("/history", []() {
  int page = server.arg("page").toInt();
  int count = server.arg("count").toInt();
  bool applyFilter = server.arg("filter").toInt() == 1;
    server.send(200, "application/json; charset=utf-8", buildHistoryJson(page, count, applyFilter));
  });


  server.on("/data", []() {
    int count = 600;
    if (server.hasArg("count")) { count = server.arg("count").toInt(); if (count < 1) count = 1; if (count > bufferCapacity) count = bufferCapacity; }
    bool applyFilter = server.hasArg("filter") ? (server.arg("filter").toInt() == 1) : true;
    server.send(200, "application/json; charset=utf-8", buildLatestJSON(count, applyFilter));
  });

  server.on("/settime", HTTP_POST, []() {
    if (server.hasArg("ts")) {
      time_t t = server.arg("ts").toInt();
      struct timeval tv = { t, 0 };
      settimeofday(&tv, nullptr);
      if (rtcOK) rtc.adjust(DateTime(t));
      bool wasSynced = timeSynced;
      timeSynced = true;
      if (!wasSynced) { hasSyncedOnce = true; cleanupUntimestampedData(); }
      // ⚠️ 手动校时后**重算采样网格**：nextSampleTime 是开机时按当时时间锚定的，
      //    时间被手动跳变后若不重算，会直到旧锚点到达才重新对齐（最多偏一个采样点）。
      nextSampleTime = 0;                    // 置 0 → loop 下轮按新时间重新锚到 :00/:30
      webLogln("🕐 已手动校时，采样网格将重新对齐");
      server.send(200, "text/plain; charset=utf-8", "🕐 已同步");
    } else { server.send(400, "text/plain; charset=utf-8", "Missing ts"); }
  });

  // /download[?f=/xxx.csv] —— 默认下主日志；带 f= 可下其它白名单文件（标定 CSV 等）
  server.on("/download", []() {
    String fn = server.arg("f");
    if (fn.length() == 0) {
      // ↓ 默认：吐内存环形缓冲（永远是最新数据；Flash/SD 文件仍可用 ?f= 指定）
      //   背景：/log.csv 曾静默停止增长（见 warnAppendFail），下载到手的是旧数据
      String csv = "序号,时间,温度(°C),湿度(%RH),气压(hPa),模式\n";
      long base = (long)recordCount - (long)bufferSize + 1;
      for (uint32_t i = 0; i < bufferSize; i++) {
        int idx = (int)((bufferHead - bufferSize + i + bufferCapacity * 2) % bufferCapacity);
        time_t t = buffer[idx].time;
        char ts[24];
        if (timeSynced && t > 1000000000) {
          struct tm tmv; localtime_r(&t, &tmv);
          strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", &tmv);
        } else {
          int hh = (int)(t / 3600), mm = (int)((t % 3600) / 60), ss = (int)(t % 60);
          snprintf(ts, sizeof(ts), "%02d:%02d:%02d", hh, mm, ss);
        }
        csv += String(base + (long)i) + "," + String(ts) + "," +
               fmtVal(buffer[idx].temp) + "," + fmtVal(buffer[idx].humidity) + "," +
               fmtVal(buffer[idx].pressure) + ",-\n";   // 模式列：缓冲里没存（老约定用 -）
      }
      server.sendHeader("Content-Disposition", "attachment; filename=weather_station.csv");
      server.sendHeader("Content-Length", String(csv.length() + 3));
      server.setContentLength(csv.length() + 3);
      char bomin[3] = { (char)0xEF, (char)0xBB, (char)0xBF };
      server.send(200, "text/csv; charset=utf-8", "");
      server.sendContent(bomin, 3);
      server.sendContent(csv);
      return;
    }
    // 白名单：只放行这两个文件，避开路径穿越
    if (fn != "/log.csv" && fn != "/battcal.csv") {
      server.send(400, "text/plain; charset=utf-8", "❌ 只允许 f=/log.csv 或 f=/battcal.csv");
      return;
    }
    if (!LittleFS.exists(fn)) { server.send(200, "text/plain; charset=utf-8", "暂无数据"); return; }
    File file = LittleFS.open(fn, "r");
    if (!file) { server.send(200, "text/plain; charset=utf-8", "打开失败"); return; }
    size_t fileSize = file.size();
    if (fileSize == 0) { server.send(200, "text/plain; charset=utf-8", "文件为空"); file.close(); return; }
    String dlName = (fn == "/log.csv") ? "weather_station.csv" : "battcal.csv";
    server.sendHeader("Content-Disposition", "attachment; filename=" + dlName);
    server.sendHeader("Content-Length", String(fileSize + 3));
    server.setContentLength(fileSize + 3);
    char bom[3] = { (char)0xEF, (char)0xBB, (char)0xBF };
    server.send(200, "text/csv; charset=utf-8", "");
    server.sendContent(bom, 3);
    uint8_t buf[512]; int bytesRead;
    while ((bytesRead = file.read(buf, sizeof(buf))) > 0) server.sendContent((const char*)buf, bytesRead);
    file.close();
  });

  server.on("/clear", []() {
    LittleFS.remove("/log.csv"); recordCount = 0; bufferSize = 0; bufferHead = 0; dayMarkIndex = -1;
    File file = LittleFS.open("/log.csv", "w");
    if (file) { file.println("序号,时间,温度(°C),湿度(%RH),气压(hPa),模式"); file.close(); }
    server.send(200, "text/html; charset=utf-8", "<html><meta charset='UTF-8'><body><h3>✅ 已清空</h3><a href='/'>返回</a></body></html>");
  });

  server.on("/archive", []() {
    // ⚠️ 2026-10-02：**只启动、不等待**——实际处理交给 loop() 的 archivePump 分片推进，
    //    否则一次跑完整个大循环会阻塞主循环（实测 90~109 秒）→ 丢采样点。
    if (g_arch.active)      server.send(200, "text/plain; charset=utf-8", "📦 归档已在进行中");
    else if (archiveStart(false)) server.send(200, "text/plain; charset=utf-8", "✅ 已启动自动归档（分片后台进行）");
    else                    server.send(200, "text/plain; charset=utf-8", "⚠️ 无法启动归档（SD/时间/已归档）");
  });

  server.on("/sdlist", []() {
    if (!ensureSD()) {                 // 运行中插的卡也能立刻列出来
      server.send(200, "text/html; charset=utf-8",
                  "<html><meta charset='UTF-8'><body><h3>❌ SD卡未就绪</h3><a href='/'>返回</a></body></html>");
      return;
    }
    File root = SD.open("/");
    String html = "<html><meta charset='UTF-8'><style>body{font-family:sans-serif;margin:20px}"
                  "a{color:#28a745}td{padding:5px 15px}"
                  ".manual{color:#e67e22}</style>"
                  "<body><h2>📂 SD卡文件</h2>"
                  "<p style='color:#888;font-size:13px'>🟢 无后缀 = 自动归档</p>"
                  "<table><tr><th>文件名</th><th>大小</th><th>操作</th></tr>";
    bool hasFile = false;
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      String name = String(entry.name());
      while (name.startsWith("/")) name = name.substring(1);
      if (name.endsWith(".csv")) {
        hasFile = true;
        String style = ""; String tag = "";
        if (name.indexOf("_manual") >= 0) { style = " class='manual'"; tag = " 🟠"; }
        String encodedName = urlEncode(name);
        html += "<tr><td" + style + ">" + name + tag + "</td><td>" + String(entry.size()) + " B</td>"
                "<td><a href='/sdread?f=" + encodedName + "'>📥 下载</a></td></tr>";
      }
      entry.close();
    }
    root.close();
    if (!hasFile) {
      html += "<tr><td colspan='3' style='color:#888;text-align:center'>（暂无归档文件）</td></tr>";
    }
    html += "</table><br><a href='/'>← 返回</a></body></html>";
    server.send(200, "text/html; charset=utf-8", html);
  });

  server.on("/sdread", []() {
    String f = server.arg("f");
    if (f.length() == 0) {
      server.send(200, "text/html; charset=utf-8",
                  "<html><meta charset='UTF-8'><body><h3>❌ 缺少文件名</h3><a href='/sdlist'>返回</a></body></html>");
      return;
    }
    if (!ensureSD()) {                 // 运行中插的卡也能立刻下载
      server.send(200, "text/html; charset=utf-8",
                  "<html><meta charset='UTF-8'><body><h3>❌ SD卡未就绪</h3><a href='/sdlist'>返回</a></body></html>");
      return;
    }
    String path1 = f;
    if (!path1.startsWith("/")) path1 = "/" + path1;
    String path2 = f;
    if (path2.startsWith("/")) path2 = path2.substring(1);
    String fullPath = "";
    if (SD.exists(path1.c_str())) fullPath = path1;
    else if (SD.exists(path2.c_str())) fullPath = path2;
    else {
      server.send(200, "text/html; charset=utf-8",
                  "<html><meta charset='UTF-8'><body><h3>❌ 文件不存在</h3><p>路径: " + f + "</p><a href='/sdlist'>返回</a></body></html>");
      return;
    }
    File file = SD.open(fullPath.c_str(), "r");
    if (!file) {
      server.send(200, "text/html; charset=utf-8",
                  "<html><meta charset='UTF-8'><body><h3>❌ 打开失败</h3><a href='/sdlist'>返回</a></body></html>");
      return;
    }
    size_t fileSize = file.size();
    if (fileSize == 0) {
      server.send(200, "text/html; charset=utf-8",
                  "<html><meta charset='UTF-8'><body><h3>⚠️ 文件为空</h3><a href='/sdlist'>返回</a></body></html>");
      file.close(); return;
    }
    String downloadName = f;
    int slashIdx = downloadName.lastIndexOf('/');
    if (slashIdx >= 0) downloadName = downloadName.substring(slashIdx + 1);
    server.sendHeader("Content-Disposition", "attachment; filename=" + downloadName);
    server.sendHeader("Content-Length", String(fileSize + 3));
    server.setContentLength(fileSize + 3);
    char bom[3] = { (char)0xEF, (char)0xBB, (char)0xBF };
    server.send(200, "text/csv; charset=utf-8", "");
    server.sendContent(bom, 3);
    uint8_t buf[512]; int bytesRead;
    while ((bytesRead = file.read(buf, sizeof(buf))) > 0) server.sendContent((const char*)buf, bytesRead);
    file.close();
  });

  // ----- 日平均曲线（30天窗口，3分钟一格） -----
  server.on("/daily-avg", []() {
    server.send(200, "application/json; charset=utf-8", buildDailyAvgJson());
  });


  // ----- 今日距平曲线（1分钟一格） -----
  server.on("/anomaly", []() {
    server.send(200, "application/json; charset=utf-8", buildAnomalyJson());
  });


  // ----- 今日极值 + 当前距平 + 24h趋势 -----
  server.on("/today-stats", []() {
    server.send(200, "application/json; charset=utf-8", buildTodayStatsJson());
  });


// 设备状态 JSON（HTTP 与 BLE 共用）。overBLE=true 时 caps 只留只读能力 → 前端自动整卡隐藏
server.on("/status", []() {
  noteWebActivity();            // 会话续期 + 「网页已打开」确认（立刻收横幅）
  server.send(200, "application/json; charset=utf-8", buildStatusJson(false));
});
  // 无线联调/调试端点：/wireless?force=auto|wifi|ble → 强制某种无线（现场调试 + 上机验证用）
  //   ⚠ 这里【只置标志】，真正的切换交给 loop 里的 updateWireless() ——
  //     在 HTTP 回调里直接关 WiFi 栈 = 在自家回调里拆自家网卡，实测会把设备搞卡死
  server.on("/wireless", []() {
    if (server.hasArg("force")) {
      String f = server.arg("force");
      int old = wlForce;
      unsigned long sec = server.hasArg("sec") ? server.arg("sec").toInt() : 180;   // 可调窗口，默认 3 分钟
      if (sec < 20) sec = 20;
      if (sec > 7200) sec = 7200;                                                   // 上限 2 小时
      wlForce = (f == "wifi" || f == "1") ? 1 : ((f == "ble" || f == "2") ? 2 : 0);
#if !ENABLE_BLE
      if (wlForce == 2) { wlForce = 0; webLogln("🔧 蓝牙已停用（ENABLE_BLE=0）→ 忽略 force=ble，按自动处理"); }
#endif
      wlForceUntil = wlForce ? (millis() + sec * 1000UL) : 0;                      // 到期自愈
      if (wlForce != old) webLogln("🔧 无线强制模式 = %s（%lu 秒后自动恢复）",
                                   wlForce == 1 ? "WiFi" : (wlForce == 2 ? "蓝牙" : "自动"), sec);
    }
    server.send(200, "application/json; charset=utf-8",
      String("{\"cur\":") + curWireless + ",\"force\":" + wlForce +
      ",\"secLeft\":" + (wlForceUntil ? (long)((wlForceUntil - millis()) / 1000) : 0) +
      ",\"ble\":" + (bleOn ? "true" : "false") +
      ",\"bleLink\":" + (bleLink ? "true" : "false") + "}");
    // 注意：不在这里调用 updateWireless()（见函数头注释），loop 会在 2 秒内完成切换
  });
  // ========== 📶 WiFi 网页配置 ==========
  // ========== 📶 WiFi 网页配置（多组：自动选信号最强）==========
  server.on("/wifi", []() {
    String cur = (WiFi.status() == WL_CONNECTED) ? WiFi.SSID() : String(wifiCfg.ssid);
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi 设置 - 微型气象站</title>)rawliteral" + String(WEB_ICONS) + R"rawliteral(
<style>
body{font-family:-apple-system,'Segoe UI',sans-serif;background:#f5f7fa;color:#222;padding:12px;max-width:720px;margin:0 auto}
h2{font-size:18px}
.card{background:#fff;border-radius:10px;padding:14px;margin:10px 0;box-shadow:0 1px 4px rgba(0,0,0,.08)}
.btn{display:inline-block;padding:9px 14px;margin:4px 4px 4px 0;border-radius:8px;border:0;color:#fff;background:#2e7d32;font-size:14px;text-decoration:none;cursor:pointer}
.btn-danger{background:#e74c3c}
.btn-gray{background:#607d8b}
.btn-sm{padding:6px 10px;font-size:13px}
table{width:100%;border-collapse:collapse;font-size:14px}
td,th{padding:7px 6px;border-bottom:1px solid #eee;text-align:left}
.rssi{font-family:monospace}
.muted{color:#888;font-size:13px}
input[type=text],input[type=password]{width:100%;padding:8px;border:1px solid #ccc;border-radius:6px;margin:4px 0;font-size:14px;box-sizing:border-box}
label{display:block;font-size:14px;margin:8px 0}
</style>
</head>
<body>
<div class="card">
<h2>📶 WiFi 设置 <span class="muted">已保存 )rawliteral";
    html += String(wifiCount) + " / " + String(WIFI_MAX_SAVED) + " 组，自动选信号最强</span></h2>";
    html += "<p class='muted'>当前：";
    if (WiFi.status() == WL_CONNECTED) {
      html += "<b>" + WiFi.SSID() + "</b>（" + String(WiFi.RSSI()) + " dBm） IP " + WiFi.localIP().toString();
    } else if (apEnabled) {
      html += "AP 模式（未连上路由器）—— 可连热点 " + String(AP_SSID) + " 后访问 " + String(AP_IP_STR);
    } else {
      html += "未连接";
    }
    html += "</p>";
    auto urlEnc = [](const String& in) {
      String o;
      for (size_t i = 0; i < in.length(); i++) {
        char c = in[i];
        if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.' || c == '~') o += c;
        else { char b[5]; snprintf(b, sizeof(b), "%%%02X", (unsigned char)c); o += b; }
      }
      return o;
    };
    html += "<table><tr><th>已保存的 SSID</th><th>地址</th><th></th></tr>";
    if (wifiCount == 0) html += "<tr><td colspan='3' class='muted'>（还没保存任何 WiFi）</td></tr>";
    for (int i = 0; i < wifiCount; i++) {
      String ss = String(wifiList[i].ssid);
      html += "<tr><td>" + ss + (ss == cur ? " <span class='muted'>← 当前</span>" : "") + "</td><td>"
            + (wifiList[i].useIP ? (String("静态 .") + String(wifiList[i].ipLast ? wifiList[i].ipLast : 200)) : String("自动(DHCP)")) + "</td><td>"
            + "<a class='btn btn-danger btn-sm' href='/wifi-del?ssid=" + urlEnc(ss) + "' onclick=\"return confirm('删除该 WiFi？')\">删除</a></td></tr>";
    }
    html += "</table>";
    if (wifiCount > 0) html += "<p><a class='btn btn-gray' href='/wifi-reconnect'>🔁 重新选择并连接</a></p>";
    html += "</div>";

    html += R"rawliteral(
<div class="card">
<h2>➕ 添加 / 修改网络</h2>
<form action="/save-wifi" method="POST">
  <label>WiFi 名称 (SSID)<input type="text" name="ssid" id="f_ssid" required></label>
  <label>密码<input type="password" name="password" id="f_pass" placeholder="留空 = 不改动已保存的密码"></label>
  <label>地址方式
    <select name="useIP" onchange="document.getElementById('ipFields').style.display=(this.value=='1')?'block':'none'">
      <option value="0">自动 (DHCP)</option>
      <option value="1")rawliteral" + String(wifiCfg.useIP ? " selected" : "") + R"rawliteral(>静态 IP（只填末段）</option>
    </select>
  </label>
  <label id="ipFields" style="display:)rawliteral" + String(wifiCfg.useIP ? "block" : "none") + R"rawliteral(">IP 末段（2~254；留空=200，被占会自动 +1）
    <input type="number" name="ipLast" min="2" max="254" value=")rawliteral" + String(wifiCfg.ipLast ? wifiCfg.ipLast : 200) + R"rawliteral(">
    <span class="muted">网关 / 掩码 / DNS 由设备连上后自动学习并保存</span>
  </label>
  <button type="submit" class="btn">💾 保存</button>
</form>
<p class="muted">同名即覆盖（<b>密码留空 = 不改动已保存的密码</b>）。<b>保存后不重启</b>：没连上时立刻自动重连；已连着别的网络时保持不动（要切换点上面「🔁 重新选择并连接」）。<br>
<b>静态 IP</b>：只填末段（默认 200）。连接时会先 DHCP 连一次学习网关/掩码/DNS，再 ping 查重（被占自动 +1，至多 5 次），都不行则退回 DHCP 并在屏幕上标出实际地址。</p>
</div>

<div class="card">
<h2>📡 附近的网络 <span class="muted">（点「选用」填入上面的表单）</span></h2>
<div id="scanlist" class="muted">扫描中…</div>
<button class="btn btn-gray" onclick="doScan()">🔄 重新扫描</button>
</div>

<div class="card">
<button class="btn btn-danger" onclick="clearConfig()">🗑️ 清除全部 WiFi 配置（重启后仅 AP）</button>
</div>
<script>
function doScan(){
  var box=document.getElementById('scanlist');
  box.textContent='扫描中…（约 3 秒）';
  fetch('/wifi-scan').then(function(r){return r.json();}).then(function(list){
    if(!list.length){box.textContent='没扫到任何网络';return;}
    var h='<table><tr><th>SSID</th><th>信号</th><th></th></tr>';
    list.forEach(function(n){
      var bars=n.rssi>-60?'▂▄▆█':(n.rssi>-75?'▂▄▆_':'▂___');
      h+='<tr><td>'+(n.saved?'✅ ':'')+n.ssid+'</td><td class="rssi">'+bars+' '+n.rssi+' dBm</td>'
       + '<td><button class="btn btn-sm" onclick="useNet(\''+n.ssid.replace(/'/g,"\\'")+'\')">选用</button></td></tr>';
    });
    box.innerHTML=h+'</table>';
  }).catch(function(){box.textContent='扫描失败';});
}
function useNet(s){document.getElementById('f_ssid').value=s;document.getElementById('f_pass').focus();}
function clearConfig(){
  if(confirm('清除全部 WiFi 配置？设备将重启为 AP 模式')){
    fetch('/clear-wifi').then(function(r){return r.text();}).then(function(m){alert(m);location.reload();});
  }
}
doScan();
</script>
</body></html>)rawliteral";
    server.send(200, "text/html; charset=utf-8", html);
  });

  // 保存/覆盖一组（不重启；同名覆盖）
  server.on("/save-wifi", HTTP_POST, []() {
    if (!server.hasArg("ssid") || server.arg("ssid").length() == 0) {
      server.send(400, "text/html; charset=utf-8",
                  "<meta charset='UTF-8'><h3>❌ SSID 不能为空</h3><a href='/wifi'>返回</a>");
      return;
    }
    String ssid = server.arg("ssid");
    // 表单里 useIP 是下拉，总是会提交；工具裸调时可以完全不传 = 不改动地址方式
    int useIPMode = server.hasArg("useIP") ? (server.arg("useIP") == "1" ? 1 : 0) : -1;
    int ipLast = server.hasArg("ipLast") ? server.arg("ipLast").toInt() : 0;
    if (ipLast <= 0 && server.hasArg("ip")) {            // 兼容旧参数（整段 IP）→ 取末段
      IPAddress t; if (t.fromString(server.arg("ip"))) ipLast = t[3];
    }
    int idx = wifiUpsert(ssid.c_str(),
                         server.hasArg("password") ? server.arg("password").c_str() : "",
                         useIPMode, ipLast);
    webLogln("📶 WiFi 已保存: %s（共 %d 组）", ssid.c_str(), wifiCount);
    if (idx < 0) {
      server.send(200, "text/html; charset=utf-8",
                  "<meta charset='UTF-8'><h3>⚠️ 已达上限 " + String(WIFI_MAX_SAVED) + " 组，请先删一组</h3><a href='/wifi'>返回</a>");
      return;
    }
    bool wasConnected = (WiFi.status() == WL_CONNECTED);
    server.send(200, "text/html; charset=utf-8",
                "<meta charset='UTF-8'><h3>✅ 已保存：" + ssid + "（共 " + String(wifiCount) + " 组）</h3>" +
                String(wasConnected
                  ? "<p>当前仍连在原来的网络；要立即切到信号最强的那组，点 <a href='/wifi-reconnect'>🔁 重新选择并连接</a>。<a href='/wifi'>返回</a></p>"
                  : "<p>正在尝试连接…约 10 秒后看设备屏幕横幅。<a href='/wifi'>返回</a></p>"));
    // 存完就试着重连（装进外壳后够不到 EN 键，不能让它干等 60 秒或等人按按钮）
    if (!wasConnected) {
      delay(300);
      if (wifiCount > 1) wifiPickBest(false);
      // 不在 HTTP 回调里直接折腾网卡（容易卡死）：交给 loop 的「连网会话」去做（含静态策略）
      beginWifiSession(String(wifiCfg.ssid));
      webLogln("🔁 保存后自动重连（连网会话）: %s", wifiCfg.ssid);
    }
  });

  // 删除一组（删掉正在用的那组 → 自动重挑重连）
  server.on("/wifi-del", []() {
    String ssid = server.arg("ssid");
    bool wasCur = (ssid == String(wifiCfg.ssid));
    bool ok = wifiRemove(ssid.c_str());
    webLogln("🗑️ WiFi 删除: %s %s（剩 %d 组）", ssid.c_str(), ok ? "OK" : "未找到", wifiCount);
    String h = "<meta charset='UTF-8'><h3>" + String(ok ? "✅ 已删除" : "⚠️ 未找到") + "：" + ssid + "</h3>"
               "<p>剩余 " + String(wifiCount) + " 组。<a href='/wifi'>返回</a></p>";
    if (ok && wasCur) h += "<script>setTimeout(function(){location.replace('/wifi-reconnect');},800);</script>";
    server.send(200, "text/html; charset=utf-8", h);
  });

  // 重新扫描并连接最强的一组（不重启）
  server.on("/wifi-reconnect", []() {
    server.send(200, "text/html; charset=utf-8",
      "<meta charset='UTF-8'><h3>🔁 正在重新选择信号最强的一组并连接…</h3>"
      "<p>约 10 秒后看设备屏幕上的横幅。若切到了别的网络，本机也要连到同一个 WiFi 才能再访问设备。<a href='/'>返回主页</a></p>");
    delay(300);
    if (wifiCount > 1) wifiPickBest(false);
    beginWifiSession(String(wifiCfg.ssid));      // 交给 loop（含静态 IP 策略：学习 → 查重 → 静态）
    webLogln("🔁 手动重连（连网会话）: %s", wifiCfg.ssid);
  });

  // 扫描附近 WiFi（设置页选用；实现共用 buildWifiScanJson）
  server.on("/wifi-scan", []() {
    server.send(200, "application/json; charset=utf-8", buildWifiScanJson());
  });

  // 已保存 WiFi 列表 → JSON（配网卡共用；与蓝牙同一份实现，不回显密码）
  server.on("/wifi-list", []() {
    server.send(200, "application/json; charset=utf-8", buildWifiListJson());
  });
  server.on("/diag", []() {   // 只读诊断：最近 6 条关键日志
    server.send(200, "application/json; charset=utf-8", buildDiagJson());
  });
  // 配网调试日志（写 Flash）：插电连上 WiFi 后，浏览器直接打开 http://<设备IP>/cfgdbg
  //   ?clear=1 → 清除并停止续写
  //   ?on=秒数 → **手动开启**调试日志（默认 3600s，上限 24h）；用于"跑一晚"这类需要长期留证的场景
  //     ⚠️ 默认那个 10 分钟窗口只在"配网动作触发"时才起，跑一晚根本覆盖不到 → 所以留这个手动口子。
  server.on("/cfgdbg", []() {
    if (server.hasArg("clear")) {
      LittleFS.remove(CFG_DBG_PATH); cfgDbgOn = false; cfgDbgBoot = true;
      server.send(200, "text/plain; charset=utf-8", "已清除配网调试日志（不会再续写）");
      return;
    }
    if (server.hasArg("on")) {
      long sec = server.arg("on").toInt();
      if (sec <= 0) sec = 3600;                     // 默认 1 小时
      if (sec > 24L * 3600L) sec = 24L * 3600L;     // 上限 24 小时
      cfgDbgOn   = true;
      cfgDbgBoot = true;                            // 标记"本开机已判断过"，避免被续写逻辑覆盖
      cfgDbgUntil = millis() + (unsigned long)sec * 1000UL;
      cfgDbgNote("\n===== 手动开启调试日志（/cfgdbg?on=）=====\n");
      char msg[96];
      snprintf(msg, sizeof(msg), "调试日志已开启 %ld 秒（到 %lu ms）；用 /cfgdbg 读取，?clear=1 停止",
               sec, (unsigned long)cfgDbgUntil);
      server.send(200, "text/plain; charset=utf-8", msg);
      return;
    }
    File f = LittleFS.open(CFG_DBG_PATH, "r");
    if (!f) { server.send(200, "text/plain; charset=utf-8", "(暂无配网调试日志：还没触发过配网 / IO9 长按)"); return; }
    String s = f.readString(); f.close();
    server.send(200, "text/plain; charset=utf-8", s);
  });

  server.on("/clear-wifi", []() {
    LittleFS.remove("/wifi.json");
    wifiCount = 0;
    wifiSyncCurrent(-1);
    server.send(200, "text/plain; charset=utf-8", "✅ WiFi配置已清除，设备即将重启");
    delay(1000);
    ESP.restart();
  });

  // 网页重启（装壳后够不到 PCB 上的 EN 键时的兼底）
  server.on("/reboot", []() {
    server.send(200, "text/html; charset=utf-8",
      "<meta charset='UTF-8'><h3>🔌 设备重启中…</h3><p>约 10 秒后回来（IP 不变）。<a href='/'>返回主页</a></p>");
    webLogln("🔌 收到网页重启指令");
    delay(600);
    ESP.restart();
  });

  // ========== 🔄 OTA 固件升级 ==========
  // ---- 屏幕自检：/ui-dump 把显存读回成 ASCII 图（逻辑坐标，1 像素 1 字符）----
  //   用法：/ui-dump                         → 整屏 250x122
  //         /ui-dump?x0=0&y0=20&x1=249&y1=121 → 只看数据区（省流量）
  server.on("/ui-dump", []() {
    int x0 = server.hasArg("x0") ? server.arg("x0").toInt() : 0;
    int y0 = server.hasArg("y0") ? server.arg("y0").toInt() : 0;
    int x1 = server.hasArg("x1") ? server.arg("x1").toInt() : 249;
    int y1 = server.hasArg("y1") ? server.arg("y1").toInt() : 121;
    // 自检：?bt=1 时临时切到蓝牙态再 dump（蓝牙模式没 HTTP，平时 dump 不到 BT）
    if (server.hasArg("bt")) { uiDisplay.setWifi(UI_WIFI_BLE); uiDisplay.showTop(time(nullptr)); }
    x0 = constrain(x0, 0, 249); x1 = constrain(x1, 0, 249);
    y0 = constrain(y0, 0, 121); y1 = constrain(y1, 0, 121);
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/plain; charset=utf-8", "");
    server.sendContent("ui-dump 逻辑(" + String(x0) + "," + String(y0) + ")-(" + String(x1) + "," + String(y1) +
                       ")  # = 墨迹  . = 空白\n");
    for (int y = y0; y <= y1; y++) {
      char pad[8];
      snprintf(pad, sizeof(pad), "%3d|", y);
      String line = pad;
      line.reserve((size_t)(x1 - x0 + 8));
      for (int x = x0; x <= x1; x++) line += uiDisplay.readLogical(x, y) ? '#' : '.';
      line += '\n';
      server.sendContent(line);
    }
    server.sendContent("");
  });

  // ---- 屏幕提示（Toast）：按当前状态在屏幕上弹一条横幅 ----
  //   连上路由器 → √ WiFi OK + SSID + IP；AP 已开 → 广播 AP MODE + 热点名 + IP；都没有 → × WiFi FAIL
  //   可选参数：?d=毫秒（5000~120000）停留时长（默认 20 秒）；?k=ok|fail|ap 强制类型（自检用）
  server.on("/toast", []() {
    uint32_t dur = 20000;
    if (server.hasArg("d")) {
      long d = server.arg("d").toInt();
      if (d >= 5000 && d <= 120000) dur = (uint32_t)d;   // 5 ~ 120 秒
    }
    uint8_t kind = 0xFF;                                 // 0xFF = 按状态自动判断
    if (server.hasArg("k")) {
      String k = server.arg("k"); k.toLowerCase();
      kind = (k == "ok") ? UI_TOAST_OK : (k == "fail") ? UI_TOAST_FAIL : (k == "ap") ? UI_TOAST_AP : 0xFF;
    }
    bool sta = (WiFi.status() == WL_CONNECTED);
    char l2[48], l3[32];
    if (sta) {
      buildSsidLine(l2, sizeof(l2), WiFi.SSID().c_str());
      snprintf(l3, sizeof(l3), "%s", WiFi.localIP().toString().c_str());
    } else if (apEnabled) {
      buildSsidLine(l2, sizeof(l2), AP_SSID);
      snprintf(l3, sizeof(l3), "%s", AP_IP_STR);
    } else {
      buildSsidLine(l2, sizeof(l2), strlen(wifiCfg.ssid) ? wifiCfg.ssid : "-");
      snprintf(l3, sizeof(l3), "no link");
    }
    if (kind == 0xFF) kind = sta ? UI_TOAST_OK : (apEnabled ? UI_TOAST_AP : UI_TOAST_FAIL);
    // 失败横幅一律不给 IP：否则会让人以为已经连上去访问（第二三行改报“连的是谁 + 没连上”）
    if (kind == UI_TOAST_FAIL) snprintf(l3, sizeof(l3), "not connected");
    const char* l1 = (kind == UI_TOAST_OK) ? wifiOkTitle() : (kind == UI_TOAST_AP) ? "AP MODE" : "WiFi FAIL";
    uiDisplay.showToast(kind, l1, l2, l3, dur);
    // 自检：?q=1 时再排一条热点横幅（模拟“开机连不上 → 10 秒后 AP MODE”的真实时序）
    if (server.arg("q") == "1") {
      char a2[48];
      buildSsidLine(a2, sizeof(a2), AP_SSID);
      uiDisplay.queueToast(UI_TOAST_AP, "AP MODE", a2, AP_IP_STR, dur);
    }
    String msg = String("✅ 屏幕显示：") + l1 + " / " + l2 + " / " + l3;
    webLog("🖥️ /toast(%lus, k=%s) → %s\n", (unsigned long)(dur / 1000),
           server.hasArg("k") ? server.arg("k").c_str() : "auto", msg.c_str());
    server.send(200, "text/plain; charset=utf-8", msg);
  });

  // ---- 设备模式（固定/移动）：GET 查询，带 ?m=0|1 则切换 ----
  server.on("/mode", []() {
    if (server.hasArg("m")) {
      saveDeviceMode((uint8_t)server.arg("m").toInt());
      uiSyncStatus();
      uiDisplay.showTop(time(nullptr));            // 立刻反映到屏幕
      webLog("🔧 设备模式 -> %s(%d)\n", deviceMode == 0 ? "固定FIX" : "移动MOV", deviceMode);
      chargeForceSlow = false;                    // V2.1.1-a：换模式 = 新场景 → 清掉临时慢充
      applyChargeStrategy(readPowerState());      // 立刻应用对应充电策略（取新鲜电源状态）
    }
    String j = String("{\"mode\":") + deviceMode + ",\"name\":\""
             + (deviceMode == 0 ? "FIX" : "MOV") + "\"}";
    server.send(200, "application/json", j);
  });

  // ---- 设备名（推送用）配置与校验（v2.2）----
  //   ⛔ 硬规则（用户明确要求，勿放宽）：
  //     ① 必须联网才能改名（AP 模式 / 未连 WiFi 一律拒绝）—— "没网改个der的网名"
  //     ② 必须向服务器确认名字没被占用；**服务器不可达也拒绝**
  //        → 因为重名两台设备的数据会混进同一 device_id，而服务器按 ts 幂等去重，
  //          从数据上根本看不出异常，等发现时已经脏了。宁可不给改。
  server.on("/devname", HTTP_GET, []() {
    char dflt[33];
    pushDeriveDevId(dflt, sizeof(dflt));
    // dev = 身份（固定不变）；name = 显示名（可改、可空）；default = 身份默认值
    String j = String("{\"dev\":\"") + pushDevId + "\",\"name\":\"" +
               pushJsonEsc(pushDevName) + "\",\"default\":\"" + dflt + "\"";
    j += ",\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
    j += ",\"srv\":\"" + String(pushSrvHost) + ":" + String(pushSrvPort) + "\"";
    j += ",\"lastSyncedTs\":" + String(lastSyncedTs) +
                           ",\"cursorOk\":" + (pushCursorValid ? "true" : "false") +   // 环已推干净（游标可信）；false=中间有空档
                           // ⚠️ 字段名绝不能留在行尾注释里 —— 之前这里写成
                           //    `..., // 说明 ",\"enabled\":" +`，注释把字段名吃掉了，
                           //    结果拼出 `"cursorOk":falsetrue}`，**是非法 JSON**（解析器直接报错）。
                           ",\"enabled\":" +
         String(pushEnabled ? "true" : "false") + "}";
    server.send(200, "application/json", j);
  });

  server.on("/devname", HTTP_POST, []() {
    if (!server.hasArg("dev")) {
      server.send(400, "application/json", "{\"ok\":false,\"err\":\"缺少 dev 参数\"}");
      return;
    }
    String want = server.arg("dev");
    want.trim();

    // ① 未连 WiFi（含 AP 模式）→ 直接拒绝（没网就无法校验，也就无法保证不重名）
    if (WiFi.status() != WL_CONNECTED) {
      webLogln("🚫 改名被拒：未连 WiFi（无法向服务器校验重名）");
      server.send(200, "application/json",
                  "{\"ok\":false,\"err\":\"未连上路由器，无法校验重名，禁止改名\"}");
      return;
    }
    // ② 没变 → 直接成功（省一次往返）
    if (want == String(pushDevName)) {
      server.send(200, "application/json",
                  String("{\"ok\":true,\"name\":\"") + pushJsonEsc(pushDevName) +
                  "\",\"dev\":\"" + pushDevId + "\",\"unchanged\":true}");
      return;
    }

    // ③~⑤ 交给 devRenameTo：本地校验 → HMAC 签名 → 服务器验签 →
    //        **成功才落 NVS**。失败一律拒绝（不落盘）——
    //        这就实现了"连不上服务器就拒绝并提醒"。
    //        重名**不再拒绝**（只影响观感），服务器会在响应里带回 duplicate_of。
    if (!devRenameTo(want.c_str())) {
      server.send(200, "application/json",
                  "{\"ok\":false,\"err\":\"改名失败（原因见设备「日志」页）\"}");
      return;
    }
    server.send(200, "application/json",
                String("{\"ok\":true,\"name\":\"") + pushJsonEsc(pushDevName) +
                "\",\"dev\":\"" + pushDevId + "\"}");
  });


  // ---- 推送：状态查询 + 手动触发 + 配置（v2.2 第二阶段，2026-10-05）----
  //   手动触发是联调的关键：不用等 15 分钟，也不受"服务器不可达就拒绝"那套限制
  //   （这里失败只影响这一次推送，不会写坏任何配置）。
  server.on("/push", HTTP_GET, []() {
    if (!pushConfigLoaded) loadPushConfig();
    String j = String("{\"enabled\":") + (pushEnabled ? "true" : "false");
    j += ",\"dev\":\"" + String(pushDevId) + "\"";
    j += ",\"name\":\"" + pushJsonEsc(pushDevName) + "\"";
    j += ",\"srv\":\"" + String(pushSrvHost) + ":" + String(pushSrvPort) + "\"";
    j += ",\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
    j += ",\"timeSynced\":" + String(timeSynced ? "true" : "false");
    j += ",\"tokenInjected\":" + String(PHT_API_TOKEN[0] ? "true" : "false");
    j += ",\"bufN\":" + String(bufferSize);
    j += ",\"lastSyncedTs\":" + String(lastSyncedTs);
    j += ",\"pending\":" + String(pushCountNewer(lastSyncedTs));
    j += ",\"lastUploadTs\":" + String(pushLastUploadTs);
    j += ",\"lastAccepted\":" + String(pushLastAccepted);
    j += ",\"lastOk\":" + String(pushLastOk ? "true" : "false");
    j += ",\"state\":" + String(pushRunState);
    j += ",\"failStreak\":" + String(pushFailStreak);
    j += ",\"passes\":" + String(pushPassCount);
    j += ",\"nextInSec\":" + String(pushNextAtMs ? (int32_t)(pushNextAtMs - millis()) / 1000 : -1);
    j += ",\"needBackfill\":" + String(pushNeedBackfill ? "true" : "false");
    j += ",\"backfillBefore\":" + String(pushBackfillBeforeTs) + "}";
    server.send(200, "application/json", j);
  });

  // 手动触发一轮同步。?full=1 → 从内存里最老的点重推（游标回退，用于验收/补推）
  server.on("/push", HTTP_POST, []() {
    if (!pushEnabled) {
      server.send(200, "application/json", "{\"ok\":false,\"err\":\"推送已关闭\"}");
      return;
    }
    if (WiFi.status() != WL_CONNECTED) {
      server.send(200, "application/json", "{\"ok\":false,\"err\":\"未连 WiFi，无法推送\"}");
      return;
    }
    if (!timeSynced) {
      server.send(200, "application/json", "{\"ok\":false,\"err\":\"时间未同步（时间戳是服务器主键，拒绝推脏数据）\"}");
      return;
    }
    bool full = (server.arg("full") == "1");
    uint32_t from = lastSyncedTs;
    if (full) {
      uint32_t oldest = 0;
      for (int i = 0; i < bufferSize; i++) {
        const DataPoint& d = buffer[(bufferHead - bufferSize + i + bufferCapacity) % bufferCapacity];
        if (hasValidDate(d.time) && (oldest == 0 || d.time < oldest)) oldest = d.time;
      }
      if (oldest) lastSyncedTs = oldest - 1;
    }
    pushTickForce = true;                       // 交给任务去干（HTTP 不进主循环）
    pushNextAtMs = 0;
    webLog("🔧 手动推送触发（%s，游标 %lu）\n", full ? "全量重推" : "增量", (unsigned long)lastSyncedTs);
    String j = String("{\"ok\":true,\"queued\":true,\"full\":") + (full ? "true" : "false");
    j += ",\"fromTs\":" + String(from) + ",\"pending\":" + String(pushCountNewer(lastSyncedTs)) + "}";
    server.send(200, "application/json", j);
  });

  // 推送配置：开关 / 服务器地址（设备名走 /devname，那里有重名校验）
  server.on("/pushcfg", HTTP_POST, []() {
    if (server.hasArg("on")) pushEnabled = (server.arg("on") == "1");
    if (server.hasArg("host")) {
      String h = server.arg("host"); h.trim();
      if (h.length() > 0 && h.length() < sizeof(pushSrvHost)) strlcpy(pushSrvHost, h.c_str(), sizeof(pushSrvHost));
    }
    if (server.hasArg("port")) {
      long pt = server.arg("port").toInt();
      if (pt > 0 && pt <= 65535) pushSrvPort = (uint16_t)pt;
    }
    savePushConfig();
    webLog("🔧 推送配置已保存：%s | %s:%u\n", pushEnabled ? "开启" : "关闭", pushSrvHost, (unsigned)pushSrvPort);
    server.send(200, "application/json",
                String("{\"ok\":true,\"enabled\":") + (pushEnabled ? "true" : "false") +
                ",\"srv\":\"" + String(pushSrvHost) + ":" + String(pushSrvPort) + "\"}");
  });

  // ==================== 一机一密：网页端点 ====================
  //   ⚠️ 这些端点**不做鉴权**，与设备上其它配置端点一致（局域网工具）。
  //      风险边界：能访问到设备网页的人本来就能改服务器地址等配置。
  //      真要收紧，应该在设备网页整体加一层认证 —— 那是另一件事。
  server.on("/devkey", HTTP_GET, []() {
    String j = String("{\"bound\":") + (devKeyBound() ? "true" : "false");
    j += ",\"keyId\":\"" + String(devKeyId) + "\"";
    j += ",\"setAt\":" + String((unsigned long)devKeySetAt);
    j += ",\"selftest\":" + String(devKeySelfTestOk ? "true" : "false");
    j += ",\"dev\":\"" + String(pushDevId) + "\"";
    j += ",\"name\":\"" + pushJsonEsc(pushDevName) + "\"";
    j += ",\"srv\":\"" + String(pushSrvHost) + ":" + String(pushSrvPort) + "\"";
    j += ",\"wifi\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
    j += ",\"clockOk\":" + String(time(nullptr) > 1700000000 ? "true" : "false");
    j += "}";
    server.send(200, "application/json", j);
  });

  // 生成（或重新生成）密钥。⚠️ 重新生成后**必须重新注册**，否则推送会 401
  server.on("/devkey", HTTP_POST, []() {
    String act = server.arg("act");
    if (act == "gen") {
      devKeyGenerate();
      server.send(200, "application/json",
                  String("{\"ok\":true,\"keyId\":\"") + devKeyId +
                  "\",\"note\":\"请立即点【注册到服务器】，否则推送会被拒\"}");
      return;
    }
    if (act == "clear") {
      devKeyClear();
      server.send(200, "application/json", "{\"ok\":true,\"bound\":false}");
      return;
    }
    if (act == "enroll") {
      String pw = server.arg("pw");
      pw.trim();
      bool ok = devKeyEnroll(pw.c_str());
      server.send(200, "application/json",
                  String("{\"ok\":") + (ok ? "true" : "false") + "}");
      return;
    }
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"未知 act\"}");
  });

  // ---- V2.1.1-b 电量校准（放电曲线标定）----
  server.on("/battcal", []() { server.send(200, "text/html; charset=utf-8", calPageHtml()); });
  server.on("/keycfg", []() { server.send(200, "text/html; charset=utf-8", keyPageHtml()); });
  server.on("/battcal/status", []() { server.send(200, "application/json", calStatusJson()); });
  server.on("/battcal/start", []() {
    String msg; bool ok = calStart(server.arg("dry") == "1", msg);
    msg.replace("\"", "'");
    server.send(200, "application/json", String("{\"ok\":") + (ok ? "true" : "false") + ",\"msg\":\"" + msg + "\"}");
  });
  server.on("/battcal/stop", []() { calAbort("网页中止"); server.send(200, "application/json", "{\"ok\":true}"); });
  server.on("/battcal/build", []() {
    bool ok = calBuildTable(true);
    server.send(200, "application/json", String("{\"ok\":") + (ok ? "true" : "false") + "}");
  });
  server.on("/battcal/report", []() { server.send(200, "application/json", calReportJson()); });
  server.on("/battcal/clear", []() {
    LittleFS.remove(CAL_JSON_PATH);
    calTableOk = false; calTableDry = false; calTableN = 0;
    webLogln("🧹 已清除标定曲线（回到线性估算）");
    server.send(200, "application/json", "{\"ok\":true}");
  });

  server.on("/update", HTTP_GET, []() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OTA 固件升级</title>)rawliteral" + String(WEB_ICONS) + R"rawliteral(
<style>
body{font-family:-apple-system,'Segoe UI',sans-serif;margin:20px;background:#f5f5f5;text-align:center}
.card{background:#fff;padding:22px;border-radius:10px;max-width:520px;margin:0 auto;box-shadow:0 1px 6px rgba(0,0,0,.08)}
h2{color:#333;font-size:18px}
input[type=file]{margin:14px 0;font-size:14px}
.btn{padding:12px 25px;border:none;border-radius:6px;color:#fff;cursor:pointer;font-size:16px;background:#28a745}
.btn:disabled{background:#9e9e9e;cursor:not-allowed}
.bar{height:10px;background:#e0e0e0;border-radius:5px;overflow:hidden;margin:14px 0 8px}
.bar>i{display:block;height:100%;width:0;background:#28a745;transition:width .2s}
#progress{color:#555;font-size:14px;margin:6px 0;min-height:20px}
.muted{color:#888;font-size:12px}
</style>
</head>
<body>
<div class="card">
<h2>🔄 OTA 固件升级</h2>
<p class="muted">选择编译好的 .bin 固件上传（约 1.2 MB，需十几秒）</p>
<input type="file" id="file" accept=".bin">
<br>
<button class="btn" id="btn">⬆️ 上传并更新</button>
<div class="bar"><i id="bar"></i></div>
<div id="progress">等待选择文件…</div>
<p class="muted">上传期间请勿断电。升级成功后设备会自动重启并回到主页。</p>
</div>
<script>
var btn=document.getElementById('btn'),pg=document.getElementById('progress'),bar=document.getElementById('bar'),fileEl=document.getElementById('file');
btn.addEventListener('click',function(){
  var f=fileEl.files[0];
  if(!f){ pg.textContent='⚠️ 请先选择 .bin 固件文件'; return; }
  var fd=new FormData(); fd.append('firmware',f,f.name);
  var xhr=new XMLHttpRequest();
  xhr.open('POST','/update');
  btn.disabled=true; btn.textContent='⏳ 更新中…';
  xhr.upload.onprogress=function(e){
    if(e.lengthComputable){ var p=Math.round(e.loaded/e.total*100); bar.style.width=p+'%';
      pg.textContent='⬆️ 上传中 '+p+'%（'+Math.round(e.loaded/1024)+' / '+Math.round(e.total/1024)+' KB）'; }
  };
  xhr.onload=function(){
    if(xhr.status===200){ bar.style.width='100%';
      pg.innerHTML='✅ 更新成功！设备正在重启…<br><span class="muted">正在等待设备回来…</span>';
      waitBack();
    } else {
      pg.innerHTML='❌ 更新失败（HTTP '+xhr.status+'）<br><span class="muted">'+(xhr.responseText||'').replace(/<[^>]+>/g,' ').slice(0,200)+'</span>';
      btn.disabled=false; btn.textContent='⬆️ 重试';
    }
  };
  xhr.onerror=function(){
    pg.innerHTML='⚠️ 连接中断（设备可能已重启）<br><span class="muted">正在等待设备回来…</span>';
    waitBack();
  };
  pg.textContent='⏳ 正在上传…';
  xhr.send(fd);
});
function waitBack(){
  var n=0;
  setTimeout(function(){
    var t=setInterval(function(){
      n++;
      fetch('/status',{cache:'no-store'}).then(function(r){
        if(r.ok){ clearInterval(t); pg.innerHTML='✅ 设备已恢复，正在返回主页…'; setTimeout(function(){location.href='/';},1200); }
      }).catch(function(){
        pg.innerHTML='⏳ 设备重启中…（已等 '+n+' 秒）';
        if(n>90){ clearInterval(t); pg.innerHTML='⚠️ 90 秒仍未回来 —— 请手动刷新页面或检查设备电源。'; }
      });
    },1500);
  },8000);
}
</script>
</body></html>)rawliteral";
    server.send(200, "text/html; charset=utf-8", html);
  });

  // ⚠ 两个回调的分工（实测踩过的坑）：**只有 final handler 能 send()**。
  //    在 upload 回调里也 send() 会导致「双重响应」——原始回包出现两套 Content-Type/Content-Length
  //    （甚至把上一次请求残留的头发进来），浏览器拿到畸形响应直接白屏/报错 = 「没反馈」。
  static bool    otaUploadOk = false;   // 两个 lambda 共享（静态存储期，无需捕获）
  static uint8_t otaFail     = 0;        // 0=无 1=begin失败 2=写入失败/中断 3=end失败
  server.on("/update", HTTP_POST, []() {
    // ---- final handler：只在这里发响应 ----
    if (otaUploadOk && otaFail == 0) {
      server.send(200, "text/html; charset=utf-8",
        "<html><meta charset='UTF-8'><body><h3>✅ 更新成功！设备重启中…</h3>"
        "<p>约 15 秒后设备回来；<a href='/'>点此返回主页</a></p></body></html>");
      delay(800);
      ESP.restart();
    } else {
      const char* why = (otaFail == 1) ? "Update.begin 失败（分区方案不支持 OTA？）"
                      : (otaFail == 2) ? "写入失败或上传被中断（空间不足/网络中断？）"
                      : (otaFail == 3) ? "Update.end 校验失败（固件无效？）" : "未知错误";
      server.send(500, "text/html; charset=utf-8",
        String("<html><meta charset='UTF-8'><body><h3>❌ 更新失败</h3><p>") + why +
        "</p><p><a href='/update'>重试</a> · <a href='/weblog'>查看设备日志</a></p></body></html>");
    }
  }, []() {
    // ---- upload 回调：只碰 Update.*，绝不 send() ----
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      otaUploadOk = false; otaFail = 0;
      webLogln("🔄 OTA 开始: %s", upload.filename.c_str());
      // 老内核(3.3.10)没有 upload.contentLength / totalSize 字段，用最大容量代替
      // Update.begin(UINT32_MAX) 兼容老内核 & 新内核，自动适配分区大小
      if (Update.isRunning()) Update.abort();
      if (!Update.begin(UINT32_MAX)) {
        otaFail = 1;
        Update.printError(Serial);
        webLogln("❌ Update.begin 失败，请检查分区方案是否支持OTA");
      } else {
        otaUploadOk = true;
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (otaUploadOk && Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        otaUploadOk = false; otaFail = 2;
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (otaUploadOk) {
        if (Update.end(true)) {
          webLogln("✅ OTA 成功！(%u 字节) 等待重启", upload.totalSize);
        } else {
          otaUploadOk = false; otaFail = 3;
          Update.printError(Serial);
          webLogln("❌ Update.end 失败");
        }
      } else if (otaFail == 0) {
        otaFail = 2;
      }
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
      otaUploadOk = false; if (!otaFail) otaFail = 2;
      webLogln("❌ OTA 上传被中断");
    }
  });

  server.begin();
}

// ========== 读取传感器 ==========
void logData(float temp, float humidity, float pressure);   // ← 加这行
void readAndLog() {
  float temp = 0, humidity = 0, pressure = 0;
  bool gotTempHum = false, gotPressure = false;

  // ===== 电源状态判定（低电保护门控） =====
  PowerState ps = readPowerState();
  g_power = ps;                       // 缓存给屏幕/其它模块
  applyChargeStrategy(ps);            // 充电策略随模式/电压联动（固定 80% 停充等）
  // 记录门控（设计文档三档；仅电池 + INA230 在线时判定）：
  //   插电(PGOOD低)   → 绝对充足，总是记录
  //   INA230 离线      → 电压未知，低电门控整体作废（照常记录、不深睡），仅告警
  //   电池 + 电压已知  → <3.6V 存SD / <3.55V 停采 / <3.5V 深睡（V2.1.1-b 上移前为 3.3/3.25/3.2）
  bool allowRecord = true;
  if (ps.powered) {
    allowRecord = true;
  } else if (!ps.inaOK) {
    static bool warnedNoIna = false;
    if (!warnedNoIna) {
      webLogln("⚠️ INA230 离线：电池电压未知，低电保护与电量显示暂失效（继续记录）");
      warnedNoIna = true;
    }
  } else {
    static bool lowVoltArchived = false;
    if (ps.battVolt < battArchiveV) {          // battArchiveV（默认 3.6V，有表则按本机表 15%）：先把已积累数据存档至 SD（只做一次）
      if (!lowVoltArchived) {
        webLogln("🔋 电池电压 %.2fV < %.2fV，存档数据至 SD", ps.battVolt, battArchiveV);
        archiveStart(false);                      // 只启动：低电时更不能阻塞（分片后台推进）
        lowVoltArchived = true;
      }
    } else {
      lowVoltArchived = false;                    // 电压回升 → 下次低电可再存档
    }
    if (ps.battVolt < battStopV) {         // battStopV（默认 3.55V，有表则按本机表 10%）：停止采集
      allowRecord = false;
      webLogln("🔋 电池电压 %.2fV < %.2fV，停止采集（电压回升后自动恢复）", ps.battVolt, battStopV);
    }
    // ---- 深睡保护（2026-10-03 加固）----
    //   用户实测事故：插着 USB 拔掉电池 → 设备黑屏关机，插电也不再启动。
    //   两个原因：
    //     ① 拔电池瞬间 INA230 读到 0V（芯片由 3.3V 轨供电，不会"离线"）→ 0 < 3.5V 直接深睡
    //     ② 深睡唤醒源写死老脚 IO2（PGOOD 已迁到 IO10）→ 睡下就永远醒不过来
    if (ps.battVolt < battSleepV) {
      // 守卫 A：有外接电源（PGOOD 或 CHG 任一）→ 绝不深睡，系统继续跑
      bool extPower = ps.powered || isExternallyPowered() || ps.charging
                      || (ps.battVolt > PR1_OFF_V);   // 电压本身偏高 → 与"低电"矛盾 → 按有电处理
      // 守卫 B：读数不可信（电池被拔/未接 → 0V 附近；或超出合理区间）
      bool vSane = ps.inaOK && ps.battVolt > LOWBAT_SANE_MIN_V && ps.battVolt <= PR1_VALID_MAX;
      if (extPower || !vSane) {
        static unsigned long lastLowSkip = 0;
        if (millis() - lastLowSkip > 300000UL) {      // 5 分钟限流
          lastLowSkip = millis();
          webLog("🛡️ 低电但**不深睡**：电压 %.3fV，%s（插电优先 / 电池读数不可信）\n",
                 ps.battVolt, extPower ? "检测到外部电源" : "读数不可信（电池可能未接）");
        }
      } else {
        // 守卫 C：**连续 3 次**采样都低才深睡（防 I2C 垃圾值 / 拔插瞬变）
        static int lowStreak = 0;
        lowStreak++;
        if (lowStreak < 3) {
          webLog("🔋 电压 %.3fV < %.2fV（第 %d/3 次确认，暂不深睡）\n",
                 ps.battVolt, battSleepV, lowStreak);
        } else {
          webLogln("🔋 电压 %.3fV < %.2fV（连续 %d 次确认）→ 触发深睡保护", ps.battVolt, battSleepV, lowStreak);
          enterDeepSleepIfNeeded();
          return;
        }
      }
    }

  }

  if (shtOK) {
    float t = sht.readTemperature();
    float h = sht.readHumidity();
    if (t > -40 && t < 85 && h >= 0 && h <= 100) { temp = t; humidity = h; gotTempHum = true; }
    else { webLogln("⚠️ SHT30 读数异常"); shtOK = false; }
  }
  if (bmpOK) {
    if (bmp.performReading()) {
      float p = bmp.readPressure();
      if (p > 300 && p < 1100) { pressure = p; gotPressure = true; }
      else { webLogln("⚠️ BMP580 气压异常"); bmpOK = false; }
    }
  }
  // 传感器失联 → 置 NaN（下游据此跳过，不再造虚拟数据；落盘为 "N/A"）
  if (!gotTempHum) { temp = NAN; humidity = NAN; }
  if (!gotPressure) { pressure = NAN; }

  // ---- 整点变率环（三要素，整点更新一次，不依赖日平均引擎）----
  {
    time_t now = time(nullptr);
    if (timeSynced && now >= 1000000000) {
      struct tm tm; localtime_r(&now, &tm);
      static int lastRingHour = -1;
      if (tm.tm_min == 0 && tm.tm_hour != lastRingHour) {
        HourSlot& hs = hourRing[tm.tm_hour];      // 槽内即 24h 前同整点
        trendT24 = (hs.valid && !isnan(temp)     && !isnan(hs.t)) ? (temp     - hs.t) : NAN;
        float ahNow = absHumidity(temp, humidity);
        float ahOld = absHumidity(hs.t, hs.h);
        trendH24 = (hs.valid && !isnan(ahNow) && !isnan(ahOld)) ? (ahNow - ahOld) : NAN;
        trendP24 = (hs.valid && !isnan(pressure) && !isnan(hs.p)) ? (pressure - hs.p) : NAN;
        hs.t = temp; hs.h = humidity; hs.p = pressure; hs.valid = true;
        lastRingHour = tm.tm_hour;
        webLogln("🕐 整点变率 @%02d:00  ΔT=%.1f  ΔAH=%.2f  ΔP=%.1f",
                 tm.tm_hour, trendT24, trendH24, trendP24);
      }
    }
  }

  // ---- V2.1.1 Bug② 修复：实时采样与「开机回灌」共用同一个喂数函数 ----
  //   本函数只负责「今天」的维护（跨天重置）；喂数逻辑见 feedTodaySample()
  if (avgSlots && todaySlots) {
    time_t now = time(nullptr);
    if (timeSynced && now >= 1000000000) {
      struct tm tm; localtime_r(&now, &tm);
      char d[11];
      snprintf(d, sizeof(d), "%04d-%02d-%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
      if (strlen(todayExt.date) == 0) {
        snprintf(todayExt.date, sizeof(todayExt.date), "%s", d);
      } else if (strcmp(d, todayExt.date) != 0) {
        rolloverDay();                       // 跨天：存档昨天 → 更新滑动窗口 → 重置今日
      }
      feedTodaySample((uint32_t)now, temp, humidity, pressure);
    }
  }
  // 仅当电源允许时才写入/记录（低电压时跳过，保护数据）
  if (allowRecord) {
    logData(temp, humidity, pressure);
  }
  // ---- 刷新屏幕 ----
  //   插电：每次采样(:00/:30)都刷，实时
  //   电池：只在整分钟(:00)那次刷，省电（见 设计文档 · 电源管理）
  time_t tNow = time(nullptr);
  bool atMinute = true;
  if (timeSynced && tNow >= 1000000000) {
    struct tm tmNow; localtime_r(&tNow, &tmNow);
    atMinute = (tmNow.tm_sec <= 1);
  }
  if (ps.powered || atMinute) {
    uiSyncStatus();
    uiDisplay.setTrend(trendT24, trendH24, trendP24);
    uiDisplay.showAll(temp, humidity, pressure, tNow);
  }
}

// ===== V2.1.1 Bug②：把「一条采样 → 今日统计」抽成函数 =====
//   背景（Bug②）：`todayExt`/`todaySlots` 原先只在**实时采样路径**里更新，开机时
//   `loadBufferFromFlash()` 只把 /log.csv 尾部 600 条灌进 RAM 环形缓冲**供图表渲染**，
//   没有回灌今日统计 → OTA / 掉电 / 长按复位之后，**当天已过去的极值与距平槽全丢**。
//   修法：实时采样与开机回灌共用本函数（对实时路径行为不变）。
//   ⚠ 本函数**不做跨天重置**（那是调用方的职责）：回灌历史样本时若触发 rolloverDay()，
//     会把刚重置的存储又清空 → 白忙。日期字段未就绪时直接返回，避免写进错误的一天。
void feedTodaySample(uint32_t t, float tC, float h, float p) {
  if (!avgSlots || !todaySlots) return;
  if (strlen(todayExt.date) != 10) return;          // 日期未初始化 → 等 readAndLog 设好再来
  struct tm tm; time_t tt = (time_t)t; localtime_r(&tt, &tm);
  // 距平基线【只吃固定数据】：移动样本不进日平均/距平（设计文档：距平仅固定可用，
  //   其平均值也仅用固定模式数据计算）；三要素齐全才计入（缺测样本整体跳过）
  if (deviceMode == 0 && !isnan(tC) && !isnan(h) && !isnan(p)) {
    int slot = tm.tm_hour * 120 + tm.tm_min * 2 + (tm.tm_sec >= 30 ? 1 : 0);
    if (slot >= 0 && slot < AVG_SLOTS) {
      todaySlots[slot].sumT += tC; todaySlots[slot].sumH += h; todaySlots[slot].sumP += p;
      todaySlots[slot].cnt++;
    }
  }
  // 极值：不经日平均门控（与实时路径一致）；updateExtreme 按数值比较、不看时间戳，回灌旧时间戳安全
  if (!isnan(tC)) { updateExtreme(todayExt.tempMax, tC, t); updateExtreme(todayExt.tempMin, tC, t, true); }
  if (!isnan(h))  { updateExtreme(todayExt.humMax,  h,  t); updateExtreme(todayExt.humMin,  h,  t, true); }
  if (!isnan(p))  { updateExtreme(todayExt.presMax, p,  t); updateExtreme(todayExt.presMin, p,  t, true); }
}

bool isNewDay(uint32_t t1, uint32_t t2) {
  if (!hasValidDate(t1) || !hasValidDate(t2)) return false;
  struct tm tm1, tm2;
  time_t tt1 = t1, tt2 = t2;
  localtime_r(&tt1, &tm1); localtime_r(&tt2, &tm2);
  return (tm1.tm_yday != tm2.tm_yday);
}

void logData(float temp, float humidity, float pressure) {
  if (!buffer) return;
  time_t t = time(nullptr);
  if (!timeSynced || t < 1000000000) t = millis() / 1000;
  if (bufferSize > 0) {
    int lastIdx = (bufferHead - 1 + bufferCapacity) % bufferCapacity;
    if (isNewDay(buffer[lastIdx].time, t)) { dayMarkIndex = bufferHead; }
  }
  struct tm* tm_info = localtime(&t);
  char timeStr[20];
  if (timeSynced && t > 1000000000) strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", tm_info);
  else { int h = t / 3600, m = (t % 3600) / 60, s = t % 60; sprintf(timeStr, "%02d:%02d:%02d", h, m, s); }
  String line = String(++recordCount) + "," + String(timeStr) + "," +
                fmtVal(temp) + "," + fmtVal(humidity) + "," + fmtVal(pressure) + "," +
                String(deviceMode);   // 第6列：采集时刻的设备模式（0=固定 1=移动）
  safeAppendFile("/log.csv", line);
  buffer[bufferHead].time = t;
  buffer[bufferHead].temp = temp;
  buffer[bufferHead].humidity = humidity;
  buffer[bufferHead].pressure = pressure;
  bufferHead = (bufferHead + 1) % bufferCapacity;
  if (bufferSize < bufferCapacity) bufferSize++;
}

// ========== AP 管理（省电：连上WiFi就关AP） ==========
void startAP() {
  if (apEnabled) return;
  IPAddress local_IP(192, 168, 5, 1), gateway(192, 168, 5, 1), subnet(255, 255, 255, 0);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  apEnabled = true;
  apClosedMs = 0;
  webLog("📡 AP 已开启: %s | %s\n", AP_SSID, AP_IP_STR);
  // 屏幕提示：热点已开（广播图标 + 热点名 + IP，20 秒）。若已有横幅在显示（比如刚弹的 × WiFi FAIL）则不抢占
  if (!uiDisplay.toastActive()) {
    char l2[48];
    buildSsidLine(l2, sizeof(l2), AP_SSID);
    uiDisplay.showToast(UI_TOAST_AP, "AP MODE", l2, AP_IP_STR, 20000);
  }
}

void stopAP() {
  if (!apEnabled) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  apEnabled = false;
  apClosedMs = 0;
  apSessionUntil = 0;
  wifiRecoverUntil = 0;            // 【兜底】AP 关了 → 救援模式随之结束（下次判定恢复正常场景）
  webLogln("📴 AP 已关闭（STA 已连接，省电模式）");
}

// AP 配网热点：带超时的按需开启（移动+电池按 IO9 → STA 失败转 AP）
//   到期自动关，防止热点一直开着把电耗光；期间有人访问网页会自动续期（见 noteWebActivity）
void startAPWithTimeout() {
  startAP();
  apSessionUntil = millis() + AP_SESSION_MS;
  webLogln("⏱️ AP 配网热点已开：%lu 分钟无人访问将自动关闭（省电）", AP_SESSION_MS / 60000UL);
}

// ===== 【兜底】AP 救援：长按 IO9 ≥5s 松手 → 强制开热点 + 断 WiFi =====
//   针对的失控场景（实测踩到）：设备连上了保存的网络，但**静态 IP + 网关/掩码/DNS 学自另一个网段**
//     → 它自己上不了网、PC 也够不着 → 网页/OTA 全部失联，只能拔电（见 开发日志 §十二）
//   做法：① 清掉该网络学到的 gw/mask/dns（关键！否则重连还会用旧网段）
//         ② 记住它的 SSID，方便网页里一键重连
//         ③ 断 STA + 开 AP 热点 + 置救援窗口（窗口内 wantWireless 恒 WiFi，且不自动重连）
void wifiRecover(const char* why) {
  String lastSsid = String(wifiCfg.ssid);
  webLogln("🆘 进入 AP 救援模式（%s）：断开 WiFi、强开热点 %s @ %s",
           why, AP_SSID, AP_IP_STR);
  // ① 清掉学来的网络参数（静态 IP 保留「末段」，地址方式保留用户选择；只清坏掉的 gw/mask/dns）
  bool hadGw = (uint32_t)wifiCfg.gw != 0;
  wifiCfg.gw = IPAddress(0, 0, 0, 0);
  wifiCfg.mask = IPAddress(0, 0, 0, 0);
  wifiCfg.dns = IPAddress(0, 0, 0, 0);
  saveWiFiConfig();
  if (hadGw) webLogln("🧹 已清除学到的网关/掩码/DNS（上次学的网段不对，重连时会重新学）");
  diag("AP 救援：断 WiFi 开热点，已清网络参数");
  cfgDbgBegin("IO9 长按 5s：AP 救援模式");
  // ② 断 STA
  WiFi.disconnect(true);
  // ③ 开热点（带超时）+ 置救援窗口：窗口内禁止自动重连，保证 AP 稳定可达
  startAP();
  apSessionUntil   = millis() + AP_SESSION_MS;
  wifiRecoverUntil = millis() + AP_SESSION_MS;
  wifiBtnArmed     = false;                 // 别让同一次操作又触发「连最强」
  pendingSSID      = "";                    // 丢弃未执行的连接任务
  uiDisplay.showToast(UI_TOAST_AP, "AP RESCUE", "已断 WiFi · 连热点配网", AP_IP_STR, 20000);
  webLogln("✅ AP 救援就绪：连热点 『%s』（密码 %s）→ 打开 http://%s/wifi 重新配网（%lu 分钟后自动关）",
           AP_SSID, AP_PASSWORD, AP_IP_STR, AP_SESSION_MS / 60000UL);
  if (lastSsid.length()) webLogln("💡 上次连的是 『%s』：网页里重新保存一次即可刷新它的网关/掩码", lastSsid.c_str());
}

// ========== setup ==========
// 网页被访问：给会话续期；并作为「用户真的进来了」的确认（立刻收横幅，别傻等）
void noteWebActivity() {
  if (wifiSessionUntil) wifiSessionUntil = millis() + WIFI_SESSION_MS;
  if (apEnabled && apSessionUntil) apSessionUntil = millis() + AP_SESSION_MS;  // AP 配网期间有人访问 → 续期
  if (!webSeenSinceConnect) {
    webSeenSinceConnect = true;
    toastDismissReq = true;      // ⚠ 不能在这里直接重绘：本函数跑在 HTTP 请求处理栈里，
                                 //   叠加整屏重绘会爆栈（实测 PANIC，屏幕只剩中文标签）→ 交给主循环
    webLogln("🌐 网页已被访问 → 横幅立即收起（配网确认成功）");
  }
}

// ---- 连网会话服务：按钮动作 + 待连接任务 + 状态机推进（放 loop 末尾，蓝牙回包已发完）----
void serviceWifiSession() {
  // 收到网页请求后要收的横幅：在这里（主循环栈）重绘，别在 HTTP 处理函数里做
  if (toastDismissReq) { toastDismissReq = false; uiDisplay.dismissToast(); }
  // 开机第一次：上次若是异常复位，屏幕直接横幅报出来
  //   （移动+电池走蓝牙时既没 HTTP 也接不了串口，屏幕是唯一能当场看到的地方）
  static bool bootNotice = false;
  if (!bootNotice) {
    bootNotice = true;
    esp_reset_reason_t rr = esp_reset_reason();
    const char* shortR = nullptr;
    switch (rr) {
      case ESP_RST_PANIC:     shortR = "PANIC 崩溃"; break;
      case ESP_RST_INT_WDT:   shortR = "INT_WDT";  break;
      case ESP_RST_TASK_WDT:  shortR = "TASK_WDT"; break;
      case ESP_RST_WDT:       shortR = "WDT";      break;
      case ESP_RST_BROWNOUT:  shortR = "BROWNOUT 电压跌落"; break;
      default: break;
    }
    if (shortR) {
      uiDisplay.showToast(UI_TOAST_FAIL, "上次异常复位", shortR, "详见 /cfgdbg", 15000);
      webLogln("⚠️ 上次是异常复位：%s", resetReasonStr());
    }
  }
  // A) 按钮长按：STA 优先（连已保存最强）→ 失败/无组转 AP 配网
  if (wifiBtnArmed) {
    wifiBtnArmed = false;
    cfgDbgBegin("IO9 长按：STA 优先，失败转 AP");
    if (wifiCount == 0) {
      webLogln("🔘 未保存任何网络 → 开 AP 配网热点");
      startAPWithTimeout();
      uiDisplay.showToast(UI_TOAST_AP, "AP 配网", "无已存网络", AP_IP_STR, 20000);
    } else {
      WiFi.mode(WIFI_STA);
      bool found = (wifiCount > 1) ? wifiPickBest(false) : true;
      if (!found) {
        webLogln("🔘 已保存的 %d 组都不在附近 → 转 AP 配网", wifiCount);
        startAPWithTimeout();
        uiDisplay.showToast(UI_TOAST_AP, "AP 配网", "附近无已存网络", AP_IP_STR, 20000);
      } else beginWifiSession(String(wifiCfg.ssid));
    }
  }
  // B) 待连接任务（会阻塞几秒 → 放最后，且延后 400ms 让 BLE 回包先发出去）
  if (pendingSSID.length() && (int32_t)(millis() - pendingAt) >= 0) {
    String ssid = pendingSSID; pendingSSID = "";
    diag("开始执行连接任务（连网会话）");
    bool ok = wifiConnectSaved(true);            // V2.1：连不上 → 自动开 AP 配网兜底
    if (ok) {
      String ip = WiFi.localIP().toString();
      wifiSessionUntil = millis() + WIFI_SESSION_MS;   // 会话生效 → 状态机保持 WiFi
      webSeenSinceConnect = false;
      updateWireless();                                // 切 WiFi（沿用刚连上的连接）
      uiDisplay.showToast(UI_TOAST_OK, wifiOkTitle(), ssid.c_str(), ip.c_str(), 20000);
      webLogln("📶 [连网会话] 成功 → 已切 WiFi，屏幕显示 IP");
    } else {
      // 失败：wifiConnectSaved(true) 已开 AP 兜底（热点带超时），这里只负责提示
      apSessionUntil = millis() + AP_SESSION_MS;
      uiDisplay.showToast(UI_TOAST_FAIL, "WiFi FAIL", ssid.c_str(), "转 AP 配网", 10000);
      webLogln("📶 [连网会话] 失败 → 已开 AP 配网兜底（连热点后访问 %s 重新配网）", AP_IP_STR);
    }
  }

  // C) AP 配网热点超时：N 分钟无人访问 → 自动关（省电；移动+电池尤其重要）
  if (apEnabled && apSessionUntil && (int32_t)(millis() - apSessionUntil) >= 0) {
    bool wasRescue = (wifiRecoverUntil != 0);
    stopAP();                                 // 内部会把 apSessionUntil / wifiRecoverUntil 清零
    wifiSessionUntil = 0;
    webLogln("⏱️ AP 配网热点超时 → 已自动关闭（省电）");
    if (wasRescue) {                          // 【兜底】救援窗口结束 → 恢复正常联网（会按当前配置重连）
      webLogln("🆘 AP 救援窗口结束 → 恢复正常无线判定（若仍连不上，可再长按 IO9 ≥5s）");
      updateWireless();
    }
  }
  // C) 状态机推进（每 2 秒一次：会话到期 → wantWireless 变回蓝牙 → 自动回切）
  static unsigned long lastTick = 0;
  if (millis() - lastTick > 2000) { lastTick = millis(); updateWireless(); }
}

void setup() {
  // ============================================================
  // 启动流程（2026-10-03 重排）
  //   ① 保命与上电安全   ② 尽早点亮屏幕   ③ 采集能力就绪   ④ 联网与服务（可慢）
  //   要点：屏幕 ~1 秒亮；电压读数趁无明显负载（才准）；SD 挂载延后；delay 1000→500
  // ============================================================

  // ---------- ① 保命与上电安全 ----------
  chargeInit();                        // 必须最先：确定 CE/ISET/EN 电平，杜绝悬空
  Serial.begin(115200);
  delay(500);                          // 等 USB 串口枚举（原 1000ms，用户裁定缩短）

  webLogln("\n=== 微型气象站 v2.1 (BMP580 + SHT30 + DS3231 + SD / ESP32-S3) ===");
  webLog("🔌 充电控制初始态：%s | %s（IO18=%d IO17=%d）\n",
         chargeIsFast() ? "快充" : "慢充",
         chargeIsEnabled() ? "允许充电" : "停充",
         digitalRead(PIN_ISET), digitalRead(PIN_CE));

  setenv("TZ", "CST-8", 1);
  tzset();
  Wire.begin(47, 48);

  // INA230 初始化（含身份诊断；详见 实现细节.md D29）
  {
    uint16_t manu = 0, die = 0;
    // ⚠️ 顺序：先 begin()（内部做身份校验并设置 _wire），再 checkIdentity() 只为读出 ID 展示。
    bool began = ina230.begin(&Wire, INA230_ADDR, INA230_R_SHUNT, INA230_MAX_A);
    bool idOk  = began ? ina230.checkIdentity(&manu, &die) : false;
    webLog("🔎 INA230 身份: MANU=0x%04X DIE=0x%04X (%s)\n",
           manu, die, idOk ? "在位(全0为INA230正常值)" : "无应答(总线浮空)");
    if (began) {
      webLog("✅ INA230 @0x%02X (MANU=0x%04X DIE=0x%04X) CAL=%u %.3fmA/bit 有效分辨率%.3fmA 量程±%.2fA\n",
             INA230_ADDR, manu, die, ina230.calibration(),
             ina230.currentLsb() * 1000.0f, ina230.currentResolution_mA(), ina230.currentMax_A());
    } else {
      webLog("⚠️ INA230 @0x%02X 未就绪：电池电压/电量不可用，低电保护暂失效（屏幕与网页将告警）\n", INA230_ADDR);
    }
  }

  // ★ 电源状态：此刻只有 ESP32 在耗电，电池电压**最接近静置** → 冷/热分流读数才准
  PowerState psWake = readPowerState();
  g_power = psWake;
  serviceBatteryStartup();             // 冷/热启动分流（阈值 PWR_COLD_START_V）
  applyChargeStrategy(psWake);

  // ---- 唤醒后的电源复核：若唤醒（含深睡唤醒）后仍是低电异常，立即再睡（保命） ----
  //   （psWake / g_power / applyChargeStrategy 已在上面 ① 段完成）
  if (!psWake.powered && psWake.inaOK && psWake.battVolt < battSleepV) {
    webLogln("🔋 唤醒后电压仍过低 (%.2fV)，重新进入深睡保护", psWake.battVolt);
    enterDeepSleepIfNeeded();
    return;
  }
  if (psWake.powered) {
    webLogln("🔌 外部电源供电，正常工作");
  } else if (!psWake.inaOK) {
    webLogln("⚠️ INA230 离线：电池电压未知，本次会话低电保护与电量显示暂失效");
  } else if (psWake.battVolt < battStopV) {
    webLogln("🔋 电池电压 %.2fV < %.2fV，本次会话将跳过写入（直到插电或电压回升）", psWake.battVolt, battStopV);
  } else {
    webLogln("🔋 电池供电 %.2fV，正常工作", psWake.battVolt);
  }

  // ---------- ② 尽早点亮屏幕 ----------
  initRTC();
  initFS();
  loadDeviceMode();                    // 读固定/移动（充电策略与顶栏都要用）
  loadPushConfig();                    // 读推送配置（设备名/服务器/同步游标）

  // 一机一密：开机跑一次密码学自检（几十微秒）。
  //   ⚠️ 为什么必须自检：SHA/HMAC 写错的表现是**全线 401**，
  //      而那看起来像"服务器密钥不对/时钟偏差"，排查会绕很远。
  //      自检用 FIPS/公开测试向量，一次就把"算法对不对"与"配置对不对"分开。
  devKeySelfTestOk = pht_crypto_selftest();
  if (!devKeySelfTestOk) {
    webLogln("❌ 密码学自检失败！一机一密不可用（SHA/HMAC 实现有问题）");
  } else if (devKeyBound()) {
    webLogln("🔑 一机一密已启用（keyId=%s…）", devKeyId);
  }

  calBootCheck();                      // 载入标定曲线 → 低电三档
  uiDisplay.init();

  // RTC 有电就先设系统时间 → 屏幕立刻显示正确时间（不必等 WiFi/NTP）
  if (rtcOK && !rtcLostPower) {
    DateTime dt = rtc.now();
    time_t rt = dt.unixtime();
    if (rt >= 1000000000) {
      struct timeval tv = { rt, 0 };
      settimeofday(&tv, nullptr);
      timeSynced = true; hasSyncedOnce = true;
      char buf[30]; strftime(buf, 30, "%Y-%m-%d %H:%M:%S", localtime(&rt));
      webLog("✅ RTC 时间: %s\n", buf);
    }
  }

  uiSyncStatus();
  uiDisplay.showTop(time(nullptr));    // ← 屏幕在此亮起

  // ---------- ③ 采集能力就绪（传感器提前于联网）----------
  // ---- BMP580 初始化 ----
  if (!bmp.begin(0x47, &Wire)) {
    webLogln("⚠️ BMP580 未在 0x47 找到");
    if (!bmp.begin(0x46, &Wire)) {
      webLogln("❌ BMP580 初始化失败"); bmpOK = false;
    } else {
      webLogln("✅ BMP580 @ 0x46"); bmpOK = true;
      bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_8X);
      bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    }
  } else {
    webLogln("✅ BMP580 @ 0x47"); bmpOK = true;
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  }

  // ---- SHT30 初始化 ----
  if (!sht.begin(0x44)) {
    webLogln("⚠️ SHT30 未在 0x44 找到");
    if (!sht.begin(0x45)) { webLogln("❌ SHT30 初始化失败"); shtOK = false; }
    else { webLogln("✅ SHT30 @ 0x45"); shtOK = true; }
  } else { webLogln("✅ SHT30 @ 0x44"); shtOK = true; }
  initAvgEngine();

  // ---------- ④ 联网与服务（可慢；SD 挂载也延后到这里）----------
  loadWiFiConfig();

  bool needAP = true;
  // ★ 2026-10-03：**先扫，扫不到就不连**（原来扫不到仍硬连 → 白等 DHCP 超时最多 10s）
  bool inRange = false;
  if (wifiCount > 0) {
    inRange = wifiPickBest(true);        // requireInRange=true：扫不到返回 false
  }
  if (strlen(wifiCfg.ssid) > 0 && !inRange) {
    webLog("⚠️ '%s' 不在扫描范围内 → 跳过连接，直接开 AP（省掉一次无用的 DHCP 超时）\n", wifiCfg.ssid);
  }
  if (strlen(wifiCfg.ssid) > 0 && inRange) {
    webLog("📶 尝试连接 '%s'（已保存 %d 组，%s）...\n", wifiCfg.ssid, wifiCount, wifiCfg.useIP ? "静态" : "DHCP");
    bool ok = wifiConnectSaved(true);
    if (ok) {
      needAP = false;
      webLog("\n✅ IPv4: %s%s\n", WiFi.localIP().toString().c_str(), wifiStaticFallback ? "（静态失败，已退回 DHCP）" : "");
      // 屏幕提示：连上了（√ WiFi OK + SSID + IP，20 秒后自动还原）
      char tl2[48], tl3[32];
      buildSsidLine(tl2, sizeof(tl2), wifiCfg.ssid);
      snprintf(tl3, sizeof(tl3), "%s", WiFi.localIP().toString().c_str());
      uiDisplay.showToast(UI_TOAST_OK, wifiOkTitle(), tl2, tl3, 20000);
      // NTP：RTC 掉电时必须立刻拿到时间（阻塞等一次）；否则非阻塞，交给 loop
      if (rtcLostPower) { webLogln("🕐 RTC 掉电 → 阻塞等一次 NTP"); syncTimeFromNTPBlocking(10000UL); }
      else              { ntpStart(); }
    } else {
      webLogln("\n⚠️ WiFi连接失败，仅AP模式");
      // 屏幕提示：连不上——**只报"试连这个 WiFi 失败了"，不给 IP**（免得误以为连上了去访问）；
      //           10 秒后接着弹热点横幅，那里才给热点名 + 热点 IP
      char tl2[48], tl3[48];
      buildSsidLine(tl2, sizeof(tl2), wifiCfg.ssid);
      uiDisplay.showToast(UI_TOAST_FAIL, "WiFi FAIL", tl2, "not connected", 10000);
      buildSsidLine(tl3, sizeof(tl3), AP_SSID);
      uiDisplay.queueToast(UI_TOAST_AP, "AP MODE", tl3, AP_IP_STR, 20000);
      _fallbackToRtcTime();
    }
  } else {
    webLogln("\n⚠️ 无WiFi配置，稍后开启AP配置模式（http://192.168.5.1 设置WiFi）");
    _fallbackToRtcTime();
  }

  backfillTodayFromLog();

  if (needAP) startAP();
  else webLogln("✅ 已连接 WiFi，AP 未开启（省电模式）");

  initSD();                            // SD 挂载延后（用户：启动时爱挂不挂）
  archivePump();
  checkAndArchive();
  startServer();
  webLog("🌐 服务器已启动 | 内存 %d 条\n", bufferSize);

  pinMode(PIN_WIFI_BTN, INPUT_PULLUP);
  curWireless = WL_WIFI;
  updateWireless();

  serviceUsbEnum();                    // USB 枚举（SOF 帧号）：主机 / 充电器

  // v2.2 推送任务：独立 FreeRTOS 任务，HTTP 只在这里阻塞（主循环一秒都不等）
  //   优先级与 loop 相同（1）→ 靠 tick 时间片轮转；绑核心 0，避免跨核竞争 WiFi 栈
  xTaskCreatePinnedToCore(pushTask, "phtpush", PUSH_TASK_STACK, nullptr, PUSH_TASK_PRIO, &pushTaskHandle, 0);
}

// ========== loop ==========
// ===================================================
// ===== V2.1.1-b 电量校准：实现 =====
//   规格见 设计文档.md · V2.1.1 · 二、电量校准
// ===================================================

// 中止按键：长按 IO9 ≥1.5s（标定期 checkWifiButton() 不跑，这里独立判定）
bool calAbortPressed() {
  static bool last = HIGH;
  static unsigned long downAt = 0;
  bool now = digitalRead(PIN_WIFI_BTN);
  if (last == HIGH && now == LOW) downAt = millis();
  bool hit = (last == LOW && now == HIGH && (millis() - downAt) >= WIFI_BTN_DEBOUNCE_MS && (millis() - downAt) <= WIFI_BTN_SHORT_MS);   // v2.2：短按
  last = now;
  return hit;
}

// 屏幕横幅（复用 Toast：盖数据区、不动顶栏；标定期不需要自动还原）
void calScreen(const char* l1, const char* l2) {
  uiDisplay.showToast(UI_TOAST_OK, l1, l2, "", 3600000UL);
}

// 进度屏：进度 = 按 4.20V↔3.00V 线性估的剩余容量（仅示意）
void calScreenProgress() {
  float v = calLastVolt;
  int pct = 0;
  if (!isnan(v)) {
    float f = (v - CAL_FLOOR_V) / (4.20f - CAL_FLOOR_V);
    if (f < 0) f = 0; if (f > 1) f = 1;
    pct = (int)(f * 100.0f + 0.5f);
  }
  calLastPct = pct;
  char l1[24], l2[56];
  uint32_t el  = (calState == CAL_RUN) ? (millis() - calStartMs) : 0;
  uint32_t sec = el / 1000UL;
  snprintf(l1, sizeof(l1), "CAL %d%%", pct);
  snprintf(l2, sizeof(l2), "%luh%02lum %.2fV %.0fmA c%lu",
           (unsigned long)(sec / 3600), (unsigned long)((sec % 3600) / 60),
           isnan(v) ? 0.0f : v, isnan(calLastI) ? 0.0f : calLastI, (unsigned long)calCycles);
  calScreen(l1, l2);
}

// 射频负载：向网关 9999 端口连发 UDP（纯发送即可，不要求对端响应）
void calUdpBurst() {
  if (WiFi.status() != WL_CONNECTED) return;
  static uint8_t buf[512];
  IPAddress gw = WiFi.gatewayIP();
  for (int i = 0; i < 20; i++) {
    buf[0] = (uint8_t)i;
    calUdp.beginPacket(gw, 9999);
    calUdp.write(buf, sizeof(buf));
    calUdp.endPacket();
  }
}

// CPU 负载：loop 里空转（不新建任务，避开抢占/崩溃风险；WiFi 任务在另一核照跑）
void calCpuBurn(uint32_t ms) {
  uint32_t t0 = millis();
  while ((uint32_t)(millis() - t0) < ms) { }
}

// INA230 配置切换：标定期尽量用大平均窗口（理想 1024/512 × 2 × 8.244ms）
//   ⚠️ 实测踩坑（2026-09-15，**当时是 INA226**）：该片的 **AVG=6(512) / 7(1024) 写进去会被夹到 4(128)**
//      （期望 0x6FE7 回读 0x4FE7，只差 bit13）→ 故先探测芯片真正接受的 AVG，再用它 begin()
//   窗口 = AVG × 2 × 8.244ms；AVG=128 → ≈2.1s，已足够盖住静置尾段
bool calInaBegin(bool calMode) {
  static const Ina230Avg A[4]  = { INA230_AVG_1024, INA230_AVG_512, INA230_AVG_256, INA230_AVG_128 };
  static const uint16_t  AN[4] = { 1024, 512, 256, 128 };
  if (!calMode) {
    bool ok = ina230.begin(&Wire, INA230_ADDR, INA230_R_SHUNT, INA230_MAX_A);
    webLog("🔧 INA230 重配(常规)：%s CONFIG=0x%04X\n", ok ? "OK" : "失败", ina230.configWord());
    return ok;
  }
  int best = -1; uint16_t bad = 0, rbad = 0;
  for (int i = 0; i < 4; i++) {
    uint16_t want = INA230::buildConfig(A[i], CAL_CT, CAL_CT, true);
    if (!ina230.writeReg(INA230_REG_CONFIG, want)) { delay(10); continue; }
    delay(10);
    uint16_t rb = 0; ina230.readReg(INA230_REG_CONFIG, rb);
    if (rb == want) { best = i; break; }
    bad = want; rbad = rb;
  }
  if (best < 0) {
    webLog("⚠️ 标定：INA230 大窗口一个都不生效（如 0x%04X→0x%04X）→ 用默认 281ms\n", bad, rbad);
    ina230.begin(&Wire, INA230_ADDR, INA230_R_SHUNT, INA230_MAX_A);
    return false;
  }
  bool ok = ina230.begin(&Wire, INA230_ADDR, INA230_R_SHUNT, INA230_MAX_A, A[best], CAL_CT, CAL_CT);
  webLog("🔧 INA230 重配(标定)：%s AVG=%u 窗口≈%.2fs CONFIG=0x%04X\n",
         ok ? "OK" : "失败", AN[best], AN[best] * 2 * 0.008244f, ina230.configWord());
  if (!ok) ina230.begin(&Wire, INA230_ADDR, INA230_R_SHUNT, INA230_MAX_A);
  return ok;
}

// 只读 INA230（观察者效应：醒来第一件事就只剩这个动作）
void calReadIna(float& v, float& i) {
  Ina230Reading ir;
  if (ina230.read(ir)) { v = ir.busVolt_V; i = ir.current_mA; }
}

// NVS 标志：标定进行中（掉电重启后靠它知道要建表）
void setCalFlag(bool on) {
  Preferences p; p.begin("pht", false); p.putUChar("cal", on ? 1 : 0); p.end();
}
uint8_t getCalFlag() {
  Preferences p; p.begin("pht", true); uint8_t v = p.getUChar("cal", 0); p.end(); return v;
}

// 时刻字符串（未对时返回 "-"）
void calNowStr(char* out, size_t n) {
  time_t now = time(nullptr);
  if (!timeSynced || now < 1000000000) { snprintf(out, n, "-"); return; }
  struct tm tmv; localtime_r(&now, &tmv);
  snprintf(out, n, "%04d-%02d-%02d %02d:%02d:%02d",
           tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
}

// 新建 CSV（覆盖）+ 元数据头
void calCsvHeader() {
  char ts[24]; calNowStr(ts, sizeof(ts));
  File f = LittleFS.open(CAL_CSV_PATH, "w");
  if (!f) { webLogln("⚠️ 标定：无法创建 CSV"); return; }
  f.printf("# pht2 battcal v1 start=%s fw=v2.1.1b dry=%d v100=%.3f load_ms=%lu rest_ms=%lu low_from=%.2f\n",
           ts, calDry ? 1 : 0, isnan(calV100) ? 0.0f : calV100,
           (unsigned long)CAL_LOAD_MS, (unsigned long)CAL_REST_MS, CAL_LOW_FROM_V);
  f.printf("t_loaded_ms,dt_ms,v_load,v_rest,i_load_mA,i_rest_mA,temp_c,wl_ok,load_ok\n");
  f.printf("# 列义：wl_ok = 采样时刻 WiFi 在线；load_ok = 整个加载段 WiFi 全程在线（0 = 中途掉线，负载腰斩 → 建表时剔除）\n");
  f.flush(); f.close();
}

// 落一行样本（掉电是预期结局 → 每行立即 flush 并关闭）
void calCsvRow(uint32_t tLoaded, uint32_t dt) {
  File f = LittleFS.open(CAL_CSV_PATH, "a");
  if (!f) { webLogln("⚠️ 标定：CSV 打开失败，本行样本丢失"); return; }
  int wl = (WiFi.status() == WL_CONNECTED) ? 1 : 0;
  // 第9列 load_ok = **整个加载段** WiFi 是否全程在线（中途掉线 → 负载腰斩 → 建表时剔除该样本）
  //   ⚠️ 2026-09-16 修正：原先这两个位置误写成同一个 wl，设计里说的「负载健康标志 loadOk」从未真正测量
  int lo = calLoadBad ? 0 : 1;
  f.printf("%lu,%lu,%.3f,%.3f,%.1f,%.1f,%.1f,%d,%d\n",
           (unsigned long)tLoaded, (unsigned long)dt,
           isnan(calVLoad) ? 0.0f : calVLoad, isnan(calVRest) ? 0.0f : calVRest,
           isnan(calILoad) ? 0.0f : calILoad, isnan(calIRest) ? 0.0f : calIRest,
           isnan(calAmbTemp) ? 0.0f : calAmbTemp, wl, lo);
  f.flush(); f.close();
}

// ---- 表：存取 ----
void calSaveTable() {
  if (LittleFS.exists(CAL_JSON_PATH)) {                 // 备份旧表
    String old = ""; File o = LittleFS.open(CAL_JSON_PATH, "r");
    if (o) { old = o.readString(); o.close(); }
    if (old.length()) { File pv = LittleFS.open(CAL_PREV_PATH, "w"); if (pv) { pv.print(old); pv.close(); } }
  }
  char ts[24]; calNowStr(ts, sizeof(ts));
  snprintf(calTableDate, sizeof(calTableDate), "%s", ts);
  File f = LittleFS.open(CAL_JSON_PATH, "w");
  if (!f) return;
  f.printf("{\"v\":1,\"dry\":%d,\"n\":%d,\"date\":\"%s\",\"v100\":%.3f,\"v0\":%.3f,\"pct\":[",
           calTableDry ? 1 : 0, calTableN, calTableDate, calTableV100, calTableVEnd);
  for (int k = 0; k < 21; k++) f.printf("%s%.3f", k ? "," : "", calPctV[k]);
  f.printf("]}\n");
  f.flush(); f.close();
}

void calLoadTable() {
  calTableOk = false; calTableDry = false; calTableN = 0;
  if (!LittleFS.exists(CAL_JSON_PATH)) return;
  File f = LittleFS.open(CAL_JSON_PATH, "r");
  if (!f) return;
  String s = f.readString(); f.close();
  int i = s.indexOf("\"pct\":[");
  if (i < 0) return;
  i += 7;
  int j = s.indexOf(']', i);
  if (j < 0) return;
  String arr = s.substring(i, j);
  int k = 0, pos = 0;
  while (k < 21) {
    int comma = arr.indexOf(',', pos);
    String vs = (comma < 0) ? arr.substring(pos) : arr.substring(pos, comma);
    vs.trim();
    if (vs.length() == 0) break;
    calPctV[k++] = vs.toFloat();
    if (comma < 0) break;
    pos = comma + 1;
  }
  if (k != 21) return;
  int d = s.indexOf("\"dry\":");
  calTableDry = (d >= 0 && s.substring(d + 6, d + 7) == "1");
  int n = s.indexOf("\"n\":");
  if (n >= 0) calTableN = s.substring(n + 4, s.indexOf(',', n)).toInt();
  int dd = s.indexOf("\"date\":\"");
  if (dd >= 0) { int e = s.indexOf('"', dd + 8); if (e > 0) snprintf(calTableDate, sizeof(calTableDate), "%s", s.substring(dd + 8, e).c_str()); }
  int v1 = s.indexOf("\"v100\":");
  if (v1 >= 0) calTableV100 = s.substring(v1 + 7, s.indexOf(',', v1)).toFloat();
  int v0 = s.indexOf("\"v0\":");
  if (v0 >= 0) calTableVEnd = s.substring(v0 + 5, s.indexOf(',', v0)).toFloat();
  for (int m = 19; m >= 0; m--) if (calPctV[m] > calPctV[m+1]) calPctV[m] = calPctV[m+1];   // 单调保险
  calTableOk = true;
}

// 查表：电压 → 电量%
//   ⚠️ 表的顺序是**递增**：calPctV[0] = 0%（最低电压）… calPctV[20] = 100%（最高电压）
//   ⚠️ 2026-09-16 修正：原先插值循环按“递减”写（v>=calPctV[k+1] && v<=calPctV[k]），在递增表上永不命中
//      → 只要电压落在区间内部就一律落到 return 0 → 屏幕/网页电量长期显示 0%
//      （只有 v ≥ 100% 点时才走上面的 100% 分支，所以满电时看着“正常”）。
uint8_t calPctFromVolt(float v) {
  if (!calTableOk) return 0;
  if (v >= calPctV[20]) return 100;
  if (v <= calPctV[0])  return 0;
  for (int k = 0; k < 20; k++) {
    if (v >= calPctV[k] && v <= calPctV[k+1]) {        // 升段：[k] → [k+1] 即 k*5% → (k+1)*5%
      float den = calPctV[k+1] - calPctV[k];
      float r = (den > 0.0001f) ? (v - calPctV[k]) / den : 0.0f;
      return (uint8_t)(k * 5.0f + r * 5.0f + 0.5f);
    }
  }
  return 0;
}

// 由 /battcal.csv 建 21 点表；partial=true 时样本要求放宽（中止后用现有样本）
bool calBuildTable(bool partial) {
  if (!LittleFS.exists(CAL_CSV_PATH)) { webLogln("⚠️ 标定：无 CSV，无法建表"); return false; }
  File f = LittleFS.open(CAL_CSV_PATH, "r");
  if (!f) return false;
  static uint32_t tx[CAL_MAX_ROWS];
  static float    vx[CAL_MAX_ROWS];
  int n = 0; float v100 = NAN; bool dry = false;
  char line[192];
  while (f.available() && n < CAL_MAX_ROWS) {
    size_t len = f.readBytesUntil('\n', line, sizeof(line) - 1);
    if (len == 0) continue;
    line[len] = 0;
    if (line[0] == '#') {
      char* p = strstr(line, "v100="); if (p) v100 = atof(p + 5);
      if (strstr(line, "dry=1")) dry = true;
      continue;
    }
    if (line[0] == 't') continue;
    unsigned long t = 0, dt = 0; float vl = 0, vr = 0, il = 0, ir = 0, tc = 0; int wl = 0, lo = 0;
    if (sscanf(line, "%lu,%lu,%f,%f,%f,%f,%f,%d,%d", &t, &dt, &vl, &vr, &il, &ir, &tc, &wl, &lo) == 9) {
      if (!wl || !lo) continue;            // 负载异常样本剔除
      if (vr <= 0 || t == 0) continue;
      tx[n] = (uint32_t)t; vx[n] = vr; n++;
    }
  }
  f.close();
  int need = (partial || dry) ? 4 : 20;   // 干跑样本本来就只有几轮 → 用低阀值
  if (n < need) { webLog("⚠️ 标定：有效样本仅 %d 条（需 ≥%d），不建表\n", n, need); return false; }
  uint32_t total = tx[n-1];
  if (total < 1000) { webLogln("⚠️ 标定：时间轴过短，不建表"); return false; }
  float tab[21];
  tab[20] = isnan(v100) ? vx[0] : v100;    // 100% = 标定时的锚点（电池侧无电流时的电压）
  for (int k = 19; k >= 0; k--) {
    float want = (float)(20 - k) / 20.0f * (float)total;
    int i = 0; while (i < n && (float)tx[i] < want) i++;
    float v;
    if (i == 0)      v = vx[0];
    else if (i >= n) v = vx[n-1];
    else { float r = (want - (float)tx[i-1]) / (float)(tx[i] - tx[i-1]); v = vx[i-1] + r * (vx[i] - vx[i-1]); }
    tab[k] = v;
  }
  for (int k = 19; k >= 0; k--) if (tab[k] > tab[k+1]) tab[k] = tab[k+1];   // 强制单调递减
  for (int k = 0; k < 21; k++) calPctV[k] = tab[k];
  calTableV100 = tab[20]; calTableVEnd = tab[0];
  calTableN = n; calTableDry = dry; calTableOk = true;
  calSaveTable();
  webLog("✅ 标定建表：%d 样本，%.3fV(100%%) → %.3fV(0%%)%s\n",
         n, calTableV100, calTableVEnd, dry ? "（干跑，不生效）" : "");
  return true;
}

// 满电已保持时长（满电锚点与标定门槛共用）
uint32_t calFullHeldMs() { return calFullStartMs ? (uint32_t)(millis() - calFullStartMs) : 0; }

// ===== 低电三档：按**本机校准表**取（V2.1.1「低电特性更新」，2026-09-29）=====
//   规则：深睡 = 表的 5% 点 − 余量；停采 = 表的 10% 点；存档 = 表的 15% 点。
//   取表的实现：表是「电压 → 电量%」的 21 点映射，这里逐档试电压，找**实际显示为 k% 的最低电压**
//     （表单调递增、且 5% 一档，结果可靠）。
//   ⚠️ 必须用 calTableDry 挡住**干跑表**：干跑表会落盘且 calTableOk=true，拿它算阈值是错的。
//   ⚠️ 余量只加在深睡档：那一档是"保命线"，要提前于 LDO 崩溃点触发；停采/存档本身就在它上面。
//   ⚠️ 合理性检查（防脏表）：三档必须单调递减(存档 > 停采 > 深睡) 且 深睡 > CAL_FLOOR_V；
//      不满足就整体回落到回落值并在日志里报出来。
void updateBattThresholds() {
  battArchiveV = BATT_ARCHIVE_SD_DEF;      // 先回落到默认（无表/表不合格时就用它）
  battStopV    = BATT_STOP_RECORD_DEF;
  battSleepV   = BATT_DEEP_SLEEP_DEF;
  if (!calTableOk || calTableDry) {
    webLogln("🔋 低电三档：无本机表（或仅干跑表）→ 用回落值 %.2f / %.2f / %.2f V",
             battArchiveV, battStopV, battSleepV);
    return;
  }
  if (calPctV[0] >= calPctV[20]) {           // 表明显不具备"低→高"单调结构
    webLogln("⚠️ 本机表不单调（0%%=%.3fV ≥ 100%%=%.3fV）→ 低电三档回落到 %.2f/%.2f/%.2f V",
             calPctV[0], calPctV[20], battArchiveV, battStopV, battSleepV);
    return;
  }
  // 从低到高扫一遍，记下"首次显示为 k%"的电压 = 该百分比的最低电压
  float v5 = NAN, v10 = NAN, v15 = NAN;
  for (int k = 0; k < 21; k++) {
    uint8_t p = calPctFromVolt(calPctV[k]);
    if (isnan(v5)  && p >= 5)  v5  = calPctV[k];
    if (isnan(v10) && p >= 10) v10 = calPctV[k];
    if (isnan(v15) && p >= 15) v15 = calPctV[k];
  }
  if (isnan(v5) || isnan(v10) || isnan(v15)) {
    webLogln("⚠️ 本机表取 5/10/15%% 点失败 → 低电三档回落到 %.2f/%.2f/%.2f V",
             battArchiveV, battStopV, battSleepV);
    return;
  }
  // 深睡档：表 5% 点 − 余量，但**不得低于硬底线 + 50mV**
  //   注：本机表的 0% 点**可能略低于** CAL_FLOOR_V（标定是被硬底线收工的；旧表是 2.969V < 当时的 3.0V。2026-10-03 起底线改 3.20V），
  //       那正是实测到的真实底线，**不构成"表不可信"** → 这里只夹紧深睡档，不整表否定。
  float sleepRaw = v5 - BATT_SLEEP_MARGIN_V;
  float sleepMin = CAL_FLOOR_V + 0.05f;
  float sleepV   = (sleepRaw > sleepMin) ? sleepRaw : sleepMin;
  bool  clamped  = (sleepV > sleepRaw);
  if (v15 > v10 && v10 > sleepV && sleepV > CAL_FLOOR_V) {
    battArchiveV = v15; battStopV = v10; battSleepV = sleepV;
    webLogln("🔋 低电三档已按**本机表**刷新：存档 %.3fV(表15%%) / 停采 %.3fV(表10%%) / 深睡 %.3fV(表5%% %.3f−%.0fmV%s)",
             battArchiveV, battStopV, battSleepV, v5, BATT_SLEEP_MARGIN_V * 1000.0f,
             clamped ? "，已夹到硬底线+50mV" : "");
    diag("低电三档按表刷新：存档" + String(battArchiveV, 3) + " 停采" + String(battStopV, 3) +
         " 深睡" + String(battSleepV, 3));
  } else {
    webLogln("⚠️ 本机表算出的三档不单调（存档 %.3f / 停采 %.3f / 深睡 %.3f）→ 回落到 %.2f/%.2f/%.2f V",
             v15, v10, sleepV, battArchiveV, battStopV, battSleepV);
  }
}

// 满电锚点（设计文档 2.6）：移动 + 插电 + BQ 已停充 + 电压 ≥4.10V，持续 ≥5min
//   → 确实满了（充放共用回路无电流，量到的就是电芯静置电压）→ 刷新表内 100% 点
void serviceBattAnchor() {
  const PowerState& ps = g_power;
  bool full = (deviceMode == 1) && ps.powered && !ps.charging && ps.inaOK && (ps.battVolt >= CAL_FULL_MIN_V);
  if (!full) { calFullStartMs = 0; return; }
  if (calFullStartMs == 0) { calFullStartMs = millis(); return; }
  if ((uint32_t)(millis() - calFullStartMs) < CAL_FULL_HOLD_MS) return;
  if (!calTableOk || calTableDry) return;
  if (ps.battVolt >= calPctV[19] && fabsf(ps.battVolt - calPctV[20]) >= 0.02f) {
    webLog("🔋 满电锚点刷新：100%% 点 %.3fV → %.3fV\n", calPctV[20], ps.battVolt);
    calPctV[20] = ps.battVolt;
    calTableV100 = ps.battVolt;
    calSaveTable();
  }
}

// 开始标定（网页调用）
// V2.1.1-c：开始标定的前置判据（抽出来给 /battcal/status 复用 → 网页按钮能灰显并显示缺什么）
//   返回 true = 可以开始；false = why 说明原因。calStart 与网页**共用同一套判据**，不会两处不一致。
bool calStartCheck(bool dry, String& why) {
  if (calState == CAL_ARM || calState == CAL_RUN) { why = "已在标定中"; return false; }
  if (deviceMode != 1) { why = "仅【移动模式】可标定（固定只充到 80%，不能当 100% 基准）"; return false; }
  PowerState ps = readPowerState();
  if (!ps.inaOK) { why = "INA230 离线，电压不可信"; return false; }
  if (!dry) {
    if (!ps.powered)  { why = "请先插上充电器"; return false; }
    if (ps.charging)  { why = "正在充电：等 BQ 停充（充满）后再开始"; return false; }
    if (ps.battVolt < CAL_FULL_MIN_V) { why = String("电压 ") + String(ps.battVolt, 3) + "V 未达满电门限 4.10V"; return false; }
    unsigned long held = calFullHeldMs();
    if (held < CAL_FULL_HOLD_MS) {
      why = String("满电需保持 5 分钟（已 ") + String(held / 60000UL) + "分" + String((held / 1000UL) % 60) + "秒）";
      return false;
    }
  }
  return true;
}

bool calStart(bool dry, String& msg) {
  if (!calStartCheck(dry, msg)) return false;     // 判据与网页灰显同一套
  PowerState ps = readPowerState();               // 判据里已读过：这里重读一次拿 V_100（几 ms，无副作用）
  calDry = dry;
  calV100 = ps.battVolt;                 // 电池侧无电流 → 就是电芯静置电压
  // 一次性取环境温度（标定期间不再读传感器；温度对锂电曲线有影响，留作参考）
  calAmbTemp = NAN;
  if (shtOK) { float t = sht.readTemperature(); if (!isnan(t) && t > -40 && t < 85) calAmbTemp = t; }
  calLoadedMs = 0; calCycles = 0; calLastPct = -1; calPhase = 0;
  calVLoad = calVRest = calILoad = calIRest = NAN;
  calLastVolt = calV100; calLastI = 0;
  calCsvHeader();
  calInaBegin(true);
  setCalFlag(true);
  if (dry) {
    webLogln("🧪 标定【干跑】开始（缩短时长、保留网页、不要求充满）");
    calBeginDischarge();
    msg = "干跑已开始（缩短时长、保留网页）";
  } else {
    server.stop();                       // 设备隐身：网页下线，屏幕是唯一界面
    calState = CAL_ARM;
    webLog("🔋 标定就绪：V_100=%.3fV；请拔掉充电器（长按 IO9 可中止）\n", calV100);
    calScreen("CAL READY", "unplug charger");
    msg = "已就绪：请拔掉充电器开始放电";
  }
  return true;
}

void calBeginDischarge() {
  calStartMs = millis();
  calPhase = 0;
  calUiNextMs = 0;
  calState = CAL_RUN;
  calReadIna(calLastVolt, calLastI);
  webLogln("🔋 标定：开始放电循环（满载 / 静置 交替）");
  calScreenProgress();
}

// tryBuild=true：本次中止属于“正常跑完”（电压触底）→ 先建表；
//   建成功 → 清标志（不必再补建）；建失败 → **保留标志**，下次开机 calBootCheck 自动补建。
void calAbort(const char* why, bool tryBuild) {
  if (calState != CAL_ARM && calState != CAL_RUN) return;
  bool built = false;
  if (tryBuild) {
    built = calBuildTable(false);
    webLog("⛔ 标定跑完：%s%s\n", why, built ? "（已自动建表 ✓）" : "（建表失败 → 保留标志，下次开机补建）");
  } else {
    webLog("⛔ 标定中止：%s（样本保留在 %s，可用网页建部分表）\n", why, CAL_CSV_PATH);
  }
  calInaBegin(false);
  if (!tryBuild || built) setCalFlag(false);   // 手动中止 / 已建出表 → 清标志
  if (!calDry) server.begin();           // 恢复网页（干跑本来就没停）
  calState = CAL_STOP;
  uiDisplay.showToast(UI_TOAST_FAIL, "CAL ABORT", why, "", 8000);
}

// 标定主循环：loop() 每圈调一次；每次跑完一个完整阶段（加载或静置）
void serviceCal() {
  if (calState == CAL_ARM) {             // 待放电：等外部电源断开
    if (isExternallyPowered()) {
      if ((int32_t)(millis() - calUiNextMs) >= 0) { calUiNextMs = millis() + 2000; calScreen("CAL READY", "unplug charger"); }
      delay(200);
      return;
    }
    calBeginDischarge();
    return;
  }
  if (calState != CAL_RUN) return;
  if (calAbortPressed()) { calAbort("按钮长按"); return; }

  if (calPhase == 0) {
    // ---- ① 加载段：CPU 空转 + UDP 连发 ----
    bool low = (!isnan(calLastVolt) && calLastVolt < CAL_LOW_FROM_V);
    calCycleLoadMs = calDry ? CAL_DRY_LOAD_MS : (low ? CAL_LOAD_LOW_MS : CAL_LOAD_MS);
    uint32_t t0 = millis();
    calUiNextMs = 0;
    calLoadBad = (WiFi.status() != WL_CONNECTED);   // 本段负载健康：进段先看一次
    while ((uint32_t)(millis() - t0) < calCycleLoadMs) {
      if (calAbortPressed()) { calAbort("按钮长按"); return; }
      if (!calDry && isExternallyPowered()) { calAbort("检测到插电"); return; }
      calUdpBurst();
      calCpuBurn(180);
      delay(2);                            // 让出（喂 loop 任务看门狗）
      if (WiFi.status() != WL_CONNECTED) calLoadBad = true;   // 中途掉线 → 黏住（负载腰斩，样本作废）
      if (calDry) server.handleClient();   // 干跑：保留网页，便于远程观察
      if ((int32_t)(millis() - calUiNextMs) >= 0) {
        calUiNextMs = millis() + 30000;
        calReadIna(calLastVolt, calLastI);
        calScreenProgress();
      }
    }
    calReadIna(calVLoad, calILoad);        // 加载末电压/电流（8.4s 平均）
    calLastVolt = calVLoad; calLastI = calILoad;
    calPhase = 1;
    return;
  }

  // ---- ② 静置段：卸载（停流量 + CPU 空闲 + 不刷屏；WiFi 保持关联）→ 静置采样 ----
  {
    uint32_t restMs = calDry ? CAL_DRY_REST_MS : CAL_REST_MS;
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < restMs) {
      if (calAbortPressed()) { calAbort("按钮长按"); return; }
      if (!calDry && isExternallyPowered()) { calAbort("检测到插电"); return; }
      if (calDry) server.handleClient();
      delay(1000);
    }
    calReadIna(calVRest, calIRest);        // 醒来第一件事：只读 INA230（观察者效应）
    calLoadedMs += calCycleLoadMs;
    calCsvRow(calLoadedMs, calCycleLoadMs);
    calCycles++;
    calPhase = 0;
    webLog("📊 标定样本 #%lu：t=%lumin  V_load=%.3f V_rest=%.3f  I_load=%.0fmA I_rest=%.0fmA\n",
           (unsigned long)calCycles, (unsigned long)(calLoadedMs / 60000UL),
           calVLoad, calVRest, calILoad, calIRest);
    if (!isnan(calVRest) && calVRest < CAL_FLOOR_V) {   // 硬底线 = 正常跑完
      webLog("⛔ 标定：电压 %.2fV 低于硬底线 %.2fV → 收工\n", calVRest, CAL_FLOOR_V);
      calAbort("电压触底", true);                        // ← 触底即正常结束：先建表再收工
      return;
    }
    if (calDry && calCycles >= CAL_DRY_CYCLES) {        // 干跑：模拟掉电重启，验证建表路径
      webLogln("🧪 干跑轮数已到 → 重启，验证「掉电后开机建表」");
      delay(300);
      ESP.restart();
    }
    calScreenProgress();
  }
}

// why 文本的 JSON 转义（calStatusJson 用）：jsonEsc() 定义在 #if ENABLE_BLE 块内，
//   ENABLE_BLE=0 时整个块被条件编译掉 → 这里自带一个（只处理 " 和 \，够用）
String calWhyEsc(const String& in) {
  String o; o.reserve(in.length() + 8);
  for (unsigned int i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; } else o += c;
  }
  return o;
}

String calStatusJson() {
  String why, whyDry;
  bool canStart    = calStartCheck(false, why);      // V2.1.1-c：网页按钮灰显用（与点按钮时同一套判据）
  bool canStartDry = calStartCheck(true,  whyDry);
  String s = "{\"state\":" + String(calState) +
             ",\"dry\":" + String(calDry ? 1 : 0) +
             ",\"cycles\":" + String(calCycles) +
             ",\"elapsedMs\":" + String((calState == CAL_RUN) ? (millis() - calStartMs) : 0) +
             ",\"loadedMs\":" + String(calLoadedMs) +
             ",\"pct\":" + String(calLastPct < 0 ? 0 : calLastPct) +
             ",\"v\":" + (isnan(calLastVolt) ? String("null") : String(calLastVolt, 3)) +
             ",\"i\":" + (isnan(calLastI) ? String("null") : String(calLastI, 1)) +
             ",\"v100\":" + (isnan(calV100) ? String("null") : String(calV100, 3)) +
             ",\"tableOk\":" + (calTableOk ? "true" : "false") +
             ",\"tableDry\":" + (calTableDry ? "true" : "false") +
             ",\"tableN\":" + String(calTableN) +
             ",\"fullHeldMs\":" + String(calFullHeldMs()) +
             ",\"canStart\":" + (canStart ? "true" : "false") +
             ",\"canStartDry\":" + (canStartDry ? "true" : "false") +
             ",\"why\":" + "\"" + calWhyEsc(why) + "\"" +
             ",\"whyDry\":" + "\"" + calWhyEsc(whyDry) + "\"" + "}";
  return s;
}

String calReportJson() {
  String s = "{\"n\":" + String(calTableN) +
             ",\"dry\":" + String(calTableDry ? 1 : 0) +
             ",\"date\":\"" + String(calTableDate) + "\"" +
             ",\"v0\":" + String(calTableOk ? calTableVEnd : 0.0f, 3) +
             ",\"v100\":" + String(calTableOk ? calTableV100 : 0.0f, 3) + ",\"pct\":[";
  for (int k = 0; k < 21; k++) { if (calTableOk) { if (k) s += ","; s += String(calPctV[k], 3); } }
  s += "]}";
  return s;
}

// ===== 密钥 / 推送地址配置页（2026-10-05）=====
//   为什么加这个页面：这几项原先**只有 JSON 接口**（/pushcfg、/devkey、/devname），
//   加一台新机器得手敲 5 条 curl，很容易记错。
//
//   ⚠️ 但设备网页**没有任何鉴权**（同局域网的人都能打开），而这里的按钮能让设备
//      立刻和服务器失联。所以：
//        * 主页入口做得**低调** —— 系统卡末尾一行灰色小字，不是大按钮
//        * 页面内**顶部一大段警告**，危险操作**都要二次确认**（清除密钥要确认两次）
//        * 「注册到服务器」在未绑定密钥时**禁用**，避免顺序点错
//   本页只驱动已有的 JSON 接口，**没有新增任何后端逻辑**。
String keyPageHtml() {
  String h = R"rawliteral(<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>密钥与推送</title>)rawliteral" + String(WEB_ICONS) + R"rawliteral(
<style>body{font-family:system-ui;margin:12px;background:#f7f7f9;color:#222}
.card{background:#fff;border-radius:10px;padding:12px;margin-bottom:12px;box-shadow:0 1px 4px #0001}
.card.danger{border:1px solid #f0a0a0}
.btn{padding:9px 13px;margin:4px 4px 4px 0;border:none;border-radius:8px;color:#fff;font-size:14px;cursor:pointer}
.b1{background:#2c7be5}.b2{background:#e67e22}.b3{background:#dc3545}.b4{background:#6c757d}
.btn[disabled]{opacity:.4;cursor:not-allowed;filter:grayscale(.6)}
input{font-size:14px;padding:6px 8px;border:1px solid #ccc;border-radius:6px;margin:3px 0}
.warn{background:#fff4f4;border:1px solid #f0a0a0;color:#a01f1f;padding:9px 11px;border-radius:8px;font-size:13px;line-height:1.75}
.note{font-size:12.5px;color:#666;line-height:1.7}
#st{font-size:13px;color:#444;white-space:pre-wrap;line-height:1.75}
#msg{font-size:13px;margin-top:8px;min-height:18px;line-height:1.6}
a.back{font-size:13px;color:#2c7be5;text-decoration:none}</style></head><body>
<div class="card danger"><h3>⚠️ 先读这段</h3>
<div class="warn">
① 点 <b>生成新密钥</b> 或 <b>清除密钥</b> 之后，这台设备会<b>立刻无法向服务器推送数据</b>（服务器认的还是旧密钥）。<br>
② 换密钥后<b>必须马上点「注册到服务器」</b>，否则推送会一直 401。<br>
③ 这个页面<b>没有口令保护</b> —— 同一局域网里任何人都能打开、都能点。<b>别让别人随手动。</b>
</div>
<div class="note" style="margin-top:8px">正常「加一台新机器」顺序：配下面的推送地址 → 生成密钥 → 填绑定口令 → 注册到服务器 → 回主页看推送是否正常。</div>
</div>

<div class="card"><h3>🔐 当前状态</h3><div id="st">加载中…</div><div id="msg"></div></div>

<div class="card"><h3>📡 推送服务器地址</h3>
<div class="note">改这项只换目标服务器，<b>不影响密钥、不会导致失联</b>。</div>
<div>地址 <input id="ph" size="15" placeholder="192.168.1.100"> 端口 <input id="pp" size="6" placeholder="8080"></div>
<button class="btn b1" onclick="savePush()">保存地址</button>
<button class="btn b4" id="bOn" onclick="togglePush()">…</button>
</div>

<div class="card"><h3>🏷 设备名</h3>
<div class="note">默认是 <b>PHT-&lt;MAC后6位&gt;</b>，天生唯一，<b>建议不要改</b>。<br>
改名会向服务器<b>查重名</b>，所以必须先连上 WiFi；只能用字母/数字/点/下划线/连字符，1~32 字符。</div>
<div>名称 <input id="dn" size="18"></div>
<button class="btn b1" onclick="saveName()">保存名称</button>
</div>

<div class="card danger"><h3>🔑 密钥操作（危险）</h3>
<div class="note">绑定口令 = 服务器 <b>/etc/pht-server.env</b> 里的 <b>PHT_ENROLL_PASSWORD</b>。</div>
<div>绑定口令 <input id="epw" type="password" size="30" autocomplete="off" placeholder="注册时才需要"></div>
<div>
<button class="btn b2" id="bGen" onclick="doGen()">生成新密钥</button>
<button class="btn b1" id="bEnroll" onclick="doEnroll()">注册到服务器</button>
<button class="btn b3" id="bClear" onclick="doClear()">清除密钥</button>
</div>
<div class="note" style="margin-top:6px">顺序：① 生成 → ② 注册。<b>只注册不生成</b>是安全的（用于密钥与服务器不一致时重新同步）。</div>
</div>

<div class="card"><a class="back" href="/">← 返回主页</a></div>
<script>
function q(i){return document.getElementById(i);}
function api(u){return fetch(u).then(function(r){return r.json();});}
function post(u,o){
  var b=Object.keys(o).map(function(k){return encodeURIComponent(k)+'='+encodeURIComponent(o[k]);}).join('&');
  return fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b})
         .then(function(r){return r.json();});
}
function msg(t,err){var m=q('msg');m.style.color=err?'#c0392b':'#16794a';m.textContent=t;}

function load(){
  api('/devkey').then(function(k){
    q('st').textContent=
      '密钥    : '+(k.bound?'已绑定':'未绑定')+'\n'+
      'keyId   : '+(k.keyId||'--')+'\n'+
      '签名自检: '+(k.selftest?'通过':'失败')+'      时钟: '+(k.clockOk?'可用':'不可用')+'\n'+
      '设备名  : '+k.dev+'\n'+
      '服务器  : '+k.srv+'      WiFi: '+(k.wifi?'已连接':'未连接');
    q('bEnroll').disabled=!k.bound;
    q('bClear').disabled=!k.bound;
  });
  api('/push').then(function(p){
    var a=String(p.srv||'').split(':');
    q('ph').value=a[0]||''; q('pp').value=a[1]||'8080';
    q('bOn').textContent=p.enabled?'暂停推送':'开启推送';
    q('bOn').setAttribute('data-on',p.enabled?'1':'0');
  });
  api('/devname').then(function(d){
    q('dn').placeholder=d.default||'';
    if(!q('dn').value) q('dn').value=d.dev||'';
  });
}

function savePush(){
  post('/pushcfg',{on:'1',host:q('ph').value.trim(),port:q('pp').value.trim()}).then(function(r){
    if(r.ok){msg('已保存：'+r.srv);load();}else msg('保存失败',1);
  });
}
function togglePush(){
  var on=(q('bOn').getAttribute('data-on')==='1')?'0':'1';
  post('/pushcfg',{on:on,host:q('ph').value.trim(),port:q('pp').value.trim()}).then(function(r){
    msg(r.enabled?'推送已开启':'推送已暂停');load();
  });
}
function saveName(){
  post('/devname',{dev:q('dn').value.trim()}).then(function(r){
    if(r.ok){msg('设备名已保存：'+(r.dev||q('dn').value));load();}
    else msg('改名被拒：'+(r.err||'未知原因'),1);
  });
}
function doGen(){
  if(!confirm('确定要【生成新密钥】吗？\n\n生成之后，这台设备会立刻无法推送数据，\n必须马上再点【注册到服务器】才能恢复。\n\n继续？'))return;
  post('/devkey',{act:'gen'}).then(function(r){
    if(r.ok){msg('新密钥已生成（keyId '+r.keyId+'）。现在立刻点【注册到服务器】！',1);load();}
    else msg('生成失败',1);
  });
}
function doEnroll(){
  var pw=q('epw').value;
  if(!pw){msg('请先填绑定口令',1);return;}
  msg('注册中…');
  post('/devkey',{act:'enroll',pw:pw}).then(function(r){
    if(r.ok){msg('已注册到服务器。');load();}
    else msg('注册被拒。看「日志」页：401=口令错  403=设备IP不在允许网段  503=服务器没配口令',1);
  });
}
function doClear(){
  if(!confirm('确定要【清除密钥】吗？\n\n清除后这台设备将无法推送数据\n（旧静态 token 已轮换，回退路径也走不通）。\n\n继续？'))return;
  if(!confirm('再确认一次：真的要清除这台设备的密钥吗？'))return;
  post('/devkey',{act:'clear'}).then(function(r){msg('密钥已清除，设备现在无法推送。',1);load();});
}
load();
</script></body></html>)rawliteral";
  return h;
}

String calPageHtml() {
  String h = R"rawliteral(<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>🔋 电量校准</title>)rawliteral" + String(WEB_ICONS) + R"rawliteral(
<style>body{font-family:system-ui;margin:12px;background:#f7f7f9;color:#222}
.card{background:#fff;border-radius:10px;padding:12px;margin-bottom:12px;box-shadow:0 1px 4px #0001}
.btn{padding:10px 14px;margin:4px 4px 4px 0;border:none;border-radius:8px;color:#fff;font-size:14px;cursor:pointer}
.b1{background:#2c7be5}.b2{background:#e67e22}.b3{background:#dc3545}.b4{background:#6c757d}
.btn[disabled]{opacity:.4;cursor:not-allowed;filter:grayscale(.6)}
#gwhy{font-size:13px;margin-top:6px;color:#555}
#st{font-size:13px;color:#444;white-space:pre-wrap;line-height:1.6}
.warn{color:#c0392b;font-size:13px;line-height:1.6}
table{border-collapse:collapse;font-size:13px;margin-top:8px}td,th{border:1px solid #ddd;padding:2px 8px}</style></head><body>
<div class="card"><h3>🔋 电量校准（放电曲线标定）</h3><div id="st">加载中…</div></div>
<div class="card"><h3>开始</h3>
<div class="warn">⚠️ 开始后：<b>所有功能临时下线</b>（网页服务停止、数据停止记录、蓝牙不跑），屏幕成为唯一界面；预计 <b>8~13 小时</b>。<br>
需「移动模式 + 插上充电器 + 已充满（BQ 停充且电压 ≥4.10V 并保持 5 分钟）」，点开始后按提示拔掉充电器。</div>
<button class="btn b1" id="bStart" onclick="go(0)" disabled>开始正式标定</button>
<button class="btn b4" id="bDry" onclick="go(1)" disabled>🧪 干跑测试（缩短时长·保留网页）</button>
<div id="gwhy">检查条件中…</div></div>
<div class="card"><h3>操作 / 报告</h3>
<button class="btn b3" id="bStop" onclick="api('/battcal/stop').then(load)" disabled>立即中止</button>
<button class="btn b4" onclick="loadRep()">查看报告/曲线</button>
<button class="btn b4" onclick="if(confirm('用现有样本建部分表？'))api('/battcal/build').then(loadRep)">建部分表</button>
<button class="btn b4" onclick="if(confirm('清除已标定曲线（回到线性估算）？'))api('/battcal/clear').then(load)">清除曲线</button>
<div id="rep"></div></div>
<script>
function api(u){return fetch(u).then(function(r){return r.json();});}
function load(){api('/battcal/status').then(function(s){
 var m=Math.round((s.elapsedMs||0)/60000);
 document.getElementById('st').textContent=
  '状态: '+['未标定','已就绪（等拔电）','放电中','已中止'][s.state]+(s.dry?' [干跑]':'')+'\n'+
  '轮次: '+s.cycles+'   已历时: '+Math.floor(m/60)+'h'+(m%60)+'m   时间轴: '+Math.round(s.loadedMs/60000)+'min\n'+
  '当前: '+(s.v===null?'--':s.v.toFixed(3))+'V  '+(s.i===null?'--':s.i.toFixed(0))+'mA   进度估: '+s.pct+'%\n'+
  '满电锚点: '+(s.v100===null?'--':s.v100.toFixed(3))+'V   满电已保持: '+Math.round((s.fullHeldMs||0)/60000)+'min\n'+
  '曲线: '+(s.tableOk?(s.tableDry?'已建（干跑，不生效，'+s.tableN+' 样本）':'已生效（'+s.tableN+' 样本）'):'无');
 var run=(s.state===1||s.state===2);
 var bs=document.getElementById('bStart'),bd=document.getElementById('bDry'),bq=document.getElementById('bStop'),gw=document.getElementById('gwhy');
 bs.disabled=!s.canStart; bd.disabled=!s.canStartDry; bq.disabled=!run;
 gw.innerHTML = run ? '⏳ 标定进行中（正式跑时网页已下线，中止请长按 IO9）'
                    : (s.canStart ? '✅ 条件已满足，可以开始正式标定'
                                  : '⛔ 暂不可开始：<b>'+(s.why||'条件未满足')+'</b>');});}
function go(d){if((d?document.getElementById('bDry'):document.getElementById('bStart')).disabled){alert('当前条件不满足，不能开始（见按钮下方提示）');return;}var m=d?'开始干跑测试（缩短时长、保留网页）？':'开始正式标定？\n\n所有功能将临时下线、数据停止记录，预计 8~13 小时。\n确定后请按屏幕提示拔掉充电器。';
 if(!confirm(m))return; api('/battcal/start?dry='+d).then(function(r){alert(r.msg);load();});}
function loadRep(){api('/battcal/report').then(function(r){
 if(!r.n){document.getElementById('rep').innerHTML='<p>暂无曲线</p>';return;}
 var t='<p>样本 '+r.n+'，'+r.v100.toFixed(3)+'V(100%) → '+r.v0.toFixed(3)+'V(0%)，'+r.date+(r.dry?' 【干跑】':'')+'</p><table><tr><th>%</th><th>V</th></tr>';
 for(var k=20;k>=0;k--)t+='<tr><td>'+(k*5)+'%</td><td>'+r.pct[k].toFixed(3)+'</td></tr>';
 document.getElementById('rep').innerHTML=t+'</table>';});}
load();setInterval(load,3000);
</script></body></html>)rawliteral";
  return h;
}

// 开机自检：载入曲线表；若上次是「标定中途掉电」则建表并提示
void calBootCheck() {
  calLoadTable();
  updateBattThresholds();      // V2.1.1：低电三档跟本机表走（无表/干跑表 → 回落值）
  if (!getCalFlag()) return;
  setCalFlag(false);
  webLogln("🧾 检测到上次标定被掉电打断 → 用已落盘样本建表");
  if (calBuildTable(false)) {
    char l2[48];
    snprintf(l2, sizeof(l2), "%.3f-%.3fV", calTableV100, calTableVEnd);
    uiDisplay.showToast(UI_TOAST_OK, "CAL DONE", l2, "", 15000);
  } else {
    uiDisplay.showToast(UI_TOAST_FAIL, "CAL FAIL", "samples", "", 15000);
  }
}

// ===== V2.1.1-c 顶栏快通道：拔插/充电/WiFi 变化 → 立刻重画顶栏 =====
//   动机：原来顶栏只跟采样走（电池供电时整分钟才刷）→ 拔插充电器要等“半天”才有反馈，
//        与时序仪表的用途不符。
//   做法：**只读 GPIO / I2C，不碰采样、不写 SD** → 数据采集节奏完全不变（底线）。
//     ① 插拔(PGOOD) / 充电(CHG)：纯 GPIO，loop 每圈查（µs 级）→ 变化立刻刷顶栏，
//        并预约 +800ms 用新鲜 INA230 补刷一次（等电流/CC·CV 稳一下）
//     ② 每 5s：查 WiFi.status() + 读一次 INA230（~1ms），但**只有屏幕上会变的量**
//        （电量% / CC·CV / INA 在线 / 无线图标 / 分钟）真的变了才刷
//        → 电池放电时仍约 1 次/分钟，与原来“整分钟刷”同量级，不额外费电；
//        插电时 5s 内就能看到 CC→CV 与电量爬升
//   ⚠️ 标定期间不跑（loop 开头就 return，交给 serviceCal）
void serviceTopBar() {
  time_t t = time(nullptr);

  // ① 插拔 / 充电：GPIO 边沿 → 立刻刷
  bool pw = isExternallyPowered();                       // PGOOD 低 = 有外部电源
  bool cg = (digitalRead(PIN_CHG) == LOW);               // CHG 低 = 正在充电
  if (pw != (bool)topSeenPw || cg != (bool)topSeenChg) {
    bool plugChanged = (pw != (bool)topSeenPw);
    topSeenPw = pw; topSeenChg = cg;
    g_power.powered = pw; g_power.onBattery = !pw; g_power.charging = cg;
    uiSyncStatus();
    uiDisplay.showTop(t);
    topRefreshCount++;
    webLog("🔌 顶栏即时刷新：%s%s\n", pw ? "插电" : "拔电(转电池)", cg ? " + 充电中" : "");
    if (plugChanged) topEdgeAt = millis() + 800;         // 预约：等电压/电流稳一下再补一次
  }
  if (topEdgeAt && (int32_t)(millis() - topEdgeAt) >= 0) {
    topEdgeAt = 0;
    PowerState ps = readPowerState();                    // 一次 I2C 读（~1ms），不碰采样
    g_power = ps;
    applyChargeStrategy(ps);                             // 插拔后充电策略立刻生效（幂等，仅变化时打日志）
    uiSyncStatus();
    uiDisplay.showTop(t);
    topRefreshCount++;
    topSeenPct = ps.battPct; topSeenCv = ps.cvZone; topSeenIna = ps.inaOK;
  }

  // ② 每 5s：值变化检测（屏上没变化就不刷，省电）
  if ((int32_t)(millis() - topPollAt) < 0) return;
  topPollAt = millis() + 5000;
  PowerState ps = readPowerState();
  uint8_t ws = (curWireless == WL_BLE) ? UI_WIFI_BLE : wifiUIState();
  int mn = -1;
  if (t >= 1000000000) { struct tm tmv; localtime_r(&t, &tmv); mn = tmv.tm_min; }
  if (ws == topSeenWifi && mn == topSeenMin &&
      ps.powered == (bool)topSeenPw && ps.charging == (bool)topSeenChg &&
      ps.cvZone == topSeenCv && ps.inaOK == topSeenIna &&
      (int)ps.battPct == topSeenPct) return;
  topSeenWifi = ws; topSeenMin = mn;
  topSeenPw = ps.powered; topSeenChg = ps.charging;
  topSeenCv = ps.cvZone; topSeenIna = ps.inaOK; topSeenPct = ps.battPct;
  g_power = ps;
  applyChargeStrategy(ps);                               // 充电策略也更跟手（幂等）
  uiSyncStatus();
  uiDisplay.showTop(t);
  topRefreshCount++;
}


// ============================================================================
// ===== 浅睡省电（V2.1.1 · 省电模式）— 正式实现，2026-09-30 =====
//   行为：采样 → 落盘 → **睡满整段**（到下一个 30s 采样点前 300ms）→ 醒来采样 → 再睡 …
//   唤醒源：① RTC 定时器（到点自醒）② **ext1 GPIO 事件**：PGOOD(IO2) 插电 / 按钮(IO9) 按下
//           → 插电或按键可**立刻打断睡眠**，所以"插电立刻切模式、按键立刻响应"是即时生效的。
//   启用守卫（三条都必须满足才睡）：
//     ① `deviceMode == 1`（移动档）—— 固定档本来就插电
//     ② 电池供电（`!isExternallyPowered()`）—— 插电时没必要睡
//        ⚠️ **不再单独判"USB 主机已连接"**（2026-09-30 用户裁定）：USB 枚举必须先有外部供电，
//           而插电已被 ② 覆盖 → 该判定冗余。要防的"浅睡挂起 USB-Serial-JTAG"必然伴随外部供电。
//     ③ `curWireless == WL_OFF` —— **WiFi 会话期间绝不睡**：用户按按钮开网页（配网/看数据）时，
//           设备要是睡过去，网页就"没人应答"了 → 等于功能失效。按钮按下本身也会被 GPIO 唤醒。
//   落盘：CSV 记录不受影响（本功能不碰采样与写卡逻辑，只在两次采样之间睡）。
// ============================================================================

RTC_DATA_ATTR static uint32_t lightSleepCount = 0;   // 累计睡眠次数（RTC 域）
RTC_DATA_ATTR static uint32_t lightWakeCount  = 0;   // 累计醒来次数
uint32_t lightSleepFail = 0;                         // 连续失败计数（成功即清零）

// 进入一次浅睡（睡满调用方给的时长，单位 ms）
void lightSleepOnce(uint32_t sleepMs) {
  if (sleepMs < 50) return;
  if (sleepMs > 29000UL) sleepMs = 29000UL;      // 上限保险（正常 ≤29.7s）

  esp_sleep_enable_timer_wakeup((uint64_t)sleepMs * 1000ULL);
  // 事件唤醒：PGOOD(IO2) 低 = 插电；按钮(IO9) 低 = 按下（两者都是 RTC GPIO，可用 ext1）
  const uint64_t wakeMask = (1ULL << PIN_PGOOD) | (1ULL << PIN_WIFI_BTN);
  esp_sleep_enable_ext1_wakeup(wakeMask, ESP_EXT1_WAKEUP_ANY_LOW);

  lightSleepCount++;
  esp_err_t sret = esp_light_sleep_start();       // 醒来后从下一行继续（RAM / 外设状态保留）
  lightWakeCount++;

  if (sret != ESP_OK) {
    lightSleepFail++;
    if (lightSleepFail <= 3 || (lightSleepFail % 50 == 0)) {
      webLogln("⚠️ 浅睡返回错误 %d（累计 %lu 次）", (int)sret, (unsigned long)lightSleepFail);
    }
  } else {
    lightSleepFail = 0;
  }
  static uint32_t lastLogMs = 0;
  if (millis() - lastLogMs > 120000UL) {          // 每 2 分钟汇报一次（电池跑时便于事后取证据；不刷屏）
    lastLogMs = millis();
    esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
    const char* why = (wc == ESP_SLEEP_WAKEUP_EXT1) ? "事件(插电/按钮)" :
                      (wc == ESP_SLEEP_WAKEUP_TIMER ? "定时器" : "其它");
    webLogln("😴 浅睡：累计睡 %lu / 醒 %lu（最近唤醒源 %s）单片 %lums 失败 %lu",
             (unsigned long)lightSleepCount, (unsigned long)lightWakeCount, why,
             (unsigned long)sleepMs, (unsigned long)lightSleepFail);
  }
}

// 睡到"下一个 30s 采样点之前 300ms"。由 loop 每圈调用（不满足条件时立即返回，零开销）。
//   `nextT` = 下一个采样点的墙上时间戳（由 loop 的调度网格给出）→ 睡眠与采样**同一锚点**。
void lightSleepService(time_t nextT) {
  // 诊断：只要「电池供电」却没睡，就每 60 秒报一次原因（插电时完全不报，不刷屏）
  static uint32_t diagAt = 0;
  bool onBatt = !isExternallyPowered();
  if (onBatt && millis() - diagAt > 60000UL) {
    diagAt = millis();
    if (deviceMode != 1)            { webLogln("🔎 浅睡未启用：非移动档(mode=%d)", (int)deviceMode); return; }
    if (curWireless != WL_OFF)      { webLogln("🔎 浅睡未启用：无线在忙(curWireless=%d)", (int)curWireless); return; }
    if (lightSleepFail >= 5)        { webLogln("🔎 浅睡未启用：已熔断(失败%lu次)", (unsigned long)lightSleepFail); return; }
    if (!timeSynced)                { webLogln("🔎 浅睡未启用：时间未同步"); return; }
    webLogln("🔎 浅睡条件满足：睡到采样点（现在 %lu 目标 %lu 差 %lds）",
             (unsigned long)time(nullptr), (unsigned long)nextT, (long)(nextT - time(nullptr)));
  }
  if (deviceMode != 1) return;                   // 守卫①：仅移动档
  if (isExternallyPowered()) return;             // 守卫②：仅电池供电（同时覆盖 USB 主机情形）
  if (curWireless != WL_OFF) return;             // 守卫③：WiFi 会话期间不睡（否则网页会"没人应答"）
  if (lightSleepFail >= 5) return;               // 熔断：连续失败过多 → 放弃浅睡，回空转
  if (!timeSynced) return;
  // 睡眠时长 = 到下一个采样点的**秒差**，再留 200ms 给唤醒/调度开销
  time_t now = time(nullptr);
  if (nextT <= now) return;                      // 已经到点（交给 loop 采样）
  uint32_t sleepMs = (uint32_t)(nextT - now) * 1000UL;
  if (sleepMs < 500UL) return;                   // 太短就不睡（避免抖动）
  sleepMs -= 200UL;
  lightSleepOnce(sleepMs);                       // **整段睡到采样点**
}
void loop() {
  // ---- V2.1.1-b：标定期间接管一切（停记录 / 停传感器 / 停网页，屏幕为唯一界面）----
  if (calState == CAL_ARM || calState == CAL_RUN) { serviceCal(); return; }

  unsigned long nowMs = millis();
  time_t nowTime = time(nullptr);

  uiDisplay.tickToast();     // 提示横幅到期自动还原（极廉价：无横幅时立即返回）

  bleProcessPending();       // 蓝牙请求处理（放在 loop 主任务里：栈大，避免压崩协议栈任务）
  bleTxPump();               // 蓝牙响应分片发送（非阻塞）
  serviceWifiSession();      // 连网会话：按钮动作 / 配网后的连接（含阻塞式连网，放最后）

  // 无线互斥状态机 + 实体 WiFi 按钮（每 2 秒看一次；插拔电源/切模式/按按钮都会触发切换）
  checkWifiButton();                          // 每圈都查（一次 digitalRead，极廉价）：原先 2 秒轮询会漏掉短按
  serviceChargeTemp();                        // V2.1.1-a：断电即复位临时慢充（常态立即返回，零开销）
  serviceNtpSync();                           // NTP 非阻塞落地（写 RTC / 记时刻 / 超时回退）
  serviceNtpDailyCheck();                     // NTP 每日 02:00 定期重同步
  servicePr1Interlock(readPowerState());      // v2.2：TPS2117 PR1 电池直供互锁（低电才切；>3.6V 硬禁）
  serviceBattAnchor();                        // V2.1.1-b：满电锚点（满电保持 5min → 刷新表内 100% 点）
  serviceTopBar();                            // V2.1.1-c：顶栏快通道（拔插/充电/WiFi 变化 → 立刻反映）
  if (nowMs - lastWirelessCheck > 2000) {
    lastWirelessCheck = nowMs;
    updateWireless();
  }

  // ---- 采样 + 浅睡：**统一调度**（2026-10-01 重做）----
  //   锚点 = 采样点网格（:00/:30 对齐）。每轮都按"下一个采样点"算睡眠时长，
  //   醒来（定时器或 GPIO 事件）后立刻采样 → 采样**真的落在 :00/:30**，不靠事后对齐。
  //   ⚠️ 历史教训：① 判据写"必须正好落在整秒"→ 浅睡醒来差几百毫秒就错过 → 节拍漂成 26~65s；
  //                ② 睡眠时长用墙上时间与 millis() 混算 → 算出 700ms 怪值。
  //      现在：睡眠时长用**墙上时间到采样点的秒差**、采样用**同一网格**，两者同源 → 不再漂。
  if (timeSynced && nowTime >= 1000000000) {
    if (nextSampleTime == 0) {
      // 首次：对齐到下一个 :00 / :30
      nextSampleTime = nowTime - (nowTime % INTERVAL_SEC) + INTERVAL_SEC;
    }
    if (nowTime >= nextSampleTime) {
      lastCollectTime = nowTime;
      uint32_t behind = (uint32_t)(nowTime - nextSampleTime);
      do { nextSampleTime += INTERVAL_SEC; } while (nextSampleTime <= nowTime);
      if (behind >= 2 * INTERVAL_SEC) {
        webLogln("⚠️ 采样落后 %lus（已追上，跳过错过的槽位）", (unsigned long)behind);
      }
      readAndLog();
      // ---- NTP 重试（见上方策略）：**只有插电 + WiFi 已连 + ≥3 天没成功** 才试；每次开机最多一次。
      //    ⛔ 不满足这三条就一个字都不做（尤其电池供电时绝不去连网校时）。
      serviceNtpDailyCheck();   // 每日 02:00 的定期重同步（取代原来的"每次开机只试一次"）
    } else {
      lightSleepService(nextSampleTime);   // 睡到采样点（守卫不满足时立即返回）
    }
  } else {
    if (nowMs - fallbackLastMs >= INTERVAL_SEC * 1000UL) {
      fallbackLastMs = nowMs;
      readAndLog();
      // ---- NTP 重试（见上方策略）：**只有插电 + WiFi 已连 + ≥3 天没成功** 才试；每次开机最多一次。
      //    ⛔ 不满足这三条就一个字都不做（尤其电池供电时绝不去连网校时）。
      serviceNtpDailyCheck();   // 每日 02:00 的定期重同步（取代原来的"每次开机只试一次"）
    }
  }

  // ---- AP 自动管理：连上WiFi关AP；断线超时重开AP（防失联） ----
  if (apEnabled) {
    if (WiFi.status() == WL_CONNECTED) {     // 热点开着时又连回路由器了 → 弹成功提示
      stopAP();
      char tl2[48], tl3[32];
      buildSsidLine(tl2, sizeof(tl2), wifiCfg.ssid);
      snprintf(tl3, sizeof(tl3), "%s", WiFi.localIP().toString().c_str());
      uiDisplay.showToast(UI_TOAST_OK, wifiOkTitle(), tl2, tl3, 20000);
    }
  } else {
    // ⚠ 只在「当前确定用 WiFi」时走这套重连/AP 兼底——否则蓝牙模式下会误开 AP（实测踩到，违反互斥）
    if (wifiRecoverUntil && (int32_t)(millis() - wifiRecoverUntil) < 0) {
      // 【兜底】AP 救援窗口内：**禁止任何 STA 重连**（否则会把刚开的热点顶掉、又变回够不着）
      apClosedMs = 0;
    } else if (curWireless != WL_WIFI) {
      apClosedMs = 0;                    // 蓝牙模式：WiFi 本就该关着，不参与重连
    } else if (strlen(wifiCfg.ssid) > 0 && WiFi.status() != WL_CONNECTED) {
      if (apClosedMs == 0) apClosedMs = nowMs;
      if (nowMs - apClosedMs > 60000UL) {   // 断线 60 秒重开 AP
        apClosedMs = 0;
        webLogln("⚠️ WiFi 断线，重新开启 AP 配置模式");
        startAP();
        if (wifiCount > 1) wifiPickBest(false);           // 多组：重连前重新挑最强的
        WiFi.begin(wifiCfg.ssid, wifiCfg.password);  // 后台继续重连
      }
    } else {
      apClosedMs = 0;
    }
  }

  servicePushTick();   // v2.2 推送：微秒级判断（插电/首次联网 → 置位；HTTP 全在独立任务里）

  archivePump();       // 归档分片泵：每圈最多 40 行，**不阻塞采样**（2026-10-02）
  checkAndArchive();
  server.handleClient();
}
