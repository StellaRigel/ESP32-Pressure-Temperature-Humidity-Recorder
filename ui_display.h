#pragma once
// ============================================================================
//  ui_display.h —— ST7305 2.13" 【横屏】UI 模块（逻辑 250x122 / 物理面板 122x250）
// ----------------------------------------------------------------------------
//  无底图：整屏（含「温度/湿度/气压」中文标签）全部由本文件动态绘制，主程序不碰任何坐标。
//  中文标签字模在 ui_cjk.h（23px，从原底图 ui_landscape.h 原样抠出，往返比对 0 差异）。
//
//      顶栏:  时间 HH:MM │ 模式 MOV/FIX │ 充电 ⚡CC/⚡CV 或 电量% │ 电池 │ WiFi
//      数据:  温度 26.5°C │ 湿度 58% │ 气压 1013hPa
//
//  主逻辑用法：
//      uiDisplay.init();
//      uiDisplay.setPower(ps.charging, ps.cvZone, ps.inaOK, ps.battPct);
//      uiDisplay.setWifi(UI_WIFI_STA);
//      uiDisplay.setMode(deviceMode);                      // 0=固定(FIX) 1=移动(MOV)
//      uiDisplay.showAll(temp, hum, pres, time(nullptr));  // 数值+顶栏 全刷
//      uiDisplay.showTop(time(nullptr));                   // 只刷顶栏（电池供电省电）
//      uiDisplay.showToast(UI_TOAST_OK, "WiFi OK", "SSID mywifi", "192.168.1.9");
//      uiDisplay.queueToast(UI_TOAST_AP, "AP MODE", "SSID hotspot", "192.168.5.1");  // 到期接着弹
//      uiDisplay.tickToast();                              // loop() 里每次调用（到期自动还原/接着弹）
//
//  方向：UI_ROT=90 正常；上下颠倒改 270（底图已删，两个方向都不用另存图片）
// ============================================================================

#include <Arduino.h>
#include <math.h>
#include "ST7305_2p13.h"
#include "ui_font_oldsans.h"    // OldSansBlack 点阵字模（顶栏 13px / 数据区 20px，比例字宽）
#include "ui_cjk.h"             // 中文点阵 23px（温/度/湿/气/压，从原底图原样抠出）

// ---- 屏幕方向 ----（底图已删，90/270 都无需另存图片）
#define UI_ROT 90          // ← 上下颠倒就改成 270

// ---- 无线状态（主逻辑传这三个之一）----
#define UI_WIFI_OFF  0     // 无连接（图标带斜杠）
#define UI_WIFI_STA  1     // 已连接路由器
#define UI_WIFI_AP   2     // AP 热点
#define UI_WIFI_BLE  3     // 蓝牙（顶栏直接显示文字 BT）

// ---- 提示横幅（Toast）类型：决定第一行左侧的自绘图标 ----
#define UI_TOAST_OK    0   // √
#define UI_TOAST_FAIL  1   // ×
#define UI_TOAST_AP    2   // 广播图标（圆点 + 三弧）

// ---- 充电阶段(CC/CV) 与电量：全部由主逻辑用 INA230 判定后传入 ----
//      屏幕不再自行用电压近似（见 setPower()）。INA230 离线时电量区改显告警。

// ---- 24h 变率右对齐基准（逻辑 x，屏幕逻辑宽 250）----
#define UI_TREND_RIGHT  247
// 变率阈值：|Δ| < 阈值 视为平稳(→)
//   ⚠️ 湿度那一档用的是**绝对湿度(g/m³)**，与温度/气压量纲不同，阈值必须单独定：
//      0.1 是为 %RH 定的；换成 g/m³ 后 0.1 ≈ 0.4%RH（比传感器噪声还小 → 箭头永远在跳）。
//      实测室温下 1%RH ≈ 0.23 g/m³，故取 0.5 g/m³ ≈ 2.2%RH，正好在"正常波动"与"明显趋势"之间。
#define UI_TREND_TH     0.1f     // 温度(°C) / 气压(hPa) 的阈值
#define UI_TREND_TH_AH  0.5f     // 湿度（绝对湿度 g/m³）的阈值

// ============================================================================
//  1. 字体 —— OldSansBlack 点阵（见 ui_font_oldsans.h）
// ----------------------------------------------------------------------------
//   顶栏 : uiFontTop  20 列x19 行  数字高 ~13px   (em22 x 0.80)
//   数据 : uiFontBot  24 列x27 行  数字高 ~20px   (em22 x 1.15)
//   比例字体: 每个字符宽度不同, 笔位用 uiFontTopAdv[]/uiFontBotAdv[] 前进
//   绘制: 字形框左上角 = (pen - UIFONT_*_PAD, y)
// ============================================================================
#if 0   // ↓↓↓ 旧 5x7 字模, 保留备查, 已弃用 ↓↓↓
static const uint8_t uiFont5x7[][5] = {
  {0x3E,0x51,0x49,0x45,0x3E}, // 0   idx0
  {0x00,0x42,0x7F,0x40,0x00}, // 1
  {0x42,0x61,0x51,0x49,0x46}, // 2
  {0x21,0x41,0x45,0x4B,0x31}, // 3
  {0x18,0x14,0x12,0x7F,0x10}, // 4
  {0x27,0x45,0x45,0x45,0x39}, // 5
  {0x3C,0x4A,0x49,0x49,0x30}, // 6
  {0x01,0x71,0x09,0x05,0x03}, // 7
  {0x36,0x49,0x49,0x49,0x36}, // 8
  {0x06,0x49,0x49,0x29,0x1E}, // 9
  {0x00,0x00,0x60,0x60,0x00}, // .   idx10
  {0x08,0x08,0x08,0x08,0x08}, // -   idx11
  {0x00,0x36,0x36,0x00,0x00}, // :   idx12
  {0x00,0x00,0x00,0x00,0x00}, // 空格 idx13
  {0x3E,0x41,0x41,0x41,0x22}, // C   idx14
  {0x0F,0x30,0x40,0x30,0x0F}, // V   idx15
  {0x43,0x30,0x08,0x06,0x61}, // %   idx16
  {0x06,0x09,0x09,0x06,0x00}, // °   idx17  (字节值 0xB0)
  {0x7F,0x04,0x04,0x78,0x00}, // h   idx18
  {0x7F,0x09,0x09,0x09,0x06}, // P   idx19
  {0x20,0x54,0x54,0x54,0x78}, // a   idx20
  {0x7F,0x08,0x08,0x08,0x7F}, // H   idx21
  {0x7E,0x11,0x11,0x11,0x7E}, // A   idx22
  {0x7F,0x02,0x04,0x08,0x7F}, // N   idx23
  {0x7F,0x41,0x41,0x22,0x1C}, // D   idx24
  {0x46,0x49,0x49,0x49,0x31}, // S   idx25
  {0x00,0x41,0x7F,0x41,0x00}, // I   idx26
  {0x01,0x01,0x7F,0x01,0x01}, // T   idx27
  {0x7F,0x49,0x49,0x49,0x41}, // E   idx28
};
static int uiFontOldIdx(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c == '.') return 10;
  if (c == '-') return 11;
  if (c == ':') return 12;
  if (c == 'C') return 14;
  if (c == 'V') return 15;
  if (c == '%') return 16;
  if ((unsigned char)c == 0xB0) return 17;   // °
  if (c == 'h') return 18;
  if (c == 'P') return 19;
  if (c == 'a') return 20;
  if (c == 'H') return 21;
  if (c == 'A') return 22;
  if (c == 'N') return 23;
  if (c == 'D') return 24;
  if (c == 'S') return 25;
  if (c == 'I') return 26;
  if (c == 'T') return 27;
  if (c == 'E') return 28;
  return 13;   // 空格 / 未知
}
#endif  // ↑↑↑ 旧 5x7 字模结束 ↑↑↑

// ============================================================================
//  2. WiFi 图标（17x10，每行一个 17bit 掩码，bit0 = 最左列）
// ============================================================================
static const uint32_t uiWifiIcons[3][10] = {
  // [0] 断开：弧 + 圆点 + 斜杠
  { 0x06fe0U,0x03c78U,0x0780cU,0x0cfc6U,0x09e72U,0x03318U,0x00380U,0x007c0U,0x00160U,0x00130U },
  // [1] 已连接(STA)：三段弧 + 圆点
  { 0x00fe0U,0x03c78U,0x0600cU,0x0c7c6U,0x09c72U,0x03018U,0x00380U,0x007c0U,0x00100U,0x00100U },
  // [2] AP 热点：中心点 + 两侧各两段弧
  { 0x01010U,0x03018U,0x0600cU,0x04824U,0x05934U,0x05bb4U,0x05934U,0x04824U,0x0600cU,0x03018U },
};

// ============================================================================
//  3. UI 类
// ============================================================================
class UiDisplay {
public:
    // ---- 状态：主逻辑直接写，或用下面的 setter ----
    bool    charging = false;          // 充电中
    bool    cvPhase  = false;          // 充电相位: false=CC(恒流) true=CV(恒压)
    bool    inaValid = true;           // INA230 是否在线（false → 电量/电池区改显告警）
    bool    pr1Direct = false;         // v2.2：TPS2117 是否电池直供（VIN1）；true → 顶栏显示直供标识
    uint8_t battPct  = 0;              // 电量 0..100（仅 inaValid 时有效）
    uint8_t wifiStat = UI_WIFI_OFF;    // UI_WIFI_OFF / STA / AP
    bool    onSite   = true;           // true=固定(FIX) false=移动(MOV)

    // ---- 24h 变率（三要素，整点更新一次；NaN = 无数据，不显示）----
    float   trendT   = NAN;
    float   trendH   = NAN;
    float   trendP   = NAN;

    void init() {
        lcd.begin();
        redrawAll(time(nullptr));     // 底图已删 → 开机即全量动态重绘（白底 + 标签 + 顶栏）
    }

    // 全部由主逻辑（INA230）判定后传入；屏幕不再自行近似
    void setPower(bool chg, bool cv, bool inaOK, uint8_t pct) {
        charging = chg;
        cvPhase  = cv;                            // 来自 INA230 cvZone
        inaValid = inaOK;                         // false → 电量/电池区改为告警
        battPct  = pct;
    }
    // v2.2：TPS2117 PR1 电源通路 —— true = 电池直供（VIN1），false = LDO（VIN2）
    void setPr1(bool batteryDirect) { pr1Direct = batteryDirect; }
    // ---- 直供标识 (11x11)：电池直供时画在顶栏 FIX/MOV 与电量之间的空档 ----
    //   只在"电池直供"这个**罕见的低压状态**下出现，常态不占位，避免顶栏拥挤。
    //   位置 x=95,y=3：FIX/MOV 结束于 x≈83（留 12px 间隙），指示区起点 x=152 之后才被占用。
    void drawPr1Mark(int lx, int ly) {
        for (int x = 0; x <= 10; x++) { px(lx + x, ly, true); px(lx + x, ly + 10, true); }
        for (int y = 1; y <= 9;  y++) { px(lx, ly + y, true); px(lx + 10, ly + y, true); }
        px(lx + 3, ly + 3, true); px(lx + 4, ly + 3, true);          // 内部一个"直通"小点
        px(lx + 6, ly + 5, true); px(lx + 7, ly + 5, true);
        px(lx + 3, ly + 7, true); px(lx + 4, ly + 7, true);
    }
    void setWifi(uint8_t st)   { wifiStat = st; }
    void setMode(uint8_t mode) { onSite = (mode == 0); }   // 0=固定 1=移动
    // 24h 变率由主逻辑整点算好后传入（每小时一次）；NaN 表示无数据
    void setTrend(float dT, float dH, float dP) { trendT = dT; trendH = dH; trendP = dP; }

    // ---- 整屏重绘：白底 → 中文标签 → 顶栏 →（可选）三要素数值 ----
    //  底图删除后这是唯一的「全屏」入口；temp/hum/pres 传 NAN 则只画标签+顶栏
    void redrawAll(time_t t, float temp = NAN, float hum = NAN, float pres = NAN) {
        lcd.fill(0x00);                 // 清成白底（一次 memset，比 loadBuffer 还快）
        drawLabels();
        drawTop(t);
        if (!isnan(temp) || !isnan(hum) || !isnan(pres)) drawClimate(temp, hum, pres);
        lcd.display();
    }

    // ---- 全刷：顶栏 + 三要素数值（顺带缓存，供提示横幅到期还原）----
    void showAll(float temp, float hum, float pres, time_t t) {
        lastTemp = temp; lastHum = hum; lastPres = pres;
        if (toastOn) { showTop(t); return; }     // Toast 期间只刷顶栏，别把横幅冲掉
        redrawAll(t, temp, hum, pres);
    }

    // ---- 只刷顶栏：数值区保持（电池供电时按整分钟调用，省电）----
    void showTop(time_t t) {
        fillRect(0, 0, 250, 22, false);   // 顶栏擦回白底（须覆盖新字体高度 20 行 @ly=1）
        drawTop(t);
        lcd.display();
    }

    // ---- 读回逻辑像素（仅供 /ui-dump 自检排版，不参与显示）----
    bool readLogical(int lx, int ly) {
        if (lx < 0 || lx >= 250 || ly < 0 || ly >= 122) return false;
#if UI_ROT == 270
        return lcd.readPoint((uint16_t)ly, (uint16_t)(249 - lx));
#else
        return lcd.readPoint((uint16_t)(121 - ly), (uint16_t)lx);
#endif
    }

    // ==================== 提示横幅（Toast）====================
    //  整条 250px 宽，盖住数据区（含中文标签与数值），**顶栏不动**；
    //  到期由 tickToast() 自动用缓存的最近一次数值全量重绘还原；
    //  Toast 期间 showAll() 只刷顶栏，不会把横幅冲掉。
    //    kind : UI_TOAST_OK / UI_TOAST_FAIL / UI_TOAST_AP
    //    l1/l2/l3 : 三行文字（l2 走 13px 小字，l1/l3 走 20px 大字）
    //  注意：调用 showToast() 会取消已排队的下一条（新事件优先）。
    void showToast(uint8_t kind, const char *l1, const char *l2, const char *l3, uint32_t durMs = 20000) {
        qOn = false;                                 // 新横幅顶掉排队中的那条
        showToastNow(kind, l1, l2, l3, durMs);
    }
    // 排队下一条：当前横幅到期后自动接着显示（参数立即拷贝，可传局部缓冲区）
    void queueToast(uint8_t kind, const char *l1, const char *l2, const char *l3, uint32_t durMs = 20000) {
        qKind = kind; qDur = durMs;
        snprintf(qL1, sizeof(qL1), "%s", l1);
        snprintf(qL2, sizeof(qL2), "%s", l2);
        snprintf(qL3, sizeof(qL3), "%s", l3);
        qOn = true;
    }
    bool toastActive() const { return toastOn; }
    // 立刻收起横幅（网页被访问 → 确认用户已进来，不必等它自然到期）
    void dismissToast() {
        if (!toastOn) return;
        toastUntil = millis() - 1;
        tickToast();
    }
    // loop() 里调用；到期后：有排队就接着显示，否则还原（用缓存的最近一次数值）
    void tickToast() {
        if (toastOn) {
            if ((int32_t)(millis() - toastUntil) < 0) return;   // 还没到期
            toastOn = false;
            if (qOn) { qOn = false; showToastNow(qKind, qL1, qL2, qL3, qDur); return; }
            redrawAll(time(nullptr), lastTemp, lastHum, lastPres);
            return;
        }
        if (qOn) { qOn = false; showToastNow(qKind, qL1, qL2, qL3, qDur); }
    }
    // 量文本宽度（逻辑像素）：主逻辑拼长文本时用来判断会不会超出屏幕
    int textWidthLogical(const char *s, bool top = false) { return textWidth(s, top); }

private:
    // 真正把横幅画上去（showToast / tickToast 内部使用）
    void showToastNow(uint8_t kind, const char *l1, const char *l2, const char *l3, uint32_t durMs) {
        fillRect(0, 22, 250, 100, false);            // 数据区刷白（中文标签一并盖掉）
        toastLine1(kind, l1);                        // 图标 + 大字，整块居中
        drawTextC(63, l2, true);
        drawTextC(87, l3, false);
        lcd.display();
        toastOn    = true;
        toastUntil = millis() + durMs;
    }

    ST7305_2p13 lcd;

    // 最近一次数据（提示横幅到期还原用）
    float lastTemp = NAN, lastHum = NAN, lastPres = NAN;
    // 提示横幅状态
    bool     toastOn    = false;
    uint32_t toastUntil = 0;
    // 排队中的下一条横幅
    bool     qOn   = false;
    uint8_t  qKind = 0;
    uint32_t qDur  = 0;
    char     qL1[32] = {0}, qL2[48] = {0}, qL3[32] = {0};

    // ---- 逻辑坐标 -> 物理坐标 ----
    void px(int lx, int ly, bool on) {
#if UI_ROT == 270
        lcd.writePoint(ly, 249 - lx, on);
#else
        lcd.writePoint(121 - ly, lx, on);
#endif
    }
    void fillRect(int lx, int ly, int w, int h, bool on) {
        for (int y = 0; y < h; y++)
            for (int x = 0; x < w; x++) px(lx + x, ly + y, on);
    }
    // ---- 绘制一个字符（OldSansBlack 点阵，比例字体）----
    //   lx : 字符笔位(pen)的 x（字形实际从 lx-PAD 开始画）
    //   ly : 字形框顶部 y
    //   top: true=顶栏字体  false=数据区字体
    void drawChar(int lx, int ly, uint16_t cp, bool top) {
        int fi = uiFontIdx(cp);
        if (fi < 0) fi = 0;
        if (top) {
            const int ox = lx - UIFONT_TOP_PAD;
            for (int col = 0; col < UIFONT_TOP_COLS; col++) {
                uint32_t bits = 0;
                for (int b = 0; b < UIFONT_TOP_BYTES; b++)
                    bits |= (uint32_t)uiFontTop[fi][col][b] << (8 * b);
                for (int row = 0; row < UIFONT_TOP_ROWS; row++)
                    if (bits & (1UL << row)) px(ox + col, ly + row, true);
            }
        } else {
            const int ox = lx - UIFONT_BOT_PAD;
            for (int col = 0; col < UIFONT_BOT_COLS; col++) {
                uint32_t bits = 0;
                for (int b = 0; b < UIFONT_BOT_BYTES; b++)
                    bits |= (uint32_t)uiFontBot[fi][col][b] << (8 * b);
                for (int row = 0; row < UIFONT_BOT_ROWS; row++)
                    if (bits & (1UL << row)) px(ox + col, ly + row, true);
            }
        }
    }

    // ---- 绘制字符串（按每字符实际宽度前进）----
    void drawText(int lx, int ly, const char *str, bool top) {
        for (const unsigned char *p = (const unsigned char *)str; *p; p++) {
            int fi = uiFontIdx((uint16_t)*p);
            drawChar(lx, ly, (uint16_t)*p, top);
            lx += top ? uiFontTopAdv[fi] : uiFontBotAdv[fi];
        }
    }

    // ---- 绘制字符串（额外字距 extra px）----
    void drawTextSp(int lx, int ly, const char *str, bool top, int extra) {
        for (const unsigned char *p = (const unsigned char *)str; *p; p++) {
            int fi = uiFontIdx((uint16_t)*p);
            drawChar(lx, ly, (uint16_t)*p, top);
            lx += (top ? uiFontTopAdv[fi] : uiFontBotAdv[fi]) + extra;
        }
    }

    // ---- 计算字符串像素宽（用于右对齐）----
    int textWidth(const char *str, bool top) {
        int w = 0;
        for (const unsigned char *p = (const unsigned char *)str; *p; p++) {
            int fi = uiFontIdx((uint16_t)*p);
            w += top ? uiFontTopAdv[fi] : uiFontBotAdv[fi];
        }
        return w;
    }

    // ---- 中文点阵（ui_cjk.h，23px）----
    const UiCjkGlyph* cjkFind(uint32_t cp) {
        for (int i = 0; i < uiCjkCount; i++)
            if (uiCjkTable[i].cp == cp) return &uiCjkTable[i];
        return nullptr;
    }
    // 画一个汉字（lx,ly = 笔位）
    void drawCjk(int lx, int ly, uint32_t cp) {
        const UiCjkGlyph* g = cjkFind(cp);
        if (!g) return;
        int stride = (g->w + 7) / 8;
        for (int r = 0; r < g->h; r++)
            for (int c = 0; c < g->w; c++)
                if (g->bits[r * stride + (c >> 3)] & (0x80 >> (c & 7)))
                    px(lx + g->xoff + c, ly + g->yoff + r, true);
    }
    // 画 UTF-8 中文字串（字距固定 UI_CJK_ADV，与底图原样一致）
    void drawCjkText(int lx, int ly, const char *s) {
        while (*s) {
            unsigned char c0 = (unsigned char)*s;
            uint32_t cp = c0; int n = 1;
            if (c0 >= 0x80) {
                if ((c0 >> 5) == 0x06)      { cp = c0 & 0x1F; n = 2; }
                else if ((c0 >> 4) == 0x0E) { cp = c0 & 0x0F; n = 3; }
                else                        { cp = c0 & 0x07; n = 4; }
                for (int i = 1; i < n; i++) cp = (cp << 6) | ((unsigned char)s[i] & 0x3F);
            }
            drawCjk(lx, ly, cp);
            lx += UI_CJK_ADV;
            s += n;
        }
    }
    // ---- 三个中文标签：坐标与删掉的底图完全一致 ----
    void drawLabels() {
        drawCjkText(6, 25, "温度");
        drawCjkText(6, 57, "湿度");
        drawCjkText(6, 90, "气压");
    }

    // ---- Toast：第一行 = 图标 + 20px 大字，整块居中（与版面预览一致）----
    void toastLine1(uint8_t kind, const char *text) {
        int iw  = (kind == UI_TOAST_FAIL) ? 11 : (kind == UI_TOAST_AP ? 21 : 13);
        int gap = (kind == UI_TOAST_AP) ? 7 : 8;
        int x   = (250 - (iw + gap + textWidth(text, false))) / 2;
        if      (kind == UI_TOAST_FAIL) drawCross(x, 35);
        else if (kind == UI_TOAST_AP)   drawBcast(x + 10, 43);
        else                            drawTick(x, 35);
        drawText(x + iw + gap, 30, text, false);
    }
    // 水平居中绘制
    void drawTextC(int ly, const char *s, bool top) {
        drawText((250 - textWidth(s, top)) / 2, ly, s, top);
    }
    // √ 13x8
    void drawTick(int lx, int ly) {
        static const uint16_t r[8] = { 0x0006,0x000C,0x0018,0x0030,0x0860,0x0CC0,0x0780,0x0300 };
        for (int y = 0; y < 8; y++)
            for (int c = 0; c < 13; c++)
                if (r[y] & (1 << (12 - c))) px(lx + c, ly + y, true);
    }
    // × 11x9
    void drawCross(int lx, int ly) {
        static const uint16_t r[9] = { 0x0603,0x0306,0x018C,0x00D8,0x0070,0x00D8,0x018C,0x0306,0x0603 };
        for (int y = 0; y < 9; y++)
            for (int c = 0; c < 11; c++)
                if (r[y] & (1 << (10 - c))) px(lx + c, ly + y, true);
    }
    // 广播图标（AP）：中心点 + 双侧对称 90° 同心弧 x2（候选 B，“((·))”）
    //  19x13 固定点阵（与离线预览逐像素同源），位(18-c)；与顶栏“上方三道弧”的 WiFi 图标区分开
    void drawBcast(int cx, int cy) {                  // cx,cy = 圆点中心
        static const uint32_t r[13] = {
            0x0001800C, 0x00030006, 0x00022022, 0x00062023, 0x00046031, 0x00044711, 0x00044711,
            0x00044711, 0x00046031, 0x00062023, 0x00022022, 0x00030006, 0x0001800C };
        for (int y = 0; y < 13; y++)
            for (int c = 0; c < 19; c++)
                if (r[y] & (1u << (18 - c))) px(cx - 9 + c, cy - 6 + y, true);
    }

    // ---- 电池图标 (17x11) ----
    static int battLevel(uint8_t pct) {
        if (pct == 0)   return 0;
        if (pct >= 100) return 4;
        return pct / 25 + 1;
    }
    void drawBattery(int lx, int ly, int level) {
        if (level < 0) level = 0;
        if (level > 4) level = 4;
        for (int x = 0; x <= 14; x++) { px(lx + x, ly,      true); px(lx + x, ly + 10, true); }
        for (int y = 0; y <= 10; y++) { px(lx,     ly + y,  true); px(lx + 14, ly + y,  true); }
        for (int y = 3; y <= 7;  y++) { px(lx + 15, ly + y, true); px(lx + 16, ly + y, true); }
        static const int bars[4] = { 2, 5, 8, 11 };
        for (int i = 0; i < level; i++)
            for (int dx = 0; dx < 2; dx++)
                for (int y = 2; y <= 8; y++) px(lx + bars[i] + dx, ly + y, true);
    }

    // ---- 告警叹号 (2x11)：INA230 离线时占电池图标位 ----
    void drawBang(int lx, int ly) {
        for (int y = 0; y < 8; y++) { px(lx, ly + y, true); px(lx + 1, ly + y, true); }
        px(lx, ly + 10, true); px(lx + 1, ly + 10, true);
    }

    // ---- 闪电 (8x12) ----
    //   注意：告警态下 ⚡ 与 "INA!" 共用右侧区，两者坐标需避让（见 drawTop）。
    void drawBolt(int lx, int ly) {
        static const uint8_t b[12] = { 0xE0,0x70,0x38,0x1C,0xFE,0xFE,0x70,0x38,0x1C,0x0E,0x07,0x03 };
        for (int y = 0; y < 12; y++)
            for (int x = 0; x < 8; x++)
                if ((b[y] >> x) & 1U) px(lx + x, ly + y, true);
    }

    // ---- WiFi 图标 ----
    void drawWifi(int lx, int ly, uint8_t which) {
        if (which > 2) which = UI_WIFI_OFF;
        const uint32_t *rows = uiWifiIcons[which];
        for (int y = 0; y < 10; y++)
            for (int x = 0; x < 17; x++)
                if ((rows[y] >> x) & 1U) px(lx + x, ly + y, true);
    }

    // ---- 顶栏 ----
    void drawTop(time_t t) {
        char b[20];

        // 时间 HH:MM  (x=6)
        if (t >= 1000000000) {
            struct tm tm; localtime_r(&t, &tm);
            snprintf(b, sizeof(b), "%02d:%02d", tm.tm_hour, tm.tm_min);
        } else {
            strcpy(b, "--:--");                 // 时间未同步
        }
        drawText(6, 1, b, true);

        // 模式 MOV / FIX  (x=70)
        drawText(70, 1, onSite ? "FIX" : "MOV", true);

        // 充电中 -> ⚡ + CC/CV ；未充电 -> 电量百分比（统一右对齐到 x=198）
        if (!inaValid) {
            // INA230 离线：电量未知 → 整块改告警（充电中仍显示 ⚡）
            // 避让："INA!" 右对齐到 x=198、宽 36 → 墨迹约 163..195（'I' 从 163 起）。
            //      故告警态 ⚡ 左移到 152（占 152..159，留 4px 间隙）；
            //      正常充电态仍为 166（与 CC/CV 的 176 对齐）。
            if (charging) drawBolt(152, 3);
            drawText(198 - textWidth("INA!", true), 1, "INA!", true);
            drawBang(213, 3);
        } else if (charging) {
            drawBolt(166, 3);
            drawText(176, 1, cvPhase ? "CV" : "CC", true);
            drawBattery(209, 4, battLevel(battPct));
        } else {
            int n = snprintf(b, sizeof(b), "%u%%", (unsigned)battPct);
            drawText(198 - textWidth(b, true), 1, b, true);
            drawBattery(209, 4, battLevel(battPct));
        }

        // v2.2：电池直供标识（仅直供时出现）
        if (pr1Direct) drawPr1Mark(95, 3);

        // WiFi
            if (wifiStat == UI_WIFI_BLE) { int bw = textWidth("bt", true); drawText(246 - bw, 2, "bt", true); }   // 蓝牙：小写 bt 更窄更小，右对齐留余量   // 蓝牙：右对齐到右缘 248、上移   // 蓝牙：直接写文字，不画点阵
    else                        drawWifi(231, 5, wifiStat);
    }

    // ---- 变率箭头（三角头 + 杆）----
    //   cx,cy = 箭头中心（逻辑坐标）；dir: 1=↑ 2=↓ 3=→
    //   ↑↓ 占 11(宽)x18(高)，→ 占 14(宽)x11(高)
    void drawArrow(int cx, int cy, int dir) {
        if (dir == 3) {                                  // →  横杆 + 右三角头
            for (int x = -8; x <= -1; x++)
                for (int dy = -1; dy <= 1; dy++) px(cx + x, cy + dy, true);
            for (int c = 0; c <= 5; c++) {
                int half = 5 - c;
                for (int dy = -half; dy <= half; dy++) px(cx + c, cy + dy, true);
            }
        } else {
            bool up = (dir == 1);
            for (int i = 0; i <= 5; i++) {                // 三角头
                int y = up ? (cy - 9 + i) : (cy + 8 - i);
                for (int x = -i; x <= i; x++) px(cx + x, y, true);
            }
            for (int i = -3; i <= 8; i++) {               // 竖杆
                int y = up ? (cy + i) : (cy - i);
                for (int x = -1; x <= 1; x++) px(cx + x, y, true);
            }
        }
    }

    // ---- 24h 变率：右对齐绘制（箭头 + 绝对值数字，省略单位）----
    //   变率 NaN 时整块不绘制（留白）；单位跟随数值行，故此处不再重复。
    void drawTrend(float rate, int rowY, float th = UI_TREND_TH) {
        if (isnan(rate)) return;
        int dir;
        float a = fabsf(rate);
        if      (rate >=  th) dir = 1;                   // ↑
        else if (rate <= -th) dir = 2;                   // ↓
        else                           dir = 3;          // →（平稳）
        char b[12];
        snprintf(b, sizeof(b), "%.1f", a);
        int aw    = (dir == 3) ? 14 : 11;                // 箭头宽度
        int total = aw + 3 + textWidth(b, false);
        int x0    = UI_TREND_RIGHT - total;
        int cx    = x0 + (dir == 3 ? 8 : 5);
        int cy    = rowY + 10;
        drawArrow(cx, cy, dir);
        drawText(x0 + aw + 3, rowY, b, false);
    }

    // ---- 温度/湿度/气压数值（带单位，一位小数）+ 右侧 24h 变率 ----
    void drawClimate(float temp, float hum, float pres) {
        char b[24];

        // 温度 26.5°C
        if (isnan(temp)) {
            strcpy(b, "--.-" "\xB0" "C");
        } else {
            int t10 = (int)lroundf(temp * 10.0f);
            int a10 = (t10 < 0) ? -t10 : t10;
            snprintf(b, sizeof(b), "%s%d.%d" "\xB0" "C", (t10 < 0 ? "-" : ""), a10 / 10, a10 % 10);
        }
        drawText(64, 24, b, false);
        drawTrend(trendT, 24);

        // 湿度 58.0%
        if (isnan(hum)) strcpy(b, "--%");
        else            snprintf(b, sizeof(b), "%.1f%%", hum);
        drawText(64, 56, b, false);
        drawTrend(trendH, 56, UI_TREND_TH_AH);   // 湿度用绝对湿度阈值

        // 气压 1013.5hPa
        if (isnan(pres)) strcpy(b, "----hPa");
        else             snprintf(b, sizeof(b), "%.1fhPa", pres);
        drawText(64, 88, b, false);
        drawTrend(trendP, 88);
    }
};
