#pragma once
// ============================================================================
//  ST7305 · 2.13" LCD (122x250) · SPI 驱动 —— Arduino / ESP32 移植版
// ----------------------------------------------------------------------------
//  基于官方 ESP32-S3 (ESP-IDF) 示例移植而来：
//    versions/YDP213H001-V3/examples/esp32s3-...-st7305-bringup/main/st7305
//  显存布局、寄存器初始化序列、命令时序与原版完全一致，仅把底层传输改为
//  Arduino 的 SPI 库 + 软件 GPIO，以便在 Arduino 环境中编译烧录。
//
//  接线（用户定义）：
//    RES  -> GPIO13
//    SCK  -> GPIO12
//    SDI  -> GPIO14
//    DC   -> GPIO27
//    CS   -> GPIO26
//
//  注意：显示内容为 1-bit 黑/白，显存 4125 字节，布局 4x2 像素合一字节。
// ============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <algorithm>
#include <stdint.h>
#include <string.h>

// ---- 引脚定义（v2.2 板：LCD 整体迁移到 IO40/41/42/1/2）----
#define LCD_RST_PIN  42   // RES  -> IO42
#define LCD_SCK_PIN  1    // SCK  -> IO1
#define LCD_MOSI_PIN 2    // SDI  -> IO2
#define LCD_DC_PIN   41   // DC   -> IO41
#define LCD_CS_PIN   40   // CS   -> IO40

#define LCD_SPI_FREQ 40000000UL   // 40 MHz

class ST7305_2p13 {
public:
    ST7305_2p13() {}

    void begin() {
        // 初始化控制引脚
        pinMode(LCD_RST_PIN, OUTPUT);
        pinMode(LCD_DC_PIN, OUTPUT);
        pinMode(LCD_CS_PIN, OUTPUT);
        digitalWrite(LCD_CS_PIN, HIGH);
        digitalWrite(LCD_DC_PIN, HIGH);

        // 初始化 SPI（ESP32 允许自定义 SCK/MOSI 引脚）
        SPI.begin(LCD_SCK_PIN, -1, LCD_MOSI_PIN, -1);
        SPI.setFrequency(LCD_SPI_FREQ);
        SPI.setDataMode(SPI_MODE0);
        SPI.setBitOrder(MSBFIRST);

        // 硬件复位
        digitalWrite(LCD_RST_PIN, HIGH);
        delay(10);
        digitalWrite(LCD_RST_PIN, LOW);
        delay(10);
        digitalWrite(LCD_RST_PIN, HIGH);
        delay(10);

        initial_st7305();
        fill(0x00);

        high_power_mode();
        display_on(true);
        display_inversion(false);
    }

    // ---- 显存操作 ----
    void fill(uint8_t data) {
        memset(_buffer, data, DISPLAY_BUFFER_LENGTH);
    }

    void clearDisplay() {
        memset(_buffer, 0x00, DISPLAY_BUFFER_LENGTH);
    }

    void writePoint(uint16_t x, uint16_t y, bool enabled) {
        if (!_buffer || x >= LCD_WIDTH || y >= LCD_HIGH) return;
        x += COL_OFFSET;
        uint16_t real_x = x / 4;
        uint16_t real_y = y / 2;
        uint32_t idx = (uint32_t)real_y * LCD_DATA_WIDTH + real_x;
        uint8_t one_two = (y % 2 == 0) ? 0 : 1;
        uint8_t line_bit_4 = x % 4;
        uint8_t bit = 7 - (line_bit_4 * 2 + one_two);
        if (enabled)      _buffer[idx] |=  (1 << bit);
        else              _buffer[idx] &= ~(1 << bit);
    }

    // 读回一个点（与 writePoint 同一映射），供 /ui-dump 屏幕自检
    bool readPoint(uint16_t x, uint16_t y) const {
        if (x >= LCD_WIDTH || y >= LCD_HIGH) return false;
        x += COL_OFFSET;
        uint16_t real_x = x / 4;
        uint16_t real_y = y / 2;
        uint32_t idx = (uint32_t)real_y * LCD_DATA_WIDTH + real_x;
        uint8_t bit = 7 - ((x % 4) * 2 + (y % 2));
        return (_buffer[idx] >> bit) & 1;
    }

    void writePoint(uint16_t x, uint16_t y, uint16_t data) {
        writePoint(x, y, data != 0);
    }

    void loadBuffer(const uint8_t *src, size_t len) {
        if (!src || len != DISPLAY_BUFFER_LENGTH) return;
        memcpy(_buffer, src, DISPLAY_BUFFER_LENGTH);
    }

    void DisplayImageAt(const uint8_t *src, int imgW, int imgH, int x, int y) {
        int src_stride_blocks = imgW / 4;
        int dst_block_x = (x + COL_OFFSET) / 4;
        int dst_pair_y = y / 2;
        int copy_blocks = std::min(src_stride_blocks, LCD_DATA_WIDTH - dst_block_x);
        int copy_pairs  = std::min(imgH / 2, (LCD_HIGH / 2) - dst_pair_y);
        if (copy_blocks <= 0 || copy_pairs <= 0) return;
        const uint8_t *src_row = src;
        for (int rp = 0; rp < copy_pairs; ++rp) {
            uint32_t dst_idx = (uint32_t)(dst_pair_y + rp) * LCD_DATA_WIDTH + dst_block_x;
            memcpy(&_buffer[dst_idx], src_row, (size_t)copy_blocks);
            src_row += src_stride_blocks;
        }
    }

    void FillRectAt(int imgW, int imgH, int x, int y, bool is_black) {
        int dst_block_x = (x + COL_OFFSET) / 4;
        int dst_pair_y = y / 2;
        int want_blocks = imgW / 4;
        int want_pairs  = imgH / 2;
        int fill_blocks = std::min(want_blocks, LCD_DATA_WIDTH - dst_block_x);
        int fill_pairs  = std::min(want_pairs, (LCD_HIGH / 2) - dst_pair_y);
        if (fill_blocks <= 0 || fill_pairs <= 0) return;
        uint8_t byte_val = is_black ? 0xFF : 0x00;
        for (int rp = 0; rp < fill_pairs; ++rp) {
            uint32_t dst_idx = (uint32_t)(dst_pair_y + rp) * LCD_DATA_WIDTH + dst_block_x;
            memset(&_buffer[dst_idx], byte_val, (size_t)fill_blocks);
        }
    }

    // ---- 送显 ----
    //  ⚠️ 必须用 writeBytes()（只写）。SPI.transfer(buf,len) 是「同一块 buffer 既发又收」，
    //  而 ST7305 只写、无 MISO，悬空回读会把 _buffer 整块覆盖（实测成 0xFF）。
    //  旧写法后果：① 显存副本失效（/ui-dump 读回全黑）② showTop() 只刷顶栏时把数据区
    //  的垃圾一起送显 → 数据区/中文标签瞬变黑。改用只写接口后两者均消除。
    void display() {
        address();
        digitalWrite(LCD_DC_PIN, HIGH);
        digitalWrite(LCD_CS_PIN, LOW);
        SPI.writeBytes(_buffer, DISPLAY_BUFFER_LENGTH);
        digitalWrite(LCD_CS_PIN, HIGH);
    }

    // ---- 工作模式 ----
    void low_power_mode() {
        _hpm = false; _lpm = true;
        write_cmd(0xC1); write_param(0x3C); write_param(0x3E); write_param(0x3C); write_param(0x3C);
        write_cmd(0xC2); write_param(0x23); write_param(0x21); write_param(0x23); write_param(0x23);
        write_cmd(0xC4); write_param(0x5A); write_param(0x5C); write_param(0x5A); write_param(0x5A);
        write_cmd(0xC5); write_param(0x37); write_param(0x35); write_param(0x37); write_param(0x37);
        write_cmd(0xC9); write_param(0x00);
        delay(20);
        write_cmd(0x39);
        delay(100);
    }

    void high_power_mode() {
        _hpm = true; _lpm = false;
        write_cmd(0x38);
        delay(300);
        write_cmd(0xC1); write_param(0x3C); write_param(0x3E); write_param(0x3C); write_param(0x3C);
        write_cmd(0xC2); write_param(0x23); write_param(0x21); write_param(0x23); write_param(0x23);
        write_cmd(0xC4); write_param(0x5A); write_param(0x5C); write_param(0x5A); write_param(0x5A);
        write_cmd(0xC5); write_param(0x37); write_param(0x35); write_param(0x37); write_param(0x37);
        write_cmd(0xC9); write_param(0x00);
        delay(20);
    }

    void display_on(bool enabled) {
        write_cmd(enabled ? 0x29 : 0x28);
    }

    void display_inversion(bool enabled) {
        write_cmd(enabled ? 0x21 : 0x20);
    }

    int width()  { return LCD_WIDTH; }
    int height() { return LCD_HIGH; }

private:
    static const int COL_OFFSET = 10;
    static const int LCD_WIDTH = 122;
    static const int LCD_HIGH  = 250;
    static const int LCD_DATA_WIDTH = 33;
    static const int LCD_DATA_HIGH  = 125;
    static const int DISPLAY_BUFFER_LENGTH = 4125;

    bool _hpm = true;
    bool _lpm = false;
    uint8_t _buffer[DISPLAY_BUFFER_LENGTH];

    void write_cmd(uint8_t cmd) {
        digitalWrite(LCD_DC_PIN, LOW);
        digitalWrite(LCD_CS_PIN, LOW);
        SPI.transfer(cmd);
        digitalWrite(LCD_CS_PIN, HIGH);
    }

    void write_param(uint8_t p) {
        digitalWrite(LCD_DC_PIN, HIGH);
        digitalWrite(LCD_CS_PIN, LOW);
        SPI.transfer(p);
        digitalWrite(LCD_CS_PIN, HIGH);
    }

    void write_data(const uint8_t *data, size_t len) {
        digitalWrite(LCD_DC_PIN, HIGH);
        digitalWrite(LCD_CS_PIN, LOW);
        for (size_t i = 0; i < len; ++i) SPI.transfer(data[i]);
        digitalWrite(LCD_CS_PIN, HIGH);
    }

    void address() {
        write_cmd(0x2A); write_param(0x19); write_param(0x23);
        write_cmd(0x2B); write_param(0x00); write_param(0x7C);
        write_cmd(0x2C);
    }

    void initial_st7305() {
        // ---- 2.13" 专用初始化序列（与原版逐条一致）----
        write_cmd(0xD6); write_param(0x17); write_param(0x02);
        write_cmd(0xD1); write_param(0x01);

        write_cmd(0xC0); write_param(0x0E); write_param(0x05);
        write_cmd(0xC1); write_param(0x3C); write_param(0x3E); write_param(0x3C); write_param(0x3C);
        write_cmd(0xC2); write_param(0x23); write_param(0x21); write_param(0x23); write_param(0x23);
        write_cmd(0xC4); write_param(0x5A); write_param(0x5C); write_param(0x5A); write_param(0x5A);
        write_cmd(0xC5); write_param(0x37); write_param(0x35); write_param(0x37); write_param(0x37);

        write_cmd(0xD8); write_param(0xA6); write_param(0xE9);
        write_cmd(0xB2); write_param(0x15);

        write_cmd(0xB3);
        const uint8_t b3[] = {0xE5,0xF6,0x17,0x77,0x77,0x77,0x77,0x77,0x77,0x71};
        write_data(b3, sizeof(b3));
        write_cmd(0xB4);
        const uint8_t b4[] = {0x05,0x46,0x77,0x77,0x77,0x77,0x76,0x45};
        write_data(b4, sizeof(b4));
        write_cmd(0x62);
        const uint8_t gtim[] = {0x32,0x03,0x1F};
        write_data(gtim, sizeof(gtim));
        write_cmd(0xB7); write_param(0x13);
        write_cmd(0xB0); write_param(0x3F);

        write_cmd(0x11);
        delay(120);

        write_cmd(0xC9); write_param(0x00);
        write_cmd(0x36); write_param(0x48);
        write_cmd(0x3A); write_param(0x11);
        write_cmd(0xB9); write_param(0x20);
        write_cmd(0xB8); write_param(0x29);

        write_cmd(0x2A); write_param(0x19); write_param(0x23);
        write_cmd(0x2B); write_param(0x00); write_param(0x7C);

        write_cmd(0x35); write_param(0x00);
        write_cmd(0xD0); write_param(0xFF);
        write_cmd(0x38);

        _hpm = true; _lpm = false;

        write_cmd(0x29);
        write_cmd(0x20);
        write_cmd(0xBB); write_param(0x4F);
    }
};
