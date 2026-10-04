/*
 * ===== SD 探针 v4：手动跑完整 SD 初始化协议 =====
 *
 * 已知：CMD0 → 0x01（卡响应 idle），但 SD.begin() 总失败。
 * 本版逐条命令走一遍，定位到底哪一步出错：
 *   CMD0   → 复位进 idle
 *   CMD8   → 查是否 SDv2（电压/版本协商），响应 R7
 *   CMD58  → 读 OCR（电压与 CCS 位）
 *   CMD55+ACMD41 → 初始化并等卡就绪（SDv2 带 HCS 位）
 *   CMD58  → 再读 OCR，看 CCS（1=SDHC 块寻址）
 *   CMD9   → 读 CSD（容量）
 *   CMD17  → 读一个扇区（真正验证数据通路）
 *
 * 输出：COM6 @115200
 */
#include <SPI.h>

static SPIClass tSPI(HSPI);
static const int CS = 7;
static const int PIN_SCK = 5, PIN_MISO = 4, PIN_MOSI = 6;

// 收 R1 响应
static uint8_t r1() {
  uint8_t r = 0xFF;
  for (int i = 0; i < 10; i++) { r = tSPI.transfer(0xFF); if (!(r & 0x80)) break; }
  return r;
}

// 发一条命令（无数据阶段）
static uint8_t cmd(uint8_t c, uint32_t arg, uint8_t crc) {
  digitalWrite(CS, LOW);
  tSPI.transfer(0x40 | c);
  tSPI.transfer((arg >> 24) & 0xFF);
  tSPI.transfer((arg >> 16) & 0xFF);
  tSPI.transfer((arg >> 8) & 0xFF);
  tSPI.transfer(arg & 0xFF);
  tSPI.transfer(crc);
  uint8_t r = r1();
  digitalWrite(CS, HIGH);
  tSPI.transfer(0xFF);
  return r;
}

// 收 R3/R7（R1 + 4 字节）
static void r7(uint8_t* out4) {
  for (int i = 0; i < 4; i++) out4[i] = tSPI.transfer(0xFF);
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("=====================================================");
  Serial.println(" SD 探针 v4：手动完整初始化协议");
  Serial.println("=====================================================");

  pinMode(CS, OUTPUT);
  digitalWrite(CS, HIGH);
  tSPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, CS);
  tSPI.setFrequency(400000);
  delay(50);

  // 上电序列：CS 高 + 80 时钟
  digitalWrite(CS, HIGH);
  for (int i = 0; i < 12; i++) tSPI.transfer(0xFF);
  delay(10);

  Serial.println("\n【逐步执行】");

  // ---------- CMD0 ----------
  uint8_t r = cmd(0, 0x00000000, 0x95);
  Serial.printf("  CMD0   (复位)      R1 = 0x%02X  %s\n", r,
                r == 0x01 ? "✅ idle" : (r == 0x00 ? "已在 ready" : "❌ 异常"));
  if (r != 0x01 && r != 0x00) { Serial.println("  CMD0 就失败 → 到此为止"); return; }

  // ---------- CMD8 ----------
  digitalWrite(CS, LOW);
  tSPI.transfer(0x48); tSPI.transfer(0x00); tSPI.transfer(0x00);
  tSPI.transfer(0x01); tSPI.transfer(0xAA); tSPI.transfer(0x87);
  r = r1();
  uint8_t r7b[4] = {0,0,0,0};
  bool r7ok = (r == 0x01);
  if (r7ok) r7(r7b);
  digitalWrite(CS, HIGH); tSPI.transfer(0xFF);
  Serial.printf("  CMD8   (查SDv2)    R1 = 0x%02X  %s", r,
                r == 0x01 ? "✅ 是 SDv2" : (r == 0x05 ? "是 SDv1/MMC（不支持CMD8）" : "❌"));
  if (r7ok) {
    uint32_t echo = ((uint32_t)r7b[0] << 24) | ((uint32_t)r7b[1] << 16) |
                    ((uint32_t)r7b[2] << 8) | r7b[3];
    Serial.printf("  R7 = 0x%08X  %s", echo,
                  (echo & 0xFFF) == 0x1AA ? "✅ voltage/echo 匹配" : "❌ echo 不符");
  }
  Serial.println();

  // ---------- CMD58 读 OCR ----------
  digitalWrite(CS, LOW);
  tSPI.transfer(0x7A); tSPI.transfer(0x00); tSPI.transfer(0x00);
  tSPI.transfer(0x00); tSPI.transfer(0x00); tSPI.transfer(0x01);
  r = r1();
  uint8_t ocr[4] = {0,0,0,0};
  if (r == 0x01 || r == 0x00) r7(ocr);
  digitalWrite(CS, HIGH); tSPI.transfer(0xFF);
  uint32_t ocrv = ((uint32_t)ocr[0] << 24) | ((uint32_t)ocr[1] << 16) |
                  ((uint32_t)ocr[2] << 8) | ocr[3];
  Serial.printf("  CMD58  (读OCR)     R1 = 0x%02X  OCR = 0x%08X  %s%s%s\n", r, ocrv,
                (ocrv & 0x00FF8000) ? "" : "⚠️电压位异常 ",
                (ocrv & 0x40000000) ? "CCS=1(块寻址) " : "CCS=0(字节寻址) ",
                (ocrv & 0x80000000) ? "✅上电完成" : "⚠️未上电完成");

  // ---------- CMD55 + ACMD41 循环 ----------
  Serial.println("  --- ACMD41 初始化循环（最多 30 次）---");
  int tries = 0;
  uint8_t last = 0xFF;
  for (; tries < 30; tries++) {
    // CMD55
    digitalWrite(CS, LOW);
    tSPI.transfer(0x77); tSPI.transfer(0x00); tSPI.transfer(0x00);
    tSPI.transfer(0x00); tSPI.transfer(0x00); tSPI.transfer(0x01);
    uint8_t r55 = r1();
    digitalWrite(CS, HIGH); tSPI.transfer(0xFF);
    if (r55 > 0x01) { Serial.printf("    第%2d次 CMD55 R1=0x%02X ❌\n", tries + 1, r55); break; }

    // ACMD41 (HCS=1)
    digitalWrite(CS, LOW);
    tSPI.transfer(0x69); tSPI.transfer(0x40); tSPI.transfer(0x00);
    tSPI.transfer(0x00); tSPI.transfer(0x00); tSPI.transfer(0x01);
    last = r1();
    digitalWrite(CS, HIGH); tSPI.transfer(0xFF);
    if (last == 0x00) break;
    delay(50);
  }
  Serial.printf("  ACMD41 结果: R1=0x%02X 经过 %d 次 %s\n", last, tries + 1,
                last == 0x00 ? "✅ 卡就绪" : "❌ 未就绪");

  if (last != 0x00) { Serial.println("  → 初始化卡在 ACMD41"); return; }

  // ---------- CMD58 再读 OCR（看 CCS）----------
  digitalWrite(CS, LOW);
  tSPI.transfer(0x7A); tSPI.transfer(0x00); tSPI.transfer(0x00);
  tSPI.transfer(0x00); tSPI.transfer(0x00); tSPI.transfer(0x01);
  r = r1(); r7(ocr);
  digitalWrite(CS, HIGH); tSPI.transfer(0xFF);
  ocrv = ((uint32_t)ocr[0] << 24) | ((uint32_t)ocr[1] << 16) |
         ((uint32_t)ocr[2] << 8) | ocr[3];
  Serial.printf("  CMD58 重读: R1=0x%02X OCR=0x%08X CCS=%d %s\n", r, ocrv,
                (ocrv >> 30) & 1, ((ocrv >> 30) & 1) ? "HIGH CAPACITY" : "standard");

  // ---------- CMD9 读 CSD ----------
  digitalWrite(CS, LOW);
  tSPI.transfer(0x49); tSPI.transfer(0x00); tSPI.transfer(0x00);
  tSPI.transfer(0x00); tSPI.transfer(0x00); tSPI.transfer(0x01);
  r = r1();
  bool csdok = false;
  if (r == 0x00) {
    for (int i = 0; i < 200 && tSPI.transfer(0xFF) != 0xFE; i++) {}   // 等数据令牌
    uint8_t csd[16];
    for (int i = 0; i < 16; i++) csd[i] = tSPI.transfer(0xFF);
    tSPI.transfer(0xFF); tSPI.transfer(0xFF);                          // 丢 CRC
    csdok = true;
    uint8_t csdVer = (csd[0] >> 6) & 0x03;
    Serial.printf("  CMD9   (读CSD)     ✅ ver=%d  CSD[0..3]=%02X %02X %02X %02X\n",
                  csdVer, csd[0], csd[1], csd[2], csd[3]);
    if (csdVer == 1) {
      uint32_t c_size = ((uint32_t)(csd[7] & 0x3F) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
      uint64_t mb = ((uint64_t)(c_size + 1) * 512ULL * 1024ULL) / (1024ULL * 1024ULL);
      Serial.printf("                  容量 ≈ %llu MB  (C_SIZE=%u)\n",
                    (unsigned long long)mb, (unsigned)c_size);
    }
  } else {
    Serial.printf("  CMD9   (读CSD)     R1=0x%02X ❌\n", r);
  }
  digitalWrite(CS, HIGH); tSPI.transfer(0xFF);

  // ---------- CMD17 读扇区 0 ----------
  bool rdok = false;
  if (csdok) {
    digitalWrite(CS, LOW);
    tSPI.transfer(0x51); tSPI.transfer(0x00); tSPI.transfer(0x00);
    tSPI.transfer(0x00); tSPI.transfer(0x00); tSPI.transfer(0x01);
    r = r1();
    if (r == 0x00) {
      for (int i = 0; i < 5000 && tSPI.transfer(0xFF) != 0xFE; i++) {}
      uint8_t sec[16];
      for (int i = 0; i < 16; i++) sec[i] = tSPI.transfer(0xFF);
      rdok = true;
      Serial.printf("  CMD17  (读扇区0)   ✅ 前16字节: ");
      for (int i = 0; i < 16; i++) Serial.printf("%02X ", sec[i]);
      Serial.println();
    } else {
      Serial.printf("  CMD17  (读扇区0)   R1=0x%02X ❌\n", r);
    }
    digitalWrite(CS, HIGH); tSPI.transfer(0xFF);
  }

  Serial.println("\n=====================================================");
  if (rdok) {
    Serial.println(" ★ 底层数据通路完全可用！问题在 SD 库/文件系统层");
  } else {
    Serial.println(" ★ 底层协议中断，看上面哪一步 ❌");
  }
  Serial.println(" 结束，每 8 秒报活");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 8000) { last = millis(); Serial.printf("  [活] %lus\n", millis() / 1000); }
}
