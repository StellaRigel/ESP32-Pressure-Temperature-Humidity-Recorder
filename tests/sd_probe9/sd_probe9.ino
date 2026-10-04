/*
 * ===== SD 探针 v9：手动读 MBR/BPB，验证文件系统结构 =====
 *
 * 目的：确认卡上【确实有有效的 FAT32 文件系统】。
 *   若 MBR 的 0x55AA 签名与 BPB 字段都正常 → 文件系统没问题，
 *   纯粹是两个库的驱动问题；否则是卡的分区/格式化问题。
 *
 * 用纯手动 SPI（已验证可读扇区）读取：
 *   · 扇区 0 = MBR（含 4 个分区项 + 0x55AA 签名）
 *   · 按 MBR 找到第一个 FAT 分区，读它的 BPB（BIOS Parameter Block）
 *   · 打印关键字段：FAT 类型、每簇扇区数、保留扇区数、FAT 数、根目录簇
 *
 * 输出：COM6 @115200
 */
#include <SPI.h>

#define PIN_CS   7
#define PIN_SCK  5
#define PIN_MISO 4
#define PIN_MOSI 6

static SPIClass spi(HSPI);

static uint8_t r1() {
  uint8_t r = 0xFF;
  for (int i = 0; i < 10; i++) { r = spi.transfer(0xFF); if (!(r & 0x80)) break; }
  return r;
}
static uint8_t cmd(uint8_t c, uint32_t arg, uint8_t crc) {
  digitalWrite(PIN_CS, LOW);
  spi.transfer(0x40 | c);
  spi.transfer((arg >> 24) & 0xFF); spi.transfer((arg >> 16) & 0xFF);
  spi.transfer((arg >> 8) & 0xFF);  spi.transfer(arg & 0xFF);
  spi.transfer(crc);
  uint8_t r = r1();
  digitalWrite(PIN_CS, HIGH);
  spi.transfer(0xFF);
  delay(5);
  return r;
}
static bool manualInit() {
  digitalWrite(PIN_CS, HIGH);
  for (int i = 0; i < 16; i++) spi.transfer(0xFF);
  delay(60);
  if (cmd(0, 0, 0x95) != 0x01) return false;
  delay(20);
  cmd(8, 0x1AA, 0x87);
  delay(20);
  for (int i = 0; i < 40; i++) {
    digitalWrite(PIN_CS, LOW);
    spi.transfer(0x77); spi.transfer(0); spi.transfer(0);
    spi.transfer(0); spi.transfer(0); spi.transfer(0x01);
    uint8_t r55 = r1();
    digitalWrite(PIN_CS, HIGH); spi.transfer(0xFF);
    if (r55 > 0x01) return false;
    delay(10);
    digitalWrite(PIN_CS, LOW);
    spi.transfer(0x69); spi.transfer(0x40); spi.transfer(0);
    spi.transfer(0); spi.transfer(0); spi.transfer(0x01);
    uint8_t r41 = r1();
    digitalWrite(PIN_CS, HIGH); spi.transfer(0xFF);
    if (r41 == 0x00) { delay(30); return true; }
    delay(60);
  }
  return false;
}

// 读一个 512 字节扇区（CMD17）
static bool readSector(uint32_t lba, uint8_t* buf) {
  digitalWrite(PIN_CS, LOW);
  spi.transfer(0x51);
  spi.transfer((lba >> 24) & 0xFF); spi.transfer((lba >> 16) & 0xFF);
  spi.transfer((lba >> 8) & 0xFF);  spi.transfer(lba & 0xFF);
  spi.transfer(0x01);
  uint8_t r = 0xFF;
  for (int i = 0; i < 12; i++) { r = spi.transfer(0xFF); if (!(r & 0x80)) break; }
  if (r != 0x00) { digitalWrite(PIN_CS, HIGH); spi.transfer(0xFF); delay(5); return false; }
  uint8_t tok = 0xFF;
  for (int i = 0; i < 20000; i++) { tok = spi.transfer(0xFF); if (tok != 0xFF) break; }
  if (tok != 0xFE) { digitalWrite(PIN_CS, HIGH); spi.transfer(0xFF); delay(5); return false; }
  for (int i = 0; i < 512; i++) buf[i] = spi.transfer(0xFF);
  spi.transfer(0xFF); spi.transfer(0xFF);      // 丢 CRC
  digitalWrite(PIN_CS, HIGH);
  spi.transfer(0xFF);
  delay(5);
  return true;
}

static uint32_t u32le(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t u16le(const uint8_t* p) { return (uint16_t)p[0] | ((uint16_t)p[1] << 8); }

static uint8_t sec[512];

// 重试直到初始化成功（手动 init 偶发失败）
static bool initWithRetry(int tries) {
  for (int i = 1; i <= tries; i++) {
    spi.end();
    pinMode(PIN_CS, OUTPUT); digitalWrite(PIN_CS, HIGH);
    delay(600);
    spi.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);
    spi.setFrequency(400000);
    if (manualInit()) {
      Serial.printf("  初始化成功（第 %d 次尝试）\n", i);
      return true;
    }
    Serial.printf("  第 %d 次初始化失败，重试...\n", i);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("=====================================================");
  Serial.println(" SD 探针 v9：手动读 MBR/BPB");
  Serial.println("=====================================================");

  if (!initWithRetry(8)) { Serial.println("❌ 初始化始终失败"); return; }

  // ---------- 扇区 0：MBR ----------
  Serial.println("\n【扇区 0 = MBR】");
  if (!readSector(0, sec)) { Serial.println("  ❌ 读扇区 0 失败"); return; }

  Serial.print("  前 16 字节: ");
  for (int i = 0; i < 16; i++) Serial.printf("%02X ", sec[i]);
  Serial.println();

  bool sig = (sec[510] == 0x55 && sec[511] == 0xAA);
  Serial.printf("  引导签名 0x55AA: %s (bytes %02X %02X)\n", sig ? "✅ 有" : "❌ 无", sec[510], sec[511]);

  bool allZero = true;
  for (int i = 0; i < 512; i++) if (sec[i]) { allZero = false; break; }
  Serial.printf("  扇区是否全 0: %s\n", allZero ? "⚠️ 是（可能未格式化）" : "否");

  if (allZero) {
    Serial.println("\n  → 扇区 0 全 0，说明卡上没有分区表/未格式化。");
    Serial.println("     这能解释两个库为什么挂不上！");
    // 继续看看别的扇区有没有内容
    Serial.println("\n【抽查其他扇区（找 FAT/数据）】");
    for (uint32_t lba : {(uint32_t)1, (uint32_t)2, (uint32_t)63, (uint32_t)64, (uint32_t)2048, (uint32_t)8192}) {
      if (readSector(lba, sec)) {
        int nz = 0; for (int i = 0; i < 512; i++) if (sec[i]) nz++;
        Serial.printf("    扇区 %-6u 非零字节 %3d  前4字节: %02X %02X %02X %02X\n",
                      lba, nz, sec[0], sec[1], sec[2], sec[3]);
      } else {
        Serial.printf("    扇区 %-6u 读取失败\n", lba);
      }
    }
    return;
  }

  // ---------- 解析 MBR 分区表 ----------
  Serial.println("\n【MBR 分区表】");
  bool found = false;
  uint32_t partLba = 0;
  for (int i = 0; i < 4; i++) {
    const uint8_t* p = sec + 446 + i * 16;
    uint8_t type = p[4];
    uint32_t lba = u32le(p + 8);
    uint32_t cnt = u32le(p + 12);
    if (type == 0 && cnt == 0) continue;
    const char* tn = (type == 0x01) ? "FAT12" : (type == 0x04 || type == 0x06) ? "FAT16" :
                     (type == 0x0B) ? "FAT32(CHS)" : (type == 0x0C) ? "FAT32(LBA)" :
                     (type == 0x07) ? "NTFS/exFAT" : (type == 0xEE) ? "GPT保护" : "其他";
    Serial.printf("  分区 %d: type=0x%02X (%s)  起始LBA=%u  扇区数=%u (%.2f GB)\n",
                  i + 1, type, tn, lba, cnt, cnt * 512.0 / 1073741824.0);
    if (!found && (type == 0x0B || type == 0x0C || type == 0x06 || type == 0x04 || type == 0x01)) {
      partLba = lba; found = true;
    }
  }

  if (!found) { Serial.println("  ⚠️ 没找到 FAT 分区"); return; }

  // ---------- 读 BPB ----------
  Serial.printf("\n【第一个 FAT 分区的 BPB（LBA %u）】\n", partLba);
  if (!readSector(partLba, sec)) { Serial.println("  ❌ 读 BPB 失败"); return; }

  uint16_t bytesPerSec = u16le(sec + 11);
  uint8_t  secPerClus  = sec[13];
  uint16_t rsvdSec     = u16le(sec + 14);
  uint8_t  numFats     = sec[16];
  uint16_t rootEntries = u16le(sec + 17);
  uint16_t totSec16    = u16le(sec + 19);
  uint32_t totSec32    = u32le(sec + 32);
  uint32_t fatSize32   = u32le(sec + 36);
  uint32_t rootClus    = u32le(sec + 44);
  const char* fstype   = (const char*)(sec + 82);   // "FAT32   " 等

  Serial.printf("  每扇区字节数  = %u        %s\n", bytesPerSec, bytesPerSec == 512 ? "✅" : "⚠️");
  Serial.printf("  每簇扇区数    = %u\n", secPerClus);
  Serial.printf("  保留扇区数    = %u\n", rsvdSec);
  Serial.printf("  FAT 个数      = %u        %s\n", numFats, (numFats == 1 || numFats == 2) ? "✅" : "⚠️");
  Serial.printf("  根目录项数    = %u\n", rootEntries);
  Serial.printf("  总扇区16      = %u\n", totSec16);
  Serial.printf("  总扇区32      = %u (%.2f GB)\n", totSec32, totSec32 * 512.0 / 1073741824.0);
  Serial.printf("  FAT 大小32    = %u 扇区\n", fatSize32);
  Serial.printf("  根目录簇号    = %u\n", rootClus);
  Serial.printf("  卷标/FSType   = '%s'\n", fstype);

  // 计算数据区起点并读根目录首扇区
  uint32_t fatStart   = partLba + rsvdSec;
  uint32_t dataStart  = fatStart + (uint32_t)numFats * fatSize32;
  uint32_t rootLba    = dataStart + (rootClus - 2) * secPerClus;
  Serial.printf("\n  推算 FAT 起始 LBA   = %u\n", fatStart);
  Serial.printf("  推算数据区起始 LBA  = %u\n", dataStart);
  Serial.printf("  推算根目录 LBA      = %u\n", rootLba);

  Serial.println("\n【根目录首扇区（找 8.3 长文件名项）】");
  if (readSector(rootLba, sec)) {
    int entries = 0;
    for (int off = 0; off < 512 && entries < 10; off += 32) {
      uint8_t first = sec[off];
      if (first == 0x00) break;          // 结束
      if (first == 0xE5) continue;       // 已删
      if (sec[off + 11] & 0x0F) continue; // 长文件名项
      char nm[13];
      memcpy(nm, sec + off, 8); nm[8] = 0;
      for (int k = 7; k >= 0 && nm[k] == ' '; k--) nm[k] = 0;
      char ext[4]; memcpy(ext, sec + off + 8, 3); ext[3] = 0;
      for (int k = 2; k >= 0 && ext[k] == ' '; k--) ext[k] = 0;
      uint32_t sz = u32le(sec + off + 28);
      uint16_t clus = u16le(sec + off + 26);
      bool isDir = (sec[off + 11] & 0x10) != 0;
      Serial.printf("     %s%s%s  %u B  起始簇 %u%s\n", nm, ext[0] ? "." : "", ext,
                    sz, clus, isDir ? "  [目录]" : "");
      entries++;
    }
    if (entries == 0) Serial.println("     （根目录为空）");
  } else {
    Serial.println("     ❌ 读根目录失败");
  }

  Serial.println("\n=====================================================");
  Serial.println(" 结论：若上面 MBR/BPB 都正常 → 文件系统没问题，是库的驱动问题");
  Serial.println(" 结束，每 8 秒报活");
}

void loop() {
  static unsigned long last = 0;
  if (millis() - last >= 8000) { last = millis(); Serial.printf("  [活] %lus\n", millis() / 1000); }
}
