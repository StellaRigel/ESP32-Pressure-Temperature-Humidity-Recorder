/*
 * ===== 最小测试固件 v2：BQ24074 充电控制（含 EN1/EN2 三档）=====
 *
 * v1 的漏洞：**只驱动 CE 和 ISET，没驱动 EN1/EN2** ——
 *   而 EN1/EN2 决定输入限流档位，芯片内部下拉只有约 285kΩ（高阻），
 *   可能不足以稳定进入某个档位 → 充电行为不可预期。
 *
 * 本版：**同时驱动 CE + ISET + EN1 + EN2**，逐相位验证。
 *
 * 相位表（每相位 20 秒，便于万用表跟测）；CE 恒为低（允许充电）：
 *   1. EN1/EN2 保持高阻（**复现 v1 症状**，作为对照）
 *   2. EN=(0,0) USB100(100mA)
 *   3. EN=(1,0) USB500(500mA)
 *   4. EN=(0,1) ISET 档（ILIM 电阻设定）
 *   5. EN=(1,0) USB500      + ISET=高（快充）
 *   6. EN=(0,1) ISET 档      + ISET=高（快充）
 */

#include <Wire.h>
#include "ina230.h"

static const int PIN_CE   = 13;   // 低 = 允许充电
static const int PIN_ISET = 17;   // 高 = 快充，低 = 慢充
static const int PIN_EN1  = 11;   // BQ24074 EN1
static const int PIN_EN2  = 12;   // BQ24074 EN2
static const int PIN_PR1  = 18;   // TPS2117 PR1：恒保持低（LDO，安全）

INA230 ina;
static const uint8_t INA_ADDR  = 0x40;
static const float   INA_SHUNT = 0.010f;

struct Phase {
  const char* name;
  int  en1;        // -1 = 保持高阻（不驱动）
  int  en2;
  int  iset;
};

static const Phase PHASES[] = {
  { "EN1/EN2 高阻(对照,复现v1)  ISET=低", -1, -1, LOW  },
  { "EN=(0,0) USB100(100mA)     ISET=低",  0,  0, LOW  },
  { "EN=(1,0) USB500(500mA)     ISET=低",  1,  0, LOW  },
  { "EN=(0,1) ISET档(ILIM)      ISET=低",  0,  1, LOW  },
  { "EN=(1,0) USB500(500mA)     ISET=高",  1,  0, HIGH },
  { "EN=(0,1) ISET档(ILIM)      ISET=高",  0,  1, HIGH },
};
static const int NPHASES = sizeof(PHASES) / sizeof(PHASES[0]);
static const unsigned long PHASE_MS = 20000UL;

static int phase = 0;
static unsigned long phaseStart = 0;

void applyPhase() {
  const Phase& p = PHASES[phase];

  // CE 恒为低 = 允许充电（本测试只验证"能不能充"）
  pinMode(PIN_CE, OUTPUT);
  digitalWrite(PIN_CE, LOW);
  digitalWrite(PIN_CE, LOW);

  pinMode(PIN_ISET, OUTPUT);
  digitalWrite(PIN_ISET, p.iset);
  digitalWrite(PIN_ISET, p.iset);

  if (p.en1 < 0 || p.en2 < 0) {
    pinMode(PIN_EN1, INPUT);            // 高阻：靠芯片内部约 285kΩ 下拉
    pinMode(PIN_EN2, INPUT);
  } else {
    pinMode(PIN_EN1, OUTPUT);
    pinMode(PIN_EN2, OUTPUT);
    digitalWrite(PIN_EN1, p.en1);
    digitalWrite(PIN_EN2, p.en2);
    digitalWrite(PIN_EN1, p.en1);
    digitalWrite(PIN_EN2, p.en2);
  }

  phaseStart = millis();

  Serial.println();
  Serial.println("-----------------------------------------------------");
  Serial.printf(">>> 相位 %d/%d: %s\n", phase + 1, NPHASES, p.name);
  Serial.printf("    CE=0(允许)  ISET=%d  EN1=%s  EN2=%s\n",
                p.iset,
                p.en1 < 0 ? "高阻" : (p.en1 ? "1" : "0"),
                p.en2 < 0 ? "高阻" : (p.en2 ? "1" : "0"));
  Serial.printf("    回读 CE=%d ISET=%d EN1=%d EN2=%d\n",
                digitalRead(PIN_CE), digitalRead(PIN_ISET),
                digitalRead(PIN_EN1), digitalRead(PIN_EN2));
  Serial.println("    [20 秒] 可量：BQ24074 ISET脚 / BAT脚 / OUT脚 / CE脚");
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_PR1, OUTPUT);
  digitalWrite(PIN_PR1, LOW);

  Wire.begin(47, 48);
  delay(50);
  bool ok = ina.begin(&Wire, INA_ADDR, INA_SHUNT, 1.0f);
  uint16_t manu = 0, die = 0;
  bool idOk = ok ? ina.checkIdentity(&manu, &die) : false;

  Serial.println();
  Serial.println("=====================================================");
  Serial.println(" BQ24074 充电测试 v2（CE+ISET+EN1+EN2 全驱动）");
  Serial.println("=====================================================");
  Serial.printf("PR1=IO%d(恒低)  CE=IO%d  ISET=IO%d  EN1=IO%d  EN2=IO%d\n",
                PIN_PR1, PIN_CE, PIN_ISET, PIN_EN1, PIN_EN2);
  Serial.printf("INA230: %s MANU=0x%04X DIE=0x%04X (%s)\n",
                ok ? "OK" : "失败", manu, die, idOk ? "在位" : "无应答");
  Serial.println("-----------------------------------------------------");

  phase = 0;
  applyPhase();
}

void loop() {
  static unsigned long lastPrint = 0;
  unsigned long now = millis();

  if (now - lastPrint >= 2500) {
    lastPrint = now;
    Ina230Reading r;
    if (ina.read(r)) {
      Serial.printf("   t=%4.1fs  V=%.3fV  I=%+7.2fmA  P=%+7.2fmW\n",
                    (now - phaseStart) / 1000.0f,
                    r.busVolt_V, r.current_mA, r.power_mW);
    } else {
      Serial.println("   INA230 读取失败");
    }
  }

  if (now - phaseStart >= PHASE_MS) {
    phase = (phase + 1) % NPHASES;
    applyPhase();
  }
}
