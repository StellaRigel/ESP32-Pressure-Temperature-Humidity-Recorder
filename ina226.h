/**
 * ina226.h —— INA226 高边电压/电流监测驱动（PHT_2_X 电池监测）
 * ================================================================
 * 硬件（见 设计文档.md · V2.1 · 电池监测）：
 *   · INA226 挂 I2C 总线，地址 0x40（A0=A1=GND）
 *   · 高边 10mΩ 采样电阻跨接在 BQ24074@BAT 与电池正极之间
 *       VIN+ 靠 BQ 侧  /  VIN- 靠电池侧   → 充电时电流为正
 *   · VBUS 引脚接到 VIN-（电池侧）→ 电压通道读【电池真实对地电压】
 *
 * 为什么单独放头文件、而不是写进 .ino：
 *   Arduino IDE 会为 .ino 自动生成函数原型并插到文件最前面，
 *   任何"签名里用到自定义类型"的函数都会报 'xxx does not name a type'。
 *   放进 .h 后不参与原型注入，可以放心用 struct / class。
 * ================================================================
 */
#ifndef PHT_INA226_H
#define PHT_INA226_H

#include <Arduino.h>
#include <Wire.h>

// ---------------- 寄存器地址 ----------------
#define INA226_REG_CONFIG   0x00
#define INA226_REG_SHUNT_V  0x01
#define INA226_REG_BUS_V    0x02
#define INA226_REG_POWER    0x03
#define INA226_REG_CURRENT  0x04
#define INA226_REG_CAL      0x05
#define INA226_REG_MASK     0x06
#define INA226_REG_ALERT    0x07
#define INA226_REG_MANU_ID  0xFE   // 厂商 ID，TI = 0x5449 ("TI")
#define INA226_REG_DIE_ID   0xFF   // 器件 ID，INA226 = 0x2260

#define INA226_MANU_ID_TI   0x5449
#define INA226_DIE_ID_226   0x2260

// ---------------- 物理 LSB（满量程固定，与配置无关）----------------
//   分流电压：±81.92 mV，LSB = 2.5 µV
//   总线电压：0~40.96 V，LSB = 1.25 mV
#define INA226_SHUNT_LSB_V  0.0000025f
#define INA226_BUS_LSB_V    0.0012500f

// ---------------- 平均次数 / 转换时间 编码 ----------------
//   注意：INA226 的编码【不是线性的】，0212 之类是跳档的，别想当然
enum Ina226Avg {
  INA226_AVG_1 = 0, INA226_AVG_4 = 1, INA226_AVG_16 = 2, INA226_AVG_64 = 3,
  INA226_AVG_128 = 4, INA226_AVG_256 = 5, INA226_AVG_512 = 6, INA226_AVG_1024 = 7
};
enum Ina226ConvTime {
  INA226_CT_140US = 0, INA226_CT_204US = 1, INA226_CT_332US = 2, INA226_CT_588US = 3,
  INA226_CT_1100US = 4, INA226_CT_2116US = 5, INA226_CT_4156US = 6, INA226_CT_8244US = 7
};

// ---------------- 读数结构 ----------------
struct Ina226Reading {
  float shuntVolt_mV = 0;   // 分流电压（mV）
  float busVolt_V    = 0;   // 总线/电池电压（V）—— 本电路即电池真实电压
  float current_mA   = 0;   // 电流（mA，充电为正、放电为负）
  float power_mW     = 0;   // 功率（mW）
  uint16_t rawShunt = 0, rawBus = 0, rawCurrent = 0, rawPower = 0;
  bool ok = false;
};

// ================================================================
class INA226 {
public:
  INA226() {}

  /**
   * 初始化：探测 → 写 CONFIG/CAL → 回读校验
   * @param wire         I2C 总线（默认 &Wire）
   * @param addr         器件地址（默认 0x40，A0=A1=GND）
   * @param rShunt       采样电阻阻值（Ω，默认 0.010 = 10mΩ）
   * @param maxCurrent_A 预期最大电流（A），用于自动选 Current_LSB
   * @param avg          平均次数
   * @param busCt        总线电压转换时间
   * @param shuntCt      分流电压转换时间
   * @return true = 器件在线且配置写入成功
   */
  bool begin(TwoWire* wire, uint8_t addr, float rShunt = 0.010f, float maxCurrent_A = 1.0f,
             Ina226Avg avg = INA226_AVG_128,
             Ina226ConvTime busCt = INA226_CT_1100US, Ina226ConvTime shuntCt = INA226_CT_1100US) {
    _wire = wire; _addr = addr; _rShunt = rShunt;
    if (!checkIdentity()) { _ok = false; return false; }

    // ---- Current_LSB 取 maxCurrent/2^15，TI 推荐做法 ----
    _currentLsb = maxCurrent_A / 32768.0f;
    _cal = calcCalibration(_currentLsb, _rShunt);
    _config = buildConfig(avg, busCt, shuntCt, true);

    if (!writeReg(INA226_REG_CONFIG, _config)) { _ok = false; return false; }
    if (!writeReg(INA226_REG_CAL, _cal))       { _ok = false; return false; }

    // ---- 回读校验 ----
    uint16_t rbCfg = 0, rbCal = 0;
    readReg(INA226_REG_CONFIG, rbCfg);
    readReg(INA226_REG_CAL, rbCal);
    _ok = (rbCfg == _config && rbCal == _cal);
    return _ok;
  }
  bool begin(uint8_t addr = 0x40, float rShunt = 0.010f, float maxCurrent_A = 1.0f) {
    return begin(&Wire, addr, rShunt, maxCurrent_A);
  }

  // ---- 单次读取（连续转换模式下读的是最近一次结果，不阻塞）----
  bool read(Ina226Reading& r) {
    r = Ina226Reading();
    if (!_ok) return false;
    uint16_t raw;
    if (!readReg(INA226_REG_SHUNT_V, raw)) return false;   r.rawShunt   = raw;
    if (!readReg(INA226_REG_BUS_V,   raw)) return false;   r.rawBus     = raw;
    if (!readReg(INA226_REG_CURRENT, raw)) return false;   r.rawCurrent = raw;
    if (!readReg(INA226_REG_POWER,   raw)) return false;   r.rawPower   = raw;

    int16_t sShunt = (int16_t)r.rawShunt;
    int16_t sCur   = (int16_t)r.rawCurrent;

    float shuntV = sShunt * INA226_SHUNT_LSB_V;
    float currA  = sCur   * _currentLsb;
    if (_invert) { shuntV = -shuntV; currA = -currA; }

    r.shuntVolt_mV = shuntV * 1000.0f;
    r.busVolt_V    = r.rawBus * INA226_BUS_LSB_V;             // 无符号
    r.current_mA   = currA * 1000.0f;
    r.power_mW     = r.rawPower * (_currentLsb * 25.0f) * 1000.0f;  // Power_LSB = 25 × Current_LSB
    r.ok = true;
    return true;
  }

  bool readBusVoltage(float& v) { Ina226Reading r; if (!read(r)) return false; v = r.busVolt_V; return true; }
  bool readCurrent(float& mA)   { Ina226Reading r; if (!read(r)) return false; mA = r.current_mA; return true; }

  // ---- 状态查询 ----
  bool    ok()           const { return _ok; }
  bool    connected()    const { return _ok; }
  uint8_t address()      const { return _addr; }
  uint16_t calibration() const { return _cal; }
  uint16_t configWord()  const { return _config; }
  float   currentLsb()   const { return _currentLsb; }
  float   rShunt()       const { return _rShunt; }
  // 实际有效分辨率 = 分流电压 LSB ÷ 采样电阻（物理下限，与 Current_LSB 无关）
  float   currentResolution_mA() const { return (INA226_SHUNT_LSB_V / _rShunt) * 1000.0f; }
  // 可测电流上限 = 81.92mV ÷ 采样电阻
  float   currentMax_A() const { return 0.08192f / _rShunt; }

  void setInvert(bool inv) { _invert = inv; }
  bool getInvert() const { return _invert; }

  // ---- 身份/诊断 ----
  bool checkIdentity(uint16_t* manuOut = nullptr, uint16_t* dieOut = nullptr) {
    uint16_t manu = 0, die = 0;
    if (!readReg(INA226_REG_MANU_ID, manu)) return false;
    if (!readReg(INA226_REG_DIE_ID,  die))  return false;
    if (manuOut) *manuOut = manu;
    if (dieOut)  *dieOut  = die;
    return (manu == INA226_MANU_ID_TI && die == INA226_DIE_ID_226);
  }

  // ---- 原始寄存器读写（排障用）----
  bool writeReg(uint8_t reg, uint16_t val) {
    if (!_wire) return false;
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write((uint8_t)(val >> 8));
    _wire->write((uint8_t)(val & 0xFF));
    return (_wire->endTransmission() == 0);
  }
  bool readReg(uint8_t reg, uint16_t& val) {
    if (!_wire) return false;
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    if (_wire->endTransmission(false) != 0) return false;
    if (_wire->requestFrom((int)_addr, 2) != 2) return false;
    uint8_t hi = _wire->read();
    uint8_t lo = _wire->read();
    val = ((uint16_t)hi << 8) | lo;
    return true;
  }

  // ---- 组配置字（公开，便于排障打印）----
  //   配置寄存器位域：
  //     bit15    RST
  //     bit14:12 AVG     平均次数
  //     bit11:9  VBUSCT  总线电压转换时间
  //     bit8:6   VSHCT   分流电压转换时间
  //     bit5:3   保留（复位默认 100，勿动）
  //     bit2:0   MODE    111 = 分流+总线 连续测量
  //   注：MODE 在 bit2:0 而非 bit5:3 —— 由「连续读数正常刷新」反推确认，
  //       若 MODE 在 bit5:3，0x4927 会落到 power-down，读数根本不会更新。
  static uint16_t buildConfig(Ina226Avg avg, Ina226ConvTime busCt, Ina226ConvTime shuntCt, bool continuous) {
    return ((uint16_t)avg << 12) | ((uint16_t)busCt << 9) | ((uint16_t)shuntCt << 6) |
           (uint16_t)(0b100 << 3) | (continuous ? 0b111 : 0b000);
  }
  static uint16_t calcCalibration(float currentLsb, float rShunt) {
    if (currentLsb <= 0 || rShunt <= 0) return 0;
    float cal = 0.00512f / (currentLsb * rShunt);
    if (cal > 65535.0f) cal = 65535.0f;
    if (cal < 1.0f)     cal = 1.0f;
    return (uint16_t)(cal + 0.5f);
  }

private:
  TwoWire* _wire = nullptr;
  uint8_t  _addr = 0x40;
  float    _rShunt = 0.010f;
  float    _currentLsb = 0.0001f;
  uint16_t _cal = 0;
  uint16_t _config = 0;
  bool     _ok = false;
  bool     _invert = false;
};

#endif  // PHT_INA226_H
