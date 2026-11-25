#include "bme280.h"

static constexpr uint8_t RESET_CMD = 0xB6;
static constexpr uint8_t STATUS_IM_UPDATE = 0x01; // copiando la calibración de la NVM
static constexpr uint8_t OSRS_X1 = 0x01;
static constexpr uint8_t MODE_NORMAL = 0x03;
static constexpr uint8_t TSB_10MS = 0x06;   // t_standby = 10 ms
static constexpr uint8_t FILTER_X4 = 0x02;  // coeficiente IIR 4 (solo T y P)

static constexpr uint8_t CTRL_HUM = OSRS_X1;
static constexpr uint8_t CTRL_MEAS = (OSRS_X1 << 5) | (OSRS_X1 << 2) | MODE_NORMAL;
static constexpr uint8_t CONFIG = (TSB_10MS << 5) | (FILTER_X4 << 2);

static uint16_t u16le(const uint8_t *b) { return (uint16_t)(b[0] | (b[1] << 8)); }
static int16_t s16le(const uint8_t *b) { return (int16_t)u16le(b); }

bool Bme280::begin(TwoWire &wire, uint8_t address) {
  wire_ = &wire;
  addr_ = address;
  present_ = false;
  uint8_t id = 0;
  if (!readReg(REG_CHIP_ID, id) || id != CHIP_ID_VALUE) return false;

  if (!writeReg(REG_RESET, RESET_CMD)) return false;
  delay(3); // arranque tras el reset: 2 ms máx.
  for (uint8_t i = 0; i < 10; i++) { // la calibración se copia de la NVM en ~1 ms
    uint8_t status = 0;
    if (!readReg(REG_STATUS, status)) return false;
    if ((status & STATUS_IM_UPDATE) == 0) break;
    delay(1);
  }
  if (!readCalibration()) return false;

  // ctrl_hum solo se aplica al escribir ctrl_meas; config se escribe en modo sleep (después del reset)
  if (!writeReg(REG_CTRL_HUM, CTRL_HUM)) return false;
  if (!writeReg(REG_CONFIG, CONFIG)) return false;
  if (!writeReg(REG_CTRL_MEAS, CTRL_MEAS)) return false;
  present_ = true;
  return true;
}

bool Bme280::readCalibration() {
  uint8_t c[24];
  if (!readRegs(REG_CALIB_TP, c, 24) || !readRegs(REG_CALIB_H1, &h1_, 1)) return false;
  t1_ = u16le(&c[0]);
  t2_ = s16le(&c[2]);
  t3_ = s16le(&c[4]);
  p1_ = u16le(&c[6]);
  p2_ = s16le(&c[8]);
  p3_ = s16le(&c[10]);
  p4_ = s16le(&c[12]);
  p5_ = s16le(&c[14]);
  p6_ = s16le(&c[16]);
  p7_ = s16le(&c[18]);
  p8_ = s16le(&c[20]);
  p9_ = s16le(&c[22]);

  uint8_t h[7];
  if (!readRegs(REG_CALIB_H2, h, sizeof(h))) return false;
  h2_ = s16le(&h[0]);
  h3_ = h[2];
  h4_ = (int16_t)((int8_t)h[3] * 16) | (int16_t)(h[4] & 0x0F); // 0xE4[7:0] / 0xE5[3:0], 12 bits con signo
  h5_ = (int16_t)((int8_t)h[5] * 16) | (int16_t)(h[4] >> 4);   // 0xE6[7:0] / 0xE5[7:4]
  h6_ = (int8_t)h[6];
  return t1_ != 0 && p1_ != 0; // calibración leída (una NVM en ceros dividiría por cero)
}

bool Bme280::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(addr_, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = wire_->read();
  return true;
}

bool Bme280::readReg(uint8_t reg, uint8_t &value) { return readRegs(reg, &value, 1); }

bool Bme280::writeReg(uint8_t reg, uint8_t value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool Bme280::read(Sample &sample) {
  uint8_t d[8];
  if (!present_ || !readRegs(REG_DATA, d, sizeof(d))) return false; // una lectura: datos de la misma medición
  const int32_t adcP = (int32_t)(((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4));
  const int32_t adcT = (int32_t)(((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4));
  const int32_t adcH = (int32_t)(((uint32_t)d[6] << 8) | d[7]);
  if (adcT == 0x80000 || adcP == 0x80000 || adcH == 0x8000) return false; // sin medición todavía

  // Temperatura (datasheet 4.2.3): resultado en 0.01 °C; t_fine se usa en P y H
  int32_t var1 = ((((adcT >> 3) - ((int32_t)t1_ << 1))) * ((int32_t)t2_)) >> 11;
  int32_t var2 = (((((adcT >> 4) - ((int32_t)t1_)) * ((adcT >> 4) - ((int32_t)t1_))) >> 12) * ((int32_t)t3_)) >> 14;
  const int32_t tFine = var1 + var2;
  sample.tempC = ((tFine * 5 + 128) >> 8) / 100.0f;

  // Presión: entero de 64 bits, resultado en Pa en Q24.8
  int64_t p1 = (int64_t)tFine - 128000;
  int64_t p2 = p1 * p1 * (int64_t)p6_;
  p2 = p2 + ((p1 * (int64_t)p5_) << 17);
  p2 = p2 + (((int64_t)p4_) << 35);
  p1 = ((p1 * p1 * (int64_t)p3_) >> 8) + ((p1 * (int64_t)p2_) << 12);
  p1 = (((((int64_t)1) << 47) + p1)) * ((int64_t)p1_) >> 33;
  if (p1 == 0) return false;
  int64_t p = 1048576 - adcP;
  p = (((p << 31) - p2) * 3125) / p1;
  p1 = (((int64_t)p9_) * (p >> 13) * (p >> 13)) >> 25;
  p2 = (((int64_t)p8_) * p) >> 19;
  p = ((p + p1 + p2) >> 8) + (((int64_t)p7_) << 4);
  sample.pressureHpa = (uint32_t)p / 25600.0f; // Q24.8 Pa -> hPa

  // Humedad: resultado en %HR en Q22.10
  int32_t h = tFine - ((int32_t)76800);
  h = (((((adcH << 14) - (((int32_t)h4_) << 20) - (((int32_t)h5_) * h)) + ((int32_t)16384)) >> 15) *
       (((((((h * ((int32_t)h6_)) >> 10) * (((h * ((int32_t)h3_)) >> 11) + ((int32_t)32768))) >> 10) +
          ((int32_t)2097152)) * ((int32_t)h2_) + 8192) >> 14));
  h = h - (((((h >> 15) * (h >> 15)) >> 7) * ((int32_t)h1_)) >> 4);
  h = h < 0 ? 0 : h;
  h = h > 419430400 ? 419430400 : h;
  sample.humidity = (uint32_t)(h >> 12) / 1024.0f;
  return true;
}

bool Bme280::sleep() { return writeReg(REG_CTRL_MEAS, CTRL_MEAS & ~MODE_NORMAL); }

bool Bme280::wake() { return writeReg(REG_CTRL_MEAS, CTRL_MEAS); }
