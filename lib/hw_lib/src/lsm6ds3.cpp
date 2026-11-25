#include "lsm6ds3.h"

static constexpr uint8_t CTRL3_SW_RESET = 0x01;
static constexpr uint8_t CTRL3_IF_INC = 0x04;
static constexpr uint8_t CTRL3_BDU = 0x40;
static constexpr uint8_t ODR_52HZ = 0x30;        // ODR_XL / ODR_G = 0011 en los bits 7:4 (52 Hz)
static constexpr uint8_t STATUS_XLDA = 0x01;
static constexpr uint8_t STATUS_GDA = 0x02;

static constexpr float ACCEL_G_PER_LSB = 0.061f / 1000.0f;  // ±2 g
static constexpr float GYRO_DPS_PER_LSB = 8.75f / 1000.0f;  // 250 dps

bool Lsm6ds3::begin(TwoWire &wire, uint8_t address) {
  wire_ = &wire;
  addr_ = address;
  present_ = false;
  uint8_t id = 0;
  if (!readReg(REG_WHO_AM_I, id) || id != WHO_AM_I_VALUE) return false;

  if (!writeReg(REG_CTRL3_C, CTRL3_SW_RESET)) return false;
  delay(1); // el reset por software tarda ~50 us
  if (!writeReg(REG_CTRL3_C, CTRL3_BDU | CTRL3_IF_INC)) return false;
  if (!writeReg(REG_CTRL1_XL, ODR_52HZ)) return false; // ±2 g
  if (!writeReg(REG_CTRL2_G, ODR_52HZ)) return false;  // 250 dps
  present_ = true;
  return true;
}

bool Lsm6ds3::readReg(uint8_t reg, uint8_t &value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(addr_, (uint8_t)1) != 1) return false;
  value = wire_->read();
  return true;
}

bool Lsm6ds3::writeReg(uint8_t reg, uint8_t value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool Lsm6ds3::dataReady() {
  uint8_t status = 0;
  return readReg(REG_STATUS, status) && (status & (STATUS_XLDA | STATUS_GDA)) == (STATUS_XLDA | STATUS_GDA);
}

bool Lsm6ds3::read(Sample &sample) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(REG_OUT_TEMP_L);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(addr_, (uint8_t)14) != 14) return false;
  int16_t raw[7];
  for (int16_t &v : raw) {
    const uint8_t lo = wire_->read();
    const uint8_t hi = wire_->read();
    v = (int16_t)((hi << 8) | lo);
  }
  sample.tempC = 25.0f + raw[0] / 256.0f;
  sample.gx = raw[1] * GYRO_DPS_PER_LSB;
  sample.gy = raw[2] * GYRO_DPS_PER_LSB;
  sample.gz = raw[3] * GYRO_DPS_PER_LSB;
  sample.ax = raw[4] * ACCEL_G_PER_LSB;
  sample.ay = raw[5] * ACCEL_G_PER_LSB;
  sample.az = raw[6] * ACCEL_G_PER_LSB;
  return true;
}

bool Lsm6ds3::powerDown() { return writeReg(REG_CTRL1_XL, 0x00) && writeReg(REG_CTRL2_G, 0x00); }
