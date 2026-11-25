#include "max17048.h"

bool Max17048::begin(TwoWire &wire) {
  wire_ = &wire;
  present_ = false;
  powerOnReset_ = false;
  if (!probe() || !readReg(REG_VERSION, version_)) return false;
  if ((version_ & 0xFFF0) != 0x0010) return false; // VERSION = 0x001x: otro chip en 0x36

  uint16_t status = 0;
  if (readStatus(status) && (status & STATUS_RI)) {
    powerOnReset_ = true;
    writeReg(REG_STATUS, status & ~STATUS_RI); // modelo por defecto (ROM): solo se marca como configurado
  }
  present_ = true;
  return true;
}

bool Max17048::probe() {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(I2C_ADDR);
  return wire_->endTransmission() == 0;
}

bool Max17048::readReg(uint8_t reg, uint16_t &value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(I2C_ADDR, (uint8_t)2) != 2) return false;
  const uint8_t msb = wire_->read();
  const uint8_t lsb = wire_->read();
  value = ((uint16_t)msb << 8) | lsb;
  return true;
}

bool Max17048::writeReg(uint8_t reg, uint16_t value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(reg);
  wire_->write((uint8_t)(value >> 8));
  wire_->write((uint8_t)(value & 0xFF));
  return wire_->endTransmission() == 0;
}

bool Max17048::readVoltageMv(uint16_t &mv) {
  uint16_t raw = 0;
  if (!readReg(REG_VCELL, raw)) return false;
  mv = (uint16_t)(((uint32_t)raw * 5) / 64); // 78.125 uV/LSB = 5/64 mV
  return true;
}

bool Max17048::readSoc(float &percent) {
  uint16_t raw = 0;
  if (!readReg(REG_SOC, raw)) return false;
  percent = raw / 256.0f;
  return true;
}

bool Max17048::readCrate(float &percentPerHour) {
  uint16_t raw = 0;
  if (!readReg(REG_CRATE, raw)) return false;
  percentPerHour = (int16_t)raw * 0.208f;
  return true;
}

bool Max17048::readStatus(uint16_t &status) { return readReg(REG_STATUS, status); }

bool Max17048::clearAlerts() {
  uint16_t status = 0, config = 0;
  if (!readStatus(status) || !readReg(REG_CONFIG, config)) return false;
  return writeReg(REG_STATUS, status & ~(STATUS_ALERTS | STATUS_RI)) && writeReg(REG_CONFIG, config & ~CONFIG_ALRT);
}
