#include "fan53741.h"

static constexpr uint8_t MODE_ENABLE = 0x02;
static constexpr uint8_t VSEL_MIN = 0x0F; // 0.6 V
static constexpr uint8_t VSEL_MAX = 0x7B; // 3.3 V

bool Fan53741::begin(TwoWire &wire) {
  wire_ = &wire;
  present_ = readReg(REG_ID_REV, idRev_) && (idRev_ >> 4) == PRODUCT_ID;
  return present_;
}

bool Fan53741::readReg(uint8_t reg, uint8_t &value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(I2C_ADDR, (uint8_t)1) != 1) return false;
  value = wire_->read();
  return true;
}

bool Fan53741::writeReg(uint8_t reg, uint8_t value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool Fan53741::setVoutMv(uint16_t mv) {
  if (mv < 600 || mv > 3300) return false;
  const uint8_t code = (uint8_t)(VSEL_MIN + (mv - 600 + 12) / 25);
  return writeReg(REG_VSEL, code > VSEL_MAX ? VSEL_MAX : code);
}

bool Fan53741::readVoutMv(uint16_t &mv) {
  uint8_t code = 0;
  if (!readReg(REG_VSEL, code)) return false;
  code &= 0x7F;
  if (code < VSEL_MIN || code > VSEL_MAX) return false; // códigos reservados
  mv = (uint16_t)(600 + (code - VSEL_MIN) * 25);
  return true;
}

bool Fan53741::enable(bool on) { return writeReg(REG_MODE, on ? MODE_ENABLE : 0x00); }
