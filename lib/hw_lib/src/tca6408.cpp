#include "tca6408.h"

bool Tca6408::begin(TwoWire &wire, uint8_t address) {
  wire_ = &wire;
  addr_ = address;
  present_ = probe() && readReg(REG_OUTPUT, out_) && readReg(REG_CONFIG, cfg_);
  return present_;
}

bool Tca6408::probe() {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  return wire_->endTransmission() == 0;
}

bool Tca6408::readReg(uint8_t reg, uint8_t &value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(addr_, (uint8_t)1) != 1) return false;
  value = wire_->read();
  return true;
}

bool Tca6408::writeReg(uint8_t reg, uint8_t value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool Tca6408::write(uint8_t bit, bool high) {
  if (bit > 7) return false;
  const uint8_t out = high ? (out_ | (1u << bit)) : (out_ & ~(1u << bit));
  if (!writeReg(REG_OUTPUT, out)) return false;
  out_ = out;
  return true;
}

bool Tca6408::pinsInput(uint8_t mask) {
  const uint8_t cfg = cfg_ | mask;
  if (!writeReg(REG_CONFIG, cfg)) return false;
  cfg_ = cfg;
  return true;
}

bool Tca6408::pinOutput(uint8_t bit, bool high) {
  if (bit > 7 || !write(bit, high)) return false;
  const uint8_t cfg = cfg_ & ~(1u << bit);
  if (!writeReg(REG_CONFIG, cfg)) return false;
  cfg_ = cfg;
  return true;
}
