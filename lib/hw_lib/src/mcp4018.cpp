#include "mcp4018.h"

bool Mcp4018::begin(TwoWire &wire, uint32_t rabOhms) {
  wire_ = &wire;
  rab_ = rabOhms;
  uint8_t step = 0;
  present_ = read(step);
  return present_;
}

bool Mcp4018::write(uint8_t step) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(step > STEP_MAX ? STEP_MAX : step);
  return wire_->endTransmission() == 0;
}

bool Mcp4018::read(uint8_t &step) {
  if (wire_ == nullptr) return false;
  if (wire_->requestFrom(I2C_ADDR, (uint8_t)1) != 1) return false;
  step = wire_->read() & 0x7F;
  return true;
}

uint8_t Mcp4018::stepFor(uint32_t ohms) const {
  if (ohms >= rab_) return STEP_MAX;
  return (uint8_t)((ohms * STEP_MAX + rab_ / 2) / rab_);
}
