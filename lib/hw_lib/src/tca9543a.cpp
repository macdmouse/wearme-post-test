#include "tca9543a.h"

bool Tca9543a::begin(TwoWire &wire, uint8_t address) {
  wire_ = &wire;
  addr_ = address;
  present_ = disable(); // responde (ACK) y queda sin canales
  return present_;
}

bool Tca9543a::select(uint8_t channel) {
  if (wire_ == nullptr) return false;
  const uint8_t control = channel > 1 ? 0x00 : (uint8_t)(1u << channel);
  wire_->beginTransmission(addr_);
  wire_->write(control);
  return wire_->endTransmission() == 0;
}

bool Tca9543a::readControl(uint8_t &value) {
  if (wire_ == nullptr) return false;
  if (wire_->requestFrom(addr_, (uint8_t)1) != 1) return false;
  value = wire_->read();
  return true;
}
