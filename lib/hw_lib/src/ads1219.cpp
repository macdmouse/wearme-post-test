#include "ads1219.h"

static constexpr uint8_t CMD_RESET = 0x06;
static constexpr uint8_t CMD_START = 0x08;
static constexpr uint8_t CMD_POWERDOWN = 0x02;
static constexpr uint8_t CMD_RDATA = 0x10;
static constexpr uint8_t CMD_RREG_CONFIG = 0x20;
static constexpr uint8_t CMD_RREG_STATUS = 0x24;
static constexpr uint8_t CMD_WREG = 0x40;
static constexpr uint8_t STATUS_DRDY = 0x80;

// Tiempo de una conversión de disparo único por velocidad (datasheet: 50.4 / 11.3 / 3.2 / 1.2 ms), con margen
static const uint8_t CONVERSION_MS[] = {52, 12, 4, 2};

bool Ads1219::begin(TwoWire &wire, uint8_t address, DataRate rate) {
  wire_ = &wire;
  addr_ = address;
  present_ = false;
  offset_ = 0;
  if (!command(CMD_RESET)) return false;
  delay(1); // el reset tarda ~0.5 ms
  conversionMs_ = CONVERSION_MS[rate & 0x03];
  // MUX = AIN0, GAIN = 1, DR, CM = 0 (disparo único), VREF = 0 (interna 2.048 V)
  if (!writeConfig((uint8_t)((MUX_AIN0 << 5) | ((rate & 0x03) << 2)))) return false;
  uint8_t readBack = 0;
  if (!readConfig(readBack) || readBack != config_) return false;
  present_ = true;
  return true;
}

bool Ads1219::command(uint8_t cmd) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(cmd);
  return wire_->endTransmission() == 0;
}

bool Ads1219::writeConfig(uint8_t value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(CMD_WREG);
  wire_->write(value);
  if (wire_->endTransmission() != 0) return false;
  config_ = value;
  return true;
}

bool Ads1219::readConfig(uint8_t &value) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(CMD_RREG_CONFIG);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(addr_, (uint8_t)1) != 1) return false;
  value = wire_->read();
  return true;
}

bool Ads1219::start(Mux mux) {
  const uint8_t wanted = (uint8_t)((config_ & 0x1F) | ((mux & 0x07) << 5));
  if (wanted != config_ && !writeConfig(wanted)) return false;
  return command(CMD_START);
}

bool Ads1219::dataReady(bool &ready) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(CMD_RREG_STATUS);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(addr_, (uint8_t)1) != 1) return false;
  ready = (wire_->read() & STATUS_DRDY) != 0;
  return true;
}

bool Ads1219::readRaw(int32_t &code) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(addr_);
  wire_->write(CMD_RDATA);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(addr_, (uint8_t)3) != 3) return false;
  uint32_t raw = (uint32_t)wire_->read() << 16;
  raw |= (uint32_t)wire_->read() << 8;
  raw |= wire_->read();
  code = (raw & 0x800000) ? (int32_t)(raw | 0xFF000000) : (int32_t)raw; // extensión de signo
  return true;
}

bool Ads1219::convert(Mux mux, int32_t &code, uint32_t timeoutMs) {
  if (!start(mux)) return false;
  delay(conversionMs_ > 1 ? conversionMs_ - 1 : 1);
  const uint32_t t0 = millis();
  bool ready = false;
  while (dataReady(ready) && !ready) {
    if (millis() - t0 >= timeoutMs) return false;
    delay(1);
  }
  return ready && readRaw(code);
}

bool Ads1219::calibrateOffset(uint8_t samples) {
  if (samples == 0) return false;
  int32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    int32_t code = 0;
    if (!convert(MUX_SHORTED, code)) return false;
    sum += code;
  }
  offset_ = sum / samples;
  return true;
}

bool Ads1219::powerDown() { return command(CMD_POWERDOWN); }
