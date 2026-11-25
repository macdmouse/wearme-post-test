#include "led_lib.h"

void StatusLeds::begin(uint8_t greenPin, uint8_t redPin, uint8_t bluePin, bool activeLow) {
  activeLow_ = activeLow;
  ch_[GREEN] = Channel{greenPin, 0, 0, 0, 0, false, false};
  ch_[RED] = Channel{redPin, 0, 0, 0, 0, false, false};
  ch_[BLUE] = Channel{bluePin, 0, 0, 0, 0, false, false};
  for (Channel &c : ch_) {
    write(c, false); // nivel de apagado antes de pasar a salida
    pinMode(c.pin, OUTPUT);
  }
}

void StatusLeds::write(Channel &c, bool on) {
  c.on = on;
  digitalWrite(c.pin, (on != activeLow_) ? HIGH : LOW);
}

void StatusLeds::flash(Led led, uint8_t count, uint16_t onMs, uint16_t offMs) {
  Channel &c = ch_[led];
  c.forced = false;
  c.togglesLeft = count * 2;
  c.onMs = onMs;
  c.offMs = offMs;
  c.nextMs = millis();
  write(c, false);
}

void StatusLeds::set(Led led, bool on) {
  Channel &c = ch_[led];
  c.togglesLeft = 0;
  c.forced = on;
  write(c, on);
}

bool StatusLeds::busy() const {
  for (const Channel &c : ch_) {
    if (c.togglesLeft > 0) return true;
  }
  return false;
}

void StatusLeds::booting() {
  for (uint8_t i = 0; i < COUNT; i++) set(static_cast<Led>(i), true);
}

void StatusLeds::allOff() {
  for (uint8_t i = 0; i < COUNT; i++) set(static_cast<Led>(i), false);
}

void StatusLeds::play(const LedCode &code) {
  if (code.blinks == 0) return;
  if (code.leds & LED_MASK_GREEN) flash(GREEN, code.blinks, code.onMs, code.offMs);
  if (code.leds & LED_MASK_RED) flash(RED, code.blinks, code.onMs, code.offMs);
  if (code.leds & LED_MASK_BLUE) flash(BLUE, code.blinks, code.onMs, code.offMs);
}

void StatusLeds::periodic(const LedCode *code, uint32_t periodMs) {
  periodic_ = (periodMs != 0) ? code : nullptr;
  periodMs_ = periodMs;
  periodicNextMs_ = millis() + periodMs; // el primero un periodo después (el cambio de estado ya destella)
}

void StatusLeds::update() {
  const uint32_t now = millis();

  for (Channel &c : ch_) {
    if (c.forced || c.togglesLeft == 0 || (int32_t)(now - c.nextMs) < 0) continue;
    write(c, !c.on);
    c.togglesLeft--;
    c.nextMs = now + (c.on ? c.onMs : c.offMs);
  }

  // Patrón periódico: se salta si alguno de sus LEDs está ocupado (destello de otro evento o fijo)
  if (periodic_ == nullptr || (int32_t)(now - periodicNextMs_) < 0) return;
  periodicNextMs_ = now + periodMs_;
  const Led leds[COUNT] = {GREEN, RED, BLUE};
  const uint8_t masks[COUNT] = {LED_MASK_GREEN, LED_MASK_RED, LED_MASK_BLUE};
  for (uint8_t i = 0; i < COUNT; i++) {
    const Channel &c = ch_[leds[i]];
    if ((periodic_->leds & masks[i]) && (c.forced || c.togglesLeft > 0)) return;
  }
  play(*periodic_);
}
