#pragma once

// =====================================================================================
// Tca9543a — multiplexor I2C de 2 canales TI TCA9543A (0x70-0x73 por A0/A1).
//
// Un solo registro de control: escribir un byte habilita canales (bit 0 = canal 0, bit 1 = canal 1);
// leerlo devuelve los canales habilitados (bits 1:0) y las interrupciones (bits 5:4). Sirve para usar
// dos dispositivos con la misma dirección (en Wearme, los dos FS3000 en 0x28).
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Tca9543a {
public:
  static constexpr uint8_t ADDR_DEFAULT = 0x70; // A0 = A1 = GND
  static constexpr uint8_t NONE = 0xFF;

  bool begin(TwoWire &wire, uint8_t address = ADDR_DEFAULT); // responde y deja los canales cerrados
  bool present() const { return present_; }
  bool select(uint8_t channel); // 0 o 1; NONE = ninguno
  bool disable() { return select(NONE); }
  bool readControl(uint8_t &value);

private:
  TwoWire *wire_ = nullptr;
  uint8_t addr_ = ADDR_DEFAULT;
  bool present_ = false;
};
