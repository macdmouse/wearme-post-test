#pragma once

// =====================================================================================
// Mcp4018 — potenciómetro digital Microchip MCP4018 (127 pasos, volátil, I2C 0x2F fija).
//
// El terminal B está unido a VSS dentro del chip: la resistencia del cursor a B es
//   R_WB = paso / 127 × R_AB (+ R_W, ~100 Ω de resistencia del cursor)
// En la tarjeta MEMS (MCP4018T-502E, R_AB = 5 kΩ ±20 %) es la resistencia de carga del sensor CH4.
// Es volátil: al encenderse vuelve a la mitad (paso 63) y hay que escribirlo en cada arranque.
// Escritura y lectura de 1 byte, sin dirección de registro.
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Mcp4018 {
public:
  static constexpr uint8_t I2C_ADDR = 0x2F;
  static constexpr uint8_t STEP_MAX = 127;

  bool begin(TwoWire &wire, uint32_t rabOhms); // verifica que responde
  bool present() const { return present_; }

  bool write(uint8_t step);
  bool read(uint8_t &step);

  // Paso más cercano a la resistencia pedida (R_WB nominal, sin R_W)
  uint8_t stepFor(uint32_t ohms) const;
  uint32_t ohmsAt(uint8_t step) const { return (uint32_t)step * rab_ / STEP_MAX; }

private:
  TwoWire *wire_ = nullptr;
  uint32_t rab_ = 5000;
  bool present_ = false;
};
