#pragma once

// =====================================================================================
// Fan53741 — buck síncrono onsemi FAN53741 (1.3 A) con salida programable por I2C.
//
// I2C 0x52 (fija). VIN 2.3 a 5.5 V: solo responde con su entrada alimentada.
// Registros: 0x00 ID (producto 0001 en bits 7:4), 0x01 fallas (flags), 0x02 fallas (en vivo),
// 0x03 MODE (bit 1 ENABLE, bit 0 FORCE_PWM), 0x04 VSEL (6:0), 0x05 ILIMIT, 0x06 PULLDOWN,
// 0x07 RESET (0x60 = reset por software).
// VSEL: VOUT = 0.6 V + (código − 0x0F) × 25 mV, de 0x0F (0.6 V) a 0x7B (3.3 V; valor por defecto).
// Al alimentarse arranca con ENABLE = 0 (salida apagada). Arranque ~600 µs con soft-start.
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Fan53741 {
public:
  static constexpr uint8_t I2C_ADDR = 0x52;
  static constexpr uint8_t PRODUCT_ID = 0x1; // bits 7:4 de ID_REV

  enum Reg : uint8_t {
    REG_ID_REV = 0x00,
    REG_FAULT_FLAG = 0x01,
    REG_FAULT_LIVE = 0x02,
    REG_MODE = 0x03,
    REG_VSEL = 0x04,
    REG_ILIMIT = 0x05,
    REG_PULLDOWN = 0x06,
    REG_RESET = 0x07,
  };

  // Responde y el ID de producto es 0001
  bool begin(TwoWire &wire);
  bool present() const { return present_; }
  uint8_t idRev() const { return idRev_; }

  bool readReg(uint8_t reg, uint8_t &value);
  bool writeReg(uint8_t reg, uint8_t value);

  bool setVoutMv(uint16_t mv);     // 600 a 3300 mV en pasos de 25 mV
  bool readVoutMv(uint16_t &mv);
  bool enable(bool on);            // bit ENABLE de MODE (FORCE_PWM en 0: PFM/PWM automático)

private:
  TwoWire *wire_ = nullptr;
  uint8_t idRev_ = 0;
  bool present_ = false;
};
