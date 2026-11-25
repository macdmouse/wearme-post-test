#pragma once

// =====================================================================================
// Max17048 — medidor de bateria (fuel gauge) Li+ de 1 celda, Maxim MAX17048 (ModelGauge).
//
// I2C a 0x36 (fija), hasta 400 kHz. Registros de 16 bits, byte alto primero; los dos bytes
// se leen en la misma transaccion. Mide la celda por VDD (alimentado directo de la bateria)
// y calcula el estado de carga (SOC) sin resistencia shunt.
//
//   VCELL 0x02  78.125 uV/LSB          SOC     0x04  1/256 % por LSB
//   CRATE 0x16  0.208 %/h por LSB, con signo (positivo = cargando)
//   VERSION 0x08 = 0x001x               STATUS 0x1A  (RI = recien alimentado, alertas)
//
// Tras encender, el primer SOC esta listo ~1 s despues (debounce de 17 ms + 175 ms + calculo).
// Compensacion de temperatura (CONFIG.RCOMP) pendiente: requiere medir la temperatura de la bateria.
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Max17048 {
public:
  static constexpr uint8_t I2C_ADDR = 0x36;

  enum Reg : uint8_t {
    REG_VCELL = 0x02,
    REG_SOC = 0x04,
    REG_MODE = 0x06,
    REG_VERSION = 0x08,
    REG_HIBRT = 0x0A,
    REG_CONFIG = 0x0C,
    REG_VALRT = 0x14,
    REG_CRATE = 0x16,
    REG_VRESET_ID = 0x18,
    REG_STATUS = 0x1A,
    REG_CMD = 0xFE,
  };

  // STATUS (0x1A): bits del byte alto
  static constexpr uint16_t STATUS_RI = 0x0100;   // recien alimentado: IC sin configurar
  static constexpr uint16_t STATUS_VH = 0x0200;   // voltaje por encima de VALRT.MAX
  static constexpr uint16_t STATUS_VL = 0x0400;   // voltaje por debajo de VALRT.MIN
  static constexpr uint16_t STATUS_VR = 0x0800;   // reset por voltaje (si EnVr)
  static constexpr uint16_t STATUS_HD = 0x1000;   // SOC bajo (CONFIG.ATHD)
  static constexpr uint16_t STATUS_SC = 0x2000;   // cambio de SOC de 1 % (CONFIG.ALSC)
  static constexpr uint16_t STATUS_ALERTS = STATUS_VH | STATUS_VL | STATUS_VR | STATUS_HD | STATUS_SC;

  // CONFIG (0x0C): bits del byte bajo
  static constexpr uint16_t CONFIG_ALRT = 0x0020; // alerta activa (pin ALRT en bajo)

  // Busca el chip y lee VERSION. Si STATUS.RI esta en 1 (recien alimentado) lo limpia.
  bool begin(TwoWire &wire);
  bool present() const { return present_; }
  uint16_t version() const { return version_; }
  bool powerOnReset() const { return powerOnReset_; } // RI estaba en 1 en begin()

  bool probe();
  bool readReg(uint8_t reg, uint16_t &value);
  bool writeReg(uint8_t reg, uint16_t value);

  bool readVoltageMv(uint16_t &mv);           // voltaje de la celda en mV
  bool readSoc(float &percent);               // estado de carga en %
  bool readCrate(float &percentPerHour);      // tasa de carga (+) / descarga (-) en %/h
  bool readStatus(uint16_t &status);
  bool clearAlerts();                         // limpia STATUS (alertas + RI) y CONFIG.ALRT

private:
  TwoWire *wire_ = nullptr;
  bool present_ = false;
  bool powerOnReset_ = false;
  uint16_t version_ = 0;
};
