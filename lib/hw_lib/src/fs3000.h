#pragma once

// =====================================================================================
// Fs3000 — sensor de velocidad de aire Renesas FS3000-1015 (0 a 15 m/s), I2C 0x28 (fija).
//
// Lectura de 5 bytes: [checksum][dato alto][dato bajo][genérico][genérico]. El dato es de 12 bits
// (bits 11:0). Checksum: la suma de los 5 bytes, en 8 bits, debe dar 0.
// Velocidad: interpolación lineal en la tabla de calibración del fabricante (cuentas -> m/s) del
// FS3000-1015; por debajo de 409 cuentas = 0 m/s y por encima de la tabla = el máximo.
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Fs3000 {
public:
  static constexpr uint8_t I2C_ADDR = 0x28;

  // Resultado de la última lectura (para decir por qué falló)
  // NOT_READY: los 5 bytes en 0 (pasan el checksum pero no son un dato: el sensor recién encendido aún no mide)
  enum Status : uint8_t { OK, NO_RESPONSE, BAD_CHECKSUM, NOT_READY };

  // Lee una muestra; false si no responde, el checksum no coincide o aún no mide (el motivo queda en lastStatus())
  bool read(TwoWire &wire, uint16_t &raw, float &metersPerSecond);
  Status lastStatus() const { return status_; }
  static float toMetersPerSecond(uint16_t raw);

private:
  Status status_ = OK;
};
