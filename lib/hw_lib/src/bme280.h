#pragma once

// =====================================================================================
// Bme280 — sensor ambiental Bosch BME280 (temperatura, humedad relativa y presión).
//
// I2C a 0x76 (SDO a GND) o 0x77. CHIP_ID (0xD0) = 0x60.
// Configuración de begin(): modo normal, sobremuestreo x1 en T, P y H, filtro IIR x4 y
// t_standby 10 ms -> una medición cada ~18 ms (~55 Hz, el sistema lee a 40 Hz). Con x1 la
// medición tarda 8 ms típ. / 9.3 ms máx. (datasheet sección 9.1).
// La compensación usa las fórmulas enteras del datasheet (sección 4.2.3) con la calibración
// de fábrica de cada chip (0x88-0xA1 y 0xE1-0xE7).
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Bme280 {
public:
  static constexpr uint8_t ADDR_LOW = 0x76;  // SDO a GND
  static constexpr uint8_t ADDR_HIGH = 0x77; // SDO a VDDIO
  static constexpr uint8_t CHIP_ID_VALUE = 0x60;

  enum Reg : uint8_t {
    REG_CALIB_TP = 0x88, // T1..T3, P1..P9 (24 bytes); 0xA1 = H1
    REG_CALIB_H1 = 0xA1,
    REG_CHIP_ID = 0xD0,
    REG_RESET = 0xE0,
    REG_CALIB_H2 = 0xE1, // H2..H6 (7 bytes)
    REG_CTRL_HUM = 0xF2,
    REG_STATUS = 0xF3,
    REG_CTRL_MEAS = 0xF4,
    REG_CONFIG = 0xF5,
    REG_DATA = 0xF7, // presión (3), temperatura (3), humedad (2)
  };

  struct Sample {
    float tempC;
    float humidity;    // %HR
    float pressureHpa; // hPa
  };

  // Verifica CHIP_ID, reset por software, lee la calibración y arranca el modo normal
  bool begin(TwoWire &wire, uint8_t address = ADDR_LOW);
  bool present() const { return present_; }
  uint8_t address() const { return addr_; }

  bool readReg(uint8_t reg, uint8_t &value);
  bool writeReg(uint8_t reg, uint8_t value);

  // Última medición (una sola lectura de 8 bytes). false si no responde o todavía no hay
  // medición (tras el reset los registros valen 0x80000 / 0x8000).
  bool read(Sample &sample);
  bool sleep(); // modo sleep (0.1 uA)
  bool wake();  // vuelve al modo normal (primera medición ~10 ms después)

private:
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
  bool readCalibration();

  TwoWire *wire_ = nullptr;
  uint8_t addr_ = ADDR_LOW;
  bool present_ = false;

  // Calibración de fábrica
  uint16_t t1_ = 0;
  int16_t t2_ = 0, t3_ = 0;
  uint16_t p1_ = 0;
  int16_t p2_ = 0, p3_ = 0, p4_ = 0, p5_ = 0, p6_ = 0, p7_ = 0, p8_ = 0, p9_ = 0;
  uint8_t h1_ = 0, h3_ = 0;
  int16_t h2_ = 0, h4_ = 0, h5_ = 0;
  int8_t h6_ = 0;
};
