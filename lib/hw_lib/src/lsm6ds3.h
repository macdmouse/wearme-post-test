#pragma once

// =====================================================================================
// Lsm6ds3 — IMU de 6 ejes ST LSM6DS3TR-C (acelerómetro + giroscopio + temperatura).
//
// I2C a 0x6A (SA0 a GND) o 0x6B. WHO_AM_I (0x0F) = 0x6A.
// Configuración de begin(): acelerómetro y giroscopio a 52 Hz (el sistema muestrea a 40 Hz), ±2 g y
// 250 dps, BDU activo
// (los bytes alto y bajo de cada dato son de la misma muestra) e incremento automático de dirección.
//   Acelerómetro ±2 g: 0.061 mg/LSB    Giroscopio 250 dps: 8.75 mdps/LSB
//   Temperatura: 25 °C + raw / 256
// Arranque del chip tras alimentarlo: ~35 ms; primera muestra ~10 ms después de configurar el ODR.
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Lsm6ds3 {
public:
  static constexpr uint8_t ADDR_LOW = 0x6A;  // SA0 a GND
  static constexpr uint8_t ADDR_HIGH = 0x6B; // SA0 a VDD
  static constexpr uint8_t WHO_AM_I_VALUE = 0x6A;

  enum Reg : uint8_t {
    REG_WHO_AM_I = 0x0F,
    REG_CTRL1_XL = 0x10,
    REG_CTRL2_G = 0x11,
    REG_CTRL3_C = 0x12,
    REG_STATUS = 0x1E,
    REG_OUT_TEMP_L = 0x20, // temperatura, luego giroscopio (0x22) y acelerómetro (0x28)
  };

  struct Sample {
    float ax, ay, az; // g
    float gx, gy, gz; // dps
    float tempC;
  };

  // Verifica WHO_AM_I, hace reset por software y configura 52 Hz / ±2 g / 250 dps
  bool begin(TwoWire &wire, uint8_t address = ADDR_LOW);
  bool present() const { return present_; }
  uint8_t address() const { return addr_; }

  bool readReg(uint8_t reg, uint8_t &value);
  bool writeReg(uint8_t reg, uint8_t value);

  bool dataReady();          // hay muestra nueva de acelerómetro y giroscopio
  bool read(Sample &sample); // lee temperatura, giroscopio y acelerómetro (una muestra)
  bool powerDown();          // acelerómetro y giroscopio apagados (ODR = 0)

private:
  TwoWire *wire_ = nullptr;
  uint8_t addr_ = ADDR_LOW;
  bool present_ = false;
};
