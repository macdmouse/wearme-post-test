#pragma once

// =====================================================================================
// Ads1219 — ADC delta-sigma de 24 bits TI ADS1219 (4 entradas, I2C).
//
// Dirección por A0/A1 (0x40-0x4F); en la tarjeta MEMS es 0x41. Se maneja por comandos:
// RESET 0x06, START/SYNC 0x08, POWERDOWN 0x02, RDATA 0x10, RREG 0x20 (config) / 0x24 (estado),
// WREG 0x40 + dato. Registro de configuración: MUX[7:5] GAIN[4] DR[3:2] CM[1] VREF[0].
// Configuración de begin(): disparo único (single-shot), ganancia 1 y referencia interna de
// 2.048 V (rango 0 a 2.048 V en modo unipolar contra AGND). DRDY# no está conectado: el fin de
// la conversión se lee en el bit DRDY del registro de estado.
// Conversión de disparo único a 330 SPS: ~3.2 ms.
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Ads1219 {
public:
  static constexpr uint8_t ADDR_DEFAULT = 0x40; // A0 = A1 = DGND
  static constexpr float VREF_INTERNAL = 2.048f;
  static constexpr int32_t CODE_MAX = 0x7FFFFF;  // fondo de escala positivo (saturado)
  static constexpr int32_t CODE_MIN = -0x800000;

  enum Mux : uint8_t {
    MUX_AIN0_AIN1 = 0,
    MUX_AIN2_AIN3 = 1,
    MUX_AIN1_AIN2 = 2,
    MUX_AIN0 = 3, // AIN0 contra AGND
    MUX_AIN1 = 4,
    MUX_AIN2 = 5,
    MUX_AIN3 = 6,
    MUX_SHORTED = 7, // entradas en corto a (AVDD + AVSS) / 2: medición del offset
  };

  enum DataRate : uint8_t { DR_20SPS = 0, DR_90SPS = 1, DR_330SPS = 2, DR_1000SPS = 3 };

  // Reset, configuración (disparo único, ganancia 1, referencia interna) y lectura de verificación
  bool begin(TwoWire &wire, uint8_t address, DataRate rate = DR_330SPS);
  bool present() const { return present_; }
  uint8_t address() const { return addr_; }
  uint8_t config() const { return config_; } // última configuración escrita
  bool readConfig(uint8_t &value);

  // Inicia una conversión en la entrada pedida (no bloquea). Lista tras conversionMs().
  bool start(Mux mux);
  bool dataReady(bool &ready); // bit DRDY del registro de estado
  bool readRaw(int32_t &code); // resultado de la última conversión (24 bits con signo)
  uint32_t conversionMs() const { return conversionMs_; }

  // Conversión completa (bloquea hasta timeoutMs). Para el POST y la calibración.
  bool convert(Mux mux, int32_t &code, uint32_t timeoutMs = 20);

  // Offset con las entradas en corto (promedio de n conversiones); se resta en toVolts()
  bool calibrateOffset(uint8_t samples = 8);
  int32_t offset() const { return offset_; }
  float toVolts(int32_t code) const { return (code - offset_) * (VREF_INTERNAL / 8388608.0f); }
  static bool saturated(int32_t code) { return code >= CODE_MAX || code <= CODE_MIN; }

  bool powerDown();

private:
  bool command(uint8_t cmd);
  bool writeConfig(uint8_t value);

  TwoWire *wire_ = nullptr;
  uint8_t addr_ = ADDR_DEFAULT;
  uint8_t config_ = 0;
  uint32_t conversionMs_ = 4;
  int32_t offset_ = 0;
  bool present_ = false;
};
