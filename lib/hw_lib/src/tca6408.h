#pragma once

// =====================================================================================
// Tca6408 — expansor de 8 E/S por I2C, TI TCA6408A.
//
// Direccion: 0x20 con ADDR a GND, 0x21 con ADDR a VCC.
// Registros: 0 entradas, 1 salidas, 2 inversion de polaridad, 3 configuracion (1 = entrada).
// Tras encendido o reset los 8 pines son entradas (alta impedancia) y el registro de salidas
// vale 0xFF: por eso pinOutput() escribe primero el nivel y despues cambia la direccion, asi
// un pin nunca sale en ALTO por un instante (importante con los EN de los SIP32431).
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Tca6408 {
public:
  static constexpr uint8_t ADDR_LOW = 0x20;  // ADDR a GND
  static constexpr uint8_t ADDR_HIGH = 0x21; // ADDR a VCC

  enum Reg : uint8_t { REG_INPUT = 0x00, REG_OUTPUT = 0x01, REG_POLARITY = 0x02, REG_CONFIG = 0x03 };

  // Busca el chip y lee sus registros de salida y configuracion. No cambia ningun pin.
  bool begin(TwoWire &wire, uint8_t address);
  bool present() const { return present_; }
  uint8_t address() const { return addr_; }

  bool probe(); // true si responde con ACK
  bool readReg(uint8_t reg, uint8_t &value);
  bool writeReg(uint8_t reg, uint8_t value);

  bool readInputs(uint8_t &value) { return readReg(REG_INPUT, value); }
  bool pinOutput(uint8_t bit, bool high); // configura el pin como salida con ese nivel
  bool pinsInput(uint8_t mask);           // pone como entrada (alta impedancia) los pines de la máscara
  bool write(uint8_t bit, bool high);     // cambia el nivel de una salida ya configurada

  uint8_t outputs() const { return out_; } // copia local del registro de salidas
  uint8_t config() const { return cfg_; }  // copia local del registro de configuracion

private:
  TwoWire *wire_ = nullptr;
  uint8_t addr_ = ADDR_LOW;
  bool present_ = false;
  uint8_t out_ = 0xFF;
  uint8_t cfg_ = 0xFF;
};
