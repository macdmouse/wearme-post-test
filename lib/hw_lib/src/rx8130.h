#pragma once

// =====================================================================================
// Rx8130 — reloj de tiempo real Epson RX8130CE (manual de aplicación ETM50E-08).
//
// I2C a 0x32 (fija), hasta 400 kHz; cada acceso debe durar < 0.95 s (si no, el RTC reinicia su I2C).
// Hora en BCD: segundos 0x10, minutos 0x11, horas 0x12 (24 h), día de la semana 0x13 (un bit por día,
// 01h = domingo), día 0x14, mes 0x15, año 0x16 (00-99 = 2000-2099).
//   1Ch Extension: FSEL1 FSEL0 USEL TE WADA TSEL2 TSEL1 TSEL0
//   1Dh Flag:      VBLF 0 UF TF AF RSF VLF VBFF      (las banderas se limpian escribiendo 0)
//   1Eh Control0:  TEST STOP UIE TIE AIE TSTP TBKON TBKE
//   1Fh Control1:  SMPTSEL1 SMPTSEL0 CHGEN INIEN 0 RSVSEL BFVSEL1 BFVSEL0
//   30h Digital offset (DTE + L7..L1)
//
// Placa Wearme: VBAT unido a VDD, sin batería de respaldo (circuito EX5 del manual): INIEN = 1,
// CHGEN = 0 -> 1Fh = 10h. CHGEN queda en 0: nunca se carga nada por VBAT. FOUT no se usa: apagado.
// VLF = 1 (primer encendido o caída de voltaje): se inicializan todos los registros (sección 18.4).
// La hora se guarda en UTC.
// =====================================================================================

#include <Arduino.h>
#include <Wire.h>

class Rx8130 {
public:
  static constexpr uint8_t I2C_ADDR = 0x32;

  static constexpr uint8_t REG_SEC = 0x10;
  static constexpr uint8_t REG_EXT = 0x1C;
  static constexpr uint8_t REG_FLAG = 0x1D;
  static constexpr uint8_t REG_CTRL0 = 0x1E;
  static constexpr uint8_t REG_CTRL1 = 0x1F;
  static constexpr uint8_t REG_OFFSET = 0x30;

  static constexpr uint8_t FLAG_VBLF = 0x80; // VBAT bajo
  static constexpr uint8_t FLAG_RSF = 0x04;  // VDD bajó de VDET1 (reset)
  static constexpr uint8_t FLAG_VLF = 0x02;  // oscilador detenido / datos de hora no válidos
  static constexpr uint8_t CTRL0_STOP = 0x40;

  // Configuración de la placa (ver arriba)
  static constexpr uint8_t EXT_CONFIG = 0xC4;   // FSEL = 11 (FOUT apagado), TE = 0, TSEL2 = 1 (valor por defecto)
  static constexpr uint8_t CTRL1_CONFIG = 0x10; // INIEN = 1, CHGEN = 0, RSVSEL = 0 (2.75 V), BFVSEL = 00

  struct DateTime {
    uint16_t year;   // 2000-2099
    uint8_t month;   // 1-12
    uint8_t day;     // 1-31
    uint8_t hour;    // 0-23
    uint8_t minute;  // 0-59
    uint8_t second;  // 0-59
    uint8_t weekday; // 0 = domingo
  };

  // Detecta el chip. Si VLF = 1 inicializa todos los registros y carga `fallback` (espera a que
  // arranque el oscilador: hasta 1 s, solo en ese caso). Si VLF = 0 solo corrige la configuración
  // (1Ch, 1Eh, 1Fh) si no es la de la placa, sin tocar la hora.
  bool begin(TwoWire &wire, const DateTime &fallback);
  bool present() const { return present_; }
  bool timeLost() const { return timeLost_; }     // VLF estaba en 1 en begin()
  bool configFixed() const { return configFixed_; } // begin() tuvo que corregir 1Ch/1Eh/1Fh
  uint8_t bootFlags() const { return bootFlags_; }  // registro 1Dh leído en begin() (diagnóstico: VLF, RSF, VBLF)

  bool readTime(DateTime &t);       // false si no responde o los registros no son BCD válidos
  bool setTime(const DateTime &t);  // STOP = 1, escribe la hora, STOP = 0 (sección 18.5) y limpia banderas
  bool readFlags(uint8_t &flags);
  bool readReg(uint8_t reg, uint8_t &value) { return readRegs(reg, &value, 1); }

  static bool buildTime(DateTime &t);                     // hora de compilación (__DATE__ / __TIME__)
  static void fromUnix(uint32_t unixSeconds, DateTime &t); // segundos desde 1970-01-01 (UTC)
  static uint32_t toUnix(const DateTime &t);
  static uint8_t weekdayOf(uint16_t year, uint8_t month, uint8_t day);

private:
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
  bool writeRegs(uint8_t reg, const uint8_t *buf, uint8_t len);
  bool writeReg(uint8_t reg, uint8_t value) { return writeRegs(reg, &value, 1); }
  bool clearVlf(); // reintenta hasta 1 s: VLF no se borra hasta que arranca el oscilador

  TwoWire *wire_ = nullptr;
  bool present_ = false;
  bool timeLost_ = false;
  bool configFixed_ = false;
  uint8_t bootFlags_ = 0;
};
