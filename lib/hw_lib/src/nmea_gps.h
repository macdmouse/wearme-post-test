#pragma once

// =====================================================================================
// NmeaGps — lector NMEA 0183 para el GPS Quectel L96-M33 (UART 9600 8N1, 1 Hz por defecto).
//
// Se le pasan los caracteres de la UART con feed() (no bloquea). Valida el checksum (*hh) y
// procesa GGA (fix, satélites, HDOP, posición, altura) y RMC (hora y fecha UTC). Acepta cualquier
// talker (GP, GN, GL, GA, BD...). Las demás tramas solo cuentan como recibidas.
// =====================================================================================

#include <Arduino.h>

class NmeaGps {
public:
  struct Fix {
    uint8_t quality = 0;   // GGA: 0 = sin fix, 1 = GPS, 2 = DGPS...
    uint8_t satellites = 0;
    float hdop = 0;
    double latitude = 0;   // grados, + norte
    double longitude = 0;  // grados, + este
    float altitudeM = 0;   // sobre el nivel del mar
    bool timeValid = false;
    bool dateValid = false;
    uint8_t hour = 0, minute = 0, second = 0; // UTC
    uint8_t day = 0, month = 0;
    uint16_t year = 0;
    uint32_t updatedMs = 0; // millis() de la última GGA
  };

  // Procesa un carácter; true cuando termina una trama válida (checksum correcto)
  bool feed(char c);
  // GPS recién encendido: olvida el fix anterior (si no, la primera trama que no es GGA lo mostraría como actual)
  void reset() { fix_ = Fix(); len_ = 0; inSentence_ = false; }

  const Fix &fix() const { return fix_; }
  bool hasFix() const { return fix_.quality > 0; }
  uint32_t sentences() const { return sentences_;   } // tramas con checksum correcto
  uint32_t checksumErrors() const { return badSentences_; }
  const char *lastType() const { return lastType_; } // p. ej. "GNGGA"

private:
  bool parse(char *sentence);
  void parseGga(char **f, uint8_t n);
  void parseRmc(char **f, uint8_t n);

  char buf_[96];
  uint8_t len_ = 0;
  bool inSentence_ = false;
  Fix fix_;
  uint32_t sentences_ = 0;
  uint32_t badSentences_ = 0;
  char lastType_[8] = "";
};
