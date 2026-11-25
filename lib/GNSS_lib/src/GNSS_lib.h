#pragma once

// =====================================================================================
// GNSS_lib — Pruebas del receptor GNSS del LR1110 (Wio Tracker 1110 / WM1110)
//
// El LR1110 NO es un GPS convencional: no calcula latitud/longitud en el dispositivo.
// Hace un escaneo corto de GPS/BeiDou y entrega:
//   - los satelites detectados (ID, C/N0 en dB, doppler)
//   - un "mensaje NAV" comprimido que un solver en la nube convierte en posicion
// Esta libreria sirve para comprobar que la antena y el receptor funcionan: cuantos
// satelites se ven y con que calidad de senal.
//
// Modos de escaneo:
//   - Autonomo: sin datos previos. Mas lento y necesita senal fuerte.
//   - Asistido: con hora GPS real (sincronizada por LoRaWAN) + posicion aproximada + almanaque
//     del LR1110. Busca solo los satelites que deberian estar visibles: mas rapido y sensible.
//     Si el almanaque esta vencido (0x08) se repite en autonomo en el mismo escaneo y no se
//     reintenta asistido por 10 escaneos. Si detecta 0 satelites, el siguiente es autonomo.
//
// Uso (junto a lora_lib):
//
//   GnssScanner gnss;
//   ...
//   if (gnss.scan()) {           // BLOQUEANTE: 2-10 s, pausa el modem LoRaWAN
//     gnss.printResult();
//     const GnssResult &r = gnss.result();
//   }
//
// IMPORTANTE:
//   - Llamar scan() solo cuando la red LoRaWAN este unida y no haya un uplink en curso
//     (fuera de las ventanas RX1/RX2). Si el modem esta ocupado, scan() devuelve false.
//   - Durante el escaneo no se ejecuta lora.process(); se reanuda al terminar.
//   - En interiores lo normal es 0 satelites: la prueba debe hacerse con cielo abierto.
// =====================================================================================

#include <Arduino.h>
#include <LbmWm1110.hpp>

static constexpr uint8_t GNSS_MAX_SATELLITES = 32;
static constexpr uint16_t GNSS_MAX_NAV_SIZE = 128;

// Puerto que usa el middleware de Semtech (y espera LoRa Cloud) para los mensajes NAV
static constexpr uint8_t GNSS_NAV_FPORT = 192;

struct GnssSatellite {
  uint8_t id;      // GPS: 0-31, BeiDou: 64-127 (numeracion interna del LR1110)
  int8_t cnr;      // C/N0 en dB (calidad de senal; >30 dB es buena)
  int16_t doppler; // Hz
  bool isBeidou() const { return id >= 64; }
};

struct GnssResult {
  bool valid = false;           // el escaneo termino (aunque detecte 0 satelites)
  const char *error = nullptr;  // motivo si valid == false
  uint8_t satellites = 0;       // total detectados
  uint8_t gps = 0;
  uint8_t beidou = 0;
  int8_t bestCnr = 0;           // mejor C/N0 (dB)
  int8_t avgCnr = 0;            // promedio C/N0 (dB)
  uint32_t scanTimeMs = 0;      // duracion total medida desde el MCU
  uint32_t radioMs = 0;         // tiempo de radio reportado por el LR1110
  uint32_t computationMs = 0;   // tiempo de calculo reportado por el LR1110
  bool assisted = false;        // true = escaneo asistido (hora + posicion), false = autonomo
  bool almanacOld = false;      // se intento asistido pero el almanaque del LR1110 esta vencido
  bool realTime = false;        // true = hora GPS de la red (DeviceTimeReq), false = de compilacion
  uint32_t gpsTime = 0;         // hora GPS (s) usada para el escaneo
  uint16_t navSize = 0;         // bytes del mensaje NAV (0 si no hay)
  uint8_t nav[GNSS_MAX_NAV_SIZE];
  GnssSatellite sats[GNSS_MAX_SATELLITES];
};

class GnssScanner {
public:
  // Constelaciones a escanear
  enum Constellation : uint8_t { GPS = 0x01, BEIDOU = 0x02, GPS_BEIDOU = 0x03 };

  explicit GnssScanner(Constellation constellations = GPS_BEIDOU, uint32_t timeoutMs = 15000);

  // Ejecuta un escaneo (bloqueante). Asistido si hay hora de red y posicion; si no, autonomo.
  // Devuelve true si el escaneo termino.
  bool scan();

  const GnssResult &result() const { return result_; }

  // Maximo de satelites que el LR1110 reporta e incluye en el NAV (Semtech usa 9; aqui 11 por
  // defecto). Cada satelite agrega ~4.5 B al NAV: con 11 el NAV ronda 53 B, el maximo a DR3/SF9.
  void setMaxSatellites(uint8_t n) { maxSatellites_ = n; }

  // Posicion aproximada (grados) para el escaneo asistido. Precision de ~150 km es suficiente.
  void setAssistancePosition(float latitude, float longitude);

  // Pide la hora GPS a la red (LoRaWAN DeviceTimeReq). Llamar una vez despues del join;
  // LBM la resincroniza sola. Devuelve true si el servicio arranco.
  static bool startTimeSync();

  // Hora GPS de la red (s). false si aun no hay sincronizacion.
  static bool networkGpsTime(uint32_t &gpsSeconds);

  // Arma el uplink NAV en el formato del middleware de Semtech (FPort 192, LoRa Cloud):
  // byte 0 = | ultimo NAV (1 bit) | RFU (2 bits) | token (5 bits) |, luego el NAV sin su byte
  // de destino. Devuelve el tamano, o 0 si el ultimo escaneo no produjo un NAV para el solver.
  uint8_t buildNavUplink(uint8_t *out, uint8_t maxLen);

  // Imprime el resultado por Serial con prefijo [GNSS]
  void printResult() const;

private:
  bool runScan(const void *radio);
  bool startScan(const void *radio);
  void readResults(const void *radio);
  static uint32_t approxGpsTimeSeconds();
  static const char *hostStatusText(uint8_t code);

  Constellation constellations_;
  uint32_t timeoutMs_;
  uint8_t maxSatellites_;
  bool versionPrinted_;
  bool hasPosition_;
  float latitude_;
  float longitude_;
  uint8_t assistSkip_;     // escaneos que se haran en autonomo antes de reintentar asistido
  uint8_t token_;          // identificador del grupo de escaneo (LoRa Cloud)
  GnssResult result_;
};
