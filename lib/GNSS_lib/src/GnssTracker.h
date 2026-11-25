#pragma once

// =====================================================================================
// GnssTracker — Ciclo GNSS completo sobre LoraNode (lora_lib), sin tocar la capa LoRaWAN.
//
// Todo ocurre en el "hueco" a mitad de camino entre dos uplinks del sensor (RX1/RX2 ya
// cerraron). En cada hueco se hace UNA sola accion, en este orden:
//   1. Pedir la hora GPS a la red (DeviceTimeReq), una vez tras el join -> habilita el asistido.
//   2. Enviar el NAV pendiente del ultimo escaneo (FPort 192, formato LoRa Cloud).
//   3. Escanear (cada scanPeriodMs) y enviar el reporte doppler (FPort 3, ver abajo). Con
//      dopplerFport = 0 se envian en su lugar estadisticas Cayenne (FPort 2, 11 B):
//      digital_in_4 satelites, analog_in_5 mejor C/N0, analog_in_6 duracion (s, -1 = fallo).
// Secuencia con 30 s / 60 s: t0 sensor | t15 hora | t30 sensor | t45 escaneo | t60 sensor | t75 NAV
//
// Reporte doppler (FPort 3, big endian), en todos los escaneos aunque haya < minSatellites.
// Sirve para calcular una posicion doppler aproximada en Node-RED:
//   byte 0     version (4 bits altos, = 1) | flags (bit0 = hora de red, bit1 = asistido,
//              bit2 = almanaque vencido)
//   bytes 1-4  hora GPS del escaneo (uint32, s)
//   byte 5     duracion del escaneo (decimas de s, saturada a 255; 255 = escaneo fallido)
//   byte 6     satelites detectados en total (0 si el escaneo fallo)
//   bytes 7..  por satelite, 4 B: id (uint8; GPS 0-31, BeiDou 64-127) | C/N0 (int8, dB) |
//              doppler (int16, Hz)
// Satelites ordenados por C/N0 de mayor a menor; solo los que caben en el DR actual
// ((maxPayload - 7) / 4: 11 satelites = 51 B a DR3/SF9). Escaneo fallido = solo los 7 B.
//
// Uso:
//   GnssTracker gnss(lora);
//   setup():  gnss.begin(cfg);
//   loop():   if (lora.send()) gnss.sensorUplinkSent();   // tras cada uplink del sensor
//             gnss.process();
// =====================================================================================

#include <lora_lib.h>
#include "GNSS_lib.h"

struct GnssTrackerConfig {
  uint32_t sensorPeriodMs = 30000; // periodo de los uplinks del sensor (define los huecos)
  uint32_t scanPeriodMs = 60000;   // cada cuanto escanear
  uint8_t statsFport = 2;          // estadisticas Cayenne (0 = no enviar); solo si dopplerFport = 0
  uint8_t dopplerFport = 3;        // reporte doppler por escaneo (0 = desactivado)
  uint8_t navFport = GNSS_NAV_FPORT; // NAV para el solver (0 = no enviar)
  uint8_t minSatellites = 6;       // el NAV solo se envia con al menos N satelites (menos = mucho error)
  uint8_t maxSatellites = 11;      // tope del escaneo (NAV de ~53 B, limite de DR3)
  bool assist = false;             // true = usar assistLatitude/Longitude para el escaneo asistido
  float assistLatitude = 0;
  float assistLongitude = 0;
};

class GnssTracker {
public:
  enum Event : uint8_t {
    SCAN_START,     // antes del escaneo bloqueante
    SCAN_NAV,       // escaneo con >= minSatellites: hay NAV pendiente
    SCAN_EMPTY,     // escaneo fallido o con menos de minSatellites: no se envia NAV
    NAV_SENT,       // NAV entregado al modem
    NAV_FAILED,     // NAV no cabe en el DR actual o el modem lo rechazo
  };
  typedef void (*EventCallback)(Event event, const GnssResult &result);

  explicit GnssTracker(LoraNode &lora, GnssScanner::Constellation constellations = GnssScanner::GPS_BEIDOU);

  void begin(const GnssTrackerConfig &config);
  void onEvent(EventCallback callback) { callback_ = callback; }

  // Avisar despues de cada uplink del sensor: marca el inicio del ciclo
  void sensorUplinkSent() { lastSensorUplinkMs_ = millis(); }

  // Llamar en cada loop(). Puede bloquear 5-10 s cuando toca escanear.
  void process();

  const GnssResult &result() const { return scanner_.result(); }

private:
  void reportTimeSync();
  void scanAndReport();
  void sendDopplerReport();
  void sendPendingNav();
  void emit(Event event) {
    if (callback_) callback_(event, scanner_.result());
  }

  LoraNode &lora_;
  GnssScanner scanner_;
  GnssTrackerConfig config_;
  EventCallback callback_ = nullptr;

  uint32_t lastSensorUplinkMs_ = 0;
  uint32_t lastSlotMs_ = 0; // lastSensorUplinkMs_ del ultimo hueco atendido
  uint32_t lastScanMs_ = 0;
  bool scanned_ = false;
  bool timeSyncStarted_ = false;
  bool timeSyncReported_ = false;
  uint8_t navBuf_[GNSS_MAX_NAV_SIZE];
  uint8_t navLen_ = 0; // NAV pendiente de enviar (0 = nada)
};
