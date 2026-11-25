#include "GnssTracker.h"
#include "debug_config.h"

// printf con el mismo interruptor que DEBUG_PRINT/DEBUG_PRINTLN (Debug_Wearme en debug_config.h)
#ifndef DEBUG_PRINTF
#define DEBUG_PRINTF(...) do { if (Debug_Wearme) { DEBUG_PORT.printf(__VA_ARGS__); } } while (0)
#endif
#include <lbm/smtc_modem_api/smtc_modem_api.h>

static constexpr uint8_t MODEM_STACK_ID = 0;

// Reporte doppler (FPort 3): formato en GnssTracker.h
static constexpr uint8_t DOPPLER_VERSION = 1;
static constexpr uint8_t DOPPLER_HEADER_SIZE = 7;
static constexpr uint8_t DOPPLER_SAT_SIZE = 4;
static constexpr uint8_t DOPPLER_DEFAULT_MAX_PAYLOAD = 51; // DR3/SF9 si el modem no informa el maximo

GnssTracker::GnssTracker(LoraNode &lora, GnssScanner::Constellation constellations)
    : lora_(lora), scanner_(constellations) {}

void GnssTracker::begin(const GnssTrackerConfig &config) {
  config_ = config;
  scanner_.setMaxSatellites(config_.maxSatellites);
  if (config_.assist) scanner_.setAssistancePosition(config_.assistLatitude, config_.assistLongitude);
}

void GnssTracker::process() {
  reportTimeSync();

  // Hueco GNSS: una accion por ciclo, lejos de los uplinks del sensor
  const uint32_t offset = config_.sensorPeriodMs / 2;
  const uint32_t sinceSensor = millis() - lastSensorUplinkMs_;
  if (!lora_.isJoined() || lastSensorUplinkMs_ == 0 || lastSlotMs_ == lastSensorUplinkMs_ ||
      sinceSensor < offset || sinceSensor >= config_.sensorPeriodMs) {
    return;
  }
  lastSlotMs_ = lastSensorUplinkMs_;

  if (!timeSyncStarted_) {
    timeSyncStarted_ = true;
    DEBUG_PRINTLN(GnssScanner::startTimeSync() ? "[GNSS] Solicitud de hora a la red (DeviceTimeReq)"
                                                : "[GNSS] No se pudo iniciar la sincronizacion de hora");
  } else if (navLen_ > 0) {
    sendPendingNav();
  } else if (!scanned_ || millis() - lastScanMs_ >= config_.scanPeriodMs - offset) {
    scanAndReport();
  }
}

void GnssTracker::reportTimeSync() {
  if (!timeSyncStarted_ || timeSyncReported_) return;
  uint32_t gpsTime;
  if (GnssScanner::networkGpsTime(gpsTime)) {
    DEBUG_PRINTF("[GNSS] Hora GPS sincronizada por la red: %lu s\n", (unsigned long)gpsTime);
    timeSyncReported_ = true;
  }
}

void GnssTracker::scanAndReport() {
  lastScanMs_ = millis();
  scanned_ = true;

  emit(SCAN_START);
  scanner_.scan(); // bloqueante (5-10 s)
  scanner_.printResult();

  const GnssResult &r = scanner_.result();
  // Pocos satelites = posicion con mucho error (multitrayecto/interferencia): no se envia el NAV
  const bool enough = r.valid && r.satellites >= config_.minSatellites;
  navLen_ = (enough && config_.navFport) ? scanner_.buildNavUplink(navBuf_, sizeof(navBuf_)) : 0;
  if (r.valid && r.satellites > 0 && !enough) {
    DEBUG_PRINTF("[GNSS] NAV no enviado: %u satelites (minimo %u)\n", r.satellites, config_.minSatellites);
  }
  emit(enough ? SCAN_NAV : SCAN_EMPTY); // el NAV sale en el siguiente hueco

  if (config_.dopplerFport) {
    sendDopplerReport();
  } else if (config_.statsFport) {
    lora_.addDigital(4, r.satellites);                           // digital_in_4: satelites detectados
    lora_.addAnalog(5, r.bestCnr);                               // analog_in_5: mejor C/N0 (dB)
    lora_.addAnalog(6, r.valid ? r.scanTimeMs / 1000.0f : -1.0f); // analog_in_6: duracion (s), -1 = fallo
    lora_.send(config_.statsFport);
  }
}

void GnssTracker::sendDopplerReport() {
  const GnssResult &r = scanner_.result();
  const uint8_t total = r.valid ? r.satellites : 0;

  uint8_t buf[DOPPLER_HEADER_SIZE + DOPPLER_SAT_SIZE * GNSS_MAX_SATELLITES];
  buf[0] = (DOPPLER_VERSION << 4) | (r.realTime ? 0x01 : 0) | (r.assisted ? 0x02 : 0) | (r.almanacOld ? 0x04 : 0);
  buf[1] = r.gpsTime >> 24;
  buf[2] = r.gpsTime >> 16;
  buf[3] = r.gpsTime >> 8;
  buf[4] = r.gpsTime;
  buf[5] = r.valid ? (uint8_t)min((r.scanTimeMs + 50) / 100, (uint32_t)255) : 255;
  buf[6] = total;

  // Solo los satelites que caben en el DR actual
  uint8_t maxPayload = 0;
  smtc_modem_get_next_tx_max_payload(MODEM_STACK_ID, &maxPayload);
  if (maxPayload == 0) maxPayload = DOPPLER_DEFAULT_MAX_PAYLOAD;
  const uint8_t fit = maxPayload > DOPPLER_HEADER_SIZE ? (maxPayload - DOPPLER_HEADER_SIZE) / DOPPLER_SAT_SIZE : 0;
  const uint8_t count = min(total, fit);

  // Orden por C/N0 de mayor a menor (insercion, a lo sumo 32 satelites)
  uint8_t order[GNSS_MAX_SATELLITES];
  for (uint8_t i = 0; i < total; i++) {
    uint8_t j = i;
    while (j > 0 && r.sats[order[j - 1]].cnr < r.sats[i].cnr) {
      order[j] = order[j - 1];
      j--;
    }
    order[j] = i;
  }

  uint8_t len = DOPPLER_HEADER_SIZE;
  for (uint8_t k = 0; k < count; k++) {
    const GnssSatellite &s = r.sats[order[k]];
    buf[len++] = s.id;
    buf[len++] = (uint8_t)s.cnr;
    buf[len++] = (uint16_t)s.doppler >> 8;
    buf[len++] = (uint16_t)s.doppler;
  }

  if (lora_.sendRaw(buf, len, config_.dopplerFport)) {
    DEBUG_PRINTF("[GNSS] Reporte doppler enviado por FPort %u (%u B, %u de %u satelites)\n", config_.dopplerFport,
                  len, count, total);
  } else {
    DEBUG_PRINTF("[GNSS] Reporte doppler no enviado por FPort %u (%u B): el modem lo rechazo\n", config_.dopplerFport,
                  len);
  }
}

void GnssTracker::sendPendingNav() {
  uint8_t maxPayload = 0;
  smtc_modem_get_next_tx_max_payload(MODEM_STACK_ID, &maxPayload);
  if (navLen_ > maxPayload) {
    DEBUG_PRINTF("[GNSS] NAV de %u B no cabe (max %u B en el DR actual): descartado\n", navLen_, maxPayload);
    emit(NAV_FAILED);
  } else if (lora_.sendRaw(navBuf_, navLen_, config_.navFport)) {
    DEBUG_PRINTF("[GNSS] NAV enviado por FPort %u (%u B)\n", config_.navFport, navLen_);
    emit(NAV_SENT);
  } else {
    emit(NAV_FAILED);
  }
  navLen_ = 0;
}
