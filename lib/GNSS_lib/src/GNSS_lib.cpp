#include "GNSS_lib.h"
#include "debug_config.h"

// printf con el mismo interruptor que DEBUG_PRINT/DEBUG_PRINTLN (Debug_Wearme en debug_config.h)
#ifndef DEBUG_PRINTF
#define DEBUG_PRINTF(...) do { if (Debug_Wearme) { DEBUG_PORT.printf(__VA_ARGS__); } } while (0)
#endif
#include <lbm/smtc_modem_api/smtc_modem_api.h>
#include <string.h>

extern "C" {
#include <lbm/smtc_modem_core/smtc_ralf/src/ralf.h>
#include <lbm/smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_gnss.h>
#include <lbm/smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_system.h>
#include <mw/geolocation_middleware/common/mw_common.h>
}

// LNA de la antena GNSS (P1.05). El variant lo deja apagado; se enciende solo durante el escaneo.
#ifndef PIN_GNSS_LNA
#define PIN_GNSS_LNA 37
#endif

static constexpr uint32_t GNSS_LNA_SETTLE_MS = 5;    // arranque del LNA antes de escanear
static constexpr uint32_t IRQ_POLL_INTERVAL_MS = 10;
static constexpr uint32_t GNSS_BUSY_EXTRA_WAIT_MS = 10000; // espera extra si el escaneo se alarga
static constexpr uint8_t ASSIST_SKIP_ALMANAC_OLD = 10; // escaneos en autonomo si el almanaque esta vencido
static constexpr uint8_t MODEM_STACK_ID = 0;

#ifndef PIN_LR1110_BUSY
#define PIN_LR1110_BUSY 43
#endif

GnssScanner::GnssScanner(Constellation constellations, uint32_t timeoutMs)
    : constellations_(constellations), timeoutMs_(timeoutMs), maxSatellites_(11), versionPrinted_(false), hasPosition_(false),
      latitude_(0), longitude_(0), assistSkip_(0), token_(0) {}

void GnssScanner::setAssistancePosition(float latitude, float longitude) {
  latitude_ = latitude;
  longitude_ = longitude;
  hasPosition_ = true;
}

bool GnssScanner::startTimeSync() {
  // MAC_SYNC = comando LoRaWAN DeviceTimeReq (TTN lo responde en el siguiente downlink).
  // Si ya hay join, LBM agenda la solicitud de inmediato y luego resincroniza periodicamente.
  return smtc_modem_time_start_sync_service(MODEM_STACK_ID, SMTC_MODEM_TIME_MAC_SYNC) == SMTC_MODEM_RC_OK;
}

bool GnssScanner::networkGpsTime(uint32_t &gpsSeconds) {
  uint32_t frac = 0;
  return smtc_modem_get_time(&gpsSeconds, &frac) == SMTC_MODEM_RC_OK && gpsSeconds != 0;
}

uint8_t GnssScanner::buildNavUplink(uint8_t *out, uint8_t maxLen) {
  if (!result_.valid || result_.navSize < 2 || result_.nav[0] != LR11XX_GNSS_DESTINATION_SOLVER) return 0;
  const uint16_t len = result_.navSize; // byte de destino reemplazado por la cabecera
  if (len > maxLen) return 0;

  // Cada escaneo es un grupo de 1 NAV: token nuevo (2..31, 0 y 1 reservados) y bit "ultimo" = 1
  token_ = (token_ < 2 || token_ >= 0x1F) ? (uint8_t)random(2, 0x20) : (uint8_t)(token_ + 1);
  out[0] = 0x80 | (token_ & 0x1F);
  memcpy(out + 1, result_.nav + 1, len - 1);
  return (uint8_t)len;
}

bool GnssScanner::scan() {
  result_ = GnssResult();

  // Pausar el modem LoRaWAN: reserva el radio para el usuario. Falla si hay una TX/RX en curso.
  if (smtc_modem_suspend_before_user_radio_access() != SMTC_MODEM_RC_OK) {
    result_.error = "modem LoRaWAN ocupado";
    return false;
  }

  // Hora: la de la red si ya se sincronizo; si no, la de compilacion (solo sirve para autonomo)
  result_.realTime = networkGpsTime(result_.gpsTime);
  if (!result_.realTime) result_.gpsTime = approxGpsTimeSeconds();
  result_.assisted = result_.realTime && hasPosition_ && assistSkip_ == 0;
  if (assistSkip_ > 0) assistSkip_--;

  const void *radio = LbmWm1110::getInstance().getRadio()->ral.context;
  const uint32_t t0 = millis();
  bool ok = runScan(radio);

  // Almanaque vencido: el LR1110 aborta el asistido en ~0.3 s. Se repite en autonomo en el
  // mismo hueco (el radio sigue reservado) y se deja de intentar asistido por un rato.
  if (result_.assisted && result_.navSize >= 2 && result_.nav[0] == LR11XX_GNSS_DESTINATION_HOST &&
      result_.nav[1] == LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD) {
    const bool realTime = result_.realTime;
    const uint32_t gpsTime = result_.gpsTime;
    result_ = GnssResult();
    result_.realTime = realTime;
    result_.gpsTime = gpsTime;
    result_.almanacOld = true;
    assistSkip_ = ASSIST_SKIP_ALMANAC_OLD;
    ok = runScan(radio);
  } else if (result_.assisted && (!ok || result_.satellites == 0)) {
    assistSkip_ = 1; // el siguiente escaneo sera autonomo
  }
  result_.scanTimeMs = millis() - t0;

  // Dejar el radio como lo espera LBM: IRQ del escaneo limpia y en sleep
  lr11xx_system_clear_irq_status(radio, LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE);
  mw_radio_set_sleep(radio);
  smtc_modem_resume_after_user_radio_access();

  result_.valid = ok;
  return ok;
}

bool GnssScanner::runScan(const void *radio) {
  if (!versionPrinted_) {
    lr11xx_gnss_version_t version;
    if (lr11xx_gnss_read_firmware_version(radio, &version) == LR11XX_STATUS_OK) {
      DEBUG_PRINTF("[GNSS] Firmware GNSS 0x%02X | formato almanaque 0x%02X\n", version.gnss_firmware,
                    version.gnss_almanac);
    }
    versionPrinted_ = true;
  }

  // Misma preparacion que el middleware de Semtech: limpiar errores y reloj LF por cristal
  if (!mw_radio_configure_for_scan(radio)) {
    result_.error = "configuracion del radio para GNSS";
    return false;
  }

  lr11xx_system_clear_irq_status(radio, LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE);

  if (lr11xx_gnss_set_scan_mode(radio, LR11XX_GNSS_SCAN_MODE_3_SINGLE_SCAN_AND_5_FAST_SCANS) != LR11XX_STATUS_OK ||
      lr11xx_gnss_set_constellations_to_use(radio, (lr11xx_gnss_constellation_mask_t)constellations_) !=
          LR11XX_STATUS_OK) {
    result_.error = "parametros del escaneo";
    return false;
  }

  digitalWrite(PIN_GNSS_LNA, HIGH);
  delay(GNSS_LNA_SETTLE_MS);

  if (!startScan(radio)) {
    digitalWrite(PIN_GNSS_LNA, LOW);
    result_.error = "inicio del escaneo";
    return false;
  }

  // Durante el escaneo el LR1110 mantiene BUSY en alto (segundos). No se le puede hablar por
  // SPI hasta que baje: el HAL solo espera 100 ms y luego transfiere igual, lo que devuelve
  // basura (0xFF...) y deja el radio en mal estado. Por eso se vigila el pin BUSY directamente
  // y solo entonces se lee el registro de IRQ.
  bool done = false;
  bool busyReleased = false;
  const uint32_t tStart = millis();
  while (millis() - tStart < timeoutMs_) {
    delay(IRQ_POLL_INTERVAL_MS);
    if (digitalRead(PIN_LR1110_BUSY) == HIGH) continue;
    busyReleased = true;
    lr11xx_system_irq_mask_t irq = LR11XX_SYSTEM_IRQ_NONE;
    if (lr11xx_system_get_irq_status(radio, &irq) == LR11XX_STATUS_OK && (irq & LR11XX_SYSTEM_IRQ_GNSS_SCAN_DONE)) {
      done = true;
      break;
    }
  }
  digitalWrite(PIN_GNSS_LNA, LOW);

  if (!done) {
    // Si el radio sigue ocupado no se le puede mandar nada: esperar a que termine antes de
    // devolverlo a LBM (un comando con BUSY alto corrompe el estado del LR1110).
    const uint32_t tExtra = millis();
    while (digitalRead(PIN_LR1110_BUSY) == HIGH && millis() - tExtra < GNSS_BUSY_EXTRA_WAIT_MS) delay(10);
    result_.error = busyReleased ? "timeout esperando GNSS_SCAN_DONE" : "timeout: radio ocupado (BUSY)";
    return false;
  }

  readResults(radio);
  return true;
}

bool GnssScanner::startScan(const void *radio) {
  if (result_.assisted) {
    // Asistido: posicion aproximada + hora real -> el LR1110 calcula con el almanaque que
    // satelites estan arriba y su doppler esperado. Ventana de busqueda de 250 Hz (Semtech).
    lr11xx_gnss_solver_assistance_position_t pos;
    pos.latitude = latitude_;
    pos.longitude = longitude_;
    if (lr11xx_gnss_set_assistance_position(radio, &pos) != LR11XX_STATUS_OK ||
        lr11xx_gnss_set_freq_search_space(radio, LR11XX_GNSS_FREQUENCY_SEARCH_SPACE_250_HZ) != LR11XX_STATUS_OK) {
      return false;
    }
    return lr11xx_gnss_scan_assisted(radio, result_.gpsTime, LR11XX_GNSS_OPTION_BEST_EFFORT,
                                     LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK, maxSatellites_) == LR11XX_STATUS_OK;
  }
  // Autonomo: no necesita almanaque ni posicion aproximada. La fecha solo marca el NAV.
  return lr11xx_gnss_scan_autonomous(radio, result_.gpsTime, LR11XX_GNSS_OPTION_BEST_EFFORT,
                                     LR11XX_GNSS_RESULTS_DOPPLER_ENABLE_MASK, maxSatellites_) == LR11XX_STATUS_OK;
}

void GnssScanner::readResults(const void *radio) {
  lr11xx_gnss_timings_t timings;
  if (lr11xx_gnss_get_timings(radio, &timings) == LR11XX_STATUS_OK) {
    result_.radioMs = timings.radio_ms;
    result_.computationMs = timings.computation_ms;
  }

  uint8_t nb = 0;
  if (lr11xx_gnss_get_nb_detected_satellites(radio, &nb) == LR11XX_STATUS_OK && nb > 0) {
    if (nb > GNSS_MAX_SATELLITES) nb = GNSS_MAX_SATELLITES;
    lr11xx_gnss_detected_satellite_t detected[GNSS_MAX_SATELLITES];
    if (lr11xx_gnss_get_detected_satellites(radio, nb, detected) == LR11XX_STATUS_OK) {
      int cnrSum = 0;
      result_.bestCnr = detected[0].cnr;
      for (uint8_t i = 0; i < nb; i++) {
        result_.sats[i].id = detected[i].satellite_id;
        result_.sats[i].cnr = detected[i].cnr;
        result_.sats[i].doppler = detected[i].doppler;
        if (result_.sats[i].isBeidou()) result_.beidou++; else result_.gps++;
        if (detected[i].cnr > result_.bestCnr) result_.bestCnr = detected[i].cnr;
        cnrSum += detected[i].cnr;
      }
      result_.satellites = nb;
      result_.avgCnr = (int8_t)(cnrSum / nb);
    }
  }

  // Mensaje NAV: primer byte = destino (0x01 = solver en la nube, 0x00 = mensaje de error al host)
  uint16_t size = 0;
  if (lr11xx_gnss_get_result_size(radio, &size) == LR11XX_STATUS_OK && size > 0) {
    if (size > GNSS_MAX_NAV_SIZE) size = GNSS_MAX_NAV_SIZE;
    if (lr11xx_gnss_read_results(radio, result_.nav, size) == LR11XX_STATUS_OK) result_.navSize = size;
  }
}

void GnssScanner::printResult() const {
  DEBUG_PRINTF("[GNSS] Modo %s | hora GPS %lu (%s)\n",result_.assisted     ? "ASISTIDO"
                : result_.almanacOld ? "autonomo (asistido descartado: almanaque vencido)"
                                     : "autonomo",
                (unsigned long)result_.gpsTime, result_.realTime ? "red LoRaWAN" : "compilacion, sin sync");
  if (!result_.valid) {
    DEBUG_PRINTF("[GNSS] Escaneo fallido: %s (%lu ms)\n", result_.error ? result_.error : "?",
                  (unsigned long)result_.scanTimeMs);
    return;
  }

  DEBUG_PRINTF("[GNSS] Escaneo OK en %lu ms (radio %lu ms, calculo %lu ms) | %u satelites (GPS %u, BeiDou %u)\n",
                (unsigned long)result_.scanTimeMs, (unsigned long)result_.radioMs,
                (unsigned long)result_.computationMs, result_.satellites, result_.gps, result_.beidou);

  if (result_.satellites > 0) {
    DEBUG_PRINTF("[GNSS] C/N0 mejor %d dB | promedio %d dB\n", result_.bestCnr, result_.avgCnr);
    for (uint8_t i = 0; i < result_.satellites; i++) {
      const GnssSatellite &s = result_.sats[i];
      // id = numeracion interna del LR1110 (GPS 0-31, BeiDou 64-127)
      DEBUG_PRINTF("[GNSS]   %s id %3u | C/N0 %2d dB | doppler %6d Hz\n", s.isBeidou() ? "BDS" : "GPS", s.id,
                    s.cnr, s.doppler);
    }
  } else {
    DEBUG_PRINTLN("[GNSS] Sin satelites: revisar cielo abierto / antena GNSS");
  }

  if (result_.navSize == 0) return;

  if (result_.nav[0] == LR11XX_GNSS_DESTINATION_HOST && result_.navSize >= 2) {
    // Mensaje de estado del LR1110 para el host (no es un NAV)
    DEBUG_PRINTF("[GNSS] Estado LR1110: 0x%02X %s\n", result_.nav[1], hostStatusText(result_.nav[1]));
    return;
  }

  const char *dest = result_.nav[0] == LR11XX_GNSS_DESTINATION_SOLVER ? "solver en la nube" : "otro";
  DEBUG_PRINTF("[GNSS] NAV %u B (destino %s): ", result_.navSize, dest);
  for (uint16_t i = 0; i < result_.navSize; i++) DEBUG_PRINTF("%02X", result_.nav[i]);
  DEBUG_PRINTLN();
}

const char *GnssScanner::hostStatusText(uint8_t code) {
  switch (code) {
  case LR11XX_GNSS_HOST_OK: return "OK";
  case LR11XX_GNSS_HOST_NO_TIME: return "sin hora GPS";
  case LR11XX_GNSS_HOST_NO_SATELLITE_DETECTED: return "ningun satelite detectado (cielo abierto?)";
  case LR11XX_GNSS_HOST_ALMANAC_IN_FLASH_TOO_OLD: return "almanaque demasiado viejo";
  case LR11XX_GNSS_HOST_NOT_ENOUGH_SV_DETECTED_TO_BUILD_A_NAV_MESSAGE: return "pocos satelites para un NAV";
  case LR11XX_GNSS_HOST_IQ_CAPTURE_FAILS: return "fallo captura IQ (antena/LNA?)";
  case LR11XX_GNSS_HOST_INVALID_PARAMETERS: return "parametros invalidos";
  default: return "";
  }
}

// Hora GPS aproximada a partir de la fecha de compilacion (respaldo mientras no hay hora de red).
// Para el escaneo autonomo solo sirve de marca de tiempo del mensaje NAV.
uint32_t GnssScanner::approxGpsTimeSeconds() {
  static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *date = __DATE__; // "Sep 30 2026"
  const char *time = __TIME__; // "10:48:17"

  int month = 1;
  for (int m = 0; m < 12; m++) {
    if (strncmp(months + m * 3, date, 3) == 0) {
      month = m + 1;
      break;
    }
  }
  int day = atoi(date + 4);
  int year = atoi(date + 7);

  // Dias desde 1970-01-01 (algoritmo de Howard Hinnant)
  year -= month <= 2;
  const int era = year / 400;
  const int yoe = year - era * 400;
  const int doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const long days = (long)era * 146097 + doe - 719468;

  const long unixSeconds = days * 86400L + atoi(time) * 3600L + atoi(time + 3) * 60L + atoi(time + 6);
  // Epoch GPS = 1980-01-06 (315964800 s Unix) + 18 s de leap seconds
  return (uint32_t)(unixSeconds - 315964800L + 18L + millis() / 1000);
}
