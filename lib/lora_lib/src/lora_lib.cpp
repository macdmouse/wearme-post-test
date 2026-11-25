#include "lora_lib.h"
#include "debug_config.h"

// printf con el mismo interruptor que DEBUG_PRINT/DEBUG_PRINTLN (Debug_Wearme en debug_config.h)
#ifndef DEBUG_PRINTF
#define DEBUG_PRINTF(...) do { if (Debug_Wearme) { DEBUG_PORT.printf(__VA_ARGS__); } } while (0)
#endif

#include <LbmWm1110.hpp>
#include <Lbmx.hpp>
#include <stdarg.h>

// Funciones del HAL modificado (LBM_WM1110/src/internal/lbm_hal/smtc_modem_hal.cpp).
// El HAL no usa interrupciones: la IRQ del LR1110 y el timer de LBM se atienden por polling.
extern "C" void smtc_modem_hal_poll_radio_irq(void);
extern "C" void smtc_modem_hal_poll_timer(void);
// Diagnostico (internas de LBM): DR/frecuencia del proximo TX, DevAddr y estrategia de DR
extern "C" uint8_t lorawan_api_next_dr_get(void);
extern "C" uint32_t lorawan_api_next_frequency_get(void);
extern "C" uint32_t lorawan_api_devaddr_get(void);
extern "C" int lorawan_api_dr_strategy_get(void);
extern "C" void smtc_modem_hal_get_poll_stats(uint32_t *irq_delivered, uint32_t *irq_ignored,
                                              uint32_t *max_gap_ms);

// Los traces de LBM (MODEM_HAL_DBG_TRACE en platformio.ini) salen por el USB serial.
// Requiere -Wl,--wrap=smtc_modem_hal_print_trace en build_flags.
extern "C" void __wrap_smtc_modem_hal_print_trace(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  char buf[256];
  vsnprintf(buf, sizeof(buf), fmt, args);
  DEBUG_PRINT(buf);
  va_end(args);
}

#define LORA_DBG(fmt, ...)                                                                                    \
  do {                                                                                                        \
    if (config_.debug) DEBUG_PRINTF("[DBG] " fmt "\n", ##__VA_ARGS__);                                         \
  } while (0)

static constexpr uint8_t STACK_ID = 0;
static constexpr uint8_t LPP_MAX_SIZE = 51; // payload maximo en AU915 DR2

LoraNode *LoraNode::instance_ = nullptr;

LoraNode::LoraNode()
    : state_(State::Idle), joined_(false), eventPending_(false), lpp_(LPP_MAX_SIZE), downlinkCallback_(nullptr),
      joinStartMs_(0), lastJoinCheckMs_(0), lastSendMs_(0), firstSendPending_(false), maxDoWorkMs_(0),
      joinTimeMs_(0), joinFailures_(0), uplinks_(0), txDone_(0), txNotSent_(0), downlinks_(0), linkAdrCount_(0) {}

bool LoraNode::begin(const LoraConfig &config) {
  if (config.devEui == nullptr || config.joinEui == nullptr || config.appKey == nullptr) return false;

  config_ = config;
  instance_ = this;

  const char *regionName = (config_.region == SMTC_MODEM_REGION_AU_915) ? "AU915" : "region";
  LORA_DBG("Inicio LoRaWAN | %s sub-banda %u | OTAA", regionName, config_.subBand);

  LbmWm1110 &board = LbmWm1110::getInstance();
  board.begin();
  LbmxEngine::begin(board.getRadio(), &LoraNode::modemEventCallback);

  // Mismo orden de inicializacion que la version probada del sketch
  pinMode(4, INPUT_PULLDOWN); // DIO1
  pinMode(43, INPUT);         // PIN_LR1110_BUSY

  // La configuracion y el join se hacen en process(), despues de la primera vuelta del motor.
  state_ = State::NotJoined;
  return true;
}

void LoraNode::process() {
  if (state_ == State::Idle) return;

  // Orden importante: IRQ del radio -> timer -> motor LBM.
  smtc_modem_hal_poll_radio_irq();
  smtc_modem_hal_poll_timer();

  const uint32_t tWork = millis();
  engineSleepMs_ = LbmxEngine::doWork(); // ms hasta la próxima tarea del modem
  engineAtMs_ = millis();
  if (engineAtMs_ - tWork > maxDoWorkMs_) maxDoWorkMs_ = engineAtMs_ - tWork;

  if (eventPending_) {
    eventPending_ = false;
    processEvents();
  }

  switch (state_) {
  case State::NotJoined:
    configureAndJoin();
    break;

  case State::Joining:
    // Respaldo por si el evento JOINED no llegara: consultar el estado cada segundo
    if (millis() - lastJoinCheckMs_ > 1000) {
      lastJoinCheckMs_ = millis();
      smtc_modem_status_mask_t status = 0;
      smtc_modem_get_status(STACK_ID, &status);
      if (status & SMTC_MODEM_STATUS_JOINED) onJoined();
    }
    break;

  default:
    break;
  }
}

uint32_t LoraNode::idleMs() const {
  if (state_ == State::Idle) return UINT32_MAX;                       // sin begin(): no hay modem que atender
  if (eventPending_ || state_ == State::NotJoined) return 0;          // trabajo pendiente ya
  // TX/RX en curso: el HAL detecta el fin de TX leyendo la IRQ por SPI en process(); dormir corre las
  // ventanas RX. Solo IDLE (nada en curso) o TX_WAIT (esperando el próximo intento) permiten dormir.
  smtc_modem_stack_state_t stack = SMTC_MODEM_STACK_STATE_BUSY;
  if (smtc_modem_get_stack_state(STACK_ID, &stack) != SMTC_MODEM_RC_OK || stack == SMTC_MODEM_STACK_STATE_BUSY) {
    return 0;
  }
  const uint32_t elapsed = millis() - engineAtMs_;
  return engineSleepMs_ > elapsed ? engineSleepMs_ - elapsed : 0;
}

void LoraNode::configureAndJoin() {
  smtc_modem_set_region(STACK_ID, config_.region);
  if (config_.subBand != 0) smtc_modem_set_region_sub_band(STACK_ID, config_.subBand);
  smtc_modem_set_network_type(STACK_ID, true); // red publica (TTN)
  smtc_modem_set_tx_power_offset_db(STACK_ID, config_.txPowerOffsetDb);
  smtc_modem_set_crystal_error_ppm(config_.crystalErrorPpm);
  // Sin reportes periodicos de Device Management (FPort 199): solo trafico de la app
  smtc_modem_dm_set_info_interval(SMTC_MODEM_DM_INFO_INTERVAL_IN_SECOND, 0);
  // El perfil ADR se deja en NETWORK_CONTROLLED (defecto): un perfil custom provoca un bucle de
  // LinkADRReq con TTN. El data rate se fija desde TTN (ADR estatico).

  LbmxEngine::setOTAA(config_.devEui, config_.joinEui, config_.appKey);
  LORA_DBG("Join solicitado");
  joinStartMs_ = millis();
  LbmxEngine::joinNetwork();
  state_ = State::Joining;
}

void LoraNode::onJoined() {
  if (joined_) return;
  joined_ = true;
  state_ = State::Joined;
  joinTimeMs_ = millis() - joinStartMs_;
  firstSendPending_ = true;
  LORA_DBG("Join OK en %lu ms (%lu fallidos)", (unsigned long)joinTimeMs_, (unsigned long)joinFailures_);
}

void LoraNode::modemEventCallback() {
  if (instance_ != nullptr) instance_->eventPending_ = true;
}

void LoraNode::processEvents() {
  smtc_modem_event_t event;
  uint8_t pending = 0;
  do {
    if (smtc_modem_get_event(&event, &pending) != SMTC_MODEM_RC_OK) return;

    switch (event.event_type) {
    case SMTC_MODEM_EVENT_JOINED:
      onJoined();
      break;

    case SMTC_MODEM_EVENT_JOINFAIL:
      joinFailures_++;
      LORA_DBG("Join fallido #%lu (Join Accept no recibido)", (unsigned long)joinFailures_);
      printHalStats();
      break;

    case SMTC_MODEM_EVENT_TXDONE:
      // NOT_SENT = LBM no transmitio (p. ej. payload mas grande que el permitido por el DR actual)
      if (event.event_data.txdone.status == SMTC_MODEM_EVENT_TXDONE_NOT_SENT) {
        txNotSent_++;
        LORA_DBG("TX no enviado por LBM (NOT_SENT #%lu)", (unsigned long)txNotSent_);
      } else if (event.event_data.txdone.status == SMTC_MODEM_EVENT_TXDONE_CONFIRMED) {
        txDone_++;
        acks_++;
        LORA_DBG("TXDONE confirmado: ACK recibido de la red");
      } else {
        txDone_++;
        LORA_DBG("TXDONE ok | DR%u %lu Hz | DevAddr %08lX | estrategia DR %d", lorawan_api_next_dr_get(),
                 (unsigned long)lorawan_api_next_frequency_get(), (unsigned long)lorawan_api_devaddr_get(),
                 lorawan_api_dr_strategy_get());
      }
      break;

    case SMTC_MODEM_EVENT_DOWNDATA: {
      downlinks_++;
      const int16_t rssi = event.event_data.downdata.rssi - 64; // dBm
      const int8_t snrQ = event.event_data.downdata.snr;        // pasos de 0.25 dB
      const float snr = snrQ / 4.0f;
      // Sin %f: el printf del core (newlib-nano) no formatea floats
      LORA_DBG("Downlink RX%u | FPort %u | %u B | RSSI %d dBm | SNR %d.%02d dB", event.event_data.downdata.window,
               event.event_data.downdata.fport, event.event_data.downdata.length, rssi, snrQ / 4,
               abs(snrQ % 4) * 25);
      if (downlinkCallback_ != nullptr) {
        downlinkCallback_(event.event_data.downdata.fport, event.event_data.downdata.data,
                          (uint8_t)event.event_data.downdata.length, rssi, snr);
      }
      break;
    }

    case SMTC_MODEM_EVENT_NEW_LINK_ADR:
      // Llega cuando el dispositivo recibe el LinkADRReq de TTN: prueba de que el downlink funciona.
      linkAdrCount_++;
      LORA_DBG("LinkADRReq recibido y aplicado");
      break;

    default:
      break;
    }
  } while (pending > 0);
}

bool LoraNode::readyToSend(uint32_t periodMs) {
  if (!joined_) return false;
  if (firstSendPending_ || millis() - lastSendMs_ >= periodMs) {
    firstSendPending_ = false;
    lastSendMs_ = millis();
    return true;
  }
  return false;
}

void LoraNode::clear() { lpp_.reset(); }

bool LoraNode::addDigital(uint8_t channel, uint8_t value) { return lpp_.addDigitalInput(channel, value) != 0; }

bool LoraNode::addAnalog(uint8_t channel, float value) { return lpp_.addAnalogInput(channel, value) != 0; }

bool LoraNode::addTemperature(uint8_t channel, float celsius) {
  return lpp_.addTemperature(channel, celsius) != 0;
}

bool LoraNode::addHumidity(uint8_t channel, float percent) {
  return lpp_.addRelativeHumidity(channel, percent) != 0;
}

bool LoraNode::addGps(uint8_t channel, float latitude, float longitude, float altitudeM) {
  return lpp_.addGPS(channel, latitude, longitude, altitudeM) != 0;
}

bool LoraNode::send(uint8_t fport, bool confirmed) {
  const bool ok = sendRaw(lpp_.getBuffer(), lpp_.getSize(), fport, confirmed);
  lpp_.reset();
  return ok;
}

bool LoraNode::sendRaw(const uint8_t *data, uint8_t length, uint8_t fport, bool confirmed) {
  if (!joined_) {
    LORA_DBG("Envio descartado: aun no hay join");
    return false;
  }

  // Tras el Join LBM arranca en DR2/SF10 (AU915: maximo 11 B) hasta que TTN manda el LinkADRReq
  // (DR3). Un payload mas grande queda NOT_SENT y LBM no vuelve a completar uplinks de la app; en
  // su lugar se manda un uplink vacio (solo MAC) para que TTN responda con el LinkADRReq.
  uint8_t maxPayload = 0;
  if (smtc_modem_get_next_tx_max_payload(STACK_ID, &maxPayload) == SMTC_MODEM_RC_OK && length > maxPayload) {
    const smtc_modem_return_code_t rcEmpty = smtc_modem_request_empty_uplink(STACK_ID, false, 0, false);
    LORA_DBG("Payload de %u B no cabe en el DR actual (max %u): uplink vacio para pedir ADR rc=%d", length,
             maxPayload, rcEmpty);
    return false;
  }
  const smtc_modem_return_code_t rc = smtc_modem_request_uplink(STACK_ID, fport, confirmed, data, length);
  if (rc == SMTC_MODEM_RC_OK) {
    uplinks_++;
    // Cualquier uplink es señal de vida: readyToSend() cuenta el período desde aquí (con mediciones frecuentes no
    // sale un bit de vida aparte)
    lastSendMs_ = millis();
    firstSendPending_ = false;
  }

  // tx_ok/dl/adr son acumulados; tx_ok va uno atras (LBM confirma el TX tras cerrar RX2)
  LORA_DBG("Uplink #%lu | FPort %u | %u B (max %u) rc=%d | tx_ok=%lu no_env=%lu dl=%lu adr=%lu",
           (unsigned long)uplinks_, fport, length, maxPayload, rc, (unsigned long)txDone_,
           (unsigned long)txNotSent_, (unsigned long)downlinks_, (unsigned long)linkAdrCount_);
  printHalStats();
  return rc == SMTC_MODEM_RC_OK;
}

void LoraNode::printHalStats() {
  if (!config_.debug || !config_.debugHal) return;
  uint32_t delivered = 0, ignored = 0, maxGap = 0;
  smtc_modem_hal_get_poll_stats(&delivered, &ignored, &maxGap);
  DEBUG_PRINTF("[DBG] HAL | irq_ok=%lu irq_ign=%lu max_loop=%lu ms max_work=%lu ms\n", (unsigned long)delivered,
                (unsigned long)ignored, (unsigned long)maxGap, (unsigned long)maxDoWorkMs_);
  maxDoWorkMs_ = 0;
}
