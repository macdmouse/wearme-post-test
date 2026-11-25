#pragma once

// =====================================================================================
// lora_lib — Nodo LoRaWAN (OTAA, clase A) para Wio Tracker / WM1110 sobre LoRa Basics Modem
//
// Encapsula todo lo que el radio necesita para funcionar de forma estable:
//   - polling de la IRQ del LR1110 y del timer de LBM (el HAL no usa interrupciones)
//   - configuracion de region, join OTAA y manejo de eventos del modem
//   - payload Cayenne LPP y envio de uplinks
//
// Uso minimo:
//
//   LoraNode lora;
//
//   void setup() {
//     LoraConfig cfg;
//     cfg.devEui = DEV_EUI; cfg.joinEui = JOIN_EUI; cfg.appKey = APP_KEY;
//     lora.begin(cfg);
//   }
//
//   void loop() {
//     lora.process();                       // SIEMPRE, en cada vuelta de loop()
//     if (lora.readyToSend(30000)) {        // unido a la red y pasaron 30 s
//       lora.addTemperature(2, 25.4);
//       lora.send();                        // FPort 1 por defecto
//     }
//   }
//
// IMPORTANTE: loop() no debe bloquearse (nada de delay() largos). La ventana de recepcion
// RX1/RX2 depende de que process() se ejecute al menos cada pocos milisegundos (< 8 ms).
// Solo se admite una instancia de LoraNode.
// =====================================================================================

#include <Arduino.h>
// LbmWm1110.hpp primero: LBM_T1000E trae los mismos headers lbm/... y el LDF de PlatformIO
// podria resolver hacia esa libreria (que no compila para esta placa).
#include <LbmWm1110.hpp>
#include <lbm/smtc_modem_api/smtc_modem_api.h>
#include <CayenneLPP.h>

struct LoraConfig {
  // Credenciales OTAA (obligatorias): 8, 8 y 16 bytes
  const uint8_t *devEui = nullptr;
  const uint8_t *joinEui = nullptr;
  const uint8_t *appKey = nullptr;

  smtc_modem_region_t region = SMTC_MODEM_REGION_AU_915;
  // Sub-banda de 8 canales (AU915/US915). 0 = no configurar (regiones sin sub-bandas).
  uint8_t subBand = 2;
  // Reduccion de potencia de TX. -10 dB porque el nodo de pruebas esta junto al gateway.
  int8_t txPowerOffsetDb = -10;
  // Margen extra de la ventana RX para absorber la latencia del polling del HAL.
  // A 5 s de RX1 delay, 1000 ppm abre la ventana ~10 ms antes y la alarga ~20 ms.
  uint32_t crystalErrorPpm = 1000;

  bool debug = true;     // mensajes [DBG] por Serial (join, cada envio, downlinks)
  bool debugHal = false; // ademas, estadisticas del HAL en cada envio
};

class LoraNode {
public:
  // Se llama cuando llega un downlink con datos de aplicacion.
  typedef void (*DownlinkCallback)(uint8_t fport, const uint8_t *data, uint8_t length, int16_t rssiDbm,
                                   float snrDb);

  LoraNode();

  // Inicializa el hardware y el modem. El join arranca solo en las primeras llamadas a process().
  bool begin(const LoraConfig &config);

  // Atiende radio, timer y eventos. Llamar en CADA vuelta de loop().
  void process();

  bool isJoined() const { return joined_; }

  // Milisegundos que LBM puede quedar sin process() antes de su próxima tarea (lo que devolvió el
  // motor en la última vuelta, menos lo que ya pasó). 0 = ocupado (TX/RX en curso o eventos pendientes):
  // no dormir. Sirve para dormir la CPU sin romper las ventanas RX (process() cada < 8 ms en TX/RX).
  uint32_t idleMs() const;

  // true (una vez) cuando la red esta unida y pasaron periodMs desde el ultimo envio (cualquier uplink aceptado por
  // sendRaw() cuenta). El primer envio tras el join es inmediato.
  bool readyToSend(uint32_t periodMs);

  // ---- Payload Cayenne LPP (el canal identifica cada variable en el decodificador) ----
  void clear();
  bool addDigital(uint8_t channel, uint8_t value);
  bool addAnalog(uint8_t channel, float value);
  bool addTemperature(uint8_t channel, float celsius);
  bool addHumidity(uint8_t channel, float percent);
  bool addGps(uint8_t channel, float latitude, float longitude, float altitudeM);
  CayenneLPP &payload() { return lpp_; } // acceso directo para otros tipos de CayenneLPP
  uint8_t payloadSize() { return lpp_.getSize(); }

  // Envia el payload Cayenne acumulado y lo limpia. false si no hay join o el modem esta ocupado.
  bool send(uint8_t fport = 1, bool confirmed = false);
  // Envia bytes arbitrarios (sin Cayenne).
  bool sendRaw(const uint8_t *data, uint8_t length, uint8_t fport = 1, bool confirmed = false);

  void onDownlink(DownlinkCallback callback) { downlinkCallback_ = callback; }

  // ---- Estadisticas desde el arranque ----
  uint32_t joinTimeMs() const { return joinTimeMs_; }
  uint32_t joinFailures() const { return joinFailures_; }
  uint32_t uplinks() const { return uplinks_; }
  uint32_t txDone() const { return txDone_; }
  uint32_t acks() const { return acks_; }        // uplinks confirmados con ACK
  uint32_t txNotSent() const { return txNotSent_; } // uplinks que LBM no transmitió (TXDONE NOT_SENT)
  uint32_t downlinks() const { return downlinks_; }
  uint32_t linkAdrCount() const { return linkAdrCount_; }

private:
  enum class State { Idle, NotJoined, Joining, Joined };

  void configureAndJoin();
  void processEvents();
  void onJoined();
  void printHalStats();

  static void modemEventCallback();
  static LoraNode *instance_;

  LoraConfig config_;
  State state_;
  bool joined_;
  volatile bool eventPending_;
  CayenneLPP lpp_;
  DownlinkCallback downlinkCallback_;

  uint32_t joinStartMs_;
  uint32_t lastJoinCheckMs_;
  uint32_t lastSendMs_;
  bool firstSendPending_;
  uint32_t maxDoWorkMs_;
  uint32_t engineSleepMs_ = 0; // lo que devolvió smtc_modem_run_engine() en la última vuelta
  uint32_t engineAtMs_ = 0;

  uint32_t joinTimeMs_;
  uint32_t joinFailures_;
  uint32_t uplinks_;
  uint32_t txDone_;
  uint32_t acks_ = 0;
  uint32_t txNotSent_;
  uint32_t downlinks_;
  uint32_t linkAdrCount_;
};
