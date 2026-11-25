#pragma once

// =====================================================================================
// failcodes — QUÉ significa cada patrón de LED de la placa Wearme. Las tablas están al inicio de
// failcodes.cpp: para cambiar un color, un número de destellos o un periodo solo se edita esa tabla.
//
// Convención: verde = máquina de estados, azul = radio LoRa, rojo = problemas.
//   1. Eventos:  un patrón cada vez que algo ocurre (uplink, ACK, downlink, Join, cambio de estado...).
//   2. Modos:    un patrón que se repite mientras la FSM está en un estado (IDLE, midiendo, SURVIVE).
//   3. Fallas:   en FAULT el rojo queda fijo y cada FAULT_CODE_PERIOD_MS se muestra el código del
//                siguiente subsistema con falla (verde / azul / cian y número de destellos), recorriendo
//                la tabla en orden. Así, sin computador, se sabe qué revisar.
// El mecanismo (destellos sin bloquear) está en lib/led_lib.
// =====================================================================================

#include <Arduino.h>
#include <led_lib.h>

// ---- 1. Eventos ----
enum LedEvent : uint8_t {
  LED_EV_POST_OK,      // el POST completo pasó
  LED_EV_STATE_CHANGE, // cambio de estado de la FSM
  LED_EV_LORA_SEARCH,  // buscando red (se repite cada LED_LORA_SEARCH_PERIOD_MS mientras no hay Join)
  LED_EV_LORA_JOINED,  // Join OK
  LED_EV_LORA_TX,      // uplink enviado
  LED_EV_LORA_ACK,     // ACK de un uplink confirmado
  LED_EV_LORA_RX,      // downlink con datos (comando) recibido
  LED_EV_COUNT
};
#define LED_LORA_SEARCH_PERIOD_MS 3000UL

// ---- 2. Modos (patrón periódico por estado de la FSM) ----
enum LedMode : uint8_t {
  LED_MODE_OFF,     // sin patrón periódico
  LED_MODE_IDLE,    // esperando un comando
  LED_MODE_MEASURE, // PREPARE / MEASURE / STORE_SEND
  LED_MODE_SURVIVE, // batería crítica
  LED_MODE_FAULT,   // rojo fijo + códigos de falla (tabla 3)
  LED_MODE_COUNT
};
struct LedModeCfg {
  LedCode code;
  uint32_t periodMs; // 0 = sin patrón
};

// ---- 3. Códigos de falla por subsistema (código POST) ----
#define FAULT_RED_STEADY       1        // 1 = rojo fijo en FAULT (0 = solo los códigos)
#define FAULT_CODE_PERIOD_MS   4000UL   // un código cada 4 s; con varias fallas se alternan
struct FailCode {
  uint8_t postCode; // POST_IMU, POST_BLOWER... (post_lib.h)
  LedCode code;     // no usar el rojo: en FAULT ya está fijo
};

const LedCode &failcodes_event(LedEvent ev);
const LedModeCfg &failcodes_mode(LedMode mode);
const FailCode *failcodes_find(uint8_t postCode);
// Siguiente subsistema de la tabla que está en faultMask (bit = código POST), empezando en `cursor`
// y avanzándolo; nullptr si ninguno de la tabla está en la máscara.
const FailCode *failcodes_next(uint32_t faultMask, uint8_t &cursor);
// "cian x3" en out (para los mensajes por serial y el registro de fallas)
const char *failcodes_describe(const LedCode &code, char *out, size_t size);
void failcodes_print(void); // tabla de códigos de falla por serial
