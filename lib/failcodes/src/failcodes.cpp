#include "failcodes.h"

#include <debug_config.h>
#include <post_lib.h>

// =====================================================================================
// TABLAS EDITABLES
// LedCode = {LEDs, destellos, ms encendido, ms apagado}. LEDs: G = verde, R = rojo, B = azul (se suman).
// =====================================================================================
static constexpr uint8_t G = LED_MASK_GREEN;
static constexpr uint8_t R = LED_MASK_RED;
static constexpr uint8_t B = LED_MASK_BLUE;

// ---- 1. Eventos (mismo orden que LedEvent) ----
static const LedCode EVENTS[LED_EV_COUNT] = {
    /* LED_EV_POST_OK      */ {G, 3, 100, 150},
    /* LED_EV_STATE_CHANGE */ {G, 2, 60, 120},
    /* LED_EV_LORA_SEARCH  */ {B, 1, 30, 0},
    /* LED_EV_LORA_JOINED  */ {B, 3, 80, 150},
    /* LED_EV_LORA_TX      */ {B, 1, 80, 0},
    /* LED_EV_LORA_ACK     */ {B, 1, 400, 0},
    /* LED_EV_LORA_RX      */ {B, 2, 80, 150},
};

// ---- 2. Modos (mismo orden que LedMode): {patrón, periodo en ms} ----
static const LedModeCfg MODES[LED_MODE_COUNT] = {
    /* LED_MODE_OFF     */ {{0, 0, 0, 0}, 0},
    /* LED_MODE_IDLE    */ {{G, 1, 50, 0}, 5000},
    /* LED_MODE_MEASURE */ {{G, 1, 50, 0}, 1000},
    /* LED_MODE_SURVIVE */ {{R, 1, 50, 0}, 10000},
    /* LED_MODE_FAULT   */ {{0, 0, 0, 0}, FAULT_CODE_PERIOD_MS}, // los códigos salen de FAIL_CODES
};

// ---- 3. Códigos de falla: el orden es el de la alternancia cuando hay varias fallas ----
// Destellos de 250 ms con 350 ms de pausa: se cuentan a simple vista.
static const FailCode FAIL_CODES[] = {
    // Base de hardware: cian
    {POST_I2C_BUS,       {G | B, 1, 250, 350}},
    {POST_IO_EXPANDER,   {G | B, 2, 250, 350}},
    {POST_POWER_RAILS,   {G | B, 3, 250, 350}},
    {POST_WATCHDOG,      {G | B, 4, 250, 350}},
    // Energía, hora y almacenamiento: verde
    {POST_BATTERY,       {G, 1, 250, 350}},
    {POST_RTC,           {G, 2, 250, 350}},
    {POST_SD_CARD,       {G, 3, 250, 350}},
    // Sensores y actuadores: azul
    {POST_IMU,           {B, 1, 250, 350}},
    {POST_ENVIRONMENT,   {B, 2, 250, 350}},
    {POST_SENSOR_CH4_H2, {B, 3, 250, 350}},
    {POST_BLOWER,        {B, 4, 250, 350}},
    {POST_GPS,           {B, 5, 250, 350}},
    {POST_WIND,          {B, 6, 250, 350}},
    // Comunicaciones: cian largo
    {POST_LORA_COMM,     {G | B, 5, 250, 350}},
    {POST_BLUETOOTH,     {G | B, 6, 250, 350}},
};
static constexpr uint8_t FAIL_CODE_COUNT = sizeof(FAIL_CODES) / sizeof(FAIL_CODES[0]);

// =====================================================================================

const LedCode &failcodes_event(LedEvent ev) { return EVENTS[ev < LED_EV_COUNT ? ev : 0]; }

const LedModeCfg &failcodes_mode(LedMode mode) { return MODES[mode < LED_MODE_COUNT ? mode : 0]; }

const FailCode *failcodes_find(uint8_t postCode) {
  for (const FailCode &f : FAIL_CODES) {
    if (f.postCode == postCode) return &f;
  }
  return nullptr;
}

const FailCode *failcodes_next(uint32_t faultMask, uint8_t &cursor) {
  for (uint8_t i = 0; i < FAIL_CODE_COUNT; i++) {
    const FailCode &f = FAIL_CODES[(cursor + i) % FAIL_CODE_COUNT];
    if (f.postCode < 32 && (faultMask & (1UL << f.postCode))) {
      cursor = (uint8_t)((cursor + i + 1) % FAIL_CODE_COUNT);
      return &f;
    }
  }
  return nullptr;
}

const char *failcodes_describe(const LedCode &code, char *out, size_t size) {
  static const char *const NAMES[8] = {"apagado", "verde", "rojo", "amarillo", "azul", "cian", "magenta", "blanco"};
  snprintf(out, size, "%s x%u", NAMES[code.leds & 0x07], code.blinks);
  return out;
}

void failcodes_print(void) {
  char led[24];
  DEBUG_PRINTLN("[LED] Codigos de falla (en FAULT: rojo fijo + este codigo cada 4 s):");
  for (const FailCode &f : FAIL_CODES) {
    DEBUG_PRINT("[LED]   ");
    DEBUG_PRINT(failcodes_describe(f.code, led, sizeof(led)));
    DEBUG_PRINT(" = ");
    DEBUG_PRINTLN(post_test_name(f.postCode));
  }
}
