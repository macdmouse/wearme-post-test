// Wearme POST Test
// Firmware de banco: inicializa la placa, corre el POST y muestra que el micro está vivo. NADA MÁS.
//   - Sin FSM, sin mediciones, sin Join LoRaWAN ni transmisiones, sin anillo, sin registro en la SD
//     (BOARD_POST_ONLY en lib/board_nrf52/board_nrf52.h).
//   - Las pruebas son las mismas de Wearme_Firmware_V3.1 (lib/board_nrf52, lib/post_lib).
//
// Señales de vida:
//   LED   un destello cada ALIVE_LED_PERIOD_MS: VERDE = POST sin fallas, ROJO = hay algún subsistema con falla.
//         Si deja de destellar (fijo o apagado) el micro está colgado; el WDT lo reinicia en WDT_TIMEOUT_S.
//   Serie línea [VIVO] cada 10 s: tiempo encendido, vueltas del loop por segundo, temperatura del nRF52840,
//         pila libre y resultado del POST (con los subsistemas que fallan). VIVO? la pide; VIVO:<s> cambia el periodo.
//   Tabla [TABLA] al arrancar, al cambiar las fallas, al terminar un POST:ALL y cada 60 s: cada subsistema con
//         su estado y, para los que fallan, QUÉ REVISAR en la placa; más la salud del micro (loop, pila,
//         temperatura, reinicios). TABLA? la pide; TABLA:<s> cambia el periodo.
// Comandos: TABLA?, POST:ALL, POST:<subsistema>, POLICY:..., RTC:SET, FAN/CH4, WDT, NODE?, BOOT?, DFU:... (lista al arrancar).

#include <Arduino.h>
#include "debug_config.h"
#include "board_nrf52.h"
#include "post_lib.h"
#include <nrf_sdm.h> // sd_softdevice_is_enabled
#include <nrf_soc.h> // sd_temp_get

static constexpr uint32_t ALIVE_LED_PERIOD_MS = 2000;   // destello de vida
static constexpr uint32_t ALIVE_DEFAULT_MS = 10000;      // línea [VIVO]
static constexpr uint32_t ALIVE_MAX_S = 3600;

static uint32_t alive_period_ms = ALIVE_DEFAULT_MS;
static uint32_t alive_last_ms = 0;
static uint32_t loops = 0;          // vueltas del loop en el segundo en curso
static uint32_t loops_per_s = 0;    // vueltas del último segundo completo
static uint32_t loops_t0 = 0;
static uint32_t loop_iter_ms = 0;     // millis() al empezar la vuelta en curso
static bool loops_window_blocked = false; // el último segundo medido tuvo una vuelta de más de 500 ms
static bool loops_blocking = false;       // el segundo en curso tiene una vuelta de más de 500 ms

// Temperatura del chip (sensor interno del nRF52840, resolución 0.25 °C): con el SoftDevice activo se pide a él
static float mcu_temp_c(void) {
  uint8_t sd = 0;
  sd_softdevice_is_enabled(&sd);
  int32_t raw = 0;
  if (sd) {
    if (sd_temp_get(&raw) != NRF_SUCCESS) return NAN;
  } else {
    NRF_TEMP->EVENTS_DATARDY = 0;
    NRF_TEMP->TASKS_START = 1;
    const uint32_t t0 = millis();
    while (!NRF_TEMP->EVENTS_DATARDY) {
      if (millis() - t0 > 10) return NAN;
    }
    NRF_TEMP->EVENTS_DATARDY = 0;
    raw = (int32_t)NRF_TEMP->TEMP;
    NRF_TEMP->TASKS_STOP = 1;
  }
  return raw * 0.25f;
}

static void print_alive(void) {
  const uint32_t s = millis() / 1000UL;
  char buf[96];
  snprintf(buf, sizeof(buf), "[VIVO] %lu:%02lu:%02lu encendido | loop %lu /s | CPU ", (unsigned long)(s / 3600UL),
           (unsigned long)((s / 60UL) % 60UL), (unsigned long)(s % 60UL), (unsigned long)loops_per_s);
  DEBUG_PRINT(buf);
  const float t = mcu_temp_c();
  if (isnan(t)) DEBUG_PRINT("?");
  else DEBUG_PRINT(t, 2);
  DEBUG_PRINT(" C | pila libre min ");
  DEBUG_PRINT(board_stack_free_min());
  DEBUG_PRINT(" B | ");

  const uint32_t mask = board_fault_mask();
  if (post_all_count() == 0) {
    DEBUG_PRINTLN("POST sin correr");
    return;
  }
  if (mask == 0) {
    DEBUG_PRINT("POST OK (");
    DEBUG_PRINT(post_all_count());
    DEBUG_PRINTLN(post_all_count() == 1 ? " corrida)" : " corridas)");
    return;
  }
  DEBUG_PRINT("FALLAS:");
  bool first = true;
  for (uint8_t code = 0; code < 32; code++) {
    if (!(mask & (1UL << code))) continue;
    DEBUG_PRINT(first ? " " : ", ");
    DEBUG_PRINT(post_test_name(code));
    first = false;
  }
  DEBUG_PRINTLN(" (POST:<subsistema> para repetir una prueba)");
}

// VIVO? / VIVO:<segundos> (lo llama post_process de lib/post_lib)
void post_alive_command(const String &cmd) {
  if (cmd == "VIVO?") {
    print_alive();
    return;
  }
  const String arg = cmd.substring(5);
  const long sec = arg.toInt();
  if (sec < 0 || sec > (long)ALIVE_MAX_S || (sec == 0 && arg != "0")) {
    DEBUG_PRINT("[VIVO] Formato: VIVO? o VIVO:<segundos 0-");
    DEBUG_PRINT(ALIVE_MAX_S);
    DEBUG_PRINTLN("> (0 = sin linea periodica; el LED sigue destellando)");
    return;
  }
  alive_period_ms = (uint32_t)sec * 1000UL;
  alive_last_ms = millis();
  DEBUG_PRINT("[VIVO] Linea periodica ");
  if (sec) {
    DEBUG_PRINT("cada ");
    DEBUG_PRINT(sec);
    DEBUG_PRINTLN(" s");
  } else {
    DEBUG_PRINTLN("apagada");
  }
}

// ---------- Tabla de diagnóstico (TABLA?) ----------
// Subsistemas del POST (post_print_table en lib/post_lib) + salud del micro medida aquí en el loop. Se imprime al
// arrancar, cada vez que cambian las fallas o termina un POST completo, y cada table_period_ms.
static constexpr uint32_t TABLE_DEFAULT_MS = 60000;
static constexpr uint32_t LOOP_MIN_PER_S = 100;  // con delay(5) el loop da ~190 vueltas/s
static constexpr uint32_t STACK_MIN_FREE_B = 512; // de 4 KB de pila del loop
static constexpr float CPU_MAX_C = 70.0f;

static uint32_t table_period_ms = TABLE_DEFAULT_MS;
static uint32_t table_last_ms = 0;
static uint32_t table_shown_mask = 0;
static uint32_t table_shown_runs = 0;

// Una fila de la salud del micro: nombre, valor, OK/ALERTA y qué revisar (solo si hay alerta)
static bool health_row(const char *name, const char *value, bool ok, const char *check) {
  char row[64];
  snprintf(row, sizeof(row), "| %-18s | %-16s | %-6s |", name, value, ok ? "OK" : "ALERTA");
  DEBUG_PRINT(row);
  if (!ok) {
    DEBUG_PRINT(" -> ");
    DEBUG_PRINT(check);
  }
  DEBUG_PRINTLN("");
  return ok;
}

static uint8_t print_micro_health(void) {
  static const char RULE[] = "+--------------------+------------------+--------+";
  char v[24];
  uint8_t alerts = 0;
  DEBUG_PRINTLN("  MICRO (nRF52840), medido en el loop:");
  DEBUG_PRINTLN(RULE);
  DEBUG_PRINTLN("| Chequeo            | Valor            | Estado |");
  DEBUG_PRINTLN(RULE);

  // Una prueba (POST:...) bloquea el loop varios segundos: justo después la cuenta baja y no es una falla
  const bool blocked = loops_window_blocked || loops_blocking || millis() - loop_iter_ms > 500UL;
  if (loops_per_s == 0) {
    health_row("Vueltas del loop", "midiendo", true, "");
  } else if (blocked) {
    health_row("Vueltas del loop", "tras una prueba", true, "");
  } else {
    snprintf(v, sizeof(v), "%lu /s", (unsigned long)loops_per_s);
    alerts += !health_row("Vueltas del loop", v, loops_per_s >= LOOP_MIN_PER_S,
                          "algo bloquea el loop (prueba larga, bus I2C colgado); normal justo tras un POST:...");
  }

  const uint32_t stack = board_stack_free_min();
  if (stack == 0) {
    health_row("Pila libre minima", "sin medir", true, "");
  } else {
    snprintf(v, sizeof(v), "%lu B", (unsigned long)stack);
    alerts += !health_row("Pila libre minima", v, stack >= STACK_MIN_FREE_B,
                          "riesgo de desborde (4 KB): buffers locales grandes o recursion; HardFault/WDT probable");
  }

  const float t = mcu_temp_c();
  if (isnan(t)) {
    snprintf(v, sizeof(v), "sin lectura");
  } else {
    snprintf(v, sizeof(v), "%d.%02d C", (int)t, (int)(fabsf(t - (int)t) * 100.0f + 0.5f));
  }
  alerts += !health_row("Temperatura CPU", v, !isnan(t) && t <= CPU_MAX_C,
                        isnan(t) ? "sensor TEMP no responde (SoftDevice?)"
                                 : "placa caliente: corto en un riel o regulador; medir el consumo");

  const uint8_t rec = board_wdt_recoveries();
  snprintf(v, sizeof(v), rec ? "%u seguidos" : "normal", (unsigned)rec);
  const bool reboot_ok = rec == 0;
  alerts += !health_row("Reinicios por WDT", v, reboot_ok, "algo bloquea el loop siempre en el mismo punto");
  if (!reboot_ok) {
    DEBUG_PRINT("|   causa del ultimo: ");
    DEBUG_PRINTLN(board_wdt_last_cause());
  }

  health_row("Alimentacion USB", board_usb_powered() ? "VBUS presente" : "sin VBUS", true, "");
  DEBUG_PRINTLN(RULE);
  return alerts;
}

static void print_table(void) {
  const uint32_t s = millis() / 1000UL;
  char buf[96];
  snprintf(buf, sizeof(buf), "\n===== [TABLA] Diagnostico de la placa | %lu:%02lu:%02lu encendido | POST completos: %lu =====",
           (unsigned long)(s / 3600UL), (unsigned long)((s / 60UL) % 60UL), (unsigned long)(s % 60UL),
           (unsigned long)post_all_count());
  DEBUG_PRINTLN(buf);
  const uint8_t faults = post_print_table();
  DEBUG_PRINTLN("");
  const uint8_t alerts = print_micro_health();
  DEBUG_PRINT("  LED de vida: ");
  DEBUG_PRINT(board_fault_mask() ? "ROJO" : "VERDE");
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(faults);
  DEBUG_PRINT(" subsistema(s) con falla, ");
  DEBUG_PRINT(alerts);
  DEBUG_PRINTLN(" alerta(s) del micro | TABLA? la repite, TABLA:<s> cambia el periodo");
  DEBUG_PRINTLN("");

  table_last_ms = millis();
  table_shown_mask = board_fault_mask();
  table_shown_runs = post_all_count();
}

// TABLA? / TABLA:<segundos> (lo llama post_process de lib/post_lib)
void post_table_command(const String &cmd) {
  if (cmd == "TABLA?") {
    print_table();
    return;
  }
  const String arg = cmd.substring(6);
  const long sec = arg.toInt();
  if (sec < 0 || sec > (long)ALIVE_MAX_S || (sec == 0 && arg != "0")) {
    DEBUG_PRINT("[TABLA] Formato: TABLA? o TABLA:<segundos 0-");
    DEBUG_PRINT(ALIVE_MAX_S);
    DEBUG_PRINTLN("> (0 = solo cuando cambian las fallas o termina un POST:ALL)");
    return;
  }
  table_period_ms = (uint32_t)sec * 1000UL;
  table_last_ms = millis();
  DEBUG_PRINT("[TABLA] Tabla periodica ");
  if (sec) {
    DEBUG_PRINT("cada ");
    DEBUG_PRINT(sec);
    DEBUG_PRINTLN(" s (y al cambiar las fallas)");
  } else {
    DEBUG_PRINTLN("apagada (sale solo al cambiar las fallas o al terminar un POST:ALL)");
  }
}

// Tabla al cambiar las fallas (una prueba suelta que falla o vuelve a pasar, el chequeo del RTC en el loop),
// al terminar un POST completo, o cada table_period_ms
static void table_service(uint32_t now) {
  if (post_all_count() == 0) return;
  const bool changed = board_fault_mask() != table_shown_mask || post_all_count() != table_shown_runs;
  const bool due = table_period_ms && now - table_last_ms >= table_period_ms;
  if (changed || due) print_table();
}

// El POST deja el GPS encendido buscando fix; aquí nadie más lo usa
static void gps_off_if_on(void) {
  if (board_gps_powered()) board_gps_off();
}

// LED de vida al día con el resultado: tras cada POST completo (board_post_result cambia los LEDs) o si cambian las
// fallas (una prueba suelta que falla o vuelve a pasar)
static void alive_leds_service(void) {
  static uint32_t shown_runs = 0;
  static uint32_t shown_mask = 0xFFFFFFFFUL;
  if (post_all_count() == 0) return;
  const uint32_t mask = board_fault_mask();
  if (mask == shown_mask && post_all_count() == shown_runs) return;
  shown_mask = mask;
  shown_runs = post_all_count();
  board_alive_leds(mask == 0, ALIVE_LED_PERIOD_MS);
}

void setup() {
  debug_init(); // puerto serie (espera el USB hasta 3 s)
  DEBUG_PRINTLN("=====================================");
  DEBUG_PRINTLN("  WEARME POST TEST (solo prueba de placa)");
  DEBUG_PRINT("  Firmware " WEARME_FW_VERSION " | compilado " __DATE__ " " __TIME__ "\n");
  char id[24];
  snprintf(id, sizeof(id), "%08lX%08lX", (unsigned long)NRF_FICR->DEVICEID[1], (unsigned long)NRF_FICR->DEVICEID[0]);
  DEBUG_PRINT("  nRF52840 DEVICEID ");
  DEBUG_PRINTLN(id);
  DEBUG_PRINTLN("  Sin Join LoRaWAN, sin anillo, sin mediciones");
  DEBUG_PRINTLN("=====================================");

  board_init(); // la misma inicialización de la placa (con BOARD_POST_ONLY)
  post();       // POST completo: prueba todo y resume al final
  gps_off_if_on();
  alive_leds_service();
  print_table();
  print_alive();
  loops_t0 = alive_last_ms = millis();
}

void loop() {
  const uint32_t iter_t0 = millis();
  if (loop_iter_ms && iter_t0 - loop_iter_ms > 500UL) loops_blocking = true; // la vuelta anterior tardó
  loop_iter_ms = iter_t0;
  board_alive_process(); // LEDs, WDT, pila, verificación del RTC
  post_process();        // comandos por serie (POST:..., VIVO?, ...)
  gps_off_if_on();       // tras un POST:GPS o POST:ALL
  alive_leds_service();

  loops++;
  const uint32_t now = millis();
  if (now - loops_t0 >= 1000UL) {
    loops_per_s = loops;
    loops = 0;
    loops_window_blocked = loops_blocking || now - loop_iter_ms > 500UL;
    loops_blocking = false;
    loops_t0 = now;
  }
  if (alive_period_ms && now - alive_last_ms >= alive_period_ms) {
    alive_last_ms = now;
    print_alive();
  }
  table_service(now);
  // Estable WDT_HEALTHY_MS: el contador de reinicios por watchdog vuelve a 0 (como en el firmware principal)
  static bool healthy = false;
  if (!healthy && now > WDT_HEALTHY_MS) {
    healthy = true;
    board_wdt_healthy();
  }
  delay(5);
}
