// board_nrf52.cpp
// Descripción: Configuración e inicialización de la placa Wearme (nRF52840 + LR1110 en el módulo WM1110).
//              Capa de hardware (estado seguro de salidas, LEDs, bus I2C con TCA4307, expansor
//              TCA6408A, interruptores de carga SIP32431 y microSD), módulos de aplicación (nodo
//              LoRaWAN y GNSS, habilitables en board_nrf52.h) y el ciclo del loop().

#include "board_nrf52.h"
#include <SPI.h>
#include <Wire.h>
#include <lora_lib.h>
#include <GnssTracker.h>
#include <led_lib.h>
#include <tca6408.h>
#include <max17048.h>
#include <lsm6ds3.h>
#include <rx8130.h>
#include <bme280.h>
#include <ads1219.h>
#include <mcp4018.h>
#include <fan53741.h>
#include <MethanePredictor.h> // lib/methane_predictor: gramos emitidos a partir de la ppm
#include <ch4_mos.h>          // lib/ch4_mos: ppm del GM-402B como Rs/R0 (línea base dinámica)
#include <nmea_gps.h>
#include <post_lib.h>
#include <energy_supervisor.h>
#include <tca9543a.h>
#include <fs3000.h>
#include <nrfx_wdt.h>
#include <sd_card.h>
#include <ble_lib.h>
#include <mesh_lib.h>                              // anillo Meshtastic de respaldo
#include <ring_diag.h>                             // diagnóstico del anillo (señal por nodo, rutas, sondeo)
#include <lora_frames.h>                           // formato del comando por el anillo (FPort 21)
#include <InternalFileSystem.h>                    // último fix del GPS en la flash interna
using LfsFile = Adafruit_LittleFS_Namespace::File; // File solo es ambiguo con el de SdFat
#include <nrf_sdm.h>                               // sd_softdevice_is_enabled
#include <nrf_soc.h>                               // sd_power_system_off

extern "C" {
#include <lbm/smtc_modem_core/smtc_ralf/src/ralf.h>
#include <lbm/smtc_modem_core/radio_drivers/lr11xx_driver/src/lr11xx_system.h>
}

// ============================================
// 1. Credenciales OTAA (TTN: LoRaWAN 1.0.4 / RP002-1.0.3, ADR estatico DR3)
// ============================================
// Nodo a programar (include/wearme_node.h) y claves de todos los nodos (include/wearme_credentials.h, fuera de git)
#include <wearme_node.h>
#include <wearme_credentials.h>

static constexpr int WEARME_CRED_COUNT = (int)(sizeof(WEARME_CREDENTIALS) / sizeof(WEARME_CREDENTIALS[0]));
static constexpr int wearme_cred_index(int i) {
  return i >= WEARME_CRED_COUNT ? -1 : (WEARME_CREDENTIALS[i].node == WEARME_NODE ? i : wearme_cred_index(i + 1));
}
static constexpr int FW_CRED = wearme_cred_index(0);
static_assert(FW_CRED >= 0, "WEARME_NODE (include/wearme_node.h) no esta en include/wearme_credentials.h");
static_assert(WEARME_NODE >= 1 && WEARME_NODE <= 999, "WEARME_NODE debe estar entre 1 y 999");

#if BOARD_ENABLE_LORA
// ============================================
// Credenciales del FIRMWARE = las del nodo WEARME_NODE en la tabla. En el PRIMER arranque de un collar (sin
// /lorawan.bin en la flash) se graban en la flash y se verifican; desde ahí mandan las de la flash y NO se
// sobreescriben, aunque se cargue un firmware compilado para otro nodo (lo avisa al arrancar). Por serie: LORA:CRED?
// (ver), NODE:SET=<nodo> (cambiar de nodo con la tabla), LORA:CRED:RESET / LORA:CRED:SET (a mano).
static const uint8_t *const FW_DEV_EUI = WEARME_CREDENTIALS[FW_CRED].devEui;
static const uint8_t *const FW_JOIN_EUI = WEARME_CREDENTIALS[FW_CRED].joinEui;
static const uint8_t *const FW_APP_KEY = WEARME_CREDENTIALS[FW_CRED].appKey;
// Credenciales EN USO (las de la flash; las del firmware si no se pudieron leer ni grabar). Las llena cred_load().
static uint8_t DEV_EUI[8], JOIN_EUI[8], APP_KEY[16];
#endif

// ============================================
// 2. Interruptores de carga SIP32431 (EN activo en ALTO) controlados por el TCA6408A
// ============================================
// Según el esquemático WearMe V12 (U56 TCA6408ARSVR, hoja WIO 1110). Los bits que no estén en la
// tabla se dejan como entrada (alta impedancia), sin tocar.
struct PowerRail {
  const char *name;
  uint8_t expanderBit;
  bool onAtBoot;
};

static const PowerRail POWER_RAILS[] = {
  // P0 CTRL_GPS -> tarjeta RF (FPC2): interruptor U59 del GPS L96-M33 (~25 mA). Diseño final SIN R154: lo enciende
  // el firmware. Si una tarjeta trae R154 (0 Ω a VDD, venía de fábrica en la primera) el pin no se maneja: ver
  // rails_fixed_mask.
  {"GPS", 0, true},
  {"CH4", 1, false}, // P1 CTRL_CH4   -> tarjeta RF (FPC2)
  // IMU encendido al arrancar: el LSM6DS3TR-C está en SDA_2_OUT/SCL_2_OUT y apagado lleva el bus a
  // bajo por sus diodos de protección, así el TCA4307 no conecta (MAX17048 y RX8130CE no responden).
  {"IMU", 2, true},       // P2 CTRL_IMU   -> U80 -> POWER_IMU (LSM6DS3TR-C, 0.9 mA)
  {"VEL", 3, false},      // P3 CTRL_VEL   -> U114 -> POWER_VEL (tarjetas de batería y sensor de velocidad)
  {"POT", 5, false},      // P5 CTRL_POT   -> U82 -> POWER_POT (AD5941)
  {"ESP32", 6, false},    // P6 CTRL_ESP32 -> U90 -> POWER_ESP32 (ESP32-S3)
  {"H2", 7, true},        // P7 CTRL_H2    -> tarjeta RF (FPC2): sensor GM-602B con alimentación fija (siempre ON)
  {nullptr, 0xFF, false}, // fin de la tabla
};

// ============================================
// 3. Módulos de la placa
// ============================================
static StatusLeds leds;
static Tca6408 expander;
static Max17048 fuelGauge;     // medidor de bateria (I2C 0x36)
static Lsm6ds3 imu;            // LSM6DS3TR-C (I2C2 0x6A, POWER_IMU)
static Rx8130 rtc;             // RX8130CE (I2C2 0x32)
static Bme280 env;             // BME280 de la tarjeta MEMS (I2C2 0x76)
static Ads1219 adc;            // ADS1219 de la tarjeta MEMS (I2C2 0x41): AIN0 = VOUT_H2, AIN1 = VOUT_CH4
static Mcp4018 ch4Load;        // MCP4018T-502E (I2C2 0x2F): resistencia de carga del sensor CH4
static Ch4Mos ch4mos;          // Rs, línea base R0 y ppm del GM-402B
static Fan53741 heater;        // FAN53741 (I2C2 0x52): buck del calefactor del CH4, VIN = 5V_CH4
static bool ch4_on = false;    // sensor CH4 encendido (HT_PWM + 5V_CH4 + FAN53741), ver board_ch4_set()
static uint32_t ch4_on_ms = 0; // millis() al encender el calefactor
// Predictor de metano (lib/methane_predictor): un MethaneState por medición, se integra solo en MEASURE
static MethaneState ch4_state;
static bool methane_active = false;
static void print_ch4_status(uint32_t now);
static const char *rail_log_prefix = "[INIT]   "; // "[PWR] " después de board_init()
static constexpr uint8_t ADS1219_ADDR = 0x41;     // A0 a 3.3 V (R85), A1 a GND (R88)
static constexpr uint32_t MCP4018_RAB_OHMS = 5000;
static Rx8130::DateTime rtc_boot_time; // hora leída en el arranque, para verificar que el reloj avanza
static uint32_t rtc_boot_ms = 0;
static bool rtc_boot_valid = false;
// Hora UTC confiable: el RTC no perdió la hora al arrancar, o se puso en hora (red LoRaWAN, GPS, serie/BLE). Las
// ventanas del anillo Meshtastic solo se abren con hora confiable (si no, el collar no coincide con los demás).
static bool rtc_trusted = false; // time_source != TIME_NONE
// Calidad de la hora (como Meshtastic: una fuente peor nunca pisa a una mejor)
enum TimeSource : uint8_t { TIME_NONE,
                            TIME_MESH,
                            TIME_RTC,
                            TIME_MANUAL,
                            TIME_GPS,
                            TIME_NET };
static TimeSource time_source = TIME_NONE;
static uint32_t unix_base = 0, unix_base_ms = 0; // caché de board_unix_now() (se borra al poner el RTC en hora)
static void time_source_set(TimeSource s);       // más abajo (guarda en la flash que el RTC tiene hora real)
static MicroSd sd(SPI, WM_PIN_SD_CS);            // SPI del variant en SPIM2 (SPIM3 es del LR1110)
static bool i2c1_ready = false;                  // Wire  (TWIM0): TCA6408A
static bool i2c2_ready = false;                  // Wire1 (TWIM1): TCA4307
static const char *fuel_gauge_bus = nullptr;     // bus donde respondió el MAX17048
static bool board_initialized = false;

#if BOARD_ENABLE_LORA
static LoraNode lora;
#endif
static BoardSnapshot snap;
static volatile bool downlink_pending = false; // downlink (o comando serie) esperando a la FSM
static uint8_t downlink_fport = 0;
static uint8_t downlink_len = 0;
static uint8_t downlink_data[BOARD_DOWNLINK_MAX];
static bool downlink_ble = false; // el comando pendiente vino de la app por BLE
static bool taken_ble = false;    // el último comando entregado a la FSM vino por BLE
static bool power_save = false;   // IDLE/FAULT: sensores en reposo y sin muestreo
static bool survive_mode = false; // SURVIVE: además todos los rieles cortados
static EnergySupervisor energy;   // RICH / CORE / SURVIVAL
static uint32_t fault_mask = 0;   // bit = código POST con falla (hasta reiniciar)

// Todos los campos de la foto en "sin dato" (NAN), excepto el contador y el blower
static void snapshot_reset(void) {
  snap.tick = 0;
  float *fields[] = {&snap.batV, &snap.batSoc, &snap.batCrate, &snap.crateFiltered, &snap.energyMode, &snap.tempC, &snap.humidity, &snap.pressureHpa, &snap.ax, &snap.ay,
                     &snap.az, &snap.gx, &snap.gy, &snap.gz, &snap.imuTempC, &snap.h2V, &snap.ch4V, &snap.h2Ppm,
                     &snap.ch4Ppm, &snap.ch4RsOhm, &snap.ch4R0Ohm, &snap.ch4Ratio, &snap.ch4Warm, &snap.ch4Raw, &snap.ch4TotalG, &snap.ch4RateGDay, &snap.ch4Event, &snap.gpsFix, &snap.gpsSats, &snap.gpsLat,
                     &snap.gpsLon, &snap.gpsAltM, &snap.windRight, &snap.windLeft, &snap.gpsAgeS, &snap.rssiDbm, &snap.snrDb};
  for (float *f : fields) *f = NAN;
  snap.fanRpm = 0;
}

// ---- GPS Quectel L96-M33 (UART en Serial1) ----
static NmeaGps gps;
static volatile uint32_t gps_edges_a = 0; // flancos en P0.14 antes de abrir la UART
static volatile uint32_t gps_edges_b = 0; // flancos en P1.09
static void gps_isr_a(void) {
  gps_edges_a++;
}
static void gps_isr_b(void) {
  gps_edges_b++;
}
static uint8_t gps_rx_pin = BOARD_PIN_NC; // pin donde llega la TX del GPS (detectado)
static bool gps_watching = false;
// Respaldo: último fix válido (RAM + flash interna). Si en una medición no hay fix se informa este, con su
// edad en gpsAgeS. Edad: si es de este arranque, por millis(); si vino de la flash, la edad al arrancar
// (RTC ahora - hora UTC del fix guardada) más el tiempo desde el arranque.
static struct {
  bool valid;
  bool thisBoot; // fix de este arranque (ms vale); si no, vino de la flash
  bool ageKnown; // fix de la flash con hora: bootAgeS vale
  float lat, lon, alt;
  uint32_t ms;       // millis() del fix (thisBoot)
  uint32_t bootAgeS; // edad al arrancar (fix de la flash)
} last_fix = {false, false, false, 0, 0, 0, 0, 0};
static bool gps_fix_this_wake = false; // hubo fix desde el último despertar (board_power_save(false))
// Alimentación (CTRL_GPS): sesión = desde que se enciende hasta que se apaga
static bool gps_powered = false;
static uint32_t gps_on_ms = 0;          // inicio de la sesión
static uint32_t gps_session_fix_ms = 0; // primer fix válido de la sesión (0 = buscando)
static bool gps_session_done = false;   // ya pasó GPS_HOLD_AFTER_FIX_MS tras el fix
static bool gps_save_pending = false;   // guardar last_fix en la flash cuando el radio esté libre
static uint32_t gps_saved_ms = 0;       // último guardado (0 = ninguno en este arranque)
static uint32_t gps_last_gga_ms = 0;    // updatedMs de la última GGA procesada
#if BOARD_ENABLE_GNSS
static GnssTracker gnss(lora);
#endif

static void print_hex8(uint8_t value) {
  DEBUG_PRINT("0x");
  if (value < 0x10) DEBUG_PRINT('0');
  DEBUG_PRINT(value, HEX);
}

// ============================================
// 4. Capa de hardware
// ============================================
// Salidas de potencia en estado seguro. El core pone OUT = 1 en todos los pines al arrancar
// (siguen como entradas hasta el pinMode), así que se escribe el nivel ANTES de pinMode.
static void safe_gpio_init(void) {
  digitalWrite(WM_PIN_HT_PWM, LOW); // calefactor apagado
  pinMode(WM_PIN_HT_PWM, OUTPUT);
  digitalWrite(WM_PIN_FAN_CTL, LOW); // ventilador apagado
  pinMode(WM_PIN_FAN_CTL, OUTPUT);
  digitalWrite(WM_PIN_LNA_CTL, LOW); // filtro/LNA GNSS apagado
  pinMode(WM_PIN_LNA_CTL, OUTPUT);
  digitalWrite(WM_PIN_CS_POT, HIGH); // potenciómetro digital sin seleccionar (antes lo hacía el variant del Dev Kit)
  pinMode(WM_PIN_CS_POT, OUTPUT);
  digitalWrite(WM_PIN_SD_CS, HIGH); // microSD sin seleccionar
  pinMode(WM_PIN_SD_CS, OUTPUT);
}

// Devuelve cuántos dispositivos respondieron
static uint8_t i2c_scan(TwoWire &bus, const char *prefix, const char *name) {
  DEBUG_PRINT(prefix);
  DEBUG_PRINT(name);
  DEBUG_PRINT(" encontrados:");
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      DEBUG_PRINT(' ');
      print_hex8(addr);
      if (addr == Tca6408::ADDR_LOW || addr == Tca6408::ADDR_HIGH) DEBUG_PRINT("(TCA6408A)");
      if (addr == Max17048::I2C_ADDR) DEBUG_PRINT("(MAX17048)");
      if (addr == 0x32) DEBUG_PRINT("(RX8130CE)");
      if (addr == 0x6A || addr == 0x6B) DEBUG_PRINT("(LSM6DS3TR-C)");
      if (addr >= 0x70 && addr <= 0x73) DEBUG_PRINT("(TCA9543A)");
      // Tarjeta MEMS (por la tarjeta RF / FPC2)
      if (addr == 0x2F) DEBUG_PRINT("(MCP4018)");
      if (addr == 0x41) DEBUG_PRINT("(ADS1219)");
      if (addr == Fan53741::I2C_ADDR) DEBUG_PRINT("(FAN53741)");
      if (addr == 0x76) DEBUG_PRINT("(BME280)");
      found++;
    }
  }
  if (found == 0) DEBUG_PRINT(" ninguno");
  DEBUG_PRINTLN("");
  return found;
}

// Con el bus libre SDA y SCL deben estar en alto (pull-ups). Si una línea está en bajo el bus no se
// inicia: el TWIM del core espera sin límite de tiempo si un dispositivo retiene SCL.
static bool i2c_bus_begin(TwoWire &bus, uint8_t sda, uint8_t scl, const char *name) {
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
  delayMicroseconds(10);
  const bool sdaLow = digitalRead(sda) == LOW;
  const bool sclLow = digitalRead(scl) == LOW;
  if (sdaLow || sclLow) {
    DEBUG_PRINT("[INIT]   ");
    DEBUG_PRINT(name);
    DEBUG_PRINT(sdaLow ? (sclLow ? ": SDA y SCL" : ": SDA") : ": SCL");
    DEBUG_PRINTLN(" en bajo (sin pull-up o bus atascado) -> bus no iniciado");
    return false;
  }
  bus.setPins(sda, scl);
  bus.begin();
  bus.setClock(100000);
  return true;
}

static void i2c_init(void) {
  i2c1_ready = i2c_bus_begin(Wire, PIN_WIRE_SDA, PIN_WIRE_SCL, "I2C1 (SDA P0.11, SCL P0.16)");
  if (i2c1_ready) i2c_scan(Wire, "[INIT]   ", "I2C1");

  if (WM_PIN_I2C_BUF_EN != BOARD_PIN_NC) {
    digitalWrite(WM_PIN_I2C_BUF_EN, HIGH);
    pinMode(WM_PIN_I2C_BUF_EN, OUTPUT);
  }
  i2c2_ready = i2c_bus_begin(Wire1, PIN_WIRE1_SDA, PIN_WIRE1_SCL, "I2C2 (SDA P1.03, SCL P1.05)");
  if (!i2c2_ready) return;

  // El TCA4307 conecta el lado de los dispositivos al ver el bus libre (READY en alto)
  if (WM_PIN_I2C_BUF_READY != BOARD_PIN_NC) {
    pinMode(WM_PIN_I2C_BUF_READY, INPUT);
    const uint32_t t0 = millis();
    while (digitalRead(WM_PIN_I2C_BUF_READY) == LOW && millis() - t0 < 100) delay(1);
    DEBUG_PRINTLN(digitalRead(WM_PIN_I2C_BUF_READY) == HIGH ? "[INIT]   TCA4307 READY" : "[INIT]   TCA4307 sin READY");
  }
  // El escaneo de I2C2 se hace después de encender los rieles (ver power_rails_init)
}

static void io_expander_init(void) {
  if (!i2c1_ready) {
    DEBUG_PRINTLN("[INIT]   Sin bus I2C1 -> TCA6408A omitido");
    return;
  }
  if (WM_PIN_EXP_RESET != BOARD_PIN_NC) {
    digitalWrite(WM_PIN_EXP_RESET, LOW);
    pinMode(WM_PIN_EXP_RESET, OUTPUT);
    delay(1);
    digitalWrite(WM_PIN_EXP_RESET, HIGH);
    delay(1);
  }

  // Se prueba la dirección configurada y la otra posición de ADDR
  if (!expander.begin(Wire, WM_EXP_I2C_ADDR) && !expander.begin(Wire, WM_EXP_I2C_ADDR ^ 0x01)) {
    DEBUG_PRINTLN("[INIT]   TCA6408A no responde en 0x20 ni 0x21");
    return;
  }
  uint8_t inputs = 0;
  expander.readInputs(inputs);
  DEBUG_PRINT("[INIT]   TCA6408A en ");
  print_hex8(expander.address());
  DEBUG_PRINT(" | entradas ");
  print_hex8(inputs);
  DEBUG_PRINT(" | salidas ");
  print_hex8(expander.outputs());
  DEBUG_PRINT(" | config ");
  print_hex8(expander.config());
  DEBUG_PRINTLN(" (1 = entrada)");
}

// Rieles que NO se manejan porque en esta tarjeta están unidos a VDD por hardware: CTRL_GPS (P0) con R154
// (WM_RF_R154_MOUNTED en board_pins.h). Manejarlo en BAJO cortocircuitaría VDD contra la salida del TCA6408A.
static uint8_t rails_fixed_mask = 0;
// Rieles que el POST encontró pegados a VDD sin estar declarados (pin pedido en BAJO que se lee ALTO: R154 montada o
// EN en corto a VDD). Desde ahí el pin queda como entrada (manejarlo en BAJO forzaría la salida del TCA6408A contra
// VDD) y el POST lo sigue marcando como falla hasta el próximo arranque
static uint8_t rails_stuck_mask = 0;
static inline uint8_t rails_nodrive_mask(void) { return rails_fixed_mask | rails_stuck_mask; }

static void rails_detect_fixed(void) {
#if WM_RF_R154_MOUNTED
  rails_fixed_mask = 1u << 0;
  expander.pinsInput(rails_fixed_mask); // alta impedancia: R154 lo deja en ALTO
  DEBUG_PRINTLN("[INIT]   WM_RF_R154_MOUNTED = 1: CTRL_GPS (P0) fijo a VDD, el pin no se maneja");
#else
  rails_fixed_mask = 0;
#endif
}

static bool rail_apply(const PowerRail &rail, bool on) {
  if (rails_nodrive_mask() & (1u << rail.expanderBit)) { // unido a VDD por hardware: siempre encendido
    DEBUG_PRINT(rail_log_prefix);
    DEBUG_PRINT("SIP32431 ");
    DEBUG_PRINT(rail.name);
    DEBUG_PRINTLN((rails_fixed_mask & (1u << rail.expanderBit)) ? " fijo a VDD (R154): sin control"
                                                                : " pegado a VDD (detectado en el POST): sin control");
    return on;
  }
  const bool ok = expander.present() && expander.pinOutput(rail.expanderBit, on);
  DEBUG_PRINT(rail_log_prefix);
  DEBUG_PRINT("SIP32431 ");
  DEBUG_PRINT(rail.name);
  DEBUG_PRINT(" (TCA6408A P");
  DEBUG_PRINT(rail.expanderBit);
  DEBUG_PRINT(") -> ");
  DEBUG_PRINTLN(!ok ? "ERROR I2C" : (on ? "ON" : "OFF"));
  return ok;
}

static void power_rails_init(void) {
  if (POWER_RAILS[0].name == nullptr) {
    DEBUG_PRINTLN("[INIT]   Tabla de SIP32431 vacia (completar POWER_RAILS en board_nrf52.cpp)");
    return;
  }
  if (!expander.present()) {
    DEBUG_PRINTLN("[INIT]   Sin TCA6408A -> SIP32431 sin control");
    return;
  }
  // El TCA6408A no se reinicia con el nRF (sin reset por hardware): conserva la configuración del
  // firmware anterior. Los pines que no están en la tabla se ponen como entrada de forma explícita.
  uint8_t tableMask = 0;
  for (const PowerRail *r = POWER_RAILS; r->name != nullptr; r++) tableMask |= 1u << r->expanderBit;
  if (!expander.pinsInput(~tableMask)) DEBUG_PRINTLN("[INIT]   TCA6408A: error al configurar entradas");

  rails_detect_fixed(); // WM_RF_R154_MOUNTED: CTRL_GPS fijo a VDD
  for (const PowerRail *r = POWER_RAILS; r->name != nullptr; r++) {
    rail_apply(*r, r->onAtBoot);
  }
  uint8_t inputs = 0;
  expander.readInputs(inputs);
  DEBUG_PRINT("[INIT]   TCA6408A final | entradas ");
  print_hex8(inputs);
  DEBUG_PRINT(" | salidas ");
  print_hex8(expander.outputs());
  DEBUG_PRINT(" | config ");
  print_hex8(expander.config());
  DEBUG_PRINTLN(" (1 = entrada)");

  // Con los rieles encendidos los chips de SDA_2_OUT/SCL_2_OUT liberan el bus y el TCA4307 conecta.
  // 20 ms: arranque del SIP32431 y del LSM6DS3TR-C (boot ~10 ms) + detección de bus libre del TCA4307.
  if (i2c2_ready) {
    delay(20);
    i2c_scan(Wire1, "[INIT]   ", "I2C2 (detras del TCA4307)");
  }
}

bool board_rail_set(const char *name, bool on) {
  for (const PowerRail *r = POWER_RAILS; r->name != nullptr; r++) {
    if (strcmp(r->name, name) == 0) return rail_apply(*r, on);
  }
  DEBUG_PRINT("[PWR] SIP32431 desconocido: ");
  DEBUG_PRINTLN(name);
  return false;
}

// Voltaje, SOC, tasa y alertas del MAX17048. false si no responde o el voltaje no es de una celda Li+.
// Sin batería el MAX17048 mide el cargador (ver BAT_ABSENT_* en board_nrf52.h)
static bool battery_looks_absent(float soc, float crate) {
  return soc > BAT_ABSENT_SOC_PCT || crate > BAT_ABSENT_CRATE_PCT_H || crate < -BAT_ABSENT_CRATE_PCT_H;
}

static bool battery_reads_charger = false; // la última lectura parecía la del cargador (SIN BATERIA)

static bool battery_print_status(const char *prefix) {
  battery_reads_charger = false;
  uint16_t mv = 0, status = 0;
  float soc = 0, crate = 0;
  DEBUG_PRINT(prefix);
  if (!fuelGauge.present() || !fuelGauge.readVoltageMv(mv) || !fuelGauge.readSoc(soc) ||
      !fuelGauge.readCrate(crate) || !fuelGauge.readStatus(status)) {
    DEBUG_PRINTLN("MAX17048 no responde en 0x36");
    return false;
  }
  DEBUG_PRINT("MAX17048 v");
  DEBUG_PRINT(fuelGauge.version(), HEX);
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(mv / 1000.0f, 3);
  DEBUG_PRINT(" V | SOC ");
  DEBUG_PRINT(soc, 1);
  DEBUG_PRINT(" % | ");
  DEBUG_PRINT(crate, 1);
  DEBUG_PRINT(" %/h");
  if (status & Max17048::STATUS_VL) DEBUG_PRINT(" | ALERTA voltaje bajo");
  if (status & Max17048::STATUS_VH) DEBUG_PRINT(" | ALERTA voltaje alto");
  if (status & Max17048::STATUS_HD) DEBUG_PRINT(" | ALERTA SOC bajo");
  if (status & Max17048::STATUS_VR) DEBUG_PRINT(" | reset por voltaje");
  DEBUG_PRINTLN("");

  // Rango de medida del MAX17048 (1 celda): 2.5 V a 5 V; por fuera hay un problema de conexión
  if (mv < 2500 || mv > 4500) {
    DEBUG_PRINT(prefix);
    DEBUG_PRINTLN("Voltaje fuera del rango de una celda Li+ (2.5-4.5 V)");
    return false;
  }
  if (battery_looks_absent(soc, crate)) {
    DEBUG_PRINT(prefix);
    DEBUG_PRINTLN("SIN BATERIA: el MAX17048 lee la salida del cargador (SOC > 103 % o C-rate > 300 %/h)");
    battery_reads_charger = true;
    return false;
  }
  return true;
}

// El MAX17048 se busca en los dos buses (POR CONFIRMAR en cuál está)
static bool fuel_gauge_find(void) {
  fuel_gauge_bus = nullptr;
  if (i2c1_ready && fuelGauge.begin(Wire))
    fuel_gauge_bus = "I2C1";
  else if (i2c2_ready && fuelGauge.begin(Wire1))
    fuel_gauge_bus = "I2C2";
  return fuel_gauge_bus != nullptr;
}

static void battery_init(void) {
  if (!i2c1_ready && !i2c2_ready) {
    DEBUG_PRINTLN("[INIT]   Sin buses I2C -> MAX17048 omitido");
    return;
  }
  if (!fuel_gauge_find()) {
    DEBUG_PRINTLN("[INIT]   MAX17048 no responde en 0x36 (ni en I2C1 ni en I2C2)");
    return;
  }
  DEBUG_PRINT("[INIT]   MAX17048 en ");
  DEBUG_PRINTLN(fuel_gauge_bus);
  if (fuelGauge.powerOnReset()) {
    DEBUG_PRINTLN("[INIT]   MAX17048 recien alimentado (RI): el SOC estara listo en ~1 s");
  }
  battery_print_status("[INIT]   ");
  // Las alertas de STATUS quedan guardadas hasta que se borran y el MAX17048 no se reinicia con el nRF52 (p. ej.
  // SOC bajo de cuando funcionó sin batería): ya se imprimieron arriba; se borran para que el POST y lo que
  // sigue muestren solo las de este arranque.
  uint16_t status = 0;
  if (fuelGauge.readStatus(status) && (status & Max17048::STATUS_ALERTS)) {
    DEBUG_PRINT("[INIT]   Alertas del MAX17048 anteriores al arranque (STATUS 0x");
    DEBUG_PRINT(status, HEX);
    DEBUG_PRINT(")");
    DEBUG_PRINTLN(fuelGauge.clearAlerts() ? ": borradas" : ": no se pudieron borrar");
  }
}

// ---- IMU LSM6DS3TR-C ----
// Lee una muestra (espera hasta 50 ms a que haya dato nuevo). false si no responde o la aceleración
// no es la de un equipo en reposo o en movimiento normal (|a| fuera de 0.5-1.5 g).
static bool imu_print_sample(const char *prefix) {
  DEBUG_PRINT(prefix);
  const uint32_t t0 = millis();
  while (!imu.dataReady() && millis() - t0 < 50) delay(2);
  Lsm6ds3::Sample s;
  if (!imu.read(s)) {
    DEBUG_PRINTLN("LSM6DS3TR-C sin lectura");
    return false;
  }
  const float mag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  DEBUG_PRINT("acel ");
  DEBUG_PRINT(s.ax, 3);
  DEBUG_PRINT(' ');
  DEBUG_PRINT(s.ay, 3);
  DEBUG_PRINT(' ');
  DEBUG_PRINT(s.az, 3);
  DEBUG_PRINT(" g (|a| ");
  DEBUG_PRINT(mag, 3);
  DEBUG_PRINT(") | giro ");
  DEBUG_PRINT(s.gx, 1);
  DEBUG_PRINT(' ');
  DEBUG_PRINT(s.gy, 1);
  DEBUG_PRINT(' ');
  DEBUG_PRINT(s.gz, 1);
  DEBUG_PRINT(" dps | ");
  DEBUG_PRINT(s.tempC, 1);
  DEBUG_PRINTLN(" C");
  if (mag < 0.5f || mag > 1.5f) {
    DEBUG_PRINT(prefix);
    DEBUG_PRINTLN("Aceleracion fuera de 0.5-1.5 g (en reposo debe ser ~1 g)");
    return false;
  }
  return true;
}

static void imu_init(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[INIT]   Sin bus I2C2 -> IMU omitido");
    return;
  }
  if (!imu.begin(Wire1)) {
    DEBUG_PRINTLN("[INIT]   LSM6DS3TR-C no responde en 0x6A (¿POWER_IMU encendido?)");
    return;
  }
  DEBUG_PRINTLN("[INIT]   LSM6DS3TR-C 0x6A (WHO_AM_I 0x6A) | 52 Hz, +-2 g, 250 dps | lectura a 40 Hz");
}

// ---- Ambiente BME280 (tarjeta MEMS) ----
// Una medición; false si no responde o está fuera del rango de operación del chip
// (-40 a 85 °C, 300 a 1100 hPa).
static bool env_print_sample(const char *prefix) {
  DEBUG_PRINT(prefix);
  Bme280::Sample s;
  if (!env.read(s)) {
    DEBUG_PRINTLN("BME280 sin medicion");
    return false;
  }
  DEBUG_PRINT(s.tempC, 2);
  DEBUG_PRINT(" C | ");
  DEBUG_PRINT(s.humidity, 1);
  DEBUG_PRINT(" %HR | ");
  DEBUG_PRINT(s.pressureHpa, 2);
  DEBUG_PRINTLN(" hPa");
  if (s.tempC < -40.0f || s.tempC > 85.0f || s.pressureHpa < 300.0f || s.pressureHpa > 1100.0f) {
    DEBUG_PRINT(prefix);
    DEBUG_PRINTLN("Medicion fuera del rango del BME280 (-40 a 85 C, 300 a 1100 hPa)");
    return false;
  }
  return true;
}

static void env_init(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[INIT]   Sin bus I2C2 -> BME280 omitido");
    return;
  }
  if (!env.begin(Wire1)) {
    DEBUG_PRINTLN("[INIT]   BME280 no responde en 0x76 (¿tarjeta MEMS conectada?)");
    return;
  }
  DEBUG_PRINTLN("[INIT]   BME280 0x76 (ID 0x60) | normal, x1, IIR 4, ~55 Hz | lectura a 40 Hz");
}

// ---- Tarjeta MEMS: sensores H2 (AIN0) y CH4 (AIN1) ----
// ---- Flash interna (InternalFS, LittleFS de Adafruit en 0xED000-0xF4000): fix del GPS y calibración del CH4 ----
// LBM guarda su contexto en las 4 páginas justo debajo de 0xED000: no se cruzan.
static bool flash_fs_begin(void) {
  static int8_t state = -1;                          // -1 sin intentar, 0 falló, 1 montada
  if (state < 0) state = InternalFS.begin() ? 1 : 0; // si no se puede montar, la formatea
  return state == 1;
}

// ---- Calibración del CH4 en la flash: R0 y curva (lib/ch4_mos) ----
static const char *CH4_CAL_FILE = "/ch4_cal.bin";
static constexpr uint32_t CH4_CAL_MAGIC = 0x57433431; // "WC41"
struct Ch4CalRecord {
  uint32_t magic;
  float r0;
  Ch4Curve curve;
};
static float ch4_saved_r0 = 0; // R0 de la flash (para decidir si hay que guardar)
static bool ch4_save_pending = false;

static void ch4_cal_load(void) {
  if (!flash_fs_begin()) return;
  Ch4CalRecord r = {};
  LfsFile f(InternalFS);
  if (!f.open(CH4_CAL_FILE, Adafruit_LittleFS_Namespace::FILE_O_READ)) {
    DEBUG_PRINTLN("[INIT]   CH4: sin calibracion guardada (R0 se toma sola al calentar; curva del datasheet)");
    return;
  }
  const bool read = f.read(&r, sizeof(r)) == (int)sizeof(r);
  f.close();
  if (!read || r.magic != CH4_CAL_MAGIC) {
    DEBUG_PRINTLN("[INIT]   CH4: calibracion guardada no valida: se ignora");
    return;
  }
  if (r.r0 > 0 && board_has_data(r.r0)) ch4mos.setR0(r.r0);
  if (Ch4Mos::curveValid(r.curve)) ch4mos.setCurve(r.curve);
  ch4_saved_r0 = ch4mos.r0();
  DEBUG_PRINT("[INIT]   CH4: R0 guardada ");
  DEBUG_PRINT(ch4mos.r0(), 0);
  DEBUG_PRINT(" ohm | curva Rs/R0 = ");
  DEBUG_PRINT(ch4mos.curve().a, 3);
  DEBUG_PRINT(" x (ppm/");
  DEBUG_PRINT(ch4mos.curve().refPpm, 0);
  DEBUG_PRINT(")^");
  DEBUG_PRINTLN(ch4mos.curve().b, 3);
}

static void ch4_cal_save(void) {
  if (!flash_fs_begin()) return;
  const Ch4CalRecord r = {CH4_CAL_MAGIC, ch4mos.r0(), ch4mos.curve()};
  InternalFS.remove(CH4_CAL_FILE); // FILE_O_WRITE agrega al final: se escribe desde cero
  LfsFile f(InternalFS);
  const bool ok = f.open(CH4_CAL_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE) &&
                  f.write((const uint8_t *)&r, sizeof(r)) == sizeof(r);
  f.close();
  if (ok) ch4_saved_r0 = r.r0;
  DEBUG_PRINT(ok ? "[CH4] Calibracion guardada en la flash: R0 " : "[CH4] No se pudo guardar la calibracion: R0 ");
  DEBUG_PRINT(r.r0, 0);
  DEBUG_PRINTLN(" ohm");
}

// ---- Ajustes de la aplicación en la flash (/settings.bin): sobreviven a los reinicios ----
// Hoy: período de la medición automática (cambiable por downlink 04 MM MM o serie MP:<min>)
static const char *SETTINGS_FILE = "/settings.bin";
static constexpr uint32_t SETTINGS_MAGIC = 0x57535431; // "WST1"
struct SettingsRecord {
  uint32_t magic;
  uint16_t measurePeriodMin;
  uint16_t rtcReal; // 1 = el RTC lo puso una fuente real (red, GPS, mano o mesh) y no perdió la hora desde entonces
};
static SettingsRecord settings = {0, 0, 0};
static bool settings_loaded = false, settings_save_pending = false;

static void settings_load(void) {
  if (settings_loaded) return;
  settings_loaded = true;
  if (!flash_fs_begin()) return;
  LfsFile f(InternalFS);
  if (!f.open(SETTINGS_FILE, Adafruit_LittleFS_Namespace::FILE_O_READ)) return;
  SettingsRecord r = {};
  if (f.read(&r, sizeof(r)) == (int)sizeof(r) && r.magic == SETTINGS_MAGIC) settings = r;
  f.close();
}

static void settings_save(void) {
  if (!flash_fs_begin()) return;
  settings.magic = SETTINGS_MAGIC;
  InternalFS.remove(SETTINGS_FILE);
  LfsFile f(InternalFS);
  const bool ok = f.open(SETTINGS_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE) &&
                  f.write((const uint8_t *)&settings, sizeof(settings)) == sizeof(settings);
  f.close();
  DEBUG_PRINTLN(ok ? "[FLASH] Ajustes guardados" : "[FLASH] No se pudieron guardar los ajustes");
}

uint16_t board_measure_period_min(uint16_t defaultMin) {
  settings_load();
  // 0xFFFF = el registro existe (p. ej. por la marca del RTC) pero nunca se guardó un período
  return settings.magic == SETTINGS_MAGIC && settings.measurePeriodMin != 0xFFFF ? settings.measurePeriodMin : defaultMin;
}

void board_save_measure_period_min(uint16_t minutes) {
  settings_load();
  settings.magic = SETTINGS_MAGIC;
  settings.measurePeriodMin = minutes;
  settings_save_pending = true;
}

static bool settings_rtc_real(void) {
  settings_load();
  return settings.magic == SETTINGS_MAGIC && settings.rtcReal == 1;
}

static void settings_set_rtc_real(bool real) {
  settings_load();
  if (settings.magic == SETTINGS_MAGIC && (settings.rtcReal == 1) == real) return;
  if (settings.magic != SETTINGS_MAGIC) settings.measurePeriodMin = 0xFFFF; // sin período guardado: ver abajo
  settings.magic = SETTINGS_MAGIC;
  settings.rtcReal = real ? 1 : 0;
  settings_save_pending = true;
}

__attribute__((unused)) static const char *time_source_name(TimeSource s) {
  switch (s) {
    case TIME_MESH: return "mesh";
    case TIME_RTC: return "RTC (conservada)";
    case TIME_MANUAL: return "manual (serie/BLE)";
    case TIME_GPS: return "GPS";
    case TIME_NET: return "red LoRaWAN";
    default: return "ninguna";
  }
}

static void time_source_set(TimeSource s) {
  time_source = s;
  rtc_trusted = s != TIME_NONE;
  unix_base = 0; // board_unix_now() vuelve a leer el RTC
  if (s != TIME_NONE && s != TIME_RTC) settings_set_rtc_real(true);
}

// Las escrituras en la flash bloquean ~100 ms: solo con el radio libre
static void ch4_cal_service(void) {
  if (ch4_save_pending && board_radio_idle()) {
    ch4_save_pending = false;
    ch4_cal_save();
  }
  if (settings_save_pending && board_radio_idle()) {
    settings_save_pending = false;
    settings_save();
  }
}

static void mems_init(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[INIT]   Sin bus I2C2 -> tarjeta MEMS omitida");
    return;
  }
  // MCP4018: volátil (arranca en el paso 63), se escribe en cada arranque
  if (!ch4Load.begin(Wire1, MCP4018_RAB_OHMS)) {
    DEBUG_PRINTLN("[INIT]   MCP4018 no responde en 0x2F (¿tarjeta MEMS conectada?)");
  } else {
    const uint8_t step = ch4Load.stepFor(CH4_LOAD_OHMS);
    uint8_t readBack = 0xFF;
    const bool ok = ch4Load.write(step) && ch4Load.read(readBack) && readBack == step;
    DEBUG_PRINT("[INIT]   MCP4018 0x2F | carga CH4: paso ");
    DEBUG_PRINT(step);
    DEBUG_PRINT("/127 = ");
    DEBUG_PRINT(ch4Load.ohmsAt(step));
    DEBUG_PRINT(" ohm (pedido ");
    DEBUG_PRINT(CH4_LOAD_OHMS);
    DEBUG_PRINTLN(ok ? ", R_AB 5k +-20 %)" : ") -> ERROR al escribir/verificar");
  }
  // Estimador del CH4: RL del paso escrito (su error de ±20 % se cancela en Rs/R0)
  Ch4Mos::Config cc;
  cc.vcVolts = CH4_VC_MV / 1000.0f;
  cc.rlOhms = ch4Load.present() ? (float)ch4Load.ohmsAt(ch4Load.stepFor(CH4_LOAD_OHMS)) : (float)CH4_LOAD_OHMS;
  cc.curve = {CH4_CURVE_A, CH4_CURVE_B, CH4_CURVE_REF_PPM};
  cc.ppmMax = CH4_PPM_MAX;
  cc.warmupMinMs = CH4_WARMUP_MIN_MS;
  cc.stablePct = CH4_STABLE_PCT;
  cc.r0TauUpS = CH4_R0_TAU_UP_S;
  cc.r0TauDownS = CH4_R0_TAU_DOWN_S;
  cc.r0MaxDropPct = CH4_R0_MAX_DROP_PCT;
  ch4mos.begin(cc);
  ch4_cal_load();

  if (!adc.begin(Wire1, ADS1219_ADDR)) {
    DEBUG_PRINTLN("[INIT]   ADS1219 no responde en 0x41 o no acepta la configuracion");
    return;
  }
  const bool cal = adc.calibrateOffset();
  DEBUG_PRINT("[INIT]   ADS1219 0x41 | 330 SPS, ganancia 1, ref interna 2.048 V | offset ");
  DEBUG_PRINT(cal ? String(adc.offset()) : String("sin calibrar"));
  DEBUG_PRINTLN(" | AIN0 = H2, AIN1 = CH4 | lectura a 40 Hz");
}

// Conversión de los dos canales en cada periodo de muestreo, sin bloquear: se inicia AIN0 en el
// tic de 25 ms y adc_process() (cada loop) lee el resultado al terminar e inicia AIN1 (~4 ms cada una).
enum AdcState : uint8_t { ADC_IDLE,
                          ADC_CONV_H2,
                          ADC_CONV_CH4 };
static AdcState adc_state = ADC_IDLE;
static uint32_t adc_start_ms = 0;
static int32_t adc_h2_code = 0;
static int32_t adc_ch4_code = 0;
static uint32_t adc_samples = 0;
static uint32_t adc_errors = 0;

static void adc_start_cycle(uint32_t now) {
  if (!adc.present() || adc_state != ADC_IDLE) return;
  if (adc.start(Ads1219::MUX_AIN0)) {
    adc_state = ADC_CONV_H2;
    adc_start_ms = now;
  } else {
    adc_errors++;
  }
}

static void adc_process(void) {
  if (adc_state == ADC_IDLE) return;
  const uint32_t now = millis();
  const uint32_t elapsed = now - adc_start_ms;
  if (elapsed < adc.conversionMs()) return;
  bool ready = false;
  if (!adc.dataReady(ready) || (!ready && elapsed > 20)) { // sin respuesta o conversión colgada
    adc_errors++;
    adc_state = ADC_IDLE;
    return;
  }
  if (!ready) return;
  int32_t code = 0;
  if (!adc.readRaw(code)) {
    adc_errors++;
    adc_state = ADC_IDLE;
    return;
  }
  if (adc_state == ADC_CONV_H2) {
    adc_h2_code = code;
    if (adc.start(Ads1219::MUX_AIN1)) {
      adc_state = ADC_CONV_CH4;
      adc_start_ms = now;
    } else {
      adc_errors++;
      adc_state = ADC_IDLE;
    }
  } else {
    adc_ch4_code = code;
    adc_samples++;
    adc_state = ADC_IDLE;
  }
}

static void print_adc_channel(const char *name, int32_t code) {
  DEBUG_PRINT(name);
  DEBUG_PRINT(' ');
  DEBUG_PRINT(adc.toVolts(code), 4);
  DEBUG_PRINT(Ads1219::saturated(code) ? " V (SATURADO: > 2.048 V)" : " V");
}

// ---- Blower Sunon UB3H3-500: FAN_CONTROL (P0.28) y salida FG (FAN_FEEDBACK P0.13) ----
// FG es de colector abierto (transistor a GND, máx. 4 mA): necesita pull-up a V+ (≤ 3.5 V). Se usa el
// pull-up interno del nRF (~13 kΩ). Motor de 6 polos: 3 pulsos por vuelta (hoja de datos, pág. 6).
static volatile uint32_t fan_pulses = 0;
static void fan_isr(void) {
  fan_pulses++;
}
static bool fan_on = false;
static uint32_t fan_rpm = 0;         // medido cada 1 s mientras está encendido
static uint32_t fan_revolutions = 0; // vueltas contadas desde el arranque

static void blower_init(void) {
  digitalWrite(WM_PIN_FAN_CTL, LOW); // ya en LOW desde safe_gpio_init()
  fan_on = false;
  pinMode(WM_PIN_FAN_FB, INPUT_PULLUP);
  attachInterrupt(WM_PIN_FAN_FB, fan_isr, RISING);
  DEBUG_PRINT("[INIT]   Blower apagado (FAN_CONTROL P0.28) | FG en P0.13 con pull-up (nivel ");
  DEBUG_PRINT(digitalRead(WM_PIN_FAN_FB) ? "ALTO" : "BAJO");
  DEBUG_PRINT(") | nominal ");
  DEBUG_PRINT(FAN_NOMINAL_RPM);
  DEBUG_PRINT(" rpm, ");
  DEBUG_PRINT(FAN_PULSES_PER_REV);
  DEBUG_PRINTLN(" pulsos/vuelta");
}

bool board_fan_set(bool on) {
  digitalWrite(WM_PIN_FAN_CTL, on ? HIGH : LOW);
  fan_on = on;
  if (!on) fan_rpm = 0;
  return true;
}

// ---- Supervisor de energía (MAX17048 + hora local) ----
// Lee la batería cada 1 s despierto y cada 10 s en reposo, actualiza RICH/CORE/SURVIVAL y aplica la
// política de rieles: el GM-602B (5V_H2, calefactor de ~78 mW siempre encendido) solo en RICH.
static int8_t local_hour(void) {
  Rx8130::DateTime utc, local;
  if (!rtc.present() || !rtc.readTime(utc)) return -1;
  Rx8130::fromUnix(Rx8130::toUnix(utc) + RTC_UTC_OFFSET_H * 3600L, local);
  return (int8_t)local.hour;
}

static void energy_apply_rails(void) {
  const bool h2 = !survive_mode && energy.allowGas();
  if (expander.present() && ((expander.outputs() >> 7) & 1) != h2) board_rail_set("H2", h2);
}

static uint8_t cutoff_readings = 0; // lecturas seguidas bajo BAT_CUTOFF_MV

bool board_battery_cutoff(void) {
  return cutoff_readings >= BAT_CUTOFF_READINGS;
}

static void energy_service(bool force) {
  static uint32_t last_ms = 0;
  static uint32_t hour_ms = 0;
  static int8_t hour = -1;
  const uint32_t now = millis();
  if (!force && now - last_ms < (power_save ? 10000UL : 1000UL)) return;
  last_ms = now;

  uint16_t mv = 0;
  float soc = 0, crate = 0;
  // Sin batería (lectura del cargador) = sin dato: no entra al supervisor de energía ni a los CSV/tramas
  const bool bat = fuelGauge.present() && fuelGauge.readVoltageMv(mv) && fuelGauge.readSoc(soc) &&
                   fuelGauge.readCrate(crate) && !battery_looks_absent(soc, crate);
  // Corte por batería agotada: lecturas seguidas bajo BAT_CUTOFF_MV y sin USB (con USB se está cargando)
  const bool usb = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
  if (bat && !usb && mv < BAT_CUTOFF_MV) {
    if (cutoff_readings < 255) cutoff_readings++;
  } else {
    cutoff_readings = 0;
  }
  snap.batV = bat ? mv / 1000.0f : NAN;
  snap.batSoc = bat ? soc : NAN;
  snap.batCrate = bat ? crate : NAN;
  if (!bat) return;

  if (force || hour < 0 || now - hour_ms >= 60000UL) { // la hora cambia lento: RTC cada minuto
    hour = local_hour();
    hour_ms = now;
  }
  const EnergySupervisor::Mode before = energy.mode();
  const bool first = !energy.valid();
  if (energy.update(soc, mv / 1000.0f, crate, hour) || first) {
    DEBUG_PRINT("[ENERGY] ");
    DEBUG_PRINT(first ? "inicio" : EnergySupervisor::name(before));
    DEBUG_PRINT(" -> ");
    DEBUG_PRINT(energy.modeName());
    DEBUG_PRINT(" | SOC ");
    DEBUG_PRINT(soc, 1);
    DEBUG_PRINT(" % | ");
    DEBUG_PRINT(mv / 1000.0f, 3);
    DEBUG_PRINT(" V | C-rate filtrado ");
    DEBUG_PRINT(energy.crateFiltered(), 2);
    DEBUG_PRINT(" %/h | ");
    DEBUG_PRINTLN(energy.isDay() ? "dia" : "noche");
  }
  snap.energyMode = (float)energy.mode();
  snap.crateFiltered = energy.crateFiltered();
  energy_apply_rails();
}

static void fan_measure(void) {
  static uint32_t last_ms = 0;
  static uint32_t last_count = 0;
  static uint32_t rev_pulses = 0;
  const uint32_t now = millis();
  if (now - last_ms < 1000) return;
  const uint32_t count = fan_pulses;
  const uint32_t delta = count - last_count;
  if (fan_on && last_ms != 0) fan_rpm = delta * 60000UL / (now - last_ms) / FAN_PULSES_PER_REV;
  // Encendido y sin vueltas = sin dato (arranque, o el blower no gira: si dura toda la medición es falla)
  snap.fanRpm = !fan_on ? 0.0f : (fan_rpm > 0 ? (float)fan_rpm : NAN);

  rev_pulses += delta;
  fan_revolutions += rev_pulses / FAN_PULSES_PER_REV;
  rev_pulses %= FAN_PULSES_PER_REV;
  last_count = count;
  last_ms = now;
}

static void gps_fix_load(void);
static void gps_publish_backup(void);

// GPS: no bloquea. Mientras no se sepa cuál pin es la TX del GPS se cuentan flancos en los dos (como
// entradas, sin transmitir); el que tenga tráfico NMEA (ráfagas a 9600 baud cada segundo) es el RX del nRF.
static void gps_init(void) {
  pinMode(WM_PIN_GPS_A, INPUT);
  pinMode(WM_PIN_GPS_B, INPUT);
  gps_edges_a = 0;
  gps_edges_b = 0;
  attachInterrupt(WM_PIN_GPS_A, gps_isr_a, CHANGE);
  attachInterrupt(WM_PIN_GPS_B, gps_isr_b, CHANGE);
  gps_watching = true;
  DEBUG_PRINTLN("[INIT]   L96-M33: buscando la TX del GPS en P0.14 y P1.09 (la UART se abre al detectarla)");
  // CTRL_GPS quedó encendido en power_rails_init(): primera sesión, hasta el primer fix válido
  gps_powered = true;
  gps_on_ms = millis();
  gps_fix_load();
  gps_publish_backup();
}

// ---- Último fix en la flash interna (ver flash_fs_begin) ----
static const char *GPS_FIX_FILE = "/gps_fix.bin";
static constexpr uint32_t GPS_FIX_MAGIC = 0x57474631; // "WGF1"
struct GpsFixRecord {
  uint32_t magic;
  float lat, lon, alt;
  uint32_t unixUtc; // hora UTC del fix (0 = desconocida)
};
static bool gps_fs_ok = false;

static bool rtc_unix_now(uint32_t &now) {
  Rx8130::DateTime t;
  if (!rtc.present() || !rtc.readTime(t)) return false;
  now = Rx8130::toUnix(t);
  return true;
}

static void gps_fix_load(void) {
  gps_fs_ok = flash_fs_begin();
  if (!gps_fs_ok) {
    DEBUG_PRINTLN("[INIT]   Flash interna (InternalFS) no disponible: el ultimo fix no se conserva");
    return;
  }
  GpsFixRecord r = {};
  LfsFile f(InternalFS);
  if (!f.open(GPS_FIX_FILE, Adafruit_LittleFS_Namespace::FILE_O_READ)) {
    DEBUG_PRINTLN("[INIT]   Sin fix guardado en la flash");
    return;
  }
  const bool read = f.read(&r, sizeof(r)) == (int)sizeof(r);
  f.close();
  if (!read || r.magic != GPS_FIX_MAGIC || !board_has_data(r.lat) || !board_has_data(r.lon)) {
    DEBUG_PRINTLN("[INIT]   Fix guardado en la flash no valido: se ignora");
    return;
  }
  uint32_t now = 0;
  const bool ageKnown = r.unixUtc != 0 && rtc_unix_now(now) && now >= r.unixUtc;
  last_fix = {true, false, ageKnown, r.lat, r.lon, r.alt, 0, ageKnown ? now - r.unixUtc : 0};
  DEBUG_PRINT("[INIT]   Ultimo fix guardado: ");
  DEBUG_PRINT(r.lat, 6);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(r.lon, 6);
  if (ageKnown) {
    DEBUG_PRINT(" | hace ");
    DEBUG_PRINT((now - r.unixUtc) / 60UL);
    DEBUG_PRINTLN(" min");
  } else {
    DEBUG_PRINTLN(" | edad desconocida (sin hora en el RTC)");
  }
}

static void gps_fix_save(void) {
  if (!gps_fs_ok || !last_fix.valid || !last_fix.thisBoot) return;
  uint32_t now = 0;
  const uint32_t ageS = (millis() - last_fix.ms) / 1000UL;
  GpsFixRecord r = {GPS_FIX_MAGIC, last_fix.lat, last_fix.lon, last_fix.alt, rtc_unix_now(now) ? now - ageS : 0};
  InternalFS.remove(GPS_FIX_FILE); // FILE_O_WRITE agrega al final: se escribe desde cero
  LfsFile f(InternalFS);
  const bool ok = f.open(GPS_FIX_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE) && f.write((const uint8_t *)&r, sizeof(r)) == sizeof(r);
  f.close();
  gps_saved_ms = millis();
  DEBUG_PRINTLN(ok ? "[GPS] Ultimo fix guardado en la flash" : "[GPS] No se pudo guardar el fix en la flash");
}

// Hora UTC del GPS al RTC (fix válido con fecha de RMC y hora de GGA): un collar que nunca se unió a LoRaWAN también
// queda en hora para las ventanas del anillo. Con hora confiable solo corrige más de 2 s, y si la diferencia pasa de
// 1 h desconfía del GPS (fecha y hora de tramas distintas justo a medianoche) y no toca el RTC.
static void gps_time_sync(const NmeaGps::Fix &f) {
  static uint32_t last_ms = 0;
  if (!f.timeValid || !f.dateValid || !rtc.present() || time_source == TIME_NET) return; // la de la red manda
  // Hora de la mesh o ninguna: el GPS la corrige ya; con hora buena, a lo sumo cada 6 h
  if (time_source >= TIME_RTC && last_ms && millis() - last_ms < 6UL * 3600UL * 1000UL) return;
  last_ms = millis() | 1;
  const Rx8130::DateTime g = {f.year, f.month, f.day, f.hour, f.minute, f.second, 0};
  const uint32_t gps = Rx8130::toUnix(g);
  uint32_t now = 0;
  const bool have = rtc_unix_now(now);
  const int32_t diff = have ? (int32_t)(gps - now) : 0;
  if (have && time_source >= TIME_RTC && (diff > 3600 || diff < -3600)) return;
  if (have && time_source >= TIME_RTC && diff >= -2 && diff <= 2) return;
  Rx8130::DateTime t;
  Rx8130::fromUnix(gps, t);
  if (!rtc.setTime(t)) return;
  time_source_set(TIME_GPS);
  DEBUG_PRINT("[RTC] Hora del GPS: ");
  DEBUG_PRINT(gps);
  DEBUG_PRINT(" (unix UTC)");
  if (have) {
    DEBUG_PRINT(" | correccion ");
    DEBUG_PRINT(diff);
    DEBUG_PRINT(" s");
  }
  DEBUG_PRINTLN("");
}

// Sin fix en este despertar: lat/lon/alt del último fix (si hubo) y su edad
static void gps_publish_backup(void) {
  if (gps_fix_this_wake) return; // el fix de esta medición se conserva aunque se pierda la señal
  snap.gpsLat = last_fix.valid ? last_fix.lat : NAN;
  snap.gpsLon = last_fix.valid ? last_fix.lon : NAN;
  snap.gpsAltM = last_fix.valid ? last_fix.alt : NAN;
  if (!last_fix.valid || (!last_fix.thisBoot && !last_fix.ageKnown))
    snap.gpsAgeS = NAN;
  else if (last_fix.thisBoot)
    snap.gpsAgeS = (float)((millis() - last_fix.ms) / 1000UL);
  else
    snap.gpsAgeS = (float)last_fix.bootAgeS + (float)(millis() / 1000UL);
}

static const char *gps_pin_name(uint8_t pin) {
  return pin == WM_PIN_GPS_A ? "P0.14" : "P1.09";
}

// Enciende o apaga el L96 (CTRL_GPS). Apagado: UART cerrada y pines como entrada (no alimentar el L96 por sus
// pines). Con R154 (CTRL_GPS fijo a VDD) no se puede apagar: queda encendido.
static void gps_power(bool on, const char *why) {
  if (!on && (rails_nodrive_mask() & 1u)) return;
  if (on == gps_powered) {
    if (on && gps_session_done) { // R154: no se apagó; sesión nueva sin cortar la alimentación
      gps_on_ms = millis();
      gps_session_fix_ms = 0;
      gps_session_done = false;
    }
    return;
  }
  if (on) {
    board_rail_set("GPS", true);
    gps.reset();
    gps_last_gga_ms = 0;
    if (gps_rx_pin != BOARD_PIN_NC) {
      Serial1.setPins(gps_rx_pin, gps_rx_pin == WM_PIN_GPS_A ? WM_PIN_GPS_B : WM_PIN_GPS_A);
      Serial1.begin(WM_GPS_BAUD);
    }
    gps_on_ms = millis();
    gps_session_fix_ms = 0;
    gps_session_done = false;
  } else {
    if (gps_rx_pin != BOARD_PIN_NC) {
      Serial1.end();
      pinMode(WM_PIN_GPS_A, INPUT);
      pinMode(WM_PIN_GPS_B, INPUT);
    }
    board_rail_set("GPS", false);
    snap.gpsFix = NAN;
    snap.gpsSats = NAN;
  }
  gps_powered = on;
  DEBUG_PRINT(on ? "[GPS] Encendido: " : "[GPS] Apagado: ");
  DEBUG_PRINTLN(why);
}

bool board_gps_powered(void) {
  return gps_powered;
}

// Fin de la sesión: GPS_HOLD_AFTER_FIX_MS después del fix (o GPS_SEARCH_MAX_MS sin fix) se apaga; el fix se
// guarda en la flash a lo sumo cada GPS_SAVE_PERIOD_MS y solo con el radio libre (la escritura bloquea ~100 ms).
#if BOARD_ENABLE_MESH
static bool diag_gps_hold(void); // sondeo del anillo en curso (más abajo): el GPS no se apaga
#else
static bool diag_gps_hold(void) {
  return false;
}
#endif

static void gps_power_service(void) {
  if (gps_powered && !gps_session_done) {
    const uint32_t now = millis();
    if (gps_session_fix_ms != 0 && now - gps_session_fix_ms >= GPS_HOLD_AFTER_FIX_MS && !diag_gps_hold()) {
      gps_session_done = true;
      if (gps_saved_ms == 0 || now - gps_saved_ms >= GPS_SAVE_PERIOD_MS) gps_save_pending = true;
      gps_power(false, "fix valido obtenido (ahorra ~25 mA hasta la proxima medicion)");
    } else if (gps_session_fix_ms == 0 && GPS_SEARCH_MAX_MS != 0 && now - gps_on_ms >= GPS_SEARCH_MAX_MS) {
      gps_session_done = true;
      gps_power(false, "sin fix en GPS_SEARCH_MAX_MS: se reintenta en la proxima medicion");
    }
  }
  if (gps_save_pending && board_radio_idle()) {
    gps_save_pending = false;
    gps_fix_save();
  }
}

static void gps_service(void) {
  if (gps_watching) {
    // Una ráfaga NMEA da cientos de flancos; el otro pin (RX del GPS) no debe moverse
    const uint32_t a = gps_edges_a, b = gps_edges_b;
    if ((a < 100 || b > 10) && (b < 100 || a > 10)) return;
    detachInterrupt(WM_PIN_GPS_A);
    detachInterrupt(WM_PIN_GPS_B);
    gps_watching = false;
    gps_rx_pin = a >= 100 ? WM_PIN_GPS_A : WM_PIN_GPS_B;
    const uint8_t tx = gps_rx_pin == WM_PIN_GPS_A ? WM_PIN_GPS_B : WM_PIN_GPS_A;
    Serial1.setPins(gps_rx_pin, tx);
    Serial1.begin(WM_GPS_BAUD);
    DEBUG_PRINT("[GPS] TX del L96 en ");
    DEBUG_PRINT(gps_pin_name(gps_rx_pin));
    DEBUG_PRINT(" -> UART abierta (RX ");
    DEBUG_PRINT(gps_pin_name(gps_rx_pin));
    DEBUG_PRINT(", TX ");
    DEBUG_PRINT(gps_pin_name(tx));
    DEBUG_PRINTLN(", 9600)");
    return;
  }
  gps_power_service();
  if (!gps_powered || gps_rx_pin == BOARD_PIN_NC) {
    static uint32_t backup_ms = 0; // GPS apagado: la edad del respaldo sigue avanzando
    if (millis() - backup_ms >= 1000) {
      backup_ms = millis();
      gps_publish_backup();
    }
    return;
  }
  while (Serial1.available()) gps.feed((char)Serial1.read());
  const NmeaGps::Fix &f = gps.fix();
  if (f.updatedMs == gps_last_gga_ms) return; // solo cuenta una GGA nueva
  gps_last_gga_ms = f.updatedMs;
  snap.gpsFix = f.quality;
  snap.gpsSats = f.satellites;
  // Fix válido: calidad, satélites y HDOP suficientes (un fix de 3 satélites puede errar cientos de metros)
  if (f.quality > 0 && f.satellites >= GPS_MIN_SATS && f.hdop > 0 && f.hdop <= GPS_MAX_HDOP) {
    last_fix = {true, true, true, (float)f.latitude, (float)f.longitude, f.altitudeM, millis(), 0};
    gps_fix_this_wake = true;
    snap.gpsLat = last_fix.lat;
    snap.gpsLon = last_fix.lon;
    snap.gpsAltM = last_fix.alt;
    snap.gpsAgeS = 0;
    gps_time_sync(f);
    if (gps_session_fix_ms == 0) {
      gps_session_fix_ms = millis();
      DEBUG_PRINT("[GPS] Fix valido tras ");
      DEBUG_PRINT((gps_session_fix_ms - gps_on_ms) / 1000UL);
      DEBUG_PRINT(" s (");
      DEBUG_PRINT(f.satellites);
      DEBUG_PRINT(" satelites, HDOP ");
      DEBUG_PRINT(f.hdop, 1);
      DEBUG_PRINT("): ");
      DEBUG_PRINT(f.latitude, 6);
      DEBUG_PRINT(", ");
      DEBUG_PRINT(f.longitude, 6);
      DEBUG_PRINT(" | sigue encendido ");
      DEBUG_PRINT(GPS_HOLD_AFTER_FIX_MS / 1000UL);
      DEBUG_PRINTLN(" s y se apaga");
    }
  } else {
    gps_publish_backup();
  }
}

// ---- Watchdog ----
// WDT de hardware del nRF52: si nadie lo alimenta en WDT_TIMEOUT_S (loop colgado, bus I2C trabado, lazo
// infinito, HardFault) reinicia el chip. Sigue contando con la CPU dormida (SLEEP = Run) y se pausa con el
// depurador (HALT = Pause). Un reinicio por software no lo detiene: el WDT sigue corriendo tras un reset
// (en el arranque se alimenta antes de que venza).
// GPREGRET2 (se conserva en los reinicios por watchdog y por software; se borra al quitar la alimentación):
// bits 3:0 = reinicios de recuperación seguidos, bit 7 = el último fue por el watchdog de software,
// bit 6 = marca de WDT de hardware (no fiable: la interrupción TIMEOUT llega ~60 us antes del reset y puede
// no alcanzar a escribirse), bit 5 = RUN: la aplicación estaba corriendo. Al arrancar se deja RUN puesto; si
// lo encuentra al arrancar sin reinicio por software, el reinicio fue no controlado (WDT de hardware, pin de
// reset o carga por USB) y cuenta como recuperación. El bootloader de Adafruit BORRA RESETREAS, por eso no
// se puede distinguir solo con RESETREAS.
static uint32_t reset_reason = 0;
static uint8_t wdt_recoveries = 0;
static uint8_t wdt_boot_flags = 0; // bits 7:5 de GPREGRET2 al arrancar
static bool wdt_running = false;

// ---- Registro de la caída (2026-10-22) ----
// Antes un HardFault (puntero corrupto, pila desbordada...) dejaba el CPU en el bucle infinito del handler débil del
// core hasta que el WDT lo reiniciaba, sin que su interrupción (prioridad 2) alcanzara a anotar nada: en fallas.csv
// solo quedaba "no controlado". Ahora el HardFault anota en RAM dónde fue (PC, LR, registros de falla) y reinicia ya;
// la interrupción del WDT anota el PC del loop colgado. Las dos guardan además el estado de la FSM, la tarea de la
// placa en curso (trace) y la pila mínima libre del loop. Al arrancar se lee y va a fallas.csv; el PC se traduce a
// función y línea con: arm-none-eabi-addr2line -f -C -e %TEMP%/pio_build_wearme/seeed_wio_tracker_1110/firmware.elf 0x<PC>
// (el ELF del MISMO firmware que corría: build_dir de platformio.ini)
// La RAM sobrevive al reset por WDT, por software y por pin (no a un corte de energía). Dirección fija en la mitad alta
// de la RAM: el bootloader de Adafruit en un arranque normal solo usa su .bss (desde 0x20008000) y su pila (tope), y la
// aplicación solo llegaría ahí con más de ~130 KB de heap. Si no es válido (encendido), magic y check no coinciden.
static constexpr uint32_t CRASH_MAGIC = 0x57435231; // "WCR1"
static constexpr uint32_t CRASH_HARDFAULT = 1, CRASH_WDT = 2;
struct CrashRecord {
  uint32_t magic, kind, pc, lr, cfsr, hfsr, faultAddr, uptimeS, stackFree;
  char state[12], step[12];
  uint32_t check;
};
static CrashRecord *const crash_ram = (CrashRecord *)0x20030000UL;
static CrashRecord crash_last;       // el del último reinicio (copia: el de RAM se borra al leerlo)
static bool crash_have = false;
static const char *volatile trace_state = "START"; // estado de la FSM (board_trace_state)
static const char *volatile trace_step = "arranque"; // tarea de la placa en curso
static volatile uint32_t trace_uptime_s = 0;
static volatile uint32_t stack_min_free = 0; // bytes; 0 = sin medir todavía

// Tarea de la placa en curso mientras exista el objeto (deja la anterior al salir)
struct TraceScope {
  const char *prev;
  explicit TraceScope(const char *s) : prev(trace_step) { trace_step = s; }
  ~TraceScope() { trace_step = prev; }
};

static uint32_t crash_check(const CrashRecord &r) {
  const uint32_t *w = (const uint32_t *)&r;
  uint32_t c = 0x9E3779B9u;
  for (size_t i = 0; i < offsetof(CrashRecord, check) / 4; i++) c = (c ^ w[i]) * 16777619u;
  return c;
}

static void crash_copy(char *dst, const char *src, size_t n) {
  size_t i = 0;
  for (; src && src[i] && i < n - 1; i++) dst[i] = src[i];
  dst[i] = 0;
}

// Desde el HardFault o la interrupción del WDT: sin llamadas a la librería, rápido (el WDT da ~60 us)
static void crash_save(uint32_t kind, uint32_t pc, uint32_t lr) {
  CrashRecord &r = *crash_ram;
  r.kind = kind;
  r.pc = pc;
  r.lr = lr;
  r.cfsr = SCB->CFSR;
  r.hfsr = SCB->HFSR;
  r.faultAddr = (r.cfsr & SCB_CFSR_BFARVALID_Msk) ? SCB->BFAR : (r.cfsr & SCB_CFSR_MMARVALID_Msk) ? SCB->MMFAR : 0;
  r.uptimeS = trace_uptime_s;
  r.stackFree = stack_min_free;
  crash_copy(r.state, trace_state, sizeof(r.state));
  crash_copy(r.step, trace_step, sizeof(r.step));
  r.check = crash_check(r);
  r.magic = CRASH_MAGIC;
}

// HardFault: el marco apilado (MSP en interrupciones, PSP en las tareas de FreeRTOS) trae el PC donde ocurrió
extern "C" __attribute__((used)) void wm_hardfault(uint32_t *frame) {
  crash_save(CRASH_HARDFAULT, frame[6], frame[5]);
  NVIC_SystemReset(); // sin esperar los WDT_TIMEOUT_S del watchdog
}

extern "C" __attribute__((naked)) void HardFault_Handler(void) {
  __asm volatile(
      "tst lr, #4      \n"
      "ite eq          \n"
      "mrseq r0, msp   \n"
      "mrsne r0, psp   \n"
      "b wm_hardfault  \n");
}

// Qué dice CFSR/HFSR, en corto (la primera causa que aplica)
static const char *crash_fault_text(uint32_t cfsr, uint32_t hfsr) {
  if (cfsr & SCB_CFSR_STKERR_Msk || cfsr & SCB_CFSR_MSTKERR_Msk) return "pila desbordada al entrar a la excepcion";
  if (cfsr & SCB_CFSR_UNDEFINSTR_Msk) return "instruccion invalida (codigo o puntero a funcion corrupto)";
  if (cfsr & (SCB_CFSR_INVSTATE_Msk | SCB_CFSR_INVPC_Msk)) return "salto a una direccion invalida (puntero corrupto)";
  if (cfsr & SCB_CFSR_DIVBYZERO_Msk) return "division por cero";
  if (cfsr & SCB_CFSR_UNALIGNED_Msk) return "acceso desalineado";
  if (cfsr & (SCB_CFSR_PRECISERR_Msk | SCB_CFSR_DACCVIOL_Msk)) return "acceso a una direccion de memoria invalida";
  if (cfsr & SCB_CFSR_IMPRECISERR_Msk) return "escritura a una direccion invalida (imprecisa: el PC es aproximado)";
  if (cfsr & (SCB_CFSR_IBUSERR_Msk | SCB_CFSR_IACCVIOL_Msk)) return "ejecucion en una direccion invalida";
  if (hfsr & SCB_HFSR_VECTTBL_Msk) return "error al leer la tabla de vectores";
  return "falla sin detalle en CFSR";
}
static constexpr uint8_t GPREG_SW_WDT = 0x80;
static constexpr uint8_t GPREG_HW_WDT = 0x40;
static constexpr uint8_t GPREG_RUN = 0x20;
static constexpr uint8_t GPREG_BURNIN = 0x10; // precalentamiento del CH4 en curso (sigue tras un reinicio)
static bool burnin_resume = false;            // retomar el precalentamiento al terminar el arranque

// GPREGRET (id 0, marca del bootloader) y GPREGRET2 (id 1, este registro) con el SoftDevice activo: SOLO por su API.
// POWER es un periférico restringido: una escritura directa dispara la protección del SoftDevice y reinicia el chip
// al instante (2026-10-29: por eso el modo DFU nunca llegaba al bootloader, y board_wdt_healthy() reiniciaba los
// collares a los 10 min de encendidos: NODO-004 el 2026-10-21). Antes del Bluetooth (wdt_init) y en las
// interrupciones (WDT, HardFault: no se puede llamar al SoftDevice desde ahí y el reinicio ya es inminente) es directo.
static bool sd_enabled(void) {
  uint8_t on = 0;
  sd_softdevice_is_enabled(&on);
  return on != 0;
}

static uint8_t gpreg_read(uint8_t id) {
  if (sd_enabled()) {
    uint32_t v = 0;
    sd_power_gpregret_get(id, &v);
    return (uint8_t)v;
  }
  return (uint8_t)(id ? NRF_POWER->GPREGRET2 : NRF_POWER->GPREGRET);
}

static void gpreg_write(uint8_t id, uint8_t v) {
  if (sd_enabled()) {
    sd_power_gpregret_clr(id, 0xFF);
    sd_power_gpregret_set(id, v);
  } else if (id) {
    NRF_POWER->GPREGRET2 = v;
  } else {
    NRF_POWER->GPREGRET = v;
  }
}

// El core ya define WDT_IRQHandler (driver nrfx_wdt): se registra este callback en nrfx_wdt_init()
static const nrfx_wdt_t wdt_instance = NRFX_WDT_INSTANCE(0);

static void wdt_timeout_handler(void) {
  NRF_POWER->GPREGRET2 = (NRF_POWER->GPREGRET2 & (0x0F | GPREG_BURNIN)) | GPREG_HW_WDT;
  // PC del loop colgado: marco que apiló la tarea interrumpida (aproximado si lo colgado era otra interrupción)
  const uint32_t *psp = (const uint32_t *)__get_PSP();
  const bool pspOk = (uint32_t)psp >= 0x20006000UL && (uint32_t)psp < 0x2003FFE0UL;
  crash_save(CRASH_WDT, pspOk ? psp[6] : 0, pspOk ? psp[5] : 0);
  while (true) {
    __asm__ volatile("nop"); // el reset llega en ~60 us
  }
}

static void print_reset_reason(uint32_t r) {
  if (r == 0) DEBUG_PRINT("encendido");
  if (r & POWER_RESETREAS_RESETPIN_Msk) DEBUG_PRINT("pin de reset ");
  if (r & POWER_RESETREAS_DOG_Msk) DEBUG_PRINT("WATCHDOG ");
  if (r & POWER_RESETREAS_SREQ_Msk) DEBUG_PRINT("software ");
  if (r & POWER_RESETREAS_LOCKUP_Msk) DEBUG_PRINT("bloqueo de CPU ");
  if (r & POWER_RESETREAS_OFF_Msk) DEBUG_PRINT("despertar de System OFF ");
  if (r & POWER_RESETREAS_VBUS_Msk) DEBUG_PRINT("VBUS ");
}

static void wdt_init(void) {
  reset_reason = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = 0xFFFFFFFF; // se limpian escribiendo 1
  const uint8_t reg = (uint8_t)NRF_POWER->GPREGRET2;
  wdt_boot_flags = reg & (GPREG_SW_WDT | GPREG_HW_WDT | GPREG_RUN);
  // RUN sin reinicio por software: la aplicación se cortó sin pasar por board_wdt_software_reset().
  // Con USB conectado (VBUS) no cuenta: casi siempre es una carga de firmware o un reset en el banco; si no,
  // tres cargas en 10 min llevarían a FAULT. Los reinicios del watchdog de software cuentan siempre.
  const bool usb = (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
  // Registro de la caída en RAM (HardFault o WDT): cuenta siempre, también con USB
  crash_have = crash_ram->magic == CRASH_MAGIC && crash_ram->check == crash_check(*crash_ram) &&
               (crash_ram->kind == CRASH_HARDFAULT || crash_ram->kind == CRASH_WDT);
  if (crash_have) crash_last = *crash_ram;
  crash_ram->magic = 0;
  const bool unexpected = (wdt_boot_flags & GPREG_RUN) && !(wdt_boot_flags & GPREG_SW_WDT) && !usb;
  const bool recovery = (reset_reason & POWER_RESETREAS_DOG_Msk) || (wdt_boot_flags & (GPREG_SW_WDT | GPREG_HW_WDT)) ||
                        unexpected || crash_have;
  wdt_recoveries = recovery ? (uint8_t)min(15, (reg & 0x0F) + 1) : 0;
  burnin_resume = (reg & GPREG_BURNIN) != 0;
  NRF_POWER->GPREGRET2 = GPREG_RUN | wdt_recoveries | (reg & GPREG_BURNIN); // RUN: el próximo reinicio no controlado lo verá

  // Corre con la CPU dormida y se pausa con el depurador. Si ya corría tras un reset por software, CONFIG y
  // CRV no se pueden cambiar (quedan los mismos valores); se habilita igual la interrupción TIMEOUT, que
  // marca el reinicio por WDT antes de que ocurra.
  nrfx_wdt_config_t config = NRFX_WDT_DEFAULT_CONFIG;
  config.behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP;
  config.reload_value = WDT_TIMEOUT_S * 1000UL;
  config.interrupt_priority = 2; // la más alta permitida con el SoftDevice (0, 1 y 4 son suyas: con otra, el BLE no arranca)
  nrfx_wdt_channel_id channel;
  if (nrfx_wdt_init(&wdt_instance, &config, wdt_timeout_handler) == NRFX_SUCCESS &&
      nrfx_wdt_channel_alloc(&wdt_instance, &channel) == NRFX_SUCCESS) {
    nrfx_wdt_enable(&wdt_instance);
  }
  wdt_running = true;
  board_wdt_feed();

  DEBUG_PRINT("[INIT]   Ultimo reinicio: ");
  print_reset_reason(reset_reason);
  if (recovery) {
    DEBUG_PRINT("(");
    DEBUG_PRINT(board_wdt_last_cause());
    DEBUG_PRINT(")");
  }
  DEBUG_PRINT(" | WDT ");
  DEBUG_PRINT(WDT_TIMEOUT_S);
  DEBUG_PRINT(" s | reinicios de recuperacion seguidos: ");
  DEBUG_PRINTLN(wdt_recoveries);
}

void board_wdt_feed(void) {
  if (wdt_running) NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

void board_wdt_software_reset(const char *reason) {
  DEBUG_PRINT("[WDT] Reinicio por watchdog de software: ");
  DEBUG_PRINTLN(reason);
  gpreg_write(1, (gpreg_read(1) & (0x0F | GPREG_BURNIN)) | GPREG_SW_WDT);
  delay(200); // que salga el mensaje por USB
  NVIC_SystemReset();
}

// Actualización del firmware por Bluetooth (OTA DFU legacy del bootloader de Adafruit, tools/ota_dfu.py o
// `pio run -t ota`). Reinicio controlado: sin RUN el arranque siguiente (firmware nuevo o, si nadie se conectó, el
// mismo) no cuenta como caída. El WDT sigue corriendo en el bootloader: se alimenta justo antes para darle el tiempo
// completo (WDT_TIMEOUT_S); si nadie se conecta, el WDT lo devuelve al firmware.
// magic: 0xA8 = OTA por Bluetooth ("AdaDFU"), 0x57 = UF2 (unidad USB), 0x4E = serie (adafruit-nrfutil)
static void board_enter_bootloader(uint8_t magic, const char *what, const char *source) {
  DEBUG_PRINT("[DFU] ");
  DEBUG_PRINT(what);
  DEBUG_PRINT(" (pedido por ");
  DEBUG_PRINT(source);
  DEBUG_PRINTLN("): reinicio en el bootloader");
  gpreg_write(1, gpreg_read(1) & GPREG_BURNIN); // sin RUN: el arranque siguiente no cuenta como caída
  gpreg_write(0, magic);                        // marca para el bootloader
  DEBUG_PRINT("[DFU] GPREGRET = 0x");
  DEBUG_PRINTLN(gpreg_read(0), HEX);
  Serial.flush();
  delay(800); // que salgan los mensajes por USB
  board_wdt_feed();
  NVIC_SystemReset();
}

bool board_usb_powered(void) {
  uint32_t status = 0;
  if (sd_enabled()) sd_power_usbregstatus_get(&status);
  else status = NRF_POWER->USBREGSTATUS;
  return (status & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

void board_enter_ota_dfu(const char *source) {
  board_enter_bootloader(0xA8, "Actualizacion por Bluetooth: anuncia \"AdaDFU\", enviar con `pio run -t ota`", source);
}

void board_enter_uf2_dfu(const char *source) {
  board_enter_bootloader(0x57, "Bootloader UF2 (unidad USB)", source);
}

void board_enter_serial_dfu(const char *source) {
  board_enter_bootloader(0x4E, "Bootloader serie (adafruit-nrfutil, el de `pio run -t upload`)", source);
}

// Bootloader instalado: versión (TIMER2->CC[0] que deja el bootloader, leída por el core al arrancar) y los textos
// de su flash (el de UF2 dice modelo y fecha). Para saber si soporta la actualización por Bluetooth (OTA).
void board_bootloader_info(void) {
  extern uint32_t bootloaderVersion;
  // Para comprobar qué firmware quedó tras una carga (USB o Bluetooth): subir WEARME_FW_VERSION en cada entrega
  DEBUG_PRINTLN("[BOOT] Firmware " WEARME_FW_VERSION " compilado: " __DATE__ " " __TIME__);
  DEBUG_PRINT("[BOOT] Version del bootloader: 0x");
  DEBUG_PRINTLN(bootloaderVersion, HEX);
  const char *p = (const char *)0xF4000UL, *end = (const char *)0xFE000UL;
  const char *keys[] = {"UF2 Bootloader", "Model:", "Board-ID:", "AdaDFU", "Date:"};
  for (const char *k : keys) {
    const size_t n = strlen(k);
    for (const char *q = p; q + n < end; q++) {
      if (memcmp(q, k, n) != 0) continue;
      char line[72];
      size_t i = 0;
      while (i < sizeof(line) - 1 && q + i < end && q[i] >= 0x20 && q[i] < 0x7F) line[i] = q[i], i++;
      line[i] = 0;
      DEBUG_PRINT("[BOOT]   ");
      DEBUG_PRINTLN(line);
      break;
    }
  }
}

void board_wdt_healthy(void) {
  if (wdt_recoveries == 0) return;
  wdt_recoveries = 0;
  gpreg_write(1, GPREG_RUN | (gpreg_read(1) & GPREG_BURNIN));
  DEBUG_PRINTLN("[WDT] Sistema estable: contador de reinicios de recuperacion en 0");
}

uint8_t board_wdt_recoveries(void) {
  return wdt_recoveries;
}

// Texto para fallas.csv y el serial (sin ';', el separador del CSV)
const char *board_wdt_last_cause(void) {
  static char text[200];
  if (crash_have) {
    const CrashRecord &r = crash_last;
    size_t n = 0;
    auto add = [&](int w) { n = w > 0 ? min(sizeof(text) - 1, n + (size_t)w) : n; };
    if (r.kind == CRASH_HARDFAULT) {
      add(snprintf(text, sizeof(text), "HardFault: %s", crash_fault_text(r.cfsr, r.hfsr)));
      if (r.faultAddr) add(snprintf(text + n, sizeof(text) - n, " (direccion 0x%08lX)", (unsigned long)r.faultAddr));
      add(snprintf(text + n, sizeof(text) - n, " | CFSR 0x%08lX", (unsigned long)r.cfsr));
    } else {
      add(snprintf(text, sizeof(text), "WDT de hardware: loop colgado %lu s", (unsigned long)WDT_TIMEOUT_S));
    }
    add(snprintf(text + n, sizeof(text) - n, " | PC 0x%08lX LR 0x%08lX | en %s/%s a los %lu s | pila libre min %lu B",
                 (unsigned long)r.pc, (unsigned long)r.lr, r.state, r.step, (unsigned long)r.uptimeS,
                 (unsigned long)r.stackFree));
    return text;
  }
  if (wdt_boot_flags & GPREG_SW_WDT) return "watchdog de software (estado atascado)";
  if (wdt_boot_flags & GPREG_HW_WDT) return "WDT de hardware (loop colgado, sin registro del PC)";
  if (wdt_boot_flags & GPREG_RUN)
    return "sin registro de la causa: corte o caida de la alimentacion, pin de reset, carga de firmware o cuelgue "
           "dentro de una interrupcion";
  if (reset_reason & POWER_RESETREAS_DOG_Msk) return "WDT de hardware";
  return "";
}

void board_trace_state(const char *name) {
  trace_state = name;
}

uint32_t board_stack_free_min(void) {
  return stack_min_free;
}

// Pila libre mínima de la tarea del loop (4 KB en el core: LOOP_STACK_SZ) y uptime para el registro de la caída
static void stack_monitor(void) {
  static uint32_t last_ms = 0;
  if (last_ms && millis() - last_ms < 1000UL) return;
  last_ms = millis();
  trace_uptime_s = millis() / 1000UL;
  const uint32_t freeB = (uint32_t)uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
  if (stack_min_free == 0 || freeB < stack_min_free) stack_min_free = freeB;
}

HwStatus board_test_watchdog(void) {
  DEBUG_PRINT("[POST]   WDT ");
  DEBUG_PRINT(NRF_WDT->RUNSTATUS ? "corriendo" : "DETENIDO");
  DEBUG_PRINT(" (");
  DEBUG_PRINT(WDT_TIMEOUT_S);
  DEBUG_PRINT(" s) | ultimo reinicio: ");
  if (*board_wdt_last_cause())
    DEBUG_PRINT(board_wdt_last_cause());
  else
    print_reset_reason(reset_reason);
  DEBUG_PRINT(" | pila libre min del loop ");
  DEBUG_PRINT(stack_min_free);
  DEBUG_PRINT(" B");
  DEBUG_PRINT(" | recuperaciones seguidas ");
  DEBUG_PRINT(wdt_recoveries);
  DEBUG_PRINT("/");
  DEBUG_PRINTLN(WDT_MAX_RECOVERIES);
  if (!NRF_WDT->RUNSTATUS) return HW_FAIL;
  if (wdt_recoveries >= WDT_MAX_RECOVERIES) {
    DEBUG_PRINTLN("[POST]   Demasiados reinicios por watchdog seguidos: algo se bloquea siempre -> FAULT");
    return HW_FAIL;
  }
  return HW_PASS;
}

// Espera atendiendo LoRaWAN (LBM), el GPS y el watchdog: para pruebas del POST de más de unos ms (p. ej. el
// blower), que también se pueden pedir por serie con el nodo ya unido a la red.
static void board_wait_ms(uint32_t ms) {
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
#if BOARD_ENABLE_LORA && !BOARD_POST_ONLY
    lora.process(); // en el firmware de prueba no: aquí arrancaría el Join
#endif
    gps_service();
    board_wdt_feed();
    delay(1);
  }
}

// ---- Viento: 2 x FS3000-1015 detrás del mux TCA9543A (I2C2 0x70) ----
// Los dos FS3000 tienen la misma dirección (0x28): canal 0 = derecho, canal 1 = izquierdo. Se alimentan de
// POWER_VEL (TCA6408A P3, conectores U3/U5). Los canales del mux solo se abren con POWER_VEL encendido
// (un sensor sin alimentación podría bloquear el bus) y se cierran después de cada lectura.
static Tca9543a windMux;
static Fs3000 windSensor;
static bool wind_on = false;
// Arranque de los FS3000 tras POWER_VEL. Medido en la placa: a 100 ms responden solo ceros y luego suben (91, 300...
// cuentas); a 1 s ya dan ~416 cuentas sin flujo (el cero de la tabla es 409)
static constexpr uint32_t WIND_POWERUP_MS = 1000;
static constexpr uint32_t WIND_READY_TIMEOUT_MS = 500; // POST: reintentos tras el arranque si alguno aún no mide

// Lee un canal: true y m/s si responde con checksum correcto. why (opcional): el motivo de la falla
static bool wind_read(uint8_t channel, uint16_t &raw, float &mps, const char **why = nullptr) {
  if (!windMux.present() || !windMux.select(channel)) {
    if (why) *why = "MUX NO CAMBIA DE CANAL (TCA9543A 0x70)";
    return false;
  }
  const bool ok = windSensor.read(Wire1, raw, mps);
  windMux.disable();
  if (why) {
    *why = ok ? "OK"
         : windSensor.lastStatus() == Fs3000::BAD_CHECKSUM ? "DATO CORRUPTO (checksum): cable o ruido en el bus"
         : windSensor.lastStatus() == Fs3000::NOT_READY    ? "SIN DATO (responde solo ceros): no arranca o tarjeta FS3000 danada"
                                                            : "NO RESPONDE en 0x28: cable o tarjeta FS3000 de ese lado";
  }
  return ok;
}

// Una línea por lado: "[POST]   Derecho (canal 0): OK 1.23 m/s (1500 cuentas)" o "...: NO RESPONDE ..."
// (la página de la estación lee estas líneas para decir qué lado falla)
static void wind_print_side(const char *side, uint8_t channel, bool ok, float mps, uint16_t raw, const char *why) {
  DEBUG_PRINT("[POST]   ");
  DEBUG_PRINT(side);
  DEBUG_PRINT(" (canal ");
  DEBUG_PRINT(channel);
  DEBUG_PRINT("): ");
  if (ok) {
    DEBUG_PRINT("OK ");
    DEBUG_PRINT(mps, 2);
    DEBUG_PRINT(" m/s (");
    DEBUG_PRINT(raw);
    DEBUG_PRINTLN(" cuentas)");
  } else {
    DEBUG_PRINTLN(why);
  }
}

static void wind_init(void) {
  if (!i2c2_ready || !windMux.begin(Wire1)) {
    DEBUG_PRINTLN("[INIT]   TCA9543A no responde en 0x70 -> sin sensores de viento");
    return;
  }
  DEBUG_PRINTLN("[INIT]   TCA9543A 0x70 | FS3000-1015: canal 0 = derecho, canal 1 = izquierdo | POWER_VEL al medir");
}

bool board_wind_set(bool on) {
  if (on == wind_on) return true;
  wind_on = on;
  if (!on) {
    if (windMux.present()) windMux.disable();
    snap.windRight = NAN;
    snap.windLeft = NAN;
  }
  return board_rail_set("VEL", on);
}

HwStatus board_test_wind(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[POST]   Sin bus I2C2");
    return HW_SKIP;
  }
  if (!windMux.present() && !windMux.begin(Wire1)) {
    DEBUG_PRINTLN("[POST]   TCA9543A no responde en 0x70");
    return HW_FAIL;
  }
  const bool wasOn = wind_on;
  if (!wasOn) {
    board_wind_set(true);
    board_wait_ms(WIND_POWERUP_MS);
  }
  uint16_t rawR = 0, rawL = 0;
  float mpsR = 0, mpsL = 0;
  const char *whyR = "", *whyL = "";
  bool okR = false, okL = false; // bFsROK, bFsLOK
  // Si alguno aún responde con ceros tras el arranque, se reintenta cada 50 ms hasta WIND_READY_TIMEOUT_MS
  const uint32_t t0 = millis();
  for (;;) {
    if (!okR) okR = wind_read(0, rawR, mpsR, &whyR);
    if (!okL) okL = wind_read(1, rawL, mpsL, &whyL);
    if ((okR && okL) || millis() - t0 >= WIND_READY_TIMEOUT_MS) break;
    board_wait_ms(50);
  }
  if (!wasOn) board_wind_set(false);

  wind_print_side("Derecho", 0, okR, mpsR, rawR, whyR);
  wind_print_side("Izquierdo", 1, okL, mpsL, rawL, whyL);
  if (!okR && !okL) {
    DEBUG_PRINTLN("[POST]   Falla: los DOS sensores (derecho e izquierdo): revisar POWER_VEL (P3) y que las dos tarjetas FS3000 esten conectadas");
  } else if (!okR) {
    DEBUG_PRINTLN("[POST]   Falla: sensor DERECHO (canal 0)");
  } else if (!okL) {
    DEBUG_PRINTLN("[POST]   Falla: sensor IZQUIERDO (canal 1)");
  }
  return (okR && okL) ? HW_PASS : HW_FAIL;
}

// Muestreo de los subsistemas a SENSOR_SAMPLE_HZ cada 25 ms, sin bloquear: IMU (14 bytes, ~1.6 ms
// a 100 kHz), BME280 (8 bytes, ~1 ms) y el inicio de las conversiones del ADS1219. Se guarda la
// última muestra de cada uno.
static constexpr uint32_t SENSOR_PERIOD_MS = 1000 / SENSOR_SAMPLE_HZ;
static Lsm6ds3::Sample imu_last;
static Bme280::Sample env_last;
static uint32_t imu_samples = 0;
static uint32_t env_samples = 0;

static void print_rate(const char *name, uint32_t samples) {
  DEBUG_PRINT(name);
  DEBUG_PRINT(" Muestreo a ");
  DEBUG_PRINT(SENSOR_SAMPLE_HZ);
  DEBUG_PRINT(" Hz: ");
  DEBUG_PRINT(samples);
  DEBUG_PRINT(" muestras en 10 s (esperadas ");
  DEBUG_PRINT(SENSOR_SAMPLE_HZ * 10);
  DEBUG_PRINTLN(")");
}

static void sensors_sample(void) {
  static uint32_t next_ms = 0;
  static uint32_t window_start_ms = 0;
  static uint32_t ticks = 0;
  static bool rate_reported = false;
  if (power_save) return; // IDLE/FAULT: sin muestreo (sensores en reposo)
  adc_process();          // segundo canal del ADS1219 (fuera del tic)
  if (!imu.present() && !env.present() && !adc.present()) return;
  const uint32_t now = millis();
  if ((int32_t)(now - next_ms) < 0) return;
  if (ticks++ == 0) window_start_ms = now;
  // Paso fijo de 25 ms; si el loop se atrasó más de un periodo (p. ej. un POST por serie) se reajusta
  next_ms = ((int32_t)(now - next_ms) > (int32_t)SENSOR_PERIOD_MS) ? now + SENSOR_PERIOD_MS : next_ms + SENSOR_PERIOD_MS;
  adc_start_cycle(now);
  const bool imuOk = imu.present() && imu.read(imu_last);
  const bool envOk = env.present() && env.read(env_last);
  if (imuOk) imu_samples++;
  if (envOk) env_samples++;

  // Foto para la aplicación (valores de este tic; el ADS1219 entrega su último par AIN0/AIN1)
  snap.tick++;
  snap.ax = imuOk ? imu_last.ax : NAN;
  snap.ay = imuOk ? imu_last.ay : NAN;
  snap.az = imuOk ? imu_last.az : NAN;
  snap.gx = imuOk ? imu_last.gx : NAN;
  snap.gy = imuOk ? imu_last.gy : NAN;
  snap.gz = imuOk ? imu_last.gz : NAN;
  snap.imuTempC = imuOk ? imu_last.tempC : NAN;
  snap.tempC = envOk ? env_last.tempC : NAN;
  snap.humidity = envOk ? env_last.humidity : NAN;
  snap.pressureHpa = envOk ? env_last.pressureHpa : NAN;
  const bool adcOk = adc.present() && adc_samples > 0;
  // Sin alimentación del sensor el voltaje no es medición: sin dato (CORE: 5V_H2 y CH4 apagados)
  const bool h2Powered = expander.present() && ((expander.outputs() >> 7) & 1);
  snap.h2V = adcOk && h2Powered && !Ads1219::saturated(adc_h2_code) ? adc.toVolts(adc_h2_code) : NAN;
  snap.ch4V = adcOk && ch4_on && !Ads1219::saturated(adc_ch4_code) ? adc.toVolts(adc_ch4_code) : NAN;
  // H2: calibración lineal ppm = m × V + b (NAN se propaga: sin voltaje no hay ppm)
  snap.h2Ppm = board_has_data(snap.h2V) ? H2_PPM_M * snap.h2V + H2_PPM_B : NAN;
  // CH4: cuentas crudas del ADS1219 (readRaw(), 24 bits con signo; no voltios). El predictor está calibrado
  // en cuentas: ADC_5000PPM y N_ADC = 2^23 - 1 en lib/methane_predictor/src/MethanePredictor.cpp.
  // Equivalencia con voltios: V = cuentas × VREF_INTERNAL (2.048 V, GAIN = 1) / 2^23 (Ads1219::toVolts).
  snap.ch4Raw = adcOk && ch4_on && !Ads1219::saturated(adc_ch4_code) ? (float)adc_ch4_code : NAN;
  // ppm como Rs/R0 (lib/ch4_mos). Saturado (Vout >= 2.048 V) se usa el fondo de escala: Rs real es menor, así que
  // la ppm queda como cota inferior en vez de perder la muestra en pleno evento.
  if (adcOk && ch4_on) {
    const float v = Ads1219::saturated(adc_ch4_code) ? adc.toVolts(0x7FFFFF) : adc.toVolts(adc_ch4_code);
    ch4mos.update(v, envOk ? env_last.tempC : NAN, envOk ? env_last.humidity : NAN, now);
  }
  snap.ch4Ppm = ch4_on ? ch4mos.ppm() : NAN;
  snap.ch4RsOhm = ch4_on ? ch4mos.rsComp() : NAN;
  snap.ch4R0Ohm = ch4mos.r0Valid() ? ch4mos.r0() : NAN;
  snap.ch4Ratio = ch4_on ? ch4mos.ratio() : NAN;
  snap.ch4Warm = ch4_on ? (ch4mos.warm() ? 1.0f : 0.0f) : NAN;
  // Viento: los dos canales del mux (~2 ms a 100 kHz); sin dato si no responde
  if (wind_on) {
    uint16_t raw = 0;
    float mps = 0;
    snap.windRight = wind_read(0, raw, mps) ? mps : NAN;
    snap.windLeft = wind_read(1, raw, mps) ? mps : NAN;
  }
  // Gramos de CH4 emitidos (MEASURE): una llamada por muestra de 40 Hz. Sin T/presión no se llama
  // (el predictor calcula la densidad del aire con P y T). Viento sin dato (FS3000 no instalados) = 0 m/s:
  // con NaN el total quedaría en NaN; con 0 el flujo de esa fosa es nulo y el evento/derivada siguen.
  // Sin ppm (sensor calentando o sin R0) no se integra.
  if (methane_active && board_has_data(snap.ch4Ppm) && envOk) {
    const float uRight = board_has_data(snap.windRight) ? snap.windRight : 0.0f; // fosa 1 = canal 0 (derecho)
    const float uLeft = board_has_data(snap.windLeft) ? snap.windLeft : 0.0f;    // fosa 2 = canal 1 (izquierdo)
    process_methane_ppm(ch4_state, snap.ch4Ppm, uRight, uLeft, env_last.tempC, env_last.pressureHpa);
  }
  snap.ch4TotalG = methane_active ? (float)ch4_state.total_grams_emitted : NAN;
  snap.ch4RateGDay = methane_active ? (float)ch4_state.current_rate_g_day : NAN;
  snap.ch4Event = methane_active ? (ch4_state.event_active ? 1.0f : 0.0f) : NAN;

  // Verificación única de la frecuencia real: muestras en los primeros 10 s
  if (!rate_reported && now - window_start_ms >= 10000) {
    rate_reported = true;
    if (imu.present()) print_rate("[IMU]", imu_samples);
    if (env.present()) print_rate("[ENV]", env_samples);
    if (adc.present()) {
      print_rate("[ADC]", adc_samples);
      if (adc_errors) {
        DEBUG_PRINT("[ADC] Conversiones fallidas: ");
        DEBUG_PRINTLN(adc_errors);
      }
    }
  }
}

// Línea de depuración por serie con la última muestra de cada subsistema
static void sensors_debug_print(void) {
#if SENSOR_DEBUG_PERIOD_MS > 0
  static uint32_t last_ms = 0;
  const uint32_t now = millis();
  if (power_save || now - last_ms < SENSOR_DEBUG_PERIOD_MS) return; // en reposo no hay muestras nuevas
  last_ms = now;
  DEBUG_PRINT("[SENS]");
  if (imu.present()) {
    DEBUG_PRINT(" IMU ");
    DEBUG_PRINT(imu_last.ax, 2);
    DEBUG_PRINT(' ');
    DEBUG_PRINT(imu_last.ay, 2);
    DEBUG_PRINT(' ');
    DEBUG_PRINT(imu_last.az, 2);
    DEBUG_PRINT(" g |");
  }
  if (env.present()) {
    DEBUG_PRINT(' ');
    DEBUG_PRINT(env_last.tempC, 2);
    DEBUG_PRINT(" C ");
    DEBUG_PRINT(env_last.humidity, 1);
    DEBUG_PRINT(" %HR ");
    DEBUG_PRINT(env_last.pressureHpa, 1);
    DEBUG_PRINT(" hPa |");
  }
  if (adc.present()) {
    print_adc_channel(" H2", adc_h2_code);
    print_adc_channel(" CH4", adc_ch4_code);
    if (ch4_on) print_ch4_status(now); // Rs, R0 y ppm (solo con el calefactor encendido)
    DEBUG_PRINT(" |");
  }
  if (wind_on) {
    DEBUG_PRINT(" viento D ");
    DEBUG_PRINT(board_has_data(snap.windRight) ? String(snap.windRight, 2) : String("-"));
    DEBUG_PRINT(" I ");
    DEBUG_PRINT(board_has_data(snap.windLeft) ? String(snap.windLeft, 2) : String("-"));
    DEBUG_PRINT(" m/s |");
  }
  DEBUG_PRINT(" GPS ");
  if (!gps_powered) {
    DEBUG_PRINT("apagado");
  } else if (gps_rx_pin == BOARD_PIN_NC) {
    DEBUG_PRINT("sin tramas");
  } else {
    const NmeaGps::Fix &f = gps.fix();
    if (f.quality > 0) {
      DEBUG_PRINT("fix ");
      DEBUG_PRINT(f.latitude, 6);
      DEBUG_PRINT(' ');
      DEBUG_PRINT(f.longitude, 6);
      DEBUG_PRINT(' ');
    } else {
      DEBUG_PRINT("sin fix ");
    }
    DEBUG_PRINT(f.satellites);
    DEBUG_PRINT(" sat HDOP ");
    DEBUG_PRINT(f.hdop, 1);
    if (f.quality > 0) {
      DEBUG_PRINT(f.satellites >= GPS_MIN_SATS && f.hdop > 0 && f.hdop <= GPS_MAX_HDOP ? " (valido)"
                                                                                       : " (no valido: < 4 sat o HDOP > 5)");
    }
  }
  // Lo que va en el CSV, la app y LoRa (fix válido de esta medición o el último guardado)
  DEBUG_PRINT(" -> envia ");
  if (board_has_data(snap.gpsLat)) {
    DEBUG_PRINT(snap.gpsLat, 6);
    DEBUG_PRINT(' ');
    DEBUG_PRINT(snap.gpsLon, 6);
    DEBUG_PRINT(" edad ");
    DEBUG_PRINT(board_has_data(snap.gpsAgeS) ? String((unsigned long)snap.gpsAgeS) + " s" : String("?"));
  } else {
    DEBUG_PRINT("sin posicion");
  }
  DEBUG_PRINT(" |");
  DEBUG_PRINT(" blower ");
  if (fan_on) {
    DEBUG_PRINT(fan_rpm);
    DEBUG_PRINT(" rpm");
  } else {
    DEBUG_PRINT("OFF");
  }
  DEBUG_PRINT(" (");
  DEBUG_PRINT(fan_revolutions);
  DEBUG_PRINT(" vueltas, ");
  DEBUG_PRINT(fan_pulses);
  DEBUG_PRINTLN(" pulsos)");
#endif
}

// ---- RTC RX8130CE (hora en UTC) ----
static void rtc_print_time(const Rx8130::DateTime &t) {
  static const char *const days[] = {"dom", "lun", "mar", "mie", "jue", "vie", "sab"};
  Rx8130::DateTime local;
  Rx8130::fromUnix(Rx8130::toUnix(t) + RTC_UTC_OFFSET_H * 3600L, local);
  char buf[64];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u UTC (local %02u:%02u:%02u %s)", t.year, t.month,
           t.day, t.hour, t.minute, t.second, local.hour, local.minute, local.second, days[local.weekday % 7]);
  DEBUG_PRINT(buf);
}

static void rtc_init(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[INIT]   Sin bus I2C2 -> RTC omitido");
    return;
  }
  // Si se perdió la hora (VLF) se carga la de compilación (hora local del PC -> UTC) hasta que llegue
  // la hora de la red LoRaWAN
  Rx8130::DateTime fallback;
  Rx8130::buildTime(fallback);
  Rx8130::fromUnix(Rx8130::toUnix(fallback) - RTC_UTC_OFFSET_H * 3600L, fallback);
  if (!rtc.begin(Wire1, fallback)) {
    DEBUG_PRINTLN(rtc.present() ? "[INIT]   RX8130CE: error al inicializar (¿oscilador?)"
                                : "[INIT]   RX8130CE no responde en 0x32");
    if (!rtc.present()) return;
  }
  if (rtc.timeLost()) {
    DEBUG_PRINTLN("[INIT]   RX8130CE VLF: hora perdida -> registros inicializados y hora de compilacion");
  } else if (rtc.configFixed()) {
    DEBUG_PRINTLN("[INIT]   RX8130CE: configuracion corregida (FOUT off, INIEN = 1, CHGEN = 0)");
  }
  rtc_boot_valid = rtc.readTime(rtc_boot_time);
  rtc_boot_ms = millis();
  // Hora confiable solo si el RTC no la perdió Y alguna vez la puso una fuente real (red, GPS, mano o mesh): un RTC
  // que sigue corriendo con la hora de compilación de un arranque con VLF no sirve para las ventanas del anillo.
  if (rtc.timeLost()) settings_set_rtc_real(false);
  // Hora anterior a la compilación de este firmware: imposible si el RTC siguió contando. Pasó en NODO-003/004/005
  // (2026-11-03/04): arrancaron con 2026-10-29 ~15:33 UTC, VLF = 0 y rtcReal = 1, se dieron por confiables y su faro
  // HORA "buena" desfasó a los demás collares (ventanas que ya no coinciden). Se descarta como si se hubiera perdido.
  const bool rtcStale = rtc_boot_valid && !rtc.timeLost() && Rx8130::toUnix(rtc_boot_time) + 60 < Rx8130::toUnix(fallback);
  if (rtcStale) settings_set_rtc_real(false);
  time_source_set(rtc_boot_valid && !rtc.timeLost() && !rtcStale && settings_rtc_real() ? TIME_RTC : TIME_NONE);
  DEBUG_PRINT("[INIT]   RX8130CE banderas 1Dh = ");
  print_hex8(rtc.bootFlags());
  DEBUG_PRINTLN(rtcStale ? " | hora ANTERIOR a la compilacion del firmware: el RTC estuvo detenido -> no confiable" : "");
  DEBUG_PRINT("[INIT]   RX8130CE 0x32 | ");
  if (rtc_boot_valid) {
    rtc_print_time(rtc_boot_time);
    DEBUG_PRINT(" | hora ");
    DEBUG_PRINTLN(rtc_trusted ? "confiable (puesta antes por una fuente real)"
                              : "NO confiable: se espera la de la red, el GPS, la app/serie o la mesh");
  } else {
    DEBUG_PRINTLN("hora invalida");
  }
}

static void sd_print_status(const char *prefix) {
  DEBUG_PRINT(prefix);
  if (!sd.cardOk()) {
    DEBUG_PRINT("microSD no responde (error ");
    print_hex8(sd.errorCode());
    DEBUG_PRINTLN("): sin tarjeta, sin alimentacion o cableado SPI");
    return;
  }
  DEBUG_PRINT("microSD ");
  DEBUG_PRINT(sd.typeName());
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(sd.sizeMB());
  DEBUG_PRINT(" MB | ");
  if (sd.mounted()) {
    DEBUG_PRINTLN(sd.fsName());
    return;
  }
  DEBUG_PRINT("sin volumen FAT/exFAT valido | sector 0: ");
  if (!sd.sector0Read()) {
    DEBUG_PRINTLN("no se pudo leer");
  } else if (!sd.mbrSignature()) {
    DEBUG_PRINTLN("sin firma 55AA (sin formato o sin MBR)");
  } else {
    DEBUG_PRINT("MBR OK, particion 1 tipo ");
    print_hex8(sd.partitionType());
    DEBUG_PRINTLN(sd.partitionType() == 0x07 ? " (exFAT o NTFS)" : sd.partitionType() == 0xEE ? " (GPT)"
                                                                                              : "");
  }
}

// ============================================
// 5. Módulos de aplicación
// ============================================
#if BOARD_ENABLE_LORA
// Downlink con datos de aplicación: se guarda para la FSM, que interpreta el comando
static void on_downlink(uint8_t fport, const uint8_t *data, uint8_t length, int16_t rssi, float snr) {
  leds.play(failcodes_event(LED_EV_LORA_RX));
  snap.rssiDbm = rssi; // quedan en el CSV de la medición (calidad del enlace de bajada)
  snap.snrDb = snr;
  DEBUG_PRINT("[DL] Downlink FPort ");
  DEBUG_PRINT(fport);
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(length);
  DEBUG_PRINT(" B | RSSI ");
  DEBUG_PRINT(rssi);
  DEBUG_PRINT(" dBm | SNR ");
  DEBUG_PRINT(snr, 1);
  DEBUG_PRINT(" |");
  for (uint8_t i = 0; i < length; i++) {
    DEBUG_PRINT(' ');
    print_hex8(data[i]);
  }
  DEBUG_PRINTLN("");
  downlink_fport = fport;
  downlink_len = length > BOARD_DOWNLINK_MAX ? BOARD_DOWNLINK_MAX : length;
  memcpy(downlink_data, data, downlink_len);
  downlink_ble = false;
  downlink_pending = true;
}

// ---- Credenciales LoRaWAN en la flash (/lorawan.bin) ----
static const char *CRED_FILE = "/lorawan.bin";
static constexpr uint32_t CRED_MAGIC = 0x574C5731; // "WLW1"
struct CredRecord {
  uint32_t magic;
  uint8_t devEui[8], joinEui[8], appKey[16];
  uint32_t check; // FNV-1a de lo anterior: detecta un archivo dañado
};
static const char *cred_source = "firmware";

static uint32_t cred_check(const CredRecord &r) {
  const uint8_t *p = (const uint8_t *)&r;
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < offsetof(CredRecord, check); i++) h = (h ^ p[i]) * 16777619u;
  return h;
}

static void cred_hex(const uint8_t *d, size_t n) {
  char b[3];
  for (size_t i = 0; i < n; i++) {
    snprintf(b, sizeof(b), "%02X", d[i]);
    DEBUG_PRINT(b);
  }
}

// AppKey enmascarada para el log de arranque (completa solo con LORA:CRED?)
static void cred_key_masked(const uint8_t *k) {
  cred_hex(k, 2);
  DEBUG_PRINT("...");
  cred_hex(k + 14, 2);
}

// 1 = leídas y válidas, 0 = no hay archivo, -1 = archivo dañado
static int cred_read(CredRecord &r) {
  if (!flash_fs_begin()) return 0;
  LfsFile f(InternalFS);
  if (!f.open(CRED_FILE, Adafruit_LittleFS_Namespace::FILE_O_READ)) return 0;
  const bool full = f.read(&r, sizeof(r)) == (int)sizeof(r);
  f.close();
  return full && r.magic == CRED_MAGIC && r.check == cred_check(r) ? 1 : -1;
}

// Graba y relee para verificar
static bool cred_write(const uint8_t *dev, const uint8_t *join, const uint8_t *key) {
  if (!flash_fs_begin()) return false;
  CredRecord r;
  r.magic = CRED_MAGIC;
  memcpy(r.devEui, dev, 8);
  memcpy(r.joinEui, join, 8);
  memcpy(r.appKey, key, 16);
  r.check = cred_check(r);
  InternalFS.remove(CRED_FILE);
  LfsFile f(InternalFS);
  const bool written = f.open(CRED_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE) &&
                       f.write((const uint8_t *)&r, sizeof(r)) == sizeof(r);
  f.close();
  CredRecord back;
  return written && cred_read(back) == 1 && memcmp(&back, &r, sizeof(r)) == 0;
}

static void cred_load(void) {
  memcpy(DEV_EUI, FW_DEV_EUI, 8);
  memcpy(JOIN_EUI, FW_JOIN_EUI, 8);
  memcpy(APP_KEY, FW_APP_KEY, 16);
  if (!flash_fs_begin()) {
    DEBUG_PRINTLN("[INIT]   Credenciales: flash interna no disponible -> se usan las del firmware");
    return;
  }
  CredRecord r;
  const int st = cred_read(r);
  if (st == 1) {
    memcpy(DEV_EUI, r.devEui, 8);
    memcpy(JOIN_EUI, r.joinEui, 8);
    memcpy(APP_KEY, r.appKey, 16);
    cred_source = "flash";
    DEBUG_PRINT("[INIT]   Credenciales LoRaWAN YA GUARDADAS en la flash: DevEUI ");
    cred_hex(DEV_EUI, 8);
    DEBUG_PRINT(" | JoinEUI ");
    cred_hex(JOIN_EUI, 8);
    DEBUG_PRINT(" | AppKey ");
    cred_key_masked(APP_KEY);
    DEBUG_PRINTLN(" (no se sobreescriben)");
    if (memcmp(r.devEui, FW_DEV_EUI, 8) || memcmp(r.joinEui, FW_JOIN_EUI, 8) || memcmp(r.appKey, FW_APP_KEY, 16)) {
      DEBUG_PRINT("[INIT]   AVISO: las del firmware son otras (DevEUI ");
      cred_hex(FW_DEV_EUI, 8);
      DEBUG_PRINTLN("): se usan las guardadas. Para cambiarlas: LORA:CRED:RESET y reiniciar");
    }
    return;
  }
  if (st < 0) {
    DEBUG_PRINTLN("[INIT]   Credenciales: /lorawan.bin DANADO -> se usan las del firmware sin sobreescribirlo "
                  "(LORA:CRED:RESET y reiniciar para regrabar)");
    return;
  }
#if BOARD_POST_ONLY
  // Firmware de prueba: no se graban (la tarjeta recibe las suyas con el firmware de producción)
  DEBUG_PRINT("[INIT]   Credenciales LoRaWAN: no hay en la flash -> NO se graban (firmware de prueba). DevEUI ");
  cred_hex(FW_DEV_EUI, 8);
  DEBUG_PRINTLN(" solo en RAM (sin Join)");
  return;
#endif
  DEBUG_PRINT("[INIT]   Credenciales LoRaWAN: no hay en la flash -> ESCRIBIENDO las del firmware (DevEUI ");
  cred_hex(FW_DEV_EUI, 8);
  DEBUG_PRINT(" | JoinEUI ");
  cred_hex(FW_JOIN_EUI, 8);
  DEBUG_PRINT(" | AppKey ");
  cred_key_masked(FW_APP_KEY);
  DEBUG_PRINT(")... ");
  const bool ok = cred_write(FW_DEV_EUI, FW_JOIN_EUI, FW_APP_KEY);
  cred_source = ok ? "flash (grabadas en este arranque)" : "firmware";
  DEBUG_PRINTLN(ok ? "guardadas y verificadas" : "ERROR al grabar: se usan las del firmware");
}

void board_lora_cred_status(void) {
  CredRecord r;
  const int st = cred_read(r);
  DEBUG_PRINT("[LORA] Nodo ");
  DEBUG_PRINTLN(board_node_name());
  DEBUG_PRINT("[LORA] En uso (");
  DEBUG_PRINT(cred_source);
  DEBUG_PRINT("): DevEUI ");
  cred_hex(DEV_EUI, 8);
  DEBUG_PRINT(" | JoinEUI ");
  cred_hex(JOIN_EUI, 8);
  DEBUG_PRINT(" | AppKey ");
  cred_hex(APP_KEY, 16);
  DEBUG_PRINTLN("");
  DEBUG_PRINT("[LORA] Flash: ");
  if (st == 1) {
    const bool same = !memcmp(r.devEui, DEV_EUI, 8) && !memcmp(r.joinEui, JOIN_EUI, 8) && !memcmp(r.appKey, APP_KEY, 16);
    DEBUG_PRINTLN(same ? "guardadas, iguales a las en uso" : "guardadas OTRAS (se usan al reiniciar)");
  } else {
    DEBUG_PRINTLN(st < 0 ? "/lorawan.bin danado" : "sin credenciales (al reiniciar se graban las del firmware)");
  }
}

bool board_lora_cred_set(const uint8_t *dev, const uint8_t *join, const uint8_t *key) {
  CredRecord r;
  if (cred_read(r) == 1) {
    DEBUG_PRINTLN("[LORA] Ya hay credenciales guardadas: NO se sobreescriben. Primero LORA:CRED:RESET");
    return false;
  }
  const bool ok = cred_write(dev, join, key);
  DEBUG_PRINTLN(ok ? "[LORA] Credenciales grabadas y verificadas: reinicie el collar para usarlas"
                   : "[LORA] ERROR al grabar las credenciales");
  return ok;
}

void board_lora_cred_reset(void) {
  const bool ok = flash_fs_begin() && (InternalFS.remove(CRED_FILE) || !InternalFS.exists(CRED_FILE));
  DEBUG_PRINTLN(ok ? "[LORA] Credenciales borradas de la flash. Al reiniciar se graban las del firmware (o "
                     "LORA:CRED:SET antes de reiniciar)"
                   : "[LORA] No se pudieron borrar las credenciales");
}

static void identity_load(void);

static void lora_init(void) {
  identity_load(); // credenciales de la flash (o se graban las del firmware la primera vez), si el BLE no lo hizo ya
  LoraConfig cfg;  // por defecto: AU915 sub-banda 2, -10 dB de TX, mensajes [DBG] activos
  cfg.devEui = DEV_EUI;
  cfg.joinEui = JOIN_EUI;
  cfg.appKey = APP_KEY;
  lora.begin(cfg); // el join arranca solo en las primeras llamadas a board_process()
#if BOARD_POST_ONLY
  DEBUG_PRINTLN("[INIT]   Firmware de prueba: LR1110 encendido para el POST, SIN Join ni transmisiones");
#endif
  lora.onDownlink(on_downlink);
  // Nota: Wm1110Hardware::begin() y lora_lib ponen P0.04 (DATA en Wearme) como entrada con
  // pull-down (era DIO1 en el Dev Kit). PIN_LR1110_IRQ ya es P1.08 (LR_DIO9) en el variant.
}
#endif

// ---- Identidad del collar: número de nodo en la flash (/node.txt) ----
// Para saber qué Wearme es: sale al arrancar ("NODO-003"), en el nombre Bluetooth ("WearMe_12_N003", con
// WEARME_HW_VERSION), en el estado del POST por BLE y en el NodeInfo del anillo. Se graba la primera vez como las
// credenciales: el nodo de la tabla (include/wearme_credentials.h) cuyo DevEUI es el que está en uso; después no se
// sobreescribe al cargar firmware. Cambiar de nodo: NODE:SET=<número> (credenciales de la tabla + nodo) y reiniciar.
static const char *NODE_FILE = "/node.txt";
static uint16_t node_num = 0; // 0 = desconocido (credenciales que no están en la tabla)
static char node_name[16] = "";
static char ble_name[32] = "";

// Número del nodo de la tabla con ese DevEUI (0 = no está)
static uint16_t node_for_deveui(const uint8_t *dev) {
  for (int i = 0; i < WEARME_CRED_COUNT; i++) {
    if (memcmp(WEARME_CREDENTIALS[i].devEui, dev, 8) == 0) return WEARME_CREDENTIALS[i].node;
  }
  return 0;
}

// Lee el número guardado; acepta el formato viejo ("NODO-0003" -> 3)
static uint16_t node_read(void) {
  if (!flash_fs_begin()) return 0;
  LfsFile f(InternalFS);
  if (!f.open(NODE_FILE, Adafruit_LittleFS_Namespace::FILE_O_READ)) return 0;
  char buf[16];
  const int n = f.read(buf, sizeof(buf) - 1);
  f.close();
  buf[n > 0 ? n : 0] = '\0';
  unsigned long v = 0;
  for (const char *p = buf; *p; p++) {
    if (*p >= '0' && *p <= '9') v = v * 10 + (unsigned long)(*p - '0');
  }
  return v >= 1 && v <= 999 ? (uint16_t)v : 0;
}

static bool node_write(uint16_t num) {
  if (!flash_fs_begin()) return false;
  char s[16];
  snprintf(s, sizeof(s), "NODO-%03u", num);
  InternalFS.remove(NODE_FILE);
  LfsFile f(InternalFS);
  const bool written = f.open(NODE_FILE, Adafruit_LittleFS_Namespace::FILE_O_WRITE) &&
                       f.write((const uint8_t *)s, strlen(s)) == strlen(s);
  f.close();
  return written && node_read() == num;
}

static void identity_names(void) {
  if (node_num) {
    snprintf(node_name, sizeof(node_name), "NODO-%03u", node_num);
    snprintf(ble_name, sizeof(ble_name), "WearMe_" WEARME_HW_VERSION "_N%03u", node_num);
  } else {
    snprintf(node_name, sizeof(node_name), "NODO-?");
    snprintf(ble_name, sizeof(ble_name), "WearMe_" WEARME_HW_VERSION "_N%04lX",
             (unsigned long)(NRF_FICR->DEVICEID[0] & 0xFFFF));
  }
}

static bool node_saved = false; // el número de nodo está grabado en la flash (no solo el del firmware en RAM)

static void identity_load(void) {
  static bool loaded = false;
  if (loaded) return;
  loaded = true;
#if BOARD_ENABLE_LORA
  cred_load();
  const uint16_t byDev = node_for_deveui(DEV_EUI); // el nodo de las credenciales en uso
#else
  const uint16_t byDev = WEARME_NODE;
#endif
  node_num = node_read();
  node_saved = node_num != 0;
  identity_names();
  if (node_num) {
    DEBUG_PRINT("[INIT]   Nodo Wearme: ");
    DEBUG_PRINT(node_name);
    DEBUG_PRINTLN(" (YA GUARDADO en la flash)");
    if (byDev && byDev != node_num) {
      DEBUG_PRINT("[INIT]   AVISO: las credenciales en uso son del nodo ");
      DEBUG_PRINT(byDev);
      DEBUG_PRINTLN(" en la tabla: revisar con LORA:CRED? (NODE:SET=<nodo> lo corrige)");
    }
  } else {
    node_num = byDev;
    identity_names();
#if BOARD_POST_ONLY
    if (node_num) {
      // Firmware de prueba: no se graba (lo hace el firmware de producción en su primer arranque)
      DEBUG_PRINT("[INIT]   Nodo Wearme: no hay en la flash -> NO se graba (firmware de prueba); se muestra ");
      DEBUG_PRINTLN(node_name);
#else
    if (node_num) {
      DEBUG_PRINT("[INIT]   Nodo Wearme: no hay en la flash -> ESCRIBIENDO ");
      DEBUG_PRINT(node_name);
      DEBUG_PRINTLN(node_write(node_num) ? "... guardado y verificado" : "... ERROR al grabar (se usa igual)");
#endif
    } else {
      DEBUG_PRINTLN("[INIT]   Nodo Wearme: las credenciales en uso NO estan en include/wearme_credentials.h -> "
                    "nodo desconocido (NODE:SET=<nodo> para asignarlo)");
    }
  }
  if (node_num && node_num != WEARME_NODE) {
    DEBUG_PRINT("[INIT]   Firmware compilado para el nodo ");
    DEBUG_PRINT(WEARME_NODE);
    DEBUG_PRINTLN(": este collar conserva su identidad (no se sobreescribe)");
  }
  DEBUG_PRINT("[INIT]   Bluetooth: \"");
  DEBUG_PRINT(ble_name);
  DEBUG_PRINTLN("\"");
}

const char *board_node_name(void) {
  identity_load();
  return node_name;
}

// Solo el número de nodo (nombre y Bluetooth), sin tocar las credenciales: para un collar con credenciales propias
// (LORA:CRED:SET) que no están en la tabla de include/wearme_credentials.h
bool board_node_set_id(uint16_t num) {
  if (num < 1 || num > 999 || !node_write(num)) {
    DEBUG_PRINTLN("[NODO] ERROR al grabar el numero de nodo");
    return false;
  }
  node_num = num;
  node_saved = true;
  identity_names();
  DEBUG_PRINT("[NODO] Numero de nodo grabado: ");
  DEBUG_PRINT(node_name);
  DEBUG_PRINT(" (credenciales sin cambiar) | Bluetooth \"");
  DEBUG_PRINT(ble_name);
  DEBUG_PRINTLN("\"");
  return true;
}

bool board_node_stored(void) {
  identity_load();
  return node_saved;
}

const char *board_ble_name(void) {
  identity_load();
  return ble_name;
}

// Cambia el nodo de este collar: credenciales de la tabla + número de nodo en la flash (a propósito, sí sobreescribe)
bool board_node_set(uint16_t num) {
  int idx = -1;
  for (int i = 0; i < WEARME_CRED_COUNT; i++) {
    if (WEARME_CREDENTIALS[i].node == num) idx = i;
  }
  if (idx < 0) {
    DEBUG_PRINT("[NODO] El nodo ");
    DEBUG_PRINT(num);
    DEBUG_PRINTLN(" no esta en include/wearme_credentials.h");
    return false;
  }
#if BOARD_ENABLE_LORA
  const WearmeCredentials &c = WEARME_CREDENTIALS[idx];
  if (!cred_write(c.devEui, c.joinEui, c.appKey)) {
    DEBUG_PRINTLN("[NODO] ERROR al grabar las credenciales");
    return false;
  }
#endif
  if (!node_write(num)) {
    DEBUG_PRINTLN("[NODO] ERROR al grabar el numero de nodo");
    return false;
  }
  node_num = num;
  node_saved = true;
  identity_names();
  DEBUG_PRINT("[NODO] Este collar ahora es ");
  DEBUG_PRINT(node_name);
  DEBUG_PRINT(" (credenciales de la tabla grabadas y verificadas) | Bluetooth \"");
  DEBUG_PRINT(ble_name);
  DEBUG_PRINTLN("\": REINICIE el collar para usarlas");
  return true;
}

#if BOARD_ENABLE_MESH
// ---- Anillo Meshtastic de respaldo (lib/mesh_lib + lib/mesh_ring) ----
// Ventana sincronizada por hora UTC: todos los collares la abren a la vez (MESH_WINDOW_S cada MESH_WINDOW_PERIOD_S).
// Durante la ventana LBM queda suspendido y el LR1110 escucha en continuo; lo que hay que transmitir (mediciones
// propias, repeticiones, RECLAMO, ENTREGADO, NodeInfo) sale entre medio. Las IRQ del radio no se enrutan a DIO9:
// todo se lee por SPI, así nada le llega al planificador de LBM. Fuera de la ventana el radio es de LoRaWAN.
static MeshtasticTx mesh;
static MeshRing ring;
static bool mesh_ok = false;
static bool win_open = false;
static uint32_t win_end_ms = 0;
static uint32_t win_period = 0xFFFFFFFFu; // período (unix / MESH_WINDOW_PERIOD_S) de la última ventana abierta
static uint32_t win_rx = 0, win_tx = 0;   // paquetes de la ventana (para el log)
static uint32_t win_busy_ms = 0;          // preámbulo detectado: no transmitir encima
static uint32_t mesh_nodeinfo_ms = 0;     // último NodeInfo (0 = ninguno desde el arranque)
static uint32_t mesh_nodeinfo_at = 0;     // NodeInfo pendiente en esta ventana (0 = no)
static uint8_t mesh_buf[MeshtasticTx::MAX_PACKET];
// Diagnóstico (RingDiag): tabla de vecinos siempre; con sondeo (RING:DIAG:<min>, o un PING oído) el radio escucha
// también fuera de las ventanas, por ciclos de DIAG_CYCLE_S alineados a la hora UTC (todos a la vez)
static_assert(MESH_WINDOW_PERIOD_S % DIAG_CYCLE_S == 0, "los ciclos del sondeo deben coincidir con las ventanas");
static RingDiag diag;
static bool diag_open = false;              // el sondeo tiene el radio (fuera de las ventanas)
static uint32_t diag_cycle_id = 0xFFFFFFFFu; // ciclo (unix / DIAG_CYCLE_S) del último PING programado
static uint32_t diag_until_ms = 0;           // fin de la escucha del ciclo actual (después: hueco para LoRaWAN)

// Registro del diagnóstico en la microSD (/datos/anillo.csv): cada evento de RingDiag con la hora local, la posición
// del GPS y el estado de LoRaWAN. Las filas se juntan en RAM y se escriben de una vez con el radio libre (fuera de la
// ventana y de la escucha del sondeo, sin TX/RX de LoRaWAN cerca): la SD bloquea decenas de ms.
static const char DIAG_CSV_HEADER[] =
    "fecha;hora;evento;nodo;nombre;ultimo_salto;saltos;rssi_dbm;snr_db;estado;ida_rssi_dbm;ida_snr_db;ida_saltos;"
    "gps_fix;gps_sats;gps_hdop;lat;lon;alt_m;gps_edad_s;lora_unido;lora_rssi_dbm;lora_snr_db;lora_uplinks;lora_acks;"
    "sondeo";
static char diag_log_buf[6144];
static size_t diag_log_len = 0;
static uint16_t diag_log_lost = 0; // filas que no cupieron en RAM antes de poder escribirlas

static void csv_num(char *&p, const char *end, float v, uint8_t dec) {
  if (p >= end) return;
  const int n = board_has_data(v) ? snprintf(p, end - p, ";%.*f", dec, (double)v) : snprintf(p, end - p, ";");
  p += n > 0 && n < end - p ? n : end - p;
}

static void diag_log(const char *fields) {
  char row[320];
  char *p = row;
  const char *end = row + sizeof(row);
  BoardTime t;
  if (board_time_local(t))
    p += snprintf(p, end - p, "%04u-%02u-%02u;%02u:%02u:%02u;", t.year, t.month, t.day, t.hour, t.minute, t.second);
  else
    p += snprintf(p, end - p, ";;");
  p += snprintf(p, end - p, "%s", fields);
  if (p > end) p = (char *)end;
  // GPS: calidad y satélites solo encendido; la posición es la del último fix (gps_edad_s > 0 = respaldo)
  csv_num(p, end, snap.gpsFix, 0);
  csv_num(p, end, snap.gpsSats, 0);
  csv_num(p, end, gps_powered && gps.fix().quality > 0 ? gps.fix().hdop : NAN, 1);
  csv_num(p, end, snap.gpsLat, 6);
  csv_num(p, end, snap.gpsLon, 6);
  csv_num(p, end, snap.gpsAltM, 1);
  csv_num(p, end, snap.gpsAgeS, 0);
  // LoRaWAN: unido, señal del último downlink del gateway, uplinks y ACK acumulados
  csv_num(p, end, lora.isJoined() ? 1.0f : 0.0f, 0);
  csv_num(p, end, snap.rssiDbm, 0);
  csv_num(p, end, snap.snrDb, 1);
  csv_num(p, end, (float)lora.uplinks(), 0);
  csv_num(p, end, (float)lora.acks(), 0);
  if (p < end) snprintf(p, end - p, ";%s", diag.initiator() ? "sondea" : diag.active() ? "responde" : "");
  const size_t n = strlen(row);
  if (diag_log_len + n + 3 > sizeof(diag_log_buf)) {
    diag_log_lost++;
    return;
  }
  if (diag_log_len) {
    memcpy(diag_log_buf + diag_log_len, "\r\n", 2); // appendCsv agrega el último salto de línea
    diag_log_len += 2;
  }
  memcpy(diag_log_buf + diag_log_len, row, n + 1);
  diag_log_len += n;
}

static void diag_log_flush(void) {
  if (!diag_log_len || win_open || diag_open) return;
  if (lora.isJoined() && lora.idleMs() < 1000) return; // TX/RX de LoRaWAN en curso o cerca
  board_wdt_feed();
  const bool ok = board_sd_append_csv("/datos", "anillo", DIAG_CSV_HEADER, diag_log_buf, nullptr, 0);
  static bool warned = false;
  if (!ok && !warned) DEBUG_PRINTLN("[DIAG] No se pudo escribir /datos/anillo.csv (¿SD?): se descartan las filas");
  warned = !ok;
  if (diag_log_lost) {
    DEBUG_PRINT("[DIAG] SD: ");
    DEBUG_PRINT(diag_log_lost);
    DEBUG_PRINTLN(" filas no cupieron en RAM antes de poder escribirlas");
    diag_log_lost = 0;
  }
  diag_log_len = 0;
}

// Con el sondeo de este collar el GPS sigue encendido (posición de cada fila), aunque ya tenga fix
static bool diag_gps_hold(void) {
  return mesh_ok && diag.initiator();
}

static void mesh_init(void) {
  static const uint8_t psk[] = MESH_PSK;
  MeshtasticTx::Config c;
  c.region = MESH_REGION;
  c.preset = MESH_PRESET;
  c.channelName = MESH_CHANNEL_NAME;
  c.psk = psk;
  c.pskLen = sizeof(psk);
  // Número de nodo = últimos 4 bytes del DevEUI (!d0079096): se reconoce el collar en la app
  c.nodeNum = ((uint32_t)DEV_EUI[4] << 24) | ((uint32_t)DEV_EUI[5] << 16) | ((uint32_t)DEV_EUI[6] << 8) | DEV_EUI[7];
  c.hopLimit = MESH_HOP_LIMIT;
  c.freqHz = MESH_FREQ_HZ;
  uint32_t now = 0;
  c.idSeed = NRF_FICR->DEVICEID[0] ^ micros() ^ (rtc_unix_now(now) ? now : 0);
  const bool aes = MeshtasticTx::aesSelfTest();
  mesh_ok = aes && mesh.begin(c);
  if (!mesh_ok) {
    DEBUG_PRINT("[INIT]   Meshtastic deshabilitado: ");
    DEBUG_PRINTLN(aes ? mesh.error() : "falla la prueba AES-128 (FIPS-197) del nRF52840");
    return;
  }
  MeshRing::Config rc;
  rc.maxAttempts = MESH_MAX_ATTEMPTS;
  rc.unixNow = board_unix_now; // hora del faro HORA
  ring.begin(&mesh, DEV_EUI, rc, c.idSeed ^ 0xA5A5A5A5u);
  char shortName[8];
  snprintf(shortName, sizeof(shortName), "N%03u", node_num); // el mismo nombre corto del NodeInfo
  RingDiag::Config dc;
  dc.lostIdleMs = 2UL * MESH_WINDOW_PERIOD_S * 1000UL + 5UL * 60UL * 1000UL; // 2 ventanas sin oírlo + margen
  dc.log = diag_log;
  diag.begin(&mesh, DEV_EUI, shortName, dc, c.idSeed ^ 0x5A5A5A5Au);
  const MeshRadio &r = mesh.radio();
  DEBUG_PRINT("[INIT]   Anillo Meshtastic: nodo !");
  DEBUG_PRINT(String(mesh.nodeNum(), HEX));
  DEBUG_PRINT(" | " MESH_REGION " " MESH_PRESET " canal \"");
  DEBUG_PRINT(mesh.channelName());
  DEBUG_PRINT("\" slot ");
  DEBUG_PRINT(mesh.channelSlot());
  DEBUG_PRINT(" ");
  DEBUG_PRINT(r.freqHz / 1e6f, 3);
  DEBUG_PRINT(" MHz SF");
  DEBUG_PRINT(r.sf);
  DEBUG_PRINT(" BW");
  DEBUG_PRINT(r.bwKHz);
  DEBUG_PRINT(" hash 0x");
  DEBUG_PRINT(String(mesh.channelHash(), HEX));
  DEBUG_PRINT(" | TX ");
  DEBUG_PRINT(MESH_TX_POWER_DBM);
  DEBUG_PRINT(MESH_TX_POWER_DBM > 14 ? " dBm (PA HP)" : " dBm");
  DEBUG_PRINT(MESH_RX_BOOSTED ? " | RX boosted" : "");
  DEBUG_PRINT(" | AES OK | ventana ");
  DEBUG_PRINT(MESH_WINDOW_S);
  DEBUG_PRINT(" s cada ");
  DEBUG_PRINT(MESH_WINDOW_PERIOD_S);
  DEBUG_PRINT(" s");
  DEBUG_PRINTLN(sizeof(psk) == 1 && psk[0] == 1 ? " | clave por defecto: canal PUBLICO" : "");
}

static ral_lora_bw_t mesh_bw(uint16_t khz) {
  return khz == 125 ? RAL_LORA_BW_125_KHZ : khz == 500 ? RAL_LORA_BW_500_KHZ
                                                       : RAL_LORA_BW_250_KHZ;
}

// Parámetros LoRa del preset; pldLen = largo del paquete a transmitir (255 para recibir)
static void mesh_params(ralf_params_lora_t &p, uint8_t pldLen) {
  const MeshRadio &m = mesh.radio();
  p = {};
  p.mod_params.sf = (ral_lora_sf_t)m.sf;
  p.mod_params.bw = mesh_bw(m.bwKHz);
  p.mod_params.cr = (ral_lora_cr_t)(m.cr - 4);                            // 5 -> RAL_LORA_CR_4_5
  p.mod_params.ldro = ((1UL << m.sf) * 1000UL / m.bwKHz) > 16000 ? 1 : 0; // símbolo > 16 ms
  p.pkt_params.preamble_len_in_symb = m.preamble;
  p.pkt_params.header_type = RAL_LORA_PKT_EXPLICIT;
  p.pkt_params.pld_len_in_bytes = pldLen;
  p.pkt_params.crc_is_on = true;
  p.pkt_params.invert_iq_is_on = false;
  p.rf_freq_in_hz = m.freqHz;
  p.output_pwr_in_dbm = MESH_TX_POWER_DBM;
  p.sync_word = m.syncWord;
}

// Recepción continua (el radio ya es nuestro: LBM suspendido). RX "boosted" del LR1110: ~2 dB más de sensibilidad
// por ~2 mA más mientras escucha (LBM no lo activa; queda puesto también en sus ventanas RX, solo mejora).
static bool mesh_rx_start(void) {
  const ralf_t *radio = LbmWm1110::getInstance().getRadio();
  ralf_params_lora_t p;
  mesh_params(p, MeshtasticTx::MAX_PACKET);
  return ral_set_standby(&radio->ral, RAL_STANDBY_CFG_RC) == RAL_STATUS_OK && ralf_setup_lora(radio, &p) == RAL_STATUS_OK &&
         ral_cfg_rx_boosted(&radio->ral, MESH_RX_BOOSTED) == RAL_STATUS_OK &&
         ral_set_dio_irq_params(&radio->ral, RAL_IRQ_NONE) == RAL_STATUS_OK &&
         ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL) == RAL_STATUS_OK &&
         ral_set_rx(&radio->ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE) == RAL_STATUS_OK;
}

// Potencia del anillo (lib/board_nrf52/mesh_tx_power.c): MESH_TX_POWER_DBM con el PA HP; en SURVIVAL (batería baja)
// los 14 dBm de siempre, que no exigen picos de ~120 mA
extern "C" volatile int8_t board_mesh_tx_dbm;
static int8_t mesh_tx_dbm(void) {
  const bool survival = board_has_data(snap.energyMode) && snap.energyMode < 0.5f; // EnergySupervisor::SURVIVAL = 0
  return survival && MESH_TX_POWER_DBM > 14 ? 14 : MESH_TX_POWER_DBM;
}

static void mesh_print_hex(const char *tag, const uint8_t *d, size_t n) {
#if MESH_DEBUG_HEX
  DEBUG_PRINT(tag);
  char b[3];
  for (size_t i = 0; i < n; i++) {
    snprintf(b, sizeof(b), "%02X", d[i]); // hex limpio: tools/mesh_decode.py --log lo lee tal cual
    DEBUG_PRINT(b);
  }
  DEBUG_PRINTLN("");
#else
  (void)tag;
  (void)d;
  (void)n;
#endif
}

// Transmite con el radio ya reservado y vuelve a recibir. Bloquea el tiempo en el aire (~0.7 s en LongFast).
static bool mesh_tx(const uint8_t *data, size_t len, const char *what) {
  const ralf_t *radio = LbmWm1110::getInstance().getRadio();
  ralf_params_lora_t p;
  mesh_params(p, (uint8_t)len);
  const uint32_t toa = ral_get_lora_time_on_air_in_ms(&radio->ral, &p.pkt_params, &p.mod_params);
  const int8_t dbm = mesh_tx_dbm();
  board_mesh_tx_dbm = dbm; // solo durante la configuración de esta TX (LoRaWAN sigue con el BSP original)
  const bool setup = ral_set_standby(&radio->ral, RAL_STANDBY_CFG_RC) == RAL_STATUS_OK && ralf_setup_lora(radio, &p) == RAL_STATUS_OK;
  board_mesh_tx_dbm = 0;
  bool ok = setup && ral_set_dio_irq_params(&radio->ral, RAL_IRQ_NONE) == RAL_STATUS_OK &&
            ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL) == RAL_STATUS_OK &&
            ral_set_pkt_payload(&radio->ral, data, (uint16_t)len) == RAL_STATUS_OK && ral_set_tx(&radio->ral) == RAL_STATUS_OK;
  bool done = false;
  const uint32_t t0 = millis();
  while (ok && !done && millis() - t0 < toa + 1000) {
    board_wdt_feed();
    delay(5);
    ral_irq_t irq = RAL_IRQ_NONE;
    done = ral_get_irq_status(&radio->ral, &irq) == RAL_STATUS_OK && (irq & RAL_IRQ_TX_DONE);
  }
  win_tx++;
  DEBUG_PRINT("[RING] TX ");
  DEBUG_PRINT(what);
  DEBUG_PRINT(" ");
  DEBUG_PRINT(len);
  DEBUG_PRINT(" B -> ");
  DEBUG_PRINT(done ? "enviado" : (ok ? "sin TX_DONE" : "error al configurar el radio"));
  DEBUG_PRINT(" (aire ");
  DEBUG_PRINT(toa);
  DEBUG_PRINT(" ms, ");
  DEBUG_PRINT(dbm);
  DEBUG_PRINTLN(" dBm)");
  mesh_print_hex("[RING] hex ", data, len);
  mesh_rx_start();
  return done;
}

// ---- Hora por la mesh (como Meshtastic) ----
// Un faro HORA lleva la hora UTC del collar que lo manda y su calidad. Se usa solo si este collar no tiene hora, la
// tomó de la mesh o es la del RTC conservado (esta última solo con un faro de hora buena); la de la red, el GPS o la
// puesta a mano nunca se pisan. +1 s: el faro sale
// con la hora truncada al segundo y tarda ~0.5 s en el aire.
static uint32_t mesh_time_ms = 0; // última vez que se tomó la hora de un faro (0 = nunca)

static void time_from_mesh(uint32_t unix, uint8_t quality, const uint8_t from[8]) {
  if (time_source > TIME_RTC || quality == MeshRing::TIME_Q_NONE) return;
  mesh_time_ms = millis() | 1; // aunque no haga falta corregir: ya hay un faro cerca, este collar no manda el suyo
  // Con hora de la mesh o del RTC conservado solo corrige un faro con hora buena (red, GPS o serie/BLE en el otro)
  if (time_source != TIME_NONE && quality < MeshRing::TIME_Q_GOOD) return;
  unix += 1;
  uint32_t now = 0;
  const bool have = rtc_unix_now(now);
  const int32_t diff = have ? (int32_t)(unix - now) : 0;
  if (time_source != TIME_NONE && have && diff >= -2 && diff <= 2) return;
  Rx8130::DateTime t;
  Rx8130::fromUnix(unix, t);
  if (!rtc.setTime(t)) return;
  time_source_set(TIME_MESH);
  DEBUG_PRINT("[RTC] Hora de la mesh (faro de ");
  char dev[17];
  for (uint8_t i = 0; i < 8; i++) snprintf(dev + 2 * i, 3, "%02X", from[i]);
  DEBUG_PRINT(dev);
  DEBUG_PRINT(quality == MeshRing::TIME_Q_GOOD ? ", hora buena): " : ", hora de la mesh): ");
  rtc_print_time(t);
  if (have) {
    DEBUG_PRINT(" | correccion ");
    DEBUG_PRINT(diff);
    DEBUG_PRINT(" s");
  }
  DEBUG_PRINTLN("");
}

// Collar SIN hora (no se unió a LoRaWAN, sin fix de GPS, nadie se la puso): no sabe cuándo son las ventanas, así
// que escucha hasta MESH_TIME_ACQ_MS (cubre al menos una ventana) esperando un faro HORA. Si no oye ninguno, lo
// vuelve a intentar a los MESH_TIME_RETRY_MS (escuchar consume ~6 mA). Unido a LoRaWAN no hace falta: la red le da
// la hora (DeviceTimeReq). Para no impedir el join (que da la hora de la red): empieza MESH_TIME_ACQ_DELAY_MS después
// de arrancar y escucha por tramos (MESH_TIME_ACQ_LISTEN_MS escuchando, MESH_TIME_ACQ_PAUSE_MS con el radio libre
// para LBM). Sin esto un collar con cobertura se quedaba 11 min sin poder hacer join (prueba del 2026-10-21).
static bool acq_session = false, acq_open = false;
static uint32_t acq_end_ms = 0, acq_next_ms = 0, acq_slice_ms = 0;

static void mesh_acq_release(void) {
  const ralf_t *radio = LbmWm1110::getInstance().getRadio();
  ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL);
  ral_set_sleep(&radio->ral, true);
  smtc_modem_resume_after_user_radio_access();
  acq_open = false;
}

static void mesh_time_acquire(void) {
  const ralf_t *radio = LbmWm1110::getInstance().getRadio();
  const uint32_t now = millis();
  const bool need = mesh_ok && !rtc_trusted && !lora.isJoined() && now >= MESH_TIME_ACQ_DELAY_MS;
  if (!need) {
    if (acq_open) mesh_acq_release();
    acq_session = false;
    return;
  }
  if (!acq_session) {
    if (acq_next_ms && (int32_t)(now - acq_next_ms) < 0) return;
    acq_session = true;
    acq_end_ms = now + MESH_TIME_ACQ_MS;
    acq_slice_ms = now; // primer tramo ya
    DEBUG_PRINT("[RING] Sin hora confiable: escuchando la mesh hasta ");
    DEBUG_PRINT(MESH_TIME_ACQ_MS / 60000UL);
    DEBUG_PRINTLN(" min (por tramos, dejando hueco al join) para tomar la hora de otro collar (faro HORA)");
  }
  if ((int32_t)(now - acq_end_ms) >= 0) { // fin de la sesión sin faro
    if (acq_open) mesh_acq_release();
    acq_session = false;
    acq_next_ms = now + MESH_TIME_RETRY_MS;
    DEBUG_PRINT("[RING] Ningun faro HORA en la escucha: se reintenta en ");
    DEBUG_PRINT(MESH_TIME_RETRY_MS / 60000UL);
    DEBUG_PRINTLN(" min (o con la red, el GPS o RTC:SET)");
    return;
  }
  if (!acq_open) {
    if ((int32_t)(now - acq_slice_ms) < 0) return; // pausa: el radio es de LBM
    if (smtc_modem_suspend_before_user_radio_access() != SMTC_MODEM_RC_OK) return;
    if (!mesh_rx_start()) {
      mesh_acq_release();
      acq_slice_ms = now + MESH_TIME_ACQ_PAUSE_MS;
      return;
    }
    acq_open = true;
    acq_slice_ms = now + MESH_TIME_ACQ_LISTEN_MS;
    return;
  }
  if ((int32_t)(now - acq_slice_ms) >= 0) { // fin del tramo: hueco para LBM
    mesh_acq_release();
    acq_slice_ms = now + MESH_TIME_ACQ_PAUSE_MS;
    return;
  }
  ral_irq_t irq = RAL_IRQ_NONE;
  if (ral_get_irq_status(&radio->ral, &irq) == RAL_STATUS_OK && irq != RAL_IRQ_NONE) {
    if (irq & RAL_IRQ_RX_DONE) {
      uint16_t len = 0;
      const bool got = ral_get_pkt_payload(&radio->ral, sizeof(mesh_buf), mesh_buf, &len) == RAL_STATUS_OK;
      ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL);
      uint32_t beacon = 0;
      uint8_t q = 0, from[8];
      if (got && !(irq & RAL_IRQ_RX_CRC_ERROR) && ring.parseTime(mesh_buf, len, beacon, q, from)) {
        time_from_mesh(beacon, q, from); // con hora, la próxima vuelta cierra la escucha y abre las ventanas
      }
    } else if (irq & (RAL_IRQ_RX_CRC_ERROR | RAL_IRQ_RX_HDR_ERROR)) {
      ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL);
    }
  }
}

// Libera el radio que tenía el sondeo (fin de la escucha del ciclo o una ventana que va a abrir)
static void mesh_diag_release(void) {
  if (!diag_open) return;
  const ralf_t *radio = LbmWm1110::getInstance().getRadio();
  ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL);
  ral_set_sleep(&radio->ral, true);
  smtc_modem_resume_after_user_radio_access();
  diag_open = false;
  diag.sliceEnd();
}

// Paquete que llegó dañado (CRC) o con la cabecera ilegible: antes se descartaba sin aviso. Con SNR bajo (< -12 dB)
// es el borde del alcance; con SNR bueno, un choque con otro paquete o interferencia.
static uint16_t win_crc = 0, win_hdr = 0; // de la ventana (o del sondeo desde la última ventana)
static void mesh_rx_error(bool crc, int16_t rssi, int16_t snr) {
  if (crc) win_crc++;
  else win_hdr++;
  char line[120];
  snprintf(line, sizeof(line), "[DIAG] RX con error de %s | RSSI %d dBm SNR %d dB -> %s", crc ? "CRC" : "cabecera", rssi,
           snr, snr < -12 ? "borde del alcance" : "choque o interferencia (senal suficiente)");
  DEBUG_PRINTLN(line);
  snprintf(line, sizeof(line), "%s;;;;;%d;%d;%s;;;", crc ? "ERROR CRC" : "ERROR CABECERA", rssi, snr,
           snr < -12 ? "BORDE" : "CHOQUE");
  diag_log(line);
}

// Recepción continua (ventana o sondeo): lee la IRQ por SPI y entrega el paquete al diagnóstico y, en la ventana,
// al anillo. No bloquea.
static void mesh_rx_poll(uint32_t now, bool inWindow) {
  const ralf_t *radio = LbmWm1110::getInstance().getRadio();
  ral_irq_t irq = RAL_IRQ_NONE;
  if (ral_get_irq_status(&radio->ral, &irq) != RAL_STATUS_OK || irq == RAL_IRQ_NONE) return;
  if (irq & RAL_IRQ_RX_DONE) {
    uint16_t len = 0;
    ral_lora_rx_pkt_status_t st = {};
    const bool got = ral_get_pkt_payload(&radio->ral, sizeof(mesh_buf), mesh_buf, &len) == RAL_STATUS_OK &&
                     ral_get_lora_rx_pkt_status(&radio->ral, &st) == RAL_STATUS_OK;
    ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL);
    win_busy_ms = 0;
    if (got && (irq & RAL_IRQ_RX_CRC_ERROR)) mesh_rx_error(true, st.rssi_pkt_in_dbm, st.snr_pkt_in_db);
    if (got && !(irq & RAL_IRQ_RX_CRC_ERROR) && len > 0) {
      win_rx++;
      mesh_print_hex("[RING] RX hex ", mesh_buf, len);
      // Señal por nodo, rutas y sondeo (todo lo que se oye, también duplicados: cada uno es un enlace distinto)
      diag.onPacket(mesh_buf, len, st.rssi_pkt_in_dbm, st.snr_pkt_in_db, now);
      if (!inWindow) return;
      ring.onPacket(mesh_buf, len, st.rssi_pkt_in_dbm, st.snr_pkt_in_db, now);
      // Comando para este collar llegado por el anillo: se ejecuta igual que un downlink de LoRaWAN
      uint8_t cmd[RING_CMD_MAX], cmdLen = 0;
      if (ring.takeCommand(cmd, cmdLen)) board_inject_downlink(cmd, cmdLen, "anillo Meshtastic");
      // Faro HORA de otro collar: corrige una hora tomada de la mesh (nunca una mejor)
      uint32_t beacon = 0;
      uint8_t q = 0, from[8];
      if (ring.takeTime(beacon, q, from)) time_from_mesh(beacon, q, from);
    }
  } else if (irq & (RAL_IRQ_RX_CRC_ERROR | RAL_IRQ_RX_HDR_ERROR)) {
    ral_lora_rx_pkt_status_t st = {};
    const bool got = ral_get_lora_rx_pkt_status(&radio->ral, &st) == RAL_STATUS_OK;
    ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL);
    win_busy_ms = 0;
    if (got) mesh_rx_error((irq & RAL_IRQ_RX_CRC_ERROR) != 0, st.rssi_pkt_in_dbm, st.snr_pkt_in_db);
  } else if (irq & (RAL_IRQ_RX_PREAMBLE_DETECTED | RAL_IRQ_RX_HDR_OK)) {
    // Llega un paquete: no transmitir encima. Una detección falsa se olvida a los 1.5 s.
    if (!win_busy_ms)
      win_busy_ms = now | 1;
    else if (now - win_busy_ms > 1500) {
      ral_clear_irq_status(&radio->ral, RAL_IRQ_RX_PREAMBLE_DETECTED | RAL_IRQ_RX_HDR_OK);
      win_busy_ms = 0;
    }
  }
}

// Sondeo de diagnóstico fuera de las ventanas: escucha por ciclos de DIAG_CYCLE_S alineados a la hora UTC (el que
// sondea manda el PING al principio; los que responden ya están escuchando) y deja los últimos DIAG_PAUSE_S de cada
// ciclo con el radio libre para LoRaWAN. Dentro de la ventana del anillo lo atiende mesh_window_service.
static void mesh_diag_service(void) {
  if (!mesh_ok) return;
  const uint32_t now = millis();
  diag.tick(now);
  if (win_open) return;
  uint32_t unix = 0;
  const bool haveTime = rtc_trusted && board_unix_now(unix);
  const uint32_t phaseW = unix % MESH_WINDOW_PERIOD_S, ph = unix % DIAG_CYCLE_S;
  const bool listen = diag.active() && haveTime && phaseW >= MESH_WINDOW_S && phaseW + 3 < MESH_WINDOW_PERIOD_S &&
                      ph < DIAG_CYCLE_S - DIAG_PAUSE_S;
  if (!listen) {
    mesh_diag_release();
    diag_log_flush(); // radio libre: las filas del registro a la SD
    return;
  }
  if (!diag_open) {
    // No cortar un uplink en curso (TX, RX1, RX2): se escucha cuando LBM termine
    if (board_lora_tx_in_flight() || (lora.isJoined() && lora.idleMs() < 1000)) return;
    if (smtc_modem_suspend_before_user_radio_access() != SMTC_MODEM_RC_OK) return;
    if (!mesh_rx_start()) {
      const ralf_t *radio = LbmWm1110::getInstance().getRadio();
      ral_set_sleep(&radio->ral, true);
      smtc_modem_resume_after_user_radio_access();
      return;
    }
    diag_open = true;
    win_busy_ms = 0;
  }
  diag_until_ms = now + (DIAG_CYCLE_S - DIAG_PAUSE_S - ph) * 1000UL;
  // Un PING por ciclo, solo si la escucha empezó al principio del ciclo (cuando todos están escuchando)
  const uint32_t cyc = unix / DIAG_CYCLE_S;
  if (cyc != diag_cycle_id && ph <= 3) {
    diag_cycle_id = cyc;
    diag.cycle(now, 300 + micros() % 700);
  }
  mesh_rx_poll(now, false);
  if (!win_busy_ms) {
    const char *what = "";
    const size_t len = diag.nextTx(now, diag_until_ms, mesh_buf, sizeof(mesh_buf), what);
    if (len) mesh_tx(mesh_buf, len, what);
  }
}

// Abre la ventana del período actual, atiende RX/TX y la cierra. No bloquea salvo durante una TX.
static void mesh_window_service(void) {
  if (!mesh_ok || !rtc_trusted) return;
  uint32_t unix = 0;
  if (!board_unix_now(unix)) return;
  const uint32_t period = unix / MESH_WINDOW_PERIOD_S, phase = unix % MESH_WINDOW_PERIOD_S;
  const ralf_t *radio = LbmWm1110::getInstance().getRadio();

  if (!win_open) {
    // Una ventana por período; no se abre si ya casi se acaba
    if (period == win_period || phase + 5 >= MESH_WINDOW_S) return;
    mesh_diag_release(); // el sondeo suelta el radio (la ventana lo atiende)
    // LoRaWAN unido: no cortar una TX/RX en curso (se reintenta en las vueltas siguientes de la misma ventana)
    if (lora.isJoined() && lora.idleMs() < 1000) return;
    if (smtc_modem_suspend_before_user_radio_access() != SMTC_MODEM_RC_OK) return;
    win_period = period;
    if (!mesh_rx_start()) {
      ral_set_sleep(&radio->ral, true);
      smtc_modem_resume_after_user_radio_access();
      DEBUG_PRINTLN("[RING] No se pudo poner el radio en recepcion: ventana perdida");
      return;
    }
    const uint32_t now = millis();
    win_open = true;
    win_rx = win_tx = 0;
    win_crc = win_hdr = 0;
    win_busy_ms = 0;
    win_end_ms = now + (MESH_WINDOW_S - phase) * 1000UL;
    const bool bridge = lora.isJoined();
    // Faro HORA: con hora buena siempre; con hora de la mesh solo si no oyó un faro en el último período (si lo oyó,
    // ya hay quien la dé en la zona y el suyo sobra; pasó en la prueba del 2026-10-21 15:30)
    const bool heardBeacon = mesh_time_ms && now - mesh_time_ms < MESH_WINDOW_PERIOD_S * 1000UL;
    // Hora "buena" solo si una fuente real la puso en este arranque (red, GPS, serie/BLE). La del RTC conservado no:
    // un RTC que estuvo detenido sin marcar VLF desfasaba a todo el anillo (2026-11-03); va como hora de la mesh.
    const uint8_t timeQ = time_source >= TIME_MANUAL ? MeshRing::TIME_Q_GOOD
                          : heardBeacon              ? MeshRing::TIME_Q_NONE
                                                     : MeshRing::TIME_Q_MESH;
    ring.windowStart(now, win_end_ms - now, bridge, board_has_data(snap.rssiDbm) ? (int16_t)snap.rssiDbm : 0, timeQ);
    // NodeInfo ("WearMe xxxx" en la app Meshtastic) al final de la ventana, cada MESH_NODEINFO_PERIOD_MS
    mesh_nodeinfo_at = (mesh_nodeinfo_ms == 0 || now - mesh_nodeinfo_ms >= MESH_NODEINFO_PERIOD_MS) && win_end_ms - now > 12000
                         ? win_end_ms - 8000 - (micros() % 3000)
                         : 0;
    DEBUG_PRINT("[RING] Ventana abierta ");
    DEBUG_PRINT((win_end_ms - now) / 1000);
    DEBUG_PRINT(" s | ");
    DEBUG_PRINT(ring.bridge() ? "PUENTE (LoRaWAN unido)"
                : bridge      ? "unido pero sin ACK reciente: no es puente"
                              : "sin LoRaWAN");
    DEBUG_PRINT(" | mediciones propias pendientes ");
    DEBUG_PRINT(ring.ownPending());
    DEBUG_PRINTLN(diag.initiator() ? " | sondeo: PING en esta ventana (suma a los collares que lo oigan)" : "");
    // Sondeo: el PING de la ventana después de los faros, ENTREGADO, COMANDOS y la mayoría de las MEDICIONES
    const uint32_t left = win_end_ms - now;
    diag.cycle(now, left > 26000 ? 16000 + micros() % 2000 : left / 2);
    return;
  }

  const uint32_t now = millis();
  mesh_rx_poll(now, true);

  if (!win_busy_ms) {
    const char *what = "";
    size_t len = ring.nextTx(now, mesh_buf, sizeof(mesh_buf), what);
    if (!len) len = diag.nextTx(now, win_end_ms - 1500, mesh_buf, sizeof(mesh_buf), what);
    if (len) {
      mesh_tx(mesh_buf, len, what);
    } else if (mesh_nodeinfo_at && (int32_t)(now - mesh_nodeinfo_at) >= 0) {
      char longName[24], shortName[8];
      // "WearMe NODO-003" y el corto "N003" (Meshtastic: hasta 4 caracteres)
      snprintf(shortName, sizeof(shortName), "N%03u", node_num);
      snprintf(longName, sizeof(longName), "WearMe %s", node_name);
      const size_t n = mesh.nodeInfo(longName, shortName, mesh_buf, sizeof(mesh_buf));
      mesh_nodeinfo_at = 0;
      if (n && mesh_tx(mesh_buf, n, "NodeInfo")) mesh_nodeinfo_ms = now | 1;
    }
  }

  if ((int32_t)(millis() - win_end_ms) >= 0) {
    ral_clear_irq_status(&radio->ral, RAL_IRQ_ALL);
    ral_set_sleep(&radio->ral, true);
    smtc_modem_resume_after_user_radio_access();
    win_open = false;
    ring.windowEnd(millis());
    diag.sliceEnd();
    DEBUG_PRINT("[RING] Ventana cerrada | RX ");
    DEBUG_PRINT(win_rx);
    DEBUG_PRINT(" TX ");
    DEBUG_PRINT(win_tx);
    DEBUG_PRINT(" | danados: CRC ");
    DEBUG_PRINT(win_crc);
    DEBUG_PRINT(" cabecera ");
    DEBUG_PRINTLN(win_hdr);
    // Sin sondeo: la tabla de vecinos con lo oído hasta ahora (en el sondeo sale cada RingDiag::Config::tableEvery PING)
    if (!diag.initiator() && diag.count()) diag.printTable(millis());
  }
}
#endif

#if BOARD_ENABLE_GNSS
static void gnss_init(void) {
  GnssTrackerConfig gcfg; // NAV en FPort 192, reporte doppler en FPort 3
  gcfg.statsFport = 0;    // sin estadisticas GNSS por FPort 2 (solo se ven por Serial)
  gcfg.dopplerFport = 3;  // satelites de cada escaneo (id, C/N0, doppler) para Node-RED
  gcfg.sensorPeriodMs = UPLINK_PERIOD_MS;
  gcfg.scanPeriodMs = GNSS_PERIOD_MS;
  gcfg.minSatellites = 6;  // NAV solo con 6+ satelites (con 3-4 el error es grande; afuera se ven ~7)
  gcfg.maxSatellites = 11; // tope del escaneo: NAV de ~53 B, el maximo a DR3/SF9
  gcfg.assist = true;      // posicion aproximada para el escaneo asistido (PUJ Cali)
  gcfg.assistLatitude = 3.349f;
  gcfg.assistLongitude = -76.531f;
  gnss.begin(gcfg);
}
#endif

#if BOARD_ENABLE_BLE
// ---- App por BLE (lib/ble_lib, protocolo del firmware V12) ----
// Número para JSON: "null" si no hay dato (NAN no es JSON válido)
static const char *json_num(char *buf, size_t size, float v, uint8_t decimals) {
  if (board_has_data(v))
    snprintf(buf, size, "%.*f", decimals, (double)v);
  else
    snprintf(buf, size, "null");
  return buf;
}

// Estado del POST para la app: 1 = falla. APPKEY va vacía a propósito (el UART por BLE no tiene autenticación).
// FSR_R/FSR_L/MUX comparten POST_WIND; MASK es board_fault_mask() completa (bit n = código POST n).
static void ble_send_post_status(void) {
  const uint32_t m = board_fault_mask();
  char id[20];
  board_device_id(id, sizeof(id));
  const unsigned wind = (m >> POST_WIND) & 1u;
  char line[256];
  snprintf(line, sizeof(line),
           "{\"POST\":\"%s\",\"DEVEUI\":\"%s\",\"APPKEY\":\"\",\"ADC\":%u,\"BME\":%u,\"MUX\":%u,\"FSR_R\":%u,"
           "\"FSR_L\":%u,\"IMU\":%u,\"LORA\":%u,\"RTC\":%u,\"SD\":%u,\"MASK\":%lu,\"NODE\":\"%s\"}",
           m ? "FAIL" : "OK", id, (unsigned)((m >> POST_SENSOR_CH4_H2) & 1u), (unsigned)((m >> POST_ENVIRONMENT) & 1u),
           wind, wind, wind, (unsigned)((m >> POST_IMU) & 1u), (unsigned)((m >> POST_LORA_COMM) & 1u),
           (unsigned)((m >> POST_RTC) & 1u), (unsigned)((m >> POST_SD_CARD) & 1u), (unsigned long)m, node_name);
  ble_send_line(line);
}

// Comandos de la app. "3" (V12: medir) se traduce a 01, "1"/"0" (V12: TEST / parar) a LIVE/STOP; el resto pasa tal cual como downlink y la FSM acepta
// "MEASURE" y "POST" (los demás los ignora). Los comandos de depuración del puerto serie (WDT:, POLICY:,
// CH4:BURNIN...) no se aceptan por BLE: cualquiera en alcance puede conectarse.
static void ble_service(void) {
  if (ble_take_subscribed()) {
    DEBUG_PRINTLN("[BLE] App suscrita: se envia el estado del POST");
    ble_send_post_status();
  }
  char cmd[BLE_RX_MAX];
  if (!ble_read_line(cmd, sizeof(cmd))) return;
  DEBUG_PRINT("[BLE] Comando: ");
  DEBUG_PRINTLN(cmd);

  if (strncmp(cmd, "RTC:", 4) == 0) {
    unsigned y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
    if (sscanf(cmd + 4, "%u/%u/%u %u:%u:%u", &d, &mo, &y, &h, &mi, &s) == 6 ||     // app V12
        sscanf(cmd + 4, "SET %u-%u-%u %u:%u:%u", &y, &mo, &d, &h, &mi, &s) == 6) { // mismo formato que la serie
      board_rtc_set_local(y, mo, d, h, mi, s);
    } else {
      DEBUG_PRINTLN("[BLE] Formato: RTC:DD/MM/AAAA HH:MM:SS o RTC:SET AAAA-MM-DD HH:MM:SS (hora local)");
    }
    return;
  }
  if (strcmp(cmd, "3") == 0) {
    const uint8_t measure = 0x01;
    board_inject_downlink(&measure, 1, "BLE");
    return;
  }
  if (strcmp(cmd, "1") == 0 || strcmp(cmd, "0") == 0) { // V12: "1" = TEST (vista en vivo), "0" = parar
    const char *text = cmd[0] == '1' ? "LIVE" : "STOP";
    board_inject_downlink((const uint8_t *)text, strlen(text), "BLE");
    return;
  }
  if (strcmp(cmd, "2") == 0) { // app: "terminar la configuración y seguir recibiendo": la trama sigue sola
    DEBUG_PRINTLN("[BLE] Configuracion terminada: la trama sigue cada 1 s mientras la app este conectada");
    return;
  }
  board_inject_downlink((const uint8_t *)cmd, strlen(cmd), "BLE");
}
#endif

void board_init_step(BoardInitStepId step) {
  switch (step) {
    case BOARD_INIT_SAFE_GPIO:
      DEBUG_PRINTLN("[INIT] Salidas en estado seguro (HT_PWM P0.07, FAN P0.28, LNA P0.29 en LOW; CS_SD P0.26 en HIGH)");
      safe_gpio_init();
      break;
    case BOARD_INIT_LEDS:
      DEBUG_PRINTLN("[INIT] LEDs de estado (verde P0.02, rojo P0.03, azul P0.31): los tres encendidos hasta el POST");
      leds.begin();
      leds.booting();
      break;
    case BOARD_INIT_I2C:
      DEBUG_PRINTLN("[INIT] Bus I2C (TCA4307)");
      i2c_init();
      break;
    case BOARD_INIT_IO_EXPANDER:
      DEBUG_PRINTLN("[INIT] Expansor de E/S TCA6408A");
      io_expander_init();
      break;
    case BOARD_INIT_POWER_RAILS:
      DEBUG_PRINTLN("[INIT] Interruptores de carga SIP32431");
      power_rails_init();
      break;
    case BOARD_INIT_SD:
      DEBUG_PRINTLN("[INIT] microSD (SPIM2: MISO P0.30, SCK P0.27, MOSI P0.05, CS P0.26)");
      sd.begin();
      sd_print_status("[INIT]   ");
      break;
    case BOARD_INIT_IMU:
      DEBUG_PRINTLN("[INIT] IMU LSM6DS3TR-C (I2C2 0x6A)");
      imu_init();
      break;
    case BOARD_INIT_ENVIRONMENT:
      DEBUG_PRINTLN("[INIT] Sensor ambiental BME280 (I2C2 0x76, tarjeta MEMS)");
      env_init();
      break;
    case BOARD_INIT_SENSOR_CH4_H2:
      DEBUG_PRINTLN("[INIT] Sensores CH4/H2: MCP4018 (carga CH4) + ADS1219 (AIN0 = H2, AIN1 = CH4)");
      mems_init();
      break;
    case BOARD_INIT_GPS:
      DEBUG_PRINTLN("[INIT] GPS Quectel L96-M33 (UART 9600, tarjeta RF)");
      gps_init();
      break;
    case BOARD_INIT_WIND:
      DEBUG_PRINTLN("[INIT] Viento: TCA9543A + 2 x FS3000-1015");
      wind_init();
      break;
    case BOARD_INIT_LORA:
#if BOARD_ENABLE_LORA
      DEBUG_PRINTLN("[INIT] Nodo LoRaWAN (OTAA)");
      lora_init();
#if BOARD_ENABLE_MESH
      mesh_init();
#endif
#else
      DEBUG_PRINTLN("[INIT] Nodo LoRaWAN -> deshabilitado (BOARD_ENABLE_LORA, sin antena)");
#endif
      break;
    case BOARD_INIT_GNSS:
#if BOARD_ENABLE_GNSS
      DEBUG_PRINTLN("[INIT] Rastreador GNSS (LR1110)");
      gnss_init();
#else
      DEBUG_PRINTLN("[INIT] Rastreador GNSS -> deshabilitado (BOARD_ENABLE_GNSS)");
#endif
      break;
    case BOARD_INIT_BATTERY:
      DEBUG_PRINTLN("[INIT] Bateria: medidor MAX17048 (I2C 0x36)");
      battery_init();
      break;
    case BOARD_INIT_BLOWER:
      DEBUG_PRINTLN("[INIT] Blower (FAN_CONTROL P0.28 + tacometro)");
      blower_init();
      break;
    case BOARD_INIT_BLUETOOTH:
#if BOARD_ENABLE_BLE
      identity_load(); // nombre del nodo (y credenciales): el Bluetooth arranca antes que LoRa
      DEBUG_PRINT("[INIT] Bluetooth: UART por BLE, anunciando como \"");
      DEBUG_PRINT(ble_name);
      DEBUG_PRINT("\"");
      DEBUG_PRINTLN(ble_begin(ble_name) ? "" : " -> ERROR: el SoftDevice no arranco");
#else
      DEBUG_PRINTLN("[INIT] Bluetooth -> deshabilitado (BOARD_ENABLE_BLE)");
#endif
      break;
    case BOARD_INIT_RTC:
      DEBUG_PRINTLN("[INIT] RTC RX8130CE (I2C2 0x32)");
      rtc_init();
      break;
    default:
      DEBUG_PRINTLN("[INIT] Paso de inicializacion no reconocido");
      break;
  }
}

// ============================================
// 6. Pruebas para el POST
// ============================================
HwStatus board_test_i2c(void) {
  HwStatus status = HW_PASS;
  if (i2c1_ready) {
    // En I2C1 siempre está el TCA6408A: un bus vacío es un problema de montaje
    if (i2c_scan(Wire, "[POST]   ", "I2C1") == 0) {
      DEBUG_PRINTLN("[POST]   I2C1 sin ningun dispositivo: TCA6408A sin soldar o sin alimentacion, SDA/SCL abiertos");
      status = HW_FAIL;
    }
  } else {
    DEBUG_PRINTLN("[POST]   I2C1 no iniciado (SDA/SCL en bajo en el arranque)");
    status = HW_FAIL;
  }

  if (!i2c2_ready) {
    DEBUG_PRINTLN("[POST]   I2C2 no iniciado (SDA/SCL en bajo en el arranque)");
    return HW_FAIL;
  }
  if (WM_PIN_I2C_BUF_READY != BOARD_PIN_NC && digitalRead(WM_PIN_I2C_BUF_READY) == LOW) {
    DEBUG_PRINTLN("[POST]   TCA4307 READY en bajo: lado de dispositivos desconectado");
    return HW_FAIL;
  }
  // Detrás del TCA4307 siempre están el MAX17048, el RX8130CE y el LSM6DS3TR-C de la placa principal: si no responde
  // ninguno, el TCA4307 no conecta el lado de dispositivos (y fallan batería, RTC, IMU y la tarjeta MEMS por lo mismo)
  if (i2c_scan(Wire1, "[POST]   ", "I2C2 (detras del TCA4307)") == 0) {
    DEBUG_PRINTLN("[POST]   I2C2 sin ningun dispositivo: el TCA4307 no conecta. Revisar el TCA4307, los pull-ups de "
                  "SDA_2_OUT/SCL_2_OUT y que ninguna linea de ese lado este a GND (un chip sin alimentacion la baja)");
    return HW_FAIL;
  }
  return status;
}

HwStatus board_test_io_expander(void) {
  if (!i2c1_ready) return HW_SKIP;
  uint8_t cfg = 0;
  if (!expander.present() || !expander.probe() || !expander.readReg(Tca6408::REG_CONFIG, cfg)) {
    DEBUG_PRINTLN("[POST]   TCA6408A no responde");
    return HW_FAIL;
  }
  if (cfg != expander.config()) {
    DEBUG_PRINTLN("[POST]   TCA6408A perdio la configuracion (reset o caida de alimentacion)");
    return HW_FAIL;
  }
  return HW_PASS;
}

HwStatus board_test_power_rails(void) {
  if (POWER_RAILS[0].name == nullptr) {
    DEBUG_PRINTLN("[POST]   Tabla de SIP32431 vacia");
    return HW_SKIP;
  }
  if (!expander.present()) {
    DEBUG_PRINTLN("[POST]   Sin TCA6408A: no se pueden verificar los SIP32431");
    return HW_FAIL;
  }
  // CTRL_GPS (P0): con R154 (0 Ω a VDD) o EN en corto a VDD el pin no baja. Encendido (así arranca) eso no se ve y
  // la prueba daba OK al arrancar y FALLA después de apagar el GPS: se baja 0.5 ms para comprobar que obedece
  const uint8_t gpsMask = 1u << 0;
  if (!(rails_nodrive_mask() & gpsMask) && (expander.config() & gpsMask) == 0 && (expander.outputs() & gpsMask)) {
    uint8_t probe = 0;
    bool read = expander.write(0, false);
    if (read) {
      delayMicroseconds(500);
      read = expander.readInputs(probe);
    }
    expander.write(0, true);
    if (read && (probe & gpsMask)) rails_stuck_mask |= gpsMask;
  }
  uint8_t out = 0, cfg = 0, in = 0;
  if (!expander.readReg(Tca6408::REG_OUTPUT, out) || !expander.readReg(Tca6408::REG_CONFIG, cfg) || !expander.readInputs(in)) {
    DEBUG_PRINTLN("[POST]   Sin TCA6408A: no se pueden verificar los SIP32431");
    return HW_FAIL;
  }
  HwStatus status = HW_PASS;
  for (const PowerRail *r = POWER_RAILS; r->name != nullptr; r++) {
    const uint8_t mask = 1u << r->expanderBit;
    if (rails_fixed_mask & mask) {
      DEBUG_PRINT("[POST]   SIP32431 ");
      DEBUG_PRINT(r->name);
      DEBUG_PRINTLN((in & mask) ? " fijo a VDD (R154) OK" : " fijo a VDD (R154) pero se lee BAJO");
      if (!(in & mask)) status = HW_FAIL;
      continue;
    }
    // Pedido en BAJO y se lee ALTO (ahora o antes en este arranque): pegado a VDD. Se suelta el pin (entrada) para
    // no forzar la salida del TCA6408A contra VDD y queda como falla hasta reiniciar
    const bool isOutputNow = (cfg & mask) == 0;
    if (!(rails_stuck_mask & mask) && isOutputNow && !(out & mask) && (in & mask)) rails_stuck_mask |= mask;
    if (rails_stuck_mask & mask) {
      if (isOutputNow) expander.pinsInput(mask);
      const bool tolerated = r->expanderBit == 0 && RAILS_GPS_STUCK_TOLERATED;
      DEBUG_PRINT("[POST]   SIP32431 ");
      DEBUG_PRINT(r->name);
      if (tolerated) {
        DEBUG_PRINTLN(" ON (AVISO, tolerado: CTRL_GPS queda en ALTO y el GPS no se puede apagar; algo lo une a VDD: "
                      "R154, R167 o un puente de soldadura)");
        continue;
      }
      DEBUG_PRINT(" -> el pin EN no baja: queda en ALTO aunque se pida OFF");
      DEBUG_PRINTLN(r->expanderBit == 0 ? " (algo une CTRL_GPS a VDD: R154 en la tarjeta RF, R167 o un puente de soldadura)"
                                        : " (EN en corto a VDD: revisar el pin y sus vecinos)");
      status = HW_FAIL;
      continue;
    }
    const bool isOutput = (cfg & mask) == 0;
    const bool wanted = (expander.outputs() & mask) != 0;
    const bool level = (in & mask) != 0; // nivel real del pin EN
    const bool ok = isOutput && ((out & mask) != 0) == wanted && level == wanted;
    DEBUG_PRINT("[POST]   SIP32431 ");
    DEBUG_PRINT(r->name);
    DEBUG_PRINT(wanted ? " ON" : " OFF");
    DEBUG_PRINTLN(ok ? " OK" : (isOutput ? " -> el pin EN no tiene el nivel pedido" : " -> EN no configurado como salida"));
    if (!ok) status = HW_FAIL;
  }
  return status;
}

HwStatus board_test_sd(void) {
  sd.check();
  sd_print_status("[POST]   ");
  return sd.cardOk() ? HW_PASS : HW_FAIL;
}

// BME280 de la tarjeta MEMS: CHIP_ID y una medición en rango. Si no respondió en el arranque se
// reintenta (p. ej. tarjeta conectada después).
HwStatus board_test_environment(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[POST]   Sin bus I2C2");
    return HW_SKIP;
  }
  if (!env.present() && !env.begin(Wire1)) {
    DEBUG_PRINTLN("[POST]   BME280 no responde en 0x76 (¿tarjeta MEMS conectada?)");
    return HW_FAIL;
  }
  uint8_t id = 0;
  if (!env.readReg(Bme280::REG_CHIP_ID, id) || id != Bme280::CHIP_ID_VALUE) {
    DEBUG_PRINT("[POST]   CHIP_ID incorrecto: ");
    print_hex8(id);
    DEBUG_PRINTLN("");
    return HW_FAIL;
  }
  // Recién iniciado todavía no hay medición: se espera hasta 25 ms (una medición tarda 9.3 ms máx.)
  Bme280::Sample s;
  const uint32_t t0 = millis();
  while (!env.read(s) && millis() - t0 < 25) delay(2);
  return env_print_sample("[POST]   BME280 ") ? HW_PASS : HW_FAIL;
}

// ---- Sensor CH4 (GM-402B): calefactor por FAN53741 + Q5 (HT_PWM) ----
// Orden de encendido: HT_PWM en ON primero (Q5 deja a GND el lado bajo del calefactor), luego 5V_CH4
// (VIN del FAN53741 y VC del sensor) y por último la salida del FAN53741 (VH). Apagado al revés.
static bool burnin = false;      // precalentamiento del CH4 en curso (ver board_ch4_burnin)
static bool burnin_wait = false; // pedido pero en pausa por SURVIVAL: se reanuda al salir

bool board_ch4_set(bool on) {
  if (!on && burnin) return true; // precalentamiento: el calefactor sigue encendido
  if (on == ch4_on) return true;
  if (!on) {
    if (heater.present()) heater.enable(false);
    board_rail_set("CH4", false);
    digitalWrite(WM_PIN_HT_PWM, LOW);
    ch4_on = false;
    ch4mos.heaterOff();
    // R0 cambió desde la flash: se guarda (cuando el radio esté libre)
    if (ch4mos.r0Valid() && (ch4_saved_r0 <= 0 || fabsf(ch4mos.r0() - ch4_saved_r0) > ch4_saved_r0 * CH4_R0_SAVE_PCT / 100.0f))
      ch4_save_pending = true;
    return true;
  }
  if (!expander.present()) {
    DEBUG_PRINTLN("[CH4] Sin TCA6408A: no se puede encender 5V_CH4");
    return false;
  }
  digitalWrite(WM_PIN_HT_PWM, HIGH); // 1) Q5 cerrado: calefactor a GND
  board_rail_set("CH4", true);       // 2) 5V_CH4
  // Tras subir VIN el FAN53741 tarda en contestar (POR): se reintenta hasta 200 ms
  const uint32_t t0 = millis();
  bool found = false;
  while (!(found = heater.begin(Wire1)) && millis() - t0 < 200) board_wait_ms(5);
  if (!found || !heater.setVoutMv(CH4_HEATER_MV) || !heater.enable(true)) { // 3) VH
    DEBUG_PRINTLN("[CH4] FAN53741 no responde en 0x52 o no acepta la configuracion -> todo apagado");
    if (found) heater.enable(false);
    board_rail_set("CH4", false);
    digitalWrite(WM_PIN_HT_PWM, LOW);
    return false;
  }
  ch4_on = true;
  ch4_on_ms = millis();
  ch4mos.heaterOn(ch4_on_ms);
  return true;
}

// CH4: Rs compensada, línea base R0 y ppm estimada (lib/ch4_mos)
static void print_ch4_status(uint32_t now) {
  DEBUG_PRINT(" Rs ");
  DEBUG_PRINT(board_has_data(ch4mos.rsComp()) ? String(ch4mos.rsComp() / 1000.0f, 2) + "k" : String("-"));
  DEBUG_PRINT(" R0 ");
  DEBUG_PRINT(ch4mos.r0Valid() ? String(ch4mos.r0() / 1000.0f, 2) + "k" : String("-"));
  if (!ch4mos.warm()) {
    DEBUG_PRINT(" calentando ");
    DEBUG_PRINT(ch4mos.heatingMs(now) / 1000);
    DEBUG_PRINT(" s");
    if (board_has_data(ch4mos.changePct())) {
      DEBUG_PRINT(" (cambio ");
      DEBUG_PRINT(ch4mos.changePct(), 1);
      DEBUG_PRINT(" %/10 s)");
    }
    return;
  }
  DEBUG_PRINT(" Rs/R0 ");
  DEBUG_PRINT(board_has_data(ch4mos.ratio()) ? String(ch4mos.ratio(), 3) : String("-"));
  DEBUG_PRINT(" = ");
  DEBUG_PRINT(board_has_data(ch4mos.ppm()) ? String(ch4mos.ppm(), 0) : String("-"));
  DEBUG_PRINT(" ppm (estimado)");
}

void board_methane_start(void) {
  ch4_state = MethaneState(); // cada medición es independiente
  methane_active = true;
}

void board_methane_stop(void) {
  methane_active = false;
}

// POST: enciende el CH4 con la secuencia de board_ch4_set(), observa AIN1 CH4_HEATER_WATCH_MS y lo
// deja como estaba. La variación es informativa: si no cambia, el sensor puede estar ya estable.
// ch4Off = AIN1 antes de encender.
static bool ch4_heater_check(int32_t ch4Off) {
  const bool wasOn = ch4_on;
  if (!wasOn && !board_ch4_set(true)) return false;

  uint16_t mv = 0;
  uint8_t mode = 0, faults = 0;
  const bool read = heater.readVoutMv(mv) && heater.readReg(Fan53741::REG_MODE, mode) &&
                    heater.readReg(Fan53741::REG_FAULT_LIVE, faults);
  const bool ok = read && mv == CH4_HEATER_MV && (mode & 0x02);
  DEBUG_PRINT("[POST]   HT_PWM ON -> 5V_CH4 ON -> FAN53741 0x52 (ID_REV ");
  print_hex8(heater.idRev());
  DEBUG_PRINT(") salida ");
  DEBUG_PRINT(mv);
  DEBUG_PRINT(" mV, MODE ");
  print_hex8(mode);
  DEBUG_PRINT(", fallas ");
  print_hex8(faults);
  DEBUG_PRINTLN(ok ? "" : " -> ERROR al configurar");

  // AIN1 cada 100 ms con el calefactor encendido
  int32_t first = 0, last = 0, lo = Ads1219::CODE_MAX, hi = Ads1219::CODE_MIN;
  bool anySample = false;
  const uint32_t t0 = millis();
  while (millis() - t0 < CH4_HEATER_WATCH_MS) {
    int32_t code = 0;
    if (adc.convert(Ads1219::MUX_AIN1, code)) {
      if (!anySample) first = code;
      anySample = true;
      last = code;
      if (code < lo) lo = code;
      if (code > hi) hi = code;
    }
    board_wait_ms(100);
  }
  if (!wasOn) board_ch4_set(false);

  if (!anySample) {
    DEBUG_PRINTLN("[POST]   AIN1 sin conversiones con el calefactor encendido");
    return false;
  }
  const float deltaMv = (adc.toVolts(last) - adc.toVolts(first)) * 1000.0f;
  DEBUG_PRINT("[POST]   AIN1 CH4: apagado ");
  DEBUG_PRINT(adc.toVolts(ch4Off), 4);
  DEBUG_PRINT(" V | calefactor ");
  DEBUG_PRINT(CH4_HEATER_WATCH_MS / 1000);
  DEBUG_PRINT(" s: ");
  DEBUG_PRINT(adc.toVolts(first), 4);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINT(adc.toVolts(last), 4);
  DEBUG_PRINT(" V (min ");
  DEBUG_PRINT(adc.toVolts(lo), 4);
  DEBUG_PRINT(", max ");
  DEBUG_PRINT(adc.toVolts(hi), 4);
  DEBUG_PRINT(Ads1219::saturated(hi) ? " SATURADO" : "");
  DEBUG_PRINT(") variacion ");
  DEBUG_PRINT(deltaMv, 1);
  DEBUG_PRINTLN(" mV");

  // Rs al final de la observación (recién encendido: la ppm sale cuando el sensor está caliente, ver lib/ch4_mos)
  const float vLast = adc.toVolts(last);
  const float vc = ch4mos.config().vcVolts;
  DEBUG_PRINT("[POST]   Rs ");
  if (Ads1219::saturated(last) || vLast <= 0.001f || vLast >= vc)
    DEBUG_PRINT("- (saturado: recien encendido)");
  else
    DEBUG_PRINT(String(ch4mos.config().rlOhms * (vc - vLast) / vLast / 1000.0f, 2) + " kohm");
  DEBUG_PRINTLN(deltaMv > -5.0f && deltaMv < 5.0f ? " | sin variacion: no es falla, el sensor puede estar ya estable"
                                                  : "");
  return ok;
}

// Tarjeta MEMS (CH4/H2), por la tarjeta RF (FPC2). El BME280 tiene su propia prueba.
//   - MCP4018: el cursor está en el paso de CH4_LOAD_OHMS (si no, se reescribe y se vuelve a leer)
//   - ADS1219: responde y convierte AIN0 (H2), AIN1 (CH4) y las entradas en corto (offset)
// La medición de gas necesita 5V_H2 / 5V_CH4 y el calefactor: sin ellos los voltajes no significan nada.
HwStatus board_test_mems(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[POST]   Sin bus I2C2");
    return HW_SKIP;
  }
  HwStatus status = HW_PASS;

  // MCP4018: carga del sensor CH4
  const uint8_t wanted = ch4Load.stepFor(CH4_LOAD_OHMS);
  uint8_t step = 0xFF;
  DEBUG_PRINT("[POST]   MCP4018 0x2F: ");
  if (!ch4Load.present() && !ch4Load.begin(Wire1, MCP4018_RAB_OHMS)) {
    DEBUG_PRINTLN("no responde");
    status = HW_FAIL;
  } else if (!ch4Load.read(step)) {
    DEBUG_PRINTLN("no se pudo leer el cursor");
    status = HW_FAIL;
  } else {
    if (step != wanted) { // p. ej. la tarjeta MEMS se reconectó (vuelve al paso 63)
      ch4Load.write(wanted);
      ch4Load.read(step);
    }
    DEBUG_PRINT("paso ");
    DEBUG_PRINT(step);
    DEBUG_PRINT("/127 = ");
    DEBUG_PRINT(ch4Load.ohmsAt(step));
    DEBUG_PRINT(" ohm (pedido ");
    DEBUG_PRINT(CH4_LOAD_OHMS);
    DEBUG_PRINTLN(step == wanted ? ") OK" : ") -> no quedo en el paso pedido");
    if (step != wanted) status = HW_FAIL;
  }

  // ADS1219: conversiones de prueba (la máquina de muestreo se reinicia después)
  adc_state = ADC_IDLE;
  if (!adc.present() && !adc.begin(Wire1, ADS1219_ADDR)) {
    DEBUG_PRINTLN("[POST]   ADS1219 0x41: no responde o no acepta la configuracion");
    return HW_FAIL;
  }
  int32_t h2 = 0, ch4 = 0, shorted = 0;
  const bool ok = adc.convert(Ads1219::MUX_AIN0, h2) && adc.convert(Ads1219::MUX_AIN1, ch4) &&
                  adc.convert(Ads1219::MUX_SHORTED, shorted);
  if (!ok) {
    DEBUG_PRINTLN("[POST]   ADS1219 0x41: la conversion no termina (DRDY)");
    return HW_FAIL;
  }
  DEBUG_PRINT("[POST]   ADS1219 0x41:");
  print_adc_channel(" AIN0 H2", h2);
  print_adc_channel(" | AIN1 CH4", ch4);
  DEBUG_PRINT(" | offset ");
  DEBUG_PRINT(shorted);
  DEBUG_PRINTLN(" codigos");

  // Calefactor del CH4: se enciende para ver si AIN1 varía (sin control ni realimentación)
  if (!ch4_heater_check(ch4)) status = HW_FAIL;
  return status;
}

// GPS L96-M33: llegan tramas NMEA con checksum correcto (hasta GPS_POST_WAIT_MS). El fix no es requisito
// (en interiores puede no haberlo); se informa.
HwStatus board_test_gps(void) {
  if (!gps_powered) {
    gps_power(true, "POST (queda buscando fix)");
    board_wait_ms(1000); // arranque del L96
  }
  const uint32_t before = gps.sentences();
  const uint32_t t0 = millis();
  while (millis() - t0 < GPS_POST_WAIT_MS && gps.sentences() - before < 3) board_wait_ms(50);
  const uint32_t got = gps.sentences() - before;

  if (gps_rx_pin == BOARD_PIN_NC) {
    DEBUG_PRINT("[POST]   L96-M33: sin trafico en P0.14 ni P1.09 (flancos ");
    DEBUG_PRINT(gps_edges_a);
    DEBUG_PRINT(" / ");
    DEBUG_PRINT(gps_edges_b);
    DEBUG_PRINTLN("): ¿GPS alimentado? ¿tarjeta RF conectada?");
    return HW_FAIL;
  }
  DEBUG_PRINT("[POST]   L96-M33: TX del GPS en ");
  DEBUG_PRINT(gps_pin_name(gps_rx_pin));
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(got);
  DEBUG_PRINT(" tramas NMEA validas en ");
  DEBUG_PRINT((millis() - t0) / 1000.0f, 1);
  DEBUG_PRINT(" s (ultima ");
  DEBUG_PRINT(gps.lastType());
  DEBUG_PRINT(", errores de checksum ");
  DEBUG_PRINT(gps.checksumErrors());
  DEBUG_PRINTLN(")");
  if (got == 0) {
    DEBUG_PRINTLN("[POST]   Llegan bits pero no tramas validas: ¿velocidad distinta de 9600?");
    return HW_FAIL;
  }
  const NmeaGps::Fix &f = gps.fix();
  DEBUG_PRINT("[POST]   ");
  if (f.quality > 0) {
    DEBUG_PRINT("Fix ");
    DEBUG_PRINT(f.quality);
    DEBUG_PRINT(" | ");
    DEBUG_PRINT(f.latitude, 6);
    DEBUG_PRINT(", ");
    DEBUG_PRINT(f.longitude, 6);
    DEBUG_PRINT(" | ");
    DEBUG_PRINT(f.altitudeM, 1);
    DEBUG_PRINT(" m | HDOP ");
    DEBUG_PRINT(f.hdop, 1);
  } else {
    DEBUG_PRINT("Sin fix todavia (normal en interiores o recien encendido)");
  }
  DEBUG_PRINT(" | ");
  DEBUG_PRINT(f.satellites);
  DEBUG_PRINT(" satelites");
  if (f.timeValid) {
    char buf[24];
    snprintf(buf, sizeof(buf), " | %02u:%02u:%02u UTC", f.hour, f.minute, f.second);
    DEBUG_PRINT(buf);
  }
  DEBUG_PRINTLN("");
  return HW_PASS;
}

// Blower: con el motor apagado no deben llegar pulsos; encendido (1.5 s de arranque) se cuentan
// durante 1 s en FAN_FEEDBACK (P0.13).
HwStatus board_test_blower(void) {
  const bool wasOn = fan_on;
  board_fan_set(false);
  if (wasOn) board_wait_ms(2000); // que se detenga antes de medir el reposo

  uint32_t p0 = fan_pulses;
  board_wait_ms(300);
  const uint32_t idle = fan_pulses - p0;
  const bool levelOff = digitalRead(WM_PIN_FAN_FB);

  board_fan_set(true);
  board_wait_ms(1500); // arranque del motor
  p0 = fan_pulses;
  const uint32_t t0 = millis();
  board_wait_ms(1000);
  const uint32_t windowMs = millis() - t0;
  const uint32_t pulses = fan_pulses - p0;
  board_fan_set(wasOn);

  DEBUG_PRINT("[POST]   FG P0.13: apagado ");
  DEBUG_PRINT(idle);
  DEBUG_PRINT(" pulsos en 0.3 s (nivel ");
  DEBUG_PRINT(levelOff ? "ALTO" : "BAJO");
  DEBUG_PRINT(") | encendido ");
  DEBUG_PRINT(pulses);
  DEBUG_PRINTLN(" pulsos en 1 s");

  if (pulses < 10 || idle * 10 >= pulses) {
    DEBUG_PRINTLN(levelOff ? "[POST]   No llegan pulsos: ¿el blower gira? ¿FG conectado a P0.13?"
                           : "[POST]   FG en BAJO en reposo: la salida es colector abierto y algo la lleva a GND "
                             "(¿R300 10k a GND?): necesita pull-up a 3.3 V");
    return HW_FAIL;
  }
  const uint32_t hz = pulses * 1000UL / windowMs;
  const uint32_t rpm = pulses * 60000UL / windowMs / FAN_PULSES_PER_REV;
  DEBUG_PRINT("[POST]   ");
  DEBUG_PRINT(hz);
  DEBUG_PRINT(" Hz -> ");
  DEBUG_PRINT(rpm);
  DEBUG_PRINT(" rpm (nominal ");
  DEBUG_PRINT(FAN_NOMINAL_RPM);
  DEBUG_PRINT(", ");
  DEBUG_PRINT(FAN_PULSES_PER_REV);
  DEBUG_PRINTLN(" pulsos/vuelta)");
  if (rpm < FAN_NOMINAL_RPM * 7 / 10 || rpm > FAN_NOMINAL_RPM * 13 / 10) {
    DEBUG_PRINTLN("[POST]   Velocidad lejos de la nominal (+-30 %): revisar FAN_PULSES_PER_REV o el motor");
  }
  return HW_PASS;
}

// Varias lecturas del MAX17048 seguidas (VCELL se actualiza cada 250 ms): sin batería el cargador hace subir y
// bajar el voltaje y el SOC/C-rate salen absurdos; con batería es estable. prefix != nullptr: imprime cada una
struct BatStats {
  uint16_t n = 0, vMin = 0xFFFF, vMax = 0, vLast = 0;
  float socMin = 1e9f, socMax = -1e9f, crateMin = 1e9f, crateMax = -1e9f, socLast = 0, crateLast = 0;
  uint16_t statusOr = 0;
};
static bool battery_sample(uint32_t totalMs, uint32_t everyMs, BatStats &s, const char *prefix) {
  const uint32_t t0 = millis();
  for (uint32_t next = 0;; next += everyMs) {
    while (millis() - t0 < next) { board_wdt_feed(); delay(5); }
    uint16_t mv = 0, status = 0;
    float soc = 0, crate = 0;
    if (!fuelGauge.readVoltageMv(mv) || !fuelGauge.readSoc(soc) || !fuelGauge.readCrate(crate) || !fuelGauge.readStatus(status)) {
      if (prefix) { DEBUG_PRINT(prefix); DEBUG_PRINTLN("MAX17048 no responde en 0x36"); }
      return false;
    }
    s.n++;
    s.vLast = mv; s.socLast = soc; s.crateLast = crate; s.statusOr |= status;
    if (mv < s.vMin) s.vMin = mv;
    if (mv > s.vMax) s.vMax = mv;
    if (soc < s.socMin) s.socMin = soc;
    if (soc > s.socMax) s.socMax = soc;
    if (crate < s.crateMin) s.crateMin = crate;
    if (crate > s.crateMax) s.crateMax = crate;
    if (prefix) {
      DEBUG_PRINT(prefix);
      DEBUG_PRINT("t=");
      DEBUG_PRINT((millis() - t0) / 1000.0f, 2);
      DEBUG_PRINT(" s | ");
      DEBUG_PRINT(mv / 1000.0f, 3);
      DEBUG_PRINT(" V | SOC ");
      DEBUG_PRINT(soc, 2);
      DEBUG_PRINT(" % | ");
      DEBUG_PRINT(crate, 1);
      DEBUG_PRINT(" %/h | STATUS 0x");
      DEBUG_PRINTLN(status, HEX);
    }
    if (next + everyMs > totalMs) return true;
  }
}
static void battery_print_stats(const char *prefix, const BatStats &s) {
  DEBUG_PRINT(prefix);
  DEBUG_PRINT("Resumen (");
  DEBUG_PRINT(s.n);
  DEBUG_PRINT(" lecturas): ");
  DEBUG_PRINT(s.vMin / 1000.0f, 3);
  DEBUG_PRINT("-");
  DEBUG_PRINT(s.vMax / 1000.0f, 3);
  DEBUG_PRINT(" V (varia ");
  DEBUG_PRINT(s.vMax - s.vMin);
  DEBUG_PRINT(" mV) | SOC ");
  DEBUG_PRINT(s.socMin, 2);
  DEBUG_PRINT("-");
  DEBUG_PRINT(s.socMax, 2);
  DEBUG_PRINT(" % | C-rate ");
  DEBUG_PRINT(s.crateMin, 1);
  DEBUG_PRINT(" a ");
  DEBUG_PRINT(s.crateMax, 1);
  DEBUG_PRINT(" %/h | USB ");
  DEBUG_PRINTLN(board_usb_powered() ? "si" : "no");
}

// BAT:LOG[=segundos]: lecturas del MAX17048 cada 250 ms (para comparar la placa con y sin batería)
void board_battery_log(uint32_t seconds) {
  if (seconds < 1) seconds = 10;
  if (seconds > 120) seconds = 120;
  if (!fuelGauge.present() && !fuel_gauge_find()) {
    DEBUG_PRINTLN("[BAT] MAX17048 no responde en 0x36 (ni en I2C1 ni en I2C2)");
    return;
  }
  DEBUG_PRINT("[BAT] MAX17048 en ");
  DEBUG_PRINT(fuel_gauge_bus);
  DEBUG_PRINT(": ");
  DEBUG_PRINT(seconds);
  DEBUG_PRINTLN(" s, una lectura cada 250 ms");
  // Registros de configuración. HIBRT (0x0A): si |C-rate| queda bajo el umbral, el MAX17048 hiberna y solo mide
  // cada 45 s (lecturas congeladas). Durante el registro se desactiva (0x0000) y luego se deja como estaba
  uint16_t hibrt = 0, mode = 0, config = 0, valrt = 0, vreset = 0, status = 0;
  const bool regs = fuelGauge.readReg(Max17048::REG_HIBRT, hibrt) && fuelGauge.readReg(Max17048::REG_MODE, mode) &&
                    fuelGauge.readReg(Max17048::REG_CONFIG, config) && fuelGauge.readReg(Max17048::REG_VALRT, valrt) &&
                    fuelGauge.readReg(Max17048::REG_VRESET_ID, vreset) && fuelGauge.readReg(Max17048::REG_STATUS, status);
  if (regs) {
    DEBUG_PRINT("[BAT] Registros: HIBRT 0x"); DEBUG_PRINT(hibrt, HEX);
    DEBUG_PRINT(" | MODE 0x"); DEBUG_PRINT(mode, HEX);
    DEBUG_PRINT(" (hibernando: "); DEBUG_PRINT((mode & 0x1000) ? "SI" : "no");
    DEBUG_PRINT(") | CONFIG 0x"); DEBUG_PRINT(config, HEX);
    DEBUG_PRINT(" | VALRT 0x"); DEBUG_PRINT(valrt, HEX);
    DEBUG_PRINT(" | VRESET/ID 0x"); DEBUG_PRINT(vreset, HEX);
    DEBUG_PRINT(" | STATUS 0x"); DEBUG_PRINTLN(status, HEX);
  }
  const bool wake = regs && hibrt != 0;
  if (wake) DEBUG_PRINTLN(fuelGauge.writeReg(Max17048::REG_HIBRT, 0) ? "[BAT] Hibernacion desactivada durante el registro"
                                                                       : "[BAT] No se pudo desactivar la hibernacion");
  BatStats s;
  if (battery_sample(seconds * 1000UL, 250, s, "[BAT] ")) battery_print_stats("[BAT] ", s);
  if (wake) DEBUG_PRINTLN(fuelGauge.writeReg(Max17048::REG_HIBRT, hibrt) ? "[BAT] HIBRT restaurado" : "[BAT] No se pudo restaurar HIBRT");
}

HwStatus board_test_battery(void) {
  if (!i2c1_ready && !i2c2_ready) {
    DEBUG_PRINTLN("[POST]   Sin buses I2C");
    return HW_SKIP;
  }
  // Si no respondió en el arranque se reintenta (p. ej. batería conectada después)
  if (!fuelGauge.present() && !fuel_gauge_find()) {
    DEBUG_PRINTLN("[POST]   MAX17048 no responde en 0x36 (ni en I2C1 ni en I2C2)");
    return HW_FAIL;
  }
  DEBUG_PRINT("[POST]   MAX17048 en ");
  DEBUG_PRINTLN(fuel_gauge_bus);
  bool ok = battery_print_status("[POST]   ");
  if (!ok && !battery_reads_charger) return HW_FAIL; // no responde o voltaje fuera de 2.5-4.5 V

  // ¿Hay batería? Sin ella el MAX17048 mide la salida del cargador, que sube y baja (diente de sierra: medido
  // 2026-11-25 sin batería, 4.08 -> 4.23 V en ~11 s y vuelve a caer). Con batería el voltaje es estable. El
  // SOC y el C-rate cambian muy lento para decidirlo en unos segundos. Se desactiva la hibernación (si hiberna
  // mide cada 45 s y las lecturas quedan congeladas) y se deja como estaba
  uint16_t hibrt = 0;
  const bool wake = fuelGauge.readReg(Max17048::REG_HIBRT, hibrt) && hibrt != 0 && fuelGauge.writeReg(Max17048::REG_HIBRT, 0);
  BatStats s;
  const bool sampled = battery_sample(BAT_CHECK_MS, 250, s, nullptr);
  if (wake) fuelGauge.writeReg(Max17048::REG_HIBRT, hibrt);
  if (!sampled) return HW_FAIL;
  battery_print_stats("[POST]   ", s);
  const uint16_t swing = s.vMax - s.vMin;
  if (battery_reads_charger || swing >= BAT_ABSENT_SWING_MV) {
    DEBUG_PRINT("[POST]   SIN BATERIA: ");
    if (swing >= BAT_ABSENT_SWING_MV) {
      DEBUG_PRINT("el voltaje varia ");
      DEBUG_PRINT(swing);
      DEBUG_PRINT(" mV en ");
      DEBUG_PRINT(BAT_CHECK_MS / 1000);
      DEBUG_PRINTLN(" s (es el cargador, no una celda): conectar la bateria o revisar su conector");
    } else {
      DEBUG_PRINTLN("el MAX17048 lee la salida del cargador (SOC o C-rate imposibles): conectar la bateria");
    }
    return HW_FAIL;
  }
  if (s.vMin < BAT_BAD_MV) {
    DEBUG_PRINT("[POST]   BATERIA MALA O MUY DESCARGADA: ");
    DEBUG_PRINT(s.vMin / 1000.0f, 3);
    DEBUG_PRINT(" V (menos de ");
    DEBUG_PRINT(BAT_BAD_MV / 1000.0f, 1);
    DEBUG_PRINTLN(board_usb_powered() ? " V aun cargando por USB): cambiar la bateria" : " V): cargarla o cambiarla");
    return HW_FAIL;
  }
  DEBUG_PRINT("[POST]   Bateria conectada: voltaje estable (varia ");
  DEBUG_PRINT(swing);
  DEBUG_PRINTLN(" mV)");
  if (ok) {
    energy_service(true);
    DEBUG_PRINT("[POST]   Energia: ");
    DEBUG_PRINT(energy.modeName());
    DEBUG_PRINT(" | C-rate filtrado ");
    DEBUG_PRINT(energy.crateFiltered(), 2);
    DEBUG_PRINT(" %/h");
    DEBUG_PRINT(energy.charging() ? " (cargando)" : "");
    DEBUG_PRINT(" | ");
    DEBUG_PRINT(energy.isDay() ? "dia" : "noche");
    DEBUG_PRINTLN(energy.allowGas() ? " | gases permitidos" : " | sin gases (no es RICH)");
  }
  return ok ? HW_PASS : HW_FAIL;
}

HwStatus board_test_imu(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[POST]   Sin bus I2C2");
    return HW_SKIP;
  }
  if (!imu.present() && !imu.begin(Wire1)) {
    DEBUG_PRINTLN("[POST]   LSM6DS3TR-C no responde en 0x6A (¿POWER_IMU encendido?)");
    return HW_FAIL;
  }
  uint8_t id = 0;
  if (!imu.readReg(Lsm6ds3::REG_WHO_AM_I, id) || id != Lsm6ds3::WHO_AM_I_VALUE) {
    DEBUG_PRINT("[POST]   WHO_AM_I incorrecto: ");
    print_hex8(id);
    DEBUG_PRINTLN("");
    return HW_FAIL;
  }
  return imu_print_sample("[POST]   ") ? HW_PASS : HW_FAIL;
}

// Responde, la hora es válida y, si ya pasó más de 1.5 s desde el arranque, el reloj avanzó lo
// mismo que millis() (±2 s): así se comprueba el oscilador sin bloquear el POST.
HwStatus board_test_rtc(void) {
  if (!i2c2_ready) {
    DEBUG_PRINTLN("[POST]   Sin bus I2C2");
    return HW_SKIP;
  }
  Rx8130::DateTime t;
  if (!rtc.present()) {
    DEBUG_PRINTLN("[POST]   RX8130CE no respondio en el arranque (0x32)");
    return HW_FAIL;
  }
  if (!rtc.readTime(t)) {
    DEBUG_PRINTLN("[POST]   RX8130CE: hora invalida");
    return HW_FAIL;
  }
  DEBUG_PRINT("[POST]   RX8130CE ");
  rtc_print_time(t);
  const uint32_t elapsedMs = millis() - rtc_boot_ms;
  if (!rtc_boot_valid || elapsedMs < 1500) {
    DEBUG_PRINTLN(" (avance sin verificar: muy poco tiempo desde el arranque)");
    return HW_PASS;
  }
  const int32_t now = t.hour * 3600L + t.minute * 60L + t.second;
  const int32_t boot = rtc_boot_time.hour * 3600L + rtc_boot_time.minute * 60L + rtc_boot_time.second;
  const int32_t rtcElapsed = (now - boot + 86400L) % 86400L;
  const int32_t expected = (int32_t)(elapsedMs / 1000);
  DEBUG_PRINT(" | avanzo ");
  DEBUG_PRINT(rtcElapsed);
  DEBUG_PRINT(" s en ");
  DEBUG_PRINT(expected);
  DEBUG_PRINTLN(" s");
  if (rtcElapsed < expected - 2 || rtcElapsed > expected + 2) {
    DEBUG_PRINTLN("[POST]   El reloj no avanza al ritmo correcto (¿oscilador?)");
    return HW_FAIL;
  }
  return HW_PASS;
}

HwStatus board_test_lora(void) {
#if BOARD_ENABLE_LORA
  // Lectura de la versión del LR1110 por SPI, con el radio reservado (falla si LBM está en TX/RX)
  if (smtc_modem_suspend_before_user_radio_access() != SMTC_MODEM_RC_OK) {
    DEBUG_PRINTLN("[POST]   LR1110 ocupado (TX/RX en curso): repetir POST:LORA");
    return HW_SKIP;
  }
  lr11xx_system_version_t version = {};
  const bool read = lr11xx_system_get_version(LbmWm1110::getInstance().getRadio()->ral.context, &version) ==
                    LR11XX_STATUS_OK;
  smtc_modem_resume_after_user_radio_access();

  DEBUG_PRINT("[POST]   LR1110 hw ");
  print_hex8(version.hw);
  DEBUG_PRINT(" | tipo ");
  print_hex8(version.type);
  DEBUG_PRINT(" | fw 0x");
  DEBUG_PRINT(version.fw, HEX);
  DEBUG_PRINT(" | LoRaWAN ");
  DEBUG_PRINTLN(lora.isJoined() ? "unido" : "sin join todavia (se une en el loop)");
  if (!read || version.type != LR11XX_SYSTEM_VERSION_TYPE_LR1110) {
    DEBUG_PRINTLN("[POST]   El LR1110 no responde por SPI");
    return HW_FAIL;
  }
  return HW_PASS;
#else
  DEBUG_PRINTLN("[POST]   LoRa deshabilitado (BOARD_ENABLE_LORA = 0)");
  return HW_SKIP;
#endif
}

HwStatus board_test_bluetooth(void) {
#if BOARD_ENABLE_BLE
  if (!ble_started()) {
    DEBUG_PRINTLN("[POST]   El SoftDevice no arranco (ver [INIT] Bluetooth)");
    return HW_FAIL;
  }
  DEBUG_PRINT("[POST]   \"");
  DEBUG_PRINT(ble_name);
  DEBUG_PRINT("\" ");
  DEBUG_PRINTLN(ble_subscribed() ? "conectado, app suscrita" : ble_connected() ? "conectado"
                                                                               : "anunciando");
  return HW_PASS;
#else
  DEBUG_PRINTLN("[POST]   Bluetooth deshabilitado (BOARD_ENABLE_BLE = 0)");
  return HW_SKIP;
#endif
}

bool board_ble_subscribed(void) {
#if BOARD_ENABLE_BLE
  return ble_subscribed();
#else
  return false;
#endif
}

bool board_ble_send(const char *line) {
#if BOARD_ENABLE_BLE
  return ble_send_line(line);
#else
  (void)line;
  return false;
#endif
}

// Energía para la app (formato V12): Hour = hora local del RTC (-1 sin hora)
void board_ble_send_energy(void) {
#if BOARD_ENABLE_BLE
  if (!ble_subscribed()) return;
  const EnergySupervisor &e = board_energy();
  BoardTime t;
  const int hour = board_time_local(t) ? t.hour : -1;
  char v[16], soc[16], crate[16], ema[16], line[200];
  snprintf(line, sizeof(line),
           "{\"Vbat\":%s,\"SOC\":%s,\"CRate\":%s,\"CRateEMA\":%s,\"Hour\":%d,\"Sun\":\"%s\",\"Trend\":\"%s\",\"Mode\":\"%s\"}",
           json_num(v, sizeof(v), snap.batV, 3), json_num(soc, sizeof(soc), snap.batSoc, 1),
           json_num(crate, sizeof(crate), snap.batCrate, 2), json_num(ema, sizeof(ema), e.crateFiltered(), 2), hour,
           e.isDay() ? "DAY" : "NIGHT", e.charging() ? "CHARGING" : "DISCHARGING", e.modeName());
  ble_send_line(line);
#endif
}

// ============================================
// 7. Inicialización principal y ciclo de trabajo
// ============================================
void board_init(void) {
  if (board_initialized) {
    DEBUG_PRINTLN("[INIT] La placa ya fue inicializada; se omite reinicialización completa");
    return;
  }
  board_initialized = true;
  snapshot_reset();

  DEBUG_PRINTLN("=====================================");
  DEBUG_PRINTLN("  Wearme NRF52  initialization       ");
  DEBUG_PRINTLN("=====================================");
  DEBUG_PRINTLN("[INIT] Watchdog (WDT de hardware)");
  wdt_init(); // primero: si algo de la inicialización se cuelga, el WDT reinicia

  for (uint8_t step = 0; step < BOARD_INIT_COUNT; step++) {
    board_init_step(static_cast<BoardInitStepId>(step));
    board_wdt_feed();
  }
  DEBUG_PRINTLN("[INIT] Supervisor de energia (RICH / CORE / SURVIVAL)");
  energy_service(true); // primer modo y política de rieles (5V_H2 solo en RICH) antes del POST
  rail_log_prefix = "[PWR] ";
}

static bool leds_ready = false; // terminó el patrón de arranque (los tres encendidos)
static LedMode led_mode = LED_MODE_OFF;
static uint32_t led_fault_ms = 0;
static uint8_t led_fault_cursor = 0; // siguiente código de falla de la tabla

void board_post_result(bool passed) {
  leds.allOff();
  if (passed)
    leds.play(failcodes_event(LED_EV_POST_OK));
  else
    leds.set(StatusLeds::RED, true);
  leds_ready = true;
  DEBUG_PRINTLN(passed ? "[LED] POST OK: verde 3 destellos" : "[LED] POST con fallas: rojo fijo");
#if BOARD_ENABLE_BLE
  ble_send_post_status();
#endif
}

void board_led_state_change(void) {
  leds.play(failcodes_event(LED_EV_STATE_CHANGE));
}

void board_led_mode(LedMode mode) {
  led_mode = mode;
  if (mode == LED_MODE_FAULT) {
    leds.periodic(nullptr, 0);
    if (FAULT_RED_STEADY) leds.set(StatusLeds::RED, true);
    led_fault_ms = millis();
    led_fault_cursor = 0;
    return;
  }
  const LedModeCfg &cfg = failcodes_mode(mode);
  leds.periodic(&cfg.code, cfg.periodMs);
}

// FAULT: cada FAULT_CODE_PERIOD_MS el código (verde/azul/cian x N) del siguiente subsistema con falla
static void fault_leds(void) {
  if (led_mode != LED_MODE_FAULT || millis() - led_fault_ms < FAULT_CODE_PERIOD_MS) return;
  led_fault_ms = millis();
  const FailCode *f = failcodes_next(fault_mask, led_fault_cursor);
  if (f != nullptr) leds.play(f->code);
}

bool board_rtc_set_local(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
  if (!rtc.present()) {
    DEBUG_PRINTLN("[RTC] RX8130CE no disponible");
    return false;
  }
  if (year < 2000 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 || minute > 59 ||
      second > 59) {
    DEBUG_PRINTLN("[RTC] Fecha u hora fuera de rango");
    return false;
  }
  Rx8130::DateTime t = {year, month, day, hour, minute, second, 0};
  Rx8130::fromUnix(Rx8130::toUnix(t) - RTC_UTC_OFFSET_H * 3600L, t); // local -> UTC
  if (!rtc.setTime(t)) {
    DEBUG_PRINTLN("[RTC] Error al escribir la hora");
    return false;
  }
  rtc_boot_time = t;
  rtc_boot_ms = millis();
  rtc_boot_valid = true;
  time_source_set(TIME_MANUAL);
  DEBUG_PRINT("[RTC] Hora ajustada a mano: ");
  rtc_print_time(t);
  DEBUG_PRINTLN("");
  return true;
}

#if BOARD_ENABLE_LORA
// Puesta en hora del RTC con la hora de la red (DeviceTimeReq, LoRaWAN 1.0.3+): tras el join se activa
// el servicio de sincronización de LBM; la respuesta llega en un downlink. Hora GPS -> UTC:
// Unix = GPS + 315964800 (1980-01-06) - 18 s de segundos intercalares.
static constexpr uint32_t GPS_TO_UNIX_S = 315964800UL - 18UL;

static void rtc_network_sync(void) {
  static bool requested = false;
  static uint32_t last_check_ms = 0;
  static uint32_t last_sync_ms = 0;
  static bool synced = false;
  if (!rtc.present() || !lora.isJoined()) return;
  if (!requested) {
    requested = true;
    const bool ok = smtc_modem_time_start_sync_service(0, SMTC_MODEM_TIME_MAC_SYNC) == SMTC_MODEM_RC_OK;
    DEBUG_PRINTLN(ok ? "[RTC] Hora solicitada a la red (DeviceTimeReq)" : "[RTC] Error al solicitar la hora a la red");
  }
  const uint32_t now = millis();
  if (now - last_check_ms < 1000 || (synced && now - last_sync_ms < RTC_RESYNC_MS)) return;
  last_check_ms = now;
  uint32_t gps = 0, frac = 0;
  if (smtc_modem_get_time(&gps, &frac) != SMTC_MODEM_RC_OK || gps == 0) return; // todavía sin respuesta

  Rx8130::DateTime before, t;
  const bool hadTime = rtc.readTime(before);
  Rx8130::fromUnix(gps + GPS_TO_UNIX_S, t);
  if (!rtc.setTime(t)) {
    DEBUG_PRINTLN("[RTC] Error al escribir la hora de la red");
    return;
  }
  synced = true;
  last_sync_ms = now;
  rtc_boot_time = t; // nueva referencia para la verificación del avance
  rtc_boot_ms = now;
  rtc_boot_valid = true;
  time_source_set(TIME_NET);
  DEBUG_PRINT("[RTC] Hora de la red: ");
  rtc_print_time(t);
  if (hadTime) {
    DEBUG_PRINT(" | correccion ");
    DEBUG_PRINT((int32_t)(Rx8130::toUnix(t) - Rx8130::toUnix(before)));
    DEBUG_PRINT(" s");
  }
  DEBUG_PRINTLN("");
}
#endif

// LED azul con los eventos de la radio que no pasan por board_lora_send() ni por on_downlink()
static void lora_leds(void) {
#if BOARD_ENABLE_LORA
  if (!leds_ready) return; // durante el arranque los tres LEDs están encendidos
  static bool was_joined = false;
  static uint32_t last_acks = 0;
  static uint32_t search_ms = 0;
  const bool joined = lora.isJoined();
  if (joined && !was_joined) leds.play(failcodes_event(LED_EV_LORA_JOINED));
  was_joined = joined;
  if (!joined && millis() - search_ms >= LED_LORA_SEARCH_PERIOD_MS) { // buscando red
    search_ms = millis();
    leds.play(failcodes_event(LED_EV_LORA_SEARCH));
  }
  if (lora.acks() != last_acks) {
    last_acks = lora.acks();
    leds.play(failcodes_event(LED_EV_LORA_ACK));
  }
#endif
}

// ---- Precalentamiento (burn-in) del CH4 ----
static uint32_t burnin_start_ms = 0;
static uint32_t burnin_log_ms = 0;
static bool burnin_logged = false;

static void fmt_fixed(char *out, size_t size, float v, uint8_t decimals) { // sin printf de float en el core
  if (!board_has_data(v)) {
    out[0] = '\0';
    return;
  }
  const bool neg = v < 0;
  const double a = neg ? -(double)v : (double)v;
  unsigned long scale = 1;
  for (uint8_t i = 0; i < decimals; i++) scale *= 10UL;
  const unsigned long scaled = (unsigned long)(a * scale + 0.5);
  int n = snprintf(out, size, "%s%lu", neg && scaled ? "-" : "", scaled / scale);
  unsigned long frac = scaled % scale;
  if (decimals == 0 || n <= 0 || n + decimals + 1 >= (int)size) return;
  out[n++] = '.';
  for (int i = decimals - 1; i >= 0; i--) {
    out[n + i] = (char)('0' + frac % 10);
    frac /= 10;
  }
  out[n + decimals] = '\0';
}

bool board_ch4_burnin(bool on) {
  if (on) {
    if (energy.valid() && energy.mode() == EnergySupervisor::SURVIVAL) { // se arranca al salir de SURVIVAL
      burnin_wait = true;
      gpreg_write(1, gpreg_read(1) | GPREG_BURNIN);
      DEBUG_PRINTLN("[BURNIN] Bateria critica (SURVIVAL): precalentamiento pedido, arranca al salir de SURVIVAL");
      return true;
    }
    if (!board_ch4_set(true)) {
      DEBUG_PRINTLN("[BURNIN] No se pudo encender el calefactor del CH4");
      return false;
    }
    if (!burnin) {
      burnin_start_ms = millis();
      burnin_logged = false;
    }
    burnin = true;
    gpreg_write(1, gpreg_read(1) | GPREG_BURNIN);
    DEBUG_PRINT("[BURNIN] Precalentamiento del CH4 en curso (calefactor ");
    DEBUG_PRINT(CH4_HEATER_MV);
    DEBUG_PRINTLN(" mV): registro cada minuto en /datos/precalentamiento.csv | CH4:BURNIN:OFF para terminar");
    DEBUG_PRINTLN("[BURNIN] Consume ~30 mA de bateria: dejar el USB o el cargador conectado");
    return true;
  }
  const bool was = burnin;
  burnin = false;
  burnin_wait = false;
  gpreg_write(1, gpreg_read(1) & ~GPREG_BURNIN);
  if (was) {
    DEBUG_PRINT("[BURNIN] Precalentamiento terminado tras ");
    DEBUG_PRINT((millis() - burnin_start_ms) / 60000UL);
    DEBUG_PRINTLN(" min en esta sesion");
  }
  if (power_save || survive_mode) board_ch4_set(false); // en MEASURE el calefactor lo maneja la FSM
  return true;
}

bool board_ch4_burnin_active(void) {
  return burnin || burnin_wait;
}

// Batería crítica: el calefactor se apaga pero el pedido sigue (bit en GPREGRET2); burnin_service() lo reanuda
// cuando la energía sale de SURVIVAL (p. ej. tras cargar por USB)
static void burnin_pause(void) {
  if (!burnin) return;
  burnin = false;
  burnin_wait = true;
  if (power_save || survive_mode) board_ch4_set(false);
  DEBUG_PRINTLN("[BURNIN] Bateria critica (SURVIVAL): precalentamiento en pausa, se reanuda al salir de SURVIVAL");
}

// AIN1 actual: con el muestreo detenido (bajo consumo) una conversión única; si no, el último código del muestreo
static bool burnin_read_ch4(int32_t &code) {
  if (!adc.present()) return false;
  if (!power_save) {
    code = adc_ch4_code;
    return adc_samples > 0;
  }
  return adc.convert(Ads1219::MUX_AIN1, code);
}

// ---- Calibración del CH4 (lib/ch4_mos) ----
bool board_ch4_warm(void) {
  return ch4_on && ch4mos.warm();
}

void board_ch4_cal_status(void) {
  const Ch4Curve &c = ch4mos.curve();
  const bool datasheet = c.a == CH4_CURVE_A && c.b == CH4_CURVE_B && c.refPpm == CH4_CURVE_REF_PPM;
  DEBUG_PRINT("[CH4] R0 ");
  DEBUG_PRINT(ch4mos.r0Valid() ? String(ch4mos.r0(), 0) + " ohm" : String("sin dato (se toma al calentar)"));
  DEBUG_PRINT(" | curva Rs/R0 = ");
  DEBUG_PRINT(c.a, 3);
  DEBUG_PRINT(" x (ppm/");
  DEBUG_PRINT(c.refPpm, 0);
  DEBUG_PRINT(")^");
  DEBUG_PRINT(c.b, 3);
  DEBUG_PRINT(datasheet ? " (tipica del datasheet: ppm ESTIMADA)" : " (propia)");
  DEBUG_PRINT(" | RL ");
  DEBUG_PRINT(ch4mos.config().rlOhms, 0);
  DEBUG_PRINT(" ohm, VC ");
  DEBUG_PRINT(ch4mos.config().vcVolts, 2);
  DEBUG_PRINTLN(" V");
  DEBUG_PRINT("[CH4] ");
  if (!ch4_on) {
    DEBUG_PRINTLN("calefactor apagado (CH4:ON o CH4:BURNIN para ver Rs)");
    return;
  }
  DEBUG_PRINT("calefactor ");
  DEBUG_PRINT((millis() - ch4_on_ms) / 1000UL);
  DEBUG_PRINT(" s |");
  print_ch4_status(millis());
  DEBUG_PRINTLN("");
}

bool board_ch4_zero(void) {
  if (!ch4_on || !ch4mos.zero()) {
    DEBUG_PRINTLN("[CH4] ZERO: el sensor debe estar encendido y caliente (CH4:ON y esperar a que [SENS] muestre Rs/R0)");
    return false;
  }
  ch4_save_pending = true;
  DEBUG_PRINT("[CH4] ZERO: R0 = ");
  DEBUG_PRINT(ch4mos.r0(), 0);
  DEBUG_PRINTLN(" ohm (aire limpio); se guarda en la flash");
  return true;
}

bool board_ch4_set_r0(float ohms) {
  if (!(ohms > 100.0f && ohms < 10000000.0f)) {
    DEBUG_PRINTLN("[CH4] R0 fuera de rango (100 ohm a 10 Mohm)");
    return false;
  }
  ch4mos.setR0(ohms);
  ch4_save_pending = true;
  DEBUG_PRINT("[CH4] R0 = ");
  DEBUG_PRINT(ohms, 0);
  DEBUG_PRINTLN(" ohm; se guarda en la flash");
  return true;
}

bool board_ch4_set_curve(float a, float b, float refPpm) {
  const Ch4Curve c = {a, b, refPpm};
  if (!Ch4Mos::curveValid(c)) {
    DEBUG_PRINTLN("[CH4] Curva no valida: a entre 0.05 y 1 (Rs/R0 en ref), b negativo (-0.01 a -5), ref > 0");
    return false;
  }
  ch4mos.setCurve(c);
  ch4_save_pending = true;
  board_ch4_cal_status();
  return true;
}

void board_ch4_cal_reset(void) {
  ch4mos.setR0(0);
  ch4mos.setCurve({CH4_CURVE_A, CH4_CURVE_B, CH4_CURVE_REF_PPM});
  ch4_saved_r0 = 0;
  ch4_save_pending = false;
  if (flash_fs_begin()) InternalFS.remove(CH4_CAL_FILE);
  DEBUG_PRINTLN("[CH4] Calibracion borrada: R0 se toma sola al calentar, curva del datasheet");
}

void board_ch4_burnin_status(void) {
  DEBUG_PRINT("[BURNIN] ");
  if (burnin_wait) {
    DEBUG_PRINTLN("en pausa por bateria critica (SURVIVAL): se reanuda al salir de SURVIVAL");
    return;
  }
  if (!burnin) {
    DEBUG_PRINTLN("sin precalentamiento en curso (CH4:BURNIN para iniciar)");
    return;
  }
  int32_t code = 0;
  DEBUG_PRINT("en curso hace ");
  DEBUG_PRINT((millis() - burnin_start_ms) / 60000UL);
  DEBUG_PRINT(" min (esta sesion) | calefactor encendido hace ");
  DEBUG_PRINT((millis() - ch4_on_ms) / 60000UL);
  DEBUG_PRINT(" min");
  if (burnin_read_ch4(code)) {
    DEBUG_PRINT(" | AIN1 ");
    DEBUG_PRINT(code);
    DEBUG_PRINT(" cuentas = ");
    DEBUG_PRINT(adc.toVolts(code), 4);
    DEBUG_PRINT(" V");
  }
  DEBUG_PRINTLN("");
}

static void burnin_service(void) {
  if (burnin_resume) { // reinicio en pleno precalentamiento: se retoma tras el POST de arranque
    burnin_resume = false;
    DEBUG_PRINTLN("[BURNIN] Reinicio durante el precalentamiento: se retoma");
    board_ch4_burnin(true);
  }
  const bool survival = energy.valid() && energy.mode() == EnergySupervisor::SURVIVAL;
  if (burnin_wait && !survival) {
    DEBUG_PRINTLN("[BURNIN] La bateria salio de SURVIVAL: se reanuda el precalentamiento");
    burnin_wait = false;
    board_ch4_burnin(true);
  }
  if (!burnin) return;
  if (survival) {
    burnin_pause();
    return;
  }
  if (!ch4_on && !board_ch4_set(true)) return; // algo lo apagó (p. ej. falla del FAN53741): se reintenta
  if (burnin_logged && millis() - burnin_log_ms < BURNIN_LOG_MS) return;
  if (!board_radio_idle()) return; // la SD tarda: no durante TX/RX de LoRa
  burnin_logged = true;
  burnin_log_ms = millis();

  int32_t code = 0;
  const bool haveCode = burnin_read_ch4(code) && !Ads1219::saturated(code);
  float tempC = NAN, hum = NAN;
  if (env.present()) {
    Bme280::Sample s;
    if (power_save) { // BME280 en sleep: una medición y de vuelta a sleep
      env.wake();
      board_wait_ms(15);
    }
    if (env.read(s)) {
      tempC = s.tempC;
      hum = s.humidity;
    }
    if (power_save) env.sleep();
  }

  // En bajo consumo no hay muestreo a 40 Hz: esta lectura alimenta el estimador (línea base R0 durante las 48 h)
  if (power_save && haveCode) ch4mos.update(adc.toVolts(code), tempC, hum, millis());

  BoardTime t;
  char date[16] = "", hour[16] = "", id[20] = "", volts[16], temp[16], rh[16], bat[16], minutes[16], counts[16] = "";
  char rsTxt[16], r0Txt[16], ratioTxt[16];
  fmt_fixed(rsTxt, sizeof(rsTxt), ch4mos.rsComp(), 0);
  fmt_fixed(r0Txt, sizeof(r0Txt), ch4mos.r0Valid() ? ch4mos.r0() : NAN, 0);
  fmt_fixed(ratioTxt, sizeof(ratioTxt), ch4mos.ratio(), 4);
  if (board_time_local(t)) {
    snprintf(date, sizeof(date), "%04u-%02u-%02u", t.year, t.month, t.day);
    snprintf(hour, sizeof(hour), "%02u:%02u:%02u", t.hour, t.minute, t.second);
  }
  board_device_id(id, sizeof(id));
  if (haveCode) snprintf(counts, sizeof(counts), "%ld", (long)code);
  fmt_fixed(volts, sizeof(volts), haveCode ? adc.toVolts(code) : NAN, 4);
  fmt_fixed(temp, sizeof(temp), tempC, 2);
  fmt_fixed(rh, sizeof(rh), hum, 1);
  fmt_fixed(bat, sizeof(bat), snap.batV, 3);
  snprintf(minutes, sizeof(minutes), "%lu", (unsigned long)((millis() - ch4_on_ms) / 60000UL));
  char row[200];
  snprintf(row, sizeof(row), "%s;%s;%s;%lu;%s;%s;%s;%s;%s;%s;%s;%s;%s", date, hour, id, (unsigned long)(millis() / 1000UL),
           minutes, counts, volts, temp, rh, bat, rsTxt, r0Txt, ratioTxt);
  const char *header =
    "fecha;hora;id;uptime_s;min_calefactor;ch4_cuentas;ch4_v;temp_c;hum_pct;bat_v;ch4_rs_ohm;ch4_r0_ohm;ch4_rs_r0";
  if (board_sd_append_csv("/datos", "precalentamiento", header, row, nullptr, 0)) {
    DEBUG_PRINT("[BURNIN] ");
    DEBUG_PRINTLN(row);
  } else {
    DEBUG_PRINTLN("[BURNIN] No se pudo escribir /datos/precalentamiento.csv");
  }
}

void board_process(void) {
  stack_monitor(); // pila libre del loop y uptime para el registro de la caída
  // trace_step: qué tarea de la placa corre (va al registro si hay un HardFault o el WDT vence)
#if BOARD_ENABLE_LORA
  trace_step = "lora";
  lora.process(); // siempre primero y sin delay() en el loop
#endif
  trace_step = "ch4_burnin";
  burnin_service();      // precalentamiento del CH4 (si está activo)
  ch4_cal_service();     // guarda la calibración del CH4 en la flash (con el radio libre)
  trace_step = "leds";
  lora_leds();           // azul: buscando red, Join OK, ACK
  fault_leds();          // FAULT: código del subsistema con falla
  leds.update();         // destellos programados (verde de IDLE, azul de LoRa)
  trace_step = "sensores";
  sensors_sample();      // subsistemas a SENSOR_SAMPLE_HZ
  fan_measure();         // rpm y vueltas del blower (cada 1 s)
  trace_step = "energia";
  energy_service(false); // batería + supervisor de energía (1 s despierto, 10 s en reposo)
  trace_step = "gps";
  gps_service();         // tramas NMEA del L96-M33
#if BOARD_ENABLE_MESH
  trace_step = "mesh";
  mesh_time_acquire();   // sin hora: escucha continua hasta oír un faro HORA de otro collar
  mesh_window_service(); // anillo Meshtastic: ventana sincronizada por hora UTC (RX/TX con LBM suspendido)
  mesh_diag_service();   // sondeo de diagnóstico (RING:DIAG): escucha también fuera de las ventanas
#endif
#if BOARD_ENABLE_BLE
  trace_step = "ble";
  ble_service(); // comandos de la app y estado del POST al suscribirse
#endif
  trace_step = "debug";
  sensors_debug_print();

  // El POST corre justo después de leer la hora y no alcanza a ver avanzar el RTC: se verifica una
  // vez 5 s después del arranque (lectura I2C corta, no bloquea). Si falla, LED rojo.
  static bool rtc_check_done = false;
  if (!rtc_check_done && rtc_boot_valid && millis() - rtc_boot_ms > 5000) {
    rtc_check_done = true;
    DEBUG_PRINTLN("[RTC] Verificacion del avance del reloj (5 s despues del arranque)");
    if (board_test_rtc() == HW_FAIL) board_report_fault(POST_RTC);
  }
#if BOARD_ENABLE_LORA
  rtc_network_sync();
#endif
  // Los uplinks los decide la aplicación (app_fsm): vida en IDLE y datos al terminar cada medición

#if BOARD_ENABLE_GNSS
  trace_step = "gnss";
  gnss.process(); // hora de red, escaneo (bloqueante) y NAV en el hueco entre uplinks
#endif
  trace_step = "fsm"; // lo que sigue en la vuelta es de la aplicación (estado en trace_state)
}

#if BOARD_POST_ONLY
// ---- Firmware de prueba (Wearme_POST_Test): solo LEDs, WDT y la verificación del RTC ----
void board_alive_process(void) {
  stack_monitor();
  leds.update();
  static bool rtc_check_done = false;
  if (!rtc_check_done && rtc_boot_valid && millis() - rtc_boot_ms > 5000) {
    rtc_check_done = true;
    DEBUG_PRINTLN("[RTC] Verificacion del avance del reloj (5 s despues del arranque)");
    if (board_test_rtc() == HW_FAIL) board_report_fault(POST_RTC);
  }
  board_wdt_feed();
}

void board_alive_leds(bool postPassed, uint32_t periodMs) {
  static const LedCode ok = {LED_MASK_GREEN, 1, 60, 0};
  static const LedCode bad = {LED_MASK_RED, 1, 60, 0};
  leds.set(StatusLeds::RED, false); // board_post_result() lo deja fijo si falló: aquí destella
  leds.periodic(postPassed ? &ok : &bad, periodMs);
}

void board_gps_off(void) {
  gps_power(false, "fin de la prueba del POST");
}
#endif

// ============================================
// 8. Servicios para la aplicación (app_fsm)
// ============================================
const BoardSnapshot &board_snapshot(void) {
  return snap;
}

bool board_time_local(BoardTime &t) {
  Rx8130::DateTime utc, local;
  if (!rtc.present() || !rtc.readTime(utc)) return false;
  Rx8130::fromUnix(Rx8130::toUnix(utc) + RTC_UTC_OFFSET_H * 3600L, local);
  t = {local.year, local.month, local.day, local.hour, local.minute, local.second};
  return true;
}

bool board_sd_append_csv(const char *dir, const char *base, const char *header, const char *row, char *usedPath,
                         size_t usedLen) {
  TraceScope trace("sd");
  if (!sd.mounted()) sd.begin(); // p. ej. tarjeta insertada después del arranque
  return sd.appendCsv(dir, base, header, row, usedPath, usedLen);
}

bool board_sd_read_line(const char *path, uint32_t &pos, char *buf, size_t size) {
  TraceScope trace("sd");
  if (!sd.mounted()) sd.begin();
  return sd.readLine(path, pos, buf, size);
}

bool board_sd_write_at(const char *path, uint32_t pos, const char *data) {
  TraceScope trace("sd");
  if (!sd.mounted()) sd.begin();
  return sd.writeAt(path, pos, data);
}

bool board_sd_rewrite_filtered(const char *path, bool (*keep)(const char *line)) {
  TraceScope trace("sd");
  if (!sd.mounted()) sd.begin();
  return sd.rewriteFiltered(path, keep);
}

void board_clear_fault(uint8_t postCode) {
  if (postCode < 32) fault_mask &= ~(1UL << postCode);
}

// ---- Hora UTC (RTC) para la aplicación y el anillo ----
// El RTC se lee a lo sumo cada 10 s; entre lecturas se extrapola con millis() (el anillo lo consulta en cada vuelta)
bool board_unix_now(uint32_t &unix) {
  if (!unix_base || millis() - unix_base_ms >= 10000UL) {
    uint32_t u = 0;
    if (!rtc_unix_now(u)) {
      unix_base = 0;
      return false;
    }
    unix_base = u;
    unix_base_ms = millis();
  }
  unix = unix_base + (millis() - unix_base_ms) / 1000UL;
  return true;
}

bool board_time_trusted(void) {
  return rtc_trusted;
}

uint32_t board_unix_from_local(const BoardTime &t) {
  const Rx8130::DateTime d = {t.year, t.month, t.day, t.hour, t.minute, t.second, 0};
  return Rx8130::toUnix(d) - RTC_UTC_OFFSET_H * 3600L;
}

// ---- Anillo Meshtastic (servicios para app_fsm) ----
bool board_mesh_queue_measurement(uint32_t unixTime, const uint8_t frame[RING_FRAME_LEN]) {
#if BOARD_ENABLE_MESH
  if (!mesh_ok || unixTime == 0) return false;
  const bool ok = ring.queueOwn(unixTime, frame);
  DEBUG_PRINT(ok ? "[RING] Medicion en cola para la proxima ventana (hora " : "[RING] Cola del anillo llena: medicion (hora ");
  DEBUG_PRINT(unixTime);
  DEBUG_PRINTLN(ok ? ")" : ") solo en PENDIENTES.csv");
  if (ok && !rtc_trusted)
    DEBUG_PRINTLN("[RING] Sin hora confiable: no hay ventanas hasta tenerla (red, GPS, app/serie o faro HORA de la mesh)");
  return ok;
#else
  (void)unixTime;
  (void)frame;
  return false;
#endif
}

bool board_mesh_delivered(uint32_t unixTime) {
#if BOARD_ENABLE_MESH
  return mesh_ok && ring.ownDelivered(unixTime);
#else
  (void)unixTime;
  return false;
#endif
}

bool board_mesh_take_upload(RingUpload &u) {
#if BOARD_ENABLE_MESH
  return mesh_ok && !win_open && ring.takeUpload(u);
#else
  (void)u;
  return false;
#endif
}

void board_mesh_upload_result(const RingUpload &u, bool ack) {
#if BOARD_ENABLE_MESH
  if (mesh_ok) ring.uploadResult(u, ack, millis());
#else
  (void)u;
  (void)ack;
#endif
}

void board_mesh_link_result(bool ack) {
#if BOARD_ENABLE_MESH
  if (mesh_ok) ring.linkResult(ack, millis());
#else
  (void)ack;
#endif
}

// frame = [versión][DevEUI destino 8 B][id uint32 big endian][comando 1-12 B] (FRAME_RING_CMD_*, lib/lora_frames)
bool board_mesh_command(const uint8_t *frame, uint8_t len, const char *source) {
  if (len < FRAME_RING_CMD_HDR + 1 || frame[0] != FRAME_RING_CMD_VERSION || len > FRAME_RING_CMD_HDR + RING_CMD_MAX) {
    DEBUG_PRINTLN("[RING] Comando por el anillo con formato no valido: [01][DevEUI 8 B][id 4 B][comando 1-12 B]");
    return false;
  }
  const uint8_t *target = frame + 1;
  const uint32_t id = ((uint32_t)frame[9] << 24) | ((uint32_t)frame[10] << 16) | ((uint32_t)frame[11] << 8) | frame[12];
  const uint8_t *cmd = frame + FRAME_RING_CMD_HDR;
  const uint8_t cmdLen = len - FRAME_RING_CMD_HDR;
#if BOARD_ENABLE_LORA
  if (memcmp(target, DEV_EUI, 8) == 0) { // es para este collar: se ejecuta como un downlink normal
    board_inject_downlink(cmd, cmdLen, "anillo (destino: este collar)");
    return true;
  }
#endif
#if BOARD_ENABLE_MESH
  const bool ok = mesh_ok && ring.queueCommand(target, id, cmd, cmdLen);
  char dev[17];
  for (uint8_t i = 0; i < 8; i++) snprintf(dev + 2 * i, 3, "%02X", target[i]);
  DEBUG_PRINT(ok ? "[RING] Comando por " : "[RING] No se pudo encolar el comando por ");
  DEBUG_PRINT(source);
  DEBUG_PRINT(" para ");
  DEBUG_PRINT(dev);
  DEBUG_PRINT(" id ");
  DEBUG_PRINT(id);
  DEBUG_PRINTLN(ok ? ": sale en la proxima ventana" : " (cola llena o mesh deshabilitada)");
  return ok;
#else
  (void)id;
  (void)source;
  return false;
#endif
}

// ---- Diagnóstico del anillo (serie: RING:DIAG:<min>, RING:VECINOS, RING:DIAG:RESET) ----
void board_mesh_diag(uint16_t minutes) {
#if BOARD_ENABLE_MESH
  if (!mesh_ok) {
    DEBUG_PRINTLN("[DIAG] Anillo Meshtastic deshabilitado: no hay sondeo");
    return;
  }
  if (minutes > DIAG_MAX_MIN) minutes = DIAG_MAX_MIN;
  diag.start(minutes, millis());
  if (minutes == 0) return;
  gps_power(true, "sondeo del anillo (posicion en /datos/anillo.csv)"); // sigue encendido hasta el fin del sondeo
  DEBUG_PRINTLN("[DIAG] Registro en /datos/anillo.csv: cada evento con hora, GPS (lat/lon/sats/HDOP) y LoRaWAN");
  DEBUG_PRINT("[DIAG] Ciclo de ");
  DEBUG_PRINT(DIAG_CYCLE_S);
  DEBUG_PRINT(" s (");
  DEBUG_PRINT(DIAG_CYCLE_S - DIAG_PAUSE_S);
  DEBUG_PRINT(" s escuchando, ");
  DEBUG_PRINT(DIAG_PAUSE_S);
  DEBUG_PRINTLN(" s libres para LoRaWAN), alineado a la hora UTC");
  uint32_t unix = 0;
  if (!rtc_trusted || !board_unix_now(unix)) {
    DEBUG_PRINTLN("[DIAG] SIN HORA CONFIABLE: el sondeo empieza cuando la tenga (red, GPS, RTC:SET o faro HORA)");
    return;
  }
  const uint32_t toWindow = (MESH_WINDOW_PERIOD_S - unix % MESH_WINDOW_PERIOD_S) % MESH_WINDOW_PERIOD_S;
  DEBUG_PRINT("[DIAG] Los demas collares se suman al oir el PING en la proxima ventana del anillo (en ");
  DEBUG_PRINT(toWindow / 60);
  DEBUG_PRINT(" min ");
  DEBUG_PRINT(toWindow % 60);
  DEBUG_PRINTLN(" s); antes solo responden los que ya esten en un sondeo");
#else
  (void)minutes;
  DEBUG_PRINTLN("[DIAG] Anillo Meshtastic deshabilitado (BOARD_ENABLE_MESH)");
#endif
}

void board_mesh_diag_table(void) {
#if BOARD_ENABLE_MESH
  if (mesh_ok) diag.printTable(millis());
  else DEBUG_PRINTLN("[DIAG] Anillo Meshtastic deshabilitado");
#endif
}

void board_mesh_diag_reset(void) {
#if BOARD_ENABLE_MESH
  if (mesh_ok) diag.reset();
#endif
}

bool board_mesh_take_cmd_ack(RingCmdAck &a) {
#if BOARD_ENABLE_MESH
  return mesh_ok && ring.takeCommandAck(a);
#else
  (void)a;
  return false;
#endif
}

bool board_lora_joined(void) {
#if BOARD_ENABLE_LORA
  return lora.isJoined();
#else
  return false;
#endif
}

bool board_lora_ready(uint32_t periodMs) {
#if BOARD_ENABLE_LORA
  return lora.readyToSend(periodMs);
#else
  (void)periodMs;
  return false;
#endif
}

// Contadores de LBM al pedir el último uplink: el resultado es el primer TXDONE que llega después
static uint32_t tx_base_done = 0, tx_base_notsent = 0, tx_base_acks = 0;
static bool tx_confirmed = false;

bool board_lora_send(const uint8_t *data, uint8_t length, uint8_t fport, bool confirmed) {
#if BOARD_ENABLE_LORA
  const uint32_t done = lora.txDone(), notSent = lora.txNotSent(), acks = lora.acks();
  const bool ok = lora.sendRaw(data, length, fport, confirmed);
  if (ok) {
    tx_base_done = done;
    tx_base_notsent = notSent;
    tx_base_acks = acks;
    tx_confirmed = confirmed;
  }
  if (ok) leds.play(failcodes_event(LED_EV_LORA_TX));
  return ok;
#else
  (void)data;
  (void)length;
  (void)fport;
  (void)confirmed;
  return false;
#endif
}

bool board_lora_tx_in_flight(void) {
#if BOARD_ENABLE_LORA
  // Los uplinks vacíos (solo MAC) también dan TXDONE sin contar como pedidos: por eso >, no !=
  return lora.uplinks() > lora.txDone() + lora.txNotSent();
#else
  return false;
#endif
}

BoardTxResult board_lora_tx_result(void) {
#if BOARD_ENABLE_LORA
  if (lora.txNotSent() != tx_base_notsent) return BOARD_TX_NOT_SENT;
  if (tx_confirmed) return lora.acks() != tx_base_acks ? BOARD_TX_ACK : BOARD_TX_PENDING;
  return lora.txDone() != tx_base_done ? BOARD_TX_SENT : BOARD_TX_PENDING;
#else
  return BOARD_TX_NOT_SENT;
#endif
}

void board_device_id(char *out, size_t size) {
#if BOARD_ENABLE_LORA
  snprintf(out, size, "%02X%02X%02X%02X%02X%02X%02X%02X", DEV_EUI[0], DEV_EUI[1], DEV_EUI[2], DEV_EUI[3], DEV_EUI[4],
           DEV_EUI[5], DEV_EUI[6], DEV_EUI[7]);
#else
  snprintf(out, size, "%08lX%08lX", (unsigned long)NRF_FICR->DEVICEID[1], (unsigned long)NRF_FICR->DEVICEID[0]);
#endif
}

bool board_take_downlink(uint8_t &fport, uint8_t *data, uint8_t &length) {
  if (!downlink_pending) return false;
  fport = downlink_fport;
  length = downlink_len;
  memcpy(data, downlink_data, downlink_len);
  downlink_pending = false;
  taken_ble = downlink_ble;
  return true;
}

void board_inject_downlink(const uint8_t *data, uint8_t length, const char *source) {
  DEBUG_PRINT("[DL] Comando por ");
  DEBUG_PRINTLN(source);
  downlink_fport = 0;
  downlink_len = length > BOARD_DOWNLINK_MAX ? BOARD_DOWNLINK_MAX : length;
  memcpy(downlink_data, data, downlink_len);
  downlink_ble = strcmp(source, "BLE") == 0;
  downlink_pending = true;
}

bool board_downlink_from_ble(void) {
  return taken_ble;
}

// ---- Bajo consumo ----
// El GPS no entra aquí: lo maneja gps_power_service() (encendido hasta el fix, en cualquier estado).
void board_power_save(bool on) {
  if (on == power_save) return;
  if (on) {
    board_ch4_set(false);
    board_fan_set(false);
    board_wind_set(false); // POWER_VEL apagado
    adc_state = ADC_IDLE;
    if (imu.present()) imu.powerDown();
    if (env.present()) env.sleep();
    if (adc.present()) adc.powerDown();
    power_save = true;
    DEBUG_PRINTLN("[PWR] Bajo consumo: sensores en reposo, blower y CH4 apagados");
    return;
  }
  gps_fix_this_wake = false; // nueva medición: sin fix hasta que haya uno, se informa el respaldo
  gps_publish_backup();
  gps_power(true, "nueva medicion (hasta un fix valido)");
  if (imu.present()) imu.begin(Wire1); // vuelve a configurar 52 Hz (power-down apaga el ODR)
  if (env.present()) env.wake();
  // El ADS1219 sale de power-down con el próximo START
  power_save = false;
  DEBUG_PRINTLN("[PWR] Sensores despiertos (muestreo a 40 Hz)");
}

bool board_power_save_active(void) {
  return power_save;
}

const EnergySupervisor &board_energy(void) {
  return energy;
}

void board_energy_force(int8_t mode) {
  if (mode < 0) {
    energy.autoMode();
    DEBUG_PRINTLN("[ENERGY] Modo automatico (por bateria)");
  } else {
    energy.force(static_cast<EnergySupervisor::Mode>(mode));
    DEBUG_PRINT("[ENERGY] Modo fijado para pruebas: ");
    DEBUG_PRINTLN(energy.modeName());
  }
  energy_service(true);
}

uint32_t board_fan_pulses(void) {
  return fan_pulses;
}

// Chequeo rápido de PREPARE (pocos ms; no enciende blower ni calefactor): bits = códigos POST con falla
uint32_t board_quick_check(bool gas) {
  uint32_t mask = 0;
  uint8_t id = 0;
  if (!imu.present() || !imu.readReg(Lsm6ds3::REG_WHO_AM_I, id) || id != Lsm6ds3::WHO_AM_I_VALUE) {
    mask |= 1UL << POST_IMU;
  }
  if (!env.present() || !env.readReg(Bme280::REG_CHIP_ID, id) || id != Bme280::CHIP_ID_VALUE) {
    mask |= 1UL << POST_ENVIRONMENT;
  }
  uint16_t mv = 0;
  if (!fuelGauge.present() || !fuelGauge.readVoltageMv(mv) || mv < 2500 || mv > 4500) mask |= 1UL << POST_BATTERY;
  Rx8130::DateTime t;
  if (!rtc.present() || !rtc.readTime(t)) mask |= 1UL << POST_RTC;
  if (!sd.check() || !sd.mounted()) mask |= 1UL << POST_SD_CARD; // check() relee el CSD: detecta si la sacaron
  if (wind_on) {                                                 // POWER_VEL ya encendido en PREPARE
    uint16_t raw = 0;
    float mps = 0;
    if (!wind_read(0, raw, mps) || !wind_read(1, raw, mps)) mask |= 1UL << POST_WIND;
  }
  if (gas) {
    uint8_t cfg = 0, step = 0xFF;
    const bool adcOk = adc.present() && adc.readConfig(cfg);
    const bool potOk = ch4Load.present() && ch4Load.read(step) && step == ch4Load.stepFor(CH4_LOAD_OHMS);
    if (!adcOk || !potOk) mask |= 1UL << POST_SENSOR_CH4_H2;
  }
  return mask;
}

// ---- SURVIVE: rieles cortados ----
// Todos los SIP32431 apagados menos POWER_IMU: el LSM6DS3TR-C sin alimentación lleva SDA_2/SCL_2 a bajo
// por sus diodos y el TCA4307 desconecta I2C2, y entonces no se podría leer el MAX17048 para saber
// cuándo salir de SURVIVAL. El IMU queda en power-down (~3 uA).
void board_system_off(void) {
  DEBUG_PRINT("[PWR] Bateria agotada (");
  DEBUG_PRINT(snap.batV, 3);
  DEBUG_PRINTLN(" V): todo apagado y System OFF. Se despierta al conectar el USB.");
  if (burnin) board_ch4_burnin(false);
  board_survive(true); // rieles cortados (GPS incluido sin R154), sensores en reposo
  leds.allOff();
  gpreg_write(1, 0); // apagado controlado: el próximo arranque no es una recuperación
  delay(200);        // que salga el mensaje por USB
  if (sd_enabled()) sd_power_system_off(); // con el SoftDevice (BLE) activo, POWER se maneja por su API
  NRF_POWER->SYSTEMOFF = 1;              // el WDT no corre en System OFF; VBUS (USB) despierta con reset
  while (true) __WFE();
}

void board_survive(bool on) {
  if (on == survive_mode) return;
  survive_mode = on;
  if (on) {
    burnin_pause(); // batería crítica: el calefactor se apaga; se reanuda al salir de SURVIVAL
    board_power_save(true);
    gps_power(false, "SURVIVE");
    for (const PowerRail *r = POWER_RAILS; r->name != nullptr; r++) {
      if (strcmp(r->name, "IMU") == 0 || strcmp(r->name, "GPS") == 0) continue; // GPS: gps_power()
      if (expander.present() && ((expander.outputs() >> r->expanderBit) & 1)) rail_apply(*r, false);
    }
    DEBUG_PRINTLN("[PWR] SURVIVE: rieles cortados (menos POWER_IMU, necesario para I2C2), sueño profundo");
    return;
  }
  gps_power(true, "salida de SURVIVE"); // el SURVIVE lo cortó con los demás
  energy_apply_rails();                 // 5V_H2 vuelve según el modo de energía
  DEBUG_PRINTLN("[PWR] Salida de SURVIVE");
}

// Duerme la CPU hasta maxMs sin romper LoRaWAN: solo si LBM está inactivo (sin TX/RX en curso) y nunca
// más allá de su próxima tarea. delay() de FreeRTOS deja la CPU en System ON de bajo consumo
// (tickless idle) hasta el próximo evento; las interrupciones (USB, tacómetro) la despiertan y vuelve a
// dormir hasta cumplir el tiempo. Devuelve los ms dormidos (0 = no se pudo).
bool board_radio_idle(void) {
#if BOARD_ENABLE_LORA
  return lora.idleMs() >= 100;
#else
  return true;
#endif
}

uint32_t board_sleep(uint32_t maxMs) {
  uint32_t ms = maxMs;
#if BOARD_ENABLE_LORA
  const uint32_t idle = lora.idleMs();
  if (idle < 20) return 0;            // modem ocupado o con trabajo inminente
  if (ms > idle - 10) ms = idle - 10; // despertar antes de su próxima tarea
#endif
  if (leds.busy() && ms > 5) ms = 5;                                   // destello en curso: no dejar el LED encendido
  if (gps_powered && ms > GPS_AWAKE_SLEEP_MS) ms = GPS_AWAKE_SLEEP_MS; // no desbordar el buffer de Serial1
#if BOARD_ENABLE_MESH
  if ((win_open || acq_open || diag_open) && ms > 5) ms = 5; // ventana, escucha o sondeo: el radio va por SPI
#endif
  const uint32_t wdtLimit = WDT_TIMEOUT_S * 1000UL / 2; // el WDT sigue contando dormido: margen de la mitad
  if (ms > wdtLimit) ms = wdtLimit;
  if (ms < 2) return 0;
  board_wdt_feed();
  delay(ms);
  return ms;
}

// ---- Fallas ----
void board_report_fault(uint8_t postCode) {
  if (postCode >= 32) return;
  // Tabla de políticas del POST: TOL/OFF no llevan a FAULT (sensor no instalado o deshabilitado)
  const PostPolicy policy = post_policy(postCode);
  if (policy == POLICY_TOLERATED || policy == POLICY_DISABLED) {
    static uint32_t warned = 0;
    if (!(warned & (1UL << postCode))) { // un aviso por subsistema
      warned |= 1UL << postCode;
      DEBUG_PRINT("[FAULT] Falla ignorada por politica ");
      DEBUG_PRINT(policy == POLICY_TOLERATED ? "TOL" : "OFF");
      DEBUG_PRINT(": ");
      DEBUG_PRINTLN(post_test_name(postCode));
    }
    return;
  }
  const uint32_t bit = 1UL << postCode;
  if (!(fault_mask & bit)) {
    DEBUG_PRINT("[FAULT] Falla registrada en ");
    DEBUG_PRINTLN(post_test_name(postCode));
  }
  fault_mask |= bit;
}

uint32_t board_fault_mask(void) {
  return fault_mask;
}
