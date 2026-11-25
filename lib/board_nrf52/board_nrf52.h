// Código: board_nrf52.h
// Función: punto único de configuración e inicialización de la placa Wearme
// (nRF52840 + LR1110 en el módulo WM1110). setup() llama board_init(); board_process() lo llama la
// máquina de estados (app_fsm) en cada vuelta de loop().

#ifndef BOARD_NRF52_H
#define BOARD_NRF52_H

#include <Arduino.h>
#include <failcodes.h>
#if defined(USE_TINYUSB) && __has_include(<Adafruit_TinyUSB.h>)
#include <Adafruit_TinyUSB.h>
#endif
#include "debug_config.h"
#include "board_pins.h"
#include <energy_supervisor.h>
#include <mesh_ring.h> // RingUpload, RING_FRAME_LEN (anillo Meshtastic)

/* ==================== CONFIGURACIÓN DE LA APLICACIÓN ==================== */

#define UPLINK_PERIOD_MS 600000                         // uplink de vida en IDLE (FPort 1), 10 min: abre las ventanas RX para los downlinks
                                                        // (un MEASURE tarda hasta este tiempo en llegar; cada uplink gasta batería)
#define GNSS_PERIOD_MS         60000                    // escaneo GNSS (NAV en FPort 192, doppler en FPort 3)
#define SENSOR_SAMPLE_HZ       40                       // muestreo de los subsistemas (IMU configurado a 52 Hz, el ODR siguiente)
#define RTC_UTC_OFFSET_H       (-5)                     // zona horaria local (Colombia, UTC-5): el RTC guarda UTC
#define RTC_RESYNC_MS          (24UL * 3600UL * 1000UL) // re-sincroniza el RTC con la hora de la red cada 24 h
#define SENSOR_DEBUG_PERIOD_MS 5000                     // línea [SENS] por serie con la última muestra de cada sensor (0 = apagada)

#define CH4_LOAD_OHMS      3000  // resistencia de carga del sensor CH4 (MCP4018T-502E, R_WB)
#define FAN_NOMINAL_RPM    12500 // blower Sunon UB3H3-500: 12500 rpm ±30 % a 3 V
#define FAN_PULSES_PER_REV 3     // motor de 6 polos: 3 pulsos FG por vuelta
// Calefactor del GM-402B (CH4): VH = 2.8 V ±0.1 V; RH = 80 Ω ±20 Ω. Más de 120 mW daña el calefactor de
// forma irreversible (3.3 V sobre 60-80 Ω = 136-181 mW): no subir de 2900 mV.
#define CH4_HEATER_MV       2800 // salida del FAN53741 con HT_PWM en ON
#define CH4_HEATER_WATCH_MS 3000 // POST: tiempo que se observa AIN1 con el calefactor encendido

// CH4 como Rs/R0 (lib/ch4_mos, manual GM-402B v2.2 en hardware/). ppm ESTIMADA hasta calibrar con gas:
//   Rs = RL × (VC − V) / V, compensada a 20 °C / 55 %HR (Fig. 4); R0 = Rs en aire limpio, dinámica y guardada en la
//   flash (/ch4_cal.bin); curva Rs/R0 = a × (ppm / ref)^b, por defecto la típica de la Fig. 3, cambiable por serie
//   (CH4:CURVE=a,b,ref) sin recompilar. Comandos: CH4:CAL?, CH4:ZERO, CH4:R0=<ohm>, CH4:CURVE=..., CH4:CAL:RESET.
#define CH4_VC_MV             5000      // VC del divisor del GM-402B (por confirmar en el esquemático)
#define CH4_CURVE_A           0.72f     // Fig. 3: Rs/R0 a CH4_CURVE_REF_PPM
#define CH4_CURVE_B           (-0.204f) // Fig. 3: pendiente log-log (1000 ppm 0.72 -> 10000 ppm 0.45)
#define CH4_CURVE_REF_PPM     1000.0f
#define CH4_PPM_MAX           10000.0f  // rango del manual
#define CH4_WARMUP_MIN_MS     60000UL   // calentamiento mínimo antes de dar ppm
#define CH4_WARMUP_MAX_MS     180000UL  // PREPARE: si no se estabiliza en este tiempo se mide igual (ppm sin dato)
#define CH4_STABLE_PCT        1.0f      // caliente: Rs cambia menos de esto (%) en 10 s
#define CH4_R0_TAU_UP_S       20.0f     // R0 sube rápido (el metano solo baja Rs)
#define CH4_R0_TAU_DOWN_S     21600.0f  // R0 baja lento (6 h): sigue la deriva, no los eventos de gas
#define CH4_R0_MAX_DROP_PCT   10.0f     // al calentar, una Rs más baja que R0 hasta esto se toma como deriva
#define CH4_R0_SAVE_PCT       2.0f      // R0 se guarda en la flash al apagar el calefactor si cambió más que esto

// Calibración de salida en ppm: ppm = m × x + b, con x = voltaje del ADC (V). Por ahora m = 1, b = 0
// (salida = voltaje) hasta hacer la calibración.
#define H2_PPM_M         1.0f // GM-602B (AIN0)
#define H2_PPM_B         0.0f
#define GPS_POST_WAIT_MS 3000 // POST: espera de tramas NMEA (el L96 manda 1 vez por segundo)

// GPS L96-M33 por CTRL_GPS (TCA6408A P0, ~25 mA): la línea nRF TX -> RXD del L96 no funciona (sin $PMTK161
// standby ni comandos), así que se ahorra cortando la alimentación. Se enciende al arrancar, al empezar cada
// medición y al salir de SURVIVE; sigue encendido (en cualquier estado) hasta un fix válido, GPS_HOLD_AFTER_FIX_MS
// más, y se apaga. El último fix válido se guarda en la flash interna (InternalFS, /gps_fix.bin): sobrevive a
// los reinicios y se informa con su edad (gps_edad_s en el CSV, gps_age_min en FPort 11).
#define GPS_MIN_SATS          4         // fix válido: calidad > 0, al menos estos satélites...
#define GPS_MAX_HDOP          5.0f      // ...y HDOP hasta este valor
#define GPS_HOLD_AFTER_FIX_MS 60000UL   // tras el fix sigue encendido: mejora la posición y baja efemérides
#define GPS_SEARCH_MAX_MS     0UL       // búsqueda sin fix: 0 = sin límite (encendido hasta que haya señal)
#define GPS_SAVE_PERIOD_MS    3600000UL // el fix se guarda en flash a lo sumo cada hora (desgaste de la flash)
#define GPS_AWAKE_SLEEP_MS    50        // con el GPS encendido la CPU duerme a lo sumo esto: el buffer de
                                        // Serial1 es de 64 B (67 ms a 9600) y se perderían tramas NMEA

// Watchdog: WDT de hardware del nRF52 (reinicia si el loop deja de alimentarlo) y contador de reinicios
// de recuperación seguidos (sobrevive al reset en GPREGRET2).
#define WDT_TIMEOUT_S      60                     // sin alimentar este tiempo -> reinicio (sigue contando con la CPU dormida)
#define WDT_MAX_RECOVERIES 3                      // reinicios por watchdog seguidos antes de quedarse en FAULT
#define WDT_HEALTHY_MS     (10UL * 60UL * 1000UL) // tiempo estable que pone el contador en 0

/* ==================== MÓDULOS HABILITADOS ==================== */
// 0 = no se inicializa ni se usa. Pruebas de arranque de la placa Wearme: solo capa de hardware.

// FIRMWARE DE PRUEBA (Wearme_POST_Test): la placa se inicializa y se prueba, y nada más.
//   - LoRa: el LR1110 se enciende (el POST lee su versión por SPI) pero NO hay Join ni transmisiones:
//     LoraNode::process(), que es quien hace el Join, no se llama nunca.
//   - Bluetooth: el SoftDevice arranca y anuncia (es lo que prueba el POST); sin comandos de la app.
//   - No se escribe la flash interna: ni credenciales LoRaWAN ni número de nodo (se leen si ya están). Así una
//     tarjeta nueva no queda marcada como el nodo de este proyecto al cargarle después el firmware de producción.
//   - Sin anillo, FSM, muestreo de sensores, energía, GPS continuo ni registro en la SD.
#define BOARD_POST_ONLY 1

#define BOARD_ENABLE_BLE  1 // UART por BLE para la app (lib/ble_lib)
#define BOARD_ENABLE_LORA 1 // 0 si se quita la antena: transmitir sin carga puede dañar el PA
#define BOARD_ENABLE_GNSS 0 // usa el LR1110 a través de LoraNode: requiere LoRa

#if BOARD_ENABLE_GNSS && !BOARD_ENABLE_LORA
#error "BOARD_ENABLE_GNSS requiere BOARD_ENABLE_LORA"
#endif

#define BOARD_ENABLE_MESH 0 // anillo Meshtastic de respaldo (lib/mesh_lib + lib/mesh_ring): usa el LR1110 vía LBM
#if BOARD_ENABLE_MESH && !BOARD_ENABLE_LORA
#error "BOARD_ENABLE_MESH requiere BOARD_ENABLE_LORA"
#endif

// Anillo Meshtastic de respaldo entre collares (ver lib/mesh_ring/src/mesh_ring.h):
//   - un collar cuya medición no llega por LoRaWAN la manda por la mesh (DevEUI + hora UTC + trama de 22 B)
//   - todos repiten lo que no han visto (hasta MESH_HOP_LIMIT saltos)
//   - un collar con LoRaWAN unido es PUENTE: la reclama, la sube por FPort 20 con el DevEUI de origen y anuncia
//     ENTREGADO (el origen la da por enviada); la plataforma descarta duplicados por DevEUI + hora
// Solo en VENTANAS sincronizadas por hora UTC (red LoRaWAN, GPS o serie/BLE): MESH_WINDOW_S cada
// MESH_WINDOW_PERIOD_S, todos a la vez. Fuera de la ventana el radio es de LoRaWAN y no se escucha (~0.3 mA promedio
// con 30 s cada 10 min; ~0.6 mA con 30 s cada 5 min). Sin hora confiable el collar no abre ventanas.
// Todos los collares (y un nodo Meshtastic que se quiera usar de repetidor) con la MISMA región, preset, nombre de
// canal y clave. Por defecto el canal público LongFast (clave "AQ==": cualquiera lo lee): para datos reales usar un
// canal propio (MESH_CHANNEL_NAME + MESH_PSK de 16 bytes, AES-128; AES-256 no está soportado).
#define MESH_WINDOW_PERIOD_S    300        // una ventana cada 5 min (minuto 0, 5, 10... UTC); antes 600 (10 min).
                                           // TODOS los collares con el mismo valor: si no, no coinciden en la ventana
#define MESH_WINDOW_S           30         // duración de la ventana
// Hora por la mesh (como Meshtastic): los collares con hora mandan un faro HORA en cada ventana; uno SIN hora escucha
// en continuo hasta MESH_TIME_ACQ_MS (cubre una ventana) y la toma del faro; si no oye ninguno reintenta cada
// MESH_TIME_RETRY_MS. Calidad: ninguna < mesh < RTC conservado < manual < GPS < red (una peor nunca pisa a una mejor).
// En el faro solo va como "buena" la de manual, GPS o red; la del RTC conservado va como de la mesh y se corrige con
// un faro bueno (un RTC detenido sin VLF desfasó el anillo el 2026-11-03).
#define MESH_TIME_ACQ_MS        ((MESH_WINDOW_PERIOD_S + 120UL) * 1000UL) // cubre una ventana aun con las pausas
#define MESH_TIME_RETRY_MS      (60UL * 60UL * 1000UL)
#define MESH_TIME_ACQ_DELAY_MS  (5UL * 60UL * 1000UL) // al arrancar primero se deja hacer el join (da la hora de la red)
#define MESH_TIME_ACQ_LISTEN_MS 50000UL // la escucha va por tramos: 50 s escuchando...
#define MESH_TIME_ACQ_PAUSE_MS  10000UL // ...y 10 s con el radio libre para que LBM intente el join
#define MESH_MAX_ATTEMPTS       5          // ventanas en que un collar reintenta su medición sin ENTREGADO (tools/ring_sim.py:
                                           // con 20 % de pérdida, 3 intentos perdían 22 % en 4 enlaces; 5 pierden 7 %)
// Región: "ANZ" (915-928 MHz) pone LongFast en 919.875 MHz, dentro de la banda AU915 para la que está la antena y
// fuera de la sub-banda 2 de LoRaWAN (uplinks 916.8-918.2, downlinks desde 923.3). Con "US" quedaba en 906.875 MHz,
// fuera de AU915 (antena desintonizada y banda a verificar con la ANE). Cambio del 2026-11-17: TODOS los collares (y
// un nodo Meshtastic usado de repetidor, región ANZ en la app) con la misma, o no se oyen.
#define MESH_REGION             "ANZ"      // "US" (902-928 MHz) o "ANZ" (915-928 MHz); LoRaWAN AU915 usa 915.2-927.8
// LongFast: SF11 BW250. LongMod (SF11 BW125 CR4/8) da +3 dB y LongSlow (SF12 BW125) +5.5 dB, pero 3x y 6x más tiempo
// en el aire: con el tráfico del anillo en la ventana de MESH_WINDOW_S y del sondeo habría que alargar ambos.
#define MESH_PRESET             "LongFast" // LongFast, LongSlow, LongMod, MediumSlow, MediumFast, ShortSlow, ShortFast, ShortTurbo
#define MESH_CHANNEL_NAME       ""         // vacío = nombre del preset (canal primario por defecto)
#define MESH_PSK                {0x01}     // 1 byte = índice de la clave por defecto (1 = "AQ=="); 16 bytes = AES-128
// Potencia del anillo: el BSP del framework fija 14 dBm para todo; con > 14 se usa el PA de alta potencia
// (lib/board_nrf52/mesh_tx_power.c, -Wl,--wrap en platformio.ini). 22 dBm = +8 dB (~120 mA de pico desde VBAT
// durante cada paquete; LoRaWAN sigue en 14). En SURVIVAL vuelve a 14. Si el collar se reinicia al transmitir
// ("[RING] TX" y luego el arranque), bajar a 17-20.
#define MESH_TX_POWER_DBM       22         // LR1110: hasta 22 dBm
#define MESH_RX_BOOSTED         true       // RX boosted del LR1110: ~2 dB más de sensibilidad, ~2 mA más escuchando
#define MESH_HOP_LIMIT          3
#define MESH_FREQ_HZ            0          // 0 = calculada como el firmware (ANZ LongFast: 919.875, US: 906.875 MHz)
#define MESH_NODEINFO_PERIOD_MS (3UL * 3600UL * 1000UL) // nombre "WearMe xxxx" (app Meshtastic) cada 3 h, en una ventana
#define MESH_DEBUG_HEX          1          // imprime cada paquete en hexadecimal (tools/mesh_decode.py lo descifra)
// Diagnóstico del anillo (lib/mesh_ring/src/ring_diag.h): cada paquete oído imprime [DIAG] RX con origen, último
// salto, saltos y RSSI/SNR; tabla de vecinos al cerrar cada ventana y con RING:VECINOS. Sondeo RING:DIAG:<min>: PING
// cada DIAG_CYCLE_S, los collares que lo oyen responden PONG (señal en los dos sentidos) y escuchan fuera de las
// ventanas mientras dure (~6 mA): DIAG_CYCLE_S - DIAG_PAUSE_S escuchando y DIAG_PAUSE_S con el radio para LoRaWAN.
#define DIAG_CYCLE_S            20         // divisor de MESH_WINDOW_PERIOD_S (ciclos alineados a la hora UTC)
#define DIAG_PAUSE_S            4
#define DIAG_MAX_MIN            120        // tope de un sondeo (minutos)

/* ==================== PASOS DE INICIALIZACIÓN ==================== */
// Orden de board_init(), los mismos grupos que el POST: base de hardware (estado seguro, buses,
// energía), energía y hora, almacenamiento, sensores, actuadores y comunicaciones.

enum BoardInitStepId {
  // Base de hardware
  BOARD_INIT_LEDS,        // verde, rojo y azul encendidos: arranque en curso (hasta el POST)
  BOARD_INIT_SAFE_GPIO,   // calefactor, ventilador/LNA, CS_POT y CS de la SD en estado seguro
  BOARD_INIT_I2C,         // I2C1 (TCA6408A) e I2C2 (TCA4307)
  BOARD_INIT_IO_EXPANDER, // TCA6408A
  BOARD_INIT_POWER_RAILS, // SIP32431 (EN desde el TCA6408A)
  // Energía y hora
  BOARD_INIT_BATTERY, // MAX17048
  BOARD_INIT_RTC,     // RX8130CE
  // Almacenamiento
  BOARD_INIT_SD, // microSD por SPI
  // Sensores
  BOARD_INIT_IMU,           // LSM6DS3TR-C
  BOARD_INIT_ENVIRONMENT,   // BME280 (tarjeta MEMS)
  BOARD_INIT_SENSOR_CH4_H2, // MCP4018 (carga del CH4) + ADS1219 (AIN0 = H2, AIN1 = CH4)
  BOARD_INIT_GPS,           // L96-M33 por UART (detección del pin TX del GPS)
  BOARD_INIT_WIND,          // mux TCA9543A (0x70) + FS3000-1015 derecho (canal 0) e izquierdo (canal 1)
  // Actuadores
  BOARD_INIT_BLOWER, // blower apagado + conteo de pulsos del tacómetro
  // Comunicaciones
  BOARD_INIT_BLUETOOTH, // ANTES de LoRa: LBM usa el RNG/HFCLK del SoftDevice solo si ya está activo
  BOARD_INIT_LORA,
  BOARD_INIT_GNSS,
  BOARD_INIT_COUNT
};

void board_init_step(BoardInitStepId step); // un paso suelto (útil para depuración)
void board_init(void);                      // inicialización completa de la placa
void board_process(void);                   // llamar en cada loop(): no bloquea
// LEDs: los patrones (colores, destellos, periodos) están en las tablas de lib/failcodes
void board_post_result(bool passed); // POST pasó (evento LED_EV_POST_OK) o falló (rojo fijo); lo llama post_lib
void board_led_state_change(void);   // evento LED_EV_STATE_CHANGE (cambio de estado de la FSM)
void board_led_mode(LedMode mode);   // patrón periódico del estado; LED_MODE_FAULT = rojo fijo + códigos de falla
// Pone el RTC en hora a partir de la hora local (RTC_UTC_OFFSET_H); comando serie RTC:SET
bool board_rtc_set_local(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second);

/* ==================== ESTADO PARA EL POST ==================== */
// Cada prueba imprime su detalle con DEBUG_PRINT y devuelve el resultado.

enum HwStatus : uint8_t { HW_PASS,
                          HW_FAIL,
                          HW_SKIP };

HwStatus board_test_i2c(void);         // I2C1 e I2C2 iniciados (líneas libres al arrancar), TCA4307 listo, escaneo
HwStatus board_test_io_expander(void); // TCA6408A responde y sus registros se leen
HwStatus board_test_power_rails(void); // cada SIP32431 de la tabla tiene su EN como salida y en el nivel pedido
HwStatus board_test_sd(void);          // relee la tarjeta; si no respondía, reintenta (se puede insertar después del arranque)
HwStatus board_test_imu(void);         // LSM6DS3TR-C: WHO_AM_I 0x6A y una muestra con |a| entre 0.5 y 1.5 g
HwStatus board_test_rtc(void);         // RX8130CE: hora válida y el reloj avanza al ritmo de millis()
HwStatus board_test_environment(void); // BME280: CHIP_ID 0x60 y una medición en rango (T, HR y presión)
HwStatus board_test_mems(void);        // tarjeta MEMS (CH4/H2): MCP4018 en CH4_LOAD_OHMS, ADS1219 convierte AIN0/AIN1
HwStatus board_test_blower(void);      // enciende el blower ~2.5 s y verifica que llegan los pulsos del tacómetro
HwStatus board_test_gps(void);         // L96-M33: tramas NMEA con checksum correcto (el fix no es requisito)
HwStatus board_test_wind(void);        // TCA9543A responde y los dos FS3000 dan una lectura válida (checksum)
HwStatus board_test_watchdog(void);    // WDT corriendo; falla si hubo WDT_MAX_RECOVERIES reinicios por watchdog seguidos
HwStatus board_test_battery(void);     // MAX17048 responde (VERSION 0x001x) y el voltaje es de una celda Li+ (2.5-4.5 V)
void board_battery_log(uint32_t seconds); // BAT:LOG[=s]: lecturas del MAX17048 cada 250 ms y resumen (comparar con/sin batería)
HwStatus board_test_lora(void);        // LR1110 responde por SPI (versión) + estado del join; SKIP si BOARD_ENABLE_LORA = 0
HwStatus board_test_bluetooth(void);   // SoftDevice activo y anunciando (o conectado); SKIP si BOARD_ENABLE_BLE = 0

/* ==================== CONTROL DE ENERGÍA ==================== */

/* ==================== DATOS Y SERVICIOS PARA LA APLICACIÓN (app_fsm) ==================== */

// Últimos valores de los sensores, actualizados por board_process() (40 Hz; batería y blower cada 1 s).
// NAN = sin dato (sensor ausente, saturado o apagado).
struct BoardSnapshot {
  uint32_t tick;                      // aumenta en cada muestreo de 25 ms
  float batV, batSoc;                 // MAX17048
  float batCrate, crateFiltered;      // C-rate del MAX17048 (%/h) y el filtrado del supervisor de energía
  float energyMode;                   // EnergySupervisor: 0 SURVIVAL, 1 CORE, 2 RICH
  float tempC, humidity, pressureHpa; // BME280
  float ax, ay, az;                   // LSM6DS3TR-C, g
  float gx, gy, gz;                   // dps
  float imuTempC;
  float h2V, ch4V;               // ADS1219 AIN0 (GM-602B) / AIN1 (GM-402B)
  float h2Ppm;                   // H2: calibración lineal H2_PPM_M × V + H2_PPM_B
  float ch4Raw;                  // AIN1 en cuentas crudas del ADS1219 (24 bits con signo, ±2^23), sin dato si saturado
  float ch4Ppm;                  // estimada con Rs/R0 (lib/ch4_mos); sin dato mientras calienta o sin R0
  float ch4RsOhm;                // Rs del GM-402B compensada a 20 °C / 55 %HR (con el calefactor encendido)
  float ch4R0Ohm;                // línea base R0 (Rs en aire limpio)
  float ch4Ratio;                // Rs / R0 (1 = aire limpio; baja con el metano); sin dato mientras calienta
  float ch4Warm;                 // 1 = sensor caliente y estable, 0 = calentando; sin dato con el calefactor apagado
  float ch4TotalG, ch4RateGDay;  // MethaneState de la medición en curso: g acumulados y tasa (g/día); solo en MEASURE
  float ch4Event;                // 1 = evento activo (|dppm/dt| >= 500 ppm/s), 0 = no; solo en MEASURE
  float fanRpm;                  // blower (0 apagado)
  float gpsFix, gpsSats;         // L96-M33: calidad del fix (0 = sin fix) y satélites; sin dato con el GPS apagado
  float gpsLat, gpsLon, gpsAltM; // solo con fix
  float windRight, windLeft;     // FS3000-1015 (m/s): derecho = canal 0, izquierdo = canal 1 del TCA9543A
  float gpsAgeS;                 // 0 = fix de esta medición; > 0 = respaldo (s desde el último fix, también de antes
                                 // de reiniciar: flash + RTC); sin dato = nunca hubo fix o edad desconocida
  float rssiDbm, snrDb;          // del último downlink recibido (sin dato hasta el primero)
};
const BoardSnapshot &board_snapshot(void);

// El framework compila con -Ofast (incluye -ffast-math): el compilador supone que no hay NaN e isnan()
// siempre da false. "Sin dato" se verifica por los bits del float (exponente todo en 1 = NaN o infinito).
static inline bool board_has_data(float v) {
  uint32_t bits;
  memcpy(&bits, &v, sizeof(bits));
  return (bits & 0x7F800000u) != 0x7F800000u;
}

struct BoardTime {
  uint16_t year;
  uint8_t month, day, hour, minute, second;
};
bool board_time_local(BoardTime &t); // hora del RTC en hora local (RTC_UTC_OFFSET_H)

// Fila CSV en la microSD (ver MicroSd::appendCsv); monta la tarjeta si hace falta
bool board_sd_append_csv(const char *dir, const char *base, const char *header, const char *row, char *usedPath,
                         size_t usedLen);

bool board_lora_joined(void);
// Identificador del collar: número de nodo (include/wearme_node.h para programar; claves en
// include/wearme_credentials.h) guardado en la flash la primera vez; no se sobreescribe al cargar firmware.
// "NODO-003", Bluetooth "WearMe_12_N003". NODE:SET=<nodo> (serie) cambia credenciales + nodo desde la tabla.
const char *board_node_name(void);
const char *board_ble_name(void);
bool board_node_set(uint16_t num);
bool board_node_set_id(uint16_t num); // solo el número de nodo (sin credenciales): collares con credenciales propias
bool board_node_stored(void); // el nodo está grabado en la flash (en el firmware de prueba puede ser solo el de RAM)
#if BOARD_ENABLE_LORA
// Credenciales LoRaWAN en la flash (/lorawan.bin): el primer arranque graba las del firmware y desde ahí no se
// sobreescriben. Comandos serie LORA:CRED?, LORA:CRED:SET <DevEUI> <JoinEUI> <AppKey>, LORA:CRED:RESET.
void board_lora_cred_status(void);                                         // en uso y en la flash (AppKey completa)
bool board_lora_cred_set(const uint8_t *dev, const uint8_t *join, const uint8_t *key); // solo si no hay guardadas
void board_lora_cred_reset(void);                                          // borra: el próximo arranque graba las del firmware
#endif
// Período de la medición automática guardado en la flash (minutos, 0 = apagada); defaultMin si nunca se guardó
uint16_t board_measure_period_min(uint16_t defaultMin);
void board_save_measure_period_min(uint16_t minutes);
// Hora UTC del RTC (false sin RTC); confiable = puesta por la red, el GPS o a mano, o el RTC no la perdió
bool board_unix_now(uint32_t &unix);
bool board_time_trusted(void);
uint32_t board_unix_from_local(const BoardTime &t); // hora local (RTC_UTC_OFFSET_H) -> unix UTC
// Anillo Meshtastic (lib/mesh_ring): medición propia que no llegó por LoRaWAN (sale en la próxima ventana)
bool board_mesh_queue_measurement(uint32_t unixTime, const uint8_t frame[RING_FRAME_LEN]);
bool board_mesh_delivered(uint32_t unixTime);   // un puente confirmó (ENTREGADO) esa medición propia
bool board_mesh_take_upload(RingUpload &u);      // puente: próxima medición ajena a subir por FPort 20
void board_mesh_upload_result(const RingUpload &u, bool ack); // con ACK se anuncia ENTREGADO en la próxima ventana
void board_mesh_link_result(bool ack); // uplink confirmado propio: sin ACK el collar deja de ser puente un rato
// Comandos por el anillo. Downlink FPort 21 (lib/lora_frames): si el destino es este collar se ejecuta aquí, si no
// se difunde por el anillo. source = "LoRaWAN" o "serie" (prueba sin gateway: RING:CMD)
bool board_mesh_command(const uint8_t *frame, uint8_t len, const char *source);
bool board_mesh_take_cmd_ack(RingCmdAck &a); // puente: COMANDO OK oído, para informar por FPort 22
// Diagnóstico del anillo: sondeo PING/PONG por minutes (0 = terminar), tabla de vecinos y borrarla
void board_mesh_diag(uint16_t minutes);
void board_mesh_diag_table(void);
void board_mesh_diag_reset(void);
bool board_lora_ready(uint32_t periodMs);                                                         // pasó periodMs desde el último envío
bool board_lora_send(const uint8_t *data, uint8_t length, uint8_t fport, bool confirmed = false); // false si no cabe o no hay join
// Resultado del último board_lora_send(). El TXDONE de LBM no dice de qué uplink es y LBM manda uplinks propios
// (p. ej. DeviceTimeReq tras el join) con su propio TXDONE; el ACK de un confirmado llegó hasta 73 s después de
// pedirlo (2026-10-16). Por eso un confirmado queda PENDING hasta que llega un ACK o LBM lo rechaza (NOT_SENT):
// el que llama decide cuándo se agotó el tiempo. Un no confirmado es SENT con el siguiente TXDONE.
enum BoardTxResult : uint8_t { BOARD_TX_PENDING,
                               BOARD_TX_ACK,
                               BOARD_TX_SENT,
                               BOARD_TX_NO_ACK,
                               BOARD_TX_NOT_SENT };
BoardTxResult board_lora_tx_result(void); // ACK = confirmado recibido; SENT = no confirmado transmitido
// Hay un uplink pedido cuyo TXDONE no ha llegado (p. ej. el bit de vida tras el join tarda decenas de s).
// Antes de un envío cuyo resultado importa hay que esperar a que sea false: si no, su TXDONE se confunde.
bool board_lora_tx_in_flight(void);
// microSD por líneas (ver MicroSd::readLine/writeAt/rewriteFiltered); montan la tarjeta si hace falta
bool board_sd_read_line(const char *path, uint32_t &pos, char *buf, size_t size);
bool board_sd_write_at(const char *path, uint32_t pos, const char *data);
bool board_sd_rewrite_filtered(const char *path, bool (*keep)(const char *line));
void board_device_id(char *out, size_t size); // DevEUI en hexadecimal (17 caracteres con el terminador)
// Precalentamiento (burn-in) del GM-402B: el fabricante pide 48 h con el calefactor encendido antes del primer uso.
// Mantiene el CH4 encendido en cualquier estado de la FSM (board_ch4_set(false) se ignora), registra cada
// BURNIN_LOG_MS una fila en /datos/precalentamiento.csv y sigue tras un reinicio (bit 4 de GPREGRET2; se pierde
// al quitar la alimentación). Se detiene solo con CH4:BURNIN:OFF o si la energía pasa a SURVIVAL.
// Gasta ~98 mW (2.8 V² / 80 Ω), ~30 mA de batería: hacerlo con USB o cargador conectado.
#define BURNIN_LOG_MS 60000UL
// Sin batería el MAX17048 mide la salida del cargador: SOC > 100 % y C-rate absurdo. Medido 2026-10-19 sin batería:
// 4.17-4.23 V, SOC 105.4 %, +2100 a +2800 %/h. Con batería: SOC <= 100.5 % y |C-rate| de decenas de %/h.
#define BAT_ABSENT_SOC_PCT 103.0f
// POST:BATTERY: lecturas cada 250 ms durante BAT_CHECK_MS. Sin batería el voltaje es el del cargador y sube y baja
// (medido 2026-11-25 con USB: sin batería 140-150 mV en 6 s, 4.08 -> 4.23 V y vuelve a caer; con batería 2-4 mV).
// Si varía BAT_ABSENT_SWING_MV o más -> SIN BATERIA.
// Bajo BAT_BAD_MV (con USB cargando) -> BATERIA MALA O MUY DESCARGADA. Ambas son FALLA del POST
// POST:POWER: CTRL_GPS que no baja (queda en ALTO aunque se pida OFF; R154 o algo que lo une a VDD). 1 = se tolera:
// se avisa en el detalle pero la prueba de rieles sigue en OK (el GPS queda siempre encendido, ~25 mA). 0 = FALLA
#define RAILS_GPS_STUCK_TOLERATED 1
#define BAT_CHECK_MS        6000UL
#define BAT_ABSENT_SWING_MV 30
#define BAT_BAD_MV          3000
// Corte por batería agotada: bajo este voltaje en BAT_CUTOFF_READINGS lecturas seguidas (y sin USB) la FSM
// registra la falla y llama board_system_off(). En la descarga del 16-18 oct: 3.30 V ≈ 2 % restante,
// 3.0 V 1.5 h después y 2.43 V media hora más tarde (celda dañada si no se corta).
#define BAT_CUTOFF_MV          3300
#define BAT_CUTOFF_READINGS    3
#define BAT_ABSENT_CRATE_PCT_H 300.0f
bool board_battery_cutoff(void); // batería bajo BAT_CUTOFF_MV en BAT_CUTOFF_READINGS lecturas seguidas, sin USB
// Apaga todo y entra en System OFF (~µA). Solo se sale con un reset o al conectar el USB (VBUS despierta al
// nRF52840). Con carga solar sin USB NO despierta: pendiente despertar con PGOOD (P1.04) o la alarma del RTC.
[[noreturn]] void board_system_off(void);
bool board_ch4_burnin(bool on);
// Calibración del CH4 (lib/ch4_mos): se guardan en la flash
bool board_ch4_warm(void);                       // sensor caliente y estable (ppm disponible)
void board_ch4_cal_status(void);                 // por serial: R0, curva, Rs actual y estado
bool board_ch4_zero(void);                       // R0 = Rs actual (aire limpio, sensor caliente)
bool board_ch4_set_r0(float ohms);
bool board_ch4_set_curve(float a, float b, float refPpm);
void board_ch4_cal_reset(void);                  // R0 sin dato y curva del datasheet
bool board_ch4_burnin_active(void);
void board_ch4_burnin_status(void); // por serial: tiempo, cuentas y voltios actuales
void board_methane_start(void);     // MEASURE: MethaneState nuevo e integración en cada muestra de 40 Hz
void board_methane_stop(void);      // fin de la medición: deja de integrar
bool board_gps_powered(void);       // CTRL_GPS encendido (buscando o manteniendo el fix)

// App por BLE (protocolo del firmware V12). La app recibe líneas de texto terminadas en '\n':
//   - estado del POST (JSON) al suscribirse y tras cada POST completo: {"POST":"OK","DEVEUI":...,"SD":0,...}
//   - energía (JSON) al empezar cada medición: {"Vbat":...,"SOC":...,"Mode":"RICH"}
//   - datos (CSV) al terminar cada medición (lo arma app_fsm con board_ble_send)
// Comandos de la app (board_process): "3" o "MEASURE" = medir, "1" = vista en vivo (LIVE), "0" = parar, "POST" = POST,
// "RTC:DD/MM/AAAA HH:MM:SS" o "RTC:SET AAAA-MM-DD HH:MM:SS" = poner en hora (hora local).
bool board_ble_send(const char *line); // false si la app no está suscrita
bool board_ble_subscribed(void);       // la app está conectada y recibe notificaciones
void board_ble_send_energy(void);

// Último downlink de aplicación pendiente (o un comando serie que lo imita). La FSM lo interpreta.
static constexpr uint8_t BOARD_DOWNLINK_MAX = 32; // FPort 21 (comando por el anillo) usa hasta 13 + 12 B
bool board_take_downlink(uint8_t &fport, uint8_t *data, uint8_t &length); // data de BOARD_DOWNLINK_MAX bytes
void board_inject_downlink(const uint8_t *data, uint8_t length, const char *source);
bool board_downlink_from_ble(void); // el último comando entregado por board_take_downlink() vino de la app por BLE

// Bajo consumo (IDLE y FAULT): blower y CH4 apagados, sin muestreo, IMU en power-down, BME280 en sleep,
// ADS1219 en power-down y GPS en standby. false = todo despierto y muestreando (MEASURE, POST).
void board_power_save(bool on);
bool board_power_save_active(void);

// Supervisor de energía (RICH / CORE / SURVIVAL), actualizado por board_process()
const EnergySupervisor &board_energy(void);
void board_energy_force(int8_t mode); // pruebas: 0 SURVIVAL, 1 CORE, 2 RICH, -1 automático (ENERGY:...)
// SURVIVE: bajo consumo + todos los rieles cortados (menos POWER_IMU, ver board_nrf52.cpp)
void board_survive(bool on);
// Duerme la CPU hasta maxMs si LBM está inactivo; devuelve los ms dormidos (0 = no se pudo)
uint32_t board_sleep(uint32_t maxMs);
bool board_radio_idle(void); // true si LBM no tiene TX/RX en curso ni inminentes (apto para trabajo largo, p. ej. SD)
// PREPARE: chequeo rápido (sin encender blower ni calefactor); bits = códigos POST con falla
uint32_t board_quick_check(bool gas);
uint32_t board_fan_pulses(void); // pulsos del tacómetro desde el arranque
// Sensores de viento: POWER_VEL (tarjetas de los FS3000) + muestreo de los dos canales del mux
bool board_wind_set(bool on);

/* ==================== WATCHDOG ==================== */
// WDT de hardware: se arranca en board_init() y lo alimenta la FSM en cada vuelta (y las esperas largas del
// POST). No se puede detener: solo un reinicio lo apaga.
void board_wdt_feed(void);
// Reinicio controlado por el watchdog de software (estado de la FSM atascado): cuenta como recuperación
void board_wdt_software_reset(const char *reason);
// Ciclo completo sin problemas: el contador de reinicios de recuperación vuelve a 0
void board_wdt_healthy(void);
const char *board_wdt_last_cause(void); // causa del último reinicio de recuperación ("" si fue encendido normal)
uint8_t board_wdt_recoveries(void);     // reinicios por watchdog seguidos (0 = arranque normal)
void board_trace_state(const char *name); // estado de la FSM para el registro de una caída (HardFault/WDT)
uint32_t board_stack_free_min(void);     // pila mínima libre de la tarea del loop en bytes (4 KB en total; 0 = sin medir)
void board_clear_fault(uint8_t postCode); // quita una falla de la máscara (p. ej. el watchdog ya estable en FAULT)
// Reinicia en el bootloader en modo OTA DFU (actualización por Bluetooth). No vuelve. Solo por LoRa/anillo/serie:
// el BLE no tiene autenticación (app_fsm rechaza el comando si llega por BLE).
void board_enter_ota_dfu(const char *source);
#define WEARME_FW_VERSION "POST-TEST 1.4" // firmware de prueba de la placa (BOOT?), basado en Wearme 3.1.1
void board_bootloader_info(void); // por serie (BOOT?): versión del firmware y del bootloader instalado
bool board_usb_powered(void);     // VBUS presente (USB o cargador conectado)
void board_enter_uf2_dfu(const char *source); // por serie (DFU:UF2): bootloader UF2 (unidad USB), para pruebas
void board_enter_serial_dfu(const char *source); // por serie (DFU:SERIAL): bootloader serie, para pruebas

// Fallas: cada subsistema con falla queda marcado (bit = código POST) hasta reiniciar. La FSM pasa a FAULT.
void board_report_fault(uint8_t postCode);
uint32_t board_fault_mask(void);

bool board_rail_set(const char *name, bool on); // enciende/apaga un SIP32431 por nombre (tabla en board_nrf52.cpp)
bool board_fan_set(bool on);                    // blower (FAN_CONTROL P0.28); comando serie FAN:ON / FAN:OFF
// Sensor CH4 encendido/apagado; comando serie CH4:ON / CH4:OFF.
//   ON:  HT_PWM ON (Q5 a GND) -> 5V_CH4 ON -> FAN53741 en CH4_HEATER_MV -> salida activada
//   OFF: salida del FAN53741 apagada -> 5V_CH4 OFF -> HT_PWM OFF
bool board_ch4_set(bool on);

#if BOARD_POST_ONLY
// Firmware de prueba: en lugar de board_process(), en cada loop(). Atiende los LEDs, el WDT, la pila y la
// verificación del avance del RTC (5 s después del arranque). No hace nada más.
void board_alive_process(void);
// LED de vida: un destello cada periodMs, verde si el POST pasó y rojo si falló. Destellando = el micro está vivo;
// fijo o apagado = colgado (el WDT lo reinicia en WDT_TIMEOUT_S).
void board_alive_leds(bool postPassed, uint32_t periodMs);
// Apaga el GPS (el POST lo deja encendido buscando fix; aquí no hay quien lo apague después)
void board_gps_off(void);
#endif

#endif
