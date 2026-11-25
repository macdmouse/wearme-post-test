#ifndef POST_LIB_H
#define POST_LIB_H

#include <Arduino.h>

/* POST Test Codes: identifican cada subsistema (comando POST:0x.. y código de 4 bits del resultado).
 * No indican el orden: el POST completo corre por grupos (ver post_tests[] en post_lib.cpp). */
#define POST_ALL_SYSTEMS    0xFF
// 0x00 (NORMAL OPERATION) se quitó: el loop de la aplicación se implementa aparte
#define POST_IMU            0x01
#define POST_BATTERY        0x02
#define POST_BLOWER         0x03
#define POST_SENSOR_CH4_H2  0x04
#define POST_LORA_COMM      0x05
#define POST_BLUETOOTH      0x06
#define POST_RTC            0x07
#define POST_I2C_BUS        0x08  // bus I2C + TCA4307
#define POST_IO_EXPANDER    0x09  // TCA6408A
#define POST_POWER_RAILS    0x0A  // SIP32431
#define POST_SD_CARD        0x0B  // microSD por SPI
#define POST_ENVIRONMENT    0x0C  // BME280 (temperatura, humedad, presión) en la tarjeta MEMS
#define POST_GPS            0x0D  // GPS Quectel L96-M33 (UART) en la tarjeta RF
#define POST_WIND           0x0E  // 2 x FS3000-1015 (viento) detrás del mux TCA9543A
#define POST_WATCHDOG       0x0F  // WDT de hardware + reinicios de recuperación seguidos

/* Política de falla por subsistema (tabla post_tests[] en post_lib.cpp; se cambia en vivo con
 * POLICY:<token>=CRIT|TOL|OFF|SIM por serie):
 *   CRIT  falla -> POST FAILED y FAULT (por defecto)
 *   TOL   la prueba corre pero su falla se tolera: avisa, el POST pasa y no hay FAULT (sensor no instalado)
 *   OFF   no se prueba (SKIPPED) y sus fallas se ignoran también en PREPARE y MEASURE
 *   SIM   emula la falla sin correr la prueba (POST, PREPARE): para comprobar que el sistema salta a FAULT */
enum PostPolicy : uint8_t { POLICY_CRITICAL, POLICY_TOLERATED, POLICY_DISABLED, POLICY_SIMULATE_FAIL };

/* POST Response Structure
 * status: "PASSED", "FAILED", "SKIPPED" (sin prueba, deshabilitado o sin configurar) o "ERROR" */
struct POST_Response {
    uint8_t code;
    String status;
    String description;
};

/* Punto de entrada único:
 *   post()          -> POST inicial: muestra los comandos y prueba todos los subsistemas
 *   post(true, x)   -> estado del subsistema x (POST_SD_CARD, POST_RTC, ... o POST_ALL_SYSTEMS)
 * Si se pide un estado sin haber hecho el POST inicial, primero se ejecuta el inicial.
 * El POST completo no se detiene en la primera falla: prueba todo y resume al final. */
POST_Response post(bool initialized = false, uint8_t subsystem_code = POST_ALL_SYSTEMS);

/* Comandos por el puerto serie ("POST:ALL", "POST:SD", "POST:0x0B"...). Llamar en cada loop().
 * Una prueba puede tardar (p. ej. reiniciar la microSD): no usar con LoRa en plena transmision. */
void post_process(void);

/* Function Prototypes */
POST_Response post_request_all(void);
POST_Response post_request_subsystem(uint8_t subsystem_code);
bool post_request_command(const String &command, POST_Response *response);
POST_Response run_post_test(uint8_t test_code);
void print_post_result(POST_Response response);
const char *post_test_name(uint8_t code); // nombre del subsistema ("IMU (LSM6DS3TR-C)"...) o "DESCONOCIDO"
PostPolicy post_policy(uint8_t code);      // política actual del subsistema (CRIT si no está en la tabla)
uint32_t post_simulated_mask(void);       // subsistemas en SIM (bit = código POST)
void post_print_policies(void);           // tabla de políticas por serie
String get_formatted_post_result(POST_Response response);

/* Firmware de prueba (Wearme_POST_Test) */
uint32_t post_all_count(void);              // POST completos corridos desde el arranque
bool post_last_all_passed(void);            // resultado del último POST completo
void post_alive_command(const String &cmd); // VIVO? / VIVO:<s> (definida en src/main.cpp)
void post_table_command(const String &cmd); // TABLA? / TABLA:<s> (definida en src/main.cpp)
// Tabla por serie: cada subsistema con politica, estado, fallas/corridas y hace cuanto se probo, y debajo
// QUE REVISAR para los que fallan (en orden de dependencia). Devuelve cuantos tienen falla (sin contar toleradas).
uint8_t post_print_table(void);

#endif
