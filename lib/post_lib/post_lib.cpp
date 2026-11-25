#include "post_lib.h"
#include "board_nrf52.h"
#include <stdlib.h>
#include <string.h>

/* Tabla de pruebas POST: grupo, codigo, nombre, comando serie (y alias) y funcion de prueba.
 * El POST completo las ejecuta en este orden, de lo que otros necesitan a lo que depende de todo:
 *   1. Base de hardware: buses I2C, expansor y rieles (sin ellos los demas chips no responden)
 *   2. Energia y hora: bateria y RTC
 *   3. Almacenamiento: microSD
 *   4. Sensores: IMU, ambiente (BME280) y CH4/H2
 *   5. Actuadores: blower
 *   6. Comunicaciones: LoRa y Bluetooth
 * El codigo de cada prueba no cambia con el orden (los comandos POST:0x.. siguen igual).
 *
 * policy = politica de falla POR DEFECTO de cada subsistema (ver PostPolicy en post_lib.h): aqui se elige
 * cuales pueden fallar sin detener el sistema (TOL), cuales no se prueban (OFF) y cuales emulan la falla
 * (SIM) para verificar que el sistema salta a FAULT. En vivo: POLICY:<token>=CRIT|TOL|OFF|SIM. */
struct PostTest {
    const char *group;
    uint8_t code;
    const char *name;
    const char *token;
    const char *alias;
    HwStatus (*run)(void);
    PostPolicy policy;
    const char *check; // que revisar en la placa si la prueba falla (columna de la tabla TABLA?)
};

static const PostTest post_tests[] = {
    {"BASE DE HARDWARE", POST_I2C_BUS,       "I2C BUSES (I2C1, I2C2/TCA4307)", "I2C",     nullptr,     board_test_i2c,         POLICY_CRITICAL,
     "I2C2 sin dispositivos o SDA/SCL en bajo: TCA4307, pull-ups de SDA_2_OUT/SCL_2_OUT, linea a GND; arreglar antes que el resto"},
    {"BASE DE HARDWARE", POST_IO_EXPANDER,   "IO EXPANDER (TCA6408A)",         "EXPANDER", "TCA6408",  board_test_io_expander, POLICY_CRITICAL,
     "TCA6408A 0x20 en I2C1 (P0.11/P0.16): soldadura, ADDR a GND, RESET# (R80); config perdida = caida de VDD"},
    {"BASE DE HARDWARE", POST_POWER_RAILS,   "POWER RAILS (SIP32431)",         "POWER",   "RAILS",     board_test_power_rails, POLICY_CRITICAL,
     "Pin EN del SIP32431 marcado en [POST] (salida del TCA6408A): nivel != pedido -> corto o carga en EN; GPS que no baja = R154 en la tarjeta RF"},
    {"BASE DE HARDWARE", POST_WATCHDOG,      "WATCHDOG (WDT + reinicios)",     "WDT",     "WATCHDOG",  board_test_watchdog,    POLICY_CRITICAL,
     "WDT detenido o reinicios por watchdog seguidos: ver causa del ultimo reinicio y que bloquea el loop"},
    {"ENERGIA Y HORA",   POST_BATTERY,       "BATTERY (MAX17048)",             "BATTERY", "BAT",       board_test_battery,     POLICY_CRITICAL,
     "MAX17048 0x36 no responde: soldadura y bus I2C2 (TCA4307); voltaje fuera de 2.5-4.5 V: conexion de bateria/cargador"},
    {"ENERGIA Y HORA",   POST_RTC,           "RTC (RX8130CE)",                 "RTC",     nullptr,     board_test_rtc,         POLICY_CRITICAL,
     "RX8130CE 0x32: hora invalida -> RTC:SET; no avanza -> cristal 32.768 kHz y respaldo del RTC"},
    {"ALMACENAMIENTO",   POST_SD_CARD,       "SD CARD",                        "SD",      nullptr,     board_test_sd,          POLICY_CRITICAL,
     "microSD insertada y en FAT32, contactos del zocalo, CS P0.26; SPI compartido con CS_POT (P1.01)"},
    {"SENSORES",         POST_IMU,           "IMU (LSM6DS3TR-C)",              "IMU",     nullptr,     board_test_imu,         POLICY_CRITICAL,
     "LSM6DS3TR-C 0x6A: riel POWER_IMU (U80, P2 del expansor), soldadura; |a| 0.5-1.5 g con la placa quieta"},
    {"SENSORES",         POST_ENVIRONMENT,   "ENVIRONMENT (BME280)",           "ENV",     "BME280",    board_test_environment, POLICY_CRITICAL,
     "BME280 0x76 en la tarjeta MEMS: tarjeta y cable conectados; CHIP_ID 0x60 (no un BMP280); lectura en rango"},
    {"SENSORES",         POST_SENSOR_CH4_H2, "SENSOR CH4/H2 (MEMS)",           "CH4",     "MEMS",      board_test_mems,        POLICY_CRITICAL,
     "Tarjeta MEMS: MCP4018 0x2F, ADS1219 0x41 (DRDY), FAN53741 0x52; riel CTRL_CH4 (P1 del expansor)"},
    {"SENSORES",         POST_GPS,           "GPS (L96-M33)",                  "GPS",     "L96",       board_test_gps,         POLICY_CRITICAL,
     "Sin flancos: CTRL_GPS (P0)/R154 y FPC2 de la tarjeta RF; bits sin tramas: 9600 baud; P0.14/P1.09"},
    // FS3000 instalados: su falla cuenta (POLICY:WIND=TOL por serie para tolerarla en una placa sin ellos)
    {"SENSORES",         POST_WIND,          "WIND (FS3000-1015 x2, TCA9543A)", "WIND",   "FS3000",    board_test_wind,        POLICY_CRITICAL,
     "Mux TCA9543A 0x70 y FS3000 (canal 0 der., 1 izq.); riel POWER_VEL (P3) y cables de las tarjetas FS3000"},
    {"ACTUADORES",       POST_BLOWER,        "BLOWER (tacometro)",             "BLOWER",  "FAN",       board_test_blower,      POLICY_CRITICAL,
     "Sin pulsos FG en P0.13: el blower gira? FAN_CONTROL P0.28, cable FG; rpm lejos -> FAN_PULSES_PER_REV"},
    {"COMUNICACIONES",   POST_LORA_COMM,     "LORA COMM",                      "LORA",    nullptr,     board_test_lora,        POLICY_CRITICAL,
     "LR1110 (WM1110) no responde por SPI: alimentacion del modulo, RESET/BUSY; si estaba ocupado, repetir"},
    {"COMUNICACIONES",   POST_BLUETOOTH,     "BLUETOOTH",                      "BT",      "BLUETOOTH", board_test_bluetooth,   POLICY_CRITICAL,
     "SoftDevice no arranco: ver [INIT] Bluetooth; S140 6.1.1 y bootloader grabados (BOOT?)"},
};
static const uint8_t POST_TEST_COUNT = sizeof(post_tests) / sizeof(post_tests[0]);

// Ultimo resultado de cada prueba (para la tabla TABLA?): se actualiza en run_one, en el POST completo o suelto
enum PostLast : uint8_t { LAST_NONE, LAST_PASS, LAST_FAIL, LAST_TOL, LAST_SKIP };
struct PostRecord {
    PostLast last;
    uint16_t runs;
    uint16_t fails; // FAILED + TOLERATED
    uint32_t at_ms; // millis() de la ultima corrida
};
static PostRecord records[POST_TEST_COUNT];

// Política actual (copia de la columna policy; se cambia por serie con POLICY:...)
static PostPolicy policies[POST_TEST_COUNT];
static bool policies_ready = false;

static void policies_init(void) {
    if (policies_ready) return;
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) policies[i] = post_tests[i].policy;
    policies_ready = true;
}

static const char *policy_text(PostPolicy p) {
    switch (p) {
        case POLICY_TOLERATED:     return "TOL";
        case POLICY_DISABLED:      return "OFF";
        case POLICY_SIMULATE_FAIL: return "SIM";
        default:                   return "CRIT";
    }
}

PostPolicy post_policy(uint8_t code) {
    policies_init();
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        if (post_tests[i].code == code) return policies[i];
    }
    return POLICY_CRITICAL;
}

uint32_t post_simulated_mask(void) {
    policies_init();
    uint32_t mask = 0;
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        if (policies[i] == POLICY_SIMULATE_FAIL && post_tests[i].code < 32) mask |= 1UL << post_tests[i].code;
    }
    return mask;
}

void post_print_policies(void) {
    policies_init();
    DEBUG_PRINTLN("[POST] Politica de falla (CRIT = falla a FAULT, TOL = tolerada, OFF = sin prueba, SIM = falla emulada):");
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        DEBUG_PRINT("[POST]   ");
        DEBUG_PRINT(post_tests[i].token);
        DEBUG_PRINT(" = ");
        DEBUG_PRINT(policy_text(policies[i]));
        DEBUG_PRINT(policies[i] != post_tests[i].policy ? " (cambiada) | " : " | ");
        DEBUG_PRINTLN(post_tests[i].name);
    }
}

static const PostTest *find_test(uint8_t code) {
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        if (post_tests[i].code == code) return &post_tests[i];
    }
    return nullptr;
}

static const char *status_text(HwStatus status) {
    switch (status) {
        case HW_PASS: return "PASSED";
        case HW_FAIL: return "FAILED";
        default:      return "SKIPPED";
    }
}

static bool parse_subsystem_token(String token, uint8_t *code) {
    token.trim();
    token.toUpperCase();

    if (token == "ALL") {
        *code = POST_ALL_SYSTEMS;
        return true;
    }

    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        const PostTest &t = post_tests[i];
        if (token == t.token || (t.alias != nullptr && token == t.alias)) {
            *code = t.code;
            return true;
        }
    }

    long parsed = -1;
    if (token.startsWith("0X")) {
        char *end_ptr = nullptr;
        parsed = strtol(token.c_str(), &end_ptr, 16);
        if (end_ptr == token.c_str() || *end_ptr != '\0') return false;
    } else if (token.length() > 0 && isDigit(token[0])) {
        parsed = token.toInt();
    }
    if (parsed < 0 || parsed > 255) return false;

    *code = static_cast<uint8_t>(parsed);
    return (*code == POST_ALL_SYSTEMS) || find_test(*code) != nullptr;
}

static bool post_initial_done = false;

static void post_init(void) {
    DEBUG_PRINTLN("\n=== POST INIT (firmware de prueba: solo POST y senal de vida) ===");
    DEBUG_PRINTLN("Comandos soportados (puerto serie):");
    DEBUG_PRINTLN("  POST:ALL                      POST completo");
    DEBUG_PRINT("  POST:<");
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        DEBUG_PRINT(post_tests[i].token);
        DEBUG_PRINT(i + 1 < POST_TEST_COUNT ? "|" : ">\n");
    }
    DEBUG_PRINTLN("                                un subsistema (ej. POST:SD, POST:I2C, POST:0x0B)");
    DEBUG_PRINTLN("  VIVO?                         estado ahora; VIVO:<s> periodo de la linea [VIVO] (0 = apagada)");
    DEBUG_PRINTLN("  TABLA?                        tabla de subsistemas y que revisar; TABLA:<s> periodo (0 = solo al cambiar)");
    DEBUG_PRINTLN("  POLICY:<token>=CRIT|TOL|OFF|SIM, POLICY:LIST   politica de falla (hasta reiniciar)");
    DEBUG_PRINTLN("  RTC:SET AAAA-MM-DD HH:MM:SS   poner en hora el RTC (hora local)");
    DEBUG_PRINTLN("  FAN:ON / FAN:OFF              blower | CH4:ON / CH4:OFF  calefactor del sensor CH4");
    DEBUG_PRINTLN("  BAT:LOG[=s]                   lecturas del MAX17048 cada 250 ms (10 s; con y sin bateria)");
    DEBUG_PRINTLN("  WDT:HANG / WDT:SW             prueba del watchdog (cuelga el loop / reinicio de software)");
    DEBUG_PRINTLN("  NODE? / BOOT?                 identidad del collar / firmware y bootloader");
    DEBUG_PRINTLN("  NODE:SET=<n>                  graba el nodo n (credenciales de la tabla) antes del firmware principal");
    DEBUG_PRINTLN("  NODE:ID=<n>                   graba solo el numero de nodo (credenciales propias)");
    DEBUG_PRINTLN("  LORA:CRED? / LORA:CRED:RESET / LORA:CRED:SET <DevEUI> <JoinEUI> <AppKey>  credenciales LoRaWAN");
    DEBUG_PRINTLN("  DFU:OTA / DFU:UF2 / DFU:SERIAL  bootloader (Bluetooth: pio run -t ota / unidad USB / serie)");
    post_print_policies();
}

const char *post_test_name(uint8_t code) {
    const PostTest *test = find_test(code);
    return test ? test->name : "DESCONOCIDO";
}

POST_Response post(bool initialized, uint8_t subsystem_code) {
    if (!initialized || !post_initial_done) {
        post_init();
        POST_Response response = post_request_all();
        post_initial_done = true;
        if (!initialized) {
            return response;
        }
    }

    if (subsystem_code == POST_ALL_SYSTEMS) {
        return post_request_all();
    }
    return post_request_subsystem(subsystem_code);
}

static uint32_t post_all_runs = 0;
static bool post_all_passed = false;
uint32_t post_all_count(void) { return post_all_runs; }
bool post_last_all_passed(void) { return post_all_passed; }

// Las pruebas leen los sensores: si la placa está en bajo consumo (IDLE/FAULT) se despierta antes y
// se vuelve a dormir después
POST_Response post_request_all(void) {
    const bool wasSaving = board_power_save_active();
    if (wasSaving) board_power_save(false);
    POST_Response response = run_post_test(POST_ALL_SYSTEMS);
    if (wasSaving) board_power_save(true);
    print_post_result(response);
    board_post_result(response.status == "PASSED"); // LED verde o rojo fijo (main.cpp lo vuelve destello de vida)
    post_all_runs++;
    post_all_passed = response.status == "PASSED";
    return response;
}

POST_Response post_request_subsystem(uint8_t subsystem_code) {
    if (find_test(subsystem_code) == nullptr) {
        POST_Response response;
        response.code = subsystem_code;
        response.status = "ERROR";
        response.description = "Invalid subsystem code";
        DEBUG_PRINTLN("[POST] Invalid subsystem code request");
        return response;
    }
    const bool wasSaving = board_power_save_active();
    if (wasSaving) board_power_save(false);
    POST_Response response = run_post_test(subsystem_code);
    if (wasSaving) board_power_save(true);
    return response;
}

bool post_request_command(const String &command, POST_Response *response) {
    if (response == nullptr) {
        return false;
    }

    String cmd = command;
    cmd.trim();
    cmd.toUpperCase();

    if (cmd.startsWith("POST:")) {
        cmd = cmd.substring(5);
        cmd.trim();
    }

    uint8_t code = 0;
    if (!parse_subsystem_token(cmd, &code)) {
        response->code = 0;
        response->status = "ERROR";
        response->description = "Unsupported POST command";
        return false;
    }

    if (code == POST_ALL_SYSTEMS) {
        *response = post_request_all();
        return true;
    }

    *response = post_request_subsystem(code);
    return true;
}

// Comandos del firmware de prueba (solo POST y lo necesario para probar a mano). Lo de la aplicación (medición,
// anillo, LoRaWAN, energía, calibración del CH4) no existe aquí.
void post_process(void) {
    static String line;
    while (DEBUG_PORT.available()) {
        const char c = static_cast<char>(DEBUG_PORT.read());
        if (c != '\r' && c != '\n') {
            if (line.length() < 100) line += c;
            continue;
        }
        if (line.length() == 0) continue;

        String upper = line;
        upper.trim();
        upper.toUpperCase();
        String raw = line;
        raw.trim();
        line = "";

        if (upper.startsWith("RTC:SET")) { // RTC:SET AAAA-MM-DD HH:MM:SS (hora local)
            unsigned y = 0, mo = 0, d = 0, h = 0, mi = 0, s = 0;
            if (sscanf(raw.c_str() + 7, " %u-%u-%u %u:%u:%u", &y, &mo, &d, &h, &mi, &s) == 6) {
                board_rtc_set_local(y, mo, d, h, mi, s);
            } else {
                DEBUG_PRINTLN("[RTC] Formato: RTC:SET AAAA-MM-DD HH:MM:SS (hora local)");
            }
            continue;
        }
        if (upper == "VIVO?" || upper.startsWith("VIVO:")) {
            post_alive_command(upper);
            continue;
        }
        if (upper == "TABLA?" || upper.startsWith("TABLA:")) {
            post_table_command(upper);
            continue;
        }
        if (upper == "WDT:HANG") {
            DEBUG_PRINT("[WDT] Loop colgado a proposito: el WDT reinicia en ");
            DEBUG_PRINT(WDT_TIMEOUT_S);
            DEBUG_PRINTLN(" s (el LED deja de destellar)");
            DEBUG_PORT.flush();
            while (true) {
                __asm__ volatile("nop"); // sin alimentar el WDT; el asm evita que el compilador quite el lazo
            }
        }
        if (upper == "WDT:SW") {
            board_wdt_software_reset("prueba por serie (WDT:SW)");
        }
        if (upper.startsWith("POLICY:")) { // POLICY:<token>=CRIT|TOL|OFF|SIM  /  POLICY:LIST
            const String arg = upper.substring(7);
            const int eq = arg.indexOf('=');
            uint8_t code = 0;
            if (arg == "LIST") {
                post_print_policies();
            } else if (eq < 0 || !parse_subsystem_token(arg.substring(0, eq), &code) || code == POST_ALL_SYSTEMS) {
                DEBUG_PRINTLN("[POST] Formato: POLICY:<token>=CRIT|TOL|OFF|SIM (ej. POLICY:WIND=TOL) o POLICY:LIST");
            } else {
                const String p = arg.substring(eq + 1);
                PostPolicy value = POLICY_CRITICAL;
                bool ok = true;
                if (p == "CRIT") value = POLICY_CRITICAL;
                else if (p == "TOL") value = POLICY_TOLERATED;
                else if (p == "OFF") value = POLICY_DISABLED;
                else if (p == "SIM") value = POLICY_SIMULATE_FAIL;
                else ok = false;
                if (!ok) {
                    DEBUG_PRINTLN("[POST] Politica: CRIT, TOL, OFF o SIM");
                } else {
                    policies_init();
                    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
                        if (post_tests[i].code == code) policies[i] = value;
                    }
                    // TOL/OFF: su falla ya no cuenta (el LED de vida vuelve a verde si era la única)
                    if (value == POLICY_TOLERATED || value == POLICY_DISABLED) board_clear_fault(code);
                    DEBUG_PRINT("[POST] Politica de ");
                    DEBUG_PRINT(post_test_name(code));
                    DEBUG_PRINT(" = ");
                    DEBUG_PRINT(policy_text(value));
                    DEBUG_PRINTLN(" (hasta reiniciar; el valor fijo esta en post_tests[] de post_lib.cpp)");
                }
            }
            continue;
        }
        if (upper == "NODE?") {
            DEBUG_PRINT("[NODO] ");
            DEBUG_PRINT(board_node_name());
            DEBUG_PRINT(" | Bluetooth \"");
            DEBUG_PRINT(board_ble_name());
            DEBUG_PRINT("\"");
            DEBUG_PRINTLN(board_node_stored() ? " | guardado en la flash" : " | SIN GUARDAR (el del firmware, solo en RAM)");
            // Identificador único del chip: la estación web registra cada tarjeta física con él
            char id[40];
            snprintf(id, sizeof(id), "[NODO] DEVICEID %08lX%08lX", (unsigned long)NRF_FICR->DEVICEID[1],
                     (unsigned long)NRF_FICR->DEVICEID[0]);
            DEBUG_PRINTLN(id);
            continue;
        }
        // Asigna la identidad (credenciales de la tabla + número de nodo) ANTES de cargar el firmware principal: así
        // este, en su primer arranque, encuentra el nodo guardado y no graba ni usa el nodo con el que se compiló.
        // Es la única escritura de la flash de este firmware y solo ocurre con este comando (la estación lo envía).
        if (upper.startsWith("NODE:SET=")) {
            const long num = upper.substring(9).toInt();
            if (num < 1 || num > 999) DEBUG_PRINTLN("[NODO] Formato: NODE:SET=<numero 1-999> (ej. NODE:SET=5)");
            else board_node_set((uint16_t)num);
            continue;
        }
        if (upper.startsWith("NODE:ID=")) { // solo el número (collar con credenciales propias, fuera de la tabla)
            const long num = upper.substring(8).toInt();
            if (num < 1 || num > 999) DEBUG_PRINTLN("[NODO] Formato: NODE:ID=<numero 1-999>");
            else board_node_set_id((uint16_t)num);
            continue;
        }
        // Credenciales LoRaWAN en la flash (como en el firmware principal): LORA:CRED? / LORA:CRED:RESET /
        // LORA:CRED:SET <DevEUI 16 hex> <JoinEUI 16 hex> <AppKey 32 hex> (solo si no hay guardadas: primero RESET)
        if (upper == "LORA:CRED?") {
            board_lora_cred_status();
            continue;
        }
        if (upper == "LORA:CRED:RESET") {
            board_lora_cred_reset();
            continue;
        }
        if (upper.startsWith("LORA:CRED:SET ")) {
            String a = upper.substring(14);
            a.trim();
            const int s1 = a.indexOf(' '), s2 = s1 > 0 ? a.indexOf(' ', s1 + 1) : -1;
            String parts[3] = {s1 > 0 ? a.substring(0, s1) : "", s2 > 0 ? a.substring(s1 + 1, s2) : "",
                               s2 > 0 ? a.substring(s2 + 1) : ""};
            uint8_t bytes[32];
            const uint8_t sizes[3] = {8, 8, 16};
            bool ok = true;
            uint8_t at = 0;
            for (uint8_t k = 0; k < 3 && ok; k++) {
                parts[k].trim();
                ok = parts[k].length() == sizes[k] * 2u;
                for (uint8_t i = 0; ok && i < sizes[k]; i++) {
                    const char h = parts[k][2 * i], l = parts[k][2 * i + 1];
                    const int hv = h >= '0' && h <= '9' ? h - '0' : (h >= 'A' && h <= 'F' ? h - 'A' + 10 : -1);
                    const int lv = l >= '0' && l <= '9' ? l - '0' : (l >= 'A' && l <= 'F' ? l - 'A' + 10 : -1);
                    ok = hv >= 0 && lv >= 0;
                    bytes[at++] = (uint8_t)(hv * 16 + lv);
                }
            }
            if (!ok) DEBUG_PRINTLN("[LORA] Formato: LORA:CRED:SET <DevEUI 16 hex> <JoinEUI 16 hex> <AppKey 32 hex>");
            else board_lora_cred_set(bytes, bytes + 8, bytes + 16);
            continue;
        }
        if (upper == "BOOT?") {
            board_bootloader_info();
            continue;
        }
        if (upper == "DFU:OTA") {
            board_enter_ota_dfu("serie");
            continue;
        }
        if (upper == "DFU:UF2") {
            board_enter_uf2_dfu("serie");
            continue;
        }
        if (upper == "DFU:SERIAL") {
            board_enter_serial_dfu("serie");
            continue;
        }
        if (upper == "BAT:LOG" || upper.startsWith("BAT:LOG=")) { // lecturas del MAX17048 cada 250 ms (10 s por defecto)
            board_battery_log(upper.length() > 8 ? (uint32_t)upper.substring(8).toInt() : 10);
            continue;
        }
        if (upper == "FAN:ON" || upper == "FAN:OFF") {
            const bool on = upper == "FAN:ON";
            board_fan_set(on);
            DEBUG_PRINTLN(on ? "[FAN] Blower encendido" : "[FAN] Blower apagado");
            continue;
        }
        if (upper == "CH4:ON" || upper == "CH4:OFF") {
            const bool on = upper == "CH4:ON";
            if (!board_ch4_set(on)) {
                DEBUG_PRINTLN("[CH4] No se pudo cambiar el sensor CH4");
            } else if (on) {
                DEBUG_PRINT("[CH4] Sensor CH4 encendido, calefactor en ");
                DEBUG_PRINT(CH4_HEATER_MV);
                DEBUG_PRINTLN(" mV (CH4:OFF para apagarlo)");
            } else {
                DEBUG_PRINTLN("[CH4] Sensor CH4 apagado");
            }
            continue;
        }

        POST_Response response;
        if (!post_request_command(upper, &response)) {
            DEBUG_PRINT("[POST] Comando no soportado: ");
            DEBUG_PRINTLN(raw);
        }
    }
}

/* Ejecuta una prueba segun su politica e imprime su linea de resultado.
 * status: PASSED, FAILED (cuenta para el POST y va a FAULT), TOLERATED (fallo con politica TOL) o SKIPPED. */
static POST_Response run_one(const PostTest &test) {
    DEBUG_PRINT("[POST] ");
    DEBUG_PRINTLN(test.name);

    const PostPolicy policy = post_policy(test.code);
    HwStatus status;
    if (policy == POLICY_DISABLED) {
        DEBUG_PRINTLN("[POST]   deshabilitada en la tabla de politicas (OFF)");
        status = HW_SKIP;
    } else if (policy == POLICY_SIMULATE_FAIL) {
        DEBUG_PRINTLN("[POST]   FALLA EMULADA (politica SIM): no se corre la prueba");
        status = HW_FAIL;
    } else {
        status = test.run();
    }
    if (status == HW_FAIL) board_report_fault(test.code); // CRIT/SIM -> FAULT; TOL/OFF se ignoran ahí
    else board_clear_fault(test.code);                    // pasó (o no corre): su falla anterior se borra
    board_wdt_feed(); // el POST completo bloquea ~15 s: se alimenta el watchdog entre pruebas

    POST_Response response;
    response.code = test.code;
    response.status = status_text(status);
    if (status == HW_FAIL && policy == POLICY_TOLERATED) {
        response.status = "TOLERATED";
        DEBUG_PRINTLN("[POST]   falla tolerada (politica TOL): el POST sigue pasando");
    }
    response.description = String(test.name) + " " + response.status;
    print_post_result(response);

    PostRecord &rec = records[&test - post_tests];
    rec.last = status == HW_PASS ? LAST_PASS
             : status == HW_FAIL ? (policy == POLICY_TOLERATED ? LAST_TOL : LAST_FAIL)
                                 : LAST_SKIP;
    if (rec.runs < 0xFFFF) rec.runs++;
    if (status == HW_FAIL && rec.fails < 0xFFFF) rec.fails++;
    rec.at_ms = millis();
    return response;
}

/* ---------- Tabla de subsistemas (TABLA?) ----------
 * Estado mostrado (la mascara de fallas manda: tambien la llenan chequeos del loop, p. ej. el avance del RTC):
 *   FALLA      bit en board_fault_mask() -> LED rojo
 *   FALLA SIM  falla emulada (politica SIM)
 *   TOLERADA   fallo con politica TOL: avisa pero no cuenta
 *   OK         la ultima corrida paso
 *   OMITIDA    no corrio (politica OFF, o falta su bus/dependencia)
 *   SIN CORRER nunca se probo desde el arranque */
enum TableState : uint8_t { TS_FAIL, TS_FAIL_SIM, TS_TOL, TS_OK, TS_SKIP, TS_NONE };

static TableState table_state(uint8_t i, uint32_t mask) {
    const PostTest &t = post_tests[i];
    const PostPolicy p = policies[i];
    if (t.code < 32 && (mask & (1UL << t.code))) return p == POLICY_SIMULATE_FAIL ? TS_FAIL_SIM : TS_FAIL;
    switch (records[i].last) {
        case LAST_TOL:  return TS_TOL;
        case LAST_PASS: return TS_OK;
        case LAST_SKIP: return TS_SKIP;
        case LAST_FAIL: return TS_SKIP; // fallo pero su politica cambio a TOL/OFF despues
        default:        return TS_NONE;
    }
}

static const char *table_state_text(TableState s) {
    switch (s) {
        case TS_FAIL:     return "FALLA";
        case TS_FAIL_SIM: return "FALLA SIM";
        case TS_TOL:      return "TOLERADA";
        case TS_OK:       return "OK";
        case TS_SKIP:     return "OMITIDA";
        default:          return "SIN CORRER";
    }
}

static void format_age(uint32_t at_ms, bool ran, char *out, size_t size) {
    if (!ran) {
        snprintf(out, size, "-");
        return;
    }
    const uint32_t s = (millis() - at_ms) / 1000UL;
    if (s < 60UL) snprintf(out, size, "%lu s", (unsigned long)s);
    else if (s < 3600UL) snprintf(out, size, "%lu min", (unsigned long)(s / 60UL));
    else snprintf(out, size, "%lu h", (unsigned long)(s / 3600UL));
}

static const char TABLE_RULE[] =
    "+------+----------------------------------+------+------------+---------+---------+";

uint8_t post_print_table(void) {
    policies_init();
    const uint32_t mask = board_fault_mask();
    static char row[112]; // static: la pila del loop es de 4 KB
    char age[12], runs[12];

    DEBUG_PRINTLN(TABLE_RULE);
    snprintf(row, sizeof(row), "| %-4s | %-32s | %-4s | %-10s | %7s | %7s |", "Cod", "Subsistema", "Pol.", "Estado",
             "Fallas", "Probado");
    DEBUG_PRINTLN(row);
    DEBUG_PRINTLN(TABLE_RULE);
    uint8_t failed = 0, tolerated = 0, ok = 0, pending = 0;
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        if (i == 0 || strcmp(post_tests[i].group, post_tests[i - 1].group) != 0) {
            snprintf(row, sizeof(row), "| %-79s |", post_tests[i].group);
            DEBUG_PRINTLN(row);
        }
        const TableState st = table_state(i, mask);
        const PostRecord &rec = records[i];
        if (st == TS_FAIL || st == TS_FAIL_SIM) failed++;
        else if (st == TS_TOL) tolerated++;
        else if (st == TS_OK) ok++;
        else pending++;
        snprintf(runs, sizeof(runs), "%u/%u", (unsigned)rec.fails, (unsigned)rec.runs);
        format_age(rec.at_ms, rec.runs > 0, age, sizeof(age));
        snprintf(row, sizeof(row), "| 0x%02X | %-32.32s | %-4s | %-10s | %7s | %7s |%s", post_tests[i].code,
                 post_tests[i].name, policy_text(policies[i]), table_state_text(st), runs, age,
                 st == TS_FAIL ? " <-- revisar" : "");
        DEBUG_PRINTLN(row);
    }
    DEBUG_PRINTLN(TABLE_RULE);
    snprintf(row, sizeof(row), "  %u OK | %u con falla | %u tolerada(s) | %u sin probar/omitidas  (Fallas = fallidas/total)",
             (unsigned)ok, (unsigned)failed, (unsigned)tolerated, (unsigned)pending);
    DEBUG_PRINTLN(row);

    if (failed + tolerated == 0) {
        DEBUG_PRINTLN(pending ? "  QUE REVISAR: nada fallo; hay pruebas sin correr (POST:ALL las corre todas)"
                              : "  QUE REVISAR: nada, todos los subsistemas pasaron");
        return 0;
    }
    // En el orden de la tabla: los de arriba (buses, expansor, rieles) suelen causar las fallas de abajo
    DEBUG_PRINTLN("  QUE REVISAR (en orden: una falla de arriba suele causar las de abajo; arreglar la primera y repetir):");
    uint8_t n = 0;
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        const TableState st = table_state(i, mask);
        if (st != TS_FAIL && st != TS_FAIL_SIM && st != TS_TOL) continue;
        const PostTest &t = post_tests[i];
        snprintf(row, sizeof(row), "  %u. %s [POST:%s]%s", (unsigned)++n, t.name, t.token,
                 st == TS_TOL ? " (tolerada)" : st == TS_FAIL_SIM ? " (emulada: POLICY:" : "");
        DEBUG_PRINT(row);
        if (st == TS_FAIL_SIM) {
            DEBUG_PRINT(t.token);
            DEBUG_PRINT("=CRIT para quitarla)");
        }
        DEBUG_PRINTLN("");
        DEBUG_PRINT("     -> ");
        DEBUG_PRINTLN(st == TS_FAIL_SIM ? "nada en la placa: la falla es emulada" : t.check);
        if (st == TS_FAIL && records[i].last != LAST_FAIL) {
            DEBUG_PRINTLN("     -> detectada en el loop (no en la ultima corrida del POST)");
        }
    }
    return failed;
}

/* Función principal que ejecuta las pruebas POST */
POST_Response run_post_test(uint8_t test_code) {
    POST_Response response;

    DEBUG_PRINTLN("\n========================================");
    DEBUG_PRINTLN("        POWER AND SELF TEST (POST)       ");
    DEBUG_PRINTLN("========================================\n");

    if (test_code != POST_ALL_SYSTEMS) {
        const PostTest *test = find_test(test_code);
        if (test == nullptr) {
            response.code = test_code;
            response.status = "ERROR";
            response.description = "Invalid test code";
            DEBUG_PRINTLN("[ERROR] Invalid POST test code");
            return response;
        }
        return run_one(*test);
    }

    DEBUG_PRINTLN("Running FULL SYSTEM TEST...\n");

    uint8_t passed = 0, failed = 0, tolerated = 0, skipped = 0;
    const char *first_failure = nullptr;
    for (uint8_t i = 0; i < POST_TEST_COUNT; i++) {
        if (i == 0 || strcmp(post_tests[i].group, post_tests[i - 1].group) != 0) {
            DEBUG_PRINT("\n--- ");
            DEBUG_PRINT(post_tests[i].group);
            DEBUG_PRINTLN(" ---");
        }
        const POST_Response r = run_one(post_tests[i]);
        if (r.status == "PASSED") {
            passed++;
        } else if (r.status == "FAILED") {
            failed++;
            if (first_failure == nullptr) first_failure = post_tests[i].name;
        } else if (r.status == "TOLERATED") {
            tolerated++;
        } else {
            skipped++;
        }
    }

    DEBUG_PRINTLN("");
    response.code = POST_ALL_SYSTEMS;
    response.status = failed ? "FAILED" : "PASSED"; // las toleradas no hacen fallar el POST
    response.description = String(passed) + " PASS, " + failed + " FAIL, " + tolerated + " TOL, " + skipped + " SKIP";
    if (first_failure != nullptr) {
        response.description += String(" | primera falla: ") + first_failure;
    }
    return response;
}

/* Formato: codigo en 4 bits (b3 b2 b1 b0), nombre y estado. El POST completo imprime un resumen. */
String get_formatted_post_result(POST_Response response) {
    if (response.code == POST_ALL_SYSTEMS) {
        return "[POST] ALL SYSTEMS - " + response.status + " (" + response.description + ")";
    }

    const PostTest *test = find_test(response.code);
    String result = "";
    for (int8_t bit = 3; bit >= 0; bit--) {
        result += ((response.code >> bit) & 0x01) ? "1\t" : "0\t";
    }
    result += test ? test->name : "UNKNOWN";
    result += " - " + response.status;
    return result;
}

void print_post_result(POST_Response response) {
    DEBUG_PRINTLN(get_formatted_post_result(response));
}
