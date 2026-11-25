// ble_lib.h
// Puerto serie por BLE (Nordic UART Service de Bluefruit, SoftDevice S140) para la app del Wearme: la app
// manda comandos de texto y el nodo le manda líneas de texto (JSON o CSV) terminadas en '\n'.
//
// Orden de arranque: ble_begin() va ANTES de iniciar LoRa Basics Modem (lora_lib). LBM decide al arrancar si
// usa el RNG y el HFCLK directamente o a través del SoftDevice (sd_rand_*, sd_clock_*); si arranca primero,
// el SoftDevice los toma después y LBM queda usando periféricos que ya no le pertenecen.
// Con el SoftDevice activo ninguna interrupción de la aplicación puede usar las prioridades 0, 1 y 4.
//
// El BLE UART no tiene autenticación (cualquiera en alcance se conecta): no enviar secretos (APPKEY) ni
// aceptar comandos que cuelguen o reinicien el nodo.

#ifndef BLE_LIB_H
#define BLE_LIB_H

#include <Arduino.h>

#define BLE_TX_POWER_DBM 4    // potencia de advertising y conexión
#define BLE_RX_IDLE_MS   100  // sin fin de línea: el comando termina tras este tiempo sin bytes nuevos
#define BLE_RX_MAX       64   // comando más largo aceptado

bool ble_begin(const char *name); // SoftDevice + UART + advertising sin límite; false si el SoftDevice no arrancó
bool ble_started(void);
bool ble_connected(void);
bool ble_subscribed(void);      // la app activó las notificaciones del UART (puede recibir líneas)
bool ble_take_subscribed(void); // true una sola vez cada vez que la app activa las notificaciones
// Comando completo de la app: termina en '\r' o '\n', o tras BLE_RX_IDLE_MS sin bytes (la app puede no mandar
// fin de línea, y un comando más largo que el MTU llega en varios paquetes). No bloquea.
bool ble_read_line(char *buf, size_t size);
bool ble_send_line(const char *text); // agrega '\n'; false si nadie está suscrito

#endif
