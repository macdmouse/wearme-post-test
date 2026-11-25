// ble_lib.cpp
// Puerto serie por BLE (ver ble_lib.h).

#include "ble_lib.h"
#include <bluefruit.h>

static BLEUart bleuart;
static bool started = false;
static bool was_subscribed = false;
static char rx_buf[BLE_RX_MAX];
static size_t rx_len = 0;
static uint32_t rx_ms = 0; // último byte recibido

bool ble_begin(const char *name) {
  Bluefruit.autoConnLed(false);                 // LED_BLUE (P0.31) es un LED de estado de la placa
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX); // MTU de hasta 247 B si el teléfono lo acepta
  if (!Bluefruit.begin()) return false;
  Bluefruit.setTxPower(BLE_TX_POWER_DBM);
  Bluefruit.setName(name);
  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // unidades de 0.625 ms: 20 ms los primeros 30 s, luego 152.5 ms
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0); // sin límite de tiempo
  started = true;
  return true;
}

bool ble_started(void) {
  return started;
}

bool ble_connected(void) {
  return started && Bluefruit.connected();
}

bool ble_subscribed(void) {
  return ble_connected() && bleuart.notifyEnabled();
}

bool ble_take_subscribed(void) {
  const bool now = ble_subscribed();
  const bool edge = now && !was_subscribed;
  was_subscribed = now;
  return edge;
}

bool ble_read_line(char *buf, size_t size) {
  if (!started || size == 0) return false;
  bool eol = false;
  while (!eol && bleuart.available()) {
    const char c = (char)bleuart.read();
    rx_ms = millis();
    if (c == '\r' || c == '\n') {
      eol = rx_len > 0; // un fin de línea suelto (el '\n' de un "\r\n") se descarta
      continue;
    }
    if (rx_len < sizeof(rx_buf) - 1) rx_buf[rx_len++] = c;
  }
  if (rx_len == 0) return false;
  if (!eol && millis() - rx_ms < BLE_RX_IDLE_MS) return false; // pueden faltar paquetes del mismo comando

  const size_t n = rx_len < size - 1 ? rx_len : size - 1;
  memcpy(buf, rx_buf, n);
  buf[n] = '\0';
  rx_len = 0;
  return true;
}

bool ble_send_line(const char *text) {
  if (!ble_subscribed()) return false;
  const size_t len = strlen(text);
  if (bleuart.write((const uint8_t *)text, len) != len) return false;
  return bleuart.write((uint8_t)'\n') == 1;
}
