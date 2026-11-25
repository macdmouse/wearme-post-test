#include "lora_frames.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

// Con -Ofast isnan() no sirve (siempre false): "sin dato" se reconoce por los bits del float
static bool has_data(float v) {
  uint32_t bits;
  memcpy(&bits, &v, sizeof(bits));
  return (bits & 0x7F800000u) != 0x7F800000u;
}

static int32_t to_int(float v, double scale, double lo, double hi, int32_t noData) {
  if (!has_data(v)) return noData;
  double x = round((double)v * scale);
  if (x < lo) x = lo;
  if (x > hi) x = hi;
  return (int32_t)x;
}

static uint32_t to_uint(float v, double scale, double hi, uint32_t noData) {
  if (!has_data(v)) return noData;
  double x = round((double)v * scale);
  if (x < 0) x = 0;
  if (x > hi) x = hi;
  return (uint32_t)x;
}

static void put16(uint8_t *&p, uint16_t v) {
  *p++ = (uint8_t)(v >> 8);
  *p++ = (uint8_t)v;
}

static void put32(uint8_t *&p, uint32_t v) {
  *p++ = (uint8_t)(v >> 24);
  *p++ = (uint8_t)(v >> 16);
  *p++ = (uint8_t)(v >> 8);
  *p++ = (uint8_t)v;
}

MainFrame frame_main_from(float ch4TotalG, float tempC, float humPct, float pressHpa, float lat, float lon,
                          float batSoc, float rssiDbm, uint32_t unixTime, float batV, float ch4V) {
  MainFrame f;
  f.id = FRAME_DEVICE_ID;
  f.ch4G = (uint16_t)to_uint(ch4TotalG, 1.0, 65535.0, 0);                          // g, saturado en 65535
  f.tempX100 = (int16_t)to_int(tempC, 100.0, -32767.0, 32767.0, INT16_MIN);       // °C × 100
  f.humX100 = (uint16_t)to_uint(humPct, 100.0, 65534.0, 0xFFFF);                  // %HR × 100
  f.pressX10 = (uint16_t)to_uint(pressHpa, 10.0, 65534.0, 0xFFFF);                // hPa × 10
  f.latE6 = to_int(lat, 1e6, -2147483647.0, 2147483647.0, INT32_MIN);             // grados × 1e6
  f.lonE6 = to_int(lon, 1e6, -2147483647.0, 2147483647.0, INT32_MIN);
  f.bat = (uint16_t)to_uint(batSoc, 1.0, 100.0, 0xFFFF);                          // % SOC entero (el MAX17048 da > 100)
  f.rssi = (int16_t)to_int(rssiDbm, 1.0, -32767.0, 32767.0, 0);                   // dBm, 0 = sin downlink
  f.time = unixTime;                                                              // UTC, 0 = sin hora
  f.batVX1e4 = (uint16_t)to_uint(batV, 10000.0, 65534.0, 0xFFFF);                 // V × 10000
  f.ch4VX1e4 = (uint16_t)to_uint(ch4V, 10000.0, 65534.0, 0xFFFF);
  return f;
}

void frame_main_encode(const MainFrame &f, uint8_t out[FRAME_MAIN_LEN]) {
  uint8_t *p = out;
  *p++ = f.id;
  *p++ = FRAME_MAIN_LEN;
  put16(p, f.ch4G);
  put16(p, (uint16_t)f.tempX100);
  put16(p, f.humX100);
  put16(p, f.pressX10);
  put32(p, (uint32_t)f.latE6);
  put32(p, (uint32_t)f.lonE6);
  put16(p, f.bat);
  put16(p, (uint16_t)f.rssi);
  put32(p, f.time);
  put16(p, f.batVX1e4);
  put16(p, f.ch4VX1e4);
}

void frame_cmd_ack_encode(const uint8_t devEui[8], uint32_t id, uint8_t out[FRAME_CMD_ACK_LEN]) {
  uint8_t *p = out;
  *p++ = 1;
  memcpy(p, devEui, 8);
  p += 8;
  put32(p, id);
}

void frame_relay_encode(const uint8_t originDevEui[8], uint8_t hops, int8_t rssi, int8_t snr,
                        const uint8_t main[FRAME_MAIN_LEN], uint8_t out[FRAME_RELAY_LEN]) {
  uint8_t *p = out;
  *p++ = FRAME_RELAY_VERSION;
  memcpy(p, originDevEui, 8);
  p += 8;
  *p++ = hops;
  *p++ = (uint8_t)rssi;
  *p++ = (uint8_t)snr;
  memcpy(p, main, FRAME_MAIN_LEN);
}

uint8_t frame_extra_encode(const ExtraValues &v, uint8_t out[FRAME_EXTRA_LEN]) {
  uint8_t *p = out;
  *p++ = FRAME_EXTRA_VERSION;
  put16(p, (uint16_t)to_uint(v.ch4Ppm, 1.0, 65534.0, 0xFFFF));
  put32(p, to_uint(v.ch4RateGDay, 1000.0, 4294967294.0, 0xFFFFFFFFu));
  put32(p, to_uint(v.ch4TotalG, 1000.0, 4294967294.0, 0xFFFFFFFFu));
  *p++ = (uint8_t)to_uint(v.ch4Event, 1.0, 1.0, 0xFF);
  put16(p, (uint16_t)to_uint(v.h2V, 10000.0, 65534.0, 0xFFFF));
  put32(p, (uint32_t)to_int(v.h2Ppm, 100.0, -2147483647.0, 2147483647.0, INT32_MIN));
  put16(p, (uint16_t)to_int(v.windRight, 100.0, -32767.0, 32767.0, INT16_MIN));
  put16(p, (uint16_t)to_int(v.windLeft, 100.0, -32767.0, 32767.0, INT16_MIN));
  *p++ = (uint8_t)to_uint(v.energyMode, 1.0, 254.0, 0xFF);
  put16(p, (uint16_t)to_uint(v.fanRpm, 1.0, 65534.0, 0xFFFF));
  put16(p, (uint16_t)to_uint(v.ch4V, 10000.0, 65534.0, 0xFFFF));
  put16(p, (uint16_t)to_uint(v.gpsAgeS, 1.0 / 60.0, 65534.0, 0xFFFF));
  return (uint8_t)(p - out);
}

void pending_row_format(const char *date, const char *time, const MainFrame &f, char *out, size_t size) {
  snprintf(out, size, "%s;%s;%u;%u;%d;%u;%u;%ld;%ld;%u;%d;%u;%u;0", date, time, f.id, f.ch4G, f.tempX100, f.humX100,
           f.pressX10, (long)f.latE6, (long)f.lonE6, f.bat, f.rssi, f.batVX1e4, f.ch4VX1e4);
}

bool pending_row_parse(const char *row, MainFrame &f, bool &sent) {
  // fecha;hora;id;ch4_g;temp_x100;hum_x100;press_x10;lat_e6;lon_e6;bat;rssi[;bat_v_x1e4;ch4_v_x1e4];enviado
  const char *p = row;
  for (uint8_t skip = 0; skip < 2; skip++) { // fecha y hora (pueden ir vacías si no había hora)
    p = strchr(p, ';');
    if (p == nullptr) return false;
    p++;
  }
  long v[11]; // id, ch4_g, temp, hum, press, lat, lon, bat, rssi, bat_v, ch4_v
  uint8_t n = 0;
  while (n < 11) {
    char *end = nullptr;
    const long x = strtol(p, &end, 10);
    if (end == p) return false; // campo vacío o no numérico (p. ej. el encabezado)
    if (*end != ';') break;     // el último campo: "enviado"
    v[n++] = x;
    p = end + 1;
  }
  if (n == 9) v[9] = v[10] = 0xFFFF; // fila de la v1: sin voltajes
  else if (n != 11) return false;
  if (*p != '0' && *p != '1') return false;
  f.id = (uint8_t)v[0];
  f.ch4G = (uint16_t)v[1];
  f.tempX100 = (int16_t)v[2];
  f.humX100 = (uint16_t)v[3];
  f.pressX10 = (uint16_t)v[4];
  f.latE6 = (int32_t)v[5];
  f.lonE6 = (int32_t)v[6];
  f.bat = (uint16_t)v[7];
  f.rssi = (int16_t)v[8];
  f.time = 0; // la pone quien lee la fila (columnas fecha y hora, en hora local)
  f.batVX1e4 = (uint16_t)v[9];
  f.ch4VX1e4 = (uint16_t)v[10];
  sent = *p == '1';
  return true;
}
