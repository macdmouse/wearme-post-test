#pragma once

// =====================================================================================
// lora_frames — tramas LoRaWAN de la medición y filas de PENDIENTES.csv. Big endian (byte alto primero),
// escala fija y orden fijo. Si se cambia un campo, actualizar tools/ttn_formatter.js.
//
// ---- FPort 10: trama principal v2 (30 B, CONFIRMADA) ----
// Fuente: especificación de la trama del proyecto WearMe (tabla "Trama LoRaWAN", 2026-10-16) = bytes 0-21; la v2
// (2026-10-21) agrega al final la hora de captura y los voltajes. Data_length dice la versión (22 = v1, 30 = v2) y
// un decodificador de la v1 sigue leyendo bien los primeros 22 B.
//   Byte   Campo        Tipo    Escala / significado                                     Sin dato
//   0      ID           uint8   ID del equipo (FRAME_DEVICE_ID = 0x04)                   -
//   1      Data_length  uint8   30 (v2; 22 en la v1)                                     -
//   2-3    CH4          uint16  gramos totales de la medición (ch4TotalG), redondeados,  0
//                               saturados en 65535
//   4-5    Temp         int16   °C × 100                                                 0x8000
//   6-7    Hum          uint16  %HR × 100                                                0xFFFF
//   8-9    Press        uint16  hPa × 10                                                 0xFFFF
//   10-13  Lat          int32   grados × 1e6 (con signo)                                 0x80000000
//   14-17  Long         int32   grados × 1e6 (con signo)                                 0x80000000
//   18-19  Battery      uint16  % SOC entero                                             0xFFFF
//   20-21  RSSI         int16   dBm del último downlink                                  0 (sin downlink)
//   22-25  meas_time    uint32  hora UTC (unix) de la captura: la plataforma (Influx) la  0 (collar sin hora)
//                               usa como hora del dato; sin ella, la hora de recepción
//   26-27  bat_v        uint16  V × 10000 de la celda (MAX17048)                         0xFFFF
//   28-29  ch4_v        uint16  V × 10000 del sensor CH4 (AIN1, 0-2.048 V), promedio de   0xFFFF
//                               las muestras no saturadas (todas saturadas = sin dato)
// La especificación no define "sin dato" salvo para RSSI; los valores de la última columna son de este
// firmware y el formateador de TTN los muestra como null.
//
// ---- FPort 11: trama complementaria (29 B, no confirmada) ----
// Campos que no caben en la principal. Versión en el byte 0 (v1 = 27 B, sin gps_age_min).
//   Byte   Campo           Tipo    Escala / significado                                  Sin dato
//   0      version         uint8   FRAME_EXTRA_VERSION (1)                               -
//   1-2    ch4_ppm         uint16  ppm (compensate_ppm: 0 o 2000-30000), promedio         0xFFFF
//   3-6    ch4_rate_g_day  uint32  g/día × 1000 (mg/día), promedio                       0xFFFFFFFF
//   7-10   ch4_mg          uint32  mg totales de la medición (el CH4 de FPort 10 en g     0xFFFFFFFF
//                                  enteros pierde todo lo menor a 0.5 g)
//   11     ch4_event       uint8   1 = hubo al menos un evento en la medición, 0 = no      0xFF
//   12-13  h2_v            uint16  V × 10000 (AIN0, 0-2.048 V)                            0xFFFF
//   14-17  h2_ppm          int32   ppm × 100 (calibración lineal H2_PPM_M/B)              0x80000000
//   18-19  wind_right_ms   int16   m/s × 100 (FS3000 canal 0)                             0x8000
//   20-21  wind_left_ms    int16   m/s × 100 (FS3000 canal 1)                             0x8000
//   22     energy_mode     uint8   0 SURVIVAL, 1 CORE, 2 RICH                             0xFF
//   23-24  fan_rpm         uint16  rpm                                                    0xFFFF
//   25-26  ch4_v           uint16  V × 10000 (AIN1, 0-2.048 V)                            0xFFFF
//   27-28  gps_age_min     uint16  edad de lat/lon de FPort 10 en minutos (0 = fix de      0xFFFF
//                                  esta medición; > 0 = último fix guardado), satura 65534
//
// ---- FPort 12: trama principal reenviada desde PENDIENTES.csv (30 B, CONFIRMADA) ----
// Los mismos 30 B de FPort 10 (la hora de captura va adentro); el puerto solo dice que es un reenvío.
// (v1: 26 B = meas_time uint32 + trama de 22 B)
//
// ---- FPort 20: medición de OTRO collar que llegó por el anillo Meshtastic (42 B, CONFIRMADA) ----
// El collar que la sube es solo el puente: el dato es del DevEUI de origen.
//   0      version         uint8   FRAME_RELAY_VERSION (2)
//   1-8    origin_deveui   8 B     DevEUI del collar que midió
//   9      hops            uint8   saltos por la mesh hasta el puente
//   10     mesh_rssi       int8    dBm del paquete recibido por el puente
//   11     mesh_snr        int8    dB
//   12-41  trama principal (los mismos 30 B de FPort 10, del collar de origen, con su hora de captura)
// (v1: 38 B = versión, DevEUI, meas_time uint32, saltos, RSSI, SNR + trama de 22 B)
// Deduplicación en la plataforma: misma medición = mismo DevEUI de origen + misma meas_time (todas las tramas v2 la
// traen exacta; solo en la v1 FPort 10 venía sin hora: la de recepción, tratar igual si difiere menos de 120 s).
//
// ---- FPort 21 (DOWNLINK a un puente): comando para un collar sin cobertura, por el anillo ----
//   0      version         uint8   FRAME_RING_CMD_VERSION (1)
//   1-8    target_deveui   8 B     collar destino (si es el mismo puente, lo ejecuta él)
//   9-12   id              uint32  id del comando (lo elige la plataforma; el destino no repite un id ya ejecutado)
//   13-    comando         1-12 B  los mismos bytes de un downlink normal: 01 MEASURE, 02 POST, 03 SS SS, 04 MM MM
// El puente lo difunde en las ventanas siguientes (hasta 3) hasta oír COMANDO OK.
//
// ---- FPort 22: COMANDO OK (13 B, no confirmada): el collar destino ejecutó el comando ----
//   0      version         uint8   1
//   1-8    target_deveui   8 B
//   9-12   id              uint32
// =====================================================================================

#include <Arduino.h>

#define FRAME_MAIN_FPORT     10
#define FRAME_MAIN_LEN       30
#define FRAME_DEVICE_ID      0x04
#define FRAME_EXTRA_FPORT    11
#define FRAME_EXTRA_LEN      29
#define FRAME_EXTRA_VERSION  2
#define FRAME_PENDING_FPORT  12 // los mismos FRAME_MAIN_LEN bytes
#define FRAME_RELAY_FPORT    20
#define FRAME_RELAY_LEN      (12 + FRAME_MAIN_LEN)
#define FRAME_RELAY_VERSION  2
#define FRAME_RING_CMD_FPORT 21 // downlink a un puente: comando para otro collar por el anillo
#define FRAME_RING_CMD_VERSION 1
#define FRAME_RING_CMD_HDR   13 // versión + DevEUI destino + id
#define FRAME_CMD_ACK_FPORT  22 // uplink del puente: COMANDO OK del collar destino
#define FRAME_CMD_ACK_LEN    13

// Valores enteros exactos de la trama principal (los mismos que se guardan en PENDIENTES.csv)
struct MainFrame {
  uint8_t id;
  uint16_t ch4G;
  int16_t tempX100;
  uint16_t humX100;
  uint16_t pressX10;
  int32_t latE6;
  int32_t lonE6;
  uint16_t bat;
  int16_t rssi;
  uint32_t time;       // hora UTC de captura (0 = sin hora); en PENDIENTES.csv sale de las columnas fecha y hora
  uint16_t batVX1e4;   // V × 10000
  uint16_t ch4VX1e4;
};

// Valores en unidades físicas para la trama complementaria (NAN = sin dato)
struct ExtraValues {
  float ch4Ppm, ch4RateGDay, ch4TotalG, ch4Event;
  float h2V, h2Ppm, windRight, windLeft, energyMode, fanRpm, ch4V;
  float gpsAgeS; // edad del fix en segundos (se envía en minutos)
};

// Valores físicos -> enteros de la trama principal (redondeo, saturación y "sin dato")
MainFrame frame_main_from(float ch4TotalG, float tempC, float humPct, float pressHpa, float lat, float lon,
                          float batSoc, float rssiDbm, uint32_t unixTime, float batV, float ch4V);
void frame_main_encode(const MainFrame &f, uint8_t out[FRAME_MAIN_LEN]);
uint8_t frame_extra_encode(const ExtraValues &v, uint8_t out[FRAME_EXTRA_LEN]); // devuelve FRAME_EXTRA_LEN
void frame_cmd_ack_encode(const uint8_t devEui[8], uint32_t id, uint8_t out[FRAME_CMD_ACK_LEN]);
void frame_relay_encode(const uint8_t originDevEui[8], uint8_t hops, int8_t rssi, int8_t snr,
                        const uint8_t main[FRAME_MAIN_LEN], uint8_t out[FRAME_RELAY_LEN]);

// ---- PENDIENTES.csv: una fila por trama principal no confirmada (enteros crudos) ----
// Las filas de la v1 (sin bat_v ni ch4_v) se siguen leyendo: esos campos quedan sin dato.
#define PENDING_FILE    "PENDIENTES"
#define PENDING_HEADER  "fecha;hora;id;ch4_g;temp_x100;hum_x100;press_x10;lat_e6;lon_e6;bat;rssi;bat_v_x1e4;ch4_v_x1e4;enviado"
// Fila con enviado = 0 (la última columna es un solo carácter: se marca "1" en su lugar al recibir el ACK)
void pending_row_format(const char *date, const char *time, const MainFrame &f, char *out, size_t size);
// false si la fila no tiene el formato (p. ej. el encabezado)
bool pending_row_parse(const char *row, MainFrame &f, bool &sent);
