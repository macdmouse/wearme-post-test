#include "fs3000.h"

// Calibración del FS3000-1015 (hoja de datos Renesas): cuentas y m/s
static const uint16_t RAW_1015[] = {409, 1203, 1597, 1908, 2187, 2400, 2629, 2801, 3006, 3178, 3309, 3563, 3686};
static const float MPS_1015[] = {0.0f, 1.07f, 2.01f, 3.00f, 3.97f, 4.96f, 5.98f, 6.99f, 7.94f, 8.97f, 9.95f, 11.96f, 13.85f};
static const uint8_t POINTS = sizeof(RAW_1015) / sizeof(RAW_1015[0]);

float Fs3000::toMetersPerSecond(uint16_t raw) {
  if (raw <= RAW_1015[0]) return 0.0f;
  if (raw >= RAW_1015[POINTS - 1]) return MPS_1015[POINTS - 1];
  for (uint8_t i = 1; i < POINTS; i++) {
    if (raw <= RAW_1015[i]) {
      const float f = (float)(raw - RAW_1015[i - 1]) / (float)(RAW_1015[i] - RAW_1015[i - 1]);
      return MPS_1015[i - 1] + f * (MPS_1015[i] - MPS_1015[i - 1]);
    }
  }
  return MPS_1015[POINTS - 1];
}

bool Fs3000::read(TwoWire &wire, uint16_t &raw, float &metersPerSecond) {
  if (wire.requestFrom(I2C_ADDR, (uint8_t)5) != 5) {
    status_ = NO_RESPONSE;
    return false;
  }
  uint8_t b[5];
  uint8_t sum = 0, any = 0;
  for (uint8_t i = 0; i < 5; i++) {
    b[i] = wire.read();
    sum += b[i];
    any |= b[i];
  }
  if (!any) { // todo en 0: el checksum da 0 pero no es una lectura (sin flujo el FS3000 da ~409 cuentas)
    status_ = NOT_READY;
    return false;
  }
  if (sum != 0) { // checksum: los 5 bytes suman 0 (mod 256)
    status_ = BAD_CHECKSUM;
    return false;
  }
  status_ = OK;
  raw = (uint16_t)(((b[1] & 0x0F) << 8) | b[2]);
  metersPerSecond = toMetersPerSecond(raw);
  return true;
}
