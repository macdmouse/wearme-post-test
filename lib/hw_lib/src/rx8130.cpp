#include "rx8130.h"
#include <string.h>

static bool bcdValid(uint8_t v) { return (v & 0x0F) <= 9 && (v >> 4) <= 9; }
static uint8_t fromBcd(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t toBcd(uint8_t v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

bool Rx8130::begin(TwoWire &wire, const DateTime &fallback) {
  wire_ = &wire;
  present_ = false;
  configFixed_ = false;
  uint8_t flags = 0;
  if (!readFlags(flags)) return false;
  bootFlags_ = flags;
  timeLost_ = (flags & FLAG_VLF) != 0;
  present_ = true;

  if (timeLost_) {
    // Inicialización "solo reloj" del manual (sección 18.4)
    if (!writeReg(REG_OFFSET, 0x00)) return false;      // sin corrección digital (DTE = 0)
    if (!writeReg(REG_EXT, EXT_CONFIG)) return false;   // FOUT apagado, temporizador detenido
    if (!writeReg(REG_FLAG, 0x00)) return false;        // limpia banderas
    if (!writeReg(REG_CTRL0, CTRL0_STOP)) return false; // TEST = 0, sin interrupciones, STOP = 1
    if (!writeReg(REG_CTRL1, CTRL1_CONFIG)) return false;
    configFixed_ = true;
    return setTime(fallback) && clearVlf();
  }

  // Hora válida: solo se asegura la configuración de la placa (no cambia la hora)
  uint8_t ext = 0, ctrl0 = 0, ctrl1 = 0;
  if (!readReg(REG_EXT, ext) || !readReg(REG_CTRL0, ctrl0) || !readReg(REG_CTRL1, ctrl1)) return false;
  if ((ext & 0xD0) != (EXT_CONFIG & 0xD0)) { // FSEL1/FSEL0 y TE
    configFixed_ = true;
    if (!writeReg(REG_EXT, EXT_CONFIG)) return false;
  }
  if (ctrl0 != 0x00) { // TEST, STOP o interrupciones encendidas
    configFixed_ = true;
    if (!writeReg(REG_CTRL0, 0x00)) return false;
  }
  if (ctrl1 != CTRL1_CONFIG) {
    configFixed_ = true;
    if (!writeReg(REG_CTRL1, CTRL1_CONFIG)) return false;
  }
  return true;
}

bool Rx8130::clearVlf() {
  for (uint8_t i = 0; i < 50; i++) { // 50 x 20 ms = 1 s (arranque del oscilador: 0.19 s típ., 1 s máx.)
    uint8_t flags = 0;
    if (!writeReg(REG_FLAG, 0x00) || !readFlags(flags)) return false;
    if ((flags & FLAG_VLF) == 0) return true;
    delay(20);
  }
  return false;
}

bool Rx8130::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) return false;
  if (wire_->requestFrom(I2C_ADDR, len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = wire_->read();
  return true;
}

bool Rx8130::writeRegs(uint8_t reg, const uint8_t *buf, uint8_t len) {
  if (wire_ == nullptr) return false;
  wire_->beginTransmission(I2C_ADDR);
  wire_->write(reg);
  wire_->write(buf, len);
  return wire_->endTransmission() == 0;
}

bool Rx8130::readFlags(uint8_t &flags) { return readRegs(REG_FLAG, &flags, 1); }

bool Rx8130::readTime(DateTime &t) {
  uint8_t r[7];
  if (!readRegs(REG_SEC, r, sizeof(r))) return false; // una sola transacción (auto incremento)
  r[0] &= 0x7F; // segundos
  r[1] &= 0x7F; // minutos
  r[2] &= 0x3F; // horas
  r[4] &= 0x3F; // día
  r[5] &= 0x1F; // mes
  static const uint8_t bcdRegs[] = {0, 1, 2, 4, 5, 6}; // todos menos el día de la semana
  for (uint8_t i : bcdRegs) {
    if (!bcdValid(r[i])) return false;
  }
  t.second = fromBcd(r[0]);
  t.minute = fromBcd(r[1]);
  t.hour = fromBcd(r[2]);
  t.weekday = 0;
  for (uint8_t d = 0; d < 7; d++) {
    if (r[3] & (1u << d)) t.weekday = d;
  }
  t.day = fromBcd(r[4]);
  t.month = fromBcd(r[5]);
  t.year = 2000 + fromBcd(r[6]);
  return t.second < 60 && t.minute < 60 && t.hour < 24 && t.day >= 1 && t.day <= 31 && t.month >= 1 &&
         t.month <= 12;
}

bool Rx8130::setTime(const DateTime &t) {
  const uint8_t r[7] = {
      toBcd(t.second),
      toBcd(t.minute),
      toBcd(t.hour),
      (uint8_t)(1u << (weekdayOf(t.year, t.month, t.day) % 7)), // 01h = domingo
      toBcd(t.day),
      toBcd(t.month),
      toBcd((uint8_t)(t.year % 100)),
  };
  // Sección 18.5: STOP = 1 durante la escritura; el reloj arranca al poner STOP = 0
  if (!writeReg(REG_CTRL0, CTRL0_STOP)) return false;
  const bool ok = writeRegs(REG_SEC, r, sizeof(r));
  if (!writeReg(REG_CTRL0, 0x00) || !ok) return false;
  return writeReg(REG_FLAG, 0x00); // limpia VLF (si el oscilador ya arrancó) y demás banderas
}

uint8_t Rx8130::weekdayOf(uint16_t year, uint8_t month, uint8_t day) {
  // Algoritmo de Sakamoto: 0 = domingo
  static const uint8_t offset[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  if (month < 3) year -= 1;
  return (uint8_t)((year + year / 4 - year / 100 + year / 400 + offset[month - 1] + day) % 7);
}

void Rx8130::fromUnix(uint32_t unixSeconds, DateTime &t) {
  // civil_from_days (H. Hinnant)
  const uint32_t days = unixSeconds / 86400;
  const uint32_t secs = unixSeconds % 86400;
  t.hour = (uint8_t)(secs / 3600);
  t.minute = (uint8_t)((secs % 3600) / 60);
  t.second = (uint8_t)(secs % 60);
  const int32_t z = (int32_t)days + 719468;
  const int32_t era = z / 146097;
  const uint32_t doe = (uint32_t)(z - era * 146097);
  const uint32_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  const uint32_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const uint32_t mp = (5 * doy + 2) / 153;
  t.day = (uint8_t)(doy - (153 * mp + 2) / 5 + 1);
  t.month = (uint8_t)(mp < 10 ? mp + 3 : mp - 9);
  t.year = (uint16_t)(yoe + era * 400 + (t.month <= 2 ? 1 : 0));
  t.weekday = weekdayOf(t.year, t.month, t.day);
}

uint32_t Rx8130::toUnix(const DateTime &t) {
  // days_from_civil (H. Hinnant)
  const int32_t y = (int32_t)t.year - (t.month <= 2 ? 1 : 0);
  const int32_t era = y / 400;
  const uint32_t yoe = (uint32_t)(y - era * 400);
  const uint32_t mp = (t.month + 9) % 12; // marzo = 0
  const uint32_t doy = (153 * mp + 2) / 5 + t.day - 1;
  const uint32_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int32_t days = era * 146097 + (int32_t)doe - 719468;
  return (uint32_t)days * 86400UL + t.hour * 3600UL + t.minute * 60UL + t.second;
}

bool Rx8130::buildTime(DateTime &t) {
  static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *date = __DATE__; // "Oct 15 2026"
  const char *time = __TIME__; // "10:22:31"
  char mon[4] = {date[0], date[1], date[2], '\0'};
  const char *p = strstr(months, mon);
  if (p == nullptr) return false;
  t.month = (uint8_t)((p - months) / 3 + 1);
  t.day = (uint8_t)atoi(date + 4);
  t.year = (uint16_t)atoi(date + 7);
  t.hour = (uint8_t)atoi(time);
  t.minute = (uint8_t)atoi(time + 3);
  t.second = (uint8_t)atoi(time + 6);
  t.weekday = weekdayOf(t.year, t.month, t.day);
  return true;
}
