#include "nmea_gps.h"
#include <stdlib.h>
#include <string.h>

static uint8_t hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;
}

// "ddmm.mmmm" + hemisferio -> grados decimales
static double nmeaToDegrees(const char *value, const char *hemi) {
  if (value[0] == '\0') return 0;
  const double raw = atof(value);
  const int degrees = (int)(raw / 100);
  double result = degrees + (raw - degrees * 100) / 60.0;
  if (hemi[0] == 'S' || hemi[0] == 'W') result = -result;
  return result;
}

static uint8_t twoDigits(const char *p) { return (uint8_t)((p[0] - '0') * 10 + (p[1] - '0')); }

bool NmeaGps::feed(char c) {
  if (c == '$') { // inicio de trama
    inSentence_ = true;
    len_ = 0;
    return false;
  }
  if (!inSentence_) return false;
  if (c == '\r' || c == '\n') {
    inSentence_ = false;
    buf_[len_] = '\0';
    return parse(buf_);
  }
  if (len_ >= sizeof(buf_) - 1) { // trama demasiado larga: se descarta
    inSentence_ = false;
    return false;
  }
  buf_[len_++] = c;
  return false;
}

bool NmeaGps::parse(char *s) {
  // Checksum: XOR de todo entre '$' y '*'
  char *star = strchr(s, '*');
  if (star == nullptr || strlen(star) < 3) {
    badSentences_++;
    return false;
  }
  uint8_t sum = 0;
  for (char *p = s; p < star; p++) sum ^= (uint8_t)*p;
  const uint8_t hi = hexValue(star[1]), lo = hexValue(star[2]);
  if (hi == 0xFF || lo == 0xFF || sum != (uint8_t)((hi << 4) | lo)) {
    badSentences_++;
    return false;
  }
  *star = '\0';
  sentences_++;

  // Campos separados por coma (los vacíos quedan como "")
  char *fields[20];
  uint8_t n = 0;
  fields[n++] = s;
  for (char *p = s; *p != '\0' && n < 20; p++) {
    if (*p == ',') {
      *p = '\0';
      fields[n++] = p + 1;
    }
  }
  strncpy(lastType_, fields[0], sizeof(lastType_) - 1);
  const size_t typeLen = strlen(fields[0]);
  if (typeLen < 5) return true;
  const char *type = fields[0] + typeLen - 3; // sin el talker
  if (strcmp(type, "GGA") == 0) parseGga(fields, n);
  else if (strcmp(type, "RMC") == 0) parseRmc(fields, n);
  return true;
}

// $xxGGA,hhmmss.ss,lat,N,lon,E,calidad,satelites,HDOP,altura,M,...
void NmeaGps::parseGga(char **f, uint8_t n) {
  if (n < 10) return;
  fix_.quality = (uint8_t)atoi(f[6]);
  fix_.satellites = (uint8_t)atoi(f[7]);
  fix_.hdop = (float)atof(f[8]);
  if (fix_.quality > 0) {
    fix_.latitude = nmeaToDegrees(f[2], f[3]);
    fix_.longitude = nmeaToDegrees(f[4], f[5]);
    fix_.altitudeM = (float)atof(f[9]);
  }
  if (strlen(f[1]) >= 6) {
    fix_.hour = twoDigits(f[1]);
    fix_.minute = twoDigits(f[1] + 2);
    fix_.second = twoDigits(f[1] + 4);
    fix_.timeValid = true;
  }
  fix_.updatedMs = millis();
}

// $xxRMC,hhmmss.ss,A/V,lat,N,lon,E,velocidad,rumbo,ddmmyy,...
void NmeaGps::parseRmc(char **f, uint8_t n) {
  if (n < 10) return;
  if (strlen(f[9]) == 6) {
    fix_.day = twoDigits(f[9]);
    fix_.month = twoDigits(f[9] + 2);
    fix_.year = (uint16_t)(2000 + twoDigits(f[9] + 4));
    fix_.dateValid = fix_.month >= 1 && fix_.month <= 12;
  }
}
