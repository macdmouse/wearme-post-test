// ch4_mos.cpp
// CH4 con el GM-402B como Rs/R0 (ver ch4_mos.h).

#include "ch4_mos.h"
#include <math.h>
#include <string.h>

// Con -Ofast isnan() no sirve (siempre false): "sin dato" se reconoce por los bits del float
bool Ch4Mos::hasData(float v) {
  uint32_t bits;
  memcpy(&bits, &v, sizeof(bits));
  return (bits & 0x7F800000u) != 0x7F800000u;
}

bool Ch4Mos::curveValid(const Ch4Curve &c) {
  return c.a > 0.05f && c.a < 1.0f && c.b < -0.01f && c.b > -5.0f && c.refPpm > 0;
}

// Fig. 4: Rs/Rso a 30/60/85 %HR entre −10 y 50 °C (medida a 5000 ppm de CH4; se usa también en aire)
float Ch4Mos::thFactor(float tempC, float humidity) {
  static const float T[] = {-10.0f, 0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f};
  static const float H[] = {30.0f, 60.0f, 85.0f};
  static const float F[3][7] = {
    {1.71f, 1.57f, 1.46f, 1.38f, 1.13f, 1.01f, 0.89f},
    {1.48f, 1.32f, 1.28f, 1.09f, 0.98f, 0.87f, 0.72f},
    {1.28f, 1.16f, 1.09f, 0.92f, 0.86f, 0.72f, 0.67f},
  };
  const float t = constrain(tempC, -10.0f, 50.0f);
  const float h = constrain(humidity, 30.0f, 85.0f);
  const int r = h <= 60.0f ? 0 : 1;
  int c = 0;
  while (c < 5 && t > T[c + 1]) c++;
  const float tf = (t - T[c]) / (T[c + 1] - T[c]);
  const float hf = (h - H[r]) / (H[r + 1] - H[r]);
  const float lo = F[r][c] + (F[r][c + 1] - F[r][c]) * tf;
  const float hi = F[r + 1][c] + (F[r + 1][c + 1] - F[r + 1][c]) * tf;
  return lo + (hi - lo) * hf;
}

void Ch4Mos::begin(const Config &cfg) {
  cfg_ = cfg;
}

void Ch4Mos::heaterOn(uint32_t nowMs) {
  heater_ = true;
  warm_ = false;
  heaterMs_ = nowMs;
  lastMs_ = 0;
  checkMs_ = nowMs;
  rs_ = rsComp_ = rsFilt_ = checkRs_ = changePct_ = NAN;
}

void Ch4Mos::heaterOff() {
  heater_ = false;
  warm_ = false;
  rs_ = rsComp_ = rsFilt_ = checkRs_ = changePct_ = NAN;
}

void Ch4Mos::update(float volts, float tempC, float humidity, uint32_t nowMs) {
  if (!heater_ || !hasData(volts) || volts <= 0.001f || volts >= cfg_.vcVolts) {
    rs_ = rsComp_ = NAN;
    return;
  }
  rs_ = cfg_.rlOhms * (cfg_.vcVolts - volts) / volts;
  // Sin T/HR se usa el punto de referencia (factor 1)
  const bool env = hasData(tempC) && hasData(humidity);
  rsComp_ = env ? rs_ * thFactor(20.0f, 55.0f) / thFactor(tempC, humidity) : rs_;

  const float dt = lastMs_ ? (nowMs - lastMs_) / 1000.0f : 0.0f;
  lastMs_ = nowMs;
  // Filtro de ~1 s para el calentamiento y la línea base (la ppm usa la muestra sin filtrar)
  if (!hasData(rsFilt_) || dt <= 0 || dt > 5.0f) rsFilt_ = rsComp_;
  else rsFilt_ += (rsComp_ - rsFilt_) * (dt / (1.0f + dt));

  if (!warm_) {
    if (nowMs - checkMs_ < 10000) return;
    if (hasData(checkRs_) && checkRs_ > 0) changePct_ = fabsf(rsFilt_ - checkRs_) / checkRs_ * 100.0f;
    checkRs_ = rsFilt_;
    checkMs_ = nowMs;
    if (nowMs - heaterMs_ < cfg_.warmupMinMs || !hasData(changePct_) || changePct_ >= cfg_.stablePct) return;
    warm_ = true;
    // Línea base al terminar el calentamiento
    if (r0_ <= 0 || rsFilt_ >= r0_) r0_ = rsFilt_;
    else if (rsFilt_ >= r0_ * (1.0f - cfg_.r0MaxDropPct / 100.0f)) r0_ = 0.5f * (r0_ + rsFilt_);
    return;
  }

  // Caliente: R0 sube rápido y baja muy lento. Acepta muestras espaciadas (precalentamiento: 1 por minuto).
  if (dt <= 0 || dt > 300.0f || r0_ <= 0) return;
  const float tau = rsFilt_ > r0_ ? cfg_.r0TauUpS : cfg_.r0TauDownS;
  r0_ += (rsFilt_ - r0_) * (dt / (tau + dt));
}

float Ch4Mos::ratio() const {
  if (!warm_ || r0_ <= 0 || !hasData(rsComp_)) return NAN;
  return rsComp_ / r0_;
}

float Ch4Mos::ppmFromRatio(float r) const {
  if (!hasData(r)) return NAN;
  const Ch4Curve &c = cfg_.curve;
  if (r >= 1.0f) return 0.0f;
  float ppm;
  if (r > c.a) ppm = c.refPpm * (1.0f - r) / (1.0f - c.a); // aire -> refPpm: tramo lineal (sin datos del manual)
  else ppm = c.refPpm * powf(r / c.a, 1.0f / c.b);
  return ppm > cfg_.ppmMax ? cfg_.ppmMax : ppm;
}

float Ch4Mos::ppm() const {
  return ppmFromRatio(ratio());
}

bool Ch4Mos::zero() {
  if (!warm_ || !hasData(rsFilt_)) return false;
  r0_ = rsFilt_;
  return true;
}
