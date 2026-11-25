// ch4_mos.h
// CH4 con el sensor MOS Winsen GM-402B (manual v2.2, hardware/mems--gm-402b--manual-v2_2.pdf) a partir del
// voltaje sobre la resistencia de carga RL. Solo cálculo: no toca hardware ni flash (eso lo hace la placa).
//
//   1. Rs = RL × (VC − Vout) / Vout                                    (Fig. 2, circuito estándar)
//   2. Rs compensada a 20 °C / 55 %HR con la tabla Rs/Rso de la Fig. 4 (T y HR del BME280)
//   3. Calentamiento: el sensor recién encendido no sirve (Vout arranca saturado y baja durante ~1 min). Queda
//      "caliente" cuando pasa warmupMinMs y Rs cambia menos de stablePct en 10 s (y se queda así hasta apagarlo).
//   4. Línea base R0 = Rs en aire limpio (definición de la Fig. 3). Es dinámica:
//        - al quedar caliente: si no hay R0 se toma Rs; si Rs > R0 se toma Rs; si Rs baja hasta r0MaxDropPct se
//          promedia (deriva); si baja más se conserva R0 (probablemente hay gas al empezar)
//        - mientras está caliente: sube rápido (tau r0TauUpS: el metano solo BAJA Rs, así que Rs > R0 significa
//          que R0 estaba baja) y baja muy lento (tau r0TauDownS: deriva de horas, los eventos de gas no la mueven)
//   5. ppm con la curva Rs/R0 = a × (ppm / refPpm)^b (Fig. 3: a = 0.72 a 1000 ppm, b = −0.204). Entre el aire
//      (Rs/R0 = 1) y refPpm el manual no da datos: tramo lineal en (1 − Rs/R0), que es como responde un MOS a baja
//      concentración. Hasta tener una curva calibrada con gas el valor es una ESTIMACIÓN (curva típica del
//      fabricante; con pendiente tan plana, 10 % de error en R0 ≈ 60 % de error en ppm).

#ifndef CH4_MOS_H
#define CH4_MOS_H

#include <Arduino.h>

struct Ch4Curve {
  float a;      // Rs/R0 a refPpm
  float b;      // exponente (negativo: Rs baja con el gas)
  float refPpm; // concentración del punto a
};

class Ch4Mos {
public:
  struct Config {
    float vcVolts = 5.0f;          // VC del divisor
    float rlOhms = 3000.0f;        // RL (se cancela en Rs/R0; solo afecta Rs y R0 en ohm)
    Ch4Curve curve = {0.72f, -0.204f, 1000.0f};
    float ppmMax = 10000.0f;       // rango del manual
    uint32_t warmupMinMs = 60000;  // calentamiento mínimo
    float stablePct = 1.0f;        // Rs estable: cambia menos de esto (%) en 10 s
    float r0TauUpS = 20.0f;
    float r0TauDownS = 21600.0f;   // 6 h
    float r0MaxDropPct = 10.0f;
  };

  void begin(const Config &cfg);
  void heaterOn(uint32_t nowMs); // empieza el calentamiento
  void heaterOff();
  // Una muestra: voltios de Vout (NAN = sin dato), T (°C) y HR (%) del ambiente (NAN = sin compensar)
  void update(float volts, float tempC, float humidity, uint32_t nowMs);

  bool heaterIsOn() const { return heater_; }
  uint32_t heatingMs(uint32_t nowMs) const { return heater_ ? nowMs - heaterMs_ : 0; }
  bool warm() const { return warm_; }        // calentamiento terminado
  float changePct() const { return changePct_; } // último cambio de Rs en 10 s (%), NAN sin dato
  float rs() const { return rs_; }           // ohm (última muestra), NAN sin dato
  float rsComp() const { return rsComp_; }   // ohm a 20 °C / 55 %HR
  float rsFiltered() const { return rsFilt_; } // rsComp filtrada (~1 s)
  bool r0Valid() const { return r0_ > 0; }
  float r0() const { return r0_; }           // ohm, 0 = sin línea base
  float ratio() const;                       // rsComp / R0 (NAN si no está caliente o sin R0)
  float ppm() const;                         // NAN si no está caliente o sin R0
  float ppmFromRatio(float ratio) const;

  void setR0(float ohms) { r0_ = ohms > 0 ? ohms : 0; }
  bool zero();                               // R0 = Rs filtrada actual (solo caliente)
  void setCurve(const Ch4Curve &c) { cfg_.curve = c; }
  const Ch4Curve &curve() const { return cfg_.curve; }
  const Config &config() const { return cfg_; }
  static bool curveValid(const Ch4Curve &c);
  static float thFactor(float tempC, float humidity); // Rs/Rso de la Fig. 4

private:
  static bool hasData(float v);
  Config cfg_;
  bool heater_ = false, warm_ = false;
  uint32_t heaterMs_ = 0, lastMs_ = 0, checkMs_ = 0;
  float rs_ = NAN, rsComp_ = NAN, rsFilt_ = NAN, checkRs_ = NAN, changePct_ = NAN;
  float r0_ = 0;
};

#endif
