#pragma once

// =====================================================================================
// EnergySupervisor — mini máquina de estados de energía: RICH / CORE / SURVIVAL.
//
// Entradas (update(), ~1 vez por segundo despierto o cada despertar en SURVIVE):
//   SOC (%) y voltaje de celda del MAX17048, C-rate (%/h, + carga / − descarga) y la hora local del RTC.
//
//   RICH      batería holgada: todo (gases con calefactor, blower, GPS)
//   CORE      batería media: se mide sin gases (sin calefactores ni blower)
//   SURVIVAL  batería crítica: no se mide; rieles cortados y sueño profundo (estado SURVIVE de la FSM)
//
// Histéresis: cada cambio tiene un umbral de entrada y uno de salida distintos (no oscila cerca del
// borde). El C-rate se filtra (media exponencial, constante de tiempo CRATE_TAU_S) porque el MAX17048 lo
// da con mucho ruido. Día/noche: de noche y sin cargar (no hay sol que recupere la batería) los umbrales
// para estar en RICH suben NIGHT_MARGIN_PCT. Sin hora válida se asume noche (conservador).
// =====================================================================================

#include <Arduino.h>

class EnergySupervisor {
public:
  enum Mode : uint8_t { SURVIVAL = 0, CORE = 1, RICH = 2 };

  // ---- Umbrales (SOC en %, voltaje en V) ----
  static constexpr float RICH_ENTER_SOC = 60.0f;     // CORE -> RICH
  static constexpr float RICH_EXIT_SOC = 50.0f;      // RICH -> CORE
  // SURVIVAL por voltaje: en la mitad baja el SOC del MAX17048 se queda corto (descarga del 16 al 18-oct-2026:
  // marcó 15 % a 3.70 V con ~30 % real; 3.50 V ≈ 11 % real). El SOC queda solo como respaldo.
  static constexpr float SURVIVAL_ENTER_SOC = 5.0f;  // CORE -> SURVIVAL (respaldo)
  static constexpr float SURVIVAL_EXIT_SOC = 20.0f;  // SURVIVAL -> CORE
  static constexpr float SURVIVAL_ENTER_V = 3.50f;   // CORE/RICH -> SURVIVAL (criterio principal)
  static constexpr float SURVIVAL_EXIT_V = 3.70f;    // para salir hace falta voltaje de sobra
  static constexpr float NIGHT_MARGIN_PCT = 10.0f;   // de noche sin carga: RICH exige 10 % más
  static constexpr float CHARGING_CRATE = 0.5f;      // %/h filtrado: por encima se considera cargando
  static constexpr float CRATE_TAU_S = 300.0f;       // constante de tiempo del filtro del C-rate
  static constexpr uint8_t DAY_START_H = 6;          // día: de 6:00 a 18:00 hora local
  static constexpr uint8_t DAY_END_H = 18;

  // hourLocal: 0-23, o −1 si no hay hora válida. Devuelve true si cambió el modo.
  bool update(float soc, float voltage, float crate, int8_t hourLocal);

  Mode mode() const { return mode_; }
  const char *modeName() const { return name(mode_); }
  static const char *name(Mode m);
  float crateFiltered() const { return crate_; }
  bool isDay() const { return day_; }
  bool charging() const { return crate_ > CHARGING_CRATE; }
  bool valid() const { return valid_; } // ya recibió una lectura de batería

  // Políticas
  bool allowGas() const { return mode_ == RICH; }         // calefactores, blower, H2
  bool allowMeasure() const { return mode_ != SURVIVAL; }

  // Pruebas: fija el modo sin mirar la batería (comando serie ENERGY:...); autoMode() vuelve a lo normal
  void force(Mode m) { forced_ = true; mode_ = m; }
  void autoMode() { forced_ = false; }
  bool forced() const { return forced_; }

private:
  Mode mode_ = CORE; // hasta la primera lectura
  bool forced_ = false;
  float crate_ = 0;
  uint32_t lastMs_ = 0;
  bool day_ = false;
  bool valid_ = false;
};
