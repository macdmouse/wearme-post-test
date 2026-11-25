#include "energy_supervisor.h"

const char *EnergySupervisor::name(Mode m) {
  switch (m) {
    case RICH: return "RICH";
    case CORE: return "CORE";
    default: return "SURVIVAL";
  }
}

bool EnergySupervisor::update(float soc, float voltage, float crate, int8_t hourLocal) {
  // Lecturas inválidas (sin MAX17048): se mantiene el modo actual
  if (!(soc >= 0.0f && soc <= 110.0f) || !(voltage > 2.0f && voltage < 5.0f)) return false;

  // C-rate filtrado: media exponencial con constante de tiempo CRATE_TAU_S
  const uint32_t now = millis();
  if (!valid_) {
    crate_ = crate;
  } else {
    const float dt = (now - lastMs_) / 1000.0f;
    const float alpha = dt / (CRATE_TAU_S + dt);
    crate_ += alpha * (crate - crate_);
  }
  lastMs_ = now;
  day_ = hourLocal >= DAY_START_H && hourLocal < DAY_END_H;

  if (forced_) {
    valid_ = true;
    return false; // modo fijado por prueba: se filtra el C-rate pero no cambia
  }

  // De noche y sin cargar, RICH exige más batería (no hay sol para recuperarla)
  const float margin = (!day_ && !charging()) ? NIGHT_MARGIN_PCT : 0.0f;

  Mode next = mode_;
  if (!valid_) {
    // Primera lectura: modo directo por umbrales de entrada
    if (soc < SURVIVAL_ENTER_SOC || voltage < SURVIVAL_ENTER_V) next = SURVIVAL;
    else if (soc >= RICH_ENTER_SOC + margin) next = RICH;
    else next = CORE;
    valid_ = true;
  } else {
    switch (mode_) {
      case RICH:
        if (soc < SURVIVAL_ENTER_SOC || voltage < SURVIVAL_ENTER_V) next = SURVIVAL;
        else if (soc < RICH_EXIT_SOC + margin) next = CORE;
        break;
      case CORE:
        if (soc < SURVIVAL_ENTER_SOC || voltage < SURVIVAL_ENTER_V) next = SURVIVAL;
        else if (soc >= RICH_ENTER_SOC + margin) next = RICH;
        break;
      case SURVIVAL:
        if (soc >= SURVIVAL_EXIT_SOC && voltage >= SURVIVAL_EXIT_V) next = CORE;
        break;
    }
  }
  const bool changed = next != mode_;
  mode_ = next;
  return changed;
}
