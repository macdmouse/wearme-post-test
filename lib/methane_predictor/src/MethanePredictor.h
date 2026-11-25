#ifndef METHANE_PREDICTOR_H
#define METHANE_PREDICTOR_H

#include <stdint.h>

/**
 * @brief Estado persistente para el predictor de 40Hz.
 */
struct MethaneState {
    double total_grams_emitted = 0.0;     // Masa total acumulada de CH4 (g)
    double current_rate_g_day = 0.0;      // Tasa de emisión instantánea (g/día)
    double prev_rate_g_day = 0.0;         // Tasa anterior para integración trapezoidal (g/día)
    float prev_ppm = 0.0f;                // Lectura de PPM anterior para cálculo de derivada
    uint32_t last_micros = 0;

    // Lógica de eventos y Trigger por Derivada
    bool event_active = false;
    double current_event_g = 0.0;
    uint32_t gap_samples = 0;
    
    // Configuración del Trigger por cambio brusco en PPM
    // Sensible a flancos crecientes y decrecientes (|dPPM/dt| >= derivative_threshold)
    float derivative_threshold = 500.0f;  // Umbral de derivada en [PPM / segundo] (Ajustable)
    uint32_t merge_gap_samples = 8;       // Ventana de histeresis (8 muestras a 40Hz = 0.2s)
};

// Áreas transversales independientes para cada fosa (m2)
// Valor anterior total (0.0004712 m2) dividido en dos
const float AREA_FOSA_1 = 0.0002356f; 
const float AREA_FOSA_2 = 0.0002356f; 

void process_methane_sample(MethaneState &state, 
                            float raw_adc, 
                            float u_fosa1_mps, 
                            float u_fosa2_mps,
                            float temp_c, 
                            float hum_rh,
                            float pressure_hpa);

float compensate_ppm(float adc_val, float temp, float rh);

// Igual que process_methane_sample pero con la concentración ya calculada (p. ej. lib/ch4_mos: Rs/R0)
void process_methane_ppm(MethaneState &state,
                         float current_ppm,
                         float u_fosa1_mps,
                         float u_fosa2_mps,
                         float temp_c,
                         float pressure_hpa);

#endif