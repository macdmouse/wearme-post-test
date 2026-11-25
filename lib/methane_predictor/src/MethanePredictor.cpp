#include "MethanePredictor.h"
#include <Arduino.h>
#include <cmath>

const float MOLAR_MASS_CH4_G = 16.04f;   // Masa molar del CH4 (g/mol)[cite: 3]
const float SECONDS_PER_DAY = 86400.0f;   // Segundos en 24 horas

// Constantes de Calibración
const float N_ADC = 8388607.0f;            // 2^23 - 1 (ADC 24 bits con signo)[cite: 3]
const float ADC_5000PPM = 2730000.0f;       // Lectura nominal a 5000 ppm
const float A_PARAM = 237758.85058f;
const float EXP_INV = 3.4922297f;           // 1 / 0.28635

float compensate_ppm(float adc_val, float temp, float rh) {
    if (adc_val <= 0.0f) return 0.0f;

    static const float temp_axis[] = {-10.0f, 0.0f, 10.0f, 20.0f, 30.0f, 40.0f, 50.0f};
    static const float rh_axis[]   = {30.0f, 60.0f, 85.0f};
    static const float matriz[3][7] = {
        {1.71f, 1.57f, 1.46f, 1.38f, 1.13f, 1.01f, 0.89f},
        {1.48f, 1.32f, 1.28f, 1.09f, 0.98f, 0.87f, 0.72f},
        {1.28f, 1.16f, 1.09f, 0.92f, 0.86f, 0.72f, 0.67f}
    };

    float t = constrain(temp, -10.0f, 50.0f);
    float h = constrain(rh, 30.0f, 85.0f);

    int r = (h <= 60.0f) ? 0 : 1;
    float rh_frac = (h - rh_axis[r]) / (rh_axis[r+1] - rh_axis[r]);

    int c = 0;
    for (int i = 0; i < 6; i++) {
        if (t >= temp_axis[i] && t <= temp_axis[i+1]) {
            c = i;
            break;
        }
    }
    float t_frac = (t - temp_axis[c]) / (temp_axis[c+1] - temp_axis[c]);

    float v1 = matriz[r][c];
    float v2 = matriz[r][c+1];
    float v3 = matriz[r+1][c];
    float v4 = matriz[r+1][c+1];

    float alpha_5000_act = (1.0f - rh_frac) * (1.0f - t_frac) * v1 + 
                           (1.0f - rh_frac) * t_frac * v2 + 
                           rh_frac * (1.0f - t_frac) * v3 + 
                           rh_frac * t_frac * v4;

    float alpha_5000_ref = 1.284f; 

    float peso_gas = constrain(adc_val / ADC_5000PPM, 0.0f, 1.0f);
    float alpha_act = 1.0f + (alpha_5000_act - 1.0f) * peso_gas;
    float alpha_corregido = alpha_act / alpha_5000_ref;

    float adc_comp = (alpha_corregido * N_ADC * adc_val) / 
                      (N_ADC + (alpha_corregido - 1.0f) * adc_val);

    if (adc_comp <= 0.0f) return 0.0f;

    float ppm_calc = powf(adc_comp / A_PARAM, EXP_INV);

    if (ppm_calc < 2000.0f) {
        return 0.0f;
    }

    return constrain(ppm_calc, 2000.0f, 30000.0f);
}

void process_methane_sample(MethaneState &state, 
                            float raw_adc, 
                            float u_fosa1_mps, 
                            float u_fosa2_mps,
                            float temp_c, 
                            float hum_rh,
                            float pressure_hpa) {
    process_methane_ppm(state, compensate_ppm(raw_adc, temp_c, hum_rh), u_fosa1_mps, u_fosa2_mps, temp_c,
                        pressure_hpa);
}

void process_methane_ppm(MethaneState &state,
                         float current_ppm,
                         float u_fosa1_mps,
                         float u_fosa2_mps,
                         float temp_c,
                         float pressure_hpa) {
    uint32_t now_micros = micros();
    if (state.last_micros == 0) {
        state.last_micros = now_micros;
        state.prev_ppm = current_ppm;
        return;
    }

    float dt = (now_micros - state.last_micros) / 1000000.0f;
    state.last_micros = now_micros;

    if (dt <= 0.0f || dt > 0.5f) dt = 0.025f; // Fallback 40Hz (25ms)

    // 1. Concentración de Metano en PPM: la entrega quien llama (process_methane_sample usa compensate_ppm)

    // 2. Cálculo de la Derivada de PPM (|dPPM / dt|)
    float dppm_dt = (current_ppm - state.prev_ppm) / dt;
    float abs_dppm_dt = fabsf(dppm_dt);
    state.prev_ppm = current_ppm;

    // 3. Densidad Molar del Aire (mol/m3) por Ley de Gases Ideales[cite: 3]
    float p_pa = pressure_hpa * 100.0f;
    float t_k = temp_c + 273.15f;
    float molar_density = p_pa / (8.31446f * t_k); 

    // 4. Flujo Volumétrico y Molar por Cada Fosa
    float vol_flow_fosa1 = fabsf(u_fosa1_mps) * AREA_FOSA_1; // m3/s
    float vol_flow_fosa2 = fabsf(u_fosa2_mps) * AREA_FOSA_2; // m3/s

    float molar_flow_total = (vol_flow_fosa1 + vol_flow_fosa2) * molar_density; // mol/s

    // 5. Flujo Molar de CH4 (mol CH4 / s)[cite: 3]
    double molar_flow_ch4 = (double)(molar_flow_total * (current_ppm / 1000000.0f));

    // 6. Conversión de Tasa Instantánea a Gramos por Día (g/día)[cite: 3]
    // Rate (g/s) = mol/s * 16.04 g/mol | Rate (g/día) = Rate (g/s) * 86400 s/día
    double rate_g_sec = molar_flow_ch4 * (double)MOLAR_MASS_CH4_G;
    state.current_rate_g_day = rate_g_sec * (double)SECONDS_PER_DAY;

    // 7. Evaluación del Trigger por Derivada (Creciente o Decreciente)
    bool trigger_condition = (abs_dppm_dt >= state.derivative_threshold);

    if (trigger_condition) {
        state.event_active = true;
        state.gap_samples = 0;
    } else if (state.event_active) {
        state.gap_samples++;
        if (state.gap_samples >= state.merge_gap_samples) {
            state.event_active = false;
            state.current_event_g = 0.0;
        }
    }

    // 8. Integración de Masa ÚNICAMENTE cuando el Evento está Activo
    if (state.event_active) {
        // Integración trapezoidal en gramos: (g/s_prev + g/s_curr) / 2 * dt_seg
        double prev_g_sec = state.prev_rate_g_day / (double)SECONDS_PER_DAY;
        double delta_grams = 0.5 * (prev_g_sec + rate_g_sec) * (double)dt;
        
        state.total_grams_emitted += delta_grams;
        state.current_event_g += delta_grams;
    }

    state.prev_rate_g_day = state.current_rate_g_day;
}