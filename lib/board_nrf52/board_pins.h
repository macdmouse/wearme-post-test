// board_pins.h
// Pines de la aplicación en la placa Wearme (nRF52840 + LR1110 dentro del módulo WM1110).
// Los pines de los buses (Wire, Wire1, SPI), los LEDs, la QSPI y el LR1110 están en el variant
// (variants/Wearme/variant.h); aquí solo lo que usa el firmware de la placa.
// Numeración Arduino: P0.xx = xx, P1.yy = 32 + yy (p. ej. P1.08 = 40). Detalle en hardware/*.csv.
//
// Componentes conectados:
//   - LEDs de estado verde / rojo / azul y LED de sistema (P0.21)
//   - microSD por SPI (SPIM2; SPIM3 lo usa el LR1110 dentro del módulo)
//   - Wire  (P0.11/P0.16) -> TCA6408ARSVR: expansor de 8 E/S (0x20 con ADDR a GND, 0x21 con ADDR a VCC)
//   - Wire1 (P1.03/P1.05) -> TCA4307DRGR: buffer I2C hot-swap (sin dirección) -> SDA_2_OUT/SCL_2_OUT:
//       MAX17048 (0x36), RX8130CE (0x32), LSM6DS3TR-C (0x6A), mux TCA9543A (0x70) y tarjeta RF (FPC2)
//   - SIP32431DNP3-T1GE4 (varios): interruptores de carga, EN activo en ALTO desde el TCA6408A
//     (tabla POWER_RAILS en board_nrf52.cpp)
//   - Calefactor (HT_PWM), ventilador (FAN_CONTROL), filtro/LNA GNSS (LNA_CTL)
//
// BOARD_PIN_NC = pin sin conectar o todavía sin definir: el código que lo usa se omite.

#ifndef BOARD_PINS_H
#define BOARD_PINS_H

#include <Arduino.h>

static constexpr uint8_t BOARD_PIN_NC = 0xFF;

/* ==================== LEDs ==================== */
static constexpr uint8_t WM_PIN_LED_GREEN = LED_GREEN; // P0.02
static constexpr uint8_t WM_PIN_LED_RED = LED_RED;     // P0.03
static constexpr uint8_t WM_PIN_LED_BLUE = LED_BLUE;   // P0.31
static constexpr bool WM_LED_ACTIVE_LOW = true;        // LEDs en la tarjeta RF (CN1) con ánodo a VDD: encienden con LOW

/* ==================== SPI: microSD (SPI del variant, en SPIM2) ==================== */
static constexpr uint8_t WM_PIN_SD_CS = SS;            // P0.26 (activo en LOW)

/* ==================== I2C ==================== */
// Wire: TCA6408A. Wire1: lado de entrada del TCA4307 (MAX17048 y RX8130CE del otro lado)
static constexpr uint8_t WM_PIN_I2C_BUF_EN = BOARD_PIN_NC;    // TCA4307 EN: pull-up R211 10k a VDD (siempre habilitado)
static constexpr uint8_t WM_PIN_I2C_BUF_READY = BOARD_PIN_NC; // TCA4307 READY: solo pull-up R212 10k, no llega al nRF
static constexpr uint8_t WM_PIN_EXP_RESET = BOARD_PIN_NC;     // TCA6408A RESET#: pull-up R80 10k a VDD
static constexpr uint8_t WM_EXP_I2C_ADDR = 0x20;              // ADDR a GND (confirmado en la placa)
static constexpr uint8_t WM_PIN_FG_ALRT = 19;                 // P0.19 MAX17048 ALRT (open-drain, activo en LOW)
static constexpr uint8_t WM_PIN_RTC_INT = 34;                 // P1.02 INT_RTC (RX8130CE)

/* ==================== Control de potencia y actuadores ==================== */
static constexpr uint8_t WM_PIN_LNA_CTL = PIN_GNSS_LNA; // P0.29 activación del filtro/LNA GNSS (R286)
static constexpr uint8_t WM_PIN_FAN_CTL = 28;           // P0.28 ventilador (R289)
static constexpr uint8_t WM_PIN_FAN_FB = 13;            // P0.13 FAN_FEEDBACK: salida FG del blower (colector abierto)
static constexpr uint8_t WM_PIN_HT_PWM = 7;             // P0.07 calefactor (PWM)
static constexpr uint8_t WM_PIN_CS_POT = 33;            // P1.01 CS_POT: CS del AD5941 (potenciostato, sin soldar), SPI compartido con la SD
static constexpr uint8_t WM_PIN_PGOOD = 36;             // P1.04 PGOOD del cosechador ADP5091

/* ==================== GPS Quectel L96-M33 (tarjeta RF, UART 9600 8N1) ==================== */
// El esquemático marca RX_GPS en P0.14 y TX_GPS en P1.09, sin decir si el nombre es desde el GPS o desde
// el nRF. El firmware detecta en cuál de los dos llegan las tramas (es la TX del GPS) antes de abrir la UART,
// para no poner nunca la TX del nRF contra la TX del GPS.
static constexpr uint8_t WM_PIN_GPS_A = 14;             // P0.14 (R10)
static constexpr uint8_t WM_PIN_GPS_B = 41;             // P1.09 (R9)
static constexpr uint32_t WM_GPS_BAUD = 9600;
// Tarjeta RF con R154 (0 Ω de CTRL_GPS a VDD, venía de fábrica en la primera tarjeta): 1 = el firmware NO maneja
// CTRL_GPS (en BAJO cortocircuitaría VDD). Diseño final SIN R154: 0, el GPS lo enciende el TCA6408A (P0).
// No se detecta solo: CTRL_GPS no tiene resistencia a GND y como entrada queda flotando en el último nivel.
#define WM_RF_R154_MOUNTED 0

#endif
