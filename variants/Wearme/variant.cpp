/*
 * variant.cpp — Placa Wearme (OMICAS)
 * Basado en el variant Seeed_Wio_Tracker_1110 (Copyright (C) 2023 Seeed K.K., MIT License).
 */

#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"

const uint32_t g_ADigitalPinMap[PINS_COUNT + 1] =
{
  0,  // P0.00 XL1
  1,  // P0.01 XL2
  2,  // P0.02 AIN0 LED_GREEN
  3,  // P0.03 AIN1 LED_RED
  4,  // P0.04 AIN2 DATA
  5,  // P0.05 AIN3 MOSI
  6,  // P0.06 CLOCK
  7,  // P0.07 HT_PWM
  8,  // P0.08
  9,  // P0.09 NFC1 RESET_AD
  10, // P0.10 NFC2 INT_PIN
  11, // P0.11 SDA
  12, // P0.12
  13, // P0.13
  14, // P0.14
  15, // P0.15 QSPI_DATA2
  16, // P0.16 SCL
  17, // P0.17 QSPI_DATA1
  18, // P0.18 nRESET
  19, // P0.19 ALRT (MAX17048)
  20, // P0.20 QSPI_SCK
  21, // P0.21 LED de sistema
  22, // P0.22 QSPI_CS
  23, // P0.23 RX_NRF
  24, // P0.24 QSPI_DATA3
  25, // P0.25 TX_NRF
  26, // P0.26 CS_SD
  27, // P0.27 SCLK
  28, // P0.28 AIN4 FAN_CONTROL
  29, // P0.29 AIN5 LNA_CTL
  30, // P0.30 AIN6 MISO
  31, // P0.31 AIN7 LED_BLUE
  32, // P1.00 QSPI_DATA0
  33, // P1.01 CS_POT
  34, // P1.02 INT_RTC
  35, // P1.03 SDA_2_IN
  36, // P1.04 PGOOD
  37, // P1.05 SCL_2_IN
  38, // P1.06
  39, // P1.07
  40, // P1.08 LR_DIO9
  41, // P1.09
  42, // P1.10 LR1110 NRESET
  43, // P1.11 LR1110 BUSY
  44, // P1.12 LR1110 NSS
  45, // P1.13 LR1110 SCK
  46, // P1.14 LR1110 MOSI
  47, // P1.15 LR1110 MISO
  255,  // NRFX_SPIM_PIN_NOT_USED
};

// Sin configuración de pines aquí: toda la inicialización de la placa está en board_init().
// Hasta entonces el core deja todos los pines como entradas (con OUT = 1).
void initVariant()
{
}
