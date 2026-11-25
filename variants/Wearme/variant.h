/*
 * variant.h — Placa Wearme (OMICAS)
 *
 * Módulo Seeed Wio-WM1110 (nRF52840 + Semtech LR1110) con el bootloader del Adafruit Feather
 * nRF52840 Express (SoftDevice S140 6.1.1). Pines según el esquemático de la placa; el detalle
 * de cada pin está en hardware/1_pines_nrf52840.csv.
 *
 * Numeración Arduino: P0.xx = xx, P1.yy = 32 + yy (mapa 1:1 en variant.cpp).
 * Basado en el variant Seeed_Wio_Tracker_1110 (Copyright (C) 2023 Seeed K.K., MIT License).
 */

#pragma once

#include "WVariant.h"

////////////////////////////////////////////////////////////////////////////////
// Low frequency clock source

#define USE_LFXO                // cristal de 32.768 kHz en XL1/XL2 (X1, C193/C194)

////////////////////////////////////////////////////////////////////////////////
// Number of pins

#define PINS_COUNT              (48)
#define NUM_DIGITAL_PINS        (48)

////////////////////////////////////////////////////////////////////////////////
// Analog pin definition (AIN0..AIN7 son fijos en el nRF52840)

#define NUM_ANALOG_INPUTS       (8)
#define NUM_ANALOG_OUTPUTS      (0)

#define A0                      (2)             // P0.02 AIN0 (LED_GREEN)
#define A1                      (3)             // P0.03 AIN1 (LED_RED)
#define A2                      (4)             // P0.04 AIN2 (DATA)
#define A3                      (5)             // P0.05 AIN3 (MOSI)
#define A4                      (28)            // P0.28 AIN4 (FAN_CONTROL)
#define A5                      (29)            // P0.29 AIN5 (LNA_CTL)
#define A6                      (30)            // P0.30 AIN6 (MISO)
#define A7                      (31)            // P0.31 AIN7 (LED_BLUE)

////////////////////////////////////////////////////////////////////////////////
// UART pin definition

// Serial1 hacia el ESP (TX_ESP -> RX_NRF, RX_ESP -> TX_NRF). R171/R172: confirmar si van montadas.
#define PIN_SERIAL1_RX          (23)            // P0.23 RX_NRF
#define PIN_SERIAL1_TX          (25)            // P0.25 TX_NRF

////////////////////////////////////////////////////////////////////////////////
// I2C pin definition

#define WIRE_INTERFACES_COUNT   (2)

#define PIN_WIRE_SDA            (11)            // P0.11 SDA   -> Wire:  TCA6408A (pull-ups R182/R183 2.1k)
#define PIN_WIRE_SCL            (16)            // P0.16 SCL
#define PIN_WIRE1_SDA           (35)            // P1.03 SDA_2_IN -> Wire1: TCA4307 -> MAX17048, RX8130CE
#define PIN_WIRE1_SCL           (37)            // P1.05 SCL_2_IN

////////////////////////////////////////////////////////////////////////////////
// SPI pin definition

#define SPI_INTERFACES_COUNT    (1)
// SPI en SPIM2: SPIM3 lo usa LBM_WM1110 para el LR1110 (pines internos P1.12-P1.15)
#define SPI_32MHZ_INTERFACE     (1)

#define PIN_SPI_MISO            (30)            // P0.30 MISO  -> microSD
#define PIN_SPI_MOSI            (5)             // P0.05 MOSI
#define PIN_SPI_SCK             (27)            // P0.27 SCLK

static const uint8_t SS = 26;                   // P0.26 CS_SD

////////////////////////////////////////////////////////////////////////////////
// QSPI Flash (externa, modelo por definir: sin EXTERNAL_FLASH_DEVICES)

#define PIN_QSPI_SCK            (20)            // P0.20
#define PIN_QSPI_CS             (22)            // P0.22
#define PIN_QSPI_IO0            (32)            // P1.00
#define PIN_QSPI_IO1            (17)            // P0.17
#define PIN_QSPI_IO2            (15)            // P0.15
#define PIN_QSPI_IO3            (24)            // P0.24

////////////////////////////////////////////////////////////////////////////////
// Builtin LEDs

#define LED_BUILTIN             (21)            // P0.21 LED de sistema (R202 + B1 a VDD): enciende con LOW
#define LED_STATE_ON            (0)             // nivel de LED_BUILTIN (ledOn/ledOff del core)

// LEDs de estado (led_lib maneja su polaridad, por confirmar)
#define LED_GREEN               (2)             // P0.02
#define LED_RED                 (3)             // P0.03
#define LED_BLUE                (31)            // P0.31

////////////////////////////////////////////////////////////////////////////////
// Semtech LR1110 (dentro del módulo WM1110; los pines del SPI los fija LBM_WM1110)

#define PIN_LR1110_NRESET       (42)            // P1.10
#define PIN_LR1110_BUSY         (43)            // P1.11
#define PIN_LR1110_NSS          (44)            // P1.12
#define PIN_LR1110_SCK          (45)            // P1.13
#define PIN_LR1110_MOSI         (46)            // P1.14
#define PIN_LR1110_MISO         (47)            // P1.15
#define PIN_LR1110_IRQ          (40)            // P1.08 <- LR_DIO9 (el HAL lee la IRQ por SPI)

#define PIN_GNSS_LNA            (29)            // P0.29 LNA_CTL: filtro/LNA de la antena GNSS
