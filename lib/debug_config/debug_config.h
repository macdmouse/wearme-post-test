// Archivo: debug_config.h
// Descripción: Configura y centraliza la depuración del firmware Wearme,
// habilitando macros de impresión por puerto serie (TinyUSB) e inicialización
// segura del puerto de debug con timeout para no bloquear el sistema.

#pragma once
#undef Debug_Wearme
#define Debug_Wearme true

#include <Arduino.h>
#if defined(USE_TINYUSB) && __has_include(<Adafruit_TinyUSB.h>)
#include <Adafruit_TinyUSB.h>
#endif
#define DEBUG_PORT Serial

#define DEBUG_PRINT(...)             \
  do {                               \
    if (Debug_Wearme) {              \
      DEBUG_PORT.print(__VA_ARGS__); \
    }                                \
  } while (0)
#define DEBUG_PRINTLN(...)             \
  do {                                 \
    if (Debug_Wearme) {                \
      DEBUG_PORT.println(__VA_ARGS__); \
    }                                  \
  } while (0)

static inline void debug_banner_wearme(void); // definida al final del archivo

static inline void debug_init(void) {
  DEBUG_PORT.begin(SERIAL_BAUDRATE);

  // Espera a que el puerto USB esté listo (con timeout de 3 segundos)
  uint32_t startTime = millis();
  while (!DEBUG_PORT && (millis() - startTime) < 3000) {
    delay(10); // Pequeño delay para no saturar el CPU
  }

  // Si el puerto está listo, imprime el mensaje
  if (DEBUG_PORT) {
    delay(2000); // Pequeña pausa para asegurar que el puerto esté listo
    debug_banner_wearme();
  } else {
    // Si no se abrió el puerto, sigue igual (pero no bloquea el sistema)
    DEBUG_PRINTLN("[WARNING] Serial not ready, continuing without debug");
  }
}

// Marca de arranque: OMICAS en matriz de puntos
static inline void debug_banner_wearme(void) {
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN(" [+]===========================================================[+] ");
  DEBUG_PRINTLN("  |                                                           |  ");
  DEBUG_PRINTLN("  |        #######  #     # ### #####     #     #####         |  ");
  DEBUG_PRINTLN("  |        #     #  ##   ##  # #     #   # #   #     #        |  ");
  DEBUG_PRINTLN("  |        #     #  # # # #  # #        #   #  #              |  ");
  DEBUG_PRINTLN("  |        #     #  #  #  #  # #       #     #  #####         |  ");
  DEBUG_PRINTLN("  |        #     #  #     #  # #       #######       #        |  ");
  DEBUG_PRINTLN("  |        #     #  #     #  # #     # #     # #     #        |  ");
  DEBUG_PRINTLN("  |        #######  #     # ### #####  #     #  #####         |  ");
  DEBUG_PRINTLN("  |                                                           |  ");
  DEBUG_PRINTLN("  |   O M I C A S  |  W E A R M E   F I R M W A R E  v1.0.0   |  ");
  DEBUG_PRINTLN("  |               Boot sequence initialized...                |  ");
  DEBUG_PRINTLN("  |                                                           |  ");
  DEBUG_PRINTLN(" [+]===========================================================[+] ");
  DEBUG_PRINTLN("");
}