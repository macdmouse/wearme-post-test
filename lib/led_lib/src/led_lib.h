#pragma once

// =====================================================================================
// led_lib — driver no bloqueante de los LEDs de estado de la placa Wearme (verde P0.02, rojo P0.03,
// azul P0.31). Solo el mecanismo: destellos, encendido fijo y un patrón periódico.
//
// QUÉ significa cada patrón (eventos, estados y códigos de falla) está en lib/failcodes, en tablas
// fáciles de cambiar. Aquí no hay significados.
//
// update() se llama en cada loop(); los destellos se programan con play()/flash() y se ejecutan solos.
// No usa delay(): respeta el límite de ~8 ms por loop que necesita LoRa Basics Modem.
// =====================================================================================

#include <Arduino.h>
#include <board_pins.h>

static constexpr uint8_t STATUS_LED_GREEN_PIN = WM_PIN_LED_GREEN;
static constexpr uint8_t STATUS_LED_RED_PIN = WM_PIN_LED_RED;
static constexpr uint8_t STATUS_LED_BLUE_PIN = WM_PIN_LED_BLUE;
static constexpr bool STATUS_LED_ACTIVE_LOW = WM_LED_ACTIVE_LOW;

// LEDs de un patrón; se combinan: LED_MASK_GREEN | LED_MASK_BLUE = cian, LED_MASK_RED | LED_MASK_GREEN = amarillo
enum : uint8_t { LED_MASK_GREEN = 0x01, LED_MASK_RED = 0x02, LED_MASK_BLUE = 0x04 };

// Patrón: `blinks` destellos simultáneos en los LEDs de `leds`, onMs encendidos y offMs apagados
struct LedCode {
  uint8_t leds;
  uint8_t blinks;
  uint16_t onMs;
  uint16_t offMs;
};

class StatusLeds {
public:
  enum Led : uint8_t { GREEN = 0, RED = 1, BLUE = 2, COUNT = 3 };

  // activeLow = true si el LED enciende con LOW
  void begin(uint8_t greenPin = STATUS_LED_GREEN_PIN, uint8_t redPin = STATUS_LED_RED_PIN,
             uint8_t bluePin = STATUS_LED_BLUE_PIN, bool activeLow = STATUS_LED_ACTIVE_LOW);

  // Llamar en cada loop(): destellos programados y el patrón periódico.
  void update();

  void booting(); // los tres encendidos (arranque en curso)
  void allOff();  // apaga todo y cancela los destellos (no el patrón periódico)

  // Destellos de un patrón (reemplaza el destello en curso de esos LEDs)
  void play(const LedCode &code);
  // play(*code) cada periodMs; nullptr = sin patrón periódico. El primero sale un periodo después.
  void periodic(const LedCode *code, uint32_t periodMs);

  // Programa `count` destellos en un LED (reemplaza el que esté en curso en ese LED)
  void flash(Led led, uint8_t count = 1, uint16_t onMs = 80, uint16_t offMs = 150);
  // Encendido fijo. Cancela los destellos de ese LED.
  void set(Led led, bool on);

  // Hay un destello en curso (no dormir más de unos ms: el LED quedaría encendido)
  bool busy() const;

private:
  struct Channel {
    uint8_t pin;
    uint8_t togglesLeft; // cambios de estado pendientes del parpadeo (2 por destello)
    uint16_t onMs, offMs;
    uint32_t nextMs;
    bool on;
    bool forced;         // set(led, true)
  };

  void write(Channel &c, bool on);

  Channel ch_[COUNT];
  bool activeLow_ = false;
  const LedCode *periodic_ = nullptr;
  uint32_t periodMs_ = 0;
  uint32_t periodicNextMs_ = 0;
};
