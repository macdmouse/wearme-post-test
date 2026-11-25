// ring_diag.h
// Diagnóstico del anillo Meshtastic por el puerto serie: intensidad de señal con cada nodo, quién está cerca o fuera
// de rango, a quién le llegaron los paquetes de este collar, por dónde llegó cada paquete y cómo varía la señal.
// Lógica pura (no toca el radio): la placa le pasa todo lo que recibe (onPacket) y transmite lo que nextTx() entrega.
//
// Pasivo (siempre): cada paquete que se oye imprime una línea [DIAG] RX con el origen, el último salto (el nodo que
//   lo transmitió: el origen si llegó directo, o el que lo repitió, por el byte relay_node de la cabecera), los
//   saltos, el RSSI/SNR y cuánto cambió respecto del anterior de ese nodo. Si otro nodo repite un paquete PROPIO,
//   le llegó (ECO). La tabla de vecinos (printTable) resume todo, ordenada por estado y señal.
//
// Activo (sesión de sondeo, RING:DIAG:<min> por serie): este collar manda un PING cada ciclo; todo collar con este
//   firmware que lo oye responde PONG con el RSSI/SNR, los saltos y el último salto con que lo oyó (enlace de ida) y
//   el que sondea mide el PONG (enlace de vuelta). Un collar que oye un PING (en una ventana del anillo) queda
//   respondiendo los minutos que dice el PING: la placa lo pone a escuchar fuera de las ventanas. PING y PONG se
//   repiten como el resto del anillo (inundación controlada) para ver las rutas de más de un salto.
//   Mensajes (puerto PRIVATE_APP, misma cabecera que el anillo 'W' 'M' tipo versión DevEUI(8) número(4)):
//     PING  número = secuencia | minutos restantes de la sesión (0 = fin) | nombre corto (4)
//     PONG  número = secuencia del PING | nodo que sondea (4) | RSSI (int8) | SNR (int8) | saltos | último salto
//           (byte relay_node) | nombre corto (4)
//   Un collar con firmware anterior solo los repite (tipo desconocido).

#ifndef RING_DIAG_H
#define RING_DIAG_H

#include <Arduino.h>
#include <mesh_lib.h>

class RingDiag {
public:
  struct Config {
    uint32_t pongMinMs = 400;      // PONG: retardo aleatorio (varios collares responden al mismo PING)
    uint32_t pongMaxMs = 6000;
    uint32_t relayMinMs = 300;     // repetición de PING/PONG (SNR bajo = antes, como el anillo)
    uint32_t relayMaxMs = 2500;
    uint8_t lostPings = 3;         // PING seguidos sin PONG de un nodo que ya respondió: FUERA DE RANGO
    uint32_t lostIdleMs = 25UL * 60UL * 1000UL; // sin sesión: sin oírlo este tiempo = FUERA DE RANGO (2 ventanas)
    uint8_t tableEvery = 6;        // en la sesión, la tabla completa cada tantos PING
    // Registro (SD): cada evento como campos CSV separados por ';' (ver LOG_FIELDS); la placa agrega hora, GPS, LoRa
    void (*log)(const char *fields) = nullptr;
  };
  static constexpr const char *LOG_FIELDS =
      "evento;nodo;nombre;ultimo_salto;saltos;rssi_dbm;snr_db;estado;ida_rssi_dbm;ida_snr_db;ida_saltos";

  void begin(MeshtasticTx *tx, const uint8_t devEui[8], const char *shortName, const Config &cfg, uint32_t seed);

  // Sesión de sondeo de este collar: minutes > 0 empieza (o extiende), 0 termina (avisa a los que responden)
  void start(uint16_t minutes, uint32_t nowMs);
  // Sondea o responde: la placa debe escuchar fuera de las ventanas
  bool active() const { return initiator_ || responder_; }
  bool initiator() const { return initiator_; }
  // Ciclo nuevo (la placa abrió la escucha o una ventana): evalúa el PING anterior y programa el siguiente en
  // pingDelayMs. Solo hace algo si este collar sondea.
  void cycle(uint32_t nowMs, uint32_t pingDelayMs);
  // Fin de la escucha: lo pendiente (PONG, repeticiones) no alcanzó y se descarta
  void sliceEnd();
  // Vencimiento de la sesión y nodos que dejan de oírse (llamar seguido)
  void tick(uint32_t nowMs);

  // Paquete recibido (cualquiera, con o sin ventana; se copia)
  void onPacket(const uint8_t *pkt, size_t len, int16_t rssi, int16_t snr, uint32_t nowMs);
  // Próximo PING / PONG / repetición a transmitir ya; untilMs = no empezar algo que no termine antes. 0 = nada.
  size_t nextTx(uint32_t nowMs, uint32_t untilMs, uint8_t *out, size_t max, const char *&what);

  void printTable(uint32_t nowMs) const;
  void reset(); // borra la tabla de vecinos (antes de una nueva prueba de recorrido)
  uint8_t count() const;

private:
  struct Node {
    uint32_t num;          // número de nodo Meshtastic (0 = libre); en un collar = últimos 4 bytes del DevEUI
    char name[6];          // nombre corto (N003) del PING/PONG o del NodeInfo
    bool wearme;
    // Enlace él -> yo (paquetes que transmitió él: los suyos directos o los que repitió)
    int16_t rssi, rssiMin, rssiMax;
    int8_t snr;
    int32_t avg16;         // RSSI promedio móvil x16
    uint16_t nDirect, nRelayed;
    uint32_t directMs, heardMs; // última vez oído directo / por cualquier ruta (0 = nunca)
    uint8_t hops, via;     // saltos y último salto (relay_node) de su último paquete
    // Enlace yo -> él (PONG o ECO)
    int16_t upRssi;
    int8_t upSnr;
    uint8_t upHops, downHops; // saltos del PING hasta él (ida) y de su PONG hasta aquí (vuelta)
    uint32_t upMs;
    uint16_t echoes;       // paquetes propios que repitió (le llegaron)
    // Sesión de sondeo
    uint16_t pongs, pings; // PONG recibidos / PING enviados desde que respondió el primero
    uint8_t miss;
    bool inSession, ponged, lost;
  };
  struct Relay {
    bool used;
    uint32_t from, id, sendAt;
    uint8_t len;
    uint8_t pkt[MeshtasticTx::MAX_PACKET];
  };
  struct Pong {
    bool used;
    uint32_t pinger, seq, sendAt;
    int8_t rssi, snr;
    uint8_t hops, via;
  };
  struct Seen {
    uint32_t from, id;
  };
  static constexpr uint8_t N_NODES = 16, N_RELAY = 4, N_PONG = 4, N_SEEN = 24;

  uint32_t rnd();
  uint32_t rndRange(uint32_t lo, uint32_t hi) { return hi > lo ? lo + rnd() % (hi - lo) : lo; }
  bool seenBefore(uint32_t from, uint32_t id);
  void cancelRelay(uint32_t from, uint32_t id);
  Node *find(uint32_t num);
  Node *add(uint32_t num, uint32_t nowMs);
  const Node *byRelay(uint8_t b) const;
  void label(const Node *n, uint32_t num, char *out, size_t size) const;
  void viaLabel(uint8_t b, char *out, size_t size) const;
  const char *state(const Node &n, uint32_t nowMs) const;
  uint8_t rank(const Node &n, uint32_t nowMs) const;
  void heardFrom(Node &n, int16_t rssi, int16_t snr, uint32_t nowMs);
  void endSession(const char *why);
  static constexpr int ND = -32768; // sin dato en logRow
  void logRow(const char *ev, uint32_t num, const char *name, const char *via, int hops, int rssi, int snr,
              const char *st, int upRssi = ND, int upSnr = ND, int upHops = ND) const;
  size_t build(uint8_t type, uint32_t number, const uint8_t *extra, uint8_t extraLen, uint8_t *out, size_t max);

  MeshtasticTx *tx_ = nullptr;
  Config cfg_;
  uint8_t dev_[8] = {};
  char name_[6] = "";
  uint32_t rng_ = 1;
  Node nodes_[N_NODES] = {};
  Relay relay_[N_RELAY] = {};
  Pong pong_[N_PONG] = {};
  Seen seen_[N_SEEN] = {};
  uint8_t seenNext_ = 0;
  // Sesión
  bool initiator_ = false, responder_ = false, stopping_ = false;
  uint32_t endMs_ = 0, respEndMs_ = 0, startMs_ = 0;
  uint32_t seq_ = 0, pingAt_ = 0, pingMs_ = 0; // pingAt_ = PING programado (0 = no); pingMs_ = envío del último
  bool pingSent_ = false;                      // el último ciclo mandó PING (se evalúa al empezar el siguiente)
  uint32_t lastPinger_ = 0;
};

#endif
