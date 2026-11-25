// mesh_ring.h
// Anillo de respaldo entre collares Wearme sobre paquetes Meshtastic (lib/mesh_lib, puerto PRIVATE_APP 256).
// Lógica pura: no toca el radio. La placa abre una ventana sincronizada por hora UTC (todos los collares a la vez),
// le pasa lo que recibe (onPacket) y transmite lo que nextTx() le entrega; después de la ventana, si el collar es
// puente (tiene LoRaWAN), sube por LoRaWAN lo que reclamó (takeUpload / uploadResult).
//
//   Origen (sin entrega por LoRaWAN): MEDICIÓN [DevEUI, hora UTC, trama de 22 B] en cada ventana, hasta oír
//                                      ENTREGADO o agotar maxAttempts ventanas (la trama sigue en PENDIENTES.csv).
//   Cualquier collar:                  repite lo que no ha visto (caché origen + id, hop_limit), con un retardo
//                                      aleatorio (menor con SNR bajo: los lejanos primero) y lo cancela si oye a otro
//                                      repetir el mismo paquete (inundación controlada, como Meshtastic).
//   Puente (LoRaWAN unido):            al oír una MEDICIÓN nueva espera un retardo (menor con mejor enlace LoRaWAN) y
//                                      manda RECLAMO; si antes oye el RECLAMO de otro, desiste (empate: gana el número
//                                      de nodo menor). Tras la ventana la sube (FPort 20, confirmada) y, con ACK,
//                                      anuncia ENTREGADO al principio de la ventana siguiente.
//
// Hora por la mesh (como Meshtastic): las ventanas dependen de la hora UTC, así que los collares con hora mandan un
// faro HORA al principio de cada ventana (uno por zona: si se oye otro de igual o mejor calidad, se cancela; no se
// repite porque llegaría atrasado). Un collar sin hora escucha en continuo (la placa) y la toma con parseTime().
//
// Comandos hacia un collar sin cobertura (downlink por el anillo):
//   Puente:   recibe de la plataforma (downlink FPort 21) o por serie (RING:CMD) [DevEUI destino, id, comando] y lo
//             manda como COMANDO en las ventanas siguientes (hasta cmdMaxAttempts o hasta oír COMANDO OK).
//   Destino:  ejecuta el comando una sola vez por id (caché de ids ejecutados) y responde COMANDO OK; si lo vuelve a
//             recibir solo re-confirma. El puente sube la confirmación a la plataforma (FPort 22).
//
// Anti-duplicados (la clave de una medición es DevEUI + hora UTC):
//   1. caché de paquetes vistos (origen + id): no se repite dos veces lo mismo
//   2. RECLAMO / ENTREGADO oídos: otros puentes no suben lo que ya se reclamó o entregó
//   3. caché de subidas propias: un puente nunca sube dos veces la misma medición (si la vuelve a oír, re-anuncia)
//   4. la plataforma descarta lo que repita DevEUI + hora (dos puentes que no se oyen pueden subir lo mismo)

#ifndef MESH_RING_H
#define MESH_RING_H

#include <Arduino.h>
#include <mesh_lib.h>

static constexpr size_t RING_FRAME_LEN = 30; // trama principal v2 (lib/lora_frames, FPort 10)
static constexpr size_t RING_CMD_MAX = 12;   // bytes de un comando (los mismos de un downlink: 01, 04 MM MM...)
static constexpr uint8_t RING_WIRE_VERSION = 2; // versión de los mensajes 'W' 'M' del anillo
// Sondeo de diagnóstico (lib/mesh_ring/src/ring_diag.h): MeshRing no los procesa ni los repite, lo hace RingDiag
static constexpr uint8_t RING_TYPE_PING = 7, RING_TYPE_PONG = 8;

struct RingCmdAck {
  uint8_t devEui[8]; // collar que ejecutó el comando
  uint32_t id;
};

struct RingUpload {
  uint8_t devEui[8];
  uint32_t time; // hora UTC de la medición
  uint8_t frame[RING_FRAME_LEN];
  uint8_t hops;  // saltos que hizo hasta este puente
  int8_t rssi, snr;
};

class MeshRing {
public:
  struct Config {
    uint32_t measSlotMinMs = 6000;   // origen: MEDICIÓN entre estos tiempos de la ventana (después de los ENTREGADO)
    uint32_t measSlotMaxMs = 15000;
    uint32_t deliveredMaxMs = 3000;  // ENTREGADO pendientes: al principio de la ventana
    uint32_t claimMinMs = 500;       // puente: retardo del RECLAMO (mejor enlace LoRaWAN = menor)
    uint32_t claimMaxMs = 4000;
    uint32_t relayMinMs = 300;       // repetición: retardo (SNR bajo = menor)
    uint32_t relayMaxMs = 2500;
    uint32_t lastTxMs = 2000;        // nada nuevo en los últimos ms de la ventana (el paquete no alcanzaría)
    uint8_t maxAttempts = 3;         // ventanas por medición propia
    uint32_t bridgePenaltyMs = 3600000; // un uplink confirmado sin ACK (propio o ajeno): no es puente este tiempo
    uint32_t cmdSlotMinMs = 3000;    // COMANDO: después de los ENTREGADO y antes de las MEDICIONES
    uint32_t cmdSlotMaxMs = 6000;
    uint8_t cmdMaxAttempts = 3;      // ventanas en que el puente repite un COMANDO sin COMANDO OK
    uint32_t timeGoodMinMs = 800;    // faro HORA de un collar con hora buena (red, GPS, mano, RTC)
    uint32_t timeGoodMaxMs = 2500;
    uint32_t timeMeshMinMs = 2600;   // ...y de uno con hora tomada de la mesh (después: el de hora buena gana)
    uint32_t timeMeshMaxMs = 3000;
    bool (*unixNow)(uint32_t &unix) = nullptr; // hora UTC actual para el faro (la placa)
  };
  static constexpr uint8_t TIME_Q_NONE = 0, TIME_Q_MESH = 1, TIME_Q_GOOD = 2;

  void begin(MeshtasticTx *tx, const uint8_t devEui[8], const Config &cfg, uint32_t seed);

  // Origen: medición que no llegó por LoRaWAN (false si la cola está llena)
  bool queueOwn(uint32_t time, const uint8_t frame[RING_FRAME_LEN]);
  // Medición propia confirmada por ENTREGADO (para marcarla en PENDIENTES.csv y no reenviarla)
  bool ownDelivered(uint32_t time) const;

  // Ventana. bridge = el collar puede subir por LoRaWAN; linkRssi = RSSI del último downlink (sin dato: 0);
  // timeQuality = TIME_Q_* de la hora de este collar (con hora manda el faro HORA)
  void windowStart(uint32_t nowMs, uint32_t windowMs, bool bridge, int16_t linkRssi, uint8_t timeQuality);
  // Faro HORA oído en la ventana (la placa decide si lo usa según la calidad de su propia hora)
  bool takeTime(uint32_t &unix, uint8_t &quality, uint8_t from[8]);
  // Sin ventana (collar sin hora escuchando en continuo): ¿es un faro HORA de este canal?
  bool parseTime(const uint8_t *pkt, size_t len, uint32_t &unix, uint8_t &quality, uint8_t from[8]) const;
  void windowEnd(uint32_t nowMs);
  bool windowOpen() const { return open_; }

  // Paquete recibido en la ventana (se copia: el buffer se puede reutilizar)
  void onPacket(const uint8_t *pkt, size_t len, int16_t rssi, int16_t snr, uint32_t nowMs);
  // Próximo paquete a transmitir ya; 0 = nada. what = descripción para el log
  size_t nextTx(uint32_t nowMs, uint8_t *out, size_t max, const char *&what);

  // Puente (después de la ventana)
  bool takeUpload(RingUpload &u);
  void uploadResult(const RingUpload &u, bool ack, uint32_t nowMs);
  // Uplink confirmado PROPIO: sin ACK el enlace no está sirviendo y el collar no es puente bridgePenaltyMs (salió de
  // cobertura o lo borraron de la red después del join: LBM sigue "unido" pero nada llega); con ACK vuelve a serlo
  void linkResult(bool ack, uint32_t nowMs);
  bool bridge() const { return bridge_; } // de la ventana actual: unido y sin penalización

  uint8_t ownPending() const;

  // Comandos por el anillo. Puente: encola un comando para otro collar (false si la cola está llena o es muy largo).
  bool queueCommand(const uint8_t target[8], uint32_t id, const uint8_t *cmd, uint8_t len);
  // Destino: comando recibido para este collar (una vez por id); la placa lo ejecuta como un downlink
  bool takeCommand(uint8_t *cmd, uint8_t &len);
  // Puente: COMANDO OK oído de un comando que encoló este puente (para informar a la plataforma)
  bool takeCommandAck(RingCmdAck &a);

private:
  struct Key {
    uint8_t dev[8];
    uint32_t time;
    bool operator==(const Key &o) const { return time == o.time && memcmp(dev, o.dev, 8) == 0; }
  };
  enum KeyState : uint8_t { K_FREE, K_CLAIMED, K_DELIVERED, K_UPLOADED };
  struct KeyEntry {
    Key key;
    KeyState state;
    uint32_t ms;
  };
  struct Seen {
    uint32_t from, id, ms;
  };
  struct Own {
    bool used, delivered;
    uint32_t time;
    uint8_t frame[RING_FRAME_LEN];
    uint8_t attempts;
    uint32_t sendAt; // 0 = no en esta ventana
  };
  enum CandState : uint8_t { C_FREE, C_WAIT, C_CLAIMED, C_QUEUED, C_UPLOADING };
  struct Candidate {
    CandState state;
    RingUpload up;
    uint32_t claimAt;
  };
  struct Relay {
    bool used;
    uint32_t from, id, sendAt;
    uint8_t len;
    uint8_t pkt[MeshtasticTx::MAX_PACKET];
  };
  struct Announce {
    bool used;
    Key key;
    uint32_t sendAt; // 0 = en la próxima ventana
  };

  struct Cmd {             // puente: comando a difundir
    bool used, acked;
    Key key;               // destino + id
    uint8_t len, attempts;
    uint8_t bytes[RING_CMD_MAX];
    uint32_t sendAt;       // 0 = no en esta ventana
  };
  struct Exec {            // destino: ids ya ejecutados (no se repiten)
    uint32_t id, ms;
  };
  struct Inbox {           // destino: comando por ejecutar
    bool used;
    uint8_t len;
    uint8_t bytes[RING_CMD_MAX];
  };

  static constexpr uint8_t TYPE_MEAS = 1, TYPE_CLAIM = 2, TYPE_DELIVERED = 3, TYPE_CMD = 4, TYPE_CMD_OK = 5,
                           TYPE_TIME = 6, VERSION = RING_WIRE_VERSION; // 2: MEDICIÓN con la trama v2 (30 B)
  static constexpr uint8_t N_SEEN = 48, N_KEYS = 32, N_OWN = 8, N_CAND = 8, N_RELAY = 6, N_ANN = 8, N_CMD = 4,
                           N_EXEC = 16, N_INBOX = 2, N_CMDOK = 4;

  uint32_t rnd();
  uint32_t rndRange(uint32_t lo, uint32_t hi) { return hi > lo ? lo + rnd() % (hi - lo) : lo; }
  bool seenBefore(uint32_t from, uint32_t id, uint32_t nowMs);
  KeyEntry *findKey(const Key &k);
  void setKey(const Key &k, KeyState s, uint32_t nowMs);
  template <size_t N> static void schedule(Announce (&list)[N], const Key &k, uint32_t sendAt);
  void announce(const Key &k, uint32_t sendAt);
  void cancelRelay(uint32_t from, uint32_t id);
  void cancelCandidate(const Key &k);
  bool canSend(uint32_t at) const { return open_ && at + cfg_.lastTxMs < windowEndMs_; }
  size_t build(uint8_t type, const Key &k, const uint8_t *frame, uint8_t *out, size_t max, uint8_t extraLen = 0);
  void handle(const MeshRx &rx, uint8_t hops, int16_t rssi, int16_t snr, uint32_t nowMs);

  MeshtasticTx *tx_ = nullptr;
  Config cfg_;
  uint8_t dev_[8] = {};
  uint32_t rng_ = 1;
  bool open_ = false, bridge_ = false;
  int16_t linkRssi_ = 0;
  uint32_t windowStartMs_ = 0, windowEndMs_ = 0, penaltyMs_ = 0;
  bool penalty_ = false; // con bandera y tiempo transcurrido: una marca 0 dejaría de "vencer" pasados 24.8 días
  Seen seen_[N_SEEN] = {};
  uint8_t seenNext_ = 0;
  KeyEntry keys_[N_KEYS] = {};
  Own own_[N_OWN] = {};
  Candidate cand_[N_CAND] = {};
  Relay relay_[N_RELAY] = {};
  Announce ann_[N_ANN] = {};
  Cmd cmd_[N_CMD] = {};
  Exec exec_[N_EXEC] = {};
  uint8_t execNext_ = 0;
  Inbox inbox_[N_INBOX] = {};
  Announce cmdOk_[N_CMDOK] = {}; // destino: COMANDO OK por mandar (key = este collar + id)
  RingCmdAck acks_[N_CMDOK] = {}; // puente: confirmaciones por informar
  bool ackUsed_[N_CMDOK] = {};
  uint8_t timeQ_ = TIME_Q_NONE;     // calidad de la hora de este collar en esta ventana
  uint32_t timeAt_ = 0;             // faro HORA propio (0 = no en esta ventana)
  bool heardTime_ = false;          // faro oído, pendiente de takeTime()
  uint32_t heardUnix_ = 0;
  uint8_t heardQ_ = 0, heardFrom_[8] = {};
};

#endif
