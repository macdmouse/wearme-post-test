// mesh_ring.cpp
// Anillo de respaldo entre collares Wearme (ver mesh_ring.h).

#include "mesh_ring.h"
#include "debug_config.h"
#include <string.h>

static constexpr uint32_t SEEN_TTL_MS = 30UL * 60UL * 1000UL;      // paquetes vistos (origen + id)
static constexpr uint32_t CLAIM_TTL_MS = 60UL * 60UL * 1000UL;     // RECLAMO oído
static constexpr uint32_t DELIVERED_TTL_MS = 24UL * 3600UL * 1000UL; // ENTREGADO oído o subida propia

static bool due(uint32_t now, uint32_t at) {
  return (int32_t)(now - at) >= 0;
}

static void printDev(const uint8_t *d) {
  char hex[17];
  for (uint8_t i = 0; i < 8; i++) snprintf(hex + 2 * i, 3, "%02X", d[i]);
  DEBUG_PRINT(hex);
}

void MeshRing::begin(MeshtasticTx *tx, const uint8_t devEui[8], const Config &cfg, uint32_t seed) {
  tx_ = tx;
  memcpy(dev_, devEui, 8);
  cfg_ = cfg;
  rng_ = seed ? seed : 0x2545F491u;
}

uint32_t MeshRing::rnd() {
  uint32_t x = rng_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rng_ = x ? x : 1;
  return rng_;
}

// ---- Cachés ----
bool MeshRing::seenBefore(uint32_t from, uint32_t id, uint32_t nowMs) {
  for (const Seen &s : seen_) {
    if (s.ms && s.from == from && s.id == id && nowMs - s.ms < SEEN_TTL_MS) return true;
  }
  seen_[seenNext_] = {from, id, nowMs | 1};
  seenNext_ = (uint8_t)((seenNext_ + 1) % N_SEEN);
  return false;
}

MeshRing::KeyEntry *MeshRing::findKey(const Key &k) {
  const uint32_t now = millis();
  for (KeyEntry &e : keys_) {
    if (e.state == K_FREE || !(e.key == k)) continue;
    const uint32_t ttl = e.state == K_CLAIMED ? CLAIM_TTL_MS : DELIVERED_TTL_MS;
    if (now - e.ms >= ttl) {
      e.state = K_FREE;
      return nullptr;
    }
    return &e;
  }
  return nullptr;
}

void MeshRing::setKey(const Key &k, KeyState s, uint32_t nowMs) {
  KeyEntry *e = findKey(k);
  if (e) {
    // No se baja de categoría: ENTREGADO/SUBIDO pesan más que RECLAMADO
    if (s == K_CLAIMED && e->state != K_CLAIMED) return;
    e->state = s;
    e->ms = nowMs;
    return;
  }
  KeyEntry *slot = &keys_[0];
  for (KeyEntry &x : keys_) {
    if (x.state == K_FREE) {
      slot = &x;
      break;
    }
    if (x.ms - slot->ms > 0x80000000u) slot = &x; // la más vieja
  }
  *slot = {k, s, nowMs};
}

// Agenda un aviso (ENTREGADO o COMANDO OK) sin repetirlo; sendAt = 0 = en la próxima ventana
template <size_t N> void MeshRing::schedule(Announce (&list)[N], const Key &k, uint32_t sendAt) {
  for (Announce &a : list) {
    if (a.used && a.key == k) {
      if (a.sendAt == 0 || (sendAt && (int32_t)(sendAt - a.sendAt) < 0)) a.sendAt = sendAt;
      return;
    }
  }
  for (Announce &a : list) {
    if (!a.used) {
      a = {true, k, sendAt};
      return;
    }
  }
}

void MeshRing::announce(const Key &k, uint32_t sendAt) {
  schedule(ann_, k, sendAt);
}

void MeshRing::cancelRelay(uint32_t from, uint32_t id) {
  for (Relay &r : relay_) {
    if (r.used && r.from == from && r.id == id) r.used = false;
  }
}

void MeshRing::cancelCandidate(const Key &k) {
  for (Candidate &c : cand_) {
    if (c.state == C_FREE || c.state == C_UPLOADING) continue;
    if (memcmp(c.up.devEui, k.dev, 8) == 0 && c.up.time == k.time) c.state = C_FREE;
  }
}

// ---- Origen ----
bool MeshRing::queueOwn(uint32_t time, const uint8_t frame[RING_FRAME_LEN]) {
  for (Own &o : own_) {
    if (o.used) continue;
    o = {};
    o.used = true;
    o.time = time;
    memcpy(o.frame, frame, RING_FRAME_LEN);
    return true;
  }
  return false;
}

bool MeshRing::ownDelivered(uint32_t time) const {
  Key k;
  memcpy(k.dev, dev_, 8);
  k.time = time;
  for (const KeyEntry &e : keys_) {
    if (e.state == K_DELIVERED && e.key == k) return true;
  }
  for (const Own &o : own_) {
    if (o.used && o.delivered && o.time == time) return true;
  }
  return false;
}

uint8_t MeshRing::ownPending() const {
  uint8_t n = 0;
  for (const Own &o : own_) n += o.used && !o.delivered;
  return n;
}

// ---- Ventana ----
void MeshRing::windowStart(uint32_t nowMs, uint32_t windowMs, bool bridge, int16_t linkRssi, uint8_t timeQuality) {
  open_ = true;
  windowStartMs_ = nowMs;
  windowEndMs_ = nowMs + windowMs;
  // Faro HORA: primero los de hora buena; uno con hora de la mesh solo si no oyó ninguno antes
  timeQ_ = timeQuality;
  heardTime_ = false;
  timeAt_ = timeQuality == TIME_Q_GOOD   ? nowMs + rndRange(cfg_.timeGoodMinMs, cfg_.timeGoodMaxMs)
            : timeQuality == TIME_Q_MESH ? nowMs + rndRange(cfg_.timeMeshMinMs, cfg_.timeMeshMaxMs)
                                         : 0;
  if (penalty_ && nowMs - penaltyMs_ >= cfg_.bridgePenaltyMs) penalty_ = false;
  bridge_ = bridge && !penalty_;
  linkRssi_ = linkRssi;
  for (Relay &r : relay_) r.used = false;
  // Un RECLAMO vale solo en su ventana: si el origen reintenta en otra es porque ese puente no la entregó (sin ACK),
  // y cualquier otro puente la puede reclamar. ENTREGADO y SUBIDO sí se recuerdan (24 h).
  for (KeyEntry &e : keys_) {
    if (e.state == K_CLAIMED) e.state = K_FREE;
  }
  for (Announce &a : ann_) {
    if (a.used && a.sendAt == 0) a.sendAt = nowMs + rndRange(200, cfg_.deliveredMaxMs);
  }
  uint8_t i = 0;
  for (Own &o : own_) {
    if (!o.used) continue;
    if (o.delivered) {
      o.used = false;
      continue;
    }
    if (o.attempts >= cfg_.maxAttempts) {
      DEBUG_PRINT("[RING] Medicion propia sin ENTREGADO tras ");
      DEBUG_PRINT(o.attempts);
      DEBUG_PRINTLN(" ventanas: se deja (sigue en PENDIENTES.csv)");
      o.used = false;
      continue;
    }
    o.sendAt = nowMs + rndRange(cfg_.measSlotMinMs, cfg_.measSlotMaxMs) + 800UL * i++;
  }
  // Puente: COMANDOS sin confirmar (entre los ENTREGADO y las MEDICIONES)
  i = 0;
  for (Cmd &c : cmd_) {
    if (!c.used) continue;
    if (c.acked) {
      c.used = false;
      continue;
    }
    if (c.attempts >= cfg_.cmdMaxAttempts) {
      DEBUG_PRINT("[RING] COMANDO id ");
      DEBUG_PRINT(c.key.time);
      DEBUG_PRINTLN(" sin COMANDO OK tras sus intentos: se descarta (la plataforma puede reintentar por otro puente)");
      c.used = false;
      continue;
    }
    c.sendAt = nowMs + rndRange(cfg_.cmdSlotMinMs, cfg_.cmdSlotMaxMs) + 500UL * i++;
  }
  // Destino: COMANDO OK que no alcanzó a salir en la ventana anterior
  for (Announce &a : cmdOk_) {
    if (a.used && a.sendAt == 0) a.sendAt = nowMs + rndRange(200, cfg_.deliveredMaxMs);
  }
}

void MeshRing::windowEnd(uint32_t nowMs) {
  (void)nowMs;
  open_ = false;
  timeAt_ = 0;
  for (Relay &r : relay_) r.used = false;
  for (Own &o : own_) o.sendAt = 0;
  for (Announce &a : ann_) {
    if (a.used) a.sendAt = 0; // no alcanzó: en la próxima ventana
  }
  for (Announce &a : cmdOk_) {
    if (a.used) a.sendAt = 0;
  }
  for (Cmd &c : cmd_) c.sendAt = 0;
  // Lo reclamado (o que nadie reclamó antes de cerrar) se sube por LoRaWAN
  for (Candidate &c : cand_) {
    if (c.state == C_CLAIMED || (c.state == C_WAIT && bridge_)) c.state = C_QUEUED;
    else if (c.state == C_WAIT) c.state = C_FREE;
  }
}

void MeshRing::onPacket(const uint8_t *pkt, size_t len, int16_t rssi, int16_t snr, uint32_t nowMs) {
  if (!open_ || !tx_ || len > MeshtasticTx::MAX_PACKET) return;
  uint8_t dec[MeshtasticTx::MAX_PACKET];
  memcpy(dec, pkt, len);
  MeshRx rx;
  if (!tx_->parse(dec, len, rx) || rx.from == tx_->nodeNum()) return;
  // Ya visto: si teníamos su repetición pendiente, otro ya lo repitió (inundación controlada)
  if (seenBefore(rx.from, rx.id, nowMs)) {
    cancelRelay(rx.from, rx.id);
    return;
  }
  const bool wearme = rx.port == MeshtasticTx::PORT_PRIVATE && rx.payloadLen >= 16 && rx.payload[0] == 'W' &&
                      rx.payload[1] == 'M' && rx.payload[3] == VERSION;
  if (!wearme) return; // solo se repite el tráfico del anillo
  // PING/PONG del sondeo de diagnóstico: los atiende y repite RingDiag (si no, saldrían dos repeticiones)
  if (rx.payload[2] == RING_TYPE_PING || rx.payload[2] == RING_TYPE_PONG) return;
  // Faro HORA: no se repite (llegaría atrasado); cancela el propio si es de igual o mejor calidad
  if (rx.payload[2] == TYPE_TIME && rx.payloadLen >= 17) {
    heardTime_ = true;
    heardUnix_ = ((uint32_t)rx.payload[12] << 24) | ((uint32_t)rx.payload[13] << 16) |
                 ((uint32_t)rx.payload[14] << 8) | rx.payload[15];
    heardQ_ = rx.payload[16];
    memcpy(heardFrom_, rx.payload + 4, 8);
    if (timeAt_ && heardQ_ >= timeQ_) timeAt_ = 0;
    return;
  }
  const uint8_t hops = rx.hopStart >= rx.hopLimit ? (uint8_t)(rx.hopStart - rx.hopLimit) : 0;
  handle(rx, hops, rssi, snr, nowMs);

  // Repetición: SNR bajo (lejos) = antes. MEDICIÓN ya entregada no se repite (handle ya re-anunció ENTREGADO).
  bool relayIt = rx.hopLimit > 0;
  if (relayIt && rx.payload[2] == TYPE_MEAS) {
    Key k;
    memcpy(k.dev, rx.payload + 4, 8);
    k.time = ((uint32_t)rx.payload[12] << 24) | ((uint32_t)rx.payload[13] << 16) | ((uint32_t)rx.payload[14] << 8) |
             rx.payload[15];
    const KeyEntry *e = findKey(k);
    if (e && (e->state == K_DELIVERED || e->state == K_UPLOADED)) relayIt = false;
  }
  if (!relayIt) return;
  for (Relay &r : relay_) {
    if (r.used) continue;
    memcpy(r.pkt, pkt, len);
    if (!tx_->relay(r.pkt, len)) return;
    const int16_t s = snr < -20 ? -20 : snr > 10 ? 10 : snr;
    r.used = true;
    r.from = rx.from;
    r.id = rx.id;
    r.len = (uint8_t)len;
    r.sendAt = nowMs + cfg_.relayMinMs + (uint32_t)(s + 20) * (cfg_.relayMaxMs - cfg_.relayMinMs) / 30 + rndRange(0, 300);
    return;
  }
}

void MeshRing::handle(const MeshRx &rx, uint8_t hops, int16_t rssi, int16_t snr, uint32_t nowMs) {
  const uint8_t *p = rx.payload;
  Key k;
  memcpy(k.dev, p + 4, 8);
  k.time = ((uint32_t)p[12] << 24) | ((uint32_t)p[13] << 16) | ((uint32_t)p[14] << 8) | p[15];
  const bool mine = memcmp(k.dev, dev_, 8) == 0;

  if (p[2] == TYPE_MEAS && rx.payloadLen >= 16 + RING_FRAME_LEN) {
    if (mine) return;
    DEBUG_PRINT("[RING] MEDICION de ");
    printDev(k.dev);
    DEBUG_PRINT(" hora ");
    DEBUG_PRINT(k.time);
    DEBUG_PRINT(" | ");
    DEBUG_PRINT(hops);
    DEBUG_PRINT(" saltos, RSSI ");
    DEBUG_PRINT(rssi);
    DEBUG_PRINT(" SNR ");
    DEBUG_PRINTLN(snr);
    const KeyEntry *e = findKey(k);
    if (e && (e->state == K_UPLOADED || e->state == K_DELIVERED)) {
      // El origen no se enteró: se le vuelve a avisar en esta ventana
      DEBUG_PRINTLN("[RING]   ya entregada: se re-anuncia ENTREGADO");
      announce(k, nowMs + rndRange(300, 1500));
      return;
    }
    if (e && e->state == K_CLAIMED) return; // otro puente la tiene
    if (!bridge_) return;
    for (const Candidate &c : cand_) {
      if (c.state != C_FREE && c.up.time == k.time && memcmp(c.up.devEui, k.dev, 8) == 0) return;
    }
    for (Candidate &c : cand_) {
      if (c.state != C_FREE) continue;
      c.state = C_WAIT;
      memcpy(c.up.devEui, k.dev, 8);
      c.up.time = k.time;
      memcpy(c.up.frame, p + 16, RING_FRAME_LEN);
      c.up.hops = hops;
      c.up.rssi = (int8_t)(rssi < -128 ? -128 : rssi > 127 ? 127 : rssi);
      c.up.snr = (int8_t)(snr < -128 ? -128 : snr > 127 ? 127 : snr);
      // Mejor enlace LoRaWAN (RSSI del último downlink) = reclama antes
      const int16_t link = linkRssi_ == 0 ? -110 : (linkRssi_ < -130 ? -130 : linkRssi_ > -60 ? -60 : linkRssi_);
      const uint32_t span = cfg_.claimMaxMs - cfg_.claimMinMs;
      c.claimAt = nowMs + cfg_.claimMinMs + (uint32_t)(-60 - link) * span / 70 + rndRange(0, 400);
      DEBUG_PRINTLN("[RING]   puente: se reclamara si nadie la reclama antes");
      return;
    }
    return;
  }

  if (p[2] == TYPE_CLAIM && rx.payloadLen >= 20) {
    uint32_t claimer = 0;
    memcpy(&claimer, p + 16, 4);
    setKey(k, K_CLAIMED, nowMs);
    for (Candidate &c : cand_) {
      if (c.state == C_FREE || c.up.time != k.time || memcmp(c.up.devEui, k.dev, 8) != 0) continue;
      // Otro la reclamó antes; si ya la reclamamos los dos, gana el número de nodo menor
      if (c.state == C_WAIT || (c.state == C_CLAIMED && claimer < tx_->nodeNum())) {
        c.state = C_FREE;
        DEBUG_PRINT("[RING] RECLAMO de !");
        DEBUG_PRINT(String(claimer, HEX));
        DEBUG_PRINTLN(": este puente desiste");
      }
    }
    return;
  }

  if (p[2] == TYPE_CMD && rx.payloadLen >= 17 && p[16] <= RING_CMD_MAX && rx.payloadLen >= 17u + p[16]) {
    if (!mine) return; // es para otro collar: solo se repite
    const uint32_t id = k.time;
    bool done = false;
    for (const Exec &e : exec_) done |= e.ms && e.id == id && nowMs - e.ms < DELIVERED_TTL_MS;
    if (!done) {
      exec_[execNext_] = {id, nowMs | 1};
      execNext_ = (uint8_t)((execNext_ + 1) % N_EXEC);
      for (Inbox &in : inbox_) {
        if (in.used) continue;
        in.used = true;
        in.len = p[16];
        memcpy(in.bytes, p + 17, in.len);
        break;
      }
    }
    DEBUG_PRINT("[RING] COMANDO id ");
    DEBUG_PRINT(id);
    DEBUG_PRINT(" para este collar (");
    DEBUG_PRINT(hops);
    DEBUG_PRINTLN(done ? " saltos): ya ejecutado, solo se re-confirma" : " saltos): se ejecuta y se confirma");
    schedule(cmdOk_, k, nowMs + rndRange(300, 1500));
    return;
  }

  if (p[2] == TYPE_CMD_OK) {
    for (Cmd &c : cmd_) {
      if (!c.used || c.acked || !(c.key == k)) continue;
      c.acked = true;
      for (uint8_t i = 0; i < N_CMDOK; i++) {
        if (ackUsed_[i]) continue;
        ackUsed_[i] = true;
        memcpy(acks_[i].devEui, k.dev, 8);
        acks_[i].id = k.time;
        break;
      }
      DEBUG_PRINT("[RING] COMANDO OK de ");
      printDev(k.dev);
      DEBUG_PRINT(" id ");
      DEBUG_PRINTLN(k.time);
    }
    return;
  }

  if (p[2] == TYPE_DELIVERED) {
    KeyEntry *e = findKey(k);
    if (!e || e->state != K_UPLOADED) setKey(k, K_DELIVERED, nowMs);
    cancelCandidate(k);
    if (mine) {
      for (Own &o : own_) {
        if (o.used && o.time == k.time) o.delivered = true;
      }
      DEBUG_PRINT("[RING] ENTREGADO: la medicion propia de hora ");
      DEBUG_PRINT(k.time);
      DEBUG_PRINTLN(" llego por LoRaWAN a traves de otro collar");
    }
  }
}

size_t MeshRing::build(uint8_t type, const Key &k, const uint8_t *frame, uint8_t *out, size_t max, uint8_t extraLen) {
  uint8_t p[16 + RING_FRAME_LEN]; // la MEDICIÓN es el mensaje más largo (un COMANDO usa 1 + RING_CMD_MAX)
  p[0] = 'W';
  p[1] = 'M';
  p[2] = type;
  p[3] = VERSION;
  memcpy(p + 4, k.dev, 8);
  p[12] = (uint8_t)(k.time >> 24);
  p[13] = (uint8_t)(k.time >> 16);
  p[14] = (uint8_t)(k.time >> 8);
  p[15] = (uint8_t)k.time;
  size_t n = 16;
  if (type == TYPE_MEAS) {
    memcpy(p + 16, frame, RING_FRAME_LEN);
    n += RING_FRAME_LEN;
  } else if (type == TYPE_CLAIM) {
    const uint32_t node = tx_->nodeNum();
    memcpy(p + 16, &node, 4);
    n += 4;
  } else if (type == TYPE_CMD) {
    p[16] = extraLen;
    memcpy(p + 17, frame, extraLen);
    n += 1 + extraLen;
  } else if (type == TYPE_TIME) {
    p[16] = extraLen; // calidad de la hora
    n += 1;
  }
  return tx_->app(p, n, out, max);
}

bool MeshRing::takeTime(uint32_t &unix, uint8_t &quality, uint8_t from[8]) {
  if (!heardTime_) return false;
  heardTime_ = false;
  unix = heardUnix_;
  quality = heardQ_;
  memcpy(from, heardFrom_, 8);
  return true;
}

bool MeshRing::parseTime(const uint8_t *pkt, size_t len, uint32_t &unix, uint8_t &quality, uint8_t from[8]) const {
  if (!tx_ || len > MeshtasticTx::MAX_PACKET) return false;
  uint8_t dec[MeshtasticTx::MAX_PACKET];
  memcpy(dec, pkt, len);
  MeshRx rx;
  if (!tx_->parse(dec, len, rx) || rx.port != MeshtasticTx::PORT_PRIVATE || rx.payloadLen < 17) return false;
  const uint8_t *p = rx.payload;
  if (p[0] != 'W' || p[1] != 'M' || p[2] != TYPE_TIME || p[3] != VERSION) return false;
  unix = ((uint32_t)p[12] << 24) | ((uint32_t)p[13] << 16) | ((uint32_t)p[14] << 8) | p[15];
  quality = p[16];
  memcpy(from, p + 4, 8);
  return true;
}

size_t MeshRing::nextTx(uint32_t nowMs, uint8_t *out, size_t max, const char *&what) {
  if (!canSend(nowMs)) return 0;
  // El pendiente más antiguo que ya venció
  int kind = -1, idx = -1;
  uint32_t best = 0;
  auto consider = [&](int k, int i, uint32_t at) {
    if (!due(nowMs, at)) return;
    if (kind < 0 || (int32_t)(at - best) < 0) {
      kind = k;
      idx = i;
      best = at;
    }
  };
  for (int i = 0; i < N_ANN; i++)
    if (ann_[i].used && ann_[i].sendAt) consider(0, i, ann_[i].sendAt);
  for (int i = 0; i < N_RELAY; i++)
    if (relay_[i].used) consider(1, i, relay_[i].sendAt);
  for (int i = 0; i < N_CAND; i++)
    if (cand_[i].state == C_WAIT) consider(2, i, cand_[i].claimAt);
  for (int i = 0; i < N_OWN; i++)
    if (own_[i].used && !own_[i].delivered && own_[i].sendAt) consider(3, i, own_[i].sendAt);
  for (int i = 0; i < N_CMD; i++)
    if (cmd_[i].used && !cmd_[i].acked && cmd_[i].sendAt) consider(4, i, cmd_[i].sendAt);
  for (int i = 0; i < N_CMDOK; i++)
    if (cmdOk_[i].used && cmdOk_[i].sendAt) consider(5, i, cmdOk_[i].sendAt);
  if (timeAt_) consider(6, 0, timeAt_);
  if (kind < 0) return 0;

  Key k;
  switch (kind) {
    case 0: {
      Announce &a = ann_[idx];
      a.used = false;
      what = "ENTREGADO";
      return build(TYPE_DELIVERED, a.key, nullptr, out, max);
    }
    case 1: {
      Relay &r = relay_[idx];
      r.used = false;
      if (r.len > max) return 0;
      memcpy(out, r.pkt, r.len);
      what = "repeticion";
      return r.len;
    }
    case 2: {
      Candidate &c = cand_[idx];
      c.state = C_CLAIMED;
      memcpy(k.dev, c.up.devEui, 8);
      k.time = c.up.time;
      what = "RECLAMO";
      return build(TYPE_CLAIM, k, nullptr, out, max);
    }
    case 4: {
      Cmd &c = cmd_[idx];
      c.sendAt = 0;
      c.attempts++;
      what = "COMANDO";
      return build(TYPE_CMD, c.key, c.bytes, out, max, c.len);
    }
    case 5: {
      Announce &a = cmdOk_[idx];
      a.used = false;
      what = "COMANDO OK";
      return build(TYPE_CMD_OK, a.key, nullptr, out, max);
    }
    case 6: {
      timeAt_ = 0;
      uint32_t unix = 0;
      if (!cfg_.unixNow || !cfg_.unixNow(unix)) return 0;
      memcpy(k.dev, dev_, 8);
      k.time = unix;
      what = "HORA";
      return build(TYPE_TIME, k, nullptr, out, max, timeQ_);
    }
    default: {
      Own &o = own_[idx];
      o.sendAt = 0;
      o.attempts++;
      memcpy(k.dev, dev_, 8);
      k.time = o.time;
      what = "MEDICION propia";
      return build(TYPE_MEAS, k, o.frame, out, max);
    }
  }
}

// ---- Comandos ----
bool MeshRing::queueCommand(const uint8_t target[8], uint32_t id, const uint8_t *cmd, uint8_t len) {
  if (len == 0 || len > RING_CMD_MAX) return false;
  Key k;
  memcpy(k.dev, target, 8);
  k.time = id;
  for (const Cmd &c : cmd_) {
    if (c.used && c.key == k) return true; // ya en cola (p. ej. el mismo downlink repetido)
  }
  for (Cmd &c : cmd_) {
    if (c.used) continue;
    c = {};
    c.used = true;
    c.key = k;
    c.len = len;
    memcpy(c.bytes, cmd, len);
    if (open_) c.sendAt = millis() + rndRange(300, 1500); // ventana abierta: ya
    return true;
  }
  return false;
}

bool MeshRing::takeCommand(uint8_t *cmd, uint8_t &len) {
  for (Inbox &in : inbox_) {
    if (!in.used) continue;
    in.used = false;
    len = in.len;
    memcpy(cmd, in.bytes, len);
    return true;
  }
  return false;
}

bool MeshRing::takeCommandAck(RingCmdAck &a) {
  for (uint8_t i = 0; i < N_CMDOK; i++) {
    if (!ackUsed_[i]) continue;
    ackUsed_[i] = false;
    a = acks_[i];
    return true;
  }
  return false;
}

// ---- Puente ----
bool MeshRing::takeUpload(RingUpload &u) {
  for (Candidate &c : cand_) {
    if (c.state != C_QUEUED) continue;
    c.state = C_UPLOADING;
    u = c.up;
    return true;
  }
  return false;
}

void MeshRing::uploadResult(const RingUpload &u, bool ack, uint32_t nowMs) {
  Key k;
  memcpy(k.dev, u.devEui, 8);
  k.time = u.time;
  for (Candidate &c : cand_) {
    if (c.state == C_UPLOADING && c.up.time == u.time && memcmp(c.up.devEui, u.devEui, 8) == 0) c.state = C_FREE;
  }
  if (ack) {
    setKey(k, K_UPLOADED, nowMs);
    announce(k, 0); // ENTREGADO al principio de la próxima ventana
    return;
  }
  penalty_ = true; // el enlace LoRaWAN no está sirviendo: que suba otro
  penaltyMs_ = nowMs;
}

void MeshRing::linkResult(bool ack, uint32_t nowMs) {
  if (ack) {
    if (penalty_) DEBUG_PRINTLN("[RING] Uplink propio con ACK: el collar vuelve a ser puente");
    penalty_ = false;
    return;
  }
  if (!penalty_) {
    DEBUG_PRINT("[RING] Uplink propio sin ACK: no actua como puente por ");
    DEBUG_PRINT(cfg_.bridgePenaltyMs / 60000UL);
    DEBUG_PRINTLN(" min (sus mediciones van por el anillo)");
  }
  penalty_ = true;
  penaltyMs_ = nowMs;
}
