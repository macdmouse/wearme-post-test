// ring_diag.cpp
// Diagnóstico del anillo Meshtastic (ver ring_diag.h).

// Solo imprime y lleva una tabla: optimizado por tamaño (el resto del firmware va con -Ofast y la flash está justa)
#pragma GCC optimize("Os")

#include "ring_diag.h"
#include "mesh_ring.h" // RING_TYPE_*, RING_WIRE_VERSION
#include "debug_config.h"
#include <string.h>

static constexpr uint32_t SESSION_STALE_MS = 90000UL; // en sesión: sin oírlo directo este tiempo = solo por saltos

static const char *ringTypeName(uint8_t t) {
  switch (t) {
    case 1: return "MEDICION";
    case 2: return "RECLAMO";
    case 3: return "ENTREGADO";
    case 4: return "COMANDO";
    case 5: return "COMANDO OK";
    case 6: return "HORA";
    case RING_TYPE_PING: return "PING";
    case RING_TYPE_PONG: return "PONG";
    default: return "anillo ?";
  }
}

static const char *portName(uint16_t port) {
  switch (port) {
    case 1: return "TEXTO";
    case 3: return "POSICION";
    case 4: return "NODEINFO";
    case 67: return "TELEMETRIA";
    case 70: return "TRACEROUTE";
    case 256: return "PRIVADO";
    default: return "Meshtastic";
  }
}

static uint32_t be32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static void fmtAge(uint32_t ms, char *out, size_t size) {
  const uint32_t s = ms / 1000;
  if (s < 120) snprintf(out, size, "%lus", (unsigned long)s);
  else if (s < 7200) snprintf(out, size, "%lum", (unsigned long)(s / 60));
  else snprintf(out, size, "%luh", (unsigned long)(s / 3600));
}

static void fmtDelta(int d, char *out, size_t size) {
  if (d > 0) snprintf(out, size, "sube %d", d);
  else if (d < 0) snprintf(out, size, "baja %d", -d);
  else snprintf(out, size, "igual");
}

static void copyName(char *dst, const uint8_t *src, size_t n) {
  size_t k = 0;
  for (size_t i = 0; i < n && k < 5; i++) {
    const char c = (char)src[i];
    if (c == '\0') break;
    if (c >= 0x20 && c < 0x7F) dst[k++] = c;
  }
  dst[k] = '\0';
}

// short_name (campo 3) del User de un NodeInfo; false si no está
static bool nodeInfoShortName(const uint8_t *p, size_t len, char *out) {
  const uint8_t *end = p + len;
  while (p < end) {
    uint32_t key = 0, v = 0;
    for (uint8_t sh = 0;; sh += 7) {
      if (p >= end || sh > 28) return false;
      const uint8_t b = *p++;
      key |= (uint32_t)(b & 0x7F) << sh;
      if (!(b & 0x80)) break;
    }
    const uint8_t wire = key & 7;
    if (wire == 0) {
      while (p < end && (*p++ & 0x80)) {
      }
    } else if (wire == 2) {
      for (uint8_t sh = 0;; sh += 7) {
        if (p >= end || sh > 28) return false;
        const uint8_t b = *p++;
        v |= (uint32_t)(b & 0x7F) << sh;
        if (!(b & 0x80)) break;
      }
      if (v > (uint32_t)(end - p)) return false;
      if ((key >> 3) == 3) {
        copyName(out, p, v);
        return out[0] != '\0';
      }
      p += v;
    } else if (wire == 5) {
      p += 4;
    } else if (wire == 1) {
      p += 8;
    } else {
      return false;
    }
  }
  return false;
}

void RingDiag::begin(MeshtasticTx *tx, const uint8_t devEui[8], const char *shortName, const Config &cfg, uint32_t seed) {
  tx_ = tx;
  memcpy(dev_, devEui, 8);
  copyName(name_, (const uint8_t *)shortName, strlen(shortName));
  cfg_ = cfg;
  rng_ = seed ? seed : 0x1F123BB5u;
}

uint32_t RingDiag::rnd() {
  uint32_t x = rng_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  rng_ = x ? x : 1;
  return rng_;
}

bool RingDiag::seenBefore(uint32_t from, uint32_t id) {
  for (const Seen &s : seen_) {
    if (s.from == from && s.id == id && (from | id)) return true;
  }
  seen_[seenNext_] = {from, id};
  seenNext_ = (uint8_t)((seenNext_ + 1) % N_SEEN);
  return false;
}

void RingDiag::cancelRelay(uint32_t from, uint32_t id) {
  for (Relay &r : relay_) {
    if (r.used && r.from == from && r.id == id) r.used = false;
  }
}

RingDiag::Node *RingDiag::find(uint32_t num) {
  for (Node &n : nodes_) {
    if (n.num == num) return &n;
  }
  return nullptr;
}

RingDiag::Node *RingDiag::add(uint32_t num, uint32_t nowMs) {
  Node *slot = nullptr;
  for (Node &n : nodes_) {
    if (n.num == 0) {
      slot = &n;
      break;
    }
    if (!slot || nowMs - n.heardMs > nowMs - slot->heardMs) slot = &n; // el que hace más que no se oye
  }
  *slot = {};
  slot->num = num;
  return slot;
}

// Nodo cuyo byte bajo es b (relay_node de la cabecera); con varios, el oído directo más reciente
const RingDiag::Node *RingDiag::byRelay(uint8_t b) const {
  const Node *best = nullptr;
  for (const Node &n : nodes_) {
    if (n.num == 0 || (uint8_t)n.num != b) continue;
    if (!best || (n.directMs && (!best->directMs || (int32_t)(n.directMs - best->directMs) > 0))) best = &n;
  }
  return best;
}

void RingDiag::label(const Node *n, uint32_t num, char *out, size_t size) const {
  if (n && n->name[0]) snprintf(out, size, "%s !%08lx", n->name, (unsigned long)num);
  else snprintf(out, size, "!%08lx", (unsigned long)num);
}

void RingDiag::viaLabel(uint8_t b, char *out, size_t size) const {
  const Node *n = byRelay(b);
  if (n && n->name[0]) snprintf(out, size, "%s", n->name);
  else if (n) snprintf(out, size, "!%08lx", (unsigned long)n->num);
  else snprintf(out, size, "?..%02x", b); // solo se conoce el byte bajo del número de nodo
}

const char *RingDiag::state(const Node &n, uint32_t nowMs) const {
  static const char *names[] = {"CERCA", "BUENO", "DEBIL", "LIMITE", "SALTOS", "CALLADO", "FUERA", "?"};
  return names[rank(n, nowMs)];
}

// 0 CERCA, 1 BUENO, 2 DEBIL, 3 LIMITE (a punto de perderse), 4 SALTOS (se oye, pero solo por un repetidor),
// 5 CALLADO (no se oye hace rato; aún no FUERA), 6 FUERA (de rango), 7 sin datos.
// LongFast (SF11) decodifica hasta SNR ~ -17.5 dB: por debajo de -12 dB el enlace está al borde.
uint8_t RingDiag::rank(const Node &n, uint32_t nowMs) const {
  if (n.lost) return 6;
  const uint32_t stale = active() ? SESSION_STALE_MS : cfg_.lostIdleMs;
  if (n.directMs && nowMs - n.directMs < stale) {
    if (n.rssi >= -85 && n.snr >= 5) return 0;
    if (n.rssi >= -105 && n.snr >= -3) return 1;
    if (n.snr >= -12) return 2;
    return 3;
  }
  if (n.heardMs && nowMs - n.heardMs < stale) return 4;
  if (n.heardMs) return 5;
  return 7;
}

void RingDiag::heardFrom(Node &n, int16_t rssi, int16_t snr, uint32_t nowMs) {
  if (n.nDirect == 0) {
    n.rssiMin = n.rssiMax = rssi;
    n.avg16 = (int32_t)rssi * 16;
  } else {
    if (rssi < n.rssiMin) n.rssiMin = rssi;
    if (rssi > n.rssiMax) n.rssiMax = rssi;
    n.avg16 += ((int32_t)rssi * 16 - n.avg16) / 4; // promedio móvil (1/4): sigue el recorrido sin saltar con cada paquete
  }
  n.rssi = rssi;
  n.snr = (int8_t)(snr < -128 ? -128 : snr > 127 ? 127 : snr);
  if (n.nDirect < 0xFFFF) n.nDirect++;
  n.directMs = nowMs | 1;
  n.heardMs = nowMs | 1;
  if (n.lost) {
    n.lost = false;
    n.miss = 0;
    char who[24];
    label(&n, n.num, who, sizeof(who));
    DEBUG_PRINT("[DIAG] ");
    DEBUG_PRINT(who);
    DEBUG_PRINT(" DE NUEVO EN ALCANCE: RSSI ");
    DEBUG_PRINT(rssi);
    DEBUG_PRINT(" dBm SNR ");
    DEBUG_PRINT(snr);
    DEBUG_PRINTLN(" dB");
    logRow("VUELVE", n.num, n.name, "", ND, rssi, snr, "");
  }
}

static const char *istr(char *b, int v) {
  if (v == -32768) b[0] = '\0';
  else snprintf(b, 8, "%d", v);
  return b;
}

void RingDiag::logRow(const char *ev, uint32_t num, const char *name, const char *via, int hops, int rssi, int snr,
                      const char *st, int upRssi, int upSnr, int upHops) const {
  if (!cfg_.log) return;
  char row[160], node[12], a[8], b[8], c[8], d[8], e[8], f[8];
  if (num) snprintf(node, sizeof(node), "!%08lx", (unsigned long)num);
  else node[0] = '\0';
  snprintf(row, sizeof(row), "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s", ev, node, name, via, istr(a, hops), istr(b, rssi),
           istr(c, snr), st, istr(d, upRssi), istr(e, upSnr), istr(f, upHops));
  cfg_.log(row);
}

uint8_t RingDiag::count() const {
  uint8_t c = 0;
  for (const Node &n : nodes_) c += n.num != 0;
  return c;
}

void RingDiag::reset() {
  for (Node &n : nodes_) n = {};
  DEBUG_PRINTLN("[DIAG] Tabla de vecinos borrada");
}

// ---- Sesión ----
void RingDiag::start(uint16_t minutes, uint32_t nowMs) {
  if (minutes == 0) {
    if (initiator_) {
      stopping_ = true; // el próximo PING sale con 0 minutos: los que responden dejan de escuchar
      endMs_ = nowMs;
      DEBUG_PRINTLN("[DIAG] Fin del sondeo: se avisa a los que responden en el proximo PING");
    } else if (responder_) {
      responder_ = false;
      DEBUG_PRINTLN("[DIAG] Este collar deja de responder el sondeo");
    } else {
      DEBUG_PRINTLN("[DIAG] No hay sondeo activo");
    }
    return;
  }
  if (!initiator_) {
    startMs_ = nowMs;
    for (Node &n : nodes_) {
      n.inSession = n.ponged = false;
      n.pings = n.pongs = 0;
      n.miss = 0;
    }
    pingSent_ = false;
  }
  initiator_ = true;
  stopping_ = false;
  endMs_ = nowMs + minutes * 60000UL;
  DEBUG_PRINT("[DIAG] Sondeo del anillo por ");
  DEBUG_PRINT(minutes);
  DEBUG_PRINTLN(" min: PING en cada ciclo, los collares que lo oyen responden PONG con la senal con que lo oyeron");
}

void RingDiag::endSession(const char *why) {
  initiator_ = false;
  stopping_ = false;
  pingAt_ = 0;
  DEBUG_PRINT("[DIAG] Sondeo terminado (");
  DEBUG_PRINT(why);
  DEBUG_PRINTLN(")");
}

void RingDiag::tick(uint32_t nowMs) {
  if (initiator_ && !stopping_ && (int32_t)(nowMs - endMs_) >= 0) {
    stopping_ = true;
    DEBUG_PRINTLN("[DIAG] Se cumplio el tiempo del sondeo: ultimo PING (fin) en el proximo ciclo");
  }
  // El PING de fin no pudo salir (sin escucha): se termina igual
  if (initiator_ && stopping_ && nowMs - endMs_ > 120000UL) endSession("sin poder mandar el PING de fin");
  if (responder_ && (int32_t)(nowMs - respEndMs_) >= 0) {
    responder_ = false;
    DEBUG_PRINTLN("[DIAG] Vencio el sondeo que respondia este collar: vuelve a escuchar solo en las ventanas");
  }
  // Sin sesión: un nodo que no se oye en lostIdleMs (2 ventanas) queda FUERA DE RANGO
  for (Node &n : nodes_) {
    if (n.num == 0 || n.lost || !n.heardMs || (n.inSession && initiator_)) continue; // en el sondeo: por PING sin PONG
    if (nowMs - n.heardMs < cfg_.lostIdleMs) continue;
    n.lost = true;
    char who[24], age[8];
    label(&n, n.num, who, sizeof(who));
    fmtAge(nowMs - n.heardMs, age, sizeof(age));
    DEBUG_PRINT("[DIAG] ");
    DEBUG_PRINT(who);
    DEBUG_PRINT(" FUERA DE RANGO: no se oye hace ");
    DEBUG_PRINT(age);
    if (n.nDirect) {
      DEBUG_PRINT(" (ultima senal directa RSSI ");
      DEBUG_PRINT(n.rssi);
      DEBUG_PRINT(" SNR ");
      DEBUG_PRINT(n.snr);
      DEBUG_PRINT(")");
    }
    DEBUG_PRINTLN("");
    logRow("FUERA", n.num, n.name, "", ND, ND, ND, "FUERA");
  }
}

void RingDiag::cycle(uint32_t nowMs, uint32_t pingDelayMs) {
  if (!initiator_) return;
  if (pingSent_) {
    // Resultado del PING anterior: quién respondió y quién no
    pingSent_ = false;
    char line[200], part[64], who[24];
    uint8_t asked = 0, answered = 0;
    size_t at = snprintf(line, sizeof(line), "[DIAG] PING #%lu:", (unsigned long)seq_);
    for (Node &n : nodes_) {
      if (n.num == 0 || !n.inSession) continue;
      asked++;
      label(&n, n.num, who, sizeof(who));
      if (n.name[0]) who[strlen(n.name)] = '\0'; // en la línea corta basta el nombre
      if (n.ponged) {
        answered++;
        n.miss = 0;
        if (n.downHops == 0) snprintf(part, sizeof(part), " | %s ida %d vuelta %d", who, n.upRssi, n.rssi);
        else snprintf(part, sizeof(part), " | %s ida %d vuelta %u saltos", who, n.upRssi, n.downHops);
      } else {
        if (n.miss < 255) n.miss++;
        snprintf(part, sizeof(part), " | %s SIN RESPUESTA x%u", who, n.miss);
        if (n.miss >= cfg_.lostPings && !n.lost) {
          n.lost = true;
          snprintf(part, sizeof(part), " | %s FUERA DE RANGO", who);
        }
        char ev[20];
        snprintf(ev, sizeof(ev), "SIN PONG #%lu", (unsigned long)seq_);
        logRow(ev, n.num, n.name, "", ND, ND, ND, n.lost ? "FUERA" : "");
      }
      n.ponged = false;
      if (at < sizeof(line) - 1) at += snprintf(line + at, sizeof(line) - at, "%s", part);
    }
    if (asked == 0) {
      DEBUG_PRINTLN(line);
      DEBUG_PRINTLN("[DIAG]   nadie ha respondido todavia (los collares empiezan a responder cuando oyen un PING en una ventana)");
    } else {
      DEBUG_PRINT(line);
      DEBUG_PRINT(" | responden ");
      DEBUG_PRINT(answered);
      DEBUG_PRINT(" de ");
      DEBUG_PRINTLN(asked);
    }
    if (cfg_.tableEvery && seq_ % cfg_.tableEvery == 0) printTable(nowMs);
  }
  pingAt_ = (nowMs + pingDelayMs) | 1;
}

void RingDiag::sliceEnd() {
  for (Relay &r : relay_) r.used = false;
  for (Pong &p : pong_) p.used = false;
  pingAt_ = 0;
}

// ---- Recepción ----
void RingDiag::onPacket(const uint8_t *pkt, size_t len, int16_t rssi, int16_t snr, uint32_t nowMs) {
  if (!tx_ || len < 16 || len > MeshtasticTx::MAX_PACKET) return;
  uint32_t from = 0, id = 0;
  memcpy(&from, pkt + 4, 4);
  memcpy(&id, pkt + 8, 4);
  const uint8_t hopLimit = pkt[12] & 0x07, hopStart = pkt[12] >> 5;
  const uint8_t hops = hopStart >= hopLimit ? (uint8_t)(hopStart - hopLimit) : 0;
  const uint8_t relayB = pkt[15];
  const uint32_t me = tx_->nodeNum();

  uint8_t dec[MeshtasticTx::MAX_PACKET];
  memcpy(dec, pkt, len);
  MeshRx rx;
  const bool ours = tx_->parse(dec, len, rx);
  const uint8_t *p = rx.payload;
  const bool wearme = ours && rx.port == MeshtasticTx::PORT_PRIVATE && rx.payloadLen >= 16 && p[0] == 'W' &&
                      p[1] == 'M' && p[3] == RING_WIRE_VERSION;
  const uint8_t type = wearme ? p[2] : 0;
  char what[16];
  snprintf(what, sizeof(what), "%s", wearme ? ringTypeName(type) : ours ? portName(rx.port) : "?");
  if (type == RING_TYPE_PING || type == RING_TYPE_PONG) {
    snprintf(what, sizeof(what), "%s #%lu", ringTypeName(type), (unsigned long)be32(p + 12));
  }

  char via[16] = "", d[16] = "", line[240];
  if (!ours) {
    // Otro canal (o clave distinta): se ve la señal pero no el contenido
    snprintf(line, sizeof(line), "[DIAG] RX de otro canal o clave (hash 0x%02x) de !%08lx | RSSI %d SNR %d", pkt[13],
             (unsigned long)from, rssi, snr);
    DEBUG_PRINTLN(line);
    return;
  }

  // Paquete PROPIO que vuelve: otro nodo lo repitió, así que le llegó
  if (from == me) {
    viaLabel(relayB, via, sizeof(via));
    Node *r = const_cast<Node *>(byRelay(relayB));
    if (r) {
      if (r->echoes < 0xFFFF) r->echoes++;
      heardFrom(*r, rssi, snr, nowMs);
    }
    snprintf(line, sizeof(line), "[DIAG] ECO: mi %s repetido por %s -> le llego | RSSI %d SNR %d", what, via, rssi, snr);
    DEBUG_PRINTLN(line);
    char ev[24];
    snprintf(ev, sizeof(ev), "ECO %s", what);
    logRow(ev, me, name_, via, hops, rssi, snr, r ? state(*r, nowMs) : "");
    return;
  }

  Node *o = find(from);
  if (!o) o = add(from, nowMs);
  o->heardMs = nowMs | 1;
  o->hops = hops;
  o->via = hops ? relayB : 0;
  if (hops && o->nRelayed < 0xFFFF) o->nRelayed++;
  if (wearme) o->wearme = true;
  if (type == RING_TYPE_PING && rx.payloadLen >= 21) copyName(o->name, p + 17, 4);
  if (type == RING_TYPE_PONG && rx.payloadLen >= 28) copyName(o->name, p + 24, 4);
  if (rx.port == MeshtasticTx::PORT_NODEINFO) {
    char sn[6] = "";
    if (nodeInfoShortName(p, rx.payloadLen, sn)) memcpy(o->name, sn, sizeof(sn));
  }

  // Quién transmitió lo que se oyó: el origen (directo) o el último que lo repitió
  Node *t = hops == 0 ? o : const_cast<Node *>(byRelay(relayB));
  const bool hadPrev = t && t->nDirect > 0;
  const int delta = hadPrev ? rssi - t->rssi : 0;
  if (t) heardFrom(*t, rssi, snr, nowMs);

  char who[24], route[56];
  label(o, from, who, sizeof(who));
  if (hops == 0) {
    snprintf(route, sizeof(route), "directo | RSSI");
  } else {
    viaLabel(relayB, via, sizeof(via));
    snprintf(route, sizeof(route), "por %s (%u salto%s) | ultimo salto RSSI", via, hops, hops == 1 ? "" : "s");
  }
  if (hadPrev) {
    char dd[12];
    fmtDelta(delta, dd, sizeof(dd));
    snprintf(d, sizeof(d), " (%s)", dd);
  }
  snprintf(line, sizeof(line), "[DIAG] RX %s de %s %s %d dBm SNR %d dB%s%s%s", what, who, route, rssi, snr, d,
           t ? " | " : "", t ? state(*t, nowMs) : "");
  DEBUG_PRINTLN(line);
  {
    // Fila de la SD; en un PONG para este collar, con cómo oyó él el PING (ida)
    int upR = ND, upS = ND, upH = ND;
    if (type == RING_TYPE_PONG && rx.payloadLen >= 28) {
      uint32_t pinger = 0;
      memcpy(&pinger, p + 16, 4);
      if (pinger == me) {
        upR = (int8_t)p[20];
        upS = (int8_t)p[21];
        upH = p[22];
      }
    }
    char ev[24];
    snprintf(ev, sizeof(ev), "RX %s", what);
    logRow(ev, from, o->name, hops ? via : "directo", hops, rssi, snr, t ? state(*t, nowMs) : "", upR, upS, upH);
  }

  if (type != RING_TYPE_PING && type != RING_TYPE_PONG) {
    seenBefore(from, id);
    return;
  }

  // PING / PONG: una sola vez por paquete (from + id); si se oye otra vez, otro ya lo repitió
  if (seenBefore(from, id)) {
    cancelRelay(from, id);
    return;
  }
  const uint32_t seq = be32(p + 12);
  if (type == RING_TYPE_PING && rx.payloadLen >= 21) {
    const uint8_t minutes = p[16];
    if (minutes == 0) {
      if (responder_ && lastPinger_ == from) {
        responder_ = false;
        DEBUG_PRINT("[DIAG] ");
        DEBUG_PRINT(who);
        DEBUG_PRINTLN(" termino el sondeo: este collar vuelve a escuchar solo en las ventanas");
      }
    } else {
      if (!responder_ || lastPinger_ != from) {
        DEBUG_PRINT("[DIAG] Sondeo de ");
        DEBUG_PRINT(who);
        DEBUG_PRINT(": este collar responde PONG y escucha fuera de las ventanas por ");
        DEBUG_PRINT(minutes);
        DEBUG_PRINTLN(" min");
      }
      responder_ = true;
      lastPinger_ = from;
      respEndMs_ = nowMs + minutes * 60000UL + 60000UL;
      for (Pong &q : pong_) {
        if (q.used) continue;
        q = {true, from, seq, (nowMs + rndRange(cfg_.pongMinMs, cfg_.pongMaxMs)) | 1,
             (int8_t)(rssi < -128 ? -128 : rssi > 127 ? 127 : rssi), (int8_t)(snr < -128 ? -128 : snr > 127 ? 127 : snr),
             hops, (uint8_t)(hops ? relayB : 0)};
        break;
      }
    }
  } else if (type == RING_TYPE_PONG && rx.payloadLen >= 28) {
    uint32_t pinger = 0;
    memcpy(&pinger, p + 16, 4);
    if (pinger == me) {
      o->upRssi = (int8_t)p[20];
      o->upSnr = (int8_t)p[21];
      o->upHops = p[22];
      o->upMs = nowMs | 1;
      o->downHops = hops;
      if (initiator_ && seq == seq_ && !o->ponged) {
        o->ponged = true;
        if (!o->inSession) {
          o->inSession = true;
          o->pings = 1;
          o->pongs = 0;
          o->miss = 0;
        }
        o->pongs++;
        if (o->lost) {
          o->lost = false;
          DEBUG_PRINT("[DIAG] ");
          DEBUG_PRINT(who);
          DEBUG_PRINTLN(" DE NUEVO EN ALCANCE (respondio el PING)");
        }
      }
      // Enlace en los dos sentidos y la ruta de cada uno
      char ida[40], vuelta[48], rtt[32] = "";
      if (p[22] == 0) {
        snprintf(ida, sizeof(ida), "(directo)");
      } else {
        viaLabel(p[23], via, sizeof(via));
        snprintf(ida, sizeof(ida), "(%u saltos, ultimo %s)", p[22], via);
      }
      if (hops == 0) {
        snprintf(vuelta, sizeof(vuelta), "RSSI %d SNR %d (directo)", rssi, snr);
      } else {
        viaLabel(relayB, via, sizeof(via));
        snprintf(vuelta, sizeof(vuelta), "%u saltos, ultimo %s", hops, via);
      }
      if (seq == seq_ && pingMs_) {
        const uint32_t ms = nowMs - pingMs_;
        snprintf(rtt, sizeof(rtt), " | respuesta en %lu.%lu s", (unsigned long)(ms / 1000), (unsigned long)(ms / 100 % 10));
      }
      snprintf(line, sizeof(line), "[DIAG]   enlace con %s: ida yo->el RSSI %d SNR %d %s | vuelta el->yo %s%s", who,
               o->upRssi, o->upSnr, ida, vuelta, rtt);
      DEBUG_PRINTLN(line);
    }
  }

  // Repetición de PING/PONG (SNR bajo = antes); se cancela si otro lo repite primero
  if (hopLimit == 0 || !active()) return;
  for (Relay &r : relay_) {
    if (r.used) continue;
    memcpy(r.pkt, pkt, len);
    if (!tx_->relay(r.pkt, len)) return;
    const int16_t s = snr < -20 ? -20 : snr > 10 ? 10 : snr;
    r.used = true;
    r.from = from;
    r.id = id;
    r.len = (uint8_t)len;
    r.sendAt = (nowMs + cfg_.relayMinMs + (uint32_t)(s + 20) * (cfg_.relayMaxMs - cfg_.relayMinMs) / 30 +
                rndRange(0, 300)) | 1;
    return;
  }
}

// ---- Transmisión ----
size_t RingDiag::build(uint8_t type, uint32_t number, const uint8_t *extra, uint8_t extraLen, uint8_t *out, size_t max) {
  uint8_t p[16 + 16];
  if (extraLen > 16) return 0;
  p[0] = 'W';
  p[1] = 'M';
  p[2] = type;
  p[3] = RING_WIRE_VERSION;
  memcpy(p + 4, dev_, 8);
  p[12] = (uint8_t)(number >> 24);
  p[13] = (uint8_t)(number >> 16);
  p[14] = (uint8_t)(number >> 8);
  p[15] = (uint8_t)number;
  memcpy(p + 16, extra, extraLen);
  return tx_->app(p, 16 + extraLen, out, max);
}

size_t RingDiag::nextTx(uint32_t nowMs, uint32_t untilMs, uint8_t *out, size_t max, const char *&what) {
  if (!tx_ || (int32_t)(untilMs - nowMs) < 1000) return 0; // un paquete LongFast corto tarda ~0.6 s
  auto due = [nowMs](uint32_t at) { return at && (int32_t)(nowMs - at) >= 0; };
  static char txLabel[24];

  // Repeticiones primero (alguien más lejos las espera), después los PONG y el PING
  for (Relay &r : relay_) {
    if (!r.used || !due(r.sendAt)) continue;
    r.used = false;
    if (r.len > max) return 0;
    memcpy(out, r.pkt, r.len);
    what = "repeticion (sondeo)";
    return r.len;
  }
  for (Pong &q : pong_) {
    if (!q.used || !due(q.sendAt)) continue;
    q.used = false;
    uint8_t e[12];
    memcpy(e, &q.pinger, 4);
    e[4] = (uint8_t)q.rssi;
    e[5] = (uint8_t)q.snr;
    e[6] = q.hops;
    e[7] = q.via;
    memset(e + 8, 0, 4);
    memcpy(e + 8, name_, strnlen(name_, 4));
    snprintf(txLabel, sizeof(txLabel), "PONG #%lu", (unsigned long)q.seq);
    what = txLabel;
    return build(RING_TYPE_PONG, q.seq, e, sizeof(e), out, max);
  }
  if (initiator_ && due(pingAt_)) {
    pingAt_ = 0;
    uint8_t e[5];
    const uint32_t left = stopping_ ? 0 : (endMs_ - nowMs + 59999UL) / 60000UL;
    e[0] = (uint8_t)(left > 255 ? 255 : left);
    memset(e + 1, 0, 4);
    memcpy(e + 1, name_, strnlen(name_, 4));
    seq_++;
    const size_t n = build(RING_TYPE_PING, seq_, e, sizeof(e), out, max);
    if (!n) return 0;
    pingMs_ = nowMs | 1;
    pingSent_ = true;
    for (Node &x : nodes_) {
      if (x.num && x.inSession && x.pings < 0xFFFF) x.pings++;
    }
    snprintf(txLabel, sizeof(txLabel), stopping_ ? "PING #%lu (fin)" : "PING #%lu", (unsigned long)seq_);
    what = txLabel;
    char ev[32];
    snprintf(ev, sizeof(ev), "TX %s", txLabel); // traza del recorrido (con el GPS) aunque nadie responda
    logRow(ev, tx_->nodeNum(), name_, "", ND, ND, ND, "");
    if (stopping_) {
      endSession("PING de fin enviado");
      printTable(nowMs);
    }
    return n;
  }
  return 0;
}

// ---- Tabla ----
void RingDiag::printTable(uint32_t nowMs) const {
  uint8_t idx[N_NODES], n = 0;
  for (uint8_t i = 0; i < N_NODES; i++) {
    if (nodes_[i].num) idx[n++] = i;
  }
  // Orden: estado (CERCA primero) y, dentro del estado, mejor RSSI primero
  for (uint8_t i = 1; i < n; i++) {
    const uint8_t k = idx[i];
    int j = i - 1;
    while (j >= 0) {
      const Node &a = nodes_[idx[j]], &b = nodes_[k];
      const uint8_t ra = rank(a, nowMs), rb = rank(b, nowMs);
      if (ra < rb || (ra == rb && a.rssi >= b.rssi)) break;
      idx[j + 1] = idx[j];
      j--;
    }
    idx[j + 1] = k;
  }

  char line[200], age[8], tmp[40];
  snprintf(line, sizeof(line), "[DIAG] ===== Vecinos del anillo | este collar %s !%08lx | ", name_,
           (unsigned long)(tx_ ? tx_->nodeNum() : 0));
  DEBUG_PRINT(line);
  if (initiator_) {
    DEBUG_PRINT("sondeando: PING #");
    DEBUG_PRINT(seq_);
    DEBUG_PRINT(", quedan ");
    DEBUG_PRINT(stopping_ ? 0 : (endMs_ - nowMs) / 60000UL);
    DEBUG_PRINTLN(" min =====");
  } else if (responder_) {
    DEBUG_PRINT("respondiendo el sondeo de !");
    DEBUG_PRINT(String(lastPinger_, HEX));
    DEBUG_PRINTLN(" =====");
  } else {
    DEBUG_PRINTLN("sin sondeo: lo oido en las ventanas =====");
  }
  if (n == 0) {
    DEBUG_PRINTLN("[DIAG] (ningun nodo oido todavia)");
    return;
  }
  DEBUG_PRINTLN("[DIAG] Nodo            Estado   RSSI  SNR  min/max  prom tend   | le llega (PONG/eco) | ruta del ultimo     | PONG       | oido");
  for (uint8_t i = 0; i < n; i++) {
    const Node &x = nodes_[idx[i]];
    size_t at = 0;
    char who[24];
    label(&x, x.num, who, sizeof(who));
    at += snprintf(line + at, sizeof(line) - at, "[DIAG] %-15s %-7s ", who, state(x, nowMs));
    if (x.nDirect) {
      const int avg = (int)(x.avg16 / 16);
      const int tr = x.rssi - avg;
      at += snprintf(line + at, sizeof(line) - at, "%5d %4d %4d/%-4d %4d %-6s | ", x.rssi, x.snr, x.rssiMin, x.rssiMax,
                     avg, tr >= 3 ? "sube" : tr <= -3 ? "baja" : "estable");
    } else {
      at += snprintf(line + at, sizeof(line) - at, "%5s %4s %9s %4s %-6s | ", "-", "-", "-", "-", "");
    }
    if (x.upMs) {
      fmtAge(nowMs - x.upMs, age, sizeof(age));
      snprintf(tmp, sizeof(tmp), "%d/%d hace %s", x.upRssi, x.upSnr, age);
    } else if (x.echoes) {
      snprintf(tmp, sizeof(tmp), "si (eco x%u)", x.echoes);
    } else {
      snprintf(tmp, sizeof(tmp), "?");
    }
    at += snprintf(line + at, sizeof(line) - at, "%-19s | ", tmp);
    if (x.hops == 0) {
      snprintf(tmp, sizeof(tmp), "directo");
    } else {
      char v[16];
      viaLabel(x.via, v, sizeof(v));
      snprintf(tmp, sizeof(tmp), "por %s (%u salto%s)", v, x.hops, x.hops == 1 ? "" : "s");
    }
    at += snprintf(line + at, sizeof(line) - at, "%-19s | ", tmp);
    if (x.inSession && x.pings) {
      snprintf(tmp, sizeof(tmp), "%u/%u %u%%", x.pongs, x.pings, (unsigned)(100u * (x.pings - x.pongs) / x.pings));
    } else {
      snprintf(tmp, sizeof(tmp), "-");
    }
    at += snprintf(line + at, sizeof(line) - at, "%-10s | ", tmp);
    fmtAge(nowMs - x.heardMs, age, sizeof(age));
    snprintf(line + at, sizeof(line) - at, "hace %s", age);
    DEBUG_PRINTLN(line);
  }
  DEBUG_PRINTLN("[DIAG] RSSI/SNR: ultimo paquete que el nodo transmitio y oyo este collar (suyo o repitiendo). 'le llega':"
                " como oyo el nodo a este collar. PONG: respuestas/PING y perdida.");
}
