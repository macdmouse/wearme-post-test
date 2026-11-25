// mesh_lib.cpp
// Paquetes Meshtastic de solo envío (ver mesh_lib.h).

#include "mesh_lib.h"
#include <string.h>
#include <nrf_sdm.h> // sd_softdevice_is_enabled
#include <nrf_soc.h> // sd_ecb_block_encrypt, nrf_ecb_hal_data_t

// Clave por defecto de Meshtastic ("AQ==" = índice 1); los índices 2-10 cambian el último byte
static const uint8_t DEFAULT_KEY[16] = {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
                                        0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01};

struct Preset {
  const char *name;
  uint8_t sf;
  uint16_t bwKHz;
  uint8_t cr;
};
static const Preset PRESETS[] = {
  {"LongFast", 11, 250, 5}, {"LongSlow", 12, 125, 8},   {"LongMod", 11, 125, 8},  {"MediumSlow", 10, 250, 5},
  {"MediumFast", 9, 250, 5}, {"ShortSlow", 8, 250, 5}, {"ShortFast", 7, 250, 5}, {"ShortTurbo", 7, 500, 5},
};

struct Region {
  const char *name;
  uint32_t startKHz, endKHz;
};
static const Region REGIONS[] = {{"US", 902000, 928000}, {"ANZ", 915000, 928000}};

// Hash del nombre del canal (djb2, igual que RadioInterface::hash del firmware)
static uint32_t djb2(const char *s) {
  uint32_t h = 5381;
  while (*s) h = h * 33u + (uint8_t)*s++;
  return h;
}

static uint8_t xorHash(const uint8_t *p, size_t n) {
  uint8_t h = 0;
  while (n--) h ^= *p++;
  return h;
}

// AES-128 de un bloque con el ECB del nRF52840 (por el SoftDevice si está activo: el BLE lo usa)
static bool aes128Block(const uint8_t key[16], const uint8_t in[16], uint8_t out[16]) {
  nrf_ecb_hal_data_t d;
  memcpy(d.key, key, 16);
  memcpy(d.cleartext, in, 16);
  uint8_t sd = 0;
  sd_softdevice_is_enabled(&sd);
  if (sd) {
    if (sd_ecb_block_encrypt(&d) != NRF_SUCCESS) return false;
  } else {
    NRF_ECB->ECBDATAPTR = (uint32_t)&d;
    NRF_ECB->EVENTS_ENDECB = 0;
    NRF_ECB->EVENTS_ERRORECB = 0;
    NRF_ECB->TASKS_STARTECB = 1;
    while (!NRF_ECB->EVENTS_ENDECB && !NRF_ECB->EVENTS_ERRORECB) {
    }
    const bool err = NRF_ECB->EVENTS_ERRORECB;
    NRF_ECB->EVENTS_ENDECB = 0;
    NRF_ECB->EVENTS_ERRORECB = 0;
    if (err) return false;
  }
  memcpy(out, d.ciphertext, 16);
  return true;
}

bool MeshtasticTx::aesSelfTest() {
  static const uint8_t key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                  0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f};
  static const uint8_t pt[16] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
                                 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff};
  static const uint8_t ct[16] = {0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
                                 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a};
  uint8_t out[16];
  return aes128Block(key, pt, out) && memcmp(out, ct, 16) == 0;
}

bool MeshtasticTx::begin(const Config &cfg) {
  cfg_ = cfg;
  const Preset *preset = nullptr;
  for (const Preset &p : PRESETS)
    if (strcmp(p.name, cfg.preset) == 0) preset = &p;
  const Region *region = nullptr;
  for (const Region &r : REGIONS)
    if (strcmp(r.name, cfg.region) == 0) region = &r;
  error_ = !preset                                         ? "preset desconocido"
           : !region                                        ? "region desconocida (US o ANZ)"
           : cfg.nodeNum == 0 || cfg.nodeNum == 0xFFFFFFFFu ? "numero de nodo no valido"
           : cfg.hopLimit > 7                               ? "hop limit mayor que 7"
           : cfg.pskLen != 1 && cfg.pskLen != 16            ? "clave de 1 o 16 bytes (AES-256 no soportado)"
                                                            : "";
  if (error_[0]) return false;

  // Clave: 1 byte = índice de la clave por defecto (0 = sin cifrado); 16 bytes = AES-128
  if (cfg.pskLen == 1) {
    encrypted_ = cfg.psk[0] != 0;
    memcpy(key_, DEFAULT_KEY, 16);
    key_[15] = (uint8_t)(key_[15] + cfg.psk[0] - 1);
  } else {
    encrypted_ = true;
    memcpy(key_, cfg.psk, 16);
  }

  strncpy(channel_, cfg.channelName && cfg.channelName[0] ? cfg.channelName : preset->name, sizeof(channel_) - 1);
  channel_[sizeof(channel_) - 1] = '\0';
  channelHash_ = xorHash((const uint8_t *)channel_, strlen(channel_)) ^ (encrypted_ ? xorHash(key_, 16) : 0);

  const uint32_t channels = (region->endKHz - region->startKHz) / preset->bwKHz;
  const uint32_t index = djb2(channel_) % channels;
  slot_ = (uint8_t)(index + 1);
  radio_.freqHz = cfg.freqHz ? cfg.freqHz
                             : region->startKHz * 1000u + preset->bwKHz * 500u + index * preset->bwKHz * 1000u;
  radio_.sf = preset->sf;
  radio_.bwKHz = preset->bwKHz;
  radio_.cr = preset->cr;
  radio_.preamble = 16;
  radio_.syncWord = 0x2B;
  idState_ = cfg.idSeed ? cfg.idSeed : (cfg.nodeNum ^ 0x9E3779B9u);
  return true;
}

uint32_t MeshtasticTx::nextId() {
  // xorshift32: ids distintos entre paquetes y reinicios (los nodos descartan duplicados por from + id)
  uint32_t x = idState_;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  idState_ = x ? x : 1;
  return idState_;
}

// AES-CTR: nonce = id (uint64 LE) + from (uint32 LE) + 0; el contador incrementa el bloque como entero big endian
bool MeshtasticTx::ctr(uint32_t packetId, uint32_t from, uint8_t *buf, size_t len) const {
  uint8_t counter[16] = {}, stream[16];
  memcpy(counter, &packetId, 4); // little endian (Cortex-M), los 4 bytes altos del uint64 quedan en 0
  memcpy(counter + 8, &from, 4);
  for (size_t off = 0; off < len; off += 16) {
    if (!aes128Block(key_, counter, stream)) return false;
    for (size_t i = 0; i < 16 && off + i < len; i++) buf[off + i] ^= stream[i];
    for (int i = 15; i >= 0 && ++counter[i] == 0; i--) {
    }
  }
  return true;
}

// ---- Protobuf mínimo ----
namespace {
struct Pb {
  uint8_t *p, *end;
  bool ok = true;
  Pb(uint8_t *b, size_t n) : p(b), end(b + n) {}
  void byte(uint8_t b) {
    if (p < end) *p++ = b;
    else ok = false;
  }
  void varint(uint64_t v) {
    do {
      uint8_t b = v & 0x7F;
      v >>= 7;
      byte(v ? b | 0x80 : b);
    } while (v);
  }
  void tag(uint32_t field, uint8_t wire) { varint((field << 3) | wire); }
  void fieldVarint(uint32_t field, uint64_t v) {
    tag(field, 0);
    varint(v);
  }
  void fieldFixed32(uint32_t field, uint32_t v) {
    tag(field, 5);
    for (int i = 0; i < 4; i++) byte((uint8_t)(v >> (8 * i)));
  }
  void fieldBytes(uint32_t field, const uint8_t *d, size_t n) {
    tag(field, 2);
    varint(n);
    for (size_t i = 0; i < n; i++) byte(d[i]);
  }
  void fieldString(uint32_t field, const char *s) { fieldBytes(field, (const uint8_t *)s, strlen(s)); }
};
} // namespace

size_t MeshtasticTx::packet(uint16_t port, const uint8_t *payload, size_t len, uint8_t *out, size_t max) {
  if (max > MAX_PACKET) max = MAX_PACKET;
  if (max < 16 + 8) return 0;
  // Data { portnum = 1; payload = 2; }
  Pb pb(out + 16, max - 16);
  pb.fieldVarint(1, port);
  pb.fieldBytes(2, payload, len);
  if (!pb.ok) return 0;
  const size_t dataLen = pb.p - (out + 16);

  const uint32_t to = 0xFFFFFFFFu, from = cfg_.nodeNum, id = nextId();
  memcpy(out, &to, 4);
  memcpy(out + 4, &from, 4);
  memcpy(out + 8, &id, 4);
  out[12] = (uint8_t)((cfg_.hopLimit << 5) | cfg_.hopLimit); // hop_start = hop_limit: sale de este nodo
  out[13] = channelHash_;
  out[14] = 0;               // next_hop: sin preferencia
  out[15] = (uint8_t)from;   // relay_node: byte bajo de quien transmite
  if (encrypted_ && !ctr(id, from, out + 16, dataLen)) return 0;
  return 16 + dataLen;
}

size_t MeshtasticTx::app(const uint8_t *payload, size_t len, uint8_t *out, size_t max) {
  return packet(PORT_PRIVATE, payload, len, out, max);
}

// Lectura de un varint protobuf; false si se sale del buffer
static bool readVarint(const uint8_t *&p, const uint8_t *end, uint64_t &v) {
  v = 0;
  for (uint8_t shift = 0; p < end && shift < 64; shift += 7) {
    const uint8_t b = *p++;
    v |= (uint64_t)(b & 0x7F) << shift;
    if (!(b & 0x80)) return true;
  }
  return false;
}

bool MeshtasticTx::parse(uint8_t *pkt, size_t len, MeshRx &out) const {
  if (len < 16 + 2 || len > MAX_PACKET) return false;
  if (pkt[13] != channelHash_) return false; // otro canal (o clave distinta)
  memcpy(&out.to, pkt, 4);
  memcpy(&out.from, pkt + 4, 4);
  memcpy(&out.id, pkt + 8, 4);
  out.hopLimit = pkt[12] & 0x07;
  out.hopStart = pkt[12] >> 5;
  out.port = 0;
  out.payload = nullptr;
  out.payloadLen = 0;
  if (encrypted_ && !ctr(out.id, out.from, pkt + 16, len - 16)) return false;
  // Data { portnum = 1 (varint); payload = 2 (bytes); ... }: se ignoran los demás campos
  const uint8_t *p = pkt + 16, *end = pkt + len;
  while (p < end) {
    uint64_t key = 0, v = 0;
    if (!readVarint(p, end, key)) return false;
    const uint8_t wire = key & 7;
    const uint32_t field = (uint32_t)(key >> 3);
    if (wire == 0) {
      if (!readVarint(p, end, v)) return false;
      if (field == 1) out.port = (uint16_t)v;
    } else if (wire == 2) {
      if (!readVarint(p, end, v) || v > (uint64_t)(end - p)) return false;
      if (field == 2) {
        out.payload = p;
        out.payloadLen = (size_t)v;
      }
      p += v;
    } else if (wire == 5) {
      if (end - p < 4) return false;
      p += 4;
    } else if (wire == 1) {
      if (end - p < 8) return false;
      p += 8;
    } else {
      return false; // tipo no válido: clave equivocada o paquete corrupto
    }
  }
  return out.port != 0;
}

bool MeshtasticTx::relay(uint8_t *pkt, size_t len) const {
  if (len < 16) return false;
  const uint8_t hopLimit = pkt[12] & 0x07;
  if (hopLimit == 0) return false;
  pkt[12] = (uint8_t)((pkt[12] & ~0x07) | (hopLimit - 1)); // hop_start se conserva
  pkt[15] = (uint8_t)cfg_.nodeNum;                          // relay_node: quien retransmite
  return true;
}

size_t MeshtasticTx::text(const char *s, uint8_t *out, size_t max) {
  return packet(PORT_TEXT, (const uint8_t *)s, strlen(s), out, max);
}

size_t MeshtasticTx::position(double lat, double lon, int32_t altM, uint32_t unixTime, uint8_t *out, size_t max) {
  uint8_t buf[48];
  Pb pb(buf, sizeof(buf));
  pb.fieldFixed32(1, (uint32_t)(int32_t)(lat * 1e7)); // latitude_i (sfixed32, grados × 1e7)
  pb.fieldFixed32(2, (uint32_t)(int32_t)(lon * 1e7)); // longitude_i
  pb.fieldVarint(3, (uint64_t)(int64_t)altM);         // altitude (int32: negativo = varint de 10 bytes)
  if (unixTime) pb.fieldFixed32(4, unixTime);         // time
  pb.fieldVarint(5, 2);                               // location_source = LOC_INTERNAL
  pb.fieldVarint(23, 32);                             // precision_bits = 32 (precisión completa)
  if (!pb.ok) return 0;
  return packet(PORT_POSITION, buf, pb.p - buf, out, max);
}

size_t MeshtasticTx::nodeInfo(const char *longName, const char *shortName, uint8_t *out, size_t max) {
  uint8_t buf[96];
  char id[12];
  snprintf(id, sizeof(id), "!%08lx", (unsigned long)cfg_.nodeNum);
  Pb pb(buf, sizeof(buf));
  pb.fieldString(1, id);        // id
  pb.fieldString(2, longName);  // long_name
  pb.fieldString(3, shortName); // short_name (hasta 4 caracteres)
  pb.fieldVarint(5, 255);       // hw_model = PRIVATE_HW
  pb.fieldVarint(7, 6);         // role = SENSOR
  if (!pb.ok) return 0;
  return packet(PORT_NODEINFO, buf, pb.p - buf, out, max);
}
