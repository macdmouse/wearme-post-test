// mesh_lib.h
// Paquetes Meshtastic de solo envío (el nodo no escucha ni retransmite) para usar la mesh como respaldo cuando no hay
// gateway LoRaWAN. Arma los bytes que van al aire; el radio lo maneja quien llama (la placa, con LBM suspendido).
//
// Formato en el aire (firmware Meshtastic 2.x):
//   cabecera de 16 B (little endian): to (0xFFFFFFFF = todos), from (número de nodo), id del paquete,
//   flags (hop_limit bits 0-2, hop_start bits 5-7), hash del canal, next_hop (0), relay_node (byte bajo de from)
//   + protobuf Data {portnum = 1, payload = 2} cifrado con AES-CTR: clave del canal, nonce = id (uint64 LE) + from
//   (uint32 LE) + 0, contador big endian en el bloque completo.
// Radio: preset (LongFast = SF11, BW 250 kHz, CR 4/5), preámbulo 16, sync word 0x2B, CRC, cabecera explícita.
// Frecuencia: región + preset + nombre del canal (djb2 % número de canales), igual que el firmware; LongFast por
// defecto = 906.875 MHz (US) / 919.875 MHz (ANZ).
// Clave: 1 byte = índice de la clave por defecto (1 = "AQ==", la del canal público LongFast; 0 = sin cifrado) o
// 16 bytes (AES-128). AES-256 (32 bytes) no está soportado: el AES es el del nRF52840 (ECB por hardware).

#ifndef MESH_LIB_H
#define MESH_LIB_H

#include <Arduino.h>

struct MeshRadio {
  uint32_t freqHz;
  uint8_t sf;
  uint16_t bwKHz;
  uint8_t cr;        // 5 = 4/5 ... 8 = 4/8
  uint16_t preamble; // símbolos
  uint8_t syncWord;
};

// Paquete recibido y descifrado
struct MeshRx {
  uint32_t to, from, id;
  uint8_t hopLimit, hopStart;
  uint16_t port;            // portnum del Data (0 = no se pudo decodificar)
  const uint8_t *payload;   // dentro del buffer de recepción (descifrado en su lugar)
  size_t payloadLen;
};

class MeshtasticTx {
public:
  static constexpr size_t MAX_PACKET = 255;
  static constexpr uint16_t PORT_TEXT = 1, PORT_POSITION = 3, PORT_NODEINFO = 4, PORT_PRIVATE = 256;

  struct Config {
    const char *region = "US";     // "US" (902-928 MHz) o "ANZ" (915-928 MHz)
    const char *preset = "LongFast";
    const char *channelName = "";  // vacío = nombre del preset (canal primario por defecto)
    const uint8_t *psk = nullptr;
    uint8_t pskLen = 0;
    uint32_t nodeNum = 0;          // número de nodo (!xxxxxxxx); no 0 ni 0xFFFFFFFF
    uint8_t hopLimit = 3;
    uint32_t freqHz = 0;           // 0 = la calculada como el firmware
    uint32_t idSeed = 0;           // semilla de los id de paquete (aleatoria)
  };

  bool begin(const Config &cfg); // false: región, preset, clave o nodo no válidos (ver error())
  const char *error() const { return error_; }
  const MeshRadio &radio() const { return radio_; }
  uint8_t channelHash() const { return channelHash_; }
  uint32_t nodeNum() const { return cfg_.nodeNum; }
  const char *channelName() const { return channel_; }
  uint8_t channelSlot() const { return slot_; } // 1-based, como en la app

  // Paquete completo listo para transmitir; devuelve la longitud (0 = no cabe o falló el cifrado)
  size_t text(const char *s, uint8_t *out, size_t max);
  size_t position(double lat, double lon, int32_t altM, uint32_t unixTime, uint8_t *out, size_t max);
  size_t nodeInfo(const char *longName, const char *shortName, uint8_t *out, size_t max);
  size_t app(const uint8_t *payload, size_t len, uint8_t *out, size_t max); // PRIVATE_APP (256), binario

  // Recepción: valida la cabecera y el hash del canal, descifra EN SU LUGAR (pkt queda modificado) y decodifica Data.
  // false = no es de este canal o no se pudo decodificar. Para repetir hay que copiar el paquete ANTES de parse().
  bool parse(uint8_t *pkt, size_t len, MeshRx &out) const;
  // Repetición: copia tal cual con hop_limit − 1 y relay_node = este nodo (el cifrado no cambia: el nonce depende de
  // from + id). false si hop_limit ya es 0.
  bool relay(uint8_t *pkt, size_t len) const;

  static bool aesSelfTest(); // vector FIPS-197 (C.1) con el AES del nRF52840

private:
  size_t packet(uint16_t port, const uint8_t *payload, size_t len, uint8_t *out, size_t max);
  bool ctr(uint32_t packetId, uint32_t from, uint8_t *buf, size_t len) const;
  uint32_t nextId();

  Config cfg_;
  MeshRadio radio_ = {};
  uint8_t key_[16] = {};
  bool encrypted_ = false;
  uint8_t channelHash_ = 0, slot_ = 0;
  char channel_[16] = "";
  uint32_t idState_ = 1;
  const char *error_ = "";
};

#endif
