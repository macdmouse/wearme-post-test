#include "sd_card.h"

bool MicroSd::begin(uint32_t maxSckHz) {
  end();
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);

  cardOk_ = fs_.cardBegin(SdSpiConfig(cs_, SHARED_SPI, maxSckHz, &spi_));
  if (!cardOk_) {
    errorCode_ = fs_.card() ? fs_.card()->errorCode() : 0xFF;
    return false;
  }
  cardType_ = fs_.card()->type();
  sizeMB_ = fs_.card()->sectorCount() / 2048; // sectores de 512 B
  mounted_ = fs_.volumeBegin();
  fatType_ = mounted_ ? fs_.fatType() : 0;
  if (!mounted_) {
    // Sin volumen: se lee el MBR para saber si falla la lectura o el formato no es FAT/exFAT
    uint8_t sector[512];
    sector0Read_ = fs_.card()->readSector(0, sector);
    mbrSig_ = sector0Read_ && sector[510] == 0x55 && sector[511] == 0xAA;
    partType_ = mbrSig_ ? sector[0x1C2] : 0; // tipo de la partición 1 (entrada en 0x1BE)
  }
  return true;
}

bool MicroSd::check() {
  if (cardOk_ && fs_.card()->sectorCount() != 0) return true;
  return begin();
}

void MicroSd::end() {
  if (cardOk_) fs_.end();
  cardOk_ = false;
  mounted_ = false;
  errorCode_ = 0;
  cardType_ = 0;
  sizeMB_ = 0;
  fatType_ = 0;
  sector0Read_ = false;
  mbrSig_ = false;
  partType_ = 0;
}

// Primera línea del archivo sin el fin de línea; false si no se puede abrir
static bool readFirstLine(SdFs &fs, const char *path, char *buf, size_t len) {
  FsFile f;
  if (!f.open(path, O_RDONLY)) return false;
  const int n = f.fgets(buf, (int)len);
  f.close();
  if (n <= 0) {
    buf[0] = '\0';
    return true;
  }
  for (int i = n - 1; i >= 0 && (buf[i] == '\r' || buf[i] == '\n'); i--) buf[i] = '\0';
  return true;
}

bool MicroSd::appendCsv(const char *dir, const char *base, const char *header, const char *row, char *usedPath,
                        size_t usedLen) {
  if (!mounted_) return false;
  if (!fs_.exists(dir) && !fs_.mkdir(dir)) return false;

  char path[64];
  static char first[1024]; // el encabezado de la medición pasa de 512 caracteres (estático: no en la pila)
  for (uint8_t n = 0; n < 100; n++) {
    if (n == 0) {
      snprintf(path, sizeof(path), "%s/%s.csv", dir, base);
    } else {
      snprintf(path, sizeof(path), "%s/%s_%u.csv", dir, base, n);
    }
    if (!fs_.exists(path)) break;                          // archivo nuevo
    if (!readFirstLine(fs_, path, first, sizeof(first))) return false;
    if (first[0] == '\0' || strcmp(first, header) == 0) break; // mismo formato (o vacío)
  }

  FsFile f;
  if (!f.open(path, O_WRONLY | O_CREAT | O_APPEND)) return false;
  bool ok = true;
  if (f.fileSize() == 0) ok = f.write(header) == strlen(header) && f.write("\r\n") == 2;
  ok = ok && f.write(row) == strlen(row) && f.write("\r\n") == 2;
  ok = f.close() && ok;
  if (usedPath != nullptr && usedLen > 0) snprintf(usedPath, usedLen, "%s", path);
  return ok;
}

const char *MicroSd::fsName() const {
  switch (fatType_) {
    case 12: return "FAT12";
    case 16: return "FAT16";
    case 32: return "FAT32";
    case FAT_TYPE_EXFAT: return "exFAT";
    default: return "-";
  }
}

const char *MicroSd::typeName() const {
  switch (cardType_) {
    case SD_CARD_TYPE_SD1: return "SD1";
    case SD_CARD_TYPE_SD2: return "SD2";
    case SD_CARD_TYPE_SDHC: return "SDHC/SDXC";
    default: return "-";
  }
}

bool MicroSd::readLine(const char *path, uint32_t &pos, char *buf, size_t size) {
  if (!mounted_ || size == 0) return false;
  FsFile f;
  if (!f.open(path, O_RDONLY)) return false;
  if (pos >= f.fileSize() || !f.seekSet(pos)) {
    f.close();
    return false;
  }
  const int n = f.fgets(buf, (int)size);
  const uint32_t next = (uint32_t)f.curPosition();
  f.close();
  if (n <= 0) return false;
  pos = next;
  for (int i = n - 1; i >= 0 && (buf[i] == '\r' || buf[i] == '\n'); i--) buf[i] = '\0';
  return true;
}

bool MicroSd::writeAt(const char *path, uint32_t pos, const char *data) {
  if (!mounted_) return false;
  FsFile f;
  if (!f.open(path, O_RDWR)) return false;
  const size_t len = strlen(data);
  bool ok = f.seekSet(pos) && f.write(data, len) == len;
  ok = f.close() && ok;
  return ok;
}

bool MicroSd::rewriteFiltered(const char *path, bool (*keep)(const char *line)) {
  if (!mounted_) return false;
  char tmp[72];
  snprintf(tmp, sizeof(tmp), "%s.tmp", path);
  FsFile in, out;
  if (!in.open(path, O_RDONLY)) return false;
  fs_.remove(tmp);
  if (!out.open(tmp, O_WRONLY | O_CREAT | O_TRUNC)) {
    in.close();
    return false;
  }
  static char line[256];
  bool ok = true;
  bool first = true;
  while (ok) {
    const int n = in.fgets(line, sizeof(line));
    if (n <= 0) break;
    for (int i = n - 1; i >= 0 && (line[i] == '\r' || line[i] == '\n'); i--) line[i] = '\0';
    if (first || keep(line)) ok = out.write(line) == strlen(line) && out.write("\r\n") == 2;
    first = false;
  }
  in.close();
  ok = out.close() && ok;
  if (!ok) {
    fs_.remove(tmp);
    return false;
  }
  return fs_.remove(path) && fs_.rename(tmp, path);
}
