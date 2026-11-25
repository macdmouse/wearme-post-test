#pragma once

// =====================================================================================
// MicroSd — microSD por SPI con la libreria SdFat del framework. Usa SdFs (FAT16/FAT32 y exFAT):
// el SdFat del framework esta configurado solo para FAT (SDFAT_FILE_TYPE 1) y las tarjetas de
// 32 GB o mas suelen venir en exFAT.
//
// El puerto SPI se pasa desde la placa: en Wearme es el SPI del variant, en SPIM2
// (SPIM3 es el del LR1110 en LBM_WM1110).
// begin() se puede repetir (p. ej. si la tarjeta se inserto despues del arranque).
// =====================================================================================

#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

class MicroSd {
public:
  MicroSd(SPIClass &spi, uint8_t csPin) : spi_(spi), cs_(csPin) {}

  // Inicializa la tarjeta y monta el sistema de archivos. true si la tarjeta responde.
  bool begin(uint32_t maxSckHz = SD_SCK_MHZ(4));
  void end();
  // Si la tarjeta ya respondio, solo relee su capacidad (CSD, rapido); si no, reintenta begin()
  bool check();

  bool cardOk() const { return cardOk_; }  // la tarjeta respondio por SPI
  bool mounted() const { return mounted_; } // ademas hay un volumen FAT o exFAT valido
  uint8_t errorCode() const { return errorCode_; }
  uint32_t sizeMB() const { return sizeMB_; }
  uint8_t fatType() const { return fatType_; } // 12, 16, 32 o FAT_TYPE_EXFAT (0 = sin montar)
  const char *typeName() const;                // "SD1", "SD2", "SDHC/SDXC" o "-"
  const char *fsName() const;                  // "FAT16", "FAT32", "exFAT" o "-"

  // Diagnóstico cuando no monta: lectura del sector 0 (MBR) y tipo de la partición 1
  bool sector0Read() const { return sector0Read_; }
  bool mbrSignature() const { return mbrSig_; }  // 0x55AA al final del sector 0
  uint8_t partitionType() const { return partType_; } // 0x0B/0x0C FAT32, 0x07 exFAT/NTFS, 0xEE GPT, 0 = sin MBR

  SdFs &fs() { return fs_; }

  // Agrega una fila a <dir>/<base>.csv (crea la carpeta si no existe). Si el archivo es nuevo escribe
  // antes el encabezado. Si existe con otro encabezado (se cambiaron las columnas) usa <base>_1.csv,
  // <base>_2.csv... para no mezclar formatos. usedPath (opcional) devuelve la ruta escrita.
  // Abre, escribe y cierra en cada llamada: lo escrito queda en la tarjeta aunque se corte la energía.
  bool appendCsv(const char *dir, const char *base, const char *header, const char *row, char *usedPath = nullptr,
                 size_t usedLen = 0);
  // Línea que empieza en `pos` (sin CR/LF); pos pasa al inicio de la siguiente. false al final del archivo.
  bool readLine(const char *path, uint32_t &pos, char *buf, size_t size);
  // Sobrescribe bytes en una posición (mismo largo: no mueve el resto del archivo)
  bool writeAt(const char *path, uint32_t pos, const char *data);
  // Reescribe el archivo dejando la primera línea (encabezado) y las que cumplan keep(línea)
  bool rewriteFiltered(const char *path, bool (*keep)(const char *line));

private:
  SPIClass &spi_;
  uint8_t cs_;
  SdFs fs_;
  bool cardOk_ = false;
  bool mounted_ = false;
  uint8_t errorCode_ = 0;
  uint8_t cardType_ = 0;
  uint32_t sizeMB_ = 0;
  uint8_t fatType_ = 0;
  bool sector0Read_ = false;
  bool mbrSig_ = false;
  uint8_t partType_ = 0;
};
