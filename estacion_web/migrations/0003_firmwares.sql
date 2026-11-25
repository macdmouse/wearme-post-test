-- Modo superusuario: firmwares principales subidos desde la página por un administrador (el firmware.zip que deja
-- PlatformIO al compilar Wearme_Firmware). El ACTIVO es el que carga la estación; si no hay ninguno activo se usa el
-- de la publicación (public/firmware/firmwares.json, generado por tools/build_estacion.py --web).
CREATE TABLE firmwares (
  id          INTEGER PRIMARY KEY AUTOINCREMENT,
  kind        TEXT NOT NULL DEFAULT 'main' CHECK (kind IN ('main')),
  version     TEXT NOT NULL,            -- WEARME_FW_VERSION: la que imprime "[BOOT] Firmware <versión> compilado:"
  size        INTEGER NOT NULL,         -- bytes del .bin
  crc16       INTEGER NOT NULL,         -- CRC16 del .bin (el mismo del paquete de inicio)
  sha256      TEXT NOT NULL UNIQUE,
  built       TEXT,                     -- fecha de compilación que trae el firmware (AAAA-MM-DD HH:MM)
  dat         TEXT NOT NULL,            -- paquete de inicio de la carga (.dat), base64
  notes       TEXT,
  active      INTEGER NOT NULL DEFAULT 0,
  uploaded_by TEXT NOT NULL,
  uploaded_at TEXT NOT NULL DEFAULT (datetime('now'))
);
-- A lo sumo uno activo por tipo
CREATE UNIQUE INDEX firmwares_active ON firmwares(kind) WHERE active = 1;

-- El .bin en trozos de base64 (D1 limita el tamaño de cada sentencia)
CREATE TABLE firmware_chunks (
  firmware_id INTEGER NOT NULL REFERENCES firmwares(id) ON DELETE CASCADE,
  seq         INTEGER NOT NULL,
  data        TEXT NOT NULL,
  PRIMARY KEY (firmware_id, seq)
);
