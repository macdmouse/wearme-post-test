-- Códigos de prueba (kind = 'test'): un administrador los sube desde el modo superusuario para cargarlos a una tarjeta
-- (p. ej. un sketch de Arduino que prueba un sensor). No tienen que ver con el firmware principal: nunca quedan
-- activos y la estación no les pide la línea [BOOT] ni NODE?.
-- SQLite no deja cambiar un CHECK: se rehacen las dos tablas conservando las filas. Primero se renombran las viejas
-- (la referencia de firmware_chunks sigue a la tabla renombrada), luego se crean las nuevas y se copian los datos.
ALTER TABLE firmware_chunks RENAME TO firmware_chunks_old;
ALTER TABLE firmwares RENAME TO firmwares_old;

CREATE TABLE firmwares (
  id          INTEGER PRIMARY KEY AUTOINCREMENT,
  kind        TEXT NOT NULL DEFAULT 'main' CHECK (kind IN ('main', 'test')),
  version     TEXT NOT NULL,            -- principal: WEARME_FW_VERSION ("[BOOT] Firmware <versión> compilado:"); prueba: el nombre que se le dio
  size        INTEGER NOT NULL,         -- bytes del .bin
  crc16       INTEGER NOT NULL,         -- CRC16 del .bin (el mismo del paquete de inicio)
  sha256      TEXT NOT NULL UNIQUE,
  built       TEXT,                     -- fecha de compilación (AAAA-MM-DD HH:MM)
  dat         TEXT NOT NULL,            -- paquete de inicio de la carga (.dat), base64
  notes       TEXT,
  active      INTEGER NOT NULL DEFAULT 0 CHECK (active = 0 OR kind = 'main'),
  uploaded_by TEXT NOT NULL,
  uploaded_at TEXT NOT NULL DEFAULT (datetime('now'))
);
INSERT INTO firmwares (id, kind, version, size, crc16, sha256, built, dat, notes, active, uploaded_by, uploaded_at)
  SELECT id, kind, version, size, crc16, sha256, built, dat, notes, active, uploaded_by, uploaded_at FROM firmwares_old;

CREATE TABLE firmware_chunks (
  firmware_id INTEGER NOT NULL REFERENCES firmwares(id) ON DELETE CASCADE,
  seq         INTEGER NOT NULL,
  data        TEXT NOT NULL,
  PRIMARY KEY (firmware_id, seq)
);
INSERT INTO firmware_chunks (firmware_id, seq, data) SELECT firmware_id, seq, data FROM firmware_chunks_old;

DROP TABLE firmware_chunks_old;
DROP TABLE firmwares_old;
-- A lo sumo uno activo por tipo (el índice viejo se fue con firmwares_old)
CREATE UNIQUE INDEX firmwares_active ON firmwares(kind) WHERE active = 1;
