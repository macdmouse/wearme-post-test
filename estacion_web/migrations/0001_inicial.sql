-- Base de la estación de montaje Wearme (Cloudflare D1 / SQLite)

-- Credenciales LoRaWAN de cada nodo (reemplaza include/wearme_credentials.h como fuente de verdad)
CREATE TABLE nodes (
  node       INTEGER PRIMARY KEY CHECK (node BETWEEN 1 AND 999),
  dev_eui    TEXT NOT NULL UNIQUE CHECK (length(dev_eui) = 16),
  join_eui   TEXT NOT NULL CHECK (length(join_eui) = 16),
  app_key    TEXT NOT NULL CHECK (length(app_key) = 32),
  notes      TEXT,
  created_by TEXT,
  created_at TEXT NOT NULL DEFAULT (datetime('now'))
);

-- Tarjetas físicas, por el DEVICEID del nRF52840. Un nodo puede estar en una sola tarjeta.
CREATE TABLE boards (
  device_id   TEXT PRIMARY KEY CHECK (length(device_id) = 16),
  node        INTEGER UNIQUE REFERENCES nodes(node),
  assigned_by TEXT,
  assigned_at TEXT,
  first_seen  TEXT NOT NULL DEFAULT (datetime('now')),
  last_seen   TEXT NOT NULL DEFAULT (datetime('now'))
);

-- Cada prueba: 'montaje' (resultado del POST) o 'principal' (firmware principal cargado y verificado)
CREATE TABLE tests (
  id         INTEGER PRIMARY KEY AUTOINCREMENT,
  device_id  TEXT NOT NULL,
  node       INTEGER,
  step       TEXT NOT NULL CHECK (step IN ('montaje', 'principal')),
  result     TEXT NOT NULL CHECK (result IN ('OK', 'FALLA')),
  failures   TEXT,      -- subsistemas con falla, separados por coma
  firmware   TEXT,      -- versión del firmware que corrió la prueba
  results    TEXT,      -- JSON: estado de cada subsistema y del micro
  operator   TEXT NOT NULL,
  created_at TEXT NOT NULL DEFAULT (datetime('now'))
);
CREATE INDEX tests_device ON tests (device_id, created_at);

-- Quién leyó credenciales, asignó nodos o agregó nodos
CREATE TABLE audit (
  id        INTEGER PRIMARY KEY AUTOINCREMENT,
  at        TEXT NOT NULL DEFAULT (datetime('now')),
  operator  TEXT NOT NULL,
  action    TEXT NOT NULL,
  node      INTEGER,
  device_id TEXT,
  detail    TEXT
);
