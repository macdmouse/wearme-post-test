-- Registro de nodos: de dónde salió cada uno y si ya está dado de alta en la plataforma LoRaWAN (servidor de red)

-- 'tabla' (include/wearme_credentials.h), 'generador' (tools/DEVEUI_APPKEY_generator), 'estacion' (creado por la
-- estación para una tarjeta sin nodo) o 'manual' (un administrador desde la página)
ALTER TABLE nodes ADD COLUMN source TEXT;
-- Fecha y persona que lo marcó como registrado en la plataforma. NULL = falta registrarlo
ALTER TABLE nodes ADD COLUMN platform_at TEXT;
ALTER TABLE nodes ADD COLUMN platform_by TEXT;

UPDATE nodes SET source = 'tabla' WHERE source IS NULL AND created_by = 'importar_credenciales.py';
UPDATE nodes SET source = 'manual' WHERE source IS NULL;
