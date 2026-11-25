// API de la estación de montaje Wearme (Cloudflare Worker + D1).
// La página y los firmwares (public/) los sirve Cloudflare directamente; aquí llegan solo las rutas /api/*.
// Todo está detrás de Cloudflare Access: cada petición trae un token firmado (Cf-Access-Jwt-Assertion) que se
// verifica con las claves públicas del equipo de Access. Sin token válido no se entrega nada.
//
//   GET  /api/me                          quién soy (correo) y si soy administrador
//   GET  /api/nodes                       nodos de la base (sin AppKey) y a qué tarjeta están asignados
//   GET  /api/nodes/:n/credentials?device=ID[&force=1]   credenciales para grabar ese nodo en esa tarjeta
//   POST /api/nodes                       (admin) agregar o corregir un nodo {node, dev_eui, join_eui, app_key, notes}
//   POST /api/boards/:device/assign       registrar que la tarjeta quedó con ese nodo {node, force}
//   GET  /api/boards/:device              tarjeta + últimas pruebas
//   POST /api/tests                       registrar una prueba {device_id, node, step, result, failures, firmware, results}
//   GET  /api/tests?limit=50              últimas pruebas
//   GET  /api/firmware/main               firmware principal activo de la base (null: se usa el de la publicación)
//   GET  /api/firmware                    (admin) firmwares subidos (principales y códigos de prueba), sin el .bin
//   GET  /api/firmware/tests              programas de prueba subidos (sin el .bin): la estación los ofrece para cargar
//   GET  /api/firmware/:id                uno con su .bin, para cargarlo a una tarjeta (no admin: solo programas de prueba)
//   POST /api/firmware                    (admin) subir uno {kind: 'main'|'test', version, built, notes, bin, dat (base64), activate}
//   POST /api/firmware/active             (admin) elegir el activo {id}; {id: null} = volver al de la publicación
//   DELETE /api/firmware/:id              (admin) borrar uno que no esté activo

class HttpError extends Error {
  constructor(status, message, extra) {
    super(message);
    this.status = status;
    this.extra = extra;
  }
}

const json = (data, status = 200) =>
  new Response(JSON.stringify(data), { status, headers: { 'content-type': 'application/json; charset=utf-8', 'cache-control': 'no-store' } });

const HEX = n => new RegExp('^[0-9A-F]{' + n + '}$');
const isNode = n => Number.isInteger(n) && n >= 1 && n <= 999;
const isDevice = d => HEX(16).test(d || '');

// ---------- Cloudflare Access: verificación del token ----------
const b64urlBytes = s => Uint8Array.from(atob(s.replace(/-/g, '+').replace(/_/g, '/') + '==='.slice((s.length + 3) % 4)), c => c.charCodeAt(0));
const b64urlJson = s => JSON.parse(new TextDecoder().decode(b64urlBytes(s)));
let certCache = { url: '', keys: null, at: 0 };

async function accessKeys(team) {
  const url = team.replace(/\/$/, '') + '/cdn-cgi/access/certs';
  if (certCache.url === url && certCache.keys && Date.now() - certCache.at < 3600e3) return certCache.keys;
  const r = await fetch(url);
  if (!r.ok) throw new HttpError(502, 'No se pudieron leer las claves de Cloudflare Access');
  certCache = { url, keys: (await r.json()).keys || [], at: Date.now() };
  return certCache.keys;
}

async function currentUser(req, env) {
  // Solo para pruebas locales (wrangler dev, archivo .dev.vars): nunca en un dominio público
  const host = new URL(req.url).hostname;
  if (env.DEV_USER && (host === 'localhost' || host === '127.0.0.1')) return env.DEV_USER.toLowerCase();
  if (!env.ACCESS_TEAM_DOMAIN || !env.ACCESS_AUD) {
    throw new HttpError(500, 'Falta configurar Cloudflare Access (ACCESS_TEAM_DOMAIN y ACCESS_AUD en wrangler.toml)');
  }
  // La cabecera la pone Access; la cookie CF_Authorization (el mismo token firmado) es el respaldo. Se verifican igual.
  const cookie = (req.headers.get('Cookie') || '').match(/(?:^|;\s*)CF_Authorization=([^;]+)/);
  const token = req.headers.get('Cf-Access-Jwt-Assertion') || (cookie && cookie[1]);
  if (!token)throw new HttpError(401, 'No autorizado: entra a la estación por su dirección, con tu cuenta');
  const [h, p, sig] = token.split('.');
  let header, payload;
  try { header = b64urlJson(h); payload = b64urlJson(p); } catch (_) { throw new HttpError(401, 'Token inválido'); }
  const jwk = (await accessKeys(env.ACCESS_TEAM_DOMAIN)).find(k => k.kid === header.kid);
  if (!jwk || header.alg !== 'RS256') throw new HttpError(401, 'Token inválido');
  const key = await crypto.subtle.importKey('jwk', jwk, { name: 'RSASSA-PKCS1-v1_5', hash: 'SHA-256' }, false, ['verify']);
  const ok = await crypto.subtle.verify('RSASSA-PKCS1-v1_5', key, b64urlBytes(sig), new TextEncoder().encode(h + '.' + p));
  const aud = Array.isArray(payload.aud) ? payload.aud : [payload.aud];
  const now = Date.now() / 1000;
  if (!ok || !aud.includes(env.ACCESS_AUD) || !(payload.exp > now) || (payload.nbf && payload.nbf > now + 60) ||
      payload.iss !== env.ACCESS_TEAM_DOMAIN.replace(/\/$/, '') || !payload.email) {
    throw new HttpError(401, 'Token inválido o vencido: vuelve a cargar la página');
  }
  return String(payload.email).toLowerCase();
}

const isAdmin = (env, email) => (env.ADMINS || '').toLowerCase().split(',').map(s => s.trim()).filter(Boolean).includes(email);

async function body(req) {
  try { return await req.json(); } catch (_) { throw new HttpError(400, 'El cuerpo debe ser JSON'); }
}
const audit = (env, operator, action, node, device, detail) =>
  env.DB.prepare('INSERT INTO audit (operator, action, node, device_id, detail) VALUES (?, ?, ?, ?, ?)')
    .bind(operator, action, node ?? null, device ?? null, detail ?? null).run();

// ---------- Credenciales nuevas (mismas reglas que tools/DEVEUI_APPKEY_generator) ----------
// EUI de 8 bytes al azar, unicast (bit I/G = 0) y administrado localmente (bit U/L = 1); AppKey de 16 bytes al azar.
// JoinEUI: al azar por dispositivo, o el fijo de la variable JOIN_EUI si se configura.
const toHex = b => [...b].map(x => x.toString(16).padStart(2, '0')).join('').toUpperCase();
function randomEui() {
  const b = crypto.getRandomValues(new Uint8Array(8));
  b[0] = (b[0] & 0xFE) | 0x02;
  return toHex(b);
}
const randomKey = () => toHex(crypto.getRandomValues(new Uint8Array(16)));
const pad3 = n => String(n).padStart(3, '0');

const NODE_LIST_SQL =
  'SELECT n.node, n.dev_eui, n.join_eui, n.notes, n.source, n.created_by, n.created_at, n.platform_at, n.platform_by, ' +
  '       b.device_id, b.assigned_at, b.assigned_by, b.last_seen, ' +
  '       (SELECT t.step || \' \' || t.result FROM tests t WHERE t.node = n.node ORDER BY t.id DESC LIMIT 1) AS last_test, ' +
  '       (SELECT t.created_at FROM tests t WHERE t.node = n.node ORDER BY t.id DESC LIMIT 1) AS last_test_at ' +
  'FROM nodes n LEFT JOIN boards b ON b.node = n.node ORDER BY n.node';

// La tarjeta que tiene ese nodo, si es otra distinta de `device`
async function nodeOwner(env, node, device) {
  const row = await env.DB.prepare('SELECT device_id FROM boards WHERE node = ?').bind(node).first();
  return row && row.device_id !== device ? row.device_id : null;
}

// ---------- Firmware principal (modo superusuario) ----------
const FIRMWARE_MAX_BYTES = 0xE9000 - 0x89000; // tools/pio_ota.py: más grande borraría el contexto de LoRaWAN
const CHUNK_CHARS = 60000;                     // base64 por fila de firmware_chunks
const b64ToBytes = s => Uint8Array.from(atob(s), c => c.charCodeAt(0));
function crc16(data) { // igual que nordicsemi/dfu/crc16.py y tools/build_estacion.py
  let crc = 0xFFFF;
  for (const b of data) {
    crc = ((crc >> 8) & 0x00FF) | ((crc << 8) & 0xFF00);
    crc ^= b;
    crc ^= (crc & 0x00FF) >> 4;
    crc ^= (crc << 8) << 4;
    crc ^= ((crc & 0x00FF) << 4) << 1;
    crc &= 0xFFFF;
  }
  return crc;
}
function bytesInclude(hay, ascii) { // ¿el texto ASCII está en los bytes?
  const n = new TextEncoder().encode(ascii);
  outer: for (let i = 0; i + n.length <= hay.length; i++) {
    for (let j = 0; j < n.length; j++) if (hay[i + j] !== n[j]) continue outer;
    return true;
  }
  return false;
}
const FW_META ='id, kind, version, size, crc16, sha256, built, notes, active, uploaded_by, uploaded_at';

// Un firmware con su .bin (base64): el principal activo, o uno por id
async function firmwareWithBin(env, where, ...args) {
  const fw = await env.DB.prepare('SELECT ' + FW_META + ', dat FROM firmwares WHERE ' + where).bind(...args).first();
  if (!fw) return null;
  const { results } = await env.DB.prepare('SELECT data FROM firmware_chunks WHERE firmware_id = ? ORDER BY seq').bind(fw.id).all();
  return { ...fw, bin: results.map(r => r.data).join('') };
}
const activeFirmware = env => firmwareWithBin(env, "kind = 'main' AND active = 1");

// kind 'main': el firmware principal (la estación lo verifica con [BOOT] y NODE?). kind 'test': un código de prueba
// cualquiera (p. ej. un sketch de Arduino) que solo se carga a una tarjeta; nunca queda activo
async function uploadFirmware(env, user, d) {
  const test = d.kind === 'test';
  const what = test ? 'código de prueba' : 'firmware';
  const version = String(d.version || '').trim();
  if (!/^[\w.\- ]{1,40}$/.test(version)) {
    throw new HttpError(400, (test ? 'Nombre' : 'Versión') + ' inválido (1 a 40 letras, números, punto, guion o espacio)');
  }
  if (/^POST-TEST/i.test(version)) throw new HttpError(400, 'POST-TEST es el firmware de prueba de la estación: usa otro nombre');
  if (test && d.activate) throw new HttpError(400, 'Un código de prueba no puede quedar como firmware principal');
  let bin, dat;
  try { bin = b64ToBytes(String(d.bin || '')); dat = b64ToBytes(String(d.dat || '')); } catch (_) {
    throw new HttpError(400, 'bin y dat deben ir en base64');
  }
  if (!bin.length || bin.length > FIRMWARE_MAX_BYTES) {
    throw new HttpError(400, 'El firmware pesa ' + bin.length + ' B: debe tener entre 1 y ' + FIRMWARE_MAX_BYTES + ' B (más grande borraría el contexto de LoRaWAN)');
  }
  const crc = crc16(bin);
  // Paquete de inicio de la carga (DFU de Nordic): los 2 últimos bytes son el CRC16 del .bin
  if (dat.length < 4 || (dat[dat.length - 2] | (dat[dat.length - 1] << 8)) !== crc) {
    throw new HttpError(400, 'El paquete de inicio (.dat) no corresponde a este .bin (CRC distinto): usa el firmware.zip completo');
  }
  // La estación verifica la carga del principal comparando con lo que imprime la tarjeta: la versión tiene que estar
  // en el firmware. A un código de prueba no se le pide
  if (!test && !bytesInclude(bin, '[BOOT] Firmware ' + version + ' compilado:')) {
    throw new HttpError(400, 'La versión "' + version + '" no aparece en el firmware ("[BOOT] Firmware ' + version + ' compilado:")');
  }
  const sha256 = toHex(new Uint8Array(await crypto.subtle.digest('SHA-256', bin))).toLowerCase();
  const dup = await env.DB.prepare('SELECT id, version FROM firmwares WHERE sha256 = ?').bind(sha256).first();
  if (dup) throw new HttpError(409, 'Ese firmware ya está subido (' + dup.version + ', id ' + dup.id + ')', { id: dup.id });
  const built = /^\d{4}-\d\d-\d\d \d\d:\d\d$/.test(d.built || '') ? d.built : null;
  const notes = d.notes ? String(d.notes).slice(0, 500) : null;

  const row = await env.DB.prepare(
    'INSERT INTO firmwares (kind, version, size, crc16, sha256, built, dat, notes, uploaded_by) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?) RETURNING id')
    .bind(test ? 'test' : 'main', version, bin.length, crc, sha256, built, String(d.dat), notes, user).first();
  const b64 = String(d.bin);
  const stmts = [];
  for (let i = 0, seq = 0; i < b64.length; i += CHUNK_CHARS, seq++) {
    stmts.push(env.DB.prepare('INSERT INTO firmware_chunks (firmware_id, seq, data) VALUES (?, ?, ?)').bind(row.id, seq, b64.slice(i, i + CHUNK_CHARS)));
  }
  if (d.activate) {
    stmts.push(env.DB.prepare("UPDATE firmwares SET active = 0 WHERE kind = 'main' AND active = 1"));
    stmts.push(env.DB.prepare('UPDATE firmwares SET active = 1 WHERE id = ?').bind(row.id));
  }
  try {
    await env.DB.batch(stmts);
  } catch (e) { // sin trozos el firmware no sirve: se quita la fila
    await env.DB.batch([env.DB.prepare('DELETE FROM firmware_chunks WHERE firmware_id = ?').bind(row.id),
                        env.DB.prepare('DELETE FROM firmwares WHERE id = ?').bind(row.id)]);
    throw e;
  }
  await audit(env, user, what + ' subido', null, null,
    version + ' (' + bin.length + ' B, SHA-256 ' + sha256.slice(0, 12) + ', id ' + row.id + ')' + (d.activate ? ' y activado' : ''));
  return { ok: true, id: row.id, kind: test ? 'test' : 'main', version, size: bin.length, crc16: crc, sha256, active: !!d.activate };
}

// ---------- Rutas ----------
async function route(req, env, user) {
  const url = new URL(req.url);
  const parts = url.pathname.replace(/\/+$/, '').split('/').slice(2); // tras /api
  const m = req.method;
  const admin = isAdmin(env, user);

  if (m === 'GET' && parts[0] === 'me' && parts.length === 1) return json({ email: user, admin });

  if (parts[0] === 'nodes' && parts.length === 1) {
    if (m === 'GET') {
      const { results } = await env.DB.prepare(NODE_LIST_SQL).all();
      // ?keys=1: con las AppKey (solo administradores; queda en el registro), para registrarlos en la plataforma
      if (url.searchParams.get('keys') === '1') {
        if (!admin) throw new HttpError(403, 'Solo un administrador puede ver las AppKey');
        const { results: keys } = await env.DB.prepare('SELECT node, app_key FROM nodes').all();
        const byNode = new Map(keys.map(k => [k.node, k.app_key]));
        for (const r of results) r.app_key = byNode.get(r.node);
        await audit(env, user, 'credenciales exportadas', null, null, results.length + ' nodos');
      }
      return json(results);
    }
    if (m === 'POST') {
      if (!admin) throw new HttpError(403, 'Solo un administrador puede agregar nodos');
      const d = await body(req);
      const node = Number(d.node), dev = String(d.dev_eui || '').toUpperCase(), join = String(d.join_eui || '').toUpperCase(),
            key = String(d.app_key || '').toUpperCase();
      if (!isNode(node) || !HEX(16).test(dev) || !HEX(16).test(join) || !HEX(32).test(key)) {
        throw new HttpError(400, 'Datos inválidos: nodo 1-999, DevEUI y JoinEUI de 16 hex, AppKey de 32 hex');
      }
      const clash = await env.DB.prepare('SELECT node FROM nodes WHERE dev_eui = ? AND node <> ?').bind(dev, node).first();
      if (clash) throw new HttpError(409, 'Ese DevEUI ya es del NODO-' + String(clash.node).padStart(3, '0'));
      await env.DB.prepare(
        "INSERT INTO nodes (node, dev_eui, join_eui, app_key, notes, created_by, source) VALUES (?, ?, ?, ?, ?, ?, 'manual') " +
        'ON CONFLICT(node) DO UPDATE SET dev_eui = excluded.dev_eui, join_eui = excluded.join_eui, app_key = excluded.app_key, notes = excluded.notes')
        .bind(node, dev, join, key, d.notes ?? null, user).run();
      await audit(env, user, 'nodo guardado', node, null, 'DevEUI ' + dev);
      return json({ ok: true, node });
    }
  }

  // Nodo nuevo para una tarjeta que no tiene: el siguiente número del consecutivo, credenciales generadas aquí y la
  // tarjeta queda registrada con él (todo en una transacción). Si la tarjeta ya tiene nodo en la base, devuelve ese
  // (así, si la grabación falló, reintentar no gasta otro número).
  if (m === 'POST' && parts[0] === 'nodes' && parts[1] === 'new' && parts.length === 2) {
    const d = await body(req);
    const device = String(d.device || '').toUpperCase();
    if (!isDevice(device)) throw new HttpError(400, 'Falta el DEVICEID de la tarjeta (16 hex)');
    const has = await env.DB.prepare(
      'SELECT n.node, n.dev_eui, n.join_eui, n.app_key FROM boards b JOIN nodes n ON n.node = b.node WHERE b.device_id = ?')
      .bind(device).first();
    if (has) {
      await audit(env, user, 'credenciales leidas', has.node, device, 'la tarjeta ya tenia nodo en la base');
      return json({ ...has, existing: true });
    }
    const fixedJoin = String(env.JOIN_EUI || '').toUpperCase();
    if (fixedJoin && !HEX(16).test(fixedJoin)) throw new HttpError(500, 'JOIN_EUI (wrangler.toml) debe tener 16 caracteres hex');
    let row = null;
    for (let i = 0; i < 5 && !row; i++) {
      const dev = randomEui(), join = fixedJoin || randomEui(), key = randomKey();
      try {
        const res = await env.DB.batch([
          env.DB.prepare(
            "INSERT INTO nodes (node, dev_eui, join_eui, app_key, notes, created_by, source) " +
            "SELECT COALESCE(MAX(node), 0) + 1, ?, ?, ?, ?, ?, 'estacion' FROM nodes RETURNING node, dev_eui, join_eui, app_key")
            .bind(dev, join, key, 'creado por la estación para la tarjeta ' + device, user),
          env.DB.prepare(
            "INSERT INTO boards (device_id, node, assigned_by, assigned_at) VALUES (?, (SELECT node FROM nodes WHERE dev_eui = ?), ?, datetime('now')) " +
            "ON CONFLICT(device_id) DO UPDATE SET node = excluded.node, assigned_by = excluded.assigned_by, assigned_at = excluded.assigned_at, last_seen = datetime('now')")
            .bind(device, dev, user),
        ]);
        row = res[0].results[0];
      } catch (e) {
        if (/CHECK constraint/i.test(e.message)) throw new HttpError(409, 'Se acabaron los números de nodo (1 a 999)');
        if (!/UNIQUE constraint/i.test(e.message)) throw e; // DevEUI repetido o dos estaciones a la vez: otro intento
      }
    }
    if (!row) throw new HttpError(503, 'No se pudo crear el nodo: vuelve a intentarlo');
    await audit(env, user, 'nodo creado', row.node, device, 'DevEUI ' + row.dev_eui);
    return json({ ...row, existing: false });
  }

  // Registrado (o no) en la plataforma LoRaWAN
  if (m === 'POST' && parts[0] === 'nodes' && parts[2] === 'platform' && parts.length === 3) {
    if (!admin) throw new HttpError(403, 'Solo un administrador puede marcar nodos como registrados en la plataforma');
    const node = Number(parts[1]);
    if (!isNode(node)) throw new HttpError(400, 'Número de nodo inválido');
    const d = await body(req);
    const r = d.registered
      ? await env.DB.prepare("UPDATE nodes SET platform_at = datetime('now'), platform_by = ? WHERE node = ?").bind(user, node).run()
      : await env.DB.prepare('UPDATE nodes SET platform_at = NULL, platform_by = NULL WHERE node = ?').bind(node).run();
    if (!r.meta.changes) throw new HttpError(404, 'El NODO-' + pad3(node) + ' no está en la base');
    await audit(env, user, d.registered ? 'registrado en la plataforma' : 'quitado de la plataforma', node, null, null);
    return json({ ok: true, node, registered: !!d.registered });
  }

  if (m === 'GET' && parts[0] === 'nodes' && parts[2] === 'credentials' && parts.length === 3) {
    const node = Number(parts[1]), device = (url.searchParams.get('device') || '').toUpperCase();
    if (!isNode(node)) throw new HttpError(400, 'Número de nodo inválido');
    const row = await env.DB.prepare('SELECT node, dev_eui, join_eui, app_key FROM nodes WHERE node = ?').bind(node).first();
    // Sin tarjeta: un administrador las consulta para darlas de alta en la plataforma
    if (!device && admin) {
      if (!row) throw new HttpError(404, 'El NODO-' + pad3(node) + ' no está en la base');
      await audit(env, user, 'credenciales vistas', node, null, 'para la plataforma');
      return json(row);
    }
    if (!isDevice(device)) throw new HttpError(400, 'Falta el DEVICEID de la tarjeta (16 hex)');
    if (!row) throw new HttpError(404, 'El NODO-' + String(node).padStart(3, '0') + ' no está en la base');
    const owner = await nodeOwner(env, node, device);
    if (owner && !(admin && url.searchParams.get('force') === '1')) {
      throw new HttpError(409, 'El NODO-' + String(node).padStart(3, '0') + ' ya está en otra tarjeta (DEVICEID ' + owner + ')' +
        (admin ? '' : '. Solo un administrador puede reasignarlo.'), { device_id: owner, can_force: admin });
    }
    await audit(env, user, 'credenciales leidas', node, device, owner ? 'reasignacion desde ' + owner : null);
    return json(row);
  }

  if (parts[0] === 'boards' && isDevice((parts[1] || '').toUpperCase())) {
    const device = parts[1].toUpperCase();
    if (m === 'GET' && parts.length === 2) {
      const board = await env.DB.prepare('SELECT * FROM boards WHERE device_id = ?').bind(device).first();
      const { results } = await env.DB.prepare(
        'SELECT id, node, step, result, failures, firmware, operator, created_at FROM tests WHERE device_id = ? ORDER BY id DESC LIMIT 10')
        .bind(device).all();
      return json({ board, tests: results });
    }
    if (m === 'POST' && parts[2] === 'assign' && parts.length === 3) {
      const d = await body(req);
      const node = Number(d.node);
      if (!isNode(node)) throw new HttpError(400, 'Número de nodo inválido');
      if (!(await env.DB.prepare('SELECT 1 FROM nodes WHERE node = ?').bind(node).first())) {
        throw new HttpError(404, 'El NODO-' + String(node).padStart(3, '0') + ' no está en la base');
      }
      const owner = await nodeOwner(env, node, device);
      if (owner && !(admin && d.force)) throw new HttpError(409, 'El nodo ya está en otra tarjeta (DEVICEID ' + owner + ')');
      const stmts = [];
      if (owner) stmts.push(env.DB.prepare('UPDATE boards SET node = NULL WHERE device_id = ?').bind(owner));
      stmts.push(env.DB.prepare(
        "INSERT INTO boards (device_id, node, assigned_by, assigned_at) VALUES (?, ?, ?, datetime('now')) " +
        "ON CONFLICT(device_id) DO UPDATE SET node = excluded.node, assigned_by = excluded.assigned_by, assigned_at = excluded.assigned_at, last_seen = datetime('now')")
        .bind(device, node, user));
      await env.DB.batch(stmts);
      await audit(env, user, 'nodo asignado', node, device, owner ? 'quitado de ' + owner : null);
      return json({ ok: true, device_id: device, node });
    }
  }

  if (parts[0] === 'tests' && parts.length === 1) {
    if (m === 'POST') {
      const d = await body(req);
      const device = String(d.device_id || '').toUpperCase();
      if (!isDevice(device)) throw new HttpError(400, 'Falta el DEVICEID de la tarjeta (16 hex)');
      if (!['montaje', 'principal'].includes(d.step) || !['OK', 'FALLA'].includes(d.result)) {
        throw new HttpError(400, 'step debe ser montaje/principal y result OK/FALLA');
      }
      const node = isNode(Number(d.node)) ? Number(d.node) : null;
      const failures = Array.isArray(d.failures) ? d.failures.join(', ') : (d.failures || null);
      const results = d.results == null ? null : JSON.stringify(d.results).slice(0, 20000);
      await env.DB.batch([
        env.DB.prepare('INSERT INTO tests (device_id, node, step, result, failures, firmware, results, operator) VALUES (?, ?, ?, ?, ?, ?, ?, ?)')
          .bind(device, node, d.step, d.result, failures, d.firmware ?? null, results, user),
        env.DB.prepare("INSERT INTO boards (device_id) VALUES (?) ON CONFLICT(device_id) DO UPDATE SET last_seen = datetime('now')").bind(device),
      ]);
      return json({ ok: true });
    }
    if (m === 'GET') {
      const limit = Math.min(500, Math.max(1, Number(url.searchParams.get('limit')) || 50));
      const { results } = await env.DB.prepare(
        'SELECT id, device_id, node, step, result, failures, firmware, operator, created_at FROM tests ORDER BY id DESC LIMIT ?')
        .bind(limit).all();
      return json(results);
    }
  }
  if (parts[0] === 'firmware') {
    // El activo lo pide la página de todos (es el que se carga en las tarjetas)
    if (m === 'GET' && parts[1] === 'main' && parts.length === 2) return json(await activeFirmware(env));
    // Los programas de prueba los puede cargar cualquier operador desde la estación (subirlos y borrarlos es del admin)
    if (m === 'GET' && parts[1] === 'tests' && parts.length === 2) {
      const { results } = await env.DB.prepare('SELECT ' + FW_META + " FROM firmwares WHERE kind = 'test' ORDER BY id DESC").all();
      return json(results);
    }
    if (m === 'GET' && parts.length === 2 && /^\d+$/.test(parts[1]) && !admin) {
      const fw = await firmwareWithBin(env, "id = ? AND kind = 'test'", Number(parts[1]));
      if (!fw) throw new HttpError(404, 'Ese programa de prueba no está en la base');
      return json(fw);
    }
    if (!admin) throw new HttpError(403, 'Solo un superusuario (administrador) puede cambiar el firmware principal');
    if (m === 'GET' && parts.length === 1) {
      const { results } = await env.DB.prepare('SELECT ' + FW_META + ' FROM firmwares ORDER BY id DESC').all();
      return json(results);
    }
    if (m === 'GET' && parts.length === 2 && /^\d+$/.test(parts[1])) { // con el .bin, para cargarlo a una tarjeta
      const fw = await firmwareWithBin(env, 'id = ?', Number(parts[1]));
      if (!fw) throw new HttpError(404, 'Ese firmware no está en la base');
      return json(fw);
    }
    if (m === 'POST' && parts.length === 1) return json(await uploadFirmware(env, user, await body(req)));
    if (m === 'POST' && parts[1] === 'active' && parts.length === 2) {
      const d = await body(req);
      if (d.id == null) { // volver al de la publicación
        await env.DB.prepare("UPDATE firmwares SET active = 0 WHERE kind = 'main' AND active = 1").run();
        await audit(env, user, 'firmware activado', null, null, 'el de la publicación');
        return json({ ok: true, id: null });
      }
      const id = Number(d.id);
      const fw = Number.isInteger(id) && await env.DB.prepare("SELECT id, version FROM firmwares WHERE id = ? AND kind = 'main'").bind(id).first();
      if (!fw) throw new HttpError(404, 'Ese firmware no está en la base');
      await env.DB.batch([
        env.DB.prepare("UPDATE firmwares SET active = 0 WHERE kind = 'main' AND active = 1"),
        env.DB.prepare('UPDATE firmwares SET active = 1 WHERE id = ?').bind(id),
      ]);
      await audit(env, user, 'firmware activado', null, null, fw.version + ' (id ' + id + ')');
      return json({ ok: true, id });
    }
    if (m === 'DELETE' && parts.length === 2) {
      const id = Number(parts[1]);
      const fw = Number.isInteger(id) && await env.DB.prepare('SELECT id, kind, version, active FROM firmwares WHERE id = ?').bind(id).first();
      if (!fw) throw new HttpError(404, 'Ese firmware no está en la base');
      if (fw.active) throw new HttpError(409, 'Es el firmware activo: activa otro antes de borrarlo');
      await env.DB.batch([env.DB.prepare('DELETE FROM firmware_chunks WHERE firmware_id = ?').bind(id),
                          env.DB.prepare('DELETE FROM firmwares WHERE id = ?').bind(id)]);
      await audit(env, user, (fw.kind === 'test' ? 'código de prueba' : 'firmware') + ' borrado', null, null, fw.version + ' (id ' + id + ')');
      return json({ ok: true });
    }
  }
  throw new HttpError(404, 'Ruta no encontrada');
}

export default {
  async fetch(req, env) {
    const url = new URL(req.url);
    if (!url.pathname.startsWith('/api/')) return env.ASSETS ? env.ASSETS.fetch(req) : new Response('No encontrado', { status: 404 });
    try {
      const user = await currentUser(req, env);
      return await route(req, env, user);
    } catch (e) {
      if (e instanceof HttpError) return json({ error: e.message, ...(e.extra || {}) }, e.status);
      return json({ error: 'Error interno: ' + e.message }, 500);
    }
  },
};
