# Estación de montaje Wearme en Cloudflare

La misma página de la estación (`tools/diagnostico.html`), publicada en internet con inicio de sesión y conectada a una
base de datos:

- **Workers + assets:** sirve la página y los firmwares por HTTPS (Web Serial lo exige) y la API (`src/index.js`).
- **D1:** la base (SQLite). Guarda los nodos y sus credenciales, las tarjetas (por el DEVICEID del nRF52840), cada prueba y quién hizo qué.
- **Access:** pide iniciar sesión antes de entregar cualquier cosa: la página, los firmwares y la API. La API además verifica la firma del token de Access en cada petición.

Todo cabe en el plan gratuito.

## Qué cambia frente a la página local

| | Archivo local (`Estacion_Wearme.html`) | Web |
|---|---|---|
| Credenciales que se graban | Tabla compilada en el firmware de prueba (`NODE:SET`) | Las de la base (`LORA:CRED:SET` + `NODE:ID`) |
| Mismo nodo en dos tarjetas | No se detecta | Bloqueado; solo un administrador puede reasignar |
| Registro de pruebas | No | Cada montaje (OK/FALLA, con el detalle) y cada carga del principal |
| Nodos nuevos | Editar `wearme_credentials.h` y recompilar | Un administrador los agrega desde la página |

## Instalación (una vez)

Todo desde esta carpeta (`estacion_web`). `wrangler` es la herramienta de Cloudflare; se usa con `npx`, sin instalar nada ni llenar OneDrive con `node_modules`.

1. **Cuenta:** crear una cuenta gratuita en https://dash.cloudflare.com.
2. **Iniciar sesión** (abre el navegador):
   ```
   npx wrangler@4.156.0 login
   ```
3. **Crear la base:**
   ```
   npx wrangler@4.156.0 d1 create wearme-estacion
   ```
   Copiar el `database_id` que imprime en `wrangler.toml`, en lugar de `PONER-AQUI-EL-ID`.
4. **Crear las tablas:**
   ```
   npx wrangler@4.156.0 d1 migrations apply wearme-estacion --remote
   ```
5. **Cargar las credenciales** de `include/wearme_credentials.h` y del generador de escritorio (`tools/DEVEUI_APPKEY_generator/credenciales_lorawan.db`):
   ```
   python ../tools/importar_credenciales.py
   ```
   Ejecutar el comando que imprime y después **borrar el archivo SQL** (trae las AppKey).
6. **Generar la página y los firmwares:**
   ```
   python ../tools/build_estacion.py --web
   ```
7. **Publicar primero SIN firmwares** (una página provisional), para que nunca estén en internet sin Access:
   ```
   mkdir %TEMP%\provisional && echo Estacion en configuracion > %TEMP%\provisional\index.html
   npx wrangler@4.156.0 deploy --assets %TEMP%\provisional
   ```
   Imprime la dirección, del tipo `https://wearme-estacion.<subdominio>.workers.dev` (el certificado HTTPS de un subdominio nuevo tarda unos minutos). La API responde 500 hasta el paso 9. `wrangler.toml` trae `preview_urls = false`: no se crean direcciones paralelas sin Access.
8. **Proteger con Access** (Cloudflare Zero Trust, plan Free):
   1. En el panel: **Zero Trust**. La primera vez pide un nombre de equipo, que define `https://<equipo>.cloudflareaccess.com` (se ve en **Zero Trust → Settings → Team domain**).
   2. **Workers & Pages → wearme-estacion → pestaña Access → Protect this Worker behind Access → All traffic.** Protege a la vez `workers.dev`, rutas y vistas previas.
   3. **Authentication policy:** los correos autorizados, o todo el dominio institucional (*Emails ending in* `@javerianacali.edu.co`). Luego **Apply Access**.
   4. **Método de inicio de sesión:** "One-time PIN" (llega un código al correo) funciona sin más configuración. También se puede conectar Microsoft (Entra ID).
   5. En **Zero Trust → Access → Applications** aparece "wearme-estacion - Cloudflare Workers": copiar su **Application Audience (AUD) Tag**.
   6. Comprobar: sin sesión, `https://wearme-estacion.<subdominio>.workers.dev/` debe redirigir a `<equipo>.cloudflareaccess.com`.
9. **Configurar la API** en `wrangler.toml`:
   ```toml
   ACCESS_TEAM_DOMAIN = "https://<equipo>.cloudflareaccess.com"
   ACCESS_AUD = "<el AUD tag>"
   ADMINS = "tu.correo@javerianacali.edu.co"
   ```
   y publicar la estación completa (ahora sí con los firmwares de `public/`): `npx wrangler@4.156.0 deploy`.
10. **Probar:** abrir la dirección en Edge o Chrome. Debe pedir el inicio de sesión, y luego mostrar tu correo arriba a la derecha (con "(admin)" si estás en `ADMINS`).

Revisa en la página de precios de Zero Trust cuántos usuarios incluye el plan gratuito.

## Uso diario

- **Firmware nuevo (de prueba o principal):** `python ../tools/build_estacion.py --web` y luego `npx wrangler@4.156.0 deploy`. El script necesita el `firmware.zip` de **los dos** proyectos en su carpeta de compilación (`%TEMP%\pio_build_wearme*`); si se limpió el del principal, hay que compilarlo de nuevo. La estación carga sola el firmware de prueba publicado en cada tarjeta (paso 1 de «Iniciar prueba y carga», o «Solo cargar firmware de prueba»): no hay que cargarlo a mano.
- **Tarjeta con otro programa** (p. ej. un sketch de Arduino, que se anuncia como `0x239A:0x8029`): la estación la pasa sola a modo carga, porque cualquier PID `0x8xxx` es un programa corriendo y recibe el toque de 1200 baudios. La primera vez el navegador pide autorizar ese puerto. El doble RESET solo hace falta si el programa no tiene USB o está colgado.
- **Modo superusuario (cambiar el firmware principal sin publicar):** un administrador (`ADMINS`) ve el botón
  **Superusuario** arriba en la estación. Ahí sube el `.zip` de carga del firmware: con PlatformIO, `firmware.zip`
  (carpeta de compilación `seeed_wio_tracker_1110\firmware.zip`); con Arduino (núcleo Seeed o Adafruit nRF52),
  `<sketch>.ino.zip` (Programa → Exportar binario compilado, carpeta `build` del sketch: los dos núcleos lo generan con
  `adafruit-nrfutil dfu genpkg`, el mismo formato). El `.hex` o el `.uf2` solos no sirven. Lo deja **en uso** para todas las estaciones al
  instante. Guarda las versiones subidas: se puede volver a cualquiera o al **de la publicación** (el que trae
  `public/firmware/firmwares.json`). La página y la API revisan que el `.zip` esté completo (CRC), que no pase el
  límite de tamaño, que no sea el firmware de prueba y que la versión esté en el firmware (`[BOOT] Firmware <versión>
  compilado:`): la estación la compara con lo que imprime la tarjeta, así que **cada firmware nuevo debe tener su
  propio `WEARME_FW_VERSION`**. Cada cambio queda en `audit`. Necesita las migraciones `0003_firmwares.sql` y
  `0004_firmware_test.sql` (`npx wrangler@4.156.0 d1 migrations apply wearme-estacion --remote`).
- **Códigos de prueba (en el mismo modo superusuario):** con la casilla **«Es un código de prueba»** se sube un
  programa que no tiene que ver con el principal (p. ej. un sketch de Arduino que prueba un sensor). No necesita la
  línea `[BOOT]`, se le pone un nombre y **nunca queda en uso** como principal. Se guarda en la lista «Códigos de
  prueba» y su botón **«Cargar en la tarjeta»** lo graba en la tarjeta conectada, sin POST, sin nodo y sin verificar
  la versión. Para volver a dejarla de collar se usa después «Iniciar prueba y carga». Si hay alguno subido, la
  pantalla principal muestra también **«Cargar programa de prueba»** en «Opciones y firmwares de esta página» (para
  cualquier operador; subirlos y borrarlos sigue siendo solo del superusuario). La página acepta también la
  carpeta `build` de Arduino comprimida entera: saca de ahí el `<sketch>.ino.zip`.
- **Tarjeta nueva:** con el nodo en **Automático**, la estación le crea el **siguiente número** (el más alto + 1; no
  reutiliza huecos) con credenciales generadas como `tools/DEVEUI_APPKEY_generator` (DevEUI y JoinEUI al azar con
  U/L = 1 e I/G = 0, AppKey de 16 bytes), lo guarda en la base y lo graba en la tarjeta. Si la tarjeta ya tiene nodo
  (en la base o en su flash) lo conserva; si tiene credenciales que no están en la base, se detiene y pregunta.
  Si el servidor de red exige un JoinEUI fijo, ponerlo en `JOIN_EUI` (`wrangler.toml`).
- **Nodos registrados** (`nodos.html`, enlace arriba en la estación): todos los DevEUI, qué tarjeta tiene cada nodo, la
  última prueba, el **siguiente número** y cuáles **faltan por registrar en la plataforma**. Un administrador ve las
  credenciales de cada nodo (para darlo de alta en el servidor de red), lo marca como registrado y puede descargar un
  CSV con las AppKey (queda en `audit`; borrar el archivo después).
- **Nodo con credenciales propias:** un administrador, en la estación: **Cambiar nodo o credenciales… → Opción 2**.
- **El generador de escritorio** (`tools/DEVEUI_APPKEY_generator`) ya no hace falta para nodos nuevos: la base es la
  fuente del consecutivo. Lo que se haya generado con él se pasa con `python ../tools/importar_credenciales.py` (no
  toca un nodo que ya esté en la base con otro DevEUI).
- **Consultar la base:** en el panel de Cloudflare (**D1 → wearme-estacion → Console**) o con `npx wrangler@4.156.0 d1 execute wearme-estacion --remote --command "..."`. Ejemplos:
  ```sql
  -- últimas pruebas
  SELECT created_at, device_id, node, step, result, failures, operator FROM tests ORDER BY id DESC LIMIT 50;
  -- qué tarjeta tiene cada nodo
  SELECT n.node, n.dev_eui, b.device_id, b.assigned_at, b.assigned_by FROM nodes n LEFT JOIN boards b ON b.node = n.node;
  -- quién leyó credenciales
  SELECT * FROM audit ORDER BY id DESC LIMIT 50;
  ```

## Problemas de acceso

| Síntoma | Causa | Qué hacer |
|---|---|---|
| **«Invalid state»** en la pantalla de Cloudflare | El inicio de sesión no coincide con la cookie de ese intento. Pasa con una pestaña de login vieja, al volver atrás, al abrir el enlace del correo en otro navegador o al tardar mucho | Cerrar la pestaña y abrir de nuevo la dirección de la estación (no una `…cloudflareaccess.com/…` del historial). Si sigue, ventana InPrivate o borrar las cookies de `workers.dev` y `cloudflareaccess.com` |
| **«La sesión de Cloudflare venció»** (antes salía «Failed to fetch») al subir o guardar | Venció la sesión de Access mientras la página estaba abierta. Access redirige a su login y el navegador bloquea esa redirección | Recargar (F5), entrar de nuevo y repetir. La duración se cambia en **Access → Applications → wearme-estacion → Session duration** |
| **Un compañero no puede entrar** | La política deja entrar a todo `@javerianacali.edu.co`. Si en **Zero Trust → Logs → Access** no aparece ni «Allowed» ni «Blocked» para su correo (quitar filtros y ampliar el rango de fechas), nunca terminó de verificar el correo | Que busque el PIN de `noreply@notify.cloudflare.com` en spam o cuarentena (Microsoft 365 los retiene), y que use el correo institucional. Si el correo no llega, agregar **Microsoft (Entra ID)** en **Login methods**. El *Policy tester* solo evalúa a quien ya entró alguna vez, así que no sirve para alguien nuevo |
| Un compañero entra pero no ve **Superusuario** | Solo los correos de `ADMINS` (`wrangler.toml`) son administradores | Agregarlo a `ADMINS` y `npx wrangler@4.156.0 deploy` |

## La API

| Ruta | Quién | Qué hace |
|---|---|---|
| `GET /api/me` | todos | Correo y si es administrador |
| `GET /api/nodes` | todos | Nodos, su tarjeta, última prueba, origen y plataforma (sin AppKey). `?keys=1`: con AppKey (admin, queda en `audit`) |
| `GET /api/nodes/:n/credentials?device=ID` | todos | Credenciales para grabar el nodo `n` en esa tarjeta. 409 si el nodo está en otra; un administrador puede forzar con `&force=1`. Sin `device`: solo admin (para la plataforma). Queda en `audit` |
| `POST /api/nodes` | admin | Agrega o corrige un nodo |
| `POST /api/nodes/new` `{device}` | todos | Nodo nuevo (siguiente número, credenciales generadas) para esa tarjeta, ya asignado. Si la tarjeta tiene nodo, devuelve ese (`existing: true`) |
| `POST /api/nodes/:n/platform` `{registered}` | admin | Marca el nodo como registrado (o pendiente) en la plataforma |
| `POST /api/boards/:ID/assign` | todos (forzar: admin) | Registra que la tarjeta quedó con ese nodo |
| `GET /api/boards/:ID` | todos | Tarjeta y sus últimas pruebas |
| `POST /api/tests`, `GET /api/tests` | todos | Registra una prueba / últimas pruebas |
| `GET /api/firmware/main` | todos | Firmware principal en uso desde la base (`null`: se usa el de la publicación) |
| `GET /api/firmware` | admin | Firmwares subidos, principales (`kind: main`) y códigos de prueba (`kind: test`), sin el `.bin` |
| `GET /api/firmware/tests` | todos | Programas de prueba subidos (sin el `.bin`): la estación muestra «Cargar programa de prueba» si hay alguno |
| `GET /api/firmware/:id` | admin; todos si es de prueba | Uno con su `.bin`, para cargarlo a una tarjeta |
| `POST /api/firmware` `{kind, version, built, notes, bin, dat, activate}` | admin | Sube uno (`bin`/`dat` en base64, del `.zip` de carga; `kind: 'test'` = código de prueba, sin `[BOOT]` ni `activate`) |
| `POST /api/firmware/active` `{id}` | admin | Elige el que está en uso; `{id: null}` vuelve al de la publicación |
| `DELETE /api/firmware/:id` | admin | Borra uno que no esté en uso |

## Pruebas en este PC (sin Cloudflare)

Desde la carpeta del proyecto (`Wearme_POST_Test`):

```
powershell -ExecutionPolicy Bypass -File estacion_web\probar_local.ps1
```

1. Crea una base local en la carpeta temporal (fuera de OneDrive) con las credenciales de `include/wearme_credentials.h`.
2. Simula el inicio de sesión como administrador. Solo funciona en `localhost`; con `-Correo tu@correo` se elige el correo.
3. Abre la estación en **http://localhost:8787**.

La primera vez tarda un poco más (descarga `wrangler`). **Ctrl+C** para terminar. La base local se conserva entre pruebas; con `-Limpiar` se borra y se vuelve a crear.

## Notas de seguridad

- Las AppKey quedan en D1, que Cloudflare cifra en reposo; la API solo entrega la del nodo que se está grabando, y deja registro. Si se quiere, se pueden cifrar además con un secreto del Worker.
- `public/firmware/firmwares.json` trae el firmware principal, que todavía tiene compiladas las claves de la tabla. Por eso **todas** las direcciones de la estación deben estar detrás de Access. El siguiente paso recomendado es quitar la tabla de los dos firmwares: con la base, ya no hace falta.
