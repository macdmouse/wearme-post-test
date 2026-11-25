# Wearme POST Test

Firmware de banco para la placa Wearme (nRF52840 + LR1110). Hace dos cosas:

1. Inicializa la placa y corre el **POST** (prueba de todos los subsistemas).
2. Muestra que **el micro está vivo** (LED y puerto serie).

No hace nada más. No tiene máquina de estados, mediciones, Join LoRaWAN ni transmisiones. Tampoco usa el anillo Meshtastic ni escribe en la SD.

Las pruebas parten de las del firmware principal (`Wearme_Firmware_V3.1`, versión 3.1.1). Se copiaron `lib/board_nrf52` y `lib/post_lib`, más las librerías de los chips. Si una prueba cambia allá, hay que volver a copiarla aquí.

Versión actual: **`POST-TEST 1.4`** (`WEARME_FW_VERSION` en `lib/board_nrf52/board_nrf52.h`). Desde la 1.1 este proyecto tiene pruebas más estrictas que el principal y todavía **no se han pasado allá**:

- **BATTERY** detecta si hay batería y si está mala, también con USB (ver [Pruebas que difieren del principal](#pruebas-que-difieren-del-principal)).
- **POWER** detecta un CTRL_GPS pegado a VDD desde el primer POST y suelta el pin.
- **WIND** es crítica y dice qué sensor falla, el derecho o el izquierdo.
- Hay un comando nuevo, `BAT:LOG`.

El principal tiene además `WIND_POWERUP_MS = 100`, que es poco: los FS3000 responden ceros hasta ~1 s después de encender.

## Uso

```
pio run -t upload        # por USB
pio device monitor       # 115200 baudios
```

Al arrancar, el firmware hace lo siguiente:

1. Imprime la identificación del chip (DEVICEID).
2. Corre la inicialización `[INIT]`.
3. Corre el POST completo, con una línea por subsistema y un resumen al final.
4. Apaga el GPS que el POST dejó encendido.
5. Imprime una línea `[VIVO]` cada 10 s.

## ¿Está vivo?

| LED | Significa |
|---|---|
| Los tres encendidos | Arrancando (antes del POST) |
| 3 destellos verdes | El POST pasó |
| **Destello verde cada 2 s** | Vivo, sin fallas |
| **Destello rojo cada 2 s** | Vivo, pero hay al menos un subsistema con falla (ver `[VIVO]`) |
| Fijo o apagado, sin destellar | Colgado. El watchdog lo reinicia a los 60 s |

Línea por el puerto serie (valores de ejemplo):

```
[VIVO] 0:05:30 encendido | loop 196 /s | CPU 27.25 C | pila libre min 2716 B | POST OK (1 corrida)
[VIVO] 0:05:40 encendido | loop 196 /s | CPU 27.50 C | pila libre min 2716 B | FALLAS: SD CARD, GPS (L96-M33) (POST:<subsistema> para repetir una prueba)
```

Si el tiempo encendido avanza y la temperatura del nRF52840 cambia, el micro está trabajando. Si el contador vuelve a 0:00:00, hubo un reinicio. La causa sale en el arranque y en la prueba WATCHDOG del POST.

Cada prueba actualiza su propia falla. Si `POST:SD` falla, el LED pasa a rojo. Si la tarjeta se inserta y `POST:SD` pasa, la falla se borra y el LED vuelve a verde.

## Estación de montaje (cargar firmware sin PlatformIO)

Para que quien prueba las tarjetas cargue el firmware de prueba y después el principal, sin código ni PlatformIO.

**Preparar la página (una vez por versión de firmware, en el computador de desarrollo):**

```
python tools/build_estacion.py            # compila los dos proyectos y genera la página
python tools/build_estacion.py --no-build # usa lo último compilado
```

El resultado es `tools/estacion/Estacion_Wearme.html`, un solo archivo con los dos firmwares adentro: este (`POST-TEST`) y el de `Wearme_Firmware_V3.1`. El script se detiene si algún firmware pasa el límite que borraría el contexto de LoRaWAN, si el CRC no coincide o si `include/wearme_credentials.h` no es igual en los dos proyectos.

> **Confidencial:** la página trae el firmware principal, y con él las claves LoRaWAN de todos los nodos. No publicarla ni enviarla por canales abiertos.

**Uso en la estación (Edge o Chrome):**

1. Abrir `Estacion_Wearme.html` y conectar la tarjeta por USB.
2. Elegir el **nodo** de la tarjeta (si ya tiene uno grabado, se elige solo) y pulsar **Iniciar prueba y carga**.
3. La página hace sola los cuatro pasos:
   1. **Firmware de prueba:** reinicia la tarjeta en modo carga (toque a 1200 baudios, como PlatformIO) y lo carga por el bootloader. Una tarjeta sin programa ya está en modo carga.
   2. **Revisar montaje:** espera el resultado del POST. Si hay fallas se detiene: corregir, **Probar todo de nuevo** y **Continuar**.
   3. **Grabar nodo:** `NODE:SET=<n>` con el firmware de prueba (credenciales de la tabla y número de nodo en la flash).
   4. **Firmware principal:** lo carga y verifica con `NODE?` y `BOOT?` que quedaron el nodo y la versión correctos.
4. Al ver **LISTO: NODO-xxx con el firmware principal x.y.z**, desconectar.

La primera vez el navegador pide autorizar dos puertos: el de la tarjeta (PID 0x8055) y el del bootloader (PID 0x0055). Son el mismo USB, y el navegador los recuerda. Si la tarjeta tiene otro programa (p. ej. un sketch de Arduino, que se anuncia como `0x239A:0x8029` "Feather nRF52840"), la estación también la pasa sola a modo carga (cualquier PID 0x8xxx es un programa corriendo); la primera vez pide autorizar ese puerto. Solo hace falta el doble RESET si ese programa no tiene USB o está colgado. Si algo se queda esperando, **Cancelar** detiene la estación. Una carga interrumpida deja la tarjeta en modo carga, lista para reintentar.

Antes de cargar el principal, la estación verifica que la tarjeta tenga credenciales LoRaWAN en la flash; si no, se detiene. Si la tarjeta tiene un nodo con credenciales propias (fuera de la tabla), lo conserva.

**Por qué se graba el nodo antes del firmware principal:** el principal, en el primer arranque de una tarjeta nueva, graba las credenciales del nodo con el que se compiló (`WEARME_NODE` en `include/wearme_node.h`) y se une a LoRaWAN con ellas. Si se cargara directo, todas las tarjetas nuevas serían ese nodo. Con el nodo ya grabado, el principal lo conserva.

**Requisito:** la tarjeta debe tener el bootloader (el de fábrica, con el SoftDevice S140). Un nRF52840 completamente virgen, sin bootloader, no aparece por USB: hay que grabarle el bootloader una vez con un programador SWD (J-Link o similar).

La carga usa el mismo protocolo que `pio run -t upload` (DFU serie de adafruit-nrfutil). La página genera exactamente los mismos bytes que adafruit-nrfutil para los dos firmwares.

## Estación web (Cloudflare)

La misma estación, publicada en internet:

- **Dirección:** https://wearme-estacion.estacion-web.workers.dev
- **Acceso:** inicio de sesión con Cloudflare Access.
- **Datos:** una base D1. Las credenciales salen de ella, no se puede poner el mismo nodo en dos tarjetas y cada prueba queda registrada con su operador.
- **Código y guía:** en `estacion_web/`, con su guía de instalación en [estacion_web/README.md](estacion_web/README.md).
- **Generación:** `python tools/build_estacion.py --web`. Las credenciales se pasan a la base con `python tools/importar_credenciales.py`.

Lo que solo tiene la versión web:

- **Modo superusuario** (administradores): cambiar el firmware principal subiendo su `.zip` de PlatformIO o de Arduino, sin volver a publicar.
- **Programas de prueba:** subir un código cualquiera, por ejemplo un sketch de Arduino que prueba un sensor. Cualquier operador lo carga con **Cargar programa de prueba** en «Opciones y firmwares de esta página».

Para eso `NODE?` del firmware de prueba también devuelve el DEVICEID del nRF52840, que identifica cada tarjeta física.

## Interfaz gráfica `tools/diagnostico.html`

Para quien no lee el monitor serie: doble clic en `tools/diagnostico.html` (se abre en Edge o Chrome; Firefox no sirve porque no tiene puerto serie), conectar la placa por USB y pulsar **Conectar placa**. No instala nada ni necesita internet.

Es la **plantilla** de la estación: no trae firmwares, así que no puede cargar (lo dice en la página). Sirve para diagnosticar, repetir pruebas y cambiar el nodo o las credenciales. Para cargar firmware se usa `tools/estacion/Estacion_Wearme.html`.

- **Guía de uso** (botón arriba): tutorial paso a paso, problemas frecuentes y el significado del LED. Se abre sola la primera vez.
- Dice en grande si el montaje está OK, qué está probando o qué falla y qué revisar primero.
- **Probar todo de nuevo** repite las 15 pruebas; para repetir una sola, se toca su tarjeta y **Repetir esta prueba**.
- **Identidad y LoRaWAN**: muestra el nodo, el DevEUI, el JoinEUI y la AppKey (oculta, con botón para mostrarla). Con **Cambiar nodo o credenciales…**:
  - *Opción 1:* un nodo de la tabla (`NODE:SET=<n>`).
  - *Opción 2:* credenciales propias para un dispositivo fuera de la tabla (`LORA:CRED:RESET`, `LORA:CRED:SET`, `NODE:ID=<n>`). Necesita el firmware de prueba.
- Muestra una tarjeta por subsistema; al tocarla, explica qué es, qué revisar y el detalle de la última prueba, con un botón para repetirla.
- Muestra la salud del micro y un historial de lo que pasó (fallas, reinicios). Si la placa se reinicia, se reconecta sola.
- **Ver ejemplo** (o abrir `diagnostico.html#ejemplo`) simula una placa con la microSD fallando, para practicar sin hardware.
- Cerrar el monitor de PlatformIO antes: el puerto COM solo lo puede abrir un programa a la vez.

Lee las mismas líneas que el monitor (`[POST]`, `[TABLA]`, `[VIVO]`). Si se cambian esos textos en el firmware, hay que ajustar el bloque PARSER del HTML.

## Tabla de diagnóstico `[TABLA]`

Sale al arrancar, cada vez que cambian las fallas (una prueba suelta que falla o vuelve a pasar, el chequeo del RTC a los 5 s), al terminar un `POST:ALL` y cada 60 s. Tiene tres partes:

1. **Subsistemas**: uno por fila, en el orden del POST (dependencia), con código, política (`CRIT`/`TOL`/`OFF`/`SIM`), estado (`OK`, `FALLA`, `FALLA SIM`, `TOLERADA`, `OMITIDA`, `SIN CORRER`), corridas fallidas/total y hace cuánto se probó.
2. **QUÉ REVISAR**: solo los que fallan, con el comando para repetir la prueba y qué mirar en la placa. Hay que arreglar primero el de más arriba: una falla del bus I2C o del expansor hace fallar a los de abajo.
3. **Micro (nRF52840)**: vueltas del loop por segundo, pila libre mínima, temperatura, reinicios por watchdog y VBUS, con `ALERTA` y qué revisar si se salen de rango.

Las pistas de cada subsistema están en la columna `check` de `post_tests[]` (`lib/post_lib/post_lib.cpp`); los umbrales del micro, al inicio de la sección de la tabla en `src/main.cpp`.

## Comandos (puerto serie)

| Comando | Qué hace |
|---|---|
| `POST:ALL` | POST completo |
| `POST:<subsistema>` | Una sola prueba: `I2C`, `EXPANDER`, `POWER`, `WDT`, `BATTERY`, `RTC`, `SD`, `IMU`, `ENV`, `CH4`, `GPS`, `WIND`, `BLOWER`, `LORA`, `BT` (o el código, ej. `POST:0x0B`) |
| `VIVO?` / `VIVO:<s>` | Estado ahora / periodo de la línea `[VIVO]` (`VIVO:0` la apaga) |
| `TABLA?` / `TABLA:<s>` | Tabla de diagnóstico ahora / periodo de la tabla (60 s por defecto; `TABLA:0` = solo cuando cambian las fallas) |
| `POLICY:<token>=CRIT\|TOL\|OFF\|SIM`, `POLICY:LIST` | Política de falla de un subsistema (hasta reiniciar) |
| `RTC:SET AAAA-MM-DD HH:MM:SS` | Pone el RTC en hora (hora local) |
| `FAN:ON` / `FAN:OFF` | Blower |
| `CH4:ON` / `CH4:OFF` | Calefactor del sensor CH4 |
| `BAT:LOG` / `BAT:LOG=<s>` | Lecturas del MAX17048 cada 250 ms durante `s` segundos (10 por defecto, hasta 120) y un resumen: voltaje mínimo y máximo, cuánto varía, SOC y C-rate. Muestra también sus registros. Desactiva la hibernación del medidor mientras mide y la deja como estaba. Sirve para comparar una tarjeta con y sin batería |
| `WDT:HANG` / `WDT:SW` | Prueba del watchdog: cuelga el loop (reinicia a los 60 s) o reinicio de software |
| `NODE?` / `BOOT?` | Identidad del collar (dice si está guardada en la flash) / versión del firmware (`POST-TEST 1.4`) y del bootloader |
| `NODE:SET=<n>` | Graba el nodo `n` con sus credenciales de la tabla (`include/wearme_credentials.h`) |
| `NODE:ID=<n>` | Graba solo el número de nodo (dispositivo con credenciales propias) |
| `LORA:CRED?` / `LORA:CRED:RESET` / `LORA:CRED:SET <DevEUI> <JoinEUI> <AppKey>` | Credenciales LoRaWAN: ver / borrar de la flash / grabar (solo si no hay guardadas) |
| `DFU:OTA` / `DFU:UF2` / `DFU:SERIAL` | Bootloader: Bluetooth (`pio run -t ota`), unidad USB o serie |

## Pruebas que difieren del principal

### BATTERY: sin batería o batería mala es FALLA

**Qué pasa sin batería.** Con el USB conectado y sin batería, el MAX17048 mide la salida del cargador. Esa salida sube y baja en diente de sierra: de 4.08 a 4.23 V en unos 11 s, cae y vuelve a subir.

**Por qué no basta una lectura.** El SOC y el C-rate cambian demasiado lento para decidir en unos segundos. Además, si el medidor está hibernando, solo mide cada 45 s, y con una sola lectura parece una batería normal: así fue como pasaban tarjetas sin batería.

**Cómo prueba ahora.** Desactiva la hibernación y lee el voltaje cada 250 ms durante **6 s** (`BAT_CHECK_MS`). Después deja HIBRT como estaba. Luego decide:

| Lectura | Resultado |
|---|---|
| Varía **30 mV o más** (`BAT_ABSENT_SWING_MV`), o SOC/C-rate imposibles | **FALLA · SIN BATERIA** |
| Menos de **3.0 V** aun cargando por USB (`BAT_BAD_MV`) | **FALLA · BATERIA MALA O MUY DESCARGADA** |
| Fuera de 2.5-4.5 V, o el chip no responde | **FALLA** |
| Estable | **OK · Bateria conectada** |

**Cómo se eligió el límite.** Medido en la placa el 2026-11-25, con USB:

| Caso | Corridas | Variación en 6 s | Resultado |
|---|---|---|---|
| Sin batería | 3 de 3 | 139-150 mV | FALLA |
| Con batería | 11 de 11 | 2-4 mV | OK |

La página muestra la variación (p. ej. `4.09 V · 92.9 % · varía 4 mV`). Los límites están en `board_nrf52.h`.

### POWER: CTRL_GPS pegado a VDD

**Qué se veía.** En la tarjeta RF, R154 (0 Ω de VDD a CTRL_GPS, la entrada ON/OFF de U59) dejaba el GPS siempre encendido. Algo más también podría unirlo a VDD: R167 en la placa principal, o un puente de soldadura. El POST del arranque no lo veía, porque el GPS arranca encendido. Después de la prueba del GPS el firmware lo apagaba y salía FALLA, que desaparecía al reiniciar.

**Qué hace ahora.**
- **Lo detecta desde el primer POST.** Baja CTRL_GPS 0.5 ms y lee el pin.
- **Suelta el pin si no baja.** Lo deja como entrada para no forzar la salida del TCA6408A contra VDD. Antes quedaba así hasta el siguiente reinicio.
- **Lo tolera.** Con `RAILS_GPS_STUCK_TOLERATED 1` (por defecto) avisa en el detalle (`SIP32431 GPS ON (AVISO, tolerado ...)`) y la prueba sigue en OK; con `0` es FALLA.
- **Los demás rieles fallan como siempre.**

En la placa probada, R154 no está montada ni en el BOM, y aun así CTRL_GPS no baja: falta encontrar qué lo une a VDD. Para localizarlo, desconectar la tarjeta RF y correr `POST:POWER`. Si deja de fallar, está en la tarjeta RF; si sigue, en la placa principal.

Si una tarjeta lleva R154 a propósito, se compila con `WM_RF_R154_MOUNTED 1` (`board_pins.h`): el pin no se maneja nunca y la prueba dice "fijo a VDD (R154) OK".

### WIND: crítica y por lado

Con los FS3000 instalados, su falla cuenta (`POLICY:WIND=TOL` la tolera en una placa sin ellos). El detalle dice qué lado falla: derecho (canal 0) o izquierdo (canal 1). También dice la causa: no responde, dato corrupto (checksum) o solo ceros.

Los sensores se leen 1 s después de encender POWER_VEL (`WIND_POWERUP_MS`), con reintentos durante 500 ms más. Medido: a los 100 ms responden solo ceros, y al segundo dan ~416 cuentas sin flujo.

### SD

`microSD no responde (error 0x01)` significa que la tarjeta no contesta ni el primer comando (CMD0, a 400 kHz). Es eléctrico: alimentación del zócalo, CS P0.26, SCK/MOSI/MISO o soldaduras. No es el formato. `POLICY:SD=TOL` la tolera hasta reiniciar.

## Qué cambia respecto al firmware principal

Todo está bajo `BOARD_POST_ONLY 1` en `lib/board_nrf52/board_nrf52.h`:

- **LoRa:** el LR1110 se enciende para que el POST lea su versión por SPI. No hay Join ni transmisiones, porque nunca se llama a `LoraNode::process()`, ni en el loop ni en las esperas del POST (`board_wait_ms`). El enlazador ni siquiera incluye el código del Join en el binario.
- **Flash interna:** no se escribe, salvo con los comandos de identidad (`NODE:SET`, `NODE:ID`, `LORA:CRED:SET`, `LORA:CRED:RESET`). Los envía la estación, antes de cargar el principal, o el panel de identidad. Las credenciales LoRaWAN y el número de nodo se leen y se muestran si ya están guardados. Si no están, no se graban. Así una tarjeta nueva no queda marcada como un nodo al pasar por este firmware: su identidad la graba el firmware de producción en su primer arranque.
- **Bluetooth:** el SoftDevice arranca y anuncia `WearMe_12_Nxxx`, que es lo que prueba el POST. No atiende comandos de la app.
- **Anillo:** apagado (`BOARD_ENABLE_MESH 0`). No incluye el aumento de potencia del anillo (`mesh_tx_power.cpp`).
- **Loop:** en lugar de `board_process()` se llama a `board_alive_process()`, que solo atiende LEDs, watchdog, pila y la verificación del avance del RTC.
- **Tamaño:** 264 124 B (`POST-TEST 1.4`), lejos del límite de 393 216 B. `tools/pio_ota.py` lo verifica en cada build, para no borrar el contexto de LoRaWAN de la tarjeta.

Después de la prueba se carga el firmware principal (`Wearme_Firmware_V3.1`) sin pasos extra: este firmware no tocó la flash interna ni el contexto de LoRaWAN.

## Archivos propios del proyecto

- `src/main.cpp`: arranque, señal de vida (`[VIVO]`, LED) y loop.
- `lib/post_lib/post_lib.cpp`: la tabla de pruebas, igual a la original, y solo los comandos de prueba.
- `lib/board_nrf52/board_nrf52.{h,cpp}`: los cambios de `BOARD_POST_ONLY` descritos arriba.
- `include/wearme_credentials.h` se copió para poder compilar. Tiene claves y está en `.gitignore`.
