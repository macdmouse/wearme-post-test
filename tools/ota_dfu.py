#!/usr/bin/env python3
"""Actualiza el firmware de un collar Wearme por Bluetooth (OTA DFU legacy del bootloader de Adafruit).

Uso normal (desde VS Code / PlatformIO):
  1. Habilitar el modo de actualización en el collar (queda en bajo consumo, se pide por LoRa):
       TTN -> downlink {"cmd":"dfu"} (FPort 1, bytes 05 44 46 55), por el anillo (FPort 21) o por serie DFU:OTA.
     El collar reinicia en el bootloader y anuncia "AdaDFU".
  2. pio run -t ota          (compila, arma firmware.zip y lo envía con este script)

Directo:
  python tools/ota_dfu.py <firmware.zip> [--address CF:46:AF:60:35:5F] [--serial COM62]

--address  dirección Bluetooth del collar (la del firmware, WearMe_12_N00X): con varios "AdaDFU" en alcance elige
           ese (el bootloader puede anunciar la misma dirección o la siguiente). Sin ella, debe haber uno solo.
--serial   manda antes DFU:OTA por ese puerto (banco de pruebas con USB): no hace falta el downlink.

Protocolo: Nordic Legacy DFU (SDK 11, dfu_version 0.5) = el que genera `adafruit-nrfutil dfu genpkg`:
servicio 00001530-1212-efde-1523-785feabcd123, punto de control 1531 (escritura + notificación), paquete 1532.
Requiere bleak (pip install bleak) en el Python que lo ejecute (PlatformIO: ~/.platformio/penv).
"""
import argparse
import asyncio
import json
import struct
import sys
import time
import zipfile

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    sys.exit("Falta bleak: instalarlo en el Python de PlatformIO con\n"
             "  ~/.platformio/penv/Scripts/python -m pip install bleak   (Windows)\n"
             "  ~/.platformio/penv/bin/python -m pip install bleak       (Linux/macOS)")

DFU_SERVICE = "00001530-1212-efde-1523-785feabcd123"
DFU_CTRL = "00001531-1212-efde-1523-785feabcd123"
DFU_PACKET = "00001532-1212-efde-1523-785feabcd123"

OP_START, OP_INIT, OP_RECEIVE, OP_VALIDATE, OP_ACTIVATE, OP_RESET, OP_PRN = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x08
OP_RESPONSE, OP_PRN_NOTIF = 0x10, 0x11
IMAGE_APPLICATION = 0x04
RESULTS = {1: "OK", 2: "estado invalido", 3: "no soportado", 4: "tamano excede el limite", 5: "error de CRC",
           6: "operacion fallida"}


class StaleSession(Exception):
    """El bootloader tenía una carga anterior a medias: se reinició y hay que pedir el modo DFU otra vez."""


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


def load_package(path):
    with zipfile.ZipFile(path) as z:
        app = json.loads(z.read("manifest.json"))["manifest"]["application"]
        return z.read(app["bin_file"]), z.read(app["dat_file"])


def same_device(a, b):
    """El bootloader anuncia la dirección del firmware o la siguiente (último byte + 1)."""
    if not a or not b:
        return False
    x, y = int(a.replace(":", ""), 16), int(b.replace(":", ""), 16)
    return x == y or y == x + 1


async def find_dfu(address, timeout):
    log("Buscando un collar en modo de actualizacion (AdaDFU)...")
    found = await BleakScanner.discover(timeout=timeout, return_adv=True)
    cands = []
    for addr, (dev, adv) in found.items():
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        name = adv.local_name or dev.name or ""
        if DFU_SERVICE in uuids or "dfu" in name.lower():
            cands.append((dev, name, adv.rssi))
    for dev, name, rssi in cands:
        log(f"  {dev.address}  '{name}'  RSSI {rssi}")
    if address:
        cands = [c for c in cands if same_device(address, c[0].address)]
    if len(cands) == 1:
        return cands[0][0]
    if not cands:
        sys.exit("Ningun collar en modo AdaDFU: mandar antes el downlink {\"cmd\":\"dfu\"} (o DFU:OTA por serie) y "
                 "esperar unos segundos")
    sys.exit("Hay varios collares en modo AdaDFU: indicar cual con --address (o custom_ota_address en platformio.ini)")


async def update(dev, firmware, init, prn, chunk_max):
    notes = asyncio.Queue()
    async with BleakClient(dev, timeout=20) as c:
        chunk = max(20, min(chunk_max, (c.mtu_size or 23) - 3))
        log(f"Conectado a {dev.address} (MTU {c.mtu_size}, paquetes de {chunk} B, aviso cada {prn})")
        await c.start_notify(DFU_CTRL, lambda _, d: notes.put_nowait(bytes(d)))

        async def expect(op, what, timeout=30):
            while True:
                d = await asyncio.wait_for(notes.get(), timeout)
                if d[0] == OP_RESPONSE and d[1] == op:
                    if d[2] != 1:
                        raise RuntimeError(f"{what}: {RESULTS.get(d[2], d[2])}")
                    return

        await c.write_gatt_char(DFU_CTRL, bytes([OP_START, IMAGE_APPLICATION]), response=True)
        await c.write_gatt_char(DFU_PACKET, struct.pack("<III", 0, 0, len(firmware)), response=False)
        try:
            await expect(OP_START, "inicio (START_DFU)")
        except RuntimeError as e:
            if "estado invalido" not in str(e):
                raise
            # Sesión anterior a medias (p. ej. una carga cortada): reinicio del bootloader -> vuelve al firmware anterior
            log("El bootloader quedo a medias de una carga anterior: se reinicia (vuelve al firmware anterior)")
            try:
                await c.write_gatt_char(DFU_CTRL, bytes([OP_RESET]), response=True)
            except Exception:
                pass
            raise StaleSession() from e
        await c.write_gatt_char(DFU_CTRL, bytes([OP_INIT, 0x00]), response=True)
        await c.write_gatt_char(DFU_PACKET, init, response=False)
        await c.write_gatt_char(DFU_CTRL, bytes([OP_INIT, 0x01]), response=True)
        await expect(OP_INIT, "paquete de inicio (version del SoftDevice, tipo de placa)")
        await c.write_gatt_char(DFU_CTRL, bytes([OP_PRN]) + struct.pack("<H", prn), response=True)
        await c.write_gatt_char(DFU_CTRL, bytes([OP_RECEIVE]), response=True)

        t0, sent, n, last_pct = time.time(), 0, 0, -1
        while sent < len(firmware):
            part = firmware[sent:sent + chunk]
            await c.write_gatt_char(DFU_PACKET, part, response=False)
            sent += len(part)
            n += 1
            if n % prn == 0 and sent < len(firmware):
                while True:  # control de flujo: el bootloader confirma cada `prn` paquetes
                    d = await asyncio.wait_for(notes.get(), 30)
                    if d[0] == OP_PRN_NOTIF:
                        break
                    if d[0] == OP_RESPONSE and d[2] != 1:
                        raise RuntimeError(f"transferencia: {RESULTS.get(d[2], d[2])}")
            pct = sent * 100 // len(firmware)
            if pct // 5 != last_pct // 5:
                last_pct = pct
                rate = sent / max(time.time() - t0, 0.001) / 1024
                log(f"  {pct:3d} %  {sent}/{len(firmware)} B  {rate:.1f} KB/s")
        await expect(OP_RECEIVE, "recepcion del firmware", 60)
        log(f"Firmware recibido en {time.time() - t0:.0f} s; validando...")
        await c.write_gatt_char(DFU_CTRL, bytes([OP_VALIDATE]), response=True)
        await expect(OP_VALIDATE, "validacion (CRC)")
        log("Validado: activando y reiniciando el collar")
        try:
            await c.write_gatt_char(DFU_CTRL, bytes([OP_ACTIVATE]), response=True)
        except Exception:
            pass  # el collar se desconecta al reiniciar


def serial_trigger(port):
    """Manda DFU:OTA por el USB (banco). Sin el puerto, el collar puede estar ya en AdaDFU (p. ej. tras un intento
    fallido): se sigue buscándolo por Bluetooth. El collar puede tardar unos segundos en atender (SURVIVE duerme)."""
    import serial
    log(f"Mandando DFU:OTA por {port}")
    try:
        with serial.Serial(port, 115200, timeout=1) as s:
            s.write(b"DFU:OTA\n")
            end = time.time() + 15
            while time.time() < end:
                line = s.readline().decode("utf-8", "replace").strip()
                if "[DFU]" in line:
                    log("  " + line)
    except Exception:
        log(f"  {port} se cerro o no esta: el collar reinicio en el bootloader (o ya estaba en AdaDFU)")


async def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("package", help="firmware.zip de la compilacion (adafruit-nrfutil)")
    ap.add_argument("--address", default="", help="direccion Bluetooth del collar (WearMe_12_N00X)")
    ap.add_argument("--serial", default="", help="puerto serie para mandar DFU:OTA antes (banco de pruebas)")
    # 8: probado (392 KB en ~6 min). Con 24 el bootloader falla a mitad ("operacion fallida": no alcanza a escribir la flash)
    ap.add_argument("--prn", type=int, default=8, help="paquetes entre confirmaciones del bootloader")
    ap.add_argument("--chunk", type=int, default=244, help="bytes maximos por paquete (limitado por el MTU)")
    ap.add_argument("--scan", type=float, default=10, help="segundos de busqueda")
    a = ap.parse_args()

    firmware, init = load_package(a.package)
    log(f"Paquete {a.package}: aplicacion {len(firmware)} B, init {len(init)} B")
    t0 = time.time()
    for attempt in (1, 2):
        if a.serial:
            serial_trigger(a.serial)
            await asyncio.sleep(4)  # reinicio en el bootloader
        dev = await find_dfu(a.address, a.scan)
        try:
            await update(dev, firmware, init, a.prn, a.chunk)
            break
        except StaleSession:
            if attempt == 2 or not a.serial:
                sys.exit("El bootloader tenia una carga a medias y se reinicio: el collar volvio al firmware anterior. "
                         "Pedir el modo DFU otra vez (downlink {\"cmd\":\"dfu\"} o DFU:OTA) y repetir.")
            log("Esperando que el collar arranque para pedir el modo DFU otra vez...")
            await asyncio.sleep(25)
        except (RuntimeError, asyncio.TimeoutError) as e:
            sys.exit(f"Fallo la actualizacion: {e or 'sin respuesta del bootloader'}. El collar conserva el firmware "
                     "anterior (o queda en AdaDFU): repetir.")
    log(f"Listo en {time.time() - t0:.0f} s: el collar arranca con el firmware nuevo (conserva nodo y credenciales)")


if __name__ == "__main__":
    asyncio.run(main())
