"""Genera la página de la estación de montaje con los dos firmwares adentro.

    python tools/build_estacion.py               compila los dos proyectos y genera la página
    python tools/build_estacion.py --no-build    usa lo último compilado
    python tools/build_estacion.py --web         además prepara estacion_web/public/ (versión Cloudflare)

Resultado: tools/estacion/Estacion_Wearme.html, un solo archivo. El operador lo abre con doble clic (Edge o Chrome),
conecta la tarjeta, elige el nodo y pulsa "Iniciar prueba y carga". La página carga el firmware de prueba, revisa el
montaje, graba el nodo y carga el firmware principal, sin PlatformIO ni código.

OJO: el firmware principal trae las claves LoRaWAN de todos los nodos (include/wearme_credentials.h), así que la
página también. Tratarla como confidencial: no publicarla ni mandarla por canales abiertos.
"""

import argparse
import base64
import datetime
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import zipfile

HERE = os.path.dirname(os.path.abspath(__file__))
POST_DIR = os.path.dirname(HERE)
MAIN_DIR = os.path.join(os.path.dirname(POST_DIR), "Wearme_Firmware_V3.1")
TEMPLATE = os.path.join(HERE, "diagnostico.html")
OUT = os.path.join(HERE, "estacion", "Estacion_Wearme.html")
WEB_PUBLIC = os.path.join(POST_DIR, "estacion_web", "public")
ENV = "seeed_wio_tracker_1110"
FIRMWARE_MAX_BYTES = 0xE9000 - 0x89000  # tools/pio_ota.py: más grande borraría el contexto de LoRaWAN


def die(msg):
    print("\n*** ERROR: " + msg)
    sys.exit(1)


def pio_exe():
    exe = shutil.which("pio") or shutil.which("platformio")
    if exe:
        return exe
    for name in ("pio.exe", "pio"):
        p = os.path.join(os.path.expanduser("~"), ".platformio", "penv", "Scripts" if os.name == "nt" else "bin", name)
        if os.path.exists(p):
            return p
    die("no se encontró PlatformIO (pio)")


def build_dir(project):
    """build_dir de platformio.ini (con ${sysenv.X}); por defecto .pio/build"""
    ini = open(os.path.join(project, "platformio.ini"), encoding="utf-8").read()
    m = re.search(r"^\s*build_dir\s*=\s*(.+?)\s*$", ini, re.M)
    if not m:
        return os.path.join(project, ".pio", "build")
    path = re.sub(r"\$\{sysenv\.(\w+)\}", lambda g: os.environ.get(g.group(1), ""), m.group(1))
    return path if os.path.isabs(path) else os.path.join(project, path)


def fw_version(project):
    h = open(os.path.join(project, "lib", "board_nrf52", "board_nrf52.h"), encoding="utf-8").read()
    m = re.search(r'#define\s+WEARME_FW_VERSION\s+"([^"]+)"', h)
    if not m:
        die("no se encontró WEARME_FW_VERSION en " + project)
    return m.group(1)


def crc16(data, crc=0xFFFF):  # igual que nordicsemi/dfu/crc16.py
    for b in data:
        crc = (crc >> 8 & 0x00FF) | (crc << 8 & 0xFF00)
        crc ^= b
        crc ^= (crc & 0x00FF) >> 4
        crc ^= (crc << 8) << 4
        crc ^= ((crc & 0x00FF) << 4) << 1
        crc &= 0xFFFF
    return crc


def firmware(project, label):
    zpath = os.path.join(build_dir(project), ENV, "firmware.zip")
    if not os.path.exists(zpath):
        die("no está %s (compilar %s)" % (zpath, project))
    with zipfile.ZipFile(zpath) as z:
        app = json.loads(z.read("manifest.json"))["manifest"]["application"]
        bin_ = z.read(app["bin_file"])
        dat = z.read(app["dat_file"])
    if len(bin_) > FIRMWARE_MAX_BYTES:
        die("%s: %d B pasa el límite de %d B (borraría el contexto de LoRaWAN)" % (label, len(bin_), FIRMWARE_MAX_BYTES))
    crc = crc16(bin_)
    if crc != app["init_packet_data"]["firmware_crc16"]:
        die("%s: el CRC del .bin no coincide con el manifest" % label)
    built = datetime.datetime.fromtimestamp(os.path.getmtime(zpath)).strftime("%Y-%m-%d %H:%M")
    info = {
        "version": fw_version(project),
        "size": len(bin_),
        "crc16": crc,
        "sha256": hashlib.sha256(bin_).hexdigest(),
        "built": built,
        "bin": base64.b64encode(bin_).decode("ascii"),
        "dat": base64.b64encode(dat).decode("ascii"),
    }
    print("  %-10s %-14s %7d B  compilado %s  (%s)" % (label, info["version"], len(bin_), built, zpath))
    return info


def nodes():
    """Números de nodo de la tabla de credenciales. El firmware de prueba graba las credenciales (NODE:SET) con SU
    copia de la tabla y el principal las usa: las dos copias tienen que ser idénticas."""
    a = os.path.join(POST_DIR, "include", "wearme_credentials.h")
    b = os.path.join(MAIN_DIR, "include", "wearme_credentials.h")
    if open(a, "rb").read() != open(b, "rb").read():
        die("include/wearme_credentials.h es distinto en los dos proyectos: copiar el del firmware principal al de "
            "prueba (el de prueba es el que graba las credenciales)")
    text = open(b, encoding="utf-8").read()
    m = re.search(r"WEARME_CREDENTIALS\s*\[\s*\]\s*=\s*\{(.*)\};", text, re.S)
    found = [int(n) for n in re.findall(r"\{\s*(\d+)\s*,", m.group(1) if m else text)]
    if not found:
        die("no se encontraron nodos en " + b)
    return sorted(set(found))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-build", action="store_true", help="no compilar: usar el último firmware.zip de cada proyecto")
    ap.add_argument("--web", action="store_true", help="también estacion_web/public/ para publicar en Cloudflare")
    args = ap.parse_args()
    if not os.path.isdir(MAIN_DIR):
        die("no está el proyecto del firmware principal en " + MAIN_DIR)

    if not args.no_build:
        for project in (POST_DIR, MAIN_DIR):
            print("Compilando " + project)
            if subprocess.call([pio_exe(), "run", "-d", project]) != 0:
                die("falló la compilación de " + project)

    print("\nFirmwares:")
    data = {
        "generated": datetime.datetime.now().strftime("%Y-%m-%d %H:%M"),
        "nodes": nodes(),
        "post": firmware(POST_DIR, "prueba"),
        "main": firmware(MAIN_DIR, "principal"),
    }
    if not data["post"]["version"].startswith("POST-TEST"):
        die("el firmware de prueba no dice POST-TEST en WEARME_FW_VERSION")

    html = open(TEMPLATE, encoding="utf-8").read()
    marker = re.compile(r"/\*FIRMWARES-BEGIN\*/.*?/\*FIRMWARES-END\*/", re.S)
    if len(marker.findall(html)) != 1:
        die("no se encontró el marcador FIRMWARES en " + TEMPLATE)
    payload = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    html = marker.sub(lambda _: "/*FIRMWARES-BEGIN*/" + payload + "/*FIRMWARES-END*/", html)
    html = html.replace("<title>Diagnóstico Wearme</title>", "<title>Estación Wearme</title>", 1)
    html = html.replace("<h1>Diagnóstico de la placa Wearme</h1>", "<h1>Estación de montaje Wearme</h1>", 1)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8", newline="\n") as f:
        f.write(html)
    print("\nNodos: %s" % ", ".join(str(n) for n in data["nodes"]))
    print("Página: %s (%d KB)" % (OUT, os.path.getsize(OUT) // 1024))
    print("OJO: trae el firmware principal con las claves LoRaWAN de todos los nodos. No publicarla.")

    if args.web:
        # Versión web: la página sin firmwares adentro (los descarga de firmware/firmwares.json, detrás de Cloudflare
        # Access) y los nodos salen de la base, no de la tabla
        web = open(TEMPLATE, encoding="utf-8").read()
        web = web.replace("<title>Diagnóstico Wearme</title>", "<title>Estación Wearme</title>", 1)
        web = web.replace("<h1>Diagnóstico de la placa Wearme</h1>", "<h1>Estación de montaje Wearme</h1>", 1)
        os.makedirs(os.path.join(WEB_PUBLIC, "firmware"), exist_ok=True)
        with open(os.path.join(WEB_PUBLIC, "index.html"), "w", encoding="utf-8", newline="\n") as f:
            f.write(web)
        fw = {k: v for k, v in data.items() if k != "nodes"}
        with open(os.path.join(WEB_PUBLIC, "firmware", "firmwares.json"), "w", encoding="utf-8", newline="\n") as f:
            json.dump(fw, f, separators=(",", ":"))
        # Registro de nodos (DevEUI, tarjetas, siguiente número, plataforma)
        shutil.copyfile(os.path.join(POST_DIR, "estacion_web", "paginas", "nodos.html"),
                        os.path.join(WEB_PUBLIC, "nodos.html"))
        print("\nVersión web: %s (publicar con: cd estacion_web && npx wrangler@4.156.0 deploy)" % WEB_PUBLIC)


if __name__ == "__main__":
    main()
