"""Pasa a la base de la estación web (Cloudflare D1) las credenciales que existen fuera de ella:

  * include/wearme_credentials.h                              (tabla del firmware)       -> origen 'tabla'
  * tools/DEVEUI_APPKEY_generator/credenciales_lorawan.db     (generador de escritorio)  -> origen 'generador'
    (solo las que tienen nombre NODO-<número>)

    python tools/importar_credenciales.py

Genera un archivo SQL en la carpeta temporal (fuera de OneDrive) y muestra el comando para cargarlo:

    cd estacion_web
    npx wrangler@4.156.0 d1 execute wearme-estacion --remote --file=<ese archivo>

Después de cargarlo, BORRAR el archivo SQL (trae las AppKey). Es idempotente. La base manda: un nodo que ya está en
ella con OTRO DevEUI no se toca (por ejemplo, uno que la estación creó para una tarjeta). No imprime ninguna clave.
"""

import datetime
import os
import re
import sqlite3
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(HERE), "include", "wearme_credentials.h")
GEN = os.path.join(HERE, "DEVEUI_APPKEY_generator", "credenciales_lorawan.db")


def hexbytes(block):
    vals = re.findall(r"0x([0-9A-Fa-f]{2})", block)
    return "".join(v.upper() for v in vals)


def from_table():
    text = open(SRC, encoding="utf-8").read()
    m = re.search(r"WEARME_CREDENTIALS\s*\[\s*\]\s*=\s*\{(.*)\};", text, re.S)
    if not m:
        sys.exit("No se encontró la tabla WEARME_CREDENTIALS en " + SRC)
    # Cada fila: {nodo, {8 bytes}, {8 bytes}, {16 bytes}}
    rows = re.findall(r"\{\s*(\d+)\s*,\s*\{([^}]*)\}\s*,\s*\{([^}]*)\}\s*,\s*\{([^}]*)\}\s*\}", m.group(1))
    if not rows:
        sys.exit("La tabla no tiene filas con el formato esperado")
    return [(int(n), hexbytes(d), hexbytes(j), hexbytes(k), None) for n, d, j, k in rows]


def from_generator():
    if not os.path.exists(GEN):
        return [], 0
    con = sqlite3.connect("file:" + GEN + "?mode=ro", uri=True)
    out, skipped = [], 0
    for name, dev, join, key, created in con.execute(
            "SELECT name, dev_eui, join_eui, app_key, created_at FROM credentials ORDER BY id"):
        m = re.fullmatch(r"NODO-0*(\d+)", (name or "").strip(), re.I)
        if not m or not join:
            skipped += 1
            continue
        try:  # el generador guarda la hora local; la base, UTC
            created = datetime.datetime.strptime(created, "%Y-%m-%d %H:%M:%S").astimezone(
                datetime.timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
        except (TypeError, ValueError):
            created = None
        out.append((int(m.group(1)), dev.upper(), join.upper(), key.upper(), created))
    con.close()
    return out, skipped


def q(s):
    return "NULL" if s is None else "'" + str(s).replace("'", "''") + "'"


def main():
    nodes = {}  # nodo -> (dev, join, key, origen, creado)
    for n, dev, join, key, created in from_table():
        nodes[n] = (dev, join, key, "tabla", created)
    gen, skipped = from_generator()
    for n, dev, join, key, created in gen:
        if n in nodes:
            if nodes[n][0] != dev:
                sys.exit("NODO-%03d tiene un DevEUI en wearme_credentials.h y OTRO en el generador: revisar cuál es el "
                         "real antes de importar" % n)
            continue  # el mismo en los dos: queda como 'tabla'
        nodes[n] = (dev, join, key, "generador", created)
    devs = [v[0] for v in nodes.values()]
    if len(set(devs)) != len(devs):
        sys.exit("Hay un DevEUI repetido en dos nodos distintos: revisar las fuentes")
    for n, (dev, join, key, _, _) in nodes.items():
        if not (1 <= n <= 999) or len(dev) != 16 or len(join) != 16 or len(key) != 32 or \
                not re.fullmatch(r"[0-9A-F]+", dev + join + key):
            sys.exit("NODO-%03d: número fuera de 1-999 o DevEUI/JoinEUI/AppKey inválido" % n)

    sql = ["-- Credenciales para la estación web. BORRAR este archivo después de cargarlo."]
    for n in sorted(nodes):
        dev, join, key, source, created = nodes[n]
        note = "importado de wearme_credentials.h" if source == "tabla" else "importado del generador de escritorio"
        sql.append(
            "INSERT INTO nodes (node, dev_eui, join_eui, app_key, notes, created_by, source, created_at) VALUES "
            "(%d, %s, %s, %s, %s, 'importar_credenciales.py', %s, COALESCE(%s, datetime('now'))) "
            "ON CONFLICT(node) DO UPDATE SET join_eui = excluded.join_eui, app_key = excluded.app_key, "
            "source = COALESCE(nodes.source, excluded.source) WHERE nodes.dev_eui = excluded.dev_eui;"
            % (n, q(dev), q(join), q(key), q(note), q(source), q(created)))
    out = os.path.join(tempfile.gettempdir(), "wearme_credenciales.sql")
    with open(out, "w", encoding="utf-8") as f:
        f.write("\n".join(sql) + "\n")
    print("Nodos: %s" % ", ".join("%d (%s)" % (n, nodes[n][3]) for n in sorted(nodes)))
    if skipped:
        print("Del generador se omitieron %d sin nombre NODO-<número> o sin JoinEUI" % skipped)
    print("Archivo: %s" % out)
    print("\nCargarlo en la base:")
    print("  cd estacion_web")
    print('  npx wrangler@4.156.0 d1 execute wearme-estacion --remote --file="%s"' % out)
    print("\nDespués BORRAR el archivo (trae las AppKey).")


if __name__ == "__main__":
    main()
