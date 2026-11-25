#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generador de credenciales LoRaWAN — DevEUI, JoinEUI y AppKey
============================================================

Interfaz gráfica (Tkinter) para generar, registrar y exportar credenciales
LoRaWAN (activación OTAA).

  * DevEUI  : 8 bytes  (16 caracteres hexadecimales), único en el registro.
  * JoinEUI : 8 bytes  — aleatorio por dispositivo o un valor fijo para todos.
  * AppKey  : 16 bytes (32 caracteres hexadecimales, clave AES-128).
  * Aleatoriedad criptográfica (módulo `secrets`).
  * Registro persistente en SQLite: credenciales_lorawan.db (junto al script).
  * Casilla ☐/☑ para marcar cada credencial como usada o disponible.
  * Pestaña dedicada a las credenciales usadas.
  * Panel con el código C listo para pegar en el firmware (orden MSB).
  * Exportación a CSV (todo, lo filtrado o lo seleccionado).

Requisitos: Python 3.8+ (Tkinter y SQLite vienen incluidos).
En Linux, si falta Tkinter:  sudo apt install python3-tk

Uso:  python generador_lorawan.py      (en Windows también:  py generador_lorawan.py)
"""

import csv
import os
import re
import secrets
import sqlite3
import sys
from datetime import datetime
import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog

APP_TITLE = "Generador de credenciales LoRaWAN"
DB_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "credenciales_lorawan.db")
CHECK_ON, CHECK_OFF = "☑", "☐"
MAX_BATCH = 10000
PREVIEW_LIMIT = 30  # credenciales mostradas en el panel de código (al copiar se copian todas)

CSV_HEADER = ["nombre", "dev_eui", "join_eui", "app_key", "usada",
              "fecha_creacion", "fecha_uso", "notas"]

SCHEMA = """
CREATE TABLE IF NOT EXISTS credentials (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    name        TEXT    NOT NULL DEFAULT '',
    dev_eui     TEXT    NOT NULL UNIQUE,
    join_eui    TEXT    NOT NULL DEFAULT '',
    app_key     TEXT    NOT NULL,
    used        INTEGER NOT NULL DEFAULT 0,
    created_at  TEXT    NOT NULL,
    used_at     TEXT,
    notes       TEXT    NOT NULL DEFAULT ''
);
CREATE TABLE IF NOT EXISTS settings (
    key   TEXT PRIMARY KEY,
    value TEXT NOT NULL
);
"""


def now():
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def c_array(label, hexstr, per_line=8):
    """Declaración C alineada, p. ej.:
    static const uint8_t APP_KEY[16] = { 0x3D, ..., 0x89,
                                         0xFB, ..., 0x89 };
    """
    head = f"static const uint8_t {label.ljust(11)} = {{ "
    octets = [f"0x{hexstr[i:i + 2]}" for i in range(0, len(hexstr), 2)]
    chunks = [", ".join(octets[i:i + per_line]) for i in range(0, len(octets), per_line)]
    return head + (",\n" + " " * len(head)).join(chunks) + " };"


def c_snippet(row):
    """Bloque C con DEV_EUI, JOIN_EUI y APP_KEY de una credencial (orden MSB)."""
    lines = [f"// {row['name'] or 'Credencial #' + str(row['id'])}",
             c_array("DEV_EUI[8]", row["dev_eui"])]
    if row["join_eui"]:
        lines.append(c_array("JOIN_EUI[8]", row["join_eui"]))
    else:
        lines.append("// JOIN_EUI no registrado para esta credencial (creada en una versión anterior)")
    lines.append(c_array("APP_KEY[16]", row["app_key"]))
    return "\n".join(lines)


# --------------------------------------------------------------------------- #
#  Capa de datos
# --------------------------------------------------------------------------- #
class CredentialStore:
    EDITABLE = {"name", "notes"}

    def __init__(self, path):
        self.path = path
        self.conn = sqlite3.connect(path)
        self.conn.row_factory = sqlite3.Row
        with self.conn:
            self.conn.executescript(SCHEMA)
            # Migración: bases creadas por la versión anterior no tenían join_eui
            cols = {r["name"] for r in self.conn.execute("PRAGMA table_info(credentials)")}
            if "join_eui" not in cols:
                self.conn.execute("ALTER TABLE credentials ADD COLUMN join_eui TEXT NOT NULL DEFAULT ''")

    # ---- ajustes persistentes -------------------------------------------- #
    def get_setting(self, key, default=""):
        row = self.conn.execute("SELECT value FROM settings WHERE key = ?", (key,)).fetchone()
        return row["value"] if row else default

    def set_setting(self, key, value):
        with self.conn:
            self.conn.execute("INSERT OR REPLACE INTO settings (key, value) VALUES (?, ?)",
                              (key, str(value)))

    # ---- validación ------------------------------------------------------ #
    @staticmethod
    def _clean_hex(text, what):
        h = re.sub(r"[\s:\-]|0x|0X|,", "", text or "").upper()
        if h and not re.fullmatch(r"[0-9A-F]+", h):
            raise ValueError(f"El {what} solo puede contener caracteres hexadecimales (0-9, A-F).")
        return h

    @classmethod
    def normalize_hex_prefix(cls, text):
        p = cls._clean_hex(text, "prefijo DevEUI")
        if len(p) % 2:
            raise ValueError("El prefijo DevEUI debe tener un número par de caracteres (bytes completos).")
        if len(p) > 12:
            raise ValueError("El prefijo DevEUI admite como máximo 6 bytes (12 caracteres) "
                             "para dejar al menos 2 bytes aleatorios.")
        return p

    @classmethod
    def normalize_eui(cls, text):
        e = cls._clean_hex(text, "JoinEUI fijo")
        if len(e) != 16:
            raise ValueError("El JoinEUI fijo debe tener exactamente 8 bytes (16 caracteres hex).\n"
                             "Ejemplos válidos: 70B3D57ED0079094 · 70:B3:D5:7E:D0:07:90:94")
        return e

    # ---- generación ------------------------------------------------------ #
    @staticmethod
    def random_eui(prefix="", local_admin=True):
        pfx = bytes.fromhex(prefix)
        eui = bytearray(pfx + secrets.token_bytes(8 - len(pfx)))
        if not pfx:
            eui[0] &= 0xFE          # bit I/G = 0 -> dirección individual (unicast)
            if local_admin:
                eui[0] |= 0x02      # bit U/L = 1 -> administrada localmente
        return eui.hex().upper()

    @staticmethod
    def random_app_key():
        return secrets.token_hex(16).upper()

    def _next_number(self, name_prefix):
        cur = self.conn.execute(
            "SELECT name FROM credentials WHERE substr(name, 1, ?) = ?",
            (len(name_prefix), name_prefix))
        highest = 0
        for (name,) in cur:
            m = re.fullmatch(r"\d+", name[len(name_prefix):])
            if m:
                highest = max(highest, int(m.group(0)))
        return highest + 1

    def generate(self, count, name_prefix="", eui_prefix="", local_admin=True,
                 join_mode="random", join_eui_fixed=""):
        prefix = self.normalize_hex_prefix(eui_prefix)
        fixed_join = self.normalize_eui(join_eui_fixed) if join_mode == "fixed" else None
        first = self._next_number(name_prefix) if name_prefix else 0
        ids = []
        with self.conn:  # todo el lote en una transacción
            for i in range(count):
                name = f"{name_prefix}{first + i:04d}" if name_prefix else ""
                join_eui = fixed_join or self.random_eui("", local_admin)
                for _ in range(200):
                    try:
                        cur = self.conn.execute(
                            "INSERT INTO credentials (name, dev_eui, join_eui, app_key, created_at) "
                            "VALUES (?, ?, ?, ?, ?)",
                            (name, self.random_eui(prefix, local_admin), join_eui,
                             self.random_app_key(), now()))
                        ids.append(cur.lastrowid)
                        break
                    except sqlite3.IntegrityError:  # DevEUI repetido: reintentar
                        continue
                else:
                    raise RuntimeError("No fue posible generar un DevEUI único con ese prefijo: "
                                       "el espacio disponible está casi agotado.")
        return ids

    # ---- consultas ------------------------------------------------------- #
    def fetch(self, status="all", search="", order="id ASC"):
        sql, cond, params = "SELECT * FROM credentials", [], []
        if status == "used":
            cond.append("used = 1")
        elif status == "free":
            cond.append("used = 0")
        if search:
            cond.append("(name LIKE ? OR dev_eui LIKE ? OR join_eui LIKE ? "
                        "OR app_key LIKE ? OR notes LIKE ?)")
            params += [f"%{search}%"] * 5
        if cond:
            sql += " WHERE " + " AND ".join(cond)
        sql += f" ORDER BY {order}"
        return [dict(r) for r in self.conn.execute(sql, params)]

    def fetch_ids(self, ids):
        if not ids:
            return []
        marks = ",".join("?" * len(ids))
        return [dict(r) for r in self.conn.execute(
            f"SELECT * FROM credentials WHERE id IN ({marks}) ORDER BY id", list(ids))]

    def stats(self):
        total, used = self.conn.execute(
            "SELECT COUNT(*), COALESCE(SUM(used), 0) FROM credentials").fetchone()
        return total, used, total - used

    # ---- modificaciones -------------------------------------------------- #
    def set_used(self, ids, used):
        if not ids:
            return
        with self.conn:
            self.conn.executemany(
                "UPDATE credentials SET used = ?, used_at = ? WHERE id = ?",
                [(1 if used else 0, now() if used else None, i) for i in ids])

    def set_field(self, cred_id, field, value):
        if field not in self.EDITABLE:
            raise ValueError(f"Campo no editable: {field}")
        with self.conn:
            self.conn.execute(f"UPDATE credentials SET {field} = ? WHERE id = ?", (value, cred_id))

    def delete(self, ids):
        with self.conn:
            self.conn.executemany("DELETE FROM credentials WHERE id = ?", [(i,) for i in ids])


# --------------------------------------------------------------------------- #
#  Tabla reutilizable (registro, usadas y último lote) + panel de código C
# --------------------------------------------------------------------------- #
class CredentialTable(ttk.Frame):
    COLUMNS = {
        "used":       ("Usada",        60,  "center"),
        "id":         ("#",            50,  "center"),
        "name":       ("Nombre",       110, "w"),
        "dev_eui":    ("DevEUI",       150, "w"),
        "join_eui":   ("JoinEUI",      150, "w"),
        "app_key":    ("AppKey",       270, "w"),
        "created_at": ("Creada",       135, "center"),
        "used_at":    ("Fecha de uso", 135, "center"),
        "notes":      ("Notas",        150, "w"),
    }

    def __init__(self, master, app, mode="all"):
        """mode: 'all' (registro con filtros), 'used' (solo usadas) o 'batch' (último lote)."""
        super().__init__(master, padding=6)
        self.app = app
        self.store = app.store
        self.mode = mode
        self.status_var = tk.StringVar(value="all")
        self.search_var = tk.StringVar()
        self.count_var = tk.StringVar()
        self.preview_title = tk.StringVar(value="Código C")
        self._rows = {}
        self._build()

    # ---- construcción ---------------------------------------------------- #
    def _build(self):
        top = ttk.Frame(self)
        top.pack(fill="x", pady=(0, 6))

        if self.mode == "all":
            ttk.Label(top, text="Mostrar:").pack(side="left")
            for text, val in (("Todas", "all"), ("Disponibles", "free"), ("Usadas", "used")):
                ttk.Radiobutton(top, text=text, value=val, variable=self.status_var,
                                command=self.refresh).pack(side="left", padx=3)
        if self.mode != "batch":
            ttk.Label(top, text="   Buscar:").pack(side="left")
            ttk.Entry(top, textvariable=self.search_var, width=24).pack(side="left", padx=3)
            self.search_var.trace_add("write", lambda *_: self.refresh())

        ttk.Button(top, text="Eliminar", command=self.delete_selected).pack(side="right", padx=2)
        ttk.Button(top, text="Marcar disponible",
                   command=lambda: self.mark_selected(False)).pack(side="right", padx=2)
        ttk.Button(top, text="Marcar usada",
                   command=lambda: self.mark_selected(True)).pack(side="right", padx=2)

        # Tabla (arriba) y panel de código C (abajo), separados por un divisor arrastrable
        panes = ttk.PanedWindow(self, orient="vertical")
        panes.pack(fill="both", expand=True)

        body = ttk.Frame(panes)
        self.tree = ttk.Treeview(body, columns=list(self.COLUMNS), show="headings",
                                 selectmode="extended", height=8)
        for col, (title, width, anchor) in self.COLUMNS.items():
            self.tree.heading(col, text=title)
            self.tree.column(col, width=width, minwidth=40, anchor=anchor,
                             stretch=col in ("app_key", "notes"))
        self.tree.tag_configure("used", foreground="#8a8a8a")
        vsb = ttk.Scrollbar(body, orient="vertical", command=self.tree.yview)
        hsb = ttk.Scrollbar(body, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        vsb.grid(row=0, column=1, sticky="ns")
        hsb.grid(row=1, column=0, sticky="ew")
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1)
        panes.add(body, weight=3)

        code_box = ttk.LabelFrame(panes, padding=4)
        code_box.configure(labelwidget=ttk.Label(code_box, textvariable=self.preview_title))
        code_bar = ttk.Frame(code_box)
        code_bar.pack(fill="x", pady=(0, 4))
        ttk.Label(code_bar, foreground="#666",
                  text="Selecciona una o varias credenciales para ver su código. "
                       "Orden MSB, listo para pegar en el firmware.").pack(side="left")
        ttk.Button(code_bar, text="Copiar código C",
                   command=self.copy_c).pack(side="right")
        text_frame = ttk.Frame(code_box)
        text_frame.pack(fill="both", expand=True)
        self.code = tk.Text(text_frame, height=6, wrap="none", font="TkFixedFont",
                            relief="flat", padx=8, pady=6, background="#f6f8fa",
                            foreground="#1f2328", state="disabled")
        csb_y = ttk.Scrollbar(text_frame, orient="vertical", command=self.code.yview)
        csb_x = ttk.Scrollbar(text_frame, orient="horizontal", command=self.code.xview)
        self.code.configure(yscrollcommand=csb_y.set, xscrollcommand=csb_x.set)
        self.code.grid(row=0, column=0, sticky="nsew")
        csb_y.grid(row=0, column=1, sticky="ns")
        csb_x.grid(row=1, column=0, sticky="ew")
        text_frame.rowconfigure(0, weight=1)
        text_frame.columnconfigure(0, weight=1)
        panes.add(code_box, weight=2)

        bottom = ttk.Frame(self)
        bottom.pack(fill="x", pady=(6, 0))
        ttk.Label(bottom, textvariable=self.count_var).pack(side="left")
        ttk.Button(bottom, text="Exportar CSV…", command=self.export).pack(side="right")
        ttk.Checkbutton(bottom, text="Marcar como usadas al exportar",
                        variable=self.app.mark_on_export).pack(side="right", padx=8)
        ttk.Combobox(bottom, textvariable=self.app.sep_var, state="readonly", width=20,
                     values=["; (Excel en español)", ", (estándar)"]).pack(side="right")
        ttk.Label(bottom, text="Separador CSV:").pack(side="right", padx=(0, 4))

        ttk.Label(self, foreground="#666",
                  text="Clic en ☐ para marcar/desmarcar · Espacio: alterna la selección · "
                       "Doble clic en Nombre o Notas para editar · Clic derecho para copiar"
                  ).pack(anchor="w", pady=(4, 0))

        self.menu = tk.Menu(self, tearoff=False)
        self.menu.add_command(label="Copiar código C (DEV_EUI, JOIN_EUI, APP_KEY)",
                              command=self.copy_c)
        self.menu.add_separator()
        self.menu.add_command(label="Copiar DevEUI", command=lambda: self.copy("dev_eui"))
        self.menu.add_command(label="Copiar JoinEUI", command=lambda: self.copy("join_eui"))
        self.menu.add_command(label="Copiar AppKey", command=lambda: self.copy("app_key"))
        self.menu.add_command(label="Copiar los tres (texto)", command=lambda: self.copy("all"))
        self.menu.add_separator()
        self.menu.add_command(label="Marcar usada", command=lambda: self.mark_selected(True))
        self.menu.add_command(label="Marcar disponible", command=lambda: self.mark_selected(False))

        self.tree.bind("<<TreeviewSelect>>", lambda e: self.update_preview())
        self.tree.bind("<Button-1>", self._on_click)
        self.tree.bind("<Double-1>", self._on_double)
        self.tree.bind("<space>", lambda e: self.toggle_selected())
        self.tree.bind("<Control-c>", lambda e: self.copy_c())
        self.tree.bind("<Control-a>", lambda e: self.select_all())
        self.tree.bind("<Button-3>", self._popup)
        if sys.platform == "darwin":
            self.tree.bind("<Button-2>", self._popup)

    # ---- datos ----------------------------------------------------------- #
    def _load(self):
        if self.mode == "batch":
            return self.store.fetch_ids(self.app.last_batch)
        search = self.search_var.get().strip()
        if self.mode == "used":
            return self.store.fetch("used", search, order="used_at DESC, id DESC")
        return self.store.fetch(self.status_var.get(), search)

    def refresh(self):
        selected = set(self.tree.selection())
        self.tree.delete(*self.tree.get_children())
        self._rows = {}
        rows = self._load()
        for r in rows:
            iid = str(r["id"])
            self._rows[iid] = r
            self.tree.insert("", "end", iid=iid, tags=("used",) if r["used"] else (), values=(
                CHECK_ON if r["used"] else CHECK_OFF, r["id"], r["name"], r["dev_eui"],
                r["join_eui"], r["app_key"], r["created_at"], r["used_at"] or "", r["notes"]))
        keep = [i for i in selected if i in self._rows]
        if keep:
            self.tree.selection_set(keep)
        used = sum(1 for r in rows if r["used"])
        self.count_var.set(f"{len(rows)} credenciales en esta vista · {used} usadas · "
                           f"{len(rows) - used} disponibles")
        self.update_preview()

    def select_all(self):
        self.tree.selection_set(self.tree.get_children())
        return "break"

    def _selected_rows(self):
        return [self._rows[i] for i in self.tree.selection() if i in self._rows]

    def update_preview(self):
        rows = self._selected_rows()
        if not rows:
            text = "// Selecciona una credencial en la tabla para ver aquí su código C."
            self.preview_title.set("Código C")
        else:
            text = "\n\n".join(c_snippet(r) for r in rows[:PREVIEW_LIMIT])
            if len(rows) > PREVIEW_LIMIT:
                text += (f"\n\n// … y {len(rows) - PREVIEW_LIMIT} credenciales más "
                         "(«Copiar código C» las incluye todas)")
            self.preview_title.set(f"Código C — {len(rows)} credencial(es) seleccionada(s)")
        self.code.configure(state="normal")
        self.code.delete("1.0", "end")
        self.code.insert("1.0", text)
        self.code.configure(state="disabled")

    # ---- eventos --------------------------------------------------------- #
    def _column_at(self, x):
        col = self.tree.identify_column(x)  # '#1', '#2', ...
        return list(self.COLUMNS)[int(col[1:]) - 1] if col else None

    def _on_click(self, event):
        if self.tree.identify_region(event.x, event.y) != "cell":
            return
        if self._column_at(event.x) == "used":
            iid = self.tree.identify_row(event.y)
            if iid in self._rows:
                self.store.set_used([int(iid)], not self._rows[iid]["used"])
                self.app.refresh_all()
                return "break"

    def _on_double(self, event):
        if self.tree.identify_region(event.x, event.y) != "cell":
            return
        field = self._column_at(event.x)
        iid = self.tree.identify_row(event.y)
        if field not in CredentialStore.EDITABLE or iid not in self._rows:
            return
        label = "Nombre" if field == "name" else "Notas"
        value = simpledialog.askstring(APP_TITLE, f"{label} de la credencial #{iid}:",
                                       initialvalue=self._rows[iid][field], parent=self)
        if value is not None:
            self.store.set_field(int(iid), field, value.strip())
            self.app.refresh_all()

    def _popup(self, event):
        iid = self.tree.identify_row(event.y)
        if iid and iid not in self.tree.selection():
            self.tree.selection_set(iid)
        if self.tree.selection():
            self.menu.tk_popup(event.x_root, event.y_root)

    # ---- acciones -------------------------------------------------------- #
    def mark_selected(self, used):
        rows = self._selected_rows()
        if not rows:
            messagebox.showinfo(APP_TITLE, "Selecciona al menos una credencial.", parent=self)
            return
        self.store.set_used([r["id"] for r in rows if bool(r["used"]) != used], used)
        self.app.refresh_all()

    def toggle_selected(self):
        rows = self._selected_rows()
        if rows:
            self.mark_selected(not all(r["used"] for r in rows))
        return "break"

    def delete_selected(self):
        rows = self._selected_rows()
        if not rows:
            messagebox.showinfo(APP_TITLE, "Selecciona al menos una credencial.", parent=self)
            return
        used = sum(1 for r in rows if r["used"])
        msg = f"¿Eliminar {len(rows)} credencial(es) del registro?"
        if used:
            msg += (f"\n\nAtención: {used} de ellas están marcadas como usadas. "
                    "Si las borras, se pierde la trazabilidad de esos dispositivos.")
        if messagebox.askyesno(APP_TITLE, msg, icon="warning", parent=self):
            ids = [r["id"] for r in rows]
            self.store.delete(ids)
            self.app.last_batch = [i for i in self.app.last_batch if i not in ids]
            self.app.refresh_all()

    def copy(self, what):
        rows = self._selected_rows()
        if not rows:
            return "break"
        if what == "all":
            text = "\n".join(f"DevEUI: {r['dev_eui']}\tJoinEUI: {r['join_eui']}\t"
                             f"AppKey: {r['app_key']}" for r in rows)
        else:
            text = "\n".join(r[what] for r in rows)
        self.app.to_clipboard(text, f"Copiado al portapapeles ({len(rows)} fila(s)).")
        return "break"

    def copy_c(self):
        rows = self._selected_rows()
        if not rows:
            messagebox.showinfo(APP_TITLE, "Selecciona al menos una credencial.", parent=self)
            return "break"
        self.app.to_clipboard("\n\n".join(c_snippet(r) for r in rows) + "\n",
                              f"Código C de {len(rows)} credencial(es) copiado al portapapeles.")
        return "break"

    def export(self):
        rows = self._selected_rows()
        if not rows:
            rows = list(self._rows.values())
        self.app.export_rows(rows, bool(self.tree.selection()))


# --------------------------------------------------------------------------- #
#  Ventana principal
# --------------------------------------------------------------------------- #
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1300x760")
        self.minsize(980, 600)

        self.store = CredentialStore(DB_FILE)
        self.last_batch = []
        self.tables = []

        self.sep_var = tk.StringVar(value="; (Excel en español)")
        self.mark_on_export = tk.BooleanVar(value=False)
        self.msg_var = tk.StringVar(value="Listo.")
        self.stats_var = tk.StringVar()

        style = ttk.Style(self)
        if sys.platform.startswith("linux") and "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Treeview", rowheight=24)
        style.configure("Accent.TButton", padding=(14, 6))

        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=True, padx=8, pady=(8, 0))
        self._build_generate_tab()

        registry = CredentialTable(self.nb, self, mode="all")
        self.nb.add(registry, text="  Registro  ")
        used = CredentialTable(self.nb, self, mode="used")
        self.nb.add(used, text="  Credenciales usadas  ")
        self.tables += [registry, used]

        bar = ttk.Frame(self, padding=(10, 4))
        bar.pack(fill="x")
        ttk.Label(bar, textvariable=self.msg_var).pack(side="left")
        ttk.Label(bar, textvariable=self.stats_var, foreground="#555").pack(side="right")

        self.bind("<Control-g>", lambda e: self.generate())
        self.refresh_all()

    # ---- pestaña Generar ------------------------------------------------- #
    def _build_generate_tab(self):
        tab = ttk.Frame(self.nb, padding=10)
        self.nb.add(tab, text="  Generar  ")

        form = ttk.LabelFrame(tab, text="Parámetros", padding=10)
        form.pack(fill="x")

        self.count_var = tk.StringVar(value="1")
        self.name_prefix_var = tk.StringVar(value=self.store.get_setting("name_prefix", "NODO-"))
        self.eui_prefix_var = tk.StringVar(value=self.store.get_setting("eui_prefix", ""))
        self.local_admin_var = tk.BooleanVar(value=self.store.get_setting("local_admin", "1") == "1")
        self.join_mode_var = tk.StringVar(value=self.store.get_setting("join_mode", "random"))
        self.join_fixed_var = tk.StringVar(value=self.store.get_setting("join_fixed", ""))

        ttk.Label(form, text="Cantidad:").grid(row=0, column=0, sticky="w", pady=3)
        ttk.Spinbox(form, from_=1, to=MAX_BATCH, textvariable=self.count_var,
                    width=8).grid(row=0, column=1, sticky="w", padx=6)

        ttk.Label(form, text="Prefijo de nombre:").grid(row=1, column=0, sticky="w", pady=3)
        ttk.Entry(form, textvariable=self.name_prefix_var, width=24).grid(
            row=1, column=1, sticky="w", padx=6)
        ttk.Label(form, foreground="#666",
                  text="Se numeran en secuencia: NODO-0001, NODO-0002… (vacío = sin nombre)").grid(
            row=1, column=2, sticky="w")

        ttk.Label(form, text="Prefijo DevEUI (hex, opcional):").grid(row=2, column=0, sticky="w", pady=3)
        ttk.Entry(form, textvariable=self.eui_prefix_var, width=24).grid(
            row=2, column=1, sticky="w", padx=6)
        ttk.Label(form, foreground="#666",
                  text="Ej. tu OUI/MA-L de IEEE (hasta 6 bytes). El resto se completa al azar.").grid(
            row=2, column=2, sticky="w")

        ttk.Label(form, text="JoinEUI:").grid(row=3, column=0, sticky="w", pady=3)
        join_row = ttk.Frame(form)
        join_row.grid(row=3, column=1, columnspan=2, sticky="w", padx=6)
        ttk.Radiobutton(join_row, text="Aleatorio por dispositivo", value="random",
                        variable=self.join_mode_var, command=self._sync_join).pack(side="left")
        ttk.Radiobutton(join_row, text="Fijo para todos:", value="fixed",
                        variable=self.join_mode_var, command=self._sync_join).pack(side="left", padx=(14, 4))
        self.join_entry = ttk.Entry(join_row, textvariable=self.join_fixed_var, width=26)
        self.join_entry.pack(side="left")
        ttk.Label(join_row, foreground="#666",
                  text="  Usa «Fijo» si tu servidor de red (TTN, ChirpStack…) te asigna uno.").pack(side="left")
        self._sync_join()

        ttk.Checkbutton(
            form, variable=self.local_admin_var,
            text="EUIs aleatorios administrados localmente (bit U/L = 1). "
                 "Aplica al DevEUI sin prefijo y al JoinEUI aleatorio. Recomendado si no tienes OUI propio."
        ).grid(row=4, column=0, columnspan=3, sticky="w", pady=(6, 3))

        ttk.Button(form, text="Generar credenciales  (Ctrl+G)", style="Accent.TButton",
                   command=self.generate).grid(row=5, column=0, columnspan=2, sticky="w", pady=(8, 0))
        form.columnconfigure(2, weight=1)

        batch_box = ttk.LabelFrame(tab, text="Último lote generado", padding=4)
        batch_box.pack(fill="both", expand=True, pady=(10, 0))
        self.batch_table = CredentialTable(batch_box, self, mode="batch")
        self.batch_table.pack(fill="both", expand=True)
        self.tables.append(self.batch_table)

    def _sync_join(self):
        self.join_entry.state(["!disabled"] if self.join_mode_var.get() == "fixed" else ["disabled"])

    def generate(self):
        try:
            count = int(self.count_var.get())
            if not 1 <= count <= MAX_BATCH:
                raise ValueError
        except ValueError:
            messagebox.showerror(APP_TITLE, f"La cantidad debe ser un número entre 1 y {MAX_BATCH}.")
            return
        try:
            ids = self.store.generate(count, self.name_prefix_var.get().strip(),
                                      self.eui_prefix_var.get(), self.local_admin_var.get(),
                                      self.join_mode_var.get(), self.join_fixed_var.get())
        except (ValueError, RuntimeError) as exc:
            messagebox.showerror(APP_TITLE, str(exc))
            return
        # Recordar la configuración para la próxima vez
        for key, var in (("name_prefix", self.name_prefix_var), ("eui_prefix", self.eui_prefix_var),
                         ("join_mode", self.join_mode_var), ("join_fixed", self.join_fixed_var)):
            self.store.set_setting(key, var.get().strip())
        self.store.set_setting("local_admin", "1" if self.local_admin_var.get() else "0")

        self.last_batch = ids
        self.refresh_all()
        self.batch_table.select_all()  # muestra el código C del lote recién generado
        self.set_msg(f"Se generaron {len(ids)} credencial(es) nuevas.")

    # ---- utilidades compartidas ----------------------------------------- #
    def refresh_all(self):
        for t in self.tables:
            t.refresh()
        total, used, free = self.store.stats()
        self.stats_var.set(f"Total: {total} · Disponibles: {free} · Usadas: {used} · "
                           f"BD: {os.path.basename(self.store.path)}")

    def set_msg(self, text):
        self.msg_var.set(f"[{datetime.now():%H:%M:%S}] {text}")

    def to_clipboard(self, text, msg):
        self.clipboard_clear()
        self.clipboard_append(text)
        self.set_msg(msg)

    def export_rows(self, rows, only_selected):
        if not rows:
            messagebox.showinfo(APP_TITLE, "No hay credenciales para exportar en esta vista.")
            return
        scope = "seleccionadas" if only_selected else "de la vista actual"
        path = filedialog.asksaveasfilename(
            parent=self, title=f"Exportar {len(rows)} credenciales {scope}",
            defaultextension=".csv", filetypes=[("Archivo CSV", "*.csv"), ("Todos", "*.*")],
            initialfile=f"credenciales_lorawan_{datetime.now():%Y%m%d_%H%M%S}.csv")
        if not path:
            return
        delimiter = ";" if self.sep_var.get().startswith(";") else ","
        try:
            # utf-8-sig: Excel reconoce bien los acentos
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                writer = csv.writer(f, delimiter=delimiter)
                writer.writerow(CSV_HEADER)
                for r in rows:
                    writer.writerow([r["name"], r["dev_eui"], r["join_eui"], r["app_key"],
                                     "Sí" if r["used"] else "No", r["created_at"],
                                     r["used_at"] or "", r["notes"]])
        except OSError as exc:
            messagebox.showerror(APP_TITLE, f"No se pudo guardar el archivo:\n{exc}")
            return
        if self.mark_on_export.get():
            self.store.set_used([r["id"] for r in rows if not r["used"]], True)
        self.refresh_all()
        self.set_msg(f"Exportadas {len(rows)} credenciales a {os.path.basename(path)}")


def main():
    if sys.platform == "win32":  # textos nítidos en pantallas con escalado
        try:
            import ctypes
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
    App().mainloop()


if __name__ == "__main__":
    main()