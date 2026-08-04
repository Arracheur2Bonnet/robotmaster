#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Carolus Launcher v3 — GUI sequentiel, dashboard live, mode MANUEL ZQSD."""

import os
import sys
import re
import json
import math
import queue
import tkinter as tk
from tkinter import filedialog, ttk
import threading
import subprocess
import time

from map_editor import MapEditor

MAPV1 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "mapv1.json")


# ── Carte live embarquée ──────────────────────────────────────────────────────

class _LiveMapCanvas(tk.Frame):
    """Mini-canvas de carte 2D lecture seule, embarqué dans le launcher."""
    CELL = 20          # px par case
    CELL_M = 0.40      # mètre par case

    def __init__(self, parent):
        super().__init__(parent, bg=BG2)
        self._cols = 26
        self._rows = 21
        cw = self._cols * self.CELL
        ch = self._rows * self.CELL
        self._canvas = tk.Canvas(self, width=cw, height=ch, bg="#101418",
                                 highlightthickness=1, highlightbackground="#2a2a2a")
        self._canvas.pack()
        self._robot_ids   = []
        self._beacon_ids  = []
        self._robot_px    = (cw // 2, ch // 2)   # canvas px du robot (par défaut centre)
        self._draw_grid()

    # ── dessin grille + obstacles ─────────────────────────────────────────────

    def _draw_grid(self):
        cw = self._cols * self.CELL
        ch = self._rows * self.CELL
        for c in range(self._cols + 1):
            self._canvas.create_line(c * self.CELL, 0, c * self.CELL, ch,
                                     fill="#1e2328", tags="grid")
        for r in range(self._rows + 1):
            self._canvas.create_line(0, r * self.CELL, cw, r * self.CELL,
                                     fill="#1e2328", tags="grid")

    def load_map(self, path):
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception:
            return False
        cols = data.get("cols", self._cols)
        rows = data.get("rows", self._rows)
        if cols != self._cols or rows != self._rows:
            self._cols = cols
            self._rows = rows
            cw, ch = cols * self.CELL, rows * self.CELL
            self._canvas.config(width=cw, height=ch)
            self._canvas.delete("grid")
            self._draw_grid()
        self._robot_px = (self._cols * self.CELL // 2, self._rows * self.CELL // 2)
        self._canvas.delete("block")
        for b in data.get("blocks", []):
            r, c = b.get("row", -1), b.get("col", -1)
            if not (0 <= r < rows and 0 <= c < cols):
                continue
            x1, y1 = c * self.CELL, r * self.CELL
            x2, y2 = x1 + self.CELL, y1 + self.CELL
            fill = {"full": "#505560", "half": "#3c4048", "quarter": "#303438"}.get(
                b.get("type", "full"), "#505560")
            self._canvas.create_rectangle(x1, y1, x2, y2, fill=fill, outline="", tags="block")
        self._canvas.tag_raise("overlay")
        return True

    # ── mises à jour live ─────────────────────────────────────────────────────

    def _world_to_px(self, wx, wy):
        ox, oy = self._robot_px
        sc = self.CELL / self.CELL_M
        return ox + wy * sc, oy - wx * sc   # y+=est, x+=nord

    def update_robot(self, wx, wy, yaw_deg):
        for cid in self._robot_ids:
            self._canvas.delete(cid)
        self._robot_ids = []
        px, py = self._world_to_px(wx, wy)
        s = max(5, self.CELL // 2 - 1)
        rad = math.radians(yaw_deg)
        self._robot_ids.append(self._canvas.create_rectangle(
            px - s, py - s, px + s, py + s,
            fill="#2255ee", outline="#4477ff", tags="overlay"))
        ax = px + (s + 5) * math.sin(rad)
        ay = py - (s + 5) * math.cos(rad)
        self._robot_ids.append(self._canvas.create_line(
            px, py, ax, ay, fill="white", width=2, arrow="last", tags="overlay"))

    def update_beacon(self, wx, wy):
        for cid in self._beacon_ids:
            self._canvas.delete(cid)
        self._beacon_ids = []
        px, py = self._world_to_px(wx, wy)
        r = max(5, self.CELL // 3)
        self._beacon_ids.append(self._canvas.create_oval(
            px - r, py - r, px + r, py + r,
            fill="#ff8800", outline="#ffcc44", tags="overlay"))

    def add_auto_beacon(self, wx, wy, facing_deg=0):
        self.update_beacon(wx, wy)

    def hide_beacon(self):
        for cid in self._beacon_ids:
            self._canvas.delete(cid)
        self._beacon_ids = []

    def reset_overlay(self):
        for cid in self._robot_ids + self._beacon_ids:
            self._canvas.delete(cid)
        self._robot_ids  = []
        self._beacon_ids = []

PI       = "ubuntu@192.168.0.103"
PI_HOST  = "192.168.0.103"

# Chemins derives de l'emplacement du script, pas codes en dur (2026-08-04).
# WS pointait vers un chemin absolu sous le home d'un seul poste : le launcher
# etait donc inutilisable ailleurs, et ce chemin exposait un nom d'utilisateur
# local dans un depot destine a devenir public. Le launcher vit dans
# <projet>/shortcuts/, donc le workspace est <projet>/carolus_ws. Surchargeable
# par CAROLUS_WS si la disposition differe.
HERE     = os.path.dirname(os.path.abspath(__file__))
_ROOT    = os.path.dirname(HERE)
WS       = os.environ.get("CAROLUS_WS", os.path.join(_ROOT, "carolus_ws"))
BEACON_PI = "/home/ubuntu/carolus_ws/src/robomaster_cam/scripts/rm_cam_beacon.py"
TF_BROADCASTER_PI = "/home/ubuntu/carolus_ws/src/carolus_node/scripts/carolus_tf_broadcaster.py"
# Docking (2026-07-27) : tourne sur le PC labo, pas le Pi -- pas de connexion SDK
# propre (commande via /carolus/cmd_vel, deja relaye par rm_cam_beacon.py), donc
# pas de contrainte "un seul proprietaire SDK" ici, meme raisonnement que T3.
DOCKING_SCRIPT = os.path.join(WS, "src/robomaster_cam/scripts/beacon_docking.py")
# MINS (2026-08-04) : tourne sur le Pi, dans son propre workspace, volontairement
# separe de carolus_ws (bac a sable jetable tant que MINS n'est pas integre).
# Mesure du 2026-08-04 : simulation.launch marche et est precis (RMSE 0.113 deg /
# 0.082 m) mais tourne a 0.3-0.4x temps reel AVEC la charge capteur de la simu
# (2 cameras + LIDAR + IMU 200Hz) -- bien plus lourde que la notre.
MINS_WS_PI    = "/home/ubuntu/mins_sandbox_ws"
MINS_LAUNCH   = "simulation.launch"
SSH_KEY  = os.path.expanduser("~/.ssh/carolus_nopass")
SSH_OPTS = ["-i", SSH_KEY, "-o", "IdentitiesOnly=yes", "-o", "StrictHostKeyChecking=no"]

HELPER  = os.path.join(HERE, "cam_view_helper.py")   # HERE defini plus haut
CAM_PNG = "/tmp/carolus_cam.png"

# ── statuts ───────────────────────────────────────────────────────────────────
S_IDLE    = "[ ]"
S_LOCKED  = "[--]"
S_RUNNING = "[...]"
S_OK      = "[OK]"

# ── palette sombre "robotique" ─────────────────────────────────────────────────
BG      = "#0d1117"   # fond principal
BG2     = "#161b22"   # panneaux
BG3     = "#21262d"   # boutons / champs
FG      = "#e6edf3"   # texte clair
FG_DIM  = "#8b949e"   # texte secondaire
ACCENT  = "#00d9a0"   # cyan-vert robotique

# ── couleurs etat robot / connexion ────────────────────────────────────────────
COL_SEARCH   = "#888888"
COL_ALIGN    = "#d4a017"   # jaune-or : orientation en cours
COL_APPROACH = "#e08000"
COL_STOP     = "#2e8b2e"
COL_IDLE     = "#3a3a3a"
COL_MANUAL   = "#1f6feb"
COL_OK       = "#2e8b2e"
COL_KO       = "#cc0000"
COL_KEY_ACT  = "#d4a017"   # or de touche active (ZQSD / numpad)

# Lignes de telemetrie haute frequence : dashboard uniquement, ne pas encombrer le log texte
_LOG_SUPPRESS = frozenset({"[ESC]", "[ATTI]", "[POS]", "[BAT]", "[VEL]", "[TOF]", "[BEACON]"})
# [BEACONPOS] volontairement hors du filtre : valeurs brutes utiles pour diagnostiquer l'orientation

RE_DEPTH     = re.compile(r"depth=([0-9.]+)m")
RE_BAT       = re.compile(r"\[BAT\]\s*(\d+)%\s*temp=([+-]?[0-9.]+)C\s*cur=(-?\d+)mA\s*adc=(\d+)")
RE_ESC       = re.compile(r"\[ESC\]\s*W1=(-?\d+)\s*W2=(-?\d+)\s*W3=(-?\d+)\s*W4=(-?\d+)")
RE_ATTI      = re.compile(r"\[ATTI\]\s*yaw=([+-]?[0-9.]+)\s*pitch=([+-]?[0-9.]+)\s*roll=([+-]?[0-9.]+)")
RE_POS       = re.compile(r"\[POS\]\s*x=([+-]?[0-9.]+)\s*y=([+-]?[0-9.]+)")
RE_BEACONPOS = re.compile(r"\[BEACONPOS\]\s*x=([+-]?[0-9.]+)\s*z=([+-]?[0-9.]+)(?:\s*byaw=([+-]?[0-9.]+))?")
RE_VEL       = re.compile(r"\[VEL\]\s*vbx=([+-]?[0-9.]+)\s*vby=([+-]?[0-9.]+)")
RE_STATUS    = re.compile(r"\[STATUS\]\s*pickup=(\d)\s*slip=(\d)\s*roll=(\d)\s*slope=(\d)\s*impact=(\d)")
RE_TOF       = re.compile(r"\[TOF\]\s*front=([0-9.]+)cm")
RE_OBSTACLE  = re.compile(r"\[OBSTACLE\]\s*(.+)")
RE_BEACON    = re.compile(r"\[BEACON\]\s*status=(DETECTED|LOST)(?:\s*yaw_err=([+-]?[0-9.]+)\s*pitch_err=([+-]?[0-9.]+))?")
RE_DOCKSTATUS = re.compile(r"\[DOCKSTATUS\]\s*status=(\S+)\s*yaw_validated=(True|False)")

BEACON_FRESH_S = 1.5   # doit matcher POSE_TIMEOUT_S dans rm_cam_beacon.py

# Polices nommees SANS taille explicite (taille explicite -> crash X11 BadLength, cf journal 2026-06-23)
FONT      = ("TkDefaultFont",)
FONT_MONO = ("TkFixedFont",)

# Debounce anti auto-repeat X11 (2026-07-23, Perplexity 11 pt.6) : X11 emet un
# KeyRelease immediatement suivi d'un KeyPress pendant qu'une touche est tenue
# -- sans filtre, chaque relachement declenchait un STOP/re-envoi parasite,
# produisant le dents-de-scie observe sur /carolus/cmd_vel (x: 0.2/0.0/0.2/0.0...).
KEY_REPEAT_DEBOUNCE_MS = 40

MANUAL_VX    = 0.20   # m/s avant/arriere en mode manuel
MANUAL_WZ    = 20.0   # deg/s rotation en mode manuel
GIMBAL_PITCH = 30.0   # deg/s pitch gimbal (numpad 8/2)
GIMBAL_YAW   = 40.0   # deg/s yaw gimbal (numpad 4/6)


# ── Kill distants (SSH) / locaux ────────────────────────────────────────────────

def ssh_kill(cmd):
    subprocess.run(
        ["ssh"] + SSH_OPTS + ["-o", "ConnectTimeout=3", PI, cmd],
        capture_output=True
    )


def local_kill(pattern):
    subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)


# ── Interface ────────────────────────────────────────────────────────────────

class App(tk.Tk):

    def __init__(self):
        super().__init__()
        # Log de session sur disque (2026-07-31) — ouvert EN PREMIER, avant tout
        # ce qui pourrait appeler _log(). Meme precaution que BUG-065/BUG-068
        # (publisher/timer utilises avant leur creation) : ici _log_fh doit
        # exister avant le premier _log_to_disk, sinon AttributeError dans un
        # chemin de log, c'est-a-dire au pire moment possible.
        self._log_fh = None
        self._session_log_path = None
        self._open_session_log()

        self.title("Carolus Launcher")
        self.configure(bg=BG)
        self.resizable(False, False)
        self.procs      = [None, None, None, None, None, None]   # Popen T1..T6
        self.cam_proc   = None                 # Popen helper video (stdin=PIPE)
        self.cam_img    = None                 # reference PhotoImage (anti-GC)
        self.last_state = None
        self.gui_mode   = "AUTO"               # "AUTO" ou "MANUAL"
        self._keys_down = set()
        self._gim_down  = set()
        self._chassis_release_pending = {}   # touche -> id after() en attente (debounce X11)
        self._gim_release_pending     = {}   # idem pour le numpad nacelle
        self._stop_monitor   = False
        self._launch_cancelled = [False, False, False, False, False]   # annulation wait_for_*
        self._log_queue  = queue.Queue()       # lignes T2 integre -> main thread
        self._chassis_btns = {}                # label widgets touches ZQSD
        self._gimbal_btns  = {}                # label widgets numpad 8/4/5/6/2
        # Editeur de map (Toplevel séparé) + carte live embarquée
        self._map_editor     = None    # instance MapEditor si ouverte
        self._live_map       = None    # _LiveMapCanvas (construit dans _build)
        self._locate_active  = False   # mode LOCALISER actif
        # Apercu camera GUI : OFF par defaut (2026-07-23) — gagner en fluidite de
        # pilotage (Tkinter mainloop moins charge, cf. Perplexity 11 pt.5) ET
        # liberer de la bande passante reseau : cam_view_helper.py s'abonne au meme
        # topic /camera/color/image_raw que Carolus, deja identifie comme goulot
        # (Perplexity 11 pt.4) — un abonne ROS en moins sur ce topic = moins de
        # trafic duplique sur le lien Pi<->labo.
        self._camera_enabled = False
        self._gimbal_lock_active = False   # lock balise actif (centrage periodique, mode MANUEL uniquement)
        self._last_robot_pos = (0.0, 0.0)   # dernière pos sub_position (m)
        self._last_robot_yaw = 0.0          # dernière orientation robot (deg, sub_attitude)
        self._last_beacon_ts = 0.0          # horodatage dernier [BEACONPOS] recu
        self._t5_dock_ready = False         # 1er [DOCKSTATUS] vu -> abonnements T5 etablis
        self._build()
        self._bind_keys()
        self._refresh_cam()
        self.after(100, self._flush_log_queue)
        self.after(300, self._check_beacon_freshness)
        # Sonde Pi : premier tir a 2s (laisse la fenetre s'afficher d'abord),
        # puis toutes les PI_PROBE_PERIOD_MS. Voir _pi_state_probe.
        self.after(2000, self._pi_state_tick)
        # Auto-chargement de la carte par défaut
        if os.path.exists(MAPV1):
            self._live_map.load_map(MAPV1)
        threading.Thread(target=self._conn_monitor, daemon=True).start()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        # Plein ecran (2026-07-23) : F11 pour basculer, Echap pour sortir.
        self._is_fullscreen = False
        self.bind("<F11>", self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)

    # ── construction UI ──────────────────────────────────────────────────────

    def _build(self):
        # ── split horizontal : left = contrôles, right = carte live ──────────
        left_col = tk.Frame(self, bg=BG)
        left_col.pack(side="left", fill="y")
        right_col = tk.Frame(self, bg=BG2, padx=8, pady=8)
        right_col.pack(side="left", fill="y")
        self._build_live_map(right_col)

        # --- header robotique ---
        header = tk.Frame(left_col, bg=BG2)
        header.pack(fill="x")
        tk.Label(header, text="  >> CAROLUS LAUNCHER", bg=BG2, fg=ACCENT,
                 anchor="w", font=FONT).pack(fill="x", padx=12, pady=(8, 0))
        tk.Label(header, text="  RoboMaster S1 - Custom Raspberry Pi", bg=BG2, fg=FG,
                 anchor="w", font=FONT).pack(fill="x", padx=12)
        conn = tk.Frame(header, bg=BG2)
        conn.pack(fill="x", padx=12, pady=(2, 8))
        tk.Label(conn, text="Connexion Pi :", bg=BG2, fg=FG_DIM, font=FONT).pack(side="left")
        self.conn_dot = tk.Canvas(conn, width=14, height=14, bg=BG2, highlightthickness=0)
        self._conn_oval = self.conn_dot.create_oval(2, 2, 12, 12, fill=COL_IDLE, outline="")
        self.conn_dot.pack(side="left", padx=6)
        self.conn_lbl = tk.Label(conn, text="verification...", bg=BG2, fg=FG_DIM, font=FONT)
        self.conn_lbl.pack(side="left")

        tk.Frame(left_col, height=2, bg=ACCENT).pack(fill="x")

        # --- controles (1 ligne par terminal) ---
        self.rows = []          # (launch_btn, status_lbl, kill_btn)
        specs = [
            ("1  roscore + Pi",              True),    # (label, enabled)
            ("2  Camera + Beacon",           False),
            ("3  Carolus Astrobee",          False),
            ("4  TF Broadcaster (quat fix)", False),
            ("5  Beacon Docking",            False),
            # T6 : independant du reste (tourne sur le Pi, sur son propre
            # roscore local). Laisse activable des le depart -- il n'attend
            # aucun topic de notre pipeline tant qu'il tourne sa simulation.
            ("6  MINS (simulation, Pi)",      True),
        ]
        body = tk.Frame(left_col, bg=BG)
        body.pack(fill="x", padx=12, pady=8)
        for i, (label, enabled) in enumerate(specs):
            f = tk.Frame(body, bg=BG)
            f.pack(fill="x", pady=3)
            launch = tk.Button(f, text=f"> {label}", width=24, anchor="w",
                               bg=BG3, fg=FG, activebackground=ACCENT, activeforeground=BG,
                               relief="flat", font=FONT,
                               state="normal" if enabled else "disabled",
                               command=lambda i=i: self._on_launch(i))
            launch.pack(side="left")
            status = tk.Label(f, text=S_IDLE if enabled else S_LOCKED, width=6,
                              bg=BG, fg=FG, anchor="w", font=FONT_MONO)
            status.pack(side="left", padx=4)
            kill = tk.Button(f, text="Kill", width=5,
                             bg=BG3, fg=COL_KO, activebackground=COL_KO, activeforeground=FG,
                             relief="flat", font=FONT,
                             command=lambda i=i: self._on_kill(i))
            kill.pack(side="left", padx=2)
            self.rows.append((launch, status, kill))

        # --- kill all + bouton mode ---
        ctrl = tk.Frame(left_col, bg=BG)
        ctrl.pack(fill="x", padx=12, pady=(0, 6))
        tk.Button(ctrl, text="KILL ALL", bg=COL_KO, fg=FG, relief="flat",
                  activebackground="#ff3333", activeforeground=FG, font=FONT,
                  command=lambda: self._on_kill(-1)).pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.mode_btn = tk.Button(ctrl, text="MODE : AUTO", bg=COL_STOP, fg=FG, relief="flat",
                                  activebackground=ACCENT, activeforeground=BG, font=FONT,
                                  command=self._toggle_mode)
        self.mode_btn.pack(side="left", fill="x", expand=True, padx=(4, 4))
        self._locate_btn = tk.Button(ctrl, text="LOCALISER", bg=BG3, fg=FG_DIM, relief="flat",
                                     activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                                     command=self._toggle_locate)
        self._locate_btn.pack(side="left", fill="x", expand=True, padx=(4, 4))
        self._lock_btn = tk.Button(ctrl, text="LOCK : OFF", bg=BG3, fg=FG_DIM, relief="flat",
                                   activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                                   command=self._toggle_gimbal_lock)
        self._lock_btn.pack(side="left", fill="x", expand=True, padx=(4, 4))
        # Periode de centrage LOCK, configurable en direct (2026-07-23) : champ de
        # saisie en SECONDES uniquement, defaut 2s. Une valeur invalide n'est jamais
        # bloquante -- rm_cam_beacon.py retombe silencieusement sur 2.0s (cf.
        # _gimbal_lock_period_cb), comme un champ de formulaire web classique.
        self._lock_period_entry = tk.Entry(ctrl, width=4, bg=BG3, fg=FG,
                                           insertbackground=FG, relief="flat", font=FONT_MONO,
                                           justify="center")
        self._lock_period_entry.insert(0, "3")
        self._lock_period_entry.bind("<Return>", self._on_lock_period_changed)
        self._lock_period_entry.pack(side="left", padx=(0, 4))
        tk.Label(ctrl, text="s", bg=BG, fg=FG_DIM, font=FONT).pack(side="left", padx=(0, 4))
        self._cam_btn = tk.Button(ctrl, text="APERCU CAM : OFF", bg=BG3, fg=FG_DIM, relief="flat",
                                  activebackground=ACCENT, activeforeground=BG, font=FONT,
                                  command=self._toggle_camera_preview)
        self._cam_btn.pack(side="left", fill="x", expand=True, padx=(4, 4))
        tk.Button(ctrl, text="ÉDITEUR MAP", bg=BG3, fg=ACCENT, relief="flat",
                  activebackground=ACCENT, activeforeground=BG, font=FONT,
                  command=self._open_map_editor).pack(side="left", fill="x", expand=True, padx=(4, 0))

        # --- blocs de pilotage visuel ---
        self._build_ctrl_blocks(left_col)

        # --- dashboard (2 colonnes) ---
        dash = tk.Frame(left_col, bg=BG)
        dash.pack(fill="x", padx=12, pady=4)

        left = tk.Frame(dash, bg=BG2)
        left.grid(row=0, column=0, sticky="nw", padx=(0, 8), ipadx=8, ipady=6)
        tk.Label(left, text="ETAT ROBOT", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w")
        srow = tk.Frame(left, bg=BG2)
        srow.pack(anchor="w", pady=2)
        self.state_dot = tk.Canvas(srow, width=18, height=18, bg=BG2, highlightthickness=0)
        self._dot = self.state_dot.create_oval(2, 2, 16, 16, fill=COL_IDLE, outline="")
        self.state_dot.pack(side="left")
        self.state_lbl = tk.Label(srow, text="---", bg=BG2, fg=FG, font=FONT)
        self.state_lbl.pack(side="left", padx=6)
        self.depth_lbl = tk.Label(left, text="", bg=BG2, fg=FG, font=FONT_MONO)
        self.depth_lbl.pack(anchor="w")
        tk.Label(left, text="Batterie", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(8, 0))
        brow = tk.Frame(left, bg=BG2)
        brow.pack(anchor="w", pady=2)
        self.batt_canvas = tk.Canvas(brow, width=120, height=16, bg=BG3, highlightthickness=0)
        self._batt_rect = self.batt_canvas.create_rectangle(0, 0, 0, 16, fill=COL_STOP, outline="")
        self.batt_canvas.pack(side="left")
        self.batt_lbl = tk.Label(brow, text="N/A", width=6, bg=BG2, fg=FG, font=FONT)
        self.batt_lbl.pack(side="left", padx=6)

        tk.Label(left, text="Batterie — détail", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(8, 0))
        self.bat_detail_lbl = tk.Label(left, text="temp: N/A   courant: N/A   adc: N/A",
                                       bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.bat_detail_lbl.pack(anchor="w")

        tk.Label(left, text="Attitude", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(8, 0))
        self.atti_lbl = tk.Label(left, text="pitch: N/A   roll: N/A", bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.atti_lbl.pack(anchor="w")

        tk.Label(left, text="Vitesse châssis", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.vel_lbl = tk.Label(left, text="vx: N/A   vy: N/A",
                                bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.vel_lbl.pack(anchor="w")

        tk.Label(left, text="Roues (RPM)", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.esc_lbl = tk.Label(left, text="W1:--- W2:--- W3:--- W4:---", bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.esc_lbl.pack(anchor="w")

        tk.Label(left, text="Statut", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.status_lbl = tk.Label(left, text="OK", bg=BG2, fg=ACCENT, font=FONT_MONO, anchor="w")
        self.status_lbl.pack(anchor="w")

        tk.Label(left, text="TOF frontal / Obstacle", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.tof_lbl = tk.Label(left, text="N/A", bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.tof_lbl.pack(anchor="w")

        # --- Etat du Raspberry Pi (2026-08-04) -------------------------------
        # Temperature/charge/RAM du Pi, pas du robot : sur ce projet le Pi porte
        # desormais TOUT le pipeline de perception (camera + Carolus, et a terme
        # MINS), donc c'est lui qui sature en premier. Le test MINS du 2026-08-04
        # a mesure un coeur a 100-118% pendant que trois restaient au repos --
        # une info invisible depuis le robot, et decisive pour comprendre un
        # ralentissement. Lu par SSH, pas par ROS : ca reste vrai meme si la
        # stack ROS est arretee ou plantee.
        tk.Label(left, text="Raspberry Pi", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.pi_lbl = tk.Label(left, text="temp --  load --  ram --",
                               bg=BG2, fg=FG_DIM, font=FONT_MONO, anchor="w")
        self.pi_lbl.pack(anchor="w")

        right = tk.Frame(dash, bg=BG2)
        right.grid(row=0, column=1, sticky="ne", ipadx=4, ipady=4)
        tk.Label(right, text="CAMERA (apercu ~20 Hz)", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w")
        self.cam_canvas = tk.Canvas(right, width=320, height=180, bg="black", highlightthickness=0)
        self.cam_canvas.pack()
        self._cam_txt = self.cam_canvas.create_text(160, 90, text="en attente...", fill=FG_DIM)

        # --- Voyant statut balise + minimap (2026-07-23) ---
        beacon_row = tk.Frame(right, bg=BG2)
        beacon_row.pack(fill="x", pady=(6, 0))
        self._beacon_dot = tk.Canvas(beacon_row, width=14, height=14, bg=BG2, highlightthickness=0)
        self._beacon_dot_id = self._beacon_dot.create_oval(2, 2, 12, 12, fill=COL_KO, outline="")
        self._beacon_dot.pack(side="left")
        self._beacon_status_lbl = tk.Label(beacon_row, text="BEACON: LOST", bg=BG2, fg=COL_KO, font=FONT_MONO)
        self._beacon_status_lbl.pack(side="left", padx=6)

        tk.Label(right, text="MINIMAP BALISE", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self._minimap = tk.Canvas(right, width=100, height=100, bg="black", highlightthickness=0)
        self._minimap.pack()
        self._minimap_cross_h = self._minimap.create_line(0, 50, 100, 50, fill="#444444")
        self._minimap_cross_v = self._minimap.create_line(50, 0, 50, 100, fill="#444444")
        self._minimap_dot = self._minimap.create_oval(46, 46, 54, 54, fill=COL_IDLE, outline="", state="hidden")
        self._beacon_detected = False

        # RECENTRER CAM (2026-07-23) : remet la nacelle a sa position de base
        # (pitch=0, yaw=0, gimbal.recenter() du SDK) -- orientation de la CAMERA,
        # independant de l'orientation du chassis robot.
        self._recenter_btn = tk.Button(right, text="RECENTRER CAM", bg=BG3, fg=FG,
                                       relief="flat", activebackground=COL_ALIGN, activeforeground=FG,
                                       font=FONT, command=self._on_gimbal_recenter)
        self._recenter_btn.pack(fill="x", pady=(6, 0))

        # --- Docking (2026-07-27) : commandes sur /carolus/dock (relayees par
        # cam_view_helper.py, meme mecanisme que RECENTER), statut lu depuis les
        # logs de T5 ([DOCKSTATUS], meme mecanisme que [BEACON]). ---
        tk.Label(right, text="DOCKING BALISE", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w", pady=(10, 0))
        # Calibration en 2 clics independants (2026-07-27) : pas de minuteur bloquant
        # entre les deux mesures -- chaque etape attend un clic explicite, a lire sur
        # le statut GUI plutot que de guetter un message dans un log qui defile.
        dock_cal_row = tk.Frame(right, bg=BG2)
        dock_cal_row.pack(fill="x", pady=(2, 0))
        tk.Button(dock_cal_row, text="CALIBRATE (1)", bg=BG3, fg=FG, relief="flat",
                  activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                  command=lambda: self._on_dock_cmd("CALIBRATE")).pack(side="left", fill="x", expand=True, padx=(0, 2))
        tk.Button(dock_cal_row, text="CAL STEP 2", bg=BG3, fg=FG, relief="flat",
                  activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                  command=lambda: self._on_dock_cmd("CALSTEP2")).pack(side="left", fill="x", expand=True, padx=(2, 0))
        # Tests isoles (2026-07-28) : ALIGN_ONLY tourne le chassis SANS jamais
        # avancer ; APPROACH_ONLY avance SANS jamais tourner le chassis (et
        # refuse si le chassis n'est pas deja aligne -- voir beacon_docking.py).
        dock_test_row = tk.Frame(right, bg=BG2)
        dock_test_row.pack(fill="x", pady=(2, 0))
        tk.Button(dock_test_row, text="ALIGN ONLY", bg=BG3, fg=FG, relief="flat",
                  activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                  command=lambda: self._on_dock_cmd("ALIGN_ONLY")).pack(side="left", fill="x", expand=True, padx=(0, 2))
        tk.Button(dock_test_row, text="APPROACH ONLY", bg=BG3, fg=FG, relief="flat",
                  activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                  command=lambda: self._on_dock_cmd("APPROACH_ONLY")).pack(side="left", fill="x", expand=True, padx=(2, 0))
        dock_row = tk.Frame(right, bg=BG2)
        dock_row.pack(fill="x", pady=(2, 0))
        tk.Button(dock_row, text="START", bg=BG3, fg=FG, relief="flat",
                  activebackground=COL_OK, activeforeground=FG, font=FONT,
                  command=lambda: self._on_dock_cmd("START")).pack(side="left", fill="x", expand=True, padx=(0, 2))
        tk.Button(dock_row, text="ABORT", bg=BG3, fg=COL_KO, relief="flat",
                  activebackground=COL_KO, activeforeground=FG, font=FONT,
                  command=lambda: self._on_dock_cmd("ABORT")).pack(side="left", fill="x", expand=True, padx=(2, 0))
        self._dock_status_lbl = tk.Label(right, text="DOCK: —", bg=BG2, fg=FG_DIM, font=FONT_MONO)
        self._dock_status_lbl.pack(anchor="w", pady=(4, 0))

        # --- logs : un onglet par terminal (T1-T5), selectionnables + bouton copier ---
        logh = tk.Frame(left_col, bg=BG)
        logh.pack(fill="x", padx=12, pady=(6, 0))
        tk.Label(logh, text="Logs :", bg=BG, fg=FG_DIM, font=FONT).pack(side="left")
        tk.Button(logh, text="Copier les logs (onglet actif)", bg=BG3, fg=FG, relief="flat",
                  activebackground=ACCENT, activeforeground=BG, font=FONT,
                  command=self._copy_logs).pack(side="right")

        style = ttk.Style()
        style.theme_use("default")
        style.configure("Carolus.TNotebook", background=BG, borderwidth=0)
        style.configure("Carolus.TNotebook.Tab", background=BG3, foreground=FG,
                        padding=(10, 4), font=FONT)
        style.map("Carolus.TNotebook.Tab",
                  background=[("selected", ACCENT)],
                  foreground=[("selected", BG)])

        self.log_nb = ttk.Notebook(left_col, style="Carolus.TNotebook")
        self.log_nb.pack(padx=12, pady=(2, 12), fill="both", expand=True)

        self.log_boxes = []
        tab_labels = ["T1 roscore+Pi", "T2 Camera+Beacon", "T3 Carolus Astrobee", "T4 TF Broadcaster", "T5 Docking", "T6 MINS"]
        for label in tab_labels:
            box = tk.Text(self.log_nb, height=16, width=66, bg=BG2, fg=FG,
                          insertbackground=FG, relief="flat", padx=6, pady=4,
                          font=FONT_MONO, wrap="none", state="disabled")
            # lecture seule MAIS selectionnable/copiable. state="disabled" (2026-07-23,
            # BUG-061) : avant ce fix, _block_edit laissait passer Z/Q/S/D/numpad vers le
            # root (pour piloter le robot depuis un onglet de logs focus) mais ca laissait
            # AUSSI le binding par defaut de Text inserer le caractere dans le log — les
            # touches de pilotage s'affichaient litteralement dans le texte, et la charge
            # d'edition (insert + reflow) sur le thread Tkinter ajoutait de la latence
            # perceptible au pilotage. Text disabled refuse tout insert/delete (y compris
            # le notre : _log() doit temporairement repasser en "normal" pour ecrire),
            # mais tag_add (selection, Ctrl+A) et .get() (copie) ne sont PAS bloques par
            # cet etat — le copier/selectionner reste fonctionnel.
            box.bind("<Key>", self._block_edit)
            box.bind("<Control-a>", lambda e, b=box: self._select_all_logs(e, b))
            box.bind("<Control-A>", lambda e, b=box: self._select_all_logs(e, b))
            self.log_nb.add(box, text=label)
            self.log_boxes.append(box)

    # ── blocs de pilotage (ZQSD + numpad) ────────────────────────────────────

    def _build_ctrl_blocks(self, parent):
        blocks = tk.Frame(parent, bg=BG)
        blocks.pack(fill="x", padx=12, pady=(0, 4))

        # ── Bloc 1 : Chassis (ZQSD) ──────────────────────────────────────────
        ch = tk.Frame(blocks, bg=BG2, padx=10, pady=6)
        ch.pack(side="left", fill="both", expand=True, padx=(0, 4))
        tk.Label(ch, text="CHASSIS  (ZQSD)", bg=BG2, fg=ACCENT,
                 anchor="w", font=FONT).pack(anchor="w", pady=(0, 4))
        ch_keys = tk.Frame(ch, bg=BG2)
        ch_keys.pack()
        self._chassis_btns['z'] = self._make_key_btn(ch_keys, "Z",  row=0, col=1)
        self._chassis_btns['q'] = self._make_key_btn(ch_keys, "Q",  row=1, col=0)
        self._chassis_btns['s'] = self._make_key_btn(ch_keys, "S",  row=1, col=1)
        self._chassis_btns['d'] = self._make_key_btn(ch_keys, "D",  row=1, col=2)
        for key, btn in self._chassis_btns.items():
            btn.bind("<ButtonPress-1>",   lambda e, k=key: self._on_chassis_btn_press(k))
            btn.bind("<ButtonRelease-1>", lambda e, k=key: self._on_chassis_btn_release(k))

        # ── Bloc 2 : Nacelle (numpad 8/4/5/6/2) ──────────────────────────────
        gm = tk.Frame(blocks, bg=BG2, padx=10, pady=6)
        gm.pack(side="left", fill="both", expand=True, padx=(4, 0))
        tk.Label(gm, text="NACELLE  (NUM 8/4/5/6/2)", bg=BG2, fg=ACCENT,
                 anchor="w", font=FONT).pack(anchor="w", pady=(0, 4))
        gm_keys = tk.Frame(gm, bg=BG2)
        gm_keys.pack()
        self._gimbal_btns['up']    = self._make_key_btn(gm_keys, "8",  row=0, col=1)
        self._gimbal_btns['left']  = self._make_key_btn(gm_keys, "4",  row=1, col=0)
        self._gimbal_btns['stop']  = self._make_key_btn(gm_keys, "5",  row=1, col=1)
        self._gimbal_btns['right'] = self._make_key_btn(gm_keys, "6",  row=1, col=2)
        self._gimbal_btns['down']  = self._make_key_btn(gm_keys, "2",  row=2, col=1)
        for action, btn in self._gimbal_btns.items():
            btn.bind("<ButtonPress-1>",   lambda e, a=action: self._on_gimbal_btn_press(a))
            btn.bind("<ButtonRelease-1>", lambda e, a=action: self._on_gimbal_btn_release(a))

        # ── Bloc 3 : Roues individuelles (tilt / wheelie) ────────────────────
        wr = tk.Frame(blocks, bg=BG2, padx=10, pady=6)
        wr.pack(side="left", fill="both", expand=True, padx=(4, 0))
        tk.Label(wr, text="ROUES (tilt)", bg=BG2, fg=ACCENT,
                 anchor="w", font=FONT).pack(anchor="w", pady=(0, 4))
        wr_keys = tk.Frame(wr, bg=BG2)
        wr_keys.pack()
        # Avant (roues arriere poussent, avant se leve)
        b_av = tk.Label(wr_keys, text="AV↑", width=4, height=1,
                        bg=BG3, fg=FG, font=FONT_MONO, bd=1, relief="raised")
        b_av.grid(row=0, column=0, padx=3, pady=3)
        # Stop roues
        b_st = tk.Label(wr_keys, text="■", width=4, height=1,
                        bg=BG3, fg=COL_KO, font=FONT_MONO, bd=1, relief="raised")
        b_st.grid(row=0, column=1, padx=3, pady=3)
        # Arriere (roues avant poussent, arriere se leve)
        b_ar = tk.Label(wr_keys, text="AR↑", width=4, height=1,
                        bg=BG3, fg=FG, font=FONT_MONO, bd=1, relief="raised")
        b_ar.grid(row=0, column=2, padx=3, pady=3)

        b_av.bind("<ButtonPress-1>",   lambda e: self._on_tilt_press("0 0 300 300"))
        b_av.bind("<ButtonRelease-1>", lambda e: self._on_tilt_release())
        b_st.bind("<ButtonPress-1>",   lambda e: self._on_tilt_release())
        b_ar.bind("<ButtonPress-1>",   lambda e: self._on_tilt_press("300 300 0 0"))
        b_ar.bind("<ButtonRelease-1>", lambda e: self._on_tilt_release())

        tk.Label(wr, text="Maintenir enfonce", bg=BG2, fg=FG_DIM,
                 font=FONT).pack(anchor="w", pady=(4, 0))

    # ── éditeur de map ───────────────────────────────────────────────────────

    def _open_map_editor(self):
        if self._map_editor is not None:
            try:
                self._map_editor.lift()
                self._map_editor.focus()
                return
            except tk.TclError:
                self._map_editor = None
        self._map_editor = MapEditor(self)
        self._map_editor.protocol("WM_DELETE_WINDOW", self._close_map_editor)
        self._bind_keys_to(self._map_editor)   # ZQSD/numpad actifs sans recliquer sur launcher

    def _close_map_editor(self):
        if self._map_editor:
            self._map_editor.destroy()
        self._map_editor = None

    # ── carte live embarquée ──────────────────────────────────────────────────

    def _build_live_map(self, parent):
        tk.Label(parent, text="MAP LIVE", bg=BG2, fg=ACCENT, font=FONT).pack(anchor="w", pady=(0, 4))
        self._live_map = _LiveMapCanvas(parent)
        self._live_map.pack()
        btn_row = tk.Frame(parent, bg=BG2)
        btn_row.pack(fill="x", pady=(6, 0))
        tk.Button(btn_row, text="Charger map...", bg=BG3, fg=FG, relief="flat",
                  activebackground=ACCENT, activeforeground=BG, font=FONT,
                  command=self._load_live_map).pack(side="left", fill="x", expand=True)

    def _load_live_map(self):
        path = filedialog.askopenfilename(
            parent=self,
            filetypes=[("Map JSON", "*.json"), ("Tous", "*.*")],
            initialdir=os.path.expanduser("~/Desktop"),
            title="Charger une carte")
        if path:
            self._live_map.load_map(path)

    def _toggle_locate(self):
        self._locate_active = not self._locate_active
        if self._locate_active:
            self._locate_btn.config(bg=COL_ALIGN, fg=FG)
            self._send_to_helper("MODE LOCATE")
        else:
            self._locate_btn.config(bg=BG3, fg=FG_DIM)
            self._send_to_helper("MODE AUTO")

    def _toggle_gimbal_lock(self):
        # Actif uniquement en mode MANUEL cote rm_cam_beacon.py (ignore silencieusement
        # en AUTO/LOCATE) — le bouton reste utilisable dans tous les modes sans risque.
        # Centrage periodique (cf. rm_cam_beacon.py::_gimbal_lock_tick), pas un servo
        # continu -- l'ancien LOCK BALISE (servo continu) a ete retire le 2026-07-23.
        self._gimbal_lock_active = not self._gimbal_lock_active
        if self._gimbal_lock_active:
            self._lock_btn.config(text="LOCK : ON", bg=COL_ALIGN, fg=FG)
            self._send_to_helper("LOCK ON")
            self.after(0, self._log, "> Lock balise ON (centrage periodique en mode MANUEL)")
        else:
            self._lock_btn.config(text="LOCK : OFF", bg=BG3, fg=FG_DIM)
            self._send_to_helper("LOCK OFF")
            self.after(0, self._log, "> Lock balise OFF")

    def _on_lock_period_changed(self, event=None):
        # Periode configurable en direct (2026-07-23), SECONDES uniquement. Aucune
        # validation bloquante ici : on envoie tel quel, rm_cam_beacon.py retombe sur
        # 2.0s si ce n'est pas un nombre valide (cf. _gimbal_lock_period_cb) -- comme
        # un champ de formulaire web qui ignore une saisie incorrecte sans planter.
        value = self._lock_period_entry.get().strip()
        self._send_to_helper(f"LOCKPERIOD {value}")
        self.after(0, self._log, f"> Periode de centrage LOCK -> {value}s (ou repli 2s si invalide)")

    def _on_gimbal_recenter(self):
        self._send_to_helper("RECENTER")
        self.after(0, self._log, "> RECENTRER CAM — nacelle vers position de base")

    def _on_dock_cmd(self, cmd):
        # T5 doit tourner pour que la commande ait un effet (personne n'est
        # abonne a /carolus/dock sinon) -- pas de garde bloquante ici, le
        # bouton reste utilisable a tout moment, meme raisonnement que LOCK.
        self._send_to_helper(f"DOCK {cmd}")
        self.after(0, self._log, f"> DOCK {cmd}")

    def _on_dock_status(self, status, yaw_validated):
        """Parse [DOCKSTATUS] status=... yaw_validated=... (~1Hz, T5) : met a jour
        le label. Meme mecanisme que _on_beacon_status pour [BEACON]."""
        if status in ("DOCKED", "CAL_DONE", "RANGE_ONLY", "ALIGN_DONE", "APPROACH_DONE"):
            color = COL_OK
        elif status in ("ABORTED", "ERROR", "CAL_FAILED", "CAL_INCONCLUSIVE", "NO_BEACON",
                        "NOT_CONVERGED", "GIMBAL_ALIGN_FAILED", "NOT_ALIGNED",
                        # 2026-07-30 : CHASSIS_ALIGN_FAILED et SEQUENCE_TIMEOUT
                        # sont emis par beacon_docking.py depuis le 2026-07-28
                        # mais n'ont jamais figure ici -- ils tombaient donc en
                        # gris "inconnu" au lieu de rouge. CHASSIS_ALIGN_FAILED
                        # est precisement le statut de la cascade du 2026-07-29.
                        "CHASSIS_ALIGN_FAILED", "SEQUENCE_TIMEOUT",
                        # nouveau statut de la boucle d'alignement verifiee
                        "ALIGN_NOT_CONVERGED"):
            color = COL_KO
        elif status in ("DOCKING", "CALIBRATING", "CAL_STEP1_DONE",
                        # ni succes franc ni echec : yaw_rel a converge mais la
                        # mesure de controle n'a pas pu etre faite (2026-07-30)
                        "ALIGN_DONE_UNVERIFIED"):
            color = COL_ALIGN
        else:
            color = FG_DIM
        suffix = " [YAW OK]" if yaw_validated else " [YAW NON VALIDE]"
        self._dock_status_lbl.config(text=f"DOCK: {status}{suffix}", fg=color)

    def _reset_beacon_ui(self):
        """Reset visuel complet voyant/minimap -- appele aux memes points que le
        reset LOCK (entree MANUEL, sortie AUTO, Kill)."""
        self._beacon_detected = False
        self._beacon_dot.itemconfig(self._beacon_dot_id, fill=COL_KO)
        self._beacon_status_lbl.config(text="BEACON: LOST", fg=COL_KO)
        self._minimap.itemconfig(self._minimap_dot, state="hidden")

    def _on_beacon_status(self, status, yaw_err_str, pitch_err_str):
        """Parse [BEACON] status=DETECTED/LOST (~5Hz, cf. rm_cam_beacon.py) : met a
        jour voyant et minimap."""
        detected = (status == "DETECTED")
        self._beacon_detected = detected

        if detected:
            self._beacon_dot.itemconfig(self._beacon_dot_id, fill=COL_OK)
            self._beacon_status_lbl.config(text="BEACON: DETECTED", fg=COL_OK)
            if yaw_err_str is not None and pitch_err_str is not None:
                yaw_err, pitch_err = float(yaw_err_str), float(pitch_err_str)
                ox = 50 + int((yaw_err / 45.0) * 50)
                oy = 50 + int((pitch_err / 45.0) * 50)
                ox = max(3, min(97, ox))
                oy = max(3, min(97, oy))
                centered = abs(yaw_err) < 3.0 and abs(pitch_err) < 3.0
                self._minimap.coords(self._minimap_dot, ox - 4, oy - 4, ox + 4, oy + 4)
                self._minimap.itemconfig(self._minimap_dot, fill=(COL_OK if centered else COL_APPROACH),
                                         state="normal")
        else:
            self._beacon_dot.itemconfig(self._beacon_dot_id, fill=COL_KO)
            self._beacon_status_lbl.config(text="BEACON: LOST", fg=COL_KO)
            self._minimap.itemconfig(self._minimap_dot, state="hidden")

    def _toggle_camera_preview(self):
        # OFF par defaut (2026-07-23) : coupe l'abonnement /camera/color/image_raw
        # cote helper (pas juste l'affichage) -- gagne en fluidite clavier ET
        # libere de la bande passante reseau sur un topic deja identifie comme
        # goulot (Perplexity 11).
        self._camera_enabled = not self._camera_enabled
        if self._camera_enabled:
            self._cam_btn.config(text="APERCU CAM : ON", bg=ACCENT, fg=BG)
            self._send_to_helper("CAM ON")
            self.after(0, self._log, "> Apercu camera ON")
        else:
            self._cam_btn.config(text="APERCU CAM : OFF", bg=BG3, fg=FG_DIM)
            self._send_to_helper("CAM OFF")
            self.cam_canvas.delete("all")
            self._cam_txt = self.cam_canvas.create_text(160, 90, text="apercu desactive", fill=FG_DIM)
            self.cam_img = None
            self.after(0, self._log, "> Apercu camera OFF (fluidite + bande passante)")

    # ── tilt roues (mode MANUEL uniquement) ──────────────────────────────────

    def _on_tilt_press(self, cmd):
        if self.gui_mode == "MANUAL":
            self._send_to_helper(f"WHEELS {cmd}")

    def _on_tilt_release(self):
        self._send_to_helper("WHEELS STOP")

    def _make_key_btn(self, parent, label, row, col):
        btn = tk.Label(parent, text=label, width=3, height=1,
                       bg=BG3, fg=FG, font=FONT_MONO, bd=1, relief="raised")
        btn.grid(row=row, column=col, padx=3, pady=3)
        return btn

    def _update_chassis_visual(self):
        for key, btn in self._chassis_btns.items():
            if key in self._keys_down:
                btn.config(bg=COL_KEY_ACT, fg=BG, relief="sunken")
            else:
                btn.config(bg=BG3, fg=FG, relief="raised")

    def _update_gimbal_visual(self):
        for action, btn in self._gimbal_btns.items():
            if action in self._gim_down:
                btn.config(bg=COL_KEY_ACT, fg=BG, relief="sunken")
            else:
                btn.config(bg=BG3, fg=FG, relief="raised")

    def _on_chassis_btn_press(self, key):
        if self.gui_mode != "MANUAL":
            return
        if key not in self._keys_down:
            self._keys_down.add(key)
            self._send_velocity()
            self._update_chassis_visual()

    def _on_chassis_btn_release(self, key):
        if self.gui_mode != "MANUAL":
            return
        self._keys_down.discard(key)
        if self._keys_down:
            self._send_velocity()
        else:
            self._send_to_helper("STOP")
        self._update_chassis_visual()

    def _on_gimbal_btn_press(self, action):
        if self.gui_mode != "MANUAL":
            return
        if action == "stop":
            self._gim_down.clear()
        elif action not in self._gim_down:
            self._gim_down.add(action)
        self._send_gimbal()
        self._update_gimbal_visual()

    def _on_gimbal_btn_release(self, action):
        if self.gui_mode != "MANUAL":
            return
        if action != "stop":
            self._gim_down.discard(action)
            self._send_gimbal()
        self._update_gimbal_visual()

    # ── logs ─────────────────────────────────────────────────────────────────

    # Nom des onglets pour le prefixe disque. Aligne sur `tab_labels` (~ligne 511)
    # mais volontairement court : ces prefixes sont faits pour etre grepes.
    # ---------------------------------------------------------------- Pi state
    # Sonde SSH periodique (2026-08-04). Trois precautions, chacune pour une
    # panne deja vue sur ce projet :
    #   - THREAD separe : le 2026-08-04 le Pi a repondu au ping tout en laissant
    #     SSH pendre indefiniment. Une lecture synchrone aurait fige la GUI.
    #   - timeout DUR sur ssh (BatchMode + ConnectTimeout) : sans lui la commande
    #     attend un mot de passe qui ne viendra jamais et ne rend jamais la main.
    #   - periode LENTE (20 s) : c'est une info de contexte, pas une telemetrie
    #     temps reel ; une sonde rapide ajouterait de la charge SSH a un Pi qu'on
    #     surveille precisement parce qu'il sature.
    PI_PROBE_PERIOD_MS = 20000

    def _pi_state_tick(self):
        """Relance la sonde puis se re-arme. Ne bloque jamais le thread GUI."""
        threading.Thread(target=self._pi_state_probe, daemon=True).start()
        self.after(self.PI_PROBE_PERIOD_MS, self._pi_state_tick)

    def _pi_state_probe(self):
        """Lit temperature / charge / RAM / frequence du Pi par SSH.

        Tout est lu depuis /sys et /proc : disponible sur Ubuntu, contrairement
        a `vcgencmd` qui n'existe que sous Raspberry Pi OS (verifie 2026-08-04).
        """
        cmd = ("cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null; echo '|';"
               "cut -d' ' -f1 /proc/loadavg; echo '|';"
               "free -m | awk '/Mem:/{print $3\" \"$2}'; echo '|';"
               "cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null")
        try:
            out = subprocess.run(
                ["ssh", *SSH_OPTS, "-o", "BatchMode=yes", "-o", "ConnectTimeout=5", PI, cmd],
                capture_output=True, text=True, timeout=12).stdout
            parts = [p.strip() for p in out.split("|")]
            temp = f"{int(parts[0]) / 1000:.0f}C" if parts[0].isdigit() else "--"
            load = parts[1] or "--"
            used, total = (parts[2].split() + ["", ""])[:2]
            ram = f"{int(used) * 100 // int(total)}%" if used.isdigit() and total.isdigit() else "--"
            freq = f"{int(parts[3]) // 1000}MHz" if len(parts) > 3 and parts[3].isdigit() else ""
            txt = f"temp {temp}  load {load}  ram {ram}  {freq}".rstrip()
            # Seuils Pi 4B : throttling thermique a 80C, on alerte avant.
            t = int(parts[0]) / 1000 if parts[0].isdigit() else 0
            col = COL_KO if t >= 75 else (ACCENT if t >= 65 else FG)
        except Exception:
            txt, col = "unreachable", FG_DIM
        self.after(0, lambda: self.pi_lbl.config(text=txt, fg=col))

    _LOG_TAGS = ["T1", "T2", "T3", "T4", "T5", "T6"]

    def _log_to_disk(self, msg, tab):
        """Ecrit une ligne dans le log de session (2026-07-31).

        Un seul fichier par lancement du launcher, sous `logs/`, horodate au
        demarrage : `logs/session-YYYY-MM-DD-HH-MM-SS.log`. Chaque ligne porte
        l'heure et l'onglet d'origine (`T1`..`T5`, ou `--` pour un evenement
        global diffuse partout), pour pouvoir grep un terminal precis apres coup
        sans avoir a rejouer la session.

        Best-effort par construction : toute erreur d'ecriture (disque plein,
        permission, chemin disparu) est avalee. Un log qui ne s'ecrit pas est un
        desagrement ; une GUI de pilotage qui tombe pendant que le robot roule
        n'en est pas un. Meme raisonnement que les `except Exception` deja en
        place autour des appels subprocess/SSH de ce fichier.
        """
        try:
            if self._log_fh is None:
                return
            tag = "--" if tab is None else self._LOG_TAGS[tab] if tab < len(self._LOG_TAGS) else f"T{tab+1}"
            self._log_fh.write(f"{time.strftime('%H:%M:%S')} [{tag}] {msg}\n")
            self._log_fh.flush()   # flush a chaque ligne : un crash ne doit pas
                                   # emporter le buffer, c'est justement dans ce
                                   # cas qu'on relira le fichier
        except Exception:
            pass

    def _open_session_log(self):
        """Ouvre le fichier de log de session. Appele une fois au demarrage.
        En cas d'echec, `_log_fh` reste None et `_log_to_disk` devient un no-op
        silencieux -- le launcher fonctionne exactement comme avant."""
        self._log_fh = None
        try:
            log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(log_dir, time.strftime("session-%Y-%m-%d-%H-%M-%S.log"))
            self._log_fh = open(path, "a", encoding="utf-8")
            self._log_fh.write(f"# Carolus launcher session log — started {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            self._log_fh.flush()
            self._session_log_path = path
        except Exception as e:
            # Echec best-effort, MAIS pas silencieux (2026-07-31). Un log qui ne
            # s'ecrit pas sans le dire est pire que pas de log du tout : on croit
            # avoir les donnees et on ne les a pas. C'est exactement le mode de
            # defaillance releve toute la journee (BUG-087 : solveur non converge
            # publie comme valide). stderr et pas la GUI, parce qu'a ce stade de
            # __init__ les widgets de log n'existent pas encore.
            self._log_fh = None
            self._session_log_path = None
            print(f"[LAUNCHER] session log unavailable ({e}) — "
                  f"logs will stay in-memory only", file=sys.stderr)

    def _log(self, msg, tab=None):
        # tab=None -> diffuse le message (evenement global) dans les 4 onglets ;
        # tab=i -> ecrit uniquement dans l'onglet du terminal Ti+1 concerne.

        # Persistance disque (2026-07-31). Avant ca, les logs ne vivaient QUE
        # dans les widgets tkinter : fermer le launcher les perdait, et chaque
        # onglet est de toute facon tronque a 300 lignes (ci-dessous). Le cout
        # concret constate le 2026-07-31 : la question "LOCK tourne-t-il encore
        # pendant un docking ?" (point 4 de 21-points-a-creuser) est restee sans
        # reponse alors que la reponse etait dans les logs T2 d'un run deja
        # effectue -- il suffisait de les avoir gardes. Un seul fichier par
        # session, prefixe par l'onglet, ecrit best-effort : une erreur d'ecriture
        # ne doit jamais faire tomber la GUI.
        self._log_to_disk(msg, tab)

        boxes = self.log_boxes if tab is None else [self.log_boxes[tab]]
        for box in boxes:
            # Text est en state="disabled" (BUG-061) : repasser en "normal" le temps
            # de l'ecriture programmatique, sinon insert()/delete() sont refuses.
            box.config(state="normal")
            box.insert("end", msg + "\n")
            n = int(box.index("end-1c").split(".")[0])
            if n > 300:
                box.delete("1.0", f"{n-300}.0")
            box.see("end")
            box.config(state="disabled")

    def _block_edit(self, event):
        # autorise copie (Ctrl+C/A) et navigation, bloque toute frappe modifiant le texte
        if event.state & 0x4 and event.keysym.lower() in ("c", "a"):
            return
        if event.keysym in ("Left", "Right", "Up", "Down", "Home", "End", "Prior", "Next",
                            "Shift_L", "Shift_R", "Control_L", "Control_R"):
            return
        # Pilotage ZQSD et numpad : laisser remonter au root meme si un onglet de logs a le focus
        if event.keysym.lower() in ("z", "q", "s", "d") or event.keysym.startswith("KP_"):
            return
        return "break"

    def _select_all_logs(self, event, box):
        box.tag_add("sel", "1.0", "end-1c")
        return "break"

    def _copy_logs(self):
        idx = self.log_nb.index("current")
        txt = self.log_boxes[idx].get("1.0", "end-1c")
        self.clipboard_clear()
        self.clipboard_append(txt)
        self._log("> Logs copies dans le presse-papier", idx)

    # ── statut / etat des lignes ──────────────────────────────────────────────

    def _set_status(self, i, text):
        self.after(0, lambda: self.rows[i][1].config(text=text))

    def _reset_row(self, i, unlocked):
        self.after(0, lambda: self.rows[i][0].config(state="normal" if unlocked else "disabled"))
        self.after(0, lambda: self.rows[i][1].config(text=S_IDLE if unlocked else S_LOCKED))

    def _close_terminal(self, i):
        if self.procs[i] is not None:
            try:
                self.procs[i].terminate()
                self.procs[i].wait(timeout=2)   # libere le zombie OS
            except Exception:
                pass
            self.procs[i] = None

    # ── dashboard ─────────────────────────────────────────────────────────────

    def _set_robot_state(self, state):
        if state == self.last_state:
            return
        self.last_state = state
        col = {"SEARCH": COL_SEARCH, "ALIGN": COL_ALIGN,
               "APPROACH": COL_APPROACH, "STOP": COL_STOP,
               "MANUEL": COL_MANUAL}.get(state, COL_IDLE)
        self.state_dot.itemconfig(self._dot, fill=col)
        self.state_lbl.config(text=state)
        if state == "SEARCH":
            self.depth_lbl.config(text="")

    def _set_depth(self, depth):
        self.depth_lbl.config(text=f"depth = {depth:.2f} m")

    def _set_battery(self, pct):
        pct = max(0, min(100, pct))
        col = COL_STOP if pct > 40 else (COL_APPROACH if pct >= 15 else COL_KO)
        self.batt_canvas.coords(self._batt_rect, 0, 0, int(120 * pct / 100), 16)
        self.batt_canvas.itemconfig(self._batt_rect, fill=col)
        self.batt_lbl.config(text=f"{pct}%")

    def _reset_dashboard(self):
        self.last_state = None
        self.state_dot.itemconfig(self._dot, fill=COL_IDLE)
        self.state_lbl.config(text="---")
        self.depth_lbl.config(text="")
        self.batt_canvas.coords(self._batt_rect, 0, 0, 0, 16)
        self.batt_lbl.config(text="N/A")
        self.bat_detail_lbl.config(text="temp: N/A   courant: N/A   adc: N/A", fg=FG)
        self.atti_lbl.config(text="pitch: N/A   roll: N/A")
        self.vel_lbl.config(text="vx: N/A   vy: N/A")
        self.esc_lbl.config(text="W1:---  W2:---  W3:---  W4:---")
        self.status_lbl.config(text="OK", fg=ACCENT)
        self.tof_lbl.config(text="N/A", fg=FG)
        self.cam_canvas.delete("all")
        self.cam_canvas.create_text(160, 90, text="en attente...", fill=FG_DIM)
        self.cam_img = None
        self._last_robot_pos = (0.0, 0.0)
        if self._map_editor:
            try:
                self._map_editor.reset_overlay()
            except Exception:
                pass
        self._live_map.reset_overlay()
        try:
            os.remove(CAM_PNG)
        except OSError:
            pass

    # ── connexion Pi (thread de fond) ─────────────────────────────────────────

    def _conn_monitor(self):
        while not self._stop_monitor:
            ok = subprocess.run(["ping", "-c", "1", "-W", "1", PI_HOST],
                                capture_output=True).returncode == 0
            self.after(0, self._set_conn, ok)
            time.sleep(5)

    def _set_conn(self, ok):
        self.conn_dot.itemconfig(self._conn_oval, fill=COL_OK if ok else COL_KO)
        self.conn_lbl.config(text=f"OK ({PI_HOST})" if ok else f"injoignable ({PI_HOST})",
                             fg=ACCENT if ok else COL_KO)

    # ── verifications avec timeout et annulation ──────────────────────────────

    def _wait_for_roscore(self, idx, timeout=60):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._launch_cancelled[idx]:
                return False
            r = subprocess.run(
                ["ssh"] + SSH_OPTS + ["-o", "ConnectTimeout=3",
                 PI, "bash -c '</dev/tcp/localhost/11311'"],
                capture_output=True)
            if r.returncode == 0:
                return True
            time.sleep(1)
        return False

    def _wait_for_camera(self, idx, timeout=60):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._launch_cancelled[idx]:
                return False
            r = subprocess.run(
                ["ssh"] + SSH_OPTS + ["-o", "ConnectTimeout=3", PI,
                 "source /opt/ros/noetic/setup.bash && "
                 "export ROS_MASTER_URI=http://localhost:11311 && "
                 "rostopic list 2>/dev/null | grep -q /camera/color/image_raw"],
                capture_output=True)
            if r.returncode == 0:
                return True
            time.sleep(1)
        return False

    # ── flux integre : thread → queue → main thread (anti-freeze) ────────────

    def _integrated_reader(self, proc, tag):
        try:
            for line in proc.stdout:
                self._log_queue.put((tag, line.rstrip()))
        except Exception:
            pass

    def _flush_log_queue(self):
        batch = []
        try:
            while len(batch) < 50:
                batch.append(self._log_queue.get_nowait())
        except queue.Empty:
            pass
        for tag, line in batch:
            if line:
                self._on_log_line(tag, line)
        self.after(50, self._flush_log_queue)

    def _on_log_line(self, tag, line):
        if not line:
            return
        idx = int(tag[1:]) - 1   # "T1".."T4" -> 0..3, onglet dedie -> plus besoin du prefixe [Tn]
        # Lignes de telemetrie haute frequence : dashboard uniquement, log texte epargne
        if not any(t in line for t in _LOG_SUPPRESS):
            self._log(line, idx)
        # parsing dashboard (pertinent pour rm_cam_beacon ; inoffensif pour les autres)
        if "Reached target -> STOP" in line:
            self._set_robot_state("STOP")
        elif "-> SEARCH" in line or "State: SEARCH" in line:
            self._set_robot_state("SEARCH")
        elif "State: ALIGN" in line or "[ALIGN]" in line:
            self._set_robot_state("ALIGN")
        elif "State: APPROACH" in line or "[APPROACH]" in line or "-> APPROACH" in line:
            self._set_robot_state("APPROACH")
        if "depth=" in line:
            m = RE_DEPTH.search(line)
            if m:
                self._set_depth(float(m.group(1)))
        if "[BAT]" in line:
            b = RE_BAT.search(line)
            if b:
                pct  = int(b.group(1))
                temp = float(b.group(2))
                cur  = int(b.group(3))
                adc  = int(b.group(4))
                self._set_battery(pct)
                temp_col = COL_KO if temp > 45 else (COL_APPROACH if temp > 38 else FG)
                cur_str  = f"{cur:+d} mA"
                self.bat_detail_lbl.config(
                    text=f"temp:{temp:.1f}°C   cur:{cur_str}   adc:{adc}",
                    fg=temp_col)
        if "[ESC]" in line:
            m = RE_ESC.search(line)
            if m:
                self.esc_lbl.config(
                    text=f"W1:{m.group(1):>4}  W2:{m.group(2):>4}  W3:{m.group(3):>4}  W4:{m.group(4):>4}")
        if "[ATTI]" in line:
            m = RE_ATTI.search(line)
            if m:
                self._last_robot_yaw = float(m.group(1))
                self.atti_lbl.config(
                    text=f"yaw:{float(m.group(1)):+.1f}°  pitch:{float(m.group(2)):+.1f}°  roll:{float(m.group(3)):+.1f}°")
        if "[POS]" in line:
            m = RE_POS.search(line)
            if m:
                x, y = float(m.group(1)), float(m.group(2))
                self._last_robot_pos = (x, y)
                if self._map_editor:
                    try:
                        self._map_editor.update_robot(x, y, self._last_robot_yaw)
                    except Exception:
                        pass
                self._live_map.update_robot(x, y, self._last_robot_yaw)
        if "[VEL]" in line:
            m = RE_VEL.search(line)
            if m:
                self.vel_lbl.config(
                    text=f"vx:{float(m.group(1)):+.3f} m/s   vy:{float(m.group(2)):+.3f} m/s")
        if "[STATUS]" in line:
            m = RE_STATUS.search(line)
            if m:
                flags = {
                    'pickup':  int(m.group(1)),
                    'slip':    int(m.group(2)),
                    'roll':    int(m.group(3)),
                    'slope':   int(m.group(4)),
                    'impact':  int(m.group(5)),
                }
                active = [k.upper() for k, v in flags.items() if v]
                if active:
                    self.status_lbl.config(text=" | ".join(active), fg=COL_KO)
                else:
                    self.status_lbl.config(text="OK", fg=ACCENT)
        if "[TOF]" in line:
            m = RE_TOF.search(line)
            if m:
                tof_cm = float(m.group(1))
                col = COL_KO if tof_cm < 60 else (COL_APPROACH if tof_cm < 100 else FG)
                self.tof_lbl.config(text=f"{tof_cm:.0f} cm", fg=col)
        if "[OBSTACLE]" in line:
            m = RE_OBSTACLE.search(line)
            if m:
                self.tof_lbl.config(text=f"⚠ {m.group(1)}", fg=COL_KO)
        if "[BEACON]" in line:
            m = RE_BEACON.search(line)
            if m:
                self._on_beacon_status(m.group(1), m.group(2), m.group(3))
        if "[BEACONPOS]" in line:
            m = RE_BEACONPOS.search(line)
            if m:
                rel_x  = float(m.group(1))
                rel_z  = float(m.group(2))
                byaw   = float(m.group(3)) if m.group(3) else 0.0
                yaw_r = math.radians(self._last_robot_yaw)
                rx, ry = self._last_robot_pos
                # Carolus publie la pose camera-dans-repère-balise (Z négatif = balise devant).
                # depth = -rel_z (positif = balise devant), lateral = rel_x (positif = droite).
                # Yaw DJI : CW positif depuis nord.
                wx = rx - rel_z * math.cos(yaw_r) - rel_x * math.sin(yaw_r)
                wy = ry - rel_z * math.sin(yaw_r) + rel_x * math.cos(yaw_r)
                face_deg = math.degrees(math.atan2(rx - wx, ry - wy))
                if self._map_editor:
                    try:
                        self._map_editor.update_beacon(wx, wy)
                        self._map_editor.add_auto_beacon(wx, wy, face_deg)
                    except Exception:
                        pass
                self._live_map.add_auto_beacon(wx, wy, face_deg)
                self._last_beacon_ts = time.time()
        if "[DOCKSTATUS]" in line:
            self._t5_dock_ready = True   # premiere ligne vue -> T5 a fini son __init__
            m = RE_DOCKSTATUS.search(line)
            if m:
                self._on_dock_status(m.group(1), m.group(2) == "True")

    def _check_beacon_freshness(self):
        """Cache le marqueur balise sur la live map si aucune detection recente
        (evite de laisser un point jaune affiche alors que le robot ne voit plus la balise)."""
        if self._last_beacon_ts and (time.time() - self._last_beacon_ts > BEACON_FRESH_S):
            self._live_map.hide_beacon()
            self._last_beacon_ts = 0.0
        self.after(300, self._check_beacon_freshness)

    # ── flux video (boucle main thread) ───────────────────────────────────────

    def _refresh_cam(self):
        # Rien a faire si l'apercu est desactive (2026-07-23) : le helper n'ecrit
        # plus de PNG (desabonne de /camera/color/image_raw), et on ne veut pas
        # decoder/redessiner un fichier perime -- gagne en fluidite clavier.
        if not self._camera_enabled:
            self.after(200, self._refresh_cam)   # cadence basse : juste pour re-detecter une reactivation
            return
        try:
            if os.path.exists(CAM_PNG):
                img = tk.PhotoImage(file=CAM_PNG)
                self.cam_canvas.delete("all")
                self.cam_canvas.create_image(160, 90, image=img, anchor="center")
                self.cam_img = img
                self._cam_txt = None
        except Exception:
            pass
        # Rafraichissement apercu GUI (2026-07-22 : 500->50 ms, soit 2->20 Hz).
        # Cosmetique : lit la vignette PNG ecrite par cam_view_helper. N'affecte pas
        # le flux Carolus. Aligne sur la cadence d'ecriture du helper (THROTTLE_S=0.05).
        self.after(50, self._refresh_cam)

    def _start_cam_helper(self):
        self._stop_cam_helper()
        self.cam_proc = subprocess.Popen(
            ["bash", "-c",
             "source /opt/ros/noetic/setup.bash && "
             f"source {WS}/devel/setup.bash && "
             "export ROS_MASTER_URI=http://192.168.0.103:11311 && "
             "export ROS_IP=192.168.0.100 && "
             f"python3 -u {HELPER} {CAM_PNG}"],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            text=True, bufsize=1)
        self.after(0, self._log, "> Helper video lance (stdin ouvert)", 1)
        # Synchronise l'etat CAM ON/OFF du GUI vers le nouveau process helper (qui
        # demarre desabonne par defaut) -- couvre le cas ou l'utilisateur avait
        # active l'apercu avant un Kill/relance de T2.
        self._send_to_helper("CAM ON" if self._camera_enabled else "CAM OFF")

    def _stop_cam_helper(self):
        if self.cam_proc is not None:
            try:
                self.cam_proc.terminate()
            except Exception:
                pass
            self.cam_proc = None
        local_kill("cam_view_helper.py")

    # ── commandes de lancement (toutes integrees depuis le 2026-07-20) ────────

    def _cmd_integrated(self, i):
        if i == 0:
            return ["ssh", "-tt"] + SSH_OPTS + ["-o", "ConnectTimeout=5", PI,
                    "sudo ip link set eth1 up; "
                    "sudo ip addr add 192.168.42.3/24 dev eth1 2>/dev/null || true; "
                    "source /opt/ros/noetic/setup.bash; "
                    "stdbuf -oL -eL roscore 2>&1"]
        if i == 1:
            return ["ssh", "-tt"] + SSH_OPTS + ["-o", "ConnectTimeout=5", PI,
                    "source /opt/ros/noetic/setup.bash; "
                    "export ROS_MASTER_URI=http://localhost:11311; "
                    "export ROS_IP=192.168.0.103; "
                    f"stdbuf -oL -eL python3 -u {BEACON_PI} 2>&1"]
        if i == 3:
            return ["ssh", "-tt"] + SSH_OPTS + ["-o", "ConnectTimeout=5", PI,
                    "source /opt/ros/noetic/setup.bash; "
                    "export ROS_MASTER_URI=http://localhost:11311; "
                    "export ROS_IP=192.168.0.103; "
                    f"stdbuf -oL -eL python3 -u {TF_BROADCASTER_PI} 2>&1"]
        if i == 4:
            # Tourne sur le PC labo (pas le Pi) : pas de connexion SDK propre,
            # commande via /carolus/cmd_vel deja relaye par rm_cam_beacon.py --
            # meme raisonnement que T3 (roslaunch carolus_node, aussi sur le PC).
            return ["bash", "-c",
                    "source /opt/ros/noetic/setup.bash && "
                    "export ROS_MASTER_URI=http://192.168.0.103:11311 && "
                    "export ROS_IP=192.168.0.100 && "
                    f"stdbuf -oL -eL python3 -u {DOCKING_SCRIPT} 2>&1"]
        if i == 5:
            # MINS tourne SUR LE PI (contrairement a T3/T5) : c'est la machine
            # qui porte les capteurs, et la seule sous Ubuntu 20.04, la cible
            # officielle de ROS Noetic. Lance sa propre simulation pour l'instant
            # -- l'etape suivante est de le brancher sur nos vrais topics.
            return ["ssh", "-tt"] + SSH_OPTS + ["-o", "ConnectTimeout=5", PI,
                    "source /opt/ros/noetic/setup.bash; "
                    "export ROS_MASTER_URI=http://localhost:11311; "
                    "export ROS_IP=192.168.0.103; "
                    f"cd {MINS_WS_PI} && source devel/setup.bash; "
                    f"stdbuf -oL -eL roslaunch mins {MINS_LAUNCH} 2>&1"]
        return ["bash", "-c",
                "source /opt/ros/noetic/setup.bash && "
                "export ROS_MASTER_URI=http://192.168.0.103:11311 && "
                "export ROS_IP=192.168.0.100 && "
                f"cd {WS} && source devel/setup.bash && "
                "stdbuf -oL -eL roslaunch carolus_node testcarolus.launch 2>&1"]

    # ── launch ─────────────────────────────────────────────────────────────────

    def _on_launch(self, i):
        self.rows[i][0].config(state="disabled")
        self._set_status(i, S_RUNNING)
        threading.Thread(target=self._run_launch, args=(i,), daemon=True).start()

    def _run_launch(self, i):
        tag = f"T{i+1}"
        # Tous les terminaux sont integres depuis le 2026-07-20 (onglet de logs dedie par terminal)
        self.after(0, self._log, "> Lancement...", i)

        # Garde-fou anti double-connexion SDK (2026-07-22, BUG-057) : avant de lancer
        # T2, tuer toute instance residuelle de rm_cam_beacon.py sur le Pi. Deux
        # instances = deux ep.initialize() -> commandes de mouvement bloquees. Motif
        # crochete [r]m_... pour que le shell distant ne se tue pas lui-meme.
        if i == 1:
            try:
                subprocess.run(
                    ["ssh"] + SSH_OPTS + ["-o", "ConnectTimeout=4", PI,
                     "ps -eo pid,args | awk '/[r]m_cam_beacon\\.py/{print $1}' | xargs -r kill -9"],
                    capture_output=True, timeout=8)
                self.after(0, self._log, "> (pre-kill) instances rm_cam_beacon.py residuelles nettoyees sur le Pi", i)
            except Exception:
                pass

        if i == 4:
            self._t5_dock_ready = False   # nouveau process : reinitialise l'attente

        self.procs[i] = subprocess.Popen(
            self._cmd_integrated(i),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        threading.Thread(target=self._integrated_reader,
                         args=(self.procs[i], tag), daemon=True).start()

        if i == 0:
            self.after(0, self._log, "> Attente roscore (port 11311)...", i)
            if not self._wait_for_roscore(i):
                self.after(0, self._log, "> Annule ou timeout — Kill pour reinitialiser", i)
                return
            self.after(0, self._log, "> OK - roscore pret", i)
        elif i == 1:
            self.after(0, self._log, "> Attente /camera/color/image_raw...", i)
            if not self._wait_for_camera(i):
                self.after(0, self._log, "> Annule ou timeout — Kill pour reinitialiser", i)
                return
            self.after(0, self._log, "> OK - Camera prete", i)
            self._start_cam_helper()
            # Mode MANUEL par defaut au demarrage de T2 (2026-07-21, demande utilisateur) —
            # remplace l'ancien auto-LOCATE (sweep automatique). LOCATE reste disponible
            # via le bouton LOCALISER si besoin, juste plus declenche automatiquement.
            # Passe par _ensure_manual_default (pas _toggle_mode direct) pour re-verifier
            # l'etat a l'instant du tir : evite qu'un KILL dans la fenetre de 500ms
            # (qui force AUTO) ne soit re-bascule en MANUEL contre une pile deja tuee.
            self.after(500, self._ensure_manual_default)
        elif i == 2:
            self.after(0, self._log, "> T3 lance - attends RPY dans les logs", i)
        elif i == 3:
            self.after(0, self._log, "> T4 lance - TF broadcaster actif (quaternion corrige, BUG-048)", i)
        elif i == 4:
            # Attente reelle avant de debloquer les boutons DOCK (2026-07-27,
            # BUG trouve en test : sans cette attente, START pouvait etre envoye
            # avant que l'abonnement ROS de beacon_docking.py a /carolus/dock
            # soit etabli -> commande silencieusement perdue, aucune erreur).
            # Meme logique que _wait_for_roscore/_wait_for_camera pour T1/T2 :
            # on attend un signe de vie reel du node (son 1er [DOCKSTATUS]),
            # pas juste que le process ait demarre.
            self.after(0, self._log, "> Attente que beacon_docking.py soit pret (1er DOCKSTATUS)...", i)
            deadline = time.time() + 15
            while time.time() < deadline and not self._t5_dock_ready:
                if self._launch_cancelled[i]:
                    self.after(0, self._log, "> Annule", i)
                    return
                time.sleep(0.2)
            if not self._t5_dock_ready:
                self.after(0, self._log, "> Timeout — T5 ne repond pas, Kill pour reinitialiser", i)
                return
            self.after(0, self._log, "> T5 lance - docking pret (attend /pose, /odom, /carolus/gimbal_yaw_rel)", i)

        self._set_status(i, S_OK)
        if i + 1 < len(self.rows):
            self._reset_row(i + 1, unlocked=True)

    # ── kill ─────────────────────────────────────────────────────────────────

    def _on_kill(self, i):
        self.after(0, self._log, "> Arret en cours...")
        threading.Thread(target=self._run_kill, args=(i,), daemon=True).start()

    def _run_kill(self, i):
        targets = [i] if i >= 0 else [0, 1, 2, 3, 4]
        # annule les wait_for_* en cours pour ces cibles
        for t in targets:
            self._launch_cancelled[t] = True
        time.sleep(0.1)   # laisse les threads voir le flag
        for t in sorted(targets, reverse=True):
            if t == 4:
                local_kill("beacon_docking.py")
                self._close_terminal(4)
                self._t5_dock_ready = False
                self.after(0, lambda: self._dock_status_lbl.config(text="DOCK: —", fg=FG_DIM))
            elif t == 3:
                ssh_kill("pkill -9 -f carolus_tf_broadcaster.py")
                self._close_terminal(3)
            elif t == 2:
                local_kill("carolus_astrobee")
                local_kill("roslaunch")
                ssh_kill("pkill -9 -f carolus_astrobee")
                self._close_terminal(2)
            elif t == 1:
                ssh_kill("pkill -9 -f rm_cam_beacon.py")
                self._close_terminal(1)
                self._stop_cam_helper()
                self.after(0, self._reset_dashboard)
                self.after(0, self._force_auto_mode)
            elif t == 0:
                ssh_kill("pkill -9 -f rm_cam_beacon.py; pkill -9 -f roscore; pkill -9 -f rosmaster; pkill -9 -f carolus_tf_broadcaster.py")
                local_kill("carolus_astrobee")
                local_kill("roslaunch")
                self._stop_cam_helper()
                self._close_terminal(0)
                self._close_terminal(1)
                self._close_terminal(2)
                self._close_terminal(3)
                self.after(0, self._reset_dashboard)
                self.after(0, self._force_auto_mode)

        time.sleep(1)
        # remet les flags a False pour permettre les prochains lancements
        for t in targets:
            self._launch_cancelled[t] = False
        killed_from = min(targets)
        self._reset_row(killed_from, unlocked=True)
        for j in range(killed_from + 1, len(self.rows)):
            self._reset_row(j, unlocked=False)
        self.after(0, self._log, "> OK - Arrete. Relance avec le bouton.")

    # ── mode AUTO / MANUEL ───────────────────────────────────────────────────

    def _ensure_manual_default(self):
        # Bascule differee vers MANUEL au demarrage de T2. Re-verifie a l'instant du
        # tir : (a) que T2 n'a pas ete tue entre-temps (procs[1] remis a None par
        # _close_terminal), (b) qu'on est bien encore en AUTO. Sinon no-op.
        if self.procs[1] is None:
            return
        if self.gui_mode != "MANUAL":
            self._toggle_mode()

    def _toggle_mode(self):
        if self.gui_mode == "AUTO":
            self.gui_mode = "MANUAL"
            self.focus_set()   # force le focus sur la fenetre root (ZQSD/numpad actifs meme si un onglet de logs est clique)
            self.mode_btn.config(text="MODE : MANUEL  (ZQSD actif)", bg=COL_APPROACH)
            # Lock balise remis a OFF a chaque ENTREE en MANUEL : garantit qu'une
            # session de test demarre toujours gimbal fixe, et neutralise un LOCK ON
            # qui aurait ete clique par erreur en AUTO/LOCATE (sans effet la-bas mais
            # qui aurait sinon persiste jusqu'ici).
            self._gimbal_lock_active = False
            self._lock_btn.config(text="LOCK : OFF", bg=BG3, fg=FG_DIM)
            self._reset_beacon_ui()
            self._send_to_helper("MODE MANUAL")
            self._send_to_helper("LOCK OFF")
            # dashboard : refleter le mode MANUEL sur le point d'etat (BUG-014)
            self.last_state = "MANUAL"
            self.state_dot.itemconfig(self._dot, fill=COL_MANUAL)
            self.state_lbl.config(text="MANUEL")
            self.depth_lbl.config(text="")
            self.after(0, self._log, "> Mode MANUEL active - ZQSD pour piloter")
        else:
            self.gui_mode = "AUTO"
            self._keys_down.clear()
            self._gim_down.clear()
            self._update_chassis_visual()
            self._update_gimbal_visual()
            self.mode_btn.config(text="MODE : AUTO", bg=COL_STOP)
            self._send_to_helper("STOP")
            self._send_to_helper("GIMBAL 0.0 0.0")
            self._send_to_helper("MODE AUTO")
            # Lock balise scope au mode MANUEL — reset a la sortie pour ne pas
            # laisser un etat "actif" trompeur au prochain passage en MANUEL.
            self._gimbal_lock_active = False
            self._lock_btn.config(text="LOCK : OFF", bg=BG3, fg=FG_DIM)
            self._send_to_helper("LOCK OFF")
            self._reset_beacon_ui()
            # dashboard : reset pour que la prochaine ligne T2 remette l'etat reel (BUG-014)
            self.last_state = None
            self.state_dot.itemconfig(self._dot, fill=COL_IDLE)
            self.state_lbl.config(text="--- (AUTO)")
            self.after(0, self._log, "> Mode AUTO active - grace period 5s")

    # ── clavier ZQSD ─────────────────────────────────────────────────────────

    def _bind_keys(self):
        self._bind_keys_to(self)

    def _bind_keys_to(self, window):
        """Bind chassis/gimbal handlers to any Tk window (root or Toplevel)."""
        # Reclame le focus clavier des qu'on survole la fenetre : necessaire car
        # T1/T3 ouvrent des gnome-terminal externes qui volent le focus WM, et
        # focus_set() n'est sinon appele qu'une fois (au bascule MODE MANUEL).
        window.bind("<Enter>", lambda e, w=window: w.focus_set())
        # BUG (2026-07-21) : si le focus quitte la fenetre pendant qu'une touche est
        # maintenue (clic ailleurs, live map, dialogue...), le KeyRelease correspondant
        # ne se delivre jamais -> la touche reste bloquee dans _keys_down/_gim_down
        # indefiniment -> le chassis/gimbal continue de recevoir la derniere vitesse
        # non nulle en boucle cote Pi (rotation/mouvement "tout seul"). Filet de
        # securite : a la perte de focus, on traite comme si toutes les touches
        # etaient relachees, quelle qu'en soit la cause.
        window.bind("<FocusOut>", self._on_focus_out)
        for key in ("z", "q", "s", "d", "Z", "Q", "S", "D"):
            window.bind(f"<KeyPress-{key}>",   self._on_key_press)
            window.bind(f"<KeyRelease-{key}>", self._on_key_release)
        # Numpad : NumLock ON → KP_8/2/4/6/5 ; NumLock OFF → KP_Up/Down/Left/Right/Begin
        for key in ("KP_8", "KP_Up",
                    "KP_2", "KP_Down",
                    "KP_4", "KP_Left",
                    "KP_6", "KP_Right",
                    "KP_5", "KP_Begin"):
            window.bind(f"<KeyPress-{key}>",   self._on_numpad_press)
            window.bind(f"<KeyRelease-{key}>", self._on_numpad_release)

    def _on_focus_out(self, event):
        if self.gui_mode != "MANUAL":
            return
        if not self._keys_down and not self._gim_down:
            return   # rien a nettoyer, evite un spam STOP a chaque changement de focus normal
        # <FocusOut> sur un toplevel se declenche AUSSI quand le focus passe a un
        # widget enfant (bouton, onglet de logs) ou a une autre fenetre de l'appli
        # (map editor). Dans ces cas le KeyRelease continue d'arriver (les bindings
        # sont partages via bindtags / re-bindes sur le map editor) : nettoyer serait
        # un STOP parasite qui couperait le pilotage a chaque clic. On differe donc le
        # test a after_idle et on ne nettoie que si le focus a REELLEMENT quitte l'appli
        # Tk (focus_get() renvoie None) — le seul cas ou un KeyRelease serait perdu.
        self.after_idle(self._focus_out_cleanup_if_left_app)

    def _focus_out_cleanup_if_left_app(self):
        if self.gui_mode != "MANUAL":
            return
        if not self._keys_down and not self._gim_down:
            return
        # On ne nettoie (STOP anti-touche-bloquee) QUE si le focus n'est PLUS sur une
        # fenetre qui possede nos bindings clavier (root ou map editor). Dans ces deux
        # fenetres le KeyRelease continue d'arriver (handlers partages) -> pas besoin de
        # STOP, et le faire couperait le pilotage a chaque clic interne. Partout ailleurs
        # (autre appli, minimisation, filedialog modale sans nos bindings, fenetre
        # etrangere) le KeyRelease serait perdu -> on nettoie. Robuste au cas filedialog
        # que la version precedente (focus_get() is None) laissait passer.
        try:
            focused = self.focus_get()
        except KeyError:
            focused = None   # focus sur une fenetre etrangere -> considere hors appli
        if focused is not None:
            top = focused.winfo_toplevel()
            bound_windows = [self]
            if self._map_editor is not None:
                bound_windows.append(self._map_editor)
            if top in bound_windows:
                return   # focus encore sur une fenetre qui gere le clavier -> rien a faire
        # Annule tout relachement differe (debounce X11) en attente : evite qu'un
        # _apply_key_release/_apply_numpad_release perime ne se declenche apres
        # ce nettoyage (inoffensif en soi -- discard()/STOP sont idempotents --
        # mais evite un after() qui traine inutilement).
        for aid in self._chassis_release_pending.values():
            self.after_cancel(aid)
        self._chassis_release_pending.clear()
        for aid in self._gim_release_pending.values():
            self.after_cancel(aid)
        self._gim_release_pending.clear()
        self._keys_down.clear()
        self._gim_down.clear()
        self._send_to_helper("STOP")
        self._send_to_helper("GIMBAL 0.0 0.0")
        self._update_chassis_visual()
        self._update_gimbal_visual()

    def _on_key_press(self, event):
        if self.gui_mode != "MANUAL":
            return
        # Ne pas interférer si le focus est sur un champ texte (ex: offset rotation map editor)
        if isinstance(event.widget, (tk.Entry, tk.Text, tk.Spinbox)):
            return
        k = event.keysym.lower()
        if k not in ("z", "q", "s", "d"):
            return
        # Auto-repeat X11 : ce KeyPress peut etre le "re-appui" synthetique qui
        # suit immediatement un KeyRelease pendant qu'on tient la touche -- on
        # annule alors le relachement differe au lieu de re-traiter une pression
        # (evite le dents-de-scie sur /carolus/cmd_vel, cf. KEY_REPEAT_DEBOUNCE_MS).
        pending = self._chassis_release_pending.pop(k, None)
        if pending is not None:
            self.after_cancel(pending)
            return
        if k not in self._keys_down:
            self._keys_down.add(k)
            self._send_velocity()
            self._update_chassis_visual()

    def _on_key_release(self, event):
        if self.gui_mode != "MANUAL":
            return
        k = event.keysym.lower()
        if k not in ("z", "q", "s", "d"):
            return
        # Relachement differe (debounce anti auto-repeat X11) : voir _on_key_press.
        self._chassis_release_pending[k] = self.after(
            KEY_REPEAT_DEBOUNCE_MS, self._apply_key_release, k)

    def _apply_key_release(self, k):
        self._chassis_release_pending.pop(k, None)
        self._keys_down.discard(k)
        if self._keys_down:
            self._send_velocity()
        else:
            self._send_to_helper("STOP")
        self._update_chassis_visual()

    def _send_velocity(self):
        vx = 0.0
        wz = 0.0
        if "z" in self._keys_down: vx += MANUAL_VX
        if "s" in self._keys_down: vx -= MANUAL_VX
        if "q" in self._keys_down: wz += MANUAL_WZ
        if "d" in self._keys_down: wz -= MANUAL_WZ
        self._send_to_helper(f"VX {vx:.2f} WZ {wz:.1f}")

    # normalise le keysym numpad quel que soit l'etat de NumLock
    _NUMPAD_MAP = {
        "KP_8": "up",   "KP_Up":    "up",
        "KP_2": "down", "KP_Down":  "down",
        "KP_4": "left", "KP_Left":  "left",
        "KP_6": "right","KP_Right": "right",
        "KP_5": "stop", "KP_Begin": "stop",
    }

    def _on_numpad_press(self, event):
        if self.gui_mode != "MANUAL":
            return
        if isinstance(event.widget, (tk.Entry, tk.Text, tk.Spinbox)):
            return
        action = self._NUMPAD_MAP.get(event.keysym)
        if not action:
            return
        # Debounce anti auto-repeat X11 -- meme logique que _on_key_press.
        pending = self._gim_release_pending.pop(action, None)
        if pending is not None:
            self.after_cancel(pending)
            return
        if action not in self._gim_down:
            self._gim_down.add(action)
            self._send_gimbal()
            self._update_gimbal_visual()

    def _on_numpad_release(self, event):
        if self.gui_mode != "MANUAL":
            return
        action = self._NUMPAD_MAP.get(event.keysym)
        if not action:
            return
        self._gim_release_pending[action] = self.after(
            KEY_REPEAT_DEBOUNCE_MS, self._apply_numpad_release, action)

    def _apply_numpad_release(self, action):
        self._gim_release_pending.pop(action, None)
        self._gim_down.discard(action)
        self._send_gimbal()
        self._update_gimbal_visual()

    def _send_gimbal(self):
        pitch = 0.0
        yaw   = 0.0
        if "up"    in self._gim_down: pitch += GIMBAL_PITCH
        if "down"  in self._gim_down: pitch -= GIMBAL_PITCH
        if "left"  in self._gim_down: yaw   += GIMBAL_YAW
        if "right" in self._gim_down: yaw   -= GIMBAL_YAW
        self._send_to_helper(f"GIMBAL {pitch:.1f} {yaw:.1f}")

    # ── envoi stdin vers cam_view_helper ─────────────────────────────────────

    def _send_to_helper(self, cmd):
        if self.cam_proc is not None and self.cam_proc.stdin is not None:
            try:
                self.cam_proc.stdin.write(cmd + "\n")
                self.cam_proc.stdin.flush()
            except Exception:
                pass

    def _force_auto_mode(self):
        self.gui_mode = "AUTO"
        self._keys_down.clear()
        self._gim_down.clear()
        self._update_chassis_visual()
        self._update_gimbal_visual()
        self.mode_btn.config(text="MODE : AUTO", bg=COL_STOP)
        self._gimbal_lock_active = False
        self._lock_btn.config(text="LOCK : OFF", bg=BG3, fg=FG_DIM)
        self._reset_beacon_ui()

    def _toggle_fullscreen(self, event=None):
        self._is_fullscreen = not self._is_fullscreen
        self.attributes("-fullscreen", self._is_fullscreen)

    def _exit_fullscreen(self, event=None):
        if self._is_fullscreen:
            self._is_fullscreen = False
            self.attributes("-fullscreen", False)

    def _on_close(self):
        self._stop_monitor = True
        # Nettoyage best-effort avant de fermer : sinon les noeuds lances par SSH
        # SURVIVENT sur le Pi (fermer la fenetre ne tue pas les process distants).
        # Un rm_cam_beacon.py orphelin garde la connexion SDK -> au prochain lancement,
        # la nouvelle instance entre en DOUBLE CONNEXION -> commandes de mouvement
        # bloquees silencieusement (le "mode manuel ne marche plus"). On tue donc les
        # process Pi + locaux + le helper video avant destroy().
        try:
            ssh_kill("pkill -9 -f rm_cam_beacon.py; pkill -9 -f carolus_tf_broadcaster.py")
        except Exception:
            pass
        try:
            self._stop_cam_helper()
        except Exception:
            pass
        local_kill("carolus_astrobee")
        local_kill("roslaunch")
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
