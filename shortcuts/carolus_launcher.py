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
from tkinter import ttk
import threading
import subprocess
import time




# ── Carte live embarquée ──────────────────────────────────────────────────────

PI       = "ubuntu@192.168.0.103"
PI_HOST  = "192.168.0.103"

# Paths derived from the script's own location, not hardcoded (2026-08-04).
# WS used to point at an absolute path under one machine's home directory: the
# launcher was therefore unusable anywhere else, and that path leaked a local
# username into a repository about to be made public. The launcher lives in
# <project>/shortcuts/, so the workspace is <project>/carolus_ws. Override with
# CAROLUS_WS if your layout differs.
HERE     = os.path.dirname(os.path.abspath(__file__))
_ROOT    = os.path.dirname(HERE)
WS       = os.environ.get("CAROLUS_WS", os.path.join(_ROOT, "carolus_ws"))
BEACON_PI = "/home/ubuntu/carolus_ws/src/robomaster_cam/scripts/rm_cam_beacon.py"
TF_BROADCASTER_PI = "/home/ubuntu/carolus_ws/src/carolus_node/scripts/carolus_tf_broadcaster.py"
# Docking (2026-07-27): runs on the lab PC, not the Pi -- it holds no SDK
# connection of its own (it commands through /carolus/cmd_vel, already relayed by
# rm_cam_beacon.py), so the "single SDK owner" constraint does not apply here.
DOCKING_SCRIPT = os.path.join(WS, "src/robomaster_cam/scripts/beacon_docking.py")
# MINS (2026-08-04): runs on the Pi, in its own workspace, deliberately kept
# separate from carolus_ws (a disposable sandbox while MINS is not integrated).
# Measured 2026-08-04: simulation.launch works and is accurate (RMSE 0.113 deg /
# 0.082 m) but runs at 0.3-0.4x real time UNDER THE SIMULATION's sensor load
# (2 cameras + LIDAR + IMU at 200 Hz), far heavier than ours. On our own recorded
# data, one camera and no LIDAR, it reached 0.7x -- still short of real time.
MINS_WS_PI    = "/home/ubuntu/mins_sandbox_ws"
MINS_LAUNCH   = "simulation.launch"
SSH_KEY  = os.path.expanduser("~/.ssh/carolus_nopass")
SSH_OPTS = ["-i", SSH_KEY, "-o", "IdentitiesOnly=yes", "-o", "StrictHostKeyChecking=no"]

HELPER  = os.path.join(HERE, "cam_view_helper.py")   # HERE defini plus haut
CAM_PNG = "/tmp/carolus_cam.png"
# Blob-detection view (2026-08-14): /postprocessed/image (Carolus's own
# black-and-white thresholded image with a coloured circle on each detected LED),
# same modern equivalent of the inherited PDF's rviz panel, second GUI thumbnail.
BLOBS_PNG = "/tmp/carolus_blobs.png"

# Base thumbnail size decoded from the PNG helper (must match cam_view_helper.py's
# SIZE). The displayed size can grow beyond this via an integer zoom -- see
# App._preview_zoom -- but this is what is actually decoded/transferred, and what
# every reset/placeholder falls back to at zoom=1.
PREVIEW_BASE_W, PREVIEW_BASE_H = 480, 270
PREVIEW_MAX_ZOOM = 3

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
COL_KEY_ACT  = "#d4a017"   # gold, for an active key (ZQSD / numpad)

# High-rate telemetry lines: dashboard only, kept out of the text log
_LOG_SUPPRESS = frozenset({"[ESC]", "[ATTI]", "[POS]", "[BAT]", "[VEL]", "[TOF]", "[BEACON]"})
# [BEACONPOS] deliberately excluded from the filter: the raw values are useful for diagnosing orientation

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

BEACON_FRESH_S = 1.5   # must match POSE_TIMEOUT_S in rm_cam_beacon.py

# Polices nommees SANS taille explicite (taille explicite -> crash X11 BadLength, cf journal 2026-06-23)
FONT      = ("TkDefaultFont",)
FONT_MONO = ("TkFixedFont",)

# X11 auto-repeat debounce (2026-07-23): while a key is held, X11 emits a
# KeyRelease immediately followed by a KeyPress. Without filtering, every one of
# those releases triggered a spurious STOP and re-send, producing the sawtooth
# observed on /carolus/cmd_vel (x: 0.2/0.0/0.2/0.0...).
KEY_REPEAT_DEBOUNCE_MS = 40

MANUAL_VX    = 0.20   # m/s avant/arriere en mode manuel
MANUAL_WZ    = 20.0   # deg/s rotation en mode manuel
GIMBAL_PITCH = 30.0   # deg/s pitch gimbal (numpad 8/2)
GIMBAL_YAW   = 40.0   # deg/s yaw gimbal (numpad 4/6)


# ── Kill distants (SSH) / locaux ────────────────────────────────────────────────

def _bracketed(pattern):
    """Turn 'rm_cam_beacon' into '[r]m_cam_beacon'.

    BUG-095 (2026-08-04): ssh runs the command inside a remote shell whose own
    /proc cmdline CONTAINS the pattern being matched. A plain
    'pkill -9 -f rm_cam_beacon' therefore matches that shell and kills it --
    possibly BEFORE reaching the real target, depending on PID scan order.
    Silent and non-deterministic. The bracket class '[r]m...' matches the
    target's cmdline but not the literal text in our own.

    Same defect fixed earlier the same day in measure_pi_pose.sh (BUG-090, then
    BUG-094). Third occurrence, three files -- hence this single helper, so the
    pattern is never hand-written again.
    """
    return "[" + pattern[0] + "]" + pattern[1:] if pattern else pattern


def ssh_kill(cmd):
    """Legacy path: still accepts a full shell command."""
    subprocess.run(
        ["ssh"] + SSH_OPTS + ["-o", "ConnectTimeout=3", PI, cmd],
        capture_output=True
    )


def remote_kill(*patterns, verify=True):
    """Kill processes on the Pi and VERIFY they are gone.

    Returns the patterns still alive afterwards (empty = everything died). We
    verify because BUG-095's failure mode was silent: the Kill button returned
    looking as though it had worked.
    """
    if not patterns:
        return []
    kills = "; ".join(f'pkill -9 -f "{_bracketed(p)}"' for p in patterns)
    check = "|".join(_bracketed(p) for p in patterns)
    cmd = f'{kills}; sleep 1; pgrep -f "{check}" >/dev/null && echo ALIVE || echo CLEAN'
    r = subprocess.run(
        ["ssh"] + SSH_OPTS + ["-o", "ConnectTimeout=3", PI, cmd],
        capture_output=True, text=True
    )
    if not verify:
        return []
    if "CLEAN" in (r.stdout or ""):
        return []
    # Second pass: identify precisely what survived.
    survivors = []
    for p in patterns:
        rr = subprocess.run(
            ["ssh"] + SSH_OPTS + ["-o", "ConnectTimeout=3", PI,
             f'pgrep -f "{_bracketed(p)}" >/dev/null && echo YES || echo NO'],
            capture_output=True, text=True
        )
        if "YES" in (rr.stdout or ""):
            survivors.append(p)
    return survivors


def local_kill(pattern):
    subprocess.run(["pkill", "-9", "-f", pattern], capture_output=True)


# ── Interface ────────────────────────────────────────────────────────────────

class App(tk.Tk):

    def __init__(self):
        super().__init__()
        # On-disk session log (2026-07-31) -- opened FIRST, before anything that
        # could call _log(). Same precaution as BUG-065/BUG-068 (a publisher or
        # timer used before it was created): _log_fh must exist before the first
        # _log_to_disk, otherwise an AttributeError fires inside a logging path,
        # i.e. at the worst possible moment.
        self._log_fh = None
        self._session_log_path = None
        # Last mtime of the preview PNG actually decoded (2026-08-10). None
        # forces the next tick to decode whatever is there -- set back to None
        # anywhere the canvas is cleared, or the preview would stay blank until
        # the helper happens to rewrite the file.
        self._cam_png_mtime = None
        # Same principle for the blob-detection thumbnail (2026-08-14).
        self._blobs_png_mtime = None
        self._open_session_log()

        self.title("Carolus Launcher")
        self.configure(bg=BG)
        # Resizable since 2026-08-10. It was locked, which is defensible for a
        # fixed layout -- but not when the window can be taller than the screen
        # it opens on: the operator then has no way to reach the bottom of it.
        self.resizable(True, True)
        self.procs      = [None, None, None, None, None, None]   # Popen T1..T6
        self.cam_proc   = None                 # Popen helper video (stdin=PIPE)
        self.cam_img    = None                 # reference PhotoImage (anti-GC)
        self.blobs_img  = None                 # reference PhotoImage blobs (anti-GC)
        # Responsive preview size (2026-08-14): the underlying PNG thumbnail stays a
        # fixed 480x270 (PREVIEW_BASE_W/H below) -- that resolution, and the network
        # traffic and decode cost that come with it, are unaffected by window size.
        # Only the DISPLAYED size grows, via an integer PhotoImage.zoom() on the
        # already-decoded image -- a local, cheap, Tk-only operation on the lab PC,
        # nothing that touches the Pi or the camera pipeline. Capped at 3x
        # (1440x810/panel) so a maximised window on a large screen cannot balloon
        # memory/CPU unboundedly. See _on_root_resize / _apply_preview_zoom.
        self._preview_zoom = 1
        self._resize_after_id = None
        self.last_state = None
        self.gui_mode   = "AUTO"               # "AUTO" ou "MANUAL"
        self._keys_down = set()
        self._gim_down  = set()
        self._chassis_release_pending = {}   # touche -> id after() en attente (debounce X11)
        self._gim_release_pending     = {}   # same, for the gimbal numpad
        self._stop_monitor   = False
        self._launch_cancelled = [False, False, False, False, False]   # annulation wait_for_*
        self._log_queue  = queue.Queue()       # lignes T2 integre -> main thread
        self._chassis_btns = {}                # label widgets touches ZQSD
        self._gimbal_btns  = {}                # label widgets numpad 8/4/5/6/2
        self._locate_active  = False   # mode LOCALISER actif
        # GUI camera preview: OFF by default (2026-07-23) -- both for smoother
        # piloting (a less loaded Tkinter mainloop) and to free network bandwidth:
        # cam_view_helper.py subscribes to the same /camera/color/image_raw topic
        # as Carolus, already identified as the bottleneck. One fewer ROS
        # subscriber on that topic means less duplicated traffic on the Pi <-> lab
        # PC link.
        self._camera_enabled = False
        # Blob-detection preview (2026-08-14): same off-by-default reasoning as
        # _camera_enabled -- one fewer subscriber on /postprocessed/image unless
        # someone actually wants to see it (tuning HSV/threshold values, or just
        # checking the LEDs are seen, per Hector's request).
        self._blobs_enabled = False
        self._gimbal_lock_active = False   # lock balise actif (centrage periodique, mode MANUEL uniquement)
        self._last_robot_pos = (0.0, 0.0)   # dernière pos sub_position (m)
        self._last_robot_yaw = 0.0          # dernière orientation robot (deg, sub_attitude)
        self._last_beacon_ts = 0.0          # horodatage dernier [BEACONPOS] recu
        self._t5_dock_ready = False         # 1er [DOCKSTATUS] vu -> abonnements T5 etablis
        self._build()
        self._bind_keys()
        self.bind("<Configure>", self._on_root_resize)
        self._refresh_cam()
        self._refresh_blobs()
        self.after(100, self._flush_log_queue)
        self.after(300, self._check_beacon_freshness)
        # Sonde Pi : premier tir a 2s (laisse la fenetre s'afficher d'abord),
        # then every PI_PROBE_PERIOD_MS. See _pi_state_probe.
        self.after(2000, self._pi_state_tick)
        self.after(self.CMD_HEARTBEAT_MS, self._cmd_heartbeat)
        threading.Thread(target=self._conn_monitor, daemon=True).start()
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        # Fullscreen (2026-07-23): F11 toggles, Escape exits.
        self._is_fullscreen = False
        self.bind("<F11>", self._toggle_fullscreen)
        self.bind("<Escape>", self._exit_fullscreen)

    # ── construction UI ──────────────────────────────────────────────────────

    def _build(self):
        # ── Two columns: left = controls, right = logs (2026-08-10).
        #
        # Everything used to be stacked in one column, which made the window
        # 1406 px tall -- 326 px MORE than the 1920x1080 primary screen, so the
        # log panel simply fell off the bottom, and `resizable(False, False)`
        # left no way to cope with it. Removing the live map freed the right
        # column; the logs are the tallest block, so they go there. Height comes
        # down to roughly what a 1080p screen can actually show.
        left_col = tk.Frame(self, bg=BG)
        left_col.pack(side="left", fill="y")
        right_col = tk.Frame(self, bg=BG)
        right_col.pack(side="left", fill="both", expand=True)
        # --- header robotique ---
        header = tk.Frame(left_col, bg=BG2)
        header.pack(fill="x")
        tk.Label(header, text="  >> CAROLUS LAUNCHER", bg=BG2, fg=ACCENT,
                 anchor="w", font=FONT).pack(fill="x", padx=12, pady=(8, 0))
        tk.Label(header, text="  RoboMaster S1 - Custom Raspberry Pi", bg=BG2, fg=FG,
                 anchor="w", font=FONT).pack(fill="x", padx=12)
        conn = tk.Frame(header, bg=BG2)
        conn.pack(fill="x", padx=12, pady=(2, 8))
        tk.Label(conn, text="Pi connection:", bg=BG2, fg=FG_DIM, font=FONT).pack(side="left")
        self.conn_dot = tk.Canvas(conn, width=14, height=14, bg=BG2, highlightthickness=0)
        self._conn_oval = self.conn_dot.create_oval(2, 2, 12, 12, fill=COL_IDLE, outline="")
        self.conn_dot.pack(side="left", padx=6)
        self.conn_lbl = tk.Label(conn, text="checking...", bg=BG2, fg=FG_DIM, font=FONT)
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
            # T6: independent of the rest (runs on the Pi, against its own local
            # roscore). Left enabled from the start -- it waits on no topic from
            # our pipeline while running its own simulation.
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
        self.mode_btn = tk.Button(ctrl, text="MODE: AUTO", bg=COL_STOP, fg=FG, relief="flat",
                                  activebackground=ACCENT, activeforeground=BG, font=FONT,
                                  command=self._toggle_mode)
        self.mode_btn.pack(side="left", fill="x", expand=True, padx=(4, 4))
        self._locate_btn = tk.Button(ctrl, text="LOCATE", bg=BG3, fg=FG_DIM, relief="flat",
                                     activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                                     command=self._toggle_locate)
        self._locate_btn.pack(side="left", fill="x", expand=True, padx=(4, 4))
        self._lock_btn = tk.Button(ctrl, text="LOCK: OFF", bg=BG3, fg=FG_DIM, relief="flat",
                                   activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                                   command=self._toggle_gimbal_lock)
        self._lock_btn.pack(side="left", fill="x", expand=True, padx=(4, 4))
        # LOCK re-centring period, live-configurable (2026-07-23): an input field
        # in SECONDS only, default 2 s. An invalid value is never blocking --
        # rm_cam_beacon.py falls back silently to 2.0 s (see
        # _gimbal_lock_period_cb), like an ordinary web form field.
        self._lock_period_entry = tk.Entry(ctrl, width=4, bg=BG3, fg=FG,
                                           insertbackground=FG, relief="flat", font=FONT_MONO,
                                           justify="center")
        self._lock_period_entry.insert(0, "3")
        self._lock_period_entry.bind("<Return>", self._on_lock_period_changed)
        self._lock_period_entry.pack(side="left", padx=(0, 4))
        tk.Label(ctrl, text="s", bg=BG, fg=FG_DIM, font=FONT).pack(side="left", padx=(0, 4))
        self._cam_btn = tk.Button(ctrl, text="CAM PREVIEW: OFF", bg=BG3, fg=FG_DIM, relief="flat",
                                  activebackground=ACCENT, activeforeground=BG, font=FONT,
                                  command=self._toggle_camera_preview)
        self._cam_btn.pack(side="left", fill="x", expand=True, padx=(4, 4))
        # Blob-detection view (2026-08-14): /postprocessed/image, Carolus's own
        # black-and-white + coloured-blob-markers image. Fixed width, not expand=True
        # like the row's other buttons -- this row is already dense (5 buttons + an
        # entry field), and this one is used far less often (tuning sessions, not
        # every launch).
        self._blobs_btn = tk.Button(ctrl, text="BLOB VIEW: OFF", bg=BG3, fg=FG_DIM, relief="flat",
                                    activebackground=ACCENT, activeforeground=BG, font=FONT,
                                    command=self._toggle_blob_preview)
        self._blobs_btn.pack(side="left", padx=(4, 4))

        # --- blocs de pilotage visuel ---
        self._build_ctrl_blocks(left_col)

        # --- dashboard (2 colonnes) ---
        dash = tk.Frame(left_col, bg=BG)
        dash.pack(fill="x", padx=12, pady=4)

        # Outer container for both status panels, stacked vertically -- reworked
        # 2026-08-14 to give Robot Status and Raspberry Pi Status each their own
        # clearly delimited panel instead of one undifferentiated block (the Pi
        # section already existed, added 2026-08-04, but lived silently nested at
        # the bottom of the robot panel with no visual boundary of its own).
        left = tk.Frame(dash, bg=BG)
        left.grid(row=0, column=0, sticky="nw", padx=(0, 8))

        robot_frame = tk.Frame(left, bg=BG2)
        robot_frame.pack(fill="x", ipadx=8, ipady=6)
        tk.Label(robot_frame, text="ROBOT STATUS", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w")
        srow = tk.Frame(robot_frame, bg=BG2)
        srow.pack(anchor="w", pady=2)
        self.state_dot = tk.Canvas(srow, width=18, height=18, bg=BG2, highlightthickness=0)
        self._dot = self.state_dot.create_oval(2, 2, 16, 16, fill=COL_IDLE, outline="")
        self.state_dot.pack(side="left")
        self.state_lbl = tk.Label(srow, text="---", bg=BG2, fg=FG, font=FONT)
        self.state_lbl.pack(side="left", padx=6)
        self.depth_lbl = tk.Label(robot_frame, text="", bg=BG2, fg=FG, font=FONT_MONO)
        self.depth_lbl.pack(anchor="w")
        tk.Label(robot_frame, text="Battery", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(8, 0))
        brow = tk.Frame(robot_frame, bg=BG2)
        brow.pack(anchor="w", pady=2)
        self.batt_canvas = tk.Canvas(brow, width=120, height=16, bg=BG3, highlightthickness=0)
        self._batt_rect = self.batt_canvas.create_rectangle(0, 0, 0, 16, fill=COL_STOP, outline="")
        self.batt_canvas.pack(side="left")
        self.batt_lbl = tk.Label(brow, text="N/A", width=6, bg=BG2, fg=FG, font=FONT)
        self.batt_lbl.pack(side="left", padx=6)

        tk.Label(robot_frame, text="Battery detail", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(8, 0))
        self.bat_detail_lbl = tk.Label(robot_frame, text="temp: N/A   current: N/A   adc: N/A",
                                       bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.bat_detail_lbl.pack(anchor="w")

        tk.Label(robot_frame, text="Attitude", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(8, 0))
        self.atti_lbl = tk.Label(robot_frame, text="pitch: N/A   roll: N/A", bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.atti_lbl.pack(anchor="w")

        tk.Label(robot_frame, text="Chassis speed", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.vel_lbl = tk.Label(robot_frame, text="vx: N/A   vy: N/A",
                                bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.vel_lbl.pack(anchor="w")

        tk.Label(robot_frame, text="Wheels (RPM)", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.esc_lbl = tk.Label(robot_frame, text="W1:--- W2:--- W3:--- W4:---", bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.esc_lbl.pack(anchor="w")

        tk.Label(robot_frame, text="Status", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.status_lbl = tk.Label(robot_frame, text="OK", bg=BG2, fg=ACCENT, font=FONT_MONO, anchor="w")
        self.status_lbl.pack(anchor="w")

        tk.Label(robot_frame, text="Front TOF / Obstacle", bg=BG2, fg=FG_DIM, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.tof_lbl = tk.Label(robot_frame, text="N/A", bg=BG2, fg=FG, font=FONT_MONO, anchor="w")
        self.tof_lbl.pack(anchor="w")

        # --- Raspberry Pi status (2026-08-04), own panel since 2026-08-14 -----
        # The Pi's temperature/load/RAM, not the robot's: on this project the Pi
        # now carries the WHOLE perception pipeline (camera + Carolus, and MINS
        # eventually), so it is what saturates first. The 2026-08-04 MINS test
        # measured one core at 100-118% while three sat idle -- information
        # invisible from the robot, and decisive when diagnosing a slowdown. Read
        # over SSH rather than through ROS, so it stays true even when the ROS
        # stack is stopped or has crashed.
        pi_frame = tk.Frame(left, bg=BG2)
        pi_frame.pack(fill="x", ipadx=8, ipady=6, pady=(6, 0))
        tk.Label(pi_frame, text="RASPBERRY PI STATUS", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w")
        self.pi_lbl = tk.Label(pi_frame, text="temp --  load --  ram --",
                               bg=BG2, fg=FG_DIM, font=FONT_MONO, anchor="w")
        self.pi_lbl.pack(anchor="w", pady=(2, 0))

        right = tk.Frame(dash, bg=BG2)
        right.grid(row=0, column=1, sticky="ne", ipadx=4, ipady=4)
        tk.Label(right, text="CAMERA (preview ~20 Hz)", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w")
        self.cam_canvas = tk.Canvas(right, width=PREVIEW_BASE_W, height=PREVIEW_BASE_H, bg="black", highlightthickness=0)
        self.cam_canvas.pack()
        self._cam_txt = self.cam_canvas.create_text(*self._preview_center(), text="waiting...", fill=FG_DIM)

        # --- Carolus blob-detection view (2026-08-14) ---
        # Black and white (threshold image) + coloured circle on each detected blob,
        # exactly the image carolus_astrobee.cpp publishes on /postprocessed/image --
        # useful to verify the LEDs are actually seen / tune the threshold, without
        # going through rviz.
        tk.Label(right, text="BLOB DETECTION (Carolus, B&W)", bg=BG2, fg=ACCENT,
                anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self.blobs_canvas = tk.Canvas(right, width=PREVIEW_BASE_W, height=PREVIEW_BASE_H, bg="black", highlightthickness=0)
        self.blobs_canvas.pack()
        self._blobs_txt = self.blobs_canvas.create_text(*self._preview_center(), text="disabled", fill=FG_DIM)

        # --- Voyant statut balise + minimap (2026-07-23) ---
        beacon_row = tk.Frame(right, bg=BG2)
        beacon_row.pack(fill="x", pady=(6, 0))
        self._beacon_dot = tk.Canvas(beacon_row, width=14, height=14, bg=BG2, highlightthickness=0)
        self._beacon_dot_id = self._beacon_dot.create_oval(2, 2, 12, 12, fill=COL_KO, outline="")
        self._beacon_dot.pack(side="left")
        self._beacon_status_lbl = tk.Label(beacon_row, text="BEACON: LOST", bg=BG2, fg=COL_KO, font=FONT_MONO)
        self._beacon_status_lbl.pack(side="left", padx=6)

        tk.Label(right, text="BEACON MINIMAP", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w", pady=(6, 0))
        self._minimap = tk.Canvas(right, width=100, height=100, bg="black", highlightthickness=0)
        self._minimap.pack()
        self._minimap_cross_h = self._minimap.create_line(0, 50, 100, 50, fill="#444444")
        self._minimap_cross_v = self._minimap.create_line(50, 0, 50, 100, fill="#444444")
        self._minimap_dot = self._minimap.create_oval(46, 46, 54, 54, fill=COL_IDLE, outline="", state="hidden")
        self._beacon_detected = False

        # RECENTRER CAM (2026-07-23) : remet la nacelle a sa position de base
        # (pitch=0, yaw=0, gimbal.recenter() du SDK) -- orientation de la CAMERA,
        # independant de l'orientation du chassis robot.
        self._recenter_btn = tk.Button(right, text="RECENTER CAM", bg=BG3, fg=FG,
                                       relief="flat", activebackground=COL_ALIGN, activeforeground=FG,
                                       font=FONT, command=self._on_gimbal_recenter)
        self._recenter_btn.pack(fill="x", pady=(6, 0))

        # --- Docking (2026-07-27): commands on /carolus/dock (relayed by
        # cam_view_helper.py, same mechanism as RECENTER), status read from T5's
        # logs ([DOCKSTATUS], same mechanism as [BEACON]). ---
        tk.Label(right, text="BEACON DOCKING", bg=BG2, fg=ACCENT, anchor="w", font=FONT).pack(anchor="w", pady=(10, 0))
        # Two independent-click calibration (2026-07-27): no blocking timer between
        # the two measurements -- each step waits for an explicit click, read from the
        # GUI status rather than by watching for a message in a scrolling log.
        dock_cal_row = tk.Frame(right, bg=BG2)
        dock_cal_row.pack(fill="x", pady=(2, 0))
        tk.Button(dock_cal_row, text="CALIBRATE (1)", bg=BG3, fg=FG, relief="flat",
                  activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                  command=lambda: self._on_dock_cmd("CALIBRATE")).pack(side="left", fill="x", expand=True, padx=(0, 2))
        tk.Button(dock_cal_row, text="CAL STEP 2", bg=BG3, fg=FG, relief="flat",
                  activebackground=COL_ALIGN, activeforeground=FG, font=FONT,
                  command=lambda: self._on_dock_cmd("CALSTEP2")).pack(side="left", fill="x", expand=True, padx=(2, 0))
        # Isolated tests (2026-07-28): ALIGN_ONLY rotates the chassis WITHOUT ever
        # advancing; APPROACH_ONLY advances WITHOUT ever rotating the chassis (and
        # refuses if the chassis is not already aligned -- see beacon_docking.py).
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
        logh = tk.Frame(right_col, bg=BG)
        logh.pack(fill="x", padx=12, pady=(6, 0))
        tk.Label(logh, text="Logs:", bg=BG, fg=FG_DIM, font=FONT).pack(side="left")
        tk.Button(logh, text="Copy logs (active tab)", bg=BG3, fg=FG, relief="flat",
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

        self.log_nb = ttk.Notebook(right_col, style="Carolus.TNotebook")
        self.log_nb.pack(padx=12, pady=(2, 12), fill="both", expand=True)

        self.log_boxes = []
        # The machine is in the label, deliberately. On 2026-08-04 a /pose
        # measurement was attributed to the Pi while Carolus was running on the
        # lab PC: the ROS master lives on the Pi in both cases, so nothing on
        # screen distinguished them. The machine name is the most useful thing
        # on the tab -- it comes first.
        tab_labels = ["T1 roscore [Pi]", "T2 Camera+Beacon [Pi]", "T3 Carolus [Pi]",
                      "T4 TF Broadcaster [Pi]", "T5 Docking [PC]", "T6 MINS [Pi]"]
        for label in tab_labels:
            box = tk.Text(self.log_nb, height=16, width=66, bg=BG2, fg=FG,
                          insertbackground=FG, relief="flat", padx=6, pady=4,
                          font=FONT_MONO, wrap="none", state="disabled")
            # Read-only BUT still selectable and copyable. state="disabled"
            # (2026-07-23, BUG-061): before this fix _block_edit let Z/Q/S/D and the
            # numpad through to the root window (so the robot could be driven from a
            # focused log tab), but it ALSO let Text's default binding insert the
            # character into the log -- the piloting keys appeared literally in the
            # text, and the editing cost (insert + reflow) on the Tkinter thread added
            # latency perceptible while driving. A disabled Text refuses every
            # insert/delete, including ours (_log() flips back to "normal" for the
            # duration of a write), but tag_add (selection, Ctrl+A) and .get() (copy)
            # are NOT blocked by that state -- copy and select still work.
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
        tk.Label(ch, text="CHASSIS (ZQSD)", bg=BG2, fg=ACCENT,
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
        tk.Label(gm, text="GIMBAL (NUM 8/4/5/6/2)", bg=BG2, fg=ACCENT,
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
        tk.Label(wr, text="WHEELS (tilt)", bg=BG2, fg=ACCENT,
                 anchor="w", font=FONT).pack(anchor="w", pady=(0, 4))
        wr_keys = tk.Frame(wr, bg=BG2)
        wr_keys.pack()
        # Forward (rear wheels push, the front lifts)
        b_av = tk.Label(wr_keys, text="FW↑", width=4, height=1,
                        bg=BG3, fg=FG, font=FONT_MONO, bd=1, relief="raised")
        b_av.grid(row=0, column=0, padx=3, pady=3)
        # Wheel stop
        b_st = tk.Label(wr_keys, text="■", width=4, height=1,
                        bg=BG3, fg=COL_KO, font=FONT_MONO, bd=1, relief="raised")
        b_st.grid(row=0, column=1, padx=3, pady=3)
        # Backward (front wheels push, the rear lifts)
        b_ar = tk.Label(wr_keys, text="BW↑", width=4, height=1,
                        bg=BG3, fg=FG, font=FONT_MONO, bd=1, relief="raised")
        b_ar.grid(row=0, column=2, padx=3, pady=3)

        b_av.bind("<ButtonPress-1>",   lambda e: self._on_tilt_press("0 0 300 300"))
        b_av.bind("<ButtonRelease-1>", lambda e: self._on_tilt_release())
        b_st.bind("<ButtonPress-1>",   lambda e: self._on_tilt_release())
        b_ar.bind("<ButtonPress-1>",   lambda e: self._on_tilt_press("300 300 0 0"))
        b_ar.bind("<ButtonRelease-1>", lambda e: self._on_tilt_release())

        tk.Label(wr, text="Press and hold", bg=BG2, fg=FG_DIM,
                 font=FONT).pack(anchor="w", pady=(4, 0))

    def _toggle_locate(self):
        self._locate_active = not self._locate_active
        if self._locate_active:
            self._locate_btn.config(bg=COL_ALIGN, fg=FG)
            self._send_to_helper("MODE LOCATE")
        else:
            self._locate_btn.config(bg=BG3, fg=FG_DIM)
            self._send_to_helper("MODE AUTO")

    def _toggle_gimbal_lock(self):
        # Active only in MANUAL mode on the rm_cam_beacon.py side (silently ignored
        # in AUTO/LOCATE) -- the button stays usable in every mode with no risk.
        # Periodic re-centring (see rm_cam_beacon.py::_gimbal_lock_tick), not a
        # continuous servo -- the old continuous-servo BEACON LOCK was removed on
        # 2026-07-23.
        self._gimbal_lock_active = not self._gimbal_lock_active
        if self._gimbal_lock_active:
            self._lock_btn.config(text="LOCK: ON", bg=COL_ALIGN, fg=FG)
            self._send_to_helper("LOCK ON")
            self.after(0, self._log, "> Beacon lock ON (periodic re-centring in MANUAL mode)")
        else:
            self._lock_btn.config(text="LOCK: OFF", bg=BG3, fg=FG_DIM)
            self._send_to_helper("LOCK OFF")
            self.after(0, self._log, "> Beacon lock OFF")

    def _on_lock_period_changed(self, event=None):
        # Live-configurable period (2026-07-23), SECONDS only. No blocking validation
        # here: the value is sent as typed and rm_cam_beacon.py falls back to 2.0 s if
        # it is not a valid number (see _gimbal_lock_period_cb) -- like a web form
        # field that ignores bad input without crashing.
        value = self._lock_period_entry.get().strip()
        self._send_to_helper(f"LOCKPERIOD {value}")
        self.after(0, self._log, f"> LOCK re-centring period -> {value}s (falls back to 2s if invalid)")

    def _on_gimbal_recenter(self):
        self._send_to_helper("RECENTER")
        self.after(0, self._log, "> RECENTER CAM -- gimbal to its base position")

    def _on_dock_cmd(self, cmd):
        # T5 must be running for the command to have any effect (nothing else
        # subscribes to /carolus/dock) -- no blocking guard here, the button stays
        # usable at any time, same reasoning as LOCK.
        self._send_to_helper(f"DOCK {cmd}")
        self.after(0, self._log, f"> DOCK {cmd}")

    def _on_dock_status(self, status, yaw_validated):
        """Parse [DOCKSTATUS] status=... yaw_validated=... (~1Hz, T5) : met a jour
        the label. Same mechanism as _on_beacon_status for [BEACON]."""
        if status in ("DOCKED", "CAL_DONE", "RANGE_ONLY", "ALIGN_DONE", "APPROACH_DONE"):
            color = COL_OK
        elif status in ("ABORTED", "ERROR", "CAL_FAILED", "CAL_INCONCLUSIVE", "NO_BEACON",
                        "NOT_CONVERGED", "GIMBAL_ALIGN_FAILED", "NOT_ALIGNED",
                        # 2026-07-30: CHASSIS_ALIGN_FAILED and SEQUENCE_TIMEOUT
                        # have been emitted by beacon_docking.py since 2026-07-28
                        # but never appeared here -- so they showed as "unknown"
                        # grey instead of red. CHASSIS_ALIGN_FAILED is precisely
                        # the status of the 2026-07-29 failure cascade.
                        "CHASSIS_ALIGN_FAILED", "SEQUENCE_TIMEOUT",
                        # new status from the verified alignment loop
                        "ALIGN_NOT_CONVERGED"):
            color = COL_KO
        elif status in ("DOCKING", "CALIBRATING", "CAL_STEP1_DONE",
                        # neither a clear success nor a failure: yaw_rel converged but the
                        # mesure de controle n'a pas pu etre faite (2026-07-30)
                        "ALIGN_DONE_UNVERIFIED"):
            color = COL_ALIGN
        else:
            color = FG_DIM
        suffix = " [YAW OK]" if yaw_validated else " [YAW NOT VALIDATED]"
        self._dock_status_lbl.config(text=f"DOCK: {status}{suffix}", fg=color)

    def _reset_beacon_ui(self):
        """Full visual reset of the indicator and minimap -- called at the same
        points as the LOCK reset (entering MANUAL, leaving AUTO, Kill)."""
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
        # OFF by default (2026-07-23): this cuts the helper's /camera/color/image_raw
        # subscription, not merely the display -- it buys keyboard smoothness AND
        # frees network bandwidth on a topic already identified as the bottleneck.
        self._camera_enabled = not self._camera_enabled
        if self._camera_enabled:
            self._cam_btn.config(text="CAM PREVIEW: ON", bg=ACCENT, fg=BG)
            self._send_to_helper("CAM ON")
            self.after(0, self._log, "> Camera preview ON")
        else:
            self._cam_btn.config(text="CAM PREVIEW: OFF", bg=BG3, fg=FG_DIM)
            self._send_to_helper("CAM OFF")
            self.cam_canvas.delete("all")
            self._cam_txt = self.cam_canvas.create_text(*self._preview_center(), text="preview disabled", fill=FG_DIM)
            self.cam_img = None

    def _toggle_blob_preview(self):
        # Same reasoning as _toggle_camera_preview: OFF cuts the helper's
        # /postprocessed/image subscription, not merely the display.
        self._blobs_enabled = not self._blobs_enabled
        if self._blobs_enabled:
            self._blobs_btn.config(text="BLOB VIEW: ON", bg=ACCENT, fg=BG)
            self._send_to_helper("BLOBS ON")
            self.after(0, self._log, "> Blob view ON")
        else:
            self._blobs_btn.config(text="BLOB VIEW: OFF", bg=BG3, fg=FG_DIM)
            self._send_to_helper("BLOBS OFF")
            self.blobs_canvas.delete("all")
            self._blobs_txt = self.blobs_canvas.create_text(*self._preview_center(), text="preview disabled", fill=FG_DIM)
            self.blobs_img = None
            self._cam_png_mtime = None   # force a decode when the preview comes back
            self.after(0, self._log, "> Camera preview OFF (smoothness + bandwidth)")

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

    # Tab names used as the on-disk prefix. Aligned with `tab_labels` (~line 511)
    # but deliberately short: these prefixes exist to be grepped.
    # ---------------------------------------------------------------- Pi state
    # Periodic SSH probe (2026-08-04). Three precautions, each for a failure
    # already seen on this project:
    #   - A SEPARATE THREAD: on 2026-08-04 the Pi answered pings while leaving SSH
    #     hanging indefinitely. A synchronous read would have frozen the GUI.
    #   - A HARD ssh timeout (BatchMode + ConnectTimeout): without it the command
    #     waits for a password that will never come and never returns.
    #   - A SLOW period (20 s): this is context, not real-time telemetry, and a
    #     fast probe would add SSH load to a Pi being watched precisely because it
    #     saturates.
    PI_PROBE_PERIOD_MS = 20000

    def _pi_state_tick(self):
        """Fire the probe, then re-arm. Never blocks the GUI thread."""
        threading.Thread(target=self._pi_state_probe, daemon=True).start()
        self.after(self.PI_PROBE_PERIOD_MS, self._pi_state_tick)

    def _pi_state_probe(self):
        """Lit temperature / charge / RAM / frequence du Pi par SSH.

        Everything is read from /sys and /proc: available on Ubuntu, unlike
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
        """Write one line to the session log (2026-07-31).

        One file per launcher run, under `logs/`, timestamped at startup:
        `logs/session-YYYY-MM-DD-HH-MM-SS.log`. Every line carries the time and
        its originating tab (`T1`..`T5`, or `--` for a global event broadcast
        everywhere), so a specific terminal can be grepped afterwards without
        replaying the session.

        Best-effort by construction: any write error (disk full, permissions, a
        path that vanished) is swallowed. A log that fails to write is an
        annoyance; a piloting GUI that dies while the robot is moving is not.
        Same reasoning as the `except Exception` blocks already wrapped around
        this file's subprocess/SSH calls.
        """
        try:
            if self._log_fh is None:
                return
            tag = "--" if tab is None else self._LOG_TAGS[tab] if tab < len(self._LOG_TAGS) else f"T{tab+1}"
            self._log_fh.write(f"{time.strftime('%H:%M:%S')} [{tag}] {msg}\n")
            self._log_fh.flush()   # flush every line: a crash must not take the
                                   # buffer with it -- a crash is exactly when the
                                   # file will be read back
        except Exception:
            pass

    def _open_session_log(self):
        """Open the session log file. Called once at startup. On failure,
        `_log_fh` stays None and `_log_to_disk` becomes a silent no-op -- the
        launcher behaves exactly as it did before."""
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
            # Best-effort failure, BUT not a silent one (2026-07-31). A log that
            # fails to write without saying so is worse than no log at all: you
            # believe you have the data and you do not. That is exactly the failure
            # mode this project keeps hitting (BUG-087: a non-converged solve
            # published as valid). Written to stderr rather than the GUI, because at
            # this point in __init__ the log widgets do not exist yet.
            self._log_fh = None
            self._session_log_path = None
            print(f"[LAUNCHER] session log unavailable ({e}) — "
                  f"logs will stay in-memory only", file=sys.stderr)

    def _log(self, msg, tab=None):
        # tab=None -> broadcast the message (a global event) to every tab;
        # tab=i    -> write only to the tab of terminal Ti+1.

        # On-disk persistence (2026-07-31). Before this, logs lived ONLY in the
        # tkinter widgets: closing the launcher lost them, and each tab is truncated
        # to 300 lines anyway (below). The concrete cost, observed on 2026-07-31:
        # the question "is LOCK still ticking during a docking run?" went unanswered
        # while the answer sat in the T2 logs of a run that had already happened --
        # they simply had not been kept. One file per session, prefixed by tab,
        # written best-effort: a write error must never bring the GUI down.
        self._log_to_disk(msg, tab)

        boxes = self.log_boxes if tab is None else [self.log_boxes[tab]]
        for box in boxes:
            # The Text widget is state="disabled" (BUG-061): flip back to "normal"
            # for the duration of the programmatic write, or insert()/delete() are
            # refused.
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
        self._log("> Logs copied to the clipboard", idx)

    # ── statut / etat des lignes ──────────────────────────────────────────────

    def _set_status(self, i, text):
        self.after(0, lambda: self.rows[i][1].config(text=text))

    def _reset_row(self, i, unlocked):
        self.after(0, lambda: self.rows[i][0].config(state="normal" if unlocked else "disabled"))
        self.after(0, lambda: self.rows[i][1].config(text=S_IDLE if unlocked else S_LOCKED))

    def _kill_on_pi(self, tab, *patterns):
        """Kill on the Pi, verify, and LOG if anything survives.

        Before BUG-095 (2026-08-04) the Kill buttons sent an unbracketed
        'pkill -f <pattern>' that could kill its own ssh shell before reaching
        the target -- with nothing reporting it. We no longer trust the
        command's return: we read the Pi's state back.
        """
        survivors = remote_kill(*patterns)
        if survivors:
            self._log(f"> !! STILL ALIVE on the Pi: {', '.join(survivors)}", tab)
        return survivors

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
        self.bat_detail_lbl.config(text="temp: N/A   current: N/A   adc: N/A", fg=FG)
        self.atti_lbl.config(text="pitch: N/A   roll: N/A")
        self.vel_lbl.config(text="vx: N/A   vy: N/A")
        self.esc_lbl.config(text="W1:---  W2:---  W3:---  W4:---")
        self.status_lbl.config(text="OK", fg=ACCENT)
        self.tof_lbl.config(text="N/A", fg=FG)
        self.cam_canvas.delete("all")
        self.cam_canvas.create_text(*self._preview_center(), text="waiting...", fill=FG_DIM)
        self.cam_img = None
        self._cam_png_mtime = None
        self.blobs_canvas.delete("all")
        self.blobs_canvas.create_text(*self._preview_center(), text="waiting...", fill=FG_DIM)
        self.blobs_img = None
        self._blobs_png_mtime = None
        self._last_robot_pos = (0.0, 0.0)
        try:
            os.remove(CAM_PNG)
        except OSError:
            pass
        try:
            os.remove(BLOBS_PNG)
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
        self.conn_lbl.config(text=f"OK ({PI_HOST})" if ok else f"unreachable ({PI_HOST})",
                             fg=ACCENT if ok else COL_KO)

    # -- checks with timeout and cancellation ----------------------------------

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

    # BUG-098 (2026-08-10): the drain ceiling was 50 lines per 50 ms tick, i.e.
    # 1000 lines/s. Measured on logs/session-2026-08-10-15-54-11.log (467050
    # lines / 919 s): the real arrival rate is a MEDIAN of 614 and a PEAK of 915
    # lines/s, 92% of that ceiling. `_log_queue` is unbounded, so at peak any
    # delayed tick makes it back up and never recover -- the operator then reads
    # a log and a dashboard that lag reality by seconds while the robot moves,
    # which is indistinguishable from the GUI having stopped responding.
    #
    # Raised to 400/tick (8000 lines/s, ~9x headroom over the measured peak).
    # The cap is kept rather than draining the queue whole: it bounds the worst
    # case a single tick can cost after a long backlog. When the backlog is real
    # it is now SAID so, once every 5 s, instead of being absorbed silently --
    # the previous behaviour hid exactly the condition worth knowing about.
    LOG_DRAIN_PER_TICK = 400

    # Manual-command heartbeat (BUG-099, 2026-08-10). Must stay comfortably
    # under the Pi's MANUAL_CMDVEL_TIMEOUT (0.5 s) -- 200 ms gives 2.5x margin,
    # so two consecutive lost heartbeats still do not trip the deadman.
    CMD_HEARTBEAT_MS = 200

    def _flush_log_queue(self):
        batch = []
        try:
            while len(batch) < self.LOG_DRAIN_PER_TICK:
                batch.append(self._log_queue.get_nowait())
        except queue.Empty:
            pass
        for tag, line in batch:
            if line:
                self._on_log_line(tag, line)
        if len(batch) == self.LOG_DRAIN_PER_TICK:
            backlog = self._log_queue.qsize()
            if backlog > self.LOG_DRAIN_PER_TICK:
                now = time.time()
                if now - getattr(self, "_last_backlog_warn", 0.0) > 5.0:
                    self._last_backlog_warn = now
                    self._log(f"> !! log backlog {backlog} lines — the GUI is "
                              f"behind the robot", None)
        self.after(50, self._flush_log_queue)

    def _on_log_line(self, tag, line):
        if not line:
            return
        idx = int(tag[1:]) - 1   # "T1".."T4" -> 0..3, onglet dedie -> plus besoin du prefixe [Tn]
        # Lignes de telemetrie haute frequence : dashboard uniquement, log texte epargne
        if not any(t in line for t in _LOG_SUPPRESS):
            self._log(line, idx)
        # dashboard parsing (relevant for rm_cam_beacon; harmless for the others)
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
                # Carolus publishes the camera-in-beacon-frame pose (negative Z = beacon ahead).
                # depth = -rel_z (positif = balise devant), lateral = rel_x (positif = droite).
                # Yaw DJI : CW positif depuis nord.
                wx = rx - rel_z * math.cos(yaw_r) - rel_x * math.sin(yaw_r)
                wy = ry - rel_z * math.sin(yaw_r) + rel_x * math.cos(yaw_r)
                face_deg = math.degrees(math.atan2(rx - wx, ry - wy))
                self._last_beacon_ts = time.time()
        if "[DOCKSTATUS]" in line:
            self._t5_dock_ready = True   # premiere ligne vue -> T5 a fini son __init__
            m = RE_DOCKSTATUS.search(line)
            if m:
                self._on_dock_status(m.group(1), m.group(2) == "True")

    def _check_beacon_freshness(self):
        """Expire the last-detection timestamp after BEACON_FRESH_S.

        Until 2026-08-10 this also hid a marker on the embedded live map; that
        feature was removed (see shortcuts/README.md). The timer stays because
        `_last_beacon_ts` remains the freshness source for the beacon
        indicator, which has its own reset path.
        """
        if self._last_beacon_ts and (time.time() - self._last_beacon_ts > BEACON_FRESH_S):
            self._last_beacon_ts = 0.0
        self.after(300, self._check_beacon_freshness)

    # -- video stream (main-thread loop) ---------------------------------------

    def _preview_size(self):
        """Current displayed preview size (base thumbnail x current zoom)."""
        return (PREVIEW_BASE_W * self._preview_zoom, PREVIEW_BASE_H * self._preview_zoom)

    def _preview_center(self):
        w, h = self._preview_size()
        return (w // 2, h // 2)

    def _on_root_resize(self, event):
        # Only react to the root window itself resizing, not every child widget's
        # own Configure event bubbling through (Tkinter delivers <Configure> to
        # every bound widget on any size change in its subtree).
        if event.widget is not self:
            return
        if self._resize_after_id is not None:
            self.after_cancel(self._resize_after_id)
        # Debounced: a window drag fires many Configure events per second: without
        # this, every one of them would decide a zoom factor and resize two
        # canvases, fighting the very drag the user is performing.
        self._resize_after_id = self.after(150, self._apply_preview_zoom)

    def _apply_preview_zoom(self):
        self._resize_after_id = None
        # Budget the camera column roughly 45% of the window's own width (the
        # status/controls column to its left needs the rest) and give it up to
        # half the window's height (it shares the right dashboard column with the
        # beacon minimap and docking controls below it) -- then take the largest
        # WHOLE multiple of the base 480x270 that still fits both budgets, capped
        # at PREVIEW_MAX_ZOOM. An integer factor keeps PhotoImage.zoom() exact
        # (no interpolation) and keeps the memory/CPU cost predictable.
        avail_w = int(self.winfo_width() * 0.45)
        avail_h = int(self.winfo_height() * 0.5)
        zoom_w = max(1, avail_w // PREVIEW_BASE_W)
        zoom_h = max(1, avail_h // PREVIEW_BASE_H)
        zoom = max(1, min(zoom_w, zoom_h, PREVIEW_MAX_ZOOM))
        if zoom == self._preview_zoom:
            return
        self._preview_zoom = zoom
        w, h = self._preview_size()
        cx, cy = self._preview_center()
        for canvas, txt_attr, img_attr, placeholder in (
            (self.cam_canvas, "_cam_txt", "cam_img", "waiting..." if self._camera_enabled else "preview disabled"),
            (self.blobs_canvas, "_blobs_txt", "blobs_img", "waiting..." if self._blobs_enabled else "preview disabled"),
        ):
            canvas.config(width=w, height=h)
            # Only re-anchor the placeholder text -- a live image is simply
            # redrawn at the new zoom on its own next tick (_refresh_cam/_blobs),
            # a few tens of ms away at most, not worth a synchronous re-zoom here.
            if getattr(self, img_attr) is None:
                canvas.delete("all")
                setattr(self, txt_attr, canvas.create_text(cx, cy, text=placeholder, fill=FG_DIM))

    def _refresh_cam(self):
        # Nothing to do while the preview is disabled (2026-07-23): the helper no
        # longer writes a PNG (it has unsubscribed from /camera/color/image_raw), and
        # we do not want to
        # decode/redraw a stale file -- buys keyboard smoothness.
        if not self._camera_enabled:
            self.after(200, self._refresh_cam)   # low rate: just to detect a re-activation
            return
        try:
            if os.path.exists(CAM_PNG):
                # Decode only when the file actually changed (2026-08-10).
                # `tk.PhotoImage(file=...)` is a full PNG decode on the main
                # thread, measured at 6.19 ms -- 12.4% of it at this 50 ms
                # period. The helper writes at its own 20 Hz (THROTTLE_S=0.05)
                # and the two are not synchronised, so a plain re-read decodes
                # the same bytes again whenever the phases drift. An mtime
                # comparison skips those: nothing new to show, nothing to pay.
                mtime = os.path.getmtime(CAM_PNG)
                if mtime != self._cam_png_mtime:
                    self._cam_png_mtime = mtime
                    img = tk.PhotoImage(file=CAM_PNG)
                    if self._preview_zoom > 1:
                        img = img.zoom(self._preview_zoom, self._preview_zoom)
                    self.cam_canvas.delete("all")
                    self.cam_canvas.create_image(*self._preview_center(), image=img, anchor="center")
                    self.cam_img = img
                    self._cam_txt = None
        except Exception:
            pass
        # Rafraichissement apercu GUI (2026-07-22 : 500->50 ms, soit 2->20 Hz).
        # Cosmetic: reads the PNG thumbnail written by cam_view_helper. Does not
        # affect the Carolus stream. Aligned with the helper's write rate
        # (THROTTLE_S=0.05).
        self.after(50, self._refresh_cam)

    def _refresh_blobs(self):
        # Mirrors _refresh_cam exactly, for the /postprocessed/image thumbnail.
        if not self._blobs_enabled:
            self.after(200, self._refresh_blobs)
            return
        try:
            if os.path.exists(BLOBS_PNG):
                mtime = os.path.getmtime(BLOBS_PNG)
                if mtime != self._blobs_png_mtime:
                    self._blobs_png_mtime = mtime
                    img = tk.PhotoImage(file=BLOBS_PNG)
                    if self._preview_zoom > 1:
                        img = img.zoom(self._preview_zoom, self._preview_zoom)
                    self.blobs_canvas.delete("all")
                    self.blobs_canvas.create_image(*self._preview_center(), image=img, anchor="center")
                    self.blobs_img = img
                    self._blobs_txt = None
        except Exception:
            pass
        self.after(50, self._refresh_blobs)

    def _start_cam_helper(self):
        self._stop_cam_helper()
        # stderr is PIPED, not discarded (2026-08-12, BUG-103). It used to be
        # subprocess.DEVNULL, and that is precisely why a five-day outage went
        # unnoticed: the helper died on `import cv2` every single launch, its
        # traceback went to /dev/null, and this method logged "lance" anyway.
        # Since the helper is also the stdin relay for every camera and gimbal
        # command, its silent death disabled all of them while the GUI kept
        # reporting success. Anything it writes to stderr now lands in T2.
        self.cam_proc = subprocess.Popen(
            ["bash", "-c",
             "source /opt/ros/noetic/setup.bash && "
             f"source {WS}/devel/setup.bash && "
             "export ROS_MASTER_URI=http://192.168.0.103:11311 && "
             "export ROS_IP=192.168.0.100 && "
             f"python3 -u {HELPER} {CAM_PNG} {BLOBS_PNG}"],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
            text=True, bufsize=1)
        self._helper_dead_reported = False
        threading.Thread(target=self._drain_helper_stderr,
                         args=(self.cam_proc,), daemon=True).start()
        self.after(0, self._log, "> Video helper launched (stdin open)", 1)
        # Liveness check, deferred (2026-08-12, BUG-103): an import-time crash
        # takes about a second to happen, so polling immediately would always
        # look healthy. This is the check that would have caught BUG-103 on the
        # very first launch instead of five days later.
        self.after(3000, self._check_helper_alive)

    def _drain_helper_stderr(self, proc):
        """Forward the helper's stderr into T2's log tab (2026-08-12, BUG-103).
        Runs on its own daemon thread: a blocking readline on the GUI thread
        would freeze the whole launcher."""
        try:
            for line in iter(proc.stderr.readline, ""):
                line = line.rstrip()
                if line:
                    self.after(0, self._log, f"> [HELPER] {line}", 1)
        except Exception:
            pass

    def _check_helper_alive(self):
        """Report a helper that died shortly after launch (2026-08-12, BUG-103).
        Without this the GUI looks fully operational while every camera and
        gimbal button is a no-op."""
        if self.cam_proc is not None and self.cam_proc.poll() is not None:
            self.after(0, self._log,
                       "> !! VIDEO HELPER DEAD -- camera preview AND camera/gimbal "
                       "commands (numpad, LOCK, RECENTER) INOPERATIVE. "
                       "See the [HELPER] lines above.", 1)
        # Synchronise l'etat CAM ON/OFF du GUI vers le nouveau process helper (qui
        # demarre desabonne par defaut) -- couvre le cas ou l'utilisateur avait
        # active l'apercu avant un Kill/relance de T2.
        self._send_to_helper("CAM ON" if self._camera_enabled else "CAM OFF")
        self._send_to_helper("BLOBS ON" if self._blobs_enabled else "BLOBS OFF")

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
            # Runs on the lab PC, not the Pi: it holds no SDK connection of its own,
            # commande via /carolus/cmd_vel deja relaye par rm_cam_beacon.py --
            # commanding through /carolus/cmd_vel, already relayed by rm_cam_beacon.py.
            return ["bash", "-c",
                    "source /opt/ros/noetic/setup.bash && "
                    "export ROS_MASTER_URI=http://192.168.0.103:11311 && "
                    "export ROS_IP=192.168.0.100 && "
                    f"stdbuf -oL -eL python3 -u {DOCKING_SCRIPT} 2>&1"]
        if i == 5:
            # MINS tourne SUR LE PI (contrairement a T3/T5) : c'est la machine
            # that carries the sensors, and the only one on Ubuntu 20.04, ROS
            # Noetic's official target. Runs its own simulation for now -- the next
            # step is pointing it at our real topics.
            return ["ssh", "-tt"] + SSH_OPTS + ["-o", "ConnectTimeout=5", PI,
                    "source /opt/ros/noetic/setup.bash; "
                    "export ROS_MASTER_URI=http://localhost:11311; "
                    "export ROS_IP=192.168.0.103; "
                    f"cd {MINS_WS_PI} && source devel/setup.bash; "
                    f"stdbuf -oL -eL roslaunch mins {MINS_LAUNCH} 2>&1"]
        # T3 -- CAROLUS, ON THE PI since 2026-08-04.
        #
        # It ran on the lab PC until then (local bash -c, ROS_IP=.100). Measured
        # 2026-08-04, same beacon at 1.00 m, same afternoon:
        #     /pose on the lab PC : 2.19 Hz
        #     /pose on the Pi     : 13.04 Hz   (x5.95)
        # On the PC every frame is an UNCOMPRESSED 1280x720 sensor_msgs/Image
        # (~2.76 MB) crossing the network before Carolus can touch it; on the Pi
        # it never leaves the machine that captured it.
        #
        # ubuntu2204_preload:=false is MANDATORY here: the LD_PRELOAD this
        # argument disables exists only for Ubuntu 22.04's library conflict and
        # hardcodes x86_64 paths -- leaving it on under aarch64 would be wrong,
        # not merely useless.
        #
        # The lab PC remains the right machine for DEVELOPMENT (compiling is
        # faster than over SSH): in that case run roslaunch by hand in a
        # terminal, not through this tab. This launcher is the OPERATIONS tool,
        # and operations run on the Pi.
        return ["ssh", "-tt"] + SSH_OPTS + ["-o", "ConnectTimeout=5", PI,
                "source /opt/ros/noetic/setup.bash; "
                "export ROS_MASTER_URI=http://localhost:11311; "
                "export ROS_IP=192.168.0.103; "
                "cd ~/carolus_ws && source devel/setup.bash; "
                "stdbuf -oL -eL roslaunch carolus_node testcarolus.launch "
                "ubuntu2204_preload:=false 2>&1"]

    # ── launch ─────────────────────────────────────────────────────────────────

    def _on_launch(self, i):
        self.rows[i][0].config(state="disabled")
        self._set_status(i, S_RUNNING)
        threading.Thread(target=self._run_launch, args=(i,), daemon=True).start()

    def _run_launch(self, i):
        tag = f"T{i+1}"
        # Every terminal has been integrated since 2026-07-20 (one log tab per terminal)
        self.after(0, self._log, "> Launching...", i)

        # Garde-fou anti double-connexion SDK (2026-07-22, BUG-057) : avant de lancer
        # T2, tuer toute instance residuelle de rm_cam_beacon.py sur le Pi. Deux
        # instances = deux ep.initialize() -> commandes de mouvement bloquees. Motif
        # bracketed [r]m_... so the remote shell does not kill itself.
        if i == 1:
            try:
                subprocess.run(
                    ["ssh"] + SSH_OPTS + ["-o", "ConnectTimeout=4", PI,
                     "ps -eo pid,args | awk '/[r]m_cam_beacon\\.py/{print $1}' | xargs -r kill -9"],
                    capture_output=True, timeout=8)
                self.after(0, self._log, "> (pre-kill) leftover rm_cam_beacon.py instances cleared on the Pi", i)
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
            self.after(0, self._log, "> Waiting for roscore (port 11311)...", i)
            if not self._wait_for_roscore(i):
                self.after(0, self._log, "> Cancelled or timeout -- Kill to reset", i)
                return
            self.after(0, self._log, "> OK - roscore ready", i)
        elif i == 1:
            self.after(0, self._log, "> Waiting for /camera/color/image_raw...", i)
            if not self._wait_for_camera(i):
                self.after(0, self._log, "> Cancelled or timeout -- Kill to reset", i)
                return
            self.after(0, self._log, "> OK - Camera ready", i)
            self._start_cam_helper()
            # MANUAL mode by default when T2 starts (2026-07-21, user request) --
            # replaces the old auto-LOCATE (automatic sweep). LOCATE is still available
            # through the LOCATE button, it is just no longer triggered automatically.
            # Goes through _ensure_manual_default rather than _toggle_mode directly, to
            # re-check the state at firing time: this prevents a KILL inside the 500 ms
            # window (which forces AUTO) from being flipped back to MANUAL against an
            # already-dead stack.
            self.after(500, self._ensure_manual_default)
        elif i == 2:
            self.after(0, self._log, "> T3 launched - watch for RPY in the logs", i)
        elif i == 3:
            self.after(0, self._log, "> T4 launched - TF broadcaster active (quaternion fixed, BUG-048)", i)
        elif i == 4:
            # A real wait before unlocking the DOCK buttons (2026-07-27, a bug
            # found in testing: without it, START could be sent before
            # beacon_docking.py's ROS subscription to /carolus/dock was established
            # -> the command was silently lost, with no error). Same logic as
            # _wait_for_roscore/_wait_for_camera for T1/T2: wait for a real sign of
            # life from the node (its first [DOCKSTATUS]), not merely for the
            # process to have started.
            self.after(0, self._log, "> Waiting for beacon_docking.py to be ready (first DOCKSTATUS)...", i)
            deadline = time.time() + 15
            while time.time() < deadline and not self._t5_dock_ready:
                if self._launch_cancelled[i]:
                    self.after(0, self._log, "> Cancelled", i)
                    return
                time.sleep(0.2)
            if not self._t5_dock_ready:
                self.after(0, self._log, "> Timeout -- T5 is not responding, use Kill to reset", i)
                return
            self.after(0, self._log, "> T5 launched - docking ready (waiting for /pose, /odom, /carolus/gimbal_yaw_rel)", i)

        self._set_status(i, S_OK)
        if i + 1 < len(self.rows):
            self._reset_row(i + 1, unlocked=True)

    # ── kill ─────────────────────────────────────────────────────────────────

    def _on_kill(self, i):
        self.after(0, self._log, "> Stopping...")
        threading.Thread(target=self._run_kill, args=(i,), daemon=True).start()

    def _run_kill(self, i):
        targets = [i] if i >= 0 else [0, 1, 2, 3, 4]
        # cancel any in-flight wait_for_* for these targets
        for t in targets:
            self._launch_cancelled[t] = True
        time.sleep(0.1)   # let the threads see the flag
        for t in sorted(targets, reverse=True):
            if t == 4:
                local_kill("beacon_docking.py")
                self._close_terminal(4)
                self._t5_dock_ready = False
                self.after(0, lambda: self._dock_status_lbl.config(text="DOCK: —", fg=FG_DIM))
            elif t == 3:
                self._kill_on_pi(3, "carolus_tf_broadcaster.py")
                self._close_terminal(3)
            elif t == 2:
                # T3 runs ON THE PI since 2026-08-04; we also kill locally to
                # clear any leftover instance on the lab PC.
                local_kill("carolus_astrobee")
                local_kill("roslaunch")
                self._kill_on_pi(2, "carolus_astrobee", "roslaunch")
                self._close_terminal(2)
            elif t == 1:
                self._kill_on_pi(1, "rm_cam_beacon.py")
                self._close_terminal(1)
                self._stop_cam_helper()
                self.after(0, self._reset_dashboard)
                self.after(0, self._force_auto_mode)
            elif t == 0:
                self._kill_on_pi(0, "rm_cam_beacon.py", "carolus_astrobee",
                                 "roslaunch", "carolus_tf_broadcaster.py",
                                 "roscore", "rosmaster")
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
        self.after(0, self._log, "> OK - Stopped. Relaunch with the button.")

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
            self.focus_set()   # force focus on the root window (ZQSD/numpad stay active even when a log tab is clicked)
            self.mode_btn.config(text="MODE: MANUAL  (ZQSD active)", bg=COL_APPROACH)
            # Beacon lock reset to OFF on every ENTRY into MANUAL: guarantees a test
            # session always starts with a fixed gimbal, and neutralises a LOCK ON
            # clicked by mistake in AUTO/LOCATE (a no-op there, but it would otherwise
            # have persisted to here).
            self._gimbal_lock_active = False
            self._lock_btn.config(text="LOCK: OFF", bg=BG3, fg=FG_DIM)
            self._reset_beacon_ui()
            self._send_to_helper("MODE MANUAL")
            self._send_to_helper("LOCK OFF")
            # dashboard: reflect MANUAL mode on the state dot (BUG-014)
            self.last_state = "MANUAL"
            self.state_dot.itemconfig(self._dot, fill=COL_MANUAL)
            self.state_lbl.config(text="MANUEL")
            self.depth_lbl.config(text="")
            self.after(0, self._log, "> MANUAL mode active - ZQSD to drive")
        else:
            self.gui_mode = "AUTO"
            self._keys_down.clear()
            self._gim_down.clear()
            self._update_chassis_visual()
            self._update_gimbal_visual()
            self.mode_btn.config(text="MODE: AUTO", bg=COL_STOP)
            self._send_to_helper("STOP")
            self._send_to_helper("GIMBAL 0.0 0.0")
            self._send_to_helper("MODE AUTO")
            # Lock balise scope au mode MANUEL — reset a la sortie pour ne pas
            # laisser un etat "actif" trompeur au prochain passage en MANUEL.
            self._gimbal_lock_active = False
            self._lock_btn.config(text="LOCK: OFF", bg=BG3, fg=FG_DIM)
            self._send_to_helper("LOCK OFF")
            self._reset_beacon_ui()
            # dashboard: reset so the next T2 line restores the real state (BUG-014)
            self.last_state = None
            self.state_dot.itemconfig(self._dot, fill=COL_IDLE)
            self.state_lbl.config(text="--- (AUTO)")
            self.after(0, self._log, "> AUTO mode active - grace period 5s")

    # ── clavier ZQSD ─────────────────────────────────────────────────────────

    def _bind_keys(self):
        self._bind_keys_to(self)

    def _bind_keys_to(self, window):
        """Bind chassis/gimbal handlers to any Tk window (root or Toplevel)."""
        # Reclaim keyboard focus on hover. NOTE (2026-08-10): this was originally
        # here because T1/T3 opened external gnome-terminal windows that stole WM
        # focus -- those were removed on 2026-07-20 when every terminal became
        # integrated, so that specific thief is gone. The binding is kept because
        # focus_set() is otherwise called only once (when switching to MANUAL
        # mode), and any other window taking focus -- another application, the
        # window manager -- would otherwise leave the keys dead with no way back
        # short of clicking.
        window.bind("<Enter>", lambda e, w=window: w.focus_set())
        # BUG (2026-07-21): if focus leaves the window while a key is held (a click
        # elsewhere, the live map, a dialogue...), the matching KeyRelease is never
        # delivered -> the key stays stuck in _keys_down/_gim_down indefinitely -> the
        # chassis/gimbal keeps receiving the last non-zero speed in a loop on the Pi
        # side (motion "all by itself"). Safety net: on focus loss, treat it as though
        # every key
        # were released, whatever the cause.
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
        # <FocusOut> on a toplevel ALSO fires when focus moves to a child widget (a
        # button, a log tab) or to another window of the app (the map editor). In those
        # cases the KeyRelease still arrives -- the bindings are shared through bindtags
        # or re-bound on the map editor -- so clearing would be a spurious STOP that
        # cut piloting on every click. The test is therefore deferred to after_idle and
        # only clears if focus has REALLY left the application
        # Tk (focus_get() renvoie None) — le seul cas ou un KeyRelease serait perdu.
        self.after_idle(self._focus_out_cleanup_if_left_app)

    def _focus_out_cleanup_if_left_app(self):
        if self.gui_mode != "MANUAL":
            return
        if not self._keys_down and not self._gim_down:
            return
        # Only clear (the stuck-key STOP) if focus is NO LONGER on a window that owns
        # our keyboard bindings (root or the map editor). In those two windows the
        # KeyRelease still arrives (shared handlers), so no STOP is needed and issuing
        # one would cut piloting on every internal click. Everywhere else (another app,
        # minimisation, a modal filedialog without our bindings, a window
        # etrangere) le KeyRelease serait perdu -> on nettoie. Robuste au cas filedialog
        # que la version precedente (focus_get() is None) laissait passer.
        try:
            focused = self.focus_get()
        except KeyError:
            focused = None   # focus sur une fenetre etrangere -> considere hors appli
        if focused is not None:
            top = focused.winfo_toplevel()
            if top is self:
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
        # Do not interfere when focus is on a text field (e.g. the map editor's rotation offset)
        if isinstance(event.widget, (tk.Entry, tk.Text, tk.Spinbox)):
            return
        k = event.keysym.lower()
        if k not in ("z", "q", "s", "d"):
            return
        # X11 auto-repeat: this KeyPress may be the synthetic "re-press" that
        # immediately follows a KeyRelease while the key is held -- so cancel the
        # deferred release instead of processing a new press (this is what avoids the
        # sawtooth on /carolus/cmd_vel, see KEY_REPEAT_DEBOUNCE_MS).
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

    def _cmd_heartbeat(self):
        """Re-send the held-key command periodically (BUG-099, 2026-08-10).

        The Pi stops the chassis if no `/carolus/cmd_vel` arrives within
        `MANUAL_CMDVEL_TIMEOUT` (0.5 s, rm_cam_beacon.py:198) -- a correct
        safety deadman. But the launcher only ever sent on a key EVENT, and
        since BUG-060's auto-repeat debounce (2026-07-23) a held key produces
        no further events at all: the X11 re-press is swallowed to cancel the
        deferred release. So holding Z sent one command, the deadman expired
        0.5 s later, and the robot stopped and stayed stopped WITH THE KEY
        STILL DOWN. From the operator's seat that reads as "the navigation
        commands stopped responding".

        This is the missing half of the deadman: the Pi is right to require a
        heartbeat, the launcher simply never sent one. 200 ms gives 2.5x margin
        against the 0.5 s timeout. It re-sends the SAME value rather than
        alternating with zeros, so it cannot reintroduce BUG-060's sawtooth.
        """
        if self.gui_mode == "MANUAL":
            if self._keys_down:
                self._send_velocity()
            if self._gim_down:
                self._send_gimbal()
        self.after(self.CMD_HEARTBEAT_MS, self._cmd_heartbeat)

    def _send_velocity(self):
        vx = 0.0
        wz = 0.0
        if "z" in self._keys_down: vx += MANUAL_VX
        if "s" in self._keys_down: vx -= MANUAL_VX
        if "q" in self._keys_down: wz += MANUAL_WZ
        if "d" in self._keys_down: wz -= MANUAL_WZ
        self._send_to_helper(f"VX {vx:.2f} WZ {wz:.1f}")

    # normalise the numpad keysym regardless of NumLock state
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
        # X11 auto-repeat debounce -- same logic as _on_key_press.
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
        # Every camera and gimbal control in this GUI goes through here: GIMBAL
        # (numpad), LOCK, LOCKPERIOD, RECENTER, MODE, VX/WZ, WHEELS, CAM ON/OFF,
        # BLOBS ON/OFF.
        # Before 2026-08-12 (BUG-103) a dead or absent helper meant each of them
        # was dropped in silence -- the guard below returned, the bare `except`
        # swallowed the rest, and the caller still logged its own success line.
        # It now says so, once per helper process, instead of pretending.
        alive = (self.cam_proc is not None
                 and self.cam_proc.stdin is not None
                 and self.cam_proc.poll() is None)
        if not alive:
            if not getattr(self, "_helper_dead_reported", False):
                self._helper_dead_reported = True
                self.after(0, self._log,
                           f"> !! command '{cmd}' NOT SENT: video helper absent "
                           "or dead. All camera/gimbal commands are inoperative "
                           "until T2 is relaunched.", 1)
            return
        try:
            self.cam_proc.stdin.write(cmd + "\n")
            self.cam_proc.stdin.flush()
        except Exception as e:
            if not getattr(self, "_helper_dead_reported", False):
                self._helper_dead_reported = True
                self.after(0, self._log,
                           f"> !! command '{cmd}' NOT SENT ({e}).", 1)

    def _force_auto_mode(self):
        self.gui_mode = "AUTO"
        self._keys_down.clear()
        self._gim_down.clear()
        self._update_chassis_visual()
        self._update_gimbal_visual()
        self.mode_btn.config(text="MODE: AUTO", bg=COL_STOP)
        self._gimbal_lock_active = False
        self._lock_btn.config(text="LOCK: OFF", bg=BG3, fg=FG_DIM)
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
        # Best-effort cleanup before closing: otherwise the nodes launched over SSH
        # SURVIVE on the Pi (closing the window does not kill remote processes). An
        # orphaned rm_cam_beacon.py keeps the SDK connection, so the next launch enters
        # a DOUBLE CONNECTION -> motion commands are silently blocked (the "manual mode
        # stopped working" symptom). So we kill the Pi-side and local processes plus the
        # video helper before destroy().
        # BUG-095 (2026-08-04): this cleanup was the most critical of the five,
        # and it was broken. The unbracketed pkill could kill its own ssh shell
        # before reaching the target -- so the orphan described just above
        # survived in exactly the case this block exists to prevent.
        # carolus_astrobee added: T3 runs on the Pi since 2026-08-04.
        try:
            remote_kill("rm_cam_beacon.py", "carolus_tf_broadcaster.py",
                        "carolus_astrobee", "roslaunch", verify=False)
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
