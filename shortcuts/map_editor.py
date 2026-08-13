#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DESACTIVE depuis le 2026-08-10 : le bouton et le cablage qui ouvraient cette
fenetre depuis carolus_launcher.py ont ete retires en meme temps que la
fonctionnalite carte live (mapv1.json). Ce fichier n'est plus lance par rien,
mais reste dans le code (pas archive) : map_collision.py, toujours actif dans
rm_cam_beacon.py, sait lire le format JSON que cet editeur produit -- si la
fonctionnalite carte est un jour relancee, c'est ce fichier qu'il faut
re-cabler dans carolus_launcher.py, pas reecrire.

MapEditor — éditeur de map 2D pour Carolus / RoboMaster S1.
Fenêtre séparée (Toplevel), auparavant ouverte depuis carolus_launcher.py.

Blocs :
  full    → case complète (48×48 px = 40 cm)
  half    → demi-case (1×½ ou ½×1), rotation 0/90/180/270°
  quarter → quart de case (½×½), rotation 0/90/180/270°
  zone    → remplissage rapide par drag : rectangle de cases pleines
  beacon  → demi-bloc balise (overlay persistant), manuel ou auto

Contrôles :
  G-clic (outil sélectionné) → poser un bloc
    · Zone : drag pour remplir un rectangle
    · Beacon : poser la balise manuelle
  D-clic sur un bloc → cycler rotation
  D-clic sur balise manuelle → rotation 90°
  Outil Effacer + G-clic → supprimer un bloc
"""

import os
import math
import json
import tkinter as tk
from tkinter import filedialog, messagebox

# ── Palette couleurs ──────────────────────────────────────────────────────────
BG        = "#0d1117"
BG2       = "#161b22"
BG3       = "#21262d"
FG        = "#e6edf3"
FG_DIM    = "#8b949e"
ACCENT    = "#00d9a0"
COL_KO    = "#cc0000"
COL_MANUAL   = "#1f6feb"
COL_APPROACH = "#e08000"
FONT      = ("TkDefaultFont",)
FONT_MONO = ("TkFixedFont",)

BCOL = {
    'full':    "#374151",
    'half':    "#1e3a8a",
    'quarter': "#14532d",
    'outline': "#6b7280",
}
BCOL_BEACON     = "#d97706"   # orange ambre — balise manuelle
BCOL_BEACON_AUTO = "#f59e0b"  # jaune-or — balises auto (légèrement différent)

# ── Constantes grille ─────────────────────────────────────────────────────────
CELL_PX   = 48
CELL_M    = 0.40
GRID_COLS = 26
GRID_ROWS = 21

BEACON_DEDUP_M = 0.5   # distance min (m) pour considérer deux détections = même balise

# Slots
_HALF_SLOTS = {
    0:   frozenset({'NW', 'NE'}),
    90:  frozenset({'NE', 'SE'}),
    180: frozenset({'SW', 'SE'}),
    270: frozenset({'NW', 'SW'}),
}
_QRT_SLOTS = {
    0:   frozenset({'NW'}),
    90:  frozenset({'NE'}),
    180: frozenset({'SE'}),
    270: frozenset({'SW'}),
}
_SLOT_OFF = {'NW': (0, 0), 'NE': (1, 0), 'SW': (0, 1), 'SE': (1, 1)}


def _slots(btype, rot):
    if btype == 'full':
        return frozenset({'NW', 'NE', 'SW', 'SE'})
    elif btype == 'half':
        return _HALF_SLOTS[rot % 360]
    return _QRT_SLOTS[rot % 360]


def _world_to_canvas_orig(wx_m, wy_m):
    """Monde (m) → coords canvas originales (zoom=1, avant pan/zoom).
    Origine monde = centre canvas. y+ monde = haut sur canvas."""
    cx0 = GRID_COLS * CELL_PX / 2
    cy0 = GRID_ROWS * CELL_PX / 2
    sc  = CELL_PX / CELL_M
    return cx0 + wx_m * sc, cy0 - wy_m * sc


# ── Dessin d'un demi-bloc balise ──────────────────────────────────────────────

def _beacon_rect(px, py, rot):
    """Renvoie (x1,y1,x2,y2) du rectangle demi-bloc balise centré en (px,py).
    rot 0/180 = horizontal (large), 90/270 = vertical (haut)."""
    half = CELL_PX // 2
    qtr  = CELL_PX // 4
    if rot % 180 == 0:   # horizontal
        return px - half, py - qtr, px + half, py + qtr
    else:                # vertical
        return px - qtr, py - half, px + qtr, py + half


def _beacon_rect_z(px, py, rot, z=1.0):
    """Comme _beacon_rect mais zoom-aware (z = facteur zoom courant)."""
    half = CELL_PX * z / 2
    qtr  = CELL_PX * z / 4
    if rot % 180 == 0:
        return px - half, py - qtr, px + half, py + qtr
    else:
        return px - qtr, py - half, px + qtr, py + half


def _beacon_arrow(px, py, facing_deg, length=20):
    """Renvoie (x1,y1,x2,y2) de la flèche d'orientation de la balise."""
    r  = math.radians(facing_deg)
    ex = px + length * math.sin(r)
    ey = py - length * math.cos(r)
    return px, py, ex, ey


# ── Classe principale ─────────────────────────────────────────────────────────

class MapEditor(tk.Toplevel):

    def __init__(self, parent):
        super().__init__(parent)
        self.title("Carolus — Éditeur de Map")
        self.configure(bg=BG)
        self.resizable(False, False)

        self._tool      = 'full'
        self._grid      = {}
        self._hover_ids = []
        self._pal_btns  = {}

        # ── Overlay robot ──
        self._robot_px        = None
        self._robot_id        = None
        self._robot_arrow_id  = None
        self._robot_locked    = True                              # verrouillé par défaut
        self._robot_drag_off  = None
        # Robot au centre absolu de la grille
        _cx0 = GRID_COLS * CELL_PX / 2
        _cy0 = GRID_ROWS * CELL_PX / 2
        self._robot_origin_px = (_cx0, _cy0)

        # ── Balise live (depuis /pose, indicateur temporaire) ──
        self._beacon_id = None

        # ── Balise manuelle ──
        self._man_beacon = {
            'px': None, 'py': None,
            'rot': 0,               # 0/90/180/270° (orientation du visage)
            'facing': 0.0,          # facing_deg libre (pour l'auto), pas utilisé en manuel
            'ids': [],
        }

        # ── Balises auto (cartographiées par le robot) ──
        self._auto_beacons  = []   # list of {'wx': float, 'wy': float, 'facing': float, 'ids': []}
        self._beacon_auto   = False  # False = manuel, True = auto

        # ── Zone (bulk fill) ──
        self._bulk_start   = None
        self._bulk_rect_id = None

        # ── Hover ghost cache (évite delete+create si même case/rot) ──
        self._hover_cell = None   # (row, col, rot) de la dernière ghost dessinée

        # ── Zoom / Pan ──
        self._zoom    = 1.0
        self._ox      = 0.0    # canvas = original * _zoom + _ox
        self._oy      = 0.0
        self._pan_last     = None
        self._pan_did_drag = False

        self._build()

    # ── Construction UI ───────────────────────────────────────────────────────

    def _build(self):
        top = tk.Frame(self, bg=BG2, padx=8, pady=6)
        top.pack(fill="x")
        tk.Label(top, text="ÉDITEUR MAP — Carolus", bg=BG2, fg=ACCENT,
                 font=FONT).pack(side="left", padx=8)
        info = (f"{GRID_COLS}×{GRID_ROWS} cases  ·  "
                f"{GRID_COLS*CELL_M:.0f} m × {GRID_ROWS*CELL_M:.0f} m  ·  "
                f"1 case = {CELL_M*100:.0f} cm ≈ S1")
        tk.Label(top, text=info, bg=BG2, fg=FG_DIM, font=FONT).pack(side="left", padx=10)
        for txt, col, cmd in [
            ("Charger",      BG3,       self._load),
            ("Sauvegarder",  BG3,       self._save),
            ("Effacer tout", "#3a0000", self._clear_all),
        ]:
            fg = ACCENT if txt == "Sauvegarder" else FG
            tk.Button(top, text=txt, bg=col, fg=fg, relief="flat", font=FONT,
                      command=cmd).pack(side="right", padx=3)

        tk.Frame(self, height=2, bg=ACCENT).pack(fill="x")

        body = tk.Frame(self, bg=BG)
        body.pack(fill="both", expand=True, padx=8, pady=8)
        self._build_palette(body)
        self._build_canvas(body)

    def _build_palette(self, parent):
        pal = tk.Frame(parent, bg=BG2, padx=8, pady=8, width=165)
        pal.pack(side="left", fill="y", padx=(0, 8))
        pal.pack_propagate(False)

        tk.Label(pal, text="OUTILS", bg=BG2, fg=ACCENT, font=FONT).pack(anchor="w", pady=(0, 6))

        for key, label, bgcol in [
            ('full',    '▓  Plein  (1×1)',   BCOL['full']),
            ('half',    '▬  Demi  (1×½)',    BCOL['half']),
            ('quarter', '▪  Quart  (½×½)',   BCOL['quarter']),
            ('zone',    '▦  Zone  (drag)',   "#1a3a2a"),
            ('beacon',  '◉  Balise',         "#5c3800"),
            ('erase',   '✕  Effacer',        "#4b1c1c"),
        ]:
            btn = tk.Button(pal, text=label, anchor="w", padx=6,
                            bg=BG3, fg=FG, relief="flat", font=FONT,
                            activebackground=ACCENT, activeforeground=BG,
                            command=lambda k=key: self._select_tool(k))
            btn.pack(fill="x", pady=2)
            self._pal_btns[key] = btn

        # ── Robot ──
        tk.Frame(pal, height=1, bg=BG3).pack(fill="x", pady=6)
        tk.Label(pal, text="Robot :", bg=BG2, fg=FG_DIM, font=FONT).pack(anchor="w")
        self._btn_robot_lock = tk.Button(
            pal, text="[V]  Verrouillé", bg=BG3, fg=FG_DIM,
            relief="flat", font=FONT, anchor="w", padx=4,
            command=self._toggle_robot_lock)
        self._btn_robot_lock.pack(fill="x", pady=2)
        tk.Label(pal, text="(déverr. → drag+snap grille)", bg=BG2,
                 fg=FG_DIM, font=FONT, anchor="w").pack(anchor="w")

        # ── Mode balise ──
        tk.Frame(pal, height=1, bg=BG3).pack(fill="x", pady=6)
        tk.Label(pal, text="Mode balise :", bg=BG2, fg=FG_DIM, font=FONT).pack(anchor="w")
        mode_row = tk.Frame(pal, bg=BG2)
        mode_row.pack(fill="x", pady=2)
        self._btn_man = tk.Button(mode_row, text="MANUEL", font=FONT,
                                   bg=ACCENT, fg=BG, relief="flat",
                                   command=lambda: self._set_beacon_mode(False))
        self._btn_man.pack(side="left", fill="x", expand=True, padx=(0, 2))
        self._btn_auto = tk.Button(mode_row, text="AUTO", font=FONT,
                                    bg=BG3, fg=FG, relief="flat",
                                    command=lambda: self._set_beacon_mode(True))
        self._btn_auto.pack(side="left", fill="x", expand=True, padx=(2, 0))
        self._lbl_auto_n = tk.Label(pal, text="0 balise(s) auto",
                                     bg=BG2, fg=FG_DIM, font=FONT)
        self._lbl_auto_n.pack(anchor="w")
        tk.Button(pal, text="Effacer auto", bg="#3a0000", fg=FG, relief="flat",
                  font=FONT, command=self._clear_auto_beacons).pack(fill="x", pady=2)

        tk.Frame(pal, height=1, bg=BG3).pack(fill="x", pady=6)

        for line in ["G-clic : poser", "D-clic : tourner",
                     "Beacon D-clic :", "rotate 90°", ""]:
            tk.Label(pal, text=line, bg=BG2, fg=FG_DIM, font=FONT,
                     anchor="w").pack(anchor="w")

        tk.Frame(pal, height=1, bg=BG3).pack(fill="x", pady=6)
        tk.Label(pal, text="Légende :", bg=BG2, fg=FG_DIM, font=FONT).pack(anchor="w")
        for btype, label in [('full', 'Plein'), ('half', 'Demi'), ('quarter', 'Quart')]:
            row = tk.Frame(pal, bg=BG2)
            row.pack(anchor="w", pady=1)
            tk.Canvas(row, width=14, height=14, bg=BCOL[btype],
                      highlightthickness=1,
                      highlightbackground=BCOL['outline']).pack(side="left")
            tk.Label(row, text=f" {label}", bg=BG2, fg=FG_DIM, font=FONT).pack(side="left")
        for col, label in [(BCOL_BEACON, "Balise manuel"),
                           (BCOL_BEACON_AUTO, "Balise auto")]:
            row = tk.Frame(pal, bg=BG2)
            row.pack(anchor="w", pady=1)
            tk.Canvas(row, width=14, height=14, bg=col,
                      highlightthickness=1,
                      highlightbackground="#92400e").pack(side="left")
            tk.Label(row, text=f" {label}", bg=BG2, fg=FG_DIM, font=FONT).pack(side="left")

        tk.Frame(pal, height=1, bg=BG3).pack(fill="x", pady=6)
        tk.Label(pal, text="■ ▲  Robot (▲=avant)", bg=BG2, fg=COL_MANUAL,   font=FONT).pack(anchor="w")
        tk.Label(pal, text="●  Balise live",        bg=BG2, fg=COL_APPROACH, font=FONT).pack(anchor="w")

    def _build_canvas(self, parent):
        CW = GRID_COLS * CELL_PX
        CH = GRID_ROWS * CELL_PX

        self._canvas = tk.Canvas(parent, width=CW, height=CH,
                                  bg="#071221", highlightthickness=1,
                                  highlightbackground=ACCENT, cursor="crosshair")
        self._canvas.pack(side="left")
        self._draw_grid()

        # Robot — snap sur case, verrouillé par défaut
        HALF = CELL_PX // 2 - 2
        cx0, cy0 = self._snap_cell_center(*self._robot_origin_px)
        self._robot_origin_px = (cx0, cy0)   # recaler sur grille dès le départ
        self._robot_id = self._canvas.create_rectangle(
            cx0 - HALF, cy0 - HALF, cx0 + HALF, cy0 + HALF,
            fill=COL_MANUAL, outline=FG, width=2, tags=("overlay", "robot_ovl"))
        self._robot_arrow_id = self._canvas.create_line(
            cx0, cy0, cx0, cy0 - CELL_PX,
            fill="#7bc8ff", width=3, arrow=tk.LAST, arrowshape=(10, 12, 4),
            tags=("overlay", "robot_ovl"))

        self._canvas.tag_bind("robot_ovl", "<ButtonPress-1>",   self._robot_press)
        self._canvas.tag_bind("robot_ovl", "<B1-Motion>",       self._robot_drag_ev)
        self._canvas.tag_bind("robot_ovl", "<ButtonRelease-1>", self._robot_release)

        # Balise live (indicateur temporaire)
        self._beacon_id = self._canvas.create_oval(
            0, 0, 1, 1, fill=COL_APPROACH, outline="white",
            width=1, state="hidden", tags="overlay")

        # Balise manuelle (créée masquée, hors canvas)
        self._man_beacon['ids'] = self._draw_beacon_shape(
            -200, -200, 0, 0.0, BCOL_BEACON, visible=False)

        # Bindings drag balise manuelle (sur le tag "man_beacon")
        self._canvas.tag_bind("man_beacon", "<ButtonPress-1>",   self._man_beacon_press)
        self._canvas.tag_bind("man_beacon", "<B1-Motion>",       self._man_beacon_drag_motion)
        self._canvas.tag_bind("man_beacon", "<ButtonRelease-1>", self._man_beacon_release)
        self._canvas.tag_bind("man_beacon", "<ButtonRelease-3>", self._man_beacon_rclick)

        self._canvas.bind("<Button-1>",        self._on_click)
        self._canvas.bind("<B1-Motion>",       self._on_drag)
        self._canvas.bind("<ButtonRelease-1>", self._on_release)
        self._canvas.bind("<ButtonRelease-3>", self._on_rclick_release)
        self._canvas.bind("<Motion>",          self._on_hover)
        self._canvas.bind("<Leave>",           self._on_leave)

        # Zoom (molette souris)
        self._canvas.bind("<MouseWheel>", self._on_zoom)   # Windows / macOS
        self._canvas.bind("<Button-4>",   self._on_zoom)   # Linux scroll up
        self._canvas.bind("<Button-5>",   self._on_zoom)   # Linux scroll down

        # Pan (clic droit maintenu + glisser)
        self._canvas.bind("<ButtonPress-3>",  self._on_pan_start)
        self._canvas.bind("<B3-Motion>",      self._on_pan_drag)

        self._select_tool('full')

    def _draw_grid(self):
        CW = GRID_COLS * CELL_PX
        CH = GRID_ROWS * CELL_PX
        for c in range(GRID_COLS + 1):
            x   = c * CELL_PX
            col = "#1e3a54" if c % 5 == 0 else "#0d1e2e"
            self._canvas.create_line(x, 0, x, CH, fill=col, tags="grid")
        for r in range(GRID_ROWS + 1):
            y   = r * CELL_PX
            col = "#1e3a54" if r % 5 == 0 else "#0d1e2e"
            self._canvas.create_line(0, y, CW, y, fill=col, tags="grid")

    # ── Outil ─────────────────────────────────────────────────────────────────

    def _select_tool(self, key):
        self._tool = key
        special = {'erase': "#4b1c1c", 'zone': "#1a3a2a", 'beacon': "#5c3800"}
        for k, btn in self._pal_btns.items():
            if k == key:
                btn.config(bg=ACCENT, fg=BG)
            else:
                btn.config(bg=special.get(k, BG3), fg=FG)

    # ── Robot lock / drag ────────────────────────────────────────────────────

    def _snap_cell_center(self, px, py):
        """Canvas coords → snap sur centre de case → canvas coords."""
        x0 = (px - self._ox) / self._zoom
        y0 = (py - self._oy) / self._zoom
        col = max(0, min(GRID_COLS - 1, int(x0) // CELL_PX))
        row = max(0, min(GRID_ROWS - 1, int(y0) // CELL_PX))
        sx = col * CELL_PX + CELL_PX // 2
        sy = row * CELL_PX + CELL_PX // 2
        return sx * self._zoom + self._ox, sy * self._zoom + self._oy

    def _toggle_robot_lock(self):
        self._robot_locked = not self._robot_locked
        if self._robot_locked:
            self._btn_robot_lock.config(text="[V]  Verrouillé", bg=BG3, fg=FG_DIM)
        else:
            self._btn_robot_lock.config(text="[O]  Libre (drag)", bg="#1a3a2a", fg=ACCENT)

    def _robot_press(self, event):
        if self._robot_locked:
            return
        rpx, rpy = self._robot_px or self._robot_origin_px
        self._robot_drag_off = (event.x - rpx, event.y - rpy)

    def _robot_drag_ev(self, event):
        if self._robot_locked or self._robot_drag_off is None:
            return
        ox, oy = self._robot_drag_off
        px, py = event.x - ox, event.y - oy
        self._draw_robot_overlay(px, py)

    def _robot_release(self, event):
        if self._robot_locked or self._robot_drag_off is None:
            return
        ox, oy = self._robot_drag_off
        self._robot_drag_off = None
        px, py = self._snap_cell_center(event.x - ox, event.y - oy)
        self._robot_origin_px = (px, py)
        self._robot_px = (px, py)
        self._draw_robot_overlay(px, py)

    # ── Mode balise ───────────────────────────────────────────────────────────

    def _set_beacon_mode(self, auto: bool):
        self._beacon_auto = auto
        if auto:
            self._btn_auto.config(bg=ACCENT, fg=BG)
            self._btn_man.config(bg=BG3, fg=FG)
        else:
            self._btn_man.config(bg=ACCENT, fg=BG)
            self._btn_auto.config(bg=BG3, fg=FG)

    # ── Dessin demi-bloc balise ───────────────────────────────────────────────

    def _draw_beacon_shape(self, px, py, rot, facing_deg, color, visible=True,
                            tag=None):
        """Dessine un demi-bloc balise centré en (px, py) avec flèche de direction.
        px, py sont en coords canvas courantes. Renvoie liste d'IDs canvas."""
        z = self._zoom
        state = "normal" if visible else "hidden"
        tags  = ("overlay",) + ((tag,) if tag else ())

        x1, y1, x2, y2 = _beacon_rect_z(px, py, rot, z)
        rid = self._canvas.create_rectangle(
            x1, y1, x2, y2, fill=color, outline="#92400e", width=2,
            state=state, tags=tags)

        ax1, ay1, ax2, ay2 = _beacon_arrow(px, py, facing_deg, length=max(1, int(18 * z)))
        aid = self._canvas.create_line(
            ax1, ay1, ax2, ay2, fill="white", width=2,
            arrow=tk.LAST, arrowshape=(max(1, int(8*z)), max(1, int(10*z)), max(1, int(3*z))),
            state=state, tags=tags)

        self._canvas.tag_raise("overlay")
        return [rid, aid]

    def _update_beacon_shape(self, ids, px, py, rot, facing_deg):
        """Déplace/réoriente un demi-bloc balise existant (px, py en coords canvas)."""
        z = self._zoom
        x1, y1, x2, y2 = _beacon_rect_z(px, py, rot, z)
        self._canvas.coords(ids[0], x1, y1, x2, y2)
        ax1, ay1, ax2, ay2 = _beacon_arrow(px, py, facing_deg, length=max(1, int(18 * z)))
        self._canvas.coords(ids[1], ax1, ay1, ax2, ay2)
        for cid in ids:
            self._canvas.itemconfig(cid, state="normal")
        self._canvas.tag_raise("overlay")

    # ── Balise manuelle ───────────────────────────────────────────────────────

    def _place_man_beacon(self, px, py):
        mb = self._man_beacon
        mb['px'], mb['py'] = px, py
        # Facing par défaut = vers le haut (nord)
        if mb['facing'] == 0.0 and mb['rot'] == 0:
            mb['facing'] = 0.0
        # Effacer les anciens IDs et recréer
        for cid in mb['ids']:
            self._canvas.delete(cid)
        mb['ids'] = self._draw_beacon_shape(
            px, py, mb['rot'], float(mb['rot']),
            BCOL_BEACON, visible=True, tag="man_beacon")

    def _redraw_man_beacon(self):
        mb = self._man_beacon
        if mb['px'] is None:
            return
        self._update_beacon_shape(mb['ids'], mb['px'], mb['py'], mb['rot'],
                                   float(mb['rot']))  # facing = rot angle en degrés
        for cid in mb['ids']:
            self._canvas.itemconfig(cid, state="normal")

    def _man_beacon_press(self, event):
        mb = self._man_beacon
        if mb['px'] is not None:
            # mb['px/py'] en coords canvas — event.x/y aussi
            self._man_beacon['_drag_off'] = (event.x - mb['px'], event.y - mb['py'])

    def _man_beacon_drag_motion(self, event):
        off = self._man_beacon.get('_drag_off')
        if off is None:
            return
        dx, dy = off
        self._man_beacon['px'] = event.x - dx
        self._man_beacon['py'] = event.y - dy
        self._redraw_man_beacon()

    def _man_beacon_release(self, event):
        self._man_beacon['_drag_off'] = None

    def _man_beacon_rclick(self, event):
        if self._pan_did_drag:
            return
        mb = self._man_beacon
        if mb['px'] is None:
            return
        mb['rot'] = (mb['rot'] + 90) % 360
        self._redraw_man_beacon()

    # ── Balises auto ──────────────────────────────────────────────────────────

    def _ep_world_to_canvas(self, x_m, y_m):
        """Repère EP (x=avant/nord, y=droite/est) → coords canvas originales,
        ancré sur l'origine robot calibrée (self._robot_origin_px)."""
        ox, oy = self._robot_origin_px
        sc = CELL_PX / CELL_M
        return ox + y_m * sc, oy - x_m * sc

    def add_auto_beacon(self, wx_m: float, wy_m: float, facing_deg: float):
        """Appelé par le launcher quand [BEACONPOS] reçu (mode auto seulement).
        Déduplication : si une balise existe à moins de BEACON_DEDUP_M → mise à jour."""
        if not self._beacon_auto:
            return
        # Monde (repère EP) → coords canvas originales → coords canvas courantes
        x0, y0 = self._ep_world_to_canvas(wx_m, wy_m)
        px = x0 * self._zoom + self._ox
        py = y0 * self._zoom + self._oy

        # Déduplication dans l'espace original (invariant au zoom/pan)
        dedup_px = BEACON_DEDUP_M / CELL_M * CELL_PX
        for ab in self._auto_beacons:
            ab_x0 = (ab['px'] - self._ox) / self._zoom
            ab_y0 = (ab['py'] - self._oy) / self._zoom
            dist = math.hypot(ab_x0 - x0, ab_y0 - y0)
            if dist < dedup_px:
                ab['wx'], ab['wy'] = wx_m, wy_m
                ab['facing'] = facing_deg
                ab['px'], ab['py'] = px, py
                self._update_beacon_shape(ab['ids'], px, py, 0, facing_deg)
                return

        # Nouvelle balise
        ids = self._draw_beacon_shape(px, py, 0, facing_deg,
                                       BCOL_BEACON_AUTO, visible=True, tag="auto_beacon")
        self._auto_beacons.append({
            'wx': wx_m, 'wy': wy_m, 'facing': facing_deg,
            'px': px,   'py': py,   'ids': ids,
        })
        self._lbl_auto_n.config(text=f"{len(self._auto_beacons)} balise(s) auto")

    def _clear_auto_beacons(self):
        for ab in self._auto_beacons:
            for cid in ab['ids']:
                self._canvas.delete(cid)
        self._auto_beacons.clear()
        self._lbl_auto_n.config(text="0 balise(s) auto")

    # ── Géométrie blocs ───────────────────────────────────────────────────────

    def _px_to_cell(self, px, py):
        """Canvas coords → (row, col) ou (None, None) si hors grille."""
        x0 = (px - self._ox) / self._zoom
        y0 = (py - self._oy) / self._zoom
        c, r = int(x0) // CELL_PX, int(y0) // CELL_PX
        if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
            return r, c
        return None, None

    def _click_rot(self, btype, sx, sy):
        half = CELL_PX // 2
        if btype == 'full':
            return 0
        elif btype == 'half':
            dx = abs(sx - half)
            dy = abs(sy - half)
            if dx >= dy:
                return 270 if sx < half else 90
            return 0 if sy < half else 180
        else:
            top  = sy < half
            left = sx < half
            if top  and left:        return 0
            if top  and not left:    return 90
            if not top and not left: return 180
            return 270

    def _occupied(self, row, col, skip=None):
        result = set()
        for i, blk in enumerate(self._grid.get((row, col), [])):
            if i != skip:
                result |= blk['slots']
        return result

    def _find_block(self, px, py):
        """Canvas coords → (row, col, idx) du bloc sous le curseur, ou None."""
        r, c = self._px_to_cell(px, py)
        if r is None:
            return None
        HALF = CELL_PX // 2
        x0 = (px - self._ox) / self._zoom
        y0 = (py - self._oy) / self._zoom
        sx, sy = int(x0) % CELL_PX, int(y0) % CELL_PX
        slot = ('NW' if sx < HALF else 'NE') if sy < HALF else ('SW' if sx < HALF else 'SE')
        for i, blk in enumerate(self._grid.get((r, c), [])):
            if slot in blk['slots']:
                return (r, c, i)
        return None

    def _draw_block(self, row, col, btype, rot, ghost=False, raise_overlay=True):
        HALF = CELL_PX // 2
        # Coords dans l'espace original (zoom=1)
        ox, oy = col * CELL_PX, row * CELL_PX
        sl = _slots(btype, rot)
        ox1 = min(ox + _SLOT_OFF[s][0] * HALF for s in sl)
        oy1 = min(oy + _SLOT_OFF[s][1] * HALF for s in sl)
        ox2 = max(ox + (_SLOT_OFF[s][0] + 1) * HALF for s in sl)
        oy2 = max(oy + (_SLOT_OFF[s][1] + 1) * HALF for s in sl)
        # Conversion en coords canvas courantes
        x1 = ox1 * self._zoom + self._ox
        y1 = oy1 * self._zoom + self._oy
        x2 = ox2 * self._zoom + self._ox
        y2 = oy2 * self._zoom + self._oy
        if ghost:
            kw = dict(fill="", outline=ACCENT, width=2, dash=(5, 3))
        else:
            kw = dict(fill=BCOL[btype], outline=BCOL['outline'], width=1)
        ids = [self._canvas.create_rectangle(x1, y1, x2, y2, **kw)]
        if raise_overlay:
            self._canvas.tag_raise("overlay")
        return ids

    def _place(self, row, col, btype, rot, raise_overlay=True):
        sl = _slots(btype, rot)
        if sl & self._occupied(row, col):
            return False
        ids = self._draw_block(row, col, btype, rot, raise_overlay=raise_overlay)
        self._grid.setdefault((row, col), []).append(
            {'type': btype, 'rot': rot, 'slots': sl, 'ids': ids})
        self._hover_cell = None  # invalide le cache hover (case maintenant occupée)
        return True

    # ── Événements canvas ────────────────────────────────────────────────────

    def _on_click(self, event):
        px, py = event.x, event.y
        if self._tool == 'beacon':
            if not self._beacon_auto:
                self._place_man_beacon(px, py)
            return
        if self._tool == 'zone':
            r, c = self._px_to_cell(px, py)
            if r is not None:
                self._bulk_start = (r, c)
            return
        r, c = self._px_to_cell(px, py)
        if r is None:
            return
        if self._tool == 'erase':
            self._erase(px, py)
            return
        x0 = (px - self._ox) / self._zoom
        y0 = (py - self._oy) / self._zoom
        sx, sy = int(x0) % CELL_PX, int(y0) % CELL_PX
        self._place(r, c, self._tool, self._click_rot(self._tool, sx, sy))

    def _on_drag(self, event):
        if self._tool != 'zone' or self._bulk_start is None:
            return
        x_min = self._ox
        x_max = GRID_COLS * CELL_PX * self._zoom + self._ox - 1
        y_min = self._oy
        y_max = GRID_ROWS * CELL_PX * self._zoom + self._oy - 1
        px = max(x_min, min(event.x, x_max))
        py = max(y_min, min(event.y, y_max))
        r, c = self._px_to_cell(px, py)
        if r is None:
            return
        r0, c0 = self._bulk_start
        # Rect en coords canvas
        x1 = min(c, c0) * CELL_PX * self._zoom + self._ox
        y1 = min(r, r0) * CELL_PX * self._zoom + self._oy
        x2 = (max(c, c0) + 1) * CELL_PX * self._zoom + self._ox
        y2 = (max(r, r0) + 1) * CELL_PX * self._zoom + self._oy
        if self._bulk_rect_id:
            self._canvas.coords(self._bulk_rect_id, x1, y1, x2, y2)
        else:
            self._bulk_rect_id = self._canvas.create_rectangle(
                x1, y1, x2, y2, fill="", outline=ACCENT, width=2, dash=(4, 3))

    def _on_release(self, event):
        if self._tool != 'zone' or self._bulk_start is None:
            self._bulk_start = None
            return
        if self._bulk_rect_id:
            self._canvas.delete(self._bulk_rect_id)
            self._bulk_rect_id = None
        x_min = self._ox
        x_max = GRID_COLS * CELL_PX * self._zoom + self._ox - 1
        y_min = self._oy
        y_max = GRID_ROWS * CELL_PX * self._zoom + self._oy - 1
        px = max(x_min, min(event.x, x_max))
        py = max(y_min, min(event.y, y_max))
        r, c = self._px_to_cell(px, py)
        r0, c0 = self._bulk_start
        self._bulk_start = None
        if r is None:
            return
        for rr in range(min(r, r0), max(r, r0) + 1):
            for cc in range(min(c, c0), max(c, c0) + 1):
                if not self._occupied(rr, cc):
                    self._place(rr, cc, 'full', 0, raise_overlay=False)
        self._canvas.tag_raise("overlay")

    def _on_rclick(self, event):
        res = self._find_block(event.x, event.y)
        if res is None:
            return
        row, col, idx = res
        blk   = self._grid[(row, col)][idx]
        btype = blk['type']
        if btype == 'full':
            return
        occ = self._occupied(row, col, skip=idx)
        for delta in (90, 180, 270):
            new_rot = (blk['rot'] + delta) % 360
            new_sl  = _slots(btype, new_rot)
            if not (new_sl & occ):
                for cid in blk['ids']:
                    self._canvas.delete(cid)
                blk['rot']   = new_rot
                blk['slots'] = new_sl
                blk['ids']   = self._draw_block(row, col, btype, new_rot)
                return

    def _erase(self, px, py):
        res = self._find_block(px, py)
        if res is None:
            return
        row, col, idx = res
        blk = self._grid[(row, col)].pop(idx)
        for cid in blk['ids']:
            self._canvas.delete(cid)
        if not self._grid[(row, col)]:
            del self._grid[(row, col)]

    def _on_hover(self, event):
        if self._tool in ('erase', 'beacon', 'zone'):
            if self._hover_ids:
                for cid in self._hover_ids:
                    self._canvas.delete(cid)
                self._hover_ids.clear()
                self._hover_cell = None
            return
        r, c = self._px_to_cell(event.x, event.y)
        if r is None:
            if self._hover_ids:
                for cid in self._hover_ids:
                    self._canvas.delete(cid)
                self._hover_ids.clear()
                self._hover_cell = None
            return
        x0 = (event.x - self._ox) / self._zoom
        y0 = (event.y - self._oy) / self._zoom
        sx, sy = int(x0) % CELL_PX, int(y0) % CELL_PX
        rot = self._click_rot(self._tool, sx, sy)
        if (r, c, rot) == self._hover_cell:
            return  # même case et même rotation → rien à faire
        for cid in self._hover_ids:
            self._canvas.delete(cid)
        self._hover_ids.clear()
        self._hover_cell = None
        if not (_slots(self._tool, rot) & self._occupied(r, c)):
            self._hover_ids = self._draw_block(r, c, self._tool, rot, ghost=True)
            self._hover_cell = (r, c, rot)

    def _on_leave(self, event):
        for cid in self._hover_ids:
            self._canvas.delete(cid)
        self._hover_ids.clear()
        self._hover_cell = None

    # ── Zoom / Pan ───────────────────────────────────────────────────────────

    def _on_zoom(self, event):
        if event.num == 4 or (hasattr(event, 'delta') and event.delta > 0):
            factor = 1.15
        elif event.num == 5 or (hasattr(event, 'delta') and event.delta < 0):
            factor = 1 / 1.15
        else:
            return
        cx, cy = event.x, event.y
        self._canvas.scale("all", cx, cy, factor, factor)
        self._zoom *= factor
        self._ox = (self._ox - cx) * factor + cx
        self._oy = (self._oy - cy) * factor + cy

        def _sc(pos):
            return ((pos[0] - cx) * factor + cx, (pos[1] - cy) * factor + cy)

        self._robot_origin_px = _sc(self._robot_origin_px)
        if self._robot_px is not None:
            self._robot_px = _sc(self._robot_px)
        mb = self._man_beacon
        if mb['px'] is not None:
            mb['px'], mb['py'] = _sc((mb['px'], mb['py']))
        for ab in self._auto_beacons:
            ab['px'], ab['py'] = _sc((ab['px'], ab['py']))

    def _on_rclick_release(self, event):
        if not self._pan_did_drag:
            self._on_rclick(event)
        self._pan_last     = None
        self._pan_did_drag = False

    def _on_pan_start(self, event):
        self._pan_last     = (event.x, event.y)
        self._pan_did_drag = False

    def _on_pan_drag(self, event):
        if self._pan_last is None:
            return
        self._pan_did_drag = True
        dx = event.x - self._pan_last[0]
        dy = event.y - self._pan_last[1]
        self._pan_last = (event.x, event.y)
        self._canvas.move("all", dx, dy)
        self._ox += dx
        self._oy += dy
        if self._robot_px is not None:
            self._robot_px = (self._robot_px[0] + dx, self._robot_px[1] + dy)
        rpx, rpy = self._robot_origin_px
        self._robot_origin_px = (rpx + dx, rpy + dy)
        mb = self._man_beacon
        if mb['px'] is not None:
            mb['px'] += dx
            mb['py'] += dy
        for ab in self._auto_beacons:
            ab['px'] += dx
            ab['py'] += dy

    # ── Sauvegarde / chargement ───────────────────────────────────────────────

    def _clear_all(self):
        if not messagebox.askyesno("Effacer tout",
                                    "Effacer toute la map (blocs + balises) ?", parent=self):
            return
        for blks in self._grid.values():
            for blk in blks:
                for cid in blk['ids']:
                    self._canvas.delete(cid)
        self._grid.clear()
        self._clear_auto_beacons()
        # Masquer balise manuelle
        for cid in self._man_beacon['ids']:
            self._canvas.itemconfig(cid, state="hidden")
        self._man_beacon['px'] = None

    def _save(self):
        path = filedialog.asksaveasfilename(
            parent=self, defaultextension=".json",
            filetypes=[("Map JSON", "*.json"), ("Tous", "*.*")],
            initialdir=os.path.expanduser("~/Desktop"),
            title="Sauvegarder la map")
        if not path:
            return
        mb = self._man_beacon
        beacon_man = None
        if mb['px'] is not None:
            # Canvas → coords originales → monde
            bx0 = (mb['px'] - self._ox) / self._zoom
            by0 = (mb['py'] - self._oy) / self._zoom
            cx0 = GRID_COLS * CELL_PX / 2
            cy0 = GRID_ROWS * CELL_PX / 2
            sc  = CELL_PX / CELL_M
            beacon_man = {
                'wx': (bx0 - cx0) / sc,
                'wy': (cy0 - by0) / sc,
                'rot': mb['rot'],
            }
        data = {
            "version": 3, "cell_m": CELL_M,
            "cols": GRID_COLS, "rows": GRID_ROWS,
            "blocks": [
                {"row": r, "col": c, "type": blk['type'], "rot": blk['rot']}
                for (r, c), blks in self._grid.items()
                for blk in blks
            ],
            "beacon_man": beacon_man,
            "beacons_auto": [
                {"wx": ab['wx'], "wy": ab['wy'], "facing": ab['facing']}
                for ab in self._auto_beacons
            ],
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2)
        self.title(f"Éditeur Map — {os.path.basename(path)}")

    def _load(self):
        path = filedialog.askopenfilename(
            parent=self,
            filetypes=[("Map JSON", "*.json"), ("Tous", "*.*")],
            initialdir=os.path.expanduser("~/Desktop"),
            title="Charger une map")
        if not path:
            return
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception as e:
            messagebox.showerror("Erreur", f"Lecture impossible :\n{e}", parent=self)
            return
        # Blocs
        for blks in self._grid.values():
            for blk in blks:
                for cid in blk['ids']:
                    self._canvas.delete(cid)
        self._grid.clear()
        for b in data.get("blocks", []):
            try:
                r, c = int(b['row']), int(b['col'])
                if 0 <= r < GRID_ROWS and 0 <= c < GRID_COLS:
                    self._place(r, c, b['type'], int(b['rot']))
            except Exception:
                pass
        # Balise manuelle
        bm = data.get("beacon_man")
        if bm:
            # Monde → coords originales → canvas courantes
            x0, y0 = _world_to_canvas_orig(bm['wx'], bm['wy'])
            px = x0 * self._zoom + self._ox
            py = y0 * self._zoom + self._oy
            self._man_beacon['rot'] = bm.get('rot', 0)
            self._place_man_beacon(px, py)
        # Balises auto
        self._clear_auto_beacons()
        for ab in data.get("beacons_auto", []):
            try:
                self.add_auto_beacon(float(ab['wx']), float(ab['wy']),
                                      float(ab.get('facing', 0)))
            except Exception:
                pass
        # Activer mode auto si des balises auto ont été chargées
        if self._auto_beacons:
            self._set_beacon_mode(True)
        self.title(f"Éditeur Map — {os.path.basename(path)}")

    # ── Mises à jour live (appelées par le launcher) ──────────────────────────

    def _draw_robot_overlay(self, px, py, yaw_deg=0.0):
        """px, py en coords canvas courantes."""
        HALF = (CELL_PX // 2 - 2) * self._zoom
        self._canvas.coords(self._robot_id, px - HALF, py - HALF, px + HALF, py + HALF)
        yr = math.radians(yaw_deg)
        tip_x = px + math.sin(yr) * CELL_PX * self._zoom
        tip_y = py - math.cos(yr) * CELL_PX * self._zoom
        self._canvas.coords(self._robot_arrow_id, px, py, tip_x, tip_y)
        self._canvas.tag_raise("overlay")

    def update_robot(self, x_m, y_m, yaw_deg=0.0):
        ox, oy = self._robot_origin_px          # coords canvas courantes
        sc = CELL_PX * self._zoom / CELL_M
        px = ox + y_m * sc   # EP y+ = droite (est) → canvas px augmente
        py = oy - x_m * sc   # EP x+ = avant (nord) → canvas py diminue
        self._robot_px = (px, py)
        self._draw_robot_overlay(px, py, yaw_deg)

    def update_beacon(self, wx_m, wy_m):
        """Indicateur temporaire (balise live), position monde absolue (repère EP)."""
        ox, oy = self._robot_origin_px
        sc = CELL_PX * self._zoom / CELL_M
        bx = ox + wy_m * sc
        by = oy - wx_m * sc
        R = 6 * self._zoom
        self._canvas.coords(self._beacon_id, bx - R, by - R, bx + R, by + R)
        self._canvas.itemconfig(self._beacon_id, state="normal")
        self._canvas.tag_raise("overlay")

    def reset_overlay(self):
        """Remet le robot à son origine, cache balise live. Ne touche PAS aux balises auto."""
        self._robot_px = None
        ox, oy = self._robot_origin_px
        self._draw_robot_overlay(ox, oy)
        self._canvas.itemconfig(self._beacon_id, state="hidden")


if __name__ == "__main__":
    print("map_editor.py est desactive : plus lance depuis carolus_launcher.py")
    print("depuis le 2026-08-10 (fonctionnalite carte live retiree). Voir le")
    print("docstring en tete de fichier. Le code reste present mais ne tourne pas.")
