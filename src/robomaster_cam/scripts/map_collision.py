#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Collision checking 2D sur la grille produite par MapEditor (shortcuts/map_editor.py).

API publique :
  mc = MapCollision("/home/ubuntu/carolus_map.json", inflation_cells=1)
  mc.loaded                                  -> bool
  mc.segment_is_free(x0, y0, x1, y1)        -> bool
  mc.lookahead_is_free(x, y, yaw_deg, dist) -> bool
  mc.point_in_zone(x, y, margin_m=0.5)      -> bool

Repère monde : origine (0,0) = position de démarrage du robot = centre de la grille.
  x+ = est  (col croissant)
  y+ = nord (row décroissant)
Toutes les coordonnées en mètres.

**BUG-046 corrigé le 2026-07-20 :** les dimensions de grille (cols/rows/cell_m) sont
désormais LUES DYNAMIQUEMENT depuis le JSON (champs `cols`, `rows`, `cell_m`, présents
depuis la v3 du schéma MapEditor), au lieu d'être codées en dur. Avant ce fix, les
constantes 20×15/0.40 restaient figées alors que la grille active fait 26×21 depuis le
2026-06-30 — ce qui décalait l'origine monde de ~1.2 m sur chaque axe et faisait vérifier
obstacles/géofencing au mauvais endroit. Les constantes ci-dessous ne servent plus que de
valeurs par défaut de repli (JSON absent ou champ manquant), pour rétro-compatibilité.
"""

import json
import math
import os

# Valeurs par défaut de repli uniquement (utilisées si le JSON est absent ou n'a pas
# les champs cell_m/cols/rows). En fonctionnement normal, les vraies dimensions sont
# lues depuis le JSON — voir _load().
DEFAULT_CELL_M    = 0.40
DEFAULT_GRID_COLS = 26
DEFAULT_GRID_ROWS = 21


def _bresenham(r0, c0, r1, c1):
    """Parcourt toutes les cellules d'un segment (r0,c0)→(r1,c1) par algo de Bresenham."""
    cells = []
    dr = abs(r1 - r0);  dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc; r += sr
        if e2 < dr:
            err += dr; c += sc
    return cells


class MapCollision:

    def __init__(self, json_path=None, inflation_cells=1):
        # Dimensions par défaut (repli). Écrasées par _load() si le JSON les fournit.
        self.cell_m = DEFAULT_CELL_M
        self.cols   = DEFAULT_GRID_COLS
        self.rows   = DEFAULT_GRID_ROWS
        self._recompute_origin()
        self._occ      = [[False] * self.cols for _ in range(self.rows)]
        self._inflated = [[False] * self.cols for _ in range(self.rows)]
        self._loaded   = False
        if json_path and os.path.exists(json_path):
            self._load(json_path, inflation_cells)

    def _recompute_origin(self):
        # Origine monde (0,0) = centre géométrique de la grille.
        self._cx0_m = self.cols / 2.0 * self.cell_m   # demi-largeur (m)
        self._cy0_m = self.rows / 2.0 * self.cell_m   # demi-hauteur (m)

    def _load(self, path, inflation):
        try:
            with open(path) as f:
                data = json.load(f)
        except Exception:
            return

        # BUG-046 : lire les dimensions réelles depuis le JSON (schéma v3+).
        # Repli sur les valeurs par défaut si un champ manque (rétro-compat schéma ancien).
        try:
            self.cell_m = float(data.get('cell_m', self.cell_m))
            self.cols   = int(data.get('cols', self.cols))
            self.rows   = int(data.get('rows', self.rows))
        except Exception:
            pass  # garde les valeurs par défaut si le JSON est malformé sur ces champs
        self._recompute_origin()

        # (Ré)allouer les grilles à la bonne taille maintenant que cols/rows sont connus.
        self._occ      = [[False] * self.cols for _ in range(self.rows)]
        self._inflated = [[False] * self.cols for _ in range(self.rows)]

        for b in data.get('blocks', []):
            try:
                r, c = int(b['row']), int(b['col'])
                if 0 <= r < self.rows and 0 <= c < self.cols:
                    self._occ[r][c] = True
            except Exception:
                pass
        # Inflation pré-calculée (une fois au démarrage)
        for r in range(self.rows):
            for c in range(self.cols):
                if self._occ[r][c]:
                    for dr in range(-inflation, inflation + 1):
                        for dc in range(-inflation, inflation + 1):
                            nr, nc = r + dr, c + dc
                            if 0 <= nr < self.rows and 0 <= nc < self.cols:
                                self._inflated[nr][nc] = True
        self._loaded = True

    @property
    def loaded(self):
        return self._loaded

    def _world_to_cell(self, x_m, y_m):
        col = int((x_m + self._cx0_m) / self.cell_m)
        row = int((self._cy0_m - y_m) / self.cell_m)
        return row, col

    def _cell_blocked(self, row, col):
        if not (0 <= row < self.rows and 0 <= col < self.cols):
            return True   # hors grille = bloqué
        return self._inflated[row][col]

    def segment_is_free(self, x0_m, y0_m, x1_m, y1_m):
        """True si le segment monde (x0,y0)→(x1,y1) ne traverse aucune cellule bloquée."""
        r0, c0 = self._world_to_cell(x0_m, y0_m)
        r1, c1 = self._world_to_cell(x1_m, y1_m)
        for r, c in _bresenham(r0, c0, r1, c1):
            if self._cell_blocked(r, c):
                return False
        return True

    def lookahead_is_free(self, x_m, y_m, yaw_deg, dist_m):
        """True si la projection à dist_m devant le robot est libre.
        yaw_deg : cap chassis en degrés (convention EP : 0 = nord/y+, +90 = est/x+)."""
        rad = math.radians(yaw_deg)
        tx  = x_m + dist_m * math.sin(rad)
        ty  = y_m + dist_m * math.cos(rad)
        return self.segment_is_free(x_m, y_m, tx, ty)

    def point_in_zone(self, x_m, y_m, margin_m=0.5):
        """True si (x,y) est dans la zone de jeu (grille entière moins marge)."""
        hw = self._cx0_m - margin_m
        hh = self._cy0_m - margin_m
        return -hw <= x_m <= hw and -hh <= y_m <= hh
