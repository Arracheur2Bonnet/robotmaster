#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pipeline de pointage nacelle a priori (carte + odometrie) vers une balise connue.

Concept : etage GROSSIER d'un
pointage nacelle en deux temps, complementaire du LOCK BALISE deja code dans
`rm_cam_beacon.py` (etage FIN, vision seule) :

  1. GROSSIER (ce module) : le robot connait la position de la balise (carte)
     et sa position estimee (odometrie/EKF, meme si elle derive). On calcule
     l'angle absolu robot->balise par trigonometrie pure, et on sert la
     nacelle vers cet angle -- SANS avoir besoin de la voir au prealable.
  2. FIN (LOCK BALISE, `rm_cam_beacon.py::_gimbal_lock_servo`) : des que
     Carolus detecte visuellement la balise (`/pose` frais), le lock visuel
     prend le relais et corrige l'erreur residuelle que l'etage 1 ne peut pas
     eliminer (l'odometrie derive, justement -- c'est pour ca qu'on a besoin
     de Carolus en premier lieu).

PAS ENCORE CABLE dans rm_cam_beacon.py -- deux prerequis manquants cote
projet, aucun des deux fourni par ce module :
  (a) une position robot fusionnee/recalee en continu (F3, EKF
      `robot_localization` -- `/odom` brut derive sans correction, ne suffit
      pas comme entree fiable a long terme) ;
  (b) une carte avec les positions de balises connues A L'AVANCE (aujourd'hui
      `map_editor.py`/`mapv1.json` n'enregistre les balises qu'APRES
      detection -- il faudrait un jeu de balises "verite terrain" pre-charge,
      cf. schema post-meetup d'Hector : "beacon positions known in advance").

Ce module ne depend d'AUCUN des deux : ce sont des fonctions de geometrie
pures, testables des aujourd'hui (voir le bloc `__main__`), pretes a etre
importees dans `rm_cam_beacon.py` le jour ou (a) et (b) existent.

⚠️ CONVENTION YAW NON CONFIRMEE SUR MATERIEL : ce module suppose une
convention yaw standard (0 deg = +x, sens positif = trigonometrique/CCW),
coherente avec `math.atan2`. Le mapping exact entre le yaw brut EP
(`sub_attitude`) et cette convention n'est PAS confirme sur le robot --
meme caveat deja note pour `/odom` dans `rm_cam_beacon.py` ("signe EP non
confirme pour une orientation absolue"). A verifier avant tout branchement
reel (tourner le robot de 90 deg connus, comparer au yaw lu).
"""

import math


def _wrap_deg(angle_deg: float) -> float:
    """Ramene un angle dans [-180, 180)."""
    return (angle_deg + 180.0) % 360.0 - 180.0


def bearing_to_beacon_deg(robot_x: float, robot_y: float,
                           beacon_x: float, beacon_y: float) -> float:
    """Cap absolu (repere carte, convention atan2 standard) du robot vers la
    balise. Pure trigonometrie -- ne depend d'aucune mesure visuelle, marche
    meme si la balise n'a jamais ete detectee."""
    return math.degrees(math.atan2(beacon_y - robot_y, beacon_x - robot_x))


def gimbal_aim_error_deg(bearing_deg: float, gimbal_yaw_ground_deg: float) -> float:
    """Erreur angulaire signee (repere monde, dans [-180,180]) entre le cap
    requis vers la balise et le cap absolu courant de la nacelle
    (`/carolus/gimbal_yaw_ground`, deja publie par `rm_cam_beacon.py` depuis
    2026-07-22 -- la nacelle EP est stabilisee inertiellement, donc ce cap
    reste fiable independamment de la rotation du chassis)."""
    return _wrap_deg(bearing_deg - gimbal_yaw_ground_deg)


def gimbal_aim_yaw_speed(error_deg: float, k_gain: float, max_speed: float,
                          deadband_deg: float = 0.0) -> float:
    """Convertit une erreur angulaire en consigne de vitesse nacelle (deg/s).
    Meme forme que `rm_cam_beacon.py::_gimbal_servo_yaw` (gain proportionnel +
    clamp) pour reutiliser directement `drive_speed(pitch_speed=0,
    yaw_speed=...)` le jour du branchement."""
    if abs(error_deg) < deadband_deg:
        return 0.0
    return max(-max_speed, min(max_speed, k_gain * error_deg))


def nearest_known_beacon(robot_x: float, robot_y: float, beacons: list):
    """Selectionne la balise connue la plus proche parmi une liste de dicts
    `{'x':.., 'y':.., ...}`. Selection minimale (distance euclidienne, pas
    d'ordonnancement de trajet) -- volontairement simple pour ce premier jet,
    a complexifier seulement si un vrai planificateur de chemin l'exige."""
    if not beacons:
        return None
    return min(beacons, key=lambda b: math.hypot(b['x'] - robot_x, b['y'] - robot_y))


if __name__ == "__main__":
    # Auto-test (aucune dependance ROS/materiel) : simule le servo de gimbal
    # qui traque en continu le cap vers une balise fixe pendant que le robot
    # avance en ligne droite avec un offset lateral -- exactement le scenario
    # physique de F2 ("aller simple, une balise"). A chaque tick : on calcule
    # l'erreur, on applique une consigne de vitesse, on integre (dt=0.1s) pour
    # approcher le cap gimbal vers le cap requis -- on verifie que l'erreur de
    # suivi (PAS le cap final, qui varie avec la geometrie) converge vers 0 a
    # chaque tick, ce qui EST la definition d'un asservissement continu.
    beacon = (2.0, 1.2)          # balise fixe, position connue a l'avance
    gimbal_yaw_ground = 0.0      # nacelle initialement plein +x (arbitraire)
    K_GAIN, MAX_SPEED, DT = 2.0, 50.0, 0.1

    print(f"{'t':>4} {'x_robot':>8} {'bearing':>9} {'gimbal':>8} {'err_avant':>10} {'err_apres':>10}")
    N_SETTLE = 10   # ticks de transitoire tolerés (rattrapage de l'ecart initial)
    steady_errs = []
    for i in range(30):
        t = i * DT
        rx, ry = t * 0.3, 0.0        # robot avance le long de +x, ligne droite
        bearing = bearing_to_beacon_deg(rx, ry, *beacon)
        err_before = gimbal_aim_error_deg(bearing, gimbal_yaw_ground)
        yaw_speed = gimbal_aim_yaw_speed(err_before, K_GAIN, MAX_SPEED)
        gimbal_yaw_ground = _wrap_deg(gimbal_yaw_ground + yaw_speed * DT)
        err_after = gimbal_aim_error_deg(bearing, gimbal_yaw_ground)
        if i % 5 == 0:
            print(f"{t:4.1f} {rx:8.2f} {bearing:9.2f} {gimbal_yaw_ground:8.2f} "
                  f"{err_before:10.2f} {err_after:10.2f}")
        if i >= N_SETTLE:
            steady_errs.append(abs(err_after))

    # Le cap requis (bearing) change en continu car le robot avance -- ce
    # n'est PAS lui qui doit tomber a 0 (il balaie vers l'abeam a mesure que
    # le robot passe devant la balise, comportement geometrique normal en
    # "aller simple"). Ce qui doit converger, c'est l'erreur de POURSUITE
    # (err_after) : apres un transitoire initial (rattrapage de l'ecart de
    # depart, normal pour tout servo -- ignore ici via N_SETTLE), l'erreur de
    # regime etabli doit rester petite et bornee (retard de poursuite propre
    # a un P-pur face a une cible qui bouge, pas une divergence).
    max_steady_err = max(steady_errs)
    assert max_steady_err < 5.0, f"le servo ne suit pas la cible en regime etabli (err max {max_steady_err:.1f} deg)"
    print(f"\nOK -- transitoire initial ignore ({N_SETTLE} ticks), "
          f"erreur de poursuite en regime etabli : max {max_steady_err:.2f} deg")
    print("Module autonome, pas encore cable dans rm_cam_beacon.py (voir le "
          "docstring en tete de fichier pour le plan a deux etages).")
