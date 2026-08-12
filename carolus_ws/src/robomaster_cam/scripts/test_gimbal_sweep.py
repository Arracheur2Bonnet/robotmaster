#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Script de test SDK gimbal — ISOLE, ne touche pas rm_cam_beacon.py.
Objectif : lever 3 inconnues avant la refonte du mode SEARCH.
  1. recenter() / moveto() sont-ils bloquants ou async ?
  2. Format + frequence reelle du callback sub_angle sur RNDIS
  3. Reference du yaw apres resume() (boot = 0 ?)

ATTENTION : ce script FAIT BOUGER LE GIMBAL (pas le chassis).
Lancer le robot sur un support stable. Le chassis ne bougera PAS.

Usage (sur le Pi) :
  python3 test_gimbal_sweep.py
"""

import time
from robomaster import robot

# Buffer des angles recus via sub_angle
_angles = []
_sub_count = [0]
_t0_sub = [None]


def cb_angle(info):
    # info = (pitch_angle, yaw_angle, pitch_ground_angle, yaw_ground_angle)
    if _t0_sub[0] is None:
        _t0_sub[0] = time.time()
    _sub_count[0] += 1
    _angles.append((time.time(), info))


def main():
    print("=== TEST GIMBAL SWEEP SDK ===")
    print("[1] Connexion RNDIS...")
    ep = robot.Robot()
    ep.initialize(conn_type="rndis")
    gim = ep.gimbal
    print("    Robot connecte")

    print("[2] resume() — activation couple")
    gim.resume()
    time.sleep(1.0)

    print("[3] Souscription sub_angle freq=10")
    gim.sub_angle(freq=10, callback=cb_angle)
    time.sleep(2.0)
    # Frequence reelle observee
    if _t0_sub[0]:
        dt = time.time() - _t0_sub[0]
        hz = _sub_count[0] / dt if dt > 0 else 0
        print(f"    Echantillons recus : {_sub_count[0]} en {dt:.2f}s -> {hz:.1f} Hz reel")
    if _angles:
        print(f"    Dernier angle (boot ref) : {_angles[-1][1]}")
        print(f"    => format = (pitch, yaw, pitch_ground, yaw_ground)")

    print("[4] Test moveto(yaw=+90) — mesure si BLOQUANT")
    t_start = time.time()
    gim.moveto(pitch=0, yaw=90, pitch_speed=0, yaw_speed=60)
    t_elapsed = time.time() - t_start
    print(f"    moveto a rendu la main en {t_elapsed:.2f}s")
    if t_elapsed > 0.5:
        print("    => moveto est BLOQUANT (attend la fin du mouvement)")
    else:
        print("    => moveto est ASYNC (rend la main immediatement)")
    time.sleep(2.0)
    if _angles:
        print(f"    yaw apres moveto(+90) : {_angles[-1][1][1]} (relatif) / {_angles[-1][1][3]} (ground)")

    print("[5] Test sweep drive_speed(yaw=40) pendant 3s en lisant sub_angle")
    yaw_start = _angles[-1][1][3] if _angles else 0
    print(f"    yaw_ground depart : {yaw_start}")
    t_start = time.time()
    while time.time() - t_start < 3.0:
        gim.drive_speed(pitch_speed=0, yaw_speed=40)  # envoi continu obligatoire
        time.sleep(0.05)
    gim.drive_speed(pitch_speed=0, yaw_speed=0)
    yaw_end = _angles[-1][1][3] if _angles else 0
    swept = yaw_end - yaw_start
    print(f"    yaw_ground fin : {yaw_end}  -> balaye {swept:.0f}° en 3s (theorique 120°)")
    print(f"    => vitesse reelle ~{swept/3:.0f}°/s (consigne 40°/s)")

    print("[6] Test recenter() — mesure si BLOQUANT")
    t_start = time.time()
    gim.recenter(pitch_speed=0, yaw_speed=60)
    t_elapsed = time.time() - t_start
    print(f"    recenter a rendu la main en {t_elapsed:.2f}s")
    if t_elapsed > 0.5:
        print("    => recenter est BLOQUANT")
    else:
        print("    => recenter est ASYNC")
    time.sleep(3.0)
    if _angles:
        print(f"    yaw_ground apres recenter : {_angles[-1][1][3]} (attendu ~0)")

    print("[7] Limite mecanique : moveto(yaw=+250) puis moveto(yaw=-250)")
    for target in (250, -250):
        t_start = time.time()
        gim.moveto(pitch=0, yaw=target, pitch_speed=0, yaw_speed=80)
        print(f"    moveto({target}) rendu en {time.time()-t_start:.2f}s", end="")
        time.sleep(4.0)
        actual = _angles[-1][1][3] if _angles else "?"
        print(f"  -> yaw_ground atteint : {actual}")

    print("[8] Nettoyage")
    gim.unsub_angle()
    gim.recenter(pitch_speed=0, yaw_speed=60)
    time.sleep(3.0)
    ep.close()
    print("=== FIN TEST ===")


if __name__ == "__main__":
    main()
