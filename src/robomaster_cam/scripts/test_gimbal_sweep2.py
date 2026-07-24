#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test SDK v2 — ISOLE, ne touche pas rm_cam_beacon.py.
Leve 2 hypotheses critiques avant la refonte SEARCH/APPROACH.

  TEST A : yaw vs yaw_ground quand le CHASSIS tourne (gimbal immobile).
           -> determine quel repere utiliser pour l'asservissement APPROACH.
  TEST B : sequence SEARCH reelle = drive_speed(sweep) -> stop ->
           recenter().wait_for_completed() -> drive_speed a nouveau.
           -> valide qu'il n'y a pas de conflit action/vitesse.

ATTENTION : LE CHASSIS VA TOURNER (Test A) + LE GIMBAL VA BOUGER (Test B).
ROUES SURELEVEES recommandees (robot sur un support, roues dans le vide).

Usage (sur le Pi) : python3 test_gimbal_sweep2.py
"""

import time
from robomaster import robot

_last = [None]   # dernier (pitch, yaw, pitch_ground, yaw_ground)


def cb_angle(info):
    _last[0] = info


def yaw_rel():
    return _last[0][1] if _last[0] else None


def yaw_gnd():
    return _last[0][3] if _last[0] else None


def main():
    print("=== TEST SDK v2 (repere + sequence SEARCH) ===")
    ep = robot.Robot()
    ep.initialize(conn_type="rndis")
    print("Robot connecte")
    gim = ep.gimbal
    chassis = ep.chassis
    gim.resume()
    gim.sub_angle(freq=10, callback=cb_angle)
    time.sleep(1.5)
    print(f"Angle initial : yaw_rel={yaw_rel()}  yaw_ground={yaw_gnd()}")

    # ---------------------------------------------------------------
    print("\n--- TEST A : chassis tourne, gimbal IMMOBILE (pas de cmd gimbal) ---")
    print("Le chassis tourne ~3s. On observe yaw_rel et yaw_ground du gimbal.")
    yr0, yg0 = yaw_rel(), yaw_gnd()
    t = time.time()
    while time.time() - t < 3.0:
        chassis.drive_speed(x=0, y=0, z=30, timeout=1)   # rotation chassis 30 (deg/s consigne)
        time.sleep(0.05)
    chassis.drive_speed(x=0, y=0, z=0, timeout=1)
    time.sleep(1.0)
    yr1, yg1 = yaw_rel(), yaw_gnd()
    print(f"  AVANT  : yaw_rel={yr0:.1f}  yaw_ground={yg0:.1f}")
    print(f"  APRES  : yaw_rel={yr1:.1f}  yaw_ground={yg1:.1f}")
    print(f"  DELTA  : yaw_rel={yr1-yr0:+.1f}  yaw_ground={yg1-yg0:+.1f}")
    if abs(yr1 - yr0) > abs(yg1 - yg0) + 5:
        print("  => yaw_REL suit le chassis ; yaw_GROUND est STABILISE monde")
        print("     (le gimbal compense la rotation chassis = stabilise inertiellement)")
    elif abs(yg1 - yg0) > abs(yr1 - yr0) + 5:
        print("  => yaw_GROUND suit le chassis ; yaw_REL constant")
        print("     (le gimbal est RIGIDE par rapport au chassis)")
    else:
        print("  => indetermine (rotation trop faible ou ambigue)")

    # ---------------------------------------------------------------
    print("\n--- TEST B : sequence SEARCH (drive_speed -> recenter -> drive_speed) ---")
    print("[B1] sweep drive_speed(yaw=20) 2s en lisant yaw_ground")
    yg_start = yaw_gnd()
    t = time.time()
    while time.time() - t < 2.0:
        gim.drive_speed(pitch_speed=0, yaw_speed=20)
        time.sleep(0.05)
    gim.drive_speed(pitch_speed=0, yaw_speed=0)
    time.sleep(0.5)
    print(f"     yaw_ground {yg_start:.1f} -> {yaw_gnd():.1f}")

    print("[B2] recenter().wait_for_completed(timeout=6)")
    try:
        act = gim.recenter(pitch_speed=0, yaw_speed=60)
        ok = act.wait_for_completed(timeout=6)
        print(f"     recenter completed={ok}  yaw_ground={yaw_gnd():.1f} (attendu ~0)")
    except Exception as e:
        print(f"     ERREUR recenter : {e}")

    print("[B3] re-sweep drive_speed(yaw=-20) 2s (verifie pas de conflit post-action)")
    try:
        yg_start = yaw_gnd()
        t = time.time()
        while time.time() - t < 2.0:
            gim.drive_speed(pitch_speed=0, yaw_speed=-20)
            time.sleep(0.05)
        gim.drive_speed(pitch_speed=0, yaw_speed=0)
        time.sleep(0.5)
        print(f"     yaw_ground {yg_start:.1f} -> {yaw_gnd():.1f}  => sequence OK, pas de conflit")
    except Exception as e:
        print(f"     ERREUR re-sweep : {e}")

    # ---------------------------------------------------------------
    print("\n[CLEANUP] recenter + close")
    try:
        gim.recenter(pitch_speed=0, yaw_speed=60).wait_for_completed(timeout=6)
    except Exception:
        pass
    gim.unsub_angle()
    ep.close()
    print("=== FIN TEST v2 ===")


if __name__ == "__main__":
    main()
