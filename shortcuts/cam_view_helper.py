#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Helper vidéo + passerelle commandes manuelles pour carolus_launcher.py.

Deux rôles :
  1. S'abonne à /camera/color/image_raw, écrit une vignette PNG (~2 Hz) dans OUT.
  2. Lit des commandes sur stdin et les publie sur /carolus/mode et /carolus/cmd_vel.

Usage : cam_view_helper.py [chemin_png_sortie]
Commandes stdin (une par ligne) :
  MODE AUTO           → publie "AUTO" sur /carolus/mode
  MODE MANUAL         → publie "MANUAL" sur /carolus/mode
  VX 0.20 WZ 5.0     → publie Twist(linear.x, angular.z) sur /carolus/cmd_vel
  STOP                → publie Twist(0, 0) sur /carolus/cmd_vel
  GIMBAL pitch yaw    → publie Twist(angular.y, angular.z) sur /carolus/gimbal_vel
  WHEELS w1 w2 w3 w4  → publie "w1 w2 w3 w4" sur /carolus/wheels
  WHEELS STOP         → publie "STOP" sur /carolus/wheels
  LOCK ON / LOCK OFF  → publie "ON"/"OFF" sur /carolus/gimbal_lock (centrage periodique, cf. rm_cam_beacon.py)
  LOCKPERIOD 5.0      → publie "5.0" sur /carolus/gimbal_lock_period (periode en secondes, repli sur 2.0 si invalide)
  RECENTER            → publie "RECENTER" sur /carolus/gimbal_recenter (position de base camera)
  CAM ON / CAM OFF    → (dés)abonne /camera/color/image_raw (OFF par defaut au demarrage)

HUD (2026-07-23, reprojection precise ajoutee le 2026-07-23 (2)) : incruste sur la
vignette PNG (pas sur /camera/color/image_raw lui-meme, qui reste inchange pour
Carolus) -- reticule au centre image, marqueur balise (vert si centree, rouge sinon)
reprojete via les VRAIES intrinseques camera (camera_info.yaml, mises a l'echelle
1280x720->320x180), pas une estimation qualitative de direction.
"""

import os
import sys
import math
import queue
import threading

import yaml
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
from cv_bridge import CvBridge

OUT        = sys.argv[1] if len(sys.argv) > 1 else "/tmp/carolus_cam.png"
# Cadence d'ecriture de la vignette d'apercu GUI (2026-07-22 : 0.25->0.05, soit
# 4->20 Hz, au niveau du flux camera natif). PUREMENT COSMETIQUE — n'affecte que
# l'apercu du launcher, pas le flux /camera/color/image_raw que consomme Carolus.
THROTTLE_S = 0.05
SIZE       = (320, 180)

# HUD (2026-07-23)
POSE_FRESH_S     = 1.0    # au-dela, pose consideree perimee -> marqueur balise cache
HUD_CENTERED_DEG = 3.0    # sous ce seuil (les deux axes), marqueur vert plutot que rouge
# Intrinseques camera (2026-07-23 (2)) : lues depuis camera_info.yaml (pas dupliquees
# en dur -- evite un desaccord silencieux si le fichier est recalibre), mises a
# l'echelle pour le resize 320x180 de la vignette (facteur exact 1/4 sur les deux axes
# -- pas de distorsion d'aspect, 1280/320=720/180=4). Utilisees pour reprojeter le
# point 3D /pose (camera frame) en position pixel EXACTE, au lieu d'une direction
# qualitative. Repli sur les valeurs connues (2026-07-23) si le fichier est absent
# (ex. deploiement partiel) -- ne doit jamais faire planter le HUD.
_CAM_INFO_PATH = os.path.join(os.path.dirname(__file__), "..", "carolus_ws", "src",
                              "robomaster_cam", "config", "camera_info.yaml")
try:
    with open(_CAM_INFO_PATH) as _f:
        _cam_info = yaml.safe_load(_f)
    _cam_w = float(_cam_info["image_width"])
    _cam_m = _cam_info["camera_matrix"]["data"]
    _fx_full, _fy_full, _cx_full, _cy_full = _cam_m[0], _cam_m[4], _cam_m[2], _cam_m[5]
except Exception as _e:
    # print (pas rospy.logwarn) : ce bloc s'execute a l'import, avant rospy.init_node()
    # dans main() -- logguer via rospy trop tot n'est pas garanti fiable.
    print(f"[CAMVIEW] camera_info.yaml illisible ({_e}), repli sur intrinseques connues (2026-07-23)")
    _cam_w = 1280.0
    _fx_full, _fy_full, _cx_full, _cy_full = 546.1957, 547.0838, 575.6041, 372.1876
_CAM_SCALE = SIZE[0] / _cam_w
CAM_FX = _fx_full * _CAM_SCALE
CAM_FY = _fy_full * _CAM_SCALE
CAM_CX = _cx_full * _CAM_SCALE
CAM_CY = _cy_full * _CAM_SCALE
_last_pose = [None, 0.0]  # [PoseStamped ou None, timestamp rospy.get_time()]

bridge  = CvBridge()
_last   = [0.0]
_pub_mode    = None
_pub_cmdvel  = None
_pub_gimbal  = None
_pub_wheels  = None
_pub_lock    = None
_pub_lock_period = None
# Abonnement camera : cree/detruit a la demande (CAM ON/OFF), pas souscrit par
# defaut (2026-07-23) — evite de dupliquer un abonne sur /camera/color/image_raw,
# topic deja identifie comme goulot reseau (Perplexity 11) entre le Pi et Carolus.
_cam_sub     = None

# Queue for async PNG writes: ROS callback enqueues, dedicated thread writes to disk.
_png_queue: "queue.Queue" = queue.Queue(maxsize=2)


def _png_writer():
    """Dedicated thread: drains _png_queue and writes PNG atomically."""
    tmp = os.path.join(os.path.dirname(OUT), "_carolus_cam_tmp.png")
    while True:
        data = _png_queue.get()
        if data is None:
            break
        try:
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, OUT)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[CAMVIEW] erreur ecriture PNG: {e}")


# ── flux vidéo ────────────────────────────────────────────────────────────────

def cb_pose(msg):
    _last_pose[0] = msg
    _last_pose[1] = rospy.get_time()


def _draw_hud(frame):
    """Incruste le HUD (2026-07-23, reprojection precise ajoutee le 2026-07-23 (2))
    sur la vignette, en place. Convention Carolus (camera frame, avant remap ROS) :
    p.x=droite, p.y=bas, p.z=profondeur (negatif) -- cf. carolus_tf_broadcaster.py.
    Le marqueur balise est reprojete via les VRAIES intrinseques camera (pinhole,
    camera_info.yaml), pas une estimation qualitative -- position pixel exacte du
    centre P4P solve, pas juste une direction approximative.
    Pas de texte BEACON: DETECTED/LOST ici (2026-07-23 (2)) -- deja affiche par le
    voyant dedie du launcher, redondant sur l'image.
    Reticule au CENTRE GEOMETRIQUE de l'image (2026-07-23 nuit, revert) -- le point
    principal reel decale le reticule du cadrage naturel de la vignette, ce qui rend
    le HUD illisible/casse visuellement (retour utilisateur). Le marqueur balise reste
    reprojete via les vraies intrinseques (precis), mais le reticule cadre l'image."""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2   # centre geometrique de l'image affichee
    cross_col = (200, 200, 200)
    L = 8
    cv2.line(frame, (cx - L, cy), (cx + L, cy), cross_col, 1)
    cv2.line(frame, (cx, cy - L), (cx, cy + L), cross_col, 1)
    # anneau de tolerance : rayon pixel correspondant a HUD_CENTERED_DEG (~zone centree)
    tol_px = int(CAM_FX * math.tan(math.radians(HUD_CENTERED_DEG)))
    cv2.circle(frame, (cx, cy), max(4, tol_px), (120, 120, 120), 1)

    pose, ts = _last_pose[0], _last_pose[1]
    fresh = pose is not None and (rospy.get_time() - ts) < POSE_FRESH_S
    if fresh:
        p = pose.pose.position
        if abs(p.z) > 0.05:
            z = abs(p.z)
            ox = int(CAM_FX * (p.x / z) + CAM_CX)
            oy = int(CAM_FY * (p.y / z) + CAM_CY)
            yaw_err_deg   = math.degrees(math.atan2(p.x, z))
            pitch_err_deg = math.degrees(math.atan2(p.y, z))
            centered = abs(yaw_err_deg) < HUD_CENTERED_DEG and abs(pitch_err_deg) < HUD_CENTERED_DEG
            marker_col = (0, 220, 0) if centered else (0, 0, 220)   # vert / rouge (BGR)
            if 0 <= ox < w and 0 <= oy < h:
                cv2.circle(frame, (ox, oy), 6, marker_col, -1)
            else:
                # balise reprojetee hors champ visible (pose extreme/aberrante) --
                # indication en bord de cadre plutot que rien du tout.
                ox_c, oy_c = max(4, min(w - 4, ox)), max(4, min(h - 4, oy))
                cv2.circle(frame, (ox_c, oy_c), 6, marker_col, 2)


def cb_image(msg):
    now = rospy.get_time()
    if now - _last[0] < THROTTLE_S:
        return
    _last[0] = now
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, SIZE)
        _draw_hud(frame)
        ok, buf = cv2.imencode(".png", frame)
        if ok and not _png_queue.full():
            _png_queue.put_nowait(buf.tobytes())
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"[CAMVIEW] erreur frame: {e}")


def _set_camera_subscription(enable):
    """(Dés)abonne /camera/color/image_raw a la demande. Idempotent."""
    global _cam_sub
    if enable and _cam_sub is None:
        _cam_sub = rospy.Subscriber("/camera/color/image_raw", Image, cb_image, queue_size=1)
        rospy.loginfo("[CAMVIEW] apercu camera ON (abonne)")
    elif not enable and _cam_sub is not None:
        _cam_sub.unregister()
        _cam_sub = None
        rospy.loginfo("[CAMVIEW] apercu camera OFF (desabonne)")


# ── lecteur stdin (thread daemon) ────────────────────────────────────────────

def _stdin_reader():
    for raw in sys.stdin:
        line = raw.strip()
        if not line:
            continue
        try:
            if line.startswith("MODE"):
                mode = line.split()[1].upper()   # "AUTO", "MANUAL" ou "LOCATE"
                msg = String(data=mode)
                _pub_mode.publish(msg)
            elif line.startswith("VX"):
                parts = line.split()             # VX 0.20 WZ 5.0
                vx = float(parts[1])
                wz = float(parts[3])
                t = Twist()
                t.linear.x  = vx
                t.angular.z = wz
                _pub_cmdvel.publish(t)
            elif line == "STOP":
                _pub_cmdvel.publish(Twist())
            elif line.startswith("GIMBAL"):
                parts = line.split()
                t = Twist()
                t.angular.y = float(parts[1])   # pitch speed (deg/s)
                t.angular.z = float(parts[2])   # yaw speed (deg/s)
                _pub_gimbal.publish(t)
            elif line.startswith("WHEELS"):
                payload = line[len("WHEELS"):].strip()   # "w1 w2 w3 w4" ou "STOP"
                _pub_wheels.publish(String(data=payload))
            elif line.startswith("LOCKPERIOD"):
                # AVANT "LOCK" ci-dessous : "LOCKPERIOD 5.0".startswith("LOCK") est aussi
                # vrai, l'ordre des elif compte pour ne pas partir sur le mauvais topic.
                value = line.split()[1]   # ex. "5.0" -- validation faite cote rm_cam_beacon.py
                _pub_lock_period.publish(String(data=value))
            elif line.startswith("LOCK"):
                state = line.split()[1].upper()   # "ON" ou "OFF"
                _pub_lock.publish(String(data=state))
            elif line.startswith("CAM"):
                state = line.split()[1].upper()   # "ON" ou "OFF"
                _set_camera_subscription(state == "ON")
            elif line == "RECENTER":
                _pub_gimbal_recenter.publish(String(data="RECENTER"))
        except Exception as e:
            rospy.logwarn(f"[CAMVIEW] commande mal formee ({line!r}): {e}")


# ── main ─────────────────────────────────────────────────────────────────────

def main():
    global _pub_mode, _pub_cmdvel, _pub_gimbal, _pub_wheels, _pub_lock, _pub_lock_period
    global _pub_gimbal_recenter

    rospy.init_node("carolus_gui_cam", anonymous=True, disable_signals=True)

    _pub_mode   = rospy.Publisher("/carolus/mode",        String, queue_size=1, latch=True)
    _pub_cmdvel = rospy.Publisher("/carolus/cmd_vel",     Twist,  queue_size=1)
    _pub_gimbal = rospy.Publisher("/carolus/gimbal_vel",  Twist,  queue_size=1)
    _pub_wheels = rospy.Publisher("/carolus/wheels",      String, queue_size=1)
    # Pas de latch (contrairement a /carolus/mode) : le lock balise est un flag de
    # securite dont le defaut sur (OFF) vit deja cote rm_cam_beacon.py. Latcher "ON"
    # ferait heriter un noeud qui redemarre d'un auto-suivi actif sans action user —
    # on prefere qu'un nouveau noeud reparte OFF et attende un clic explicite.
    _pub_lock   = rospy.Publisher("/carolus/gimbal_lock", String, queue_size=1)
    _pub_lock_period = rospy.Publisher("/carolus/gimbal_lock_period", String, queue_size=1)
    # RECENTRER CAM (2026-07-23) : pas de latch, action ponctuelle (pas un etat
    # persistant a rejouer au redemarrage d'un noeud).
    _pub_gimbal_recenter = rospy.Publisher("/carolus/gimbal_recenter", String, queue_size=1)

    # /pose : toujours souscrit (message leger, pas d'image) pour le HUD -- independant
    # du toggle CAM ON/OFF qui ne concerne que /camera/color/image_raw (le vrai goulot
    # reseau, Perplexity 11).
    rospy.Subscriber("/pose", PoseStamped, cb_pose, queue_size=1)

    # Pas d'abonnement camera par defaut (2026-07-23) : active a la demande via
    # la commande stdin "CAM ON" (bouton APERCU CAM du launcher, OFF par defaut).

    # Latch le mode initial MANUAL + lock balise OFF au démarrage (2026-07-22, securite
    # BUG-058) : demarrer en MANUAL evite tout mouvement autonome de la nacelle/chassis
    # tant que l'utilisateur ne le demande pas explicitement. Coherent avec le defaut
    # MANUAL cote rm_cam_beacon.py.
    rospy.sleep(0.3)
    _pub_mode.publish(String(data="MANUAL"))
    _pub_lock.publish(String(data="OFF"))

    threading.Thread(target=_png_writer, daemon=True).start()
    threading.Thread(target=_stdin_reader, daemon=True).start()

    rospy.loginfo(f"[CAMVIEW] pret — vignette -> {OUT}")
    rospy.spin()


if __name__ == "__main__":
    main()
