#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Video helper and manual-command gateway for carolus_launcher.py.

Two roles:
  1. Subscribes to /camera/color/image_raw and writes a PNG thumbnail (~2 Hz) to OUT.
  2. Reads commands on stdin and publishes them on /carolus/mode, /carolus/cmd_vel, etc.

Usage: cam_view_helper.py [output_png_path]
stdin commands (one per line):
  MODE AUTO           -> publishes "AUTO" on /carolus/mode
  MODE MANUAL         -> publishes "MANUAL" on /carolus/mode
  VX 0.20 WZ 5.0      -> publishes Twist(linear.x, angular.z) on /carolus/cmd_vel
  STOP                -> publishes Twist(0, 0) on /carolus/cmd_vel
  GIMBAL pitch yaw    -> publishes Twist(angular.y, angular.z) on /carolus/gimbal_vel
  WHEELS w1 w2 w3 w4  -> publishes "w1 w2 w3 w4" on /carolus/wheels
  WHEELS STOP         -> publishes "STOP" on /carolus/wheels
  LOCK ON / LOCK OFF  -> publishes "ON"/"OFF" on /carolus/gimbal_lock (periodic re-centring, see rm_cam_beacon.py)
  LOCKPERIOD 5.0      -> publishes "5.0" on /carolus/gimbal_lock_period (seconds, falls back to 2.0 if invalid)
  RECENTER            -> publishes "RECENTER" on /carolus/gimbal_recenter (gimbal base position)
  CAM ON / CAM OFF    -> subscribes/unsubscribes /camera/color/image_raw (OFF by default at startup)

HUD (2026-07-23, exact reprojection added 2026-07-23 (2)): drawn on the PNG
thumbnail only, never on /camera/color/image_raw itself, which stays untouched for
Carolus -- a reticle at the image centre plus a beacon marker (green when centred,
red otherwise) reprojected through the REAL camera intrinsics (camera_info.yaml,
scaled 1280x720 -> 320x180), not a qualitative direction estimate.
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
# Write rate of the GUI preview thumbnail (2026-07-22: 0.25 -> 0.05, i.e. 4 -> 20 Hz,
# matching the native camera stream). PURELY COSMETIC -- affects only the launcher's
# preview, never the /camera/color/image_raw stream Carolus consumes.
THROTTLE_S = 0.05
SIZE       = (320, 180)

# HUD (2026-07-23)
POSE_FRESH_S     = 1.0    # au-dela, pose consideree perimee -> marqueur balise cache
HUD_CENTERED_DEG = 3.0    # below this threshold (both axes), green marker rather than red
# Camera intrinsics (2026-07-23 (2)): read from camera_info.yaml rather than
# duplicated as literals -- duplicating them would drift silently the day the file is
# recalibrated. Scaled for the thumbnail's 320x180 resize (exactly 1/4 on both axes,
# so no aspect distortion: 1280/320 = 720/180 = 4). Used to reproject the 3D /pose
# point (camera frame) to an EXACT pixel position instead of a qualitative direction.
# Falls back to the known 2026-07-23 values if the file is missing (e.g. a partial
# deployment) -- the HUD must never be what crashes.
_CAM_INFO_PATH = os.path.join(os.path.dirname(__file__), "..", "carolus_ws", "src",
                              "robomaster_cam", "config", "camera_info.yaml")
try:
    with open(_CAM_INFO_PATH) as _f:
        _cam_info = yaml.safe_load(_f)
    _cam_w = float(_cam_info["image_width"])
    _cam_m = _cam_info["camera_matrix"]["data"]
    _fx_full, _fy_full, _cx_full, _cy_full = _cam_m[0], _cam_m[4], _cam_m[2], _cam_m[5]
except Exception as _e:
    # print, not rospy.logwarn: this block runs at import time, before
    # rospy.init_node() in main() -- logging through rospy that early is not reliable.
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
# Camera subscription: created and destroyed on demand (CAM ON/OFF), not subscribed
# by default (2026-07-23) -- avoids a second subscriber on /camera/color/image_raw,
# a topic already identified as the network bottleneck between the Pi and Carolus.
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


# -- video stream --------------------------------------------------------------

def cb_pose(msg):
    _last_pose[0] = msg
    _last_pose[1] = rospy.get_time()


def _draw_hud(frame):
    """Draw the HUD (2026-07-23, exact reprojection added 2026-07-23 (2))
    on the thumbnail, in place. Carolus convention (camera frame, before the ROS remap):
    p.x=droite, p.y=bas, p.z=profondeur (negatif) -- cf. carolus_tf_broadcaster.py.
    The beacon marker is reprojected through the REAL camera intrinsics (pinhole,
    camera_info.yaml), not a qualitative estimate -- the exact pixel position of
    the P4P solve's centre, not merely an approximate direction.
    No BEACON: DETECTED/LOST text here (2026-07-23 (2)) -- already displayed by the
    launcher's own indicator, and redundant on the image.
    Reticle at the image's GEOMETRIC CENTRE (night of 2026-07-23, a revert) -- the
    real principal point shifts the reticle away from the thumbnail's natural framing,
    which made the HUD unreadable and visually broken (user feedback). The beacon
    marker is still reprojected through the real intrinsics (accurate), but the
    reticle frames the image."""
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2   # geometric centre of the displayed image
    cross_col = (200, 200, 200)
    L = 8
    cv2.line(frame, (cx - L, cy), (cx + L, cy), cross_col, 1)
    cv2.line(frame, (cx, cy - L), (cx, cy + L), cross_col, 1)
    # tolerance ring: the pixel radius corresponding to HUD_CENTERED_DEG (centred zone)
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
                # Beacon reprojected outside the visible frame (extreme or aberrant
                # pose) -- show an edge-of-frame indication rather than nothing.
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
    """Subscribe to / unsubscribe from /camera/color/image_raw on demand. Idempotent."""
    global _cam_sub
    if enable and _cam_sub is None:
        _cam_sub = rospy.Subscriber("/camera/color/image_raw", Image, cb_image, queue_size=1)
        rospy.loginfo("[CAMVIEW] apercu camera ON (abonne)")
    elif not enable and _cam_sub is not None:
        _cam_sub.unregister()
        _cam_sub = None
        rospy.loginfo("[CAMVIEW] apercu camera OFF (desabonne)")


# -- stdin reader (daemon thread) ----------------------------------------------

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
                # BEFORE the "LOCK" branch below: "LOCKPERIOD 5.0".startswith("LOCK")
                # is also true, so the elif order decides which topic is used.
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
            elif line.startswith("DOCK"):
                cmd = line.split()[1].upper()   # "START", "CALIBRATE" ou "ABORT"
                _pub_dock.publish(String(data=cmd))
        except Exception as e:
            rospy.logwarn(f"[CAMVIEW] commande mal formee ({line!r}): {e}")


# -- main ----------------------------------------------------------------------

def main():
    global _pub_mode, _pub_cmdvel, _pub_gimbal, _pub_wheels, _pub_lock, _pub_lock_period
    global _pub_gimbal_recenter, _pub_dock

    rospy.init_node("carolus_gui_cam", anonymous=True, disable_signals=True)

    _pub_mode   = rospy.Publisher("/carolus/mode",        String, queue_size=1, latch=True)
    _pub_cmdvel = rospy.Publisher("/carolus/cmd_vel",     Twist,  queue_size=1)
    _pub_gimbal = rospy.Publisher("/carolus/gimbal_vel",  Twist,  queue_size=1)
    _pub_wheels = rospy.Publisher("/carolus/wheels",      String, queue_size=1)
    # No latch here, unlike /carolus/mode: the beacon lock is a safety flag whose
    # safe default (OFF) already lives in rm_cam_beacon.py. Latching "ON" would make a
    # restarting node inherit active auto-tracking with no user action -- we prefer a
    # fresh node to come up OFF and wait for an explicit click.
    _pub_lock   = rospy.Publisher("/carolus/gimbal_lock", String, queue_size=1)
    _pub_lock_period = rospy.Publisher("/carolus/gimbal_lock_period", String, queue_size=1)
    # RECENTER CAM (2026-07-23): no latch. It is a one-shot action, not a persistent
    # state to be replayed when a node restarts.
    _pub_gimbal_recenter = rospy.Publisher("/carolus/gimbal_recenter", String, queue_size=1)
    # Docking (2026-07-27, beacon_docking.py): no latch, same reason as RECENTER --
    # a START/CALIBRATE/ABORT command must never be replayed automatically when a
    # node restarts.
    _pub_dock = rospy.Publisher("/carolus/dock", String, queue_size=1)

    # /pose: always subscribed (a light message, no image) for the HUD -- independent
    # of the CAM ON/OFF toggle, which only concerns /camera/color/image_raw, the actual
    # network bottleneck.
    rospy.Subscriber("/pose", PoseStamped, cb_pose, queue_size=1)

    # No camera subscription by default (2026-07-23): enabled on demand through the
    # "CAM ON" stdin command (the launcher's CAM PREVIEW button, OFF by default).

    # Latch the initial MANUAL mode and beacon lock OFF at startup (2026-07-22,
    # BUG-058 safety): starting in MANUAL prevents any autonomous gimbal or chassis
    # motion until the user explicitly asks for it. Consistent with rm_cam_beacon.py's
    # own MANUAL default.
    rospy.sleep(0.3)
    _pub_mode.publish(String(data="MANUAL"))
    _pub_lock.publish(String(data="OFF"))

    threading.Thread(target=_png_writer, daemon=True).start()
    threading.Thread(target=_stdin_reader, daemon=True).start()

    rospy.loginfo(f"[CAMVIEW] pret — vignette -> {OUT}")
    rospy.spin()


if __name__ == "__main__":
    main()
