#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Video helper and manual-command gateway for carolus_launcher.py.

Three roles:
  1. Subscribes to /camera/color/image_raw and writes a PNG thumbnail (~2 Hz) to OUT.
  2. Subscribes to /postprocessed/image (Carolus's own blob-detection view -- a
     black-and-white thresholded image with a coloured circle drawn on each detected
     LED blob, published by carolus_astrobee.cpp) and writes a second PNG thumbnail
     to OUT_BLOBS. Added 2026-08-14: this is the same view the project's inherited
     PDF (`Camera_Calibration_and_Carolus_node_for_Astrobee_simulation_HM_PK_VF.pdf`,
     p.16) showed through rviz -- here it is a second GUI panel instead, no rviz
     needed.
  3. Reads commands on stdin and publishes them on /carolus/mode, /carolus/cmd_vel, etc.

Usage: cam_view_helper.py [output_png_path] [output_blobs_png_path]
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
  BLOBS ON / BLOBS OFF -> subscribes/unsubscribes /postprocessed/image (OFF by default at startup)

Both thumbnails are plain resizes of their source topic, no overlay drawn on
either (the reticle/beacon-marker HUD that used to be drawn here was removed
2026-08-14 -- see journal.md for why).
"""

import os
import sys

# 2026-08-12 -- BUG-103, self-correcting environment guard. IDENTICAL in cause
# and fix to BUG-102 (capture_checkerboard.py, 2026-08-11); this file was simply
# never audited for it at the time, and the failure here is far more damaging
# because it is INVISIBLE:
#
# 1. A 2026-08-07 pip install (pandas/scikit-learn/scipy/tavily) put numpy 2.2.6
#    in ~/.local/lib/python3.10/site-packages, which SHADOWS the system numpy
#    1.21.5 that apt's python3-opencv (cv2) was compiled against. `import cv2`
#    below dies instantly with "AttributeError: _ARRAY_API not found".
# 2. Even past that, apt's cv2 (OpenCV 4.5.4) and ROS Noetic's system cv_bridge
#    (OpenCV 4.2) corrupt the heap at exit if both load in one process.
#
# Why this went unnoticed for five days: carolus_launcher.py starts this helper
# with stderr=subprocess.DEVNULL, so the traceback went nowhere, and the
# launcher logged "> Helper video lance" regardless. Since this helper is ALSO
# the stdin relay for every camera/gimbal command (GIMBAL, LOCK, RECENTER,
# MODE), its death silently disabled all of them -- the launcher kept logging
# each button press as if it had been delivered. Symptom seen on 2026-08-12:
# no camera preview, gimbal frozen and unable to be recentred onto the beacon,
# Carolus then finding ~1250 contours per frame and publishing no pose at all.
#
# Both fixes need a fresh interpreter (site-packages and env vars are read at
# startup), hence the one-shot re-exec.
if os.environ.get("_CAM_VIEW_HELPER_REEXEC") != "1":
    _ros_pkgs = "/opt/ros/noetic/lib/python3/dist-packages"
    _local_cv_bridge = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..", "mins_sandbox", "catkin_ws", "devel", "lib", "python3", "dist-packages",
    )
    _local_cv_bridge = os.path.normpath(_local_cv_bridge)
    env = dict(os.environ)
    env["PYTHONNOUSERSITE"] = "1"
    env["PYTHONPATH"] = os.pathsep.join(
        p for p in [_local_cv_bridge, _ros_pkgs, env.get("PYTHONPATH", "")] if p
    )
    env["_CAM_VIEW_HELPER_REEXEC"] = "1"
    os.execve(sys.executable, [sys.executable] + sys.argv, env)

import queue
import threading

import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from cv_bridge import CvBridge

OUT        = sys.argv[1] if len(sys.argv) > 1 else "/tmp/carolus_cam.png"
OUT_BLOBS  = sys.argv[2] if len(sys.argv) > 2 else "/tmp/carolus_blobs.png"
# Write rate of the GUI preview thumbnail (2026-07-22: 0.25 -> 0.05, i.e. 4 -> 20 Hz,
# matching the native camera stream). PURELY COSMETIC -- affects only the launcher's
# preview, never the /camera/color/image_raw stream Carolus consumes.
THROTTLE_S = 0.05
# 2026-08-14: bumped 320x180 -> 480x270 (1.5x each side, 2.25x the pixels) on
# request, for both thumbnails (raw + blobs share this one constant). Tradeoff:
# tk.PhotoImage decode was measured at 6.19 ms/frame at 320x180 (12.4% of the
# 50 ms/20 Hz GUI refresh budget, see carolus_launcher.py's _refresh_cam). At
# 2.25x the pixels, expect roughly 2.25x the decode time (~14 ms, ~28% of
# budget) -- still comfortable, but this is the knob to turn back down first
# if the preview ever visibly costs keyboard smoothness.
SIZE       = (480, 270)

bridge  = CvBridge()
_last   = [0.0]
_last_blobs = [0.0]
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
# Blob-detection view (2026-08-14): same on-demand pattern, same reasoning -- off
# by default, one fewer subscriber unless someone actually wants to see it.
_blob_sub    = None

# Queue for async PNG writes: ROS callback enqueues, dedicated thread writes to disk.
_png_queue: "queue.Queue" = queue.Queue(maxsize=2)
_png_queue_blobs: "queue.Queue" = queue.Queue(maxsize=2)


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
            rospy.logwarn_throttle(5.0, f"[CAMVIEW] PNG write error: {e}")


def _png_writer_blobs():
    """Same as _png_writer, for the /postprocessed/image thumbnail -- a separate
    thread and a separate tmp file so the two streams never race on one file."""
    tmp = os.path.join(os.path.dirname(OUT_BLOBS), "_carolus_blobs_tmp.png")
    while True:
        data = _png_queue_blobs.get()
        if data is None:
            break
        try:
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, OUT_BLOBS)
        except Exception as e:
            rospy.logwarn_throttle(5.0, f"[CAMVIEW] blobs PNG write error: {e}")


# -- video stream --------------------------------------------------------------

def cb_image(msg):
    now = rospy.get_time()
    if now - _last[0] < THROTTLE_S:
        return
    _last[0] = now
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, SIZE)
        ok, buf = cv2.imencode(".png", frame)
        if ok and not _png_queue.full():
            _png_queue.put_nowait(buf.tobytes())
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"[CAMVIEW] frame error: {e}")


def _set_camera_subscription(enable):
    """Subscribe to / unsubscribe from /camera/color/image_raw on demand. Idempotent."""
    global _cam_sub
    if enable and _cam_sub is None:
        _cam_sub = rospy.Subscriber("/camera/color/image_raw", Image, cb_image, queue_size=1)
        rospy.loginfo("[CAMVIEW] camera preview ON (subscribed)")
    elif not enable and _cam_sub is not None:
        _cam_sub.unregister()
        _cam_sub = None
        rospy.loginfo("[CAMVIEW] camera preview OFF (unsubscribed)")


def cb_blob_image(msg):
    """/postprocessed/image (carolus_astrobee.cpp): a BGR8 image built from the
    grayscale thresholded blob mask, with a coloured circle drawn on each detected
    LED -- black-and-white background, coloured markers on the blobs, exactly the
    view carolus_astrobee.cpp produces. No HUD drawn here: unlike cb_image, this
    thumbnail's whole purpose is to show Carolus's own detection output untouched,
    for tuning HSV/threshold values (see config/robomaster_s1.yaml)."""
    now = rospy.get_time()
    if now - _last_blobs[0] < THROTTLE_S:
        return
    _last_blobs[0] = now
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        frame = cv2.resize(frame, SIZE)
        ok, buf = cv2.imencode(".png", frame)
        if ok and not _png_queue_blobs.full():
            _png_queue_blobs.put_nowait(buf.tobytes())
    except Exception as e:
        rospy.logwarn_throttle(5.0, f"[CAMVIEW] blobs frame error: {e}")


def _set_blob_subscription(enable):
    """Subscribe to / unsubscribe from /postprocessed/image on demand. Idempotent."""
    global _blob_sub
    if enable and _blob_sub is None:
        _blob_sub = rospy.Subscriber("/postprocessed/image", Image, cb_blob_image, queue_size=1)
        rospy.loginfo("[CAMVIEW] blob view ON (subscribed)")
    elif not enable and _blob_sub is not None:
        _blob_sub.unregister()
        _blob_sub = None
        rospy.loginfo("[CAMVIEW] blob view OFF (unsubscribed)")


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
            elif line.startswith("BLOBS"):
                state = line.split()[1].upper()   # "ON" ou "OFF"
                _set_blob_subscription(state == "ON")
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
    threading.Thread(target=_png_writer_blobs, daemon=True).start()
    threading.Thread(target=_stdin_reader, daemon=True).start()

    rospy.loginfo(f"[CAMVIEW] ready -- thumbnail -> {OUT}")
    rospy.spin()


if __name__ == "__main__":
    main()
