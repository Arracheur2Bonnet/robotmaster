#!/usr/bin/env python3
"""Capture checkerboard frames from /camera/color/image_raw for the MATLAB
Camera Calibration Toolbox procedure (technical.tex, Chapter "Camera
Calibration"). Runs on the lab PC; subscribes over the network to the
Pi-published camera topic -- same as cam_view_helper.py's pattern.

Usage:
    python3 shortcuts/capture_checkerboard.py [output_dir]

Live preview window. Press:
    s  -- save the current frame as a PNG into output_dir
    q  -- quit

Aim for >=15 saved frames, checkerboard tilted >45 degrees from the optical
axis in each one, varied position/orientation across the field of view,
occupying roughly 20-25% of the frame -- per the manual's own recommended
procedure.
"""

import os
import sys

# 2026-08-11 -- self-correcting environment guard, added after this script
# crashed on first real use with two stacked, unrelated problems on this lab
# PC:
#
# 1. A 2026-08-07 pip install (pandas/scikit-learn/scipy/tavily) put numpy
#    2.2.6 in ~/.local/lib/python3.10/site-packages, which SHADOWS the
#    system numpy 1.21.5 that apt's python3-opencv (cv2) was compiled
#    against -- "AttributeError: _ARRAY_API not found". Fix: PYTHONNOUSERSITE
#    so the interpreter never looks in ~/.local at all.
# 2. Even with that fixed, apt's cv2 (linked against OpenCV 4.5.4) and ROS
#    Noetic's system cv_bridge (linked against OpenCV 4.2, confirmed via
#    ldd) corrupt the heap at process exit if both load in one process --
#    the exact same class of bug as BUG-101 (MINS build, same day), just a
#    different pair of libraries hitting it. Fix: put the locally-rebuilt
#    cv_bridge from that fix (mins_sandbox/catkin_ws, resolves to the same
#    OpenCV 4.5.4 apt's cv2 uses) ahead of the system one on PYTHONPATH.
#
# Both fixes require a fresh interpreter (site-packages/env vars are read at
# startup), so this re-execs itself once with the corrected environment
# rather than asking the user to remember two flags every time.
if os.environ.get("_CAPTURE_CHECKERBOARD_REEXEC") != "1":
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
    env["_CAPTURE_CHECKERBOARD_REEXEC"] = "1"
    os.execve(sys.executable, [sys.executable] + sys.argv, env)

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

OUTPUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "data/checkerboard"
TOPIC = "/camera/color/image_raw"

bridge = CvBridge()
latest_frame = [None]
saved_count = [0]


def cb_image(msg):
    latest_frame[0] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    rospy.init_node("capture_checkerboard", anonymous=True)
    rospy.Subscriber(TOPIC, Image, cb_image, queue_size=1)

    print("Waiting for frames on %s ..." % TOPIC)
    print("s = save frame, q = quit. Target: >=15 frames, checkerboard")
    print("tilted >45deg from the optical axis, varied across the frame.")

    while not rospy.is_shutdown():
        frame = latest_frame[0]
        if frame is not None:
            display = frame.copy()
            cv2.putText(
                display,
                "saved: %d  (s=save, q=quit)" % saved_count[0],
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow("capture_checkerboard", display)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s") and frame is not None:
            saved_count[0] += 1
            path = os.path.join(OUTPUT_DIR, "checkerboard_%03d.png" % saved_count[0])
            cv2.imwrite(path, frame)
            print("saved %s (%d total)" % (path, saved_count[0]))

    cv2.destroyAllWindows()
    print("Done. %d frames saved to %s" % (saved_count[0], OUTPUT_DIR))


if __name__ == "__main__":
    main()
