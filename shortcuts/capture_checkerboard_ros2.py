#!/usr/bin/env python3
"""Manual, keypress-triggered checkerboard capture for the ROS2/Logitech
C920 side (subscribes to /image_raw, usb_cam's topic) -- NOT the same
target as capture_checkerboard.py, which is ROS1 and subscribes to
/camera/color/image_raw (the RoboMaster S1's own onboard camera, published
by robomaster_cam). Two different physical cameras, two different scripts;
written 2026-09-02 after checking the existing tool and finding it can't
serve this camera (rospy/cv2/cv_bridge, wrong topic, GUI window needed --
none of which apply here).

Press Enter to save the CURRENT frame, move the board, press Enter again.
No timer, full control over when each shot happens.

Saves as PPM (P6) -- zero image-library dependencies (no PIL, no cv2),
matching this project's established numpy-only convention for quick
diagnostic tools (optical_drift_observer.py, beacon_intensity_monitor.py).
MATLAB's Camera Calibrator reads PPM natively.

Targets, for reference while capturing (Source: documentations/Hector/2
Camera calibration and UVGS-2 Deployment for Astrobee Simulation.pdf, p.7):
board fills 20-25% of frame, tilted >=45 deg off the optical axis, varied
positions across the field of view, minimum 10 usable images.

NOTE ON DISTORTION MODEL: unlike the RoboMaster's own camera (technical.tex
ch. "Camera Calibration", which recommends 3 radial coefficients for that
camera's wider-FOV lens), the Logitech C920 is a standard consumer webcam --
use MATLAB's default 2-coefficient radial model, matching
logitech_1080p.yaml's existing 4-slot `distortion: [k1, k2, p1, p2]` format.
Do not copy the RoboMaster camera's 3-radial setting here without re-deriving
it for this camera specifically.

Usage: capture_checkerboard_ros2.py [out_dir]
Run interactively -- this needs your own terminal's stdin, it will not
work piped through a non-interactive SSH command.
"""
import os
import sys
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "data/checkerboard/logitech_capture"

latest = {"img": None, "w": 0, "h": 0}
lock = threading.Lock()


class Capture(Node):
    def __init__(self):
        super().__init__("checkerboard_capture")
        self.create_subscription(
            Image, "/image_raw", self.cb, QoSPresetProfiles.SENSOR_DATA.value
        )

    def cb(self, msg):
        a = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = a.reshape(msg.height, msg.width, -1)
        except ValueError:
            return
        if img.shape[2] >= 3:
            img = img[:, :, :3]
        with lock:
            latest["img"] = np.ascontiguousarray(img)
            latest["w"] = msg.width
            latest["h"] = msg.height


def spin_thread(node):
    rclpy.spin(node)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    rclpy.init()
    node = Capture()
    t = threading.Thread(target=spin_thread, args=(node,), daemon=True)
    t.start()

    print(f"Saving to {OUT_DIR}.")
    print("Position the checkerboard, then press ENTER to save that exact frame.")
    print("Empty input (just ENTER) saves; type 'q' + ENTER to stop.\n")

    count = 0
    try:
        while True:
            line = input(f"[{count} saved] ENTER to capture, 'q' to quit: ").strip().lower()
            if line == "q":
                break
            with lock:
                img = latest["img"]
                w, h = latest["w"], latest["h"]
            if img is None:
                print("  no frame received yet -- is usb_cam running?")
                continue
            path = os.path.join(OUT_DIR, f"calib_{count:03d}.ppm")
            with open(path, "wb") as f:
                f.write(f"P6\n{w} {h}\n255\n".encode("ascii"))
                f.write(img.tobytes())
            count += 1
            print(f"  saved {path}")
    except (KeyboardInterrupt, EOFError):
        pass

    print(f"\nDone -- {count} images in {OUT_DIR}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
