#!/usr/bin/env python3
"""Live beacon-brightness meter, ROS2 port for the second 24.04 PC's carolus_ros2.

Ported 2026-08-25 from shortcuts/beacon_brightness_live.py (rospy, tuned for the
RoboMaster's own beacon). This machine's beacon is a DIFFERENT physical unit
(verified against known_points by tape measure 2026-08-24, not the same rig),
so the original TARGET_LO/HI saturated-pixel bands (calibrated on that other
beacon) are NOT reused here -- they would mislabel a working setting on a
different LED/camera pair exactly the way the tool's own history already
warns about. This version shows raw numbers and lets the DETECTION RATE speak
for itself, which is the rig-agnostic signal the original tool converged on
after its first version (pixel-count-only) was shown to be wrong.

Usage, over SSH to the second PC:
    ssh -i ~/.ssh/carolus_nopass <user>@<host>
    source /opt/ros/jazzy/setup.bash
    python3 beacon_brightness_live_ros2.py
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

SAT = 250  # a pixel this bright is saturated on at least one channel
RATE_WINDOW_S = 5.0


class BeaconBrightnessLive(Node):
    def __init__(self):
        super().__init__('beacon_brightness_live')
        self.sat = None
        self.hue = None
        self.frames = 0
        self.pose_times = []
        self.declare_parameter('image_topic', '/image_raw')
        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, image_topic, self.cb_img,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/pose', self.cb_pose, 10)

    def cb_img(self, msg):
        a = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = a.reshape(msg.height, msg.width, -1)
        except ValueError:
            return
        lum = img.max(axis=2)
        mask = lum > SAT
        n = int(mask.sum())
        self.sat = n
        self.frames += 1
        if n:
            b = float(img[:, :, 0][mask].mean())
            g = float(img[:, :, 1][mask].mean())
            r = float(img[:, :, 2][mask].mean())
            mx, mn = max(b, g, r), min(b, g, r)
            if mx == mn:
                h = 0.0
            elif mx == r:
                h = (60 * ((g - b) / (mx - mn)) + 360) % 360
            elif mx == g:
                h = 60 * ((b - r) / (mx - mn)) + 120
            else:
                h = 60 * ((r - g) / (mx - mn)) + 240
            self.hue = h / 2.0  # OpenCV 0-180 scale, as the config uses
        else:
            self.hue = None

    def cb_pose(self, _msg):
        now = time.time()
        self.pose_times.append(now)
        cutoff = now - RATE_WINDOW_S
        while self.pose_times and self.pose_times[0] < cutoff:
            self.pose_times.pop(0)

    def rate(self):
        now = time.time()
        cutoff = now - RATE_WINDOW_S
        while self.pose_times and self.pose_times[0] < cutoff:
            self.pose_times.pop(0)
        return len(self.pose_times) / RATE_WINDOW_S


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=0.0,
                    help="exit after N seconds instead of running until Ctrl-C")
    args = ap.parse_args()

    rclpy.init()
    node = BeaconBrightnessLive()
    print("No pixel-count target used -- this beacon was never characterised "
          "against a fixed band (that band belongs to a different physical "
          "beacon on a different rig). Hue should read 90-140 (OpenCV 0-180, "
          "blue window). TUNE FOR THE RATE. Ctrl-C to stop.\n")
    start = time.time()
    last_line = ""
    try:
        while rclpy.ok():
            if args.duration and (time.time() - start) > args.duration:
                break
            rclpy.spin_once(node, timeout_sec=0.4)
            n = node.sat
            if n is None:
                sys.stdout.write("\r  waiting for /camera/color/image_raw ...   ")
                sys.stdout.flush()
                continue
            if n == 0:
                verdict = "NO LIGHT  "
            else:
                verdict = "          "
            hue = node.hue
            hue_s = f"hue {hue:5.1f}" if hue is not None else "hue   -- "
            if hue is not None and not (90 <= hue <= 140):
                hue_s += "!"
            else:
                hue_s += " "
            hz = node.rate()
            if hz >= 8.0:
                rate_s = f"RATE {hz:5.1f} Hz EXCELLENT"
            elif hz >= 4.0:
                rate_s = f"RATE {hz:5.1f} Hz ok       "
            elif hz > 0.2:
                rate_s = f"RATE {hz:5.1f} Hz WEAK     "
            else:
                rate_s = f"RATE {hz:5.1f} Hz LOST     "
            last_line = f"sat px {n:7d}  {verdict} {hue_s} {rate_s}"
            sys.stdout.write(f"\r  {last_line}  ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nstopped.")
    except rclpy.executors.ExternalShutdownException:
        pass
    print(f"\nFINAL: {last_line}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
