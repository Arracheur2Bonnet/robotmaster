#!/usr/bin/env python3
"""Live camera view with an intensity/detection HUD overlaid, for tuning the
beacon's LED knob by eye instead of reading numbers off a scrolling terminal.

Deliberately does NOT reuse the RoboMaster's fixed saturated-pixel band
(2000-45000 px) -- that band was measured on a different physical beacon and
a different camera, and a threshold like that does not transfer between rigs.
What DOES transfer: the OpenCV hue window (90-140, blue LEDs, same convention
this project uses everywhere) and, above all, the DETECTION RATE -- the only
number that says whether Carolus is actually working, independent of what a
"good" pixel count looks like on this particular camera's sensor.

Run this on a machine/session with a real display and X11 forwarding, e.g.:
    ssh -X -i ~/.ssh/carolus_nopass <user>@<host>
    source /opt/ros/jazzy/setup.bash
    python3 ~/beacon_hud_view.py

Press 'q' in the window to quit.
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

SAT = 250
RATE_WINDOW_S = 5.0


class BeaconHudView(Node):
    def __init__(self):
        super().__init__('beacon_hud_view')
        self.declare_parameter('image_topic', '/image_raw')
        self.image_topic = self.get_parameter('image_topic').value
        self.frame = None
        self.pose_times = []
        self.create_subscription(Image, self.image_topic, self.cb_img,
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/pose', self.cb_pose, 10)

    def cb_img(self, msg):
        a = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            img = a.reshape(msg.height, msg.width, -1)
        except ValueError:
            return
        self.frame = img.copy()

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


def draw_hud(img, sat_n, hue, hz, image_topic):
    h, w = img.shape[:2]
    overlay = img.copy()
    cv2.rectangle(overlay, (0, 0), (w, 130), (0, 0, 0), -1)
    img = cv2.addWeighted(overlay, 0.55, img, 0.45, 0)

    if hz >= 8.0:
        rate_color, rate_txt = (60, 220, 60), f"RATE {hz:5.1f} Hz  EXCELLENT"
    elif hz >= 4.0:
        rate_color, rate_txt = (60, 200, 220), f"RATE {hz:5.1f} Hz  OK"
    elif hz > 0.2:
        rate_color, rate_txt = (0, 165, 255), f"RATE {hz:5.1f} Hz  WEAK"
    else:
        rate_color, rate_txt = (0, 0, 255), f"RATE {hz:5.1f} Hz  LOST -- adjust the knob"

    hue_ok = hue is not None and 90 <= hue <= 140
    hue_color = (60, 220, 60) if hue_ok else (0, 0, 255)
    hue_txt = f"hue {hue:5.1f} (target 90-140)" if hue is not None else "hue --  (no saturated px)"

    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(img, "TUNE FOR THE RATE -- no fixed pixel target on this rig",
               (12, 24), font, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, f"saturated px (>{SAT}): {sat_n}", (12, 52), font, 0.6,
               (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, hue_txt, (12, 78), font, 0.6, hue_color, 2, cv2.LINE_AA)
    cv2.putText(img, rate_txt, (12, 108), font, 0.7, rate_color, 2, cv2.LINE_AA)
    cv2.putText(img, image_topic, (w - 220, 24), font, 0.5, (180, 180, 180), 1,
               cv2.LINE_AA)
    return img


def main():
    rclpy.init()
    node = BeaconHudView()
    print("Window 'beacon HUD' should appear. Press 'q' in it to quit.")
    print("Requires X11 forwarding (ssh -X) and a working display.\n")
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is None:
                continue
            frame = node.frame
            lum = frame.max(axis=2)
            mask = lum > SAT
            n = int(mask.sum())
            hue = None
            if n:
                b = float(frame[:, :, 0][mask].mean())
                g = float(frame[:, :, 1][mask].mean())
                r = float(frame[:, :, 2][mask].mean())
                mx, mn = max(b, g, r), min(b, g, r)
                if mx != mn:
                    if mx == r:
                        h = (60 * ((g - b) / (mx - mn)) + 360) % 360
                    elif mx == g:
                        h = 60 * ((b - r) / (mx - mn)) + 120
                    else:
                        h = 60 * ((r - g) / (mx - mn)) + 240
                    hue = h / 2.0
            hz = node.rate()
            disp = draw_hud(frame, n, hue, hz, node.image_topic)
            cv2.imshow("beacon HUD", disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
