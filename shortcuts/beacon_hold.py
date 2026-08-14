#!/usr/bin/env python3
"""Hold the beacon centred in the camera by visual servoing -- cancels BUG-104's gimbal drift.

WHY THIS EXISTS (2026-08-14). BUG-104: the robot's attitude estimate integrates
a constant misaligned gyro bias with no gravity correction, so it rotates about
a tilted axis at ~0.09 deg/s. In FREE mode the gimbal holds its *inertial*
heading against that drifting reference, so it physically slews at the same
rate -- about 5 deg/min. Measured live: the beacon was detected for ~4 minutes
after being placed dead ahead, then left the usable frame and Carolus reported
"Not enough blobs < 4" 1120 times in a row while the gimbal had wandered from
0.0 to -8.4 deg on its own.

That makes any long measurement impossible. 1.4b (the internship's main task)
needs stable tracking for ~30 minutes, over which the gimbal would wander ~150
deg.

WHY VISUAL SERVOING, AND NOT THE OBVIOUS ALTERNATIVES
  * Open-loop counter-rotation at a fixed -0.09 deg/s: cancels today's drift
    rate and nothing else. The rate is a gyro bias; it is not guaranteed
    constant across power cycles or temperature, and an uncorrected residual
    still accumulates without bound.
  * Closing the loop on /carolus/gimbal_yaw_rel: that signal accumulates the
    chassis-rotation history (BUG-080) and was measured drifting -50 deg while
    gimbal torque was CUT, i.e. while the joint physically could not move. It
    is trustworthy across a commanded move and not as an absolute reference.
  * The image is the only reference that is external, absolute, and directly
    measures the quantity we care about.

WHY THIS IS SAFE FOR 1.4b SPECIFICALLY. Holding the beacon fixed in the image
is the same thing as holding the camera fixed *relative to the beacon* -- which
is exactly 1.4b's requirement (camera still, only the beacon rotating). And
rotating the beacon about its own axis barely moves its centroid, so the
controller does not fight the very rotation being measured. The two needs are
compatible rather than in conflict.

The chassis is never commanded. Only the gimbal moves.

USAGE
    python3 beacon_hold.py                # hold wherever the beacon currently is
    python3 beacon_hold.py --duration 1800
    python3 beacon_hold.py --status       # report only, command nothing
"""
import argparse
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

# Carolus's own blob criteria (carolus_node/config/robomaster_s1.yaml), so this
# tracks the same blobs the solver does rather than a looser proxy. Using a
# looser threshold picked up 11 room reflections while Carolus saw fewer than 4
# real LEDs -- a proxy that disagrees with the real detector is worse than none.
SAT_THRESHOLD = 80
MIN_CIRCULARITY = 0.6
MIN_AREA, MAX_AREA = 8.0, 1800.0

DEADBAND_PX = 25.0      # don't chase sensor noise
KP = 0.045              # deg/s per pixel of error
MAX_RATE = 12.0         # deg/s ceiling -- gentle, nothing like LOCK's 540
LOST_GRACE_S = 8.0      # keep still this long before declaring the beacon lost


def find_leds(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    _, low_sat = cv2.threshold(hsv[:, :, 1], SAT_THRESHOLD, 255, cv2.THRESH_BINARY_INV)
    _, bright = cv2.threshold(hsv[:, :, 2], 200, 255, cv2.THRESH_BINARY)
    mask = cv2.bitwise_and(low_sat, bright)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    pts = []
    for c in contours:
        area = cv2.contourArea(c)
        if not (MIN_AREA <= area <= MAX_AREA):
            continue
        per = cv2.arcLength(c, True)
        if per <= 0 or 4 * np.pi * area / (per * per) < MIN_CIRCULARITY:
            continue
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        pts.append((m["m10"] / m["m00"], m["m01"] / m["m00"]))
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=900.0)
    ap.add_argument("--status", action="store_true", help="report only, send no commands")
    args = ap.parse_args()

    rospy.init_node("beacon_hold", anonymous=True)
    pub = rospy.Publisher("/carolus/gimbal_vel", Twist, queue_size=1)
    if not args.status:
        while pub.get_num_connections() == 0 and not rospy.is_shutdown():
            time.sleep(0.2)

    t_start = time.time()
    last_seen = 0.0
    corrected = 0.0
    n_frames = 0
    n_locked = 0
    print("beacon_hold: %s, deadband %.0f px, Kp %.3f deg/s/px, max %.0f deg/s"
          % ("STATUS ONLY" if args.status else "servoing gimbal", DEADBAND_PX, KP, MAX_RATE))

    while not rospy.is_shutdown() and time.time() - t_start < args.duration:
        try:
            msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5)
        except Exception:
            continue
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)[:, :, :3]
        pts = find_leds(img)
        n_frames += 1
        cx_img = msg.width / 2.0

        if len(pts) < 3:
            # Not enough LEDs to trust a centroid. Do NOT guess -- stop the gimbal
            # rather than servo on noise, which would chase a reflection.
            if not args.status and time.time() - last_seen > LOST_GRACE_S:
                pub.publish(Twist())
            if n_frames % 20 == 0:
                print("  [%5.0fs] beacon LOST (%d blobs) -- gimbal held still"
                      % (time.time() - t_start, len(pts)))
            continue

        last_seen = time.time()
        n_locked += 1
        cx = float(np.mean([p[0] for p in pts]))
        err = cx - cx_img

        rate = 0.0
        if abs(err) > DEADBAND_PX:
            rate = float(np.clip(-KP * err, -MAX_RATE, MAX_RATE))
            if not args.status:
                t = Twist()
                t.angular.z = rate
                pub.publish(t)
                corrected += abs(rate) * 0.1
        elif not args.status:
            pub.publish(Twist())

        if n_frames % 20 == 0:
            print("  [%5.0fs] %d LEDs  centroid_x=%6.1f  err=%+7.1f px  cmd=%+5.1f deg/s"
                  % (time.time() - t_start, len(pts), cx, err, rate))

    if not args.status:
        pub.publish(Twist())
    print("\nheld %.0fs | frames %d | beacon visible in %d (%.0f%%) | total correction %.1f deg"
          % (time.time() - t_start, n_frames, n_locked,
             100.0 * n_locked / max(n_frames, 1), corrected))


if __name__ == "__main__":
    main()
