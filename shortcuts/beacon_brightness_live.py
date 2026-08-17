#!/usr/bin/env python3
"""Live beacon-brightness meter, for tuning the LED intensity by hand.

WHY THIS EXISTS. On 2026-08-17 the beacon was rebuilt on a tripod and
detection stopped completely. Carolus only said `Not enough blobs with
required circularity` and then `0 contours found`, which reads like the
beacon is absent -- it was the opposite. The LEDs were so bright they bloomed
into one merged, irregular blob covering 88 628 saturated pixels (9.6% of the
frame) against the ~118 px the same four LEDs occupy when detection works.
A blob that large is rejected by `max_area`, and its irregular shape fails
`min_circularity`, so the failure is indistinguishable from "no beacon" in the
log.

Turning the intensity down is a physical knob with no feedback, so this gives
the feedback: run it, turn the knob, watch the number. It is passive -- it
subscribes to the camera and prints, and commands nothing.

WHAT THE TARGET IS. The four LEDs occupied roughly 118 saturated px at ~0.9 m
in the sessions where detection worked (13.9 Hz on `/pose`). The band below is
set around that, generously, because it varies with distance and exposure:
the number to trust is `pose`, and the pixel count is the knob-turning aid
that gets you there.

Usage, from the lab PC:
    ssh -t ubuntu@192.168.0.103 'source /opt/ros/noetic/setup.bash; \
        export ROS_MASTER_URI=http://localhost:11311; \
        python3 /tmp/beacon_brightness_live.py'

Ctrl-C to stop.
"""
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image

SAT = 250          # a pixel this bright is saturated on at least one channel
TARGET_LO = 60     # below this the LEDs are likely too dim to segment reliably
TARGET_HI = 900    # above this they are blooming and will merge

state = {"sat": None, "hue": None, "last_pose": 0.0, "frames": 0}
# Rolling window of /pose arrival times. The DETECTION RATE is the real
# quality signal, not the pixel count: on 2026-08-17 a setting that looked
# plausible by brightness alone was delivering 1.5 Hz, against 5.0 Hz ten
# minutes earlier and 13.9 Hz the same afternoon. Tune to maximise this
# number; the pixel count is only the knob-turning aid.
pose_times = []
RATE_WINDOW_S = 5.0


def cb_img(msg):
    a = np.frombuffer(msg.data, dtype=np.uint8)
    try:
        img = a.reshape(msg.height, msg.width, -1)
    except ValueError:
        return
    lum = img.max(axis=2)
    mask = lum > SAT
    n = int(mask.sum())
    state["sat"] = n
    state["frames"] += 1
    if n:
        # bgr8 on this camera; mean colour of the saturated region only
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
        state["hue"] = h / 2.0        # OpenCV 0-180 scale, as the config uses
    else:
        state["hue"] = None


def cb_pose(_msg):
    now = time.time()
    state["last_pose"] = now
    pose_times.append(now)
    cutoff = now - RATE_WINDOW_S
    while pose_times and pose_times[0] < cutoff:
        pose_times.pop(0)


def main():
    rospy.init_node("beacon_brightness_live", anonymous=True, disable_signals=True)
    rospy.Subscriber("/camera/color/image_raw", Image, cb_img, queue_size=1)
    rospy.Subscriber("/pose", PoseStamped, cb_pose, queue_size=1)

    print(f"target {TARGET_LO}-{TARGET_HI} saturated px   "
          f"(hue should read 90-140, the blue window the config expects)")
    print("turn the beacon's intensity knob and watch. Ctrl-C to stop.\n")
    try:
        while not rospy.is_shutdown():
            time.sleep(0.4)
            n = state["sat"]
            if n is None:
                sys.stdout.write("\r  waiting for /camera/color/image_raw ...   ")
                sys.stdout.flush()
                continue
            if n == 0:
                verdict = "NO LIGHT     "
            elif n < TARGET_LO:
                verdict = "TOO DIM      "
            elif n > TARGET_HI:
                verdict = "TOO BRIGHT   "
            else:
                verdict = "GOOD         "
            hue = state["hue"]
            hue_s = f"hue {hue:5.1f}" if hue is not None else "hue   -- "
            if hue is not None and not (90 <= hue <= 140):
                hue_s += "!"          # outside the configured blue window
            else:
                hue_s += " "
            now = time.time()
            cutoff = now - RATE_WINDOW_S
            while pose_times and pose_times[0] < cutoff:
                pose_times.pop(0)
            hz = len(pose_times) / RATE_WINDOW_S
            if hz >= 8.0:
                rate_s = f"RATE {hz:5.1f} Hz EXCELLENT"
            elif hz >= 4.0:
                rate_s = f"RATE {hz:5.1f} Hz ok       "
            elif hz > 0.2:
                rate_s = f"RATE {hz:5.1f} Hz WEAK     "
            else:
                rate_s = f"RATE {hz:5.1f} Hz LOST     "
            sys.stdout.write(f"\r  sat px {n:7d}  {verdict} {hue_s} {rate_s}  ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nstopped.")


if __name__ == "__main__":
    main()
