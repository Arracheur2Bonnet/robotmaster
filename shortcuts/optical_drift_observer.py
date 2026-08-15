#!/usr/bin/env python3
"""Independent optical observer for the launch bisection.

WHY THIS EXISTS. The bisection has to answer "does the robot drift with only
terminal N running", but the usual instruments are themselves part of what is
being tested: `gimbal_yaw_rel` needs T2's SDK connection, and the beacon
bearing normally comes from `/pose`, which needs T3. Measuring drift with T3
running would make it impossible to clear T3.

This subscribes ONLY to /camera/color/image_raw (published by T2) and finds
the beacon by brightness alone -- no /pose, no Carolus, no T3, no T4. It is
therefore usable from the T1+T2 stage onward, and it measures the one thing
that matters: is the camera's view of a fixed beacon rotating.

It is passive: it subscribes and prints. It publishes nothing, commands
nothing, and opens no SDK connection.

Deliberately numpy-only, no cv2/cv_bridge: those pull the OpenCV ABI zoo this
project has already been bitten by three times (BUG-101/102/108), and a
brightness centroid needs none of it.

Usage: observer.py <seconds> [label]
"""
import sys
import time

import numpy as np
import rospy
from sensor_msgs.msg import Image

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
LABEL = sys.argv[2] if len(sys.argv) > 2 else "run"
# Only the brightest pixels count -- the LEDs saturate, everything else does
# not. Same principle as Carolus's own image_threshold, deliberately a bit
# stricter so ambient reflections do not pull the centroid.
THRESH = int(__import__("os").environ.get("OBS_THRESH", "252"))

samples = []


def cb(msg):
    # sensor_msgs/Image, bgr8 or rgb8: take a luminance proxy cheaply.
    a = np.frombuffer(msg.data, dtype=np.uint8)
    try:
        img = a.reshape(msg.height, msg.width, -1)
    except ValueError:
        return
    lum = img.max(axis=2)                 # brightest channel per pixel
    mask = lum > THRESH
    n = int(mask.sum())
    if n < 4:                             # nothing bright enough -> no beacon
        return
    ys, xs = np.nonzero(mask)
    cx = float(xs.mean())
    # normalised horizontal position, -1 (left edge) .. +1 (right edge)
    samples.append((time.time(), (cx - msg.width / 2.0) / (msg.width / 2.0), n))


def fit(ts, vs):
    n = len(ts)
    sx, sy = sum(ts), sum(vs)
    sxx = sum(t * t for t in ts)
    sxy = sum(a * b for a, b in zip(ts, vs))
    denom = n * sxx - sx * sx
    if denom == 0:
        return 0.0, 0.0
    m = (n * sxy - sx * sy) / denom
    c = (sy - m * sx) / n
    mean = sy / n
    sst = sum((v - mean) ** 2 for v in vs)
    ssr = sum((v - (m * t + c)) ** 2 for t, v in zip(ts, vs))
    return m, (1 - ssr / sst if sst > 0 else 0.0)


def main():
    rospy.init_node("bisect_observer", anonymous=True, disable_signals=True)
    rospy.Subscriber("/camera/color/image_raw", Image, cb, queue_size=1)
    t0 = time.time()
    while time.time() - t0 < DURATION and not rospy.is_shutdown():
        time.sleep(0.2)

    print(f"[{LABEL}]", end=" ")
    if len(samples) < 20:
        print(f"INSUFFICIENT DATA (n={len(samples)}) -- beacon not bright enough, "
              f"or /camera/color/image_raw not publishing")
        return
    ts = [t - samples[0][0] for t, _, _ in samples]
    xs = [x for _, x, _ in samples]
    slope, r2 = fit(ts, xs)
    span = max(xs) - min(xs)
    dur = ts[-1]
    print(f"n={len(samples)} over {dur:.0f}s ({len(samples)/dur:.1f} Hz)  "
          f"blobpx~{int(np.median([p for _, _, p in samples]))}")
    print(f"        beacon x: {xs[0]:+.4f} -> {xs[-1]:+.4f}   "
          f"slope {slope*60:+.4f} /min   R2={r2:.3f}   span {span:.4f}")
    # A frame half-width is ~50 deg for this camera; convert for readability.
    print(f"        approx {slope*60*50:+.2f} deg/min   "
          f"{'DRIFTING' if abs(slope*60) > 0.01 and r2 > 0.5 else 'stable'}")


if __name__ == "__main__":
    main()
