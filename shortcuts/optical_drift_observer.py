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

2026-08-31 -- promoted from bisection tool to PRIMARY instrument for
protocol 25, after `chassis.sub_attitude` was disqualified as a drift
measurement channel (it integrates a constant gyro bias with no gravity
correction: 2026-08-13/14 measured it claiming 18.1 deg of rotation while
this optical channel measured 0.4 px where a real 18.1 deg would have moved
227 px -- 567x). Two changes for that role, both here:
  1. raw samples are written to CSV, not only summarised, so a session can be
     re-analysed without re-running the robot;
  2. the pixels-per-degree figure is the MEASURED one, not an assumed
     half-field-of-view -- see PX_PER_DEG below.

Usage: observer.py <seconds> [label] [csv_path]
       (csv_path defaults to /tmp/optical_drift_<label>_<timestamp>.csv)
"""
import os
import sys
import time

import numpy as np
import rospy
from sensor_msgs.msg import Image

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
LABEL = sys.argv[2] if len(sys.argv) > 2 else "run"
CSV_PATH = (sys.argv[3] if len(sys.argv) > 3
            else f"/tmp/optical_drift_{LABEL}_{time.strftime('%Y%m%d_%H%M%S')}.csv")
# Only the brightest pixels count -- the LEDs saturate, everything else does
# not. Same principle as Carolus's own image_threshold, deliberately a bit
# stricter so ambient reflections do not pull the centroid.
THRESH = int(os.environ.get("OBS_THRESH", "252"))

# MEASURED, not assumed. 2026-08-13/14: a commanded 14.6 deg gimbal move
# produced 182.9 px of image shift on this camera => 182.9/14.6 = 12.53 px/deg.
# This replaces the "a frame half-width is ~50 deg" approximation this file
# used until 2026-08-31. The two happen to agree closely (1280 px wide, so a
# 640 px half-width / 12.53 = 51.1 deg, ~2 % from the old guess) -- the point
# is not that the number moved much, it is that it is now traceable to a
# measurement instead of resting on an unsourced estimate.
PX_PER_DEG = float(os.environ.get("OBS_PX_PER_DEG", "12.53"))

samples = []


FRAME_W = None   # set on the first frame; needed for the px <-> normalised map


def cb(msg):
    global FRAME_W
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
    FRAME_W = msg.width
    # Store the RAW pixel centroid as well as the normalised position. The raw
    # value is what makes the CSV re-analysable against a different px/deg
    # calibration later without re-running the robot -- the whole reason the
    # 2026-08-10 dataset could not be rescued.
    samples.append((time.time(), cx,
                    (cx - msg.width / 2.0) / (msg.width / 2.0), n))


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

    # Write the CSV FIRST, before any analysis can fail. A session that
    # produced samples must never lose them to a divide-by-zero in a summary
    # line -- the raw data is the expensive part, the fit is free to redo.
    try:
        with open(CSV_PATH, "w") as fh:
            fh.write(f"# label={LABEL} thresh={THRESH} px_per_deg={PX_PER_DEG} "
                     f"frame_w={FRAME_W}\n")
            fh.write("wall_time_s,t_rel_s,cx_px,x_norm,blob_px\n")
            t0 = samples[0][0] if samples else 0.0
            for t, cx, xn, n in samples:
                fh.write(f"{t:.6f},{t - t0:.6f},{cx:.3f},{xn:.6f},{n}\n")
        print(f"[{LABEL}] raw samples -> {CSV_PATH}  (n={len(samples)})")
    except OSError as e:
        print(f"[{LABEL}] !! COULD NOT WRITE CSV {CSV_PATH}: {e}")

    print(f"[{LABEL}]", end=" ")
    if len(samples) < 20:
        print(f"INSUFFICIENT DATA (n={len(samples)}) -- beacon not bright enough, "
              f"or /camera/color/image_raw not publishing")
        return
    ts = [t - samples[0][0] for t, _, _, _ in samples]
    px = [c for _, c, _, _ in samples]
    slope_px, r2 = fit(ts, px)            # px/s
    span_px = max(px) - min(px)
    dur = ts[-1]
    deg_per_min = slope_px * 60.0 / PX_PER_DEG
    print(f"n={len(samples)} over {dur:.0f}s ({len(samples)/dur:.1f} Hz)  "
          f"blobpx~{int(np.median([p for _, _, _, p in samples]))}")
    print(f"        beacon cx: {px[0]:.1f} -> {px[-1]:.1f} px   "
          f"slope {slope_px*60:+.3f} px/min   R2={r2:.3f}   span {span_px:.1f} px")
    # px/deg is MEASURED (see PX_PER_DEG at the top), not an assumed half-FOV.
    print(f"        {deg_per_min:+.3f} deg/min   (px_per_deg={PX_PER_DEG}, measured)   "
          f"{'DRIFTING' if abs(deg_per_min) > 0.5 and r2 > 0.5 else 'stable'}")


if __name__ == "__main__":
    main()
