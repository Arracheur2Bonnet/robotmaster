#!/usr/bin/env python3
"""BUG-105: detect FRAME TEARING under chassis motion -- the condition 2.3 runs under.

RESULT 2026-08-13: 0/60 torn frames rotating, 0/60 driving, 0/60 idle, with the
detector validated 6/6 against synthetic seams (see the self-test note below).
The phenomenon is intermittent (2 instances in a whole session on 2026-08-12),
so a clean 120-frame run lowers the risk for 2.3 rather than eliminating it.

SELF-TEST BEFORE TRUSTING A NEGATIVE. A "no tearing" result is worthless if the
detector is blind. Validate it by splicing two genuinely DIFFERENT frames (roll
one horizontally, or slew the gimbal between grabs) and confirming the score
clears the threshold. Splicing two frames of a static scene does NOT work --
they are near-identical, the splice has no seam, and the detector correctly
sees nothing; that mistake was made first and looked like detector failure.

v1 counted bright-value contours, reproducing the 2026-08-12 signature
(~1000 spurious contours). That signature was measured with the lab lights
OFF, where a normal frame has almost no bright pixels. With the lights on it
fires on 100% of frames in every condition, including idle -- it measures
illumination, not tearing. Useless.

What tearing actually is: the frame buffer is filled from two different
captures, so the image contains a horizontal SEAM -- one row boundary where
content jumps discontinuously. Natural images have smooth row-to-row
transitions; a tear puts one sharp outlier among them.

Score = max(row-to-row mean abs diff) / median(row-to-row mean abs diff).
Intact frame: the max is just the strongest real edge, so the ratio is modest.
Torn frame: one seam dwarfs everything, so the ratio spikes.

The threshold is NOT assumed -- it is derived from the idle baseline measured
in this same run (idle median + 6 sigma), so it adapts to the actual scene
instead of inheriting a number from a differently-lit session.
"""
import threading
import time

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

rospy.init_node("tear_detect", anonymous=True)
pub = rospy.Publisher("/carolus/cmd_vel", Twist, queue_size=1)


def tear_score(bgr):
    g = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    d = np.abs(np.diff(g, axis=0)).mean(axis=1)   # per-row-boundary difference
    med = float(np.median(d))
    if med <= 1e-6:
        return 0.0, 0
    return float(d.max() / med), int(d.argmax())


def collect(n, label):
    scores, rows = [], []
    for _ in range(n):
        try:
            m = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=8)
        except Exception:
            continue
        a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)[:, :, :3]
        s, r = tear_score(a)
        scores.append(s)
        rows.append(r)
    arr = np.array(scores) if scores else np.array([0.0])
    print("%-24s n=%3d  median=%6.2f  p90=%6.2f  max=%7.2f"
          % (label, len(scores), np.median(arr), np.percentile(arr, 90), arr.max()))
    return arr, rows


stop = threading.Event()


def oscillate(vx, wz, period, duration):
    t0 = time.time()
    last = t0
    sign = 1.0
    while time.time() - t0 < duration and not stop.is_set():
        if time.time() - last >= period:
            sign = -sign
            last = time.time()
        t = Twist()
        t.linear.x = vx * sign
        t.angular.z = wz * sign
        pub.publish(t)
        time.sleep(0.1)
    pub.publish(Twist())


idle, _ = collect(60, "IDLE (baseline)")
thresh = float(np.median(idle) + 6.0 * (idle.std() if idle.std() > 0 else 1.0))
print("\n--> tear threshold derived from THIS scene: %.2f (idle median + 6 sigma)\n" % thresh)

results = {}
for label, vx, wz, period in [("ROTATING in place", 0.0, 0.6, 3.0),
                              ("DRIVING fwd/back", 0.12, 0.0, 1.5)]:
    stop.clear()
    th = threading.Thread(target=oscillate, args=(vx, wz, period, 32))
    th.start()
    time.sleep(1.5)
    arr, rows = collect(60, label)
    stop.set()
    th.join()
    pub.publish(Twist())
    time.sleep(2)
    n_tear = int((arr > thresh).sum())
    results[label] = (n_tear, len(arr))
    print("   -> frames over threshold: %d/%d (%.1f%%)\n" % (n_tear, len(arr), 100.0 * n_tear / max(len(arr), 1)))

pub.publish(Twist())
n_idle = int((idle > thresh).sum())
print("SUMMARY  idle %d/%d over threshold (expected ~0 by construction)" % (n_idle, len(idle)))
for k, (n, tot) in results.items():
    print("         %-22s %d/%d" % (k, n, tot))
print("STOPPED")
