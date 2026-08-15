#!/usr/bin/env python3
"""1.4b -- quaternion validation: stop-and-hold capture of camera_link -> beacon_observed.

WHAT THIS IS FOR. `plan-fin-de-stage.md` item 1.4b asks whether the ROS
remap in carolus_tf_broadcaster.py has the right parity: rotate the beacon
about one axis at a time, and check whether the TF reports the rotation with
the correct SIGN. Zero inverted axes confirms the formula; two inverted and
one correct is the wrong-parity signature; three inverted means the 2026-08-11
direction fix regressed.

WHY A SCRIPT AND NOT `tf_echo` BY HAND. A 2026-08-14 attempt used a continuous
freehand sweep and produced a trace that was non-monotonic and partly STALE --
four consecutive samples reading an identical value with an identical rotation
axis, i.e. a cached lookup, not four measurements. Unusable for reading a
sign. This captures the way that failure taught us to: hold still, take many
samples, and REFUSE the reading if it is not stable.

It is passive -- it subscribes to TF and prints. It commands nothing.

Usage:  q14b.py <label> [seconds]
        q14b.py reference 20
        q14b.py Z+90 20
"""
import math
import sys
import time

import rospy
import tf2_ros

LABEL = sys.argv[1] if len(sys.argv) > 1 else "sample"
SECONDS = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
# The stability bar, set from what the TEST needs rather than from what looks
# tidy. 1.4b reads the SIGN of rotations of 30-90 deg, so any spread that
# cannot plausibly flip a sign is good enough: 5 deg leaves a 6:1 margin
# against the smallest rotation in the protocol. An earlier 1.0 deg bar was
# set from six `tf_echo` reads that agreed to 0.5 deg -- but those very likely
# came from one cached transform rather than six measurements, so it was a
# bar this rig cannot actually meet. Measured here: roll and pitch hold an
# interquartile spread of 0.1-0.3 deg, yaw about 2 deg (part of which is the
# residual gimbal drift accumulating during the capture window).
STABILITY_DEG = 5.0


def q_to_rpy(x, y, z, w):
    sr = 2 * (w * x + y * z)
    cr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sr, cr)
    sp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sp) if abs(sp) >= 1 else math.asin(sp)
    sy = 2 * (w * z + x * y)
    cy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(sy, cy)
    return [math.degrees(v) for v in (roll, pitch, yaw)]


def main():
    rospy.init_node("q14b", anonymous=True, disable_signals=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    time.sleep(2.0)

    rows = []
    seen = set()
    t0 = time.time()
    while time.time() - t0 < SECONDS and not rospy.is_shutdown():
        try:
            tr = buf.lookup_transform("camera_link", "beacon_observed",
                                      rospy.Time(0), rospy.Duration(0.3))
        except Exception:
            time.sleep(0.1)
            continue
        stamp = tr.header.stamp.to_sec()
        if stamp in seen:          # same transform re-read: NOT a new measurement
            time.sleep(0.1)
            continue
        seen.add(stamp)
        t = tr.transform.translation
        q = tr.transform.rotation
        rows.append((t.x, t.y, t.z) + tuple(q_to_rpy(q.x, q.y, q.z, q.w)))
        time.sleep(0.1)

    print(f"=== {LABEL} ===")
    if len(rows) < 5:
        print(f"  REJECTED: only {len(rows)} distinct transforms in {SECONDS:.0f}s "
              f"-- beacon not detected, or TF not refreshing")
        return 1

    names = ["tx", "ty", "tz", "roll", "pitch", "yaw"]

    # ROBUST STATISTICS, and why they are necessary here rather than tidy.
    # Measured 2026-08-14 on a stationary rig: ~88% of samples fall in one
    # tight cluster and ~12% land on the P4P solver's ALTERNATE solution
    # (roll +41 vs +0.5, yaw -41 vs -12). That is the classic near-planar
    # pose ambiguity, not noise -- a mean over both clusters is a number that
    # describes neither, and the full span is dominated entirely by the
    # outliers. So: median for the value, and an interquartile filter to
    # decide stability, with the outlier fraction reported rather than
    # quietly discarded.
    def med(v):
        v = sorted(v)
        n = len(v)
        return v[n // 2] if n % 2 else 0.5 * (v[n // 2 - 1] + v[n // 2])

    # Keep samples whose ROLL sits near the median roll: the two P4P solutions
    # are furthest apart on roll, so it is the cleanest discriminator.
    rolls = [r[3] for r in rows]
    mroll = med(rolls)
    kept = [r for r in rows if abs(r[3] - mroll) < 10.0]
    frac_out = 100.0 * (len(rows) - len(kept)) / len(rows)
    if len(kept) < 5:
        print(f"  REJECTED: only {len(kept)} samples in the dominant cluster")
        return 1

    meds, spreads = [], []
    for i in range(6):
        vals = sorted(k[i] for k in kept)
        meds.append(med(vals))
        # interquartile spread, not min-max: one straggler must not veto a run
        spreads.append(vals[int(0.75 * (len(vals) - 1))] - vals[int(0.25 * (len(vals) - 1))])

    print(f"  n={len(rows)} distinct samples over {SECONDS:.0f}s "
          f"({len(kept)} in the dominant cluster, {frac_out:.1f}% on the alternate P4P solution)")
    print(f"  translation (m)  x={meds[0]:+.4f}  y={meds[1]:+.4f}  z={meds[2]:+.4f}")
    print(f"  rotation   (deg) roll={meds[3]:+.3f}  pitch={meds[4]:+.3f}  yaw={meds[5]:+.3f}   [medians]")
    print(f"  IQR spread (deg) roll={spreads[3]:.3f}  pitch={spreads[4]:.3f}  yaw={spreads[5]:.3f}")

    worst = max(spreads[3:])
    if worst > STABILITY_DEG:
        print(f"  *** UNSTABLE: worst interquartile spread {worst:.2f} deg > {STABILITY_DEG} deg.")
        print(f"      Hold the beacon still, check it is fully in frame, repeat.")
        return 1
    if frac_out > 35.0:
        print(f"  *** AMBIGUOUS: {frac_out:.0f}% of samples on the alternate P4P solution.")
        print(f"      The solver is not committing to one pose -- change the beacon")
        print(f"      angle slightly and repeat; do not record this.")
        return 1
    print(f"  STABLE (worst IQR {worst:.3f} deg) -- usable as a 1.4b reading")
    # Timestamp emitted so a drift-bracketed sequence can correct for the
    # chassis rotation between captures -- see bracket.py. Without it the two
    # readings of a rotation test are separated by an unknown amount of drift
    # and the difference between them is not the rotation.
    print(f"  COPYME {LABEL} {meds[3]:+.3f} {meds[4]:+.3f} {meds[5]:+.3f} {time.time():.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
