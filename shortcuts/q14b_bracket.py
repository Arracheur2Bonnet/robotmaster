#!/usr/bin/env python3
"""Drift-bracketed analysis for 1.4b.

THE PROBLEM THIS SOLVES. The chassis rotates slowly on its own (~9 deg/min,
cause not in our software -- no command is sent). So between a reference
capture and a rotated capture, the CAMERA has turned by an unknown amount,
and the difference between the two readings is (beacon rotation) + (camera
drift). Attributing all of it to the beacon is wrong, and on 2026-08-14 that
error looked like the operator moving the beacon when they had not.

THE FIX, which is the standard one for a linear drift: bracket. Capture the
reference, rotate, capture, rotate BACK, capture the reference again. The two
references differ only by drift, which gives the rate directly; the rotated
reading is then corrected to the time of the first reference.

Usage: paste the three COPYME lines as arguments:
  bracket.py "REF1 r p y t" "ROT r p y t" "REF2 r p y t"
"""
import sys


def parse(a):
    f = a.split()
    return f[0], float(f[1]), float(f[2]), float(f[3]), float(f[4])


def main():
    if len(sys.argv) != 4:
        print(__doc__)
        return 1
    n1, r1, p1, y1, t1 = parse(sys.argv[1])
    nr, rr, pr, yr, tr = parse(sys.argv[2])
    n2, r2, p2, y2, t2 = parse(sys.argv[3])

    span = t2 - t1
    if span <= 0:
        print("  the two references must be separated in time")
        return 1
    frac = (tr - t1) / span

    print(f"bracket: {n1} -> {nr} -> {n2}   ({span:.0f}s total, "
          f"rotated capture at {100*frac:.0f}% of the way through)")
    print()
    print(f"  drift measured between the two references, over {span:.0f}s:")
    for nm, a, b in (("roll", r1, r2), ("pitch", p1, p2), ("yaw", y1, y2)):
        print(f"    {nm:<6} {a:+8.3f} -> {b:+8.3f}   drift {b-a:+7.3f} deg "
              f"({(b-a)/span*60:+6.2f} deg/min)")
    print()
    print(f"  rotation, corrected for the drift accumulated by that point:")
    verdict = []
    for nm, a, b, m in (("roll", r1, r2, rr), ("pitch", p1, p2, pr), ("yaw", y1, y2, yr)):
        expected_drift = (b - a) * frac
        corrected = m - a - expected_drift
        print(f"    {nm:<6} raw {m-a:+8.3f}   minus drift {expected_drift:+7.3f}"
              f"   = {corrected:+8.3f} deg")
        verdict.append((nm, corrected))
    print()
    dom = max(verdict, key=lambda v: abs(v[1]))
    print(f"  dominant axis: {dom[0]} at {dom[1]:+.2f} deg")
    others = [v for v in verdict if v[0] != dom[0]]
    print(f"  other axes: " + ", ".join(f"{n} {v:+.2f}" for n, v in others))
    return 0


if __name__ == "__main__":
    sys.exit(main())
