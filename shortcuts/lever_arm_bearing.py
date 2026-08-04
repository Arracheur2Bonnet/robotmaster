#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lever-arm bearing model — how much apparent bearing change does a chassis
rotation produce, purely from the camera's optical centre being offset from
the chassis rotation centre?

WHY THIS EXISTS (2026-07-31)
----------------------------
On 2026-07-30 an ALIGN run showed the beacon's bearing in the camera image
growing +6.3 -> +18.2 deg (about +12 deg) over roughly 97 deg of in-place
chassis rotation, while the gimbal's ground-referenced heading stayed flat
(-142.3 -> -142.4 deg). Flat yaw_ground means the camera did NOT rotate in
the world frame, so the bearing change cannot come from camera rotation.

Point 2 of `research-log/21-points-a-creuser/` lists two candidate causes:
  H2 - geometry: the optical centre sits off the chassis rotation centre, so
       rotating in place sweeps the camera along an arc. That is a real
       translation of the observation point with ZERO wheel slip.
  H3 - genuine Mecanum lateral slip during in-place rotation.

Three research arms independently derived the same closed form for H2
(arc length d*dtheta, bearing change following from the displacement). This
script implements it EXACTLY rather than to first order, so the prediction
can be checked against the real measurement instead of an approximation.

HOW TO USE IT
-------------
    python3 lever_arm_bearing.py                 # the 2026-07-30 run, solved
    python3 lever_arm_bearing.py --d 0.12        # predict for a measured d

`--d` is the only number that needs a tape measure: the distance from the
chassis's rotation centre to the camera's optical centre, in metres.

WHAT IT CANNOT TELL YOU
-----------------------
It models translation only. If the measured d predicts much LESS than the
observed bearing change, the remainder needs another explanation (H3 slip,
or bad intrinsics). If it predicts MORE, the geometry assumption about where
the camera sits relative to the line of sight is probably wrong. Either way
the point is to size the effect before spending a hardware session on it.
"""

import argparse
import math

# ── The 2026-07-30 observation, from journal.md (2026-07-30 (2)) ─────────────
OBS_ROTATION_DEG = 97.0    # chassis rotation over the phase (yaw_rel +97.2 -> ~0)
OBS_BEARING_CHANGE_DEG = 12.0   # ecart_image +6.3 -> +18.2 deg
OBS_RANGE_M = 0.87         # measured beacon range at the verification step


def bearing_change_deg(d, rotation_deg, range_m, phi_deg=0.0, bearing0_deg=0.0):
    """Exact apparent bearing change of a fixed target, for a camera at
    lever arm `d` from the rotation centre, after the chassis rotates by
    `rotation_deg` in place.

    Frame: rotation centre at the origin. The camera starts at angle
    `phi_deg` around that centre, at distance `d`. The camera's own heading
    does NOT change (yaw-stabilised gimbal, confirmed on hardware). The
    target sits at range `range_m` from the camera's INITIAL position, at
    bearing `bearing0_deg` in the camera frame.

    Returns the signed change in target bearing, in degrees.
    """
    th = math.radians(rotation_deg)
    phi = math.radians(phi_deg)
    b0 = math.radians(bearing0_deg)

    # Camera position before and after the chassis rotation
    c0 = (d * math.cos(phi), d * math.sin(phi))
    c1 = (d * math.cos(phi + th), d * math.sin(phi + th))

    # Target, expressed from the initial camera position and bearing.
    # Camera heading is fixed, so the camera frame does not rotate: bearings
    # are measured against a constant world direction throughout.
    target = (c0[0] + range_m * math.cos(b0), c0[1] + range_m * math.sin(b0))

    # Bearing after the camera has translated along its arc
    b1 = math.atan2(target[1] - c1[1], target[0] - c1[0])
    return math.degrees(b1 - b0)


def max_bearing_change_deg(d, rotation_deg, range_m):
    """Upper bound on the effect: the whole camera displacement taken
    perpendicular to the line of sight. Chord of the arc is 2*d*sin(dtheta/2).
    Useful as a first sanity check, since it needs no assumption about where
    the target sits relative to the lever arm."""
    chord = 2.0 * d * math.sin(math.radians(rotation_deg) / 2.0)
    return math.degrees(math.atan2(chord, range_m))


def solve_d_for(bearing_change_deg_target, rotation_deg, range_m):
    """Invert the bound: what lever arm would be needed to produce this much
    bearing change at all? Returns d in metres."""
    chord = range_m * math.tan(math.radians(bearing_change_deg_target))
    return chord / (2.0 * math.sin(math.radians(rotation_deg) / 2.0))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--d", type=float, default=None,
                    help="measured lever arm, rotation centre -> optical centre, in metres")
    ap.add_argument("--rotation", type=float, default=OBS_ROTATION_DEG,
                    help=f"chassis rotation in degrees (default {OBS_ROTATION_DEG}, the 2026-07-30 run)")
    ap.add_argument("--range", type=float, default=OBS_RANGE_M, dest="rng",
                    help=f"target range in metres (default {OBS_RANGE_M})")
    args = ap.parse_args()

    print("Lever-arm bearing model — geometry only, zero wheel slip assumed")
    print(f"  chassis rotation : {args.rotation:.1f} deg")
    print(f"  target range     : {args.rng:.2f} m")
    print()

    if args.d is None:
        d_needed = solve_d_for(OBS_BEARING_CHANGE_DEG, args.rotation, args.rng)
        print(f"No --d given, so solving the inverse question instead:")
        print(f"  observed bearing change (2026-07-30) : {OBS_BEARING_CHANGE_DEG:+.1f} deg")
        print(f"  lever arm needed to explain ALL of it : {d_needed*100:.1f} cm")
        print()
        print("  Read this as a plausibility test, not a measurement. If the real")
        print("  optical centre sits about that far from the rotation centre, geometry")
        print("  alone accounts for the observation and no slip is needed to explain it.")
        print("  If the real offset is far smaller, the remainder is something else.")
        print()
        print("  Measure it, then re-run with --d <metres>.")
        return

    ub = max_bearing_change_deg(args.d, args.rotation, args.rng)
    print(f"  lever arm d      : {args.d*100:.1f} cm")
    print()
    print(f"  upper bound on bearing change : {ub:+.1f} deg")
    print("    (whole displacement taken perpendicular to the line of sight)")
    print()
    print("  Exact model, for a few target directions relative to the lever arm:")
    for phi in (0.0, 45.0, 90.0, 135.0, 180.0):
        val = bearing_change_deg(args.d, args.rotation, args.rng, phi_deg=phi)
        print(f"    camera start angle {phi:5.0f} deg -> {val:+7.2f} deg")
    print()
    print(f"  observed on 2026-07-30        : {OBS_BEARING_CHANGE_DEG:+.1f} deg")
    if ub >= OBS_BEARING_CHANGE_DEG:
        print("  -> geometry CAN account for the observation. Slip is not required.")
    else:
        print("  -> geometry CANNOT account for it alone; a remainder needs another cause")
        print("     (Mecanum slip, or intrinsics error — points 2 and 14).")


if __name__ == "__main__":
    main()
