#!/usr/bin/env python3
"""Measure Carolus-ROS2's maximum update rate, with the input removed as a bottleneck.

WHY THIS EXISTS (2026-08-22). Hector asked three questions after reading
technical-ros2.tex: what is Carolus's max update rate on the PC, does running
Jazzy in a container cost performance, and what rate should we expect on the
Pi 5. None of them could be answered, because every run this project has ever
done was input-bound: the C920 offers only 10 FPS at 1280x720, so "10 Hz" is
the camera's hardware ceiling and says nothing about Carolus.

THE TRAP THIS SCRIPT IS BUILT AROUND. The obvious approach -- publish
synthetic frames fast and measure /pose -- silently measures the *publisher*
if the publisher is slower than the node. A 1280x720x3 frame is 2.7 MB; naive
Python republishing tops out well below what a compiled node can consume, and
the resulting number would look like a Carolus limit while being a harness
limit. So this script:

  * builds and serialises the Image message ONCE and only rewrites the
    header stamp per publish, which is what makes a high harness rate
    achievable from Python at all;
  * measures its OWN achieved publish rate alongside /pose;
  * REFUSES TO REPORT a max rate when the two are within HEADROOM_FRAC of
    each other, because that is the signature of an input-bound run.

A check that cannot come back negative is not a check. `--selftest` exercises
the refusal logic without needing ROS.

TWO PATHS ARE MEASURED SEPARATELY, because they cost very differently:
  --mode beacon  four synthetic blobs -> detection + sort + Ceres solve.
                 This is the number that answers "Carolus's update rate".
  --mode empty   black frame -> rejected at the contour stage. Upper bound on
                 the non-solve path; the gap between the two is the solver's
                 share.

USAGE
    ros2 run ... carolus_node --ros-args -p image_threshold:=150   # separate terminal
    python3 bench_carolus_rate.py --mode beacon --duration 30

    python3 bench_carolus_rate.py --selftest      # no ROS needed
"""
import argparse
import os
import statistics
import sys
import threading
import time

# BUG-102, same trap capture_checkerboard.py already hits: a 2026-08-07 pip
# install put numpy 2.2.6 in ~/.local/lib/python3.10/site-packages, shadowing
# the system numpy 1.21.5 that apt's python3-opencv was compiled against
# ("ImportError: numpy.core.multiarray failed to import"). PYTHONNOUSERSITE
# makes the interpreter skip ~/.local entirely, and it is only read at
# startup, so this re-execs once rather than asking anyone to remember a flag.
# The cv_bridge half of capture_checkerboard.py's guard is NOT needed here:
# that one is ROS1/Noetic-specific, and this script runs under ROS2.
if os.environ.get("_BENCH_CAROLUS_REEXEC") != "1" and not os.environ.get("PYTHONNOUSERSITE"):
    _env = dict(os.environ)
    _env["PYTHONNOUSERSITE"] = "1"
    _env["_BENCH_CAROLUS_REEXEC"] = "1"
    os.execve(sys.executable, [sys.executable] + sys.argv, _env)

# Matches carolus_node_ros2.cpp's declare_parameter defaults (lines 57-60), so
# the synthetic projection is self-consistent when the node runs with no
# --params-file. These are the RoboMaster's intrinsics, NOT the C920's -- see
# technical-ros2.tex's warning on this exact point.
FX, FY, CX, CY = 546.1957, 547.0838, 575.6041, 372.1876
Z_M = 0.7
POINTS_3D = [(0.0825, 0, 0), (-0.0825, 0, 0), (0, 0.072, 0), (0, 0, 0.0555)]
WIDTH, HEIGHT = 1280, 720

# Below this much headroom between harness and node, the run is input-bound
# and the node's rate is a lower bound, not a maximum.
HEADROOM_FRAC = 0.15

WARMUP_S = 3.0


def verdict(pub_hz, pose_hz, headroom_frac=HEADROOM_FRAC):
    """Decide whether pose_hz is a real ceiling or just the harness's rate.

    Returns (is_valid, text). Pure function so --selftest can exercise it.
    """
    if pose_hz <= 0.0:
        return False, "NO OUTPUT -- /pose never published; nothing measured."
    if pub_hz <= 0.0:
        return False, "NO INPUT -- publisher never ran; nothing measured."
    if pose_hz > pub_hz * (1.0 + headroom_frac):
        return False, (f"IMPLAUSIBLE -- /pose ({pose_hz:.1f} Hz) exceeds input "
                       f"({pub_hz:.1f} Hz). Check for a second publisher.")
    if pose_hz >= pub_hz * (1.0 - headroom_frac):
        return False, (f"INPUT-BOUND -- /pose {pose_hz:.1f} Hz vs harness "
                       f"{pub_hz:.1f} Hz ({100*pose_hz/pub_hz:.0f}% of input). "
                       f"The node kept up with everything it was given, so this "
                       f"is a LOWER BOUND, not a maximum. Speed up the harness "
                       f"(smaller frame, or a compiled publisher) and re-run.")
    return True, (f"VALID -- /pose {pose_hz:.1f} Hz against {pub_hz:.1f} Hz of "
                  f"input ({100*pose_hz/pub_hz:.0f}%). The node is the "
                  f"bottleneck, so {pose_hz:.1f} Hz is its maximum here.")


def selftest():
    cases = [
        ("saturated input", 40.0, 39.0, False),
        ("clear headroom", 200.0, 42.0, True),
        ("no output", 200.0, 0.0, False),
        ("no input", 0.0, 0.0, False),
        ("output above input", 40.0, 90.0, False),
        ("just inside the band", 100.0, 86.0, False),
        ("just outside the band", 100.0, 84.0, True),
    ]
    bad = 0
    for name, pub, pose, want in cases:
        got, text = verdict(pub, pose)
        ok = got == want
        bad += not ok
        print(f"  [{'ok ' if ok else 'FAIL'}] {name:24} pub={pub:6.1f} pose={pose:6.1f} "
              f"-> valid={got} (want {want})")
        if not ok:
            print(f"         {text}")
    print("selftest PASSED" if not bad else f"selftest FAILED ({bad})")
    return 1 if bad else 0


def build_frame(mode):
    import cv2
    import numpy as np
    img = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    if mode == "beacon":
        # Colour picked in HSV then converted: a hand-guessed BGR "blue" easily
        # lands below the preprocessing threshold once greyscaled, which is the
        # whole of BUG-130.
        hsv = np.uint8([[[115, 90, 255]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0].tolist()
        for X, Y, _ in POINTS_3D:
            u, v = int(FX * X / Z_M + CX), int(FY * Y / Z_M + CY)
            cv2.circle(img, (u, v), 14, bgr, -1)
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["beacon", "empty"], default="beacon")
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--pose-topic", default="/pose")
    ap.add_argument("--target-hz", type=float, default=0.0,
                    help="0 = publish as fast as possible (the point of the test)")
    ap.add_argument("--label", default="", help="free text echoed into the report")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge

    frame = build_frame(args.mode)

    class Bench(Node):
        def __init__(self):
            super().__init__("carolus_rate_bench")
            self.pub = self.create_publisher(Image, args.topic, qos_profile_sensor_data)
            self.pose_stamps = []
            self.create_subscription(
                self._pose_type(), args.pose_topic, self._on_pose,
                qos_profile_sensor_data)
            # Serialise ONCE. Rewriting only the stamp per publish is what makes
            # a useful harness rate reachable from Python.
            self.msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")
            self.msg.header.frame_id = "camera"

        @staticmethod
        def _pose_type():
            from geometry_msgs.msg import PoseStamped
            return PoseStamped

        def _on_pose(self, _):
            self.pose_stamps.append(time.perf_counter())

    rclpy.init()
    node = Bench()

    stop = threading.Event()
    pub_stamps = []

    def publish_loop():
        period = 1.0 / args.target_hz if args.target_hz > 0 else 0.0
        nxt = time.perf_counter()
        while not stop.is_set():
            now = node.get_clock().now().to_msg()
            node.msg.header.stamp = now
            node.pub.publish(node.msg)
            pub_stamps.append(time.perf_counter())
            if period:
                nxt += period
                d = nxt - time.perf_counter()
                if d > 0:
                    time.sleep(d)

    print(f"== carolus max-rate bench ==  mode={args.mode}  "
          f"target={'max' if args.target_hz == 0 else f'{args.target_hz} Hz'}  "
          f"frame={WIDTH}x{HEIGHT}" + (f"  [{args.label}]" if args.label else ""))
    print(f"   warmup {WARMUP_S:.0f}s (discarded), then {args.duration:.0f}s measured")

    t = threading.Thread(target=publish_loop, daemon=True)
    t.start()

    t0 = time.perf_counter()
    while time.perf_counter() - t0 < WARMUP_S:
        rclpy.spin_once(node, timeout_sec=0.01)
    pub_stamps.clear()
    node.pose_stamps.clear()

    t1 = time.perf_counter()
    while time.perf_counter() - t1 < args.duration:
        rclpy.spin_once(node, timeout_sec=0.01)
    elapsed = time.perf_counter() - t1

    stop.set()
    t.join(timeout=2.0)

    pub_hz = len(pub_stamps) / elapsed if elapsed > 0 else 0.0
    pose_hz = len(node.pose_stamps) / elapsed if elapsed > 0 else 0.0

    print(f"\n   frames published : {len(pub_stamps):6d}  -> {pub_hz:8.2f} Hz  (harness ceiling)")
    print(f"   /pose received   : {len(node.pose_stamps):6d}  -> {pose_hz:8.2f} Hz")
    if len(node.pose_stamps) > 2:
        d = [b - a for a, b in zip(node.pose_stamps, node.pose_stamps[1:])]
        print(f"   /pose interval   : mean {1000*statistics.mean(d):.2f} ms, "
              f"median {1000*statistics.median(d):.2f} ms, max {1000*max(d):.2f} ms")

    ok, text = verdict(pub_hz, pose_hz)
    print(f"\n   {text}")

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
