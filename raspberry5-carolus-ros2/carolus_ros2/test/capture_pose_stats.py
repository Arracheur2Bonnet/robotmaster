#!/usr/bin/env python3
"""Capture N /pose samples, print each, then mean+std per axis.
Protocol 23 (protocoles-terrain.md) -- BUG-132 displacement re-validation.

Usage: python3 capture_pose_stats.py [--n 20] [--label "before"]
"""
import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class Capture(Node):
    def __init__(self, n):
        super().__init__('capture_pose_stats')
        self.n = n
        self.samples = []
        self.create_subscription(PoseStamped, '/pose', self.cb, 10)

    def cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        self.samples.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        sys.stdout.write(f"\r  sample {len(self.samples):3d}/{self.n}  "
                         f"pos=({p.x:+.4f},{p.y:+.4f},{p.z:+.4f})")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--label", type=str, default="capture")
    ap.add_argument("--cluster", action="store_true",
                    help="group samples into clusters, report count and sequence")
    ap.add_argument("--cluster-tol", type=float, default=0.01,
                    help="metres; samples within this of a cluster mean join it")
    args = ap.parse_args()

    rclpy.init()
    node = Capture(args.n)
    print(f"=== {args.label}: waiting for {args.n} /pose samples ===")
    while rclpy.ok() and len(node.samples) < args.n:
        rclpy.spin_once(node, timeout_sec=1.0)
    print()

    arr = np.array(node.samples)
    pos = arr[:, :3]
    mean = pos.mean(axis=0)
    std = pos.std(axis=0)
    print(f"\n=== {args.label}: {len(node.samples)} samples ===")
    print(f"  mean position (x,y,z) = ({mean[0]:.6f}, {mean[1]:.6f}, {mean[2]:.6f})")
    print(f"  std  position (x,y,z) = ({std[0]:.6f}, {std[1]:.6f}, {std[2]:.6f})")
    print(f"  RESULT_MEAN {mean[0]:.6f} {mean[1]:.6f} {mean[2]:.6f}")
    print(f"  RESULT_STD {std[0]:.6f} {std[1]:.6f} {std[2]:.6f}")

    if args.cluster:
        # Greedy clustering: two samples are the same cluster if within
        # `tol` metres of the cluster's running mean. Order-independent
        # enough for this purpose -- the point is counting distinct groups,
        # not a rigorous clustering algorithm.
        tol = args.cluster_tol
        clusters = []  # list of [count, sum_xyz]
        assign = []
        for row in pos:
            best = None
            for ci, (cnt, s) in enumerate(clusters):
                if np.linalg.norm(row - s / cnt) < tol:
                    best = ci
                    break
            if best is None:
                clusters.append([1, row.copy()])
                assign.append(len(clusters) - 1)
            else:
                clusters[best][0] += 1
                clusters[best][1] += row
                assign.append(best)
        print(f"\n  --- clustering at tol={tol*1000:.1f} mm: "
             f"{len(clusters)} distinct cluster(s) ---")
        for ci, (cnt, s) in enumerate(clusters):
            m = s / cnt
            print(f"    cluster {ci}: n={cnt:3d}  mean=({m[0]:+.4f},{m[1]:+.4f},{m[2]:+.4f})")
        print(f"  sequence of cluster ids over time: {assign}")

    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
