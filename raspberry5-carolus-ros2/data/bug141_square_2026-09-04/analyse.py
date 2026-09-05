#!/usr/bin/env python3
"""Re-derive every number quoted about these recordings, from the recordings.

    python3 analyse.py

Needs numpy and nothing else. No ROS, no camera, no robot -- it reads the
four .log files sitting beside it.

Written 2026-09-04, after a figure quoted in four documents ("0.95 m of depth
excursion") turned out to be the distance out to a single bad frame. The point
of shipping this next to the data is that the next person does not have to
take any of these numbers on faith.
"""
import os
import re
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))

# PoseFilter parameters, from carolus_node/config/logitech_1080p.yaml and the
# node's own defaults for the two it does not name.
FILTER_SIZE = 7
TRANSLATION_THRESHOLD = 0.5      # m
MAX_AGE_NS = 5 * 10**9           # max_time_fifo, 5 s
REJECT_LIMIT = 5

# Edge boundaries, in seconds from the start of recording 02. Read off the
# trajectory by hand; the four corners are visible as the axis of travel
# changing. Gaps between edges are the corners themselves, excluded.
EDGES = [("1  away  (depth)",   30.0, 48.0,  2),
         ("2  left  (lateral)", 50.0, 64.0,  0),
         ("3  back  (depth)",   66.0, 85.0,  2),
         ("4  right (lateral)", 88.0, 110.0, 0)]


def parse_pose_log(path):
    """-> (timestamps_ns, xyz). Parsed record-by-record, not by line offset.

    Note for anyone adapting this: z is the THIRD line after 'position:'.
    A grep -A2 finds only x and y and silently reports nothing wrong.
    """
    records = open(path).read().split("---")
    ts, ps = [], []
    for r in records:
        m_s = re.search(r"sec:\s*(\d+)\s*\n\s*nanosec:\s*(\d+)", r)
        m_p = re.search(r"position:\s*\n\s*x:\s*(-?[\d.eE+-]+)"
                        r"\s*\n\s*y:\s*(-?[\d.eE+-]+)"
                        r"\s*\n\s*z:\s*(-?[\d.eE+-]+)", r)
        if m_s and m_p:
            ts.append(int(m_s.group(1)) * 10**9 + int(m_s.group(2)))
            ps.append([float(m_p.group(i)) for i in (1, 2, 3)])
    return np.array(ts, dtype=np.int64), np.array(ps)


def pose_filter(ts, ps):
    """Replay of PoseFilter's translation path, transcribed from pose_filter.cpp.

    The rotation gate is omitted because these logs give positions only, so
    this accepts at least as many poses as the real filter would. Stated
    rather than hidden: it makes this an upper bound on filter benefit.
    """
    out = np.empty_like(ps)
    window, rejects, has_accepted, last_accepted_ns = [], 0, False, 0
    for i in range(len(ps)):
        if window and has_accepted and (ts[i] - last_accepted_ns) > MAX_AGE_NS:
            window.clear()
            rejects = 0
        if window and np.linalg.norm(window[-1] - ps[i]) > TRANSLATION_THRESHOLD:
            rejects += 1
            if rejects > REJECT_LIMIT:
                window.clear()
                rejects = 0
            out[i] = ps[i] if not window else window[-1]
            continue
        window.append(ps[i])
        last_accepted_ns, has_accepted = ts[i], True
        if len(window) > FILTER_SIZE:
            window.pop(0)
        out[i] = np.mean(np.array(window), axis=0)
    return out


def straightness(seg, travel_axis):
    """Perpendicular RMS and max deviation from the best-fit line."""
    other = 0 if travel_axis == 2 else 2
    a, b = np.polyfit(seg[:, travel_axis], seg[:, other], 1)
    perp = (seg[:, other] - (a * seg[:, travel_axis] + b)) / np.sqrt(1 + a * a)
    return (np.sqrt(np.mean(perp ** 2)), np.max(np.abs(perp)),
            abs(np.ptp(seg[:, travel_axis])))


def main():
    print("=" * 74)
    print("BUG-141 square-path recordings -- every quoted number, re-derived")
    print("=" * 74)

    for tag, f in (("01 BEFORE the fix", "01_before_fix_pose.log"),
                   ("02 AFTER the fix", "02_after_fix_pose.log")):
        ts, ps = parse_pose_log(os.path.join(HERE, f))
        dt = np.diff(ts) / 1e9
        print(f"\n### {tag}")
        print(f"  samples              {len(ps)}")
        print(f"  duration             {(ts[-1] - ts[0]) / 1e9:.1f} s "
              f"at {1 / np.median(dt):.2f} Hz")
        print(f"  poses with z >= 0    {int((ps[:, 2] >= 0).sum())}"
              f"   <-- the whole point: 685 before, 0 after")

    ts, ps = parse_pose_log(os.path.join(HERE, "02_after_fix_pose.log"))
    t = (ts - ts[0]) / 1e9

    print("\n" + "=" * 74)
    print("THE OUTLIER, and why 0.95 m was wrong")
    print("=" * 74)
    i = int(np.argmax(ps[:, 2]))
    z_clean = np.delete(ps[:, 2], i)
    print(f"  worst frame: index {i}, t={t[i]:.2f}s, z={ps[i, 2]:+.4f} m")
    print(f"  z span including it   {np.ptp(ps[:, 2]):.4f} m   <- the figure once quoted")
    print(f"  z span excluding it   {np.ptp(z_clean):.4f} m   <- the square's depth edge")

    print("\n" + "=" * 74)
    print("IS IT A SQUARE?  (the question Hector actually asked)")
    print("=" * 74)
    filt = pose_filter(ts, ps)
    print(f"  {'edge':<20}{'length':>9}{'RAW rms':>10}{'RAW max':>10}"
          f"{'FILT rms':>10}{'FILT max':>10}")
    lengths, dirs = [], []
    for name, t0, t1, ax in EDGES:
        m = (t >= t0) & (t <= t1)
        r_rms, r_max, ln = straightness(ps[m], ax)
        f_rms, f_max, _ = straightness(filt[m], ax)
        lengths.append(ln)
        seg = ps[m][:, [0, 2]]
        d = seg[-1] - seg[0]
        dirs.append(d / np.linalg.norm(d))
        print(f"  {name:<20}{ln * 100:7.1f}cm{r_rms * 1000:8.2f}mm"
              f"{r_max * 1000:8.2f}mm{f_rms * 1000:8.2f}mm{f_max * 1000:8.2f}mm")
    print(f"\n  edge lengths {', '.join(f'{x * 100:.1f}' for x in lengths)} cm "
          f"(nominal 30, mean {np.mean(lengths) * 100:.1f})")
    for k in range(4):
        ang = np.degrees(np.arccos(np.clip(np.dot(dirs[k], dirs[(k + 1) % 4]), -1, 1)))
        print(f"  corner {k + 1}: {ang:6.2f} deg   (ideal 90)")
    print("\n  Straightness of 2-7 mm over ~30 cm edges is 0.7-2.4% of edge length.")
    print("  This trajectory does not curve.")

    print("\n" + "=" * 74)
    print("WOULD fifo_on:true HELP?")
    print("=" * 74)
    step = np.linalg.norm(np.diff(ps, axis=0), axis=1)
    print(f"  inter-sample step: p50 {np.median(step) * 1000:.2f} mm, "
          f"p95 {np.percentile(step, 95) * 1000:.2f} mm, max {step.max() * 1000:.0f} mm")
    print(f"  steps > 5 cm between frames (impossible by hand): "
          f"{int((step > 0.05).sum())} of {len(step)}")
    print("\n  Its value here is outlier rejection, not smoothing: on the three")
    print("  clean edges it changes the RMS by well under a millimetre, while on")
    print("  edge 4 it takes 25.15 mm down to 4.64 mm by absorbing one bad frame.")
    lag = FILTER_SIZE * np.median(np.diff(ts) / 1e9) / 2
    print(f"  Cost: a {FILTER_SIZE}-pose window at "
          f"{1 / np.median(np.diff(ts) / 1e9):.1f} Hz trails real motion by "
          f"~{lag * 1000:.0f} ms.")

    print("\n" + "=" * 74)
    print("CANDIDATE SWITCHING (from the node log)")
    print("=" * 74)
    vals = [int(m.group(1)) for m in re.finditer(
        r"candidate=(\d+)", open(os.path.join(HERE, "02_after_fix_node.log")).read())]
    switches = sum(1 for a, b in zip(vals, vals[1:]) if a != b)
    from collections import Counter
    print(f"  {len(vals)} decisions, {switches} switches "
          f"({100 * switches / (len(vals) - 1):.1f}% of frames)")
    print(f"  distribution: {dict(sorted(Counter(vals).items()))}")
    print("  These lines carry no timestamp, so they CANNOT be correlated")
    print("  sample-by-sample against the poses above. Standalone figure only.")


if __name__ == "__main__":
    main()
