# Square-path recordings, 2026-09-04 — the twisted-pair ambiguity and its fix

Four raw logs from the two recordings that found and then validated the
cheirality fix described in `technical-ros2.tex` and in the report's section
on the mirror-image pose. Kept because no further hardware sessions are
planned and these cannot be re-recorded.

## Conditions, identical for both recordings

| | |
|---|---|
| Machine | Raspberry Pi 5 (`maglev-desktop`), Ubuntu 24.04.4, ROS2 Jazzy |
| Camera | Logitech C920, exposure 40 set via `v4l2-ctl` |
| Config | `carolus_ros2/config/logitech_1080p.yaml`, `multi_hypothesis_sort: true` |
| Distance | ~1 m, close to fronto-parallel |
| Motion | beacon dragged by hand around a 30 cm square, camera fixed |
| Filtering | none — the ROS2 node has no `PoseFilter`, output is raw per-frame |

There is **no independent ground truth** in either recording: the path was
traced by hand against a marked square, so residual waviness measures the
steadiness of a human arm at least as much as it measures Carolus. Treat the
square's nominal 30 cm as an approximate scale, not a reference.

## The files

| File | What it is |
|---|---|
| `01_before_fix_node.log` | Node stdout, **before** the fix. Carries the per-frame `candidate=` and `final_cost=` lines. |
| `01_before_fix_pose.log` | `/pose` echo, **before** the fix. 1469 published poses. |
| `02_after_fix_node.log` | Node stdout, **after** the fix. |
| `02_after_fix_pose.log` | `/pose` echo, **after** the fix. 3037 published poses. |

The `pose` and `node` logs are separate processes and are correlated by
timestamp, not by line number. A 30 ms nearest-timestamp match pairs 1417 of
1469 poses in recording 01 and 2785 of 3037 in recording 02; the unmatched
remainder is a timing artefact of two independent streams, not dropped data.
**Counts quoted from the node log and counts quoted from the matched subset
will differ, and both are correct** — say which one you mean. (Example: the
winning candidate 14 appears 331 times in `02_after_fix_node.log` and 288
times among matched poses.)

## What each recording shows

**`01` — the bug.** `x` and `y` trace a plausible path throughout. `z` runs
at about −0.99 m for roughly the first half, jumps to about +0.99 m partway
through, and never returns. Not oscillation: one sustained sign flip.
Correlating against the node log, candidate 15 gives z<0 on 699 of 699
samples and candidate 14 gives z≥0 on 586 of 586. Those two candidates differ
only in which detected blob is called P1 and which P2 — mirror points about
the beacon's own symmetry plane — so candidate 14 is a reflection of the real
geometry that fits the image about as well, to within 0.3 mm in reported
distance.

**`02` — the fix.** Same setup, with the cheirality check in place: any
candidate placing the beacon behind the camera (`t.z >= 0`) is skipped inside
the search loop. 3037 poses. z spans −1.3137 to −0.3638 m, but **that 0.95 m
figure is misleading and was quoted as real for a while**: −0.3638 m is a
single bad detection frame (index 2692, t=100.47 s). Excluding it the depth
range is −1.3137 to −0.9753 m, i.e. **0.338 m** — the square's own depth edge,
which matches the tape. Measured offline from these very files: edges of
28.8 / 33.2 / 30.7 / 30.2 cm, corners square to within 4°, straightness
2–7 mm RMS per edge. **The path does not curve.** Six different candidates win
across the recording — 15: 1520, 7: 1138, **14: 331**, 3: 35, 6: 12, 9: 1 —
and **zero** samples report z ≥ 0. Candidate 14 still wins legitimately: the
check disqualifies wrong-signed *solutions*, not candidate *numbers*.
`final_cost` p50 0.028, p90 0.132, p99 0.443, plus one outlier at 2498
(a momentary bad detection during hand-held motion; it carries no sign flip).

## Reading them

Both `*_pose.log` files are plain `ros2 topic echo` output. The node logs are
plain stdout. Nothing is compressed or post-processed — these are the files as
written during the sessions.

```bash
# distribution of winning candidates
grep -o 'candidate=[0-9]*' 02_after_fix_node.log | sort | uniq -c | sort -rn
#   1520 candidate=15 / 1138 candidate=7 / 331 candidate=14 / 35 / 12 / 1

# how many poses sit on the wrong side of the camera?
grep -A3 'position:' 02_after_fix_pose.log | grep '^\s*z:' | awk '$2 >= 0' | wc -l
#   -> 0     (after the fix)
grep -A3 'position:' 01_before_fix_pose.log | grep '^\s*z:' | awk '$2 >= 0' | wc -l
#   -> 685   (before the fix -- run this one too, or the 0 above proves nothing)

# sanity check that the pattern matched every pose at all
grep -A3 'position:' 02_after_fix_pose.log | grep -c '^\s*z:'   # -> 3037
grep -A3 'position:' 01_before_fix_pose.log | grep -c '^\s*z:'  # -> 1469
```

**`-A3`, not `-A2`.** `z` is the third line after `position:`. With `-A2` both
commands return `0` — including the one that should return 685 — which reads
exactly like a clean result. Run the count line above whenever you change the
pattern; a filter that matches nothing and a filter that finds nothing wrong
are indistinguishable otherwise.

**On 685 vs the 586 quoted elsewhere:** 685 is every wrong-signed pose in the
pose log; 586 is the subset that also paired to a `candidate=14` line within
the 30 ms window. Same relationship as 331 vs 288 above.
