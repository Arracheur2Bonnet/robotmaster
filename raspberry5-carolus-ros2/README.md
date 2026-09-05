# Carolus under ROS2 — lab PC and Raspberry Pi 5

Self-contained ROS2 port of Carolus's beacon-detection and pose-estimation
pipeline. **This folder builds and runs on its own** — copy it anywhere, no
other part of the parent repository is needed.

Carolus detects a physical 4-LED beacon in a camera frame and solves a
Perspective-4-Point problem for the camera's 6-DOF pose relative to it.
Originally NASA's SVGS system for the Astrobee free-flyer, adapted here for a
DJI RoboMaster S1. **No robot is involved anywhere in this folder** — both
targets run against a plain USB webcam.

## Start here

`technical-ros2.tex` is the manual. Read it rather than this file — it carries
the install commands, the verified results, and the traps. This README only
says what is where.

| Path | What |
|---|---|
| `technical-ros2.tex` | The manual: lab PC (ch. 1) and Raspberry Pi 5 (ch. 2) |
| `carolus_ros2/src/carolus_node_ros2.cpp` | The `rclcpp` wrapper — ROS2-specific code |
| `carolus_ros2/src/{beacon_detector,ceresP4P,pose_est,pose_filter}.cpp` | The ROS-free core: detection, P4P solve, target sort, FIFO filter |
| `carolus_ros2/include/carolus_node/` | Headers for the above |
| `carolus_ros2/config/logitech_1080p.yaml` | Camera intrinsics + beacon geometry + detection parameters |

This and the parent repo's `carolus_ws/` (ROS1) are the only two maintained
copies of this code — no third, comparison-only tree exists.
`shortcuts/ros2_sync_check.sh` (parent repo) catches drift between the two;
the ROS1-vs-ROS2 comparison itself is in the manual, §"Where this node
genuinely differs from the ROS1 one".

## Quick build

```bash
source /opt/ros/humble/setup.bash          # or /opt/ros/jazzy on the Pi 5
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
ln -s /path/to/raspberry5-carolus-ros2/carolus_ros2 src/carolus_ros2  # replace with this folder's actual path
colcon build --packages-select carolus_ros2
```

Full install (ROS2 itself, the camera driver, the C++ dependencies) is in the
manual — this assumes ROS2 is already present.

## Status

| | |
|---|---|
| Builds on ROS2 Humble (x86_64) | verified 2026-08-18; rebuilt clean 2026-08-21 after a Pi 5 fix to the shared source, runtime not re-tested that day (no webcam attached) |
| Builds on ROS2 Jazzy (container) | verified 2026-08-18 |
| Detects a real beacon, publishes `/pose` | verified 2026-08-20, lab PC + Logitech C920; verified again 2026-08-21, Pi 5 + same beacon |
| Builds/runs on Raspberry Pi 5 (aarch64, native) | verified 2026-08-21 — build and webcam confirmed; beacon detection in the row above |
| Maximum update rate | **measured 2026-08-22, confirmed on real bare-metal 24.04 hardware 2026-08-24:** ~265 Hz lab PC (native or container, ±0.6%), 52.8 Hz Pi 5, 203.7 Hz on a second dedicated 24.04 machine — all far above the 10 FPS camera ceiling; see the manual's "Maximum update rate" section |
| Guide load-tested end to end | **2026-08-24, second blank 24.04 machine, followed literally** — found 3 real gaps (no clone instructions, a missing Jazzy-host build recipe, a DDS-discovery timing note), all fixed in the manual the same day |
| Pose values trustworthy | **root cause found, fixed, and hardware-validated 2026-08-25 (BUG-132 + BUG-088).** The ROS2 port had dropped a pixel-space conversion (BUG-132, `converged=0→1` after the fix); a second defect (BUG-088, no warm start between frames) meant an independently-restarted reading could land in its own disconnected local minimum. **Real hardware, both fixed, node left running through a real 30 cm move: 71.2% response, correct direction** — up from 1.43% pre-fix and from wrong-direction 45-55% with BUG-132 alone. Not 100%: this camera's own intrinsics are still "approximate, never formally calibrated" — now a fair question to ask, since the pose is actually tracking. See the manual's `sec:bug088` section |
| Correspondence sort stable | **no — confirmed unstable on real hardware, geometry-dependent, deliberately left unfixed.** The sort compares unit bearing vectors (angular separation), not the beacon's real dimensions. Synthetically: 1000 trials at a near-tie selected 5 distinct groups, 134 outright failures. **On real hardware, 2026-08-25: 60 cm gave 8 distinct pose clusters over 40 stationary samples; 90 cm and 120 cm on the same rig were each rock-stable** — the unstable zone is wider than one synthetic point. Warm-starting (above) mitigates its practical impact for continuous tracking but does not fix the sort itself. Reproduce with `carolus_ros2/test/instrument_p4p_sort.cpp` (synthetic) or `capture_pose_stats.py --cluster` (real hardware) |

## The one thing to know before running it

A lit blue beacon can be invisible to the detector while looking perfectly
fine on screen: its hue and saturation pass, but saturated blue converts to a
*low* grayscale value. Symptom is `0 contours found` on every frame with the
beacon plainly visible — the brightness cut runs *before* any colour test, so
the frame is already black by the time hue would be checked. Measured on this
camera at grayscale 131, well under the RoboMaster profile's 190, which this
file inherited unexamined when it was first copied. `image_threshold: 100` is
now `logitech_1080p.yaml`'s own default, confirmed on both the lab PC and the
Pi 5, and since 2026-08-21 the ROS1 webcam profile carries the same value for
the same measured reason. A different camera or beacon still needs its own
measurement — the manual explains how, rather than guessing a new number.

## Relationship to the ROS1 project

The four core `.cpp`/`.hpp` files are a **copy** of the ROS1 node's own
detection/solver code (`carolus_ws/src/libuvgs_astrobee/` in the parent
repository), duplicated here on purpose so this folder stays portable.

**That copy does not update itself.** If the ROS1 originals are fixed, the
change has to be brought across by hand. This is a deliberate trade —
portability over a single source of truth — not an oversight.
