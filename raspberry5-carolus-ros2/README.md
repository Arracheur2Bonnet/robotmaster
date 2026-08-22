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
| Pose values trustworthy | **no** — solver reports `converged=0` on both targets; see the manual's warningbox |

## The one thing to know before running it

A lit blue beacon can be invisible to the detector while looking perfectly
fine on screen: its hue and saturation pass, but saturated blue converts to a
*low* grayscale value. Symptom is `0 contours found` on every frame with the
beacon plainly visible. Measured on this camera at grayscale 131, well under
the ROS1 profile's inherited default of 190 — `image_threshold: 100` is now
`logitech_1080p.yaml`'s own default, confirmed on both the lab PC and the
Pi 5. A different camera or beacon still needs its own measurement — the
manual explains how, rather than guessing a new number.

## Relationship to the ROS1 project

The four core `.cpp`/`.hpp` files are a **copy** of the ROS1 node's own
detection/solver code (`carolus_ws/src/libuvgs_astrobee/` in the parent
repository), duplicated here on purpose so this folder stays portable.

**That copy does not update itself.** If the ROS1 originals are fixed, the
change has to be brought across by hand. This is a deliberate trade —
portability over a single source of truth — not an oversight.
