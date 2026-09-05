# `carolus_ws/` — the ROS1 workspace

This is what actually runs on the robot. Full setup and operation
instructions are in [`overleaf/technical.tex`](../overleaf/technical.tex)
(compile it yourself; see the root [`README.md`](../README.md)) — this file
is a map of what's in `src/`, not a how-to.

## Packages

| Package | Ours or vendored | What it is |
|---|---|---|
| `libuvgs_astrobee` | Inherited (original author: zauberflote1), modified here | The Carolus/UVGS-2 algorithm itself: LED detection, target correspondence, the Ceres P4P solve, the outlier filter, and the ROS node wrapping them. Own [README](src/libuvgs_astrobee/README.md) (the original author's, kept as written). |
| `carolus_node` | Ours | Launch files and per-robot config profiles (`config/*.yaml`), the TF-related scripts (`carolus_ground_tf.py`, `carolus_tf_broadcaster.py`, `beacon_absolute_pose.py`) that convert a Carolus pose into the ROS frame and, from there, an EKF-ready odometry input. |
| `robomaster_cam` | Ours | The RoboMaster S1 SDK bridge — see below. |
| `ff_msgs` | Vendored (NASA Astrobee) | Message definitions `libuvgs_astrobee` depends on, inherited unmodified. |
| `usb_cam` | Vendored (Bosch/ROS community) | Generic ROS1 USB camera driver, used for the Logitech-webcam ground-test setup rather than the robot's own camera. Locally patched for a newer host than upstream targets — see `NOTICE`. |
| `cmake_shims/` | Ours (not a ROS package) | Ubuntu-22.04 build workarounds. See its own [README](cmake_shims/README.md). |

## `robomaster_cam/scripts/` — which file does what

Six scripts live here with no other grouping to tell them apart at a
glance:

- **`rm_cam_beacon.py`** — the one that matters continuously. The single
  RoboMaster SDK connection: camera bridge, beacon-tracking gimbal lock,
  IMU/odometry publishing, MANUAL-mode chassis and gimbal control. Runs for
  the entire duration of every session.
- **`beacon_docking.py`**, **`gimbal_bearing.py`** — secondary features on
  top of the main pipeline (fixed-pose docking against the beacon; a coarse
  map-and-odometry gimbal pre-aim stage, complementary to the fine visual
  lock in `rm_cam_beacon.py`). Neither is wired into
  `shortcuts/carolus_launcher.py`'s current button set.
- **`map_collision.py`** — reads the map JSON produced by
  `shortcuts/map_editor.py` for obstacle-aware navigation.
- **`test_gimbal_sweep.py`**, **`test_gimbal_sweep2.py`** — one-off,
  isolated SDK diagnostic scripts, each written to answer a specific
  question (whether gimbal moves are blocking or async; which yaw
  reference frame to trust) before a past redesign of the SEARCH/APPROACH
  logic. Not part of the running pipeline; kept for reference rather than
  deleted, since the questions they answered could resurface.

**A note on language:** most of this project's source is in English. A
handful of files still carry comments or docstrings in French from earlier
in development, most extensively in `beacon_docking.py` and
`rm_cam_beacon.py`. Functionally unaffected, but flagged here rather than
left for a reader to wonder about — not yet swept, since a translation pass
touching this many lines of code that drives a physical robot deserves a
deliberate, careful pass of its own rather than a quick edit.
