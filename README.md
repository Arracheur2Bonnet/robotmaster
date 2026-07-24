# Carolus / RoboMaster S1

Vision-based relative navigation (Carolus/UVGS-2: 4-LED beacon detection + P4P
pose solve) integrated on a rooted RoboMaster S1, fused with wheel odometry via
`robot_localization`.

## Versioning

This repository is tagged in two stages, per the project's supervisor request:

- **`v0`** — the inherited baseline as it existed before this stage's work:
  `libuvgs_astrobee` (the P4P/beacon detection core), `ff_msgs` (Astrobee
  message definitions, a build dependency of the inherited code), and the
  original top-level `launch/`/`moveuvgs.cc`/`dockerROS1`.
- **`v1`** (current) — this stage's contribution: `carolus_node` (TF
  broadcasting, `robot_localization` EKF integration, absolute pose from
  beacon detection), `robomaster_cam` (RoboMaster S1 SDK bridge,
  LOCATE/SEARCH/ALIGN/APPROACH state machine), `cmake_shims` (local build
  fixes for ROS Noetic on Ubuntu 22.04), and `shortcuts` (the Tkinter
  launcher GUI and operational scripts). `libuvgs_astrobee` also received one
  targeted update in v1: recalibrated camera intrinsics in
  `carolus_astrobee.cpp` for the actual mounted camera.

## Building

Requires ROS Noetic. `catkin build` from the workspace root (`src/` here is
the catkin `src/` directory).

**Not included in this repository** (install separately, standard upstream
packages, unmodified):
- [`robot_localization`](https://github.com/cra-ros-pkg/robot_localization),
  branch `noetic-devel`, commit `9ef26a57f` — the loosely-coupled EKF used by
  `carolus_node`. Left out to keep this repository to the code actually
  written/modified for this project (the upstream clone alone is ~36 MB).

See `shortcuts/README.md` for the operational scripts (launcher, deployment,
leak scan) and `cmake_shims/` for the Ubuntu 22.04 build workarounds (no
`sudo` required).

---

## Legacy notes (original inherited README, `v0`)

# p4p-zbft
P4P Solver on Manifolds by ZBFT

## CHANGES
- CLEAN UP OF OLD METHODS, EXTRA VARS, AND BLOB WINDOW
- ADD QUEUE SIZE, CONTROL IMAGE FREQUENCY FOR PROCESSING
- USES CONDITIONAL VARIABLE (NOTIFY_ONE()) TO NOTIFY PROCESS IMAGE THREAD
- PROCESS IMAGE THREAD RUNS UNTIL QUEUE IS EMPTY
- SUPPORTS BAYER IMAGES
- CPU REDUCTION DONE
- FISHEYE MODEL
- MONO IMAGES SEPARATE PIPELINE
- TENTATIVE ASTROBEE PREPROCESSING
- FOV MODEL
- ADD EQUIVALENCY TO ASTROBEE COORDINATE FRAME

## TODO NEAR FUTURE

- HANDLE CASE WITH MULTIPLE TARGETS IN VIEW
- PUBLISH MSGS AS MARKER TRACKING

@1822 TROPICAL EMPIRE -- ZBFT
