# Carolus / RoboMaster S1

Vision-based relative navigation (Carolus/UVGS-2: 4-LED beacon detection + P4P
pose solve) integrated on a rooted RoboMaster S1, fused with wheel odometry via
`robot_localization`.

## Versioning

Per the project's supervisor request (branches for the useful part of the
inherited code, then new versions/changes on top):

- **Branch `quentin-legacy`** (tag `v0`) — the inherited baseline:
  `libuvgs_astrobee` (the P4P/beacon detection core), `ff_msgs` (Astrobee
  message definitions, a build dependency of the inherited code), and the
  original top-level `launch/`/`moveuvgs.cc`/`dockerROS1`. History preserved
  from the original repository (29 commits).
- **Branch `main`** (tag `v1` at the current tip) — this stage's
  contribution on top of the legacy branch: `carolus_node` (TF broadcasting,
  `robot_localization` EKF integration, absolute pose from beacon
  detection), `robomaster_cam` (RoboMaster S1 SDK bridge,
  LOCATE/SEARCH/ALIGN/APPROACH state machine), `cmake_shims` (local build
  fixes for ROS Noetic on Ubuntu 22.04), `shortcuts` (the Tkinter launcher
  GUI and operational scripts), and `overleaf/` (report). `libuvgs_astrobee`
  also received one targeted update on `main`: recalibrated camera
  intrinsics in `carolus_astrobee.cpp` for the actual mounted camera.

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

## Testing

Full step-by-step reproduction guide (fresh Pi/S1 to first launch):
`research-log/08-guide-demarrage.md` in the project's working repository
(not duplicated here — see that file for hardware setup, SDK install, and
RNDIS networking).

Prerequisite for every session: robot powered on (double chime), Pi
reachable at its RNDIS/Wi-Fi address, `eth1` interface up on the Pi with an
address on the robot's subnet.

Launching the stack:

```bash
python3 shortcuts/carolus_launcher.py
```

Then, in order, the launcher's 4 buttons:

| Button | What runs | Unlocked when |
|---|---|---|
| 1 · roscore + Pi | SSH → `eth1 up` + `roscore` | port 11311 open |
| 2 · Camera + Beacon | SSH → `rm_cam_beacon.py` + `cam_view_helper.py` | `/camera/color/image_raw` published |
| 3 · Carolus Astrobee | `roslaunch carolus_node testcarolus.launch` | manual |
| 4 · TF Broadcaster | SSH → `carolus_tf_broadcaster.py` on the Pi | manual |

**Sanity check that the pipeline is actually working**: with all 4
terminals running and an LED beacon in the camera's field of view,
`rostopic hz /pose` should report ~2.5 Hz. `rostopic echo /pose` should show
a `geometry_msgs/PoseStamped` with a plausible Z distance matching the
beacon's real distance from the camera. In the launcher GUI, the
`BEACON: DETECTED` indicator should light up.

**Known non-blocking warnings** you'll see on a normal launch (documented in
`CLAUDE.md`, not worth re-diagnosing): a missing
`~/catkin_ws/devel/setup.bash` line in the lab PC's `.bashrc` (leftover from
a previous setup), a ROS log-directory-over-1GB warning on the Pi, and a
`pillow`/`imageio` version mismatch from the `myqr` dependency (unused in
this RNDIS-based pipeline).

## Report

`overleaf/` holds the English-language LaTeX report for the project
supervisor (skeleton only for now — see `overleaf/README.md`).

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
