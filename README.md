# Carolus / RoboMaster S1

Vision-based relative navigation (Carolus/UVGS-2: 4-LED beacon detection + P4P
pose solve) integrated on a rooted RoboMaster S1, fused with wheel odometry via
`robot_localization`.

## License

Apache License 2.0 — see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

This matches what the third-party components already declared rather than
overriding them: `src/ff_msgs/` is NASA Astrobee code (Apache-2.0) and
`src/libuvgs_astrobee/` (Carolus/uVGS-2, original author zauberflote1)
declares Apache-2.0 in its own `package.xml`. `NOTICE` lists each component,
its origin, and what was modified here.


## Versioning

Per the project's supervisor request (branches for the useful part of the
inherited code, then new versions/changes on top):

- **Branch `quentin-legacy`** (tag `v0`) — the inherited baseline:
  `libuvgs_astrobee` (the P4P/beacon detection core), `ff_msgs` (Astrobee
  message definitions, a build dependency of the inherited code), and the
  original top-level `launch/`/`moveuvgs.cc`/`dockerROS1`. History preserved
  from the original repository (29 commits).
- **Branch `nawfel-legacy`** (tag `v1` at the current tip, and the
  repository's default branch) — this stage's
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

## Running Carolus on a different robot

Carolus itself is **robot-agnostic**: the detection and P4P solver
(`src/libuvgs_astrobee/`) contain no reference to DJI, RoboMaster or RNDIS.
It takes a camera image on a ROS topic and publishes a pose. Everything
specific to one robot lives in a YAML profile, not in the source.

To run it on other hardware, you need three things — none of which requires
editing any code:

1. **Calibrate your camera.** You need `fx`, `fy`, `cx`, `cy` and the
   plumb_bob distortion coefficients. The MATLAB Camera Calibration Toolbox
   procedure is documented in `overleaf/technical.pdf` (calibration chapter),
   along with a Kalibr alternative.

2. **Measure your beacon.** Four LED positions in metres, in Carolus's
   left-handed frame, with **one point deliberately off the plane of the
   other three** — P4P needs that to resolve pose unambiguously.

3. **Write your profile.** Copy `src/carolus_node/config/robomaster_s1.yaml`,
   substitute your values, and launch:

```bash
roslaunch carolus_node carolus.launch \
    config:=$(rospack find carolus_node)/config/my_robot.yaml \
    camera_topic:=/my/camera/image_raw
```

On Ubuntu 20.04 (including a Raspberry Pi), also pass
`ubuntu2204_preload:=false` — the `LD_PRELOAD` workaround it controls is
only needed on 22.04, which is not an officially supported ROS Noetic
target.

The `robomaster_cam/` package is the DJI SDK bridge for this specific robot
and is **not** needed on other hardware — supply the camera stream however
your platform already does, and point `camera_topic` at it.

`testcarolus.launch` remains the RoboMaster S1 entry point: it loads the S1
profile and adds an S1-specific static transform, then delegates to
`carolus.launch`.

## Testing

Full step-by-step reproduction guide, from a fresh Pi and a fresh S1 through
to a running Carolus pipeline: **[`overleaf/technical.pdf`](overleaf/technical.pdf)**,
in this repository. It is self-contained — powering on, rooting the S1,
Raspberry Pi and ROS setup, RNDIS networking, the camera bridge, camera
calibration, and building/launching Carolus, including how to run it on a
robot other than this one.

Prerequisite for every session: robot powered on (double chime), Pi
reachable at its RNDIS/Wi-Fi address, `eth1` interface up on the Pi with an
address on the robot's subnet.

Launching the stack:

```bash
python3 shortcuts/carolus_launcher.py
```

Then, in order, the launcher's 5 buttons:

| Button | What runs | Unlocked when |
|---|---|---|
| 1 · roscore + Pi | SSH → `eth1 up` + `roscore` | port 11311 open |
| 2 · Camera + Beacon | SSH → `rm_cam_beacon.py` + `cam_view_helper.py` | `/camera/color/image_raw` published |
| 3 · Carolus Astrobee | `roslaunch carolus_node testcarolus.launch` | manual |
| 4 · TF Broadcaster | SSH → `carolus_tf_broadcaster.py` on the Pi | manual |
| 5 · Docking | `beacon_docking.py` (lab PC) — `ALIGN_ONLY`/`APPROACH_ONLY`/`START`/`ABORT` panel | first `[DOCKSTATUS]` line seen |

**Sanity check that the pipeline is actually working**: with the terminals
running and an LED beacon in the camera's field of view, `rostopic hz /pose`
should report a steady rate. Carolus running on the Raspberry Pi processes
~24 frames/s (measured 2026-08-04); running it on the lab PC instead drops
this to ~2.5 Hz, because every uncompressed 1280x720 frame has to cross the
network first — see the technical manual for why the Pi is the right target. `rostopic echo /pose` should show
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

`overleaf/` holds two LaTeX documents:

- **`technical.pdf` / `technical.tex`** — the self-contained technical manual
  (power-on through building and launching Carolus). This is the one to read
  to set the system up.
- **`main.tex`** — the progress report for the project supervisor.

See `overleaf/README.md` for the difference and for how to compile them.

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
