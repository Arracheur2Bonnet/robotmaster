#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Beacon docking -- a FIXED, repeatable final position relative to the beacon.

Goal (supervisor request): whatever angle the robot arrives from, it must always
end up in the SAME place relative to the beacon -- DOCK_DISTANCE_M in front of
it, on the beacon's frontal axis, facing it. The user's analogy: a car parking in
a garage, square, not at an angle.

The existing pipeline (SEARCH -> ALIGN -> APPROACH -> STOP in `rm_cam_beacon.py`)
stops at "point at the beacon and drive to 0.70 m": it uses ONLY the beacon's
position (x, y, z), never its ORIENTATION. It therefore converges to a circle of
radius 0.70 m around the beacon, not to a point. This module adds the missing
stage: the off-axis angle.


=====================================================================
ARCHITECTURE -- why a separate node rather than one more state
=====================================================================
Mini-ADR. *Context*: docking needs to drive the chassis, but `rm_cam_beacon.py`
holds the SINGLE SDK connection to the robot (its own header documents that two
simultaneous connections break `drive_speed`). *Options*: (a) add a DOCK state
inside `rm_cam_beacon.py` -- direct SDK access, hence access to lateral motion
(the Mecanum chassis is holonomic), but it modifies a file that is tested and
validated on hardware; (b) a separate node commanding through the ROS topics
already exposed (`/carolus/cmd_vel`, `/carolus/gimbal_vel`), touching nothing
that exists. *Choice*: (b) -- no regression is possible on the validated chain
(ALIGN/APPROACH, both hardware-tested), and docking can be launched, tested and
abandoned without redeploying `rm_cam_beacon.py`.
*Accepted consequence*: `/carolus/cmd_vel` is only wired to vx and wz in MANUAL
mode (`drive_speed(x=vx, y=0.0, z=wz)`) -- the holonomic chassis's lateral
translation is NOT reachable through that channel. Docking is therefore treated
as a NON-HOLONOMIC problem (turn / drive / turn) rather than as a direct lateral
correction. Condition that would revise this: if `rm_cam_beacon.py` ever relays
`msg.linear.y` to `drive_speed(y=...)`, the three-segment manoeuvre could be
replaced by a simple lateral offset.


=====================================================================
CONTROL STRATEGY -- iterative "look-and-move", not a continuous servo
=====================================================================
Mini-ADR. *Context*: when this was written, Carolus published `/pose` at ~2.5 Hz
(the bottleneck being network transport of the images). *Options*: (a) a
continuous control law in polar coordinates (the classic rho/alpha/beta parking
controller for a unicycle robot); (b) a "measure at rest -> plan -> execute
blind -> re-measure" loop, repeated until tolerance. *Choice*: (b). A continuous
law closed at 2.5 Hz on an angle whose sign is not confirmed (see the SIGN
CONVENTIONS section) oscillates instead of converging; the iterative loop makes
each segment verifiable, bounds the error by re-measuring, and degrades cleanly
(if one iteration makes things worse, you SEE it on the next one and stop). It is
the same "look-and-move" paradigm already chosen for ALIGN.
*Revision condition, now partly met*: moving Carolus onto the Pi (roadmap F0.C)
raising `/pose` above 10 Hz. That happened on 2026-08-04 -- `/pose` measured
13.04 Hz on the Pi against 2.19 Hz on the lab PC -- so the continuous-law option
deserves re-examination. It is NOT reopened here, because the sign conventions
below remain the dominant risk and a continuous law is precisely what they
punish.


=====================================================================
SIGN CONVENTIONS -- the real risk in this module
=====================================================================
This project has a history of unconfirmed signs (GIM_YAW_SIGN was only confirmed
by test on 2026-06-26; GIM_PITCH_SIGN is still unconfirmed after the BUG-058
incident; "EP sign unconfirmed" is noted for `/odom`). Docking depends on
SEVERAL of those signs, so:

  * Chassis rotation sign (`cmd_vel.angular.z`) -> MEASURED AT STARTUP by
    `_probe_turn_sign()` (a small rotation, watching which way `/odom` yaw
    moves). No constant to guess.
  * Gimbal rotation sign (`gimbal_vel.angular.z`) -> MEASURED the same way by
    `_probe_gimbal_sign()` on `/carolus/gimbal_yaw_rel`.
  * Beacon orientation (`p.yaw`) -> CANNOT be self-calibrated: it requires
    knowing where the beacon physically points. Hence the CALIBRATE mode (see
    below), to be run ONCE before the first real docking.

WARNING: until BEACON_YAW_SIGN / BEACON_YAW_OFFSET_DEG have been established by
CALIBRATE mode, this module refuses to dock (the `_yaw_convention_ok` guard) and
falls back to simple distance-holding, behaviour equivalent to the existing
APPROACH. That is deliberate: a wrong sign would turn the robot the WRONG WAY
around the beacon, moving away from the solution.


=====================================================================
USAGE
=====================================================================
Prerequisites: `rm_cam_beacon.py` running (it provides `/odom` and
`/carolus/gimbal_yaw_rel`, and consumes `/carolus/cmd_vel`), Carolus running (it
provides `/pose`), beacon visible.

    python3 beacon_docking.py

**GUI-integrated (2026-07-27)**: launched from `carolus_launcher.py` (terminal
T5), "DOCKING BALISE" panel with CALIBRATE / CAL STEP 2 / START / ABORT buttons
and a live status readout -- see `shortcuts/README.md`.

Or manually, from another terminal:

    # calibrate the beacon orientation convention (once, in two independent
    # clicks -- no timer, each step waits for the next explicit command, at
    # your own pace):
    #   1. place the robot FACING the beacon (~1 m), then:
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'CALIBRATE'"
    #   2. move the robot ~30 deg to the right of the beacon, then:
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'CALSTEP2'"

    # full docking (phases 1+2+3 chained)
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'START'"

    # isolated test (2026-07-28): chassis alignment ONLY, never advances
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'ALIGN_ONLY'"

    # isolated test (2026-07-28): drive ONLY. Verifies alignment before moving
    # and refuses (NOT_ALIGNED) if the chassis is not already aligned -- run
    # ALIGN_ONLY first if needed
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'APPROACH_ONLY'"

    # emergency stop
    rostopic pub -1 /carolus/dock std_msgs/String "data: 'ABORT'"

State is published continuously on `/carolus/dock_status` (String) and logged as
`[DOCKSTATUS] status=... yaw_validated=...` (parsable by `carolus_launcher.py`,
same mechanism as `[BEACON]`), alongside the usual `[DOCK] ...` lines.

This file modifies NO existing file and opens NO SDK connection.
"""

import math
import threading
import time

# Tolerant ROS imports: this module's geometry (`plan_maneuver`) is a pure
# function, testable on a machine with no ROS installed or sourced
# (`python3 beacon_docking.py --selftest`). So we only fail when the node is
# actually started, not at import time.
try:
    import rospy
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Float32, String
    _ROS_AVAILABLE = True
except ImportError as _e:          # pragma: no cover — depend de l'environnement
    _ROS_AVAILABLE = False
    _ROS_IMPORT_ERROR = _e


# =========================================================
# CONFIG
# =========================================================

# ── Cible de docking ─────────────────────────────────────────────────────────
DOCK_DISTANCE_M = 0.20    # distance finale robot<->balise (2026-07-27, demande
                          # explicite utilisateur -- avant : 0.70, aligne sur
                          # STOP_DISTANCE_M de rm_cam_beacon.py)

# Simplified mode (2026-07-27, explicit user request for the first hardware
# test): turn to face the beacon (bearing -> 0), THEN drive straight to
# DOCK_DISTANCE_M. No alignment on the beacon's frontal axis (off-axis is
# ignored), no iterative loop -- one measurement, one turn, one drive. The full
# manoeuvre (plan_maneuver, line-up point, iterations to convergence) remains in
# the code below, disabled by this flag -- set it back to False to re-enable.
SIMPLE_APPROACH_ONLY = True

# -- Final tolerances (docking stops when all three are satisfied) -----------
TOL_RANGE_M      = 0.06   # accepted |range - DOCK_DISTANCE_M| (m)
TOL_OFFAXIS_DEG  = 8.0    # accepted |off-axis angle| (deg) -- "square on"
TOL_BEARING_DEG  = 6.0    # accepted |beacon not centred ahead of the robot| (deg)

# -- Iterative loop ----------------------------------------------------------
MAX_ITERATIONS   = 5      # beyond this we consider it is not converging
MIN_PROGRESS_DEG = 2.0    # if an iteration does not gain at least this on the
                          # off-axis angle, stop (avoids circling forever if a
                          # sign is wrong despite calibration)

# -- Measurement (at rest, robot stationary) ---------------------------------
MEAS_SAMPLES     = 7      # poses aggregated per measurement (median)
MEAS_TIMEOUT_S   = 8.0    # give up if MEAS_SAMPLES are not collected in time
MEAS_MAX_SPREAD_DEG = 25.0  # max tolerated spread on the off-axis angle;
                            # beyond it the measurement is deemed unreliable

# -- Speeds (deliberately low: this is a precision manoeuvre) ----------------
TURN_WZ_DEG_S    = 25.0   # chassis rotation speed during the manoeuvre
DRIVE_VX_M_S     = 0.12   # forward speed during the manoeuvre
CMD_RATE_HZ      = 10.0   # cmd_vel re-publication rate (the receiver cuts out
                          # after 0.5 s, see MANUAL_CMDVEL_TIMEOUT in
                          # rm_cam_beacon.py)

# -- Motion primitives -------------------------------------------------------
TURN_TOL_DEG     = 2.0    # stopping precision of a rotation
TURN_TIMEOUT_MAX_S = 15.0 # safety: never turn for longer than this
SEQUENCE_TIMEOUT_S = 45.0 # 2026-07-28: overall budget for phases 1+2+3 of the
                          # simple pipeline, cumulative (each phase already has
                          # its own timeout; this is an extra safety net, not a
                          # replacement) -- a starting value to be calibrated,
                          # not derived from any existing measurement
DRIVE_TOL_M      = 0.03   # stopping precision of a drive
DRIVE_TIMEOUT_MAX_S = 20.0
MAX_SEGMENT_M    = 2.5    # longest drive executed in one go. A longer plan is
                          # NOT an error (a very off-axis docking at long range
                          # genuinely needs a detour): we truncate and let the
                          # next iteration re-plan from a fresh measurement --
                          # safer than driving 3 m blind on a 2.5 Hz measurement.
ABSURD_SEGMENT_M = 10.0   # beyond this the measurement must be wrong -> abort

# -- Sign probing (self-calibration at the start of the manoeuvre) -----------
PROBE_TURN_DEG_S = 20.0   # speed of the probing rotation
PROBE_DURATION_S = 0.8    # duration of the probing rotation
PROBE_MIN_DELTA_DEG = 1.5 # below this we consider nothing moved

# ── Gimbal ───────────────────────────────────────────────────────────────────
# Measurement is taken with the gimbal aligned on the chassis (yaw_rel ~ 0):
# that makes the camera frame == the chassis frame, so all the geometry below
# does without the sign of yaw_rel, which is unconfirmed. This is the central
# simplification of this module.
GIMBAL_NULL_TOL_DEG  = 3.0
GIMBAL_NULL_SPEED    = 25.0
GIMBAL_NULL_TIMEOUT_S = 12.0
# 2026-07-30: phase 1 used to declare the gimbal aligned on a SINGLE reading
# under tolerance, while phase 2 already required 3 consecutive readings
# (ALIGN_CONSECUTIVE_OK). That inverted the real priority: phase 1's output is
# THE reference for everything downstream, so it is the one most deserving of
# confirmation. With P4P noise on the order of 1 deg against a 3 deg tolerance,
# an isolated reading can fall under tolerance on noise alone. The readings
# counted are DISTINCT poses (different stamps): at a 10 Hz loop against 2.5 Hz
# of vision, the same message would otherwise have been counted three times.
GIMBAL_CONFIRM_OK    = 3
# Time without a fresh pose after which phase 1 concludes "beacon not in view"
# rather than "did not converge" -- two distinct causes that until then produced
# the same timeout message. In the 2026-07-29 cascade, 8 runs out of 11 died in
# phase 1 with logs that could not say which of the two had happened.
GIMBAL_NO_POSE_S     = 4.0

# -- Terminal alignment verification (2026-07-30) ----------------------------
# Phase 2 servos `yaw_rel` -> 0, which means "chassis aligned on the GIMBAL".
# That the chassis is aligned on the BEACON follows ONLY if the gimbal is still
# pointing at the beacon at that instant -- which was never verified. Hardware
# proof (2026-07-29, run 1): SUCCESS reported with yaw_rel=+0.6 deg while the
# image-frame error was -26.3 deg at the same moment.
#
# The fix is NOT to gate on the image error again (tried on the morning of
# 2026-07-29 and rightly removed that afternoon: the value was read DURING the
# rotation, so it was not meaningful at decision time). It is to MEASURE
# properly -- robot stopped, settled, aggregated through _measure() as
# everywhere else in this module -- and then to CORRECT the residual with an
# extra pass instead of merely reporting it.
ALIGN_SETTLE_S       = 1.2   # full stop plus vision refresh before measuring
                             # (at the 2.5 Hz /pose of the time, ~3 frames)
ALIGN_VERIFY_PASSES  = 3     # max chained passes (gimbal + chassis)
ALIGN_VERIFY_MIN_GAIN_DEG = 2.0   # minimum gain required from one pass to the
                                  # next, otherwise stop (a further pass would
                                  # only wear the mechanics)
# Max tolerated drift of the gimbal's ABSOLUTE heading between the end of phase
# 1 and the end of phase 2. yaw_ground is a reliable and free witness, confirmed
# on 2026-07-29: it stayed stable to +0.2 deg while the chassis rotated 104.8
# deg. If the reference moved, the conclusion "chassis aligned on the beacon"
# no longer holds, whatever yaw_rel says.
ALIGN_REF_DRIFT_MAX_DEG = 8.0

# -- Beacon orientation convention (TO BE ESTABLISHED BY CALIBRATE MODE) -----
# psi = BEACON_YAW_SIGN * p.yaw + BEACON_YAW_OFFSET_DEG
# psi is defined as the OFF-AXIS ANGLE: 0 means the robot sits exactly on the
# beacon's frontal axis (it sees the beacon square on), non-zero means it sees
# it at an angle. Values established by CALIBRATE on 2026-07-27 (reference
# yaw_face, delta measured at +57.2 deg after moving roughly 45 deg right).
BEACON_YAW_SIGN       = +1.0
BEACON_YAW_OFFSET_DEG = +2.4
# Set to True ONLY after validating both values above through CALIBRATE mode on
# the hardware. False means docking is refused (falling back to simple
# distance-holding), see the header.
BEACON_YAW_VALIDATED  = True

POSE_TIMEOUT_S = 1.5      # same value as rm_cam_beacon.py


# =========================================================
# Helpers
# =========================================================

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def angle_diff_deg(a, b):
    """Smallest signed angular difference a-b, in degrees, within [-180, 180].
    (Same helper as rm_cam_beacon.py's `_angle_diff_deg` -- duplicated
    deliberately: this module must import no existing file so it stays
    deployable on its own.)"""
    return ((a - b + 180.0) % 360.0) - 180.0


def median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        return 0.0
    mid = n // 2
    if n % 2 == 1:
        return s[mid]
    return 0.5 * (s[mid - 1] + s[mid])


# 2026-07-28: `_align_chassis_yaw_rel`'s decision logic extracted into pure
# functions (same principle as `plan_maneuver`) so it can be exercised through
# --selftest, with neither ROS nor a robot.

def chassis_align_tick(yaw_rel, deadband_deg, consecutive_ok):
    """New count of consecutive in-tolerance readings (0 when out of tolerance,
    which resets the counter)."""
    if abs(yaw_rel) < deadband_deg:
        return consecutive_ok + 1
    return 0


def chassis_no_progress(err_before, err_now, min_gain_deg):
    """True if the absolute error has not decreased enough between two checks
    spaced in time (divergence or stagnation)."""
    return (err_before - err_now) < min_gain_deg


def chassis_is_blocked(yaw_rel_ref, yaw_rel_now, min_delta_deg, commands_sent):
    """True if non-zero commands have already been sent but yaw_rel has barely
    moved since the last reference -- the chassis is physically stalled
    (mechanical stop, jammed wheel, etc.).

    2026-07-30: the comparison now goes through `angle_diff_deg`. Before, two
    already-normalised angles were subtracted directly, which breaks across
    the +/-180 deg wrap: a STRICTLY stationary chassis at ref=+179 / now=-179
    gave |diff|=358 deg and therefore read as "moving" -- exactly the case
    where a stall had to be detected."""
    return commands_sent > 0 and \
        abs(angle_diff_deg(yaw_rel_now, yaw_rel_ref)) < min_delta_deg


def gimbal_confirm_tick(err_deg, tol_deg, consecutive_ok):
    """Same discipline as `chassis_align_tick`, applied to phase 1's image
    error: a count of consecutive in-tolerance readings, reset to zero as soon
    as one reading falls outside tolerance (2026-07-30)."""
    if abs(err_deg) < tol_deg:
        return consecutive_ok + 1
    return 0


def align_verify_verdict(residual_deg, tol_deg, prev_residual_deg,
                         min_gain_deg, passes_done, max_passes):
    """Decide what to do after a verified alignment pass (2026-07-30).

    A PURE function (no I/O, no state), so it is covered by --selftest, like
    `plan_maneuver` and the three `chassis_*` helpers.

    `residual_deg` is the beacon bearing measured in the CHASSIS frame with the
    robot stopped. Returns one of:
      * "ok"        -- residual within tolerance: alignment genuinely reached
      * "retry"     -- outside tolerance, budget left and progress sufficient:
                       one more pass is worth it
      * "no_gain"   -- the previous pass did not gain min_gain_deg: insisting
                       would only wear the mechanics
      * "exhausted" -- budget de passes epuise
    """
    if abs(residual_deg) <= tol_deg:
        return "ok"
    if passes_done >= max_passes:
        return "exhausted"
    if prev_residual_deg is not None and \
            (abs(prev_residual_deg) - abs(residual_deg)) < min_gain_deg:
        return "no_gain"
    return "retry"


class DockAbort(Exception):
    """Raised as soon as an ABORT is requested or a safety trips.
    Propagates up to `_dock_sequence`, which stops the robot cleanly."""


# =========================================================
# Nœud
# =========================================================

class BeaconDocking:

    def __init__(self):
        # --- etat mesure ---
        self._pose = None          # (x, y, z, yaw_deg, stamp) brut camera
        self._pose_lock = threading.Lock()
        self._yaw_rel = 0.0
        self._yaw_rel_lock = threading.Lock()
        # 2026-07-29 (BUG-080, instrumentation): /carolus/gimbal_yaw_ground has
        # existed and been published by rm_cam_beacon.py since 2026-07-27, but was
        # never listened to here. It is now subscribed purely to LOG yaw_ground and
        # (yaw_ground - yaw_rel) at every step of _align_chassis_yaw_rel -- not to
        # change behaviour. The point: that the NEXT real hardware run, whatever it
        # is and without needing a dedicated protocol, automatically captures the
        # evidence confirming or refuting H1.
        self._yaw_ground = 0.0
        self._yaw_ground_lock = threading.Lock()
        self._odom = None          # (x, y, yaw_deg)
        self._odom_lock = threading.Lock()

        # --- etat manœuvre ---
        self._abort = False
        # Robot mode, as reported by rm_cam_beacon.py on a latched topic.
        # 'unknown' until it arrives, and 'unknown' is permissive: refusing to
        # dock because a status topic is missing would be worse than the bug
        # this guards against.
        self._robot_mode = "unknown"
        self._busy = False
        self._turn_sign = None     # +1/-1, determine par _probe_turn_sign()
        self._gimbal_sign = None   # +1/-1, determined by _probe_gimbal_sign()
        # The gimbal's ABSOLUTE heading (yaw_ground) at the moment phase 1
        # declared it aligned on the beacon. Serves as a drift witness for that
        # reference during phase 2 (2026-07-30, see ALIGN_REF_DRIFT_MAX_DEG).
        # None until a phase 1 has succeeded.
        self._gimbal_ref_ground = None
        self._cal_yaw_face = None  # result of CALIBRATE step 1/2, awaiting step 2
        self._status = "IDLE"

        # --- republished-pose detection (2026-07-28) ---
        # carolus_astrobee.cpp::getFilteredPose() (lines 560-598) republishes the
        # OLD pose (bit-identical raw values) with a fresh header.stamp whenever a
        # new detection is judged too different and rejected (lines 569-576,
        # 583-590). Consequence: _fresh_pose(), which is based on ROS reception
        # time, CANNOT distinguish a genuinely new detection from a republication.
        # The only usable signal without touching the .cpp is to compare the raw
        # values (x, y, z, yaw) between two successive receptions -- a
        # republication makes them bit-for-bit identical, which an independent P4P
        # solve essentially never produces (solver numerical noise). Known limit:
        # this is a heuristic, not a guarantee -- also documented in the header.
        self._last_raw_pose_values = None   # (x, y, z, yaw_deg) of the last reception
        self._pose_repeat_count = 0         # consecutive identical receptions

        # --- ROS ---
        self.pub_cmd = rospy.Publisher("/carolus/cmd_vel", Twist, queue_size=1)
        self.pub_gim = rospy.Publisher("/carolus/gimbal_vel", Twist, queue_size=1)
        self.pub_mode = rospy.Publisher("/carolus/mode", String, queue_size=1)
        self.pub_lock = rospy.Publisher("/carolus/gimbal_lock", String, queue_size=1)
        self.pub_status = rospy.Publisher("/carolus/dock_status", String, queue_size=1)

        rospy.Subscriber("/pose", PoseStamped, self._pose_cb)
        rospy.Subscriber("/carolus/gimbal_yaw_rel", Float32, self._yaw_rel_cb)
        rospy.Subscriber("/carolus/gimbal_yaw_ground", Float32, self._yaw_ground_cb)
        rospy.Subscriber("/odom", Odometry, self._odom_cb)
        rospy.Subscriber("/carolus/dock", String, self._cmd_cb)
        rospy.Subscriber("/carolus/robot_mode", String, self._robot_mode_cb)

        rospy.Timer(rospy.Duration(0.5), self._status_tick)

        rospy.loginfo("[DOCK] pret — commandes sur /carolus/dock : START / CALIBRATE / CALSTEP2 / ABORT")
        if not BEACON_YAW_VALIDATED:
            rospy.logwarn("[DOCK] BEACON_YAW_VALIDATED=False -- full docking is "
                          "DESACTIVE tant que la convention d'orientation balise n'a pas "
                          "ete etablie (mode CALIBRATE). START fera un simple maintien "
                          "de distance.")

    # ---------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------

    def _pose_cb(self, msg):
        p = msg.pose.position
        q = msg.pose.orientation
        if not all(map(math.isfinite, [p.x, p.y, p.z, q.x, q.y, q.z, q.w])):
            return
        # Beacon yaw extraction -- the SAME formula as rm_cam_beacon.py's
        # `_pose_cb` (rotation about the camera's y axis). Deliberately identical:
        # this is the value already logged as `byaw` for months, so the only one
        # with field history. Its limitation is known: exact for a pure rotation
        # about y, approximate as soon as the beacon is tilted or the gimbal
        # pitched.
        siny = 2.0 * (q.w * q.y + q.z * q.x)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny, cosy))
        raw = (p.x, p.y, p.z, yaw_deg)
        with self._pose_lock:
            self._pose = (p.x, p.y, p.z, yaw_deg, time.time())
            if raw == self._last_raw_pose_values:
                self._pose_repeat_count += 1
            else:
                self._pose_repeat_count = 0
            self._last_raw_pose_values = raw

    def _get_pose_repeat_count(self):
        with self._pose_lock:
            return self._pose_repeat_count

    def _yaw_rel_cb(self, msg):
        with self._yaw_rel_lock:
            self._yaw_rel = float(msg.data)

    def _yaw_ground_cb(self, msg):
        with self._yaw_ground_lock:
            self._yaw_ground = float(msg.data)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.degrees(math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        with self._odom_lock:
            self._odom = (p.x, p.y, yaw)

    def _robot_mode_cb(self, msg):
        mode = msg.data.strip().lower()
        if mode != self._robot_mode:
            rospy.loginfo("[DOCK] robot mode: %r", mode)
            if mode not in ("free", "unknown"):
                rospy.logwarn("[DOCK] chassis alignment is UNAVAILABLE in %r "
                              "(gimbal follows chassis -> yaw_rel cannot change). "
                              "Relaunch the camera node with RM_ROBOT_MODE=free.", mode)
        self._robot_mode = mode

    def _cmd_cb(self, msg):
        cmd = msg.data.strip().upper()
        if cmd == "ABORT":
            self._abort = True
            rospy.logwarn("[DOCK] ABORT demande")
            return
        if self._busy:
            rospy.logwarn(f"[DOCK] {cmd} ignore : manœuvre deja en cours")
            return
        # 2026-08-14 (BUG-111): refuse chassis alignment outright in chassis_lead.
        #
        # Chassis alignment servos on `yaw_rel`, the gimbal-to-chassis angle,
        # rotating the chassis until it reaches zero. In chassis_lead the gimbal
        # FOLLOWS the chassis, so that angle is constant by construction and
        # rotating the chassis cannot change it -- the loop has no observable and
        # can never converge. The no-progress guard turns that into an abort
        # rather than an endless spin, but the abort looks like a hardware fault
        # and would send the next person hunting the wrong bug. Fail loudly and
        # name the remedy instead. CALIBRATE/CALSTEP2 are exempt: they measure the
        # sign convention and do not run the alignment loop.
        if cmd in ("START", "ALIGN_ONLY") and self._robot_mode not in ("free", "unknown"):
            rospy.logerr("[DOCK] %s REFUSED: robot is in %r, which makes chassis "
                         "alignment impossible -- in that mode the gimbal follows "
                         "the chassis, so yaw_rel cannot change and the alignment "
                         "never converges. Relaunch the camera node with "
                         "RM_ROBOT_MODE=free, then retry.", cmd, self._robot_mode)
            return

        if cmd == "START":
            threading.Thread(target=self._run, args=(self._dock_sequence,),
                             daemon=True).start()
        elif cmd == "ALIGN_ONLY":
            threading.Thread(target=self._run, args=(self._align_only,),
                             daemon=True).start()
        elif cmd == "APPROACH_ONLY":
            threading.Thread(target=self._run, args=(self._approach_only,),
                             daemon=True).start()
        elif cmd == "CALIBRATE":
            threading.Thread(target=self._run, args=(self._calibrate_step1,),
                             daemon=True).start()
        elif cmd == "CALSTEP2":
            threading.Thread(target=self._run, args=(self._calibrate_step2,),
                             daemon=True).start()

    def _status_tick(self, _event):
        self.pub_status.publish(String(data=self._status))
        # Logged line (in addition to the /carolus/dock_status topic) so that
        # carolus_launcher.py can parse it exactly like the [BEACON]/[BEACONPOS]
        # lines rm_cam_beacon.py already publishes -- same mechanism, no new
        # pattern.
        rospy.loginfo_throttle(
            1.0, f"[DOCKSTATUS] status={self._status} yaw_validated={BEACON_YAW_VALIDATED}")

    # ---------------------------------------------------------
    # Accesseurs
    # ---------------------------------------------------------

    def _get_pose(self):
        with self._pose_lock:
            return self._pose

    def _fresh_pose(self):
        p = self._get_pose()
        return p is not None and (time.time() - p[4]) < POSE_TIMEOUT_S

    def _get_yaw_rel(self):
        with self._yaw_rel_lock:
            return self._yaw_rel

    def _get_yaw_ground(self):
        with self._yaw_ground_lock:
            return self._yaw_ground

    def _get_odom(self):
        with self._odom_lock:
            return self._odom

    def _check_abort(self):
        if self._abort or rospy.is_shutdown():
            raise DockAbort()

    # ---------------------------------------------------------
    # Commandes bas niveau
    # ---------------------------------------------------------

    def _send_cmd(self, vx=0.0, wz=0.0):
        t = Twist()
        t.linear.x = vx
        t.angular.z = wz
        self.pub_cmd.publish(t)

    def _send_gimbal(self, yaw_speed=0.0):
        t = Twist()
        t.angular.z = yaw_speed
        self.pub_gim.publish(t)

    def _stop(self):
        for _ in range(3):
            self._send_cmd(0.0, 0.0)
            self._send_gimbal(0.0)
            time.sleep(0.05)

    def _take_control(self):
        """Put the robot into MANUAL (the only mode where `/carolus/cmd_vel` and
        `/carolus/gimbal_vel` are relayed to the SDK) and switch the beacon LOCK
        off (otherwise the LOCK tick ignores our gimbal commands -- see the
        three-priority arbitration in rm_cam_beacon.py's MANUAL loop)."""
        self.pub_mode.publish(String(data="MANUAL"))
        self.pub_lock.publish(String(data="OFF"))
        time.sleep(0.4)   # let the mode take effect before commanding

    # ---------------------------------------------------------
    # Self-calibration of the rotation signs
    # ---------------------------------------------------------

    def _probe_turn_sign(self):
        """Determine which sign of `cmd_vel.angular.z` INCREASES the `/odom` yaw.
        Removes any need to know the EP convention in advance -- it is
        unconfirmed on this project, see the header."""
        if self._turn_sign is not None:
            return
        od = self._get_odom()
        if od is None:
            raise DockAbort()
        yaw0 = od[2]
        t0 = time.time()
        while time.time() - t0 < PROBE_DURATION_S:
            self._check_abort()
            self._send_cmd(0.0, PROBE_TURN_DEG_S)
            time.sleep(1.0 / CMD_RATE_HZ)
        self._stop()
        time.sleep(0.4)   # let the chassis settle before reading again
        od = self._get_odom()
        if od is None:
            raise DockAbort()
        delta = angle_diff_deg(od[2], yaw0)
        if abs(delta) < PROBE_MIN_DELTA_DEG:
            rospy.logerr(f"[DOCK] sondage rotation : le chassis n'a pas bouge "
                         f"({delta:.1f} deg) — robot bloque, mode non MANUEL, ou "
                         f"/odom absent")
            raise DockAbort()
        self._turn_sign = 1.0 if delta > 0 else -1.0
        rospy.loginfo(f"[DOCK] sondage rotation : wz>0 -> yaw {delta:+.1f} deg "
                      f"-> turn_sign={self._turn_sign:+.0f}")

    def _probe_gimbal_sign(self):
        """Same for the gimbal, on `/carolus/gimbal_yaw_rel`. Retries once, in
        the opposite direction and for twice as long, if the first probe does not
        move (2026-07-27, observed on hardware: the gimbal can sit near a
        mechanical stop in the first direction tried, especially after several
        CALIBRATE/docking attempts in one session)."""
        if self._gimbal_sign is not None:
            return
        attempts = [(PROBE_TURN_DEG_S, PROBE_DURATION_S),
                    (-PROBE_TURN_DEG_S, PROBE_DURATION_S * 2)]
        for n, (speed, dur) in enumerate(attempts, start=1):
            y0 = self._get_yaw_rel()
            t0 = time.time()
            while time.time() - t0 < dur:
                self._check_abort()
                self._send_gimbal(speed)
                time.sleep(1.0 / CMD_RATE_HZ)
            self._send_gimbal(0.0)
            time.sleep(0.4)
            delta = angle_diff_deg(self._get_yaw_rel(), y0)
            if abs(delta) >= PROBE_MIN_DELTA_DEG:
                self._gimbal_sign = 1.0 if (delta > 0) == (speed > 0) else -1.0
                rospy.loginfo(f"[DOCK] gimbal probe (attempt {n}/2, cmd={speed:+.0f}): "
                              f"yaw_rel {delta:+.1f} deg -> gimbal_sign={self._gimbal_sign:+.0f}")
                return
            rospy.logwarn(f"[DOCK] gimbal probe (attempt {n}/2, cmd={speed:+.0f}): "
                          f"gimbal did not move ({delta:.1f} deg)")
        rospy.logwarn("[DOCK] gimbal probe: no movement in either direction after 2 "
                      "attempts -- gimbal alignment UNAVAILABLE this run (0 = unusable)")
        self._gimbal_sign = 0.0   # 0 = gimbal inutilisable, on s'en passe

    # ---------------------------------------------------------
    # Primitives de mouvement (asservies sur /odom)
    # ---------------------------------------------------------

    def _null_gimbal(self):
        """Bring the gimbal back onto the chassis axis (yaw_rel -> 0) so that the
        camera frame coincides with the chassis frame during measurement.

        Note: `gimbal.recenter()` (available through `/carolus/gimbal_recenter`)
        is deliberately NOT used -- recenter returns the gimbal to its POWER-ON
        frame, which has no reason to be aligned with the current chassis (it is
        explicitly documented as "independent of chassis orientation" in
        rm_cam_beacon.py). Here we want yaw_rel = 0, so we servo on yaw_rel."""
        if self._gimbal_sign == 0.0:
            return
        self._probe_gimbal_sign()
        if self._gimbal_sign == 0.0:
            return
        t0 = time.time()
        while time.time() - t0 < GIMBAL_NULL_TIMEOUT_S:
            self._check_abort()
            err = self._get_yaw_rel()          # cible = 0
            if abs(err) < GIMBAL_NULL_TOL_DEG:
                break
            # On veut faire DECROITRE yaw_rel : commande de signe oppose a
            # the error, corrected by the sign measured during probing.
            cmd = -self._gimbal_sign * clamp(err, -GIMBAL_NULL_SPEED, GIMBAL_NULL_SPEED)
            cmd = clamp(cmd, -GIMBAL_NULL_SPEED, GIMBAL_NULL_SPEED)
            self._send_gimbal(cmd)
            time.sleep(1.0 / CMD_RATE_HZ)
        self._send_gimbal(0.0)
        rospy.loginfo(f"[DOCK] gimbal aligne chassis : yaw_rel={self._get_yaw_rel():.1f} deg")

    def _turn_by(self, delta_deg):
        """Rotate the chassis by delta_deg (sign follows this module's geometric
        convention: positive = toward the camera's RIGHT, see `_measure`).
        Servoed on the `/odom` yaw, therefore insensitive to the SDK's own sign
        convention, which `_probe_turn_sign` has measured."""
        if abs(delta_deg) < TURN_TOL_DEG:
            return
        self._probe_turn_sign()
        od = self._get_odom()
        if od is None:
            raise DockAbort()
        yaw0 = od[2]
        rospy.loginfo(f"[DOCK] rotation: target={delta_deg:+.1f} deg (camera frame, "
                      f"+ = right) turn_sign={self._turn_sign:+.0f} "
                      f"raw_start_odom_yaw={yaw0:+.1f} deg")
        # Theoretical duration with a 3x margin, capped: a guard if /odom freezes.
        timeout = min(TURN_TIMEOUT_MAX_S,
                      max(3.0, 3.0 * abs(delta_deg) / max(TURN_WZ_DEG_S, 1.0)))
        # `delta_deg` is expressed in this module's geometric frame; the command
        # sign to apply is the one measured by the probe. The probe established the
        # link between wz>0 and the direction /odom yaw moves in; here we assume
        # /odom yaw grows in the same sense as our geometric angle. If a hardware
        # test shows the opposite, THIS sign (and only this one) is what to
        # invert.
        target = delta_deg
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._check_abort()
            od = self._get_odom()
            if od is None:
                break
            done = angle_diff_deg(od[2], yaw0) * self._turn_sign
            remaining = target - done
            if abs(remaining) < TURN_TOL_DEG:
                break
            wz = self._turn_sign * clamp(remaining, -TURN_WZ_DEG_S, TURN_WZ_DEG_S)
            # floor: below ~5 deg/s the chassis does not reliably start moving
            if 0.0 < abs(wz) < 5.0:
                wz = math.copysign(5.0, wz)
            self._send_cmd(0.0, wz)
            time.sleep(1.0 / CMD_RATE_HZ)
        self._stop()
        time.sleep(0.3)
        od = self._get_odom()
        if od is not None:
            raw_delta = angle_diff_deg(od[2], yaw0)
            got = raw_delta * self._turn_sign
            rospy.loginfo(f"[DOCK] rotation demandee={delta_deg:+.1f} deg obtenue={got:+.1f} deg "
                          f"(yaw_odom_brut : {yaw0:+.1f} -> {od[2]:+.1f} deg, delta_brut="
                          f"{raw_delta:+.1f} deg, turn_sign={self._turn_sign:+.0f})")

    def _track_beacon_gimbal_tick(self):
        """One gimbal correction to keep the beacon centred in the image
        DURING the drive (2026-07-27, user request -- same goal as LOCK in
        rm_cam_beacon.py, but integrated here: LOCK is switched off for the whole
        of `_dock_sequence` by `_take_control`, since the two systems cannot
        command the gimbal at once without fighting).

        Reuses EXACTLY the validated formula from `_null_gimbal`
        (cmd = -gimbal_sign * clamp(error)) with a different error: `_null_gimbal`
        nulls yaw_rel (camera aligned on the CHASSIS, for the at-rest
        measurement); here we null the beacon's lateral offset IN
        THE IMAGE (p.x/p.z, the same formula as `_measure`/LOCK), so the camera
        FOLLOWS the beacon while the robot drives. A reasoned generalisation of
        the probed sign (same physical sense: both errors decrease when the
        gimbal turns the same way) but NOT independently verified on hardware
        -- watch it on the first test.

        2026-07-29: now returns the computed image error (or None if no usable
        fresh pose), so a caller can verify the gimbal is REALLY on the beacon
        at that instant instead of guessing. Reused by
        `_align_chassis_yaw_rel` (see BUG-080: `yaw_rel` alone may not be
        reliable; `err_img` depends on no assumption about gimbal
        stabilisation -- it is a direct visual reading, immune to the history
        of
        rotation de la plateforme). Changement de signature retrocompatible
        : le seul appelant existant (`_drive_by`) ignorait deja la valeur
        de retour."""
        if self._gimbal_sign is None or self._gimbal_sign == 0.0:
            return None
        p = self._get_pose()
        if p is None or not self._fresh_pose() or abs(p[2]) < 0.05:
            self._send_gimbal(0.0)
            return None
        err = math.degrees(math.atan2(p[0], abs(p[2])))
        if abs(err) < GIMBAL_NULL_TOL_DEG:
            self._send_gimbal(0.0)
            return err
        cmd = -self._gimbal_sign * clamp(err, -GIMBAL_NULL_SPEED, GIMBAL_NULL_SPEED)
        self._send_gimbal(cmd)
        return err

    def _align_gimbal_to_beacon(self, timeout_s=GIMBAL_NULL_TIMEOUT_S):
        """Phase 1/3 of the simple pipeline: point the gimbal at the beacon with
        the chassis stationary. Reuses `_track_beacon_gimbal_tick` (the same servo
        used during the drive).

        Renvoie True si la nacelle est reellement alignee (a servir de
        a reliable reference for phase 2), False otherwise -- 2026-07-27:
        before this fix, a silent failure here (gimbal_sign=0, gimbal not
        moving) still let phase 2 align on an arbitrary reference, the direct
        cause of a collision observed on hardware."""
        if self._gimbal_sign is None:
            self._probe_gimbal_sign()
        if self._gimbal_sign == 0.0:
            rospy.logerr("[DOCK] phase 1 aborted: gimbal unusable "
                         "(probe produced no movement) -- no reliable reference")
            return False
        t0 = time.time()
        consecutive_ok = 0
        last_counted_stamp = None   # count only DISTINCT poses
        t_last_pose = t0
        no_pose_warned = False
        while time.time() - t0 < timeout_s:
            self._check_abort()
            self._track_beacon_gimbal_tick()
            p = self._get_pose()
            if p is not None and self._fresh_pose() and abs(p[2]) > 0.05:
                t_last_pose = time.time()
                # A reading only counts if it comes from a NEW message
                # (2026-07-30): the loop runs at CMD_RATE_HZ=10 Hz against 2.5 Hz
                # of vision, so without this filter the same pose would be counted
                # three times in a row and the "confirmation" would confirm
                # nothing at all.
                if p[4] != last_counted_stamp:
                    last_counted_stamp = p[4]
                    err = math.degrees(math.atan2(p[0], abs(p[2])))
                    consecutive_ok = gimbal_confirm_tick(
                        err, GIMBAL_NULL_TOL_DEG, consecutive_ok)
                    if consecutive_ok >= GIMBAL_CONFIRM_OK:
                        self._send_gimbal(0.0)
                        # yaw_ground : d'abord instrumentation BUG-080
                        # (2026-07-29), desormais AUSSI la reference de derive
                        # relue en fin de phase 2 (2026-07-30).
                        self._gimbal_ref_ground = self._get_yaw_ground()
                        rospy.loginfo(f"[DOCK] gimbal aligned on the beacon "
                                      f"({consecutive_ok}/{GIMBAL_CONFIRM_OK} distinct "
                                      f"poses confirmed, "
                                      f"yaw_rel={self._get_yaw_rel():+.1f} deg, "
                                      f"yaw_ground={self._gimbal_ref_ground:+.1f} deg, "
                                      f"err_img={err:+.1f} deg)")
                        return True
            elif (not no_pose_warned) and (time.time() - t_last_pose) > GIMBAL_NO_POSE_S:
                no_pose_warned = True
                rospy.logwarn(f"[DOCK] phase 1 : aucune pose fraiche depuis "
                              f"{GIMBAL_NO_POSE_S}s -- the beacon appears to be outside the "
                              f"camera field of view, the gimbal cannot servo")
            time.sleep(1.0 / CMD_RATE_HZ)
        self._send_gimbal(0.0)
        # 2026-07-30: two clearly distinct failure causes, until now conflated
        # under the same "timed out without converging" message (the 2026-07-29
        # cascade: 8 runs out of 11 died here with no diagnosis possible from the
        # logs).
        no_pose_for = time.time() - t_last_pose
        if no_pose_for > GIMBAL_NO_POSE_S:
            rospy.logerr(f"[DOCK] phase 1: FAILED -- BEACON NOT IN VIEW "
                         f"(aucune pose fraiche depuis {no_pose_for:.1f}s sur "
                         f"{timeout_s}s of budget). This is not a servo problem: "
                         f"re-orient the robot/gimbal toward the beacon "
                         f"before restarting")
        else:
            rospy.logerr(f"[DOCK] phase 1: FAILED -- beacon seen but gimbal did not "
                         f"converge in {timeout_s}s "
                         f"({consecutive_ok}/{GIMBAL_CONFIRM_OK} confirmations, "
                         f"yaw_rel={self._get_yaw_rel():+.1f} deg, "
                         f"yaw_ground={self._get_yaw_ground():+.1f} deg) — "
                         f"reference non fiable")
        return False

    def _align_chassis_yaw_rel(self, timeout_s=TURN_TIMEOUT_MAX_S):
        """Phase 2/3 of the simple pipeline: rotate the CHASSIS to null yaw_rel
        (the gimbal, already pointed at the beacon in phase 1, is the reference --
        once yaw_rel ~ 0 the chassis is de facto pointed at the beacon).

        N'utilise PAS `_turn_sign` (sonde sur /odom, suspect n°1 du "robot
        part a l'oppose" observe avec `_turn_by`). Reutilise a la place un
        fact ALREADY CONFIRMED on this robot and this same command path
        (/carolus/cmd_vel -> rm_cam_beacon.py -> chassis.drive_speed): the
        existing ALIGN state (2026-06-26, K_BODY_YAW) measured that
        wz = +K*yaw_rel DECREASES yaw_rel (106 deg -> 1.6 deg with wz=+10).
        This module therefore does NOT need to re-probe that sign.

        Freshness note: `yaw_rel` comes from `/carolus/gimbal_yaw_rel`,
        published by rm_cam_beacon.py from `gimbal.sub_angle` (a direct SDK
        encoder reading) -- NOT derived from `/pose`. The republished-pose
        risk from carolus_astrobee.cpp (getFilteredPose) therefore does not
        apply here; it is handled in `_measure()`/`_pose_cb` instead, where
        `/pose`
        est reellement consomme.

        2026-07-28 (ajout garde-fous, suite a l'absence totale de verification
        of the false-success behaviour seen in the previous version):
          - returns True/False (no caller can ignore a failure any more)
          - requires ALIGN_CONSECUTIVE_OK consecutive in-tolerance readings
            before declaring success (a single reading could be noise)
          - detects lack of progress / divergence (compares the absolute error
            at every command sent)
          - detects a physically stalled chassis (yaw_rel does not move at all
            despite non-zero commands sent for a
            raisonnable)
          - log structure par commande : erreur avant, commande envoyee,
            erreur apres, progres
          - arret + vitesse nulle garantis sur TOUTE sortie (succes, echec,
            timeout, blocage, divergence)

        2026-07-29 (confirmation visuelle continue, suite a BUG-080) :
        `yaw_rel` is defined by the SDK as "gimbal angle relative to the
        chassis" -- a chassis-relative reading, NOT an absolute heading. If the
        gimbal is actively stabilised (hypothesis H1, unconfirmed on this
        robot), `yaw_rel` can carry a history of earlier rotations unrelated to
        the current beacon -- observed on hardware at +255.1 deg for a real
        physical offset of ~90 deg (2026-07-28).

        Rather than wait for BUG-080 to be resolved before fixing the cause, a
        first attempt (morning of 2026-07-29) tried to make this function
        robust WHATEVER the answer to H1/H2, by having the gimbal track the
        beacon CONTINUOUSLY during the chassis rotation
        (`_track_beacon_gimbal_tick`). **Revised the same day, after the first
        hardware test**: that active tracking produced an oscillating image
        error (-2.9 -> +13.9 -> -22.9 deg in a single run) and the chassis never
        converged (failed on "no progress") -- a likely sign that active visual
        correction fights a stabilisation already handled by the gimbal
        firmware (H1).

        Previous version (afternoon of 2026-07-29): the gimbal received no
        command at all in this function, but success was only declared if
        `yaw_rel` AND the passively-read image error were BOTH near zero --
        otherwise "inconsistent signals".

        2026-07-29 (deuxieme test materiel, retrait du garde-fou image) :
        that test was the first to see yaw_rel converge on hardware
        (-106.3 -> -1.4 deg, 3/3 readings then beyond) -- AND it produced the
        cleanest evidence to date for H1: yaw_ground stayed strictly stable
        (+166.4 -> +166.6 deg, delta=+0.2 deg) while yaw_rel absorbed
        +104.8 deg of chassis rotation, with the gimbal completely still (no
        gimbal command sent that run). But the image guard failed:
        image_error=+30.5 deg at the moment of convergence, never coming back
        under GIMBAL_NULL_TOL_DEG. The run ended in FAILURE through the global
        timeout rather than through the guard itself (a close race, with
        convergence arriving late in the timeout_s budget).

        Since yaw_ground proves the gimbal did NOT change absolute heading,
        the image error cannot come from a wrong rotation -- it more likely
        comes from a lateral drift of the chassis during the rotation (already
        identified, roadmap item 12), which this function does not command
        anyway (it drives only wz, never vy). Requiring an image confirmation
        therefore meant blocking on a defect no heading correction can fix.
        The "inconsistent signals" guard is removed from the success/failure
        path: the image error is still read and logged (free, useful for
        BUG-080 and for quantifying item 12's drift, with a `logwarn` if the
        error is notable), but it no longer blocks an otherwise clean yaw_rel
        convergence.
        """
        ALIGN_DEADBAND_DEG = 2.0
        ALIGN_GAIN = 0.8
        ALIGN_MAX_WZ = 10.0
        # 2026-07-28 -- the guard parameters below are reasonable starting
        # values (consistent with TURN_TOL_DEG=2.0 deg and the behaviour observed
        # on 2026-07-27, where wz=+10 took yaw_rel from 106 to 1.6 deg in a few
        # seconds), but NONE of them comes from a dedicated measurement campaign
        # on this robot -- calibrate them if the isolated phase-2 test produces a
        # false positive (a stall or divergence reported while the chassis was
        # merely converging slowly) or a false negative (the timeout is reached
        # without a stall or divergence having been detected earlier).
        ALIGN_CONSECUTIVE_OK = 3        # consecutive in-tolerance readings required
        NO_PROGRESS_WINDOW_S = 3.0      # sliding window over which progress is judged
        NO_PROGRESS_MIN_GAIN_DEG = 1.0  # minimum gain expected over that window
        BLOCKED_CHECK_S = 2.0           # time before judging the chassis stalled
        BLOCKED_MIN_DELTA_DEG = 1.0     # min yaw_rel change expected over BLOCKED_CHECK_S

        # 2026-07-29 (BUG-081): raw yaw_rel is NOT wrapped to [-180, 180] --
        # observed on hardware at +255.1 deg. Used as-is, wz=+K*yaw_rel commands a
        # rotation in the WRONG direction and 2.4x longer than necessary (255 deg
        # instead of the short way round, -104.9 deg). Every reading is therefore
        # normalised through angle_diff_deg (already used elsewhere in this file).
        # This does NOT fix the cause of the aberrant value (BUG-080, still open)
        # -- only the controller's reaction to a value outside [-180, 180].
        yaw_rel_0 = angle_diff_deg(self._get_yaw_rel(), 0.0)
        # 2026-07-29 (BUG-080, instrumentation): yaw_ground is logged here PURELY
        # as evidence -- it plays no part in the control logic below. The point is
        # that this run, whether it succeeds or fails, leaves enough in the logs to
        # decide between H1 and H2: if yaw_ground stays stable while yaw_rel moves
        # by the angle the chassis turned, H1 is supported.
        yaw_ground_0 = self._get_yaw_ground()
        rospy.loginfo(f"[DOCK] chassis alignment: target yaw_rel=0, "
                      f"initial error={yaw_rel_0:+.1f} deg "
                      f"(yaw_ground initial={yaw_ground_0:+.1f} deg, "
                      f"yaw_ground-yaw_rel={yaw_ground_0 - yaw_rel_0:+.1f} deg -- "
                      f"BUG-080, preuve H1/H2)")

        consecutive_ok = 0
        t0 = time.time()
        t_last_progress_check = t0
        err_at_last_progress_check = abs(yaw_rel_0)
        t_block_ref = t0
        yaw_rel_block_ref = yaw_rel_0
        commands_sent = 0
        last_img_err = None    # dernier ecart image connu (suivi continu, 2026-07-29)

        def _exit(success, reason, final_err):
            self._stop()
            time.sleep(0.3)
            final_check = self._get_yaw_rel()
            final_ground = self._get_yaw_ground()  # BUG-080 instrumentation, 2026-07-29
            rospy.loginfo(f"[DOCK] chassis alignment done: "
                          f"{'SUCCESS' if success else 'FAILED'} ({reason}) -- "
                          f"final error={final_err:+.1f} deg, "
                          f"yaw_rel post-arret={final_check:+.1f} deg, "
                          f"yaw_ground post-arret={final_ground:+.1f} deg, "
                          f"delta yaw_rel sur ce run={final_check - yaw_rel_0:+.1f} deg, "
                          f"delta yaw_ground sur ce run={final_ground - yaw_ground_0:+.1f} deg, "
                          f"{commands_sent} commande(s) envoyee(s)")
            return success

        while True:
            self._check_abort()
            now = time.time()

            # 2026-07-29, revised the same day after a hardware test: the gimbal
            # is NO LONGER commanded here. The first attempt (active visual
            # tracking through _track_beacon_gimbal_tick for the whole of phase 2)
            # produced an oscillating image error (-2.9 -> +13.9 -> -22.9 deg while
            # the chassis turned) and the run failed on "no progress" -- a sign
            # that our correction is probably fighting a stabilisation already
            # active in the firmware (hypothesis H1). Back to a STATIONARY gimbal
            # during the chassis rotation (as on 2026-07-28), at the user's direct
            # request after that observation.
            #
            # The image-error reading is kept but becomes PASSIVE: the same formula
            # as _track_beacon_gimbal_tick (atan2 on the current pose), never
            # calling _send_gimbal(). The goal is unchanged -- an independent
            # confirmation of yaw_rel before declaring success -- simply without
            # commanding the gimbal any more.
            p = self._get_pose()
            if p is not None and self._fresh_pose() and abs(p[2]) > 0.05:
                last_img_err = math.degrees(math.atan2(p[0], abs(p[2])))

            if now - t0 > timeout_s:
                return _exit(False, f"timeout {timeout_s}s",
                             angle_diff_deg(self._get_yaw_rel(), 0.0))

            yaw_rel = angle_diff_deg(self._get_yaw_rel(), 0.0)  # BUG-081
            err = abs(yaw_rel)

            consecutive_ok = chassis_align_tick(yaw_rel, ALIGN_DEADBAND_DEG, consecutive_ok)
            if consecutive_ok > 0:
                self._send_cmd(0.0, 0.0)
                rospy.loginfo(f"[DOCK] chassis alignment: within tolerance "
                              f"({yaw_rel:+.1f} deg), {consecutive_ok}/{ALIGN_CONSECUTIVE_OK}, "
                              f"ecart image={'N/A' if last_img_err is None else f'{last_img_err:+.1f} deg'}")
                if consecutive_ok >= ALIGN_CONSECUTIVE_OK:
                    # 2026-07-29 (retrait du garde-fou image, cf. docstring) :
                    # yaw_rel converged -> success. The image error is still
                    # logged (free, and useful for BUG-080 and the lateral-drift
                    # item) but no longer gates the conclusion -- a heading
                    # correction cannot make up for a position drift anyway.
                    if last_img_err is not None and abs(last_img_err) >= GIMBAL_NULL_TOL_DEG:
                        rospy.logwarn(f"[DOCK] chassis alignment: yaw_rel converged but "
                                      f"ecart image={last_img_err:+.1f} deg non confirme -- "
                                      f"probable derive laterale (item 12), n'empeche plus "
                                      f"le succes (cf. BUG-080)")
                    img_note = ("image indisponible" if last_img_err is None
                                else f"ecart image={last_img_err:+.1f} deg")
                    return _exit(True, f"convergence stable yaw_rel ({img_note})", yaw_rel)
                time.sleep(1.0 / CMD_RATE_HZ)
                continue

            # Physical stall detection: non-zero commands sent for
            # BLOCKED_CHECK_S without yaw_rel moving significantly.
            if now - t_block_ref > BLOCKED_CHECK_S:
                if chassis_is_blocked(yaw_rel_block_ref, yaw_rel, BLOCKED_MIN_DELTA_DEG, commands_sent):
                    return _exit(False, f"chassis bloque (yaw_rel immobile sur "
                                        f"{BLOCKED_CHECK_S}s malgre commande)", yaw_rel)
                t_block_ref = now
                yaw_rel_block_ref = yaw_rel

            # Detection d'absence de progres / divergence sur une fenetre glissante.
            if now - t_last_progress_check > NO_PROGRESS_WINDOW_S:
                if chassis_no_progress(err_at_last_progress_check, err, NO_PROGRESS_MIN_GAIN_DEG):
                    return _exit(False, f"pas de progres sur {NO_PROGRESS_WINDOW_S}s "
                                        f"({err_at_last_progress_check:.1f} -> {err:.1f} deg)", yaw_rel)
                t_last_progress_check = now
                err_at_last_progress_check = err

            wz = clamp(ALIGN_GAIN * yaw_rel, -ALIGN_MAX_WZ, ALIGN_MAX_WZ)
            self._send_cmd(0.0, wz)
            commands_sent += 1
            time.sleep(1.0 / CMD_RATE_HZ)
            # 2026-07-30 (BUG-084): `err_after` was the ONLY yaw_rel reading in
            # this function that did not go through angle_diff_deg, while
            # `err_before` does. The two halves of the same log sentence therefore
            # showed the same physical angle in two different conventions -- hence
            # lines like "err_before=-106.2 deg ... err_after=+253.8 deg" (captured
            # on hardware 2026-07-29), which read as a spectacular divergence when
            # nothing abnormal was happening. No impact on the command sent (this
            # value is only logged), but a real impact on diagnosis: these are
            # exactly the lines relied on to understand a failed run.
            err_after = abs(angle_diff_deg(self._get_yaw_rel(), 0.0))
            # yaw_ground added here (BUG-080, instrumentation 2026-07-29): a
            # growing gap between yaw_rel and yaw_ground while the chassis turns is
            # the very signature of H1 -- free to log, and it changes nothing about
            # the command sent.
            rospy.loginfo(f"[DOCK] chassis alignment: err_before={yaw_rel:+.1f} deg "
                          f"commande wz={wz:+.1f} erreur_apres={err_after:+.1f} deg "
                          f"yaw_ground={self._get_yaw_ground():+.1f} deg "
                          f"ecart_image={'N/A' if last_img_err is None else f'{last_img_err:+.1f} deg'}")

    def _verify_alignment(self):
        """Measure the REAL beacon bearing in the CHASSIS frame, with the robot
        stopped and settled. Returns the angle in degrees, or None if the
        measurement could not be made (2026-07-30).

        Why this function exists: until now, "chassis aligned on the
        beacon" was inferred from `yaw_rel ~ 0`, which only says "chassis
        aligned on the GIMBAL". The missing step -- is the gimbal still pointing
        at the beacon? -- was never measured. This is the only place in the
        module where that question gets a direct answer.

        Why the measurement is valid in the chassis frame: `_measure()` returns
        a bearing in the CAMERA frame, and documents itself that this frame
        equals the chassis frame when the gimbal is aligned on the chassis.
        C'est exactement la situation ici, par construction : on n'est appele
        only after phase 2 has converged, so |yaw_rel| < ALIGN_DEADBAND_DEG
        (2 deg). The approximation is therefore bounded and known.

        Pourquoi on n'ajoute PAS yaw_rel au resultat pour etre "exact" : le
        relative sign of yaw_rel and the camera bearing is precisely what is
        still unconfirmed on this robot (BUG-080). Adding a
        signe inconnu a une mesure correcte la degraderait. On garde donc la
        mesure brute, bornee a +/-2 deg pres, et on logue yaw_rel a cote pour
        the operator sees for themselves that it is indeed small.

        Why a measurement failure does not fail the alignment: being unable to
        verify is no worse than the previous state (which verified
        nothing). We degrade to "not verified" rather than turning a correct run
        into a failure -- exactly the trap of the image guard removed on
        2026-07-29. A user ABORT, by contrast, still propagates.
        """
        self._stop()
        time.sleep(ALIGN_SETTLE_S)
        yaw_rel_now = angle_diff_deg(self._get_yaw_rel(), 0.0)
        try:
            _rng, bearing, _offaxis = self._measure()
        except DockAbort:
            if self._abort or rospy.is_shutdown():
                raise           # vrai ABORT utilisateur : ne pas l'avaler
            rospy.logwarn("[DOCK] verification : mesure impossible (balise "
                          "perdue ou dispersion excessive) — alignement laisse "
                          "NOT VERIFIED rather than declared failed")
            return None
        rospy.loginfo(f"[DOCK] verification (robot arrete, {ALIGN_SETTLE_S}s de "
                      f"stabilisation): beacon/chassis bearing="
                      f"{bearing:+.1f} deg (yaw_rel residuel={yaw_rel_now:+.1f} deg, "
                      f"tolerance={TOL_BEARING_DEG} deg)")
        return bearing

    def _align_chassis_to_beacon(self, label, timeout_s=TURN_TIMEOUT_MAX_S,
                                 budget_left=None):
        """Full, VERIFIED ALIGN (2026-07-30). Chains (gimbal phase +
        phase chassis), puis MESURE le residu reel a l'arret, et recommence
        while the residual exceeds tolerance and progress is being made.

        Renvoie `(succes, statut, residu_deg)`.

        Why an outer loop rather than a direct correction: correcting a residual
        of R degrees would mean turning the chassis by R, which requires knowing
        the sign relating the camera bearing to the chassis rotation direction --
        that is `_turn_by`, whose sign is precisely suspect number one in BUG-077
        (the robot heading the opposite way). Re-running the gimbal+chassis pair
        does the same work while introducing no new sign assumption: phase 1
        absorbs the residual into `yaw_rel` (it only tracks the beacon visually),
        and phase 2 brings it back to
        zero through the only path whose sign is confirmed on hardware
        (2026-06-26). It is a fixed-point iteration reusing only what is
        already validated.

        Cost in the nominal case: none. If the first pass is already within
        tolerance -- the expected case when all is well -- the loop exits
        immediately and the behaviour is the previous one, plus a measurement.
        """
        prev_residual = None
        last_residual = None
        for attempt in range(1, ALIGN_VERIFY_PASSES + 1):
            if budget_left is not None and budget_left() <= 0:
                return False, "SEQUENCE_TIMEOUT", last_residual

            rospy.loginfo(f"[DOCK] {label} — passe {attempt}/{ALIGN_VERIFY_PASSES} "
                          f"(a) alignement nacelle sur la balise")
            gim_timeout = (GIMBAL_NULL_TIMEOUT_S if budget_left is None
                           else min(GIMBAL_NULL_TIMEOUT_S, budget_left()))
            if not self._align_gimbal_to_beacon(timeout_s=gim_timeout):
                return False, "GIMBAL_ALIGN_FAILED", last_residual
            ref_ground = self._gimbal_ref_ground

            rospy.loginfo(f"[DOCK] {label} — passe {attempt}/{ALIGN_VERIFY_PASSES} "
                          f"(b) alignement chassis (yaw_rel -> 0)")
            chassis_timeout = (timeout_s if budget_left is None
                               else min(timeout_s, budget_left()))
            if not self._align_chassis_yaw_rel(timeout_s=chassis_timeout):
                return False, "CHASSIS_ALIGN_FAILED", last_residual

            # Reference-drift witness: if the gimbal's ABSOLUTE heading
            # a bouge entre la fin de la phase 1 et la fin de la phase 2, alors
            # "the chassis is aligned on the gimbal" no longer means "the
            # chassis is aligned on the beacon". Free to check since H1 was
            # confirmed on 2026-07-29.
            if ref_ground is not None:
                drift = angle_diff_deg(self._get_yaw_ground(), ref_ground)
                if abs(drift) > ALIGN_REF_DRIFT_MAX_DEG:
                    rospy.logwarn(f"[DOCK] {label}: the gimbal reference DRIFTED by "
                                  f"{drift:+.1f} deg (absolute heading) during the chassis "
                                  f"rotation, beyond the {ALIGN_REF_DRIFT_MAX_DEG} deg "
                                  f"tolerated -- the alignment obtained therefore targets a "
                                  f"direction differente de celle mesuree en phase 1")
                else:
                    rospy.loginfo(f"[DOCK] {label} : reference nacelle stable "
                                  f"(derive cap absolu={drift:+.1f} deg)")

            residual = self._verify_alignment()
            if residual is None:
                return True, "ALIGN_DONE_UNVERIFIED", None
            last_residual = residual

            verdict = align_verify_verdict(residual, TOL_BEARING_DEG,
                                           prev_residual, ALIGN_VERIFY_MIN_GAIN_DEG,
                                           attempt, ALIGN_VERIFY_PASSES)
            if verdict == "ok":
                rospy.loginfo(f"[DOCK] {label} : ALIGNEMENT CONFIRME en {attempt} "
                              f"passe(s) — gisement residuel={residual:+.1f} deg "
                              f"(<= {TOL_BEARING_DEG} deg)")
                return True, "ALIGN_DONE", residual
            if verdict == "exhausted":
                rospy.logerr(f"[DOCK] {label} : {ALIGN_VERIFY_PASSES} passes epuisees, "
                             f"gisement residuel={residual:+.1f} deg toujours hors "
                             f"tolerance ({TOL_BEARING_DEG} deg)")
                return False, "ALIGN_NOT_CONVERGED", residual
            if verdict == "no_gain":
                rospy.logerr(f"[DOCK] {label} : passe {attempt} sans gain reel "
                             f"({prev_residual:+.1f} -> {residual:+.1f} deg, "
                             f"< {ALIGN_VERIFY_MIN_GAIN_DEG} deg) -- one more pass "
                             f"n'y changerait rien, arret")
                return False, "ALIGN_NOT_CONVERGED", residual

            rospy.logwarn(f"[DOCK] {label} : gisement residuel={residual:+.1f} deg "
                          f"hors tolerance ({TOL_BEARING_DEG} deg) — passe "
                          f"supplementaire")
            prev_residual = residual

        return False, "ALIGN_NOT_CONVERGED", last_residual

    def _drive_by(self, dist_m):
        """Drive straight for dist_m (>0 only -- reversing
        is not used by the manoeuvre, and it is sensor-blind).
        Servoed on the displacement measured in `/odom`. Gimbal servoed on the
        beacon during the drive (`_track_beacon_gimbal_tick`, 2026-07-27)."""
        if dist_m < DRIVE_TOL_M:
            return
        if dist_m > ABSURD_SEGMENT_M:
            rospy.logerr(f"[DOCK] planned segment is absurd ({dist_m:.2f} m > "
                         f"{ABSURD_SEGMENT_M} m) -- the measurement must be wrong, aborting")
            raise DockAbort()
        od = self._get_odom()
        if od is None:
            raise DockAbort()
        self._probe_gimbal_sign()
        x0, y0 = od[0], od[1]
        timeout = min(DRIVE_TIMEOUT_MAX_S,
                      max(3.0, 3.0 * dist_m / max(DRIVE_VX_M_S, 0.01)))
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._check_abort()
            od = self._get_odom()
            if od is None:
                break
            travelled = math.hypot(od[0] - x0, od[1] - y0)
            if travelled >= dist_m - DRIVE_TOL_M:
                break
            self._send_cmd(DRIVE_VX_M_S, 0.0)
            self._track_beacon_gimbal_tick()
            time.sleep(1.0 / CMD_RATE_HZ)
        self._stop()
        self._send_gimbal(0.0)
        time.sleep(0.3)
        od = self._get_odom()
        if od is not None:
            got = math.hypot(od[0] - x0, od[1] - y0)
            rospy.loginfo(f"[DOCK] avance demandee={dist_m:.2f} m obtenue={got:.2f} m")

    # ---------------------------------------------------------
    # Mesure
    # ---------------------------------------------------------

    def _measure(self):
        """Aggregate MEAS_SAMPLES poses (robot stationary, gimbal aligned on the
        chassis) and
        renvoie (range_m, bearing_deg, offaxis_deg).

        The 2D frame used throughout this module (== the chassis frame, since the
        gimbal is aligned):
          * "forward" = camera optical axis
          * "right"   = +x camera
          * bearing   = atan2(x, |z|)  -> positive means the beacon is to the RIGHT
            (a formula STRICTLY identical to LOCK's and to `_gimbal_servo_yaw` in
            rm_cam_beacon.py, validated on hardware on 2026-06-26 -- we do not
            re-derive a convention that is already proven)
          * range    = hypot(x, z)  -> true distance, not |z|. The difference
            from |z| (used by APPROACH as "depth") is negligible head-on but
            real at an angle -- precisely the case docking targets.
          * offaxis  = BEACON_YAW_SIGN * yaw + BEACON_YAW_OFFSET_DEG
            -> 0 = on voit la balise de face (on est sur son axe frontal)
        """
        MEAS_MAX_ATTEMPTS = 3   # nb de lots re-tentes si dispersion excessive
        for attempt in range(1, MEAS_MAX_ATTEMPTS + 1):
            samples = []
            t0 = time.time()
            last_stamp = 0.0
            stale_skipped = 0
            while len(samples) < MEAS_SAMPLES:
                self._check_abort()
                if time.time() - t0 > MEAS_TIMEOUT_S:
                    rospy.logerr(f"[DOCK] mesure : seulement {len(samples)}/{MEAS_SAMPLES} "
                                 f"poses en {MEAS_TIMEOUT_S}s — balise perdue ?")
                    raise DockAbort()
                p = self._get_pose()
                if p is None or (time.time() - p[4]) > POSE_TIMEOUT_S:
                    time.sleep(0.05)
                    continue
                if p[4] == last_stamp:      # same pose as the previous loop (no new message yet)
                    time.sleep(0.05)
                    continue
                last_stamp = p[4]
                # Pose republiee par carolus_astrobee.cpp (getFilteredPose,
                # lignes 560-598) : valeurs brutes identiques a la reception
                # previous one despite a new message and timestamp. Not an
                # independent observation -> it does not count in the sample.
                if self._get_pose_repeat_count() > 0:
                    stale_skipped += 1
                    time.sleep(0.05)
                    continue
                x, _y, z, yaw_deg = p[0], p[1], p[2], p[3]
                if abs(z) < 0.05:
                    continue
                rng = math.hypot(x, z)
                bearing = math.degrees(math.atan2(x, abs(z)))
                offaxis = BEACON_YAW_SIGN * yaw_deg + BEACON_YAW_OFFSET_DEG
                offaxis = angle_diff_deg(offaxis, 0.0)   # ramene dans [-180, 180]
                samples.append((rng, bearing, offaxis))
                time.sleep(0.05)

            rng = median([s[0] for s in samples])
            bearing = median([s[1] for s in samples])
            offaxis = median([s[2] for s in samples])

            # Dispersion : un P4P qui saute d'une solution a l'autre se voit ici.
            spread = max(s[2] for s in samples) - min(s[2] for s in samples)
            rospy.loginfo(f"[DOCK] mesure (essai {attempt}/{MEAS_MAX_ATTEMPTS}) : "
                          f"range={rng:.3f} m bearing={bearing:+.1f} deg "
                          f"offaxis={offaxis:+.1f} deg (dispersion={spread:.1f} deg sur "
                          f"{len(samples)} poses, {stale_skipped} pose(s) repetee(s) ignoree(s))")
            if spread <= MEAS_MAX_SPREAD_DEG:
                return rng, bearing, offaxis
            rospy.logwarn(f"[DOCK] dispersion angulaire elevee ({spread:.1f} deg > "
                          f"{MEAS_MAX_SPREAD_DEG}) — mesure rejetee, "
                          f"{'nouvelle tentative' if attempt < MEAS_MAX_ATTEMPTS else 'abandon'}")

        rospy.logerr(f"[DOCK] mesure : dispersion excessive sur {MEAS_MAX_ATTEMPTS} "
                     f"tentatives — orientation balise non exploitable, abandon")
        raise DockAbort()

    # ---------------------------------------------------------
    # Planification de la manœuvre
    # ---------------------------------------------------------

    @staticmethod
    def plan_maneuver(rng, bearing_deg, offaxis_deg, dock_distance=DOCK_DISTANCE_M):
        """Compute the turn-drive-turn manoeuvre that brings the robot to the
        docking point. A PURE function (no I/O, no state), therefore testable off
        the robot -- see the __main__ block at the end of this file.

        Geometry, in the 2D frame described by `_measure` (robot at the origin,
        looking "forward"):

          B  = beacon position                  = rng * (sin(bearing), cos(bearing))
          Seen from the beacon, the B->robot vector makes an angle `offaxis` with
          the normal to its face. The beacon's outward normal is therefore obtained
          by
          faisant tourner la direction B->robot de -offaxis.
          G  = point de docking                 = B + dock_distance * normale
          On veut finir EN G, tourne vers B.

        Returns (turn1_deg, drive_m, turn2_deg), positive angles = to the
        right (same convention as `bearing`)."""
        br = math.radians(bearing_deg)
        bx = rng * math.sin(br)      # composante droite
        bf = rng * math.cos(br)      # composante avant

        # Beacon -> robot direction, expressed as an angle in our frame.
        phi = math.atan2(-bx, -bf)
        # Outward normal of the beacon's face: cancel the off-axis angle.
        n = phi - math.radians(offaxis_deg)

        gx = bx + dock_distance * math.sin(n)
        gf = bf + dock_distance * math.cos(n)

        drive = math.hypot(gx, gf)

        # Degenerate case: the robot is ALREADY at the docking point. `atan2(gx,
        # gf)` would then be operating on numerical noise and return an arbitrary
        # direction (measured: -90 deg for a robot that was perfectly placed). All
        # that remains is to pivot toward the beacon. Covered by self-test case 1.
        if drive < DRIVE_TOL_M:
            return 0.0, 0.0, bearing_deg

        turn1 = math.degrees(math.atan2(gx, gf))
        # Once at G and oriented along turn1, the angle left to face the beacon is
        # the difference between the G->B direction and turn1.
        head_gb = math.degrees(math.atan2(bx - gx, bf - gf))
        turn2 = angle_diff_deg(head_gb, turn1)
        return turn1, drive, turn2

    # ---------------------------------------------------------
    # Sequences
    # ---------------------------------------------------------

    def _run(self, fn):
        """Common wrapper: flags, clean stop, and never an exception that
        laisserait le robot en mouvement."""
        self._busy = True
        self._abort = False
        try:
            fn()
        except DockAbort:
            self._status = "ABORTED"
            rospy.logwarn("[DOCK] sequence interrompue")
        except Exception as e:                      # noqa: BLE001 — filet de securite
            self._status = "ERROR"
            rospy.logerr(f"[DOCK] erreur inattendue : {e}")
        finally:
            self._stop()
            self._busy = False
            rospy.loginfo(f"[DOCK] etat final : {self._status}")

    def _align_only(self):
        """ALIGN_ONLY command (2026-07-28, user request: be able to test the
        chassis rotation in isolation, never advancing).

        Reuses exactly phases 1+2 of `_dock_sequence` (gimbal then chassis
        alignment, same functions, same guards) but stops there: NO call to
        `_measure()`/`_drive_by()`, so no advance is possible under any
        condition.

        Why phase 1 (gimbal) is included when the request only mentions the
        chassis: `yaw_rel` (used by phase 2 to orient the chassis) is the
        gimbal/chassis angle, NOT the chassis/beacon angle. It only makes sense
        as a reference for aligning the chassis on the beacon IF the gimbal is
        already pointing at the beacon (phase 1). Without that, phase 2 would
        align the chassis on an arbitrary direction. So this is not an added
        feature: it is the same dependency that already exists in
        `_dock_sequence`, reused as-is."""
        self._status = "DOCKING"
        rospy.loginfo("[DOCK] === ALIGN_ONLY: gimbal then chassis alignment, NO drive ===")
        self._take_control()
        self._stop()

        if not self._fresh_pose():
            rospy.logerr("[DOCK] ALIGN_ONLY : aucune pose fraiche sur /pose — "
                         "balise non visible, abandon")
            self._status = "NO_BEACON"
            return

        # 2026-07-30: goes through the VERIFIED loop rather than calling the two
        # phases back to back. Same work, plus a control measurement at rest
        # between passes -- and that measurement is what was missing for "SUCCESS"
        # to mean anything (the 2026-07-29 run reported SUCCESS with a real 26 deg
        # error).
        ok, status, residual = self._align_chassis_to_beacon("ALIGN_ONLY",
                                                             timeout_s=TURN_TIMEOUT_MAX_S)
        self._status = status
        if not ok:
            rospy.logerr(f"[DOCK] ALIGN_ONLY: FAILED ({status}"
                         f"{'' if residual is None else f', residual bearing={residual:+.1f} deg'})"
                         f" -- no drive performed")
            return

        if status == "ALIGN_DONE_UNVERIFIED":
            rospy.logwarn("[DOCK] ALIGN_ONLY termine mais NON VERIFIE : "
                          "yaw_rel a converge, la mesure de controle n'a pas pu "
                          "etre faite (balise perdue a l'arret ?)")
            return

        rospy.loginfo(f"[DOCK] ALIGN_ONLY done: chassis aligned on the beacon, "
                      f"verifie a {residual:+.1f} deg, aucune avance effectuee")

    def _approach_only(self):
        """APPROACH_ONLY command (2026-07-28, user request: drive only, never
        rotating the chassis).

        Verifies FIRST that the chassis is already aligned (same tolerance as
        `_align_chassis_yaw_rel`, TURN_TOL_DEG) before any movement. If it is
        not: no drive, an explicit NOT_ALIGNED status, and no automatic
        correction attempt.

        One point flagged for transparency (not a chassis rotation, but worth
        mentioning): `_drive_by()`, reused here as-is, does move the GIMBAL
        slightly (not the chassis) during the drive -- `_track_beacon_gimbal_tick()`
        on each iteration to keep the beacon in frame while the robot advances,
        plus a one-off `_probe_gimbal_sign()` the very first time (a mechanism
        that already existed, added 2026-07-27, unmodified here). Neither
        rotates the chassis."""
        self._status = "DOCKING"
        rospy.loginfo("[DOCK] === APPROACH_ONLY: drive only, NO chassis rotation ===")
        self._take_control()
        self._stop()

        if not self._fresh_pose():
            rospy.logerr("[DOCK] APPROACH_ONLY : aucune pose fraiche sur /pose — "
                         "balise non visible, abandon")
            self._status = "NO_BEACON"
            return

        yaw_rel = self._get_yaw_rel()
        if abs(yaw_rel) >= TURN_TOL_DEG:
            self._status = "NOT_ALIGNED"
            rospy.logerr(f"[DOCK] APPROACH_ONLY: robot not aligned "
                         f"(yaw_rel={yaw_rel:+.1f} deg, tolerance="
                         f"{TURN_TOL_DEG:.1f} deg) -- no drive. "
                         f"Run ALIGN_ONLY first.")
            return

        rng, bearing, off = self._measure()
        rospy.loginfo(f"[DOCK] APPROACH_ONLY : deja aligne (yaw_rel={yaw_rel:+.1f} deg), "
                      f"avance de {max(0.0, rng - DOCK_DISTANCE_M):.2f} m "
                      f"(range mesure={rng:.2f} m, cible={DOCK_DISTANCE_M:.2f} m)")
        self._drive_by(max(0.0, rng - DOCK_DISTANCE_M))
        rng2, bearing2, off2 = self._measure()
        rospy.loginfo(f"[DOCK] APPROACH_ONLY termine : range={rng2:.3f} m "
                      f"bearing={bearing2:+.1f} deg (offaxis={off2:+.1f} deg "
                      f"non corrige)")
        self._status = "APPROACH_DONE"

    def _dock_sequence(self):
        self._status = "DOCKING"
        rospy.loginfo("[DOCK] === debut docking ===")
        self._take_control()
        self._stop()

        if not self._fresh_pose():
            rospy.logerr("[DOCK] no fresh pose on /pose -- beacon not visible, aborting")
            self._status = "NO_BEACON"
            return

        if SIMPLE_APPROACH_ONLY or not BEACON_YAW_VALIDATED:
            # Simple mode (see SIMPLE_APPROACH_ONLY at the top of the file) or the
            # documented fallback (orientation convention not validated, so the
            # frontal axis cannot be targeted): no frontal-axis alignment (offaxis
            # ignore), pas de boucle de convergence.
            #
            # Pipeline en 3 phases (2026-07-27, demande utilisateur suite au
            # comportement observe avec un simple _turn_by(bearing) direct) :
            #   1. Align the GIMBAL on the beacon (image servo, the same one
            #      already used during the drive -- _track_beacon_gimbal_tick).
            #   2. Align the CHASSIS by nulling yaw_rel -- NOT through _turn_by,
            #      which assumes an odom-yaw / camera-bearing relationship that
            #      was never confirmed and is suspect number one in "the robot
            #      heads the opposite way". Instead we reuse a fact ALREADY
            #      CONFIRMED on this robot and this command path
            #      (/carolus/cmd_vel): rm_cam_beacon.py's ALIGN state
            #      (2026-06-26, K_BODY_YAW) -- wz = +K*yaw_rel DECREASES yaw_rel
            #      (measured: 106 deg -> 1.6 deg with wz=+10). Once yaw_rel ~ 0
            #      and the gimbal is on the beacon (phase 1), the chassis is de
            #      facto pointed at it.
            #   3. Drive straight (the chassis is already oriented) to
            #      DOCK_DISTANCE_M, with the gimbal still tracking (_drive_by).
            reason = ("simple mode requested" if SIMPLE_APPROACH_ONLY
                      else "beacon orientation convention NOT validated")
            rospy.loginfo(f"[DOCK] {reason} -> 3-phase pipeline: "
                          f"align gimbal, align chassis (yaw_rel), drive")

            seq_t0 = time.time()

            def _seq_timeout_left():
                return SEQUENCE_TIMEOUT_S - (time.time() - seq_t0)

            # Phases 1+2 : boucle VERIFIEE (2026-07-30). Les deux garde-fous
            # historical guards remain in force unchanged -- a gimbal failure
            # (2026-07-27, the direct cause of a collision) and a chassis failure
            # (2026-07-28) both still forbid driving. What changes: success itself
            # is now MEASURED rather than inferred from yaw_rel alone. A residual
            # outside tolerance also forbids driving, where before it went
            # unnoticed -- phase 3's `_measure()` was already measuring that
            # bearing... and throwing it away (see `_off`, ignored below).
            rospy.loginfo("[DOCK] phases 1-2/3: gimbal + chassis alignment (verified)")
            if _seq_timeout_left() <= 0:
                self._status = "SEQUENCE_TIMEOUT"
                rospy.logerr("[DOCK] aborting: overall timeout reached before phase 1 even started")
                return
            ok, status, residual = self._align_chassis_to_beacon(
                "docking phases 1-2",
                timeout_s=TURN_TIMEOUT_MAX_S,
                budget_left=_seq_timeout_left)
            if not ok:
                self._status = status
                rospy.logerr(f"[DOCK] aborting: alignment not reached ({status}"
                             f"{'' if residual is None else f', residual={residual:+.1f} deg'})"
                             f" -- no drive (safety)")
                return
            if status == "ALIGN_DONE_UNVERIFIED":
                self._status = "CHASSIS_ALIGN_FAILED"
                rospy.logerr("[DOCK] aborting: alignment NOT VERIFIABLE (control "
                             "measurement impossible) -- no drive. Unlike "
                             "ALIGN_ONLY, driving commits the robot toward an "
                             "obstacle : on n'avance pas sur un alignement non "
                             "confirme.")
                return

            rospy.loginfo("[DOCK] phase 3/3 : avance")
            if _seq_timeout_left() <= 0:
                self._status = "SEQUENCE_TIMEOUT"
                rospy.logerr("[DOCK] aborting: overall timeout reached before phase 3")
                self._stop()
                return
            rng, bearing, _off = self._measure()
            self._drive_by(max(0.0, rng - DOCK_DISTANCE_M))
            rng2, bearing2, off2 = self._measure()
            rospy.loginfo(f"[DOCK] approche simple terminee : range={rng2:.3f} m "
                          f"bearing={bearing2:+.1f} deg (offaxis={off2:+.1f} deg "
                          f"non corrige)")
            self._status = "RANGE_ONLY"
            return

        self._null_gimbal()

        prev_offaxis = None
        for i in range(1, MAX_ITERATIONS + 1):
            self._check_abort()
            rospy.loginfo(f"[DOCK] --- iteration {i}/{MAX_ITERATIONS} ---")
            self._null_gimbal()
            rng, bearing, offaxis = self._measure()

            if (abs(rng - DOCK_DISTANCE_M) < TOL_RANGE_M
                    and abs(offaxis) < TOL_OFFAXIS_DEG
                    and abs(bearing) < TOL_BEARING_DEG):
                rospy.loginfo(f"[DOCK] ✅ docke : range={rng:.3f} m "
                              f"offaxis={offaxis:+.1f} deg bearing={bearing:+.1f} deg "
                              f"(iteration {i})")
                self._status = "DOCKED"
                return

            # Anti-divergence guard: if the off-axis angle does not improve, that
            # is the classic symptom of a wrong BEACON_YAW_SIGN -> stop rather than
            # circling the beacon indefinitely.
            if prev_offaxis is not None:
                gain = abs(prev_offaxis) - abs(offaxis)
                if gain < MIN_PROGRESS_DEG:
                    rospy.logerr(f"[DOCK] no progress on the off-axis angle "
                                 f"({abs(prev_offaxis):.1f} -> {abs(offaxis):.1f} deg). "
                                 f"Most likely cause: BEACON_YAW_SIGN inverted. "
                                 f"Re-run CALIBRATE mode. Stopping.")
                    self._status = "NO_PROGRESS"
                    return
            prev_offaxis = offaxis

            turn1, drive, turn2 = self.plan_maneuver(rng, bearing, offaxis)
            rospy.loginfo(f"[DOCK] manœuvre : tourner {turn1:+.1f} deg, "
                          f"avancer {drive:.2f} m, tourner {turn2:+.1f} deg")

            self._turn_by(turn1)
            if drive > MAX_SEGMENT_M:
                # Detour long (cas typique : tres hors-axe et loin). On n'avance
                # pas 3 m en aveugle sur une mesure a 2.5 Hz : on tronque, et on
                # laisse l'iteration suivante re-planifier depuis une mesure
                # fraiche. `turn2` est volontairement SAUTE — il n'a de sens
                # qu'arrive au point de docking, pas a mi-chemin.
                rospy.loginfo(f"[DOCK] segment tronque a {MAX_SEGMENT_M} m "
                              f"(plan={drive:.2f} m) — re-mesure a l'iteration suivante")
                self._drive_by(MAX_SEGMENT_M)
                # Manœuvre incomplete : le controle anti-divergence ci-dessus
                # compares two END-of-manoeuvre states. Neutralise it for the next
                # round, otherwise a truncated detour -- whose angular progress is
                # normally small -- would be mistaken for an inverted sign.
                prev_offaxis = None
                continue
            self._drive_by(drive)
            self._turn_by(turn2)

        rospy.logwarn(f"[DOCK] {MAX_ITERATIONS} iterations without converging -- stopping")
        self._status = "NOT_CONVERGED"

    def _calibrate_step1(self):
        """Establish the beacon orientation convention, the one unknown the robot
        cannot measure by itself.

        A two-independent-click protocol: validating a monocular orientation
        without a test bench or motion capture. Deliberately NO blocking timer
        between the two steps (the initial version embedded a fixed 20 s delay in
        the same sequence, unreadable when the T5 logs are scrolling -- fixed on
        2026-07-27): each step waits for an explicit command from its own GUI
        button, to be run at the user's own pace rather than inside a window that
        can be missed.

          1. (CALIBRATE) Placer le robot EN FACE de la balise, sur son axe
             frontal, a ~1 m, PUIS cliquer CALIBRATE. -> on lit yaw_face.
          2. (CAL STEP 2) Move the robot about 30 deg TO THE RIGHT of the
             beacon (beacon stationary), THEN click CAL STEP 2 once in place.
             -> yaw_right is read.
        From (1) we get the offset; from the DIRECTION of change between (1)
        and (2) we get the sign.
        """
        self._status = "CALIBRATING"
        self._take_control()
        self._stop()

        rospy.loginfo("[DOCK][CAL] Etape 1/2 — mesure en cours (3 s, ne pas bouger "
                      "le robot)...")
        for _ in range(3):
            self._check_abort()
            time.sleep(1.0)
        self._null_gimbal()
        p = self._get_pose()
        if p is None or not self._fresh_pose():
            rospy.logerr("[DOCK][CAL] no fresh pose -- aborting. Check the beacon is "
                         "visible, then re-run CALIBRATE.")
            self._status = "CAL_FAILED"
            return
        self._cal_yaw_face = median([self._get_pose()[3] for _ in range(3)])
        rospy.loginfo(f"[DOCK][CAL] step 1 OK: raw head-on yaw = "
                      f"{self._cal_yaw_face:+.1f} deg")
        rospy.loginfo("[DOCK][CAL] -> deplace maintenant le robot d'environ 30 deg "
                      "vers la DROITE de la balise (balise immobile), a ton rythme, "
                      "puis clique CAL STEP 2. Statut GUI : CAL_STEP1_DONE.")
        self._status = "CAL_STEP1_DONE"

    def _calibrate_step2(self):
        if self._cal_yaw_face is None:
            rospy.logerr("[DOCK][CAL] etape 1 pas encore faite — clique d'abord "
                         "CALIBRATE.")
            self._status = "CAL_FAILED"
            return
        self._status = "CALIBRATING"
        self._take_control()
        self._stop()

        rospy.loginfo("[DOCK][CAL] Etape 2/2 — mesure en cours (3 s, ne pas bouger "
                      "le robot)...")
        for _ in range(3):
            self._check_abort()
            time.sleep(1.0)
        self._null_gimbal()
        if not self._fresh_pose():
            rospy.logerr("[DOCK][CAL] pas de pose fraiche — abandon")
            self._status = "CAL_FAILED"
            return
        yaw_right = median([self._get_pose()[3] for _ in range(3)])
        rospy.loginfo(f"[DOCK][CAL] etape 2 OK : yaw brut de biais = "
                      f"{yaw_right:+.1f} deg")

        delta = angle_diff_deg(yaw_right, self._cal_yaw_face)
        if abs(delta) < 5.0:
            rospy.logerr(f"[DOCK][CAL] variation trop faible ({delta:+.1f} deg) : "
                         f"soit le robot n'a pas ete deplace, soit l'orientation "
                         f"renvoyee par Carolus n'est pas exploitable a cette "
                         f"distance. Calibration NON concluante — clique CALIBRATE "
                         f"pour recommencer depuis l'etape 1.")
            self._status = "CAL_INCONCLUSIVE"
            self._cal_yaw_face = None
            return

        sign = +1.0 if delta > 0 else -1.0
        offset = -sign * self._cal_yaw_face
        rospy.loginfo("[DOCK][CAL] ================ RESULTAT ================")
        rospy.loginfo(f"[DOCK][CAL] BEACON_YAW_SIGN       = {sign:+.1f}")
        rospy.loginfo(f"[DOCK][CAL] BEACON_YAW_OFFSET_DEG = {offset:+.1f}")
        rospy.loginfo("[DOCK][CAL] BEACON_YAW_VALIDATED  = True")
        rospy.loginfo("[DOCK][CAL] -> reporter ces 3 valeurs en tete de "
                      "beacon_docking.py, puis relancer le nœud.")
        rospy.loginfo("[DOCK][CAL] ===========================================")
        self._status = "CAL_DONE"
        self._cal_yaw_face = None


# =========================================================
# Auto-test de la geometrie (hors robot)
# =========================================================

def _self_test():
    """Verifie `plan_maneuver` sur des cas dont la solution est evidente a la
    main. Executable sans ROS ni robot : `python3 beacon_docking.py --selftest`."""
    ok = True

    def check(name, got, expected, tol):
        """Comparaison ANGULAIRE (gere le repliement +/-180)."""
        nonlocal ok
        good = abs(angle_diff_deg(got, expected)) < tol
        print(f"  {'OK ' if good else 'FAIL'} {name}: attendu {expected:+.2f}, obtenu {got:+.2f}")
        ok = ok and good

    def check_val(name, got, expected, tol):
        """Comparaison SCALAIRE (distances)."""
        nonlocal ok
        good = abs(got - expected) < tol
        print(f"  {'OK ' if good else 'FAIL'} {name}: attendu {expected:.2f}, obtenu {got:.2f}")
        ok = ok and good

    print("Cas 1 — deja parfaitement docke (de face, a la bonne distance) :")
    t1, d, t2 = BeaconDocking.plan_maneuver(DOCK_DISTANCE_M, 0.0, 0.0)
    check("turn1", t1, 0.0, 1e-6)
    print(f"  {'OK ' if abs(d) < 1e-6 else 'FAIL'} drive: attendu 0.00, obtenu {d:.2f}")
    ok = ok and abs(d) < 1e-6
    check("turn2", t2, 0.0, 1e-6)

    print("Cas 2 — de face, trop loin (2 m au lieu de 0.70) :")
    t1, d, t2 = BeaconDocking.plan_maneuver(2.0, 0.0, 0.0)
    check("turn1", t1, 0.0, 1e-6)
    print(f"  {'OK ' if abs(d - (2.0 - DOCK_DISTANCE_M)) < 1e-6 else 'FAIL'} "
          f"drive: expected {2.0 - DOCK_DISTANCE_M:.2f}, got {d:.2f}")
    ok = ok and abs(d - (2.0 - DOCK_DISTANCE_M)) < 1e-6
    check("turn2", t2, 0.0, 1e-6)

    print("Case 3 -- beacon seen square on but offset to the right (bearing=+20):")
    # Seen square on (offaxis=0) => we are already on its frontal axis, so all
    # that is needed is to pivot toward it and adjust the distance.
    t1, d, t2 = BeaconDocking.plan_maneuver(2.0, 20.0, 0.0)
    check("turn1", t1, 20.0, 1e-6)
    print(f"  {'OK ' if abs(d - (2.0 - DOCK_DISTANCE_M)) < 1e-6 else 'FAIL'} "
          f"drive: expected {2.0 - DOCK_DISTANCE_M:.2f}, got {d:.2f}")
    ok = ok and abs(d - (2.0 - DOCK_DISTANCE_M)) < 1e-6
    check("turn2", t2, 0.0, 1e-6)

    # Off-axis cases: we do not know the answer by hand, so instead we check the
    # 3 PROPERTIES the manoeuvre must guarantee by construction, replaying the
    # geometry from its own output.
    for rng_in, bearing_in, offaxis_in in [(2.0, 0.0, 40.0), (1.5, -25.0, -35.0),
                                           (3.0, 15.0, 70.0), (1.2, 40.0, -10.0)]:
        print(f"Off-axis case -- range={rng_in} bearing={bearing_in:+.0f} "
              f"offaxis={offaxis_in:+.0f} :")
        t1, d, t2 = BeaconDocking.plan_maneuver(rng_in, bearing_in, offaxis_in)
        print(f"  info  turn1={t1:+.1f} deg  drive={d:.2f} m  turn2={t2:+.1f} deg")

        br = math.radians(bearing_in)
        bx, bf = rng_in * math.sin(br), rng_in * math.cos(br)          # beacon
        gx, gf = d * math.sin(math.radians(t1)), d * math.cos(math.radians(t1))  # arrival

        # (1) the robot ends up DOCK_DISTANCE_M from the beacon
        final_range = math.hypot(bx - gx, bf - gf)
        check_val("final distance", final_range, DOCK_DISTANCE_M, 1e-6)

        # (2) the robot ends up facing the beacon (final heading == G->B direction)
        heading_final = t1 + t2
        dir_gb = math.degrees(math.atan2(bx - gx, bf - gf))
        check("final heading toward the beacon", heading_final, dir_gb, 1e-6)

        # (3) the robot ends up ON the beacon's FRONTAL AXIS: the beacon->robot
        #     direction must coincide with the outward normal of its face.
        phi_in = math.atan2(-bx, -bf)
        normal = math.degrees(phi_in - math.radians(offaxis_in))
        dir_bg = math.degrees(math.atan2(gx - bx, gf - bf))
        check("robot on the frontal axis", dir_bg, normal, 1e-6)

    # 2026-07-28: _align_chassis_yaw_rel's decision logic, extracted into pure
    # functions so it can be exercised here without ROS or a robot.

    def check_bool(name, got, expected):
        nonlocal ok
        good = got == expected
        print(f"  {'OK ' if good else 'FAIL'} {name}: attendu {expected}, obtenu {got}")
        ok = ok and good

    print("Chassis — succes d'alignement (3 lectures consecutives requises) :")
    c = 0
    c = chassis_align_tick(1.0, 2.0, c)   # dans tolerance -> 1
    check_val("compteur apres 1ere lecture OK", c, 1, 1e-9)
    c = chassis_align_tick(15.0, 2.0, c)  # hors tolerance -> reset a 0
    check_val("compteur reinitialise si une lecture sort de tolerance", c, 0, 1e-9)
    c = chassis_align_tick(0.5, 2.0, c)
    c = chassis_align_tick(0.5, 2.0, c)
    c = chassis_align_tick(0.5, 2.0, c)
    check_val("compteur apres 3 lectures OK consecutives", c, 3, 1e-9)

    print("Chassis — absence de progres / divergence :")
    check_bool("pas de progres (erreur quasi inchangee)",
               chassis_no_progress(10.0, 9.5, 1.0), True)
    check_bool("progres suffisant (erreur nettement reduite)",
               chassis_no_progress(10.0, 5.0, 1.0), False)
    check_bool("divergence (erreur qui augmente)",
               chassis_no_progress(10.0, 15.0, 1.0), True)

    print("Chassis — detection de blocage physique :")
    check_bool("bloque (commandes envoyees, yaw_rel immobile)",
               chassis_is_blocked(50.0, 49.5, 1.0, commands_sent=5), True)
    check_bool("not stalled (yaw_rel moved enough)",
               chassis_is_blocked(50.0, 40.0, 1.0, commands_sent=5), False)
    check_bool("no verdict before the first command (avoids a startup false positive)",
               chassis_is_blocked(50.0, 50.0, 1.0, commands_sent=0), False)
    # 2026-07-30: non-regression for the switch to angle_diff_deg. With the old
    # direct subtraction this case (a chassis STRICTLY stationary straddling
    # +/-180) gave |diff|=358 deg and therefore answered "not stalled".
    check_bool("stalled even straddling +/-180 deg (former false negative)",
               chassis_is_blocked(179.7, -179.8, 1.0, commands_sent=5), True)

    print("\nGimbal -- multi-reading confirmation (phase 1):")
    check_bool("1re lecture sous tolerance ne suffit plus",
               gimbal_confirm_tick(1.0, 3.0, 0) >= GIMBAL_CONFIRM_OK, False)
    check_val("compteur incremente sous tolerance",
              gimbal_confirm_tick(1.0, 3.0, 2), 3.0, 0.001)
    check_val("compteur remis a zero hors tolerance",
              gimbal_confirm_tick(5.0, 3.0, 2), 0.0, 0.001)

    print("\nTerminal alignment verification:")
    check_bool("residual within tolerance -> done",
               align_verify_verdict(3.0, 6.0, None, 2.0, 1, 3) == "ok", True)
    check_bool("residual outside tolerance on pass 1 -> retry",
               align_verify_verdict(20.0, 6.0, None, 2.0, 1, 3) == "retry", True)
    check_bool("progres franc entre deux passes -> on retente",
               align_verify_verdict(9.0, 6.0, 20.0, 2.0, 2, 3) == "retry", True)
    check_bool("pass with no real gain -> stop (do not wear the mechanics)",
               align_verify_verdict(19.0, 6.0, 20.0, 2.0, 2, 3) == "no_gain", True)
    check_bool("pass budget exhausted -> failure",
               align_verify_verdict(20.0, 6.0, 40.0, 2.0, 3, 3) == "exhausted", True)
    # The hardware case from 2026-07-29 (run 1): yaw_rel converged to +0.6 deg
    # while the real bearing was -26.3 deg. The old code reported SUCCESS; the new
    # one must refuse to conclude and ask for another pass.
    check_bool("real 2026-07-29 case (residual -26.3 deg) is NOT a success",
               align_verify_verdict(-26.3, TOL_BEARING_DEG, None,
                                    ALIGN_VERIFY_MIN_GAIN_DEG, 1,
                                    ALIGN_VERIFY_PASSES) == "retry", True)

    print("\nRESULTAT :", "TOUS LES CAS PASSENT" if ok else "ECHEC")
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(_self_test())
    if not _ROS_AVAILABLE:
        print(f"ROS unavailable in this environment ({_ROS_IMPORT_ERROR}).\n"
              f"Source ROS (source /opt/ros/noetic/setup.bash) to launch the node,\n"
              f"or use --selftest to check the geometry off-robot.")
        sys.exit(1)
    rospy.init_node("beacon_docking")
    BeaconDocking()
    rospy.spin()
