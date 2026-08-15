#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera and beacon tracking -- the project's single SDK connection.

Replaces rm_cam_ros.py + follow_beacon.py, which held one SDK connection each: two
simultaneous connections silently block chassis motion after a few drive_speed calls,
with no error surfaced anywhere. This node is therefore the ONLY process allowed to
call robot.Robot().initialize() while the stack is running.
"""

import os
import sys
import threading
import time
import math
import queue
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(__file__))
from map_collision import MapCollision

import rospy
import tf2_ros
import tf.transformations as tft
from robomaster import robot
from robomaster.battery import BatterySubject as _BatBase
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge


# =========================================================
# CONFIG
# =========================================================

# Publish rate for /camera/color/image_raw. 30 is the S1 sensor's native rate
# (raised 20 -> 30 on 2026-07-22 to hand Carolus fresher frames).
#
# HISTORICAL NOTE, kept because the conclusion changed: this used to warn that
# Carolus's ~2.5 Hz was network TRANSPORT (Pi -> lab PC) and that raising the rate
# would only add bandwidth to an already saturated link. That was right, and F0.C
# happened: Carolus was moved onto the Pi on 2026-08-04 and /pose went from 2.19 Hz
# to 13.04 Hz with the same beacon. The bottleneck is now the Pi's CPU -- this very
# bridge and Carolus compete for it, and the camera itself drops from 30.0 Hz to
# 16.5 Hz once Carolus runs alongside. So lowering this value is once again a lever,
# but for a different reason than it was in July.
TARGET_FPS       = 30
# IMU subscription rate. The SDK admits only 1, 5, 10, 20 and 50 Hz
# (chassis.py:577); any other value is rejected. Exposed as an environment
# variable so those values can be swept without editing this file -- see BUG-089
# (2026-08-04): at 50 Hz the subscription is accepted and, before the fix, no
# callback ever arrived.
IMU_FREQ         = int(os.environ.get("RM_IMU_FREQ", "50"))
# Accelerometer scale applied before publishing on /imu (2026-08-04, BUG-092).
# Measured at rest: linear_acceleration.z reads -1.00664, i.e. the SDK returns
# **g**, while sensor_msgs/Imu mandates m/s^2. So every consumer currently sees
# accelerations 9.81x too small under a contract that says otherwise, and
# neither Kalibr nor MINS can detect it.
#
# Deliberately left at 1.0 (unconverted) until a session with a KNOWN rotation
# settles scale, sign and gyro units TOGETHER: the sign is also suspect (a
# REP-103 z-up frame at rest should read +9.81, not -1.0 g), and fixing the
# scale alone would produce a topic that looks correct while its frame
# convention is still unverified -- strictly worse than one that is visibly
# wrong. Set RM_IMU_ACCEL_SCALE=9.80665 once that session has run.
IMU_ACCEL_SCALE  = float(os.environ.get("RM_IMU_ACCEL_SCALE", "1.0"))
# ---- FRAME CHAIN CONSTANTS (2026-08-04) ---------------------------------
# All four default to "no change from the pre-2026-08-04 behaviour", so the
# chain restructure below is a pure refactor until these are measured.
#
# SIGNS. DJI documents its world frame as NED -- Z pointing DOWN -- while ROS
# REP-103 has Z UP. A rotation about a down axis is the opposite sense of one
# about an up axis, so yaw_DJI = -yaw_ROS is the expected relationship and
# these should very likely end up at -1. They are NOT set to -1 today because
# that is a reasoned expectation, not a measurement, and BUG-077 is what
# happens when a sign is assumed. Protocol 22, step B1 measures them.
GIMBAL_YAW_SIGN_TF   = float(os.environ.get("RM_GIMBAL_YAW_SIGN_TF", "1.0"))
GIMBAL_PITCH_SIGN_TF = float(os.environ.get("RM_GIMBAL_PITCH_SIGN_TF", "1.0"))
# TRANSLATIONS, metres, in the parent frame's axes.
# BASE_TO_GIMBAL_XYZ: chassis rotation centre -> gimbal yaw axis.
# GIMBAL_TO_CAM_XYZ : gimbal yaw/pitch axis intersection -> optical centre.
# Their sum is the camera lever arm, independently estimated twice already
# (0.165 m measured 2026-08-04, 0.123 m derived 2026-07-31). Split at the
# gimbal yaw axis, which is physically identifiable on the robot. Protocol 22
# step B2 measures them; until then both are zero and the composed transform
# is identical to the old single one.
BASE_TO_GIMBAL_XYZ = (0.0, 0.0, 0.0)
GIMBAL_TO_CAM_XYZ  = (0.0, 0.0, 0.0)

# Beacon duty-cycle report period, seconds. See _beacon_status_tick.
DUTY_REPORT_PERIOD_S = 30.0
# Number of poses averaged before each [LATENCY] report. See _pose_cb.
LATENCY_REPORT_EVERY = 50
STOP_DISTANCE_M  = 0.70
POSE_TIMEOUT_S   = 1.5

MAX_VX  = 0.20
MAX_WZ  = 10.0
MIN_VX  = 0.06
K_VX    = 0.6

# -- SEARCH: gimbal sweep (robot stationary), then a chassis step on failure ----
# The gimbal is inertially stabilised: yaw_ground stays constant even while the
# chassis turns underneath it. The sweep is driven by reading yaw_rel (the gimbal
# angle relative to the chassis); the chassis does NOT move during it. If a full
# sweep of the range finds nothing -> recenter (gimbal aligned with the body),
# advance 1 m in a straight line, and start over.
SEARCH_GIMBAL_YAW_S = 20.0    # gimbal drive_speed setpoint (~20 deg/s measured, direct sense)
SEARCH_YAW_LIMIT    = 200.0   # max sweep amplitude in yaw_rel (deg, under the 250 deg mechanical limit)
SEARCH_STEP_M       = 1.0     # chassis step after a full sweep with no detection (m)
SEARCH_STEP_VX      = 0.15    # speed of that step (m/s)
SEARCH_MAX_STEPS    = 3       # max steps before stopping (protects the Ethernet cable)
SEARCH_GRACE_S      = 2.0     # initial pause before the sweep starts
RECENTER_TIMEOUT_S  = 6.0     # wait_for_completed timeout on the recenter action

# -- Fan pattern: axis exhausted (SEARCH_MAX_STEPS reached) -> turn and restart --
# Parameters from a 2026-07-01 coverage-strategy research round: 60 deg increment,
# 6 axes (two passes of three), global bound of 90 s or ~13.5 m travelled.
SEARCH_FAN_AXIS_COUNT    = 6      # total axes in the fan pattern (two passes of three)
SEARCH_FAN_AXIS_INC_DEG  = 60.0   # rotation increment between two axes (deg)
SEARCH_FAN_ROT_WZ        = 30.0   # chassis rotation speed between axes (deg/s)
SEARCH_FAN_ROT_TIMEOUT_S = 8.0    # safety: 60 deg at 30 deg/s is ~2 s, so 4x margin
SEARCH_TOTAL_TIMEOUT_S   = 90.0   # autonomous-search bound before giving up (s)
SEARCH_TOTAL_DIST_M      = 13.5   # autonomous-search bound before giving up (m travelled, middle of the 12-15 m range)

# -- APPROACH: gimbal locked on the beacon, chassis follows the gimbal angle ----
K_GIM_YAW   = 2.0   # deg/s of gimbal servo per degree of lateral beacon error
GIM_YAW_MAX = 50.0  # max gimbal servo speed (deg/s)
K_BODY_YAW  = 0.8   # chassis rotation gain used to null yaw_rel (body alignment)
GIM_YAW_SIGN = -1   # gimbal servo sign: -1 confirmed by the 2026-06-26 test (drive_speed(positive) = left, yaw_err_deg<0 = left, so a negative sign converges)

# -- PITCH servo (vertical centring). Used ONLY by the MANUAL-mode LOCK auto-track
# (2026-07-22), never by APPROACH, whose tested behaviour was left untouched. Keeps
# the beacon's four LEDs in frame -> more stable detection at an angle.
#
# DISABLED on 2026-07-22 after a HARDWARE INCIDENT (BUG-058): the pitch sign had
# never been confirmed, so on the first LOCK click the gimbal pitched to its
# mechanical stop, snagged the RNDIS cable, overloaded the motor and turned the LED
# red. Pitch tracking stays DISABLED until (a) the sign is verified by hand with the
# gimbal free, AND (b) angle limits are added so no runaway can reach a cable.
# Set GIM_PITCH_TRACK_ENABLED=True only once both hold.
GIM_PITCH_TRACK_ENABLED = False
GIM_PITCH_SPEED_MAX = 30.0   # max pitch servo speed (deg/s, deliberately conservative)
GIM_PITCH_SIGN      = -1     # WARNING: sign NOT confirmed (see the BUG-058 incident)

# -- BEACON LOCK: periodic re-centring by relative motion (2026-07-23) ---------
# History: a LOCK v1 existed (continuous P servo at 20 Hz, with gating, ramping and
# outlier rejection) and was removed on 2026-07-23, judged redundant and less
# practical than the periodic re-centring below. Only this mechanism remains: every
# GIM_LOCK_PERIOD_S seconds (live-configurable through /carolus/gimbal_lock_period)
# we read the beacon-to-image-centre angular error and send ONE relative position
# command, gimbal.move(yaw=...), that brings it back to centre -- independently of
# any chassis motion. gimbal.move() was characterised as ASYNC in
# test_gimbal_sweep.py: it returns immediately.
GIM_LOCK_PERIOD_S_DEFAULT = 3.0   # default period (s); also the fallback if an invalid value arrives
# Speed at the SDK ceiling (2026-07-23, explicit user request for maximum speed).
# The SDK documents move(yaw_speed=...) over [0, 540] deg/s. Never tested above
# 80 deg/s on THIS robot before this change (test_gimbal_sweep.py), but the BUG-058
# risk (mechanical stop plus a snagged cable) is judged low here: these are one-shot
# movements of SMALL bounded angle (see the deadband and max_err below), not a large
# sweep toward a stop -- a higher speed only shortens the movement's duration, not
# its angular amplitude.
GIM_LOCK_YAW_SPEED   = 540.0
GIM_LOCK_MAX_ERR_DEG = 45.0   # beyond this the pose is probably aberrant -> skip this tick
# Deadband (2026-07-23): below this threshold, do not re-correct -- avoids chasing a
# residual of P4P measurement noise and mechanical imprecision that will not go down
# any further. 5.0 deg confirmed on hardware as the right setting (a reduction to
# 3.0 deg was tried and reverted the same evening: 5.0 was already the behaviour
# judged correct).
GIM_LOCK_DEADBAND_DEG = 5.0
# RECENTER CAM (night of 2026-07-23): gimbal.recenter() is an ASYNC action that can
# take up to ~0.7 s (a large angle at 360 deg/s). The MANUAL loop re-sends
# drive_speed(0,0) at 20 Hz, which CANCELS the action on the next tick (~50 ms), so
# the recenter never completed. Fix: a "gimbal busy" window during which the MANUAL
# loop and the LOCK tick suspend their commands and let the action finish. This is
# also why LOCK worked without such a window while recenter did not -- LOCK's
# small-angle move() completes in under 20 ms.
GIMBAL_RECENTER_BUSY_S = 2.5
# Pitch NOT included by default: pitch tracking has been disabled since the BUG-058
# incident (gimbal at its mechanical stop, cable snagged). Reuses the
# GIM_PITCH_TRACK_ENABLED guard -- do not enable without (a) verifying
# GIM_PITCH_SIGN and (b) adding angle limits.

# -- ALIGN: pre-orient the chassis before APPROACH -----------------------------
# From the visual-servoing literature (the look-and-move pattern): orient first,
# approach second. The gimbal servo is OFF during ALIGN -- its world stabilisation
# is enough.
ALIGN_YAW_THRESHOLD = 12.0   # |yaw_rel| < X deg -> count the pose as valid
ALIGN_YAW_DEADBAND  = 2.0    # |yaw_rel| < X deg -> wz = 0 (prevents chattering)
ALIGN_MAX_WZ        = 8.0    # max deg/s during ALIGN (gentler than APPROACH)
ALIGN_TIMEOUT_ALPHA = 1.5    # T_max = alpha * |yaw_rel_init| / ALIGN_MAX_WZ
ALIGN_TIMEOUT_MAX_S = 12.0   # cap on the dynamic timeout (s)
ALIGN_VALID_POSES   = 3      # consecutive valid poses required before APPROACH

MANUAL_CMDVEL_TIMEOUT = 0.5   # auto-stop if no command received for X seconds
# BUG-100 (2026-08-10): the gimbal had NO equivalent. `_gim_stamp` was written on
# every /carolus/gimbal_vel message and never read, so the last commanded gimbal
# speed was re-sent at 20 Hz forever -- a lost KeyRelease (X11 focus change, GUI
# stall, operator window switch) left the gimbal turning with nothing to stop it.
# The chassis has been protected this way since "Bug 4"; the gimbal simply was
# not. Same value, same reasoning.
MANUAL_GIMBAL_TIMEOUT = 0.5

# ---- protocol 22 / B4 test flags (2026-08-10) --------------------------------
# BUG-093: the chassis rotates ~0.12-0.38 deg/s with an SDK session open and is
# still without one. Two hypotheses fit every observation equally well, and no
# test run so far can separate them:
#   HA  gimbal reaction torque -- the gimbal fights its own drifting gyro and
#       the yaw motor's reaction turns the chassis, which Mecanum wheels at zero
#       commanded velocity do not resist.
#   HB  our own chassis velocity loop -- a session means drive_speed(0,0,0) at
#       20 Hz, i.e. a closed loop that simply does not exist without a session;
#       its deadband or integral behaviour could be the whole effect.
# Three conditions, one flag each, so B4 is three launches rather than three
# hand edits. Both default OFF: normal operation is unchanged.
#   1. neither flag           -> HA drifts, HB drifts   (today's state)
#   2. RM_B4_SUSPEND_GIMBAL=1 -> HA STOPS, HB drifts
#   3. RM_B4_NO_DRIVE=1       -> HA drifts, HB STOPS
B4_SUSPEND_GIMBAL = os.environ.get("RM_B4_SUSPEND_GIMBAL", "0") == "1"
# Gimbal torque, 2026-08-14. DEFAULT OFF -- see the long comment at the call
# site for the measurements. "off" recentres then cuts torque: the gimbal stops
# drifting (6.35 -> 0.30 deg/min) but can no longer be aimed. "on" restores the
# old aimable-but-drifting behaviour.
GIMBAL_TORQUE_ON = os.environ.get("RM_GIMBAL_TORQUE", "off").strip().lower() == "on"
# How long to wait for the async recenter to finish before cutting torque.
# gimbal.recenter() is characterised at ~0.7 s for a large angle at 360 deg/s
# (test_gimbal_sweep.py); 2.5 s is the same margin GIMBAL_RECENTER_BUSY_S uses.
GIMBAL_RECENTER_SETTLE_S = 2.5
B4_NO_DRIVE       = os.environ.get("RM_B4_NO_DRIVE", "0") == "1"

# -- Obstacle avoidance --------------------------------------------------------
OBSTACLE_TOF_CM     = 50    # front TOF distance (cm) -> emergency stop
LOOKAHEAD_M         = 0.60  # collision look-ahead distance in front of the robot (m)
MAP_JSON_PATH       = "/home/ubuntu/carolus_map.json"
MAP_INFLATION_CELLS = 0     # no inflation -- obstacles are drawn block-exact on the map

GIMBAL_PITCH_MAX = 55.0   # deg/s (approximate mechanical limit)
GIMBAL_YAW_MAX   = 90.0

# -- /odom covariance (2026-07-22) --------------------------------------------
# The diagonal must be non-zero: an EKF (robot_localization, roadmap item F3)
# rejects an all-zero covariance, reading it as "perfect certainty", which breaks
# the filter. ROS flattened 6x6 order: [x, y, z, roll, pitch, yaw]. This is a 2D
# robot, so z/roll/pitch are pinned at a large variance (_ODOM_BIG). The values are
# indicative -- wheel odometry is decent short-term with slow drift -- and are to be
# refined in F3 if needed. Kept as module constants so no list is rebuilt per tick.
_ODOM_BIG = 1e6
_ODOM_POSE_COV = [
    0.02, 0.0,  0.0,       0.0,       0.0,       0.0,
    0.0,  0.02, 0.0,       0.0,       0.0,       0.0,
    0.0,  0.0,  _ODOM_BIG, 0.0,       0.0,       0.0,
    0.0,  0.0,  0.0,       _ODOM_BIG, 0.0,       0.0,
    0.0,  0.0,  0.0,       0.0,       _ODOM_BIG, 0.0,
    0.0,  0.0,  0.0,       0.0,       0.0,       0.05,
]
_ODOM_TWIST_COV = [
    0.02, 0.0,  0.0,       0.0,       0.0,       0.0,
    0.0,  0.02, 0.0,       0.0,       0.0,       0.0,
    0.0,  0.0,  _ODOM_BIG, 0.0,       0.0,       0.0,
    0.0,  0.0,  0.0,       _ODOM_BIG, 0.0,       0.0,
    0.0,  0.0,  0.0,       0.0,       _ODOM_BIG, 0.0,
    0.0,  0.0,  0.0,       0.0,       0.0,       0.05,
]


# =========================================================
# Helpers
# =========================================================

def clamp(v, vmin, vmax):
    return max(vmin, min(vmax, v))


def _angle_diff_deg(a, b):
    """Smallest signed angular difference a-b, in degrees, wrapped to [-180, 180]."""
    return ((a - b + 180.0) % 360.0) - 180.0


class _FullBatSubject(_BatBase):
    """Expose the temperature, current and ADC voltage that BatterySubject.data_info() hides."""
    def data_info(self):
        # struct '<HhiBB' : adc_value(uint16), temperature(int16 en 0.1°C), current(int32 en mA), percent(uint8)
        temp_c = self._temperature / 10.0
        return (self._percent, temp_c, self._current, self._adc_value)


@dataclass
class PoseData:
    x:     float
    y:     float
    z:     float
    stamp: float
    yaw:   float = 0.0   # beacon yaw in degrees (camera frame; orientation estimate)


class PoseBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._pose = None

    def update(self, x, y, z, yaw=0.0):
        with self._lock:
            self._pose = PoseData(x=float(x), y=float(y), z=float(z),
                                   stamp=time.time(), yaw=float(yaw))

    def get(self):
        with self._lock:
            return self._pose

    def invalidate(self):
        """Clear the buffer on every state transition, so stale poses are rejected."""
        with self._lock:
            self._pose = None


# =========================================================
# Node principal
# =========================================================

class EPCameraBeaconFollower:

    def __init__(self):
        self.bridge   = CvBridge()
        self.posebuf  = PoseBuffer()
        self.running  = True

        self.search_dir    = 1       # sens du sweep gimbal (+1 / -1)
        self.search_edges  = 0       # nb de bords atteints (2 = cycle complet -> avancer)
        self.search_steps  = 0       # nb d'avancements effectues sans detection sur l'axe courant
        self._search_since = time.time()

        # Fan pattern (axis exhausted -> chassis rotation -> next axis)
        self.search_axis_idx       = 0     # index de l'axe courant (0..SEARCH_FAN_AXIS_COUNT-1)
        self._search_dist_total_m  = 0.0   # distance cumulee avancee sur l'episode de recherche courant
        self._search_episode_since = time.time()   # horodatage debut d'episode (reset a l'entree fraiche AUTO/LOCATE)

        # Etat ALIGN
        self._align_since    = 0.0
        self._align_yaw_init = 0.0
        self._align_valid_n  = 0     # poses consecutives valides pendant ALIGN

        # Telemetrie consolidee (ESC, attitude, position, batterie, vitesse, statut, TOF)
        # One lock for seven streams: less contention and overhead than seven separate locks.
        self._telem_lock = threading.Lock()
        self._telem = {
            'esc':    [0, 0, 0, 0],
            'atti':   (0.0, 0.0, 0.0),   # yaw, pitch, roll (deg)
            'pos':    (0.0, 0.0),          # x, y odometrie (m)
            'bat':    (0, 0.0, 0, 0),      # pct, temp_c, cur_ma, adc
            'vel':    (0.0, 0.0),          # vbx, vby (m/s)
            'status': {},
            'dist':   999.0,               # TOF frontal (cm)
        }

        # Carte d'obstacles (chargée si MAP_JSON_PATH existe)
        self._colmap = MapCollision(MAP_JSON_PATH, inflation_cells=MAP_INFLATION_CELLS)

        # Commande roues individuelles (mode MANUEL seulement)
        self._wheels_active = False
        self._wheels_vals   = (0, 0, 0, 0)
        self._wheels_lock   = threading.Lock()

        # Angle gimbal lu via sub_angle : (pitch, yaw_rel, pitch_ground, yaw_ground)
        # yaw_rel    = gimbal angle relative to the chassis (APPROACH's alignment signal)
        # yaw_ground = the gimbal's absolute heading (world frame, inertially stabilised)
        self._gim_yaw_rel = 0.0
        self._gim_yaw_ground = 0.0
        self._gim_angle_lock = threading.Lock()

        # Mode : "AUTO" (recherche/approche) ou "MANUAL" (commandes ZQSD du GUI).
        # DEFAULT = MANUAL since 2026-07-22 (BUG-058 safety): starting in AUTO
        # made the gimbal sweep (SEARCH) as soon as the node started, risking a snagged
        # cable. In MANUAL the robot stays still until the user commands
        # rien. Le passage en AUTO reste explicite (bouton MODE du launcher).
        self._mode          = "MANUAL"
        self._mode_lock     = threading.Lock()
        self._man_vx        = 0.0
        self._man_wz        = 0.0
        self._man_stamp     = 0.0   # timestamp derniere commande MANUEL recue
        self._idle_braked   = False # BUG-093: has the one-shot idle brake been sent?
        self._gim_idle_braked = False # BUG-106: same, for the gimbal (see the control loop)
        self._man_lock      = threading.Lock()
        # Gimbal manuel
        self._gim_pitch = 0.0
        self._gim_yaw   = 0.0
        self._gim_stamp = 0.0
        self._gim_lock  = threading.Lock()
        # BEACON LOCK (2026-07-23): periodic re-centring, OFF by default. The v1
        # historical version (continuous 20 Hz servo) was removed the same day as redundant.
        self._gimbal_lock = False
        self._gimbal_lock_period_s = GIM_LOCK_PERIOD_S_DEFAULT   # live-configurable via /carolus/gimbal_lock_period
        self._lock_timer = None   # current rospy.Timer, recreated when the period changes
        # "Gimbal busy" window (night of 2026-07-23): while time.time() is below this
        # value an async gimbal action (recenter) is in flight, so the MANUAL loop and
        # the LOCK tick suspend their commands rather than cancelling it.
        self._gimbal_busy_until = 0.0

        self._beacon_was_fresh = False   # detects LOST->DETECTED transition (indicator/minimap)
        # Beacon duty cycle and /pose latency, accumulated continuously
        # (2026-08-04). Both were "the number nobody had"; they are now produced
        # by every run instead of by a dedicated session.
        self._duty_seen  = 0
        self._duty_total = 0
        self._duty_since = time.time()
        self._lat_sum = 0.0
        self._lat_n   = 0

        # Connexion unique au robot
        rospy.loginfo("[RM] Connecting over RNDIS...")
        self.ep = robot.Robot()
        self.ep.initialize(conn_type="rndis")
        self.cam     = self.ep.camera
        self.chassis = self.ep.chassis
        self.gimbal  = self.ep.gimbal
        rospy.loginfo("[RM] Robot connected")

        # ROBOT MODE (2026-08-04) -- instrumentation added after observing
        # the chassis rotating on its own at ~0.38 deg/s while drive_speed(x=0,y=0,z=0)
        # was being sent at 20 Hz. The official protocol documentation
        # (robomaster-dev.readthedocs.io, text_sdk/protocol_api.html, extracted verbatim
        # on 2026-08-04) states that in gimbal_lead mode ("chassis follows gimbal") the
        # chassis "does not respond to the yaw axis control part of all control
        # commands", explicitly including chassis movement speed control -- so our z
        # would simply be ignored. In free mode it must not be.
        #
        # initialize() calls reset(), which already requests free (robot.py:1179), BUT
        # on this rooted S1 a call that does not raise proves nothing (see BUG-091, same
        # day: sub_imu returned True while no data ever arrived). So we READ the actual
        # mode instead of assuming it, force it explicitly, and read it back to verify
        # the request was honoured.
        # 2026-08-14 (BUG-113): the mode is now selectable, defaulting to
        # CHASSIS_LEAD rather than FREE.
        #
        # WHY. BUG-104 established that this robot's attitude estimate integrates a
        # constant misaligned gyro bias with no gravity correction, rotating about a
        # fixed axis tilted ~11 deg from the body vertical at +0.0918 deg/s
        # (R^2=0.9997). In FREE mode "the gimbal and the chassis move without
        # affecting each other" (DJI's own wording), i.e. the gimbal holds an
        # INERTIAL heading -- against that drifting reference. So it physically
        # slews at the same rate: measured -34 deg -> -229 deg unattended, closing
        # on the ~250 deg mechanical limit, and the beacon left the frame after
        # roughly four minutes every time.
        #
        # In CHASSIS_LEAD "the gimbal follows the chassis to rotate along the yaw
        # axis" -- the yaw setpoint becomes chassis-referenced, so the drifting
        # attitude estimate is no longer in the loop at all. The camera test of
        # 2026-08-13 proved the chassis does not physically rotate (0.4 px measured
        # against 227 px expected, 567x), so a chassis-referenced gimbal has nothing
        # left to chase.
        #
        # TRADE-OFF, stated because it is real: in CHASSIS_LEAD the gimbal no longer
        # stabilises against genuine chassis rotation, so while DRIVING it will swing
        # with the robot. That is the correct trade for a stationary measurement
        # (1.4b, calibration) and the wrong one for driving with a locked heading.
        # Set RM_ROBOT_MODE=free to restore the old behaviour.
        #
        # *** CHASSIS_LEAD BREAKS DOCKING -- USE free FOR T5. ***
        # beacon_docking.py aligns the chassis by driving `yaw_rel` to zero
        # (chassis_align_tick). In CHASSIS_LEAD the gimbal follows the chassis, so
        # yaw_rel is CONSTANT BY DEFINITION and rotating the chassis cannot change
        # it -- the control law has no observable to servo on and can never
        # converge. Its no-progress guard turns that into a visible abort rather
        # than an endless spin, but docking simply does not work in this mode.
        # Relaunch the camera node with RM_ROBOT_MODE=free before using T5.
        _mode_env = os.environ.get("RM_ROBOT_MODE", "chassis_lead").strip().lower()
        _mode_map = {"free": robot.FREE,
                     "chassis_lead": robot.CHASSIS_LEAD,
                     "gimbal_lead": robot.GIMBAL_LEAD}
        _mode_want = _mode_map.get(_mode_env, robot.CHASSIS_LEAD)
        try:
            _mode_before = self.ep.get_robot_mode()
            rospy.loginfo(f"[MODE] mode read at connection: {_mode_before!r}")
            _set_ok = self.ep.set_robot_mode(mode=_mode_want)
            _mode_after = self.ep.get_robot_mode()
            rospy.loginfo(f"[MODE] set_robot_mode({_mode_want!r}) returned {_set_ok!r} "
                          f"-> mode read back: {_mode_after!r}")
            if _mode_after != _mode_want:
                rospy.logwarn(f"[MODE] requested {_mode_want!r} but the robot reports "
                              f"{_mode_after!r} -- the request was NOT honoured")
            elif _mode_want == robot.GIMBAL_LEAD:
                rospy.logwarn("[MODE] gimbal_lead: the chassis IGNORES the z component "
                              "of drive_speed, which looks like an uncommanded rotation")
        except Exception as e:
            rospy.logwarn(f"[MODE] could not read/write the robot mode: {e}")
            _mode_after = "unknown"
        # Remember what the robot ACTUALLY reported, not what we asked for -- this
        # rooted S1 accepts calls it does not implement (BUG-089), so the read-back
        # is the only trustworthy value. Published once the publishers exist.
        self._robot_mode = _mode_after if isinstance(_mode_after, str) else "unknown"

        # ---- GIMBAL TORQUE: the fix for the drift, measured 2026-08-14 -------
        #
        # THE DEFAULT IS NOW TORQUE OFF, and that is a deliberate reversal.
        #
        # What was happening: with torque active the gimbal's own stabilisation
        # loop holds a constant heading in the POWER-ON frame (`yaw_ground`).
        # That frame is derived from the chassis attitude estimate, which drifts
        # (BUG-104). Holding a constant heading against a drifting reference
        # means the firmware physically rotates the gimbal, forever. That
        # rotation is the "drift" -- the camera really does turn, and it walks
        # off the beacon within minutes.
        #
        # Measured on this robot, stationary, T1+T2 only, 40 s windows:
        #
        #                       torque ON        torque OFF
        #     yaw_rel           -6.35 deg/min    -0.30 deg/min   (21x better)
        #     yaw_ground        +0.00 (held)     +7.81 deg/min   (the raw bias)
        #
        # With torque off, `yaw_rel` spans 0.2 deg across 381 samples -- the
        # sensor's own quantisation step. The gimbal is genuinely motionless.
        # `yaw_ground` then shows BUG-104's bias undisguised, which is the same
        # defect measured directly for the first time with the servo out of the
        # loop.
        #
        # THE ORDER MATTERS. Suspending torque lets the gimbal fall to its
        # mechanical rest position: measured, it dropped 17 deg (yaw_rel -21 ->
        # -38) and lost the beacon entirely. So recenter FIRST, let the async
        # recenter finish, and only then cut torque -- the gimbal ends up
        # centred AND motionless.
        #
        # COST, stated plainly: with torque off the gimbal cannot be aimed. The
        # numpad gimbal keys and LOCK have no effect. That is the right trade
        # for a stationary measurement (the internship's 1.4b needs a camera
        # that does not move for ~30 min) and the wrong one for tracking a
        # moving target. Set RM_GIMBAL_TORQUE=on to get the old behaviour back,
        # drift included.
        if GIMBAL_TORQUE_ON:
            try:
                self.gimbal.resume()
                rospy.logwarn("[GIMBAL] RM_GIMBAL_TORQUE=on -- torque ACTIVE. The gimbal "
                              "can be aimed, and it WILL drift ~6 deg/min against the "
                              "attitude bias (BUG-104). Use the default (off) for any "
                              "stationary measurement.")
            except Exception as e:
                rospy.logwarn(f"[GIMBAL] resume failed: {e}")
        else:
            try:
                self.gimbal.resume()          # torque must be on to move at all
                self.gimbal.recenter(pitch_speed=360.0, yaw_speed=360.0)
                time.sleep(GIMBAL_RECENTER_SETTLE_S)
                self.gimbal.suspend()
                rospy.loginfo("[GIMBAL] recentred, then torque CUT (default) -- the gimbal "
                              "is now motionless and will not drift. It also cannot be "
                              "aimed: numpad and LOCK are inert. RM_GIMBAL_TORQUE=on to "
                              "restore aiming.")
            except Exception as e:
                rospy.logwarn(f"[GIMBAL] recenter+suspend failed: {e}")
        if B4_NO_DRIVE:
            rospy.logwarn("[B4] RM_B4_NO_DRIVE=1 -- the 20 Hz chassis command loop is "
                          "SUPPRESSED. The robot cannot be driven. Test mode.")

        # Gimbal publishers created BEFORE sub_angle (BUG-064bis, 2026-07-23): the SDK
        # can invoke _gimbal_angle_cb the moment the callback is registered, before the
        # rest of __init__ has run -- an AttributeError observed in a real session on
        # pub_gimbal_yaw back when the Publisher was created further down (line
        # ~399 auparavant).
        self.pub_gimbal_yaw = rospy.Publisher("/carolus/gimbal_yaw_rel", Float32, queue_size=10)
        self.pub_gimbal_yaw_ground = rospy.Publisher("/carolus/gimbal_yaw_ground", Float32, queue_size=10)
        # BUG-089 (2026-08-04): the SAME defect as BUG-064bis, reintroduced on
        # 2026-07-30 by adding pub_imu further down. The IMU callback raised
        # AttributeError ('pub_imu' did not exist yet) inside the DDS dispatcher thread,
        # that thread died, and NO further IMU callback ever ran -- hence a /imu topic
        # that was advertised but silent, with nothing visible on the ROS side. The
        # BUG-064bis lesson had only been applied to the gimbal: EVERY publisher touched
        # by an SDK callback is created here, above the subscriptions, never below.
        # Do not move these down.
        self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.pub_imu  = rospy.Publisher("/imu", Imu, queue_size=50)
        self.pub_img  = rospy.Publisher("/camera/color/image_raw", Image, queue_size=1)
        # 2026-08-14 (BUG-111): publish the robot mode, LATCHED, so other nodes can
        # check it instead of guessing. beacon_docking.py needs this: its chassis
        # alignment servos on `yaw_rel`, which cannot change in chassis_lead (the
        # gimbal follows the chassis), so docking silently has no observable to
        # converge on. Latched because a consumer starting later must still see it.
        self.pub_robot_mode = rospy.Publisher("/carolus/robot_mode", String,
                                              queue_size=1, latch=True)
        self.pub_robot_mode.publish(String(data=getattr(self, "_robot_mode", "unknown")))
        # Dynamic base_link->camera_link TF (BUG-067, 2026-07-23, F3): the gimbal
        # really does move (yaw_rel/pitch_rel are non-zero as soon as the gimbal is off
        # centre) -- a static identity TF (testcarolus.launch) made
        # beacon_absolute_pose.py's absolute-pose computation wrong the moment it turned
        # (z aberrant observe en test reel : 1.68m au lieu de ~0m). Publiee ici, pas
        # in carolus_tf_broadcaster.py, because pitch_rel/yaw_rel are already in memory
        # in this process (no intermediate topic needed).
        self.tf_br_gimbal = tf2_ros.TransformBroadcaster()

        # Real-time gimbal-angle tracking. REVERTED to 10 Hz on 2026-07-22: raising it
        # to 20 Hz had made sub_angle go silent (gimbal_yaw_rel stopped
        # publishing, blind angle feedback) -- 20 is probably not a valid frequency for
        # this subject, or the DDS link saturates. 10 Hz is the original, known-good
        # value. 2026-08-04: that episode is retrospectively explained by the same defect
        # as BUG-089 -- add_subject_info returns False on a refused frequency without
        # raising, and the "OK" logged here never looked at that return. Now routed
        # through self._sub, so a refusal is visible immediately instead of showing up
        # as silence.
        self._sub("GIMBAL", self.gimbal.sub_angle, freq=10,
                  callback=self._gimbal_angle_cb)

        # Télémétrie batterie etendue (percent + temp + courant + tension ADC)
        try:
            self.battery = self.ep.battery
            _bat_subj = _FullBatSubject()
            _bat_subj.freq = 1
            # add_subject_info returns a bool, same reason as above (BUG-089).
            _bat_ok = self.ep.dds.add_subject_info(_bat_subj, self._battery_cb, (), {})
            if _bat_ok is False:
                rospy.logwarn("[BAT] Extended battery subscription REFUSED (returned False)")
            else:
                rospy.loginfo("[BAT] Extended battery subscription OK (temp + current + voltage)")
        except Exception as e:
            rospy.logwarn(f"[BAT] unavailable: {e}")

        # SDK telemetry -- best effort, routed through self._sub, which checks the
        # boolean return instead of settling for "nothing raised" (BUG-089).
        self._sub("ESC", self.chassis.sub_esc, freq=5, callback=self._esc_cb)

        # REVERTED to 5 Hz on 2026-07-22 (the original value): back to a known-good
        # state after the 20 Hz bumps coincided with sub_angle going silent.
        self._sub("ATTI", self.chassis.sub_attitude, freq=5, callback=self._atti_cb)
        self._sub("POS", self.chassis.sub_position, cs=0, freq=5, callback=self._pos_cb)
        self._sub("VEL", self.chassis.sub_velocity, freq=5, callback=self._vel_cb)
        self._sub("STATUS", self.chassis.sub_status, freq=5, callback=self._status_cb)

        # IMU brute (accelero + gyro), 2026-07-30 (prerequis calibration MINS,
        # cf. research-log/02-protocoles/protocoles-terrain, protocole Kalibr).
        # BUG-089 (2026-08-04): on THIS robot the subscription was accepted but the
        # callback was NEVER called -- 0 /imu messages in 40 s while /odom ran at
        # 16.5 Hz over the same SDK connection. Root cause turned out to be ours (the
        # publisher did not exist yet when the first callback fired, killing the DDS
        # dispatcher thread), and it is fixed. IMU_FREQ is still exposed here so the
        # SDK's other admissible rates (1, 5, 10, 20, 50 -- chassis.py:577) can be
        # swept without reopening the file: only 50 has ever been tried, taken as-is
        # from a community ROS2 example (s1_sdk_hack_v0.0.5/.../ros2_robot.py:42) and
        # never validated on our hardware. Override with RM_IMU_FREQ.
        self._sub("IMU", self.chassis.sub_imu, freq=IMU_FREQ, callback=self._imu_cb)

        self._sub("TOF", self.ep.sensor.sub_distance, freq=10, callback=self._dist_cb)

        # Carte d'obstacles
        if self._colmap.loaded:
            rospy.loginfo(f"[MAP] Obstacle map loaded from {MAP_JSON_PATH}")
        else:
            rospy.logwarn(f"[MAP] {MAP_JSON_PATH} missing -- collision map disabled")

        self.cam.start_video_stream(display=False)
        rospy.loginfo("[RM] Camera stream started")

        # Subscribers ROS. Les publishers touches par un callback SDK (pub_img,
        # pub_odom, pub_imu, pub_gimbal_*) are ALL created above, before the SDK
        # subscriptions -- see BUG-064bis and BUG-089. Do not recreate any here.
        # Note /odom : position convertie en REP-103 (y_ros=-y_ep, cf. Perplexity 07 /
        # GitHub SDK officiel : EP y+=droite, ROS y+=gauche). Yaw publie tel quel
        # (the EP sign is unconfirmed for an absolute orientation -- adequate for a
        # mesure de derive relative comme ici ; a verifier avant reutilisation dans
        # l'EKF robot_localization de F3).
        rospy.Subscriber("/pose",               PoseStamped, self._pose_cb)
        rospy.Subscriber("/carolus/mode",       String,      self._mode_cb)
        rospy.Subscriber("/carolus/cmd_vel",    Twist,       self._cmdvel_cb)
        rospy.Subscriber("/carolus/gimbal_vel", Twist,       self._gimbal_cb)
        rospy.Subscriber("/carolus/wheels",     String,      self._wheels_cb)
        rospy.Subscriber("/carolus/gimbal_lock", String,     self._gimbal_lock_cb)
        rospy.Subscriber("/carolus/gimbal_lock_period", String, self._gimbal_lock_period_cb)
        rospy.Subscriber("/carolus/gimbal_recenter", String, self._gimbal_recenter_cb)
        rospy.loginfo("[ROS] Subscriptions actives (/pose, /carolus/mode, /carolus/cmd_vel, /carolus/gimbal_vel, /carolus/wheels, /carolus/gimbal_lock, /carolus/gimbal_lock_period, /carolus/gimbal_recenter)")

        # BEACON LOCK (2026-07-23): a timer independent of the 20 Hz main loop --
        # periodic re-centring, not a continuous servo. Held on self._lock_timer so it
        # can be recreated with a new period (see _gimbal_lock_period_cb).
        self._lock_timer = rospy.Timer(rospy.Duration(self._gimbal_lock_period_s), self._gimbal_lock_tick)
        # Beacon DETECTED/LOST status (2026-07-23): a periodic log parsed by
        # carolus_launcher.py (same pattern as [BAT]/[ATTI]/[POS]) to drive the
        # indicator and the minimap. 5 Hz, independent of the LOCK ON/OFF state.
        rospy.Timer(rospy.Duration(0.2), self._beacon_status_tick)

        self.rate = rospy.Rate(TARGET_FPS)

    # ---- souscriptions telemetrie SDK ----

    def _sub(self, tag, fn, **kw):
        """Subscribe to an SDK telemetry channel and log the REAL result.

        The SDK's sub_* methods return a bool (from dds.add_subject_info): a
        subscription the robot refuses returns False WITHOUT raising. The original
        code (2026-07-30) discarded that return and logged "OK" as soon as nothing
        had raised -- which is why BUG-089 (sub_imu reporting OK while /imu never
        published anything) looked like a success for a whole session. An "OK" must
        only mean OK if the robot said so.
        """
        try:
            ok = fn(**kw)
        except Exception as e:
            rospy.logwarn(f"[{tag}] {fn.__name__} exception: {e}")
            return False
        if ok is False:
            # Not an exception: the robot explicitly refused. Distinct from a crash,
            # and distinct from an "OK" -- see BUG-089.
            rospy.logwarn(f"[{tag}] {fn.__name__} REFUSED by the robot "
                          f"(returned False) -- no data will ever arrive")
            return False
        rospy.loginfo(f"[{tag}] {fn.__name__} OK (returned {ok!r})")
        return True

    # ---- callbacks ROS ----

    def _pose_cb(self, msg):
        # LATENCY, not rate (2026-08-04). We measured Carolus's pose RATE
        # (13.04 Hz on the Pi) and quoted it to the supervisor, but for drift
        # correction the metric that decides quality is LATENCY: a correction
        # arriving at 13 Hz but 200 ms late is worse than one at 5 Hz arriving
        # in 30 ms, because the filter has to roll its state back that far.
        #
        # This is only meaningful because carolus_astrobee now stamps the pose
        # with the image's ACQUISITION time (stamp_from_acquisition, same day).
        # Before that the stamp was ros::Time::now() at publication, so this
        # subtraction would have measured nothing but transport delay -- and,
        # worse, every fusion consumer was being told the pose described the
        # present when it described an image from N ms earlier.
        try:
            lat_ms = (rospy.Time.now() - msg.header.stamp).to_sec() * 1000.0
            # A negative or absurd value means the stamp is not an acquisition
            # time (old binary, or clocks not synchronised between machines) --
            # say so rather than reporting a meaningless number.
            if -1.0 < lat_ms < 5000.0:
                self._lat_sum += lat_ms
                self._lat_n   += 1
                if self._lat_n >= LATENCY_REPORT_EVERY:
                    rospy.loginfo(f"[LATENCY] /pose {self._lat_sum / self._lat_n:.0f} ms "
                                  f"(mean over {self._lat_n} poses)")
                    self._lat_sum = 0.0
                    self._lat_n   = 0
            else:
                rospy.logwarn_throttle(
                    30.0,
                    f"[LATENCY] implausible ({lat_ms:.0f} ms) — /pose is probably "
                    f"stamped at publication time, not acquisition. Check that "
                    f"carolus_astrobee runs with stamp_from_acquisition:=true, "
                    f"and that both machines' clocks agree.")
        except Exception:
            pass

        p = msg.pose.position
        q = msg.pose.orientation
        # Beacon yaw (rotation about the camera's y axis = horizontal tilt)
        siny = 2.0 * (q.w * q.y + q.z * q.x)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_deg = math.degrees(math.atan2(siny, cosy))
        self.posebuf.update(p.x, p.y, p.z, yaw_deg)

    def _battery_cb(self, info):
        try:
            if isinstance(info, (list, tuple)) and len(info) == 4:
                pct, temp_c, cur_ma, adc = info
            else:
                pct  = info[0] if isinstance(info, (list, tuple)) else int(info)
                temp_c, cur_ma, adc = 0.0, 0, 0
            with self._telem_lock:
                self._telem['bat'] = (int(pct), float(temp_c), int(cur_ma), int(adc))
        except Exception:
            pass

    def _mode_cb(self, msg):
        new_mode = msg.data.strip().upper()
        if new_mode not in ("AUTO", "MANUAL", "LOCATE"):
            return
        with self._mode_lock:
            old_mode = self._mode
            self._mode = new_mode
        if new_mode == "MANUAL" and old_mode != "MANUAL":
            self.stop_chassis()
            # 2026-08-14: this resume() is skipped when the gimbal was
            # deliberately suspended at startup (RM_B4_SUSPEND_GIMBAL=1).
            # Without the guard the suspend was pointless: the launcher sends
            # MODE MANUAL ~500 ms after T2 comes up, this fired, torque came
            # straight back, and the test lever silently did nothing.
            if B4_SUSPEND_GIMBAL or not GIMBAL_TORQUE_ON:
                # Without this guard the startup suspend was pointless: the
                # launcher sends MODE MANUAL ~500 ms after T2 comes up, this
                # fired, torque came straight back, and the gimbal resumed
                # drifting. Found 2026-08-14 while fixing exactly that.
                rospy.loginfo("[GIMBAL] MANUAL entered, torque deliberately left CUT "
                              "(RM_GIMBAL_TORQUE=off) -- no resume, no drift")
            else:
                try:
                    self.gimbal.resume()   # re-activer le couple au cas ou drive_speed(0,0) l'aurait suspendu
                except Exception as e:
                    rospy.logwarn(f"[GIMBAL] resume at MANUAL: {e}")
            rospy.loginfo("[CTRL] Mode -> MANUAL")
        elif new_mode == "LOCATE" and old_mode != "LOCATE":
            self.stop_chassis()
            self._search_since = time.time()
            self.search_edges  = 0
            self.search_steps  = 0
            self.search_axis_idx       = 0
            self._search_dist_total_m  = 0.0
            self._search_episode_since = time.time()
            rospy.loginfo("[CTRL] Mode -> LOCATE (sweep gimbal, no advance, no approach)")
        elif new_mode == "AUTO" and old_mode in ("MANUAL", "LOCATE"):
            self.stop_chassis()                # arret immediat des vitesses residuelles MANUEL
            self._search_since = time.time()   # grace period avant de recommencer a balayer
            self.search_edges  = 0             # repart sur un cycle de sweep propre
            self.search_steps  = 0             # repart sur un compteur d'avancements propre
            self.search_axis_idx       = 0             # nouvel episode -> premier axe du motif
            self._search_dist_total_m  = 0.0           # nouvel episode -> distance cumulee remise a zero
            self._search_episode_since = time.time()   # nouvel episode -> horloge globale remise a zero
            try:
                self.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
            except Exception:
                pass
            with self._gim_lock:
                self._gim_pitch = 0.0
                self._gim_yaw   = 0.0
            rospy.loginfo("[CTRL] Mode -> AUTO (grace period active)")

    def _cmdvel_cb(self, msg):
        with self._man_lock:
            self._man_vx    = msg.linear.x
            self._man_wz    = msg.angular.z
            self._man_stamp = time.time()

    def _gimbal_cb(self, msg):
        with self._gim_lock:
            self._gim_pitch = clamp(msg.angular.y, -GIMBAL_PITCH_MAX, GIMBAL_PITCH_MAX)
            self._gim_yaw   = clamp(msg.angular.z, -GIMBAL_YAW_MAX,   GIMBAL_YAW_MAX)
            self._gim_stamp = time.time()

    def _gimbal_lock_cb(self, msg):
        self._gimbal_lock = (msg.data.strip().upper() == "ON")
        rospy.loginfo(f"[LOCK] Lock balise (centrage periodique) -> {'ON' if self._gimbal_lock else 'OFF'}")

    def _gimbal_lock_period_cb(self, msg):
        """Live-configurable re-centring period (2026-07-23), in SECONDS only.
        Silent fallback to GIM_LOCK_PERIOD_S_DEFAULT if the received value is not
        a positive number (the same tolerance logic as a web form field)."""
        try:
            period = float(msg.data)
            if period <= 0:
                raise ValueError
        except (ValueError, TypeError):
            period = GIM_LOCK_PERIOD_S_DEFAULT
            rospy.logwarn(f"[LOCK] periode recue invalide ({msg.data!r}) -> repli sur {period}s")
        self._gimbal_lock_period_s = period
        # Guard: the subscribers are registered (~line 435) BEFORE the timer is created
        # (~line 442) in __init__; a message arriving in that window
        # would find _lock_timer=None (line 302) -> AttributeError on .shutdown().
        if self._lock_timer is not None:
            self._lock_timer.shutdown()
        self._lock_timer = rospy.Timer(rospy.Duration(period), self._gimbal_lock_tick)
        rospy.loginfo(f"[LOCK] re-centring period -> {period}s")

    def _gimbal_lock_tick(self, event):
        """BEACON LOCK (2026-07-23): every self._gimbal_lock_period_s seconds,
        if active and a fresh pose exists, send ONE relative gimbal.move() command
        that centres the beacon in frame. Independent of chassis motion.
        Yaw only -- pitch stays behind GIM_PITCH_TRACK_ENABLED (BUG-058)."""
        if not self._gimbal_lock:
            return
        if self._mode != "MANUAL":
            return
        # A RECENTER action in flight? Do not disturb it with a competing move().
        if time.time() < self._gimbal_busy_until:
            return
        # Note: the numpad no longer has priority here (night of 2026-07-23) -- while
        # LOCK is ON, manual gimbal piloting is ignored (in the MANUAL loop too).
        if not self.has_fresh_pose():
            return
        p = self.posebuf.get()
        if p is None or abs(p.z) < 0.05:
            return
        yaw_err_deg = math.degrees(math.atan2(p.x, abs(p.z)))
        if abs(yaw_err_deg) > GIM_LOCK_MAX_ERR_DEG:
            rospy.logwarn(f"[LOCK] erreur trop grande ({yaw_err_deg:.1f}°), pose probablement aberrante -> ignoree")
            return
        if abs(yaw_err_deg) < GIM_LOCK_DEADBAND_DEG:
            rospy.loginfo(f"[LOCK] already centred (err={yaw_err_deg:.1f} deg) -> no correction")
            return
        pitch_delta = 0.0
        if GIM_PITCH_TRACK_ENABLED:
            pitch_err_deg = math.degrees(math.atan2(p.y, abs(p.z)))
            pitch_delta = GIM_PITCH_SIGN * pitch_err_deg
        try:
            self.gimbal.move(pitch=pitch_delta, yaw=GIM_YAW_SIGN * yaw_err_deg,
                              pitch_speed=GIM_PITCH_SPEED_MAX, yaw_speed=GIM_LOCK_YAW_SPEED)
            rospy.loginfo(f"[LOCK] centrage : yaw_err={yaw_err_deg:.1f}° -> move(yaw={GIM_YAW_SIGN*yaw_err_deg:.1f})")
        except Exception as e:
            rospy.logwarn(f"[LOCK] gimbal.move a echoue: {e}")

    def _beacon_status_tick(self, event):
        """Periodic DETECTED/LOST log (2026-07-23), parsed by carolus_launcher.py
        (indicator, minimap). Also accumulates the beacon DUTY CYCLE.

        The duty cycle -- the fraction of time the beacon is actually in view --
        is arguably the most decision-relevant number in this project, and until
        2026-08-04 nobody had it. Carolus is the only drift-free source in the
        stack; everything else (wheel odometry, gyro integration) drifts without
        bound. So how often Carolus can correct decides how much inertial
        quality has to be bought:

          ~90% in view -> drift between corrections is negligible, and the
                          IMU-rate question (the SDK caps us at 50 Hz where
                          VIO tooling wants 200-500 Hz) largely stops mattering
          ~20% in view -> drift dominates, and that 50 Hz ceiling becomes a
                          structural limit of the robot

        Measuring it used to require a dedicated session. It is accumulated here
        instead, on the tick that already knows the answer, so every run reports
        it for free and the number is never stale.
        """
        fresh = self.has_fresh_pose()
        self._beacon_was_fresh = fresh

        # --- duty cycle accumulation (2026-08-04) ---
        self._duty_total += 1
        if fresh:
            self._duty_seen += 1
        now = time.time()
        if now - self._duty_since >= DUTY_REPORT_PERIOD_S and self._duty_total > 0:
            pct = 100.0 * self._duty_seen / self._duty_total
            rospy.loginfo(f"[DUTY] beacon in view {pct:.1f}% "
                          f"({self._duty_seen}/{self._duty_total} samples over "
                          f"{now - self._duty_since:.0f}s)")
            self._duty_since = now
            self._duty_seen = 0
            self._duty_total = 0

        if fresh:
            p = self.posebuf.get()
            if p is not None and abs(p.z) > 0.05:
                yaw_err_deg = math.degrees(math.atan2(p.x, abs(p.z)))
                pitch_err_deg = math.degrees(math.atan2(p.y, abs(p.z)))
                rospy.loginfo(f"[BEACON] status=DETECTED yaw_err={yaw_err_deg:.1f} pitch_err={pitch_err_deg:.1f}")
                return
        rospy.loginfo("[BEACON] status=LOST")

    def _gimbal_recenter_cb(self, msg):
        """RECENTER CAM (2026-07-23): returns the gimbal to its base position
        (pitch=0, yaw=0 in the gimbal's power-on frame, via the SDK's
        gimbal.recenter()) -- the CAMERA's orientation, independent of the
        chassis orientation. Same scope as LOCK (MANUAL mode only)."""
        if self._mode != "MANUAL":
            rospy.logwarn("[GIMBAL] RECENTER ignore : hors mode MANUEL")
            return
        # Open the "busy" window BEFORE starting the action: otherwise the MANUAL loop
        # (20 Hz) or the LOCK tick would send a competing command that cancels the
        # recenter before it completes -- the cause of the "recenter does not work" bug.
        self._gimbal_busy_until = time.time() + GIMBAL_RECENTER_BUSY_S
        try:
            # recenter() plafonne a [-360,360] deg/s (different de move(), 540 max) --
            # do not reuse GIM_LOCK_YAW_SPEED (540) as-is, it is out of range here.
            self.gimbal.recenter(pitch_speed=360.0, yaw_speed=360.0)
            rospy.loginfo("[GIMBAL] RECENTER: gimbal to base position (pitch=0, yaw=0)")
        except Exception as e:
            self._gimbal_busy_until = 0.0   # failure -> release immediately
            rospy.logwarn(f"[GIMBAL] RECENTER : gimbal.recenter a echoue: {e}")

    def _gimbal_angle_cb(self, info):
        if not isinstance(info, (list, tuple)) or len(info) < 4:
            return
        with self._gim_angle_lock:
            self._gim_yaw_rel    = info[1]
            self._gim_yaw_ground = info[3]
        self.pub_gimbal_yaw.publish(Float32(data=info[1]))
        self.pub_gimbal_yaw_ground.publish(Float32(data=info[3]))

        # ---- FRAME CHAIN (reworked 2026-08-04) -------------------------------
        # Until 2026-08-04 this published ONE transform, base_link -> camera_link,
        # carrying a rotation and NO translation at all. Three things were wrong
        # with that, and they explain several older open bugs:
        #
        #  1. The translation was never assigned, so we declared the camera to
        #     be at the chassis centre. Measured the same day: a beacon 1.00 m
        #     from the chassis centre reads 0.835 m through Carolus -- a 16.5 cm
        #     lever arm, the same order as the 12.3 cm the 2026-07-31 analysis
        #     needed to explain a +12 deg bearing drift.
        #
        #  2. Worse than a missing constant: the camera is NOT rigidly attached
        #     to base_link, it is on a gimbal. Its position in base_link is not
        #     a fixed offset -- the camera SWINGS ON AN ARC as the gimbal yaws.
        #     One collapsed transform models a moving point as stationary, and a
        #     bearing error that GROWS with gimbal rotation is exactly the
        #     signature recorded on 2026-07-31 (+12 deg over 97 deg).
        #
        #  3. The child was named camera_link while carrying GIMBAL axes. DJI
        #     puts X along the optical axis; ROS camera_link is X forward and
        #     camera_optical is Z forward. Anyone applying standard ROS camera
        #     conventions to that frame was 90 deg out.
        #
        # The chain below is the standard decomposition:
        #     base_link  -> gimbal_base    static translation (chassis centre
        #                                  to the gimbal yaw axis)
        #     gimbal_base-> gimbal_link    dynamic rotation (yaw/pitch, signed)
        #     gimbal_link-> camera_link    static translation (yaw/pitch axis
        #                                  intersection to the optical centre)
        #     camera_link-> camera_optical static (-90, 0, -90), the ROS optical
        #                                  convention
        #
        # SAFETY OF THIS CHANGE: with the default constants (both translations
        # zero, both signs +1) the composed base_link -> camera_link transform is
        # numerically identical to what this function published before. Nothing
        # moves until protocol 22's B1 supplies a measured sign and B2 a measured
        # lever arm. That is deliberate -- three of the four defects above cannot
        # be fixed without a measured sign, and writing a fix on an unverified
        # sign is precisely how BUG-077 became a workaround instead of a fix.
        now = rospy.Time.now()
        tfs = []

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "base_link"
        t.child_frame_id = "gimbal_base"
        t.transform.translation.x = BASE_TO_GIMBAL_XYZ[0]
        t.transform.translation.y = BASE_TO_GIMBAL_XYZ[1]
        t.transform.translation.z = BASE_TO_GIMBAL_XYZ[2]
        t.transform.rotation.w = 1.0
        tfs.append(t)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "gimbal_base"
        t.child_frame_id = "gimbal_link"
        q = tft.quaternion_from_euler(
            0.0,
            math.radians(GIMBAL_PITCH_SIGN_TF * info[0]),
            math.radians(GIMBAL_YAW_SIGN_TF * info[1]))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tfs.append(t)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "gimbal_link"
        t.child_frame_id = "camera_link"
        t.transform.translation.x = GIMBAL_TO_CAM_XYZ[0]
        t.transform.translation.y = GIMBAL_TO_CAM_XYZ[1]
        t.transform.translation.z = GIMBAL_TO_CAM_XYZ[2]
        t.transform.rotation.w = 1.0
        tfs.append(t)

        # camera_link (X forward, body convention) -> camera_optical (Z forward,
        # X right, Y down). Fixed rotation, the ROS convention every camera
        # driver publishes. Consumers expecting optical axes must use
        # camera_optical; camera_link keeps body axes so existing consumers
        # (beacon_absolute_pose.py) are unaffected.
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "camera_link"
        t.child_frame_id = "camera_optical"
        q = tft.quaternion_from_euler(-math.pi / 2.0, 0.0, -math.pi / 2.0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        tfs.append(t)

        self.tf_br_gimbal.sendTransform(tfs)

    def _esc_cb(self, sub_info):
        try:
            speed, angle, timestamp, state = sub_info
            with self._telem_lock:
                self._telem['esc'] = list(speed)
        except Exception:
            pass

    def _atti_cb(self, sub_info):
        try:
            yaw, pitch, roll = sub_info
            with self._telem_lock:
                self._telem['atti'] = (float(yaw), float(pitch), float(roll))
        except Exception:
            pass

    def _pos_cb(self, sub_info):
        try:
            x, y, z = sub_info
            with self._telem_lock:
                self._telem['pos'] = (float(x), float(y))
        except Exception:
            pass

    def _vel_cb(self, sub_info):
        try:
            vgx, vgy, vgz, vbx, vby, vbz = sub_info
            with self._telem_lock:
                self._telem['vel'] = (float(vbx), float(vby))
        except Exception:
            pass

    def _status_cb(self, sub_info):
        try:
            static, up_hill, down_hill, on_slope, is_pickup, slip, ix, iy, iz, roll_over, hill_static = sub_info
            with self._telem_lock:
                self._telem['status'] = {
                    'is_pickup': is_pickup,
                    'slip':      slip,
                    'roll_over': roll_over,
                    'on_slope':  on_slope,
                    'impact':    (ix or iy or iz),
                }
        except Exception:
            pass

    def _imu_cb(self, sub_info):
        """IMU brute (2026-07-30, prerequis calibration MINS). Republie directement
        sur /imu, sans passer par self._telem (contrairement aux autres sub_* de ce
        bloc) : un consommateur type Kalibr/MINS a besoin de chaque echantillon
        avec son propre timestamp, pas d'un dernier-etat interroge a la demande.

        Return order (ax, ay, az, wx, wy, wz) confirmed by a community ROS2
        example for this same rooted-S1 family (sub_imu(freq=50), see
        s1_sdk_hack_v0.0.5/.../ros2_robot.py:83-91) -- but the UNITS were NOT
        confirmed at the time (m/s^2 and rad/s assumed by analogy with
        sensor_msgs/Imu). Since measured: the accelerometer returns **g**, not
        m/s^2 (BUG-092) -- at rest one axis reads 1.00664 where 9.81 was
        expected. The gyro units remain unconfirmed, which is why
        IMU_ACCEL_SCALE is left at 1.0 until scale, sign and gyro units can be
        settled together against a known rotation.
        orientation_covariance[0]=-1: the standard ROS convention for "no
        orientation data" (sub_imu provides only accelerometer + gyro).
        """
        try:
            ax, ay, az, wx, wy, wz = sub_info
        except (TypeError, ValueError):
            return
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.orientation_covariance[0] = -1.0
        # IMU_ACCEL_SCALE: 1.0 by default = published as the SDK returns them,
        # which measurement shows to be **g**, not the m/s^2 sensor_msgs/Imu
        # mandates (BUG-092). See the constant for why this is not simply set
        # to 9.80665 today.
        msg.linear_acceleration.x = float(ax) * IMU_ACCEL_SCALE
        msg.linear_acceleration.y = float(ay) * IMU_ACCEL_SCALE
        msg.linear_acceleration.z = float(az) * IMU_ACCEL_SCALE
        msg.angular_velocity.x = float(wx)
        msg.angular_velocity.y = float(wy)
        msg.angular_velocity.z = float(wz)
        # Guard (BUG-089, 2026-08-04): on the SDK build installed on the Pi,
        # dds.py:201 calls the callback DIRECTLY inside the dispatcher thread, with no
        # executor -- an exception propagating out kills that thread and cuts all
        # telemetry. That is exactly what happened for a whole session. The publisher
        # creation order is fixed above, but we no longer let an exception from this
        # callback be able to take the dispatcher down with it.
        try:
            self.pub_imu.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"[IMU] publish failed: {e}")

    def _dist_cb(self, sub_info):
        try:
            vals = [float(v) for v in sub_info if v is not None and float(v) > 0]
            with self._telem_lock:
                self._telem['dist'] = min(vals) if vals else 999.0
        except Exception:
            pass

    def _wheels_cb(self, msg):
        parts = msg.data.strip().split()
        with self._wheels_lock:
            if not parts or parts[0].upper() == "STOP":
                self._wheels_active = False
                self._wheels_vals   = (0, 0, 0, 0)
            elif len(parts) == 4:
                try:
                    w = tuple(int(p) for p in parts)
                    self._wheels_vals   = w
                    self._wheels_active = True
                except ValueError:
                    pass

    def _is_path_blocked(self):
        """Arbiter prioritaire : TOF < seuil OU collision map OU hors zone → (True, raison)."""
        with self._telem_lock:
            tof     = self._telem['dist']
            px, py  = self._telem['pos']
            yaw_deg = self._telem['atti'][0]
        if tof < OBSTACLE_TOF_CM:
            return True, f"TOF={tof:.0f}cm"
        if self._colmap.loaded:
            if not self._colmap.lookahead_is_free(px, py, yaw_deg, LOOKAHEAD_M):
                return True, f"MAP col={px:.2f}m yaw={yaw_deg:.0f}deg"
            if not self._colmap.point_in_zone(px, py):
                return True, f"GEOFENCE x={px:.2f} y={py:.2f}"
        return False, ""

    def get_gimbal_yaw_rel(self):
        with self._gim_angle_lock:
            return self._gim_yaw_rel

    # ---- helpers ----

    def has_fresh_pose(self):
        p = self.posebuf.get()
        return (p is not None) and (time.time() - p.stamp < POSE_TIMEOUT_S)

    def stop_chassis(self):
        try:
            self.chassis.drive_speed(x=0, y=0, z=0, timeout=1)
        except Exception:
            pass

    def _is_auto(self):
        with self._mode_lock:
            return self._mode in ("AUTO", "LOCATE")

    def _is_locate(self):
        with self._mode_lock:
            return self._mode == "LOCATE"

    def stop_gimbal(self):
        try:
            self.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)
        except Exception:
            pass

    def _gimbal_servo_yaw(self, p):
        """Gimbal yaw speed (deg/s) that centres the beacon, from the lateral error.
        Utilise par l'etat APPROACH (AUTO)."""
        yaw_err_deg = math.degrees(math.atan2(p.x, abs(p.z)))
        return clamp(GIM_YAW_SIGN * K_GIM_YAW * yaw_err_deg, -GIM_YAW_MAX, GIM_YAW_MAX)

    def do_gimbal_sweep(self):
        """Sweep the gimbal in the current direction. INSENSITIVE TO drive_speed's SIGN:
        end of travel is detected by |yaw_rel| > limit OR by a STALL (yaw_rel
        stops progressing = mechanical stop reached). The chassis does NOT move.
        Returns: 'FOUND' (pose detected) / 'EDGE' (end of travel) / 'MANUAL' (switch).
        Pilotage a l'angle lu (yaw_rel), la consigne de vitesse n'est pas fiable."""
        last_yaw = self.get_gimbal_yaw_rel()
        stall_t  = time.time()
        t_log    = 0.0
        # "armed": an end-of-travel can only be detected once we have first returned to
        # the central zone. Prevents an immediate re-trigger when a pass starts already
        # past the bound (overshoot from the previous pass).
        armed = abs(last_yaw) < (SEARCH_YAW_LIMIT - 15)
        while self.running and not rospy.is_shutdown():
            if self.has_fresh_pose():
                self.stop_gimbal()
                return "FOUND"
            if not self._is_auto():
                self.stop_gimbal()
                return "MANUAL"
            yaw = self.get_gimbal_yaw_rel()
            now = time.time()
            if now - t_log > 1.0:
                rospy.loginfo(f"[SEARCH] sweep dir={self.search_dir} yaw_rel={yaw:.1f} armed={armed}")
                t_log = now
            # progression : reset du timer de stall si l'angle bouge encore
            if abs(yaw - last_yaw) > 2.0:
                last_yaw = yaw
                stall_t  = now
            # arm once back in the central zone
            if not armed and abs(yaw) < (SEARCH_YAW_LIMIT - 15):
                armed = True
            # End of travel: the bound (only once armed) OR a stall (always checked,
            # and it covers the opposite sign too: pushing into the mechanical stop
            # leaves yaw unchanged)
            hit_limit = armed and abs(yaw) > SEARCH_YAW_LIMIT
            hit_stall = (now - stall_t > 1.5)
            if hit_limit or hit_stall:
                self.stop_gimbal()
                reason = "borne" if hit_limit else "butee"
                rospy.loginfo(f"[SEARCH] fin de course ({reason}) yaw_rel={yaw:.1f} -> inversion")
                return "EDGE"
            try:
                self.gimbal.drive_speed(pitch_speed=0,
                                        yaw_speed=SEARCH_GIMBAL_YAW_S * self.search_dir)
            except Exception:
                pass
            time.sleep(0.05)
        self.stop_gimbal()
        return "MANUAL"

    def do_step_forward(self):
        """Full sweep with no target: re-centre the gimbal on the body (yaw_rel -> 0),
        then advance one step in a straight line. Watches pose/mode while driving."""
        rospy.loginfo(f"[SEARCH] Full cycle with no target -> recenter + advance {SEARCH_STEP_M}m")
        try:
            self.gimbal.recenter(pitch_speed=0, yaw_speed=60)
        except Exception as e:
            rospy.logwarn(f"[SEARCH] recenter: {e}")
        t0 = time.time()
        while time.time() - t0 < RECENTER_TIMEOUT_S:
            if not self._is_auto() or self._is_locate():
                return
            if abs(self.get_gimbal_yaw_rel()) < 5.0:
                break
            time.sleep(0.1)
        dur = SEARCH_STEP_M / SEARCH_STEP_VX
        t = time.time()
        while time.time() - t < dur:
            if not self.running or rospy.is_shutdown():
                break
            if self.has_fresh_pose() or not self._is_auto() or self._is_locate():
                break
            blocked, reason = self._is_path_blocked()
            if blocked:
                rospy.logwarn(f"[SEARCH] avancement bloque ({reason}) — arret")
                break
            try:
                self.chassis.drive_speed(x=SEARCH_STEP_VX, y=0.0, z=0.0, timeout=1)
            except Exception:
                pass
            time.sleep(0.1)
        self.stop_chassis()

    def _rotate_body_by(self, delta_deg):
        """Rotate the chassis by a fixed angle (absolute value measured via EP attitude).
        The command is always issued in the same direction -- we target a fixed
        RELATIVE rotation, not an absolute heading, so sub_attitude's exact sign
        need not be known in advance (unlike the gimbal servo or the body in
        ALIGN, which aim at a precise target value).

        The gimbal is recentred beforehand (as in do_step_forward): it is
        inertially stabilised (world frame), so after a sweep it can sit close to
        its ~250 deg mechanical limit. Rotating the chassis under a gimbal that
        holds its world heading would move yaw_rel by the same amount --
        recentring first avoids pushing it past the limit."""
        try:
            self.gimbal.recenter(pitch_speed=0, yaw_speed=60)
        except Exception as e:
            rospy.logwarn(f"[SEARCH] recenter avant rotation axe: {e}")
        t0 = time.time()
        while time.time() - t0 < RECENTER_TIMEOUT_S:
            if not self._is_auto() or self._is_locate():
                return
            if abs(self.get_gimbal_yaw_rel()) < 5.0:
                break
            time.sleep(0.1)

        with self._telem_lock:
            yaw0 = self._telem['atti'][0]
        t0 = time.time()
        while time.time() - t0 < SEARCH_FAN_ROT_TIMEOUT_S:
            if not self.running or rospy.is_shutdown() or not self._is_auto() or self._is_locate():
                break
            with self._telem_lock:
                yaw_now = self._telem['atti'][0]
            if abs(_angle_diff_deg(yaw_now, yaw0)) >= delta_deg:
                break
            try:
                self.chassis.drive_speed(x=0.0, y=0.0, z=SEARCH_FAN_ROT_WZ, timeout=1)
            except Exception:
                pass
            time.sleep(0.05)
        self.stop_chassis()

    # ---- boucle de contrôle (thread séparé) ----

    def _control_loop(self):
        state = "SEARCH"
        self._search_since = time.time()
        rospy.loginfo("[CTRL] Started -- place the beacon in front of the camera")
        time.sleep(2.0)
        rospy.loginfo(f"[CTRL] State: SEARCH (grace period {SEARCH_GRACE_S:.0f}s)")

        while self.running and not rospy.is_shutdown():

            with self._mode_lock:
                mode = self._mode

            # Logging telemetrie (dashboard GUI) — throttle 1 Hz
            with self._telem_lock:
                s                    = list(self._telem['esc'])
                _yaw, _pitch, _roll  = self._telem['atti']
                _px, _py             = self._telem['pos']
                _pct, _tc, _cur, _adc = self._telem['bat']
                _vbx, _vby           = self._telem['vel']
                _st                  = dict(self._telem['status'])
                _tof                 = self._telem['dist']
            rospy.loginfo_throttle(1.0, f"[ESC] W1={s[0]} W2={s[1]} W3={s[2]} W4={s[3]}")
            rospy.loginfo_throttle(1.0, f"[ATTI] yaw={_yaw:.1f} pitch={_pitch:.1f} roll={_roll:.1f}")
            rospy.loginfo_throttle(1.0, f"[POS] x={_px:.3f} y={_py:.3f}")

            # /odom publication (F2, 2026-07-21) -- the same values already read above,
            # with no extra subscription or connection. y converted to REP-103 (-y_ep).
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = _px
            odom.pose.pose.position.y = -_py
            odom.pose.pose.position.z = 0.0
            yaw_rad = math.radians(_yaw)
            odom.pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            odom.pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
            odom.twist.twist.linear.x = _vbx
            odom.twist.twist.linear.y = -_vby
            odom.pose.covariance  = _ODOM_POSE_COV
            odom.twist.covariance = _ODOM_TWIST_COV
            self.pub_odom.publish(odom)
            rospy.loginfo_throttle(5.0, f"[BAT] {_pct}%  temp={_tc:.1f}C  cur={_cur}mA  adc={_adc}")
            rospy.loginfo_throttle(1.0, f"[VEL] vbx={_vbx:.3f} vby={_vby:.3f}")
            if any(_st.values()):
                rospy.loginfo_throttle(1.0,
                    f"[STATUS] pickup={_st.get('is_pickup',0)} slip={_st.get('slip',0)} "
                    f"roll={_st.get('roll_over',0)} slope={_st.get('on_slope',0)} impact={_st.get('impact',0)}")
            if _tof < 200:
                rospy.loginfo_throttle(1.0, f"[TOF] front={_tof:.0f}cm")

            # Beacon position (for the map) if a fresh pose exists
            _p = self.posebuf.get()
            if _p is not None and (time.time() - _p.stamp < POSE_TIMEOUT_S):
                rospy.loginfo_throttle(0.5,
                    f"[BEACONPOS] x={_p.x:.3f} z={_p.z:.3f} byaw={_p.yaw:.1f}")

            # ── MODE MANUEL ──────────────────────────────────────────────────
            if mode == "MANUAL":
                with self._man_lock:
                    vx    = self._man_vx
                    wz    = self._man_wz
                    stamp = self._man_stamp

                with self._wheels_lock:
                    wheels_active = self._wheels_active
                    w1, w2, w3, w4 = self._wheels_vals

                if wheels_active:
                    # Commande roues individuelles (tilt / wheelie)
                    try:
                        self.chassis.drive_wheels(w1=w1, w2=w2, w3=w3, w4=w4, timeout=1)
                    except Exception:
                        pass
                else:
                    # Bug 4: safety timeout -- stop if no recent command.
                    # `cmd_fresh` is consumed by the BUG-093 brake-once logic below.
                    cmd_fresh = (time.time() - stamp) <= MANUAL_CMDVEL_TIMEOUT
                    if not cmd_fresh:
                        vx, wz = 0.0, 0.0

                    # Garde-fou minimal en MANUEL (2026-07-22) : bloque l'AVANCE si un
                    # TOF obstacle is close. Rotation (wz) and reverse (vx<0) stay
                    # allowed so the robot can free itself. Deliberately TOF-only, not
                    # the map/geofence, which need a calibrated odometric origin --
                    # avoids false blocks while driving manually. Inert if the TOF is
                    # absent (S1 roote) : _telem['dist'] reste a 999.0 -> jamais declenche.
                    if vx > 0.0:
                        with self._telem_lock:
                            _tof_cm = self._telem['dist']
                        if _tof_cm < OBSTACLE_TOF_CM:
                            vx = 0.0
                            rospy.logwarn_throttle(1.0, f"[MANUAL] avance bloquee (TOF={_tof_cm:.0f}cm)")

                    # BUG-093 FIX (2026-08-10) — brake once, then go silent.
                    #
                    # Measured by protocol 22's B4 the same day: re-sending
                    # drive_speed(0,0,0) at 20 Hz IS the uncommanded rotation.
                    # Normal +0.1734 deg/s; gimbal torque cut +0.1327 deg/s (77%
                    # of it survives); commands suppressed +0.0000 deg/s. It was
                    # never the gimbal's gyro -- it is this loop.
                    #
                    # But simply not sending is NOT the fix, also measured: from
                    # 0.15 m/s, ceasing to send leaves a 16.2 cm tail over 1.1 s
                    # (the SDK's own command timeout eventually stops it, slowly).
                    # Sending ONE explicit zero first cuts that to 1.7 cm in
                    # 0.1 s -- a 10x shorter stopping distance for one extra call.
                    #
                    # So: while a command is fresh, drive normally. When the
                    # deadman expires, brake ONCE and then send nothing at all,
                    # which is the state B4 measured at exactly zero drift.
                    if not B4_NO_DRIVE:
                        # 2026-08-12 -- this block used to end in `except Exception: pass`,
                        # the same silent-failure anti-pattern that made BUG-103 invisible
                        # for five days (and BUG-087 before it). An SDK refusal here looks
                        # exactly like a robot that ignores you: telemetry keeps flowing,
                        # /odom keeps publishing, and nothing anywhere says the command
                        # was rejected. The gimbal path immediately below already logged
                        # its errors; the chassis path did not. It does now.
                        #
                        # The [MANUAL-DRIVE] line is the positive counterpart, and exists
                        # to separate three otherwise-identical symptoms when "the robot
                        # does not move": the call is not reached at all (line absent ->
                        # wrong mode, or wheels_active stuck true), the call is made and
                        # throws (error line), or the call is made and returns cleanly
                        # while the robot stays put (line present, no error -> the SDK
                        # accepted and dropped it, look at the connection, not the code).
                        try:
                            if cmd_fresh:
                                self._idle_braked = False
                                rospy.loginfo_throttle(
                                    1.0, f"[MANUAL-DRIVE] drive_speed vx={vx:.3f} wz={wz:.1f}")
                                self.chassis.drive_speed(x=vx, y=0.0, z=wz, timeout=1)
                            elif not self._idle_braked:
                                self.chassis.drive_speed(x=0.0, y=0.0, z=0.0, timeout=1)
                                self._idle_braked = True
                            # else: deliberately send NOTHING -- see above
                        except Exception as e:
                            rospy.logwarn_throttle(
                                1.0, f"[MANUAL-DRIVE] chassis.drive_speed REFUSEE: {e}")

                # Gimbal (MANUEL) : arbitrage en 3 priorites (2026-07-23 nuit) :
                #   1. Async action in flight (RECENTER) -> emit nothing, let it finish
                #      (drive_speed(0,0) would cancel it, see GIMBAL_RECENTER_BUSY_S).
                #   2. LOCK ON -> the _gimbal_lock_tick drives via move(); the numpad
                #      is IGNORED while LOCK is active (user request).
                #   3. Otherwise -> direct numpad relay (0,0 when no key is held = hold).
                # The SDK invalidates velocity commands after ~0.1-0.5 s without a
                # repeat, hence the unconditional send on every tick in case 3.
                if time.time() < self._gimbal_busy_until:
                    pass
                elif self._gimbal_lock:
                    pass
                else:
                    with self._gim_lock:
                        gim_pitch = self._gim_pitch
                        gim_yaw   = self._gim_yaw
                        gim_stamp = self._gim_stamp
                    # BUG-100: same deadman the chassis already had.
                    gim_fresh = (time.time() - gim_stamp) <= MANUAL_GIMBAL_TIMEOUT
                    if not gim_fresh:
                        gim_pitch, gim_yaw = 0.0, 0.0

                    # BUG-106 FIX (2026-08-12) — brake once, then go silent.
                    #
                    # This is BUG-093's fix, applied to the gimbal. BUG-093
                    # established that re-sending drive_speed(0,0,0) to the
                    # CHASSIS at 20 Hz is itself what rotates the robot, and
                    # was fixed here by braking once and then sending nothing.
                    # The identical pattern was left in place on the gimbal,
                    # and it turns out to drive the same symptom by a
                    # different route.
                    #
                    # Measured 2026-08-12, with the chassis loop already
                    # silent (`[MANUAL-DRIVE]` absent from the whole run) and
                    # nothing publishing on /carolus/cmd_vel:
                    #   d_chassis_yaw = +1.07 deg over 10 s   (from /odom)
                    #   d_gimbal_rel  = -1.10 deg over 10 s   (sub_angle)
                    #   sum           = -0.03 deg
                    # The sum being zero is the whole proof: the gimbal holds
                    # its heading while the CHASSIS turns underneath it by
                    # exactly the opposite amount. +0.107 deg/s is ~64 deg in
                    # ten minutes, which is the drift reported from the field.
                    #
                    # Mechanism: a continuous stream of zero-velocity commands
                    # keeps the gimbal yaw motor actively servoing. Its
                    # reaction torque has to go somewhere, and Mecanum wheels
                    # resist yaw very poorly, so the chassis is what moves.
                    # Ceasing to send lets the axis settle instead of holding.
                    #
                    # As on the chassis, one explicit zero is still sent first
                    # rather than simply stopping: the SDK invalidates a
                    # velocity command after ~0.1-0.5 s, and letting it lapse
                    # leaves the last non-zero setpoint running until then.
                    try:
                        if gim_fresh:
                            self._gim_idle_braked = False
                            self.gimbal.drive_speed(pitch_speed=gim_pitch,
                                                    yaw_speed=gim_yaw)
                        elif not self._gim_idle_braked:
                            self.gimbal.drive_speed(pitch_speed=0.0, yaw_speed=0.0)
                            self._gim_idle_braked = True
                            rospy.loginfo("[GIMBAL] idle -> braked once, now silent "
                                          "(BUG-106)")
                        # else: deliberately send NOTHING -- see above
                    except Exception as e:
                        rospy.logwarn_throttle(2.0, f"[GIMBAL] drive_speed error: {e}")

                time.sleep(0.05)
                continue

            # ── MODE AUTO ────────────────────────────────────────────────────

            if state == "SEARCH":
                if self.has_fresh_pose():
                    if not self._is_locate():
                        self.stop_gimbal()
                        self.stop_chassis()
                        yaw_init = self.get_gimbal_yaw_rel()
                        self._align_since    = time.time()
                        self._align_yaw_init = yaw_init
                        self._align_valid_n  = 0
                        state = "ALIGN"
                        rospy.loginfo(f"[CTRL] State: ALIGN (yaw_rel_init={yaw_init:.1f}deg)")
                    else:
                        # LOCATE: beacon visible -- freeze the position and log it
                        self.stop_gimbal()
                        self.stop_chassis()
                        time.sleep(0.2)
                    continue
                # Grace period : attendre avant de commencer a balayer
                if time.time() - self._search_since < SEARCH_GRACE_S:
                    time.sleep(0.1)
                    continue
                # Sweep gimbal (robot immobile)
                result = self.do_gimbal_sweep()
                if result == "FOUND":
                    if not self._is_locate():
                        self.stop_chassis()
                        self.search_steps = 0
                        yaw_init = self.get_gimbal_yaw_rel()
                        self._align_since    = time.time()
                        self._align_yaw_init = yaw_init
                        self._align_valid_n  = 0
                        state = "ALIGN"
                        rospy.loginfo(f"[CTRL] State: ALIGN (yaw_rel_init={yaw_init:.1f}deg)")
                    else:
                        rospy.loginfo_throttle(2.0, "[LOCATE] balise trouvee — position sur map mise a jour")
                elif result == "MANUAL":
                    continue
                elif result == "EDGE":
                    # Limit reached: reverse direction. Two edges make a full cycle
                    # (the gimbal covered +/-limit) -> take one step in a straight line.
                    self.search_edges += 1
                    self.search_dir   = -self.search_dir
                    if self.search_edges >= 2:
                        self.search_edges = 0
                        if self._is_locate():
                            # LOCATE : ne jamais avancer, comportement voulu — rien a logger
                            pass
                        elif self.search_steps < SEARCH_MAX_STEPS:
                            # Borne globale de l'episode de recherche (temps ou distance
                            # cumulee) — Perplexity 08 : 90s ou ~13.5m plutot qu'un
                            # a fixed axis count, more robust to field variability.
                            elapsed = time.time() - self._search_episode_since
                            if (elapsed >= SEARCH_TOTAL_TIMEOUT_S
                                    or self._search_dist_total_m >= SEARCH_TOTAL_DIST_M):
                                self.stop_chassis()
                                rospy.logwarn_throttle(
                                    10.0,
                                    f"[SEARCH] borne recherche atteinte ({elapsed:.0f}s, "
                                    f"{self._search_dist_total_m:.1f}m parcourus) — "
                                    "arret, sweep sur place (intervention requise)")
                            else:
                                self.do_step_forward()
                                self.search_steps += 1
                                self._search_dist_total_m += SEARCH_STEP_M
                        elif self.search_axis_idx < SEARCH_FAN_AXIS_COUNT - 1:
                            # Axe epuise (SEARCH_MAX_STEPS avancements sans cible) : motif
                            # fan pattern -> turn the chassis to the next axis rather
                            # than giving up (60 deg increment, 6 axes).
                            self.search_axis_idx += 1
                            rospy.loginfo(
                                f"[SEARCH] axe {self.search_axis_idx + 1}/{SEARCH_FAN_AXIS_COUNT} "
                                f"epuise sans cible -> rotation {SEARCH_FAN_AXIS_INC_DEG:.0f}deg")
                            self._rotate_body_by(SEARCH_FAN_AXIS_INC_DEG)
                            self.search_steps = 0
                        else:
                            # Every axis of the fan pattern exhausted with no detection.
                            self.stop_chassis()
                            rospy.logwarn_throttle(
                                10.0,
                                f"[SEARCH] {SEARCH_FAN_AXIS_COUNT} axes du motif en eventail "
                                "epuises — arret, sweep sur place (intervention requise)")

            elif state == "ALIGN":
                # Pose fraiche ?
                if not self.has_fresh_pose():
                    self.stop_chassis()
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges  = 0
                    self.search_steps  = 0
                    self._align_valid_n = 0
                    rospy.loginfo("[CTRL] ALIGN: cible perdue -> SEARCH")
                    continue

                # Timeout dynamique : alpha * |yaw_rel_init| / ALIGN_MAX_WZ, plancher 2s, cap 12s
                timeout = max(2.0, min(
                    ALIGN_TIMEOUT_ALPHA * abs(self._align_yaw_init) / max(ALIGN_MAX_WZ, 1.0),
                    ALIGN_TIMEOUT_MAX_S))
                if time.time() - self._align_since > timeout:
                    self.stop_chassis()
                    self.posebuf.invalidate()
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges  = 0
                    self.search_steps  = 0
                    self._align_valid_n = 0
                    rospy.logwarn(f"[CTRL] ALIGN: timeout ({timeout:.1f}s) -> SEARCH")
                    continue

                yaw_rel = self.get_gimbal_yaw_rel()

                # Commande rotation (deadband evite chattering autour de 0)
                if abs(yaw_rel) < ALIGN_YAW_DEADBAND:
                    wz = 0.0
                else:
                    wz = clamp(K_BODY_YAW * yaw_rel, -ALIGN_MAX_WZ, ALIGN_MAX_WZ)

                # Count consecutive valid poses while yaw_rel is within the threshold
                p = self.posebuf.get()
                if p is not None and abs(yaw_rel) < ALIGN_YAW_THRESHOLD:
                    depth = abs(p.z)
                    if 0.3 < depth < 2.5:
                        self._align_valid_n += 1
                    else:
                        self._align_valid_n = 0
                else:
                    self._align_valid_n = 0

                rospy.loginfo_throttle(1.0,
                    f"[ALIGN] yaw_rel={yaw_rel:.1f}deg wz={wz:.1f} "
                    f"valides={self._align_valid_n}/{ALIGN_VALID_POSES}")

                # Transition APPROACH apres N poses valides consecutives
                if self._align_valid_n >= ALIGN_VALID_POSES:
                    self.stop_chassis()
                    state = "APPROACH"
                    self._align_valid_n = 0
                    rospy.loginfo(
                        f"[CTRL] ALIGN -> APPROACH (yaw_rel={yaw_rel:.1f}deg, "
                        f"{ALIGN_VALID_POSES} poses valides)")
                    continue

                # Gimbal servo OFF during ALIGN -- world stabilisation is enough to keep
                self.stop_gimbal()

                try:
                    self.chassis.drive_speed(x=0.0, y=0.0, z=wz, timeout=1)
                except Exception as e:
                    rospy.logwarn(f"[ALIGN] drive_speed error: {e}")

                time.sleep(0.10)

            elif state == "APPROACH":
                if not self.has_fresh_pose():
                    self.stop_chassis()
                    self.stop_gimbal()
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges = 0
                    self.search_steps = 0
                    rospy.loginfo("[CTRL] Lost target -> SEARCH")
                    continue

                p = self.posebuf.get()
                if p is None:
                    continue
                depth = abs(p.z)
                if depth <= 0.05:
                    time.sleep(0.05)
                    continue

                dist_err    = depth - STOP_DISTANCE_M

                if dist_err <= 0.0:
                    self.stop_chassis()
                    self.stop_gimbal()
                    state = "STOP"
                    rospy.loginfo("[CTRL] Reached target -> STOP")
                    continue

                # Gimbal: lock the beacon at the image centre (servo on the lateral
                # error). Being world-stabilised, it holds the target even while the body
                # turns. Servo shared with the MANUAL auto-track (_gimbal_servo_yaw).
                gim_yaw = self._gimbal_servo_yaw(p)
                try:
                    self.gimbal.drive_speed(pitch_speed=0, yaw_speed=gim_yaw)
                except Exception:
                    pass

                # Chassis: turn to null yaw_rel (realigning the body behind the camera)
                # and advance in proportion to the remaining distance.
                yaw_rel = self.get_gimbal_yaw_rel()
                vx = clamp(K_VX * dist_err, 0.0, MAX_VX)
                if vx < MIN_VX and dist_err > 0.15:
                    vx = MIN_VX
                wz = clamp(K_BODY_YAW * yaw_rel, -MAX_WZ, MAX_WZ)

                rospy.loginfo_throttle(1.0, f"[APPROACH] depth={depth:.3f}m dist_err={dist_err:.3f}m "
                                       f"yaw_rel={yaw_rel:.1f} gim_yaw={gim_yaw:.1f} vx={vx:.3f} wz={wz:.1f}")

                # Arbiter prioritaire : obstacle > servoing visuel
                blocked, reason = self._is_path_blocked()
                if blocked:
                    self.stop_chassis()
                    self.stop_gimbal()
                    self.posebuf.invalidate()
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges  = 0
                    self.search_steps  = 0
                    rospy.logwarn(f"[OBSTACLE] {reason} — APPROACH suspendu -> SEARCH")
                    continue

                try:
                    self.chassis.drive_speed(x=vx, y=0.0, z=wz, timeout=1)
                except Exception as e:
                    rospy.logwarn(f"[APPROACH] drive_speed error: {e}")

                time.sleep(0.10)

            elif state == "STOP":
                self.stop_chassis()
                self.stop_gimbal()
                if not self.has_fresh_pose():
                    state = "SEARCH"
                    self._search_since = time.time()
                    self.search_edges = 0
                    self.search_steps = 0
                    rospy.loginfo("[CTRL] Target lost -> SEARCH")
                time.sleep(0.2)

        self.stop_chassis()
        rospy.loginfo("[CTRL] Boucle de controle terminee")

    # ---- boucle caméra (thread principal) ----

    def spin(self):
        ctrl = threading.Thread(target=self._control_loop, daemon=True)
        ctrl.start()

        # 2026-08-11 -- makes "which code is actually running?" answerable from
        # the log instead of by inference. BUG-093's fix (brake once when the
        # MANUAL deadman expires, then send nothing) was deployed and then
        # measured as still drifting, and there is no record of whether the
        # node had been restarted between the two -- a Python file on disk
        # changes nothing until the process is relaunched. Reading the code
        # afterwards could not settle it either way. This line settles it for
        # every future session: if it is absent from the log, the running node
        # predates the fix regardless of what the file on disk says.
        rospy.loginfo("[IDLE-POLICY] MANUAL deadman=%.2fs -> brake once, then silent "
                      "(BUG-093 fix present)", MANUAL_CMDVEL_TIMEOUT)
        rospy.loginfo("[CAM] Publishing on /camera/color/image_raw")
        while not rospy.is_shutdown():
            try:
                frame = self.cam.read_cv2_image(strategy="newest")
                if frame is None:
                    self.rate.sleep()
                    continue
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp    = rospy.Time.now()
                msg.header.frame_id = "camera_link"
                self.pub_img.publish(msg)
            except queue.Empty:
                pass
            except Exception as e:
                rospy.logerr_throttle(2.0, f"[CAM] Erreur : {e}")
            self.rate.sleep()

        self.running = False
        ctrl.join(timeout=2.0)
        self.shutdown()

    def shutdown(self):
        rospy.loginfo("[RM] Arret...")
        self.running = False
        try:
            self.gimbal.unsub_angle()
        except Exception:
            pass
        try:
            self.battery.unsub_battery_info()
        except Exception:
            pass
        try:
            self.chassis.unsub_esc()
        except Exception:
            pass
        try:
            self.chassis.unsub_attitude()
        except Exception:
            pass
        try:
            self.chassis.unsub_position()
        except Exception:
            pass
        try:
            self.chassis.unsub_velocity()
        except Exception:
            pass
        try:
            self.chassis.unsub_status()
        except Exception:
            pass
        try:
            self.chassis.unsub_imu()
        except Exception:
            pass
        try:
            self.ep.sensor.unsub_distance()
        except Exception:
            pass
        try:
            self.cam.stop_video_stream()
        except Exception:
            pass
        try:
            self.ep.close()
        except Exception:
            pass
        rospy.loginfo("[RM] Deconnecte proprement")


if __name__ == "__main__":
    rospy.init_node("ep_camera_beacon_follower")
    node = EPCameraBeaconFollower()
    node.spin()
