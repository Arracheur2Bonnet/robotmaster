#!/usr/bin/env python3
"""Carolus -> ground frame, via tf2. A generic template, not a robot-specific script.

WHY THIS EXISTS
---------------
Carolus publishes where the BEACON is, in the CAMERA's own frame. Almost every
useful question is the other way round and in a different frame: "where is my
ROBOT, on the ground?" Two previous integrations of Carolus (a Leo Rover and a
Limo) each answered that by hand-tuning a component permutation until the plot
looked right, and neither result transferred to the other robot -- the Limo
report says so in as many words: "The quaternion is not always the same on
Carolus to Limo for example or neither Carolus to Leo Rover ... I needed by
doing test to find the exact quaternion that works on Limo."

This script does it by derivation instead, and nothing in it is specific to one
robot: every frame name, the camera mounting, and the beacon's surveyed position
are ROS parameters. Point it at a different robot by changing a YAML file.

THE IDEA, IN ONE LINE
---------------------
The beacon does not move. So publishing `beacon -> camera` (the INVERSE of what
Carolus reports) turns a fixed beacon into a fixed anchor, and tf2 will then
chain that with your robot's own URDF to answer "where is base_link, in the
ground frame?" -- without this script ever computing it.

    ground --(static, you survey it once)--> beacon
    beacon --(this script, from /pose)-----> camera
    camera --(static, camera mounting)-----> base_link

tf2 composes and inverts all of that on demand. You only publish edges.

WHY THE ORIENTATION MATH IS NOT A PERMUTATION
---------------------------------------------
Position is a vector and changes basis with a single multiplication, which is
why the permutation (x,y,z)_ROS = (-z, x, -y)_camera that both prior projects
found by trial and error is genuinely correct -- it IS B @ v, written out.

A rotation is an operator, not a vector. It changes basis by CONJUGATION:

    q_ros = b (x) q_cam^-1 (x) b^-1

Permuting the quaternion's components instead -- the tempting shortcut, because
it is what works for the position -- gives a perfectly valid unit quaternion
describing the wrong rotation, with nothing to signal the error. The `^-1` is
separately load-bearing: Carolus transposes its solved rotation before
publishing, so /pose carries R_camera->beacon while a `beacon -> camera` tf edge
needs R_beacon->camera.

Full derivation, the numeric value of b, and the roll/pitch/yaw conversion:
overleaf/technical.tex, chapter "Converting a Carolus Pose into the ROS Frame".

VERIFIED
--------
The conjugation formula was checked against the matrix form B R^T B^-1 on 2000
random unit quaternions (max component difference 6.7e-16) and matches the
Limo project's independently-found empirical permutation exactly.

NOT VERIFIED: this script has not been run against a moving robot. The formula
it applies is the one validated on hardware 2026-08-17 (yaw -28.8, pitch -23.7,
roll -23.6 deg, zero axes inverted); the ground-frame CHAINING on top of it is
new here and untested.

USAGE
-----
Check the maths first -- needs ROS on PYTHONPATH for `tf.transformations`, but
no roscore, no robot, no beacon:

    source /opt/ros/noetic/setup.bash
    python3 carolus_ground_tf.py --selftest

Then run it for real:

    rosrun carolus_node carolus_ground_tf.py \
        _camera_frame:=camera_optical _beacon_frame:=beacon \
        _ground_frame:=map _beacon_xyz:="[2.0, 0.0, 0.8]"

Then ask tf2 the question you actually care about:

    rosrun tf tf_echo map base_link
"""

import math

import numpy as np
import tf.transformations as tft

# rospy / tf2_ros / geometry_msgs are imported lazily, inside main(), so that
# --selftest can exercise the maths without pulling in the node runtime.
#
# What that does and does NOT buy you, stated precisely because an earlier
# version of this docstring overclaimed it: `tf.transformations` above is still
# a ROS package, so ROS must be on PYTHONPATH either way -- source
# /opt/ros/noetic/setup.bash first. What the lazy import removes is the need
# for a roscore, a robot, or the node's own message dependencies.
rospy = None
tf2_ros = None
PoseStamped = None
TransformStamped = None


def _import_ros_runtime():
    """Pull in the node-only ROS pieces. Called from main(), not at import."""
    global rospy, tf2_ros, PoseStamped, TransformStamped
    import rospy as _rospy
    import tf2_ros as _tf2_ros
    from geometry_msgs.msg import PoseStamped as _PoseStamped
    from geometry_msgs.msg import TransformStamped as _TransformStamped
    rospy, tf2_ros = _rospy, _tf2_ros
    PoseStamped, TransformStamped = _PoseStamped, _TransformStamped


# --------------------------------------------------------------------------
# Change of basis: Carolus camera axes -> ROS body axes.
#
#   Carolus camera (= REP-103 "_optical"):  X right, Y down,  Z forward
#   ROS body       (= REP-103):             X forward, Y left, Z up
#
# As a matrix acting on coordinate columns. det(B) = +1, so it is a proper
# rotation and therefore HAS a quaternion representation -- an improper
# (det = -1) matrix would not, and would have to be handled as a matrix.
#
# Overridable via the `basis_matrix` parameter for a robot whose camera is
# mounted in a genuinely different orientation. Do not change it to "fix" a
# sign you do not understand: check the mounting, then the beacon survey.
# --------------------------------------------------------------------------
DEFAULT_BASIS = [[0, 0, -1],
                 [1, 0, 0],
                 [0, -1, 0]]


def basis_quaternion(matrix_3x3):
    """Unit quaternion (x,y,z,w) for a proper 3x3 rotation matrix.

    Raises if the matrix is not a proper rotation -- an improper matrix has no
    quaternion form at all, and silently returning something plausible here is
    exactly the class of error this whole module exists to avoid.
    """
    m = np.asarray(matrix_3x3, dtype=float)
    if m.shape != (3, 3):
        raise ValueError("basis_matrix must be 3x3, got %r" % (m.shape,))
    if not np.allclose(m @ m.T, np.eye(3), atol=1e-9):
        raise ValueError("basis_matrix is not orthogonal: M @ M.T != I")
    det = np.linalg.det(m)
    if not np.isclose(det, 1.0, atol=1e-9):
        raise ValueError(
            "basis_matrix has det=%.6f, expected +1. A determinant of -1 is a "
            "mirror, not a rotation, and has no quaternion representation." % det)
    m4 = np.eye(4)
    m4[:3, :3] = m
    return tft.quaternion_from_matrix(m4)


def carolus_pose_to_ros(position_xyz, quat_xyzw, basis_xyzw, basis_inv_xyzw):
    """Convert one Carolus /pose reading into the ROS body convention.

    Returns (translation_xyz, quaternion_xyzw) for the BEACON -> CAMERA edge,
    i.e. already inverted relative to what Carolus reports.
    """
    px, py, pz = position_xyz

    # Position: an ordinary vector, so a single multiplication by B.
    # Written out, B @ (px,py,pz) is exactly (-pz, px, -py).
    t_ros = np.array([-pz, px, -py], dtype=float)

    # Orientation: an operator, so conjugation -- and on the INVERSE, because
    # /pose carries R_camera->beacon while this edge needs R_beacon->camera.
    q_inv = tft.quaternion_inverse(quat_xyzw)
    q_ros = tft.quaternion_multiply(
        tft.quaternion_multiply(basis_xyzw, q_inv), basis_inv_xyzw)

    norm = float(np.linalg.norm(q_ros))
    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("degenerate quaternion after conjugation")
    return t_ros, q_ros / norm


def quaternion_to_rpy(q_xyzw):
    """REP-103 fixed-axis roll/pitch/yaw, radians. Input must be in ROS axes.

    Applying this to a raw Carolus quaternion produces three numbers that look
    reasonable and mean nothing -- roll/pitch/yaw are named after axes, and the
    axes are only right after the change of basis above.
    """
    x, y, z, w = np.asarray(q_xyzw, dtype=float) / np.linalg.norm(q_xyzw)
    sin_pitch = float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    if abs(sin_pitch) >= 0.99999:
        # Gimbal lock: only roll+yaw is determined. Fold it all into yaw.
        # NOTE the signs here differ from the general branch below -- this is
        # not the general formula with pitch substituted.
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
        roll = 0.0
        yaw = math.atan2(2.0 * (w * z - x * y), 1.0 - 2.0 * (x * x + z * z))
    else:
        pitch = math.asin(sin_pitch)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


class CarolusGroundTF(object):
    def __init__(self):
        self.camera_frame = rospy.get_param("~camera_frame", "camera_optical")
        self.beacon_frame = rospy.get_param("~beacon_frame", "beacon")
        self.ground_frame = rospy.get_param("~ground_frame", "map")
        self.pose_topic = rospy.get_param("~pose_topic", "/pose")
        self.max_rate = float(rospy.get_param("~max_rate", 20.0))

        # Where the beacon physically is, in the ground frame. You survey this
        # once with a tape measure; it is the only number here that comes from
        # the room rather than from the robot.
        self.beacon_xyz = [float(v) for v in
                           rospy.get_param("~beacon_xyz", [0.0, 0.0, 0.0])]
        self.beacon_rpy = [float(v) for v in
                           rospy.get_param("~beacon_rpy", [0.0, 0.0, 0.0])]

        basis = rospy.get_param("~basis_matrix", DEFAULT_BASIS)
        self.basis_xyzw = basis_quaternion(basis)
        self.basis_inv_xyzw = tft.quaternion_inverse(self.basis_xyzw)

        self.br = tf2_ros.TransformBroadcaster()
        self.static_br = tf2_ros.StaticTransformBroadcaster()
        self._publish_ground_to_beacon()

        self.min_dt = 1.0 / self.max_rate if self.max_rate > 0 else 0.0
        self.last_publish = 0.0
        self.rejected = 0

        self.sub = rospy.Subscriber(self.pose_topic, PoseStamped, self.cb,
                                    queue_size=1)
        rospy.loginfo("carolus_ground_tf: %s -> %s -> %s, from %s",
                      self.ground_frame, self.beacon_frame, self.camera_frame,
                      self.pose_topic)

    def _publish_ground_to_beacon(self):
        """The surveyed, unchanging ground -> beacon edge.

        Static, so it is published once with a latched broadcaster rather than
        re-sent every frame.
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.ground_frame
        t.child_frame_id = self.beacon_frame
        t.transform.translation.x = self.beacon_xyz[0]
        t.transform.translation.y = self.beacon_xyz[1]
        t.transform.translation.z = self.beacon_xyz[2]
        q = tft.quaternion_from_euler(*self.beacon_rpy)
        t.transform.rotation.x, t.transform.rotation.y = q[0], q[1]
        t.transform.rotation.z, t.transform.rotation.w = q[2], q[3]
        self.static_br.sendTransform(t)

    def cb(self, msg):
        now = rospy.Time.now().to_sec()
        if self.min_dt and (now - self.last_publish) < self.min_dt:
            return

        p, q = msg.pose.position, msg.pose.orientation
        if not all(map(math.isfinite, [p.x, p.y, p.z, q.x, q.y, q.z, q.w])):
            self.rejected += 1
            return
        if max(abs(q.x), abs(q.y), abs(q.z), abs(q.w)) < 1e-6:
            self.rejected += 1
            return

        try:
            t_ros, q_ros = carolus_pose_to_ros(
                (p.x, p.y, p.z), (q.x, q.y, q.z, q.w),
                self.basis_xyzw, self.basis_inv_xyzw)
        except ValueError:
            self.rejected += 1
            return

        tf_msg = TransformStamped()
        # Carry the measurement's own stamp, not now(): tf2 interpolates on
        # these, and stamping with now() quietly claims the pose was observed
        # later than it was.
        tf_msg.header.stamp = msg.header.stamp if msg.header.stamp != rospy.Time(0) \
            else rospy.Time.now()
        tf_msg.header.frame_id = self.beacon_frame
        tf_msg.child_frame_id = self.camera_frame
        tf_msg.transform.translation.x = float(t_ros[0])
        tf_msg.transform.translation.y = float(t_ros[1])
        tf_msg.transform.translation.z = float(t_ros[2])
        tf_msg.transform.rotation.x = float(q_ros[0])
        tf_msg.transform.rotation.y = float(q_ros[1])
        tf_msg.transform.rotation.z = float(q_ros[2])
        tf_msg.transform.rotation.w = float(q_ros[3])

        # Always publish, even when the pose has not changed. A tf transform is
        # not a latched value -- consumers look it up at a timestamp and tf2
        # expires buffer entries after ~10 s, so a transform that stops being
        # republished stops EXISTING ("frame does not exist", not "stale").
        # That bites hardest with a stationary robot and a stationary beacon,
        # which is exactly the validation setup.
        self.br.sendTransform(tf_msg)
        self.last_publish = now


def _selftest():
    """Needs ROS on PYTHONPATH (for tf.transformations) but no roscore, no
    robot, no messages:

        source /opt/ros/noetic/setup.bash
        python3 carolus_ground_tf.py --selftest

    Checks the two claims a reader would reasonably doubt, and is written so it
    can actually come back negative rather than passing by construction.
    """
    ok = True

    b = basis_quaternion(DEFAULT_BASIS)
    expect = np.array([0.5, 0.5, -0.5, -0.5])          # (x,y,z,w)
    same = np.allclose(b, expect) or np.allclose(b, -expect)
    print("[%s] b = %s (expect +/- %s)" % ("ok" if same else "FAIL", b, expect))
    ok &= same

    # Conjugation must equal the matrix form B R^T B^-1.
    B = np.asarray(DEFAULT_BASIS, dtype=float)
    b_inv = tft.quaternion_inverse(b)
    worst = 0.0
    rng = np.random.default_rng(0)
    for _ in range(2000):
        v = rng.normal(size=4)
        q = v / np.linalg.norm(v)
        _, q_ros = carolus_pose_to_ros((0.0, 0.0, 0.0), q, b, b_inv)
        lhs = tft.quaternion_matrix(q_ros)[:3, :3]
        rhs = B @ tft.quaternion_matrix(q)[:3, :3].T @ np.linalg.inv(B)
        worst = max(worst, float(np.abs(lhs - rhs).max()))
    good = worst < 1e-12
    print("[%s] conjugation vs B R^T B^-1 over 2000 quats: max diff %.2e"
          % ("ok" if good else "FAIL", worst))
    ok &= good

    # Position permutation must equal B @ v.
    v = np.array([0.11, -0.23, 0.87])
    t, _ = carolus_pose_to_ros(v, (0, 0, 0, 1), b, b_inv)
    good = np.allclose(t, B @ v)
    print("[%s] position: %s == B@v %s" % ("ok" if good else "FAIL", t, B @ v))
    ok &= good

    # An improper matrix must be REJECTED, not silently accepted. Without this
    # the orthogonality/determinant guard could be vacuous.
    try:
        basis_quaternion([[1, 0, 0], [0, 1, 0], [0, 0, -1]])
        print("[FAIL] improper (det=-1) matrix was accepted")
        ok = False
    except ValueError:
        print("[ok] improper (det=-1) matrix correctly rejected")

    # Gimbal-lock branch must reconstruct the same rotation.
    worst = 0.0
    for sign in (1.0, -1.0):
        for yaw0 in np.linspace(-3.0, 3.0, 15):
            m = tft.euler_matrix(0.0, sign * math.pi / 2, yaw0, 'sxyz')
            q = tft.quaternion_from_matrix(m)
            r, p_, y_ = quaternion_to_rpy(q)
            worst = max(worst, float(np.abs(
                tft.euler_matrix(r, p_, y_, 'sxyz') - m).max()))
    good = worst < 1e-9
    print("[%s] gimbal-lock rpy round-trip: max diff %.2e"
          % ("ok" if good else "FAIL", worst))
    ok &= good

    print("\n%s" % ("ALL CHECKS PASSED" if ok else "SOME CHECKS FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    if "--selftest" in sys.argv:
        sys.exit(_selftest())
    _import_ros_runtime()
    rospy.init_node("carolus_ground_tf")
    CarolusGroundTF()
    rospy.spin()
