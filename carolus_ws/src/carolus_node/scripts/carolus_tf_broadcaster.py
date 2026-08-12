#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, TransformStamped
import math

# Quaternion (x,y,z,w) -- convention native de tf.transformations -- de la rotation
# de changement de base Carolus -> ROS, correspondant a la matrice R utilisee pour
# le remapping de position ci-dessous:
#   R = [[ 0, 0,-1],
#        [ 1, 0, 0],
#        [ 0,-1, 0]]
# (verifie : det(R)=1, R.R^T=I -- derivation complete dans overleaf/technical.tex,
# chapitre "Converting a Carolus Pose into the ROS Frame")
_R_BASIS_XYZW = tft.quaternion_from_matrix(np.array([
    [0, 0, -1, 0],
    [1, 0,  0, 0],
    [0, -1, 0, 0],
    [0, 0,  0, 1],
], dtype=float))
_R_BASIS_INV_XYZW = tft.quaternion_inverse(_R_BASIS_XYZW)


class CarolusTFBroadcaster:
    def __init__(self):
        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Carolus pose subscriber (TON topic réel)
        self.sub = rospy.Subscriber(
            "/pose",
            PoseStamped,
            self.cb,
            queue_size=1
        )

        # Anti-saturation /tf
        self.MAX_RATE = 10.0          # Hz
        self.MIN_DT = 1.0 / self.MAX_RATE
        self.last_tf_time = 0.0

        # Change detection
        self.last_pos = None
        self.last_yaw = None

        rospy.loginfo("✅ Carolus TF broadcaster started (robust + NaN safe)")

    def cb(self, msg: PoseStamped):
        now = rospy.Time.now().to_sec()

        # =====================================================
        # RATE LIMIT (anti /tf saturation)
        # =====================================================
        if now - self.last_tf_time < self.MIN_DT:
            return

        # =====================================================
        # RAW CAROLUS DATA
        # =====================================================
        p = msg.pose.position
        q = msg.pose.orientation

        # =====================================================
        # HARD SAFETY CHECKS (CRITICAL)
        # Reject any invalid Carolus output
        # =====================================================
        # Position NaN / inf
        if not all(map(math.isfinite, [p.x, p.y, p.z])):
            return

        # Quaternion NaN / inf
        if not all(map(math.isfinite, [q.x, q.y, q.z, q.w])):
            return

        # Null quaternion (0,0,0,0)
        if abs(q.x) < 1e-6 and abs(q.y) < 1e-6 and abs(q.z) < 1e-6 and abs(q.w) < 1e-6:
            return

        # =====================================================
        # AXIS REMAPPING (Carolus camera → ROS convention)
        #
        # Carolus camera:
        #   X → right
        #   Y → down
        #   Z → forward
        #
        # ROS:
        #   X → forward
        #   Y → left
        #   Z → up
        # =====================================================
        x_ros = -p.z
        y_ros =  p.x
        z_ros = -p.y

        # =====================================================
        # QUATERNION REMAPPING (conjugation on the INVERSE of q_cam)
        #
        # 2026-08-11 -- fixes a direction bug introduced by BUG-048's own fix.
        #
        # q (msg.pose.orientation) is R_camera->beacon: carolus_astrobee.cpp
        # solves camera_point = R_raw * beacon_point + t (standard PnP
        # extrinsic, R_raw = R_beacon->camera -- see ceresP4P.cpp's
        # ReprojectionErrorWithAnalyticDiff::Evaluate), then TRANSPOSES before
        # publishing (carolus_astrobee.cpp:1249), so what actually reaches
        # /pose is R_raw^T = R_camera->beacon.
        #
        # This TF edge is camera_link -> beacon_observed, which by TF2
        # convention needs R_beacon->camera (v_camera_link = R * v_beacon),
        # i.e. exactly R_raw, i.e. exactly q_cam INVERTED, not q_cam itself.
        # BUG-048 (2026-07-20) fixed the operation (composition instead of a
        # vector-style component permutation) but kept the wrong direction --
        # a rotation operator's change of basis is a similarity transform,
        # q_ros = r * q_operator * r^-1, and q_operator here is q_cam^-1, not
        # q_cam.
        #
        # Verified two ways before changing this: (1) algebraically, from the
        # cost function above, not inferred from behaviour; (2) numerically,
        # against LIMO's independently-found (by trial and error, per
        # Hector's 2026-08-11 mail) empirical formula on their document's own
        # matrix (identical det=+1 matrix to ours) -- r*q_cam^-1*r^-1 matches
        # LIMO's formula exactly (< 1e-15) over 8 random quaternions, while
        # the previous r*q_cam formula differs from it by ~1.0-1.3 on every
        # trial. Not yet validated on hardware (tf_echo against a physically
        # known beacon orientation) -- see roadmap.md.
        # =====================================================
        q_cam_xyzw = (q.x, q.y, q.z, q.w)
        q_cam_inv_xyzw = tft.quaternion_inverse(q_cam_xyzw)
        qx_ros, qy_ros, qz_ros, qw_ros = tft.quaternion_multiply(
            tft.quaternion_multiply(_R_BASIS_XYZW, q_cam_inv_xyzw),
            _R_BASIS_INV_XYZW,
        )

        # =====================================================
        # QUATERNION NORMALIZATION (SAFETY)
        # =====================================================
        norm = math.sqrt(qx_ros*qx_ros + qy_ros*qy_ros +
                         qz_ros*qz_ros + qw_ros*qw_ros)
        if not math.isfinite(norm) or norm < 1e-6:
            return

        qx_ros /= norm
        qy_ros /= norm
        qz_ros /= norm
        qw_ros /= norm

        # =====================================================
        # YAW EXTRACTION (for change detection only)
        # =====================================================
        yaw = math.atan2(
            2.0 * (qw_ros * qz_ros + qx_ros * qy_ros),
            1.0 - 2.0 * (qy_ros*qy_ros + qz_ros*qz_ros)
        )

        pos = np.array([x_ros, y_ros, z_ros])

        # =====================================================
        # PUBLISH ONLY IF THE POSE REALLY CHANGED
        # =====================================================
        if self.last_pos is not None:
            if (np.linalg.norm(pos - self.last_pos) < 1e-3 and
                abs(yaw - self.last_yaw) < 1e-3):
                return

        self.last_tf_time = now
        self.last_pos = pos
        self.last_yaw = yaw

        # =====================================================
        # TF MESSAGE
        # =====================================================
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()

        # ✅ FORCE correct TF parent
        t.header.frame_id = "camera_link"
        t.child_frame_id = "beacon_observed"

        t.transform.translation.x = x_ros
        t.transform.translation.y = y_ros
        t.transform.translation.z = z_ros

        t.transform.rotation.x = qx_ros
        t.transform.rotation.y = qy_ros
        t.transform.rotation.z = qz_ros
        t.transform.rotation.w = qw_ros

        self.br.sendTransform(t)


if __name__ == "__main__":
    rospy.init_node("carolus_tf")
    CarolusTFBroadcaster()
    rospy.spin()