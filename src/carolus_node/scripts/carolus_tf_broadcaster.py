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
# (verifie : det(R)=1, R.R^T=I -- cf. research-log/07-perplexity/10-fix-quaternion-remapping-carolus-tf.md)
_R_BASIS_XYZW = tft.quaternion_from_matrix(np.array([
    [0, 0, -1, 0],
    [1, 0,  0, 0],
    [0, -1, 0, 0],
    [0, 0,  0, 1],
], dtype=float))


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
        # QUATERNION REMAPPING (composition, PAS permutation de composantes)
        #
        # q represente R_camera->beacon (verifie dans carolus_astrobee.cpp:1187,
        # bestPose.R.transpose() -- cf. research-log/07-perplexity/10-fix-...md).
        # Un quaternion de rotation change de base par composition, pas par
        # permutation des composantes comme un vecteur : q_ros = r ⊗ q, avec r
        # le quaternion de la rotation de changement de base Carolus->ROS.
        # =====================================================
        q_cam_xyzw = (q.x, q.y, q.z, q.w)
        qx_ros, qy_ros, qz_ros, qw_ros = tft.quaternion_multiply(_R_BASIS_XYZW, q_cam_xyzw)

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