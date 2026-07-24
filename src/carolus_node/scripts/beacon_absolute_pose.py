#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Convertit la TF `camera_link -> beacon_observed` (publiee par
`carolus_tf_broadcaster.py`, deja en convention ROS/REP-103) plus la position
CONNUE de la balise dans le repere `odom` en une pose absolue de la camera
(approximee a `base_link`, cf. transform identite camera_link->honey/body dans
testcarolus.launch) -- publiee comme `PoseWithCovarianceStamped` sur
`/carolus/absolute_pose` pour fusion par `robot_localization` (F3).

Geometrie : la TF observee donne T_cam_beacon (position/orientation de la
balise VUE DEPUIS la camera). La pose de la camera dans le repere odom est
alors : T_odom_cam = T_odom_beacon * inverse(T_cam_beacon), avec T_odom_beacon
la pose connue (fixe, parametree) de la balise.

Limitations V1 assumees (a documenter/revisiter, cf. journal) :
- Une seule balise, position/orientation fixees par parametres ROS (pas de
  carte multi-balises -- prerequis F6 non rempli, cf. gimbal_bearing.py).
- camera_link == base_link (transform identite deja en vigueur dans le
  pipeline existant, pas une nouvelle approximation introduite ici).
- Covariance provisoire fixe (diagonale, parametrable) -- la caracterisation
  empirique par distance/angle (research-log/12-protocole-covariance-carolus.md)
  n'a pas encore ete faite (protocole terrain, materiel physique requis).
"""

import math

import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseWithCovarianceStamped


class BeaconAbsolutePose:
    def __init__(self):
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.camera_frame = rospy.get_param("~camera_frame", "camera_link")
        self.beacon_frame = rospy.get_param("~beacon_frame", "beacon_observed")

        # Pose connue de la balise dans le repere odom (parametrable -- balise
        # unique, V1). Orientation en RPY degres pour lisibilite du launch/yaml.
        self.beacon_x = rospy.get_param("~beacon_x", 0.0)
        self.beacon_y = rospy.get_param("~beacon_y", 0.0)
        self.beacon_z = rospy.get_param("~beacon_z", 0.0)
        self.beacon_roll_deg = rospy.get_param("~beacon_roll_deg", 0.0)
        self.beacon_pitch_deg = rospy.get_param("~beacon_pitch_deg", 0.0)
        self.beacon_yaw_deg = rospy.get_param("~beacon_yaw_deg", 0.0)

        self.T_odom_beacon = tft.concatenate_matrices(
            tft.translation_matrix((self.beacon_x, self.beacon_y, self.beacon_z)),
            tft.euler_matrix(
                math.radians(self.beacon_roll_deg),
                math.radians(self.beacon_pitch_deg),
                math.radians(self.beacon_yaw_deg),
            ),
        )

        # Covariance provisoire fixe (diagonale) -- placeholder documente
        # jusqu'a la caracterisation terrain (research-log/12-protocole-...).
        pos_var = rospy.get_param("~position_variance", 0.05 ** 2)   # (m^2)
        rot_var = rospy.get_param("~orientation_variance", math.radians(5.0) ** 2)  # (rad^2)
        self.covariance = [0.0] * 36
        for i in range(3):
            self.covariance[i * 6 + i] = pos_var
        for i in range(3, 6):
            self.covariance[i * 6 + i] = rot_var

        self.pub = rospy.Publisher(
            "/carolus/absolute_pose", PoseWithCovarianceStamped, queue_size=1
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.rate_hz = rospy.get_param("~publish_rate", 10.0)
        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._tick)

        rospy.loginfo(
            "beacon_absolute_pose: balise connue a (%.2f, %.2f, %.2f) m, "
            "yaw=%.1f deg dans '%s'",
            self.beacon_x, self.beacon_y, self.beacon_z, self.beacon_yaw_deg,
            self.odom_frame,
        )

    def _tick(self, _event):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.camera_frame, self.beacon_frame, rospy.Time(0),
                timeout=rospy.Duration(0.05),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        T_cam_beacon = tft.concatenate_matrices(
            tft.translation_matrix((t.x, t.y, t.z)),
            tft.quaternion_matrix((q.x, q.y, q.z, q.w)),
        )

        T_odom_cam = tft.concatenate_matrices(
            self.T_odom_beacon, tft.inverse_matrix(T_cam_beacon)
        )

        trans = tft.translation_from_matrix(T_odom_cam)
        quat = tft.quaternion_from_matrix(T_odom_cam)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = tf_msg.header.stamp
        msg.header.frame_id = self.odom_frame
        msg.pose.pose.position.x = trans[0]
        msg.pose.pose.position.y = trans[1]
        msg.pose.pose.position.z = trans[2]
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        msg.pose.covariance = self.covariance

        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("beacon_absolute_pose")
    BeaconAbsolutePose()
    rospy.spin()
