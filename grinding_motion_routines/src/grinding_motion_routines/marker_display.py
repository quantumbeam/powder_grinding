import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import tf
from tf.transformations import *
from geometry_msgs.msg import Quaternion

import numpy as np


class MarkerDisplay(object):
    last_index = 0

    def __init__(self, marker_publisher_name):
        self.pub = rospy.Publisher(marker_publisher_name, MarkerArray, queue_size=10)

    def clear_marker(self):
        marker = Marker()
        marker_array = MarkerArray()
        marker.action = Marker.DELETEALL
        marker_array.markers.append(marker)
        self.pub.publish(marker_array)

    def display_waypoints(self, waypoints, scale=0.003, type=None, clear=False):
        if clear:
            self.clear_marker()

        pub = self.pub
        marker = Marker()
        marker_array = MarkerArray()
        if type is None:
            type = marker.SPHERE

        rate = rospy.Rate(25)
        last_index = self.last_index
        # while not rospy.is_shutdown():
        for index, pose in enumerate(waypoints):
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "waypoints"
            marker.id = last_index + index
            marker.action = Marker.ADD

            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = pose[2]
            marker.pose.orientation = Quaternion(pose[3], pose[4], pose[5], pose[6])

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            if type == marker.ARROW:
                marker.scale.x = scale
                marker.scale.y = scale * 0.1
                marker.scale.z = scale * 0.1

                # arrowが初期状態だとX軸方向を向いているのでY軸をpi/2回転させて-Z方向を向かせる
                quat = [
                    marker.pose.orientation.x,
                    marker.pose.orientation.y,
                    marker.pose.orientation.z,
                    marker.pose.orientation.w,
                ]
                q_orig = np.array(quat)
                q_rot = quaternion_from_euler(0, np.pi / 2, 0)
                q_new = quaternion_multiply(q_rot, q_orig)
                marker.pose.orientation = Quaternion(
                    q_new[0], q_new[1], q_new[2], q_new[3]
                )
            else:
                marker.scale.x = scale
                marker.scale.y = scale
                marker.scale.z = scale

            marker.type = type

            marker.lifetime = rospy.Duration()  # existing marker for ever
            marker_array.markers.append(marker)

            rate.sleep()  # sleepかませないとpublihが早すぎるのかうまくいかない

            pub.publish(marker_array)
        self.last_index = last_index + index
