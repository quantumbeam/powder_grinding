#!/usr/bin/env python3
import rospy
from rospy.rostime import Duration
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

import copy

import marker_display

pestle_length = 0.108
base_link = "base_link"
pestle_link = "pestle_link"
tip_link = "tip_link"
mocap_topic = "/mocap/rigid_bodies/mortar/pose"
# mocap_topic = "/mocap/rigid_bodies/mortar_jig/pose"
node_name = "pestle_tf_broadcaster"


class publish_pose:
    def __init__(self):
        self.marker = marker_display.MarkerDisplay("marker_pub")
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    def breadcast_to_tf(self, msg):

        # pestle linkのtfを流す
        tf_pestle = geometry_msgs.msg.TransformStamped()
        tf_pestle.header.stamp = rospy.Time.now()
        tf_pestle.header.frame_id = base_link
        tf_pestle.child_frame_id = pestle_link
        tf_pestle.transform.translation = msg.pose.position
        tf_pestle.transform.rotation = msg.pose.orientation
        self.broadcaster.sendTransform(tf_pestle)

        # tip_linkのstatic_tfを流す
        tf_tip = geometry_msgs.msg.TransformStamped()
        tf_tip.header.stamp = tf_pestle.header.stamp
        tf_tip.header.frame_id = pestle_link
        tf_tip.child_frame_id = tip_link
        tf_tip.transform.translation.x = 0
        tf_tip.transform.translation.y = 0
        tf_tip.transform.translation.z = pestle_length
        tf_tip.transform.rotation.x = 0
        tf_tip.transform.rotation.y = 0
        tf_tip.transform.rotation.z = 0
        tf_tip.transform.rotation.w = 1
        self.static_broadcaster.sendTransform(tf_tip)

        # self.publish_marker(tf_tip)

        # rate = rospy.Rate(10)
        # rate.sleep()

    def publish_marker(self, tf):
        # self.tf_array = []
        tf_array = []
        marker = self.marker
        # インタラクティブマーカーを表示
        tf_array.append(tf)

        marker.display_tf(tf_array)
        # if len(tf_array) > 20:
        #     marker.display_tf(tf_array)
        # self.tf_array = copy.deepcopy(tf_array.pop(-1))
        # print(self.tf_array)


if __name__ == "__main__":
    publish_class = publish_pose()

    rospy.init_node(node_name, anonymous=True)
    rospy.Subscriber(mocap_topic, PoseStamped, publish_class.breadcast_to_tf)
    rospy.spin()
