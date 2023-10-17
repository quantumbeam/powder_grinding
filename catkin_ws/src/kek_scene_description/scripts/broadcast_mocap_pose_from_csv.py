#!/usr/bin/env python3
import rospy
from rospy.rostime import Duration
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped

import pandas as pd

base_link = "base_link"
pestle_link = "pestle_link"
tip_link = "tip_link"
node_name = "tf_broadcaster"


def breadcast_tf(x, y, z, qx, qy, qz, w, parent_link, child_link):

    # pestle linkのtfを流す
    broadcaster = tf2_ros.TransformBroadcaster()
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = parent_link
    tf.child_frame_id = child_link
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz
    tf.transform.rotation.w = w
    broadcaster.sendTransform(tf)


if __name__ == "__main__":
    rospy.init_node(node_name, anonymous=True)
    rate = rospy.Rate(100)

    file_path = "/root/ocla/catkin_ws/bagfiles/osx_experiments/211027_osx_experiment/csv/extract_motion_211027_nacl_diagonally_camera_liner_corr_0.6_split_0.3_sec.csv"
    df_pose = pd.read_csv(file_path)

    for index, row in df_pose.iterrows():
        try:
            x = row["field.pose.position.x"]
            y = row["field.pose.position.y"]
            z = row["field.pose.position.z"]
            qx = row["field.pose.orientation.x"]
            qy = row["field.pose.orientation.y"]
            qz = row["field.pose.orientation.z"]
            w = row["field.pose.orientation.w"]
            breadcast_tf(x, y, z, qx, qy, qz, w, base_link, pestle_link)

            x = row["field.pose.tip_pos.x"]
            y = row["field.pose.tip_pos.y"]
            z = row["field.pose.tip_pos.z"]
            qx = row["field.pose.orientation.x"]
            qy = row["field.pose.orientation.y"]
            qz = row["field.pose.orientation.z"]
            w = row["field.pose.orientation.w"]
            breadcast_tf(x, y, z, qx, qy, qz, w, base_link, tip_link)

            rate.sleep()

        except:
            break
