#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Quaternion


class TFPublisher:
    def __init__(self) -> None:
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.tf = geometry_msgs.msg.TransformStamped()
        
        pass

    def broadcast_tf_with_waypoints(
        self, waypoints, parent_link="base_link", child_link="debug_"
    ):
        rate = rospy.Rate(10)
        for index, pose in enumerate(waypoints):
            pub_pose = Pose()
            pub_pose.position.x = pose[0]
            pub_pose.position.y = pose[1]
            pub_pose.position.z = pose[2]
            pub_pose.orientation = Quaternion(pose[3], pose[4], pose[5], pose[6])

            self.breadcast_tf(
                pub_pose.position,
                pub_pose.orientation,
                parent_link,
                child_link + str(index),
            )
            rate.sleep()

    def broadcast_tf_with_pose(self, pose, parent_link="base_link", child_link="debug"):
        pub_pose = Pose()
        pub_pose.position.x = pose[0]
        pub_pose.position.y = pose[1]
        pub_pose.position.z = pose[2]
        pub_pose.orientation = Quaternion(pose[3], pose[4], pose[5], pose[6])
        self.breadcast_tf(
            pub_pose.position, pub_pose.orientation, parent_link, child_link
        )

    def breadcast_tf(self, tf_pos, tf_rot, parent_link, child_link):
        self.tf.header.stamp = rospy.Time.now()
        self.tf.header.frame_id = parent_link
        self.tf.child_frame_id = child_link
        self.tf.transform.translation = tf_pos
        self.tf.transform.rotation = tf_rot
        self.broadcaster.sendTransform(self.tf)

    def listen_tf(self, child, parent):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
        rate.sleep()

        try:
            trans = tfBuffer.lookup_transform(child, parent, rospy.Time())
            return trans
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as err:
            rospy.loginfo("tf listen error%s" % err)
            return err
