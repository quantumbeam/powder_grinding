#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseArray, Pose, Quaternion
import tf


class PoseArrayPublisher:
    def __init__(self,pose_array_publisher_name="/debug_poses"):
        self.pose_array_pub = rospy.Publisher(pose_array_publisher_name, PoseArray, queue_size=10)
        
        pass

    def publish_pose_array_with_waypoints(
        self, waypoints, frame_id="base_link"
    ):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = frame_id
        
        for pose in waypoints:
            pub_pose = Pose()
            pub_pose.position.x = pose[0]
            pub_pose.position.y = pose[1]
            pub_pose.position.z = pose[2]
            pub_pose.orientation = Quaternion(pose[3], pose[4], pose[5], pose[6])
            pose_array.poses.append(pub_pose)
        
        self.pose_array_pub.publish(pose_array)

    def publish_pose_array_with_pose(self, pose, frame_id="base_link"):
        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = frame_id
        
        pub_pose = Pose()
        pub_pose.position.x = pose[0]
        pub_pose.position.y = pose[1]
        pub_pose.position.z = pose[2]
        pub_pose.orientation = Quaternion(pose[3], pose[4], pose[5], pose[6])
        pose_array.poses.append(pub_pose)
        
        self.pose_array_pub.publish(pose_array)

    def listen_tf(self, child, parent):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        rate.sleep()

        try:
            (trans, rot) = listener.lookupTransform(child, parent, rospy.Time(0))
            return (trans, rot)
        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as err:
            rospy.loginfo("tf listen error%s" % err)
            return err