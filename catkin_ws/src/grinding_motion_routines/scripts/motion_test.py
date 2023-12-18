#!/usr/bin/env python3

import rospy
import copy
from scipy.spatial.transform import Rotation
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose
from grinding_motion_routines import (
    motion_generator,
    moveit_executor,
    JTC_executor,
    motion_primitive,
    marker_display,
    tf_publisher,
    load_planning_scene,
)


def _pose_stamped_to_list(pose_msg):
    return [
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        pose_msg.pose.position.z,
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w,
    ]


def main():
    rospy.init_node("mechano_grinding", anonymous=True)
    debug_tf = tf_publisher.TFPublisher()
    pi = np.pi


    moveit = moveit_executor.MoveitExecutor("manipulator", "tool0")
    r = Rotation.from_euler("xyz", [pi, pi / 2, pi / 2], degrees=False)
    quat = r.as_quat()

    tmp_pose = [0.16, 0, 0.35] + list(quat)

    waypoints = [tmp_pose]
    debug_tf.broadcast_tf_with_waypoints(waypoints, "base_link")
    moveit.execute_to_goal_pose(
        tmp_pose,
        ee_link="tool0",
        vel_scale=0.2,
        acc_scale=0.2,
        execute=True,
    )
    # moveit.execute_to_goal_pose(
    #     pouring_pose,
    #     ee_link="spoon_tip",
    #     vel_scale=0.1,
    #     acc_scale=0.1,
    #     execute=True,
    # )



if __name__ == "__main__":
    main()
