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

    mortar_base_position = rospy.get_param("/loading_planning_scene/mortar_position")
    MasterSizer_position = rospy.get_param(
        "/loading_planning_scene/MasterSizer_position"
    )

    moveit = moveit_executor.MoveitExecutor("manipulator", "spoon_tip")
    r = Rotation.from_euler("xyz", [pi, pi / 2, pi / 2], degrees=False)
    quat = r.as_quat()
    MasterSizer_base_pose = list(MasterSizer_position.values()) + list(quat)
    pouring_pose = copy.deepcopy(MasterSizer_base_pose)
    pouring_pose[2] += 0.32

    tmp_pose = [0.16, 0, 0.35] + list(quat)

    waypoints = [tmp_pose, pouring_pose]
    debug_tf.broadcast_tf_with_waypoints(waypoints, "base_link")
    # moveit.execute_to_goal_pose(
    #     tmp_pose,
    #     ee_link="spoon_tip",
    #     vel_scale=0.2,
    #     acc_scale=0.2,
    #     execute=False,
    # )
    # moveit.execute_to_goal_pose(
    #     pouring_pose,
    #     ee_link="spoon_tip",
    #     vel_scale=0.1,
    #     acc_scale=0.1,
    #     execute=True,
    # )

    # execute pouring
    pose_list = []
    current_pose = _pose_stamped_to_list(
        moveit.move_group.get_current_pose("spoon_tip")
    )
    for i in range(10):
        delta_rot = Rotation.from_euler("xyz", np.array([0, 0, pi / 3 * 2]) / 10)
        current_rot = Rotation.from_quat(current_pose[3:])
        next_rot = current_rot * delta_rot
        current_position = current_pose[:3]
        next_pose = current_position + list(next_rot.as_quat())
        pose_list.append(next_pose)
        current_pose = next_pose
    moveit.execute_cartesian_path_by_waypoints(
        pose_list,
        "spoon_tip",
        vel_scale=0.2,
        acc_scale=0.2,
        execute=False,
    )


if __name__ == "__main__":
    main()
