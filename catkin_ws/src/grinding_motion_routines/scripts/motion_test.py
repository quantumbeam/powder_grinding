#!/usr/bin/env python3

import rospy
from grinding_motion_routines import (
    motion_generator,
    moveit_executor,
    JTC_executor,
    motion_primitive,
    marker_display,
    tf_publisher,
    load_planning_scene,
)


def main():
    rospy.init_node("mechano_grinding", anonymous=True)

    moveit = moveit_executor.MoveitExecutor("arm", "pestle_tip")
    current_pose = moveit.move_group.get_current_pose()
    print(current_pose)
    current_pose.pose.position.z += 0.03
    pose_list = [
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z,
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w,
    ]

    pose_list = [0.15, 0.15, 0.05, -0.68332, 0.71787, -0.067482, 0.11482]
    moveit.execute_to_goal_pose(
        pose_list,
        ee_link="pestle_tip",
        vel_scale=1,
        acc_scale=1,
    )


if __name__ == "__main__":
    main()
