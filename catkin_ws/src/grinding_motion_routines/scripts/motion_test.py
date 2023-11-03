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

    moveit = moveit_executor.MoveitExecutor("arm", "J6")
    current_pose = moveit.move_group.get_current_pose()

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
    moveit.execute_to_goal_pose(pose_list)


if __name__ == "__main__":
    main()
