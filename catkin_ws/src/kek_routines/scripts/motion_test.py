#!/usr/bin/env python3

import rospy
from kek_routines import moveit_interface, motion_routines, marker_display, tf_publisher
import geometry_msgs.msg


def main():
    rospy.init_node("mechano_grinding", anonymous=True)

    moveit = moveit_interface.MoveitInterface("manipulator", "pestle_tip")
    current_pose = moveit.move_group.get_current_pose()

    current_pose.pose.position.z -= 0.01
    moveit.move_PTP(current_pose)


if __name__ == "__main__":
    main()
