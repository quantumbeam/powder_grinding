#!/usr/bin/env python3

# Python 2/3 compatibility imports
from hashlib import new
from logging import exception


import rospy
import geometry_msgs.msg
from rospy.exceptions import ROSException
from rospy.timer import sleep
import tf.transformations as tf
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
import numpy as np
import matplotlib.pyplot as plt
import copy
import time
import sys

try:
    from math import pi, tau, dist, fabs, cos, sin, sqrt
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt, sin

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


import roslib.packages

import moveit_interface
import motion_routines
import marker_display
import tf_publisher

from kek_vision.srv import PowderPos

from inputimeout import inputimeout, TimeoutOccurred


def main():
    ################### init params ###################
    MORTAR_HIGHT = 47 * 0.001

    MORTAR_POS = geometry_msgs.msg.Point()
    MORTAR_POS.x = 115.3 * 0.001
    MORTAR_POS.y = 120.3 * 0.001
    MORTAR_POS.z = 145 * 0.001  # mortar bottom

    MORTAR_SCALE = geometry_msgs.msg.Point()
    MORTAR_SCALE.x = 40 * 0.001
    MORTAR_SCALE.y = 40 * 0.001
    MORTAR_SCALE.z = (
        36 * 0.001
    )  # official infomation says 40*40*32 ,but z is more deep actually

    TIMEOUT_SEC = 0.1

    # GRINDING params
    GRINDING_RADIUS_MAX = 13
    GRINDING_RADIUS = 10

    GRINDING_ANGLE_PARAM = 1
    GRINDING_YAW_ANGLE = pi
    GRINDING_WAYPOINTS_COUNTS = 50
    GRINDING_VEL_SCALE = 0.9
    GRINDING_ACC_SCALE = 0.9

    GRINDING_NUMBER_OF_ROTATION = 10
    MOTION_REPEAT_COUNTS = 1

    GRINDING_RZ_BEGINING = 35.5
    GRINDING_RZ_END = 35.5

    # GRINDING_RADIUS = 10
    # GRINDING_POS_BEGINING = [-GRINDING_RADIUS, -GRINDING_RADIUS]
    # GRINDING_POS_END = [-GRINDING_RADIUS, -GRINDING_RADIUS + 0.0001]

    # gathering params
    GATHERING_RZ_BEGINING = 45
    GATHERING_RZ_END = 33
    GATHERING_RADIUS_END = 10
    GATHERING_RADIUS_BEGINING = 37
    GATHERING_WAYPOINTS_COUNTS = 5
    GATHERING_MOTION_COUNTS = 6
    GATHERING_YAW_ANGLE = 0
    GATHERING_ANGLE_PARAM = 0
    GATHERING_FIX_ANGLE = True
    GATHERING_VEL_SCALE = 0.3
    GATHERING_ACC_SCALE = 0.3

    args = sys.argv
    if len(args) < 3:
        print("Arguments are too short")
        exit()
    grinding_type = args[1]
    grinding_counts = args[2]
    whole_motion_counts = args[3]

    if grinding_type == "circle":
        GRINDING_BEGINING_POS = [-GRINDING_RADIUS, -GRINDING_RADIUS]
        GRINDING_END_POS = [-GRINDING_RADIUS, -GRINDING_RADIUS + 0.0001]
    elif grinding_type == "spiral":
        GRINDING_BEGINING_POS = [0, 0]
        GRINDING_END_POS = [-GRINDING_RADIUS, -GRINDING_RADIUS]
    else:
        print(
            "'circle' or 'spiral' is available at the grinding type on the first argument"
        )
        exit()

    if grinding_counts.isdecimal() == False:
        print(
            "A decimal number is available at the grinding counts on the secound argument"
        )
        exit()
    grinding_counts = int(grinding_counts)

    if whole_motion_counts.isdecimal() == False:
        print(
            "A decimal number is available at the whole motion counts on the third argument"
        )
        exit()
    whole_motion_counts = int(whole_motion_counts)

    rospy.loginfo(
        "Grinding motion is %s, motion counts of grinding and gathering is %d and 1."
        % (grinding_type, grinding_counts)
    )

    ################### init class ###################
    cobotta_motion = motion_routines.MotionRoutine(
        "arm", "pestle_tip", MORTAR_POS, MORTAR_HIGHT, MORTAR_SCALE
    )

    my_marker = marker_display.MarkerDisplay("marker_pub")
    tf_pub = tf_publisher.TFPublisher()

    ###### goto init pose
    init_pose = cobotta_motion.move_group.get_current_pose()
    init_pose.pose.position.z += 0.05
    cobotta_motion.go_to_pose(init_pose, vel_scale=0.5, acc_scale=0.5)

    ################### init planning scene ###################
    work_mesh_dir = roslib.packages.get_pkg_dir("kek_scene_description") + "/mesh/"
    mortar_pose = geometry_msgs.msg.PoseStamped()
    mortar_pose.header.frame_id = cobotta_motion.planning_frame
    mortar_pose.pose.position = MORTAR_POS
    cobotta_motion.scene.add_mesh(
        "mortar_35mm", mortar_pose, work_mesh_dir + "mortar_35mm.stl"
    )

    ################### create waypoints ###################

    cobotta_motion.create_grinding_waypoints(
        begining_position=GRINDING_BEGINING_POS,
        end_position=GRINDING_END_POS,
        begining_radious_z=GRINDING_RZ_BEGINING,
        end_radious_z=GRINDING_RZ_END,
        angle_param=GRINDING_ANGLE_PARAM,
        yaw_angle=GRINDING_YAW_ANGLE,
        number_of_rotations=GRINDING_NUMBER_OF_ROTATION,
        waypoints_count=GRINDING_WAYPOINTS_COUNTS,
        debug=False,
    )

    cobotta_motion.create_gathering_waypoints_list(
        begining_theta=0,
        end_tehta=pi * 2,
        begining_length_from_center=GATHERING_RADIUS_BEGINING,
        end_length_from_center=GATHERING_RADIUS_END,
        begining_radius_z=GATHERING_RZ_BEGINING,
        end_radius_z=GATHERING_RZ_END,
        waypoints_count=GATHERING_WAYPOINTS_COUNTS,
        gathering_motion_count=GATHERING_MOTION_COUNTS,
        angle_param=GATHERING_ANGLE_PARAM,
        fix_angle=GATHERING_FIX_ANGLE,
        yaw_angle=GATHERING_YAW_ANGLE,
        debug=False,
    )

    try:
        for counts in range(whole_motion_counts):
            cobotta_motion.execute_grinding_motion_routine(
                "pestle_tip",
                velocity_scale=GRINDING_VEL_SCALE,
                acceleration_scale=GRINDING_ACC_SCALE,
                repeat_counts=grinding_counts,
            )
            cobotta_motion.execute_gathering_motion_routine(
                "spatula_tip",
                velocity_scale=GATHERING_VEL_SCALE,
                acceleration_scale=GATHERING_ACC_SCALE,
            )

    except rospy.ROSInterruptException as err:
        rospy.loginfo(err)
        exit()
    except KeyboardInterrupt as err:
        rospy.loginfo(err)
        exit()


if __name__ == "__main__":
    main()
