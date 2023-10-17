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

    # GRINDING params
    GRINDING_RADIUS_MAX = 13
    GRINDING_ANGLE_PARAM = 1
    GRINDING_YAW_ANGLE = pi
    GRINDING_WAYPOINTS_COUNT = 50
    GRINDING_VEL_SCALE = 0.9
    GRINDING_ACC_SCALE = 0.9

    GRINDING_NUMBER_OF_ROTATION = 10
    MOTION_REPEAT_COUNTS = 60

    args = sys.argv
    if len(args) < 2:
        print("Arguments are too short")
        exit()
    if args[1] == "circle":
        GRINDING_RADIUS = 10
        GRINDING_RZ_BEGINING = 35.5
        GRINDING_RZ_END = 35.5
        GRINDING_BEGINING_POS = [-GRINDING_RADIUS, -GRINDING_RADIUS]
        GRINDING_END_POS = [-GRINDING_RADIUS, -GRINDING_RADIUS + 0.0001]
    elif args[1] == "spiral":
        GRINDING_RADIUS = 10
        GRINDING_RZ_BEGINING = 35.5
        GRINDING_RZ_END = 35.5
        GRINDING_BEGINING_POS = [0, 0]
        GRINDING_END_POS = [-GRINDING_RADIUS, -GRINDING_RADIUS]
    else:
        print("First argument is 'circle' or 'spiral'")
        exit()

    # gathering params
    GATHERING_RZ_BEGINING = 45  # 36
    GATHERING_RZ_END = 37  # 35
    GATHERING_END_RADIUS = 12
    GATHERING_MOTION_RADIUS_MAX = 36
    GATHERING_VEL_SCALE = 0.2
    GATHERING_ACC_SCALE = 0.2

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

    #### add mortar in RViz
    mortar_pose = geometry_msgs.msg.PoseStamped()
    mortar_pose.header.frame_id = cobotta_motion.planning_frame
    mortar_pose.pose.position = MORTAR_POS
    cobotta_motion.scene.add_mesh(
        "mortar_35mm", mortar_pose, work_mesh_dir + "mortar_35mm.stl"
    )

    #### mortar pos calibration
    rospy.loginfo("goto mortar pos calibration pose and exit automatically")
    calib_pose = geometry_msgs.msg.PoseStamped()
    calib_pose.header.frame_id = cobotta_motion.planning_frame
    calib_pose.pose.position = MORTAR_POS
    calib_pose.pose.position.z += MORTAR_HIGHT
    q = tf.quaternion_from_euler(pi, 0, -pi)
    calib_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    # cobotta_motion.go_to_pose(calib_pose, "pestle_tip", vel_scale=0.2, acc_scale=0.2)
    # calib_pose.pose.position.z -= MORTAR_SCALE.z
    # cobotta_motion.go_to_pose(calib_pose, "pestle_tip", vel_scale=0.2, acc_scale=0.2)
    # exit()

    waypoints = cobotta_motion.create_grinding_waypoints(
        begining_position=GRINDING_BEGINING_POS,
        end_position=GRINDING_END_POS,
        begining_radious_z=GRINDING_RZ_BEGINING,
        end_radious_z=GRINDING_RZ_END,
        angle_param=GRINDING_ANGLE_PARAM,
        yaw_angle=GRINDING_YAW_ANGLE,
        number_of_rotations=GRINDING_NUMBER_OF_ROTATION,
        waypoints_count=GRINDING_WAYPOINTS_COUNT,
        debug=False,
    )
    cobotta_motion.execute_grinding_motion_routine(
        eef_link="pestle_tip",
        velocity_scale=GRINDING_VEL_SCALE,
        acceleration_scale=GRINDING_ACC_SCALE,
        repeat_counts=MOTION_REPEAT_COUNTS,
    )


if __name__ == "__main__":
    main()
