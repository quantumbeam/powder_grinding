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
import datetime

import pandas as pd

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

from grinding_vision.srv import PowderPos

from inputimeout import inputimeout, TimeoutOccurred


################### Fixed params ###################
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


MORTAR_RADIUS = 40

TIMEOUT_SEC = 0.1

# experiments params
# for visula feedback
EXPERIMENTS_TIME_MINUTE = (
    71.4
    # 71.4 circle G #75.3 circle G10 G1   # 111 circle G10 G10 # 35.4 spiral G
)
last_motion = "grinding"  # "grinding"  or "gathering"
# POWDER_MOVING_THRESHOLD = 1  # mm

# for all experiments
EXPERIMENTS_MOTION_COUNTS = 60
GG_EXPERIMENTS_GRINDING_COUNTS = 1
PESTLE_RADIUS_MARGIN = 0
motion_type = sys.argv[1]
if motion_type == "VF" or motion_type == "VFG":
    GRINDING_VEL_SCALE = 0.8
    GRINDING_ACC_SCALE = 0.8
else:
    GRINDING_VEL_SCALE = 0.9
    GRINDING_ACC_SCALE = 0.9
print("vel", GRINDING_VEL_SCALE)

# GRINDING params
GRINDING_RADIUS_MAX = 10  # up to 13 is available but protective stop may occour
GRINDING_RADIUS_MIN = 2
GRINDING_ANGLE_PARAM = 1
GRINDING_YAW_ANGLE = pi
GRINDING_WAYPOINTS_COUNTS = 50
GRINDING_NUMBER_OF_ROTATION = 10


GRINDING_RZ_BEGINING = 35.5
GRINDING_RZ_END = 35.5

DEFAULT_GRINDING_RADIUS = 10
GRINDING_POS_BEGINING = [-DEFAULT_GRINDING_RADIUS, -DEFAULT_GRINDING_RADIUS]
GRINDING_POS_END = [-DEFAULT_GRINDING_RADIUS, -DEFAULT_GRINDING_RADIUS + 0.0001]

print(
    "Whole grinding rotations in trial:",
    GRINDING_NUMBER_OF_ROTATION * EXPERIMENTS_MOTION_COUNTS,
)
args = sys.argv
if len(args) > 2:
    grinding_type = args[2]
    if grinding_type == "circle":
        GRINDING_POS_BEGINING = [-DEFAULT_GRINDING_RADIUS, -DEFAULT_GRINDING_RADIUS]
        GRINDING_POS_END = [-DEFAULT_GRINDING_RADIUS, -DEFAULT_GRINDING_RADIUS + 0.0001]
        rospy.loginfo("Circle motion is selected as grinding type")
    elif grinding_type == "spiral":
        GRINDING_POS_BEGINING = [0, 0]
        GRINDING_POS_END = [-DEFAULT_GRINDING_RADIUS, -DEFAULT_GRINDING_RADIUS]
        rospy.loginfo("Spiral motion is selected as grinding type")
    else:
        rospy.loginfo("The secound argument is 'circle' or 'spiral'")
        exit()
else:
    rospy.loginfo(
        "Default circle motion is selected as grinding type (you can choose 'circle' or 'spiral' on the secound argument)"
    )

# gathering params
GATHERING_RZ_BEGINING = 45
GATHERING_RZ_END = 30
GATHERING_RADIUS_END = 10
GATHERING_RADIUS_BEGINING = 37
GATHERING_WAYPOINTS_COUNTS = 5
GATHERING_MOTION_COUNTS = 6
GATHERING_YAW_ANGLE = 0
GATHERING_ANGLE_PARAM = 0
GATHERING_FIX_ANGLE = True
GATHERING_VEL_SCALE = 0.2
GATHERING_ACC_SCALE = 0.2

# VF params
RADIUS_THRESHOLD_OF_GATHERING = GRINDING_RADIUS_MAX
debug_data_dir_path = "/root/ocla/catkin_ws/src/grinding_vision/"
recursive_counts = 0
# debug class
my_marker = marker_display.MarkerDisplay("marker_pub")
tf_pub = tf_publisher.TFPublisher()


def execute_grinding(motion_routines_class):
    motion_routines_class.create_grinding_waypoints(
        begining_position=GRINDING_POS_BEGINING,
        end_position=GRINDING_POS_END,
        begining_radious_z=GRINDING_RZ_BEGINING,
        end_radious_z=GRINDING_RZ_END,
        angle_param=GRINDING_ANGLE_PARAM,
        yaw_angle=GRINDING_YAW_ANGLE,
        number_of_rotations=GRINDING_NUMBER_OF_ROTATION,
        waypoints_count=GRINDING_WAYPOINTS_COUNTS,
        debug=False,
    )
    motion_routines_class.execute_grinding_motion_routine(
        "pestle_tip",
        velocity_scale=GRINDING_VEL_SCALE,
        acceleration_scale=GRINDING_ACC_SCALE,
    )


def execute_gathering(motion_routines_class):
    motion_routines_class.create_gathering_waypoints_list(
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

    motion_routines_class.execute_gathering_motion_routine(
        "spatula_tip",
        velocity_scale=GATHERING_VEL_SCALE,
        acceleration_scale=GATHERING_ACC_SCALE,
    )


def grinding_with_visual_feedback(
    motion_routines_class, vision_server, last_motion, last_powder_center
):
    global recursive_counts
    ################### vision feedback ###################
    while True:
        powder_info = vision_server("run", MORTAR_RADIUS)
        ######### grinding radius
        if powder_info.radius > DEFAULT_GRINDING_RADIUS:
            grinding_radius = DEFAULT_GRINDING_RADIUS
        else:
            grinding_radius = powder_info.radius
        powder_center = [
            powder_info.center_x,
            powder_info.center_y,
        ]
        grinding_distance_from_origin = (
            np.sqrt(powder_center[0] ** 2 + powder_center[1] ** 2) + grinding_radius
        )
        if grinding_distance_from_origin > GRINDING_RADIUS_MAX:
            over_rad = grinding_distance_from_origin - GRINDING_RADIUS_MAX
            grinding_radius -= over_rad
            rospy.loginfo("Grinding position is over GRINDING_RADIUS_MAX")
            rospy.loginfo(
                "Fixed radius %s to %s"
                % (str(grinding_radius + over_rad), str(grinding_radius))
            )
        ######### grinding center
        grinding_center = powder_center

        ######### dicision param
        center_distance_from_origin = np.sqrt(
            powder_center[0] ** 2 + powder_center[1] ** 2
        )
        rospy.loginfo("Center distance is %f" % round(center_distance_from_origin, 2))

        ######### detection of powder moving
        # if last_powder_center[0] is None or last_powder_center[1] is None:
        #     difference_of_center_distance = POWDER_MOVING_THRESHOLD + 1
        # else:
        #     last_center_distance_from_origin = np.sqrt(
        #         last_powder_center[0] ** 2 + last_powder_center[1] ** 2
        #     )
        #     difference_of_center_distance = fabs(
        #         last_center_distance_from_origin - center_distance_from_origin
        #     )
        # rospy.loginfo(
        #     "Difference of center distance is %f"
        #     % round(difference_of_center_distance, 2)
        # )
        # print("last_powder_center", last_powder_center)

        ######### recursive function for appropriate vision
        if (
            fabs(powder_center[0]) > MORTAR_RADIUS
            or fabs(powder_center[1]) > MORTAR_RADIUS
            # or fabs(grinding_radius) > MORTAR_RADIUS
        ):
            rospy.loginfo("Recursive vision service")
            rospy.sleep(5.0)
            recursive_counts += 1
            if recursive_counts >= 5:
                rospy.logerr("5 times recursive vision detected, exit now")
                exit()
        else:
            recursive_counts = 0
            break

    if center_distance_from_origin > (GRINDING_RADIUS_MAX + PESTLE_RADIUS_MARGIN):
        grinding_radius = DEFAULT_GRINDING_RADIUS
        grinding_center = [0, 0]
        center_distance_from_origin = 0

    ################### motion ###################
    # and difference_of_center_distance > POWDER_MOVING_THRESHOLD
    rospy.loginfo("Decide grinding motion!")
    rospy.loginfo("Grinding center is %s" % str(grinding_center))
    rospy.loginfo("Grinding radius is %f" % round(grinding_radius, 2))
    args = sys.argv
    if len(args) > 2 and args[2] == "spiral":
        begining_pos = [0, 0]
        end_pos = [-grinding_radius, -grinding_radius]
    else:
        begining_pos = [-grinding_radius, -grinding_radius]
        end_pos = [-grinding_radius, -grinding_radius + 0.0001]
    motion_routines_class.create_grinding_waypoints(
        begining_position=copy.deepcopy(begining_pos),
        end_position=copy.deepcopy(end_pos),
        begining_radious_z=GRINDING_RZ_BEGINING,
        end_radious_z=GRINDING_RZ_END,
        angle_param=GRINDING_ANGLE_PARAM,
        yaw_angle=GRINDING_YAW_ANGLE,
        number_of_rotations=GRINDING_NUMBER_OF_ROTATION,
        waypoints_count=GRINDING_WAYPOINTS_COUNTS,
        center_position=copy.deepcopy(grinding_center),
        debug=False,
    )

    success = motion_routines_class.execute_grinding_motion_routine(
        "pestle_tip",
        velocity_scale=GRINDING_VEL_SCALE,
        acceleration_scale=GRINDING_ACC_SCALE,
    )
    print("success", success)
    if success == True:
        return "grinding", grinding_radius, grinding_center, [None, None]
    else:
        return last_motion, None, [None, None], [None, None]


def grinding_and_gathering_with_visual_feedback(
    motion_routines_class, vision_server, last_motion, last_powder_center
):
    global recursive_counts
    ################### vision feedback ###################
    if last_motion == "grinding":
        while True:
            powder_info = vision_server("run", MORTAR_RADIUS)
            ######### grinding radius
            if powder_info.radius > DEFAULT_GRINDING_RADIUS:
                grinding_radius = DEFAULT_GRINDING_RADIUS
            else:
                grinding_radius = powder_info.radius
            powder_center = [
                powder_info.center_x,
                powder_info.center_y,
            ]
            grinding_distance_from_origin = (
                np.sqrt(powder_center[0] ** 2 + powder_center[1] ** 2) + grinding_radius
            )
            if grinding_distance_from_origin > GRINDING_RADIUS_MAX:
                over_rad = grinding_distance_from_origin - GRINDING_RADIUS_MAX
                grinding_radius -= over_rad
                rospy.loginfo("Grinding position is over GRINDING_RADIUS_MAX")
                rospy.loginfo(
                    "Fixed radius %s to %s"
                    % (str(grinding_radius + over_rad), str(grinding_radius))
                )
            ######### grinding center
            grinding_center = powder_center

            ######### fixed grinding params
            grinding_radius = DEFAULT_GRINDING_RADIUS
            grinding_center = [0, 0]

            ######### dicision param
            center_distance_from_origin = np.sqrt(
                powder_center[0] ** 2 + powder_center[1] ** 2
            )
            rospy.loginfo(
                "Center distance is %f" % round(center_distance_from_origin, 2)
            )

            ######### detection of powder moving
            # if last_powder_center[0] is None or last_powder_center[1] is None:
            #     difference_of_center_distance = POWDER_MOVING_THRESHOLD + 1
            # else:
            #     last_center_distance_from_origin = np.sqrt(
            #         last_powder_center[0] ** 2 + last_powder_center[1] ** 2
            #     )
            #     difference_of_center_distance = fabs(
            #         last_center_distance_from_origin - center_distance_from_origin
            #     )
            # rospy.loginfo(
            #     "Difference of center distance is %f"
            #     % round(difference_of_center_distance, 2)
            # )
            # print("last_powder_center", last_powder_center)

            ######### recursive function for appropriate vision
            if (
                fabs(powder_center[0]) > MORTAR_RADIUS
                or fabs(powder_center[1]) > MORTAR_RADIUS
                # or fabs(grinding_radius) > MORTAR_RADIUS
            ):
                rospy.loginfo("Recursive vision service")
                rospy.sleep(5.0)
                recursive_counts += 1
                if recursive_counts >= 5:
                    rospy.logerr("5 times recursive vision detected, exit now")
                    exit()
            else:
                recursive_counts = 0
                break

    elif last_motion == "gathering":
        grinding_radius = DEFAULT_GRINDING_RADIUS
        grinding_center = [0, 0]
        center_distance_from_origin = 0
        # difference_of_center_distance = POWDER_MOVING_THRESHOLD + 1
    else:
        rospy.loginfo("last motion faild")
        exit()

    ################### motion ###################
    # and difference_of_center_distance > POWDER_MOVING_THRESHOLD
    if (
        center_distance_from_origin < (GRINDING_RADIUS_MAX + PESTLE_RADIUS_MARGIN)
        or last_motion == "gathering"
    ):
        rospy.loginfo("Decide grinding motion!")
        rospy.loginfo("Grinding center is %s" % str(grinding_center))
        rospy.loginfo("Grinding radius is %f" % round(grinding_radius, 2))
        args = sys.argv
        if len(args) > 2 and args[2] == "spiral":
            begining_pos = [0, 0]
            end_pos = [-grinding_radius, -grinding_radius]
        else:
            begining_pos = [-grinding_radius, -grinding_radius]
            end_pos = [-grinding_radius, -grinding_radius + 0.0001]
        motion_routines_class.create_grinding_waypoints(
            begining_position=copy.deepcopy(begining_pos),
            end_position=copy.deepcopy(end_pos),
            begining_radious_z=GRINDING_RZ_BEGINING,
            end_radious_z=GRINDING_RZ_END,
            angle_param=GRINDING_ANGLE_PARAM,
            yaw_angle=GRINDING_YAW_ANGLE,
            number_of_rotations=GRINDING_NUMBER_OF_ROTATION,
            waypoints_count=GRINDING_WAYPOINTS_COUNTS,
            center_position=copy.deepcopy(grinding_center),
            debug=False,
        )

        success = motion_routines_class.execute_grinding_motion_routine(
            "pestle_tip",
            velocity_scale=GRINDING_VEL_SCALE,
            acceleration_scale=GRINDING_ACC_SCALE,
        )
        print("success", success)
        if success == True:
            return "grinding", grinding_radius, grinding_center, [None, None]
        else:
            return last_motion, None, [None, None], [None, None]

    else:
        rospy.loginfo("Decide gathering motion!")
        motion_routines_class.create_gathering_waypoints_list(
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
        success = motion_routines_class.execute_gathering_motion_routine(
            "spatula_tip",
            velocity_scale=GATHERING_VEL_SCALE,
            acceleration_scale=GATHERING_ACC_SCALE,
        )
        print("success", success)
        if success == True:
            return "gathering", "", ["", ""], powder_center
        else:
            return last_motion, None, [None, None], powder_center


def main():
    global last_motion
    ################### init class ###################
    motion_routines_class = motion_routines.MotionRoutine(
        "arm", "pestle_tip", MORTAR_POS, MORTAR_HIGHT, MORTAR_SCALE
    )

    ################### select motion type ###################
    args = sys.argv
    if len(args) < 2:
        rospy.loginfo("Arguments are too short")
        exit()
    motion_type = args[1]
    if motion_type == "campose":
        motion_routines_class.goto_camera_pose()
        exit()
    elif motion_type == "G":
        rospy.loginfo("Selected grinding only")
    elif motion_type == "G2":
        rospy.loginfo("Selected gathering only")
    elif motion_type == "GG" or motion_type == "GGtime":
        rospy.loginfo(
            "Selected grinding %d and gathering 1" % GG_EXPERIMENTS_GRINDING_COUNTS
        )

    elif (
        motion_type == "VF"
        or motion_type == "VFG"
        or motion_type == "Gcam"
        or motion_type == "GGcam"
    ):
        rospy.loginfo("Selected grinding and gathering with visual feedback")
        ###### init vision service
        rospy.loginfo("Wait for grinding_vision service")
        try:
            rospy.wait_for_service("/powder_pos", timeout=3.0)
            vision_server = rospy.ServiceProxy(
                "/powder_pos", PowderPos, persistent=True
            )
        except ROSException:
            rospy.logerr("timeout grinding_vision service")
            time.sleep(1)
            exit()

    else:
        rospy.logerr("The motion type on the first argument is wrong")
        exit()

    ################### goto init pose ###################
    init_pose = motion_routines_class.move_group.get_current_pose()
    init_pose.pose.position.z += 0.05
    motion_routines_class.go_to_pose(init_pose, vel_scale=0.5, acc_scale=0.5)

    ################### init planning scene ###################
    work_mesh_dir = roslib.packages.get_pkg_dir("grinding_descriptions") + "/mesh/"
    mortar_pose = geometry_msgs.msg.PoseStamped()
    mortar_pose.header.frame_id = motion_routines_class.planning_frame
    mortar_pose.pose.position = MORTAR_POS
    motion_routines_class.scene.add_mesh(
        "mortar_35mm", mortar_pose, work_mesh_dir + "mortar_35mm.stl"
    )

    #### mortar pos calibration
    # rospy.loginfo("goto mortar pos calibration pose and exit automatically")
    # calib_pose = geometry_msgs.msg.PoseStamped()
    # calib_pose.header.frame_id = motion_routines_class.planning_frame
    # calib_pose.pose.position = MORTAR_POS
    # calib_pose.pose.position.z += MORTAR_HIGHT
    # q = tf.quaternion_from_euler(pi, 0, pi)
    # calib_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    # motion_routines_class.go_to_pose(calib_pose, vel_scale=0.2, acc_scale=0.2)

    motion_counts = 0
    init_time = time.time()
    experiment_time_list = []
    VF_motion_list = []
    grinding_radius_list = []
    grinding_center_x_list = []
    grinding_center_y_list = []
    powder_center_x_list = []
    powder_center_y_list = []

    grinding_radius = DEFAULT_GRINDING_RADIUS
    grinding_center = [None, None]
    last_powder_center = [None, None]
    while True:
        try:
            key = inputimeout(
                prompt="If you want to finish, Prease put q.(timeout = %s s)\n"
                % str(TIMEOUT_SEC),
                timeout=TIMEOUT_SEC,
            )

            if "q" in key:
                exit()

        except TimeoutOccurred:
            if motion_type == "G":
                execute_grinding(motion_routines_class)
            elif motion_type == "G2":
                execute_gathering(motion_routines_class)
            elif motion_type == "GG" or motion_type == "GGtime":
                if last_motion == "gathering":
                    for counts in range(GG_EXPERIMENTS_GRINDING_COUNTS):
                        execute_grinding(motion_routines_class)
                    last_motion = "grinding"
                elif last_motion == "grinding":
                    execute_gathering(motion_routines_class)
                    last_motion = "gathering"
            elif motion_type == "VF":
                if last_motion == "grinding":
                    motion_routines_class.goto_camera_pose("pestle_tip")
                (
                    last_motion,
                    grinding_radius,
                    grinding_center,
                    last_powder_center,
                ) = grinding_and_gathering_with_visual_feedback(
                    motion_routines_class,
                    vision_server,
                    last_motion,
                    last_powder_center,
                )
            elif motion_type == "VFG":
                motion_routines_class.goto_camera_pose("pestle_tip")
                (
                    last_motion,
                    grinding_radius,
                    grinding_center,
                    last_powder_center,
                ) = grinding_with_visual_feedback(
                    motion_routines_class,
                    vision_server,
                    last_motion,
                    last_powder_center,
                )
            elif motion_type == "Gcam":
                motion_routines_class.goto_camera_pose("pestle_tip")
                vision_server("run")
                execute_grinding(motion_routines_class)
            elif motion_type == "GGcam":
                if last_motion == "gathering":
                    motion_routines_class.goto_camera_pose("pestle_tip")
                    vision_server("run")
                    execute_grinding(motion_routines_class)
                    last_motion = "grinding"
                elif last_motion == "grinding":
                    motion_routines_class.goto_camera_pose("pestle_tip")
                    vision_server("run")
                    execute_gathering(motion_routines_class)
                    last_motion = "gathering"

            # experiments info and exit
            print("last_motion: ", last_motion)
            current_experiment_time = time.time() - init_time
            experiment_time_list.append(current_experiment_time)
            if motion_type == "VF" or motion_type == "VFG" or motion_type == "GGtime":
                print(
                    "Current experiment time (min):",
                    round(current_experiment_time / 60, 2),
                    "/",
                    EXPERIMENTS_TIME_MINUTE,
                )
                VF_motion_list.append(last_motion)
                grinding_radius_list.append(grinding_radius)
                grinding_center_x_list.append(grinding_center[0])
                grinding_center_y_list.append(grinding_center[1])
                powder_center_x_list.append(last_powder_center[0])
                powder_center_y_list.append(last_powder_center[1])
                df = pd.DataFrame(
                    np.array(
                        [
                            experiment_time_list,
                            VF_motion_list,
                            grinding_radius_list,
                            grinding_center_x_list,
                            grinding_center_y_list,
                            powder_center_x_list,
                            powder_center_y_list,
                        ]
                    ).T
                )
                df.to_csv(
                    debug_data_dir_path + "%s_VF_motion_log.csv" % init_time,
                    header=False,
                    index=False,
                )
                if current_experiment_time > EXPERIMENTS_TIME_MINUTE * 60:
                    print("Finished trial")
                    exit()
            else:
                motion_counts += 1
                print(
                    "Current experiments counts:",
                    motion_counts,
                    "/",
                    EXPERIMENTS_MOTION_COUNTS,
                )
                print(
                    "Total experiment time (min):",
                    round(current_experiment_time / 60, 5),
                )
                if motion_counts >= EXPERIMENTS_MOTION_COUNTS:
                    exit()

        except rospy.ROSInterruptException as err:
            rospy.loginfo(err)
            exit()
        except KeyboardInterrupt as err:
            rospy.loginfo(err)
            exit()


if __name__ == "__main__":
    main()
