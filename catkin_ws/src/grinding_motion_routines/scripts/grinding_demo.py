#!/usr/bin/env python3

# Python 2/3 compatibility imports
from hashlib import new
from logging import exception


import rospy
import geometry_msgs.msg
import tf.transformations as tf
from geometry_msgs.msg import Quaternion
import time

from inputimeout import inputimeout, TimeoutOccurred

try:
    from math import pi, tau, dist, fabs, cos, sin, sqrt
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt, sin

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


import roslib.packages


from grinding_motion_routines import (
    moveit_interface,
    motion_routines,
    marker_display,
    tf_publisher,
)
from kek_vision.srv import PowderPos


################### Fixed params ###################


# experiments params
TIMEOUT_SEC = 0.1

# debug class
debug_marker = marker_display.MarkerDisplay("debug_marker")
debug_tf = tf_publisher.TFPublisher()


def execute_grinding(motion_routines_class, eef_link, execute=True):
    waypoints = motion_routines_class.create_circular_waypoints(
        begining_position=rospy.get_param("~grinding_pos_begining"),
        end_position=rospy.get_param("~grinding_pos_end"),
        begining_radious_z=rospy.get_param("~grinding_rz_begining"),
        end_radious_z=rospy.get_param("~grinding_rz_end"),
        angle_param=rospy.get_param("~grinding_angle_param"),
        yaw_angle=rospy.get_param("~grinding_yaw_angle"),
        number_of_rotations=rospy.get_param("~grinding_number_of_rotation"),
        number_of_waypoints_per_circle=rospy.get_param("~grinding_waypoints_counts"),
        center_position=rospy.get_param("~grinding_center_pos"),
        debug=False,
    )
    # debug_marker.display_waypoints(waypoints)

    result = motion_routines_class.execute_grinding(
        waypoints,
        eef_link,
        grinding_velocity_scale=rospy.get_param("~grinding_vel_scale"),
        grinding_acceleration_scale=rospy.get_param("~grinding_acc_scale"),
        moving_velocity_scale=rospy.get_param("~moving_vel_scale"),
        moving_acceleration_scale=rospy.get_param("~moving_acc_scale"),
        execute=execute,
    )
    print("grinding success", result)

    return result


def execute_gathering(motion_routines_class, eef_link, execute=True):
    waypoints_list = motion_routines_class.create_liner_waypoints_list(
        begining_theta=rospy.get_param("~gathering_theta_begining"),
        end_tehta=rospy.get_param("~gathering_theta_end"),
        begining_length_from_center=rospy.get_param("~gathering_radius_begining"),
        end_length_from_center=rospy.get_param("~gathering_radius_end"),
        begining_radius_z=rospy.get_param("~gathering_rz_begining"),
        end_radius_z=rospy.get_param("~gathering_rz_end"),
        waypoints_count=rospy.get_param("~gathering_waypoints_counts"),
        gathering_motion_count=rospy.get_param("~gathering_motion_counts"),
        angle_param=rospy.get_param("~gathering_angle_param"),
        fixed_quaternion=rospy.get_param("~gathering_fixed_quaternion"),
        yaw_angle=rospy.get_param("~gathering_yaw_angle"),
        debug=False,
    )
    # for waypoints in motion_routines_class.gathering_waypoints_list:
    #     debug_marker.display_waypoints(waypoints)
    # debug_tf.broadcast_tf_with_waypoints(waypoints, parent_link="base_link")
    result = motion_routines_class.execute_gathering(
        waypoints_list,
        eef_link,
        gathering_velocity_scale=rospy.get_param("~gathering_vel_scale"),
        gathering_acceleration_scale=rospy.get_param("~gathering_acc_scale"),
        moving_velocity_scale=rospy.get_param("~moving_vel_scale"),
        moving_acceleration_scale=rospy.get_param("~moving_acc_scale"),
        yaw_angle=rospy.get_param("~gathering_yaw_angle"),
        execute=execute,
    )
    print("gathering success", result)
    return result


def execute_circular_gathering(motion_routines_class, eef_link, execute=True):
    waypoints = motion_routines_class.create_circular_waypoints(
        begining_position=rospy.get_param("~gathering_2_pos_begining"),
        end_position=rospy.get_param("~gathering_2_pos_end"),
        begining_radious_z=rospy.get_param("~gathering_2_rz_begining"),
        end_radious_z=rospy.get_param("~gathering_2_rz_end"),
        angle_param=rospy.get_param("~gathering_2_angle_param"),
        yaw_angle=rospy.get_param("~gathering_2_yaw_angle"),
        number_of_rotations=rospy.get_param("~gathering_2_number_of_rotation"),
        number_of_waypoints_per_circle=rospy.get_param("~gathering_2_waypoints_counts"),
        debug=False,
    )

    result = motion_routines_class.execute_circular_gathering(
        waypoints,
        eef_link,
        grinding_velocity_scale=rospy.get_param("~gathering_2_vel_scale"),
        grinding_acceleration_scale=rospy.get_param("~gathering_2_acc_scale"),
        moving_velocity_scale=rospy.get_param("~moving_vel_scale"),
        moving_acceleration_scale=rospy.get_param("~moving_acc_scale"),
        yaw_angle=rospy.get_param("~gathering_2_yaw_angle"),
        execute=execute,
    )
    print("grinding success", result)

    return result


def init_planning_scene(motion_routines_class):
    work_mesh_dir = roslib.packages.get_pkg_dir("grinding_descriptions") + "/mesh/"
    table_pos = rospy.get_param("~table_position")
    table_scale = rospy.get_param("~table_scale")
    mortar_pos = rospy.get_param("~mortar_position")
    mortar_inner_scale = rospy.get_param("~mortar_inner_scale")

    # table
    table_pose = geometry_msgs.msg.PoseStamped()
    table_pose.header.frame_id = motion_routines_class.planning_frame
    table_pose.pose.orientation.w = 1.0
    table_pose.pose.position.z = -table_scale["z"] / 2
    motion_routines_class.scene.add_box(
        "table",
        table_pose,
        size=(table_scale["x"], table_scale["y"], table_scale["z"]),
    )

    # mortar base
    mortar_base_pose = geometry_msgs.msg.PoseStamped()
    mortar_base_pose.header.frame_id = motion_routines_class.planning_frame
    mortar_base_pose.pose.orientation.w = 1.0
    mortar_base_pose.pose.position.y = mortar_pos["y"]
    mortar_base_pose.pose.position.z = mortar_pos["z"] + table_pos["z"]
    motion_routines_class.scene.add_box(
        "mortar_base",
        mortar_base_pose,
        size=(table_scale["x"], mortar_inner_scale["y"] * 2, mortar_pos["z"]),
    )
    # mortar
    mortar_pose = geometry_msgs.msg.PoseStamped()
    mortar_pose.header.frame_id = motion_routines_class.planning_frame
    mortar_pose.pose.position.x = mortar_pos["x"]
    mortar_pose.pose.position.y = mortar_pos["y"]
    mortar_pose.pose.position.z = mortar_pos["z"]
    motion_routines_class.scene.add_mesh(
        "mortar", mortar_pose, work_mesh_dir + "mortar_40mm.stl"
    )


def get_out_lock_condition(motion_routines_class):
    goal_pose = geometry_msgs.msg.Pose()
    current_pose = motion_routines_class.move_group.get_current_pose()
    goal_pose.position = current_pose.pose.position
    goal_pose.orientation = current_pose.pose.orientation
    goal_pose.position.z += 0.1
    motion_routines_class.move_cartesian_path_with_goal_pose(
        goal_pose,
        vel_scale=rospy.get_param("~moving_vel_scale"),
        acc_scale=rospy.get_param("~moving_acc_scale"),
        execute=True,
    )


def exit_demo(msg=""):
    if msg != "":
        rospy.loginfo(msg)
    rospy.loginfo("Exit demo")
    rospy.signal_shutdown("finish")
    exit()


def mortar_hight_calibration(motion_routines_class):
    rospy.loginfo("goto mortar hight calibration pose")

    mortar_pos = rospy.get_param("~mortar_position")
    calib_pose = geometry_msgs.msg.Pose()
    calib_pose.position.x = mortar_pos["x"]
    calib_pose.position.y = mortar_pos["y"]
    calib_pose.position.z = mortar_pos["z"]
    calib_pose.position.z += rospy.get_param("~mortar_hight")
    q = tf.quaternion_from_euler(pi, 0, pi)
    calib_pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    motion_routines_class.move_cartesian_path_with_goal_pose(
        calib_pose, end_effector_link=rospy.get_param("~grinding_eef_link")
    )


def command_to_execute(cmd):
    if cmd == "y":
        return True
    elif cmd == "s":
        return False
    else:
        return None


def main():
    global last_motion

    ################### init node ###################
    rospy.init_node("move_group_python_interface", anonymous=True)

    ################### read params ###################
    experimental_time = rospy.get_param("~experimental_time")
    move_group_name = rospy.get_param("~move_group_name")
    grinding_eef_link = rospy.get_param("~grinding_eef_link")
    gathering_eef_link = rospy.get_param("~gathering_eef_link")
    motion_routines_class = motion_routines.MotionRoutine(
        move_group_name, grinding_eef_link
    )

    ################### init planning scene ###################
    init_planning_scene(motion_routines_class)

    ################### init planning scene ###################

    motion_result = True

    try:
        while not rospy.is_shutdown():
            motion_command = input(
                "Input command.\n\n q \t\t\t= exit.\n"
                + "scene \t\t\t= init planning scene.\n "
                + "grinding, gathering \t= 'g' grinding,'G' linear gathering,'G2' circular gathering and 'GG' both it.\n"
                + "continue \t\t= continue GG motion during the experiment time.\n"
                + "calib \t\t= go to calibration pose of mortar hight.\n"
                + "\n"
            )

            if motion_command == "q":
                exit_demo()

            elif motion_command == "scene":
                rospy.loginfo("Init planning scene")
                init_planning_scene(motion_routines_class)

            if motion_command == "g":
                key = input(
                    "Start grinding demo.\n execute = 'y', step by step = 's', continue grinding = 'c', canncel = other\n"
                )
                if command_to_execute(key) != None:
                    motion_result = execute_grinding(
                        motion_routines_class,
                        grinding_eef_link,
                        execute=command_to_execute(key),
                    )
            if motion_command == "G":
                key = input(
                    "Start linear gathering demo.\n execute = 'y', step by step = 's',  canncel = other\n"
                )
                if command_to_execute(key) != None:
                    motion_result = execute_gathering(
                        motion_routines_class,
                        gathering_eef_link,
                        execute=command_to_execute(key),
                    )
            if motion_command == "G2":
                key = input(
                    "Start circular gathering demo.\n execute = 'y', step by step = 's',  canncel = other\n"
                )
                if command_to_execute(key) != None:
                    motion_result = execute_circular_gathering(
                        motion_routines_class,
                        gathering_eef_link,
                        execute=command_to_execute(key),
                    )
            if motion_command == "GG":
                st = time.time()
                execute_grinding(motion_routines_class, grinding_eef_link, execute=True)
                motion_result = execute_gathering(
                    motion_routines_class, gathering_eef_link, execute=True
                )
                rospy.loginfo("GG time: " + str(time.time() - st))
            if motion_command == "continue":
                motion_counts = 0
                st = time.time()
                while True:
                    try:
                        key = inputimeout(
                            prompt="If you want to finish grinding, Prease enter 'q'.\n",
                            timeout=TIMEOUT_SEC,
                        )
                        if key == "q":
                            exit_demo()

                    except TimeoutOccurred:
                        execute_grinding(
                            motion_routines_class, grinding_eef_link, execute=True
                        )
                        motion_result = execute_circular_gathering(
                            motion_routines_class, gathering_eef_link, execute=True
                        )
                        motion_counts += 1

                    experiment_time = (time.time() - st) / 60
                    rospy.loginfo("Experiment time: " + str(experiment_time) + " min")
                    if experiment_time > experimental_time:
                        rospy.loginfo("Over experiment time")
                        exit_demo("Motion counts: " + str(motion_counts))

            if motion_command == "calib":
                mortar_hight_calibration(motion_routines_class)

            if motion_result == False:
                rospy.loginfo("Get out lock condition")
                get_out_lock_condition(motion_routines_class)
                motion_result = True

    except rospy.ROSInterruptException as err:
        exit_demo(err)
    except KeyboardInterrupt as err:
        exit_demo(err)


if __name__ == "__main__":
    main()
