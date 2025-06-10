#!/usr/bin/env python3


import rospy
import tf.transformations as tf
import time

from inputimeout import inputimeout, TimeoutOccurred

from math import pi
import copy
from scipy.spatial.transform import Rotation
import numpy as np

from grinding_motion_routines import (
    motion_generator,
    moveit_executor,
    JTC_executor,
    motion_primitive,
    marker_display,
    tf_publisher,
)

from grinding_descriptions import load_planning_scene

################### Fixed params ###################

# pouse time
pouse_time_list = [300]
initial_experiment_time = 0

# experiments params
TIMEOUT_SEC = 0.1

# debug class
debug_marker = marker_display.MarkerDisplay("debug_marker")
debug_tf = tf_publisher.TFPublisher()


def display_debug_waypoints(waypoints, debug_type, tf_name="debug"):
    if debug_type == "mk":
        rospy.loginfo("Display waypoints marker")
        debug_marker.display_waypoints(waypoints, clear=True)
    elif debug_type == "tf":
        rospy.loginfo("Display waypoints tf")
        debug_tf.broadcast_tf_with_waypoints(
            waypoints, parent_link="base_link", child_link=tf_name + "_waypoints"
        )


def compute_grinding_waypoints(motion_generator, debug_type=False):
    waypoints = motion_generator.create_circular_waypoints(
        beginning_position=rospy.get_param("~grinding_pos_beginning"),
        end_position=rospy.get_param("~grinding_pos_end"),
        beginning_radius_z=rospy.get_param("~grinding_rz_beginning"),
        end_radius_z=rospy.get_param("~grinding_rz_end"),
        angle_scale=rospy.get_param("~grinding_angle_scale"),
        yaw_bias=rospy.get_param("~grinding_yaw_bias"),
        number_of_rotations=rospy.get_param("~grinding_number_of_rotation"),
        number_of_waypoints_per_circle=rospy.get_param(
            "~grinding_number_of_waypoints_per_circle"
        ),
        center_position=rospy.get_param("~grinding_center_pos"),
    )
    if debug_type != False:
        display_debug_waypoints(waypoints, debug_type)
    return waypoints


def compute_gathering_waypoints(motion_generator, debug_type=False):
    waypoints = motion_generator.create_circular_waypoints(
        beginning_position=rospy.get_param("~gathering_pos_beginning"),
        end_position=rospy.get_param("~gathering_pos_end"),
        beginning_radius_z=rospy.get_param("~gathering_rz_beginning"),
        end_radius_z=rospy.get_param("~gathering_rz_end"),
        angle_scale=rospy.get_param("~gathering_angle_scale"),
        yaw_bias=rospy.get_param("~gathering_yaw_bias"),
        number_of_rotations=rospy.get_param("~gathering_number_of_rotation"),
        number_of_waypoints_per_circle=rospy.get_param(
            "~gathering_number_of_waypoints_per_circle"
        ),
    )
    if debug_type != False:
        display_debug_waypoints(waypoints, debug_type)
    return waypoints


def compute_epicycloid_grinding_waypoints(motion_gen, debug_type=False):
    """
    Compute grinding waypoints based on an epicycloid curve. The ROS parameters used are:
      ~epicycloid_scale_radius   : Scale radius in millimeters (R + 2r).
      ~epicycloid_ratio_R_r       : The ratio R/r.
      ~epicycloid_num_waypoints   : Number of waypoints to generate.
      ~epicycloid_angle_scale     : Angle scale parameter (default 0).
      ~epicycloid_yaw_bias        : Yaw bias value.
    """
    radius_mm = rospy.get_param("~epicycloid_radius_mm", 8)
    ratio_R_r = rospy.get_param("~epicycloid_ratio_R_r", 200 / 87)
    num_waypoints = rospy.get_param("~epicycloid_num_waypoints", 5000)
    angle_scale = rospy.get_param("~epicycloid_angle_scale", 0.3)
    yaw_bias = rospy.get_param("~epicycloid_yaw_bias", 0)

    waypoints = motion_gen.create_epicycloid_waypoints(
        radius_mm=radius_mm,
        ratio_R_r=ratio_R_r,
        number_of_waypoints=num_waypoints,
        angle_scale=angle_scale,
        yaw_bias=yaw_bias,
    )
    if debug_type:
        display_debug_waypoints(waypoints, debug_type)
    return waypoints

def exit_process(msg=""):
    if msg != "":
        rospy.loginfo(msg)
    rospy.loginfo("Exit mechano grinding")
    rospy.signal_shutdown("finish")
    rospy.spin()
    exit()


def command_to_execute(cmd):
    if cmd == "y":
        return True
    elif cmd == "mk":
        return False
    elif cmd == "tf":
        return False
    else:
        return None


def main():
    ################### init node ###################
    rospy.init_node("mechano_grinding", anonymous=True)
    experiment_time = rospy.get_param("~experiment_time", None)

    ################### motion generator ###################
    mortar_top_pos = rospy.get_param("~mortar_top_position", None)
    mortar_inner_size = rospy.get_param("~mortar_inner_size", None)
    motion_gen = motion_generator.MotionGenerator(mortar_top_pos, mortar_inner_size)

    ################### motion executor ###################
    move_group_name = rospy.get_param("~move_group_name", None)
    grinding_ee_link = rospy.get_param("~grinding_ee_link", None)
    gathering_ee_link = rospy.get_param("~gathering_ee_link", None)
    grinding_joint_difference_limit_for_motion_planning = rospy.get_param(
        "~grinding_joint_difference_limit_for_motion_planning", None
    )
    gathering_joint_difference_limit_for_motion_planning = rospy.get_param(
        "~gathering_joint_difference_limit_for_motion_planning", None
    )
    motion_planner_id = rospy.get_param("~motion_planner_id", None)
    planning_time = rospy.get_param("~planning_time", None)
    rospy.loginfo(grinding_joint_difference_limit_for_motion_planning)
    moveit = moveit_executor.MoveitExecutor(
        move_group_name, grinding_ee_link, motion_planner_id, planning_time
    )

    ################### init pose ###################
    init_pos = copy.deepcopy(mortar_top_pos)
    rospy.loginfo("Mortar pos: " + str(init_pos))
    init_pos["z"] += 0.05
    yaw = (
        np.arctan2(mortar_top_pos["y"], mortar_top_pos["x"])
        + rospy.get_param("~grinding_yaw_bias", 0)
    )
    euler = [pi, 0, yaw]
    r = Rotation.from_euler("xyz", euler, degrees=False)
    quat = r.as_quat()
    init_pose = list(init_pos.values()) + list(quat)
    moveit.execute_to_goal_pose(
        init_pose, ee_link=grinding_ee_link, vel_scale=0.5, acc_scale=0.5
    )
    rospy.loginfo("Goto init pose")

    ################### motion primitive ###################
    ik_solver = rospy.get_param("~ik_solver", None)
    robot_urdf_pkg = rospy.get_param("~robot_urdf_pkg", None)
    robot_urdf_file_name = rospy.get_param("~robot_urdf_file_name", None)
    joint_trajectory_controller_name = rospy.get_param(
        "~joint_trajectory_controller_name", None
    )
    ns = rospy.get_param("~ns", None)
    motion_planner_id = rospy.get_param("~motion_planner_id", None)
    planning_time = rospy.get_param("~planning_time", None)

    primitive = motion_primitive.MotionPrimitive(
        init_pose=init_pose,
        ee_link=grinding_ee_link,
        robot_urdf_pkg=robot_urdf_pkg,
        robot_urdf_file_name=robot_urdf_file_name,
        joint_trajectory_controller_name=joint_trajectory_controller_name,
        ns=ns,
        joint_names_prefix=None,
        move_group_name=move_group_name,
        planner_id=motion_planner_id,
        planning_time=planning_time,
        ft_topic=None,
        ik_solver=ik_solver,
    )

    ################### init planning scene ###################
    planning_scene = load_planning_scene.PlanningSceneLoader(moveit.move_group)
    planning_scene.init_planning_scene()

    grinding_sec = rospy.get_param("~grinding_sec_per_rotation") * rospy.get_param(
        "~grinding_number_of_rotation"
    )
    gathering_sec = rospy.get_param("~gathering_sec_per_rotation") * rospy.get_param(
        "~gathering_number_of_rotation"
    )
    try:
        while not rospy.is_shutdown():
            motion_command = input(
                "q \t= exit.\n"
                + "scene \t= init planning scene.\n"
                + "pestle_calib \t= go to caliblation pose of pestle tip position.\n"
                + "g \t= grinding demo.\n"
                + "G \t= circular gathering demo.\n"
                + "Rg \t= repeate grinding motion during the experiment time.\n"
                + "RGG \t= repeate grinding and circular motion during the experiment time.\n"
                + "EPG \t= epicycloid grinding demo.\n"
                + "\n"
            )

            if motion_command == "q":
                exit_process()

            elif motion_command == "scene":
                rospy.loginfo("Init planning scene")
                planning_scene.init_planning_scene()
            elif motion_command == "pestle_calib":
                rospy.loginfo("Go to caliblation pose of pestle tip position")
                pos = copy.deepcopy(mortar_top_pos)
                quat = tf.quaternion_from_euler(pi, 0, 0)
                calib_pose = list(pos.values()) + quat.tolist()
                moveit.execute_cartesian_path_to_goal_pose(
                    calib_pose, ee_link=grinding_ee_link, vel_scale=0.9, acc_scale=0.9
                )

            elif motion_command == "g":
                key = input(
                    "Start grinding demo.\n execute = 'y', show waypoints marker = 'mk', show waypoints tf = 'tf', canncel = other\n"
                )
                exec = command_to_execute(key)
                if exec:
                    primitive.execute_grinding(
                        compute_grinding_waypoints(motion_gen),
                        grinding_sec=grinding_sec,
                        joint_difference_limit=grinding_joint_difference_limit_for_motion_planning,
                        max_attempts=100,
                        ee_link=grinding_ee_link,
                    )
                elif exec == False:
                    compute_grinding_waypoints(motion_gen, debug_type=key)
            elif motion_command == "G":
                key = input(
                    "Start circular gathering demo.\n execute = 'y', show waypoints marker = 'mk', show waypoints tf = 'tf', canncel = other\n"
                )
                exec = command_to_execute(key)
                if exec:
                    primitive.execute_gathering(
                        compute_gathering_waypoints(motion_gen),
                        gathering_sec=gathering_sec,
                        joint_difference_limit=gathering_joint_difference_limit_for_motion_planning,
                        max_attempts=100,
                        ee_link=gathering_ee_link,
                    )
                elif exec == False:
                    compute_gathering_waypoints(motion_gen, debug_type=key)
            elif motion_command == "EPG":
                key = input(
                    "Start epicycloid grinding demo.\n execute = 'y', show waypoints marker = 'mk', show waypoints tf = 'tf', canncel = other\n"
                )
                exec = command_to_execute(key)
                if exec:
                    primitive.execute_grinding(
                        compute_epicycloid_grinding_waypoints(motion_gen),
                        grinding_sec=10,
                        joint_difference_limit=0.1,
                        max_attempts=100,
                        ee_link=grinding_ee_link,
                    )
                elif exec == False:
                    compute_epicycloid_grinding_waypoints(motion_gen, debug_type=key)

            elif motion_command == "Rg" or motion_command == "RGG":
                motion_counts = 0
                pouse_list_number = 0
                experiment_time = initial_experiment_time
                grinding_waypoints = np.array([])
                gathering_waypoints = np.array([])
                while True:
                    try:
                        key = inputimeout(
                            prompt="If you want to finish grinding -> 'q', pose -> 'p'.\n",
                            timeout=TIMEOUT_SEC,
                        )
                        if key == "q":
                            exit_process()
                        elif key == "p":
                            input("Press Enter to continue...")

                    except TimeoutOccurred:
                        st = time.time()
                        if motion_command == "Rg":
                            if grinding_waypoints.all():
                                grinding_waypoints = compute_grinding_waypoints(
                                    motion_gen
                                )
                            primitive.execute_grinding(
                                grinding_waypoints,
                                grinding_sec=grinding_sec,
                                joint_difference_limit=grinding_joint_difference_limit_for_motion_planning,
                                max_attempts=100,
                                ee_link=grinding_ee_link,
                            )
                        elif motion_command == "RGG":
                            if grinding_waypoints.all():
                                print("compute grinding waypoints")
                                grinding_waypoints = compute_grinding_waypoints(
                                    motion_gen
                                )
                            primitive.execute_grinding(
                                compute_grinding_waypoints(motion_gen),
                                grinding_sec=grinding_sec,
                                joint_difference_limit=grinding_joint_difference_limit_for_motion_planning,
                                max_attempts=100,
                                ee_link=grinding_ee_link,
                            )
                            if gathering_waypoints.all():
                                print("compute gathering waypoints")
                                gathering_waypoints = compute_gathering_waypoints(
                                    motion_gen
                                )
                            primitive.execute_gathering(
                                compute_gathering_waypoints(motion_gen),
                                gathering_sec=gathering_sec,
                                joint_difference_limit=gathering_joint_difference_limit_for_motion_planning,
                                max_attempts=100,
                                ee_link=gathering_ee_link,
                            )
                        motion_counts += 1

                    experiment_time += (time.time() - st) / 60
                    rospy.loginfo("Experiment time: " + str(experiment_time) + " min")
                    if pouse_time_list[pouse_list_number] < experiment_time:
                        pouse_list_number += 1
                        input(
                            "Pouse experiment on pouse settings. Press Enter to continue..."
                        )
                    if experiment_time > experiment_time:
                        rospy.loginfo("Over experiment time")
                        exit_process("Motion counts: " + str(motion_counts))

    except rospy.ROSInterruptException as err:
        exit_process(err)
    except KeyboardInterrupt as err:
        exit_process(err)


if __name__ == "__main__":
    main()
