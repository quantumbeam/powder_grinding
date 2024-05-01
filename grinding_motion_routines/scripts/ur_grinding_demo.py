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
        begining_position=rospy.get_param("~grinding_pos_begining"),
        end_position=rospy.get_param("~grinding_pos_end"),
        begining_radious_z=rospy.get_param("~grinding_rz_begining"),
        end_radious_z=rospy.get_param("~grinding_rz_end"),
        angle_param=rospy.get_param("~grinding_angle_param"),
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
        begining_position=rospy.get_param("~gathering_pos_begining"),
        end_position=rospy.get_param("~gathering_pos_end"),
        begining_radious_z=rospy.get_param("~gathering_rz_begining"),
        end_radious_z=rospy.get_param("~gathering_rz_end"),
        angle_param=rospy.get_param("~gathering_angle_param"),
        yaw_bias=rospy.get_param("~gathering_yaw_bias"),
        number_of_rotations=rospy.get_param("~gathering_number_of_rotation"),
        number_of_waypoints_per_circle=rospy.get_param(
            "~gathering_number_of_waypoints_per_circle"
        ),
    )
    if debug_type != False:
        display_debug_waypoints(waypoints, debug_type)
    return waypoints


def compute_scooping_waypoints(motion_generator, debug_type=False):
    waypoints = motion_generator.create_cartesian_waypoints(
        begining_position=rospy.get_param("~scooping_pos_begining"),
        end_position=rospy.get_param("~scooping_pos_end"),
        begining_radius_z=rospy.get_param("~scooping_rz_begining"),
        end_radius_z=rospy.get_param("~scooping_rz_end"),
        angle_param=rospy.get_param("~scooping_angle_param"),
        yaw_bias=rospy.get_param("~scooping_yaw_bias"),
        number_of_waypoints=rospy.get_param("~scooping_number_of_waypoints"),
    )
    if debug_type != False:
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
    experimental_time = rospy.get_param("~experimental_time", None)

    ################### motion generator ###################
    mortar_top_pos = rospy.get_param("~mortar_top_position", None)
    mortar_inner_scale = rospy.get_param("~mortar_inner_scale", None)
    funnel_position = rospy.get_param("~funnel_position", None)
    pouring_hight = rospy.get_param("~pouring_hight_at_funnel", None)
    motion_gen = motion_generator.MotionGenerator(mortar_top_pos, mortar_inner_scale)

    ################### motion executor ###################
    move_group_name = rospy.get_param("~move_group_name", None)
    grinding_ee_link = rospy.get_param("~grinding_eef_link", None)
    gathering_ee_link = rospy.get_param("~gathering_eef_link", None)
    scooping_ee_link = rospy.get_param("~scooping_eef_link", None)
    grinding_total_joint_diffence_for_planning = rospy.get_param(
        "~grinding_total_joint_diffence_for_planning", None
    )
    gathering_total_joint_diffence_for_planning = rospy.get_param(
        "~gathering_total_joint_diffence_for_planning", None
    )
    motion_planner_id = rospy.get_param("~motion_planner_id", None)
    planning_time = rospy.get_param("~planning_time", None)
    rospy.loginfo(grinding_total_joint_diffence_for_planning)
    moveit = moveit_executor.MoveitExecutor(
        move_group_name, grinding_ee_link, motion_planner_id, planning_time
    )

    ################### init pose ###################
    init_pos = copy.deepcopy(mortar_top_pos)
    rospy.loginfo("Mortar pos: " + str(init_pos))
    init_pos["z"] += 0.05
    yaw = np.arctan2(mortar_top_pos["y"], mortar_top_pos["x"])
    euler = [pi, 0, yaw]
    r = Rotation.from_euler("xyz", [pi, 0, yaw], degrees=False)
    quat = r.as_quat()
    init_pose = list(init_pos.values()) + list(euler)
    init_pose_quat = list(init_pos.values()) + list(quat)
    moveit.execute_to_goal_pose(
        init_pose_quat, ee_link=grinding_ee_link, vel_scale=0.5, acc_scale=0.5
    )
    debug_tf.broadcast_tf_with_pose(init_pose_quat, "base_link", "init_pose")
    rospy.loginfo("Goto init pose")

    ################### motion primitive ###################
    urdf_name = rospy.get_param("~urdf_name", None)
    ik_solver = rospy.get_param("~ik_solver", None)
    primitive = motion_primitive.MotionPrimitive(
        init_pose=init_pose,
        ns=None,
        move_group_name=move_group_name,
        ee_link=grinding_ee_link,
        robot_urdf=urdf_name,
        planner_id=motion_planner_id,
        planning_time=planning_time,
        ik_solver=ik_solver,
    )
    pouring_position = copy.deepcopy(list(funnel_position.values()))
    pouring_position[2] += pouring_hight

    ################### init planning scene ###################
    planning_scene = load_planning_scene.PlanningScene(moveit.move_group)
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
                + "g \t= grinding demo.\n"
                + "G \t= circular gathering demo.\n"
                + "sc \t= scooping demo.\n"
                + "po \t= powder pouring demo.\n"
                + "Rg \t= repeate grinding motion during the experiment time.\n"
                + "RGG \t= repeate grinding and circular motion during the experiment time.\n"
                + "\n"
            )

            if motion_command == "q":
                exit_process()

            elif motion_command == "scene":
                rospy.loginfo("Init planning scene")
                planning_scene.init_planning_scene()

            elif motion_command == "g":
                key = input(
                    "Start grinding demo.\n execute = 'y', show waypoints marker = 'mk', show waypoints tf = 'tf', canncel = other\n"
                )
                exec = command_to_execute(key)
                if exec:
                    primitive.execute_grinding(
                        compute_grinding_waypoints(motion_gen),
                        grinding_sec=grinding_sec,
                        total_joint_limit=grinding_total_joint_diffence_for_planning,
                        trial_number=10,
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
                        total_joint_limit=gathering_total_joint_diffence_for_planning,
                        trial_number=10,
                        ee_link=gathering_ee_link,
                    )
                elif exec == False:
                    compute_gathering_waypoints(motion_gen, debug_type=key)

            elif motion_command == "sc":
                key = input(
                    "Start scooping demo.\n execute = 'y', show waypoints marker = 'mk', show waypoints tf = 'tf', canncel = other\n"
                )
                exec = command_to_execute(key)
                if exec:
                    primitive.execute_scooping(compute_scooping_waypoints(motion_gen))
                elif exec == False:
                    display_debug_waypoints(
                        compute_scooping_waypoints(motion_gen), debug_type=key
                    )
            elif motion_command == "po":
                key = input(
                    "Start powder pouring demo at current position.\n execute = 'y', show waypoints marker = 'mk', show waypoints tf = 'tf', canncel = other\n"
                )
                exec = command_to_execute(key)
                if exec:
                    primitive.execute_powder_pouring(pouring_position)

            elif motion_command == "Rg" or motion_command == "RGG":
                motion_counts = 0
                pouse_list_number = 0
                experiment_time = initial_experiment_time
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
                            primitive.execute_grinding(
                                compute_grinding_waypoints(motion_gen),
                                grinding_sec=grinding_sec,
                                total_joint_limit=grinding_total_joint_diffence_for_planning,
                                trial_number=10,
                                ee_link=grinding_ee_link,
                            )
                        elif motion_command == "RGG":
                            primitive.execute_grinding(
                                compute_grinding_waypoints(motion_gen),
                                grinding_sec=grinding_sec,
                                total_joint_limit=grinding_total_joint_diffence_for_planning,
                                trial_number=10,
                                ee_link=grinding_ee_link,
                            )
                            primitive.execute_gathering(
                                compute_gathering_waypoints(motion_gen),
                                gathering_sec=gathering_sec,
                                total_joint_limit=gathering_total_joint_diffence_for_planning,
                                trial_number=10,
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
                    if experiment_time > experimental_time:
                        rospy.loginfo("Over experiment time")
                        exit_process("Motion counts: " + str(motion_counts))

    except rospy.ROSInterruptException as err:
        exit_process(err)
    except KeyboardInterrupt as err:
        exit_process(err)


if __name__ == "__main__":
    main()
