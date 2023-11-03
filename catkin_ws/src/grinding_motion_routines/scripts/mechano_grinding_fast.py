#!/usr/bin/env python3


import rospy
import tf.transformations as tf
import time

from inputimeout import inputimeout, TimeoutOccurred

from math import pi, tau, dist, fabs, cos, sin, sqrt

from grinding_motion_routines import (
    motion_generator,
    moveit_executor,
    JTC_executor,
    motion_primitive,
    marker_display,
    tf_publisher,
    load_planning_scene,
)


################### Fixed params ###################

# pouse time
pouse_time_list = [300]
initial_experiment_time = 0

# experiments params
TIMEOUT_SEC = 0.1

# debug class
debug_marker = marker_display.MarkerDisplay("debug_marker")
debug_tf = tf_publisher.TFPublisher()


def display_debug_waypoints(waypoints):
    debug_marker.display_waypoints(waypoints)
    debug_tf.broadcast_tf_with_waypoints(waypoints, "base_link")


def compute_grinding_waypoints(motion_generator):
    waypoints = motion_generator.create_circular_waypoints(
        begining_position=rospy.get_param("~grinding_pos_begining"),
        end_position=rospy.get_param("~grinding_pos_end"),
        begining_radious_z=rospy.get_param("~grinding_rz_begining"),
        end_radious_z=rospy.get_param("~grinding_rz_end"),
        angle_param=rospy.get_param("~grinding_angle_param"),
        yaw_angle=rospy.get_param("~grinding_yaw_angle"),
        number_of_rotations=rospy.get_param("~grinding_number_of_rotation"),
        number_of_waypoints_per_circle=rospy.get_param(
            "~grinding_number_of_waypoints_per_circle"
        ),
        center_position=rospy.get_param("~grinding_center_pos"),
    )
    # display_debug_waypoints(waypoints)
    return waypoints


def compute_gathering_waypoints(motion_generator):
    waypoints = motion_generator.create_circular_waypoints(
        begining_position=rospy.get_param("~circular_gathering_pos_begining"),
        end_position=rospy.get_param("~circular_gathering_pos_end"),
        begining_radious_z=rospy.get_param("~circular_gathering_rz_begining"),
        end_radious_z=rospy.get_param("~circular_gathering_rz_end"),
        angle_param=rospy.get_param("~circular_gathering_angle_param"),
        yaw_angle=rospy.get_param("~circular_gathering_yaw_angle"),
        number_of_rotations=rospy.get_param("~circular_gathering_number_of_rotation"),
        number_of_waypoints_per_circle=rospy.get_param(
            "~circular_gathering_number_of_waypoints_per_circle"
        ),
    )
    # display_debug_waypoints(waypoints)
    return waypoints


def compute_scooping_waypoints(motion_generator):
    waypoints = motion_generator.create_cartesian_waypoints(
        begining_position=rospy.get_param("~scooping_pos_begining"),
        end_position=rospy.get_param("~scooping_pos_end"),
        begining_radius_z=rospy.get_param("~scooping_rz_begining"),
        end_radius_z=rospy.get_param("~scooping_rz_end"),
        angle_param=rospy.get_param("~scooping_angle_param"),
        yaw_angle=rospy.get_param("~scooping_yaw_angle"),
        number_of_waypoints=rospy.get_param("~scooping_number_of_waypoints"),
    )
    # display_debug_waypoints(waypoints)

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
    elif cmd == "s":
        return False
    else:
        return None


def main():
    ################### init node ###################
    rospy.init_node("mechano_grinding", anonymous=True)
    experimental_time = rospy.get_param("~experimental_time")

    ################### motion generator ###################
    mortar_base_pos = rospy.get_param("~mortar_position")
    mortar_high = rospy.get_param("~mortar_hight")
    mortar_inner_scale = rospy.get_param("~mortar_inner_scale")
    pouring_to_funnel_position = rospy.get_param("~pouring_to_funnel_position")
    motion_gen = motion_generator.MotionGenerator(
        mortar_base_pos, mortar_high, mortar_inner_scale
    )

    ################### motion executor ###################
    move_group_name = rospy.get_param("~move_group_name")
    grinding_ee_link = rospy.get_param("~grinding_eef_link")
    gathering_ee_link = rospy.get_param("~gathering_eef_link")
    scooping_ee_link = rospy.get_param("~scooping_eef_link")
    moveit = moveit_executor.MoveitExecutor(move_group_name, grinding_ee_link)

    ################### motion primitive ###################
    start_joint_angle = rospy.get_param("~start_joint_angles")
    grinding_ready_joint_angle = rospy.get_param("~grinding_ready_joint_angles")
    gathering_ready_joint_angle = rospy.get_param("~gathering_ready_joint_angles")
    scooping_ready_joint_angle = rospy.get_param("~scooping_ready_joint_angles")
    primitive = motion_primitive.MotionPrimitive(
        start_joint_angle,
        grinding_ready_joint_angle,
        gathering_ready_joint_angle,
        scooping_ready_joint_angle,
    )

    ################### init planning scene ###################
    planning_scene = load_planning_scene.PlanningScene(moveit.move_group)
    planning_scene.init_planning_scene()

    ################### init pose ###################
    rospy.loginfo("goto init pose")
    moveit.execute_to_joint_goal(start_joint_angle, vel_scale=0.1, acc_scale=0.1)

    grinding_sec = rospy.get_param("~grinding_sec_per_rotation") * rospy.get_param(
        "~grinding_number_of_rotation"
    )
    try:
        while not rospy.is_shutdown():
            motion_command = input(
                "q \t= exit.\n"
                + "scene \t= init planning scene.\n"
                + "calib_hight \t= go to calibration pose of mortar hight.\n"
                + "calib_pos \t= go to calibration pose of mortar position.\n"
                + "g \t= grinding demo.\n"
                + "G \t= circular gathering demo.\n"
                + "sc \t= scooping demo.\n"
                + "po \t= powder pouring demo.\n"
                + "all \t= demo of grinding, gathering, scooping and pouring.\n"
                + "Rg \t= repeate grinding motion during the experiment time.\n"
                + "RGG \t= repeate grinding and circular motion during the experiment time.\n"
                + "\n"
            )

            if motion_command == "q":
                exit_process()

            elif motion_command == "scene":
                rospy.loginfo("Init planning scene")
                planning_scene.init_planning_scene()
            elif motion_command == "calib_hight":
                rospy.loginfo("Go to caliblation pose of mortar hight")
                pos = mortar_base_pos
                pos["z"] += rospy.get_param("~mortar_hight")
                quat = tf.quaternion_from_euler(0, -pi, 0)
                calib_pose = list(pos.values()) + quat.tolist()
                moveit.execute_cartesian_path_to_goal_pose(
                    calib_pose, ee_link=grinding_ee_link
                )
            elif motion_command == "calib_pos":
                rospy.loginfo("Go to caliblation pose of mortar position")
                mortar_position_calib_joint_angles = rospy.get_param(
                    "~mortar_position_calib_joint_angles"
                )
                moveit.execute_to_joint_goal(
                    mortar_position_calib_joint_angles, vel_scale=0.1, acc_scale=0.1
                )

            elif motion_command == "g":
                key = input(
                    "Start grinding demo.\n execute = 'y', step by step = 's', canncel = other\n"
                )
                if command_to_execute(key) != None:
                    grinding_joint_trajectory = (
                        primitive.JTC_executor.generate_joint_trajectory(
                            compute_grinding_waypoints(motion_gen),
                            total_joint_limit=1,
                            ee_link=grinding_ee_link,
                            trial_number=20,
                        )
                    )
                    primitive.execute_grinding(
                        grinding_joint_trajectory,
                        ee_link=grinding_ee_link,
                        grinding_sec=grinding_sec,
                    )
            elif motion_command == "G":
                key = input(
                    "Start circular gathering demo.\n execute = 'y', step by step = 's',  canncel = other\n"
                )
                if command_to_execute(key) != None:
                    primitive.execute_gathering(
                        compute_gathering_waypoints(motion_gen),
                        ee_link=gathering_ee_link,
                    )
            elif motion_command == "sc":
                key = input(
                    "Start scooping demo.\n execute = 'y', step by step = 's',  canncel = other\n"
                )
                if command_to_execute(key) != None:
                    primitive.execute_scooping(compute_scooping_waypoints(motion_gen))
            elif motion_command == "po":
                key = input(
                    "Start powder pouring demo at current position.\n execute = 'y', step by step = 's',  canncel = other\n"
                )
                if command_to_execute(key) != None:
                    primitive.execute_powder_pouring(
                        pouring_to_funnel_position.values()
                    )
            elif motion_command == "all":
                key = input(
                    "Start demo of grinding, gathering, scooring and pouring.\n execute = 'y', step by step = 's',  canncel = other\n"
                )
                if command_to_execute(key) != None:
                    grinding_joint_trajectory = (
                        primitive.JTC_executor.generate_joint_trajectory(
                            compute_grinding_waypoints(motion_gen),
                            total_joint_limit=1,
                            ee_link=grinding_ee_link,
                            trial_number=20,
                        )
                    )
                    primitive.execute_grinding(
                        grinding_joint_trajectory,
                        ee_link=grinding_ee_link,
                        grinding_sec=grinding_sec,
                    )
                    primitive.execute_gathering(
                        compute_gathering_waypoints(motion_gen),
                        ee_link=gathering_ee_link,
                    )
                    primitive.execute_scooping(compute_scooping_waypoints(motion_gen))
                    primitive.execute_powder_pouring(
                        pouring_to_funnel_position.values()
                    )

            elif motion_command == "Rg" or motion_command == "RGG":
                motion_counts = 0
                pouse_list_number = 0
                experiment_time = initial_experiment_time
                grinding_waypoints = compute_grinding_waypoints(motion_gen)
                grinding_joint_trajectory = (
                    primitive.JTC_executor.generate_joint_trajectory(
                        grinding_waypoints,
                        total_joint_limit=1,
                        ee_link=grinding_ee_link,
                        trial_number=20,
                    )
                )
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
                                grinding_joint_trajectory,
                                ee_link=grinding_ee_link,
                                grinding_sec=grinding_sec,
                            )
                        elif motion_command == "RGG":
                            primitive.execute_grinding(
                                grinding_joint_trajectory,
                                ee_link=grinding_ee_link,
                                grinding_sec=grinding_sec,
                            )
                            primitive.execute_gathering(
                                compute_gathering_waypoints(motion_gen),
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
