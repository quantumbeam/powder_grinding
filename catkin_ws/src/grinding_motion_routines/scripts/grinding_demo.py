#!/usr/bin/env python3


import rospy
import tf.transformations as tf
import copy
import time


from math import pi, tau, dist, fabs, cos, sin, sqrt
from scipy.spatial.transform import Rotation

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
initial_experiment_time = 0

# experiments params
TIMEOUT_SEC = 0.1

# debug class
debug_marker = marker_display.MarkerDisplay("debug_marker")
debug_tf = tf_publisher.TFPublisher()


def display_debug_waypoints(waypoints):
    rospy.loginfo("Display debug marker and/or TF.")
    debug_marker.display_waypoints(waypoints)
    # debug_tf.broadcast_tf_with_waypoints(waypoints, "base_link")


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
    rospy.loginfo("Exit grinding demo")
    rospy.signal_shutdown("finish")
    rospy.spin()
    exit()


def input_command(msg):
    msg = msg + "\n execute = 'y', canncel = other\n"
    cmd = input(msg)
    if cmd == "y":
        return True
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
    motion_gen = motion_generator.MotionGenerator(
        mortar_base_pos, mortar_high, mortar_inner_scale
    )

    ################### motion executor ###################
    move_group_name = rospy.get_param("~move_group_name")
    grinding_ee_link = rospy.get_param("~grinding_eef_link")
    gathering_ee_link = rospy.get_param("~gathering_eef_link")
    # scooping_ee_link = rospy.get_param("~scooping_eef_link")
    moveit = moveit_executor.MoveitExecutor(move_group_name, grinding_ee_link)

    ################### motion primitive ###################
    start_joint_angle = rospy.get_param("~start_joint_angles")
    grinding_ready_joint_angle = rospy.get_param("~grinding_ready_joint_angles")
    gathering_ready_joint_angle = rospy.get_param("~gathering_ready_joint_angles")
    scooping_ready_joint_angle = rospy.get_param("~scooping_ready_joint_angles")
    # primitive = motion_primitive.MotionPrimitive(
    #     start_joint_angle,
    #     grinding_ready_joint_angle,
    #     gathering_ready_joint_angle,
    #     scooping_ready_joint_angle,
    #     move_group_name=move_group_name,
    # )

    ################### init planning scene ###################
    planning_scene = load_planning_scene.PlanningScene(moveit.move_group)
    planning_scene.init_planning_scene()

    ################### init pose ###################
    rospy.loginfo("goto init pose")
    init_pos = copy.deepcopy(mortar_base_pos)
    init_pos["z"] += 0.1
    r = Rotation.from_euler("xyz", [pi, 0, 0], degrees=False)
    quat = r.as_quat()
    init_pose = list(init_pos.values()) + list(quat)
    print(init_pose)
    moveit.execute_to_goal_pose(init_pose, vel_scale=0.9, acc_scale=0.9)

    try:
        while not rospy.is_shutdown():
            motion_command = input(
                "q \t= exit.\n"
                + "scene \t= init planning scene.\n"
                + "hight \t= go to calibration pose of mortar hight.\n"
                + "g \t= grinding demo.\n"
                + "G \t= circular gathering demo.\n"
                + "GG \t= grinding and gathering demo.\n"
                + "\n"
            )

            if motion_command == "q":
                exit_process()

            elif motion_command == "scene":
                rospy.loginfo("Init planning scene")
                planning_scene.init_planning_scene()
            elif motion_command == "hight":
                rospy.loginfo("Go to caliblation pose of mortar hight")
                pos = copy.deepcopy(mortar_base_pos)
                pos["z"] += rospy.get_param("~mortar_hight")
                quat = tf.quaternion_from_euler(0, -pi, 0)
                calib_pose = list(pos.values()) + quat.tolist()
                moveit.execute_cartesian_path_to_goal_pose(
                    calib_pose, ee_link=grinding_ee_link, vel_scale=0.9, acc_scale=0.9
                )

            elif motion_command == "g":
                waypoints = compute_grinding_waypoints(motion_gen)
                if input_command("Start grinding demo.") != None:
                    moveit.execute_cartesian_path_by_waypoints(
                        waypoints,
                        ee_link=grinding_ee_link,
                        vel_scale=1,
                        acc_scale=1,
                    )
                    moveit.execute_to_goal_pose(
                        init_pose,
                        ee_link=grinding_ee_link,
                        vel_scale=0.5,
                        acc_scale=0.5,
                    )
            elif motion_command == "G":
                waypoints = compute_gathering_waypoints(motion_gen)
                if input_command("Start gathering demo.") != None:
                    moveit.execute_to_goal_pose(
                        init_pose,
                        ee_link=gathering_ee_link,
                        vel_scale=0.5,
                        acc_scale=0.5,
                    )
                    moveit.execute_cartesian_path_by_waypoints(
                        waypoints,
                        ee_link=gathering_ee_link,
                        vel_scale=0.8,
                        acc_scale=0.8,
                    )
                    moveit.execute_to_goal_pose(
                        init_pose,
                        ee_link=grinding_ee_link,
                        vel_scale=0.5,
                        acc_scale=0.5,
                    )
            elif motion_command == "GG":
                if input_command("Start grinding and gathering demo.") != None:
                    time.sleep(5)
                    waypoints = compute_grinding_waypoints(motion_gen)
                    moveit.execute_cartesian_path_by_waypoints(
                        waypoints,
                        ee_link=grinding_ee_link,
                        vel_scale=1,
                        acc_scale=1,
                    )
                    moveit.execute_to_goal_pose(
                        init_pose,
                        ee_link=grinding_ee_link,
                        vel_scale=0.5,
                        acc_scale=0.5,
                    )
                    
                    moveit.execute_to_goal_pose(
                        init_pose,
                        ee_link=gathering_ee_link,
                        vel_scale=0.5,
                        acc_scale=0.5,
                    )
                    waypoints = compute_gathering_waypoints(motion_gen)
                    moveit.execute_cartesian_path_by_waypoints(
                        waypoints,
                        ee_link=gathering_ee_link,
                        vel_scale=0.8,
                        acc_scale=0.8,
                    )
                    moveit.execute_to_goal_pose(
                        init_pose,
                        ee_link=gathering_ee_link,
                        vel_scale=0.5,
                        acc_scale=0.5,
                    )
                    moveit.execute_to_goal_pose(
                        init_pose,
                        ee_link=grinding_ee_link,
                        vel_scale=0.5,
                        acc_scale=0.5,
                    )

    except rospy.ROSInterruptException as err:
        exit_process(err)
    except KeyboardInterrupt as err:
        exit_process(err)


if __name__ == "__main__":
    main()
