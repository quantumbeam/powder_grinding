#!/usr/bin/env python3


import rospy
import tf.transformations as tf
import copy
import tkinter as tk

from math import pi, tau, dist, fabs, cos, sin, sqrt
from scipy.spatial.transform import Rotation

from grinding_motion_routines import (
    motion_generator,
    motion_primitive,
    moveit_executor,
    marker_display,
    tf_publisher,
)
from grinding_descriptions import load_planning_scene


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
    # debug_marker.display_waypoints(waypoints)
    debug_tf.broadcast_tf_with_waypoints(waypoints, "base_link")


def compute_grinding_waypoints(motion_generator):
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
    # display_debug_waypoints(waypoints)
    return waypoints


def compute_gathering_waypoints(motion_generator):
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
    # display_debug_waypoints(waypoints)
    return waypoints


def exit_process(msg=""):
    if msg != "":
        rospy.loginfo(msg)
    rospy.loginfo("Exit grinding demo")
    rospy.signal_shutdown("finish")
    rospy.spin()
    exit()


def main():
    ################### init node ###################
    rospy.init_node("fr_grinding_demo", anonymous=True)

    ################### motion generator ###################
    mortar_top_pos = rospy.get_param("~mortar_top_position")
    mortar_inner_scale = rospy.get_param("~mortar_inner_scale")
    motion_gen = motion_generator.MotionGenerator(mortar_top_pos, mortar_inner_scale)

    # paarmeeters for grinding and gathering
    grinding_sec = rospy.get_param("~grinding_sec_per_rotation") * rospy.get_param(
        "~grinding_number_of_rotation"
    )
    gathering_sec = rospy.get_param("~gathering_sec_per_rotation") * rospy.get_param(
        "~gathering_number_of_rotation"
    )
    grinding_total_joint_diffence_for_planning = rospy.get_param(
        "~grinding_total_joint_diffence_for_planning", None
    )
    gathering_total_joint_diffence_for_planning = rospy.get_param(
        "~gathering_total_joint_diffence_for_planning", None
    )

    ################### motion executor ###################
    move_group_name = rospy.get_param("~move_group_name")
    grinding_ee_link = rospy.get_param("~grinding_eef_link")
    gathering_ee_link = rospy.get_param("~gathering_eef_link")
    moveit = moveit_executor.MoveitExecutor(move_group_name, grinding_ee_link)

    ################### init planning scene ###################
    planning_scene = load_planning_scene.PlanningScene(moveit.move_group)
    planning_scene.init_planning_scene()

    ################### init pose ###################
    rospy.loginfo("goto init pose")
    init_pos = copy.deepcopy(mortar_top_pos)
    init_pos["z"] += 0.1
    r = Rotation.from_euler("xyz", [pi, 0, 0], degrees=False)
    quat = r.as_quat()
    init_pose = list(init_pos.values()) + list(quat)
    moveit.execute_to_goal_pose(init_pose, vel_scale=0.9, acc_scale=0.9)

    ################### motion primitive ###################
    ik_solver = rospy.get_param("~ik_solver", None)
    primitive = motion_primitive.MotionPrimitive(
        init_pose=init_pose,
        ee_link=grinding_ee_link,
        robot_urdf_pkg="robot_description",
        robot_urdf_file_name="fr3_robot",
        ns="/frrobot",
        joint_names_prefix=None,
        move_group_name=move_group_name,
        planner_id="RRTConnectkConfigDefault",
        planning_time=20,
        ft_topic=None,
        ik_solver="trac_ik",
    )

    # main loop
    def send_grinding_command():
        primitive.execute_grinding(
            compute_grinding_waypoints(motion_gen),
            grinding_sec=grinding_sec,
            total_joint_limit=grinding_total_joint_diffence_for_planning,
            trial_number=10,
            ee_link=grinding_ee_link,
        )

    def send_gathering_command():
        primitive.execute_gathering(
            compute_gathering_waypoints(motion_gen),
            gathering_sec=gathering_sec,
            total_joint_limit=gathering_total_joint_diffence_for_planning,
            trial_number=10,
            ee_link=gathering_ee_link,
        )

    def send_grinding_and_gathering_command():
        send_grinding_command()
        send_gathering_command()

    root = tk.Tk()
    font = ("Meiryo", 20)
    root.title("ROS Commands")

    grinding_button = tk.Button(
        root, text="Grinding", command=send_grinding_command, font=font
    )
    grinding_button.pack(pady=5)

    gathering_button = tk.Button(
        root, text="Gathering", command=send_gathering_command, font=font
    )
    gathering_button.pack(pady=5)

    grinding_and_gathering_button = tk.Button(
        root,
        text="Grinding & Gathering",
        command=send_grinding_and_gathering_command,
        font=font,
    )
    grinding_and_gathering_button.pack(pady=5)

    grinding_button = tk.Button(root, text="Exit demo", command=exit_process, font=font)
    grinding_button.pack(pady=5)

    root.mainloop()


if __name__ == "__main__":
    main()