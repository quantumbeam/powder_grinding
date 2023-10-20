#!/usr/bin/env python3

import rospy
import tf.transformations as tf
from scipy.spatial.transform import Rotation
from kek_routines.moveit_executor import MoveitExecutor
from kek_routines.JTC_executor import JointTrajectoryControllerExecutor

import numpy as np
from numpy import pi


class MotionPrimitive:
    def __init__(
        self,
        start_joint_angle,
        grinding_ready_joint_angle,
        gathering_ready_joint_angle,
        scooping_ready_joint_angle,
        ns=None,
        move_group_name="manipulator",
        ee_link="pestle_tip",
        robot_urdf="ur5e",
    ):
        self.start_joint_angle = start_joint_angle
        self.grinding_ready_joint_angle = grinding_ready_joint_angle
        self.gathering_ready_joint_angle = gathering_ready_joint_angle
        self.scooping_ready_joint_angle = scooping_ready_joint_angle
        self.moveit_executor = MoveitExecutor(move_group_name, ee_link)
        self.JTC_executor = JointTrajectoryControllerExecutor(
            ns=ns, robot_urdf=robot_urdf, tcp_link=ee_link
        )

    def _pose_stamped_to_list(self, pose_msg):
        return [
            pose_msg.pose.position.x,
            pose_msg.pose.position.y,
            pose_msg.pose.position.z,
            pose_msg.pose.orientation.x,
            pose_msg.pose.orientation.y,
            pose_msg.pose.orientation.z,
            pose_msg.pose.orientation.w,
        ]

    def execute_grinding(
        self,
        joint_trajectory,
        ee_link="pestle_tip",
        grinding_sec=10,
        moving_velocity_scale=0.1,
        moving_acceleration_scale=0.1,
        pre_motion=True,
        post_motion=True,
    ):
        if pre_motion:
            self.moveit_executor.execute_to_joint_goal(
                self.start_joint_angle,
                vel_scale=moving_velocity_scale,
                acc_scale=moving_acceleration_scale,
                execute=True,
            )
            self.moveit_executor.execute_to_joint_goal(
                self.grinding_ready_joint_angle,
                vel_scale=moving_velocity_scale,
                acc_scale=moving_acceleration_scale,
                execute=True,
            )

        self.JTC_executor.execute_to_joint_goal(
            joint_trajectory[0],
            time_to_reach=2,
            wait=True,
        )
        self.JTC_executor.execute_by_joint_trajectory(
            joint_trajectory,
            time_to_reach=grinding_sec,
        )

        if post_motion:
            exit_mortar_pose = self.moveit_executor.move_group.get_current_pose(ee_link)
            exit_mortar_pose = self._pose_stamped_to_list(exit_mortar_pose)
            exit_mortar_pose[2] += 0.1
            self.JTC_executor.execute_to_goal_pose(
                exit_mortar_pose,
                ee_link=ee_link,
                time_to_reach=3,
            )
            self.moveit_executor.execute_to_joint_goal(
                self.start_joint_angle,
                vel_scale=moving_velocity_scale,
                acc_scale=moving_acceleration_scale,
                execute=True,
            )

    def execute_gathering(
        self,
        waypoints,
        ee_link="spatula_tip",
        grinding_velocity_scale=0.1,
        grinding_acceleration_scale=0.1,
        moving_velocity_scale=0.3,
        moving_acceleration_scale=0.3,
    ):
        self.moveit_executor.execute_to_joint_goal(
            self.gathering_ready_joint_angle,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )

        success = self.moveit_executor.execute_cartesian_path_by_waypoints(
            waypoints=waypoints,
            jump_threshold=0.0,
            ee_link=ee_link,
            vel_scale=grinding_velocity_scale,
            acc_scale=grinding_acceleration_scale,
            avoid_collisions=False,
        )
        if success == True:
            exit_mortar_pose = waypoints[0]
            exit_mortar_pose[2] += 0.05
            self.moveit_executor.execute_cartesian_path_to_goal_pose(
                exit_mortar_pose,
                ee_link=ee_link,
                vel_scale=grinding_velocity_scale,
                acc_scale=grinding_acceleration_scale,
                avoid_collisions=False,
            )
            self.moveit_executor.execute_to_joint_goal(
                self.gathering_ready_joint_angle,
                vel_scale=moving_velocity_scale,
                acc_scale=moving_acceleration_scale,
                execute=True,
            )

        self.moveit_executor.execute_to_joint_goal(
            self.start_joint_angle,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )

        return success

    def execute_scooping(
        self,
        waypoints,
        ee_link="spoon_tip",
        scooping_velocity_scale=0.001,
        scooping_acceleration_scale=0.001,
        moving_velocity_scale=0.01,
        moving_acceleration_scale=0.01,
    ):
        self.moveit_executor.execute_to_joint_goal(
            self.scooping_ready_joint_angle,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )
        success = self.moveit_executor.execute_cartesian_path_by_waypoints(
            waypoints=waypoints,
            jump_threshold=0.0,
            ee_link=ee_link,
            vel_scale=scooping_velocity_scale,
            acc_scale=scooping_acceleration_scale,
            execute=True,
            avoid_collisions=False,
        )
        if success == True:
            exit_mortar_pose = waypoints[-1]
            exit_mortar_pose[2] += 0.1
            self.moveit_executor.execute_cartesian_path_to_goal_pose(
                exit_mortar_pose,
                ee_link=ee_link,
                vel_scale=moving_velocity_scale,
                acc_scale=moving_acceleration_scale,
                avoid_collisions=False,
            )
        self.moveit_executor.execute_to_joint_goal(
            self.start_joint_angle,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )

        return success

    def execute_powder_pouring(
        self,
        target_position,
        delta_euler=np.array([0, 0, pi / 3 * 2]),
        ee_link="spoon_tip",
        pouring_time=5,
        moving_velocity_scale=0.01,
        moving_acceleration_scale=0.01,
        number_of_motion_steps=10,
    ):
        self.moveit_executor.execute_to_joint_goal(
            self.start_joint_angle,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )

        pouring_ready_pose = self._pose_stamped_to_list(
            self.moveit_executor.move_group.get_current_pose(ee_link)
        )
        pouring_ready_pose[0:3] = target_position
        self.JTC_executor.execute_to_goal_pose(
            pouring_ready_pose,
            ee_link=ee_link,
            time_to_reach=5,
        )
        pose_list = []
        current_pose = self._pose_stamped_to_list(
            self.moveit_executor.move_group.get_current_pose(ee_link)
        )
        for i in range(number_of_motion_steps):
            delta_rot = Rotation.from_euler("xyz", delta_euler / number_of_motion_steps)
            current_rot = Rotation.from_quat(current_pose[3:])
            next_rot = current_rot * delta_rot
            current_position = current_pose[:3]
            next_pose = current_position + list(next_rot.as_quat())
            pose_list.append(next_pose)
            current_pose = next_pose

        # execute pouring
        self.moveit_executor.execute_cartesian_path_by_waypoints(
            pose_list,
            ee_link,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
        )

        # reverse pouring
        self.moveit_executor.execute_cartesian_path_by_waypoints(
            reversed(pose_list),
            ee_link,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
        )

        self.moveit_executor.execute_to_joint_goal(
            self.start_joint_angle,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )
