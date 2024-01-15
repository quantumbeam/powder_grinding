#!/usr/bin/env python3

from pickle import FALSE
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf.transformations as tf

import numpy as np

from ur_control.arm import Arm


class JointTrajectoryControllerExecutor(Arm):
    """Motion Executor including IK and JointTrajectoryController(JTC) ."""

    def __init__(self, ns, robot_urdf, tcp_link, ft_topic="wrench",ik_solver='trac_ik'):
        joint_names_prefix = ns + "_" if ns else ""
        super().__init__(
            gripper=False,
            namespace=ns,
            joint_names_prefix=joint_names_prefix,
            robot_urdf=robot_urdf,
            ee_link=tcp_link,
            ft_topic=ft_topic,
            ik_solver=ik_solver,
        )
        self.joint_names_prefix = joint_names_prefix
        self.init_end_effector_link = tcp_link

    def execute_to_goal_pose(
        self,
        goal_pose,
        ee_link="",
        time_to_reach=5.0,
        wait=True,
    ):
        """Supported pose is only x y z aw ax ay az"""
        if ee_link == "":
            ee_link = self.ee_link
        if self.ee_link != ee_link:
            self._change_ee_link(ee_link)
        self.set_target_pose(goal_pose, t=time_to_reach, wait=wait)

    def generate_joint_trajectory(
        self,
        waypoints,
        total_joint_limit,
        ee_link="",
        trial_number=10,
    ):
        """Supported pose is only list of [x y z aw ax ay az]"""
        if ee_link == "":
            ee_link = self.ee_link
        if self.ee_link != ee_link:
            self._change_ee_link(ee_link)

        for i in range(trial_number):
            joint_trajectory = []
            for pose in waypoints:
                joints = self._solve_ik(pose)
                if np.any(joints == "ik_not_found") or joints is None:
                    rospy.logerr("IK not found")
                else:
                    joint_trajectory.append(list(joints))
            if len(joint_trajectory) == 0:
                rospy.logerr("Skip this trajectory")
                continue
            total_joint_difference = np.sum(
                np.max(joint_trajectory, axis=0) - np.min(joint_trajectory, axis=0)
            )
            if total_joint_difference > total_joint_limit:
                rospy.logerr("Trajectory is out of joint limit")
                rospy.logerr(f"Total joints difference: {total_joint_difference}")
                rospy.logerr("Try again")
            else:
                rospy.loginfo(
                    f"Trajectory is in total joint limit:{total_joint_difference}"
                )
                return joint_trajectory
        rospy.logerr("Failed to find trajectory")
        return None

    def execute_by_joint_trajectory(self, joint_trajectory, time_to_reach=5.0):
        self.set_joint_trajectory(joint_trajectory, t=time_to_reach)

    def execute_by_waypoints(
        self,
        waypoints,
        total_joint_limit,
        ee_link="",
        time_to_reach=5.0,
        trial_number=10,
    ):
        """Supported pose is only list of [x y z aw ax ay az]"""

        joint_trajectory = self.generate_joint_trajectory(
            waypoints,
            total_joint_limit,
            ee_link=ee_link,
            trial_number=trial_number,
        )
        self.set_joint_trajectory(joint_trajectory, t=time_to_reach)

    def execute_to_joint_goal(self, joint_goal, time_to_reach=5.0, wait=True):
        self.set_joint_positions(joint_goal, t=time_to_reach, wait=wait)

    def _change_ee_link(self, new_ee_link):
        old_ee_link = self.ee_link
        rospy.loginfo(
            "============ Cange End effector link: %s to %s"
            % (old_ee_link, new_ee_link)
        )
        self.ee_link = (
            new_ee_link
            if self.joint_names_prefix is None
            else self.joint_names_prefix + new_ee_link
        )
        self._init_ik_solver(self.base_link, self.ee_link)
