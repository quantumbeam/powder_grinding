#!/usr/bin/env python3

import numpy as np
import rospy
from grinding_motion_routines.arm import Arm


class JointTrajectoryControllerExecutor(Arm):
    """Motion Executor including IK and JointTrajectoryController(JTC) ."""

    def __init__(
        self,
        robot_urdf_pkg,
        robot_urdf_file_name,
        joint_trajectory_controller_name,
        tcp_link,
        ns=None,
        joint_names_prefix=None,
        ft_topic=None,
        gripper=False,
        ik_solver="trac_ik",
        solve_type="Distance",
    ):

        if joint_names_prefix is None:
            joint_names_prefix = ""
        super().__init__(
            robot_urdf_pkg,
            robot_urdf_file_name,
            joint_trajectory_controller_name,
            gripper=gripper,
            namespace=ns,
            joint_names_prefix=joint_names_prefix,
            ee_link=tcp_link,
            ft_topic=ft_topic,
            ik_solver=ik_solver,
            solve_type=solve_type,
        )
        self.joint_names_prefix = joint_names_prefix
        self.init_end_effector_link = tcp_link
        self.solve_type = solve_type

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
        return self.set_target_pose(goal_pose, t=time_to_reach, wait=wait)

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
            start_joint = self.joint_angles()
            for pose in waypoints:
                joints = self._solve_ik(pose, q_guess=start_joint)
                if np.any(joints == "ik_not_found") or joints is None:
                    rospy.logerr("IK not found, Try again")
                    break
                else:
                    start_joint = joints
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
        self._init_ik_solver(self.base_link, self.ee_link, self.solve_type)
