#!/usr/bin/env python3

import numpy as np
import rospy
from grinding_motion_routines.arm import Arm
from tqdm import tqdm

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
        trial_number=5,
    ):
        """Supported pose is only list of [x y z aw ax ay az]"""
        if ee_link == "":
            ee_link = self.ee_link
        if self.ee_link != ee_link:
            self._change_ee_link(ee_link)

        joint_trajectory = []
        start_joint = self.joint_angles()

        total_waypoints = len(waypoints)  # 全体のwaypoints数を取得
        for i, pose in tqdm(enumerate(waypoints), total=total_waypoints, desc="Planning motion for waypoints"):
            retry_count = 0  # 各ポーズごとの再試行カウントをリセット
            while retry_count < trial_number:  # trial_number回までループ
                ik_joint = self._solve_ik(pose, q_guess=start_joint)
                if np.any(ik_joint == "ik_not_found") or ik_joint is None:
                    rospy.logerr("IK not found, Please check the pose")
                    return None
                else:
                    joint_difference = np.sum(np.array(start_joint[0:-1]) - np.array(ik_joint[0:-1]))
                    if joint_difference > total_joint_limit:
                        retry_count += 1  # 再試行カウントを増やす
                        if retry_count >= trial_number:  # トライアル数を超えた場合のエラーログ
                            rospy.logerr(
                                f"Waypoint {i + 1}/{total_waypoints} failed after {trial_number} trials, "
                                f"Joint difference was too large ({joint_difference})."
                            )
                            return None
                        continue  # 同じポーズに対して再計算
                    else:
                        start_joint = ik_joint  # 成功した場合、start_jointを更新
                        joint_trajectory.append(list(ik_joint))
                        break  # 成功したので、次のポーズへ
        
        # すべてのwaypointsを処理した後に結果を返す
        return joint_trajectory if joint_trajectory else None

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
