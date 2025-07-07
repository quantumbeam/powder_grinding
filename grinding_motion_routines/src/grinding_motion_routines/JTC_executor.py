#!/usr/bin/env python3

import numpy as np
import rospy
from grinding_motion_routines.arm import Arm
from tqdm import tqdm
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp



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

    def is_outlier_iqr(self, data, k=1.5):
        """Detect outliers based on the IQR."""
        if len(data) < 4:  # Need at least 4 data points to calculate quartiles
            return np.zeros_like(data, dtype=bool)
        q1 = np.percentile(data, 25)
        q3 = np.percentile(data, 75)
        iqr = q3 - q1
        lower_bound = q1 - k * iqr
        upper_bound = q3 + k * iqr
        return (data < lower_bound) | (data > upper_bound)

    def execute_to_goal_pose(
        self,
        goal_pose,
        ee_link="",
        time_to_reach=5.0,
        joint_difference_limit=1.5*np.pi,
        max_attempts=100,
        wait=True,
        error_rate_check=False,
    ):
        """Supported pose is only x y z aw ax ay az"""
        if ee_link == "":
            ee_link = self.ee_link
        if self.ee_link != ee_link:
            self._change_ee_link(ee_link)

        best_ik = None
        best_dif = float("inf")

        dif_list = []
        start_joint = self.joint_angles()
        for i in tqdm(
                    range(max_attempts),
                    desc="Finding best IK solution",
                ):
            joint_goal = self._solve_ik(goal_pose)
            if joint_goal is None or np.any(joint_goal == "ik_not_found"):
                rospy.logerr("IK not found, Please check the pose")
                continue
            dif = np.sum(np.abs(np.array(start_joint[0:-1]) - np.array(joint_goal[0:-1])))
            dif_list.append(dif)
            if dif < best_dif:
                best_ik = joint_goal
                best_dif = dif

        if error_rate_check:
            dif_from_best = np.array(dif_list) - best_dif  # Difference of each dif from best_dif
            outlier_mask = self.is_outlier_iqr(dif_from_best)
            outlier_count = np.sum(outlier_mask)
            outlier_ratio = outlier_count / len(dif_list) if dif_list else 0.0
            rospy.loginfo(
                f"Best joint difference : best_joint_difference {best_dif} ( joint_difference_limit {joint_difference_limit} )"
                )
            rospy.loginfo(
                f"Statistical outlier ratio of joint difference (based on best_dif) (IQR): {outlier_ratio * 100:.2f}% ({outlier_count}/{len(dif_list)})"
            )

        if best_dif > joint_difference_limit:
            rospy.logerr(
            f"Best joint difference was too large: best_joint_difference {best_dif} > joint_difference_limit {joint_difference_limit}"
            )
            return None
        self.set_joint_positions(best_ik, t=time_to_reach, wait=wait)
        return best_ik

    def generate_joint_trajectory(
        self,
        waypoints,
        ee_link="",
        joint_difference_limit=0.03,
        max_attempts=1000,
        max_attempts_for_first_waypoint=100,
    ):
        """Supported pose is only list of [x y z aw ax ay az]"""
        if ee_link == "":
            ee_link = self.ee_link
        if self.ee_link != ee_link:
            self._change_ee_link(ee_link)

        joint_trajectory = []
        start_joint = self.joint_angles()
        total_waypoints = len(waypoints)  # 全体のwaypoints数を取得
        success_joint_difference_list=[]
        for i, pose in tqdm(
            enumerate(waypoints),
            total=total_waypoints,
            desc="Planning motion for waypoints",
        ):
            if i == 0:
                best_joint_difference = float("inf")
                for i in tqdm(
                    range(max_attempts_for_first_waypoint),
                    desc="Finding best IK solution for 1st waypoint",
                ):
                    ik_joint = self._solve_ik(pose, q_guess=start_joint)
                    if ik_joint is None or np.any(ik_joint == "ik_not_found"):
                        rospy.logerr("IK not found, Please check the pose")
                        continue
                    joint_difference = np.sum(np.abs(np.array(start_joint[0:-1]) - np.array(ik_joint[0:-1])))
                    if joint_difference < best_joint_difference:
                        best_ik_joint = ik_joint
                        best_joint_difference = joint_difference
                start_joint = best_ik_joint
                joint_trajectory.append(list(best_ik_joint))
            else:
                # 2番目以降のwaypoint
                retry_count = 0  # 各ポーズごとの再試行カウントをリセット
                joint_difference_list = []
                while retry_count < max_attempts:
                    ik_joint = self._solve_ik(pose, q_guess=start_joint)
                    if ik_joint is None or np.any(ik_joint == "ik_not_found"):
                        rospy.logerr("IK not found, Please check the pose")
                        return None
                    joint_difference = np.sum(np.abs(np.array(start_joint[0:-1]) - np.array(ik_joint[0:-1])))
                    joint_difference_list.append(joint_difference)
                    if joint_difference > joint_difference_limit:
                        retry_count += 1  # 再試行カウントを増やす
                        if retry_count >= max_attempts:
                            rospy.logerr(
                                f"Waypoint {i + 1}/{total_waypoints} failed after {max_attempts} trials, "
                                f"Joint difference was too large (min diff:{round(min(joint_difference_list),4)})"
                            )
                            return None
                        continue
                    else:
                        start_joint = ik_joint
                        joint_trajectory.append(list(ik_joint))
                        success_joint_difference_list.append(joint_difference)
                        break
        rospy.loginfo(f"Joint difference was in limit (max diff:{round(max(success_joint_difference_list),4)} min diff:{round(min(success_joint_difference_list),4)})")
        return joint_trajectory if joint_trajectory else None

    def execute_by_joint_trajectory(self, joint_trajectory, time_to_reach=5.0,strict_velocity_control=False):
        
        if strict_velocity_control:
            waypoints = []
            for joints in tqdm(joint_trajectory, desc="Generating waypoints from joint trajectory"):
                pose = self.kdl.forward(joints)
                waypoints.append(pose)
            constant_velocity_vector_list = self.generate_constant_velocity_vector(
                waypoints,
                joint_trajectory,
                time_to_reach,
            )
        else:
            constant_velocity_vector_list = None
        self.set_joint_trajectory(joint_trajectory, velocities=constant_velocity_vector_list, t=time_to_reach)

    def execute_by_waypoints(
        self,
        waypoints,
        joint_difference_limit,
        ee_link="",
        time_to_reach=5.0,
        max_attempts=1000,
        max_attempts_for_first_waypoint=100,
        strict_velocity_control=False,
    ):
        """Supported pose is only list of [x y z aw ax ay az]"""

        joint_trajectory = self.generate_joint_trajectory(
            waypoints,
            ee_link=ee_link,
            joint_difference_limit=joint_difference_limit,
            max_attempts=max_attempts,
            max_attempts_for_first_waypoint=max_attempts_for_first_waypoint
        )
        if strict_velocity_control:
            constant_velocity_vector_list = self.generate_constant_velocity_vector(
                waypoints,
                joint_trajectory,
                time_to_reach,
            )
        else:   
            constant_velocity_vector_list = None
        self.set_joint_trajectory(joint_trajectory,velocities=constant_velocity_vector_list, t=time_to_reach)

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
    def generate_constant_velocity_vector(
        self,
        waypoints,
        joint_trajectory,
        time_to_reach,
    ):
        """
        各ウェイポイント区間の目標手先速度を達成するための、代表的な関節速度を計算します。

        この関数は、手先がウェイポイント間を一定の並進速度で移動することを想定しています。
        各区間の関節速度は、その区間の開始時点の関節角度におけるヤコビアンを用いて計算されます。
        これは、区間内でのヤコビアンの変化は小さいという近似に基づいています。

        Args:
            waypoints (list):
                手先の姿勢を表すリストのリスト。各要素は [x, y, z, qx, qy, qz, qw] の形式。
                クォータニオンは [x, y, z, w] の順です。
            joint_trajectory (list):
                各ウェイポイントに対応する関節角度(rad)のリスト。
            time_to_reach (float):
                全ウェイポイントを通過するための合計目標時間（秒）。

        Returns:
            list or None:
                各区間に対する関節速度ベクトル(np.ndarray)のリスト。
                計算不可能な場合はNoneを返します。
        """
        # 1. 全体の並進移動距離を計算
        total_linear_distance = 0.0
        segment_distances = []
        for i in range(len(waypoints) - 1):
            pos_start = np.array(waypoints[i][0:3])
            pos_end = np.array(waypoints[i+1][0:3])
            dist = np.linalg.norm(pos_end - pos_start)
            total_linear_distance += dist
            segment_distances.append(dist)

        if total_linear_distance <= 1e-6:
            rospy.logwarn("ウェイポイント間の合計距離がほぼゼロです。計算をスキップします。")
            return None

        # 2. 全体で一定となる目標並進速度の大きさを計算
        linear_velocity_magnitude = total_linear_distance / time_to_reach

        joint_velocities_list = []
        for i in range(len(waypoints) - 1):
            # --- セグメントiの情報を設定 ---
            start_pos = np.array(waypoints[i][0:3])
            start_quat = np.array(waypoints[i][3:])
            end_pos = np.array(waypoints[i+1][0:3])
            end_quat = np.array(waypoints[i+1][3:])

            rotations = Rotation.from_quat([start_quat, end_quat])

            # --- このセグメントの目標速度を計算 ---
            # セグメントの移動にかかる時間を計算
            segment_duration = segment_distances[i] / linear_velocity_magnitude
            if segment_duration < 1e-6:
                # 移動時間が非常に短い場合はゼロ速度として扱う
                joint_velocities_list.append(np.zeros(len(joint_trajectory[i])))
                continue

            # 目標並進速度ベクトル v = 距離 / 時間
            v_target = (end_pos - start_pos) / segment_duration

            # 目標角速度ベクトル ω = 回転(rad) / 時間
            relative_rotation = rotations[1] * rotations[0].inv()
            angle_axis = relative_rotation.as_rotvec()  # 回転ベクトル [axis*angle]
            omega_target = angle_axis / segment_duration

            # 6次元の目標手先速度ベクトルを作成
            cartesian_target_velocity = np.concatenate([v_target, omega_target])

            # --- 関節速度を計算 ---
            try:
                # 注意: この区間の開始点(i)の関節角度でヤコビアンを計算しています。
                # これは、区間が短く、ヤコビアンの変化が小さいという前提での近似です。
                jacobian_inv = self.kdl.jacobian_pseudo_inverse(joint_trajectory[i])

                # ヤコビアンの擬似逆行列を用いて関節速度を計算
                joint_velocities = jacobian_inv @ cartesian_target_velocity
                joint_velocities = np.array(joint_velocities).flatten()
            except np.linalg.LinAlgError:
                rospy.logwarn(f"警告: 区間{i}のヤコビアン計算でエラー。速度をゼロにします。")
                joint_velocities = np.zeros(len(joint_trajectory[i]))
            
            joint_velocities_list.append(joint_velocities.tolist())

        # 計算された関節速度の統計情報をログに出力
        if joint_velocities_list:
            all_velocities = np.array(joint_velocities_list)
            max_velocity = np.max(np.abs(all_velocities))
            rospy.loginfo(f"Calculated Joint Velocities - Max(abs): {max_velocity:.4f} rad/s")

        # 最初のポイントに0速度を追加（軌道の開始時は静止状態）
        if joint_velocities_list:
            first_zero_velocity = [0.0] * len(joint_velocities_list[0])
            joint_velocities_list.insert(0, first_zero_velocity)
            
        return joint_velocities_list