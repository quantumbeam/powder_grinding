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

    def execute_by_joint_trajectory(self, joint_trajectory, time_to_reach=5.0,strict_velocity_control=True):
        
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
            self._plot_joint_velocities_and_positions(constant_velocity_vector_list, joint_trajectory, time_to_reach)
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
        時間ベースでリサンプリングされた軌道に対し、手先等速運動を実現する関節速度を計算します。

        軌道全体の開始・終了姿勢と所要時間から、一定の目標手先速度（並進・角）を算出します。
        その後、軌道上の各点において、その時点でのヤコビアンと、算出した一定の目標手先速度を
        用いて関節速度を計算します。これにより、数値微分によるノイズの増幅を防ぎます。

        Args:
            waypoints (list):
                リサンプリングされた手先姿勢のリスト。各要素は [x, y, z, qx, qy, qz, qw]。
                クォータニオンは [x, y, z, w] の順です。
            joint_trajectory (list):
                リサンプリングされた関節角度(rad)のリスト。waypointsと長さが一致している必要があります。
            time_to_reach (float):
                全ウェイポイントを通過するための合計目標時間（秒）。

        Returns:
            list or None:
                各時刻における関節速度ベクトル(np.ndarray)のリスト。
        """
        if not waypoints or not joint_trajectory:
            rospy.logwarn("ウェイポイントまたは関節軌道が空です。")
            return None
        
        if time_to_reach <= 0:
            rospy.logwarn("time_to_reachは正の値である必要があります。")
            return None

        # --- 1. 軌道全体で一定となる目標手先速度を計算 ---
        # 軌道の開始と終了の姿勢を取得
        start_pos = np.array(waypoints[0][0:3])
        start_quat = np.array(waypoints[0][3:])
        end_pos = np.array(waypoints[-1][0:3])
        end_quat = np.array(waypoints[-1][3:])

        # 目標並進速度ベクトル v = (P_end - P_start) / T_total
        v_target = (end_pos - start_pos) / time_to_reach

        # 目標角速度ベクトル ω = (AngleAxis_total) / T_total
        rotations = Rotation.from_quat([start_quat, end_quat])
        relative_rotation = rotations[1] * rotations[0].inv()
        angle_axis = relative_rotation.as_rotvec()
        omega_target = angle_axis / time_to_reach

        # 6次元の目標手先速度ベクトル（この値はループ内で不変）
        cartesian_target_velocity = np.concatenate([v_target, omega_target])
        rospy.loginfo(f"Constant Target Cartesian Velocity: v={v_target.round(3)}, ω={omega_target.round(3)}")

        joint_velocities_list = []
        condition_numbers = []

        # --- 2. 各時刻の関節角度から関節速度を計算 ---
        for i, joint_angles in enumerate(joint_trajectory):
            try:
                # 現在の関節角度におけるヤコビアンを取得
                jacobian = self.kdl.jacobian(joint_angles)
                cond_num = np.linalg.cond(jacobian)
                condition_numbers.append(cond_num)

                # 擬似逆行列を計算
                jacobian_inv = np.linalg.pinv(jacobian)
                
                #【重要】ループの外で計算した一定の目標手先速度を常に使用する
                joint_velocities = jacobian_inv @ cartesian_target_velocity
                joint_velocities = np.array(joint_velocities).flatten()

            except np.linalg.LinAlgError:
                rospy.logwarn(f"警告: インデックス{i}のヤコビアン計算でエラー。速度をゼロにします。")
                joint_velocities = np.zeros(len(joint_angles))
            
            joint_velocities_list.append(joint_velocities.tolist())

        # --- 3. 統計情報の出力 ---
        if joint_velocities_list:
            all_velocities = np.array(joint_velocities_list)
            max_velocity = np.max(np.abs(all_velocities))
            rospy.loginfo(f"Calculated Joint Velocities - Max(abs): {max_velocity:.4f} rad/s")

        if condition_numbers:
            cond_array = np.array(condition_numbers)
            rospy.loginfo(f"Jacobian Condition Number Stats - "
                          f"Max: {np.max(cond_array):.2f}, "
                          f"Min: {np.min(cond_array):.2f}, "
                          f"Avg: {np.mean(cond_array):.2f}")
            if np.max(cond_array) > 100:
                rospy.logwarn("High condition number detected, indicating proximity to a singularity.")
            
        return joint_velocities_list
    
    def _plot_joint_velocities_and_positions(self, velocity_list, position_list, time_to_reach):
        """
        関節速度と位置をプロットする。
        
        Args:
            velocity_list (list): 各ポイントの関節速度のリスト
            position_list (list): 各ポイントの関節位置のリスト
            time_to_reach (float): 総実行時間
        """
        if not velocity_list or not position_list:
            rospy.logwarn("No velocities or positions to plot.")
            return
            
        try:
            import matplotlib.pyplot as plt
            import numpy as np
            
            # 時間軸を作成
            time_points_vel = np.linspace(0, time_to_reach, len(velocity_list))
            time_points_pos = np.linspace(0, time_to_reach, len(position_list))
            
            # 関節数を取得
            num_joints = len(velocity_list[0])
            
            # データを配列に変換
            velocities = np.array(velocity_list)
            positions = np.array(position_list)
            
            # プロット作成
            plt.figure(figsize=(15, 12))
            
            # 速度プロット
            for joint_idx in range(num_joints):
                plt.subplot(2, num_joints, joint_idx + 1)
                plt.plot(time_points_vel, velocities[:, joint_idx], 'b-', alpha=0.7, label='Line')
                plt.scatter(time_points_vel, velocities[:, joint_idx], c='red', s=8, alpha=0.6, label='Points')
                plt.title(f'Joint {joint_idx} Velocity')
                plt.xlabel('Time (s)')
                plt.ylabel('Velocity (rad/s)')
                plt.grid(True)
                if joint_idx == 0:
                    plt.legend()
            
            # 位置プロット
            for joint_idx in range(num_joints):
                plt.subplot(2, num_joints, num_joints + joint_idx + 1)
                plt.plot(time_points_pos, positions[:, joint_idx], 'g-', alpha=0.7, label='Line')
                plt.scatter(time_points_pos, positions[:, joint_idx], c='orange', s=8, alpha=0.6, label='Points')
                plt.title(f'Joint {joint_idx} Position')
                plt.xlabel('Time (s)')
                plt.ylabel('Position (rad)')
                plt.grid(True)
                if joint_idx == 0:
                    plt.legend()
                
            plt.tight_layout()
            plt.savefig('/tmp/joint_velocities_and_positions_plot.png', dpi=150, bbox_inches='tight')
            plt.show()
            rospy.loginfo("Joint velocity and position plot saved to /tmp/joint_velocities_and_positions_plot.png")
            
        except ImportError:
            rospy.logwarn("matplotlib not available. Skipping velocity and position plot.")
        except Exception as e:
            rospy.logerr(f"Error plotting joint velocities and positions: {e}")