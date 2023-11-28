#!/usr/bin/env python3

# ROS-related imports
import rospy
import tf.transformations as tf
from threading import Event
import tf2_ros

from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped
from grinding_motion_routines.motion_generator import MotionGenerator
from grinding_motion_routines.moveit_executor import MoveitExecutor
from grinding_motion_routines.srv import PositionCalibrateVector

import argparse
import numpy as np
import time
from math import pi


class MortarPositionFineTuning:
    def __init__(
        self,
        move_group_name="manipulator",
        ee_link="pestle_tip",
    ):
        self.moveit = MoveitExecutor(move_group_name, ee_link)
        self.grinding_eef_link = rospy.get_param("~grinding_eef_link")

        self.mortar_position = rospy.get_param("~mortar_position")
        self.mortar_scale = rospy.get_param("~mortar_inner_scale")
        self.force_threshold = rospy.get_param("~force_threshold")
        self.total_joints_limits_for_trajectory = 0.1

        self.filterd_wrench_topic = rospy.get_param("~filterd_wrench_topic")
        rospy.Subscriber(
            self.filterd_wrench_topic,
            WrenchStamped,
            self._wrench_callback,
            queue_size=1,
        )

        # Subscribe to wrench service
        rospy.loginfo("Waiting for wrench service")
        self.zero_wrench_service_name = rospy.get_param("~zero_wrench_service")
        rospy.wait_for_service(self.zero_wrench_service_name)
        rospy.wait_for_service("/start_stop_recording")
        try:
            self.service_client = rospy.ServiceProxy(
                "/start_stop_recording", PositionCalibrateVector
            )
            self.ft_service = rospy.ServiceProxy(self.zero_wrench_service_name, Empty)

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            exit()

    def _zero_ft_sensor(self):
        time.sleep(1)
        rospy.loginfo("Zeroing FT sensor")
        self.ft_service.call()
        time.sleep(1)

    def _wrench_callback(self, wrench_msg):
        self.current_force = wrench_msg.wrench.force
        self.current_torque = wrench_msg.wrench.torque

    def _listen_tf(self, parent_frame, child_frame):
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        rate = rospy.Rate(10.0)
        rate.sleep()

        try:
            trans = tfBuffer.lookup_transform(child_frame, parent_frame, rospy.Time())
            return trans
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as err:
            print("tf listen error%s" % err)
            return err

    def move_to_position_above_mortar(self):
        rospy.loginfo("Moving to start position")
        q = tf.quaternion_from_euler(pi, 0, pi)
        pose = [
            self.mortar_position["x"],
            self.mortar_position["y"],
            self.mortar_position["z"] + 0.1,
        ] + list(q)
        self.moveit.execute_cartesian_path_to_goal_pose(
            pose,
            ee_link=self.grinding_eef_link,
            vel_scale=0.1,
            acc_scale=0.1,
        )
        self._zero_ft_sensor()

    def caliblate_pestle_tip_position(self, boad_tickness):
        # moving to vertical pose of current position
        vertical_pose = self._pose_stumped_to_list(
            self.moveit.move_group.get_current_pose()
        )
        orientation = tf.quaternion_from_euler(0, pi, 0)
        vertical_pose[3:] = orientation
        print(vertical_pose)
        self.moveit.execute_to_goal_pose(vertical_pose, ee_link=self.grinding_eef_link)

        delta = np.zeros(7)
        for _ in range(1000):
            delta[2] = -0.001
            result = self._move_and_check_force(delta)
            if result:
                rospy.loginfo("Calibration finished")
                # calculate the pestle tip position
                robot_base_palte_tickness = rospy.get_param(
                    "~robot_base_palte_tickness"
                )
                tool0_pose = self._listen_tf("base_link", "tool0")
                pestle_tip_length = (
                    abs(tool0_pose.transform.translation.z)
                    + robot_base_palte_tickness
                    - boad_tickness
                )

                rospy.loginfo("Pestle tip length: %s", pestle_tip_length)
                rospy.loginfo("You can update the pestle tip length in URDF file")
                break
            if rospy.is_shutdown():
                break

    def _pose_stumped_to_list(self, pose):
        return [
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ]

    def caliblate_mortar_hight(self, boad_tickness):
        delta = np.zeros(7)

        for _ in range(1000):
            delta[2] = -0.001
            result = self._move_and_check_force(delta)
            if result:
                rospy.loginfo("Calibration finished")
                pestle_tip_pose = self._pose_stumped_to_list(
                    self.moveit.move_group.get_current_pose()
                )
                mortar_position_Z = (
                    pestle_tip_pose.transform.translation.z - boad_tickness
                )
                rospy.loginfo("Mortar position Z: %s", mortar_position_Z)
                break
            if rospy.is_shutdown():
                break

    def _move_and_check_force(self, delta):
        current_pose = self._pose_stumped_to_list(
            self.moveit.move_group.get_current_pose()
        )
        next_pose = current_pose + delta

        self.moveit.execute_cartesian_path_to_goal_pose(
            next_pose,
            ee_link=self.grinding_eef_link,
            vel_scale=0.1,
            acc_scale=0.1,
        )

        rospy.loginfo(f"Force z after move: {self.current_force.z}")

        if np.abs(self.current_force.z) > self.force_threshold:
            return True
        else:
            return False

    def fine_tuning_mortar_position(self):
        generator = MotionGenerator()
        self.move_to_position_above_mortar()

        for i in range(10):
            waypoints = generator.create_circular_waypoints(
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
                debug=False,
                format="list",
            )

            # Inverse kinematics
            # 収束判定つき、ジョイントリミットを超えていたら再計算
            isConverged = False
            while not isConverged:
                joint_trajectory = []
                for pose in waypoints:
                    joints = self._solve_ik(pose)

                    if joints == "ik_not_found":
                        rospy.logerr("IK not found")
                    else:
                        joint_trajectory.append(joints)
                joints_difference = np.max(joint_trajectory, axis=0) - np.min(
                    joint_trajectory, axis=0
                )
                sum_joints_difference = np.sum(joints_difference)
                if sum_joints_difference > self.sum_joints_limits_for_trajectory:
                    rospy.logerr("Trajectory is out of joints limits -> recalculating")
                    rospy.logerr(f"Sum of joints difference: {sum_joints_difference}")
                else:
                    rospy.loginfo(f"Trajectory is in joints limits:{joints_difference}")
                    isConverged = True

            self.set_joint_positions(joint_trajectory[0], t=3, wait=True)
            self.service_client("start")
            self.set_joint_trajectory(joint_trajectory, t=5)
            response = self.service_client("stop")

            # Move to start position
            self.move_to_position_above_mortar()

            # Update mortar position
            mortar_position = generator.mortar_center_position
            if response.integral_force_x > 0:
                mortar_position.x -= self.calibration_step
            elif response.integral_force_x < 0:
                mortar_position.x += self.calibration_step

            if response.integral_force_y > 0:
                mortar_position.y -= self.calibration_step
            elif response.integral_force_y < 0:
                mortar_position.y += self.calibration_step
            generator.update_mortar_position(mortar_position)
            rospy.loginfo(f"New mortar position: {mortar_position}")

    def subscribe_wrench(self):
        rospy.loginfo("Subscribing to wrench for 10 seconds")
        self.timer_event = Event()
        self.display_timer = rospy.Timer(rospy.Duration(1), self._display_wrench_data)
        self.stop_timer = rospy.Timer(
            rospy.Duration(10), self._stop_wrench_subscription, oneshot=True
        )
        self.timer_event.wait()

    def _display_wrench_data(self, event):
        if not self.timer_event.is_set():
            force = np.array(
                [self.current_force.x, self.current_force.y, self.current_force.z]
            )
            torque = np.array(
                [self.current_torque.x, self.current_torque.y, self.current_torque.z]
            )
            rospy.loginfo(f"Force: {force}, Torque: {torque}")

    def _stop_wrench_subscription(self, event):
        self.display_timer.shutdown()
        self.timer_event.set()
        rospy.loginfo("Stopping wrench subscription after 10 seconds")


def main():
    rospy.init_node("mortar_position_calibration", anonymous=True)

    arm = MortarPositionFineTuning()

    while not rospy.is_shutdown():
        key = input(
            "Select calibration mode and press enter\n"
            + "0: Exit\n"
            + "1: Calibrate pestle tip position\n"
            + "2: Set rough Mortar XY position as pestle tip\n"
            + "3: Move to position above Mortar\n"
            + "4: Calibrate Mortar Z\n"
            + "5: Fine tuning Mortar XY position\n"
            + "6: Calibrate grinding target force\n"
            + "7: Subscribe wrench\n"
        )
        if key == "0":
            rospy.loginfo("Exit calibration mode")
            rospy.signal_shutdown("finish")
        elif key == "1":
            try:
                tickness = input(
                    "Enter the tickness of board in mm, then press enter to start: "
                )
                if tickness == "":
                    print("Invalid input. Please enter a number.\n")
                else:
                    arm.caliblate_pestle_tip_position(float(tickness))
            except ValueError:
                # 数値に変換できない場合のエラーメッセージ
                print("Invalid input. Please enter a number.\n")

        elif key == "3":
            arm.move_to_position_above_mortar()
        elif key == "4":
            tickness = input(
                "Enter the tickness of board in mm, then press enter to start: "
            )
            arm.caliblate_mortar_hight(tickness)

        elif key == "5":
            arm.fine_tuning_mortar_position()
        elif key == "6":
            arm.caliblate_grinding_motion_to_target_force()
        elif key == "7":
            arm.subscribe_wrench()


if __name__ == "__main__":
    main()
