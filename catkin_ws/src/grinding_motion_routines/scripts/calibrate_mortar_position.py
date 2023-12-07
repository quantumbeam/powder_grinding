#!/usr/bin/env python3

# ROS-related imports

from scipy.spatial.transform import Rotation
from threading import Event
import rospy
import tf2_ros

from std_srvs.srv import Empty
from geometry_msgs.msg import WrenchStamped
from grinding_motion_routines.motion_generator import MotionGenerator
from grinding_motion_routines.moveit_executor import MoveitExecutor
from grinding_motion_routines.load_planning_scene import PlanningScene
from grinding_motion_routines.marker_display import MarkerDisplay

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
        self.marker_display = MarkerDisplay("calibration_marker_array")
        # Initialize MoveitExecutor
        self.moveit = MoveitExecutor(move_group_name, ee_link)
        self.grinding_eef_link = rospy.get_param("~grinding_eef_link")

        self.scene = PlanningScene(self.moveit.move_group)

        self.mortar_position = rospy.get_param("~mortar_top_position")
        self.mortar_scale = rospy.get_param("~mortar_inner_scale")
        self.force_threshold = rospy.get_param("~force_threshold")
        self.caliblate_log_dir_path = rospy.get_param("~caliblate_log_dir_path")

        self.filterd_mortar_wrench_topic = rospy.get_param("~FT_sensor_under_mortar_topic")
        self.filterd_UR_wrench_topic = rospy.get_param("~UR_FT_sensor_topic")
        rospy.Subscriber(
            self.filterd_mortar_wrench_topic,WrenchStamped,self._mortar_wrench_callback,queue_size=1)
        rospy.Subscriber(
            self.filterd_UR_wrench_topic,WrenchStamped,self._UR_wrench_callback,queue_size=1)

        # Subscribe to wrench service
        rospy.loginfo("Waiting for wrench service")
        self.zero_mortar_wrench_service_name = rospy.get_param("~zero_wrench_under_mortar_service")
        rospy.wait_for_service(self.zero_mortar_wrench_service_name)
        self.zero_UR_wrench_service_name = rospy.get_param("~zero_wrench_UR_service")
        rospy.wait_for_service(self.zero_UR_wrench_service_name)
        rospy.wait_for_service("/start_stop_recording")
        try:
            self.xy_calibrate_service_client = rospy.ServiceProxy(
                "/start_stop_recording", PositionCalibrateVector
            )
            self.ft_mortar_service = rospy.ServiceProxy(self.zero_mortar_wrench_service_name, Empty)
            self.ft_UR_service = rospy.ServiceProxy(self.zero_UR_wrench_service_name, Empty)

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            exit()

    def _zero_mortar_ft_sensor(self):
        time.sleep(1)
        rospy.loginfo("Zeroing FT sensor")
        self.ft_mortar_service.call()
        time.sleep(1)

    def _zero_UR_ft_sensor(self):
        time.sleep(1)
        rospy.loginfo("Zeroing FT sensor")
        self.ft_UR_service.call()
        time.sleep(1)

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

    def _mortar_wrench_callback(self, wrench_msg):
        self.current_mortar_force = wrench_msg.wrench.force
        self.current_mortar_torque = wrench_msg.wrench.torque
    
    def _UR_wrench_callback(self, wrench_msg):
        self.current_UR_force = wrench_msg.wrench.force
        self.current_UR_torque = wrench_msg.wrench.torque

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

    def _move_and_check_force_z(self, delta,force_z):
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

        rospy.loginfo(f"Force z after move: {force_z}")

        if np.abs(force_z) > self.force_threshold:
            return True
        else:
            return False

    def _move_verticale_pose_of_current_position(self):
        # moving to vertical pose of current position
        vertical_pose = self._pose_stumped_to_list(
            self.moveit.move_group.get_current_pose()
        )
        quat = Rotation.from_euler("xyz", [0, pi, 0]).as_quat()
        vertical_pose[3:] = quat
        self.moveit.execute_to_goal_pose(vertical_pose, ee_link=self.grinding_eef_link)

    def move_to_position_above_mortar(self):
        rospy.loginfo("Moving to start position")
        q = Rotation.from_euler("xyz", [0, pi, 0]).as_quat()
        pose = [
            self.mortar_position["x"],
            self.mortar_position["y"],
            self.mortar_position["z"] + 0.05,
        ] + list(q)
        self.moveit.execute_cartesian_path_to_goal_pose(
            pose,
            ee_link=self.grinding_eef_link,
            vel_scale=0.1,
            acc_scale=0.1,
        )


    def caliblate_pestle_tip_position(self, boad_tickness):
        self._move_verticale_pose_of_current_position()
        self._zero_UR_ft_sensor()

        delta = np.zeros(7)
        step = rospy.get_param("~Z_calibration_step")
        for _ in range(1000):
            delta[2] = -step
            result = self._move_and_check_force_z(delta,self.current_UR_force.z)
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

    def caliblate_mortar_hight(self, boad_tickness):
        self._move_verticale_pose_of_current_position()
        self._zero_UR_ft_sensor()

        delta = np.zeros(7)
        step = rospy.get_param("~Z_calibration_step")
        for _ in range(1000):
            delta[2] = -step
            result = self._move_and_check_force_z(delta,self.current_mortar_force.z)
            if result:
                rospy.loginfo("Calibration finished")
                pestle_tip_pose = self.moveit.move_group.get_current_pose() 
                mortar_position_Z = pestle_tip_pose.pose.position.z - boad_tickness
                rospy.loginfo("Mortar position Z: %s", mortar_position_Z)
                self.mortar_position["z"] = mortar_position_Z
                rospy.set_param("~mortar_top_position", self.mortar_position)
                self.scene.init_planning_scene()
                break
            if rospy.is_shutdown():
                break

    def set_rough_mortar_position(self):
        self._move_verticale_pose_of_current_position()
        self.rough_mortar_position = self.moveit.move_group.get_current_pose()

        rospy.loginfo(f"Rough mortar position: {self.rough_mortar_position}")
        self.mortar_position["x"] = self.rough_mortar_position.pose.position.x
        self.mortar_position["y"] = self.rough_mortar_position.pose.position.y
        rospy.set_param("~mortar_top_position", self.mortar_position)
        self.scene.init_planning_scene()

    def fine_tuning_mortar_position(self):
        date = time.strftime("%Y%m%d_%H%M%S_")
        while not rospy.is_shutdown():
            variance_threshold = rospy.get_param("~calibrated_variance_of_force")

            self.move_to_position_above_mortar()
            self._zero_mortar_ft_sensor()
            generator = MotionGenerator(self.mortar_position, self.mortar_scale)

            # Create waypoints
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
            )

            self.marker_display.display_waypoints(waypoints, clear=True)

            # Execute waypoints
            rospy.loginfo("Goto initial position")
            self.moveit.execute_cartesian_path_by_waypoints(
                waypoints[0:10],
                ee_link=self.grinding_eef_link,
                vel_scale=0.1,
                acc_scale=0.1,
            )
            rospy.sleep(1)  # wait for force sensor to settle

            rospy.loginfo("Start calibration grinding motion")
            self.xy_calibrate_service_client("start")
            self.moveit.execute_cartesian_path_by_waypoints(
                waypoints, ee_link=self.grinding_eef_link, vel_scale=1, acc_scale=1
            )
            response = self.xy_calibrate_service_client("stop")
            self.move_to_position_above_mortar()
            rospy.loginfo("Calibration grinding motion finished")

            # Update mortar position
            # direction of x axis of end-effector is same as force sensor
            # direction of y axis of end-effector is opposite from force sensor
            step = rospy.get_param("~XY_calibration_step")
            new_mortar_position = self.mortar_position.copy()
            if abs(response.variance_force_x) < 1:
                rospy.loginfo("No movement in x direction")
            elif response.integral_force_x > 0:
                new_mortar_position["x"] -= step
            elif response.integral_force_x < 0:
                new_mortar_position["x"] += step
    
            if abs(response.variance_force_y) < 1:
                rospy.loginfo("No movement in y direction")
            elif response.integral_force_y > 0:
                new_mortar_position["y"] -= step
            elif response.integral_force_y < 0:
                new_mortar_position["y"] += step

            rospy.loginfo("Current mortar position: %s", self.mortar_position)
            rospy.loginfo("New mortar position: %s", new_mortar_position)
            writer = open(
                self.caliblate_log_dir_path
                + date
                + "calibration_result.txt",
                "a",
            )
            writer.write(
                f"Current mortar position: {self.mortar_position},\n New mortar position: {new_mortar_position},\n response: {response}\n\n"
            )
            writer.close()

            rospy.set_param("~mortar_top_position", new_mortar_position)
            self.scene.init_planning_scene()
            generator.update_mortar_position(new_mortar_position)
            self.mortar_position = new_mortar_position

            # Check variance of force
            if abs(response.variance_force_z) < variance_threshold:
                rospy.loginfo("Calibration finished")
                break

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

    def initialize_scene(self):
        self.scene.init_planning_scene()


def main():
    rospy.init_node("mortar_position_calibration", anonymous=True)

    arm = MortarPositionFineTuning()

    while not rospy.is_shutdown():
        key = input(
            "Select calibration mode and press enter\n"
            + "0: Exit\n"
            + "1: Calibrate pestle tip position\n"
            + "2: Calibrate Mortar Z\n"
            + "3: Set rough Mortar XY position as pestle tip\n"
            + "4: Fine tuning Mortar XY position\n"
            + "5: Calibrate grinding target force\n"
            + "6: Subscribe wrench(debug)\n"
            + "7: Initialize scene\n"
        )
        if key == "0":
            rospy.loginfo("Exit calibration mode")
            rospy.signal_shutdown("finish")
        elif key == "1" or key == "2":
            try:
                tickness = input(
                    "Enter the tickness of board in mm, then press enter to start: "
                )
                if tickness == "":
                    print("Invalid input. Please enter a number.\n")
                else:
                    if key == "1":
                        arm.caliblate_pestle_tip_position(float(tickness))
                    elif key == "2":
                        arm.caliblate_mortar_hight(float(tickness))
            except ValueError:
                print("Invalid input. Please enter a number.\n")
        elif key == "3":
            arm.set_rough_mortar_position()
        elif key == "4":
            arm.fine_tuning_mortar_position()
        elif key == "5":
            arm.caliblate_grinding_motion_to_target_force()
        elif key == "6":
            arm.subscribe_wrench()
        elif key == "7":
            arm.initialize_scene()


if __name__ == "__main__":
    main()
