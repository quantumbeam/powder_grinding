#!/usr/bin/env python3

# ROS-related imports
import rospy
from geometry_msgs.msg import WrenchStamped

from ur_control.arm import Arm
from ur_control import transformations
import tf.transformations as tf

from grinding_motion_routines.motion_routines import MotionGenerator
from grinding_motion_routines.srv import PositionCalibrateVector

import argparse
import numpy as np
import time
from math import pi


class MortarPositionFineTuning(Arm):
    def __init__(
        self, use_gripper, ns, joint_names_prefix, robot_urdf, tcp_link, ft_topic
    ) -> None:
        super().__init__(
            gripper=use_gripper,
            namespace=ns,
            joint_names_prefix=joint_names_prefix,
            robot_urdf=robot_urdf,
            robot_urdf_package=robot_urdf,
            ee_link=tcp_link,
            ft_topic=ft_topic,
        )

        self.mortar_position = rospy.get_param("~mortar_position")
        self.mortar_scale = rospy.get_param("~mortar_inner_scale")
        self.force_threshold = rospy.get_param(
            "~force_threshold_of_morat_hight_calibration"
        )
        self.calibration_step = rospy.get_param("~calibration_step")
        self.sum_joints_limits_for_trajectory = rospy.get_param(
            "~sum_joints_limits_for_trajectory"
        )

        # Subscribe to wrench service
        rospy.wait_for_service("/start_stop_recording")
        try:
            self.service_client = rospy.ServiceProxy(
                "/start_stop_recording", PositionCalibrateVector
            )

        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        # Move to start position
        self._goto_start_position()

    def _goto_start_position(self):
        rospy.loginfo("Moving to start position")
        q = tf.quaternion_from_euler(pi, 0, pi)
        pose = [
            self.mortar_position["x"],
            self.mortar_position["y"],
            self.mortar_position["z"] + 0.04,
        ] + list(q)
        self.set_target_pose(pose, wait=True, t=3)

        time.sleep(1)
        rospy.loginfo("Zeroing FT sensor")
        self.zero_ft_sensor()
        time.sleep(1)

    def caliblate_mortar_hight(self):
        # Move to start position
        self._goto_start_position()

        # Move to near mortar bottom
        xc = transformations.transform_pose(self.end_effector(), [0, 0, -0.03, 0, 0, 0])
        self.set_target_pose(pose=xc, wait=True, t=2)

        # Move to mortar bottom
        delta = np.zeros(6)
        delta[2] = -0.0001
        for i in range(1000):
            x = self.end_effector()
            xc = transformations.transform_pose(x, delta)
            self.set_target_pose(pose=xc, wait=True, t=0.01)
            force_z = self.get_ee_wrench()[2]
            rospy.loginfo(f"Force z: {force_z}")
            if np.abs(force_z) > self.force_threshold:
                break
        rospy.loginfo("Calibration finished")
        print("EE Pose:", np.round(self.end_effector(), 5).tolist())
        print(
            "Moratr hight:",
            np.round(self.end_effector()[2] + self.mortar_scale["z"], 5).tolist(),
        )

    def fine_tuning_mortar_position(self):
        generator = MotionGenerator()
        self._goto_start_position()

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
            self._goto_start_position()

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


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt, description=main.__doc__)
    parser.add_argument(
        "--namespace",
        type=str,
        help="Namespace of arm (useful when having multiple arms)",
        default=None,
    )
    parser.add_argument(
        "--gripper", action="store_true", help="enable gripper commands"
    )
    parser.add_argument(
        "--robot",
        type=str,
        help='Version of Universal Robot arm. Default="ur3e"',
        default="ur3e",
    )
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("mortar_position_calibration", anonymous=True)

    ns = args.namespace
    robot_urdf = "ur5e"
    rospackage = None
    tcp_link = "pestle_tip"
    use_gripper = args.gripper
    joint_names_prefix = ns + "_" if ns else ""
    ft_topic = "wrench"

    arm = MortarPositionFineTuning(
        use_gripper,
        ns,
        joint_names_prefix,
        robot_urdf,
        tcp_link,
        ft_topic,
    )

    # arm.caliblate_mortar_hight()
    arm.fine_tuning_mortar_position()


if __name__ == "__main__":
    main()
