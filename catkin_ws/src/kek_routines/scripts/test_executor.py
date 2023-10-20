#!/usr/bin/env python3

# ROS-related imports
import rospy

from kek_routines.motion_generator import MotionGenerator
from kek_routines.JCT_executor import JointTrajectoryControllerExecutor
from kek_routines.moveit_executor import MoveitExecutor

from numpy import pi


class GeneratorAndExecutor:
    def __init__(self):
        ns = None
        robot_urdf = "ur5e"
        tcp_link = "pestle_tip"
        ft_topic = "wrench"

        self.JCT_exec = JointTrajectoryControllerExecutor(
            ns,
            robot_urdf,
            tcp_link,
            ft_topic,
        )
        self.moveit_exec = MoveitExecutor("manipulator", "pestle_tip")

    def test_motion_generator_and_execute_by_JTC(self):
        motion_generator = MotionGenerator(
            mortar_base_position={"x": 0.39, "y": 0.39, "z": 0},
            moratr_hight=0.1,
            mortar_inner_scale={"x": 0.04, "y": 0.04, "z": 0.036},
        )

        waypoints = motion_generator.create_circular_waypoints(
            begining_position=[-5, 0],
            end_position=[-5, 0.001],
            begining_radious_z=36,
            end_radious_z=36,
            angle_param=1,
            yaw_angle=pi,
            number_of_rotations=10,
            number_of_waypoints_per_circle=50,
            center_position=[0, 0],
        )
        self.JCT_exec.execute_to_goal_pose(
            waypoints[0], ee_link="pestle_tip", time_to_reach=1, wait=True
        )
        self.JCT_exec.execute_by_waypoints(
            waypoints, total_joint_limit=1.5, ee_link="pestle_tip", time_to_reach=10
        )

    def test_motion_generator_and_execute_by_MoveIt(self):
        motion_generator = MotionGenerator(
            mortar_base_position={"x": 0.39, "y": 0.39, "z": 0},
            moratr_hight=0.1,
            mortar_inner_scale={"x": 0.04, "y": 0.04, "z": 0.036},
        )

        waypoints = motion_generator.create_circular_waypoints(
            begining_position=[-5, 0],
            end_position=[-5, 0.001],
            begining_radious_z=36,
            end_radious_z=36,
            angle_param=1,
            yaw_angle=pi,
            number_of_rotations=10,
            number_of_waypoints_per_circle=50,
            center_position=[0, 0],
        )
        self.moveit_exec.execute_cartesian_path_to_goal_pose(
            waypoints[0], ee_link="pestle_tip"
        )
        self.moveit_exec.execute_cartesian_path_by_waypoints(
            waypoints, ee_link="pestle_tip"
        )

    def test_executor_of_JTC_and_MoveIt(self):
        rospy.logwarn(
            "Test executor for JTC and MoveIt. The robot arm will start operating after pressing 'Enter'. Please ensure the robot's movement space is clear."
        )
        input("Press Enter to continue...")

        p1 = [0.39968, 0.36337, 0.09222, -0.00596, 0.99998, -0.00207, 0.00299]
        p2 = [0.41639, 0.21623, 0.44164, 0.10776, 0.91744, 0.35579, 0.1418]
        joint_p1 = [0.4915, -1.2838, 1.9699, -2.2587, 4.7054, 2.0538]
        joint_p2 = [0.2855, -1.7882, 1.6626, -1.7512, -1.7593, 4.2289]

        self.JCT_exec.execute_to_joint_goal(joint_p1)
        self.JCT_exec.execute_to_goal_pose(
            p2, ee_link="pestle_tip", time_to_reach=2, wait=True
        )
        waypoints = [p1, p2, p1]
        self.JCT_exec.execute_by_waypoints(
            waypoints, total_joint_limit=3, ee_link="pestle_tip", time_to_reach=3
        )

        # test moveit_executor
        self.moveit_exec.execute_to_joint_goal(joint_p2)
        self.moveit_exec.execute_cartesian_path_to_goal_pose(p1)
        self.moveit_exec.execute_to_goal_pose(p2)

        rospy.loginfo("Test finished.")


if __name__ == "__main__":
    rospy.init_node("JointTrajectoryControllerExecutor", anonymous=True)
    robot = GeneratorAndExecutor()
    # robot.test_motion_generator_and_execute_by_JTC()
    robot.test_motion_generator_and_execute_by_MoveIt()
    # robot.test_executor_of_JTC_and_MoveIt()
