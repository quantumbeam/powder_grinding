#!/usr/bin/env python3

import rospy
import tf.transformations as tf
from scipy.spatial.transform import Rotation
from grinding_motion_routines.moveit_executor import MoveitExecutor
from grinding_motion_routines.JTC_executor import JointTrajectoryControllerExecutor
from grinding_motion_routines.tf_publisher import TFPublisher

import numpy as np
from numpy import pi
from copy import deepcopy


class MotionPrimitive:
    def __init__(
        self,
        init_pose,
        ee_link,
        ns=None,
        move_group_name="manipulator",
        robot_urdf="ur5e",
        planner_id="RRTConnectkConfigDefault",
        planning_time =20,
        ik_solver='trac_ik'
    ):
        self.moveit_executor = MoveitExecutor(move_group_name, ee_link,planner_id,planning_time )
        self.JTC_executor = JointTrajectoryControllerExecutor(
            ns=ns, robot_urdf=robot_urdf, tcp_link=ee_link,ik_solver=ik_solver
        )

        debug_tf_pub = TFPublisher()

        #  init pose
        init_pos=init_pose[0:3]
        grinding_init_euler=init_pose[3:6]
        yaw_bias=rospy.get_param("~grinding_yaw_bias",None)
        grinding_init_euler[2] += yaw_bias
        r = Rotation.from_euler("xyz", grinding_init_euler, degrees=False)
        quat = r.as_quat()
        grinding_init_pose = init_pos + list(quat)
        self.grinding_init_pose = grinding_init_pose
        # debug_tf_pub.broadcast_tf_with_pose(grinding_init_pose, "base_link", "g_init_pose")
        gathering_init_euler=init_pose[3:6]
        yaw_bias=rospy.get_param("~gathering_yaw_bias",None)
        gathering_init_euler[2] += yaw_bias
        r = Rotation.from_euler("xyz", gathering_init_euler, degrees=False)
        quat = r.as_quat()
        gathering_init_pose = init_pos + list(quat)
        self.gathering_init_pose = gathering_init_pose
        # debug_tf_pub.broadcast_tf_with_pose(gathering_init_pose, "base_link", "G_init_pose")

        self.grinding_ee_link = rospy.get_param("~grinding_eef_link", "pestle_tip")
        self.gathering_ee_link = rospy.get_param("~gathering_eef_link", "spatula_tip")

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
        waypoints,
        total_joint_limit,
        trial_number,
        grinding_sec,
        ee_link="pestle_tip",
        moving_velocity_scale=0.1,
        moving_acceleration_scale=0.1,
        pre_motion=True,
        post_motion=True,
    ):
        if pre_motion:
            self.JTC_executor.execute_to_goal_pose(
                self.grinding_init_pose,
                ee_link=ee_link,
                time_to_reach=5,
            )
            # result=self.moveit_executor.execute_to_goal_pose(
            #     self.grinding_init_pose,
            #     ee_link=ee_link,
            #     vel_scale=moving_velocity_scale,
            #     acc_scale=moving_acceleration_scale,
            #     execute=True,
            # )
            # if result == False:
            #     rospy.logerr("Failed to move to Grinding init pose")
            #     return False
            
        joint_trajectory=self.JTC_executor.generate_joint_trajectory(
                        waypoints,
                        total_joint_limit=total_joint_limit,
                        ee_link=ee_link,
                        trial_number=trial_number,
                    )
        if joint_trajectory == None:
            rospy.logerr("No joint trajectory is generated")
            return False
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
           
            
        return True

    def execute_gathering(
        self,
        waypoints,
        total_joint_limit,
        trial_number,
        gathering_sec,
        ee_link="spatula_tip",
        grinding_velocity_scale=0.1,
        grinding_acceleration_scale=0.1,
        moving_velocity_scale=0.3,
        moving_acceleration_scale=0.3,
    ):
        self.JTC_executor.execute_to_goal_pose(
                self.gathering_init_pose,
                ee_link=ee_link,
                time_to_reach=5,
            )
        # self.moveit_executor.execute_to_goal_pose(
        #     self.gathering_init_pose,
        #     ee_link=self.grinding_ee_link,
        #     vel_scale=moving_velocity_scale,
        #     acc_scale=moving_acceleration_scale,
        #     execute=True,
        # )
        # result=self.moveit_executor.execute_to_goal_pose(
        #     self.gathering_init_pose,
        #     ee_link=ee_link,
        #     vel_scale=moving_velocity_scale,
        #     acc_scale=moving_acceleration_scale,
        #     execute=True,
        # )
        # if result == False:
        #         rospy.logerr("Failed to move to Gathering init pose")
        #         return False

        joint_trajectory=self.JTC_executor.generate_joint_trajectory(
                        waypoints,
                        total_joint_limit=total_joint_limit,
                        ee_link=ee_link,
                        trial_number=trial_number,
                    )
        if joint_trajectory == None:
            rospy.logerr("No joint trajectory is generated")
            return False
        self.JTC_executor.execute_to_joint_goal(
            joint_trajectory[0],
            time_to_reach=2,
            wait=True,
        )
        self.JTC_executor.execute_by_joint_trajectory(
            joint_trajectory,
            time_to_reach=gathering_sec,
        )

        exit_mortar_pose = waypoints[0]
        exit_mortar_pose[2] += 0.05
        self.JTC_executor.execute_to_goal_pose(
                exit_mortar_pose,
                ee_link=ee_link,
                time_to_reach=3,
        )
       
      
        return True

    def execute_scooping(
        self,
        waypoints,
        ee_link="spoon_tip",
        scooping_velocity_scale=0.2,
        scooping_acceleration_scale=0.2,
        moving_velocity_scale=0.3,
        moving_acceleration_scale=0.3,
    ):
        self.moveit_executor.execute_to_goal_pose(
            self.grinding_init_pose,
            ee_link="pestle_tip",
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )
        scooping_ready_pose = self.moveit_executor.move_group.get_current_pose(
            "pestle_tip"
        )
        scooping_ready_pose = self._pose_stamped_to_list(scooping_ready_pose)
        r = Rotation.from_quat(scooping_ready_pose[3:])
        r = r * Rotation.from_euler(
            "xyz", [0, 0, pi + pi / 16]
        )  # pi rotation has half probability to be clockwise or counterclockwise, so we add pi/16 to make it more likely to be counterclockwise
        scooping_ready_pose[3:] = r.as_quat()
        self.moveit_executor.execute_to_goal_pose(
            scooping_ready_pose,
            ee_link="pestle_tip",
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
        self.moveit_executor.execute_to_goal_pose(
            scooping_ready_pose,
            ee_link="pestle_tip",
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )
        self.moveit_executor.execute_to_goal_pose(
            self.grinding_init_pose,
            ee_link="pestle_tip",
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
        pouring_velocity_scale=0.5,
        pouring_acceleration_scale=0.5,
        moving_velocity_scale=0.2,
        moving_acceleration_scale=0.2,
        number_of_motion_steps=10,
    ):
        self.moveit_executor.execute_to_goal_pose(
            self.grinding_init_pose,
            ee_link="pestle_tip",
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
            vel_scale=pouring_velocity_scale,
            acc_scale=pouring_acceleration_scale,
        )

        # reverse pouring
        self.moveit_executor.execute_cartesian_path_by_waypoints(
            reversed(pose_list),
            ee_link,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
        )

        self.moveit_executor.execute_to_goal_pose(
            self.grinding_init_pose,
            ee_link="pestle_tip",
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )

    def execute_powder_pouring_to_MasterSizer(
        self,
        target_position,
        relay_position=None,
        delta_euler=np.array([0, 0, pi / 3 * 2]),
        ee_link="spoon_tip",
        pouring_velocity_scale=0.5,
        pouring_acceleration_scale=0.5,
        moving_velocity_scale=0.2,
        moving_acceleration_scale=0.2,
        number_of_motion_steps=10,
    ):
        # move to initial pose
        self.moveit_executor.execute_to_goal_pose(
            self.grinding_init_pose,
            ee_link="pestle_tip",
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )

        r = Rotation.from_euler("xyz", [pi, pi / 2, pi / 2], degrees=False)
        quat = r.as_quat()

        # move to relay position
        if relay_position is not None:
            relay_pose = list(relay_position) + list(quat)
            self.moveit_executor.execute_to_goal_pose(
                relay_pose,
                ee_link=ee_link,
                vel_scale=moving_velocity_scale,
                acc_scale=moving_acceleration_scale,
                execute=True,
            )

        # move to pouring ready pose
        pouring_ready_pose = self._pose_stamped_to_list(
            self.moveit_executor.move_group.get_current_pose(ee_link)
        )
        pouring_ready_pose[0:3] = target_position
        self.moveit_executor.execute_to_goal_pose(
            pouring_ready_pose,
            ee_link=ee_link,
        )

        # execute pouring
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
        self.moveit_executor.execute_cartesian_path_by_waypoints(
            pose_list,
            ee_link,
            vel_scale=pouring_velocity_scale,
            acc_scale=pouring_acceleration_scale,
        )

        # reverse pouring
        self.moveit_executor.execute_cartesian_path_by_waypoints(
            reversed(pose_list),
            ee_link,
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
        )

        self.moveit_executor.execute_to_goal_pose(
            self.grinding_init_pose,
            ee_link="pestle_tip",
            vel_scale=moving_velocity_scale,
            acc_scale=moving_acceleration_scale,
            execute=True,
        )
