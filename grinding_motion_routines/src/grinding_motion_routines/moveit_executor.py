#!/usr/bin/env python3

from pickle import FALSE
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import tf.transformations as tf

import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial.transform import Slerp


class MoveitExecutor(object):
    """Executor including MoveItCommander . This class can command moving manipurator with single pose or way points."""

    def __init__(self, move_group_name, ee_link,planner_id="RRTConnectkConfigDefault",planning_time =20):
        # initialize
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        # set moveit group name
        move_group = moveit_commander.MoveGroupCommander(move_group_name)

        # set moveit velocity and accelalation
        move_group.set_max_acceleration_scaling_factor(1.0)
        move_group.set_max_velocity_scaling_factor(1.0)

        # create display trajectry
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )


        # set planner
        move_group.set_planner_id(planner_id)
        move_group.set_planning_time (planning_time)
        # set end effector link
        move_group.set_end_effector_link(ee_link)

        # rospy.loginfo robot parameters
        rospy.loginfo("============ Planning frame: %s" % move_group.get_planning_frame())
        rospy.loginfo("============ Planner ID: %s" % move_group.get_planner_id())
        rospy.loginfo("============ Planning time: %s" % move_group.get_planning_time())
        rospy.loginfo("============ End effector link: %s" % move_group.get_end_effector_link())
        rospy.loginfo("============ End effector pose: %s" % move_group.get_current_pose())
        rospy.loginfo(
            "============ Available Planning Groups: %s" % robot.get_group_names()
        )
        
        # Robot state
        rospy.loginfo("============ Rospy.loginfoing robot state")
        rospy.loginfo(robot.get_current_state())

        # Misc variables
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = ee_link
        self.init_ee_link = ee_link
        self.group_names = move_group_name

        self.eef_step = 0.1  # the step of the cartesian path planning
        self.avoid_collisions = False
        self.path_constraints = None

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

    def _list_to_pose(self, pose_list):
        if len(pose_list) != 7:
            raise ValueError(
                "The input list must contain exactly 7 elements [x, y, z, ax, ay, az, aw]"
            )

        pose_msg = Pose()
        pose_msg.position.x = pose_list[0]
        pose_msg.position.y = pose_list[1]
        pose_msg.position.z = pose_list[2]
        pose_msg.orientation.x = pose_list[3]
        pose_msg.orientation.y = pose_list[4]
        pose_msg.orientation.z = pose_list[5]
        pose_msg.orientation.w = pose_list[6]

        return pose_msg
    def change_planner_id(self, planner_id):
        self.move_group.set_planner_id(planner_id)
        rospy.loginfo("============ Planner ID: %s" % planner_id)

    def execute_to_goal_pose(
        self,
        goal_pose,
        ee_link="",
        vel_scale=0.1,
        acc_scale=0.1,
        execute=True,
    ):
        """Supported pose is only [x y z ax ay az aw]"""

        if ee_link == "":
            ee_link = self.move_group.get_end_effector_link()

        if self.move_group.get_end_effector_link() != ee_link:
            self._change_end_effector_link(ee_link)

        self.move_group.set_max_velocity_scaling_factor(vel_scale)
        self.move_group.set_max_acceleration_scaling_factor(acc_scale)
        self.move_group.set_pose_target(self._list_to_pose(goal_pose))

        if execute:
            result = self.move_group.go(wait=True)

            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.move_group.clear_pose_targets()
            return result

        else:
            self.move_group.plan()
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return False

    def execute_cartesian_path_to_goal_pose(
        self,
        goal_pose,
        ee_link="",
        vel_scale=0.1,
        acc_scale=0.1,
        execute=True,
        eef_step=0.01,
        jump_threshold=0.0,
        avoid_collisions=True,
        path_constraints=None,
        number_of_waypoints=10,
    ):
        """Supported pose is only [x y z ax ay az aw]"""

        start_pose = self._pose_stamped_to_list(self.move_group.get_current_pose())

        positions = np.linspace(
            start_pose[0:3], goal_pose[0:3], number_of_waypoints, endpoint=False
        )

        rotations = Rotation.from_quat([start_pose[3:], goal_pose[3:]])
        quatanions = []
        slerp = Slerp([0, 1], rotations)
        for i in range(number_of_waypoints):
            ratio = float(i) / float(number_of_waypoints)
            quatanions.append(slerp(ratio).as_quat())
        quatanions = np.array(quatanions)

        waypoints = np.concatenate([positions, quatanions], axis=1).tolist()

        result = self.execute_cartesian_path_by_waypoints(
            waypoints=waypoints,
            ee_link=ee_link,
            vel_scale=vel_scale,
            acc_scale=acc_scale,
            execute=execute,
            eef_step=eef_step,
            jump_threshold=jump_threshold,
            avoid_collisions=avoid_collisions,
            path_constraints=path_constraints,
        )
        return result

    def execute_cartesian_path_by_waypoints(
        self,
        waypoints,
        ee_link="",
        vel_scale=0.1,
        acc_scale=0.1,
        execute=True,
        eef_step=0.01,
        jump_threshold=0.0,
        avoid_collisions=True,
        path_constraints=None,
    ):
        """Supported pose is list of [x y z ax ay az aw]"""

        if ee_link == "":
            ee_link = self.move_group.get_end_effector_link()
        if self.move_group.get_end_effector_link() != ee_link:
            self._change_end_effector_link(ee_link)

        # compute cartesian path with lerp of waypoints
        waypoints = [self._list_to_pose(pose) for pose in waypoints]
        (path, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            eef_step=eef_step,
            jump_threshold=jump_threshold,
            avoid_collisions=avoid_collisions,
            path_constraints=path_constraints,
        )
        if fraction < 1.0:
            rospy.loginfo(
                "compute cartision path faild. complete planning of %s"
                % str(fraction * 100)
                + "%"
            )
            return False

        plan = self.move_group.retime_trajectory(
            self.robot.get_current_state(),
            path,
            vel_scale,
            acc_scale,
            algorithm="time_optimal_trajectory_generation",
        )
        # default algorithm is iterative_time_parameterization

        if execute:
            success = self.move_group.execute(plan, wait=True)
            return success
        else:
            return plan

    def execute_to_joint_goal(
        self, joint_goal, vel_scale=0.1, acc_scale=0.1, execute=True
    ):
        self.move_group.set_joint_value_target(joint_goal)
        if execute:
            self.move_group.set_max_velocity_scaling_factor(vel_scale)
            self.move_group.set_max_acceleration_scaling_factor(acc_scale)
            self.move_group.go(wait=True)
            self.move_group.stop()
        self.move_group.clear_pose_targets()

    def _change_end_effector_link(self, new_ee_link):
        old_eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo(
            "============ Cange End effector link: %s to %s"
            % (old_eef_link, new_ee_link)
        )
        self.move_group.set_end_effector_link(new_ee_link)
