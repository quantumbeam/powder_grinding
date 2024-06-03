# The MIT License (MIT)
#
# Copyright (c) 2018-2023 Cristian Beltran
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Cristian Beltran

import collections
import numpy as np

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from geometry_msgs.msg import WrenchStamped

from grinding_motion_routines import utils, spalg, conversions, transformations
from grinding_motion_routines.constants import (
    FT_SUBSCRIBER,
    TRAC_IK,
    DONE,
    SPEED_LIMIT_EXCEEDED,
    IK_NOT_FOUND,
    BASE_LINK,
    EE_LINK,
)

from std_srvs.srv import Empty, SetBool, Trigger

from grinding_motion_routines.controllers_connection import ControllersConnection
from grinding_motion_routines.controllers import (
    JointTrajectoryController,
    FTsensor,
    GripperController,
)
from trac_ik_python.trac_ik import IK as TRACK_IK_SOLVER

cprint = utils.TextColors()


def resolve_parameter(value, default_value):
    if value:
        return value
    if default_value:
        return default_value
    raise ValueError("No value defined for parameter")


class Arm(object):
    """arm controller"""

    def __init__(
        self,
        robot_urdf_pkg,
        robot_urdf_file_name,
        joint_trajectory_controller_name,
        ik_solver=TRAC_IK,
        solve_type="Distance",
        namespace=None,
        gripper=False,
        joint_names_prefix=None,
        ft_topic=None,
        ft_sensor=False,
        base_link=None,
        ee_link=None,
    ):
        """ee_transform array [x,y,z,ax,ay,az,w]: optional transformation to the end-effector
                                              that is applied before doing any operation in task-space
        robot_urdf string: name of the robot urdf file to be used
        namespace string: nodes namespace prefix
        gripper bool: enable gripper control
        """

        cprint.ok("ft_sensor: {}, ee_link: {}".format(bool(ft_topic), ee_link))

        self._joint_angle = dict()
        self._joint_velocity = dict()
        self._joint_effort = dict()

        self.current_ft_value = np.zeros(6)
        self.wrench_queue = collections.deque(maxlen=25)  # store history of FT data

        self.ft_topic = resolve_parameter(ft_topic, FT_SUBSCRIBER)
        self.joint_trajectory_controller = joint_trajectory_controller_name
        self.ik_solver = ik_solver

        self.ns = namespace if namespace else ""
        self._robot_urdf_package = robot_urdf_pkg
        self._robot_urdf_file_name = robot_urdf_file_name

        base_link = resolve_parameter(base_link, BASE_LINK)
        ee_link = resolve_parameter(ee_link, EE_LINK)

        # Support for joint prefixes
        self.joint_names_prefix = joint_names_prefix if joint_names_prefix else ""
        self.base_link = (
            base_link if joint_names_prefix is None else joint_names_prefix + base_link
        )
        self.ee_link = (
            ee_link if joint_names_prefix is None else joint_names_prefix + ee_link
        )

        # self.max_joint_speed = np.deg2rad([100, 100, 100, 200, 200, 200]) # deg/s -> rad/s
        self.max_joint_speed = np.deg2rad([191, 191, 191, 371, 371, 371])

        self._init_ik_solver(self.base_link, self.ee_link, solve_type)
        self._init_controllers(gripper)
        if ft_sensor:
            self._init_ft_sensor()

        self.controller_manager = ControllersConnection(namespace)

    ### private methods ###

    def _init_controllers(self, gripper):
        traj_publisher = self.joint_trajectory_controller
        self.joint_names = rospy.get_param(
            self.ns + "/" + self.joint_trajectory_controller + "/joints"
        )

        # Flexible trajectory (point by point)

        traj_publisher_flex = self.ns + "/" + traj_publisher + "/command"
        cprint.blue("connecting to: {}".format(traj_publisher_flex))
        self._flex_trajectory_pub = rospy.Publisher(
            traj_publisher_flex, JointTrajectory, queue_size=10
        )
        print("Connected to: ", traj_publisher_flex)

        self.joint_traj_controller = JointTrajectoryController(
            publisher_name=traj_publisher,
            namespace=self.ns,
            joint_names=self.joint_names,
            timeout=1.0,
        )

        self.gripper = None
        if gripper:
            self.gripper = GripperController(
                namespace=self.ns, prefix=self.joint_names_prefix, timeout=2.0
            )

    def _init_ik_solver(self, base_link, ee_link, solve_type):
        self.base_link = base_link
        self.ee_link = ee_link
        if self.ik_solver == TRAC_IK:
            try:
                if not rospy.has_param("robot_description"):
                    self.trac_ik = TRACK_IK_SOLVER(
                        base_link=base_link,
                        tip_link=ee_link,
                        solve_type=solve_type,
                        timeout=0.002,
                        epsilon=1e-5,
                        urdf_string=utils.load_urdf_string(
                            self._robot_urdf_package, self._robot_urdf_file_name
                        ),
                    )
                else:
                    self.trac_ik = TRACK_IK_SOLVER(
                        base_link=base_link, tip_link=ee_link, solve_type=solve_type
                    )
            except Exception as e:
                rospy.logerr("Could not instantiate TRAC_IK" + str(e))
        else:
            raise Exception("unsupported ik_solver", self.ik_solver)

    def _init_ft_sensor(self):
        # Publisher of wrench
        ft_namespace = "%s/%s/filtered" % (self.ns, self.ft_topic)
        rospy.Subscriber(ft_namespace, WrenchStamped, self.__ft_callback__)

        self._zero_ft_filtered = rospy.ServiceProxy(
            "%s/%s/filtered/zero_ftsensor" % (self.ns, self.ft_topic), Empty
        )
        self._zero_ft_filtered.wait_for_service(rospy.Duration(2.0))

        if not rospy.has_param("use_gazebo_sim"):
            self._zero_ft = rospy.ServiceProxy(
                "%s/ur_hardware_interface/zero_ftsensor" % self.ns, Trigger
            )
            self._zero_ft.wait_for_service(rospy.Duration(2.0))

        self._ft_filtered = rospy.ServiceProxy(
            "%s/%s/filtered/enable_filtering" % (self.ns, self.ft_topic), SetBool
        )
        self._ft_filtered.wait_for_service(rospy.Duration(1.0))

        # Check that the FT topic is publishing
        if not utils.wait_for(lambda: self.current_ft_value is not None, timeout=2.0):
            rospy.logerr("Timed out waiting for {0} topic".format(ft_namespace))
            return

    def __ft_callback__(self, msg):
        self.current_ft_value = conversions.from_wrench(msg.wrench)
        self.wrench_queue.append(self.current_ft_value)

    def _flexible_trajectory(self, position, time=5.0, vel=None):
        """Publish point by point making it more flexible for real-time control"""
        # Set up a trajectory message to publish.
        action_msg = JointTrajectory()
        action_msg.joint_names = (
            rospy.get_param(self.joint_trajectory_controller + "/joints")
            if self.joint_names is None
            else self.joint_names
        )

        # Create a point to tell the robot to move to.
        target = JointTrajectoryPoint()
        target.positions = position

        # These times determine the speed at which the robot moves:
        if vel is not None:
            target.velocities = [vel] * 6

        target.time_from_start = rospy.Duration(time)

        # Package the single point into a trajectory of points with length 1.
        action_msg.points = [target]

        self._flex_trajectory_pub.publish(action_msg)

    def _solve_ik(self, pose, q_guess=None, attempts=5, verbose=True):
        q_guess_ = q_guess if q_guess is not None else self.joint_angles()
        if isinstance(q_guess, np.float64):
            q_guess_ = None

        if self.ik_solver == TRAC_IK:
            ik = self.trac_ik.get_ik(q_guess_, *pose)
            if ik is None:
                if attempts > 0:
                    return self._solve_ik(pose, q_guess, attempts - 1)
                if verbose:
                    rospy.logwarn("TRACK-IK: solution not found!")

        return ik

    ### Public methods ###

    def get_filtered_ft(self):
        """Get measurements from FT Sensor in its default frame of reference.
        Measurements are filtered with a low-pass filter.
        Measurements are given in sensors orientation.
        """
        if self.current_ft_value is None:
            raise Exception("FT Sensor not initialized")

        ft_limitter = [
            300,
            300,
            300,
            30,
            30,
            30,
        ]  # Enforce measurement limits (simulation)
        ft = self.current_ft_value
        ft = [
            ft[i] if abs(ft[i]) < ft_limitter[i] else ft_limitter[i] for i in range(6)
        ]
        return np.array(ft)

    def get_ee_wrench_hist(self, hist_size=24):
        if self.current_ft_value is None:
            raise Exception("FT Sensor not initialized")

        q_hist = self.joint_traj_controller.get_joint_positions_hist()[:hist_size]
        ft_hist = np.array(self.wrench_queue)[:hist_size]

        poses_hist = [self.end_effector(q) for q in q_hist]
        wrench_hist = [
            spalg.convert_wrench(wft, p).tolist() for p, wft in zip(poses_hist, ft_hist)
        ]

        return np.array(wrench_hist)

    def get_ee_wrench(self, hand_frame_control=False):
        """Compute the wrench (force/torque) in task-space"""
        if self.current_ft_value is None:
            return np.zeros(6)

        wrench_force = self.current_ft_value
        if not hand_frame_control:
            return wrench_force
        else:
            # Transform force to end effector frame
            # Fix, the forces need to be converted by the transform between the wrist_3_link and the end effector link
            # transform = self.kdl.get_transform_between_links(self.joint_names_prefix + "wrist_3_link", self.ee_link)
            # transform = self.kdl.get_transform_between_links(self.joint_names_prefix + "gripper_tip_link", self.base_link)
            transform = self.end_effector(tip_link=self.joint_names_prefix + "tool0")
            ee_wrench_force = spalg.convert_wrench(wrench_force, transform)

            return ee_wrench_force

    def zero_ft_sensor(self):
        if not rospy.has_param("use_gazebo_sim"):
            # First try to zero FT from ur_driver
            self._zero_ft()
        # Then update filtered one
        self._zero_ft_filtered()

    def set_ft_filtering(self, active=True):
        self._ft_filtered(active)

    def end_effector(self, joint_angles=None, rot_type="quaternion", tip_link=None):
        """Return End Effector Pose"""

        joint_angles = self.joint_angles() if joint_angles is None else joint_angles

        if rot_type == "quaternion":
            # forward kinematics
            return self.kdl.forward(joint_angles, tip_link)

        elif rot_type == "euler":
            x = self.end_effector(joint_angles)
            euler = np.array(transformations.euler_from_quaternion(x[3:], axes="rxyz"))
            return np.concatenate((x[:3], euler))

        else:
            raise Exception("Rotation Type not supported", rot_type)

    def joint_angle(self, joint):
        """
        Return the requested joint angle.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: angle in radians of individual joint
        """
        return self.joint_traj_controller.get_joint_positions()[joint]

    def joint_angles(self):
        """
        Return all joint angles.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to angle (rad) Values
        """
        return self.joint_traj_controller.get_joint_positions()

    def joint_velocity(self, joint):
        """
        Return the requested joint velocity.

        @type joint: str
        @param joint: name of a joint
        @rtype: float
        @return: velocity in radians/s of individual joint
        """
        return self.joint_traj_controller.get_joint_velocities()[joint]

    def joint_velocities(self):
        """
        Return all joint velocities.

        @rtype: dict({str:float})
        @return: unordered dict of joint name Keys to velocity (rad/s) Values
        """
        return self.joint_traj_controller.get_joint_velocities()

    ### Basic Control Methods ###

    def set_joint_positions(
        self, position, velocities=None, accelerations=None, wait=False, t=5.0
    ):
        self.joint_traj_controller.add_point(
            positions=position,
            time=t,
            velocities=velocities,
            accelerations=accelerations,
        )
        self.joint_traj_controller.start(delay=0.01, wait=wait)
        self.joint_traj_controller.clear_points()
        return DONE

    def set_joint_trajectory(
        self, trajectory, velocities=None, accelerations=None, t=5.0
    ):
        dt = float(t) / float(len(trajectory))

        vel = None
        acc = None

        if velocities is not None:
            vel = velocities
        if accelerations is not None:
            acc = accelerations
        for i, q in enumerate(trajectory):
            self.joint_traj_controller.add_point(
                positions=q, time=(i + 1) * dt, velocities=vel, accelerations=acc
            )
        self.joint_traj_controller.start(delay=0.01, wait=True)
        self.joint_traj_controller.clear_points()

    def set_joint_positions_flex(self, position, t=5.0, v=None):
        qc = self.joint_angles()
        speed = (qc - position) / t
        cmd = position
        if np.any(np.abs(speed) > self.max_joint_speed):
            rospy.logwarn(
                "Exceeded max speed %s deg/s, ignoring command"
                % np.round(np.rad2deg(speed), 0)
            )
            return SPEED_LIMIT_EXCEEDED
        self._flexible_trajectory(cmd, t, v)
        return DONE

    def set_target_pose(self, pose, wait=False, t=5.0):
        """Supported pose is only x y z aw ax ay az"""
        q = self._solve_ik(pose)
        if q is None:
            # IK not found
            return IK_NOT_FOUND
        else:
            self.set_joint_positions(q, wait=wait, t=t)
            return q

    def set_target_pose_flex(self, pose, t=5.0):
        """Supported pose is only x y z aw ax ay az"""
        q = self._solve_ik(pose)
        if q is None:
            # IK not found
            return IK_NOT_FOUND
        else:
            return self.set_joint_positions_flex(q, t=t)

    def move_relative(
        self, transformation, relative_to_tcp=True, duration=5.0, wait=True
    ):
        """Move end-effector backwards relative to its position in a straight line"""
        new_pose = transformations.transform_pose(
            self.end_effector(), transformation, rotated_frame=relative_to_tcp
        )
        self.set_target_pose(pose=new_pose, t=duration, wait=wait)
