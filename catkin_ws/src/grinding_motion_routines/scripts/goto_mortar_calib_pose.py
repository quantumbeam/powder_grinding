#!/usr/bin/env python3

import rospy
import copy
from scipy.spatial.transform import Rotation
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose
from grinding_motion_routines import (
    motion_generator,
    moveit_executor,
    JTC_executor,
    motion_primitive,
    marker_display,
    tf_publisher,
    load_planning_scene,
)


def main():
    rospy.init_node("mechano_grinding", anonymous=True)
    debug_tf = tf_publisher.TFPublisher()
    pi = np.pi

    mortar_base_position = rospy.get_param("/loading_planning_scene/mortar_position")

    moveit = moveit_executor.MoveitExecutor("manipulator", "camera")
    r = Rotation.from_euler("xyz", [pi, 0, pi / 2], degrees=False)
    quat = r.as_quat()
    mortar_base_pose = list(mortar_base_position.values()) + list(quat)

    camera_view_pose = copy.deepcopy(mortar_base_pose)
    camera_view_pose[2] += 0.2
    debug_tf.broadcast_tf_with_pose(camera_view_pose, "base_link")

    moveit.execute_to_goal_pose(
        camera_view_pose, ee_link="camera", vel_scale=0.1, acc_scale=0.1, execute=True
    )


if __name__ == "__main__":
    main()
