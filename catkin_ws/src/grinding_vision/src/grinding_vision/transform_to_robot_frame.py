#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped


class PoseTransformer:
    def __init__(self):
        # tfリスナーを初期化
        self.listener = tf.TransformListener()

        # 変換された姿勢の公開
        self.pose_pub = rospy.Publisher(
            "/mortar_pose_in_base_link", PoseStamped, queue_size=10
        )

    def transform_pose(
        self,
        input_pose,
        robot_frame="/base_link",
        parent_frame="/calibrated_depth_optical_frame",
    ):
        try:
            # calibrated_color_optical_frameからbase_linkへの変換を待つ
            self.listener.waitForTransform(
                robot_frame,
                parent_frame,
                rospy.Time(0),
                rospy.Duration(4.0),
            )

            # 姿勢の変換
            transformed_pose = self.listener.transformPose(robot_frame, input_pose)

            # 変換された姿勢を公開
            # self.pose_pub.publish(transformed_pose)
            # rospy.loginfo("Transformed position" + str(transformed_pose.pose.position))
            return transformed_pose

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.logerr(e)


if __name__ == "__main__":
    rospy.init_node("pose_transformer")
    try:
        transformer = PoseTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
