#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import PoseStamped


class PoseTransformer:
    def __init__(self):
        rospy.init_node("pose_transformer")

        # tfリスナーを初期化
        self.listener = tf.TransformListener()

        # 姿勢データの購読
        self.pose_sub = rospy.Subscriber(
            "/mortar_pose_from_camera_frame", PoseStamped, self.pose_callback
        )

        # 変換された姿勢の公開
        self.pose_pub = rospy.Publisher(
            "/mortar_pose_in_base_link", PoseStamped, queue_size=10
        )

    def pose_callback(self, msg):
        try:
            # calibrated_color_optical_frameからbase_linkへの変換を待つ
            self.listener.waitForTransform(
                "/base_link", msg.header.frame_id, rospy.Time(0), rospy.Duration(4.0)
            )

            # 姿勢の変換
            transformed_pose = self.listener.transformPose("/base_link", msg)

            # 変換された姿勢を公開
            self.pose_pub.publish(transformed_pose)
            print(transformed_pose)

        except (
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as e:
            rospy.logerr(e)


if __name__ == "__main__":
    try:
        transformer = PoseTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
