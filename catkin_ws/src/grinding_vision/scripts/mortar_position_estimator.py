#!/usr/bin/env python3

import threading
import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from vision_msgs.msg import Detection2DArray
from pixel_to_3d_position import PixelTo3DPosition  # Realsense3DCoordinatesクラスのインポート
from transform_to_robot_frame import PoseTransformer  # PoseTransformerクラスのインポート


class MortarDetector:
    def __init__(self):
        self.parent_frame = "calibrated_depth_optical_frame"
        self.child_frame = "base_link"
        self.target_yolo_class_id = 61  # yolov8のtoiletのクラスID

        self.realsense_position_estimator = PixelTo3DPosition()
        self.pose_transformer = PoseTransformer()  # PoseTransformerのインスタンス化

        # YOLO8 Detection Subscriber
        self.detection_sub = rospy.Subscriber(
            "/yolo8_detection_result", Detection2DArray, self.detection_callback
        )

        # Marker Publisher
        self.marker_pub = rospy.Publisher(
            "/mortar_position_marker", Marker, queue_size=10
        )

    def detection_callback(self, msg):
        if not msg.detections:
            return
        center = None
        for detection in msg.detections:
            if detection.results[0].id == self.target_yolo_class_id:
                center = detection.bbox.center
                break
        if center is None:
            return

        # # ターゲットピクセルを更新
        # detector.realsense_position_estimator.target_pixel = (int(320), int(240))
        self.realsense_position_estimator.target_pixel = (int(center.x), int(center.y))

        mortar_position = self.realsense_position_estimator.position_result

        # 3D座標を使用して姿勢を作成
        pose_3d = PoseStamped()
        # pose_3d.header.stamp = rospy.Time.now()
        pose_3d.header.frame_id = self.parent_frame
        pose_3d.pose.position.x = mortar_position[0]
        pose_3d.pose.position.y = mortar_position[1]
        pose_3d.pose.position.z = mortar_position[2]
        pose_3d.pose.orientation.w = 1.0

        # PoseTransformerを使用して座標変換
        transformed_pose = self.pose_transformer.transform_pose(
            pose_3d, self.child_frame, self.parent_frame
        )

        #  publish marker
        marker = Marker()
        marker.header.frame_id = self.child_frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "mortar_position_marker"
        marker.id = 0
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose = transformed_pose.pose
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("dynamic_object_detector")
    detector = MortarDetector()
    threading.Thread(target=detector.realsense_position_estimator.run).start()

    rospy.spin()
