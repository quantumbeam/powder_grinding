#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
import time


class RealsenseCoordinateConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_info = None

        # CameraInfoのSubscriberをここに追加
        self.camera_info_sub = rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo, self.camera_info_callback
        )

    def camera_info_callback(self, msg):
        self.camera_info = msg

    def pixel_to_3d(self, x, y, depth):
        if self.camera_info is None:
            rospy.logwarn("Camera info not available.")
            return

        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]

        pz = depth
        px = (x - cx) * depth / fx
        py = (y - cy) * depth / fy

        point = (px, py, pz)

        if point is None:
            print("Faild to calculate 3D point")

        return point


if __name__ == "__main__":
    rospy.init_node("realsense_coordinate_converter")

    x = 642
    y = 386
    depth = 0.086

    realsense_converter = RealsenseCoordinateConverter()
    time.sleep(1)
    realsense_converter.pixel_to_3d(x, y, depth)

    rospy.spin()
