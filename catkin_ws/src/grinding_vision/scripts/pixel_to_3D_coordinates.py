#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import message_filters


class Realsense3DCoordinates:
    def __init__(self):
        rospy.init_node("realsense_3d_coordinates", anonymous=True)

        # Set up ROS subscribers
        self.image_sub = message_filters.Subscriber("/camera/color/image_raw", Image)
        self.depth_sub = message_filters.Subscriber(
            "/camera/depth/image_rect_raw", Image
        )
        self.camera_info_sub = rospy.Subscriber(
            "/camera/color/camera_info", CameraInfo, self.camera_info_callback
        )

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Initialize camera intrinsics
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

    def camera_info_callback(self, msg):
        # Extract camera intrinsics from CameraInfo message
        self.fx = msg.K[0]  # focal length in x direction
        self.fy = msg.K[4]  # focal length in y direction
        self.cx = msg.K[2]  # principal point in x direction
        self.cy = msg.K[5]  # principal point in y direction

    def image_depth_callback(self, color_msg, depth_msg):
        if self.fx is None or self.fy is None or self.cx is None or self.cy is None:
            rospy.logwarn("Camera intrinsics not yet received. Skipping frame.")
            return

        try:
            # Convert ROS messages to OpenCV images
            color_image = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")

            # Draw a red circle at the target pixel
            target_pixel = (296, 240)  # center of mortar
            cv2.circle(color_image, target_pixel, 5, (0, 0, 255), -1)

            # Display the image with the drawn circle
            cv2.imshow("Color Image with Circle", color_image)
            cv2.waitKey(1)

            # Convert pixel coordinates to distance at the specified pixel
            depth_value = depth_image[int(target_pixel[1]), int(target_pixel[0])]

            # Process distance as needed
            if depth_value > 0:
                # Get 3D coordinates of target pixel
                # Step1: pixel to normalized coordinates
                x_normalized = (target_pixel[0] - self.cx) / self.fx
                y_normalized = (target_pixel[1] - self.cy) / self.fy

                # Step2: normalized coordinates to 3D coordinates
                x = x_normalized * depth_value
                y = y_normalized * depth_value
                z = depth_value

                # Step3: mm to m
                x = x / 1000
                y = y / 1000
                z = z / 1000

                print(f"3D coordinates at pixel {target_pixel}: ({x}, {y}, {z}) meters")
            else:
                print("No depth value at target pixel")

        except Exception as e:
            rospy.logerr(f"Error processing images: {e}")

    def run(self):
        try:
            # Synchronize the image and depth topics
            ts = message_filters.ApproximateTimeSynchronizer(
                [self.image_sub, self.depth_sub], queue_size=10, slop=0.1
            )
            ts.registerCallback(self.image_depth_callback)

            rospy.spin()

        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        realsense_3d_coordinates = Realsense3DCoordinates()
        realsense_3d_coordinates.run()
    except rospy.ROSInterruptException:
        pass
