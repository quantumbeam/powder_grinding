#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import pyrealsense2 as rs


class Realsense3DCoordinates:
    def __init__(self):
        rospy.init_node("realsense_3d_coordinates", anonymous=True)

        # Configure depth and color streams
        pipeline = rs.pipeline()
        self.pipeline = pipeline
        config = rs.config()
        config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        config.enable_stream(rs.stream.infrared, 2, 640, 480, rs.format.y8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        self.pipeline.start(config)

    def get_frames(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            return None, None

        self.depth_frame = depth_frame
        self.color_frame = color_frame

        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())

    def convert_pixel_to_distance(self, depth_frame, target_pixel):
        # Convert pixel coordinates to distance
        x, y = target_pixel
        distance = depth_frame.get_distance(x, y)
        return distance

    def run(self, target_pixel):
        try:
            while not rospy.is_shutdown():
                self.get_frames()
                color_frame = self.color_frame
                depth_frame = self.depth_frame
                depth_image = self.depth_image
                color_image = self.color_image

                if depth_frame is not None and color_image is not None:
                    # Perform image processing or use the images as needed
                    # For example, you can display the color image
                    # cv2.imshow("Color Image", color_image)

                    # Draw a red circle at the target pixel
                    cv2.circle(color_image, target_pixel, 5, (0, 0, 255), -1)

                    # Display the image with the drawn circle
                    cv2.imshow("Color Image with Circle", color_image)
                    cv2.waitKey(1)

                    depth_colormap = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET
                    )
                    # cv2.imshow("Depth Image", depth_colormap)

                    # Convert pixel coordinates to distance at the specified pixel
                    distance = self.convert_pixel_to_distance(depth_frame, target_pixel)

                    # Process distance as needed
                    if distance > 0:
                        # print(f"Distance at pixel {target_pixel}: {distance} meters")

                        # Get 3D coordinates of target pixel
                        # Step1: pixel to normalized coordinates
                        # u = x pixel, v = y pixel
                        # x_normalized = (u - cx) / fx
                        # y_normalized = (v - cy) / fy
                        # Step2: normalized coordinates to 3D coordinates
                        # depth_value = depth_frame.get_distance(u, v)
                        # x = x_normalized * depth_value
                        # y = y_normalized * depth_value
                        # z = depth_value
                        depth_intrin = (
                            depth_frame.profile.as_video_stream_profile().intrinsics
                        )
                        depth_pixel = [target_pixel[0], target_pixel[1]]
                        depth_point = rs.rs2_deproject_pixel_to_point(
                            depth_intrin, depth_pixel, distance
                        )
                        print(f"3D coordinates at pixel {target_pixel}: {depth_point}")

        finally:
            # Stop streaming
            self.pipeline.stop()


if __name__ == "__main__":
    target_pixel = (296, 240)  # center of mortar
    # target_pixel = (470, 240)  # edge of mortar
    try:
        realsense_3d_coordinates = Realsense3DCoordinates()
        realsense_3d_coordinates.run(target_pixel)
    except rospy.ROSInterruptException:
        pass
