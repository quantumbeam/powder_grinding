#!/usr/bin/env python3

# ROS-related imports
import rospy
from geometry_msgs.msg import Point, TransformStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from grinding_vision.srv import MortarCenter, MortarCenterResponse
from grinding_vision import realsense_coordinate_converter
import tf2_ros
import tf2_geometry_msgs  # required to transform PointStamped
from geometry_msgs.msg import PointStamped

import cv2
import numpy as np


class MortarPositionEstimater:
    def __init__(self):
        # Convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

        self.coordinate_converter = (
            realsense_coordinate_converter.RealsenseCoordinateConverter()
        )

        # Get the image topic to subscribe to
        self.image_topic = rospy.get_param("~image_topic", "/camera/color/image_raw")
        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)

        # Publisher for the composite image visualizing the detection
        self.comp_pub = rospy.Publisher(
            "/calibrate_mortar_position", Image, queue_size=10
        )

        # Parameters for image processing
        self.closing_kernel_size = rospy.get_param(
            "~closing_kernel_size", 5
        )  # Kernel size for morphological 'closing' operation
        self.binary_threshold = rospy.get_param(
            "~binary_threshold", 127
        )  # Threshold for binary thresholding
        self.distance_to_mortar_from_camera = rospy.get_param(
            "~distance_to_mortar_from_camera", 0.15
        )  # Distance from the camera to the mortar

        self.br = tf2_ros.TransformBroadcaster()

    def image_callback(self, data):
        """
        Callback function for the image subscriber. Processes the image to detect an octagon and its center.
        """
        try:
            color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Image processing steps
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        kernel = np.ones((self.closing_kernel_size, self.closing_kernel_size), np.uint8)
        closing = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)

        # Find and process contours
        contours, _ = cv2.findContours(
            closing.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        contour_image = color.copy()
        self.detect_mortar(color, contours, contour_image)

        # Concatenate images for visualization
        self.publish_debug_image(color, binary, closing, contour_image)

        # Calculate the coordinates of the center of the octagon
        point = self.coordinate_converter.pixel_to_3d(
            self.center_pixel[0],
            self.center_pixel[1],
            self.distance_to_mortar_from_camera,
        )
        rospy.loginfo(f"Center coordinates: {point}")

        if self.transform_point(point) is not False:
            point_by_base_link = self.transformed_point
            rospy.loginfo(f"Center coordinates by base_link: {point_by_base_link}")
            self.publish_tf_frame("calibrated_camera_color_optical_frame", point)

    def detect_mortar(self, color, contours, contour_image):
        """
        Find the largest octagon contour and draw it on the image. Set its center if found.
        """
        max_area = 0
        max_contour = None
        for cnt in contours:
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            if len(approx) == 8:
                area = cv2.contourArea(approx)
                if area > max_area:
                    max_area = area
                    max_contour = approx
            cv2.drawContours(contour_image, [approx], -1, (0, 255, 0), 2)
        if max_contour is not None:
            cv2.drawContours(color, [max_contour], -1, (0, 255, 0), 2)
            M = cv2.moments(max_contour)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            self.center_pixel = (cX, cY)

    def transform_point(self, point):
        tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # calibrated_camera_color_optical_frameに基づいた点を定義
        point_in_calibrated_camera_frame = PointStamped()
        point_in_calibrated_camera_frame.header.frame_id = (
            "calibrated_camera_color_optical_frame"
        )
        point_in_calibrated_camera_frame.header.stamp = rospy.Time.now()
        point_in_calibrated_camera_frame.point.x = point[0]
        point_in_calibrated_camera_frame.point.y = point[1]
        point_in_calibrated_camera_frame.point.z = point[2]

        rospy.sleep(2.0)  # TF情報を取得する時間を確保

        try:
            # 点をbase_linkに変換
            transformed_point = tf_buffer.transform(
                point_in_calibrated_camera_frame, "base_link", rospy.Duration(1.0)
            )
            self.transformed_point = (
                transformed_point.point.x,
                transformed_point.point.y,
                transformed_point.point.z,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            print("Transform error")
            return False

    def publish_tf_frame(self, pearent_frame, point):
        t = TransformStamped()

        # メッセージのヘッダ情報をセットします
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = pearent_frame
        t.child_frame_id = "mortar_center_frame"

        # 変換後の座標をセットします
        t.transform.translation.x = point[0]
        t.transform.translation.y = point[1]
        t.transform.translation.z = point[2]
        t.transform.rotation.x = 0  # ここでは回転は考慮していません
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        # TFをbroadcastします
        self.br.sendTransform(t)

    def publish_debug_image(self, color, binary, closing, contour_image):
        """
        Create a composite image for visualization and publish it.
        """

        # add text
        x, y = self.center_pixel
        cv2.circle(color, (x, y), 10, (255, 0, 0), -1)
        cv2.putText(
            color,
            f"Center: ({x}, {y})",
            (x - 50, y - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 0, 255),
            3,
        )
        cv2.putText(
            color,
            "Original",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            2,
            (0, 0, 255),
            3,
        )

        # Resize images to fit in the composite image
        color = cv2.resize(color, (640, 480))
        binary_resized = cv2.resize(binary, (640, 480))
        closing_resized = cv2.resize(closing, (640, 480))
        contour_image = cv2.resize(contour_image, (640, 480))
        h_concat1 = cv2.hconcat(
            [
                cv2.cvtColor(color, cv2.COLOR_BGR2RGB),
                cv2.cvtColor(binary_resized, cv2.COLOR_GRAY2BGR),
            ]
        )
        h_concat2 = cv2.hconcat(
            [cv2.cvtColor(closing_resized, cv2.COLOR_GRAY2BGR), contour_image]
        )
        comp_image = cv2.vconcat([h_concat1, h_concat2])
        try:
            self.comp_pub.publish(self.bridge.cv2_to_imgmsg(comp_image, "bgr8"))
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    rospy.init_node("mortar_detector", anonymous=True)
    mortar_detector = MortarPositionEstimater()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")
