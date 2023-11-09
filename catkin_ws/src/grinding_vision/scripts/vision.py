#!/usr/bin/env python3

import cv2
import numpy as np
from numpy.core.numeric import count_nonzero
from numpy.lib.function_base import _parse_input_dimensions
import pandas as pd
import sys
import matplotlib.pyplot as plt

from cv_bridge import CvBridge
import rospy
import sensor_msgs.msg

import mortar_image_correction
import catkin_ws.src.grinding_vision.scripts.powder_detector as powder_detector
import copy
import datetime


from grinding_vision.srv import PowderPos, PowderPosResponse

# study_name = "TiO2_line_the_bottom_with_lubber"
# fmt = cv2.VideoWriter_fourcc("m", "p", "4", "v")
# save_path = (
#     "C:/Users/YusakuNakajima/Desktop/211110_cobotta_motion_experiments/"
#     + study_name
#     + ".mp4"
# )

# args = sys.argv
# if "debug" in args:
#     print(cv2.getBuildInformation())
#     exit()

# if "camera" in args:
#     print("use realsense")
# elif "video" in args:
#     cap = cv2.VideoCapture(
#         "C:/Users/YusakuNakajima/Desktop/211110_cobotta_motion_experiments/TiO2_spiral.mp4"
#     )
#     using_camera = False
# else:
#     print("Prease write 'camera' or 'video' in args")
#     exit()


class GrindingVision:
    # This class advertises the vision actions that we call during execution.
    def __init__(self):
        # Setup camera image subscribers
        camera_name = "camera"
        self.image_sub = rospy.Subscriber(
            "/" + camera_name + "/color/image_raw",
            sensor_msgs.msg.Image,
            self.image_subscriber_callback,
        )
        self.img_pub = rospy.Publisher(
            "grinding_vision_pub", sensor_msgs.msg.Image, queue_size=10
        )
        self.bridge = CvBridge()

        # Setup powder CV server
        self.srv = rospy.Service("/powder_pos", PowderPos, self.image_server)

        # Setup params
        self.vertex_points = [
            [688, 154],
            [560, 292],
            [562, 472],
            [698, 600],
            [878, 599],
            [1011, 466],
            [1014, 274],
            [874, 148],
        ]

        self.mortar_inner_diameter = 80
        self.mortar_outer_max_diameter = 109
        self.img_rotate_angle = 360 - 45
        self.mask_diameter_margin = -5

        self.save_dir = "/root/ocla/catkin_ws/src/grinding_vision/"
        self.debug_data_columns = []
        self.debug_data_list = []

        rospy.loginfo(" ===== Grinding vision node has started up!")

    def __del__(self):
        # end processing
        cv2.destroyAllWindows()

    def image_server(self, param):
        frame = self.bridge.imgmsg_to_cv2(self._last_rgb_image, "bgr8")
        return self.powder_cv(frame, param.command, param.grinding_radius_max)

    # ===== Subscriber callbacks
    def image_subscriber_callback(self, rgb_image_msg):
        self._last_rgb_image = rgb_image_msg

    def depth_image_sub_callback(self, depth_image_msg):
        self._last_depth_image_ros = depth_image_msg

    def camera_info_callback(self, cam_info_message):
        self._camera_info = cam_info_message

    def powder_cv(self, frame, command, grinding_radius_max):
        bridge = self.bridge
        h, w = frame.shape[0], frame.shape[1]

        img_correction = mortar_image_correction.MotarImageCorrection(
            vertex_length_mm=self.mortar_outer_max_diameter
        )

        if command == "write_debug_data":
            df = pd.DataFrame(self.debug_data_list, columns=self.debug_data_columns)
            df.to_csv(
                self.save_dir + "%s_debug_data.csv" % datetime.datetime.now(),
                header=True,
                index=False,
            )

        # set mortar vertex points
        if command == "calibration":
            self.vertex_points = img_correction.set_vertex_points(frame, 8)
        points = self.vertex_points

        # calc mortar center and pixcel length
        mortar_center, mm_per_pixcel = img_correction.calc_center_and_pixcel_length(
            points
        )

        # rotate image
        rotation_matrix = cv2.getRotationMatrix2D(
            mortar_center, self.img_rotate_angle, scale=1.0
        )
        frame = cv2.warpAffine(frame, rotation_matrix, (w, h))
        origin_frame = copy.deepcopy(frame)

        # mortar area mask
        radius = (self.mortar_inner_diameter + self.mask_diameter_margin) / 2
        mortar_radius_pixcel = int(radius / mm_per_pixcel)
        mortar_area_mask = np.zeros(origin_frame.shape, dtype=np.uint8)
        cv2.circle(
            mortar_area_mask,
            (mortar_center[0], mortar_center[1]),
            mortar_radius_pixcel,
            (255, 255, 255),
            -1,
        )
        trimed_mortar_area_frame = origin_frame & mortar_area_mask

        # grinding area mask
        origin_frame = copy.deepcopy(frame)
        grinding_area_mask = np.zeros(origin_frame.shape, dtype=np.uint8)
        cv2.circle(
            grinding_area_mask,
            (mortar_center[0], mortar_center[1]),
            int(grinding_radius_max / mm_per_pixcel),
            (255, 255, 255),
            -1,
        )
        trimed_grinding_area_frame = origin_frame & grinding_area_mask

        # powder detection
        powder_cv = powder_detector.PowderDetector(
            mm_per_pixcel, mortar_center, mortar_radius_pixcel
        )
        powder_cv.convert_threshold_image(
            trimed_mortar_area_frame, method=cv2.THRESH_BINARY
        )
        powder_cv.detect_simple_contour()
        powder_cv.detect_watershed()

        # calc contour distance
        powder_cv.calc_contour_distance_and_center_of_gravity(
            powder_cv.watershed_contour, powder_cv.watershed_contour_frame
        )

        # calc center of gravity for threshold image
        powder_cv.calc_threshold_distance_image()

        # imshow
        cv_img = powder_cv.imshow(frame, scale=0.5, show=False)
        self.img_pub.publish(bridge.cv2_to_imgmsg(cv_img))

        # save img
        # cv2.imwrite(
        #     self.save_dir + "%s_origin_img.jpg" % datetime.datetime.now(),
        #     origin_frame,
        # )
        # cv2.imwrite(
        #     self.save_dir + "%s_trimed_img.jpg" % datetime.datetime.now(),
        #     trimed_mortar_area_frame,
        # )

        cv2.imwrite(
            self.save_dir + "%s_cv_img.jpg" % datetime.datetime.now(),
            cv_img,
        )

        return PowderPosResponse(
            powder_cv.threshold_MIC_center[0],
            powder_cv.threshold_MIC_center[1],
            powder_cv.threshold_MIC_radius,
        )


if __name__ == "__main__":
    rospy.init_node("grinding_vision", anonymous=False)
    c = GrindingVision()
    rospy.spin()
