#!/usr/bin/env python3

import cv2
import matplotlib
import numpy as np
from numpy.core.numeric import count_nonzero
from numpy.lib.function_base import _parse_input_dimensions
import pandas as pd
import sys
import matplotlib.pyplot as plt
from statistics import mean

from cv_bridge import CvBridge
import rospy
import sensor_msgs.msg

import mortar_image_correction
import catkin_ws.src.grinding_vision.scripts.powder_detector as powder_detector
import copy
import datetime
import time


from grinding_vision.srv import DiagonalPowderRadius, DiagonalPowderRadiusResponse


class GrindingVision:
    # This class advertises the vision actions that we call during execution.
    def __init__(self, mortar_calib=False, threshold_check=False):
        self.bridge = CvBridge()

        # Setup camera image subscribers
        camera_name = "camera"
        self.image_sub = rospy.Subscriber(
            "/" + camera_name + "/color/image_raw",
            sensor_msgs.msg.Image,
            self.image_subscriber_callback,
        )
        self.debug_img_pub = rospy.Publisher(
            "grinding_vision_debug3", sensor_msgs.msg.Image, queue_size=10
        )
        self.debug_img_pub_2 = rospy.Publisher(
            "grinding_vision_debug4", sensor_msgs.msg.Image, queue_size=10
        )

        # Setup powder CV server
        self.srv = rospy.Service(
            "/diagonal_vision",
            DiagonalPowderRadius,
            self.diagonal_vision_server_callback,
        )

        # Setup params
        self.vertex_points = [
            [472, 85],
            [492, 193],
            [604, 276],
            [735, 268],
            [800, 192],
            [771, 87],
            [680, 20],
            [560, 14],
        ]

        self.mortar_inner_diameter = 80
        self.mortar_outer_max_diameter = 109
        self.img_rotate_angle = 360 - 45
        self.mask_diameter_margin = -5
        self.binary_threshold = 100

        self.save_dir = "/root/ocla/catkin_ws/src/grinding_vision/dump/"
        self.debug_data_columns = []
        self.debug_data_list = []
        self.original_frame_list = []
        self.img_list = []

        self.mortar_calib = mortar_calib
        self.threshold_check = threshold_check

        self.stack_sec = 1.0
        self.init_mortar_image_correction = True
        self.camera_fps = 30

        self.powder_radius = 0
        self.powder_radius_list = []

        # init class
        self.powder_detector = powder_detector.PowderDetector(self.camera_fps)

        rospy.loginfo(" ===== diagonal vision node has started up!")

    def __del__(self):
        cv2.destroyAllWindows()

    def diagonal_vision_server_callback(self, param):
        if param.start_stack_frame:
            self.powder_detector.start_stack_frame = self.powder_detector.binary_frame
            rospy.loginfo("set start stack frame as current binarry frame")
        if param.stack_sec >= 0:
            self.stack_sec = param.stack_sec
            rospy.loginfo("set stack sec: " + str(param.stack_sec))

        # return self.powder_detector.ellipse_radius_major_mm_fixed

        # for i, frame in enumerate(self.powder_detector.binary_stack_list):
        #     print(i)
        #     cv2.imwrite(
        #         self.save_dir + "binary%d.jpg" % i,
        #         self.powder_detector.binary_stack_list[i],
        #     )
        #     cv2.imwrite(
        #         self.save_dir + "origin%d.jpg" % i,
        #         self.original_frame_list[i],
        #     )
        # rospy.loginfo("Save video")
        # rospy.loginfo(len(self.img_list))
        # shape = self.img_list[0].shape
        # now = datetime.datetime.now()
        # output = self.save_dir + "/" + now.strftime("%Y%m%d_%H%M%S") + "video.mp4"
        # fourcc = cv2.VideoWriter_fourcc("m", "p", "4", "v")
        # outfh = cv2.VideoWriter(output, fourcc, 30, (shape[1], shape[0]))
        # for img in self.img_list:
        #     outfh.write(img)
        # outfh.release()

        return mean(self.powder_radius_list)

    # RealSense Subscriber callbacks
    def image_subscriber_callback(self, rgb_image_msg):
        self.rgb_image = rgb_image_msg
        frame = self.bridge.imgmsg_to_cv2(rgb_image_msg, "bgr8")
        if self.init_mortar_image_correction:
            self.mortar_image_correction(frame)
            self.init_mortar_image_correction = False

        self.publish_powder_info(frame)

        # save frames
        self.original_frame_list.append(frame)
        # save radius
        self.powder_radius_list.append(self.powder_radius)
        number_of_save_frames = int(self.camera_fps * self.stack_sec) + 1
        # remove over frames
        if len(self.powder_radius_list) > number_of_save_frames:
            self.powder_radius_list = self.powder_radius_list[
                len(self.powder_radius_list) - number_of_save_frames :
            ]
            self.original_frame_list = self.original_frame_list[
                len(self.original_frame_list) - number_of_save_frames :
            ]

    def depth_image_sub_callback(self, depth_image_msg):
        self.depth_image = depth_image_msg

    def camera_info_callback(self, cam_info_message):
        self.camera_info = cam_info_message

    def trackbar_callback(self):
        pass

    def get_Otsu_tureshold(self, gray):
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        hist_norm = hist.ravel() / hist.sum()
        Q = hist_norm.cumsum()
        bins = np.arange(256)
        fn_min = np.inf
        thresh = -1
        for i in range(1, 256):
            p1, p2 = np.hsplit(hist_norm, [i])  # probabilities
            q1, q2 = Q[i], Q[255] - Q[i]  # cum sum of classes
            if q1 < 1.0e-6 or q2 < 1.0e-6:
                continue
            b1, b2 = np.hsplit(bins, [i])  # weights
            # finding means and variances
            m1, m2 = np.sum(p1 * b1) / q1, np.sum(p2 * b2) / q2
            v1, v2 = (
                np.sum(((b1 - m1) ** 2) * p1) / q1,
                np.sum(((b2 - m2) ** 2) * p2) / q2,
            )
            # calculates the minimization function
            fn = v1 * q1 + v2 * q2
            if fn < fn_min:
                fn_min = fn
                thresh = i
        return thresh

    def mortar_image_correction(self, frame):
        img_correction = mortar_image_correction.MotarImageCorrection(
            vertex_length_mm=self.mortar_outer_max_diameter
        )

        # set mortar vertex points
        if self.mortar_calib:
            self.vertex_points = img_correction.set_vertex_points(frame, 8)
            self.mortar_calib = False

            df = pd.DataFrame(self.vertex_points)
            df.to_csv(
                self.save_dir + "%s_debug_data.csv" % datetime.datetime.now(),
                header=False,
                index=False,
            )

        # calc mortar center and pixcel length
        mortar_center, mm_per_pixcel = img_correction.calc_center_and_pixcel_length(
            self.vertex_points
        )
        radius = (self.mortar_inner_diameter + self.mask_diameter_margin) / 2
        mortar_radius_pixcel = int(radius / mm_per_pixcel)

        # set params
        enlarge_frame = self.enlarge_frame(frame)
        self.powder_detector.set_params(
            mm_per_pixcel,
            mortar_center,
            mortar_radius_pixcel,
            (enlarge_frame.shape[0], enlarge_frame.shape[1]),
        )

        self.mortar_center = mortar_center
        self.mm_per_pixcel = mm_per_pixcel
        self.mortar_radius_pixcel = mortar_radius_pixcel

        rospy.loginfo(" ===== mortar image is corrected!")

    def enlarge_frame(self, frame):
        return frame[40:270, 480:780]

    def publish_powder_info(self, original_frame):
        powder_detector = self.powder_detector

        # mortar area mask
        mortar_area_mask = np.zeros(original_frame.shape, dtype=np.uint8)
        cv2.ellipse(
            mortar_area_mask,
            (
                (self.mortar_center[0], self.mortar_center[1]),
                (self.mortar_radius_pixcel * 2, self.mortar_radius_pixcel * 1.6),
                0,
            ),
            (255, 255, 255),
            -1,
        )
        trimed_mortar_area_frame = original_frame & mortar_area_mask
        trimed_mortar_area_frame = self.enlarge_frame(trimed_mortar_area_frame)

        # binarry frame
        # calculate Otsu threshold value in the masked area (masked area mean nonzero area)
        # ref_frame = cv2.cvtColor(trimed_mortar_area_frame, cv2.COLOR_BGR2GRAY)
        # non_zero_frame = ref_frame[ref_frame.nonzero()]
        # threshold = self.get_Otsu_tureshold(cv2.GaussianBlur(non_zero_frame, (3, 3), 3))
        # threshold, _ = powder_detector.create_binary_image(
        #     trimed_mortar_area_frame, threshold, method=cv2.THRESH_BINARY
        # )
        # rospy.loginfo(threshold)

        # color filter
        #  bgr
        # hsvdata = cv2.cvtColor(trimed_mortar_area_frame, cv2.COLOR_BGR2HSV)
        # lower_hsv = np.array([100, 0, 0])
        # upper_hsv = np.array([115, 255, 255])
        # img_mask = cv2.inRange(hsvdata, lower_hsv, upper_hsv)
        # frame = cv2.bitwise_and(hsvdata, hsvdata, mask=img_mask)
        # color_filterd_frame = cv2.cvtColor(frame, cv2.COLOR_HSV2BGR)

        threshold, _ = powder_detector.create_binary_image(
            trimed_mortar_area_frame, self.binary_threshold, method=cv2.THRESH_BINARY
        )

        # stack frame
        powder_detector.create_stack_image(stack_sec=self.stack_sec)

        # powder_detector.detect_simple_contour()
        # powder_detector.detect_watershed()

        # calc contour distance
        # powder_detector.calc_contour_distance_and_center_of_gravity(
        #     powder_detector.watershed_contour, powder_detector.watershed_contour_frame
        # )

        powder_detector.fit_MEC(powder_detector.stack_frame)
        powder_detector.fit_MIC(powder_detector.stack_frame)
        ellipse_binary_frame = powder_detector.fit_ellipse(powder_detector.binary_frame)
        ellipse_stack_frame = powder_detector.fit_ellipse(powder_detector.stack_frame)

        self.powder_radius = powder_detector.ellipse_radius_major_mm_fixed

        # print(
        #     "major:",
        #     round(powder_detector.ellipse_radius_major_mm_fixed * 2, 2),
        #     "minor:",
        #     round(powder_detector.ellipse_radius_minor_mm_fixed * 2, 2),
        #     "MIC:",
        #     round(
        #         powder_detector.binary_MIC_radius * 2,
        #         2,
        #     ),
        # )

        ############ threshold check
        if self.threshold_check:
            cv2.namedWindow("window", cv2.WINDOW_KEEPRATIO)
            cv2.createTrackbar(
                "track", "window", self.binary_threshold, 255, self.trackbar_callback
            )
            threshold = cv2.getTrackbarPos("track", "window")
            ret, th = cv2.threshold(
                trimed_mortar_area_frame, threshold, 255, cv2.THRESH_BINARY
            )

            self.binary_threshold = threshold

            cv2.imshow("window", th)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                self.threshold_check = False
                cv2.destroyAllWindows()

        ############ imshow
        # cv_img = powder_detector.imshow(frame, scale=0.5, show=False)

        # publish debug img
        pub_binary = cv2.cvtColor(powder_detector.binary_frame, cv2.COLOR_GRAY2BGR)
        pub_stack = cv2.cvtColor(powder_detector.stack_frame, cv2.COLOR_GRAY2BGR)

        pub_img1 = cv2.hconcat([pub_stack, ellipse_stack_frame])
        pub_img2 = cv2.hconcat([pub_binary, ellipse_binary_frame])
        pub_img = cv2.vconcat([pub_img1, pub_img2])

        self.img_list.append(pub_img)

        self.debug_img_pub.publish(self.bridge.cv2_to_imgmsg(pub_img, "bgr8"))

        self.debug_img_pub_2.publish(
            self.bridge.cv2_to_imgmsg(trimed_mortar_area_frame, "bgr8")
        )

        # save img
        # cv2.imwrite(
        #     self.save_dir + "%s_cv_img.jpg" % datetime.datetime.now(),
        #     cv_img,
        # )


if __name__ == "__main__":
    rospy.init_node("grinding_vision", anonymous=True)
    c = GrindingVision(mortar_calib=False, threshold_check=False)
    rospy.spin()
