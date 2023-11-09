from cmath import sin
from math import dist
from black import write_cache
from cv2 import imshow
from skimage.feature import peak_local_max
from skimage.morphology import watershed
from scipy import ndimage

import copy
import cv2
import numpy as np
import rospy


class PowderDetector:
    def __init__(self, camera_fps):
        self.canny_thresh2 = 50  # エッジ検出そのものの閾値
        self.canny_thresh1 = 50  # 検出したエッジの隣接もエッジとするか
        self.morphology_karnel = np.ones((3, 3), np.uint8)
        self.binary_MIC_morphology_karnel_size = 3

        self.disrance_mask_size = 3
        self.sure_bg_karnel = np.ones((5, 5), np.uint8)
        self.sure_fg_coef = (
            0.8  # sure fore ground area is smoll as the coefficient gets bigger
        )

        self.diagonal_distortion_rate = 1.2
        self.camera_fps = camera_fps
        self.binary_stack_list = []

        self.frame_text_pos = (10, 30)

    def set_params(self, mm_per_pixcel, mortar_center, mortar_radius, shape):
        self.mm_per_pixcel = mm_per_pixcel
        self.mortar_center = mortar_center
        self.mortar_radius = mortar_radius
        self.start_stack_frame = np.zeros(shape, dtype=np.uint8)

    def create_binary_image(self, frame, thresh=130, method=cv2.THRESH_BINARY):
        self.origin_frame = frame
        self.gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        self.blur_frame = cv2.GaussianBlur(self.gray_frame, (3, 3), 3)
        if method == cv2.THRESH_OTSU:
            thresh = 0
        ret, binary_frame = cv2.threshold(self.blur_frame, thresh, 255, method)

        self.binary_frame = binary_frame
        self.binary_ret = ret

        # noise reduction with morphology
        karnel = np.ones(
            (
                self.binary_MIC_morphology_karnel_size,
                self.binary_MIC_morphology_karnel_size,
            ),
            np.uint8,
        )
        morphology_frame = cv2.morphologyEx(binary_frame, cv2.MORPH_CLOSE, karnel)

        self.binary_morphology_frame = morphology_frame

        return ret, morphology_frame

    def create_stack_image(self, stack_sec=0.5):
        fps = self.camera_fps

        # save binary frame
        self.binary_stack_list.append(self.binary_frame)

        # create a stack frame
        stack_frame = self.binary_stack_list[-1]
        for i in range(len(self.binary_stack_list)):
            stack_frame = cv2.bitwise_or(stack_frame, self.binary_stack_list[i])
        stack_frame = cv2.bitwise_or(stack_frame, self.start_stack_frame)

        self.stack_frame = stack_frame

        # delete past frames so as not to exceed stack sec
        number_of_frames = int(fps * stack_sec) + 1
        if len(self.binary_stack_list) > number_of_frames:
            self.binary_stack_list = self.binary_stack_list[
                len(self.binary_stack_list) - number_of_frames :
            ]

    def detect_watershed(self):
        frame = self.binary_frame

        # sure background 領域を抽出する。
        sure_bg_frame = cv2.dilate(frame, self.sure_bg_karnel, iterations=3)

        # 距離マップを作成する。
        dist = cv2.distanceTransform(frame, cv2.DIST_L2, self.disrance_mask_size)

        # sure foreground 領域を抽出する。
        ret, sure_fg_frame = cv2.threshold(dist, self.sure_fg_coef * dist.max(), 255, 0)
        sure_fg_frame = sure_fg_frame.astype(np.uint8)  # float32 -> uint8

        # 前景か背景か判断できない領域を抽出する。
        unknown_frame = cv2.subtract(sure_bg_frame, sure_fg_frame)

        # sure foreground にたいして、ラベル付を行う。
        ret, markers = cv2.connectedComponents(sure_fg_frame)  # bg=0,unknown and fg=1〜
        markers += 1  # bg=1,unknown and fg=2〜
        markers[
            unknown_frame == 255
        ] = 0  # unknown =0, bg=1, fg=2〜, labels for watershed

        # watershed アルゴリズムを適用する。
        watershed_frame = copy.deepcopy(self.origin_frame)
        markers = cv2.watershed(watershed_frame, markers)
        watershed_frame[markers == -1] = [255, 0, 0]  # watershedに色塗る

        labels = np.unique(markers)  # , return_counts=True)
        labels_for_debug = np.unique(markers, return_counts=True)
        # find contours
        contours_list = []
        for label in labels[2:]:  # watershedで得られた 0:背景ラベル １：境界ラベル は無視する。
            # ラベル label の領域のみ前景、それ以外は背景となる2値画像を作成する。
            target = np.where(markers == label, 255, 0).astype(np.uint8)

            # 作成した2値画像に対して、輪郭抽出を行う。
            contours, hierarchy = cv2.findContours(
                target, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            contours_list.append(contours[0])

        # calc contours area
        area_list = []
        for index, cnt in enumerate(contours_list):
            area_list.append(cv2.contourArea(cnt))
        if len(area_list) == 0:
            rospy.logwarn("contours not found")
            contour_frame = copy.deepcopy(frame)
            self.write_text(contour_frame, "contours not found")

            return False, False
        max_area_contour_index = area_list.index(max(area_list))
        contour_of_max_area = contours_list[max_area_contour_index]

        # show a contour of max area
        contour_frame = copy.deepcopy(
            self.gray_frame
        )  # np.zeros((frame.shape), np.uint8)
        cv2.drawContours(
            contour_frame,
            contours_list,
            max_area_contour_index,
            color=(0, 0, 0),
            thickness=2,
        )

        # show all countours
        contours_frame = copy.deepcopy(
            self.gray_frame
        )  # np.zeros((frame.shape), np.uint8)
        cv2.drawContours(
            contours_frame,
            contours_list,
            -1,
            color=(0, 0, 0),
            thickness=2,
        )

        # calc minEnclosingCircle
        (x, y), MEC_rad = cv2.minEnclosingCircle(contour_of_max_area)
        MEC_center = (int(x), int(y))

        center_mm_x = (MEC_center[0] - self.mortar_center[0]) * self.mm_per_pixcel
        center_mm_y = -(MEC_center[1] - self.mortar_center[1]) * self.mm_per_pixcel
        MEC_center_mm = [center_mm_x, center_mm_y]
        MEC_rad_mm = MEC_rad * self.mm_per_pixcel

        # calc maxInscribingCircle
        contour_masked_fram = np.zeros((frame.shape), np.uint8)
        cv2.drawContours(
            contour_masked_fram,
            contours_list,
            max_area_contour_index,
            color=(255, 255, 255),
            thickness=-1,
        )
        distance_fram = cv2.distanceTransform(
            contour_masked_fram,
            cv2.DIST_L2,
            self.disrance_mask_size,
            cv2.DIST_LABEL_PIXEL,
        )
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(distance_fram)
        MIC_frame = cv2.circle(
            copy.deepcopy(self.gray_frame), max_loc, int(max_val), (0, 0, 0), 2
        )
        center_mm_x = (max_loc[0] - self.mortar_center[0]) * self.mm_per_pixcel
        center_mm_y = -(max_loc[1] - self.mortar_center[1]) * self.mm_per_pixcel
        MIC_center_mm = [center_mm_x, center_mm_y]
        MIC_radius_mm = max_val * self.mm_per_pixcel
        self.write_text(
            MIC_frame,
            "radius: " + str(round(MIC_radius_mm, 2)),
            pos=(10, 120),
        )
        self.write_text(
            MIC_frame,
            "center: [ "
            + str(round(MIC_center_mm[0], 3))
            + ", "
            + str(round(MIC_center_mm[1], 3))
            + " ]",
            pos=(10, 190),
        )

        # over gray frame for debug
        watershed_MEC_frame = cv2.circle(
            copy.deepcopy(self.gray_frame), MEC_center, int(MEC_rad), (0, 0, 0), 2
        )
        self.write_text(
            watershed_MEC_frame,
            "radius: " + str(round(MEC_rad_mm, 2)),
            pos=(10, 120),
        )
        self.write_text(
            watershed_MEC_frame,
            "center: [ "
            + str(round(center_mm_x, 3))
            + ", "
            + str(round(center_mm_y, 3))
            + " ]",
            pos=(10, 190),
        )

        self.sure_bg_frame = sure_bg_frame
        self.dist_frame = dist
        self.sure_fg_frame = sure_fg_frame
        self.unknown_frame = unknown_frame
        self.watershed_contour_frame = contour_frame
        self.watershed_contours_frame = contours_frame
        self.watershed_MEC_frame = watershed_MEC_frame
        self.watershed_MEC_radius_pixcel = MEC_rad
        self.watershed_MEC_center_pixcel = MEC_center
        self.watershed_MEC_radius = MEC_rad_mm
        self.watershed_MEC_center = MEC_center_mm
        self.watershed_MIC_frame = MIC_frame
        self.watershed_MIC_center = MIC_center_mm
        self.watershed_MIC_radius = MIC_radius_mm

        self.watershed_contour = contour_of_max_area
        self.watershed_contours = contours_list
        self.watershed_contour_index = max_area_contour_index

        return MIC_center_mm, MIC_radius_mm

    def create_MEC_mask_frame(self, frame_shape, center, rad):
        MEC_mask_frame = np.zeros(frame_shape, dtype=np.uint8)
        cv2.circle(
            MEC_mask_frame,
            (center[0], center[1]),
            rad,
            color=255,
            thickness=-1,
        )
        return MEC_mask_frame

    def create_contour_mask_frame(self, frame_shape, contours, contour_index):
        contour_mask_frame = np.zeros(frame_shape, dtype=np.uint8)
        cv2.drawContours(
            contour_mask_frame, contours, contour_index, color=255, thickness=-1
        )
        return contour_mask_frame

    def calc_shade_rate(self, mask_frame, write_result_frame):
        binary_frame = self.binary_frame

        # calc area内のpixcel counts
        calc_area_pixcel_counts = cv2.countNonZero(mask_frame)

        # calc area内をmaskした時のwhite pixcelのcounts
        calc_area_masked_frame = binary_frame & mask_frame
        white_pixcel_counts = cv2.countNonZero(calc_area_masked_frame)

        # area内の白と黒の割合を計算
        white_rate = white_pixcel_counts / calc_area_pixcel_counts
        black_rate = 1 - white_rate

        # 結果を描画
        self.write_text(
            write_result_frame,
            "white: " + str(round(white_rate, 3)),
            pos=(10, 260),
        )
        self.write_text(
            write_result_frame,
            "black: " + str(round(black_rate, 3)),
            pos=(10, 340),
        )

        return [black_rate, white_rate]

    def fit_MEC(self, frame):
        # edge detection with Canny
        canny_frame = cv2.Canny(frame, self.canny_thresh1, self.canny_thresh2)
        self.canny_frame = canny_frame

        # edge integration with morphology calculation

        morphology_frame = cv2.morphologyEx(
            canny_frame, cv2.MORPH_CLOSE, self.morphology_karnel
        )
        self.morphology_frame = morphology_frame

        # contours detection
        contours, hierarchy = cv2.findContours(
            morphology_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        self.contours = contours

        # calc contours area
        area_list = []
        for index, cnt in enumerate(contours):
            area_list.append(cv2.contourArea(cnt))
        if len(area_list) == 0:
            rospy.logwarn("contours not found")
            contour_frame = copy.deepcopy(frame)
            self.write_text(contour_frame, "contours not found")

            return False, False
        max_area_contour_index = area_list.index(max(area_list))
        contour_of_max_area = contours[max_area_contour_index]

        # calc minEnclosingCircle
        (x, y), MEC_rad = cv2.minEnclosingCircle(contour_of_max_area)
        MEC_center = (int(x), int(y))
        MEC_rad = int(MEC_rad)

        ###################### show frame
        # show a contour of max area
        contour_frame = copy.deepcopy(
            self.origin_frame
        )  # np.zeros((frame.shape), np.uint8)
        cv2.drawContours(
            contour_frame, contours, max_area_contour_index, (255, 0, 0), 2
        )  # the third arg is contour number . if you choose -1, all contours show
        self.contour_frame = contour_frame

        contour_binary_frame = np.zeros((frame.shape), np.uint8)
        cv2.drawContours(
            contour_binary_frame,
            contours,
            max_area_contour_index,
            (255, 255, 255),
            -1,
        )  # the third arg is contour number . if you choose -1, all contours show
        self.contour_binary_frame = contour_binary_frame

        # show allcontours
        contours_frame = copy.deepcopy(self.gray_frame)
        cv2.drawContours(contours_frame, contours, -1, (255, 0, 0), 2)
        self.write_text(
            contours_frame,
            "contours counts: " + str(len(area_list)),
            pos=self.frame_text_pos,
        )
        self.contours_frame = contours_frame

        center_mm_x = (MEC_center[0] - self.mortar_center[0]) * self.mm_per_pixcel
        center_mm_y = -(MEC_center[1] - self.mortar_center[1]) * self.mm_per_pixcel
        MEC_center_mm = [center_mm_x, center_mm_y]
        MEC_rad_mm = MEC_rad * self.mm_per_pixcel

        # show circle over frame
        contour_MEC_frame = cv2.circle(
            copy.deepcopy(self.origin_frame),
            MEC_center,
            MEC_rad,
            (255, 0, 0),
            2,
        )
        self.write_text(
            contour_MEC_frame,
            "radius: " + str(round(MEC_rad_mm, 2)),
            pos=self.frame_text_pos,
            size=2,
        )
        # self.write_text(
        #     contour_MEC_frame,
        #     "center: [ "
        #     + str(round(center_mm_x, 3))
        #     + ", "
        #     + str(round(center_mm_y, 3))
        #     + " ]",
        #     pos=(10, 190),
        # )

        self.fitted_MEC_frame = contour_MEC_frame

        self.MEC_radius_pixcel = MEC_rad
        self.MEC_center_pixcel = MEC_center
        self.MEC_center = MEC_center_mm
        self.MEC_radius = MEC_rad_mm

        self.contour = contour_of_max_area
        self.contours = contours
        self.contour_index = max_area_contour_index

        return MEC_center_mm, MEC_rad_mm

    def fit_MIC(self, frame):
        distance_fram = cv2.distanceTransform(
            frame,
            cv2.DIST_L2,
            self.disrance_mask_size,
            cv2.DIST_LABEL_PIXEL,
        )
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(distance_fram)
        MIC_frame = cv2.circle(
            copy.deepcopy(self.origin_frame), max_loc, int(max_val), (255, 0, 0), 2
        )
        center_mm_x = (max_loc[0] - self.mortar_center[0]) * self.mm_per_pixcel
        center_mm_y = -(max_loc[1] - self.mortar_center[1]) * self.mm_per_pixcel
        MIC_center_mm = [center_mm_x, center_mm_y]
        MIC_center_distance = np.sqrt(center_mm_x**2 + center_mm_y**2)
        MIC_radius_mm = max_val * self.mm_per_pixcel / 2

        MIC_info_frame = copy.deepcopy(MIC_frame)
        self.write_text(
            MIC_info_frame,
            "radius: " + str(round(MIC_radius_mm * 2, 2)),
            pos=self.frame_text_pos,
            size=2,
        )
        # self.write_text(
        #     MIC_info_frame,
        #     "center: [ "
        #     + str(round(MIC_center_mm[0], 3))
        #     + ", "
        #     + str(round(MIC_center_mm[1], 3))
        #     + " ]",
        #     pos=(10, 40),
        #     size=1,
        # )
        # self.write_text(
        #     MIC_info_frame,
        #     "distance: " + str(round(MIC_center_distance, 2)),
        #     pos=(10, 60),
        #     size=1,
        # )
        self.fitted_MIC_frame = MIC_frame
        self.MIC_info_frame = MIC_info_frame
        self.MIC_center_pixcel = max_loc
        self.MIC_radius_pixcel = max_val / 2
        self.MIC_center = MIC_center_mm
        self.MIC_radius = MIC_radius_mm

    def fit_ellipse(self, frame):
        contours, hierarchy = cv2.findContours(
            frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        if len(contours) == 0:
            return False

        max_contour = max(contours, key=lambda x: cv2.contourArea(x))
        ellipse = cv2.fitEllipse(max_contour)

        fited_ellipse_frame = cv2.ellipse(
            copy.deepcopy(self.origin_frame), ellipse, (0, 0, 255), 2
        )

        ellipse_center = list(ellipse[0])
        center_mm_x = (ellipse_center[0] - self.mortar_center[0]) * self.mm_per_pixcel
        center_mm_y = -(ellipse_center[1] - self.mortar_center[1]) * self.mm_per_pixcel
        ellipse_center_mm = [center_mm_x, center_mm_y]

        self.fited_ellipse_frame = fited_ellipse_frame
        self.ellipse_center = list(ellipse[0])
        self.ellipse_radius_major = max(ellipse[1]) / 2
        self.ellipse_radius_minor = min(ellipse[1]) / 2
        self.ellipse_radius_major_mm = max(ellipse[1]) * self.mm_per_pixcel / 2
        self.ellipse_radius_minor_mm = min(ellipse[1]) * self.mm_per_pixcel / 2
        self.ellipse_angle = ellipse[2]

        # fix major radius
        theta = (90 - self.ellipse_angle) / 360 * 2 * np.pi
        alpha = self.diagonal_distortion_rate
        fix_rate = 1 - (1 - alpha) * abs(sin(theta))
        self.ellipse_radius_major_mm_fixed = self.ellipse_radius_major_mm * fix_rate

        # fix minor radius
        theta = self.ellipse_angle / 360 * 2 * np.pi
        alpha = self.diagonal_distortion_rate
        fix_rate = 1 - (1 - alpha) * abs(sin(theta))
        self.ellipse_radius_minor_mm_fixed = self.ellipse_radius_minor_mm * fix_rate

        self.fited_ellipse_info_frame = copy.deepcopy(fited_ellipse_frame)
        self.write_text(
            self.fited_ellipse_info_frame,
            "radius: " + str(round(self.ellipse_radius_major_mm, 2)),
            pos=self.frame_text_pos,
            size=2,
        )

        return fited_ellipse_frame

    def calc_contour_distance_and_center_of_gravity(self, contour, draw_frame):
        # reshape 3D to 2D
        reshaped_contour = contour.reshape(contour.shape[0], 2)

        # calc centermorphology_frameotar_center[1]) * self.mm_per_pixcel

        # calc max distance of vertex points
        all_diffs = np.expand_dims(reshaped_contour, axis=1) - np.expand_dims(
            reshaped_contour, axis=0
        )
        distance_matrix = np.sqrt(np.sum(all_diffs**2, axis=-1)).astype(np.int32)
        distance = distance_matrix.max()
        distance_mm = distance * self.mm_per_pixcel

        # draw center
        cv2.circle(
            draw_frame,
            center,
            5,
            (0, 0, 0),
            3,
        )

        # draw distance
        # cv2.line(
        #     draw_frame,
        #     [],
        #     [],
        #     (0, 0, 0),
        #     2,
        # )

        self.write_text(
            draw_frame,
            "max distance/2: " + str(round(distance_mm / 2, 2)),
            pos=(10, 120),
        )
        self.write_text(
            draw_frame,
            "center: [ "
            + str(round(center_mm_x, 3))
            + ", "
            + str(round(center_mm_y, 3))
            + " ]",
            pos=(10, 190),
        )

        return [center_mm_x, center_mm_y], distance_mm / 2

    def resize(self, frame, scale=1):
        return cv2.resize(frame, dsize=None, fx=scale, fy=scale)

    def write_text(self, img, text, size=3, pos=(10, 50)):
        cv2.putText(
            img=img,
            text=text,
            org=pos,
            fontFace=cv2.FONT_HERSHEY_PLAIN,
            fontScale=size,
            color=(255, 255, 255),
            thickness=2,
            lineType=cv2.LINE_AA,
        )

    def imshow(self, origin_frame, scale=1, show=True):
        self.write_text(origin_frame, "origin")
        self.write_text(self.gray_frame, "gray")
        self.write_text(self.blur_frame, "blur")
        self.write_text(self.binary_frame, "thresh")
        self.write_text(self.canny_frame, "canny")
        self.write_text(self.morphology_frame, "morphology")
        self.write_text(self.sure_bg_frame, "sure bg")
        self.write_text(self.dist_frame, "dist")
        self.write_text(self.sure_fg_frame, "sure fg")
        self.write_text(self.unknown_frame, "unknown")
        self.write_text(self.contour_frame, "simple contour")
        self.write_text(self.contours_frame, "simple contours")

        self.write_text(self.watershed_contour_frame, "watershed contour")
        self.write_text(self.watershed_contours_frame, "watershed contours")
        self.write_text(self.watershed_MEC_frame, "watershed contour MEC")
        self.write_text(self.watershed_MIC_frame, "watershed contour MIC")
        self.write_text(self.contour_MEC_frame, "simple contour MEC")
        self.write_text(self.binary_MIC_frame, "threshold MIC")
        self.write_text(self.binary_morphology_frame, "threshold morphology")

        self.write_text(
            self.binary_frame, "ret: " + str(self.binary_ret), pos=(10, 120)
        )

        img1 = cv2.vconcat([self.watershed_MIC_frame, self.contour_MEC_frame])
        img2 = cv2.vconcat([self.watershed_contour_frame, self.binary_MIC_frame])
        img3 = cv2.vconcat([self.binary_morphology_frame, self.binary_frame])
        imgs = cv2.hconcat([img1, img2, img3])

        if show:
            cv2.imshow("imgs", self.resize(imgs, scale))
            cv2.imshow("origin", origin_frame)
        return imgs
