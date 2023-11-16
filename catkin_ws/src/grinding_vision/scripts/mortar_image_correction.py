import numpy as np
import cv2
import rospy


class MotarImageCorrection:
    def __init__(self, vertex_length_mm) -> None:
        self.points = []
        self.index = 0
        self.vertex_length_mm = vertex_length_mm

    def mouse_event(self, event, x, y, flags, param):
        # 配列外参照回避
        if self.index > (self.data_num - 1):
            return
        # クリック地点を配列に格納
        if event == cv2.EVENT_LBUTTONUP:
            self.points.append([x, y])  # 格納
            cv2.circle(self.vertex_img, (x, y), 5, (255, 0, 0), -1)
            self.index += 1

    def set_vertex_points(self, img, data_num):
        vertex_img = img.copy()
        self.vertex_img = vertex_img
        self.data_num = data_num

        # ウィンドウ生成
        cv2.namedWindow("click vertex points", cv2.WINDOW_KEEPRATIO)

        # マウスイベント時に関数mouse_eventの処理を行う
        cv2.setMouseCallback("click vertex points", self.mouse_event)

        # 「q」が押されるまでループ
        while True:
            # 画像の表示
            cv2.imshow("click vertex points", vertex_img)

            # キー入力
            if cv2.waitKey(1) & 0xFF == ord("q"):
                if len(self.points) == self.data_num:
                    break
                else:
                    rospy.logwarn(
                        "data points not enough %s" % str(len(self.points))
                        + "/"
                        + str(data_num)
                    )

        cv2.destroyAllWindows()

        self.points = np.array(self.points)
        rospy.loginfo("mortar_vertex_points:%s" % self.points)

        return self.points

    def calc_center_and_pixcel_length(self, vertex_points):
        # calc center
        x = np.array([p[0] for p in vertex_points])
        y = np.array([p[1] for p in vertex_points])
        center = [int(np.average(x)), int(np.average(y))]

        # calc max distance of vertex points
        all_diffs = np.expand_dims(vertex_points, axis=1) - np.expand_dims(
            vertex_points, axis=0
        )
        distance = np.sqrt(np.sum(all_diffs**2, axis=-1)).astype(np.int32)
        mm_per_pixcel = self.vertex_length_mm / distance.max()

        return center, mm_per_pixcel
