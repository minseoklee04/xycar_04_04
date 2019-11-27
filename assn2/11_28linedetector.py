# !/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.left, self.right, self.fix_left, self.fix_right = -1, -1, -1, -1
        self.roi_vertical_pos = 290
        self.scan_height = 20
        self.image_width = 640
        self.scan_width = 260
        self.area_width = 20
        self.area_height = 10
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.bridge = CvBridge()
        rospy.Subscriber(topic, Image, self.conv_image)
        # test
        self.cnt = 0
        self.check = False

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.3
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, 10, 100)
        self.edge = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)

    def detect_lines(self):
        if self.cnt < 30:
            lmid = self.scan_width
            rmid = self.image_width - lmid

            pixel_cnt_threshold = 0.3 * self.area_width * self.area_height

            self.cam_img = cv2.rectangle(self.cam_img, (0, self.roi_vertical_pos),
                                         (self.image_width - 1, self.roi_vertical_pos + self.scan_height),
                                         (255, 0, 0), 3)

            hsv2 = cv2.cvtColor(self.edge, cv2.COLOR_BGR2HSV)

            avg_value = np.average(hsv2[:, :, 2])
            value_threshold = avg_value * 1.0

            lbound_1 = np.array([0, 0, value_threshold], dtype=np.uint8)
            ubound_1 = np.array([255, 128, 255], dtype=np.uint8)

            bin_1 = cv2.inRange(hsv2, lbound_1, ubound_1)

            left_start = -1
            for l in range(lmid, self.area_width, -1):
                area = bin_1[self.row_begin:self.row_end, l: l + self.area_width]
                if cv2.countNonZero(area) > 1:
                    left_start = l - self.area_width
                    break

            right_start = -1
            for r in range(rmid, self.image_width - self.area_width):
                area = bin_1[self.row_begin:self.row_end, r - self.area_width:r]
                if cv2.countNonZero(area) > 1:
                    right_start = r + self.area_width
                    break

            if right_start != -1 and left_start != -1:
                self.cnt += 1

            if left_start != -1:
                for l in range(left_start, lmid):
                    area = self.mask[self.row_begin:self.row_end, l: l + self.area_width]
                    if cv2.countNonZero(area) > pixel_cnt_threshold:
                        if self.cnt < 15 or self.fix_left == -1:
                            self.fix_left = l
                        else:
                            if abs(self.fix_left - l) <= 5:
                                self.fix_left = l
                    break

            if right_start != -1:
                for r in range(right_start, rmid, -1):
                    area = self.mask[self.row_begin:self.row_end, r - self.area_width: r]
                    if cv2.countNonZero(area) > pixel_cnt_threshold:
                        if self.cnt < 15 or self.fix_left == -1:
                            self.fix_right = r
                        else:
                            if abs(self.fix_right - r) <= 5:
                                self.fix_right = r
                    break

            print("")
            print("fix_left:", self.fix_left, "fix_right:", self.fix_right)
            print("l_s", left_start, "r_S", right_start)
            if self.cnt == 30:
                self.check = True
            return self.left, self.right, self.fix_left, self.fix_right, self.check

        else:
            lmid = self.scan_width
            rmid = self.image_width - lmid

            pixel_cnt_threshold = 0.3 * self.area_width * self.area_height

            self.cam_img = cv2.rectangle(self.cam_img, (0, self.roi_vertical_pos),
                                         (self.image_width - 1, self.roi_vertical_pos + self.scan_height),
                                         (255, 0, 0), 3)

            hsv2 = cv2.cvtColor(self.edge, cv2.COLOR_BGR2HSV)

            avg_value = np.average(hsv2[:, :, 2])
            value_threshold = avg_value * 1.0

            lbound_1 = np.array([0, 0, value_threshold], dtype=np.uint8)
            ubound_1 = np.array([255, 128, 255], dtype=np.uint8)

            bin_1 = cv2.inRange(hsv2, lbound_1, ubound_1)

            left_start = -1
            for l in range(lmid, self.area_width, -1):
                area = bin_1[self.row_begin:self.row_end, l: l + self.area_width]
                if cv2.countNonZero(area) > 1:
                    left_start = l - self.area_width
                    break

            right_start = -1
            for r in range(rmid, self.image_width - self.area_width):
                area = bin_1[self.row_begin:self.row_end, r - self.area_width:r]
                if cv2.countNonZero(area) > 1:
                    right_start = r + self.area_width
                    break

            if right_start > self.fix_right - 15:
                for l in range(left_start, lmid):
                    area = self.mask[self.row_begin:self.row_end, l: l + self.area_width]
                    if cv2.countNonZero(area) > pixel_cnt_threshold:
                        self.left = l
                        break

            if left_start < self.fix_left + 15:
                for r in range(right_start, rmid, -1):
                    area = self.mask[self.row_begin:self.row_end, r - self.area_width: r]
                    if cv2.countNonZero(area) > pixel_cnt_threshold:
                        self.right = r
                        break

            print("")
            print("fix_left:", self.fix_left, "fix_right:", self.fix_right)
            print("left:", self.left, "right:", self.right)
            print("start_l:", left_start, "start_r", right_start)

            # Return positions of left and right lines detected.
            return self.left, self.right, self.fix_left, self.fix_right, self.check

    def show_images(self, left, right):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
        pass
