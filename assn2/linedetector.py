import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.roi_vertical_pos = 300
        self.scan_height = 20
        self.image_width = 640
        self.scan_width = 250
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

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.0
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edge = cv2.Canny(blur, 30, 150)
        self.edge = cv2.cvtColor(edge, cv2.COLOR_GRAY2BGR)

    def detect_lines(self):
        lmid = self.scan_width
        rmid = self.image_width - lmid

        pixel_cnt_threshold = 0.3 * self.area_width * self.area_height

        self.cam_img = cv2.rectangle(self.cam_img, (0, self.roi_vertical_pos),
                              (self.image_width - 1, self.roi_vertical_pos + self.scan_height),
                              (255, 0, 0), 3)

        hsv2 = self.edge

        avg_value = np.average(hsv2[:, :, 2])
        value_threshold = avg_value * 1.0

        lbound_1 = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound_1 = np.array([255, 128, 255], dtype=np.uint8)

        bin_1 = cv2.inRange(hsv2, lbound_1, ubound_1)

        left_start = -1
        for l in range(lmid, self.area_width, -1):
            area = bin_1[self.row_begin:self.row_end, l: l + self.area_width]
            if cv2.countNonZero(area) > 10:
                left_start = l - self.area_width
                break

        right_start = -1
        for r in range(rmid, self.image_width - self.area_width):
            area = bin_1[self.row_begin:self.row_end, r - self.area_width:r]
            if cv2.countNonZero(area) > 10:
                right_start = r + self.area_width
                break

        left, right = -1, -1

        for l in range(left_start, lmid):
            area = bin[self.row_begin:self.row_end, l: l + self.area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                left = l
                break

        for r in range(right_start, rmid, -1):
            area = bin[self.row_begin:self.row_end, r - self.area_width: r]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                right = r
                break

        # Return positions of left and right lines detected.
        return left, right

    def show_images(self, left, right):
        if left != -1:
            self.cam_img = cv2.rectangle(self.cam_img,
                                    (left, self.row_begin),
                                    (left + self.area_width, self.row_end),
                                    (0, 255, 255), 2)
        else:
            print("Lost left line")
        if right != -1:
            self.cam_img = cv2.rectangle(self.cam_img,
                                    (right - self.area_width, self.row_begin),
                                    (right, self.row_end),
                                    (0, 255, 255), 2)
        else:
            print("Lost right line")

        cv2.imshow("view", self.cam_img)
        cv2.waitKey(1)
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
        pass
