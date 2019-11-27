#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_l, line_r, fix_left, fix_right, check = self.line_detector.detect_lines()
        self.line_detector.show_images(line_l, line_r)
        angle = self.steer(line_l, line_r, fix_left, fix_right, check)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r, check)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right, fix_l, fix_r, ck):
        if ck:
            fix_mid = (fix_l + fix_r) // 2
            mid = (left + right) // 2
            angle = -(fix_mid - mid)
            if abs(angle) < 15:
                angle = 0
        else:
            angle = 0
        return angle

    def accelerate(self, angle, left, mid, right, ck):
        if ck:
            if angle == 0:
                speed = 40
            else:
                speed = 30
        else:
            speed = 0
        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)