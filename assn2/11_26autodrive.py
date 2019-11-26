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
        line_l, line_r = self.line_detector.detect_lines()
        self.line_detector.show_images(line_l, line_r)
        angle = self.steer(line_l, line_r)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2
        angle = 0
        if 280 < mid < 360:
            if left > 70:
                angle += 40
            elif right < 570:
                angle -= 40
        elif mid < 280:
            if right < 570:
                angle -= 40
        if 600 <= right <= 640:
            if mid > 360:
                angle -= 20
            else:
                angle += 0
        elif 570 <= right < 600:
            angle -= 30
            if mid < 280:
                angle -= 10

        if 0 <= left < 65:
            if mid < 280:
                angle += 20
            else:
                angle += 0
        elif 65 <= left < 130:
            angle += 30
            if mid > 360:
                angle += 10

        print("left", left, "mid", mid, "right", right, )
        print("")

        return angle

    def accelerate(self, angle, left, mid, right):
        if min(left, mid, right) < 50:
            speed = 0
        if angle < -20 or angle > 20:
            speed = 25
        else:
            speed = 30
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
