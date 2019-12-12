import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver


class GoToPlace:

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
        print("reft",left,"mid",mid,"right", right,)
        print("")
        if mid < 280:
            angle = -40
        elif mid > 360:
            angle = 40
        else:
            angle = 0
        return angle

    def accelerate(self, angle, left, mid, right):
        if min(left, mid, right) < 50:
            speed = 0
        if angle < -20 or angle > 20:
            speed = 25
        else:
            speed = 30
        return speed

#수평까지 이동 ( 원래 코드에서 왼/오 canny값 대신 받아와준 픽셀값 사용 )
    def gotoplace(self,pixel_left,pixel_right, mid):
        self.pixle_left= pixel_left
        self.pixle_right= pixel_right

        mid = (pixel_right + pixel_left) // 2

        if mid < 280:
            angle = -40
        elif mid > 360:
            angle = 40
        else:
            angle = 0
        return angle

#받아온 왼/오 canny와 인식한 왼/오 픽셀값이 일치 시 true를 리턴
        if left == pixel_left and right == pixel_right:
            return True

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = GoToPlace()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)