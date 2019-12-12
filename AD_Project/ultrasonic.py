#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray
from filter import MovingAverage
# 이미지를 받아오고서 차단기를 인식하면 True를 출력해서 True이면 실행하도록
# 근데 이러면 필터를 할 필요가 없어짐
# import recognize_bar

motor_pub = None
usonic_data = None

class ultrasonic:
    # 노드생성
    def init_node(self, callback):
        global motor_pub
        rospy.init_node('ultrasonic')
        rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
        motor_pub = rospy.Publisher('xycar_motor_msg',
                                    Int32MultiArray, queue_size=1)

    def exit_node(self):
        return

    # def drive(angle, speed):
    #     global motor_pub
    #     drive_info = [angle, speed]
    #     pub_data = Int32MultiArray(data=drive_info)
    #     motor_pub.publish(pub_data)

    def callback(self, data):
        global usonic_data
        usonic_data = data.data

    # 여기서는 앞에 방해물이 있으면 True를 리턴하도록
    def obstruction_detect(self):
        ma = MovingAverage(10)
        rate = rospy.Rate(10)

        # 일단 초반 10개 값은 받고
        while len(ma.data) <= 10:
            ma.add_sample(usonic_data[1])
            rate.sleep()

        # 그 다음부터 튀는값 제거
        # 평균의 5정도 사이가 아니라면 무시
        while True:
            # 평균값 근처
            if ma.getmm - 5 < usonic_data[1] < ma.getmm + 5:
                ma.add_sample(usonic_data[1])

            # 튀는값이 나오면 리스트의 맨 마지막 값을 추가
            else:
                ma.add_sample(ma.data[-1])
            rate.sleep()

            # 장애물 탐지
            if usonic_data[1] < 30 and (ma.getmm - 5 < usonic_data[1] < ma.getmm + 5):
                return True

            # 운전코드에서 True값이 리턴되면 정지하도록 하면 좋을거같음
