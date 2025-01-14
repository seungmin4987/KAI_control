#!/usr/bin/env python

import rospy
import math
from custom_msg_pkg.msg import ControlMsg  # 메시지 파일 import

def talker(amplitude, offset, period):
    pub = rospy.Publisher('control_topic', ControlMsg, queue_size=10)
    rospy.init_node('control_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10Hz 주기로 메시지 전송

    start_time = rospy.Time.now().to_sec()  # 시작 시간 기록

    while not rospy.is_shutdown():
        current_time = rospy.Time.now().to_sec() - start_time  # 경과 시간 계산
        sine_input = amplitude * math.sin(2 * math.pi * current_time / period) + offset

        control_msg = ControlMsg()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.target_speed =  0 #sine_input  # 사인 입력을 속도에 적용
        control_msg.target_angle = sine_input  # 각도는 고정된 값으로 설정 (필요에 따라 변경 가능)

        rospy.loginfo("Publishing ControlMsg: speed=%.2f, angle=%.2f" % (control_msg.target_speed, control_msg.target_angle))
        pub.publish(control_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # 사용자 정의 진폭, 오프셋, 주기를 매개변수로 설정
        amplitude = float(input("Enter amplitude: "))
        offset = float(input("Enter offset: "))
        period = float(input("Enter period (seconds): "))

        talker(amplitude, offset, period)
    except rospy.ROSInterruptException:
        pass

