#!/usr/bin/env python

import rospy
from custom_msg_pkg.msg import ControlMsg  # 메시지 파일 import

def talker():
    pub = rospy.Publisher('control_topic', ControlMsg, queue_size=10)
    rospy.init_node('control_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        control_msg = ControlMsg()
        control_msg.header.stamp = rospy.Time.now()
        control_msg.target_speed = 0  # 디버깅용 예시 데이터
        control_msg.target_angle = 90    # 디버깅용 예시 데이터
        
        rospy.loginfo(control_msg)
        pub.publish(control_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
