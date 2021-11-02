#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String

def commander():
    # Publisher
    pub = rospy.Publisher('command', String, queue_size=10)
    # Initialize Node
    rospy.init_node('commander', anonymous=True)
    rate = rospy.Rate(10) 
    
    print("指令値を入力してください||制御方法の切り替えは，vel:速度制御 pos:位置制御")
    print("現在の制御モードは速度制御")

    while not rospy.is_shutdown():
        msg = str()
        msg = input()
        pub.publish(msg)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass
