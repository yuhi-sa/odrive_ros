#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String

def commander():
    #Publisherを作成('トピック名',型,サイズ)
    pub = rospy.Publisher('command', String, queue_size=10)
    #ノード名の宣言
    rospy.init_node('commander', anonymous=True)
    #ループの周期
    rate = rospy.Rate(10) 
    
    print("指令値を入力してください||制御方法の切り替えは，vel:速度制御 pos:位置制御")
    print("現在の制御モードは速度制御")

    while not rospy.is_shutdown():
        #標準入力からkeyを入力
        msg = str()
        msg = input()

        print("指令値を入力してください||制御方法の切り替えは，vel:速度制御 pos:位置制御")
        #指令値をパブリッシュする
        pub.publish(msg)
        rate.sleep()
    

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass
