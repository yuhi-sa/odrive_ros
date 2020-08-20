#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String
from odrive_ros.msg import Status
#odriveのパッケージ
import odrive
#odriveの初期値などが入っているパッケージ
from odrive.enums import *
#その他の利用パッケージ
import time
import matplotlib.pyplot as plt
import numpy as np

#起動しているodriveを見つけて太郎と名付ける
taro=odrive.find_any()
print("太郎発見・セットアップします") 
time.sleep(1)
#キャリブレーションする
taro.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
time.sleep(20)
#閉ループ回路に設定する
taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(1)
#コントロールモードを指定
taro.axis1.controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
time.sleep(1)
#ポジションゲインを10に設定する
taro.axis1.controller.config.pos_gain = 10.0

#制御モードの記録 0が速度，1が位置
mode = 0

print("太郎準備完了")
print("現在の制御モードは速度制御")

def callback(data):
    #subscriberのID　rospy.get_caller_id()
    #subscrineしたデータの中身　data.data
    rospy.loginfo(rospy.get_caller_id() + 'データきたー %s', data.data)

    global mode

    #速度制御と位置制御切り替え
    if data.data == "vel":
        taro.axis1.controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
        mode = 0
        print("速度制御に切り替わりました")
    elif data.data =="pos":
        taro.axis1.controller.config.control_mode=CTRL_MODE_POSITION_CONTROL
        mode = 1
        print("位置制御に切り替わりました")
    elif data.data == 'stop':
        taro.axis1.requested_state = AXIS_STATE_IDLE
    elif data.data == 'plotout':
        plotout()
    #太郎の制御
    elif mode == 0:
        taro.axis1.controller.vel_setpoint=int(data.data)
    elif mode == 1:
        taro.axis1.controller.pos_setpoint=int(data.data)
        

def controller():
    #おまじない　ノード名を宣言
    rospy.init_node('controller', anonymous=True)
    #Subscriberを作成．トピックを読み込む．
    sub = rospy.Subscriber('command', String, callback)
    #Publisherを作成('トピック名',型,サイズ)
    pub = rospy.Publisher('status', Status, queue_size=1)
    #ループの周期．
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        #ステータスデータを記録
        array=Status() #msgに作成したステータスメッセージファイル Status.msg
        array.pos = np.array(taro.axis1.encoder.pos_estimate % 8192 - 4096, dtype='f8')
        array.vel = np.array(taro.axis1.encoder.vel_estimate, dtype='f8')
        #データをパブリッシュ
        pub.publish(array)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInitException:
        pass