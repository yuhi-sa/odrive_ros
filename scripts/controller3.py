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

#制御モードの記録 0が速度，1が位置, 2が位置ゲイン, 3が速度Pゲイン, 4が速度Iゲイン
#mode = 0

print("太郎準備完了")
print("現在の制御モードは速度制御")

def callback(data):
    #subscriberのID　rospy.get_caller_id()
    #subscribeしたデータの中身　data.data
    rospy.loginfo(rospy.get_caller_id() + 'データきたー %s', data.data)

    instr, val = separate_instr_val(data.data)
    print('命令:{}, 値:{}'.format(instr, val))

    #制御内容切り替え
    if instr == "vel":
        taro.axis1.controller.config.control_mode=CTRL_MODE_VELOCITY_CONTROL
        time.sleep(1)
        taro.axis1.controller.vel_setpoint=int(val)
        #mode = 0
        print("速度制御に切り替わりました")
    elif instr == "pos":
        taro.axis1.controller.config.control_mode=CTRL_MODE_POSITION_CONTROL
        time.sleep(1)
        taro.axis1.controller.pos_setpoint=int(val)
        #mode = 1
        print("位置制御に切り替わりました")

    #ゲイン調整
    elif instr == "pos_gain":
        print("位置制御のゲインの調整を行います(初期値10)")
        taro.axis1.requested_state = AXIS_STATE_IDLE
        time.sleep(1)
        taro.axis1.controller.config.pos_gain=val
        taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        #mode = 2
    elif instr == "vel_gain":
        print("速度制御のPゲインの調整を行います(初期値5/10000)")
        taro.axis1.requested_state = AXIS_STATE_IDLE
        time.sleep(1)
        taro.axis1.controller.config.vel_gain=val
        taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        #mode = 3
    elif instr == "vel_in_gain":
        print("速度制御のIゲインの調整を行います(初期値10/10000)")
        taro.axis1.requested_state = AXIS_STATE_IDLE
        time.sleep(1)
        taro.axis1.controller.config.vel_integrator_gain=val
        taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        #mode = 4

    #情報確認（ゲイン，コマンド）
    elif instr == "info":
        print("現在の位置ゲインです")
        #taro.axis1.controller.config.pos_gain
        time.sleep(1)
        print("現在の速度Pゲインです")
        #taro.axis1.controller.config.vel_gain
        time.sleep(1)
        print("現在の速度Iゲインです")
        #taro.axis1.controller.config.vel_integrator_gain
        time.sleep(1)
        print("コマンドは以下の通りです")
        print("vel:速度制御, pos:位置制御, \
        \n pos_gain:位置ゲイン調整, vel_gain:速度Pゲイン調整, vel_in_gain:速度Iゲイン調整 \
        \n stop:停止, reboot:リキャリブレーション(ゲインはそのまま)")

    #停止
    elif instr == 'stop':
        taro.axis1.requested_state = AXIS_STATE_IDLE

    #再起動
    elif instr == 'reboot':
        print("太郎！新しい顔よ！")
        #キャリブレーションする
        taro.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        time.sleep(20)
        #閉ループ回路に設定する
        taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1)
        #コントロールモードを指定
        taro.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        time.sleep(1)
        print("太郎準備完了")
        print("現在の制御モードは速度制御")

        #太郎の制御
        #elif mode == 0:
        #    taro.axis1.controller.vel_setpoint=int(val)
        #elif mode == 1:
        #    taro.axis1.controller.pos_setpoint=int(val)
        #elif mode == 2:
        #    taro.axis1.controller.config.pos_gain=val
        #    taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #elif mode == 3:
        #    taro.axis1.controller.config.vel_gain=val
        #    taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        #elif mode == 4:
        #    taro.axis1.controller.config.vel_integrator_gain=val
        #    taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


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
        array.pos_gain = np.array(taro.axis1.controller.config.pos_gain, dtype='f8')
        array.vel_gain_P = np.array(taro.axis1.controller.config.vel_gain, dtype='f8')
        array.vel_gain_I = np.array(taro.axis1.controller.config.vel_integrator_gain, dtype='f8')
        #データをパブリッシュ
        pub.publish(array)
        rate.sleep()

#入力されたデータの分割
def separate_instr_val(command):
    if len(command.split(' ')) == 2:
        instr, val = command.split(' ')
        val = to_float(val)
    elif command in ['stop', 'reboot', 'info']:
        instr = command
        val = None
    else:
        instr = None
        val = None
    return instr, val
#入力値のfloat変換（エラー対応）
def to_float(val):
    try:
        return float(val)
    except ValueError:
        return None


if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInitException:
        pass
