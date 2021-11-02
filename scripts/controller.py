#!/usr/bin/env python
# coding: utf-8
import rospy
from std_msgs.msg import String
from odrive_ros.msg import Status

import odrive
from odrive.enums import *

import time
import matplotlib.pyplot as plt
import numpy as np

taro=odrive.find_any()
print("Setup Start") 
time.sleep(1)
# Calibration
taro.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
time.sleep(20)
# Closed Loop Control
taro.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(1)
# Control Mode
taro.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
time.sleep(1)
# Position Gain：10
taro.axis1.controller.config.pos_gain = 10.0
# 制御モード(0：速度，1：位置)
mode = 0
print("現在の制御モードは速度制御")

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'データ取得: %s', data.data)
    global mode
    if data.data == "vel":
        taro.axis1.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        mode = 0
        print("速度制御に切り替わりました")
    elif data.data =="pos":
        taro.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        mode = 1
        print("位置制御に切り替わりました")
    elif data.data == 'stop':
        taro.axis1.requested_state = AXIS_STATE_IDLE
    elif data.data == 'plotout':
        plotout()
    elif mode == 0:
        taro.axis1.controller.vel_setpoint=int(data.data)
    elif mode == 1:
        taro.axis1.controller.pos_setpoint=int(data.data)

def controller():
    # Initialize Node
    rospy.init_node('controller', anonymous=True)
    # Subscriber
    sub = rospy.Subscriber('command', String, callback)
    # Publisher
    pub = rospy.Publisher('status', Status, queue_size=1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        array=Status()
        array.pos = np.array(taro.axis1.encoder.pos_estimate % 8192 - 4096, dtype='f8')
        array.vel = np.array(taro.axis1.encoder.vel_estimate, dtype='f8')
        pub.publish(array)
        rate.sleep()

if __name__ == '__main__':
    try:
        controller()
    except rospy.ROSInitException:
        pass