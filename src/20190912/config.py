# -*- coding: utf-8 -*-
import numpy as np
import PID
import pigpio
import time
exe_time = 30 # Bigger, slower
image_shape = (4 * exe_time, 4 * exe_time)
rate = 0.6 # 控制 image transform

# image color range
lower = {}
upper = {}
lower['black'] = np.array([0,0,0])
lower['gray'] = np.array([0,0,46])
lower['blue'] = np.array([85,100,100])
lower['green'] = np.array([37,43,84])
lower['red_1'] = np.array([160,66,80])
lower['red_2'] = np.array([0,66,80])

upper['black'] = np.array([180,180,46])
upper['gray'] = np.array([180,43,220])
upper['blue'] = np.array([120,255,255])
upper['green'] = np.array([75,255,255])
upper['red_1'] = np.array([180,255,255])
upper['red_2'] = np.array([10,255,255])
color_area = image_shape[0]*image_shape[1] // 9

pins = {'throttle':17, 'pitch':17, 'roll':17, 'yaw':17}
directions = ['throttle', 'pitch', 'roll', 'yaw']
# altitude hold 588 ~ 600
dc_min = {'throttle':506, 'pitch':631, 'roll':756, 'yaw':881}
dc_max = {'throttle':612, 'pitch':745, 'roll':870, 'yaw':995}
safety_min = {'throttle':549, 'pitch':660, 'roll':785, 'yaw':925}
safety_max = {'throttle':583, 'pitch':716, 'roll':841, 'yaw':951}
use_safety = True

pi = pigpio.pi()
for direction in directions:
    pin = pins[direction]
    pi.set_PWM_frequency(pin, 50)
    pi.set_PWM_range(pin, 10000) 
    pi.set_PWM_dutycycle(pin, 0)

pid = {}
# max_gain = {'throttle':1000, 'pitch':1000, 'roll':1000, 'yaw':10}
for direction in directions:
    pid[direction] = PID.PID(0, 0, 0, 1000)
    pid[direction].SetPoint=0

def set_pid(mission_name):
    def take_off_pid():
        pid['throttle'].setKp(0.01)
        pid['throttle'].setKi(0.00)
        pid['throttle'].setKd(0.01)
    def altitude_hold_pid():
        pid['throttle'].setKp(0.015)
        pid['throttle'].setKi(0.005)
        pid['throttle'].setKd(0.08)
    def landing_pid():
        pid['throttle'].setKp(0.015)
        pid['throttle'].setKi(0.005)
        pid['throttle'].setKd(0.01)
    def loiter_pid():
        pid['roll'].setKp(0.007)
        pid['roll'].setKi(0.000)
        pid['roll'].setKd(0.0085)
        pid['pitch'].setKp(0.007)
        pid['pitch'].setKi(0.000)
        pid['pitch'].setKd(0.0085)
        
        pid['yaw'].setKp(0.12)
        pid['yaw'].setKi(0)
        pid['yaw'].setKd(0.02)
    def line_follow_pid():
        pid['roll'].setKp(0.005)
        pid['roll'].setKi(0.005)
        pid['roll'].setKd(0.0001)
        pid['pitch'].setKp(0.005)
        pid['pitch'].setKi(0)
        pid['pitch'].setKd(0)
        '''
        pid['yaw'].setKp(0.01)
        pid['yaw'].setKi(0)
        pid['yaw'].setKd(0.001)'''
    if mission_name == 'takeoff':
        take_off_pid()
        loiter_pid()
    if mission_name == 'takeoff_loiter':
        altitude_hold_pid()
        loiter_pid()
    elif (mission_name == 'line_follow_1') or (mission_name == 'line_follow_2') or (mission_name == 'line_follow_3'):
        altitude_hold_pid()
        line_follow_pid()
    elif mission_name == 'led_finding':
        altitude_hold_pid()
        line_follow_pid()
    elif mission_name == 'led_loiter':
        altitude_hold_pid()
        loiter_pid()
    elif mission_name == 'sandbag_throwing':
        altitude_hold_pid()
        line_follow_pid()
    elif mission_name == 'line_follow_3':
        altitude_hold_pid()
        line_follow_pid()
    elif mission_name == 'landing':
        landing_pid()
        loiter_pid()

def output_to_apm(apm_input):
    dc = {}
    dc_rate = {}
    # apm_input['throttle'] = 0
    # apm_input['roll'] = 0
    # apm_input['pitch'] = 0
    # apm_input['yaw'] = 0
    for direction in directions:
        pin = pins[direction]
        dc_mid = (dc_min[direction] + dc_max[direction]) // 2
        dc_range = dc_max[direction] - dc_mid
        dc[direction] = int(apm_input[direction] * dc_range + dc_mid)
        if use_safety:
            if dc[direction] > safety_max[direction]:
                dc[direction] = safety_max[direction]
            elif dc[direction] < safety_min[direction]:
                dc[direction] = safety_min[direction]
        else:
            if dc[direction] > dc_max[direction]:
                dc[direction] = dc_max[direction]
            elif dc[direction] < dc_min[direction]:
                dc[direction] = dc_min[direction]
        pi.set_PWM_dutycycle(pin, dc[direction])
        dc_rate[direction] = (dc[direction] - dc_mid) / dc_range
        time.sleep(0.01)
    return dc_rate