# -*- coding: utf-8 -*-
import numpy as np
import PID
import pigpio
import time
import csv

exe_time = 30 # Bigger, slower
image_shape = (4 * exe_time, 4 * exe_time)
vision = 1
rate = 0.5 # 控制 image transform
constant_height = 75
constant_pitch = 25

with open('hsv.csv', 'r', encoding='utf-8') as f:
    rows = csv.reader(f)
    hsv = []
    for row in rows:
        hsv.append(row)
# image color range
lower = {}
upper = {}
for row in hsv:
    lower[row[0]] = np.array(row[1:4]).astype(int)
    upper[row[0]] = np.array(row[4:7]).astype(int)


pins = {'throttle':17, 'pitch':17, 'roll':17, 'yaw':17}
directions = ['throttle', 'pitch', 'roll', 'yaw']
# altitude hold 588 ~ 600
dc_min = {'throttle':506, 'pitch':631, 'roll':756, 'yaw':881}
dc_max = {'throttle':612, 'pitch':745, 'roll':870, 'yaw':995}
safety_min = {'throttle':544, 'pitch':660, 'roll':785, 'yaw':881}
safety_max = {'throttle':583, 'pitch':716, 'roll':841, 'yaw':995}
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
        pid['throttle'].setKd(0.014)
    def landing_pid():
        pid['throttle'].setKp(0.015)
        pid['throttle'].setKi(0.005)
        pid['throttle'].setKd(0.014)
    
    def loiter_pid(strength = 1):
        pid['roll'].setKp(0.007*strength)
        pid['roll'].setKi(0.00)
        pid['roll'].setKd(0.009*strength)
        pid['pitch'].setKp(0.007*strength)
        pid['pitch'].setKi(0.00)
        pid['pitch'].setKd(0.009*strength)
        
        pid['yaw'].setKp(0.15)
        pid['yaw'].setKi(0)
        pid['yaw'].setKd(0.00)
    def lf_pid(strength = 1):
        pid['roll'].setKp(0.007*strength)
        pid['roll'].setKi(0.00)
        pid['roll'].setKd(0.009*strength)
        pid['pitch'].setKp(0.007)
        pid['pitch'].setKi(0)
        pid['pitch'].setKd(0)
        
        pid['yaw'].setKp(0.25)
        pid['yaw'].setKi(0)
        pid['yaw'].setKd(0.00)
    def zero_pid():
        pid['roll'].setKp(0)
        pid['roll'].setKi(0)
        pid['roll'].setKd(0)
        pid['pitch'].setKp(0)
        pid['pitch'].setKi(0)
        pid['pitch'].setKd(0)
        
        pid['yaw'].setKp(0)
        pid['yaw'].setKi(0)
        pid['yaw'].setKd(0)
    if mission_name == 'takeoff':
        take_off_pid()
        # loiter_pid()
    if mission_name == 'takeoff_loiter':
        altitude_hold_pid()
        loiter_pid()
    elif mission_name == 'lf_1':
        altitude_hold_pid()
        lf_pid(0.6)
    elif mission_name == 'led_loiter':
        altitude_hold_pid()
        loiter_pid(0.6)
    elif mission_name == 'lf_2':
        altitude_hold_pid()
        lf_pid(0.6)
    elif mission_name == 'lf_3':
        altitude_hold_pid()
        lf_pid(0.9)
    elif mission_name == 'st_blue':
        altitude_hold_pid()
        loiter_pid(0.6)
    elif mission_name == 'st_red':
        altitude_hold_pid()
        lf_pid(0.6)
    elif mission_name == 'st_green':
        altitude_hold_pid()
        loiter_pid(0.6)
    elif mission_name == 'lf_4':
        altitude_hold_pid()
        lf_pid(0.9)
    elif mission_name == 'l_red':
        altitude_hold_pid()
        loiter_pid(0.6)
    elif mission_name == 'l_color':
        altitude_hold_pid()
        loiter_pid(0.6)
    elif mission_name == 'l_start':
        landing_pid()
        loiter_pid(0.85)
    elif mission_name == 'l_end':
        landing_pid()
        zero_pid()

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

