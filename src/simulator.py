# -*- coding: UTF-8 -*-
from vpython import *
from msvcrt import kbhit, getwch
import PID
import random
import time
from PIL import ImageGrab
import cv2
import numpy as np
"""
 1. 參數設定, 設定變數及初始值
"""
# 重要
# length為x軸，horizontal shift
# height為y軸，vertical shift
# width為z軸，也就是飛行高度

scene = canvas(title = "", width = 800, height = 600, x = 0, y = 0, center = vector(750, 90, 10), background = color.gray(0.5))
scene.caption = 'Press c in command line to change the perspective'
# Map initialize
class Map():
    def __init__(self):
        self.floor_length = 900
        self.floor_height = 875
        floor_width = 2 # 上下各1
        self.floor = box(pos = vector(self.floor_length/2, self.floor_height/2, -1), length = self.floor_length, height = self.floor_height, width = floor_width, color = vector(230/255, 150/255, 40/255))
        # 黑色標記
        # 起始mark
        self.take_off_spot = cylinder(pos = vector(750, 90, 0), radius = 10, length = 1, axis = vector(0, 0, 1), color = vector(0, 0, 0))
        self.take_off_ring = ring(pos = vector(750, 90, 0), radius = 60, thickness = 1, length = 2, axis = vector(0, 0, 1), color = vector(0, 0, 0))
        # Black path
        self.black_path_1 = box(pos = vector(750, 405, 0), length = 2, height = 630, width = 1, color = vector(0, 0, 0))
        self.black_path_2 = box(pos = vector(667.5, 720, 0), length = 165, height = 2, width = 1, color = vector(0, 0, 0))
        self.black_path_3 = box(pos = vector(232.5, 720, 0), length = 165, height = 2, width = 1, color = vector(0, 0, 0))
        self.black_path_4 = box(pos = vector(150, 510, 0), length = 2, height = 422, width = 1, color = vector(0, 0, 0))
        self.black_path_5 = box(pos = vector(218.25, 300, 0), length = 137.5, height = 2, width = 1, color = vector(0, 0, 0))
        # Black line
        self.black_line_1 = box(pos = vector(750, 205, 0), length = 265, height = 2, width = 1, color = vector(0, 0, 0))
        self.black_line_1 = box(pos = vector(750, 385, 0), length = 265, height = 2, width = 1, color = vector(0, 0, 0))
        # LED
        self.led = box(pos = vector(750, 295, 0), length = 90, height = 60, width = 0.9, color = color.red)
        # 沙包投擲區
        self.sandbad_area_blue= box(pos = vector(540, 720, 0), length = 90, height = 90, width = 0.9, color = color.blue)
        self.sandbad_area_red= box(pos = vector(450, 720, 0), length = 90, height = 90, width = 0.9, color = color.red)
        self.sandbad_area_blue= box(pos = vector(360, 720, 0), length = 90, height = 90, width = 0.9, color = color.green)
        # 降落區
        self.landing_line = box(pos = vector(287.5, 300, 0), length = 2, height = 250, width = 1, color = vector(0.3, 0, 0.1))
        self.landing_spot_blue = cylinder(pos = vector(450, 115, 0), radius = 90, length = 0.9, axis = vector(0, 0, 1), color = color.blue)
        self.landing_spot_black_1 = cylinder(pos = vector(450, 115, 0), radius = 10, length = 1, axis = vector(0, 0, 1), color = vector(0, 0, 0))
        self.landing_spot_red = cylinder(pos = vector(450, 295, 0), radius = 90, length = 0.9, axis = vector(0, 0, 1), color = color.red)
        self.landing_spot_black_2 = cylinder(pos = vector(450, 295, 0), radius = 10, length = 1, axis = vector(0, 0, 1), color = vector(0, 0, 0))
        self.landing_spot_green = cylinder(pos = vector(450, 475, 0), radius = 90, length = 0.9, axis = vector(0, 0, 1), color = color.green)
        self.landing_spot_black_3 = cylinder(pos = vector(450, 475, 0), radius = 10, length = 1, axis = vector(0, 0, 1), color = vector(0, 0, 0))
        # Wall
        wall_color = vector(0.8, 0.8, 0.8)
        self.wall_1 = box(pos = vector(888.5, 437.5, 100), length = 25, height = 875, width = 200, color = wall_color)
        self.wall_2 = box(pos = vector(450, 862.5, 100), length = 900, height = 25, width = 200, color = wall_color)
        self.wall_3 = box(pos = vector(12.5, 437.5, 100), length = 25, height = 875, width = 200, color = wall_color)
        self.wall_4 = box(pos = vector(312.5, 12.5, 100), length = 625, height = 25, width = 200, color = wall_color)
        self.wall_5 = box(pos = vector(612.5, 300, 100), length = 25, height = 580, width = 200, color = wall_color)
        self.wall_6 = box(pos = vector(450, 577.5, 100), length = 350, height = 25, width = 200, color = wall_color)
        self.wall_7 = box(pos = vector(287.5, 100, 100), length = 25, height = 150, width = 200, color = wall_color)
        self.wall_8 = box(pos = vector(287.5, 500, 100), length = 25, height = 150, width = 200, color = wall_color)
# Drone initialize
class Drone():
    def __init__(self, pos = vector(750, 90, 0), scale = 20):
        self.center = box(pos = pos, length = scale, height = scale, width = 3, color = color.gray(0.2))
        self.arm_1 = box(pos = pos + vector(scale*1, scale*1, 1), length = scale*2, height = 5, width = 2, axis = vector(1, 1, 0), color = color.red)
        self.arm_2 = box(pos = pos + vector(-scale*1, scale*1, 1), length = scale*2, height = 5, width = 2, axis = vector(-1, 1, 0), color = color.red)
        self.arm_3 = box(pos = pos + vector(scale*1, -scale*1, 1), length = scale*2, height = 5, width = 2, axis = vector(1, -1, 0), color = color.white)
        self.arm_4 = box(pos = pos + vector(-scale*1, -scale*1, 1), length = scale*2, height = 5, width = 2, axis = vector(-1, -1, 0), color = color.white)
        '''
        self.propeller_1 = box(pos=pos + vector(scale*1.6, scale*1.6, 5), length = scale, height = 3, width = 1, color=color.white, opacity=0.7)
        self.propeller_2 = box(pos=pos + vector(-scale*1.6, scale*1.6, 5), length = scale, height = 3, width = 1, color=color.white, opacity=0.7)
        self.propeller_3 = box(pos=pos + vector(scale*1.6, -scale*1.6, 5), length = scale, height = 3, width = 1, color=color.white, opacity=0.7)
        self.propeller_4 = box(pos=pos + vector(-scale*1.6, -scale*1.6, 5), length = scale, height = 3, width = 1, color=color.white, opacity=0.7)
        '''
        self.propeller_1 = cylinder(pos=pos + vector(scale*1.6, scale*1.6, 3), radius=scale/2, length=1, axis=vector(0, 0, 1), color=color.white, opacity=0.7)
        self.propeller_2 = cylinder(pos=pos + vector(-scale*1.6, scale*1.6, 3), radius=scale/2, length=1, axis=vector(0, 0, 1), color=color.white, opacity=0.7)
        self.propeller_3 = cylinder(pos=pos + vector(scale*1.6, -scale*1.6, 3), radius=scale/2, length=1, axis=vector(0, 0, 1), color=color.white, opacity=0.7)
        self.propeller_4 = cylinder(pos=pos + vector(-scale*1.6, -scale*1.6, 3), radius=scale/2, length=1, axis=vector(0, 0, 1), color=color.white, opacity=0.7)
        
        self.pos = self.center.pos
        self.v = vector(0, 0, 0)
        self.av = 0
        self.v_max = vector(50, 50, 50)
        self.av_max = 10
        self.propeller_speed = 0
    def move(self):
        # 無人機根據速度移動
        self.center.pos.x += self.v.x * dt
        self.center.pos.y += self.v.y * dt
        self.center.pos.z += self.v.z * dt
        self.center.rotate(angle=self.av * dt, axis=vec(0,0,1))
        self.pos = self.center.pos
        
        self.arm_1.pos.x += self.v.x * dt
        self.arm_1.pos.y += self.v.y * dt
        self.arm_1.pos.z += self.v.z * dt
        self.arm_1.rotate(angle=self.av * dt, axis=vec(0,0,1), origin = self.pos)
        self.arm_2.pos.x += self.v.x * dt
        self.arm_2.pos.y += self.v.y * dt
        self.arm_2.pos.z += self.v.z * dt
        self.arm_2.rotate(angle=self.av * dt, axis=vec(0,0,1), origin = self.pos)
        self.arm_3.pos.x += self.v.x * dt
        self.arm_3.pos.y += self.v.y * dt
        self.arm_3.pos.z += self.v.z * dt
        self.arm_3.rotate(angle=self.av * dt, axis=vec(0,0,1), origin = self.pos)
        self.arm_4.pos.x += self.v.x * dt
        self.arm_4.pos.y += self.v.y * dt
        self.arm_4.pos.z += self.v.z * dt
        self.arm_4.rotate(angle=self.av * dt, axis=vec(0,0,1), origin = self.pos)
        
        self.propeller_1.pos.x += self.v.x * dt
        self.propeller_1.pos.y += self.v.y * dt
        self.propeller_1.pos.z += self.v.z * dt
        self.propeller_1.rotate(angle=self.av * dt, axis=vec(0,0,1), origin = self.pos)
        # self.propeller_1.rotate(angle=self.propeller_speed * dt, axis=vec(0,0,1))
        
        self.propeller_2.pos.x += self.v.x * dt
        self.propeller_2.pos.y += self.v.y * dt
        self.propeller_2.pos.z += self.v.z * dt
        self.propeller_2.rotate(angle=self.av * dt, axis=vec(0,0,1), origin = self.pos)
        # self.propeller_2.rotate(angle=self.propeller_speed * dt, axis=vec(0,0,1))
        
        self.propeller_3.pos.x += self.v.x * dt
        self.propeller_3.pos.y += self.v.y * dt
        self.propeller_3.pos.z += self.v.z * dt
        self.propeller_3.rotate(angle=self.av * dt, axis=vec(0,0,1), origin = self.pos)
        # self.propeller_3.rotate(angle=self.propeller_speed * dt, axis=vec(0,0,1))
        
        self.propeller_4.pos.x += self.v.x * dt
        self.propeller_4.pos.y += self.v.y * dt
        self.propeller_4.pos.z += self.v.z * dt
        self.propeller_4.rotate(angle=self.av * dt, axis=vec(0,0,1), origin = self.pos)
        # self.propeller_4.rotate(angle=self.propeller_speed * dt, axis=vec(0,0,1))
        
# PID setting
map = Map()
drone = Drone()
directions = ['throttle', 'pitch', 'roll', 'yaw']
Kp = {'throttle':1.2, 'pitch':1.2, 'roll':1.2, 'yaw':1.2}
Ki = {'throttle':1, 'pitch':3, 'roll':3, 'yaw':1}
Kd = {'throttle':0.001, 'pitch':0.001, 'roll':0.001, 'yaw':0.001}
max_gain = {'throttle':100, 'pitch':100, 'roll':100, 'yaw':100}
setpoint = {'throttle':100, 'pitch':0, 'roll':0, 'yaw':0}
input = {'throttle':0, 'pitch':0, 'roll':0, 'yaw':0}
max_input = {'throttle':drone.v_max.z, 'pitch':drone.v_max.y, 'roll':drone.v_max.x, 'yaw':drone.av_max}
# throttle control
throttle_pid = PID.PID(Kp['throttle'], Ki['throttle'], Kd['throttle'], maxGain=max_gain['throttle'])
# pitch control
pitch_pid = PID.PID(Kp['pitch'], Ki['pitch'], Kd['pitch'], maxGain=max_gain['pitch'])
# roll control
roll_pid = PID.PID(Kp['roll'], Ki['roll'], Kd['roll'], maxGain=max_gain['roll'])
# yaw control
yaw_pid = PID.PID(Kp['yaw'], Ki['yaw'], Kd['yaw'], maxGain=max_gain['yaw'])
pid = {'throttle':throttle_pid, 'pitch':pitch_pid, 'roll':roll_pid, 'yaw':yaw_pid}
for direction in list(setpoint):
    pid[direction].SetPoint=setpoint[direction]

use_drone_camera = True
last_use_drone_camera = use_drone_camera
dt = 0.01
sensitivity = 100

if __name__ == '__main__':
    # rate(1/dt)
    for i in range(1000000):
        t = time.time()
        # Loiter
        # Start point vector(750, 90, 0)
        pid['throttle'].update(drone.pos.z)
        pid['pitch'].update(drone.pos.y - 90)
        pid['roll'].update(drone.pos.x - 750)
        pid['yaw'].update(0)
        output_throttle, output_pitch, output_roll, output_yaw = pid['throttle'].output, pid['pitch'].output, pid['roll'].output, pid['yaw'].output
        # print('Throttle: %.2f, Pitch: %.2f, Roll: %.2f, Yaw: %2f' % (output_throttle, output_pitch, output_roll, output_yaw))
        # Output
        drone.v += rotate(vector(output_roll, output_pitch, output_throttle), drone.av * dt)
        if drone.v.x > drone.v_max.x:
            drone.v.x = drone.v_max.x
        elif drone.v.x < -drone.v_max.x:
            drone.v.x = -drone.v_max.x
        if drone.v.y > drone.v_max.y:
            drone.v.y = drone.v_max.y
        elif drone.v.y < -drone.v_max.y:
            drone.v.y = -drone.v_max.y
        if drone.v.z > drone.v_max.z:
            drone.v.z = drone.v_max.z
        elif drone.v.z < -drone.v_max.z:
            drone.v.z = -drone.v_max.z
        if drone.av > drone.av_max:
            drone.av = drone.av_max
        elif drone.av < -drone.av_max:
            drone.av = -drone.av_max
        
        # Add noises
        noise_z = random.uniform(0,50)
        noise_y = random.uniform(0,50)
        noise_x = random.uniform(0,50)
        drone.v.z += noise_z
        drone.v.y += noise_y
        drone.v.x += noise_x
        
        drone.move()

        
        # Camera control
        if kbhit():
            key = getwch()
            if (key == 'c') | (key == 'C'):
                use_drone_camera = not use_drone_camera
        # Drone camera
        if use_drone_camera != last_use_drone_camera:
            switch = True
        else:
            switch = False
        if use_drone_camera:
            scene.camera.pos = vector(drone.pos.x, drone.pos.y, drone.pos.z-10)
            scene.camera.axis = vector(0, 0, -(drone.pos.z-10))
        else:
            # Map camera
            if switch:
                scene.center = vector(map.floor_length/2, map.floor_height/2, 1300)
                scene.camera.axis = vector(0, 0, -1300)
        last_use_drone_camera = use_drone_camera

        # image = ImageGrab.grab((0,0,800,600))  # screen box from (0,0)-(800,600)
        image = ImageGrab.grab((10,130,1010,880))
        # image = np.array(image.getdata()).reshape(image.size[0], image.size[1], 3)
        print(np.array(image).shape)
        # time.sleep(0.01)
        print(time.time()-t)





    '''
        # Keyboard input
        if kbhit():
            key = getwch()
            if key == 'w':
                input['throttle'] += sensitivity
            elif key == 's':
                input['throttle'] -= sensitivity
            elif key == 'i':
                input['pitch'] += sensitivity
            elif key == 'k':
                input['pitch'] -= sensitivity
            elif key == 'l':
                input['roll'] += sensitivity
            elif key == 'j':
                input['roll'] -= sensitivity
            elif key == 'd':
                input['yaw'] += sensitivity
            elif key == 'a':
                input['yaw'] -= sensitivity
        # Setpoint
        for direction in directions:
            if input[direction] > max_input[direction]:
                input[direction] = max_input[direction]
            elif input[direction] < -max_input[direction]:
                input[direction] = -max_input[direction]
            pid[direction].SetPoint = input[direction]
        # print('Throttle: %.2f, Pitch: %.2f, Roll: %.2f, Yaw: %2f' % (input['throttle'], input['pitch'], input['roll'], input['yaw']))
        # Update
        pid['throttle'].update(drone.v.z)
        pid['pitch'].update(drone.v.y)
        pid['roll'].update(drone.v.x)
        pid['yaw'].update(0)
        output_throttle, output_pitch, output_roll, output_yaw = pid['throttle'].output, pid['pitch'].output, pid['roll'].output, pid['yaw'].output
        # print('Throttle: %.2f, Pitch: %.2f, Roll: %.2f, Yaw: %2f' % (output_throttle, output_pitch, output_roll, output_yaw))
        # Output
        drone.v.y += output_pitch
        drone.v.x += output_roll'''
