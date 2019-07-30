# -*- coding: UTF-8 -*-

# For simulator
from vpython import *
from msvcrt import kbhit, getwch
import PID
import random
import time
from PIL import ImageGrab
import cv2
import numpy as np
import multiprocessing as mp
# For calculate
import config as g
import color_recognization as cr
import tracking as t

"""
 1. 參數設定, 設定變數及初始值
"""
# 重要
# length為x軸，horizontal shift
# height為y軸，vertical shift
# width為z軸，也就是飛行高度
# throttle: z, pitch: y, roll: x
directions = ['throttle', 'pitch', 'roll', 'yaw']

# Map initialize
class Map():
    def __init__(self):
        self.floor_length = 900
        self.floor_height = 875
        floor_width = 2 # 上下各1
        self.floor = box(pos = vector(self.floor_length/2, self.floor_height/2, -1), length = self.floor_length, height = self.floor_height, width = floor_width, color = vector(219/255, 112/255, 7/255))
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
        self.black_line_2 = box(pos = vector(750, 385, 0), length = 265, height = 2, width = 1, color = vector(0, 0, 0))
        # LED
        self.led = box(pos = vector(750, 295, 0), length = 90, height = 60, width = 0.9, color = color.red)
        # 沙包投擲區
        self.sandbad_area_blue= box(pos = vector(540, 720, 0), length = 90, height = 90, width = 0.9, color = color.blue)
        self.sandbad_area_red= box(pos = vector(450, 720, 0), length = 90, height = 90, width = 0.9, color = color.red)
        self.sandbad_area_green= box(pos = vector(360, 720, 0), length = 90, height = 90, width = 0.9, color = color.green)
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
        self.objects = [self.floor, self.take_off_spot, self.take_off_ring,
                        self.black_path_1, self.black_path_2, self.black_path_3, self.black_path_4, self.black_path_5,
                        self.black_line_1, self.black_line_2, self.led, self.sandbad_area_blue, self.sandbad_area_red, self.sandbad_area_green,
                        self.landing_line, self.landing_spot_blue, self.landing_spot_black_1, self.landing_spot_red, self.landing_spot_black_2, self.landing_spot_green, self.landing_spot_black_3,
                        self.wall_1, self.wall_2, self.wall_3, self.wall_4,
                        self.wall_5, self.wall_6, self.wall_7, self.wall_8]
                
# Drone initialize
class Drone():
    def __init__(self, pos = vector(750, 90, 0), scale = 20, dt = 0.001):
        self.center = box(pos = pos, length = scale, height = scale, width = 3, color = color.gray(0.2))
        self.arm_1 = box(pos = pos + vector(scale*1, scale*1, 1), length = scale*2, height = 5, width = 2, axis = vector(1, 1, 0), color = color.red)
        self.arm_2 = box(pos = pos + vector(-scale*1, scale*1, 1), length = scale*2, height = 5, width = 2, axis = vector(-1, 1, 0), color = color.red)
        self.arm_3 = box(pos = pos + vector(scale*1, -scale*1, 1), length = scale*2, height = 5, width = 2, axis = vector(1, -1, 0), color = color.white)
        self.arm_4 = box(pos = pos + vector(-scale*1, -scale*1, 1), length = scale*2, height = 5, width = 2, axis = vector(-1, -1, 0), color = color.white)
        self.propeller_1 = cylinder(pos=pos + vector(scale*1.6, scale*1.6, 3), radius=scale/2, length=1, axis=vector(0, 0, 1), color=color.white, opacity=0.7)
        self.propeller_2 = cylinder(pos=pos + vector(-scale*1.6, scale*1.6, 3), radius=scale/2, length=1, axis=vector(0, 0, 1), color=color.white, opacity=0.7)
        self.propeller_3 = cylinder(pos=pos + vector(scale*1.6, -scale*1.6, 3), radius=scale/2, length=1, axis=vector(0, 0, 1), color=color.white, opacity=0.7)
        self.propeller_4 = cylinder(pos=pos + vector(-scale*1.6, -scale*1.6, 3), radius=scale/2, length=1, axis=vector(0, 0, 1), color=color.white, opacity=0.7)
        self.objects = [self.center, self.arm_1, self.arm_2, self.arm_3, self.arm_4, self.propeller_1, self.propeller_2, self.propeller_3, self.propeller_4]
        self.pos = self.center.pos
        self.v = vector(0, 0, 0)
        self.av = 0
        self.v_max = vector(1000, 1000, 1000)
        self.av_max = 5
        self.propeller_speed = 0
        self.dt = dt
    def move(self, map):
        # 無人機根據速度移動
        for obj in self.objects:
            obj.pos.x += self.v.x * self.dt
            obj.pos.y += self.v.y * self.dt
            obj.pos.z += self.v.z * self.dt
        for obj in map.objects:
            obj.rotate(angle=self.av * self.dt, axis=vec(0,0,1), origin = self.pos)
        
scene_width = 600
scene_height = 450
def result(input_queue):
    # Scene setting
    dt = 0.001
    rate(1/dt)
    scene = canvas(title = "", width = scene_width, height = scene_height, x = 0, y = 0, center = vector(750, 90, 10), background = color.gray(0.5))
    scene.lights=[]
    distant_light(direction=vector( 5,  10,  10), color=color.gray(0.6))
    distant_light(direction=vector( -5,  10,  10), color=color.gray(0.6))
    map = Map()
    drone = Drone(pos = vector(750, 90, 100), dt = dt)
    
    # Button setting
    button_flags = {}
    button_flags['use_drone_camera'] = True
    button_flags['last_use_drone_camera'] = True
    if button_flags['use_drone_camera']:
        scene.camera.pos = vector(drone.pos.x, (drone.pos.y+5), drone.pos.z-15)
        scene.camera.axis = vector(0, 0, -(drone.pos.z-15))
    def perspective(b1):
        button_flags['use_drone_camera'] = not button_flags['use_drone_camera']
        if button_flags['use_drone_camera']:
            b1.text = 'Full scene'
        else:
            b1.text = 'Drone camera'
    b1 = button(text='Full scene', bind = perspective)
    
    # PID setting
    apm_Kp = {'throttle':1.2, 'pitch':1.2, 'roll':1.2, 'yaw':1.2}
    apm_Ki = {'throttle':3, 'pitch':3, 'roll':3, 'yaw':1}
    apm_Kd = {'throttle':0.001, 'pitch':0.001, 'roll':0.001, 'yaw':0.001}
    apm_max_gain = {'throttle':100, 'pitch':100, 'roll':100, 'yaw':10}
    apm_setpoint = {'throttle':0, 'pitch':0, 'roll':0, 'yaw':0}
    apm_pid = {}
    for direction in directions:
        apm_pid[direction] = PID.PID(apm_Kp[direction], apm_Ki[direction], apm_Kd[direction], maxGain=apm_max_gain[direction])
        apm_input = dict({'throttle':0, 'pitch':0, 'roll':0, 'yaw':0, 'led_loiter_stability': False})
    '''
    if random.choice([True, False]):
        mission_color = color.green
    else:
        mission_color = color.blue'''
    mission_color = color.green # Blue has problem
    while True:
        startTime = time.time()
        """
        1. Camera control
        """
        # Drone camera
        if button_flags['use_drone_camera'] != button_flags['last_use_drone_camera']:
            switch = True
        else:
            switch = False
        if button_flags['use_drone_camera']:
            scene.camera.pos = vector(drone.pos.x, drone.pos.y, drone.pos.z-15)
            scene.camera.axis = vector(0, 0, -(drone.pos.z-15))
        else:
            # Map camera
            if switch:
                scene.center = vector(map.floor_length/2, map.floor_height/2, 1000)
                scene.camera.axis = vector(0, 0, -1300)
        button_flags['last_use_drone_camera'] = button_flags['use_drone_camera']
        """
        2. APM pid
        """
        # APM input
        if not input_queue.empty():
            apm_input = input_queue.get()
        if apm_input['led_loiter_stability']:
            map.led.color = mission_color
        # print(message)
        # Setpoint
        for direction in directions:
            apm_pid[direction].SetPoint = apm_input[direction]
        # print('Throttle: %.2f, Pitch: %.2f, Roll: %.2f, Yaw: %2f' % (apm_input['throttle'], apm_input['pitch'], apm_input['roll'], apm_input['yaw']))
        # APM pid Update
        apm_pid['throttle'].update(drone.v.z)
        apm_pid['pitch'].update(drone.v.y)
        apm_pid['roll'].update(drone.v.x)
        apm_pid['yaw'].update(drone.av)
        apm_output_throttle, apm_output_pitch, apm_output_roll, apm_output_yaw = apm_pid['throttle'].output, apm_pid['pitch'].output, apm_pid['roll'].output, apm_pid['yaw'].output
        # print('Throttle: %.2f, Pitch: %.2f, Roll: %.2f, Yaw: %2f' % (apm_output_throttle, apm_output_pitch, apm_output_roll, apm_output_yaw))
        drone.av += apm_output_yaw
        if drone.av > drone.av_max:
            drone.av = drone.av_max
        elif drone.av < -drone.av_max:
            drone.av = -drone.av_max
        drone.v += vector(apm_output_roll, apm_output_pitch, apm_output_throttle)
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
        
        """
        3. Calculate result
        """
        # Add noises
        # noise_z = random.uniform(-10, -20)
        noise_y = random.uniform(10, 40)
        noise_x = random.uniform(10, 40)
        noise_av = random.uniform(0, 0.3)
        # drone.v.z += noise_z
        drone.v.y += noise_y
        drone.v.x += noise_x
        drone.av += noise_av
        if button_flags['use_drone_camera']:
            drone.move(map)
        # print('Time per loop:', time.time()-startTime)

def mission(image, mission_name):
    mission_names = ['takeoff_loiter', 'line_follow_1', 'led_loiter',
                     'line_follow_2', 'sandbag_throwing', 'line_follow_3',
                     'landing']
    feedbacks = dict({'throttle':0, 'pitch':0, 'roll':0, 'yaw':0})
    def loiter_pid():
        g.pid['roll'].setKp(1)
        g.pid['roll'].setKi(1)
        g.pid['roll'].setKd(0.1)
        g.pid['pitch'].setKp(1)
        g.pid['pitch'].setKi(1)
        g.pid['pitch'].setKd(0.1)
        g.pid['yaw'].setKp(3)
        g.pid['yaw'].setKi(1)
        g.pid['yaw'].setKd(0.1)
    def line_follow_pid():
        g.pid['roll'].setKp(0.3)
        g.pid['roll'].setKi(1)
        g.pid['roll'].setKd(0.01)
        g.pid['pitch'].setKp(1)
        g.pid['pitch'].setKi(0)
        g.pid['pitch'].setKd(0)
        g.pid['yaw'].setKp(3)
        g.pid['yaw'].setKi(1)
        g.pid['yaw'].setKd(0.1)
    if mission_name == 'takeoff_loiter':
        loiter_pid()
        feedbacks['roll'], feedbacks['pitch'] = ct.get_shift(image)
        feedbacks['yaw'] = lt.get_slope(image)
    elif (mission_name == 'line_follow_1') | (mission_name == 'line_follow_2') | (mission_name == 'line_follow_3'):
        line_follow_pid()
        feedbacks['roll'] = lt.get_shift(image)
        feedbacks['pitch'] = -30
        feedbacks['yaw'] = lt.get_slope(image)
    elif mission_name == 'led_loiter':
        loiter_pid()
        if flags['color']:
            color = flags['color']
        else:
            color = 'red'
        horizontal_shift, vertical_shift = rt.get_shift(image)
        print(horizontal_shift, vertical_shift)
        if (horizontal_shift!=0) & (vertical_shift!=0):
            flags['loiter_point_found'] = True
        # If led hasn't found, keep following line
        if flags['loiter_point_found']:
            feedbacks['roll'], feedbacks['pitch'] = horizontal_shift, vertical_shift
            feedbacks['yaw'] = lt.get_slope(image)
        
        else:
            line_follow_pid()
            feedbacks['roll'] = lt.get_shift(image)
            feedbacks['pitch'] = -30
            feedbacks['yaw'] = lt.get_slope(image)
    elif mission_name == 'sandbag_throwing':
        # 繼續直行
        line_follow_pid()
        feedbacks['pitch'] = -30
        feedbacks['yaw'] = rt.get_slope(image)
        if image_color:
            # print('Image color', image_color)
            horizontal_shift, vertical_shift = rt.get_shift(image)
            feedbacks['roll'] = horizontal_shift
            
            if image_color == 'green':
                if (abs(horizontal_shift)<20) & (vertical_shift>50):
                    flags['mission'] = 'line_follow_3'
        
    return feedbacks

def motion_blur(image, degree=12, angle=90):
    image = np.array(image)
    M = cv2.getRotationMatrix2D((degree / 2, degree / 2), angle, 1)
    motion_blur_kernel = np.diag(np.ones(degree))
    motion_blur_kernel = cv2.warpAffine(motion_blur_kernel, M, (degree, degree))
 
    motion_blur_kernel = motion_blur_kernel / degree
    blurred = cv2.filter2D(image, -1, motion_blur_kernel)
 
    # convert to uint8
    cv2.normalize(blurred, blurred, 0, 255, cv2.NORM_MINMAX)
    blurred = np.array(blurred, dtype=np.uint8)
    return blurred

if __name__ == '__main__':
    # Multiprocessing
    manager = mp.Manager()
    apm_input = manager.dict({'throttle':0, 'pitch':0, 'roll':0, 'yaw':0, 'led_loiter_stability': False}) # led_loiter_stability will not be used in reality
    output_queue = manager.Queue()
    mp_result = mp.Process(target=result, args=(output_queue,))
    mp_result.start()
    # For calculating feedbacks
    lt = t.Line_tracking()
    ct = t.Circle_tracking()
    rt = t.Rectangle_tracking()
    # For mission manager
    flags = {'mission': 'line_follow_2', 'color': None, 'loiter_point_found': False, 'loiter_stability': 0}
    loiter_stable_time = 10
    loiter_stability_range = dict({'throttle':20, 'pitch':20, 'roll':20, 'yaw':5})
    time.sleep(3)
    while True:
        startTime = time.time()

        """
        1. Take photo
        """
        image = ImageGrab.grab((10, 138, int(scene_width*1.25)+10, int(scene_height*1.25)+138))
        image = np.array(image)[:,:,::-1]
        image = cv2.resize(image, g.image_shape)
        image = motion_blur(image)
        # Show image
        # cv2.imshow('live', image)
        # cv2.waitKey(1)

        """
        2. Get feedbacks
        """
        feedbacks = mission(image, flags['mission'])
        print('Feedbacks\nThrottle: %.2f, Pitch: %.2f, Roll: %.2f, Yaw: %2f' % (feedbacks['throttle'], feedbacks['pitch'], feedbacks['roll'], feedbacks['yaw']))

        """
        3. Update PID
        """
        for direction in directions:
            g.pid[direction].update(feedbacks[direction])
            apm_input[direction] = g.pid[direction].output

        """
        4. Mission manager
        """
        # Color recognization
        image_color = cr.image_color(image)

        if flags['mission'] == 'takeoff_loiter':
            # Calculate loiter stability
            if (abs(apm_input['throttle'])<loiter_stability_range['throttle'])&(abs(apm_input['pitch'])<loiter_stability_range['pitch']):
                if (abs(apm_input['roll'])<loiter_stability_range['roll'])&(abs(apm_input['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'line_follow_1'
                
        elif flags['mission'] == 'line_follow_1':
            if flags['color'] == None:
                if image_color == 'red':
                    flags['mission'] = 'led_loiter'
        # Loiter on LED
        elif flags['mission'] == 'led_loiter':
            if image_color == 'green':
                flags['color'] = 'green'
            elif image_color == 'blue':
                flags['color'] = 'blue'
            # Calculate loiter stable
            if (abs(apm_input['throttle'])<loiter_stability_range['throttle'])&(abs(apm_input['pitch'])<loiter_stability_range['pitch']):
                if (abs(apm_input['roll'])<loiter_stability_range['roll'])&(abs(apm_input['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['loiter_stability'] = 0
                apm_input['led_loiter_stability'] = True # will not be used in reality
                
                if image_color == flags['color']:
                    flags['loiter_stability'] = 0
                    flags['loiter_point_found'] = False
                    flags['mission'] = 'line_follow_2'
        # elif flags['mission'] == 'line_follow_2':
            # if image_color == 'blue':
            #     flags['mission'] = 'sandbag_throwing'
        elif flags['mission'] == 'sandbag_throwing':
            pass
        elif flags['mission'] == 'line_follow_3':
            if image_color == 'red':
                flags['mission'] = 'landing'
            
        
        print('Flags\nmission: %s, color: %s, loiter point found: %s, loiter stability: %d' % (flags['mission'], flags['color'], flags['loiter_point_found'], flags['loiter_stability']))

        """
        5. Keyboard input if needed
        """
        sensitivity = 500
        if kbhit():
            key = getwch()
            if key == 'w':
                apm_input['throttle'] = -sensitivity
            elif key == 's':
                apm_input['throttle'] = sensitivity
            elif key == 'i':
                apm_input['pitch'] = sensitivity
            elif key == 'k':
                apm_input['pitch'] = -sensitivity
            elif key == 'l':
                apm_input['roll'] = sensitivity
            elif key == 'j':
                apm_input['roll'] = -sensitivity
            elif key == 'd':
                apm_input['yaw'] = 2
            elif key == 'a':
                apm_input['yaw'] = -2
        
        """
        6. Output to APM
        """
        print('APM input\nThrottle: %.2f, Pitch: %.2f, Roll: %.2f, Yaw: %2f' % (apm_input['throttle'], apm_input['pitch'], apm_input['roll'], apm_input['yaw']))
        output_queue.put(apm_input)
        print('Time per loop:', time.time()-startTime)
        print()
