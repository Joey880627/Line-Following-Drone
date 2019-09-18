# -*- coding: UTF-8 -*-

import sys
from camera import Camera
import PID
import random
import time
import numpy as np
from distance import get_distance
# For calculate
import cv2
import config as g
import os
import functions as f
"""
 1. 參數設定, 設定變數及初始值
"""
# 重要
# length為x軸，horizontal shift
# height為y軸，vertical shift
# width為z軸，也就是飛行高度
# throttle: z, pitch: y, roll: x
directions = ['throttle', 'pitch', 'roll', 'yaw']


def pixel_to_cm(pixel, distance):
    # adjust feedbacks
    sensitivity = 5
    distance = 60 + (distance - 75) / sensitivity
    focalLength = 96 * g.exe_time / 30
    cm = pixel * (distance + 10) / focalLength
    return cm

def mission(image, mission_name, drone_height):
    g.set_pid(mission_name)
    feedbacks = dict({'throttle':0, 'pitch':0, 'roll':0, 'yaw':0})
    if (mission_name == 'takeoff') or (mission_name == 'takeoff_loiter'):
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image)
        feedbacks['yaw'], img2 = f.yaw(image)
    elif (mission_name == 'lf_1') or (mission_name == 'lf_2') or (mission_name == 'lf_3'):
        feedbacks['roll'], _, img1 = f.roll_pitch(image, use_dilate=True)
        if mission_name == 'lf_1':
            feedbacks['yaw'], img2 = f.yaw(image)
        else:
            feedbacks['yaw'], img2 = f.yaw(image, turn = True)
        if feedbacks['yaw'] < 1:
            feedbacks['pitch'] = -g.constant_pitch
        else:
            feedbacks['roll'] = 0
            feedbacks['pitch'] = 0
    elif mission_name == 'led_loiter':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, 'rgb')
        feedbacks['yaw'], img2 = f.yaw(image, rgb=True)
    elif mission_name == 'st_blue':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, 'blue')
        feedbacks['yaw'], img2 = f.yaw(image, rgb = True)
    elif flags['mission'] == 'st_red':
        feedbacks['roll'], _, img1 = f.roll_pitch(image, 'rgb')
        feedbacks['pitch'] = -g.constant_pitch
        feedbacks['yaw'], img2 = f.yaw(image, rgb = True)
    elif flags['mission'] == 'st_green':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, 'green')
        feedbacks['yaw'], img2 = f.yaw(image, rgb = True)
        
    elif mission_name == 'l_red':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, 'red')
        img2 = image
    elif mission_name == 'l_red_c':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, use_dilate=True)
        img2 = image
    elif mission_name == 'l_color':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, flags['color'])
        img2 = image
    elif mission_name == 'l_color_c':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, use_dilate=True)
        img2 = image
    elif mission_name == 'landing':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, use_dilate=True)
        img2 = image
        
    if mission_name == 'landing':
        feedbacks['throttle'] = drone_height - 0
    elif mission_name == 'l_red_c':
        feedbacks['throttle'] = drone_height - 100
    else:
        feedbacks['throttle'] = drone_height - g.constant_height
    feedbacks['pitch'] = pixel_to_cm(feedbacks['pitch'], drone_height)
    feedbacks['roll'] = pixel_to_cm(feedbacks['roll'], drone_height)
    try:
        img = cv2.addWeighted(img1,0.6,img2,0.6,0)
    except:
        pass
    return feedbacks, img

if __name__ == '__main__':
    
    drone_height = 0
    cam = Camera(0)
    apm_input = dict({'throttle':0, 'pitch':0, 'roll':0, 'yaw':0})
    # For mission manager
    mission_names = ['takeoff', 'takeoff_loiter', 'lf_1', 'led_loiter',
                     'lf_2', 'st_blue', 'st_red', 'st_green',
                     'lf_3', 'l_red', 'l_red_c', 'l_color', 'l_color_c', 'landing']
    try:
        initial_mission = sys.argv[1] if (sys.argv[1] in mission_names) else mission_names[1]
    except:
        initial_mission = mission_names[1]
    try:
        color_mission = sys.argv[2] if (sys.argv[2] == 'blue' or sys.argv[2] == 'green') else None
    except:
        color_mission = None
    flags = {'mission': 'takeoff', 'color': color_mission, 'loiter_stability': 0, 'lf_2_startTime': 0}
    loiter_stable_time = 30
    loiter_stability_range = dict({'throttle':30, 'pitch':g.exe_time, 'roll':g.exe_time, 'yaw':10})
    
    count = 0
    start = time.time()
    while True:
        '''
        if count == 0:
            startTime = time.time()
        elif count == 100:
            print((time.time() - startTime)/100)
            count = -1
        count += 1
        '''
        
        """
        1. Take photo
        """
        image = cam.read()
        # Color recognization
        image_color = f.image_color(image)
        # print('1.', time.time()-startTime)
        """
        2. Get feedbacks
        """
        drone_height = get_distance()
        # drone_height = 90
        feedbacks, image = mission(image, flags['mission'], drone_height)
        """
        3. Update PID
        """
        for direction in directions:
            g.pid[direction].update(feedbacks[direction])
            apm_input[direction] = g.pid[direction].output
        
        """
        4. Mission manager
        """
        if flags['mission'] == 'takeoff':
            if drone_height > 7:
                flags['mission'] = initial_mission
        elif flags['mission'] == 'takeoff_loiter':
            # Calculate loiter stability
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'lf_3'
                flags['loiter_stability'] = 0
        elif flags['mission'] == 'lf_1':
            if flags['color'] == None:
                if image_color == 'red' or image_color == 'green' or image_color == 'blue':
                    flags['mission'] = 'led_loiter'
        # Loiter on LED
        elif flags['mission'] == 'led_loiter':
            if image_color == 'green':
                flags['color'] = 'green'
            elif image_color == 'blue':
                flags['color'] = 'blue'
            # Calculate loiter stable
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'lf_2'
                flags['lf_2_startTime'] = time.time()
                flags['loiter_stability'] = 0
        elif flags['mission'] == 'lf_2':
            lf_2_time = time.time() - flags['lf_2_startTime']
            if (image_color == 'blue') and (lf_2_time >= 10):
                flags['mission'] = 'st_blue'
        elif flags['mission'] == 'st_blue':
            # Calculate loiter stable
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'st_red'
                if flags['color'] == 'blue':
                    f.sandbag(True)
                flags['loiter_stability'] = 0
        elif flags['mission'] == 'st_red':
            if image_color == 'green':
                flags['mission'] = 'st_green'
        elif flags['mission'] == 'st_green':
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'lf_3'
                if flags['color'] == 'green':
                    f.sandbag(True)
                else:
                    f.sandbag(False)
                flags['loiter_stability'] = 0
        elif flags['mission'] == 'lf_3':
            if image_color != 'green':
                f.sandbag(False)
            if image_color == 'red':
                flags['mission'] = 'l_red'
        elif flags['mission'] == 'l_red':
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'l_red_c'
                flags['loiter_stability'] = 0
        elif flags['mission'] == 'l_red_c':
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'l_color'
                flags['loiter_stability'] = 0
        elif flags['mission'] == 'l_color':
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'l_color_c'
                flags['loiter_stability'] = 0
        elif flags['mission'] == 'l_color_c':
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            if flags['loiter_stability'] > loiter_stable_time:
                flags['mission'] = 'landing'
                flags['loiter_stability'] = 0
            
        

        
        
        """
        5. Output to APM
        """
        
        
        dc = g.output_to_apm(apm_input)
        
        image = cv2.resize(image, (250, 250))
        texts = []
        texts.append('Mission: ' + flags['mission'])
        texts.append('Height: ' + str(int(drone_height)) + 'cm')
        texts.append('Throttle: '+ str(round(dc['throttle'], 3)))
        texts.append('Pitch: '+ str(round(dc['pitch'], 3)))
        texts.append('Roll: '+ str(round(dc['roll'], 3)))
        texts.append('Yaw: '+ str(round(dc['yaw'], 3)) + ' ' + str(round(feedbacks['yaw'], 3)))
        for i in range(len(texts)):
            cv2.putText(image, texts[i], (0, 25+25*i), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        # cv2.resizeWindow("live", 400, 300)
        cv2.imshow('live', image)
        key = cv2.waitKey(1)
        if key == 27:
            break
        '''
        print('Feedbacks\nThrottle: %.3f\tPitch: %.3f\tRoll: %.3f\tYaw: %3f' % (feedbacks['throttle'], feedbacks['pitch'], feedbacks['roll'], feedbacks['yaw']))
        print('APM input\nThrottle: %.3f\tPitch: %.3f\tRoll: %.3f\tYaw: %3f' % (dc['throttle'], dc['pitch'], dc['roll'], dc['yaw']))
        
        print('Flags\nmission: %s\tcolor: %s\tloiter stability: %d' % (flags['mission'], flags['color'], flags['loiter_stability']))
        
        # print('Time per loop:', time.time()-startTime)
        print('========================================================')'''
    f.sandbag(False)
    cam.stop()
