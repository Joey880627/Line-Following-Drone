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
    constant_pitch = 100
    if (mission_name == 'takeoff') or (mission_name == 'takeoff_loiter') or (mission_name == 'landing'):
        # feedbacks['roll'], feedbacks['pitch'], img = f.circle(image)
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image)
        feedbacks['yaw'], img2 = f.yaw(image)
    elif (mission_name == 'line_follow_1') | (mission_name == 'line_follow_2'):
        feedbacks['roll'], _, img1 = f.roll_pitch(image)
        feedbacks['pitch'] = -constant_pitch
        feedbacks['yaw'], img2 = f.yaw(image)
    elif mission_name == 'led_finding':
        
        feedbacks['roll'], _, img1 = f.roll_pitch(image)
        feedbacks['pitch'] = -constant_pitch
        feedbacks['yaw'], img2= f.yaw(image)
        _, feedback_pitch, _ = f.roll_pitch(image, 'rgb')
        if feedback_pitch != 0:
            feedbacks['pitch'] = feedback_pitch
            flags['mission'] = 'led_loiter'
    elif mission_name == 'led_loiter':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, 'rgb')
        feedbacks['yaw'], img2 = f.yaw(image)
    elif mission_name == 'sandbag_throwing':
        # 繼續直行
        feedbacks['roll'], pitch_feedback, img1 = f.roll_pitch(image, 'rgb')
        feedbacks['pitch'] = -constant_pitch
        feedbacks['yaw'], img2 = f.yaw(image, rgb = True)
  
        if image_color == 'green':
            if (abs(feedbacks['roll']) < 20) and (pitch_feedback > 100):
                    flags['mission'] = 'line_follow_3'
    elif mission_name == 'line_follow_3':
        feedbacks['roll'], _, img1 = f.roll_pitch(image)
        feedbacks['pitch'] = -constant_pitch
        feedbacks['yaw'], img2 = f.yaw(image)
        if f.find_landing_line(image):
            flags['mission'] = 'landing_line_found'
    elif mission_name == 'landing_line_found':
        feedbacks['roll'], feedbacks['pitch'], img1 = f.roll_pitch(image, 'red')
        feedbacks['pitch'] -= 200
        feedbacks['yaw'], img2 = f.yaw(image)
        
        if image_color == 'red':
            flags['mission'] = 'landing_color_red_found'
        
    elif mission_name == 'landing_color_red_found':
        feedbacks['roll'], feedbacks['pitch'], img = f.roll_pitch(image, 'red')
        roll_feedback, pitch_feedback, _ = f.circle(image)
        if roll_feedback != 0 or pitch_feedback != 0:
            flags['mission'] = 'landing_circle_red_found'
    elif mission_name == 'landing_circle_red_found':
        feedbacks['roll'], feedbacks['pitch'], img = f.circle(image)
        flags['color'] = 'blue'
        if flags['color'] == 'green':
            # Go left
            if image_color == 'green':
                flags['mission'] = 'landing_color_found'

        elif flags['color'] == 'blue':
             # Go right
             if image_color == 'blue':
                flags['mission'] = 'landing_color_found'
    elif mission_name == 'landing_color_found':
        feedbacks['roll'], feedbacks['pitch'], img = f.roll_pitch(image, 'rgb')
        roll_feedback, pitch_feedback, _ = f.circle(image)
        if roll_feedback != 0 or pitch_feedback != 0:
            flags['mission'] = 'landing_circle_found'
    elif mission_name == 'landing_circle_found':
        feedbacks['roll'], feedbacks['pitch'], img = f.circle(image)
        # After stabilize
        flags['mission'] = 'landing'
        
    if mission_name == 'landing':
        feedbacks['throttle'] = drone_height - 0
    elif mission_name == 'takeoff':
        feedbacks['throttle'] = drone_height - 75
    else:
        feedbacks['throttle'] = drone_height - 75
    feedbacks['pitch'] = pixel_to_cm(feedbacks['pitch'], drone_height)
    feedbacks['roll'] = pixel_to_cm(feedbacks['roll'], drone_height)
    try:
        img = cv2.addWeighted(img1,0.75,img2,0.75,0)
    except:
        pass
    return feedbacks, img

if __name__ == '__main__':
    
    drone_height = 0
    cam = Camera(0)
    apm_input = dict({'throttle':0, 'pitch':0, 'roll':0, 'yaw':0})
    # For mission manager
    mission_names = ['takeoff', 'takeoff_loiter', 'line_follow_1', 'led_finding', 'led_loiter',
                     'line_follow_2', 'sandbag_throwing', 'line_follow_3', 'landing_line_found',
                     'landing_color_red_found', 'landing_circle_red_found',
                     'landing_color_found', 'landing_circle_found', 'landing']
    try:
        initial_mission = sys.argv[1] if (sys.argv[1] in mission_names) else mission_names[0]
    except:
        initial_mission = mission_names[0]
    flags = {'mission': initial_mission, 'color': None, 'loiter_stability': 0, 'line_follow_2_startTime': 0}
    loiter_stable_time = 20
    loiter_stability_range = dict({'throttle':1, 'pitch':10, 'roll':10, 'yaw':1})
    
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
            if drone_height > 30:
                flags['mission'] = 'takeoff_loiter'
        elif flags['mission'] == 'takeoff_loiter':
            # Calculate loiter stability
            if (abs(feedbacks['throttle'])<loiter_stability_range['throttle'])&(abs(feedbacks['pitch'])<loiter_stability_range['pitch']):
                if (abs(feedbacks['roll'])<loiter_stability_range['roll'])&(abs(feedbacks['yaw'])<loiter_stability_range['yaw']):
                    flags['loiter_stability'] += 1
                else:
                    flags['loiter_stability'] = 0
            else:
                flags['loiter_stability'] = 0
            # if flags['loiter_stability'] > loiter_stable_time:
            #     flags['mission'] = 'line_follow_1'
                
        elif flags['mission'] == 'line_follow_1':
            if flags['color'] == None:
                if image_color == 'red':
                    flags['mission'] = 'led_finding'
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
                
                if image_color == flags['color']:
                    flags['loiter_stability'] = 0
                    flags['mission'] = 'line_follow_2'
                    flags['line_follow_2_startTime'] = time.time()
                flags['loiter_stability'] = 0
        elif flags['mission'] == 'line_follow_2':
            line_follow_2_time = time.time() - flags['line_follow_2_startTime']
            if (image_color == 'blue') and (line_follow_2_time >= 15):
                flags['mission'] = 'sandbag_throwing'
        elif flags['mission'] == 'sandbag_throwing':
            pass
        elif flags['mission'] == 'line_follow_3':
            pass
        
        # landing test
        if time.time()-start > 30:
            flags['mission'] = 'landing'
            
        

        
        
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
            cv2.putText(image, texts[i], (0, 25+25*i), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (0, 255, 0), 1, cv2.LINE_AA)
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
    cam.stop()
