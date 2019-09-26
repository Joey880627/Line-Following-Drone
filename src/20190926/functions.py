# -*- coding: utf-8 -*

import numpy as np
import time
import PID
import cv2
import config as g
from camera import Camera
import math
import warnings
warnings.filterwarnings('ignore')

import RPi.GPIO as GPIO
# Next we setup the pins for use!
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(20,GPIO.OUT)
GPIO.setup(21,GPIO.OUT)
PWM_FREQ = 50

pwm1 = GPIO.PWM(20, PWM_FREQ)
pwm2 = GPIO.PWM(21, PWM_FREQ)

pwm1.start(0)
pwm2.start(0)

def image_color(image):
    colors = ['blue', 'green', 'red', None]
    color_index = -1
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = {}
    for color in ['blue', 'green', 'red']:
        if g.lower[color][0] <= g.upper[color][0]:
            mask[color] = cv2.inRange(hsv, g.lower[color], g.upper[color])
        else:
            mask_1 = cv2.inRange(hsv, np.array([0, g.lower[color][1], g.lower[color][2]]), g.upper[color])
            mask_2 = cv2.inRange(hsv, g.lower[color], np.array([180, g.upper[color][1], g.upper[color][2]]))
            mask[color] = cv2.bitwise_or(mask_1, mask_2)

    color_sum = [(mask['blue']!=0).sum(),
                 (mask['green']!=0).sum(),
                 (mask['red']!=0).sum()]
    max_color_index = np.argmax(color_sum)
    color_area = image.shape[0]*image.shape[1] // 9
    if color_sum[max_color_index] >= color_area:
        color_index = max_color_index
    return colors[color_index]

def transform(image, color = 'black'):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if color == 'black':
        # image = erode(image)
        # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # mask_black = cv2.inRange(hsv, g.lower['black'], g.upper['black'])
        # mask = mask_black
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        mask = ((gray_image<=g.rate*np.average(gray_image))*255).astype(np.uint8)
    elif color == 'rgb':
        mask = {}
        for color in ['blue', 'green', 'red']:
            if g.lower[color][0] <= g.upper[color][0]:
                mask[color] = cv2.inRange(hsv, g.lower[color], g.upper[color])
            else:
                mask_1 = cv2.inRange(hsv, np.array([0, g.lower[color][1], g.lower[color][2]]), g.upper[color])
                mask_2 = cv2.inRange(hsv, g.lower[color], np.array([180, g.upper[color][1], g.upper[color][2]]))
                mask[color] = cv2.bitwise_or(mask_1, mask_2)
        temp = cv2.bitwise_or(mask['blue'], mask['green'])
        mask = cv2.bitwise_or(temp, mask['red'])
    else:
        if g.lower[color][0] <= g.upper[color][0]:
            mask = cv2.inRange(hsv, g.lower[color], g.upper[color])
        else:
            mask_1 = cv2.inRange(hsv, np.array([0, g.lower[color][1], g.lower[color][2]]), g.upper[color])
            mask_2 = cv2.inRange(hsv, g.lower[color], np.array([180, g.upper[color][1], g.upper[color][2]]))

            mask = cv2.bitwise_or(mask_1, mask_2)
    return mask

def erode(image, color = 'black'):
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.erode(image, kernel)
    return mask

def dilate(image, ker = 10):
    kernel = np.ones((ker, ker), np.uint8)
    mask = cv2.dilate(image, kernel)
    return mask

def canny(image, color = 'black'):
    mask = transform(image, color)
    mask = dilate(mask)
    return cv2.Canny(mask, 0, 150, apertureSize = 3)

def roll_pitch(image, color = 'black', use_dilate = False):
    global last_roll
    modified = image.copy()
    if color == 'black':
        mask = transform(image)
    else:
        if color == 'rgb':
            color = image_color(image)
            if color == None:
                color = 'rgb'
        image = cv2.copyMakeBorder(image, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, [0,0,0])
        mask = transform(image, color)
    if use_dilate:
        mask = dilate(mask)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
    centers_x_sum = 0
    centers_y_sum = 0
    area_sum = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]
        M = cv2.moments(cnt)
        if area > g.image_shape[0] * g.image_shape[1] * 0.00:
            cv2.drawContours(modified, [approx], 0, (255, 255, 255), 2)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            centers_x_sum += cx * area
            centers_y_sum += cy * area
            area_sum += area
    if area_sum > 0:
        cx = centers_x_sum // area_sum
        cy = centers_y_sum // area_sum
        roll_feedback = g.image_shape[0] // 2 - cx
        pitch_feedback = cy - g.image_shape[1] // 2
        cv2.circle(modified ,(int(cx), int(cy)), 2, (0,0,255),-1)
        last_roll = roll_feedback
        return roll_feedback, pitch_feedback, modified
    try:
        if abs(last_roll) < (g.exe_time//5):
            roll_feedback = 0
        else:
            roll_feedback = last_roll - (abs(last_roll)/last_roll) * (g.exe_time//5)
    except:
        roll_feedback = 0
    last_roll = roll_feedback
    pos = g.image_shape[0] // 2 - roll_feedback
    cv2.circle(modified ,(int(pos), g.image_shape[1] // 2), 2, (0,0,255),-1)
    return roll_feedback, 0, modified

class Last:
    def __init__(self):
        self.roll = 0
        self.pitch = 0
last = Last()

def roll_pitch(image, color = 'black', use_dilate = False, method='area'):
    modified = image.copy()
    if color == 'black':
        mask = transform(image)
    else:
        if color == 'rgb':
            color = image_color(image)
            if color == None:
                color = 'rgb'
        mask = cv2.copyMakeBorder(image, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, [0,0,0])
        mask = transform(mask, color)
    if use_dilate:
        mask = dilate(mask)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
    if method == 'area':
        centers_x_sum = 0
        centers_y_sum = 0
        area_sum = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            M = cv2.moments(cnt)
            if area > g.image_shape[0] * g.image_shape[1] * 0.00:
                cv2.drawContours(modified, [approx], 0, (255, 255, 255), 2)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centers_x_sum += cx * area
                centers_y_sum += cy * area
                area_sum += area
        if area_sum > 0:
            cx = centers_x_sum // area_sum
            cy = centers_y_sum // area_sum
            roll_feedback = g.image_shape[0] // 2 - cx
            pitch_feedback = cy - g.image_shape[1] // 2
        else:
            roll_feedback = 0
            pitch_feedback = 0
    elif method == 'center':
        threshold = g.image_shape[0] // 2 * g.image_shape[1]
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            M = cv2.moments(cnt)
            if area > g.image_shape[0] * g.image_shape[1] * 0.00:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                roll_feedback = g.image_shape[0] // 2 - cx
                pitch_feedback = cy - g.image_shape[1] // 2
                if abs(roll_feedback) + abs(pitch_feedback) < threshold:
                    target = cnt
                    threshold = abs(roll_feedback) + abs(pitch_feedback)
        try:
            approx = cv2.approxPolyDP(target, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            M = cv2.moments(target)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.drawContours(modified, [approx], 0, (255, 255, 255), 2)
            roll_feedback = g.image_shape[0] // 2 - cx
            pitch_feedback = cy - g.image_shape[1] // 2
        except:
            roll_feedback = 0
            pitch_feedback = 0
    elif method == 'red' or method == 'green' or method == 'blue':
        rect = 5
        targets = []
        mask_color = cv2.copyMakeBorder(modified, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, [0,0,0])
        mask_color = transform(mask_color, method)
        mask_color = dilate(mask_color)
        max_area = 0
        _, contours_color, _ = cv2.findContours(mask_color, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
        has_color = False
        for cnt in contours_color:
            area = cv2.contourArea(cnt)
            if area > max_area:
                cnt_color = cnt
                max_area = area
                has_color = True
        if has_color:
            has_target = False
            approx = cv2.approxPolyDP(cnt_color, 0.02*cv2.arcLength(cnt, True), True)
            cv2.drawContours(modified, [approx], 0, (255, 255, 255), 2)
            for cnt in contours:
                approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                dist = cv2.pointPolygonTest(cnt_color,(x, y),True)
                if dist > 0:
                    target = cnt
                    has_target = True
            if has_target:
                approx = cv2.approxPolyDP(target, 0.02*cv2.arcLength(cnt, True), True)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                M = cv2.moments(target)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.drawContours(modified, [approx], 0, (255, 255, 255), 2)
                roll_feedback = g.image_shape[0] // 2 - cx
                pitch_feedback = cy - g.image_shape[1] // 2
            else:
                approx = cv2.approxPolyDP(cnt_color, 0.02*cv2.arcLength(cnt, True), True)
                x = approx.ravel()[0]
                y = approx.ravel()[1]
                M = cv2.moments(cnt_color)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.drawContours(modified, [approx], 0, (255, 255, 255), 2)
                roll_feedback = g.image_shape[0] // 2 - cx
                pitch_feedback = cy - g.image_shape[1] // 2
        else:
            roll_feedback = 0
            pitch_feedback = 0
    
    d_roll = roll_feedback - last.roll
    d_pitch = pitch_feedback - last.pitch
    roll_feedback = last.roll + d_roll*0.2
    pitch_feedback = last.pitch + d_pitch*0.2
    
    cv2.circle(modified ,(int(g.image_shape[0] // 2 - roll_feedback), int(g.image_shape[1] // 2 + pitch_feedback)), 2, (0,0,255),-1)
    last.roll = roll_feedback
    last.pitch = pitch_feedback
    return roll_feedback, pitch_feedback, modified

def yaw(image, rgb = False, turn = False):
    modified = image.copy()
    if rgb:
        mask = canny(image, 'rgb')
    else:
        mask = canny(image)
    lines = cv2.HoughLinesP(mask, 1, np.pi/180,0, np.array([]),minLineLength=g.exe_time//2,maxLineGap=5)
    try:
        len(lines)
    except:
        return 0, modified
    yaw_feedback = vertical_slope(modified, lines, turn=turn)
    return yaw_feedback, modified

def vertical_slope(img, lines, lines_color=[0, 255, 255], thickness=2, turn = False):
    global turn_left_flag
    try:
        turn_left_flag = turn_left_flag * 1
    except:
        turn_left_flag = 0
    find_vertical_flag = 0
    find_horizontal_flag = 0
    max_slope = 1000000
    most_vertical = max_slope
    most_horizontal = 0
    v_point = [0,0,0,0]
    h_point = [0,0,0,0]
    transform_img = dilate(transform(img))
    x_max = g.image_shape[0]
    y_max = g.image_shape[1]

    for line in lines:
        for x1, y1, x2, y2 in line:
            # Vertical detection
            is_horizontal = False
            try:
                if x1 == x2:
                    cv2.line(img, (x1, y1), (x2, y2), lines_color, thickness)
                    if find_vertical_flag:
                        continue
                    else:
                        find_vertical_flag = 1
                        v_point[0],v_point[1],v_point[2],v_point[3] =  x1,y1,x2,y2
                        continue

                a = (y2 - y1) / (x2 - x1)

                if abs(a) < 1:
                    is_horizontal = True
                if not is_horizontal:
                    find_vertical_flag = 1
                    if abs(a) < most_vertical:
                        most_vertical = a
                        v_point[0],v_point[1],v_point[2],v_point[3] =  x1,y1,x2,y2
                    cv2.line(img, (x1, y1), (x2, y2), lines_color, thickness)
                    continue
            except:
                pass

            # Horizontal detection
            try:
                a = (y2 - y1) / (x2 - x1)

                if abs(a) > 1:
                    continue
                find_horizontal_flag = 1
                if abs(a) > most_horizontal:
                    most_horizontal = a
                    h_point[0],h_point[1],h_point[2],h_point[3] =  x1,y1,x2,y2
                cv2.line(img, (x1, y1), (x2, y2), lines_color, thickness)    
            except:
                pass
    if not turn:
        if most_vertical == max_slope:
            return 0
        return 1/most_vertical
    if find_vertical_flag and not find_horizontal_flag:
        turn_left_flag = 0
        
    elif not find_vertical_flag and find_horizontal_flag:
        turn_left_flag += 1
        
    elif not find_vertical_flag and not find_horizontal_flag:
        turn_left_flag = 0
    else:    
        #check one or two side
        if most_vertical == max_slope:
            h_b = h_point[1] - most_horizontal*h_point[0]
            intersect_x = v_point[0]
            intersect_y = most_horizontal*intersect_x + h_b
        else:
            v_b = v_point[1] - most_vertical*v_point[0]
            h_b = h_point[1] - most_horizontal*h_point[0]
            intersect_y = (most_horizontal*v_b - most_vertical*h_b) / (most_horizontal - most_vertical)
            intersect_x = (intersect_y - v_b) / most_vertical
        
        #print('%2f,%2f'%(intersect_x,intersect_y))

        if intersect_x < x_max and intersect_x >=0 and intersect_y < y_max and intersect_y >=0:
            #check if rightside of horizontal line has black point
            h_rightside_x = [int(( (x_max-1) + intersect_x ) // 2)]
            h_rightside_y = [int(most_horizontal*h_rightside_x[0] + h_b)]
            flag_black_h = 0
            for i in range(1):
                for j in range(25):
                    h_check_y = h_rightside_y[i]-2 + int(j/5)
                    h_check_x = h_rightside_x[i]-2 + j%5
                    if h_check_y >= 0 and h_check_y < y_max and h_check_x >= 0 and h_check_x < x_max:
                        if transform_img[h_check_y][h_check_x] == 255:
                            flag_black_h=1
                            break             
            
            #check if upside of vertical line has black point
            v_upside_y = [int(intersect_y // 2)]
            if most_vertical == max_slope:
                v_upside_x = [int(intersect_x)]
            else:
                v_upside_x = [int((v_upside_y[0] - v_b) / most_vertical)]
            flag_black_v = 0
            for i in range(1):
                for j in range(25):
                    v_check_y = v_upside_y[i]-2 + int(j/5)
                    v_check_x = v_upside_x[i]-2 + j%5
                    if v_check_y >= 0 and v_check_y < y_max and v_check_x >= 0 and v_check_x < x_max:
                        if transform_img[v_check_y][v_check_x] == 255:
                            flag_black_v=1
                            break

            if not flag_black_h and not flag_black_v:
                turn_left_flag += 1
                
            else:
                turn_left_flag = 0
                
        else:
            turn_left_flag = 0
            
    if turn_left_flag >= 3:
        # print('turn_left', turn_left_flag)
        return 10
    else:
        # print('straight', turn_left_flag)
        if most_vertical == max_slope:
            return 0
        return 1/most_vertical
    
def sandbag(throw):
    if throw:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(100)
    else:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(0)
    
def find_landing_line(image):
    landing_lind_found = False
    mask = transform(image, 'red')
    lines = lines = cv2.HoughLinesP(mask, 1, np.pi/180,0, np.array([]),minLineLength=50,maxLineGap=5)
    
    try:
        if len(lines) > 2:
            landing_lind_found = True
    except:
        pass
    return landing_lind_found

if __name__ == '__main__':
    cam = Camera(0)
    while True:
        image = cam.read()
        cv2.imshow('Frame', cv2.resize(image, (400, 400)))
        image = dilate(transform(image))
        # r, p, image = roll_pitch(image, 'black')
        # r, p, image = circle(image)
        # y, image = yaw(image, rgb = True)
        image = cv2.resize(image, (400, 400))
        cv2.imshow('live', image)
        key = cv2.waitKey(1)
        if key == 27:
            break
    cam.stop()
    
