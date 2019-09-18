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

def fish_eye_dis(img):
    width_in, height_in = img.shape[0], img.shape[1]
    im_out = np.zeros(img.shape, dtype=np.uint8)
    radius = max(width_in, height_in)/2
    #assume the fov is 180
    #R = f*theta
    lens = radius*2/(math.pi*9.5/18)
    for i in range(width_in):
        for j in range(height_in):
            #offset to center
            x = i - width_in/2
            y = j - height_in/2
            r = math.sqrt(x*x + y*y)
            theta = math.atan(r/radius)
            if theta<0.0001:
                k = 1
            else:
                k = lens*theta/r

            src_x = x*k
            src_y = y*k
            src_x = src_x+width_in/2
            src_y = src_y+height_in/2
            pixel = img[int(src_x),int(src_y)]
            im_out[i, j] = pixel

    return im_out

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
    if color_sum[max_color_index] >= g.color_area:
        color_index = max_color_index
    return colors[color_index]

def transform(image, color = 'black'):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if color == 'black':
        mask_black = cv2.inRange(hsv, g.lower['black'], g.upper['black'])
        mask = mask_black
        # gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # mask = ((gray_image<=g.rate*np.average(gray_image))*255).astype(np.uint8)
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
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(image, kernel)
    return mask

def dilate(image, color = 'black'):
    kernel = np.ones((12, 12), np.uint8)
    mask = cv2.dilate(image, kernel)
    return mask

def canny(image, color = 'black'):
    mask = transform(image, color)
    mask = dilate(mask)
    # mask = cv2.GaussianBlur(mask,(3,3),0)
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

        

def circle(image):
    modified = image.copy()
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mask = cv2.GaussianBlur(gray_image, (5,5), 0)
    try:
        circles= np.array(cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT, 1, 120,param1=80,param2=50,minRadius=20,maxRadius=200))[0]
    except:
        return 0, 0, modified
    
    max_radius = np.argmax(circles[:, 2])
    x, y, r = circles[max_radius][0], circles[max_radius][1], circles[max_radius][2]
    # 最大圓
    cv2.circle(modified ,(x, y),r,(255,255,255),2,8,0)
    # 所有圓
    # for x, y, r in circles:
    #     cv2.circle(image ,(x, y),r,(255,255,255),2,8,0)
    roll_feedback = g.image_shape[0] // 2 - x
    pitch_feedback = y - g.image_shape[1] // 2
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
            h_rightside_x = int(( (x_max-1) + intersect_x ) // 2)
            h_rightside_y = int(most_horizontal*h_rightside_x + h_b)
            flag_black_h = 0
            for i in range(25):
                h_check_y = h_rightside_y-2 + int(i/5)
                h_check_x = h_rightside_x-2 + i%5
                if h_check_y >= 0 and h_check_y < y_max and h_check_x >= 0 and h_check_x < x_max:
                    if transform_img[h_check_y][h_check_x] == 255:
                        flag_black_h=1
                        break             
            
            #check if upside of vertical line has black point
            v_upside_y = int(intersect_y // 2)
            if most_vertical == max_slope:
                v_upside_x = int(intersect_x)
            else:
                v_upside_x = int((v_upside_y - v_b) / most_vertical)
            flag_black_v = 0
            for i in range(25):
                v_check_y = v_upside_y-2 + int(i/5)
                v_check_x = v_upside_x-2 + i%5
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
        t = 0
        r = 0
        p = 0
        y = 0
        image = dilate(transform(image, 'rgb'))
        # image = canny(image)
        # r, p, image = roll_pitch(image, 'black')
        # r, p, image = circle(image)
        # y, image = yaw(image, rgb = True)
        image = cv2.resize(image, (400, 400))
        texts = []
        texts.append('Throttle: '+str(t))
        texts.append('Pitch: '+str(p))
        texts.append('Roll: '+str(r))
        texts.append('Yaw: '+str(y))
        for i in range(4):
            cv2.putText(image, texts[i], (0, 25+25*i), cv2.FONT_HERSHEY_TRIPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow('live', image)
        key = cv2.waitKey(1)
        if key == 27:
            break
    cam.stop()
    
