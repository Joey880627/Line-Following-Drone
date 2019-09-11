# -*- coding: utf-8 -*

import numpy as np
import time
import PID
import cv2
import config as g
from camera import Camera
import warnings
warnings.filterwarnings('ignore')

def image_color(image):
    colors = ['blue', 'green', 'red', None]
    color_index = -1
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask_blue = cv2.inRange(hsv, g.lower['blue'], g.upper['blue'])
    mask_green = cv2.inRange(hsv, g.lower['green'], g.upper['green'])
    mask_red_1 = cv2.inRange(hsv, g.lower['red_1'], g.upper['red_1'])
    mask_red_2 = cv2.inRange(hsv, g.lower['red_2'], g.upper['red_2'])
    mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)

    color_sum = [(mask_blue!=0).sum(),
                 (mask_green!=0).sum(),
                 (mask_red!=0).sum()]
    max_color_index = np.argmax(color_sum)
    if color_sum[max_color_index] >= g.color_area:
        color_index = max_color_index
    return colors[color_index]

def transform(image, color = 'black'):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if color == 'black':
        # mask_black = cv2.inRange(hsv, g.lower['black'], g.upper['black'])
        # mask = mask_black
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        mask = ((gray_image<=g.rate*np.average(gray_image))*255).astype(np.uint8)
    elif color == 'red':
        mask_red_1 = cv2.inRange(hsv, g.lower['red_1'], g.upper['red_1'])
        mask_red_2 = cv2.inRange(hsv, g.lower['red_2'], g.upper['red_2'])
        mask = cv2.bitwise_or(mask_red_1, mask_red_2)
    elif color == 'rgb':
        mask_blue = cv2.inRange(hsv, g.lower['blue'], g.upper['blue'])
        mask_green = cv2.inRange(hsv, g.lower['green'], g.upper['green'])
        mask_red_1 = cv2.inRange(hsv, g.lower['red_1'], g.upper['red_1'])
        mask_red_2 = cv2.inRange(hsv, g.lower['red_2'], g.upper['red_2'])
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        temp = cv2.bitwise_or(mask_blue, mask_green)
        mask = cv2.bitwise_or(temp, mask_red)
    else:
        mask = cv2.inRange(hsv, g.lower[color], g.upper[color])
    return mask

def erode(image, color = 'black'):
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(image, kernel)
    return mask

def canny(image, color = 'black'):
    mask = transform(image, color)
    # mask = cv2.GaussianBlur(mask,(3,3),0)
    return cv2.Canny(mask, 10, 150, apertureSize = 3)

def roll_pitch(image, color = 'black'):
    modified = image.copy()
    if color == 'black':
        mask = erode(transform(image))
    else:
        if color == 'rgb':
            color = image_color(image)
            if color == None:
                color = 'rgb'
        image = cv2.copyMakeBorder(image, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, [0,0,0])
        # mask = erode(transform(image, color))
        mask = transform(image, color)
    
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                      cv2.CHAIN_APPROX_SIMPLE)
    
    if contours == None:
        return 0, 0, modified
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
            if color == 'black':
                cv2.drawContours(modified, [approx], 0, (255, 255, 255), 1)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                centers_x_sum += cx * area
                centers_y_sum += cy * area
                area_sum += area
            else:
                if len(approx) <= 6:
                    cv2.drawContours(modified, [approx], 0, (255, 255, 255), 1)
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
        cv2.circle(modified ,(int(cx), int(cy)), 1, (0,0,255),-1)
        return roll_feedback, pitch_feedback, modified
    
    cv2.circle(modified ,(g.image_shape[0] // 2, g.image_shape[1] // 2), 2, (0,0,255),-1)
    return 0, 0, modified

        

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

def yaw(image, rgb = False):
    modified = image.copy()
    if rgb:
        mask = canny(image, 'rgb')
    else:
        mask = canny(image)
    lines = cv2.HoughLinesP(mask, 1, np.pi/180,0, np.array([]),minLineLength=15,maxLineGap=5)
    try:
        len(lines)
    except:
        return 0, modified
    if rgb:
        yaw_feedback = vertical_slope(modified, lines)
    else:
        yaw_feedback = vertical_slope(modified, lines)
    return yaw_feedback, modified

def vertical_slope(img, lines, color=[0, 255, 255], thickness=3):   
    find_vertical_flag = 0
    find_horizontal_flag = 0
    max_slope = 1000000
    most_vertical = max_slope
    most_horizontal = 0

    for line in lines:
        for x1, y1, x2, y2 in line:
            # Vertical detection
            is_horizontal = False
            try:
                if x1 == x2:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                    if find_vertical_flag:
                        continue
                    else:
                        find_vertical_flag = 1
                        v_point[0],v_point[1],v_point[2],v_point[3] =  x1,y1,x2,y2
                        continue

                a = (y2 - y1) / (x2 - x1)

                if abs(a) < 1.2:
                    is_horizontal = True
                if not is_horizontal:
                    find_vertical_flag = 1
                    if abs(a) < most_vertical:
                        most_vertical = a
                        v_point[0],v_point[1],v_point[2],v_point[3] =  x1,y1,x2,y2
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                    success += 1
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
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)    
            except:
                pass
    if not find_vertical_flag and find_horizontal_flag:
        return 2 - 0.05*most_horizontal
    else:
        if most_vertical == max_slope:
            return 0
        return 1/most_vertical
    
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
        image = transform(image, 'black')
        # image = canny(image)
        # r, p, image = roll_pitch(image, 'black')
        # r, p, image = circle(image)
        # y, image = yaw(image, rgb = True)
        image = cv2.resize(image, (400, 300))
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
    
