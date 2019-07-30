# -*- coding: utf-8 -*

import numpy as np
import time
import PID
import cv2
import config as g
import color_recognization as cr
from camera import Camera

def transform(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    transform_image = ((gray_image>g.rate*np.average(gray_image))*255).astype(np.uint8)
    return transform_image

def draw_lines(img, lines, color=[0, 255, 0], thickness=3):
    success = 0
    most_vertical = 0
    most_horizontal = 0
    for line in lines:
        for x1, y1, x2, y2 in line:
            # Vertical detection
            is_horizontal = False
            try:
                a = (x2 - x1) / (y2 - y1)
                # 要夠垂直
                if abs(a) > 2:
                    is_horizontal = True
                if not is_horizontal:
                    if abs(a) > most_vertical:
                        most_vertical = a
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                    success += 1
                    continue
            except:
                pass

            # Horizontal detection
            a = (y2 - y1) / (x2 - x1)
            # 要夠水平
            if abs(a) > 2:
                continue
            most_horizontal = a
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)
            success += 1
    # print(success)
    return most_vertical
def draw_path(img, lines, color=[0, 255, 0], thickness=3):   
    success = 0
    find_vertical_flag = 0
    find_horizontal_flag = 0
    max_slope = 1000000
    most_vertical = max_slope
    most_horizontal = 0
    v_point = [0,0,0,0]
    h_point = [0,0,0,0]
    transform_img = transform(img)
    x_max = g.image_shape[0]
    y_max = g.image_shape[1]

    for line in lines:
        for x1, y1, x2, y2 in line:
            # Vertical detection
            is_horizontal = False
            try:
                if x1 == x2:
                    cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                    if find_vertical_flag:
                        continues
                    else:
                        find_vertical_flag = 1
                        v_point[0],v_point[1],v_point[2],v_point[3] =  x1,y1,x2,y2
                        continue

                a = (y2 - y1) / (x2 - x1)
                # 要夠垂直
                if abs(a) < 0.8:
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
                # 要夠水平
                if abs(a) > 1:
                    continue
                find_horizontal_flag = 1
                if abs(a) > most_horizontal:
                    most_horizontal = a
                    h_point[0],h_point[1],h_point[2],h_point[3] =  x1,y1,x2,y2
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)    
            except:
                pass

    if find_vertical_flag and not find_horizontal_flag:
        print('only_vertical\n')
        if most_vertical == max_slope:
            return 0
        return 1/most_vertical
        #return 0
    elif not find_vertical_flag and find_horizontal_flag:
        print('only_horizontal\n')
        # return -10*most_horizontal + 2
        return 0.5
        #return 0
    elif not find_vertical_flag and not find_horizontal_flag:
        print('no_line_detected\n')
        if most_vertical == max_slope:
            return 0
        return 1/most_vertical
        #return 0

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
            if transform_img[h_rightside_y-2 + int(i/5)][h_rightside_x-2 + i%5] == 0:
                flag_black_h=1
        if flag_black_h:    
            print('horizontal: two_side\n')
            if most_vertical == max_slope:
                return 0
            return 1/most_vertical
            #return 0
        else:  
            print('horizontal: one_side')
        
        #check if upside of vertical line has black point
        v_upside_y = int(intersect_y // 2)
        if most_vertical == max_slope:
            v_upside_x = int(intersect_x)
        else:
            v_upside_x = int((v_upside_y - v_b) / most_vertical)
        flag_black_v = 0
        for i in range(25):
            if transform_img[v_upside_y-1 + int(i/5)][v_upside_x-1 + i%5] == 0:
                flag_black_v=1
        if flag_black_v:
            print('vertical: up_side\n')
            if most_vertical == max_slope:
                return 0
            return 1/most_vertical
            #return 0
        else: 
            print('vertical: down_side\n')
            # return -10*most_horizontal + 2
            return 0.5
            #return 0
    return 0
def line_pos(image):
    image_height = image.shape[0]
    image_width = image.shape[1]
    num = 0
    total = 0
    for i in range(image_width):
        if image[image_height//2, i] == 0:
            total += i
            num += 1
    try:
        pos = total / num
    except:
        pos = image_width / 2
    return pos

class Line_tracking():
    def __init__(self):
        self.roll_feedback = 0
        self.yaw_feedback = 0
    def get_shift(self, image):
        # self.roll_feedback = g.image_shape[0] // 2 - line_pos(transform(image))
        # return self.roll_feedback
        roll_feedback = g.image_shape[0] // 2 - line_pos(transform(image))
        return roll_feedback
    def get_slope(self, image):
        rho = 1
        theta = np.pi / 180
        threshold = 0
        min_line_length = 50
        max_line_gap = 5
        original = image.copy()
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = transform(image)
        image = cv2.Canny(image, 10, 150, apertureSize = 3)
        lines = cv2.HoughLinesP(image, rho, theta,
                                threshold, np.array([]),
                                minLineLength=min_line_length,
                                maxLineGap=max_line_gap)
        
        try:
            self.yaw_feedback = draw_path(original, lines)
        except:
            # cv2.imshow('live', original)
            # cv2.waitKey(1)
            # return self.yaw_feedback
            return 0
        # cv2.imshow('live', original)
        # cv2.waitKey(1)
        return self.yaw_feedback
class Rectangle_tracking:
    def __init__(self):
        self.horizontal_shift = 0
        self.vertical_shift = 0
        self.yaw_feedback = 0
        self.color = 'red'
        
    def get_shift(self, image):
        color = cr.image_color(image)
        if color == None:
            color = self.color
        else:
            self.color = color
        image = cv2.copyMakeBorder(image, 1, 1, 1, 1, cv2.BORDER_CONSTANT, None, [0,0,0])
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = {}
        mask['blue'] = cv2.inRange(hsv, g.lower_blue, g.upper_blue)
        mask['green'] = cv2.inRange(hsv, g.lower_green, g.upper_green)
        mask_red_1 = cv2.inRange(hsv, g.lower_red_1, g.upper_red_1)
        mask_red_2 = cv2.inRange(hsv, g.lower_red_2, g.upper_red_2)
        mask['red'] = cv2.bitwise_or(mask_red_1, mask_red_2)
        kernel = np.ones((5, 5), np.uint8)
        mask[color] = cv2.erode(mask[color], kernel)
        mask[color] = cv2.Canny(mask[color], 50, 160)
        _, contours, _ = cv2.findContours(mask[color], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        font = cv2.FONT_HERSHEY_COMPLEX
        centers_x_sum = 0
        centers_y_sum = 0
        area_sum = 0
        if contours == None:
            # return self.horizontal_shift, self.vertical_shift
            return 0, 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
            x = approx.ravel()[0]
            y = approx.ravel()[1]
            M = cv2.moments(cnt)
            if area > 400:
                # if len(approx) == 4:
                if True:
                    cv2.drawContours(image, [approx], 0, (255, 255, 255), 2)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    centers_x_sum += cx * area
                    centers_y_sum += cy * area
                    area_sum += area
                    cv2.putText(image, color, (x, y+25), font, 1, (0, 0, 0))
        if area_sum > 0:
            cx = centers_x_sum // area_sum
            cy = centers_y_sum // area_sum
            self.horizontal_shift = g.image_shape[0] // 2 - cx
            self.vertical_shift = cy - g.image_shape[1] // 2
        # cv2.imshow("Mask", mask[color])
        # cv2.imshow("live", image)
        # cv2.waitKey(1)
        return self.horizontal_shift, self.vertical_shift
    def get_slope(self, image):
        color = cr.image_color(image)
        if color == None:
            color = self.color
        else:
            self.color = color
        original = image.copy()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = {}
        mask['blue'] = cv2.inRange(hsv, g.lower_blue, g.upper_blue)
        mask['green'] = cv2.inRange(hsv, g.lower_green, g.upper_green)
        mask_red_1 = cv2.inRange(hsv, g.lower_red_1, g.upper_red_1)
        mask_red_2 = cv2.inRange(hsv, g.lower_red_2, g.upper_red_2)
        mask['red'] = cv2.bitwise_or(mask_red_1, mask_red_2)
        kernel = np.ones((5, 5), np.uint8)
        mask[color] = cv2.erode(mask[color], kernel)
        mask[color] = cv2.Canny(mask[color], 50, 160)
        rho = 1
        theta = np.pi / 180
        threshold = 0
        min_line_length = 50
        max_line_gap = 5
        lines = cv2.HoughLinesP(mask[color], rho, theta,
                                threshold, np.array([]),
                                minLineLength=min_line_length,
                                maxLineGap=max_line_gap)
        
        try:
            self.yaw_feedback = draw_lines(original, lines)
        except:
            # cv2.imshow('live', original)
            # cv2.waitKey(1)
            # return self.yaw_feedback
            return 0
        # cv2.imshow('live', original)
        # cv2.waitKey(1)
        return self.yaw_feedback

class Circle_tracking:
    def __init__(self):
        self.horizontal_shift = 0
        self.vertical_shift = 0
    def get_shift(self, image):
        original = image.copy()
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = cv2.GaussianBlur(gray_image, (5,5), 0)
        # image = cv2.Canny(image, 10, 150, apertureSize = 3)
        #霍夫變換圓檢測
        try:
            circles= np.array(cv2.HoughCircles(image,cv2.HOUGH_GRADIENT,1,50,param1=80,param2=30,minRadius=1,maxRadius=1000))[0]
        except:
            # return self.horizontal_shift, self.vertical_shift
            return 0, 0
        print(len(circles))
        max_radius = np.argmax(circles[:, 2])
        '''
        flag = 255
        for i in range(len(circles)):
            x, y, r = int(circles[i][0]), int(circles[i][1]), circles[i][2]
            if x < 0:
                x = 0
            elif x > image.shape[0]:
                x = image.shape[0]-1
            if y < 0:
                y = 0
            elif y > image.shape[1]:
                y = image.shape[1]-1
            if gray_image[x, y] < flag:
                flag = gray_image[x][y]
                max_radius = i'''
        x, y, r = circles[max_radius][0], circles[max_radius][1], circles[max_radius][2]
        #顯示最大圓
        img=cv2.circle(original ,(x, y),r,(255,255,255),2,8,0)
        # cv2.imshow('live',img)
        # cv2.waitKey(1)
        self.horizontal_shift = g.image_shape[0] // 2 - x
        self.vertical_shift = y - g.image_shape[1] // 2
        return self.horizontal_shift, self.vertical_shift

if __name__ == '__main__':
    cam = Camera()
    while True:
        s =time.time()
        image = cam.getframe()
        feedback = get_shift(image)
        output, dc = g.pid_output(feedback, 'yaw')
        print('Yaw output', output, 'Yaw dc:', dc)
        print(time.time()-s)
    cam.release()
