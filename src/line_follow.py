
# -*- coding: utf-8 -*
# 需要依據環境調整的參數
# rate ( 控制 image transform )

import numpy as np
import time
import picamera
import PID
import pigpio
import cv2
import config as g

# without printing        

def transform(image):
    rate = 0.5
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    transform_image = (gray_image>g.rate*np.average(gray_image)).astype(float)
    return transform_image
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
def angle_to_dc(angle):
    '''
    -180<=angle<=180
    500<=dc<=1000
    '''
    dc = 25 * angle / 18 + 750
    return dc
def output_yaw(image_path):
    image = cv2.imread(image_path)
    image_shape = image.shape
    pid = PID.PID(g.yaw_Kp, g.yaw_Ki, g.yaw_Kd)
    pid.SetPoint=image_shape[1]//2
    pid.setSampleTime(0.01)
    transform_image = transform(image)
    feedback = line_pos(transform_image)
    pid.update(feedback)
    output = -pid.output
    dc = angle_to_dc(output)
    print('output', output, 'dc:', dc)
    return

pi = pigpio.pi()
pi.set_PWM_frequency(g.yaw_pin, g.PWM_FREQ)
pi.set_PWM_range(g.yaw_pin, g.PWM_RANGE)


if __name__ == '__main__':
    camera = picamera.PiCamera()
    camera.resolution = g.image_shape
    while True:
        camera.capture(g.image_path)
        output_yaw(g.image_path)
    camera.close()
