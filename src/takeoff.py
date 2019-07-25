# -*- coding: utf-8 -*

import numpy as np
import time
import PID
import pigpio
import cv2
import config as g
from camera import Camera

def transform(image):
    rate = 0.5
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    transform_image = (gray_image>g.rate*np.average(gray_image)).astype(float)
    cv2.imshow('transform', transform_image)
    cv2.waitKey(30)
    return transform_imagezz

def calculate_distance(image):
    image = transform(image)
    focal = 840
    length = 4
    image_height = image.shape[0]
    image_width = image.shape[1]
    num = 0
    total = 0
    for i in range(image_width):
        if image[image_height//2, i] == 0:
            total += i
            num += 1
    try:
        distance = focal * length / num
        print(distance)
    except:
        return

if __name__ == '__main__':
    cam = Camera()
    while True:
        s =time.time()
        image = cam.getframe()
        calculate_distance(image)
    cam.release()

