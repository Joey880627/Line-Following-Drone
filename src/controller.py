
# -*- coding: utf-8 -*
import picamera
import line_follow as lf

image_path = '../data/test.jpg'
image_shape = (300, 300)

if __name__ == '__main__':
    camera = picamera.PiCamera()
    camera.resolution = image_shape
    while True:
        camera.capture(image_path)
        lf.output_yaw(image_path)
    camera.close()
