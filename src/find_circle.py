# -*- coding: utf-8 -*-
import  cv2
import numpy as np
from camera import Camera
import config as g
class Circle_tracking:
    def __init__(self):
        self.horizontal_shift = 0
        self.vertical_shift = 0
    def get_shift(self, image):
        result = cv2.blur(image, (5,5))
        gray=cv2.cvtColor(result,cv2.COLOR_BGR2GRAY)
        #霍夫變換圓檢測
        try:
            circles= np.array(cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,50,param1=80,param2=30,minRadius=1,maxRadius=1000))[0]
        except:
            return self.horizontal_shift, self.vertical_shift
            # return 0, 0
        max_radius = np.argmax(circles[:, 2])
        x, y, r = circles[max_radius][0], circles[max_radius][1], circles[max_radius][2]
        #顯示新圖像
        # img=cv2.circle(image,(x, y),r,(0,0,255),1,8,0)
        # cv2.imshow('circle',img)
        # cv2.waitKey(1)
        self.horizontal_shift = g.image_shape[0] // 2 - x
        self.vertical_shift = y - g.image_shape[1] // 2
        return self.horizontal_shift, self.vertical_shift
    
if __name__=='__main__':
    cam = Camera()
    ct = Circle_tracking()
    while True:
        image = cam.getframe()
        horizontal_shift, vertical_shift = ct.get_shift(image)
        print('Horizontal shift:', horizontal_shift)
        print('Vertical shift:', vertical_shift)
    cam.release()