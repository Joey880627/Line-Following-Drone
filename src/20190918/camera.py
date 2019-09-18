# -*- coding: utf-8 -*-
import cv2
import numpy as np
import time
import threading
import config as g
import math
from PIL import Image

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
            if theta<0.00001:
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

# 接收攝影機串流影像，採用多執行緒的方式，降低緩衝區堆疊圖幀的問題。
class Camera:
    def __init__(self, URL=0):
        self.Frame = []
        self.status = False
        self.isstop = False
        
        # 攝影機連接。
        self.capture = cv2.VideoCapture(URL)
        self.capture.set(cv2.CAP_PROP_FPS,40)
        self.start()

    def start(self):
    # 把程式放進子執行緒，daemon=True 表示該執行緒會隨著主執行緒關閉而關閉。
        threading.Thread(target=self.queryframe, daemon=True, args=()).start()
        time.sleep(1)

    def stop(self):
    # 記得要設計停止無限迴圈的開關。
        self.isstop = True
   
    def readimg(self):
    # 當有需要影像時，再回傳最新的影像。
        return self.Frame
    def queryframe(self):
        while (not self.isstop):
            self.status, self.Frame = self.capture.read()
        self.capture.release()
    def read(self):
        image = self.readimg()
        image_shape = image.shape
        vision_deletion = (1-g.vision)/2
        image = image[int(image_shape[0]*vision_deletion): int(image_shape[0]*(1-vision_deletion)), int(image_shape[1]*vision_deletion): int(image_shape[1]*(1-vision_deletion))]
        image = cv2.resize(image, g.image_shape)
        return image
if __name__ =='__main__':

    # 連接攝影機
    cam = Camera(0)
    # 使用無窮迴圈擷取影像，直到按下Esc鍵結束
    while True:
        # 使用 getframe 取得最新的影像
        start = time.time()
        image = cam.read()
        cv2.imshow('Image', image)
        if cv2.waitKey(1) == 27:
            cv2.destroyAllWindows()
            cam.stop()
            break
        # print(time.time()-start)
    cam.stop()
