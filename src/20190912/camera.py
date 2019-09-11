# -*- coding: utf-8 -*-
import cv2
import time
import threading
import config as g

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
   
    def read(self):
    # 當有需要影像時，再回傳最新的影像。
        image_shape = self.Frame.shape
        # self.Frame = self.Frame[int(image_shape[0]*0.2): int(image_shape[0]*0.8), int(image_shape[1]*0.2): int(image_shape[1]*0.8)]
        self.Frame = cv2.resize(self.Frame, g.image_shape)
        return self.Frame
    def queryframe(self):
        while (not self.isstop):
            self.status, self.Frame = self.capture.read()
        
        self.capture.release()
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
