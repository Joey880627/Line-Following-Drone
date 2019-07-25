# -*- coding: utf-8 -*
import numpy as np
import cv2
import time
import math
from camera import Camera
import config as g



# No display
class OF:
    def __init__(self, old_frame, features_num=100):
        self.features_num = features_num
        self.feature_params = dict(maxCorners=self.features_num, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(15, 15),maxLevel=2,criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.color = np.random.randint(0, 255, (self.features_num, 3))
        self.old_frame = old_frame
        self.old_gray = cv2.cvtColor(self.old_frame, cv2.COLOR_BGR2GRAY)
        self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **self.feature_params)
        self.mask = np.zeros_like(old_frame)
        self.horizontal_shift = 0.0
        self.vertical_shift = 0.0
        self.horizontal_shift_per_frame = 0.0
        self.vertical_shift_per_frame = 0.0
    def get_shift(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 计算光流以获取点的新位置
        try:    
            self.p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
        except:
            self.p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
            self.horizontal_shift = 0.0
            self.vertical_shift = 0.0
            return self.horizontal_shift, self.vertical_shift
        # 选择good points
        if len(self.p0) == 0:
            self.p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
            self.horizontal_shift = 0.0
            self.vertical_shift = 0.0
            return self.horizontal_shift, self.vertical_shift
        good_new = self.p1[st == 1]
        good_old = self.p0[st == 1]
        # 绘制跟踪框
        horizontal_shift_list = []
        vertical_shift_list = []
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            horizontal_shift_list.append(c-a)
            vertical_shift_list.append(b-d)
            
        self.horizontal_shift_per_frame = np.average(np.array([horizontal_shift_list]))
        self.vertical_shift_per_frame = np.average(np.array([vertical_shift_list]))
        if math.isnan(self.horizontal_shift_per_frame):
            self.horizontal_shift_per_frame = 0.0
            self.vertical_shift_per_frame = 0.0
        self.horizontal_shift += self.horizontal_shift_per_frame
        self.vertical_shift += self.vertical_shift_per_frame
        self.old_gray = frame_gray.copy()
        self.p0 = good_new.reshape(-1, 1, 2)
        return self.horizontal_shift, self.vertical_shift
    def display_shift(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # 计算光流以获取点的新位置
        try:    
            self.p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)
        except:
            self.p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
            self.mask = np.zeros_like(self.old_frame)
            return
        # 选择good points
        if len(self.p0) == 0:
            self.p0 = cv2.goodFeaturesToTrack(frame_gray, mask=None, **self.feature_params)
            self.mask = np.zeros_like(self.old_frame)
            return
        good_new = self.p1[st == 1]
        good_old = self.p0[st == 1]
        for i, (new, old) in enumerate(zip(good_new, good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            self.mask = cv2.line(self.mask, (a, b), (c, d), self.color[i].tolist(), 2)
            frame = cv2.circle(frame, (a, b), 5, self.color[i].tolist(), -1)
        img = cv2.add(frame, self.mask)
        cv2.imshow('frame', img)
        if cv2.waitKey(30) == 27:
            return 'end'
        self.old_gray = frame_gray.copy()
        self.p0 = good_new.reshape(-1, 1, 2)
    
if __name__ == '__main__':
    cam = Camera()
    flow = OF(cam.getframe())
    hor = 0
    ver = 0
    while True:
        image = cam.getframe()
        horizontal_shift, vertical_shift = flow.get_shift(image)
        print('Horizontal shift:', horizontal_shift)
        print('Vertical shift:', vertical_shift)
        # if flow.display_shift(image) == 'end':
        #     break
    cam.release()