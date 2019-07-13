
# -*- coding: utf-8 -*
# 需要依據環境調整的參數
# rate ( 控制 image transform )
# split_num ( 控制圖片分割數 )
# avg_gap ( 控制黑白分界 )

import skimage.io as io
from skimage.transform import resize
from skimage.color import rgb2gray
import numpy as np
import time
import picamera

# without printing
def transform(image):
    rate = 0.5
    gray_image = rgb2gray(image)
    transform_image = (gray_image>rate*np.average(gray_image)).astype(float)
    return transform_image
def split_image(image):
    split_num = 3
    avg_gap = 0.99
    array = []
    height = image.shape[0]//split_num
    width = image.shape[1]//split_num
    for i in range(split_num):
        row = []
        for j in range(split_num):
            temp = image[height*i:height*(i+1),width*j:width*(j+1)]
            avg = np.average(temp)
            if avg >= avg_gap:
                row.append(0)
            else:
                row.append(1)
        array.append(row)
    return np.array(array)
# Only for 3*3 split
def motion(array):
    height = array.shape[0] # 3
    width = array.shape[1] # 3
    
    left_forward = array[0, 0]
    forward = array[0, 1]
    right_forward = array[0, 2]
    
    left = array[1, 0]
    middle = array[1, 1]
    right = array[1, 2]
    
    left_backward = array[2, 0]
    backward = array[2, 1]
    right_backward = array[2, 2]
    
    motion = 'error'
    
    # 如果前方是1，則往前走
    if forward == 1:
        motion = 'forward'
    # 如果前方是0，則看左前和右前
    else:
        # 如果左前和右前都是1，檢查左右
        if (left_forward == 1) & (right_forward == 1):
            if (left == 1) & (right == 1):
                if backward == 1:
                    motion = 'backward'
            elif left == 1:
                motion = 'left forward'
            elif right == 1:
                motion = 'right forward'
            elif (left == 0) & (right == 0):
                pass
        # 如果左前是1，則往左前走
        elif left_forward == 1:
            motion = 'left forward'
        # 如果右前是1，則往右前走
        elif right_forward == 1:
            motion = 'right forward'
        # 如果左前和右前都是0，檢查左右
        elif (left_forward == 0) & (right_forward == 0):
            if (left == 1) & (right == 1):
                # 左中右都是1，降落
                if middle == 1:
                    motion = 'landing'
                elif backward == 1:
                    motion = 'backward'
            elif left == 1:
                motion = 'left'
            elif right == 1:
                motion = 'right'
            elif (left == 0) & (right == 0):
                motion = 'backward'
    return motion
def scatter_data(image):
    x = []
    y = []
    for i in range(0, image.shape[0], 10):
        for j in range(0, image.shape[1], 10):
            if image[i, j] == 0:
                x.append(j)
                y.append(i)
    return [x, y]
def rotation(image):
    data = scatter_data(image)
    try:
        weights = np.polyfit(data[1], data[0], 1)
        return weights[0]
    except:
        return 0
def all_steps(image):
    transform_image = transform(image)
    array = split_image(transform_image)
    move = motion(array)
    rotate = rotation(transform_image)
    print('Motion:', move)
    print('Rotation range:', rotate)
    return move

image_shape = (300, 300)

if __name__ == '__main__':
    camera = picamera.PiCamera()
    camera.resolution = image_shape
    while True:
        camera.capture('../data/test.jpg')
        s = time.time()
        image = io.imread('../data/test.jpg')
        move = all_steps(image)
        print(time.time()-s)
        if move == 'landing':
            break
    camera.close()

