import cv2
import numpy as np
import config as g
image_path = '../data/red_test.jpg'
def image_color(image_path):
    colors = ['blue', 'green', 'red', None]
    color_index = -1
    image = cv2.imread(image_path)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    mask_blue = cv2.inRange(hsv, g.lower_blue, g.upper_blue)
    mask_green = cv2.inRange(hsv, g.lower_green, g.upper_green)
    mask_red_1 = cv2.inRange(hsv, g.lower_red_1, g.upper_red_1)
    mask_red_2 = cv2.inRange(hsv, g.lower_red_2, g.upper_red_2)
    mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)

    color_sum = [mask_blue.sum(), mask_green.sum(), mask_red.sum()]

    '''
    res_blue = cv2.bitwise_and(image, image, mask = mask_blue)
    res_green = cv2.bitwise_and(image, image, mask = mask_green)
    res_red = cv2.bitwise_and(image, image, mask = mask_red)
    cv2.imshow('original image', image)
    cv2.imshow('res_blue', res_blue)
    cv2.imshow('res_green', res_green)
    cv2.imshow('res_red', res_red)
    cv2.waitKey()
    cv2.destroyAllWindows()'''
    
    max_color_index = np.argmax(color_sum)
    if color_sum[max_color_index] >= 2000000:
        color_index = max_color_index
    return colors[color_index]
if __name__ == '__main__':
    print(image_color(image_path))
