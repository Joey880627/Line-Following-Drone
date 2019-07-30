import numpy as np
import PID
image_path = '../data/test.jpg'
exe_time = 100 # Bigger, slower
image_shape = (4 * exe_time, 3 * exe_time)
rate = 0.6 # 控制 image transform
PWM_FREQ = 50
PWM_RANGE = 10000
# image color range
lower_blue = np.array([80,43,46])
upper_blue = np.array([120,255,255])
lower_green = np.array([35,43,46])
upper_green = np.array([65,255,255])
lower_red_1 = np.array([170,100,100])
upper_red_1 = np.array([180,255,255])
lower_red_2 = np.array([0,100,100])
upper_red_2 = np.array([8,255,255])
color_area = image_shape[0]*image_shape[1] // 9

pin = {'throttle':0, 'pitch':18, 'roll':17, 'yaw':14}
Kp = {'throttle':1.2, 'pitch':1.2, 'roll':1.2, 'yaw':3}
Ki = {'throttle':4, 'pitch':4, 'roll':4, 'yaw':1}
Kd = {'throttle':0.005, 'pitch':0.05, 'roll':0.05, 'yaw':0.001}
max_gain = {'throttle':500, 'pitch':500, 'roll':500, 'yaw':1000}
sensitivity = {'throttle':1, 'pitch':0.5, 'roll':1, 'yaw':1} # Determine control sensitivity (0~1)
setpoint = {'throttle':0, 'pitch':0, 'roll':0, 'yaw':0}
directions = ['throttle', 'pitch', 'roll', 'yaw']

pid = {}
for direction in directions:
    pid[direction] = PID.PID(Kp[direction], Ki[direction], Kd[direction], maxGain=max_gain[direction])
    pid[direction].SetPoint=setpoint[direction]
