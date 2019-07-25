import numpy as np
import PID
import pigpio
image_path = '../data/test.jpg'
exe_time = 150 # Bigger, slower
image_shape = (4 * exe_time, 3 * exe_time)
rate = 0.5 # 控制 image transform
PWM_FREQ = 50
PWM_RANGE = 10000
# image color range
lower_blue = np.array([80,80,80])
upper_blue = np.array([100,255,255])
lower_green = np.array([35,43,46])
upper_green = np.array([55,255,255])
lower_red_1 = np.array([170,100,100])
upper_red_1 = np.array([180,255,255])
lower_red_2 = np.array([0,100,100])
upper_red_2 = np.array([8,255,255])

pin = {'throttle':0, 'pitch':18, 'roll':17, 'yaw':14}
Kp = {'throttle':1.2, 'pitch':1.2, 'roll':1.2, 'yaw':1.2}
Ki = {'throttle':1, 'pitch':1, 'roll':1, 'yaw':1}
Kd = {'throttle':0.001, 'pitch':0.001, 'roll':0.001, 'yaw':0.001}
max_gain = {'throttle':500, 'pitch':500, 'roll':500, 'yaw':1000}
sensitivity = {'throttle':1, 'pitch':0.5, 'roll':1, 'yaw':1} # Determine control sensitivity (0~1)
setpoint = {'throttle':0, 'pitch':0, 'roll':0, 'yaw':image_shape[1]//2}

# throttle control
throttle_pid = PID.PID(Kp['throttle'], Ki['throttle'], Kd['throttle'], maxGain=max_gain['throttle'])
# pitch control
pitch_pid = PID.PID(Kp['pitch'], Ki['pitch'], Kd['pitch'], maxGain=max_gain['pitch'])
# roll control
roll_pid = PID.PID(Kp['roll'], Ki['roll'], Kd['roll'], maxGain=max_gain['roll'])
# yaw control
yaw_pid = PID.PID(Kp['yaw'], Ki['yaw'], Kd['yaw'], maxGain=max_gain['yaw'])

print(setpoint['yaw'])
pid = {'throttle':throttle_pid, 'pitch':pitch_pid, 'roll':roll_pid, 'yaw':yaw_pid}
for direction in list(setpoint):
    pid[direction].SetPoint=setpoint[direction]

# pigpio output
pi = pigpio.pi()
pi.set_PWM_frequency(pin['roll'], PWM_FREQ)
pi.set_PWM_range(pin['roll'], PWM_RANGE)
pi.set_PWM_frequency(pin['pitch'], PWM_FREQ)
pi.set_PWM_range(pin['pitch'], PWM_RANGE)
pi.set_PWM_frequency(pin['yaw'], PWM_FREQ)
pi.set_PWM_range(pin['yaw'], PWM_RANGE)


def output_to_dc(output, direction):
    dc_min = 500
    dc_max = 1000
    dc_mid = (dc_min + dc_max) // 2
    dc_range = dc_max - dc_mid
    dc = output * dc_range * sensitivity[direction] / max_gain[direction] + dc_mid
    return dc
def pid_output(feedback, direction):
    pid[direction].update(feedback)
    if direction == 'yaw':
        output = -pid[direction].output
    else:
        output = pid[direction].output
    dc = output_to_dc(output, direction)
    pi.set_PWM_dutycycle(pin[direction], dc)
    return output, dc