

import time
import pigpio
import numpy as np

TRIGGER=23
ECHO=24

high_tick = None # global to hold high tick.
def cbfunc(gpio, level, tick):
   global high_tick
   global cms
   if level == 0: # echo line changed from high to low.
      if high_tick is not None:
         echo = pigpio.tickDiff(high_tick, tick)
         cms = (echo / 1000000.0) * 34030 / 2
         # print("echo was {} micros long ({:.1f} cms)".format(echo, cms))
   else:
      high_tick = tick

pi = pigpio.pi() # Connect to local Pi.

pi.set_mode(TRIGGER, pigpio.OUTPUT)
pi.set_mode(ECHO, pigpio.INPUT)

cb = pi.callback(ECHO, pigpio.EITHER_EDGE, cbfunc)
start = time.time()
pi.gpio_trigger(TRIGGER, 10)
time.sleep(0.1)

def get_distance():
    global last
    '''
    distances = []
    for i in range(1):
        pi.gpio_trigger(TRIGGER, 10)
        time.sleep(0.001)
        distances.append(cms)
    distance = max(distances)
    if distance > 120:
        distance = 120
    return distance
    '''
    pi.gpio_trigger(TRIGGER, 10)
    distance = cms
    if distance > 90:
        distance = 90
    if distance <= 2:
        try:
            return last
        except:
            return 2
    last = distance
    return distance

if __name__ == '__main__':
    cb = pi.callback(ECHO, pigpio.EITHER_EDGE, cbfunc)
    pi.gpio_trigger(TRIGGER, 10)
    time.sleep(0.1)
    while True:
        start = time.time()
        print('Distance: %2fcm\tTime: %6f' %(get_distance(), time.time()-start))
        # time.sleep(0.1)