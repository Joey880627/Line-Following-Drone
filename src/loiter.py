# -*- coding: utf-8 -*
import time
import optical_flow as of
import find_circle as fc
import config as g
from camera import Camera

if __name__ == '__main__':
    # 連接攝影機
    cam = Camera()
    image = cam.getframe()
    flow = of.OF(image)
    ct = fc.Circle_tracking()
    while True:
        startTime = time.time()
        image = cam.getframe()
         # Optical flow
        # horizontal_shift, vertical_shift = flow.get_shift(image)
        # Circle
        horizontal_shift, vertical_shift = ct.get_shift(image)
        
        print('Horizontal shift:', horizontal_shift)
        print('Vertical shift:', vertical_shift)
        # Output
        roll_output, roll_dc = g.pid_output(horizontal_shift, 'roll')
        time.sleep(0.02)
        pitch_output, pitch_dc = g.pid_output(vertical_shift, 'pitch')
        print('Roll output', roll_output, 'Roll dc:', roll_dc)
        print('Pitch output', pitch_output, 'Pitch dc:', pitch_dc)
        
        print('time: ', time.time()-startTime)
        # time.sleep(0.3)
        print()
    cam.release()

