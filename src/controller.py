# -*- coding: utf-8 -*
import time
import line_follow as lf
import color_recognization as cr
import optical_flow as of
import config as g
from camera import Camera

if __name__ == '__main__':
    # 連接攝影機
    cam = Camera()
    image = cam.getframe()
    flow = of.OF(image)
    while True:
        startTime = time.time()
        image = cam.getframe()
        image_color = cr.image_color(image)
        # Color recognization
        if image_color:
            print(image_color)
            
        # Output throttle
        
        # optical flow (pitch and roll feedback)
        horizontal_shift, vertical_shift = flow.get_shift(image)
        print('Horizontal shift:', horizontal_shift)
        print('Vertical shift:', vertical_shift)
        # Output pitch
        # pitch_output, pitch_dc = g.pid_output(vertical_shift, 'pitch')
        # Output roll
        # roll_output, roll_dc = g.pid_output(horizontal_shift, 'roll')
        # print('Roll output', roll_output, 'Roll dc:', roll_dc)
        # print('Pitch output', pitch_output, 'Pitch dc:', pitch_dc)
        
        # Yaw feedback
        feedback = lf.get_shift(image)
        # Output yaw
        yaw_output, yaw_dc = g.pid_output(feedback, 'yaw')
        print('Yaw output', yaw_output, 'Yaw dc:', yaw_dc)
        
        
        print('time: ', time.time()-startTime)
        print()
        # time.sleep(0.5)
    cam.release()
