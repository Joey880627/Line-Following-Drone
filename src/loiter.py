# -*- coding: utf-8 -*
import time
import optical_flow as of
import config as g
from camera import Camera

def output_roll(horizontal_shift):
    g.roll_pid.SetPoint=0
    g.roll_pid.setSampleTime(0.01)
    g.roll_pid.update(horizontal_shift)
    output = g.roll_pid.output
    dc = g.output_to_dc(output)
    return output, dc

def output_pitch(vertical_shift):
    g.pitch_pid.SetPoint=0
    g.pitch_pid.setSampleTime(0.01)
    g.pitch_pid.update(vertical_shift)
    output = g.pitch_pid.output
    dc = g.output_to_dc(output, 'pitch')
    return output, dc

if __name__ == '__main__':
    # 連接攝影機
    cam = Camera()
    image = cam.getframe()
    flow = of.OF(image)
    while True:
        startTime = time.time()
        image = cam.getframe()
         # Optical flow
        horizontal_shift, vertical_shift = flow.get_shift(image)
        print('Horizontal shift:', horizontal_shift)
        print('Vertical shift:', vertical_shift)
        # Output
        roll_output, roll_dc = g.pid_output(horizontal_shift, 'roll')
        pitch_output, pitch_dc = g.pid_output(vertical_shift, 'pitch')
        print('Roll output', roll_output, 'Roll dc:', roll_dc)
        print('Pitch output', pitch_output, 'Pitch dc:', pitch_dc)
        
        print('time: ', time.time()-startTime)
        print()
    cam.release()

