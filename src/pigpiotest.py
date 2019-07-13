import time
import pigpio

# gpio 12 outputs pwm
pin = 18

pi = pigpio.pi()
pi.set_PWM_frequency(pin, 50)
pi.set_PWM_range(pin, 10000) 
pi.set_PWM_dutycycle(17, 0)

delay = 3
while True:
    dc = int(input('Please enter a number between 500 ~ 1000 : '))
    print('Change duty cycle to', dc)
    pi.set_PWM_dutycycle(pin, dc)
    '''
    i = 500
    print(i)
    pi.set_PWM_dutycycle(pin, i)
    time.sleep(delay)
    i = 750
    print(i)
    pi.set_PWM_dutycycle(pin, i)
    time.sleep(delay)'''
