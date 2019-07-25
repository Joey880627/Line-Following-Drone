import time
import pigpio

# gpio 12 outputs pwm
pin = 18

pi = pigpio.pi()
pi.set_PWM_frequency(pin, 50)
pi.set_PWM_range(pin, 10000) 
pi.set_PWM_dutycycle(pin, 0)

delay = 3
while True:
    print('Enter -1 to shutdown')
    dc = eval(input('Please enter a number between 500 ~ 1000 : '))
    if dc == -1:
        pi.set_PWM_dutycycle(pin, 0)
        break
    '''if (dc > 1000) | (dc < 500):
        print()
        continue'''
    print('Change duty cycle to', dc)
    pi.set_PWM_dutycycle(pin, dc)
    print()
