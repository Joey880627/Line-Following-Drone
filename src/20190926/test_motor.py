import time

import RPi.GPIO as GPIO



# Next we setup the pins for use!

GPIO.setmode(GPIO.BCM)

GPIO.setwarnings(False)

GPIO.setup(20,GPIO.OUT)

GPIO.setup(21,GPIO.OUT)

PWM_FREQ = 50

pwm1 = GPIO.PWM(20, PWM_FREQ)
pwm2 = GPIO.PWM(21, PWM_FREQ)

pwm1.start(0)
pwm2.start(0)

while True:

  try:
    signal = int(input())
    if (signal > 100) | (signal < -100):
        continue
    if signal > 0:
        pwm1.ChangeDutyCycle(signal)

        pwm2.ChangeDutyCycle(0)
    else:
        pwm1.ChangeDutyCycle(0)

        pwm2.ChangeDutyCycle(-signal)

  except(KeyboardInterrupt):

    # If a keyboard interrupt is detected then it exits cleanly!

    print('Finishing up!')

    GPIO.output(20, False)

    GPIO.output(21, False)

    quit()
