import time

import RPi.GPIO as GPIO



# Next we setup the pins for use!

GPIO.setmode(GPIO.BCM)

GPIO.setwarnings(False)

GPIO.setup(17,GPIO.OUT)

GPIO.setup(18,GPIO.OUT)

PWM_FREQ = 50

pwm1 = GPIO.PWM(17, PWM_FREQ)
pwm2 = GPIO.PWM(18, PWM_FREQ)

pwm1.start(0)
pwm2.start(0)


print('Starting motor sequence!')



while True:

  try:

    # Makes the motor spin one way for 3 seconds

    pwm1.ChangeDutyCycle(30)

    pwm2.ChangeDutyCycle(0)

    # GPIO.output(17, True)

    # GPIO.output(18, False)

    time.sleep(2)
    pwm1.ChangeDutyCycle(60)
    pwm2.ChangeDutyCycle(0)
    time.sleep(2)
    print('turn')
    # Spins the other way for a further 3 seconds

    pwm1.ChangeDutyCycle(-50)

    pwm2.ChangeDutyCycle(0)

    #GPIO.output(17, False)

    #GPIO.output(18, True)

    time.sleep(2)

  except(KeyboardInterrupt):

    # If a keyboard interrupt is detected then it exits cleanly!

    print('Finishing up!')

    GPIO.output(17, False)

    GPIO.output(18, False)

    quit()
