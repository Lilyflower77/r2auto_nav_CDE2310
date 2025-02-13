# EG2310
# This code is used to generate a pwm signal

import time
import RPi.GPIO as GPIO

# Set pin numbering convention
# Can't use both, please choose one and comment away the other
GPIO.setmode(GPIO.BCM)
#GPIO.setmode(GPIO.BOARD)

# Set the pin to generate the pwm signal
# This pin needs to be one that is on a PWM channel
test_point = 18

# Set the pin as an output
GPIO.setup(test_point , GPIO.OUT)

# Initialise pwm object with 1 kHz frequency
pwm = GPIO.PWM(test_point, 50)
pwm.start(0)

# Begin pwm experiment

print("Start")

for i in range(0, 100, 1):
    pwm.ChangeDutyCycle(i)
    print("Brightness is ", i, "%")
    time.sleep(0.2)
else:
    print("Finished")

# End the script and exit
pwm.stop()
GPIO.cleanup()