# The servo motor may be chosen to tilt the payload
import time
import RPi.GPIO as GPIO

def setAngle(angle):
    if 0 <= angle <= 180:
        p.ChangeDutyCycle(angle / 18 + 2.5)
        time.sleep(2)

# Set pin numbering convention
# GPIO.setmode(GPIO.BOARD)    # number on circle
GPIO.setmode(GPIO.BCM)        # number beside GPIO

# Choose an appropriate PWM channel to be used to control the servo
servo_pin = 18

# Set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize the servo to be controlled by PWM with 50 Hz frequency
p = GPIO.PWM(servo_pin, 50)

# Set servo to 90 degrees as its starting position
p.start(7.5)   # 7.5 refers to duty cycle of 7.5%

try:
    while True:
        # p.ChangeDutyCycle(7.5)  # 90 deg position
        # time.sleep(1)  # delay 1 second
        # p.ChangeDutyCycle(2.5)  # 0 deg position
        # time.sleep(1)  # delay 1 second again
        # p.ChangeDutyCycle(12.5)  # 180 deg position
        # time.sleep(1)  # delay 1 second again ... ...

        setAngle(90)
        setAngle(270)  # This will give p.ChangeDutyCycle(17.5) which 
                       # is invalid thus the servo wouldn't move from this command
        setAngle(0)

except KeyboardInterrupt:
    p.stop()  # stop PWM
    GPIO.cleanup()  # remove all setup
