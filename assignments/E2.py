import time
import RPi.GPIO as GPIO


# GPIO.setmode(GPIO.BOARD)    # number on circle
GPIO.setmode(GPIO.BCM)  # number beside GPIO
servo_pin = 40
GPIO.setup(servo_pin, GPIO.OUT)

def setPin(userInput):
    if userInput == 1:
        GPIO.output(servo_pin, GPIO.HIGH)
    else :
        GPIO.output(servo_pin, GPIO.LOW)

try:
    while True:
        userInput = bool(input("Do you want to launch?"))
except KeyboardInterrupt:
    GPIO.cleanup()  # remove all setup