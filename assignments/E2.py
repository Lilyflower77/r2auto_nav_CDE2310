import time
import RPi.GPIO as GPIO


# GPIO.setmode(GPIO.BOARD)    # number on circle
GPIO.setmode(GPIO.BCM)  # number beside GPIO
servo_pin = 21
GPIO.setup(servo_pin, GPIO.OUT)

def setPin(userInput):
    if userInput == True:
        GPIO.output(servo_pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(servo_pin, GPIO.LOW)
    else: 
        print("Not launching")
try:
    while True:
        userInput = bool(input("Do you want to launch?"))
        GPIO.output(servo_pin, GPIO.LOW)
        setPin(userInput)
except KeyboardInterrupt:
    GPIO.cleanup()  # remove all setup