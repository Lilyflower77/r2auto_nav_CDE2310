import time
import RPi.GPIO as GPIO

# GPIO Setup
GPIO.setmode(GPIO.BCM)
servo_pin = 18
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize PWM
p = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
p.start(7.5)  # 7.5% duty cycle -> 90 degrees

def setAngle(angle):
    """Moves the servo to a given angle."""
    global p  # Ensure `p` is accessible
    if 0 <= angle <= 180:
        duty = angle / 18 + 2.5
        p.ChangeDutyCycle(duty)
        time.sleep(0.5)  # Allow servo time to reach position
        p.ChangeDutyCycle(0)  # Reset PWM to prevent jitter
    else:
        print("Invalid angle! Must be between 0 and 180 degrees.")

try:
    while True:
        input_angle = int(input("Enter angle (0-180): "))
        setAngle(input_angle)

except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
    print("\nServo control stopped, GPIO cleaned up.")
