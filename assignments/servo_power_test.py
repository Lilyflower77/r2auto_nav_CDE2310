import RPi.GPIO as GPIO
import time

# GPIO Setup
GPIO.setmode(GPIO.BCM)
servo_pin = 18
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize PWM
p = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
p.start(2.5)  # Start at minimum position

# Define servo movement limits
MIN_DUTY_CYCLE = 2.5  # 0 degrees
MAX_DUTY_CYCLE = 12.5  # 180 degrees
STEP_SIZE = 0.5  # Step increment for duty cycle
STEP_DELAY = 0.1  # Delay between each step

try:
    while True:
        # Move from min to max
        for duty in np.arange(MIN_DUTY_CYCLE, MAX_DUTY_CYCLE + STEP_SIZE, STEP_SIZE):
            p.ChangeDutyCycle(duty)
            time.sleep(STEP_DELAY)
        
        time.sleep(0.5)  # Pause at max position

        # Move from max to min
        for duty in np.arange(MAX_DUTY_CYCLE, MIN_DUTY_CYCLE - STEP_SIZE, -STEP_SIZE):
            p.ChangeDutyCycle(duty)
            time.sleep(STEP_DELAY)
        
        time.sleep(0.5)  # Pause at min position

except KeyboardInterrupt:
    print("Stopping...")
    p.stop()
    GPIO.cleanup()
