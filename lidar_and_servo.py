import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import RPi.GPIO as GPIO


# GPIO Setup
# GPIO.setmode(GPIO.BOARD)    # number on circle
GPIO.setmode(GPIO.BCM)  # number beside GPIO
servo_pin = 18
solenoid_pin = 21
GPIO.setup(servo_pin, GPIO.OUT)
GPIO.setup(solenoid_pin, GPIO.OUT)

# Initialize PWM
p = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
p.start(7.5)  # 7.5% duty cycle -> 90 degrees

# define constants: 
angle = 45

class Scanner(Node):

    def __init__(self):
        super().__init__('scanner')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # create numpy array
        laser_range = np.array(msg.ranges)
        # replace 0's with nan
        laser_range[laser_range == 0] = np.nan
        
        # Get shortest distance in the range of 0 to 5 degrees
        shortest_distance_front = self.get_shortest_distance_front(laser_range, msg.angle_min, msg.angle_increment)

        # find index with minimum value
        lr2i = np.nanargmin(laser_range)
        lr2i2 = lr2i * (360 / 230)
        # log the info
        self.get_logger().info('Shortest distance at %i degrees' % lr2i2)
        if shortest_distance_front is not None:
            self.get_logger().info(f'Shortest distance in 0-5 degrees: {shortest_distance_front:.3f} meters')
            if shortest_distance_front < 1.0:
                launch_solenoid()

    def get_shortest_distance_front(self, laser_range, angle_min, angle_increment):
        # Convert angle range to indices
        start_angle = 0  # 0 degrees
        end_angle = 5  # 5 degrees
        
        start_index = int((start_angle - angle_min) / angle_increment)
        end_index = int((end_angle - angle_min) / angle_increment)
        
        # Extract relevant slice and find the minimum
        relevant_ranges = laser_range[start_index:end_index + 1]
        return np.nanmin(relevant_ranges) if np.any(~np.isnan(relevant_ranges)) else None

    def handle_obstacle(self):
        self.get_logger().warn("Obstacle detected within 1 meter in the front!")


def main(args=None):
    rclpy.init(args=args)

    scanner = Scanner()

    try:
        rclpy.spin(scanner)
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Cleaning up...")
    finally:
        scanner.destroy_node()
        rclpy.shutdown()
        cleanup_gpio(None, None)

def launch_solenoid():
        duty = angle / 18 + 2.5
        p.ChangeDutyCycle(duty)
        time.sleep(0.5)  # Allow servo time to reach position
        p.ChangeDutyCycle(2.5)  # Reset PWM to prevent jitter
        time.sleep(0.5)

        # Launch solenoid
        GPIO.output(solenoid_pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(solenoid_pin, GPIO.LOW)

def cleanup_gpio(signal, frame):
    print("Cleaning up GPIO...")
    p.stop()
    GPIO.cleanup()


if __name__ == '__main__':
    main()
