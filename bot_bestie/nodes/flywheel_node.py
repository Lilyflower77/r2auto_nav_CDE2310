#flywheel = receive "50"
#flywheel_status = "Flywheel sequence complete"
#use those 2 topics
from std_msgs.msg import String
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
from gpiozero import Servo

class FlywheelSubscriber(Node):

    def __init__(self):
        super().__init__('flywheel_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'flywheel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'flywheel_status', 10)
        self.subscription  # prevent unused variable warning

        # GPIO Setup
        self.ESC_PIN = 18
        self.SERVO_PIN = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ESC_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.ESC_PIN, 50)  # 50Hz for ESC
        self.pwm.start(0)
        self.my_servo = Servo(self.SERVO_PIN, min_pulse_width=0.9/1000, max_pulse_width=2.1/1000, frame_width=20/1000)
        self.my_servo.value = -0.26
        print("should be not moving")
        duty_cycle = 30*0.1
        self.pwm.ChangeDutyCycle(duty_cycle)
    def listener_callback(self, msg):
        def calibrate_stop():
            self.my_servo.value = -0.26  # Stop (neutral)
            print("Calibrating Stop Position...")

        def move_forward():
            self.my_servo.value = 1  # Full forward
            print("Moving Forward...")

        def move_reverse():
            self.my_servo.value = -1  # Full reverse
            print("Moving Reverse...")
        throttle = msg.data
        if throttle == 50:
                #move_reverse()
                #sleep(0.8)
                duty_cycle = throttle*0.1
                self.pwm.ChangeDutyCycle(duty_cycle)
                self.get_logger().info(f'Throttle set to {throttle}%')
                sleep(2)
                move_forward()
                sleep(0.85)
                #first ball shot
                calibrate_stop()
                sleep(0.5)
                move_reverse()
                sleep(0.8)
                calibrate_stop()
                sleep(1.85)
                move_forward()
                sleep(0.85)
                #second ball shot
                calibrate_stop()
                sleep(0.2)
                move_reverse()
                sleep(0.8)
                calibrate_stop()
                sleep(0.15)
                move_forward()
                sleep(0.85)
                #third ball shot
                calibrate_stop()
                sleep(0.5)
                move_reverse()
                sleep(0.8)
                calibrate_stop()
                sleep(0.5)
                #send message to say this shit done
                duty_cycle = 0
                throttle = 0
                self.pwm.ChangeDutyCycle(duty_cycle)
                self.get_logger().info(f'Throttle set to {throttle}%')
                done_msg = String()
                done_msg.data = "Flywheel sequence complete"
                self.publisher_.publish(done_msg)
                self.get_logger().info("Published status: Flywheel sequence complete")

        else:
            self.get_logger().warn('Throttle out of range)')

        def destroy_node(self):
                self.pwm.stop()
                GPIO.cleanup()
                super().destroy_node()
def main(args=None):
    rclpy.init(args=args)
    flywheel_subscriber = FlywheelSubscriber()

    try:
        rclpy.spin(flywheel_subscriber)
    except KeyboardInterrupt:
        pass

    flywheel_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()