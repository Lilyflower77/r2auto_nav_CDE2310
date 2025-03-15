import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

import numpy as np
import threading
import time
import math
import cmath

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]

stop_distance = 0.2  # distance in mm

def main_control_loop(navigationNode):
    """
    Runs your custom robot logic in a loop, while the executor 
    handles communication in a separate thread.
    """
    flag = True

    while rclpy.ok():
        # with navigationNode.lock:
        #     left_distance = navigationNode.get_left_distance()
        #     right_distance = navigationNode.get_right_distance()
        #     # We read front_distance once here for debugging
        #     front_distance = navigationNode.get_front_distance()
        #     print('-----------------------------------')
        #     print(f"Initial front_distance: {front_distance}")

        # We only enter this loop while flag is True
        while flag and rclpy.ok():
            # Move forward
            navigationNode.moveforward()

            # Sleep a bit to let the robot actually move
            time.sleep(0.2)

            # Re-check front_distance in each iteration
            with navigationNode.lock:
                front_distance = navigationNode.get_front_distance()

            print(f"Rechecking front_distance: {front_distance}")

            # If obstacle is close, stop and break
            if front_distance < stop_distance:
                navigationNode.get_logger().info('Front distance: %f' % front_distance)
                flag = False
                navigationNode.stopbot()
                break

        # Now do the rotations (only after we finish the while flag loop)
        navigationNode.rotatebot(90)
        navigationNode.rotatebot(-90)

        time.sleep(0.2) # run at 20hz to prevent the control loop from running too fast
    return

class navigationNodes(Node):
    # Publisher
    def __init__(self):
        super().__init__('navigation_nodes')
        self.lock = threading.Lock()

        # Publisher initliazation
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber initialization
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        self.occupancy_subscription = self.create_subscription(
            OccupancyGrid, 
            'map', 
            self.occ_callback, 
            qos_profile_sensor_data
        )
        self.occdata = np.array([])

        self.scan_subscription = self.create_subscription(
            LaserScan, 
            'scan', 
            self.scan_callback, 
            qos_profile_sensor_data
        )
        self.laser_range = np.array([])

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        Convert quaternion into euler angles (roll, pitch, yaw).
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def odom_callback(self, msg):
        with self.lock:
            orientation_quat = msg.pose.pose.orientation
            self.roll, self.pitch, self.yaw = self.euler_from_quaternion(
                orientation_quat.x,
                orientation_quat.y,
                orientation_quat.z,
                orientation_quat.w
            )

    def occ_callback(self, msg):
        with self.lock:
            msgdata = np.array(msg.data)
            oc2 = msgdata + 1
            # Reshape to 2D array
            self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))

    def scan_callback(self, msg):
        with self.lock:
            laser_data = np.array(msg.ranges)
            laser_data[laser_data == 0] = np.nan
            self.laser_range = laser_data

    def rotatebot(self, rot_angle):
        with self.lock:
            current_yaw = self.yaw
            c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
            target_yaw = current_yaw + math.radians(rot_angle)
            c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))

            self.c_change_dir = np.sign((c_target_yaw / c_yaw).imag)

            # Start rotating
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.c_change_dir * rotatechange
            self.publisher_.publish(twist)

        # Now busy-wait until done
        rate = 0.05  # 50 ms
        while rclpy.ok():
            with self.lock:
                current_yaw = self.yaw
                c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
                c_change = target_yaw / c_yaw
                c_dir_diff = np.sign(c_change.imag)

                if self.c_change_dir * c_dir_diff <= 0:
                    # We've crossed the target
                    break
            
            time.sleep(rate)

        # Stop rotation when done
        with self.lock:
            twist = Twist()
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    def stopbot(self):
        with self.lock:
            self.get_logger().info('In stopbot')
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    def moveforward(self):
        with self.lock:
            twist = Twist()
            twist.linear.x = speedchange
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    def get_left_distance(self):
        if self.laser_range.size == 0:
            return float('inf')
        left_data = self.laser_range[80:100]
        if left_data.size == 0 or np.all(np.isnan(left_data)):
            return float('inf')
        return np.nanmin(left_data)

    def get_right_distance(self):
        if self.laser_range.size == 0:
            return float('inf')
        right_data = self.laser_range[260:280]
        if right_data.size == 0 or np.all(np.isnan(right_data)):
            return float('inf')
        return np.nanmin(right_data)

    def get_front_distance(self):
        if self.laser_range.size == 0:
            return float('inf')
        front_data = np.concatenate((self.laser_range[0:10], self.laser_range[350:359]))
        if front_data.size == 0 or np.all(np.isnan(front_data)):
            return float('inf')
        return np.nanmin(front_data)

def main(args=None):
    rclpy.init(args=args)
    navigation_node = navigationNodes()

    # CHANGED: Use a MultiThreadedExecutor instead of manual threads
    executor = MultiThreadedExecutor(num_threads=2)  # or more threads if desired
    executor.add_node(navigation_node)

    # Start spinning the executor in its own thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)  # <-- CHANGED
    executor_thread.start()  # <-- CHANGED

    try:
        # Run your custom control loop in the main thread
        main_control_loop(navigation_node)
    except KeyboardInterrupt:
        navigation_node.get_logger().info('Keyboard interrupt detected. Shutting down...')
    finally:
        # cleanup
        navigation_node.stopbot()
        time.sleep(1)

        # Shut down the executor
        executor.shutdown()                # <-- CHANGED
        executor_thread.join()             # <-- CHANGED

        navigation_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
