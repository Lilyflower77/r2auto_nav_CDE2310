import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np 
import threading
import time
import math
import cmath

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]


# Look at these constants! You have to use these for your project 
stop_distance = 0.5


def main_control_loop(navigationNodes):

    
    while rclpy.ok():
        with navigationNodes.lock: # to prevent race condition
            ############################   WRITE YUR CODE HERE   ############################
            
            # Get the distance to the left wall
            left_distance = navigationNodes.get_left_distance()
            # Get the distance to the right wall
            right_distance = navigationNodes.get_right_distance()
            # Get the distance to the front wall
            front_distance = navigationNodes.get_front_distance()
            print('Left distance: ', left_distance)
            print('Right distance: ', right_distance)
            print('Front distance: ', front_distance)

            print('-----------------------------------') 
            navigationNodes.moveforward()
            # if the distance to the front wall is less than the stop distance, stop the robot
            if front_distance < stop_distance:
                navigationNodes.stopbot()

            navigationNodes.rotatebot(90)
            navigationNodes.rotatebot(-90)

    time.sleep(0.1) # run at 10hz to prevent the control loop from running too fast
    return 



class navigationNodes(Node):
    # Publisher
    def __init__(self):
        super().__init__('navigationNodes')
        self.lock = threading.Lock()

        # Publisher initliazation
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber initialization
        self.odom_subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        self.occupancy_subscription = self.create_subscription(OccupancyGrid, 'map', self.occupancy_callback, qos_profile_sensor_data)
        self.occdata = np.array([])       
        self.scan_subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)
        self.laser_range = np.array([])

    @staticmethod
    def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
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

        return roll_x, pitch_y, yaw_z # in radians

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = self.euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def rotatebot(self, rot_angle):
        self.get_logger().info('In rotatebot')

        # Get current yaw angle
        current_yaw = self.yaw
        c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))

        # Calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        c_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))

        # Set the rotation direction based on shortest path
        c_change_dir = np.sign((c_target_yaw / c_yaw).imag)

        # Start rotating
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = c_change_dir * rotatechange
        self.publisher_.publish(twist)

        self.get_logger().info(f'Starting rotation to target: {math.degrees(math.atan2(c_target_yaw.imag, c_target_yaw.real))} degrees')

        # Blocking loop until the target yaw is reached
        while True:
            # Allow callbacks to update the yaw value
            rclpy.spin_once(self)

            # Get current yaw
            current_yaw = self.yaw
            c_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))

            # Get difference between target and current yaw
            c_change = c_target_yaw / c_yaw
            c_dir_diff = np.sign(c_change.imag)

            self.get_logger().info(f'Current Yaw: {math.degrees(current_yaw)} degrees')

            # Stop when the rotation direction reverses (meaning target is reached)
            if c_change_dir * c_dir_diff <= 0:
                self.get_logger().info('Rotation complete')
                break

        # Stop the rotation
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

        self.get_logger().info('Rotation stopped')


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def moveforward(self):
        self.get_logger().info('In moveforward')
        # create Twist object
        twist = Twist()
        # set linear speed
        twist.linear.x = speedchange
        # set angular speed
        twist.angular.z = 0.0
        # publish to cmd_vel to move TurtleBot
        self.publisher_.publish(twist)

    def get_left_distance(self):
        # get the distance to the left wall
        left_distance = np.nanmin(self.laser_range[80:100])
        return left_distance
    
    def get_right_distance(self):
        # get the distance to the right wall
        right_distance = np.nanmin(self.laser_range[260:280])
        return right_distance
    
    def get_front_distance(self):
        # get the distance to the front wall
        front_distance = np.nanmin(self.laser_range[0:10] + self.laser_range[350:359])
        return front_distance



def main(args=None):
    # initializing ROS2
    rclpy.init(args=args)
    # initializing the node
    navigationNode = navigationNodes()

    # Start ROS commuinication in its own thread to avoid blocking
    ros_thread = threading.Thread(target=rclpy.spin, args=(navigationNode,), daemon=True)
    ros_thread.start() # starts the thread

    # Start the main control loop
    control_thread = threading.Thread(target=main_control_loop, daemon=True)
    control_thread.start() # starts the thread

    try:
        ros_thread.join() # neither of these threads should ever finish
        control_thread.join()
    except KeyboardInterrupt:
        navigationNode.get_logger().info('Keyboard interrupt detected. Shutting down...')
    finally:
        # cleanup
        navigationNode.destroy_node()



    # when the garbage collector destroys the node object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
