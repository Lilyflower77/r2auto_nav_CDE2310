import rclpy
from rclpy.node import Node, ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState, ChangeState
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from enum import Enum, auto
import concurrent.futures
import time
import threading
import numpy as np
import tf2_ros


class GlobalController(Node):
    """
    GlobalController:
    - Handles high-level state management
    - Sends goals to the planner
    - Monitors state feedback
    - Manages lifecycle transitions
    - Provides hooks for future expansion
    """
    # different states the bot can take on
    class State(Enum):
        Initializing = auto()
        Exploratory_Mapping = auto()
        Goal_Navigation = auto()
        Launching_Balls = auto()
        Imu_Interrupt = auto()
        Attempting_Ramp = auto()
        Returning_Home = auto()

    def __init__(self):
        super().__init__('global_controller')

        ## initialize all publishers and subscribers

        # odom
        self.odom_subscription = self.create_subscription(
        Odometry,
        'odom',
        self.odom_callback,
        10)

        # occupancy grid
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])

        # temperature sensors
        self.left_temperature = self.create_subscription(
            Float32MultiArray,
            'temperature_sensor_1',
            self.sensor1_callback,
            10)
        self.right_temperature = self.create_subscription(
            Float32MultiArray,
            'temperature_sensor_2',
            self.sensor2_callback,
            10)
        
        # IMU subscription
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Allow for global positioning 
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Temperature Attributes
        self.temp_and_location_data = []  # [[x, y], left_temp, right_temp]
        self.latest_left_temp = None
        self.latest_right_temp = None

        # IMU Attributes stored as (timestamp, pitch)
        self.pitch_window = []
        
        # logic attributes
        self.state = GlobalController.State.Initializing
        self.ball_launches_attempted = 0
        self.max_heat_locations = [None] * 5
        self.ramp_location = [None]

        # Multi Threading functionality
        self.lock = threading.Lock()
        # Triggers the fast loop at 10hz
        self.fast_timer = self.create_timer(0.1, self.fast_loop)
        # Triggers the control loop at 1hz
        self.control_loop_timer = self.create_timer(1.0, self.control_loop)
        # ✅ Thread pool for heavy background tasks
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)


        self.get_logger().info('Global Controller Initialized, changing state to Exploratory Mapping')
        self.set_state(GlobalController.State.Exploratory_Mapping)

    ## Callback handers for temperature sensors
    def sensor1_callback(self, msg):
        if msg.data:
            with self.lock:
                self.latest_left_temp = msg.data[0]

    def sensor2_callback(self, msg):
        if msg.data:
            with self.lock:
                self.latest_right_temp = msg.data[0]

    ## callback handler for IMU
    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        now = self.get_clock().now().nanoseconds / 1e9  # seconds
        with self.lock:
            self.pitch_window.append((now, pitch))

            # Keep only the last 0.5 seconds of data
            self.pitch_window = [
                (t, p) for t, p in self.pitch_window if now - t <= 0.5
            ]

    # def odom_callback(self, msg): ## not needed for now
    #     x = msg.pose.pose.position.x
    #     y = msg.pose.pose.position.y
    #     #self.get_logger().info(f"Odometry: x={x}, y={y}")

    # =======================
    # Thread safe State Management
    # =======================
    
    def set_state(self, new_state):
        """Thread-safe state setter"""
        with self.lock:
            self.state = new_state
            self.get_logger().info(f"State changed to: {self.state}")

    def get_state(self):
        """Thread-safe state getter"""
        with self.lock:
            return self.state
    
    def get_robot_global_position(self):
        try:
            now = rclpy.time.Time()
            trans = self.tfBuffer.lookup_transform(
                'map',         # target frame (global)
                'base_link',   # source frame (robot)
                rclpy.time.Time())
            
            x = trans.transform.translation.x
            y = trans.transform.translation.y

            # Convert quaternion to yaw
            rot = trans.transform.rotation
            quat = [rot.x, rot.y, rot.z, rot.w]
            (_, _, yaw) = euler_from_quaternion(quat)

            return (x, y, yaw)

        except Exception as e:
            self.get_logger().warn(f"[TF] Failed to get global robot position: {e}")
            return None

    def log_temperature(self):
        position = self.get_robot_global_position()
        if position is None:
            return  # Skip if tf lookup failed

        with self.lock:
            if self.latest_left_temp is not None and self.latest_right_temp is not None:
                self.temp_and_location_data.append([
                    position,
                    self.latest_left_temp,
                    self.latest_right_temp
                ])
    

    # =======================
    # Fast Loop (10 Hz) – Sensor Polling
    # =======================

    def fast_loop(self):
        """
        High-frequency loop (10 Hz) for real-time monitoring.
        This should NEVER block.
        """
        bot_current_state = self.get_state()
        if bot_current_state == GlobalController.State.Imu_Interrupt:
            self.get_logger().info("IMU Interrupt detected, doing appropriate action")
            pass
        elif bot_current_state == GlobalController.State.Exploratory_Mapping:
            self.get_logger().info("Exploratory Mapping...")
            ## TODO: 
            ## IMU interrupt checking
            self.log_temperature()
            pass
        elif bot_current_state == GlobalController.State.Goal_Navigation:
            ## IMU interrupt checking
            pass
        elif bot_current_state == GlobalController.State.Launching_Balls:
            ## do nothing, waiting on controller to change state, this state should be idle
            pass
        elif bot_current_state == GlobalController.State.Attempting_Ramp:
            ## check for ramp using IMU Data (potentially), poll for when IMU is flat, so there is no pitch meaning the top of the remp has been reached
            pass





    # =======================
    # ✅ Control Loop (1 Hz) – Decision Making
    # =======================

    def control_loop(self):
        """Slower decision-making loop (1 Hz)"""
        current_state = self.get_state()

        if current_state == 'INITIALIZING':
            self.initialize_system()
        elif current_state == 'ACTIVE':
            self.update_goal_status()
        elif current_state == 'RECOVERING':
            self.recover_behavior()
        elif current_state == 'IDLE':
            self.get_logger().info("Waiting for new task...")
        elif current_state == 'ERROR':
            self.get_logger().error("System in ERROR state!")

    def initialize_system(self):
        """ Placeholder init logic """
        self.get_logger().info("System initializing...")
        time.sleep(1.0)
        self.set_state('ACTIVE')

    def update_goal_status(self):
        """ Placeholder active logic """
        self.get_logger().info("Goal is active and being monitored.")

    def recover_behavior(self):
        """ Recovery logic example """
        self.get_logger().info("Attempting recovery...")
        time.sleep(1.0)
        self.set_state('IDLE')



def main(args=None):
    rclpy.init(args=args)

    # ✅ Use MultiThreadedExecutor to support timers + background tasks
    executor = MultiThreadedExecutor(num_threads=3)

    global_controller = GlobalController()
    executor.add_node(global_controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        global_controller.get_logger().info("Shutting down...")
    finally:
        global_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
