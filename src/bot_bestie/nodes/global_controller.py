import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState, ChangeState
import concurrent.futures
import time

class GlobalController(Node):
    """
    GlobalController:
    - Handles high-level state management
    - Sends goals to the planner
    - Monitors state feedback
    - Manages lifecycle transitions
    - Provides hooks for future expansion
    """

    def __init__(self):
        super().__init__('global_controller')

        # ✅ State Variables
        self.state = 'INITIALIZING'
        self.goal_active = False
        self.lock = threading.Lock()

        # ✅ High-frequency loop (10 Hz)
        self.fast_timer = self.create_timer(0.1, self.fast_loop)

        # ✅ Goal publisher (to planner)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ✅ State feedback subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # ✅ Lifecycle service clients
        self.get_state_client = self.create_client(GetState, '/planner_server/get_state')
        self.change_state_client = self.create_client(ChangeState, '/planner_server/change_state')

        # ✅ State manager loop (slower control loop)
        self.control_loop_timer = self.create_timer(1.0, self.control_loop)

        # ✅ Thread pool for heavy tasks
        self.executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)

        self.get_logger().info('Global Controller Initialized')

    # =======================
    # ✅ State Management
    # =======================
    
    def set_state(self, new_state):
        """
        Updates the current state.
        Thread-safe using lock.
        """
        with self.lock:
            self.state = new_state
            self.get_logger().info(f"State changed to: {self.state}")

    def get_state(self):
        """
        Returns the current state.
        Thread-safe using lock.
        """
        with self.lock:
            return self.state
    
    # =======================
    # ✅ Fast Loop (10 Hz) – Sensor Polling
    # =======================

    def fast_loop(self):
        """
        High-frequency loop (10 Hz) for real-time monitoring.
        This should NEVER block.
        """
        # Example: Poll for temperature data
        temperature = self.get_temperature_data()
        self.get_logger().info(f"Temperature = {temperature:.2f}°C")

        # Example: Poll for IMU interrupt
        imu_interrupt = self.check_imu_interrupt()
        if imu_interrupt:
            self.get_logger().info("IMU Interrupt detected — logging data.")
            self.executor.submit(self.log_sensor_data, temperature, imu_interrupt)

    def get_temperature_data(self):
        """ Simulated temperature reading """
        return 25.0 + (0.5 - time.time() % 1)  # Simulated fluctuation

    def check_imu_interrupt(self):
        """ Simulated IMU check """
        return time.time() % 5 < 0.1  # Trigger every 5 seconds

    def log_sensor_data(self, temperature, imu_interrupt):
        """ Blocking helper function for logging sensor data """
        time.sleep(2)  # Simulate logging time
        self.get_logger().info(f"Logged Temp={temperature:.2f}, IMU={imu_interrupt}")

    # =======================
    # ✅ Control Loop (1 Hz) – Decision Making
    # =======================

    def control_loop(self):
        """
        Lower-frequency loop for state management and goal setting.
        This can handle blocking functions.
        """
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

    def recover_behavior(self):
        """ Recovery logic example """
        self.get_logger().info("Attempting recovery...")
        time.sleep(1.0)
        self.set_state('IDLE')

    # =======================
    # ✅ Feedback Handling
    # =======================

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"Odometry: x={x}, y={y}")

def main(args=None):
    rclpy.init(args=args)

    # ✅ Multithreaded for fast loop + control loop
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
