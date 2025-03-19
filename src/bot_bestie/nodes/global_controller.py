import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from lifecycle_msgs.srv import GetState, ChangeState
import threading
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
        self.timer = self.create_timer(0.1, self.control_loop)

        # ✅ Goal publisher (to planner)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # ✅ State feedback subscriber
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # ✅ Lifecycle service clients
        self.get_state_client = self.create_client(GetState, '/planner_server/get_state')
        self.change_state_client = self.create_client(ChangeState, '/planner_server/change_state')

        # ✅ Thread for map polling (example of parallel execution)
        self.polling_thread = threading.Thread(target=self.poll_map, daemon=True)
        self.polling_thread.start()

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
    # ✅ High-Frequency Control Loop
    # =======================

    def control_loop(self):
        """
        High-frequency control loop.
        Runs at 10Hz.
        Handles high-level logic based on the state.
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

    # =======================
    # ✅ Feedback Callbacks
    # =======================

    def odom_callback(self, msg):
        """
        Receives feedback from odometry.
        Example of reading real-time feedback.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f"Odometry: x={x}, y={y}")

    # =======================
    # ✅ Goal Handling
    # =======================

    def send_goal(self, x, y):
        """
        Sends a goal to the planner.
        """
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.set_state('ACTIVE')
        self.get_logger().info(f"Goal sent to x={x}, y={y}")

    def update_goal_status(self):
        """
        Checks the status of the goal.
        Example for future logic to handle goal completion.
        """
        if self.goal_active:
            self.get_logger().info("Goal is still active...")

    # =======================
    # ✅ Lifecycle Handling
    # =======================

    def get_lifecycle_state(self, node_name):
        """
        Gets the lifecycle state of a node.
        """
        if not self.get_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Lifecycle service for {node_name} not available.")
            return None

        request = GetState.Request()
        request.node_name = node_name
        future = self.get_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            return future.result().current_state.label
        return None

    # =======================
    # ✅ Parallel Execution: Polling Example
    # =======================

    def poll_map(self):
        """
        Polls the map at 2 Hz.
        Example for continuous data processing.
        """
        while rclpy.ok():
            # Example placeholder for map polling
            self.get_logger().info("Polling map data...")
            time.sleep(0.5)

    # =======================
    # ✅ Recovery Behavior Example
    # =======================

    def recover_behavior(self):
        """
        Example for recovery logic.
        """
        self.get_logger().info("Attempting recovery...")
        time.sleep(1.0)  # Example delay for recovery
        self.set_state('IDLE')

def main(args=None):
    rclpy.init(args=args)

    # ✅ Use MultithreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)

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
