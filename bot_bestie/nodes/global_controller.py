import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped ,Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray, Int32
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState, ChangeState
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
from enum import Enum, auto
from collections import deque
import concurrent.futures
import time
import threading
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math
from collections import deque
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


# constants
rotatechange = 0.15 # was 0.1
stop_distance = 0.25 # distance to stop in front of heat source

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
        Go_to_Heat_Souce = auto()

    def __init__(self):
        super().__init__('global_controller')
        ## initialize all publishers and subscribers

        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # odom
        self.odom_subscription = self.create_subscription(
        Odometry,
        'odom',
        self.odom_callback,
        10)

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        )

        # occupancy grid
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            map_qos)
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
        try: 
            self.tfBuffer = tf2_ros.Buffer()
            self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
            self.get_logger().info("TF listener created")
        except Exception as e :
            self.get_logger().error("TF listener failed: %s" % str(e))

        
        # Ball launcher
        self.flywheel_publisher = self.create_publisher(
            Int32, 
            'flywheel', 
            10)


        # Temperature Attributes
        self.temp_and_location_data = []  # [[x, y], left_temp, right_temp]
        self.latest_left_temp = None
        self.latest_right_temp = None

        # IMU Attributes stored as (timestamp, pitch)
        self.pitch_window = deque()
        # For global moving average
        self.global_pitch_sum = 0.0
        self.global_pitch_count = 0
        # For recent average (last 0.3s)
        self.recent_pitch_avg = 0.0
        self.global_pitch_avg = 0.0
        #occ map variables
        self.padding = 3
        self.confirming_unknowns = False
        
        # logic attributes
        self.state = GlobalController.State.Initializing
        self.previous_state = None
        self.ball_launches_attempted = 0
        self.max_heat_locations = [None] * 3
        self.ramp_location = [None]
        self.finished_mapping = False
        self.goal_active = False
        self.just_reached_goal = False

        self.occ_callback_called = False
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 Action Server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 Action Server available. Ready to send goals.")
        self.visited_frontiers = set()

        # Multi Threading functionality
        self.lock = threading.Lock()
        # Triggers the fast loop at 10hz
        self.fast_timer = self.create_timer(0.1, self.fast_loop)
        # Triggers the control loop at 1hz
        self.control_loop_timer = self.create_timer(1.0, self.control_loop)
        # ✅ Thread pool for heavy background tasks
        self.thread_pool = concurrent.futures.ThreadPoolExecutor(max_workers=2)


        self.get_logger().info('Global Controller Initialized, changing state to Exploratory Mapping')
        self.set_state(GlobalController.State.Initializing)

    ## Callback handers for temperature sensors
    def sensor1_callback(self, msg):
        if msg.data and len(msg.data) == 64:
            indices = [
            18, 19, 20, 21,
            26, 27, 28, 29,
            34, 35, 36, 37,
            42, 43, 44, 45
        ]
            center_values = [msg.data[i] for i in indices]
            self.latest_left_temp = sum(center_values) / len(center_values)

    def sensor2_callback(self, msg):
        if msg.data and len(msg.data) == 64:
            indices = [
            18, 19, 20, 21,
            26, 27, 28, 29,
            34, 35, 36, 37,
            42, 43, 44, 45
        ]
            center_values = [msg.data[i] for i in indices]
            self.latest_right_temp = sum(center_values) / len(center_values)

    ## method to launch balls
    def launch_ball(self):
        msg = Int32()
        msg.data = 50
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % msg.data)
        self.ball_launches_attempted += 1
        self.get_logger().info(f"Ball launches attempted: {self.ball_launches_attempted}")

    ## callback handler for IMU
    def imu_callback(self, msg):
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, pitch, _ = euler_from_quaternion(quat)

        now = self.get_clock().now().nanoseconds / 1e9  # seconds

        with self.lock:
            # Append latest value
            self.pitch_window.append((now, abs(pitch)))  # use abs to ignore direction

            # Remove old entries beyond 0.5s
            while self.pitch_window and now - self.pitch_window[0][0] > 0.5:
                self.pitch_window.popleft()

            # Update global moving average
            self.global_pitch_sum += abs(pitch)
            self.global_pitch_count += 1
            self.global_pitch_avg = self.global_pitch_sum / self.global_pitch_count

            # Compute recent average for last 0.3s
            recent_values = [p for t, p in self.pitch_window if now - t <= 0.3]
            if recent_values:
                self.recent_pitch_avg = sum(recent_values) / len(recent_values)    


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion([orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])


    def occ_callback(self, msg):
        #self.get_logger().info('In occ_callback - Updating Map Metadata')
        # Store map metadata
        self.map_resolution = msg.info.resolution
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        # create numpy array
        msgdata = np.array(msg.data)

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        #self.get_logger().info(f"Unique values in occupancy grid: {np.unique(self.occdata)}")

        for node in self.visited_frontiers:
            if(self.occdata[node[1], node[0]] != 101):
                self.occdata[node[1], node[0]] = 1
        
        
        height, width = self.occdata.shape

        # Create a copy to store expanded obstacles
        expanded_occdata = self.occdata.copy()
        self.original_occdata = self.occdata.copy()

        for y in range(height):
            for x in range(width):
                if self.occdata[y, x] == 101:  # Only use the original grid
                    for dy in range(-self.padding, self.padding + 1):
                        for dx in range(-self.padding, self.padding + 1):
                            nx, ny = x + dx, y + dy  # New x, y coordinates
                            if 0 <= ny < height and 0 <= nx < width:  # Bounds check
                                expanded_occdata[ny, nx] = 101  # Mark as occupied
        # Apply the expanded costmap
        self.occdata = expanded_occdata
        
               
        self.occ_callback_called = True
        #rows, cols = self.occdata.shape
        #print(f"Occupancy Grid Size: {rows} x {cols}")

        if np.any(self.occdata == 0):
            pass
        else:
            self.get_logger().info("No unknown cells found in the occupancy grid.")

        #self.plot_func()
        
        # 0 = Unknown
        # 1 - 99 = Free Space
        # >= 100 = Occupied Space

    def rotate_till_occu(self):
        self.get_logger().info("Rotating till occupied space found")
        twist = Twist()
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = rotatechange
        # start rotation
        self.publisher_.publish(twist)

        while np.sum(self.occdata == 101) < 100:
            rclpy.spin_once(self)
            
        self.stopbot()
        self.get_logger().info("Starting navigation......")


    def rotate_till_path_available(self):
        self.get_logger().info("Rotating till path available")
        twist = Twist()
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = rotatechange
        # start rotation
        self.publisher_.publish(twist)

        while self.distance_to_goal is None and self.shortest_path is None:
            rclpy.spin_once(self)
        self.stopbot()
        self.get_logger().info(f"Distance to goal is {self.distance_to_goal}")
        self.get_logger().info(f"Shortest path is {self.shortest_path}")
        self.get_logger().info("Path found. Starting navigation......")


    def get_robot_grid_position(self):
        """
        Calculate the robot's (x, y) position in occupancy grid coordinates.
        Uses TF transforms if available; otherwise, falls back to odometry.
        """
        # Check if TF transform is available
        try:
            trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            cur_pos_x = trans.transform.translation.x
            cur_pos_y = trans.transform.translation.y
            self.robot_x = cur_pos_x
            self.robot_y = cur_pos_y
            #self.get_logger().info("Using TF transform for position.")
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("TF lookup failed, falling back to odometry.")
            
            # Use odometry as fallback
            if hasattr(self, 'robot_x') and hasattr(self, 'robot_y'):
                cur_pos_x = self.robot_x
                cur_pos_y = self.robot_y
            else:
                self.get_logger().error("No valid position available.")
                return None, None  # Return None if no valid data is found
        
        # Ensure map metadata is available
        if not hasattr(self, 'map_resolution') or not hasattr(self, 'map_origin_x'):
            self.get_logger().error("Map metadata not available.")
            return None, None

        # Convert world coordinates to grid indices
        grid_x = int((cur_pos_x - self.map_origin_x) / self.map_resolution)
        grid_y = int((cur_pos_y - self.map_origin_y) / self.map_resolution)

        #self.get_logger().info(f"Robot Grid Position: ({grid_x}, {grid_y})")
        return grid_x, grid_y


    def is_frontier(self, map_data, x, y):
        """
        Check if a given cell (x, y) is a frontier. A frontier is defined as a free space
        adjacent to an unknown cell.

        :param map_data: 2D occupancy grid data.
        :param x: The x coordinate of the cell.
        :param y: The y coordinate of the cell.
        :return: True if the cell is a frontier, False otherwise.
        """
        
        # Check if the current cell is occupied or unknown
        if map_data[y, x] == 101 or map_data[y, x] == 0:
            return False  # This cell is either occupied (101) or unknown (0)

        # Ensure that we are not considering the robot's current position
        if (x, y) == (self.robot_x, self.robot_y):
            return False  # Exclude the robot's current position

        # Check for neighboring unknown cells (0)
        #neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connected neighbors
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if 0 <= ny < map_data.shape[0] and 0 <= nx < map_data.shape[1]:
                if map_data[ny, nx] == 0:  # If a neighbor is unknown
                    return True  # This cell is a frontier

        return False  # No adjacent unknown cells, not a frontier


    def detect_closest_frontier_outside(self, robot_pos, min_distance=3):

        # Use squared distance to avoid unnecessary sqrt calculations
        min_dist_sq = min_distance ** 2

        queue = deque([robot_pos])
        visited = set([robot_pos])

        count = 0
        while queue:

            x, y = queue.popleft()
            count += 1
            # Calculate squared distance from the robot's position
            dist_sq = (x - robot_pos[0])**2 + (y - robot_pos[1])**2

            # Check only cells that are outside the minimum distance
            if dist_sq >= min_dist_sq:
                if self.is_frontier(self.occdata, x, y) and (x, y) not in self.visited_frontiers:
                    for dx in range(-1, 2):  # Covers [-1, 0, 1]
                        for dy in range(-1, 2):  # Covers [-1, 0, 1]
                            nx, ny = x + dx, y + dy
                            if 0 <= ny < self.occdata.shape[0] and 0 <= nx < self.occdata.shape[1]:
                                self.visited_frontiers.add((nx, ny))
                    return (x, y)

            # Explore 8-connected neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited and 0 <= ny < self.occdata.shape[0] and 0 <= nx < self.occdata.shape[1]:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        print(count)
        return None
    
    def detect_closest_frontier_outside_without_processing(self, robot_pos, min_distance=3):

        # Use squared distance to avoid unnecessary sqrt calculations
        min_dist_sq = min_distance ** 2

        queue = deque([robot_pos])
        visited = set([robot_pos])

        count = 0
        while queue:

            x, y = queue.popleft()
            count += 1
            # Calculate squared distance from the robot's position
            dist_sq = (x - robot_pos[0])**2 + (y - robot_pos[1])**2

            # Check only cells that are outside the minimum distance
            if dist_sq >= min_dist_sq:
                if self.is_frontier(self.occdata, x, y) and (x, y) not in self.visited_frontiers:
                    for dx in range(-1, 2):  # Covers [-1, 0, 1]
                        for dy in range(-1, 2):  # Covers [-1, 0, 1]
                            nx, ny = x + dx, y + dy
                            if 0 <= ny < self.occdata.shape[0] and 0 <= nx < self.occdata.shape[1]:
                                self.visited_frontiers.add((nx, ny))
                    return (x, y)

            # Explore 8-connected neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited and 0 <= ny < self.original_occdata.shape[0] and 0 <= nx < self.original_occdata.shape[1]:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        print(count)
        return None


    def find_closest_unknown_outside(self, robot_pos, min_distance=2):
        """
        Finds the closest unknown cell (value == 0) outside the min_distance radius 
        from the robot's current grid position.
        
        :param robot_pos: (rx, ry) robot's current grid coordinates.
        :param min_distance: Minimum Euclidean distance (in grid cells) to exclude.
        :return: (x, y) grid coordinates of the closest unknown cell outside the radius, or None if not found.
        """
        queue = deque([robot_pos])
        visited = set([robot_pos])
        
        while queue:
            x, y = queue.popleft()
            
            # Only consider cells outside the min_distance radius
            if math.sqrt((x - robot_pos[0])**2 + (y - robot_pos[1])**2) >= min_distance:
                if self.occdata[x, y] == 0:
                    return (x, y)
            
            # Expand search in 4-connected neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited and 0 <= nx < self.occdata.shape[0] and 0 <= ny < self.occdata.shape[1]:
                    visited.add((nx, ny))
                    queue.append((nx, ny))
        return None


    def IMU_interrupt_check(self):
        with self.lock:
            if self.global_pitch_avg > 0 and self.recent_pitch_avg > 5 * self.global_pitch_avg: # set as 5 * moving average
                self.get_logger().info("IMU Interrupt detected")
                return True
            else:
                return False


    def grid_to_world(self, grid_x, grid_y):
        """
        Converts grid coordinates (x, y) to world coordinates.
        :param grid_x: Grid X position.
        :param grid_y: Grid Y position.
        :return: (x_world, y_world) in meters.
        """
        if not hasattr(self, 'map_resolution') or not hasattr(self, 'map_origin_x'):
            self.get_logger().error("Map metadata not available.")
            return None, None

        x_world = self.map_origin_x + (grid_x * self.map_resolution)
        y_world = self.map_origin_y + (grid_y * self.map_resolution)

        return x_world, y_world
 

    def stopbot(self):
        self.get_logger().info('Stopping the robot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)


    def wait_for_map(self):
        self.get_logger().info("Waiting for initial map update...")
        while self.occdata.size == 0 or not np.any(self.occdata != 0):
            self.get_logger().warn("No occupancy grid data yet, waiting...")
            rclpy.spin_once(self)
            time.sleep(1)
        self.get_logger().info("Map received. Starting Dijkstra movement.")
        

    def dijk_mover(self):
        try:
            # Get the current position of the robot
            start = self.get_robot_grid_position()

            if start[0] is None or start[1] is None:
                self.get_logger().warn("No valid robot position available.")
                return
            
            #self.get_logger().info(f"Current position: ({start[0]}, {start[1]})")
            # Find the closest frontier (unmapped cell)
            if self.confirming_unknowns:
                frontier = self.detect_closest_frontier_outside_without_processing(start, min_distance=2)
            else:
                frontier = self.detect_closest_frontier_outside(start, min_distance=2)
            #self.get_logger().info(f"Frontier detected at ({frontier[0]}, {frontier[1]})")
            if frontier is not None:
                # Convert the frontier grid coordinates to world coordinates
                world_x, world_y = self.grid_to_world(frontier[0], frontier[1])
                #self.get_logger().info(f"Frontier detected at ({frontier[0]}, {frontier[1]})")
                # Send Nav2 goal to the frontier and wait for confirmation
                self.get_logger().info(f"Navigating to closest unmapped cell at {world_x}, {world_y}")
                goal_reached = self.nav_to_goal(world_x, world_y)

                if goal_reached:
                    #self.get_logger().info("Robot successfully navigated to the frontier. Proceeding to the next frontier.")
                    time.sleep(1)  # Small delay before next iteration
                else:
                    self.get_logger().warn("Failed to reach the goal, retrying or taking action.")
            else:
                self.confirming_unknowns = True
                self.get_logger().warn("No frontiers found. Robot is stuck!")

                
        except Exception as e:
            self.get_logger().error(f"Error in dijk_mover: {e}")

        finally:
            # Stop robot if needed
            self.stopbot()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info(f"[Feedback] Distance remaining: {feedback.distance_remaining:.2f}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌ Goal was rejected by Nav2.")
            self.goal_active = False
            return

        self.get_logger().info("✅ Goal accepted by Nav2.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)


    def goal_result_callback(self, future):
        try:
            result_msg = future.result()
            status = result_msg.status  # ✅ status is here
            result = result_msg.result  # this is the NavigateToPose_Result message

            self.get_logger().info(f"🏁 Nav2 goal finished with status: {status}")

            if status == 3:  # STATUS_SUCCEEDED
                self.get_logger().info("🎯 Goal reached successfully!")
                self.goal_active = False
                self.just_reached_goal = True
            else:
                self.get_logger().warn(f"⚠️ Goal ended with failure status: {status}")

        except Exception as e:
            self.get_logger().error(f"❌ Exception in goal_result_callback: {e}")



    def nav_to_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f"📬 Sending goal to Nav2: x={x:.2f}, y={y:.2f}")

        self.goal_active = True  # Flag to prevent multiple goals

        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)



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

    def IMU_interrupt_check(self):
        with self.lock:
            if self.global_pitch_avg > 0 and self.recent_pitch_avg > 5 * self.global_pitch_avg: # set as 5 * moving average
                self.get_logger().info("IMU Interrupt detected")
                return True
            else:
                return False

    

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
            self.get_logger().info("IMU Interrupt detected, Stopping all movement")
            self.stopbot()
            time.sleep(5) # to give time for control loop to choose a new path and place a "do not go" marker
            pass
        elif bot_current_state == GlobalController.State.Exploratory_Mapping:
            if(self.previous_state != bot_current_state):
                self.get_logger().info("Exploratory Mapping...")
                self.previous_state = bot_current_state
            if self.IMU_interrupt_check(): ## IMU interrupt is true
                self.set_state(GlobalController.State.Imu_Interrupt)
                self.get_logger().info("IMU Interrupt detected, changing state to IMU Interrupt")
            self.log_temperature()
            pass
        elif bot_current_state == GlobalController.State.Goal_Navigation:
            if self.IMU_interrupt_check(): ## IMU interrupt is true
                self.set_state(GlobalController.State.Imu_Interrupt)
                self.get_logger().info("IMU Interrupt detected, changing state to IMU Interrupt")
            pass
        elif bot_current_state == GlobalController.State.Go_to_Heat_Souce:
            self.IMU_interrupt_check()

        

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
        bot_current_state = self.get_state()
        if bot_current_state == GlobalController.State.Initializing:
            self.initialise()
            self.set_state(GlobalController.State.Exploratory_Mapping)
        elif bot_current_state == GlobalController.State.Imu_Interrupt:
            self.get_logger().info("IMU Interrupt detected from control loop, walling off are and setting alternative goal")
            # TODO: Create function to change map to wall off area and reset goal

        elif bot_current_state == GlobalController.State.Exploratory_Mapping:
            self.get_logger().info("Exploratory Mapping (control_loop)...")
            self.dijk_mover()

            ## TODO: set threshold for fully maped area to cut off exploratory mapping
            if self.finished_mapping:
                ## TODO: get robot max heat positions and set goal to the max heat position (stored in self.temp_and_location_data), store it in self.max_heat_locations
                self.get_logger().info("Finished Mapping, changing state to Goal Navigation")
                self.set_state(GlobalController.State.Goal_Navigation)

        elif bot_current_state == GlobalController.State.Goal_Navigation:
            ## Go to max heat location then change state to Launching Balls
            if self.just_reached_goal:
                self.get_logger().info("✅ Goal reached, switching to Launching_Balls")
                self.just_reached_goal = False
                self.set_state(GlobalController.State.Launching_Balls)
                return

            if not self.goal_active and self.max_heat_locations:
                location = self.max_heat_locations.pop(0)
                if location is not None:
                    self.nav_to_goal(location[0], location[1])
                    return
                
        elif bot_current_state == GlobalController.State.Go_to_Heat_Souce:
            # TODO: Funciton to go to heat source in while loop
                # if self.IMU_interrupt_check():
                # exit condition when too close or some shit
            if self.ball_lauches_attempted == 2:
                self.get_logger().info("Ball launches attempted, changing state to goal navigation")
                self.set_state(GlobalController.State.Attempting_Ramp)
                
            else:
                self.get_logger().info("Going back to goal navigation state")
                self.set_state(GlobalController.State.Goal_Navigation)
                
        elif bot_current_state == GlobalController.State.Launching_Balls:
            self.launch_ball()
            self.get_logger().info("Launching Balls...")
            time.sleep(15)
            self.get_logger().info("Finished Launching Balls, changing state back to goal navigation")
            self.set_state(GlobalController.State.Goal_Navigation)
        elif bot_current_state == GlobalController.State.Attempting_Ramp:
            ## check for ramp using IMU Data (potentially), poll for when IMU is flat, so there is no pitch meaning the top of the remp has been reached
            pass


    def initialise(self):
        self.wait_for_map()
        self.rotate_till_occu()


def main(args=None):#
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
