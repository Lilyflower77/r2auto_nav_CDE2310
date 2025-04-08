import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped ,Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import Float32MultiArray
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState, ChangeState
from sensor_msgs.msg import Imu , LaserScan
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
from sklearn.cluster import KMeans
from collections import Counter


'''
# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
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
'''

# constants
rotatechange = 0.15 # was 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

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

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.laser_callback,
            qos_profile_sensor_data)
        self.subscription  # prevent unused variable warning

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
        
        #TODO: Create a publisher for the ball launcher (maybe action etc, must have feedback for when the ball is done launching)


        # Temperature Attributes
        self.latest_left_temp = None
        self.latest_right_temp = None
        self.unfiltered_x_y_list = []
        self.filtered_x_y_list = []
        self.heat_left_world_x_y = []
        self.heat_right_world_x_y = []

        # IMU Attributes stored as (timestamp, pitch)
        self.pitch_window = deque()
        self.ramp_location = None
        self.hit_ramped = False
        # For global moving average
        self.global_pitch_sum = 0.0
        self.global_pitch_count = 0
        # For recent average (last 0.3s)
        self.recent_pitch_avg = 0.0
        self.global_pitch_avg = 0.0
        #occ map variables
        self.padding = 2
        self.confirming_unknowns = False

        
        # logic attributes
        self.state = GlobalController.State.Initializing
        self.previous_state = None
        self.ball_launches_attempted = 0
        self.max_heat_locations = [None] * 2
        self.ramp_location = [None]
        self.finished_mapping = False

        self.occ_callback_called = False
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 Action Server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 Action Server available. Ready to send goals.")
        self.visited_frontiers = set()
        self.distance_to_heat = None
        self.angle_to_heat = None

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
            self.latest_left_temp = center_values

            if(self.valid_heat(self.latest_left_temp)):
                #TODO : adjust to the real angle range of where the sensor points
                angle , distance = self.laser_avg_angle_and_distance_in_mode_bin(-5,5, 0.1)
                x , y = self.calculate_heat_world(angle , distance)
                self.heat_left_world_x_y.append([x,y])


    def sensor2_callback(self, msg):
        if msg.data and len(msg.data) == 64:
            indices = [
            18, 19, 20, 21,
            26, 27, 28, 29,
            34, 35, 36, 37,
            42, 43, 44, 45
        ]
            
            center_values = [msg.data[i] for i in indices]
            #self.latest_right_temp = sum(center_values) / len(center_values)
            self.latest_right_temp = center_values

            if(self.valid_heat(self.latest_right_temp)):
                #TODO : adjust to the real angle range of where the sensor points
                angle , distance = self.laser_avg_angle_and_distance_in_mode_bin(-5,5, 0.1)
                x , y = self.calculate_heat_world(angle , distance)
                self.heat_right_world_x_y.append([x,y])



    def laser_callback(self, msg):
        self.laser_msg = msg

        

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
        # compute histogram to identify percent of bins with -1
        #occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        #total_bins = msg.info.width * msg.info.height
        # log the info
        #self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        
        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height, msg.info.width))
        # self.get_logger().info(f"Unique values in occupancy grid: {np.unique(self.occdata)}")
        np.savetxt(mapfile, self.occdata)

        # Safely mark visited frontiers
        for node in self.visited_frontiers:
            x, y = node
            if 0 <= y < self.occdata.shape[0] and 0 <= x < self.occdata.shape[1]:
                if self.occdata[y, x] != 101:
                    self.occdata[y, x] = 1

        
        
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
    
    def detect_closest_frontier_outside_without_processing(self, robot_pos, min_distance=2):

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

        if not hasattr(self, 'map_resolution') or not hasattr(self, 'map_origin_x'):
            self.get_logger().error("Map metadata not available.")
            return None, None
    
        world_x = self.map_origin_x + (grid_x * self.map_resolution)
        world_y = self.map_origin_y + (grid_y * self.map_resolution)

        return world_x, world_y
 

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
        


    def valid_heat(self, array , threshold = 30.0):
        if array is None:
            return False

        array = np.array(array)

        if array.size == 0:
            return False

        if np.max(array) >= threshold:
            return True

        return False


    def find_centers(self,n_centers = 2):
        
        # Apply KMeans clustering
        kmeans = KMeans(n_clusters=n_centers, random_state=0)
        full_list = self.heat_left_world_x_y + self.heat_right_world_x_y
        data = np.array(full_list)  # ← your list of (x, y)
        if(data.size == 0):
            return
        kmeans.fit(data)

        # Get the center points
        centers = kmeans.cluster_centers_
        return centers


    def dijk_mover(self):
        try:
            # Get the current position of the robot
            start = self.get_robot_grid_position()

            self.confirming_unknowns = False
            if start[0] is None or start[1] is None:
                self.get_logger().warn("No valid robot position available.")
                return
            
            if self.confirming_unknowns:
                frontier = self.detect_closest_frontier_outside_without_processing(start, min_distance=2)
            else:
                frontier = self.detect_closest_frontier_outside(start, min_distance=2)
            if frontier is not None:
                world_x, world_y = self.grid_to_world(frontier[0], frontier[1])
                self.get_logger().info(f"Navigating to closest unmapped cell at {world_x}, {world_y}")
                self.nav_to_goal(world_x, world_y)
            else:
                '''
                if(self.confirming_unknowns and self.padding == 0):
                    self.finished_mapping = True
                    self.get_logger().warn("No frontiers found. Robot is stuck!")
                elif self.padding > 0:
                    self.padding -= 1
                    self.visited_frontiers = set()
                    self.confirming_unknowns = True
                '''
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



    def calculate_heat_world(self , angle_to_heat , distance_to_heat):
        x,y,yaw = self.get_robot_global_position()
        if x is None or y is None or angle_to_heat is None or distance_to_heat is None:
            self.get_logger().warn("Unagle to calculate heat grid, missing data.")
            return
        #angle in rad
        return self.polar_to_world_coords(angle_to_heat, distance_to_heat, x, y, yaw)
        

    def world_to_grid(x_world, y_world, origin_x, origin_y, resolution):
        
        grid_x = int((x_world - origin_x) / resolution)
        grid_y = int((y_world - origin_y) / resolution)
        return grid_x, grid_y


    def goal_result_callback(self, future):
        try:
            result_msg = future.result()
            status = result_msg.status  # ✅ status is here
            result = result_msg.result  # this is the NavigateToPose_Result message

            self.get_logger().info(f"🏁 Nav2 goal finished with status: {status}")

            if status == 3:  # STATUS_SUCCEEDED
                self.reached_heat = True
                self.get_logger().info("🎯 Goal reached successfully!")
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


    def laser_avg_angle_and_distance_in_mode_bin(self, angle_min_deg=-30, angle_max_deg=30, bin_width=0.1):

        scan_msg = self.laser_msg
        ranges = np.array(scan_msg.ranges)
        angles = np.arange(scan_msg.angle_min, scan_msg.angle_max, scan_msg.angle_increment)

        if len(angles) > len(ranges):
            angles = angles[:len(ranges)]
        else:
            ranges = ranges[:len(angles)]

        angles_deg = np.degrees(angles)
        mask = (angles_deg >= angle_min_deg) & (angles_deg <= angle_max_deg)

        filtered_ranges = ranges[mask]
        filtered_angles = angles[mask]  # radians

        valid_mask = np.isfinite(filtered_ranges) & (filtered_ranges > 0.01)
        filtered_ranges = filtered_ranges[valid_mask]
        filtered_angles = filtered_angles[valid_mask]

        if len(filtered_ranges) == 0:
            return None, None

        binned = np.floor(filtered_ranges / bin_width).astype(int)
        from collections import Counter
        mode_bin, _ = Counter(binned).most_common(1)[0]

        bin_min = mode_bin * bin_width
        bin_max = bin_min + bin_width
        in_mode = (filtered_ranges >= bin_min) & (filtered_ranges < bin_max)

        mode_distances = filtered_ranges[in_mode]
        mode_angles = filtered_angles[in_mode]

        if len(mode_angles) == 0:
            return None, None

        return np.mean(mode_angles), np.mean(mode_distances)  # both in radians and meters



    def polar_to_world_coords(avg_angle_rad, avg_distance, robot_x, robot_y, robot_yaw_rad):
        x_local = avg_distance * math.cos(avg_angle_rad)
        y_local = avg_distance * math.sin(avg_angle_rad)

        x_world = robot_x + (x_local * math.cos(robot_yaw_rad)) - (y_local * math.sin(robot_yaw_rad))
        y_world = robot_y + (x_local * math.sin(robot_yaw_rad)) + (y_local * math.cos(robot_yaw_rad))
        return (x_world, y_world)



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


    def IMU_interrupt_check(self):
        with self.lock:
            if self.global_pitch_avg > 0 and self.recent_pitch_avg > 5 * self.global_pitch_avg: # set as 5 * moving average
                self.get_logger().info("IMU Interrupt detected")
                return True
            else:
                return False


    def is_valid(self, neighbor, visited):
        x, y = neighbor
        return (x, y) not in visited and 0 <= x < self.occdata.shape[0] and 0 <= y < self.occdata.shape[1]


    def normal_bfs_from_world(self , world_x, world_y):
        grid_x , grid_y = self.world_to_grid(world_x,world_y)
        # Initialize frontier with a starting point
        frontier = deque()
        start = (grid_x, grid_y)  # Replace x and y with your starting coords
        frontier.append(start)

        # Set to keep track of visited nodes
        visited = set()
        visited.add(start)

        # Example grid directions (8-connected)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),(-1, -1), (-1, 1), (1, -1), (1, 1)]

        self.reached_heat = False
        while frontier:
            
            if(self.reached_heat):
                return True
            
            if not self.goal_active:
                current = frontier.popleft()

                x,y = current
                if(self.occdata[y,x] == 1):
                    cur_x , cur_y  = self.grid_to_world(x,y)
                    self.nav_to_goal(cur_x , cur_y)

            
                for dx, dy in directions:
                    neighbor = (current[0] + dx, current[1] + dy)
                    if self.is_valid(neighbor, visited):
                        frontier.append(neighbor)
                        visited.add(neighbor)
        return False
                    

    def seal_line_along_facing_axis(self ,length=21):
        """
        Seals a straight axis-aligned line in the direction the robot is roughly facing.
        Direction is snapped to the nearest axis (N/S/E/W).
        """
        # Convert robot position to grid
        world_x , world_y , yaw_rad = self.get_robot_global_position()
        cx, cy = self.world_to_grid(world_x, world_y)

        # Determine axis-aligned direction
        dx = np.cos(yaw_rad)
        dy = np.sin(yaw_rad)

        if abs(dx) > abs(dy):
            # More aligned with x-axis
            direction = 'E' if dx > 0 else 'W'
        else:
            # More aligned with y-axis
            direction = 'N' if dy > 0 else 'S'

        # Pick unit vector for direction
        dir_map = {
            'N': (0, 1),
            'S': (0, -1),
            'E': (1, 0),
            'W': (-1, 0)
        }
        step_x, step_y = dir_map[direction]

        half_len = length // 2
        line_coords = []

        for i in range(-half_len, half_len + 1):
            gx = cx + i * step_x
            gy = cy + i * step_y
            #TODO: is this suppose to be x and y swapped?
            if 0 <= gx < self.occdata.shape[1] and 0 <= gy < self.occdata.shape[0]:
                line_coords.append((gx, gy))
            else:
                return  # Abort if any part goes out of bounds

        # Check ends
        (sx, sy), (ex, ey) = line_coords[0], line_coords[-1]

        def is_connected(x, y):
            return self.occdata[y, x] == 101

        if is_connected(sx, sy) and is_connected(ex, ey):
            for x, y in line_coords:
                self.occdata[y, x] = 101
            self.get_logger().info(f"🧱 Sealed {direction}-axis line from ({cx},{cy})")
        else:
            self.get_logger().info("❌ Not sealing: ends not connected.")



    
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
            self.stopbot()
            pass
        elif bot_current_state == GlobalController.State.Exploratory_Mapping:
            if(self.previous_state != bot_current_state):
                self.get_logger().info("Exploratory Mapping...")
                self.previous_state = bot_current_state
            if self.IMU_interrupt_check() and not self.hit_ramped: ## IMU interrupt is true
                self.set_state(GlobalController.State.Imu_Interrupt)
                self.get_logger().info("IMU Interrupt detected, changing state to IMU Interrupt")
            self.log_temperature()
            pass
        elif bot_current_state == GlobalController.State.Goal_Navigation:
            if self.IMU_interrupt_check() and not self.hit_ramped:
                self.set_state(GlobalController.State.Imu_Interrupt)
                self.get_logger().info("IMU Interrupt detected, changing state to IMU Interrupt")
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
        bot_current_state = self.get_state()
        if bot_current_state == GlobalController.State.Initializing:
            self.initialise()
            self.set_state(GlobalController.State.Exploratory_Mapping)
        elif bot_current_state == GlobalController.State.Imu_Interrupt:
            self.get_logger().info("IMU Interrupt detected from control loop, walling off are and setting alternative goal")
            self.ramp_location = self.get_robot_global_position()
            self.seal_line_along_facing_axis(21)
            self.hit_ramped = True
            time.sleep(5) # to give time for control loop to choose a new path and place a "do not go" marker
            bot_current_state = GlobalController.State.Exploratory_Mapping

        elif bot_current_state == GlobalController.State.Exploratory_Mapping:
            self.get_logger().info("Exploratory Mapping (control_loop)...")
            self.dijk_mover()

            ## TODO: set threshold for fully maped area to cut off exploratory mapping
            if self.finished_mapping:
                ## TODO: get robot max heat positions and set goal to the max heat position (stored in self.temp_and_location_data), store it in self.max_heat_locations
                self.get_logger().info("Finished Mapping, changing state to Goal Navigation")
                self.set_state(GlobalController.State.Goal_Navigation)

        elif bot_current_state == GlobalController.State.Goal_Navigation:
            '''
            self.get_logger().info(str(self.latest_left_temp))
            ## Go to max heat location then change state to Launching Balls
            if max(self.latest_left_temp) >= 32:
                self.stopbot()
                self.get_logger().info("done")
            else:
                self.p_controller_to_twist(np.array(self.latest_left_temp).reshape((4, 4)))
            '''
            #uncomment if want skip heat
            #self.set_state(GlobalController.State.Launching_Balls)
            max_heat_locations = self.find_centers(n_centers=2)
            for location in max_heat_locations:

                world_x, world_y = location
                result = self.normal_bfs_from_world(world_x,world_y)

                if not result:
                    self.get_logger().info("Unable to reach heat")

                self.get_logger().info("Goal Navigation, setting state to Launching Balls")
                self.set_state(GlobalController.State.Launching_Balls)

            self.set_state(GlobalController.State.Attempting_Ramp)
        elif bot_current_state == GlobalController.State.Launching_Balls:
            ## publish to the ball launcher
            self.stopbot()
            self.get_logger().info("Launching Balls...")
            ## TODO: create function to launch balls from the publisher
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
