# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import heapq
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from collections import deque


# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'

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

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        try: 
            self.tfBuffer = tf2_ros.Buffer()
            self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)
            self.get_logger().info("TF listener created")
        except Exception as e :
            self.get_logger().error("TF listener failed: %s" % str(e))
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])


        #initial shortest path
        self.shortest_path = []
        self.distance_to_goal = 0

        self.THRESHOLD = 100

        #sharing variable
        self.reach_target = False


    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat = msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


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
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        #self.get_logger().info(f"Unique values in occupancy grid: {np.unique(self.occdata)}")
        # print to fil
        np.savetxt(mapfile, self.occdata)

        #update dijkstra
        #self.get_logger().info('Updating Dijkstra')
        
        self.distance_to_goal ,self.shortest_path =  self.Dijkstra()
        
        # 0 = Unknown
        # 1 - 99 = Free Space
        # >= 100 = Occupied Space


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan


    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        #self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        #self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = c_change_dir * rotatechange
        # start rotation
        self.publisher_.publish(twist)

        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        #self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)


    def rotate_till_occu(self):
        self.get_logger().info("Rotating till occupied space found")
        twist = Twist()
        twist.linear.x = 0.0
        # set the direction to rotate
        twist.angular.z = rotatechange
        # start rotation
        self.publisher_.publish(twist)

        while self.THRESHOLD not in self.occdata:
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

        while self.distance_to_goal is not None and self.shortest_path is not None:
            rclpy.spin_once(self)
        self.stopbot()
        self.get_logger().info("Path found. Starting navigation......")


    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            # use nanargmax as there are nan's in laser_range added to replace 0's
            lr2i = np.nanargmax(self.laser_range)
            self.get_logger().info('Picked direction: %d %f m' % (lr2i, self.laser_range[lr2i]))
        else:
            lr2i = 0
            self.get_logger().info('No data!')

        # rotate to that direction
        self.rotatebot(float(lr2i))

        # start moving
        self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(1)
        self.publisher_.publish(twist)


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


    '''
    #blocking needs padding
    def create_graph_from_occupancy_grid(self):
        """
        Converts the occupancy grid into a graph representation with 8-way connectivity.
        
        :param occupancy_grid: 2D numpy array representing the map.
        :return: Graph as a dictionary {node: [(neighbor, weight)]}
        """
        height, width = self.occdata.shape
        graph = {}

        # Define movement options with weights (8-way movement)
        neighbors = [
            (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),  # Up, Down, Left, Right (weight=1)
            (-1, -1, np.sqrt(2)), (-1, 1, np.sqrt(2)), (1, -1, np.sqrt(2)), (1, 1, np.sqrt(2))  # Diagonals (weight=sqrt(2))
        ]

        for x in range(height):
            for y in range(width):
                if 1 <= self.occdata[x, y] <= 80:  # Only consider free space as nodes
                    node = (x, y)
                    graph[node] = []

                    for dx, dy, weight in neighbors:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < height and 0 <= ny < width and 1 <= self.occdata[nx, ny] <= 80:
                            graph[node].append(((nx, ny), weight))  # Add edge with weight
                        
        return graph
    '''


    def create_graph_from_occupancy_grid(self , start):
        """
        Converts the occupancy grid into a graph representation with 8-way connectivity.
        Blocks nodes within a 5-grid radius if self.occdata[x, y] > 80.
        
        :return: Graph as a dictionary {node: [(neighbor, weight)]}
        """
        height, width = self.occdata.shape
        graph = {}

        # Define movement options with weights (8-way movement)
        neighbors = [
            (-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),  # Up, Down, Left, Right (weight=1)
            (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))  # Diagonals (weight=sqrt(2))
        ]

        # Create a blocked set to store nodes within the danger zone
        blocked_nodes = set()

        '''
        # Identify all cells where occupancy > 80 and mark a 5-grid radius around them as blocked
        for x in range(height):
            for y in range(width):
                if self.occdata[x, y] > 80:  # If cell is occupied, block surrounding area
                    for dx in range(-5, 6):  # -5 to 5 range
                        for dy in range(-5, 6):
                            nx, ny = x + dx, y + dy
                            if 0 <= nx < height and 0 <= ny < width:
                                blocked_nodes.add((nx, ny))  # Add to blocked nodes set
        '''
        # Create the graph, avoiding blocked nodes
        for x in range(height):
            for y in range(width):
                if (x, y) not in blocked_nodes and (1 <= self.occdata[x, y] <= self.THRESHOLD or (x == start[0] and y == start[1])):  # Only consider free space as nodes
                    node = (x, y)
                    graph[node] = []

                    for dx, dy, weight in neighbors:
                        nx, ny = x + dx, y + dy
                        if (nx, ny) not in blocked_nodes and 0 <= nx < height and 0 <= ny < width and 1 <= self.occdata[nx, ny] <= self.THRESHOLD:
                            graph[node].append(((nx, ny), weight))  # Add edge with weight

        return graph


    def contains_obstacle(self):
        if not self.shortest_path:
            return True

        for point in self.shortest_path:
            occ_value = self.occdata[point[0], point[1]] 
            if np.isscalar(occ_value):  # If it's a single value, compare directly
                if occ_value >= self.THRESHOLD:
                    return True
            else:  # If occ_value is an array, use np.any()
                if np.any(occ_value >= self.THRESHOLD):
                    return True
        return False


    def Dijkstra(self):
        
        if(self.contains_obstacle() or self.reach_target):
            start = self.get_robot_grid_position()
            end = self.detect_frontiers(start)
            #self.get_logger().info(f"End: {end}")
            #self.get_logger().info(f"Occupancy value at start: {self.occdata[start[0], start[1]]}")
            #self.get_logger().info(f"Occupancy value at end: {self.occdata[end[0], end[1]]}")

            if start is None or end is None:
                self.get_logger().error("Invalid start or end position.")
                return None, None
            
            graph = self.create_graph_from_occupancy_grid(start)
            #print(graph)
            if start not in graph:
                self.get_logger().error("Start position not in graph.")
                return None, None

            if end not in graph:
                self.get_logger().error("End position not in graph")
                return None, None
        
            pq = []
            heapq.heappush(pq, (0, start))
            shortest_paths = {node: float('inf') for node in graph}
            shortest_paths[start] = 0

            previous_nodes = {}

            while pq:
                current_distance, current_node = heapq.heappop(pq)

                if current_node == end:
                    break


                for neighbor, weight in graph[current_node]:
                    distance = current_distance + weight
                
                    if distance < shortest_paths[neighbor]:  # Found a shorter path
                        shortest_paths[neighbor] = distance
                        previous_nodes[neighbor] = current_node
                        heapq.heappush(pq, (distance, neighbor))


            
            # Reconstruct shortest path
            path = []
            node = end
            while node in previous_nodes:
                path.append(node)
                node = previous_nodes[node]
            path.append(start)
            path.reverse()

            if shortest_paths[end] == float('inf'):  # No path found
                self.get_logger().warn("No path found.")
                return None, None
            
            return shortest_paths[end], path #dist and path

        start = self.get_robot_grid_position()
        self.get_logger().info(f"Current position: {start}")
        return self.distance_to_goal , self.shortest_path


    def is_frontier(self, map_data, x, y):
        if map_data[x, y] >= self.THRESHOLD or map_data[x,y] == 0:  # Ensure it's free space
            return False

        # Check 4-connected neighbors ,cannot use 8 because of uneven weight
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                    #(-1, -1), (-1, 1), (1, -1), (1, 1)

        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_data.shape[0] and 0 <= ny < map_data.shape[1]:  # Stay within bounds
                if map_data[nx, ny] == 0:  # Adjacent unknown cell
                    return True
        return False


    def detect_frontiers(self, start):
        """
        Identifies all frontier cells in the occupancy grid.

        :param map_data: 2D numpy array representing the occupancy grid.
        :return: List of (x, y) coordinates of frontier cells.
        """
        frontiers = []
        for x in range(1, self.occdata.shape[0] - 1):
            for y in range(1, self.occdata.shape[1] - 1):
                if self.is_frontier(self.occdata, x, y):
                    #self.get_logger().info(f"Frontier detected at ({x}, {y})")
                    return x,y
        return None, None
                

    def move_to_position(self, target_x, target_y):
        """
        Moves the robot to a specific world coordinate.
        :param target_x: X coordinate in meters.
        :param target_y: Y coordinate in meters.
        """
        twist = Twist()
        tolerance = 0.05  # Stop when within 5cm

        while rclpy.ok():
            # Get robot's current world position
            current_x = self.robot_x
            current_y = self.robot_y

            # Compute the direction to the target
            angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
            distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

            '''
            # Rotate towards the target
            yaw_error = angle_to_target - self.yaw
            if abs(yaw_error) > 0.1:  # Only rotate if significant error
                twist.linear.x = 0.0
                twist.angular.z = np.sign(yaw_error) * rotatechange
                time.sleep(1)
                self.publisher_.publish(twist)
                #rclpy.spin_once(self)
                continue

            # Move forward if properly aligned
            if distance_to_target > tolerance:
                twist.linear.x = speedchange
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
            else:
                break  # Stop moving

            #rclpy.spin_once(self)
            '''
        # Stop robot when goal is reached
        self.stopbot()


    def move_to_pos(self, target_x, target_y , grid_x , grid_y):
        current_x = self.robot_x
        current_y = self.robot_y

        # Compute the direction to the target
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

        angle_deg = math.degrees(angle_to_target)  # Convert to degrees
        
        # Ensure the angle is in the range [0, 360)
        if angle_deg < 0:
            angle_deg += 360

        self.get_logger().info(f"Moving to ({grid_x}, {grid_y}) by rotating {angle_deg} degrees")
        # rotate to that direction
        self.rotatebot(float(angle_deg))

        # start moving
        #self.get_logger().info('Start moving')
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(0.01)
        self.publisher_.publish(twist)

        start_x, start_y = self.robot_x, self.robot_y

        while(self.occdata[grid_x ,grid_y] < self.THRESHOLD):
            # Update current position
            current_x, current_y = self.robot_x, self.robot_y

            # Compute distance traveled
            distance_traveled = math.sqrt((current_x - start_x) ** 2 + (current_y - start_y) ** 2)
            
            #self.get_logger().info(f"Current position: ({self.robot_x}, {self.robot_y})")
            #self.get_logger().info(f"Distance left: {distance_to_target - distance_traveled}")

            # Check if the robot has reached the target
            if distance_traveled >= distance_to_target:
                self.stopbot()
                return True  # Target reached


            # Allow callback functions to update the position
            rclpy.spin_once(self)

        self.stopbot()
        return False  # Target not reached


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
        

    def mover(self):
        try:
            # initialize variable to write elapsed time to file
            # contourCheck = 1

            # find direction with the largest distance from the Lidar,
            # rotate to that direction, and start moving
            self.pick_direction()

            while rclpy.ok():
                if self.laser_range.size != 0:
                    # check distances in front of TurtleBot and find values less
                    # than stop_distance
                    lri = (self.laser_range[front_angles]<float(stop_distance)).nonzero()
                    # self.get_logger().info('Distances: %s' % str(lri))

                    # if the list is not empty
                    if(len(lri[0])>0):
                        # stop moving
                        self.stopbot()
                        # find direction with the largest distance from the Lidar
                        # rotate to that direction
                        # start moving
                        self.pick_direction()
                    
                # allow the callback functions to run
                rclpy.spin_once(self)

        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


    def find_longest_straight_path(self, shortest_path):
        """
        Finds the longest straight-line segment from the first node in the shortest path.

        :param shortest_path: List of (x, y) tuples representing the path
        :return: (end_x, end_y) - The last point in the longest straight-line segment
        """
        if not shortest_path or len(shortest_path) < 2:
            return shortest_path[0] if shortest_path else (None, None)

        # Start from the first node
        start_x, start_y = shortest_path[0]
        end_x, end_y = start_x, start_y

        # Determine initial movement direction
        dx = shortest_path[1][0] - start_x
        dy = shortest_path[1][1] - start_y

        # Iterate through the path to find the longest straight-line segment
        for i in range(1, len(shortest_path)):
            x, y = shortest_path[i]

            # Check if movement remains consistent in the same direction
            if (x - start_x) == (i * dx) and (y - start_y) == (i * dy):
                end_x, end_y = x, y  # Update the end of the straight line
            else:
                break  # Stop when direction changes

        return end_x, end_y


    def wait_for_map(self):
        self.get_logger().info("Waiting for initial map update...")
        while self.occdata.size == 0:
            self.get_logger().warn("No occupancy grid data yet, waiting...")
            rclpy.spin_once(self)
            time.sleep(1)
        self.get_logger().info("Map received. Starting Dijkstra movement.")
        

    def dijk_mover(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                #self.get_logger().info(f"Distance to Goal: {self.distance_to_goal}")
                if self.distance_to_goal is not None and self.shortest_path is not None:
                    '''
                    for x,y in self.shortest_path:
                    
                        #check if occupied
                        if self.occdata[x,y] >= 80:
                            self.get_logger().warn(f"Path blocked at ({x}, {y}), replanning...")
                            break

                        world_x, world_y = self.grid_to_world(x , y)
                        if world_x is not None and world_y is not None:
                            self.move_to_pos(world_x, world_y)
                            # allow the callback functions to run
                            rclpy.spin_once(self)
                    '''
                    x , y = self.find_longest_straight_path(self.shortest_path)

                    if self.occdata[x,y] >= self.THRESHOLD:
                        self.get_logger().warn(f"Path blocked at ({x}, {y}), replanning...")
                        continue

                    world_x, world_y = self.grid_to_world(x , y)
                    if world_x is not None and world_y is not None:
                        
                        self.reach_target = self.move_to_pos(world_x, world_y , x , y)
                        #self.get_logger().info(f"Moving to ({x}, {y})")
                        #time.sleep(2)
                        # allow the callback functions to run
                        rclpy.spin_once(self)
                else:

                    self.get_logger().warn("No valid path found. Falling back to default movement.")
                    self.rotate_till_path_available()
                    #self.pick_direction()
                    #time.sleep(0.5)
                    #rclpy.spin_once(self)
                    
        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()

    


def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()
    #auto_nav.mover()
    auto_nav.wait_for_map()
    auto_nav.rotate_till_occu()
    auto_nav.dijk_mover()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
