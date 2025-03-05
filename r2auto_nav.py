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
from sensor_msgs.msg import Temperature
import numpy as np
import math
import cmath
import time
import heapq
import tf2_ros
import scipy.ndimage
import cv2
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from collections import deque
from matplotlib import pyplot as plt
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import matplotlib.colors as mcolors


# constants
rotatechange = 0.15 # was 0.1
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

        self.subscription = self.create_subscription(
            Temperature,
            '/temperature',
            self.temp_callback,
            10
        )


        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.cbar = None

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("Waiting for Nav2 Action Server...")
        self.nav_client.wait_for_server()
        self.get_logger().info("Nav2 Action Server available. Ready to send goals.")
        self.visited_frontiers = set()




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

        for node in self.visited_frontiers:
            if(self.occdata[node[0], node[1]] != 101):
                self.occdata[node[0], node[1]] = 1

        rows, cols = self.occdata.shape
        print(f"Occupancy Grid Size: {rows} x {cols}")

        if np.any(self.occdata == 0):
            pass
        else:
            self.get_logger().info("No unknown cells found in the occupancy grid.")

        self.plot_func()
        
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

    def temp_callback(self, msg):
        pass
        #self.get_logger().info(f'Received temperature: {msg.temperature:.2f}°C')
        
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
        if map_data[x, y] == 101 or map_data[x, y] == 0:
            return False  # This cell is either occupied (101) or unknown (0)

        # Ensure that we are not considering the robot's current position
        if (x, y) == (self.robot_x, self.robot_y):
            return False  # Exclude the robot's current position

        # Check for neighboring unknown cells (0)
        #neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 4-connected neighbors
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in neighbors:
            nx, ny = x + dx, y + dy
            if 0 <= nx < map_data.shape[0] and 0 <= ny < map_data.shape[1]:
                if map_data[nx, ny] == 0:  # If a neighbor is unknown
                    return True  # This cell is a frontier

        return False  # No adjacent unknown cells, not a frontier



    def detect_closest_frontier_outside(self, robot_pos, min_distance=2):
        """
        Uses BFS to detect the closest frontier cell (free cell adjacent to unknown space)
        that is outside the specified minimum distance from the robot.

        :param robot_pos: (x, y) robot's current grid position.
        :param min_distance: Minimum Euclidean distance (in grid cells) from the robot to ignore.
        :return: (x, y) grid coordinates of the closest frontier, or None if not found.
        """
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
                            if 0 <= nx < self.occdata.shape[0] and 0 <= ny < self.occdata.shape[1]:
                                self.visited_frontiers.add((nx, ny))
                    return (x, y)

            # Explore 8-connected neighbors
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),(-1, -1), (-1, 1), (1, -1), (1, 1)]:
                nx, ny = x + dx, y + dy
                if (nx, ny) not in visited and 0 <= nx < self.occdata.shape[0] and 0 <= ny < self.occdata.shape[1]:
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


    def wait_for_map(self):
        self.get_logger().info("Waiting for initial map update...")
        while self.occdata.size == 0 or not np.any(self.occdata != 0):
            self.get_logger().warn("No occupancy grid data yet, waiting...")
            rclpy.spin_once(self)
            time.sleep(1)
        self.get_logger().info("Map received. Starting Dijkstra movement.")
        

    def dijk_mover(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)

                # Get the current position of the robot
                start = self.get_robot_grid_position()

                if start[0] is None or start[1] is None:
                    self.get_logger().warn("No valid robot position available.")
                    continue
                
                #self.get_logger().info(f"Current position: ({start[0]}, {start[1]})")
                # Find the closest frontier (unmapped cell)
                frontier = self.detect_closest_frontier_outside(start, min_distance=2)
                #self.get_logger().info(f"Frontier detected at ({frontier[0]}, {frontier[1]})")
                if frontier is not None:
                    # Convert the frontier grid coordinates to world coordinates
                    world_x, world_y = self.grid_to_world(frontier[0], frontier[1])
                    #self.get_logger().info(f"Frontier detected at ({frontier[0]}, {frontier[1]})")
                    # Send Nav2 goal to the frontier and wait for confirmation
                    self.get_logger().info(f"Navigating to closest unmapped cell at {world_x}, {world_y}")
                    goal_reached = self.send_nav_goal(world_x, world_y)

                    if goal_reached:
                        #self.get_logger().info("Robot successfully navigated to the frontier. Proceeding to the next frontier.")
                        time.sleep(1)  # Small delay before next iteration
                    else:
                        self.get_logger().warn("Failed to reach the goal, retrying or taking action.")
                else:
                    self.get_logger().warn("No frontiers found. Robot is stuck!")
                    #print(self.occdata[start[0], start[1]])
                    #print(np.unique(self.occdata))

                
        except Exception as e:
            self.get_logger().error(f"Error in dijk_mover: {e}")

        finally:
            # Stop robot if needed
            self.stopbot()

    def start(self):
        twist = Twist()
        twist.linear.x = speedchange
        twist.angular.z = 0.0
        # not sure if this is really necessary, but things seem to work more
        # reliably with this
        time.sleep(0.01)
        self.publisher_.publish(twist)
        time.sleep(10)
        self.stopbot()
    
    def plot_func(self): 
        
        costmap = np.full(self.occdata.shape, 0, dtype=np.uint8)  # Initialize all as free space

        # Assign correct cost values based on your occupancy grid convention
        costmap[self.occdata == 0] = 128   # Unknown areas
        costmap[self.occdata == 1] = 0   # Free space
        costmap[self.occdata == 101] = 254  # Obstacles

        
        self.ax.clear()
        self.ax.set_title("Real-Time Costmap Update")
        im = self.ax.imshow(costmap, cmap="hot", origin="lower", vmin=0, vmax=254)

        if self.cbar:
            self.cbar.remove()

        self.cbar = self.fig.colorbar(im, ax=self.ax)
        # Refresh the figure
        plt.draw()
        plt.pause(0.1)  # Small pause to allow rendering


    def plot_occupancy_grid(self):
        # Normalize the data for visualization
        vis_grid = np.zeros_like(self.occdata, dtype=np.uint8)

        # Map values to 0-255 scale
        vis_grid[self.occdata == 0] = 127   # Unknown (gray)
        vis_grid[self.occdata == 1] = 255   # Free (white)
        vis_grid[self.occdata == 101] = 0   # Occupied (black)

        # Define custom colormap
        cmap = mcolors.ListedColormap(['black', 'gray', 'white'])
        bounds = [-1, 0.5, 1.5, 101.5]
        norm = mcolors.BoundaryNorm(bounds, cmap.N)

        plt.figure(figsize=(8, 8))
        plt.imshow(vis_grid, cmap=cmap, norm=norm, origin='lower')
        plt.colorbar(label="Occupancy")
        plt.xlabel("X-axis (Grid Cells)")
        plt.ylabel("Y-axis (Grid Cells)")
        plt.title("Occupancy Grid Map")
        plt.show()

        # Example usage:
        # Assuming self.occdata is a 2D NumPy array
        # self.occdata = np.random.choice([0, 1, 101], size=(50, 50), p=[0.3, 0.5, 0.2])
        # plot_occupancy_grid(self.occdata)



    def send_nav_goal(self, x, y, yaw=0.0):
        goal_msg = NavigateToPose.Goal()

        # Set frame and timestamp
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set target position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        #self.get_logger().info(f'Sending goal: x={x}, y={y}')
        
        # Send goal asynchronously
        send_goal_future = self.nav_client.send_goal_async(goal_msg)

        # Wait for the result to be completed
        rclpy.spin_until_future_complete(self, send_goal_future)

        # Check if the goal was successfully completed
        result = send_goal_future.result()
        if result and result.status == 2:  # Status 2 means goal succeeded
            self.get_logger().info("Goal successfully reached!")
            return True  # Confirmation received, goal reached successfully
        else:
            self.get_logger().warn("Goal failed or was preempted!")
            return False  # Goal failed, need to retry or handle error



def main(args=None):
    rclpy.init(args=args)

    auto_nav = AutoNav()  
    auto_nav.wait_for_map()
    auto_nav.rotate_till_occu()
    #auto_nav.start()
    #print(auto_nav.occdata)
    #auto_nav.temp_func()
    #auto_nav.mover()
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
