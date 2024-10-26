#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile
import numpy as np
import math
import heapq

# Import the Obstacles message
from f112th_sim_2402_bravo.msg import Obstacles  

expansion_size = 1

def euler_from_quaternion(x, y, z, w):
    # Convert quaternion to Euler angles (yaw)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(siny_cosp, cosy_cosp)
    return yaw_z

def costmap(data, width, height, resolution):
    # Expand walls in the costmap
    data = np.array(data).reshape(height, width)
    wall = np.where(data == 100)
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = np.clip(wall[0] + i, 0, height - 1)
            y = np.clip(wall[1] + j, 0, width - 1)
            data[x, y] = 100
    return data

def distance(a, b):
    # Calculate Euclidean distance between two points
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    # A* pathfinding algorithm
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0),
                 (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: distance(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + distance(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] != 0:
                        continue
                else:
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + distance(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation Node Started')

        self.goal_x = []
        self.goal_y = []
        self.path_world = []  # Store path in world coordinates

        # Subscriptions
        self.subscription_map = self.create_subscription(
            OccupancyGrid, '/map', self.OccGrid_callback, 10)
        self.subscription_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.Goal_Pose_callback, QoSProfile(depth=10))
        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to obstacle positions and velocities
        self.subscription_obstacles = self.create_subscription(
            Obstacles, 'detected_obstacles', self.obstacle_callback, 10)

        # Robot's current position and orientation
        self.robot_pose_x = None
        self.robot_pose_y = None
        self.robot_yaw = None

        # Control parameters
        self.look_ahead_distance = 1.0  # Adjust as necessary
        self.max_linear_speed = 0.1    # Adjust as necessary
        self.max_angular_speed = 3.0    # Adjust as necessary
        self.safety_distance = 0.5  # Safety distance to obstacles

        # Timer for control loop
        self.control_timer = None

        # Obstacle positions and velocities
        self.obstacle_positions = []  # List of obstacles with positions and velocities

    def OccGrid_callback(self, msg):
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_data = msg.data

    def odom_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        self.robot_yaw = euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

    def obstacle_callback(self, msg):
        # Update the list of obstacle positions and velocities
        self.obstacle_positions = []
        num_obstacles = len(msg.ids)
        for idx in range(num_obstacles):
            obstacle = {
                'id': msg.ids[idx],
                'x': msg.positions[idx].x,
                'y': msg.positions[idx].y,
                'vx': msg.velocities[idx].x,
                'vy': msg.velocities[idx].y
            }
            self.obstacle_positions.append(obstacle)

        # Replan if necessary
        self.get_logger().info("Obstacles updated, checking for replanning...")
        if self.control_timer:
            self.get_map()

    def Goal_Pose_callback(self, msg):
        # Clear previous goals if starting a new navigation task
        user_input = input("Start a new navigation task? (y/n): ")
        if user_input.lower() == 'y':
            self.goal_x.clear()
            self.goal_y.clear()
            self.path_world.clear()
            if self.control_timer:
                self.control_timer.cancel()
            self.current_index = 0  # Reset current index

        self.goal_x.append(msg.pose.position.x)
        self.goal_y.append(msg.pose.position.y)
        if input("More waypoints? (y/n): ") == 'n':
            self.get_map()

    def get_map(self):
        # Ensure we have the robot's current position
        if self.robot_pose_x is None or self.robot_pose_y is None:
            self.get_logger().error("Robot's current position is unknown.")
            return

        # Create the costmap from the map data
        data = costmap(self.map_data, self.width, self.height, self.resolution)

        # Incorporate detected obstacles into the costmap
        data = self.add_obstacles_to_costmap(data)

        # Convert goal and start positions to grid indices
        goal_column = int((self.goal_x[-1] - self.originX) / self.resolution)
        goal_row = int((self.goal_y[-1] - self.originY) / self.resolution)
        start_column = int((self.robot_pose_x - self.originX) / self.resolution)
        start_row = int((self.robot_pose_y - self.originY) / self.resolution)

        start = (start_row, start_column)
        goal = (goal_row, goal_column)

        # Check if start and goal are within bounds
        if not (0 <= start_row < self.height and 0 <= start_column < self.width):
            self.get_logger().error("Start position is out of bounds.")
            return

        if not (0 <= goal_row < self.height and 0 <= goal_column < self.width):
            self.get_logger().error("Goal position is out of bounds.")
            return

        # Check if start or goal positions are in obstacles
        if data[start_row][start_column] != 0:
            self.get_logger().error("Start position is in an obstacle.")
            return

        if data[goal_row][goal_column] != 0:
            self.get_logger().error("Goal position is in an obstacle.")
            return

        # Convert data to numpy array for astar
        data_array = np.array(data)

        # Find the path using A*
        path = astar(data_array, start, goal)

        if not path:
            self.get_logger().error("No valid path found.")
            return

        # Convert path from grid indices to world coordinates
        self.path_world = []
        for row, col in path:
            x = col * self.resolution + self.originX + self.resolution / 2.0
            y = row * self.resolution + self.originY + self.resolution / 2.0
            self.path_world.append((x, y))

        # Start the control loop
        if self.control_timer:
            self.control_timer.cancel()
        self.control_timer = self.create_timer(0.1, self.pure_pursuit_control)

    def add_obstacles_to_costmap(self, data):
        # Convert the costmap to a mutable numpy array
        costmap_array = np.array(data, dtype=np.int8)

        # Time horizon for prediction (seconds)
        time_horizon = 1.0

        # Loop over the obstacles and mark predicted positions in the costmap
        for obs in self.obstacle_positions:
            x = obs['x']
            y = obs['y']
            vx = obs['vx']
            vy = obs['vy']

            # Predict future position
            x_future = x + vx * time_horizon
            y_future = y + vy * time_horizon

            # Convert future positions to grid indices
            col = int((x_future - self.originX) / self.resolution)
            row = int((y_future - self.originY) / self.resolution)

            # Check bounds
            if 0 <= row < self.height and 0 <= col < self.width:
                # Mark the cell as occupied
                costmap_array[row, col] = 100

                # Optionally expand the obstacle in the costmap
                for i in range(-expansion_size, expansion_size + 1):
                    for j in range(-expansion_size, expansion_size + 1):
                        r = row + i
                        c = col + j
                        if 0 <= r < self.height and 0 <= c < self.width:
                            costmap_array[r, c] = 100

        return costmap_array

    def pure_pursuit_control(self):
        if not self.path_world or self.robot_pose_x is None or self.robot_pose_y is None:
            return

        # Current look-ahead point is the first point in path_world initially
        if not hasattr(self, 'current_index'):
            self.current_index = 0

        # Check if the robot has reached the current look-ahead point
        current_point = self.path_world[self.current_index]
        dx = abs(current_point[0] - self.robot_pose_x)
        dy = abs(current_point[1] - self.robot_pose_y)

        # If both dx and dy are smaller than the threshold, go to the next point
        if dx < 0.1 and dy < 0.1:  # Threshold of 0.1 meters
            self.current_index += 1  # Move to the next point

            # If we reached the end of the path, stop the robot
            if self.current_index >= len(self.path_world):
                self.get_logger().info("Goal reached!")
                self._stop_robot()
                return

            current_point = self.path_world[self.current_index]  # Update to the new point

        # Compute the steering angle to the current look-ahead point
        dx = current_point[0] - self.robot_pose_x
        dy = current_point[1] - self.robot_pose_y
        angle_to_goal = math.atan2(dy, dx)

        # Calculate the angle error
        angle_error = angle_to_goal - self.robot_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalize

        # Log debugging information
        self.get_logger().info(f"Angle Error: {angle_error:.2f}")

        # Compute control commands
        linear_speed = self.max_linear_speed
        angular_speed = self.max_angular_speed * angle_error

        # Limit the angular speed
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        # Publish the velocity command
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.publisher.publish(twist)

    def _stop_robot(self):
        """Stop the robot and cancel the control timer."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        if self.control_timer:
            self.control_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
