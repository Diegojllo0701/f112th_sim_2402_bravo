#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math

expansion_size = 1

def euler_from_quaternion(x, y, z, w):
    # Convert quaternion to Euler angles (yaw)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    yaw_z = math.atan2(t0, t1)
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
            if (0 <= neighbor[0] < array.shape[0] and
                0 <= neighbor[1] < array.shape[1]):
                if array[neighbor[0]][neighbor[1]] != 0:
                    continue
            else:
                continue
            tentative_g_score = gscore[current] + distance(current, neighbor)
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

        # Subscriptions
        self.subscription_map = self.create_subscription(
            OccupancyGrid, '/map', self.OccGrid_callback, 10)
        self.subscription_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.Goal_Pose_callback, QoSProfile(depth=10))
        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot's current position
        self.robot_pose_x = None
        self.robot_pose_y = None

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

    def Goal_Pose_callback(self, msg):
        # Clear previous goals if starting a new navigation task
        user_input = input("Start a new navigation task? (y/n): ")
        if user_input.lower() == 'y':
            self.goal_x.clear()
            self.goal_y.clear()

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
            self.get_logger().error("No se encontró un camino válido.")
            return

        # Plotting the path
        fig, ax = plt.subplots()
        ax.imshow(data_array, cmap='gray')

        # Extract x and y coordinates from path
        y_coords, x_coords = zip(*path)
        ax.plot(x_coords, y_coords, 'r-', linewidth=2)

        # Plot start and goal positions
        ax.plot(start_column, start_row, 'go', markersize=5)  # Start position in green
        ax.plot(goal_column, goal_row, 'bo', markersize=5)    # Goal position in blue

        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
