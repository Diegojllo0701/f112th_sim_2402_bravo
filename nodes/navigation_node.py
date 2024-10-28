#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.qos import QoSProfile
import numpy as np
import math
import heapq
import threading

# Import the Obstacles message
from custom_msgs.msg import Obstacles  

expansion_size = 1  # For expanding obstacles in the costmap

def euler_from_quaternion(x, y, z, w):
    """
    Convert quaternion to Euler angles (yaw).
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(siny_cosp, cosy_cosp)
    return yaw_z

def distance(a, b):
    """
    Calculate Euclidean distance between two points.
    """
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def costmap(data, width, height, resolution):
    """
    Expand walls in the costmap based on the expansion_size.
    """
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

class PriorityQueue:
    """
    A priority queue that allows updating the priority of existing items.
    """
    def __init__(self):
        self.elements = []
        self.entry_finder = {}
        self.REMOVED = '<removed-task>'
        self.counter = 0

    def push(self, item, priority):
        if item in self.entry_finder:
            self.remove_item(item)
        count = self.counter
        entry = [priority, count, item]
        self.entry_finder[item] = entry
        heapq.heappush(self.elements, entry)
        self.counter += 1

    def remove_item(self, item):
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED

    def pop(self):
        while self.elements:
            priority, count, item = heapq.heappop(self.elements)
            if item is not self.REMOVED:
                del self.entry_finder[item]
                return priority, item
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return not any(item[2] is not self.REMOVED for item in self.elements)

class DStarLite:
    """
    Optimized implementation of the D* Lite algorithm for incremental path planning.
    """
    def __init__(self, start, goal, grid, heuristic=None):
        self.start = start  # (row, col)
        self.goal = goal    # (row, col)
        self.grid = grid    # 2D numpy array representing the occupancy grid
        self.heuristic = heuristic if heuristic else self.euclidean_distance

        # Initialize g and rhs values
        self.g = {}
        self.rhs = {}
        self.g[self.goal] = float('inf')
        self.rhs[self.goal] = 0

        # Initialize the priority queue
        self.U = PriorityQueue()
        self.U.push(self.goal, self.calculate_key(self.goal))

    def euclidean_distance(self, a, b):
        """
        Heuristic function: Euclidean distance.
        """
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def calculate_key(self, node):
        """
        Calculate the priority key for a node.
        """
        g_rhs = min(self.g.get(node, float('inf')), self.rhs.get(node, float('inf')))
        return (g_rhs + self.heuristic(self.start, node), g_rhs)

    def get_successors(self, node):
        """
        Get all reachable successors of a node.
        """
        neighbors = [(-1,0),(1,0),(0,-1),(0,1),
                     (-1,-1),(-1,1),(1,-1),(1,1)]
        successors = []
        for d in neighbors:
            succ = (node[0] + d[0], node[1] + d[1])
            if 0 <= succ[0] < self.grid.shape[0] and 0 <= succ[1] < self.grid.shape[1]:
                if self.grid[succ] == 0:  # Assuming 0 is free space
                    successors.append(succ)
        return successors

    def get_predecessors(self, node):
        """
        Get all reachable predecessors of a node.
        """
        # In grid-based maps, predecessors are the same as successors
        return self.get_successors(node)

    def cost(self, a, b):
        """
        Return the cost of moving from node a to node b.
        """
        if self.grid[b] == 100:  # Assuming 100 is occupied
            return float('inf')
        return distance(a, b)

    def update_vertex(self, u):
        """
        Update the vertex u in the planner.
        """
        if u != self.goal:
            self.rhs[u] = min([self.g.get(s, float('inf')) + self.cost(u, s) for s in self.get_successors(u)])
        self.U.push(u, self.calculate_key(u))

    def compute_shortest_path(self):
        """
        Compute the shortest path using the D* Lite algorithm.
        """
        while not self.U.empty():
            current_key, current_node = self.U.pop()
            current_key_new = self.calculate_key(current_node)
            if current_key < current_key_new:
                self.U.push(current_node, current_key_new)
                continue
            if self.g.get(current_node, float('inf')) > self.rhs.get(current_node, float('inf')):
                self.g[current_node] = self.rhs[current_node]
                for s in self.get_successors(current_node):
                    self.update_vertex(s)
            else:
                self.g[current_node] = float('inf')
                self.update_vertex(current_node)
                for s in self.get_successors(current_node):
                    self.update_vertex(s)

    def get_path(self):
        """
        Extract the path from start to goal.
        """
        path = []
        current = self.start
        if self.g.get(current, float('inf')) == float('inf'):
            return path  # No path exists

        while current != self.goal:
            path.append(current)
            neighbors = self.get_successors(current)
            if not neighbors:
                return []  # No path exists
            # Select the neighbor with the lowest g + cost
            min_cost = float('inf')
            next_node = current
            for s in neighbors:
                cost = self.g.get(s, float('inf')) + self.cost(current, s)
                if cost < min_cost:
                    min_cost = cost
                    next_node = s
            if next_node == current:
                return []  # Stuck, no path
            current = next_node
        path.append(self.goal)
        return path

class NavigationNode(Node):
    """
    ROS 2 Node for Navigation using D* Lite algorithm.
    """
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation Node Started')

        self.goal_x = []
        self.goal_y = []
        self.path_world = []  # Store path in world coordinates

        # Subscriptions
        qos_profile = QoSProfile(depth=10)
        self.subscription_map = self.create_subscription(
            OccupancyGrid, '/map', self.OccGrid_callback, qos_profile)
        self.subscription_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.Goal_Pose_callback, qos_profile)
        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, qos_profile)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # Subscribe to obstacle positions and velocities
        self.subscription_obstacles = self.create_subscription(
            Obstacles, 'detected_obstacles', self.obstacle_callback, qos_profile)

        # Robot's current position and orientation
        self.robot_pose_x = None
        self.robot_pose_y = None
        self.robot_yaw = None

        # Control parameters
        self.look_ahead_distance = 1.0  # Adjust as necessary
        self.max_linear_speed = 0.1     # Adjust as necessary
        self.max_angular_speed = 3.0    # Adjust as necessary
        self.safety_distance = 0.5       # Safety distance to obstacles

        # Timer for control loop
        self.control_timer = None

        # Obstacle positions and velocities
        self.obstacle_positions = []  # List of obstacles with positions and velocities

        # D* Lite planner
        self.planner = None  # Will be initialized when map and goal are available

        # Store the static map for reference
        self.static_map = None

        # Lock for thread-safe operations
        self.lock = threading.Lock()

    def OccGrid_callback(self, msg):
        """
        Callback for OccupancyGrid messages.
        """
        with self.lock:
            self.resolution = msg.info.resolution
            self.originX = msg.info.origin.position.x
            self.originY = msg.info.origin.position.y
            self.width = msg.info.width
            self.height = msg.info.height
            self.map_data = msg.data

            # Convert map data to costmap with expanded obstacles
            self.static_map = costmap(self.map_data, self.width, self.height, self.resolution)
            self.grid = np.array(self.static_map)

            self.get_logger().info("Occupancy Grid received and processed.")

            # Initialize D* Lite planner if goal is already set and robot pose is known
            if self.goal_x and self.goal_y and self.robot_pose_x is not None and self.robot_pose_y is not None:
                self.initialize_planner()

    def initialize_planner(self):
        """
        Initialize the D* Lite planner with the current map and goal.
        """
        # Define start and goal positions in grid indices
        start_col = int((self.robot_pose_x - self.originX) / self.resolution)
        start_row = int((self.robot_pose_y - self.originY) / self.resolution)
        goal_col = int((self.goal_x[-1] - self.originX) / self.resolution)
        goal_row = int((self.goal_y[-1] - self.originY) / self.resolution)

        start = (start_row, start_col)
        goal = (goal_row, goal_col)

        # Check if start and goal are within bounds
        if not (0 <= start_row < self.height and 0 <= start_col < self.width):
            self.get_logger().error("Start position is out of bounds.")
            return

        if not (0 <= goal_row < self.height and 0 <= goal_col < self.width):
            self.get_logger().error("Goal position is out of bounds.")
            return

        # Check if start or goal positions are in obstacles
        if self.grid[start_row][start_col] != 0:
            self.get_logger().error("Start position is in an obstacle.")
            return

        if self.grid[goal_row][goal_col] != 0:
            self.get_logger().error("Goal position is in an obstacle.")
            return

        # Initialize D* Lite planner
        self.planner = DStarLite(start, goal, self.grid)
        self.planner.compute_shortest_path()
        self.path = self.planner.get_path()
        self.get_logger().info(f"Path computed with {len(self.path)} nodes.")

        if not self.path:
            self.get_logger().error("No valid path found.")
            return

        # Convert path to world coordinates
        self.path_world = []
        for row, col in self.path:
            x = col * self.resolution + self.originX + self.resolution / 2.0
            y = row * self.resolution + self.originY + self.resolution / 2.0
            self.path_world.append((x, y))

        self.get_logger().info("Path converted to world coordinates.")

        # Start the control loop
        if self.control_timer:
            self.control_timer.cancel()
        self.control_timer = self.create_timer(0.1, self.pure_pursuit_control)

    def odom_callback(self, msg):
        """
        Callback for Odometry messages to update robot's pose.
        """
        with self.lock:
            self.robot_pose_x = msg.pose.pose.position.x
            self.robot_pose_y = msg.pose.pose.position.y
            orientation_q = msg.pose.pose.orientation
            self.robot_yaw = euler_from_quaternion(
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
            self.get_logger().debug(f"Updated robot pose to ({self.robot_pose_x}, {self.robot_pose_y}) with yaw {self.robot_yaw:.2f}")

    def obstacle_callback(self, msg):
        """
        Callback for Obstacles messages to update obstacle positions and velocities.
        """
        with self.lock:
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

            self.get_logger().info("Obstacles updated, checking for replanning...")

            # Replan if necessary
            if self.planner:
                self.update_costmap()
                self.planner.compute_shortest_path()
                self.path = self.planner.get_path()
                self.get_logger().info(f"Path updated with {len(self.path)} nodes.")

                if not self.path:
                    self.get_logger().error("No valid path found after replanning.")
                    return

                # Convert path to world coordinates
                self.path_world = []
                for row, col in self.path:
                    x = col * self.resolution + self.originX + self.resolution / 2.0
                    y = row * self.resolution + self.originY + self.resolution / 2.0
                    self.path_world.append((x, y))

    def Goal_Pose_callback(self, msg):
        """
        Callback for Goal Pose messages to set new navigation goals.
        """
        with self.lock:
            goal = msg.pose.position
            self.goal_x.append(goal.x)
            self.goal_y.append(goal.y)
            self.get_logger().info(f"Received new goal: ({goal.x}, {goal.y})")

            # Initialize planner if possible
            if self.static_map is not None and self.robot_pose_x is not None and self.robot_pose_y is not None:
                self.initialize_planner()

    def update_costmap(self):
        """
        Incorporate detected obstacles into the costmap by predicting their future positions.
        Filters out obstacles that coincide with static walls.
        """
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
                # Check if the cell is already occupied in the static map (wall)
                if self.static_map[row, col] == 100:
                    continue  # Skip marking to avoid redundant replanning due to walls

                # Mark the cell as occupied for dynamic obstacles
                self.grid[row, col] = 100

                # Optionally expand the obstacle in the costmap
                for i in range(-expansion_size, expansion_size + 1):
                    for j in range(-expansion_size, expansion_size + 1):
                        r = row + i
                        c = col + j
                        if 0 <= r < self.height and 0 <= c < self.width:
                            if self.static_map[r, c] != 100:  # Avoid overwriting static walls
                                self.grid[r, c] = 100

        # Update the planner's grid with the new dynamic obstacles
        self.planner.grid = self.grid

    def pure_pursuit_control(self):
        """
        Control loop using Pure Pursuit algorithm to follow the planned path.
        """
        with self.lock:
            if not self.path_world or self.robot_pose_x is None or self.robot_pose_y is None:
                return

            # Current look-ahead point is the first point in path_world initially
            if not hasattr(self, 'current_index'):
                self.current_index = 0

            # Check if the robot has reached the current look-ahead point
            if self.current_index >= len(self.path_world):
                self.get_logger().info("Goal reached!")
                self._stop_robot()
                return

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
            self.get_logger().debug(f"Angle Error: {angle_error:.2f}")

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
        """
        Stop the robot and cancel the control timer.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        if self.control_timer:
            self.control_timer.cancel()
        self.get_logger().info("Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Navigation node terminated by user.")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
