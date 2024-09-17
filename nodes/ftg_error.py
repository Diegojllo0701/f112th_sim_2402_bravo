#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np

class FollowTheGapNode(Node):

    def __init__(self):
        super().__init__('follow_the_gap_node')

        # Declare and assign parameters
        self.lidar_num_points = self.declare_parameter('lidar_num_points', 360).value
        self.safety_bubble_radius = self.declare_parameter('safety_bubble_radius', 1).value
        self.lidar_start_angle = self.declare_parameter('lidar_start_angle', -100).value
        self.lidar_end_angle = self.declare_parameter('lidar_end_angle', 100).value
        self.gap_selection_mode = self.declare_parameter('gap_selection_mode', 'widest').value
        self.gap_depth_threshold = self.declare_parameter('gap_depth_threshold', 1.0).value
        self.gap_width_threshold = self.declare_parameter('gap_width_threshold', 0.2).value
        self.no_gap_behavior = self.declare_parameter('no_gap_behavior', 'stop').value
        self.input_topic = self.declare_parameter('input_topic', '/scan').value
        self.output_topic = self.declare_parameter('output_topic', '/ftg_error').value
        self.max_steering_angle = self.declare_parameter('max_steering_angle', 1.0).value
        self.min_gap_points = self.declare_parameter('min_gap_points', 5).value

        # Subscribers and Publishers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )
        self.error_publisher = self.create_publisher(Float32, self.output_topic, 10)

    def scan_callback(self, msg):
        # Processing the LIDAR data
        ranges = np.array(msg.ranges)

        # Convert angles from -180 to 180 to array indices
        start_index = int((self.lidar_start_angle + 180) / 360 * self.lidar_num_points)
        end_index = int((self.lidar_end_angle + 180) / 360 * self.lidar_num_points)

        # Filter LIDAR data to only use the front-facing angles
        relevant_ranges = ranges[start_index:end_index]

        # Step 1: Find the nearest obstacle and its index
        min_distance = np.min(relevant_ranges)
        min_index = np.argmin(relevant_ranges)

        # Step 2: Calculate the angle of the closest point
        min_angle = self.lidar_start_angle + (min_index / len(relevant_ranges)) * (self.lidar_end_angle - self.lidar_start_angle)
        self.get_logger().info(f"Closest point at {min_distance}m and {min_angle} degrees")
        min_angle_rad = np.radians(min_angle)

        # Step 3: Determine which points fall inside the safety bubble
        for i, distance in enumerate(relevant_ranges):
            angle = self.lidar_start_angle + (i / len(relevant_ranges)) * (self.lidar_end_angle - self.lidar_start_angle)
            angle_rad = np.radians(angle)

            # Calculate the Euclidean distance from the closest point
            distance_to_closest_point = np.sqrt(
                min_distance**2 + distance**2 - 2 * min_distance * distance * np.cos(angle_rad - min_angle_rad)
            )

            # If the distance to the closest point is less than the safety bubble radius, mark as unsafe
            if distance_to_closest_point < self.safety_bubble_radius:
                relevant_ranges[i] = 0

        # Step 3: Find the maximum gap
        free_space_indices = np.where(relevant_ranges > 3.0)[0]
        if len(free_space_indices) == 0:
            self.handle_no_gaps()
            return

        gaps = np.split(free_space_indices, np.where(np.diff(free_space_indices) != 1)[0] + 1)
        
        # Biasing gaps towards the front
        front_angle = 0  # this is the forward direction in degrees
        def gap_bias(gap):
            mid_index = gap[len(gap) // 2]
            mid_angle = self.index_to_angle(mid_index, len(relevant_ranges))
            bias = np.cos(mid_angle)  # bias factor, higher value means closer to the front
            return len(gap) * bias  # combine gap width with bias

        best_gap = max(gaps, key=gap_bias)
        
        # Step 4: Select the target point
        if self.gap_selection_mode == "widest":
            target_index = best_gap[len(best_gap) // 2]
        elif self.gap_selection_mode == "deepest":
            max_depth = 0
            best_gap = None
            for gap in gaps:
                gap_depth = np.max(relevant_ranges[gap])
                if gap_depth > max_depth:
                    max_depth = gap_depth
                    best_gap = gap
            target_index = best_gap[len(best_gap) // 2]

        # Calculate the steering angle error
        angle_to_target = -self.index_to_angle(target_index, len(relevant_ranges))
        self.get_logger().info(f"Steering angle to target: {np.rad2deg(angle_to_target)} degrees")
        angle_error = np.clip(angle_to_target, -self.max_steering_angle, self.max_steering_angle)

        # Publish the angle error
        self.publish_error(angle_error)



    def index_to_angle(self, index, relevant_num_points):
        # Convert an index in the relevant_ranges to an angle in radians
        relevant_angle_range = self.lidar_end_angle - self.lidar_start_angle
        return np.radians(self.lidar_start_angle + (index / relevant_num_points) * relevant_angle_range)

    def publish_error(self, angle_error):
        error_msg = Float32()
        error_msg.data = angle_error
        self.get_logger().info(f"Publishing error: {angle_error}")
        self.error_publisher.publish(error_msg)

    def handle_no_gaps(self):
        if self.no_gap_behavior == 'stop':
            self.publish_error(0.0)  # Stop by setting error to zero
        elif self.no_gap_behavior == 'continue':
            pass  # Continue in the last known direction, potentially with last angle_error
        elif self.no_gap_behavior == 'last_direction':
            # Implement logic to continue in the last known direction
            pass

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()