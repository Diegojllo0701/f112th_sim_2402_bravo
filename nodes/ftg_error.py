#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt

class FollowTheGapNode(Node):

    def __init__(self):
        super().__init__('follow_the_gap_node')

        # Declare and assign parameters
        self.lidar_num_points = self.declare_parameter('lidar_num_points', 360).value
        self.safety_bubble_radius = self.declare_parameter('safety_bubble_radius', 1).value
        self.lidar_start_angle = -100  # We want to keep the range from -100 to 100 degrees
        self.lidar_end_angle = 100
        self.gap_selection_mode = self.declare_parameter('gap_selection_mode', 'widest').value
        self.max_lidar_range = self.declare_parameter('max_lidar_range', 10.0).value  # Maximum range of LIDAR
        self.input_topic = self.declare_parameter('input_topic', '/scan').value
        self.output_topic = self.declare_parameter('output_topic', '/ftg_error').value
        self.max_steering_angle = self.declare_parameter('max_steering_angle', 1.0).value
        self.no_gap_behavior = self.declare_parameter('no_gap_behavior', 'stop').value
        self.enable_graphing = self.declare_parameter('enable_graphing', True).value  # Enable or disable graphing

        # Subscribers and Publishers
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            self.input_topic,
            self.scan_callback,
            10
        )
        self.error_publisher = self.create_publisher(Float32, self.output_topic, 10)

        # Enable interactive plotting mode
        if self.enable_graphing:
            plt.ion()

    def scan_callback(self, msg):
        # Use angles from -100 to 100 degrees, but calculate based on actual angle_min and angle_increment
        angles_degrees = np.linspace(self.lidar_start_angle, self.lidar_end_angle, self.lidar_num_points)
        distances = self.get_distances_in_angle_range(msg, self.lidar_start_angle, self.lidar_end_angle)

        # Ensure both arrays have the same size
        assert len(angles_degrees) == len(distances), "Mismatch between angles and distances"

        # Log the front-facing distances for debugging (around 0 degrees)
        mid_index = len(distances) // 2  # This should correspond to 0 degrees
        self.get_logger().info(f"Distance at 0 degrees (front): {distances[mid_index]}")

        # Use the distances and relevant logic for gap detection
        safe_ranges = np.array(distances)

        # Find free space (gaps) and split the gaps based on proximity
        free_space_indices = np.where(safe_ranges > 3.0)[0]
        if len(free_space_indices) == 0:
            self.handle_no_gaps()
            return

        gaps = np.split(free_space_indices, np.where(np.diff(free_space_indices) != 1)[0] + 1)

        # Select the best gap: widest or deepest
        if self.gap_selection_mode == 'widest':
            # Widest gap logic (selecting the gap with the most points)
            best_gap = max(gaps, key=len)
        elif self.gap_selection_mode == 'deepest':
            # Deepest gap logic (selecting the gap with the maximum depth)
            best_gap = max(gaps, key=lambda gap: np.max(safe_ranges[gap]))

        # Target the middle of the best gap
        target_index = best_gap[len(best_gap) // 2]

        # Calculate the steering angle to the middle of the gap
        target_angle = angles_degrees[target_index]
        self.get_logger().info(f"Steering angle to target: {target_angle} degrees")

        # Convert the steering angle to radians and calculate the error
        angle_error = np.radians(target_angle)
        angle_error = np.clip(angle_error, -self.max_steering_angle, self.max_steering_angle)

        # Publish the angle error
        self.publish_error(angle_error)

        # Plot the data if graphing is enabled
        if self.enable_graphing:
            self.plot_data(angles_degrees, safe_ranges, free_space_indices)


    def get_distances_in_angle_range(self, msg, start_angle_deg, end_angle_deg):
        """ Get distances from the LIDAR data for angles between start_angle_deg and end_angle_deg """
        distances = []
        angles_to_check = np.linspace(start_angle_deg, end_angle_deg, self.lidar_num_points)
        for angle_deg in angles_to_check:
            # Convert angle to radians
            angle_rad = angle_deg * np.pi / 180.0
            # Calculate the index using angle_min and angle_increment from the LaserScan message
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)
            
            # Make sure the index is within the range of the array
            if 0 <= index < len(msg.ranges):
                distances.append(msg.ranges[index])
            else:
                distances.append(float('inf'))  # If the index is out of range, append infinity
        return distances

    def plot_data(self, angles_degrees, relevant_ranges, free_space_indices):
        """ Plot LIDAR data as points in a linear plot, highlight gaps """
        plt.clf()  # Clear previous plot

        # Plot LIDAR points in blue
        plt.plot(angles_degrees, relevant_ranges, 'b.', label='LIDAR Points')

        # Highlight the gaps in orange
        plt.plot(angles_degrees[free_space_indices], relevant_ranges[free_space_indices], 'o', color='orange', label='Gap')

        # Label the axes and set limits
        plt.xlabel('Angle (degrees)')
        plt.ylabel('Distance (m)')
        plt.title('LIDAR Data with Gaps')
        plt.xlim([self.lidar_start_angle, self.lidar_end_angle])  # Show the full degree range
        plt.ylim([0, self.max_lidar_range])  # Limit the range to make all points visible

        # Finalize plot
        plt.legend()
        plt.draw()
        plt.pause(0.001)

    def publish_error(self, angle_error):
        error_msg = Float32()
        error_msg.data = angle_error
        self.get_logger().info(f"Publishing error: {angle_error}")
        self.error_publisher.publish(error_msg)

    def handle_no_gaps(self):
        """Handle the behavior when no gaps are found."""
        if self.no_gap_behavior == 'stop':
            self.publish_error(0.0)  # Stop by setting error to zero
            #self.get_logger().info('No gaps found, stopping the robot.')
        elif self.no_gap_behavior == 'continue':
            #self.get_logger().info('No gaps found, continuing the last known direction.')
            pass
        elif self.no_gap_behavior == 'last_direction':
            #self.get_logger().info('No gaps found, using the last known direction.')
            pass

    def destroy_node(self):
        """ Clean up the node and ensure plot closes properly. """
        if self.enable_graphing:
            plt.close()  # Ensure matplotlib window is closed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
