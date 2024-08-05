#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.subscription = self.create_subscription(
            Float32,
            'error_signal',
            self.wall_distance_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.Kp = 5.0  # Proportional gain constant
        self.Kd = 0.1  # Derivative gain constant
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()
        self.use_derivative = False  # Set to False to disable derivative control
        self.linear_velocity = 0.5  # Constant linear velocity
        self.get_logger().info('WallFollower node has been started.')

    def wall_distance_callback(self, msg):
        current_error = msg.data
        current_time = self.get_clock().now()
        delta_time = (current_time - self.previous_time).nanoseconds / 1e9

        self.get_logger().info(f'Received wall distance error signal: {current_error}')

        # Proportional control
        control_signal = self.Kp * current_error

        if self.use_derivative and delta_time > 0:
            # Derivative control
            error_derivative = (current_error - self.previous_error) / delta_time
            control_signal += self.Kd * error_derivative
            self.get_logger().info(f'Error derivative: {error_derivative}, Control signal (PD): {control_signal}')
        else:
            self.get_logger().info(f'Control signal (P): {control_signal}')

        # Update previous error and time
        self.previous_error = current_error
        self.previous_time = current_time

        self.publish_control_signal(control_signal)

    def publish_control_signal(self, control_signal):
        msg = Twist()
        # Set constant linear velocity
        msg.linear.x = self.linear_velocity
        # Set angular velocity based on control signal
        msg.angular.z = control_signal
        if math.isnan(control_signal)==False:
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published control signal to cmd_vel_nav topic. Angular velocity: {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
