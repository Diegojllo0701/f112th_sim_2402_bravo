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

        # PID parameters
        self.Kp = 0.8  # Proportional gain
        self.Kd = 0.2  # Derivative gain
        self.Ki = 0.1  # Integral gain
        self.previous_error = 0.0
        self.integral_error = 0.0  # Integral accumulator
        self.previous_time = self.get_clock().now()

        # Control parameters
        self.linear_velocity = 0.28  # Constant linear velocity
        self.max_integral = 1.0  # Max integral windup limit
        self.rotating = False  # Flag to disable movement during rotation

        # Time settings
        self.sampling_time = self.declare_parameter('sampling_time', 0.025).value

        self.get_logger().info('WallFollower node has been started.')

    def wall_distance_callback(self, msg):
        if not self.rotating:
            current_error = msg.data
            current_time = self.get_clock().now()
            delta_time = (current_time - self.previous_time).nanoseconds / 1e9  # Time difference in seconds

            # PID Control
            control_signal = self.Kp * current_error

            # Update the integral (with anti-windup)
            self.integral_error += current_error * delta_time
            self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)  # Clamp the integral

            # Calculate derivative term
            if delta_time > 0:
                error_derivative = (current_error - self.previous_error) / delta_time
                control_signal += self.Kd * error_derivative
            else:
                error_derivative = 0.0

            # Add integral term to control signal
            control_signal += self.Ki * self.integral_error

            # Limit the control signal to prevent excessive rotation
            control_signal = max(min(control_signal, 1.0), -1.0)

            # Update the previous error and time for the next cycle
            self.previous_error = current_error
            self.previous_time = current_time

            # Publish the control signal (angular velocity)
            self.publish_control_signal(control_signal)

    def publish_control_signal(self, control_signal):
        if not self.rotating:
            msg = Twist()
            msg.linear.x = self.linear_velocity  # Constant forward velocity
            msg.angular.z = control_signal  # PID-controlled angular velocity

            if not math.isnan(control_signal):  # Ensure the control signal is valid
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published control signal: Linear velocity: {msg.linear.x}, Angular velocity: {msg.angular.z}')
            else:
                self.get_logger().warn('Control signal was NaN, not publishing.')

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
