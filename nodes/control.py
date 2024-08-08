#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
import time
import threading
import math

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.subscription = self.create_subscription(
            Float32,
            'error_signal',
            self.wall_distance_callback,
            10)
        self.break_subscription = self.create_subscription(
            Bool,
            'brake_state',
            self.break_state_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.Kp = 5.0  # Proportional gain constant
        self.Kd = 0.1  # Derivative gain constant
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()
        self.use_derivative = False  # Set to False to disable derivative control
        self.linear_velocity = 1.0  # Constant linear velocity, ensuring it's a float
        self.angular_velocity = math.pi / 4  # 45 degrees per second for rotation
        self.rotation_duration = math.pi / 2 / self.angular_velocity  # Duration to rotate 90 degrees
        self.get_logger().info('WallFollower node has been started.')

        self.brake_state = False
        self.rotating = False

    def break_state_callback(self, msg):
        if msg.data != self.brake_state:
            self.brake_state = msg.data
            if self.brake_state and not self.rotating:
                self.get_logger().info('Received brake state signal: True')
                self.start_rotation()
            elif not self.brake_state:
                self.get_logger().info('Received brake state signal: False')

    def wall_distance_callback(self, msg):
        if not self.brake_state and not self.rotating:  # Only process wall distance if brake is not active
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

    def start_rotation(self):
        self.rotating = True
        self.get_logger().info('Starting rotation')
        rotation_thread = threading.Thread(target=self.perform_rotation)
        rotation_thread.start()

    def perform_rotation(self):
        msg = Twist()
        msg.linear.x = 0.0  # Stop linear movement
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        self.get_logger().info('Continuing 90-degree rotation.')
        time.sleep(self.rotation_duration)
        self.stop_rotation()

    def stop_rotation(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Completed 90-degree rotation.')
        self.rotating = False

    def publish_control_signal(self, control_signal):
        if not self.brake_state and not self.rotating:  # Only publish control signals if brake is not active
            msg = Twist()
            # Set constant linear velocity
            msg.linear.x = float(self.linear_velocity)  # Ensuring it's a float
            # Set angular velocity based on control signal
            msg.angular.z = float(control_signal)  # Ensuring it's a float
            if not math.isnan(control_signal):
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
