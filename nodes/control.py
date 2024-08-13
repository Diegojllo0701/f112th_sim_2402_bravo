#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from f112th_sim_2402_bravo.msg import AngleDistance  # Ensure to import the correct message
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
        self.angle_subscription = self.create_subscription(
            AngleDistance,
            'angle_distances',
            self.angle_distances_callback,
            10)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        self.Kp = 1.5 # Proportional gain constant
        self.Kd = 0.2# Derivative gain constant
        self.ki = 1 # Integrative gain constant
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()
        self.use_derivative = True # Set to False to disable derivative control
        self.linear_velocity = 0.5# Constant linear velocity, ensuring it's a float
        self.angular_velocity = math.pi/10 # 45 degrees per second for rotation
        self.rotation_duration = math.pi /10/ self.angular_velocity  # Duration to rotate 90 degrees
        self.get_logger().info('WallFollower node has been started.')

        self.rotating = False

    def wall_distance_callback(self, msg):
        if not self.rotating:  # Only process wall distance if not rotating
            current_error = msg.data
            self.error = current_error
            current_time = self.get_clock().now()
            delta_time = (current_time - self.previous_time).nanoseconds / 1e9

            self.get_logger().info(f'Received wall distance error signal: {current_error}')

            # Proportional control
            control_signal = self.Kp * current_error
            error_integrative = delta_time*current_error

            if self.use_derivative and delta_time > 0:
                # Derivative control
                error_integrative+=error_integrative
                error_derivative = (current_error - self.previous_error) / delta_time
                control_signal += self.Kd * error_derivative + self.ki*error_integrative
                self.get_logger().info(f'Error derivative: {error_derivative}, Control signal (PD): {control_signal}')
            else:
                self.get_logger().info(f'Control signal (P): {control_signal}')

            # Update previous error and time
            self.previous_error = current_error
            self.previous_time = current_time

            self.publish_control_signal(control_signal)

    def angle_distances_callback(self, msg):
        if not self.rotating:
            distances_right = msg.distances_right[0]
            distance_front = msg.distance_front
            distances_left = msg.distances_left[0]

            self.get_logger().info(f'Received angle distances: right: {distances_right}, front: {distance_front}, left: {distances_left}')

            if distances_right>1.8:  # No wall to the right
                self.girar_derecha()
            elif distance_front > 0.5:  # No wall in front
                return  # Continue forward, handled by wall_distance_callback
            elif distances_left>1.8:  # No wall to the left
                self.girar_izquierda()
            else:  # Wall on all sides
                self.dar_media_vuelta()


    def girar_derecha(self):
        self.get_logger().info('Turning right.')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = -self.angular_velocity
        self.publisher_.publish(msg)
        time.sleep(self.rotation_duration)
        self.stop_rotation()

    def girar_izquierda(self):
        self.get_logger().info('Turning left.')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        time.sleep(self.rotation_duration)
        self.stop_rotation()

    def dar_media_vuelta(self):
        self.get_logger().info('Turning around.')
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = self.angular_velocity
        self.publisher_.publish(msg)
        time.sleep(2 * self.rotation_duration)
        self.stop_rotation()

    def stop_rotation(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Completed rotation.')
        self.rotating = False

    def publish_control_signal(self, control_signal):
        if not self.rotating:  # Only publish control signals if not rotating
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
