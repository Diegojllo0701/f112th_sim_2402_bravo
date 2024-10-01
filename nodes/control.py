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
        self.Kp = 1  # Constante proporcional
        self.Kd = 0.1  # Constante derivativa
        self.ki = 0.2  # Constante integrativa
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()
        self.linear_velocity = 0.5  # Velocidad lineal constante
        self.rotating = False
        self.declare_parameter('sampling_time', 0.025)

        self.get_logger().info('WallFollower node has been started.')

    def wall_distance_callback(self, msg):
        if not self.rotating:
            current_error = msg.data
            current_time = self.get_clock().now()
            delta_time = (current_time - self.previous_time).nanoseconds / 1e9

            # Controlador PID
            control_signal = self.Kp * current_error
            error_integrative = self.ki * (self.previous_error + current_error) * delta_time

            if delta_time > 0:
                error_derivative = (current_error - self.previous_error) / delta_time
                control_signal += self.Kd * error_derivative + error_integrative

            # Limitar la señal de control
            control_signal = max(min(control_signal, 1.0), -1.0)

            # Actualizar el error y el tiempo previo
            self.previous_error = current_error
            self.previous_time = current_time

            # Publicar la señal de control
            self.publish_control_signal(control_signal)

    def publish_control_signal(self, control_signal):
        if not self.rotating:
            msg = Twist()
            msg.linear.x = self.linear_velocity
            msg.angular.z = control_signal
            if not math.isnan(control_signal):
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publicado señal de control: Angular velocity: {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
