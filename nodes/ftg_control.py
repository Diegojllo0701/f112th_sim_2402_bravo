#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class DigitalPIDControllerNode(Node):

    def __init__(self):
        super().__init__('digital_pid_controller_node')

        # Declare and initialize parameters
        self.declare_parameter('kp', 2.5)  # Proportional gain
        self.declare_parameter('ki', 0.1)  # Integral gain
        self.declare_parameter('kd', 0.01)  # Derivative gain
        self.declare_parameter('max_steering_angle', 10.0)  # Maximum steering angle (radians)
        self.declare_parameter('max_velocity', 1.8)  # Maximum linear velocity (m/s)
        self.declare_parameter('min_velocity', 0.5)  # Minimum linear velocity (m/s)
        self.declare_parameter('sampling_time', 0.05)  # Sampling time (seconds)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.min_velocity = self.get_parameter('min_velocity').value
        self.sampling_time = self.get_parameter('sampling_time').value

        # Initialize PID variables
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.previous_velocity = 0.0

        # Subscribe to the ftg_error topic
        self.error_subscriber = self.create_subscription(
            Float32,
            '/ftg_error',
            self.error_callback,
            10
        )

        # Publisher for robot velocity and steering
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # Timer to control the loop frequency
        self.timer = self.create_timer(self.sampling_time, self.control_loop)

        # Initialize the error for the control loop
        self.current_error = 0.0

    def error_callback(self, msg):
        self.current_error = msg.data

    def control_loop(self):
        # Calculate the PID control output
        proportional_term = self.kp * self.current_error
        self.integral_error += self.current_error * self.sampling_time
        integral_term = self.ki * self.integral_error
        derivative_term = self.kd * (self.current_error - self.previous_error) / self.sampling_time

        # PID output (steering angle)
        steering_angle = proportional_term + integral_term + derivative_term

        # Limit the steering angle
        steering_angle = max(min(steering_angle, self.max_steering_angle), -self.max_steering_angle)

        # Regulate the velocity based on the error
        velocity = self.max_velocity - (abs(self.current_error) / self.max_steering_angle) * (self.max_velocity - self.min_velocity)
        velocity = max(velocity, self.min_velocity)
        velocity= (velocity+self.previous_velocity)/2

        # Publish the velocity and steering commands
        twist_msg = Twist()
        twist_msg.linear.x = velocity
        twist_msg.angular.z = float(steering_angle)
        self.cmd_vel_publisher.publish(twist_msg)

        # Update previous error
        self.previous_error = self.current_error

def main(args=None):
    rclpy.init(args=args)
    node = DigitalPIDControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
