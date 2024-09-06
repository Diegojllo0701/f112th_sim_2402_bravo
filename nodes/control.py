#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from f112th_sim_2402_bravo.msg import AngleDistance  # Asegúrate de importar el mensaje correcto
import time
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
        self.Kp = 0.5  # Constante proporcional
        self.Kd = 0  # Constante derivativa
        self.ki = 0.2  # Constante integrativa
        self.previous_error = 0.0
        self.previous_time = self.get_clock().now()
        self.linear_velocity = 0.5  # Velocidad lineal constante
        self.angular_velocity = math.pi / 4  # Velocidad angular para giros
        self.rotation_duration = math.pi / 8.5 / self.angular_velocity  # Duración para girar 90 grados

        self.rotating = False
        self.get_logger().info('WallFollower node has been started.')

    def wall_distance_callback(self, msg):
        if not self.rotating:
            current_error = msg.data
            current_time = self.get_clock().now()
            delta_time = (current_time - self.previous_time).nanoseconds / 1e9

            # Control proporcional-derivativo
            control_signal = self.Kp * current_error
            error_integrative = delta_time * current_error

            if delta_time > 0:
                error_derivative = (current_error - self.previous_error) / delta_time
                control_signal += self.Kd * error_derivative + self.ki * error_integrative

            # Actualizar el error y el tiempo previo
            self.previous_error = current_error
            self.previous_time = current_time

        # Update previous error and time
        self.previous_error = current_error
        self.previous_time = current_time
        self.control_sig = control_signal

    def angle_distances_callback(self, msg):
        if not self.rotating:
            distances_right = msg.distances_right[0]
            distance_front = msg.distances_front[1]
            distance_left = msg.distances_left[0]

            # Log para depuración
            self.get_logger().info(f'Right: {distances_right}, Front: {distance_front}, Left: {distance_left}')
                
            if distance_front > 0.8:  # No hay pared enfrente
                self.avance_lineal()
            elif distance_left > (distances_right+distance_left)/2:  # Detecta un callejón a la izquierda
                self.girar_izquierda()
            elif distances_right > (distances_right+distance_left)/2:  # Mantener la distancia con la pared derecha
                self.girar_derecha()

    def girar_derecha(self):
        self.get_logger().info('Girando a la derecha.')
        self.rotar(-self.angular_velocity)

    def girar_izquierda(self):
        self.get_logger().info('Girando a la izquierda.')
        self.rotar(self.angular_velocity)

    def rotar(self, angular_velocity):
        self.rotating = True
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angular_velocity
        self.publisher_.publish(msg)
        time.sleep(self.rotation_duration)
        self.stop_rotation()

    def stop_rotation(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.rotating = False
        self.get_logger().info('Rotación completada.')

    def avance_lineal(self):
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Avanzando en línea recta.')
        self.rotating = False

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