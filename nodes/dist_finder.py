#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from f112th_sim_2402_bravo.msg import AngleDistance  # Asegúrate de que la importación sea correcta
import math

class AngleDistancesReader(Node):
    def __init__(self):
        super().__init__('angle_distances_reader')
        self.subscription = self.create_subscription(
            AngleDistance,
            'angle_distances',
            self.angle_distances_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'error_signal', 10)
        self.get_logger().info('AngleDistancesReader node has been started.')

    def angle_distances_callback(self, msg):
        self.get_logger().info('Received angle distances message.')
        
        # Use distances_right values as a and b
        if len(msg.distances_right) >= 2:
            a = msg.distances_right[0]
            b = msg.distances_right[1]
            CD,alpha = self.calculate_CD_distance(a, b)
            error = self.calculate_error(CD,alpha)
            if error is not None:
                self.get_logger().info(f'error_signal: {error}')
                self.publish_CD_distance(error)
            else:
                self.get_logger().warn(f'Could not calculate error signal.error_signal: {error}')
        else:
            self.get_logger().warn('Not enough distances_right values to calculate CD distance.')

    def calculate_CD_distance(self, a, b):
        angle1 = 100
        angle2 = 90

        angle_1_rad = math.radians(angle1)
        angle_diff_rad = math.radians(angle1 - angle2)
        sin_diff = math.sin(angle_diff_rad)
        cos_diff = math.cos(angle_diff_rad)
        AC = 1

        # Compute alpha using the given formula
        if a<30 and b<30:
            try:
                alpha = math.atan((a * cos_diff - b) / (a * sin_diff))
                AB = b * math.cos(alpha)
                CD = AB - AC * math.sin(alpha)#verificar signo
                self.get_logger().info(f'Calculated alpha: {alpha} radians, {math.degrees(alpha)} degrees, CD distance: {CD}')
                return CD, alpha
            except ZeroDivisionError:
                self.get_logger().error('ZeroDivisionError: sin_diff is zero, cannot calculate alpha.')
                return None, None
        else:
            CD =0
            alpha=0.1
            return CD,alpha


    def calculate_error(self, CD, alpha):
        fixed_distance=1.5
        error=CD-fixed_distance
        self.get_logger().info(f'Calculated error: {error}')    
        return error


    def publish_CD_distance(self, CD):
        msg = Float32()
        msg.data = CD
        self.publisher_.publish(msg)
        self.get_logger().info('Published error to error_signal topic.')

def main(args=None):
    rclpy.init(args=args)
    angle_distances_reader = AngleDistancesReader()
    rclpy.spin(angle_distances_reader)
    angle_distances_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
