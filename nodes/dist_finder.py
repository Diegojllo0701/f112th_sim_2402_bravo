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
        
        # Uso de las distancias derecha para el cálculo (75°, 90°, 105°)
        if len(msg.distances_right) >= 3:
            d_75 = msg.distances_right[0]
            d_90 = msg.distances_right[1]
            d_105 = msg.distances_right[2]
            
            CD, alpha = self.calculate_CD_distance(d_75, d_105)
            error = self.calculate_error(CD, alpha, d_90)
            
            if error is not None:
                self.get_logger().info(f'error_signal: {error}')
                self.publish_error(error)
            else:
                self.get_logger().warn(f'Could not calculate error signal.')
        else:
            self.get_logger().warn('Not enough distances_right values to calculate CD distance.')

    def calculate_CD_distance(self, d_75, d_105):
        angle_diff = math.radians(30)  # Diferencia entre 75° y 105°

        if d_75 < 30 and d_105 < 30:
            try:
                # Calcular el ángulo alpha
                alpha = math.atan2(d_75 - d_105, 0.3)  # Distancia aproximada entre los puntos en el eje X
                # Calcular la distancia CD a la pared como el promedio de d_75 y d_105
                CD = (d_75 + d_105) / 2
                self.get_logger().info(f'Calculated alpha: {alpha} radians, {math.degrees(alpha)} degrees, CD distance: {CD}')
                return CD, alpha
            except ZeroDivisionError:
                self.get_logger().error('ZeroDivisionError: Cannot calculate alpha.')
                return None, None
        else:
            return 0, 0.1  # Valores por defecto si las mediciones son erróneas

    def calculate_error(self, CD, alpha, d_90):
        target_distance = 0.75  # Distancia deseada de la pared
        error = target_distance - CD
        self.get_logger().info(f'Calculated error: {error}')    
        return error

    def publish_error(self, error):
        msg = Float32()
        msg.data = error
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
