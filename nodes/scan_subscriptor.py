#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from f112th_sim_2402_bravo.msg import AngleDistance

class ScanReader(Node):
    def __init__(self):
        super().__init__('scan_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(AngleDistance, 'angle_distances', 10)

    def scan_callback(self, msg):
        # Agregar los ángulos que necesitas para los cálculos de 75°, 90° y 105° para la derecha
        angles_to_check = [-180, -105, -90, -75, -10, 0, 10, 75, 90, 105]
        distances = self.get_distances_at_angles(msg, angles_to_check)
        self.get_logger().info('Distances at angles {}: {}'.format(angles_to_check, distances))
        self.publish_distances(distances)

    def get_distances_at_angles(self, msg, angles):
        distances = []
        for angle in angles:
            # Convert angle to radians
            angle_rad = angle * 3.14159265 / 180.0
            # Calculate the index
            index = int((angle_rad - msg.angle_min) / msg.angle_increment)

            # Make sure the index is within the range of the array
            if 0 <= index < len(msg.ranges):
                distances.append(msg.ranges[index])
            else:
                distances.append(float('inf'))  # If the index is out of range, append infinity
        return distances

    def publish_distances(self, distances):
        msg = AngleDistance()

        # Ajuste de las distancias para izquierda, frente y derecha, asegurando 3 distancias para la derecha
        msg.distance_back = distances[0]
        msg.distances_left = [distances[7], distances[8]]  # Ángulos 75° y 90°
        msg.distances_front = [distances[4], distances[5], distances[6]]  # Ángulos -10°, 0°, 10°
        msg.distances_right = [distances[1], distances[2], distances[3]]  # Ángulos 105°, 90°, 75°

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    scan_reader = ScanReader()
    rclpy.spin(scan_reader)
    scan_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
