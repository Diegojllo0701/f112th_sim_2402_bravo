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
        angles_to_check = [-180, -90,-80,-10,0, 10, 80,90]
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
            
            # Debug information
            #self.get_logger().info('Angle: {} degrees, {} radians, index: {}'.format(angle, angle_rad, index))
            #self.get_logger().info('Angle min: {}, Angle max: {}, Angle increment: {}'.format(msg.angle_min, msg.angle_max, msg.angle_increment))
            #self.get_logger().info('Range size: {}'.format(len(msg.ranges)))

            # Make sure the index is within the range of the array
            if 0 <= index < len(msg.ranges):
                distances.append(msg.ranges[index])
            else:
                distances.append(float('inf')) # If the index is out of range, append infinity
        return distances

    def publish_distances(self, distances):
        msg = AngleDistance()
        msg.distance_back = distances[0]
        msg.distances_left = [distances[6], distances[7]]
        msg.distances_front = [distances[3],distances[4],distances[5]]
        msg.distances_right = [distances[1], distances[2]]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    scan_reader = ScanReader()
    rclpy.spin(scan_reader)
    scan_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
