#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
import math

class AngleDistancesReader(Node):
    def __init__(self):
        super().__init__('angle_distances_reader')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'angle_distances',
            self.angle_distances_callback,
            10)
        self.publisher_ = self.create_publisher(Float32, 'wall_distance', 10)
        self.get_logger().info('AngleDistancesReader node has been started.')

    def angle_distances_callback(self, msg):
        self.get_logger().info('Received angle distances message.')
        # Define the angles we are interested in
        angles_to_check = [100, 90]
        
        # Extract the distances for the specified angles and calculate CD distance
        CD = self.get_CD_distance(msg, angles_to_check)
        
        # Log the results and publish the CD distance
        if CD is not None:
            self.get_logger().info(f'CD distance to wall: {CD}')
            self.publish_CD_distance(CD)
        else:
            self.get_logger().warn('Could not find distances for both angles')

    def get_CD_distance(self, msg, angles):
        distances = {}
        data = msg.data

        # Extract distance values for the specified angles
        for i in range(0, len(data), 2):
            angle = data[i]
            distance = data[i + 1]
            self.get_logger().info(f'Angle: {angle}, Distance: {distance}')
            if angle in angles:
                distances[angle] = distance
        
        # Retrieve distances for angles 100 and 90
        angle1=100
        angle2=90
        a = distances.get(angle1)
        b = distances.get(angle2)

        # Calculate CD distance if both distances are available
        if a is not None and b is not None:
            self.get_logger().info(f'Distances at {angle1} and {angle2} degrees found: {a}, {b}')
            angle_1_rad = math.radians(angle1)
            angle_diff_rad = math.radians(angle1 - angle2)
            sin_diff = math.sin(angle_diff_rad)
            cos_diff = math.cos(angle_diff_rad)
            AC=3
            # Compute alpha using the given formula
            alpha = math.atan((a * cos_diff - b) / (a * sin_diff))
            AB = b * math.cos(alpha)
            CD = AB + AC * math.sin(alpha)
            self.get_logger().info(f'Calculated alpha: {alpha} radians, {math.degrees(alpha)} degrees')
        else:
            self.get_logger().warn(f'Distances for angles {angle1} or {angle2} not found.')
            CD = None

        return CD

    def publish_CD_distance(self, CD):
        msg = Float32()
        msg.data = CD
        self.publisher_.publish(msg)
        self.get_logger().info('Published CD distance to wall_distance topic.')

def main(args=None):
    rclpy.init(args=args)
    angle_distances_reader = AngleDistancesReader()
    rclpy.spin(angle_distances_reader)
    angle_distances_reader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
