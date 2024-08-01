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
        angles_to_check = [80, 90]
        
        # Extract the distances for the specified angles and calculate CD distance
        CD = self.get_CD_distance(msg, angles_to_check)
        
        # Log the results and publish the CD distance
        if CD is not None:
            self.get_logger().info(f'CD distance to wall: {CD}')
            self.publish_CD_distance(CD)
        else:
            self.get_logger().warn('Could not find distances for both angles 80 and 90')

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
        
        # Retrieve distances for angles 80 and 90
        dist_80 = distances.get(80)
        dist_90 = distances.get(90)

        # Calculate CD distance if both distances are available
        if dist_80 is not None and dist_90 is not None:
            self.get_logger().info(f'Distances at 80 and 90 degrees found: {dist_80}, {dist_90}')
            angle_80_rad = math.radians(80)
            angle_diff_rad = math.radians(90 - 80)
            sin_80 = math.sin(angle_80_rad)
            cos_diff = math.cos(angle_diff_rad)

            # Compute alpha using the given formula
            alpha = math.atan((dist_80 * cos_diff - dist_90) / (dist_80 * sin_80))
            AB = dist_90 * math.cos(alpha)
            CD = AB + 3 * math.sin(alpha)
            self.get_logger().info(f'Calculated alpha: {alpha} radians, {math.degrees(alpha)} degrees')
        else:
            self.get_logger().warn('Distances for angles 80 or 90 not found.')
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
