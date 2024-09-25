#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import math

class TransformNode(Node):
    def __init__(self):
        super().__init__('transform_node')
        self.br = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel_joy', self.cmd_callback, 10)
        
        # Initialize variables for the moving frame
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Publish the static transform at the start
        self.publish_static_transform()

    def publish_static_transform(self):
        t_static = TransformStamped()
        t_static.header.stamp = self.get_clock().now().to_msg()
        t_static.header.frame_id = 'world'
        t_static.child_frame_id = 'static_frame'
        
        # Set static transform parameters
        t_static.transform.translation.x = 0.0
        t_static.transform.translation.y = 0.0
        t_static.transform.translation.z = 0.0
        t_static.transform.rotation.x = 0.0
        t_static.transform.rotation.y = 0.0
        t_static.transform.rotation.z = 0.0
        t_static.transform.rotation.w = 1.0  # Neutral quaternion
        
        self.br.sendTransform(t_static)

    def cmd_callback(self, msg):
        # Update current_yaw (angular.z)
        self.current_yaw += msg.angular.z

        # Calculate the movement in the local x-direction of the moving frame
        delta_x = msg.linear.x * math.cos(self.current_yaw)
        delta_y = msg.linear.x * math.sin(self.current_yaw)

        self.current_x += delta_x
        self.current_y += delta_y

        # Publish the updated moving transform
        self.publish_moving_transform()

    def publish_moving_transform(self):
        t_moving = TransformStamped()
        t_moving.header.stamp = self.get_clock().now().to_msg()
        t_moving.header.frame_id = 'static_frame'
        t_moving.child_frame_id = 'moving_frame'

        # Set the dynamic transform
        t_moving.transform.translation.x = self.current_x
        t_moving.transform.translation.y = self.current_y
        t_moving.transform.translation.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(self.current_yaw / 2.0)
        qw = math.cos(self.current_yaw / 2.0)
        t_moving.transform.rotation.x = 0.0
        t_moving.transform.rotation.y = 0.0
        t_moving.transform.rotation.z = qz
        t_moving.transform.rotation.w = qw

        # Broadcast the moving transform
        self.br.sendTransform(t_moving)

def main(args=None):
    rclpy.init(args=args)
    node = TransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


