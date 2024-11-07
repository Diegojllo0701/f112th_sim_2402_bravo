#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry  # Import Odometry message type
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.duration import Duration
import math
import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Subscribe to the /robot2/pose topic
        self.subscription = self.create_subscription(
            Pose,
            '/robot2/pose',
            self.pose_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        # Create a TransformBroadcaster for dynamic transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a StaticTransformBroadcaster for laser_frame
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publisher for the /odom topic
        self.odom_publisher = self.create_publisher(Odometry, '/bravo_odom', 10)
        
        # Timer setup for periodic callbacks
        self.sampling_time = 0.1  # Timer interval in seconds
        self.timer = self.create_timer(self.sampling_time, self.timer_callback)
        
        # Initialize variables to store the latest pose
        self.latest_pose = Pose()
        self.frame_id = 'bravo_odom'             # Parent frame ID for Odometry
        self.child_frame_id = 'bravo_base_link'  # Child frame ID for Odometry
        
        self.get_logger().info('OdomTfPublisher node has been started.')

    def pose_callback(self,msg):
        self.latest_pose = msg
        current_time = self.get_clock().now()
        
        # Broadcast the dynamic transform from odom to base_link
        t = TransformStamped()

        # Populate TransformStamped message
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        # Set translation from the latest_pose
        t.transform.translation.x = self.latest_pose.position.x
        t.transform.translation.y = self.latest_pose.position.y
        t.transform.translation.z = self.latest_pose.position.z

        # Set rotation from the latest_pose
        t.transform.rotation = self.latest_pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f'Broadcasted transform from {t.header.frame_id} to {t.child_frame_id}')

        # Publish the /odom message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = 'laser'

        # Set the pose
        odom_msg.pose.pose = self.latest_pose

        # Initialize Pose covariance (identity matrix scaled by a small number or set to zero)
        # Here, we set it to zero, indicating unknown covariance
        odom_msg.pose.covariance = [0.0] * 36

        # Initialize Twist (linear and angular velocities)
        # If you have velocity data, populate these fields accordingly
        # For now, we'll set them to zero
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = 0.0

        # Initialize Twist covariance (set to zero, indicating unknown)
        odom_msg.twist.covariance = [0.0] * 36

        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)
        self.get_logger().debug('Published /odom message.')

        # Broadcast the static transform from base_link to laser_frame
        static_t = TransformStamped()
        
        # Populate StaticTransformStamped message
        static_t.header.stamp = current_time.to_msg()
        static_t.header.frame_id = self.child_frame_id
        static_t.child_frame_id = 'laser'
        
        # Set the translation based on the origin offset
        static_t.transform.translation.x = -0.04
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.1
        
        q=quaternion_from_euler(0,0,math.pi)
        # Set the rotation (no rotation, so quaternion is (0,0,0,1))
        static_t.transform.rotation.x = q[0]
        static_t.transform.rotation.y = q[1]
        static_t.transform.rotation.z = q[2]
        static_t.transform.rotation.w = q[3]
        
        # Broadcast the static transform
        self.static_tf_broadcaster.sendTransform(static_t)
        self.get_logger().debug('Published static transform from base_link to laser_frame')


def main(args=None):
    rclpy.init(args=args)
    node = OdomTfPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
