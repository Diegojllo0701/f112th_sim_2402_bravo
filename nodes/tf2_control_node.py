#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from nav_msgs.msg import Odometry  # Import Odometry message type
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.duration import Duration
import tf_transformations
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
        self.subscription = self.create_subscription(
            Pose,
            '/robot1/pose',
            self.pose_callback,
            10)
        self.subscription 
        # Create a TransformBroadcaster for dynamic transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a StaticTransformBroadcaster for laser_frame
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Timer setup for periodic callbacks
        self.sampling_time = 0.1  # Timer interval in seconds
        # Initialize variables to store the latest pose
        self.latest_pose = Pose()
        self.frame_id = 'odom'             # Parent frame ID for Odometry
        self.child_frame_id = 'base_footprint'  # Child frame ID for Odometry
        self.get_logger().info('OdomTfPublisher node has been started.')
        self.create_publisher(Odometry, '/odom', 10)

    def pose_callback(self,msg):
        current_time = self.get_clock().now()
        
        # Broadcast the dynamic transform from odom to base_link
        t = TransformStamped()

        # Populate TransformStamped message
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id

        # Set the translation based on the received pose
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y 
        t.transform.translation.z = msg.position.z
        self.get_logger().info(f'Broadcasted transform odom_base with: {msg.position} {msg.orientation}')

        # Set the rotation based on the received orientation
        t.transform.rotation = msg.orientation
        
        q0=tf_transformations.quaternion_from_euler(0,0,0)
        # Set the rotation (no rotation, so quaternion is (0,0,0,1))
        t.transform.rotation = msg.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug(f'Broadcasted transform from {t.header.frame_id} to {t.child_frame_id}')

        # Broadcast the static transform from base_link to laser_frame
        static_t = TransformStamped()
        
        # Populate StaticTransformStamped message
        static_t.header.stamp = current_time.to_msg()
        static_t.header.frame_id = 'base_footprint'
        static_t.child_frame_id = 'laser'
        
        # Set the translation based on the origin offset
        static_t.transform.translation.x = -0.04
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.1
        
        q=tf_transformations.quaternion_from_euler(0,0,math.pi)
        # Set the rotation (no rotation, so quaternion is (0,0,0,1))
        static_t.transform.rotation.x = q[0]
        static_t.transform.rotation.y = q[1]
        static_t.transform.rotation.z = q[2]
        static_t.transform.rotation.w = q[3]
        self.get_logger().info(f'Broadcasted transform odom_base with orientation: {q}')
        # Broadcast the static transform
        self.static_tf_broadcaster.sendTransform(static_t)
        self.get_logger().debug('Published static transform from base_link to laser_frame')

        t1 = TransformStamped()

        # Populate TransformStamped message
        t1.header.stamp = current_time.to_msg()
        t1.header.frame_id = 'laser'
        t1.child_frame_id = 'laser_frame'

        # Set the translation based on the origin offset
        t1.transform.translation.x = -0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.0
        
        q0=tf_transformations.quaternion_from_euler(0,0,0)
        # Set the rotation (no rotation, so quaternion is (0,0,0,1))
        t1.transform.rotation.x = q0[0]
        t1.transform.rotation.y = q0[1]
        t1.transform.rotation.z = q0[2]
        t1.transform.rotation.w = q0[3]
        self.get_logger().info(f'Broadcasted transform odom_base with orientation: {q0}')

        ct = TransformStamped()
        
        # Populate StaticTransformStamped message
        ct.header.stamp = current_time.to_msg()
        ct.header.frame_id = 'base_footprint'
        ct.child_frame_id = 'camera_link'
        
        # Set the translation based on the origin offset
        ct.transform.translation.x = 0.1
        ct.transform.translation.y = 0.0
        ct.transform.translation.z = 0.03
        
        q=tf_transformations.quaternion_from_euler(0,0,0)
        # Set the rotation (no rotation, so quaternion is (0,0,0,1))
        ct.transform.rotation.x = q[0]
        ct.transform.rotation.y = q[1]
        ct.transform.rotation.z = q[2]
        ct.transform.rotation.w = q[3]
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(ct)


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