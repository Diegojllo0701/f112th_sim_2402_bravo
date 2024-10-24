#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from rclpy.duration import Duration

class OdomTfPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        
        # Subscribe to the /robot1/pose topic
        self.subscription = self.create_subscription(
            Pose,
            '/robot1/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create a TransformBroadcaster for dynamic transforms
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Create a StaticTransformBroadcaster for laser_frame
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.sampling_time=0.1
        self.timer = self.create_timer(self.sampling_time, self.timer_callback)
        
        # Publish the static transform once
        self.publish_static_transform()
        
    def publish_static_transform(self):
        static_t = TransformStamped()
        
        # Set the static transform's header
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'base_link'
        static_t.child_frame_id = 'laser_frame'
        
        # Set the translation based on the origin offset
        static_t.transform.translation.x = -0.04
        static_t.transform.translation.y = 0.0
        static_t.transform.translation.z = 0.1
        
        # Set the rotation (no rotation, so quaternion is (0,0,0,1))
        static_t.transform.rotation.x = 0.0
        static_t.transform.rotation.y = 0.0
        static_t.transform.rotation.z = 0.0
        static_t.transform.rotation.w = 1.0
        
        # Broadcast the static transform
        self.static_tf_broadcaster.sendTransform(static_t)
        self.get_logger().info('Published static transform from base_link to laser_frame')
    
    def timer_callback(self):
        # Create a TransformStamped message for odom to base_link
        
        t = TransformStamped()

        # Set the time stamp to the current time
        t.header.stamp = self.get_clock().now().to_msg()
        # Set the parent frame to 'odom'
        t.header.frame_id = 'odom'
        # Set the child frame to 'base_link'
        t.child_frame_id = 'base_link'
        msg=self.latest_pose
        # Copy the position data from the PoseStamped message
        t.transform.translation.x = msg.position.x/1000
        t.transform.translation.y = msg.position.y/1000
        t.transform.translation.z = msg.position.z/1000

        # Copy the orientation data from the PoseStamped message
        t.transform.rotation = msg.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'Broadcasted transform from {t.header.frame_id} to {t.child_frame_id}')

    def pose_callback(self, msg):
        self.latest_pose = msg
        self.get_logger().debug('Received new pose message.')

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
