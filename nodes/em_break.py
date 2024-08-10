#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from f112th_sim_2402_bravo.msg import AngleDistance
from std_msgs.msg import Bool
from rclpy.time import Time

class EmergencyBrakeNode(Node):
    def __init__(self):
        super().__init__('emergency_brake_node')
        
        self.distance_front = None
        self.previous_distance_front = None
        self.previous_time = None
        self.ttc_threshold = 1.0  # Define your TTC threshold (seconds)
        
        # Subscribe to angle_distances
        self.subscription_distance = self.create_subscription(
            AngleDistance,
            'angle_distances',
            self.distance_callback,
            10)
        
        # Subscribe to cmd_vel_par
        self.subscription_cmd_vel_par = self.create_subscription(
            Twist,
            'cmd_vel_par',
            self.cmd_vel_par_callback,
            10)
        
        # Publisher for cmd_vel
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.last_cmd_vel_par = Twist()

        # Publisher for brake state
        self.publisher_brake = self.create_publisher(Bool, 'brake_state', 10)
        
    def distance_callback(self, msg):
        current_time = self.get_clock().now()
        self.distance_front = msg.distance_front
        
        if self.previous_distance_front is not None and self.previous_time is not None:
            time_difference = (current_time - self.previous_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
            distance_difference = self.previous_distance_front - self.distance_front
            
            if time_difference > 0:
                self.relative_velocity = distance_difference / time_difference
            else:
                self.relative_velocity = 0
        else:
            self.relative_velocity = 0
        
        self.previous_distance_front = self.distance_front
        self.previous_time = current_time
        
        self.check_and_publish_cmd_vel()

    def cmd_vel_par_callback(self, msg):
        self.last_cmd_vel_par = msg
        self.check_and_publish_cmd_vel()

    def calculate_ttc(self):
        if self.distance_front is None or self.relative_velocity <= 0:
            return float('inf')  # Infinite TTC if there's no distance or the velocity is zero or negative
        
        return self.distance_front / self.relative_velocity

    def check_and_publish_cmd_vel(self):
        if self.distance_front is not None:
            cmd_vel = Twist()
            brake_state = Bool()
            
            ttc = self.calculate_ttc()
            
            if ttc < self.ttc_threshold and self.last_cmd_vel_par.linear.x > 0:
                # Apply emergency brake
                cmd_vel.linear.x = float(0)
                cmd_vel.angular.z = float(0)
                brake_state.data = True
            else:
                # Pass through cmd_vel_par values to cmd_vel
                cmd_vel.linear.x = float(self.last_cmd_vel_par.linear.x)
                cmd_vel.angular.z = float(self.last_cmd_vel_par.angular.z)
                brake_state.data = False
            
            self.publisher_cmd_vel.publish(cmd_vel)
            self.publisher_brake.publish(brake_state)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrakeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
