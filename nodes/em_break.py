#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from f112th_sim_2402_bravo.msg import AngleDistance

class EmergencyBrakeNode(Node):
    def __init__(self):
        super().__init__('em_break')
        
        self.distance_front = None
        
        # Suscripción a distancia frontal
        self.subscription_distance = self.create_subscription(
            AngleDistance,
            'angle_distances',
            self.distance_callback,
            10)
        
        # Suscripción a comandos de velocidad
        self.subscription_cmd_vel_par = self.create_subscription(
            Twist,
            'cmd_vel_par',
            self.cmd_vel_par_callback,
            10)
        
        # Publicador para cmd_vel
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.last_cmd_vel_par = Twist()

        # Publicador estado frenado
        #self.publisher_break = self.create_publisher(bool, 'break_state', 10)
        
        
    def distance_callback(self, msg):
        self.distance_front = msg.distance_front
        self.check_and_publish_cmd_vel()

    def cmd_vel_par_callback(self, msg):
        self.last_cmd_vel_par = msg
        self.check_and_publish_cmd_vel()

    def check_and_publish_cmd_vel(self):
        if self.distance_front is not None:
            cmd_vel = Twist()
            #break_state = bool
            
            if self.distance_front < 0.8 and self.last_cmd_vel_par.linear.x > 0:
                # Aplicar freno de emergencia
                cmd_vel.linear.x = float(0)
                cmd_vel.angular.z = float(0)
<<<<<<< HEAD
                #self.publisher_break.publish(True)
=======
                break_state = True
                
>>>>>>> b2ecb86 (break_state)
            else:
                # Pasar valores de cmd_vel_par a cmd_vel
                cmd_vel.linear.x = float(self.last_cmd_vel_par.linear.x)
                cmd_vel.angular.z = float(self.last_cmd_vel_par.angular.z)
<<<<<<< HEAD
                #self.publisher_break.publish(False)
=======
                break_state = False
                
>>>>>>> b2ecb86 (break_state)
            
            self.publisher_cmd_vel.publish(cmd_vel)
            #self.publisher_break.publish(break_state)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrakeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
