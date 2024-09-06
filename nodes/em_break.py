#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from f112th_sim_2402_bravo.msg import AngleDistance
from std_msgs.msg import Bool  # Importar el tipo de mensaje Bool

class EmergencyBrakeNode(Node):
    def __init__(self):
        super().__init__('emergency_brake_node')
        
        self.distances_front = [None, None, None]  # Para almacenar las distancias -10°, 0°, 10°
        self.ttc_threshold = 0.5  # Define tu umbral de TTC (segundos)
        
        # Suscripción al topic angle_distances
        self.subscription_distance = self.create_subscription(
            AngleDistance,
            'angle_distances',
            self.distance_callback,
            10)
        
        # Suscripción al topic cmd_vel_par
        self.subscription_cmd_vel_par = self.create_subscription(
            Twist,
            'cmd_vel_par',
            self.cmd_vel_par_callback,
            10)
        
        # Publicación en el topic cmd_vel
        self.publisher_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Publicador para el estado del freno
        self.publisher_break_state = self.create_publisher(Bool, 'break_state', 10)

        self.last_cmd_vel_par = Twist()


    def distance_callback(self, msg):
        # Guardar las distancias de -10°, 0°, 10° (posiciones 0, 1, 2)
        self.distances_front = [msg.distances_front[0], msg.distances_front[1], msg.distances_front[2]]
        self.check_and_publish_cmd_vel()

    def cmd_vel_par_callback(self, msg):
        self.last_cmd_vel_par = msg
        self.check_and_publish_cmd_vel()

    def check_and_publish_cmd_vel(self):
        if None not in self.distances_front:
            cmd_vel = Twist()
            break_state = Bool()
            self.get_logger().info(f'Distances: {self.distances_front}')
            if self.distances_front[1] < 1 and self.last_cmd_vel_par.linear.x > 0:
                # Aplicar freno de emergencia
                cmd_vel.linear.x = float(0)
                cmd_vel.angular.z = float(self.last_cmd_vel_par.angular.z)
                self.get_logger().warn('Emergency brake applied!')
                break_state.data = True
            else:
                # Pasar los valores de cmd_vel_par a cmd_vel
                cmd_vel.linear.x = float(self.last_cmd_vel_par.linear.x)
                cmd_vel.angular.z = float(self.last_cmd_vel_par.angular.z)
                self.get_logger().info('Emergency brake not applied.')
                break_state.data = False

            self.publisher_cmd_vel.publish(cmd_vel)
            self.publisher_break_state.publish(break_state)  # Publicar el estado del freno

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrakeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()