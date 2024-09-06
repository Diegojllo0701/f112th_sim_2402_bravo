#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from f112th_sim_2402_bravo.msg import AngleDistance
<<<<<<< HEAD
from std_msgs.msg import Bool
=======
from std_msgs.msg import Bool  # Importar el tipo de mensaje Bool
>>>>>>> main

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
        
<<<<<<< HEAD
        # Publicación del estado de frenado
        self.publisher_brake = self.create_publisher(Bool, 'brake_state', 10)
        
=======
        # Publicador para el estado del freno
        self.publisher_break_state = self.create_publisher(Bool, 'break_state', 10)

>>>>>>> main
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
<<<<<<< HEAD
            brake_state = Bool()

            # Verificar si alguna de las distancias es menor al umbral
            if any(distance < self.ttc_threshold for distance in self.distances_front) and self.last_cmd_vel_par.linear.x > 0:
                # Activar el freno de emergencia
                cmd_vel.linear.x = float(0)
                cmd_vel.angular.z = float(0)
                brake_state.data = True
=======
            break_state = Bool()

            if self.distance_front < 1 and self.last_cmd_vel_par.linear.x > 0:
                # Aplicar freno de emergencia
                cmd_vel.linear.x = float(0)
                cmd_vel.angular.z = float(self.last_cmd_vel_par.angular.z)
                break_state.data = True
>>>>>>> main
            else:
                # Pasar los valores de cmd_vel_par a cmd_vel
                cmd_vel.linear.x = float(self.last_cmd_vel_par.linear.x)
                cmd_vel.angular.z = float(self.last_cmd_vel_par.angular.z)
<<<<<<< HEAD
                brake_state.data = False

            # Publicar los valores de cmd_vel y el estado del freno
            self.publisher_cmd_vel.publish(cmd_vel)
            self.publisher_brake.publish(brake_state)
=======
                break_state.data = False

            self.publisher_cmd_vel.publish(cmd_vel)
            self.publisher_break_state.publish(break_state)  # Publicar el estado del freno
>>>>>>> main

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrakeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()