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
        self.distances_front = list(msg.distances_front)
        self.check_and_publish_cmd_vel()

    def cmd_vel_par_callback(self, msg):
        self.last_cmd_vel_par = msg
        self.check_and_publish_cmd_vel()

    def check_and_publish_cmd_vel(self):
        if None not in self.distances_front:
            cmd_vel = Twist()
            break_state = Bool()
            #self.get_logger().info(f'Distances: {self.distances_front}')
            
            # Seleccionar distancias basadas en el ángulo de giro
            if self.last_cmd_vel_par.angular.z < -0.2:  # Giro hacia la derecha
                relevant_distances = self.distances_front[1:3]  # Priorizar los ángulos positivos
            elif self.last_cmd_vel_par.angular.z > 0.2:  # Giro hacia la izquierda
                relevant_distances = self.distances_front[7:9]  # Priorizar los ángulos negativos
            else:  # Recto
                relevant_distances = self.distances_front[2:8]  # Priorizar ángulos frontales (-20° a 20°)

            # Tomar la distancia más cercana en la dirección de movimiento
            min_distance = min(relevant_distances)
            self.get_logger().info(f'Min distance: {min_distance}')
            if min_distance < 0.5 and self.last_cmd_vel_par.linear.x > 0:
                reverse_speed = -0.2  # Ajusta este valor según la capacidad de frenado del coche
                cmd_vel.linear.x = reverse_speed
                cmd_vel.angular.z = float(self.last_cmd_vel_par.angular.z)
                #self.get_logger().warn(f'Emergency brake applied with reverse speed {reverse_speed}!')
                break_state.data = True
            else:
                # Pasar los valores de cmd_vel_par a cmd_vel
                cmd_vel.linear.x = float(self.last_cmd_vel_par.linear.x)
                cmd_vel.angular.z = float(self.last_cmd_vel_par.angular.z)
                #self.get_logger().info('Emergency brake not applied.')
                break_state.data = False

            self.publisher_cmd_vel.publish(cmd_vel)
            self.publisher_break_state.publish(break_state)

def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrakeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()