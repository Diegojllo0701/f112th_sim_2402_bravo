#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Twist, Point, Vector3
from std_msgs.msg import Header
from rclpy.qos import QoSProfile
import numpy as np
import math
import heapq
from threading import Lock

from custom_msgs.msg import Obstacles

expansion_size = 1

def euler_from_quaternion(x, y, z, w):
    # Convertir cuaternión a ángulos de Euler (yaw)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(siny_cosp, cosy_cosp)
    return yaw_z

def costmap(data, width, height, resolution):
    # Expandir paredes en el costmap
    data = np.array(data).reshape(height, width)
    wall = np.where(data == 100)
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = np.clip(wall[0] + i, 0, height - 1)
            y = np.clip(wall[1] + j, 0, width - 1)
            data[x, y] = 100
    return data

def distance(a, b):
    # Calcular distancia Euclidiana entre dos puntos
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal):
    # Algoritmo de búsqueda A* para planificación de ruta
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: distance(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + distance(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] != 0:
                        continue
                else:
                    continue
            else:
                continue
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, float('inf')):
                continue
            if tentative_g_score < gscore.get(neighbor, float('inf')) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + distance(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Nodo de Navegación Iniciado')

        self.goal_x = []
        self.goal_y = []
        self.path_world = []  # Almacenar ruta en coordenadas del mundo

        # Suscripciones
        self.subscription_map = self.create_subscription(
            OccupancyGrid, '/map', self.OccGrid_callback, 10)
        self.subscription_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.Goal_Pose_callback, QoSProfile(depth=10))
        self.subscription_odom = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.subscription_obstacles = self.create_subscription(
            Obstacles, '/detected_obstacles', self.obstacles_callback, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Posición y orientación actuales del robot
        self.robot_pose_x = None
        self.robot_pose_y = None
        self.robot_yaw = None

        # Parámetros de control
        self.look_ahead_distance = 1.0  # Ajustar según sea necesario
        self.max_linear_speed = 0.1      # Ajustar según sea necesario
        self.max_angular_speed = 3.0     # Ajustar según sea necesario

        # Obstáculos dinámicos
        self.dynamic_obstacles = []
        self.obstacle_lock = Lock()

        # Temporizador para el bucle de control
        self.control_timer = None

    def OccGrid_callback(self, msg):
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_data = msg.data

    def odom_callback(self, msg):
        self.robot_pose_x = msg.pose.pose.position.x
        self.robot_pose_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        self.robot_yaw = euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)

    def Goal_Pose_callback(self, msg):
        # Limpiar objetivos anteriores si se inicia una nueva tarea de navegación
        user_input = input("¿Iniciar una nueva tarea de navegación? (y/n): ")
        if user_input.lower() == 'y':
            self.goal_x.clear()
            self.goal_y.clear()
            self.path_world.clear()
            if self.control_timer:
                self.control_timer.cancel()

        self.goal_x.append(msg.pose.position.x)
        self.goal_y.append(msg.pose.position.y)
        if input("¿Más puntos de referencia? (y/n): ") == 'n':
            self.get_map()

    def obstacles_callback(self, msg):
        with self.obstacle_lock:
            self.dynamic_obstacles = []
            for id, pos, vel in zip(msg.ids, msg.positions, msg.velocities):
                self.dynamic_obstacles.append({
                    'id': id,
                    'position': (pos.x, pos.y),
                    'velocity': (vel.x, vel.y)
                })
        self.get_logger().debug(f"Obstáculos dinámicos actualizados: {self.dynamic_obstacles}")

    def get_map(self):
        # Asegurarse de tener la posición actual del robot
        if self.robot_pose_x is None or self.robot_pose_y is None:
            self.get_logger().error("La posición actual del robot es desconocida.")
            return

        # Crear el costmap a partir de los datos del mapa
        data = costmap(self.map_data, self.width, self.height, self.resolution)

        # Integrar obstáculos dinámicos en el costmap
        with self.obstacle_lock:
            for obstacle in self.dynamic_obstacles:
                x, y = obstacle['position']
                grid_x = int((x - self.originX) / self.resolution)
                grid_y = int((y - self.originY) / self.resolution)
                # Asegurarse de que las coordenadas están dentro del mapa
                if 0 <= grid_x < self.height and 0 <= grid_y < self.width:
                    data[grid_x, grid_y] = 100  # Marcar como obstáculo

        # Convertir posiciones de inicio y fin a índices de la cuadrícula
        goal_column = int((self.goal_x[-1] - self.originX) / self.resolution)
        goal_row = int((self.goal_y[-1] - self.originY) / self.resolution)
        start_column = int((self.robot_pose_x - self.originX) / self.resolution)
        start_row = int((self.robot_pose_y - self.originY) / self.resolution)

        start = (start_row, start_column)
        goal = (goal_row, goal_column)

        # Verificar si el inicio y el objetivo están dentro de los límites
        if not (0 <= start_row < self.height and 0 <= start_column < self.width):
            self.get_logger().error("La posición de inicio está fuera de los límites.")
            return

        if not (0 <= goal_row < self.height and 0 <= goal_column < self.width):
            self.get_logger().error("La posición del objetivo está fuera de los límites.")
            return

        # Verificar si el inicio o el objetivo están en obstáculos
        if data[start_row][start_column] != 0:
            self.get_logger().error("La posición de inicio está en un obstáculo.")
            return

        if data[goal_row][goal_column] != 0:
            self.get_logger().error("La posición del objetivo está en un obstáculo.")
            return

        # Convertir los datos a un array de numpy para A*
        data_array = np.array(data)

        # Encontrar la ruta utilizando A*
        path = astar(data_array, start, goal)

        if not path:
            self.get_logger().error("No se encontró una ruta válida.")
            return

        # Convertir la ruta de índices de cuadrícula a coordenadas del mundo
        self.path_world = []
        for row, col in path:
            x = col * self.resolution + self.originX + self.resolution / 2.0
            y = row * self.resolution + self.originY + self.resolution / 2.0
            self.path_world.append((x, y))

        self.get_logger().info(f"Ruta planificada con {len(self.path_world)} puntos.")

        # Iniciar el bucle de control
        self.control_timer = self.create_timer(0.1, self.pure_pursuit_control)

    def pure_pursuit_control(self):
        if not self.path_world or self.robot_pose_x is None or self.robot_pose_y is None:
            return

        # Punto de mira actual es el primer punto en path_world inicialmente
        if not hasattr(self, 'current_index'):
            self.current_index = 0

        # Verificar si el robot ha alcanzado el punto de mira actual
        current_point = self.path_world[self.current_index]
        dx = abs(current_point[0] - self.robot_pose_x)
        dy = abs(current_point[1] - self.robot_pose_y)

        # Si ambas dx y dy son menores que el umbral, ir al siguiente punto
        if dx < 0.1 and dy < 0.1:  # Umbral de 0.1 metros
            self.current_index += 1  # Pasar al siguiente punto

            # Si se llegó al final de la ruta, detener el robot
            if self.current_index >= len(self.path_world):
                self.get_logger().info("¡Objetivo alcanzado!")
                self._stop_robot()
                return

            current_point = self.path_world[self.current_index]  # Actualizar al nuevo punto

        # Verificar si hay obstáculos dinámicos cerca del camino
        if self.is_obstacle_in_path():
            self.get_logger().warn("Obstáculo dinámico detectado en el camino. Re-planificando...")
            self.get_map()  # Re-planificar la ruta
            return

        # Calcular el ángulo de dirección al punto de mira actual
        dx = current_point[0] - self.robot_pose_x
        dy = current_point[1] - self.robot_pose_y
        angle_to_goal = math.atan2(dy, dx)

        # Calcular el error angular
        angle_error = angle_to_goal - self.robot_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalizar

        # Información de depuración
        self.get_logger().debug(f"Error Angular: {angle_error:.2f}")

        # Calcular comandos de control
        linear_speed = self.max_linear_speed
        angular_speed = self.max_angular_speed * angle_error

        # Limitar la velocidad angular
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        # Publicar el comando de velocidad
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.publisher.publish(twist)

    def is_obstacle_in_path(self):
        """
        Verifica si hay obstáculos dinámicos cerca de la ruta actual.
        Puedes ajustar el radio de detección según tus necesidades.
        """
        detection_radius = 0.5  # metros
        with self.obstacle_lock:
            for obstacle in self.dynamic_obstacles:
                ox, oy = obstacle['position']
                dist = distance((self.robot_pose_x, self.robot_pose_y), (ox, oy))
                if dist < detection_radius:
                    return True
        return False

    def _stop_robot(self):
        """Detiene el robot y cancela el temporizador de control."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        if self.control_timer:
            self.control_timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navegación interrumpida por el usuario.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
