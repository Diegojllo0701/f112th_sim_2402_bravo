#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist
from custom_msgs.msg import Obstacles
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
import heapq
from threading import Lock
from rclpy.qos import QoSProfile
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from visualization_msgs.msg import Marker, MarkerArray  # Para RViz
from scipy.ndimage import binary_dilation  # Para la expansión de obstáculos
from geometry_msgs.msg import Pose
import time

def euler_from_quaternion(x, y, z, w):
    """
    Convierte un cuaternión a ángulos de Euler (yaw).
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(siny_cosp, cosy_cosp)
    return yaw_z

def distance(a, b):
    """
    Calcula la distancia Euclidiana entre dos puntos.
    """
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)

def astar(array, start, goal, occupied_threshold):
    """
    Algoritmo de búsqueda A* para planificación de ruta.
    """
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
                    cell_value = array[neighbor[0]][neighbor[1]]
                    if cell_value > occupied_threshold or cell_value == -1:
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

# Clase KalmanFilter para manejar el filtro de Kalman de cada obstáculo
class KalmanFilter:
    def __init__(self, initial_state, initial_covariance):
        self.x = initial_state  # Vector de estado [x, y, vx, vy]
        self.P = initial_covariance  # Matriz de covarianza
        # Matriz de transición de estado (se actualizará según delta_t)
        self.F = np.eye(4)
        # Matriz de observación
        self.H = np.eye(4)
        # Covarianza del ruido del proceso
        self.Q = np.eye(4) * 0.1
        # Covarianza del ruido de medición
        self.R = np.eye(4) * 0.5

    def predict(self, delta_t):
        # Actualizar la matriz F según delta_t
        self.F = np.array([
            [1, 0, delta_t, 0],
            [0, 1, 0, delta_t],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        # Predecir el siguiente estado
        self.x = self.F @ self.x
        # Predecir la siguiente covarianza
        self.P = self.F @ self.P @ self.F.T + self.Q

    def predict_state(self, delta_t):
        # Predecir el estado futuro sin modificar el estado actual
        F = np.array([
            [1, 0, delta_t, 0],
            [0, 1, 0, delta_t],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        future_x = F @ self.x
        return future_x

    def update(self, z):
        # Residual de la medición
        y = z - self.H @ self.x
        # Covarianza residual
        S = self.H @ self.P @ self.H.T + self.R
        # Ganancia de Kalman
        K = self.P @ self.H.T @ np.linalg.inv(S)
        # Actualizar la estimación del estado
        self.x = self.x + K @ y
        # Actualizar la estimación de la covarianza
        I = np.eye(self.x.shape[0])
        self.P = (I - K @ self.H) @ self.P

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Nodo de Navegación Iniciado')

        # Listas para objetivos
        self.goal_x = []
        self.goal_y = []
        self.path_world = []  # Almacenar ruta en coordenadas del mundo

        # Suscripciones
        self.subscription_map = self.create_subscription(
            OccupancyGrid, '/map', self.OccGrid_callback, 10)
        self.subscription_goal = self.create_subscription(
            PoseStamped, '/goal_pose', self.Goal_Pose_callback, QoSProfile(depth=10))
        self.subscription_odom = self.create_subscription(
            Pose, '/robot1/pose', self.odom_callback, 10)
        self.subscription_obstacles = self.create_subscription(
            Obstacles, '/detected_obstacles', self.obstacles_callback, 10)
        
        # Nueva suscripción para los puntos filtrados
        self.subscription_filtered_points = self.create_subscription(
            PointCloud2, '/filtered_points', self.filtered_points_callback, 10)

        # Publicadores
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/updated_map', 10)
        self.path_publisher = self.create_publisher(Path, '/planned_path', 10)
        self.marker_publisher = self.create_publisher(MarkerArray, '/dynamic_obstacles_markers', 10)  # Para RViz

        # Posición y orientación actuales del robot
        self.robot_pose_x = None
        self.robot_pose_y = None
        self.robot_yaw = None

        # Parámetros de control
        self.max_linear_speed = 1.0    # Ajustar según sea necesario
        self.max_angular_speed = 2.0     # Ajustar según sea necesario
        self.integral =0.0
        # Parámetros de predicción
        self.declare_parameter('prediction_time_horizon', 0.2)  # Tiempo en segundos para predecir posiciones futuras
        self.declare_parameter('prediction_time_step', 0.5)    # Intervalo de tiempo entre predicciones
        self.declare_parameter('dynamic_obstacle_expansion', 0.15)  # Tamaño de expansión basado en velocidad y robot
        self.declare_parameter('raw_point_ttl', 1.0)  # Tiempo en segundos para que los puntos crudos expiren

        # Parámetro para habilitar/deshabilitar la visualización
        self.declare_parameter('enable_visualization', True)
        self.enable_visualization = self.get_parameter('enable_visualization').get_parameter_value().bool_value

        # Umbral para considerar una celda como ocupada
        self.declare_parameter('occupied_threshold', 50)  # Puedes ajustar este valor según tus necesidades
        self.occupied_threshold = self.get_parameter('occupied_threshold').get_parameter_value().integer_value

        # Nuevo parámetro para el error angular máximo
        self.declare_parameter('max_angular_error', 1.0)  # Umbral máximo de error angular en radianes
        self.max_angular_error = self.get_parameter('max_angular_error').get_parameter_value().double_value

        # Parámetro para la distancia de anticipación en Pure Pursuit
        self.declare_parameter('look_ahead_distance', 0.5)  # Ajusta este valor según tus necesidades
        self.look_ahead_distance = self.get_parameter('look_ahead_distance').get_parameter_value().double_value

        # Obstáculos dinámicos
        self.dynamic_obstacles = []
        self.obstacle_lock = Lock()

        # Diccionarios para los filtros de Kalman y tiempos de último avistamiento
        self.kalman_filters = {}  # Diccionario para almacenar filtros de Kalman por obstáculo
        self.obstacle_last_seen = {}  # Rastrea el último tiempo visto de cada obstáculo

        # Puntos crudos filtrados con timestamps
        self.filtered_points = []  # Lista de tuples: (x, y, timestamp)
        self.point_lock = Lock()

        # Temporizador para limpiar puntos expirados
        self.cleanup_timer = self.create_timer(1.0, self.cleanup_filtered_points)  # Cada segundo

        # Temporizador para el bucle de control
        self.control_timer = None

        # Temporizador para la visualización
        if self.enable_visualization:
            self.visualization_timer = self.create_timer(0.5, self.visualization_callback)  # Actualiza cada 0.5 segundos
            self.initialize_visualization()

        # Inicializar el gridmap actualizado
        self.updated_map = None

        # Bandera para verificar si el mapa ha sido recibido
        self.map_received = False

        self.declare_parameter('kp', 0.5)  # Ganancia Proporcional
        self.declare_parameter('ki', 0.0)  # Ganancia Integral
        self.declare_parameter('kd', 0.05)  # Ganancia Derivativa

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        # Solicitar al usuario el número de puntos para la navegación
        self.num_points = int(input("Ingrese el número de puntos para la navegación: "))
        self.get_logger().info(f"Número de puntos a recibir: {self.num_points}")

        # Contador de puntos recibidos
        self.points_received = 0

        # Indicar al usuario que envíe los puntos desde RViz2
        self.get_logger().info("Por favor, envíe los puntos de destino utilizando el tópico /goal_pose desde RViz2.")

    def initialize_visualization(self):
        """
        Inicializa la configuración de Matplotlib.
        """
        plt.ion()  # Modo interactivo
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_title('Occupancy Grid y Planificación de Ruta')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.legend_added = False  # Control para agregar la leyenda solo una vez

    def OccGrid_callback(self, msg):
        """
        Callback para manejar la recepción del OccupancyGrid.
        """
        self.resolution = msg.info.resolution
        self.originX = msg.info.origin.position.x
        self.originY = msg.info.origin.position.y
        self.width = msg.info.width
        self.height = msg.info.height
        self.map_data = msg.data

        self.get_logger().info(f"OccupancyGrid recibido con {len(self.map_data)} datos.")

        if self.updated_map is None:
            self.initialize_updated_map()
            self.map_received = True
            self.get_logger().info("OccupancyGrid inicializado y actualizado.")

    def initialize_updated_map(self):
        """
        Inicializa el OccupancyGrid actualizado con los datos recibidos.
        """
        self.updated_map = OccupancyGrid()
        self.updated_map.header.frame_id = "map"
        self.updated_map.info.resolution = self.resolution
        self.updated_map.info.width = self.width
        self.updated_map.info.height = self.height
        self.updated_map.info.origin.position.x = self.originX
        self.updated_map.info.origin.position.y = self.originY
        self.updated_map.info.origin.position.z = 0.0
        self.updated_map.info.origin.orientation.w = 1.0
        self.updated_map.data = list(self.map_data)  # Convertir array.array a lista

        self.get_logger().info(f"OccupancyGrid inicializado con {len(self.updated_map.data)} datos.")

    def odom_callback(self, msg):
        """
        Callback para manejar la recepción de la odometría.
        """
        self.robot_pose_x = msg.position.x
        self.robot_pose_y = msg.position.y
        orientation_q = msg.orientation
        self.robot_yaw = euler_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        
        self.get_logger().debug(f"Odometría actualizada: x={self.robot_pose_x}, y={self.robot_pose_y}, yaw={self.robot_yaw}")

    def Goal_Pose_callback(self, msg):
        """
        Callback para manejar la recepción de objetivos de navegación desde /goal_pose.
        """
        if self.points_received < self.num_points:
            # Añadir objetivos a las listas
            self.goal_x.append(msg.pose.position.x)
            self.goal_y.append(msg.pose.position.y)
            self.points_received += 1
            self.get_logger().info(f"Punto {self.points_received} recibido: ({msg.pose.position.x}, {msg.pose.position.y})")
            if self.points_received == self.num_points:
                self.get_logger().info("Se han recibido todos los puntos. Iniciando planificación de ruta.")
                self.get_map()
        else:
            self.get_logger().warn("Se han recibido más puntos de los especificados. Ignorando punto adicional.")

    def obstacles_callback(self, msg):
        """
        Callback para manejar la recepción de obstáculos detectados.
        """
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                       self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        with self.obstacle_lock:
            for id, pos, vel in zip(msg.ids, msg.positions, msg.velocities):
                obstacle_id = id
                position = np.array([pos.x, pos.y])
                velocity = np.array([vel.x, vel.y])
                z = np.hstack((position, velocity))
                if obstacle_id not in self.kalman_filters:
                    # Inicializar filtro de Kalman para nuevo obstáculo
                    initial_state = z
                    initial_covariance = np.eye(4)
                    self.kalman_filters[obstacle_id] = KalmanFilter(initial_state, initial_covariance)
                    self.get_logger().info(f"Filtro de Kalman inicializado para el obstáculo {obstacle_id}")
                else:
                    # Calcular delta_t desde la última actualización
                    delta_t = current_time - self.obstacle_last_seen[obstacle_id]
                    # Paso de predicción
                    self.kalman_filters[obstacle_id].predict(delta_t)
                    # Paso de actualización
                    self.kalman_filters[obstacle_id].update(z)
                # Actualizar el tiempo de último avistamiento
                self.obstacle_last_seen[obstacle_id] = current_time
        self.get_logger().debug("Obstáculos actualizados con filtros de Kalman.")

        # Eliminar obstáculos no vistos recientemente
        self.cleanup_obstacles(current_time)

        # Actualizar el OccupancyGrid con las predicciones de obstáculos dinámicos
        if self.map_received:
            self.update_dynamic_obstacles_in_map()

    def cleanup_obstacles(self, current_time):
        """
        Elimina obstáculos que no han sido vistos durante un tiempo determinado.
        """
        obstacle_timeout = 5.0  # Tiempo en segundos para esperar antes de eliminar un obstáculo
        to_remove = []
        for obstacle_id, last_seen in self.obstacle_last_seen.items():
            if current_time - last_seen > obstacle_timeout:
                to_remove.append(obstacle_id)
        for obstacle_id in to_remove:
            del self.kalman_filters[obstacle_id]
            del self.obstacle_last_seen[obstacle_id]
            self.get_logger().info(f"Obstáculo {obstacle_id} eliminado por tiempo de espera.")

    def update_dynamic_obstacles_in_map(self):
        """
        Actualiza el OccupancyGrid con obstáculos dinámicos utilizando predicciones del filtro de Kalman.
        Incluye expansión de obstáculos y manejo de puntos con tiempo.
        """
        if self.updated_map is None:
            self.get_logger().warn("OccupancyGrid no está inicializado.")
            return

        # Clonar el mapa base para mantener la parte estática
        dynamic_data = np.array(self.map_data).reshape((self.height, self.width))

        # Integrar puntos crudos filtrados
        with self.point_lock:
            for (x, y, _) in self.filtered_points:
                grid_index = self.world_to_grid_index(x, y)
                if grid_index != -1:
                    grid_row = grid_index // self.width
                    grid_col = grid_index % self.width
                    dynamic_data[grid_row, grid_col] = 100  # Marcar como ocupado
                    self.get_logger().debug(f"Punto filtrado agregado al grid: ({grid_col}, {grid_row})")
                else:
                    self.get_logger().debug(f"Punto filtrado fuera del grid: ({x}, {y})")

        # Integrar obstáculos dinámicos utilizando los filtros de Kalman
        with self.obstacle_lock:
            for obstacle_id, kalman_filter in self.kalman_filters.items():
                # Predecir posiciones futuras
                prediction_time_horizon = self.get_parameter('prediction_time_horizon').get_parameter_value().double_value
                prediction_time_step = self.get_parameter('prediction_time_step').get_parameter_value().double_value
                dynamic_obstacle_expansion = self.get_parameter('dynamic_obstacle_expansion').get_parameter_value().integer_value

                num_steps = int(prediction_time_horizon / prediction_time_step)
                for step in range(1, num_steps + 1):
                    delta_t = prediction_time_step * step
                    # Predecir estado futuro
                    future_state = kalman_filter.predict_state(delta_t)
                    future_x = future_state[0]
                    future_y = future_state[1]
                    future_grid_x = int((future_x - self.originX) / self.resolution)
                    future_grid_y = int((future_y - self.originY) / self.resolution)
                    if 0 <= future_grid_x < self.width and 0 <= future_grid_y < self.height:
                        # Expansión basada en el tamaño del robot
                        for dx in range(-dynamic_obstacle_expansion, dynamic_obstacle_expansion + 1):
                            for dy in range(-dynamic_obstacle_expansion, dynamic_obstacle_expansion + 1):
                                neighbor_x = future_grid_x + dx
                                neighbor_y = future_grid_y + dy
                                if 0 <= neighbor_x < self.width and 0 <= neighbor_y < self.height:
                                    dynamic_data[neighbor_y, neighbor_x] = 100  # Marcar como obstáculo
                                    self.get_logger().debug(f"Obstáculo dinámico futuro agregado al grid: ({neighbor_x}, {neighbor_y})")
                    else:
                        self.get_logger().debug(f"Posición predicha del obstáculo fuera del mapa: ({future_x}, {future_y})")

        # Expandir los obstáculos para proporcionar una zona de seguridad
        robot_radius = 0.13  # Radio del robot en metros (ajusta según el tamaño de tu robot)
        expansion_radius = int(math.ceil(robot_radius / self.resolution))
        dynamic_data_binary = (dynamic_data >= self.occupied_threshold).astype(np.int32)
        expanded_data = binary_dilation(dynamic_data_binary, structure=np.ones((3,3)), iterations=expansion_radius)
        dynamic_data = np.where(expanded_data, 100, dynamic_data)

        # Actualizar el OccupancyGrid actualizado
        self.updated_map.data = dynamic_data.flatten().tolist()

        # Publicar el OccupancyGrid actualizado
        self.publish_updated_map()

        # Publicar los markers para RViz
        self.publish_dynamic_obstacles_markers()

    def filtered_points_callback(self, msg):
        """
        Callback para manejar la recepción de puntos filtrados.
        """
        if not self.map_received:
            self.get_logger().warn("OccupancyGrid no recibido aún. Ignorando puntos filtrados.")
            return

        current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                       self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        # Convertir PointCloud2 a lista de coordenadas XY
        points = self.pointcloud2_to_xy_array(msg)

        if not points:
            self.get_logger().warn("No se recibieron puntos filtrados en /filtered_points.")
            return

        with self.point_lock:
            for point in points:
                x, y = point
                self.filtered_points.append((x, y, current_time))
                self.get_logger().debug(f"Punto filtrado almacenado: ({x}, {y})")

        self.get_logger().debug(f"Puntos filtrados recibidos y almacenados: {len(points)}")

        # Publicar el OccupancyGrid actualizado con los nuevos puntos
        self.update_dynamic_obstacles_in_map()

    def pointcloud2_to_xy_array(self, cloud_msg):
        """
        Convierte un mensaje PointCloud2 a una lista de coordenadas XY.
        """
        points = []
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            points.append((x, y))
        return points

    def world_to_grid_index(self, x, y):
        """
        Convierte coordenadas del mundo a un índice en el gridmap.
        """
        grid_x = int((x - self.originX) / self.resolution)
        grid_y = int((y - self.originY) / self.resolution)

        if grid_x < 0 or grid_x >= self.width or grid_y < 0 or grid_y >= self.height:
            return -1  # Fuera del gridmap

        return grid_y * self.width + grid_x

    def cleanup_filtered_points(self):
        """
        Limpia puntos filtrados que han expirado según raw_point_ttl.
        """
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + \
                       self.get_clock().now().seconds_nanoseconds()[1] * 1e-9
        ttl = self.get_parameter('raw_point_ttl').get_parameter_value().double_value

        with self.point_lock:
            initial_count = len(self.filtered_points)
            self.filtered_points = [
                (x, y, t) for (x, y, t) in self.filtered_points if (current_time - t) <= ttl
            ]
            removed_count = initial_count - len(self.filtered_points)
        
        if removed_count > 0:
            self.get_logger().debug(f"Puntos filtrados eliminados por TTL: {removed_count}")
            self.get_logger().debug(f"Puntos filtrados restantes: {len(self.filtered_points)}")

        if self.map_received:
            self.update_dynamic_obstacles_in_map()

    def get_map(self):
        """
        Planifica la ruta basada en el OccupancyGrid actualizado utilizando A* entre cada par de puntos.
        """
        # Esperar hasta que se haya recibido la posición del robot
        if self.robot_pose_x is None or self.robot_pose_y is None:
            self.get_logger().warn("Posición del robot desconocida. Esperando odometría.")
            return

        if self.updated_map is None:
            self.get_logger().error("El OccupancyGrid actualizado no está disponible.")
            return

        # Convertir el OccupancyGrid actualizado a un array de numpy para A*
        data_array = np.array(self.updated_map.data).reshape((self.height, self.width))

        # Verificar que el OccupancyGrid tiene el tamaño correcto
        expected_size = self.height * self.width
        actual_size = len(data_array.flatten())
        if actual_size != expected_size:
            self.get_logger().error(f"OccupancyGrid tiene tamaño incorrecto: esperado {expected_size}, recibido {actual_size}.")
            return

        # Inicializar la ruta total
        self.path_world = []

        # Posición inicial del robot
        current_x = self.robot_pose_x
        current_y = self.robot_pose_y

        # Planificar ruta entre cada par de puntos
        for idx in range(len(self.goal_x)):
            # Convertir posiciones de inicio y fin a índices de la cuadrícula
            start_column = int((current_x - self.originX) / self.resolution)
            start_row = int((current_y - self.originY) / self.resolution)
            goal_column = int((self.goal_x[idx] - self.originX) / self.resolution)
            goal_row = int((self.goal_y[idx] - self.originY) / self.resolution)

            start = (start_row, start_column)
            goal = (goal_row, goal_column)

            # Verificar si el inicio y el objetivo están dentro de los límites
            if not (0 <= start_row < self.height and 0 <= start_column < self.width):
                self.get_logger().error(f"La posición de inicio está fuera de los límites para el segmento {idx+1}.")
                return

            if not (0 <= goal_row < self.height and 0 <= goal_column < self.width):
                self.get_logger().error(f"La posición del objetivo está fuera de los límites para el punto {idx+1}.")
                return

            # # Verificar si el inicio o el objetivo están en obstáculos
            # if data_array[start_row][start_column] > self.occupied_threshold or data_array[start_row][start_column] == -1:
            #     self.get_logger().error(f"La posición de inicio está en un obstáculo para el segmento {idx+1}.")
            #     return

            # if data_array[goal_row][goal_column] > self.occupied_threshold or data_array[goal_row][goal_column] == -1:
            #     self.get_logger().error(f"La posición del objetivo está en un obstáculo para el punto {idx+1}.")
            #     return

            # Encontrar la ruta utilizando A*
            path = astar(data_array, start, goal, self.occupied_threshold)

            if not path:
                self.get_logger().error(f"No se encontró una ruta válida para el segmento {idx+1}.")
                return

            # Convertir la ruta de índices de cuadrícula a coordenadas del mundo
            segment_path_world = []
            for row, col in path:
                x = col * self.resolution + self.originX + self.resolution / 2.0
                y = row * self.resolution + self.originY + self.resolution / 2.0
                segment_path_world.append((x, y))

            self.get_logger().info(f"Ruta planificada para el segmento {idx+1} con {len(segment_path_world)} puntos.")

            # Evitar duplicar el punto inicial en segmentos consecutivos
            if idx > 0 and segment_path_world:
                segment_path_world = segment_path_world[1:]

            # Añadir el segmento a la ruta total
            self.path_world.extend(segment_path_world)

            # Actualizar la posición actual para el próximo segmento
            current_x = self.goal_x[idx]
            current_y = self.goal_y[idx]

        self.get_logger().info(f"Ruta total planificada con {len(self.path_world)} puntos.")

        # Publicar la ruta planificada para RViz
        self.publish_planned_path()
        # Iniciar el bucle de control
        if self.control_timer:
            self.control_timer.cancel()
        self.get_logger().info("Iniciando el control de Pure Pursuit.")
        self.control_timer = self.create_timer(0.1, self.pure_pursuit_control)

    def publish_planned_path(self):
        """
        Publica la ruta planificada en el tópico /planned_path.
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in self.path_world:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        self.path_publisher.publish(path_msg)

    def pure_pursuit_control(self):
        """
        Control de seguimiento de ruta utilizando un controlador PID.
        """
        self.get_logger().info("Ejecutando el control de Pure Pursuit.")
        if not self.path_world or self.robot_pose_x is None or self.robot_pose_y is None:
            self.get_logger().warn("Ruta o posición del robot no disponible. Deteniendo el robot.")
            return

        # Punto de mira actual es el primer punto en path_world inicialmente
        if not hasattr(self, 'current_index'):
            self.current_index = 0

        # Verificar si el robot ha alcanzado el punto de mira actual
        current_point = self.path_world[self.current_index]
        dx = current_point[0]-0.1 - self.robot_pose_x
        dy = current_point[1] - self.robot_pose_y
        distance_to_point = math.sqrt(dx**2 + dy**2)
        self.get_logger().info(f"Distancia al punto de mira actual: {distance_to_point}")
        # Si la distancia es menor que el umbral, pasar al siguiente punto
        if distance_to_point < 0.3:  # Umbral de 0.2 metros
            self.current_index += 9 # Pasar al siguiente punto

            # Si se llegó al final de la ruta, detener el robot
            if self.current_index >= len(self.path_world):
                self.get_logger().info("¡Objetivo alcanzado!")
                self._stop_robot()
                return

            current_point = self.path_world[self.current_index]  # Actualizar al nuevo punto
        self.get_logger().info(f"Punto de mira actual: ({current_point[0]}, {current_point[1]})")
        #Verificar si hay obstáculos en el camino actual
        if self.is_path_blocked():
            self.get_logger().warn("Obstáculo detectado en la ruta planificada. Re-planificando...")
            self.get_map()  # Re-planificar la ruta
            return

        # Calcular el ángulo de dirección al punto de mira actual
        dx = current_point[0] - self.robot_pose_x
        dy = current_point[1] - self.robot_pose_y
        angle_to_goal = math.atan2(dy, dx)
        self.get_logger().info(f"Ángulo al punto de mira: {angle_to_goal}")
        # Calcular el error angular
        angle_error = angle_to_goal - self.robot_yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))  # Normalizar
        self.get_logger().info(f"Error angular: {angle_error}")
        # Actualización del PID
        #self.integral += angle_error * 0.1  # Asumiendo un tiempo de muestreo de 0.1s
        #derivative = (angle_error - self.previous_error) / 0.1
        #self.previous_error = angle_errorf
        #self.get_logger().info(f"Error integral: {self.integral}, derivativo: {derivative}")
        # Calcular comandos de control
        max_angle_error = math.pi / 2  # Ángulo máximo considerado (90 grados)
        angle_error_normalized = abs(angle_error) / max_angle_error
        linear_speed = self.max_linear_speed * (1 - angle_error_normalized)
        linear_speed = 0.52
        angular_speed = (self.kp * angle_error) #+ (self.ki * self.integral) + (self.kd * derivative)
        self.get_logger().info(f"Velocidad angular: {angular_speed}")
        # Limitar la velocidad angular
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))

        # Publicar el comando de velocidad
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.publisher.publish(twist)
        self.get_logger().debug(f"Comando de velocidad publicado: linear.x={linear_speed}, angular.z={angular_speed}")

    def is_path_blocked(self):
        """
        Verifica si hay obstáculos en la ruta planificada desde la posición actual hasta el punto de mira.
        """
        # Definir una tolerancia de desviación
        N = 30  # Número de puntos por delante para verificar

        for point in self.path_world[self.current_index:self.current_index + N]:
            x, y = point
            grid_index = self.world_to_grid_index(x, y)
            if grid_index != -1:
                occupancy = self.updated_map.data[grid_index]
                if occupancy > self.occupied_threshold or occupancy == -1:
                    self.get_logger().debug(f"Obstáculo detectado en la ruta: ({x}, {y})")
                    return True
            else:
                # Punto fuera del mapa, considerar como obstáculo
                self.get_logger().debug(f"Punto de ruta fuera del mapa: ({x}, {y})")
                return True
        return False

    def _stop_robot(self):
        """
        Detiene el robot y cancela el temporizador de control.
        """
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        if self.control_timer:
            self.control_timer.cancel()
        self.get_logger().info("Robot detenido.")

    def publish_updated_map(self):
        """
        Publica el OccupancyGrid actualizado.
        """
        if self.updated_map is None:
            self.initialize_updated_map()

        self.updated_map.header.stamp = self.get_clock().now().to_msg()
        self.map_publisher.publish(self.updated_map)
        self.get_logger().debug("OccupancyGrid actualizado publicado en /updated_map.")

    def publish_dynamic_obstacles_markers(self):
        """
        Publica obstáculos dinámicos como markers para RViz.
        """
        marker_array = MarkerArray()
        with self.obstacle_lock:
            for idx, (obstacle_id, kalman_filter) in enumerate(self.kalman_filters.items()):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "dynamic_obstacles"
                marker.id = idx
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = kalman_filter.x[0]
                marker.pose.position.y = kalman_filter.x[1]
                marker.pose.position.z = 0.1  # Altura del cilindro
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.4  # Radio
                marker.scale.y = 0.4
                marker.scale.z = 0.2  # Altura
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)
        self.get_logger().debug("Markers de obstáculos dinámicos publicados en /dynamic_obstacles_markers.")

    def visualization_callback(self):
        """
        Callback para actualizar la visualización de Matplotlib.
        """
        if self.updated_map is not None:
            expected_size = self.height * self.width
            actual_size = len(self.updated_map.data)
            
            if actual_size != expected_size:
                self.get_logger().warn(f"Tamaño de OccupancyGrid incorrecto: esperado {expected_size}, recibido {actual_size}.")
                return

            # Convertir el OccupancyGrid a una imagen
            map_array = np.array(self.updated_map.data).reshape((self.height, self.width))

            occupied_threshold = self.occupied_threshold  # Usar el parámetro definido

            # Inicializar el mapa de imagen con celdas desconocidas en gris
            map_image = np.full_like(map_array, 127)  # Celdas desconocidas en gris

            # Marcar celdas libres como blancas y ocupadas como negras
            map_image[map_array >= 0] = 255  # Inicialmente marcar todas las celdas conocidas como libres
            map_image[map_array > occupied_threshold] = 0   # Marcar celdas ocupadas como negras

            # Tratar celdas desconocidas como ocupadas (opcional)
            map_image[map_array == -1] = 0  # Tratar celdas desconocidas como ocupadas

            # Mostrar el mapa
            self.ax.clear()
            self.ax.imshow(map_image, cmap='gray_r', origin='lower',
                           extent=[self.originX, self.originX + self.width * self.resolution,
                                   self.originY, self.originY + self.height * self.resolution])

            # Dibujar obstáculos dinámicos con transparencia
            with self.obstacle_lock:
                for obstacle_id, kalman_filter in self.kalman_filters.items():
                    ox, oy = kalman_filter.x[0], kalman_filter.x[1]
                    circle = patches.Circle((ox, oy), 0.2, linewidth=1, edgecolor='r', facecolor='r', alpha=0.5)
                    self.ax.add_patch(circle)

            # Dibujar la ruta planificada
            if self.path_world:
                path_x, path_y = zip(*self.path_world)
                self.ax.plot(path_x, path_y, 'r-', linewidth=2, label='Ruta Planificada')
                self.get_logger().debug("Ruta planificada visualizada en Matplotlib.")

            # Dibujar la posición actual del robot
            if self.robot_pose_x is not None and self.robot_pose_y is not None:
                self.ax.plot(self.robot_pose_x, self.robot_pose_y, 'bo', markersize=5, label='Robot')
                self.get_logger().debug(f"Posición del robot visualizada en Matplotlib: ({self.robot_pose_x}, {self.robot_pose_y})")

            # Dibujar el punto de mira actual
            if hasattr(self, 'current_index') and self.current_index < len(self.path_world):
                current_point = self.path_world[self.current_index]
                self.ax.plot(current_point[0], current_point[1], 'go', markersize=10, label='Punto de Mira')
                self.get_logger().debug(f"Punto de mira visualizado en Matplotlib: {current_point}")

            # Agregar leyenda una vez
            if not self.legend_added:
                # Para evitar múltiples entradas en la leyenda, crea handles manualmente
                handles, labels = self.ax.get_legend_handles_labels()
                unique = dict(zip(labels, handles))
                self.ax.legend(unique.values(), unique.keys(), loc='upper right')
                self.legend_added = True

            # Ajustar los límites del gráfico si es necesario
            self.ax.set_xlim(self.originX, self.originX + self.width * self.resolution)
            self.ax.set_ylim(self.originY, self.originY + self.height * self.resolution)

            plt.draw()
            plt.pause(0.001)  # Pausa breve para actualizar el gráfico

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
        # Cerrar la visualización de Matplotlib
        if node.enable_visualization:
            plt.ioff()
            plt.show()

if __name__ == '__main__':
    main()
