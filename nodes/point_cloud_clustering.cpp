// point_cloud_clustering.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>  // For pcl::transformPointCloud

#include "custom_msgs/msg/obstacles.hpp"

#include <unordered_map>
#include <vector>
#include <cmath>
#include <deque>
#include <unordered_set>
#include <utility>      // Para std::pair
#include <functional>   // Para std::hash

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Include Eigen for transformation matrices
#include <Eigen/Dense>

// Definir la estructura 'pair_hash' en el ámbito global
struct pair_hash {
    std::size_t operator () (const std::pair<int, int>& p) const {
        // Combinar los hashes de los dos enteros
        return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
    }
};

class TrackedObstacle
{
public:
    int id;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Vector3 velocity;
    rclcpp::Time last_seen;

    // History buffer to store past positions and their timestamps
    std::deque<std::pair<geometry_msgs::msg::Point, rclcpp::Time>> history_;
    size_t history_size_ = 5;  // Maximum number of history entries

    // Default constructor (required for unordered_map)
    TrackedObstacle() : id(-1)
    {
        velocity.x = 0.0;
        velocity.y = 0.0;
        velocity.z = 0.0;
    }

    TrackedObstacle(int obstacle_id, const geometry_msgs::msg::Point& pos, const rclcpp::Time& time)
        : id(obstacle_id), position(pos), last_seen(time)
    {
        velocity.x = 0.0;
        velocity.y = 0.0;
        velocity.z = 0.0;
        history_.emplace_back(pos, time);
    }

    // Method to update position and history
    void updatePosition(const geometry_msgs::msg::Point& new_pos, const rclcpp::Time& new_time)
    {
        // Add new position to history
        history_.emplace_back(new_pos, new_time);

        // Maintain history size
        if (history_.size() > history_size_)
        {
            history_.pop_front();
        }

        // Calculate velocity based on history
        calculateVelocity();
        
        // Update current position and timestamp
        position = new_pos;
        last_seen = new_time;
    }

private:
    void calculateVelocity()
    {
        if (history_.size() < 2)
        {
            velocity.x = 0.0;
            velocity.y = 0.0;
            velocity.z = 0.0;
            return;
        }

        // Calculate velocity based on the oldest and newest positions
        const auto& oldest = history_.front();
        const auto& newest = history_.back();

        double dt = (newest.second - oldest.second).seconds();
        if (dt <= 0.0)
        {
            velocity.x = 0.0;
            velocity.y = 0.0;
            velocity.z = 0.0;
            return;
        }

        velocity.x = (newest.first.x - oldest.first.x) / dt;
        velocity.y = (newest.first.y - oldest.first.y) / dt;
        velocity.z = (newest.first.z - oldest.first.z) / dt;
    }
};

class PointCloudClusteringNode : public rclcpp::Node
{
public:
    PointCloudClusteringNode()
        : Node("point_cloud_clustering_node"),
          next_id_(0),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Set use_sim_time parameter to true without declaring it
        this->set_parameter(rclcpp::Parameter("use_sim_time", true));

        // Subscriber to point cloud data
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points",  // Ajusta este tópico según tu configuración
            10,
            std::bind(&PointCloudClusteringNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for visualization markers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_clusters", 10);

        // Publisher for obstacle positions and velocities
        obstacle_publisher_ = this->create_publisher<custom_msgs::msg::Obstacles>("detected_obstacles", 10);

        // Publisher for filtered points
        filtered_pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

        RCLCPP_INFO(this->get_logger(), "Point Cloud Clustering Node ha sido iniciado.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Frame ID del point cloud: %s", msg->header.frame_id.c_str());

        // Convertir el mensaje ROS a un point cloud de PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Verificar si el point cloud está vacío
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Se recibió un point cloud vacío.");
            return;
        }

        // Transformar el point cloud al frame 'odom'
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = transformPointCloudToOdom(pcl_cloud, msg->header.stamp);
        if (transformed_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "El point cloud transformado está vacío.");
            return;
        }

        // Aplicar filtro PassThrough en el eje z para eliminar puntos del piso
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(transformed_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.25, 2.0);  // Eliminar puntos por debajo de 0.25m y por encima de 1.5m
        pass.filter(*filtered_cloud);

        if (filtered_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Point cloud filtrado está vacío después del PassThrough.");
            return;
        }

        // Eliminar puntos con coordenadas (x, y) muy similares
        pcl::PointCloud<pcl::PointXYZ>::Ptr unique_filtered_cloud = removeDuplicatePoints(filtered_cloud, 0.05);  // Tolerancia de 5 cm

        if (unique_filtered_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Point cloud filtrado único está vacío después de eliminar duplicados.");
            return;
        }

        // Publicar los puntos filtrados únicos
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*unique_filtered_cloud, filtered_msg);
        filtered_msg.header.frame_id = "odom";
        filtered_msg.header.stamp = msg->header.stamp;
        filtered_pointcloud_publisher_->publish(filtered_msg);

        // Downsample the point cloud using a voxel grid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(unique_filtered_cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // Ajusta el tamaño de la hoja según sea necesario
        voxel_filter.filter(*downsampled_cloud);

        if (downsampled_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Point cloud downsampled está vacío.");
            return;
        }

        // Perform Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        performClustering(downsampled_cloud, cluster_indices);

        if (cluster_indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No se encontraron clusters.");
            return;
        }

        // Actualizar el seguimiento de obstáculos con la marca de tiempo del mensaje
        updateObstacleTracking(downsampled_cloud, cluster_indices, msg->header.stamp);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloudToOdom(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const rclcpp::Time& stamp)
    {
        // Buscar la transformación de 'camera_link_optical' a 'odom' en el tiempo del mensaje
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform("odom", "camera_link_optical", stamp, tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "No se pudo obtener la transformación: %s", ex.what());
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }

        // Extraer la traslación
        float tx = transform_stamped.transform.translation.x;
        float ty = transform_stamped.transform.translation.y;
        float tz = transform_stamped.transform.translation.z;

        // Extraer la rotación como un cuaternión
        float qx = transform_stamped.transform.rotation.x;
        float qy = transform_stamped.transform.rotation.y;
        float qz = transform_stamped.transform.rotation.z;
        float qw = transform_stamped.transform.rotation.w;

        // Convertir el cuaternión a Eigen::Quaternionf
        Eigen::Quaternionf q(qw, qx, qy, qz);

        // Crear la matriz de transformación
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << tx, ty, tz;
        transform.rotate(q);

        // Transformar el point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

        return transformed_cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr removeDuplicatePoints(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        double tolerance)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr unique_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Crear una cuadrícula espacial para eliminar duplicados
        std::unordered_set<std::pair<int, int>, pair_hash> grid;

        for (const auto& point : cloud->points)
        {
            int x_idx = static_cast<int>(std::floor(point.x / tolerance));
            int y_idx = static_cast<int>(std::floor(point.y / tolerance));

            std::pair<int, int> grid_idx = {x_idx, y_idx};

            if (grid.find(grid_idx) == grid.end())
            {
                grid.insert(grid_idx);
                unique_cloud->points.emplace_back(point);
            }
        }

        unique_cloud->width = unique_cloud->points.size();
        unique_cloud->height = 1;
        unique_cloud->is_dense = true;

        return unique_cloud;
    }

    void performClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices)
    {
        // Crear un árbol KD para el algoritmo de clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        kd_tree->setInputCloud(cloud);

        // Configurar la extracción de clusters Euclidianos
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.20);  // Tolerancia en metros (ajusta según sea necesario)
        ec.setMinClusterSize(10);      // Número mínimo de puntos para formar un cluster
        ec.setMaxClusterSize(2000);    // Número máximo de puntos en un cluster
        ec.setSearchMethod(kd_tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "Encontrados %zu clusters.", cluster_indices.size());
    }

    void updateObstacleTracking(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<pcl::PointIndices>& cluster_indices,
        const rclcpp::Time& msg_stamp)
    {
        rclcpp::Time current_time = msg_stamp;

        // Lista de posiciones de obstáculos detectados en el frame actual
        std::vector<geometry_msgs::msg::Point> detected_positions;

        // Calcular centroides de los clusters
        for (const auto& indices : cluster_indices)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud, indices.indices, centroid);

            geometry_msgs::msg::Point position;

            // Usar las coordenadas del centroid directamente (ya están en el frame 'odom')
            position.x = centroid[0];
            position.y = centroid[1];
            position.z = centroid[2];

            detected_positions.push_back(position);
        }

        // Asignar clusters a obstáculos rastreados
        assignClustersToObstacles(detected_positions, current_time);

        // Eliminar obstáculos que no han sido vistos por un tiempo
        removeLostObstacles(current_time);

        // Publicar obstáculos y marcadores
        publishObstaclesAndMarkers(current_time);
    }

    void assignClustersToObstacles(const std::vector<geometry_msgs::msg::Point>& detected_positions, const rclcpp::Time& current_time)
    {
        // Parámetros
        const double max_match_distance = 0.5;  // Distancia máxima para considerar una coincidencia (ajusta según sea necesario)

        // Crear una lista de posiciones detectadas sin asignar
        std::vector<bool> matched(detected_positions.size(), false);

        // Para cada obstáculo rastreado, intentar encontrar la posición detectada más cercana
        for (auto& obstacle_pair : tracked_obstacles_)
        {
            TrackedObstacle& obstacle = obstacle_pair.second;

            double min_distance = std::numeric_limits<double>::max();
            size_t best_match = 0;
            for (size_t i = 0; i < detected_positions.size(); ++i)
            {
                if (matched[i])
                    continue;

                double dist = euclideanDistance(obstacle.position, detected_positions[i]);
                if (dist < min_distance)
                {
                    min_distance = dist;
                    best_match = i;
                }
            }

            if (min_distance < max_match_distance)
            {
                // Actualizar la posición y velocidad del obstáculo
                obstacle.updatePosition(detected_positions[best_match], current_time);
                matched[best_match] = true;
            }
        }

        // Añadir nuevos obstáculos para detecciones sin asignar
        for (size_t i = 0; i < detected_positions.size(); ++i)
        {
            if (!matched[i])
            {
                int new_id = next_id_++;
                TrackedObstacle new_obstacle(new_id, detected_positions[i], current_time);
                tracked_obstacles_[new_id] = new_obstacle;
            }
        }
    }

    void removeLostObstacles(const rclcpp::Time& current_time)
    {
        const double obstacle_timeout = 2.0;  // Segundos para mantener un obstáculo después de no ser visto (ajusta según sea necesario)
        std::vector<int> obstacles_to_remove;
        for (const auto& obstacle_pair : tracked_obstacles_)
        {
            int id = obstacle_pair.first;
            const TrackedObstacle& obstacle = obstacle_pair.second;

            double time_since_seen = (current_time - obstacle.last_seen).seconds();
            if (time_since_seen > obstacle_timeout)
            {
                obstacles_to_remove.push_back(id);
            }
        }
        for (int id : obstacles_to_remove)
        {
            tracked_obstacles_.erase(id);
        }
    }

    void publishObstaclesAndMarkers(const rclcpp::Time& current_time)
    {
        visualization_msgs::msg::MarkerArray marker_array;
        custom_msgs::msg::Obstacles obstacles_msg;

        obstacles_msg.header.frame_id = "odom";  // Usando el frame "odom"
        obstacles_msg.header.stamp = current_time;

        for (const auto& obstacle_pair : tracked_obstacles_)
        {
            int id = obstacle_pair.first;
            const TrackedObstacle& obstacle = obstacle_pair.second;

            // Añadir datos del obstáculo al mensaje obstacles_msg
            obstacles_msg.ids.push_back(id);
            obstacles_msg.positions.push_back(obstacle.position);
            obstacles_msg.velocities.push_back(obstacle.velocity);

            // Crear un marcador para visualización
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";  // Usando el frame "odom"
            marker.header.stamp = current_time;
            marker.ns = "clusters";
            marker.id = id;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            marker.pose.position = obstacle.position;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8;

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);
            marker_array.markers.push_back(marker);
        }

        // Publicar el array de marcadores
        marker_publisher_->publish(marker_array);

        // Publicar el mensaje de obstáculos
        obstacle_publisher_->publish(obstacles_msg);
    }

    double euclideanDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        return std::sqrt(
            std::pow(p1.x - p2.x, 2) +
            std::pow(p1.y - p2.y, 2) +
            std::pow(p1.z - p2.z, 2));
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::Publisher<custom_msgs::msg::Obstacles>::SharedPtr obstacle_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_publisher_;

    int next_id_;
    std::unordered_map<int, TrackedObstacle> tracked_obstacles_;

    // TF2 buffer and listener
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudClusteringNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
