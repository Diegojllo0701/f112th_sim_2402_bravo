// point_cloud_clustering.cpp

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>

#include "obstacle_msgs/msg/obstacles.hpp"

#include <unordered_map>
#include <vector>
#include <cmath>

class TrackedObstacle
{
public:
    int id;
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Vector3 velocity;
    rclcpp::Time last_seen;

    TrackedObstacle(int obstacle_id, const geometry_msgs::msg::Point& pos, const rclcpp::Time& time)
        : id(obstacle_id), position(pos), last_seen(time)
    {
        velocity.x = 0.0;
        velocity.y = 0.0;
        velocity.z = 0.0;
    }
};

class PointCloudClusteringNode : public rclcpp::Node
{
public:
    PointCloudClusteringNode()
        : Node("point_cloud_clustering_node"), next_id_(0)
    {
        // Subscriber to point cloud data
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/points",  // Adjusted to your point cloud topic
            10,
            std::bind(&PointCloudClusteringNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for visualization markers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_clusters", 10);

        // Publisher for obstacle positions and velocities
        obstacle_publisher_ = this->create_publisher<obstacle_msgs::msg::Obstacles>("detected_obstacles", 10);

        RCLCPP_INFO(this->get_logger(), "Point Cloud Clustering Node has been started.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the ROS message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Apply PassThrough filter to remove floor points
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(pcl_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.1, 2.0);  // Adjust the limits based on your environment
        pass.filter(*filtered_cloud);

        if (filtered_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Filtered point cloud is empty.");
            return;
        }

        // Downsample the point cloud using a voxel grid filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(filtered_cloud);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f);  // Adjust the leaf size as needed
        voxel_filter.filter(*downsampled_cloud);

        if (downsampled_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Downsampled point cloud is empty.");
            return;
        }

        // Perform Euclidean clustering
        std::vector<pcl::PointIndices> cluster_indices;
        performClustering(downsampled_cloud, cluster_indices);

        if (cluster_indices.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No clusters found.");
            return;
        }

        // Update obstacle tracking
        updateObstacleTracking(downsampled_cloud, cluster_indices);
    }

    void performClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<pcl::PointIndices>& cluster_indices)
    {
        // Create a KD-Tree for the clustering algorithm
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        kd_tree->setInputCloud(cloud);

        // Set up the Euclidean Cluster Extraction
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.25);  // Tolerance in meters (adjust as needed)
        ec.setMinClusterSize(25);     // Minimum number of points to form a cluster
        ec.setMaxClusterSize(25000);  // Maximum number of points in a cluster
        ec.setSearchMethod(kd_tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "Found %zu clusters.", cluster_indices.size());
    }

    void updateObstacleTracking(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::vector<pcl::PointIndices>& cluster_indices)
    {
        rclcpp::Time current_time = this->now();

        // List of detected obstacle positions in the current frame
        std::vector<geometry_msgs::msg::Point> detected_positions;

        // Calculate centroids of clusters
        for (const auto& indices : cluster_indices)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud, indices.indices, centroid);

            geometry_msgs::msg::Point position;
            position.x = centroid[0];
            position.y = centroid[1];
            position.z = centroid[2];

            detected_positions.push_back(position);
        }

        // Assign clusters to tracked obstacles
        assignClustersToObstacles(detected_positions, current_time);

        // Remove obstacles not seen for a while
        removeLostObstacles(current_time);

        // Publish obstacles and markers
        publishObstaclesAndMarkers(current_time);
    }

    void assignClustersToObstacles(const std::vector<geometry_msgs::msg::Point>& detected_positions, const rclcpp::Time& current_time)
    {
        // Parameters
        const double max_match_distance = 0.5;  // Maximum distance to consider a match (adjust as needed)

        // Create a list of unmatched detected positions
        std::vector<bool> matched(detected_positions.size(), false);

        // For each tracked obstacle, try to find the nearest detected position
        for (auto& [id, obstacle] : tracked_obstacles_)
        {
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
                // Update obstacle position and velocity
                geometry_msgs::msg::Point prev_pos = obstacle.position;
                double dt = (current_time - obstacle.last_seen).seconds();
                if (dt > 0)
                {
                    obstacle.velocity.x = (detected_positions[best_match].x - prev_pos.x) / dt;
                    obstacle.velocity.y = (detected_positions[best_match].y - prev_pos.y) / dt;
                    obstacle.velocity.z = (detected_positions[best_match].z - prev_pos.z) / dt;
                }

                obstacle.position = detected_positions[best_match];
                obstacle.last_seen = current_time;
                matched[best_match] = true;
            }
        }

        // Add new obstacles for unmatched detections
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
        const double obstacle_timeout = 2.0;  // Seconds to keep an obstacle after last seen (adjust as needed)
        std::vector<int> obstacles_to_remove;
        for (const auto& [id, obstacle] : tracked_obstacles_)
        {
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
        obstacle_msgs::msg::Obstacles obstacles_msg;
        obstacles_msg.header.frame_id = "map";  // Adjust the frame as necessary
        obstacles_msg.header.stamp = current_time;

        for (const auto& [id, obstacle] : tracked_obstacles_)
        {
            // Create a marker for visualization
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";  // Adjust the frame as necessary
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

            // Set marker color (consistent for each obstacle)
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8;

            marker.lifetime = rclcpp::Duration::from_seconds(0.5);

            marker_array.markers.push_back(marker);

            // Add obstacle data to obstacles_msg
            obstacles_msg.ids.push_back(id);
            obstacles_msg.positions.push_back(obstacle.position);
            obstacles_msg.velocities.push_back(obstacle.velocity);
        }

        // Publish the marker array
        marker_publisher_->publish(marker_array);

        // Publish the obstacles with velocities
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
    rclcpp::Publisher<obstacle_msgs::msg::Obstacles>::SharedPtr obstacle_publisher_;

    int next_id_;
    std::unordered_map<int, TrackedObstacle> tracked_obstacles_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudClusteringNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
