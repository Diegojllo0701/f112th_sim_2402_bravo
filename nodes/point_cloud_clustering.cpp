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

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Include Eigen for transformation matrices
#include <Eigen/Dense>

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
            "/camera/points",  // Adjust this to your point cloud topic
            10,
            std::bind(&PointCloudClusteringNode::pointCloudCallback, this, std::placeholders::_1));

        // Publisher for visualization markers
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_clusters", 10);

        // Publisher for obstacle positions and velocities
        obstacle_publisher_ = this->create_publisher<custom_msgs::msg::Obstacles>("detected_obstacles", 10);

        RCLCPP_INFO(this->get_logger(), "Point Cloud Clustering Node has been started.");
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Point cloud frame ID: %s", msg->header.frame_id.c_str());

        // Convert the ROS message to a PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Check if point cloud is empty
        if (pcl_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
            return;
        }

        // Transform the point cloud to the 'odom' frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = transformPointCloudToOdom(pcl_cloud, msg->header.stamp);
        if (transformed_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Transformed point cloud is empty.");
            return;
        }

        // Apply PassThrough filter on the z-axis to remove floor points (z < 0.25)
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(transformed_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.25, 1.5);  // Remove points below 0.25
        pass.filter(*filtered_cloud);

        if (filtered_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Filtered point cloud is empty after PassThrough.");
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

        // Update obstacle tracking with the message timestamp
        updateObstacleTracking(downsampled_cloud, cluster_indices, msg->header.stamp);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloudToOdom(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const rclcpp::Time& stamp)
    {
        // Lookup the transform from 'camera_link_optical' to 'odom' at the time of the message
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform("odom", "camera_link_optical", stamp, tf2::durationFromSec(0.1));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }

        // Extract translation
        float tx = transform_stamped.transform.translation.x;
        float ty = transform_stamped.transform.translation.y;
        float tz = transform_stamped.transform.translation.z;

        // Extract rotation as a quaternion
        float qx = transform_stamped.transform.rotation.x;
        float qy = transform_stamped.transform.rotation.y;
        float qz = transform_stamped.transform.rotation.z;
        float qw = transform_stamped.transform.rotation.w;

        // Convert quaternion to Eigen::Quaternionf
        Eigen::Quaternionf q(qw, qx, qy, qz);

        // Create transformation matrix
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << tx, ty, tz;
        transform.rotate(q);

        // Transform the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

        return transformed_cloud;
    }

    void performClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<pcl::PointIndices>& cluster_indices)
    {
        // Create a KD-Tree for the clustering algorithm
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>);
        kd_tree->setInputCloud(cloud);

        // Set up the Euclidean Cluster Extraction
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.25);  // Tolerance in meters (adjust as needed)
        ec.setMinClusterSize(25);      // Minimum number of points to form a cluster
        ec.setMaxClusterSize(2000);    // Maximum number of points in a cluster
        ec.setSearchMethod(kd_tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        RCLCPP_INFO(this->get_logger(), "Found %zu clusters.", cluster_indices.size());
    }

    void updateObstacleTracking(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const std::vector<pcl::PointIndices>& cluster_indices,
        const rclcpp::Time& msg_stamp)
    {
        rclcpp::Time current_time = msg_stamp;

        // List of detected obstacle positions in the current frame
        std::vector<geometry_msgs::msg::Point> detected_positions;

        // Calculate centroids of clusters
        for (const auto& indices : cluster_indices)
        {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud, indices.indices, centroid);

            geometry_msgs::msg::Point position;

            // Use the centroid coordinates directly (already in 'odom' frame)
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
                // Update obstacle position and velocity using the enhanced method
                obstacle.updatePosition(detected_positions[best_match], current_time);
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

        obstacles_msg.header.frame_id = "odom";  // Using "odom" frame
        obstacles_msg.header.stamp = current_time;

        for (const auto& obstacle_pair : tracked_obstacles_)
        {
            int id = obstacle_pair.first;
            const TrackedObstacle& obstacle = obstacle_pair.second;

            // Add obstacle data to obstacles_msg
            obstacles_msg.ids.push_back(id);
            obstacles_msg.positions.push_back(obstacle.position);
            obstacles_msg.velocities.push_back(obstacle.velocity);

            // Create a marker for visualization
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom";  // Using "odom" frame
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

        // Publish the marker array
        marker_publisher_->publish(marker_array);

        // Publish the obstacles message
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
