from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to other launch files
    sim_launch_path = os.path.join(
        get_package_share_directory('f112th_sim_2402_bravo'),
        'launch',
        'launch_sim.launch.py'
    )
    slam_launch_path = os.path.join(
        get_package_share_directory('f112th_sim_2402_bravo'),
        'launch',
        'slam_localization_launch.py'
    )

    # Include the existing launch files
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sim_launch_path)
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_path)
    )

    # Node for point_cloud_clustering
    point_cloud_clustering_node = Node(
        package='f112th_sim_2402_bravo',
        executable='point_cloud_clustering_node',
        name='point_cloud_clustering_node',
        output='screen'
    )

    return LaunchDescription([
        sim_launch,
        slam_launch,
        point_cloud_clustering_node
    ])
