import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Declare arguments
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("f112th_sim_2402_bravo"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file for slam_toolbox')

    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='bravo', 
        description='Namespace for the robot')

    # Define the SLAM node with remapped topics and a namespace
    start_localization_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/map', 'bravo/map'),
            ('/odom', 'bravo/odom'),
            ('/goal_pose', 'bravo/goal_pose')
        ]
    )

    # Create a group action to push the namespace
    slam_with_namespace = GroupAction(
        actions=[
            PushRosNamespace(robot_namespace),
            start_localization_slam_toolbox_node
        ]
    )

    # Create and return the LaunchDescription
    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_robot_namespace_cmd)
    ld.add_action(slam_with_namespace)

    return ld
