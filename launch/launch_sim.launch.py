import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Define package name and configurations
    package_name = 'f112th_sim_2402_bravo'

    # Argument to allow optional namespacing
    agent_ns = LaunchConfiguration('agent_ns')
    agent_ns_launch_arg = DeclareLaunchArgument('agent_ns', default_value='')

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'agent_ns': agent_ns, 'use_sim_time': 'true', 'use_ros2_control': 'true'}.items(),
    )

    # Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )])
    )

    # Entity Spawner Node
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        namespace=agent_ns,
        arguments=['-topic', 'robot_description', '-entity', 'my_bot'],
        output='screen'
    )

    # Controller Spawner for Ackermann Steering
    ackermann_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=agent_ns,
        arguments=['ackermann_controller'],
        output='screen'
    )

    # Joint Broadcaster Spawner
    joint_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=agent_ns,
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Twist Mux Configuration
    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        namespace=agent_ns,
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )

    # Joystick Node
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'joystick.launch.py'
        )]),
        launch_arguments={'agent_ns': agent_ns, 'use_sim_time': 'true'}.items(),
    )

    # Optional Emergency Brake and Scan Reader Nodes
    scan_reader_node = Node(
        package=package_name,
        executable='scan_subscriptor.py',
        output='screen'
    )
    emergency_brake_node = Node(
        package=package_name,
        executable='em_break.py',
        output='screen'
    )

    # Launch everything
    return LaunchDescription([
        agent_ns_launch_arg,
        rsp,
        gazebo,
        spawn_entity,
        ackermann_controller_spawner,
        joint_broadcaster_spawner,
        joystick,
        twist_mux_node,
        # Uncomment if needed:
        # scan_reader_node,
        # emergency_brake_node,
    ])
