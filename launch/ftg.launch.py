# library to move between files and folders in the O.S.
import os

from ament_index_python.packages import get_package_share_directory

# libraries to define the Launch file and Function
from launch import LaunchDescription


from launch_ros.actions import Node

def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='f112th_sim_2402_bravo' #<--- CHANGE ME

    ftg_error_node = Node(
        package= package_name,  # Replace with your package name
        executable='ftg_error.py',  # Ensure this matches the executable name
        output='screen'
    )

    ftg_control_node = Node(
        package= package_name,  # Replace with your package name
        executable='ftg_control.py',  # Ensure this matches the executable name
        output='screen'
    )

    # Launch them all!
    return LaunchDescription([
        ftg_error_node,
        ftg_control_node
    ])