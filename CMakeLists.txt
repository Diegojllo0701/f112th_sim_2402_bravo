cmake_minimum_required(VERSION 3.5)
project(f112th_sim_2402_bravo)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AngleDistance.msg"
  "msg/Obstacles.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Add the C++ executable
add_executable(point_cloud_clustering_node nodes/point_cloud_clustering.cpp)

# Link the required libraries to the executable
ament_target_dependencies(point_cloud_clustering_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  PCL
)

# Install the C++ executable
install(TARGETS point_cloud_clustering_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  nodes/scan_subscriptor.py
  nodes/dist_finder.py
  nodes/control.py
  nodes/em_break.py
  nodes/ftg_error.py
  nodes/ftg_control.py
  nodes/tf2_control_node.py
  nodes/navigation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

# Export the package dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
