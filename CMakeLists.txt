cmake_minimum_required(VERSION 3.10)
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
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(custom_msgs REQUIRED)  # Ensure custom_msgs is found
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)



# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Add the C++ executable
add_executable(point_cloud_clustering_node nodes/point_cloud_clustering.cpp)

# Link the required libraries to the executable
ament_target_dependencies(point_cloud_clustering_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_msgs
  custom_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  PCL
  Eigen3
)

# Link PCL libraries
target_link_libraries(point_cloud_clustering_node
  ${PCL_LIBRARIES}
  ${tf2_LIBRARIES}
  Eigen3::Eigen
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

# Export package dependencies
ament_export_dependencies(rosidl_default_runtime custom_msgs)
ament_package()
