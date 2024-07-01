cmake_minimum_required(VERSION 3.5)
project(scout_navigation)

# Set compile options
add_compile_options(-std=c++17 -Wall)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(
  include
)

# Add library for gridmap_converter
add_library(gridmap_converter src/gridmap_converter.cpp)

# Link library with dependencies
ament_target_dependencies(gridmap_converter 
  rclcpp 
  std_msgs 
  nav_msgs 
  grid_map_ros 
  grid_map_msgs 
)

# Add executable for gridmap_converter_node
add_executable(gridmap_converter_node src/nodes/gridmap_converter_node.cpp)

# Link node executable to the gridmap_converter library
target_link_libraries(gridmap_converter_node gridmap_converter)

# Specify dependencies for the node executable
ament_target_dependencies(gridmap_converter_node 
  rclcpp 
  std_msgs 
  nav_msgs 
  grid_map_ros 
  grid_map_msgs 
)

# Add executable for occmap_to_costmap node
add_executable(occmap_to_costmap src/occmap_to_costmap.cpp)

# Specify dependencies for the occmap_to_costmap node executable
ament_target_dependencies(occmap_to_costmap 
  rclcpp 
  nav2_costmap_2d 
  nav_msgs 
  tf2 
  tf2_ros
)

# Install targets
install(
  TARGETS gridmap_converter gridmap_converter_node occmap_to_costmap
  DESTINATION lib/${PROJECT_NAME}
)

# Install other project-specific directories
install(
  DIRECTORY launch config maps rviz slam_params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

