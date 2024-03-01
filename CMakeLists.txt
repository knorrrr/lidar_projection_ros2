cmake_minimum_required(VERSION 3.8)
find_package(ament_cmake_auto REQUIRED)
project(lidar_projection_ros2)

ament_auto_find_build_dependencies()
ament_auto_add_executable(lidar_projection_node
DIRECTORY src include
)
ament_auto_package(INSTALL_TO_SHARE 
launch
params)