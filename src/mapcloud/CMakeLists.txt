cmake_minimum_required(VERSION 3.5)
project(mapcloud)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)


# Add executable
add_executable(mapcloud_node src/mapcloud.cpp)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}  # 使用 PCL 提供的头文件目录
)

# Link the executable with the necessary libraries
ament_target_dependencies(mapcloud_node
  rclcpp
  nav_msgs
  sensor_msgs
  cv_bridge
  pcl
)

# Install the executable
install(TARGETS
  mapcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
