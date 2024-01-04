cmake_minimum_required(VERSION 3.5)
project(imodo)
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# Add include directories
include_directories(include)
# Create executable
add_executable(imodo src/imodo.cpp)
# Add dependencies to the executable
ament_target_dependencies(imodo
rclcpp
sensor_msgs
nav_msgs
std_msgs
OpenCV
)
# Install executable
install(TARGETS imodo
DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)

# Install the package.xml file
install(FILES package.xml
DESTINATION share/${PROJECT_NAME}
)
ament_package()