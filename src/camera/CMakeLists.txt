cmake_minimum_required(VERSION 3.8)
project(camera)

# 默认编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# 添加可执行文件
add_executable(camera_display_node src/camera.cpp)

# 为可执行文件添加依赖
ament_target_dependencies(camera_display_node
  rclcpp
  OpenCV
  sensor_msgs
  image_transport
  cv_bridge
)

# 安装可执行文件
install(TARGETS
  camera_display_node
  DESTINATION lib/${PROJECT_NAME}
)

# 为测试配置自动lint检查
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 打包配置
ament_package()