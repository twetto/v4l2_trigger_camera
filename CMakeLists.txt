cmake_minimum_required(VERSION 3.8)
project(v4l2_trigger_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtimulib_ros2 REQUIRED)

add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PRIVATE src)
ament_target_dependencies(camera_node rclcpp sensor_msgs rtimulib_ros2)

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
