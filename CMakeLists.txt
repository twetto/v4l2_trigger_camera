cmake_minimum_required(VERSION 3.8)
project(v4l2_trigger_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rtimulib_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# libjpeg-turbo: decode MJPEG to mono8 in-process, see decodeGray() in camera_node.cpp
find_package(JPEG REQUIRED)

# Per-frame exposure and gain from the CSI source, stamped like the image.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraExposure.msg"
  DEPENDENCIES std_msgs
)

add_executable(camera_node src/camera_node.cpp)
target_include_directories(camera_node PRIVATE src)
ament_target_dependencies(camera_node rclcpp sensor_msgs rtimulib_ros2)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(camera_node ${JPEG_LIBRARIES} "${cpp_typesupport_target}")
target_include_directories(camera_node PRIVATE ${JPEG_INCLUDE_DIR})

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
