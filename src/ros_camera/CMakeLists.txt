cmake_minimum_required(VERSION 3.8)
project(ros_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(camera_node
  src/camera_node.cpp
)

ament_target_dependencies(camera_node
  rclcpp
)

add_executable(image_publisher
  src/image_publisher.cpp
)

ament_target_dependencies(image_publisher
  rclcpp
  rcl_interfaces
  sensor_msgs
)

add_executable(image_subscriber
  src/image_subscriber.cpp
)

ament_target_dependencies(image_subscriber
  rclcpp
  sensor_msgs
)

install(TARGETS
  camera_node
  image_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()