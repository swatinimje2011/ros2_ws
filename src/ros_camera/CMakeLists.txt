cmake_minimum_required(VERSION 3.8)
project(ros_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(camera_node
  src/camera_node.cpp
)

ament_target_dependencies(camera_node
  rclcpp
)

install(TARGETS
  camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()