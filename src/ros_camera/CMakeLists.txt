cmake_minimum_required(VERSION 3.8)
project(ros_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core videoio)
find_package(sensor_msgs REQUIRED)

add_executable(camera_node
  src/camera_node.cpp
)

ament_target_dependencies(camera_node
  rclcpp
)

add_library(webcam_capture
  src/webcam_capture.cpp
)

target_include_directories(webcam_capture PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(webcam_capture
  ${OpenCV_LIBS}
)

add_executable(image_publisher
  src/image_publisher.cpp
)

ament_target_dependencies(image_publisher
  cv_bridge
  rclcpp
  rcl_interfaces
  sensor_msgs
)

target_link_libraries(image_publisher
  webcam_capture
)

add_executable(image_subscriber
  src/image_subscriber.cpp
)

ament_target_dependencies(image_subscriber
  rclcpp
  sensor_msgs
)

install(TARGETS
  webcam_capture
  camera_node
  image_publisher
  image_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()