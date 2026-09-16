cmake_minimum_required(VERSION 3.8)
project(tmr_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(lane_detector_node
  src/lane_detector.cpp
  src/lane_detector_node.cpp
)
ament_target_dependencies(lane_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
)
target_link_libraries(lane_detector_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  lane_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
