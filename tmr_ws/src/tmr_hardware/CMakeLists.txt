cmake_minimum_required(VERSION 3.8)
project(tmr_hardware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(arduino_bridge_node src/arduino_bridge_node.cpp)
ament_target_dependencies(arduino_bridge_node
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS
  arduino_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
