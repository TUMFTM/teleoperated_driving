cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_safety_gate)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_status_msgs REQUIRED)
find_package(tod_topic_monitoring_msgs REQUIRED)

add_executable(safety_gate src/safety_gate.cpp src/safety_gate_node.cpp)
ament_target_dependencies(safety_gate rclcpp tod_status_msgs tod_topic_monitoring_msgs tod_vehicle_msgs)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  safety_gate
  DESTINATION lib/${PROJECT_NAME})

ament_package()