cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_transform)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tod_core REQUIRED)

set(TARGETS ${TARGETS} DynamicTransformPublisher)
add_executable(DynamicTransformPublisher src/dynamic_transform_publisher_node.cpp)
ament_target_dependencies(DynamicTransformPublisher rclcpp tf2_ros nav_msgs)

set(TARGETS ${TARGETS} StaticTransformPublisher)
add_executable(StaticTransformPublisher src/static_transform_publisher_node.cpp)
ament_target_dependencies(StaticTransformPublisher rclcpp tf2_ros tod_core)

install(
  DIRECTORY 
  launch
  config
  DESTINATION 
  share/${PROJECT_NAME}
)

install(TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
