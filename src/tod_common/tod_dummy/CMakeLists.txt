cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_dummy)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tod_status_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

add_subdirectory(src)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(rclcpp tod_status_msgs sensor_msgs geometry_msgs autoware_auto_planning_msgs)
ament_package()
