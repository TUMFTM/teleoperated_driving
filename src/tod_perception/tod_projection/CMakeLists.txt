cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_projection)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_helper REQUIRED)
find_package(tod_core REQUIRED)

include_directories(include)

add_subdirectory(src)

install(
    DIRECTORY 
    launch
    config
    DESTINATION 
    share/${PROJECT_NAME}
    )

ament_export_dependencies(rclcpp nav_msgs tod_vehicle_msgs tod_helper tod_core)
ament_package()