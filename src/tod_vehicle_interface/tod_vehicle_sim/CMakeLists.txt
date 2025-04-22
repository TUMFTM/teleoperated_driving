cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_vehicle_sim)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_status_msgs REQUIRED)
find_package(tod_helper REQUIRED)
find_package(tod_core REQUIRED)

include_directories(include)

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

install(
    DIRECTORY 
    launch
    config 
    DESTINATION 
    share/${PROJECT_NAME}
    )

ament_export_dependencies(rclcpp nav_msgs tf2 tod_vehicle_msgs tod_status_msgs tod_helper tod_core)
ament_package()