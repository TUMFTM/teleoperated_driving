cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_command_forwarder)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_status_msgs REQUIRED)

include_directories(include)

add_subdirectory(src)

install(
    DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME}
    )

ament_export_include_directories(include)
ament_export_dependencies(rclcpp tod_vehicle_msgs tod_status_msgs)
ament_package()