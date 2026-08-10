cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_command_creation)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tod_operator_msgs REQUIRED)
find_package(tod_status_msgs REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_helper REQUIRED)
find_package(tod_core REQUIRED)

option(NOT_OPEN_SOURCE "Build open source and not for open source" ON)

include_directories(include)

add_subdirectory(src)

install(
    DIRECTORY 
    launch
    config
    DESTINATION 
    share/${PROJECT_NAME}
    )
    
ament_export_dependencies(rclcpp tod_operator_msgs tod_status_msgs tod_vehicle_msgs tod_helper tod_core)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gear_selector test/test_gear_selector.cpp)
  ament_target_dependencies(test_gear_selector tod_vehicle_msgs)
  target_include_directories(
    test_gear_selector PRIVATE include ${tod_vehicle_msgs_INCLUDE_DIRS})
endif()

ament_package()
