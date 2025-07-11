cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_input_devices)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tod_operator_msgs REQUIRED)
find_package(tod_status_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(src)
ament_export_dependencies(rclcpp tod_operator_msgs tod_status_msgs sensor_msgs std_msgs)
ament_package()
