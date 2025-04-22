cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_topic_monitoring)
project(${PROJECT_NAME})

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tod_status_msgs REQUIRED)
find_package(tod_topic_monitoring_msgs REQUIRED)

add_executable(topic_monitor src/topic_monitor.cpp src/topic_monitoring_node.cpp)
ament_target_dependencies(topic_monitor rclcpp tod_status_msgs tod_topic_monitoring_msgs)

install(
    TARGETS
        topic_monitor
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()