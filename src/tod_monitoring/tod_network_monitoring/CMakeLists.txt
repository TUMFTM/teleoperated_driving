cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_network_monitoring)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(BUILD_TESTS "Build the test programs" OFF)

include_directories(include)
include_directories(/usr/include)

# ROS2 integration
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tod_network_monitoring_msgs REQUIRED)

# network tester
add_executable(network_tester src/network_tester.cpp src/network_tester_node.cpp)
ament_target_dependencies(network_tester rclcpp tod_network_monitoring_msgs)
target_include_directories(network_tester PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS})

# network monitor
add_executable(network_monitor src/network_monitor.cpp src/network_monitor_node.cpp)
ament_target_dependencies(network_monitor rclcpp tod_network_monitoring_msgs)
target_include_directories(network_monitor PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS})

# packet_logger
add_executable(packet_logger src/packet_logger.cpp src/packet_logger_node.cpp)
ament_target_dependencies(packet_logger rclcpp tod_network_monitoring_msgs)
target_link_libraries(packet_logger pcap)
target_include_directories(packet_logger PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS})

if(BUILD_TESTS)
    # just for latency service testing
    add_executable(service_tester_latency test/test_latency_service.cpp)
    ament_target_dependencies(service_tester_latency rclcpp tod_network_monitoring_msgs)
    target_include_directories(service_tester_latency PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS})

    # just for bandwidth service testing
    add_executable(service_tester_bandwidth test/test_bandwidth_service.cpp)
    ament_target_dependencies(service_tester_bandwidth rclcpp tod_network_monitoring_msgs)
    target_include_directories(service_tester_bandwidth PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS})

    # just for packet_capture service testing
    add_executable(service_tester_packet_capture test/test_packet_capture_service.cpp)
    ament_target_dependencies(service_tester_packet_capture rclcpp tod_network_monitoring_msgs)
    target_include_directories(service_tester_packet_capture PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS})

    # just for network_monitor service testing
    add_executable(service_tester_network_monitor test/test_monitor_service.cpp)
    ament_target_dependencies(service_tester_network_monitor rclcpp tod_network_monitoring_msgs)
    target_include_directories(service_tester_network_monitor PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS})
endif()

install(
    TARGETS
        network_tester
        network_monitor
        packet_logger
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTS)
    install(
        TARGETS
            service_tester_latency
            service_tester_bandwidth
            service_tester_network_monitor
            service_tester_packet_capture
        DESTINATION lib/${PROJECT_NAME}
    )
endif()

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
################################################################################

# C++ only
################################################################################

# # with packet_logger
# add_executable(testit test/test.cpp src/network_monitor.cpp src/network_tester.cpp src/packet_logger.cpp)
# target_link_libraries(testit pcap)

# # without packet_logger
# add_executable(test test/test.cpp src/network_monitor.cpp src/network_tester.cpp)

################################################################################