cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_network_monitoring_msgs)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/NetworkMetrics.msg"
    )

set(srv_files
    "srv/BandwidthService.srv"
    "srv/LatencyService.srv"
    "srv/NetworkMonitorService.srv"
    "srv/PacketCaptureService.srv"
    )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs rosidl_default_generators
)

ament_export_dependencies(std_msgs rosidl_default_runtime)
ament_package()