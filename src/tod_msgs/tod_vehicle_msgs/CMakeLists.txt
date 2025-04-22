cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_vehicle_msgs)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/CompressedPointCloud.msg"
    "msg/PrimaryControlCmd.msg"
    "msg/PrimaryVehicleState.msg"
    "msg/SafetyDriverStatus.msg"
    "msg/SecondaryControlCmd.msg"
    "msg/SecondaryVehicleState.msg"
    "msg/VideoInfo.msg"
    )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs rosidl_default_generators
)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs sensor_msgs std_msgs rosidl_default_runtime)
ament_package()
