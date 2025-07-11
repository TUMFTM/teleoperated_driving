cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tum_autoware_vehicle_msgs)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

set(MSG_FILES
    "msg/HeadLightReport.msg"
    "msg/HighBeamReport.msg"
    "msg/HonkReport.msg"
    "msg/WiperReport.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES 
  "std_msgs"
  "builtin_interfaces"
)

ament_export_dependencies(std_msgs builtin_interfaces rosidl_default_runtime)
ament_package()