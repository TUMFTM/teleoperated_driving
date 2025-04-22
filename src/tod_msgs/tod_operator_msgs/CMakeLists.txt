cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_operator_msgs)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/Color.msg"
    "msg/ColoredPoint.msg"
    "msg/ColoredPolygon.msg"
    "msg/KeyPress.msg"
    "msg/Mesh.msg"
    )

set(srv_files
    "srv/InputDevice.srv"
    )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES geometry_msgs std_msgs rosidl_default_generators
)

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(geometry_msgs std_msgs  rosidl_default_runtime)
ament_package()