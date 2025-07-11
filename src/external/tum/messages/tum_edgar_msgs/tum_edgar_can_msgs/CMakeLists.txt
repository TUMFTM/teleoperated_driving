cmake_minimum_required(VERSION 3.10)
project(tum_edgar_can_msgs)

set(CMAKE_CXX_STANDARD 14)

set(MSG_DEPS std_msgs geometry_msgs can_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(can_msgs REQUIRED)

set(MSG_FILES
  msg/TUMEdgarPowertrain1.msg
  msg/TUMEdgarMotion.msg
  msg/TUMEdgarSteering.msg
  msg/TUMEdgarWheelspeed.msg
  msg/TUMEdgarGPSHeading.msg
  msg/TUMEdgarMisc1.msg
  msg/TUMEdgarGatewayStates.msg
  msg/TUMEdgarSteeringWheelButtons.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES ${MSG_DEPS}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
