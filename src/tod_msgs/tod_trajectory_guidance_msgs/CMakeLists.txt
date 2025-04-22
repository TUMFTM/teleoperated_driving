cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_trajectory_guidance_msgs)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

set(msg_files
    "msg/ControlPoint.msg"
    "msg/ControlPoints.msg"
    "msg/PathPoint.msg"
    "msg/TrajectoryPoint.msg"
    "msg/Trajectory.msg"
    "msg/TrajectoryGuidanceControl.msg"
    "msg/Path.msg"
    "msg/TrajectoryGuidanceState.msg"
    "msg/PpLog.msg"

)



rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs rosidl_default_generators
)

ament_export_dependencies(std_msgs geometry_msgs sensor_msgs rosidl_default_runtime)
ament_package()
