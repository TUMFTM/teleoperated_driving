set(TARGETS ${TARGETS} OperatorLaneProjection)
add_executable(OperatorLaneProjection operator_lane_projection.cpp)
ament_target_dependencies(OperatorLaneProjection rclcpp tod_vehicle_msgs nav_msgs tod_helper tod_core)

install(
    TARGETS ${TARGETS} 
    DESTINATION lib/${PROJECT_NAME})
