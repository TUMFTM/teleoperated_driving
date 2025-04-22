set(TARGETS ${TARGETS} VehicleStateMachine)
add_executable(VehicleStateMachine vehicle_state_machine_node.cpp vehicle_state_machine.cpp)
ament_target_dependencies(VehicleStateMachine rclcpp tod_status_msgs tod_vehicle_msgs)

install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)
