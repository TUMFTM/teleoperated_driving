set(TARGETS ${TARGETS} OperatorStateMachine)
add_executable(OperatorStateMachine operator_state_machine_node.cpp operator_state_machine.cpp)
ament_target_dependencies(OperatorStateMachine rclcpp tod_status_msgs)

install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)

