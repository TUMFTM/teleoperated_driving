set(TARGETS ${TARGETS} ForwardPrimaryCtrlCmd)
add_executable(ForwardPrimaryCtrlCmd forward_primary_ctrl_cmd.cpp forward_primary_ctrl_cmd_node.cpp)
ament_target_dependencies(ForwardPrimaryCtrlCmd rclcpp tod_status_msgs tod_vehicle_msgs)

install(TARGETS ${TARGETS}
        DESTINATION lib/${PROJECT_NAME})
