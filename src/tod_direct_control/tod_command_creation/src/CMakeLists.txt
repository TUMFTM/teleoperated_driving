set(TARGETS ${TARGETS} OperatorCommandCreator)
add_executable(OperatorCommandCreator command_creator_node.cpp command_creator.cpp)
ament_target_dependencies(OperatorCommandCreator rclcpp tod_operator_msgs tod_status_msgs tod_vehicle_msgs tod_helper tod_core)
target_include_directories(OperatorCommandCreator PRIVATE ${tod_vehicle_msgs_INCLUDE_DIRS} ${tod_operator_msgs_INCLUDE_DIRS})

install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)
