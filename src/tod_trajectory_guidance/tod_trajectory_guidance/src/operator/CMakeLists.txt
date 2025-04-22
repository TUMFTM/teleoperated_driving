set(TARGETS ${TARGETS} OperatorPathCreator)
add_executable(OperatorPathCreator path_creator.cpp path_helper.cpp)
ament_target_dependencies(OperatorPathCreator rclcpp nav_msgs tod_trajectory_guidance_msgs tod_status_msgs tf2_geometry_msgs tf2 geometry_msgs tod_operator_msgs tod_vehicle_msgs tod_core)
# target_include_directories(OperatorPathCreator PRIVATE ${tod_msgs_INCLUDE_DIRS} ) # ${trajectory_guidance_INCLUDE_DIRS}


install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)
