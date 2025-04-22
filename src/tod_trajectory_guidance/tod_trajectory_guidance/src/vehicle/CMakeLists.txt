set(TARGETS ${TARGETS} TrajectoryGuidanceNode)
add_executable(TrajectoryGuidanceNode trajectory_guidance_node.cpp)
add_library(TrajectoryGuidance trajectory_guidance.cpp ) 
target_link_libraries(TrajectoryGuidanceNode TrajectoryGuidance)
target_include_directories(TrajectoryGuidance PRIVATE ${INCLUDE_DIRECTORIES})
ament_target_dependencies(TrajectoryGuidance rclcpp tod_trajectory_guidance_msgs tod_vehicle_msgs tod_status_msgs tod_safety_msgs)



install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)