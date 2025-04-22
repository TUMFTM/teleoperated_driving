
set(TARGETS ${TARGETS} PointCloudDecoderNode)
add_executable(PointCloudDecoderNode point_cloud_decoder_node.cpp point_cloud_decoder.cpp) 
ament_target_dependencies(PointCloudDecoderNode rclcpp rcpputils tod_core sensor_msgs tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs tod_vehicle_msgs)
target_link_libraries(PointCloudDecoderNode draco::draco )


install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)
