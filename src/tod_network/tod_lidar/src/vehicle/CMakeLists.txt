set(TARGETS ${TARGETS} PointCloudEncoderNode)
add_executable(PointCloudEncoderNode point_cloud_encoder_node.cpp point_cloud_encoder.cpp)
ament_target_dependencies(PointCloudEncoderNode rclcpp rcpputils tod_vehicle_msgs sensor_msgs tod_core pcl_conversions)
target_link_libraries(PointCloudEncoderNode draco::draco ${PCL_LIBRARIES})


install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)
