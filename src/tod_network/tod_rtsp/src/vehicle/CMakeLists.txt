include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(TARGETS ${TARGETS} VehicleRtspServer)
add_executable(VehicleRtspServer rtsp_server_node.cpp rtsp_server.cpp rtsp_stream.cpp)
ament_target_dependencies(VehicleRtspServer rclcpp sensor_msgs tod_network_msgs tod_status_msgs tod_vehicle_msgs tod_config_msgs tod_core)
#ament_target_dependencies(OperatorRtspClients rclcpp sensor_msgs tod_msgs tod_core)
target_link_libraries(VehicleRtspServer ${GSTREAMER_LIBRARIES} ${GSTREAMER-RTP_LIBRARIES} ${GSTREAMER-RTSP-SERVER_LIBRARIES} ${GSTREAMER-APP_LIBRARIES} pthread )
target_include_directories(VehicleRtspServer PRIVATE ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER-RTP_INCLUDE_DIRS} ${GSTREAMER-APP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/tod_rtsp)

install(TARGETS ${TARGETS}
        DESTINATION lib/${PROJECT_NAME}
)