include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(TARGETS ${TARGETS} OperatorRtspClients)
add_executable(OperatorRtspClients rtsp_client_nodes.cpp rtsp_clients.cpp)
ament_target_dependencies(OperatorRtspClients rclcpp sensor_msgs tod_network_msgs tod_status_msgs tod_vehicle_msgs tod_core)
#ament_target_dependencies(OperatorRtspClients rclcpp sensor_msgs tod_msgs tod_core)
target_link_libraries(OperatorRtspClients ${GSTREAMER_LIBRARIES} ${GSTREAMER-RTP_LIBRARIES} ${GSTREAMER-APP_LIBRARIES} pthread )
target_include_directories(OperatorRtspClients PRIVATE ${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER-RTP_INCLUDE_DIRS} ${GSTREAMER-APP_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR}/include/tod_rtsp)

install(TARGETS ${TARGETS}
        DESTINATION lib/${PROJECT_NAME}
)