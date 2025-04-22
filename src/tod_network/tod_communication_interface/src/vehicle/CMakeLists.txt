set(TARGETS ${TARGETS} NetworkMonitorServiceListener)
add_executable(NetworkMonitorServiceListener network_monitor_service_listener.cpp)
ament_target_dependencies(NetworkMonitorServiceListener PUBLIC rclcpp tod_network_monitoring_msgs tod_config_msgs tod_network tod_network_protocols)
target_include_directories(NetworkMonitorServiceListener PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS} ${tod_config_msgs_INCLUDE_DIRS} ${tod_network_INCLUDE_DIRS} ${tod_core_INCLUDE_DIRS})

set(TARGETS ${TARGETS} PacketCaptureServiceListener)
add_executable(PacketCaptureServiceListener packet_capture_service_listener.cpp)
ament_target_dependencies(PacketCaptureServiceListener PUBLIC rclcpp tod_network_monitoring_msgs tod_config_msgs tod_network tod_network_protocols)
target_include_directories(PacketCaptureServiceListener PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS} ${tod_config_msgs_INCLUDE_DIRS} ${tod_network_INCLUDE_DIRS} ${tod_core_INCLUDE_DIRS})

set(TARGETS ${TARGETS} VideoConfigServiceListener)
add_executable(VideoConfigServiceListener video_config_service_listener.cpp)
ament_target_dependencies(VideoConfigServiceListener PUBLIC rclcpp tod_network_monitoring_msgs tod_config_msgs tod_network tod_network_protocols)
target_include_directories(VideoConfigServiceListener PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS} ${tod_config_msgs_INCLUDE_DIRS} ${tod_network_INCLUDE_DIRS} ${tod_core_INCLUDE_DIRS})

set(TARGETS ${TARGETS} VideoParamServiceListener)
add_executable(VideoParamServiceListener video_param_service_listener.cpp)
ament_target_dependencies(VideoParamServiceListener PUBLIC rclcpp tod_network_monitoring_msgs tod_config_msgs tod_network tod_network_protocols)
target_include_directories(VideoParamServiceListener PRIVATE ${tod_network_monitoring_msgs_INCLUDE_DIRS} ${tod_config_msgs_INCLUDE_DIRS} ${tod_network_INCLUDE_DIRS} ${tod_core_INCLUDE_DIRS})

install(TARGETS ${TARGETS}
        DESTINATION lib/${PROJECT_NAME}
     )