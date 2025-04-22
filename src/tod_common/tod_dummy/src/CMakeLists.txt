set(TARGETS ${TARGETS} JoystickDummyPub)
add_executable(JoystickDummyPub joystick_dummy_pub.cpp)
ament_target_dependencies(JoystickDummyPub rclcpp tod_status_msgs sensor_msgs)
target_include_directories(JoystickDummyPub PRIVATE ${CMAKE_SOURCE_DIR}/include)

set(TARGETS ${TARGETS} StatusDummyPub)
add_executable(StatusDummyPub status_dummy_pub.cpp)
ament_target_dependencies(StatusDummyPub rclcpp tod_status_msgs)
target_include_directories(StatusDummyPub PRIVATE ${CMAKE_SOURCE_DIR}/include)

set(TARGETS ${TARGETS} PathDummyPub)
add_executable(PathDummyPub path_dummy_pub.cpp)
ament_target_dependencies(PathDummyPub rclcpp geometry_msgs autoware_auto_planning_msgs)
target_include_directories(PathDummyPub PRIVATE ${CMAKE_SOURCE_DIR}/include)

install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)
