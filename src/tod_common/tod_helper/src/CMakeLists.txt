set(TARGETS ${TARGETS} ColoredPolygonViz)
add_executable(ColoredPolygonViz ColoredPolygonViz.cpp)
ament_target_dependencies(ColoredPolygonViz rclcpp tod_operator_msgs geometry_msgs tf2 tf2_geometry_msgs)

set(TARGETS ${TARGETS} ObjectListViz)
add_executable(ObjectListViz ObjectListViz.cpp)
ament_target_dependencies(ObjectListViz rclcpp tod_automation_msgs visualization_msgs tf2 tf2_geometry_msgs geometry_msgs)

install(TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME})
