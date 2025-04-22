set(TARGETS ${TARGETS} TopicLogger)
add_executable(TopicLogger topic_logger_node.cpp topic_logger.cpp)
ament_target_dependencies(TopicLogger rclcpp diagnostic_msgs yaml-cpp)
target_include_directories(TopicLogger
                         PUBLIC
                            ${YAML_CPP_INCLUDE_DIRS}  )
target_link_libraries(TopicLogger ${YAML_CPP_LIBRARIES})

install(
    TARGETS ${TARGETS}
    DESTINATION lib/${PROJECT_NAME}
)