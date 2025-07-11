set(VIDEO_MANAGER_SOURCES
    src/video_manager.cpp
    src/video_manager_node.cpp
    src/application_layer/src/video_state_layer.cpp
    src/application_layer/src/camera_position_layer.cpp
    src/application_layer/src/video_manager_docking_layer.cpp


)

add_executable(video_manager ${VIDEO_MANAGER_SOURCES})

ament_target_dependencies(video_manager 
    PUBLIC 
    rclcpp 
    tod_helper 
    tod_core 
    nav_msgs 
    tod_config_msgs
    tf2_geometry_msgs
)

target_include_directories(video_manager PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${tod_core_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/application_layer/include
)


target_link_libraries(video_manager PRIVATE tod_gl)

set_target_properties(video_manager PROPERTIES 
    CXX_EXTENSIONS OFF
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

install(
    TARGETS video_manager
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)
