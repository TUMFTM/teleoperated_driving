set(TOD_GL_SOURCES
    # core
    src/core/application.cpp
    src/core/cursor_position.cpp
    src/core/graphics_context.cpp
    src/core/layer.cpp
    src/core/layer_stack.cpp
    src/core/model_loader.cpp
    src/core/scene_layer.cpp
    src/core/scene_layer_stack.cpp
    src/core/scene_application.cpp
    src/core/state_manager.cpp
    src/core/window.cpp
    # events
    src/events/event.cpp
    src/events/application_event.cpp
    src/events/scene_layer_event.cpp
    src/events/layer_event.cpp
    src/events/mouse_event.cpp
    src/events/key_event.cpp
    # layers
    src/layers/debug_layer.cpp
    src/layers/docking_layer.cpp
    src/layers/docking_scene_layer.cpp

    src/layers/imgui_layer.cpp
    src/layers/imgui_scene_layer.cpp
    src/layers/io_layer.cpp
    # renderer
    src/renderer/data_container.cpp
    src/renderer/glfw_window.cpp
    src/renderer/opengl_context.cpp
    src/renderer/opengl_renderer_api.cpp
    src/renderer/renderer_command.cpp
    src/renderer/renderer.cpp
    src/renderer/renderer_api.cpp
    # ros_interface
    ## subscribing
    src/ros_interface/ros_interface.cpp
    src/ros_interface/subscribing_components/automation_status_component.cpp
    src/ros_interface/subscribing_components/point_cloud_component.cpp
    src/ros_interface/subscribing_components/driving_lane_component.cpp
    src/ros_interface/subscribing_components/image_component.cpp
    src/ros_interface/subscribing_components/joy_stick_component.cpp
    src/ros_interface/subscribing_components/odometry_component.cpp
    src/ros_interface/subscribing_components/network_metrics_component.cpp
    src/ros_interface/subscribing_components/path_component.cpp
    src/ros_interface/subscribing_components/path_control_points_component.cpp
    src/ros_interface/subscribing_components/predicted_object_component.cpp
    src/ros_interface/subscribing_components/primary_control_component.cpp
    src/ros_interface/subscribing_components/primary_vehicle_state_component.cpp
    src/ros_interface/subscribing_components/secondary_control_component.cpp
    src/ros_interface/subscribing_components/secondary_vehicle_state_component.cpp
    src/ros_interface/subscribing_components/tod_status_component.cpp
    src/ros_interface/subscribing_components/trajectory_component.cpp
    src/ros_interface/subscribing_components/trajectory_guidance_state_component.cpp

    ## publishing
    src/ros_interface/publishing_components/manager_button_status_component.cpp
    ## services
    src/ros_interface/service_components/packet_capture_component.cpp
    src/ros_interface/service_components/video_config_component.cpp
    src/ros_interface/service_components/input_device_component.cpp
    src/ros_interface/service_components/network_monitor_component.cpp
    # scene
    src/scene/scene.cpp
    src/scene/scene_serialization.cpp
    src/scene/camera_controller.cpp
    # systems
    src/systems/camera_system.cpp
    src/systems/transform_system.cpp
    src/systems/shader_system.cpp
    src/systems/vr_system.cpp
    # utils
    src/utils/utils.cpp
)

add_library(tod_gl ${TOD_GL_SOURCES})

ament_target_dependencies(tod_gl PUBLIC 
    rclcpp 
    tod_vehicle_msgs
    tod_config_msgs
    tod_automation_msgs
    tod_network_monitoring_msgs
    tod_trajectory_guidance_msgs
    tod_status_msgs
    tod_operator_msgs
    tod_core
    tf2_geometry_msgs 
    nav_msgs 
    lanelet2_io 
    lanelet2_core 
    lanelet2_projection 
    PCL 
    pcl_ros
    cv_bridge
)

target_include_directories(tod_gl
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${tod_vehicle_msgs_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${FREETYPE_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(tod_gl 
    PRIVATE
        OpenGL::GLU
        glfw
        OpenVR::OpenVR
        assimp
    PUBLIC
        GLM::GLM
        ${YAML_CPP_LIBRARIES}
        ${FREETYPE_LIBRARIES}
        imgui
        glad
        entt
        stb_image
        ${PCL_LIBRARIES}
        ${pcl_ros_LIBRARIES}
        ${QHULL_LIBRARIES}
        ${OpenCV_LIBS}
)

target_compile_features(tod_gl PUBLIC cxx_std_17)

install(
    TARGETS tod_gl
    EXPORT export_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)
