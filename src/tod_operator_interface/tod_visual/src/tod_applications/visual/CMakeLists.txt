set(VISUAL_SOURCES 
  # application layer
  application_layer/src/drive_info_layer.cpp
  application_layer/src/scene_layer.cpp
  application_layer/src/settings_layer.cpp
  # application_layer/src/speed_layer.cpp
  application_layer/src/state_layer.cpp
  # application_layer/src/switch_camera_layer.cpp
  # application_layer/src/traffic_sign_layer.cpp
  application_layer/src/trajectory_guidance_state_layer.cpp
  application_layer/src/ui_layer.cpp
  # application_layer/src/video_layer_new.cpp
  application_layer/src/video_layer.cpp
  application_layer/src/view_port_layer.cpp
  application_layer/src/visual_io_layer.cpp
  # node
  visual_node.cpp
)

add_executable(visual ${VISUAL_SOURCES})

ament_target_dependencies(visual 
  PUBLIC
  rclcpp
  tod_operator_msgs # TODO: Get rid of ros msg dependencies in application layers
  tod_trajectory_guidance_msgs # Needed for templating
  tod_helper
  tod_core
  nav_msgs
  tf2_geometry_msgs)

target_include_directories(visual 
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/application_layer/include
  ${CMAKE_SOURCE_DIR}/src/tod_entities/tod_static_entities/include
  ${CMAKE_SOURCE_DIR}/src/tod_entities/tod_dynamic_entities/include
  ${tod_core_INCLUDE_DIRS}
  ${tod_operator_msgs_INCLUDE_DIRS}

)

target_link_libraries(visual 
  PRIVATE
  tod_gl
  tod_static_entities
  tod_dynamic_entities
)

set_target_properties(visual PROPERTIES CXX_EXTENSIONS OFF)
target_compile_features(visual PUBLIC cxx_std_17)

install(
  TARGETS visual
  DESTINATION lib/${PROJECT_NAME}
)
