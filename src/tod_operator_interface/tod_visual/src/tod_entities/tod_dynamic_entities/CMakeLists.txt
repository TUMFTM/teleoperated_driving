add_library(tod_dynamic_entities 
  STATIC
    src/point_cloud_renderer.cpp
    src/driving_lane_renderer.cpp
    src/lanelet_map_renderer.cpp
    src/object_list_renderer.cpp
    src/path_control_points_renderer.cpp
    src/path_renderer.cpp
    src/position_controller.cpp
    src/trajectory_renderer.cpp
    src/video_renderer.cpp
    src/wheel_controller.cpp  
    src/utils/lanelet_mgrs_projector.cpp
)

target_include_directories(tod_dynamic_entities
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(tod_dynamic_entities PUBLIC tod_core nav_msgs tod_trajectory_guidance_msgs )
target_link_libraries(tod_dynamic_entities PUBLIC GLM::GLM tod_gl)
target_compile_features(tod_dynamic_entities PUBLIC cxx_std_17)

install(
  TARGETS tod_dynamic_entities
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
