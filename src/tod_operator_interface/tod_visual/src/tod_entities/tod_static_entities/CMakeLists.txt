add_library(tod_static_entities 
  STATIC 
    src/camera.cpp
    src/coordinate_system.cpp
    src/display.cpp
    src/floor.cpp
    src/grid.cpp
    src/orbital_point.cpp
)

target_include_directories(tod_static_entities 
  PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
    $<INSTALL_INTERFACE:include>
    ${tod_vehicle_msgs_INCLUDE_DIRS}
)

ament_target_dependencies(tod_static_entities PUBLIC tod_vehicle_msgs tod_core nav_msgs)
target_link_libraries(tod_static_entities PUBLIC GLM::GLM tod_gl)
target_compile_features(tod_static_entities PUBLIC cxx_std_17)

install(
  TARGETS tod_static_entities
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
