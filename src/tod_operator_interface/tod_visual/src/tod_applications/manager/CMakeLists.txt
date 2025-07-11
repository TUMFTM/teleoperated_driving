set(MANAGER_SOURCES
    src/manager.cpp
    src/manager_node.cpp
    src/application_layer/src/operator_state_layer.cpp
    src/application_layer/src/vehicle_state_layer.cpp
    src/application_layer/src/vehicle_interfaces_layer.cpp
    src/application_layer/src/operator_manager_docking_layer.cpp
    src/application_layer/src/ipv4_validity_checker.cpp

)

add_executable(manager ${MANAGER_SOURCES})

ament_target_dependencies(manager 
    PUBLIC 
    rclcpp 
    tod_helper 
    tod_core 
    nav_msgs
    tod_status_msgs
    tf2_geometry_msgs
)

target_include_directories(manager PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${tod_core_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/application_layer/include
)

target_link_libraries(manager PRIVATE tod_gl)
target_link_libraries(manager PRIVATE tinyfiledialogs)

set_target_properties(manager PROPERTIES 
    CXX_EXTENSIONS OFF
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

install(
    TARGETS manager
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)
