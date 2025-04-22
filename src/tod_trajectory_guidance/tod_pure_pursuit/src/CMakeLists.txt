add_library(${PROJECT_NAME}_lib pure_pursuit.cpp vehicle_model.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${EXPORTED_TARGETS} rclcpp tod_status_msgs nav_msgs tf2_ros tod_core tf2_geometry_msgs geometry_msgs tod_trajectory_guidance_msgs tod_vehicle_msgs)
target_link_libraries(${PROJECT_NAME}_lib ${LIBS})                                              # ament_target_dependencies 
# target_include_directories(${PROJECT_NAME}_lib PRIVATE ${INCLUDE_DIRS} PUBLIC ../include)            # 
target_compile_features(${PROJECT_NAME}_lib PRIVATE cxx_std_17)

add_executable(${PROJECT_NAME}_node path_tracking_control_node.cpp path_tracking_control.cpp)
ament_target_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${EXPORTED_TARGETS} rclcpp tod_status_msgs  tf2_ros tf2_geometry_msgs tod_core tod_trajectory_guidance_msgs tod_vehicle_msgs)
target_link_libraries(${PROJECT_NAME}_node ${LIBS} ${PROJECT_NAME}_lib)
# target_include_directories(${PROJECT_NAME}_node PRIVATE ${INCLUDE_DIRS} PUBLIC ../include)
target_compile_features(${PROJECT_NAME}_node PRIVATE cxx_std_17)


add_executable(${PROJECT_NAME}_simulator_node path_simulator_node.cpp path_simulator.cpp)
ament_target_dependencies(${PROJECT_NAME}_simulator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${EXPORTED_TARGETS} rclcpp tod_status_msgs  tf2_ros tf2_geometry_msgs tod_core tod_trajectory_guidance_msgs tod_vehicle_msgs)
target_link_libraries(${PROJECT_NAME}_simulator_node ${LIBS} ${PROJECT_NAME}_lib)
# target_include_directories(${PROJECT_NAME}_node PRIVATE ${INCLUDE_DIRS} PUBLIC ../include)
target_compile_features(${PROJECT_NAME}_simulator_node PRIVATE cxx_std_17)

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}_simulator_node ${PROJECT_NAME}_lib
    #RUNTIME DESTINATION bin
    #LIBRARY DESTINATION lib
    DESTINATION lib/${PROJECT_NAME}
)