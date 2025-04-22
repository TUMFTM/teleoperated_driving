## Vehicle Test Node
set(TARGETS ${TARGETS} VehicleSimNode)
add_executable(VehicleSimNode vehicle_sim_node.cpp vehicle_model.cpp)
ament_target_dependencies(VehicleSimNode rclcpp tod_vehicle_msgs tod_status_msgs tod_helper tod_core)
target_include_directories(VehicleSimNode PRIVATE ${tod_vehicle_msgs_INCLUDE_DIRS})

install(
    TARGETS ${TARGETS}  
    DESTINATION lib/${PROJECT_NAME}
    )