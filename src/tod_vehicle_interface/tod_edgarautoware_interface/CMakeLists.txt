cmake_minimum_required(VERSION 3.8)
project(tod_edgarautoware_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_automation_msgs REQUIRED)
find_package(tum_edgar_can_msgs REQUIRED)
find_package(tum_autoware_vehicle_msgs REQUIRED)
find_package(tum_edgar_utils REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(autoware_auto_perception_msgs REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)

find_package(tier4_external_api_msgs REQUIRED)
find_package(tod_generic_interface REQUIRED)
find_package(tod_core REQUIRED)


# Create the executable
add_executable(ActuationInterfaceNode 
    src/edgarautoware_actuation_interface_node.cpp 
    src/edgarautoware_actuation_interface.cpp
)

# Link dependencies for the executable
ament_target_dependencies(ActuationInterfaceNode
    rclcpp
    sensor_msgs
    nav_msgs
    tod_vehicle_msgs
    tum_edgar_can_msgs
    tum_autoware_vehicle_msgs
    tum_edgar_utils
    autoware_auto_vehicle_msgs
    autoware_auto_control_msgs
    tier4_external_api_msgs
    tod_generic_interface
)

# Add include directories
target_include_directories(ActuationInterfaceNode PRIVATE
    include
    ${CMAKE_INSTALL_PREFIX}/include
    ${tod_vehicle_msgs_INCLUDE_DIRS}
)

# Create the executable
add_executable(SensingInterfaceNode 
    src/edgarautoware_sensing_interface_node.cpp 
    src/edgarautoware_sensing_interface.cpp
)

# Link dependencies for the executable
ament_target_dependencies(SensingInterfaceNode
    rclcpp
    sensor_msgs
    nav_msgs
    tod_vehicle_msgs
    tum_edgar_can_msgs
    tum_autoware_vehicle_msgs
    tum_edgar_utils
    autoware_auto_vehicle_msgs
    autoware_auto_control_msgs
    tod_generic_interface
    tod_core
)

# Add include directories
target_include_directories(SensingInterfaceNode PRIVATE
    include
    ${CMAKE_INSTALL_PREFIX}/include
    ${tod_vehicle_msgs_INCLUDE_DIRS}
)

# Create the executable
add_executable(AutomationInterfaceNode 
    src/edgarautoware_automation_interface_node.cpp 
    src/edgarautoware_automation_interface.cpp
)

# Link dependencies for the executable
ament_target_dependencies(AutomationInterfaceNode
    rclcpp
    sensor_msgs
    nav_msgs
    tod_vehicle_msgs
    tum_edgar_can_msgs
    tum_autoware_vehicle_msgs
    tum_edgar_utils
    autoware_adapi_v1_msgs
    autoware_auto_perception_msgs
    autoware_auto_planning_msgs
    tod_generic_interface
    tod_core
    tod_automation_msgs
)

# Add include directories
target_include_directories(AutomationInterfaceNode PRIVATE
    include
    ${CMAKE_INSTALL_PREFIX}/include
    ${tod_vehicle_msgs_INCLUDE_DIRS}
)

# Install the executable
install(
    TARGETS ActuationInterfaceNode
    DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS SensingInterfaceNode
    DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS AutomationInterfaceNode
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Install other files (e.g., launch, config)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Package setup
ament_package()
