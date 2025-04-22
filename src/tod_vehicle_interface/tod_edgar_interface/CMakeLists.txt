cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME, tod_edgar_interface)
project(tod_edgar_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tum_edgar_can_msgs REQUIRED)
find_package(tum_autoware_vehicle_msgs REQUIRED)
find_package(tum_edgar_utils REQUIRED)
find_package(autoware_auto_vehicle_msgs REQUIRED)
find_package(autoware_auto_control_msgs REQUIRED)
find_package(tod_generic_interface REQUIRED)
find_package(tod_core REQUIRED)

# Create the executable
add_executable(ActuationInterfaceNode 
    src/edgar_actuation_interface_node.cpp 
    src/edgar_actuation_interface.cpp
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
    src/edgar_sensing_interface_node.cpp 
    src/edgar_sensing_interface.cpp
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

# Install the executable
install(
    TARGETS ActuationInterfaceNode
    DESTINATION lib/${PROJECT_NAME}
)
install(
    TARGETS SensingInterfaceNode
    DESTINATION lib/${PROJECT_NAME}
)

# Install other files (e.g., launch, config)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Package setup
ament_package()