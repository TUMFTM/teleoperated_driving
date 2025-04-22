cmake_minimum_required(VERSION 3.8)
project(tod_rc-car_interface)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tod_generic_interface REQUIRED)
find_package(tod_core REQUIRED)

# Create the executable
add_executable(ActuationInterfaceNode 
    src/rc-car_actuation_interface_node.cpp 
    src/rc-car_actuation_interface.cpp
)

# Link dependencies for the executable
ament_target_dependencies(ActuationInterfaceNode
    rclcpp
    sensor_msgs
    nav_msgs
    tod_vehicle_msgs
    ackermann_msgs
    tod_generic_interface
    tod_core
)

# Add include directories
target_include_directories(ActuationInterfaceNode PRIVATE
    include
    ${CMAKE_INSTALL_PREFIX}/include
    ${tod_vehicle_msgs_INCLUDE_DIRS}
)


# Create the executable
add_executable(SensingInterfaceNode 
    src/rc-car_sensing_interface_node.cpp 
    src/rc-car_sensing_interface.cpp
)

# Link dependencies for the executable
ament_target_dependencies(SensingInterfaceNode
    rclcpp
    sensor_msgs
    nav_msgs
    tod_generic_interface
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
