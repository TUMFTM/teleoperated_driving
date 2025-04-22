cmake_minimum_required(VERSION 3.8)
project(tod_data_interface)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tod_status_msgs REQUIRED)
find_package(tod_network_protocols REQUIRED)

# Include directories
include_directories(include)

# Executables
add_executable(generic_tod_sender src/generic_sender_node.cpp src/generic_tod_sender.cpp)
ament_target_dependencies(generic_tod_sender tod_status_msgs tod_network_protocols rclcpp)

add_executable(generic_tod_receiver src/generic_receiver_node.cpp src/generic_tod_receiver.cpp)
ament_target_dependencies(generic_tod_receiver tod_status_msgs tod_network_protocols rclcpp)

# Install targets
install(TARGETS
  generic_tod_sender
  generic_tod_receiver
  DESTINATION lib/${PROJECT_NAME}
)

# Install other files (e.g., launch, config)
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
