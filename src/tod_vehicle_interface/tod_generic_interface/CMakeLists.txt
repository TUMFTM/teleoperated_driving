cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_generic_interface)
project(${PROJECT_NAME})

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_automation_msgs REQUIRED)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/base_interface.cpp
    src/actuation_interface.cpp
    src/sensing_interface.cpp
    src/automation_interface.cpp
)

# Create a library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    nav_msgs
    tod_vehicle_msgs
    tod_automation_msgs
)

# Install targets
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install headers
install(
    DIRECTORY include/
    DESTINATION include
)

# Export the library for use in other packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp sensor_msgs nav_msgs tod_vehicle_msgs)

# Package setup
ament_package()
