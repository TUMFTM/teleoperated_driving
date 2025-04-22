cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_communication_interface)
project(${PROJECT_NAME})
set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)




if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tod_network_monitoring_msgs REQUIRED)
find_package(tod_config_msgs REQUIRED)
find_package(tod_network_protocols REQUIRED)
find_package(tod_network REQUIRED)
find_package(rcl_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src/operator)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src/vehicle)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp sensor_msgs network_monitoring_msgs tod_config_msgs tod_network) 
ament_package()
