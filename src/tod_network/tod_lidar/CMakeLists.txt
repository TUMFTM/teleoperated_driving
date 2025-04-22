cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_lidar)
project(${PROJECT_NAME})
set(CMAKE_CXX_STANDARD 17)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tod_vehicle_msgs  REQUIRED)
find_package(tod_core)
find_package(Draco REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

# Include PCL directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src/vehicle)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src/operator)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()


ament_export_dependencies(rclcpp rcpputils tf2_ros tf2_sensor_msgs tf2_geometry_msgs sensor_msgs tod_vehicle_msgs tod_core PCL )

install(
    DIRECTORY launch config 
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()