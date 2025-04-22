cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_rtsp)
project(${PROJECT_NAME})

set(DEPRECATION_FLAG "-Wno-deprecated-declarations -Wno-format")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${DEPRECATION_FLAG}")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FindPkgConfig)
include_directories(include)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tod_network_msgs REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_status_msgs REQUIRED)
find_package(tod_config_msgs REQUIRED)
find_package(tod_core REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER-RTSP-SERVER gstreamer-rtsp-server-1.0)
pkg_check_modules(GSTREAMER-APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER-RTP REQUIRED gstreamer-rtp-1.0)


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

# NOT FOR OPEN SOURCE
# option(QUALITY_ASSESSMENT "Build quality assessment target" OFF)
# if (${QUALITY_ASSESSMENT})
#     add_subdirectory(quality-assessment)
# endif()

install(
  DIRECTORY include/
  DESTINATION include
)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)

#ament_export_dependencies(rclcpp sensor_msgs geometry_msgs tod_network_msgs tod_network tod_core) 
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs tod_msgs tod_core) 
ament_package()