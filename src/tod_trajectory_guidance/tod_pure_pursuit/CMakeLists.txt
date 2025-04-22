cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_pure_pursuit)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(NOT_OPEN_SOURCE "Build open source and not for open source" ON)
if (${NOT_OPEN_SOURCE})
    add_definitions(-DNOT_OPEN_SOURCE_MACRO=1)
else()
    add_definitions(-DNOT_OPEN_SOURCE_MACRO=0)
endif()

option(INCLUDE_PERCEPTION_MODIFICATION "Build incl. perc mod" ON)
if (${PERCEPTION_MODIFICATION_TRUE_MACRO})
    add_definitions(-DPERCEPTION_MODIFICATION_TRUE_MACRO=1)
else()
    add_definitions(-DPERCEPTION_MODIFICATION_TRUE_MACRO=0)
endif()

if (${NOT_OPEN_SOURCE} OR ${INCLUDE_PERCEPTION_MODIFICATION})
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(tf2 REQUIRED)
    find_package(tf2_ros REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(tod_core REQUIRED)
    find_package(tod_trajectory_guidance_msgs REQUIRED)
    find_package(tod_status_msgs REQUIRED)
    find_package(tod_vehicle_msgs REQUIRED)

    install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
        )
    
    install(
        DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
        )
    
    include_directories(include)

    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME}_lib)
    ament_export_dependencies(rclcpp tod_status_msgs rosidl_default_runtime)
    ament_package()

    option(VEHICLE "Build vehicle targets" ON)
    if (${VEHICLE})
        add_subdirectory(src)
    else()
        message("skipping ${PROJECT_NAME} vehicle targets")
    endif()
else()
    message("Skipping ${PROJECT_NAME} package")
endif()
