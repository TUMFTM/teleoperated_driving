cmake_minimum_required(VERSION 3.10)
set(PROJECT_NAME tod_visual)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/modules" ${CMAKE_MODULE_PATH})

if (POLICY CMP0072) # supress warning for find_package(OpenGL)
    cmake_policy (SET CMP0072 OLD)
endif()

# set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")

set(CMAKE_CXX_FLAGS_RELEASE "-O2") # Or "-O2", "-Os"
set(CMAKE_C_FLAGS_RELEASE "-O2")   # Also set for C flags if you have C code

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



find_package(ament_cmake REQUIRED)

option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    find_package(GLM REQUIRED)
    find_package(assimp REQUIRED)
    find_package(glfw3 3.2 REQUIRED)
    find_package(Freetype REQUIRED)
    find_package(OpenVR REQUIRED)
    find_package(yaml-cpp REQUIRED)
    find_package(OpenGL REQUIRED)
    find_package(Qhull REQUIRED)
    find_package(OpenCV REQUIRED)
    find_package(cv_bridge REQUIRED)

    find_package(rclcpp REQUIRED)
    find_package(pcl_ros REQUIRED)
    find_package(PCL REQUIRED COMPONENTS common io visualization)
    find_package(lanelet2_io REQUIRED)
    find_package(lanelet2_projection REQUIRED)
    find_package(lanelet2_core REQUIRED)

    find_package(nav_msgs REQUIRED)
    find_package(tf2_geometry_msgs REQUIRED)

    find_package(tod_helper REQUIRED)
    find_package(tod_core REQUIRED)
    find_package(tod_vehicle_msgs REQUIRED)
    find_package(tod_operator_msgs REQUIRED)
    find_package(tod_automation_msgs REQUIRED)
    find_package(tod_trajectory_guidance_msgs REQUIRED)
    find_package(tod_config_msgs REQUIRED)
    find_package(tod_network_monitoring_msgs REQUIRED)
    find_package(tod_status_msgs REQUIRED)

    add_subdirectory(src/tod_gl)
    add_subdirectory(src/tod_entities/tod_static_entities)
    add_subdirectory(src/tod_entities/tod_dynamic_entities)
    add_subdirectory(src/tod_applications/visual)
    add_subdirectory(src/tod_applications/manager)
    add_subdirectory(src/tod_applications/video_manager)
    add_subdirectory(external/glad)
    add_subdirectory(external/entt)
    add_subdirectory(external/stb_image)
    add_subdirectory(external/dearImgui)
    add_subdirectory(external/tinyfiledialogs)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

install(DIRECTORY
    launch
    resources
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
