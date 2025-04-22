cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_trajectory_guidance)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(tod_trajectory_guidance_msgs REQUIRED)

find_package(tod_core REQUIRED)
find_package(tod_status_msgs REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)
find_package(tod_operator_msgs REQUIRED)
find_package(tod_safety_msgs REQUIRED)

# Project-wide include path
include_directories(include)

# include operator targets
option(OPERATOR "Build operator targets" ON)
if (${OPERATOR})
    add_subdirectory(src/operator)
else()
    message("skipping ${PROJECT_NAME} operator targets")
endif()

# include vehicle targets
option(VEHICLE "Build vehicle targets" ON)
if (${VEHICLE})
    add_subdirectory(src/vehicle)
else()
    message("skipping ${PROJECT_NAME} vehicle targets")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# install launch and config
install(
    DIRECTORY 
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp tod_network tod_helper tod_core tod_trajectory_guidance_msgs) # add other dependencies
ament_package()