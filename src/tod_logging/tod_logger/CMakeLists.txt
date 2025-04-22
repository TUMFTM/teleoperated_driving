cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_logger)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(diagnostic_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include-Path
include_directories(include)
add_subdirectory(src)

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
  DESTINATION 
  share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp diagnostic_msgs yaml-cpp)
ament_package()
