cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_helper)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tod_automation_msgs REQUIRED)
find_package(tod_operator_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

include_directories(include)
add_subdirectory(src)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(unit_tests
          test/geometry/testGeometryHelpers.cpp
          ) # unit tests
  ament_target_dependencies(unit_tests geometry_msgs tf2 tf2_geometry_msgs)
  target_link_libraries(unit_tests rclcpp::rclcpp) 

endif()

install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp tf2 tf2_ros tf2_geometry_msgs sensor_msgs geometry_msgs visualization_msgs nav_msgs tod_automation_msgs tod_operator_msgs autoware_planning_msgs)
ament_package()
