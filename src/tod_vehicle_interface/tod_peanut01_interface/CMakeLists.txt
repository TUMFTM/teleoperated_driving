cmake_minimum_required(VERSION 3.8)
project(tod_peanut01_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tod_vehicle_msgs REQUIRED)

add_library(dry_run_state src/dry_run_state.cpp)
target_include_directories(dry_run_state PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(dry_run_state tod_vehicle_msgs)

add_executable(Peanut01DryRunInterface
    src/dry_run_interface_main.cpp
    src/dry_run_interface_node.cpp)
target_include_directories(Peanut01DryRunInterface PRIVATE include)
target_link_libraries(Peanut01DryRunInterface dry_run_state)
ament_target_dependencies(Peanut01DryRunInterface rclcpp tod_vehicle_msgs)

install(TARGETS dry_run_state Peanut01DryRunInterface
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_dry_run_state test/test_dry_run_state.cpp)
    target_link_libraries(test_dry_run_state dry_run_state)
    ament_target_dependencies(test_dry_run_state tod_vehicle_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(tod_vehicle_msgs)
ament_package()
