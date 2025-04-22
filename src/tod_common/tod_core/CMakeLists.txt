cmake_minimum_required(VERSION 3.8)
set(PROJECT_NAME tod_core)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}
  src/ParameterHandler.cpp
  src/VehicleParameters.cpp
  src/CameraParameters.cpp
  src/VideoParameters.cpp
  src/VehicleInformation.cpp
  src/TransformParameters.cpp
  src/LidarParameters.cpp
  src/EnvModelParameters.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -fPIC)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${YAML_CPP_INCLUDE_DIRS}
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} rclcpp tf2)
target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET) # so client can also call target_link_libraries(tod_core::tod_core)
ament_export_include_directories(include) # superfluous: handeled by INCLUDES DESTINATION
ament_export_libraries(${PROJECT_NAME}) # superfluous: handeled by HAS_LIBRARY_TARGET

if(BUILD_TESTING)
  install(DIRECTORY
      test/test_config
      DESTINATION share/${PROJECT_NAME}
  )
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(unit_tests test/unit_test.cpp) # unit tests
  target_link_libraries(unit_tests ${PROJECT_NAME})

  add_executable(test_node test/parameter_test_node.cpp) # build this node only for testing
  target_link_libraries(test_node ${PROJECT_NAME})
  install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})
endif()

ament_export_dependencies(ament_cmake rclcpp tf2 yaml-cpp) # other dependencies so client does not need to call find_package on them
ament_package()