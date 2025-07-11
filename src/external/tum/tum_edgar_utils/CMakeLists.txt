cmake_minimum_required(VERSION 3.12)
set(PROJECT_NAME tum_edgar_utils)
project(${PROJECT_NAME})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()