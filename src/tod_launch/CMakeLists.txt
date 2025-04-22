cmake_minimum_required(VERSION 3.5)
set(PROJECT_NAME tod_launch)
project(${PROJECT_NAME})

# Points to the directory containing our parameter files. Usually they are located in tod/config/config.
set(PARAM_DIR "${CMAKE_SOURCE_DIR}/../../config/config")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(utils)

install(
    DIRECTORY launch ${PARAM_DIR}
    DESTINATION share/${PROJECT_NAME}
    )
    
ament_package()