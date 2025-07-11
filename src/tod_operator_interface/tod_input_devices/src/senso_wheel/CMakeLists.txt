set(THREADS_PREFER_PTHREAD_FLAG ON) 
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})

find_package(PCAN REQUIRED)
find_package(Threads REQUIRED)

set(SENSO_SOURCES
    src/senso_input_device.cpp
    src/senso_controller.cpp
    src/can_interface.cpp
    src/audi_steering_wheel_controller.cpp
)

add_library(senso_input_device
    ${SENSO_SOURCES}
)  

target_include_directories(senso_input_device 
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../general
        ${CMAKE_CURRENT_SOURCE_DIR}/src

    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/src 
)

target_link_libraries(senso_input_device 
    PRIVATE
        PCAN::PCAN
        Threads::Threads
)

set(DEVICE_TARGETS ${DEVICE_TARGETS} senso_input_device PARENT_SCOPE)
