set(QT_MINIMUM_VERSION 5.9)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
if(Qt5_VERSION VERSION_LESS QT_MINIMUM_VERSION)
    MESSAGE(FATAL_ERROR "Minimum supported Qt version: ${QT_MINIMUM_VERSION}.
        Installed version: ${Qt5_VERSION}.")
endif()

# ------------------------- Add Specific Input Devices -----------------
# adds target to variable DEVICE_TARGETS
add_subdirectory(virtual_device)
add_subdirectory(usb_input_device)

option(SENSO "Build Senso target" OFF)
if (${SENSO})
    add_definitions(-DSENSO_MACRO=1)
    add_subdirectory(senso_wheel)
else()
    add_definitions(-DSENSO_MACRO=0)
endif()

# ------------------ Specify Files for input_device_controller -----------
set(GENERAL_CPP
        input_device_controller.cpp
        ros_interface.cpp
        input_device_node.cpp  
        general/my_input_device.cpp
)

# --------------------------- Build Executables ------------------------
add_executable(InputDevice ${GENERAL_CPP})
target_compile_features(InputDevice PRIVATE cxx_std_17)

ament_target_dependencies(InputDevice ${${PROJECT_NAME}_EXPORTED_TARGETS} rclcpp tod_operator_msgs tod_status_msgs sensor_msgs)
target_include_directories(InputDevice PRIVATE ${tod_operator_msgs_INCLUDE_DIRS})
target_include_directories(InputDevice PRIVATE ${INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} general )
target_link_libraries(InputDevice Qt5::Widgets ${DEVICE_TARGETS} ${LIBS} stdc++fs)

add_executable(UsbEventHandler usb_event_handler_node.cpp usb_event_device/usb_event_device.cpp)
ament_target_dependencies(UsbEventHandler ${${PROJECT_NAME}_EXPORTED_TARGETS} rclcpp tod_operator_msgs tod_status_msgs sensor_msgs)
target_compile_features(UsbEventHandler PRIVATE cxx_std_17)
target_include_directories(UsbEventHandler PRIVATE ${INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(UsbEventHandler ${DEVICE_TARGETS} ${LIBS})

install(TARGETS
    InputDevice
    UsbEventHandler
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)
