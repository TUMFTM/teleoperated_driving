set(QT_MINIMUM_VERSION 5.9)
find_package(Qt5 REQUIRED COMPONENTS QuickWidgets)
if(Qt5_VERSION VERSION_LESS QT_MINIMUM_VERSION)
    MESSAGE(FATAL_ERROR "Minimum supported Qt version: ${QT_MINIMUM_VERSION}.
        Installed version: ${Qt5_VERSION}.")
endif()

set(VIRTUAL_SOURCES
    virtual_input_device.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../general/my_input_device.cpp)

qt5_wrap_ui(VIRTUAL_HEADERS_MOC ui/joystickwindow.ui)
qt5_add_resources(VIRTUAL_RESOURCES resources/qml.qrc)
qt5_wrap_cpp(VIRTUAL_SOURCES virtual_input_device.hpp)

set(TARGET virtual_input_device)
add_library(virtual_input_device
    ${VIRTUAL_SOURCES}
    ${VIRTUAL_HEADERS_MOC}
    ${VIRTUAL_RESOURCES}
)

add_custom_target(resursi
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_LIST_DIR}/resources/ ${CMAKE_INSTALL_PREFIX}/lib/tod_input_devices/
)

add_dependencies(virtual_input_device resursi)

target_include_directories(virtual_input_device
    PRIVATE
        resources
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../general
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(virtual_input_device
    PRIVATE
        Qt5::QuickWidgets
)

install(TARGETS virtual_input_device
    DESTINATION lib/${PROJECT_NAME}
)

set(DEVICE_TARGETS ${DEVICE_TARGETS} virtual_input_device PARENT_SCOPE)
