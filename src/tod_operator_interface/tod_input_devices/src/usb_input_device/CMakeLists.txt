set(USB_SOURCES
    usb_input_device.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../general/my_input_device.cpp)

set(TARGET usb_input_device)
add_library(usb_input_device
    ${USB_SOURCES}
) 


target_include_directories(usb_input_device
    PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../general
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)



install(TARGETS usb_input_device
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(DEVICE_TARGETS ${DEVICE_TARGETS} usb_input_device PARENT_SCOPE)
