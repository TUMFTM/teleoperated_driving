find_package(DL REQUIRED)

add_library(glad src/glad.c)
target_include_directories(glad PUBLIC include/)
target_link_libraries(glad PRIVATE DL::DL)

install(TARGETS glad
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
