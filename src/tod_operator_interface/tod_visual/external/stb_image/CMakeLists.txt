# Füge die Library hinzu
add_library(stb_image src/stb_image.cpp)

# Setze Include-Verzeichnisse
target_include_directories(stb_image PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Installiere die Library
install(TARGETS stb_image
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Installiere die Header-Datei
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/stb_image/stb_image.h
        DESTINATION include/${PROJECT_NAME})
