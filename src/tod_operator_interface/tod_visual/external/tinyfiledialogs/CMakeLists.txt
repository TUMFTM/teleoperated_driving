cmake_minimum_required(VERSION 3.10)

# Projektname setzen
project(tinyfiledialogs)

# Bibliothek hinzufügen
add_library(tinyfiledialogs STATIC
    src/tinyfiledialogs.c
)

# Header-Dateien dem Include-Pfad hinzufügen
target_include_directories(tinyfiledialogs PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Eigenschaften setzen
set_target_properties(tinyfiledialogs PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    POSITION_INDEPENDENT_CODE ON
)

# Optional: Installationsregeln für die Bibliothek und Header
install(TARGETS tinyfiledialogs
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)
