include(FindPackageHandleStandardArgs)

find_path(GLM_INCLUDE_DIR
  NAMES
    glm/glm.hpp
  PATHS
        ${CMAKE_CURRENT_SOURCE_DIR}/libs/glm
  DOC "GLM include directory"
)

find_package_handle_standard_args(GLM
  DEFAULT_MSG GLM_INCLUDE_DIR)

mark_as_advanced(FORCE GLM_INCLUDE_DIR)

if (GLM_FOUND)
  set(GLM_INCLUDE_DIRS "${GLM_INCLUDE_DIR}")
  if (NOT TARGET GLM::GLM)
    add_library(GLM::GLM INTERFACE IMPORTED)
    set_target_properties(GLM::GLM PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${GLM_INCLUDE_DIR})
  endif ()
endif ()

