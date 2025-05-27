
find_path(PCAN_INCLUDE_DIR
  NAMES
    PCANBasic.h
  PATH_SUFFIXES
  NO_DEFAULT_PATH
  PATHS
        /usr/include/
  DOC "PCAN include directory"
)
mark_as_advanced(PCAN_INCLUDE_DIR)

find_library(PCAN_LIBRARY
  NAMES pcanbasic
  PATH_SUFFIXEs
  NO_DEFAULT_PATH
  PATHS
        /usr/lib/
  DOC "PCAN API library")
mark_as_advanced(PCAN_LIBRARY)


include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PCAN
  REQUIRED_VARS PCAN_LIBRARY PCAN_INCLUDE_DIR)

if (PCAN_FOUND)
  set(PCAN_INCLUDE_DIRS "${PCAN_INCLUDE_DIR}")
  set(PCAN_LIBRARIES "${PCAN_LIBRARY}")
  if (NOT TARGET PCAN::PCAN)
    add_library(PCAN::PCAN UNKNOWN IMPORTED)
    set_target_properties(PCAN::PCAN PROPERTIES
      IMPORTED_LOCATION "${PCAN_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${PCAN_INCLUDE_DIR}")
  endif ()
endif ()

