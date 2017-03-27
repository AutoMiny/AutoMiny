#----------------------------------------------------------------
# Generated CMake target import file for configuration "".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "realsense::realsense" for configuration ""
set_property(TARGET realsense::realsense APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(realsense::realsense PROPERTIES
  IMPORTED_LINK_INTERFACE_LIBRARIES_NOCONFIG "usb-1.0;-lpthread"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/arm-linux-gnueabihf/librealsense.so.1.12.1"
  IMPORTED_SONAME_NOCONFIG "librealsense.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS realsense::realsense )
list(APPEND _IMPORT_CHECK_FILES_FOR_realsense::realsense "${_IMPORT_PREFIX}/lib/arm-linux-gnueabihf/librealsense.so.1.12.1" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
