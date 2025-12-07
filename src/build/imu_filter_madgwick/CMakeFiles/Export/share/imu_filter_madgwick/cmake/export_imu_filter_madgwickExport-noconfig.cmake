#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "imu_filter_madgwick::imu_filter_madgwick" for configuration ""
set_property(TARGET imu_filter_madgwick::imu_filter_madgwick APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(imu_filter_madgwick::imu_filter_madgwick PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libimu_filter_madgwick.so"
  IMPORTED_SONAME_NOCONFIG "libimu_filter_madgwick.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS imu_filter_madgwick::imu_filter_madgwick )
list(APPEND _IMPORT_CHECK_FILES_FOR_imu_filter_madgwick::imu_filter_madgwick "${_IMPORT_PREFIX}/lib/libimu_filter_madgwick.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
