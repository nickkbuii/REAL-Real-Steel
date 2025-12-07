# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_imu_tools_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED imu_tools_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(imu_tools_FOUND FALSE)
  elseif(NOT imu_tools_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(imu_tools_FOUND FALSE)
  endif()
  return()
endif()
set(_imu_tools_CONFIG_INCLUDED TRUE)

# output package information
if(NOT imu_tools_FIND_QUIETLY)
  message(STATUS "Found imu_tools: 2.1.5 (${imu_tools_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'imu_tools' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${imu_tools_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(imu_tools_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${imu_tools_DIR}/${_extra}")
endforeach()
