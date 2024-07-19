# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_camera_mananger_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED camera_mananger_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(camera_mananger_FOUND FALSE)
  elseif(NOT camera_mananger_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(camera_mananger_FOUND FALSE)
  endif()
  return()
endif()
set(_camera_mananger_CONFIG_INCLUDED TRUE)

# output package information
if(NOT camera_mananger_FIND_QUIETLY)
  message(STATUS "Found camera_mananger: 0.0.0 (${camera_mananger_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'camera_mananger' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${camera_mananger_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(camera_mananger_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${camera_mananger_DIR}/${_extra}")
endforeach()
