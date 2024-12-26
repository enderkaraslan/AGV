# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_display_camera_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED display_camera_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(display_camera_FOUND FALSE)
  elseif(NOT display_camera_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(display_camera_FOUND FALSE)
  endif()
  return()
endif()
set(_display_camera_CONFIG_INCLUDED TRUE)

# output package information
if(NOT display_camera_FIND_QUIETLY)
  message(STATUS "Found display_camera: 0.0.0 (${display_camera_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'display_camera' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${display_camera_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(display_camera_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${display_camera_DIR}/${_extra}")
endforeach()
