# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_shobot_log_recorder_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED shobot_log_recorder_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(shobot_log_recorder_FOUND FALSE)
  elseif(NOT shobot_log_recorder_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(shobot_log_recorder_FOUND FALSE)
  endif()
  return()
endif()
set(_shobot_log_recorder_CONFIG_INCLUDED TRUE)

# output package information
if(NOT shobot_log_recorder_FIND_QUIETLY)
  message(STATUS "Found shobot_log_recorder: 0.1.0 (${shobot_log_recorder_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'shobot_log_recorder' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT shobot_log_recorder_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(shobot_log_recorder_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${shobot_log_recorder_DIR}/${_extra}")
endforeach()
