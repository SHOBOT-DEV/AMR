# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_shobot_zone_management_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED shobot_zone_management_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(shobot_zone_management_FOUND FALSE)
  elseif(NOT shobot_zone_management_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(shobot_zone_management_FOUND FALSE)
  endif()
  return()
endif()
set(_shobot_zone_management_CONFIG_INCLUDED TRUE)

# output package information
if(NOT shobot_zone_management_FIND_QUIETLY)
  message(STATUS "Found shobot_zone_management: 0.1.0 (${shobot_zone_management_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'shobot_zone_management' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT shobot_zone_management_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(shobot_zone_management_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${shobot_zone_management_DIR}/${_extra}")
endforeach()
