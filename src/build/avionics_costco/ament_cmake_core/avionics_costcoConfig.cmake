# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_avionics_costco_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED avionics_costco_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(avionics_costco_FOUND FALSE)
  elseif(NOT avionics_costco_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(avionics_costco_FOUND FALSE)
  endif()
  return()
endif()
set(_avionics_costco_CONFIG_INCLUDED TRUE)

# output package information
if(NOT avionics_costco_FIND_QUIETLY)
  message(STATUS "Found avionics_costco: 0.0.0 (${avionics_costco_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'avionics_costco' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${avionics_costco_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(avionics_costco_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${avionics_costco_DIR}/${_extra}")
endforeach()
