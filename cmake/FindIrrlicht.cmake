#.rst:
# FindAsio
# -----------
#
# Find the Irrlicht library.
#
# IMPORTED Targets
# ^^^^^^^^^^^^^^^^
#
# This module defines the following :prop_tgt:`IMPORTED` targets if
# Irrlicht has been found::
#
#   Irrlicht::Irrlicht
#
# Result Variables
# ^^^^^^^^^^^^^^^^
#
# This module defines the following variables::
#
#   Irrlicht_FOUND                - System has Irrlicht
#

#=============================================================================
# Copyright Fondazione Istituto Italiano di Tecnologia
#   Authors: Silvio Traversaro <silvio.traversaro@iit.it>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

find_package(Irrlicht QUIET NO_MODULE)
if(Irrlicht_FOUND)
  # Some Irrlicht-package in vcpkg only define Irrlicht, not Irrlicht::Irrlicht 
  if(NOT TARGET Irrlicht::Irrlicht) 
    if(TARGET Irrlicht)
      add_library(Irrlicht::Irrlicht INTERFACE IMPORTED)
      set_target_properties(Irrlicht::Irrlicht PROPERTIES INTERFACE_LINK_LIBRARIES Irrlicht)
    endif()
  endif()
  find_package_handle_standard_args(Irrlicht CONFIG_MODE)
  return()
endif()

find_path(Irrlicht_INCLUDE_DIR irrlicht.h
          PATH_SUFFIXES irrlicht)
mark_as_advanced(Irrlicht_INCLUDE_DIR)
find_library(Irrlicht_LIBRARY Irrlicht libIrrlicht)
mark_as_advanced(Irrlicht_LIBRARY)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Irrlicht
                                  FOUND_VAR Irrlicht_FOUND
                                  REQUIRED_VARS Irrlicht_INCLUDE_DIR Irrlicht_LIBRARY)

if(Irrlicht_FOUND AND NOT TARGET Irrlicht::Irrlicht)
  add_library(Irrlicht::Irrlicht UNKNOWN IMPORTED)
  set_target_properties(Irrlicht::Irrlicht PROPERTIES
                        INTERFACE_INCLUDE_DIRECTORIES "${Irrlicht_INCLUDE_DIR}")
  set_property(TARGET Irrlicht::Irrlicht APPEND PROPERTY
               IMPORTED_LOCATION "${Irrlicht_LIBRARY}")
endif()
