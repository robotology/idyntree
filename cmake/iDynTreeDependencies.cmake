# Copyright: (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#########################################################################
# Enable/disable dependencies
# DO_NOT_SILENTLY_SEARCH: Do not search for the package to set the default
#                         value of IDYNTREE_USES_<dep> option, but just set
#                         it to OFF 
macro(idyntree_handle_dependency package)
  set(options DO_NOT_SILENTLY_SEARCH)
  set(singleValueArgs MINIMUM_VERSION)
  set(multiValueArgs COMPONENTS)
  cmake_parse_arguments(IHD "${options}" "${singleValueArgs}" "${multiValueArgs}" ${ARGN})
  string(TOUPPER ${package} PKG)
  if (NOT IHD_DO_NOT_SILENTLY_SEARCH)
    if (IHD_COMPONENTS)
      find_package(${package} ${IHD_MINIMUM_VERSION} QUIET COMPONENTS ${IHD_COMPONENTS})
    else ()
      find_package(${package} ${IHD_MINIMUM_VERSION} QUIET)
    endif ()
    set(IDYNTREE_USES_${PKG}_DEFAULT ${${package}_FOUND})
  else ()
    set(IDYNTREE_USES_${PKG}_DEFAULT FALSE)
  endif ()
  option(IDYNTREE_USES_${PKG} "Build the part of iDynTree that depends on package ${package}" ${IDYNTREE_USES_${PKG}_DEFAULT})
  if (IDYNTREE_USES_${PKG})
    if (IHD_COMPONENTS)
      find_package(${package} ${IHD_MINIMUM_VERSION} COMPONENTS ${IHD_COMPONENTS} REQUIRED)
    else ()
      find_package(${package} ${IHD_MINIMUM_VERSION} REQUIRED)
    endif ()
  endif ()
endmacro ()

# Eigen is compulsory (minimum version 3.2.92)
find_package(Eigen3 3.2.92 REQUIRED)
find_package(LibXml2 REQUIRED)

# For orocos_kdl we have custom logic, because we want to set it to FALSE by default
option(IDYNTREE_USES_KDL "Build the part of iDynTree that depends on package orocos_kdl" FALSE)
if (IDYNTREE_USES_KDL)
    # KDL requires a system version of TinyXML
  find_package(TinyXML REQUIRED)
  include(OrocosKDLFindLogic)
  find_package(orocos_kdl REQUIRED)
endif ()

find_package(ICUB QUIET)
option(IDYNTREE_USES_ICUB_MAIN "Build the part of iDynTree that depends on package ICUB" ${ICUB_FOUND})
if (IDYNTREE_USES_ICUB)
  find_package(ICUB REQUIRED)
endif ()

idyntree_handle_dependency(YARP)
set(YARP_REQUIRED_VERSION 2.3.62)
if(IDYNTREE_USES_YARP AND YARP_FOUND)
  if(${YARP_VERSION} VERSION_LESS ${YARP_REQUIRED_VERSION})
    message(FATAL_ERROR "YARP version ${YARP_VERSION} not sufficient, at least version ${YARP_REQUIRED_VERSION} is required.")
  endif()
endif()

idyntree_handle_dependency(IPOPT)
idyntree_handle_dependency(Irrlicht)
idyntree_handle_dependency(Qt5 COMPONENTS Qml Quick Widgets)
idyntree_handle_dependency(OsqpEigen)
idyntree_handle_dependency(ALGLIB MINIMUM_VERSION 3.14.0)
idyntree_handle_dependency(WORHP)
# Workaround for https://github.com/robotology/idyntree/issues/599
idyntree_handle_dependency(ASSIMP DO_NOT_SILENTLY_SEARCH)
