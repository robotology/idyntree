# Copyright: (C) 2017 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

#########################################################################
# Enable/disable dependencies
macro(idyntree_handle_dependency package)
  set(singleValueArgs MINIMUM_VERSION)
  set(multiValueArgs COMPONENTS)
  cmake_parse_arguments(IHD "" "${singleValueArgs}" "${multiValueArgs}" ${ARGN})
  if (IHD_COMPONENTS)
    find_package(${package} ${IHD_MINIMUM_VERSION} COMPONENTS ${IHD_COMPONENTS})
  else ()
    find_package(${package} ${IHD_MINIMUM_VERSION})
  endif ()
  string(TOUPPER ${package} PKG)
  option(IDYNTREE_USES_${PKG} "Build the part of iDynTree that depends on package ${package}" ${${package}_FOUND})
  if (IDYNTREE_USES_${PKG})
    if (IHD_COMPONENTS)
      find_package(${package} ${IHD_MINIMUM_VERSION} COMPONENTS ${IHD_COMPONENTS} REQUIRED)
    else ()
      find_package(${package} ${IHD_MINIMUM_VERSION} REQUIRED)
    endif ()
  endif ()
endmacro ()

# Eigen is compulsory (minimum version 3.3)
find_package(Eigen3 REQUIRED 3.3)

# For orocos_kdl we have custom logic, because we want to set it to FALSE by default
option(IDYNTREE_USES_KDL "Build the part of iDynTree that depends on package orocos_kdl" FALSE)
if (IDYNTREE_USES_KDL)
  include(OrocosKDLFindLogic)
  find_package(orocos_kdl REQUIRED)
  find_package(TinyXML    REQUIRED)
endif ()

find_package(ICUB)
option(IDYNTREE_USES_ICUB_MAIN "Build the part of iDynTree that depends on package ICUB" ${ICUB_FOUND})
if (IDYNTREE_USES_ICUB)
  find_package(ICUB REQUIRED)
endif ()

idyntree_handle_dependency(YARP MINIMUM_VERSION 2.3.62)
idyntree_handle_dependency(IPOPT)
idyntree_handle_dependency(Irrlicht)
idyntree_handle_dependency(Qt5 COMPONENTS Qml Quick Widgets)


# If KDL is not used, an external TinyXML is not compulsory
# (because no public headers contain TinyXML includes)
# and so we can use the internal copy of TinyXML, see logic
# in extern/CMakeLists.txt . For this reason we just check
# the system if a local copy of TinyXML exists, and otherwise
# we use the copy in extern/CMakeLists.txt
find_package(TinyXML)
