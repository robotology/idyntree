# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Sensors CXX)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

#local headers
include_directories(include/iDynTree/Sensors)

#include system headers
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR})
                            
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
include_directories(SYSTEM ${IDYNTREE_TREE_INCLUDE_DIRS})

set(IDYNTREE_SENSORS_HEADERS include/iDynTree/Sensors/Sensors.hpp
                             include/iDynTree/Sensors/SixAxisFTSensor.hpp)

#file (GLOB IDYNTREE_BASE_PRIVATE_INCLUDES)

set(IDYNTREE_SENSORS_SOURCES src/Sensors.cpp
                             src/SixAxisFTSensor.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_SENSORS_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_SENSORS_HEADERS})
#SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_SENSORS_PRIVATE_INCLUDES})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")

set(libraryname idyntree-sensors)

add_library(${libraryname} ${IDYNTREE_SENSORS_SOURCES} ${IDYNTREE_SENSORS_HEADERS} ${IDYNTREE_SENSORS_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                         PRIVATE ${EIGEN3_INCLUDE_DIR})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_SENSORS_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Sensors
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Sensors/Impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
