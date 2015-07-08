# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Regressors CXX)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

include_directories(include/kdl_codyco/regressors)
include_directories(include)

file(GLOB IDYNTREE_REGRESSORS_HEADERS
     "include/kdl_codyco/regressors/*.hpp"
 )

set(IDYNTREE_REGRESSOR_HEADERS_EXP include/iDynTree/Regressors/DynamicsRegressorParameters.h)

# TODO \todo : avoid globbing and explicitly state everithing

file(GLOB IDYNTREE_REGRESSORS_SOURCES
    "src/*.cpp"
)

set(IDYNTREE_REGRESSOR_SOURCES_EXP DynamicsRegressorParameters.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_REGRESSORS_SOURCES} ${IDYNTREE_REGRESSORS_SOURCES_EXP})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_REGRESSORS_HEADERS} ${IDYNTREE_REGRESSORS_HEADERS_EXP})

#include other iDynTree headers as system
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(SYSTEM ${IDYNTREE_TREE_INCLUDE_DIRS})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS}
                           ${EIGEN3_INCLUDE_DIR})

add_library(idyntree-regressors ${IDYNTREE_REGRESSORS_SOURCES}     ${IDYNTREE_REGRESSORS_HEADERS}
                                ${IDYNTREE_REGRESSORS_SOURCES_EXP} ${IDYNTREE_REGRESSORS_HEADERS_EXP})

target_include_directories(idyntree-regressors PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(idyntree-regressors ${orocos_kdl_LIBRARIES} idyntree-core idyntree-sensors idyntree-modelio-urdf ${TinyXML_LIBRARIES} )

set_target_properties(idyntree-regressors PROPERTIES
  SOVERSION "${${VARS_PREFIX}_VERSION}"
  VERSION "${${VARS_PREFIX}_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}")

set_property(TARGET idyntree-regressors idyntree-modelio-urdf PROPERTY PUBLIC_HEADER ${IDYNTREE_REGRESSORS_HEADERS})

install(TARGETS idyntree-regressors
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco/regressors)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-regressors)

