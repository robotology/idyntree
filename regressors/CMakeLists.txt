# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Regressors CXX)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})

file(GLOB IDYNTREE_REGRESSORS_HDRS
     "include/kdl_codyco/regressors/*.hpp"
 )

# TODO \todo : avoid globbing and explicitly state everithing

file(GLOB  IDYNTREE_REGRESSORS_SRC
    "src/*.cpp"
)


SOURCE_GROUP("Source Files" FILES ${IDYNTREE_REGRESSORS_SRC})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_REGRESSORS_HDRS})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})
include(YCMDefaultDirs)


add_library(idyntree-regressors ${IDYNTREE_REGRESSORS_SRC} ${IDYNTREE_REGRESSORS_HDRS})

target_include_directories(idyntree-regressors PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>")

target_link_libraries(idyntree-regressors ${orocos_kdl_LIBRARIES} idyntree-core)

set_target_properties(idyntree-regressors PROPERTIES
  SOVERSION "${KDL_CODYCO_VERSION_MAJOR}.${KDL_CODYCO_VERSION_MINOR}"
  VERSION "${KDL_CODYCO_VERSION}"
  COMPILE_FLAGS "${CMAKE_CXX_FLAGS_ADD}")

set_property(TARGET idyntree-regressors PROPERTY PUBLIC_HEADER ${IDYNTREE_REGRESSORS_HDRS})

install(TARGETS idyntree-regressors
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco/regressors)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-regressors)

