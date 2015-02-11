# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Core CXX)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

#local headers
include_directories(include/kdl_codyco)

file(GLOB IDYNTREE_CORE_HEADERS
     "include/kdl_codyco/*.hpp"
 )

file (GLOB IDYNTREE_CORE_PRIVATE_INCLUDES
    "include/kdl_codyco/config.h"
)

file(GLOB IDYNTREE_CORE_SOURCES
    "src/*.cpp"
)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_CORE_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_CORE_HEADERS})
SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_CORE_PRIVATE_INCLUDES})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")

#include system headers
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS}
                            ${EIGEN3_INCLUDE_DIR})

add_library(idyntree-core ${IDYNTREE_CORE_SOURCES} ${IDYNTREE_CORE_HEADERS} ${IDYNTREE_CORE_PRIVATE_INCLUDES})

target_include_directories(idyntree-core PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                                ${EIGEN3_INCLUDE_DIR})

target_link_libraries(idyntree-core ${orocos_kdl_LIBRARIES})

set_property(TARGET idyntree-core PROPERTY PUBLIC_HEADER ${IDYNTREE_CORE_HEADERS})

install(TARGETS idyntree-core
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-core)
