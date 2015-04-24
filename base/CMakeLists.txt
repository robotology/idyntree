# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Base CXX)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

#local headers
include_directories(include/iDynTree/Base)

file(GLOB IDYNTREE_BASE_HEADERS
     "include/iDynTree/Base/*.h"
 )

#file (GLOB IDYNTREE_BASE_PRIVATE_INCLUDES)

file(GLOB IDYNTREE_BASE_SOURCES
    "src/*.cpp"
)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_BASE_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_BASE_HEADERS})
#SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_BASE_PRIVATE_INCLUDES})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")

add_library(idyntree-base ${IDYNTREE_BASE_SOURCES} ${IDYNTREE_BASE_HEADERS} ${IDYNTREE_BASE_PRIVATE_INCLUDES})

target_include_directories(idyntree-base PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                         PRIVATE ${EIGEN3_INCLUDE_DIR})

set_property(TARGET idyntree-base PROPERTY PUBLIC_HEADER ${IDYNTREE_BASE_HEADERS})

install(TARGETS idyntree-base
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Base
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Base/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-base)
