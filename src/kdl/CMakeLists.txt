# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Core CXX)

file(GLOB IDYNTREE_KDL_HEADERS
     "include/kdl_codyco/*.hpp"
 )

file (GLOB IDYNTREE_KDL_PRIVATE_INCLUDES
    "include/kdl_codyco/config.h"
)

file(GLOB IDYNTREE_KDL_SOURCES
    "src/*.cpp"
)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_KDL_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_KDL_HEADERS})
SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_KDL_PRIVATE_INCLUDES})

set(libraryname idyntree-kdl)

add_library(${libraryname} ${IDYNTREE_KDL_SOURCES} ${IDYNTREE_KDL_HEADERS} ${IDYNTREE_KDL_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(${libraryname} PUBLIC ${EIGEN3_INCLUDE_DIR}
                                                 ${orocos_kdl_INCLUDE_DIRS})

target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-model ${orocos_kdl_LIBRARIES})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_KDL_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})

