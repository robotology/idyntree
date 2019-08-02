# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

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

# Legacy library, remove deprecated warnings when compiling
idyntree_disable_deprecation_warnings()

set(libraryname idyntree-kdl)

add_library(${libraryname} ${IDYNTREE_KDL_SOURCES} ${IDYNTREE_KDL_HEADERS} ${IDYNTREE_KDL_PRIVATE_INCLUDES})

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")

if((${CMAKE_CXX_COMPILER_ID} MATCHES "GNU") OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
    target_compile_options(${libraryname} PRIVATE "-Wno-deprecated")
endif()

target_link_libraries(${libraryname} idyntree-core idyntree-model ${orocos_kdl_LIBRARIES})

target_include_directories(${libraryname} INTERFACE ${EIGEN3_INCLUDE_DIR}
                                                    ${orocos_kdl_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR}
                           ${orocos_kdl_INCLUDE_DIRS})

# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
target_include_directories(${libraryname} BEFORE PRIVATE ${IDYNTREE_TREE_INCLUDE_DIRS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_KDL_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
