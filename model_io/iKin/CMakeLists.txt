# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_ModelIO_iKin CXX)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)

set(IDYNTREE_MODELIO_IKIN_HEADERS include/iDynTree/ModelIO/iKin_export.hpp)

set(IDYNTREE_MODELIO_IKIN_SOURCES src/iKin_export.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODELIO_IKIN_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODELIO_IKIN_HEADERS})

# share headers with all iDynTree targets
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)
include_directories(SYSTEM ${IDYNTREE_TREE_INCLUDE_DIRS})

set(libraryname idyntree-modelio-ikin)

add_library(${libraryname} ${IDYNTREE_MODELIO_IKIN_SOURCES} ${IDYNTREE_MODELIO_IKIN_HEADERS})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                 ${orocos_kdl_INCLUDE_DIRS}
                                                 ${iKin_INCLUDE_DIRS}
                                                 ${YARP_INCLUDE_DIRS}
                                          PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core iKin ${orocos_kdl_LIBRARIES})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_IKIN_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/ModelIO)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})