# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_ModelIO_URDF CXX)

find_package(TinyXML)

if( IDYNTREE_USES_INTERNAL_URDFDOM )
    # urdfdom not available in the system, using the internal version
    add_definitions(-DURDF_USE_PLAIN_POINTERS)
    add_subdirectory(external/urdfdom)
    set(urdfdom_LIBRARIES iDynTree_priv_urdfdom_model)
elseif()
    find_package(urdfdom_headers)
    find_package(urdfdom)
endif()

if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif(MSVC)


#local headers
include_directories(include/kdl_format_io)

set(IDYNTREE_MODELIO_URDF_HEADERS include/kdl_format_io/urdf_compatibility.h
                                  include/kdl_format_io/urdf_export.hpp
                                  include/kdl_format_io/urdf_import.hpp
                                  include/kdl_format_io/urdf_sensor_import.hpp
                                  include/kdl_format_io/config.h)

set(IDYNTREE_MODELIO_URDF_SOURCES src/urdf_export.cpp
                                  src/urdf_import.cpp
                                  src/urdf_sensor_import.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODELIO_URDF_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODELIO_URDF_HEADERS})

get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

message(STATUS "IDYNTREE_TREE_INCLUDE_DIRS : ${IDYNTREE_TREE_INCLUDE_DIRS}")
include_directories(SYSTEM ${IDYNTREE_TREE_INCLUDE_DIRS})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")

set(libraryname idyntree-modelio-urdf)

add_library(${libraryname} ${IDYNTREE_MODELIO_URDF_SOURCES} ${IDYNTREE_MODELIO_URDF_HEADERS})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                                 ${orocos_kdl_INCLUDE_DIRS})

target_link_libraries(${libraryname} idyntree-core ${orocos_kdl_LIBRARIES} ${urdfdom_LIBRARIES} ${console_bridge_LIBRARIES} ${TinyXML_LIBRARIES})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_URDF_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_format_io)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})