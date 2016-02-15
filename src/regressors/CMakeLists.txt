# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Regressors CXX)

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

set(libraryname idyntree-regressors)

add_library(${libraryname} ${IDYNTREE_REGRESSORS_SOURCES}     ${IDYNTREE_REGRESSORS_HEADERS}
                                ${IDYNTREE_REGRESSORS_SOURCES_EXP} ${IDYNTREE_REGRESSORS_HEADERS_EXP})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
include_directories(SYSTEM ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core idyntree-sensors idyntree-modelio-urdf-kdl ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES} )

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_REGRESSORS_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/kdl_codyco/regressors)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})

