# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_Estimation_KDL CXX)

set(IDYNTREE_ESTIMATION_KDL_HEADERS include/iDynTree/Estimation/simpleLeggedOdometryKDL.h
                                include/iDynTree/Estimation/robotStatus.h)

set(IDYNTREE_ESTIMATION_KDL_PRIVATE_INCLUDES )

set(IDYNTREE_ESTIMATION_KDL_SOURCES src/simpleLeggedOdometry.cpp
                                src/robotStatus.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_ESTIMATION_KDL_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_ESTIMATION_KDL_HEADERS})

# Legacy library, remove deprecated warnings when compiling
idyntree_disable_deprecation_warnings()


set(libraryname idyntree-estimation-kdl)

add_library(${libraryname} ${IDYNTREE_ESTIMATION_KDL_SOURCES} ${IDYNTREE_ESTIMATION_KDL_HEADERS} ${IDYNTREE_ESTIMATION_KDL_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS})

target_link_libraries(${libraryname} idyntree-core idyntree-kdl idyntree-sensors idyntree-modelio-urdf-kdl idyntree-yarp-kdl ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES})

if((${CMAKE_CXX_COMPILER_ID} MATCHES "GNU") OR (${CMAKE_CXX_COMPILER_ID} MATCHES "Clang"))
    target_compile_options(${libraryname} PRIVATE "-Wno-deprecated")
endif()

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Estimation
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Estimation/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
