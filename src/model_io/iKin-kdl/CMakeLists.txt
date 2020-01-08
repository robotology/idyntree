# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_ModelIO_iKin_KDL CXX)

set(IDYNTREE_MODELIO_IKIN_KDL_HEADERS include/iDynTree/ModelIO/iKin_export.hpp)

set(IDYNTREE_MODELIO_IKIN_KDL_SOURCES src/iKin_export.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_MODELIO_IKIN_KDL_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_MODELIO_IKIN_KDL_HEADERS})

set(libraryname idyntree-modelio-ikin-kdl)

add_library(${libraryname} ${IDYNTREE_MODELIO_IKIN_KDL_SOURCES} ${IDYNTREE_MODELIO_IKIN_KDL_HEADERS})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                                 ${orocos_kdl_INCLUDE_DIRS}
                                                 ${iKin_INCLUDE_DIRS}
                                                 ${YARP_INCLUDE_DIRS}
                                          PRIVATE ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core iKin YARP::YARP_os YARP::YARP_math ${orocos_kdl_LIBRARIES})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_MODELIO_IKIN_KDL_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/ModelIO)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
