# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Estimation CXX)

set(IDYNTREE_ESTIMATION_HEADERS include/iDynTree/Estimation/simpleLeggedOdometry.h
                                include/iDynTree/Estimation/robotStatus.h)

set(IDYNTREE_ESTIMATION_PRIVATE_INCLUDES )

set(IDYNTREE_ESTIMATION_SOURCES src/simpleLeggedOdometry.cpp
                                src/robotStatus.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_ESTIMATION_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_ESTIMATION_HEADERS})

set(libraryname idyntree-estimation)

add_library(${libraryname} ${IDYNTREE_ESTIMATION_SOURCES} ${IDYNTREE_ESTIMATION_HEADERS} ${IDYNTREE_ESTIMATION_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(${libraryname} SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR}
                                                         ${orocos_kdl_INCLUDE_DIRS})

target_link_libraries(${libraryname} idyntree-core idyntree-kdl idyntree-sensors idyntree-modelio-urdf idyntree-yarp ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES})


set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_ESTIMATION_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Estimation
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Estimation/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})
