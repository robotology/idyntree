# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Estimation_KDL CXX)

set(IDYNTREE_ESTIMATION_KDL_HEADERS include/iDynTree/Estimation/simpleLeggedOdometry.h
                                include/iDynTree/Estimation/robotStatus.h)

set(IDYNTREE_ESTIMATION_KDL_PRIVATE_INCLUDES )

set(IDYNTREE_ESTIMATION_KDL_SOURCES src/simpleLeggedOdometry.cpp
                                src/robotStatus.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_ESTIMATION_KDL_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_ESTIMATION_KDL_HEADERS})

set(libraryname idyntree-estimation-kdl)

add_library(${libraryname} ${IDYNTREE_ESTIMATION_KDL_SOURCES} ${IDYNTREE_ESTIMATION_KDL_HEADERS} ${IDYNTREE_ESTIMATION_KDL_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${IDYNTREE_TREE_INCLUDE_DIRS}>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS})

target_link_libraries(${libraryname} idyntree-core idyntree-kdl idyntree-sensors idyntree-modelio-urdf idyntree-yarp-kdl ${orocos_kdl_LIBRARIES} ${TinyXML_LIBRARIES})


set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_ESTIMATION_KDL_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Estimation
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Estimation/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
