# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Estimation CXX)

set(IDYNTREE_ESTIMATION_HEADERS include/iDynTree/Estimation/ExternalWrenchesEstimation.h
                                include/iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h)

set(IDYNTREE_ESTIMATION_PRIVATE_INCLUDES )

set(IDYNTREE_ESTIMATION_SOURCES src/ExternalWrenchesEstimation.cpp
                                src/ExtWrenchesAndJointTorquesEstimator.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_ESTIMATION_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_ESTIMATION_HEADERS})

set(libraryname idyntree-estimation)

add_library(${libraryname} ${IDYNTREE_ESTIMATION_SOURCES} ${IDYNTREE_ESTIMATION_HEADERS} ${IDYNTREE_ESTIMATION_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include;${IDYNTREE_TREE_INCLUDE_DIRS}>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core idyntree-model idyntree-sensors idyntree-modelio-urdf)

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

set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
