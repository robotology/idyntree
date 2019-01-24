# Copyright (C) 2015 Fondazione Istituto Italiano di Tecnologia
#
# Licensed under either the GNU Lesser General Public License v3.0 :
# https://www.gnu.org/licenses/lgpl-3.0.html
# or the GNU Lesser General Public License v2.1 :
# https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
# at your option.

project(iDynTree_Estimation CXX)

set(IDYNTREE_ESTIMATION_HEADERS include/iDynTree/Estimation/BerdyHelper.h
                                include/iDynTree/Estimation/ExternalWrenchesEstimation.h
                                include/iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h
                                include/iDynTree/Estimation/SimpleLeggedOdometry.h
                                include/iDynTree/Estimation/BerdySparseMAPSolver.h
                                include/iDynTree/Estimation/SchmittTrigger.h
                                include/iDynTree/Estimation/ContactStateMachine.h
                                include/iDynTree/Estimation/BipedFootContactClassifier.h
                                include/iDynTree/Estimation/GravityCompensationHelpers.h
                                include/iDynTree/Estimation/ExtendedKalmanFilter.h
                                include/iDynTree/Estimation/AttitudeEstimator.h
                                include/iDynTree/Estimation/AttitudeMahonyFilter.h
                                include/iDynTree/Estimation/AttitudeQuaternionEKF.h)

set(IDYNTREE_ESTIMATION_PRIVATE_INCLUDES include/iDynTree/Estimation/AttitudeEstimatorUtils.h)

set(IDYNTREE_ESTIMATION_SOURCES src/BerdyHelper.cpp
                                src/ExternalWrenchesEstimation.cpp
                                src/ExtWrenchesAndJointTorquesEstimator.cpp
                                src/SimpleLeggedOdometry.cpp
                                src/BerdySparseMAPSolver.cpp
                                src/SchmittTrigger.cpp
                                src/ContactStateMachine.cpp
                                src/BipedFootContactClassifier.cpp
                                src/GravityCompensationHelpers.cpp
                                src/ExtendedKalmanFilter.cpp
                                src/AttitudeEstimator.cpp
                                src/AttitudeEstimatorUtils.cpp
                                src/AttitudeMahonyFilter.cpp
                                src/AttitudeQuaternionEKF.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_ESTIMATION_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_ESTIMATION_HEADERS})

set(libraryname idyntree-estimation)

add_library(${libraryname} ${IDYNTREE_ESTIMATION_SOURCES} ${IDYNTREE_ESTIMATION_HEADERS} ${IDYNTREE_ESTIMATION_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(${libraryname} PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})

target_link_libraries(${libraryname} idyntree-core idyntree-model idyntree-sensors idyntree-modelio-urdf)

# Ensure that build include directories are always included before system ones
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

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
