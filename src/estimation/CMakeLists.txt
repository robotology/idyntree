# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause


set(IDYNTREE_ESTIMATION_HEADERS include/iDynTree/BerdyHelper.h
                                include/iDynTree/ExternalWrenchesEstimation.h
                                include/iDynTree/ExtWrenchesAndJointTorquesEstimator.h
                                include/iDynTree/SimpleLeggedOdometry.h
                                include/iDynTree/BerdySparseMAPSolver.h
                                include/iDynTree/SchmittTrigger.h
                                include/iDynTree/ContactStateMachine.h
                                include/iDynTree/BipedFootContactClassifier.h
                                include/iDynTree/GravityCompensationHelpers.h
                                include/iDynTree/ExtendedKalmanFilter.h
                                include/iDynTree/AttitudeEstimator.h
                                include/iDynTree/AttitudeMahonyFilter.h
                                include/iDynTree/AttitudeQuaternionEKF.h
                                include/iDynTree/KalmanFilter.h                                )

set(IDYNTREE_ESTIMATION_PRIVATE_INCLUDES include/iDynTree/AttitudeEstimatorUtils.h)

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
                                src/AttitudeQuaternionEKF.cpp
                                src/KalmanFilter.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_ESTIMATION_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_ESTIMATION_HEADERS})

set(libraryname idyntree-estimation)

add_library(${libraryname} ${IDYNTREE_ESTIMATION_SOURCES} ${IDYNTREE_ESTIMATION_HEADERS} ${IDYNTREE_ESTIMATION_PRIVATE_INCLUDES})
add_library(iDynTree::${libraryname} ALIAS ${libraryname})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

target_link_libraries(${libraryname} PUBLIC idyntree-core idyntree-model idyntree-modelio
                                     PRIVATE Eigen3::Eigen)

target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_ESTIMATION_HEADERS})

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

# Create deprecated target idyntree-modelio-urdf 
add_library(idyntree-modelio-urdf INTERFACE)
add_library(iDynTree::idyntree-modelio-urdf ALIAS idyntree-modelio-urdf)
target_link_libraries(idyntree-modelio-urdf INTERFACE idyntree-modelio)
install(TARGETS idyntree-modelio-urdf
        EXPORT iDynTree)
if(CMAKE_VERSION VERSION_GREATER 3.17)
  set_target_properties(idyntree-modelio-urdf
                        PROPERTIES DEPRECATION "Do not use deprecated target iDynTree::idyntree-modelio-urdf, use iDynTree::idyntree-modelio instead.")
endif()

# Install deprecated headers
install(DIRECTORY include/iDynTree/Estimation
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif(BUILD_TESTING)
