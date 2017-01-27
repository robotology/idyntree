# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_Base CXX)

set(IDYNTREE_CORE_EXP_HEADERS include/iDynTree/Core/Axis.h
                              include/iDynTree/Core/ArticulatedBodyInertia.h
                              include/iDynTree/Core/ClassicalAcc.h
                              include/iDynTree/Core/Direction.h
                              include/iDynTree/Core/EigenHelpers.h
                              include/iDynTree/Core/InertiaNonLinearParametrization.h
                              include/iDynTree/Core/MatrixDynSize.h
                              include/iDynTree/Core/MatrixFixSize.h
                              include/iDynTree/Core/Position.h
                              include/iDynTree/Core/PositionRaw.h
                              include/iDynTree/Core/PositionSemantics.h
                              include/iDynTree/Core/Rotation.h
                              include/iDynTree/Core/RotationRaw.h
                              include/iDynTree/Core/RotationSemantics.h
                              include/iDynTree/Core/RotationalInertiaRaw.h
                              include/iDynTree/Core/SpatialAcc.h
                              include/iDynTree/Core/SpatialForceVector.h
                              include/iDynTree/Core/SpatialInertiaRaw.h
                              include/iDynTree/Core/SpatialInertia.h
                              include/iDynTree/Core/SpatialMomentum.h
                              include/iDynTree/Core/SpatialMotionVector.h
                              include/iDynTree/Core/TestUtils.h
                              include/iDynTree/Core/Transform.h
                              include/iDynTree/Core/TransformDerivative.h
                              include/iDynTree/Core/TransformSemantics.h
                              include/iDynTree/Core/Twist.h
                              include/iDynTree/Core/Utils.h
                              include/iDynTree/Core/VectorFixSize.h
                              include/iDynTree/Core/VectorDynSize.h
                              include/iDynTree/Core/LinearForceVector3.h
                              include/iDynTree/Core/AngularForceVector3.h
                              include/iDynTree/Core/LinearMotionVector3.h
                              include/iDynTree/Core/AngularMotionVector3.h
                              include/iDynTree/Core/Wrench.h
                              include/iDynTree/Core/PrivateUtils.h
                              include/iDynTree/Core/PrivateMotionForceVertorAssociations.h
                              include/iDynTree/Core/PrivatePreProcessorUtils.h
                              include/iDynTree/Core/PrivateSemanticsMacros.h
                              include/iDynTree/Core/GeomVector3.h
                              include/iDynTree/Core/MotionVector3.h
                              include/iDynTree/Core/ForceVector3.h
                              include/iDynTree/Core/SpatialVector.h
                              include/iDynTree/Core/SparseMatrix.h
                              include/iDynTree/Core/Triplets.h)


set(IDYNTREE_CORE_EXP_SOURCES src/Axis.cpp
                              src/ArticulatedBodyInertia.cpp
                              src/ClassicalAcc.cpp
                              src/Direction.cpp
                              src/InertiaNonLinearParametrization.cpp
                              src/MatrixDynSize.cpp
                              src/LinearForceVector3.cpp
                              src/AngularForceVector3.cpp
                              src/LinearMotionVector3.cpp
                              src/AngularMotionVector3.cpp
                              src/ForceVector3.cpp
                              src/MotionVector3.cpp
                              src/GeomVector3.cpp
                              src/Position.cpp
                              src/PositionRaw.cpp
                              src/PositionSemantics.cpp
                              src/Rotation.cpp
                              src/RotationRaw.cpp
                              src/RotationSemantics.cpp
                              src/RotationalInertiaRaw.cpp
                              src/SpatialAcc.cpp
                              src/SpatialForceVector.cpp
                              src/SpatialMomentum.cpp
                              src/SpatialMotionVector.cpp
                              src/SpatialInertiaRaw.cpp
                              src/SpatialInertia.cpp
                              src/TestUtils.cpp
                              src/Transform.cpp
                              src/TransformDerivative.cpp
                              src/TransformSemantics.cpp
                              src/Twist.cpp
                              src/Utils.cpp
                              src/VectorDynSize.cpp
                              src/Wrench.cpp
                              src/PrivateUtils.cpp
                              src/SparseMatrix.cpp
                              src/Triplets.cpp)

SOURCE_GROUP("Source Files" FILES ${IDYNTREE_CORE_EXP_SOURCES})
SOURCE_GROUP("Header Files" FILES ${IDYNTREE_CORE_EXP_HEADERS})
#SOURCE_GROUP("Header Files\\Private" FILES ${IDYNTREE_CORE_EXP_PRIVATE_INCLUDES})

# Check if this does not break existing build
# reason: avoid including with <iDynTree/Core/**> inside .cpp but using directly
# "**" which clearly states the difference between in-library files and external files
include_directories(include/iDynTree/Core)

set(libraryname idyntree-core)

add_library(${libraryname} ${IDYNTREE_CORE_EXP_SOURCES} ${IDYNTREE_CORE_EXP_HEADERS} ${IDYNTREE_CORE_EXP_PRIVATE_INCLUDES})

target_include_directories(${libraryname} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                 "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}>"
                                          PRIVATE ${EIGEN3_INCLUDE_DIR})

set_property(TARGET ${libraryname} PROPERTY PUBLIC_HEADER ${IDYNTREE_CORE_EXP_HEADERS})

if(NOT CMAKE_VERSION VERSION_LESS 2.8.12)
    target_compile_options(${libraryname} PRIVATE ${IDYNTREE_WARNING_FLAGS})
endif()

if(MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
endif()

install(TARGETS ${libraryname}
        EXPORT iDynTree
        COMPONENT runtime
        RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Core
        PRIVATE_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/iDynTree/Core/impl)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS ${libraryname})

get_property(IDYNTREE_COMP_INCLUDE_DIRS TARGET ${libraryname} PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS ${IDYNTREE_COMP_INCLUDE_DIRS})


if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
